#include "robosim/simulation.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace robosim {
SimulationResult simulate(Json config) {
    config = normalize_scenario(std::move(config));
    Plant plant(config["plant"]);
    PID pid(config["pid"]);
    Actuator actuator(config["actuator"]);
    Sensor sensor(config["sensor"], static_cast<unsigned>(config["seed"].get<double>()));
    SafetyMonitor safety(config["safety"]);
    SimulationResult result;
    result.scenario = config["name"].get<std::string>();
    const double dt = config["dt"].get<double>(), duration = config["duration"].get<double>();
    const auto steps = static_cast<std::size_t>(std::nearbyint(duration / dt));
    result.samples.reserve(steps + 1);
    const auto& diag = config["diagnostics"];
    const double delay_limit = diag.value("feedback_latency_limit", diag.value("max_feedback_delay", dt));
    const int unchanged_limit = diag.value("unchanged_samples", 3);
    std::optional<double> previous_measurement;
    int unchanged_count = 0;
    std::set<std::string> seen;
    const bool configured_dropout = sensor.dropout, configured_invalid = sensor.invalid;
    const bool configured_interruption = sensor.communication_interruption;
    const double configured_drift = sensor.drift_per_second;
    const auto configured_stuck = sensor.stuck_value;
    for (std::size_t index = 0; index <= steps; ++index) {
        const double time = static_cast<double>(index) * dt, target = target_at(config, time);
        std::set<std::string> names;
        double drift = configured_drift;
        for (const auto& fault : config["faults"]) {
            if (fault.value("start", 0.0) <= time && time <= fault.value("end", duration)) {
                const auto type = fault.at("type").get<std::string>();
                names.insert(type);
                if (type == "sensor_drift") drift = fault.value("rate", 0.1);
            }
        }
        if (configured_dropout) names.insert("sensor_dropout");
        sensor.dropout = names.count("sensor_dropout") != 0;
        sensor.communication_interruption = configured_interruption || names.count("communication_interruption");
        sensor.invalid = configured_invalid || names.count("sensor_invalid") || names.count("invalid_reading");
        sensor.drift_per_second = drift;
        if (names.count("sensor_stuck")) {
            if (!sensor.stuck_value) sensor.stuck_value = plant.position;
        } else sensor.stuck_value = configured_stuck;
        const bool emergency = names.count("emergency_stop") != 0;
        actuator.stuck = names.count("actuator_stuck") != 0;
        if (emergency) { safety.safe = true; safety.reason = "emergency stop requested"; }
        if (names.count("sensor_stuck")) { safety.safe = true; safety.reason = "sensor stuck fault detected"; }
        for (const auto& name : names) {
            if (seen.insert(name).second) {
                auto code = name;
                std::transform(code.begin(), code.end(), code.begin(), [](unsigned char c) {
                    return static_cast<char>(std::toupper(c));
                });
                result.events.push_back({"WARNING", code, "fault active: " + name, time});
            }
        }
        const auto reading = sensor.read(plant.position, time);
        const bool valid = reading.first.has_value();
        const double measured = reading.first.value_or(plant.position);
        const double request = safety.safe ? 0.0 : pid.update(target, measured, dt);
        const double force = actuator.apply(request, dt);
        const bool safe = safety.check(target - plant.position, force, valid);
        plant.step(safe ? 0.0 : force, dt);
        if (!std::isfinite(plant.position) || !std::isfinite(plant.velocity) ||
            !std::isfinite(request) || !std::isfinite(force) || !std::isfinite(measured) ||
            !std::isfinite(actuator.velocity) || !std::isfinite(actuator.acceleration))
            throw std::runtime_error("non-finite simulation state; reduce dt or check parameters");
        const bool delayed = valid && reading.second > delay_limit && std::abs(request) > 1e-9;
        if (valid && previous_measurement && std::abs(measured - *previous_measurement) <= 1e-12 && std::abs(request) > 1e-9)
            ++unchanged_count;
        else unchanged_count = 0;
        const bool unchanged = unchanged_count >= unchanged_limit;
        if (delayed && seen.insert("FEEDBACK_DELAY").second)
            result.diagnostics.push_back({"WARNING", "FEEDBACK_DELAY", "feedback measurement is delayed during commanded motion", time});
        if (unchanged && seen.insert("SENSOR_UNCHANGED").second)
            result.diagnostics.push_back({"WARNING", "SENSOR_UNCHANGED", "sensor value is unchanged during commanded motion", time});
        previous_measurement = valid ? std::optional<double>(measured) : std::nullopt;
        Sample sample;
        sample.time = time; sample.target = target; sample.position = plant.position;
        sample.velocity = plant.velocity; sample.command = force; sample.measured_position = measured;
        sample.requested_command = request; sample.safe = safe; sample.emergency_stop = emergency;
        sample.measurement_age = reading.second; sample.measurement_valid = valid;
        if (!names.empty()) {
            std::string fault;
            for (const auto& name : names) { if (!fault.empty()) fault += ','; fault += name; }
            sample.fault = fault;
        } else if (!valid) sample.fault = sensor.invalid ? "sensor_invalid" : "sensor_dropout";
        sample.position_limit_violation = std::abs(plant.position) >= plant.limit;
        sample.velocity_limit_violation = plant.velocity_limit && std::abs(plant.velocity) >= *plant.velocity_limit;
        sample.actuator_limit_violation = std::abs(request) > actuator.maximum;
        sample.actuator_velocity_violation = actuator.velocity_violation;
        sample.actuator_acceleration_violation = actuator.acceleration_violation;
        sample.acceleration_limit_violation = plant.acceleration_limit_violation;
        sample.actuator_velocity = actuator.velocity; sample.actuator_acceleration = actuator.acceleration;
        sample.feedback_delayed = delayed; sample.sensor_unchanged = unchanged;
        result.samples.push_back(sample);
        if (safe && seen.insert("SAFE_STATE").second)
            result.diagnostics.push_back({"ERROR", "SAFE_STATE", safety.reason, time});
    }
    const auto& acceptance = config["acceptance"];
    const bool expected_safe = acceptance.value("expected_safe", false);
    const bool expected_stop = acceptance.value("expected_emergency_stop", false);
    const bool stopped = std::any_of(result.samples.begin(), result.samples.end(), [](const Sample& s) { return s.emergency_stop; });
    const double error = std::abs(result.samples.back().target - result.samples.back().position);
    result.passed = safety.safe == expected_safe && (!expected_stop || stopped) &&
        (expected_safe || error <= acceptance.value("final_error", 0.1));
    result.reason = result.passed ? "pass" : (safety.reason.empty() ? "acceptance threshold failed" : safety.reason);
    return result;
}
SimulationResult run_file(const Path& path) { return simulate(load_scenario(path)); }
} // namespace robosim
