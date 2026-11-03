#include "robosim/simulation.hpp"

#include <algorithm>
#include <cmath>

namespace robosim {
namespace {
Json diagnostic_json(const Diagnostic& d) {
    return {{"level", d.level}, {"code", d.code}, {"message", d.message}, {"time", d.time}};
}
Json sample_json(const Sample& s) {
    return {{"time", s.time}, {"target", s.target}, {"position", s.position}, {"velocity", s.velocity},
        {"command", s.command}, {"measured_position", s.measured_position},
        {"fault", s.fault ? Json(*s.fault) : Json(nullptr)}, {"safe", s.safe},
        {"requested_command", s.requested_command}, {"emergency_stop", s.emergency_stop},
        {"measurement_age", s.measurement_age}, {"measurement_valid", s.measurement_valid},
        {"position_limit_violation", s.position_limit_violation}, {"velocity_limit_violation", s.velocity_limit_violation},
        {"actuator_limit_violation", s.actuator_limit_violation}, {"actuator_velocity_violation", s.actuator_velocity_violation},
        {"actuator_acceleration_violation", s.actuator_acceleration_violation}, {"acceleration_limit_violation", s.acceleration_limit_violation},
        {"actuator_velocity", s.actuator_velocity}, {"actuator_acceleration", s.actuator_acceleration},
        {"feedback_delayed", s.feedback_delayed}, {"sensor_unchanged", s.sensor_unchanged}};
}
} // namespace

Json SimulationResult::metrics() const {
    Json out = {{"scenario", scenario}, {"passed", passed}, {"reason", reason}, {"samples", samples.size()}};
    if (samples.empty()) return out;
    const double count = static_cast<double>(samples.size());
    const double dt = samples.size() > 1 ? samples[1].time - samples[0].time : 0.0;
    const double target = samples.front().target, initial = samples.front().position;
    const double band = 0.02 * std::max(std::abs(target - initial), 1.0);
    const std::size_t tail = std::max<std::size_t>(1, samples.size() / 10);
    double max_error = 0.0, sum_error = 0.0, squared = 0.0, tail_error = 0.0;
    double overshoot = 0.0, max_command = 0.0, effort = 0.0, max_velocity = 0.0, age = 0.0;
    std::size_t saturation = 0, position_limits = 0, velocity_limits = 0, command_limits = 0;
    std::size_t actuator_velocity_limits = 0, actuator_acceleration_limits = 0, acceleration_limits = 0;
    std::size_t faults = 0, invalid = 0, delayed = 0, unchanged = 0;
    Json safe_time = nullptr, rise_time = nullptr, settling_time = nullptr;
    bool emergency = false;
    std::optional<std::size_t> last_outside;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto& s = samples[i];
        const double error = std::abs(s.target - s.position);
        max_error = std::max(max_error, error); sum_error += error; squared += error * error;
        if (i >= samples.size() - tail) tail_error += error;
        if (error > band) last_outside = i;
        overshoot = std::max(overshoot, target >= initial ? s.position - target : target - s.position);
        if (rise_time.is_null() && std::abs(s.position) >= 0.9 * std::abs(target)) rise_time = s.time;
        max_command = std::max(max_command, std::abs(s.command)); effort += std::abs(s.command);
        max_velocity = std::max(max_velocity, std::abs(s.velocity)); age += s.measurement_age;
        saturation += std::abs(s.command) >= 0.999999 * std::max(std::abs(s.requested_command), 1.0) && std::abs(s.command) > 0;
        position_limits += s.position_limit_violation; velocity_limits += s.velocity_limit_violation;
        command_limits += s.actuator_limit_violation; actuator_velocity_limits += s.actuator_velocity_violation;
        actuator_acceleration_limits += s.actuator_acceleration_violation; acceleration_limits += s.acceleration_limit_violation;
        faults += s.fault.has_value(); invalid += !s.measurement_valid;
        delayed += s.feedback_delayed; unchanged += s.sensor_unchanged;
        if (s.safe && safe_time.is_null()) safe_time = s.time;
        emergency = emergency || s.emergency_stop;
    }
    if (!last_outside) settling_time = samples.front().time;
    else if (*last_outside + 1 < samples.size()) settling_time = samples[*last_outside + 1].time;
    Json latency = nullptr;
    if (!events.empty()) {
        std::optional<double> detection, safety;
        for (const auto& d : diagnostics) {
            auto& destination = d.code == "SAFE_STATE" ? safety : detection;
            if (!destination || d.time < *destination) destination = d.time;
        }
        if (!detection) detection = safety;
        if (detection) latency = std::max(0.0, *detection - events.front().time);
    }
    out.update({{"duration", samples.back().time}, {"max_error", max_error}, {"peak_error", max_error},
        {"rmse", std::sqrt(squared / count)}, {"final_error", std::abs(samples.back().target - samples.back().position)},
        {"steady_state_error", tail_error / static_cast<double>(tail)}, {"mean_absolute_error", sum_error / count},
        {"integral_absolute_error", sum_error * dt}, {"integral_squared_error", squared * dt},
        {"overshoot", overshoot}, {"settling_time", settling_time}, {"rise_time", rise_time},
        {"max_command", max_command}, {"control_effort", effort * dt}, {"saturation_fraction", static_cast<double>(saturation) / count},
        {"max_velocity", max_velocity}, {"position_limit_violations", position_limits}, {"velocity_limit_violations", velocity_limits},
        {"command_limit_violations", command_limits}, {"actuator_velocity_violations", actuator_velocity_limits},
        {"actuator_acceleration_violations", actuator_acceleration_limits}, {"acceleration_limit_violations", acceleration_limits},
        {"mean_measurement_age", age / count}, {"safe_state_time", safe_time}, {"fault_count", faults},
        {"diagnostic_count", diagnostics.size() + events.size()}, {"safe_state", !safe_time.is_null()},
        {"emergency_stop", emergency}, {"invalid_readings", invalid}, {"fault_detection_latency", latency},
        {"feedback_delay_diagnostics", delayed}, {"sensor_unchanged_diagnostics", unchanged}});
    return out;
}

Json SimulationResult::to_json() const {
    Json out = {{"metrics", metrics()}, {"samples", Json::array()}, {"diagnostics", Json::array()}, {"events", Json::array()}};
    for (const auto& s : samples) out["samples"].push_back(sample_json(s));
    for (const auto& d : diagnostics) out["diagnostics"].push_back(diagnostic_json(d));
    for (const auto& d : events) out["events"].push_back(diagnostic_json(d));
    return out;
}
} // namespace robosim
