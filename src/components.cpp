#include "robosim/simulation.hpp"

#include <algorithm>
#include <cmath>

namespace robosim {
namespace {
std::optional<double> optional(const Json& config, const char* key, const char* alias = nullptr) {
    if (config.contains(key) && !config[key].is_null()) return config[key].get<double>();
    if (alias && config.contains(alias) && !config[alias].is_null()) return config[alias].get<double>();
    return std::nullopt;
}
double bound(double value, double maximum) { return std::clamp(value, -maximum, maximum); }
} // namespace

PID::PID(const Json& c) : kp(c.value("kp", 4.0)), ki(c.value("ki", 0.2)), kd(c.value("kd", 0.5)),
    output_min(c.value("output_min", -10.0)), output_max(c.value("output_max", 10.0)) {}
double PID::update(double target, double measured, double dt) {
    const double error = target - measured;
    const double candidate = integral + error * dt;
    const double derivative = previous_error ? (error - *previous_error) / dt : 0.0;
    const double output = kp * error + ki * candidate + kd * derivative;
    const double clamped = std::clamp(output, output_min, output_max);
    if (output == clamped) integral = candidate;
    previous_error = error;
    return clamped;
}
void PID::reset() { integral = 0.0; previous_error.reset(); }

Plant::Plant(const Json& c) : position(c.value("position", 0.0)), velocity(c.value("velocity", 0.0)),
    mass(c.value("mass", 1.0)), damping(c.value("damping", 0.2)), limit(c.value("limit", 10.0)),
    velocity_limit(optional(c, "velocity_limit", "max_velocity")), acceleration_limit(optional(c, "acceleration_limit")) {}
void Plant::step(double force, double dt) {
    acceleration = (force - damping * velocity) / mass;
    acceleration_limit_violation = acceleration_limit && std::abs(acceleration) > *acceleration_limit;
    if (acceleration_limit) acceleration = bound(acceleration, *acceleration_limit);
    velocity += acceleration * dt;
    if (velocity_limit) velocity = bound(velocity, *velocity_limit);
    position = bound(position + velocity * dt, limit);
    if (std::abs(position) >= limit) velocity = 0.0;
}

Actuator::Actuator(const Json& c) : maximum(c.value("maximum", 10.0)),
    slew_rate(optional(c, "slew_rate")), max_velocity(optional(c, "max_velocity", "velocity_limit")),
    max_acceleration(optional(c, "max_acceleration", "acceleration_limit")) {}
double Actuator::apply(double request, double dt) {
    velocity_violation = false; acceleration_violation = false;
    if (stuck) { velocity = 0.0; acceleration = 0.0; return output; }
    double desired = bound(request, maximum);
    if (slew_rate) desired = output + bound(desired - output, *slew_rate * dt);
    double requested_velocity = (desired - output) / dt;
    if (max_velocity && std::abs(requested_velocity) > *max_velocity) {
        velocity_violation = true; requested_velocity = bound(requested_velocity, *max_velocity);
    }
    double requested_acceleration = (requested_velocity - velocity) / dt;
    if (max_acceleration && std::abs(requested_acceleration) > *max_acceleration) {
        acceleration_violation = true; requested_acceleration = bound(requested_acceleration, *max_acceleration);
    }
    acceleration = requested_acceleration;
    velocity += acceleration * dt;
    if (max_velocity) velocity = bound(velocity, *max_velocity);
    output = bound(output + velocity * dt, maximum);
    return output;
}

Sensor::Sensor(const Json& c, unsigned seed) : dropout(c.value("dropout", false)),
    communication_interruption(c.value("communication_interruption", false)), invalid(c.value("invalid", false)),
    drift_per_second(c.value("drift_per_second", 0.0)), stuck_value(optional(c, "stuck_value")),
    noise_std_(c.value("noise_std", 0.0)), bias_(c.value("bias", 0.0)), latency_(c.value("latency", 0.0)),
    quantization_(optional(c, "quantization_step", "quantization")), valid_min_(optional(c, "valid_min")),
    valid_max_(optional(c, "valid_max")), rng_(seed) {}

double Sensor::gaussian() {
    // Explicit Box-Muller avoids implementation-specific std::normal_distribution sequences.
    constexpr double scale = 4294967296.0;
    const double u1 = (static_cast<double>(rng_()) + 0.5) / scale;
    const double u2 = (static_cast<double>(rng_()) + 0.5) / scale;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(6.2831853071795864769 * u2);
}
std::pair<std::optional<double>, double> Sensor::read(double position, double time) {
    if (dropout || communication_interruption || invalid) return {std::nullopt, 0.0};
    history_.emplace_back(time, position);
    const double source_time = time - latency_;
    const auto after = std::upper_bound(history_.begin(), history_.end(), source_time,
        [](double timestamp, const auto& entry) { return timestamp < entry.first; });
    double delayed = after == history_.begin() ? position : std::prev(after)->second;
    if (stuck_value) delayed = *stuck_value;
    double measured = delayed + bias_ + drift_per_second * time + (noise_std_ != 0.0 ? gaussian() * noise_std_ : 0.0);
    if (quantization_ && *quantization_ > 0) measured = std::nearbyint(measured / *quantization_) * *quantization_;
    if ((valid_min_ && measured < *valid_min_) || (valid_max_ && measured > *valid_max_))
        return {std::nullopt, latency_};
    return {measured, latency_};
}

SafetyMonitor::SafetyMonitor(const Json& c) : error_limit(c.value("error_limit", 5.0)),
    command_limit(c.value("command_limit", 10.0)), dropout_limit(c.value("dropout_limit", 3)) {}
bool SafetyMonitor::check(double error, double command, bool measurement_valid) {
    dropouts = measurement_valid ? 0 : dropouts + 1;
    if (std::abs(error) > error_limit) { safe = true; reason = "position error limit exceeded"; }
    else if (std::abs(command) > command_limit + 1e-9) { safe = true; reason = "command limit exceeded"; }
    else if (dropouts >= dropout_limit) { safe = true; reason = "sensor dropout limit exceeded"; }
    return safe;
}
} // namespace robosim
