#pragma once

#include <filesystem>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

namespace robosim {
using Json = nlohmann::json;
using Path = std::filesystem::path;

Json load_scenario(const Path& path);
Json normalize_scenario(Json config);
double target_at(const Json& config, double time);

struct PID {
    double kp = 4.0, ki = 0.2, kd = 0.5;
    double output_min = -10.0, output_max = 10.0, integral = 0.0;
    std::optional<double> previous_error;
    explicit PID(const Json& config = Json::object());
    double update(double target, double measured, double dt);
    void reset();
};

struct Plant {
    double position = 0.0, velocity = 0.0, mass = 1.0, damping = 0.2, limit = 10.0;
    std::optional<double> velocity_limit, acceleration_limit;
    double acceleration = 0.0;
    bool acceleration_limit_violation = false;
    explicit Plant(const Json& config = Json::object());
    void step(double force, double dt);
};

struct Actuator {
    double maximum = 10.0, output = 0.0, velocity = 0.0, acceleration = 0.0;
    std::optional<double> slew_rate, max_velocity, max_acceleration;
    bool stuck = false, velocity_violation = false, acceleration_violation = false;
    explicit Actuator(const Json& config = Json::object());
    double apply(double request, double dt);
};

class Sensor {
public:
    explicit Sensor(const Json& config, unsigned seed);
    std::pair<std::optional<double>, double> read(double position, double time);
    bool dropout = false, communication_interruption = false, invalid = false;
    double drift_per_second = 0.0;
    std::optional<double> stuck_value;
private:
    double noise_std_ = 0.0, bias_ = 0.0, latency_ = 0.0;
    std::optional<double> quantization_, valid_min_, valid_max_;
    std::mt19937 rng_;
    std::vector<std::pair<double, double>> history_;
    double gaussian();
};

struct SafetyMonitor {
    double error_limit = 5.0, command_limit = 10.0;
    int dropout_limit = 3, dropouts = 0;
    bool safe = false;
    std::string reason;
    explicit SafetyMonitor(const Json& config = Json::object());
    bool check(double error, double command, bool measurement_valid);
};

struct Diagnostic {
    std::string level, code, message;
    double time = 0.0;
};

struct Sample {
    double time = 0.0, target = 0.0, position = 0.0, velocity = 0.0, command = 0.0;
    double measured_position = 0.0, requested_command = 0.0, measurement_age = 0.0;
    double actuator_velocity = 0.0, actuator_acceleration = 0.0;
    std::optional<std::string> fault;
    bool safe = false, emergency_stop = false, measurement_valid = true;
    bool position_limit_violation = false, velocity_limit_violation = false;
    bool actuator_limit_violation = false, actuator_velocity_violation = false;
    bool actuator_acceleration_violation = false, acceleration_limit_violation = false;
    bool feedback_delayed = false, sensor_unchanged = false;
};

struct SimulationResult {
    std::string scenario;
    std::vector<Sample> samples;
    std::vector<Diagnostic> diagnostics, events;
    bool passed = false;
    std::string reason;
    Json metrics() const;
    Json to_json() const;
};

SimulationResult simulate(Json config);
SimulationResult run_file(const Path& path);
void write_json(const SimulationResult& result, const Path& path);
void write_csv(const SimulationResult& result, const Path& path);
void write_plot(const SimulationResult& result, const Path& path, bool error = false);
void write_output_bundle(const SimulationResult& result, const Path& directory);
void write_summary(const std::vector<SimulationResult>& results, const Path& directory);
} // namespace robosim
