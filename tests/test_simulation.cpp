#include "robosim/simulation.hpp"

#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace robosim;
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance = 1e-10) {
    require(std::abs(actual - expected) <= tolerance,
        "expected " + std::to_string(expected) + ", got " + std::to_string(actual));
}
void rejects(const std::function<void()>& action) {
    bool threw = false;
    try { action(); } catch (const std::exception&) { threw = true; }
    require(threw, "expected input rejection");
}
std::string read(const Path& path) {
    std::ifstream input(path, std::ios::binary);
    require(input.good(), "missing file " + path.string());
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    const Path root = argv[1];
    const Path output = std::filesystem::current_path() / "test-artifacts";
    std::filesystem::create_directories(output);
    int failures = 0, cases = 0;
    auto test = [&](const std::string& name, const std::function<void()>& action) {
        ++cases;
        try { action(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    };

    test("nominal determinism and Python reference parity", [&] {
        const auto first = run_file(root / "scenarios/nominal.yaml");
        const auto second = run_file(root / "scenarios/nominal.yaml");
        require(first.to_json() == second.to_json(), "repeated simulations differ");
        require(first.passed, "nominal failed acceptance");
        require(first.samples.size() == 201, "unexpected sample count");
        near(first.metrics().at("final_error").get<double>(), 0.10699429380680048);
        near(first.samples.front().position, 0.0001);
        near(first.samples.back().time, 2.0);
    });
    test("all eight scenarios pass", [&] {
        int count = 0;
        for (const auto& entry : std::filesystem::directory_iterator(root / "scenarios")) {
            if (entry.path().extension() != ".yaml") continue;
            const auto result = run_file(entry.path());
            require(!result.samples.empty(), "empty result");
            require(result.passed, entry.path().filename().string() + " failed: " + result.reason);
            ++count;
        }
        require(count == 8, "expected eight baseline scenarios");
    });
    test("noiseless fault scenarios match Python reference errors", [&] {
        const std::vector<std::pair<std::string, double>> references = {
            {"actuator_saturation", 8.0}, {"communication_interruption", 0.1120439626894214},
            {"emergency_stop", 0.06047897806775049}, {"sensor_drift", 6.1791448286573125},
            {"sensor_dropout", 0.9272407545673598}, {"sensor_stuck", 0.3728445142360802}};
        for (const auto& reference : references)
            near(run_file(root / "scenarios" / (reference.first + ".yaml")).metrics().at("final_error").get<double>(), reference.second);
    });
    test("dropout timing and latched safe state", [&] {
        const auto result = run_file(root / "scenarios/sensor_dropout.yaml");
        near(result.metrics().at("safe_state_time").get<double>(), 0.02);
        near(result.metrics().at("fault_detection_latency").get<double>(), 0.02);
        require(result.samples.back().safe, "safe state did not latch");
        require(result.metrics().at("invalid_readings") == result.samples.size(), "dropouts not counted");
    });
    test("emergency stop and residual motion", [&] {
        const auto result = run_file(root / "scenarios/emergency_stop.yaml");
        require(result.passed && result.metrics().at("emergency_stop").get<bool>(), "stop missing");
        near(result.metrics().at("safe_state_time").get<double>(), 0.35);
        require(result.samples[35].velocity > 0, "safe state should retain existing velocity");
    });
    test("scheduled dropout recovers feedback but keeps safety latched", [&] {
        const auto result = simulate({{"duration", 0.2}, {"target", 1.0},
            {"faults", Json::array({{{"type", "sensor_dropout"}, {"start", 0.02}, {"end", 0.08}}})},
            {"acceptance", {{"expected_safe", true}}}});
        require(result.passed, "expected safe state");
        require(result.samples.back().measurement_valid && result.samples.back().safe, "recovery/safe latch failed");
    });
    test("PID saturation anti-windup and reset", [&] {
        PID pid({{"kp", 2.0}, {"ki", 1.0}, {"kd", 0.0}, {"output_min", -1.0}, {"output_max", 1.0}});
        near(pid.update(100, 0, 0.01), 1.0); near(pid.integral, 0.0);
        near(pid.update(0.1, 0, 0.01), 0.201); near(pid.integral, 0.001);
        pid.reset(); near(pid.integral, 0.0); require(!pid.previous_error, "reset retained error");
    });
    test("actuator saturation slew and stuck behavior", [&] {
        Actuator actuator({{"maximum", 2.0}, {"slew_rate", 1.0}});
        near(actuator.apply(100, 0.1), 0.1);
        for (int i = 0; i < 100; ++i) require(std::abs(actuator.apply(100, 0.1)) <= 2.0, "output out of range");
        actuator.stuck = true; near(actuator.apply(-100, 0.1), 2.0); near(actuator.velocity, 0.0);
    });
    test("actuator velocity and acceleration constraints", [&] {
        Actuator actuator({{"maximum", 10.0}, {"max_velocity", 1.0}, {"max_acceleration", 2.0}});
        actuator.apply(10, 0.1);
        require(actuator.velocity_violation && actuator.acceleration_violation, "violations missing");
        near(actuator.acceleration, 2.0); near(actuator.velocity, 0.2); near(actuator.output, 0.02);
        for (int i = 0; i < 20; ++i) {
            actuator.apply(10, 0.1);
            require(std::abs(actuator.velocity) <= 1.0 + 1e-12, "velocity limit exceeded");
            require(std::abs(actuator.acceleration) <= 2.0 + 1e-12, "acceleration limit exceeded");
        }
    });
    test("plant acceleration velocity and position constraints", [&] {
        Plant plant({{"mass", 2.0}, {"damping", 0.0}, {"limit", 0.15}, {"max_velocity", 0.5}, {"acceleration_limit", 1.0}});
        plant.step(100, 0.1); near(plant.acceleration, 1.0); near(plant.position, 0.01);
        require(plant.acceleration_limit_violation, "acceleration violation missing");
        for (int i = 0; i < 100; ++i) plant.step(100, 0.1);
        near(plant.position, 0.15); near(plant.velocity, 0.0);
    });
    test("sensor latency bias quantization and range checks", [&] {
        Sensor sensor({{"latency", 0.1}, {"bias", 0.02}, {"quantization_step", 0.1}}, 1);
        near(*sensor.read(1.0, 0.0).first, 1.0);
        const auto reading = sensor.read(2.0, 0.1);
        near(*reading.first, 1.0); near(reading.second, 0.1);
        Sensor bounded({{"valid_min", -1.0}, {"valid_max", 1.0}}, 1);
        require(!bounded.read(2.0, 0).first, "out-of-range reading accepted");
        bounded.dropout = true; require(!bounded.read(0, 0).first, "dropout returned a value");
    });
    test("seeded noise reproducibility and seed sensitivity", [&] {
        auto config = load_scenario(root / "scenarios/sensor_noise.yaml");
        const auto first = simulate(config), second = simulate(config);
        require(first.to_json() == second.to_json(), "same seed differs");
        config["seed"] = 12;
        require(first.to_json() != simulate(config).to_json(), "different seed unchanged");
    });
    test("configured drift and stuck reading persist", [&] {
        auto config = Json{{"duration", 0.1}, {"sensor", {{"drift_per_second", 2.0}, {"stuck_value", 0.0}}}};
        const auto result = simulate(config);
        near(result.samples.back().measured_position, 0.2);
    });
    test("delayed and unchanged feedback diagnostics", [&] {
        auto delayed = simulate({{"duration", 0.2}, {"sensor", {{"latency", 0.05}}}});
        require(delayed.metrics().at("feedback_delay_diagnostics").get<int>() > 0, "delay not detected");
        auto stuck = simulate({{"duration", 0.2}, {"sensor", {{"stuck_value", 0.0}}}});
        require(stuck.metrics().at("sensor_unchanged_diagnostics").get<int>() > 0, "unchanged not detected");
    });
    test("fixed step ramp and interpolated trajectories", [&] {
        auto config = normalize_scenario({{"duration", 2.0}, {"trajectory", {{"type", "fixed"}, {"value", 3.0}}}});
        near(target_at(config, 1), 3.0);
        config["trajectory"] = {{"type", "step"}, {"time", 1.0}, {"before", 0.0}, {"after", 2.0}};
        near(target_at(config, 0.99), 0.0); near(target_at(config, 1.0), 2.0);
        config["trajectory"] = {{"type", "ramp"}, {"start", 0.0}, {"end", 2.0}, {"duration", 2.0}};
        near(target_at(config, 1.0), 1.0); near(target_at(config, 3.0), 2.0);
        config["trajectory"] = {{"type", "multi-point"}, {"points", Json::array({
            {{"time", 0.0}, {"value", 0.0}}, {{"time", 1.0}, {"value", 2.0}}, {{"time", 2.0}, {"value", 1.0}}})}};
        near(target_at(config, 0.5), 1.0); near(target_at(config, 1.5), 1.5); near(target_at(config, 5), 1.0);
        config["trajectory"]["points"] = Json::array({Json::array({0, 0}), Json::array({1, 2})});
        near(target_at(config, 0.5), 1.0);
    });
    test("acceptance failure and expected-safe criteria", [&] {
        require(!simulate({{"duration", 0.1}, {"acceptance", {{"final_error", 0.0}}}}).passed, "bad tracking passed");
        require(!simulate({{"duration", 0.1}, {"acceptance", {{"expected_safe", true}}}}).passed, "missing safe state passed");
        require(!simulate({{"duration", 0.1}, {"acceptance", {{"expected_emergency_stop", true}}}}).passed, "missing stop passed");
        const auto safe = simulate({{"target", 100.0}, {"duration", 0.1}, {"acceptance", {{"expected_safe", true}, {"final_error", 0.0}}}});
        require(safe.passed, "expected safe state should bypass error threshold");
    });
    test("known metrics from synthetic trace", [&] {
        SimulationResult result;
        result.scenario = "metrics";
        for (int i = 0; i < 3; ++i) {
            Sample s; s.time = i * 0.1; s.target = 1.0; s.position = i * 0.5;
            result.samples.push_back(s);
        }
        const auto metrics = result.metrics();
        near(metrics.at("final_error").get<double>(), 0.0);
        near(metrics.at("mean_absolute_error").get<double>(), 0.5);
        near(metrics.at("rmse").get<double>(), std::sqrt(1.25 / 3.0));
        near(metrics.at("integral_absolute_error").get<double>(), 0.15);
        near(metrics.at("settling_time").get<double>(), 0.2);
        near(metrics.at("overshoot").get<double>(), 0.0);
        require(metrics.at("safe_state_time").is_null(), "spurious safe state");
    });
    test("report bundle schema SVG and repeatable outputs", [&] {
        const auto result = run_file(root / "scenarios/nominal.yaml");
        write_output_bundle(result, output / "nominal");
        for (const auto* name : {"result.json", "metrics.json", "events.json", "report.md", "timeseries.csv", "trajectory.svg", "tracking_error.svg"})
            require(std::filesystem::file_size(output / "nominal" / name) > 0, std::string("empty output ") + name);
        require(Json::parse(read(output / "nominal/result.json")) == result.to_json(), "JSON round trip differs");
        require(Json::parse(read(output / "nominal/events.json")).empty(), "nominal has unexpected events");
        require(read(output / "nominal/trajectory.svg").find("<polyline") != std::string::npos, "missing plot data");
        const auto json = read(output / "nominal/result.json");
        write_output_bundle(result, output / "nominal");
        require(json == read(output / "nominal/result.json"), "report changed on repeat");
        write_summary({result}, output);
        require(Json::parse(read(output / "summary.json")).size() == 1, "bad summary");
        rejects([&] { write_plot(result, output / "wrong.png"); });
    });
    test("YAML flow mappings sequences duplicate keys and invalid syntax", [&] {
        const Path yaml = output / "input.yaml";
        { std::ofstream file(yaml); file << "name: test\nduration: 0.1\nplant: {mass: 2}\nfaults: [{type: sensor_dropout, start: 0}]\n"; }
        require(load_scenario(yaml)["plant"]["mass"] == 2.0, "YAML mapping failed");
        { std::ofstream file(yaml); file << "duration: 1\nduration: 2\n"; }
        rejects([&] { load_scenario(yaml); });
        { std::ofstream file(yaml); file << "plant: [invalid\n"; }
        rejects([&] { load_scenario(yaml); });
    });
    test("configuration validation rejects dangerous and misspelled values", [&] {
        const std::vector<Json> bad = {
            {{"dt", 0}}, {{"dt", -0.1}}, {{"duration", -1}}, {{"seed", -1}}, {{"seed", 0.5}},
            {{"seed", 4294967296.0}}, {{"duration", 1e9}}, {{"name", 1}}, {{"dt", "0.01"}},
            {{"target", std::numeric_limits<double>::infinity()}}, {{"plant", {{"mass", 0}}}},
            {{"plant", {{"mass", nullptr}}}}, {{"plant", {{"limit", -1}}}}, {{"plant", {{"mas", 1}}}},
            {{"pid", {{"output_min", 10}, {"output_max", -10}}}}, {{"sensor", {{"noise_std", -1}}}},
            {{"sensor", {{"dropout", 1}}}}, {{"sensor", {{"valid_min", 2}, {"valid_max", 1}}}},
            {{"safety", {{"dropout_limit", 0}}}}, {{"diagnostics", {{"unchanged_samples", 1.5}}}},
            {{"faults", Json::array({{{"type", "unknown"}}})}},
            {{"faults", Json::array({{{"type", "sensor_dropout"}, {"start", 2}, {"end", 1}}})}},
            {{"trajectory", {{"type", "ramp"}, {"duration", 0}}}},
            {{"trajectory", {{"type", "multi-point"}, {"points", Json::array({Json::array({1, 0}), Json::array({0, 1})})}}}}
        };
        for (const auto& config : bad) rejects([&] { simulate(config); });
        rejects([&] { normalize_scenario(Json::array()); });
    });
    std::cout << cases - failures << '/' << cases << " test groups passed\n";
    return failures == 0 ? 0 : 1;
}
