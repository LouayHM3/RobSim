#include "robosim/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace robosim {
namespace {
Json convert(const YAML::Node& node) {
    if (!node || node.IsNull()) return nullptr;
    if (node.IsMap()) {
        Json result = Json::object();
        for (const auto& entry : node) {
            const auto key = entry.first.as<std::string>();
            if (result.contains(key)) throw std::invalid_argument("duplicate YAML key: " + key);
            result[key] = convert(entry.second);
        }
        return result;
    }
    if (node.IsSequence()) {
        Json result = Json::array();
        for (const auto& entry : node) result.push_back(convert(entry));
        return result;
    }
    const auto value = node.Scalar();
    if (node.Tag() == "!" || node.Tag() == "tag:yaml.org,2002:str") return value;
    if (value == "true" || value == "True" || value == "TRUE") return true;
    if (value == "false" || value == "False" || value == "FALSE") return false;
    char* end = nullptr;
    const double number = std::strtod(value.c_str(), &end);
    if (!value.empty() && end == value.c_str() + value.size()) return number;
    return value;
}

void number(const Json& object, const std::string& key, double lower,
            bool strictly_positive = false) {
    if (!object.contains(key) || object[key].is_null()) return;
    if (!object[key].is_number()) throw std::invalid_argument(key + " must be a number");
    const double value = object[key].get<double>();
    if (!std::isfinite(value) || value < lower || (strictly_positive && value <= 0.0))
        throw std::invalid_argument("invalid value for " + key);
}

void boolean(const Json& object, const std::string& key) {
    if (object.contains(key) && !object[key].is_boolean())
        throw std::invalid_argument(key + " must be true or false");
}

void keys(const Json& object, const std::set<std::string>& allowed,
          const std::string& section) {
    for (auto it = object.begin(); it != object.end(); ++it)
        if (!allowed.count(it.key()))
            throw std::invalid_argument("unknown " + section + " setting: " + it.key());
}

void finite_numbers(const Json& object) {
    for (const auto& value : object) {
        if (value.is_number() && !std::isfinite(value.get<double>()))
            throw std::invalid_argument("scenario numbers must be finite");
        if (value.is_structured()) finite_numbers(value);
    }
}
} // namespace

Json load_scenario(const Path& path) {
    try {
        return normalize_scenario(convert(YAML::LoadFile(path.string())));
    } catch (const std::exception& error) {
        throw std::invalid_argument(path.string() + ": " + error.what());
    }
}

Json normalize_scenario(Json config) {
    if (!config.is_object()) throw std::invalid_argument("scenario must be a YAML mapping");
    keys(config, {"name", "duration", "dt", "seed", "target", "trajectory", "plant", "pid",
                  "actuator", "sensor", "safety", "diagnostics", "faults", "acceptance"}, "scenario");
    finite_numbers(config);
    const Json defaults = {{"name", "nominal"}, {"duration", 5.0}, {"dt", 0.01},
                           {"seed", 1}, {"target", 1.0}};
    for (auto it = defaults.begin(); it != defaults.end(); ++it)
        if (!config.contains(it.key())) config[it.key()] = it.value();
    if (!config["name"].is_string()) throw std::invalid_argument("name must be a string");
    for (const auto* section : {"plant", "pid", "actuator", "sensor", "safety", "diagnostics", "acceptance"}) {
        if (!config.contains(section)) config[section] = Json::object();
        if (!config[section].is_object()) throw std::invalid_argument(std::string(section) + " must be a mapping");
    }
    number(config, "duration", 0.0, true);
    number(config, "dt", 0.0, true);
    number(config, "target", -std::numeric_limits<double>::max());
    number(config, "seed", 0.0);
    const double seed = config.at("seed").get<double>();
    if (seed != std::floor(seed) || seed > std::numeric_limits<unsigned>::max())
        throw std::invalid_argument("seed must be a 32-bit unsigned integer");
    if (config.at("duration").get<double>() / config.at("dt").get<double>() > 1000000.0)
        throw std::invalid_argument("scenario exceeds one million simulation steps");
    const std::set<std::string> plant_keys = {"position", "velocity", "mass", "damping", "limit",
        "velocity_limit", "max_velocity", "acceleration_limit"};
    keys(config["plant"], plant_keys, "plant");
    keys(config["pid"], {"kp", "ki", "kd", "output_min", "output_max"}, "pid");
    keys(config["actuator"], {"maximum", "slew_rate", "max_velocity", "max_acceleration",
                             "velocity_limit", "acceleration_limit"}, "actuator");
    keys(config["sensor"], {"noise_std", "bias", "dropout", "drift_per_second", "stuck_value",
        "latency", "communication_interruption", "quantization", "quantization_step",
        "valid_min", "valid_max", "invalid"}, "sensor");
    keys(config["safety"], {"error_limit", "command_limit", "dropout_limit"}, "safety");
    keys(config["diagnostics"], {"feedback_latency_limit", "max_feedback_delay", "unchanged_samples"}, "diagnostics");
    keys(config["acceptance"], {"final_error", "expected_safe", "expected_emergency_stop"}, "acceptance");
    // Every component field is numeric except the explicitly listed sensor booleans.
    for (const auto* section : {"plant", "pid", "actuator", "sensor", "safety", "diagnostics", "acceptance"}) {
        for (auto it = config[section].begin(); it != config[section].end(); ++it) {
            const bool is_bool = (std::string(section) == "sensor" &&
                (it.key() == "dropout" || it.key() == "communication_interruption" || it.key() == "invalid")) ||
                (std::string(section) == "acceptance" && it.key() != "final_error");
            if (is_bool) boolean(config[section], it.key());
            else number(config[section], it.key(), -std::numeric_limits<double>::max());
            if (it.value().is_null() && (std::string(section) == "pid" ||
                (std::string(section) == "plant" && (it.key() == "mass" || it.key() == "damping" ||
                  it.key() == "position" || it.key() == "velocity" || it.key() == "limit"))))
                throw std::invalid_argument(it.key() + " cannot be null");
        }
    }
    number(config["plant"], "mass", 0.0, true);
    for (const auto* key : {"limit", "velocity_limit", "max_velocity", "acceleration_limit"})
        number(config["plant"], key, 0.0, true);
    number(config["plant"], "damping", 0.0);
    for (const auto* key : {"maximum", "slew_rate", "max_velocity", "max_acceleration", "velocity_limit", "acceleration_limit"})
        number(config["actuator"], key, 0.0);
    for (const auto* key : {"noise_std", "latency", "quantization", "quantization_step"})
        number(config["sensor"], key, 0.0);
    for (const auto* key : {"error_limit", "command_limit"}) number(config["safety"], key, 0.0);
    number(config["acceptance"], "final_error", 0.0);
    for (const auto* key : {"feedback_latency_limit", "max_feedback_delay"}) number(config["diagnostics"], key, 0.0);
    for (const auto& item : {std::make_pair("safety", "dropout_limit"), std::make_pair("diagnostics", "unchanged_samples")}) {
        if (config[item.first].contains(item.second)) {
            const double value = config[item.first][item.second].get<double>();
            if (value < 1 || value > 1000000 || value != std::floor(value))
                throw std::invalid_argument(std::string(item.second) + " must be a positive integer <= 1000000");
        }
    }
    const auto& pid = config["pid"];
    if (pid.value("output_min", -10.0) > pid.value("output_max", 10.0))
        throw std::invalid_argument("output_min must not exceed output_max");
    const auto& sensor = config["sensor"];
    if (sensor.contains("valid_min") && sensor.contains("valid_max") &&
        !sensor["valid_min"].is_null() && !sensor["valid_max"].is_null() &&
        sensor["valid_min"].get<double>() > sensor["valid_max"].get<double>())
        throw std::invalid_argument("valid_min must not exceed valid_max");
    if (!config.contains("faults")) config["faults"] = Json::array();
    if (!config["faults"].is_array()) throw std::invalid_argument("faults must be a sequence");
    const std::set<std::string> fault_types = {"sensor_dropout", "sensor_drift", "sensor_stuck",
        "communication_interruption", "emergency_stop", "sensor_invalid", "invalid_reading", "actuator_stuck"};
    for (const auto& fault : config["faults"]) {
        if (!fault.is_object() || !fault.contains("type") || !fault["type"].is_string())
            throw std::invalid_argument("each fault requires a type");
        keys(fault, {"type", "start", "end", "rate"}, "fault");
        if (!fault_types.count(fault.at("type").get<std::string>())) throw std::invalid_argument("unknown fault type");
        number(fault, "start", 0.0); number(fault, "end", 0.0);
        number(fault, "rate", -std::numeric_limits<double>::max());
        if (fault.value("start", 0.0) > fault.value("end", config.at("duration").get<double>()))
            throw std::invalid_argument("fault start must not exceed end");
    }
    if (config.contains("trajectory") && !config["trajectory"].is_null()) {
        const auto& trajectory = config["trajectory"];
        if (!trajectory.is_object()) throw std::invalid_argument("trajectory must be a mapping");
        keys(trajectory, {"type", "value", "before", "initial", "time", "at", "step_time", "after",
            "final", "start", "from", "end", "to", "start_time", "duration", "points"}, "trajectory");
        for (auto it = trajectory.begin(); it != trajectory.end(); ++it)
            if (it.key() != "type" && it.key() != "points")
                number(trajectory, it.key(), -std::numeric_limits<double>::max());
        const auto kind = trajectory.value("type", std::string("fixed"));
        if (kind != "fixed" && kind != "step" && kind != "ramp" && kind != "multi-point" &&
            kind != "multipoint" && kind != "multi_point") throw std::invalid_argument("unknown trajectory type: " + kind);
        if (kind == "ramp") number(trajectory, "duration", 0.0, true);
        if (trajectory.contains("points")) {
            if (!trajectory["points"].is_array()) throw std::invalid_argument("points must be a sequence");
            double last_time = -1.0;
            for (const auto& point : trajectory["points"]) {
                if (!(point.is_object() && point.contains("time") && point.contains("value")) &&
                    !(point.is_array() && point.size() == 2)) throw std::invalid_argument("invalid trajectory point");
                const auto& time = point.is_object() ? point.at("time") : point.at(0);
                const auto& value = point.is_object() ? point.at("value") : point.at(1);
                if (!time.is_number() || !value.is_number() || time.get<double>() < 0.0 || time.get<double>() <= last_time)
                    throw std::invalid_argument("trajectory times must be nonnegative and strictly increasing");
                last_time = time.get<double>();
            }
        }
    }
    return config;
}

double target_at(const Json& config, double time) {
    const double target = config.value("target", 0.0);
    if (!config.contains("trajectory") || config["trajectory"].is_null()) return target;
    const auto& t = config["trajectory"];
    const auto kind = t.value("type", std::string("fixed"));
    if (kind == "fixed") return t.value("value", target);
    if (kind == "step") return time < t.value("time", t.value("at", t.value("step_time", 0.0)))
        ? t.value("before", t.value("initial", target)) : t.value("after", t.value("final", t.value("value", target)));
    if (kind == "ramp") {
        const double start = t.value("start", t.value("from", 0.0));
        const double end = t.value("end", t.value("to", target));
        const double fraction = std::clamp((time - t.value("start_time", 0.0)) /
            t.value("duration", config.value("duration", 1.0)), 0.0, 1.0);
        return start + (end - start) * fraction;
    }
    const auto points = t.value("points", Json::array());
    if (points.empty()) return target;
    auto coordinate = [](const Json& point, bool value) {
        return point.is_object() ? point.at(value ? "value" : "time").get<double>() : point.at(value ? 1 : 0).get<double>();
    };
    if (time <= coordinate(points.front(), false)) return coordinate(points.front(), true);
    for (std::size_t i = 1; i < points.size(); ++i) {
        const double t0 = coordinate(points[i - 1], false), t1 = coordinate(points[i], false);
        if (time <= t1) {
            const double v0 = coordinate(points[i - 1], true), v1 = coordinate(points[i], true);
            return v0 + (v1 - v0) * (time - t0) / (t1 - t0);
        }
    }
    return coordinate(points.back(), true);
}
} // namespace robosim
