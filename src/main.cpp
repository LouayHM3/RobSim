#include "robosim/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace {
void help() {
    std::cout << "RoboSim 0.2.0 - C++ actuator simulation and verification\n\n"
        "Usage:\n"
        "  robosim run SCENARIO [--json FILE] [--csv FILE] [--plot FILE.svg] [--output DIR]\n"
        "  robosim verify SCENARIO_OR_DIRECTORY [--output DIR]\n"
        "  robosim benchmark SCENARIO [--repeats N]\n\n"
        "verify/benchmark: exit 0 on acceptance success, 1 on failure.\n"
        "run: exit 0 after simulation completion. Invalid input or I/O: exit 2.\n";
}
int repeats(const std::string& text) {
    if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("repeats must be an integer between 1 and 100000");
    const auto value = std::stoull(text);
    if (value < 1 || value > 100000) throw std::invalid_argument("repeats must be between 1 and 100000");
    return static_cast<int>(value);
}
} // namespace

int main(int argc, char** argv) {
    using namespace robosim;
    try {
        if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) { help(); return 0; }
        if (argc == 2 && std::string(argv[1]) == "--version") { std::cout << "RoboSim 0.2.0\n"; return 0; }
        if (argc < 3) { help(); return 2; }
        const std::string command = argv[1];
        const Path scenario = argv[2];
        std::set<std::string> allowed;
        if (command == "run") allowed = {"--json", "--csv", "--plot", "--output"};
        else if (command == "verify") allowed = {"--output"};
        else if (command == "benchmark") allowed = {"--repeats"};
        else throw std::invalid_argument("unknown command: " + command);
        std::map<std::string, std::string> options;
        for (int i = 3; i < argc; ++i) {
            const std::string option = argv[i];
            if (!allowed.count(option)) throw std::invalid_argument("unknown option: " + option);
            if (options.count(option)) throw std::invalid_argument("duplicate option: " + option);
            if (i + 1 >= argc || std::string(argv[i + 1]).rfind("--", 0) == 0)
                throw std::invalid_argument("missing value for " + option);
            options[option] = argv[++i];
        }
        if (options.count("--plot") && Path(options.at("--plot")).extension() != ".svg")
            throw std::invalid_argument("--plot requires a .svg output path");
        if (command == "benchmark") {
            const int count = options.count("--repeats") ? repeats(options.at("--repeats")) : 3;
            const auto config = load_scenario(scenario);
            const auto started = std::chrono::steady_clock::now();
            bool passed = true;
            for (int i = 0; i < count; ++i) passed = simulate(config).passed && passed;
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            std::cout << Json({{"repeats", count}, {"elapsed_seconds", elapsed}, {"all_passed", passed}}).dump() << '\n';
            return passed ? 0 : 1;
        }
        if (command == "verify" && std::filesystem::is_directory(scenario)) {
            std::vector<Path> paths;
            std::set<std::string> stems;
            for (const auto& entry : std::filesystem::directory_iterator(scenario)) {
                if (entry.is_regular_file() && (entry.path().extension() == ".yaml" || entry.path().extension() == ".yml")) {
                    if (!stems.insert(entry.path().stem().string()).second)
                        throw std::invalid_argument("duplicate scenario filename stem: " + entry.path().stem().string());
                    paths.push_back(entry.path());
                }
            }
            if (paths.empty()) throw std::invalid_argument("no YAML scenarios found in " + scenario.string());
            std::sort(paths.begin(), paths.end());
            const Path output = options.count("--output") ? Path(options.at("--output")) : Path("results/verify");
            std::vector<SimulationResult> results;
            bool passed = true;
            for (const auto& path : paths) {
                auto result = run_file(path);
                write_output_bundle(result, output / path.stem());
                std::cout << result.metrics().dump() << '\n';
                passed = passed && result.passed;
                results.push_back(std::move(result));
            }
            write_summary(results, output);
            return passed ? 0 : 1;
        }
        const auto result = run_file(scenario);
        std::cout << result.metrics().dump(2) << '\n';
        if (options.count("--json")) write_json(result, options.at("--json"));
        if (options.count("--csv")) write_csv(result, options.at("--csv"));
        if (options.count("--plot")) write_plot(result, options.at("--plot"));
        if (options.count("--output")) write_output_bundle(result, options.at("--output"));
        return command == "verify" && !result.passed ? 1 : 0;
    } catch (const std::exception& error) {
        std::cerr << "robosim: " << error.what() << '\n';
        return 2;
    }
}
