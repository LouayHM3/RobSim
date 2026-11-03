#include "robosim/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace robosim {
namespace {
std::ofstream output_file(const Path& path) {
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    std::ofstream file;
    file.exceptions(std::ios::failbit | std::ios::badbit);
    file.open(path, std::ios::binary);
    file.imbue(std::locale::classic());
    file << std::setprecision(17);
    return file;
}
void text_file(const Path& path, const std::string& text) {
    auto file = output_file(path); file << text; file.close();
}
std::string escape(const std::string& text) {
    std::string result;
    for (char c : text) {
        switch (c) {
        case '&': result += "&amp;"; break;
        case '<': result += "&lt;"; break;
        case '>': result += "&gt;"; break;
        case '"': result += "&quot;"; break;
        default: result += c;
        }
    }
    return result;
}
std::string cell(const Json& value) {
    return value.is_string() ? value.get<std::string>() : value.dump();
}
std::string markdown_cell(const Json& value) {
    auto text = cell(value);
    std::string result;
    for (char c : text) {
        if (c == '|') result += "\\|";
        else if (c == '\n' || c == '\r') result += ' ';
        else result += c;
    }
    return result;
}
std::string csv_cell(const Json& value) {
    if (value.is_null()) return "";
    const auto text = cell(value);
    if (text.find_first_of(",\"\r\n") == std::string::npos) return text;
    std::string escaped = "\"";
    for (char c : text) { escaped += c; if (c == '"') escaped += c; }
    return escaped + '"';
}
} // namespace

void write_json(const SimulationResult& result, const Path& path) {
    text_file(path, result.to_json().dump(2) + '\n');
}

void write_csv(const SimulationResult& result, const Path& path) {
    auto file = output_file(path);
    const auto rows = result.to_json().at("samples");
    if (!rows.empty()) {
        bool first = true;
        for (auto it = rows.front().begin(); it != rows.front().end(); ++it) {
            if (!first) file << ',';
            file << it.key(); first = false;
        }
        file << '\n';
        for (const auto& row : rows) {
            first = true;
            for (const auto& value : row) {
                if (!first) file << ',';
                file << csv_cell(value); first = false;
            }
            file << '\n';
        }
    }
    file.close();
}

void write_plot(const SimulationResult& result, const Path& path, bool error) {
    if (path.extension() != ".svg") throw std::invalid_argument("plot path must use the .svg extension");
    if (result.samples.empty()) throw std::invalid_argument("cannot plot an empty simulation");
    double low = 0.0, high = 0.0;
    for (const auto& s : result.samples) {
        low = std::min(low, error ? s.target - s.position : std::min(s.target, s.position));
        high = std::max(high, error ? s.target - s.position : std::max(s.target, s.position));
    }
    const double padding = std::max(0.05, (high - low) * 0.12);
    low -= padding; high += padding;
    const double duration = std::max(result.samples.back().time, 0.01);
    auto x = [duration](double t) { return 90.0 + 820.0 * t / duration; };
    auto y = [low, high](double v) { return 420.0 - 310.0 * (v - low) / (high - low); };
    auto file = output_file(path);
    file << std::fixed << std::setprecision(3);
    file << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"960\" height=\"500\" viewBox=\"0 0 960 500\" role=\"img\" aria-labelledby=\"title description\">\n"
         << "<title id=\"title\">" << escape(result.scenario) << (error ? ": tracking error" : ": target and position") << "</title>\n"
         << "<desc id=\"description\">Fixed-step RoboSim C++ simulation. Time is in seconds; position uses scenario units.</desc>\n"
         << "<rect width=\"960\" height=\"500\" fill=\"#0f172a\"/>\n"
         << "<g font-family=\"Arial, sans-serif\" fill=\"#e2e8f0\">\n"
         << "<text x=\"90\" y=\"40\" font-size=\"24\" font-weight=\"bold\">" << escape(result.scenario) << "</text>\n"
         << "<text x=\"90\" y=\"67\" font-size=\"14\" fill=\"#94a3b8\">" << (error ? "Tracking error" : "Closed-loop actuator response") << "</text>\n";
    for (int i = 0; i <= 5; ++i) {
        const double t = duration * static_cast<double>(i) / 5.0;
        const double value = low + (high - low) * static_cast<double>(i) / 5.0;
        file << "<path d=\"M " << x(t) << " 110 V 420\" stroke=\"#243247\"/>\n"
             << "<path d=\"M 90 " << y(value) << " H 910\" stroke=\"#243247\"/>\n"
             << "<text x=\"" << x(t) << "\" y=\"445\" text-anchor=\"middle\" font-size=\"12\">" << t << "</text>\n"
             << "<text x=\"78\" y=\"" << y(value) + 4 << "\" text-anchor=\"end\" font-size=\"12\">" << value << "</text>\n";
    }
    file << "<path d=\"M 90 110 V 420 H 910\" fill=\"none\" stroke=\"#94a3b8\"/>\n";
    if (error) file << "<path d=\"M 90 " << y(0.0) << " H 910\" stroke=\"#64748b\" stroke-dasharray=\"5 4\"/>\n";
    for (int series = 0; series < (error ? 1 : 2); ++series) {
        file << "<polyline fill=\"none\" stroke=\"" << (series == 0 && !error ? "#fbbf24" : "#38bdf8")
             << "\" stroke-width=\"2.5\" stroke-linejoin=\"round\" points=\"";
        for (const auto& s : result.samples)
            file << x(s.time) << ',' << y(error ? s.target - s.position : (series == 0 ? s.target : s.position)) << ' ';
        file << "\"/>\n";
    }
    file << "<text x=\"500\" y=\"480\" text-anchor=\"middle\" font-size=\"14\">Time (s)</text>\n"
         << "<text transform=\"translate(23 265) rotate(-90)\" text-anchor=\"middle\" font-size=\"14\">"
         << (error ? "Tracking error" : "Position") << " (scenario units)</text>\n";
    if (!error) file << "<path d=\"M 650 60 H 680\" stroke=\"#fbbf24\" stroke-width=\"3\"/><text x=\"690\" y=\"65\" font-size=\"14\">Target</text>\n"
        << "<path d=\"M 770 60 H 800\" stroke=\"#38bdf8\" stroke-width=\"3\"/><text x=\"810\" y=\"65\" font-size=\"14\">Position</text>\n";
    file << "</g>\n</svg>\n";
    file.close();
}

void write_output_bundle(const SimulationResult& result, const Path& directory) {
    std::filesystem::create_directories(directory);
    write_json(result, directory / "result.json");
    const auto metrics = result.metrics();
    text_file(directory / "metrics.json", metrics.dump(2) + '\n');
    write_csv(result, directory / "timeseries.csv");
    const auto data = result.to_json();
    auto events = data["events"];
    for (const auto& d : data["diagnostics"]) events.push_back(d);
    text_file(directory / "events.json", events.dump(2) + '\n');
    std::ostringstream report;
    report << "# RoboSim report: " << markdown_cell(result.scenario) << "\n\n"
           << "Generated by the C++ simulation engine.\n\nStatus: **" << (result.passed ? "PASS" : "FAIL") << "**\n\n"
           << "![Target and position](trajectory.svg)\n\n![Tracking error](tracking_error.svg)\n\n## Metrics\n\n";
    for (auto it = metrics.begin(); it != metrics.end(); ++it)
        report << "- **" << it.key() << "**: " << markdown_cell(it.value()) << '\n';
    report << "\n## Diagnostics\n\n";
    if (events.empty()) report << "No fault events or diagnostics.\n";
    for (const auto& d : events) report << "- `" << d.at("time").get<double>() << "s` "
        << cell(d["level"]) << ' ' << cell(d["code"]) << ": " << markdown_cell(d["message"]) << '\n';
    text_file(directory / "report.md", report.str());
    write_plot(result, directory / "trajectory.svg");
    write_plot(result, directory / "tracking_error.svg", true);
}

void write_summary(const std::vector<SimulationResult>& results, const Path& directory) {
    Json rows = Json::array();
    const std::vector<std::string> columns = {"scenario", "passed", "final_error", "max_error",
        "duration", "safe_state", "fault_detection_latency"};
    std::ostringstream report;
    report << "# RoboSim scenario verification\n\nGenerated by the C++ simulation engine.\n\n|";
    for (const auto& column : columns) report << ' ' << column << " |";
    report << "\n|";
    for (std::size_t i = 0; i < columns.size(); ++i) report << " --- |";
    report << '\n';
    for (const auto& result : results) {
        const auto metrics = result.metrics(); rows.push_back(metrics);
        report << '|';
        for (const auto& column : columns) report << ' ' << markdown_cell(metrics.value(column, Json(nullptr))) << " |";
        report << '\n';
    }
    text_file(directory / "summary.json", rows.dump(2) + '\n');
    text_file(directory / "summary.md", report.str());
}
} // namespace robosim
