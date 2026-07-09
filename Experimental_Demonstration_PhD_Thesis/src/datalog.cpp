#include "datalog.h"

#include <chrono>
#include <ctime>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace fs = std::filesystem;

namespace datalog {

// -----------------------------------------------------------------------------
// Timestamps
// -----------------------------------------------------------------------------
static std::tm localNow() {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    std::tm timeinfo;
    localtime_s(&timeinfo, &t);
    return timeinfo;
}

std::string stamp() {
    std::tm ti = localNow();
    std::ostringstream oss;
    oss << std::put_time(&ti, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::string date() {
    std::tm ti = localNow();
    std::ostringstream oss;
    oss << std::put_time(&ti, "%Y-%m-%d");
    return oss.str();
}

std::string clockTime() {
    std::tm ti = localNow();
    std::ostringstream oss;
    oss << std::put_time(&ti, "%H:%M:%S");
    return oss.str();
}

// -----------------------------------------------------------------------------
// Directorios
// -----------------------------------------------------------------------------
bool ensureDir(const std::string& path) {
    std::error_code ec;
    if (fs::exists(path, ec)) return true;
    return fs::create_directories(path, ec);
}

// -----------------------------------------------------------------------------
// Metadata
// -----------------------------------------------------------------------------
void Metadata::set(const std::string& key, const std::string& value) {
    items_.emplace_back(key, value);
}

void Metadata::set(const std::string& key, double value) {
    std::ostringstream oss;
    oss << std::setprecision(10) << value;
    items_.emplace_back(key, oss.str());
}

void Metadata::set(const std::string& key, int value) {
    items_.emplace_back(key, std::to_string(value));
}

bool Metadata::write(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) return false;
    for (const auto& kv : items_) {
        f << kv.first << "=" << kv.second << "\n";
    }
    f.close();
    return true;
}

// -----------------------------------------------------------------------------
// CSV writer
// -----------------------------------------------------------------------------
bool CsvWriter::open(const std::string& path, const std::string& header, bool append) {
    if (append) {
        f_.open(path, std::ios::app);
    } else {
        f_.open(path);
        if (f_.is_open() && !header.empty()) {
            f_ << header << "\n";
            f_.flush();
        }
    }
    return f_.is_open();
}

// -----------------------------------------------------------------------------
// Estado de reanudacion
// -----------------------------------------------------------------------------
void saveState(const std::string& stateFile, const ResumeState& s) {
    std::ofstream f(stateFile);
    if (f.is_open()) {
        f << s.nextIndex    << "\n"
          << s.csvPath      << "\n"
          << s.sessionStamp << "\n"
          << s.counter      << "\n";
        f.close();
    }
}

ResumeState loadState(const std::string& stateFile) {
    ResumeState s;
    std::ifstream f(stateFile);
    if (f.is_open()) {
        std::string line;
        if (std::getline(f, line)) s.nextIndex = std::stoi(line);
        if (std::getline(f, line)) s.csvPath = line;
        if (std::getline(f, line)) s.sessionStamp = line;
        if (std::getline(f, line)) s.counter = std::stoi(line);
        s.valid = true;
        f.close();
    }
    return s;
}

void deleteState(const std::string& stateFile) {
    std::remove(stateFile.c_str());
}

} // namespace datalog
