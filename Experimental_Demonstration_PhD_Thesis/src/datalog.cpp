#include "datalog.h"

#include <chrono>
#include <ctime>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <iostream>
#include <streambuf>

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
// Console logger (tee: consola + archivo)
// -----------------------------------------------------------------------------
class TeeBuf : public std::streambuf {
public:
    TeeBuf(std::streambuf* a, std::streambuf* b) : a_(a), b_(b) {}
protected:
    int overflow(int c) override {
        if (c == std::char_traits<char>::eof()) return c;
        const int r1 = a_ ? a_->sputc(static_cast<char>(c)) : c;
        const int r2 = b_ ? b_->sputc(static_cast<char>(c)) : c;
        return (r1 == std::char_traits<char>::eof() || r2 == std::char_traits<char>::eof())
               ? std::char_traits<char>::eof() : c;
    }
    int sync() override {
        const int r1 = a_ ? a_->pubsync() : 0;
        const int r2 = b_ ? b_->pubsync() : 0;
        return (r1 == 0 && r2 == 0) ? 0 : -1;
    }
private:
    std::streambuf* a_;
    std::streambuf* b_;
};

ConsoleLogger::ConsoleLogger() = default;

ConsoleLogger::~ConsoleLogger() { stop(); }

bool ConsoleLogger::start(const std::string& path, bool append) {
    if (file_.is_open()) return true;
    file_.open(path, append ? std::ios::app : std::ios::out);
    if (!file_.is_open()) return false;
    oldCout_ = std::cout.rdbuf();
    oldCerr_ = std::cerr.rdbuf();
    outTee_  = std::make_unique<TeeBuf>(oldCout_, file_.rdbuf());
    errTee_  = std::make_unique<TeeBuf>(oldCerr_, file_.rdbuf());
    std::cout.rdbuf(outTee_.get());
    std::cerr.rdbuf(errTee_.get());
    return true;
}

void ConsoleLogger::stop() {
    if (oldCout_) { std::cout.rdbuf(oldCout_); oldCout_ = nullptr; }
    if (oldCerr_) { std::cerr.rdbuf(oldCerr_); oldCerr_ = nullptr; }
    outTee_.reset();
    errTee_.reset();
    if (file_.is_open()) file_.close();
}

bool ConsoleLogger::isOpen() const { return file_.is_open(); }

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
    if (!f.is_open()) return s;
    try {
        std::string line;
        if (std::getline(f, line)) s.nextIndex = std::stoi(line);
        if (std::getline(f, line)) s.csvPath = line;
        if (std::getline(f, line)) s.sessionStamp = line;
        if (std::getline(f, line)) s.counter = std::stoi(line);
        // Only treat the state as resumable if the essential fields are present.
        s.valid = !s.sessionStamp.empty();
    } catch (const std::exception& e) {
        std::cerr << "[Warn] Ignoring corrupt state file '" << stateFile
                  << "': " << e.what() << "\n";
        s = ResumeState{};  // valid == false
    }
    return s;
}

void deleteState(const std::string& stateFile) {
    std::remove(stateFile.c_str());
}

} // namespace datalog
