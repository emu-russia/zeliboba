#include "common/log.h"

#include <cstdio>

namespace zlb {

Log& Log::instance() {
    static Log log;
    return log;
}

void Log::set_category_level(const std::string& category, LogLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);
    category_levels_[category] = level;
}

bool Log::enabled(const std::string& category, LogLevel level) const {
    if (level < level_) return false;
    auto it = category_levels_.find(category);
    if (it != category_levels_.end() && level < it->second) return false;
    return true;
}

void Log::add_sink(Sink sink) {
    std::lock_guard<std::mutex> lock(mutex_);
    sinks_.push_back(std::move(sink));
}

void Log::clear_sinks() {
    std::lock_guard<std::mutex> lock(mutex_);
    sinks_.clear();
}

void Log::log(LogLevel level, const std::string& category, const std::string& text) {
    if (!enabled(category, level)) return;

    LogRecord record;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        record.sequence = ++sequence_;
    }
    record.level = level;
    record.category = category;
    record.text = text;

    std::vector<Sink> sinks;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sinks = sinks_;
    }
    for (auto& sink : sinks) sink(record);
}

void Log::printf(LogLevel level, const std::string& category, const char* fmt, ...) {
    if (!enabled(category, level)) return;

    char stack_buffer[1024];
    va_list args;
    va_start(args, fmt);
    int needed = std::vsnprintf(stack_buffer, sizeof(stack_buffer), fmt, args);
    va_end(args);

    if (needed < 0) return;

    if (static_cast<size_t>(needed) < sizeof(stack_buffer)) {
        log(level, category, std::string(stack_buffer, static_cast<size_t>(needed)));
        return;
    }

    std::string big(static_cast<size_t>(needed) + 1, '\0');
    va_start(args, fmt);
    std::vsnprintf(big.data(), big.size(), fmt, args);
    va_end(args);
    big.resize(static_cast<size_t>(needed));
    log(level, category, big);
}

const char* to_string(LogLevel level) {
    switch (level) {
        case LogLevel::Trace: return "trace";
        case LogLevel::Debug: return "debug";
        case LogLevel::Info: return "info";
        case LogLevel::Warn: return "warn";
        case LogLevel::Error: return "error";
        case LogLevel::Off: return "off";
    }
    return "?";
}

bool parse_log_level(const std::string& text, LogLevel& out) {
    if (text == "trace") { out = LogLevel::Trace; return true; }
    if (text == "debug") { out = LogLevel::Debug; return true; }
    if (text == "info") { out = LogLevel::Info; return true; }
    if (text == "warn") { out = LogLevel::Warn; return true; }
    if (text == "error") { out = LogLevel::Error; return true; }
    if (text == "off") { out = LogLevel::Off; return true; }
    return false;
}

}  // namespace zlb
