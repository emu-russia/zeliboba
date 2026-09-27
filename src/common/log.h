// zeliboba - logging.
//
// Every subsystem logs through a named category so that the debugger can turn
// individual noisy areas (mmio, emmc, bigmac, ...) on and off at runtime.
#pragma once

#include <cstdarg>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/types.h"

namespace zlb {

enum class LogLevel : int { Trace = 0, Debug = 1, Info = 2, Warn = 3, Error = 4, Off = 5 };

struct LogRecord {
    LogLevel level;
    std::string category;
    std::string text;
    u64 sequence;
};

class Log {
public:
    using Sink = std::function<void(const LogRecord&)>;

    static Log& instance();

    void set_level(LogLevel level) { level_ = level; }
    LogLevel level() const { return level_; }

    void set_category_level(const std::string& category, LogLevel level);
    bool enabled(const std::string& category, LogLevel level) const;
    bool category_enabled(const std::string& category) const { return enabled(category, LogLevel::Trace); }

    void add_sink(Sink sink);
    void clear_sinks();

    void log(LogLevel level, const std::string& category, const std::string& text);

    // Convenience wrappers -------------------------------------------------
    void trace(const std::string& c, const std::string& t) { log(LogLevel::Trace, c, t); }
    void debug(const std::string& c, const std::string& t) { log(LogLevel::Debug, c, t); }
    void info(const std::string& c, const std::string& t) { log(LogLevel::Info, c, t); }
    void warn(const std::string& c, const std::string& t) { log(LogLevel::Warn, c, t); }
    void error(const std::string& c, const std::string& t) { log(LogLevel::Error, c, t); }

    // printf style helper --------------------------------------------------
    void printf(LogLevel level, const std::string& category, const char* fmt, ...);

private:
    Log() = default;

    LogLevel level_ = LogLevel::Info;
    std::unordered_map<std::string, LogLevel> category_levels_;
    std::vector<Sink> sinks_;
    std::mutex mutex_;
    u64 sequence_ = 0;
};

// Shorthands used all over the code base.
#define ZLB_LOG_INFO(cat, ...)  ::zlb::Log::instance().printf(::zlb::LogLevel::Info, cat, __VA_ARGS__)
#define ZLB_LOG_WARN(cat, ...)  ::zlb::Log::instance().printf(::zlb::LogLevel::Warn, cat, __VA_ARGS__)
#define ZLB_LOG_ERROR(cat, ...) ::zlb::Log::instance().printf(::zlb::LogLevel::Error, cat, __VA_ARGS__)
#define ZLB_LOG_DBG(cat, ...)   ::zlb::Log::instance().printf(::zlb::LogLevel::Debug, cat, __VA_ARGS__)
#define ZLB_LOG_TRACE(cat, ...) ::zlb::Log::instance().printf(::zlb::LogLevel::Trace, cat, __VA_ARGS__)

const char* to_string(LogLevel level);
bool parse_log_level(const std::string& text, LogLevel& out);

}  // namespace zlb
