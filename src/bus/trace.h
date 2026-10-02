// zeliboba - bus access trace.
//
// A lock free-ish ring buffer of the last N bus accesses. The debugger uses it
// for "what touched this register" style questions and for the live MMIO view;
// the regression tests use it to assert that a boot stage touched the hardware
// it is supposed to touch.
#pragma once

#include <string>
#include <vector>

#include "common/types.h"

namespace zlb {

enum class AccessKind : u8 { Read = 0, Write = 1, Fetch = 2 };

const char* to_string(AccessKind kind);

struct AccessRecord {
    AccessKind kind = AccessKind::Read;
    u8 size = 4;
    u32 address = 0;
    u64 value = 0;
    u32 pc = 0;
    int device = -1;      // index into Bus::devices(), -1 for plain RAM
    bool unmapped = false;
    u64 sequence = 0;
};

class TraceLog {
public:
    explicit TraceLog(size_t capacity = 1u << 18);

    void set_capacity(size_t capacity);
    size_t capacity() const { return records_.size(); }

    void push(const AccessRecord& record);

    /// Copy the newest `count` records, oldest first.
    std::vector<AccessRecord> tail(size_t count) const;
    std::vector<AccessRecord> all() const;

    /// Records newer than `sequence`, oldest first, at most `max_count`.
    /// Watchpoints use this: scanning the whole ring once per instruction made
    /// stepping quadratic.
    std::vector<AccessRecord> since(u64 sequence, size_t max_count = 512) const;

    /// Records matching an address (and optionally a mask), oldest first.
    std::vector<AccessRecord> find(u32 address, u32 mask = 0xFFFFFFFFu, size_t max_count = 256) const;

    void clear();
    u64 total() const { return total_; }
    size_t count() const { return count_; }

    /// Restore the monotonic sequence counter after a save state is loaded. The
    /// ring contents themselves are not part of a state (they are a debugging
    /// aid, not machine state), so `clear()` plus this keeps `since(sequence)`
    /// working for watchpoints.
    void set_total(u64 value) { total_ = value; }

    /// When false, RAM accesses are not recorded (MMIO always is).
    void set_trace_ram(bool enabled) { trace_ram_ = enabled; }
    bool trace_ram() const { return trace_ram_; }

private:
    std::vector<AccessRecord> records_;
    size_t head_ = 0;
    size_t count_ = 0;
    u64 total_ = 0;
    bool trace_ram_ = false;
};

}  // namespace zlb
