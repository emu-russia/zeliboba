// zeliboba - the event tracing system.
//
// The model is deliberately close to Windows ETW, because that is the mental
// model a Vita kernel/boot trace wants to be read with:
//
//   provider   an emitter with a stable name and GUID ("Zeliboba-Boot")
//   manifest   the provider's event table: id, level, opcode, task, keyword
//   keyword    64-bit category mask; a session subscribes to a subset
//   level      Critical..Verbose; a session records events at or below its level
//   opcode     Info for one-shot events, Start/Stop for activities
//   activity   a 64-bit id that links the Start and the Stop of a long operation
//
// Events are timestamped with the *emulated* clock (nanoseconds derived from the
// Kermit cycle counter), so a trace line and a WPA-style graph describe guest
// time, not host time.
//
// Storage is a bounded ring buffer: one session per machine, records overwritten
// oldest-first, plus per-provider counters that survive the wrap so the summary
// view stays correct after a long run.  Emission is on the emulation thread.
#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/types.h"

namespace zlb {

// ---------------------------------------------------------------------------
// Levels (ETW EventLevel)
// ---------------------------------------------------------------------------

enum class EventLevel : u8 {
    LogAlways = 0,
    Critical = 1,
    Error = 2,
    Warning = 3,
    Informational = 4,
    Verbose = 5,
};

const char* to_string(EventLevel level);
bool parse_event_level(const std::string& text, EventLevel& out);
constexpr int kEventLevelCount = 6;

// ---------------------------------------------------------------------------
// Opcodes (ETW EventOpcode)
// ---------------------------------------------------------------------------

enum class EventOpcode : u8 {
    Info = 0,
    Start = 1,
    Stop = 2,
    DcStart = 3,
    DcStop = 4,
    Extension = 5,
    Reply = 6,
    Resume = 7,
    Suspend = 8,
    Send = 9,
    Receive = 10,
};

const char* to_string(EventOpcode opcode);
inline bool is_activity_start(EventOpcode opcode) {
    return opcode == EventOpcode::Start || opcode == EventOpcode::DcStart;
}
inline bool is_activity_stop(EventOpcode opcode) {
    return opcode == EventOpcode::Stop || opcode == EventOpcode::DcStop;
}

// ---------------------------------------------------------------------------
// Keywords: a 64-bit mask grouped by subsystem, like ETW keyword bits.
// ---------------------------------------------------------------------------

using EventKeyword = u64;

namespace event_keyword {
constexpr EventKeyword kNone = 0;
constexpr EventKeyword kState = 1ull << 0;      ///< reset/build/save state
constexpr EventKeyword kBoot = 1ull << 1;       ///< boot chain stages
constexpr EventKeyword kCpu = 1ull << 2;        ///< core lifecycle and faults
constexpr EventKeyword kInterrupt = 1ull << 3;  ///< IRQ/FIQ raise, delivery, EOI
constexpr EventKeyword kTimer = 1ull << 4;      ///< timers and systimer
constexpr EventKeyword kMemory = 1ull << 5;     ///< RAM regions, allocations
constexpr EventKeyword kStorage = 1ull << 6;    ///< eMMC, SDIF, DMA
constexpr EventKeyword kDisplay = 1ull << 7;    ///< display, IFTU, DSI, GPU
constexpr EventKeyword kPower = 1ull << 8;      ///< syscon / power / reset
constexpr EventKeyword kSecurity = 1ull << 9;   ///< CMeP, mailbox, keyring, crypto
constexpr EventKeyword kLoader = 1ull << 10;    ///< SLB2/SELF/ELF loading
constexpr EventKeyword kKernel = 1ull << 11;    ///< guest kernel milestones
constexpr EventKeyword kComm = 1ull << 12;      ///< SC/mailbox/guest messaging
constexpr EventKeyword kDebug = 1ull << 13;     ///< debugger stops and probes
/// Bus register/RAM tracing: very high volume, so a session has to ask for it.
constexpr EventKeyword kBusAccess = 1ull << 14;
/// The default subscription: everything except the high-volume bus access trace.
constexpr EventKeyword kDefault = ~kBusAccess;
constexpr EventKeyword kAll = ~0ull;
}  // namespace event_keyword

bool parse_event_keyword(const std::string& text, EventKeyword& out);
std::string describe_event_keywords(EventKeyword mask);

// ---------------------------------------------------------------------------
// Areas: the Graph Explorer top level.
// ---------------------------------------------------------------------------

enum class EventArea : u8 {
    SystemActivity = 0,
    Computation,
    Storage,
    Memory,
    Video,
    Power,
    Communications,
    Other,
    Count,
};

constexpr int kEventAreaCount = static_cast<int>(EventArea::Count);
const char* to_string(EventArea area);
bool parse_event_area(const std::string& text, EventArea& out);

// ---------------------------------------------------------------------------
// Providers
// ---------------------------------------------------------------------------

enum class EventProvider : u8 {
    Machine = 0,
    Boot,
    Cpu,
    Interrupt,
    Timer,
    Memory,
    Emmc,
    Sdif,
    Dma,
    Display,
    Gpu,
    Syscon,
    Cmep,
    Mailbox,
    Crypto,
    Loader,
    Kernel,
    Bus,
    Debugger,
    Test,
    Count,
};

constexpr int kEventProviderCount = static_cast<int>(EventProvider::Count);

const char* to_string(EventProvider provider);
bool parse_event_provider(const std::string& text, EventProvider& out);

// ---------------------------------------------------------------------------
// Manifest
// ---------------------------------------------------------------------------

struct EventMetadata {
    u16 id = 0;
    u8 version = 0;
    u8 channel = 0;
    EventLevel level = EventLevel::Informational;
    EventOpcode opcode = EventOpcode::Info;
    u16 task = 0;
    EventKeyword keyword = event_keyword::kNone;
    const char* name = "";        ///< "StageEnter"
    const char* fields = "";      ///< "stage,detail" - documentation only
    const char* description = "";
};

struct EventProviderInfo {
    EventProvider id = EventProvider::Test;
    const char* name = "";
    const char* guid = "";        ///< stable "8-4-4-4-12" string, ETW style
    EventArea area = EventArea::Other;
    const char* description = "";
    const EventMetadata* events = nullptr;
    size_t event_count = 0;
};

const EventProviderInfo& event_provider_info(EventProvider provider);
/// Manifest entry for an event id, or nullptr when the provider has none.
const EventMetadata* event_metadata(EventProvider provider, u16 id);
const char* event_name(EventProvider provider, u16 id);

// ---------------------------------------------------------------------------
// Values and records
// ---------------------------------------------------------------------------

enum class EventValueKind : u8 { None = 0, UInt, Int, Bool, Float, Address, Text };

struct EventValue {
    EventValueKind kind = EventValueKind::None;
    u8 width = 0;              ///< bytes, for UInt/Int (0 = natural 8)
    u64 bits = 0;
    const char* text = nullptr;

    static EventValue from_u64(u64 value, u8 width = 0);
    static EventValue from_i64(s64 value);
    static EventValue from_bool(bool value);
    static EventValue from_double(double value);
    static EventValue from_address(u64 value);
    static EventValue from_text(const char* value);

    std::string to_string() const;
};

struct EventField {
    const char* name = nullptr;
    EventValue value;
};

constexpr size_t kMaxEventFields = 4;

struct EventRecord {
    u64 time_ns = 0;           ///< emulated time since reset
    u64 sequence = 0;          ///< monotonic, survives the ring wrap
    u64 activity = 0;          ///< 0 = no activity
    u64 parent_activity = 0;   ///< enclosing activity, for nested Begin/End
    EventProvider provider = EventProvider::Test;
    u16 id = 0;
    u16 task = 0;
    EventKeyword keyword = event_keyword::kNone;
    u8 version = 0;
    EventLevel level = EventLevel::Informational;
    EventOpcode opcode = EventOpcode::Info;
    u8 field_count = 0;
    EventField fields[kMaxEventFields] = {};
    const char* name = nullptr;  ///< manifest name, stable

    bool has_fields() const { return field_count != 0; }
    const EventField* find(const char* field_name) const;
    std::string payload_text() const;
    bool is_activity() const { return is_activity_start(opcode) || is_activity_stop(opcode); }
};

// ---------------------------------------------------------------------------
// Filtering and aggregation
// ---------------------------------------------------------------------------

struct EventFilter {
    EventLevel max_level = EventLevel::Verbose;
    EventKeyword keyword = event_keyword::kAll;
    u64 provider_mask = ~0ull;
    u64 area_mask = ~0ull;
    u64 time_lo_ns = 0;
    u64 time_hi_ns = ~0ull;
    std::string text;             ///< case-insensitive substring over the record text
    bool activities_only = false; ///< only Start/Stop records
    bool one_shot_only = false;   ///< only non-activity records
    bool has_task = false;
    u16 task = 0;
    bool has_opcode = false;
    EventOpcode opcode = EventOpcode::Info;
    bool has_id = false;
    u16 id = 0;

    void set_provider(EventProvider provider, bool on);
    bool provider_enabled(EventProvider provider) const;
    void set_area(EventArea area, bool on);
    bool area_enabled(EventArea area) const;
    void clear_providers() { provider_mask = 0; }
    void all_providers() { provider_mask = ~0ull; }
};

struct EventCounts {
    u64 records = 0;
    u64 providers[kEventProviderCount] = {};
    u64 areas[kEventAreaCount] = {};
    u64 levels[kEventLevelCount] = {};
    u64 opcodes[16] = {};
    u64 total_span_ns = 0;   ///< last - first matched timestamp
};

/// A matched Begin/End pair (or an unmatched Begin still open at the end).
struct EventActivitySpan {
    EventProvider provider = EventProvider::Test;
    u16 id = 0;
    u16 task = 0;
    u64 activity = 0;
    u64 parent = 0;
    u64 begin_ns = 0;
    u64 end_ns = 0;
    u64 duration_ns = 0;
    bool open = false;
    EventLevel level = EventLevel::Informational;
    EventKeyword keyword = event_keyword::kNone;
    const char* name = nullptr;

    double duration_ms() const { return static_cast<double>(duration_ns) / 1.0e6; }
};

/// One row of the WPA-style summary table: Count / Weight / %Weight.
struct EventSummaryRow {
    EventProvider provider = EventProvider::Test;
    u16 id = 0;
    u16 task = 0;
    EventOpcode opcode = EventOpcode::Info;
    const char* name = nullptr;
    EventLevel level = EventLevel::Verbose;  ///< most severe level in the group
    u64 count = 0;
    u64 weight_ns = 0;   ///< summed duration for activities
    u64 min_ns = 0;
    u64 max_ns = 0;
    double percent = 0.0;
};

struct EventTimelineBucket {
    u64 time_ns = 0;
    u64 end_ns = 0;
    u64 counts[kEventAreaCount] = {};
    u64 weight_ns[kEventAreaCount] = {};
    u64 total = 0;
};

// ---------------------------------------------------------------------------
// The session log
// ---------------------------------------------------------------------------

class EventLog {
public:
    explicit EventLog(size_t capacity = 1u << 16);
    ~EventLog();

    EventLog(const EventLog&) = delete;
    EventLog& operator=(const EventLog&) = delete;

    // -- session configuration ------------------------------------------

    void set_recording(bool on) { recording_ = on; }
    bool recording() const { return recording_; }

    void set_capacity(size_t capacity);
    size_t capacity() const { return ring_.size(); }

    void set_max_level(EventLevel level) { max_level_ = level; }
    EventLevel max_level() const { return max_level_; }

    void set_keyword_mask(EventKeyword mask) { keyword_mask_ = mask; }
    EventKeyword keyword_mask() const { return keyword_mask_; }

    void set_provider_mask(u64 mask) { provider_mask_ = mask; }
    u64 provider_mask() const { return provider_mask_; }
    void set_provider_enabled(EventProvider provider, bool on);
    bool provider_enabled(EventProvider provider) const;
    void set_all_providers(bool on);

    /// Enable a whole Graph Explorer area (Storage, Video, ...).
    void set_area_enabled(EventArea area, bool on);
    bool area_enabled(EventArea area) const;

    /// Drop every record and counter but keep the session configuration.
    void clear();
    /// Drop records, counters, activity stack, sequence and the intern table.
    void reset();

    // -- clock -----------------------------------------------------------

    void set_clock(std::function<u64()> clock) { clock_ = std::move(clock); }
    u64 now() const { return clock_ ? clock_() : 0; }

    // -- emission --------------------------------------------------------

    bool should_record(EventProvider provider, EventLevel level, EventKeyword keyword) const;

    /// Raw append (used by the builder and by tests). Fills the timestamp and
    /// sequence number when they are zero and applies the session filter.
    bool emit(EventRecord& record);

    class Builder {
    public:
        Builder(EventLog& log, EventRecord record) : log_(&log), record_(record) {}

        Builder& field(const char* name, u64 value, u8 width = 0);
        Builder& field(const char* name, s64 value);
        Builder& field(const char* name, bool value);
        Builder& field(const char* name, double value);
        Builder& field(const char* name, const std::string& value);
        Builder& address(const char* name, u64 value);
        Builder& level(EventLevel level);
        Builder& keyword(EventKeyword keyword);
        Builder& task(u16 task);
        Builder& opcode(EventOpcode opcode);

        /// Append the record. Returns the assembled record (the local copy when
        /// the session filtered it out).
        const EventRecord& emit();
        /// Give an already-created activity id to a Stop record.
        Builder& activity(u64 activity);
        /// Pop this activity off the stack when the record is emitted.  Used by
        /// end_event() so the Stop record can carry result fields.
        Builder& pop_on_emit(u64 activity);
        /// Push this activity onto the stack when the record is emitted.  Used by
        /// begin_event() so the Start record can carry fields.
        Builder& push_on_emit(u64 activity);
        const EventRecord& record() const { return record_; }

    private:
        EventLog* log_ = nullptr;
        EventRecord record_;
        bool emitted_ = false;
        u64 pop_activity_ = 0;
        u64 push_activity_ = 0;
    };

    /// One-shot event, metadata (level/opcode/task/keyword) from the manifest.
    Builder event(EventProvider provider, u16 id);
    /// Start an activity and return a Builder for its fields.  The activity id is
    /// already in `record().activity`; the stack is updated when emit() runs.
    Builder begin_event(EventProvider provider, u16 id);
    /// Start an activity: returns the new activity id and appends a Start record.
    u64 begin(EventProvider provider, u16 id);
    /// Append a Stop record for `activity` (no extra fields).
    void end(u64 activity, EventProvider provider, u16 id);
    /// Builder for a Stop record with fields (result codes etc.).
    Builder end_event(u64 activity, EventProvider provider, u16 id);

    u64 current_activity() const { return activity_stack_.empty() ? 0 : activity_stack_.back(); }
    size_t activity_depth() const { return activity_stack_.size(); }
    void reset_activities();

    /// Intern a string so a record can hold a stable `const char*`.
    const char* intern(const std::string& text);

    // -- queries ---------------------------------------------------------

    size_t count() const { return count_; }
    u64 total() const { return total_; }
    u64 filtered() const { return filtered_; }
    u64 sequence() const { return sequence_; }

    std::vector<EventRecord> all() const { return tail(count_); }
    std::vector<EventRecord> tail(size_t count) const;
    /// Newest matching records, oldest first, at most `count`. Scans the ring
    /// backwards and stops as soon as `count` matches are found, so a live view
    /// never copies the whole log.
    std::vector<EventRecord> tail_matching(const EventFilter& filter, size_t count) const;
    bool match(const EventRecord& record, const EventFilter& filter) const;
    std::vector<EventRecord> query(const EventFilter& filter, size_t max_results = 0) const;

    u64 first_time() const;
    u64 last_time() const;
    /// Time [lo, hi] of the records matching a filter; false when none match.
    bool time_range(const EventFilter& filter, u64& lo, u64& hi) const;

    struct ProviderCounters {
        u64 emitted = 0;
        u64 begins = 0;
        u64 ends = 0;
        u64 open = 0;
        u64 total_ns = 0;
        u64 max_ns = 0;
        u64 min_ns = 0;
        u64 first_ns = 0;
        u64 last_ns = 0;
        u64 dropped_by_ring = 0;
    };
    const ProviderCounters& provider_counters(EventProvider provider) const;
    EventCounts counts(const EventFilter& filter) const;

    std::vector<EventActivitySpan> activities(const EventFilter& filter, size_t max_results = 0) const;
    std::vector<EventSummaryRow> summary(const EventFilter& filter, size_t max_results = 0) const;
    std::vector<EventTimelineBucket> timeline(const EventFilter& filter, int buckets) const;

    // -- formatting and export -------------------------------------------

    std::string format_record(const EventRecord& record, bool with_payload = true) const;
    std::string format_table(const std::vector<EventRecord>& records) const;
    std::string export_csv(const EventFilter& filter) const;
    bool save_csv(const std::string& path, const EventFilter& filter, std::string& error) const;

private:
    void account(const EventRecord& record);
    void pop_activity(u64 activity);
    size_t bucket_of(u64 time, u64 lo, u64 hi, int buckets) const;

    std::vector<EventRecord> ring_;
    size_t head_ = 0;
    size_t count_ = 0;
    u64 total_ = 0;
    u64 filtered_ = 0;
    u64 sequence_ = 0;
    u64 next_activity_ = 1;

    bool recording_ = true;
    EventLevel max_level_ = EventLevel::Informational;
    EventKeyword keyword_mask_ = event_keyword::kDefault;
    u64 provider_mask_ = ~0ull;

    std::function<u64()> clock_;
    std::vector<u64> activity_stack_;
    std::unordered_map<u64, u64> open_activities_;   ///< activity -> begin time
    std::array<ProviderCounters, kEventProviderCount> counters_{};
    std::unordered_map<std::string, std::unique_ptr<char[]>> interned_;
};

// ---------------------------------------------------------------------------
// Process-wide active log
// ---------------------------------------------------------------------------
//
// The bus, the devices and the cores do not own a machine pointer, so they emit
// into the active log, which Vita::build() points at its own EventLog.  When no
// machine is built (or after it is destroyed) `events()` returns a shared
// disabled log, so every call site works unconditionally.

EventLog& events();
EventLog* active_events();
void set_active_events(EventLog* log);

/// RAII Begin/End pair.  The Stop record is emitted by end() or by the
/// destructor, so an early return can never leave a dangling activity.
class EventActivity {
public:
    EventActivity(EventLog& log, EventProvider provider, u16 id)
        : log_(&log), provider_(provider), id_(id), activity_(log.begin(provider, id)) {}

    EventActivity(const EventActivity&) = delete;
    EventActivity& operator=(const EventActivity&) = delete;

    ~EventActivity() { end_if_open(); }

    void end_if_open() {
        if (log_ && activity_ != 0) {
            log_->end(activity_, provider_, id_);
            log_ = nullptr;
            activity_ = 0;
        }
    }
    EventLog::Builder end_event() {
        EventLog& log = log_ ? *log_ : events();
        EventLog::Builder builder = log.end_event(activity_, provider_, id_);
        log_ = nullptr;
        activity_ = 0;
        return builder;
    }

    u64 activity() const { return activity_; }
    bool open() const { return activity_ != 0; }

private:
    EventLog* log_ = nullptr;
    EventProvider provider_ = EventProvider::Test;
    u16 id_ = 0;
    u64 activity_ = 0;
};

}  // namespace zlb
