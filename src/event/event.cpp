// zeliboba - the event tracing system implementation.
//
// The hot path is `should_record()` + a fixed-size ring push: no allocation, no
// string formatting, no map lookups.  Everything expensive (matching a filter,
// pairing activities, building the timeline) happens in the query methods the
// debugger and the UI call when they refresh a view.
#include "event/event.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <map>
#include <tuple>

#include "common/util.h"
#include "event/providers.h"

namespace zlb {

// ---------------------------------------------------------------------------
// Levels
// ---------------------------------------------------------------------------

const char* to_string(EventLevel level) {
    switch (level) {
        case EventLevel::LogAlways: return "LogAlways";
        case EventLevel::Critical: return "Critical";
        case EventLevel::Error: return "Error";
        case EventLevel::Warning: return "Warning";
        case EventLevel::Informational: return "Informational";
        case EventLevel::Verbose: return "Verbose";
    }
    return "?";
}

bool parse_event_level(const std::string& text, EventLevel& out) {
    const std::string name = to_lower(trim(text));
    if (name.empty()) return false;
    if (name == "0" || name == "always" || name == "logalways") { out = EventLevel::LogAlways; return true; }
    if (name == "1" || name == "critical" || name == "crit") { out = EventLevel::Critical; return true; }
    if (name == "2" || name == "error" || name == "err") { out = EventLevel::Error; return true; }
    if (name == "3" || name == "warning" || name == "warn") { out = EventLevel::Warning; return true; }
    if (name == "4" || name == "info" || name == "information" || name == "informational") {
        out = EventLevel::Informational;
        return true;
    }
    if (name == "5" || name == "verbose" || name == "debug" || name == "trace") {
        out = EventLevel::Verbose;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Opcodes
// ---------------------------------------------------------------------------

const char* to_string(EventOpcode opcode) {
    switch (opcode) {
        case EventOpcode::Info: return "Info";
        case EventOpcode::Start: return "Start";
        case EventOpcode::Stop: return "Stop";
        case EventOpcode::DcStart: return "DCStart";
        case EventOpcode::DcStop: return "DCEnd";
        case EventOpcode::Extension: return "Extension";
        case EventOpcode::Reply: return "Reply";
        case EventOpcode::Resume: return "Resume";
        case EventOpcode::Suspend: return "Suspend";
        case EventOpcode::Send: return "Send";
        case EventOpcode::Receive: return "Receive";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Keywords
// ---------------------------------------------------------------------------

namespace {

struct KeywordName {
    const char* name;
    EventKeyword bit;
};

constexpr KeywordName kKeywordNames[] = {
    {"state", event_keyword::kState},
    {"boot", event_keyword::kBoot},
    {"cpu", event_keyword::kCpu},
    {"interrupt", event_keyword::kInterrupt},
    {"irq", event_keyword::kInterrupt},
    {"timer", event_keyword::kTimer},
    {"memory", event_keyword::kMemory},
    {"mem", event_keyword::kMemory},
    {"storage", event_keyword::kStorage},
    {"io", event_keyword::kStorage},
    {"display", event_keyword::kDisplay},
    {"video", event_keyword::kDisplay},
    {"power", event_keyword::kPower},
    {"security", event_keyword::kSecurity},
    {"sec", event_keyword::kSecurity},
    {"loader", event_keyword::kLoader},
    {"kernel", event_keyword::kKernel},
    {"comm", event_keyword::kComm},
    {"communication", event_keyword::kComm},
    {"debug", event_keyword::kDebug},
    {"bus", event_keyword::kBusAccess},
    {"bus-access", event_keyword::kBusAccess},
};

}  // namespace

bool parse_event_keyword(const std::string& text, EventKeyword& out) {
    const std::string name = to_lower(trim(text));
    if (name.empty()) return false;
    if (name == "all" || name == "any") { out = event_keyword::kAll; return true; }
    if (name == "default") { out = event_keyword::kDefault; return true; }
    if (name == "none" || name == "0") { out = event_keyword::kNone; return true; }

    // A number is taken as a raw mask.
    if (std::isdigit(static_cast<unsigned char>(name[0])) || name[0] == '$') {
        u64 value = 0;
        if (parse_u64(name, value)) { out = value; return true; }
        return false;
    }

    // Otherwise a '|' or ',' separated list of names.
    EventKeyword mask = 0;
    for (const std::string& part : split(replace_all(name, "|", ","), ',')) {
        const std::string item = trim(part);
        if (item.empty()) continue;
        bool matched = false;
        for (const KeywordName& entry : kKeywordNames) {
            if (item == entry.name) {
                mask |= entry.bit;
                matched = true;
                break;
            }
        }
        if (!matched) return false;
    }
    out = mask;
    return true;
}

std::string describe_event_keywords(EventKeyword mask) {
    if (mask == event_keyword::kAll) return "all";
    if (mask == event_keyword::kDefault) return "default";
    if (mask == event_keyword::kNone) return "none";
    std::string text;
    EventKeyword seen = 0;
    for (const KeywordName& entry : kKeywordNames) {
        if (entry.bit == 0 || (seen & entry.bit) != 0) continue;
        if ((mask & entry.bit) == 0) continue;
        seen |= entry.bit;
        if (!text.empty()) text += "|";
        text += entry.name;
    }
    if (text.empty()) return hex(mask);
    return text;
}

// ---------------------------------------------------------------------------
// Areas and providers
// ---------------------------------------------------------------------------

const char* to_string(EventArea area) {
    switch (area) {
        case EventArea::SystemActivity: return "System Activity";
        case EventArea::Computation: return "Computation";
        case EventArea::Storage: return "Storage";
        case EventArea::Memory: return "Memory";
        case EventArea::Video: return "Video";
        case EventArea::Power: return "Power";
        case EventArea::Communications: return "Communications";
        case EventArea::Other: return "Other";
        default: return "?";
    }
}

bool parse_event_area(const std::string& text, EventArea& out) {
    const std::string name = to_lower(trim(text));
    if (name.empty()) return false;
    if (name == "system" || name == "system activity" || name == "activity" || name == "0") {
        out = EventArea::SystemActivity;
        return true;
    }
    if (name == "computation" || name == "cpu" || name == "1") { out = EventArea::Computation; return true; }
    if (name == "storage" || name == "io" || name == "2") { out = EventArea::Storage; return true; }
    if (name == "memory" || name == "mem" || name == "3") { out = EventArea::Memory; return true; }
    if (name == "video" || name == "display" || name == "4") { out = EventArea::Video; return true; }
    if (name == "power" || name == "5") { out = EventArea::Power; return true; }
    if (name == "communications" || name == "comm" || name == "6") { out = EventArea::Communications; return true; }
    if (name == "other" || name == "7") { out = EventArea::Other; return true; }
    return false;
}

const char* to_string(EventProvider provider) {
    switch (provider) {
        case EventProvider::Machine: return "Machine";
        case EventProvider::Boot: return "Boot";
        case EventProvider::Cpu: return "CPU";
        case EventProvider::Interrupt: return "Interrupt";
        case EventProvider::Timer: return "Timer";
        case EventProvider::Memory: return "Memory";
        case EventProvider::Emmc: return "eMMC";
        case EventProvider::Sdif: return "SDIF";
        case EventProvider::Dma: return "DMA";
        case EventProvider::Display: return "Display";
        case EventProvider::Gpu: return "GPU";
        case EventProvider::Syscon: return "Syscon";
        case EventProvider::Cmep: return "CMeP";
        case EventProvider::Mailbox: return "Mailbox";
        case EventProvider::Crypto: return "Crypto";
        case EventProvider::Loader: return "Loader";
        case EventProvider::Kernel: return "Kernel";
        case EventProvider::Bus: return "Bus";
        case EventProvider::Debugger: return "Debugger";
        case EventProvider::Test: return "Test";
        default: return "?";
    }
}

bool parse_event_provider(const std::string& text, EventProvider& out) {
    const std::string name = to_lower(trim(text));
    if (name.empty()) return false;
    for (int i = 0; i < kEventProviderCount; ++i) {
        const EventProvider provider = static_cast<EventProvider>(i);
        if (name == to_lower(to_string(provider))) { out = provider; return true; }
        // Also accept the manifest name ("Zeliboba-Boot") and its tail ("boot").
        const EventProviderInfo& info = event_provider_info(provider);
        const std::string full = to_lower(info.name);
        if (name == full) { out = provider; return true; }
        if (full.size() > 10 && name == full.substr(10)) { out = provider; return true; }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Values and records
// ---------------------------------------------------------------------------

EventValue EventValue::from_u64(u64 value, u8 width) {
    EventValue v;
    v.kind = EventValueKind::UInt;
    v.width = width;
    v.bits = value;
    return v;
}

EventValue EventValue::from_i64(s64 value) {
    EventValue v;
    v.kind = EventValueKind::Int;
    v.bits = static_cast<u64>(value);
    return v;
}

EventValue EventValue::from_bool(bool value) {
    EventValue v;
    v.kind = EventValueKind::Bool;
    v.bits = value ? 1u : 0u;
    return v;
}

EventValue EventValue::from_double(double value) {
    EventValue v;
    v.kind = EventValueKind::Float;
    v.bits = 0;
    v.text = nullptr;
    std::memcpy(&v.bits, &value, sizeof(double));
    return v;
}

EventValue EventValue::from_address(u64 value) {
    EventValue v;
    v.kind = EventValueKind::Address;
    v.width = 4;
    v.bits = value;
    return v;
}

EventValue EventValue::from_text(const char* value) {
    EventValue v;
    v.kind = EventValueKind::Text;
    v.text = value;
    return v;
}

std::string EventValue::to_string() const {
    switch (kind) {
        case EventValueKind::None:
            return {};
        case EventValueKind::UInt:
            if (width != 0) return "0x" + zlb::hex(bits, static_cast<int>(width) * 2);
            return zlb::format("%llu", static_cast<unsigned long long>(bits));
        case EventValueKind::Int:
            return zlb::format("%lld", static_cast<long long>(static_cast<s64>(bits)));
        case EventValueKind::Bool:
            return bits ? "true" : "false";
        case EventValueKind::Float: {
            double value = 0.0;
            std::memcpy(&value, &bits, sizeof(double));
            return zlb::format("%.4f", value);
        }
        case EventValueKind::Address:
            return "0x" + zlb::hex(bits, static_cast<int>(width == 0 ? 4 : width) * 2);
        case EventValueKind::Text:
            return text ? std::string(text) : std::string();
    }
    return {};
}

const EventField* EventRecord::find(const char* field_name) const {
    if (!field_name) return nullptr;
    for (u8 i = 0; i < field_count && i < kMaxEventFields; ++i) {
        if (fields[i].name && std::strcmp(fields[i].name, field_name) == 0) return &fields[i];
    }
    return nullptr;
}

std::string EventRecord::payload_text() const {
    std::string text;
    for (u8 i = 0; i < field_count && i < kMaxEventFields; ++i) {
        if (!text.empty()) text += "  ";
        if (fields[i].name) {
            text += fields[i].name;
            text += '=';
        }
        text += fields[i].value.to_string();
    }
    return text;
}

// ---------------------------------------------------------------------------
// EventFilter
// ---------------------------------------------------------------------------

void EventFilter::set_provider(EventProvider provider, bool on) {
    const int index = static_cast<int>(provider);
    if (index < 0 || index >= 64) return;
    const u64 bit = 1ull << index;
    if (on) provider_mask |= bit;
    else provider_mask &= ~bit;
}

bool EventFilter::provider_enabled(EventProvider provider) const {
    const int index = static_cast<int>(provider);
    if (index < 0 || index >= 64) return false;
    return (provider_mask & (1ull << index)) != 0;
}

void EventFilter::set_area(EventArea area, bool on) {
    for (int i = 0; i < kEventProviderCount; ++i) {
        const EventProvider provider = static_cast<EventProvider>(i);
        if (event_provider_info(provider).area != area) continue;
        set_provider(provider, on);
    }
}

bool EventFilter::area_enabled(EventArea area) const {
    for (int i = 0; i < kEventProviderCount; ++i) {
        const EventProvider provider = static_cast<EventProvider>(i);
        if (event_provider_info(provider).area != area) continue;
        if (provider_enabled(provider)) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// EventLog
// ---------------------------------------------------------------------------

EventLog::EventLog(size_t capacity) { ring_.resize(capacity == 0 ? 1 : capacity); }
EventLog::~EventLog() = default;

void EventLog::set_capacity(size_t capacity) {
    ring_.assign(capacity == 0 ? 1 : capacity, EventRecord{});
    clear();
}

void EventLog::clear() {
    head_ = 0;
    count_ = 0;
    total_ = 0;
    filtered_ = 0;
    sequence_ = 0;
    next_activity_ = 1;
    activity_stack_.clear();
    open_activities_.clear();
    for (ProviderCounters& counters : counters_) counters = ProviderCounters{};
}

void EventLog::reset() {
    clear();
    interned_.clear();
}

void EventLog::set_provider_enabled(EventProvider provider, bool on) {
    const int index = static_cast<int>(provider);
    if (index < 0 || index >= 64) return;
    if (on) provider_mask_ |= (1ull << index);
    else provider_mask_ &= ~(1ull << index);
}

bool EventLog::provider_enabled(EventProvider provider) const {
    const int index = static_cast<int>(provider);
    if (index < 0 || index >= 64) return false;
    return (provider_mask_ & (1ull << index)) != 0;
}

void EventLog::set_all_providers(bool on) { provider_mask_ = on ? ~0ull : 0ull; }

void EventLog::set_area_enabled(EventArea area, bool on) {
    for (int i = 0; i < kEventProviderCount; ++i) {
        const EventProvider provider = static_cast<EventProvider>(i);
        if (event_provider_info(provider).area == area) set_provider_enabled(provider, on);
    }
}

bool EventLog::area_enabled(EventArea area) const {
    for (int i = 0; i < kEventProviderCount; ++i) {
        const EventProvider provider = static_cast<EventProvider>(i);
        if (event_provider_info(provider).area == area && provider_enabled(provider)) return true;
    }
    return false;
}

bool EventLog::should_record(EventProvider provider, EventLevel level, EventKeyword keyword) const {
    if (!recording_) return false;
    if (level > max_level_) return false;
    if (!provider_enabled(provider)) return false;
    // A keyword of 0 means "always", like an ETW event that declares no keyword.
    if (keyword != 0 && (keyword & keyword_mask_) == 0) return false;
    return true;
}

const char* EventLog::intern(const std::string& text) {
    auto iter = interned_.find(text);
    if (iter != interned_.end()) return iter->second.get();
    auto owned = std::make_unique<char[]>(text.size() + 1);
    std::memcpy(owned.get(), text.c_str(), text.size() + 1);
    const char* pointer = owned.get();
    interned_.emplace(text, std::move(owned));
    return pointer;
}

void EventLog::account(const EventRecord& record) {
    const int index = static_cast<int>(record.provider);
    if (index < 0 || index >= kEventProviderCount) return;
    ProviderCounters& counters = counters_[static_cast<size_t>(index)];
    ++counters.emitted;
    if (counters.first_ns == 0 || record.time_ns < counters.first_ns) counters.first_ns = record.time_ns;
    if (record.time_ns > counters.last_ns) counters.last_ns = record.time_ns;

    if (is_activity_start(record.opcode)) {
        ++counters.begins;
        if (record.activity != 0) {
            open_activities_[record.activity] = record.time_ns;
            ++counters.open;
        }
    } else if (is_activity_stop(record.opcode)) {
        ++counters.ends;
        auto iter = open_activities_.find(record.activity);
        if (iter != open_activities_.end()) {
            const u64 duration = record.time_ns >= iter->second ? record.time_ns - iter->second : 0;
            counters.total_ns += duration;
            if (duration > counters.max_ns) counters.max_ns = duration;
            if (counters.min_ns == 0 || duration < counters.min_ns) counters.min_ns = duration;
            open_activities_.erase(iter);
            if (counters.open > 0) --counters.open;
        }
    }
}

bool EventLog::emit(EventRecord& record) {
    if (!should_record(record.provider, record.level, record.keyword)) {
        ++filtered_;
        return false;
    }
    if (record.name == nullptr) record.name = event_name(record.provider, record.id);
    if (record.time_ns == 0) record.time_ns = now();
    record.sequence = ++sequence_;
    if (record.field_count > kMaxEventFields) record.field_count = static_cast<u8>(kMaxEventFields);

    account(record);

    ring_[head_] = record;
    head_ = (head_ + 1) % ring_.size();
    if (count_ < ring_.size()) ++count_;
    ++total_;
    return true;
}

void EventLog::reset_activities() {
    activity_stack_.clear();
    open_activities_.clear();
}

// -- Builder ----------------------------------------------------------------

EventLog::Builder& EventLog::Builder::field(const char* name, u64 value, u8 width) {
    if (record_.field_count >= kMaxEventFields) return *this;
    EventField& field = record_.fields[record_.field_count++];
    field.name = name;
    field.value = EventValue::from_u64(value, width);
    return *this;
}

EventLog::Builder& EventLog::Builder::field(const char* name, s64 value) {
    if (record_.field_count >= kMaxEventFields) return *this;
    EventField& field = record_.fields[record_.field_count++];
    field.name = name;
    field.value = EventValue::from_i64(value);
    return *this;
}

EventLog::Builder& EventLog::Builder::field(const char* name, bool value) {
    if (record_.field_count >= kMaxEventFields) return *this;
    EventField& field = record_.fields[record_.field_count++];
    field.name = name;
    field.value = EventValue::from_bool(value);
    return *this;
}

EventLog::Builder& EventLog::Builder::field(const char* name, double value) {
    if (record_.field_count >= kMaxEventFields) return *this;
    EventField& field = record_.fields[record_.field_count++];
    field.name = name;
    field.value = EventValue::from_double(value);
    return *this;
}

EventLog::Builder& EventLog::Builder::field(const char* name, const std::string& value) {
    if (record_.field_count >= kMaxEventFields) return *this;
    EventField& field = record_.fields[record_.field_count++];
    field.name = name;
    field.value = EventValue::from_text(log_ ? log_->intern(value) : nullptr);
    return *this;
}

EventLog::Builder& EventLog::Builder::address(const char* name, u64 value) {
    if (record_.field_count >= kMaxEventFields) return *this;
    EventField& field = record_.fields[record_.field_count++];
    field.name = name;
    field.value = EventValue::from_address(value);
    return *this;
}

EventLog::Builder& EventLog::Builder::level(EventLevel level) {
    record_.level = level;
    return *this;
}

EventLog::Builder& EventLog::Builder::keyword(EventKeyword keyword) {
    record_.keyword = keyword;
    return *this;
}

EventLog::Builder& EventLog::Builder::task(u16 task) {
    record_.task = task;
    return *this;
}

EventLog::Builder& EventLog::Builder::opcode(EventOpcode opcode) {
    record_.opcode = opcode;
    return *this;
}

EventLog::Builder& EventLog::Builder::activity(u64 activity) {
    record_.activity = activity;
    return *this;
}

EventLog::Builder& EventLog::Builder::pop_on_emit(u64 activity) {
    pop_activity_ = activity;
    return *this;
}

EventLog::Builder& EventLog::Builder::push_on_emit(u64 activity) {
    push_activity_ = activity;
    return *this;
}

const EventRecord& EventLog::Builder::emit() {
    if (!emitted_) {
        if (log_) {
            log_->emit(record_);
            if (push_activity_ != 0) log_->activity_stack_.push_back(push_activity_);
            if (pop_activity_ != 0) log_->pop_activity(pop_activity_);
        }
        emitted_ = true;
    }
    return record_;
}

void EventLog::pop_activity(u64 activity) {
    while (!activity_stack_.empty()) {
        const u64 top = activity_stack_.back();
        activity_stack_.pop_back();
        if (top == activity) break;
    }
}

// -- Emission ---------------------------------------------------------------

EventLog::Builder EventLog::event(EventProvider provider, u16 id) {
    EventRecord record;
    record.provider = provider;
    record.id = id;
    const EventMetadata* metadata = event_metadata(provider, id);
    if (metadata) {
        record.version = metadata->version;
        record.level = metadata->level;
        record.opcode = metadata->opcode;
        record.task = metadata->task;
        record.keyword = metadata->keyword;
        record.name = metadata->name;
    } else {
        record.name = "Event";
    }
    return Builder(*this, record);
}

EventLog::Builder EventLog::begin_event(EventProvider provider, u16 id) {
    EventRecord record;
    record.provider = provider;
    record.id = id;
    record.opcode = EventOpcode::Start;
    record.activity = next_activity_++;
    record.parent_activity = current_activity();
    const EventMetadata* metadata = event_metadata(provider, id);
    if (metadata) {
        record.version = metadata->version;
        record.level = metadata->level;
        record.task = metadata->task;
        record.keyword = metadata->keyword;
        record.name = metadata->name;
    } else {
        record.name = "Activity";
    }
    return Builder(*this, record).push_on_emit(record.activity);
}

u64 EventLog::begin(EventProvider provider, u16 id) {
    Builder builder = begin_event(provider, id);
    const u64 activity = builder.record().activity;
    builder.emit();
    return activity;
}

void EventLog::end(u64 activity, EventProvider provider, u16 id) {
    EventRecord record;
    record.provider = provider;
    record.id = id;
    record.opcode = EventOpcode::Stop;
    record.activity = activity;
    const EventMetadata* metadata = event_metadata(provider, id);
    if (metadata) {
        record.version = metadata->version;
        record.level = metadata->level;
        record.task = metadata->task;
        record.keyword = metadata->keyword;
        record.name = metadata->name;
    } else {
        record.name = "ActivityEnd";
    }
    emit(record);
    pop_activity(activity);
}

EventLog::Builder EventLog::end_event(u64 activity, EventProvider provider, u16 id) {
    EventRecord record;
    record.provider = provider;
    record.id = id;
    record.opcode = EventOpcode::Stop;
    record.activity = activity;
    const EventMetadata* metadata = event_metadata(provider, id);
    if (metadata) {
        record.version = metadata->version;
        record.level = metadata->level;
        record.task = metadata->task;
        record.keyword = metadata->keyword;
        record.name = metadata->name;
    } else {
        record.name = "ActivityEnd";
    }
    return Builder(*this, record).pop_on_emit(activity);
}

// -- Queries ----------------------------------------------------------------

std::vector<EventRecord> EventLog::tail(size_t count) const {
    if (count > count_) count = count_;
    std::vector<EventRecord> out;
    out.reserve(count);
    for (size_t i = count; i > 0; --i) {
        const size_t index = (head_ + ring_.size() - i) % ring_.size();
        out.push_back(ring_[index]);
    }
    return out;
}

namespace {

std::string lower_copy(const std::string& text) {
    std::string out = text;
    for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return out;
}

}  // namespace

bool EventLog::match(const EventRecord& record, const EventFilter& filter) const {
    if (record.level > filter.max_level) return false;
    if (record.keyword != 0 && (record.keyword & filter.keyword) == 0) return false;
    const int provider_index = static_cast<int>(record.provider);
    if (provider_index < 0 || provider_index >= 64) return false;
    if ((filter.provider_mask & (1ull << provider_index)) == 0) return false;
    const int area_index = static_cast<int>(event_provider_info(record.provider).area);
    if (area_index < 0 || area_index >= kEventAreaCount) return false;
    if ((filter.area_mask & (1ull << area_index)) == 0) return false;
    if (record.time_ns < filter.time_lo_ns || record.time_ns > filter.time_hi_ns) return false;
    if (filter.activities_only && !record.is_activity()) return false;
    if (filter.one_shot_only && record.is_activity()) return false;
    if (filter.has_task && record.task != filter.task) return false;
    if (filter.has_opcode && record.opcode != filter.opcode) return false;
    if (filter.has_id && record.id != filter.id) return false;
    if (!filter.text.empty()) {
        std::string haystack = to_string(record.provider);
        haystack += ' ';
        haystack += record.name ? record.name : "";
        haystack += ' ';
        haystack += to_string(record.opcode);
        haystack += ' ';
        haystack += record.payload_text();
        if (lower_copy(haystack).find(lower_copy(filter.text)) == std::string::npos) return false;
    }
    return true;
}

std::vector<EventRecord> EventLog::query(const EventFilter& filter, size_t max_results) const {
    std::vector<EventRecord> out;
    for (size_t i = 0; i < count_; ++i) {
        const size_t index = (head_ + ring_.size() - count_ + i) % ring_.size();
        const EventRecord& record = ring_[index];
        if (!match(record, filter)) continue;
        out.push_back(record);
        if (max_results != 0 && out.size() >= max_results) break;
    }
    return out;
}

std::vector<EventRecord> EventLog::tail_matching(const EventFilter& filter, size_t count) const {
    std::vector<EventRecord> out;
    if (count == 0) return out;
    out.reserve(std::min(count, count_));
    for (size_t i = 0; i < count_; ++i) {
        const size_t index = (head_ + ring_.size() - 1 - i) % ring_.size();
        const EventRecord& record = ring_[index];
        if (!match(record, filter)) continue;
        out.push_back(record);
        if (out.size() >= count) break;
    }
    std::reverse(out.begin(), out.end());
    return out;
}

u64 EventLog::first_time() const {
    for (size_t i = 0; i < count_; ++i) {
        const size_t index = (head_ + ring_.size() - count_ + i) % ring_.size();
        return ring_[index].time_ns;
    }
    return 0;
}

u64 EventLog::last_time() const {
    if (count_ == 0) return 0;
    const size_t index = (head_ + ring_.size() - 1) % ring_.size();
    return ring_[index].time_ns;
}

bool EventLog::time_range(const EventFilter& filter, u64& lo, u64& hi) const {
    bool any = false;
    for (size_t i = 0; i < count_; ++i) {
        const size_t index = (head_ + ring_.size() - count_ + i) % ring_.size();
        const EventRecord& record = ring_[index];
        if (!match(record, filter)) continue;
        if (!any || record.time_ns < lo) lo = record.time_ns;
        if (!any || record.time_ns > hi) hi = record.time_ns;
        any = true;
    }
    return any;
}

const EventLog::ProviderCounters& EventLog::provider_counters(EventProvider provider) const {
    static const ProviderCounters empty;
    const int index = static_cast<int>(provider);
    if (index < 0 || index >= kEventProviderCount) return empty;
    return counters_[static_cast<size_t>(index)];
}

EventCounts EventLog::counts(const EventFilter& filter) const {
    EventCounts result;
    u64 lo = 0;
    u64 hi = 0;
    bool any = false;
    for (size_t i = 0; i < count_; ++i) {
        const size_t index = (head_ + ring_.size() - count_ + i) % ring_.size();
        const EventRecord& record = ring_[index];
        if (!match(record, filter)) continue;
        ++result.records;
        const int provider_index = static_cast<int>(record.provider);
        if (provider_index >= 0 && provider_index < kEventProviderCount) {
            ++result.providers[provider_index];
        }
        const int area_index = static_cast<int>(event_provider_info(record.provider).area);
        if (area_index >= 0 && area_index < kEventAreaCount) ++result.areas[area_index];
        const int level_index = static_cast<int>(record.level);
        if (level_index >= 0 && level_index < kEventLevelCount) ++result.levels[level_index];
        const int opcode_index = static_cast<int>(record.opcode);
        if (opcode_index >= 0 && opcode_index < 16) ++result.opcodes[opcode_index];
        if (!any || record.time_ns < lo) lo = record.time_ns;
        if (!any || record.time_ns > hi) hi = record.time_ns;
        any = true;
    }
    if (any && hi >= lo) result.total_span_ns = hi - lo;
    return result;
}

std::vector<EventActivitySpan> EventLog::activities(const EventFilter& filter, size_t max_results) const {
    std::vector<EventActivitySpan> spans;
    std::map<u64, size_t> open;
    for (size_t i = 0; i < count_; ++i) {
        const size_t index = (head_ + ring_.size() - count_ + i) % ring_.size();
        const EventRecord& record = ring_[index];
        if (!match(record, filter)) continue;
        if (is_activity_start(record.opcode)) {
            EventActivitySpan span;
            span.provider = record.provider;
            span.id = record.id;
            span.task = record.task;
            span.activity = record.activity;
            span.parent = record.parent_activity;
            span.begin_ns = record.time_ns;
            span.end_ns = record.time_ns;
            span.level = record.level;
            span.keyword = record.keyword;
            span.name = record.name;
            open[record.activity] = spans.size();
            spans.push_back(span);
        } else if (is_activity_stop(record.opcode)) {
            auto iter = open.find(record.activity);
            if (iter == open.end()) continue;
            EventActivitySpan& span = spans[iter->second];
            span.end_ns = record.time_ns;
            span.duration_ns = record.time_ns >= span.begin_ns ? record.time_ns - span.begin_ns : 0;
            open.erase(iter);
        }
    }
    for (const auto& entry : open) {
        EventActivitySpan& span = spans[entry.second];
        span.open = true;
        span.duration_ns = span.end_ns >= span.begin_ns ? span.end_ns - span.begin_ns : 0;
    }

    // Keep at most max_results spans, sampled evenly so a long trace still shows
    // its shape instead of only its oldest events.
    if (max_results != 0 && spans.size() > max_results) {
        std::vector<EventActivitySpan> sampled;
        sampled.reserve(max_results);
        const double step = static_cast<double>(spans.size()) / static_cast<double>(max_results);
        for (size_t i = 0; i < max_results; ++i) {
            const size_t index = static_cast<size_t>(static_cast<double>(i) * step);
            sampled.push_back(spans[std::min(index, spans.size() - 1)]);
        }
        return sampled;
    }
    return spans;
}

std::vector<EventSummaryRow> EventLog::summary(const EventFilter& filter, size_t max_results) const {
    using Key = std::tuple<int, u16, u8, u16>;
    struct Accumulator {
        EventSummaryRow row;
        u64 min_ns = 0;
        u64 max_ns = 0;
    };
    std::map<Key, Accumulator> groups;

    for (size_t i = 0; i < count_; ++i) {
        const size_t index = (head_ + ring_.size() - count_ + i) % ring_.size();
        const EventRecord& record = ring_[index];
        if (!match(record, filter)) continue;
        if (record.is_activity()) continue;  // activities are counted through their span
        const Key key{static_cast<int>(record.provider), record.task, static_cast<u8>(record.opcode), record.id};
        Accumulator& acc = groups[key];
        acc.row.provider = record.provider;
        acc.row.id = record.id;
        acc.row.task = record.task;
        acc.row.opcode = record.opcode;
        acc.row.name = record.name;
        acc.row.count += 1;
        if (record.level < acc.row.level) acc.row.level = record.level;
    }

    const std::vector<EventActivitySpan> spans = activities(filter);
    u64 total_weight = 0;
    for (const EventActivitySpan& span : spans) {
        const Key key{static_cast<int>(span.provider), span.task, static_cast<u8>(EventOpcode::Start), span.id};
        Accumulator& acc = groups[key];
        acc.row.provider = span.provider;
        acc.row.id = span.id;
        acc.row.task = span.task;
        acc.row.opcode = EventOpcode::Start;
        acc.row.name = span.name;
        acc.row.count += 1;
        acc.row.weight_ns += span.duration_ns;
        if (span.level < acc.row.level) acc.row.level = span.level;
        if (acc.min_ns == 0 || span.duration_ns < acc.min_ns) acc.min_ns = span.duration_ns;
        if (span.duration_ns > acc.max_ns) acc.max_ns = span.duration_ns;
        total_weight += span.duration_ns;
    }

    std::vector<EventSummaryRow> rows;
    rows.reserve(groups.size());
    for (auto& entry : groups) {
        entry.second.row.min_ns = entry.second.min_ns;
        entry.second.row.max_ns = entry.second.max_ns;
        entry.second.row.percent = total_weight != 0
                                       ? 100.0 * static_cast<double>(entry.second.row.weight_ns) /
                                             static_cast<double>(total_weight)
                                       : 0.0;
        rows.push_back(entry.second.row);
    }
    std::sort(rows.begin(), rows.end(), [](const EventSummaryRow& a, const EventSummaryRow& b) {
        if (a.weight_ns != b.weight_ns) return a.weight_ns > b.weight_ns;
        if (a.count != b.count) return a.count > b.count;
        if (a.provider != b.provider) return static_cast<int>(a.provider) < static_cast<int>(b.provider);
        return a.id < b.id;
    });
    if (max_results != 0 && rows.size() > max_results) rows.resize(max_results);
    return rows;
}

size_t EventLog::bucket_of(u64 time, u64 lo, u64 hi, int buckets) const {
    if (buckets <= 0) return 0;
    if (hi <= lo) return 0;
    if (time <= lo) return 0;
    if (time >= hi) return static_cast<size_t>(buckets - 1);
    const u64 span = hi - lo + 1;
    const size_t index = static_cast<size_t>((time - lo) * static_cast<u64>(buckets) / span);
    return std::min(index, static_cast<size_t>(buckets - 1));
}

std::vector<EventTimelineBucket> EventLog::timeline(const EventFilter& filter, int buckets) const {
    std::vector<EventTimelineBucket> result;
    if (buckets <= 0) return result;
    u64 lo = 0;
    u64 hi = 0;
    if (!time_range(filter, lo, hi)) return result;
    if (hi <= lo) hi = lo + 1;

    result.resize(static_cast<size_t>(buckets));
    const u64 span = hi - lo + 1;
    for (int b = 0; b < buckets; ++b) {
        EventTimelineBucket& bucket = result[static_cast<size_t>(b)];
        bucket.time_ns = lo + span * static_cast<u64>(b) / static_cast<u64>(buckets);
        bucket.end_ns = lo + span * static_cast<u64>(b + 1) / static_cast<u64>(buckets);
    }

    for (size_t i = 0; i < count_; ++i) {
        const size_t index = (head_ + ring_.size() - count_ + i) % ring_.size();
        const EventRecord& record = ring_[index];
        if (!match(record, filter)) continue;
        if (is_activity_stop(record.opcode)) continue;  // weight comes from the span
        const int area_index = static_cast<int>(event_provider_info(record.provider).area);
        if (area_index < 0 || area_index >= kEventAreaCount) continue;
        const size_t b = bucket_of(record.time_ns, lo, hi, buckets);
        ++result[b].counts[area_index];
        ++result[b].total;
    }

    for (const EventActivitySpan& activity : activities(filter)) {
        const int area_index = static_cast<int>(event_provider_info(activity.provider).area);
        if (area_index < 0 || area_index >= kEventAreaCount) continue;
        const size_t first = bucket_of(activity.begin_ns, lo, hi, buckets);
        const size_t last = bucket_of(activity.end_ns, lo, hi, buckets);
        if (first == last || activity.duration_ns == 0) {
            result[first].weight_ns[area_index] += activity.duration_ns;
            continue;
        }
        for (size_t b = first; b <= last; ++b) {
            const u64 bucket_lo = result[b].time_ns;
            const u64 bucket_hi = result[b].end_ns;
            const u64 overlap_lo = std::max(bucket_lo, activity.begin_ns);
            const u64 overlap_hi = std::min(bucket_hi, activity.end_ns);
            if (overlap_hi <= overlap_lo) continue;
            // Split the duration proportionally to the overlap, so a long stage
            // shows up as a ramp rather than a single full-height bar.
            const u64 overlap = overlap_hi - overlap_lo;
            const u64 share = activity.duration_ns * overlap / std::max<u64>(1, activity.end_ns - activity.begin_ns);
            result[b].weight_ns[area_index] += share;
        }
    }
    return result;
}

// -- Formatting and export --------------------------------------------------

std::string EventLog::format_record(const EventRecord& record, bool with_payload) const {
    std::string line = format("%6llu  %12.6f  %-14s %-4u %-18s %-8s", static_cast<unsigned long long>(record.sequence),
                              static_cast<double>(record.time_ns) / 1.0e9, to_string(record.provider), record.task,
                              record.name ? record.name : event_name(record.provider, record.id),
                              to_string(record.opcode));
    if (record.activity != 0) {
        line += format("  a=%llu", static_cast<unsigned long long>(record.activity));
    }
    if (with_payload) {
        const std::string payload = record.payload_text();
        if (!payload.empty()) {
            line += "  ";
            line += payload;
        }
    }
    return line;
}

std::string EventLog::format_table(const std::vector<EventRecord>& records) const {
    std::string text = format("%6s  %12s  %-14s %-4s %-18s %-8s  %s", "#", "time (s)", "provider", "task", "event",
                              "opcode", "payload");
    text += '\n';
    for (const EventRecord& record : records) {
        text += format_record(record);
        text += '\n';
    }
    return text;
}

std::string EventLog::export_csv(const EventFilter& filter) const {
    std::string text;
    text += "sequence,time_ns,time_s,provider,provider_guid,area,task,event_id,event,opcode,level,activity,parent,"
            "payload\n";
    for (size_t i = 0; i < count_; ++i) {
        const size_t index = (head_ + ring_.size() - count_ + i) % ring_.size();
        const EventRecord& record = ring_[index];
        if (!match(record, filter)) continue;
        const EventProviderInfo& info = event_provider_info(record.provider);
        text += format("%llu,%llu,%.6f,%s,%s,%s,%u,%u,%s,%s,%s,%llu,%llu,", static_cast<unsigned long long>(record.sequence),
                       static_cast<unsigned long long>(record.time_ns), static_cast<double>(record.time_ns) / 1.0e9,
                       info.name, info.guid, to_string(info.area), record.task, record.id,
                       record.name ? record.name : "", to_string(record.opcode), to_string(record.level),
                       static_cast<unsigned long long>(record.activity),
                       static_cast<unsigned long long>(record.parent_activity));
        std::string payload = record.payload_text();
        for (char& c : payload) {
            if (c == '"') c = '\'';
            else if (c == ',') c = ';';
        }
        if (!payload.empty() && payload.front() == ' ') payload.erase(payload.begin());
        text += '"';
        text += payload;
        text += "\"\n";
    }
    return text;
}

bool EventLog::save_csv(const std::string& path, const EventFilter& filter, std::string& error) const {
    const std::string text = export_csv(filter);
    if (!write_file(path, text.data(), text.size())) {
        error = "cannot write " + path;
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Active log
// ---------------------------------------------------------------------------

namespace {

EventLog& fallback_log() {
    static EventLog log(256);
    static const bool initialized = [] {
        log.set_recording(false);
        return true;
    }();
    (void)initialized;
    return log;
}

EventLog* g_active_events = nullptr;

}  // namespace

EventLog& events() { return g_active_events ? *g_active_events : fallback_log(); }

EventLog* active_events() { return g_active_events; }

void set_active_events(EventLog* log) { g_active_events = log; }

}  // namespace zlb
