// zeliboba - the event tracing system (ETW-style) tests.
//
// The core is exercised on a private EventLog with a controllable clock; the
// machine smoke tests at the end check that the instrumentation in the boot
// chain, the cores and the bus really produces events for a real Vita.
#include <memory>
#include <string>
#include <vector>

#include "common/types.h"
#include "common/util.h"
#include "debug/debugger.h"
#include "event/event.h"
#include "event/providers.h"
#include "machine/vita.h"
#include "test_framework.h"

using namespace zlb;

namespace {

/// A private log with a deterministic clock, so durations and buckets are exact.
/// EventLog is not copyable, so tests hold the fixture and take a reference.
struct TestLog {
    struct Clock {
        u64 ns = 0;
        u64 advance(u64 delta) { return ns += delta; }
    };

    Clock clock;
    EventLog log;

    explicit TestLog(size_t capacity = 64) : log(capacity) {
        log.set_clock([this] { return clock.ns; });
    }
    EventLog& operator*() { return log; }
    EventLog* operator->() { return &log; }
};

}  // namespace

// ---------------------------------------------------------------------------
// Manifest and parsing
// ---------------------------------------------------------------------------

ZLB_TEST(event_manifest_describes_every_provider) {
    for (int i = 0; i < kEventProviderCount; ++i) {
        const EventProvider provider = static_cast<EventProvider>(i);
        const EventProviderInfo& info = event_provider_info(provider);
        ZLB_EXPECT_EQ(static_cast<int>(info.id), i);
        ZLB_EXPECT_TRUE(info.name != nullptr && info.name[0] != '\0');
        ZLB_EXPECT_TRUE(info.guid != nullptr && info.guid[0] != '\0');
        ZLB_EXPECT_TRUE(static_cast<int>(info.area) < kEventAreaCount);
        ZLB_EXPECT_TRUE(info.events != nullptr);
        ZLB_EXPECT_TRUE(info.event_count > 0);
        for (size_t a = 0; a < info.event_count; ++a) {
            const EventMetadata& event = info.events[a];
            ZLB_EXPECT_TRUE(event.id != 0);
            ZLB_EXPECT_TRUE(event.name != nullptr && event.name[0] != '\0');
            ZLB_EXPECT_TRUE(static_cast<int>(event.level) <= static_cast<int>(EventLevel::Verbose));
            for (size_t b = a + 1; b < info.event_count; ++b) {
                ZLB_EXPECT_TRUE(info.events[b].id != event.id);
            }
        }
    }
    ZLB_EXPECT_TRUE(event_metadata(EventProvider::Boot, ev::boot::kStageBegin) != nullptr);
    ZLB_EXPECT_TRUE(event_metadata(EventProvider::Boot, 0x7FFF) == nullptr);
    ZLB_EXPECT_TRUE(std::string(event_name(EventProvider::Boot, ev::boot::kMilestone)) == "Milestone");
}

ZLB_TEST(event_level_and_keyword_parsing) {
    EventLevel level = EventLevel::LogAlways;
    ZLB_EXPECT_TRUE(parse_event_level("info", level));
    ZLB_EXPECT_TRUE(level == EventLevel::Informational);
    ZLB_EXPECT_TRUE(parse_event_level("VERBOSE", level));
    ZLB_EXPECT_TRUE(level == EventLevel::Verbose);
    ZLB_EXPECT_TRUE(parse_event_level("warn", level));
    ZLB_EXPECT_TRUE(level == EventLevel::Warning);
    ZLB_EXPECT_FALSE(parse_event_level("nonsense", level));

    EventKeyword keyword = 0;
    ZLB_EXPECT_TRUE(parse_event_keyword("all", keyword));
    ZLB_EXPECT_EQ(keyword, event_keyword::kAll);
    ZLB_EXPECT_TRUE(parse_event_keyword("default", keyword));
    ZLB_EXPECT_EQ(keyword, event_keyword::kDefault);
    ZLB_EXPECT_TRUE(parse_event_keyword("boot|storage", keyword));
    ZLB_EXPECT_EQ(keyword, event_keyword::kBoot | event_keyword::kStorage);
    ZLB_EXPECT_TRUE(parse_event_keyword("0x40", keyword));
    ZLB_EXPECT_EQ(keyword, 0x40u);
    ZLB_EXPECT_FALSE(parse_event_keyword("bogus-bit", keyword));
    ZLB_EXPECT_TRUE(describe_event_keywords(event_keyword::kAll) == "all");
    ZLB_EXPECT_TRUE(describe_event_keywords(event_keyword::kBoot).find("boot") != std::string::npos);

    EventProvider provider = EventProvider::Test;
    ZLB_EXPECT_TRUE(parse_event_provider("boot", provider));
    ZLB_EXPECT_TRUE(provider == EventProvider::Boot);
    ZLB_EXPECT_TRUE(parse_event_provider("Zeliboba-eMMC", provider));
    ZLB_EXPECT_TRUE(provider == EventProvider::Emmc);
    ZLB_EXPECT_FALSE(parse_event_provider("nope", provider));

    EventArea area = EventArea::Other;
    ZLB_EXPECT_TRUE(parse_event_area("storage", area));
    ZLB_EXPECT_TRUE(area == EventArea::Storage);
    ZLB_EXPECT_TRUE(parse_event_area("Video", area));
    ZLB_EXPECT_TRUE(area == EventArea::Video);
}

// ---------------------------------------------------------------------------
// Emission and filtering
// ---------------------------------------------------------------------------

ZLB_TEST(event_one_shot_carries_fields_and_time) {
    TestLog fixture;
    EventLog& log = fixture.log;
    fixture.clock.advance(1500);

    log.event(EventProvider::Boot, ev::boot::kMilestone)
        .field("text", std::string("first loader entered"))
        .field("stage", static_cast<u64>(3))
        .emit();

    std::vector<EventRecord> records = log.all();
    ZLB_EXPECT_EQ(records.size(), static_cast<size_t>(1));
    if (records.size() != 1) return;
    const EventRecord& record = records[0];
    ZLB_EXPECT_EQ(record.time_ns, 1500u);
    ZLB_EXPECT_EQ(record.sequence, 1u);
    ZLB_EXPECT_TRUE(record.provider == EventProvider::Boot);
    ZLB_EXPECT_EQ(static_cast<int>(record.level), static_cast<int>(EventLevel::Informational));
    ZLB_EXPECT_TRUE(record.name != nullptr && std::string(record.name) == "Milestone");
    ZLB_EXPECT_EQ(record.field_count, static_cast<u8>(2));
    ZLB_EXPECT_TRUE(record.find("text") != nullptr);
    ZLB_EXPECT_TRUE(record.find("stage") != nullptr);
    ZLB_EXPECT_TRUE(record.find("missing") == nullptr);
    ZLB_EXPECT_TRUE(record.find("stage")->value.to_string() == "3");
    ZLB_EXPECT_TRUE(record.payload_text().find("first loader entered") != std::string::npos);
    ZLB_EXPECT_EQ(log.total(), 1u);
    ZLB_EXPECT_EQ(log.count(), static_cast<size_t>(1));
}

ZLB_TEST(event_record_has_metadata_defaults) {
    TestLog fixture;
    EventLog& log = fixture.log;
    log.event(EventProvider::Emmc, ev::emmc::kTransferBegin).emit();
    const EventRecord record = log.all().at(0);
    ZLB_EXPECT_TRUE(record.opcode == EventOpcode::Start);
    ZLB_EXPECT_EQ(record.keyword, event_keyword::kStorage);
    ZLB_EXPECT_TRUE(record.level == EventLevel::Informational);
    ZLB_EXPECT_TRUE(std::string(record.name) == "Transfer");
    ZLB_EXPECT_TRUE(record.task != 0);
}

ZLB_TEST(event_session_level_filter_drops_verbose) {
    TestLog fixture;
    EventLog& log = fixture.log;
    log.set_max_level(EventLevel::Informational);
    log.event(EventProvider::Crypto, ev::crypto::kSha256Begin).field("bytes", static_cast<u64>(64)).emit();
    log.event(EventProvider::Machine, ev::machine::kReset).field("cold", true).emit();
    ZLB_EXPECT_EQ(log.total(), 1u);
    ZLB_EXPECT_EQ(log.filtered(), 1u);

    log.set_max_level(EventLevel::Verbose);
    log.event(EventProvider::Crypto, ev::crypto::kSha256Begin).field("bytes", static_cast<u64>(64)).emit();
    ZLB_EXPECT_EQ(log.total(), 2u);
}

ZLB_TEST(event_session_keyword_and_provider_filter) {
    TestLog fixture;
    EventLog& log = fixture.log;
    log.set_keyword_mask(event_keyword::kBoot);
    log.set_all_providers(false);
    log.set_provider_enabled(EventProvider::Boot, true);
    log.set_provider_enabled(EventProvider::Emmc, true);

    log.event(EventProvider::Boot, ev::boot::kMilestone).field("text", std::string("kept")).emit();
    log.event(EventProvider::Emmc, ev::emmc::kAttach).field("path", std::string("x.img")).emit();
    log.event(EventProvider::Syscon, ev::syscon::kPowerState).field("to", static_cast<u64>(1)).emit();

    ZLB_EXPECT_EQ(log.total(), 1u);
    ZLB_EXPECT_TRUE(log.all().at(0).payload_text().find("kept") != std::string::npos);

    // A keyword of 0 (the Test provider) is not gated by the keyword mask.
    log.set_provider_enabled(EventProvider::Test, true);
    log.event(EventProvider::Test, ev::test::kProbe).field("name", std::string("probe")).emit();
    ZLB_EXPECT_EQ(log.total(), 2u);
}

ZLB_TEST(event_ring_buffer_wraps_and_keeps_counters) {
    TestLog fixture(4);
    EventLog& log = fixture.log;
    for (int i = 0; i < 10; ++i) {
        fixture.clock.advance(100);
        log.event(EventProvider::Test, ev::test::kProbe).field("value", static_cast<u64>(i)).emit();
    }
    ZLB_EXPECT_EQ(log.total(), 10u);
    ZLB_EXPECT_EQ(log.count(), static_cast<size_t>(4));
    std::vector<EventRecord> records = log.all();
    ZLB_EXPECT_EQ(records.size(), static_cast<size_t>(4));
    ZLB_EXPECT_TRUE(records[0].find("value")->value.to_string() == "6");
    ZLB_EXPECT_TRUE(records[3].find("value")->value.to_string() == "9");
    ZLB_EXPECT_EQ(records[3].time_ns, 1000u);
    ZLB_EXPECT_EQ(log.provider_counters(EventProvider::Test).emitted, 10u);
    ZLB_EXPECT_EQ(log.first_time(), 700u);
    ZLB_EXPECT_EQ(log.last_time(), 1000u);
}

// ---------------------------------------------------------------------------
// Activities
// ---------------------------------------------------------------------------

ZLB_TEST(event_activity_pairs_begin_and_end) {
    TestLog fixture;
    EventLog& log = fixture.log;
    const u64 activity = log.begin(EventProvider::Emmc, ev::emmc::kTransferBegin);
    ZLB_EXPECT_EQ(activity, 1u);
    ZLB_EXPECT_EQ(log.current_activity(), 1u);
    ZLB_EXPECT_EQ(log.activity_depth(), static_cast<size_t>(1));
    fixture.clock.advance(2000);
    log.end(activity, EventProvider::Emmc, ev::emmc::kTransferEnd);
    ZLB_EXPECT_EQ(log.activity_depth(), static_cast<size_t>(0));

    std::vector<EventActivitySpan> spans = log.activities(EventFilter{});
    ZLB_EXPECT_EQ(spans.size(), static_cast<size_t>(1));
    if (spans.empty()) return;
    ZLB_EXPECT_EQ(spans[0].duration_ns, 2000u);
    ZLB_EXPECT_FALSE(spans[0].open);
    ZLB_EXPECT_TRUE(spans[0].provider == EventProvider::Emmc);
    ZLB_EXPECT_EQ(log.provider_counters(EventProvider::Emmc).total_ns, 2000u);
    ZLB_EXPECT_EQ(log.provider_counters(EventProvider::Emmc).open, 0u);
}

ZLB_TEST(event_activities_nest_with_parents) {
    TestLog fixture;
    EventLog& log = fixture.log;
    log.set_max_level(EventLevel::Verbose);  // the inner SHA-256 activity is Verbose
    const u64 outer = log.begin(EventProvider::Boot, ev::boot::kStageBegin);
    fixture.clock.advance(100);
    const u64 inner = log.begin(EventProvider::Crypto, ev::crypto::kSha256Begin);
    fixture.clock.advance(100);
    log.end(inner, EventProvider::Crypto, ev::crypto::kSha256End);
    fixture.clock.advance(100);
    log.end(outer, EventProvider::Boot, ev::boot::kStageEnd);
    ZLB_EXPECT_EQ(log.activity_depth(), static_cast<size_t>(0));

    std::vector<EventActivitySpan> spans = log.activities(EventFilter{});
    ZLB_EXPECT_EQ(spans.size(), static_cast<size_t>(2));
    if (spans.size() != 2) return;
    ZLB_EXPECT_EQ(spans[0].activity, outer);
    ZLB_EXPECT_EQ(spans[0].parent, 0u);
    ZLB_EXPECT_EQ(spans[0].duration_ns, 300u);
    ZLB_EXPECT_EQ(spans[1].activity, inner);
    ZLB_EXPECT_EQ(spans[1].parent, outer);
    ZLB_EXPECT_EQ(spans[1].duration_ns, 100u);
}

ZLB_TEST(event_open_activity_is_reported) {
    TestLog fixture;
    EventLog& log = fixture.log;
    log.begin(EventProvider::Dma, ev::dma::kTransferBegin);
    fixture.clock.advance(500);
    std::vector<EventActivitySpan> spans = log.activities(EventFilter{});
    ZLB_EXPECT_EQ(spans.size(), static_cast<size_t>(1));
    if (spans.empty()) return;
    ZLB_EXPECT_TRUE(spans[0].open);
    ZLB_EXPECT_EQ(log.provider_counters(EventProvider::Dma).open, 1u);
    log.reset_activities();
    ZLB_EXPECT_EQ(log.activity_depth(), static_cast<size_t>(0));
}

ZLB_TEST(event_raii_activity_closes_on_scope_exit) {
    TestLog fixture;
    EventLog& log = fixture.log;
    {
        EventActivity activity(log, EventProvider::Loader, ev::loader::kLoadBegin);
        ZLB_EXPECT_TRUE(activity.open());
        fixture.clock.advance(750);
    }
    ZLB_EXPECT_EQ(log.activity_depth(), static_cast<size_t>(0));
    std::vector<EventActivitySpan> spans = log.activities(EventFilter{});
    ZLB_EXPECT_EQ(spans.size(), static_cast<size_t>(1));
    if (!spans.empty()) ZLB_EXPECT_EQ(spans[0].duration_ns, 750u);
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

ZLB_TEST(event_query_filters_by_time_text_and_kind) {
    TestLog fixture;
    EventLog& log = fixture.log;
    fixture.clock.advance(100);
    log.event(EventProvider::Loader, ev::loader::kVerify).field("name", std::string("kprx_auth_sm.self")).emit();
    fixture.clock.advance(100);
    log.begin(EventProvider::Loader, ev::loader::kLoadBegin);
    fixture.clock.advance(100);
    log.event(EventProvider::Loader, ev::loader::kError).field("name", std::string("broken.enp")).emit();
    log.end(1, EventProvider::Loader, ev::loader::kLoadEnd);
    fixture.clock.advance(100);
    log.event(EventProvider::Emmc, ev::emmc::kAttach).field("path", std::string("emmc.img")).emit();

    EventFilter filter;
    filter.text = "kprx";
    ZLB_EXPECT_EQ(log.query(filter).size(), static_cast<size_t>(1));

    filter = EventFilter{};
    filter.one_shot_only = true;
    ZLB_EXPECT_EQ(log.query(filter).size(), static_cast<size_t>(3));

    filter = EventFilter{};
    filter.activities_only = true;
    ZLB_EXPECT_EQ(log.query(filter).size(), static_cast<size_t>(2));

    filter = EventFilter{};
    filter.time_lo_ns = 150;
    filter.time_hi_ns = 250;
    ZLB_EXPECT_EQ(log.query(filter).size(), static_cast<size_t>(1));

    filter = EventFilter{};
    filter.clear_providers();
    filter.set_provider(EventProvider::Emmc, true);
    filter.has_task = true;
    filter.task = event_metadata(EventProvider::Emmc, ev::emmc::kAttach)->task;
    ZLB_EXPECT_EQ(log.query(filter).size(), static_cast<size_t>(1));

    filter = EventFilter{};
    filter.set_provider(EventProvider::Loader, false);
    ZLB_EXPECT_EQ(log.query(filter).size(), static_cast<size_t>(1));

    filter = EventFilter{};
    filter.set_area(EventArea::Storage, false);
    ZLB_EXPECT_EQ(log.query(filter).size(), static_cast<size_t>(4));
}

ZLB_TEST(event_summary_weights_activities) {
    TestLog fixture;
    EventLog& log = fixture.log;
    const u64 first = log.begin(EventProvider::Emmc, ev::emmc::kTransferBegin);
    fixture.clock.advance(1000);
    log.end(first, EventProvider::Emmc, ev::emmc::kTransferEnd);
    const u64 second = log.begin(EventProvider::Emmc, ev::emmc::kTransferBegin);
    fixture.clock.advance(3000);
    log.end(second, EventProvider::Emmc, ev::emmc::kTransferEnd);
    fixture.clock.advance(100);
    log.event(EventProvider::Emmc, ev::emmc::kAttach).field("path", std::string("emmc.img")).emit();

    std::vector<EventSummaryRow> rows = log.summary(EventFilter{});
    ZLB_EXPECT_TRUE(!rows.empty());
    if (rows.empty()) return;
    const EventSummaryRow& top = rows[0];
    ZLB_EXPECT_TRUE(top.provider == EventProvider::Emmc);
    ZLB_EXPECT_EQ(top.count, 2u);
    ZLB_EXPECT_EQ(top.weight_ns, 4000u);
    ZLB_EXPECT_EQ(top.min_ns, 1000u);
    ZLB_EXPECT_EQ(top.max_ns, 3000u);
    ZLB_EXPECT_NEAR(top.percent, 100.0, 0.001);
    bool found_attach = false;
    for (const EventSummaryRow& row : rows) {
        if (row.name && std::string(row.name) == "Attach") {
            found_attach = true;
            ZLB_EXPECT_EQ(row.count, 1u);
            ZLB_EXPECT_EQ(row.weight_ns, 0u);
        }
    }
    ZLB_EXPECT_TRUE(found_attach);
}

ZLB_TEST(event_counts_and_timeline) {
    TestLog fixture;
    EventLog& log = fixture.log;
    log.set_max_level(EventLevel::Verbose);  // store everything, then filter on read
    for (int i = 0; i < 10; ++i) {
        fixture.clock.advance(1000);
        if (i % 2 == 0) {
            log.event(EventProvider::Interrupt, ev::interrupt::kRaise).field("line", static_cast<u64>(i)).emit();
        } else {
            log.event(EventProvider::Emmc, ev::emmc::kCommand).field("cmd", static_cast<u64>(i)).emit();
        }
    }
    ZLB_EXPECT_EQ(log.total(), 10u);
    ZLB_EXPECT_EQ(log.filtered(), 0u);

    // The read-side filter is independent of what the session recorded.
    EventFilter filter;
    filter.max_level = EventLevel::Informational;
    const EventCounts quiet = log.counts(filter);
    ZLB_EXPECT_EQ(quiet.records, 5u);
    ZLB_EXPECT_EQ(quiet.providers[static_cast<int>(EventProvider::Interrupt)], 5u);
    ZLB_EXPECT_EQ(quiet.providers[static_cast<int>(EventProvider::Emmc)], 0u);

    filter.max_level = EventLevel::Verbose;
    const EventCounts all = log.counts(filter);
    ZLB_EXPECT_EQ(all.records, 10u);
    ZLB_EXPECT_EQ(all.areas[static_cast<int>(EventArea::Computation)], 5u);
    ZLB_EXPECT_EQ(all.areas[static_cast<int>(EventArea::Storage)], 5u);

    std::vector<EventTimelineBucket> buckets = log.timeline(filter, 5);
    ZLB_EXPECT_EQ(buckets.size(), static_cast<size_t>(5));
    u64 total = 0;
    for (const EventTimelineBucket& bucket : buckets) total += bucket.total;
    ZLB_EXPECT_EQ(total, 10u);
    ZLB_EXPECT_EQ(buckets[0].counts[static_cast<int>(EventArea::Computation)], 1u);
    ZLB_EXPECT_EQ(buckets[0].counts[static_cast<int>(EventArea::Storage)], 1u);
}

ZLB_TEST(event_csv_export_has_one_row_per_record) {
    TestLog fixture;
    EventLog& log = fixture.log;
    log.event(EventProvider::Boot, ev::boot::kMilestone).field("text", std::string("hello, world")).emit();
    fixture.clock.advance(500);
    const u64 activity = log.begin(EventProvider::Emmc, ev::emmc::kTransferBegin);
    fixture.clock.advance(500);
    log.end(activity, EventProvider::Emmc, ev::emmc::kTransferEnd);

    std::string csv = log.export_csv(EventFilter{});
    const std::vector<std::string> lines = split(csv, '\n');
    ZLB_EXPECT_TRUE(lines.size() >= 4);
    ZLB_EXPECT_TRUE(lines[0].find("sequence,time_ns") == 0);
    ZLB_EXPECT_TRUE(csv.find("Zeliboba-Boot") != std::string::npos);
    ZLB_EXPECT_TRUE(csv.find("hello; world") != std::string::npos);
}

ZLB_TEST(event_clear_resets_records_but_keeps_config) {
    TestLog fixture;
    EventLog& log = fixture.log;
    log.set_max_level(EventLevel::Warning);
    log.event(EventProvider::Boot, ev::boot::kFailure).field("reason", std::string("x")).emit();
    ZLB_EXPECT_EQ(log.total(), 1u);
    log.clear();
    ZLB_EXPECT_EQ(log.total(), 0u);
    ZLB_EXPECT_EQ(log.count(), static_cast<size_t>(0));
    ZLB_EXPECT_TRUE(log.max_level() == EventLevel::Warning);
    ZLB_EXPECT_EQ(log.provider_counters(EventProvider::Boot).emitted, 0u);
    log.reset();
    ZLB_EXPECT_EQ(log.sequence(), 0u);
}

ZLB_TEST(event_tail_matching_returns_the_newest_records) {
    TestLog fixture;
    EventLog& log = fixture.log;
    for (int i = 0; i < 20; ++i) {
        fixture.clock.advance(10);
        if (i % 2 == 0) {
            log.event(EventProvider::Test, ev::test::kProbe).field("value", static_cast<u64>(i)).emit();
        } else {
            log.event(EventProvider::Emmc, ev::emmc::kAttach).field("path", std::string("x")).emit();
        }
    }
    std::vector<EventRecord> newest = log.tail_matching(EventFilter{}, 5);
    ZLB_EXPECT_EQ(newest.size(), static_cast<size_t>(5));
    if (newest.size() == 5) {
        // Oldest first inside the returned window, and the window is the tail.
        ZLB_EXPECT_TRUE(newest.front().sequence < newest.back().sequence);
        ZLB_EXPECT_EQ(newest.back().sequence, 20u);
        ZLB_EXPECT_EQ(newest.front().sequence, 16u);
    }
    EventFilter test_only;
    test_only.set_provider(EventProvider::Emmc, false);
    const std::vector<EventRecord> probes = log.tail_matching(test_only, 3);
    ZLB_EXPECT_EQ(probes.size(), static_cast<size_t>(3));
    for (const EventRecord& record : probes) ZLB_EXPECT_TRUE(record.provider == EventProvider::Test);
}

ZLB_TEST(event_begin_event_carries_fields_on_the_start_record) {
    TestLog fixture;
    EventLog& log = fixture.log;
    EventLog::Builder begin = log.begin_event(EventProvider::Dma, ev::dma::kTransferBegin);
    begin.field("channel", static_cast<u64>(2)).field("bytes", static_cast<u64>(4096));
    const u64 activity = begin.emit().activity;
    ZLB_EXPECT_TRUE(activity != 0);
    ZLB_EXPECT_EQ(log.current_activity(), activity);
    fixture.clock.advance(300);
    log.end(activity, EventProvider::Dma, ev::dma::kTransferEnd);

    std::vector<EventRecord> records = log.all();
    ZLB_EXPECT_EQ(records.size(), static_cast<size_t>(2));
    if (records.size() != 2) return;
    ZLB_EXPECT_TRUE(records[0].opcode == EventOpcode::Start);
    ZLB_EXPECT_TRUE(records[0].find("bytes") != nullptr);
    ZLB_EXPECT_TRUE(records[0].find("bytes")->value.to_string() == "4096");
    ZLB_EXPECT_EQ(records[1].activity, activity);
    ZLB_EXPECT_EQ(log.provider_counters(EventProvider::Dma).total_ns, 300u);
}

// ---------------------------------------------------------------------------
// Debugger commands and the machine instrumentation
// ---------------------------------------------------------------------------

namespace {

/// One machine for the event command tests. Like the other debugger tests it
/// builds without an eMMC image: the board, the cores and the boot chain still
/// emit their events.
Vita& events_test_vita() {
    static std::unique_ptr<Vita> vita = [] {
        auto machine = std::make_unique<Vita>();
        VitaConfig config;
        config.rebuild_emmc = false;
        machine->build(config);
        machine->reset(true);
        return machine;
    }();
    return *vita;
}

Debugger& events_test_debugger() {
    static Debugger debugger(events_test_vita());
    return debugger;
}

}  // namespace

ZLB_TEST(event_debugger_commands_control_the_session) {
    Debugger& debugger = events_test_debugger();
    EventLog& log = debugger.event_log();
    log.set_recording(true);
    log.set_all_providers(true);
    log.set_max_level(EventLevel::Informational);
    log.set_keyword_mask(event_keyword::kDefault);

    std::string output;
    debugger.set_output([&output](const std::string& text) { output = text; });

    debugger.execute("event");
    ZLB_EXPECT_TRUE(output.find("session") != std::string::npos);

    debugger.execute("event level verbose");
    ZLB_EXPECT_TRUE(output.find("Verbose") != std::string::npos);
    ZLB_EXPECT_TRUE(log.max_level() == EventLevel::Verbose);

    debugger.execute("event keyword all");
    ZLB_EXPECT_TRUE(output.find("all") != std::string::npos);
    ZLB_EXPECT_EQ(log.keyword_mask(), event_keyword::kAll);

    debugger.execute("event disable Test");
    ZLB_EXPECT_FALSE(log.provider_enabled(EventProvider::Test));
    debugger.execute("event enable Test");
    ZLB_EXPECT_TRUE(log.provider_enabled(EventProvider::Test));

    debugger.execute("event providers");
    ZLB_EXPECT_TRUE(output.find("Zeliboba-Boot") != std::string::npos);
    ZLB_EXPECT_TRUE(output.find("Computation") != std::string::npos);

    log.event(EventProvider::Test, ev::test::kProbe).field("name", std::string("probe-marker")).emit();
    debugger.execute("event dump 8");
    ZLB_EXPECT_TRUE(output.find("probe-marker") != std::string::npos);

    debugger.execute("event find probe-marker");
    ZLB_EXPECT_TRUE(output.find("probe-marker") != std::string::npos);
    // `event find` sets the view filter, so clear it before the other views.
    debugger.execute("event filter clear");

    debugger.execute("event stat 6");
    ZLB_EXPECT_TRUE(output.find("weight") != std::string::npos);

    debugger.execute("event activities 6");
    ZLB_EXPECT_TRUE(output.find("duration") != std::string::npos);

    debugger.execute("event timeline 6");
    ZLB_EXPECT_TRUE(output.find("timeline") != std::string::npos || output.find("no records") != std::string::npos);

    debugger.execute("event filter provider Test");
    ZLB_EXPECT_TRUE(output.find("view filter") != std::string::npos);
    debugger.execute("event filter kind activity");
    ZLB_EXPECT_TRUE(debugger.event_filter().activities_only);
    debugger.execute("event filter clear");
    ZLB_EXPECT_FALSE(debugger.event_filter().activities_only);

    debugger.execute("event save build/test-events.csv");
    ZLB_EXPECT_TRUE(output.find("exported") != std::string::npos || output.find("failed") != std::string::npos);

    debugger.execute("event off");
    ZLB_EXPECT_FALSE(log.recording());
    debugger.execute("event on");
    ZLB_EXPECT_TRUE(log.recording());

    debugger.execute("event bogus-command");
    ZLB_EXPECT_TRUE(output.find("unknown event command") != std::string::npos);

    debugger.set_output({});
}

ZLB_TEST(event_machine_instrumentation_emits_boot_events) {
    Vita& vita = events_test_vita();
    EventLog& log = vita.events();
    log.set_recording(true);
    log.set_max_level(EventLevel::Verbose);
    log.set_keyword_mask(event_keyword::kAll);
    log.set_all_providers(true);
    vita.reset(true);  // a fresh trace: build/reset/parts and every core reset

    ZLB_EXPECT_TRUE(log.total() > 0);
    // Board lifecycle plus one event per fitted part.
    ZLB_EXPECT_TRUE(log.provider_counters(EventProvider::Machine).emitted >= 2);
    // Four ARM cores, the CMeP and Ernie.
    ZLB_EXPECT_TRUE(log.provider_counters(EventProvider::Cpu).emitted >= 4);
    // The boot chain opened its first stage and recorded the transition.
    ZLB_EXPECT_TRUE(log.provider_counters(EventProvider::Boot).emitted >= 1);

    // A stage is an open Begin/End activity until the stage changes.
    std::vector<EventActivitySpan> spans = log.activities(EventFilter{});
    bool found_stage = false;
    for (const EventActivitySpan& span : spans) {
        if (span.provider == EventProvider::Boot && span.name != nullptr && std::string(span.name) == "Stage") {
            found_stage = true;
        }
    }
    ZLB_EXPECT_TRUE(found_stage);
    ZLB_EXPECT_EQ(log.provider_counters(EventProvider::Boot).open, 1u);

    // Entering another stage closes the old activity and opens a new one.
    const u64 before = log.provider_counters(EventProvider::Boot).emitted;
    vita.enter_stage(BootStage::CmepFirstLoader);
    vita.run_slice();
    ZLB_EXPECT_TRUE(log.provider_counters(EventProvider::Boot).emitted > before);
    ZLB_EXPECT_EQ(log.provider_counters(EventProvider::Boot).open, 1u);
}
