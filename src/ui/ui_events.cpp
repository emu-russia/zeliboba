// zeliboba - the Events panel: an ETW/WPA-style trace view.
//
// Layout (1280x720 canvas, the panel is the area between the tab bar and the
// console):
//
//   +----------------------+--------------------------------------------------+
//   | Graph Explorer       | Utilization by area   (stacked weighted bars)     |
//   |  System Activity     |   +------------------------------+  Trace Rundown |
//   |  Computation         |   |                              |                |
//   |   CPU / Interrupt... |   +------------------------------+                |
//   |  Storage             | Activity by provider (Begin/End bars)             |
//   |   eMMC / SDIF / DMA  | Generic events by provider (one-shot markers)      |
//   |  ...                 |--------------------------------------------------|
//   |                      | #  Time  Provider  Task  Event  Opcode  Payload   |
//   +----------------------+--------------------------------------------------+
//
// The tree filters two different things: the checkbox on the left is the session
// (what the machine records), the highlighted state is the view filter (what the
// graphs and the table show). The view filter object is shared with the
// debugger's `event filter` command, so the GUI and the CLI never disagree.
#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "common/util.h"
#include "ui/ui.h"

namespace zlb {

namespace {

using namespace ui_layout;

constexpr int kTreeWidth = 300;
constexpr int kGraphHeight = 262;
constexpr int kLaneLabel = 76;
constexpr int kMaxLanes = 6;
constexpr int kMaxMarkerLanes = 5;

struct AreaColor {
    EventArea area;
    u32 color;
};

constexpr AreaColor kAreaColors[] = {
    {EventArea::SystemActivity, 0xFF6BD68Bu}, {EventArea::Computation, 0xFF4FA3F7u},
    {EventArea::Storage, 0xFF8BE9A0u},        {EventArea::Memory, 0xFFC792EAu},
    {EventArea::Video, 0xFFFFB86Cu},          {EventArea::Power, 0xFFF0B357u},
    {EventArea::Communications, 0xFF4DD0E1u}, {EventArea::Other, 0xFF9AA5B1u},
};

u32 area_color(EventArea area) {
    for (const AreaColor& entry : kAreaColors) {
        if (entry.area == area) return entry.color;
    }
    return ui_theme::kText;
}

u32 level_color(EventLevel level) {
    switch (level) {
        case EventLevel::Critical:
        case EventLevel::Error: return ui_theme::kError;
        case EventLevel::Warning: return ui_theme::kWarn;
        case EventLevel::Verbose: return ui_theme::kTextDim;
        default: return ui_theme::kText;
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Tree
// ---------------------------------------------------------------------------

void UiApp::refresh_events_tree() {
    EventLog& log = debugger_.event_log();
    EventFilter& filter = debugger_.event_filter();
    EventsState& state = events_ui_;

    // Keep the selection pointed at the same node across rebuilds.
    EventsState::Row previous;
    bool had_previous = state.tree_selected >= 0 && state.tree_selected < static_cast<int>(state.rows.size());
    if (had_previous) previous = state.rows[static_cast<size_t>(state.tree_selected)];

    state.rows.clear();

    // "System Activity" is the aggregate of every provider, like WPA's top node.
    EventsState::Row system_row;
    system_row.kind = EventsState::Row::Kind::System;
    system_row.label = "System Activity";
    state.rows.push_back(system_row);

    // Area index 0 (SystemActivity) has no providers of its own: the System row
    // above is its aggregate.
    for (int area_index = static_cast<int>(EventArea::Computation); area_index < kEventAreaCount; ++area_index) {
        const EventArea area = static_cast<EventArea>(area_index);
        u64 count = 0;
        bool any_recording = false;
        bool any_visible = false;
        bool first = true;
        for (int i = 0; i < kEventProviderCount; ++i) {
            const EventProvider provider = static_cast<EventProvider>(i);
            if (event_provider_info(provider).area != area) continue;
            const u64 emitted = log.provider_counters(provider).emitted;
            count += emitted;
            any_recording = any_recording || log.provider_enabled(provider);
            any_visible = any_visible || filter.provider_enabled(provider);
            if (first) {
                EventsState::Row row;
                row.kind = EventsState::Row::Kind::Area;
                row.area = area;
                row.label = to_string(area);
                row.recording = any_recording;
                row.visible = any_visible;
                state.rows.push_back(row);
                first = false;
            }
            EventsState::Row row;
            row.kind = EventsState::Row::Kind::Provider;
            row.area = area;
            row.provider = provider;
            row.label = to_string(provider);
            row.count = emitted;
            row.recording = log.provider_enabled(provider);
            row.visible = filter.provider_enabled(provider);
            state.rows.push_back(row);
        }
        if (first) {
            EventsState::Row row;
            row.kind = EventsState::Row::Kind::Area;
            row.area = area;
            row.label = to_string(area);
            state.rows.push_back(row);
        }
    }
    // The area rows carry the aggregate of their providers; the System row
    // carries the aggregate of everything.
    u64 total_count = 0;
    bool total_recording = false;
    bool total_visible = false;
    for (size_t i = 1; i < state.rows.size(); ++i) {
        if (state.rows[i].kind != EventsState::Row::Kind::Area) continue;
        u64 count = 0;
        bool recording = false;
        bool visible = false;
        for (size_t j = i + 1; j < state.rows.size() && state.rows[j].kind == EventsState::Row::Kind::Provider; ++j) {
            count += state.rows[j].count;
            recording = recording || state.rows[j].recording;
            visible = visible || state.rows[j].visible;
        }
        state.rows[i].count = count;
        state.rows[i].recording = recording;
        state.rows[i].visible = visible;
        total_count += count;
        total_recording = total_recording || recording;
        total_visible = total_visible || visible;
    }
    state.rows[0].count = total_count;
    state.rows[0].recording = total_recording;
    state.rows[0].visible = total_visible;

    if (had_previous) {
        for (size_t i = 0; i < state.rows.size(); ++i) {
            const EventsState::Row& row = state.rows[i];
            if (row.kind != previous.kind || row.label != previous.label) continue;
            state.tree_selected = static_cast<int>(i);
            break;
        }
    }
    state.tree_selected = std::max(0, std::min(state.tree_selected, static_cast<int>(state.rows.size()) - 1));
}

// ---------------------------------------------------------------------------
// Graph caches
// ---------------------------------------------------------------------------

void UiApp::events_filter_revision() { ++event_filter_revision_; }

void UiApp::rebuild_event_graphs(bool force) {
    EventLog& log = debugger_.event_log();
    const EventFilter& filter = debugger_.event_filter();
    EventsState& state = events_ui_;

    const u64 now = SDL_GetTicksNS();
    const bool log_changed = log.total() != state.cache_total;
    const bool filter_changed = event_filter_revision_ != state.cache_revision;
    const bool stale = now - state.cache_wall_ns > 250'000'000ull;
    if (!force && !filter_changed && !(log_changed && stale)) return;

    state.cache_total = log.total();
    state.cache_revision = event_filter_revision_;
    state.cache_wall_ns = now;

    state.timeline = log.timeline(filter, state.timeline_buckets);
    state.activities = log.activities(filter, 600);
    state.matched_records = log.counts(filter).records;

    // Activity lanes: the providers with the most Begin/End spans.
    std::map<EventProvider, int> activity_counts;
    for (const EventActivitySpan& span : state.activities) ++activity_counts[span.provider];
    std::vector<std::pair<int, EventProvider>> ordered;
    for (const auto& entry : activity_counts) ordered.emplace_back(entry.second, entry.first);
    std::sort(ordered.begin(), ordered.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first > b.first;
        return static_cast<int>(a.second) < static_cast<int>(b.second);
    });
    state.lane_providers.clear();
    for (const auto& entry : ordered) {
        if (static_cast<int>(state.lane_providers.size()) >= kMaxLanes) break;
        state.lane_providers.push_back(entry.second);
    }

    // Marker lanes: one-shot events sampled from the tail, grouped by provider.
    state.markers.clear();
    EventFilter oneshot = filter;
    oneshot.activities_only = false;
    oneshot.one_shot_only = true;
    const std::vector<EventRecord> sample = log.tail_matching(oneshot, 1500);
    std::map<EventProvider, int> marker_counts;
    for (const EventRecord& record : sample) {
        state.markers.push_back({record.time_ns, record.provider});
        ++marker_counts[record.provider];
    }
    std::vector<std::pair<int, EventProvider>> marker_order;
    for (const auto& entry : marker_counts) marker_order.emplace_back(entry.second, entry.first);
    std::sort(marker_order.begin(), marker_order.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first > b.first;
        return static_cast<int>(a.second) < static_cast<int>(b.second);
    });
    state.marker_providers.clear();
    for (const auto& entry : marker_order) {
        if (static_cast<int>(state.marker_providers.size()) >= kMaxMarkerLanes) break;
        state.marker_providers.push_back(entry.second);
    }

    // Drawing window: the span the *events* cover, so the graph is never mostly
    // empty. Guest time after the last event is the "Trace Rundown" band on the
    // right (a broken axis: its width is a fixed share, its label carries the
    // real duration).
    u64 lo = 0;
    u64 hi = 0;
    const bool any = log.time_range(filter, lo, hi);
    state.range_lo_ns = any ? lo : log.first_time();
    state.range_hi_ns = any ? std::max(hi, lo + 1) : state.range_lo_ns + 1;
    state.rundown_ns = any ? hi : state.range_lo_ns;
    const u64 emulated_now = vita_.emulated_nanoseconds();
    state.rundown_duration_ns = emulated_now > state.rundown_ns ? emulated_now - state.rundown_ns : 0;

    if (state.summary_mode) state.summary = log.summary(filter, 200);
}

// ---------------------------------------------------------------------------
// Refresh
// ---------------------------------------------------------------------------

void UiApp::refresh_events() {
    EventLog& log = debugger_.event_log();
    const EventFilter& filter = debugger_.event_filter();
    EventsState& state = events_ui_;

    if (state.rows.empty()) refresh_events_tree();
    else refresh_events_tree();  // counters and checkbox state track the session

    rebuild_event_graphs(false);

    // Bottom table: the newest rows that fit, oldest first.
    const UiRect area = panel_rect();
    const int graph_bottom = area.y + 18 + kGraphHeight;
    const int table_y = graph_bottom + 2;
    const int footer = 14;
    state.table_rows = std::max(1, (area.bottom() - 15 - footer - (table_y + kLineHeight + 2)) / kLineHeight);

    const u64 matched = state.matched_records;
    state.table_scroll = std::max(0, std::min(state.table_scroll,
                                              std::max(0, static_cast<int>(matched) - state.table_rows)));

    const size_t wanted = static_cast<size_t>(state.table_rows + state.table_scroll);
    std::vector<EventRecord> records = log.tail_matching(filter, wanted);
    if (records.size() > static_cast<size_t>(state.table_rows)) {
        records.erase(records.begin(), records.end() - state.table_rows);
    }

    // Duration per activity, so a Stop row shows how long its operation took.
    std::map<u64, u64> durations;
    for (const EventActivitySpan& span : state.activities) durations[span.activity] = span.duration_ns;

    state.table.clear();
    state.table.reserve(records.size());
    for (const EventRecord& record : records) {
        EventsState::TableRow row;
        row.sequence = record.sequence;
        row.time_ns = record.time_ns;
        row.provider = record.provider;
        row.task = record.task;
        row.name = record.name ? record.name : event_name(record.provider, record.id);
        row.opcode = record.opcode;
        row.level = static_cast<u8>(record.level);
        const auto iter = durations.find(record.activity);
        if (iter != durations.end()) {
            row.duration_ns = iter->second;
            row.has_duration = true;
        }
        row.payload = record.payload_text();
        state.table.push_back(std::move(row));
    }
    state.selected_row = std::max(0, std::min(state.selected_row, static_cast<int>(state.table.size()) - 1));
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

void UiApp::draw_events_panel(const UiRect& area) {
    EventLog& log = debugger_.event_log();
    const EventFilter& filter = debugger_.event_filter();
    EventsState& state = events_ui_;

    // -- header ---------------------------------------------------------
    canvas_.fill(UiRect{area.x + 1, area.y + 1, area.w - 2, 16}, ui_theme::kPanelAlt);
    const std::string header =
        format("event trace   %s   records %zu/%zu   total=%llu   filtered=%llu   session: level<=%s kw=%s   "
               "view: providers=%d areas=%s text='%s'%s",
               log.recording() ? "session=ON" : "session=OFF", log.count(), log.capacity(),
               static_cast<unsigned long long>(log.total()), static_cast<unsigned long long>(log.filtered()),
               to_string(log.max_level()), describe_event_keywords(log.keyword_mask()).c_str(),
               [&] {
                   int on = 0;
                   for (int i = 0; i < kEventProviderCount; ++i) {
                       if (filter.provider_enabled(static_cast<EventProvider>(i))) ++on;
                   }
                   return on;
               }(),
               filter.area_mask >= (1ull << kEventAreaCount) - 1 ? "all" : "some", filter.text.c_str(),
               filter.activities_only ? "  kind=activity" : (filter.one_shot_only ? "  kind=one-shot" : ""));
    canvas_.draw_text_clip(UiRect{area.x + 6, area.y + 1, area.w - 160, 16}, area.x + 6, area.y + 4, header,
                           ui_theme::kAccent);

    const int top = area.y + 18;
    const int bottom = area.bottom() - 15;

    // -- Graph Explorer tree --------------------------------------------
    const UiRect tree{area.x + 1, top, kTreeWidth - 2, bottom - top};
    canvas_.fill(tree, 0xFF12171Eu);
    canvas_.vline(tree.right(), tree.y, tree.h, ui_theme::kRule);

    const int tree_rows = std::max(1, (tree.h - 14) / kLineHeight);
    if (state.tree_selected < state.tree_scroll) state.tree_scroll = state.tree_selected;
    if (state.tree_selected >= state.tree_scroll + tree_rows) state.tree_scroll = state.tree_selected - tree_rows + 1;
    state.tree_scroll = std::max(0, std::min(state.tree_scroll,
                                             std::max(0, static_cast<int>(state.rows.size()) - tree_rows)));

    canvas_.draw_text(tree.x + 5, tree.y + 3, "Graph Explorer", ui_theme::kTextBright);
    const int row0 = tree.y + 3 + kLineHeight + 2;
    state.hover_row = -1;
    if (tree.contains(mouse_x_, mouse_y_) && mouse_y_ >= row0) {
        const int hovered = state.tree_scroll + (mouse_y_ - row0) / kLineHeight;
        if (hovered >= 0 && hovered < static_cast<int>(state.rows.size())) state.hover_row = hovered;
    }
    int y = row0;
    for (int i = 0; i < tree_rows; ++i) {
        const int index = state.tree_scroll + i;
        if (index >= static_cast<int>(state.rows.size())) break;
        const EventsState::Row& row = state.rows[static_cast<size_t>(index)];
        const bool selected = index == state.tree_selected;
        const bool hover = index == state.hover_row;
        if (selected) canvas_.fill(UiRect{tree.x + 1, y - 1, tree.w - 2, kLineHeight}, ui_theme::kSelection);
        else if (hover) canvas_.fill(UiRect{tree.x + 1, y - 1, tree.w - 2, kLineHeight}, ui_theme::kPanelAlt);

        const u32 color = area_color(row.area);
        const int x = row.kind == EventsState::Row::Kind::Provider ? tree.x + 16 : tree.x + 5;
        if (row.kind == EventsState::Row::Kind::System) {
            canvas_.fill(UiRect{tree.x + 5, y + 1, 7, 7}, row.visible ? ui_theme::kOk : ui_theme::kTextDim);
            canvas_.draw_text_clip(tree, tree.x + 16, y, "System Activity", ui_theme::kOk);
        } else if (row.kind == EventsState::Row::Kind::Area) {
            canvas_.draw_text_clip(tree, x, y, std::string("[>] ") + row.label, ui_theme::kAccent);
        } else {
            canvas_.fill(UiRect{tree.x + 6, y + 2, 5, 5}, color);
            // The session checkbox is what the machine records; the label colour
            // shows whether the view filter keeps it.
            canvas_.draw_text_clip(tree, x, y, row.recording ? "[x]" : "[ ]",
                                   row.recording ? ui_theme::kOk : ui_theme::kTextDim);
            canvas_.draw_text_fit(UiRect{tree.x + 40, tree.y, 150, tree.h}, tree.x + 40, y, row.label,
                                  row.visible ? (row.recording ? ui_theme::kTextBright : ui_theme::kText)
                                              : ui_theme::kTextDim);
        }
        canvas_.draw_text_clip(tree, tree.x + 200, y, format("%8llu", static_cast<unsigned long long>(row.count)),
                               row.visible ? ui_theme::kText : ui_theme::kTextDim);
        if (!row.visible) canvas_.draw_text_clip(tree, tree.x + 264, y, "off", ui_theme::kTextDim);
        y += kLineHeight;
    }

    // -- graphs ----------------------------------------------------------
    const UiRect graph{area.x + kTreeWidth, top, area.w - kTreeWidth - 1, kGraphHeight};
    canvas_.fill(graph, 0xFF10151Bu);
    const int gx = graph.x + 6;
    const int gw = graph.w - 12;
    const int hist_y = graph.y + 13;
    const int hist_h = 70;

    const bool have_timeline = !state.timeline.empty();
    u64 max_weight = 0;
    u64 max_count = 0;
    for (const EventTimelineBucket& bucket : state.timeline) {
        u64 weight = 0;
        for (int a = 0; a < kEventAreaCount; ++a) weight += bucket.weight_ns[a];
        max_weight = std::max(max_weight, weight);
        max_count = std::max(max_count, bucket.total);
    }

    canvas_.draw_text_clip(graph, gx, graph.y + 2,
                           have_timeline
                               ? format("Utilization by area  (weighted duration, %d buckets; bar height = "
                                        "share of the busiest bucket)",
                                        static_cast<int>(state.timeline.size()))
                               : std::string("Utilization by area  (no events match the filter yet)"),
                           ui_theme::kAccent);
    canvas_.fill(UiRect{gx - 2, hist_y, gw + 4, hist_h}, 0xFF0C1016);
    canvas_.outline(UiRect{gx - 2, hist_y, gw + 4, hist_h}, ui_theme::kRule);

    // Trace Rundown: guest time after the last recorded event. The band is drawn
    // under the bars, at a fixed share of the width, because its duration is not
    // comparable to the event span (a broken axis - the label carries the time).
    const u64 span = state.range_hi_ns > state.range_lo_ns ? state.range_hi_ns - state.range_lo_ns : 1;
    const bool have_rundown = state.rundown_duration_ns > 0;
    const int plot_w = have_rundown ? (gw * 88) / 100 : gw;
    int rundown_x = -1;
    if (have_rundown) {
        rundown_x = gx - 1 + plot_w;
        canvas_.fill(UiRect{rundown_x, hist_y, std::max(1, gx - 1 + gw - rundown_x), hist_h}, 0xFF2A323Cu);
    }

    if (have_timeline) {
        const double bucket_width = static_cast<double>(plot_w) / static_cast<double>(state.timeline.size());
        for (size_t b = 0; b < state.timeline.size(); ++b) {
            const EventTimelineBucket& bucket = state.timeline[b];
            u64 weight = 0;
            for (int a = 0; a < kEventAreaCount; ++a) weight += bucket.weight_ns[a];
            // Bars are weighted when the trace has durations (activities), and
            // fall back to plain event counts for an all one-shot trace.
            const double scale = max_weight != 0 ? static_cast<double>(weight) / static_cast<double>(max_weight)
                                                 : static_cast<double>(bucket.total) /
                                                       static_cast<double>(max_count != 0 ? max_count : 1);
            int bar = static_cast<int>(scale * (hist_h - 2) + 0.5);
            if (bar <= 0 && bucket.total != 0) bar = 1;
            int yy = hist_y + hist_h - 1;
            for (int a = 0; a < kEventAreaCount && bar > 0; ++a) {
                const u64 share = max_weight != 0 ? bucket.weight_ns[a] : bucket.counts[a];
                int part = max_weight != 0
                               ? static_cast<int>(static_cast<double>(share) / static_cast<double>(max_weight) *
                                                      (hist_h - 2) + 0.5)
                               : (bucket.total != 0
                                      ? static_cast<int>(static_cast<double>(bucket.counts[a]) /
                                                             static_cast<double>(bucket.total) * bar +
                                                         0.5)
                                      : 0);
                if (part <= 0) continue;
                part = std::min(part, bar);
                const int px = gx - 1 + static_cast<int>(static_cast<double>(b) * bucket_width);
                const int pw = std::max(1, static_cast<int>(bucket_width));
                canvas_.fill(UiRect{px, yy - part, pw, part}, area_color(static_cast<EventArea>(a)));
                yy -= part;
                bar -= part;
            }
        }
    }

    // Trace Rundown: the marker line and label, drawn over the bars. The label
    // carries the real untraced duration, because the band uses a broken axis.
    if (rundown_x >= 0) {
        canvas_.vline(rundown_x, hist_y, hist_h, 0xFF3E4A57u);
        canvas_.draw_text_clip(graph, rundown_x + 4, hist_y + 4, "Trace Rundown", 0xFFB6C2D0u);
        canvas_.draw_text_clip(graph, rundown_x + 4, hist_y + 4 + kLineHeight,
                               format("%.6f s", static_cast<double>(state.rundown_duration_ns) / 1.0e9),
                               0xFF8C99A8u);
        canvas_.draw_text_clip(graph, rundown_x + 4, hist_y + 4 + 2 * kLineHeight, "no events", 0xFF8C99A8u);
    }

    // Time axis (the event range; the rundown band is outside it).
    const int axis_y = hist_y + hist_h + 2;
    canvas_.hline(gx - 2, axis_y, gw + 4, ui_theme::kRule);
    const int axis_w = have_rundown ? plot_w : gw;
    for (int i = 0; i < 8; ++i) {
        const double frac = static_cast<double>(i) / 7.0;
        const int x = gx - 2 + static_cast<int>(static_cast<double>(axis_w) * frac);
        canvas_.vline(std::min(x, area.right() - 2), axis_y, 4, ui_theme::kTextDim);
        const std::string label = format("%.4f", static_cast<double>(state.range_lo_ns) / 1.0e9 +
                                                     static_cast<double>(span) / 1.0e9 * frac);
        const int lx = i < 7 ? x - 4 : x - Canvas::text_width(label) + 2;
        canvas_.draw_text_clip(graph, lx, axis_y + 3, label, ui_theme::kTextDim);
    }

    // Time -> x helper for the lanes: the event range maps to the plot width.
    auto time_x = [&](u64 time) {
        const double frac = static_cast<double>(time - state.range_lo_ns) / static_cast<double>(span);
        const int width = have_rundown ? plot_w : gw;
        return gx + kLaneLabel + static_cast<int>(std::min(1.0, std::max(0.0, frac)) *
                                                  (width - kLaneLabel - 2));
    };
    const int lane_w = (have_rundown ? plot_w : gw) - kLaneLabel - 2;

    int lane_y = axis_y + 26;
    canvas_.draw_text_clip(graph, gx, lane_y - 12, "Activity by provider  (Begin/End bars; width = duration)",
                           ui_theme::kAccent);
    const int lane_x = gx + kLaneLabel;
    if (state.lane_providers.empty()) {
        canvas_.draw_text_clip(graph, lane_x, lane_y, "(no activities recorded yet)", ui_theme::kTextDim);
        lane_y += kLineHeight;
    }
    for (EventProvider provider : state.lane_providers) {
        canvas_.draw_text_clip(graph, gx, lane_y, to_string(provider), ui_theme::kTextDim);
        canvas_.hline(lane_x, lane_y + 5, lane_w, 0xFF1A222Cu);
        const u32 color = area_color(event_provider_info(provider).area);
        for (const EventActivitySpan& activity : state.activities) {
            if (activity.provider != provider) continue;
            const int x0 = time_x(activity.begin_ns);
            const int x1 = time_x(activity.open ? state.range_hi_ns : activity.end_ns);
            canvas_.fill(UiRect{x0, lane_y, std::max(2, x1 - x0), 8},
                         activity.open ? ui_theme::kWarn : color);
        }
        lane_y += kLineHeight;
    }

    int marker_y = lane_y + 13;
    canvas_.draw_text_clip(graph, gx, marker_y - 12, "Generic events by provider  (one-shot markers)",
                           ui_theme::kAccent);
    if (state.marker_providers.empty()) {
        canvas_.draw_text_clip(graph, lane_x, marker_y, "(no one-shot events match the filter)", ui_theme::kTextDim);
        marker_y += kLineHeight;
    }
    for (EventProvider provider : state.marker_providers) {
        canvas_.draw_text_clip(graph, gx, marker_y, to_string(provider), ui_theme::kTextDim);
        canvas_.hline(lane_x, marker_y + 5, lane_w, 0xFF1A222Cu);
        const u32 color = area_color(event_provider_info(provider).area);
        int drawn = 0;
        for (const EventsState::Marker& marker : state.markers) {
            if (marker.provider != provider) continue;
            const int x = time_x(marker.time_ns);
            canvas_.fill(UiRect{x, marker_y + 2, 1, 5}, color);
            canvas_.fill(UiRect{x - 1, marker_y + 4, 3, 1}, color);
            if (++drawn > 400) break;
        }
        marker_y += kLineHeight;
    }

    // -- table -----------------------------------------------------------
    const int table_y = graph.y + graph.h + 2;
    const int table_h = bottom - table_y;
    const UiRect table_clip{graph.x, table_y, graph.w, table_h};
    canvas_.fill(UiRect{graph.x, table_y, graph.w, table_h}, 0xFF12171Eu);

    if (state.summary_mode) {
        const int cols[6] = {6, 40, 116, 200, 340, 470};
        canvas_.fill(UiRect{graph.x, table_y, graph.w, kLineHeight + 2}, ui_theme::kPanelAlt);
        canvas_.draw_text(graph.x + cols[0], table_y + 1, "#", ui_theme::kTextBright);
        canvas_.draw_text(graph.x + cols[1], table_y + 1, "Time", ui_theme::kTextBright);
        canvas_.draw_text(graph.x + cols[2], table_y + 1, "Provider", ui_theme::kTextBright);
        canvas_.draw_text(graph.x + cols[3], table_y + 1, "Event", ui_theme::kTextBright);
        canvas_.draw_text(graph.x + cols[4], table_y + 1, "Count", ui_theme::kTextBright);
        canvas_.draw_text(graph.x + cols[5], table_y + 1, "Weight/%", ui_theme::kTextBright);
        canvas_.hline(graph.x, table_y + kLineHeight + 1, graph.w, ui_theme::kRule);
        int row_y = table_y + kLineHeight + 3;
        for (size_t i = 0; i < state.summary.size(); ++i) {
            if (row_y + kLineHeight > bottom - 14) break;
            const EventSummaryRow& row = state.summary[i];
            if (static_cast<int>(i) == state.selected_row) {
                canvas_.fill(UiRect{graph.x + 1, row_y - 1, graph.w - 2, kLineHeight}, ui_theme::kSelection);
            }
            canvas_.draw_text_clip(table_clip, graph.x + cols[0], row_y, format("%zu", i + 1),
                                   ui_theme::kTextDim);
            canvas_.draw_text_clip(table_clip, graph.x + cols[1], row_y, "-", ui_theme::kTextDim);
            canvas_.draw_text_clip(table_clip, graph.x + cols[2], row_y, to_string(row.provider),
                                   area_color(event_provider_info(row.provider).area));
            canvas_.draw_text_fit(UiRect{graph.x + cols[3], table_y, 130, table_h}, graph.x + cols[3], row_y,
                                  row.name ? row.name : "?", level_color(row.level));
            canvas_.draw_text_clip(table_clip, graph.x + cols[4], row_y,
                                   format("%llu", static_cast<unsigned long long>(row.count)), ui_theme::kText);
            canvas_.draw_text_clip(table_clip, graph.x + cols[5], row_y,
                                   format("%.3f ms  %5.1f%%", static_cast<double>(row.weight_ns) / 1.0e6,
                                          row.percent),
                                   row.weight_ns != 0 ? ui_theme::kChanged : ui_theme::kTextDim);
            row_y += kLineHeight;
        }
        canvas_.draw_text_clip(table_clip, graph.x + 6, bottom - 14,
                               format("%zu summary groups  (s = record table)", state.summary.size()),
                               ui_theme::kTextDim);
    } else {
        const int cols[8] = {6, 40, 118, 218, 256, 410, 480, 560};
        const char* names[8] = {"#", "Time (s)", "Provider", "Task", "Event", "Opcode", "Duration", "Payload"};
        canvas_.fill(UiRect{graph.x, table_y, graph.w, kLineHeight + 2}, ui_theme::kPanelAlt);
        for (int i = 0; i < 8; ++i) canvas_.draw_text(graph.x + cols[i], table_y + 1, names[i], ui_theme::kTextBright);
        canvas_.hline(graph.x, table_y + kLineHeight + 1, graph.w, ui_theme::kRule);

        int row_y = table_y + kLineHeight + 3;
        for (size_t i = 0; i < state.table.size(); ++i) {
            if (row_y + kLineHeight > bottom - 14) break;
            const EventsState::TableRow& row = state.table[i];
            const bool selected = static_cast<int>(i) == state.selected_row;
            if (selected) canvas_.fill(UiRect{graph.x + 1, row_y - 1, graph.w - 2, kLineHeight}, ui_theme::kSelection);
            canvas_.draw_text_clip(table_clip, graph.x + cols[0], row_y,
                                   format("%llu", static_cast<unsigned long long>(row.sequence)),
                                   ui_theme::kTextDim);
            canvas_.draw_text_clip(table_clip, graph.x + cols[1], row_y,
                                   format("%.6f", static_cast<double>(row.time_ns) / 1.0e9), ui_theme::kTextDim);
            canvas_.draw_text_clip(table_clip, graph.x + cols[2], row_y, to_string(row.provider),
                                   area_color(event_provider_info(row.provider).area));
            canvas_.draw_text_clip(table_clip, graph.x + cols[3], row_y, format("%u", row.task),
                                   ui_theme::kTextDim);
            canvas_.draw_text_fit(UiRect{graph.x + cols[4], table_y, 150, table_h}, graph.x + cols[4], row_y,
                                  row.name ? row.name : "?", level_color(static_cast<EventLevel>(row.level)));
            canvas_.draw_text_clip(table_clip, graph.x + cols[5], row_y, to_string(row.opcode),
                                   ui_theme::kTextDim);
            canvas_.draw_text_clip(table_clip, graph.x + cols[6], row_y,
                                   row.has_duration
                                       ? format("%.4f ms", static_cast<double>(row.duration_ns) / 1.0e6)
                                       : std::string("-"),
                                   row.has_duration ? ui_theme::kChanged : ui_theme::kTextDim);
            canvas_.draw_text_fit(UiRect{graph.x + cols[7], table_y, graph.w - cols[7] - 6, table_h},
                                  graph.x + cols[7], row_y, row.payload, ui_theme::kText);
            row_y += kLineHeight;
        }
        if (state.table.empty()) {
            canvas_.draw_text_clip(table_clip, graph.x + 8, table_y + kLineHeight + 4,
                                   "(no records match the filter - run the machine with `g` or the RUN button)",
                                   ui_theme::kWarn);
        }
        canvas_.draw_text_clip(table_clip, graph.x + 6, bottom - 14,
                               format("%zu of %llu records shown   scroll: PgUp/PgDn or the wheel   s = summary",
                                      state.table.size(), static_cast<unsigned long long>(log.total())),
                               ui_theme::kTextDim);
    }
}

// ---------------------------------------------------------------------------
// Interaction
// ---------------------------------------------------------------------------

namespace {

const EventsState::Row* selected_row_of(const EventsState& state) {
    if (state.tree_selected < 0 || state.tree_selected >= static_cast<int>(state.rows.size())) return nullptr;
    return &state.rows[static_cast<size_t>(state.tree_selected)];
}

}  // namespace

void UiApp::events_toggle_selected(bool record) {
    EventLog& log = debugger_.event_log();
    EventFilter& filter = debugger_.event_filter();
    const EventsState::Row* row = selected_row_of(events_ui_);
    if (row == nullptr) return;

    if (row->kind == EventsState::Row::Kind::Provider) {
        if (record) {
            const bool on = !row->recording;
            log.set_provider_enabled(row->provider, on);
            console_print(format("event session: provider %s %s", to_string(row->provider), on ? "on" : "off"),
                          ui_theme::kTextDim);
        } else {
            const bool on = !row->visible;
            filter.set_provider(row->provider, on);
            console_print(format("event view: provider %s %s", to_string(row->provider), on ? "shown" : "hidden"),
                          ui_theme::kTextDim);
            events_filter_revision();
        }
    } else if (row->kind == EventsState::Row::Kind::System) {
        // The top node toggles everything at once.
        const bool on = !row->visible;
        if (record) {
            log.set_all_providers(on);
        } else {
            if (on) filter.all_providers();
            else filter.clear_providers();
            events_filter_revision();
        }
        console_print(format("event %s: all providers %s", record ? "session" : "view", on ? "on" : "off"),
                      ui_theme::kTextDim);
    } else {
        // An area row toggles every provider under it.
        bool any = false;
        for (const EventsState::Row& candidate : events_ui_.rows) {
            if (candidate.kind != EventsState::Row::Kind::Provider || candidate.area != row->area) continue;
            any = any || (record ? candidate.recording : candidate.visible);
        }
        const bool on = !any;
        if (record) log.set_area_enabled(row->area, on);
        else {
            filter.set_area(row->area, on);
            events_filter_revision();
        }
        console_print(format("event %s: area %s %s", record ? "session" : "view", to_string(row->area),
                             on ? "on" : "off"),
                      ui_theme::kTextDim);
    }

    refresh_events_tree();
    rebuild_event_graphs(true);
}

bool UiApp::events_key(SDL_Keycode key) {
    EventLog& log = debugger_.event_log();
    EventFilter& filter = debugger_.event_filter();
    EventsState& state = events_ui_;

    switch (key) {
        case SDLK_C:
            log.clear();
            console_print("event: session cleared", ui_theme::kTextDim);
            rebuild_event_graphs(true);
            return true;
        case SDLK_R:
            log.set_recording(!log.recording());
            console_print(log.recording() ? "event: session ON" : "event: session OFF", ui_theme::kTextDim);
            return true;
        case SDLK_L: {
            // Cycle the session level: what the machine records at all.
            static const EventLevel levels[] = {EventLevel::Critical, EventLevel::Error, EventLevel::Warning,
                                                EventLevel::Informational, EventLevel::Verbose};
            int index = 0;
            for (int i = 0; i < 5; ++i) {
                if (levels[i] == log.max_level()) index = i;
            }
            log.set_max_level(levels[(index + 1) % 5]);
            console_print(format("event level: %s", to_string(log.max_level())), ui_theme::kTextDim);
            rebuild_event_graphs(true);
            return true;
        }
        case SDLK_K: {
            static const char* masks[] = {"default", "all", "boot|storage", "interrupt|timer", "security"};
            static const EventKeyword parsed[] = {event_keyword::kDefault, event_keyword::kAll,
                                                  event_keyword::kBoot | event_keyword::kStorage,
                                                  event_keyword::kInterrupt | event_keyword::kTimer,
                                                  event_keyword::kSecurity};
            int index = 0;
            for (int i = 0; i < 5; ++i) {
                if (parsed[i] == log.keyword_mask()) index = i;
            }
            const int next = (index + 1) % 5;
            log.set_keyword_mask(parsed[next]);
            console_print(format("event keywords: %s", masks[next]), ui_theme::kTextDim);
            rebuild_event_graphs(true);
            return true;
        }
        case SDLK_A: {
            // Cycle the view area filter, so one key walks the Graph Explorer.
            const u64 all = (1ull << kEventAreaCount) - 1;
            int index = -1;
            for (int i = 0; i < kEventAreaCount; ++i) {
                if (filter.area_mask == (1ull << i)) index = i;
            }
            if (filter.area_mask == all) index = -1;
            const int next = index + 1;
            filter.area_mask = next >= kEventAreaCount ? all : (1ull << next);
            console_print(filter.area_mask == all
                              ? std::string("event view: all areas")
                              : format("event view: area %s", to_string(static_cast<EventArea>(next))),
                          ui_theme::kTextDim);
            events_filter_revision();
            rebuild_event_graphs(true);
            return true;
        }
        case SDLK_T: {
            // Quick text filter: the selected node's name, or clear it.
            const EventsState::Row* row = selected_row_of(state);
            if (row == nullptr) return true;
            if (!filter.text.empty() && filter.text == to_lower(row->label)) {
                filter.text.clear();
                console_print("event view: text filter cleared", ui_theme::kTextDim);
            } else {
                filter.text = to_lower(row->label);
                console_print(format("event view: text filter '%s'", filter.text.c_str()), ui_theme::kTextDim);
            }
            events_filter_revision();
            rebuild_event_graphs(true);
            return true;
        }
        case SDLK_S:
            state.summary_mode = !state.summary_mode;
            console_print(state.summary_mode ? "event: summary table (count/weight/%weight)"
                                             : "event: record table",
                          ui_theme::kTextDim);
            rebuild_event_graphs(true);
            return true;
        case SDLK_E: {
            const std::string path = resolve_workspace_path("build/events.csv");
            std::string error;
            if (log.save_csv(path, filter, error)) {
                console_print(format("event: %zu records exported to %s", log.query(filter).size(), path.c_str()),
                              ui_theme::kOk);
            } else {
                console_print("event save failed: " + error, ui_theme::kError);
            }
            return true;
        }
        case SDLK_X:
            filter = EventFilter{};
            events_filter_revision();
            rebuild_event_graphs(true);
            console_print("event view: filter reset", ui_theme::kTextDim);
            return true;
        case SDLK_B:
        case SDLK_O: {
            const bool want_activity = key == SDLK_B;
            const bool already = want_activity ? filter.activities_only : filter.one_shot_only;
            filter.activities_only = want_activity && !already;
            filter.one_shot_only = !want_activity && !already;
            console_print(filter.activities_only
                              ? "event view: activities only"
                              : (filter.one_shot_only ? "event view: one-shot events only"
                                                      : "event view: all records"),
                          ui_theme::kTextDim);
            events_filter_revision();
            rebuild_event_graphs(true);
            return true;
        }
        default:
            return false;
    }
}

void UiApp::events_scroll(int lines) {
    EventsState& state = events_ui_;
    if (state.focus == 1) {
        state.table_scroll = std::max(0, state.table_scroll + lines);
        return;
    }
    const int count = static_cast<int>(state.rows.size());
    if (count == 0) return;
    state.tree_selected = std::max(0, std::min(count - 1, state.tree_selected - lines));
}

void UiApp::events_click(int x, int y) {
    const UiRect area = panel_rect();
    EventsState& state = events_ui_;
    const int top = area.y + 18;
    const int bottom = area.bottom() - 15;

    // Graph Explorer tree.
    const UiRect tree{area.x + 1, top, kTreeWidth - 2, bottom - top};
    if (tree.contains(x, y)) {
        state.focus = 0;
        const int row0 = tree.y + 3 + kLineHeight + 2;
        const int row = (y - row0) / kLineHeight;
        const int index = state.tree_scroll + row;
        if (row >= 0 && index >= 0 && index < static_cast<int>(state.rows.size())) {
            // Clicking the checkbox toggles what the session records; clicking
            // anywhere else on the row toggles the view filter.
            const bool checkbox = x < tree.x + 40;
            state.tree_selected = index;
            events_toggle_selected(checkbox);
        }
        return;
    }

    // Graphs: clicking an activity bar shows the record it came from.
    const UiRect graph{area.x + kTreeWidth, top, area.w - kTreeWidth - 1, kGraphHeight};
    if (graph.contains(x, y)) {
        state.focus = 0;
        return;
    }

    // Table.
    const int table_y = top + kGraphHeight + 2;
    if (y >= table_y) {
        state.focus = 1;
        const int row = (y - (table_y + kLineHeight + 3)) / kLineHeight;
        if (row >= 0) state.selected_row = row;
    }
}

}  // namespace zlb
