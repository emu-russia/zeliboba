// zeliboba - the debugger panels.
//
// Every panel is a thin projection of the Debugger API the CLI uses:
//   disassembly -> Debugger::disassemble()
//   registers   -> Debugger::registers()
//   memory      -> Debugger::memory_dump()
//   trace       -> Bus::trace (the same records `trace` prints)
//   devices     -> Bus::devices() + Device::enumerate_registers()
//   boot        -> Vita::boot_report()/boot_status()/plan_boot() + `stage`
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "common/util.h"
#include "ui/ui.h"

namespace zlb {

namespace {

using namespace ui_layout;

/// Rows that fit in a panel body, leaving room for the header and help line.
int body_rows(const UiRect& area, int header_height) {
    return std::max(1, (area.h - header_height - 16) / kLineHeight);
}

struct RegisterRow {
    bool header = false;
    std::string text;
    std::string key;
    u64 value = 0;
    std::string note;
};

}  // namespace

// ---------------------------------------------------------------------------
// State refresh
// ---------------------------------------------------------------------------

void UiApp::refresh_views() {
    const UiRect area = panel_rect();
    const int text_rows = body_rows(area, 18);

    switch (tab_) {
        case UiTab::Disassembly: {
            Cpu* cpu = debugger_.active_core();
            if (disasm_.follow_pc && cpu) disasm_.address = cpu->get_pc();
            disasm_.rows = text_rows;
            disasm_.lines = debugger_.disassemble(disasm_.address, disasm_.rows);
            const int count = static_cast<int>(disasm_.lines.size());
            if (count > 0) disasm_.selected = std::min(disasm_.selected, count - 1);
            else disasm_.selected = 0;
            break;
        }
        case UiTab::Registers: {
            const u64 instructions = vita_.total_instructions();
            if (instructions != registers_.snapshot_instructions || registers_.values.empty()) {
                std::vector<RegValue> values = debugger_.registers();
                std::set<std::string> changed;
                std::map<std::string, u64> current;
                for (const RegValue& value : values) {
                    const std::string key = value.group + "." + value.name;
                    current[key] = value.value;
                    if (registers_.have_previous) {
                        auto iter = registers_.previous.find(key);
                        if (iter != registers_.previous.end() && iter->second != value.value) changed.insert(key);
                    }
                }
                registers_.previous.swap(current);
                registers_.changed.swap(changed);
                registers_.have_previous = true;
                registers_.values = std::move(values);
                registers_.snapshot_instructions = instructions;
            }
            break;
        }
        case UiTab::Memory: {
            memory_.rows = text_rows;
            if (!memory_.initialized) {
                memory_.initialized = true;
                Cpu* cpu = debugger_.active_core();
                if (cpu) memory_.address = cpu->get_pc();
            }
            memory_.lines = debugger_.memory_dump(memory_.address, memory_.rows, memory_.bytes_per_row);
            break;
        }
        case UiTab::Trace: {
            Cpu* cpu = debugger_.active_core();
            if (!cpu) break;
            Bus& bus = *cpu->bus;
            trace_.count = std::max(64, text_rows + 64);
            std::vector<AccessRecord> records = bus.trace.tail(static_cast<size_t>(trace_.count));
            trace_.rows.clear();
            trace_.rows.reserve(records.size());
            for (const AccessRecord& record : records) {
                if (trace_.only_mmio && record.device < 0 && !record.unmapped) continue;
                TraceState::Row row;
                row.kind = record.kind;
                row.size = record.size;
                row.address = record.address;
                row.value = record.value;
                row.pc = record.pc;
                row.unmapped = record.unmapped;
                if (record.device >= 0 && record.device < static_cast<int>(bus.devices().size())) {
                    row.device = bus.devices()[static_cast<size_t>(record.device)]->name();
                }
                trace_.rows.push_back(std::move(row));
            }
            const int max_scroll = std::max(0, static_cast<int>(trace_.rows.size()) - text_rows);
            trace_.scroll = std::min(trace_.scroll, max_scroll);
            break;
        }
        case UiTab::Devices: {
            rebuild_device_list();
            if (devices_.entries.empty()) break;
            select_device(devices_.selected);
            Device* device = devices_.entries[static_cast<size_t>(devices_.selected)].device;
            for (size_t i = 0; i < devices_.registers.size(); ++i) {
                const RegisterInfo& info = devices_.registers[i];
                u64 value = 0;
                if (!device->peek_register(info.name, value)) {
                    value = device->read(info.address, info.width == 0 ? 4 : info.width);
                }
                devices_.values[i] = value;
            }
            break;
        }
        case UiTab::Boot: {
            boot_.report = split(vita_.boot_report(), '\n');
            boot_.plan = vita_.plan_boot();
            boot_.milestones = vita_.milestones();
            break;
        }
        default:
            break;
    }
}

void UiApp::rebuild_device_list() {
    if (!devices_.entries.empty()) return;
    struct BusEntry {
        const char* name;
        Bus* bus;
    };
    const BusEntry buses[3] = {{"mep", &vita_.cmep_bus()}, {"arm", &vita_.arm_bus()},
                               {"rl78", &vita_.syscon_bus()}};
    for (const BusEntry& entry : buses) {
        for (const auto& device : entry.bus->devices()) {
            DeviceEntry item;
            item.bus = entry.bus;
            item.device = device.get();
            item.bus_name = entry.name;
            devices_.entries.push_back(std::move(item));
        }
    }
    if (!devices_.entries.empty()) select_device(0);
}

void UiApp::select_device(int index) {
    if (devices_.entries.empty()) return;
    index = std::max(0, std::min(static_cast<int>(devices_.entries.size()) - 1, index));
    Device* device = devices_.entries[static_cast<size_t>(index)].device;
    devices_.selected = index;
    if (devices_.selected_name == device->name() && !devices_.registers.empty()) return;
    devices_.selected_name = device->name();
    devices_.registers.clear();
    device->enumerate_registers(devices_.registers);
    devices_.values.assign(devices_.registers.size(), 0);
    devices_.register_scroll = 0;
}

// ---------------------------------------------------------------------------
// Disassembly
// ---------------------------------------------------------------------------

void UiApp::draw_disassembly_panel(const UiRect& area) {
    Cpu* cpu = debugger_.active_core();
    const u32 pc = cpu ? cpu->get_pc() : 0;

    const std::string header = format("%s %s  pc=%s  base=%s  %s", to_string(debugger_.active_arch()),
                                      cpu ? cpu->core_name() : "(no core)", hex(pc, 8).c_str(),
                                      hex(disasm_.address, 8).c_str(),
                                      disasm_.follow_pc ? "[following PC]" : "[detached - press f to follow]");
    canvas_.fill(UiRect{area.x + 1, area.y + 1, area.w - 2, 16}, ui_theme::kPanelAlt);
    canvas_.draw_text_clip(area, area.x + 6, area.y + 4, header, ui_theme::kAccent);
    if (cpu) {
        const std::string state = cpu->status_line();
        if (!state.empty()) {
            canvas_.draw_text_clip(area, area.right() - Canvas::text_width(state) - 8, area.y + 4, state,
                                   ui_theme::kTextDim);
        }
    }

    const int first_y = area.y + 20;
    for (size_t i = 0; i < disasm_.lines.size(); ++i) {
        const Debugger::DisassemblyLine& line = disasm_.lines[i];
        const int y = first_y + static_cast<int>(i) * kLineHeight;
        if (y + kLineHeight > area.bottom() - 16) break;

        const bool selected = static_cast<int>(i) == disasm_.selected;
        if (line.is_pc) canvas_.fill(UiRect{area.x + 1, y - 1, area.w - 2, kLineHeight}, ui_theme::kAccentDim);
        else if (selected) canvas_.fill(UiRect{area.x + 1, y - 1, area.w - 2, kLineHeight}, ui_theme::kSelection);

        if (line.has_breakpoint) canvas_.fill(UiRect{area.x + 4, y + 2, 5, 5}, ui_theme::kBreakpoint);
        if (line.is_pc) canvas_.draw_text(area.x + 12, y, ">", ui_theme::kPc);

        canvas_.draw_text_clip(area, area.x + 26, y, hex(line.address, 8), ui_theme::kTextDim);

        std::string bytes;
        for (size_t b = 0; b < line.bytes.size() && b < 8; ++b) bytes += format("%02X ", line.bytes[b]);
        canvas_.draw_text_clip(area, area.x + 104, y, bytes, 0xFF8FA3B8u);

        const u32 text_color = line.is_pc ? ui_theme::kPc : (selected ? ui_theme::kTextBright : ui_theme::kText);
        canvas_.draw_text_fit(area, area.x + 300, y, line.text, text_color);
    }

    if (disasm_.lines.empty()) {
        canvas_.draw_text_clip(area, area.x + 8, first_y, "(nothing to disassemble - no active core)",
                               ui_theme::kWarn);
    }
}

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

void UiApp::draw_registers_panel(const UiRect& area) {
    std::vector<RegisterRow> rows;
    std::string last_group;
    for (const RegValue& value : registers_.values) {
        if (value.group != last_group) {
            RegisterRow header;
            header.header = true;
            header.text = value.group.empty() ? "(registers)" : value.group;
            rows.push_back(std::move(header));
            last_group = value.group;
        }
        RegisterRow row;
        row.key = value.group + "." + value.name;
        row.text = value.name;
        row.value = value.value;
        row.note = value.note;
        rows.push_back(std::move(row));
    }

    canvas_.fill(UiRect{area.x + 1, area.y + 1, area.w - 2, 16}, ui_theme::kPanelAlt);
    canvas_.draw_text_clip(area, area.x + 6, area.y + 4,
                           format("%s registers  (%zu entries, %zu changed since the last step)",
                                  to_string(debugger_.active_arch()), registers_.values.size(),
                                  registers_.changed.size()),
                           ui_theme::kAccent);

    if (rows.empty()) {
        canvas_.draw_text_clip(area, area.x + 8, area.y + 24, "(no registers for the active core)",
                               ui_theme::kWarn);
        return;
    }

    // Two columns when the list is long enough, so nothing scrolls off screen.
    const int first_y = area.y + 22;
    const int total_rows = static_cast<int>(rows.size());
    const int available = body_rows(area, 22);
    const bool two_columns = total_rows > available;
    const int column_height = two_columns ? (total_rows + 1) / 2 : total_rows;
    const int half_width = two_columns ? area.w / 2 : area.w;

    for (int column = 0; column < (two_columns ? 2 : 1); ++column) {
        int start = column * column_height;
        if (start >= total_rows) break;
        const int x = area.x + column * half_width;
        const UiRect clip{x, area.y + 20, half_width, area.bottom() - area.y - 36};

        // Repeat the group header when a column starts in the middle of a group.
        int cursor = start - registers_.scroll;
        if (start != 0 && !rows[static_cast<size_t>(start)].header) {
            for (int back = start - 1; back >= 0; --back) {
                if (rows[static_cast<size_t>(back)].header) {
                    canvas_.draw_text_clip(clip, x + 8, first_y, "-- " + rows[static_cast<size_t>(back)].text,
                                           ui_theme::kAccent);
                    break;
                }
            }
            cursor = start - registers_.scroll + 1;
        }

        for (int i = start; i < std::min(start + column_height, total_rows); ++i) {
            const RegisterRow& row = rows[static_cast<size_t>(i)];
            const int y = first_y + cursor * kLineHeight;
            ++cursor;
            if (y + kLineHeight > area.bottom() - 16) break;
            if (row.header) {
                canvas_.draw_text_clip(clip, x + 8, y, "-- " + row.text, ui_theme::kAccent);
                continue;
            }
            const bool changed = registers_.changed.count(row.key) != 0;
            if (changed) canvas_.fill(UiRect{x + 4, y - 1, half_width - 10, kLineHeight}, 0xFF1E3A2A);
            canvas_.draw_text_clip(clip, x + 12, y, row.text, changed ? ui_theme::kChanged : ui_theme::kText);
            canvas_.draw_text_clip(clip, x + 130, y, format("0x%016llX", static_cast<unsigned long long>(row.value)),
                                   changed ? ui_theme::kChanged : ui_theme::kTextBright);
            if (!row.note.empty()) {
                canvas_.draw_text_fit(clip, x + 288, y, row.note, ui_theme::kTextDim);
            }
        }
    }

    if (!registers_.have_previous) {
        canvas_.draw_text_clip(area, area.x + 8, area.bottom() - 30, "(step or run to compare register values)",
                               ui_theme::kTextDim);
    }
}

// ---------------------------------------------------------------------------
// Memory
// ---------------------------------------------------------------------------

void UiApp::draw_memory_panel(const UiRect& area) {
    canvas_.fill(UiRect{area.x + 1, area.y + 1, area.w - 2, 16}, ui_theme::kPanelAlt);
    canvas_.draw_text_clip(area, area.x + 6, area.y + 4,
                           format("%s memory  %s..%s  %d bytes/row  j to jump", to_string(debugger_.active_arch()),
                                  hex(memory_.address, 8).c_str(),
                                  hex(memory_.address + static_cast<u32>(memory_.rows * memory_.bytes_per_row), 8).c_str(),
                                  memory_.bytes_per_row),
                           ui_theme::kAccent);

    const int first_y = area.y + 20;
    const int ascii_x = area.x + 496;
    const int hex_x = area.x + 84;
    for (int row = 0; row < static_cast<int>(memory_.lines.size()); ++row) {
        const std::string& line = memory_.lines[static_cast<size_t>(row)];
        const int y = first_y + row * kLineHeight;
        if (y + kLineHeight > area.bottom() - 16) break;
        if (row == memory_.selected_row) {
            canvas_.fill(UiRect{area.x + 1, y - 1, area.w - 2, kLineHeight}, ui_theme::kSelection);
        }

        // The dump line is "<addr>  <hex bytes> |ascii|"; show the three parts in
        // their own columns so the ASCII column is a real column.
        const std::string address = line.substr(0, std::min<size_t>(8, line.size()));
        const size_t bar = line.find('|');
        std::string hex_part;
        std::string ascii;
        if (bar != std::string::npos && line.size() > bar + 1) {
            const size_t end = line.rfind('|');
            ascii = line.substr(bar + 1, end > bar + 1 ? end - bar - 1 : 0);
            const size_t hex_start = line.size() > 10 ? 10 : line.size();
            hex_part = line.substr(hex_start, bar > hex_start ? bar - hex_start : 0);
        } else if (line.size() > 10) {
            hex_part = line.substr(10);
        }

        canvas_.draw_text_clip(area, area.x + 6, y, address, ui_theme::kTextDim);

        // One column per byte, with the wide gap the dump puts in the middle.
        std::string compact;
        for (char c : hex_part) {
            if (c != ' ') compact.push_back(c);
        }
        for (int b = 0; b < 16 && b * 2 + 1 < static_cast<int>(compact.size()); ++b) {
            const std::string byte = compact.substr(static_cast<size_t>(b) * 2, 2);
            const int bx = hex_x + b * 3 * font::kGlyphWidth + (b >= 8 ? font::kGlyphWidth : 0);
            canvas_.draw_text_clip(area, bx, y, byte, byte == "00" ? 0xFF56626Fu : ui_theme::kText);
        }

        canvas_.draw_text_clip(area, ascii_x, y, ascii, 0xFF9FD3A0u);
    }

    if (memory_.lines.empty()) {
        canvas_.draw_text_clip(area, area.x + 8, first_y, "(no memory)", ui_theme::kWarn);
    }
}

// ---------------------------------------------------------------------------
// Trace
// ---------------------------------------------------------------------------

void UiApp::draw_trace_panel(const UiRect& area) {
    canvas_.fill(UiRect{area.x + 1, area.y + 1, area.w - 2, 16}, ui_theme::kPanelAlt);
    canvas_.draw_text_clip(area, area.x + 6, area.y + 4,
                           format("%s bus trace  %zu accesses shown  total=%llu  %s", to_string(debugger_.active_arch()),
                                  trace_.rows.size(),
                                  static_cast<unsigned long long>(debugger_.active_core()
                                                                      ? debugger_.active_core()->bus->trace.total()
                                                                      : 0),
                                  trace_.only_mmio ? "(MMIO only)" : "(MMIO + RAM)"),
                           ui_theme::kAccent);

    const int first_y = area.y + 20;
    const int visible = body_rows(area, 18);
    int count = static_cast<int>(trace_.rows.size());
    int start = std::max(0, count - visible - trace_.scroll);
    for (int i = 0; i < visible && start + i < count; ++i) {
        const TraceState::Row& row = trace_.rows[static_cast<size_t>(start + i)];
        const int y = first_y + i * kLineHeight;
        if (y + kLineHeight > area.bottom() - 16) break;

        u32 color = ui_theme::kText;
        const char* kind = "R";
        if (row.kind == AccessKind::Write) {
            kind = "W";
            color = 0xFFE8B27Du;
        } else if (row.kind == AccessKind::Fetch) {
            kind = "F";
            color = ui_theme::kTextDim;
        }
        if (row.unmapped) color = ui_theme::kError;

        canvas_.draw_text_clip(area, area.x + 6, y, format("%5d", start + i), ui_theme::kTextDim);
        canvas_.draw_text_clip(area, area.x + 54, y, kind, color);
        canvas_.draw_text_clip(area, area.x + 70, y, format("%u", row.size), ui_theme::kTextDim);
        canvas_.draw_text_clip(area, area.x + 90, y, hex(row.address, 8), ui_theme::kText);
        canvas_.draw_text_clip(area, area.x + 170, y,
                               format("= %016llX", static_cast<unsigned long long>(row.value)),
                               color);
        canvas_.draw_text_clip(area, area.x + 340, y, format("pc=%s", hex(row.pc, 8).c_str()),
                               ui_theme::kTextDim);
        canvas_.draw_text_fit(area, area.x + 440, y,
                              row.device.empty() ? std::string(row.unmapped ? "(unmapped)" : "(ram)") : row.device,
                              row.device.empty() ? ui_theme::kError : ui_theme::kAccent);
    }

    if (trace_.rows.empty()) {
        canvas_.draw_text_clip(area, area.x + 8, first_y,
                               "(trace empty - run the machine; the ring buffer records MMIO always and RAM "
                               "only when enabled)",
                               ui_theme::kTextDim);
    }
}

// ---------------------------------------------------------------------------
// Devices
// ---------------------------------------------------------------------------

void UiApp::draw_devices_panel(const UiRect& area) {
    rebuild_device_list();

    const int list_width = 430;
    const UiRect list{area.x + 1, area.y + 18, list_width, area.h - 34};
    const UiRect detail{area.x + list_width + 2, area.y + 18, area.w - list_width - 4, area.h - 34};
    canvas_.fill(UiRect{area.x + 1, area.y + 1, area.w - 2, 16}, ui_theme::kPanelAlt);
    canvas_.draw_text_clip(area, area.x + 6, area.y + 4,
                           format("%zu MMIO devices on 3 buses  (up/down select, PgUp/PgDn registers)",
                                  devices_.entries.size()),
                           ui_theme::kAccent);

    const int visible = std::max(1, list.h / kLineHeight - 1);
    // Keep the selection inside the visible window.
    if (devices_.selected < devices_.scroll) devices_.scroll = devices_.selected;
    if (devices_.selected >= devices_.scroll + visible) devices_.scroll = devices_.selected - visible + 1;
    devices_.scroll = std::max(0, std::min(devices_.scroll,
                                           std::max(0, static_cast<int>(devices_.entries.size()) - visible)));

    for (int i = 0; i < visible; ++i) {
        const int index = devices_.scroll + i;
        if (index >= static_cast<int>(devices_.entries.size())) break;
        const DeviceEntry& entry = devices_.entries[static_cast<size_t>(index)];
        const int y = list.y + i * kLineHeight;
        if (index == devices_.selected) canvas_.fill(UiRect{list.x, y - 1, list.w, kLineHeight}, ui_theme::kSelection);
        canvas_.draw_text_clip(list, list.x + 4, y, format("%-3s", entry.bus_name.c_str()), ui_theme::kTextDim);
        canvas_.draw_text_fit(UiRect{list.x + 30, list.y, list.w - 160, list.h}, list.x + 30, y,
                              entry.device->name(), index == devices_.selected ? ui_theme::kTextBright
                                                                              : ui_theme::kText);
        canvas_.draw_text_clip(list, list.x + list.w - 122, y,
                               format("%s+%X", hex(entry.device->base(), 8).c_str(), entry.device->size()),
                               ui_theme::kTextDim);
    }

    if (devices_.entries.empty()) {
        canvas_.draw_text_clip(list, list.x + 6, list.y, "(no devices registered)", ui_theme::kWarn);
        return;
    }

    Device* device = devices_.entries[static_cast<size_t>(devices_.selected)].device;
    int y = detail.y;
    canvas_.draw_text_clip(detail, detail.x + 4, y, device->name(), ui_theme::kAccent);
    y += kLineHeight;
    canvas_.draw_text_fit(detail, detail.x + 4, y,
                          format("base=%s size=0x%X  %s", hex(device->base(), 8).c_str(), device->size(),
                                 device->summary().c_str()),
                          ui_theme::kText);
    y += kLineHeight;

    std::vector<std::string> describe;
    device->describe(describe);
    for (const std::string& line : describe) {
        canvas_.draw_text_fit(detail, detail.x + 8, y, line, ui_theme::kTextDim);
        y += kLineHeight;
    }
    y += 2;

    canvas_.draw_text_clip(detail, detail.x + 4, y,
                           format("registers (%zu)", devices_.registers.size()), ui_theme::kTextBright);
    y += kLineHeight;

    const int register_rows = std::max(0, (detail.bottom() - y) / kLineHeight);
    for (int i = 0; i < register_rows; ++i) {
        const int index = devices_.register_scroll + i;
        if (index >= static_cast<int>(devices_.registers.size())) break;
        const RegisterInfo& info = devices_.registers[static_cast<size_t>(index)];
        const u64 value = index < static_cast<int>(devices_.values.size()) ? devices_.values[static_cast<size_t>(index)]
                                                                          : 0;
        const int row_y = y + i * kLineHeight;
        canvas_.draw_text_clip(detail, detail.x + 4, row_y, format("+%03X", info.address), ui_theme::kTextDim);
        canvas_.draw_text_fit(UiRect{detail.x + 54, detail.y, 220, detail.h}, detail.x + 54, row_y, info.name,
                              ui_theme::kText);
        canvas_.draw_text_clip(detail, detail.x + 286, row_y,
                               format("0x%0*llX", static_cast<int>(info.width) * 2,
                                      static_cast<unsigned long long>(value)),
                               value == info.reset_value ? ui_theme::kTextBright : ui_theme::kChanged);
        if (value != info.reset_value) {
            canvas_.draw_text_clip(detail, detail.x + 440, row_y,
                                   format("(reset %llX)", static_cast<unsigned long long>(info.reset_value)),
                                   ui_theme::kTextDim);
        }
    }

    if (devices_.registers.empty()) {
        canvas_.draw_text_clip(detail, detail.x + 8, y, "(this device does not enumerate registers)",
                               ui_theme::kTextDim);
    }
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

void UiApp::draw_boot_panel(const UiRect& area) {
    canvas_.fill(UiRect{area.x + 1, area.y + 1, area.w - 2, 16}, ui_theme::kPanelAlt);
    canvas_.draw_text_clip(area, area.x + 6, area.y + 4,
                           format("boot chain - stage=%s  arm_released=%d  steps_in_stage=%llu",
                                  to_string(vita_.stage()), vita_.boot_status().arm_released ? 1 : 0,
                                  static_cast<unsigned long long>(vita_.boot_status().steps_in_stage)),
                           ui_theme::kAccent);

    const BootStatus& status = vita_.boot_status();
    int y = area.y + 22;
    canvas_.draw_text_clip(area, area.x + 8, y,
                           format("%s  (status=0x%llX arm_entry=0x%08X)", to_string(status.stage),
                                  static_cast<unsigned long long>(status.cmep_status), status.arm_entry),
                           ui_theme::kTextBright);
    y += kLineHeight;
    canvas_.draw_text_fit(area, area.x + 8, y, status.detail, ui_theme::kTextDim);
    y += kLineHeight;

    // Stage buttons (also bound to keys 1..4).
    static const char* stages[4] = {"first", "second", "kbl", "kernel"};
    boot_.button_labels.assign(stages, stages + 4);
    boot_.button_rects.clear();
    int button_x = area.x + 8;
    for (int i = 0; i < 4; ++i) {
        const std::string label = format("%d: stage %s", i + 1, stages[i]);
        const int width = Canvas::text_width(label) + 14;
        const UiRect rect{button_x, y, width, kLineHeight + 5};
        const bool hover = rect.contains(mouse_x_, mouse_y_);
        canvas_.fill(rect, hover ? ui_theme::kAccent : ui_theme::kTabIdle);
        canvas_.outline(rect, ui_theme::kRule);
        canvas_.draw_text(rect.x + 7, rect.y + 3, label, ui_theme::kTextBright);
        boot_.button_rects.push_back(rect);
        button_x += width + 6;
    }
    y += kLineHeight + 12;

    canvas_.draw_text_clip(area, area.x + 8, y, "boot report", ui_theme::kAccent);
    y += kLineHeight;
    for (const std::string& line : boot_.report) {
        if (y > area.bottom() - 40) break;
        if (line.empty()) continue;
        canvas_.draw_text_fit(area, area.x + 14, y, line, ui_theme::kText);
        y += kLineHeight;
    }

    y += 4;
    canvas_.draw_text_clip(area, area.x + 8, y, "plan", ui_theme::kAccent);
    y += kLineHeight;
    for (const std::string& line : boot_.plan) {
        if (y > area.bottom() - 30) break;
        canvas_.draw_text_fit(area, area.x + 14, y, line, ui_theme::kTextDim);
        y += kLineHeight;
    }

    y += 4;
    canvas_.draw_text_clip(area, area.x + 8, y, "milestones", ui_theme::kAccent);
    y += kLineHeight;
    if (boot_.milestones.empty()) {
        canvas_.draw_text_clip(area, area.x + 14, y, "(none yet)", ui_theme::kTextDim);
        y += kLineHeight;
    }
    for (const std::string& line : boot_.milestones) {
        if (y > area.bottom() - 18) break;
        canvas_.draw_text_fit(area, area.x + 14, y, "* " + line, ui_theme::kOk);
        y += kLineHeight;
    }
}

// ---------------------------------------------------------------------------
// Inline address editor
// ---------------------------------------------------------------------------

void UiApp::draw_edit_field(const UiRect& area, const std::string& label) {
    const UiRect box{area.x + 6, area.y + 20, std::min(area.w - 12, 560), 2 * kLineHeight + 8};
    canvas_.fill(box, 0xFF0C1016u);
    canvas_.outline(box, ui_theme::kAccent);
    canvas_.draw_text(box.x + 6, box.y + 5, label + ":", ui_theme::kTextDim);
    const int text_x = box.x + 6 + Canvas::text_width(label + ": ");
    canvas_.draw_text(text_x, box.y + 5, edit_text_, ui_theme::kTextBright);
    canvas_.fill(UiRect{text_x + Canvas::text_width(edit_text_), box.y + 4, 2, kLineHeight},
                 ui_theme::kCursor);
    canvas_.draw_text(box.x + 6, box.y + 5 + kLineHeight, "Enter accept   Esc cancel   hex or 0x/$, 0b binary",
                      ui_theme::kTextDim);
}

}  // namespace zlb
