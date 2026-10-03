// zeliboba - the in-window command line.
//
// The console is the same interface as the headless one: every line goes
// through Debugger::execute(), so a command typed here behaves exactly like the
// same command in `-ex`. Completion comes from Debugger::complete(), history is
// local, and the scrollback keeps both the command output and the machine log.
#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

#include "common/util.h"
#include "ui/ui.h"

namespace zlb {

namespace {
using namespace ui_layout;

/// "zlb> " prompt, also used to size the input field.
constexpr const char* kPrompt = "zlb> ";
constexpr int kPromptWidth = 5 * kCharWidth;
}  // namespace

// ---------------------------------------------------------------------------
// Address parsing (shared with the panels)
// ---------------------------------------------------------------------------

bool parse_address(const std::string& text, u32& out) {
    const std::string t = trim(text);
    if (t.empty()) return false;

    if (t.size() > 2 && t[0] == '0' && (t[1] == 'b' || t[1] == 'B')) {
        u64 value = 0;
        for (size_t i = 2; i < t.size(); ++i) {
            if (t[i] != '0' && t[i] != '1') return false;
            value = (value << 1) | static_cast<u64>(t[i] - '0');
        }
        out = static_cast<u32>(value);
        return true;
    }

    size_t start = 0;
    if (t.size() > 2 && t[0] == '0' && (t[1] == 'x' || t[1] == 'X')) start = 2;
    else if (t[0] == '$') start = 1;

    u64 value = 0;
    size_t digits = 0;
    for (size_t i = start; i < t.size(); ++i) {
        const char c = t[i];
        if (c == 'h' || c == 'H') {
            if (i + 1 != t.size()) return false;
            break;
        }
        int digit = -1;
        if (c >= '0' && c <= '9') digit = c - '0';
        else if (c >= 'a' && c <= 'f') digit = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') digit = c - 'A' + 10;
        else return false;
        value = (value << 4) | static_cast<u64>(digit);
        ++digits;
    }
    if (digits == 0) return false;
    out = static_cast<u32>(value);
    return true;
}

// ---------------------------------------------------------------------------
// UiConsole
// ---------------------------------------------------------------------------

void UiConsole::add(const std::string& text, u32 color) {
    Line line;
    line.text = text;
    line.color = color;
    lines_.push_back(std::move(line));
    if (lines_.size() > max_lines_) {
        lines_.erase(lines_.begin(), lines_.begin() + static_cast<long>(max_lines_ / 4));
    }
}

void UiConsole::add_chunk(const std::string& text, u32 color) {
    size_t start = 0;
    while (start <= text.size()) {
        size_t end = text.find('\n', start);
        std::string piece = end == std::string::npos ? text.substr(start) : text.substr(start, end - start);
        if (!piece.empty() && piece.back() == '\r') piece.pop_back();
        // The 8x8 font has no glyph for tabs; turn them into spaces.
        for (char& c : piece) {
            if (c == '\t') c = ' ';
            else if (static_cast<unsigned char>(c) < 32) c = ' ';
        }
        if (!piece.empty() || end != std::string::npos) add(piece, color);
        if (end == std::string::npos) break;
        start = end + 1;
    }
}

void UiConsole::clear() {
    lines_.clear();
    scroll_ = 0;
}

void UiConsole::scroll(int lines) {
    const int max_scroll = std::max(0, static_cast<int>(lines_.size()) - visible_lines_);
    scroll_ = std::max(0, std::min(max_scroll, scroll_ + lines));
}

void UiConsole::insert_text(const std::string& text) {
    if (cursor_ > input_.size()) cursor_ = input_.size();
    input_.insert(cursor_, text);
    cursor_ += text.size();
    scroll_ = 0;
}

void UiConsole::push_history(const std::string& line) {
    if (line.empty()) return;
    if (!history_.empty() && history_.back() == line) return;
    history_.push_back(line);
    if (history_.size() > 200) history_.erase(history_.begin());
}

void UiConsole::complete(Debugger& debugger) {
    // Only the command word is completed; arguments are command specific and the
    // Debugger only knows about command names.
    if (input_.find(' ') != std::string::npos) return;
    std::vector<std::string> candidates = debugger.complete(input_);
    if (candidates.empty()) return;
    if (candidates.size() == 1) {
        input_ = candidates[0] + " ";
        cursor_ = input_.size();
        return;
    }
    std::string common = candidates[0];
    for (const std::string& candidate : candidates) {
        size_t n = 0;
        while (n < common.size() && n < candidate.size() && common[n] == candidate[n]) ++n;
        common.resize(n);
    }
    input_ = common;
    cursor_ = input_.size();
    std::string listing = format("%zu completions:", candidates.size());
    for (size_t i = 0; i < candidates.size(); ++i) {
        listing += " " + candidates[i];
        if ((i + 1) % 8 == 0) listing += "\n           ";
    }
    add_chunk(listing, ui_theme::kTextDim);
}

bool UiConsole::handle_key(SDL_Keycode key, SDL_Scancode scancode, SDL_Keymod mod, Debugger& debugger) {
    (void)scancode;
    const bool ctrl = (mod & SDL_KMOD_CTRL) != 0;
    switch (key) {
        case SDLK_RETURN:
        case SDLK_KP_ENTER:
            submitted_line_ = input_;
            submitted_ = true;
            input_.clear();
            cursor_ = 0;
            history_pos_ = -1;
            scroll_ = 0;
            return true;
        case SDLK_BACKSPACE:
            if (ctrl) {
                input_.clear();
                cursor_ = 0;
            } else if (cursor_ > 0 && !input_.empty()) {
                input_.erase(cursor_ - 1, 1);
                --cursor_;
            }
            return true;
        case SDLK_DELETE:
            if (cursor_ < input_.size()) input_.erase(cursor_, 1);
            return true;
        case SDLK_LEFT:
            if (cursor_ > 0) --cursor_;
            return true;
        case SDLK_RIGHT:
            if (cursor_ < input_.size()) ++cursor_;
            return true;
        case SDLK_HOME:
            cursor_ = 0;
            return true;
        case SDLK_END:
            cursor_ = input_.size();
            return true;
        case SDLK_TAB:
            complete(debugger);
            return true;
        case SDLK_UP:
            if (history_.empty()) return true;
            if (history_pos_ < 0) history_pos_ = static_cast<int>(history_.size()) - 1;
            else history_pos_ = std::max(0, history_pos_ - 1);
            input_ = history_[static_cast<size_t>(history_pos_)];
            cursor_ = input_.size();
            return true;
        case SDLK_DOWN:
            if (history_.empty()) return true;
            if (history_pos_ < 0) return true;
            if (history_pos_ + 1 >= static_cast<int>(history_.size())) {
                history_pos_ = -1;
                input_.clear();
            } else {
                ++history_pos_;
                input_ = history_[static_cast<size_t>(history_pos_)];
            }
            cursor_ = input_.size();
            return true;
        case SDLK_PAGEUP:
            scroll(10);
            return true;
        case SDLK_PAGEDOWN:
            scroll(-10);
            return true;
        default:
            return false;
    }
}

bool UiConsole::take_submitted(std::string& line) {
    if (!submitted_) return false;
    submitted_ = false;
    line = submitted_line_;
    submitted_line_.clear();
    return true;
}

void UiConsole::draw(Canvas& canvas, const UiRect& area) {
    canvas.fill(area, ui_theme::kBackground);
    canvas.hline(area.x, area.y, area.w, ui_theme::kRule);

    const int input_top = area.bottom() - kLineHeight - 8;
    const int visible = std::max(1, (input_top - area.y - 6) / kLineHeight);
    visible_lines_ = visible;
    const int total = static_cast<int>(lines_.size());
    const int max_scroll = std::max(0, total - visible);
    scroll_ = std::max(0, std::min(max_scroll, scroll_));
    const int first = std::max(0, total - visible - scroll_);

    const UiRect log_clip{area.x + 4, area.y + 3, area.w - 8, input_top - area.y - 5};
    for (int i = 0; i < visible && first + i < total; ++i) {
        const Line& line = lines_[static_cast<size_t>(first + i)];
        const int y = area.y + 4 + i * kLineHeight;
        canvas.draw_text_fit(log_clip, area.x + 6, y, line.text, line.color);
    }
    if (lines_.empty()) {
        canvas.draw_text_clip(log_clip, area.x + 6, area.y + 4, "console: type a command, 'help' for the list",
                              ui_theme::kTextDim);
    }
    if (scroll_ > 0) {
        canvas.draw_text_clip(area, area.right() - 150, area.y + 4,
                              format("-- scrolled up %d --", scroll_), ui_theme::kWarn);
    }

    // Input line.
    const UiRect input{area.x + 2, input_top, area.w - 4, kLineHeight + 6};
    canvas.fill(input, focused_ ? 0xFF0C1016u : ui_theme::kPanel);
    canvas.outline(input, focused_ ? ui_theme::kAccent : ui_theme::kRule);
    canvas.draw_text(input.x + 6, input.y + 3, kPrompt, focused_ ? ui_theme::kAccent : ui_theme::kTextDim);

    const int text_x = input.x + 6 + kPromptWidth;
    std::string shown = input_;
    const int room = std::max(1, (input.right() - text_x - 10) / kCharWidth);
    if (static_cast<int>(shown.size()) > room) {
        // Keep the caret visible by trimming from the left.
        const size_t keep = static_cast<size_t>(room - 1);
        shown = "~" + shown.substr(shown.size() - keep);
    }
    canvas.draw_text_clip(input, text_x, input.y + 3, shown, ui_theme::kTextBright);
    if (focused_) {
        size_t caret = cursor_;
        if (caret > input_.size()) caret = input_.size();
        const int caret_x = text_x + static_cast<int>(caret) * kCharWidth;
        canvas.fill(UiRect{caret_x, input.y + 2, 2, kLineHeight}, ui_theme::kCursor);
    }

    const std::string hint = focused_ ? "Tab complete   Up/Down history   Esc close"
                                      : "Enter/` to type a command";
    canvas.draw_text_clip(input, input.right() - Canvas::text_width(hint) - 8, input.y + 3, hint,
                          ui_theme::kTextDim);
}

// ---------------------------------------------------------------------------
// UiApp console plumbing
// ---------------------------------------------------------------------------

void UiApp::console_print(const std::string& text) { console_.add_chunk(text, ui_theme::kText); }

void UiApp::console_print(const std::string& text, u32 color) { console_.add_chunk(text, color); }

void UiApp::execute_command(const std::string& line) {
    const std::string trimmed = trim(line);
    if (trimmed.empty()) return;
    console_.add("zlb> " + trimmed, ui_theme::kAccent);
    console_.scroll_to_bottom();
    if (!debugger_.execute(trimmed)) quit_ = true;
    console_.scroll_to_bottom();
}

void UiApp::open_editor(EditTarget target, const std::string& initial) {
    edit_target_ = target;
    edit_text_ = initial;
    edit_cursor_ = edit_text_.size();
    console_.set_focused(false);
}

bool UiApp::editor_key(SDL_Keycode key, SDL_Keymod mod) {
    (void)mod;
    switch (key) {
        case SDLK_ESCAPE:
            edit_target_ = EditTarget::None;
            edit_text_.clear();
            edit_cursor_ = 0;
            return true;
        case SDLK_RETURN:
        case SDLK_KP_ENTER: {
            u32 address = 0;
            const EditTarget target = edit_target_;
            if (parse_address(edit_text_, address)) {
                edit_target_ = target;
                goto_address(address);
                console_print(format("goto %s", hex(address, 8).c_str()), ui_theme::kTextDim);
            } else {
                console_print(format("cannot parse address '%s'", edit_text_.c_str()), ui_theme::kError);
            }
            edit_target_ = EditTarget::None;
            edit_text_.clear();
            edit_cursor_ = 0;
            return true;
        }
        case SDLK_BACKSPACE:
            if (!edit_text_.empty()) edit_text_.pop_back();
            edit_cursor_ = edit_text_.size();
            return true;
        default:
            return false;
    }
}

void UiApp::goto_address(u32 address) {
    if (edit_target_ == EditTarget::GotoMemory) {
        memory_.address = address;
        memory_.selected_row = 0;
    } else {
        disasm_.address = address;
        disasm_.follow_pc = false;
        disasm_.selected = 0;
    }
}

void UiApp::handle_tab_click(int index) {
    const int count = static_cast<int>(UiTab::Count);
    if (index < 0 || index >= count) return;
    tab_ = static_cast<UiTab>(index);
    if (tab_ == UiTab::Devices) rebuild_device_list();
    if (tab_ == UiTab::Events) {
        refresh_events_tree();
        rebuild_event_graphs(true);
    }
}

void UiApp::toggle_breakpoint_at_selection() {
    if (disasm_.selected < 0 || disasm_.selected >= static_cast<int>(disasm_.lines.size())) return;
    const u32 address = disasm_.lines[static_cast<size_t>(disasm_.selected)].address;
    const Arch arch = debugger_.active_arch();
    if (debugger_.breakpoints(arch).count(address) != 0) {
        debugger_.remove_breakpoint(arch, address);
        console_print(format("breakpoint cleared at %s %s", to_string(arch), hex(address, 8).c_str()),
                      ui_theme::kTextDim);
    } else {
        debugger_.add_breakpoint(arch, address);
        console_print(format("breakpoint set at %s %s", to_string(arch), hex(address, 8).c_str()),
                      ui_theme::kBreakpoint);
    }
}

}  // namespace zlb
