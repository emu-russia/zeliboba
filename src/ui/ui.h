// zeliboba - SDL3 frontend: window, input, audio and the debugger panels.
//
// The frontend owns one Vita, one Debugger and one window. Everything the UI
// shows comes from the same public API the console frontend uses, so a panel
// and a debugger command can never disagree. The whole interface is drawn into
// an ARGB8888 SDL_Surface with the embedded 8x8 font and uploaded to a
// streaming texture once per frame.
#pragma once

#include <SDL3/SDL.h>

#include <atomic>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/log.h"
#include "debug/debugger.h"
#include "machine/vita.h"
#include "ui/font8x8.h"

namespace zlb {

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------

/// The Vita's own pad, as reported by the frontend.
///
/// The kernel is the current target, so nothing consumes these bits yet: the
/// struct is the mapping layer that a later HID workstream plugs into the
/// machine. Keyboard and gamepad both feed it.
struct VitaInput {
    bool up = false;
    bool down = false;
    bool left = false;
    bool right = false;
    bool cross = false;     ///< "X"
    bool circle = false;    ///< "O"
    bool square = false;    ///< "square"
    bool triangle = false;  ///< "triangle"
    bool start = false;
    bool select = false;
    bool l1 = false;
    bool r1 = false;

    /// Analog sticks, -1..1, y positive is down (matching the pad hardware).
    float left_x = 0.0f;
    float left_y = 0.0f;
    float right_x = 0.0f;
    float right_y = 0.0f;

    std::string source = "keyboard";  ///< last thing that produced input
    bool gamepad_connected = false;
    bool any_pressed() const;

    void clear_buttons();
    /// "UP|CROSS L(-0.42,0.10)" style summary for the status bar.
    std::string status() const;
};

/// Keyboard -> Vita pad stub mapping (arrows, Z/X/A/S, Q/E, Enter/Shift).
/// Returns true when the key belongs to the pad, filling `input`.
bool vita_input_from_key(SDL_Scancode scancode, bool down, VitaInput& input);

// ---------------------------------------------------------------------------
// Theme
// ---------------------------------------------------------------------------

namespace ui_layout {
/// The window is resizable, the canvas is not: it is stretched to fit, which
/// keeps the panel geometry independent of the display and of the DPI scale.
constexpr int kCanvasWidth = 1280;
constexpr int kCanvasHeight = 720;
constexpr int kTabBarHeight = 24;
constexpr int kConsoleHeight = 176;
constexpr int kStatusHeight = 26;
constexpr int kLineHeight = 11;
constexpr int kCharWidth = 8;
}  // namespace ui_layout

namespace ui_theme {
constexpr u32 kBackground = 0xFF10141Au;
constexpr u32 kPanel = 0xFF161C24u;
constexpr u32 kPanelAlt = 0xFF1C242Eu;
constexpr u32 kText = 0xFFD8DEE9u;
constexpr u32 kTextDim = 0xFF7A8899u;
constexpr u32 kTextBright = 0xFFFFFFFFu;
constexpr u32 kAccent = 0xFF4FA3F7u;
constexpr u32 kAccentDim = 0xFF27405Cu;
constexpr u32 kPc = 0xFFFFD866u;
constexpr u32 kBreakpoint = 0xFFFF6B6Bu;
constexpr u32 kChanged = 0xFF8BE9A0u;
constexpr u32 kOk = 0xFF6BD68Bu;
constexpr u32 kWarn = 0xFFF0B357u;
constexpr u32 kError = 0xFFFF7B72u;
constexpr u32 kTabActive = 0xFF2E7DD1u;
constexpr u32 kTabIdle = 0xFF1B2430u;
constexpr u32 kSelection = 0xFF26405Cu;
constexpr u32 kRule = 0xFF2B3642u;
constexpr u32 kCursor = 0xFFFFD866u;
}  // namespace ui_theme

// ---------------------------------------------------------------------------
// Canvas
// ---------------------------------------------------------------------------

struct UiRect {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;

    bool contains(int px, int py) const {
        return px >= x && px < x + w && py >= y && py < y + h;
    }
    int right() const { return x + w; }
    int bottom() const { return y + h; }
};

/// An ARGB8888 software surface plus the few primitives the UI needs.
class Canvas {
public:
    Canvas() = default;
    ~Canvas();

    Canvas(const Canvas&) = delete;
    Canvas& operator=(const Canvas&) = delete;

    bool create(int width, int height);
    void destroy();

    bool valid() const { return surface_ != nullptr; }
    int width() const { return width_; }
    int height() const { return height_; }
    int pitch() const { return pitch_; }
    SDL_Surface* surface() { return surface_; }
    const void* pixels() const { return pixels_; }

    void clear(u32 argb);
    void pixel(int x, int y, u32 argb);
    void fill(const UiRect& area, u32 argb);
    void outline(const UiRect& area, u32 argb);
    void hline(int x, int y, int width, u32 argb);
    void vline(int x, int y, int height, u32 argb);
    void draw_char(int x, int y, char c, u32 argb, int scale = 1);
    void draw_text(int x, int y, const std::string& text, u32 argb, int scale = 1);
    /// Draw text clipped to `clip` horizontally and vertically.
    void draw_text_clip(const UiRect& clip, int x, int y, const std::string& text, u32 argb, int scale = 1);
    /// Like draw_text_clip but stops at the right edge instead of being cut off.
    void draw_text_fit(const UiRect& clip, int x, int y, const std::string& text, u32 argb, int scale = 1);

    static int text_width(const std::string& text, int scale = 1) {
        return static_cast<int>(text.size()) * font::kGlyphWidth * scale;
    }
    static int text_height(int scale = 1) { return font::kGlyphHeight * scale; }

private:
    SDL_Surface* surface_ = nullptr;
    u32* pixels_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    int pitch_ = 0;
};

// ---------------------------------------------------------------------------
// Audio
// ---------------------------------------------------------------------------

/// A real SDL3 audio stream (48 kHz, stereo, S16).
///
/// The machine does not produce samples yet, so the callback writes silence -
/// or a quiet test tone, which proves end to end that the stream is live.
class UiAudio {
public:
    UiAudio() = default;
    ~UiAudio();

    UiAudio(const UiAudio&) = delete;
    UiAudio& operator=(const UiAudio&) = delete;

    /// Open the default playback device. Fills `status()` either way; when it
    /// returns false the frontend simply runs without audio.
    bool open();
    void close();

    bool available() const { return stream_ != nullptr; }
    /// Human readable result of the last open() attempt, including SDL_GetError.
    const std::string& status() const { return status_; }

    void set_muted(bool muted) { muted_.store(muted); }
    bool muted() const { return muted_.load(); }
    void set_test_tone(bool on) { test_tone_.store(on); }
    bool test_tone() const { return test_tone_.load(); }

    u64 callbacks() const { return callbacks_.load(); }
    u64 frames() const { return frames_.load(); }

    std::string summary() const;

private:
    static void SDLCALL trampoline(void* userdata, SDL_AudioStream* stream, int additional, int total);
    void render(SDL_AudioStream* stream, int additional);

    SDL_AudioStream* stream_ = nullptr;
    SDL_AudioDeviceID device_ = 0;
    std::string status_ = "audio: not opened";
    std::atomic<bool> muted_{false};
    std::atomic<bool> test_tone_{false};
    double phase_ = 0.0;
    std::vector<u8> scratch_;  ///< audio thread only
    std::atomic<u64> callbacks_{0};
    std::atomic<u64> frames_{0};
};

// ---------------------------------------------------------------------------
// Console
// ---------------------------------------------------------------------------

/// The in-window command line: scrollback, history, Tab completion.
class UiConsole {
public:
    void add(const std::string& text, u32 color);
    /// Split a multi-line chunk (debugger output, log record) into lines.
    void add_chunk(const std::string& text, u32 color);
    void clear();

    void set_focused(bool focused) { focused_ = focused; }
    bool focused() const { return focused_; }

    void scroll(int lines);
    void scroll_to_bottom() { scroll_ = 0; }

    /// Feed a key event. Returns true when the console consumed it.
    bool handle_key(SDL_Keycode key, SDL_Scancode scancode, SDL_Keymod mod, Debugger& debugger);
    /// Insert text from SDL_EVENT_TEXT_INPUT.
    void insert_text(const std::string& text);
    /// True (once) after Enter was pressed; the owner then runs the line.
    bool take_submitted(std::string& line);

    void draw(Canvas& canvas, const UiRect& area);

    const std::string& input() const { return input_; }
    size_t line_count() const { return lines_.size(); }

private:
    struct Line {
        std::string text;
        u32 color = ui_theme::kText;
    };

    void push_history(const std::string& line);
    void complete(Debugger& debugger);

    std::vector<Line> lines_;
    std::vector<std::string> history_;
    std::string input_;
    size_t cursor_ = 0;
    int scroll_ = 0;         ///< lines scrolled up from the bottom
    int history_pos_ = -1;   ///< -1 = editing a fresh line
    int visible_lines_ = 13; ///< updated by draw(), used to clamp the scroll
    size_t max_lines_ = 4000;
    bool focused_ = false;
    bool submitted_ = false;
    std::string submitted_line_;
};

// ---------------------------------------------------------------------------
// Panels
// ---------------------------------------------------------------------------

enum class UiTab : int {
    Disassembly = 0,
    Registers,
    Memory,
    Trace,
    Devices,
    Boot,
    Display,
    Count,
};

const char* tab_name(UiTab tab);

struct DisasmState {
    u32 address = 0;
    int rows = 0;       ///< filled in from the panel height each frame
    int selected = 0;
    bool follow_pc = true;
    std::vector<Debugger::DisassemblyLine> lines;
};

struct RegisterState {
    std::vector<RegValue> values;
    std::map<std::string, u64> previous;
    std::set<std::string> changed;
    int scroll = 0;
    bool have_previous = false;
    /// Instruction count the snapshot was taken at: the "changed since the last
    /// step" highlight is only recomputed when the machine actually advanced.
    u64 snapshot_instructions = 0;
};

struct MemoryState {
    u32 address = 0;
    int rows = 0;
    int bytes_per_row = 16;
    std::vector<std::string> lines;
    int selected_row = 0;
    /// The view opens on the active core's PC the first time it is shown.
    bool initialized = false;
};

struct TraceState {
    struct Row {
        AccessKind kind = AccessKind::Read;
        unsigned size = 4;
        u32 address = 0;
        u64 value = 0;
        u32 pc = 0;
        std::string device;
        bool unmapped = false;
    };
    std::vector<Row> rows;
    int count = 64;
    int scroll = 0;
    bool only_mmio = true;
};

struct DeviceEntry {
    Bus* bus = nullptr;
    Device* device = nullptr;
    std::string bus_name;
};

struct DevicesState {
    std::vector<DeviceEntry> entries;
    int selected = 0;
    int scroll = 0;
    std::vector<RegisterInfo> registers;
    std::vector<u64> values;
    int register_scroll = 0;
    std::string selected_name;
};

struct BootState {
    std::vector<std::string> report;
    std::vector<std::string> plan;
    std::vector<std::string> milestones;
    /// Stage buttons, rebuilt every time the panel is drawn.
    std::vector<std::string> button_labels;
    std::vector<UiRect> button_rects;
};

struct DisplayState {
    int width = 0;
    int height = 0;
    int stride = 0;
    int bytes_per_pixel = 0;
    u64 frames = 0;
    bool present = false;
    u64 last_change = 0;
};

enum class EditTarget { None, GotoDisassembly, GotoMemory };

/// Parse an address the way the panels show them: hex by default, with "0x"/"$"
/// prefixes accepted, "0b" for binary and a trailing 'h' tolerated.
bool parse_address(const std::string& text, u32& out);

// ---------------------------------------------------------------------------
// The application
// ---------------------------------------------------------------------------

class UiApp {
public:
    UiApp();
    ~UiApp();

    UiApp(const UiApp&) = delete;
    UiApp& operator=(const UiApp&) = delete;

    /// Build the machine, open the window and run until the user quits.
    /// Returns the process exit code.
    int run(int argc, char** argv);

private:
    // -- lifecycle (ui_main.cpp) ---------------------------------------
    bool init_machine(int argc, char** argv);
    bool init_window();
    /// Give up on the window and keep drawing into the ARGB8888 canvas, so the
    /// screenshot mode also works on a machine with no display at all.
    bool enable_offscreen(const std::string& reason);
    void shutdown();
    void main_loop();
    void pump_events();
    void handle_event(const SDL_Event& event);
    void handle_key(SDL_Keycode key, SDL_Scancode scancode, bool down, SDL_Keymod mod);
    void handle_text_input(const char* text);
    void handle_mouse_button(const SDL_MouseButtonEvent& button, bool down);
    void handle_mouse_motion(const SDL_MouseMotionEvent& motion);
    void handle_mouse_wheel(const SDL_MouseWheelEvent& wheel);
    void handle_gamepad_event(const SDL_Event& event);
    void open_gamepads();
    void close_gamepads();
    bool window_to_canvas(float window_x, float window_y, int& canvas_x, int& canvas_y) const;

    // -- emulation (ui_main.cpp) ---------------------------------------
    void run_frame();
    void toggle_pause();
    void step_into();
    void step_over();
    void set_running(bool running);
    Arch breakpoint_hit();

    // -- input helpers (ui_main.cpp) -----------------------------------
    void sync_input_from_keyboard(SDL_Scancode scancode, bool down);

    // -- drawing (ui_main.cpp) -----------------------------------------
    void render();
    void draw_tab_bar();
    void draw_status_bar();
    void draw_panel();
    void draw_help_line(const UiRect& area);
    void draw_display_panel(const UiRect& area);
    void blit_display(const UiRect& area, const u8* pixels, int width, int height, int stride, int bpp);
    void draw_edit_field(const UiRect& area, const std::string& label);

    // -- panels (ui_panels.cpp) ----------------------------------------
    void refresh_views();
    void draw_disassembly_panel(const UiRect& area);
    void draw_registers_panel(const UiRect& area);
    void draw_memory_panel(const UiRect& area);
    void draw_trace_panel(const UiRect& area);
    void draw_devices_panel(const UiRect& area);
    void draw_boot_panel(const UiRect& area);
    void rebuild_device_list();
    void select_device(int index);

    // -- console plumbing (ui_console.cpp) -----------------------------
    void console_print(const std::string& text);
    void console_print(const std::string& text, u32 color);
    void execute_command(const std::string& line);
    void open_editor(EditTarget target, const std::string& initial);
    bool editor_key(SDL_Keycode key, SDL_Keymod mod);
    void goto_address(u32 address);
    void handle_tab_click(int index);
    /// Enter on the disassembly tab: add/remove a breakpoint on the selection.
    void toggle_breakpoint_at_selection();

    // -- layout ---------------------------------------------------------
    UiRect tab_rect(int index) const;
    UiRect panel_rect() const;
    UiRect console_rect() const;
    UiRect status_rect() const;

    // -- state ----------------------------------------------------------
    Vita vita_;
    Debugger debugger_;
    VitaConfig config_;

    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    SDL_Texture* texture_ = nullptr;
    Canvas canvas_;
    std::vector<SDL_Gamepad*> gamepads_;

    UiAudio audio_;
    UiConsole console_;

    UiTab tab_ = UiTab::Disassembly;
    bool paused_ = true;
    bool quit_ = false;

    DisasmState disasm_;
    RegisterState registers_;
    MemoryState memory_;
    TraceState trace_;
    DevicesState devices_;
    BootState boot_;
    DisplayState display_;

    VitaInput input_;
    std::string input_note_;

    EditTarget edit_target_ = EditTarget::None;
    std::string edit_text_;
    size_t edit_cursor_ = 0;

    /// The last printable key consumed as a UI shortcut: the matching
    /// SDL_EVENT_TEXT_INPUT that follows it is swallowed so that "n" steps the
    /// machine instead of opening the console with an "n" in it.
    char suppress_char_ = 0;
    int mouse_x_ = 0;
    int mouse_y_ = 0;

    double fps_ = 0.0;
    u64 fps_frames_ = 0;
    u64 fps_last_ns_ = 0;
    double emulated_per_wall_ = 0.0;
    double last_emulated_ = 0.0;
    u64 last_wall_ns_ = 0;

    /// --frames self test: quit after this many drawn frames (0 = never).
    int frame_limit_ = 0;
    u64 frames_drawn_ = 0;
    /// --run self test: machine steps to execute before the first frame.
    int step_limit_ = 0;
    /// --screenshot <file.bmp>: capture mode. Works without any display.
    std::string screenshot_path_;
    bool screenshot_requested_ = false;
    /// True when the renderer runs offscreen (no window): the canvas is the output.
    bool offscreen_ = false;
    /// --screenshot-tab console: open the capture with the command line focused.
    bool focus_console_at_start_ = false;
    /// --stage <first|second|kbl|kernel>: enter a boot stage before capturing.
    std::string start_stage_;
    /// --exec <command>: debugger commands run before the capture.
    std::vector<std::string> pre_commands_;

    /// Keeps the log sink from touching the console after the app is gone.
    std::shared_ptr<bool> log_alive_;

    std::string startup_note_ = "window ok";
    bool window_ok_ = false;
    bool quiet_ = false;
    bool sdl_video_ok_ = false;
    bool sdl_audio_ok_ = false;
    bool sdl_gamepad_ok_ = false;
};

}  // namespace zlb
