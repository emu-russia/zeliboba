// zeliboba - SDL3 frontend: window, event loop, emulation frame, status bar.
//
// This is `int main` for the zeliboba_ui target. It can also act as the console
// frontend: `--headless`, `-ex` and `--script` are forwarded to zlb::cli_main.
//
// Frame shape:
//   1. pump SDL events            (input, pause, tabs, console)
//   2. run_slice() until either 1/60 s of emulated time or a real time budget
//      is used, stopping on a breakpoint so the UI stays responsive
//   3. refresh the active panel from the Debugger/Vita API
//   4. draw everything into the ARGB8888 canvas and blit it to the window
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "common/util.h"
#include "debug/cli.h"
#include "hw/emmc.h"
#include "hw/soc.h"
#include "ui/ui.h"
#include "ui/zeliboba_icon_pixels.h"

namespace zlb {

namespace {

using namespace ui_layout;

/// Real time the emulator may spend in one frame before the UI gets its turn.
constexpr u64 kFrameWallBudgetNs = 8'000'000ull;  // 8 ms of a ~16.6 ms frame
constexpr int kMaxSlicesPerFrame = 1 << 19;
constexpr int kSlicesBetweenWallChecks = 8;

SDL_FRect canvas_destination(SDL_Window* window, bool in_pixels) {
    int ww = kCanvasWidth;
    int wh = kCanvasHeight;
    if (in_pixels) SDL_GetWindowSizeInPixels(window, &ww, &wh);
    else SDL_GetWindowSize(window, &ww, &wh);
    if (ww <= 0 || wh <= 0) return SDL_FRect{0.0f, 0.0f, static_cast<float>(kCanvasWidth),
                                            static_cast<float>(kCanvasHeight)};
    const double scale = std::min(ww / static_cast<double>(kCanvasWidth),
                                  wh / static_cast<double>(kCanvasHeight));
    const float dw = static_cast<float>(kCanvasWidth * scale);
    const float dh = static_cast<float>(kCanvasHeight * scale);
    return SDL_FRect{(ww - dw) * 0.5f, (wh - dh) * 0.5f, dw, dh};
}

u32 color_for_log(LogLevel level) {
    switch (level) {
        case LogLevel::Warn: return ui_theme::kWarn;
        case LogLevel::Error: return ui_theme::kError;
        case LogLevel::Info: return ui_theme::kTextDim;
        default: return ui_theme::kTextDim;
    }
}

/// `--tab` accepts a name ("disassembly", "display", ...) or 1..7.
UiTab parse_tab(const std::string& text) {
    const std::string name = to_lower(trim(text));
    if (name == "disasm" || name == "disassembly" || name == "code" || name == "1") return UiTab::Disassembly;
    if (name == "registers" || name == "regs" || name == "2") return UiTab::Registers;
    if (name == "memory" || name == "mem" || name == "3") return UiTab::Memory;
    if (name == "trace" || name == "4") return UiTab::Trace;
    if (name == "devices" || name == "dev" || name == "5") return UiTab::Devices;
    if (name == "boot" || name == "6") return UiTab::Boot;
    if (name == "panel" || name == "display" || name == "7") return UiTab::Display;
    return UiTab::Disassembly;
}

}  // namespace

// ---------------------------------------------------------------------------
// Panels
// ---------------------------------------------------------------------------

const char* tab_name(UiTab tab) {
    switch (tab) {
        case UiTab::Disassembly: return "Disassembly";
        case UiTab::Registers: return "Registers";
        case UiTab::Memory: return "Memory";
        case UiTab::Trace: return "Trace";
        case UiTab::Devices: return "Devices";
        case UiTab::Boot: return "Boot";
        case UiTab::Display: return "Display";
        default: return "?";
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int ui_main(int argc, char** argv) {
    // Anything scripted belongs to the console frontend: it is the same
    // debugger, but it works without a window.
    std::vector<std::string> args;
    args.reserve(static_cast<size_t>(argc));
    for (int i = 0; i < argc; ++i) args.emplace_back(argv[i]);

    // `-ex`/`--script`/`--headless` mean "console frontend" - unless a screenshot
    // was asked for, in which case the commands run as pre-commands inside the UI
    // (that is how a screenshot of a *specific* state is taken).
    bool screenshot_wanted = false;
    for (size_t i = 1; i < args.size(); ++i) {
        if (args[i] == "--screenshot") screenshot_wanted = true;
    }
    for (size_t i = 1; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (screenshot_wanted) break;
        if (arg == "--headless" || arg == "-ex" || arg == "--exec" || arg == "--script") {
            std::vector<char*> forwarded;
            forwarded.reserve(args.size());
            for (std::string& text : args) {
                if (text == "--headless") continue;
                forwarded.push_back(text.data());
            }
            forwarded.push_back(nullptr);
            return cli_main(static_cast<int>(forwarded.size()) - 1, forwarded.data());
        }
    }

    UiApp app;
    return app.run(argc, argv);
}

// ---------------------------------------------------------------------------
// VitaInput
// ---------------------------------------------------------------------------

bool VitaInput::any_pressed() const {
    return up || down || left || right || cross || circle || square || triangle || start || select || l1 || r1;
}

void VitaInput::clear_buttons() {
    up = down = left = right = false;
    cross = circle = square = triangle = false;
    start = select = l1 = r1 = false;
}

std::string VitaInput::status() const {
    std::string out;
    auto add = [&out](const char* name) {
        if (!out.empty()) out += '|';
        out += name;
    };
    if (up) add("UP");
    if (down) add("DOWN");
    if (left) add("LEFT");
    if (right) add("RIGHT");
    if (cross) add("CROSS");
    if (circle) add("CIRCLE");
    if (square) add("SQUARE");
    if (triangle) add("TRIANGLE");
    if (start) add("START");
    if (select) add("SELECT");
    if (l1) add("L");
    if (r1) add("R");
    if (out.empty()) out = "-";
    out += format(" L(%+.2f,%+.2f) R(%+.2f,%+.2f)", left_x, left_y, right_x, right_y);
    return out;
}

bool vita_input_from_key(SDL_Scancode scancode, bool down, VitaInput& input) {
    bool* target = nullptr;
    switch (scancode) {
        case SDL_SCANCODE_UP: target = &input.up; break;
        case SDL_SCANCODE_DOWN: target = &input.down; break;
        case SDL_SCANCODE_LEFT: target = &input.left; break;
        case SDL_SCANCODE_RIGHT: target = &input.right; break;
        case SDL_SCANCODE_Z: target = &input.cross; break;
        case SDL_SCANCODE_X: target = &input.circle; break;
        case SDL_SCANCODE_A: target = &input.square; break;
        case SDL_SCANCODE_S: target = &input.triangle; break;
        case SDL_SCANCODE_Q: target = &input.l1; break;
        case SDL_SCANCODE_E: target = &input.r1; break;
        case SDL_SCANCODE_RETURN: target = &input.start; break;
        case SDL_SCANCODE_KP_ENTER: target = &input.start; break;
        case SDL_SCANCODE_RSHIFT: target = &input.select; break;
        default: return false;
    }
    *target = down;
    input.source = "keyboard";
    return true;
}

// ---------------------------------------------------------------------------
// Canvas
// ---------------------------------------------------------------------------

Canvas::~Canvas() { destroy(); }

bool Canvas::create(int width, int height) {
    destroy();
    surface_ = SDL_CreateSurface(width, height, SDL_PIXELFORMAT_ARGB8888);
    if (!surface_) return false;
    pixels_ = static_cast<u32*>(surface_->pixels);
    width_ = surface_->w;
    height_ = surface_->h;
    pitch_ = surface_->pitch;
    return true;
}

void Canvas::destroy() {
    if (surface_) SDL_DestroySurface(surface_);
    surface_ = nullptr;
    pixels_ = nullptr;
    width_ = height_ = pitch_ = 0;
}

void Canvas::clear(u32 argb) {
    if (!pixels_) return;
    const int words = pitch_ / 4;
    for (int y = 0; y < height_; ++y) {
        u32* row = pixels_ + static_cast<size_t>(y) * static_cast<size_t>(words);
        for (int x = 0; x < width_; ++x) row[x] = argb;
    }
}

void Canvas::pixel(int x, int y, u32 argb) {
    if (!pixels_ || x < 0 || y < 0 || x >= width_ || y >= height_) return;
    pixels_[static_cast<size_t>(y) * static_cast<size_t>(pitch_ / 4) + static_cast<size_t>(x)] = argb;
}

void Canvas::fill(const UiRect& area, u32 argb) {
    if (!pixels_) return;
    const int x0 = std::max(0, area.x);
    const int y0 = std::max(0, area.y);
    const int x1 = std::min(width_, area.right());
    const int y1 = std::min(height_, area.bottom());
    for (int y = y0; y < y1; ++y) {
        u32* row = pixels_ + static_cast<size_t>(y) * static_cast<size_t>(pitch_ / 4);
        for (int x = x0; x < x1; ++x) row[x] = argb;
    }
}

void Canvas::outline(const UiRect& area, u32 argb) {
    hline(area.x, area.y, area.w, argb);
    hline(area.x, area.bottom() - 1, area.w, argb);
    vline(area.x, area.y, area.h, argb);
    vline(area.right() - 1, area.y, area.h, argb);
}

void Canvas::hline(int x, int y, int width, u32 argb) {
    for (int i = 0; i < width; ++i) pixel(x + i, y, argb);
}

void Canvas::vline(int x, int y, int height, u32 argb) {
    for (int i = 0; i < height; ++i) pixel(x, y + i, argb);
}

void Canvas::draw_char(int x, int y, char c, u32 argb, int scale) {
    if (!pixels_ || scale <= 0) return;
    const u8* rows = font::glyph(c);
    for (int gy = 0; gy < font::kGlyphHeight; ++gy) {
        const u8 bits = rows[gy];
        if (bits == 0) continue;
        for (int gx = 0; gx < font::kGlyphWidth; ++gx) {
            if ((bits & (1u << gx)) == 0) continue;
            if (scale == 1) {
                pixel(x + gx, y + gy, argb);
            } else {
                fill(UiRect{x + gx * scale, y + gy * scale, scale, scale}, argb);
            }
        }
    }
}

void Canvas::draw_text(int x, int y, const std::string& text, u32 argb, int scale) {
    int pen = x;
    for (char c : text) {
        draw_char(pen, y, c, argb, scale);
        pen += font::kGlyphWidth * scale;
    }
}

void Canvas::draw_text_clip(const UiRect& clip, int x, int y, const std::string& text, u32 argb, int scale) {
    const int height = font::kGlyphHeight * scale;
    if (y + height <= clip.y || y >= clip.bottom()) return;
    int pen = x;
    for (char c : text) {
        if (pen >= clip.right()) return;
        if (pen + font::kGlyphWidth * scale > clip.x) draw_char(pen, y, c, argb, scale);
        pen += font::kGlyphWidth * scale;
    }
}

void Canvas::draw_text_fit(const UiRect& clip, int x, int y, const std::string& text, u32 argb, int scale) {
    const size_t room = static_cast<size_t>(std::max(0, clip.right() - x)) /
                        static_cast<size_t>(font::kGlyphWidth * scale);
    if (text.size() <= room) {
        draw_text_clip(clip, x, y, text, argb, scale);
        return;
    }
    if (room <= 1) return;
    std::string cut = text.substr(0, room - 1) + "~";
    draw_text_clip(clip, x, y, cut, argb, scale);
}

// ---------------------------------------------------------------------------
// UiApp: construction and lifecycle
// ---------------------------------------------------------------------------

UiApp::UiApp() : debugger_(vita_) {}

UiApp::~UiApp() { shutdown(); }

int UiApp::run(int argc, char** argv) {
    if (!init_machine(argc, argv)) return 2;
    if (!init_window()) {
        shutdown();
        return 1;
    }

    // init_window() already reported the exact window/audio results on stdout.
    console_print(startup_note_, offscreen_ ? ui_theme::kWarn : ui_theme::kOk);
    console_print(audio_.summary(), audio_.available() ? ui_theme::kOk : ui_theme::kWarn);
    console_print(format("gamepads: %zu connected", gamepads_.size()), ui_theme::kTextDim);
    console_print("F1..F6 panels, F7 display, F9 audio test tone, SPACE pause, n step, g go",
                  ui_theme::kTextDim);

    // Screenshot mode: bounded emulated work first, so the panels have content.
    if (screenshot_requested_) {
        if (focus_console_at_start_) console_.set_focused(true);
        // --exec commands first (a scripted setup), then --stage, then --run.
        for (const std::string& command : pre_commands_) execute_command(command);
        if (!start_stage_.empty()) {
            execute_command("stage " + start_stage_);
            std::printf("zeliboba_ui: entered stage '%s' -> %s\n", start_stage_.c_str(),
                        to_string(vita_.stage()));
        }
        if (step_limit_ > 0) {
            console_.set_focused(false);
            execute_command(format("run %d", step_limit_));
            if (focus_console_at_start_) console_.set_focused(true);
            std::printf("zeliboba_ui: ran %d machine steps before capture (stage %s)\n", step_limit_,
                        to_string(vita_.stage()));
            std::fflush(stdout);
        }
    }

    main_loop();

    int exit_code = 0;
    if (screenshot_requested_) {
        exit_code = 3;
        if (!canvas_.valid()) {
            std::fprintf(stderr, "screenshot failed: the %dx%d ARGB8888 canvas is not available\n",
                         kCanvasWidth, kCanvasHeight);
        } else if (screenshot_path_.empty()) {
            std::fprintf(stderr, "screenshot failed: no output path given to --screenshot\n");
        } else if (!SDL_SaveBMP(canvas_.surface(), screenshot_path_.c_str())) {
            std::fprintf(stderr, "screenshot failed: cannot write '%s': %s\n", screenshot_path_.c_str(),
                         SDL_GetError());
        } else {
            std::printf("screenshot written: %s (%dx%d)\n", screenshot_path_.c_str(), canvas_.width(),
                        canvas_.height());
            exit_code = 0;
        }
        std::fflush(stdout);
        std::fflush(stderr);
    }

    shutdown();
    return exit_code;
}

bool UiApp::init_machine(int argc, char** argv) {
    Log::instance().set_level(LogLevel::Info);

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "zeliboba_ui: %s requires an argument\n", what);
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--emmc") config_.emmc_image = next("--emmc");
        else if (arg == "--first-loader") config_.first_loader = next("--first-loader");
        else if (arg == "--syscon") config_.syscon_firmware = next("--syscon");
        else if (arg == "--fs") config_.fs_root = next("--fs");
        else if (arg == "--no-syscon") config_.run_syscon_firmware = false;
        else if (arg == "--no-rebuild") config_.rebuild_emmc = false;
        else if (arg == "--log") {
            LogLevel level = LogLevel::Info;
            const std::string value = next("--log");
            if (parse_log_level(value, level)) Log::instance().set_level(level);
            else std::fprintf(stderr, "zeliboba_ui: unknown log level '%s'\n", value.c_str());
        } else if (arg == "--verbose") {
            Log::instance().set_level(LogLevel::Debug);
        } else if (arg == "-q" || arg == "--quiet") {
            quiet_ = true;
        } else if (arg == "--frames" || arg == "--screenshot-frames") {
            const std::string value = next("--screenshot-frames");
            frame_limit_ = std::max(1, std::atoi(value.c_str()));
        } else if (arg == "--run") {
            step_limit_ = std::max(0, std::atoi(next("--run").c_str()));
        } else if (arg == "--stage") {
            start_stage_ = to_lower(trim(next("--stage")));
        } else if (arg == "--core") {
            pre_commands_.push_back("core " + to_lower(trim(next("--core"))));
        } else if (arg == "--exec" || arg == "-ex") {
            pre_commands_.push_back(next("--exec"));
        } else if (arg == "--screenshot") {
            screenshot_path_ = next("--screenshot");
            screenshot_requested_ = true;
        } else if (arg == "--screenshot-tab" || arg == "--tab") {
            const std::string name = to_lower(trim(next("--screenshot-tab")));
            if (name == "console") {
                // The console lives under every panel, so capture the code view
                // with the command line focused.
                tab_ = UiTab::Disassembly;
                focus_console_at_start_ = true;
            } else {
                tab_ = parse_tab(name);
            }
        } else if (arg == "-h" || arg == "--help") {
            std::printf(
                "zeliboba_ui - SDL3 frontend for zeliboba\n"
                "\n"
                "  --headless             run the console frontend instead\n"
                "  -ex <command>          console frontend: run a debugger command\n"
                "  --script <file>        console frontend: run a command file\n"
                "\n"
                "screenshot mode (works with no display at all):\n"
                "  --screenshot <file.bmp>     write the rendered frame to a BMP and exit\n"
                "  --screenshot-frames <n>     frames to render first (default 2)\n"
                "  --screenshot-tab <name>     disasm|registers|memory|trace|devices|boot|panel|console\n"
                "  --run <n>                   machine steps before the first frame (default 200000)\n"
                "\n"
                "  --frames <n>           draw n frames then exit (same as --screenshot-frames)\n"
                "  --tab <name|1..7>      start on that panel\n"
                "  --stage <name>         start at first|second|kbl|kernel\n"
                "  --core <name>          active core at startup: mep|arm|rl78\n"
                "  -ex <command>          run a debugger command before the first frame\n"
                "  --emmc <file> --first-loader <file> --syscon <file> --fs <dir>\n"
                "  --no-syscon --no-rebuild --log <level> --verbose -q\n"
                "  -h, --help             this text\n"
                "\n"
                "keys: F1..F6 panels, F7 display, F8/SPACE pause, F9 audio test tone,\n"
                "      F10 step over, F11 step into, F12 reset, n step, g go,\n"
                "      Enter breakpoint (disassembly) / console, ` console, Esc quit\n");
            std::exit(0);
        } else if (!arg.empty() && arg[0] == '-') {
            std::fprintf(stderr, "zeliboba_ui: ignoring unknown option '%s'\n", arg.c_str());
        }
    }

    // Screenshot mode defaults: render a couple of frames after running a
    // bounded amount of emulated work, then write the BMP and exit.
    if (screenshot_requested_) {
        if (frame_limit_ <= 0) frame_limit_ = 2;
        if (step_limit_ < 0) step_limit_ = 200000;
    }

    // The UI console is the frontend's log, so the console gets its own sink.
    log_alive_ = std::make_shared<bool>(true);
    std::shared_ptr<bool> alive = log_alive_;
    Log::instance().add_sink([this, alive](const LogRecord& record) {
        if (!*alive) return;
        if (record.level < LogLevel::Info) return;
        console_.add_chunk(format("[%-5s] %-9s %s", to_string(record.level), record.category.c_str(),
                                  record.text.c_str()),
                           color_for_log(record.level));
    });

    vita_.build(config_);
    vita_.reset(true);

    debugger_.set_output([this](const std::string& text) { console_.add_chunk(text, ui_theme::kText); });

    console_print("zeliboba - PlayStation Vita emulator (SDL3 frontend)", ui_theme::kAccent);
    if (!quiet_) {
        console_print(format("workspace: %s", workspace_root().c_str()), ui_theme::kTextDim);
        console_print(format("stage: %s", to_string(vita_.stage())), ui_theme::kTextDim);
    }
    console_print("machine is paused; press SPACE to run, n to step", ui_theme::kTextDim);
    return true;
}

bool UiApp::init_window() {
    SDL_SetAppMetadata("zeliboba", "0.1", "zlb.zeliboba.ui");

    const SDL_InitFlags flags = SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMEPAD;
    const bool init_ok = SDL_Init(flags);
    std::string init_error = init_ok ? std::string("ok") : std::string(SDL_GetError());

    // Even when the combined SDL_Init fails, report exactly which subsystems are
    // usable: a session without a gamepad (or without audio) is still usable.
    sdl_video_ok_ = SDL_WasInit(SDL_INIT_VIDEO) != 0;
    if (!sdl_video_ok_ && SDL_InitSubSystem(SDL_INIT_VIDEO)) sdl_video_ok_ = true;
    sdl_audio_ok_ = SDL_WasInit(SDL_INIT_AUDIO) != 0;
    if (!sdl_audio_ok_ && SDL_InitSubSystem(SDL_INIT_AUDIO)) sdl_audio_ok_ = true;
    sdl_gamepad_ok_ = SDL_WasInit(SDL_INIT_GAMEPAD) != 0;
    if (!sdl_gamepad_ok_ && SDL_InitSubSystem(SDL_INIT_GAMEPAD)) sdl_gamepad_ok_ = true;

    std::printf("zeliboba_ui: SDL_Init(VIDEO|AUDIO|GAMEPAD) = %s\n", init_error.c_str());
    std::printf("zeliboba_ui: subsystems video=%d audio=%d gamepad=%d\n", sdl_video_ok_ ? 1 : 0,
                sdl_audio_ok_ ? 1 : 0, sdl_gamepad_ok_ ? 1 : 0);
    std::fflush(stdout);

    // The canvas is created first: it is the framebuffer, and it is the only
    // thing the screenshot mode needs.
    if (!canvas_.create(kCanvasWidth, kCanvasHeight)) {
        std::printf("zeliboba_ui: SDL_CreateSurface(%dx%d, ARGB8888) failed: %s\n", kCanvasWidth,
                    kCanvasHeight, SDL_GetError());
        startup_note_ = std::string("SDL_CreateSurface failed: ") + SDL_GetError();
        return false;
    }

    if (!sdl_video_ok_) {
        const std::string reason = std::string("no video subsystem (") + SDL_GetError() + ")";
        if (screenshot_requested_) return enable_offscreen(reason);
        std::printf("zeliboba_ui: %s\n", reason.c_str());
        std::printf("zeliboba_ui: use --headless (or -ex/--script) for the console frontend,\n");
        std::printf("zeliboba_ui: or --screenshot <file.bmp> to render a frame without a display\n");
        startup_note_ = reason;
        return false;
    }

    window_ = SDL_CreateWindow("zeliboba - PlayStation Vita (CMeP / Cortex-A9 / Ernie)", kCanvasWidth,
                               kCanvasHeight, SDL_WINDOW_RESIZABLE);
    if (!window_) {
        const std::string reason = std::string("SDL_CreateWindow failed: ") + SDL_GetError();
        if (screenshot_requested_) return enable_offscreen(reason);
        std::printf("zeliboba_ui: %s\n", reason.c_str());
        std::printf("zeliboba_ui: use --headless for the console frontend, or --screenshot for an "
                    "offscreen capture\n");
        startup_note_ = reason;
        return false;
    }

    // Zeliboba, the blue fur spirit of the yard: the window/taskbar icon from
    // artwork/make_artwork.py (the pixels are embedded, no file to find).
    if (SDL_Surface* icon = SDL_CreateSurfaceFrom(
            ui_artwork::kWindowIconWidth, ui_artwork::kWindowIconHeight,
            SDL_PIXELFORMAT_ARGB8888,
            const_cast<u32*>(ui_artwork::kWindowIconPixels),
            ui_artwork::kWindowIconWidth * static_cast<int>(sizeof(u32)))) {
        SDL_SetWindowIcon(window_, icon);
        SDL_DestroySurface(icon);
    } else {
        std::printf("zeliboba_ui: window icon unavailable: %s\n", SDL_GetError());
    }

    renderer_ = SDL_CreateRenderer(window_, nullptr);
    if (!renderer_) {
        const std::string reason = std::string("SDL_CreateRenderer failed: ") + SDL_GetError();
        if (screenshot_requested_) return enable_offscreen(reason);
        std::printf("zeliboba_ui: %s\n", reason.c_str());
        startup_note_ = reason;
        return false;
    }
    const char* renderer_name = SDL_GetRendererName(renderer_);

    const bool vsync_ok = SDL_SetRenderVSync(renderer_, 1);
    if (!vsync_ok) {
        std::printf("zeliboba_ui: SDL_SetRenderVSync(1) failed: %s (running without vsync)\n",
                    SDL_GetError());
    }

    texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                 kCanvasWidth, kCanvasHeight);
    if (!texture_) {
        const std::string reason = std::string("SDL_CreateTexture failed: ") + SDL_GetError();
        if (screenshot_requested_) return enable_offscreen(reason);
        std::printf("zeliboba_ui: %s\n", reason.c_str());
        startup_note_ = reason;
        return false;
    }
    SDL_SetTextureScaleMode(texture_, SDL_SCALEMODE_NEAREST);
    SDL_StartTextInput(window_);

    open_gamepads();
    audio_.open();

    window_ok_ = true;
    startup_note_ = format("window %dx%d resizable ok (renderer %s, vsync %s) canvas %dx%d ARGB8888",
                           kCanvasWidth, kCanvasHeight, renderer_name ? renderer_name : "?",
                           vsync_ok ? "on" : "off", kCanvasWidth, kCanvasHeight);
    std::printf("zeliboba_ui: %s\n", startup_note_.c_str());
    std::printf("zeliboba_ui: %s\n", audio_.status().c_str());
    std::fflush(stdout);
    return true;
}

bool UiApp::enable_offscreen(const std::string& reason) {
    // No window: keep rendering into the ARGB8888 canvas and save that. This is
    // what makes --screenshot work on a machine with no display session.
    if (texture_) {
        SDL_DestroyTexture(texture_);
        texture_ = nullptr;
    }
    if (renderer_) {
        SDL_DestroyRenderer(renderer_);
        renderer_ = nullptr;
    }
    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
    offscreen_ = true;
    window_ok_ = true;  // shutdown() must still call SDL_Quit()
    startup_note_ = format("offscreen mode: %s - drawing into the %dx%d ARGB8888 canvas",
                           reason.c_str(), kCanvasWidth, kCanvasHeight);
    std::printf("zeliboba_ui: %s\n", startup_note_.c_str());

    // Debug output, audio and scripted input still work without a window.
    open_gamepads();
    audio_.open();
    std::printf("zeliboba_ui: %s\n", audio_.status().c_str());
    std::fflush(stdout);
    return true;
}

void UiApp::shutdown() {
    if (log_alive_) *log_alive_ = false;
    if (texture_) {
        SDL_DestroyTexture(texture_);
        texture_ = nullptr;
    }
    if (renderer_) {
        SDL_DestroyRenderer(renderer_);
        renderer_ = nullptr;
    }
    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
    canvas_.destroy();
    close_gamepads();
    audio_.close();
    if (window_ok_ || sdl_video_ok_) SDL_Quit();
    window_ok_ = false;
}

void UiApp::open_gamepads() {
    if (!sdl_gamepad_ok_) return;
    int count = 0;
    SDL_JoystickID* ids = SDL_GetGamepads(&count);
    if (!ids) return;
    for (int i = 0; i < count; ++i) {
        SDL_Gamepad* pad = SDL_OpenGamepad(ids[i]);
        if (!pad) {
            std::printf("zeliboba_ui: SDL_OpenGamepad(%d) failed: %s\n", ids[i], SDL_GetError());
            continue;
        }
        gamepads_.push_back(pad);
        const char* name = SDL_GetGamepadName(pad);
        std::printf("zeliboba_ui: gamepad %zu: %s\n", gamepads_.size() - 1, name ? name : "?");
    }
    SDL_free(ids);
    input_.gamepad_connected = !gamepads_.empty();
    if (!gamepads_.empty()) {
        const char* name = SDL_GetGamepadName(gamepads_.front());
        input_note_ = name ? name : "gamepad";
    }
}

void UiApp::close_gamepads() {
    for (SDL_Gamepad* pad : gamepads_) SDL_CloseGamepad(pad);
    gamepads_.clear();
    input_.gamepad_connected = false;
}

// ---------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------

void UiApp::main_loop() {
    while (!quit_) {
        pump_events();
        if (quit_) break;
        run_frame();
        refresh_views();
        render();
        ++frames_drawn_;
        if (frame_limit_ > 0 && frames_drawn_ >= static_cast<u64>(frame_limit_)) {
            std::printf("zeliboba_ui: --frames %d reached, exiting\n", frame_limit_);
            std::fflush(stdout);
            break;
        }
    }
}

void UiApp::pump_events() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        handle_event(event);
        if (quit_) return;
    }
}

void UiApp::handle_event(const SDL_Event& event) {
    switch (event.type) {
        case SDL_EVENT_QUIT:
            quit_ = true;
            break;
        case SDL_EVENT_KEY_DOWN:
        case SDL_EVENT_KEY_UP:
            handle_key(event.key.key, event.key.scancode, event.type == SDL_EVENT_KEY_DOWN, event.key.mod);
            break;
        case SDL_EVENT_TEXT_INPUT:
            handle_text_input(event.text.text);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            handle_mouse_button(event.button, true);
            break;
        case SDL_EVENT_MOUSE_BUTTON_UP:
            handle_mouse_button(event.button, false);
            break;
        case SDL_EVENT_MOUSE_MOTION:
            handle_mouse_motion(event.motion);
            break;
        case SDL_EVENT_MOUSE_WHEEL:
            handle_mouse_wheel(event.wheel);
            break;
        case SDL_EVENT_GAMEPAD_ADDED:
        case SDL_EVENT_GAMEPAD_REMOVED:
        case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
        case SDL_EVENT_GAMEPAD_BUTTON_UP:
        case SDL_EVENT_GAMEPAD_AXIS_MOTION:
            handle_gamepad_event(event);
            break;
        default:
            break;
    }
}

void UiApp::handle_key(SDL_Keycode key, SDL_Scancode scancode, bool down, SDL_Keymod mod) {
    suppress_char_ = 0;
    if (!down) {
        // Releases only matter for the pad.
        if (scancode != SDL_SCANCODE_UNKNOWN) sync_input_from_keyboard(scancode, false);
        return;
    }

    // A shortcut that consumes a printable key arms suppress_char_ so that the
    // SDL_EVENT_TEXT_INPUT which follows it is dropped.
    auto consumed = [this, key]() {
        if (key > 32 && key < 127) suppress_char_ = static_cast<char>(std::tolower(static_cast<int>(key)));
    };

    // 1. Function keys and the console toggle are never text: they work whatever
    //    has the focus.
    if (key >= SDLK_F1 && key <= SDLK_F7) {
        tab_ = static_cast<UiTab>(static_cast<int>(key - SDLK_F1));
        return;
    }
    switch (key) {
        case SDLK_F8:
            toggle_pause();
            return;
        case SDLK_F9:
            audio_.set_test_tone(!audio_.test_tone());
            console_print(audio_.test_tone() ? "audio test tone on (440 Hz)" : "audio test tone off",
                          ui_theme::kTextDim);
            return;
        case SDLK_F10:
            step_over();
            return;
        case SDLK_F11:
            step_into();
            return;
        case SDLK_F12:
            debugger_.execute("reset");
            return;
        case SDLK_GRAVE:
            if (!console_.focused()) {
                console_.set_focused(true);
                consumed();
            }
            return;
        default:
            break;
    }

    // 2. The console has the keyboard while it is focused. Printable characters
    //    arrive through SDL_EVENT_TEXT_INPUT, so nothing is suppressed here.
    if (console_.focused()) {
        if (key == SDLK_ESCAPE) {
            console_.set_focused(false);
        } else {
            console_.handle_key(key, scancode, mod, debugger_);
            std::string submitted;
            if (console_.take_submitted(submitted)) execute_command(submitted);
        }
        return;
    }

    // 3. The inline address editor.
    if (edit_target_ != EditTarget::None) {
        editor_key(key, mod);
        return;
    }

    switch (key) {
        case SDLK_ESCAPE:
            quit_ = true;
            return;
        case SDLK_SPACE:
            toggle_pause();
            consumed();
            return;
        case SDLK_TAB:
            tab_ = static_cast<UiTab>((static_cast<int>(tab_) + 1) % static_cast<int>(UiTab::Count));
            return;
        case SDLK_RETURN:
        case SDLK_KP_ENTER:
            if (tab_ == UiTab::Disassembly) toggle_breakpoint_at_selection();
            else console_.set_focused(true);
            return;
        default:
            break;
    }

    // 4. Panel navigation and the single key debugger shortcuts.
    const int page = std::max(1, panel_rect().h / kLineHeight - 2);
    switch (key) {
        case SDLK_N:
            step_into();
            consumed();
            return;
        case SDLK_G:
            set_running(true);
            consumed();
            return;
        case SDLK_J:
            if (tab_ == UiTab::Memory) open_editor(EditTarget::GotoMemory, format("%08X", memory_.address));
            else open_editor(EditTarget::GotoDisassembly, format("%08X", disasm_.address));
            consumed();
            return;
        case SDLK_F:
            if (tab_ == UiTab::Disassembly) {
                disasm_.follow_pc = !disasm_.follow_pc;
                console_print(disasm_.follow_pc ? "disassembly follows the PC" : "disassembly is detached",
                              ui_theme::kTextDim);
            }
            consumed();
            return;
        case SDLK_UP:
            if (tab_ == UiTab::Disassembly) {
                if (disasm_.selected > 0) --disasm_.selected;
                else {
                    disasm_.address -= 4;
                    disasm_.follow_pc = false;
                }
            } else if (tab_ == UiTab::Memory) {
                memory_.selected_row = std::max(0, memory_.selected_row - 1);
            } else if (tab_ == UiTab::Trace) {
                trace_.scroll = std::max(0, trace_.scroll - 1);
            } else if (tab_ == UiTab::Devices) {
                select_device(devices_.selected - 1);
            } else if (tab_ == UiTab::Registers) {
                registers_.scroll = std::max(0, registers_.scroll - 1);
            }
            return;
        case SDLK_DOWN:
            if (tab_ == UiTab::Disassembly) {
                if (disasm_.selected + 1 < static_cast<int>(disasm_.lines.size())) ++disasm_.selected;
                else disasm_.address += 4;
                disasm_.follow_pc = false;
            } else if (tab_ == UiTab::Memory) {
                memory_.selected_row = std::min(std::max(0, memory_.rows - 1), memory_.selected_row + 1);
            } else if (tab_ == UiTab::Trace) {
                trace_.scroll = std::min(static_cast<int>(trace_.rows.size()), trace_.scroll + 1);
            } else if (tab_ == UiTab::Devices) {
                select_device(devices_.selected + 1);
            } else if (tab_ == UiTab::Registers) {
                registers_.scroll += 1;
            }
            return;
        case SDLK_PAGEUP:
            if (tab_ == UiTab::Memory) memory_.address -= static_cast<u32>(page * memory_.bytes_per_row);
            else if (tab_ == UiTab::Trace) trace_.scroll = std::min(static_cast<int>(trace_.rows.size()),
                                                                    trace_.scroll + page);
            else if (tab_ == UiTab::Devices) {
                devices_.register_scroll = std::max(0, devices_.register_scroll - page);
            } else {
                disasm_.address -= static_cast<u32>(page * 4);
                disasm_.follow_pc = false;
            }
            return;
        case SDLK_PAGEDOWN:
            if (tab_ == UiTab::Memory) memory_.address += static_cast<u32>(page * memory_.bytes_per_row);
            else if (tab_ == UiTab::Trace) trace_.scroll = std::max(0, trace_.scroll - page);
            else if (tab_ == UiTab::Devices) devices_.register_scroll += page;
            else {
                disasm_.address += static_cast<u32>(page * 4);
                disasm_.follow_pc = false;
            }
            return;
        case SDLK_HOME:
            if (tab_ == UiTab::Disassembly) {
                disasm_.follow_pc = true;
                disasm_.selected = 0;
            } else if (tab_ == UiTab::Memory) {
                Cpu* cpu = debugger_.active_core();
                if (cpu) memory_.address = cpu->get_pc();
            } else if (tab_ == UiTab::Trace) {
                trace_.scroll = 0;
            }
            return;
        default:
            break;
    }

    // 5. Boot stage shortcuts.
    if (tab_ == UiTab::Boot && key >= SDLK_1 && key <= SDLK_4) {
        static const char* stages[4] = {"first", "second", "kbl", "kernel"};
        execute_command(format("stage %s", stages[key - SDLK_1]));
        consumed();
        return;
    }

    // 6. Anything else that belongs to the emulated pad.
    if (vita_input_from_key(scancode, true, input_)) consumed();
}

void UiApp::handle_text_input(const char* text) {
    if (!text || text[0] == '\0') return;
    if (suppress_char_ != 0) {
        const char first = static_cast<char>(std::tolower(static_cast<unsigned char>(text[0])));
        const bool matches = (text[1] == '\0' && first == suppress_char_);
        suppress_char_ = 0;
        if (matches) return;
    }
    if (edit_target_ != EditTarget::None) {
        edit_text_ += text;
        edit_cursor_ = edit_text_.size();
        return;
    }
    console_.set_focused(true);
    console_.insert_text(text);
}

void UiApp::handle_mouse_button(const SDL_MouseButtonEvent& button, bool down) {
    int x = 0;
    int y = 0;
    if (!window_to_canvas(button.x, button.y, x, y)) return;
    mouse_x_ = x;
    mouse_y_ = y;
    if (!down) return;
    if (button.button != SDL_BUTTON_LEFT) return;

    // Tab bar.
    for (int i = 0; i < static_cast<int>(UiTab::Count); ++i) {
        if (tab_rect(i).contains(x, y)) {
            handle_tab_click(i);
            return;
        }
    }

    const UiRect console = console_rect();
    if (console.contains(x, y)) {
        console_.set_focused(true);
        console_.scroll_to_bottom();
        return;
    }
    console_.set_focused(false);

    const UiRect panel = panel_rect();
    if (!panel.contains(x, y)) return;

    switch (tab_) {
        case UiTab::Disassembly: {
            // Row 0 is the first instruction, which the panel draws at area.y+20.
            const int row = (y - (panel.y + 20)) / kLineHeight;
            if (row >= 0 && row < static_cast<int>(disasm_.lines.size())) {
                disasm_.selected = row;
                disasm_.follow_pc = false;
            }
            break;
        }
        case UiTab::Memory: {
            const int row = (y - (panel.y + 20)) / kLineHeight;
            if (row >= 0 && row < memory_.rows) memory_.selected_row = row;
            break;
        }
        case UiTab::Devices: {
            const int row = (y - (panel.y + 18)) / kLineHeight;
            const int index = devices_.scroll + row;
            if (row >= 0 && index < static_cast<int>(devices_.entries.size())) select_device(index);
            break;
        }
        case UiTab::Boot: {
            for (size_t i = 0; i < boot_.button_rects.size(); ++i) {
                if (boot_.button_rects[i].contains(x, y)) {
                    execute_command("stage " + boot_.button_labels[i]);
                    break;
                }
            }
            break;
        }
        default:
            break;
    }
}

void UiApp::handle_mouse_motion(const SDL_MouseMotionEvent& motion) {
    int x = 0;
    int y = 0;
    if (window_to_canvas(motion.x, motion.y, x, y)) {
        mouse_x_ = x;
        mouse_y_ = y;
    }
}

void UiApp::handle_mouse_wheel(const SDL_MouseWheelEvent& wheel) {
    int x = 0;
    int y = 0;
    if (!window_to_canvas(wheel.mouse_x, wheel.mouse_y, x, y)) return;
    const int lines = static_cast<int>(wheel.y) * 3;
    if (console_rect().contains(x, y)) {
        console_.scroll(lines);
        return;
    }
    if (!panel_rect().contains(x, y)) return;
    switch (tab_) {
        case UiTab::Disassembly:
            disasm_.address += static_cast<u32>(lines * 4);
            disasm_.follow_pc = false;
            break;
        case UiTab::Registers:
            registers_.scroll = std::max(0, registers_.scroll - lines);
            break;
        case UiTab::Memory:
            memory_.address += static_cast<u32>(lines * memory_.bytes_per_row);
            break;
        case UiTab::Trace:
            trace_.scroll = std::max(0, trace_.scroll - lines);
            break;
        case UiTab::Devices:
            if (lines > 0) devices_.register_scroll += lines;
            else devices_.register_scroll = std::max(0, devices_.register_scroll + lines);
            break;
        default:
            break;
    }
}

void UiApp::handle_gamepad_event(const SDL_Event& event) {
    switch (event.type) {
        case SDL_EVENT_GAMEPAD_ADDED: {
            SDL_Gamepad* pad = SDL_OpenGamepad(event.gdevice.which);
            if (pad) {
                gamepads_.push_back(pad);
                const char* name = SDL_GetGamepadName(pad);
                console_print(format("gamepad connected: %s", name ? name : "?"), ui_theme::kOk);
            }
            input_.gamepad_connected = !gamepads_.empty();
            break;
        }
        case SDL_EVENT_GAMEPAD_REMOVED: {
            for (size_t i = 0; i < gamepads_.size(); ++i) {
                if (SDL_GetGamepadID(gamepads_[i]) != event.gdevice.which) continue;
                SDL_CloseGamepad(gamepads_[i]);
                gamepads_.erase(gamepads_.begin() + static_cast<long>(i));
                break;
            }
            input_.gamepad_connected = !gamepads_.empty();
            break;
        }
        case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
        case SDL_EVENT_GAMEPAD_BUTTON_UP: {
            const bool down = event.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN;
            input_.source = "gamepad";
            switch (event.gbutton.button) {
                case SDL_GAMEPAD_BUTTON_DPAD_UP: input_.up = down; break;
                case SDL_GAMEPAD_BUTTON_DPAD_DOWN: input_.down = down; break;
                case SDL_GAMEPAD_BUTTON_DPAD_LEFT: input_.left = down; break;
                case SDL_GAMEPAD_BUTTON_DPAD_RIGHT: input_.right = down; break;
                case SDL_GAMEPAD_BUTTON_SOUTH: input_.cross = down; break;
                case SDL_GAMEPAD_BUTTON_EAST: input_.circle = down; break;
                case SDL_GAMEPAD_BUTTON_WEST: input_.square = down; break;
                case SDL_GAMEPAD_BUTTON_NORTH: input_.triangle = down; break;
                case SDL_GAMEPAD_BUTTON_START: input_.start = down; break;
                case SDL_GAMEPAD_BUTTON_BACK: input_.select = down; break;
                case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER: input_.l1 = down; break;
                case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER: input_.r1 = down; break;
                default: break;
            }
            break;
        }
        case SDL_EVENT_GAMEPAD_AXIS_MOTION: {
            const float value = static_cast<float>(event.gaxis.value) / 32767.0f;
            input_.source = "gamepad";
            switch (event.gaxis.axis) {
                case SDL_GAMEPAD_AXIS_LEFTX: input_.left_x = value; break;
                case SDL_GAMEPAD_AXIS_LEFTY: input_.left_y = value; break;
                case SDL_GAMEPAD_AXIS_RIGHTX: input_.right_x = value; break;
                case SDL_GAMEPAD_AXIS_RIGHTY: input_.right_y = value; break;
                default: break;
            }
            break;
        }
        default:
            break;
    }
}

void UiApp::sync_input_from_keyboard(SDL_Scancode scancode, bool down) {
    if (console_.focused() || edit_target_ != EditTarget::None) return;
    vita_input_from_key(scancode, down, input_);
}

bool UiApp::window_to_canvas(float window_x, float window_y, int& canvas_x, int& canvas_y) const {
    if (!window_) return false;
    const SDL_FRect dst = canvas_destination(window_, false);
    if (dst.w <= 0.0f || dst.h <= 0.0f) return false;
    const float sx = (window_x - dst.x) / dst.w * static_cast<float>(kCanvasWidth);
    const float sy = (window_y - dst.y) / dst.h * static_cast<float>(kCanvasHeight);
    canvas_x = static_cast<int>(sx);
    canvas_y = static_cast<int>(sy);
    return true;
}

// ---------------------------------------------------------------------------
// Emulation
// ---------------------------------------------------------------------------

void UiApp::set_running(bool running) {
    if (!running) {
        paused_ = true;
        return;
    }
    if (paused_) console_print("running", ui_theme::kTextDim);
    paused_ = false;
}

void UiApp::toggle_pause() {
    paused_ = !paused_;
    console_print(paused_ ? "paused" : "running", ui_theme::kTextDim);
}

void UiApp::step_into() {
    paused_ = true;
    debugger_.step(1);
}

void UiApp::step_over() {
    // Run until the active core has left the instruction it is sitting on. A
    // taken branch or a call simply runs to the step cap, which is the usual
    // debugger behaviour for "step over".
    paused_ = true;
    Cpu* cpu = debugger_.active_core();
    if (!cpu) return;
    const auto lines = debugger_.disassemble(cpu->get_pc(), 1);
    unsigned length = lines.empty() ? 0u : lines.front().length;
    if (length == 0) length = 4;
    const u32 start = cpu->get_pc();
    const u32 fall_through = start + length;
    for (int i = 0; i < 100000; ++i) {
        debugger_.step(1);
        const u32 pc = cpu->get_pc();
        if (pc == fall_through || cpu->halted || pc < start) break;
        if (breakpoint_hit() != Arch::Unknown) break;
    }
}

Arch UiApp::breakpoint_hit() {
    for (Arch arch : {Arch::Arm, Arch::MeP, Arch::Rl78}) {
        const std::set<u32>& points = debugger_.breakpoints(arch);
        if (points.empty()) continue;
        Cpu* cpu = vita_.core(arch);
        if (!cpu || cpu->halted) continue;
        if (points.count(cpu->get_pc()) != 0) return arch;
    }
    return Arch::Unknown;
}

void UiApp::run_frame() {
    const u64 frame_start = SDL_GetTicksNS();

    if (!paused_) {
        const double target = 1.0 / 60.0;
        const double emulated_start = vita_.emulated_seconds();
        int slices = 0;
        Arch hit = Arch::Unknown;
        while (slices < kMaxSlicesPerFrame) {
            vita_.run_slice();
            ++slices;
            if (vita_.emulated_seconds() - emulated_start >= target) break;
            if ((slices % kSlicesBetweenWallChecks) == 0 &&
                SDL_GetTicksNS() - frame_start >= kFrameWallBudgetNs) {
                break;
            }
            hit = breakpoint_hit();
            if (hit != Arch::Unknown) break;
            if (vita_.stage() == BootStage::Failed) break;
        }
        if (hit != Arch::Unknown) {
            Cpu* cpu = vita_.core(hit);
            paused_ = true;
            console_print(format("[stop] %s breakpoint at 0x%08X", to_string(hit),
                                 cpu ? cpu->get_pc() : 0u),
                          ui_theme::kBreakpoint);
        } else if (vita_.stage() == BootStage::Failed) {
            paused_ = true;
            console_print("[stop] boot chain reported failure", ui_theme::kError);
        }
    }

    const u64 now = SDL_GetTicksNS();
    if (last_wall_ns_ != 0 && now > last_wall_ns_) {
        const double wall = static_cast<double>(now - last_wall_ns_) / 1.0e9;
        const double emulated = vita_.emulated_seconds() - last_emulated_;
        if (wall > 0.0) emulated_per_wall_ = emulated / wall;
    }
    last_wall_ns_ = now;
    last_emulated_ = vita_.emulated_seconds();

    ++fps_frames_;
    if (fps_last_ns_ == 0) fps_last_ns_ = now;
    if (now - fps_last_ns_ >= 500'000'000ull) {
        fps_ = static_cast<double>(fps_frames_) * 1.0e9 / static_cast<double>(now - fps_last_ns_);
        fps_frames_ = 0;
        fps_last_ns_ = now;
    }
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

UiRect UiApp::tab_rect(int index) const {
    // The label is "F<n> <name>", so the width has to account for the prefix.
    auto width_of = [](int i) {
        const std::string label = format("F%d %s", i + 1, tab_name(static_cast<UiTab>(i)));
        return Canvas::text_width(label) + 18;
    };
    int x = 4;
    for (int i = 0; i < index; ++i) x += width_of(i);
    return UiRect{x, 2, width_of(index), kTabBarHeight - 4};
}

UiRect UiApp::panel_rect() const {
    return UiRect{0, kTabBarHeight, kCanvasWidth,
                  kCanvasHeight - kTabBarHeight - kConsoleHeight - kStatusHeight};
}

UiRect UiApp::console_rect() const {
    return UiRect{0, kCanvasHeight - kStatusHeight - kConsoleHeight, kCanvasWidth, kConsoleHeight};
}

UiRect UiApp::status_rect() const {
    return UiRect{0, kCanvasHeight - kStatusHeight, kCanvasWidth, kStatusHeight};
}

void UiApp::render() {
    if (!canvas_.valid()) return;

    canvas_.clear(ui_theme::kBackground);
    draw_tab_bar();
    draw_panel();
    console_.draw(canvas_, console_rect());
    draw_status_bar();

    // Offscreen (screenshot) mode: the canvas *is* the output.
    if (!window_ || !renderer_ || !texture_) return;

    // Upload the ARGB8888 surface to a streaming texture and stretch it to the
    // window (the canvas is a fixed logical resolution, the window is resizable).
    SDL_UpdateTexture(texture_, nullptr, canvas_.pixels(), canvas_.pitch());
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);
    const SDL_FRect dst = canvas_destination(window_, true);
    SDL_RenderTexture(renderer_, texture_, nullptr, &dst);
    SDL_RenderPresent(renderer_);
}

void UiApp::draw_tab_bar() {
    const UiRect bar{0, 0, kCanvasWidth, kTabBarHeight};
    canvas_.fill(bar, ui_theme::kPanel);
    canvas_.hline(0, kTabBarHeight - 1, kCanvasWidth, ui_theme::kRule);

    for (int i = 0; i < static_cast<int>(UiTab::Count); ++i) {
        const UiTab tab = static_cast<UiTab>(i);
        const UiRect rect = tab_rect(i);
        const bool active = tab == tab_;
        const bool hover = rect.contains(mouse_x_, mouse_y_);
        canvas_.fill(rect, active ? ui_theme::kTabActive : (hover ? ui_theme::kSelection : ui_theme::kTabIdle));
        const std::string label = format("F%d %s", i + 1, tab_name(tab));
        canvas_.draw_text(rect.x + 9, rect.y + 6, label, active ? ui_theme::kTextBright : ui_theme::kText);
    }

    // Right hand side: run state and emulated/wall time, so it is always visible.
    std::string right = paused_ ? "PAUSED" : "RUNNING";
    right += format("  emu %.3fs  wall %.1fs (%.1f%%)  fps %.1f", vita_.emulated_seconds(),
                    static_cast<double>(SDL_GetTicksNS()) / 1.0e9, emulated_per_wall_ * 100.0, fps_);
    const int width = Canvas::text_width(right);
    canvas_.draw_text(kCanvasWidth - width - 8, 8, right,
                      paused_ ? ui_theme::kWarn : ui_theme::kOk);
}

void UiApp::draw_panel() {
    const UiRect area = panel_rect();
    canvas_.fill(area, ui_theme::kPanel);
    canvas_.outline(area, ui_theme::kRule);

    switch (tab_) {
        case UiTab::Disassembly: draw_disassembly_panel(area); break;
        case UiTab::Registers: draw_registers_panel(area); break;
        case UiTab::Memory: draw_memory_panel(area); break;
        case UiTab::Trace: draw_trace_panel(area); break;
        case UiTab::Devices: draw_devices_panel(area); break;
        case UiTab::Boot: draw_boot_panel(area); break;
        case UiTab::Display: draw_display_panel(area); break;
        default: break;
    }
    if (edit_target_ != EditTarget::None) {
        draw_edit_field(area, edit_target_ == EditTarget::GotoMemory ? "goto address (memory)"
                                                                    : "goto address (disassembly)");
    }
    draw_help_line(area);
}

void UiApp::draw_help_line(const UiRect& area) {
    static const char* help[static_cast<int>(UiTab::Count)] = {
        "disasm: up/down select  PgUp/PgDn scroll  j goto  Enter breakpoint  f follow PC  n step  g go  "
        "` console",
        "registers: up/down scroll  PgUp/PgDn page  values changed since the last step are green",
        "memory: up/down row  PgUp/PgDn page  j goto address  Hex + ASCII  ` console",
        "trace: up/down scroll  wheel scroll  newest access at the bottom  ` console",
        "devices: up/down select device  PgUp/PgDn registers  values read live from the device",
        "boot: 1 first  2 second  3 kbl  4 kernel  (click the buttons or press 1..4)",
        "panel: emulated display output; RGB565/RGBA8888 from display format",
    };
    const std::string text = help[static_cast<int>(tab_)];
    canvas_.fill(UiRect{area.x + 1, area.bottom() - 15, area.w - 2, 14}, ui_theme::kPanelAlt);
    canvas_.draw_text_clip(area, area.x + 6, area.bottom() - 13, text, ui_theme::kTextDim);
}

void UiApp::draw_status_bar() {
    const UiRect area = status_rect();
    canvas_.fill(area, ui_theme::kPanelAlt);
    canvas_.hline(area.x, area.y, area.w, ui_theme::kRule);

    Cpu* cpu = debugger_.active_core();
    const Arch arch = debugger_.active_arch();
    const u64 total = vita_.total_instructions();
    u64 arm_insns = 0;
    u64 mep_insns = 0;
    u64 rl78_insns = 0;
    if (Cpu* c = vita_.core(Arch::Arm)) arm_insns = c->instructions;
    if (Cpu* c = vita_.core(Arch::MeP)) mep_insns = c->instructions;
    if (Cpu* c = vita_.core(Arch::Rl78)) rl78_insns = c->instructions;

    const std::string line1 = format(
        "core=%-4s %-16s pc=%s  insns=%llu (arm %llu  mep %llu  rl78 %llu)  stage=%s",
        to_string(arch), cpu ? cpu->core_name() : "(absent)", cpu ? hex(cpu->get_pc(), 8).c_str() : "--------",
        static_cast<unsigned long long>(total), static_cast<unsigned long long>(arm_insns),
        static_cast<unsigned long long>(mep_insns), static_cast<unsigned long long>(rl78_insns),
        to_string(vita_.stage()));
    canvas_.draw_text_clip(area, 6, area.y + 2, line1, ui_theme::kText);

    const EmmcCard& card = vita_.emmc();
    const std::string emmc = card.attached()
                                 ? format("%s %s r=%llu w=%llu xfers=%llu lba=%llu", path_filename(card.path()).c_str(),
                                          human_size(card.capacity_bytes()).c_str(),
                                          static_cast<unsigned long long>(card.reads()),
                                          static_cast<unsigned long long>(card.writes()),
                                          static_cast<unsigned long long>(vita_.kermit().emmc_transfers()),
                                          static_cast<unsigned long long>(vita_.kermit().last_emmc_lba()))
                                 : std::string("not attached");
    std::string line2 = format("emmc=%s  pad=%s", emmc.c_str(), input_.status().c_str());
    if (!input_.gamepad_connected) line2 += " (no gamepad)";
    if (audio_.available()) {
        line2 += format("  audio=48kHz stereo s16 cbs=%llu%s", static_cast<unsigned long long>(audio_.callbacks()),
                        audio_.muted() ? " muted" : (audio_.test_tone() ? " tone" : ""));
    } else {
        line2 += "  audio=off";
    }
    if (vita_.kernel_started()) line2 += "  kernel=started";
    if (vita_.kernel_running()) line2 += " running";
    const std::string right = format("emu %.3fs / wall %.1fs  fps %.1f", vita_.emulated_seconds(),
                                     static_cast<double>(SDL_GetTicksNS()) / 1.0e9, fps_);
    // The pad/audio line shares its row with the time indicator: clip it so the
    // two can never overlap.
    const int right_width = Canvas::text_width(right) + 16;
    const UiRect line2_clip{area.x, area.y + 13, area.w - right_width, area.h - 13};
    canvas_.draw_text_fit(line2_clip, 6, area.y + 14, line2, ui_theme::kTextDim);
    canvas_.draw_text_clip(area, area.right() - Canvas::text_width(right) - 6, area.y + 14, right,
                           ui_theme::kText);
}

// ---------------------------------------------------------------------------
// Display (F7)
// ---------------------------------------------------------------------------

void UiApp::draw_display_panel(const UiRect& area) {
    int width = 0;
    int height = 0;
    int stride = 0;
    int bpp = 0;
    const u8* pixels = vita_.kermit().framebuffer(width, height, stride, &bpp);

    display_.frames = vita_.kermit().frame_counter();
    display_.width = width;
    display_.height = height;
    display_.stride = stride;

    const UiRect view{area.x + 4, area.y + 16, area.w - 8, area.h - 34};

    if (pixels && width > 0 && height > 0 && stride > 0) {
        display_.bytes_per_pixel = bpp;
        display_.present = true;
        blit_display(view, pixels, width, height, stride, bpp);
        const char* kind = bpp == 2 ? "RGB565" : (bpp == 4 ? "RGBA8888" : "unknown");
        canvas_.draw_text_clip(area, area.x + 6, area.y + 3,
                               format("framebuffer %dx%d stride=%d (%d bpp, %s)  frame=%llu", width, height,
                                      stride, bpp, kind, static_cast<unsigned long long>(display_.frames)),
                               ui_theme::kOk);
        return;
    }

    display_.present = false;
    display_.bytes_per_pixel = 0;
    canvas_.fill(view, 0xFF0A0C10u);
    canvas_.outline(view, ui_theme::kRule);

    static const char* lines[] = {
        "no framebuffer yet",
        "",
        "the guest has not submitted a supported display buffer.",
        "open the Devices tab to inspect display state.",
    };
    const int count = static_cast<int>(sizeof(lines) / sizeof(lines[0]));
    int y = view.y + std::max(4, view.h / 2 - (count * kLineHeight) / 2);
    for (int i = 0; i < count; ++i) {
        const int width_px = Canvas::text_width(lines[i]);
        canvas_.draw_text_clip(view, view.x + (view.w - width_px) / 2, y, lines[i],
                               i == 0 ? ui_theme::kWarn : ui_theme::kTextDim);
        y += kLineHeight;
    }

    const std::string status = format("stage=%s  detail=%s", to_string(vita_.stage()),
                                      vita_.boot_status().detail.c_str());
    canvas_.draw_text_clip(area, area.x + 6, area.y + 3, status, ui_theme::kTextDim);
}

void UiApp::blit_display(const UiRect& area, const u8* pixels, int width, int height, int stride, int bpp) {
    if (!pixels || width <= 0 || height <= 0 || (bpp != 2 && bpp != 4)) return;

    const double scale = std::min(area.w / static_cast<double>(width), area.h / static_cast<double>(height));
    const int dst_w = std::max(1, static_cast<int>(width * scale));
    const int dst_h = std::max(1, static_cast<int>(height * scale));
    const int origin_x = area.x + (area.w - dst_w) / 2;
    const int origin_y = area.y + (area.h - dst_h) / 2;

    for (int y = 0; y < dst_h; ++y) {
        const int sy = static_cast<int>(static_cast<long long>(y) * height / dst_h);
        const u8* row = pixels + static_cast<size_t>(sy) * static_cast<size_t>(stride);
        for (int x = 0; x < dst_w; ++x) {
            const int sx = static_cast<int>(static_cast<long long>(x) * width / dst_w);
            u32 argb = 0xFF000000u;
            if (bpp == 2) {
                const u16 value = static_cast<u16>(row[sx * 2] | (row[sx * 2 + 1] << 8));
                const u32 r = (value >> 11) & 0x1Fu;
                const u32 g = (value >> 5) & 0x3Fu;
                const u32 b = value & 0x1Fu;
                argb |= ((r << 3) | (r >> 2)) << 16;
                argb |= ((g << 2) | (g >> 4)) << 8;
                argb |= (b << 3) | (b >> 2);
            } else {
                // The display controller stores RGBA8888 bytes; the canvas is
                // ARGB8888, so the red and blue channels swap.
                argb |= static_cast<u32>(row[sx * 4 + 0]) << 16;
                argb |= static_cast<u32>(row[sx * 4 + 1]) << 8;
                argb |= static_cast<u32>(row[sx * 4 + 2]);
            }
            canvas_.pixel(origin_x + x, origin_y + y, argb);
        }
    }
    canvas_.outline(UiRect{origin_x - 1, origin_y - 1, dst_w + 2, dst_h + 2}, ui_theme::kRule);
}

}  // namespace zlb

int main(int argc, char** argv) { return zlb::ui_main(argc, argv); }
