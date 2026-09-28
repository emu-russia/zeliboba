// zeliboba - console frontend and command line handling.
#include "debug/cli.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "common/log.h"
#include "common/util.h"
#include "debug/debugger.h"
#include "machine/vita.h"

namespace zlb {

namespace {

void print_banner(const Vita& vita) {
    std::printf("zeliboba - PlayStation Vita emulator (CMeP / Cortex-A9 / Ernie)\n");
    std::printf("firmware target: 1.04    workspace: %s\n", workspace_root().c_str());
    std::printf("stage: %s    type 'help' for commands\n\n", to_string(vita.stage()));
}

void attach_console_log(bool verbose) {
    Log::instance().set_level(verbose ? LogLevel::Debug : LogLevel::Info);
    // NOTE: Log::log() already drops records below the configured level (global
    // and per-category), so the sink must print everything it is handed - a
    // second ">= Info" filter here silently swallowed every debug line.
    Log::instance().add_sink([](const LogRecord& record) {
        std::printf("[%-5s] %-10s %s\n", to_string(record.level), record.category.c_str(), record.text.c_str());
        std::fflush(stdout);
    });
}

int run_script(Debugger& debugger, const std::vector<std::string>& commands, bool echo) {
    for (const std::string& command : commands) {
        if (command.empty() || command[0] == '#' || command[0] == ';') continue;
        if (echo) std::printf("zlb> %s\n", command.c_str());
        const bool keep_going = debugger.execute(command);
        // `quit` is a normal way to end a script, not a failure.
        if (!keep_going) return 0;
    }
    return 0;
}

void print_usage() {
    std::printf(
        "usage: zeliboba [options]\n"
        "\n"
        "  --emmc <file>          eMMC image (built automatically when missing)\n"
        "  --first-loader <file>  CMeP first loader binary\n"
        "  --syscon <file>        Ernie (RL78) firmware dump\n"
        "  --fs <dir>             extracted firmware tree (Vita_104_Firmware/Out)\n"
        "  --no-syscon            do not execute the syscon firmware (functional model only)\n"
        "  --no-rebuild           never build the eMMC image\n"
        "  --no-provision         leave the emulated CMeP key table empty (the first\n"
        "                         loader's RSA check then fails, as on an unprovisioned\n"
        "                         console - see docs/KBL.md)\n"
        "  --stage <name>         start at first|second|secure|kbl|nskbl|kernel\n"
        "  --run <n>              run n machine steps after setup\n"
        "  -ex <command>          execute a debugger command (repeatable)\n"
        "  --script <file>        execute debugger commands from a file\n"
        "  --info                 print the machine layout and exit\n"
        "  --boot                 print the boot plan and exit\n"
        "  --log <level>          trace|debug|info|warn|error|off\n"
        "  --verbose              same as --log debug\n"
        "  -q, --quiet            no banner\n"
        "  -h, --help             this text\n");
}

}  // namespace

int cli_main(int argc, char** argv) {
    VitaConfig config;
    std::vector<std::string> script;
    std::string stage_name;
    std::string script_file;
    int run_steps = 0;
    bool show_info = false;
    bool show_boot = false;
    bool quiet = false;
    bool verbose = false;
    LogLevel level = LogLevel::Info;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "zeliboba: %s requires an argument\n", what);
                std::exit(2);
            }
            return argv[++i];
        };

        if (arg == "-h" || arg == "--help") { print_usage(); return 0; }
        else if (arg == "--emmc") config.emmc_image = next("--emmc");
        else if (arg == "--first-loader") config.first_loader = next("--first-loader");
        else if (arg == "--syscon") config.syscon_firmware = next("--syscon");
        else if (arg == "--fs") config.fs_root = next("--fs");
        else if (arg == "--no-syscon") config.run_syscon_firmware = false;
        else if (arg == "--no-rebuild") config.rebuild_emmc = false;
        else if (arg == "--no-provision") config.provision_keys = false;
        else if (arg == "--stage") stage_name = next("--stage");
        else if (arg == "--run") run_steps = std::atoi(next("--run").c_str());
        else if (arg == "-ex" || arg == "--exec") script.push_back(next("-ex"));
        else if (arg == "--script") script_file = next("--script");
        else if (arg == "--info") show_info = true;
        else if (arg == "--boot") show_boot = true;
        else if (arg == "--log") {
            std::string value = next("--log");
            if (!parse_log_level(value, level)) {
                std::fprintf(stderr, "zeliboba: unknown log level '%s'\n", value.c_str());
                return 2;
            }
        } else if (arg == "--verbose") verbose = true;
        else if (arg == "-q" || arg == "--quiet") quiet = true;
        else {
            std::fprintf(stderr, "zeliboba: unknown option '%s'\n", arg.c_str());
            print_usage();
            return 2;
        }
    }

    attach_console_log(verbose);
    if (verbose) level = LogLevel::Debug;
    Log::instance().set_level(level);

    Vita vita;
    vita.build(config);
    vita.reset(true);

    if (!script_file.empty()) {
        if (auto data = read_file(resolve_workspace_path(script_file))) {
            std::string text(data->begin(), data->end());
            for (const std::string& line : split(text, '\n')) {
                std::string clean = trim(line);
                if (!clean.empty()) script.push_back(clean);
            }
        } else {
            std::fprintf(stderr, "zeliboba: cannot read script %s\n", script_file.c_str());
            return 2;
        }
    }

    Debugger debugger(vita);
    debugger.set_output([](const std::string& text) {
        std::printf("%s\n", text.c_str());
        std::fflush(stdout);
    });

    if (!stage_name.empty()) {
        if (stage_name == "first" || stage_name == "firstloader") vita.enter_stage(BootStage::CmepFirstLoader);
        else if (stage_name == "second") vita.enter_stage(BootStage::CmepSecondLoader);
        else if (stage_name == "kbl") vita.enter_stage(BootStage::ArmKernelBootLoader);
        else if (stage_name == "nskbl") vita.enter_stage(BootStage::NskblEntry);
        else if (stage_name == "kernel") vita.enter_stage(BootStage::KernelEntry);
        else {
            std::fprintf(stderr, "zeliboba: unknown stage '%s'\n", stage_name.c_str());
            return 2;
        }
    }

    if (!quiet) print_banner(vita);

    if (show_info) {
        debugger.execute("info");
        for (const auto& line : split(vita.cmep_bus().describe_map(), '\n')) {
            if (!line.empty()) std::printf("%s\n", line.c_str());
        }
        for (const auto& line : split(vita.arm_bus().describe_map(), '\n')) {
            if (!line.empty()) std::printf("%s\n", line.c_str());
        }
        if (!script.empty()) return run_script(debugger, script, true);
        return 0;
    }

    if (show_boot) {
        debugger.execute("boot");
        if (!script.empty()) return run_script(debugger, script, true);
        return 0;
    }

    if (run_steps > 0) {
        debugger.execute(format("run %d", run_steps));
        std::printf("%s\n", vita.boot_report().c_str());
    }

    if (!script.empty()) return run_script(debugger, script, true);

    // Interactive console.
    std::string line;
    while (!debugger.quit_requested()) {
        std::printf("zlb> ");
        std::fflush(stdout);
        if (!std::getline(std::cin, line)) break;
        debugger.execute(line);
    }
    return 0;
}

}  // namespace zlb
