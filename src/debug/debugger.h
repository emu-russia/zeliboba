// zeliboba - the debugger.
//
// One class drives both frontends: the headless console (src/debug/cli.cpp) and
// the SDL3 window (src/ui). All inspection and control is expressed as text
// commands, so anything you can do interactively can also be scripted with -ex.
#pragma once

#include <deque>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "cpu/cpu.h"
#include "machine/vita.h"

namespace zlb {

enum class WatchKind : u8 { Read = 1, Write = 2, Execute = 4, Any = 7 };

struct Watchpoint {
    int id = 0;
    u32 address = 0;
    u32 mask = 0xFFFFFFFFu;  ///< compare (address & mask)
    unsigned kind = 0;       ///< bitmask of WatchKind
    bool enabled = true;
    std::string note;
};

struct BreakpointInfo {
    Arch arch = Arch::Unknown;
    u32 address = 0;
    bool enabled = true;
};

/// What stopped the last run.
struct StopInfo {
    bool stopped = false;
    std::string reason;
    Arch arch = Arch::Unknown;
    u32 address = 0;
    u64 steps = 0;
};

class Debugger {
public:
    using Output = std::function<void(const std::string&)>;

    explicit Debugger(Vita& vita);

    void set_output(Output output) { output_ = std::move(output); }
    void print(const std::string& text);

    // ------------------------------------------------------------------
    // Command interface (shared by the CLI and the GUI)
    // ------------------------------------------------------------------

    /// Execute one command line. Returns true when the debugger should keep
    /// running (false means "quit").
    bool execute(const std::string& line);
    bool quit_requested() const { return quit_; }

    /// Command completion candidates for the given prefix (GUI/CLI completion).
    std::vector<std::string> complete(const std::string& prefix) const;

    // ------------------------------------------------------------------
    // Execution control
    // ------------------------------------------------------------------

    /// Execute `count` instructions on the active core.
    void step(int count = 1);
    /// Run until `count` instructions have been executed on the active core, or
    /// a breakpoint/watchpoint/stop condition hits.
    void run(int64_t count);
    /// Run until the active core's pc equals `address`.
    void run_until(u32 address, int64_t limit = 50'000'000);
    /// Run whole machine slices (all cores) until stopped.
    void run_machine(int64_t slices);
    /// True when any core sits on a breakpoint after a machine slice.
    bool machine_breakpoint_check();
    void stop() { stop_requested_ = true; }

    Cpu* active_core() const;
    Arch active_arch() const { return active_arch_; }
    void set_active_arch(Arch arch);
    /// Which core of the Kermit cluster `core arm` refers to (0..3).
    int arm_core_index() const { return arm_core_index_; }
    void set_arm_core_index(int index);

    const StopInfo& last_stop() const { return last_stop_; }

    // ------------------------------------------------------------------
    // Breakpoints and watchpoints
    // ------------------------------------------------------------------

    void add_breakpoint(Arch arch, u32 address);
    void remove_breakpoint(Arch arch, u32 address);
    void clear_breakpoints();
    const std::set<u32>& breakpoints(Arch arch) const;

    int add_watchpoint(u32 address, unsigned kind, u32 mask = 0xFFFFFFFFu, const std::string& note = {});
    bool remove_watchpoint(int id);
    void clear_watchpoints();
    const std::deque<Watchpoint>& watchpoints() const { return watchpoints_; }

    // ------------------------------------------------------------------
    // Views used by the GUI
    // ------------------------------------------------------------------

    struct DisassemblyLine {
        u32 address = 0;
        std::string text;
        bool is_pc = false;
        bool has_breakpoint = false;
        std::vector<u8> bytes;
    };

    std::vector<DisassemblyLine> disassemble(u32 address, int count, Arch arch = Arch::Unknown);
    std::vector<std::string> memory_dump(u32 address, int rows, int bytes_per_row = 16);
    std::vector<RegValue> registers_of(Cpu& cpu);

    /// Registers of the active core.
    std::vector<RegValue> registers() { return registers_of(*active_core()); }

    // ------------------------------------------------------------------
    // Logging / tracing
    // ------------------------------------------------------------------

    void set_log_level(LogLevel level);
    void set_category_level(const std::string& category, LogLevel level);
    const std::vector<LogRecord>& recent_log() const { return log_records_; }
    void clear_log() { log_records_.clear(); }

    /// Advance every core by exactly one instruction (used by step/run).
    void step_machine_once();

    /// Execution history: `history on` starts recording every executed PC, so a
    /// long run that ends in a bad branch can still be reconstructed.
    void set_history_enabled(bool enabled) { history_enabled_ = enabled; }
    bool history_enabled() const { return history_enabled_; }
    void clear_history();
    /// Last `count` executed PCs of one core (Arch::Unknown = the active core).
    std::vector<u32> history(Arch arch, size_t count) const;

private:
    void hook_log();
    void check_watchpoints(u64 trace_from);

    std::string cmd_help(const std::vector<std::string>& args);
    std::string cmd_info(const std::vector<std::string>& args);
    std::string cmd_regs(const std::vector<std::string>& args);
    std::string cmd_dis(const std::vector<std::string>& args);
    std::string cmd_mem(const std::vector<std::string>& args);
    std::string cmd_trace(const std::vector<std::string>& args);
    std::string cmd_devices(const std::vector<std::string>& args);
    std::string cmd_dev(const std::vector<std::string>& args, bool write);
    std::string cmd_emmc(const std::vector<std::string>& args);
    std::string cmd_boot(const std::vector<std::string>& args);
    std::string cmd_keyring(const std::vector<std::string>& args);
    std::string cmd_image(const std::vector<std::string>& args);
    std::string cmd_log(const std::vector<std::string>& args);

    Vita& vita_;
    Output output_;
    Arch active_arch_ = Arch::MeP;
    int arm_core_index_ = 0;
    std::map<Arch, std::set<u32>> breakpoints_;
    std::deque<Watchpoint> watchpoints_;
    int next_watchpoint_id_ = 1;
    bool stop_requested_ = false;
    bool quit_ = false;
    StopInfo last_stop_;
    std::vector<LogRecord> log_records_;
    u64 log_hook_installed_ = 0;

    bool history_enabled_ = false;
    /// One ring per core, so a busy core cannot evict a quiet one's history.
    std::map<Arch, std::deque<u32>> history_;
    size_t history_limit_ = 1u << 20;
};

}  // namespace zlb
