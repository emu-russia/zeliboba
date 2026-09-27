// zeliboba - CPU core abstraction.
//
// Three interpreters implement this interface: the ARM Cortex-A9 (main SoC),
// the Toshiba MeP-c5 (CMeP "F00D" security core) and the Renesas RL78 (Ernie
// syscon). The debugger and the machine layer only ever talk to this interface.
#pragma once

#include <functional>
#include <set>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/log.h"
#include "common/types.h"

namespace zlb {

enum class Arch { Unknown = 0, Arm, MeP, Rl78 };

const char* to_string(Arch arch);
bool parse_arch(const std::string& text, Arch& out);

struct RegValue {
    std::string group;
    std::string name;
    u64 value = 0;
    std::string note;

    RegValue() = default;
    RegValue(std::string g, std::string n, u64 v, std::string note_ = {})
        : group(std::move(g)), name(std::move(n)), value(v), note(std::move(note_)) {}
};

struct StepResult {
    u32 address = 0;
    unsigned length = 0;
    std::string text;
    bool faulted = false;
    std::string fault;
    bool was_branch = false;
};

/// Exception/interrupt line numbering shared by all cores.
enum class IrqLine : int {
    Irq = 0,
    FiQ = 1,
    // ARM GIC shared peripheral interrupts that the machine layer raises.
    ArmTimer = 2,
    Syscon = 3,
    Emmc = 4,
    Dma = 5,
};

class Cpu {
public:
    virtual ~Cpu() = default;

    virtual Arch arch() const = 0;
    virtual const char* core_name() const = 0;

    std::string name = "cpu";
    Bus* bus = nullptr;

    u64 instructions = 0;
    u64 cycles = 0;

    bool halted = false;
    std::string halt_reason;
    u32 pc = 0;
    bool undefined_instruction = false;

    std::set<u32> breakpoints;

    virtual void reset() = 0;
    virtual void reset(u32 entry) {
        reset();
        set_pc(entry);
    }

    /// Architecture specific reset environment, applied by the machine right
    /// after `reset(entry)`:
    ///   ARM  - a0..a3 -> r0..r3 (the kernel boot loader passes an ATAGS-like
    ///          structure in r0/r1/r2);
    ///   MeP  - a0 -> $0, which the reset hardware leaves holding the stack
    ///          pointer the first loader copies into $sp;
    ///   RL78 - unused.
    virtual void prepare_reset_context(u64 a0, u64 a1, u64 a2, u64 a3) {
        (void)a0;
        (void)a1;
        (void)a2;
        (void)a3;
    }

    virtual StepResult step() = 0;

    virtual std::string disassemble(u32 address, unsigned& length) = 0;
    virtual void registers(std::vector<RegValue>& out) const = 0;
    virtual bool set_register(const std::string& name, u64 value) { (void)name; (void)value; return false; }
    virtual bool get_register(const std::string& name, u64& value) const { (void)name; (void)value; return false; }

    virtual u32 get_pc() const { return pc; }
    virtual void set_pc(u32 value) { pc = value; }

    /// One line of extra state for the status bar ("Thumb SVC nzCv mmu=on").
    virtual std::string status_line() const { return {}; }

    /// Extra state in the debugger's state view.
    virtual void describe_state(std::vector<std::string>& lines) const { (void)lines; }

    /// Interrupt input. Level triggered: `asserted == false` deasserts.
    virtual void set_irq(int line, bool asserted) { (void)line; (void)asserted; }

    /// Returns true when an interrupt is taken at the next step boundary.
    virtual bool interrupt_pending() const { return false; }

    /// Called by the machine scheduler; cores advance their timers here.
    virtual void tick(u64 cycles_) { cycles += cycles_; }

    bool hit_breakpoint() const { return !breakpoints.empty() && breakpoints.count(pc) != 0; }

    void halt(const std::string& reason) {
        halted = true;
        halt_reason = reason;
        ZLB_LOG_WARN("cpu", "%s halted: %s", name.c_str(), reason.c_str());
    }

    /// Run up to `max_steps` instructions. Stops on halt, fault, breakpoint or
    /// when `abort` returns true. Returns instructions executed.
    int run(int64_t max_steps, const std::function<bool()>& abort = nullptr);

protected:
    explicit Cpu(Bus& bus_) : bus(&bus_) {}
};

}  // namespace zlb
