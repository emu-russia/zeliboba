// zeliboba - Toshiba MeP-c5 ("CMeP" / F00D) interpreter.
//
// The MeP-c5 is the PS Vita's security core: it boots the first loader out of
// the 128 KiB CMeP RAM window at 0x40000, talks to the ARM through the mailbox
// at 0xE0000000 and to the crypto blocks at 0xE0030000..0xE0070000.
//
// Semantics follow the CGEN architecture description used by binutils/GDB 2.37
// (cpu/mep-core.cpu, cpu/mep-c5.cpu).  All memory access goes through the Bus;
// the core never touches host memory directly, so the access trace stays
// meaningful for the debugger.
//
// GROUND TRUTH WARNING: dumps/bootrom_analysis/*.annotated.asm are generated
// from a *different revision* of the CMeP ROM than dumps/vita_prototype_
// bootrom.bin, so they disagree with the binary from 0x5C01A onwards.  Byte
// offsets known to differ: 0x5C01A (listing shows a 4 byte `sw $1,4($2)`, the
// binary has the 2 byte `sw $1,($2)`), 0x5C020 (listing `beqi $9,0x5,0x5c02a`,
// binary `add $2,8`) and 0x5C578 (listing `add $10,16`, binary `movh
// $10,0xe000`).  The binary plus the `_scratch/mep_first_loader.log` listing
// are the authoritative pair; use the .annotated.asm files only for structure
// and comments.
#pragma once

#include <array>
#include <string>

#include "cpu/cpu.h"
#include "cpu/mep/mep_isa.h"

namespace zlb {

class MePCore : public Cpu {
public:
    /// Software model of the control bus (`stcb`/`ldcb`) space. The INTC
    /// registers at 0..7 are 32-bit words. Locations 0x400..0x405 drive a hardware
    /// one shot timer that both loaders poll (first loader 0x5E660, second
    /// loader 0x45500).  The real device lives in the hardware layer, so this
    /// model is deliberately small and deterministic - enough that the poll
    /// loops terminate.  Register map, from the two call sites:
    ///   0x400/0x401  one shot reload, 16 bit big endian (0x400 = high byte)
    ///   0x402        control, bit 0 = enable (start)
    ///   0x404        status, bit 0 = "the count reached zero" latch
    struct ControlBus {
        std::array<u32, 0x420> regs{};
        u32 irq_levels = 0;
        u32 irq_edges = 0;

        /// Reload value assembled from 0x400 (high) and 0x401 (low).
        u32 count = 0;
        /// Instructions left until the timer raises its completion latch.
        u32 remaining = 0;
        bool running = false;
        /// Completion latch reported in status bit 0; cleared by writing 0x404.
        bool done = false;
        /// Set by the hardware layer (or a test) to force completion.
        bool force_expired = false;

        /// Plain helper class: no `override` (the base declares no virtuals).
        /// Every mutable member above is written, `regs` as a fixed array.
        void save_state(StateWriter& writer) const;
        void load_state(StateReader& reader);

        void reset();
        /// Read one control bus word; `ldcb $rn,0x404` is the timer poll.
        u32 read(unsigned address) const;
        /// Write one control bus word; writing the count reloads the timer.
        void write(unsigned address, u32 value);
        /// Advance the one shot timer by one instruction.
        void advance();
        /// True while the timer is still counting (status bit 0 reads 0).
        bool busy() const;
        void set_irq_level(unsigned source, bool asserted);
        /// Highest eligible interrupt, or -1; equal levels favor larger channels.
        int pending_irq() const;
        void acknowledge_irq(unsigned source);
    };

    explicit MePCore(Bus& bus);

    /// Machine-level "stop before this PC" hook: the boot chain uses it to
    /// intercept the first loader's service entry point (0x5FF00, which the
    /// second loader calls with `jmp` after clearing its registers) without
    /// having to model the ROM routine that lives there.  Returning true stops
    /// the core *before* the instruction executes and records the halt reason,
    /// so the machine can substitute the service at a slice boundary.
    std::function<bool(u32)> pc_hook;
    /// External INTC input. Requests remain pending until the device deasserts
    /// its level or software clears an edge-triggered ISR bit.
    void set_irq_level(unsigned source, bool asserted);
    /// Board-selected boot vector bank used while CFG.EVM=0. Ordinary MeP
    /// systems use zero; the CMeP boot stages remap this bank through hardware
    /// whose register interface is not yet modeled.
    void set_boot_vector_base(u32 base) { boot_vector_base_ = base; }

    // ------------------------------------------------------------------ Cpu
    Arch arch() const override { return Arch::MeP; }
    const char* core_name() const override { return "CMeP"; }

    void reset() override;
    void reset(u32 entry) override;
    void prepare_reset_context(u64 a0, u64 a1, u64 a2, u64 a3) override;

    StepResult step() override;
    std::string disassemble(u32 address, unsigned& length) override;

    void registers(std::vector<RegValue>& out) const override;
    bool set_register(const std::string& name, u64 value) override;
    bool get_register(const std::string& name, u64& value) const override;

    std::string status_line() const override;
    void describe_state(std::vector<std::string>& lines) const override;
    void tick(u64 cycles_) override;

    // ------------------------------------------------------------ save states
    /// `Cpu::save_state` first (counters, pc, halt state), then the MeP
    /// registers and the control bus model.  `pc_hook` is a host callback, not
    /// machine state, and is deliberately skipped.
    void save_state(StateWriter& writer) const override;
    void load_state(StateReader& reader) override;

    // ------------------------------------------------------------- registers
    std::array<u32, 16> r{};   ///< $0..$12 GPRs, $13/$tp, $14/$gp, $15/$sp
    u32 hi = 0;
    u32 lo = 0;
    u32 sar = 0;
    u32 lp = 0;
    u32 epc = 0;
    u32 npc = 0;
    u32 tmp = 0;
    u32 psw = 0;
    u32 exc = 0;
    u32 cfg = 0;
    u32 vid = 0;
    u32 id = 0;
    u32 dbg = 0;
    u32 depc = 0;
    u32 opt = 0;
    u32 rcfg = 0;
    u32 ccfg = 0;
    /// Coprocessor condition register used by the `bcpeq`..`bcpaf` family.
    u32 cr0 = 0;
    /// Condition flag file (MeP CFR).  Bit i is Cond[i].
    std::array<bool, 8> cond{};

    // Loop unit (hardware `repeat` block).
    u32 rpb = 0;
    u32 rpe = 0;
    u32 rpc = 0;

    // Modulo/end address registers of the addressing unit.
    u32 mb0 = 0;
    u32 me0 = 0;
    u32 mb1 = 0;
    u32 me1 = 0;

    ControlBus cbus;

    /// PSW bit 12 is the "operating mode" bit: set while a VLIW packet from the
    /// IVC2 coprocessor is running (Venezia only; always clear on CMeP).
    bool vliw_mode = false;

    /// Reset vector used by the no argument reset().  The architectural CMeP
    /// reset vector is 0x00040000, but the machine layer always calls
    /// reset(entry) with the address the boot stage was loaded at.
    u32 reset_vector = 0x00040000;

    /// Packed condition flags, as shown by `registers()` and status_line().
    u32 cond_packed() const;

private:
    // ------------------------------------------------------------- execution
    void execute(const mep::Insn& insn, u32 word, u32 address);
    void repeat_step_end(u32 address, bool branch_taken);
    /// set_pc plus the "a branch was taken" flag the hardware loop unit needs.
    void branch_to(u32 target);
    void refresh_irq_line();
    bool take_pending_irq();
    void cop_word(const mep::Insn& insn, u32 word, u32 address);
    void cop_word64(const mep::Insn& insn, u32 word, u32 address);
    /// Mark the current instruction as unimplemented.  The PC is left on the
    /// offending instruction so the debugger can stop on it.
    std::string mark_undefined(const std::string& what, u32 address);

    // --------------------------------------------------------------- CSRs
    u32 get_csr(int index) const;
    void set_csr(int index, u32 value);

    // ------------------------------------------------------------ memory
    u8 load8(u32 address);
    u16 load16(u32 address);
    u32 load32(u32 address);
    void store8(u32 address, u8 value);
    void store16(u32 address, u16 value);
    void store32(u32 address, u32 value);
    /// Refresh bus->context so the trace attributes the access correctly.
    void set_context(u32 address);
    /// `ZLB_MEP_PC=<addr>[,<addr>...]` dumps the GPRs whenever the core reaches
    /// one of those addresses (bring-up aid, see the comment in the .cpp).
    void trace_watched_pc();
    /// Coprocessor/control bus transfer.  No external handler exists yet, so
    /// the built in model answers; the hardware layer replaces it later.
    u32 cop_access(const char* kind, unsigned size, u32 address, u32 value, bool load);

    bool rep_active_ = false;
    bool rep_pending_back_ = false;
    bool rep_endless_ = false;
    /// Set by branch_to() for the instruction currently executing.  The hardware
    /// loop unit (`erepeat`) must escape when the trailing slot takes a branch,
    /// and comparing PCs is not enough: a branch to the next instruction is still
    /// a taken branch.
    bool branch_taken_ = false;
    u32 boot_vector_base_ = 0;
};

}  // namespace zlb
