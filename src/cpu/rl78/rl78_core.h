// zeliboba - Renesas RL78 interpreter (the Vita "ErnIE" system controller).
//
// MEMORY MODEL (RL78 Family User's Manual: Software, R01US0056; encodings from
// the binutils RL78 decoder, see rl78_isa.h):
//
//   0x00000 - 0xEFFFF  internal program memory (flash)
//   0x00000 - 0x0007F  vector table (2 bytes per vector, RESET first)
//   0x00080 - 0x000BF  CALLT table
//   0xF0000 - 0xFFFFF  data page: a16 direct, register indirect and based
//                      addressing land here (0xF0000 | a16, 0xF0000 + HL, ...)
//   0xFFF00 - 0xFFFFF  SFR window (SP 0xFFFF8, PSW 0xFFFFA, CS 0xFFFFC,
//                      ES 0xFFFFD, PMC 0xFFFFE)
//   0xFFEE0 - 0xFFEFF  register banks RB0..RB3
//
// The 0x11 prefix ("ES:") selects a bank for the next instruction only: the
// effective data address becomes (ES << 16) | a16.
//
// The control registers are memory mapped on real silicon, but SP/PSW/CS/ES/PMC
// live in the core here and every data access to those five addresses is
// served from (and written back to) the register file.  Modelling them as plain
// RAM is what broke the reset handler in the first place: `CB F8 20 FE` is
// "movw sp, #0xFE20", not a store to 0xFFFF8.
//
// REGISTER BANKS
// --------------
// SEL RB0..RB3 is modelled through PSW.RBS0/RBS1: `bank()` reports the selected
// bank and `bank_base()` its address (0xFFEE0 + 8 * bank).  There is one
// physical register file, exactly like the reference core, so switching banks
// changes the bank select bits but not which A/X/B/C/D/E/H/L the instructions
// read.  On silicon the general-purpose registers are additionally visible as
// data memory - 0xFFEF8..0xFFEFF for the active bank on the RL78-S1 core,
// 0xFFEE0..0xFFEFF for all four on S2/S3 - and a bank switching interrupt
// handler relies on that window.  The boot path measured here never touches it,
// so the window is left as plain RAM; a product that needs compiler generated
// bank save/restore has to add the alias and four register files.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "cpu/cpu.h"
#include "cpu/rl78/rl78_decode.h"
#include "cpu/rl78/rl78_isa.h"

namespace zlb {

class Rl78Core : public Cpu {
public:
    explicit Rl78Core(Bus& bus);

    Arch arch() const override { return Arch::Rl78; }
    const char* core_name() const override { return "RL78"; }

    void reset() override;
    void reset(u32 entry) override;
    StepResult step() override;
    std::string disassemble(u32 address, unsigned& length) override;
    void registers(std::vector<RegValue>& out) const override;
    bool set_register(const std::string& name, u64 value) override;
    bool get_register(const std::string& name, u64& value) const override;
    std::string status_line() const override;
    void describe_state(std::vector<std::string>& lines) const override;
    void set_irq(int line, bool asserted) override;
    bool interrupt_pending() const override;

    // ------------------------------------------------------------------
    // Register file
    // ------------------------------------------------------------------
    u8 a = 0;
    u8 x = 0;
    u8 b = 0;
    u8 c = 0;
    u8 d = 0;
    u8 e = 0;
    u8 h = 0;
    u8 l = 0;
    u16 sp = 0;
    u16 psw = 0x06;  // reset value: ISP0 = ISP1 = 1, all flags clear
    u8 es = 0x0F;    // reset value per the RL78/G13 hardware manual
    u8 cs = 0;
    u8 pmc = 0;

    /// Word fetched from address 0 at reset.
    u32 reset_vector = 0;

    /// Data accesses served by the core (the SFR mirror and the bus).
    u64 data_accesses = 0;

    /// Instructions whose encoding was not in the table.
    u64 unknown_instructions = 0;

    /// Pending maskable interrupt vector, or -1.
    int pending_vector = -1;

    // ------------------------------------------------------------------
    // Flag / bank views
    // ------------------------------------------------------------------
    bool flag_z() const { return (psw & kRl78FlagZ) != 0; }
    bool flag_cy() const { return (psw & kRl78FlagCy) != 0; }
    bool flag_ac() const { return (psw & kRl78FlagAc) != 0; }
    bool flag_ie() const { return (psw & kRl78FlagIe) != 0; }
    void set_flag(u8 mask, bool value) {
        if (value) psw = static_cast<u16>(psw | mask);
        else psw = static_cast<u16>(psw & ~mask);
    }

    /// ISP0/ISP1 (PSW bits 2:1) as a 2-bit value.
    int isp_level() const { return (psw >> 1) & 3; }

    /// Register bank select (RBS0 = PSW.3, RBS1 = PSW.5).
    int bank() const { return ((psw >> 3) & 1) | (((psw >> 5) & 1) << 1); }

    /// Register bank area base: 0xFFEE0 + 8 * bank.
    u32 bank_base() const {
        return kRl78BankBase + kRl78BankStride * static_cast<u32>(bank());
    }

    bool interrupts_enabled() const { return (psw & kRl78FlagIe) != 0; }

    // ------------------------------------------------------------------
    // Register access (also used by the tests)
    // ------------------------------------------------------------------
    u8 get_reg8(Rl78Reg reg) const;
    void set_reg8(Rl78Reg reg, u8 value);
    u16 get_reg16(Rl78Reg reg) const;
    void set_reg16(Rl78Reg reg, u16 value);

    /// Byte/word data access with the control registers mirrored.
    u8 read_data8(u32 address);
    void write_data8(u32 address, u8 value);
    u16 read_data16(u32 address);
    void write_data16(u32 address, u16 value);

    /// Request a maskable interrupt; taken before the next instruction when
    /// PSW.IE is set, otherwise dropped (the RL78 has no NMI: non-maskable
    /// events are reset sources).
    void request_interrupt(int vector);

    /// Default vector used when `set_irq` asserts an input line.  The mapping
    /// belongs to the INTC block, which does not exist yet, so the machine layer
    /// programs it explicitly; unprogrammed lines use vector 0.
    void set_irq_vector(int line, int vector);
    int irq_vector(int line) const;

    /// Cycle estimate for one instruction (SFR / short-direct / direct accesses
    /// take longer).
    static int cycles_for(const Rl78Decoded& insn);

private:
    /// Address page for data accesses: the 0xF0000 page, or the ES page while
    /// an 0x11 ES: prefix is in effect.
    u32 page_ = 0xF0000;

    /// Set by execute() when the instruction wrote PC itself.  step() uses it to
    /// tell "this instruction does not touch PC, so advance it" apart from "this
    /// branch/jump/return moved PC onto the address it already had", which is a
    /// real transfer (a self loop) and must not be advanced.
    bool pc_written_ = false;

    std::array<bool, 8> irq_levels_{};
    std::array<u8, 8> irq_vectors_{};

    u32 addrof(const Rl78Operand& op) const;

    u8 read_op(const Rl78Operand& op);
    void write_op(const Rl78Operand& op, u8 value);
    u16 read_op_word(const Rl78Operand& op);
    void write_op_word(const Rl78Operand& op, u16 value);
    bool read_bit(const Rl78Operand& op);
    void write_bit(const Rl78Operand& op, bool value);

    void execute(const Rl78Decoded& insn);
    void take_pending();

    /// MOV, including the CLRW/ONEW word forms and the mul/div fallback.
    void do_mov(const Rl78Decoded& insn);

    // RL78/G14 multiply-accumulate and divide instructions.
    void mul_u();
    void mul_hu(bool signed_operands);
    void div_hu();
    void div_wu();
    void mac_hu(bool signed_operands);

    void push(u16 value);
    u16 pop();
    void push_frame(u16 return_pc, u16 return_psw);
    u32 pop_frame(u16& return_psw);
};

}  // namespace zlb
