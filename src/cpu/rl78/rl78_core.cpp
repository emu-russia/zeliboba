// zeliboba - Renesas RL78 interpreter (see rl78_core.h).
#include "cpu/rl78/rl78_core.h"

#include <cstring>
#include <memory>
#include <string>
#include <type_traits>

#include "common/log.h"
#include "common/util.h"
#include "cpu/factory.h"
#include "cpu/rl78/rl78_disasm.h"

namespace zlb {
namespace {

// ---------------------------------------------------------------------------
// Per-row execution opcode.
//
// Computed once from the row's semantics id, its mnemonic and the shape of its
// operands, so the hot path never compares strings.
// ---------------------------------------------------------------------------
enum ExecOp : u8 {
    kEoNop = 0,
    kEoHalt,
    kEoStop,
    kEoBreak,
    kEoRet,
    kEoReti,
    kEoSel,
    kEoSkip,
    kEoMov,
    kEoMovs,
    kEoPush,
    kEoPop,
    kEoBitMov,
    kEoXch,
    kEoAdd,
    kEoAddc,
    kEoSub,
    kEoSubc,
    kEoCmp,
    kEoLogic,
    kEoBitLogic,
    kEoNot,
    kEoShift,
    kEoBranch,
    kEoBCond,
    kEoBtclr,
    kEoMulDiv,
    kEoMulu,
    kEoCallt,
    kEoUnknown,
};

bool mnemonic_is(const Rl78Row& row, const char* text) {
    return std::strcmp(kRl78Mnemonics[row.mnemonic], text) == 0;
}

/// One shift/rotate step: updates `value`, returns the bit shifted out (which
/// becomes CY and feeds the next ROLC/RORC step).
template <typename T>
bool shift_step(Rl78Id id, T& value, bool carry_in) {
    constexpr unsigned kTop = sizeof(T) * 8 - 1;
    const T high_bit = static_cast<T>(T(1) << kTop);
    const int carry = carry_in ? 1 : 0;
    bool out_bit;
    switch (id) {
        case Rl78Id::Shl:
            out_bit = (value & high_bit) != 0;
            value = static_cast<T>(value << 1);
            break;
        case Rl78Id::Shr:
            out_bit = (value & 1) != 0;
            value = static_cast<T>(value >> 1);
            break;
        case Rl78Id::Sar:
            out_bit = (value & 1) != 0;
            value = static_cast<T>(std::make_signed_t<T>(value) >> 1);
            break;
        case Rl78Id::Rol:
            out_bit = (value & high_bit) != 0;
            value = static_cast<T>(static_cast<T>(value << 1) | (out_bit ? T(1) : T(0)));
            break;
        case Rl78Id::Ror:
            out_bit = (value & 1) != 0;
            value = static_cast<T>(static_cast<T>(value >> 1) | (out_bit ? high_bit : T(0)));
            break;
        case Rl78Id::Rolc:
            out_bit = (value & high_bit) != 0;
            value = static_cast<T>(static_cast<T>(value << 1) | static_cast<T>(carry));
            break;
        default:  // Rorc
            out_bit = (value & 1) != 0;
            value = static_cast<T>(static_cast<T>(value >> 1) | static_cast<T>(carry << kTop));
            break;
    }
    return out_bit;
}

const std::array<u8, kRl78RowCount>& exec_ops() {
    static const std::array<u8, kRl78RowCount> table = [] {
        std::array<u8, kRl78RowCount> result{};
        for (int row = 0; row < kRl78RowCount; ++row) {
            const Rl78Row& entry = kRl78Rows[row];
            const u8 op0_type = kRl78Descs[row * 2].type;
            const u8 op0_add = kRl78Descs[row * 2].ra;
            const u8 op1_type = kRl78Descs[row * 2 + 1].type;
            u8 exec = kEoUnknown;
            switch (static_cast<Rl78Id>(entry.id)) {
                case Rl78Id::Unknown: exec = kEoUnknown; break;
                case Rl78Id::Nop: exec = kEoNop; break;
                case Rl78Id::Halt: exec = kEoHalt; break;
                case Rl78Id::Stop: exec = kEoStop; break;
                case Rl78Id::Break: exec = kEoBreak; break;
                case Rl78Id::Ret: exec = kEoRet; break;
                case Rl78Id::Reti: exec = kEoReti; break;
                case Rl78Id::Sel: exec = kEoSel; break;
                case Rl78Id::Skip: exec = kEoSkip; break;
                case Rl78Id::Mulu: exec = kEoMulu; break;
                case Rl78Id::Divhu:
                case Rl78Id::Divwu:
                case Rl78Id::Mulhu:
                case Rl78Id::Mulh:
                case Rl78Id::Mach:
                case Rl78Id::Machu: exec = kEoMulDiv; break;
                case Rl78Id::Xch: exec = kEoXch; break;
                case Rl78Id::Add: exec = kEoAdd; break;
                case Rl78Id::Addc: exec = kEoAddc; break;
                case Rl78Id::Sub: exec = kEoSub; break;
                case Rl78Id::Subc: exec = kEoSubc; break;
                case Rl78Id::Cmp: exec = kEoCmp; break;
                case Rl78Id::Shl:
                case Rl78Id::Shr:
                case Rl78Id::Sar:
                case Rl78Id::Rol:
                case Rl78Id::Ror:
                case Rl78Id::Rolc:
                case Rl78Id::Rorc: exec = kEoShift; break;
                case Rl78Id::Branch: exec = kEoBranch; break;
                case Rl78Id::Call:
                    exec = mnemonic_is(entry, "callt") ? kEoCallt : kEoBranch;
                    break;
                case Rl78Id::BranchCond: exec = kEoBCond; break;
                case Rl78Id::BranchCondClear: exec = kEoBtclr; break;
                case Rl78Id::And:
                case Rl78Id::Or:
                    exec = op0_type == static_cast<u8>(Rl78OpType::Bit) ? kEoBitLogic : kEoLogic;
                    break;
                case Rl78Id::Xor:
                    if (mnemonic_is(entry, "not1")) {
                        exec = kEoNot;
                    } else {
                        exec = op0_type == static_cast<u8>(Rl78OpType::Bit) ? kEoBitLogic : kEoLogic;
                    }
                    break;
                case Rl78Id::Mov:
                    if (mnemonic_is(entry, "push")) {
                        exec = kEoPush;
                    } else if (mnemonic_is(entry, "pop")) {
                        exec = kEoPop;
                    } else if (mnemonic_is(entry, "movs")) {
                        exec = kEoMovs;
                    } else if (mnemonic_is(entry, "set1") || mnemonic_is(entry, "clr1") ||
                               mnemonic_is(entry, "mov1")) {
                        exec = kEoBitMov;
                    } else if (op0_type == static_cast<u8>(Rl78OpType::Ind) &&
                               op0_add == kRl78AkSfr &&
                               op1_type == static_cast<u8>(Rl78OpType::Imm)) {
                        // "mov %s0, #%1": the RL78/G14 mul/div/mach multiplex.
                        exec = kEoMulDiv;
                    } else {
                        exec = kEoMov;
                    }
                    break;
                default: exec = kEoUnknown; break;
            }
            result[static_cast<size_t>(row)] = exec;
        }
        return result;
    }();
    return table;
}

}  // namespace

// ---------------------------------------------------------------------------
// Construction / reset
// ---------------------------------------------------------------------------

Rl78Core::Rl78Core(Bus& bus) : Cpu(bus) {
    name = "RL78";
    (void)exec_ops();
}

void Rl78Core::reset() {
    instructions = 0;
    cycles = 0;
    data_accesses = 0;
    unknown_instructions = 0;
    halted = false;
    halt_reason.clear();
    undefined_instruction = false;

    a = x = b = c = d = e = h = l = 0;
    psw = 0x06;
    es = 0x0F;
    cs = 0;
    pmc = 0;
    sp = 0;
    page_ = 0xF0000;
    pending_vector = -1;
    irq_levels_.fill(false);

    bus->context.pc = 0;
    bus->context.core = name.c_str();
    reset_vector = bus->read16(0);
    pc = reset_vector & 0xFFFFu;
}

void Rl78Core::reset(u32 entry) {
    reset();
    pc = entry & 0xFFFFFu;
}

// ---------------------------------------------------------------------------
// Register file
// ---------------------------------------------------------------------------

u8 Rl78Core::get_reg8(Rl78Reg reg) const {
    switch (reg) {
        case Rl78Reg::X: return x;
        case Rl78Reg::A: return a;
        case Rl78Reg::C: return c;
        case Rl78Reg::B: return b;
        case Rl78Reg::E: return e;
        case Rl78Reg::D: return d;
        case Rl78Reg::L: return l;
        case Rl78Reg::H: return h;
        case Rl78Reg::ES: return es;
        case Rl78Reg::CS: return cs;
        case Rl78Reg::PSW: return static_cast<u8>(psw & 0xFF);
        default: return 0;
    }
}

void Rl78Core::set_reg8(Rl78Reg reg, u8 value) {
    switch (reg) {
        case Rl78Reg::X: x = value; break;
        case Rl78Reg::A: a = value; break;
        case Rl78Reg::C: c = value; break;
        case Rl78Reg::B: b = value; break;
        case Rl78Reg::E: e = value; break;
        case Rl78Reg::D: d = value; break;
        case Rl78Reg::L: l = value; break;
        case Rl78Reg::H: h = value; break;
        case Rl78Reg::ES: es = value; break;
        case Rl78Reg::CS: cs = value; break;
        case Rl78Reg::PSW: psw = static_cast<u16>((psw & 0xFF00) | value); break;
        default: break;
    }
}

u16 Rl78Core::get_reg16(Rl78Reg reg) const {
    switch (reg) {
        case Rl78Reg::AX: return static_cast<u16>((a << 8) | x);
        case Rl78Reg::BC: return static_cast<u16>((b << 8) | c);
        case Rl78Reg::DE: return static_cast<u16>((d << 8) | e);
        case Rl78Reg::HL: return static_cast<u16>((h << 8) | l);
        case Rl78Reg::SP: return sp;
        case Rl78Reg::PSW: return psw;
        case Rl78Reg::ES: return es;
        case Rl78Reg::CS: return cs;
        default: return get_reg8(reg);
    }
}

void Rl78Core::set_reg16(Rl78Reg reg, u16 value) {
    switch (reg) {
        case Rl78Reg::AX: a = static_cast<u8>(value >> 8); x = static_cast<u8>(value); break;
        case Rl78Reg::BC: b = static_cast<u8>(value >> 8); c = static_cast<u8>(value); break;
        case Rl78Reg::DE: d = static_cast<u8>(value >> 8); e = static_cast<u8>(value); break;
        case Rl78Reg::HL: h = static_cast<u8>(value >> 8); l = static_cast<u8>(value); break;
        case Rl78Reg::SP: sp = value; break;
        case Rl78Reg::PSW: psw = value; break;
        case Rl78Reg::ES: es = static_cast<u8>(value); break;
        case Rl78Reg::CS: cs = static_cast<u8>(value); break;
        default: set_reg8(reg, static_cast<u8>(value)); break;
    }
}

// ---------------------------------------------------------------------------
// Data access with the control registers mirrored
//
// SP/PSW/CS/ES/PMC are memory mapped on real silicon but live in the core here:
// serving them from RAM is what made "movw sp, #0xFE20" a store to 0xFFFF8 and
// left the reset handler without a stack (defect #1 of the RL78 handoff).
// ---------------------------------------------------------------------------

u8 Rl78Core::read_data8(u32 address) {
    const u32 target = address & 0xFFFFFu;
    ++data_accesses;
    switch (target) {
        case kRl78SfrPsw: return static_cast<u8>(psw & 0xFF);
        case kRl78SfrEs: return es;
        case kRl78SfrCs: return cs;
        case kRl78SfrSp: return static_cast<u8>(sp & 0xFF);
        case kRl78SfrPmc: return pmc;
        default: return bus->read8(target);
    }
}

void Rl78Core::write_data8(u32 address, u8 value) {
    const u32 target = address & 0xFFFFFu;
    ++data_accesses;
    switch (target) {
        case kRl78SfrPsw: psw = static_cast<u16>((psw & 0xFF00) | value); return;
        case kRl78SfrEs: es = value; return;
        case kRl78SfrCs: cs = value; return;
        case kRl78SfrSp: sp = static_cast<u16>((sp & 0xFF00) | value); return;
        case kRl78SfrPmc: pmc = value; return;
        default: bus->write8(target, value); return;
    }
}

u16 Rl78Core::read_data16(u32 address) {
    const u32 target = address & 0xFFFFFu;
    data_accesses += 2;
    switch (target) {
        case kRl78SfrPsw: return psw;
        case kRl78SfrSp: return sp;
        case kRl78SfrEs: return static_cast<u16>(es | (pmc << 8));
        case kRl78SfrPmc: return static_cast<u16>(pmc | (bus->read8(target + 1) << 8));
        default: return bus->read16(target);
    }
}

void Rl78Core::write_data16(u32 address, u16 value) {
    const u32 target = address & 0xFFFFFu;
    data_accesses += 2;
    switch (target) {
        case kRl78SfrPsw: psw = value; return;
        case kRl78SfrSp: sp = value; return;
        case kRl78SfrEs:
            es = static_cast<u8>(value);
            pmc = static_cast<u8>(value >> 8);
            return;
        case kRl78SfrPmc:
            pmc = static_cast<u8>(value);
            bus->write8(target + 1, static_cast<u8>(value >> 8));
            return;
        default: bus->write16(target, value); return;
    }
}

// ---------------------------------------------------------------------------
// Operand addressing
// ---------------------------------------------------------------------------

u32 Rl78Core::addrof(const Rl78Operand& op) const {
    if (op.reg == Rl78Reg::None) {
        switch (op.add_kind) {
            case kRl78AkCallt:
                // The CALLT table entry address itself.
                return op.address & 0xFFFFFu;
            case kRl78AkImmu2:
            case kRl78AkImmu3:
                // !addr16 / !addr20: the page comes from ES (0xF0000 by default).
                return (page_ | (static_cast<u32>(op.addend) & 0xFFFFu)) & 0xFFFFFu;
            default:
                // saddr / sfr / pc-relative targets are already absolute.
                return op.address & 0xFFFFFu;
        }
    }

    // Register indirect / based: [reg], [reg+disp] and [hl+b] / [hl+c].  The
    // addend is the displacement, *not* an absolute address, so the page is
    // added to the sum.  (The reference C# core short-circuits the two-byte
    // addend kinds before looking at the base register, which loses B/C/BC/HL
    // for `mov a, 0x0100[b]` and friends; the disassembly already prints those
    // as based operands, so the address has to include the base here.)
    u32 offset = static_cast<u32>(op.addend);
    if (op.reg2 != Rl78Reg::None) offset += get_reg8(op.reg2);
    switch (op.reg) {
        case Rl78Reg::HL: return (page_ + static_cast<u32>(((h << 8) | l) + offset)) & 0xFFFFFu;
        case Rl78Reg::DE: return (page_ + static_cast<u32>(((d << 8) | e) + offset)) & 0xFFFFFu;
        case Rl78Reg::SP: return (page_ + static_cast<u32>(sp + offset)) & 0xFFFFFu;
        case Rl78Reg::B: return (page_ + (static_cast<u32>(b) + offset)) & 0xFFFFFu;
        case Rl78Reg::C: return (page_ + (static_cast<u32>(c) + offset)) & 0xFFFFFu;
        case Rl78Reg::BC: return (page_ + static_cast<u32>(((b << 8) | c) + offset)) & 0xFFFFFu;
        case Rl78Reg::A: return (page_ + (static_cast<u32>(a) + offset)) & 0xFFFFFu;
        case Rl78Reg::X: return (page_ + (static_cast<u32>(x) + offset)) & 0xFFFFFu;
        default: return (page_ + offset) & 0xFFFFFu;
    }
}

u8 Rl78Core::read_op(const Rl78Operand& op) {
    switch (op.type) {
        case Rl78OpType::Imm: return static_cast<u8>(op.addend);
        case Rl78OpType::Reg: return get_reg8(op.reg);
        case Rl78OpType::Ind: return read_data8(addrof(op));
        case Rl78OpType::Bit:
        case Rl78OpType::BitInd: return static_cast<u8>(read_bit(op) ? 1 : 0);
        default: return 0;
    }
}

void Rl78Core::write_op(const Rl78Operand& op, u8 value) {
    switch (op.type) {
        case Rl78OpType::Reg: set_reg8(op.reg, value); return;
        case Rl78OpType::Ind: write_data8(addrof(op), value); return;
        case Rl78OpType::Bit:
        case Rl78OpType::BitInd: write_bit(op, value != 0); return;
        default: return;
    }
}

u16 Rl78Core::read_op_word(const Rl78Operand& op) {
    switch (op.type) {
        case Rl78OpType::Imm: return static_cast<u16>(op.addend);
        case Rl78OpType::Reg: return get_reg16(op.reg);
        case Rl78OpType::Ind: return read_data16(addrof(op));
        default: return 0;
    }
}

void Rl78Core::write_op_word(const Rl78Operand& op, u16 value) {
    switch (op.type) {
        case Rl78OpType::Reg: set_reg16(op.reg, value); return;
        case Rl78OpType::Ind: write_data16(addrof(op), value); return;
        default: return;
    }
}

bool Rl78Core::read_bit(const Rl78Operand& op) {
    const int bit = op.bit & 7;
    switch (op.type) {
        case Rl78OpType::Bit:
            if (op.reg == Rl78Reg::PSW) return (psw & (1u << bit)) != 0;
            if (op.reg == Rl78Reg::A) return ((a >> bit) & 1) != 0;
            return false;
        case Rl78OpType::BitInd:
            return ((read_data8(addrof(op)) >> bit) & 1) != 0;
        default:
            return false;
    }
}

void Rl78Core::write_bit(const Rl78Operand& op, bool value) {
    const int bit = op.bit & 7;
    switch (op.type) {
        case Rl78OpType::Bit:
            if (op.reg == Rl78Reg::PSW) {
                if (value) psw = static_cast<u16>(psw | (1u << bit));
                else psw = static_cast<u16>(psw & ~(1u << bit));
            } else if (op.reg == Rl78Reg::A) {
                a = static_cast<u8>(value ? (a | (1 << bit)) : (a & ~(1 << bit)));
            }
            return;
        case Rl78OpType::BitInd: {
            const u32 address = addrof(op);
            const u8 current = read_data8(address);
            write_data8(address, static_cast<u8>(value ? (current | (1 << bit))
                                                       : (current & ~(1 << bit))));
            return;
        }
        default:
            return;
    }
}

// ---------------------------------------------------------------------------
// Stack
// ---------------------------------------------------------------------------

void Rl78Core::push(u16 value) {
    sp = static_cast<u16>(sp - 2);
    write_data16(0xF0000u | sp, value);
}

u16 Rl78Core::pop() {
    const u16 value = read_data16(0xF0000u | sp);
    sp = static_cast<u16>(sp + 2);
    return value;
}

void Rl78Core::push_frame(u16 return_pc, u16 return_psw) {
    sp = static_cast<u16>(sp - 1);
    write_data8(0xF0000u | sp, static_cast<u8>(return_psw & 0xFF));
    sp = static_cast<u16>(sp - 3);
    write_data8(0xF0000u | sp, static_cast<u8>(return_pc & 0xFF));
    write_data8(0xF0000u | (sp + 1), static_cast<u8>(return_pc >> 8));
    write_data8(0xF0000u | (sp + 2), 0);
}

u32 Rl78Core::pop_frame(u16& return_psw) {
    u32 return_pc = static_cast<u32>(read_data8(0xF0000u | sp));
    return_pc |= static_cast<u32>(read_data8(0xF0000u | (sp + 1))) << 8;
    sp = static_cast<u16>(sp + 3);
    return_psw = read_data8(0xF0000u | sp);
    sp = static_cast<u16>(sp + 1);
    return return_pc & 0xFFFFu;
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

void Rl78Core::request_interrupt(int vector) {
    pending_vector = vector & 0x3F;
    if (!interrupts_enabled()) {
        ZLB_LOG_DBG("cpu", "RL78 interrupt vector %d requested with IE=0, held", pending_vector);
    }
}

bool Rl78Core::interrupt_pending() const {
    return pending_vector >= 0 && interrupts_enabled();
}

void Rl78Core::set_irq(int line, bool asserted) {
    if (line < 0 || line >= static_cast<int>(irq_levels_.size())) return;
    irq_levels_[static_cast<size_t>(line)] = asserted;
    if (asserted) request_interrupt(irq_vectors_[static_cast<size_t>(line)]);
}

void Rl78Core::set_irq_vector(int line, int vector) {
    if (line < 0 || line >= static_cast<int>(irq_vectors_.size())) return;
    irq_vectors_[static_cast<size_t>(line)] = static_cast<u8>(vector & 0x3F);
}

int Rl78Core::irq_vector(int line) const {
    if (line < 0 || line >= static_cast<int>(irq_vectors_.size())) return 0;
    return irq_vectors_[static_cast<size_t>(line)];
}

void Rl78Core::take_pending() {
    const int vector = pending_vector;
    pending_vector = -1;
    push_frame(static_cast<u16>(pc), psw);
    psw = static_cast<u16>(psw & ~kRl78FlagIe);
    pc = bus->read16(static_cast<u32>(vector) * 2) & 0xFFFFu;
    ZLB_LOG_DBG("cpu", "RL78 interrupt vector %d -> PC=0x%05X", vector, pc);
}

void Rl78Core::do_mov(const Rl78Decoded& insn) {
    // CLRW/ONEW are word operations even though the .opc does not mark them with
    // W() (their only operand is always a register pair).
    const bool word = insn.is_word || std::strcmp(insn.mnemonic, "clrw") == 0 ||
                      std::strcmp(insn.mnemonic, "onew") == 0;
    if (word) write_op_word(insn.ops[0], read_op_word(insn.ops[1]));
    else write_op(insn.ops[0], read_op(insn.ops[1]));
}

void Rl78Core::mul_u() {
    const u16 product = static_cast<u16>(a * x);
    a = static_cast<u8>(product >> 8);
    x = static_cast<u8>(product);
}

void Rl78Core::mul_hu(bool signed_operands) {
    u32 product;
    if (signed_operands) {
        const s32 lhs = static_cast<s16>(static_cast<u16>((a << 8) | x));
        const s32 rhs = static_cast<s16>(static_cast<u16>((b << 8) | c));
        product = static_cast<u32>(lhs * rhs);
    } else {
        product = static_cast<u32>((a << 8) | x) * static_cast<u32>((b << 8) | c);
    }
    a = static_cast<u8>(product >> 8);
    x = static_cast<u8>(product);
    b = static_cast<u8>(product >> 24);
    c = static_cast<u8>(product >> 16);
}

void Rl78Core::div_hu() {
    const u16 dividend = static_cast<u16>((a << 8) | x);
    const u16 divisor = static_cast<u16>((d << 8) | e);
    if (divisor == 0) {
        a = 0xFF;
        x = 0xFF;
        d = 0xFF;
        e = 0xFF;
        return;
    }
    const u16 quotient = static_cast<u16>(dividend / divisor);
    const u16 remainder = static_cast<u16>(dividend % divisor);
    a = static_cast<u8>(quotient >> 8);
    x = static_cast<u8>(quotient);
    d = static_cast<u8>(remainder >> 8);
    e = static_cast<u8>(remainder);
}

void Rl78Core::div_wu() {
    const u32 dividend = (static_cast<u32>((b << 8) | c) << 16) | static_cast<u32>((a << 8) | x);
    const u32 divisor = (static_cast<u32>((d << 8) | e) << 16) | static_cast<u32>((h << 8) | l);
    if (divisor == 0) {
        a = x = b = c = 0xFF;
        d = e = h = l = 0xFF;
        return;
    }
    const u32 quotient = dividend / divisor;
    const u32 remainder = dividend % divisor;
    a = static_cast<u8>(quotient >> 24);
    x = static_cast<u8>(quotient >> 16);
    b = static_cast<u8>(quotient >> 8);
    c = static_cast<u8>(quotient);
    d = static_cast<u8>(remainder >> 24);
    e = static_cast<u8>(remainder >> 16);
    h = static_cast<u8>(remainder >> 8);
    l = static_cast<u8>(remainder);
}

void Rl78Core::mac_hu(bool signed_operands) {
    const u32 accumulator = (static_cast<u32>((d << 8) | e) << 16) | static_cast<u32>((h << 8) | l);
    u32 product;
    if (signed_operands) {
        const s32 lhs = static_cast<s16>(static_cast<u16>((a << 8) | x));
        const s32 rhs = static_cast<s16>(static_cast<u16>((b << 8) | c));
        product = static_cast<u32>(lhs * rhs);
    } else {
        product = static_cast<u32>((a << 8) | x) * static_cast<u32>((b << 8) | c);
    }
    const u64 sum = static_cast<u64>(accumulator) + static_cast<u64>(product);
    set_flag(kRl78FlagCy, sum > 0xFFFFFFFFull);
    set_flag(kRl78FlagAc, signed_operands && ((product >> 31) != 0));
    const u32 result = static_cast<u32>(sum);
    write_data8(0xFFFF0, static_cast<u8>(result));
    write_data8(0xFFFF1, static_cast<u8>(result >> 8));
    write_data8(0xFFFF2, static_cast<u8>(result >> 16));
    write_data8(0xFFFF3, static_cast<u8>(result >> 24));
}

// ---------------------------------------------------------------------------
// step
// ---------------------------------------------------------------------------

int Rl78Core::cycles_for(const Rl78Decoded& insn) {
    int cost = 1;
    for (int i = 0; i < 2; ++i) {
        const Rl78Operand& op = insn.ops[i];
        if (op.type != Rl78OpType::Ind) continue;
        switch (op.add_kind) {
            case kRl78AkSfr:
            case kRl78AkSaddr:
            case kRl78AkImmu2:
            case kRl78AkImmu3: cost = 4; break;
            default: break;
        }
    }
    return cost;
}

StepResult Rl78Core::step() {
    StepResult result;
    result.address = pc;
    bus->context.pc = pc;
    bus->context.core = name.c_str();

    if (pending_vector >= 0 && interrupts_enabled()) take_pending();

    Rl78Decoded insn;
    if (!rl78_decode(*bus, pc, insn)) {
        undefined_instruction = true;
        ++unknown_instructions;
        result.length = 1;
        result.text = "??";
        result.faulted = true;
        result.fault = format("undefined instruction at 0x%05X", pc);
        ++instructions;
        // Skip the single byte that could not be decoded so a caller can
        // resynchronise; the instruction was not executed.
        pc = (pc + 1) & 0xFFFFFu;
        halt(result.fault);
        return result;
    }

    result.length = insn.length;
    // See Cpu::step_text: the listing is not read by the machine or the debugger,
    // so it is only formatted when a tool asks for it.
    if (step_text) result.text = rl78_format_instruction(insn);
    result.was_branch = insn.id == Rl78Id::Branch || insn.id == Rl78Id::BranchCond ||
                        insn.id == Rl78Id::BranchCondClear || insn.id == Rl78Id::Call ||
                        insn.id == Rl78Id::Ret || insn.id == Rl78Id::Reti ||
                        insn.id == Rl78Id::Skip || insn.id == Rl78Id::Break;
    ++instructions;

    page_ = insn.has_es_prefix ? (static_cast<u32>(es & 0x0F) << 16) : 0xF0000u;

    // A transfer to the address the instruction already has (the classic
    // "br $-2" / "bnz $-2" idle loop) leaves PC numerically unchanged, so the
    // old `pc == before` test in this function could not tell it apart from an
    // instruction that never writes PC: the loop was advanced by insn.length and
    // fell through into whatever followed.  execute() now reports whether it
    // wrote PC instead.  Evidence: `EF FE` at 0x1000 (br $-2) used to end with
    // PC=0x1002 instead of 0x1000; tests/test_rl78.cpp
    // rl78_self_branches_do_not_fall_through pins the fixed behaviour.
    pc_written_ = false;
    execute(insn);
    cycles += static_cast<u64>(cycles_for(insn));
    if (!halted && !pc_written_) pc = (pc + insn.length) & 0xFFFFFu;
    bus->context.pc = pc;
    return result;
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------

void Rl78Core::execute(const Rl78Decoded& insn) {
    const char* mnemonic = insn.mnemonic;
    const u8 flags = insn.flags;

    switch (exec_ops()[static_cast<size_t>(insn.row)]) {
        case kEoNop:
            break;

        case kEoHalt:
            halt("HALT");
            break;

        case kEoStop:
            halt("STOP");
            break;

        case kEoBreak:
            push_frame(static_cast<u16>(pc + insn.length), psw);
            psw = static_cast<u16>(psw & ~kRl78FlagIe);
            pc = bus->read16(0x7E) & 0xFFFFu;
            pc_written_ = true;
            ZLB_LOG_DBG("cpu", "RL78 BRK -> vector 0x007E, PC=0x%05X", pc);
            break;

        case kEoRet:
            pc = pop();
            pc_written_ = true;
            break;

        case kEoReti: {
            u16 return_psw = 0;
            const u32 return_pc = pop_frame(return_psw);
            pc = return_pc;
            psw = return_psw;
            pc_written_ = true;
            ZLB_LOG_DBG("cpu", "RL78 RETI -> PC=0x%05X", pc);
            break;
        }

        case kEoSel: {
            // SEL RBn: op[1] is the immediate bank number.
            const int selected = insn.ops[1].addend & 3;
            psw = static_cast<u16>((psw & ~(kRl78FlagRbs0 | kRl78FlagRbs1)) |
                                   ((selected & 1) != 0 ? kRl78FlagRbs0 : 0) |
                                   ((selected & 2) != 0 ? kRl78FlagRbs1 : 0));
            break;
        }

        case kEoSkip: {
            bool take;
            switch (insn.ops[1].condition) {
                case Rl78Cond::C: take = flag_cy(); break;
                case Rl78Cond::NC: take = !flag_cy(); break;
                case Rl78Cond::Z: take = flag_z(); break;
                case Rl78Cond::NZ: take = !flag_z(); break;
                case Rl78Cond::H: take = !flag_cy() && !flag_z(); break;
                case Rl78Cond::NH: take = flag_cy() || flag_z(); break;
                case Rl78Cond::T: take = true; break;
                default: take = false; break;
            }
            if (take) {
                const u32 next = pc + insn.length;
                pc = next + rl78_instruction_length(*bus, next);
                pc_written_ = true;
            }
            break;
        }

        case kEoMov:
            do_mov(insn);
            break;

        case kEoMovs: {
            // MOVS [HL+byte], X: store X and set Z/CY/AC.
            const u8 offset = static_cast<u8>(insn.ops[0].addend & 0xFF);
            write_data8(addrof(insn.ops[0]), x);
            set_flag(kRl78FlagZ, x == 0);
            set_flag(kRl78FlagCy, x == 0 || offset == 0);
            set_flag(kRl78FlagAc, (offset & 0x0F) > (x & 0x0F));
            break;
        }

        case kEoBitMov: {
            // SET1 / CLR1 / MOV1.  One operand may be the CY bit, which the .opc
            // writes as PSW.0 (DCY/SCY).
            const Rl78Operand& dst = insn.ops[0];
            const Rl78Operand& src = insn.ops[1];
            const bool dst_cy = dst.type == Rl78OpType::Bit && dst.reg == Rl78Reg::PSW;
            const bool src_cy = src.type == Rl78OpType::Bit && src.reg == Rl78Reg::PSW;
            if (dst_cy) {
                set_flag(kRl78FlagCy, src.type == Rl78OpType::Imm ? src.addend != 0 : read_bit(src));
                break;
            }
            if (src_cy) {
                write_bit(dst, flag_cy());
                break;
            }
            if (src.type == Rl78OpType::Imm) write_bit(dst, src.addend != 0);
            else write_bit(dst, read_bit(src));
            break;
        }

        case kEoPush: {
            const Rl78Operand& src =
                insn.ops[0].type == Rl78OpType::PreDec ? insn.ops[1] : insn.ops[0];
            push(read_op_word(src));
            break;
        }

        case kEoPop: {
            const Rl78Operand& dst =
                insn.ops[0].type == Rl78OpType::PostInc ? insn.ops[1] : insn.ops[0];
            write_op_word(dst, pop());
            break;
        }

        case kEoXch: {
            if (insn.is_word) {
                const u16 lhs = read_op_word(insn.ops[0]);
                const u16 rhs = read_op_word(insn.ops[1]);
                write_op_word(insn.ops[0], rhs);
                write_op_word(insn.ops[1], lhs);
                break;
            }
            const u8 lhs = read_op(insn.ops[0]);
            const u8 rhs = read_op(insn.ops[1]);
            write_op(insn.ops[0], rhs);
            write_op(insn.ops[1], lhs);
            break;
        }

        case kEoBitLogic: {
            // AND1 / OR1 / XOR1 CY, bit.
            const bool value = read_bit(insn.ops[1]);
            const bool carry = flag_cy();
            const bool result = insn.id == Rl78Id::And ? (carry && value)
                                : insn.id == Rl78Id::Or ? (carry || value)
                                                        : (carry != value);
            set_flag(kRl78FlagCy, result);
            break;
        }

        case kEoNot:
            set_flag(kRl78FlagCy, !flag_cy());
            break;

        case kEoAdd:
        case kEoAddc: {
            const bool with_carry = insn.id == Rl78Id::Addc;
            if (insn.is_word) {
                const u16 dst = read_op_word(insn.ops[0]);
                const u16 src = read_op_word(insn.ops[1]);
                const int carry = with_carry && flag_cy() ? 1 : 0;
                const int sum = dst + src + carry;
                const u16 result = static_cast<u16>(sum);
                write_op_word(insn.ops[0], result);
                if ((flags & kRl78FlagZ) != 0) set_flag(kRl78FlagZ, result == 0);
                if ((flags & kRl78FlagCy) != 0) set_flag(kRl78FlagCy, sum > 0xFFFF);
                if ((flags & kRl78FlagAc) != 0) {
                    set_flag(kRl78FlagAc, (((dst & 0xFFF) + (src & 0xFFF) + carry) & 0x1000) != 0);
                }
                break;
            }
            const u8 dst = read_op(insn.ops[0]);
            const u8 src = read_op(insn.ops[1]);
            const int carry = with_carry && flag_cy() ? 1 : 0;
            const int sum = dst + src + carry;
            const u8 result = static_cast<u8>(sum);
            write_op(insn.ops[0], result);
            if ((flags & kRl78FlagZ) != 0) set_flag(kRl78FlagZ, result == 0);
            if ((flags & kRl78FlagCy) != 0) set_flag(kRl78FlagCy, sum > 0xFF);
            if ((flags & kRl78FlagAc) != 0) {
                set_flag(kRl78FlagAc, (((dst & 0xF) + (src & 0xF) + carry) & 0x10) != 0);
            }
            break;
        }

        case kEoSub:
        case kEoSubc: {
            const bool with_carry = insn.id == Rl78Id::Subc;
            if (insn.is_word) {
                const u16 dst = read_op_word(insn.ops[0]);
                const u16 src = read_op_word(insn.ops[1]);
                const int borrow = with_carry && flag_cy() ? 1 : 0;
                const int difference = dst - src - borrow;
                const u16 result = static_cast<u16>(difference);
                write_op_word(insn.ops[0], result);
                if ((flags & kRl78FlagZ) != 0) set_flag(kRl78FlagZ, result == 0);
                if ((flags & kRl78FlagCy) != 0) set_flag(kRl78FlagCy, difference < 0);
                if ((flags & kRl78FlagAc) != 0) {
                    set_flag(kRl78FlagAc, (dst & 0xF) < ((src & 0xF) + borrow));
                }
                break;
            }
            const u8 dst = read_op(insn.ops[0]);
            const u8 src = read_op(insn.ops[1]);
            const int borrow = with_carry && flag_cy() ? 1 : 0;
            const int difference = dst - src - borrow;
            const u8 result = static_cast<u8>(difference);
            write_op(insn.ops[0], result);
            if ((flags & kRl78FlagZ) != 0) set_flag(kRl78FlagZ, result == 0);
            if ((flags & kRl78FlagCy) != 0) set_flag(kRl78FlagCy, difference < 0);
            if ((flags & kRl78FlagAc) != 0) {
                set_flag(kRl78FlagAc, (dst & 0xF) < ((src & 0xF) + borrow));
            }
            break;
        }

        case kEoCmp: {
            if (std::strcmp(mnemonic, "cmps") == 0) {
                // CMPS X, [HL+byte]
                const u8 src = read_op(insn.ops[1]);
                const int difference = x - src;
                if ((flags & kRl78FlagZ) != 0) set_flag(kRl78FlagZ, static_cast<u8>(difference) == 0);
                if ((flags & kRl78FlagCy) != 0) set_flag(kRl78FlagCy, x == 0 || src == 0);
                if ((flags & kRl78FlagAc) != 0) set_flag(kRl78FlagAc, (x & 0xF) < (src & 0xF));
                break;
            }
            if (insn.is_word) {
                const u16 dst = read_op_word(insn.ops[0]);
                const u16 src = read_op_word(insn.ops[1]);
                const int difference = dst - src;
                if ((flags & kRl78FlagZ) != 0) {
                    set_flag(kRl78FlagZ, static_cast<u16>(difference) == 0);
                }
                if ((flags & kRl78FlagCy) != 0) set_flag(kRl78FlagCy, difference < 0);
                if ((flags & kRl78FlagAc) != 0) set_flag(kRl78FlagAc, (dst & 0xF) < (src & 0xF));
                break;
            }
            const u8 dst = read_op(insn.ops[0]);
            const u8 src = read_op(insn.ops[1]);
            const int difference = dst - src;
            if ((flags & kRl78FlagZ) != 0) {
                set_flag(kRl78FlagZ, static_cast<u8>(difference) == 0);
            }
            if ((flags & kRl78FlagCy) != 0) set_flag(kRl78FlagCy, difference < 0);
            if ((flags & kRl78FlagAc) != 0) set_flag(kRl78FlagAc, (dst & 0xF) < (src & 0xF));
            break;
        }

        case kEoLogic: {
            const u8 lhs = read_op(insn.ops[0]);
            const u8 rhs = read_op(insn.ops[1]);
            const u8 result = insn.id == Rl78Id::And ? static_cast<u8>(lhs & rhs)
                              : insn.id == Rl78Id::Or ? static_cast<u8>(lhs | rhs)
                                                      : static_cast<u8>(lhs ^ rhs);
            write_op(insn.ops[0], result);
            if ((flags & kRl78FlagZ) != 0) set_flag(kRl78FlagZ, result == 0);
            break;
        }

        case kEoMulu:
            mul_u();
            break;

        case kEoMulDiv: {
            // The 0x61/0xCE page multiplexes mul/div/mach on the immediate byte:
            // "mov 0xFFEFB, #n" is MULHU/MULH/DIVHU/DIVWU/MACHU/MACH (RL78/G14).
            if (insn.id == Rl78Id::Mulu) {
                mul_u();
                break;
            }
            if (insn.id != Rl78Id::Mov) {
                switch (insn.id) {
                    case Rl78Id::Mulhu: mul_hu(false); break;
                    case Rl78Id::Mulh: mul_hu(true); break;
                    case Rl78Id::Divhu: div_hu(); break;
                    case Rl78Id::Divwu: div_wu(); break;
                    case Rl78Id::Machu: mac_hu(false); break;
                    case Rl78Id::Mach: mac_hu(true); break;
                    default: break;
                }
                break;
            }
            if (insn.ops[0].addend != static_cast<s32>(rl78_sfr(0xFB))) {
                do_mov(insn);
                break;
            }
            switch (insn.ops[1].addend & 0xFF) {
                case 0x01: mul_hu(false); break;
                case 0x02: mul_hu(true); break;
                case 0x03: div_hu(); break;
                case 0x0B: div_wu(); break;
                case 0x05: mac_hu(false); break;
                case 0x06: mac_hu(true); break;
                default: do_mov(insn); break;
            }
            break;
        }

        case kEoShift: {
            int count = 1;
            if (insn.ops[1].type == Rl78OpType::Imm) count = insn.ops[1].addend & 0xFF;
            if (count == 0) break;

            bool carry = flag_cy();
            if (insn.is_word) {
                u16 value = read_op_word(insn.ops[0]);
                for (int i = 0; i < count; ++i) carry = shift_step<u16>(insn.id, value, carry);
                set_flag(kRl78FlagCy, carry);
                write_op_word(insn.ops[0], value);
                break;
            }
            u8 value = read_op(insn.ops[0]);
            for (int i = 0; i < count; ++i) carry = shift_step<u8>(insn.id, value, carry);
            set_flag(kRl78FlagCy, carry);
            write_op(insn.ops[0], value);
            break;
        }

        case kEoBranch: {
            u32 target;
            if (insn.ops[0].type == Rl78OpType::Reg && insn.ops[0].reg != Rl78Reg::None) {
                // BR AX / CALL AX: the page comes from CS.
                target = (static_cast<u32>(cs & 0x0F) << 16) | get_reg16(insn.ops[0].reg);
            } else {
                target = static_cast<u32>(insn.ops[0].addend) & 0xFFFFFu;
            }
            if (insn.id == Rl78Id::Call) {
                push(static_cast<u16>(pc + insn.length));
            }
            pc = target & 0xFFFFFu;
            pc_written_ = true;
            break;
        }

        case kEoCallt: {
            // CALLT [addr5]: the target is the word in the CALLT table.
            const u32 entry = insn.ops[0].address & 0xFFFFu;
            const u32 target = bus->read16(entry) & 0xFFFFu;
            push(static_cast<u16>(pc + insn.length));
            pc = target;
            pc_written_ = true;
            break;
        }

        case kEoBCond: {
            const u32 target = insn.ops[0].type == Rl78OpType::Imm
                                   ? static_cast<u32>(insn.ops[0].addend)
                                   : pc;
            switch (insn.ops[1].type) {
                case Rl78OpType::Bit:
                case Rl78OpType::BitInd: {
                    const bool value = read_bit(insn.ops[1]);
                    const bool take = insn.ops[1].condition == Rl78Cond::T ? value : !value;
                    if (take) {
                        pc = target & 0xFFFFFu;
                        pc_written_ = true;
                    }
                    break;
                }
                default: {
                    bool take;
                    switch (insn.ops[1].condition) {
                        case Rl78Cond::C: take = flag_cy(); break;
                        case Rl78Cond::NC: take = !flag_cy(); break;
                        case Rl78Cond::H: take = !flag_cy() && !flag_z(); break;
                        case Rl78Cond::NH: take = flag_cy() || flag_z(); break;
                        case Rl78Cond::Z: take = flag_z(); break;
                        case Rl78Cond::NZ: take = !flag_z(); break;
                        case Rl78Cond::T: take = true; break;
                        default: take = false; break;
                    }
                    if (take) {
                        pc = target & 0xFFFFFu;
                        pc_written_ = true;
                    }
                    break;
                }
            }
            break;
        }

        case kEoBtclr: {
            // BTCLR: branch when the bit is set, then clear it.
            if (read_bit(insn.ops[1])) {
                write_bit(insn.ops[1], false);
                pc = static_cast<u32>(insn.ops[0].addend) & 0xFFFFFu;
                pc_written_ = true;
            }
            break;
        }

        default:
            ZLB_LOG_WARN("cpu", "RL78 unhandled operation '%s' at 0x%05X", mnemonic, pc);
            halt(format("unhandled op %s", mnemonic));
            break;
    }
}

// ---------------------------------------------------------------------------
// Disassembly
// ---------------------------------------------------------------------------

std::string Rl78Core::disassemble(u32 address, unsigned& length) {
    return rl78_disassemble(*bus, address, length);
}

// ---------------------------------------------------------------------------
// Register interface
// ---------------------------------------------------------------------------

void Rl78Core::registers(std::vector<RegValue>& out) const {
    out.emplace_back("general", "PC", pc);
    out.emplace_back("general", "AX", static_cast<u64>((a << 8) | x));
    out.emplace_back("general", "BC", static_cast<u64>((b << 8) | c));
    out.emplace_back("general", "DE", static_cast<u64>((d << 8) | e));
    out.emplace_back("general", "HL", static_cast<u64>((h << 8) | l));
    out.emplace_back("general", "SP", sp);
    out.emplace_back("general", "PSW", psw,
                     std::string(flag_z() ? "Z " : "- ") + (flag_ac() ? "AC " : "- ") +
                         (flag_cy() ? "CY " : "- ") + (flag_ie() ? "IE " : "- ") + "bank" +
                         std::to_string(bank()));
    out.emplace_back("general", "ES", es);
    out.emplace_back("general", "CS", cs);
    out.emplace_back("general", "A", a);
    out.emplace_back("general", "X", x);
    out.emplace_back("general", "B", b);
    out.emplace_back("general", "C", c);
    out.emplace_back("general", "D", d);
    out.emplace_back("general", "E", e);
    out.emplace_back("general", "H", h);
    out.emplace_back("general", "L", l);
}

bool Rl78Core::set_register(const std::string& name, u64 value) {
    const std::string key = to_lower(name);
    if (key == "a") { a = static_cast<u8>(value); return true; }
    if (key == "x") { x = static_cast<u8>(value); return true; }
    if (key == "b") { b = static_cast<u8>(value); return true; }
    if (key == "c") { c = static_cast<u8>(value); return true; }
    if (key == "d") { d = static_cast<u8>(value); return true; }
    if (key == "e") { e = static_cast<u8>(value); return true; }
    if (key == "h") { h = static_cast<u8>(value); return true; }
    if (key == "l") { l = static_cast<u8>(value); return true; }
    if (key == "ax") { a = static_cast<u8>(value >> 8); x = static_cast<u8>(value); return true; }
    if (key == "bc") { b = static_cast<u8>(value >> 8); c = static_cast<u8>(value); return true; }
    if (key == "de") { d = static_cast<u8>(value >> 8); e = static_cast<u8>(value); return true; }
    if (key == "hl") { h = static_cast<u8>(value >> 8); l = static_cast<u8>(value); return true; }
    if (key == "sp") { sp = static_cast<u16>(value); return true; }
    if (key == "psw") { psw = static_cast<u16>(value); return true; }
    if (key == "es") { es = static_cast<u8>(value); return true; }
    if (key == "cs") { cs = static_cast<u8>(value); return true; }
    if (key == "pc") { pc = static_cast<u32>(value) & 0xFFFFFu; bus->context.pc = pc; return true; }
    return false;
}

bool Rl78Core::get_register(const std::string& name, u64& value) const {
    value = 0;
    const std::string key = to_lower(name);
    if (key == "a") { value = a; return true; }
    if (key == "x") { value = x; return true; }
    if (key == "b") { value = b; return true; }
    if (key == "c") { value = c; return true; }
    if (key == "d") { value = d; return true; }
    if (key == "e") { value = e; return true; }
    if (key == "h") { value = h; return true; }
    if (key == "l") { value = l; return true; }
    if (key == "ax") { value = static_cast<u64>((a << 8) | x); return true; }
    if (key == "bc") { value = static_cast<u64>((b << 8) | c); return true; }
    if (key == "de") { value = static_cast<u64>((d << 8) | e); return true; }
    if (key == "hl") { value = static_cast<u64>((h << 8) | l); return true; }
    if (key == "sp") { value = sp; return true; }
    if (key == "psw") { value = psw; return true; }
    if (key == "es") { value = es; return true; }
    if (key == "cs") { value = cs; return true; }
    if (key == "pc") { value = pc; return true; }
    return false;
}

std::string Rl78Core::status_line() const {
    return format("RL78 PC=%05X AX=%04X BC=%04X DE=%04X HL=%04X SP=%04X PSW=%02X [%c%c%c%c] "
                  "ES=%02X CS=%02X bank=%d",
                  pc, (a << 8) | x, (b << 8) | c, (d << 8) | e, (h << 8) | l, sp, psw,
                  flag_z() ? 'Z' : '-', flag_ac() ? 'A' : '-', flag_cy() ? 'C' : '-',
                  flag_ie() ? 'I' : '-', es, cs, bank());
}

void Rl78Core::describe_state(std::vector<std::string>& lines) const {
    lines.push_back(format("RL78 bank=%d (RB%d at 0x%05X) ISP=%d", bank(), bank(), bank_base(),
                           isp_level()));
    lines.push_back(format("RL78 flags: Z=%d AC=%d CY=%d IE=%d", flag_z() ? 1 : 0,
                           flag_ac() ? 1 : 0, flag_cy() ? 1 : 0, flag_ie() ? 1 : 0));
    lines.push_back(format("RL78 reset vector 0x%05X, data accesses %llu, unknown %llu",
                           reset_vector, static_cast<unsigned long long>(data_accesses),
                           static_cast<unsigned long long>(unknown_instructions)));
    if (pending_vector >= 0) {
        lines.push_back(format("RL78 pending interrupt vector %d (IE=%d)", pending_vector,
                               flag_ie() ? 1 : 0));
    }
}

// ---------------------------------------------------------------------------
// Save states
// ---------------------------------------------------------------------------

void Rl78Core::save_state(StateWriter& writer) const {
    Cpu::save_state(writer);

    writer.put_u8(a);
    writer.put_u8(x);
    writer.put_u8(b);
    writer.put_u8(c);
    writer.put_u8(d);
    writer.put_u8(e);
    writer.put_u8(h);
    writer.put_u8(l);
    writer.put_u16(sp);
    writer.put_u16(psw);
    writer.put_u8(es);
    writer.put_u8(cs);
    writer.put_u8(pmc);
    writer.put_u32(reset_vector);
    writer.put_u64(data_accesses);
    writer.put_u64(unknown_instructions);
    writer.put_i32(pending_vector);
    writer.put_u32(page_);
    writer.put_bool(pc_written_);
    writer.fixed(irq_levels_, [&](bool level) { writer.put_bool(level); });
    writer.fixed(irq_vectors_, [&](u8 vector) { writer.put_u8(vector); });
}

void Rl78Core::load_state(StateReader& reader) {
    Cpu::load_state(reader);

    a = reader.get_u8();
    x = reader.get_u8();
    b = reader.get_u8();
    c = reader.get_u8();
    d = reader.get_u8();
    e = reader.get_u8();
    h = reader.get_u8();
    l = reader.get_u8();
    sp = reader.get_u16();
    psw = reader.get_u16();
    es = reader.get_u8();
    cs = reader.get_u8();
    pmc = reader.get_u8();
    reset_vector = reader.get_u32();
    data_accesses = reader.get_u64();
    unknown_instructions = reader.get_u64();
    pending_vector = reader.get_i32();
    page_ = reader.get_u32();
    pc_written_ = reader.get_bool();
    reader.fixed(irq_levels_, [&](bool& level) { level = reader.get_bool(); });
    reader.fixed(irq_vectors_, [&](u8& vector) { vector = reader.get_u8(); });
}

}  // namespace zlb

namespace zlb {

std::unique_ptr<Cpu> create_rl78_core(Bus& bus) { return std::make_unique<Rl78Core>(bus); }

}  // namespace zlb
