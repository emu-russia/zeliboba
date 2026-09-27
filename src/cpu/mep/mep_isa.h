// zeliboba - Toshiba MeP-c5 instruction encoding tables and decode helpers.
//
// The MeP-c5 is a variable length little endian ISA: the major opcode (bits
// 12..15 of the first halfword) decides whether an instruction is 16 or 32 bits
// wide, and no 16 bit instruction can be confused with the low half of a 32 bit
// one.  The tables below are the CGEN derived encodings from binutils 2.37
// (cpu/mep-core.cpu + cpu/mep-c5.cpu); `MePCore` executes them and
// `mep_disasm.cpp` prints them.
//
// Everything here is pure decoding - no bus access, no state - so the
// disassembler, the interpreter and the self tests share one source of truth.
#pragma once

#include <array>
#include <cstddef>
#include <string>

#include "common/types.h"

namespace zlb::mep {

/// One entry per distinct instruction encoding in the CGEN opcode table.
enum class Op : u16 {
    None = 0,
    Sb, Sh, Sw, Lb, Lh, Lw, Lbu, Lhu,
    SwSp, LwSp,
    SbTp, ShTp, SwTp, LbTp, LhTp, LwTp, LbuTp, LhuTp,
    Sb16, Sh16, Sw16, Lb16, Lh16, Lw16, Lbu16, Lhu16,
    Sw24, Lw24,
    Extb, Exth, Extub, Extuh, Ssarb,
    Mov, Movi8, Movi16, Movu24, Movu16, Movh,
    Add3, Add, Add3i, Advck3, Sub, Sbvck3, Neg,
    Slt3, Sltu3, Slt3i, Sltu3i, Sl1ad3, Sl2ad3, Add3x, Slt3x, Sltu3x,
    Or, And, Xor, Nor, Or3, And3, Xor3,
    Sra, Srl, Sll, Srai, Srli, Slli, Sll3, Fsft,
    Bra, Beqz, Bnez, Beqi, Bnei, Blti, Bgei, Beq, Bne,
    Bsr12, Bsr24, Jmp, Jmp24, Jsr, Ret, Repeat, Erepeat,
    StcLp, StcHi, StcLo, Stc, LdcLp, LdcHi, LdcLo, Ldc,
    Di, Ei, Reti, Halt, Sleep, Swi, Break, Syncm,
    Stcb, Ldcb, Bsetm, Bclrm, Bnotm, Btstm, Tas, Cache,
    Mul, Mulu, Mulr, Mulru, Madd, Maddu, Maddr, Maddru, Div, Divu,
    Dret, Dbreak, Ldz, Abs, Ave, Min, Max, Minu, Maxu, Clip, Clipu,
    Sadd, Ssub, Saddu, Ssubu,
    Swcp, Lwcp, Smcp, Lmcp, Swcpi, Lwcpi, Smcpi, Lmcpi,
    Swcp16, Lwcp16, Smcp16, Lmcp16,
    Sbcpa, Lbcpa, Shcpa, Lhcpa, Swcpa, Lwcpa, Smcpa, Lmcpa,
    Sbcpm0, Lbcpm0, Shcpm0, Lhcpm0, Swcpm0, Lwcpm0, Smcpm0, Lmcpm0,
    Sbcpm1, Lbcpm1, Shcpm1, Lhcpm1, Swcpm1, Lwcpm1, Smcpm1, Lmcpm1,
    Bcpeq, Bcpne, Bcpat, Bcpaf, Synccp, Jsrv, Bsrv, SimSyscall,
    Ri0, Ri1, Ri2, Ri3, Ri4, Ri5, Ri6, Ri7, Ri8, Ri9, Ri10, Ri11, Ri12, Ri13, Ri14,
    Ri15, Ri17, Ri20, Ri21, Ri22, Ri23, Ri26,
    StcbR, LdcbR, Pref, Prefd,
    Casb3, Cash3, Casw3,
    Sbcp, Lbcp, Lbucp, Shcp, Lhcp, Lhucp,
    Lbucpa, Lhucpa, Lbucpm0, Lhucpm0, Lbucpm1, Lhucpm1,
    Uci, Dsp, Dsp0, Dsp1,
};

/// Named bit fields.  `start`/`length` follow the CGEN convention: `start` is
/// the bit index of the *last* bit of the field counted from the MSB of the
/// 32 bit instruction word, so the byte swapped halfwords of the MeP are
/// handled transparently by `raw()`.
enum class Field : u8 {
    Const = 0,
    F12s20, F12s4a2, F16s16, F16u16, F17s16a2, F24s5a2n, F24u4n, F24u5a2n,
    F24u8a4n, F24u8n, F2u10, F2u6, F3u5, F4u8, F5u24, F5u8, F6s8, F7u9,
    F7u9a2, F7u9a4, F8s8, F8s8a2, FC5Rm, FC5Rnm, FCallnum, FCcrn, FCdisp10,
    FCrn, FCrnx, FCsrn, FRl5, Rl, Rm, Rn, Rn3,
};

/// How an operand is rendered by the formatter.
enum class Print : u8 { Reg, CpReg, Csrn, UHex, SDec, Label, Cdisp10 };

/// Description of one operand of one instruction.
struct Opnd {
    Field field = Field::Const;
    u16 constant = 0;
    Print print = Print::UHex;

    /// True for the branches/jumps whose `label` operand is a computed address
    /// rather than a plain immediate.
    constexpr bool is_label() const { return print == Print::Label; }
};

/// A single encoding: `(word & mask) == value`, `len` bytes wide.
struct Insn {
    const char* mnem = "";
    const char* fmt = "";
    u32 mask = 0;
    u32 value = 0;
    u8 len = 2;
    Op op = Op::None;
    std::array<Opnd, 3> ops{};
    u8 op_count = 0;

    bool matches(u32 word) const { return (word & mask) == value; }
};

/// Instruction length in bytes; only major opcode 15 has 32 bit forms.
inline unsigned length_of(u32 word) { return ((word >> 12) & 0xF) >= 12 ? 4u : 2u; }

// ---------------------------------------------------------------------------
// Bit field extraction
// ---------------------------------------------------------------------------

/// Extract `length` bits starting at CGEN bit position `start` of a 32 bit word.
/// CGEN numbers the first halfword (which the MeP fetches first) from the top,
/// so each 16 bit half of `word` holds its fields with local MSB-first order.
u32 raw(u32 word, int start, int length);

/// Sign extend the low `bits` bits of `value`.
u32 sext(u32 value, int bits);

/// Value of a named field, including PC relative branch targets and the
/// scaled displacement forms.
u32 field_value(Field field, u32 word, u32 pc);

/// Decode a 32 bit little endian instruction word.  Never returns nullptr.
const Insn* decode(u32 word);

/// objdump style mnemonic plus operands, exactly as the annotated boot ROM
/// listings print it.
std::string format(const Insn& insn, u32 word, u32 pc);

/// Register name for index 0..15 ("$0".."$12", "$tp", "$gp", "$sp").
const char* register_name(int index);
/// Control register name for the `stc`/`ldc` encoding space ("$hi", ...).
const char* control_register_name(int index);

/// True when the encoding is one of the "--reserved--" filler patterns that
/// CGEN knows about but the core must treat as undefined.
inline bool is_reserved(Op op) {
    return op >= Op::Ri0 && op <= Op::Ri26;
}

}  // namespace zlb::mep
