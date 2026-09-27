// zeliboba - Renesas RL78 instruction set model.
//
// ENCODING SOURCE OF TRUTH
// ------------------------
// `rl78_isa.cpp` is a mechanical transcription of GNU binutils' RL78 decoder
// (opcodes/rl78-decode.opc via the rl78-decode.c it generates) and of its
// reference printer (opcodes/rl78-dis.c).  The C# reference core in
// VitaTestSuite/Core ships the same table (Rl78IsaTable.g.cs, 312 rows and 624
// operand descriptors); the C++ table keeps the identical row/descriptor model
// so the two can be diffed row by row.
//
// OPERAND MODEL
// -------------
// Like binutils, a decoded instruction carries exactly two operand slots,
// op[0] = destination and op[1] = source, as built by the .opc macros
// (DR/DRB/DRW, SR/SRB/SRW, DM/SM/DM2/SM2, DC/SC, DB/SB, COND, DPUSH/SPOP).  An
// operand is one of:
//
//   Imm      an immediate value (immediate, shift count, bank number, target)
//   Reg      a register (X A C B E D L H AX BC DE HL SP PSW CS ES)
//   Ind      [reg (+reg2) (+addend)] or a direct/short-direct/SFR address
//   Bit      reg.bit
//   BitInd   [reg (+addend)].bit / addr.bit
//   PreDec   [--SP]   (PUSH)
//   PostInc  [SP++]   (POP)
//
// Register numbers match binutils' RL78_Register enumeration, which is also the
// only place where the raw opcode register fields differ from the ISA register
// order: the 3-bit field value n selects register n + RL78_Reg_X.
//
// Instruction length is "opcode bytes + the operand data bytes", read from the
// byte offset the generator recorded per operand slot.  That offset follows the
// order in which the .opc macros consume bytes, *not* the slot order: for
// "bt %s1, $%a0" the short-direct byte comes before the displacement even
// though the destination slot is the branch target.
#pragma once

#include "common/types.h"

namespace zlb {

// ---------------------------------------------------------------------------
// Basic enumerations
// ---------------------------------------------------------------------------

/// Operand kind, mirroring binutils' RL78_Operand_Type.
enum class Rl78OpType : u8 {
    None = 0,
    Imm = 1,
    Reg = 2,
    Ind = 3,
    Bit = 4,
    BitInd = 5,
    PreDec = 6,
    PostInc = 7,
};

/// RL78 registers, numbered exactly like binutils' RL78_Register.
enum class Rl78Reg : u8 {
    None = 0,
    // 8-bit registers: the opcode field value is (reg - X).
    X = 1,
    A = 2,
    C = 3,
    B = 4,
    E = 5,
    D = 6,
    L = 7,
    H = 8,
    // 16-bit register pairs: the opcode field value is (reg - AX).
    AX = 9,
    BC = 10,
    DE = 11,
    HL = 12,
    // control registers (unordered in binutils)
    SP = 13,
    PSW = 14,
    CS = 15,
    ES = 16,
    PMC = 17,
    MEM = 18,
};

/// RL78 condition codes, encoded in the low 3 bits of the branch field.
enum class Rl78Cond : u8 {
    T = 0,
    F = 1,
    C = 2,
    NC = 3,
    H = 4,
    NH = 5,
    Z = 6,
    NZ = 7,
};

/// Semantics id, mirroring binutils' RL78_Opcode_ID.  The mnemonic alone is not
/// enough: "mov" covers MOV, MOVS, SET1/CLR1/MOV1, PUSH/POP and the G14
/// mul/div multiplex.
enum class Rl78Id : u8 {
    Unknown = 0,
    Add = 1,
    Addc = 2,
    And = 3,
    Branch = 4,
    BranchCond = 5,
    BranchCondClear = 6,
    Break = 7,
    Call = 8,
    Cmp = 9,
    Divhu = 10,
    Divwu = 11,
    Halt = 12,
    Mov = 13,
    Mach = 14,
    Machu = 15,
    Mulu = 16,
    Mulh = 17,
    Mulhu = 18,
    Nop = 19,
    Or = 20,
    Ret = 21,
    Reti = 22,
    Rol = 23,
    Rolc = 24,
    Ror = 25,
    Rorc = 26,
    Sar = 27,
    Sel = 28,
    Shr = 29,
    Shl = 30,
    Skip = 31,
    Stop = 32,
    Sub = 33,
    Subc = 34,
    Xch = 35,
    Xor = 36,
};

/// How a register operand gets its register.
enum : u8 {
    kRl78RmNone = 0,
    kRl78RmFixed = 1,    // rf holds the register number
    kRl78RmField8 = 2,   // Reg   = X  + ((switch >> rs) & rx)
    kRl78RmField16 = 3,  // Reg   = AX + ((switch >> rs) & rx)
};

/// How an operand's addend was formed.
enum : u8 {
    kRl78AkNone = 0,
    kRl78AkLit = 1,     // rp is the literal (shift count, bank number)
    kRl78AkImmu1 = 2,   // 1-byte unsigned immediate
    kRl78AkImmu2 = 3,   // 2-byte unsigned immediate / !addr16
    kRl78AkImmu3 = 4,   // 3-byte unsigned immediate (20-bit call/br target)
    kRl78AkImms1 = 5,   // 1-byte signed displacement
    kRl78AkImms2 = 6,   // 2-byte signed displacement
    kRl78AkSfr = 7,     // 1-byte SFR number -> 0xFFF00 + n
    kRl78AkSaddr = 8,   // 1-byte short direct -> 0xFFF00/0xFFE00 + n
    kRl78AkRel8 = 9,    // pc + length + signed byte
    kRl78AkRel16 = 10,  // pc + length + signed word
    kRl78AkCallt = 11,  // 0x80 + mm * 16 + nnn * 2
    kRl78AkField = 12,  // (switch >> rs) & rx
};

/// Bit selector of a Bit / BitInd operand.
enum : u8 {
    kRl78BsNone = 0xFF,   // the operand has no bit number
    kRl78BsFixed = 0xFE,  // bf holds the fixed bit number
};

/// PSW bits (binutils' Fz/Fc/... masks).  ISP0/ISP1 sit in bits 2:1, RBS0 in
/// bit 3 and RBS1 in bit 5; the two halves of the bank select are not adjacent.
enum : u8 {
    kRl78FlagCy = 0x01,    // PSW.0 carry
    kRl78FlagIsp0 = 0x02,  // PSW.1 interrupt stack pointer bit 0
    kRl78FlagIsp1 = 0x04,  // PSW.2 interrupt stack pointer bit 1
    kRl78FlagRbs0 = 0x08,  // PSW.3 register bank select bit 0
    kRl78FlagAc = 0x10,    // PSW.4 auxiliary carry
    kRl78FlagRbs1 = 0x20,  // PSW.5 register bank select bit 1
    kRl78FlagZ = 0x40,     // PSW.6 zero
    kRl78FlagIe = 0x80,    // PSW.7 interrupt enable
};

/// Control register SFR addresses (RL78 user's manual, table 3-4).
enum : u32 {
    kRl78SfrSp = 0xFFFF8,
    kRl78SfrPsw = 0xFFFFA,
    kRl78SfrCs = 0xFFFFC,
    kRl78SfrEs = 0xFFFFD,
    kRl78SfrPmc = 0xFFFFE,
    kRl78SfrMem = 0xFFFFF,
};

/// Register bank area: 0xFFEE0 + 8 * bank.
enum : u32 { kRl78BankBase = 0xFFEE0, kRl78BankStride = 8 };

// ---------------------------------------------------------------------------
// Table rows
// ---------------------------------------------------------------------------

/// One row of the ISA table.
struct Rl78Row {
    u16 value;       ///< fixed bits of the switch byte
    u8 prefix;       ///< first opcode byte for a two-byte page, else 0
    u8 q;            ///< 1 = one-byte opcode, 2 = prefix + switch byte
    u8 var;          ///< bits of the switch byte the row leaves variable
    u8 switchmask;   ///< the page's switch mask on the switch byte
    u8 length;       ///< total length, excluding a leading 0x11 ES prefix
    u8 mnemonic;     ///< kRl78Mnemonics[] index
    u8 syntax;       ///< kRl78Syntaxes[] index
    u8 id;           ///< Rl78Id
    u8 flags;        ///< PSW flag mask the instruction writes
    u8 is_word;      ///< binutils' W()
    u8 operands;     ///< operand descriptor count (0..2)
};

/// One operand descriptor; indexed by 2 * row + slot.
struct Rl78Desc {
    u8 type;  ///< Rl78OpType
    u8 rm;    ///< register mode (kRl78Rm*)
    u8 rf;    ///< fixed register / base register of [reg+reg2]
    u8 rs;    ///< register or bit field shift
    u8 rx;    ///< register or bit field mask
    u8 ra;    ///< addend kind (kRl78Ak*)
    u8 rp;    ///< addend parameter (literal, byte count or relative base)
    u8 bs;    ///< bit selector (kRl78Bs*)
    u8 bx;    ///< bit field mask
    u8 bf;    ///< fixed bit number
    u8 cn;    ///< Rl78Cond
    u8 r2;    ///< index register of [reg+reg2]
    u8 off;   ///< byte offset of this operand's data inside the instruction
};

constexpr int kRl78RowCount = 312;
constexpr int kRl78MnemonicCount = 70;
constexpr int kRl78SyntaxCount = 190;

extern const Rl78Row kRl78Rows[kRl78RowCount];
extern const Rl78Desc kRl78Descs[2 * kRl78RowCount];
extern const char* const kRl78Mnemonics[kRl78MnemonicCount];
extern const char* const kRl78Syntaxes[kRl78SyntaxCount];

/// Fixed/variable bit mask of a row's opcode.
///
/// A one-byte page masks the single opcode byte.  A two-byte page combines the
/// fully fixed prefix byte with the *page switch mask*: binutils generates
/// `switch (op[1] & switchmask)` per page and every row then only fixes the bits
/// its own pattern spells out.
///
/// The 0x31 page is the one that matters.  `rl78-decode.c` emits
/// `case 0x31: GETBYTE(); switch (op[1] & 0x8f)` and each nested case fixes the
/// low nibble only, so the mask must be `0xFF00 | (switchmask & ~var)`.  With the
/// tempting `0xFFFF & ~var` instead, a row like `0011 0001 0bit 0010` (var == 0)
/// would be pinned to the single encoding `0x3102` and `bt 0xFFFF0.3, $target`
/// would not decode at all; this was defect #4 of the RL78 rework handoff.
constexpr u16 rl78_row_mask(const Rl78Row& row) {
    if (row.q == 2) {
        return static_cast<u16>(0xFF00u | (row.switchmask & static_cast<u8>(~row.var)));
    }
    return static_cast<u16>(0xFFu & static_cast<u8>(~row.var));
}

// ---------------------------------------------------------------------------
// Small helpers over the model
// ---------------------------------------------------------------------------

/// Address of SFR number n (binutils' sfr()).
constexpr u32 rl78_sfr(int n) { return 0xFFF00u + static_cast<u32>(n); }

/// Short-direct byte address: 0xFFF00 + n for n < 0x20, else 0xFFE00 + n.
constexpr u32 rl78_short_direct(int n) {
    return n < 0x20 ? 0xFFF00u + static_cast<u32>(n) : 0xFFE00u + static_cast<u32>(n);
}

/// Lower-case register name, as printed by rl78-dis.c.
const char* rl78_reg_name(Rl78Reg reg);

/// True for the four 16-bit register pairs.
constexpr bool rl78_is_word_reg(Rl78Reg reg) {
    return reg == Rl78Reg::AX || reg == Rl78Reg::BC || reg == Rl78Reg::DE || reg == Rl78Reg::HL;
}

}  // namespace zlb
