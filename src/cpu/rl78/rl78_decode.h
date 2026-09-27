// zeliboba - RL78 instruction decoder.
//
// Decodes one instruction into a table row (rl78_isa.h) plus the operand slots
// op[0] (destination) and op[1] (source), exactly as binutils' rl78-decode.c
// does.  Every row stores the fixed bits of its opcode, so the lookup is "read
// the switch byte, then test each candidate row with (byte & mask) == value";
// for a two-byte opcode the whole 16-bit word (prefix byte + switch byte) is
// matched and the mask is the page switch mask combined with the row's own
// variable bits (see rl78_row_mask()).
//
// Operand data is read at the offset the table records per slot, which is the
// order in which the .opc macros consume bytes - *not* the slot order: for
// "bt %s1, $%a0" the short-direct byte comes before the displacement even
// though the destination slot is the branch target.
//
// Instruction fetches go through Bus::fetch8(), so the access trace attributes
// them to the core (bus.context) like any other fetch.
#pragma once

#include <string>

#include "bus/bus.h"
#include "common/types.h"
#include "cpu/rl78/rl78_isa.h"

namespace zlb {

/// One decoded operand.
struct Rl78Operand {
    Rl78OpType type = Rl78OpType::None;
    /// Register, or the base register of [reg+reg2+addend].
    Rl78Reg reg = Rl78Reg::None;
    /// Index register of [HL+B] / [HL+C], else None.
    Rl78Reg reg2 = Rl78Reg::None;
    /// Raw addend: immediate value, byte displacement or absolute address.
    s32 addend = 0;
    /// How `addend` was formed (kRl78Ak*).
    u8 add_kind = kRl78AkNone;
    /// Effective address for operands the decoder resolves itself (SFR, saddr,
    /// direct, CALLT and the pc-relative branch targets).
    u32 address = 0;
    /// Bit number of a Bit / BitInd operand.
    u8 bit = 0;
    /// Condition carried by the source operand of a branch / skip.
    Rl78Cond condition = Rl78Cond::T;
    /// Decoded while an 0x11 ES: prefix was active.
    bool es = false;
    /// Byte offset of this operand's data inside the instruction.
    u8 offset = 0;
    /// Bytes consumed by this operand's data (0, 1, 2 or 3).
    u8 size = 0;
};

/// A fully decoded instruction.
struct Rl78Decoded {
    /// Table row index, or -1 when the encoding is unknown.
    int row = -1;
    /// Total length in bytes, including an optional 0x11 ES prefix.
    unsigned length = 0;
    /// True when the instruction was prefixed with 0x11 (ES:).
    bool has_es_prefix = false;
    Rl78Id id = Rl78Id::Unknown;
    /// PSW flag mask written by the instruction.
    u8 flags = 0;
    /// True for word operations (binutils' W()).
    bool is_word = false;
    /// SYNTAX() template of the row; drives operand printing.
    const char* syntax = "";
    const char* mnemonic = "";
    Rl78Operand ops[2];

    const Rl78Operand& operand(int index) const { return ops[index & 1]; }
};

/// Longest encoding in the table, plus the ES prefix.
constexpr unsigned kRl78MaxInstructionLength = 7;

/// Decode the instruction at `address`.  Returns false when no row matches.
bool rl78_decode(Bus& bus, u32 address, Rl78Decoded& out);

/// Instruction length in bytes; 1 when the encoding is unknown.
unsigned rl78_instruction_length(Bus& bus, u32 address);

/// True when the byte introduces a two-byte opcode (0x31, 0x61, 0x71).
bool rl78_byte_is_prefix(u8 byte);

/// Row index selected by an opcode, or -1.  `op1` is only consulted for the
/// two-byte pages; diagnostics and tests use this to check the tie-breaks.
int rl78_find_row(u8 op0, u8 op1);

/// Number of table rows (diagnostics).
int rl78_row_count();

}  // namespace zlb
