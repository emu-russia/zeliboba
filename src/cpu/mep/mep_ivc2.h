// zeliboba - MeP IVC2 coprocessor disassembler (Venezia's VLIW slots).
//
// The IVC2 is the VLIW coprocessor of the MeP core used by the PlayStation
// Vita's "Venezia" media engine.  It adds 688 instruction encodings that live
// in four slots:
//
//   V1  [---- core, 16 bit ----][ p0s ][-------- p1 --------]
//   V2  [------- core, 32 bit -------]xxxx[------ p1 -------]
//   V3  1111[--p0--]0111[------- p0 -------][------ p1 -------]
//
// where each `p*` is a 32 (or, for p0s, 16) bit coprocessor instruction word.
// That split, the slot selection and the "A + B" rendering follow binutils'
// opcodes/mep-dis.c (mep_examine_ivc2_insns / print_slot_insn), which is the
// reference implementation of this format.
//
// The slot word is built from the packet's bytes exactly the way CGEN builds
// it: for the little endian MeP the byte at file offset i of a 16 bit unit
// carries instruction bits 8*i..8*i+7, so e.g. the p0s word is
//   buf[2] | buf[3] << 8 | buf[4] << 16 | buf[5] << 24
// and `mep::raw()` extracts the fields from that value.
//
// The encoder table is generated from the CGEN description in
// MeP/mep-ivc2.cpu (see mep_ivc2_table.inc); `decode_slot()` is then plain
// mask/match, most specific mask first.
#pragma once

#include <cstddef>
#include <string>

#include "common/types.h"

namespace zlb::mep {

/// The four IVC2 slots.  P0S is the 16 bit co-slot of a V1 packet, P0 the 32
/// bit leading slot of a V3 packet and P1 the trailing 32 bit slot.
enum class Ivc2Slot : u8 { C3 = 0, P0S, P0, P1 };

/// Register class of a decoded operand, used by the formatter.
enum class Ivc2Reg : u8 { None = 0, Reg, Cp64, Cp32, Ivc2Ccr, Csr };

/// How an operand is printed.  The register forms imply the register class.
enum class Ivc2Print : u8 { Imm = 0, SImm, Reg, Cp64, Cp32, Ivc2Ccr, Csr, Name };

/// One (possibly split) field of an operand.  `shift` is the left shift the
/// CGEN multi-ifield's extract expression applies, `start`/`length` follow the
/// same MSB-first convention as `mep::Field`.
struct Ivc2Part {
    u8 start = 0;
    u8 length = 0;
    u8 shift = 0;
    bool sign = false;
};

/// One operand of one instruction.
struct Ivc2Op {
    const char* name = "";           ///< spec name, also used for Ivc2Print::Name
    Ivc2Print print = Ivc2Print::Imm;
    Ivc2Part parts[4] = {};
    u8 part_count = 0;
};

struct Ivc2Insn {
    const char* mnem = "";
    const char* fmt = "";
    u32 mask = 0;
    u32 value = 0;
    Ivc2Slot slot = Ivc2Slot::C3;
    u8 op_count = 0;
    Ivc2Op ops[6] = {};
};

/// Split a 64 bit VLIW packet (little endian fetch order) into its slots.
struct Ivc2Packet {
    /// True when the packet actually holds IVC2 slots.
    bool valid = false;
    bool has_core = true;       ///< a core instruction precedes the slots
    unsigned core_length = 0;   ///< 2 or 4 bytes
    /// Slot words, in packet order.  `slot` is empty for unused slots.
    struct Piece {
        Ivc2Slot slot = Ivc2Slot::P1;
        u32 word = 0;
    };
    Piece pieces[2];
    unsigned piece_count = 0;
};

/// Decide how a 64 bit packet splits into core and coprocessor slots.
Ivc2Packet ivc2_split_packet(const u8 bytes[8]);

/// Decode one slot word; nullptr when the encoding is unknown.
const Ivc2Insn* ivc2_decode_slot(Ivc2Slot slot, u32 word);

/// Render one decoded instruction ("mnemonic op,op"), objdump style.  `pc` is
/// the address of the slot instruction itself, used for its operands only.
std::string ivc2_format(const Ivc2Insn& insn, u32 word, u32 pc);

/// Formatter for one slot of a packet, including the "*unknown-<slot>*"
/// placeholder objdump prints.
std::string ivc2_disassemble_slot(Ivc2Slot slot, u32 word, u32 pc);

/// True when a packet at `bytes` is an IVC2 packet (binutils' heuristic: the
/// high nibble of the first byte is below 0xc0).
bool ivc2_packet_present(const u8 bytes[8]);

}  // namespace zlb::mep
