// zeliboba - RL78 disassembler.
//
// Like binutils' rl78-dis.c this does not guess: the row's SYNTAX() template
// (taken verbatim from rl78-decode.opc) decides which operands are printed, in
// which order and with which modifiers:
//
//   %0 %1     operand 0 / operand 1
//   %e        print "es:" when the instruction was ES:-prefixed
//   %!        print "!" (direct address)
//   %a        print as a branch/call target
//   %s        print SFR names (sp, psw, spl, sph, cs, es, pmc, mem)
//   %c        print the condition of the source operand (sk%c1)
//   %x        hex (parsed for compatibility; see rl78_disasm.cpp)
//
// This is why "bt %s1, $%a0" prints the bit operand first and the branch target
// second, and why "clrb %e!0" does not print the implicit constant operand.
#pragma once

#include <string>

#include "bus/bus.h"
#include "common/types.h"
#include "cpu/rl78/rl78_decode.h"

namespace zlb {

/// Disassemble the instruction at `address`; `length` receives its byte length
/// (1 for an unknown encoding).
std::string rl78_disassemble(Bus& bus, u32 address, unsigned& length);

/// Render an already decoded instruction.
std::string rl78_format_instruction(const Rl78Decoded& insn);

/// Named SFR printed by the %s modifier, or nullptr.  `word` selects the
/// 16-bit spelling ("sp" vs "spl").
const char* rl78_sfr_name(u32 address, bool word);

}  // namespace zlb
