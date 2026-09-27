// zeliboba - ARM Cortex-A9 disassembler for A32, Thumb (T16) and Thumb-2 (T32).
//
// The disassembler is deliberately independent of the core's execution state: it
// reads instruction bytes through the Bus and prints branch targets as absolute
// addresses, so the debugger listing looks exactly like a linker map:
//
//     bl 0x81003a8a
//     ldr r0, [pc, #0x10]  ; 0x81000b9c
#pragma once

#include <string>

#include "bus/bus.h"
#include "common/types.h"

namespace zlb {

/// Disassemble one instruction at `address`. `length` receives 2 or 4 (bytes).
/// Never throws; unknown encodings come back as `.word 0x...` / `.hword 0x...`.
std::string arm_disassemble(Bus& bus, u32 address, bool thumb, unsigned& length);

/// Mnemonic only (the text before the first space).
std::string arm_mnemonic(Bus& bus, u32 address, bool thumb);

}  // namespace zlb
