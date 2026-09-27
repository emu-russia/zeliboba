// zeliboba - standalone MeP-c5 disassembler.
//
// Same decoder and formatter as the interpreter; exposed separately so the
// debugger and the `zdis` tool can disassemble a MeP image without constructing
// a core.  Output matches the annotated boot ROM listings produced by
// objdump (mep-elf binutils 2.37).
#pragma once

#include <string>

#include "bus/bus.h"
#include "common/types.h"

namespace zlb {

/// Disassemble one instruction at `address`, fetching through `bus` so that the
/// access shows up in the trace.  `length` receives the instruction size in
/// bytes (2 or 4).  Undecodable words render as "*unknown*" and report 4 bytes,
/// which is what objdump does.
std::string mep_disassemble(Bus& bus, u32 address, unsigned& length);

}  // namespace zlb
