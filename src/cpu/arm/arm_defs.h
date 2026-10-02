// zeliboba - ARM Cortex-A9 (ARMv7-A) shared definitions.
//
// The constants in this header are shared by the interpreter (arm_core.cpp),
// the disassembler (arm_disasm.cpp), the MMU (arm_mmu.cpp) and the VFP unit
// (arm_vfp.cpp). They follow the ARM Architecture Reference Manual
// ARMv7-A/ARMv7-R (DDI0406C) naming so that the port of the C# reference core
// stays readable next to it.
#pragma once

#include <string>

#include "common/types.h"

namespace zlb {
namespace arm {

// ---------------------------------------------------------------------------
// CPSR / SPSR
// ---------------------------------------------------------------------------

constexpr u32 kFlagN = 1u << 31;
constexpr u32 kFlagZ = 1u << 30;
constexpr u32 kFlagC = 1u << 29;
constexpr u32 kFlagV = 1u << 28;
constexpr u32 kFlagQ = 1u << 27;
constexpr u32 kFlagJ = 1u << 24;
constexpr u32 kFlagGE = 0x000F0000u;

constexpr u32 kFlagE = 1u << 9;   // CPSR.E - data endianness
constexpr u32 kFlagA = 1u << 8;   // asynchronous abort mask
constexpr u32 kFlagI = 1u << 7;   // IRQ mask
constexpr u32 kFlagF = 1u << 6;   // FIQ mask
constexpr u32 kFlagT = 1u << 5;   // Thumb state
// CPSR has no Security-state bit: SCR.NS and Monitor mode determine the world.
constexpr u32 kModeMask = 0x1Fu;

// ---------------------------------------------------------------------------
// TrustZone (Security Extensions)
// ---------------------------------------------------------------------------

/// SCR (Secure Configuration Register, CP15 c1 c1 0) bit 0 - NS.
constexpr u32 kScrNs = 1u << 0;
constexpr u32 kScrIrq = 1u << 1;
constexpr u32 kScrFiq = 1u << 2;
constexpr u32 kScrEa = 1u << 3;
constexpr u32 kScrFw = 1u << 4;
constexpr u32 kScrAw = 1u << 5;

/// Monitor mode has its own vector table at MVBAR; SMC enters it at offset 8
/// (the remaining entries are aborts/IRQ/FIQ taken while already in monitor).
constexpr u32 kVecMonitorSmc = 0x08;

// ---------------------------------------------------------------------------
// Processor modes
// ---------------------------------------------------------------------------

constexpr u32 kModeUser = 0x10;
constexpr u32 kModeFiq = 0x11;
constexpr u32 kModeIrq = 0x12;
constexpr u32 kModeSupervisor = 0x13;
constexpr u32 kModeMonitor = 0x16;
constexpr u32 kModeAbort = 0x17;
constexpr u32 kModeHyp = 0x1A;
constexpr u32 kModeUndefined = 0x1B;
constexpr u32 kModeSystem = 0x1F;

/// Number of slots in the banked register backing store (indexed by mode field).
constexpr int kModeCount = 32;

/// CPSR value used out of reset: SVC mode, ARM state, A/I/F masked.
constexpr u32 kResetCpsr = 0x1D3u;

// ---------------------------------------------------------------------------
// Exception vectors (offsets from VBAR / the high or low vector base)
// ---------------------------------------------------------------------------

constexpr u32 kVecReset = 0x00;
constexpr u32 kVecUndefined = 0x04;
constexpr u32 kVecSupervisor = 0x08;
constexpr u32 kVecPrefetchAbort = 0x0C;
constexpr u32 kVecDataAbort = 0x10;
constexpr u32 kVecHyp = 0x14;
constexpr u32 kVecIrq = 0x18;
constexpr u32 kVecFiq = 0x1C;

// ---------------------------------------------------------------------------
// Fault bookkeeping
// ---------------------------------------------------------------------------

enum class FaultKind : int {
    None = 0,
    Prefetch = 1,
    Data = 2,
};

/// VMSAv7 fault reason; maps onto the DFSR/IFSR status codes.
enum class MmFaultKind : int {
    None = 0,
    Alignment,
    Background,
    Section,
    Page,
    Domain,
    Permission,
    AccessFlag,
};

const char* fault_name(MmFaultKind kind);

/// Human readable fault description ("permission fault @ VA 0x... (write)").
std::string fault_text(MmFaultKind kind, u32 va, bool write, bool fetch);

/// Result of one MMU translation.
struct MmResult {
    bool ok = false;
    u32 phys_addr = 0;
    bool strongly_ordered = false;
    bool device = false;
    bool normal = false;
    bool supersection = false;
    MmFaultKind fault = MmFaultKind::None;
    u32 fsr_status = 0;
    u32 fsr_full = 0;
};

// ---------------------------------------------------------------------------
// Small helpers shared by the cores
// ---------------------------------------------------------------------------

/// 32-bit population count.
inline int popcount32(u32 v) {
    int n = 0;
    while (v != 0) {
        n += static_cast<int>(v & 1u);
        v >>= 1;
    }
    return n;
}

/// Byte swap of a 32-bit value.
inline u32 reverse_bytes(u32 v) {
    return ((v & 0x000000FFu) << 24) | ((v & 0x0000FF00u) << 8) | ((v & 0x00FF0000u) >> 8) |
           ((v & 0xFF000000u) >> 24);
}

/// Rotate right by 0..31, with 0 meaning "no rotation".
inline u32 rotate_right(u32 v, int r) {
    if (r == 0) return v;
    return (v >> r) | (v << (32 - r));
}

/// Count leading zeros (32 on a zero input).
inline u32 count_leading_zeros(u32 v) {
    if (v == 0) return 32;
    u32 n = 0;
    if ((v & 0xFFFF0000u) == 0) { n += 16; v <<= 16; }
    if ((v & 0xFF000000u) == 0) { n += 8; v <<= 8; }
    if ((v & 0xF0000000u) == 0) { n += 4; v <<= 4; }
    if ((v & 0xC0000000u) == 0) { n += 2; v <<= 2; }
    if ((v & 0x80000000u) == 0) { n += 1; }
    return n;
}

/// Sign extend the low `bits` bits of `value` into a signed 32-bit int.
inline s32 sign_extend32(u32 value, unsigned bits) {
    const u32 m = 1u << (bits - 1);
    return static_cast<s32>((value ^ m) - m);
}

/// Saturate a signed value to `bits` bits (ARM ARM A2.3.1).
inline s32 saturate_signed(s64 v, unsigned bits) {
    const s64 max = (static_cast<s64>(1) << (bits - 1)) - 1;
    const s64 min = -(static_cast<s64>(1) << (bits - 1));
    if (v > max) return static_cast<s32>(max);
    if (v < min) return static_cast<s32>(min);
    return static_cast<s32>(v);
}

/// Saturate an unsigned value into `bits` bits (ARM ARM A2.3.2), used by the
/// unsigned saturating parallel forms UQADD8/16, UQSUB8/16, UQASX, UQSAX.
inline u32 saturate_unsigned(s64 v, unsigned bits) {
    const s64 max = (static_cast<s64>(1) << bits) - 1;
    if (v < 0) return 0u;
    if (v > max) return static_cast<u32>(max);
    return static_cast<u32>(v);
}

/// ThumbExpandImm - expand a T32 12-bit modified immediate (ARM ARM A7.4.3).
inline u32 thumb_expand_imm(u32 imm12) {
    const u32 top8 = (imm12 >> 8) & 0xFu;
    const u32 low8 = imm12 & 0xFFu;
    if ((imm12 & 0xC00u) == 0u) {
        switch (top8) {
            case 0u: return low8;
            case 1u: return (low8 << 16) | low8;
            case 2u: return (low8 << 24) | (low8 << 8);
            case 3u: return (low8 << 24) | (low8 << 16) | (low8 << 8) | low8;
            default: {
                const u32 v = low8 | 0x80u;
                const int rot = static_cast<int>(top8 - 8u);
                return (v >> rot) | (v << (32 - rot));
            }
        }
    }
    const u32 unrotated = 0x80u | low8;
    const int r = static_cast<int>((imm12 >> 7) & 0x1Fu);
    if (r == 0) return unrotated;
    return (unrotated >> r) | (unrotated << (32 - r));
}

/// Expand an A32 12-bit rotated immediate.
inline u32 decode_imm12(u32 instr) {
    const u32 imm = instr & 0xFFu;
    const u32 rot = ((instr >> 8) & 0xFu) * 2u;
    if (rot == 0) return imm;
    return (imm >> static_cast<int>(rot)) | (imm << (32 - static_cast<int>(rot)));
}

/// Address of the PC as seen by an instruction operand (PC + 8 / PC + 4).
inline u32 read_pc_value(bool thumb, u32 instr_addr) {
    if (thumb) return (instr_addr + 4u) & 0xFFFFFFFCu;
    return instr_addr + 8u;
}

/// A32 condition-code evaluation (ARM ARM A8.3). `cond` is the raw 4-bit field.
inline bool condition_passed(u32 cond, u32 cpsr) {
    const bool n = (cpsr & kFlagN) != 0;
    const bool z = (cpsr & kFlagZ) != 0;
    const bool c = (cpsr & kFlagC) != 0;
    const bool v = (cpsr & kFlagV) != 0;
    switch (cond) {
        case 0x0u: return z;
        case 0x1u: return !z;
        case 0x2u: return c;
        case 0x3u: return !c;
        case 0x4u: return n;
        case 0x5u: return !n;
        case 0x6u: return v;
        case 0x7u: return !v;
        case 0x8u: return c && !z;
        case 0x9u: return !c || z;
        case 0xAu: return n == v;
        case 0xBu: return n != v;
        case 0xCu: return !z && (n == v);
        case 0xDu: return z || (n != v);
        default: return true;
    }
}

/// Condition-code mnemonic suffix ("eq", "ne", ..., "" for AL, "??" reserved).
inline const char* condition_name(u32 cond) {
    switch (cond) {
        case 0x0u: return "eq";
        case 0x1u: return "ne";
        case 0x2u: return "cs";
        case 0x3u: return "cc";
        case 0x4u: return "mi";
        case 0x5u: return "pl";
        case 0x6u: return "vs";
        case 0x7u: return "vc";
        case 0x8u: return "hi";
        case 0x9u: return "ls";
        case 0xAu: return "ge";
        case 0xBu: return "lt";
        case 0xCu: return "gt";
        case 0xDu: return "le";
        case 0xEu: return "";
        default: return "??";
    }
}

/// Mode name for the CPSR mode field ("SVC", "IRQ", ...).
inline const char* mode_name(u32 mode) {
    switch (mode & kModeMask) {
        case kModeUser: return "USR";
        case kModeFiq: return "FIQ";
        case kModeIrq: return "IRQ";
        case kModeSupervisor: return "SVC";
        case kModeMonitor: return "MON";
        case kModeAbort: return "ABT";
        case kModeHyp: return "HYP";
        case kModeUndefined: return "UND";
        case kModeSystem: return "SYS";
        default: return "???";
    }
}

}  // namespace arm
}  // namespace zlb
