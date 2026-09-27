// zeliboba - ARMv7-A (Cortex-A9 MPCore) VMSAv7 short-descriptor MMU.
//
// Implements the translation scheme the PS Vita's Kermit main CPU uses while
// SCTLR.M is set:
//
//   * first level descriptors: fault / coarse page table / section / supersection
//   * second level descriptors: fault / large page (64K, 4 subpages) / small page (4K)
//   * domains (DACR), AP/APX permissions, XN/XNX execute-never, TEX/C/B attributes
//   * access-flag update and DFSR/IFSR/DFAR/IFAR fault reporting
//
// The "MMU disabled" case is handled by Translate(): the virtual address is
// returned unchanged, which is what the kernel boot loader relies on before it
// switches the MMU on.
#pragma once

#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/types.h"
#include "cpu/arm/arm_defs.h"

namespace zlb {

/// One Cortex-A9 core's worth of CP15 / translation state.
class ArmMmu {
public:
    explicit ArmMmu(Bus& bus) : bus_(&bus) {}

    // ---- CP15 system control registers ------------------------------------

    /// SCTLR: bit 0 = M, bit 1 = A, bit 2 = C, bit 3 = W, bit 12 = I, bit 13 = V.
    u32 sctlr = 0x00C50078;  // Cortex-A9 reset value (MMU off, Vectors low)
    u32 ttbr0 = 0;
    u32 ttbr1 = 0;
    u32 ttbcr = 0;
    u32 dacr = 0x00000001;   // Cortex-A9 reset: domain 0 = client
    u32 dfsr = 0;
    u32 dfar = 0;
    u32 ifsr = 0;
    u32 ifar = 0;
    u32 adfsr = 0;
    u32 aifsr = 0;
    u32 prrr = 0x98E8E8E8;
    u32 nmrr = 0x00009898;
    u32 vbar = 0;
    u32 context_idr = 0;

    /// PAR (CP15 c7, c4, 0): the result of the last VA-to-PA (V2P) operation.
    /// The kernel boot loader uses the hardware translator to find the physical
    /// address behind a virtual one before it patches the section describing it,
    /// so this register is part of the boot path and not just a debug nicety.
    u32 par = 0;

    /// Translation table walks performed (statistics for the debugger).
    u64 walks = 0;

    // ---- translation / fault records --------------------------------------
    //
    // The debugger's first question after an abort is "why", and the answer is
    // always in DFSR/DFAR plus the descriptor the walk actually used. These
    // records keep the last walk and a small ring of the first distinct faults
    // so `describe_state()` can answer without the reader reconstructing it from
    // the bus trace. Fixed size, O(1) per access.
    struct WalkRecord {
        u32 va = 0;
        u32 pc = 0;
        u32 ttbr_base = 0;
        int ttbr_num = -1;
        u32 l1_addr = 0;
        u32 l1_desc = 0;
        u32 l2_addr = 0;
        u32 l2_desc = 0;
        u32 domain = 0;
        bool used_l2 = false;
        arm::MmFaultKind fault = arm::MmFaultKind::None;
        bool ok = false;
        bool write = false;
        bool fetch = false;
        u64 repeats = 1;
    };

    static constexpr int kFaultLogSize = 8;
    WalkRecord last_walk;
    WalkRecord faults[kFaultLogSize];
    int fault_count = 0;   ///< number of entries written (<= kFaultLogSize)
    u64 total_faults = 0;

    void note_fault(const WalkRecord& walk, u32 pc, bool write, bool fetch);

    void reset();

    bool enabled() const { return (sctlr & 1u) != 0; }
    bool strict_alignment() const { return (sctlr & 2u) != 0; }
    bool high_vectors() const { return (sctlr & (1u << 13)) != 0; }

    /// Exception vector base: VBAR when set, else 0xFFFF0000 with high vectors.
    u32 vector_base() const;

    /// Translate a virtual address. Always succeeds when the MMU is off.
    arm::MmResult translate(u32 va, bool write, bool fetch, u32 mode);

    /// Update DFSR/DFAR for a data abort.
    void report_data_abort(const arm::MmResult& result, u32 va, bool write);
    /// Update IFSR/IFAR for a prefetch abort.
    void report_prefetch_abort(const arm::MmResult& result, u32 va);

    /// One line summary for the debugger status view.
    std::string describe() const;

    /// Multi-line CP15 / MMU state for the debugger's `info` and `describe_state`
    /// views: SCTLR, TTBR0/TTBR1/TTBCR/DACR, the DFSR/DFAR/IFSR/IFAR fault
    /// registers, the last translation performed and the first distinct faults.
    void describe_extended(std::vector<std::string>& lines) const;


    /// TTBR0/TTBR1 base address as used by a walk (bit [13:0] masked off).
    u32 ttbr_base_for(u32 va, int& ttbr_num) const { return select_ttbr(va, ttbr_num); }

private:
    u32 read_table(u32 address);
    u32 select_ttbr(u32 va, int& ttbr_num) const;
    arm::MmResult translate_walk(u32 va, bool write, bool fetch, u32 mode);

    Bus* bus_;
};

}  // namespace zlb
