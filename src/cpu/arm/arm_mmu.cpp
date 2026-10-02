#include "cpu/arm/arm_mmu.h"

#include <cstdio>
#include <vector>

#include "common/log.h"
#include "common/util.h"

namespace zlb {

namespace {

/// Build the complete DFSR/IFSR value for a fault (ARM ARM B3.9.5, table B3-26).
/// Bit 11 is WnR (data aborts only).
u32 fsr_for(arm::MmFaultKind kind, bool write, u32 domain, bool second_level) {
    u32 status;
    switch (kind) {
        case arm::MmFaultKind::Alignment: status = 0x01u; break;
        case arm::MmFaultKind::Background: status = 0x00u; break;
        case arm::MmFaultKind::Domain: status = 0x09u | ((domain & 0xFu) << 4); break;
        case arm::MmFaultKind::Permission: status = second_level ? 0x0Fu : 0x0Du; break;
        case arm::MmFaultKind::AccessFlag: status = second_level ? 0x06u : 0x03u; break;
        case arm::MmFaultKind::Section: status = 0x05u; break;
        default: status = 0x07u; break;  // page translation fault
    }
    u32 full = status & 0x7FFu;
    if (write && kind != arm::MmFaultKind::Background) full |= 1u << 11;
    return full;
}

bool domain_fault(u32 dacr, u32 domain) {
    return ((dacr >> static_cast<int>(domain * 2)) & 3u) == 0u;  // 0 = no access
}

bool domain_manager(u32 dacr, u32 domain) {
    return ((dacr >> static_cast<int>(domain * 2)) & 3u) == 3u;  // 3 = manager
}

/// All short descriptors use the same AP[2:0] permissions (ARM ARM B3.7.1,
/// table B3-8). Section AP[2] is descriptor bit 15; page AP[2] is bit 9.
bool check_ap(u32 ap, u32 mode, bool write) {
    const bool privileged = (mode & arm::kModeMask) != arm::kModeUser;
    switch (ap & 7u) {
        case 0u: return false;
        case 1u: return privileged;
        case 2u: return privileged || !write;
        case 3u: return true;
        case 5u: return privileged && !write;
        case 6u: case 7u: return !write;
        default: return false; // 0b100 is reserved; do not grant access.
    }
}

/// Classify a TEX/C/B combination into Normal / Device / Strongly-ordered.
void classify_memory(u32 tex, u32 c, u32 b, arm::MmResult& result) {
    if (tex == 0u && c == 0u && b == 0u) result.strongly_ordered = true;
    else if ((tex == 0u && c == 0u && b == 1u) ||
             (tex == 2u && c == 0u && b == 0u)) result.device = true;
    else result.normal = true;
}

}  // namespace

const char* arm::fault_name(arm::MmFaultKind kind) {
    switch (kind) {
        case arm::MmFaultKind::Alignment: return "alignment fault";
        case arm::MmFaultKind::Background: return "translation fault (no TTBR entry)";
        case arm::MmFaultKind::Domain: return "domain fault";
        case arm::MmFaultKind::Permission: return "permission fault";
        case arm::MmFaultKind::AccessFlag: return "access flag fault";
        case arm::MmFaultKind::Section: return "section translation fault";
        default: return "page translation fault";
    }
}

std::string arm::fault_text(arm::MmFaultKind kind, u32 va, bool write, bool fetch) {
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "%s @ VA 0x%08X (%s)", arm::fault_name(kind), va,
                  fetch ? "fetch" : (write ? "write" : "read"));
    return std::string(buffer);
}

void ArmMmu::reset() {
    sctlr = 0x00C50078;
    ttbr0 = 0;
    ttbr1 = 0;
    ttbcr = 0;
    dacr = 0x00000001;
    dfsr = 0;
    dfar = 0;
    ifsr = 0;
    ifar = 0;
    adfsr = 0;
    aifsr = 0;
    prrr = 0x98E8E8E8;
    nmrr = 0x00009898;
    vbar = 0;
    context_idr = 0;
    par = 0;
    walks = 0;
    last_walk = WalkRecord{};
    for (auto& record : faults) record = WalkRecord{};
    fault_count = 0;
    total_faults = 0;
}

void ArmMmu::note_fault(const WalkRecord& walk, u32 pc, bool write, bool fetch) {
    ++total_faults;
    // `walk` is a snapshot the caller took before it knew the faulting PC, so the
    // PC and the access kind have to be filled in here - both for last_walk and
    // for the ring the debugger prints ("which instruction faulted" is the whole
    // point of keeping records at all).
    WalkRecord record = walk;
    record.pc = pc;
    record.write = write;
    record.fetch = fetch;
    record.repeats = 1;
    last_walk = record;
    // Collapse an exception loop (the same VA/PC faulting forever) into one
    // record with a repeat count instead of flooding the view.
    if (fault_count > 0) {
        WalkRecord& previous = faults[fault_count - 1];
        if (previous.va == record.va && previous.fault == record.fault && previous.pc == record.pc) {
            ++previous.repeats;
            ++last_walk.repeats;
            return;
        }
    }
    if (fault_count < kFaultLogSize) {
        faults[fault_count] = record;
        ++fault_count;
    }
}

u32 ArmMmu::vector_base() const {
    if (vbar != 0) return vbar;
    if (high_vectors()) return 0xFFFF0000u;
    return 0x00000000u;
}

u32 ArmMmu::read_table(u32 address) {
    ++walks;
    return bus_->read32(address & 0xFFFFFFFCu);
}

/// TTBR select according to TTBCR.N (ARM ARM B3.5.2).
u32 ArmMmu::select_ttbr(u32 va, int& ttbr_num) const {
    const u32 n = ttbcr & 7u;
    if (n == 0) {
        ttbr_num = 0;
        return ttbr0 & 0xFFFFC000u;
    }
    const u32 boundary = 1u << (32 - static_cast<int>(n));
    if (va < boundary) {
        ttbr_num = 0;
        return ttbr0 & 0xFFFFC000u;
    }
    ttbr_num = 1;
    u32 mask = 0xFFFFC000u;
    mask &= ~((1u << (14 - static_cast<int>(n))) - 1u);
    return ttbr1 & mask;
}

/// Record every translation (successful or not) and every fault, then return.
/// The wrapper keeps the hot translation path free of bookkeeping branches other
/// than the ones the debugger actually needs.
arm::MmResult ArmMmu::translate(u32 va, bool write, bool fetch, u32 mode) {
    const arm::MmResult result = translate_walk<false>(va, write, fetch, mode);
    if (!result.ok) {
        // Faults are rare, so the rich record is built only here; a successful
        // translation is not recorded at all unless the debugger asked for it
        // (record_walks, ZLB_MMU_WALKS=1).  Copying a WalkRecord per memory access
        // used to cost more than the walk itself.
        WalkRecord record = last_walk;
        record.va = va;
        record.fault = result.fault;
        record.write = write;
        record.fetch = fetch;
        note_fault(record, bus_->context.pc, write, fetch);
        return result;
    }
    if (record_walks) {
        WalkRecord record = last_walk;
        record.va = va;
        record.ok = true;
        record.write = write;
        record.fetch = fetch;
        last_walk = record;
    }
    return result;
}

arm::MmResult ArmMmu::inspect_translation(u32 va, bool fetch, u32 mode) const {
    ArmMmu snapshot = *this;
    return snapshot.translate_walk<true>(va, false, fetch, mode);
}

template <bool Inspect>
arm::MmResult ArmMmu::translate_walk(u32 va, bool write, bool fetch, u32 mode) {
    arm::MmResult result;
    if (record_walks) last_walk = WalkRecord{};

    const auto read_descriptor = [&](u32 address) -> u32 {
        if constexpr (!Inspect) {
            return read_table(address);
        } else {
            address &= ~3u;
            const MemRegion* region = bus_->region_at(address, 4);
            if (!region) return 0u;
            for (u32 byte = 0; byte < 4; ++byte) {
                if (bus_->find_device(address + byte)) return 0u;
            }
            const u8* bytes = region->bytes() + (address - region->base);
            return static_cast<u32>(bytes[0]) | (static_cast<u32>(bytes[1]) << 8) |
                   (static_cast<u32>(bytes[2]) << 16) | (static_cast<u32>(bytes[3]) << 24);
        }
    };

    if (!enabled()) {
        result.ok = true;
        result.phys_addr = va;
        result.normal = true;
        return result;
    }

    int ttbr_num = 0;
    u32 ttbr = select_ttbr(va, ttbr_num);
    if (ttbr_num == 1 && (ttbcr & 7u) == 0) ttbr = ttbr0 & 0xFFFFC000u;
    last_walk.ttbr_num = ttbr_num;
    last_walk.ttbr_base = ttbr;

    const u32 l1_index = (va >> 20) & 0xFFFu;
    const u32 l1_addr = ttbr | (l1_index << 2);
    const u32 l1 = read_descriptor(l1_addr);
    last_walk.l1_addr = l1_addr;
    last_walk.l1_desc = l1;

    const u32 type = l1 & 3u;
    if (type == 0u || type == 3u) {
        result.fault = arm::MmFaultKind::Section;
        result.fsr_full = fsr_for(arm::MmFaultKind::Section, write, 0, false);
        result.fsr_status = result.fsr_full;
        return result;
    }

    if (type == 2u) {
        // Section (1 MiB) or supersection (16 MiB).
        const bool supersection = (l1 & (1u << 18)) != 0;
        const u32 ns = (l1 >> 19) & 1u;
        const u32 domain = (l1 >> 5) & 0xFu;
        last_walk.domain = domain;
        const u32 ap = ((l1 >> 10) & 3u) | (((l1 >> 15) & 1u) << 2);
        const u32 tex = (l1 >> 12) & 7u;
        const u32 c = (l1 >> 3) & 1u;
        const u32 b = (l1 >> 2) & 1u;
        const u32 xn = (l1 >> 4) & 1u;

        if (domain_fault(dacr, domain)) {
            if constexpr (!Inspect) {
                ZLB_LOG_DBG("mmu", "domain fault: VA=0x%08X dacr=0x%08X domain=%u l1=0x%08X", va, dacr, domain,
                            l1);
            }
            result.fault = arm::MmFaultKind::Domain;
            result.fsr_full = fsr_for(arm::MmFaultKind::Domain, write, domain, false);
            result.fsr_status = result.fsr_full;
            return result;
        }

        const bool manager = domain_manager(dacr, domain);
        if (!manager && (sctlr & (1u << 29)) != 0u && (ap & 1u) == 0u) {
            result.fault = arm::MmFaultKind::AccessFlag;
            result.fsr_full = fsr_for(arm::MmFaultKind::AccessFlag, write, domain, false);
            result.fsr_status = result.fsr_full;
            return result;
        }
        if (!manager && !check_ap(ap, mode, write)) {
            result.fault = arm::MmFaultKind::Permission;
            result.fsr_full = fsr_for(arm::MmFaultKind::Permission, write, domain, false);
            result.fsr_status = result.fsr_full;
            return result;
        }

        if (fetch && !manager && xn != 0) {
            result.fault = arm::MmFaultKind::Permission;
            result.fsr_full = fsr_for(arm::MmFaultKind::Permission, write, domain, false) | (1u << 3);
            result.fsr_status = result.fsr_full;
            return result;
        }

        u32 phys;
        if (supersection) {
            if (ns == 1) {
                result.fault = arm::MmFaultKind::Section;
                result.fsr_full = 0x05u;
                result.fsr_status = result.fsr_full;
                return result;
            }
            phys = (l1 & 0xFF000000u) | (va & 0x00FFFFFFu);
        } else {
            phys = (l1 & 0xFFF00000u) | (va & 0x000FFFFFu);
        }

        classify_memory(tex, c, b, result);

        result.ok = true;
        result.phys_addr = phys;
        result.supersection = supersection;
        return result;
    }

    // type == 1: coarse page table.
    const u32 l2_base = l1 & 0xFFFFFC00u;
    const u32 l2_index = (va >> 12) & 0xFFu;
    const u32 l2_addr = l2_base | (l2_index << 2);
    u32 l2 = read_descriptor(l2_addr);
    // Development substitution (round 236, docs/NSKBL.md 8.69): keep a section that the
    // guest replaced with this page table serving the pages the table leaves unmapped.
    // The synthesized descriptor is the small-page equivalent of the section's
    // attributes (AP[2:0] = {bit 15, bit 11, bit 10} -> {bit 9, bit 5, bit 4},
    // TEX 14:12 -> 8:6, C/B stay bits 3/2, XN bit 4 -> bit 0) and the section's
    // physical address with the in-section offset.
    if ((l2 & 3u) == 0u && replaced_section != 0u &&
        (va & 0xFFF00000u) == (replaced_section & 0xFFF00000u)) {
        const u32 pa = (replaced_section & 0xFFF00000u) | (va & 0x000FF000u);
        l2 = pa | ((replaced_section & 0x00007000u) >> 6) |
             (((replaced_section >> 10) & 3u) << 4) | (((replaced_section >> 15) & 1u) << 9) |
             (replaced_section & 0x0000000Cu) | ((replaced_section >> 4) & 1u) | 2u;
        ++replaced_section_hits;
    }
    last_walk.used_l2 = true;
    last_walk.l2_addr = l2_addr;
    last_walk.l2_desc = l2;

    const u32 t2 = l2 & 3u;
    if (t2 == 0u) {
        result.fault = arm::MmFaultKind::Page;
        result.fsr_full = fsr_for(arm::MmFaultKind::Page, write, 0, true);
        result.fsr_status = result.fsr_full;
        return result;
    }

    // The domain for a page-table mapping comes from the *first-level* page
    // table descriptor (bits [8:5]), not from the second-level entry: bits [8:6]
    // of a small-page descriptor are TEX and bit 5 is AP[0]. Reading a domain
    // out of L2 picks up TEX bits and silently changes the permission model of
    // the whole page table.
    const u32 domain2 = (l1 >> 5) & 0xFu;
    last_walk.domain = domain2;
    if (domain_fault(dacr, domain2)) {
        result.fault = arm::MmFaultKind::Domain;
        result.fsr_full = fsr_for(arm::MmFaultKind::Domain, write, domain2, true);
        result.fsr_status = result.fsr_full;
        return result;
    }
    const bool manager2 = domain_manager(dacr, domain2);

    const bool large = t2 == 1u;

    u32 ap_field;
    u32 tex;
    u32 c;
    u32 b;
    u32 xn;
    u32 phys;

    if (large) {
        // ARMv7 large pages have one AP field for all 64 KiB (ARM ARM B3.5.1,
        // figure B3-5), not ARMv5's four subpage permission fields.
        ap_field = ((l2 >> 4) & 3u) | (((l2 >> 9) & 1u) << 2);
        tex = (l2 >> 12) & 7u;
        c = (l2 >> 3) & 1u;
        b = (l2 >> 2) & 1u;
        xn = (l2 >> 15) & 1u;
        phys = (l2 & 0xFFFF0000u) | (va & 0x0000FFFFu);
    } else {
        // Small page: 4 KiB. AP[2:0] = {descriptor[9], descriptor[5:4]}.
        const u32 ap10 = (l2 >> 4) & 3u;
        const u32 ap2 = (l2 >> 9) & 1u;
        ap_field = ap10 | (ap2 << 2);
        tex = (l2 >> 6) & 7u;
        c = (l2 >> 3) & 1u;
        b = (l2 >> 2) & 1u;
        // XN is *bit 0* of a small-page descriptor (bits [1:0] = 0b1x select the
        // format, so bit 0 is free for it) - bit 15 belongs to the physical address
        // here, because a 4 KiB page carries PA[31:12].  Reading bit 15 made every
        // page whose PA has bit 15 set execute-never: PA 0x40118000 (where the model
        // backs the ARM low window) is one of them, so the very first instruction
        // fetch of an exception vector there raised IFSR = 0xF.  Evidence:
        // `arm_mmu_small_page_xn_is_bit_zero` in tests/test_arm.cpp.
        xn = l2 & 1u;
        phys = (l2 & 0xFFFFF000u) | (va & 0x00000FFFu);
    }

    // Cortex-A9 uses software management of AF: when AFE is set, AP[0]
    // must already be 1. With AF set, the shared AP table also implements
    // the simplified AP[2:1] permissions model.
    if (!manager2 && (sctlr & (1u << 29)) != 0u && (ap_field & 1u) == 0u) {
        result.fault = arm::MmFaultKind::AccessFlag;
        result.fsr_full = fsr_for(arm::MmFaultKind::AccessFlag, write, domain2, true);
        result.fsr_status = result.fsr_full;
        return result;
    }
    const bool allowed = check_ap(ap_field, mode, write);
    if (!manager2 && !allowed) {
        if constexpr (!Inspect) {
            ZLB_LOG_DBG("mmu", "permission fault (page): VA=0x%08X %s%s mode=0x%02X l2=0x%08X ap_field=0x%X",
                        va, write ? "write" : "read", fetch ? "/fetch" : "", mode & arm::kModeMask, l2,
                        ap_field);
        }
        result.fault = arm::MmFaultKind::Permission;
        result.fsr_full = fsr_for(arm::MmFaultKind::Permission, write, domain2, true);
        result.fsr_status = result.fsr_full;
        return result;
    }

    if (fetch && !manager2 && xn != 0) {
        if constexpr (!Inspect) {
            ZLB_LOG_DBG("mmu", "execute-never (page): VA=0x%08X l2=0x%08X", va, l2);
        }
        result.fault = arm::MmFaultKind::Permission;
        result.fsr_full = fsr_for(arm::MmFaultKind::Permission, write, domain2, true) | (1u << 3);
        result.fsr_status = result.fsr_full;
        return result;
    }

    classify_memory(tex, c, b, result);

    result.ok = true;
    result.phys_addr = phys;
    return result;
}

void ArmMmu::report_data_abort(const arm::MmResult& result, u32 va, bool write) {
    if (result.fault == arm::MmFaultKind::Alignment) {
        dfsr = 0x01u | (write ? (1u << 11) : 0u);
        dfar = va;
    } else {
        dfsr = result.fsr_full & 0xFFFu;
        if (write) dfsr |= 1u << 11;
        else dfsr &= ~(1u << 11);
        if ((dfsr & 0xFu) != 0x00u) dfar = va;
    }
    adfsr = dfsr;
}

void ArmMmu::report_prefetch_abort(const arm::MmResult& result, u32 va) {
    ifsr = result.fsr_full & 0xFFFu;
    if (result.fault == arm::MmFaultKind::Permission && (result.fsr_full & (1u << 3)) != 0) {
        ifsr = 0x0Fu | (1u << 3);  // section/page permission fault, XN
    }
    ifar = va;
    aifsr = ifsr;
}

void ArmMmu::describe_extended(std::vector<std::string>& lines) const {
    lines.push_back(format("MMU %s  SCTLR=0x%08X (M=%u A=%u C=%u W=%u I=%u V=%u TE=%u)",
                           enabled() ? "on" : "off", sctlr, sctlr & 1u, (sctlr >> 1) & 1u,
                           (sctlr >> 2) & 1u, (sctlr >> 3) & 1u, (sctlr >> 12) & 1u,
                           (sctlr >> 13) & 1u, (sctlr >> 30) & 1u));
    {
        int num = 0;
        const u32 ttbr0_base = select_ttbr(0u, num);
        const u32 ttbr1_base = select_ttbr(0xFFFFFFFFu, num);
        lines.push_back(format("CP15 TTBR0=0x%08X (base 0x%08X)  TTBR1=0x%08X (base 0x%08X)  "
                               "TTBCR=0x%X (N=%u)  DACR=0x%08X",
                               ttbr0, ttbr0_base, ttbr1, ttbr1_base, ttbcr, ttbcr & 7u, dacr));
    }
    lines.push_back(format("CP15 DFSR=0x%08X (%s)  DFAR=0x%08X  IFSR=0x%08X  IFAR=0x%08X  "
                           "VBAR=0x%08X  vectors=0x%08X",
                           dfsr, arm::fault_name(last_walk.fault), dfar, ifsr, ifar, vbar,
                           vector_base()));
    lines.push_back(format("CP15 walks=%llu  faults=%llu  PAR=0x%08X (%s 0x%08X)",
                           static_cast<unsigned long long>(walks),
                           static_cast<unsigned long long>(total_faults), par,
                           (par & 1u) ? "last V2P faulted, status" : "last VA->PA =",
                           (par & 1u) ? (par >> 1) : (par & 0xFFFFF000u)));
    if (last_walk.va != 0 || last_walk.ttbr_num >= 0) {
        lines.push_back(format("last walk: VA=0x%08X pc=0x%08X %s%s -> TTBR%d base=0x%08X "
                               "L1[0x%08X]=0x%08X%s%s",
                               last_walk.va, last_walk.pc, last_walk.fetch ? "fetch" : "data",
                               last_walk.write ? "(w)" : "", last_walk.ttbr_num,
                               last_walk.ttbr_base, last_walk.l1_addr, last_walk.l1_desc,
                               last_walk.used_l2 ? format(" L2[0x%08X]=0x%08X", last_walk.l2_addr,
                                                          last_walk.l2_desc).c_str()
                                                 : "",
                               last_walk.ok ? "  OK" : "  FAULT"));
    }
    for (int i = 0; i < fault_count; ++i) {
        const WalkRecord& record = faults[i];
        lines.push_back(format("fault[%d]: VA=0x%08X pc=0x%08X %s%s %s%s  domain=%u  "
                               "TTBR%d base=0x%08X L1[0x%08X]=0x%08X%s  x%llu",
                               i, record.va, record.pc, record.fetch ? "fetch" : "data",
                               record.write ? "(w)" : "", arm::fault_name(record.fault),
                               record.used_l2 ? " (page)" : " (section)", record.domain,
                               record.ttbr_num, record.ttbr_base, record.l1_addr, record.l1_desc,
                               record.used_l2 ? format(" L2[0x%08X]=0x%08X", record.l2_addr,
                                                       record.l2_desc).c_str()
                                              : "",
                               static_cast<unsigned long long>(record.repeats)));
    }
}
std::string ArmMmu::describe() const {
    char buffer[192];
    std::snprintf(buffer, sizeof(buffer),
                  "MMU %s  TTBR0=0x%08X TTBR1=0x%08X TTBCR=0x%X DACR=0x%08X walks=%llu",
                  enabled() ? "on" : "off", ttbr0, ttbr1, ttbcr, dacr,
                  static_cast<unsigned long long>(walks));
    return std::string(buffer);
}
}  // namespace zlb
