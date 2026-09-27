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

/// AP/APX check. Returns true when the access is allowed.
///
/// Short-descriptor format uses two different encodings and they are easy to
/// conflate:
///   * first level (sections): AP[1:0] in bits [11:10] plus APX in bit 15.
///     APX is an *override to no user access*, not an AP[2] read-only bit, so
///     AP[1:0] = 0b11 with APX = 0 is "read/write for everyone" (ARM ARM B3-13).
///   * second level (pages): a real 3-bit AP[2:0] where AP[2] = read-only.
bool check_ap(u32 ap, u32 apx, u32 mode, bool write, bool page_table_format) {
    const bool privileged = (mode & arm::kModeMask) != arm::kModeUser;

    if (page_table_format) {
        // AP[2] (bit 2) = read-only for privileged modes.
        const u32 ap2 = (ap >> 2) & 1u;
        const u32 ap10 = ap & 3u;
        if (write && ap2 != 0) return false;   // read-only for everyone
        if ((ap10 & 1u) == 0) return privileged;  // 0b00 / 0b10: no user access
        if ((ap10 & 2u) != 0) return true;     // 0b_11: full access
        return privileged || !write;           // 0b_01: user read-only
    }

    const u32 ap01 = ap & 3u;
    const u32 apx2 = apx & 1u;
    if (apx2 != 0) {
        // APX set: privileged permissions from AP[1:0], no user access at all.
        return (ap01 >= 2u) ? (privileged && !write) : privileged;
    }
    switch (ap01) {
        case 0: return privileged;              // privileged RW, user none
        case 1: return privileged || !write;    // privileged RW, user RO
        case 2: return privileged && !write;    // privileged RO, user none
        default: return true;                   // read/write for everyone
    }
}

/// Large page AP check: the second level AP field selects one of four subpage
/// permission sets (ARM ARM B3.7.3, table B3-16).
bool check_large_ap(u32 ap, u32 subpage, u32 mode, bool write) {
    const bool privileged = (mode & arm::kModeMask) != arm::kModeUser;
    switch (ap & 3u) {
        case 0u: return subpage == 0u && privileged;
        case 1u:
            if (subpage == 0u) return false;
            if (subpage == 1u) return privileged;
            return !write;  // subpages 2,3: user read-only
        case 2u: return privileged || !write || (subpage >= 2u);
        default: return !write;  // read-only for everyone
    }
}

/// Classify a TEX/C/B combination into Normal / Device / Strongly-ordered.
void classify_memory(u32 tex_cb, u32 tex, arm::MmResult& result) {
    const u32 main = tex_cb & 7u;  // TEX[0],C,B
    const u32 tex_high = (tex >> 1) & 3u;
    if (tex_high != 0u) {
        if (tex_high == 1u) {
            result.strongly_ordered = true;
            return;
        }
        result.normal = true;
        return;
    }
    switch (main) {
        case 0: result.strongly_ordered = true; break;
        case 1: result.device = true; break;  // shared device
        case 2: result.device = true; break;  // non-shared device (TEX=0b010)
        case 3: result.normal = true; break;  // write-back, no write allocate
        case 6: result.device = true; break;  // TEX=0b001, C=1,B=0
        case 7: result.normal = true; break;  // write-back, write allocate
        default: result.normal = true; break;
    }
}

}  // namespace

const char* arm::fault_name(arm::MmFaultKind kind) {
    switch (kind) {
        case arm::MmFaultKind::Alignment: return "alignment fault";
        case arm::MmFaultKind::Background: return "translation fault (no TTBR entry)";
        case arm::MmFaultKind::Domain: return "domain fault";
        case arm::MmFaultKind::Permission: return "permission fault";
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
    const arm::MmResult result = translate_walk(va, write, fetch, mode);
    WalkRecord record = last_walk;
    record.va = va;
    record.ok = result.ok;
    record.fault = result.fault;
    record.write = write;
    record.fetch = fetch;
    if (!result.ok) note_fault(record, bus_->context.pc, write, fetch);
    else last_walk = record;
    return result;
}

arm::MmResult ArmMmu::translate_walk(u32 va, bool write, bool fetch, u32 mode) {
    arm::MmResult result;
    last_walk = WalkRecord{};

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
    const u32 l1 = read_table(l1_addr);
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
        const u32 ap = (l1 >> 10) & 3u;
        const u32 apx = (l1 >> 15) & 1u;
        const u32 tex = (l1 >> 12) & 7u;
        const u32 c = (l1 >> 3) & 1u;
        const u32 b = (l1 >> 2) & 1u;
        const u32 xn = (l1 >> 4) & 1u;

        if (domain_fault(dacr, domain)) {
            ZLB_LOG_DBG("mmu", "domain fault: VA=0x%08X dacr=0x%08X domain=%u l1=0x%08X", va, dacr, domain,
                          l1);
            result.fault = arm::MmFaultKind::Domain;
            result.fsr_full = fsr_for(arm::MmFaultKind::Domain, write, domain, false);
            result.fsr_status = result.fsr_full;
            return result;
        }

        const bool manager = domain_manager(dacr, domain);
        if (!manager && !check_ap(ap, apx, mode, write, false)) {
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

        // First level TEX[0],C,B / TEX[2:1] encoding (ARM ARM B3.7, table B3-13).
        const u32 tex_cb = ((tex & 1u) << 3) | (c << 2) | (b << 1);
        classify_memory(tex_cb, tex, result);

        // Access-flag update is deliberately not performed. For a first-level
        // section descriptor bits [8:5] hold the *domain*, so the reference
        // core's `l1 |= 1 << 8` writes into the domain field and silently
        // changes the permission model of the whole 1 MiB region (that is what
        // this MMU was doing before). The access flag is an advisory hint that
        // software uses for page reclamation, and this model never reclaims, so
        // not setting it cannot change a translation result.

        result.ok = true;
        result.phys_addr = phys;
        return result;
    }

    // type == 1: coarse page table.
    const u32 l2_base = l1 & 0xFFFFFC00u;
    const u32 l2_index = (va >> 12) & 0xFFu;
    const u32 l2_addr = l2_base | (l2_index << 2);
    const u32 l2 = read_table(l2_addr);
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
    const u32 subpage = large ? ((va >> 14) & 3u) : 0u;

    u32 ap_field;
    u32 tex;
    u32 c;
    u32 b;
    u32 xn;
    u32 phys;

    if (large) {
        // Large page: 64 KiB, 4 x 16 KiB subpages.
        ap_field = (l2 >> 4) & 3u;
        tex = (l2 >> 6) & 7u;
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
        xn = (l2 >> 15) & 1u;
        phys = (l2 & 0xFFFFF000u) | (va & 0x00000FFFu);
    }

    const bool allowed = large ? check_large_ap(ap_field, subpage, mode, write)
                               : check_ap(ap_field, 0u, mode, write, true);
    if (!manager2 && !allowed) {
        ZLB_LOG_DBG("mmu", "permission fault (page): VA=0x%08X %s%s mode=0x%02X l2=0x%08X ap_field=0x%X",
                    va, write ? "write" : "read", fetch ? "/fetch" : "", mode & arm::kModeMask, l2,
                    ap_field);
        result.fault = arm::MmFaultKind::Permission;
        result.fsr_full = fsr_for(arm::MmFaultKind::Permission, write, domain2, true);
        result.fsr_status = result.fsr_full;
        return result;
    }

    if (fetch && !manager2 && xn != 0) {
        ZLB_LOG_DBG("mmu", "execute-never (page): VA=0x%08X l2=0x%08X", va, l2);
        result.fault = arm::MmFaultKind::Permission;
        result.fsr_full = fsr_for(arm::MmFaultKind::Permission, write, domain2, true) | (1u << 3);
        result.fsr_status = result.fsr_full;
        return result;
    }

    const u32 tex_cb = ((tex & 1u) << 3) | (c << 2) | (b << 1);
    classify_memory(tex_cb, tex, result);

    // Access flag: bit [10] for a small page descriptor, not bit [8] (bits
    // [8:6] are TEX). Not updated for the same reason as above.

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
                           (par & 1u) ? "last VA->PA =" : "last V2P faulted, status",
                           (par & 1u) ? (par & 0xFFFFF000u) : (par >> 1)));
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
