// zeliboba - boot chain orchestration.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "common/log.h"
#include "common/util.h"
#include "cpu/arm/arm_core.h"
#include "cpu/factory.h"
#include "hw/cmep.h"
#include "hw/emmc.h"
#include "hw/soc.h"
#include "hw/syscon.h"
#include "machine/bootchain.h"
#include "machine/bootkeys.h"
#include "machine/vita.h"

namespace zlb {

const char* to_string(BootStage stage) {
    switch (stage) {
        case BootStage::PowerOn: return "power-on";
        case BootStage::ArmBootRom: return "arm-bootrom";
        case BootStage::CmepFirstLoader: return "cmep-first-loader";
        case BootStage::CmepSecondLoader: return "cmep-second-loader";
        case BootStage::CmepSecureKernel: return "cmep-secure-kernel";
        case BootStage::ArmKernelBootLoader: return "arm-kernel-boot-loader";
        case BootStage::KernelEntry: return "kernel-entry";
        case BootStage::KernelRunning: return "kernel-running";
        case BootStage::Failed: return "failed";
    }
    return "?";
}

namespace {

/// Development substitutions are on unless ZLB_NO_SUBSTITUTION says otherwise.
bool substitutions_enabled_static() {
    static const bool disabled = [] {
        const char* value = std::getenv("ZLB_NO_SUBSTITUTION");
        return value != nullptr && value[0] != '0';
    }();
    return !disabled;
}

/// Round 114: the class-instance arena.  The KBL's instance base field `[obj+0x14]`
/// is never filled by the stage the model stands in for, so the getter 0x4002C1AC
/// computes `[obj+0x14] + table[size_class]` from the 0xFFFFFFFF sentinel and the
/// model must give it a real base.  The arena lives in the low kernel window above
/// the KBL's hardcoded 0x01031000 base: VA 0x01100000, mapped (dram-abs) to
/// PA 0x41100000+ - free DRAM above the KBL image (ends 0x40075B94) and the
/// partition region.
constexpr u32 kInstanceArenaVa = 0x01100000u;
constexpr u32 kInstanceArenaSize = 0x00100000u;   // 1 MiB = 256 pages
/// Tree-node arena for the region allocator rebuild (round 124): the sentinel and
/// root it needs overlap the class list, so fresh nodes are carved out of a
/// separate low-window page, mapped (dram-abs) to free DRAM above the KBL image.
constexpr u32 kTreeNodeArenaVa = 0x01200000u;
constexpr u32 kTreeNodeArenaSize = 0x00100000u;   // 1 MiB
/// Lookup-object arena (round 134): fresh pages handed to the type-0x1000B object
/// lookups in SceSysmem, mapped (dram-abs) to free DRAM above the other arenas.
constexpr u32 kLookupArenaVa = 0x01300000u;
constexpr u32 kLookupArenaSize = 0x00100000u;     // 1 MiB

/// Install an L1 section descriptor for `va` (1 MiB-aligned) once, so direct
/// translations (which do not consult the fault hook) see it as writable.  Uses
/// the same section attributes the KBL itself puts on its DRAM entries.
void ensure_arena_section(ArmCore* arm, Bus* bus, u32 va) {
    if (arm == nullptr || bus == nullptr) return;
    const u32 l1_base = arm->mmu.ttbr0 & 0xFFFFC000u;
    const u32 l1_slot = l1_base + ((va >> 20) & 0xFFFu) * 4u;
    if ((bus->read32(l1_slot) & 3u) == 0u) {
        const u32 section_pa = (0x40000000u + va) & 0xFFF00000u;
        bus->write32(l1_slot, section_pa | 0x1158Eu);
    }
}

void ensure_instance_arena(ArmCore* arm, Bus* bus) {
    ensure_arena_section(arm, bus, kInstanceArenaVa);
}

/// How long the CMeP may keep running after the secure kernel's "done" jump while
/// its second loader finishes the ARM boot context (see poll_boot_chain).
constexpr u64 kCmepFinishBudget = 2000000;

/// Read the whole SLB2 container: the eMMC boot partition first (that is where
/// the hardware looks at power-on), then the two "bls" copies in the user area
/// that a reconstructed image carries as well.
bool read_slb2_container(EmmcCard& card, std::vector<u8>& out) {
    if (!card.attached()) return false;

    // Read enough of the boot area to cover the whole container: the 1.04 SLB2
    // spans about 0xA0000 bytes (its last entry starts at 0x9D400).
    const size_t probe = 2 * 1024 * 1024;
    std::vector<u8> buffer(probe, 0);

    for (EmmcPartition partition : {EmmcPartition::Boot0, EmmcPartition::Boot1}) {
        if (card.read_bytes(partition, 0, buffer.data(), buffer.size()) &&
            std::memcmp(buffer.data(), "SLB2", 4) == 0) {
            out = buffer;
            return true;
        }
    }

    // User area copies (0x800000 / 0xC00000) as produced by the image builder.
    for (u64 offset : {0x00000000ull, 0x00800000ull, 0x00C00000ull}) {
        if (card.read_bytes(EmmcPartition::User, offset, buffer.data(), buffer.size()) &&
            std::memcmp(buffer.data(), "SLB2", 4) == 0) {
            out = std::move(buffer);
            return true;
        }
    }
    return false;
}

const Slb2Entry* find_entry(const Slb2Image& image, const std::string& name) {
    for (const auto& entry : image.entries) {
        if (entry.name == name) return &entry;
    }
    // Accept the "_" variant written by some firmware versions.
    for (const auto& entry : image.entries) {
        if (entry.name == name + "_") return &entry;
    }
    return nullptr;
}

}  // namespace

// ---------------------------------------------------------------------------
// ARM boot ROM model
// ---------------------------------------------------------------------------

bool Vita::stage_in_dram(u32 address, const std::vector<u8>& bytes) {
    // Both buses share the DRAM backing store, so either one can take the write;
    // go through the CMeP because that is the side that consumes the image.
    for (size_t i = 0; i < bytes.size(); ++i) cmep_bus_->write8(address + static_cast<u32>(i), bytes[i]);
    return true;
}

bool Vita::arm_boot_rom_stage_second_loader() {
    if (!emmc_) return false;

    std::vector<u8> container;
    std::string source;

    if (read_slb2_container(*emmc_, container)) {
        source = "eMMC SLB2";
    } else {
        // Fall back to the file extracted from the 1.04 PUP.
        std::string path = resolve_workspace_path("Vita_104_Firmware/Out/PUP_dec/boot_slb2-00.pkg.seg02");
        if (auto raw = read_file(path)) {
            container = *raw;
            source = path_filename(path);
        } else {
            path = resolve_workspace_path("Vita_104_Firmware/Out/SLB2/second_loader.enp");
            if (auto raw = read_file(path)) {
                // No container: stage the bare image.
                std::vector<u8> staged = *raw;
                if (config_.provision_keys) {
                    std::string why;
                    provision_boot_keys(*cmep_bus_, staged, why);
                }
                stage_in_dram(board::kSecondLoaderStagingDram, staged);
                second_loader_pa_ = board::kSecondLoaderStagingDram;
                cmep_block_->set_arm_to_cmep_command(second_loader_pa_ | 1u);
                boot_.detail = "staged bare second_loader.enp in DRAM";
                add_milestone("ARM boot ROM staged second_loader.enp in DRAM (bare file)");
                return true;
            }
            ZLB_LOG_WARN("machine", "no SLB2 container and no second_loader.enp available");
            return false;
        }
    }

    auto slb2 = parse_slb2(container);
    if (!slb2) {
        ZLB_LOG_ERROR("machine", "SLB2 parse failed for %s", source.c_str());
        return false;
    }
    ZLB_LOG_INFO("machine", "SLB2 from %s: %zu entries", source.c_str(), slb2->entries.size());
    for (const auto& entry : slb2->entries) {
        ZLB_LOG_INFO("boot", "  SLB2 entry %-24s offset=0x%X size=%zu", entry.name.c_str(), entry.offset,
                     entry.data.size());
    }

    // The CMeP first loader expects the *encrypted* variant: 0x5CBCC hands
    // image+0x2C0 to the Bigmac (keyring slot 10 = the fused boot key, see
    // CmepBlock::seed_boot_keyring) and the plaintext it produces is the image the
    // loader then jumps to at 0x40000.  The .enp entry stays as a fallback for a
    // card that only carries the decrypted container.
    const Slb2Entry* second = find_entry(*slb2, "second_loader.enc");
    if (!second || second->data.empty()) second = find_entry(*slb2, "second_loader.enp");
    if (!second || second->data.empty()) {
        ZLB_LOG_ERROR("machine", "SLB2 does not contain a usable second_loader container");
        return false;
    }
    if (second->data.size() > kermit::kScuSize) {
        ZLB_LOG_ERROR("machine", "second loader is %zu bytes, larger than the modelled DRAM", second->data.size());
        return false;
    }

    // Provision the emulated CMeP key table and re-sign the payload so the first
    // loader's own RSA verification can succeed (see machine/bootkeys.h).
    std::vector<u8> staged = second->data;
    if (config_.provision_keys) {
        std::string why;
        if (provision_boot_keys(*cmep_bus_, staged, why)) {
            add_milestone("CMeP boot keys provisioned (development key table at 0xE0066000)");
        } else {
            ZLB_LOG_WARN("boot", "boot key provisioning skipped: %s", why.c_str());
        }
    }

    stage_in_dram(board::kSecondLoaderStagingDram, staged);
    second_loader_pa_ = board::kSecondLoaderStagingDram;
    cmep_block_->set_arm_to_cmep_command(second_loader_pa_ | 1u);

    // The second loader also stages the two kernel modules the secure/non-secure
    // boot loaders hand to the kernel (wiki: "the kprx_auth_sm.self and
    // prog_rvk.srvk read from the eMMC SLB2 partition are both loaded into DRAM");
    // their paddrs/sizes go into SceKblParam (0x90/0x98).
    if (const Slb2Entry* kprx = find_entry(*slb2, "kprx_auth_sm.self"); kprx && !kprx->data.empty()) {
        stage_in_dram(board::kKprxAuthSmStaging, kprx->data);
        kprx_auth_sm_pa_ = board::kKprxAuthSmStaging;
        kprx_auth_sm_size_ = static_cast<u32>(kprx->data.size());
    }
    if (const Slb2Entry* rvk = find_entry(*slb2, "prog_rvk.srvk"); rvk && !rvk->data.empty()) {
        stage_in_dram(board::kProgRvkStaging, rvk->data);
        prog_rvk_pa_ = board::kProgRvkStaging;
        prog_rvk_size_ = static_cast<u32>(rvk->data.size());
    }

    add_milestone("ARM boot ROM staged " + second->name + " at 0x" + hex(second_loader_pa_, 8) + " (" +
                  std::to_string(second->data.size()) + " bytes)");
    boot_.detail = "second loader staged in DRAM";
    ZLB_LOG_INFO("machine", "mailbox 0xE0000010 <- 0x%08X (image present)", second_loader_pa_ | 1u);
    return true;
}

// ---------------------------------------------------------------------------
// Stage transitions
// ---------------------------------------------------------------------------

bool Vita::load_second_loader_direct() {
    std::string path = resolve_workspace_path("Vita_104_Firmware/Out/SLB2_dec/second_loader.bin");
    auto data = read_file(path);
    if (!data) {
        ZLB_LOG_ERROR("machine", "decrypted second loader not found: %s", path.c_str());
        return false;
    }
    if (!cmep_bus_->load(board::kSecondLoaderStaging, data->data(), data->size(), "second_loader")) {
        ZLB_LOG_ERROR("machine", "could not map the second loader at 0x40000");
        return false;
    }
    second_loader_entry_ = board::kSecondLoaderStaging;
    cmep_->reset(second_loader_entry_);
    cmep_->halted = false;
    boot_.stage = BootStage::CmepSecondLoader;
    boot_.detail = "second loader loaded directly (bypassing the first loader crypto chain)";
    add_milestone("CMeP second loader loaded directly at 0x00040000 (" + std::to_string(data->size()) + " bytes)");
    return true;
}

bool Vita::load_cmep_secure_kernel() {
    // The second loader reads secure_kernel.enp from the SLB2 block and jumps into
    // it once its per-console SCE validation chain has passed (docs/BOOT.md 4).
    // This stage loads the decrypted payload (a MeP image) so the next link of the
    // boot sequence can be exercised while that chain is still unsolved.
    //
    // The image is linked at 0x800000, not at the 0x40000 window: its first words
    // are a vector table of absolute jumps (`jmp 0x800100`, `jmp 0x80028C`, ...),
    // which is the CMeP's private 2 MiB window (region `CMeP.Private`).  Staging it
    // at 0x40000 made the CMeP run straight into unmapped space.
    std::string path = resolve_workspace_path("Vita_104_Firmware/Out/SLB2_dec/secure_kernel.bin");
    auto data = read_file(path);
    if (!data) {
        ZLB_LOG_ERROR("machine", "decrypted secure kernel not found: %s", path.c_str());
        return false;
    }
    if (!cmep_bus_->load(board::kCmepSecureKernelBase, data->data(), data->size(), "secure_kernel")) {
        ZLB_LOG_ERROR("machine", "could not map the secure kernel at 0x%08X", board::kCmepSecureKernelBase);
        return false;
    }
    cmep_->reset(board::kCmepSecureKernelBase);
    cmep_->halted = false;
    secure_kernel_active_ = true;
    secure_kernel_size_ = static_cast<u32>(data->size());
    boot_.stage = BootStage::CmepSecureKernel;
    boot_.detail = format("CMeP secure kernel loaded directly at 0x%08X", board::kCmepSecureKernelBase);
    add_milestone("CMeP secure kernel loaded directly at 0x" + hex(board::kCmepSecureKernelBase, 8) + " (" +
                  std::to_string(data->size()) + " bytes)");
    return true;
}

// ---------------------------------------------------------------------------
// The low window kernel_boot_loader expects to inherit (development substitution)
// ---------------------------------------------------------------------------

bool Vita::satisfy_arm_boot_fault(u32 core, u32 va, bool write, bool fetch) {
    // The stage that runs before kernel_boot_loader - the ARM boot ROM together
    // with the 0xC0-byte "SKBL Reset Vector" the second loader writes to
    // PA 0x40000000 (wiki Boot_Sequence and the FW 3.60 Secure DRAM layout) - is
    // what maps the ARM's low window.  None of it is in the dumps, and the table
    // the KBL builds for itself covers only VA 0x0000-0x7FFF (its DRAM copy of the
    // first 32 KiB), so the first access to the low window (VA 0x40000, the MeP
    // boot mirror) and the fetch of its own vector page (VBAR 0x16100) fault.
    // Reproduce the inherited mapping - identity for VA < 1 MiB, which is where
    // the ARM's boot aliases live - by adding the missing entry to the KBL's own
    // L2 and retrying the access.
    static const bool disabled = [] {
        const char* value = std::getenv("ZLB_NO_SUBSTITUTION");
        return value != nullptr && value[0] != '0';
    }();
    if (disabled) return false;
    if (core >= static_cast<u32>(kArmCoreCount)) return false;

    // Diagnostic (round 112): every ARM translation the model cannot fix is logged in
    // order with the pc that asked for it.  The KBL's own abort handlers only report
    // a code (`0x40021998+`), so the faulting address and the faulting instruction
    // have to be captured here.  ZLB_KBL_FAULT_TRACE=1 enables it, capped at 24 lines
    // (a faulting instruction inside a retry loop would otherwise flood the log).
    static const bool fault_trace = [] {
        return std::getenv("ZLB_KBL_FAULT_TRACE") != nullptr;
    }();
    if (fault_trace && fault_trace_hits_ < 24u) {
        ++fault_trace_hits_;
        Cpu* cpu = arm_cores_[core].get();
        ArmCore* arm = dynamic_cast<ArmCore*>(cpu);
        u64 lr = 0;
        if (arm != nullptr) arm->get_register("r14", lr);
        ZLB_LOG_INFO("machine",
                     "ARM fault arm%u: VA=0x%08X %s pc=0x%08X lr=0x%08X (%s) [%u]",
                     core, va, fetch ? "fetch" : (write ? "write" : "read"),
                     arm != nullptr ? arm->get_pc() : 0u, static_cast<u32>(lr),
                     fetch ? "prefetch" : "data", fault_trace_hits_);
        // A fetch fault on a garbage address usually means a `pop {…,pc}` took a
        // corrupted return address off the stack, so dump the frame too: the slot
        // that must be watched is the word just below SP (the pop has already
        // updated SP when the fetch faults).
        if (arm != nullptr && fetch && fault_trace_hits_ <= 6u) {
            u64 sp = 0;
            arm->get_register("r13", sp);
            for (u32 row = 0; row < 3; ++row) {
                u32 frame[4] = {0, 0, 0, 0};
                const u32 base = static_cast<u32>(sp) - 0x10u + row * 16u;
                bool mapped = true;
                for (u32 i = 0; i < 4; ++i) {
                    u32 pa = 0;
                    std::string fault;
                    if (!arm->translate(base + i * 4u, false, false, pa, fault)) {
                        mapped = false;
                        break;
                    }
                    frame[i] = arm_bus_->read32(pa);
                }
                ZLB_LOG_INFO("machine", "ARM stack 0x%08X: %08X %08X %08X %08X%s", base, frame[0],
                             frame[1], frame[2], frame[3], mapped ? "" : "  <unmapped>");
            }
        }
    }
    // VA >= 1 MiB is handled by the section-level branch below (round 110); the
    // L2 walk after it only ever runs for the inherited low window.

    // Which physical address the low window holds is the open question here: the
    // wiki pins the SPAD32K alias at 0x0 and the "MeP boot" mirror of the CMeP SRAM
    // at 0x40000, but the KBL keeps a window record of its own - the object at
    // 0x400B2B30 has {+0x158 = 0x40118000, +0x15C = 0x00040000, +0x160 = 0x8000,
    // +0x164 = 0x8000}, i.e. "physical 0x40118000 <-> virtual 0x40000, 32 KiB" -
    // and that physical address is exactly the copy of the low window it maps at
    // VA 0x0000-0x7FFF.  The rule is selectable while this is being pinned down:
    //   identity             VA 0x00000..0xFFFFF -> PA = VA
    //   dram                 VA < 0x40000 -> PA = VA, above that PA = 0x40000000 + VA - 0x40000
    //   dram-abs             VA < 0x40000 -> PA = VA, above that PA = 0x40000000 + VA
    //   window   (default)   VA < 0x40000 -> PA = VA, above that PA = 0x40118000 + VA - 0x40000
    //
    // `window` is the one the KBL's own record asks for, and measured runs agree:
    // with it the core keeps executing and moves on (0x4002BD1C -> 0x4002B054),
    // while identity and dram-abs end in the 0x4003B724 trap and dram ends in an
    // endless free-list walk at 0x40031E8A.
    static const int low_map = [] {
        const char* value = std::getenv("ZLB_ARM_LOW_MAP");
        if (value == nullptr) return 3;
        if (std::strcmp(value, "identity") == 0) return 0;
        if (std::strcmp(value, "dram") == 0) return 1;
        if (std::strcmp(value, "dram-abs") == 0) return 2;
        return 3;
    }();

    Cpu* cpu = arm_cores_[core].get();
    ArmCore* arm = dynamic_cast<ArmCore*>(cpu);
    if (!arm || !arm->mmu.enabled()) return false;

    // Substitution (round 110): the KBL also keeps a *low* kernel memory area that
    // its own tables never map.  Measured: the class method 0x4002B3DC is called
    // from 0x4002BC4E as `f(obj, 0, size, 0x01031000)` - the base is a literal
    // (`mov.w r3,#0x1000; movt r3,#0x103` at 0x4002BC42) - and its first store is
    // `str r4,[r6,#4]` at 0x4002B422 with r6 = 0x01040000, so the first access to
    // VA 0x01040000 raises a *section translation fault* (DFSR = 0x805,
    // DFAR = 0x01040004), the KBL's own abort handler reports fatal code 0x8A and
    // hangs at 0x400219A0.  VA 0x01000000 lives in the TTBR0 range, and the KBL's
    // L1 entry for it (0x40108040) is not a table, so at 1 MiB granularity there is
    // nothing to patch - install a section descriptor for the whole 1 MiB, mapping
    // the VA into DRAM with the same `dram-abs` rule the low window uses
    // (PA = 0x40000000 + VA, i.e. 0x41031000 - free DRAM: the KBL image ends at
    // 0x40075B94 and the partition region at 0x40300000).  The attribute bits are
    // the KBL's own section attributes (0x1158E, the value in its TTBR1 entries).
    // ZLB_NO_SUBSTITUTION=1 disables it.
    if (va >= 0x00100000u && va < 0x40000000u) {
        const u32 l1_base = arm->mmu.ttbr0 & 0xFFFFC000u;
        const u32 l1_slot = l1_base + ((va >> 20) & 0xFFFu) * 4u;
        const u32 l1_desc = arm_bus_->read32(l1_slot);
        const u32 pa = 0x40000000u + va;
        // Round 141: after round 110 installs the 1 MiB section, the KBL replaces it
        // with its own coarse page table (L1 = 0x...01, typically L2 base PA 0 - the
        // CMeP scratch mirror).  The section branch then bails out on the "already
        // described" check and the access faults again.  Patch the L2 entry instead,
        // mirroring the low-window L2 patch below with the same dram-abs rule.
        if ((l1_desc & 3u) == 1u) {
            const u32 l2_base = l1_desc & 0xFFFFFC00u;
            const u32 l2_index = (va >> 12) & 0xFFu;
            const u32 slot = l2_base + l2_index * 4u;
            if ((arm_bus_->read32(slot) & 3u) != 0u) return false;   // already mapped
            u32 attributes = arm_bus_->read32(l2_base) & 0xFFFu;
            if ((attributes & 3u) != 2u) attributes = 0x47Eu;        // small page, AP=11
            arm_bus_->write32(slot, (pa & 0xFFFFF000u) | attributes);
            ++boot_fault_fixes_;
            ZLB_LOG_INFO("machine",
                         "low kernel L2 mapping supplied: arm%u VA 0x%08X -> PA 0x%08X (attr 0x%03X) in L2 0x%08X[0x%02X] (%s)",
                         core, va, pa, attributes, l2_base, l2_index,
                         fetch ? "fetch" : (write ? "write" : "read"));
            add_milestone("ARM low kernel L2 mapping supplied for VA 0x" + hex(va, 8) +
                          " (development substitution)");
            return true;
        }
        if ((l1_desc & 3u) != 0u) return false;   // already described (section)
        arm_bus_->write32(l1_slot, (pa & 0xFFF00000u) | 0x1158Eu);
        ++boot_fault_fixes_;
        ZLB_LOG_INFO("machine",
                     "low kernel memory mapping supplied: arm%u VA 0x%08X -> PA 0x%08X in L1 0x%08X[0x%03X] (%s)",
                     core, va, pa, l1_base, (va >> 20) & 0xFFFu,
                     fetch ? "fetch" : (write ? "write" : "read"));
        add_milestone("ARM low kernel memory mapping supplied for VA 0x" + hex(va, 8) +
                      " (development substitution)");
        return true;
    }

    // Substitution (round 127): with TTBCR=2, VA >= 0x40000000 walks TTBR1, and the
    // KBL only ever maps section 0x40000000 (its image) there.  Sections 0x40100000
    // and 0x40200000 (the rest of its 3 MiB DRAM region) stay unmapped, so its
    // page-clearing loop faults on 0x402F???? (`write section translation fault`).
    // Supply the missing sections with the KBL's own attributes (0x1158E, the value
    // in TTBR1 L1[0x400] = 0x4001158E) using the dram-abs rule (PA = VA).  The 3 MiB
    // bound matches the KBL's hardcoded heap region {0x40000000, 0x300000}.
    if (va >= 0x40000000u && va < 0x40300000u) {
        const u32 l1_base = arm->mmu.ttbr1 & 0xFFFFC000u;
        const u32 l1_slot = l1_base + ((va >> 20) & 0xFFFu) * 4u;
        if ((arm_bus_->read32(l1_slot) & 3u) != 0u) return false;   // already described
        arm_bus_->write32(l1_slot, (va & 0xFFF00000u) | 0x1158Eu);
        ++boot_fault_fixes_;
        ZLB_LOG_INFO("machine",
                     "TTBR1 DRAM section supplied: arm%u VA 0x%08X -> PA 0x%08X in L1 0x%08X[0x%03X] (%s)",
                     core, va, va & 0xFFF00000u, l1_base, (va >> 20) & 0xFFFu,
                     fetch ? "fetch" : (write ? "write" : "read"));
        add_milestone("ARM TTBR1 DRAM section supplied for VA 0x" + hex(va, 8) +
                      " (development substitution)");
        return true;
    }

    // Walk the core's own TTBR0: L1 -> coarse L2, the layout the KBL installs.
    if (va >= 0x00100000u) return false;   // only the low window reaches the L2 walk
    const u32 l1_base = arm->mmu.ttbr0 & 0xFFFFC000u;
    const u32 l1_entry = arm_bus_->read32(l1_base + ((va >> 20) & 0xFFFu) * 4u);
    if ((l1_entry & 3u) != 1u) return false;                    // not a coarse table
    const u32 l2_base = l1_entry & 0xFFFFFC00u;
    const u32 l2_index = (va >> 12) & 0xFFu;
    const u32 slot = l2_base + l2_index * 4u;
    if (arm_bus_->read32(slot) != 0u) return false;             // already mapped

    u32 pa = va;
    if (low_map == 1 && va >= 0x40000u) pa = 0x40000000u + (va - 0x40000u);
    else if (low_map == 2 && va >= 0x40000u) pa = 0x40000000u + va;
    else if (low_map == 3 && va >= 0x40000u) pa = 0x40118000u + (va - 0x40000u);

    // Take the attribute bits from the KBL's own first entry so the substituted
    // page has the same cacheability/permissions as the pages it installed.
    u32 attributes = arm_bus_->read32(l2_base) & 0xFFFu;
    if ((attributes & 3u) != 2u) attributes = 0x47Eu;           // small page, AP=11
    arm_bus_->write32(slot, (pa & 0xFFFFF000u) | attributes);

    ++boot_fault_fixes_;
    ZLB_LOG_INFO("machine",
                 "boot window mapping supplied: arm%u VA 0x%08X -> PA 0x%08X (attr 0x%03X) in L2 0x%08X[0x%02X] (%s)",
                 core, va, pa, attributes, l2_base, l2_index, fetch ? "fetch" : (write ? "write" : "read"));
    add_milestone("ARM low window mapping supplied for VA 0x" + hex(va, 8) + " (development substitution)");
    return true;
}

// Development substitution experiment for the partition allocator's cache-only flag
// check - **off by default**, enabled with ZLB_ALLOC_CARVE=1.
//
// kernel_boot_loader builds its physical memory partition itself (VA 0x51C0: region
// {0x40000000, 3 MiB}, a per-core cache of 0x1000-byte block pointers and free lists
// per size class), but nothing in the KBL ever *adds* memory to it, and every
// allocation site asks for flag 0x10, which the allocator treats as "must come from
// the cache": `bics r0, r11, #0x21` at 0x40032356 returns 0x80020005 instead of
// carving a fresh block.  Measured over 230M instructions the cache is written
// exactly 37 times, all of them part of its own initialisation, so every allocation
// fails, the object manager at boot context +0x8C is never created and all 24 class
// registrations return 0x80024501 (docs/KBL.md, rounds 50-51).
//
// The experiment redirects the PC from the failure path (0x4003235C) into the
// allocator's own carving path (0x40032366), i.e. it treats flag 0x10 as
// carve-capable.  Result: each of the four cores ends up spinning in the partition
// mutex (`0x4003A1B0`, the word at partition+0x0E, which is already non-zero), so the
// KBL gets *stuck* instead of failing cleanly - the missing memory really has to
// arrive pre-populated from the stage before the KBL, and that is what to model next.
// Disabled with ZLB_NO_SUBSTITUTION=1 (which wins over ZLB_ALLOC_CARVE).
bool Vita::satisfy_arm_boot_pc(u32 core, u32 pc) {
    // Previous instruction address of this core, for the error-path diagnostics
    // below: the KBL's shared panic stub is entered by a *branch* from one of ~36
    // sites, so the pc that led there is the only way to name the failing check.
    u32 previous_pc = 0;
    if (core < static_cast<u32>(kArmCoreCount)) {
        previous_pc = last_arm_pc_[core];
        last_arm_pc_[core] = pc;
    }
    // Substitution (round 117): make the object manager lock (0x601C = 0x5F80+0x9c)
    // a no-op.  Round 116 measured that only arm0 ever acquires it (20000 balanced
    // acquire/release, zero non-arm0 entries), and the four-core deadlock is arm0
    // re-entering that same lock inside its own critical section
    // (0x4002B3E0 -> 0x4002B1C8 -> ... -> 0x4002B3E0), which the non-recursive
    // spinlock cannot satisfy.  On hardware the object manager arrives pre-populated
    // from the secure world, so this re-entrant path never runs.  With no other core
    // contending for 0x601C, skipping the acquire is safe: the lock word stays 0 and
    // the unlock (0x4003A224) is a no-op too.  ZLB_NO_SUBSTITUTION=1 disables it;
    // ZLB_KBL_OBJMGR_NOLOCK=0 disables just this substitution.
    static const bool objmgr_nolock = [] {
        const char* off = std::getenv("ZLB_NO_SUBSTITUTION");
        if (off != nullptr && off[0] != '0') return false;
        const char* on = std::getenv("ZLB_KBL_OBJMGR_NOLOCK");
        return on == nullptr || on[0] != '0';
    }();
    if (objmgr_nolock && pc == 0x4003A1B0u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 lock = 0;
                if (arm->get_register("r0", lock) && static_cast<u32>(lock) == 0x601Cu) {
                    arm->set_register("r0", 0u);       // interrupt mask (I/F were 0)
                    arm->set_pc(0x4003A1ECu);          // bx lr - skip the acquire
                    ++boot_pc_fixes_;
                    return true;                       // handled: do not execute the lock
                }
            }
        }
    }
    // Substitution (round 119): the same trick for the SceUID/class-registration
    // lock 0x12008 (object 0x12000 + 0x8).  Round 119 measured a single acquire from
    // arm0 and no release - the word is already 0x80000000 (held) before the first
    // acquisition, so the non-recursive spinlock 0x4003A28C spins forever.  arm0 is
    // the only core that touches it, so skipping the acquire is safe; its unlock
    // (0x4003A300) then writes 0 and leaves the word free.  Gated by the same
    // ZLB_KBL_OBJMGR_NOLOCK / ZLB_NO_SUBSTITUTION flags.
    if (objmgr_nolock && pc == 0x4003A28Cu) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 lock = 0;
                if (arm->get_register("r0", lock) && static_cast<u32>(lock) == 0x12008u) {
                    arm->set_register("r0", 0u);       // interrupt mask
                    arm->set_pc(0x4003A2C8u);          // bx lr - skip the acquire
                    ++boot_pc_fixes_;
                    return true;
                }
            }
        }
    }
    // Substitution (round 136): the per-core rendezvous 0x4003B34C waits on
    // `[obj+6] == core-id` with wfe (the master writes the field via the exclusive
    // store 0x4003A3EC, the slaves spin in 0x4003B366).  After round 135 the master
    // core is stuck at the four-core barrier, so the slaves wait forever.  Make the
    // rendezvous a no-op: the loader's per-core setup is idempotent in this model
    // (cores run round-robin, not truly in parallel), so no rendezvous is needed.
    // Gated by the same ZLB_KBL_OBJMGR_NOLOCK / ZLB_NO_SUBSTITUTION flags.
    if (objmgr_nolock && pc == 0x4003B34Cu) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                arm->set_pc(arm->r[14] & ~1u);          // bx lr - skip the rendezvous
                arm->set_register("THUMB", 1u);
                ++boot_pc_fixes_;
                return true;
            }
        }
    }
    // Substitution (round 136): the KBL's halfword exclusive store helper 0x4003A3EC
    // (ldrexh/strexh) livelocks under the round-robin schedule (the reservation is
    // cleared between the load and store), so the barrier never writes its counter.
    // Make it a plain non-exclusive 16-bit store of r1 to [r0]: with no true
    // parallel execution the exclusivity is unnecessary.  Same gate as the other
    // round-136 substitutions.
    if (objmgr_nolock && pc == 0x4003A3ECu) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u32 pa = 0;
                std::string fault;
                if (arm->translate(arm->r[0], true, false, pa, fault)) {
                    arm_bus_->write16(pa, static_cast<u16>(arm->r[1] & 0xFFFFu));
                    arm->set_pc(arm->r[14] & ~1u);      // bx lr
                    arm->set_register("THUMB", 1u);
                    ++boot_pc_fixes_;
                    return true;
                }
            }
        }
    }
    // Substitution (round 136): the four-core barrier's arrive step is an exclusive
    // halfword decrement (0x4003A41C: ldrexh/sub/strexh).  Under the model's
    // round-robin core schedule the reservation keeps being cleared, so the counter
    // [obj+4] stays at 4 and every core sits in the leave wait (0x4003B3D2).  Do the
    // decrement non-exclusively at the call site 0x4003B3A6 (r6 = &[obj+4], r1 = 1)
    // and skip the helper, returning the old value in r0 as the helper would.
    // Gated by the same ZLB_KBL_OBJMGR_NOLOCK / ZLB_NO_SUBSTITUTION flags.
    if (objmgr_nolock && pc == 0x4003B3A6u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                const u32 counter_va = arm->r[6];
                u32 pa = 0;
                std::string fault;
                if (arm->translate(counter_va, true, false, pa, fault)) {
                    const u32 old = arm_bus_->read16(pa);
                    const u32 value = (old - 1u) & 0xFFFFu;
                    arm_bus_->write16(pa, static_cast<u16>(value));
                    arm->set_register("r0", old);
                    arm->set_pc(0x4003B3AAu);          // skip blx 0x4003A41C
                    barrier_old_[core] = static_cast<u16>(old & 0xFFFFu);
                    ++boot_pc_fixes_;
                    ++barrier_decrements_;
                    if (barrier_decrements_ <= 64) {
                        ZLB_LOG_INFO("machine", "barrier decrement arm%u: [0x%08X] %u -> %u",
                                     core, counter_va, old, value);
                    }
                    return true;
                }
            }
        }
    }
    // Substitution (round 140): the four-core barrier saves its "old" arrival value
    // on the stack (`strh.w r1,[sp,#6]` at 0x4003B3AE) and re-reads it at 0x4003B3BA
    // to pick phase 1/phase 2.  The secondary cores still run on the shared initial
    // stack (SP 0x3F00) here, so that halfword slot is clobbered between cores and
    // the leader (old == 4) reads a follower's value, drops into the follower's
    // phase-1 wait and never resets the counter - the whole barrier deadlocks with
    // all four cores in the leave wait.  Replay the per-core value recorded above.
    if (objmgr_nolock && pc == 0x4003B3BAu) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                arm->set_register("r3", static_cast<u32>(barrier_old_[core]));
                arm->set_pc(0x4003B3BEu);          // skip ldrh.w r3, [sp, #6]
                arm->set_register("THUMB", 1u);
                ++boot_pc_fixes_;
                return true;
            }
        }
    }
    // Experiment (round 150): the region/partition method 0x40033FC8 dereferences
    // [this+40] as an allocator pointer, but the class list it aliases is not fully
    // registered (the field holds flags == 2), so the walk hits the class-name string
    // "edHe" and faults.  Return success without doing the work so the caller moves
    // on; gated by ZLB_KBL_ALLOC_NOOP=1 for easy toggling.
    static const bool alloc_noop = [] {
        const char* on = std::getenv("ZLB_KBL_ALLOC_NOOP");
        return on != nullptr && on[0] != '0';
    }();
    if (alloc_noop && pc == 0x40033FC8u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                arm->set_register("r0", 0u);
                arm->set_pc(arm->r[14] & ~1u);
                arm->set_register("THUMB", 1u);
                ++boot_pc_fixes_;
                return true;
            }
        }
    }
    // Round 94/95 measurements (docs/KBL.md round 95): every allocation the KBL
    // makes for itself calls the partition allocator 0x40032278 with flag 0x10.
    //   * size == 0x1000 is the *only* size served from the per-core page cache
    //     (`cmp.w r1,#0x1000; bne <carve>` at 0x40032278), whose slot is
    //     {u16 target, u16 count, u32 head, u32 extra0, u32 extra1} at
    //     `[partition+0x9C] + core*0x20`; the cache is only ever *filled* by the
    //     "return block" path 0x40032768, so it starts empty;
    //   * any other power-of-two size goes to the carving path 0x40032366, which
    //     needs a class marker 0x00010002 in `partition+0x38+8*class` plus a
    //     matching page entry in the partition's page table at `[partition+0x20]`
    //     (entry = state<<28 | class<<20 | pages, states 0x10000000/0x20000000
    //     are the ones it accepts) - that is the "memory the previous stage
    //     leaves behind" the round-82 analysis predicted, now pinned to a
    //     structure and a consumer.
    // The substitution therefore seeds the page cache for class 0x1000 (which is
    // what the object manager's own block is) at the allocator entry, so the
    // loader's *own* cache path (0x4003228C..0x400322F4) hands the blocks out and
    // keeps its counters consistent.  ZLB_NO_SUBSTITUTION=1 disables it.
    constexpr u32 kPartitionAllocEntry = 0x40032278u;
    constexpr u32 kInstanceAllocPc = 0x40031BE4u;    // bl 0x40032278 (0x2000)
    constexpr u32 kManagerAllocPc = 0x4002BB62u;     // bl 0x40032278 (0x1000)

    // Experiment (ZLB_PART_ALLOC_CARVE=1): same call sites, but the cache-only
    // flag is cleared so the allocator takes its own carving path (0x40032366).
    // Result (docs/KBL.md rounds 72/95): the carve path does *not* succeed either -
    // the class markers are empty, so it walks off the end and reports 0x80020005.
    static const bool carve_experiment = [] {
        const char* value = std::getenv("ZLB_NO_SUBSTITUTION");
        if (value != nullptr && value[0] != '0') return false;
        const char* carve = std::getenv("ZLB_PART_ALLOC_CARVE");
        return carve != nullptr && carve[0] != '0';
    }();
    if (carve_experiment && (pc == kInstanceAllocPc || pc == kManagerAllocPc)) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            Cpu* cpu = arm_cores_[core].get();
            if (ArmCore* arm = dynamic_cast<ArmCore*>(cpu)) {
                u64 flags = 0;
                if (arm->get_register("r2", flags)) {
                    arm->set_register("r2", flags & ~0x10ull);
                    ++boot_pc_fixes_;
                    if (boot_pc_fixes_ <= 2) {
                        ZLB_LOG_INFO("machine",
                                     "partition allocator: flag 0x10 -> 0x%llX at pc=0x%08X "
                                     "(ZLB_PART_ALLOC_CARVE experiment)",
                                     static_cast<unsigned long long>(flags & ~0x10ull), pc);
                    }
                }
            }
        }
        return false;    // the instruction still has to run
    }

    // Substitution (on by default, off with ZLB_NO_SUBSTITUTION=1): the allocator is
    // about to run for the class whose page cache is empty.
    static const bool supply_blocks = [] {
        const char* value = std::getenv("ZLB_NO_SUBSTITUTION");
        return value == nullptr || value[0] == '0';
    }();
    if (supply_blocks && (pc == kPartitionAllocEntry || pc == kInstanceAllocPc ||
                          pc == kManagerAllocPc)) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 size = 0;
                u64 pool = 0x51C0u;
                arm->get_register("r1", size);
                if (pc == kPartitionAllocEntry) arm->get_register("r0", pool);
                supply_kbl_partition_block(core, static_cast<u32>(pool), static_cast<u32>(size));
            }
        }
        return false;    // fall through to the instruction itself
    }

    // Substitution (round 93): the SceUID registration at 0x4002BD00 walks a
    // per-class table through the global 0x400B291C
    //   0x4002BFAC  movwls r2, #0x291c / movtls r2, #0x400b ; r2 = 0x400B291C
    //   0x4002BFB8  ldrls  r2, [r2]                        ; table base
    //   0x4002BFBC  addls.w sb, r2, r3, lsl #2             ; r3 = class*5
    //   0x4002BFC2  blx    0x4003A28C                      ; lock(&table[class])
    // and that global is *never* written: a write trap over the whole run sees only
    // the BSS zero-fill (pc 0x400211E0) at 0x400B291C, so the table base stays 0,
    // the lock call gets a NULL pointer and the KBL spins in the second spinlock
    // (pc 0x4003A2B8, R0 = 0, LR = 0x4002BFC7).  The model creates it: an 8-entry,
    // 0x14-byte-per-entry table in a writable page it allocates for itself, stored
    // into the global before the registration loop runs.  ZLB_NO_SUBSTITUTION=1
    // disables it.
    constexpr u32 kClassTableGlobal = 0x400B291Cu;
    constexpr u32 kSystemInitPc = 0x4002BB32u;   // `str r1,[r5]` in the manager builder
    // Round 110: with the memory walls behind it the KBL now runs the *whole* manager
    // builder and zeroes this field itself (`str r1,[r4,#0x1C]` at 0x4002ADB8, where
    // r4 = 0x400B2900 and +0x1C = the very slot our substitution fills), so the two
    // writes race and the registration loop ends up locking a NULL pointer
    // (pc 0x4003A28C, R0 = 0, LR = 0x4002BFC7).  The substitution can therefore be
    // switched off with ZLB_KBL_CLASS_TABLE=0 to see whether the loader builds the
    // table on its own now.
    static const bool supply_class_table = [] {
        const char* value = std::getenv("ZLB_KBL_CLASS_TABLE");
        return value == nullptr || value[0] != '0';
    }();
    if (supply_blocks && supply_class_table && pc == kSystemInitPc) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                supply_kbl_class_table(core);
            }
        }
        return false;    // the instruction still has to run
    }

    // Round 111: the loader's own class-descriptor loop overwrites the slot *after*
    // the substitution above ran.  The trace
    //     ZLB_KBL_TRACE_PC=0x4002ADB8  ->  from pc=0x4002ADB6 lr=0x4002B153,
    //                                      r4 = 0x14000/0x14020/0x14040/0x14060,
    //                                      r1 = 0
    //     ZLB_KBL_TRACE_PC=0x4002B14E  ->  r0 = the same 0x140xx, r1 = 0 (from the
    //                                      caller's *stack local* [r4+0x10])
    // shows four class descriptors (VA 0x14000 + n*0x20, i.e. PA 0x400B2900 + n*0x20)
    // whose +0x1C table pointer the loader sets to 0 because the local that should
    // carry the table pointer is zero.  Supply the table again right before the
    // registration loop reads the pointer, so the loop does not lock NULL:
    //     0x4002BFB8  ldrls r2,[r2]        ; r2 = [0x400B291C]
    //     0x4002BFBC  addls.w sb, r2, r3, lsl #2
    //     0x4002BFC2  blx 0x4003A28C       ; lock(&table[class])
    // `supply_kbl_class_table` is a no-op when the field is already non-zero, so this
    // only stands in where the loader left nothing.  ZLB_KBL_CLASS_TABLE=0 disables it.
    constexpr u32 kClassTableReadPc = 0x4002BFB8u;
    if (supply_blocks && supply_class_table && pc == kClassTableReadPc) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                // Reuse the table the first supply created: by this point the loader
                // has rebuilt its page tables, so re-deriving the region through VA
                // 0x51C0 can return garbage and the allocator inside
                // supply_kbl_class_table then probes its way into the *low* window
                // (measured: it once "supplied" a table at VA 0x0004705F and the
                // registration loop faulted).  Only the very first supply allocates.
                bool done = false;
                if (class_table_va_ != 0u) {
                    // Write the *table address* into the *pointer slot* - translating
                    // the table's own VA here would overwrite the table's first word
                    // with its own address and the registration loop would then spin
                    // on a lock value of 0x400B0000 (measured, round 111).
                    u32 pa = 0;
                    std::string fault;
                    if (arm->translate(kClassTableGlobal, true, false, pa, fault)) {
                        arm_bus_->write32(pa, class_table_va_);
                        done = true;
                        ++class_tables_supplied_;
                        if (class_tables_supplied_ <= 6) {
                            ZLB_LOG_INFO("machine",
                                         "class table restored at VA 0x%08X (global 0x%08X) "
                                         "(development substitution)",
                                         class_table_va_, kClassTableGlobal);
                        }
                    }
                }
                if (!done) supply_kbl_class_table(core);
            }
        }
        return false;    // the load still has to run
    }

    // Substitution (round 94): restore the ARM exception vectors at the mapping the
    // KBL installed.  The KBL points VBAR at VA 0x16100 and maps that VA onto its
    // own DRAM page (`vpa 0x16100` = PA 0x40000100), where its copy is incomplete
    // (two `ldr pc,[pc,#0x18]` words followed by data), so every exception entry
    // falls into zeros and the core ends up hopping through the whole vector page.
    // The model stages the real 0xC0-byte table from the KBL's ELF segment (vaddr 0)
    // the first time the core enters the vector window, at the physical address the
    // core's own tables resolve.  ZLB_NO_SUBSTITUTION=1 disables it.
    if (supply_blocks && !kbl_vectors_restored_ && !kbl_vectors_.empty() && pc >= 0x16100u &&
        pc < 0x16200u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                // The KBL's mapping of the vector page is a device page, so a *write*
                // translation is refused; resolve it read-only and put the bytes down
                // through the bus (the model owns the memory), falling back to the
                // mapping measured with `vpa 0x16100` when even that fails.
                u32 pa = 0;
                std::string fault;
                const bool mapped = arm->translate(0x00016100u, false, false, pa, fault);
                if (!mapped) pa = 0x40000100u;
                for (size_t i = 0; i < kbl_vectors_.size(); ++i) {
                    arm_bus_->write8(pa + static_cast<u32>(i), kbl_vectors_[i]);
                }
                kbl_vectors_restored_ = true;
                ZLB_LOG_INFO("machine",
                             "ARM exception vectors restored at PA 0x%08X (VA 0x16100 mapping%s, "
                             "%zu bytes) (development substitution)",
                             pa, mapped ? "" : " not readable - using the measured fallback",
                             kbl_vectors_.size());
                add_milestone("ARM exception vectors restored at PA 0x" + hex(pa, 8) +
                              " (development substitution)");
            }
        }
        return false;    // the instruction still has to run
    }

    // Substitution (round 100): the class-constructor loop (0x400316A0..0x40031820)
    // can be handed a bogus block pointer - traced in round 98/99 to the getter at
    // 0x4002C1AC, which adds the object's base field [obj+0x14] that nothing in the
    // loader writes (it is zero in the model, so the raw -1 from the caller's table
    // lookup reaches `str.w r1,[r8]` and faults).  Instead of inventing that base,
    // skip the store *and* the constructor call for such a block so the loop moves on
    // and the boot reaches the next barrier.  ZLB_NO_SUBSTITUTION=1 disables it.
    constexpr u32 kBlockStorePc = 0x4003180Cu;    // str.w r1,[r8]
    constexpr u32 kAfterBlockPc = 0x40031816u;    // ldrh r2,[r4,#0x22]
    if (supply_blocks && pc == kBlockStorePc) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 block = 0;
                if (arm->get_register("r8", block) &&
                    (block == 0xFFFFFFFFull || block < 0x1000ull)) {
                    arm->set_pc(kAfterBlockPc);
                    ++boot_pc_fixes_;
                    if (boot_pc_fixes_ <= 4) {
                        ZLB_LOG_INFO("machine",
                                     "class constructor loop: skipped a bogus block 0x%llX at pc=0x%08X "
                                     "(%s) (development substitution)",
                                     static_cast<unsigned long long>(block), pc,
                                     "[obj+0x14] base is not modelled");
                        add_milestone("KBL class constructor loop skipped a bogus block "
                                      "(development substitution)");
                    }
                    return true;   // handled: do not execute the store
                }
            }
        }
        return false;    // a valid block: run the instruction
    }

    // Diagnostic (ZLB_KBL_PANIC_TRACE=1): the check at 0x40035966 is
    //     0x40035966  ldr r1,[r4,#0x10]
    //     0x40035968  cmp r1,r5
    //     0x4003596A  bls.w 0x4003561C     ; continue when [r4+0x10] <= r5
    //     0x4003596E  bl 0x4003B724        ; else the KBL's own panic (`b .`)
    // and it is the place the boot currently dies (round 109: measured with `runm`,
    // which is the mode that honours this hook - the debugger's `run` single-steps
    // and never consults it).  Log the operands and the arena so the failing input
    // is visible without another debugging session.
    static const bool panic_trace = [] { return std::getenv("ZLB_KBL_PANIC_TRACE") != nullptr; }();
    if (panic_trace && pc == 0x4003596Eu) {        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 r0 = 0, r1 = 0, r2 = 0, r3 = 0, r4 = 0, r5 = 0, r6 = 0, r7 = 0, lr = 0, sp = 0;
                arm->get_register("r0", r0);
                arm->get_register("r1", r1);
                arm->get_register("r2", r2);
                arm->get_register("r3", r3);
                arm->get_register("r4", r4);
                arm->get_register("r5", r5);
                arm->get_register("r6", r6);
                arm->get_register("r7", r7);
                arm->get_register("r14", lr);
                arm->get_register("r13", sp);
                ZLB_LOG_INFO("machine",
                             "KBL panic arm%u: entered from pc=0x%08X lr=0x%08X sp=0x%08X "
                             "r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X r4=0x%08X r5=0x%08X "
                             "r6=0x%08X r7=0x%08X",
                             core, previous_pc, static_cast<u32>(lr), static_cast<u32>(sp),
                             static_cast<u32>(r0), static_cast<u32>(r1), static_cast<u32>(r2),
                             static_cast<u32>(r3), static_cast<u32>(r4), static_cast<u32>(r5),
                             static_cast<u32>(r6), static_cast<u32>(r7));
            }
        }
    }
    // Generic diagnostic (round 111): ZLB_KBL_TRACE_PC=<hex> logs the first few times
    // the ARM reaches that pc, together with the pc that led there and the register
    // file.  Rounds 109/110 showed this is the only way to name the branch that
    // enters one of the KBL's shared stubs (panic 0x4003596E, fatal codes 0x40021998+)
    // or a store that overwrites a pointer (0x4002ADB8).
    static const u32 trace_pc = [] {
        const char* value = std::getenv("ZLB_KBL_TRACE_PC");
        if (value == nullptr) return 0u;
        return static_cast<u32>(std::strtoul(value, nullptr, 16));
    }();
    if (trace_pc != 0u && pc == trace_pc) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                if (trace_pc_hits_ < 64u) {
                    ++trace_pc_hits_;
                    u64 r[8] = {0}, lr = 0, sp = 0;
                    for (u32 i = 0; i < 8; ++i) {
                        arm->get_register("r" + std::to_string(i), r[i]);
                    }
                    arm->get_register("r14", lr);
                    arm->get_register("r13", sp);
                    ZLB_LOG_INFO("machine",
                                 "trace pc=0x%08X arm%u from pc=0x%08X lr=0x%08X sp=0x%08X "
                                 "r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X r4=0x%08X r5=0x%08X "
                                 "r6=0x%08X r7=0x%08X",
                                 pc, core, previous_pc, static_cast<u32>(lr), static_cast<u32>(sp),
                                 static_cast<u32>(r[0]), static_cast<u32>(r[1]),
                                 static_cast<u32>(r[2]), static_cast<u32>(r[3]),
                                 static_cast<u32>(r[4]), static_cast<u32>(r[5]),
                                 static_cast<u32>(r[6]), static_cast<u32>(r[7]));
                }
            }
        }
    }

    // Diagnostic (round 114): the class constructor 0x4002A964 (`movs r1,#0` ...
    // `str.w r2,[r3,#0x14]`) is reached through `blx [r6+0x34]` with r0 = r8 = the
    // block the caller 0x40031654 is about to hand out.  Round 113 showed the fatal
    // iteration hands the constructor r0 = 0x40060F20 - the current stack frame - so
    // the +0x14 sentinel store clobbers the caller's saved LR.  Measured (round 114):
    // the per-core block cache [cls+core*0x10+0x60] stays 0 the whole run, and the
    // object instead comes from the region walk `r8 = [cls+0x3c]; r8 += stride`,
    // stride = [cls+0x20], bounded by 0x4003CCC0([cls+0x1c], stride).  This dumps
    // every constructor entry together with the region-walk state so the overrun
    // into the stack can be named.  ZLB_KBL_CTOR_TRACE=1 enables it.
    static const bool ctor_trace = [] { return std::getenv("ZLB_KBL_CTOR_TRACE") != nullptr; }();
    if (ctor_trace && pc == 0x4002A96Cu && ctor_trace_hits_ < 200u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                ++ctor_trace_hits_;
                const u32 obj = arm->r[0];
                const u32 cls = arm->r[4];
                auto rd = [&](u32 va, bool& ok) {
                    u32 pa = 0;
                    std::string fault;
                    if (!arm->translate(va, false, false, pa, fault)) {
                        ok = false;
                        return 0u;
                    }
                    ok = true;
                    return arm_bus_->read32(pa);
                };
                bool ok = true;
                const u32 base = rd(cls + 0x14u, ok);
                const u32 size = rd(cls + 0x1Cu, ok);
                const u32 cur = rd(cls + 0x3Cu, ok);
                const u32 stride = rd(cls + 0x20u, ok);
                const u32 cnt = rd(cls + 0x40u, ok);
                const u32 f38 = rd(cls + 0x38u, ok);
                const u32 klass = rd(cls + 0x04u, ok);
                const bool suspect = obj >= 0x40000000u;
                ZLB_LOG_INFO("machine",
                             "ctor arm%u #%u obj=0x%08X cls=0x%08X lr=0x%08X sp=0x%08X | "
                             "class=0x%08X base=0x%08X size=0x%08X stride=0x%08X cnt=0x%08X "
                             "cur=0x%08X f38=0x%08X%s",
                             core, ctor_trace_hits_, obj, cls, arm->r[14], arm->r[13], klass, base,
                             size, stride, cnt, cur, f38, suspect ? "  <-- DRAM/STACK OBJ" : "");
            }
        }
    }

    // Diagnostic (round 115): the ARM-mode spinlock 0x4003A1B0 loops through the
    // `ldrex [r0]` at 0x4003A1C0 (the `bne` at 0x4003A1E0 branches back there), so
    // the loop head is 0x4003A1C0, not the blx entry.  When [r0] is non-zero the
    // core spins on the `wfe` path.  Log every contended loop head (lock address +
    // held value) to name the stuck lock.  ZLB_KBL_LOCK_TRACE=1 enables it.
    static const bool lock_trace = [] { return std::getenv("ZLB_KBL_LOCK_TRACE") != nullptr; }();
    if (lock_trace && pc == 0x4003A1C0u && lock_trace_hits_ < 24u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 lock = 0;
                if (arm->get_register("r0", lock)) {
                    u32 pa = 0;
                    std::string fault;
                    if (arm->translate(static_cast<u32>(lock), false, false, pa, fault)) {
                        const u32 held = arm_bus_->read32(pa);
                        if (held != 0u) {
                            ++lock_trace_hits_;
                            ZLB_LOG_INFO("machine",
                                         "spinlock contended: arm%u lock=0x%08X held=0x%08X "
                                         "r4=0x%08X (development diagnostic)",
                                         core, static_cast<u32>(lock), held, arm->r[4]);
                        }
                    }
                }
            }
        }
    }

    // Diagnostic (round 116): log the acquire/release of the object manager lock
    // (0x601C = 0x5F80+0x9c) with the core id and the caller, so the acquire/release
    // pairing names the core that holds the lock when the four-core deadlock forms.
    // The acquire is the spinlock entry 0x4003A1B0, the release is 0x4003A224; both
    // take r0 = the lock word.  ZLB_KBL_OBJMGR_TRACE=1 enables it.
    static const bool objmgr_trace = [] { return std::getenv("ZLB_KBL_OBJMGR_TRACE") != nullptr; }();
    if (objmgr_trace && (pc == 0x4003A1B0u || pc == 0x4003A224u)) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 lock = 0;
                // arm0 owns the "allocate low kernel memory" loop (thousands of
                // balanced entries), so cap its noise; the holder is a non-arm0 core.
                if (arm->get_register("r0", lock) && static_cast<u32>(lock) == 0x601Cu &&
                    (core != 0u || objmgr_trace_hits_ < 64u)) {
                    ++objmgr_trace_hits_;
                    ZLB_LOG_INFO("machine",
                                 "objmgr %s arm%u lock=0x601C lr=0x%08X (development diagnostic)",
                                 pc == 0x4003A1B0u ? "acquire" : "release", core, arm->r[14]);
                }
            }
        }
    }

    // Diagnostic (round 119): the SceUID/class-registration lock is 0x12008
    // (object 0x12000 + 0x8), acquired through the second spinlock 0x4003A28C
    // (lock value 0x80000000) and released through 0x4003A300.  Log the
    // acquire/release pairing with the core id so the holder of the stuck lock can
    // be named.  ZLB_KBL_SCEUID_TRACE=1 enables it.
    static const bool sceuid_trace = [] { return std::getenv("ZLB_KBL_SCEUID_TRACE") != nullptr; }();
    if (sceuid_trace && (pc == 0x4003A28Cu || pc == 0x4003A300u) && sceuid_trace_hits_ < 128u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 lock = 0;
                if (arm->get_register("r0", lock) && static_cast<u32>(lock) == 0x12008u) {
                    ++sceuid_trace_hits_;
                    ZLB_LOG_INFO("machine",
                                 "sceuid %s arm%u lock=0x12008 lr=0x%08X (development diagnostic)",
                                 pc == 0x4003A28Cu ? "acquire" : "release", core, arm->r[14]);
                }
            }
        }
    }

    // Diagnostic (round 120): dump the partition region tree when the allocator
    // 0x4002EE60 is entered.  r0 = the partition object; the tree head is at
    // [obj+0x4C]+0xC and the walk follows [node+0x24] until it returns to the head.
    // ZLB_KBL_TREE_TRACE=1 enables it.
    static const bool tree_trace = [] { return std::getenv("ZLB_KBL_TREE_TRACE") != nullptr; }();
    if (tree_trace && pc == 0x4002EE60u && tree_trace_hits_ < 16u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                ++tree_trace_hits_;
                auto rd = [&](u32 va, bool& ok) {
                    u32 pa = 0;
                    std::string fault;
                    if (!arm->translate(va, false, false, pa, fault)) { ok = false; return 0u; }
                    ok = true;
                    return arm_bus_->read32(pa);
                };
                bool ok = true;
                const u32 obj = arm->r[0];
                const u32 part = rd(obj + 0x4Cu, ok);
                const u32 head = rd(part + 0xCu, ok);
                u32 node = rd(head + 0x24u, ok);
                const u32 req_base = rd(obj + 0x8u, ok);
                ZLB_LOG_INFO("machine",
                             "region tree: arm%u obj=0x%08X part=0x%08X head=0x%08X root=0x%08X "
                             "req_base=0x%08X req_size=0x%08X",
                             core, obj, part, head, node, req_base, arm->r[1]);
                for (u32 i = 0; i < 8 && ok && node != head; ++i) {
                    const u32 base = rd(node + 0x18u, ok);
                    const u32 size = rd(node + 0x14u, ok);
                    const u32 next = rd(node + 0x24u, ok);
                    ZLB_LOG_INFO("machine", "  node[%u] 0x%08X base=0x%08X size=0x%08X next=0x%08X",
                                 i, node, base, size, next);
                    node = next;
                }
            }
        }
    }

    // Substitution (round 121): the partition region tree walked by 0x4002EE60 has a
    // single garbage root node (measured base=5, size=0) because the secure world's
    // memory map never arrives.  Stamp the root node with the model's DRAM region so
    // the allocator has a region to hand out of.  ZLB_NO_SUBSTITUTION=1 disables it;
    // ZLB_KBL_TREE_FIX=0 disables just this substitution.
    static const bool tree_fix = [] {
        const char* off = std::getenv("ZLB_NO_SUBSTITUTION");
        if (off != nullptr && off[0] != '0') return false;
        const char* on = std::getenv("ZLB_KBL_TREE_FIX");
        return on == nullptr || on[0] != '0';
    }();
    if (tree_fix && pc == 0x4002EE60u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                auto rd = [&](u32 va, bool& ok) {
                    u32 pa = 0;
                    std::string fault;
                    if (!arm->translate(va, false, false, pa, fault)) { ok = false; return 0u; }
                    ok = true;
                    return arm_bus_->read32(pa);
                };
                auto wr = [&](u32 va, u32 value) {
                    u32 pa = 0;
                    std::string fault;
                    if (!arm->translate(va, true, false, pa, fault)) return false;
                    arm_bus_->write32(pa, value);
                    return true;
                };
                bool ok = true;
                const u32 obj = arm->r[0];
                const u32 part = rd(obj + 0x4Cu, ok);
                const u32 head = rd(part + 0xCu, ok);
                const u32 root = rd(head + 0x24u, ok);
                const u32 size = rd(root + 0x14u, ok);
                if (ok && (root == head || size == 0u || size < 0x1000u)) {
                    // Complete tree rebuild (round 124): the sentinel (head) overlaps the
                    // class-registration list in physical memory (VA 0x12040 and 0x10040
                    // both map to PA 0x40), so its +0x0C flag is corrupted to a non-zero
                    // class id and the walk follows [head+0x00] straight into the class
                    // list instead of the root at [head+0x24].  Each call carves a fresh
                    // node out of a dedicated arena (VA 0x01200000) and installs it as a
                    // single-node tree so the allocator's base==req_base (aligned) path
                    // returns it; the tree empties again after the unlink, and the next
                    // request is served the same way.
                    const u32 req_base = rd(obj + 0x8u, ok);
                    u32 req_size = static_cast<u32>(arm->r[1]);
                    if (req_size == 0u || req_size > 0x100000u) req_size = 0x1000u;
                    ensure_arena_section(arm, arm_bus_.get(), kTreeNodeArenaVa);
                    u32 node = 0u;
                    if (tree_node_next_ + 0x1000u <= kTreeNodeArenaSize) {
                        node = kTreeNodeArenaVa + tree_node_next_;
                        tree_node_next_ += 0x1000u;
                    }
                    if (node != 0u) {
                        // Zero the node page (one page per node; fields used span +0x00..+0x24).
                        for (u32 off = 0; off < 0x28u; off += 4u) wr(node + off, 0u);
                        // Sentinel (head): flag 0 selects [head+0x24] as the root, left
                        // link, self neighbour, zero base/size.
                        wr(head + 0x0Cu, 0u);
                        wr(head + 0x00u, node);
                        wr(head + 0x20u, head);
                        wr(head + 0x14u, 0u);
                        wr(head + 0x18u, 0u);
                        wr(head + 0x24u, node);
                        // Fresh node: NIL children/parent/neighbours = head, black,
                        // base/size = the requested block.
                        wr(node + 0x00u, head);
                        wr(node + 0x08u, head);
                        wr(node + 0x0Cu, 0u);
                        wr(node + 0x14u, req_size);
                        wr(node + 0x18u, req_base);
                        wr(node + 0x20u, head);
                        wr(node + 0x24u, head);
                        ++tree_nodes_supplied_;
                        ++tree_fix_hits_;
                        if (tree_fix_hits_ <= 8) {
                            ZLB_LOG_INFO("machine",
                                         "region tree rebuilt: head 0x%08X node 0x%08X base=0x%08X "
                                         "size=0x%08X (was root 0x%08X size=0x%08X) (development substitution)",
                                         head, node, req_base, req_size, root, size);
                            add_milestone("KBL region tree rebuilt (development substitution)");
                        }
                    }
                }
            }
        }
        return false;    // the allocator still runs its own code
    }

    // Substitution (round 134): the SceSysmem heap lookup 0x4002C4D8 walks the
    // object manager for `(type=0x0001000B, uid=r0)` and, on a miss, prints
    // "sceKernelAllocHeapMemory failed(NULL)" (kprintf of the .rodata string
    // 0x4005A9A4).  The heap object 0x5900 is created (round 130) but its SceUID is
    // never allocated (empty registry 0x5A40), so every lookup sees uid == 0 and
    // fails.  Stand in: when the uid is 0, return the created heap object directly
    // so the allocator has a heap to carve out of.  ZLB_NO_SUBSTITUTION=1 disables
    // it; ZLB_KBL_HEAP_LOOKUP=0 disables just this substitution.
    static const bool heap_lookup_fix = [] {
        const char* off = std::getenv("ZLB_NO_SUBSTITUTION");
        if (off != nullptr && off[0] != '0') return false;
        const char* on = std::getenv("ZLB_KBL_HEAP_LOOKUP");
        return on == nullptr || on[0] != '0';
    }();
    if (heap_lookup_fix && pc == 0x4002C4D8u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                // Hand back a fresh, zeroed page from the lookup arena: the four
                // callers (0x4002E0FA/0x4002E108/0x4002E114/0x4002E120) link these
                // objects together, so each needs its own writable address.
                ensure_arena_section(arm, arm_bus_.get(), kLookupArenaVa);
                u32 obj = 0u;
                if (lookup_object_next_ + 0x1000u <= kLookupArenaSize) {
                    obj = kLookupArenaVa + lookup_object_next_;
                    lookup_object_next_ += 0x1000u;
                }
                if (obj != 0u) {
                    for (u32 off = 0; off < 0x1000u; off += 4u) {
                        u32 pa = 0;
                        std::string fault;
                        if (!arm->translate(obj + off, true, false, pa, fault)) { obj = 0u; break; }
                        arm_bus_->write32(pa, 0u);
                    }
                }
                if (obj != 0u) {
                    arm->set_register("r0", obj);
                    arm->set_pc(arm->r[14] & ~1u);          // return to the caller
                    arm->set_register("THUMB", 1u);
                    ++heap_lookup_fixes_;
                    if (heap_lookup_fixes_ <= 4) {
                        ZLB_LOG_INFO("machine",
                                     "heap lookup substituted: arm%u -> object VA 0x%08X "
                                     "(development substitution)",
                                     core, obj);
                        add_milestone("KBL heap lookup substituted (development substitution)");
                    }
                    return true;
                }
            }
        }
    }

    // Experiment (round 122): bypass the region-tree allocator 0x4002EE60 entirely.
    // Return success (r0=0) and hand back the requested base as the block, so the
    // KBL can proceed past the empty red-black tree without a full tree model.
    // ZLB_KBL_TREE_BYPASS=1 enables it (off by default).
    static const bool tree_bypass = [] { return std::getenv("ZLB_KBL_TREE_BYPASS") != nullptr; }();
    if (tree_bypass && pc == 0x4002EE60u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 obj = 0;
                if (arm->get_register("r0", obj)) {
                    auto rd = [&](u32 va, bool& ok) {
                        u32 pa = 0;
                        std::string fault;
                        if (!arm->translate(va, false, false, pa, fault)) { ok = false; return 0u; }
                        ok = true;
                        return arm_bus_->read32(pa);
                    };
                    bool ok = true;
                    const u32 req_base = rd(static_cast<u32>(obj) + 0x8u, ok);
                    u32 pa = 0;
                    std::string fault;
                    if (ok && arm->translate(static_cast<u32>(obj) + 0x50u, true, false, pa, fault)) {
                        arm_bus_->write32(pa, req_base);    // "block" = requested base
                        arm->set_register("r0", 0u);        // success
                        arm->set_pc(arm->r[14] & ~1u);      // return to the caller
                        arm->set_register("THUMB", 1u);     // caller is Thumb
                        return true;                        // handled
                    }
                }
            }
        }
    }

    // Diagnostic (round 123): dump the range-check inputs of the region allocator
    // right before 0x4002EEA2 (the base comparison).  ZLB_KBL_RANGECHK_TRACE=1.
    static const bool rangechk_trace = [] { return std::getenv("ZLB_KBL_RANGECHK_TRACE") != nullptr; }();
    if (rangechk_trace && pc == 0x4002EE9Eu && rangechk_trace_hits_ < 8u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                ++rangechk_trace_hits_;
                auto rd = [&](u32 va, bool& ok) {
                    u32 pa = 0;
                    std::string fault;
                    if (!arm->translate(va, false, false, pa, fault)) { ok = false; return 0u; }
                    ok = true;
                    return arm_bus_->read32(pa);
                };
                bool ok = true;
                const u32 node = arm->r[4];
                const u32 obj = arm->r[5];
                const u32 base = rd(node + 0x18u, ok);
                const u32 size = rd(node + 0x14u, ok);
                const u32 req_base = rd(obj + 0x8u, ok);
                ZLB_LOG_INFO("machine",
                             "rangechk: arm%u node=0x%08X base=0x%08X size=0x%08X req_base=0x%08X "
                             "r7(off)=0x%08X r8(reqsize)=0x%08X",
                             core, node, base, size, req_base, arm->r[7], arm->r[8]);
            }
        }
    }

    // Same diagnostic (round 110) for the KBL's "fatal code" stubs: the block around
    // 0x40021998 is a table of them
    //     0x40021998  push {r3,lr} / movs r0,#0x8A / bl 0x40036998 / b .
    //     0x400219A4  … r0 = 0x8B …, 0x400219B8 … r0 = 0x8C …
    // and every one of them ends in `b .`, so the pc alone says nothing about which
    // check failed.  The previous pc is the branch that entered the stub - i.e. the
    // failing test.  ZLB_KBL_PANIC_TRACE=1 enables it.
    if (panic_trace && pc >= 0x40021998u && pc <= 0x400219D0u &&
        (pc & 3u) == 0u && fatal_stub_hits_ < 12u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                ++fatal_stub_hits_;   // the stub ends in `b .`: cap the output
                u64 r0 = 0, r1 = 0, r2 = 0, r3 = 0, lr = 0;
                arm->get_register("r0", r0);
                arm->get_register("r1", r1);
                arm->get_register("r2", r2);
                arm->get_register("r3", r3);
                arm->get_register("r14", lr);
                ZLB_LOG_INFO("machine",
                             "KBL fatal stub arm%u: entered from pc=0x%08X lr=0x%08X "
                             "r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X",
                             core, previous_pc, static_cast<u32>(lr), static_cast<u32>(r0),
                             static_cast<u32>(r1), static_cast<u32>(r2), static_cast<u32>(r3));
            }
        }
    }
    if (panic_trace && pc == 0x40035966u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 r4 = 0, r5 = 0, r0 = 0, r2 = 0, r3 = 0, r6 = 0, r7 = 0, lr = 0;
                arm->get_register("r4", r4);
                arm->get_register("r5", r5);
                arm->get_register("r0", r0);
                arm->get_register("r2", r2);
                arm->get_register("r3", r3);
                arm->get_register("r6", r6);
                arm->get_register("r7", r7);
                arm->get_register("r14", lr);
                u32 pa = 0;
                std::string fault;
                const u32 arena = static_cast<u32>(r4);
                const bool mapped = arm->translate(arena + 0x10u, false, false, pa, fault);
                ZLB_LOG_INFO("machine",
                             "panic check arm%u: r4=0x%08X r5=0x%08X [r4+0x10]=0x%08X%s r0=0x%08X "
                             "r2=0x%08X r3=0x%08X r6=0x%08X r7=0x%08X lr=0x%08X (%s the panic)",
                             core, arena, static_cast<u32>(r5),
                             mapped ? arm_bus_->read32(pa) : 0u,
                             mapped ? "" : " <unmapped>", static_cast<u32>(r0), static_cast<u32>(r2),
                             static_cast<u32>(r3), static_cast<u32>(r6), static_cast<u32>(r7),
                             static_cast<u32>(lr),
                             (mapped && arm_bus_->read32(pa) > static_cast<u32>(r5)) ? "about to take"
                                                                                    : "will skip");
            }
        }
    }

    // Note (round 109): the loop at 0x4003B3D2..0x4003B3DC (`blx 0x4003A018` relax,
    // then spin while the 16-bit field at [obj+4] > 0, obj = 0x4005C004) is *not* a
    // missing hardware completion: it is the KBL's four-core rendezvous, i.e. the
    // barrier the whole-machine stepping loop already documents for 0x4003B384.  It
    // resolves by itself as soon as the four Kermit cores are interleaved at
    // instruction level, which `runm` (whole slices) does - and it deadlocks under
    // the debugger's `run`, which single-steps the cores and never consults this
    // hook at all (see docs/KBL.md round 109).  No substitution is needed here.

    // Substitution (round 110): the KBL's allocator entry validates its heap object
    //      40034A0C  ldr r2,[r0,#0x24]        ; the object's cookie field
    //      40034A12  ldr r3,[r6=0x400B2974]   ; the KBL's build cookie (round 109.6)
    //      40034A16  cmp r2,r3
    //      40034A18  bne.w 0x4003596E         ; -> the panic stub
    // and the heap object the loader builds for itself (VA 0x400C1000) never gets
    // +0x24 written: a write trap over 0x400C1000-0x400C1040 lists every store its
    // constructor makes (+8, +0xC, +0xE, +0x10, +0x14 = 0xFFFFFFFF, +0x20, +0x28 …
    // +0x3C) and +0x24 is not among them, so the field stays 0 and the second
    // validation (the first one runs before the cookie exists, when both sides are
    // 0) fails.  Stand in for whatever is supposed to leave that stamp: when the
    // allocator entry is entered with a zero cookie field and the global cookie is
    // already set, write the cookie in.  ZLB_NO_SUBSTITUTION=1 disables it.
    constexpr u32 kAllocEntryPc = 0x40034A00u;
    constexpr u32 kCookieVa = 0x400B2974u;
    if (supply_blocks && pc == kAllocEntryPc) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 heap = 0;
                if (arm->get_register("r0", heap)) {
                    // Round 135: route a garbage heap pointer (measured 0x4B656353 =
                    // the first word of the string "SceKernel…" read little-endian, a
                    // never-initialised [pool] word) to the real created heap 0x5900,
                    // and zero its cookie so the 0x40034A16 comparison against the
                    // (still zero) global cookie 0x400B2974 passes.
                    const u32 hv = static_cast<u32>(heap);
                    if (hv >= 0x10000u) {
                        arm->set_register("r0", 0x00005900u);
                        heap = 0x00005900ull;
                        u32 field_pa = 0;
                        std::string fault;
                        if (arm->translate(0x00005900u + 0x24u, true, false, field_pa, fault)) {
                            arm_bus_->write32(field_pa, 0u);
                        }
                        ++heap_route_fixes_;
                        if (heap_route_fixes_ <= 8) {
                            ZLB_LOG_INFO("machine",
                                         "heap routed: arm%u 0x%08X -> 0x5900 (development substitution)",
                                         core, hv);
                            add_milestone("KBL heap routed to 0x5900 (development substitution)");
                        }
                    }
                }
                if (heap >= 0x1000ull && heap < 0x80000000ull) {
                    u32 cookie_pa = 0, field_pa = 0;
                    std::string fault;
                    if (arm->translate(kCookieVa, false, false, cookie_pa, fault)) {
                        const u32 cookie = arm_bus_->read32(cookie_pa);
                        if (cookie != 0u &&
                            arm->translate(static_cast<u32>(heap) + 0x24u, true, false,
                                           field_pa, fault)) {
                            const u32 current = arm_bus_->read32(field_pa);
                            if (current == 0u) {
                                arm_bus_->write32(field_pa, cookie);
                                ++cookie_stamps_;
                                if (cookie_stamps_ <= 8) {
                                    ZLB_LOG_INFO("machine",
                                                 "heap cookie stamped: VA 0x%08X+0x24 = 0x%08X "
                                                 "(heap 0x%08X) (development substitution)",
                                                 static_cast<u32>(heap), cookie,
                                                 static_cast<u32>(heap));
                                    add_milestone("KBL heap cookie stamped at VA 0x" +
                                                  hex(static_cast<u32>(heap), 8) +
                                                  " (development substitution)");
                                }
                            }
                        }
                    }
                }
            }
        }
        return false;    // the allocator still runs its own code
    }

    // Substitution (round 108, corrected round 114): the getter 0x4002C1AC builds
    // the instance pointer as `[obj+0x14] + table[size_class]`.  The object's base
    // field `[obj+0x14]` is the 0xFFFFFFFF sentinel the class constructor stores
    // (round 108.1 measured the store itself), because the stage the model stands in
    // for never fills it.  Round 114: the old stand-in replaced the getter *result*
    // at 0x4002C1CC and only caught `0xFFFFFFFF + 0`; the table's non-zero entries
    // are 0x1000/0x2000, so the sum wrapped to 0x0FFF/0x1FFF and the constructor
    // loop walked into the low window and panicked on the heap-cookie check with a
    // bogus heap object.  Fill the base field itself with the instance arena VA:
    // the getter then returns arena + table[size_class], a valid per-class address.
    // ZLB_NO_SUBSTITUTION=1 disables it; ZLB_KBL_INSTANCE_BLOCK=0 keeps only the
    // result fallback below.
    static const bool supply_instance_blocks = [] {
        const char* value = std::getenv("ZLB_KBL_INSTANCE_BLOCK");
        return value == nullptr || value[0] != '0';
    }();
    constexpr u32 kGetterEntryPc = 0x4002C1ACu;    // ldr r3,[r0,#0x30] - getter entry
    if (supply_blocks && supply_instance_blocks && pc == kGetterEntryPc) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                ensure_instance_arena(arm, arm_bus_.get());
                u64 obj = 0;
                if (arm->get_register("r0", obj) && obj >= 0x1000ull && obj < 0x80000000ull) {
                    u32 pa = 0;
                    std::string fault;
                    if (arm->translate(static_cast<u32>(obj) + 0x14u, true, false, pa, fault)) {
                        const u32 base = arm_bus_->read32(pa);
                        if (base == 0u || base == 0xFFFFFFFFu) {
                            arm_bus_->write32(pa, kInstanceArenaVa);
                            ++instance_base_fixes_;
                            if (instance_base_fixes_ <= 4) {
                                ZLB_LOG_INFO("machine",
                                             "instance base filled: obj VA 0x%08X+0x14 = 0x%08X -> "
                                             "arena 0x%08X (development substitution)",
                                             static_cast<u32>(obj), base, kInstanceArenaVa);
                                add_milestone("KBL instance base filled at VA 0x" +
                                              hex(static_cast<u32>(obj), 8) +
                                              " (development substitution)");
                            }
                        }
                    }
                }
            }
        }
        return false;    // the getter still runs, now reading a valid base
    }

    constexpr u32 kGetterStorePc = 0x4002C1CCu;    // str r3,[r1] in the getter
    if (supply_blocks && supply_instance_blocks && pc == kGetterStorePc) {
        // Result fallback.  The getter computes `[obj+0x14] + table[size_class]`;
        // when the base field is the 0xFFFFFFFF sentinel (or obj is NULL so the
        // field reads garbage) the sum is 0xFFFFFFFF + {0,0x1000,0x2000} =
        // 0xFFFFFFFF / 0x0FFF / 0x1FFF - none of them page-aligned, and the latter
        // two walk into the low window.  Any non-page-aligned result is therefore
        // bogus; hand out the next arena page instead (0xFFFFFFFF & 0xFFF != 0, so
        // the old sentinel check is subsumed).
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 value = 0;
                if (arm->get_register("r3", value) &&
                    (value == 0u || (value & 0xFFFull) != 0u)) {
                    u32 block = 0;
                    if (supply_kbl_instance_block(core, block)) {
                        arm->set_register("r3", block);
                        ++boot_pc_fixes_;
                        if (boot_pc_fixes_ <= 4) {
                            ZLB_LOG_INFO("machine",
                                         "instance base getter: 0x%08llX -> block VA 0x%08X at "
                                         "pc=0x%08X (development substitution)",
                                         static_cast<unsigned long long>(value), block, pc);
                            add_milestone("KBL instance base supplied at VA 0x" + hex(block, 8) +
                                          " (development substitution)");
                        }
                    }
                }
            }
        }
        return false;    // the store still has to run, with a usable value
    }

    // Experiment (ZLB_KBL_CARVE=1): instead of the model handing blocks out of the
    // page cache, stand in for the region bookkeeping the secure kernel would leave,
    // so the loader's OWN carve path (0x40032366 -> 0x400323F8) hands the block out:
    //   * class marker `0x00010002` plus the free page index in pool+0x38+8*class
    //     (0x00010001 means "class empty", as the loader itself initialises it);
    //   * the page-table entry at [pool+0x20] + n*4 with state 0x20000000, the class
    //     in bits 20..24 and the size in 4 KiB units (states 0x10000000/0x20000000
    //     are the two the carve path accepts, docs/KBL.md §95.1).
    // ZLB_NO_SUBSTITUTION=1 disables it; ZLB_KBL_CARVE_STATE=<hex> overrides the state.
    constexpr u32 kCarvePathEntryPc = 0x40032366u;
    static const bool carry_supply = [] {
        const char* off = std::getenv("ZLB_NO_SUBSTITUTION");
        if (off != nullptr && off[0] != '0') return false;
        const char* on = std::getenv("ZLB_KBL_CARVE");
        return on != nullptr && on[0] != '0';
    }();
    if (carry_supply && pc == kCarvePathEntryPc) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                u64 size = 0;
                u64 pool = 0;
                arm->get_register("r5", size);
                arm->get_register("r4", pool);
                supply_kbl_carve_state(core, static_cast<u32>(pool), static_cast<u32>(size));
            }
        }
        return false;    // let the loader's own carve path run
    }

    constexpr u32 kCacheOnlyFailurePc = 0x4003235Cu;  // `mov.w r10, #5 / movt 0x8002`
    constexpr u32 kCarvePathPc = 0x40032366u;         // `add.w r7, r4, #14` (take the mutex)
    static const bool disabled = [] {
        const char* value = std::getenv("ZLB_NO_SUBSTITUTION");
        if (value != nullptr && value[0] != '0') return true;
        const char* carve = std::getenv("ZLB_ALLOC_CARVE");
        return carve == nullptr || carve[0] == '0';
    }();
    if (disabled || pc != kCacheOnlyFailurePc) return false;
    if (core >= static_cast<u32>(kArmCoreCount)) return false;
    Cpu* cpu = arm_cores_[core].get();
    if (!cpu) return false;

    cpu->set_pc(kCarvePathPc);
    if (boot_pc_fixes_ == 0) {
        ZLB_LOG_INFO("machine",
                     "partition allocator: empty 0x1000 block cache -> using the carving path at 0x%08X "
                     "(ZLB_ALLOC_CARVE experiment)",
                     kCarvePathPc);
        add_milestone(
            "KBL partition allocator cache miss redirected to its carving path (ZLB_ALLOC_CARVE experiment)");
    }
    ++boot_pc_fixes_;
    return true;
}

// Substitution body (round 93): create the per-class table the SceUID registration
// walks (global 0x400B291C), because nothing in the loader ever writes that global
// - see the note at the call site.  The table is 8 entries of 0x14 bytes
// ({lock, count, head, next, spare}) and lives in a page the model allocates for
// itself out of the partition's region, walking down from the tail until a page
// accepts writes.
bool Vita::supply_kbl_class_table(u32 core) {
    if (core >= static_cast<u32>(kArmCoreCount)) return false;
    ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get());
    if (arm == nullptr) return false;
    constexpr u32 kClassTableGlobalVa = 0x400B291Cu;
    auto read_va = [&](u32 va, bool& ok) {
        u32 pa = 0;
        std::string fault;
        if (!arm->translate(va, false, false, pa, fault)) {
            ok = false;
            return 0u;
        }
        return arm_bus_->read32(pa);
    };
    auto write_va = [&](u32 va, u32 value) {
        u32 pa = 0;
        std::string fault;
        if (!arm->translate(va, true, false, pa, fault)) return false;
        arm_bus_->write32(pa, value);
        return true;
    };

    bool ok = true;
    if (read_va(kClassTableGlobalVa, ok) != 0u || !ok) return false;  // already there

    // The partition pointer itself sits in the system structure (0x400B294C was
    // written by 0x40032CC8); fall back to the documented VA when it is not up yet.
    bool pool_ok = true;
    u32 pool_va = read_va(0x400B294Cu, pool_ok);
    if (!pool_ok || pool_va == 0u) pool_va = 0x000051C0u;
    const u32 region_base = read_va(pool_va + 0x18u, ok);
    const u32 region_size = read_va(pool_va + 0x1Cu, ok);
    // The region must be DRAM: once the loader has rebuilt its page tables the pool
    // VA can resolve to something unrelated, and without this check the probing loop
    // below walks down into the mapped low window and "succeeds" there (round 111).
    if (!ok || region_base < 0x40000000u || region_base >= 0x80000000u ||
        region_size < 0x2000u || region_size > 0x40000000u) {
        return false;
    }

    constexpr u32 kTableBytes = 8u * 0x14u;
    u32& cursor = partition_block_next_[1];
    if (cursor == 0u || cursor > region_size) cursor = region_size - 0x1000u;
    while (cursor >= 0x1000u) {
        const u32 table = region_base + cursor;
        bool writable = true;
        for (u32 offset = 0; offset < kTableBytes; offset += 4u) {
            if (!write_va(table + offset, 0u)) {
                writable = false;
                break;
            }
        }
        if (writable) {
            if (!write_va(kClassTableGlobalVa, table)) return false;
            cursor = cursor > 0x1000u ? cursor - 0x1000u : 0u;
            class_table_va_ = table;    // remember it: the loader zeroes the slot later
            ++class_tables_supplied_;
            ZLB_LOG_INFO("machine",
                         "class table supplied at VA 0x%08X (global 0x%08X, %u entries) "
                         "(development substitution)",
                         table, kClassTableGlobalVa, 8u);
            add_milestone("KBL class table created at VA 0x" + hex(table, 8) +
                          " (development substitution)");
            return true;
        }
        cursor = cursor > 0x1000u ? cursor - 0x1000u : 0u;
    }
    return false;
}

// Experiment body (round 101): give one size class the free-chunk marker and the
// page-table entry the loader's carve path looks for, so the loader hands the block
// out itself.  See the note at the call site.
bool Vita::supply_kbl_carve_state(u32 core, u32 pool_va, u32 size) {
    if (core >= static_cast<u32>(kArmCoreCount)) return false;
    if (size < 0x2000u || size > 0x100000u || (size & (size - 1u)) != 0u) return false;
    u32 class_index = 0;
    for (u32 s = 0x1000u; s < size; s <<= 1u) ++class_index;    // 0x2000 -> 1, 0x4000 -> 2
    ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get());
    if (arm == nullptr) return false;
    auto read_va = [&](u32 va, bool& ok) {
        u32 pa = 0;
        std::string fault;
        if (!arm->translate(va, false, false, pa, fault)) {
            ok = false;
            return 0u;
        }
        return arm_bus_->read32(pa);
    };
    auto write_va = [&](u32 va, u32 value) {
        u32 pa = 0;
        std::string fault;
        if (!arm->translate(va, true, false, pa, fault)) return false;
        arm_bus_->write32(pa, value);
        return true;
    };
    if (pool_va == 0u) pool_va = 0x000051C0u;
    bool ok = true;
    if (read_va(pool_va + 0x08u, ok) != 0x4080502Bu || !ok) return false;
    const u32 marker_va = pool_va + 0x38u + 8u * class_index;
    if (read_va(marker_va, ok) != 0x00010001u || !ok) return false;   // already has a chunk
    const u32 table = read_va(pool_va + 0x20u, ok);
    const u32 start_page = read_va(pool_va + 0x84u, ok);
    const u32 end_page = read_va(pool_va + 0x88u, ok);
    if (!ok || table == 0u || end_page <= start_page) return false;

    u32& cursor = carve_page_next_;                    // a *page index*, not a byte offset
    const u32 span = end_page - start_page;
    const u32 pages = size >> 12;
    if (cursor == 0u || cursor + pages >= span) cursor = span / 4u;
    const u32 index = cursor;
    cursor += pages;
    // The page-table entry: state | class<<20 | pages.
    u32 state = 0x20000000u;
    if (const char* value = std::getenv("ZLB_KBL_CARVE_STATE")) {
        state = static_cast<u32>(std::strtoul(value, nullptr, 16));
    }
    if (!write_va(table + index * 4u, state | (class_index << 20) | pages)) return false;
    if (!write_va(marker_va + 4u, start_page + index)) return false;
    if (!write_va(marker_va, 0x00010002u)) return false;
    ++partition_supplied_;
    if (partition_supplied_ <= 6) {
        ZLB_LOG_INFO("machine",
                     "carve state supplied: pool=0x%08X class 0x%X (index %u) start=%u end=%u "
                     "page %u entry 0x%08X (development substitution experiment)",
                     pool_va, size, class_index, start_page, end_page, start_page + index,
                     state | (class_index << 20) | pages);
    }
    return true;
}

// Substitution body (round 108, fixed round 114): hand out one zeroed page for a
// class instance whose base field the loader never fills in.  The getter
// 0x4002C1AC returns `[obj+0x14] + table[..]` and the instance the constructor loop
// passes carries 0xFFFFFFFF there, so the loop receives -1 and its `str.w r1,[r8]`
// faults (round 108.1).
//
// Round 114: the original page source - the partition-region tail, walked down -
// collides with the KBL's own memory: the region 0x40000000..0x40100000 also holds
// the KBL image (ends 0x40075B94), its stack (~0x40060xxx), the class table
// (0x400B0000) and the heap objects (0x400C1000).  Walking down from the tail
// eventually handed the constructor a page inside the current stack frame, so the
// constructor's `obj+0x14 = 0xFFFFFFFF` store overwrote the saved return address
// and the run died on a fetch of 0xFFFFFFFE (round 113).  Pages now come from a
// dedicated 1 MiB arena in the low kernel window (VA 0x01100000+, the section above
// the KBL's hardcoded 0x01031000 base), mapped with the round-110 dram-abs rule to
// PA 0x41100000+ - free DRAM above the KBL image and the partition region.
bool Vita::supply_kbl_instance_block(u32 core, u32& out_block) {
    if (core >= static_cast<u32>(kArmCoreCount)) return false;
    ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get());
    if (arm == nullptr) return false;

    constexpr u32 kPageSize = 0x1000u;
    ensure_instance_arena(arm, arm_bus_.get());

    u32& cursor = instance_block_next_;
    if (cursor == 0u || cursor >= kInstanceArenaSize) cursor = 0u;   // 0 = arena not started
    if (cursor + kPageSize > kInstanceArenaSize) return false;

    const u32 block = kInstanceArenaVa + cursor;
    cursor += kPageSize;

    // Zero the page so the objects the loader builds inside it stay clean.
    for (u32 offset = 0; offset < kPageSize; offset += 4u) {
        u32 pa = 0;
        std::string fault;
        if (!arm->translate(block + offset, true, false, pa, fault)) return false;
        arm_bus_->write32(pa, 0u);
    }

    out_block = block;
    ++instance_blocks_supplied_;
    if (instance_blocks_supplied_ <= 4) {
        ZLB_LOG_INFO("machine",
                     "instance block supplied: arm%u VA 0x%08X (arena VA 0x%08X + 0x%X, PA 0x%08X) "
                     "(development substitution)",
                     core, block, kInstanceArenaVa, cursor - kPageSize, 0x40000000u + block);
    }
    return true;
}

// Substitution body: put free blocks of the requested class into the calling
// core's page-cache slot of the KBL's own partition.  The partition record, its
// cache table and the region it describes are all addressed through the core's
// active page tables, because the loader rebuilds them while it boots (docs/KBL.md
// round 57), and every field offset here was read out of the allocator itself
// (round 95): +0x08 magic 0x4080502B, +0x18 size, +0x1C base, +0x9C cache table,
// cache slot = table + core*0x20.
bool Vita::supply_kbl_partition_block(u32 core, u32 pool_va, u32 size) {
    // The page cache is only consulted for 0x1000 (see the note at the call site);
    // larger classes need the class markers and the page table instead.
    if (size != 0x1000u) return false;

    static bool configured = false;
    if (!configured) {
        configured = true;
        if (const char* value = std::getenv("ZLB_PART_BLOCK_CACHE")) {
            const long count = std::strtol(value, nullptr, 0);
            // 0 disables the seeding entirely, so the loader's own carve path can be
            // exercised on its own (ZLB_KBL_CARVE=1).
            if (count <= 0) {
                partition_blocks_per_class_ = 0;
                return false;
            }
            partition_blocks_per_class_ = static_cast<u32>(count);
        }
    }
    // The slot holds {u16 target, u16 count, u32 head, u32 extra0, u32 extra1} and
    // the pop path only reads head / extra0 / extra1, so at most three blocks can
    // be handed out from one seeding.
    const u32 blocks = partition_blocks_per_class_ < 3u ? partition_blocks_per_class_ : 3u;

    Cpu* cpu = arm_cores_[core].get();
    ArmCore* arm = dynamic_cast<ArmCore*>(cpu);
    if (!arm) return false;
    auto read_va = [&](u32 va, bool& ok) {
        u32 pa = 0;
        std::string fault;
        if (!arm->translate(va, false, false, pa, fault)) {
            ok = false;
            return 0u;
        }
        return arm_bus_->read32(pa);
    };
    auto write_va = [&](u32 va, u32 value) {
        u32 pa = 0;
        std::string fault;
        if (!arm->translate(va, true, false, pa, fault)) return false;
        arm_bus_->write32(pa, value);
        return true;
    };

    if (pool_va == 0u) pool_va = 0x000051C0u;
    bool ok = true;
    const u32 magic = read_va(pool_va + 0x08u, ok);
    if (!ok || magic != 0x4080502Bu) return false;   // partition not built yet
    const u32 cache_table = read_va(pool_va + 0x9Cu, ok);
    if (!ok || cache_table == 0u) return false;

    const u32 slot = cache_table + core * 0x20u;
    const u32 word0 = read_va(slot, ok);
    // The slot is {u16 target, u16 count} in one word (the allocator reads the
    // target with `ldrh [r7]` at 0x400322AA and the count with `ldrh [r7,#2]` at
    // 0x400322A0), then the head pointer at +4 and two spare pointers after it.
    const u32 target = word0 & 0xFFFFu;
    const u32 count = (word0 >> 16) & 0xFFFFu;
    if (count != 0u) return false;                        // the KBL filled it itself
    if (target == 0u) return false;                       // slot not initialised

    // The blocks come from the tail of the region the partition describes: the
    // builder 0x40032108 stores the base at +0x18 (`str r3,[r0,#0x18]`) and the
    // size at +0x1C (`str.w r5,[r0,#0x1c]`), and the loader's own allocations grow
    // from the start, so the tail is the part that stays free.
    const u32 region_base = read_va(pool_va + 0x18u, ok);
    const u32 region_size = read_va(pool_va + 0x1Cu, ok);
    if (!ok || region_size < 0x1000u || region_base == 0u) {
        partition_region_base_ = 0x40000000u;
        partition_region_size_ = 0x00300000u;
    } else {
        partition_region_base_ = region_base;
        partition_region_size_ = region_size;
    }
    u32& next = partition_block_next_[0];
    const u32 span = 0x1000u * blocks;
    // Always seed from the *tail* of the region: the loader's own allocations grow
    // from the base (its first request is a whole 1 MiB), so anything below is
    // already spoken for.  The loader maps the region lazily, though, so the tail
    // pages may still be unmapped (a probe write faults) - walk downwards from the
    // tail and keep the pages that accept a zero write.  Zeroing them also keeps
    // the objects the loader builds inside them clean, which the "stale lock"
    // symptom of round 95 needed.
    if (next == 0u || next > region_size || next < region_size / 4u) next = region_size - span;
    if (next < span) return false;

    u32 head = 0;
    u32 written = 0;
    u32 cursor = next;
    while (written < blocks && cursor >= 0x1000u) {
        const u32 block = partition_region_base_ + cursor;
        if (!write_va(block, 0u)) {
            // Not mapped writable yet: skip this page and try the one below.
            cursor = cursor > 0x1000u ? cursor - 0x1000u : 0u;
            continue;
        }
        if (written == 0u) {
            head = block;
            if (!write_va(slot + 4u, block)) break;
        } else {
            if (!write_va(slot + 4u + 4u * written, block)) break;
        }
        ++written;
        cursor = cursor > 0x1000u ? cursor - 0x1000u : 0u;
    }
    if (written == 0u) return false;
    next = cursor;
    // Count last (high half of the target word): the allocator only looks at the
    // slot when the count is non-zero.
    if (!write_va(slot, target | (written << 16))) return false;

    ++partition_supplied_;
    if (partition_supplied_ <= 4) {
        ZLB_LOG_INFO("machine",
                     "partition page cache supplied: arm%u class 0x%X -> %u block(s) at VA 0x%08X "
                     "(slot 0x%08X) (development substitution)",
                     core, size, written, head, slot);
        add_milestone("KBL partition page cache pre-populated for class 0x" + hex(size, 0) +
                      " (development substitution)");
    }
    return true;
}

// Diagnostic PC tracer (see vita.h).  Enabled only when ZLB_PCTRAP=<lo>-<hi> is
// set; it never changes the run, it only writes to stderr, so it can be left on
// while the machine executes hundreds of millions of instructions.
bool Vita::trace_arm_boot_pc(u32 core, u32 pc) {
    if (!pc_trace_enabled_) return false;
    if (pc < pc_trace_lo_ || pc > pc_trace_hi_) return false;
    if (core >= static_cast<u32>(kArmCoreCount)) return false;
    ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get());
    if (!arm) return false;

    const u32 insn = (arm->cpsr & 0x20u) ? arm_bus_->read16(pc) : arm_bus_->read32(pc);
    std::fprintf(stderr,
                 "[pctrap] arm%u pc=%08X insn=%08X sp=%08X lr=%08X r0=%08X r1=%08X r2=%08X r3=%08X "
                 "r4=%08X r5=%08X r6=%08X r7=%08X r8=%08X r9=%08X sl=%08X sb=%08X r12=%08X cpsr=%08X\n",
                 core, pc, insn, arm->r[13], arm->r[14], arm->r[0], arm->r[1], arm->r[2], arm->r[3],
                 arm->r[4], arm->r[5], arm->r[6], arm->r[7], arm->r[8], arm->r[9], arm->r[10],
                 arm->r[11], arm->r[12], arm->cpsr);
    if (pc == 0x4003234Eu || pc == 0x40032366u || pc == 0x400327D6u || pc == 0x400327D8u ||
        pc == 0x40031BE4u || pc == 0x4002BB62u) {
        std::fprintf(stderr,
                     "[pcalloc] pc=%08X core=%u r0=%08X r1=%08X r2=%08X r3=%08X r4=%08X r5=%08X "
                     "r6=%08X r7=%08X lr=%08X\n",
                     pc, core, arm->r[0], arm->r[1], arm->r[2], arm->r[3], arm->r[4], arm->r[5],
                     arm->r[6], arm->r[7], arm->r[14]);
    }
    // Diagnostic: dump the partition allocator's inputs and the region table it is
    // about to read (partition VA 0x51C0, region descriptor VA 0x5180), translated
    // through the core's own page tables - the loader rebuilds them as it boots.
    if (pc == 0x40032366u || pc == 0x40031BE4u || pc == 0x4002BB62u) {
        for (u32 row = 0; row < 10; ++row) {
            const u32 va = 0x5180u + row * 16u;
            u32 pa = 0;
            std::string fault;
            if (!arm->translate(va, false, false, pa, fault)) {
                std::fprintf(stderr, "[pcpool] %08X  <no translation: %s>\n", va, fault.c_str());
                continue;
            }
            std::fprintf(stderr, "[pcpool] %08X  %08X %08X %08X %08X\n", va, arm_bus_->read32(pa),
                         arm_bus_->read32(pa + 4), arm_bus_->read32(pa + 8), arm_bus_->read32(pa + 12));
        }
    }
    return true;
}

// Reads ZLB_PCTRAP ("<lo>-<hi>", hex) once, at boot-chain construction.
void Vita::configure_arm_pc_trace() {
    const char* value = std::getenv("ZLB_PCTRAP");
    if (value == nullptr) return;
    const char* dash = std::strchr(value, '-');
    if (dash == nullptr) return;
    pc_trace_lo_ = static_cast<u32>(std::strtoul(value, nullptr, 16));
    pc_trace_hi_ = static_cast<u32>(std::strtoul(dash + 1, nullptr, 16));
    pc_trace_enabled_ = pc_trace_lo_ <= pc_trace_hi_;
    ZLB_LOG_INFO("machine", "ARM PC trace armed: 0x%08X-0x%08X (ZLB_PCTRAP)", pc_trace_lo_, pc_trace_hi_);
}

bool Vita::mirror_cmep_scratch_to_arm() {    // See the note at the call site: ARM PA 0 mirrors the CMeP's 32 KiB scratch.
    constexpr u32 kScratchSize = 0x8000;
    if (shared_sram_.size() < kScratchSize) return false;
    for (u32 i = 0; i < kScratchSize; ++i) arm_bus_->write8(i, shared_sram_[i]);
    // Report what the ARM now sees, so the mirror is visible in the log.
    ZLB_LOG_INFO("boot", "mirrored the CMeP scratch (32 KiB) to ARM PA 0 (context at +0x100: %08X %08X)",
                 arm_bus_->read32(0x100), arm_bus_->read32(0x104));
    add_milestone("CMeP scratch mirrored to ARM PA 0 (boot context visible to the ARM)");
    return true;
}

bool Vita::build_kbl_param() {
    // SceKblParam - the 0x100-byte record the second loader creates and the secure
    // and non-secure kernel boot loaders read (wiki: KBL_Param).  Its DIP-switch
    // field sits at physical 0x1F000080, which fixes the record at 0x1F000040 in
    // SPAD32K.  The per-console chain that would build it cannot complete in this
    // model, so the documented fields are written here (a development
    // substitution; the magic and layout follow the wiki).
    const u32 base = board::kKblParamBase;
    const u32 dram_base = board::kKblParamDram;
    auto put8 = [&](u32 offset, u8 value) { arm_bus_->write8(base + offset, value); };
    auto put32 = [&](u32 offset, u32 value) { arm_bus_->write32(base + offset, value); };
    auto put8_dram = [&](u32 offset, u8 value) { arm_bus_->write8(dram_base + offset, value); };
    auto put32_dram = [&](u32 offset, u32 value) { arm_bus_->write32(dram_base + offset, value); };
    // Most fields go into both copies (the second loader leaves one in the scratchpad
    // and one in secure DRAM); the magic is scratchpad-only, see below.
    auto put_field8 = [&](u32 offset, u8 value) { put8(offset, value); put8_dram(offset, value); };
    auto put_field32 = [&](u32 offset, u32 value) { put32(offset, value); put32_dram(offset, value); };

    for (u32 i = 0; i < board::kKblParamSize; ++i) {
        put8(i, 0);
        put8_dram(i, 0);
    }
    put_field8(0x00, 1);                                         // version
    put_field8(0x01, 0);
    put_field8(0x02, board::kKblParamSize & 0xFF);               // size = 0x100
    put_field8(0x03, (board::kKblParamSize >> 8) & 0xFF);
    put_field32(0x04, 0x01040000);                               // current firmware version (1.04)
    put_field32(0x08, 0x01040000);                               // minimum firmware version (SMI leaf)
    // 0x20 QA flags: none.  0x30 boot flags: no Ernie NVS overrides.
    // 0x40 DIP switches (0x20 bytes): no CP board (zeroes), then the release-mode
    // values the wiki lists (sdk 0, shell 0, debug 0x00080002, system 0x20000000).
    put_field32(0x50, 0x00000000);                               // SDK (SCE) flags
    put_field32(0x54, 0x00000000);                               // Shell flags
    put_field32(0x58, 0x00080002);                               // Debug control flags (release)
    put_field32(0x5C, 0x20000000);                               // System control flags (release)
    put_field32(0x60, 0x40000000);                               // DRAM base paddr
    put_field32(0x64, kermit::kScuSize);                         // DRAM size (modelled window)
    put_field32(0x6C, 0x00000004);                               // boot type indicator 1: product mode
    // 0x70 OpenPsId: no per-console id in the dumps, left zero.
    put_field32(0x80, board::kCmepSecureKernelBase);             // secure_kernel.enp paddr
    put_field32(0x84, secure_kernel_size_);
    // 0x88 context_auth_sm.self: not present in the 1.04 SLB2.
    put_field32(0x90, kprx_auth_sm_pa_);                         // kprx_auth_sm.self paddr
    put_field32(0x94, kprx_auth_sm_size_);
    put_field32(0x98, prog_rvk_pa_);                             // prog_rvk.srvk paddr
    put_field32(0x9C, prog_rvk_size_);
    put_field32(0xA8, 0x5A5A0001);                               // __stack_chk_guard (model constant)
    put_field32(0xAC, 0xA5A50002);                               // unknown (model constant)
    for (u32 i = 0; i < 0x10; ++i) put_field8(0xB0 + i, static_cast<u8>(0x10 + i));  // session id
    put_field32(0xC0, 0x00000060);                               // sleep factor (syscon cmd 3)
    put_field32(0xC4, 0x0000FF14);                               // wakeup factor (syscon cmd 0x10)
    put_field32(0xC8, 0x00000040);                               // USB info (syscon cmd 0x800)
    put_field32(0xCC, 0x00000000);                               // boot controls info (cmd 0x100)
    put_field32(0xD0, 0x00000000);                               // resume context paddr (cold boot)
    put_field32(0xD4, 0x00406000);                               // hardware info (syscon cmd 5, IRS-002)
    put_field32(0xD8, 0x0000000C);                               // power info: AC + power button
    put_field32(0xE8, 0x00000000);                               // hardware info 2 (syscon cmd 6)
    put_field32(0xF8, 0x00010000);                               // bootloader revision
    // The wiki's Secure DRAM layout marks the DRAM copy as "SceKblParam with magic
    // not set": only the scratchpad copy carries the magic.
    put32(0xFC, board::kKblParamMagic);

    ZLB_LOG_INFO("boot",
                 "SceKblParam built at 0x%08X (magic 0x%08X, dram 0x%08X+0x%X, kprx_auth_sm 0x%08X/0x%X, "
                 "prog_rvk 0x%08X/0x%X)",
                 base, arm_bus_->read32(base + 0xFC), arm_bus_->read32(base + 0x60),
                 arm_bus_->read32(base + 0x64), arm_bus_->read32(base + 0x90), arm_bus_->read32(base + 0x94),
                 arm_bus_->read32(base + 0x98), arm_bus_->read32(base + 0x9C));
    add_milestone("SceKblParam built at 0x1F000040 (development substitution, wiki layout)");
    return true;
}

bool Vita::start_arm_kernel_boot_loader() {
    if (!arm_) return false;

    std::vector<u8> container;
    std::vector<u8> kbl;
    std::string source;

    if (read_slb2_container(*emmc_, container)) {
        auto slb2 = parse_slb2(container);
        if (slb2) {
            if (const Slb2Entry* entry = find_entry(*slb2, "kernel_boot_loader.self")) {
                kbl = entry->data;
                source = "eMMC SLB2";
            }
        }
    }
    if (kbl.empty()) {
        std::string path = resolve_workspace_path("Vita_104_Firmware/Out/SLB2/kernel_boot_loader.self");
        if (auto raw = read_file(path)) {
            kbl = *raw;
            source = path_filename(path);
        }
    }
    if (kbl.empty()) {
        ZLB_LOG_ERROR("machine", "kernel_boot_loader.self not found");
        return false;
    }

    LoadResult result = load_image(*arm_bus_, kbl, "kernel_boot_loader.self", keys_);
    ZLB_LOG_INFO("machine", "KBL image: %zu bytes from %s -> kind=%s entry=0x%08X elf=%d: %s", kbl.size(),
                 source.c_str(), to_string(result.info.kind), result.entry, result.elf ? 1 : 0,
                 result.message.c_str());
    if (!result.ok) {
        ZLB_LOG_ERROR("machine", "kernel boot loader load failed: %s", result.message.c_str());
        return false;
    }

    kbl_entry_ = result.entry;
    // The KBL's ELF carries the ARM exception vector table as a segment at
    // vaddr 0: eight `ldr pc,[pc,#0x18]` vectors followed by the handler pointer
    // table (vector[4] -> 0x40020394, the data abort handler).  The wiki's FW 3.60
    // Secure DRAM layout pins exactly these 0xC0 bytes at PA 0x40000000 as the
    // "SKBL Reset Vector (ARM entry!)", and the KBL points VBAR at a copy of them
    // at board::kArmVectorPage.  The second loader is what normally writes both
    // copies; the model stages them here so the ARM's exception path exists.
    std::vector<u8> boot_vectors;
    if (result.elf) {
        for (const Segment& segment : result.elf->segments) {
            ZLB_LOG_DBG("boot", "  KBL segment vaddr=0x%08X memsz=0x%X filesz=0x%X flags=%u", segment.vaddr,
                        segment.memsz, segment.filesz, segment.flags);
            if (segment.vaddr != 0u || segment.filesz == 0u || segment.filesz > 0x1000u) continue;
            if ((segment.flags & 1u) == 0u) continue;   // the executable (vector) one
            if (segment.offset + segment.filesz > result.elf->data.size()) continue;
            boot_vectors.assign(result.elf->data.begin() + static_cast<ptrdiff_t>(segment.offset),
                                result.elf->data.begin() + static_cast<ptrdiff_t>(segment.offset + segment.filesz));
        }
    }
    ZLB_LOG_INFO("boot", "KBL first word at 0x%08X = 0x%08X (ram=%d)", kbl_entry_,
                 arm_bus_->read32(kbl_entry_), arm_bus_->is_ram(kbl_entry_, 64) ? 1 : 0);

    // SceKblParam has to be in place before the boot loader runs: the wiki pins its
    // DIP-switch field at 0x1F000080 and the loaders read the DRAM range, the boot
    // type and the staged kernel-module paddrs out of it.  Build it first, then
    // mirror the scratch so the record is visible at ARM PA 0x40 as well.
    build_kbl_param();

    // The wiki's boot sequence: the CMeP's 32 KiB scratch buffer (SPAD32K) is
    // "mirror mapped to 0x00000000 on ARM", and the second loader copies its ARM
    // boot context into that scratch (image 0x40A86 copies 1 KiB from DRAM to
    // scratch+0x100).  The model keeps the scratch in the shared boot SRAM, so
    // mirror its first 32 KiB into the ARM's low window before the release.
    mirror_cmep_scratch_to_arm();

    // The syscon releases the whole Kermit cluster, not just the boot core: the
    // kernel boot loader brings the other three cores up itself (each reads MPIDR,
    // builds its own tables and joins the four-core barrier).
    u32 arm_entry = kbl_entry_;
    if (boot_vectors.size() >= 0x20u) {
        // Stage the vector page the KBL's VBAR points at.  The *same* bytes belong
        // at PA 0x40000000 in the wiki's secure DRAM layout ("SKBL Reset Vector
        // (ARM entry!)"), but the model's MPCore peripheral block (SCU / global
        // timer / private timer / GIC) currently sits at 0x40000000+ and
        // Bus::find_device prefers a device over RAM, so those bytes would land in
        // SCU_CONTROL and the reset vector would read registers back instead of
        // pointers.  The block has to move out of the DRAM window first (see the
        // note on kermit::kMpcoreBase - the test suite still pins the current
        // placement); until then the ARM starts at the KBL entry, which is what the
        // reset vector's first pointer resolves to anyway.
        for (size_t i = 0; i < boot_vectors.size(); ++i) {
            arm_bus_->write8(board::kArmVectorPage + static_cast<u32>(i), boot_vectors[i]);
        }
        // Round 94: the KBL re-maps VA 0x16100 onto its own DRAM page
        // (`vpa 0x16100` = PA 0x40000100) and what it has there is only two vector
        // words plus data, so its exception entries land in zeros and the core ends
        // up in an exception storm.  Keep the bytes so the model can restore them
        // at whatever mapping the KBL installed, the first time the core enters the
        // vector window (see satisfy_arm_boot_pc).
        kbl_vectors_ = boot_vectors;
        kbl_vectors_restored_ = false;
        ZLB_LOG_INFO("machine", "vector page staged at PA 0x%08X (%zu bytes); ARM entry 0x%08X",
                     board::kArmVectorPage, boot_vectors.size(), arm_entry);
        add_milestone("ARM vector page staged at 0x" + hex(board::kArmVectorPage, 8) +
                      " (development substitution)");
    }
    for (int i = 0; i < kArmCoreCount; ++i) {
        Cpu* core = arm_cores_[static_cast<size_t>(i)].get();
        if (!core) continue;
        core->reset(arm_entry);
        core->prepare_reset_context(0, 0, 0, 0);
        core->halted = false;
    }
    boot_.stage = BootStage::ArmKernelBootLoader;
    boot_.arm_entry = arm_entry;
    boot_.arm_released = true;
    boot_.detail = "ARM released on the kernel boot loader";
    add_milestone("ARM started on kernel_boot_loader at 0x" + hex(kbl_entry_, 8) + " (" + source + ")");
    return true;
}

bool Vita::start_kernel() {
    if (!arm_) return false;

    // The kernel modules live in the os0 FAT16 partition. When the kernel boot
    // loader cannot get there yet, the stage command loads the first kernel
    // module directly so the rest of the machine can be exercised.
    const char* candidates[] = {
        "Vita_104_Firmware/Out/fs_dec/os0/kd/bootimage.elf",
        "Vita_104_Firmware/Out/fs_dec/os0/kd/sysmem.elf",
        "Vita_104_Firmware/Out/fs_dec/os0/kd/threadmgr.elf",
    };

    for (const char* relative : candidates) {
        std::string path = resolve_workspace_path(relative);
        auto data = read_file(path);
        if (!data) continue;

        LoadResult result = load_image(*arm_bus_, *data, path_filename(path), keys_);
        if (!result.ok) {
            ZLB_LOG_WARN("machine", "kernel module %s: %s", relative, result.message.c_str());
            continue;
        }
        kernel_entry_ = result.entry;
        arm_->reset(kernel_entry_);
        arm_->halted = false;
        kernel_started_ = true;
        boot_.stage = BootStage::KernelEntry;
        boot_.arm_entry = kernel_entry_;
        boot_.detail = std::string("kernel module started: ") + path_filename(path);
        add_milestone("kernel entry reached: " + path_filename(path) + " at 0x" + hex(kernel_entry_, 8));
        return true;
    }

    ZLB_LOG_ERROR("machine", "no kernel module could be loaded - run the kernel boot loader first");
    return false;
}

bool Vita::cmep_pc_hook(u32 pc) {
    // The second loader finishes by clearing its register file, loading $1..$5
    // with Bigmac/mailbox addresses and jumping to the first loader's service
    // entry point at 0x5FF00 (docs/KBL.md round 39).  That entry lives in the
    // first loader's heap/stack area, so the ROM routine behind it is not in any
    // dump we have - intercept the call and substitute the service instead.
    static const bool disabled = [] {
        const char* value = std::getenv("ZLB_NO_SUBSTITUTION");
        return value != nullptr && value[0] != '0';
    }();
    if (disabled) return false;
    // The secure kernel's success path ends by jumping back into the 0x40000 window
    // (it re-enters its caller with "done"), which on hardware is the point where
    // it has told the syscon to reset the ARM at 0x00000000 (wiki, step 4).  Our
    // model has no SC path for that message yet, so the jump is turned into the
    // syscon release directly.  The test is "the secure kernel is the current
    // image" rather than the boot stage: by the time the first instruction at
    // 0x40000 runs, poll_boot_chain() has already relabelled the stage from the PC.
    if (secure_kernel_active_ && pc >= board::kSecondLoaderStaging && pc < board::kFirstLoaderBase) {
        secure_kernel_done_ = true;
        return true;
    }
    if (boot_.stage == BootStage::CmepSecondLoader && pc == board::kFirstLoaderServiceEntry) {
        cmep_service_pending_ = true;
        return true;
    }
    return false;
}

bool Vita::serve_cmep_service_call() {
    if (!cmep_service_pending_) return false;
    cmep_service_pending_ = false;
    // Per the wiki's boot sequence this call is step 4: "the Second Loader resets
    // itself with a pointer to secure_kernel.enp" - i.e. the very step that hands
    // the CMeP over to the secure kernel, which then releases the ARM.
    ZLB_LOG_INFO("boot",
                 "second loader called the first-loader service at 0x%05X (development substitution: "
                 "restarting the CMeP into secure_kernel.enp)",
                 board::kFirstLoaderServiceEntry);
    add_milestone("second loader called the first-loader service at 0x5FF00 -> CMeP restarted into "
                  "secure_kernel.enp (development substitution)");
    return load_cmep_secure_kernel();
}

bool Vita::enter_stage(BootStage stage) {
    if (!built_) build();
    switch (stage) {
        case BootStage::CmepFirstLoader:
            cmep_->reset(config_.first_loader_base);
            cmep_->prepare_reset_context(board::kCmepStackTop, 0x00040000, 0, 0);
            cmep_->halted = false;
            boot_.stage = stage;
            return true;
        case BootStage::CmepSecondLoader:
            return load_second_loader_direct();
        case BootStage::CmepSecureKernel:
            return load_cmep_secure_kernel();
        case BootStage::ArmKernelBootLoader:
            return start_arm_kernel_boot_loader();
        case BootStage::KernelEntry:
        case BootStage::KernelRunning:
            return start_kernel();
        default:
            ZLB_LOG_WARN("machine", "cannot jump to stage %s", to_string(stage));
            return false;
    }
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

void Vita::poll_boot_chain() {
    if (!built_) return;

    const u32 cmep_pc = cmep_ ? cmep_->get_pc() : 0;
    boot_.cmep_status = cmep_block_ ? cmep_block_->cmep_status() : 0;

    // ARM release: the syscon or the CMeP has signalled that the SoC may start.
    //
    // Round 92: the syscon's power-on path releases the SoC immediately
    // (ernie_power.cpp: "the syscon releases the SoC as soon as the reset
    // sequencing is done"), which is true for the *ARM boot ROM* but not for
    // kernel_boot_loader: on hardware the boot ROM waits for the CMeP, whose
    // second loader is what leaves the boot context in the scratchpad.  Starting
    // the KBL at power-on made it read the scratch *before* the second loader had
    // finished with it (the model's record at +0xC0..+0x100 was cleared by the
    // second loader's memset at 0x40ABC, so the KBL saw zeroes).  Hold the KBL
    // back until the CMeP's context phase is over, with a slice budget as a
    // safety net.  `ZLB_ARM_WAIT_CMEP=0` restores the old timing.
    if (!boot_.arm_released && arm_ && arm_->halted) {
        static const bool wait_for_cmep = [] {
            const char* value = std::getenv("ZLB_ARM_WAIT_CMEP");
            return value == nullptr || value[0] != '0';
        }();
        if (!cmep_context_done_) {
            ++arm_wait_slices_;
            const bool budget_out = arm_wait_slices_ >= kArmWaitCmepSlices;
            if (!wait_for_cmep || budget_out) {
                cmep_context_done_ = true;
                ZLB_LOG_INFO("boot", "ARM release: not waiting for the CMeP boot context (%s, %llu slices)",
                             wait_for_cmep ? "budget reached" : "ZLB_ARM_WAIT_CMEP=0",
                             static_cast<unsigned long long>(arm_wait_slices_));
            }
        }
        if (cmep_context_done_ && ernie_ && ernie_->soc_released()) {
            start_arm_kernel_boot_loader();
        }
    }

    if (boot_.stage == BootStage::ArmBootRom || boot_.stage == BootStage::PowerOn) {
        if (cmep_pc >= config_.first_loader_base) {
            boot_.stage = BootStage::CmepFirstLoader;
            boot_.detail = "CMeP first loader executing";
            add_milestone("CMeP first loader entered at 0x" + hex(cmep_pc, 5));
        }
    }

    if (boot_.stage == BootStage::CmepFirstLoader) {
        if (boot_.cmep_status == 1) {
            add_milestone("first loader reported SUCCESS to the ARM mailbox");
            boot_.detail = "first loader reported success";
        } else if (boot_.cmep_status == 2) {
            boot_.stage = BootStage::Failed;
            boot_.detail = "first loader reported FAILURE to the ARM mailbox";
            // The loader's failure path is a spin loop, so its PC is the only clue
            // to *which* check rejected the staged image (see docs/KBL.md, the
            // retail build rejects what the prototype build accepts).
            ZLB_LOG_ERROR("machine", "first loader failure path taken (mailbox = 2) at pc=%s",
                          hex(cmep_pc, 5).c_str());
            return;
        }
    }

    // Once the first loader hands control to the staged image, the CMeP runs the
    // second loader out of its own RAM window below the first loader.
    if (cmep_pc < config_.first_loader_base && cmep_pc >= 0x00040000) {
        if (boot_.stage != BootStage::CmepSecondLoader) {
            boot_.stage = BootStage::CmepSecondLoader;
            boot_.detail = "CMeP second loader executing at 0x" + hex(cmep_pc, 5);
            add_milestone("CMeP handed off to the staged image at 0x" + hex(cmep_pc, 5));
        }
    }

    // The second loader's own end: it calls the first loader's service at 0x5FF00.
    // The MeP core stopped on the service entry (Vita::cmep_pc_hook), so the
    // substitution runs here, between slices.
    if (cmep_service_pending_) {
        if (!serve_cmep_service_call()) {
            boot_.stage = BootStage::Failed;
            boot_.detail = "could not substitute the first-loader service call";
            return;
        }
        boot_.cmep_status = cmep_block_ ? cmep_block_->cmep_status() : 0;
    }

    // The secure kernel's success path re-enters the 0x40000 window (see
    // Vita::cmep_pc_hook): that jump goes back into the *second loader*, whose
    // remaining work is what fills the ARM boot context (the second loader copies
    // 1 KiB from DRAM to scratch+0x100, image 0x40A86, and clears the scratch
    // header - which is also what our traces saw at SPAD+0x100).  On hardware the
    // syscon releases the ARM only after the CMeP is done, so the model keeps the
    // CMeP running for a bounded budget first: halting it at the jump lost the
    // context and left kernel_boot_loader without its objects.
    if (secure_kernel_done_) {
        secure_kernel_done_ = false;
        secure_kernel_active_ = false;
        if (substitutions_enabled_static()) {
            cmep_finish_pending_ = true;
            cmep_finish_steps_ = 0;
            ZLB_LOG_INFO("boot",
                         "secure kernel finished (jump back to 0x40000): letting the second loader finish "
                         "before the ARM is released");
            add_milestone("secure kernel finished -> second loader continues (ARM boot context)");
        } else {
            cmep_->halted = true;
            if (ernie_) ernie_->release_soc();
            cmep_context_done_ = true;   // no CMeP continuation is modelled (round 92)
            ZLB_LOG_INFO("boot",
                         "secure kernel finished (jump back to 0x40000): substituting the syscon handshake, "
                         "releasing the ARM");
            add_milestone("CMeP secure kernel finished -> syscon SoC release (development substitution)");
        }
    }

    // Bounded continuation: the second loader's post-processing runs, then the ARM
    // is released (either because the CMeP stops by itself or when the budget ends).
    if (cmep_finish_pending_ && !boot_.arm_released) {
        ++cmep_finish_steps_;
        const bool stopped = cmep_ == nullptr || cmep_->halted;
        if (stopped || cmep_finish_steps_ >= kCmepFinishBudget) {
            cmep_finish_pending_ = false;
            if (cmep_) cmep_->halted = true;
            if (ernie_) ernie_->release_soc();
            cmep_context_done_ = true;   // the ARM may start now (round 92)
            ZLB_LOG_INFO("boot",
                         "second loader finished after the secure kernel (%s, %llu steps): releasing the ARM",
                         stopped ? "it stopped by itself" : "budget reached",
                         static_cast<unsigned long long>(cmep_finish_steps_));
            add_milestone("CMeP second-loader post-processing done -> syscon SoC release");
        }
    }

    // The secure kernel starts by waiting for the CMeP->ARM status word to be
    // consumed: its loop at 0x008003C0 is
    //     $3 = 0xE0000000; erepeat; lw $2,($3); $2 &= 0xFFFF; beqz $2, <continue>
    // so somebody on the ARM side has to read it back as zero before it goes on.
    // That "somebody" is the ARM's boot ROM, which our model stands in for.
    if (boot_.stage == BootStage::CmepSecureKernel && boot_.cmep_status != 0u) {
        const u32 status = boot_.cmep_status;
        cmep_block_->set_cmep_status(0);
        boot_.cmep_status = 0;
        // The secure kernel's start-up is a handshake: it clears the ARM->CMeP
        // register (writes 0xFFFFFFFF to 0xE0000010 at 0x8003FA), posts a status
        // (0x101 at 0x800408) and then parses the answer; the check at 0x800430 is
        //     bnei $8, 0x1, <post 0x802F and idle>
        // so the answer has to be 1 for it to continue.  The ARM's boot ROM is
        // what answers on hardware.
        cmep_block_->set_arm_to_cmep_command(1);
        // The CMeP's handshake is interrupt driven: after posting 0x102 the secure
        // kernel waits for its own handler 0x80099E (reached from the dispatch
        // table at 0x800B8A) to set the state variable at `$gp - 32748` to 9
        // (`bnei $0,0x9,<idle>` at 0x800462).  The MeP core does not model
        // interrupt delivery yet, so the ARM's reply is turned into that event
        // directly; the address comes from the firmware's own $gp, which the
        // machine reads from the core.
        u64 gp = 0;
        if (cmep_->get_register("gp", gp) && gp != 0u) {
            cmep_bus_->write32(static_cast<u32>(gp) - 32748u, 9u);
        }
        // ... and wake the core: the wait loop is
        //     0x80045E  bsr 0x801D5A          ; read the state
        //     0x800462  bnei $0, 0x9, 0x800488
        //     0x800488  sleep                 ; wait for the reply
        //     0x80048A  bra 0x80045E
        // so the secure kernel is *asleep* when the answer arrives.  Without the
        // wake the state variable is set but never read again, the secure kernel
        // never reaches the code that builds the ARM boot context (the structure
        // with the 0x61CE6649 signature at DRAM+0xA0 which the second loader then
        // copies into the scratchpad), and kernel_boot_loader finds an empty
        // context.  This stands in for the CMeP interrupt the mailbox raises.
        if (cmep_->halted) {
            cmep_->halted = false;
            ZLB_LOG_INFO("boot", "woke the CMeP for the handshake reply (MeP interrupt not modelled)");
            add_milestone("CMeP woken for the handshake reply (development substitution)");
        }
        ZLB_LOG_INFO("boot",
                     "ARM boot ROM consumed the CMeP status 0x%X and answered 0xE0000010 <- 1 (handshake)",
                     status);
        add_milestone("ARM boot ROM answered the CMeP handshake (status 0x" + hex(status, 3) + ")");
    }

    if (kernel_started_ && !kernel_running_ && arm_ && arm_->instructions > 0) {
        kernel_running_ = true;
        boot_.stage = BootStage::KernelRunning;
        boot_.detail = "kernel executing";
        add_milestone("kernel is executing instructions");
    }
}

bool Vita::kernel_started() const { return kernel_started_; }
bool Vita::kernel_running() const { return kernel_running_; }

std::vector<std::string> Vita::plan_boot() {
    // Authoritative sequence: wiki.henkaku.xyz/vita/Boot_Sequence (Boot Process),
    // cross-checked with dumps/bootrom_analysis/ANALYSIS.md.  The workspace file
    // `dumps/vita_prototype_bootrom.bin` is the prototype **first loader** (a MeP
    // executable run by the CMeP at 0x5C000) - the name says "bootrom", but the
    // wiki's First Loader page is explicit that first_loader is *not* the BootROM;
    // the on-chip CMeP boot ROM itself is not dumped anywhere in the workspace.
    std::vector<std::string> plan;
    plan.push_back("1. Syscon powers up DRAM, builds the boot context and turns Kermit on,");
    plan.push_back("   which starts the CMeP");
    plan.push_back("2. CMeP boot ROM (on-chip, never dumped; the wiki *also* nicknames it \"First");
    plan.push_back("   Loader\", which is where the name confusion comes from) is the first code that");
    plan.push_back("   runs.  Something before first_loader must place the 16 KiB first-loader image");
    plan.push_back("   into the CMeP RAM window at 0x5C000: ANALYSIS.md proves that window is RAM");
    plan.push_back("   (.bss at 0x5EB00 is cleared, the entry word is overwritten with 0 at hand-off,");
    plan.push_back("   and pch-5c-cold_first_loader.bin is a post-boot RAM snapshot), but *how* the");
    plan.push_back("   image gets there (ROM self-copy vs. an eMMC/SD read) is documented nowhere");
    plan.push_back("   and cannot be checked without a ROM dump.  first_loader itself only relocates");
    plan.push_back("   a 32-byte stub (0x5C098 -> 0x5FFE0), zeroes its own entry point and hands off");
    plan.push_back("   to the next stage staged at 0x40000");
    plan.push_back("3. second_loader (MeP): SMI/idstorage checks, keyring+Bigmac+Bignum chain,");
    plan.push_back("   eMMC bring-up; decrypts kernel_boot_loader.self into DRAM (it holds the");
    plan.push_back("   secure kernel bootloader + TrustZone kernel + NSKBL), loads kprx_auth_sm");
    plan.push_back("   and prog_rvk, then restarts itself with a pointer to secure_kernel.enp");
    plan.push_back("4. CMeP restarts into secure_kernel.enp (F00D) and tells the syscon to");
    plan.push_back("   reset the ARM at 0x00000000 (CMeP scratch buffer)");
    plan.push_back("5. ARM secure world: secure kernel bootloader decompresses the TrustZone");
    plan.push_back("   kernel, sets VBAR/MVBAR, then decompresses NSKBL, sets NS in SCR and");
    plan.push_back("   jumps into it in SVC mode");
    plan.push_back("6. ARM non-secure world: NSKBL brings up eMMC again and starts");
    plan.push_back("   os0:psp2bootconfig.skprx, then SceSysStateMgr maps the SceKernelBootimage");
    plan.push_back("   modules to os0:kd/ (\"the kernel\") and loads the rest");
    plan.push_back("7. SceShell (LiveArea), then userland");
    plan.push_back("");
    plan.push_back("model: step 2 is a development substitution - the missing ROM is not emulated, the");
    plan.push_back("dumped first loader is loaded straight into the 0x5C000 window, and second_loader is");
    plan.push_back("fed through the first loader's *own documented* boot mode 'A' (0x41, bit0 = image");
    plan.push_back("comes from ARM via mailbox 0xE0000010) rather than by the ROM reading eMMC; step 3");
    plan.push_back("currently stops inside the per-console SCE chain; `stage kbl` jumps straight to");
    plan.push_back("step 5, and TrustZone (NS/SCR/MVBAR/secure banked state) is not modelled yet, so");
    plan.push_back("only the non-secure half of KBL can work.");
    plan.push_back("");
    plan.push_back("model (round 40): the second loader's own end - an absolute `jmp 0x5FF00`");
    plan.push_back("with $1..$5 loaded with Bigmac/mailbox addresses - is intercepted by a MeP");
    plan.push_back("pc hook, and the ROM routine behind it is substituted by a restart of the");
    plan.push_back("CMeP into secure_kernel.enp at its link address 0x800000 (step 3 -> 4).");
    plan.push_back("The ARM side of the mailbox handshake (0xE0000000 consumed, 0xE0000010");
    plan.push_back("answered) is modelled by the ARM boot ROM stand-in.  The secure kernel");
    plan.push_back("then posts 0x9/0x101/0x802F and idles: the reply its parser at 0x8018B2");
    plan.push_back("expects is not reproduced yet, which is where step 4 stops.");
    return plan;
}

std::string Vita::boot_report() const {
    std::string out;
    out += format("stage        : %s\n", to_string(boot_.stage));
    out += format("detail       : %s\n", boot_.detail.c_str());
    if (cmep_) out += format("CMeP         : pc=%s insns=%llu halted=%d\n", hex(cmep_->get_pc(), 5).c_str(),
                             (unsigned long long)cmep_->instructions, cmep_->halted ? 1 : 0);
    if (arm_) out += format("ARM          : pc=%s insns=%llu halted=%d entry=0x%08X\n",
                            hex(arm_->get_pc(), 8).c_str(), (unsigned long long)arm_->instructions,
                            arm_->halted ? 1 : 0, boot_.arm_entry);
    if (syscon_) out += format("Ernie        : pc=%s insns=%llu\n", hex(syscon_->get_pc(), 5).c_str(),
                               (unsigned long long)syscon_->instructions);
    if (cmep_block_) out += format("first loader : status=0x%X keyring writes=%llu bigmac ops=%llu sc transfers=%llu\n",
                                   cmep_block_->cmep_status(), (unsigned long long)cmep_block_->keyring_writes(),
                                   (unsigned long long)cmep_block_->bigmac_operations(),
                                   (unsigned long long)cmep_block_->sc_transfers());
    if (ernie_) out += format("SC commands  : %llu served\n", (unsigned long long)ernie_->commands_served());
    out += format("emulated time: %.4f s\n", emulated_seconds());
    if (!milestones_.empty()) {
        out += "milestones   :\n";
        for (const auto& milestone : milestones_) out += "  * " + milestone + "\n";
    }
    return out;
}

}  // namespace zlb
