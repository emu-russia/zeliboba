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
        case BootStage::NskblEntry: return "nskbl";
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
/// Heap arena (round 180): blocks handed to NSKBL's general allocator while the map
/// object's heap pointer (`[[0x5113B5AC] + 0x8C]`) is zero, so the code that creates
/// the heap (absent from the model's flow) does not abort the class installer.
constexpr u32 kHeapArenaVa = 0x01400000u;
constexpr u32 kHeapArenaSize = 0x00100000u;       // 1 MiB

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
    // Diagnostic (ZLB_ARM_FAULT_LOG=1): the *unsubstituted* behaviour is what an
    // honest run needs to see, and with ZLB_NO_SUBSTITUTION=1 this function returns
    // before its own trace, so the fault that reaches the guest's abort handler is
    // invisible.  This branch only logs - it never repairs the mapping.
    if (disabled) {
        static const bool fault_log = [] {
            return std::getenv("ZLB_ARM_FAULT_LOG") != nullptr;
        }();
        static u32 logged = 0;
        if (fault_log && logged < 24u && core < static_cast<u32>(kArmCoreCount)) {
            ++logged;
            ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get());
            u32 dfsr = 0, ifsr = 0, dfar = 0, ifar = 0, sctlr = 0, ttbr0 = 0, ttbr1 = 0;
            if (arm != nullptr) {
                dfsr = arm->mmu.dfsr;
                ifsr = arm->mmu.ifsr;
                dfar = arm->mmu.dfar;
                ifar = arm->mmu.ifar;
                sctlr = arm->mmu.sctlr;
                ttbr0 = arm->mmu.ttbr0;
                ttbr1 = arm->mmu.ttbr1;
            }
            // The page-table state at the fault decides what the guest was entitled
            // to: `l1` is the first-level entry the faulting VA resolves through (in
            // whichever of TTBR0/TTBR1 TTBCR selects for that VA), so a zero `l1`
            // means "the guest never described this VA" and a non-zero one points at
            // the descriptor that was rejected instead.
            u32 l1 = 0;
            u32 l2 = 0;
            if (arm != nullptr && arm->mmu.enabled()) {
                const bool ttbr1_range = va >= 0x40000000u && (arm->mmu.ttbcr & 7u) != 0u;
                const u32 ttbr = ttbr1_range ? arm->mmu.ttbr1 : arm->mmu.ttbr0;
                const u32 l1_base = ttbr & 0xFFFFC000u;
                l1 = arm_bus_->read32(l1_base + ((va >> 20) & 0xFFFu) * 4u);
                if ((l1 & 3u) == 1u) {
                    l2 = arm_bus_->read32((l1 & 0xFFFFFC00u) + ((va >> 12) & 0xFFu) * 4u);
                }
            }
            ZLB_LOG_INFO("machine",
                         "ARM fault (unsubstituted) arm%u VA=0x%08X %s pc=0x%08X sctlr=0x%08X M=%d "
                         "ttbr0=0x%08X ttbr1=0x%08X ttbcr=0x%X L1=0x%08X L2=0x%08X "
                         "dfsr=0x%X ifsr=0x%X dfar=0x%08X ifar=0x%08X [%u]",
                         core, va, fetch ? "fetch" : (write ? "write" : "read"),
                         arm != nullptr ? arm->get_pc() : 0u, sctlr,
                         arm != nullptr ? (arm->mmu.enabled() ? 1 : 0) : -1, ttbr0, ttbr1,
                         arm != nullptr ? arm->mmu.ttbcr : 0u, l1, l2, dfsr, ifsr,
                         dfar, ifar, logged);
        }
        return false;
    }
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
    Cpu* cpu = arm_cores_[core].get();
    ArmCore* arm = dynamic_cast<ArmCore*>(cpu);
    if (!arm || !arm->mmu.enabled()) return false;

    // Diagnostic (ZLB_SECURE_FAULT_LOG=1): the secure stage's repairs below are
    // skipped for the *unsubstituted* run, so a fault there is invisible - and when
    // the secure low window is left to the guest it is exactly what has to be seen.
    // This only logs; nothing is repaired here.
    static const bool secure_fault_log = [] {
        return std::getenv("ZLB_SECURE_FAULT_LOG") != nullptr;
    }();
    static u32 secure_faults = 0;
    if (secure_fault_log && secure_faults < 24u) {
        ++secure_faults;
        ZLB_LOG_INFO("machine",
                     "ARM fault arm%u: VA=0x%08X %s pc=0x%08X lr=0x%08X secure=%d ttbr0=0x%08X "
                     "ttbr1=0x%08X dfsr=0x%X dfar=0x%08X ifsr=0x%X ifar=0x%08X [%u]",
                     core, va, fetch ? "fetch" : (write ? "write" : "read"),
                     arm->get_pc(), static_cast<u32>(arm->r[14]), arm->secure_state() ? 1 : 0,
                     arm->mmu.ttbr0, arm->mmu.ttbr1, arm->mmu.dfsr, arm->mmu.dfar, arm->mmu.ifsr,
                     arm->mmu.ifar, secure_faults);
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
    // Round 219 (measured, reverted): the guess that this substitution is what
    // captures the partition's node array at VA 0x00100000 does not hold.  Skipping it
    // for the non-secure stage - so NSKBL's own abort handler sees the fault - leaves
    // the run bit-identical (114 758 543 instructions, checkpoint 0xAD), and the write
    // trap shows no store at PA 0x40100000 (the page this rule would pick for
    // VA 0x00100000).  The array's physical page is therefore not the one this branch
    // installs; see docs/NSKBL.md 8.55 for what is actually known.
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
    const u32 existing = arm_bus_->read32(slot);

    // Substitution (round 213): NSKBL's own low window.  The kernel boot loader
    // hands the non-secure stage a table that describes VA 0x30000-0x3FFFF as a
    // 64 KiB *large page* whose AP field is 00, i.e. "no access" for every mode
    // (0x4034941D: large page, base PA 0x40340000, XN, AP = 0).  NSKBL's own
    // allocator hands out memory from that part of the window and the first store
    // raises a *permission* fault (DFSR 0x80F, DFAR 0x00030000), which is why the
    // earlier rounds saw the fault hook never run: the address *is* described, so
    // there is no translation miss to repair.  The rest of the window (VA
    // 0x4000-0x2FFFF) consists of small pages following the model's rule
    // PA = VA + 0x40300000, so replace the large page with sixteen small pages of
    // that same rule - the "fix the table entry" the round-203 analysis asked for,
    // triggered by the fault itself instead of by a pc.  Only while the non-secure
    // NSKBL runs; the secure KBL keeps the page it built.
    // ZLB_NSKBL_LOWWIN=0 and ZLB_NO_SUBSTITUTION=1 disable it.
    static const bool nskbl_lowwin = [] {
        const char* off = std::getenv("ZLB_NO_SUBSTITUTION");
        if (off != nullptr && off[0] != '0') return false;
        const char* on = std::getenv("ZLB_NSKBL_LOWWIN");
        return on == nullptr || on[0] != '0';
    }();
    if (existing != 0u) {
        if (!nskbl_lowwin) return false;
        if (arm->secure_state()) return false;
        if ((existing & 3u) != 1u) return false;                 // not a large page
        // Only the permission fields change: the KBL's physical base (bits [31:16])
        // is kept exactly as it built it, because it is the only record of where
        // that part of the window really lives on hardware.  For a level-2 large
        // page the model decodes AP from bits [5:4] with the ARMv5 subpage table
        // (`check_large_ap`), where AP = 01 denies subpage 0 - exactly the 16 KiB
        // block NSKBL writes into (DFAR 0x00030000).  AP = 10 grants privileged
        // read/write on every subpage and leaves the user read-only.
        //
        // Round 216: an instruction *fetch* needs XN (bit 15) cleared as well, and the
        // KBL sets it on all of these large pages.  That is what turned NSKBL's own
        // abort into an endless loop once the table substitution let it run far
        // enough to fault: the non-secure vector base is VA 0x40100 (VBAR_NS), so the
        // data-abort vector lands inside the VA 0x40000-0x7FFFF large page, the fetch
        // there raises a *permission* fault of its own (IFSR 0xF, IFAR 0x4010C) and
        // the handler can never report anything.  A page that holds exception vectors
        // has to be executable, so clear XN only for the fetch case.
        u32 patched = (existing & ~0x30u) | (2u << 4);
        if (fetch) patched &= ~(1u << 15);
        arm_bus_->write32(slot, patched);
        ++boot_fault_fixes_;
        ZLB_LOG_INFO("machine",
                     "NSKBL low window: arm%u large page 0x%08X for VA 0x%08X granted "
                     "privileged access (AP field -> 2%s, now 0x%08X; development substitution)",
                     core, existing, va, fetch ? ", XN cleared for the fetch" : "", patched);
        add_milestone("NSKBL low window large page made writable (development substitution)");
        return true;
    }

    u32 pa = va;
    if (low_map == 1 && va >= 0x40000u) pa = 0x40000000u + (va - 0x40000u);
    else if (low_map == 2 && va >= 0x40000u) pa = 0x40000000u + va;
    else if (low_map == 3 && va >= 0x40000u) pa = 0x40118000u + (va - 0x40000u);
    else if (!arm->secure_state() && va >= 0x10000u && va < 0x40000u) {
        // Round 233: the same correction as below, for the second quarter of the low
        // window.  The window rule above (0x40118000 + VA - 0x40000) is the *KBL's*
        // MeP boot mirror; NSKBL's own tables place this range at VA + 0x40310000 -
        // measured with `vpa`: VA 0x10000 -> PA 0x40320000, 0x20000 -> 0x40330000,
        // 0x30000 -> 0x40340000, which is exactly what the KBL's large-page descriptors
        // there hold (0x4032941D for VA 0x10000, 0x4033941D for 0x20000, 0x4034941D for
        // 0x30000 - the round-213 patch only changes their AP field).  Use the guest's
        // rule for the non-secure stage; ZLB_NSKBL_LOWIDENT=2 keeps the window rule.
        static const char* const window_rule = std::getenv("ZLB_NSKBL_LOWIDENT");
        if (window_rule == nullptr || window_rule[0] != '2') pa = va + 0x40310000u;
    }
    else if (!arm->secure_state() && va < 0x10000u) {
        // Round 231: the identity rule above belongs to the *secure* KBL, whose low
        // window really is at PA 0 (= VA) - its vector page is the model's own example
        // (VA 0x16100 -> PA 0x16100).  NSKBL's low window is relocated: measured with
        // `vpa`, VA 0x47C0 -> 0x403047C0, 0x4900 -> 0x40304900, 0x5300 -> 0x40305300,
        // 0x60C0 -> 0x403060C0, 0x6940 -> 0x40306940, i.e. PA = VA + 0x40300000, and
        // every other NSKBL substitution in this file assumes exactly that rule
        // (boot-config VA 0x47C0 -> PA 0x403047C0, the partition at VA 0x5300, the
        // physical pool, ...).  Substituting *identity* for the handful of low VAs the
        // guest's own tables leave unmapped therefore hands NSKBL the CMeP scratch
        // mirror instead of its own low window: the run logs five such mappings
        // (VA 0x10, 0x38, 0x10B0, 0x19FF8, 0x1A12C) and the guest then reads words
        // there and uses them as pointers (the garbage-pointer aborts this file has been
        // chasing since round 2: DFAR 0x656C7398 / 0xF8D1B178).  Use the rule the guest
        // itself uses.  ZLB_NSKBL_LOWIDENT=1 restores identity.
        // Round 232: it has to cover the whole first 64 KiB.  Restricting it (identity
        // for VA < 0x1000, so that the guest would read the ARM boot alias at VA 0)
        // brings the panic 0xAD straight back: `VA 0x10 -> PA 0x10` hands NSKBL the CMeP
        // scratch mirror (0xFFFFFFFF), which it then uses as a pointer.  Measured both
        // ways; the panic-free rule is the one below.
        static const bool low_identity = [] {
            const char* value = std::getenv("ZLB_NSKBL_LOWIDENT");
            return value != nullptr && value[0] != '0';
        }();
        if (!low_identity) pa = va + 0x40300000u;
    }

    // Substitution gate (round 230): optionally let an instruction fetch in the low
    // window reach the *non-secure* stage's own abort handler instead of being papered
    // over.  Measured in the `ZLB_NSKBL_PHYSPOOL=1` state: the core fetches 0x40110,
    // 0x41000, 0x42000, ... and the substitution maps every one of them on the fly (98
    // substitutions), after which the core runs through the zeros behind those pages for
    // tens of millions of instructions - a silent wander.  With
    // ZLB_NSKBL_FETCH_FAULT=1 the fault reaches NSKBL's handler (round 216 mirrors its
    // vector page into the physical page VBAR_NS points at, below) and the state becomes
    // diagnosable: IFSR = 7 / IFAR = 0x41000 (page translation fault) on top of the
    // original data abort (DFSR = 5, DFAR = 0xF8D1B178).  The handler then retries the
    // same instruction, so the run loops instead of advancing - hence opt-in, off by
    // default, with the documented state unchanged.
    static const bool deliver_fetch_faults = [] {
        const char* value = std::getenv("ZLB_NSKBL_FETCH_FAULT");
        return value != nullptr && value[0] != '0';
    }();
    const bool vector_page =
        fetch && !arm->secure_state() && arm->vbar_nonsecure != 0u &&
        (va & 0xFFFFF000u) == (arm->vbar_nonsecure & 0xFFFFF000u);
    if (deliver_fetch_faults && fetch && !arm->secure_state() && !vector_page) {
        return false;
    }

    // Take the attribute bits from the KBL's own first entry so the substituted
    // page has the same cacheability/permissions as the pages it installed.
    u32 attributes = arm_bus_->read32(l2_base) & 0xFFFu;
    if ((attributes & 3u) != 2u) attributes = 0x47Eu;           // small page, AP=11
    arm_bus_->write32(slot, (pa & 0xFFFFF000u) | attributes);

    // Substitution (round 216): the *non-secure vector page*.  NSKBL writes VBAR
    // twice - first 0x51000000 (its own image's vector table, `mcr` at 0x51000388)
    // and then VA 0x00040100 (`mcr` at 0x5100049C) - and the model has nothing there:
    // the low-window rule backs VA 0x40000+ with the KBL's own copy of the low 32 KiB
    // (PA 0x40118000+), which holds no NSKBL vectors.  So the first abort lands on
    // foreign code and the handler either faults again or loops, and NSKBL's own
    // checkpoint is never written.  The model already mirrors a vector page for the
    // KBL ("SKBL vector page mirrored to PA 0x00016100"); do the same for the
    // non-secure base: copy the 0xC0-byte ARM vector table that starts the NSKBL image
    // into the page VBAR_NS points at.  The table is position independent
    // (`ldr pc,[pc,#0x18]` plus absolute literals), so it works at any address.
    if (vector_page) {
        const u32 destination = pa & 0xFFFFF000u;
        for (u32 i = 0; i < 0xC0u; ++i) {
            arm_bus_->write8(destination + i, arm_bus_->read8(0x51000000u + i));
        }
        ++boot_fault_fixes_;
        ZLB_LOG_INFO("machine",
                     "NSKBL vector page mirrored to PA 0x%08X (VBAR_NS 0x%08X, %u bytes; "
                     "development substitution)",
                     destination, arm->vbar_nonsecure, 0xC0u);
        add_milestone("NSKBL non-secure vector page mirrored (development substitution)");
    }

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
    // Diagnostic (ZLB_SKBL_ORDER_LOG=1): the low-window entries are planted by the
    // loader's own code at 0x40029F34/0x4002DBD0, so the order between those stores
    // and the first access that needs them (0x4002F074, a read of VA 0x34) is what
    // separates an honest run from a substituted one.  Print both events with the
    // callers so the divergent path can be named.
    static const bool order_log = [] {
        return std::getenv("ZLB_SKBL_ORDER_LOG") != nullptr;
    }();
    if (order_log && core < static_cast<u32>(kArmCoreCount)) {
        static u32 order_events = 0;
        static u32 enter_events = 0;
        static u32 order_29f34 = 0;
        static u32 line_events = 0;
        ArmCore* o = dynamic_cast<ArmCore*>(arm_cores_[core].get());
        if (pc == 0x40029EE0u && order_29f34 < 4u) {
            ++order_29f34;
            u32 st0 = 0, st1 = 0, st2 = 0, st3 = 0;
            if (o != nullptr) {
                u32 pa = 0;
                std::string fault;
                for (u32 i = 0; i < 4; ++i) {
                    if (o->translate(static_cast<u32>(o->r[13]) + i * 4u, false, false, pa, fault)) {
                        const u32 v = arm_bus_->read32(pa);
                        if (i == 0) st0 = v;
                        else if (i == 1) st1 = v;
                        else if (i == 2) st2 = v;
                        else st3 = v;
                    }
                }
            }
            ZLB_LOG_INFO("machine",
                         "skbl 29EE0 entry #%u: r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X r4=0x%08X "
                         "r5=0x%08X r6=0x%08X r7=0x%08X lr=0x%08X sp=0x%08X stack=%08X %08X %08X %08X",
                         order_29f34, o != nullptr ? static_cast<u32>(o->r[0]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[1]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[2]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[3]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[4]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[5]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[6]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[7]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[14]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[13]) : 0u, st0, st1, st2, st3);
        }
        if (pc == 0x40029F34u && order_29f34 < 4u) {
            ++order_29f34;
            ZLB_LOG_INFO("machine",
                         "skbl 29F34 #%u: r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X r4=0x%08X "
                         "r5=0x%08X r6=0x%08X r7=0x%08X lr=0x%08X sp=0x%08X",
                         order_29f34, o != nullptr ? static_cast<u32>(o->r[0]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[1]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[2]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[3]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[4]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[5]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[6]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[7]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[14]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[13]) : 0u);
        }
        if (pc == 0x40030D1Cu && line_events < 8u) {
            ++line_events;
            ZLB_LOG_INFO("machine",
                         "skbl line #%u: r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X r4=0x%08X "
                         "r5=0x%08X r6=0x%08X r7=0x%08X lr=0x%08X sp=0x%08X",
                         line_events, o != nullptr ? static_cast<u32>(o->r[0]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[1]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[2]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[3]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[4]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[5]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[6]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[7]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[14]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[13]) : 0u);
        }
        if (pc == 0x40031CB0u && enter_events < 8u) {
            ++enter_events;
            u32 st[4] = {0, 0, 0, 0};
            if (o != nullptr) {
                u32 pa = 0;
                std::string fault;
                for (u32 i = 0; i < 4; ++i) {
                    if (o->translate(static_cast<u32>(o->r[13]) + i * 4u, false, false, pa, fault)) {
                        st[i] = arm_bus_->read32(pa);
                    }
                }
            }
            ZLB_LOG_INFO("machine",
                         "skbl order arm%u ENTER 0x40031CB0 lr=0x%08X r0=0x%08X r1=0x%08X "
                         "r2=0x%08X sp=0x%08X stack=%08X %08X %08X %08X",
                         core, o != nullptr ? static_cast<u32>(o->r[14]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[0]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[1]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[2]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[13]) : 0u, st[0], st[1], st[2], st[3]);
        }
        if (pc == 0x4002F070u && order_events < 32u) {
            ++order_events;
            ZLB_LOG_INFO("machine", "skbl r0 at 0x4002F070 #%u: r0=0x%08X lr=0x%08X r1=0x%08X",
                         order_events, o != nullptr ? static_cast<u32>(o->r[0]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[14]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[1]) : 0u);
        }
        if ((pc == 0x40029F34u || pc == 0x4002DBD0u || pc == 0x40020394u) &&
            order_events < 32u) {
            ++order_events;
            u32 r8 = 0;
            u32 r9 = 0;
            u32 st0 = 0;
            u32 st1 = 0;
            if (o != nullptr) {
                r8 = static_cast<u32>(o->r[8]);
                r9 = static_cast<u32>(o->r[9]);
                // The walker takes its array base from r8; the first two entries say
                // whether the array the two runs walk is the same one.
                u32 pa = 0;
                std::string fault;
                if (r8 >= 0x1000u && o->translate(r8, false, false, pa, fault)) {
                    st0 = arm_bus_->read32(pa);
                    st1 = arm_bus_->read32(pa + 4u);
                }
            }
            ZLB_LOG_INFO("machine",
                         "skbl order arm%u pc=0x%08X lr=0x%08X r0=0x%08X sp=0x%08X r8=0x%08X "
                         "[r8]=%08X [r8+4]=%08X r9=0x%08X",
                         core, pc, o != nullptr ? static_cast<u32>(o->r[14]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[0]) : 0u,
                         o != nullptr ? static_cast<u32>(o->r[13]) : 0u, r8, st0, st1, r9);
        }
    }

    // Substitution (round 378): the storage device's method.  The driver calls it
    // through `[device+0x24A0]` at 0x5101D6E8 with the request in r0; no image in the
    // workspace contains it (round 377), so it is modelled in C++ - see
    // serve_nskbl_device_call().  ZLB_NSKBL_SERVICE=1 opts in; ZLB_NO_SUBSTITUTION=1
    // disables every substitution including this one.
    static const bool nskbl_service = [] {
        const char* off = std::getenv("ZLB_NO_SUBSTITUTION");
        if (off != nullptr && off[0] != '0') return false;
        const char* on = std::getenv("ZLB_NSKBL_SERVICE");
        return on != nullptr && on[0] != '0';
    }();
    if (nskbl_service && pc == 0x5101D6E8u) {
        if (serve_nskbl_device_call(core)) return true;
    }
    // Diagnostic (round 394): the os0: open fails because the validator 0x5101A4B0 rejects
    // the buffer it is given (it wants "SC-" = 53 43 2D, the buffer held "SCE\0" =
    // 53 43 45 00).  Log the buffer's first bytes at the validator's entry (r0 = buffer,
    // r1 = length) to see what string is actually being checked.  ZLB_NSKBL_VALIDATOR_LOG=1.
    static const bool validator_log = [] {
        const char* on = std::getenv("ZLB_NSKBL_VALIDATOR_LOG");
        return on != nullptr && on[0] != '0';
    }();
    if (validator_log && pc == 0x5101A4B0u && core < static_cast<u32>(kArmCoreCount)) {        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 buffer = static_cast<u32>(arm->r[0]);
            const u32 length = static_cast<u32>(arm->r[1]);
            std::string bytes;
            std::string text;
            for (u32 i = 0; i < 32u && i < length; ++i) {
                const arm::MmResult r = arm->translate_or_fix(buffer + i, false, false);
                const u8 value = r.ok ? arm_bus_->read8(r.phys_addr) : 0u;
                bytes += format(" %02X", value);
                text += (value >= 0x20 && value < 0x7F) ? static_cast<char>(value) : '.';
            }
            u32 caller = 0;
            caller = static_cast<u32>(arm->r[14]);
            ZLB_LOG_INFO("machine",
                         "NSKBL validator 0x5101A4B0: buffer 0x%08X len %u caller 0x%08X bytes%s | %s",
                         buffer, length, caller, bytes.c_str(), text.c_str());
        }
    }
    // Diagnostic (round 399): the allocator's list walk (0x5100C192-0x5100C1C4, next at
    // +0xA0) rejects a block whose `[node] & 0x30000000` is neither 0x10000000 nor
    // 0x20000000, returning 0x80024300 (built at 0x5100C20E).  Log each candidate node and
    // its flag word at the check itself (r2 = node, r1 = its word).  ZLB_NSKBL_BLOCK_LOG=1.
    static const bool block_log = [] {
        const char* on = std::getenv("ZLB_NSKBL_BLOCK_LOG");
        return on != nullptr && on[0] != '0';
    }();
    if (block_log && pc == 0x5100C1FAu && core < static_cast<u32>(kArmCoreCount)) {
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 word = static_cast<u32>(arm->r[1]);
            const u32 bits = word & 0x30000000u;
            if (bits != 0x10000000u && bits != 0x20000000u && nskbl_service_fail_logs_ < 16u) {
                ++nskbl_service_fail_logs_;
                ZLB_LOG_INFO("machine",
                             "NSKBL allocator block REJECTED: node 0x%08X word 0x%08X (bits 0x%08X) "
                             "lr 0x%08X r5 0x%08X r8 0x%08X (ZLB_NSKBL_BLOCK_LOG=1)",
                             static_cast<u32>(arm->r[2]), word, bits, static_cast<u32>(arm->r[14]),
                             static_cast<u32>(arm->r[5]), static_cast<u32>(arm->r[8]));
            }
        }
    }
    // Experiment (round 391): fix the transfer parameters at the last possible moment.
    // Measured with the command writer's own trace (ZLB_KBL_TRACE_PC=0x51022664): the
    // failing CMD18 is written for the substitution's node 0x00000300 with `[node+0x7C] = 0`
    // (no ADMA2 table, so `sdif+0x58 = 0`) and a 1-byte block size, while the driver's own
    // volume reads carry a table (0x17D480/0x17D900/0x17DD80) and size 512.  The writer is
    // called from 0x5101DAE2 with the node in r5 (its first instruction reads r5), and it
    // reads the size from its own r0, so both can be supplied there.
    static const bool nskbl_dma_fix = [] {
        const char* off = std::getenv("ZLB_NO_SUBSTITUTION");
        if (off != nullptr && off[0] != '0') return false;
        const char* on = std::getenv("ZLB_NSKBL_DMA_FIX");
        return on != nullptr && on[0] != '0';
    }();
    if (nskbl_dma_fix && pc == 0x5101DAE0u && core < static_cast<u32>(kArmCoreCount)) {
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            // 0x5101DAE0 is `blx r3` with the node in r0 (0x5101DADA `mov r0, r5`); the
            // call goes to `[[device+0x24A0]+4]` = 0x51022605, whose body (0x51022604)
            // computes the SDIF registers *from the node*.  The earlier attempt hooked
            // 0x5101DAE2 - which is the caller's `pop`, i.e. it ran after the method had
            // already read the node.
            const u32 node = static_cast<u32>(arm->r[0]);
            auto read32_at = [&](u32 va, bool& ok32) {
                const arm::MmResult r = arm->translate_or_fix(va, false, false);
                if (!r.ok) {
                    ok32 = false;
                    return 0u;
                }
                ok32 = true;
                return arm_bus_->read32(r.phys_addr);
            };
            auto write32_at = [&](u32 va, u32 value) {
                const arm::MmResult r = arm->translate_or_fix(va, true, false);
                if (!r.ok) return false;
                arm_bus_->write32(r.phys_addr, value);
                return true;
            };
            bool ok32 = false;
            const u32 command = read32_at(node + 0x08u, ok32);
            const u32 argument = read32_at(node + 0x0Cu, ok32);
            const u32 table = read32_at(node + 0x7Cu, ok32);
            const u32 buffer_va = read32_at(node + 0x20u, ok32);
            const u32 size_count = read32_at(node + 0x24u, ok32);
            const u32 size = static_cast<u32>(arm->r[0] & 0xFFFFu);
            if (ok32 && (command & 0xFFu) == 0x12u && node < 0x00100000u) {
                u32 changes = 0;
                if (table == 0u && write32_at(node + 0x7Cu, 0x510FF000u)) ++changes;
                if (size < 512u) {
                    arm->set_register("r0", 512u);
                    ++changes;
                }
                if (argument > 0x00100000u && (argument % 512u) == 0u) {
                    // Measured with tools/emmc_os0_fat.py: the os0 volume starts at LBA 65536
                    // (data area at 65608, 8 sectors per cluster) and psp2bootconfig.skprx
                    // ("PSP2BO~1") has first cluster 903 -> LBA 72816, where the card really
                    // holds a module header ("SCE\0" = 53 43 45 00).  The loader's own
                    // argument converts to LBA 72800 (16 blocks earlier, arbitrary data), so
                    // the byte-offset-to-block conversion carries a constant 16-block bias.
                    if (write32_at(node + 0x0Cu, argument / 512u + 16u)) ++changes;
                }
                // Round 392: the descriptor must point at the *driver's* buffer (the node's
                // +0x20), not at a scratch page: the open path validates the first 64 bytes
                // it read (0x5101A4B0 wants "SC-" or 0x7F '-L.'), and with the data landing
                // in the substitution's own buffer that check failed with 0x80025001.
                {
                    const u32 count = static_cast<u16>((size_count >> 16) & 0xFFFFu);
                    const u32 bytes = count != 0u ? count * 512u : 0x4000u;
                    const arm::MmResult target = arm->translate_or_fix(buffer_va, true, false);
                    if (buffer_va != 0u && target.ok) {
                        const arm::MmResult t0 = arm->translate_or_fix(0x510FF000u, true, false);
                        const arm::MmResult t1 = arm->translate_or_fix(0x510FF004u, true, false);
                        if (t0.ok && t1.ok) {
                            arm_bus_->write32(t0.phys_addr, (bytes << 16) | 0x0023u);
                            arm_bus_->write32(t1.phys_addr, target.phys_addr);
                            ++changes;
                            if (nskbl_async_bit_logs_ < 12u) {
                                ++nskbl_async_bit_logs_;
                                ZLB_LOG_INFO("machine",
                                             "NSKBL transfer for node 0x%08X: descriptor -> buffer "
                                             "0x%08X (PA 0x%08X), %u bytes (ZLB_NSKBL_DMA_FIX=1)",
                                             node, buffer_va, target.phys_addr, bytes);
                            }
                        }
                    }
                }
                if (changes != 0u && nskbl_async_bit_logs_ < 24u) {
                    ++nskbl_async_bit_logs_;
                    ZLB_LOG_INFO("machine",
                                 "NSKBL storage method call for node 0x%08X cmd 0x%02X: size %u -> 512, "
                                 "arg 0x%08X -> LBA %u, table 0x%08X -> 0x510FF000 (%u change(s), "
                                 "ZLB_NSKBL_DMA_FIX=1)",
                                 node, command & 0xFFu, size, argument, argument / 512u, table, changes);
                }
            }
        }
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
    // Round 366: NSKBL's *first external load* - the file read of psp2bootconfig.skprx
    // - never reaches the SDIF.  Measured: the boot reads the os0 boot sector and root
    // directory (LBA 0/65536/65568) and then stops, NSKBL's own checkpoint sits at 0xA9
    // ("kernel pre-init done, before first external load"), and the driver's wait
    // (0x5101FE60) pops an empty device completion list at VA 0x2640 and builds
    // 0x80320011.  The device object that list belongs to is the hard-wired VA 0x240,
    // which nothing in the chain writes.  Fill it as soon as the driver first enters
    // that wait: early enough that the object is read (it is read inside the call) and
    // late enough that the low window is mapped and no longer being cleared.
    // ZLB_NSKBL_DEV=1 opts in.
    static const bool supply_nskbl_device = [] {
        const char* value = std::getenv("ZLB_NSKBL_DEV");
        if (value != nullptr && value[0] != '0') return true;
        // The device service (round 378) needs the object too: without it the driver's
        // dispatch dereferences NULL and the service is never reached.
        const char* service = std::getenv("ZLB_NSKBL_SERVICE");
        return service != nullptr && service[0] != '0';
    }();
    // The object is read inside the wait, and the driver re-initialises its lists per
    // request (and the guest clears that page during its own table setup - measured:
    // 65 zero writes from the lock init), so re-apply it at every entry into the wait
    // rather than once.  The first attempt latched at the wait's entry and the second
    // hooked the pop call itself; both left the instruction count bit-identical
    // (767606285), so neither had taken effect where the driver reads.
    constexpr u32 kNskblWaitPc = 0x5101FE60u;
    if (supply_nskbl_device && pc == kNskblWaitPc && core < static_cast<u32>(kArmCoreCount)) {
        supply_nskbl_device_object(core);
    }

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

    // Diagnostic experiment (round 155).  The loader builds the low page tables
    // itself through the helper at 0x4003AD40 (`str r3,[r5,r4,lsl #2]` is the
    // store), and three of the mappings come out with a *zero* page base:
    //     L2[0x10] = 0x45F   VA 0x10000
    //     L2[0x12] = 0x45F   VA 0x12000
    //     L2[0x14] = 0x45F   VA 0x14000
    // i.e. three different virtual windows land on physical page 0, which is also
    // where the whole boot context (SceKblParam at +0x100, the vector stub, the
    // flags) lives.  Everything the loader puts in those windows therefore
    // overwrites everything else - the class descriptors, the heap object and the
    // region nodes all end up on top of each other, which is exactly the state the
    // "uninitialised descriptor" wall (0x40033FF2, r3 = [r0+56] = 0x65486465,
    // docs/KBL.md rounds 143-150) shows.  Give each of those windows its own
    // physical page and see whether the boot gets past that wall.
    // Measured (round 156): splitting *all three* makes the boot worse, but
    // splitting **only VA 0x12000** (the heap's window, L2[0x12]) takes the loader
    // past the wall: it reaches checkpoint 0x8A instead of 0x87 and its walk
    // count grows from 2.0M to 17.8M.  So the collision that matters is the one
    // between the heap window and whatever else lives on physical page 0 (the
    // class descriptors with their inline names start there), and the loader's
    // memory map (0x400B2B30) really does carry a record with vbase 0x12000 and
    // pbase 0.  What the *correct* page is (the record's own field, or a page the
    // loader should have allocated) is still open; the knob exists to keep the
    // two states one flag apart.
    // ZLB_KBL_LOWALIAS=<list> where list is "all" or a comma separated set of L2
    // indices ("12", "10,14", ...); unset/0 keeps the stock behaviour.
    static const bool lowalias_fix = [] {
        const char* value = std::getenv("ZLB_KBL_LOWALIAS");
        return value == nullptr || value[0] != '0';
    }();
    // The knob takes a comma separated list of L2 indices to split off ("12",
    // "10,14", "all").  Round 201 measured that adding index 3 does *not* back the
    // VA 0x30000 write NSKBL aborts on once the heap exists (the low window comes
    // from the KBL's own page-table store at 0x4003AD6C, whose indices differ), so
    // the default stays the heap window alone.
    static const std::string lowalias_list = [] {
        const char* value = std::getenv("ZLB_KBL_LOWALIAS");
        return std::string(value != nullptr ? value : "12,30");
    }();
    static const bool lowalias_all = lowalias_list.find("all") != std::string::npos;
    constexpr u32 kLowMapStorePc = 0x4003AD6Cu;
    if (lowalias_fix && pc == kLowMapStorePc && core < static_cast<u32>(kArmCoreCount)) {
        static std::array<u32, 0x100> pages{};
        static u32 next = 0x40140000u;   // just above the loader's own page tables
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 index = arm->r[4];
            const bool wanted = lowalias_all || index == 0x10u || index == 0x12u || index == 0x14u;
            if (wanted && arm->r[3] == 0x45Fu && index < pages.size()) {
                const std::string needle = format("%X", index);
                if (lowalias_all || lowalias_list.find(needle) != std::string::npos) {
                    if (pages[index] == 0u) {
                        pages[index] = next | 0x45Fu;
                        next += 0x1000u;
                    }
                    arm->r[3] = pages[index];
                    ZLB_LOG_INFO("machine",
                                 "low window L2[0x%02X] given its own page 0x%08X (diagnostic)", index,
                                 pages[index] & 0xFFFFF000u);
                }
            }
        }
    }

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

    // Substitution (round 94, extended round 163): restore the ARM exception vectors
    // at the mapping the KBL installed.  The KBL points VBAR at VA 0x16100 and maps
    // that VA onto its own DRAM page (`vpa 0x16100` = PA 0x40000100), where its copy
    // is incomplete (two `ldr pc,[pc,#0x18]` words followed by data), so every
    // exception entry falls into zeros and the core ends up hopping through the whole
    // vector page.  The model stages the real 0xC0-byte table from the KBL's ELF
    // segment (vaddr 0) the first time the core enters the vector window, at the
    // physical address the core's own tables resolve.
    //
    // Round 163: staging the ELF table is not enough for the TrustZone monitor.  SKBL
    // *builds* the two tables at runtime, with the MMU off, straight into the physical
    // page PA 0x40000100: the exception stubs at +0x00 and their handlers at +0x20
    // (measured: {0x4002826C, 0x400288C4, ...}) and the monitor stubs at +0x40..+0x5F
    // with the SMC handler pointer (MVBAR+0x28 -> 0x4002831C) at +0x68.  NSKBL, however,
    // clears SCTLR at 0x51000138 and therefore runs with the MMU *off*: its first
    // `smc` (0x510002E4, r12 = 0x103) fetches the monitor vector physically from
    // MVBAR = 0x16140, which in the model is the low boot page - the staged ELF table
    // has zeros there, so the core executed the zero page and derailed.  Mirror the
    // page SKBL built into the page the MMU-off fetch actually reads.
    if (supply_blocks && pc >= 0x16100u && pc < 0x16200u) {
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
                if (!kbl_vectors_restored_ && !kbl_vectors_.empty()) {
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
                // SKBL's runtime page, when it exists and lives somewhere else.
                const u32 runtime_page = 0x40000100u;
                if (pa != runtime_page && arm_bus_->read32(runtime_page) != 0u &&
                    kbl_vector_mirrors_ < 8u) {
                    bool copied = false;
                    for (u32 i = 0; i < 0x100u; ++i) {
                        const u8 byte = arm_bus_->read8(runtime_page + i);
                        if (byte != 0u) copied = true;
                        arm_bus_->write8(pa + i, byte);
                    }
                    if (copied) {
                        ++kbl_vector_mirrors_;
                        ZLB_LOG_INFO("machine",
                                     "SKBL vector page mirrored to PA 0x%08X (MMU-off monitor "
                                     "fetch at MVBAR 0x16140) (development substitution)",
                                     pa);
                        add_milestone("SKBL vector page mirrored to PA 0x" + hex(pa, 8) +
                                      " (development substitution)");
                    }
                }
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
                if (trace_pc_hits_ < 512u) {
                    ++trace_pc_hits_;
                    u64 r[8] = {0}, lr = 0, sp = 0, r12 = 0;
                    for (u32 i = 0; i < 8; ++i) {
                        arm->get_register("r" + std::to_string(i), r[i]);
                    }
                    arm->get_register("r14", lr);
                    arm->get_register("r13", sp);
                    // Round 218: r12 as well - a string copy takes its destination in
                    // `ip`, which is what tells apart two VAs that the model maps to
                    // the same physical page.
                    arm->get_register("r12", r12);
                    ZLB_LOG_INFO("machine",
                                 "trace pc=0x%08X arm%u from pc=0x%08X insns=%llu lr=0x%08X sp=0x%08X "
                                 "r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X r4=0x%08X r5=0x%08X "
                                 "r6=0x%08X r7=0x%08X r12=0x%08X",
                                 pc, core, previous_pc,
                                 static_cast<unsigned long long>(total_instructions()),
                                 static_cast<u32>(lr), static_cast<u32>(sp),
                                 static_cast<u32>(r[0]), static_cast<u32>(r[1]),
                                 static_cast<u32>(r[2]), static_cast<u32>(r[3]),
                                 static_cast<u32>(r[4]), static_cast<u32>(r[5]),
                                 static_cast<u32>(r[6]), static_cast<u32>(r[7]),
                                 static_cast<u32>(r12));
                    // Round 220: translate the four argument registers too.  The
                    // question this answers is which *physical* page a virtual address
                    // names at this instant - the partition's node array lives at the
                    // constant VA 0x00100000, and the run both writes and reads it, so
                    // comparing the two translations says whether the guest's own
                    // tables changed under the model's feet.
                    for (u32 i = 0; i < 4; ++i) {
                        const u32 va = static_cast<u32>(r[i]);
                        if (va < 0x1000u) continue;
                        u32 pa = 0;
                        std::string fault;
                        if (arm->translate(va, false, false, pa, fault)) {
                            if (arm->mmu.last_walk.used_l2) {
                                ZLB_LOG_INFO("machine",
                                             "trace   r%u=0x%08X -> PA 0x%08X (ttbr%u L1[0x%03X]@0x%08X="
                                             "0x%08X L2[0x%02X]@0x%08X=0x%08X)",
                                             i, va, pa, arm->mmu.last_walk.ttbr_num,
                                             (va >> 20) & 0xFFFu, arm->mmu.last_walk.l1_addr,
                                             arm->mmu.last_walk.l1_desc, (va >> 12) & 0xFFu,
                                             arm->mmu.last_walk.l2_addr, arm->mmu.last_walk.l2_desc);
                            } else {
                                ZLB_LOG_INFO("machine",
                                             "trace   r%u=0x%08X -> PA 0x%08X (ttbr%u L1[0x%03X]@0x%08X="
                                             "0x%08X, section)",
                                             i, va, pa, arm->mmu.last_walk.ttbr_num,
                                             (va >> 20) & 0xFFFu, arm->mmu.last_walk.l1_addr,
                                             arm->mmu.last_walk.l1_desc);
                            }
                        } else {
                            ZLB_LOG_INFO("machine", "trace   r%u=0x%08X -> %s", i, va, fault.c_str());
                        }
                    }
                    // Round 222: the four stack words, translated.  Helpers like the
                    // L2 page filler 0x510152A0 take their physical base as a *stack*
                    // argument, which is exactly what decides where the partition's
                    // node array ends up (docs/NSKBL.md 8.56).
                    {
                        u32 sp_pa = 0;
                        std::string fault;
                        if (arm->translate(static_cast<u32>(sp), false, false, sp_pa, fault)) {
                            ZLB_LOG_INFO("machine",
                                         "trace   sp[0]=0x%08X sp[4]=0x%08X sp[8]=0x%08X "
                                         "sp[12]=0x%08X",
                                         arm_bus_->read32(sp_pa), arm_bus_->read32(sp_pa + 4u),
                                         arm_bus_->read32(sp_pa + 8u), arm_bus_->read32(sp_pa + 12u));
                        }
                    }
                }
            }
        }
    }

    // Diagnostic (round 212): full instruction trace of a VA range.  The
    // round-209 measurements could only name branch targets because the other
    // diagnostics sample one pc; the NSKBL memory-manager failure
    // (0x5100C04C returning 0x80020005 for a 64 KiB request) needs the whole
    // path through the function.  ZLB_ARM_TRACE_RANGE=<lo>-<hi> logs every
    // instruction executed inside the range, with the register file and the
    // return-address words on the stack, until ZLB_ARM_TRACE_LIMIT (default
    // 4000) lines are printed or the core leaves the range.  One core is
    // traced at a time (the first one to enter).
    static const bool trace_range_on = [] {
        const char* value = std::getenv("ZLB_ARM_TRACE_RANGE");
        return value != nullptr && value[0] != '\0';
    }();
    static const u32 trace_range_lo = [] {
        const char* value = std::getenv("ZLB_ARM_TRACE_RANGE");
        if (value == nullptr) return 0u;
        return static_cast<u32>(std::strtoul(value, nullptr, 16));
    }();
    static const u32 trace_range_hi = [] {
        const char* value = std::getenv("ZLB_ARM_TRACE_RANGE");
        if (value == nullptr) return 0u;
        const char* dash = std::strchr(value, '-');
        if (dash == nullptr) return 0u;
        return static_cast<u32>(std::strtoul(dash + 1, nullptr, 16));
    }();
    static const u32 trace_range_limit = [] {
        const char* value = std::getenv("ZLB_ARM_TRACE_LIMIT");
        if (value == nullptr) return 4000u;
        return static_cast<u32>(std::strtoul(value, nullptr, 10));
    }();
    static int trace_range_core = -1;
    static u32 trace_range_lines = 0;
    static bool trace_range_inside = false;
    static bool trace_range_armed = false;
    // Optional trigger pc: tracing starts at that instruction and then covers the
    // whole range, which is how the *first* call of a shared helper (0x5100C04C)
    // from a specific caller (0x51005700) can be isolated.
    static const u32 trace_range_trigger = [] {
        const char* value = std::getenv("ZLB_ARM_TRACE_TRIGGER");
        if (value == nullptr) return 0xFFFFFFFFu;
        return static_cast<u32>(std::strtoul(value, nullptr, 16));
    }();
    if (trace_range_on && trace_range_trigger != 0xFFFFFFFFu && !trace_range_armed &&
        pc == trace_range_trigger) {
        trace_range_armed = true;
    }
    if (trace_range_on && (trace_range_trigger == 0xFFFFFFFFu || trace_range_armed) &&
        trace_range_lines < trace_range_limit &&
        core < static_cast<u32>(kArmCoreCount)) {
        const bool inside = pc >= trace_range_lo && pc < trace_range_hi;
        if (inside && (trace_range_core < 0 || trace_range_core == static_cast<int>(core))) {
            if (!trace_range_inside) {
                trace_range_core = static_cast<int>(core);
                ZLB_LOG_INFO("machine", "trace-range arm%u ENTER 0x%08X from 0x%08X", core, pc,
                             previous_pc);
                trace_range_inside = true;
            }
            if (ArmCore* arm = dynamic_cast<ArmCore*>(this->arm_cores_[core].get())) {
                unsigned length = 0;
                std::string text = arm->disassemble(pc, length);
                u64 r[8] = {0}, lr = 0, sp = 0;
                for (u32 i = 0; i < 8; ++i) {
                    arm->get_register("r" + std::to_string(i), r[i]);
                }
                arm->get_register("r14", lr);
                arm->get_register("r13", sp);
                ++trace_range_lines;
                ZLB_LOG_INFO("machine",
                             "  [%u] %08X  %-28s r0=%08X r1=%08X r2=%08X r3=%08X "
                             "r4=%08X r5=%08X r6=%08X r7=%08X sp=%08X lr=%08X",
                             trace_range_lines, pc, text.c_str(), static_cast<u32>(r[0]),
                             static_cast<u32>(r[1]), static_cast<u32>(r[2]),
                             static_cast<u32>(r[3]), static_cast<u32>(r[4]),
                             static_cast<u32>(r[5]), static_cast<u32>(r[6]),
                             static_cast<u32>(r[7]), static_cast<u32>(sp),
                             static_cast<u32>(lr));
            }
        } else if (trace_range_inside && !inside) {
            ZLB_LOG_INFO("machine", "trace-range arm%u LEAVE to 0x%08X", core, pc);
            trace_range_inside = false;
        }
    }

    // Diagnostic (round 214): ZLB_ARM_TRACE_RING=<pc> keeps a ring of the last
    // instructions each core executed and dumps the whole ring when the listed pc
    // is reached.  The other diagnostics sample one pc per slice (the debugger's
    // `history`) or need a range to be entered first, so the instruction *path*
    // into an abort handler - which is what "the object pointer is a string"
    // needs - was not visible at all.  ZLB_ARM_TRACE_RING=0x51000C74 dumps 4096
    // instructions of every core that reaches it.
    static const u32 trace_ring_pc = [] {
        const char* value = std::getenv("ZLB_ARM_TRACE_RING");
        if (value == nullptr) return 0u;
        return static_cast<u32>(std::strtoul(value, nullptr, 16));
    }();
    if (trace_ring_pc != 0u && core < static_cast<u32>(kArmCoreCount)) {
        // ZLB_ARM_TRACE_RING_SIZE narrows the ring: the default 16 K dump floods the
        // log, and naming a divergence usually needs the last few hundred
        // instructions rather than the whole history.
        static const u32 ring_size = [] {
            const char* value = std::getenv("ZLB_ARM_TRACE_RING_SIZE");
            if (value == nullptr) return 16384u;
            const u32 parsed = static_cast<u32>(std::strtoul(value, nullptr, 10));
            return parsed >= 64u ? parsed : 16384u;
        }();
        // ZLB_ARM_TRACE_RING_DUMPS=<n> allows comparing the 1st, 2nd, ... visit of the
        // same pc; the cap also keeps a trap loop from flooding the log.
        static const u32 ring_dumps = [] {
            const char* value = std::getenv("ZLB_ARM_TRACE_RING_DUMPS");
            if (value == nullptr) return 1u;
            const u32 parsed = static_cast<u32>(std::strtoul(value, nullptr, 10));
            return parsed >= 1u ? parsed : 1u;
        }();
        trace_ring_max_dumps = ring_dumps;
        const u32 kRingSize = ring_size;
        static std::array<std::array<u32, 16384u>, kArmCoreCount> ring{};
        static std::array<u32, kArmCoreCount> ring_pos{};
        auto& slot = ring[core];
        auto& pos = ring_pos[core];
        if (pc == trace_ring_pc && trace_ring_dumps_[core] < trace_ring_max_dumps) {
            ++trace_ring_dumps_[core];
            ZLB_LOG_INFO("machine", "trace-ring arm%u reached 0x%08X; last %u instructions:",
                         core, pc, kRingSize < pos ? kRingSize : pos);
            const u32 count = kRingSize < pos ? kRingSize : pos;
            std::string line;
            for (u32 i = 0; i < count; ++i) {
                const u32 entry = slot[(pos - count + i) % kRingSize];
                line += format(" %08X", entry);
                if ((i % 8u) == 7u) {
                    ZLB_LOG_INFO("machine", "  %s", line.c_str());
                    line.clear();
                }
            }
            if (!line.empty()) ZLB_LOG_INFO("machine", "  %s", line.c_str());
            // Restart the ring so the next dump covers the *next* visit: several
            // calls of the same function have to be compared one by one, and a
            // running ring would carry the previous visit's tail into the dump.
            pos = 0;
            slot.fill(0u);
        } else if (trace_ring_dumps_[core] < trace_ring_max_dumps) {
            slot[pos % kRingSize] = pc;
            ++pos;
        }
    }

    // Round 167 note (not enabled): handing the empty map slot 0xD8 a fresh arena page
    // looks like the pool fix, but it *misroutes* the flow: the device open then proceeds
    // with a zero-filled container, and the coverage map (ZLB_ARM_COV=1 + the debugger's
    // `cov`) shows the descriptor init at 0x5100E850-0x5100EEA0 is then never executed,
    // while without the substitution it runs and the run instead dies on the class-magic
    // check with obj = NULL (docs/NSKBL.md 8.9).  The real gap is that NSKBL's container
    // builders (the never-executed pages 0x51007000-0x51009000) do not run at all, so the
    // faithful state is the unsubstituted one.

    // Substitution (round 169): the flow leaves the normal list/allocator path for the
    // device-open block at 0x5100B58A because the pool object the model supplies is
    // empty.  Measured (ZLB_KBL_TRACE_PC=0x5100B638): the block is entered from
    // 0x5100B58A, which is `bls 0x5100B638` after
    //
    //   5100B57C  ldrh r1,[r4,#0x30]      ; free slots
    //   5100B57E  ldrh r3,[r4,#0x32]      ; used slots
    //   5100B586  cmp  r1,r3
    //   5100B58A  bls  0x5100B638         ; "no room" -> device-open -> fatal()
    //
    // with r4 = 0x01100000, the arena page handed out by ZLB_NSKBL_POOL, whose counters
    // are both zero (0 <= 0).  NSKBL's own pool builder (the never-executed pages
    // 0x51007000-0x51009000) would have filled them.  Give the empty pool one free slot
    // so the normal path runs; ZLB_NSKBL_POOLFIX=0 and ZLB_NO_SUBSTITUTION=1 disable it.
    static const bool pool_fix = [] {
        const char* value = std::getenv("ZLB_NSKBL_POOLFIX");
        if (value != nullptr && value[0] == '0') return false;
        return substitutions_enabled_static();
    }();
    static const bool nskbl_class = [] {
        const char* value = std::getenv("ZLB_NSKBL_CLASS");
        if (value != nullptr && value[0] == '0') return false;
        return substitutions_enabled_static();
    }();
    if (pool_fix && pc == 0x5100B57Cu && core < static_cast<u32>(kArmCoreCount)) {
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 obj = static_cast<u32>(arm->r[4]);
            std::string fault;
            u32 pa = 0;
            if (obj >= kInstanceArenaVa && obj < kInstanceArenaVa + kInstanceArenaSize &&
                arm->translate(obj + 0x30u, true, false, pa, fault) &&
                arm_bus_->read16(pa) == 0u && arm_bus_->read16(pa + 2u) == 0u) {
                arm_bus_->write16(pa, 4u);                      // free slots
                arm_bus_->write16(pa + 2u, 0u);                 // used slots
                arm_bus_->write32(pa + 4u, obj + 0x100u);       // pointer array
                arm_bus_->write32(pa + 0x1Cu, obj + 0x200u);    // second pointer array
                // Substitution (round 170): the object also needs a class pointer at +4.
                // Without it the virtual dispatch `ldr r2,[r4,#4]; ldr r3,[r2,#0x38];
                // blx r3` at 0x5100B59E goes through VA 0x38 (the fault hook answers with
                // the low page) to 0xFFFFFFFE and panics with 0xAC.  Build a minimal class
                // page in the arena whose method slots are all `bx lr` (0x51014B94, ARM),
                // so any virtual call through it returns harmlessly.
                if (nskbl_class) {
                    const u32 class_va = kInstanceArenaVa + kInstanceArenaSize - 0x1000u;
                    std::string cfault;
                    u32 class_pa = 0;
                    if (arm->translate(class_va, true, false, class_pa, cfault)) {
                        for (u32 off = 0; off < 0x40u; off += 4u) {
                            arm_bus_->write32(class_pa + off, 0x51014B94u);
                        }
                        arm_bus_->write32(pa - 0x30u + 4u, class_va);
                    }
                }
                ++boot_pc_fixes_;
                if (nskbl_pool_fixes_ < 8u) {
                    ++nskbl_pool_fixes_;
                    ZLB_LOG_INFO("machine",
                                 "NSKBL pool 0x%08X given one free slot before the 0x5100B58A "
                                 "check (development substitution)", obj);
                    add_milestone("NSKBL pool seeded (development substitution)");
                }
            }
        }
    }

    // Substitution (round 166): NSKBL's object constructor 0x5100B41C is handed the
    // pool/container for the requested size class, which the getter 0x5100B6E4 reads
    // from the map object (`[[0x5113B5AC] + 0x5C/0x64/0x6C]` for types 20/40/80).  The
    // model's map object has those fields zero: the code that builds the pools (the
    // list builders around 0x51009516) is never executed - a 0x51009518-0x51009548 PC
    // trap sees no hit in the whole run - so the constructor gets r0 = 0, dereferences
    // NULL, spills into NSKBL's low page and the following `ldr r8,[r4,#12]` reads the
    // garbage 0x4B656350 whose dereference raises a section translation fault and
    // panics with 0xAD (docs/NSKBL.md 8.7).  On hardware that pool exists, so hand the
    // constructor a page from the model's instance arena instead.  ZLB_NSKBL_POOL=0
    // and ZLB_NO_SUBSTITUTION=1 disable it.
    static const bool nskbl_pool = [] {
        const char* value = std::getenv("ZLB_NSKBL_POOL");
        if (value != nullptr && value[0] == '0') return false;
        return substitutions_enabled_static();
    }();
    if (nskbl_pool && pc == 0x5100B41Cu) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                if (static_cast<u32>(arm->r[0]) == 0u && nskbl_pool_next_ + 0x1000u <= kInstanceArenaSize) {
                    ensure_arena_section(arm, arm_bus_.get(), kInstanceArenaVa);
                    const u32 block = kInstanceArenaVa + nskbl_pool_next_;
                    nskbl_pool_next_ += 0x1000u;
                    arm->set_register("r0", block);
                    ++boot_pc_fixes_;
                    if (nskbl_pool_supplies_ < 8u) {
                        ++nskbl_pool_supplies_;
                        ZLB_LOG_INFO("machine",
                                     "NSKBL object pool supplied at 0x%08X (arm%u, r0 was 0; "
                                     "development substitution)",
                                     block, core);
                        add_milestone("NSKBL object pool supplied (development substitution)");
                    }
                }
            }
        }
    }

    // Substitution (round 180): NSKBL's general allocator 0x5100D400 asks the heap
    // resolver 0x5100567C for a block, which reads the heap object from the map
    // (`[[0x5113B5AC] + 0x8C]`) and searches a free block in it (0x510049F4 → 0x5100B82C).
    // In the model that field is zero - and a write trap over the whole run shows it is
    // only ever written with zero (the page wipe at 0x5101354C and the map init at
    // 0x510064D6), so nothing creates the heap.  The failure then aborts NSKBL's class
    // installer 0x51007E70 on its first allocations (12/32/64/44 bytes), which is what
    // leaves one core away from the barrier (counter stuck at 3 of 4) and livelocks the
    // machine (docs/NSKBL.md 8.21-8.23).  On hardware the heap exists, so hand out a
    // zeroed block from the model's heap arena instead of calling the resolver.
    // ZLB_NSKBL_HEAP=1 enables it (opt-in: round 180 measured that handing out blocks
    // changes the installer's control flow without unblocking the boot - the failure is
    // handled further down, so the real fix is the missing heap-creation step, not a
    // stand-in block).  ZLB_NO_SUBSTITUTION=1 still disables everything.
    static const bool nskbl_heap = [] {
        const char* value = std::getenv("ZLB_NSKBL_HEAP");
        if (value == nullptr || value[0] == '0') return false;
        return substitutions_enabled_static();
    }();
    // Round 194: the heap has to exist before the first *use* of it.  The block
    // search is entered from the resolver 0x51004AD0 (measured: trace of
    // 0x5100B82C shows r0 = 0 coming from 0x51004A0A), which happens before the
    // allocator entry 0x5100D418, so install the object at the resolver too.
    if (nskbl_heap && (pc == 0x51004AD0u || pc == 0x51004C7Eu || pc == 0x5100D418u)) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                const u32 map_global_va = 0x5113B5ACu;
                const u32 map_va = arm_bus_->read32(map_global_va);
                // NSKBL's low window maps VA 0x0-0x3FFFF to PA 0x40300000+.
                const u32 map_pa = (map_va < 0x40000u) ? (map_va + 0x40300000u) : map_va;
                const u32 heap = arm_bus_->read32(map_pa + 0x8Cu);
                const u32 size = static_cast<u32>(arm->r[1]);
                // Only stand in for the class installer 0x51007E70 (its allocations are
                // the ones whose failure aborts the class setup); other callers must keep
                // seeing the real (empty) heap so their control flow is unchanged.  The
                // installer is recognised from the caller chain on the stack.
                bool from_installer = false;
                const u32 sp = static_cast<u32>(arm->r[13]);
                for (u32 i = 0; i < 24u && !from_installer; ++i) {
                    const u32 word = arm_bus_->read32(sp + i * 4u);
                    from_installer = word >= 0x51007E70u && word < 0x51008400u;
                }
                if (heap == 0u &&
                    (pc == 0x51004AD0u || pc == 0x51004C7Eu || from_installer)) {
                    ensure_arena_section(arm, arm_bus_.get(), kHeapArenaVa);
                    // Round 193: build a heap that the *firmware's own* search accepts,
                    // so no call has to be skipped.  With granule = 1 (heap+0x20) and
                    // base = 1 (heap+0x1C) the helpers collapse:
                    //   r8    = round_up(base, granule)   = 1
                    //   index = round_up(size, r8)        = size
                    //   helper 0x51025AD4(size, 1)        = size - 1*size = 0
                    //   block = granule * 0 + table[index] = table[index]
                    // so the block address *is* the class-table entry, and pre-filling
                    // the table with distinct arena pages hands out real memory through
                    // the allocator's normal path (docs/NSKBL.md 8.29).
                    const u32 heap_obj = kHeapArenaVa + 0x1000u;
                    const u32 class_table = kHeapArenaVa + 0x2000u;
                    const u32 classes = 4096u;
                    const u32 block_stride = 0x1000u;
                    const u32 block_count = 240u;
                    for (u32 i = 0; i < 0x100u; ++i) {
                        arm_bus_->write32(heap_obj + i * 4u, 0u);
                    }
                    for (u32 i = 0; i < classes; ++i) {
                        const u32 block =
                            kHeapArenaVa + 0x10000u + (i % block_count) * block_stride;
                        arm_bus_->write32(class_table + i * 4u, block);
                    }
                    arm_bus_->write32(heap_obj + 0x1Cu, 1u);
                    arm_bus_->write16(heap_obj + 0x20u, 1u);
                    arm_bus_->write16(heap_obj + 0x32u, static_cast<u16>(classes));
                    arm_bus_->write32(heap_obj + 0x34u, class_table);
                    arm_bus_->write32(map_pa + 0x8Cu, heap_obj);
                    ZLB_LOG_INFO("machine",
                                 "NSKBL heap object 0x%08X installed at map+0x8C "
                                 "(granule 1, base 1, %u classes, %u arena blocks; "
                                 "development substitution)",
                                 heap_obj, classes, block_count);
                    add_milestone("NSKBL heap object installed (development substitution)");
                    ++boot_pc_fixes_;
                    if (nskbl_heap_supplies_ < 8u) ++nskbl_heap_supplies_;
                }
                if (false) {
                    ensure_arena_section(arm, arm_bus_.get(), kHeapArenaVa);
                    const u32 block = kHeapArenaVa + nskbl_heap_next_;
                    const u32 step = (size + 15u) & ~15u;
                    for (u32 offset = 0; offset < step; offset += 4u) {
                        arm_bus_->write32(block + offset, 0u);
                    }
                    nskbl_heap_next_ += step;
                    // Also give the map a real heap *object* (layout measured in
                    // docs/NSKBL.md 8.29: +0x1C base, +0x20 granule, +0x32 class count,
                    // +0x34 class table), because later code inspects the heap - the
                    // sanity check at 0x5100FE0E compares a block field against a value
                    // that comes from the heap, and with map->[0x8C] still zero that
                    // check panics (round 180/184 measurement).
                    if (nskbl_heap_supplies_ == 0u) {
                        const u32 heap_obj = kHeapArenaVa + 0x1000u;
                        const u32 class_table = kHeapArenaVa + 0x2000u;
                        const u32 classes = 256u;
                        for (u32 i = 0; i < 0x100u; ++i) {
                            arm_bus_->write32(heap_obj + i * 4u, 0u);
                        }
                        for (u32 i = 0; i < classes; ++i) {
                            arm_bus_->write32(class_table + i * 4u, 0xFFFFFFFFu);
                        }
                        arm_bus_->write32(heap_obj + 0x1Cu, kHeapArenaVa + 0x4000u);
                        arm_bus_->write16(heap_obj + 0x20u, 16u);
                        arm_bus_->write16(heap_obj + 0x32u, static_cast<u16>(classes));
                        arm_bus_->write32(heap_obj + 0x34u, class_table);
                        arm_bus_->write32(map_pa + 0x8Cu, heap_obj);
                        nskbl_heap_next_ = 0x4000u;
                        ZLB_LOG_INFO("machine",
                                     "NSKBL heap object 0x%08X installed at map+0x8C "
                                     "(base 0x%08X, granule 16, %u classes; development "
                                     "substitution)",
                                     heap_obj, kHeapArenaVa + 0x4000u, classes);
                        add_milestone("NSKBL heap object installed (development substitution)");
                    }
                    arm->set_register("r0", block);
                    arm->set_pc(0x5100D41Cu);              // return from 0x5100567C
                    arm->set_register("THUMB", 1u);
                    ++boot_pc_fixes_;
                    if (nskbl_heap_supplies_ < 16u) {
                        ++nskbl_heap_supplies_;
                        ZLB_LOG_INFO("machine",
                                     "NSKBL heap block 0x%08X (%u bytes) from the model "
                                     "arena (arm%u, map->[0x8C] was 0; development "
                                     "substitution)",
                                     block, size, core);
                        add_milestone("NSKBL heap block supplied (development substitution)");
                    }
                }
            }
        }
    }

    // Substitution (round 184): NSKBL's boot-mode gate 0x51010F14 reads the byte at
    // boot_config[0x33] and treats 0xFF as "the boot configuration is valid"; anything
    // else sends it down the path that prints ".Safe Mode : [ YES ]" (docs/NSKBL.md
    // 8.26-8.27).  Nothing in the model ever fills that byte - the whole structure at
    // VA 0x47C0 is zeroed by NSKBL's own initialiser and no CPU store or DMA to it
    // exists - so the machine sits in the first-boot/safe path forever.  A console that
    // booted before has this marker persisted, so stamping it stands in for "the boot
    // configuration is valid" and is on by default (ZLB_NSKBL_BOOTCFG=0 disables it).
    // Measured effect: the mode gate 0x51010F14 returns 0 instead of 1, the safe-mode
    // print (0x5100100E/0x51001022) no longer runs, and ~600 more bytes of NSKBL (the
    // class installer 0x51007E88-0x51007EF8 and the stage tail 0x51001042-0x51001054)
    // execute.
    static const bool bootcfg_marker = [] {
        const char* value = std::getenv("ZLB_NSKBL_BOOTCFG");
        if (value != nullptr && value[0] == '0') return false;
        return substitutions_enabled_static();
    }();
    if (bootcfg_marker && pc == 0x51010F20u) {
        // boot-config VA 0x47C0 lives in NSKBL's low window: PA = VA + 0x40300000.
        static bool stamped = false;
        const u32 bootcfg_pa = 0x403047C0u;
        if (!stamped) {
            arm_bus_->write8(bootcfg_pa + 0x33u, 0xFFu);
            // Round 261: the first external load is gated on bit 0 of the boot config's
            // +0x6C field.  The dispatcher 0x51018F6C calls 0x51010F00 for the object call
            // 0x10005 ("open path"), and that helper is literally
            //     r3 = [obj+0x3C] (boot config) ; r0 = [r3+0x6C] ; r0 &= 1 ; bx lr
            // A zero result sends the dispatcher to its failure exit 0x51019116, which is
            // why NSKBL's open of os0:psp2bootconfig.skprx returns 0x803FF007.  Bit 0 means
            // "storage loading enabled" for the boot configuration the model supplies.
            const u32 gate_pa = bootcfg_pa + 0x6Cu;
            // Bit 0 gates the dispatcher's first check (0x51010F00) and the firmware sets
            // it itself (pc 0x5101587C writes 1 there), so supplying it matches the
            // guest's own intent.  Bit 2 gates the second check (0x51010EEC,
            // `ubfx r0, r0, #2, #1`), but setting it did not change the run and the guest
            // never writes it - so the model does not fabricate it.
            arm_bus_->write32(gate_pa, arm_bus_->read32(gate_pa) | 1u);
            stamped = true;
            ZLB_LOG_INFO("machine",
                         "boot-config marker written (PA 0x%08X = 0xFF; PA 0x%08X |= 1; development "
                         "substitution)",
                         bootcfg_pa + 0x33u, gate_pa);
            add_milestone("NSKBL boot-config marker stamped (development substitution)");
        }
    }

    // Substitution (round 236): remember the section NSKBL replaces with a page table, so
    // that the window's content can be measured with and without it (docs/NSKBL.md 8.69).
    // The guest builds its per-frame state table through the 2 MiB section
    // `VA 0x00100000 -> PA 0x40400000` (installed at pc 0x51014F28) and then switches the
    // same VA to a page table (pc 0x51014D9E, `insns = 52 891 619`) whose table maps a
    // single 4 KiB page, after which every allocation verification reads zeros and NSKBL
    // ends in its own fatal sink.  Opt-in: ZLB_NSKBL_SECTION_KEEP=1.
    // Substitution (round 236-238): keep the window's content across the mapping switch.
    // The guest builds its per-frame state table through the 2 MiB section
    // `VA 0x00100000 -> PA 0x40400000` (installed at pc 0x51014F28) and then replaces it
    // with a page table (pc 0x51014D9E, `insns = 52 891 619`) whose table maps a single
    // 4 KiB page, so the records written through the section become unreachable and every
    // allocation verification reads zeros (`0x80024300`), leaving NSKBL's object registry
    // NULL and ending the run in its own fatal sink (0x51015B7C).  Measured effect of
    // carrying the section's page over at the first record read: stage 0xA7 without the
    // fatal sink, the run stops waiting in the lock area (pc 0x510147DC) after 99.5M
    // instructions instead of running to 153.2M and calling 0x51015B7C.  Default on;
    // ZLB_NSKBL_SECTION_KEEP=0 disables (ZLB_NO_SUBSTITUTION=1 wins).
    static const bool keep_section = [] {
        const char* value = std::getenv("ZLB_NSKBL_SECTION_KEEP");
        if (value != nullptr && value[0] == '0') return false;
        return substitutions_enabled_static();
    }();
    if (keep_section && pc == 0x51014D9Eu) {
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 va = arm->r[7];
            const u32 l1_addr = (arm->mmu.ttbr0 & 0xFFFFC000u) | (((va >> 20) & 0xFFFu) << 2);
            const u32 old = arm_bus_->read32(l1_addr);
            if ((old & 3u) == 2u) {
                arm->mmu.replaced_section = old;
                arm->mmu.replaced_section_va = va & 0xFFF00000u;
                ZLB_LOG_INFO("machine",
                             "window section kept: VA 0x%08X had section 0x%08X -> PA 0x%08X "
                             "(development substitution)",
                             va, old, old & 0xFFF00000u);
                add_milestone("NSKBL window section remembered (development substitution)");
            }
        }
    }
    // The guest attaches the page table first (above) and only then maps pages into it.
    // The unique window caller is 0x51005C92 (`bl 0x51009A54`, measured: target VA in r2
    // = 0x00100000, the physical page on the stack), so hook the instruction after it and
    // carry the section's page over into whatever page the window now resolves to.  That
    // is the measurement that decides whether the per-frame records written through the
    // section before the switch are what the allocation verification needs.
    if (keep_section && pc == 0x51005C96u) {
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 section = arm->mmu.replaced_section;
            const u32 va = arm->mmu.replaced_section_va;
            if ((section & 3u) == 2u && va != 0u) {
                const u32 l1_addr = (arm->mmu.ttbr0 & 0xFFFFC000u) | (((va >> 20) & 0xFFFu) << 2);
                const u32 l1 = arm_bus_->read32(l1_addr);
                u32 pa = 0;
                if ((l1 & 3u) == 1u) {
                    const u32 l2 = arm_bus_->read32((l1 & 0xFFFFFC00u) | (((va >> 12) & 0xFFu) << 2));
                    // A small page is bits[1:0] = 0b1x (bit 1 = 1, bit 0 = XN), so the
                    // check is "bit 1 set", not "== 2": NSKBL's own entry here is
                    // 0x4030245F (XN set) and the stricter test silently made the whole
                    // migration below dead code (round 238 measurement).
                    if ((l2 & 2u) != 0u) pa = l2 & 0xFFFFF000u;
                }
                if (pa != 0u && pa != (section & 0xFFF00000u)) {
                    const u32 source = section & 0xFFF00000u;
                    for (u32 b = 0; b < 0x1000u; ++b) {
                        arm_bus_->write8(pa + b, arm_bus_->read8(source + b));
                    }
                    ZLB_LOG_INFO("machine",
                                 "window page migrated: VA 0x%08X -> PA 0x%08X (from PA 0x%08X; "
                                 "development substitution)",
                                 va, pa, source);
                    add_milestone("NSKBL window page migrated (development substitution)");
                }
            }
        }
    }
    // Round 238: the mapping above happens *before* the switch (measured 52 889 526 <
    // 52 891 619), so at the switch the attached table is still empty and nothing can be
    // migrated there.  Where the loss actually shows is the class path's own record read
    // (`pc 0x5100C1F8`, address `[partition+0x20] + index*4` = VA 0x0010000C), which after
    // the switch resolves into the freshly mapped page and reads zeros.  Carry the
    // section's page over there - once per page - and measure whether the allocation
    // verification passes.
    if (keep_section && pc == 0x5100C1F8u) {
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 section = arm->mmu.replaced_section;
            const u32 base = arm->mmu.replaced_section_va;
            const u32 slot_va = arm->r[2];
            const u32 page_va = slot_va & 0xFFFFF000u;
            if ((section & 3u) == 2u && base != 0u && page_va >= base &&
                page_va < base + 0x00100000u) {
                const u32 l1_addr = (arm->mmu.ttbr0 & 0xFFFFC000u) | (((page_va >> 20) & 0xFFFu) << 2);
                const u32 l1 = arm_bus_->read32(l1_addr);
                u32 pa = 0;
                if ((l1 & 3u) == 1u) {
                    const u32 l2 = arm_bus_->read32((l1 & 0xFFFFFC00u) | (((page_va >> 12) & 0xFFu) << 2));
                    // A small page is bits[1:0] = 0b1x (bit 1 = 1, bit 0 = XN), so the
                    // check is "bit 1 set", not "== 2": NSKBL's own entry here is
                    // 0x4030245F (XN set) and the stricter test silently made the whole
                    // migration below dead code (round 238 measurement).
                    if ((l2 & 2u) != 0u) pa = l2 & 0xFFFFF000u;
                }
                const u32 source = (section & 0xFFF00000u) | (page_va - base);
                static bool migrated_once = false;
                if (!migrated_once) {
                    migrated_once = true;
                    ZLB_LOG_INFO("machine",
                                 "window record read: slot=0x%08X page=0x%08X section=0x%08X "
                                 "base=0x%08X pa=0x%08X source=0x%08X",
                                 slot_va, page_va, section, base, pa, source);
                }
                if (pa != 0u && pa != source) {
                    static bool copied_once = false;
                    if (!copied_once) {
                        copied_once = true;
                        for (u32 b = 0; b < 0x1000u; ++b) {
                            arm_bus_->write8(pa + b, arm_bus_->read8(source + b));
                        }
                        ZLB_LOG_INFO("machine",
                                     "window page restored at the record read: VA 0x%08X -> "
                                     "PA 0x%08X (from PA 0x%08X; development substitution)",
                                     page_va, pa, source);
                        add_milestone("NSKBL window page restored (development substitution)");
                    }
                }
            }
        }
    }

    // Diagnostic (round 270): NSKBL's first external load fails inside 0x510232EC, which
    // returns zero from its shared tail (0x510238F4) without ever walking a directory.
    // Static reading of that function's branches did not match the run (pc 0x5102413A is
    // never executed), so take the predecessor from the machine's own last-pc record
    // instead of guessing: point ZLB_NSKBL_OPEN_EXIT at the pc of interest (see
    // docs/NSKBL.md section 8) and it prints the pc that led there.  Off by default.
    static const bool log_open_exit = [] {
        const char* value = std::getenv("ZLB_NSKBL_OPEN_EXIT");
        return value != nullptr && value[0] != '0';
    }();
    static const u32 open_exit_pc = [] {
        const char* value = std::getenv("ZLB_NSKBL_OPEN_EXIT_PC");
        return value != nullptr ? static_cast<u32>(std::strtoul(value, nullptr, 16)) : 0x510238F4u;
    }();
    if (log_open_exit && pc == open_exit_pc) {
        static unsigned reported = 0;
        if (reported++ < 8u && core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                ZLB_LOG_INFO("machine", "nskbl pc 0x%08X: r0=0x%08X r1=0x%08X from pc=0x%08X", pc,
                             static_cast<u32>(arm->r[0]), static_cast<u32>(arm->r[1]),
                             previous_pc);
            }
        }
    }

    // Substitution (round 197): NSKBL's kernel physical-memory partition
    // (`ScePhyMemPartKD`, VA 0x5300) keeps a per-core range table at `[+0x9C]`
    // (32-byte records: u16 type, u16 count, then seven u32 range values).  In the
    // model that table is empty - only the page wipe ever writes it - so the range
    // request 0x5100C04C fails, 0x51005700 returns early and SceKernelSysrootClass
    // is never registered, which is why no heap exists and the class installer
    // aborts (docs/NSKBL.md 8.33).  On hardware the boot configuration provides the
    // physical memory ranges; here the model hands out free DRAM pages, refilling
    // the record whenever the firmware has consumed all of them.
    // ZLB_NSKBL_PHYSPOOL=0 and ZLB_NO_SUBSTITUTION=1 disable it.
    // Round 215/216: still opt-in.  Supplying the table makes the run execute ~38M
    // instructions *more* (114.8M -> 153.2M) and it no longer panics: after the MMU
    // XN fix (round 216) NSKBL's abort handler works, so instead of stopping at its
    // own checkpoint 0xAD the non-secure core keeps running through the low window
    // (`pc` wanders between 0x5B60C and 0x64CFC over 150k-250k slices) - a state that
    // is harder to attribute than the clean stop below.  The default therefore keeps
    // NSKBL's diagnosable panic (0xAD at 0x5100EEB8, docs/NSKBL.md 8.50) and the
    // substitution stays available as ZLB_NSKBL_PHYSPOOL=1.
    static const bool physpool = [] {
        const char* value = std::getenv("ZLB_NSKBL_PHYSPOOL");
        if (value == nullptr || value[0] == '0') return false;
        return substitutions_enabled_static();
    }();
    if (physpool && pc == 0x5100C04Cu && core < static_cast<u32>(kArmCoreCount)) {
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            // VA 0x01500000 <-> PA 0x41500000: the dram-abs rule the other
            // arenas use, so the section can be mapped for the guest (below).
            //
            // Round 217: the page pool and the frame-number words used to share one
            // megabyte (words at PA 0x41580000, pages counting up from 0x41500000), so
            // after 128 handed-out pages the pool overwrote its own descriptors and
            // NSKBL read garbage frame numbers back - which is how the non-secure core
            // ended up executing uninitialised low-window memory.  The words now live
            // in their own section (VA 0x01F00000 -> PA 0x41F00000, room for 256 Ki
            // frames) and the pool is capped below it.
            constexpr u32 kPhysPoolBase = 0x41500000u;
            constexpr u32 kPhysPoolPages = 7u;           // the record holds seven ranges
            constexpr u32 kPhysPoolMaxPages = 2560u;     // 10 MiB, below 0x41F00000
            constexpr u32 kPhysPoolWordsVa = 0x01F00000u;
            const u32 partition_va = static_cast<u32>(arm->r[0]);
            const u32 partition_pa =
                partition_va < 0x40000u ? partition_va + 0x40300000u : partition_va;
            const u32 table_va = arm_bus_->read32(partition_pa + 0x9Cu);
            const u32 table_pa = table_va < 0x40000u ? table_va + 0x40300000u : table_va;
            if (table_va != 0u) {
                ensure_arena_section(arm, arm_bus_.get(), kPhysPoolBase - 0x40000000u);
                ensure_arena_section(arm, arm_bus_.get(), kPhysPoolWordsVa);
                bool filled = false;
                for (u32 index = 0; index < kArmCoreCount; ++index) {
                    const u32 record = table_pa + index * 32u;
                    if (arm_bus_->read16(record + 2u) != 0u) continue;
                    if (nskbl_physpool_next_ + kPhysPoolPages > kPhysPoolMaxPages) {
                        if (nskbl_physpool_fills_ < 8u) {
                            ZLB_LOG_WARN("machine",
                                         "NSKBL physical-memory pool exhausted at page %u "
                                         "(development substitution)",
                                         nskbl_physpool_next_);
                        }
                        continue;
                    }
                    arm_bus_->write16(record + 2u, static_cast<u16>(kPhysPoolPages));
                    for (u32 slot = 0; slot < kPhysPoolPages; ++slot) {
                        const u32 page = kPhysPoolBase +
                                         (nskbl_physpool_next_ + slot) * 0x1000u;
                        for (u32 offset = 0; offset < 0x1000u; offset += 4u) {
                            arm_bus_->write32(page + offset, 0u);
                        }
                        // The consumer (0x5100C010) does `r0 = *value; r0 <<= 12`, so
                        // the record holds the address of a word with the frame number.
                        const u32 descriptor_va =
                            kPhysPoolWordsVa + (nskbl_physpool_next_ + slot) * 4u;
                        arm_bus_->write32(descriptor_va, page >> 12);
                        arm_bus_->write32(record + 4u + slot * 4u, descriptor_va);
                    }
                    nskbl_physpool_next_ += kPhysPoolPages;
                    filled = true;
                }
                if (filled) {
                    ++boot_pc_fixes_;
                    if (nskbl_physpool_fills_ < 8u) {
                        ++nskbl_physpool_fills_;
                        ZLB_LOG_INFO("machine",
                                     "NSKBL physical-memory range table seeded with %u pages "
                                     "(record 0x%08X, next page 0x%08X; development "
                                     "substitution)",
                                     kPhysPoolPages, table_pa,
                                     kPhysPoolBase + nskbl_physpool_next_ * 0x1000u);
                        add_milestone("NSKBL physical-memory ranges supplied (development "
                                      "substitution)");
                    }
                }
            }
        }
    }

    // Experiment (round 200): the sysroot path asks the kernel physical partition for
    // a 64 KiB range (0x5100576A) and gets 0x80020005 back, so it exits at 0x51005718
    // and SceKernelSysrootClass is never registered (docs/NSKBL.md 8.36).  Treat the
    // request as satisfied at the return point: clear the error and put a frame-number
    // descriptor into the caller's out slot.  Same opt-in switch as the range supplier.
    if (physpool && pc == 0x5100576Eu && core < static_cast<u32>(kArmCoreCount)) {
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const s32 result = static_cast<s32>(arm->r[0]);
            if (result < 0) {
                // Round 217: the frame-number words of this path live in their own
                // section too (VA 0x01F80000 -> PA 0x41F80000), so they cannot be
                // overwritten by the pages the other path hands out.
                constexpr u32 kExperimentBase = 0x41500000u;
                constexpr u32 kExperimentWordsVa = 0x01F80000u;
                ensure_arena_section(arm, arm_bus_.get(), kExperimentWordsVa);
                const u32 descriptor_va =
                    kExperimentWordsVa + (nskbl_physpool_next_ & 0x3FFu) * 4u;
                const u32 page = kExperimentBase + (nskbl_physpool_next_ & 0xFFu) * 0x1000u;
                arm_bus_->write32(descriptor_va, page >> 12);
                const u32 out_slot = static_cast<u32>(arm->r[13]) + 0x1Cu;
                arm_bus_->write32(out_slot, descriptor_va);
                arm->set_register("r0", 0u);
                ++nskbl_physpool_fills_;
                if (nskbl_physpool_fills_ < 6u) {
                    ZLB_LOG_INFO("machine",
                                 "NSKBL range request for 0x%08X treated as satisfied "
                                 "(descriptor 0x%08X, out slot 0x%08X; development "
                                 "substitution)",
                                 static_cast<u32>(arm->r[7]), descriptor_va, out_slot);
                    add_milestone("NSKBL range request satisfied (development substitution)");
                }
            }
        }
    }

    // Substitution (round 204): NSKBL's low window page table.  The kernel boot
    // loader leaves VA 0x30000 described as a 64 KiB large page based at physical
    // 0x00000000; the model does not back that physical range, so NSKBL's object
    // writes there (pc 0x5100B4E2) abort as a bus error - and because the address
    // *is* mapped, the fault hook never runs (docs/NSKBL.md 8.39/8.40).  Fix the
    // mapping instead: on NSKBL's first instruction rewrite L2 indices 0x30-0x3F as
    // small pages pointing at PA = VA + 0x40300000, the low-window rule the rest of
    // the model already assumes.  ZLB_NSKBL_LOWWIN=1 enables it.
    static const bool lowwin_fix = [] {
        const char* value = std::getenv("ZLB_NSKBL_LOWWIN");
        return value != nullptr && value[0] != '0';
    }();
    if (lowwin_fix && (pc == 0x510002E4u || pc == 0x5100B41Cu) && core < static_cast<u32>(kArmCoreCount)) {
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 l1_base = arm->mmu.ttbr0 & 0xFFFFC000u;
            const u32 l1_desc = arm_bus_->read32(l1_base);
            // Round 205: report the entry itself, so the case where the fix does
            // not apply (or where the tables were rebuilt) is visible in the log.
            if (pc == 0x5100B41Cu) {
                ZLB_LOG_INFO("machine",
                             "NSKBL low window probe: TTBR0 0x%08X L1 0x%08X type %u "
                             "(L2/L1 base 0x%08X)",
                             static_cast<u32>(arm->mmu.ttbr0), l1_desc,
                             l1_desc & 3u, l1_desc & 0xFFFFFC00u);
            }
            if ((l1_desc & 3u) == 1u) {
                const u32 l2_base = l1_desc & 0xFFFFFC00u;
                u32 attributes = arm_bus_->read32(l2_base) & 0xFFFu;
                if ((attributes & 3u) != 2u) attributes = 0x47Eu;   // small page, AP=11
                for (u32 index = 0x30u; index <= 0x3Fu; ++index) {
                    const u32 va = index << 12;
                    arm_bus_->write32(l2_base + index * 4u,
                                      ((va + 0x40300000u) & 0xFFFFF000u) | attributes);
                }
                ++boot_pc_fixes_;
                ZLB_LOG_INFO("machine",
                             "NSKBL low window remapped: VA 0x30000-0x3FFFF -> PA 0x40300000+ "
                             "(L2 0x%08X, development substitution)", l2_base);
                add_milestone("NSKBL low window remapped (development substitution)");
            }
        }
    }

    // Substitution (round 208): the class-magic check.  0x5100EEBE loads obj->[0x24]
    // and compares it with the global [0x5113B604], jumping to the fatal sink when
    // they differ (docs/NSKBL.md 8.44).  The object that trips it (0x4010047E) never
    // went through NSKBL's class constructor, so its field is not the expected magic;
    // copy the global into the field at the check's entry, which is what the
    // constructor would have written.  ZLB_NSKBL_MAGIC=1 enables it.
    static const bool magic_fix = [] {
        const char* value = std::getenv("ZLB_NSKBL_MAGIC");
        return value != nullptr && value[0] != '0';
    }();
    if (magic_fix && pc == 0x5100EEA8u && core < static_cast<u32>(kArmCoreCount)) {
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 object = static_cast<u32>(arm->r[0]);
            const u32 expected = arm_bus_->read32(0x5113B604u);
            const u32 actual = (object != 0u) ? arm_bus_->read32(object + 0x24u) : 0u;
            // Round 210: publish the magic on *both* sides - the global (which NSKBL
            // never initialises in the model) and the object's field - so the check at
            // 0x5100EEBE cannot fail for either reason.
            // Round 211: align on *every* entry.  The scan of branches into the fatal
            // tail shows that only 0x5100EEC0 (this magic `bne`) ever executes, and the
            // earlier version only acted while the global was still zero, so later
            // objects kept failing the comparison.
            if (object != 0u && actual != expected) {
                // The global class magic is never initialised in the model (write trap:
                // only the zeroing at 0x510008F8), so publish the magic the object
                // already carries - what NSKBL's own class initialisation would have done.
                // Round 211b: publish the global first (NSKBL never initialises it in
                // the model), then align the object's field - doing it the other way
                // round left the two sides different.
                arm_bus_->write32(0x5113B604u, actual);
                arm_bus_->write32(object + 0x24u, actual);
                ++boot_pc_fixes_;
                if (magic_fixes_ < 6u) {
                    ++magic_fixes_;
                    ZLB_LOG_INFO("machine",
                                 "NSKBL class magic published as 0x%08X (object 0x%08X, was "
                                 "0x%08X vs global 0x%08X; development substitution)",
                                 actual, object, actual, expected);
                    add_milestone("NSKBL class magic published (development substitution)");
                }
            } else if (object != 0u && actual != expected) {
                arm_bus_->write32(object + 0x24u, expected);
                ++boot_pc_fixes_;
                if (magic_fixes_ < 6u) {
                    ++magic_fixes_;
                    ZLB_LOG_INFO("machine",
                                 "NSKBL class magic aligned on object 0x%08X: 0x%08X -> "
                                 "0x%08X (development substitution)",
                                 object, actual, expected);
                    add_milestone("NSKBL class magic aligned (development substitution)");
                }
            }
        }
    }

    // Substitution (round 165): NSKBL's spinlock acquire 0x51014970 livelocks when
    // the object pointer is NULL.  Measured chain: the dispatch at 0x5100B726 reads
    // the list head `[[0x5113B5AC] + 0x64]` (the global holds VA 0x4900 - NSKBL's own
    // memory-map object at PA 0x40304900, whose +0x64 stays zero), passes 0 to the
    // setter 0x5100B41C, and the lock then lands at VA 0x28 - the low boot page, where
    // the KBL left its own scratch value 0x4680 (write trap: pc 0x4002CE90, many times)
    // - so `ldrex/strex` can never acquire it and all cores sit in WFE
    // (docs/NSKBL.md 8.6).  On hardware the list head is not NULL, so the lock lands on
    // a real object.  Skip the acquire when the lock address is below the first page,
    // the same treatment the KBL's own locks get.  ZLB_NSKBL_NOLOCK=0 and
    // ZLB_NO_SUBSTITUTION=1 disable it.
    //
    // Round 215: the same helper is also entered with an *unaligned* address, which
    // cannot be a lock object at all - ARM `ldrex`/`strex` require word alignment and
    // the address the run passes is a Thumb code pointer (`0x40021D51`, i.e. the value
    // of `[fixedheap+0x4C] + 8`, a field the object's constructor would have filled
    // with a lock).  Adding the alignment test to the "bogus address" predicate keeps
    // the substitution's meaning ("that is not a lock") while covering this case.
    static const bool nskbl_nolock = [] {
        const char* value = std::getenv("ZLB_NSKBL_NOLOCK");
        if (value != nullptr && value[0] == '0') return false;
        return substitutions_enabled_static();
    }();
    if (nskbl_nolock && pc == 0x51014970u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                const u32 lock_va = static_cast<u32>(arm->r[0]);
                if (lock_va < 0x1000u || (lock_va & 3u) != 0u) {
                    arm->set_pc(arm->r[14] & ~1u);          // bx lr - skip the acquire
                    arm->set_register("THUMB", 1u);
                    ++boot_pc_fixes_;
                    if (nskbl_lock_skips_ < 8u) {
                        ++nskbl_lock_skips_;
                        ZLB_LOG_INFO("machine",
                                     "NSKBL spinlock skipped at bogus address 0x%08X (arm%u, "
                                     "development substitution)",
                                     lock_va, core);
                        add_milestone("NSKBL spinlock skipped at a bogus address (development "
                                      "substitution)");
                    }
                    return true;
                }
            }
        }
    }

    // Substitution (round 164, default on): the KBL hands every *secondary* core the
    // same stage stack (0x4000 - measured with a write trap on its per-core
    // structures at 0x40020Bxx/0x400207B0), so the three secondaries run the whole
    // boot-setup stage on one stack, their frames overlap and the barrier's
    // `pop {r4-r6,pc}` at 0x4003B3D0 reads a zero (ZLB_KBL_TRACE_ZERO=1:
    // `arm1 from pc=0x4003B3D0 sp=0x3F10`).  On hardware the stage before the KBL
    // gives each core its own stack through the ARM boot context, which the model
    // does not have (docs/STATUS.md 3.1).  The trampoline 0x4003A140 is
    // `mov sp,r2; bx r1`, so biasing r2 per core gives each secondary its own 4 KiB
    // page inside the same low window.  Measured effect: NSKBL goes 0xA4 -> 0xA7
    // (all four cores pass "other cores start" and the MMU/VBAR step and run at
    // 0x80000000), where without it the secondaries die in the barrier.
    // ZLB_KBL_CORE_STACK=<hex> overrides the step, 0 disables it,
    // ZLB_NO_SUBSTITUTION=1 disables every substitution.
    static const u32 core_stack_step = [] {
        const char* value = std::getenv("ZLB_KBL_CORE_STACK");
        if (value != nullptr) return static_cast<u32>(std::strtoul(value, nullptr, 16));
        return substitutions_enabled_static() ? 0x1000u : 0u;
    }();
    if (core_stack_step != 0u && pc == 0x4003A140u && core > 0u) {
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 biased = static_cast<u32>(arm->r[2]) + core * core_stack_step;
            arm->set_register("r2", biased);
            if (core_stack_biases_ < 4u) {
                ++core_stack_biases_;
                ZLB_LOG_INFO("machine",
                             "per-core stage stack: arm%u 0x%08X -> 0x%08X (development "
                             "substitution; ZLB_KBL_CORE_STACK=%X)",
                             core, static_cast<u32>(arm->r[2]), biased, core_stack_step);
                add_milestone("per-core ARM stage stack for arm" + std::to_string(core) +
                              " (development substitution)");
            }
        }
    }

    // Diagnostic (round 176): NSKBL's printf-style logger at 0x51011B5C is called from
    // fourteen places, and the caller that passes the allocation-failure format is the
    // wrapper around sceKernelAllocHeapMemory - which is where the NULL heap comes from
    // (docs/NSKBL.md 8.16-8.20).  ZLB_NSKBL_LOG_CALLS=1 logs each call with its caller
    // and the format pointer so that wrapper can be named without a breakpoint session.
    static const bool log_calls = [] {
        const char* value = std::getenv("ZLB_NSKBL_LOG_CALLS");
        return value != nullptr && value[0] != '0';
    }();
    // Diagnostic (round 177/178): the printf core is 0x51011078.  Hooking its entry and
    // dumping the caller's stack names the chain that reports the allocation failures
    // (`format = 0x5102889C`), which is where the NULL heap comes from.
    // Diagnostic (round 182): NSKBL reports its progress through the printf core
    // 0x51011078 (format in r1).  Logging every *distinct* format once gives the
    // complete list of messages the model's run produces - and, by omission, shows
    // which part of the firmware's output never happens.  ZLB_NSKBL_MSG=1 enables it.
    static const bool log_messages = [] {
        const char* value = std::getenv("ZLB_NSKBL_MSG");
        return value != nullptr && value[0] != '0';
    }();
    if (log_messages && pc == 0x51011078u && core < static_cast<u32>(kArmCoreCount)) {
        static u32 seen[96];
        static u32 seen_count = 0;
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 fmt = static_cast<u32>(arm->r[1]);
            bool known = false;
            for (u32 i = 0; i < seen_count && !known; ++i) {
                known = seen[i] == fmt;
            }
            if (!known && seen_count < 96u) {
                seen[seen_count++] = fmt;
                const u32 sp = static_cast<u32>(arm->r[13]);
                u32 words[8] = {0, 0, 0, 0, 0, 0, 0, 0};
                for (u32 i = 0; i < 8u; ++i) {
                    words[i] = arm_bus().read32(sp + i * 4u);
                }
                ZLB_LOG_INFO("machine",
                             "NSKBL message #%u arm%u from 0x%08X format=0x%08X sp=0x%08X "
                             "stack=[%08X %08X %08X %08X %08X %08X %08X %08X]",
                             seen_count, core, previous_pc, fmt, sp, words[0], words[1],
                             words[2], words[3], words[4], words[5], words[6], words[7]);
            }
        }
    }

    if (log_calls && pc == 0x51011078u && core < static_cast<u32>(kArmCoreCount)) {
        static u32 logged = 0;
        if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
            const u32 fmt = static_cast<u32>(arm->r[1]);
            // Round 177: log every logger call (no format filter) and remember the last
            // few, so the late, repeated calls - the allocation-failure reports - can be
            // told apart from the boot banner.
            if (logged < 40u || (fmt >= 0x51028800u && fmt < 0x51028A00u && logged < 200u)) {
                ++logged;
                const u32 sp = static_cast<u32>(arm->r[13]);
                u32 words[6] = {0, 0, 0, 0, 0, 0};
                for (u32 i = 0; i < 6u; ++i) {
                    words[i] = arm_bus().read32(sp + i * 4u);
                }
                ZLB_LOG_INFO("machine",
                             "alloc-report arm%u from 0x%08X: r0=0x%08X format=0x%08X "
                             "r2=0x%08X r3=0x%08X lr=0x%08X sp=0x%08X stack=[%08X %08X "
                             "%08X %08X %08X %08X]",
                             core, previous_pc, static_cast<u32>(arm->r[0]), fmt,
                             static_cast<u32>(arm->r[2]), static_cast<u32>(arm->r[3]),
                             static_cast<u32>(arm->r[14]), sp, words[0], words[1], words[2],
                             words[3], words[4], words[5]);
            }
        }
    }

    // Diagnostic (round 164): the KBL's *secondary* cores die by jumping to the zero
    // page (pc == 0) and then walking the low window until they fetch open bus.  The
    // instruction that sends them there is what has to be named, and
    // ZLB_KBL_TRACE_PC cannot express pc = 0 (its guard is `trace_pc != 0`).
    // ZLB_KBL_TRACE_ZERO=1 logs the first few arrivals at pc == 0 with the pc that
    // led there and the register file.
    static const bool trace_zero = [] {
        return std::getenv("ZLB_KBL_TRACE_ZERO") != nullptr;
    }();
    if (trace_zero && pc == 0u && trace_zero_hits_ < 8u) {
        if (core < static_cast<u32>(kArmCoreCount)) {
            if (ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get())) {
                ++trace_zero_hits_;
                u64 lr = 0, sp = 0;
                arm->get_register("r14", lr);
                arm->get_register("r13", sp);
                ZLB_LOG_INFO("machine",
                             "trace zero-page: arm%u from pc=0x%08X lr=0x%08X sp=0x%08X cpsr=0x%08X "
                             "r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X r4=0x%08X r5=0x%08X",
                             core, previous_pc, static_cast<u32>(lr), static_cast<u32>(sp),
                             arm->cpsr, arm->r[0], arm->r[1], arm->r[2], arm->r[3], arm->r[4],
                             arm->r[5]);
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

    // Substitution (round 108, corrected round 114, turned off by default in round 158):
    // the getter 0x4002C1AC builds the instance pointer as `[obj+0x14] + table[size_class]`.
    // The object's base field `[obj+0x14]` is the 0xFFFFFFFF sentinel the class
    // constructor stores (round 108.1 measured the store itself), because the stage the
    // model stands in for never fills it.  Round 114: the old stand-in replaced the
    // getter *result* at 0x4002C1CC and only caught `0xFFFFFFFF + 0`; the table's
    // non-zero entries are 0x1000/0x2000, so the sum wrapped to 0x0FFF/0x1FFF.
    //
    // Round 158 found that *filling* the base with the model's instance arena is what
    // kept the loader out of its own boot-setup stage: with the arena in place the
    // heap's block field `[heap+0x38]` ends up on an arena page whose +0x1C carries the
    // class signature 0xD2519E9B, the class method 0x40031654 is entered with that
    // signature as `this` and the run dies on the data abort at 0x40031674 - before the
    // KBL ever reaches checkpoint 0x88.  Leaving the sentinel alone lets the loader
    // take its own path: it enters the boot-setup stage 0x400204A8 on all four cores,
    // reports checkpoint 0x88 and calls the NSKBL loader from there.  The arena is
    // therefore *off* by default now; ZLB_KBL_INSTANCE_BLOCK=1 restores round 114.
    static const bool supply_instance_blocks = [] {
        const char* value = std::getenv("ZLB_KBL_INSTANCE_BLOCK");
        return value != nullptr && value[0] != '0';
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

// Development substitution (round 366): give NSKBL's storage driver the device
// object its code reads.  See the declaration in vita.h for the measurements; the
// write helper follows supply_kbl_class_table above (translate the VA, write through
// the bus, so the model's own low-window substitution stays in charge of mapping).
// Development substitution (round 378): the device method.  The driver dispatches
// through `[device+0x24A0]` (`0x5101D6E4 ldr r2,[device+0x24A0]; 0x5101D6E6 ldr r1,[r2];
// 0x5101D6E8 blx r1`, with the request in r0); round 377 measured that no image in the
// workspace contains that method (NSKBL never materialises the SDIF base, the table at
// 0x51029FC0 is data read only by the secure KBL), so it is modelled here: perform the
// storage operation the request describes and return success.  Field layout measured in
// rounds 371-373 from the SDIF command writer 0x51022640 and the dispatch entry
// 0x51022604.
bool Vita::serve_nskbl_device_call(u32 core) {
    if (core >= static_cast<u32>(kArmCoreCount)) return false;
    ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get());
    if (arm == nullptr || !emmc_) return false;
    // NOTE: `ArmCore::translate()` is the raw walk - it fails for the low window and for
    // mappings the model installs through its fault hook, which is exactly where the
    // driver's requests live (measured: reading request+8 of 0x5117D400 failed with the
    // raw call while the CPU accesses the same address fine).  `translate_or_fix()`
    // applies the same fault fixups the CPU uses, so the service sees what the guest sees.
    auto read_va = [&](u32 va, bool& ok) {
        const arm::MmResult mm = arm->translate_or_fix(va, false, false);
        if (!mm.ok) {
            ok = false;
            return 0u;
        }
        ok = true;
        return arm_bus_->read32(mm.phys_addr);
    };
    auto write_va = [&](u32 va, u32 value) {
        const arm::MmResult mm = arm->translate_or_fix(va, true, false);
        if (!mm.ok) return false;
        arm_bus_->write32(mm.phys_addr, value);
        return true;
    };
    auto write_half = [&](u32 va, u16 value) {
        const arm::MmResult mm = arm->translate_or_fix(va, true, false);
        if (!mm.ok) return false;
        arm_bus_->write16(mm.phys_addr, value);
        return true;
    };
    u64 request64 = 0;
    if (!arm->get_register("r0", request64) || request64 == 0u) return false;
    const u32 request = static_cast<u32>(request64);
    // Only the requests that come from the *substituted* device object use this service.
    // Measured (round 378): the same dispatch is entered four times, three of them with
    // the driver's own DRAM requests (0x5117D400/0x5117D880/0x5117DD00, method
    // 0x51022681) - those must keep running through the driver's own method, which
    // works; the fourth carries a pool node from the low window (VA 0x300..), and that
    // is the path the real device would answer.
    u64 method64 = 0;
    arm->get_register("r1", method64);
    const u32 method = static_cast<u32>(method64);
    if (request >= 0x00100000u) {
        // Round 379 diagnostic: log what the dispatch is actually asked to do, once per
        // distinct (request, method) pair, so the gate can be drawn from measurement
        // instead of guesswork.
        const u64 key = (static_cast<u64>(request) << 32) | method;
        if (nskbl_service_seen_.find(key) == nskbl_service_seen_.end() &&
            nskbl_service_seen_.size() < 16u) {
            nskbl_service_seen_.insert(key);
            ZLB_LOG_INFO("machine",
                         "NSKBL device dispatch: request 0x%08X method 0x%08X r7=0x%08X (not served: "
                         "outside the substituted object) (ZLB_NSKBL_SERVICE=1)",
                         request, method, static_cast<u32>(arm->r[7]));
        }
        return false;
    }
    bool ok = false;
    // Diagnostic (round 378): the first few attempts log where they stop, because a
    // failed translation here is indistinguishable from "the method was never called".
    const auto note_failure = [&](const char* what, u32 offset, u32 va) {
        if (nskbl_service_fail_logs_ < 8u) {
            ++nskbl_service_fail_logs_;
            ZLB_LOG_INFO("machine",
                         "NSKBL device service: cannot %s request 0x%08X + 0x%X (VA 0x%08X) "
                         "(ZLB_NSKBL_SERVICE=1)",
                         what, request, offset, va);
        }
    };
    const u32 command = read_va(request + 0x08u, ok);
    if (!ok) { note_failure("read", 0x08u, request + 0x08u); return false; }
    const u32 argument = read_va(request + 0x0Cu, ok);
    if (!ok) { note_failure("read", 0x0Cu, request + 0x0Cu); return false; }
    const u32 size_count = read_va(request + 0x24u, ok);
    if (!ok) { note_failure("read", 0x24u, request + 0x24u); return false; }
    const u32 buffer = read_va(request + 0x20u, ok);
    if (!ok) { note_failure("read", 0x20u, request + 0x20u); return false; }
    const u32 table = read_va(request + 0x7Cu, ok);
    if (!ok) { note_failure("read", 0x7Cu, request + 0x7Cu); return false; }
    const u8 index = static_cast<u8>(command & 0xFFu);          // CMD18 = 0x12
    const u16 block_size = static_cast<u16>(size_count & 0xFFFFu);
    const u16 count = static_cast<u16>((size_count >> 16) & 0xFFFFu);

    u32 transferred = 0;
    std::string what = "control";
    // Round 379: the driver polls the device through a *state* word.  Its control helper
    // 0x5101F8F4 reads `ldrh r6,[device+0x241C]`, puts `r6 << 16` into the request's +0x14
    // and stores the answer from +0x18 into the caller's output, so the device is expected
    // to keep a sequence there.  The model keeps that sequence for the completions it
    // produces and answers control requests with it.
    constexpr u32 kStateOffset = 0x241Cu;
    const bool data_request = (index == 17u || index == 18u || index == 24u || index == 25u) && count != 0u;
    if (data_request) {
        const u64 lba = static_cast<u64>(argument) / 512ull;
        // The card's block is 512 bytes.  The request's "size" field is 512 on the driver's
        // own data requests, but the wait's template fills it from the request's command
        // index for the device path (measured: `size 1 count 32` for a CMD18), so clamp it
        // to the card block - otherwise the transfer would be count bytes, not blocks.
        const u32 block_bytes = block_size >= 512u ? block_size : 512u;
        const u32 bytes = block_bytes * count;
        const u32 blocks = bytes / 512u;
        std::vector<u8> data(static_cast<size_t>(blocks) * 512u, 0u);
        if (emmc_->read_blocks(EmmcPartition::User, lba, blocks, data.data())) {
            u32 done = 0;
            if (table != 0u) {
                for (u32 i = 0; i < 64u && done < bytes; ++i) {   // ADMA2 descriptors
                    bool ok2 = false;
                    const u32 word0 = read_va(table + i * 8u, ok2);
                    if (!ok2) break;
                    const u32 target = read_va(table + i * 8u + 4u, ok2);
                    if (!ok2) break;
                    const u16 attr = static_cast<u16>(word0 & 0xFFFFu);
                    u32 length = (word0 >> 16) & 0xFFFFu;
                    if (length == 0u) length = 0x10000u;           // ADMA2: 0 means 64 KiB
                    if ((attr & 1u) == 0u) break;                  // VALID
                    const u32 chunk = std::min(length, bytes - done);
                    const arm::MmResult mm = arm->translate_or_fix(target, true, false);
                    if (mm.ok) arm_bus_->write_bytes(mm.phys_addr, data.data() + done, chunk);
                    done += length;
                    if (attr & 2u) break;                          // END
                }
            } else if (buffer != 0u) {
                const arm::MmResult mm = arm->translate_or_fix(buffer, true, false);
                if (mm.ok) {
                    arm_bus_->write_bytes(mm.phys_addr, data.data(), bytes);
                    done = bytes;
                }
            }
            transferred = done;
        }
        what = format("read lba=%llu blocks=%u (%u bytes moved)", static_cast<unsigned long long>(lba),
                      blocks, transferred);
        if (transferred != 0u) {
            ++nskbl_service_state_;                       // one more completion on the device
            write_half(0x00000240u + kStateOffset, static_cast<u16>(nskbl_service_state_));
        }
    } else {
        // A control request: answer with the device's current sequence, the way the state
        // word the driver reads at +0x241C leads it to expect.
        write_va(request + 0x18u, nskbl_service_state_);
        what = format("control code %u -> state %u", index, nskbl_service_state_);
    }
    // The driver reads these two as the transfer's progress (`0x5101FF12 ldrd r2,r3,
    // [r6,#0x1B8]; ... sub` then stores the delta at `[device+0x9B0+0x12]`).
    write_va(request + 0x1B0u, 0u);
    write_va(request + 0x1B8u, transferred);
    if (nskbl_service_calls_ < 32u) {
        ZLB_LOG_INFO("machine",
                     "NSKBL device service #%u: request 0x%08X cmd 0x%02X arg 0x%08X size %u count %u "
                     "table 0x%08X buffer 0x%08X -> %s (ZLB_NSKBL_SERVICE=1, development substitution)",
                     nskbl_service_calls_, request, index, argument, block_size, count, table, buffer,
                     what.c_str());
    }
    ++nskbl_service_calls_;
    arm->set_register("r0", 0u);          // success
    arm->set_pc(0x5101D6EAu);             // past the blx
    return true;
}

bool Vita::supply_nskbl_device_object(u32 core) {
    if (core >= static_cast<u32>(kArmCoreCount)) return false;
    ArmCore* arm = dynamic_cast<ArmCore*>(arm_cores_[core].get());
    if (arm == nullptr) return false;
    constexpr u32 kDeviceVa = 0x00000240u;    // hard-wired by the driver (0x5101FDBC)
    constexpr u32 kSdifBase = 0xE0B00000u;    // measured at the working command writer
    // Round 380: the driver's *own* method table, found in the image data at 0x5102A2C8.
    // Its first entry is the submit method the driver's own (working) requests are
    // dispatched with - measured in the round-379 trace as `method 0x51022681`:
    //   [0x5102A2C8] = 0x51022681  submit
    //   [0x5102A2CC] = 0x51022605  the SDIF command writer wrapper (what the earlier
    //                              substitution pointed at, which is *not* what the
    //                              driver uses for its data requests)
    //   [0x5102A2D0] = 0x5102254D  the failure handler (0x5101D6F2 `ldr r1,[r2,#8]`)
    //   followed by the module names ("SceSdif", "SceSfat", ...)
    // Pointing the device object at this table - instead of a hand-made table that held
    // 0x51022605 - makes the device path use the very method the driver is measured to
    // use successfully for its own volume reads.
    constexpr u32 kDriverMethodTable = 0x5102A2C8u;
    constexpr u32 kSubmitRoutine = 0x51022681u;  // [0x5102A2C8] - the driver's submit method
    constexpr u32 kNodeVa = 0x00000300u;
    auto write_va = [&](u32 va, u32 value) {
        u32 pa = 0;
        std::string fault;
        if (!arm->translate(va, true, false, pa, fault)) return false;
        arm_bus_->write32(pa, value);
        return true;
    };
    // +0x2430 is the field the command writer reads as its register base
    // (5102260C add r3,r4,#0x2400 ; 51022610 ldr r4,[r3,#0x30]).
    if (!write_va(kDeviceVa + 0x2430u, kSdifBase)) return false;
    // +0x2440 is copied into every request at +0x70 (5101D670/5101D67C).
    write_va(kDeviceVa + 0x2440u, 1u);
    // Round 385: +0x2410 is the field the submission chain insists on before it hands a
    // request to the device methods.  Measured with a PCTRAP over 0x5101EE98 (the chain
    // both the wait and the driver's own reads go through): at 0x5101EF20 it reads
    // `ldr r0,[r6,#16]` with r6 = device + 0x2400 and, when r0 is zero, takes the cold
    // block at 0x5101F0CC that presets r3 = 0x8032001A (0x5101F0D0) and returns it.  For
    // the driver's own requests r6 = its own object 0x5117EF00 and that word is 1, so the
    // check passes; with the object supplied here it was zero and every submission from
    // the substituted pool node came back 0x8032001A - the error NSKBL prints on the UART
    // ("0x8032001a 1169 0 0x11c60(72800) 32", docs/NSKBL.md round 383).
    write_va(kDeviceVa + 0x2410u, 1u);
    // +0x2480 + 0x20 -> the dispatch table; its first entry is the "submit" method the
    // device routine calls with the request in r0 (5101D69A-5101D6A4).
    write_va(kDeviceVa + 0x2480u + 0x20u, kDriverMethodTable);
    // The pool of free nodes at +0x2400 (VA 0x2640).  Round 371: the wait pops a node,
    // fills it with its own template (`vstr d16,[r0]` = device + flags 0x514), sets +8,
    // +0x0C and +0x24/+0x26, then hands it to the device submission 0x5101F6BC.  One
    // node satisfies the first pop but not the second one on the `r12 == 12` branch
    // (0x5102057A), which is why a single node is bit-identical to no substitution at
    // all - measured twice (rounds 368/376).  Eight nodes let the driver reach the
    // device dispatch at 0x5101D6E8.
    constexpr u32 kNodeCount = 8u;
    // Round 380: the nodes must also carry somewhere for the data to go.  With the driver's
    // own method table the device path runs the driver's real transfer code, which programs
    // the SDIF from the node's ADMA2 table at +0x7C and buffer at +0x20.  Measured: adding
    // the table and the buffer changed nothing yet (same 22 eMMC reads, same instruction
    // count as with both zero), so this is preparation for the point where the transfer is
    // actually attempted rather than a fix - one descriptor covers the measured 16 KiB.
    constexpr u32 kDmaBufferVa = 0x51100000u;
    constexpr u32 kDmaBufferSize = 0x4000u;
    constexpr u32 kDmaTableVa = 0x510FF000u;
    write_va(kDmaTableVa, (kDmaBufferSize << 16) | 0x0023u);      // VALID|END|ACT0, 16 KiB
    write_va(kDmaTableVa + 4u, kDmaBufferVa);
    for (u32 i = 0; i < kNodeCount; ++i) {
        const u32 node = kNodeVa + i * 0x200u;
        write_va(node + 0x00u, kDeviceVa);
        write_va(node + 0x04u, 0x80000514u);   // dispatchable: bit 0x400 gates the indirect calls
        write_va(node + 0x08u, 0x12u);
        write_va(node + 0x20u, kDmaBufferVa);
        write_va(node + 0x7Cu, kDmaTableVa);
        write_va(node + 0x60u, (i + 1u < kNodeCount) ? (node + 0x200u) : 0u);
    }
    write_va(kDeviceVa + 0x2400u, kNodeVa);                              // head
    write_va(kDeviceVa + 0x2404u, kNodeVa + (kNodeCount - 1u) * 0x200u); // tail
    ZLB_LOG_INFO("machine",
                 "NSKBL device object supplied at VA 0x%03X (SDIF base 0x%08X, method table 0x%08X "
                 "-> submit 0x%08X, %u nodes from 0x%03X) (ZLB_NSKBL_DEV=1, development substitution)",
                 kDeviceVa, kSdifBase, kDriverMethodTable, kSubmitRoutine, kNodeCount, kNodeVa);
    add_milestone("NSKBL device object supplied (development substitution)");
    return true;
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
        // Each page the partition hands out carries *its own physical page number*
        // in the first word: the heap builder reads it back as `[block] << 12`
        // (0x40031C32 -> 0x4003223C) to learn the physical base of the window it is
        // about to map, so the loader's own carve path must have written it.  The
        // substitution therefore offers to write it too (ZLB_KBL_PAGENUM=1).
        //
        // Measured (round 157): with the page numbers written, *all three* low
        // windows (VA 0x10000/0x12000/0x14000) end up on their own block pages and
        // the boot stops earlier (checkpoint 0x84 instead of 0x87) - i.e. some of
        // those windows are expected to alias physical page 0, where the class
        // descriptors live.  Off by default; the knob is the switch between the two
        // readings of the loader's page bookkeeping.
        static const bool write_page_number = [] {
            const char* value = std::getenv("ZLB_KBL_PAGENUM");
            return value != nullptr && value[0] != '0';
        }();
        if (!write_va(block, write_page_number ? (block >> 12u) : 0u)) {
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
    // and non-secure kernel boot loaders read (wiki: KBL_Param).  It lives at
    // board::kKblParamBase = 0x1F000100 in SPAD32K; the earlier 0x1F000040 came
    // from reading the wiki's "fallback DIP switch buffer at physical 0x80" as the
    // record base, which is 0xC0 too low (the loader's own builder at 0x41B4A
    // loads 0x1F000100).  The per-console chain that would build it cannot
    // complete in this model, so the documented fields are written here (a
    // development substitution; the magic and layout follow the wiki).
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
    add_milestone(format("SceKblParam built at 0x%08X (development substitution, wiki layout)", base));
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

    // SceKblParam is built by the second loader itself: its cold path reaches
    // checkpoint 0x5A and the builder at 0x41B4A writes the record at base
    // 0x1F000100 (docs/SYSCON.md 8.15, verified field by field against the
    // wiki's KBL_Param layout).  The C++ builder below is only the fallback for
    // the run where the development substitutions are off and the loader cannot
    // get that far; the mirror is always needed, because the ARM reads the
    // record through the PA 0 alias of the power scratchpad.
    if (!substitutions_enabled_static()) build_kbl_param();

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

// ---------------------------------------------------------------------------
// NSKBL - the non-secure kernel boot loader
// ---------------------------------------------------------------------------
//
// kernel_boot_loader.self carries the NSKBL as its last segment: an ARZL stream
// at PA 0x50000000 (segment 4, ARZL header "ARZL" + the range coder).  SKBL
// decodes it with its own routines and jumps to 0x51000000 in the non-secure
// world (wiki NSKBL, and the call site at 0x40020588 in the KBL's own code:
// `sceArlzDecode(0x51000000, 0x1000000, 0x50000004, NULL)` followed by the ARM
// filter at 0x4003CB40 with version 0).
//
// SKBL's boot path cannot reach that call site in the model yet (it is stuck in
// its per-console object manager), so this stage runs the two firmware routines
// itself on the emulated core.  Nothing about ARZL or the ARM filter is
// reimplemented: the code that runs is the code in kernel_boot_loader.self.

namespace {
/// Free DRAM above the staged second loader (0x407C0000+0x16C00): the scratch
/// stack for the firmware calls, and the return sentinel they unwind to.
constexpr u32 kFirmwareCallStackTop = 0x40800000;
constexpr u32 kFirmwareCallSentinel = 0x407E0000;
constexpr u32 kArlzDecodeEntry = 0x4003C330;   ///< SceSkbl#sceArlzDecode (ARM mode)
constexpr u32 kArlzArmFilterEntry = 0x4003CB40;  ///< SceSkbl#sceArlzArmFilter
constexpr u32 kNskblCompressedPa = 0x50000000;  ///< where SKBL stages the ARZL stream
constexpr u32 kNskblEntry = 0x51000000;         ///< decoded image / reset vector
constexpr u32 kNskblMaxSize = 0x1000000;        ///< 16 MiB (KBP bootkernimg mapping)
}  // namespace

u32 Vita::arm_call(u32 address, u32 a0, u32 a1, u32 a2, u32 a3, bool* ok) {
    if (ok != nullptr) *ok = false;
    ArmCore* core = dynamic_cast<ArmCore*>(arm_cores_[0].get());
    if (core == nullptr) return 0;

    core->reset(address);
    core->r[0] = a0;
    core->r[1] = a1;
    core->r[2] = a2;
    core->r[3] = a3;
    core->r[13] = kFirmwareCallStackTop;
    core->r[14] = kFirmwareCallSentinel;

    // A plain step loop: the boot substitutions must not run inside a firmware
    // helper (they are keyed to the loader's own PCs, not to these routines).
    const u64 budget = 200000000ull;
    u64 steps = 0;
    while (core->get_pc() != kFirmwareCallSentinel && steps < budget) {
        core->step();
        ++steps;
    }
    if (core->get_pc() != kFirmwareCallSentinel) {
        ZLB_LOG_WARN("machine", "firmware call 0x%08X did not return (%llu steps, pc=0x%08X)", address,
                     static_cast<unsigned long long>(steps), core->get_pc());
        return core->r[0];
    }
    if (ok != nullptr) *ok = true;
    ZLB_LOG_INFO("machine", "firmware call 0x%08X returned r0=0x%08X after %llu steps", address,
                 core->r[0], static_cast<unsigned long long>(steps));
    return core->r[0];
}

bool Vita::start_nskbl() {
    if (!arm_) return false;

    // The decoder lives in the KBL, and the compressed NSKBL is one of its
    // segments, so the image has to be in the ARM's memory either way.
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
        ZLB_LOG_ERROR("machine", "NSKBL: kernel_boot_loader.self not found");
        return false;
    }

    LoadResult result = load_image(*arm_bus_, kbl, "kernel_boot_loader.self", keys_);
    if (!result.ok) {
        ZLB_LOG_ERROR("machine", "NSKBL: kernel boot loader load failed: %s", result.message.c_str());
        return false;
    }
    // The record NSKBL reads (its boot() copies the KBL Param out of the power
    // scratchpad) and the scratchpad mirror at PA 0.  A full cold boot leaves the
    // record there itself; entering this stage directly does not, so the model's
    // builder fills it in (that is the same fallback the KBL stage uses).
    const bool have_record = arm_bus_->read32(board::kKblParamBase) != 0;
    if (!substitutions_enabled_static() || !have_record) build_kbl_param();
    mirror_cmep_scratch_to_arm();

    const u32 compressed = arm_bus_->read32(kNskblCompressedPa);
    if (compressed != 0x4C5A5241u) {   // "ARZL"
        ZLB_LOG_ERROR("machine", "NSKBL: no ARZL stream at PA 0x%08X (found 0x%08X)", kNskblCompressedPa,
                      compressed);
        return false;
    }

    bool ok = false;
    const u32 decoded = arm_call(kArlzDecodeEntry, kNskblEntry, kNskblMaxSize,
                                 kNskblCompressedPa + 4, 0, &ok);
    if (!ok || decoded == 0 || decoded > kNskblMaxSize) {
        ZLB_LOG_ERROR("machine", "NSKBL: sceArlzDecode failed (r0=0x%08X, ok=%d)", decoded, ok ? 1 : 0);
        return false;
    }
    const u32 filtered = arm_call(kArlzArmFilterEntry, kNskblEntry, decoded, 0, 0, &ok);
    ZLB_LOG_INFO("machine", "NSKBL: ARZL 0x%X bytes at PA 0x%08X decoded to 0x%X bytes at 0x%08X", 0x194CF,
                 kNskblCompressedPa, decoded, kNskblEntry);
    if (!ok) {
        ZLB_LOG_ERROR("machine", "NSKBL: sceArlzArmFilter failed");
        return false;
    }
    if (filtered != 0 && filtered != decoded)
        ZLB_LOG_INFO("machine", "NSKBL: ARM filter reports 0x%X bytes", filtered);

    for (int i = 0; i < kArmCoreCount; ++i) {
        ArmCore* core = dynamic_cast<ArmCore*>(arm_cores_[static_cast<size_t>(i)].get());
        if (core == nullptr) continue;
        core->reset(kNskblEntry);
        // NSKBL is the first code of the non-secure world: SKBL sets SCR.NS and
        // enters it in SVC mode with the MMU off (wiki NSKBL#Reset).
        core->ns_ = true;
        core->scr |= 1u;   // SCR.NS
        core->halted = false;
    }
    boot_.stage = BootStage::NskblEntry;
    boot_.arm_entry = kNskblEntry;
    boot_.arm_released = true;
    boot_.detail = "ARM in the non-secure world on NSKBL";
    add_milestone("NSKBL decoded to 0x" + hex(kNskblEntry, 8) + " by the KBL's own sceArlzDecode (" +
                  source + ")");
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
        // Round 152 experiment: do NOT swallow the jump but let the second loader
        // run its post-secure-kernel continuation - that is where 0x408EC ->
        // 0x41B4A would build SceKblParam for real instead of the substitution.
        // Measured with ZLB_CMEP_HANDOFF_RUN=1: the CMeP reaches the cold branch
        // 0x40858 (it never got there before) and runs ~14.8M instructions, but
        // within 60k slices it does not reach the builder and it never signals
        // the ARM release, so the run ends with the ARM still parked.  Off by
        // default; kept as the starting point for the next round.
        static const bool run_through = [] {
            const char* value = std::getenv("ZLB_CMEP_HANDOFF_RUN");
            return value != nullptr && value[0] != '0';
        }();
        return !run_through;
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
        case BootStage::NskblEntry:
            return start_nskbl();
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
        const u32 status = static_cast<u32>(boot_.cmep_status);
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
    plan.push_back("");
    plan.push_back("model (NSKBL): step 5's second half - the ARZL decode of the non-secure kernel");
    plan.push_back("boot loader - runs in `stage nskbl`.  The decoder and the ARM filter are the");
    plan.push_back("KBL's *own* routines (0x4003C330 and 0x4003CB40, called from the KBL itself at");
    plan.push_back("0x40020588), so only the call is modelled: the model sets up the AAPCS");
    plan.push_back("registers and a scratch stack on arm0 and runs them.  NSKBL starts at");
    plan.push_back("0x51000000 in the non-secure world and reaches its checkpoint 0xA1; the next");
    plan.push_back("wall is its first `smc` into the (not modelled) TrustZone monitor.");
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
    if (wfe_ticks_ != 0u || wfe_irq_wakeups_ != 0u || barrier_unstuck_ != 0u || wfe_wakeups_ != 0u) {
        // How the model got the cluster past its WFE waits: the timer tick
        // (ZLB_KBL_WFE_TICK=1) or the counter-patching engine, plus how many cores
        // each of them actually woke.  Without these counters the two paths look
        // identical in the report (docs/KBL.md 7.1.11).
        out += format("WFE          : ticks=%llu irq_wakeups=%llu counter_patches=%llu "
                      "cluster_wakeups=%llu\n",
                      static_cast<unsigned long long>(wfe_ticks_),
                      static_cast<unsigned long long>(wfe_irq_wakeups_),
                      static_cast<unsigned long long>(barrier_unstuck_),
                      static_cast<unsigned long long>(wfe_wakeups_));
    }
    if (!milestones_.empty()) {
        out += "milestones   :\n";
        for (const auto& milestone : milestones_) out += "  * " + milestone + "\n";
    }
    return out;
}

}  // namespace zlb
