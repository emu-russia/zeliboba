// zeliboba - boot chain orchestration.
#include <algorithm>
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
    if (va >= 0x00100000u) return false;
    if (core >= static_cast<u32>(kArmCoreCount)) return false;

    Cpu* cpu = arm_cores_[core].get();
    ArmCore* arm = dynamic_cast<ArmCore*>(cpu);
    if (!arm || !arm->mmu.enabled()) return false;

    // Walk the core's own TTBR0: L1 -> coarse L2, the layout the KBL installs.
    const u32 l1_base = arm->mmu.ttbr0 & 0xFFFFC000u;
    const u32 l1_entry = arm_bus_->read32(l1_base + ((va >> 20) & 0xFFFu) * 4u);
    if ((l1_entry & 3u) != 1u) return false;                    // not a coarse table
    const u32 l2_base = l1_entry & 0xFFFFFC00u;
    const u32 l2_index = (va >> 12) & 0xFFu;
    const u32 slot = l2_base + l2_index * 4u;
    if (arm_bus_->read32(slot) != 0u) return false;             // already mapped

    // Take the attribute bits from the KBL's own first entry so the substituted
    // page has the same cacheability/permissions as the pages it installed.
    u32 attributes = arm_bus_->read32(l2_base) & 0xFFFu;
    if ((attributes & 3u) != 2u) attributes = 0x47Eu;           // small page, AP=11
    arm_bus_->write32(slot, (va & 0xFFFFF000u) | attributes);

    ++boot_fault_fixes_;
    ZLB_LOG_INFO("machine",
                 "boot window mapping supplied: arm%u VA 0x%08X -> PA 0x%08X (attr 0x%03X) in L2 0x%08X[0x%02X] (%s)",
                 core, va, va, attributes, l2_base, l2_index, fetch ? "fetch" : (write ? "write" : "read"));
    add_milestone("ARM low window mapping supplied for VA 0x" + hex(va, 8) + " (development substitution)");
    return true;
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
    if (!boot_.arm_released && ernie_ && ernie_->soc_released() && arm_ && arm_->halted) {
        start_arm_kernel_boot_loader();
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
    // Vita::cmep_pc_hook): on hardware that is the point where it has told the
    // syscon to reset the ARM at 0x00000000 (wiki, step 4).
    if (secure_kernel_done_) {
        secure_kernel_done_ = false;
        secure_kernel_active_ = false;
        cmep_->halted = true;
        if (ernie_) ernie_->release_soc();
        ZLB_LOG_INFO("boot",
                     "secure kernel finished (jump back to 0x40000): substituting the syscon handshake, "
                     "releasing the ARM");
        add_milestone("CMeP secure kernel finished -> syscon SoC release (development substitution)");
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
