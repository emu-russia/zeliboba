// zeliboba - the machine: buses, devices, cores and scheduling.
#include "machine/vita.h"

#include <algorithm>
#include <cstring>

#include "bus/device.h"
#include "common/log.h"
#include "common/util.h"
#include "cpu/factory.h"
#include "cpu/arm/arm_core.h"
#include "cpu/mep/mep_core.h"
#include "hw/cmep.h"
#include "hw/emmc.h"
#include "hw/soc/soc_internal.h"
#include "hw/syscon.h"

namespace zlb {

namespace {
/// The Vita's Cortex-A9 runs at 333 MHz; used to convert cycles into wall time.
constexpr double kArmClockHz = 333.0e6;
}  // namespace

/// The PL310 L2 cache controller the kernel boot loader programs. Its cache
/// maintenance registers complete immediately: the driver writes a command and
/// then polls bit 0 until it clears, which a plain register file never does.
///   4003A56E  movw r1,#0xFFFF / str.w r1,[r0,#1916]   -> L2CC + 0x77C
///   4003A57A  ldr.w r1,[r0,#1916] / tst r1,#1 / bne   -> wait for bit 0 clear
///   4003A4EA  str r1=0,[r0,#0x730] / poll bit 0        -> "Cache Sync"
class L2CacheController : public RegisterFile {
public:
    explicit L2CacheController(u32 base)
        : RegisterFile(format("Kermit.L2CC@%08X", base), base, 0x10000) {
        define(base + 0x100, "L2CC_FILTERING_START", 0);
        define(base + 0x104, "L2CC_FILTERING_END", 0);
        define(base + 0x730, "L2CC_CACHE_SYNC", 0);
        define(base + 0x77C, "L2CC_MAINTENANCE", 0);
        define(base + 0x7FC, "L2CC_CLEAN_INVALIDATE_WAY", 0);
    }

    u64 read(u32 address, unsigned size) override {
        // Only the registers we actually named keep their value. Everything else
        // in this block behaves like a command/status register whose operation
        // completes immediately: the driver writes a mask and polls bit 0 until it
        // clears, so an undefined register must read back as zero.
        if (values_.count(address) != 0) return RegisterFile::read(address, size);
        return 0;
    }

    void write(u32 address, unsigned size, u64 value) override {
        if (values_.count(address) != 0) {
            RegisterFile::write(address, size, value);
            return;
        }
        // Command register: accepted and finished at once.
        (void)size;
        (void)value;
    }
};

const std::string& workspace_root() {
    static const std::string root = ZLB_WORKSPACE_DIR;
    return root;
}

std::string resolve_workspace_path(const std::string& path) {
    if (path.empty()) return path;
    // Absolute paths (drive letter or UNC) are used as-is.
    if (path.size() >= 2 && path[1] == ':') return path;
    if (path.size() >= 2 && path[0] == '\\' && path[1] == '\\') return path;
    if (!path.empty() && (path[0] == '/' || path[0] == '\\')) return path;
    return path_join(workspace_root(), path);
}

Vita::Vita() = default;
Vita::~Vita() = default;

// ---------------------------------------------------------------------------
// Tooling: PC coverage (ZLB_ARM_COV=1 / ZLB_MEP_COV=1, printed by `cov`)
// ---------------------------------------------------------------------------

void Vita::arm_cov_mark(u32 pc) {
    static const bool enabled = [] {
        const char* value = std::getenv("ZLB_ARM_COV");
        return value != nullptr && value[0] != '0';
    }();
    if (!enabled) return;
    if (arm_cov_bits_.empty()) {
        // The default window is the NSKBL image (0x51000000+256 KiB); the KBL runs
        // at 0x40020000, so its runs pass ZLB_ARM_COV_BASE/SIZE.
        if (const char* base = std::getenv("ZLB_ARM_COV_BASE")) {
            arm_cov_base_ = static_cast<u32>(std::strtoul(base, nullptr, 0));
        }
        if (const char* size = std::getenv("ZLB_ARM_COV_SIZE")) {
            const u32 value = static_cast<u32>(std::strtoul(size, nullptr, 0));
            if (value >= 0x1000u) arm_cov_size_ = value;
        }
        // 2 bytes per bit = one Thumb instruction: at 16 bytes the "did this site execute"
        // question answered yes whenever *any* address in the block ran, which made the
        // missed-edge scan report calls that never happened (round 167).
        if (const char* gran = std::getenv("ZLB_ARM_COV_GRAN")) {
            const u32 value = static_cast<u32>(std::strtoul(gran, nullptr, 0));
            if (value >= 2u && value <= 64u) arm_cov_gran_ = value;
        }
        const size_t bits = arm_cov_size_ / arm_cov_gran_;
        arm_cov_bits_.assign((bits + 7u) / 8u, 0u);
        ZLB_LOG_INFO("machine", "ARM coverage armed: 0x%08X-0x%08X, %u bytes per bit",
                     arm_cov_base_, arm_cov_base_ + arm_cov_size_, arm_cov_gran_);
    }
    if (pc < arm_cov_base_ || pc >= arm_cov_base_ + arm_cov_size_) return;
    const u32 index = (pc - arm_cov_base_) / arm_cov_gran_;
    arm_cov_bits_[index >> 3] |= static_cast<u8>(1u << (index & 7u));
}

bool Vita::arm_cov_executed(u32 addr) const {
    if (arm_cov_bits_.empty()) return false;
    if (addr < arm_cov_base_ || addr >= arm_cov_base_ + arm_cov_size_) return false;
    const u32 index = (addr - arm_cov_base_) / arm_cov_gran_;
    return (arm_cov_bits_[index >> 3] & static_cast<u8>(1u << (index & 7u))) != 0;
}

void Vita::mep_cov_mark(u32 pc) {
    static const bool enabled = [] {
        const char* value = std::getenv("ZLB_MEP_COV");
        return value != nullptr && value[0] != '0';
    }();
    if (!enabled) return;
    if (mep_cov_bits_.empty()) {
        if (const char* base = std::getenv("ZLB_MEP_COV_BASE")) {
            mep_cov_base_ = static_cast<u32>(std::strtoul(base, nullptr, 0));
        }
        if (const char* size = std::getenv("ZLB_MEP_COV_SIZE")) {
            const u32 value = static_cast<u32>(std::strtoul(size, nullptr, 0));
            if (value >= 0x1000u) mep_cov_size_ = value;
        }
        if (const char* gran = std::getenv("ZLB_MEP_COV_GRAN")) {
            const u32 value = static_cast<u32>(std::strtoul(gran, nullptr, 0));
            if (value >= 2u && value <= 64u) mep_cov_gran_ = value;
        }
        const size_t bits = mep_cov_size_ / mep_cov_gran_;
        mep_cov_bits_.assign((bits + 7u) / 8u, 0u);
        ZLB_LOG_INFO("machine", "CMeP coverage armed: 0x%08X-0x%08X, %u bytes per bit",
                     mep_cov_base_, mep_cov_base_ + mep_cov_size_, mep_cov_gran_);
    }
    if (pc < mep_cov_base_ || pc >= mep_cov_base_ + mep_cov_size_) return;
    const u32 index = (pc - mep_cov_base_) / mep_cov_gran_;
    mep_cov_bits_[index >> 3] |= static_cast<u8>(1u << (index & 7u));
}

bool Vita::mep_cov_executed(u32 addr) const {
    if (mep_cov_bits_.empty()) return false;
    if (addr < mep_cov_base_ || addr >= mep_cov_base_ + mep_cov_size_) return false;
    const u32 index = (addr - mep_cov_base_) / mep_cov_gran_;
    return (mep_cov_bits_[index >> 3] & static_cast<u8>(1u << (index & 7u))) != 0;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

void Vita::build(const VitaConfig& config) {
    if (built_) return;
    config_ = config;

    if (config_.first_loader.empty()) config_.first_loader = "dumps/vita_prototype_bootrom.bin";
    if (config_.syscon_firmware.empty()) config_.syscon_firmware = "ernie-master/USS-1001.bin";
    if (config_.emmc_image.empty()) config_.emmc_image = "zeliboba/build/emmc.img";
    if (config_.fs_root.empty()) config_.fs_root = "Vita_104_Firmware/Out";

    build_buses();
    build_devices();
    build_cores();
    wire_bridges();

    built_ = true;
    ZLB_LOG_INFO("machine", "board built: 3 processors, %zu MMIO devices",
                 cmep_bus_->devices().size() + arm_bus_->devices().size() + syscon_bus_->devices().size());
}

void Vita::build_buses() {
    arm_bus_ = std::make_unique<Bus>();
    cmep_bus_ = std::make_unique<Bus>();
    syscon_bus_ = std::make_unique<Bus>();

    shared_sram_.assign(board::kSharedSramSize, 0);
    // The CMeP's private 2 MiB window starts with the 128 KiB SRAM the wiki calls
    // out ("0x00800000 0x20000 S Cmep 128KiB SRAM. Stores second_loader,
    // secure_kernel and Secure Modules"); the ARM sees that SRAM mirrored at
    // PA 0x00040000-0x0005FFFF ("MeP boot. Mirror of physical address 0x00800000").
    // One backing store, two windows.
    cmep_priv_.assign(cmep::kPrivateSize, 0);
    // The main DRAM module is one piece of silicon: the CMeP stages what the
    // ARM later runs (the CMeP second loader builds its ADMA2 table at
    // 0x40000400 and reads the secure kernel / kernel boot loader straight into
    // 0x40000000+), so both processors have to see the same bytes at the same
    // physical addresses. One backing store, two windows.
    dram_.assign(kermit::kScuSize, 0);

    // --- CMeP ("F00D") -----------------------------------------------------
    cmep_bus_->add_ram("cmep_ram", 0x20000, cmep::kRamBase, "CMeP RAM / first loader window (128 KiB)");
    cmep_bus_->add_ram_alias("cmep_priv", cmep::kPrivateBase, cmep::kPrivateSize, cmep_priv_.data(),
                             "CMeP private RAM (128 KiB SRAM + secure kernel)");
    cmep_bus_->add_ram_alias("cmep_dram", kermit::kDramWindowBase, kermit::kScuSize, dram_.data(),
                             "main DRAM seen by the CMeP (ADMA2 table, kernel staging)");
    build_shared_windows();

    // --- ARM Cortex-A9 -----------------------------------------------------
    // PA 0 is an alias of the ScePower scratchpad (wiki: "ARM Boot. By default,
    // alias of physical address 0x1F000000 i.e. ScePower scratchpad"), which is
    // where the second loader leaves SceKblParam and the secure boot stack.
    arm_bus_->add_ram_alias("arm_bootrom", 0x00000000, board::kArmBootWindowSize, shared_sram_.data(),
                            "ARM boot window (alias of the power scratchpad)");
    // PA 0x00040000-0x0005FFFF mirrors the CMeP's 128 KiB SRAM: the kernel boot
    // loader reads the staged images and its parameters through this window.
    arm_bus_->add_ram_alias("arm_mep_boot", board::kArmMepBootBase, board::kArmMepBootSize, cmep_priv_.data(),
                            "MeP boot window (mirror of CMeP SRAM 0x00800000)");
    // The kernel boot loader runs with the MMU off and is linked at 0x40020000,
    // so 0x40000000 is the physical DRAM window; it is also where the kernel
    // modules are mapped through 0x80000000 later on.  Round 162: the MPCore
    // peripheral block no longer shadows this window (see kermit::kScuBase) - the
    // KBL keeps its exception/monitor vector page at PA 0x40000100 and the monitor
    // table at PA 0x40000140.
    arm_bus_->add_ram_alias("arm_priv", kermit::kDramWindowBase, kermit::kScuSize, dram_.data(),                            "physical DRAM window (kernel boot loader and kernel image)");
    arm_bus_->add_ram("arm_dram", 0x04000000, kermit::kDramBase, "main DRAM (64 MiB module window)");
    // Round 142: the KBL maps VA 0x1C000000 identity onto the 2 MiB Scratchpad SRAM
    // (L1[0x1C0] = 0x1C01158E) and uses it as a work buffer; without the backing RAM
    // its byte-copy/bignum loop reads zeroes back and never advances.
    arm_bus_->add_ram("arm_scratchpad", kermit::kScratchpadSramSize, kermit::kScratchpadSramBase,
                      "Scratchpad SRAM (SLSK image, display/camera, PSP eDRAM, BSOD)");
    // 0x50000000 is inside the 512 MiB DRAM window (the wiki puts the ARZL-compressed
    // NSKBL at 0x50000000 and the uncompressed one at 0x51000000), so no separate
    // staging block is mapped there any more.

    // --- Ernie (RL78 syscon) ----------------------------------------------
    syscon_bus_->add_ram("ernie_flash", ernie::kFlashSize, ernie::kFlashBase, "Ernie code/data flash");
}

void Vita::build_shared_windows() {
    // The boot SRAM is physically one block, visible at the same address from
    // both the ARM and the CMeP.
    cmep_bus_->add_ram_alias("cmep_shared_sram", board::kSharedSramBase, board::kSharedSramSize, shared_sram_.data(),
                             "shared boot SRAM (ARM boot ROM -> CMeP)");
    arm_bus_->add_ram_alias("arm_sram", board::kSharedSramBase, board::kSharedSramSize, shared_sram_.data(),
                            "shared boot SRAM (boot ROM staging area)");
}

void Vita::build_devices() {
    emmc_ = std::make_unique<EmmcCard>();

    ernie_ = std::make_unique<ErnieBlock>(*syscon_bus_, emmc_.get());
    ernie_->install();

    cmep_block_ = std::make_unique<CmepBlock>(*cmep_bus_, keys_);
    cmep_block_->install();
    cmep_block_->attach_storage(ernie_.get(), emmc_.get());

    kermit_ = std::make_unique<KermitBlock>(*arm_bus_, emmc_.get());
    kermit_->install();
    kermit_->set_syscon(ernie_.get());
    // The boot chain reaches the syscon over SPI0 (0xE0A00000), not through the
    // CMeP's SC mailbox: the CMeP second loader drives it at 0x436E4. The
    // device lives on the ARM's bus but the CMeP's bus mirrors the same
    // peripherals, so one attachment serves both.
    kermit_->attach_syscon_spi(*cmep_bus_, ernie_.get());
}

void Vita::build_cores() {
    cmep_ = make_cpu(Arch::MeP, *cmep_bus_);
    if (cmep_) cmep_->name = "CMeP";
    // Intercept the first loader service call the second loader makes at the end
    // of its work (see Vita::cmep_pc_hook).  The same per-instruction hook marks
    // the CMeP coverage map (ZLB_MEP_COV=1), so the security core needs no second
    // callback.
    if (MePCore* mep = dynamic_cast<MePCore*>(cmep_.get())) {
        mep->pc_hook = [this](u32 pc) {
            mep_cov_mark(pc);
            return cmep_pc_hook(pc);
        };
    }

    // Kermit is a quad core Cortex-A9 MPCore. Each core gets its own CP15/MMU
    // state (kernel_boot_loader gives every core its own L1 tables) but they share
    // the bus, the GIC and the SCU.
    for (int i = 0; i < kArmCoreCount; ++i) {
        arm_cores_[static_cast<size_t>(i)] = make_cpu(Arch::Arm, *arm_bus_);
        Cpu* core = arm_cores_[static_cast<size_t>(i)].get();
        if (!core) continue;
        core->name = i == 0 ? std::string("ARM Cortex-A9")
                            : format("ARM Cortex-A9 #%d", i);
        if (ArmCore* arm = dynamic_cast<ArmCore*>(core)) {
            arm->core_id_ = static_cast<u32>(i);
            // Give the machine a chance to supply the low-window mappings that the
            // stage before kernel_boot_loader leaves behind (see
            // Vita::satisfy_arm_boot_fault).
            arm->fault_hook = [this](u32 id, u32 va, bool write, bool fetch) {
                return satisfy_arm_boot_fault(id, va, write, fetch);
            };
            // SEV (round 140) wakes every WFE-waiting core: the KBL's barrier spins
            // on WFE until another core's SEV, and instruction-level round-robin
            // otherwise races the sense-reversing counter.
            arm->sev_hook = [this]() {
                for (size_t k = 0; k < kArmCoreCount; ++k) {
                    if (ArmCore* other = dynamic_cast<ArmCore*>(arm_cores_[k].get())) {
                        other->event_pending_ = true;
                        if (other->wfe_waiting_) {
                            other->wfe_waiting_ = false;
                            other->halted = false;
                            other->halt_reason.clear();
                        }
                    }
                }
            };
        }
    }
    arm_ = arm_cores_[0].get();

    // Round 342: give the GIC its CPU.  `kermit_set_cpu` had no callers anywhere, so
    // `Gic::cpu_` stayed null and every interrupt the controller asserted was dropped
    // on the floor - measured as "GIC line assert: ... id=58 (cpu=null)" in
    // Gic::refresh_line, with ArmCore::set_irq never firing even when the SDIF
    // asserted its line 188 times in a run.  The model implements CPU0's interface
    // (see gic.cpp), so core 0 is the one to attach; a run that never enables the
    // controller is unaffected.
    if (arm_ != nullptr) kermit_set_cpu(*arm_bus_, arm_);

    // Round 354 (opt-in, ZLB_NSKBL_VBAR=1): the non-secure VBAR is never written by
    // the guest - the debug log shows only the model's SKBL vector-page mirroring for
    // MVBAR 0x16140 - so `ArmMmu::vector_base()` stays 0 and any interrupt delivered
    // to NSKBL would vector into unmapped low memory.  The model already stages the
    // secure world's vector page (mirrored at PA 0x40326100 for the MMU-off monitor
    // fetch); NSKBL's own vectors live at VA 0x40100, which is what bootchain.cpp:528
    // and the boot code assume.  Without this, enabling the controller in the GIC
    // only produces exceptions that go nowhere - which is the most likely reason the
    // enable substitution of round 343 measured *worse* than strict.
    static const bool set_nskbl_vbar = [] {
        const char* value = std::getenv("ZLB_NSKBL_VBAR");
        return value != nullptr && value[0] != '0';
    }();
    if (set_nskbl_vbar) {
        for (size_t k = 0; k < kArmCoreCount; ++k) {
            if (ArmCore* core = dynamic_cast<ArmCore*>(arm_cores_[k].get())) {
                core->vbar_nonsecure = 0x00040100u;
            }
        }
        ZLB_LOG_INFO("machine",
                     "NSKBL non-secure VBAR set to 0x00040100 on %d cores (development substitution)",
                     static_cast<int>(kArmCoreCount));
    }
    // Round 344: the "the secure world left the controller enabled" policy is NOT
    // turned on by default.  It was measured against the strict behaviour and lost:
    // strict completes 22 eMMC reads and keeps NSKBL's own checkpoints through A9,
    // while assuming the enables (which is what the substitution of round 343 did)
    // drops to 19 reads, skips the whole NSKBL A-series and ends with the secure side
    // halting at KBL checkpoint 0x8E.  The two fixes that made interrupts possible at
    // all stay - Gic::refresh_line() on a device assertion (kermit.cpp) and the GIC
    // actually being given a CPU (above) - but nothing enables the controller, so a
    // default run is unchanged.  ZLB_GIC_CPUIF=1 still forces the policy on and
    // ZLB_GIC_STRICT=1 forces it off, for experiments.

    syscon_ = ernie_->cpu();
    if (syscon_) syscon_->name = "Ernie";
}

void Vita::wire_bridges() {
    // The ARM sees the CMeP mailboxes: that is how the boot chain and the
    // "secure world" services talk to each other.
    for (Device* device : cmep_block_->devices()) {
        if (device->name() == "CMeP.Mailbox") {
            arm_bus_->add_device(std::make_unique<DeviceMirror>(*device, cmep::kMailboxBase, 0x100));
        }
    }

    // The ARM also sees the syscon. The kernel boot loader writes the SC doorbell
    // at 0xE310005C and polls it back (0x4003C01E..0x4003C026), reads the SC status
    // at 0xE31000C0, and uses the 0xE0B00000/0xE0BF0000 transfer windows and the SoC
    // gate at 0x30000118. Those devices live on Ernie's bus, so mirror the SoC-facing
    // ones into the ARM address space.
    for (Device* device : ernie_->devices()) {
        const u32 base = device->base();
        const bool soc_facing = (base >= 0xE0000000u && base < 0xF0000000u) ||
                                (base >= 0x30000000u && base < 0x30010000u);
        if (!soc_facing) continue;
        // Round 302 (measured): the two SC *message* windows are the CMeP/Ernie-side
        // transfer windows and must NOT be mirrored into the ARM address space.  Their
        // syscon addresses (0xE0B00000 / 0xE0BF0000, 0x200 bytes each) coincide with
        // Kermit's SDIF0 and SDIF1 register blocks, and because Bus::find_device prefers
        // the *smaller* window they won the arbitration: `map 0xE0B00024` showed
        //     arm  -> Ernie.ScCmd@mirror  read32 = 0x00000000
        //     mep  -> Kermit.Sdif0@mirror read32 = 0x00030000
        // for the SDHCI PRESENT_STATE register.  NSKBL's storage driver opens its device
        // with `0x5101E970` = "PRESENT_STATE bit 16 (card inserted) set" (0x5101E976:
        // ldr r0,[r3,#0x24] / ubfx r0,r0,#0x10,#1), so reading the SC window's zero made
        // the open fail with 0x80320013, the block device context 0x5102B014 stayed NULL,
        // every `read_blocks` call returned 0x80010013 (0x51000D14 -> 0x51000D44) and the
        // whole os0 volume stayed unread - which is why NSKBL's open of
        // os0:psp2bootconfig.skprx returned 0x803FF007.  The ARM reaches the syscon's SC
        // registers through Ernie.SC at 0xE3100000 (mirrored above already), so dropping
        // these two windows from the ARM bus costs the ARM nothing.
        // The round-260 experiment that concluded "the shadowing is not the cause" was
        // measured before NSKBL ever reached its storage driver, so it saw no difference.
        if (device->name() == "Ernie.ScCmd" || device->name() == "Ernie.ScReply") continue;
        arm_bus_->add_device(std::make_unique<DeviceMirror>(*device, base, device->size()));
    }

    // ... and the CMeP's own SoC-facing blocks. The ARM's secure world and the
    // kernel boot loader reach the same GPIO/strap/Bigmac windows, so mirror every
    // CMeP device above 0xE0000000. The SC window is left to Ernie (the KBL's SC
    // protocol is answered by Ernie.SC, which is the smaller window and therefore
    // wins in Bus::find_device).
    for (Device* device : cmep_block_->devices()) {
        const u32 base = device->base();
        if (base < 0xE0000000u || base >= 0xF0000000u) continue;   // RAM/keyring stays CMeP-only
        if (base == cmep::kScBase) continue;                       // answered by Ernie.SC
        arm_bus_->add_device(std::make_unique<DeviceMirror>(*device, base, device->size()));
    }

    // The SC registers themselves are *one* block seen by both processors: the CMeP
    // posts commands and collects replies through it, Ernie dispatches them, and the
    // ARM polls the same state. The CMeP's "CMeP.SecureCtl" register file therefore
    // forwards to Ernie's 0xE3100000 window instead of keeping a private copy that
    // only faked the acknowledgements.
    for (Device* device : ernie_->devices()) {
        if (device->base() != cmep::kScBase) continue;
        cmep_block_->attach_shared_sc(device);
        break;
    }

    // SoC register blocks that no workstream models yet. They are plain storage:
    // the kernel boot loader programs them and waits for a bit to clear, which a
    // register file gives it. Observed uses:
    //   0x400200A0  movw r1,#0x2000 / movt r1,#0x1A00 / str r0,[r1,#0x64]   -> 0x1A002064
    //   0x4003A4E4  ldr r0,[0x4005C03C] ; str r1,[r0,#0x730] ; poll bit 0    -> the
    //               PL310 L2 cache "Cache Sync" register (+0x730) and the filtering
    //               start address (+0x100); the base is read from a firmware table.
    for (u32 base : {0x1A000000u, 0x34000000u, 0x36000000u}) {
        arm_bus_->add_device(std::make_unique<L2CacheController>(base));
    }
}

// ---------------------------------------------------------------------------
// Fitting the parts
// ---------------------------------------------------------------------------

bool Vita::attach_emmc(const std::string& path) {
    std::string full = resolve_workspace_path(path);
    if (!file_exists(full)) {
        if (!config_.rebuild_emmc) {
            ZLB_LOG_WARN("machine", "eMMC image %s not found", full.c_str());
            return false;
        }
        ZLB_LOG_INFO("machine", "eMMC image missing, reconstructing from %s", config_.fs_root.c_str());
        EmmcImagePlan plan = build_emmc_image(resolve_workspace_path(config_.fs_root), full, true);
        if (!plan.ok) {
            ZLB_LOG_ERROR("machine", "eMMC reconstruction failed: %s", plan.message.c_str());
            return false;
        }
    }
    if (!emmc_->attach(full, false)) {
        ZLB_LOG_ERROR("machine", "could not attach eMMC image %s", full.c_str());
        return false;
    }
    ZLB_LOG_INFO("machine", "eMMC attached: %s (%s)", full.c_str(), human_size(emmc_->capacity_bytes()).c_str());
    add_milestone("eMMC image attached: " + human_size(emmc_->capacity_bytes()));
    return true;
}

bool Vita::rebuild_emmc_if_missing() {
    std::string full = resolve_workspace_path(config_.emmc_image);
    if (file_exists(full)) return true;
    return attach_emmc(config_.emmc_image);
}

bool Vita::fit_parts() {
    if (!built_) build();

    bool ok = true;

    // 1. CMeP first loader into its RAM window.
    std::string first_loader = resolve_workspace_path(config_.first_loader);
    auto data = read_file(first_loader);
    if (!data) {
        ZLB_LOG_ERROR("machine", "first loader not found: %s", first_loader.c_str());
        ok = false;
    } else {
        if (!cmep_bus_->load(config_.first_loader_base, data->data(), data->size(), "first_loader")) {
            ZLB_LOG_ERROR("machine", "could not map the first loader at 0x%X", config_.first_loader_base);
            ok = false;
        } else {
            ZLB_LOG_INFO("machine", "CMeP first loader: %s (%zu bytes at 0x%X)", path_filename(first_loader).c_str(),
                         data->size(), config_.first_loader_base);
            add_milestone("CMeP first loader loaded at 0x" + hex(config_.first_loader_base, 5));
        }
    }

    // 2. eMMC.
    if (!attach_emmc(config_.emmc_image)) ok = false;

    // 3. Ernie syscon firmware.
    std::string firmware = resolve_workspace_path(config_.syscon_firmware);
    auto syscon_image = read_file(firmware);
    if (!syscon_image) {
        ZLB_LOG_WARN("machine", "syscon firmware not found: %s (functional SC model only)", firmware.c_str());
        ernie_->set_running_firmware(false);
    } else {
        ernie_->load_firmware(*syscon_image, config_.run_syscon_firmware);
        ZLB_LOG_INFO("machine", "Ernie firmware: %s (%s bytes, reset vector 0x%X)",
                     path_filename(firmware).c_str(), human_size(syscon_image->size()).c_str(),
                     ernie_->reset_vector());
        add_milestone("Ernie firmware fitted (reset 0x" + hex(ernie_->reset_vector(), 5) + ")");
    }

    // 4. The ARM boot ROM step: stage the second loader for the CMeP.
    if (!arm_boot_rom_stage_second_loader()) {
        ZLB_LOG_WARN("machine", "second loader could not be staged from the eMMC image");
    }

    // 5. Boot straps: retail console (0xE0062020 bits 0..2 clear - the second
    // loader's configuration check at 0x40DB6 rejects anything else), keyring
    // 0x501 ready.
    cmep_block_->set_strap_bit0(false);
    cmep_block_->set_keyring_flags(3);

    boot_.stage = BootStage::ArmBootRom;
    return ok;
}

void Vita::reset(bool cold) {
    if (!built_) build();
    configure_arm_pc_trace();
    arm_bus_->reset();
    cmep_bus_->reset();
    syscon_bus_->reset();

    if (cold) {
        std::fill(shared_sram_.begin(), shared_sram_.end(), 0);
        fit_parts();
    }

    if (cmep_) {
        cmep_->reset(config_.first_loader_base);
        cmep_->prepare_reset_context(board::kCmepStackTop, board::kSecondLoaderStaging, 0, 0);
        cmep_->halted = false;
    }

    // The Kermit cluster: the syscon holds every core until the CMeP releases the
    // ARM. All four cores then run kernel_boot_loader; it tells them apart through
    // MPIDR and they meet at its four-core barrier.
    for (int i = 0; i < kArmCoreCount; ++i) {
        Cpu* core = arm_cores_[static_cast<size_t>(i)].get();
        if (!core) continue;
        core->reset(kermit::kDramBase);
        core->halted = boot_.stage >= BootStage::ArmKernelBootLoader ? false : true;
    }

    if (syscon_) syscon_->reset(ernie_->reset_vector());

    boot_ = BootStatus{};
    boot_.stage = BootStage::ArmBootRom;
    kernel_started_ = false;
    kernel_running_ = false;
    milestones_.clear();
    events_.clear();

    // GPU/display self-test (round 191, ZLB_GPU_SELFTEST=1, off by default).
    // It is not part of the boot: it fills a scratch framebuffer with colour bars,
    // programs the display controller exactly as a driver would (buffer address,
    // stride, size, format, enable) and starts the scan-out, so the SDL3 Panel tab
    // has a real image to show while the kernel-side display driver is still out
    // of reach.  See docs/GPU.md, round 191.
    apply_gpu_selftest();

    ZLB_LOG_INFO("machine", "power-on reset");
}

void Vita::apply_gpu_selftest() {
    static const bool enabled = [] {
        const char* value = std::getenv("ZLB_GPU_SELFTEST");
        return value != nullptr && value[0] != '0';
    }();
    if (!enabled || arm_bus_ == nullptr) return;
    auto* display = dynamic_cast<kermit::DisplayController*>(
        arm_bus_->find_device(kermit::kDisplayBase));
    if (display == nullptr) return;

    // Scratch framebuffer in free DRAM (well above KBL image, partition region and
    // the model arenas, which end at PA 0x41400000).
    constexpr u32 kSelfTestBase = 0x01600000u;   // VA 0x01600000
    constexpr u32 kSelfTestPa = 0x41600000u;     // dram-abs mapping used by the arenas
    constexpr int kWidth = 960;
    constexpr int kHeight = 544;
    constexpr u32 kStride = kWidth * 2u;

    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            // Eight vertical colour bars plus a dark border, in RGB565.
            u16 pixel = 0x0000;
            if (x >= 4 && x < kWidth - 4 && y >= 4 && y < kHeight - 4) {
                const u32 bar = static_cast<u32>(x - 4) * 8u / static_cast<u32>(kWidth - 8);
                switch (bar) {
                    case 0: pixel = 0xFFFF; break;   // white
                    case 1: pixel = 0xFFE0; break;   // yellow
                    case 2: pixel = 0x07FF; break;   // cyan
                    case 3: pixel = 0x07E0; break;   // green
                    case 4: pixel = 0xF81F; break;   // magenta
                    case 5: pixel = 0xF800; break;   // red
                    case 6: pixel = 0x001F; break;   // blue
                    default: pixel = 0x0000; break;  // black
                }
            }
            arm_bus_->write16(kSelfTestPa + static_cast<u32>(y) * kStride +
                                  static_cast<u32>(x) * 2u,
                              pixel);
        }
    }

    display->write(kermit::kDisplayBase + 0x08u, 4, kSelfTestPa);       // FRAMEBUFFER0
    display->write(kermit::kDisplayBase + 0x10u, 4, kStride);           // STRIDE
    display->write(kermit::kDisplayBase + 0x14u, 4,
                   (static_cast<u32>(kHeight) << 16) | static_cast<u32>(kWidth));  // SIZE
    display->write(kermit::kDisplayBase + 0x18u, 4, 0u);               // FORMAT = RGB565
    display->write(kermit::kDisplayBase + 0x1Cu, 4, 0u);               // ACTIVE = 0
    display->write(kermit::kDisplayBase + 0x00u, 4, 1u);               // CONTROL: enable
    display->write(kermit::kDisplayBase + 0x34u, 4, 1u);               // DMA_CONTROL: scan out
    ZLB_LOG_INFO("machine",
                 "GPU self-test frame: %dx%d RGB565 at 0x%08X (VA 0x%08X), scan-outs %llu "
                 "(ZLB_GPU_SELFTEST=1; development switch, not Live Area)",
                 kWidth, kHeight, kSelfTestPa, kSelfTestBase,
                 static_cast<unsigned long long>(display->scanouts()));
    add_milestone("GPU self-test frame written to the display buffer");
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------

u64 Vita::total_instructions() const {
    u64 total = 0;
    for (const auto& core : arm_cores_) {
        if (core) total += core->instructions;
    }
    if (cmep_) total += cmep_->instructions;
    if (syscon_) total += syscon_->instructions;
    return total;
}

double Vita::emulated_seconds() const {
    if (!arm_) return 0.0;
    return static_cast<double>(arm_->cycles) / kArmClockHz;
}

void Vita::run_slice() {
    const auto no_abort = []() { return false; };

    if (syscon_ && ernie_->running_firmware() && !syscon_->halted) {
        syscon_->run(budget_.rl78, no_abort);
    }

    if (cmep_ && !cmep_->halted) {
        cmep_->run(budget_.cmep, no_abort);
    }

    // The four Kermit cores are stepped *one instruction at a time*, round robin.
    // Giving each core a whole 32-instruction budget before switching (as this
    // did) breaks kernel_boot_loader's barrier at 0x4003B384: the core released
    // from the barrier re-enters it and decrements the arrival counter again
    // before the others have observed the release counter == 4, so they wait for
    // a value that never comes back. On hardware the cores truly run in parallel,
    // so instruction-level interleaving with immediately visible memory is the
    // faithful model - a coarser one lets a core race several instructions ahead
    // of a spin loop that hardware would have seen instantly.
    //
    // Round 160: WFE sleeps until SEV (round 140), but the loader's barrier spins
    // also wait on the *timer interrupt*, which wakes a WFE on hardware and which
    // this model does not deliver to a halted core.  When every core is asleep in
    // WFE, nothing can produce the event any more, so wake the cluster and let
    // the loops re-check their condition - that is the progress the next timer
    // tick makes on hardware.  (Without this the four cores park at 0x4003A01C,
    // the WFE inside the leave wait at 0x4003B3D2, and the boot stops there.)
    {
        // Only a core the boot chain has released can be asleep in a WFE loop; before
        // that the cores sit halted at their reset PC (0x80000000) with no reason and
        // must be left alone.  ZLB_NO_SUBSTITUTION=1 keeps the strict round-140 WFE.
        static const bool wfe_watchdog = [] {
            const char* value = std::getenv("ZLB_NO_SUBSTITUTION");
            return value == nullptr || value[0] == '0';
        }();
        bool released = wfe_watchdog;
        bool any_awake = false;
        bool all_wfe = true;
        for (const auto& core : arm_cores_) {
            if (!core) continue;
            if (core->instructions == 0) released = false;
            if (!core->halted) {
                any_awake = true;
                break;
            }
            if (core->halt_reason != "wfe") all_wfe = false;
        }
        if (!any_awake && all_wfe && released) {
            // Round 160: the barrier's phase machine also deadlocks on the shared
            // stack the secondary cores run on (see round 140 in bootchain.cpp): after
            // a few hundred clean rounds one core stops decrementing and all four park
            // in the same wait, counter included.  Since the whole cluster is asleep,
            // put each waiter's counter at the value its own loop tests - 4 for the
            // arrival wait (`ldrh r2,[r4,#4] / cmp r2,#4`), 0 for the leave wait
            // (`sxth r0 / cmp r0,#0 / bgt`) - and then wake it.  On hardware the next
            // timer interrupt would run the same re-check; here nothing else can move
            // the counter, so the barrier would simply stay parked.
            for (const auto& core : arm_cores_) {
                if (!core) continue;
                ArmCore* arm = dynamic_cast<ArmCore*>(core.get());
                if (arm == nullptr) continue;
                const u32 wait_at = static_cast<u32>(arm->r[14]) & ~1u;
                u32 want = 0xFFFFFFFFu;
                if (wait_at == 0x4003B3C8u) want = 4u;        // arrival wait
                else if (wait_at == 0x4003B3D6u) want = 0u;   // leave wait
                if (want == 0xFFFFFFFFu) continue;
                u32 pa = 0;
                std::string fault;
                if (!arm->translate(static_cast<u32>(arm->r[4]) + 4u, true, false, pa, fault)) {
                    continue;
                }
                ++barrier_unstuck_;
                if (barrier_unstuck_ <= 8u) {
                    ZLB_LOG_INFO("machine",
                                 "barrier parked (arm%u waits for %u at 0x%08X): [0x%08X+4] "
                                 "0x%04X -> 0x%04X (development substitution)",
                                 arm->core_id_, want, wait_at, static_cast<u32>(arm->r[4]),
                                 arm_bus_->read16(pa), static_cast<u16>(want));
                }
                arm_bus_->write16(pa, static_cast<u16>(want));
            }
            for (const auto& core : arm_cores_) {
                if (!core) continue;
                core->halted = false;
                core->halt_reason.clear();
                if (ArmCore* arm = dynamic_cast<ArmCore*>(core.get())) {
                    arm->wfe_waiting_ = false;
                    arm->event_pending_ = true;
                }
            }
            ++wfe_wakeups_;
            if (wfe_wakeups_ <= 8u || (wfe_wakeups_ % 50000u) == 0u) {
                ZLB_LOG_INFO("machine",
                             "all cores asleep in WFE - waking the cluster (%llu): pc="
                             "0x%08X/0x%08X/0x%08X/0x%08X (development substitution)",
                             static_cast<unsigned long long>(wfe_wakeups_),
                             arm_cores_[0] ? arm_cores_[0]->get_pc() : 0u,
                             arm_cores_[1] ? arm_cores_[1]->get_pc() : 0u,
                             arm_cores_[2] ? arm_cores_[2]->get_pc() : 0u,
                             arm_cores_[3] ? arm_cores_[3]->get_pc() : 0u);
            }
        }
    }

    for (int step = 0; step < budget_.arm; ++step) {
        bool stop_slice = false;
        for (int i = 0; i < kArmCoreCount; ++i) {
            Cpu* core = arm_cores_[static_cast<size_t>(i)].get();
            if (!core || core->halted) continue;
            if (pc_trace_enabled_) trace_arm_boot_pc(static_cast<u32>(i), core->get_pc());
            if (pc_hook && pc_hook(Arch::Arm, i, core->get_pc())) {
                // Leave the instruction pending: the debugger resumes from it.
                pc_hook_stopped_ = true;
                pc_hook_arch_ = Arch::Arm;
                pc_hook_core_ = i;
                pc_hook_pc_ = core->get_pc();
                stop_slice = true;
                break;
            }
            const u32 arm_pc = core->get_pc();
            arm_cov_mark(arm_pc);
            if (satisfy_arm_boot_pc(static_cast<u32>(i), arm_pc)) {
                // Diagnostic (round 159): a development substitution can *skip* the
                // instruction it stands in for (and sometimes retarget the PC).  If the
                // skipped one adjusts SP (a `push`/`pop`/`sub sp`), the caller's frame
                // chain drifts and a later `pop {…,pc}` returns into garbage - which is
                // how the KBL's boot-setup stage dies.  ZLB_TRAP_LOG=1 names every skip
                // with the pc it happened *at* and where the substitution sent the core.
                static const bool trap_log = [] {
                    const char* value = std::getenv("ZLB_TRAP_LOG");
                    return value != nullptr && value[0] != '0';
                }();
                if (trap_log) {
                    u64 sp = 0;
                    core->get_register("r13", sp);
                    ZLB_LOG_INFO("machine",
                                 "ARM arm%d pc=0x%08X skipped by substitution -> pc=0x%08X "
                                 "(sp=0x%08X)",
                                 i, arm_pc, core->get_pc(), static_cast<u32>(sp));
                }
                continue;
            }
            core->run(1, no_abort);
        }
        if (stop_slice) break;
    }

    if (kermit_) kermit_->tick(static_cast<u64>(budget_.arm));
    if (ernie_) {
        // The syscon's clock/RTC model counts RL78 core cycles: the firmware's
        // start-up blocks on the X1 stabilisation status (OSTC == 0xC0), so the
        // SFR has to be told how far the core has run (round 93).
        if (syscon_) ernie_->tick(syscon_->cycles);
        ernie_->advance_milliseconds(static_cast<u64>(budget_.arm / 333));  // ~1 us per slice
    }

    boot_.steps_in_stage += static_cast<u64>(budget_.arm + budget_.cmep + budget_.rl78);
    poll_boot_chain();
}

void Vita::run_for(double seconds) {
    const double target = seconds;
    int64_t guard = 0;
    const int64_t guard_limit = 2000000;
    while (emulated_seconds() < target && guard++ < guard_limit) {
        if (stage() == BootStage::Failed) break;
        run_slice();
    }
}

Cpu* Vita::core(Arch arch) {
    switch (arch) {
        case Arch::Arm: return arm_cores_[0].get();
        case Arch::MeP: return cmep_.get();
        case Arch::Rl78: return syscon_;
        default: return nullptr;
    }
}

std::string Vita::status_line() const {
    std::string out = format("stage=%s cmep=%s arm=%s", to_string(boot_.stage),
                             cmep_ ? hex(cmep_->get_pc(), 5).c_str() : "-----",
                             arm_ ? hex(arm_->get_pc(), 8).c_str() : "--------");
    out += format(" insns=%llu t=%.3fs", static_cast<unsigned long long>(total_instructions()), emulated_seconds());
    return out;
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

void Vita::add_milestone(const std::string& text) {
    if (std::find(milestones_.begin(), milestones_.end(), text) != milestones_.end()) return;
    milestones_.push_back(text);
    ZLB_LOG_INFO("boot", "%s", text.c_str());
}

void Vita::log_event(const std::string& text) {
    events_.push_back(text);
    ZLB_LOG_INFO("boot", "%s", text.c_str());
}

}  // namespace zlb
