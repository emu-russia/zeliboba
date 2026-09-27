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
#include "hw/soc.h"
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
    cmep_bus_->add_ram_alias("cmep_dram", kermit::kScuBase, kermit::kScuSize, dram_.data(),
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
    // modules are mapped through 0x80000000 later on.
    arm_bus_->add_ram_alias("arm_priv", kermit::kScuBase, kermit::kScuSize, dram_.data(),
                            "physical DRAM window (kernel boot loader and kernel image)");
    arm_bus_->add_ram("arm_dram", 0x04000000, kermit::kDramBase, "main DRAM (64 MiB module window)");
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
    // of its work (see Vita::cmep_pc_hook).
    if (MePCore* mep = dynamic_cast<MePCore*>(cmep_.get())) {
        mep->pc_hook = [this](u32 pc) { return cmep_pc_hook(pc); };
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
        }
    }
    arm_ = arm_cores_[0].get();

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
    ZLB_LOG_INFO("machine", "power-on reset");
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
            if (satisfy_arm_boot_pc(static_cast<u32>(i), core->get_pc())) continue;
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
