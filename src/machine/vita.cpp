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
/// Per-core rings of the most recent ARM PCs, used to show the path that produced a
/// module-start failure (see the MODULEMGR_NO_LIB dump below). They are per core
/// because the four cores interleave - a single ring made the path look scrambled
/// (a delay loop from core 2 appeared between core 0's instructions).
/// Deduplicated set of the NSKBL code addresses executed since the current module
/// start began. A ring was not enough: the loader's cache-flush loop at 0x510143xx
/// fills it and evicts the history that matters. Reset on each module start and
/// dumped when the start fails.
constexpr unsigned kPcSetSlots = 4096;
u32 g_pc_set[kPcSetSlots];
unsigned char g_pc_set_used[kPcSetSlots];
u32 g_pc_set_count = 0;

/// Ordered ring of NSKBL addresses, excluding the cache-flush table at
/// 0x510143xx-0x510144xx and the start helper at 0x510194xx-0x510196xx, which
/// otherwise flood it. Dumped in order on a failed start so the branch that
/// produced the error is the tail of the list.
/// Ordered, per-core ring of every executed address during the current module start,
/// excluding only the loops known to flood it: the loader's cache-flush table
/// (0x510143xx-0x510144xx), the start helper (0x510194xx-0x510196xx) and the kernel's
/// delay loop (0x4F9DB8-0x4F9DD0). Widened from NSKBL-only because the loader calls
/// shared code outside 0x510xxxxx, which is where the failing branch may live.
constexpr unsigned kNskblRingSize = 4096;
constexpr unsigned kNskblRingCores = 4;
u32 g_nskbl_ring[kNskblRingCores][kNskblRingSize];
u32 g_nskbl_pos[kNskblRingCores] = {0, 0, 0, 0};

void nskbl_ring_add(unsigned core, u32 pc) {
    if (core >= kNskblRingCores) return;
    if (pc >= 0x51014300u && pc < 0x51014500u) return;
    if (pc >= 0x51019400u && pc < 0x51019600u) return;
    if (pc >= 0x004F9DB8u && pc < 0x004F9DD0u) return;
    g_nskbl_ring[core][(g_nskbl_pos[core]++) & (kNskblRingSize - 1u)] = pc;
}

void pc_set_reset() {
    std::memset(g_pc_set_used, 0, sizeof(g_pc_set_used));
    g_pc_set_count = 0;
}

void pc_set_add(u32 pc) {
    u32 slot = (pc * 2654435761u) & (kPcSetSlots - 1u);
    for (unsigned probe = 0; probe < kPcSetSlots; ++probe) {
        const u32 index = (slot + probe) & (kPcSetSlots - 1u);
        if (g_pc_set_used[index] == 0u) {
            g_pc_set_used[index] = 1u;
            g_pc_set[index] = pc;
            ++g_pc_set_count;
            return;
        }
        if (g_pc_set[index] == pc) return;
    }
}

constexpr unsigned kArmTraceCores = 4;
constexpr unsigned kArmTraceSize = 512;
u32 g_arm_trace_ring[kArmTraceCores][kArmTraceSize];
constexpr unsigned kUidRingSize = 64;
u32 g_uid_ring_site[kUidRingSize];
u32 g_uid_ring_value[kUidRingSize];
u32 g_uid_ring_result[kUidRingSize];
u32 g_uid_ring_pos = 0;
void uid_ring_result(u32 pos, u32 value) { g_uid_ring_result[pos & (kUidRingSize - 1u)] = value; }
void uid_ring_add(u32 site, u32 uid) {
    const u32 slot = g_uid_ring_pos++ & (kUidRingSize - 1u);
    g_uid_ring_site[slot] = site;
    g_uid_ring_value[slot] = uid;
}
u32 g_arm_trace_pos[kArmTraceCores] = {0, 0, 0, 0};
}  // namespace

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
        : RegisterFile(format("Kermit.L2CC@%08X", base), base, 0x1000) {
        define(base + 0x100, "L2CC_CONTROL", 0);
        define(base + 0x104, "L2CC_AUX_CONTROL", 0);
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
        const u32 offset = (address - base()) & ~3u;
        if (offset == 0x730u || offset == 0x77Cu || offset == 0x7FCu) {
            // These maintenance commands complete synchronously in the cache
            // model. Their way/busy bits must clear even though the debugger
            // exposes them as named registers.
            RegisterFile::write(base() + offset, 4, 0);
            return;
        }
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
    if (config_.emmc_image.empty()) config_.emmc_image = path_join(ZLB_ROOT_DIR, "build/emmc.img");
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
    // ZLB_EXCL_LOG=1 traces every exclusive-monitor reservation change; the KBL's
    // spin locks are the reason the secure boot loader stalls without the barrier
    // substitution (docs/KBL.md 7.1.27).
    if (const char* excl = std::getenv("ZLB_EXCL_LOG"); excl != nullptr && excl[0] != '0') {
        arm_bus_->exclusive_trace = true;
    }
    // ZLB_ARM_PC_LOG=<file>: per-instruction PC trace of the ARM cluster, for the
    // two-run diff that locates a divergence (docs/KBL.md 7.1.29).
    if (const char* path = std::getenv("ZLB_ARM_PC_LOG");
        path != nullptr && path[0] != '\0') {
        if (std::FILE* file = std::fopen(path, "wb")) {
            arm_pc_log_ = {file, &std::fclose};
            arm_pc_log_enabled_ = true;
            arm_pc_log_limit_ = 400000u;
            if (const char* limit = std::getenv("ZLB_ARM_PC_LIMIT"); limit != nullptr) {
                arm_pc_log_limit_ = static_cast<u32>(std::strtoul(limit, nullptr, 10));
            }
            if (const char* regs = std::getenv("ZLB_ARM_PC_REGS"); regs != nullptr && regs[0] != '0') {
                arm_pc_log_regs_ = true;
            }
            ZLB_LOG_INFO("machine", "ARM pc log -> %s (limit %u instructions per core)", path,
                         arm_pc_log_limit_);
        }
    }
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
    // CDRAM is a separate memory device, shared by ARM and display/GPU DMA
    // through this physical bus. Controller commands at E8200000 establish
    // its native enable state; electrical power gating is not modelled here.
    // Keep the existing board reset policy (zero RAM), without a guessed
    // CMeP alias or an alias of main DRAM / the logo's SRAM work buffer.
    arm_bus_->add_ram("arm_cdram", board::kCdramSize, board::kCdramBase,
                      "CDRAM (128 MiB independent graphics memory)");
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
            // Secure-runtime state machine: 0x801D54 is the setter for the CMeP's
            // state word at 0x8076AC (gp-32748). Logging the written value together
            // with the caller's return address shows who drives the transitions and
            // where the 3 -> 4 -> 5 -> 8 retry cycle comes from.
            if (pc == 0x801D54u) {
                static u32 state_logged = 0;
                if (state_logged < 24u) {
                    ++state_logged;
                    if (MePCore* st = dynamic_cast<MePCore*>(cmep_.get())) {
                        ZLB_LOG_INFO("machine", "secure: state <- 0x%X (caller lp=0x%08X)",
                                     st->r[1], st->lp);
                    }
                }
            }
            // 0x800C2E is the command dispatcher: it loads 0x806FD8 + $3*4 and jumps
            // through it, so $3 is the command id. Logging it shows which commands the
            // secure kernel processes and whether the 3->4->5->8 cycle repeats one.
            // kprx_auth_sm.self is staged from SLB2 into DRAM at 0x40000500 and the
            // secure kernel copies it into its own SRAM at 0x0080B000 (verified: the
            // byte writes there match the decrypted module exactly). This probe says
            // whether the secure core ever *executes* it, i.e. whether the module's
            // entry is reached.
            if (pc >= 0x0080B000u && pc < 0x00810000u) {
                static u64 module_hits = 0;
                static u32 module_last = 0;
                static u32 module_logged = 0;
                ++module_hits;
                if (module_logged < 8u && (module_hits <= 4u || (module_hits % 2000000u) == 0u)) {
                    ++module_logged;
                    ZLB_LOG_INFO("machine", "secure: kprx_auth_sm hit=%llu pc=0x%08X last=0x%08X",
                                 static_cast<unsigned long long>(module_hits), pc, module_last);
                }
                module_last = pc;
            }
            // 0x800BFA is the step machine's entry; its arg4 selects the step
            // (1..8, >8 goes to the error path). Logging the entry shows whether the
            // caller repeats the same step, which is what the 3->4->5->8 cycle looks
            // like from the outside.
            if (pc == 0x800BFAu) {
                static u32 step_logged = 0;
                if (step_logged < 40u) {
                    ++step_logged;
                    if (MePCore* st = dynamic_cast<MePCore*>(cmep_.get())) {
                        ZLB_LOG_INFO("machine", "secure: step entry arg3=0x%X arg4=0x%X r1=0x%X r2=0x%X lp=0x%08X",
                                     st->r[3], st->r[4], st->r[1], st->r[2], st->lp);
                    }
                }
            }
            if (pc == 0x800C2Eu) {
                static u32 cmd_logged = 0;
                if (cmd_logged < 40u) {
                    ++cmd_logged;
                    if (MePCore* st = dynamic_cast<MePCore*>(cmep_.get())) {
                        ZLB_LOG_INFO("machine", "secure: dispatch cmd=%u (0x%X) r1=0x%X r2=0x%X lp=0x%08X",
                                     st->r[3], st->r[3], st->r[1], st->r[2], st->lp);
                    }
                }
            }
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
            // An asserted IRQ line takes the core out of WFE even when the interrupt
            // is masked (see ArmCore::set_irq), which is how the timer tick releases
            // the boot loader's barrier waits.  The tick itself is driven from
            // run_slice via KermitBlock::raise_irq, so this only clears the sleep.
            arm->irq_hook = [this, i]() {
                Cpu* cpu = arm_cores_[static_cast<size_t>(i)].get();
                ArmCore* self = dynamic_cast<ArmCore*>(cpu);
                if (self == nullptr) return;
                if (self->wfe_waiting_) {
                    self->wfe_waiting_ = false;
                    self->event_pending_ = true;
                    self->halted = false;
                    self->halt_reason.clear();
                    ++wfe_irq_wakeups_;
                }
            };
        }
    }
    arm_ = arm_cores_[0].get();

    // Each A9 core has its own GIC CPU interface. Attach the whole cluster so
    // the firmware's target bytes select the actual receiving core (native
    // Smsched routes mailbox interrupts 200..203 to core 3).
    for (u32 core_id = 0; core_id < kArmCoreCount; ++core_id) {
        if (Cpu* core = arm_cores_[core_id].get()) {
            kermit_set_cpu(*arm_bus_, core, core_id);
        }
    }

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
    // The ARM sees the shared mailbox state through its own sender/ack port.
    // A generic mirror would give its writes the opposite CMeP-side semantics.
    cmep_block_->install_arm_mailbox(*arm_bus_);
    // Secure_kernel handles command channel 0 on INTC source 8 and registers
    // the auxiliary handlers on sources 9..11 (0x8023E0..0x8023F4). Native ARM
    // Smsched registers the reverse direction on GIC interrupts 200..203.
    cmep_block_->set_mailbox_irq_callbacks(
        [this](unsigned channel, bool asserted) {
            // The secure kernel's step machine polls software flags that its
            // interrupt handlers are supposed to set; a write trap on those flags
            // shows nothing but the reset ever writes them. This log says whether
            // the mailbox even raises the line that leads to those handlers.
            static u32 irq_logged = 0;
            if (asserted && irq_logged < 40u) {
                ++irq_logged;
                if (auto* mep = dynamic_cast<MePCore*>(cmep_.get())) {
                    ZLB_LOG_INFO("machine",
                                 "secure: mailbox irq to CMeP channel=%u isr=0x%08X imr=0x%08X psw=0x%X "
                                 "seen=%llu taken=%llu last=%d",
                                 channel, mep->interrupt_flag_register(), mep->interrupt_mask_register(),
                                 mep->psw, static_cast<unsigned long long>(mep->irq_sources_seen),
                                 static_cast<unsigned long long>(mep->irq_sources_taken), mep->irq_last_source);
                }
            }
            if (auto* mep = dynamic_cast<MePCore*>(cmep_.get())) {
                mep->set_irq_level(8u + channel, asserted);
            }
        },
        [this](unsigned channel, bool asserted) {
            kermit_->raise_irq(200u + channel, asserted);
        });

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
        if (base == cmep::kMailboxBase) continue;                 // explicit ARM endpoint above
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
    // PERIPHBASE 0x1A000000 is the SCU/GIC region. PL310 starts at
    // 0x1A002000, also used directly by KBL at 0x400200A0. A broad cache
    // window at PERIPHBASE intercepted the native GIC CPU-interface writes.
    for (u32 base : {0x1A002000u, 0x34000000u, 0x36000000u}) {
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
    apply_os0_elf_form();
    return true;
}

bool Vita::rebuild_emmc_if_missing() {
    std::string full = resolve_workspace_path(config_.emmc_image);
    if (file_exists(full)) return true;
    return attach_emmc(config_.emmc_image);
}

// Historical opt-in experiment (round 396): replace os0 module bytes with ELF.
// The old "SCE rejected" diagnosis was caused by the emulator's Thumb IT flag
// bug. With correct flags the genuine validator returns 2 for SCE and 1 for ELF;
// fs/os0/*.skprx are encrypted/compressed SELF containers. This experiment is
// not a firmware repair and must remain disabled for an ordinary boot. It also
// replaces psp2config with psp2bootconfig below, so it cannot validate module
// loading or original image integrity. The fixed-cluster writes use the
// layout measured from the volume: psp2bootconfig.skprx ("PSP2BO~1") sits at cluster 903,
// i.e. LBA 72816, in a volume whose data area starts at LBA 65608 with 8 sectors/cluster.
// Gated by ZLB_OS0_ELF=1; by default the card is untouched.
void Vita::apply_os0_elf_form() {
    static const bool enabled = [] {
        const char* on = std::getenv("ZLB_OS0_ELF");
        return on != nullptr && on[0] != '0';
    }();
    if (!enabled || !emmc_) return;
    constexpr u64 kFileLba = 72816u;      // cluster 903 of the os0 volume
    constexpr u64 kFileBlocks = 11u;      // 5330 bytes in the directory entry (rounded up)
    const std::string elf_path =
        resolve_workspace_path("Vita_104_Firmware/Out/fs_dec/os0/psp2bootconfig.elf");
    auto elf = read_file(elf_path);
    if (!elf || elf->empty()) {
        ZLB_LOG_WARN("machine", "os0 ELF form not found: %s (ZLB_OS0_ELF=1 ignored)", elf_path.c_str());
        return;
    }
    std::vector<u8> blocks(static_cast<size_t>(kFileBlocks) * 512u, 0u);
    std::memcpy(blocks.data(), elf->data(), std::min<size_t>(elf->size(), blocks.size()));
    if (emmc_->write_blocks(EmmcPartition::User, kFileLba, static_cast<u32>(kFileBlocks), blocks.data())) {
        ZLB_LOG_INFO("machine",
                     "os0 ELF form written to the card: %s -> LBA %llu (%zu bytes in %llu blocks, "
                     "ZLB_OS0_ELF=1, development substitution)",
                     path_filename(elf_path).c_str(), static_cast<unsigned long long>(kFileLba),
                     elf->size(), static_cast<unsigned long long>(kFileBlocks));
        add_milestone("os0 ELF form written to the eMMC card (development substitution)");
    }
    // Experiment (round 397): the validator's very first failing call receives *psp2config*'s
    // data (its +0x18 is B0 09; psp2bootconfig's is 18 0A - measured with the 32-byte diagnostic
    // ZLB_NSKBL_VALIDATOR_LOG).  The workspace has no ELF form for psp2config.skprx (only
    // fs_dec/os0/psp2config.skprx.seg00/.seg01/.seg02), so to tell "format was the blocker"
    // apart from "content matters", serve the same ELF there as well and see whether the
    // loader's format check passes.
    constexpr u64 kSecondFileLba = 72832u;   // cluster 905 of the volume
    if (emmc_->write_blocks(EmmcPartition::User, kSecondFileLba, static_cast<u32>(kFileBlocks),
                            blocks.data())) {
        ZLB_LOG_INFO("machine",
                     "os0 ELF form also written for the second file (LBA %llu, experiment, "
                     "ZLB_OS0_ELF=1)",
                     static_cast<unsigned long long>(kSecondFileLba));
    }
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
    // Restore the boot-phase low SRAM backing before either cold or warm RAM
    // reset. Bus::reset() clears region.bytes(); leaving the secure-kernel alias
    // installed would clear private SRAM twice and retain stale boot storage.
    // Its cached page pointers must change before fitted loader bytes are loaded.
    if (auto* low_sram = cmep_bus_->region_at(board::kCmepRamBase, board::kCmepRamSize)) {
        low_sram->external = nullptr;
        cmep_bus_->rebuild_map();
    }
    arm_bus_->reset();
    cmep_bus_->reset();
    syscon_bus_->reset();
    secure_modules_staged_ = false;
    secure_kernel_active_ = false;
    cmep_service_pending_ = false;
    cmep_context_done_ = false;
    arm_wait_slices_ = 0;

    if (cold) {
        std::fill(shared_sram_.begin(), shared_sram_.end(), 0);
        fit_parts();
    }

    if (cmep_) {
        cmep_->reset(config_.first_loader_base);
        if (auto* mep = dynamic_cast<MePCore*>(cmep_.get())) {
            mep->set_boot_vector_base(board::kSecondLoaderStaging);
        }
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
        core->halted = true;
    }

    if (syscon_) syscon_->reset(ernie_->reset_vector());

    boot_ = BootStatus{};
    boot_.stage = BootStage::ArmBootRom;
    kernel_started_ = false;
    kernel_running_ = false;
    nskbl_seen_ = false;
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
    // The machine's clock comes from the Kermit block, which is ticked exactly
    // once per machine step/slice with the CPU cycles that step represents.  The
    // old implementation read arm0's `cycles`, which stops the moment arm0 parks
    // in WFE: `boot` then reported the same "emulated time" for the rest of the
    // run (and `run_for` could never reach its target).  Fall back to the core
    // only for a machine whose Kermit block has not been ticked yet.
    if (kermit_) {
        const u64 cycles = kermit_->total_cycles();
        if (cycles != 0u) return static_cast<double>(cycles) / kArmClockHz;
    }
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
    // Historical WFE experiments remain opt-in. Native SEV and interrupt delivery
    // now wake the cores; the full boot reaches bootconfig start with both the
    // artificial timer and the legacy counter/wake watchdog disabled.
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
            // Round 161: the honest fix for "the whole cluster is asleep and nothing
            // can wake it" is the periodic tick a core's timer would deliver: an
            // asserted interrupt line takes a core out of WFE, the core re-checks
            // its loop condition and goes back to sleep if it has to.  The model
            // does not reproduce whatever arms that timer (the guest programs none
            // of the MPCore timers and no other interrupt fires - docs/KBL.md
            // 7.1.8), so the driver raises the tick itself.  ZLB_KBL_WFE_TICK=0
            // keeps the strict WFE; ZLB_KBL_WFE_WATCHDOG=1 restores the old
            // substitution that wrote the barrier's counter directly, which is kept
            // only to compare the two states.
            static const bool tick_driver = [] {
                const char* value = std::getenv("ZLB_KBL_WFE_TICK");
                return value != nullptr && value[0] != '0';
            }();
            // Keep the old barrier-counter and unconditional cluster-wake engine
            // only for comparisons with historical runs. It must not manufacture
            // events during ordinary boot or a later guest display wait.
            static const bool legacy_watchdog = [] {
                const char* value = std::getenv("ZLB_KBL_WFE_WATCHDOG");
                return value != nullptr && value[0] != '0';
            }();
            u64 retired = 0;
            for (const auto& core : arm_cores_) {
                if (core) retired += core->instructions;
            }
            if (retired == last_arm_instructions_) {
                ++all_wfe_streak_;
            } else {
                all_wfe_streak_ = 0;
            }
            last_arm_instructions_ = retired;
            // One tick per `kTickSlices` slices with no progress at all: a shorter
            // period would inject interrupts into a healthy idle loop, a longer one
            // just costs slices.
            // One tick per `tick_slices` slices with no progress at all: a shorter
            // period wakes the cores so often that the barrier's interleaving breaks
            // (measured: 2402 ticks / 9608 wake-ups over 200k slices end at 0x49
            // instead of 0xA9, docs/KBL.md 7.1.12), so the period is a knob.
            static const u32 tick_slices = [] {
                const char* value = std::getenv("ZLB_KBL_WFE_TICK_SLICES");
                if (value == nullptr) return 64u;
                const u32 parsed = static_cast<u32>(std::strtoul(value, nullptr, 10));
                return parsed >= 8u ? parsed : 64u;
            }();
            const u32 kTickSlices = tick_slices;
            if (tick_driver && all_wfe_streak_ >= kTickSlices && kermit_ != nullptr) {
                all_wfe_streak_ = 0;
                ++wfe_ticks_;
                u32 woke = 0;
                for (const auto& core : arm_cores_) {
                    if (core == nullptr) continue;
                    if (!core->halted) continue;
                    core->halted = false;
                    core->halt_reason.clear();
                    if (ArmCore* arm = dynamic_cast<ArmCore*>(core.get())) {
                        arm->wfe_waiting_ = false;
                        arm->event_pending_ = true;
                    }
                    ++woke;
                }
                wfe_irq_wakeups_ += woke;
                // PPI 29: the private timer of this MPCore.  The line is raised and
                // lowered again on the next slice so the level-triggered distributor
                // does not keep it pending after the re-check.  The wake-up itself
                // does not depend on the distributor accepting the interrupt: on
                // hardware an *asserted* line takes a core out of WFE even when the
                // interrupt is masked or not enabled at the distributor, and the
                // model's Gic::refresh_line only asserts for an interrupt that would
                // actually be taken - so the tick also notifies the cores directly.
                kermit_->raise_irq(kermit::kIrqPpiPrivateTimer, true);
                for (const auto& core : arm_cores_) {
                    if (core == nullptr) continue;
                    if (ArmCore* arm = dynamic_cast<ArmCore*>(core.get())) {
                        if (arm->irq_hook) arm->irq_hook();
                    }
                }
                tick_pending_ = true;
                if (wfe_ticks_ <= 8u || (wfe_ticks_ % 1000u) == 0u) {
                    ZLB_LOG_INFO("machine",
                                 "timer tick %llu: cluster idle for %u slices, raising PPI %u "
                                 "(pc=0x%08X/0x%08X/0x%08X/0x%08X)",
                                 static_cast<unsigned long long>(wfe_ticks_), kTickSlices,
                                 kermit::kIrqPpiPrivateTimer,
                                 arm_cores_[0] ? arm_cores_[0]->get_pc() : 0u,
                                 arm_cores_[1] ? arm_cores_[1]->get_pc() : 0u,
                                 arm_cores_[2] ? arm_cores_[2]->get_pc() : 0u,
                                 arm_cores_[3] ? arm_cores_[3]->get_pc() : 0u);
                }
            }
            if (legacy_watchdog) {
                // ZLB_KBL_WFE_PATCH_LIMIT=<n> stops patching after n stalls, so the
                // question "does the barrier need every patch or only the first one"
                // becomes measurable (docs/KBL.md 7.1.13).
                static const u64 patch_limit = [] {
                    const char* value = std::getenv("ZLB_KBL_WFE_PATCH_LIMIT");
                    if (value == nullptr) return ~0ull;
                    return static_cast<u64>(std::strtoull(value, nullptr, 10));
                }();
                if (barrier_unstuck_ >= patch_limit) return;
                // The old substitution: put each waiter's counter at the value its own
                // loop tests - 4 for the arrival wait (`ldrh r2,[r4,#4] / cmp r2,#4`),
                // 0 for the leave wait (`sxth r0 / cmp r0,#0 / bgt`) - and then wake it.
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
            }   // legacy_watchdog
        }
        // Lower the tick line again: the distributor is level triggered, so a line
        // left asserted would keep PPI 29 pending and turn the tick into a storm.
        if (tick_pending_) {
            tick_pending_ = false;
            if (kermit_ != nullptr) kermit_->raise_irq(kermit::kIrqPpiPrivateTimer, false);
        }
    }

    for (int step = 0; step < budget_.arm; ++step) {
        bool stop_slice = false;
        // ZLB_ARM_BUDGET=<n> gives each core n instructions before the next one runs
        // (default 1: instruction-level round robin).  Real cores run in parallel, so
        // a block is closer to the hardware; the KBL's barrier was the reason the
        // default was lowered to 1 (see the note above), and this knob measures both
        // ends (docs/KBL.md 7.1.23).
        static const int core_budget = [] {
            const char* value = std::getenv("ZLB_ARM_BUDGET");
            if (value == nullptr) return 1;
            const int parsed = std::atoi(value);
            return parsed >= 1 ? parsed : 1;
        }();
        for (int i = 0; i < kArmCoreCount; ++i) {
            Cpu* core = arm_cores_[static_cast<size_t>(i)].get();
            if (!core || core->halted) continue;
            // Diagnostic: ZLB_ARM_CORE_STAGGER=<n> holds core i back for i*n of its
            // own instructions, modelling the natural start skew real cores have.
            // Round-robin lockstep is what makes the KBL's per-core phases
            // order-sensitive (docs/KBL.md 7.1.20), so this measures the effect.
            static const u64 stagger = [] {
                const char* value = std::getenv("ZLB_ARM_CORE_STAGGER");
                if (value == nullptr) return 0ull;
                return static_cast<u64>(std::strtoull(value, nullptr, 10));
            }();
            if (stagger != 0ull && core->instructions < stagger * static_cast<u64>(i)) continue;
            for (int b = 0; b < core_budget; ++b) {
            if (core->halted) break;
            if (pc_trace_enabled_) trace_arm_boot_pc(static_cast<u32>(i), core->get_pc());
            if (arm_pc_log_enabled_ && arm_pc_log_ != nullptr) {
                // Cap per core so the KBL phase (which is over well before a million
                // instructions) is always covered in both runs.  ZLB_ARM_PC_REGS=1
                // adds r0..r3, sp, lr and the flags: two runs can follow the same path
                // yet carry different data, and only the full tuple shows that.
                const u64 logged = core->instructions;
                if (logged < static_cast<u64>(arm_pc_log_limit_)) {
                    if (arm_pc_log_regs_) {
                        const ArmCore* a = dynamic_cast<const ArmCore*>(core);
                        if (a != nullptr) {
                            std::fprintf(arm_pc_log_.get(),
                                         "%d:%08X %08X %08X %08X %08X %08X %08X %08X\n", i,
                                         core->get_pc(), a->r[0], a->r[1], a->r[2], a->r[3],
                                         a->r[13], a->r[14], a->cpsr);
                        } else {
                            std::fprintf(arm_pc_log_.get(), "%d:%08X\n", i, core->get_pc());
                        }
                    } else {
                        std::fprintf(arm_pc_log_.get(), "%d:%08X\n", i, core->get_pc());
                    }
                }
            }
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
            // Ring of the most recent ARM PCs. When a module start returns the
            // MODULEMGR_NO_LIB error the ring is dumped, deduplicated, so the path
            // that produced the failure is visible (the loader's own code, not the
            // ordinary resolution pass the persistent probes keep catching).
            if (i < kArmTraceCores) {
                g_arm_trace_ring[i][(g_arm_trace_pos[i]++) & (kArmTraceSize - 1u)] = arm_pc;
            }
            if (arm_pc >= 0x51000000u && arm_pc < 0x51100000u) pc_set_add(arm_pc);
            nskbl_ring_add(i, arm_pc);
            // All 25 NSKBL sites that build 0x80024501 (SCE_KERNEL_ERROR_INVALID_UID,
            // SDK kernel/error.h:310). The failing module start returns that code, so
            // whichever of them executes is the check that rejects a UID - in this
            // branch the value is r6 = [r10+8].
            {
                static const u32 invalid_uid_sites[] = {
                    0x51004A12u, 0x51004A2Au, 0x51004B26u, 0x51004B9Eu, 0x51004BC4u,
                    0x51004C5Eu, 0x51004FBCu, 0x51004FCEu, 0x5100509Au, 0x51005172u,
                    0x510052A2u, 0x510052ACu, 0x510052CEu, 0x510052D8u, 0x510052F8u,
                    0x510053B4u, 0x51005420u, 0x510055BAu, 0x510055DEu, 0x510055E8u,
                    0x5100560Au, 0x51005646u, 0x51005650u, 0x510056C0u, 0x51016CFAu,
                };
                for (u32 site : invalid_uid_sites) {
                    if (arm_pc != site) continue;
                    static u32 site_logged = 0;
                    if (site_logged < 20u) {
                        ++site_logged;
                        if (const ArmCore* uid_core = dynamic_cast<const ArmCore*>(core)) {
                            uid_ring_add(arm_pc, uid_core->r[6]);
                        }
                    }
                    break;
                }
            }
            // The module-start failure ends at 0x51005172, which returns
            // 0x80024501 = SCE_KERNEL_ERROR_INVALID_UID (SDK kernel/error.h line 310).
            // r6 was loaded from [r10+8] at 0x51005162 and is the UID being rejected:
            // its bits 0x500000 and 0xA00000 are clear on the failing path.
            // Division results: 0x5100B84C follows "blx 0x510258D0" (r0 = table base,
            // r1 = count) in the UID lookup, and 0x5100B856 follows the second call
            // (r0 = class index, r1 = the first result). Compare both with theory.
            if (arm_pc == 0x5100B84Cu || arm_pc == 0x5100B856u) {
                static u32 div_logged = 0;
                if (div_logged < 20u) {
                    ++div_logged;
                    if (const ArmCore* dv = dynamic_cast<const ArmCore*>(core)) {
                        ZLB_LOG_INFO("machine", "div result at 0x%08X: r0=0x%08X r1=0x%08X r2=0x%08X",
                                     arm_pc, dv->r[0], dv->r[1], dv->r[2]);
                    }
                }
            }
            // 0x510258D0 is the software division the UID lookup uses for its bucket
            // index. It starts with "clz r3,r0" at 0x510258EC and "clz r2,r1" at
            // 0x510258F0, so probing 0x510258F0 (r0 = dividend, r3 = clz(r0)) and
            // 0x510258F4 (r1 = divisor, r2 = clz(r1)) checks CLZ against theory.
            if (arm_pc == 0x510258F0u || arm_pc == 0x510258F4u) {
                static u32 clz_logged = 0;
                if (clz_logged < 24u) {
                    ++clz_logged;
                    if (const ArmCore* cz = dynamic_cast<const ArmCore*>(core)) {
                        ZLB_LOG_INFO("machine", "clz probe 0x%08X: r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X",
                                     arm_pc, cz->r[0], cz->r[1], cz->r[2], cz->r[3]);
                    }
                }
            }
            // 0x5100B82C is the UID table lookup: r0 = table base, r1 = the class
            // index derived from the UID, r2 = the out pointer. The table layout read
            // there is +0x1C base, +0x20 count, +0x32 count2, +0x34 array, and the
            // index is bounded by count2 (return "not found" when it is too large).
            if (arm_pc == 0x5100B82Cu) {
                static u32 tbl_logged = 0;
                if (tbl_logged < 30u) {
                    if (ArmCore* tb = dynamic_cast<ArmCore*>(core)) {
                    // The routine table walks use classes 0..8; sdif.skprx's UID
                    // 0x200F3 yields class (0x200F3 >> 1) & 0x7FFF = 0x10079, so only
                    // report the large-class lookups.
                    if (tb->r[1] < 0x40u) return;
                    ++tbl_logged;
                        const u32 base = tb->r[0];
                        u32 count = 0, count2 = 0;
                        if (const arm::MmResult r1 = tb->mmu.translate(base + 0x20u, false, false, tb->mode()); r1.ok) {
                            count = arm_bus_->read32(r1.phys_addr) & 0xFFFFu;
                        }
                        if (const arm::MmResult r2 = tb->mmu.translate(base + 0x32u, false, false, tb->mode()); r2.ok) {
                            count2 = arm_bus_->read32(r2.phys_addr) & 0xFFFFu;
                        }
                        ZLB_LOG_INFO("machine", "uid table: base=0x%08X index=0x%X count=0x%X count2=0x%X",
                                     base, tb->r[1], count, count2);
                    }
                }
            }
            // 0x51004A04 is "ubfx r1,r1,#1,#1": log r1 on entry and on the next
            // instruction, so the extracted value can be compared with the expected
            // (uid >> 1) & 1.
            if (arm_pc == 0x51004A04u || arm_pc == 0x51004A08u) {
                static u32 bfx_logged = 0;
                if (bfx_logged < 14u) {
                    ++bfx_logged;
                    if (const ArmCore* bf = dynamic_cast<const ArmCore*>(core)) {
                        ZLB_LOG_INFO("machine", "ubfx step 0x%08X: r1=0x%08X r4(uid)=0x%08X",
                                     arm_pc, bf->r[1], bf->r[4]);
                    }
                }
            }
            // 0x51004A04 is "ubfx r1,r1,#1,#1" (extract the UID class bit) and
            // 0x51004A0A is the "bl 0x5100B82C" lookup that uses it. r1 must be
            // (uid >> 1) & 1 on entry to the lookup - for uid 0x200F3 that is 1.
            if (arm_pc == 0x51004A0Au) {
                static u32 class_logged = 0;
                if (class_logged < 10u) {
                    ++class_logged;
                    if (const ArmCore* cls = dynamic_cast<const ArmCore*>(core)) {
                        ZLB_LOG_INFO("machine", "uid lookup: class(r1)=0x%X uid(r4)=0x%08X r2=0x%08X",
                                     cls->r[1], cls->r[4], cls->r[2]);
                    }
                }
            }
            // Opcode sanity: 0x51004A12/0x51004A16 are "movw r0,#0x4501" then
            // "movt r0,#0x8002"; 0x51004A1A is the instruction after them, so r0 must
            // be 0x80024501 there if movw/movt execute correctly. Likewise 0x51004A22
            // compares the requested UID (r4) with the record's UID (r0).
            if (arm_pc == 0x51004A1Au || arm_pc == 0x51004A22u) {
                static u32 op_logged = 0;
                if (op_logged < 16u) {
                    ++op_logged;
                    if (const ArmCore* op = dynamic_cast<const ArmCore*>(core)) {
                        ZLB_LOG_INFO("machine", "opcode check at 0x%08X: r0=0x%08X r4=0x%08X",
                                     arm_pc, op->r[0], op->r[4]);
                    }
                }
            }
            // 0x51004A0E is the "cmp r0,#0" right after the UID lookup at 0x51004A0A:
            // r0 is the lookup result and r4 the UID being validated. Recording both
            // shows which UID fails, which is the object the model never created.
            if (arm_pc == 0x51004A0Eu) {
                if (const ArmCore* lk = dynamic_cast<const ArmCore*>(core)) {
                    uid_ring_add(0x51004A0Eu, lk->r[4]);
                    uid_ring_result(g_uid_ring_pos - 1u, lk->r[0]);
                }
            }
            if (arm_pc == 0x51005172u) {
                static u32 uid_logged = 0;
                if (uid_logged < 12u) {
                    ++uid_logged;
                    if (const ArmCore* uid_core = dynamic_cast<const ArmCore*>(core)) {
                        ZLB_LOG_INFO("machine",
                                     "module: INVALID_UID at 0x51005172 r6(uid)=0x%08X r10=0x%08X r5=0x%08X r7=0x%08X",
                                     uid_core->r[6], uid_core->r[10], uid_core->r[5], uid_core->r[7]);
                    }
                }
            }
            // Secure World question: does anything ever hand the ARM over to its
            // TrustZone side? Count instructions per mode and report the first time
            // each mode is seen. USR/SVC/SYS/IRQ/FIQ/ABT/UND/MON, and the monitor
            // mode only runs if some code issued SMC.
            if (const ArmCore* mode_core = dynamic_cast<const ArmCore*>(core)) {
                static u64 mode_counts[32] = {};
                static u32 modes_logged = 0;
                const u32 mode_now = mode_core->mode() & 0x1Fu;
                if (mode_now < 32u) {
                    ++mode_counts[mode_now];
                    if (mode_counts[mode_now] == 1u && modes_logged < 12u) {
                        ++modes_logged;
                        ZLB_LOG_INFO("machine", "arm: first instruction in mode 0x%02X (pc=0x%08X core=%d)",
                                     mode_now, arm_pc, i);
                    }
                }
                // Periodic per-mode census: shows whether the Secure World keeps
                // being entered after the early boot, and how often.
                static u64 mode_total = 0;
                static u32 mode_reports = 0;
                if (++mode_total % 200000000ull == 0ull && mode_reports < 8u) {
                    ++mode_reports;
                    ZLB_LOG_INFO("machine",
                                 "arm modes: SVC=%llu IRQ=%llu FIQ=%llu SYS=%llu MON=%llu USR=%llu (total=%llu)",
                                 static_cast<unsigned long long>(mode_counts[0x13]),
                                 static_cast<unsigned long long>(mode_counts[0x12]),
                                 static_cast<unsigned long long>(mode_counts[0x11]),
                                 static_cast<unsigned long long>(mode_counts[0x1F]),
                                 static_cast<unsigned long long>(mode_counts[0x16]),
                                 static_cast<unsigned long long>(mode_counts[0x10]),
                                 static_cast<unsigned long long>(mode_total));
                }
            }
            // Diagnostic (ZLB_MODULE_LOG=1): NSKBL's native module-start loop.  The
            // remaining boot gap is that only 22 of the 28 modules the bootconfig
            // lists are ever started (docs/STATUS.md), so this logs both sides of the
            // question: every slot the loop pulls out of its UID array (`r5` walks it
            // with `ldr r9/r10, [r5], #4`) and every entry into the start helper
            // 0x510194C0, whose argument is the module UID.  A loaded-but-skipped
            // module then shows up as a slot with no matching start.  Read-only: it
            // inspects registers and never touches the core's state.
            static const bool module_log = [] {
                const char* value = std::getenv("ZLB_MODULE_LOG");
                return value != nullptr && value[0] != '\0' && value[0] != '0';
            }();
            if (module_log) {
                // Where a core goes idle is where its current thread was parked.  The
                // module-start thread is abandoned once (round 5/6: 22 starts, 21
                // returns, and 12M further slices start nothing), so dumping the last
                // few pcs of every core when it enters the kernel idle loop names the
                // parking path - the missing piece for "what is the thread waiting on".
                static u32 idle_ring[kArmCoreCount][8] = {};
                static u32 idle_ring_at[kArmCoreCount] = {};
                static u32 idle_logged = 0;
                // Only interesting once the module phase has begun: the early kernel
                // init uses the same wfe wait loop, and it must not eat the budget.
                static bool any_module_start = false;
                // The interesting window is *after* the last start (UID 0x200ED, the
                // 22nd); the earlier counters were exhausted long before it, so the
                // aftermath went unlogged.  This mirrors the count in the loop below.
                static u32 starts_seen = 0;
                if (i < static_cast<int>(kArmCoreCount)) {
                    const u32* ring = idle_ring[i];
                    const u32 next = idle_ring_at[i]++;
                    idle_ring[i][next & 7u] = arm_pc;
                    if (arm_pc == 0x47969Cu && i == 0 && starts_seen >= 22u && idle_logged < 40u &&
                        ring[(next + 7u) & 7u] != 0x47969Cu) {
                        ++idle_logged;
                        std::string path;
                        for (u32 k = 0; k < 8u; ++k) {
                            path += format(" %08X", ring[(next + 1u + k) & 7u]);
                        }
                        ZLB_LOG_INFO("machine", "module: core %d idle entry #%u from%s", i, idle_logged,
                                     path.c_str());
                    }
                    // The kernel thread switcher restores a thread's context here; the
                    // pointer (r1) names the thread's control block, which is how the
                    // parked module-start thread can be found and its saved pc read.
                    static u32 restore_logged = 0;
                    if (starts_seen >= 22u && i == 0 && arm_pc == 0x000EC848u && restore_logged < 12u) {
                        ++restore_logged;
                        const ArmCore* ctx_arm = dynamic_cast<const ArmCore*>(core);
                        ZLB_LOG_INFO("machine",
                                     "module: core 0 context restore #%u from=0x%08X ptr=0x%08X sp=0x%08X lr=0x%08X",
                                     restore_logged, ctx_arm != nullptr ? ctx_arm->r[4] : 0u,
                                     ctx_arm != nullptr ? ctx_arm->r[1] : 0u,
                                     ctx_arm != nullptr ? ctx_arm->r[13] : 0u,
                                     ctx_arm != nullptr ? ctx_arm->r[14] : 0u);
                    }
                    // 0x4394CE loads the queue object the library links the op into:
                    // r8 = [op+0x20], a doubly-linked list with head [r8] and tail [r8+4].
                    // Knowing the queue address says whether anything ever consumes it.
                    static u32 q_logged = 0;
                    if (starts_seen >= 22u && i == 0 && arm_pc == 0x004394CEu && q_logged < 4u) {
                        ++q_logged;
                        const ArmCore* qc = dynamic_cast<const ArmCore*>(core);
                        if (qc != nullptr) {
                            ZLB_LOG_INFO("machine", "module: DMA queue op=0x%08X queue=0x%08X (r8) head=0x%08X",
                                         qc->r[4], qc->r[8], 0u);
                        }
                    }
                    // Trace the enqueue itself: from its call site (0x5BE956) until it
                    // returns (0x5BE95A). That shows how the DMA library reaches the kernel
                    // to submit an operation, and where the transfer should be started.
                    static bool enq_trace = false;
                    static u32 enq_seen[220];
                    static u32 enq_count = 0;
                    static u32 enq_logged2 = 0;
                    if (starts_seen >= 22u && i == 0) {
                        if (arm_pc == 0x005BE956u) {
                            enq_trace = true;
                            enq_count = 0;
                        } else if (enq_trace && arm_pc == 0x005BE95Au) {
                            enq_trace = false;
                        } else if (enq_trace && enq_count < 220u) {
                            bool known = false;
                            for (u32 k = 0; k < enq_count; ++k) {
                                if (enq_seen[k] == arm_pc) {
                                    known = true;
                                    break;
                                }
                            }
                            if (!known) {
                                enq_seen[enq_count++] = arm_pc;
                                if (enq_logged2 < 220u) {
                                    ++enq_logged2;
                                    ZLB_LOG_INFO("machine", "module: enqueue pc #%u 0x%08X", enq_logged2, arm_pc);
                                }
                            }
                        }
                    }
                    // The sibling entry 0x4ADD9C shares 0x4AD714 but passes flag 1. Probing
                    // both shows which service each of the module's DMA calls reaches.
                    static u32 sys2_logged = 0;
                    if (starts_seen >= 22u && i == 0 && arm_pc == 0x004ADD9Cu && sys2_logged < 16u) {
                        ++sys2_logged;
                        const ArmCore* sc2 = dynamic_cast<const ArmCore*>(core);
                        if (sc2 != nullptr) {
                            ZLB_LOG_INFO("machine", "module: syscall 0x4ADD9C #%u r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X lr=0x%08X",
                                         sys2_logged, sc2->r[0], sc2->r[1], sc2->r[2], sc2->r[3], sc2->r[14]);
                        }
                    }
                    // The other three sites that build 0x8002D003 are in the KBL's loader
                    // (VA 0x40024CF8, 0x40024D82, 0x40024DC2), which NSKBL reuses when it
                    // resolves a module's imports. Probing them catches the failing lookup.
                    static u32 kbl_logged = 0;
                    if ((arm_pc == 0x40024CF8u || arm_pc == 0x40024D82u || arm_pc == 0x40024DC2u) &&
                        kbl_logged < 8u) {
                        ++kbl_logged;
                        ArmCore* kb = dynamic_cast<ArmCore*>(core);
                        if (kb != nullptr) {
                            // r0 names the structure the lookup is working on (a bootconfig
                            // library record, later consumed and zeroed). Dump its first
                            // words and, if one points at a name string, the string itself.
                            u32 words[4] = {0, 0, 0, 0};
                            std::string text;
                            // Try every register as a pointer: the loader's lookup takes
                            // the library name (or a record holding it) in one of r0..r3,
                            // and this fires at the moment it is live.
                            auto read_string_at = [&](u32 va, char* buf, size_t cap) -> bool {
                                if (va < 0x1000u || va > 0x0FFFFFFFu) return false;
                                const arm::MmResult r = kb->mmu.translate(va, false, false, kb->mode());
                                if (!r.ok) return false;
                                size_t c = 0;
                                for (; c + 1u < cap; ++c) {
                                    const char ch =
                                        static_cast<char>(arm_bus_->read32(r.phys_addr + c) & 0xFFu);
                                    if (ch < 32 || ch > 126) break;
                                    buf[c] = ch;
                                }
                                buf[c] = '\0';
                                return c >= 4u;
                            };
                            for (u32 k = 0; k < 4u; ++k) {
                                const arm::MmResult r =
                                    kb->mmu.translate(kb->r[k], false, false, kb->mode());
                                if (r.ok) words[k] = arm_bus_->read32(r.phys_addr);
                            }
                            char buf[48];
                            for (u32 k = 0; k < 4u && text.empty(); ++k) {
                                if (read_string_at(kb->r[k], buf, sizeof(buf))) text = buf;
                            }
                            for (u32 w = 0; w < 4u && text.empty(); ++w) {
                                const u32 candidate = words[w];
                                for (u32 delta = 0; delta <= 0x40u && text.empty(); delta += 4u) {
                                    if (read_string_at(candidate + delta, buf, sizeof(buf))) text = buf;
                                }
                            }
                            ZLB_LOG_INFO("machine",
                                         "module: NO_LIB(KBL) at 0x%08X r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X lr=0x%08X "
                                         "words=[0x%08X 0x%08X 0x%08X 0x%08X] text=\"%s\"",
                                         arm_pc, kb->r[0], kb->r[1], kb->r[2], kb->r[3], kb->r[14],
                                         words[0], words[1], words[2], words[3], text.c_str());
                        }
                    }
                    // The code that builds SCE_KERNEL_ERROR_MODULEMGR_NO_LIB (0x8002D003)
                    // lives at PA 0x407252F0/0x40725378, which maps to VA ~0x5992F0 in the
                    // module loaded at 0x590000. Probing it shows what the failing lookup
                    // was holding - the library name pointer in particular.
                    static u32 nolib_logged = 0;
                    if ((arm_pc == 0x0058D2F0u || arm_pc == 0x0058D378u) && nolib_logged < 8u) {
                        ++nolib_logged;
                        const ArmCore* nb = dynamic_cast<const ArmCore*>(core);
                        if (nb != nullptr) {
                            ZLB_LOG_INFO("machine",
                                         "module: NO_LIB site 0x%08X r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X lr=0x%08X core=%d",
                                         arm_pc, nb->r[0], nb->r[1], nb->r[2], nb->r[3], nb->r[14], i);
                        }
                    }
                    // Trace a module start end to end: from the helper's call (0x510194FE)
                    // to its return (0x51019502). Start #23 (sdif.skprx) fails with
                    // MODULEMGR_NO_LIB inside this window, so the distinct code addresses
                    // are what shows where the import resolution gives up.
                    static bool start_trace = false;
                    static u32 start_seen[300];
                    static u32 start_count = 0;
                    static u32 start_logged = 0;
                    if (arm_pc == 0x510194FEu && !start_trace) {
                        start_trace = true;
                        start_count = 0;
                    } else if (start_trace && arm_pc == 0x51019502u) {
                        start_trace = false;
                    } else if (start_trace && start_count < 300u) {
                        bool known = false;
                        for (u32 k = 0; k < start_count; ++k) {
                            if (start_seen[k] == arm_pc) {
                                known = true;
                                break;
                            }
                        }
                        if (!known) {
                            start_seen[start_count++] = arm_pc;
                            if (start_logged < 300u) {
                                ++start_logged;
                                ZLB_LOG_INFO("machine", "module: start pc #%u 0x%08X", start_logged, arm_pc);
                            }
                        }
                    }
                    // 0x510194C0 is NSKBL's module-start helper; its arguments carry the
                    // entry point of the module being started, which is how a failing
                    // module (sdif.skprx at start #23) can be located.
                    static u32 helper_logged = 0;
                    if (arm_pc == 0x510194C0u && helper_logged < 40u) {
                        ++helper_logged;
                        const ArmCore* hc = dynamic_cast<const ArmCore*>(core);
                        if (hc != nullptr) {
                            ZLB_LOG_INFO("machine", "module: start helper #%u r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X core=%d",
                                         helper_logged, hc->r[0], hc->r[1], hc->r[2], hc->r[3], i);
                        }
                    }
                    // ksceKernelSetEventFlag is implemented at 0x4ADEA8 (the stub 0x43AD88
                    // jumps there). If the DMA completion really signals the flag the
                    // parked thread waits on, this call must carry r0 = 0x10AB9.
                    static u32 sef_logged = 0;
                    if (arm_pc == 0x004ADEA8u && sef_logged < 8u) {
                        ++sef_logged;
                        const ArmCore* sf = dynamic_cast<const ArmCore*>(core);
                        if (sf != nullptr) {
                            ZLB_LOG_INFO("machine", "module: SetEventFlag #%u uid=0x%08X bits=0x%08X core=%d t=%.6f",
                                         sef_logged, sf->r[0], sf->r[1], i, emulated_seconds());
                        }
                    }
                    // Trace the guest's DMA interrupt handler: from its entry (0x438790)
                    // to its tail (0x43879C), collecting the distinct code addresses and
                    // the argument the kernel passes in r0.
                    static bool isr_trace = false;
                    static u32 isr_seen[200];
                    static u32 isr_count = 0;
                    static u32 isr_logged = 0;
                    if (arm_pc == 0x00438790u && !isr_trace) {
                        isr_trace = true;
                        isr_count = 0;
                        const ArmCore* ia = dynamic_cast<const ArmCore*>(core);
                        ZLB_LOG_INFO("machine", "module: DMA ISR entry r0=0x%08X r1=0x%08X core=%d",
                                     ia != nullptr ? ia->r[0] : 0u, ia != nullptr ? ia->r[1] : 0u, i);
                    } else if (isr_trace && arm_pc == 0x0043879Cu) {
                        isr_trace = false;
                    } else if (isr_trace && isr_count < 200u) {
                        bool known = false;
                        for (u32 k = 0; k < isr_count; ++k) {
                            if (isr_seen[k] == arm_pc) {
                                known = true;
                                break;
                            }
                        }
                        if (!known) {
                            isr_seen[isr_count++] = arm_pc;
                            if (isr_logged < 200u) {
                                ++isr_logged;
                                ZLB_LOG_INFO("machine", "module: ISR pc #%u 0x%08X", isr_logged, arm_pc);
                            }
                        }
                    }
                    // The DMA library registered handler 0x438791 for the channel IRQs
                    // (0x70..0x7F). Whether the guest actually takes the interrupt the
                    // model pulses is what the completion experiment turns on.
                    static u32 dmairq_logged = 0;
                    if (arm_pc == 0x00438790u && dmairq_logged < 8u) {
                        ++dmairq_logged;
                        ZLB_LOG_INFO("machine", "module: DMA irq handler 0x438790 ran #%u core=%d t=%.6f",
                                     dmairq_logged, i, emulated_seconds());
                    }
                    // ksceKernelRegisterIntrHandler is implemented at 0xED29C (the stub
                    // 0x43ACF8 jumps there). Its arguments name the interrupt a driver
                    // wants - the DMA library's own registration is what says which IRQ
                    // its completion handler runs on.
                    static u32 intr_logged = 0;
                    if (arm_pc == 0x000ED29Cu && intr_logged < 64u) {
                        ++intr_logged;
                        const ArmCore* ic = dynamic_cast<const ArmCore*>(core);
                        if (ic != nullptr) {
                            ZLB_LOG_INFO("machine",
                                         "module: RegisterIntrHandler #%u irq=0x%02X name=0x%08X r2=0x%08X handler=0x%08X "
                                         "arg=[sp]=0x%08X [sp+4]=0x%08X lr=0x%08X core=%d",
                                         intr_logged, ic->r[0], ic->r[1], ic->r[2], ic->r[3],
                                         arm_bus_->read32(static_cast<u32>(ic->r[13])),
                                         arm_bus_->read32(static_cast<u32>(ic->r[13]) + 4u), ic->r[14], i);
                        }
                    }
                    // 0x4ADD88 is where the library's syscall trampoline (0x43AD28) lands in
                    // the kernel. Its arguments name the service the DMA library asks for -
                    // and therefore what the parked thread is really waiting on.
                    static u32 sys_logged = 0;
                    if (starts_seen >= 22u && i == 0 && arm_pc == 0x004ADD88u && sys_logged < 16u) {
                        ++sys_logged;
                        const ArmCore* sc = dynamic_cast<const ArmCore*>(core);
                        if (sc != nullptr) {
                            ZLB_LOG_INFO("machine", "module: syscall 0x4ADD88 #%u r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X lr=0x%08X",
                                         sys_logged, sc->r[0], sc->r[1], sc->r[2], sc->r[3], sc->r[14]);
                        }
                    }
                    // What the kernel actually does between "the DMA operation was
                    // queued" (0x5BE95A) and "the thread parks": collect the distinct
                    // code addresses of that window.  1 ms of guest time is thousands
                    // of instructions, but the distinct set stays small and names the
                    // functions involved.
                    static bool trace_window = false;
                    static u32 window_seen[400];
                    static u32 window_count = 0;
                    static u32 window_logged = 0;
                    if (arm_pc == 0x005BE95Au && starts_seen >= 22u && i == 0) {
                        trace_window = true;
                        window_count = 0;
                    }
                    if (trace_window && i == 0) {
                        bool known = false;
                        for (u32 k = 0; k < window_count; ++k) {
                            if (window_seen[k] == arm_pc) {
                                known = true;
                                break;
                            }
                        }
                        if (!known && window_count < 400u) {
                            window_seen[window_count++] = arm_pc;
                            if (window_logged < 400u) {
                                ++window_logged;
                                ZLB_LOG_INFO("machine", "module: post-enqueue pc #%u 0x%08X t=%.6f",
                                             window_logged, arm_pc, emulated_seconds());
                            }
                        }
                    }
                    // 0x5BE95A is the instruction right after the display module's
                    // ksceKernelDmaOpEnQueue call, so r0 there is that call's return
                    // value: it decides whether the operation was really queued.
                    static u32 enq_logged = 0;
                    if (starts_seen >= 22u && i == 0 && arm_pc == 0x005BE95Au && enq_logged < 6u) {
                        ++enq_logged;
                        const ArmCore* eq = dynamic_cast<const ArmCore*>(core);
                        ZLB_LOG_INFO("machine", "module: DmaOpEnQueue returned 0x%08X (t=%.6f)",
                                     eq != nullptr ? eq->r[0] : 0u, emulated_seconds());
                    }
                    // 0x4399CA is the import wrapper the display module reaches before the
                    // syscall trampoline (0x43AD28) and the kernel. Its r0 names the import
                    // slot and its LR names the module's call site, which is what identifies
                    // the service the module is blocked on.
                    static u32 imp_logged = 0;
                    if (starts_seen >= 22u && i == 0 && arm_pc == 0x004399CAu && imp_logged < 24u) {
                        ++imp_logged;
                        const ArmCore* im = dynamic_cast<const ArmCore*>(core);
                        if (im != nullptr) {
                            ZLB_LOG_INFO("machine", "module: import wrapper #%u r0=0x%08X r1=0x%08X r2=0x%08X lr=0x%08X",
                                         imp_logged, im->r[0], im->r[1], im->r[2], im->r[14]);
                        }
                    }
                    // The kernel routine that ends up parking the thread is 0x4AE710
                    // (six arguments). Its arguments and its caller name the operation
                    // the module initialisation is blocked on, which is the one thing
                    // left unidentified.
                    static u32 enter_logged = 0;
                    if (starts_seen >= 22u && i == 0 && arm_pc == 0x004AE710u && enter_logged < 8u) {
                        ++enter_logged;
                        const ArmCore* en = dynamic_cast<const ArmCore*>(core);
                        if (en != nullptr) {
                            ZLB_LOG_INFO("machine",
                                         "module: park entry #%u r0=0x%08X r1=0x%08X r2=0x%08X r3=0x%08X "
                                         "s52=0x%08X s56=0x%08X lr=0x%08X",
                                         enter_logged, en->r[0], en->r[1], en->r[2], en->r[3],
                                         arm_bus_->read32(static_cast<u32>(en->r[13]) + 52u),
                                         arm_bus_->read32(static_cast<u32>(en->r[13]) + 56u), en->r[14]);
                        }
                    }
                    // The wait descriptor at VA 0x0030FA80 (=[TCB+0x0C]) carries two code
                    // pointers - 0x0048E55C and 0x0049F5D8 - which is where a completion
                    // would land. Whether they ever execute after the park decides
                    // between "the wake never comes" and "the wake ran but did not
                    // resume the thread".
                    static u32 callback_logged = 0;
                    if (starts_seen >= 22u && callback_logged < 24u &&
                        (arm_pc == 0x0048E55Cu || arm_pc == 0x0049F5D8u)) {
                        ++callback_logged;
                        ZLB_LOG_INFO("machine", "module: wait callback 0x%08X ran at t=%.6f core=%d",
                                     arm_pc, emulated_seconds(), i);
                    }
                    // The abandoned thread's stack is 0x7D000..0x80000 (TCB+0xDC/+0xE0).
                    // Any execution with SP inside it means that thread is running, so a
                    // coarse timeline of such samples answers "did it ever come back".
                    static double stack_last = -1.0;
                    static u32 stack_logged = 0;
                    if (starts_seen >= 22u && stack_logged < 40u) {
                        const ArmCore* st_arm = dynamic_cast<const ArmCore*>(core);
                        const u32 sp = st_arm != nullptr ? st_arm->r[13] : 0u;
                        if (sp >= 0x0007D000u && sp < 0x00080000u) {
                            const double now = emulated_seconds();
                            if (stack_last < 0.0 || now - stack_last >= 0.05) {
                                stack_last = now;
                                ++stack_logged;
                                ZLB_LOG_INFO("machine", "module: thread-stack sample #%u t=%.6f pc=0x%08X sp=0x%08X core=%d",
                                             stack_logged, now, arm_pc, sp, i);
                            }
                        }
                    }
                    // The park site itself: 0x4A93F0 is the call into the kernel thread
                    // switcher (0x4A3D94 -> 0xEC8AC) with r4 = TPIDRPRW, the current
                    // thread structure, and [r4+12]/[r4+16] as the wait arguments.
                    // Logging r4 names the object the abandoned thread waits on.
                    static u32 park_logged = 0;
                    if (starts_seen >= 22u && i == 0 && arm_pc == 0x4A93F0u && park_logged < 60u) {
                        ++park_logged;
                        const ArmCore* park_arm = dynamic_cast<const ArmCore*>(core);
                        const u32 tcb = park_arm != nullptr ? park_arm->r[4] : 0u;
                        ZLB_LOG_INFO("machine",
                                     "module: core 0 park #%u t=%.6f TCB=0x%08X sp=0x%08X obj=0x%08X",
                                     park_logged, emulated_seconds(), tcb,
                                     park_arm != nullptr ? park_arm->r[13] : 0u,
                                     park_arm != nullptr ? park_arm->r[0] : 0u);
                    }
                    // What the waiting thread asked for: 0x4A9384 is the entry of the
                    // kernel wait function; r0/r1 are its arguments and LR names the
                    // caller, which is the operation the module-start thread blocks on.
                    static u32 waitcall_logged = 0;
                    if (starts_seen >= 22u && i == 0 && arm_pc == 0x4A9384u && waitcall_logged < 6u) {
                        ++waitcall_logged;
                        const ArmCore* wc = dynamic_cast<const ArmCore*>(core);
                        ZLB_LOG_INFO("machine",
                                     "module: core 0 wait call #%u r0=0x%08X r1=0x%08X lr=0x%08X sp=0x%08X",
                                     waitcall_logged, wc != nullptr ? wc->r[0] : 0u,
                                     wc != nullptr ? wc->r[1] : 0u, wc != nullptr ? wc->r[14] : 0u,
                                     wc != nullptr ? wc->r[13] : 0u);
                    }
                    // Coarse whereabouts of core 0 after the module phase began: every
                    // 200k executions the pc is logged, which shows whether the
                    // abandoned thread stays in one loop or moves through the kernel.
                    static u64 where_tick = 0;
                    static u32 where_logged = 0;
                    if (any_module_start && i == 0 && (++where_tick % 2000000ull) == 0ull &&
                        where_logged < 60u) {
                        ++where_logged;
                        ZLB_LOG_INFO("machine", "module: core 0 pc sample %u: 0x%08X", where_logged, arm_pc);
                    }
                    // 0x47AE92 is the body of the kernel's wfe wait loop
                    // (while [r4+4] > 0).  r4 is the object being waited on, so naming
                    // it (and the counter's value) says *what* the parked thread waits
                    // for; the two halves at +4/+6 are the same pair NSKBL's spinlock
                    // 0x51015874 uses.
                    static u32 wait_logged = 0;
                    if (arm_pc == 0x47AE92u && i == 0 && starts_seen >= 22u && wait_logged < 400u) {
                        ++wait_logged;
                        const ArmCore* wait_arm = dynamic_cast<const ArmCore*>(core);
                        const u32 obj = wait_arm != nullptr ? wait_arm->r[4] : 0u;
                        const u32 ctr = wait_arm != nullptr ? wait_arm->r[6] : 0u;
                        ZLB_LOG_INFO("machine", "module: core 0 wfe-wait #%u object=0x%08X (+4=0x%08X)",
                                     wait_logged, obj, ctr);
                    }
                }
                static u32 nskbl_last[kArmCoreCount] = {0, 0, 0, 0};
                if (i < static_cast<int>(kArmCoreCount)) {
                    const bool in_nskbl = arm_pc >= 0x51000000u && arm_pc < 0x51100000u;
                    static bool was_in_nskbl[kArmCoreCount] = {false, false, false, false};
                    if (in_nskbl) {
                        nskbl_last[i] = arm_pc;
                        was_in_nskbl[i] = true;
                    } else if (was_in_nskbl[i]) {
                        was_in_nskbl[i] = false;
                        ZLB_LOG_INFO("machine", "module: core %d left NSKBL at 0x%08X -> 0x%08X", i,
                                     nskbl_last[i], arm_pc);
                    }
                }
                // 0x510012F4 is the batch entry (r0=records, r1=UID array, r2=count);
                // 0x51001326 / 0x51001368 / 0x5100139C follow the three UID loads in
                // it - the UID is in r9 at the first and in r10 at the other two;
                // 0x510194C0 is the start helper.
                const bool batch = arm_pc == 0x510012F4u;
                const bool slot = arm_pc == 0x51001326u || arm_pc == 0x51001368u || arm_pc == 0x5100139Cu;
                // The three instructions after the three `bl 0x510194C0` sites: the
                // helper's return value is in r0, and a negative one makes the loop
                // `blt 0x510013F4` straight out, abandoning the rest of the batch.
                const bool result = arm_pc == 0x510013F0u || arm_pc == 0x51001410u || arm_pc == 0x5100149Cu;
                // 0x510194E4 is the helper's `pop {r4-r7, r15}`: a start whose
                // helper never reaches it is a start that never returned.
                const bool ret = arm_pc == 0x510194E4u;
                if (batch || slot || result || ret || arm_pc == 0x510194C0u) {
                    const ArmCore* log_arm = dynamic_cast<const ArmCore*>(core);
                    const u32 r9 = log_arm != nullptr ? log_arm->r[9] : 0u;
                    const u32 r10 = log_arm != nullptr ? log_arm->r[10] : 0u;
                    static u32 module_starts = 0;
                    static u32 module_slots = 0;
                    static u32 module_batches = 0;
                    static u32 module_returns = 0;
                    if (batch) {
                        ++module_batches;
                        ZLB_LOG_INFO("machine",
                                     "module batch #%u records=0x%08X uids=0x%08X count=%u core=%d",
                                     module_batches, log_arm != nullptr ? log_arm->r[0] : 0u,
                                     log_arm != nullptr ? log_arm->r[1] : 0u,
                                     log_arm != nullptr ? log_arm->r[2] : 0u, i);
                    } else if (slot) {
                        const u32 uid = arm_pc == 0x51001326u ? r9 : r10;
                        ++module_slots;
                        ZLB_LOG_INFO("machine", "module slot #%u uid=0x%08X next=0x%08X at 0x%08X core=%d",
                                     module_slots, uid, log_arm != nullptr ? log_arm->r[5] : 0u, arm_pc, i);
                    } else if (ret) {
                        ++module_returns;
                        ZLB_LOG_INFO("machine", "module return #%u core=%d (uids=0x%08X)", module_returns, i,
                                     log_arm != nullptr ? log_arm->r[5] : 0u);
                    } else if (result) {
                        const u32 result_value = log_arm != nullptr ? log_arm->r[0] : 0u;
                        ZLB_LOG_INFO("machine", "module result 0x%08X at 0x%08X core=%d (uids=0x%08X)",
                                     result_value, arm_pc, i, log_arm != nullptr ? log_arm->r[5] : 0u);
                        if ((result_value & 0x80000000u) != 0u) {
                            // Dump the recent PC path (deduplicated, newest first) so the
                            // code that produced the error is identifiable.
                            static u32 dumped[64];
                            u32 dumped_count = 0;
                            const unsigned trace_core = i < kArmTraceCores ? i : 0u;
                            for (u32 back = 0; back < kArmTraceSize && dumped_count < 64u; ++back) {
                                const u32 pc_value =
                                    g_arm_trace_ring[trace_core]
                                                    [(g_arm_trace_pos[trace_core] - 1u - back) &
                                                     (kArmTraceSize - 1u)];
                                bool known = false;
                                for (u32 k = 0; k < dumped_count; ++k) {
                                    if (dumped[k] == pc_value) { known = true; break; }
                                }
                                if (!known) dumped[dumped_count++] = pc_value;
                            }
                            for (u32 k = 0; k < dumped_count; ++k) {
                                ZLB_LOG_INFO("machine", "module fail path [%u] 0x%08X", k, dumped[k]);
                            }
                            // Every NSKBL address executed during this start, so the
                            // resolution path is visible even when a flush loop runs.
                            const unsigned tail_core = i < kNskblRingCores ? i : 0u;
                            ZLB_LOG_INFO("machine", "module fail uid ring (oldest to newest):");
                            for (u32 back = kUidRingSize; back > 0u; --back) {
                                const u32 slot = (g_uid_ring_pos - back) & (kUidRingSize - 1u);
                                if (g_uid_ring_site[slot] != 0u) {
                                    ZLB_LOG_INFO("machine", "module fail uid 0x%08X at site 0x%08X -> 0x%08X",
                                                 g_uid_ring_value[slot], g_uid_ring_site[slot],
                                                 g_uid_ring_result[slot]);
                                }
                            }
                            ZLB_LOG_INFO("machine", "module fail seq (core %u, oldest to newest):",
                                         tail_core);
                            for (u32 back = kNskblRingSize; back > 0u; --back) {
                                const u32 pc_value =
                                    g_nskbl_ring[tail_core]
                                                [(g_nskbl_pos[tail_core] - back) &
                                                 (kNskblRingSize - 1u)];
                                if (pc_value != 0u) {
                                    ZLB_LOG_INFO("machine", "module fail seq 0x%08X", pc_value);
                                }
                            }
                            ZLB_LOG_INFO("machine", "module fail set: %u NSKBL addresses", g_pc_set_count);
                            for (u32 k = 0; k < kPcSetSlots; ++k) {
                                if (g_pc_set_used[k] != 0u) {
                                    ZLB_LOG_INFO("machine", "module fail addr 0x%08X", g_pc_set[k]);
                                }
                            }
                        }
                    } else {
                        ++module_starts;
                        any_module_start = true;
                        starts_seen = module_starts;
                        pc_set_reset();
                        for (unsigned c = 0; c < kNskblRingCores; ++c) g_nskbl_pos[c] = 0;
                        ZLB_LOG_INFO("machine", "module start #%u uid=0x%08X core=%d", module_starts,
                                     log_arm != nullptr ? log_arm->r[0] : 0u, i);
                    }
                }
            }
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
        }
        if (stop_slice) break;
    }

    if (kermit_) kermit_->tick(static_cast<u64>(budget_.arm));
    // Diagnostic (ZLB_KBL_PARAM_LOG=1): follow the ARM-visible copy of the KBL record
    // (PA 0x1C0/0x1C4 in the SPAD32K window) across the run.  The debugger's `mem`
    // reads the same bus, so a mismatch here means the word really is overwritten
    // after the builder, not that two views disagree (docs/KBL.md 7.1.34).
    static const bool param_log = [] {
        const char* value = std::getenv("ZLB_KBL_PARAM_LOG");
        return value != nullptr && value[0] != '0';
    }();
    if (param_log && arm_bus_ != nullptr) {
        static u64 last_slice = ~0ull;
        const u64 slice = boot_.steps_in_stage;
        if (last_slice == ~0ull || slice >= last_slice + 2048u) {
            last_slice = slice;
            ZLB_LOG_INFO("machine", "kbl param follow: slice=%llu PA 0x1C0=0x%08X PA 0x1C4=0x%08X",
                         static_cast<unsigned long long>(slice), arm_bus_->read32(0x1C0u),
                         arm_bus_->read32(0x1C4u));
        }
    }
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
