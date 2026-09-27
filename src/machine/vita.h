// zeliboba - the PlayStation Vita machine.
//
// Three processors live on the board and each has its own physical address
// space, bridged by shared hardware blocks:
//
//   CMeP  ("F00D")     MeP-c5, 128 KiB RAM window at 0x40000, keyring/Bigmac/
//                      Bignum/keyring/eMMC-crypto engines, SC bridge to Ernie
//   Kermit (SoC)       ARM Cortex-A9 MPCore, 64 MiB DRAM window at 0x80000000
//   Ernie  (syscon)    Renesas RL78, 1 MiB flash, SFR peripherals, eMMC host
//
// The machine owns the buses, the devices, the cores and the boot chain.
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "cpu/cpu.h"
#include "loader/keys.h"
#include "loader/loader.h"
#include "machine/bootchain.h"

namespace zlb {

class CmepBlock;      // hw/cmep.h
class ErnieBlock;     // hw/syscon.h
class EmmcCard;       // hw/emmc.h
class EmmcHost;       // hw/emmc.h
class KermitBlock;    // hw/soc.h

/// Per-core instruction budget for one scheduler slice.
struct CoreBudget {
    int arm = 256;
    int cmep = 256;
    int rl78 = 128;
};

struct VitaConfig {
    /// Paths of the fitted parts. Relative paths are resolved against the
    /// workspace root (the folder that contains `zeliboba`).
    std::string emmc_image;          ///< reconstructed eMMC image
    std::string first_loader;        ///< CMeP first loader (dumps/vita_prototype_bootrom.bin)
    std::string syscon_firmware;     ///< Ernie dump (ernie-master/USS-1001.bin)
    std::string fs_root;             ///< extracted firmware tree (Vita_104_Firmware/Out/fs)

    bool run_syscon_firmware = true; ///< false: use the functional SC model only
    bool strict = false;             ///< halt on unmapped accesses
    u32 first_loader_base = 0x5C000;
    u64 emmc_size = 512ull * 1024 * 1024;  ///< size of a reconstructed image
    bool rebuild_emmc = true;        ///< build the image when it is missing
    /// Seed the emulated CMeP key table (0xE0066000) with a development key and
    /// re-sign the staged second loader, so the first loader's RSA verification can
    /// succeed. The console's real key table is not in any dump we have; see
    /// docs/KBL.md and src/machine/bootkeys.h. The eMMC image is never modified.
    bool provision_keys = true;
};

/// Board level physical addresses shared by the boot chain and the machine.
namespace board {
constexpr u32 kSharedSramBase = 0x1F000000;   ///< visible to both ARM and CMeP
constexpr u32 kSharedSramSize = 0x00040000;   ///< 256 KiB
/// The ARM's low window aliases the power scratchpad (wiki Physical_Memory:
/// "0x00000000 0x40000 ARM Boot. By default, alias of physical address 0x1F000000
/// i.e. ScePower scratchpad").  The window is 256 KiB, which is what puts the
/// SKBL's vector page at PA 0x16100 inside it.
constexpr u32 kArmBootWindowSize = 0x00040000;
/// Vector page kernel_boot_loader points VBAR at (the value comes out of its own
/// platform table).  It is *not* covered by the table the KBL builds for itself
/// (that one maps only VA 0x0000-0x7FFF to its scratch copy), so the stage that
/// runs before it - the wiki's "SKBL Reset Vector" written by the second loader -
/// is what plants the vector table there.  See Vita::satisfy_arm_boot_fault().
constexpr u32 kArmVectorPage = 0x00016100;
/// Wiki's FW 3.60 Secure DRAM layout: "0x40000000 0xC0 SKBL Reset Vector (ARM
/// entry!)".  The KBL's own ELF carries exactly that 0xC0-byte segment at vaddr 0
/// (8 vectors + the pointer table: vector[0] is `ldr pc,[pc,#0x18]` and the first
/// pointer is 0x40020000, i.e. the KBL entry).
constexpr u32 kSkblResetVectorDram = 0x40000000;
/// CMeP 128 KiB SRAM (wiki: "0x00800000 0x20000 S Cmep 128KiB SRAM. Stores
/// second_loader, secure_kernel and Secure Modules").  The ARM sees it mirrored at
/// 0x00040000-0x0005FFFF ("MeP boot. Mirror of physical address 0x00800000").
constexpr u32 kArmMepBootBase = 0x00040000;
constexpr u32 kArmMepBootSize = 0x00020000;
constexpr u32 kCmepStackTop = 0x00060000;     ///< reset value of the CMeP $0/stack
constexpr u32 kSecondLoaderStaging = 0x00040000;  ///< where the first loader stages stage 2
constexpr u32 kCmepRamBase = 0x00040000;
constexpr u32 kCmepRamSize = 0x00020000;
constexpr u32 kFirstLoaderBase = 0x0005C000;  ///< CMeP first loader entry / window
/// Service entry point inside the first loader's window that the second loader
/// calls (absolute `jmp`) once its own work is done.  The routine itself is not
/// in any dump - see Vita::serve_cmep_service_call().
constexpr u32 kFirstLoaderServiceEntry = 0x0005FF00;
/// Where `secure_kernel.enp` is linked and staged: its first bytes are a MeP
/// vector table of absolute jumps to 0x800100/0x80028C/0x8002A6/0x80037A, i.e.
/// the image runs from the CMeP's private 2 MiB window at 0x800000.
constexpr u32 kCmepSecureKernelBase = 0x00800000;
/// Where the ARM boot ROM stages the second-loader container.  This must *not* be
/// the shared SRAM: the wiki's `KBL Param` page pins the SceKblParam DIP-switch
/// field at physical 0x1F000080, i.e. the record lives at 0x1F000040 inside
/// SPAD32K, and staging the container there overwrote it (see kKblParamBase).
constexpr u32 kSecondLoaderStagingDram = 0x407C0000;  ///< inside the shared DRAM window
/// SceKblParam: 0x100-byte record the second loader builds and the secure/non-secure
/// kernel boot loaders read (wiki KBL_Param).  Two copies are documented: one in the
/// power scratchpad (its DIP-switch field is pinned at 0x1F000080, so the record is
/// at 0x1F000040) and one in secure DRAM ("0x4001FD00 - SceKblParam with magic not
/// set" in the wiki's FW 3.60 Secure DRAM layout, right below SKBL).
constexpr u32 kKblParamBase = 0x1F000040;
constexpr u32 kKblParamDram = 0x4001FD00;
constexpr u32 kKblParamSize = 0x100;
constexpr u32 kKblParamMagic = 0xCBAC03AAu;
/// Where the two SLB2 kernel-module images are staged.  The wiki's Physical_Memory
/// says the CMeP's 128 KiB SRAM (0x00800000, which the ARM sees mirrored at
/// 0x00040000) "Stores second_loader, secure_kernel and Secure Modules", while the
/// FW 3.60 secure-DRAM table gives the same modules offsets 0x500/0x9B00 inside the
/// secure image layout.  The second reading is what the kernel boot loader consumes
/// through the MeP window, so the modules are staged at those offsets from the SRAM
/// base and the parameter records the window addresses.
constexpr u32 kKprxAuthSmStaging = 0x00800500;  // CMeP SRAM base (0x00800000) + 0x500
constexpr u32 kProgRvkStaging = 0x00809B00;     // CMeP SRAM base (0x00800000) + 0x9B00
}  // namespace board

class Vita {
public:
    Vita();
    ~Vita();

    Vita(const Vita&) = delete;
    Vita& operator=(const Vita&) = delete;

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /// Build the board: memory maps, devices, cores. Idempotent.
    void build(const VitaConfig& config = {});
    const VitaConfig& config() const { return config_; }
    VitaConfig& config() { return config_; }

    /// Power-on reset. `cold` also clears RAM and reloads the fitted parts.
    void reset(bool cold = true);

    /// Load the CMeP first loader, the syscon firmware and the eMMC image.
    bool fit_parts();

    // ------------------------------------------------------------------
    // Processors and buses
    // ------------------------------------------------------------------

    Bus& arm_bus() { return *arm_bus_; }
    Bus& cmep_bus() { return *cmep_bus_; }
    Bus& syscon_bus() { return *syscon_bus_; }

    Cpu* arm() { return arm_; }
    Cpu* cmep() { return cmep_.get(); }
    Cpu* syscon() { return syscon_; }
    Cpu* core(Arch arch);

    /// The Kermit is a quad core Cortex-A9 MPCore. kernel_boot_loader runs the
    /// whole cluster: it reads MPIDR to decide whether a core is the boot core
    /// (the per-core MMU table setup in 0x40020Bxx) and synchronises all four at a
    /// barrier (0x4003B384) that decrements a counter and waits for the other
    /// arrivals. A single core deadlocks there, so the machine models four.
    static constexpr int kArmCoreCount = 4;
    Cpu* arm_core(int index) {
        return (index >= 0 && index < kArmCoreCount) ? arm_cores_[static_cast<size_t>(index)].get()
                                                     : nullptr;
    }
    const Cpu* arm_core(int index) const {
        return (index >= 0 && index < kArmCoreCount)
                   ? arm_cores_[static_cast<size_t>(index)].get()
                   : nullptr;
    }

    CmepBlock& cmep_block() { return *cmep_block_; }
    ErnieBlock& ernie() { return *ernie_; }
    EmmcCard& emmc() { return *emmc_; }
    KermitBlock& kermit() { return *kermit_; }

    // ------------------------------------------------------------------
    // Scheduling
    // ------------------------------------------------------------------

    CoreBudget& budget() { return budget_; }

    /// Run one scheduler slice: each core gets its budget of instructions.
    void run_slice();

    /// Run for `seconds` of emulated time (approximate, core-clock based).
    void run_for(double seconds);

    /// Total instructions executed by every core.
    u64 total_instructions() const;

    /// Emulated time in seconds since reset, derived from the ARM cycle count.
    double emulated_seconds() const;

    // ------------------------------------------------------------------
    // Boot chain
    // ------------------------------------------------------------------

    /// Loads the firmware 1.04 boot chain into the machine and returns a
    /// human readable plan of what will run.
    std::vector<std::string> plan_boot();

    /// Advance the boot chain state machine (mailbox handshakes, ARM release).
    void poll_boot_chain();

    const BootStatus& boot_status() const { return boot_; }
    BootStage stage() const { return boot_.stage; }

    /// Move to a specific stage by hand; used by the debugger's `stage` command
    /// and by the tools to start the emulator part-way through the boot chain.
    bool enter_stage(BootStage stage);

    /// True once the ARM kernel (`os0`) entry point has been started.
    bool kernel_started() const;
    /// True once the kernel has performed its first SVC/IRQ after entry.
    bool kernel_running() const;

    std::string status_line() const;
    std::string boot_report() const;

    // ------------------------------------------------------------------
    // Media
    // ------------------------------------------------------------------

    /// Attach an eMMC image file.
    bool attach_emmc(const std::string& path);
    /// Build the eMMC image from the extracted firmware when it is missing.
    bool rebuild_emmc_if_missing();

    // ------------------------------------------------------------------
    // Diagnostics
    // ------------------------------------------------------------------

    /// Signatures of the hardware blocks that have been touched since reset.
    const std::vector<std::string>& milestones() const { return milestones_; }
    void add_milestone(const std::string& text);

    /// Records a boot-chain event; visible in the debugger and in the report.
    void log_event(const std::string& text);

private:
    void build_buses();
    void build_devices();
    void build_cores();
    void wire_bridges();

    /// Map registers/memory shared by more than one core.
    void build_shared_windows();

    /// The C++ model of the ARM boot ROM step: read SLB2 from the eMMC boot
    /// partition, stage `second_loader.enp` in the shared SRAM and ring the CMeP
    /// mailbox. Returns false when the image could not be produced.
    bool arm_boot_rom_stage_second_loader();

    /// Start the ARM on the kernel boot loader contained in SLB2.
    bool start_arm_kernel_boot_loader();

    /// Load the kernel (`os0`) into DRAM and jump to it.
    bool start_kernel();

    /// Load the decrypted second loader directly at 0x40000 (fallback / stage cmd).
    bool load_second_loader_direct();

    /// Boot the CMeP's *secure kernel* directly: the second loader reads
    /// `secure_kernel.enp` from SLB2 and jumps into it once its SCE validation
    /// chain has passed (docs/BOOT.md 4).  The stage command loads the decrypted
    /// payload (`Out/SLB2_dec/secure_kernel.bin`, a MeP image) at the same
    /// 0x40000 window so the next link of the boot sequence can be exercised
    /// without the second loader's per-console checks.
    bool load_cmep_secure_kernel();

    /// Copy the CMeP scratch (shared boot SRAM, first 32 KiB) into the ARM's low
    /// window at PA 0, which is where the wiki says the scratch is mirrored.
    bool mirror_cmep_scratch_to_arm();

    /// Write the SceKblParam record (wiki KBL_Param) at board::kKblParamBase.
    bool build_kbl_param();
    /// Stage `bytes` into DRAM through the CMeP bus (shared backing store).
    bool stage_in_dram(u32 address, const std::vector<u8>& bytes);

    /// Development substitution for the page tables the stage *before*
    /// kernel_boot_loader leaves behind.  The ARM boot ROM / the second loader's
    /// 0xC0-byte reset vector are not in the dumps, so the model cannot reproduce
    /// their tables; the KBL builds its own (VA 0x0000-0x7FFF -> its DRAM scratch
    /// copy) and then faults on its first access to the low window (VA 0x40000)
    /// and on its own vector page (VBAR 0x16100).  Installing the low window as an
    /// identity mapping in the KBL's own L2 and retrying is what the inherited
    /// mapping would have provided.  Returns true when the fault was satisfied.
    bool satisfy_arm_boot_fault(u32 core, u32 va, bool write, bool fetch);
    u64 boot_fault_fixes_ = 0;

    /// CMeP pre-instruction hook, installed on the MeP core: intercepts the first
    /// loader's service entry point (0x5FF00) that the second loader calls at the
    /// end of its work (see docs/KBL.md, round 39).
    bool cmep_pc_hook(u32 pc);
    /// Handle a hit recorded by cmep_pc_hook() at a slice boundary.
    bool serve_cmep_service_call();
    bool cmep_service_pending_ = false;
    /// True while the CMeP runs the secure kernel image (set by
    /// load_cmep_secure_kernel, cleared when its "done" jump is handled).
    bool secure_kernel_active_ = false;
    /// Set when the secure kernel re-enters the 0x40000 window (its "done" path).
    bool secure_kernel_done_ = false;
    /// SceKblParam inputs collected while staging the SLB2 images.
    u32 secure_kernel_size_ = 0;
    u32 kprx_auth_sm_pa_ = 0;
    u32 kprx_auth_sm_size_ = 0;
    u32 prog_rvk_pa_ = 0;
    u32 prog_rvk_size_ = 0;

    EmmcCard& emmc_ref() { return *emmc_; }

    VitaConfig config_;
    CoreBudget budget_;

    std::unique_ptr<Bus> arm_bus_;
    std::unique_ptr<Bus> cmep_bus_;
    std::unique_ptr<Bus> syscon_bus_;

    std::array<std::unique_ptr<Cpu>, kArmCoreCount> arm_cores_{};
    Cpu* arm_ = nullptr;  ///< arm_cores_[0]: the boot core, and the debugger's ARM.
    std::unique_ptr<Cpu> cmep_;
    Cpu* syscon_ = nullptr;  // owned by ErnieBlock

    SceKeys keys_;

    std::unique_ptr<CmepBlock> cmep_block_;
    std::unique_ptr<ErnieBlock> ernie_;
    std::unique_ptr<EmmcCard> emmc_;
    std::unique_ptr<KermitBlock> kermit_;

    std::vector<std::string> milestones_;
    std::vector<std::string> events_;
    BootStatus boot_;
    bool built_ = false;
    bool kernel_started_ = false;
    bool kernel_running_ = false;

    /// Shared boot SRAM: the ARM boot ROM stages the second loader here and the
    /// CMeP reads it. Both buses alias the same host buffer at 0x1F000000.
    std::vector<u8> shared_sram_;
    /// CMeP private window (2 MiB at 0x00800000); its first 128 KiB is the SRAM the
    /// ARM mirrors at 0x40000.
    std::vector<u8> cmep_priv_;
    /// Main DRAM (the module's 128 MiB physical window): the CMeP stages the
    /// secure kernel and the kernel boot loader in it and the ARM runs them from
    /// there, so both buses alias this one buffer at 0x40000000.
    std::vector<u8> dram_;
    u32 second_loader_pa_ = 0;
    u32 second_loader_entry_ = 0;
    u32 kernel_entry_ = 0;
    u32 kbl_entry_ = 0;
};

/// Resolve a path relative to the workspace root (ZLB_WORKSPACE_DIR) unless it
/// is already absolute.
std::string resolve_workspace_path(const std::string& path);

/// Path of the workspace root the emulator was built against.
const std::string& workspace_root();

}  // namespace zlb
