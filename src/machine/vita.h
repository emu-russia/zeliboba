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
#include <cstddef>
#include <memory>
#include <set>
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
/// the shared SRAM (SPAD32K at 0x1F000000): the second loader builds SceKblParam
/// at 0x1F000100 inside it (see kKblParamBase) and the kernel boot loader reads
/// the record from there through the ARM's physical alias, so staging the
/// container over the scratchpad would destroy it.
constexpr u32 kSecondLoaderStagingDram = 0x407C0000;  ///< inside the shared DRAM window
/// SceKblParam: the 0x100-byte record the second loader builds at physical
/// 0x1F000100 inside SPAD32K and the secure/non-secure kernel boot loaders read
/// (wiki KBL_Param).  The base is what the loader's own builder uses - the
/// disassembly at 0x41B4A loads `0x1F000100` and the record it writes matches
/// the wiki's field offsets (DRAM base at +0x60, kprx_auth_sm at +0x90, ...),
/// which was confirmed on a real run (docs/SYSCON.md 8.15).  The earlier
/// 0x1F000040 came from reading the wiki's "fallback DIP switch buffer at
/// physical 0x80" as the record's DIP field; it is 0xC0 too low and overlapped
/// the loader's own header.
constexpr u32 kKblParamBase = 0x1F000100;
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

    /// Optional per-instruction PC observation, used by the debugger to make
    /// breakpoints exact without giving up the fast budget.  A slice runs
    /// `budget().arm` instructions per Kermit core, so a breakpoint tested only
    /// at the slice boundary would be silently skipped; this hook is called with
    /// the PC *about to execute* and, when it returns true, the slice ends
    /// immediately with that instruction still pending.  The CMeP and Ernie
    /// cores run a whole budget per slice, so they are not covered - a
    /// breakpoint there is still only exact with a one-instruction budget.
    std::function<bool(Arch arch, int core, u32 pc)> pc_hook;

    /// True when `pc_hook` ended the last slice; cleared by clear_pc_hook_stop().
    bool pc_hook_stopped() const { return pc_hook_stopped_; }
    void clear_pc_hook_stop() { pc_hook_stopped_ = false; }
    Arch pc_hook_arch() const { return pc_hook_arch_; }
    int pc_hook_core() const { return pc_hook_core_; }
    u32 pc_hook_pc() const { return pc_hook_pc_; }

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
    /// Substitution (round 396): write the ELF form of the os0 modules onto the card, because
    /// the workspace only holds their decrypted "SCE\0" form while NSKBL validates "\x7FELF"
    /// (gated by ZLB_OS0_ELF=1, see docs/NSKBL.md rounds 394/395).
    void apply_os0_elf_form();

    // ------------------------------------------------------------------
    // Diagnostics
    // ------------------------------------------------------------------

    /// Signatures of the hardware blocks that have been touched since reset.
    const std::vector<std::string>& milestones() const { return milestones_; }
    void add_milestone(const std::string& text);

    /// Records a boot-chain event; visible in the debugger and in the report.
    void log_event(const std::string& text);

    // ------------------------------------------------------------------
    // ARM PC coverage (round 167 tooling)
    // ------------------------------------------------------------------

    /// True when the address's coverage bit is set (see arm_cov_mark).
    bool arm_cov_executed(u32 addr) const;
    u32 arm_cov_base() const { return arm_cov_base_; }
    u32 arm_cov_size() const { return arm_cov_size_; }
    u32 arm_cov_granularity() const { return arm_cov_gran_; }
    bool arm_cov_armed() const { return !arm_cov_bits_.empty(); }
    /// Raw bitmap (for `cov save`), one bit per arm_cov_granularity() bytes.
    const std::vector<u8>& arm_cov_bytes() const { return arm_cov_bits_; }

    // ------------------------------------------------------------------
    // CMeP (MeP) PC coverage
    // ------------------------------------------------------------------

    /// Same map for the security core: ZLB_MEP_COV=1 arms it, and the debugger's
    /// `cov mep` prints it.  The MeP side has no `c`-command of its own, so this is
    /// the only way to answer "which part of first_loader / second_loader /
    /// secure_kernel actually ran".
    bool mep_cov_executed(u32 addr) const;
    u32 mep_cov_base() const { return mep_cov_base_; }
    u32 mep_cov_size() const { return mep_cov_size_; }
    u32 mep_cov_granularity() const { return mep_cov_gran_; }
    bool mep_cov_armed() const { return !mep_cov_bits_.empty(); }
    const std::vector<u8>& mep_cov_bytes() const { return mep_cov_bits_; }

private:
    void build_buses();
    /// Round 191: optional GPU/display self-test (ZLB_GPU_SELFTEST=1) that puts a
    /// synthetic frame into the display controller so the SDL3 Panel tab has an
    /// image while the kernel display driver is out of reach.
    void apply_gpu_selftest();    void build_devices();
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

    /// Run the non-secure kernel boot loader (NSKBL).  The secure kernel boot
    /// loader stages the ARZL-compressed NSKBL at PA 0x50000000 and decodes it to
    /// 0x51000000 with its *own* routines (sceArlzDecode 0x4003C330 and
    /// sceArlzArmFilter 0x4003CB40, both inside kernel_boot_loader.self); this
    /// stage calls those two functions from the model - nothing about the decode
    /// is reimplemented - and then enters NSKBL in the non-secure world.
    bool start_nskbl();

    /// Call one firmware routine on arm0 with the AAPCS argument registers and a
    /// scratch stack.  Returns the routine's r0; `ok` reports whether it returned
    /// (rather than exhausting the step budget).
    u32 arm_call(u32 address, u32 a0, u32 a1, u32 a2, u32 a3, bool* ok);

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

    /// Parse ZLB_PCTRAP into pc_trace_lo_/hi_ (call once, before the ARM runs).
    void configure_arm_pc_trace();

    /// Diagnostic: every time an ARM core is about to execute an address inside
    /// ZLB_PCTRAP=<lo>-<hi> (hex), log the PC, the core, the register file and the
    /// instruction word to stderr.  The KBL's boot-init walk (0x4002C594) is one
    /// long sequence of calls into helpers that are only reachable through
    /// pointers, so a breakpoint (which stops on the first hit) cannot show the
    /// order in which the steps run; this can, and it does not change the run.
    /// Returns true when the PC was logged (the run continues either way).
    bool trace_arm_boot_pc(u32 core, u32 pc);
    u32 pc_trace_lo_ = 0;
    u32 pc_trace_hi_ = 0;
    bool pc_trace_enabled_ = false;

    /// Round 160: how many times the cluster was woken out of an all-WFE sleep
    /// (see the watchdog in run_slice: the loader's barrier spins also wait on the
    /// timer interrupt, which this model does not deliver to a halted core).
    u64 wfe_wakeups_ = 0;
    /// Round 160: how many parked barrier counters the same watchdog put back to
    /// the value the waiting core's loop tests.
    u64 barrier_unstuck_ = 0;

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

    /// Development substitution for the data the stage before kernel_boot_loader
    /// is supposed to leave in the ARM's scratchpad.  The KBL builds a physical
    /// memory partition (VA 0x51C0) with a per-core cache of 0x1000-byte blocks,
    /// but nothing in the KBL ever *adds* memory to it, and every allocation asks
    /// for a flag (0x10) that forbids carving a fresh block - so the cache stays
    /// empty for the whole run, every allocation returns 0x80020005, the object
    /// manager (boot context +0x8C) is never created and all class registrations
    /// fail with 0x80024501 (docs/KBL.md, round 51).  On hardware the partition
    /// arrives pre-populated, so this hook lets the allocator take its own carving
    /// path at 0x40032366 instead of the cache-only failure at 0x4003234E.
    /// Returns true when it redirected the PC (the caller then skips stepping).
    bool satisfy_arm_boot_pc(u32 core, u32 pc);
    u64 boot_pc_fixes_ = 0;

    /// Development substitution: hand the KBL's own partition the block cache it
    /// never fills itself (see the comment on satisfy_arm_boot_pc).  Called from
    /// the PC hook when an allocation is about to run with the cache-only flag and
    /// a class whose cache slot is empty; writes `blocks` 0x1000/0x2000-byte
    /// blocks taken from the region's free end into that slot (target/step 7, as
    /// the partition's own init leaves it).  Off only with ZLB_NO_SUBSTITUTION=1,
    /// tuned with ZLB_PART_BLOCK_CACHE=<n> (default 4).
    bool supply_kbl_partition_block(u32 core, u32 pool_va, u32 size);
    /// Create the per-class table the SceUID registration walks through the global
    /// 0x400B291C; the loader never writes that global itself (round 93).
    bool supply_kbl_class_table(u32 core);
    /// Development substitution (round 366): NSKBL's storage driver addresses its
    /// device through a hard-wired object at VA 0x240 (`str r2,[r0]` with r2 = 0x240
    /// in the request builder at 0x5101FDBC).  That object lives in the low window
    /// and nothing in the boot chain ever writes it - measured: zero writes over its
    /// first page in a whole run - so the driver's first data access dereferences a
    /// NULL function pointer (`blx [0]` out of `[device+0x24A0]`, docs/NSKBL.md
    /// 10.66).  This fills the fields the driver is measured to read: the SDIF
    /// register base at +0x2430 (0xE0B00000, the value the working ADMA command
    /// writer uses), a pointer at +0x24A0 to a one-entry dispatch table holding the
    /// driver's own submit routine (0x51022604, which takes the request in r0, writes
    /// the SDIF command registers and returns 0), and a free node on the device's
    /// completion list at +0x2400 (VA 0x2640) - the list the driver's wait pops and
    /// which is empty in every measured run (docs/NSKBL.md 10.69).
    bool supply_nskbl_device_object(u32 core);
    /// One-shot latch for the substitution above (round 366).
    bool nskbl_device_supplied_ = false;

    /// Development substitution (round 378): the device method itself.  Round 377
    /// measured that *no* image in the workspace contains a constructor for the
    /// device object (NSKBL has no instruction that materialises the SDIF base, and
    /// the table at 0x51029FC0 is read only by the secure KBL), so the method the
    /// driver dispatches to at 0x5101D6E8 cannot be guest code.  When
    /// ZLB_NSKBL_SERVICE=1 this intercepts the `blx r1` and performs the storage
    /// operation the request describes in C++: the command index is `[request+8]`
    /// (`0x12` = CMD18), the SDIF argument `[request+0x0C]` (a byte offset), the
    /// buffer `[request+0x20]`, block size and count `[request+0x24]`/`[request+0x26]`
    /// and the ADMA2 table `[request+0x7C]`; the transfer is read from the eMMC card
    /// into the descriptor targets, the progress fields `[request+0x1B0]`/
    /// `[request+0x1B8]` are filled in and r0 = 0 (success) is returned.
    bool serve_nskbl_device_call(u32 core);
    u32 nskbl_service_calls_ = 0;
    u32 nskbl_service_fail_logs_ = 0;
    /// Device completion sequence (round 379) - see serve_nskbl_device_call().
    u32 nskbl_service_state_ = 0;
    std::set<u64> nskbl_service_seen_;
    /// Nodes handed back to the device's pool, so the driver's wait keeps popping.
    u32 nskbl_service_completions_ = 0;
    /// Log cap for the round-388-391 submission experiments.
    u32 nskbl_async_bit_logs_ = 0;
    /// Experiment (round 101): free-chunk marker plus page-table entry for one size
    /// class, so the loader own carve path hands the block out.
    bool supply_kbl_carve_state(u32 core, u32 pool_va, u32 size);
    /// Substitution (round 108): one fresh zeroed page out of the partition region
    /// for a class instance whose base field `[obj+0x14]` is the 0xFFFFFFFF sentinel.
    bool supply_kbl_instance_block(u32 core, u32& out_block);
    /// Cursor (arena byte offset, walks up) for `supply_kbl_instance_block`
    /// (round 114: the instance pages come from the dedicated VA 0x01100000 arena).
    u32 instance_block_next_ = 0;
    u32 instance_blocks_supplied_ = 0;
    /// How many times the instance base field `[obj+0x14]` was filled with the arena
    /// VA at the getter entry (round 114).
    u32 instance_base_fixes_ = 0;
    /// Cursor (arena byte offset) for the region-allocator tree-node arena
    /// (round 124: fresh red-black-tree nodes carved out of VA 0x01200000).
    u32 tree_node_next_ = 0;
    /// Number of tree nodes handed out by the region-allocator rebuild (round 124).
    u32 tree_nodes_supplied_ = 0;
    /// Number of SceSysmem heap-lookup substitutions (round 134).
    u32 heap_lookup_fixes_ = 0;
    /// Cursor (arena byte offset) for the SceSysmem lookup-object arena (round 134).
    u32 lookup_object_next_ = 0;
    /// Number of heap-pointer routing substitutions (round 135).
    u32 heap_route_fixes_ = 0;
    /// Number of barrier-decrement substitutions (round 136 diagnostics).
    u32 barrier_decrements_ = 0;
    /// Per-core "old" arrival value recorded by the barrier-decrement substitution
    /// and replayed at 0x4003B3BA so the phase decision is not clobbered by the
    /// shared initial stack (round 140).
    u16 barrier_old_[kArmCoreCount] = {};
    /// Last ARM pc the boot-chain hook saw, per core (round 109 diagnostics).
    u32 last_arm_pc_[kArmCoreCount] = {};
    /// How many times the allocator entry got the heap object's cookie field stamped
    /// (round 110 substitution).
    u32 cookie_stamps_ = 0;
    /// Lines already emitted by the ZLB_KBL_TRACE_PC diagnostic (round 111).
    u32 trace_pc_hits_ = 0;
    /// Set once the ZLB_ARM_TRACE_RING dump has been printed (round 214).
    bool trace_ring_pc_dumped_ = false;
    /// Lines already emitted by the fatal-stub diagnostic (round 111): the stubs end
    /// in `b .`, so without the cap the log grows without bound.
    u32 fatal_stub_hits_ = 0;
    /// Lines already emitted by the unfixable-fault diagnostic (round 112).
    u32 fault_trace_hits_ = 0;
    /// Lines already emitted by the class-constructor diagnostic (round 114).
    u32 ctor_trace_hits_ = 0;
    /// Lines already emitted by the contended-spinlock diagnostic (round 115).
    u32 lock_trace_hits_ = 0;
    /// Lines already emitted by the object-manager lock diagnostic (round 116).
    u32 objmgr_trace_hits_ = 0;
    /// Lines already emitted by the SceUID lock diagnostic (round 119).
    u32 sceuid_trace_hits_ = 0;
    /// Lines already emitted by the region-tree diagnostic (round 120).
    u32 tree_trace_hits_ = 0;
    /// Lines already emitted by the region-tree stamp substitution (round 121).
    u32 tree_fix_hits_ = 0;
    /// Lines already emitted by the range-check diagnostic (round 123).
    u32 rangechk_trace_hits_ = 0;
    /// Page index (inside the partition region) the carve experiment hands out next.
    u32 carve_page_next_ = 0;
    u32 partition_blocks_per_class_ = 4;
    u32 partition_supplied_ = 0;
    u32 class_tables_supplied_ = 0;
    /// VA of the class table the model created (round 111): the loader zeroes the
    /// pointer slot after the first supply, so it is restored from this instead of
    /// allocating a second table.
    u32 class_table_va_ = 0;
    u32 partition_block_next_[2] = {0x00070000u, 0x000B0000u};  ///< cursor + class-table cursor
    u32 partition_region_base_ = 0x40000000u;
    u32 partition_region_size_ = 0x00300000u;
    /// The KBL's 0xC0-byte exception vector table (its ELF vaddr=0 segment), kept so
    /// the model can restore it at the mapping the KBL installs for VA 0x16100
    /// (round 94).
    std::vector<u8> kbl_vectors_;
    bool kbl_vectors_restored_ = false;
    /// Round 163: how often SKBL's runtime vector page was mirrored into the low
    /// boot page, which is where the MMU-off monitor fetch (MVBAR 0x16140) reads.
    u32 kbl_vector_mirrors_ = 0;
    /// Round 164: ZLB_KBL_TRACE_ZERO hit counter (jumps to pc == 0).
    u32 trace_zero_hits_ = 0;
    /// Round 164: how many secondary cores got their own stage stack.
    u32 core_stack_biases_ = 0;
    /// Round 165: NSKBL spinlock acquires skipped at a bogus (low) lock address.
    u32 nskbl_lock_skips_ = 0;
    /// Round 167 tooling: ARM PC coverage - one bit per `arm_cov_gran_` bytes of the VA
    /// range starting at `arm_cov_base_`.  ZLB_ARM_COV=1 arms it and the debugger's
    /// `cov` command prints the executed/unexecuted map, so "which NSKBL function never
    /// ran" takes seconds instead of a breakpoint session.
    u32 arm_cov_base_ = 0x51000000u;
    u32 arm_cov_size_ = 0x00040000u;   ///< 256 KiB of VA space (covers the NSKBL image)
    u32 arm_cov_gran_ = 2u;            ///< bytes per bit (2 = one Thumb instruction)
    std::vector<u8> arm_cov_bits_;
    void arm_cov_mark(u32 pc);
    /// CMeP PC coverage: the window covers the 128 KiB CMeP RAM (first loader and
    /// the staged second loader at 0x40000) *and* the private window at 0x800000
    /// where the secure kernel is linked, because one window has to hold both.
    /// Narrow it with ZLB_MEP_COV_BASE / ZLB_MEP_COV_SIZE.
    u32 mep_cov_base_ = 0x00040000u;
    u32 mep_cov_size_ = 0x00800000u;   ///< 0x40000..0x840000
    u32 mep_cov_gran_ = 2u;            ///< bytes per bit (2 = one MeP instruction)
    std::vector<u8> mep_cov_bits_;
    void mep_cov_mark(u32 pc);
    /// Round 166: pages handed to NSKBL's object constructor when the pool pointer is
    /// NULL (the map object's container fields are empty in the model).
    u32 nskbl_pool_next_ = 0;
    u32 nskbl_pool_supplies_ = 0;
    /// Round 169: pool headers seeded with one free slot (ZLB_NSKBL_POOLFIX).
    u32 nskbl_pool_fixes_ = 0;
    /// Round 180: blocks handed to NSKBL's allocator from the model's heap arena when
    /// the map object's heap pointer (map->[0x8C]) is zero (ZLB_NSKBL_HEAP).
    /// Round 197: free physical pages handed to NSKBL's kernel partition.
    /// Round 208: class-magic alignments made by the model.
    u32 magic_fixes_ = 0;
    u32 nskbl_physpool_next_ = 0;
    u32 nskbl_physpool_fills_ = 0;
    u32 nskbl_heap_next_ = 0;
    u32 nskbl_heap_supplies_ = 0;
    /// Round 167: map-object container slots handed out by the model.

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
    /// The secure kernel's "done" jump returns into the *second loader*, which then
    /// finishes its own work - and that work is what fills the ARM boot context at
    /// the scratch +0x100.  The model lets the CMeP run on for a bounded budget
    /// before releasing the ARM instead of halting it at the jump; otherwise the
    /// second loader's post-processing (and with it the context the ARM reads)
    /// never happens (see docs/KBL.md round 48.15).
    bool cmep_finish_pending_ = false;
    u64 cmep_finish_steps_ = 0;
    /// Round 92: the power-on path of the syscon releases the SoC as soon as the
    /// reset sequencing is done (ernie_power.cpp), so without this gate the model
    /// starts kernel_boot_loader *before* the CMeP second loader has even been
    /// handed off - and the ARM then reads the boot context the second loader is
    /// about to clear/fill (see docs/KBL.md round 92).  On hardware the ARM boot
    /// ROM waits for the CMeP.  `ZLB_ARM_WAIT_CMEP=0` restores the old timing.
    bool cmep_context_done_ = false;
    u64 arm_wait_slices_ = 0;
    /// Safety net: never hold the ARM back longer than this many machine slices.
    static constexpr u64 kArmWaitCmepSlices = 400000;
    /// SceKblParam inputs collected while staging the SLB2 images.
    u32 secure_kernel_size_ = 0;
    u32 kprx_auth_sm_pa_ = 0;
    u32 kprx_auth_sm_size_ = 0;
    u32 prog_rvk_pa_ = 0;
    u32 prog_rvk_size_ = 0;

    EmmcCard& emmc_ref() { return *emmc_; }

    VitaConfig config_;
    CoreBudget budget_;

    bool pc_hook_stopped_ = false;
    Arch pc_hook_arch_ = Arch::Unknown;
    int pc_hook_core_ = 0;
    u32 pc_hook_pc_ = 0;

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
