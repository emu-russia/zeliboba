// zeliboba - ARM Cortex-A9 (ARMv7-A + Thumb-2 + VFPv3-D16) interpreter.
//
// The PS Vita's "Kermit" main CPU is an ARM Cortex-A9 MPCore: ARMv7-A with
// TrustZone, a VFPv3-D16 floating point unit and the ARMv7-A short-descriptor
// MMU, executing the A32, T16 and T32 instruction sets in little-endian data
// mode. This core is a straight port of the reference VitaTestSuite C#
// interpreter (Core/ArmCore.cs); the semantics and the decode classification are
// deliberately identical so both can be diffed instruction by instruction.
//
// Design notes
//   * step() is allocation free in the hot path: no string formatting unless the
//     debugger asks for the disassembly text.
//   * step() never throws and never lets an exception escape: undefined
//     instructions set `undefined_instruction` and fill StepResult::fault.
//   * Memory ordering and MMIO belong to the Bus; this core only performs the
//     virtual -> physical translation and then hands the physical address on.
#pragma once

#include <string>

#include "bus/bus.h"
#include "cpu/arm/arm_defs.h"
#include "cpu/arm/arm_mmu.h"
#include "cpu/arm/arm_vfp.h"
#include "cpu/cpu.h"

namespace zlb {

/// Optional hook for coprocessor instructions the core does not implement
/// itself (p0..p14). Returning true means the hook handled the instruction.
struct ArmCoprocessorHook {
    virtual ~ArmCoprocessorHook() = default;
    virtual bool operator()(bool two_reg, bool load, u32 cpnum, u32 opc1, u32 crn, u32 crm, u32 opc2,
                            u32& rt, u32& rt2) = 0;
};

class ArmCore : public Cpu {
public:
    explicit ArmCore(Bus& bus);

    Arch arch() const override { return Arch::Arm; }
    const char* core_name() const override { return "ARM Cortex-A9"; }

    // ---- architectural state ---------------------------------------------

    /// R0..R15 (R15 is the PC). The banked registers live in the backing store.
    u32 r[16] = {};

    u32 cpsr = arm::kResetCpsr;

    // ---- TrustZone (Security Extensions) ---------------------------------
    //
    // Security state comes from SCR.NS and CPSR.M, not from a CPSR flag.
    // Monitor always executes Secure. Its MRC/MCR accesses to banked CP15
    // registers still select the bank named by SCR.NS (ARM ARM B3.15).
    bool ns_ = false;   // mirror of SCR.NS, including while in Monitor mode
    u32 scr = 0;
    u32 mvbar = 0;
    u32 vbar_nonsecure = 0;

    /// True while executing in the secure world.
    bool secure_state() const { return mode() == arm::kModeMonitor || !ns_; }

    /// Vector base the next exception will use: MVBAR for exceptions taken to
    /// monitor mode, otherwise the VBAR of the world the exception came from.
    u32 effective_vector_base() const;

    /// Current instruction set: true = Thumb (T16/T32), false = ARM (A32).
    bool thumb = false;

    /// MPIDR Aff0: which core of the Kermit cluster this instance models. The
    /// machine has one core, so it is 0, but kernel_boot_loader reads MPIDR to
    /// decide whether it is the boot core and code that runs here is per-core.
    u32 core_id_ = 0;

    /// The current execution world's translation bank, also used by machine
    /// fault hooks and the debugger. Monitor execution always selects Secure.
    ArmMmu mmu;
    ArmVfp vfp;

    /// Optional hook for the p0..p14 coprocessor space.
    ArmCoprocessorHook* coprocessor_hook = nullptr;

    /// Optional hook for translation faults: `(core id, va, write, fetch)`.
    /// A machine can use it to install a mapping the modelled firmware would have
    /// inherited (see Vita::satisfy_arm_boot_fault) - the access is then retried
    /// instead of raising the abort.  Returning false keeps the normal path.
    std::function<bool(u32 core, u32 va, bool write, bool fetch)> fault_hook;

    /// Optional hook called when this core executes SEV; the machine uses it to
    /// wake every WFE-waiting core (round 140: the barrier's sense-reversing wait
    /// races under instruction-level round-robin, so WFE must sleep until SEV).
    std::function<void()> sev_hook;

    /// Optional hook called when this core's IRQ line is asserted.  On hardware an
    /// asserted interrupt takes a core out of WFE whether or not the interrupt is
    /// then taken (the wake-up happens before the CPSR mask is consulted), which is
    /// how a timer tick releases the kernel boot loader's barrier wait; this hook
    /// lets the machine model that without the core knowing about the machine.
    std::function<void()> irq_hook;

    /// Set when this core is blocked in WFE (wait for event); cleared by SEV.
    bool wfe_waiting_ = false;
    /// ARM event register: SEV sets it, WFE consumes it (and skips blocking).
    bool event_pending_ = false;

    /// MMU lookup for one access, giving `fault_hook` one chance to fix a miss.
    arm::MmResult translate_or_fix(u32 va, bool write, bool fetch);

    /// Emit one log line per executed instruction.
    bool trace_instructions = false;

    u64 exception_count = 0;
    u32 last_undefined_instruction = 0;

    // ---- Cpu interface ----------------------------------------------------

    void reset() override;
    void reset(u32 entry) override;
    StepResult step() override;
    std::string disassemble(u32 address, unsigned& length) override;
    void registers(std::vector<RegValue>& out) const override;
    bool set_register(const std::string& name, u64 value) override;
    bool get_register(const std::string& name, u64& value) const override;
    std::string status_line() const override;
    void describe_state(std::vector<std::string>& lines) const override;

    void set_irq(int line, bool asserted) override;
    bool interrupt_pending() const override;

    u32 get_pc() const override { return r[15]; }
    void set_pc(u32 value) override { write_r15(value); }

    // ---- helpers used by tests and the debugger ---------------------------

    u32 mode() const { return cpsr & arm::kModeMask; }
    bool flag_n() const { return (cpsr & arm::kFlagN) != 0; }
    bool flag_z() const { return (cpsr & arm::kFlagZ) != 0; }
    bool flag_c() const { return (cpsr & arm::kFlagC) != 0; }
    bool flag_v() const { return (cpsr & arm::kFlagV) != 0; }

    u32 spsr() const;
    void set_spsr(u32 value);

    /// Banked SPSR of an arbitrary mode field (debugger / test helper).
    u32 banked_spsr(u32 mode_field) const { return bank_spsr_[mode_field & arm::kModeMask]; }

    u32 vector_base() const { return mmu.vector_base(); }

    /// Translate a virtual address for the debugger (never faults when off).
    bool translate(u32 va, bool write, bool fetch, u32& pa, std::string& fault);

    /// Read-only execution-bank access for inspection. Monitor uses Secure
    /// even when SCR.NS selects the Non-secure bank for guest MRC/MCR accesses.
    const ArmMmu& inspection_mmu() const { return mmu_bank(!secure_state()); }

    /// Take a synchronous exception (ARM ARM B1.8.3 / B1.9).
    void take_exception(u32 vector_offset, u32 new_mode, u32 return_address);

private:
    // The other Security bank. Keep `mmu` as the active translation regime so
    // bus/debugger consumers observe the same state as instruction/data access.
    ArmMmu inactive_mmu_;
    bool active_mmu_nonsecure_ = false;
    void sync_mmu_bank();
    ArmMmu& mmu_bank(bool nonsecure);
    const ArmMmu& mmu_bank(bool nonsecure) const;
    ArmMmu& cp15_mmu();

    // ---- banked registers -------------------------------------------------

    // System mode aliases the User slots; it has no separate SP/LR bank.
    u32 bank_sp_[arm::kModeCount] = {};
    u32 bank_lr_[arm::kModeCount] = {};
    u32 bank_spsr_[arm::kModeCount] = {};
    u32 bank_r8_[arm::kModeCount][5] = {};

    // ---- exclusive monitor -------------------------------------------------

    bool exclusive_valid_ = false;
    u32 exclusive_addr_ = 0;
    u32 exclusive_id_ = 0;

    // ---- interrupt lines ---------------------------------------------------

    bool irq_line_ = false;
    bool fiq_line_ = false;

    // ---- double-fault (exception loop) detection ---------------------------
    u32 last_abort_pc_ = 0;
    int abort_loop_count_ = 0;

    // ---- abort bookkeeping -------------------------------------------------

    arm::FaultKind pending_fault_ = arm::FaultKind::None;
    arm::MmResult pending_mm_fault_;

    // ---- instruction decode temporaries ------------------------------------

    u32 cur_instr_addr_ = 0;
    u32 cur_instr_ = 0;
    unsigned cur_instr_len_ = 4;

    // IT state (Thumb only).
    u32 it_state_ = 0;
    bool it_state_valid_ = false;

    // ---- CPSR / banking ----------------------------------------------------

    u32 read_reg(int n) const;
    void write_reg(int n, u32 value);
    void write_r15(u32 value) {
        r[15] = value;
        pc = value;
    }
    void bank_switch(u32 old_mode, u32 new_mode);
    u32 cpsr_instruction_mask(u32 mask) const;
    void write_cpsr_masked(u32 value, u32 mask, bool instruction = true);
    void set_nz(u32 result);
    void set_nzcv(u32 result, bool carry, bool overflow);

    // ---- shifter -----------------------------------------------------------

    u32 shift_imm(u32 value, int type, int amount, bool set_carry);
    u32 shift_reg(u32 value, int type, u32 amount, bool set_carry = true);

    static u32 add_with_carry(u32 a, u32 b, bool carry_in, bool& carry_out, bool& overflow);
    static u32 sub_with_carry(u32 a, u32 b, bool carry_in, bool& carry_out, bool& overflow);

    // ---- memory ------------------------------------------------------------

    void set_access_pc();
    /// Physical address behind a virtual one, for the bus-side exclusive monitor
    /// (round 240).  Falls back to the virtual address when the MMU is off or the
    /// translation faults.
    u32 exclusive_phys(u32 va);

    u32 mem_read_word(u32 va, bool fetch);
    // MemU for single-word LDR/STR. Fetch, multiple, doubleword and exclusive
    // accesses retain their separate alignment/endianness contracts.
    u32 mem_read_single_word(u32 va);
    void mem_write_single_word(u32 va, u32 value);
    void single_word_alignment_fault(u32 va, bool write);
    bool unaligned_single_byte_allowed(u32 va, u32 byte_va, bool write);
    u32 mem_read_half(u32 va);
    u32 mem_read_byte(u32 va);
    void mem_write_word(u32 va, u32 value);
    void mem_write_half(u32 va, u32 value);
    void mem_write_byte(u32 va, u32 value);
    u64 mem_read_double(u32 va);
    void mem_write_double(u32 va, u64 value);
    u32 fetch_half(u32 addr);

    // ---- exception helpers -------------------------------------------------

    void exception_return(u32 target);
    void exception_return_with_cpsr(u32 target, u32 new_cpsr);
    void raise_data_abort();
    void raise_prefetch_abort();
    void undefined(const char* why);

    // ---- A32 ---------------------------------------------------------------

    void execute_arm();
    u32 shifter_operand(u32 instr, bool set_carry);
    void execute_data_processing(u32 instr, u32 op, bool immediate);
    void decode_arm_mul_media(u32 instr);

    /// A32 (ARM state) exclusive access: LDREX/STREX and the byte/halfword
    /// variants. The kernel boot loader uses `ldrex`/`strex` for its locks, so
    /// these must not fall through to the multiply decoder.
    void execute_arm_exclusive(u32 instr);
    u32 dsp_smla(u32 instr, int rd, int rn, int rs, int rm);
    u32 dsp_smul(u32 instr, int rs, int rm);
    void decode_smlalxy(u32 instr, u32 rd_lo, u32 rd_hi, int rm, int rs);
    void decode_arm_media(u32 instr);
    void decode_media_a(u32 instr, int rd, int rn, int rs, int rm, u32 ga);
    static u32 select_bytes(u32 a, u32 b, u32 cpsr_value);
    void parallel_add_sub(u32 instr, int rd, int rn, int rm, u32 op1, u32 op2);
    void decode_sat_extend(u32 instr, int rd, int rn, int rm, u32 op1, u32 op2);
    void decode_smuad(u32 instr, int rd, int rn, int rs, int rm, u32 op1, u32 op2);
    void decode_smmul(u32 instr, int rd, int rn, int rs, int rm, u32 op1);
    void decode_bitfield(u32 instr, int rd, int rn, int rs);
    static bool is_arm_misc_encoding(u32 instr);
    void decode_arm_misc(u32 instr);
    void msr_arm(u32 instr, u32 value);
    void decode_arm_extra_load_store(u32 instr);
    void execute_arm_load_store(u32 instr, bool immediate);
    void execute_arm_load_store_multiple(u32 instr);
    void execute_arm_branch(u32 instr);
    void execute_arm_unconditional(u32 instr);
    void decode_arm_misc_uncond(u32 instr);
    void decode_arm_rfe_srs(u32 instr);
    void decode_arm_cps_setend(u32 instr);
    void execute_arm_coprocessor(u32 instr);
    bool cp15_thread_id_access_allowed(bool read, u32 opc1, u32 crn, u32 crm, u32 opc2);
    u32 cp15_read(u32 opc1v, u32 crn, u32 crm, u32 opc2v, int rt);
    void cp15_write(u32 opc1v, u32 crn, u32 crm, u32 opc2v, u32 value);
    void cp15_cache_op(u32 opc1v, u32 crm, u32 opc2v, u32 value);
    void cp15_tlb_op(u32 opc1v, u32 crm, u32 opc2v, u32 value);
    /// VA-to-PA probe (CP15 c7, c8, opc2): translate and publish the result in PAR.
    void cp15_va_to_pa(u32 opc1v, u32 opc2v, u32 va);

    // ---- Thumb-16 ----------------------------------------------------------

    void execute_thumb16(bool in_it_block);
    void thumb_shift_imm(u32 i, bool setflags);
    void thumb_add_sub(u32 i, bool setflags);
    void thumb_mov_cmp_imm(u32 i, bool setflags);
    void thumb_alu_ops(u32 i, bool setflags);
    void thumb_special_data(u32 i);
    void thumb_literal_load(u32 i);
    void thumb_load_store_reg(u32 i);
    void thumb_load_store_imm(u32 i);
    void thumb_load_store_half_imm(u32 i);
    void thumb_sp_relative(u32 i);
    void thumb_adr(u32 i);
    void thumb_ldm_stm(u32 i);
    void thumb_cond_branch(u32 i);
    void thumb_uncond_branch(u32 i);
    void thumb_misc16(u32 i);

    u32 thumb_condition(bool& in_it_block, bool& last_in_it);
    u32 thumb_condition_peek(bool& in_it_block, bool& last_in_it) const;

    // ---- Thumb-32 ----------------------------------------------------------

    void execute_thumb32();
    void thumb32_data_processing_shifted();
    void thumb32_data_processing_modified();
    void thumb32_data_processing_plain();
    void thumb32_data_processing_register();
    void thumb32_parallel_add_sub(u32 op1, u32 op2, int rd, int rn, int rm);
    void thumb32_extend_add(u32 op1, u32 op2, int rd, int rn, int rm);
    void thumb32_misc_register(u32 op1, u32 op2, int rd, int rn, int rm);
    void thumb32_load_store_dual_excl_table();
    void thumb32_load_store_single();
    void thumb32_coprocessor();
    void thumb32_branch_misc();
    void thumb32_misc();

    // ---- VFP ---------------------------------------------------------------

    void execute_arm_vfp(u32 instr);
    void execute_vfp_data_processing(u32 instr, bool cp10);
    void execute_vfp_op(bool dbl, u32 instr, u32 opcv1, u32 opcv2, int vd, int vn, int vm);
    void execute_vfp_two_register(u32 instr, bool cp10);
    void execute_neon(u32 instr);
    void execute_vfp_load_store(u32 instr, u32 cpnum, bool single);
    void execute_vfp_load_store_multiple(u32 instr, bool single);

    void branch_to(u32 target);

    // ARM ARM B4.1.150..152: software-only 32-bit registers, banked by
    // Security state. MRC/MCR in Monitor select their copy with SCR.NS.
    struct ThreadIdRegisters {
        u32 user_rw = 0;
        u32 user_ro = 0;
        u32 privileged_rw = 0;
    };
    ThreadIdRegisters thread_ids_[2];
    ThreadIdRegisters& cp15_thread_ids() { return thread_ids_[(scr & arm::kScrNs) != 0u]; }
    const ThreadIdRegisters& cp15_thread_ids() const { return thread_ids_[(scr & arm::kScrNs) != 0u]; }
};

}  // namespace zlb
