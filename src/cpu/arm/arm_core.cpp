// zeliboba - ARM Cortex-A9 (ARMv7-A + Thumb-2 + VFPv3-D16) interpreter.
//
// This is a faithful port of the VitaTestSuite reference implementation
// (Core/ArmCore.cs). Keeping the decode classification identical matters: the
// ARMv7-A instruction space has several overlapping encodings (the DSP media
// space shares op1 == 011 with register-offset load/store, MRS/MSR share bits
// [27:20] with the signed multiplies, ...) and the reference tables are known to
// cover firmware 1.04's kernel boot loader.
#include "cpu/arm/arm_core.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "common/log.h"
#include "common/util.h"
#include "cpu/arm/arm_disasm.h"
#include "cpu/factory.h"

namespace zlb {

namespace {

/// Conservative per-instruction cycle costs; the machine layer only uses these
/// for scheduling and for the debugger's throughput display.
u64 cycles_for(u32 instr, bool thumb, bool branch) {
    (void)instr;
    (void)thumb;
    return branch ? 3u : 1u;
}

}  // namespace

// ===========================================================================
// Construction / reset
// ===========================================================================

ArmCore::ArmCore(Bus& bus) : Cpu(bus), mmu(bus) {
    name = "cpu0";
    reset(0);
}

void ArmCore::reset() {
    for (u32& value : r) value = 0;
    for (int i = 0; i < arm::kModeCount; ++i) {
        bank_sp_[i] = 0;
        bank_lr_[i] = 0;
        bank_spsr_[i] = 0;
        for (int j = 0; j < 5; ++j) bank_r8_[i][j] = 0;
    }
    cpsr = arm::kResetCpsr;  // SVC mode, ARM state, A/I/F set, flags clear
    // Reset starts in the secure world (SCR.NS = 0, NS = 0) - that is what makes
    // the secure kernel bootloader the first ARM code to run.
    ns_ = false;
    scr = 0;
    mvbar = 0;
    vbar_nonsecure = 0;
    thumb = false;
    halted = false;
    halt_reason.clear();
    undefined_instruction = false;
    last_undefined_instruction = 0;
    instructions = 0;
    cycles = 0;
    exception_count = 0;
    exclusive_valid_ = false;
    exclusive_addr_ = 0;
    exclusive_id_ = 0;
    pending_fault_ = arm::FaultKind::None;
    it_state_ = 0;
    it_state_valid_ = false;
    cur_instr_ = 0;
    cur_instr_len_ = 4;
    mmu.reset();
    vfp.reset();
    write_r15(0);
}

void ArmCore::reset(u32 entry) {
    reset();
    // Bit 0 of the entry point selects the instruction set (BXWritePC semantics).
    thumb = (entry & 1u) != 0u;
    if (thumb) cpsr |= arm::kFlagT;
    else cpsr &= ~arm::kFlagT;
    write_r15(entry & ~1u);
}

// ===========================================================================
// Mode / CPSR helpers
// ===========================================================================

u32 ArmCore::spsr() const {
    const u32 m = mode();
    if (m == arm::kModeUser || m == arm::kModeSystem) return 0;
    return bank_spsr_[m];
}

void ArmCore::set_spsr(u32 value) {
    const u32 m = mode();
    if (m == arm::kModeUser || m == arm::kModeSystem) return;
    bank_spsr_[m] = value;
}

void ArmCore::bank_switch(u32 old_mode, u32 new_mode) {
    if (old_mode == new_mode) return;
    const int o = static_cast<int>(old_mode & arm::kModeMask);
    const int n = static_cast<int>(new_mode & arm::kModeMask);

    bank_sp_[o] = r[13];
    bank_lr_[o] = r[14];
    if (o == static_cast<int>(arm::kModeFiq)) {
        for (int j = 0; j < 5; ++j) bank_r8_[o][j] = r[8 + j];
    }

    cpsr = (cpsr & ~arm::kModeMask) | (new_mode & arm::kModeMask);
    r[13] = bank_sp_[n];
    r[14] = bank_lr_[n];
    if (n == static_cast<int>(arm::kModeFiq)) {
        for (int j = 0; j < 5; ++j) r[8 + j] = bank_r8_[n][j];
    }
}

void ArmCore::set_nz(u32 result) {
    cpsr &= 0x3FFFFFFFu;
    if ((result & 0x80000000u) != 0) cpsr |= arm::kFlagN;
    if (result == 0) cpsr |= arm::kFlagZ;
}

void ArmCore::set_nzcv(u32 result, bool carry, bool overflow) {
    cpsr &= 0x0FFFFFFFu;
    if ((result & 0x80000000u) != 0) cpsr |= arm::kFlagN;
    if (result == 0) cpsr |= arm::kFlagZ;
    if (carry) cpsr |= arm::kFlagC;
    if (overflow) cpsr |= arm::kFlagV;
}

u32 ArmCore::add_with_carry(u32 a, u32 b, bool carry_in, bool& carry_out, bool& overflow) {
    const u64 sum = static_cast<u64>(a) + static_cast<u64>(b) + (carry_in ? 1ull : 0ull);
    const u32 result = static_cast<u32>(sum);
    carry_out = (sum >> 32) != 0;
    overflow = ((a ^ result) & (b ^ result) & 0x80000000u) != 0;
    return result;
}

u32 ArmCore::sub_with_carry(u32 a, u32 b, bool carry_in, bool& carry_out, bool& overflow) {
    const u64 sum = static_cast<u64>(a) + static_cast<u64>(static_cast<u32>(~b)) +
                    (carry_in ? 1ull : 0ull);
    const u32 result = static_cast<u32>(sum);
    carry_out = (sum >> 32) != 0;
    overflow = ((a ^ b) & (a ^ result) & 0x80000000u) != 0;
    return result;
}

// ===========================================================================
// Shifter
// ===========================================================================

u32 ArmCore::shift_imm(u32 value, int type, int amount, bool set_carry) {
    u32 carry = flag_c() ? 1u : 0u;
    switch (type) {
        case 0:
            if (amount == 0) return value;
            if (amount < 32) {
                carry = (value >> (32 - amount)) & 1u;
                value <<= amount;
            } else if (amount == 32) {
                carry = value & 1u;
                value = 0;
            } else {
                carry = 0;
                value = 0;
            }
            break;
        case 1:
            if (amount == 0) amount = 32;
            if (amount < 32) {
                carry = (value >> (amount - 1)) & 1u;
                value >>= amount;
            } else if (amount == 32) {
                carry = (value >> 31) & 1u;
                value = 0;
            } else {
                carry = 0;
                value = 0;
            }
            break;
        case 2:
            if (amount == 0) amount = 32;
            if (amount < 32) {
                carry = (value >> (amount - 1)) & 1u;
                value = static_cast<u32>(static_cast<s32>(value) >> amount);
            } else {
                carry = (value >> 31) & 1u;
                value = (value & 0x80000000u) != 0 ? 0xFFFFFFFFu : 0u;
            }
            break;
        default:
            if (amount == 0) {
                const u32 next_carry = value & 1u;
                value = (value >> 1) | (carry << 31);
                carry = next_carry;
            } else {
                const int rr = amount & 31;
                if (rr == 0) {
                    carry = (value >> 31) & 1u;
                } else {
                    value = (value >> rr) | (value << (32 - rr));
                    carry = (value >> 31) & 1u;
                }
            }
            break;
    }
    if (set_carry) {
        if (carry != 0) cpsr |= arm::kFlagC;
        else cpsr &= ~arm::kFlagC;
    }
    return value;
}

u32 ArmCore::shift_reg(u32 value, int type, u32 amount) {
    u32 carry = flag_c() ? 1u : 0u;
    if (amount == 0) return value;
    switch (type) {
        case 0:
            if (amount < 32) {
                carry = (value >> static_cast<int>(32 - amount)) & 1u;
                value <<= static_cast<int>(amount);
            } else if (amount == 32) {
                carry = value & 1u;
                value = 0;
            } else {
                carry = 0;
                value = 0;
            }
            break;
        case 1:
            if (amount < 32) {
                carry = (value >> static_cast<int>(amount - 1)) & 1u;
                value >>= static_cast<int>(amount);
            } else if (amount == 32) {
                carry = (value >> 31) & 1u;
                value = 0;
            } else {
                carry = 0;
                value = 0;
            }
            break;
        case 2:
            if (amount < 32) {
                carry = (value >> static_cast<int>(amount - 1)) & 1u;
                value = static_cast<u32>(static_cast<s32>(value) >> static_cast<int>(amount));
            } else {
                carry = (value >> 31) & 1u;
                value = (value & 0x80000000u) != 0 ? 0xFFFFFFFFu : 0u;
            }
            break;
        default: {
            const int rr = static_cast<int>(amount & 31);
            if (rr == 0) {
                carry = (value >> 31) & 1u;
            } else {
                value = (value >> rr) | (value << (32 - rr));
                carry = (value >> 31) & 1u;
            }
            break;
        }
    }
    if (carry != 0) cpsr |= arm::kFlagC;
    else cpsr &= ~arm::kFlagC;
    return value;
}

// ===========================================================================
// Register / PC access
// ===========================================================================

u32 ArmCore::read_reg(int n) const {
    if (n == 15) return arm::read_pc_value(thumb, cur_instr_addr_);
    return r[n];
}

void ArmCore::write_reg(int n, u32 value) {
    if (n == 15) write_r15(value);
    else r[n] = value;
}

void ArmCore::write_cpsr_masked(u32 value, u32 mask) {
    const u32 old_mode = mode();
    const bool privileged = old_mode != arm::kModeUser;
    const u32 user_mask = privileged ? mask : (mask & 0xF0000000u);
    u32 new_cpsr = (cpsr & ~user_mask) | (value & user_mask);
    new_cpsr &= ~arm::kFlagJ;

    u32 new_mode = new_cpsr & arm::kModeMask;
    if (!privileged) new_mode = old_mode;

    if (new_mode != old_mode) {
        bank_switch(old_mode, new_mode);
        cpsr = (cpsr & ~user_mask) | (value & user_mask);
        cpsr = (cpsr & ~arm::kModeMask) | new_mode;
        cpsr &= ~arm::kFlagJ;
    } else {
        cpsr = new_cpsr;
    }

    thumb = (cpsr & arm::kFlagT) != 0;
}

// ===========================================================================
// Memory path
// ===========================================================================

void ArmCore::set_access_pc() {
    bus->context.pc = cur_instr_addr_;
    bus->context.core = name.c_str();
}

arm::MmResult ArmCore::translate_or_fix(u32 va, bool write, bool fetch) {
    arm::MmResult result = mmu.translate(va, write, fetch, mode());
    if (result.ok || !fault_hook) return result;
    if (!fault_hook(core_id_, va, write, fetch)) return result;
    return mmu.translate(va, write, fetch, mode());
}

u32 ArmCore::mem_read_word(u32 va, bool fetch) {
    set_access_pc();
    if (mmu.enabled()) {
        const arm::MmResult result = translate_or_fix(va, false, fetch);
        if (!result.ok) {
            pending_mm_fault_ = result;
            pending_fault_ = fetch ? arm::FaultKind::Prefetch : arm::FaultKind::Data;
            if (fetch) mmu.report_prefetch_abort(result, va);
            else mmu.report_data_abort(result, va, false);
            return 0xFFFFFFFFu;
        }
        return fetch ? bus->fetch32(result.phys_addr) : bus->read32(result.phys_addr);
    }
    return fetch ? bus->fetch32(va) : bus->read32(va);
}

u32 ArmCore::mem_read_half(u32 va) {
    set_access_pc();
    if (mmu.enabled()) {
        const arm::MmResult result = translate_or_fix(va, false, false);
        if (!result.ok) {
            pending_mm_fault_ = result;
            pending_fault_ = arm::FaultKind::Data;
            mmu.report_data_abort(result, va, false);
            return 0xFFFFu;
        }
        return bus->read16(result.phys_addr);
    }
    return bus->read16(va);
}

u32 ArmCore::mem_read_byte(u32 va) {
    set_access_pc();
    if (mmu.enabled()) {
        const arm::MmResult result = translate_or_fix(va, false, false);
        if (!result.ok) {
            pending_mm_fault_ = result;
            pending_fault_ = arm::FaultKind::Data;
            mmu.report_data_abort(result, va, false);
            return 0xFFu;
        }
        return bus->read8(result.phys_addr);
    }
    return bus->read8(va);
}

void ArmCore::mem_write_word(u32 va, u32 value) {
    set_access_pc();
    if (mmu.enabled()) {
        const arm::MmResult result = translate_or_fix(va, true, false);
        if (!result.ok) {
            pending_mm_fault_ = result;
            pending_fault_ = arm::FaultKind::Data;
            mmu.report_data_abort(result, va, true);
            return;
        }
        bus->write32(result.phys_addr, value);
        return;
    }
    bus->write32(va, value);
}

void ArmCore::mem_write_half(u32 va, u32 value) {
    set_access_pc();
    if (mmu.enabled()) {
        const arm::MmResult result = translate_or_fix(va, true, false);
        if (!result.ok) {
            pending_mm_fault_ = result;
            pending_fault_ = arm::FaultKind::Data;
            mmu.report_data_abort(result, va, true);
            return;
        }
        bus->write16(result.phys_addr, static_cast<u16>(value));
        return;
    }
    bus->write16(va, static_cast<u16>(value));
}

void ArmCore::mem_write_byte(u32 va, u32 value) {
    set_access_pc();
    if (mmu.enabled()) {
        const arm::MmResult result = translate_or_fix(va, true, false);
        if (!result.ok) {
            pending_mm_fault_ = result;
            pending_fault_ = arm::FaultKind::Data;
            mmu.report_data_abort(result, va, true);
            return;
        }
        bus->write8(result.phys_addr, static_cast<u8>(value));
        return;
    }
    bus->write8(va, static_cast<u8>(value));
}

u64 ArmCore::mem_read_double(u32 va) {
    const u32 lo = mem_read_word(va, false);
    if (pending_fault_ != arm::FaultKind::None) return 0;
    const u32 hi = mem_read_word(va + 4u, false);
    if (pending_fault_ != arm::FaultKind::None) return 0;
    if ((cpsr & arm::kFlagE) != 0) return (static_cast<u64>(lo) << 32) | hi;
    return static_cast<u64>(lo) | (static_cast<u64>(hi) << 32);
}

void ArmCore::mem_write_double(u32 va, u64 value) {
    if ((cpsr & arm::kFlagE) != 0) {
        mem_write_word(va, static_cast<u32>(value >> 32));
        if (pending_fault_ != arm::FaultKind::None) return;
        mem_write_word(va + 4u, static_cast<u32>(value));
    } else {
        mem_write_word(va, static_cast<u32>(value));
        if (pending_fault_ != arm::FaultKind::None) return;
        mem_write_word(va + 4u, static_cast<u32>(value >> 32));
    }
}

u32 ArmCore::fetch_half(u32 addr) {
    set_access_pc();
    if (mmu.enabled()) {
        const arm::MmResult result = translate_or_fix(addr, false, true);
        if (!result.ok) {
            pending_mm_fault_ = result;
            pending_fault_ = arm::FaultKind::Prefetch;
            mmu.report_prefetch_abort(result, addr);
            return 0xFFFFu;
        }
        return bus->fetch16(result.phys_addr);
    }
    return bus->fetch16(addr);
}

// ===========================================================================
// Exceptions
// ===========================================================================

void ArmCore::take_exception(u32 vector_offset, u32 new_mode, u32 return_address) {
    const u32 old_mode = mode();
    ++exception_count;

    // Pick the vector table *before* the world can change: monitor entries live
    // at MVBAR, everything else uses the VBAR of the world the exception is
    // taken from (the non-secure world has its own bank of VBAR).
    const bool to_monitor = (new_mode & arm::kModeMask) == arm::kModeMonitor;
    const u32 vector_base = to_monitor && mvbar != 0u ? mvbar : effective_vector_base();

    const u32 saved_cpsr = cpsr;

    u32 new_cpsr = new_mode & arm::kModeMask;
    new_cpsr |= arm::kFlagF;
    if (new_mode != arm::kModeFiq) new_cpsr |= arm::kFlagI;
    if (new_mode != arm::kModeHyp) new_cpsr |= arm::kFlagA;
    new_cpsr |= (cpsr & 0x0F000000u);  // keep NZCV
    new_cpsr |= (cpsr & arm::kFlagE);  // keep E
    new_cpsr &= ~arm::kFlagT;          // ARM state on entry

    const int o = static_cast<int>(old_mode & arm::kModeMask);
    bank_sp_[o] = r[13];
    bank_lr_[o] = r[14];
    bank_spsr_[o] = saved_cpsr;
    if (o == static_cast<int>(arm::kModeFiq)) {
        for (int j = 0; j < 5; ++j) bank_r8_[o][j] = r[8 + j];
    }

    cpsr = new_cpsr;
    thumb = false;
    it_state_valid_ = false;
    it_state_ = 0;

    const int n = static_cast<int>(new_mode & arm::kModeMask);
    r[13] = bank_sp_[n];
    r[14] = return_address;
    if (n == static_cast<int>(arm::kModeFiq)) {
        for (int j = 0; j < 5; ++j) r[8 + j] = bank_r8_[n][j];
    }

    write_r15(vector_base + vector_offset);
}

u32 ArmCore::effective_vector_base() const {
    if (secure_state()) return mmu.vector_base();
    return vbar_nonsecure != 0u ? vbar_nonsecure : mmu.vector_base();
}

void ArmCore::exception_return(u32 target) {
    const u32 spsr_value = spsr();
    const u32 old_mode = mode();
    const u32 new_mode = spsr_value & arm::kModeMask;

    const int o = static_cast<int>(old_mode & arm::kModeMask);
    bank_sp_[o] = r[13];
    bank_lr_[o] = r[14];
    if (o == static_cast<int>(arm::kModeFiq)) {
        for (int j = 0; j < 5; ++j) bank_r8_[o][j] = r[8 + j];
    }

    cpsr = spsr_value & ~arm::kFlagJ;
    thumb = (cpsr & arm::kFlagT) != 0;
    it_state_valid_ = false;
    it_state_ = 0;

    const int n = static_cast<int>(new_mode & arm::kModeMask);
    r[13] = bank_sp_[n];
    r[14] = bank_lr_[n];
    if (n == static_cast<int>(arm::kModeFiq)) {
        for (int j = 0; j < 5; ++j) r[8 + j] = bank_r8_[n][j];
    }

    // The instruction set on return comes from SPSR.T, never the branch address.
    write_r15(target & ~1u);
    pending_fault_ = arm::FaultKind::None;
}

void ArmCore::exception_return_with_cpsr(u32 target, u32 new_cpsr) {
    const u32 old_mode = mode();
    const int o = static_cast<int>(old_mode & arm::kModeMask);
    bank_sp_[o] = r[13];
    bank_lr_[o] = r[14];
    if (o == static_cast<int>(arm::kModeFiq)) {
        for (int j = 0; j < 5; ++j) bank_r8_[o][j] = r[8 + j];
    }

    cpsr = new_cpsr & ~arm::kFlagJ;
    thumb = (cpsr & arm::kFlagT) != 0;
    it_state_valid_ = false;
    it_state_ = 0;

    const int n = static_cast<int>(cpsr & arm::kModeMask);
    r[13] = bank_sp_[n];
    r[14] = bank_lr_[n];
    if (n == static_cast<int>(arm::kModeFiq)) {
        for (int j = 0; j < 5; ++j) r[8 + j] = bank_r8_[n][j];
    }
    write_r15(target & ~1u);
    pending_fault_ = arm::FaultKind::None;
}

void ArmCore::raise_data_abort() {
    take_exception(arm::kVecDataAbort, arm::kModeAbort, cur_instr_addr_ + (thumb ? 4u : 8u));
}

void ArmCore::raise_prefetch_abort() {
    take_exception(arm::kVecPrefetchAbort, arm::kModeAbort, cur_instr_addr_ + (thumb ? 4u : 8u));
}

void ArmCore::undefined(const char* why) {
    undefined_instruction = true;
    last_undefined_instruction = cur_instr_;
    char buffer[192];
    std::snprintf(buffer, sizeof(buffer), "undefined instruction 0x%08X at 0x%08X (%s)", cur_instr_,
                  cur_instr_addr_, why);
    halt(buffer);
}

void ArmCore::branch_to(u32 target) {
    thumb = (target & 1u) != 0u;
    if (thumb) cpsr |= arm::kFlagT;
    else cpsr &= ~arm::kFlagT;
    write_r15(target & ~1u);
}

// ===========================================================================
// Interrupts
// ===========================================================================

void ArmCore::set_irq(int line, bool asserted) {
    if (line == static_cast<int>(IrqLine::Irq)) irq_line_ = asserted;
    else if (line == static_cast<int>(IrqLine::FiQ)) fiq_line_ = asserted;
}

bool ArmCore::interrupt_pending() const {
    if (fiq_line_ && (cpsr & arm::kFlagF) == 0) return true;
    if (irq_line_ && (cpsr & arm::kFlagI) == 0) return true;
    return false;
}

// ===========================================================================
// step()
// ===========================================================================

StepResult ArmCore::step() {
    StepResult result;
    result.address = r[15];
    cur_instr_addr_ = r[15];
    cur_instr_len_ = thumb ? 2u : 4u;
    pending_fault_ = arm::FaultKind::None;
    const bool thumb_at_entry = thumb;
    bool branch_taken = false;

    // Interrupts are only taken at an instruction boundary.
    if (fiq_line_ && (cpsr & arm::kFlagF) == 0) {
        take_exception(arm::kVecFiq, arm::kModeFiq, cur_instr_addr_ + (thumb ? 4u : 4u));
        ++instructions;
        ++cycles;
        result.address = cur_instr_addr_;
        result.length = cur_instr_len_;
        result.text = arm_disassemble(*bus, cur_instr_addr_, thumb_at_entry, cur_instr_len_);
        return result;
    }
    if (irq_line_ && (cpsr & arm::kFlagI) == 0) {
        take_exception(arm::kVecIrq, arm::kModeIrq, cur_instr_addr_ + (thumb ? 4u : 4u));
        ++instructions;
        ++cycles;
        result.address = cur_instr_addr_;
        result.length = cur_instr_len_;
        result.text = arm_disassemble(*bus, cur_instr_addr_, thumb_at_entry, cur_instr_len_);
        return result;
    }

    if (thumb) {
        write_r15(cur_instr_addr_ + 4u);
        const u32 hw1 = fetch_half(cur_instr_addr_);
        if (pending_fault_ == arm::FaultKind::None && (hw1 & 0xF800u) >= 0xE800u) {
            const u32 hw2 = fetch_half(cur_instr_addr_ + 2u);
            if (pending_fault_ == arm::FaultKind::None) {
                cur_instr_ = (hw1 << 16) | hw2;
                cur_instr_len_ = 4;
                execute_thumb32();
            }
        } else if (pending_fault_ == arm::FaultKind::None) {
            cur_instr_ = hw1;
            cur_instr_len_ = 2;
            // Thumb-16 instructions other than B<c> and CBZ/CBNZ ignore their own
            // condition; inside an IT block the block's condition decides.
            bool in_it = false;
            bool last_in_it = false;
            const u32 tcond = thumb_condition(in_it, last_in_it);
            (void)last_in_it;
            if (in_it && !arm::condition_passed(tcond, cpsr)) {
                write_r15(cur_instr_addr_ + 2u);
            } else {
                execute_thumb16();
            }
        }
    } else {
        write_r15(cur_instr_addr_ + 8u);
        cur_instr_ = mem_read_word(cur_instr_addr_, true);
        cur_instr_len_ = 4;
        if (pending_fault_ == arm::FaultKind::None) execute_arm();
    }

    branch_taken = (r[15] != cur_instr_addr_ + cur_instr_len_);

    ++instructions;
    cycles += cycles_for(cur_instr_, thumb_at_entry, branch_taken);

    if (!undefined_instruction && pending_fault_ != arm::FaultKind::None) {
        const arm::FaultKind fault = pending_fault_;
        const arm::MmResult mm = pending_mm_fault_;
        pending_fault_ = arm::FaultKind::None;
        if (fault == arm::FaultKind::Prefetch) raise_prefetch_abort();
        else raise_data_abort();
        result.faulted = true;
        // Include the descriptor the walk actually fetched: "section translation
        // fault at VA X" is only half the story, and the descriptor type plus the
        // table it came from is what tells you whether the table was built at all
        // or whether the wrong TTBR/index was used.
        result.fault = arm::fault_text(mm.fault, cur_instr_addr_, false,
                                       fault == arm::FaultKind::Prefetch);
        result.fault += format(" [%s L1[0x%08X]=0x%08X via TTBR%d base=0x%08X%s]",
                               mm.ok ? "ok" : "fault", mmu.last_walk.l1_addr, mmu.last_walk.l1_desc,
                               mmu.last_walk.ttbr_num, mmu.last_walk.ttbr_base,
                               mmu.last_walk.used_l2
                                   ? format(" L2[0x%08X]=0x%08X", mmu.last_walk.l2_addr,
                                            mmu.last_walk.l2_desc).c_str()
                                   : "");

        // A fault while fetching an exception vector faults again on the vector
        // page itself: a double fault, which the architecture has no recovery
        // path for. Real hardware locks up; report it instead of spinning
        // forever (the kernel boot loader hits exactly this when its tables do
        // not map the low vector page, and the spin otherwise shows up only as a
        // huge exception count).
        const u32 vector_base = mmu.vector_base();
        if (cur_instr_addr_ >= vector_base && cur_instr_addr_ < vector_base + 0x20u) {
            if (cur_instr_addr_ == last_abort_pc_) {
                if (++abort_loop_count_ >= 8) {
                    char why[160];
                    std::snprintf(why, sizeof(why),
                                  "exception loop: vector at 0x%08X faults (%s), DFSR=0x%08X "
                                  "DFAR=0x%08X IFSR=0x%08X IFAR=0x%08X",
                                  cur_instr_addr_, result.fault.c_str(), mmu.dfsr, mmu.dfar, mmu.ifsr,
                                  mmu.ifar);
                    halt(why);
                }
            } else {
                last_abort_pc_ = cur_instr_addr_;
                abort_loop_count_ = 1;
            }
        } else {
            abort_loop_count_ = 0;
        }
    } else if (!undefined_instruction) {
        pc = r[15];
    }

    result.address = cur_instr_addr_;
    result.length = cur_instr_len_;
    result.was_branch = branch_taken;

    unsigned disasm_length = 0;
    result.text = arm_disassemble(*bus, cur_instr_addr_, thumb_at_entry, disasm_length);
    if (undefined_instruction) {
        result.faulted = true;
        if (result.fault.empty()) result.fault = halt_reason;
        if (result.text.empty()) {
            char buffer[32];
            std::snprintf(buffer, sizeof(buffer), ".word 0x%08X", cur_instr_);
            result.text = buffer;
        }
    }
    if (trace_instructions) {
        ZLB_LOG_TRACE("cpu", "%s 0x%08X  %s", name.c_str(), cur_instr_addr_, result.text.c_str());
    }
    return result;
}

// ===========================================================================
// A32
// ===========================================================================

void ArmCore::execute_arm() {
    const u32 instr = cur_instr_;
    const u32 cond = instr >> 28;
    const u32 op1v = (instr >> 25) & 7u;

    if (cond == 0xFu) {
        execute_arm_unconditional(instr);
        return;
    }

    const bool passed = (cond == 0xEu) || arm::condition_passed(cond, cpsr);
    if (!passed) {
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    if (op1v == 0u) {
        // 000x: H1 extra load/store, multiply/swap, DSP media, MISC, then data
        // processing (register). The classification is a function of bits
        // [27:20] (ga) and bits [7:4] (nib).
        const u32 ga = (instr >> 20) & 0xFFu;
        const u32 nib = instr & 0xF0u;
        if (nib == 0xB0u || nib == 0xD0u || nib == 0xF0u) {
            decode_arm_extra_load_store(instr);
            return;
        }
        if (nib == 0x90u) {
            // 0001 100x ... 1111 1001 is the A32 exclusive space (LDREX/STREX),
            // not a multiply: bits 27-20 select the variant.
            const u32 excl = instr & 0x0FF00FF0u;
            if (excl == 0x01900F90u || excl == 0x01800F90u ||  // LDREX  / STREX
                excl == 0x01D00F90u || excl == 0x01C00F90u ||  // LDREXB / STREXB
                excl == 0x01F00F90u || excl == 0x01E00F90u) {  // LDREXH / STREXH
                execute_arm_exclusive(instr);
                return;
            }
            decode_arm_mul_media(instr);
            return;
        }
        if (ga >= 0x10u && ga <= 0x17u && (ga & 1u) == 0u &&
            (nib == 0x50u || (nib & 0x90u) == 0x80u)) {
            decode_arm_media(instr);
            return;
        }
        if ((instr & 0x0FFF0FF0u) == 0x016F0F10u) {
            r[(instr >> 12) & 0xFu] = arm::count_leading_zeros(r[instr & 0xFu]);
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        if (ga >= 0x10u && ga <= 0x17u && is_arm_misc_encoding(instr)) {
            decode_arm_misc(instr);
            return;
        }
        execute_data_processing(instr, (instr >> 21) & 0xFu, false);
        return;
    }

    if (op1v == 1u) {
        // 001x: data processing (immediate) / MSR immediate.
        const u32 dp_op = (instr >> 21) & 0xFu;
        if ((dp_op & 0xAu) == 0xAu && ((instr >> 12) & 0xFu) == 0xFu) {
            msr_arm(instr, arm::decode_imm12(instr));
            return;
        }
        execute_data_processing(instr, dp_op, true);
        return;
    }

    if (op1v == 2u || op1v == 3u) {
        // 010x / 011x: load/store word and unsigned byte. Bit 4 set in the
        // register-offset form (011) selects the DSP/SIMD media space.
        if (op1v == 3u && (instr & 0x10u) != 0u) {
            decode_arm_media(instr);
            return;
        }
        execute_arm_load_store(instr, op1v == 2u);
        return;
    }

    if (op1v == 4u) {
        execute_arm_load_store_multiple(instr);
        return;
    }
    if (op1v == 5u) {
        execute_arm_branch(instr);
        return;
    }

    // bits [27:24] == 1111 with cond != 1111 is SVC.
    if ((instr & 0x0F000000u) == 0x0F000000u) {
        take_exception(arm::kVecSupervisor, arm::kModeSupervisor, cur_instr_addr_ + 4u);
        return;
    }

    execute_arm_coprocessor(instr);
}

u32 ArmCore::shifter_operand(u32 instr, bool set_carry) {
    const u32 rm = instr & 0xFu;
    if ((instr & 0x10u) == 0) {
        const int type = static_cast<int>((instr >> 5) & 3u);
        const int amount = static_cast<int>((instr >> 7) & 0x1Fu);
        return shift_imm(read_reg(static_cast<int>(rm)), type, amount, set_carry);
    }
    const u32 rs = (instr >> 8) & 0xFu;
    const int type = static_cast<int>((instr >> 5) & 3u);
    if (rs == 15) {
        const u32 amount = (arm::read_pc_value(false, cur_instr_addr_) + 4u) & 0xFFu;
        return shift_imm(read_reg(static_cast<int>(rm)), type, static_cast<int>(amount), false);
    }
    return shift_reg(read_reg(static_cast<int>(rm)), type, r[rs] & 0xFFu);
}

void ArmCore::execute_data_processing(u32 instr, u32 op, bool immediate) {
    const bool s = (instr & (1u << 20)) != 0;
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    // AND/EOR/TST/TEQ only produce N/Z (the shifter supplies C); the arithmetic
    // ops including CMP/CMN set all four flags. The reference core classified
    // TST/TEQ/CMP/CMN together as "logical", so CMP/CMN never updated C or V --
    // which breaks every subsequent conditional that tests carry or overflow.
    const bool logical = op <= 1u || op == 0x8u || op == 0x9u;

    // A32 MOVW / MOVT: bits [27:25] = 001, opcode 1000 (MOVW) or 1010 (MOVT)
    // with S == 0; otherwise those opcodes are TST / CMP.
    if (immediate && !s && (op == 0x8u || op == 0xAu) && rd != 15) {
        const u32 imm16 = (static_cast<u32>(rn) << 12) | (instr & 0xFFFu);
        if (op == 0x8u) r[rd] = imm16;
        else r[rd] = (r[rd] & 0x0000FFFFu) | (imm16 << 16);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    u32 op2;
    if (immediate) op2 = arm::decode_imm12(instr);
    else op2 = shifter_operand(instr, s && logical);

    const u32 a = (op == 0xFu || op == 0xDu) ? 0u : read_reg(rn);
    u32 result = 0;
    bool carry = flag_c();
    bool overflow = flag_v();
    bool write_result = true;

    switch (op) {
        case 0x0u: result = a & op2; break;
        case 0x1u: result = a ^ op2; break;
        case 0x2u: result = sub_with_carry(a, op2, true, carry, overflow); break;
        case 0x3u: result = sub_with_carry(op2, a, true, carry, overflow); break;
        case 0x4u: result = add_with_carry(a, op2, false, carry, overflow); break;
        case 0x5u: result = add_with_carry(a, op2, flag_c(), carry, overflow); break;
        case 0x6u: result = sub_with_carry(a, op2, flag_c(), carry, overflow); break;
        case 0x7u: result = sub_with_carry(op2, a, flag_c(), carry, overflow); break;
        case 0x8u: result = a & op2; write_result = false; break;
        case 0x9u: result = a ^ op2; write_result = false; break;
        case 0xAu: result = sub_with_carry(a, op2, true, carry, overflow); write_result = false; break;
        case 0xBu: result = add_with_carry(a, op2, false, carry, overflow); write_result = false; break;
        case 0xCu: result = a | op2; break;
        case 0xDu: result = op2; break;
        case 0xEu: result = a & ~op2; break;
        default: result = ~op2; break;
    }

    if (write_result) {
        if (rd == 15) {
            if (s) {
                if (mode() == arm::kModeUser) {
                    ZLB_LOG_WARN("cpu", "%s S bit with Rd=15 in User mode is unpredictable", name.c_str());
                }
                exception_return(result);
                return;
            }
            // ARM ALUWritePC: the instruction set does not change.
            write_r15(result & ~3u);
            return;
        }
        r[rd] = result;
    }

    if (s) {
        if (logical) set_nz(result);
        else set_nzcv(result, carry, overflow);
    }

    write_r15(cur_instr_addr_ + 4u);
}

// ---- multiply / media (bits [27:23] = 00001) ------------------------------

void ArmCore::execute_arm_exclusive(u32 instr) {
    const u32 kind = (instr >> 20) & 0xFu;  // 8 STREX, 9 LDREX, C STREXB, D LDREXB, E STREXH, F LDREXH
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const int rd = static_cast<int>((instr >> 12) & 0xFu);  // Rt for loads, Rd for stores
    const int low = static_cast<int>(instr & 0xFu);         // Rt for stores, imm4 for loads
    const u32 base = read_reg(rn);

    const bool is_load = (kind == 0x9u || kind == 0xDu || kind == 0xFu);
    if (is_load) {
        // A32 LDREX/LDREXB/LDREXH take no offset: bits 3-0 are the fixed 1111.
        (void)low;
        const unsigned width = (kind == 0xDu) ? 1u : ((kind == 0xFu) ? 2u : 4u);
        const u32 address = (kind == 0xDu) ? base : ((kind == 0xFu) ? (base & ~1u) : (base & ~3u));
        u32 value = 0;
        if (kind == 0x9u) value = mem_read_word(address, false);
        else if (kind == 0xDu) value = mem_read_byte(address) & 0xFFu;
        else value = mem_read_half(address) & 0xFFFFu;
        if (pending_fault_ != arm::FaultKind::None) return;

        // The reservation lives in the bus (the SCU's global monitor) so that a
        // write by another core makes this core's STREX fail, as on hardware.
        bus->mark_exclusive(address, width, static_cast<int>(core_id_), mmu.context_idr);
        exclusive_valid_ = true;
        exclusive_addr_ = address;
        exclusive_id_ = mmu.context_idr;
        r[rd] = value;
    } else {
        const unsigned width = (kind == 0xCu) ? 1u : ((kind == 0xEu) ? 2u : 4u);
        const u32 address = (kind == 0xCu) ? base : ((kind == 0xEu) ? (base & ~1u) : (base & ~3u));
        u32 status = 1u;
        if (bus->take_exclusive(address, width, static_cast<int>(core_id_), mmu.context_idr)) {
            if (kind == 0x8u) mem_write_word(address, r[low]);
            else if (kind == 0xCu) mem_write_byte(address, r[low] & 0xFFu);
            else mem_write_half(address, r[low] & 0xFFFFu);
            if (pending_fault_ != arm::FaultKind::None) return;
            status = 0u;
        }
        exclusive_valid_ = false;
        r[rd] = status;
    }
    write_r15(cur_instr_addr_ + 4u);
}

void ArmCore::decode_arm_mul_media(u32 instr) {
    const u32 op = (instr >> 21) & 0xFu;
    const bool s = (instr & (1u << 20)) != 0;
    const int rn_field = static_cast<int>((instr >> 16) & 0xFu);
    const int rd_field = static_cast<int>((instr >> 12) & 0xFu);
    const int rs = static_cast<int>((instr >> 8) & 0xFu);
    const int rm = static_cast<int>(instr & 0xFu);

    // SDIV/UDIV: 0111 0001 / 0111 0011 with Rd = bits [19:16] and Rn = [15:12].
    if (op == 0x1u && (instr & 0xFu) == 0u) {
        const u32 dividend = r[rn_field];
        const u32 divisor = r[rs];
        u32 quotient;
        if (divisor == 0) {
            quotient = 0u;  // SDIV/UDIV by zero yields 0
        } else if ((instr & (1u << 22)) != 0u) {
            quotient = dividend / divisor;  // UDIV
        } else {
            const s32 a = static_cast<s32>(dividend);
            const s32 b = static_cast<s32>(divisor);
            if (a == INT32_MIN && b == -1) quotient = 0x80000000u;
            else quotient = static_cast<u32>(a / b);
        }
        r[rn_field] = quotient;
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    if (op == 0x1u) {
        // SMLABB/SMLABT/SMLATB/SMLATT and SMLAWB/SMLAWT.
        r[rd_field] = dsp_smla(instr, rd_field, rn_field, rs, rm);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }
    if (op == 0x3u) {
        // SMULBB/SMULBT/SMULTB/SMULTT, SMULWB/SMULWT.
        r[rd_field] = dsp_smul(instr, rs, rm);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }
    if (op == 0x5u) {
        // SMLALBB/SMLALBT/SMLALTB/SMLALTT.
        decode_smlalxy(instr, static_cast<u32>(rd_field), static_cast<u32>(rn_field), rm, rs);
        return;
    }
    if (op == 0x2u || op == 0x6u) {
        if (op == 0x2u) r[rd_field] = dsp_smul(instr, rs, rm);  // SMULW
        else r[rd_field] = dsp_smla(instr, rd_field, rn_field, rs, rm);  // SMLAW
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    switch (op) {
        case 0u: {
            const u32 value = r[rm] * r[rs];
            r[rn_field] = value;
            if (s) set_nz(value);
            break;
        }
        case 4u: {
            const u64 value = static_cast<u64>(r[rm]) * r[rs] + r[rd_field] + r[rn_field];
            r[rd_field] = static_cast<u32>(value);
            r[rn_field] = static_cast<u32>(value >> 32);
            break;
        }
        case 6u: {
            const u32 value = r[rn_field] - r[rm] * r[rs];
            r[rd_field] = value;
            break;
        }
        case 8u:
        case 9u:
        case 0xAu:
        case 0xBu:
        case 0xCu:
        case 0xDu:
        case 0xEu:
        case 0xFu: {
            const bool is_unsigned = (op & 4u) == 0u;
            const bool accumulate = (op & 2u) != 0u;
            u64 product;
            if (is_unsigned) {
                product = static_cast<u64>(r[rm]) * r[rs];
            } else {
                product = static_cast<u64>(static_cast<s64>(static_cast<s32>(r[rm])) *
                                           static_cast<s32>(r[rs]));
            }
            if (accumulate) {
                product += static_cast<u64>(r[rn_field]) | (static_cast<u64>(r[rd_field]) << 32);
            }
            r[rn_field] = static_cast<u32>(product);
            r[rd_field] = static_cast<u32>(product >> 32);
            if (s) {
                cpsr &= 0x0FFFFFFFu;
                if ((r[rd_field] & 0x80000000u) != 0) cpsr |= arm::kFlagN;
                if (r[rd_field] == 0 && r[rn_field] == 0) cpsr |= arm::kFlagZ;
            }
            break;
        }
        default:
            undefined("multiply encoding");
            return;
    }
    write_r15(cur_instr_addr_ + 4u);
}

u32 ArmCore::dsp_smla(u32 instr, int rd, int rn, int rs, int rm) {
    (void)rd;
    const bool bit5 = (instr & 0x20u) != 0;
    const bool bit6 = (instr & 0x40u) != 0;
    if (!bit5) {
        // SMLAWB / SMLAWT: word x halfword, result = (prod + acc) >> 16.
        const s64 product = static_cast<s64>(static_cast<s32>(r[rm])) *
                            static_cast<s16>(bit6 ? (r[rs] >> 16) : (r[rs] & 0xFFFFu));
        const s64 sum = product + static_cast<s32>(r[rn]);
        const bool overflow = (sum > 2147483647LL) || (sum < -2147483648LL);
        if (overflow) cpsr |= arm::kFlagQ;
        else cpsr &= ~arm::kFlagQ;
        return static_cast<u32>(static_cast<s32>(sum >> 16));
    }
    const s64 product = static_cast<s64>(static_cast<s16>(bit5 ? (r[rm] >> 16) : (r[rm] & 0xFFFFu))) *
                        static_cast<s64>(static_cast<s16>(bit6 ? (r[rs] >> 16) : (r[rs] & 0xFFFFu)));
    const s64 sum = product + static_cast<s32>(r[rn]);
    const bool overflow = (sum > 2147483647LL) || (sum < -2147483648LL);
    if (overflow) cpsr |= arm::kFlagQ;
    else cpsr &= ~arm::kFlagQ;
    return static_cast<u32>(static_cast<s32>(sum));
}

u32 ArmCore::dsp_smul(u32 instr, int rs, int rm) {
    const bool bit5 = (instr & 0x20u) != 0;
    const bool bit6 = (instr & 0x40u) != 0;
    if (!bit5) {
        // SMULWB / SMULWT.
        const s64 product = static_cast<s64>(static_cast<s32>(r[rm])) *
                            static_cast<s16>(bit6 ? (r[rs] >> 16) : (r[rs] & 0xFFFFu));
        return static_cast<u32>(static_cast<s32>(product >> 16));
    }
    const s32 v = static_cast<s16>(bit5 ? (r[rm] >> 16) : (r[rm] & 0xFFFFu));
    const s32 w = static_cast<s16>(bit6 ? (r[rs] >> 16) : (r[rs] & 0xFFFFu));
    return static_cast<u32>(v * w);
}

void ArmCore::decode_smlalxy(u32 instr, u32 rd_lo, u32 rd_hi, int rm, int rs) {
    const bool x = (instr & 0x20u) != 0;
    const bool y = (instr & 0x40u) != 0;
    const s64 product = static_cast<s64>(static_cast<s16>(x ? (r[rm] >> 16) : (r[rm] & 0xFFFFu))) *
                        static_cast<s64>(static_cast<s16>(y ? (r[rs] >> 16) : (r[rs] & 0xFFFFu)));
    const s64 acc = static_cast<s64>(static_cast<u64>(r[rd_lo]) | (static_cast<u64>(r[rd_hi]) << 32));
    const s64 sum = acc + product;
    r[rd_lo] = static_cast<u32>(static_cast<u64>(sum));
    r[rd_hi] = static_cast<u32>(static_cast<u64>(sum) >> 32);
    write_r15(cur_instr_addr_ + 4u);
}

// ---- media space (bits [27:25] = 011, bit 4 = 1) ---------------------------

void ArmCore::decode_arm_media(u32 instr) {
    const u32 o1 = (instr >> 20) & 0xFFu;
    const u32 o2 = (instr >> 5) & 7u;
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const int rs = static_cast<int>((instr >> 8) & 0xFu);
    const int rm = static_cast<int>(instr & 0xFu);

    switch (o1) {
        case 0x10u: case 0x12u: case 0x14u: case 0x16u:
            decode_media_a(instr, rd, rn, rs, rm, o1);
            break;
        case 0x60u:
            undefined("media 0110 0000");
            return;
        case 0x61u: case 0x62u: case 0x63u:
        case 0x65u: case 0x66u: case 0x67u:
            parallel_add_sub(instr, rd, rn, rm, o1, o2);
            break;
        case 0x68u:
            if ((instr & 0xFF0u) == 0xFB0u) {
                r[rd] = select_bytes(r[rn], r[rm], cpsr);
                break;
            }
            if ((o2 & 3u) == 0u) {
                const u32 shift = (instr >> 7) & 0x1Fu;
                r[rd] = (shift_imm(r[rn], 0, static_cast<int>(shift), false) & 0xFFFF0000u) |
                       (r[rm] & 0xFFFFu);
            } else if ((o2 & 3u) == 2u) {
                u32 shift = (instr >> 7) & 0x1Fu;
                if (shift == 0) shift = 32;
                r[rd] = (r[rn] & 0xFFFF0000u) |
                       (shift_imm(r[rm], 2, static_cast<int>(shift), false) & 0xFFFFu);
            } else if ((o2 & 3u) == 3u) {
                const u32 src = arm::rotate_right(r[rm], static_cast<int>((instr >> 10) & 3u) * 8);
                u32 ext = static_cast<u32>(static_cast<s16>(src & 0xFFFFu));
                ext |= static_cast<u32>(static_cast<s16>(src >> 16)) << 16;
                r[rd] = rn == 15 ? ext : ext + r[rn];
            } else {
                undefined("media 0110 1000 op2");
                return;
            }
            break;
        case 0x6Au:
        case 0x6Bu:
            decode_sat_extend(instr, rd, rn, rm, o1, o2);
            break;
        case 0x6Cu:
            if ((o2 & 3u) == 3u) {
                const u32 src = arm::rotate_right(r[rm], static_cast<int>((instr >> 10) & 3u) * 8);
                const u32 ext = (src & 0xFFFFu) | ((src >> 16) << 16);
                r[rd] = rn == 15 ? ext : ext + r[rn];
            } else {
                undefined("media 0110 1100 op2");
                return;
            }
            break;
        case 0x6Eu:
        case 0x6Fu:
            decode_sat_extend(instr, rd, rn, rm, o1, o2);
            break;
        case 0x70u:
        case 0x74u:
            decode_smuad(instr, rd, rn, rs, rm, o1, o2);
            break;
        case 0x78u:
            if (o2 == 0u) {
                const bool a = (instr & (1u << 20)) != 0;
                s32 diff = 0;
                for (int k = 0; k < 4; ++k) {
                    const s32 b1 = static_cast<s32>((r[rm] >> (k * 8)) & 0xFFu);
                    const s32 b2 = static_cast<s32>((r[rs] >> (k * 8)) & 0xFFu);
                    diff += b1 > b2 ? b1 - b2 : b2 - b1;
                }
                u32 res = static_cast<u32>(diff);
                if (a) res += r[rn];
                r[rd] = res;
            } else if (o2 == 2u) {
                const bool sub = (instr & (1u << 20)) != 0;
                const s64 p1 = static_cast<s64>(static_cast<s16>(r[rm] & 0xFFFFu)) *
                               static_cast<s16>(r[rs] & 0xFFFFu);
                const s64 p2 = static_cast<s64>(static_cast<s16>(r[rm] >> 16)) *
                               static_cast<s16>(r[rs] >> 16);
                const s64 acc =
                    static_cast<s64>(static_cast<u64>(r[rn]) | (static_cast<u64>(r[rd]) << 32));
                const s64 res = sub ? acc - (p1 + p2) : acc + (p1 + p2);
                r[rn] = static_cast<u32>(static_cast<u64>(res));
                r[rd] = static_cast<u32>(static_cast<u64>(res) >> 32);
            } else {
                undefined("media 0111 1000 op2");
                return;
            }
            break;
        case 0x7Au: case 0x7Bu:
        case 0x7Cu: case 0x7Du: case 0x7Eu: case 0x7Fu:
            decode_bitfield(instr, rd, rn, rs);
            break;
        case 0x75u: case 0x76u: case 0x77u:
        case 0x71u: case 0x72u: case 0x73u:
            decode_smmul(instr, rd, rn, rs, rm, o1);
            break;
        default:
            undefined("media op1");
            return;
    }
    write_r15(cur_instr_addr_ + 4u);
}

void ArmCore::decode_media_a(u32 instr, int rd, int rn, int rs, int rm, u32 ga) {
    const u32 nib = instr & 0xF0u;
    const bool x = (instr & 0x20u) != 0u;
    const bool y = (instr & 0x40u) != 0u;
    const s32 mh = x ? static_cast<s16>(r[rm] >> 16) : static_cast<s16>(r[rm] & 0xFFFFu);
    const s32 sh = y ? static_cast<s16>(r[rs] >> 16) : static_cast<s16>(r[rs] & 0xFFFFu);

    if (nib == 0x50u) {
        const s64 rm_v = static_cast<s32>(r[rm]);
        const s64 rn_v = static_cast<s32>(r[rn]);
        s64 v;
        if (ga == 0x10u) {
            v = rm_v + rn_v;                        // QADD
        } else if (ga == 0x12u) {
            v = rm_v - rn_v;                        // QSUB
        } else {
            s64 dbl = 2LL * rn_v;                   // QDADD / QDSUB
            if (dbl > 2147483647LL) {
                dbl = 2147483647LL;
                cpsr |= arm::kFlagQ;
            } else if (dbl < -2147483648LL) {
                dbl = -2147483648LL;
                cpsr |= arm::kFlagQ;
            }
            v = (ga == 0x14u) ? rm_v + dbl : rm_v - dbl;
        }
        s32 res;
        if (v > 2147483647LL) {
            res = 2147483647;
            cpsr |= arm::kFlagQ;
        } else if (v < -2147483648LL) {
            res = -2147483647 - 1;
            cpsr |= arm::kFlagQ;
        } else {
            res = static_cast<s32>(v);
        }
        r[rd] = static_cast<u32>(res);
        return;
    }

    if ((nib & 0x90u) != 0x80u) {
        undefined("media 0001 0xx0");
        return;
    }

    switch (ga) {
        case 0x10u:  // SMLAxy
            r[rn] = static_cast<u32>(static_cast<s32>(r[rd]) + mh * sh);
            return;
        case 0x12u: {  // SMLAWy / SMULWy
            const s64 product = static_cast<s64>(mh) * static_cast<s32>(r[rs]);
            const s32 hi = static_cast<s32>(product >> 16);
            r[rn] = x ? static_cast<u32>(hi) : static_cast<u32>(static_cast<s32>(r[rd]) + hi);
            return;
        }
        case 0x14u: {  // SMLALxy
            const s64 acc = static_cast<s64>(static_cast<u64>(r[rd]) | (static_cast<u64>(r[rn]) << 32));
            const s64 res = acc + static_cast<s64>(mh) * sh;
            r[rd] = static_cast<u32>(static_cast<u64>(res));
            r[rn] = static_cast<u32>(static_cast<u64>(res) >> 32);
            return;
        }
        default:  // SMULxy
            r[rn] = static_cast<u32>(mh * sh);
            return;
    }
}

u32 ArmCore::select_bytes(u32 a, u32 b, u32 cpsr_value) {
    u32 result = 0;
    for (int k = 0; k < 4; ++k) {
        const bool ge = (cpsr_value & (0x10000000u >> k)) != 0;
        const u32 mask = 0xFFu << (k * 8);
        result |= ge ? (b & mask) : (a & mask);
    }
    return result;
}

void ArmCore::parallel_add_sub(u32 instr, int rd, int rn, int rm, u32 op1, u32 op2) {
    (void)instr;
    const bool is_unsigned = op1 >= 5u;
    const bool halving = (op1 == 3u || op1 == 7u);
    const bool saturating = (op1 == 2u || op1 == 6u);
    const u32 a = r[rn];
    const u32 b = r[rm];
    u32 result = 0;
    u32 ge = 0;

    const bool is8 = (op2 == 4u || op2 == 7u);
    const bool is_asx = (op2 == 1u);
    const bool is_sax = (op2 == 2u);
    const bool subtract = (op2 == 3u || op2 == 7u);

    if (is8) {
        for (int k = 0; k < 4; ++k) {
            s32 x = static_cast<s32>((a >> (k * 8)) & 0xFFu);
            s32 y = static_cast<s32>((b >> (k * 8)) & 0xFFu);
            if (!is_unsigned) {
                x = static_cast<s8>(x);
                y = static_cast<s8>(y);
            }
            s32 res = subtract ? x - y : x + y;
            if (saturating) res = arm::saturate_signed(res, 8);
            else if (halving) res >>= 1;
            result |= static_cast<u32>(res & 0xFF) << (k * 8);
            if (res >= 0) ge |= 0x10000000u >> k;
        }
    } else if (is_asx || is_sax) {
        s32 a0 = static_cast<s32>(a & 0xFFFFu);
        s32 a1 = static_cast<s32>(a >> 16);
        s32 b0 = static_cast<s32>(b & 0xFFFFu);
        s32 b1 = static_cast<s32>(b >> 16);
        if (!is_unsigned) {
            a0 = static_cast<s16>(a0);
            a1 = static_cast<s16>(a1);
            b0 = static_cast<s16>(b0);
            b1 = static_cast<s16>(b1);
        }
        s32 r0;
        s32 r1;
        if (is_asx) {
            if (subtract) {
                r0 = a0 - b1;
                r1 = a1 - b0;
            } else {
                r0 = a0 + b1;
                r1 = a1 + b0;
            }
        } else {
            if (subtract) {
                r0 = a0 - b0;
                r1 = a1 - b1;
            } else {
                r0 = a0 + b0;
                r1 = a1 + b1;
            }
        }
        if (saturating) {
            r0 = arm::saturate_signed(r0, 16);
            r1 = arm::saturate_signed(r1, 16);
        } else if (halving) {
            r0 >>= 1;
            r1 >>= 1;
        }
        result = static_cast<u32>(((r1 & 0xFFFF) << 16) | (r0 & 0xFFFF));
    } else {
        s32 a0 = static_cast<s32>(a & 0xFFFFu);
        s32 a1 = static_cast<s32>(a >> 16);
        s32 b0 = static_cast<s32>(b & 0xFFFFu);
        s32 b1 = static_cast<s32>(b >> 16);
        if (!is_unsigned) {
            a0 = static_cast<s16>(a0);
            a1 = static_cast<s16>(a1);
            b0 = static_cast<s16>(b0);
            b1 = static_cast<s16>(b1);
        }
        s32 r0;
        s32 r1;
        if (subtract) {
            r0 = a0 - b0;
            r1 = a1 - b1;
        } else {
            r0 = a0 + b0;
            r1 = a1 + b1;
        }
        if (saturating) {
            r0 = arm::saturate_signed(r0, 16);
            r1 = arm::saturate_signed(r1, 16);
        } else if (halving) {
            r0 >>= 1;
            r1 >>= 1;
        }
        result = static_cast<u32>(((r1 & 0xFFFF) << 16) | (r0 & 0xFFFF));
    }

    r[rd] = result;
    if (is8) cpsr = (cpsr & 0x0FFFFFFFu) | ge;
}

void ArmCore::decode_sat_extend(u32 instr, int rd, int rn, int rm, u32 op1, u32 op2) {
    (void)op2;
    const bool is_unsigned = (op1 & 4u) != 0u;
    const bool half = (op1 & 1u) != 0u;
    const u32 sel = (instr >> 5) & 3u;

    if (sel == 3u) {
        // SXTB/SXTH/UXTB/UXTH (Rn == 1111) and SXTAB/SXTAH/UXTAB/UXTAH.
        const int rot = static_cast<int>((instr >> 10) & 3u) * 8;
        const u32 src = arm::rotate_right(r[rm], rot);
        u32 ext;
        if (half) {
            ext = is_unsigned ? (src & 0xFFFFu)
                              : static_cast<u32>(static_cast<s32>(static_cast<s16>(src & 0xFFFFu)));
        } else {
            ext = is_unsigned ? (src & 0xFFu)
                              : static_cast<u32>(static_cast<s32>(static_cast<s8>(src & 0xFFu)));
        }
        r[rd] = rn == 15 ? ext : ext + r[rn];
        return;
    }
    if ((sel & 1u) != 0u && ((instr >> 7) & 0x1Fu) == 0x1Eu) {
        // SSAT16 / USAT16.
        const int sat16 = static_cast<int>((instr >> 16) & 0xFu);
        u32 v = r[rm] & 0xFFFFu;
        if (is_unsigned) {
            const u32 max = sat16 >= 16 ? 0xFFFFu : ((1u << sat16) - 1u);
            if (v > max) {
                v = max;
                cpsr |= arm::kFlagQ;
            }
        } else {
            const int bits16 = sat16 + 1;
            s32 sv = static_cast<s16>(v);
            const s32 max = (1 << (bits16 - 1)) - 1;
            const s32 min = -(1 << (bits16 - 1));
            if (sv > max) {
                sv = max;
                cpsr |= arm::kFlagQ;
            } else if (sv < min) {
                sv = min;
                cpsr |= arm::kFlagQ;
            }
            v = static_cast<u32>(sv & 0xFFFF);
        }
        r[rd] = v;
        return;
    }
    {
        // SSAT / USAT.
        const u32 sat_imm = (instr >> 16) & 0x1Fu;
        const bool asr = (instr & 0x40u) != 0;
        const int shift = static_cast<int>((instr >> 7) & 0x1Fu);
        const u32 value = shift_imm(r[rm], asr ? 2 : 0, shift, false);
        s32 sat;
        if (is_unsigned) {
            const s32 max = static_cast<s32>(sat_imm) >= 32 ? 0x7FFFFFFF
                                                            : (1 << static_cast<s32>(sat_imm)) - 1;
            const s64 lv = static_cast<s32>(value);
            if (lv > max) {
                sat = max;
                cpsr |= arm::kFlagQ;
            } else if (lv < 0) {
                sat = 0;
                cpsr |= arm::kFlagQ;
            } else {
                sat = static_cast<s32>(lv);
            }
        } else {
            sat = arm::saturate_signed(static_cast<s32>(value), static_cast<unsigned>(sat_imm) + 1u);
        }
        r[rd] = static_cast<u32>(sat);
        return;
    }
}

void ArmCore::decode_smuad(u32 instr, int rd, int rn, int rs, int rm, u32 op1, u32 op2) {
    const bool a = (instr & (1u << 20)) != 0;
    const bool x = (instr & 0x20u) != 0;
    const bool y = (instr & 0x40u) != 0;
    const bool alternating = op1 == 0x74u;
    s32 m0 = x ? static_cast<s16>(r[rm] >> 16) : static_cast<s16>(r[rm] & 0xFFFFu);
    s32 m1 = x ? static_cast<s16>(r[rm] & 0xFFFFu) : static_cast<s16>(r[rm] >> 16);
    const s32 s0 = y ? static_cast<s16>(r[rs] >> 16) : static_cast<s16>(r[rs] & 0xFFFFu);
    const s32 s1 = y ? static_cast<s16>(r[rs] & 0xFFFFu) : static_cast<s16>(r[rs] >> 16);
    if (alternating) {
        const s32 t = m0;
        m0 = m1;
        m1 = t;
    }

    const s64 p1 = static_cast<s64>(m0) * s0;
    const s64 p2 = static_cast<s64>(m1) * s1;
    s64 res = (op2 == 1u || op2 == 3u || op2 == 5u || op2 == 7u) ? (p1 - p2) : (p1 + p2);
    if (a) res += static_cast<s32>(r[rn]);
    const bool overflow = res > 2147483647LL || res < -2147483648LL;
    if (overflow) cpsr |= arm::kFlagQ;
    else cpsr &= ~arm::kFlagQ;
    r[rd] = static_cast<u32>(static_cast<s32>(res));
}

void ArmCore::decode_smmul(u32 instr, int rd, int rn, int rs, int rm, u32 op1) {
    const bool round = (instr & 0x20u) != 0;
    const s64 product = static_cast<s64>(static_cast<s32>(r[rm])) * static_cast<s32>(r[rs]);
    s64 res = product;
    const u32 kind = (op1 >> 1) & 3u;  // 0 = SMMUL, 2 = SMMLA, 3 = SMMLS
    if (kind == 2u) res = product + static_cast<s32>(r[rn]);
    else if (kind == 3u) res = static_cast<s32>(r[rn]) - product;
    if (round) res += 0x80000000LL;
    r[rd] = static_cast<u32>(static_cast<s32>(res >> 32));
}

void ArmCore::decode_bitfield(u32 instr, int rd, int rn, int rs) {
    (void)rn;
    (void)rs;
    const int lsb = static_cast<int>((instr >> 7) & 0x1Fu);
    const int field = static_cast<int>((instr >> 16) & 0x1Fu);
    const int rm = static_cast<int>(instr & 0xFu);
    const u32 o1 = (instr >> 20) & 0xFFu;
    const u32 sel = (instr >> 4) & 7u;

    if (sel == 1u) {
        // BFI / BFC.
        const int width = field - lsb + 1;
        if (width < 1 || lsb + width > 32) {
            undefined("BFI/BFC width");
            return;
        }
        const u32 mask = width >= 32 ? 0xFFFFFFFFu : ((1u << width) - 1u);
        const u32 src = rm == 15 ? 0u : r[rm];
        r[rd] = (r[rd] & ~(mask << lsb)) | ((src & mask) << lsb);
        return;
    }
    if (sel == 5u) {
        // SBFX / UBFX.
        const bool is_unsigned = (o1 & 4u) != 0u;
        const int width = field + 1;
        if (width <= 0 || width > 32 || lsb + width > 32) {
            undefined("bitfield width");
            return;
        }
        const u32 shifted = r[rm] >> lsb;
        u32 value = width >= 32 ? shifted : (shifted & ((1u << width) - 1u));
        if (!is_unsigned && width < 32 && (value & (1u << (width - 1))) != 0) {
            value |= ~((1u << width) - 1u);
        }
        r[rd] = value;
        return;
    }
    undefined("bitfield encoding");
}

// ---- misc (bits [27:23] = 00010/00011/00110/00111) -------------------------

bool ArmCore::is_arm_misc_encoding(u32 instr) {
    if ((instr & 0x0FBF0FFFu) == 0x010F0000u) return true;  // MRS
    if ((instr & 0x0FB0FFF0u) == 0x0120F000u) return true;  // MSR (register)
    // SMC / HVC live in the same space (bits [27:24] = 0001, op1 = 0x16/0x14) and
    // must reach decode_arm_misc before the MRS/MSR tests there.
    if ((instr & 0x0FF000F0u) == 0x01600070u) return true;  // SMC
    if ((instr & 0x0FF000F0u) == 0x01400070u) return true;  // HVC
    if ((instr & 0x0FFFFFF0u) == 0x012FFF10u) return true;  // BX
    if ((instr & 0x0FFFFFF0u) == 0x012FFF20u) return true;  // BXJ
    if ((instr & 0x0FFFFFF0u) == 0x012FFF30u) return true;  // BLX
    return false;
}

void ArmCore::decode_arm_misc(u32 instr) {
    const u32 o1 = (instr >> 20) & 0xFFu;
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    const int rm = static_cast<int>(instr & 0xFu);

    // BX / BLX / BXJ must be tested before MRS/MSR (they share bits [27:20]).
    if ((instr & 0x0FFFFFF0u) == 0x012FFF10u || (instr & 0x0FFFFFF0u) == 0x012FFF20u ||
        (instr & 0x0FFFFFF0u) == 0x012FFF30u) {
        const u32 target = read_reg(rm);
        if ((instr & 0x30u) == 0x30u) r[14] = cur_instr_addr_ + 4u;  // BLX
        branch_to(target);
        return;
    }
    // HVC / SMC first: they share bits [27:20] with MRS/MSR below.
    if ((instr & 0x0FF000F0u) == 0x01400070u) {
        ZLB_LOG_DBG("cpu", "%s HVC #%u -> hypervisor at %08X", name.c_str(), instr & 0xFu, cur_instr_addr_);
        take_exception(arm::kVecHyp, arm::kModeHyp, cur_instr_addr_ + 4u);
        return;
    }
    if ((instr & 0x0FF000F0u) == 0x01600070u) {
        // SMC: the secure monitor call. It enters monitor mode through the
        // monitor vector table (MVBAR + 8) with LR_mon = the next instruction.
        ZLB_LOG_DBG("cpu", "%s SMC #%u -> monitor (MVBAR=0x%08X, %s world) at %08X", name.c_str(),
                    instr & 0xFu, mvbar, secure_state() ? "secure" : "non-secure", cur_instr_addr_);
        take_exception(arm::kVecMonitorSmc, arm::kModeMonitor, cur_instr_addr_ + 4u);
        return;
    }
    if (o1 == 0x10u || o1 == 0x14u) {
        const bool use_spsr = (instr & (1u << 22)) != 0;
        r[rd] = use_spsr ? spsr() : cpsr;
        write_r15(cur_instr_addr_ + 4u);
        return;
    }
    if (o1 == 0x12u || o1 == 0x16u) {
        msr_arm(instr, read_reg(rm));
        return;
    }

    if ((instr & 0x10u) != 0) {
        if ((instr & 0x0FFF0FF0u) == 0x016F0F10u) {
            r[rd] = arm::count_leading_zeros(r[rm]);
            if (rd == 15) {
                branch_to(r[15]);
                return;
            }
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        if ((instr & 0xF10u) == 0x100u) {
            decode_arm_extra_load_store(instr);
            return;
        }
        undefined("misc bit4 group");
        return;
    }
    undefined("misc encoding");
}

void ArmCore::msr_arm(u32 instr, u32 value) {
    const bool use_spsr = (instr & (1u << 22)) != 0;
    const u32 fields = (instr >> 16) & 0xFu;
    u32 mask = 0;
    if ((fields & 8u) != 0) mask |= 0xFF000000u;
    if ((fields & 4u) != 0) mask |= 0x00FF0000u;
    if ((fields & 2u) != 0) mask |= 0x0000FF00u;
    if ((fields & 1u) != 0) mask |= 0x000000FFu;
    if (mask == 0u) {
        undefined("MSR with an empty field mask");
        return;
    }

    if (use_spsr) {
        if (mode() == arm::kModeUser) {
            undefined("MSR SPSR from User mode");
            return;
        }
        set_spsr((spsr() & ~mask) | (value & mask));
    } else {
        write_cpsr_masked(value, mask);
    }
    write_r15(cur_instr_addr_ + 4u);
}

// ---- extra load/store (halfword, signed, doubleword) ----------------------

void ArmCore::decode_arm_extra_load_store(u32 instr) {
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const bool p = (instr & (1u << 24)) != 0;
    const bool u = (instr & (1u << 23)) != 0;
    const bool immediate = (instr & (1u << 22)) != 0;
    const bool w = (instr & (1u << 21)) != 0;
    const bool l = (instr & (1u << 20)) != 0;
    const u32 op2v = (instr >> 5) & 3u;

    u32 offset;
    if (immediate) offset = (((instr >> 8) & 0xFu) << 4) | (instr & 0xFu);
    else offset = r[instr & 0xFu];
    if (!u) offset = static_cast<u32>(-static_cast<s32>(offset));

    const u32 base_addr = read_reg(rn);
    const u32 addr = p ? (base_addr + offset) : base_addr;
    const u32 writeback = base_addr + offset;

    // bits [6:5] are (S, H).
    if (!l && (op2v == 2u || op2v == 3u)) {
        const bool pair_load = (op2v == 2u);
        if (pair_load) {
            const u64 value = mem_read_double(addr);
            if (pending_fault_ != arm::FaultKind::None) return;
            if (rd == 15) {
                undefined("LDRD with Rd == PC");
                return;
            }
            if (rd == 14) {
                r[14] = static_cast<u32>(value);
                r[0] = static_cast<u32>(value >> 32);
            } else {
                r[rd] = static_cast<u32>(value);
                r[rd + 1] = static_cast<u32>(value >> 32);
            }
        } else {
            if (rd == 15 || rd == 14) {
                undefined("STRD with an invalid register pair");
                return;
            }
            const u64 value = static_cast<u64>(r[rd]) | (static_cast<u64>(r[rd + 1]) << 32);
            mem_write_double(addr, value);
            if (pending_fault_ != arm::FaultKind::None) return;
        }
        if (w && rn != 15) r[rn] = writeback;
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    const bool signed_load = (op2v == 2u) || (op2v == 3u);
    const bool half = (op2v == 1u) || (op2v == 3u);

    if (l) {
        const u32 raw = half ? mem_read_half(addr) : mem_read_byte(addr);
        if (pending_fault_ != arm::FaultKind::None) return;
        u32 value;
        if (signed_load) {
            value = half ? static_cast<u32>(static_cast<s32>(static_cast<s16>(raw & 0xFFFFu)))
                         : static_cast<u32>(static_cast<s32>(static_cast<s8>(raw & 0xFFu)));
        } else {
            value = raw & 0xFFFFu;
        }
        write_reg(rd, value);
        if (rd == 15) {
            branch_to(r[15]);
            return;
        }
    } else {
        if (half) mem_write_half(addr, r[rd] & 0xFFFFu);
        else mem_write_byte(addr, r[rd] & 0xFFu);
        if (pending_fault_ != arm::FaultKind::None) return;
    }

    if (w) r[rn] = writeback;
    write_r15(cur_instr_addr_ + 4u);
}

// ---- load/store ------------------------------------------------------------

void ArmCore::execute_arm_load_store(u32 instr, bool immediate) {
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const bool p = (instr & (1u << 24)) != 0;
    const bool u = (instr & (1u << 23)) != 0;
    const bool b = (instr & (1u << 22)) != 0;
    const bool w = (instr & (1u << 21)) != 0;
    const bool l = (instr & (1u << 20)) != 0;

    u32 offset;
    if (immediate) {
        offset = instr & 0xFFFu;
    } else {
        const u32 rm_val = r[instr & 0xFu];
        const int type = static_cast<int>((instr >> 5) & 3u);
        const int amount = static_cast<int>((instr >> 7) & 0x1Fu);
        offset = shift_imm(rm_val, type, amount, false);
    }
    if (!u) offset = static_cast<u32>(-static_cast<s32>(offset));

    const u32 base_addr = read_reg(rn);
    const u32 addr = p ? (base_addr + offset) : base_addr;
    const u32 writeback = base_addr + offset;

    if (l) {
        if (b) {
            const u32 value = mem_read_byte(addr);
            if (pending_fault_ != arm::FaultKind::None) return;
            write_reg(rd, value & 0xFFu);
        } else {
            const bool unaligned = (addr & 3u) != 0;
            if (unaligned && mmu.enabled() && mmu.strict_alignment()) {
                arm::MmResult mm;
                mm.fault = arm::MmFaultKind::Alignment;
                mm.fsr_full = 1u;
                mm.fsr_status = 1u;
                mmu.report_data_abort(mm, addr, false);
                pending_mm_fault_ = mm;
                pending_fault_ = arm::FaultKind::Data;
                return;
            }
            u32 value = mem_read_word(addr & ~3u, false);
            if (pending_fault_ != arm::FaultKind::None) return;
            if (unaligned) {
                const int rot = static_cast<int>(addr & 3u) * 8;
                value = (value >> rot) | (value << (32 - rot));
            }
            write_reg(rd, value);
        }
        if (rd == 15) {
            branch_to(r[15]);
            return;
        }
    } else {
        if (b) mem_write_byte(addr, r[rd] & 0xFFu);
        else mem_write_word(addr & ~3u, r[rd]);
        if (pending_fault_ != arm::FaultKind::None) return;
    }

    if (w) r[rn] = writeback;
    write_r15(cur_instr_addr_ + 4u);
}

void ArmCore::execute_arm_load_store_multiple(u32 instr) {
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const bool p = (instr & (1u << 24)) != 0;
    const bool u = (instr & (1u << 23)) != 0;
    const bool s = (instr & (1u << 22)) != 0;
    const bool w = (instr & (1u << 21)) != 0;
    const bool l = (instr & (1u << 20)) != 0;
    const u32 list = instr & 0xFFFFu;

    if (list == 0u) {
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    const u32 base_addr = read_reg(rn);
    const int count = arm::popcount32(list);
    // The lowest (or highest, for a descending mode) address accessed:
    //   IA (P=0,U=1): base ..            IB (P=1,U=1): base+4 ..
    //   DA (P=0,U=0): base-4n ..         DB (P=1,U=0): base-4n ..
    // The reference core used `if (p == l) start += 4`, which is right for
    // LDMIA/LDMIB/LDMDA/STMDA but wrong for STMIA and STMIB.
    u32 start = u ? base_addr : base_addr - static_cast<u32>(count * 4);
    if (p != 0u && u) start += 4u;
    const u32 new_base = u ? base_addr + static_cast<u32>(count * 4)
                           : base_addr - static_cast<u32>(count * 4);

    const bool user_bank = s && !(l && (list & 0x8000u) != 0);
    u32 saved_sp = 0;
    u32 saved_lr = 0;
    if (user_bank) {
        saved_sp = r[13];
        saved_lr = r[14];
        r[13] = bank_sp_[arm::kModeUser];
        r[14] = bank_lr_[arm::kModeUser];
    }

    u32 addr = start;
    if (l) {
        for (int k = 0; k < 16; ++k) {
            if ((list & (1u << k)) == 0) continue;
            const u32 value = mem_read_word(addr, false);
            if (pending_fault_ != arm::FaultKind::None) break;
            if (k == 15) r[15] = value;
            else r[k] = value;
            addr += 4u;
        }
    } else {
        const u32 pc_value = arm::read_pc_value(false, cur_instr_addr_);
        for (int k = 0; k < 16; ++k) {
            if ((list & (1u << k)) == 0) continue;
            mem_write_word(addr, k == 15 ? pc_value : r[k]);
            if (pending_fault_ != arm::FaultKind::None) break;
            addr += 4u;
        }
    }

    if (user_bank) {
        if (l) {
            bank_sp_[arm::kModeUser] = r[13];
            bank_lr_[arm::kModeUser] = r[14];
        }
        r[13] = saved_sp;
        r[14] = saved_lr;
    }

    if (pending_fault_ != arm::FaultKind::None) return;

    if (l && (list & 0x8000u) != 0) {
        const u32 target = r[15];
        if (s) {
            exception_return(target);
            return;
        }
        branch_to(target);
        return;
    }

    if (w && rn != 15) r[rn] = new_base;
    write_r15(cur_instr_addr_ + 4u);
}

// ---- branches --------------------------------------------------------------

void ArmCore::execute_arm_branch(u32 instr) {
    const bool link = (instr & (1u << 24)) != 0;
    s32 imm = static_cast<s32>(instr & 0xFFFFFFu);
    if ((imm & 0x800000) != 0) imm |= static_cast<s32>(0xFF000000u);
    const u32 target = cur_instr_addr_ + 8u + static_cast<u32>(imm << 2);
    if (link) r[14] = cur_instr_addr_ + 4u;
    write_r15(target);
}

// ---- unconditional space ---------------------------------------------------

void ArmCore::execute_arm_unconditional(u32 instr) {
    const u32 blk = (instr >> 25) & 7u;

    // 1111 101H: BLX (immediate). H is *bit 24* and contributes bit 1 of the
    // offset: target = PC + 8 + SignExtend(imm24:H:'0', 26), then word aligned.
    // (The reference core tested bit 4 as if it were H and dropped H from the
    // offset entirely, which rejects every BLX whose imm24 has bit 4 set --
    // including the one the kernel boot loader uses at 0x40020340.)
    if (blk == 5u) {
        const u32 h = (instr >> 24) & 1u;
        u32 imm = (instr & 0xFFFFFFu) << 2;
        imm |= h << 1;
        const s32 offset = arm::sign_extend32(imm, 26);
        const u32 target = cur_instr_addr_ + 8u + static_cast<u32>(offset);
        r[14] = cur_instr_addr_ + 4u;
        thumb = true;
        cpsr |= arm::kFlagT;
        write_r15(target & ~3u);
        return;
    }

    // 1111 110x / 1111 111x: coprocessor and VFP.
    if (blk == 6u || blk == 7u) {
        if ((instr & 0x10u) == 0u) {
            if (blk == 6u) {
                execute_arm_coprocessor(instr);
                return;
            }
            decode_arm_misc_uncond(instr);
            return;
        }
        if (blk == 6u) {
            execute_arm_coprocessor(instr);
            return;
        }
        execute_arm_vfp(instr);
        return;
    }

    // 1111 010x / 1111 011x.
    if (blk == 2u || blk == 3u) {
        // Advanced SIMD element / structure load-store lives at 1111 0100.
        // This is checked before the barrier encodings because those are
        // 1111 0101 (CLREX / DSB / DMB / ISB), which differs in bit 24.
        if ((instr & 0x0F000000u) == 0x04000000u) {
            execute_neon(instr);
            return;
        }
        // Barriers and the exclusive monitor clear: CLREX / DSB / DMB / ISB.
        // The reference core has a handler for these but dispatches them into a
        // lane it never reaches, so firmware that relies on them (the kernel
        // boot loader does) cannot run; the tests below are the encodings from
        // ARM ARM A8.8 that actually live in this lane.
        if ((instr & 0xFFFFFF00u) == 0xF57FF000u || (instr & 0xFFFFFFF0u) == 0xF57FF010u) {
            decode_arm_misc_uncond(instr);
            return;
        }
        if ((instr & 0x0FE00000u) == 0x0F800000u && (instr & 0x10u) == 0u) {
            decode_arm_cps_setend(instr);
            return;
        }
        if ((instr & 0x0F000000u) == 0x09000000u) {
            decode_arm_rfe_srs(instr);
            return;
        }
        if ((instr & 0x0D700000u) == 0x05500000u || (instr & 0x0D200000u) == 0x05000000u) {
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        undefined("unconditional encoding");
        return;
    }

    // 1111 000x / 1111 001x.
    if (blk == 0u || blk == 1u) {
        // 1111 001x (op1 == 01) is the Advanced SIMD "data processing" space
        // (VAND/VORR/VADD/... and the unconditional-only relatives). 1111 000x
        // is *not* NEON here: it holds CPS / SETEND / hints and the memory
        // hints, which are decoded further down.
        if ((instr & 0x0E000000u) == 0x02000000u) {
            execute_neon(instr);
            return;
        }
        if ((instr & 0x0D700000u) == 0x05500000u || (instr & 0x0D200000u) == 0x05000000u) {
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        if ((instr & 0x0F000000u) == 0x09000000u) {
            decode_arm_rfe_srs(instr);
            return;
        }
        // CPS / SETEND live at 1111 0001 0000 .... with bit 4 clear:
        // `(instr & FE700010) == F1000000`. Bit 24 is not part of the ARMv7-A
        // encoding so it is masked out here (the reference core leaves it in and
        // therefore never reaches its own CPS/SETEND handler for real code).
        if ((instr & 0xFF300020u) == 0xF1000000u) {
            decode_arm_cps_setend(instr);
            return;
        }
        undefined("unconditional encoding");
        return;
    }

    // 1111 100x: RFE/SRS.
    if (blk == 4u) {
        decode_arm_rfe_srs(instr);
        return;
    }

    undefined("unconditional encoding");
}

void ArmCore::decode_arm_misc_uncond(u32 instr) {
    // CLREX (1111 0101 0111 1111 1111 0000 0001 1111).
    if ((instr & 0xFFFFFFF0u) == 0xF57FF010u) {
        exclusive_valid_ = false;
        bus->clear_exclusive_for(static_cast<int>(core_id_));
        write_r15(cur_instr_addr_ + 4u);
        return;
    }
    // DSB/DMB/ISB (1111 0101 0111 1111 1111 0000 0xxx xxxx).
    if ((instr & 0xFFFFFF00u) == 0xF57FF000u) {
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    // BX / BLX / BXJ.
    if ((instr & 0x0FFFFFF0u) == 0x012FFF10u) {
        branch_to(read_reg(static_cast<int>(instr & 0xFu)));
        return;
    }
    if ((instr & 0x0FFFFFF0u) == 0x012FFF20u) {
        r[14] = cur_instr_addr_ + 4u;
        branch_to(read_reg(static_cast<int>(instr & 0xFu)));
        return;
    }
    if ((instr & 0x0FFFFFF0u) == 0x012FFF30u) {
        branch_to(read_reg(static_cast<int>(instr & 0xFu)));
        return;
    }

    // HVC / SMC are handled at the top of decode_arm_misc (they share bits
    // [27:20] with MRS/MSR).

    undefined("unconditional misc");
}

void ArmCore::decode_arm_rfe_srs(u32 instr) {
    const bool p = (instr & (1u << 24)) != 0;
    const bool u = (instr & (1u << 23)) != 0;
    const bool w = (instr & (1u << 21)) != 0;
    const bool rfe = (instr & (1u << 20)) != 0;
    const int rn = static_cast<int>((instr >> 16) & 0xFu);

    if (rfe) {
        // RFE: load PC and CPSR from the exception return state.
        const u32 base_addr = read_reg(rn);
        u32 addr;
        if (u) addr = p ? base_addr + 4u : base_addr;
        else addr = p ? base_addr - 4u : base_addr - 8u;
        const u32 new_cpsr = mem_read_word(addr, false);
        if (pending_fault_ != arm::FaultKind::None) return;
        const u32 new_pc = mem_read_word(addr + 4u, false);
        if (pending_fault_ != arm::FaultKind::None) return;
        if (w && rn != 15) r[rn] = u ? base_addr + 8u : base_addr - 8u;
        exception_return_with_cpsr(new_pc & ~3u, new_cpsr);
        return;
    }

    // SRS: store the return state using the mode in bits [4:0].
    if (mode() == arm::kModeUser) {
        undefined("SRS from User mode");
        return;
    }
    const u32 target_mode = instr & 0x1Fu;
    u32 saved_cpsr;
    u32 saved_lr;
    if (target_mode == arm::kModeUser || target_mode == arm::kModeSystem) {
        saved_cpsr = cpsr;
        saved_lr = r[14];
    } else {
        saved_cpsr = bank_spsr_[target_mode];
        saved_lr = bank_lr_[target_mode];
    }
    const u32 base_value = read_reg(rn);
    u32 addr;
    if (u) addr = p ? base_value + 4u : base_value;
    else addr = p ? base_value - 4u : base_value - 8u;
    mem_write_word(addr, saved_cpsr);
    if (pending_fault_ != arm::FaultKind::None) return;
    mem_write_word(addr + 4u, saved_lr);
    if (pending_fault_ != arm::FaultKind::None) return;
    if (w && rn != 15) r[rn] = u ? base_value + 8u : base_value - 8u;
    write_r15(cur_instr_addr_ + 4u);
}

void ArmCore::decode_arm_cps_setend(u32 instr) {
    // SETEND: 1111 0001 0000 0001 0000 0000 0 E 0 0.
    if ((instr & 0xFFFFFDFFu) == 0xF1010000u) {
        if ((instr & (1u << 9)) != 0) cpsr |= arm::kFlagE;
        else cpsr &= ~arm::kFlagE;
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    // CPS/CPSIE/CPSID: 1111 0001 0000 imod(2) M(1) 0 AIF(3) mode(5).
    //
    // Note: the reference core tests `(instr & 0xFFF10020) == 0xF1000000` /
    // `== 0xF1000020`; that mask does not clear bit 24, which is *not* part of
    // the ARMv7-A CPS encoding (M is bit 17). Real firmware's `CPSID IF`
    // (0xF10C00C0) and `CPS #0x1B` (0xF10E00DB, used to set up the per-mode
    // stacks) are rejected by that test, which is why the reference cannot run
    // the kernel boot loader. Masking bit 24 keeps the reference's intent and
    // accepts every ARMv7-A encoding.
    const bool cps = (instr & 0xFF300020u) == 0xF1000000u;
    if (cps) {
        const u32 imod = (instr >> 18) & 3u;
        const u32 m = (instr >> 17) & 1u;
        const u32 aif = (instr >> 6) & 7u;
        const u32 new_mode = instr & 0x1Fu;
        if (mode() == arm::kModeUser) {
            undefined("CPS from User mode");
            return;
        }

        if (m != 0 && new_mode != 0) {
            bank_switch(mode(), new_mode);
            cpsr = (cpsr & ~arm::kModeMask) | (new_mode & arm::kModeMask);
            thumb = (cpsr & arm::kFlagT) != 0;
        }
        if (imod == 2u) {
            if ((aif & 4u) != 0) cpsr &= ~arm::kFlagA;
            if ((aif & 2u) != 0) cpsr &= ~arm::kFlagI;
            if ((aif & 1u) != 0) cpsr &= ~arm::kFlagF;
        } else if (imod == 3u) {
            if ((aif & 4u) != 0) cpsr |= arm::kFlagA;
            if ((aif & 2u) != 0) cpsr |= arm::kFlagI;
            if ((aif & 1u) != 0) cpsr |= arm::kFlagF;
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    // Hint instructions: 1111 0011 0010 0000 1111 0000 0000 xxxx.
    if ((instr & 0xFFFFFFF0u) == 0xE320F000u) {
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    undefined("CPS/SETEND/hint");
}

// ---- coprocessor (A32) -----------------------------------------------------

void ArmCore::execute_arm_coprocessor(u32 instr) {
    const u32 cpnum = (instr >> 8) & 0xFu;
    const u32 opc1v = (instr >> 21) & 7u;
    const u32 crn = (instr >> 16) & 0xFu;
    const u32 crm = instr & 0xFu;
    const u32 opc2v = (instr >> 5) & 7u;
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    const u32 blk = (instr >> 25) & 7u;

    if (blk == 6u) {
        // bits [24:21] == 0100 selects MCRR/MRRC; everything else is LDC/STC
        // (or a VFP load/store for coprocessors 10/11).
        if ((instr & 0x01E00000u) == 0x00400000u) {
            u32 rt2 = (instr >> 16) & 0xFu;
            const bool load = (instr & (1u << 20)) != 0;
            u32 rt = static_cast<u32>(rd);
            const u32 mcrr_opc1 = (instr >> 4) & 0xFu;
            if (cpnum == 15) {
                undefined("MCRR/MRRC p15");
                return;
            }
            if (coprocessor_hook != nullptr &&
                (*coprocessor_hook)(true, load, cpnum, mcrr_opc1, crn, crm, opc2v, rt, rt2)) {
                if (load) {
                    r[rd] = rt;
                    r[rt2] = rt2;
                }
                write_r15(cur_instr_addr_ + 4u);
                return;
            }
            ZLB_LOG_DBG("cpu", "%s MCRR/MRRC p%u ignored", name.c_str(), cpnum);
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        if (cpnum == 10u || cpnum == 11u) {
            const bool single = cpnum == 10u;
            if ((instr & (1u << 24)) != 0u && (instr & (1u << 21)) == 0u) {
                execute_vfp_load_store(instr, cpnum, single);
            } else {
                execute_vfp_load_store_multiple(instr, single);
            }
            return;
        }
        ZLB_LOG_DBG("cpu", "%s LDC/STC p%u ignored", name.c_str(), cpnum);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    // 0xEE: bit 4 == 1 is MCR/MRC (CP15 included), bit 4 == 0 is CDP.
    if ((instr & 0x10u) != 0u) {
        if (cpnum == 15u) {
            const bool load = (instr & (1u << 20)) != 0;
            if (load) {
                const u32 value = cp15_read(opc1v, crn, crm, opc2v, rd);
                if (rd != 15) r[rd] = value;
            } else {
                cp15_write(opc1v, crn, crm, opc2v, rd == 15 ? 0u : r[rd]);
            }
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        if (cpnum == 10u || cpnum == 11u) {
            execute_arm_vfp(instr);
            return;
        }
        const bool load = (instr & (1u << 20)) != 0;
        if (coprocessor_hook != nullptr) {
            u32 rt = rd == 15 ? 0u : r[rd];
            u32 rt2 = 0;
            if ((*coprocessor_hook)(false, load, cpnum, opc1v, crn, crm, opc2v, rt, rt2)) {
                if (load && rd != 15) r[rd] = rt;
                write_r15(cur_instr_addr_ + 4u);
                return;
            }
        }
        ZLB_LOG_DBG("cpu", "%s MCR/MRC p%u ignored", name.c_str(), cpnum);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    if (cpnum == 10u || cpnum == 11u) {
        execute_arm_vfp(instr);
        return;
    }
    ZLB_LOG_DBG("cpu", "%s CDP p%u ignored", name.c_str(), cpnum);
    write_r15(cur_instr_addr_ + 4u);
}

u32 ArmCore::cp15_read(u32 opc1v, u32 crn, u32 crm, u32 opc2v, int rt) {
    (void)rt;
    if (opc1v != 0u) {
        // Opc1 selects the cache-level space (CCSIDR/CLIDR/CSSELR). Nothing reads
        // it yet, but silently answering 0 there would look like "no caches" to a
        // stage that sizes its cache maintenance from it, so say so in the log.
        ZLB_LOG_DBG("cpu", "%s CP15 read opc1=%u c0/%u/%u unmodelled -> 0", name.c_str(), opc1v, crm,
                    opc2v);
        return 0u;
    }

    switch (crn) {
        case 0u:
            // The identification registers are selected by CRm and Op2.
            switch (crm) {
                case 0u:
                    switch (opc2v) {
                        case 0u: return 0x410FC090u;  // MIDR: Cortex-A9 r0p0
                        // CTR: ARMv7 format 4, CWG/ERG = 4 words, DminLine = IminLine
                        // = 3 (8 words = 32 byte lines, as the Cortex-A9 has).
                        // This value is load bearing: kernel_boot_loader picks its
                        // cache-line round-up helper from CTR[20:16] (0 -> +31/BIC
                        // 0x1F, 1 -> +63/BIC 0x3F, see 0x4003B684/0x4003B68C), so a
                        // wrong line length sends it into a wild pointer.
                        case 1u: return 0x8444C003u;  // CTR
                        case 2u: return 0x80000000u;  // TCMTR
                        case 3u: return 0x00000000u;  // TLBTR
                        // MPIDR: one cluster, this core's Aff0 in bits [1:0].
                        case 5u: return 0x80000000u | (core_id_ & 0x3u);
                        default: return 0u;
                    }
                case 1u:
                    switch (opc2v) {
                        case 0u: return 0x000F0000u;  // ID_PFR0
                        case 1u: return 0x00000001u;  // ID_PFR1 (TrustZone)
                        case 2u: return 0x00000000u;  // ID_DFR0
                        case 3u: return 0x00000000u;  // ID_AFR0
                        case 4u: return 0x00100010u;  // ID_MMFR0
                        case 5u: return 0x00000000u;  // ID_MMFR1
                        case 6u: return 0x00000000u;  // ID_MMFR2
                        case 7u: return 0x00000000u;  // ID_MMFR3
                        default: return 0u;
                    }
                case 2u:
                    switch (opc2v) {
                        case 0u: return 0x01101110u;  // ID_ISAR0
                        case 1u: return 0x02111000u;  // ID_ISAR1
                        case 2u: return 0x32112110u;  // ID_ISAR2
                        case 3u: return 0x01101011u;  // ID_ISAR3
                        case 4u: return 0x21111010u;  // ID_ISAR4
                        case 5u: return 0x00000000u;  // ID_ISAR5
                        default: return 0u;
                    }
                default: return 0u;
            }
        case 1u:
            if (crm == 0u && opc2v == 0u) return mmu.sctlr;
            if (crm == 0u && opc2v == 2u) return 0u;  // CPACR
            if (crm == 1u && opc2v == 0u) return scr;  // SCR (TrustZone)
            if (crm == 1u && opc2v == 1u) return 0u;   // Secure Debug Enable Register
            if (crm == 1u && opc2v == 2u) return 0u;   // Non-secure Access Control Register
            return 0u;
        case 2u:
            if (crm == 0u && opc2v == 0u) return mmu.ttbr0;
            if (crm == 0u && opc2v == 1u) return mmu.ttbr1;
            if (crm == 0u && opc2v == 2u) return mmu.ttbcr;
            return 0u;
        case 3u:
            // DACR. The kernel boot loader brackets its VA->PA probing with a
            // save/restore of this register, so a read that answers 0 silently
            // disables every domain on the restore and aborts the boot.
            if (crm == 0u && opc2v == 0u) return mmu.dacr;
            return 0u;
        case 5u:
            if (crm == 0u && opc2v == 0u) return mmu.dfsr;
            if (crm == 0u && opc2v == 1u) return mmu.dfar;
            if (crm == 1u && opc2v == 0u) return mmu.ifsr;
            if (crm == 1u && opc2v == 1u) return mmu.ifar;
            if (crm == 1u && opc2v == 2u) return mmu.adfsr;
            if (crm == 1u && opc2v == 3u) return mmu.aifsr;
            return 0u;
        case 6u:
            if (crm == 0u && opc2v == 0u) return mmu.dfar;
            if (crm == 0u && opc2v == 2u) return mmu.ifar;
            return 0u;
        case 7u:
            // PAR holds the result of the last V2P (VA->PA) operation below.
            if (crm == 4u && opc2v == 0u) return mmu.par;
            return 0u;
        case 10u:
            if (crm == 2u && opc2v == 0u) return mmu.prrr;
            if (crm == 2u && opc2v == 1u) return mmu.nmrr;
            return 0u;
        case 12u:
            // VBAR is banked per world; MVBAR (secure only) is the monitor's.
            if (crm == 0u && opc2v == 0u) return secure_state() ? mmu.vbar : vbar_nonsecure;
            if (crm == 0u && opc2v == 1u) return mvbar;
            return 0u;
        case 13u:
            if (crm == 0u && opc2v == 1u) return mmu.context_idr;
            return 0u;
        default:
            return 0u;
    }
}

void ArmCore::cp15_write(u32 opc1v, u32 crn, u32 crm, u32 opc2v, u32 value) {
    switch (crn) {
        case 1u:
            if (crm == 0u && opc2v == 0u) {
                mmu.sctlr = value;
                ZLB_LOG_DBG("cpu", "%s SCTLR = 0x%08X (MMU %s) at %08X", name.c_str(), value,
                            mmu.enabled() ? "on" : "off", cur_instr_addr_);
            } else if (crm == 1u && opc2v == 0u) {
                // SCR: writing NS switches the world. Only secure privileged code
                // may write it (else the write is ignored).
                if (!secure_state()) {
                    ZLB_LOG_WARN("cpu", "%s SCR write 0x%08X ignored in non-secure world at %08X", name.c_str(),
                                 value, cur_instr_addr_);
                    break;
                }
                scr = value;
                ns_ = (value & arm::kScrNs) != 0u;
                ZLB_LOG_INFO("cpu", "%s SCR = 0x%08X -> %s world at %08X", name.c_str(), value,
                             secure_state() ? "secure" : "non-secure", cur_instr_addr_);
            }
            break;
        case 2u:
            if (crm == 0u && opc2v == 0u) {
                mmu.ttbr0 = value & 0xFFFFC000u;
                ZLB_LOG_DBG("cpu", "%s TTBR0 = 0x%08X (base 0x%08X) at %08X", name.c_str(), value, mmu.ttbr0,
                            cur_instr_addr_);
            } else if (crm == 0u && opc2v == 1u) {
                mmu.ttbr1 = value;
                int num = 0;
                ZLB_LOG_DBG("cpu", "%s TTBR1 = 0x%08X (base 0x%08X for TTBCR.N=%u) at %08X", name.c_str(),
                            value, mmu.ttbr_base_for(0xFFFFFFFFu, num), mmu.ttbcr & 7u, cur_instr_addr_);
            } else if (crm == 0u && opc2v == 2u) {
                mmu.ttbcr = value & 7u;
                ZLB_LOG_DBG("cpu", "%s TTBCR = 0x%08X (N=%u, split at 0x%08X) at %08X", name.c_str(), value,
                            mmu.ttbcr, mmu.ttbcr == 0 ? 0u : (1u << (32 - mmu.ttbcr)), cur_instr_addr_);
            }
            break;
        case 3u:
            mmu.dacr = value;
            ZLB_LOG_DBG("cpu", "%s DACR = 0x%08X at %08X", name.c_str(), value, cur_instr_addr_);
            break;
        case 5u:
            if (crm == 0u && opc2v == 0u) mmu.dfsr = value & 0xFFFu;
            else if (crm == 0u && opc2v == 1u) mmu.dfar = value;
            else if (crm == 1u && opc2v == 0u) mmu.ifsr = value & 0xFFFu;
            else if (crm == 1u && opc2v == 1u) mmu.ifar = value;
            break;
        case 6u:
            if (crm == 0u && opc2v == 0u) mmu.dfar = value;
            else if (crm == 0u && opc2v == 2u) mmu.ifar = value;
            break;
        case 7u:
            // CRm=8 with CRn=7 selects the VA-to-PA translation operations
            // (V2PCWPR/V2PCWUR/V2POWPR/V2POWUR); Rt holds the virtual address
            // and the answer comes back through PAR (c7, c4, 0).
            if (crm == 8u) {
                cp15_va_to_pa(opc1v, opc2v, value);
                break;
            }
            cp15_cache_op(opc1v, crm, opc2v, value);
            break;
        case 8u:
            cp15_tlb_op(opc1v, crm, opc2v, value);
            break;
        case 10u:
            if (crm == 2u && opc2v == 0u) mmu.prrr = value;
            else if (crm == 2u && opc2v == 1u) mmu.nmrr = value;
            break;
        case 12u:
            if (crm == 0u && opc2v == 0u) {
                // VBAR is banked: the secure world and the non-secure world each
                // keep their own vector base.
                if (secure_state()) {
                    mmu.vbar = value;
                    ZLB_LOG_DBG("cpu", "%s VBAR(secure) = 0x%08X (vectors at 0x%08X) at %08X", name.c_str(),
                                value, mmu.vector_base(), cur_instr_addr_);
                } else {
                    vbar_nonsecure = value;
                    ZLB_LOG_DBG("cpu", "%s VBAR(non-secure) = 0x%08X at %08X", name.c_str(), value,
                                cur_instr_addr_);
                }
                break;
            }
            if (crm == 0u && opc2v == 1u) {
                mvbar = value;
                ZLB_LOG_DBG("cpu", "%s MVBAR = 0x%08X (monitor vectors at 0x%08X) at %08X", name.c_str(), value,
                            mvbar + arm::kVecMonitorSmc, cur_instr_addr_);
                break;
            }
            break;
        case 13u:
            if (crm == 0u && opc2v == 1u) {
                mmu.context_idr = value;
                ZLB_LOG_DBG("cpu", "%s CONTEXTIDR = 0x%08X at %08X", name.c_str(), value, cur_instr_addr_);
            }
            break;
        default:
            break;
    }
}

void ArmCore::cp15_cache_op(u32 opc1v, u32 crm, u32 opc2v, u32 value) {
    // VMSAv7 CP15 c7: cache, branch predictor and barrier operations. The
    // interpreter has no caches, so these are pure visibility points - but the
    // trace names them, because a cache op sequence is the clearest signature of
    // "the loader is about to publish code or page tables".
    const char* cache_name = nullptr;
    if (opc1v == 0u) {
        if (crm == 1u && opc2v == 0u) cache_name = "ICIALLUIS";
        else if (crm == 1u && opc2v == 6u) cache_name = "BPIALLIS";
        else if (crm == 5u && opc2v == 0u) cache_name = "ICIALLU";
        else if (crm == 5u && opc2v == 1u) cache_name = "ICIMVAU";
        else if (crm == 5u && opc2v == 4u) cache_name = "ISB";
        else if (crm == 5u && opc2v == 6u) cache_name = "BPIALL";
        else if (crm == 5u && opc2v == 7u) cache_name = "BPIMVA";
        else if (crm == 6u && opc2v == 1u) cache_name = "DCIMVAC";
        else if (crm == 6u && opc2v == 2u) cache_name = "DCISW";
        else if (crm == 10u && opc2v == 1u) cache_name = "DCCMVAC";
        else if (crm == 10u && opc2v == 2u) cache_name = "DCCSW";
        else if (crm == 10u && opc2v == 4u) cache_name = "DSB";
        else if (crm == 10u && opc2v == 5u) cache_name = "DMB";
        else if (crm == 11u && opc2v == 1u) cache_name = "DCCMVAU";
        else if (crm == 13u && opc2v == 1u) cache_name = "DCCIMVAC?";
        else if (crm == 14u && opc2v == 1u) cache_name = "DCCIMVAC";
        else if (crm == 14u && opc2v == 2u) cache_name = "DCCISW";
    }

    if (cache_name != nullptr) {
        ZLB_LOG_TRACE("cpu", "%s cache/branch maintenance %s(0x%08X)", name.c_str(), cache_name, value);
    } else {
        ZLB_LOG_TRACE("cpu", "%s CP15 c7 op1=%u crm=%u opc2=%u ignored", name.c_str(), opc1v, crm, opc2v);
    }
}

void ArmCore::cp15_tlb_op(u32 opc1v, u32 crm, u32 opc2v, u32 value) {
    // CP15 c8: TLB maintenance. There is no TLB to invalidate (every access
    // walks the tables), but walking is what makes the emulator's view of the
    // tables always current, so the operations only need to be recognised.
    const char* op = nullptr;
    if (opc1v == 0u) {
        if (crm == 3u) {
            switch (opc2v) {
                case 0u: op = "TLBIALL"; break;
                case 1u: op = "TLBIALLIS"; break;
                case 2u: op = "TLBIMVA"; break;
                case 3u: op = "TLBIMVAIS"; break;
                case 4u: op = "TLBIASID"; break;
                case 5u: op = "TLBIASIDIS"; break;
                default: break;
            }
        } else if (crm == 5u) {
            switch (opc2v) {
                case 0u: op = "TLBIMVAL"; break;
                case 1u: op = "TLBIMVALIS"; break;
                case 2u: op = "TLBIMVAAL"; break;
                case 3u: op = "TLBIMVAALIS"; break;
                default: break;
            }
        } else if (crm == 7u) {
            switch (opc2v) {
                case 0u: op = "TLBIMVAA"; break;
                case 1u: op = "TLBIMVAAIS"; break;
                default: break;
            }
        }
    }
    if (op != nullptr) ZLB_LOG_TRACE("cpu", "%s TLB maintenance %s(0x%08X)", name.c_str(), op, value);
    else ZLB_LOG_TRACE("cpu", "%s CP15 c8 op1=%u crm=%u opc2=%u ignored", name.c_str(), opc1v, crm, opc2v);
}

void ArmCore::cp15_va_to_pa(u32 opc1v, u32 opc2v, u32 va) {
    // V2PCWPR/V2PCWUR/V2POWPR/V2POWUR: ask the MMU to translate `va` as a
    // privileged (opc2 bit 0 clear) or user (bit 0 set) access, reading (bit 1
    // clear) or writing (bit 1 set), and publish the answer in PAR. A fault is
    // reported through PAR instead of raising an exception, which is exactly why
    // the boot loader can use these to probe a mapping.
    const bool user = (opc2v & 1u) != 0u;
    const bool write = (opc2v & 2u) != 0u;
    const u32 mode = user ? arm::kModeUser : arm::kModeSupervisor;
    const arm::MmResult result = mmu.translate(va, write, false, mode);
    if (result.ok) {
        mmu.par = (result.phys_addr & 0xFFFFF000u) | 1u;  // PAR[0] = 1: translated
    } else {
        mmu.par = (result.fsr_status << 1) & 0xFFEu;  // PAR[11:1] = fault status
    }
    (void)opc1v;
    ZLB_LOG_TRACE("cpu", "%s V2P %s%s VA=0x%08X -> PAR=0x%08X%s", name.c_str(),
                  write ? "write " : "read ", user ? "user" : "priv", va, mmu.par,
                  result.ok ? "" : " (fault)");
}

// ===========================================================================
// Thumb-16
// ===========================================================================

void ArmCore::execute_thumb16() {
    const u32 i = cur_instr_;
    const u32 top = (i >> 11) & 0x1Fu;
    switch (top) {
        case 0u: case 1u: case 2u: thumb_shift_imm(i); return;
        case 3u: thumb_add_sub(i); return;
        case 4u: case 5u: case 6u: case 7u: thumb_mov_cmp_imm(i); return;
        case 8u:
            if ((i & 0x400u) != 0u) thumb_special_data(i);
            else thumb_alu_ops(i);
            return;
        case 9u: thumb_literal_load(i); return;
        case 10u: case 11u: thumb_load_store_reg(i); return;
        case 12u: case 13u: case 14u: case 15u: thumb_load_store_imm(i); return;
        case 16u: case 17u: thumb_load_store_half_imm(i); return;
        case 18u: case 19u: thumb_sp_relative(i); return;
        case 20u: case 21u: thumb_adr(i); return;
        case 22u: case 23u: thumb_misc16(i); return;
        case 24u: case 25u: thumb_ldm_stm(i); return;
        case 26u: case 27u: thumb_cond_branch(i); return;
        case 28u: thumb_uncond_branch(i); return;
        default: undefined("Thumb-16 encoding"); return;
    }
}

void ArmCore::thumb_shift_imm(u32 i) {
    const u32 op = (i >> 11) & 3u;
    const u32 imm5 = (i >> 6) & 0x1Fu;
    const int rm = static_cast<int>((i >> 3) & 7u);
    const int rd = static_cast<int>(i & 7u);
    if (op == 3u) {
        undefined("Thumb shift op 11");
        return;
    }

    const u32 src = r[rm];
    u32 value;
    bool carry;
    if (op == 0u && imm5 == 0u) {
        value = src;
        carry = flag_c();
    } else {
        const int amount = (op == 0u) ? static_cast<int>(imm5) : (imm5 == 0u ? 32 : static_cast<int>(imm5));
        carry = ((src >> (amount - 1)) & 1u) != 0u;
        value = shift_imm(src, static_cast<int>(op), amount, false);
    }
    r[rd] = value;
    set_nz(value);
    if (carry) cpsr |= arm::kFlagC;
    else cpsr &= ~arm::kFlagC;
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_add_sub(u32 i) {
    const bool immediate = (i & 0x400u) != 0u;
    const bool sub = (i & 0x200u) != 0u;
    // Encoding (ARM ARM A7.2, ADD/SUB register T1): `0001 100 Rm Rn Rd`, i.e. the
    // *second* source is in bits[8:6] and the first in bits[5:3].  This used to read
    // both operands from bits[5:3] (`r[rm] + r[rm]`), which silently computed
    // `Rn + Rn` for every three-operand register ADD/SUB.  kernel_boot_loader builds
    // its memory-map node size/end with `adds r0, r1, r3` (0x40032C7A): with r1 =
    // 0x300000 and r3 = 0x40000000 it produced 0x600000 instead of 0x40300000, so the
    // page-map walk was skipped as "empty region" and the physical memory partition
    // stayed empty (docs/KBL.md, round 52).
    const int rn = static_cast<int>((i >> 3) & 7u);  // first source operand
    const int rm = static_cast<int>((i >> 6) & 7u);  // second source operand (or imm3)
    const int rd = static_cast<int>(i & 7u);
    const u32 op2v = immediate ? static_cast<u32>(rm) : r[rm];
    bool carry;
    bool overflow;
    const u32 result = sub ? sub_with_carry(r[rn], op2v, true, carry, overflow)
                           : add_with_carry(r[rn], op2v, false, carry, overflow);
    r[rd] = result;
    set_nzcv(result, carry, overflow);
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_mov_cmp_imm(u32 i) {
    const u32 op = (i >> 11) & 3u;
    const int rd = static_cast<int>((i >> 8) & 7u);
    const u32 imm8 = i & 0xFFu;
    bool carry;
    bool overflow;
    u32 result;
    switch (op) {
        case 0u:
            r[rd] = imm8;
            set_nz(imm8);
            break;
        case 1u:
            // CMP (immediate): the flags come from Rn - imm8.  The operands were
            // the other way round here, which only agreed with the hardware when
            // Rn == imm8 (that is why the IT tests, all built on `cmp r0, #1` with
            // r0 = 1, kept passing).  kernel_boot_loader compares lengths against
            // constants, e.g. `cmp r3, #1` at 0x4003B6EA, so the carry and sign
            // bits it branches on were inverted for every other value.
            result = sub_with_carry(r[rd], imm8, true, carry, overflow);
            set_nzcv(result, carry, overflow);
            break;
        case 2u:
            result = add_with_carry(r[rd], imm8, false, carry, overflow);
            r[rd] = result;
            set_nzcv(result, carry, overflow);
            break;
        default:
            result = sub_with_carry(r[rd], imm8, true, carry, overflow);
            r[rd] = result;
            set_nzcv(result, carry, overflow);
            break;
    }
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_alu_ops(u32 i) {
    const u32 op = (i >> 6) & 0xFu;
    const int rm = static_cast<int>((i >> 3) & 7u);
    const int rd = static_cast<int>(i & 7u);
    const u32 a = r[rd];
    const u32 b = r[rm];
    bool carry = flag_c();
    bool overflow = flag_v();
    u32 result;
    switch (op) {
        case 0x0u: result = a & b; r[rd] = result; set_nz(result); break;
        case 0x1u: result = a ^ b; r[rd] = result; set_nz(result); break;
        case 0x2u: result = shift_reg(a, 0, b & 0xFFu); r[rd] = result; set_nz(result); break;
        case 0x3u: result = shift_reg(a, 1, b & 0xFFu); r[rd] = result; set_nz(result); break;
        case 0x4u: result = shift_reg(a, 2, b & 0xFFu); r[rd] = result; set_nz(result); break;
        case 0x5u:
            result = add_with_carry(a, b, carry, carry, overflow);
            r[rd] = result;
            set_nzcv(result, carry, overflow);
            break;
        case 0x6u:
            result = sub_with_carry(a, b, carry, carry, overflow);
            r[rd] = result;
            set_nzcv(result, carry, overflow);
            break;
        case 0x7u: result = shift_reg(a, 3, b & 0xFFu); r[rd] = result; set_nz(result); break;
        case 0x8u: result = a & b; set_nz(result); break;
        case 0x9u:
            result = sub_with_carry(0u, b, true, carry, overflow);
            r[rd] = result;
            set_nzcv(result, carry, overflow);
            break;
        case 0xAu:
            result = sub_with_carry(a, b, true, carry, overflow);
            set_nzcv(result, carry, overflow);
            break;
        case 0xBu:
            result = add_with_carry(a, b, false, carry, overflow);
            set_nzcv(result, carry, overflow);
            break;
        case 0xCu: result = a | b; r[rd] = result; set_nz(result); break;
        case 0xDu: result = a * b; r[rd] = result; set_nz(result); break;
        case 0xEu: result = a & ~b; r[rd] = result; set_nz(result); break;
        default: result = ~b; r[rd] = result; set_nz(result); break;
    }
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_special_data(u32 i) {
    const u32 op = (i >> 8) & 3u;
    const int rm = static_cast<int>((i >> 3) & 0xFu);
    const int rd = static_cast<int>((i & 7u) | ((i >> 4) & 8u));
    const u32 a = read_reg(rd);
    const u32 b = read_reg(rm);
    bool carry;
    bool overflow;

    switch (op) {
        case 0u: {
            const u32 value = add_with_carry(a, b, false, carry, overflow);
            if (rd == 15) {
                write_r15(value & ~1u);
                return;
            }
            r[rd] = value;
            break;
        }
        case 1u: {
            const u32 value = sub_with_carry(a, b, true, carry, overflow);
            set_nzcv(value, carry, overflow);
            break;
        }
        case 2u:
            if (rd == 15) branch_to(b);
            else r[rd] = b;
            break;
        default:
            if ((i & 0x0080u) != 0u) r[14] = (cur_instr_addr_ + 2u) | 1u;
            branch_to(read_reg(rm));
            return;
    }
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_literal_load(u32 i) {
    const int rd = static_cast<int>((i >> 8) & 7u);
    const u32 imm8 = static_cast<u32>(i & 0xFFu) * 4u;
    const u32 addr = ((cur_instr_addr_ + 4u) & ~3u) + imm8;
    const u32 value = mem_read_word(addr, false);
    if (pending_fault_ != arm::FaultKind::None) return;
    r[rd] = value;
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_load_store_reg(u32 i) {
    const u32 op = (i >> 9) & 7u;
    const int rm = static_cast<int>((i >> 6) & 7u);
    const int rn = static_cast<int>((i >> 3) & 7u);
    const int rd = static_cast<int>(i & 7u);
    const u32 addr = r[rn] + r[rm];
    switch (op) {
        case 0u: mem_write_word(addr & ~3u, r[rd]); break;
        case 1u: mem_write_half(addr & ~1u, r[rd] & 0xFFFFu); break;
        case 2u: mem_write_byte(addr, r[rd] & 0xFFu); break;
        case 3u: {
            const u32 value = mem_read_byte(addr);
            if (pending_fault_ == arm::FaultKind::None) {
                r[rd] = static_cast<u32>(static_cast<s32>(static_cast<s8>(value & 0xFFu)));
            }
            break;
        }
        case 4u: {
            const u32 value = mem_read_word(addr & ~3u, false);
            if (pending_fault_ == arm::FaultKind::None) r[rd] = value;
            break;
        }
        case 5u: {
            const u32 value = mem_read_half(addr & ~1u);
            if (pending_fault_ == arm::FaultKind::None) r[rd] = value & 0xFFFFu;
            break;
        }
        case 6u: {
            const u32 value = mem_read_byte(addr);
            if (pending_fault_ == arm::FaultKind::None) r[rd] = value & 0xFFu;
            break;
        }
        default: {
            const u32 value = mem_read_half(addr & ~1u);
            if (pending_fault_ == arm::FaultKind::None) {
                r[rd] = static_cast<u32>(static_cast<s32>(static_cast<s16>(value & 0xFFFFu)));
            }
            break;
        }
    }
    if (pending_fault_ != arm::FaultKind::None) return;
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_load_store_imm(u32 i) {
    const bool byte_op = (i & 0x1000u) != 0u;
    const bool load = (i & 0x0800u) != 0u;
    const u32 imm5 = (i >> 6) & 0x1Fu;
    const int rn = static_cast<int>((i >> 3) & 7u);
    const int rd = static_cast<int>(i & 7u);
    const u32 offset = byte_op ? imm5 : imm5 * 4u;
    const u32 addr = r[rn] + offset;
    if (load) {
        const u32 value = byte_op ? mem_read_byte(addr) : mem_read_word(addr & ~3u, false);
        if (pending_fault_ != arm::FaultKind::None) return;
        r[rd] = byte_op ? (value & 0xFFu) : value;
    } else {
        if (byte_op) mem_write_byte(addr, r[rd] & 0xFFu);
        else mem_write_word(addr & ~3u, r[rd]);
        if (pending_fault_ != arm::FaultKind::None) return;
    }
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_load_store_half_imm(u32 i) {
    const bool load = (i & 0x0800u) != 0u;
    const u32 imm5 = (i >> 6) & 0x1Fu;
    const int rn = static_cast<int>((i >> 3) & 7u);
    const int rd = static_cast<int>(i & 7u);
    const u32 addr = r[rn] + imm5 * 2u;
    if (load) {
        const u32 value = mem_read_half(addr & ~1u);
        if (pending_fault_ == arm::FaultKind::None) r[rd] = value & 0xFFFFu;
    } else {
        mem_write_half(addr & ~1u, r[rd] & 0xFFFFu);
    }
    if (pending_fault_ != arm::FaultKind::None) return;
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_sp_relative(u32 i) {
    const bool load = (i & 0x0800u) != 0u;
    const int rd = static_cast<int>((i >> 8) & 7u);
    const u32 imm8 = static_cast<u32>(i & 0xFFu) * 4u;
    const u32 addr = r[13] + imm8;
    if (load) {
        const u32 value = mem_read_word(addr & ~3u, false);
        if (pending_fault_ == arm::FaultKind::None) r[rd] = value;
    } else {
        mem_write_word(addr & ~3u, r[rd]);
    }
    if (pending_fault_ != arm::FaultKind::None) return;
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_adr(u32 i) {
    const bool sp = (i & 0x0800u) != 0u;
    const int rd = static_cast<int>((i >> 8) & 7u);
    const u32 imm8 = static_cast<u32>(i & 0xFFu) * 4u;
    r[rd] = (sp ? r[13] : ((cur_instr_addr_ + 4u) & ~3u)) + imm8;
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_ldm_stm(u32 i) {
    const bool load = (i & 0x0800u) != 0u;
    const int rn = static_cast<int>((i >> 8) & 7u);
    const u32 list = i & 0xFFu;
    if (list == 0u) {
        write_r15(cur_instr_addr_ + 2u);
        return;
    }
    const int count = arm::popcount32(list);
    const u32 base_addr = r[rn];
    u32 addr = base_addr;
    bool writeback = true;
    if (load) {
        for (int reg = 0; reg < 8; ++reg) {
            if ((list & (1u << reg)) == 0) continue;
            const u32 value = mem_read_word(addr, false);
            if (pending_fault_ != arm::FaultKind::None) return;
            r[reg] = value;
            addr += 4u;
        }
        if ((list & (1u << rn)) != 0) writeback = false;
    } else {
        for (int reg = 0; reg < 8; ++reg) {
            if ((list & (1u << reg)) == 0) continue;
            mem_write_word(addr, r[reg]);
            if (pending_fault_ != arm::FaultKind::None) return;
            addr += 4u;
        }
        if ((list & (1u << rn)) != 0) writeback = false;
    }
    if (writeback) r[rn] = base_addr + static_cast<u32>(count * 4);
    write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_cond_branch(u32 i) {
    const u32 cond = (i >> 8) & 0xFu;
    if (cond == 0xFu) {
        take_exception(arm::kVecSupervisor, arm::kModeSupervisor, cur_instr_addr_ + 2u);
        return;
    }
    if (cond == 0xEu) {
        undefined("Thumb UDF");
        return;
    }
    s32 imm = static_cast<s32>(i & 0xFFu);
    if ((imm & 0x80) != 0) imm |= static_cast<s32>(0xFFFFFF00u);
    if (arm::condition_passed(cond, cpsr)) write_r15(cur_instr_addr_ + 4u + static_cast<u32>(imm << 1));
    else write_r15(cur_instr_addr_ + 2u);
}

void ArmCore::thumb_uncond_branch(u32 i) {
    s32 imm = static_cast<s32>(i & 0x7FFu);
    if ((imm & 0x400) != 0) imm |= static_cast<s32>(0xFFFFF800u);
    write_r15(cur_instr_addr_ + 4u + static_cast<u32>(imm << 1));
}

void ArmCore::thumb_misc16(u32 i) {
    const u32 sub = (i >> 8) & 0xFu;

    if (sub == 0u) {
        const bool subtract = (i & 0x80u) != 0u;
        const u32 imm = static_cast<u32>(i & 0x7Fu) * 4u;
        r[13] = subtract ? r[13] - imm : r[13] + imm;
        write_r15(cur_instr_addr_ + 2u);
        return;
    }
    if ((i & 0xF500u) == 0xB100u) {
        // CBZ / CBNZ.
        const bool nz = (i & 0x800u) != 0u;
        const u32 high = (i >> 9) & 1u;
        const u32 imm5 = (i >> 3) & 0x1Fu;
        const u32 offset = ((high << 5) | imm5) << 1;
        const bool zero = r[i & 7u] == 0u;
        if (nz ? !zero : zero) write_r15(cur_instr_addr_ + 4u + offset);
        else write_r15(cur_instr_addr_ + 2u);
        return;
    }
    if ((i & 0xFF00u) == 0xB200u) {
        const u32 op2v = (i >> 6) & 3u;
        const int rm = static_cast<int>((i >> 3) & 7u);
        const int rd = static_cast<int>(i & 7u);
        switch (op2v) {
            case 0u: r[rd] = static_cast<u32>(static_cast<s32>(static_cast<s16>(r[rm] & 0xFFFFu))); break;
            case 1u: r[rd] = static_cast<u32>(static_cast<s32>(static_cast<s8>(r[rm] & 0xFFu))); break;
            case 2u: r[rd] = r[rm] & 0xFFFFu; break;
            default: r[rd] = r[rm] & 0xFFu; break;
        }
        write_r15(cur_instr_addr_ + 2u);
        return;
    }
    if ((i & 0xFE00u) == 0xB400u) {
        u32 list = i & 0xFFu;
        if ((i & 0x100u) != 0u) list |= 0x4000u;
        const u32 sp = r[13] - static_cast<u32>(arm::popcount32(list) * 4);
        u32 addr = sp;
        for (int reg = 0; reg < 16; ++reg) {
            if ((list & (1u << reg)) == 0) continue;
            mem_write_word(addr, r[reg]);
            if (pending_fault_ != arm::FaultKind::None) return;
            addr += 4u;
        }
        r[13] = sp;
        write_r15(cur_instr_addr_ + 2u);
        return;
    }
    if ((i & 0xFF00u) == 0xB600u) {
        if ((i & 0x00F0u) == 0x0060u) {
            if (mode() != arm::kModeUser) cpsr |= arm::kFlagI;
        } else if ((i & 0x00F0u) == 0x0070u) {
            if (mode() != arm::kModeUser) cpsr &= ~arm::kFlagI;
        } else if ((i & 0x00FFu) == 0x0002u) {
            cpsr |= arm::kFlagE;
        } else if ((i & 0x00FFu) == 0x0000u) {
            cpsr &= ~arm::kFlagE;
        }
        write_r15(cur_instr_addr_ + 2u);
        return;
    }
    if ((i & 0xFF00u) == 0xBA00u) {
        const u32 a = (i >> 6) & 3u;
        const int rm = static_cast<int>((i >> 3) & 7u);
        const int rd = static_cast<int>(i & 7u);
        const u32 value = r[rm];
        switch (a) {
            case 0u:
                r[rd] = arm::reverse_bytes(value);
                break;
            case 1u: {
                const u32 reversed = arm::reverse_bytes(value);
                r[rd] = (reversed >> 16) | (reversed << 16);
                break;
            }
            default:
                r[rd] = static_cast<u32>(
                    static_cast<s32>(static_cast<s16>(((value & 0xFFu) << 8) | ((value >> 8) & 0xFFu))));
                break;
        }
        write_r15(cur_instr_addr_ + 2u);
        return;
    }
    if ((i & 0xFE00u) == 0xBC00u) {
        u32 list = i & 0xFFu;
        if ((i & 0x100u) != 0u) list |= 0x8000u;
        const u32 sp = r[13];
        u32 addr = sp;
        bool pc_loaded = false;
        for (int reg = 0; reg < 16; ++reg) {
            if ((list & (1u << reg)) == 0) continue;
            const u32 value = mem_read_word(addr, false);
            if (pending_fault_ != arm::FaultKind::None) return;
            if (reg == 15) {
                r[15] = value;
                pc_loaded = true;
            } else {
                r[reg] = value;
            }
            addr += 4u;
        }
        r[13] = sp + static_cast<u32>(arm::popcount32(list) * 4);
        if (pc_loaded) {
            branch_to(r[15]);
            return;
        }
        write_r15(cur_instr_addr_ + 2u);
        return;
    }
    if ((i & 0xFF00u) == 0xBE00u) {
        ZLB_LOG_DBG("cpu", "%s BKPT #%u", name.c_str(), i & 0xFFu);
        take_exception(arm::kVecPrefetchAbort, arm::kModeAbort, cur_instr_addr_ + 2u);
        return;
    }
    if ((i & 0xFF00u) == 0xBF00u) {
        const u32 mask = i & 0xFu;
        const u32 firstcond = (i >> 4) & 0xFu;
        if (mask == 0u) {
            // NOP / YIELD / WFE / WFI / SEV / DBG.
            write_r15(cur_instr_addr_ + 2u);
            return;
        }
        if (firstcond == 0xFu || (firstcond == 0xEu && (mask & 0x7u) == 0u)) {
            undefined("IT with an invalid condition");
            return;
        }
        // ITSTATE = firstcond : mask (ARM ARM A7.7.37), i.e. an 8 bit value whose
        // top nibble is the block's condition and whose low nibble is the mask.
        // Each instruction takes its condition from ITSTATE<7:4> and then advances
        // ITSTATE<4:0> = ITSTATE<3:0>:'0' - the shift *across* the nibble boundary
        // is what turns a 0 mask bit into the inverted condition of an 'E'
        // position.  Storing firstcond[3:1] with bit 4 forced to 0 (as this used
        // to) dropped firstcond[0], so every block with an odd condition ran
        // inverted: kernel_boot_loader's `it ls` (mask 1101, 0x4003B6EC) executed
        // as `it hi` and loaded a function pointer through a garbage base.
        it_state_ = ((firstcond & 0xFu) << 4) | (mask & 0xFu);
        it_state_valid_ = true;
        write_r15(cur_instr_addr_ + 2u);
        return;
    }
    undefined("Thumb-16 misc");
}

// ---- IT state --------------------------------------------------------------

u32 ArmCore::thumb_condition(bool& in_it_block, bool& last_in_it) {
    if (!it_state_valid_) {
        in_it_block = false;
        last_in_it = false;
        return 0xEu;
    }
    in_it_block = true;
    const u32 state = it_state_ & 0xFFu;
    // ITSTATE[7:4] is the condition of the instruction about to execute.  Bit 4 is
    // fed by the shift below, so an 'E' position inverts the block's first
    // condition - without this every instruction of an ITE block ran on the same
    // condition (KBL 0x40020AAC: `ite hs / ldrhs r0,[r0,#0x40] / ldrlo r0,[r0,#0x60]`).
    const u32 cond = (state >> 4) & 0xFu;
    if ((state & 0x7u) == 0u) {
        last_in_it = true;
        it_state_valid_ = false;
        it_state_ = 0;
    } else {
        last_in_it = false;
        it_state_ = (state & 0xE0u) | ((state << 1) & 0x1Fu);
    }
    return cond;
}

u32 ArmCore::thumb_condition_peek(bool& in_it_block, bool& last_in_it) const {
    if (!it_state_valid_) {
        in_it_block = false;
        last_in_it = false;
        return 0xEu;
    }
    const u32 state = it_state_ & 0xFFu;
    in_it_block = true;
    last_in_it = (state & 0x7u) == 0u;
    return (state >> 4) & 0xFu;
}

// ===========================================================================
// Thumb-32
// ===========================================================================

void ArmCore::execute_thumb32() {
    bool in_it = false;
    bool last_in_it = false;
    const u32 cond = thumb_condition(in_it, last_in_it);
    (void)last_in_it;

    if (in_it) {
        if (cond == 0xFu) {
            undefined("T32 instruction with cond AL inside an IT block");
            return;
        }
        if (!arm::condition_passed(cond, cpsr)) {
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
    }

    const u32 hw1 = cur_instr_ >> 16;
    const u32 t1 = (hw1 >> 11) & 3u;
    const u32 top = (hw1 >> 4) & 0x7Fu;

    if (t1 == 1u) {
        if ((top & 0x60u) == 0x00u) {
            thumb32_load_store_dual_excl_table();
            return;
        }
        if ((top & 0x60u) == 0x20u) {
            thumb32_data_processing_shifted();
            return;
        }
        thumb32_coprocessor();
        return;
    }

    if (t1 == 2u) {
        const u32 hw2 = cur_instr_ & 0xFFFFu;
        if ((hw2 & 0x8000u) != 0u) {
            thumb32_branch_misc();
            return;
        }
        if ((hw1 & 0x200u) == 0u) {
            thumb32_data_processing_modified();
            return;
        }
        thumb32_data_processing_plain();
        return;
    }

    if ((top & 0x60u) == 0x20u) {
        thumb32_data_processing_register();
        return;
    }
    thumb32_load_store_single();
}

void ArmCore::thumb32_data_processing_shifted() {
    const u32 hw1 = cur_instr_ >> 16;
    const u32 hw2 = cur_instr_ & 0xFFFFu;
    const u32 opc = (hw1 >> 5) & 0xFu;
    const bool sf = (hw1 & 0x10u) != 0u;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rd = static_cast<int>((hw2 >> 8) & 0xFu);
    const int rm = static_cast<int>(hw2 & 0xFu);
    const int type = static_cast<int>((hw2 >> 4) & 3u);
    const int amount = static_cast<int>(((hw2 >> 12) & 7u) << 2 | ((hw2 >> 6) & 3u));

    const u32 shifted = shift_imm(r[rm], type, amount, false);

    u32 result;
    bool carry = flag_c();
    bool overflow = flag_v();
    switch (opc) {
        case 0x0u: result = r[rn] & shifted; break;
        case 0x1u: result = r[rn] & ~shifted; break;
        case 0x2u: result = rn == 15 ? shifted : (r[rn] | shifted); break;
        case 0x3u: result = rn == 15 ? ~shifted : (r[rn] | ~shifted); break;
        case 0x4u: result = rn == 15 ? shifted : (r[rn] ^ shifted); break;
        case 0x6u: {
            // PKH (T32): PKHBT / PKHTB.
            const u32 amount2 = ((hw2 >> 12) & 7u) << 2 | ((hw2 >> 6) & 3u);
            const bool tb = (hw2 & 0x20u) != 0u;
            if (!tb) {
                const u32 sh = shift_imm(r[rn], 0, static_cast<int>(amount2), false);
                result = (sh & 0xFFFF0000u) | (r[rm] & 0xFFFFu);
            } else {
                const int amt = static_cast<int>(amount2 == 0 ? 32u : amount2);
                const u32 sh = shift_imm(r[rm], 2, amt, false);
                result = (r[rn] & 0xFFFF0000u) | (sh & 0xFFFFu);
            }
            break;
        }
        case 0x8u: result = add_with_carry(r[rn], shifted, false, carry, overflow); break;
        case 0xAu: result = add_with_carry(r[rn], shifted, flag_c(), carry, overflow); break;
        case 0xBu: result = sub_with_carry(r[rn], shifted, flag_c(), carry, overflow); break;
        case 0xDu: result = sub_with_carry(r[rn], shifted, true, carry, overflow); break;
        case 0xEu: result = sub_with_carry(shifted, r[rn], true, carry, overflow); break;
        default:
            undefined("T32 shifted-register opc");
            return;
    }

    r[rd] = result;
    if (sf) {
        if (opc <= 7u) set_nz(result);
        else set_nzcv(result, carry, overflow);
    }
    write_r15(cur_instr_addr_ + 4u);
}

void ArmCore::thumb32_data_processing_modified() {
    const u32 hw1 = cur_instr_ >> 16;
    const u32 hw2 = cur_instr_ & 0xFFFFu;
    const u32 i = (hw1 >> 10) & 1u;
    const u32 opc = (hw1 >> 5) & 0xFu;
    const bool sf = (hw1 & 0x10u) != 0u;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rd = static_cast<int>((hw2 >> 8) & 0xFu);
    const u32 imm12v = (i << 11) | (((hw2 >> 12) & 7u) << 8) | (hw2 & 0xFFu);
    const u32 imm = arm::thumb_expand_imm(imm12v);

    // The "Rd == 1111" forms of AND/EOR/ADD/SUB are the TST/TEQ/CMN/CMP aliases.
    if (rd == 15 && (opc == 0x0u || opc == 0x4u || opc == 0x8u || opc == 0xDu)) {
        const u32 a = r[rn];
        u32 result;
        bool carry = flag_c();
        bool overflow = flag_v();
        if (opc == 0x0u) result = a & imm;
        else if (opc == 0x4u) result = a ^ imm;
        else if (opc == 0x8u) result = add_with_carry(a, imm, false, carry, overflow);
        else result = sub_with_carry(a, imm, true, carry, overflow);
        if (sf) {
            if (opc == 0x0u || opc == 0x4u) set_nz(result);
            else set_nzcv(result, carry, overflow);
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    u32 result;
    bool carry = flag_c();
    bool overflow = flag_v();
    bool logical = false;
    switch (opc) {
        case 0x0u: result = r[rn] & imm; logical = true; break;
        case 0x1u: result = r[rn] & ~imm; logical = true; break;
        case 0x2u: result = rn == 15 ? imm : (r[rn] | imm); logical = true; break;
        case 0x3u: result = rn == 15 ? ~imm : (r[rn] | ~imm); logical = true; break;
        case 0x4u: result = rn == 15 ? imm : (r[rn] ^ imm); logical = true; break;
        case 0x8u: result = add_with_carry(r[rn], imm, false, carry, overflow); break;
        case 0xAu: result = add_with_carry(r[rn], imm, flag_c(), carry, overflow); break;
        case 0xBu: result = sub_with_carry(r[rn], imm, flag_c(), carry, overflow); break;
        case 0xDu: result = sub_with_carry(r[rn], imm, true, carry, overflow); break;
        case 0xEu: result = sub_with_carry(imm, r[rn], true, carry, overflow); break;
        default:
            undefined("T32 modified-immediate opc");
            return;
    }
    r[rd] = result;
    if (sf) {
        if (logical) set_nz(result);
        else set_nzcv(result, carry, overflow);
    }
    write_r15(cur_instr_addr_ + 4u);
}

void ArmCore::thumb32_data_processing_plain() {
    const u32 hw1 = cur_instr_ >> 16;
    const u32 hw2 = cur_instr_ & 0xFFFFu;
    const u32 i = (hw1 >> 10) & 1u;
    const u32 opc = (hw1 >> 4) & 0x1Fu;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rd = static_cast<int>((hw2 >> 8) & 0xFu);
    const u32 imm3v = (hw2 >> 12) & 7u;
    const u32 imm8 = hw2 & 0xFFu;
    const u32 imm12v = (i << 11) | (imm3v << 8) | imm8;
    const int lsb = static_cast<int>(imm3v << 2 | ((hw2 >> 6) & 3u));
    const int shift_type = static_cast<int>((hw2 >> 4) & 3u);

    switch (opc) {
        case 0x00u:  // ADDW
            r[rd] = r[rn] + imm12v;
            break;
        case 0x04u:  // MOVW
            r[rd] = (hw1 & 0xFu) << 12 | (i << 11) | (imm3v << 8) | imm8;
            break;
        case 0x0Au:  // SUBW
            r[rd] = r[rn] - imm12v;
            break;
        case 0x0Cu:  // MOVT - writes the top half only
            r[rd] = (r[rd] & 0x0000FFFFu) |
                   ((((hw1 & 0xFu) << 12) | (i << 11) | (imm3v << 8) | imm8) << 16);
            break;
        case 0x10u: {  // SSAT
            const u32 sat = hw1 & 0xFu;
            const u32 value = shift_imm(r[rn], shift_type, lsb, false);
            r[rd] = static_cast<u32>(arm::saturate_signed(static_cast<s32>(value),
                                                          static_cast<unsigned>(sat) + 1u));
            break;
        }
        case 0x12u: {  // SSAT16
            const int sat = static_cast<int>(hw1 & 0xFu);
            const s32 v0 = static_cast<s16>(r[rn] & 0xFFFFu);
            const s32 v1 = static_cast<s16>(r[rn] >> 16);
            r[rd] = static_cast<u32>(
                ((arm::saturate_signed(v1, static_cast<unsigned>(sat) + 1u) & 0xFFFF) << 16) |
                (arm::saturate_signed(v0, static_cast<unsigned>(sat) + 1u) & 0xFFFF));
            break;
        }
        case 0x14u:  // SBFX
        case 0x1Cu: {  // UBFX
            const int width = static_cast<int>(hw2 >> 12);
            const int l2 = static_cast<int>(((hw2 >> 6) & 3u) | ((hw2 >> 10) & 0x1Cu));
            const int w = width + 1;
            if (w <= 0 || w > 32 || l2 + w > 32) {
                undefined("T32 bitfield width");
                return;
            }
            const u32 shifted = r[rn] >> l2;
            u32 value = w >= 32 ? shifted : (shifted & ((1u << w) - 1u));
            if (opc == 0x14u && w < 32 && (value & (1u << (w - 1))) != 0) {
                value |= ~((1u << w) - 1u);
            }
            r[rd] = value;
            break;
        }
        case 0x16u: {  // BFI / BFC
            const int width = static_cast<int>(hw2 >> 12);
            const int l2 = static_cast<int>(((hw2 >> 6) & 3u) | ((hw2 >> 10) & 0x1Cu));
            const int w = width + 1;
            if (w <= 0 || w > 32 || l2 + w > 32) {
                undefined("T32 BFI width");
                return;
            }
            const u32 mask = w >= 32 ? 0xFFFFFFFFu : ((1u << w) - 1u);
            const u32 src = rn == 15 ? 0u : r[rn];
            r[rd] = (r[rd] & ~(mask << l2)) | ((src & mask) << l2);
            break;
        }
        case 0x18u: {  // USAT
            const u32 sat = hw1 & 0xFu;
            const u32 value = shift_imm(r[rn], shift_type, lsb, false);
            const s64 lv = static_cast<s32>(value);
            const s32 max = sat >= 32u ? 0x7FFFFFFF : (1 << static_cast<s32>(sat)) - 1;
            u32 res;
            if (lv > max) res = static_cast<u32>(max);
            else if (lv < 0) res = 0u;
            else res = static_cast<u32>(lv);
            r[rd] = res;
            break;
        }
        case 0x1Au: {  // USAT16
            const int sat = static_cast<int>(hw1 & 0xFu);
            const int max = sat >= 16 ? 0xFFFF : (1 << sat) - 1;
            const int v0 = static_cast<int>(r[rn] & 0xFFFFu);
            const int v1 = static_cast<int>(r[rn] >> 16);
            r[rd] = static_cast<u32>((((v1 > max ? max : v1) & 0xFFFF) << 16) |
                                     ((v0 > max ? max : v0) & 0xFFFF));
            break;
        }
        default:
            undefined("T32 plain-immediate opc");
            return;
    }
    write_r15(cur_instr_addr_ + 4u);
}

void ArmCore::thumb32_data_processing_register() {
    const u32 hw1 = cur_instr_ >> 16;
    const u32 hw2 = cur_instr_ & 0xFFFFu;
    const u32 op1 = (hw1 >> 4) & 0xFu;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rd = static_cast<int>((hw2 >> 8) & 0xFu);
    const int ra = static_cast<int>((hw2 >> 12) & 0xFu);
    const int rm = static_cast<int>(hw2 & 0xFu);
    const u32 op2 = (hw2 >> 4) & 0xFu;

    // LSL/LSR/ASR/ROR (register) T2: 1111 1010 op1(4) Rn | 1111 Rd 0000 00 Rm.
    // op1 = 0/1 LSL, 2/3 LSR, 4/5 ASR, 6/7 ROR (odd = set flags).  These share the
    // 1111 1010 prefix with the parallel add/sub group, but only op1 <= 7 are
    // shifts, and an AND.W Rd, Rn, Rm (1110 1010 0000 Rn ...) must NOT land here.
    if (ra == 15 && op2 == 0u && op1 <= 7u && ((hw1 >> 8) & 0xFu) == 0xAu) {
        const u32 shifted = shift_reg(r[rn], static_cast<int>(op1 >> 1), r[rm] & 0xFFu);
        r[rd] = shifted;
        if ((op1 & 1u) != 0u) set_nz(shifted);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    if (((hw1 >> 8) & 0xFu) == 0xAu) {
        parallel_add_sub32(op1, op2, rd, rn, rm);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    switch (op1) {
        case 0x0u:
            if (op2 == 0u) {
                if (ra == 15) r[rd] = r[rn] * r[rm];             // MUL
                else r[rd] = r[rn] * r[rm] + r[ra];              // MLA
                write_r15(cur_instr_addr_ + 4u);
                return;
            }
            break;
        case 0x1u:
            if (op2 == 0u) {
                r[rd] = r[ra] - r[rn] * r[rm];                   // MLS
                write_r15(cur_instr_addr_ + 4u);
                return;
            }
            break;
        case 0x8u:
            if (op2 == 0u) {
                // SMULL
                r[rd] = static_cast<u32>((static_cast<s64>(static_cast<s32>(r[rn])) *
                                          static_cast<s32>(r[rm])) >> 32);
                write_r15(cur_instr_addr_ + 4u);
                return;
            }
            if (op2 == 1u) {
                // UMULL
                r[rd] = static_cast<u32>((static_cast<u64>(r[rn]) * r[rm]) >> 32);
                write_r15(cur_instr_addr_ + 4u);
                return;
            }
            break;
        default:
            break;
    }
    undefined("T32 data-processing register");
}

void ArmCore::parallel_add_sub32(u32 op1, u32 op2, int rd, int rn, int rm) {
    const bool is_unsigned = (op1 & 4u) != 0u;
    const bool saturating = (op1 & 3u) == 2u;
    const bool halving = (op1 & 3u) == 3u;

    const u32 a = r[rn];
    const u32 b = r[rm];
    u32 result;
    const bool is8 = (op2 & 1u) != 0u;
    const bool is_asx = op2 == 2u || op2 == 6u;
    const bool is_sax = op2 == 3u || op2 == 7u;
    const bool subtract = op2 >= 4u;

    if (is8) {
        result = 0;
        for (int k = 0; k < 4; ++k) {
            s32 x = static_cast<s32>((a >> (k * 8)) & 0xFFu);
            s32 y = static_cast<s32>((b >> (k * 8)) & 0xFFu);
            if (!is_unsigned) {
                x = static_cast<s8>(x);
                y = static_cast<s8>(y);
            }
            s32 res = subtract ? x - y : x + y;
            if (saturating) res = arm::saturate_signed(res, 8);
            else if (halving) res >>= 1;
            result |= static_cast<u32>(res & 0xFF) << (k * 8);
        }
    } else if (is_asx || is_sax) {
        s32 a0 = static_cast<s32>(a & 0xFFFFu);
        s32 a1 = static_cast<s32>(a >> 16);
        s32 b0 = static_cast<s32>(b & 0xFFFFu);
        s32 b1 = static_cast<s32>(b >> 16);
        if (!is_unsigned) {
            a0 = static_cast<s16>(a0);
            a1 = static_cast<s16>(a1);
            b0 = static_cast<s16>(b0);
            b1 = static_cast<s16>(b1);
        }
        s32 r0;
        s32 r1;
        if (is_asx) {
            r0 = a0 + (subtract ? -b1 : b1);
            r1 = a1 + (subtract ? -b0 : b0);
        } else {
            r0 = a0 + (subtract ? -b0 : b0);
            r1 = a1 + (subtract ? -b1 : b1);
        }
        if (saturating) {
            r0 = arm::saturate_signed(r0, 16);
            r1 = arm::saturate_signed(r1, 16);
        } else if (halving) {
            r0 >>= 1;
            r1 >>= 1;
        }
        result = static_cast<u32>(((r1 & 0xFFFF) << 16) | (r0 & 0xFFFF));
    } else {
        s32 a0 = static_cast<s32>(a & 0xFFFFu);
        s32 a1 = static_cast<s32>(a >> 16);
        s32 b0 = static_cast<s32>(b & 0xFFFFu);
        s32 b1 = static_cast<s32>(b >> 16);
        if (!is_unsigned) {
            a0 = static_cast<s16>(a0);
            a1 = static_cast<s16>(a1);
            b0 = static_cast<s16>(b0);
            b1 = static_cast<s16>(b1);
        }
        s32 r0;
        s32 r1;
        if (subtract) {
            r0 = a0 - b0;
            r1 = a1 - b1;
        } else {
            r0 = a0 + b0;
            r1 = a1 + b1;
        }
        if (saturating) {
            r0 = arm::saturate_signed(r0, 16);
            r1 = arm::saturate_signed(r1, 16);
        } else if (halving) {
            r0 >>= 1;
            r1 >>= 1;
        }
        result = static_cast<u32>(((r1 & 0xFFFF) << 16) | (r0 & 0xFFFF));
    }
    r[rd] = result;
}

void ArmCore::thumb32_load_store_dual_excl_table() {
    const u32 hw1 = cur_instr_ >> 16;
    const u32 hw2 = cur_instr_ & 0xFFFFu;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rt = static_cast<int>((hw2 >> 12) & 0xFu);
    const bool db = (hw1 & 0x100u) != 0u;
    const bool u = (hw1 & 0x080u) != 0u;
    const bool fixed6 = (hw1 & 0x040u) != 0u;
    const bool w = (hw1 & 0x020u) != 0u;
    const bool l = (hw1 & 0x010u) != 0u;

    if (!fixed6) {
        // LDM / STM (IA when hw1[8] == 0, DB when hw1[8] == 1).
        const u32 list = hw2;
        if (list == 0u) {
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        const int count = arm::popcount32(list);
        const bool pc_in_list = (list & 0x8000u) != 0u;
        if (!l && pc_in_list) {
            undefined("T32 STM with PC in the list");
            return;
        }

        if (l) {
            const u32 base_for_load = read_reg(rn);
            u32 addr = db ? base_for_load - static_cast<u32>(count * 4) : base_for_load;
            for (int reg = 0; reg < 16; ++reg) {
                if ((list & (1u << reg)) == 0) continue;
                const u32 value = mem_read_word(addr, false);
                if (pending_fault_ != arm::FaultKind::None) return;
                if (reg == 15) r[15] = value;
                else r[reg] = value;
                addr += 4u;
            }
            if (w && rn != 15) {
                r[rn] = db ? read_reg(rn) - static_cast<u32>(count * 4)
                           : read_reg(rn) + static_cast<u32>(count * 4);
            }
            if (pc_in_list && pending_fault_ == arm::FaultKind::None) {
                branch_to(r[15]);
                return;
            }
        } else {
            const u32 base_addr = read_reg(rn);
            u32 addr = db ? base_addr - static_cast<u32>(count * 4) : base_addr;
            const u32 pc_value = arm::read_pc_value(true, cur_instr_addr_);
            for (int reg = 0; reg < 16; ++reg) {
                if ((list & (1u << reg)) == 0) continue;
                mem_write_word(addr, reg == 15 ? pc_value : r[reg]);
                if (pending_fault_ != arm::FaultKind::None) return;
                addr += 4u;
            }
            if (w && rn != 15) {
                r[rn] = db ? base_addr - static_cast<u32>(count * 4)
                           : base_addr + static_cast<u32>(count * 4);
            }
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    // Exclusive access: STREX/LDREX.
    if (!u && !db && !w) {
        const u32 imm = (hw2 & 0xFFu) * 4u;
        if (!l) {
            const int rd = static_cast<int>((hw2 >> 8) & 0xFu);
            u32 status = 1u;
            if (bus->take_exclusive(read_reg(rn), 4u, static_cast<int>(core_id_), mmu.context_idr)) {
                mem_write_word(read_reg(rn), r[rt]);
                if (pending_fault_ != arm::FaultKind::None) return;
                status = 0u;
            }
            exclusive_valid_ = false;
            r[rd] = status;
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        const u32 address = read_reg(rn) + imm;
        const u32 value = mem_read_word(address, false);
        if (pending_fault_ != arm::FaultKind::None) return;
        bus->mark_exclusive(address, 4u, static_cast<int>(core_id_), mmu.context_idr);
        exclusive_valid_ = true;
        exclusive_addr_ = address;
        exclusive_id_ = mmu.context_idr;
        r[rt] = value;
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    // LDREXH / STREXH (T1): hw1 = 0xE840 | Rn (store) / 0xE850 | Rn (load).
    if (!u && !db && !w && ((hw1 & 0xFFF0u) == 0xE840u || (hw1 & 0xFFF0u) == 0xE850u)) {
        const bool load = (hw1 & 0x10u) != 0u;
        const u32 address = read_reg(rn) & ~1u;
        if (!load) {
            const int rd = static_cast<int>(hw2 & 0xFu);
            u32 status = 1u;
            if (bus->take_exclusive(address, 2u, static_cast<int>(core_id_), mmu.context_idr)) {
                mem_write_half(address, r[rt] & 0xFFFFu);
                if (pending_fault_ != arm::FaultKind::None) return;
                status = 0u;
            }
            exclusive_valid_ = false;
            r[rd] = status;
        } else {
            const u32 value = mem_read_half(address);
            if (pending_fault_ != arm::FaultKind::None) return;
            bus->mark_exclusive(address, 2u, static_cast<int>(core_id_), mmu.context_idr);
            exclusive_valid_ = true;
            exclusive_addr_ = address;
            exclusive_id_ = mmu.context_idr;
            r[rt] = value & 0xFFFFu;
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    if (db && !u && !((hw1 & 0xFFF0u) == 0xE8C0u || (hw1 & 0xFFF0u) == 0xE8D0u)) {
        undefined("T32 exclusive byte");
        return;
    }
    if ((hw1 & 0xFFF0u) == 0xE8C0u || (hw1 & 0xFFF0u) == 0xE8D0u) {
        // STREXB / LDREXB.
        const bool load = (hw1 & 0x10u) != 0u;
        const u32 address = read_reg(rn);
        if (!load) {
            const int rd = static_cast<int>(hw2 & 0xFu);
            u32 status = 1u;
            if (bus->take_exclusive(address, 1u, static_cast<int>(core_id_), mmu.context_idr)) {
                mem_write_byte(address, r[rt] & 0xFFu);
                if (pending_fault_ != arm::FaultKind::None) return;
                status = 0u;
            }
            exclusive_valid_ = false;
            r[rd] = status;
        } else {
            const u32 value = mem_read_byte(address);
            if (pending_fault_ != arm::FaultKind::None) return;
            bus->mark_exclusive(address, 1u, static_cast<int>(core_id_), mmu.context_idr);
            exclusive_valid_ = true;
            exclusive_addr_ = address;
            exclusive_id_ = mmu.context_idr;
            r[rt] = value & 0xFFu;
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    // Table branch (TBB/TBH) is not decoded by the reference core.
    if (u && !db && w && hw2 == 0x0000u) {
        undefined("T32 table branch");
        return;
    }

    // LDRD / STRD.
    const int rt2 = static_cast<int>((hw2 >> 8) & 0xFu);
    const u32 imm8 = (hw2 & 0xFFu) * 4u;
    const u32 base_addr = read_reg(rn);
    u32 address = base_addr;
    if (db) address = u ? base_addr + imm8 : base_addr - imm8;
    if (l) {
        const u64 value = mem_read_double(address);
        if (pending_fault_ != arm::FaultKind::None) return;
        r[rt] = static_cast<u32>(value);
        r[rt2] = static_cast<u32>(value >> 32);
    } else {
        const u64 value = static_cast<u64>(r[rt]) | (static_cast<u64>(r[rt2]) << 32);
        mem_write_double(address, value);
        if (pending_fault_ != arm::FaultKind::None) return;
    }
    if (w && rn != 15) r[rn] = u ? base_addr + imm8 : base_addr - imm8;
    write_r15(cur_instr_addr_ + 4u);
}

void ArmCore::thumb32_load_store_single() {
    const u32 hw1 = cur_instr_ >> 16;
    const u32 hw2 = cur_instr_ & 0xFFFFu;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rt = static_cast<int>((hw2 >> 12) & 0xFu);
    const bool signed_space = (hw1 & 0x100u) != 0u;
    const bool imm12_form = (hw1 & 0x80u) != 0u;
    const bool is_word = (hw1 & 0x40u) != 0u;
    const bool is_half = (hw1 & 0x20u) != 0u;
    const bool l = (hw1 & 0x10u) != 0u;

    // The register-offset form is marked by hw2[11:6] == 0, NOT by hw2[11]:
    // in the 12-bit immediate forms hw2[11:0] *is* the offset, so offsets below
    // 0x800 would otherwise be misread as `[Rn, Rm]` (the kernel boot loader's
    // page-table writer stores descriptors with `str.w r3,[r0,r2,lsl#2]`,
    // hw1 = 0xF840, hw2 = 0x3022, and fills its tables with 12-bit immediate
    // stores such as `str.w r7,[r5,#0xB0]`, hw1 = 0xF8C5, hw2 = 0x70B0).
    const bool register_form = (hw2 & 0x0FC0u) == 0u;
    // Offset of the register form: Rm shifted by imm2 (0..3).
    const auto register_offset = [&]() {
        const int rm = static_cast<int>(hw2 & 0xFu);
        return r[rm] << ((hw2 >> 4) & 3u);
    };

    if (signed_space) {
        const u32 size_sel = hw1 & 0x30u;
        if (size_sel != 0x10u && size_sel != 0x30u) {
            undefined("T32 NEON load/store");
            return;
        }
        const bool half = size_sel == 0x30u;
        if (imm12_form) {
            const u32 addr = read_reg(rn) + (hw2 & 0xFFFu);
            const u32 value = half ? mem_read_half(addr) : mem_read_byte(addr);
            if (pending_fault_ != arm::FaultKind::None) return;
            r[rt] = half ? static_cast<u32>(static_cast<s32>(static_cast<s16>(value & 0xFFFFu)))
                         : static_cast<u32>(static_cast<s32>(static_cast<s8>(value & 0xFFu)));
        } else if (register_form) {
            const u32 addr = read_reg(rn) + register_offset();
            const u32 value = half ? mem_read_half(addr) : mem_read_byte(addr);
            if (pending_fault_ != arm::FaultKind::None) return;
            r[rt] = half ? static_cast<u32>(static_cast<s32>(static_cast<s16>(value & 0xFFFFu)))
                         : static_cast<u32>(static_cast<s32>(static_cast<s8>(value & 0xFFu)));
        } else {
            const bool p = (hw2 & 0x400u) != 0u;
            const bool u = (hw2 & 0x200u) != 0u;
            const bool w = (hw2 & 0x100u) != 0u;
            const u32 imm8 = hw2 & 0xFFu;
            const u32 base_addr = read_reg(rn);
            const u32 addr = p ? (u ? base_addr + imm8 : base_addr - imm8) : base_addr;
            const u32 value = half ? mem_read_half(addr) : mem_read_byte(addr);
            if (pending_fault_ != arm::FaultKind::None) return;
            r[rt] = half ? static_cast<u32>(static_cast<s32>(static_cast<s16>(value & 0xFFFFu)))
                         : static_cast<u32>(static_cast<s32>(static_cast<s8>(value & 0xFFu)));
            // T1: W == 1 means writeback, for both the pre- and post-indexed forms.
            if (w && rn != 15) r[rn] = u ? base_addr + imm8 : base_addr - imm8;
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    // A load with Rt == 1111 is the PLD/PLI preload hint (no effect).
    if (l && rt == 15) {
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    const int size = is_word ? 2 : (is_half ? 1 : 0);

    if (imm12_form) {
        const u32 imm12 = hw2 & 0xFFFu;
        const u32 addr = read_reg(rn) + imm12;
        if (l) {
            u32 value;
            if (size == 0) value = mem_read_byte(addr) & 0xFFu;
            else if (size == 1) value = mem_read_half(addr & ~1u) & 0xFFFFu;
            else value = mem_read_word(addr & ~3u, false);
            if (pending_fault_ != arm::FaultKind::None) return;
            r[rt] = value;
        } else {
            if (size == 0) mem_write_byte(addr, r[rt] & 0xFFu);
            else if (size == 1) mem_write_half(addr & ~1u, r[rt] & 0xFFFFu);
            else mem_write_word(addr & ~3u, r[rt]);
            if (pending_fault_ != arm::FaultKind::None) return;
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    if (register_form) {
        // T2: [Rn, Rm, lsl #imm2] - no writeback.
        const u32 addr = read_reg(rn) + register_offset();
        if (l) {
            u32 value;
            if (size == 0) value = mem_read_byte(addr) & 0xFFu;
            else if (size == 1) value = mem_read_half(addr & ~1u) & 0xFFFFu;
            else value = mem_read_word(addr & ~3u, false);
            if (pending_fault_ != arm::FaultKind::None) return;
            r[rt] = value;
        } else {
            if (size == 0) mem_write_byte(addr, r[rt] & 0xFFu);
            else if (size == 1) mem_write_half(addr & ~1u, r[rt] & 0xFFFFu);
            else mem_write_word(addr & ~3u, r[rt]);
            if (pending_fault_ != arm::FaultKind::None) return;
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    // 8-bit immediate with P/U/W.
    const bool p = (hw2 & 0x400u) != 0u;
    const bool u = (hw2 & 0x200u) != 0u;
    const bool w = (hw2 & 0x100u) != 0u;
    const u32 imm8 = hw2 & 0xFFu;
    const u32 base_addr = read_reg(rn);
    const u32 addr = p ? (u ? base_addr + imm8 : base_addr - imm8) : base_addr;
    if (l) {
        u32 value;
        if (size == 0) value = mem_read_byte(addr) & 0xFFu;
        else if (size == 1) value = mem_read_half(addr & ~1u) & 0xFFFFu;
        else value = mem_read_word(addr & ~3u, false);
        if (pending_fault_ != arm::FaultKind::None) return;
        r[rt] = value;
    } else {
        if (size == 0) mem_write_byte(addr, r[rt] & 0xFFu);
        else if (size == 1) mem_write_half(addr & ~1u, r[rt] & 0xFFFFu);
        else mem_write_word(addr & ~3u, r[rt]);
        if (pending_fault_ != arm::FaultKind::None) return;
    }
    if (w && rn != 15) r[rn] = u ? base_addr + imm8 : base_addr - imm8;
    write_r15(cur_instr_addr_ + 4u);
}

void ArmCore::thumb32_coprocessor() {
    const u32 cp = (cur_instr_ >> 8) & 0xFu;
    const bool load = (cur_instr_ & (1u << 20)) != 0;

    if (cp == 10u || cp == 11u) {
        if ((cur_instr_ & 0x0E000000u) == 0x0C000000u) {
            if ((cur_instr_ & 0x02000000u) != 0u) {
                execute_vfp_load_store_multiple(cur_instr_, (cur_instr_ & 0x100u) != 0);
            } else {
                execute_vfp_load_store(cur_instr_, cp, (cur_instr_ & 0x100u) != 0);
            }
            return;
        }
        execute_arm_vfp(cur_instr_);
        return;
    }
    if (cp == 15u) {
        const int rd = static_cast<int>((cur_instr_ >> 12) & 0xFu);
        const u32 o1 = (cur_instr_ >> 21) & 7u;
        const u32 crn = (cur_instr_ >> 16) & 0xFu;
        const u32 crm = cur_instr_ & 0xFu;
        const u32 o2 = (cur_instr_ >> 5) & 7u;
        if (load) {
            const u32 value = cp15_read(o1, crn, crm, o2, rd);
            if (rd != 15) r[rd] = value;
        } else {
            cp15_write(o1, crn, crm, o2, rd == 15 ? 0u : r[rd]);
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }
    ZLB_LOG_DBG("cpu", "%s T32 coprocessor p%u ignored", name.c_str(), cp);
    write_r15(cur_instr_addr_ + 4u);
}

void ArmCore::thumb32_branch_misc() {
    const u32 hw1 = cur_instr_ >> 16;
    const u32 hw2 = cur_instr_ & 0xFFFFu;
    const u32 sv = (hw1 >> 10) & 1u;
    const u32 j1 = (hw2 >> 13) & 1u;
    const u32 j2 = (hw2 >> 11) & 1u;
    const u32 imm11 = hw2 & 0x7FFu;

    // The IT state was already advanced by execute_thumb32().
    bool in_it = false;
    bool last_in_it = false;
    const u32 it_cond = thumb_condition_peek(in_it, last_in_it);
    (void)it_cond;
    if (in_it && last_in_it && (hw2 & 0x4000u) == 0u && (hw2 & 0x1000u) == 0u &&
        ((hw1 >> 6) & 0xFu) <= 0xDu) {
        undefined("T32 conditional branch as the last instruction of an IT block");
        return;
    }

    if ((hw2 & 0x4000u) == 0u) {
        if ((hw2 & 0x1000u) != 0u) {
            // T4: B.W (unconditional).
            const u32 imm10 = hw1 & 0x3FFu;
            const u32 i1 = (~(j1 ^ sv)) & 1u;
            const u32 i2 = (~(j2 ^ sv)) & 1u;
            const s32 offset =
                arm::sign_extend32((sv << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1), 25);
            write_r15(cur_instr_addr_ + 4u + static_cast<u32>(offset));
            return;
        }
        const u32 cond = (hw1 >> 6) & 0xFu;
        if (cond <= 0xDu) {
            // T3: B<c>.W.
            const u32 imm6 = hw1 & 0x3Fu;
            const s32 offset =
                arm::sign_extend32((sv << 20) | (j2 << 19) | (j1 << 18) | (imm6 << 12) | (imm11 << 1), 21);
            if (arm::condition_passed(cond, cpsr)) {
                write_r15(cur_instr_addr_ + 4u + static_cast<u32>(offset));
            } else {
                write_r15(cur_instr_addr_ + 4u);
            }
            return;
        }
        // cond == 111x: miscellaneous control.
        thumb32_misc();
        return;
    }

    // hw2[15:14] == 11: BLX (hw2[12] == 0) or BL (hw2[12] == 1).
    const u32 imm10 = hw1 & 0x3FFu;
    const u32 i1 = (~(j1 ^ sv)) & 1u;
    const u32 i2 = (~(j2 ^ sv)) & 1u;
    if ((hw2 & 0x1000u) != 0u) {
        const s32 offset =
            arm::sign_extend32((sv << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1), 25);
        r[14] = (cur_instr_addr_ + 4u) | 1u;
        write_r15(cur_instr_addr_ + 4u + static_cast<u32>(offset));
        return;
    }
    if ((hw2 & 1u) != 0u) {
        undefined("T32 BLX with hw2[0] == 1");
        return;
    }
    const u32 imm10l = (hw2 >> 1) & 0x3FFu;
    const s32 offset =
        arm::sign_extend32((sv << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm10l << 2), 25);
    const u32 target = (cur_instr_addr_ + 4u) & ~3u;
    r[14] = (cur_instr_addr_ + 4u) | 1u;
    thumb = false;
    cpsr &= ~arm::kFlagT;
    write_r15(target + static_cast<u32>(offset));
}

void ArmCore::thumb32_misc() {
    const u32 hw1 = cur_instr_ >> 16;
    const u32 hw2 = cur_instr_ & 0xFFFFu;
    const int rd = static_cast<int>((hw2 >> 8) & 0xFu);
    const int rm = static_cast<int>(hw2 & 0xFu);

    // Hints: 1111 0011 1010 ....
    if (hw1 == 0xF3AFu && (hw2 & 0x8000u) != 0u) {
        write_r15(cur_instr_addr_ + 4u);
        return;
    }
    if (hw1 == 0xF3BFu && (hw2 & 0xFF00u) == 0x8F00u) {
        // CLREX / DSB / DMB / ISB.
        if ((hw2 & 0xF0u) == 0x10u) {
            exclusive_valid_ = false;
            bus->clear_exclusive_for(static_cast<int>(core_id_));
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }
    // MRS: 1111 0011 1110 1111 1000 Rd 0000 0000.
    if (hw1 == 0xF3EFu && (hw2 & 0xF000u) == 0x8000u) {
        const bool use_spsr = (hw2 & 0x100u) != 0u;
        r[rd] = use_spsr ? spsr() : cpsr;
        write_r15(cur_instr_addr_ + 4u);
        return;
    }
    // MSR (register): 1111 0011 1000 1110 1000 Rd 0000 Rm.
    if ((hw1 & 0xFFF0u) == 0xF380u && (hw2 & 0xF000u) == 0x8000u) {
        const u32 sysm = (hw2 >> 8) & 0xFu;
        const bool use_spsr = (hw2 & 0x100u) != 0u;
        const u32 value = r[rm];
        u32 mask = 0xFFFFFFFFu;
        if (sysm == 0u) mask = 0x000000FFu;
        else if (sysm == 1u) mask = 0x0000FF00u;
        else if (sysm == 2u) mask = 0x00FF0000u;
        else if (sysm == 3u) mask = 0xFF000000u;
        if (use_spsr) set_spsr((spsr() & ~mask) | (value & mask));
        else write_cpsr_masked(value, mask);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }
    // SMC.W / SVC.W / HVC.W / UDF.W.
    // 0xF7F0 0x80i0 is SMC (Secure Monitor Call) - the second halfword carries
    // the second imm4 and must be 0x8000..0x800F.  Anything else in the 0xF7F0
    // space stays the SVC-like fallback the reference core used.
    if ((hw1 & 0xFFF0u) == 0xF7F0u && (hw2 & 0xFFF0u) == 0x8000u) {
        ZLB_LOG_DBG("cpu", "%s SMC.W #%u -> monitor (MVBAR=0x%08X, %s world) at %08X", name.c_str(),
                    ((hw1 & 0xFu) << 4) | (hw2 & 0xFu), mvbar, secure_state() ? "secure" : "non-secure",
                    cur_instr_addr_);
        take_exception(arm::kVecMonitorSmc, arm::kModeMonitor, cur_instr_addr_ + 4u);
        return;
    }
    if ((hw1 & 0xFFF0u) == 0xF7F0u) {
        take_exception(arm::kVecSupervisor, arm::kModeSupervisor, cur_instr_addr_ + 4u);
        return;
    }
    if ((hw1 & 0xFFE0u) == 0xF7E0u) {
        take_exception(arm::kVecHyp, arm::kModeHyp, cur_instr_addr_ + 4u);
        return;
    }
    if (hw1 == 0xF3AFu && (hw2 & 0xFFF0u) == 0x80F0u) {
        write_r15(cur_instr_addr_ + 4u);  // DBG
        return;
    }

    undefined("T32 misc control");
}

// ===========================================================================
// VFP
// ===========================================================================

void ArmCore::execute_arm_vfp(u32 instr) {
    const u32 cpnum = (instr >> 8) & 0xFu;

    if (cpnum == 10u || cpnum == 11u) {
        if ((instr & 0x0E000000u) == 0x0C000000u) {
            const bool single_ls = cpnum == 10u;
            if ((instr & (1u << 24)) != 0u && (instr & (1u << 21)) == 0u) {
                execute_vfp_load_store(instr, cpnum, single_ls);
            } else {
                execute_vfp_load_store_multiple(instr, single_ls);
            }
            return;
        }
        if ((instr & 0x10u) == 0u) {
            execute_vfp_data_processing(instr, cpnum == 10u);
            return;
        }
        execute_vfp_two_register(instr, cpnum == 10u);
        return;
    }

    undefined("Advanced SIMD / coprocessor VFP space");
}

void ArmCore::execute_vfp_data_processing(u32 instr, bool cp10) {
    (void)cp10;
    if (!vfp.enabled()) {
        undefined("VFP disabled (FPEXC.EN == 0)");
        return;
    }

    const bool sz = (instr & 0x100u) != 0;  // bit 8: 0 = single, 1 = double
    const u32 opcv1 = (instr >> 20) & 0xFu;
    const u32 opcv2 = (instr >> 6) & 0xFu;

    const u32 f_vd = (instr >> 12) & 0xFu;
    const u32 f_vn = (instr >> 16) & 0xFu;
    const u32 f_vm = instr & 0xFu;
    const bool b_d = (instr & (1u << 22)) != 0u;
    const bool b_n = (instr & 0x80u) != 0u;
    const bool b_m = (instr & 0x20u) != 0u;
    const int vd = sz ? static_cast<int>(f_vd | (b_d ? 16u : 0u)) : static_cast<int>((f_vd << 1) | (b_d ? 1u : 0u));
    const int vn = sz ? static_cast<int>(f_vn | (b_n ? 16u : 0u)) : static_cast<int>((f_vn << 1) | (b_n ? 1u : 0u));
    const int vm = sz ? static_cast<int>(f_vm | (b_m ? 16u : 0u)) : static_cast<int>((f_vm << 1) | (b_m ? 1u : 0u));

    execute_vfp_op(sz, instr, opcv1, opcv2, vd, vn, vm);
}

void ArmCore::execute_vfp_op(bool dbl, u32 instr, u32 opcv1, u32 opcv2, int vd, int vn, int vm) {
    (void)opcv1;
    // The operation selector is bits [23:20] with bit 22 removed (bit 22 is the
    // destination high bit D). Bit 6 is opc3 and bit 7 is the N bit.
    const u32 sel = (((instr >> 23) & 1u) << 2) | (((instr >> 21) & 1u) << 1) | ((instr >> 20) & 1u);
    const bool opc3 = (instr & 0x40u) != 0;
    const bool n_bit = (instr & 0x80u) != 0;
    const u32 opc2v = (instr >> 16) & 0xFu;

    if (sel == 7u) {
        if (!opc3) {
            // VMOV (immediate): imm8 = imm4H(bits [19:16]) : imm4L(bits [3:0]).
            const u32 imm4h = (instr >> 16) & 0xFu;
            const u32 imm8 = (imm4h << 4) | (instr & 0xFu);
            if (dbl) vfp.write_d(vd, ArmVfp::expand_immediate64(imm8));
            else vfp.write_s(vd, ArmVfp::expand_immediate(imm8));
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        if (opc2v == 4u) {
            // VCMP (N == 0) / VCMPE (N == 1); Vm == 0 compares with 0.0.
            if (dbl) vfp.compare(vfp.read_f64(vd), vm == 0 ? 0.0 : vfp.read_f64(vm), n_bit);
            else {
                vfp.compare(static_cast<f64>(vfp.read_f32(vd)),
                            vm == 0 ? 0.0 : static_cast<f64>(vfp.read_f32(vm)), n_bit);
            }
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        if (opc2v == 8u || opc2v == 0xCu || opc2v == 0xDu) {
            // VCVT / VCVTR between floating point and 32-bit integers. The integer
            // operand is always addressed as an S register.
            const int sd = static_cast<int>((((instr >> 12) & 0xFu) << 1) | ((instr >> 22) & 1u));
            const int sm = static_cast<int>(((instr & 0xFu) << 1) | ((instr >> 5) & 1u));
            if (opc2v == 8u) {
                const u32 raw = vfp.read_s(sm);
                if (dbl) vfp.write_f64(vd, n_bit ? static_cast<f64>(static_cast<s32>(raw))
                                                 : static_cast<f64>(raw));
                else vfp.write_f32(vd, n_bit ? static_cast<f32>(static_cast<s32>(raw))
                                             : static_cast<f32>(raw));
            } else {
                const bool signed_int = (opc2v == 0xDu) || (opc2v == 0xCu ? false : n_bit);
                const u32 value = dbl ? vfp.float_to_int(vfp.read_f64(vm), !signed_int)
                                      : vfp.float_to_int(static_cast<f64>(vfp.read_f32(vm)), !signed_int);
                vfp.write_s(sd, value);
            }
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        if (opc2v == 1u) {
            if (n_bit) {
                if (dbl) vfp.write_f64(vd, vfp.sqrt_double(vfp.read_f64(vm)));
                else vfp.write_f32(vd, vfp.sqrt_single(vfp.read_f32(vm)));
            } else {
                if (dbl) vfp.write_f64(vd, -vfp.read_f64(vm));
                else vfp.write_f32(vd, -vfp.read_f32(vm));
            }
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        if (opc2v == 0u) {
            if (dbl) vfp.write_f64(vd, n_bit ? std::fabs(vfp.read_f64(vm)) : vfp.read_f64(vm));
            else vfp.write_f32(vd, n_bit ? std::fabs(vfp.read_f32(vm)) : vfp.read_f32(vm));
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
        undefined("VFP 2-register opc2");
        return;
    }

    if (sel == 4u && opc3) {
        undefined("VFP VDIV with opc3 == 1");
        return;
    }

    if (dbl) {
        const f64 a = vfp.read_f64(vn);
        const f64 b = vfp.read_f64(vm);
        const f64 d = vfp.read_f64(vd);
        switch (sel) {
            case 0u:  // VMLA / VMLS
                vfp.write_f64(vd, opc3 ? vfp.sub_double(d, vfp.mul_double(a, b))
                                       : vfp.add_double(d, vfp.mul_double(a, b)));
                break;
            case 1u:  // VNMLS (opc3 == 0) / VNMLA (opc3 == 1)
                vfp.write_f64(vd, opc3 ? vfp.add_double(d, vfp.mul_double(a, b))
                                       : vfp.sub_double(d, vfp.mul_double(a, b)));
                break;
            case 2u: {  // VMUL / VNMUL
                const f64 p = vfp.mul_double(a, b);
                vfp.write_f64(vd, opc3 ? -p : p);
                break;
            }
            case 3u:  // VADD / VSUB
                vfp.write_f64(vd, opc3 ? vfp.sub_double(a, b) : vfp.add_double(a, b));
                break;
            case 4u:  // VDIV
                vfp.write_f64(vd, vfp.div_double(a, b));
                break;
            case 5u:  // VFNMS (opc3 == 0) / VFNMA (opc3 == 1)
                vfp.write_f64(vd, opc3 ? vfp.sub_double(vfp.mul_double(a, b), d)
                                       : -vfp.add_double(d, vfp.mul_double(a, b)));
                break;
            default:  // VFMA / VFMS
                vfp.write_f64(vd, opc3 ? vfp.sub_double(d, vfp.mul_double(a, b))
                                       : vfp.add_double(d, vfp.mul_double(a, b)));
                break;
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    const f32 af = vfp.read_f32(vn);
    const f32 bf = vfp.read_f32(vm);
    const f32 df = vfp.read_f32(vd);
    switch (sel) {
        case 0u:
            vfp.write_f32(vd, opc3 ? vfp.sub_single(df, vfp.mul_single(af, bf))
                                   : vfp.add_single(df, vfp.mul_single(af, bf)));
            break;
        case 1u:
            vfp.write_f32(vd, opc3 ? vfp.add_single(df, vfp.mul_single(af, bf))
                                   : vfp.sub_single(df, vfp.mul_single(af, bf)));
            break;
        case 2u: {
            const f32 p = vfp.mul_single(af, bf);
            vfp.write_f32(vd, opc3 ? -p : p);
            break;
        }
        case 3u:
            vfp.write_f32(vd, opc3 ? vfp.sub_single(af, bf) : vfp.add_single(af, bf));
            break;
        case 4u:
            vfp.write_f32(vd, vfp.div_single(af, bf));
            break;
        case 5u:
            vfp.write_f32(vd, opc3 ? vfp.sub_single(vfp.mul_single(af, bf), df)
                                   : -vfp.add_single(df, vfp.mul_single(af, bf)));
            break;
        default:
            vfp.write_f32(vd, opc3 ? vfp.sub_single(df, vfp.mul_single(af, bf))
                                   : vfp.add_single(df, vfp.mul_single(af, bf)));
            break;
    }
    write_r15(cur_instr_addr_ + 4u);
}

// ---------------------------------------------------------------------------
// Advanced SIMD (NEON)
//
// The Cortex-A9 has NEON and it shares the floating-point register file with
// VFP. This core only implements the small subset the firmware bring-up path
// needs -- the "clear the register file" idiom
//
//     vdup.32 q0, r0       ; 0xEEA00B10, r0 == 0
//     vorr    q1, q0, q0   ; 0xF22x x150   (x15 of them)
//
// -- and reports every other Advanced SIMD encoding as an *undefined
// instruction* (which the caller surfaces as StepResult::faulted) rather than
// letting it silently do nothing and corrupt state. The two implemented forms
// are exact, not approximations: VDUP.32 replicates a core register into all
// four 32-bit lanes, and VORR (register) is a bitwise OR of two vectors.
//
// Encodings (ARM ARM A8.8.395 VDUP, A8.8.336 VORR):
//   VDUP.32 Qd, Rt : cond 1110 1 D 10 imm4 Vd 1011 00 opc Q M 0
//                    (must be `(instr & 0x0FB00FF0) == 0x0EA00B10`)
//   VORR   Qd,Qn,Qm: 1111 0010 0 D 0 Vn Vd 0001 N Q M 1 Vm  (bit 6 = Q)
// ---------------------------------------------------------------------------
void ArmCore::execute_neon(u32 instr) {
    // ---- VDUP (ARM core register to all lanes) ----------------------------
    // `cond 1110 1 D 10 imm4 Vd 1011 00 opc Q M 0`, size .32 when opc == 00.
    if ((instr & 0x0FB00FF0u) == 0x0EA00B10u) {
        const int dd = static_cast<int>((instr >> 16) & 0xFu);   // imm4 = destination d
        const int rt = static_cast<int>((instr >> 12) & 0xFu);
        if ((dd & 1) != 0) {
            undefined("NEON VDUP.32 with an odd destination register");
            return;
        }
        const u64 value = static_cast<u64>(r[rt]) | (static_cast<u64>(r[rt]) << 32);
        vfp.write_d32(dd, value);
        vfp.write_d32(dd + 1, value);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }
    // The .16 / .8 forms of the same encoding.
    if ((instr & 0x0FB00FF0u) == 0x0EA00B30u || (instr & 0x0FB00FF0u) == 0x0EE00B10u) {
        const bool half = (instr & 0x20u) != 0u;
        const int dd = static_cast<int>((instr >> 16) & 0xFu);
        const int rt = static_cast<int>((instr >> 12) & 0xFu);
        if ((dd & 1) != 0) {
            undefined("NEON VDUP with an odd destination register");
            return;
        }
        const u32 lane = half ? (r[rt] & 0xFFFFu) : (r[rt] & 0xFFu);
        u64 value = 0;
        const int width = half ? 16 : 8;
        for (int offset = 0; offset < 64; offset += width) {
            value |= static_cast<u64>(lane) << offset;
        }
        vfp.write_d32(dd, value);
        vfp.write_d32(dd + 1, value);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    // ---- Advanced SIMD data processing (register) -------------------------
    // `1111 001 U 0 D size Vn Vd opc1 N Q M opc2 Vm`. U is bit 24, size is bits
    // [21:20], opc1 is bits [11:8], opc2 is bit 4. For opc1 == 0001 with
    // opc2 == 1 the size field selects the *logical* ops, which ignore the
    // element size; opc1 == 0011 selects the bitwise select family.
    if ((instr & 0xFE800000u) == 0xF2000000u) {
        const u32 opc1 = (instr >> 8) & 0xFu;
        const bool opc2 = (instr & 0x10u) != 0u;
        const u32 size = (instr >> 20) & 3u;
        const bool u_bit = (instr & (1u << 24)) != 0u;
        const int d = (static_cast<int>((instr >> 22) & 1u) << 4) | static_cast<int>((instr >> 12) & 0xFu);
        const int n = (static_cast<int>((instr >> 7) & 1u) << 4) | static_cast<int>((instr >> 16) & 0xFu);
        const int m = (static_cast<int>((instr >> 5) & 1u) << 4) | static_cast<int>(instr & 0xFu);
        const bool quad = (instr & 0x40u) != 0u;
        const int lanes = quad ? 2 : 1;

        enum class LogicOp { None, And, Bic, Orr, Orn, Eor, Bsl, Bit, Bif };
        LogicOp op = LogicOp::None;
        if (opc1 == 0x1u && opc2 && !u_bit) {
            switch (size) {
                case 0u: op = LogicOp::And; break;
                case 1u: op = LogicOp::Bic; break;
                case 2u: op = LogicOp::Orr; break;
                default: op = LogicOp::Orn; break;
            }
        } else if (opc1 == 0x3u && opc2 && !u_bit) {
            switch (size) {
                case 0u: op = LogicOp::Eor; break;
                case 1u: op = LogicOp::Bsl; break;
                case 2u: op = LogicOp::Bit; break;
                default: op = LogicOp::Bif; break;
            }
        }

        if (op != LogicOp::None) {
            if (!quad && ((d & 1) != 0 || (n & 1) != 0 || (m & 1) != 0)) {
                undefined("NEON logical op with a misaligned double register");
                return;
            }
            for (int k = 0; k < lanes; ++k) {
                const int da = (d & ~1) + k;
                const int na = (n & ~1) + k;
                const int ma = (m & ~1) + k;
                const u64 src_n = vfp.read_d32(na);
                const u64 src_m = vfp.read_d32(ma);
                const u64 src_d = vfp.read_d32(da);
                u64 result = 0;
                switch (op) {
                    case LogicOp::And: result = src_n & src_m; break;
                    case LogicOp::Bic: result = src_n & ~src_m; break;
                    case LogicOp::Orr: result = src_n | src_m; break;
                    case LogicOp::Orn: result = src_n | ~src_m; break;
                    case LogicOp::Eor: result = src_n ^ src_m; break;
                    case LogicOp::Bsl: result = src_d ^ ((src_d ^ src_m) & src_n); break;
                    case LogicOp::Bit: result = src_d ^ ((src_d ^ src_n) & src_m); break;
                    default: result = src_d ^ ((src_d ^ src_n) & ~src_m); break;
                }
                vfp.write_d32(da, result);
            }
            write_r15(cur_instr_addr_ + 4u);
            return;
        }

        // Everything else in the Advanced SIMD data-processing space: report it
        // clearly instead of guessing. The raw encoding is in the halt reason.
        char why[96];
        std::snprintf(why, sizeof(why), "Advanced SIMD data processing opc1=%X opc2=%u size=%u",
                      opc1, opc2 ? 1u : 0u, size);
        undefined(why);
        return;
    }

    // ---- Advanced SIMD element / structure load-store ---------------------
    if ((instr & 0x0F000000u) == 0x04000000u) {
        undefined("Advanced SIMD element/structure load-store not implemented");
        return;
    }

    undefined("Advanced SIMD (NEON) encoding not implemented");
}
void ArmCore::execute_vfp_two_register(u32 instr, bool cp10) {
    // VMSR/VMRS and the identification registers are *system register* accesses:
    // they must work whatever FPEXC.EN is, otherwise the VFP unit could never be
    // enabled in the first place. The FPEXC.EN gate below therefore applies only
    // to the data-processing side of this space.
    const u32 opcv1 = (instr >> 20) & 0xFu;
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    int dm = static_cast<int>((instr & 0xFu) << 1 | ((instr >> 5) & 1u));
    int dd = static_cast<int>(((instr >> 12) & 0xFu) << 1 | ((instr >> 22) & 1u));
    if (dm > 31) dm -= 32;
    if (dd > 31) dd -= 32;

    // Bit 4 == 1 selects the register-transfer half of the coprocessor 10/11
    // space. Inside it, bits [23:20] == 1010 or 1011 is the Advanced SIMD (NEON)
    // space, not VFP: 0xEEA00B10 is `vdup.32 q0, r0`, for instance. Without this
    // split the VFP transfer path reports NEON as an undefined "VFP register
    // transfer", which is what stopped the kernel boot loader.
    {
        const u32 neon_tag = (instr >> 20) & 0xFu;
        if (neon_tag == 0xAu || neon_tag == 0xBu) {
            execute_neon(instr);
            return;
        }
    }

    // VMOV two singles <-> one double: 1110 1110 1011 0 D 11 Vd 101 000 M 0.
    if ((instr & 0x0FE00F10u) == 0x0C400A10u || (instr & 0x0FE00F10u) == 0x0C400B10u) {
        const bool to_double = (instr & 0x100u) != 0;
        if (to_double) {
            const u64 value = static_cast<u64>(r[rd]) | (static_cast<u64>(r[(instr >> 16) & 0xFu]) << 32);
            vfp.write_d(dd >> 1, value);
        } else {
            const u64 value = vfp.read_d(dd >> 1);
            r[rd] = static_cast<u32>(value);
            r[(instr >> 16) & 0xFu] = static_cast<u32>(value >> 32);
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    const u32 idx = (instr >> 16) & 0xFu;
    const bool high = (instr & 0x80u) != 0u;
    const int sn_reg = static_cast<int>((idx << 1) | (high ? 1u : 0u));
    const int dn_reg = static_cast<int>(idx | (high ? 16u : 0u));

    // VMOV between a core register and a single/double register.
    if ((instr & 0x0F800E1Fu) == 0x0E000A10u) {
        const bool to_core = (instr & (1u << 20)) != 0;
        if (cp10) {
            if (to_core) r[rd] = vfp.read_s(sn_reg);
            else vfp.write_s(sn_reg, r[rd]);
        } else {
            const int lane = static_cast<int>((instr >> 21) & 1u);
            const u64 value = vfp.read_d(dn_reg);
            if (to_core) {
                r[rd] = static_cast<u32>((value >> (lane * 32)) & 0xFFFFFFFFull);
            } else {
                const u64 mask = 0xFFFFFFFFull << (lane * 32);
                vfp.write_d(dn_reg, (value & ~mask) | ((static_cast<u64>(r[rd]) << (lane * 32)) & mask));
            }
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    if (cp10 && (opcv1 == 0xEu || opcv1 == 0xFu)) {
        // VMRS / VMSR.  opcv1 here is bits 23..20, i.e. opc1 (bits 23..21) with the
        // L bit (bit 20) still attached, so VMSR is 0b1110 and VMRS is 0b1111.
        // These system-register transfers must work even while FPEXC.EN == 0 --
        // `vmsr fpexc, r0` is precisely the instruction that turns the unit on.
        // The transfer register is Rt at bits [15:12] and the system register
        // number is the reg field at bits [19:16]:
        //   VMSR FPEXC, Rt = 1110 1110 1110 Rt 1010 0001 0000
        //   VMRS Rt, FPEXC = 1110 1110 1111 Rt 1010 0001 0000
        // Bit 20 is the L bit (1 = VMRS, 0 = VMSR).
        const u32 reg = (instr >> 16) & 0xFu;
        const int rt = static_cast<int>((instr >> 12) & 0xFu);
        if ((instr & (1u << 20)) != 0) {
            u32 value;
            switch (reg) {
                case 0u: value = vfp.fpsid; break;
                case 1u: value = vfp.fpscr; break;
                case 6u: value = vfp.mvfr0; break;
                case 7u: value = vfp.mvfr1; break;
                default: value = 0u; break;
            }
            if (rt == 15) cpsr = (cpsr & 0x0FFFFFFFu) | (value & 0xF0000000u);
            else r[rt] = value;
        } else {
            if (reg == 1u) vfp.fpscr = (vfp.fpscr & 0x0FFFFFFFu) | (r[rt] & 0xF0000000u);
            else if (reg == 8u) vfp.fpexc = r[rt];
        }
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

    if ((instr & 0x0FB00F50u) == 0x0EB00A40u) {
        // VMOV immediate.
        const u32 imm4 = (instr >> 16) & 0xFu;
        const u32 imm3 = (instr >> 12) & 7u;
        const u32 cmode = (instr >> 8) & 0xFu;
        if ((cmode & 9u) == 8u && (instr & 0x100u) != 0) {
            if (!vfp.enabled()) {
                undefined("VFP disabled (FPEXC.EN == 0)");
                return;
            }
            const u32 imm8 = (imm4 << 4) | (imm3 << 1) | ((instr >> 6) & 1u);
            vfp.write_s(dd, ArmVfp::expand_immediate(imm8));
            write_r15(cur_instr_addr_ + 4u);
            return;
        }
    }

    undefined("VFP register transfer");
}

void ArmCore::execute_vfp_load_store(u32 instr, u32 cpnum, bool single) {
    (void)cpnum;
    if (!vfp.enabled()) {
        undefined("VFP disabled (FPEXC.EN == 0)");
        return;
    }
    const bool load = (instr & (1u << 20)) != 0;
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const bool d_bit = (instr & (1u << 22)) != 0;
    const bool p = (instr & (1u << 24)) != 0;
    const bool u = (instr & (1u << 23)) != 0;
    const bool w = (instr & (1u << 21)) != 0;
    const u32 imm8 = (instr & 0xFFu) * 4u;
    const u32 base_addr = read_reg(rn);
    const u32 addr = p ? (u ? base_addr + imm8 : base_addr - imm8) : base_addr;

    if (single) {
        const int s = static_cast<int>((((instr >> 12) & 0xFu) << 1) | (d_bit ? 1u : 0u));
        if (load) {
            const u32 value = mem_read_word(addr & ~3u, false);
            if (pending_fault_ == arm::FaultKind::None) vfp.write_s(s & 31, value);
        } else {
            mem_write_word(addr & ~3u, vfp.read_s(s & 31));
        }
    } else {
        const int d = static_cast<int>(((instr >> 12) & 0xFu) | (d_bit ? 0x10u : 0u));
        if (load) {
            const u64 value = mem_read_double(addr & ~7u);
            if (pending_fault_ == arm::FaultKind::None) vfp.write_d(d, value);
        } else {
            mem_write_double(addr & ~7u, vfp.read_d(d));
        }
    }
    if (pending_fault_ != arm::FaultKind::None) return;
    if (w && rn != 15) r[rn] = u ? base_addr + imm8 : base_addr - imm8;
    write_r15(cur_instr_addr_ + 4u);
}

void ArmCore::execute_vfp_load_store_multiple(u32 instr, bool single) {
    if (!vfp.enabled()) {
        undefined("VFP disabled (FPEXC.EN == 0)");
        return;
    }
    const bool load = (instr & (1u << 20)) != 0;
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const bool d_bit = (instr & (1u << 22)) != 0;
    const bool p = (instr & (1u << 24)) != 0;
    const bool u = (instr & (1u << 23)) != 0;
    const bool w = (instr & (1u << 21)) != 0;
    const u32 imm8 = instr & 0xFFu;

    const int count = single ? static_cast<int>(imm8) : static_cast<int>(imm8 / 2u);
    const int first = single ? static_cast<int>((((instr >> 12) & 0xFu) << 1) | (d_bit ? 1u : 0u))
                             : static_cast<int>(((instr >> 12) & 0xFu) | (d_bit ? 0x10u : 0u));

    const u32 base_addr = read_reg(rn);
    const u32 delta = static_cast<u32>(count * (single ? 4 : 8));
    u32 addr;
    if (u) addr = p ? base_addr + (single ? 4u : 8u) : base_addr;
    else addr = p ? base_addr - delta : base_addr - delta + (single ? 4u : 8u);

    for (int n = 0; n < count; ++n) {
        if (single) {
            const int s = (first + n) & 31;
            if (load) {
                const u32 value = mem_read_word(addr, false);
                if (pending_fault_ == arm::FaultKind::None) vfp.write_s(s, value);
            } else {
                mem_write_word(addr, vfp.read_s(s));
            }
            addr += 4u;
        } else {
            const int d = (first + n) & 15;
            if (load) {
                const u64 value = mem_read_double(addr);
                if (pending_fault_ == arm::FaultKind::None) vfp.write_d(d, value);
            } else {
                mem_write_double(addr, vfp.read_d(d));
            }
            addr += 8u;
        }
        if (pending_fault_ != arm::FaultKind::None) return;
    }

    if (w && rn != 15) r[rn] = u ? base_addr + delta : base_addr - delta;
    write_r15(cur_instr_addr_ + 4u);
}

// ===========================================================================
// Debugger interface
// ===========================================================================

std::string ArmCore::disassemble(u32 address, unsigned& length) {
    return arm_disassemble(*bus, address, thumb, length);
}

bool ArmCore::translate(u32 va, bool write, bool fetch, u32& pa, std::string& fault) {
    const arm::MmResult result = mmu.translate(va, write, fetch, mode());
    pa = result.ok ? result.phys_addr : 0u;
    if (result.ok) {
        fault.clear();
        return true;
    }
    fault = arm::fault_text(result.fault, va, write, fetch);
    return false;
}

void ArmCore::registers(std::vector<RegValue>& out) const {
    for (int i = 0; i < 16; ++i) {
        std::string note;
        if (i == 13) note = "SP";
        else if (i == 14) note = "LR";
        else if (i == 15) note = "PC";
        out.emplace_back("ARM", "R" + std::to_string(i), i == 15 ? r[15] : r[i], note);
    }
    const std::string flags = std::string(flag_n() ? "N" : "n") + (flag_z() ? "Z" : "z") +
                              (flag_c() ? "C" : "c") + (flag_v() ? "V" : "v");
    out.emplace_back("ARM", "CPSR", cpsr,
                     std::string(arm::mode_name(mode())) + (thumb ? " Thumb" : " ARM") + " " + flags);
    if (mode() != arm::kModeUser && mode() != arm::kModeSystem) {
        const u32 value = spsr();
        out.emplace_back("ARM", "SPSR", value, arm::mode_name(value & arm::kModeMask));
    }

    out.emplace_back("CP15", "MIDR", 0x410FC090u, "Cortex-A9 r0p0");
    out.emplace_back("CP15", "SCTLR", mmu.sctlr, mmu.enabled() ? "MMU on" : "MMU off");
    out.emplace_back("CP15", "TTBR0", mmu.ttbr0);
    out.emplace_back("CP15", "TTBR1", mmu.ttbr1);
    out.emplace_back("CP15", "TTBCR", mmu.ttbcr);
    out.emplace_back("CP15", "DACR", mmu.dacr);
    out.emplace_back("CP15", "DFSR", mmu.dfsr, arm::fault_name(mmu.last_walk.fault));
    out.emplace_back("CP15", "DFAR", mmu.dfar);
    out.emplace_back("CP15", "IFSR", mmu.ifsr);
    out.emplace_back("CP15", "IFAR", mmu.ifar);
    out.emplace_back("CP15", "ADFSR", mmu.adfsr);
    out.emplace_back("CP15", "AIFSR", mmu.aifsr);
    out.emplace_back("CP15", "VBAR", mmu.vbar);
    out.emplace_back("CP15", "VBAR_NS", vbar_nonsecure);
    out.emplace_back("CP15", "MVBAR", mvbar);
    out.emplace_back("CP15", "SCR", scr, secure_state() ? "secure world" : "non-secure world");
    out.emplace_back("CP15", "CONTEXTIDR", mmu.context_idr);
    out.emplace_back("CP15", "walks", mmu.walks);
    out.emplace_back("CP15", "faults", mmu.total_faults);

    out.emplace_back("VFP", "FPSCR", vfp.fpscr);
    out.emplace_back("VFP", "FPEXC", vfp.fpexc, vfp.enabled() ? "enabled" : "disabled");
    for (int i = 0; i < 16; ++i) {
        const u64 value = vfp.read_d(i);
        char note[24];
        std::snprintf(note, sizeof(note), "0x%016llX", static_cast<unsigned long long>(value));
        out.emplace_back("VFP", "D" + std::to_string(i), value, note);
    }
}

bool ArmCore::set_register(const std::string& name, u64 value) {
    if (name.empty()) return false;
    std::string key;
    key.reserve(name.size());
    for (char c : name) key.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    const u32 v = static_cast<u32>(value);

    if (key == "PC") {
        write_r15(v);
        return true;
    }
    if (key == "SP") {
        r[13] = v;
        return true;
    }
    if (key == "LR") {
        r[14] = v;
        return true;
    }
    if (key == "CPSR") {
        write_cpsr_masked(v, 0xFFFFFFFFu);
        return true;
    }
    if (key == "SPSR") {
        set_spsr(v);
        return true;
    }
    if (key == "FPSCR") {
        vfp.fpscr = v;
        return true;
    }
    if (key == "FPEXC") {
        vfp.fpexc = v;
        return true;
    }
    if (key == "THUMB") {
        thumb = v != 0;
        if (thumb) cpsr |= arm::kFlagT;
        else cpsr &= ~arm::kFlagT;
        return true;
    }
    if (key == "MMU") {
        if (v != 0) mmu.sctlr |= 1u;
        else mmu.sctlr &= ~1u;
        return true;
    }
    if (key == "TTBR0") { mmu.ttbr0 = v; return true; }
    if (key == "TTBR1") { mmu.ttbr1 = v; return true; }
    if (key == "TTBCR") { mmu.ttbcr = v; return true; }
    if (key == "DACR") { mmu.dacr = v; return true; }
    if (key == "VBAR" || key == "VBAR_S") { mmu.vbar = v; return true; }
    if (key == "VBAR_NS") { vbar_nonsecure = v; return true; }
    if (key == "MVBAR") { mvbar = v; return true; }
    if (key == "SCR") {
        scr = v;
        ns_ = (v & arm::kScrNs) != 0u;
        return true;
    }
    if (key == "CONTEXTIDR") { mmu.context_idr = v; return true; }
    if (key == "SCTLR") {
        mmu.sctlr = v;
        return true;
    }

    if (key.size() >= 2 && (key[0] == 'R' || key[0] == 'S' || key[0] == 'D')) {
        char* end = nullptr;
        const long index = std::strtol(key.c_str() + 1, &end, 10);
        if (end != key.c_str() + 1 && *end == '\0') {
            if (key[0] == 'R' && index >= 0 && index < 16) {
                r[index] = v;
                if (index == 15) write_r15(v);
                return true;
            }
            if (key[0] == 'S' && index >= 0 && index < 32) { vfp.write_s(static_cast<int>(index), v); return true; }
            if (key[0] == 'D' && index >= 0 && index < 16) { vfp.write_d(static_cast<int>(index), value); return true; }
        }
    }
    return false;
}

bool ArmCore::get_register(const std::string& name, u64& value) const {
    if (name.empty()) return false;
    std::string key;
    key.reserve(name.size());
    for (char c : name) key.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));

    if (key == "PC") { value = r[15]; return true; }
    if (key == "SP") { value = r[13]; return true; }
    if (key == "LR") { value = r[14]; return true; }
    if (key == "CPSR") { value = cpsr; return true; }
    if (key == "SPSR") { value = spsr(); return true; }
    if (key == "FPSCR") { value = vfp.fpscr; return true; }
    if (key == "FPEXC") { value = vfp.fpexc; return true; }
    if (key == "MIDR") { value = 0x410FC090u; return true; }
    if (key == "SCTLR") { value = mmu.sctlr; return true; }
    if (key == "DFSR") { value = mmu.dfsr; return true; }
    if (key == "DFAR") { value = mmu.dfar; return true; }
    if (key == "IFSR") { value = mmu.ifsr; return true; }
    if (key == "IFAR") { value = mmu.ifar; return true; }
    if (key == "TTBR0") { value = mmu.ttbr0; return true; }
    if (key == "TTBR1") { value = mmu.ttbr1; return true; }
    if (key == "TTBCR") { value = mmu.ttbcr; return true; }
    if (key == "DACR") { value = mmu.dacr; return true; }
    if (key == "VBAR") { value = mmu.vbar; return true; }
    if (key == "VBAR_NS") { value = vbar_nonsecure; return true; }
    if (key == "MVBAR") { value = mvbar; return true; }
    if (key == "SCR") { value = scr; return true; }
    if (key == "CONTEXTIDR") { value = mmu.context_idr; return true; }

    if (key.size() >= 2 && (key[0] == 'R' || key[0] == 'S' || key[0] == 'D')) {
        char* end = nullptr;
        const long index = std::strtol(key.c_str() + 1, &end, 10);
        if (end != key.c_str() + 1 && *end == '\0') {
            if (key[0] == 'R' && index >= 0 && index < 16) {
                value = index == 15 ? r[15] : r[index];
                return true;
            }
            if (key[0] == 'S' && index >= 0 && index < 32) {
                value = vfp.read_s(static_cast<int>(index));
                return true;
            }
            if (key[0] == 'D' && index >= 0 && index < 16) {
                value = vfp.read_d(static_cast<int>(index));
                return true;
            }
        }
    }
    return false;
}

std::string ArmCore::status_line() const {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%s %s %s%s%s%s %s mmu=%s", thumb ? "Thumb" : "ARM",
                  arm::mode_name(mode()), flag_n() ? "N" : "n", flag_z() ? "Z" : "z",
                  flag_c() ? "C" : "c", flag_v() ? "V" : "v",
                  secure_state() ? "secure" : "non-secure", mmu.enabled() ? "on" : "off");
    return std::string(buffer);
}

void ArmCore::describe_state(std::vector<std::string>& lines) const {
    // SCTLR/TTBR/DACR plus the CP15 c5/c6 fault registers and the last
    // translation, so an abort can be diagnosed from `info` alone.
    mmu.describe_extended(lines);
    lines.push_back(vfp.describe());
    lines.push_back(format("exceptions=%llu instructions=%llu cycles=%llu",
                           static_cast<unsigned long long>(exception_count),
                           static_cast<unsigned long long>(instructions),
                           static_cast<unsigned long long>(cycles)));
    const bool exclusive_held = bus != nullptr && bus->has_exclusive(static_cast<int>(core_id_));
    lines.push_back(format("vectors=0x%08X core=%u exclusive=%s", effective_vector_base(), core_id_,
                           exclusive_held ? "valid" : "none"));
    lines.push_back(format("trustzone: %s world, SCR=0x%08X MVBAR=0x%08X VBAR=0x%08X VBAR_NS=0x%08X",
                           secure_state() ? "secure" : "non-secure", scr, mvbar, mmu.vbar, vbar_nonsecure));
}

// ===========================================================================
// Factory
// ===========================================================================

std::unique_ptr<Cpu> create_arm_core(Bus& bus) { return std::make_unique<ArmCore>(bus); }

}  // namespace zlb
