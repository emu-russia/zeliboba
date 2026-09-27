// zeliboba - Toshiba MeP-c5 (CMeP) interpreter.
//
// The instruction semantics mirror the CGEN reference used by binutils/GDB
// 2.37; where the hardware is under specified (the control bus timer, the
// coprocessor condition register) the model is deliberately explicit so that
// the first loader run stays deterministic.
#include "cpu/mep/mep_core.h"

#include <cctype>
#include <cstdio>
#include <cstring>

#include "common/log.h"
#include "common/util.h"
#include "cpu/factory.h"

namespace zlb {
namespace {

constexpr u32 kPswInterruptEnable = 1u << 0;
constexpr u32 kPswException = 1u << 9;
constexpr u32 kPswHalt = 1u << 11;
constexpr u32 kPswOperatingMode = 1u << 12;

/// Control bus address space seen through `stcb`/`ldcb`.  The first loader only
/// touches the delay timer block at 0x400..0x405.
constexpr unsigned kCbTimerCount = 0x400;
constexpr unsigned kCbTimerCtrl = 0x402;
constexpr unsigned kCbTimerStatus = 0x404;

/// Number of instructions the built in one shot timer counts down from a zero
/// reload value.  Any finite value terminates the delay loop; the real hardware
/// counts bus clocks and the hardware layer can override the model.
constexpr u32 kCbTimerQuantum = 64;

bool signed_add_overflow(u32 a, u32 b) {
    const u32 result = a + b;
    return (((a ^ result) & (b ^ result)) & 0x80000000u) != 0;
}

bool signed_sub_overflow(u32 a, u32 b) {
    const u32 result = a - b;
    return (((a ^ result) & (~b ^ result)) & 0x80000000u) != 0;
}

}  // namespace

// ---------------------------------------------------------------------------
// Control bus model
// ---------------------------------------------------------------------------

void MePCore::ControlBus::reset() {
    regs.fill(0);
    count = 0;
    remaining = 0;
    running = false;
    done = false;
    force_expired = false;
}

bool MePCore::ControlBus::busy() const { return running && !force_expired; }

u32 MePCore::ControlBus::read(unsigned address) const {
    if (address >= regs.size()) return 0;
    u8 value = regs[address];
    if (address == kCbTimerStatus) {
        // Bit 0 is the "the count reached zero" latch, *not* a busy flag.  Both
        // poll loops in the boot chain wait for it to become 1:
        //   first loader  0x5E686: ldcb / and $0,$2 / beqz  -> loop while bit0 == 0
        //   second loader 0x45516: ldcb / and3 $3,$3,1 / bnez -> escape when bit0 != 0
        // Reading the status never changes it; software clears it by writing 0.
        value = static_cast<u8>((value & 0xFEu) | ((done || force_expired) ? 1u : 0u));
    }
    return value;
}

void MePCore::ControlBus::write(unsigned address, u32 value) {
    if (address >= regs.size()) return;
    regs[address] = static_cast<u8>(value & 0xFF);
    if (address == kCbTimerCount || address == kCbTimerCount + 1) {
        // The reload value is 16 bit at 0x400..0x401, big endian: the first
        // loader writes the low byte to 0x401 (0x5E67A) while the second loader
        // writes the high byte to 0x400 (0x45502).
        count = (static_cast<u32>(regs[kCbTimerCount]) << 8) | regs[kCbTimerCount + 1];
    } else if (address == kCbTimerStatus) {
        // Writing the status clears/sets the completion latch (0x5E67E and
        // 0x45508 both write zero before starting the next delay).
        done = (regs[kCbTimerStatus] & 0x01u) != 0;
        if (done) force_expired = false;
    } else if (address == kCbTimerCtrl) {
        if ((value & 0x01u) != 0) {
            // Start: reload the counter and drop the completion latch.  Zero
            // still means "wait one quantum" because the poll loop must observe
            // at least one sample with the latch clear.
            remaining = count != 0 ? count : kCbTimerQuantum;
            running = true;
            done = false;
            force_expired = false;
        } else {
            // Clearing the counter enable aborts the delay (shutdown path).
            running = false;
        }
    }
}

void MePCore::ControlBus::advance() {
    if (!running) return;
    if (remaining > 0) --remaining;
    if (remaining == 0) {
        running = false;
        done = true;
    }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

MePCore::MePCore(Bus& bus) : Cpu(bus) { name = "CMeP"; }

void MePCore::reset() { reset(reset_vector); }

void MePCore::reset(u32 entry) {
    r.fill(0);
    cond.fill(false);

    hi = lo = sar = lp = epc = npc = tmp = 0;
    psw = exc = cfg = vid = id = dbg = depc = opt = rcfg = ccfg = 0;
    cr0 = 0;
    rpb = rpe = rpc = 0;
    mb0 = me0 = mb1 = me1 = 0;

    cbus.reset();

    rep_active_ = false;
    rep_pending_back_ = false;
    rep_endless_ = false;
    vliw_mode = false;

    halted = false;
    halt_reason.clear();
    undefined_instruction = false;
    instructions = 0;
    cycles = 0;

    set_pc(entry);
    set_context(entry);
    // NOTE: a core reset must not power-cycle the board. The machine layer calls
    // Bus::reset_devices() itself; resetting them here wipes state that the boot
    // chain has already programmed (the ARM->CMeP mailbox, for instance).
}

void MePCore::prepare_reset_context(u64 a0, u64 a1, u64 a2, u64 a3) {
    (void)a1;
    (void)a2;
    (void)a3;
    // The CMeP reset hardware leaves the stack pointer the boot ROM expects in
    // $0; the first loader copies it to $sp with `mov $sp,$0`.
    r[0] = static_cast<u32>(a0);
}

void MePCore::tick(u64 cycles_) {
    cycles += cycles_;
    for (u64 i = 0; i < cycles_; ++i) cbus.advance();
}

// ---------------------------------------------------------------------------
// Memory
// ---------------------------------------------------------------------------

void MePCore::set_context(u32 address) {
    if (bus == nullptr) return;
    bus->context.core = name.c_str();
    bus->context.pc = address;
}

void MePCore::trace_watched_pc() {
    // Development aid: ZLB_MEP_PC=0x46556,0x468A2 prints the general registers
    // every time the CMeP reaches one of the listed addresses.  The loader's
    // error paths are only reachable through shared report routines, and the
    // debugger's breakpoints cannot be stepped past inside a single script, so a
    // plain address trace is the only way to see every pass through a check.
    static const std::vector<u32> watched = [] {
        std::vector<u32> list;
        const char* text = std::getenv("ZLB_MEP_PC");
        if (text == nullptr) return list;
        std::string current;
        for (const char* p = text;; ++p) {
            if (*p == ',' || *p == '\0') {
                if (!current.empty()) {
                    u32 value = 0;
                    if (parse_u32(current, value)) list.push_back(value);
                    current.clear();
                }
                if (*p == '\0') break;
            } else {
                current.push_back(*p);
            }
        }
        return list;
    }();
    if (watched.empty()) return;
    bool hit = false;
    for (u32 address : watched) {
        if (address == pc) {
            hit = true;
            break;
        }
    }
    if (!hit) return;
    std::fprintf(stderr,
                 "[mep] lp=%08X pc=%08X $0=%08X $1=%08X $2=%08X $3=%08X $4=%08X $5=%08X $6=%08X $7=%08X\n",
                 lp, pc, r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
}

u8 MePCore::load8(u32 address) {
    set_context(pc);
    return bus->read8(address);
}

u16 MePCore::load16(u32 address) {
    set_context(pc);
    return bus->read16(address);
}

u32 MePCore::load32(u32 address) {
    set_context(pc);
    return bus->read32(address);
}

void MePCore::store8(u32 address, u8 value) {
    set_context(pc);
    bus->write8(address, value);
}

void MePCore::store16(u32 address, u16 value) {
    set_context(pc);
    bus->write16(address, value);
}

void MePCore::store32(u32 address, u32 value) {
    set_context(pc);
    bus->write32(address, value);
}

// ---------------------------------------------------------------------------
// Control registers (the `stc`/`ldc` encoding space)
// ---------------------------------------------------------------------------

u32 MePCore::get_csr(int index) const {
    switch (index) {
        case 0: return pc;
        case 1: return lp;
        case 2: return sar;
        case 4: return rpb;
        case 5: return rpe;
        case 6: return rpc;
        case 7: return hi;
        case 8: return lo;
        case 12: return mb0;
        case 13: return me0;
        case 14: return mb1;
        case 15: return me1;
        case 16: return psw;
        case 17: return id;
        case 18: return tmp;
        case 19: return epc;
        case 20: return exc;
        case 21: return cfg;
        case 22: return vid;
        case 23: return npc;
        case 24: return dbg;
        case 25: return depc;
        case 26: return opt;
        case 27: return rcfg;
        case 28: return ccfg;
        default: return 0;
    }
}

void MePCore::set_csr(int index, u32 value) {
    switch (index) {
        case 0: set_pc(value); break;
        case 1: lp = value; break;
        case 2: sar = value; break;
        case 4:
            rpb = value;
            // Writing an empty loop begin register disarms the loop unit.
            if (value == 0) {
                rep_active_ = false;
                rep_endless_ = false;
            }
            break;
        case 5: rpe = value; break;
        case 6: rpc = value; break;
        case 7: hi = value; break;
        case 8: lo = value; break;
        case 12: mb0 = value; break;
        case 13: me0 = value; break;
        case 14: mb1 = value; break;
        case 15: me1 = value; break;
        case 16:
            psw = value;
            vliw_mode = (value & kPswOperatingMode) != 0;
            break;
        case 17: id = value; break;
        case 18: tmp = value; break;
        case 19: epc = value; break;
        case 20: exc = value; break;
        case 21: cfg = value; break;
        case 22: vid = value; break;
        case 23: npc = value; break;
        case 24: dbg = value; break;
        case 25: depc = value; break;
        case 26: opt = value; break;
        case 27: rcfg = value; break;
        case 28: ccfg = value; break;
        default: break;
    }
}

/// Control register number of a `stc`/`ldc`; the dedicated $lp/$hi/$lo forms
/// encode the register in the opcode instead of in the FCsrn field.
int stc_ldc_csr(const mep::Insn& insn, u32 word) {
    switch (insn.op) {
        case mep::Op::StcLp:
        case mep::Op::LdcLp: return 1;
        case mep::Op::StcHi:
        case mep::Op::LdcHi: return 7;
        case mep::Op::StcLo:
        case mep::Op::LdcLo: return 8;
        default: return static_cast<int>(mep::field_value(mep::Field::FCsrn, word, 0));
    }
}

// ---------------------------------------------------------------------------
// Coprocessor / control bus transfers
// ---------------------------------------------------------------------------

u32 MePCore::cop_access(const char* kind, unsigned size, u32 address, u32 value, bool load) {
    // No device models exist yet, so the control bus is answered locally: the
    // delay timer at 0x400..0x405 is the only location the first loader polls.
    (void)kind;
    if (load) {
        u32 out = 0;
        const unsigned width = size == 0 ? 1u : size;
        for (unsigned i = 0; i < width; ++i) {
            out |= static_cast<u32>(cbus.read(address + i)) << (8 * i);
        }
        if (width == 1) out &= 0xFFu;
        if (width == 2) out &= 0xFFFFu;
        return out;
    }
    const unsigned width = size == 0 ? 1u : size;
    for (unsigned i = 0; i < width; ++i) {
        cbus.write(address + i, (value >> (8 * i)) & 0xFFu);
    }
    return value;
}

void MePCore::cop_word(const mep::Insn& insn, u32 word, u32 address) {
    const int crn = static_cast<int>(mep::raw(word, 4, 4));
    const int rm = static_cast<int>(mep::raw(word, 8, 4));
    const u32 base = r[static_cast<size_t>(rm)];

    u32 disp = 0;
    switch (insn.op) {
        case mep::Op::Swcp:
        case mep::Op::Lwcp:
        case mep::Op::Swcpi:
        case mep::Op::Lwcpi: disp = 0; break;
        case mep::Op::Swcp16:
        case mep::Op::Lwcp16:
            disp = mep::field_value(mep::Field::F16s16, word, address);
            break;
        case mep::Op::Sbcp:
        case mep::Op::Lbcp:
        case mep::Op::Lbucp:
        case mep::Op::Shcp:
        case mep::Op::Lhcp:
        case mep::Op::Lhucp:
            disp = mep::field_value(mep::Field::F12s20, word, address);
            break;
        default: disp = mep::field_value(mep::Field::FCdisp10, word, address); break;
    }

    unsigned size = 4;
    switch (insn.op) {
        case mep::Op::Sbcp:
        case mep::Op::Lbcp:
        case mep::Op::Lbucp:
        case mep::Op::Sbcpa:
        case mep::Op::Lbcpa:
        case mep::Op::Sbcpm0:
        case mep::Op::Lbcpm0:
        case mep::Op::Sbcpm1:
        case mep::Op::Lbcpm1:
        case mep::Op::Lbucpa:
        case mep::Op::Lbucpm0:
        case mep::Op::Lbucpm1: size = 1; break;
        case mep::Op::Shcp:
        case mep::Op::Lhcp:
        case mep::Op::Lhucp:
        case mep::Op::Shcpa:
        case mep::Op::Lhcpa:
        case mep::Op::Shcpm0:
        case mep::Op::Lhcpm0:
        case mep::Op::Shcpm1:
        case mep::Op::Lhcpm1:
        case mep::Op::Lhucpa:
        case mep::Op::Lhucpm0:
        case mep::Op::Lhucpm1: size = 2; break;
        default: break;
    }

    bool load = false;
    switch (insn.op) {
        case mep::Op::Lwcp:
        case mep::Op::Lwcpi:
        case mep::Op::Lwcp16:
        case mep::Op::Lbcp:
        case mep::Op::Lbucp:
        case mep::Op::Lhcp:
        case mep::Op::Lhucp:
        case mep::Op::Lbcpa:
        case mep::Op::Lhcpa:
        case mep::Op::Lwcpa:
        case mep::Op::Lbcpm0:
        case mep::Op::Lhcpm0:
        case mep::Op::Lwcpm0:
        case mep::Op::Lbcpm1:
        case mep::Op::Lhcpm1:
        case mep::Op::Lwcpm1:
        case mep::Op::Lbucpa:
        case mep::Op::Lbucpm0:
        case mep::Op::Lbucpm1:
        case mep::Op::Lhucpa:
        case mep::Op::Lhucpm0:
        case mep::Op::Lhucpm1: load = true; break;
        default: break;
    }

    u32 target = base + disp;
    if (size == 2) target &= 0xFFFFFFFEu;
    else if (size == 4) target &= 0xFFFFFFFCu;

    if (load) {
        const char* kind = size == 4 ? "lwcp" : (size == 2 ? "lhcp" : "lbcp");
        r[static_cast<size_t>(crn)] = cop_access(kind, size, target, 0, true);
    } else {
        const char* kind = size == 4 ? "swcp" : (size == 2 ? "shcp" : "sbcp");
        cop_access(kind, size, target, r[static_cast<size_t>(crn)], false);
    }

    // The a/m forms are the addressing unit variants: they post-increment the
    // base register by the access size (modulo forms wrap between MB/ME).
    switch (insn.op) {
        case mep::Op::Swcpi:
        case mep::Op::Lwcpi:
        case mep::Op::Swcpa:
        case mep::Op::Lwcpa:
        case mep::Op::Swcpm0:
        case mep::Op::Lwcpm0:
        case mep::Op::Swcpm1:
        case mep::Op::Lwcpm1: r[static_cast<size_t>(rm)] = base + 4; break;
        case mep::Op::Sbcpa:
        case mep::Op::Lbcpa:
        case mep::Op::Sbcpm0:
        case mep::Op::Lbcpm0:
        case mep::Op::Sbcpm1:
        case mep::Op::Lbcpm1:
        case mep::Op::Lbucpa:
        case mep::Op::Lbucpm0:
        case mep::Op::Lbucpm1: r[static_cast<size_t>(rm)] = base + 1; break;
        case mep::Op::Shcpa:
        case mep::Op::Lhcpa:
        case mep::Op::Shcpm0:
        case mep::Op::Lhcpm0:
        case mep::Op::Shcpm1:
        case mep::Op::Lhcpm1:
        case mep::Op::Lhucpa:
        case mep::Op::Lhucpm0:
        case mep::Op::Lhucpm1: r[static_cast<size_t>(rm)] = base + 2; break;
        default: break;
    }
}

void MePCore::cop_word64(const mep::Insn& insn, u32 word, u32 address) {
    const int crn = static_cast<int>(mep::raw(word, 4, 4));
    const int rm = static_cast<int>(mep::raw(word, 8, 4));
    const bool load = insn.mnem[0] == 'l';

    u32 disp = 0;
    if (insn.op == mep::Op::Smcp16 || insn.op == mep::Op::Lmcp16) {
        disp = mep::field_value(mep::Field::F16s16, word, address);
    } else if (insn.op != mep::Op::Smcp && insn.op != mep::Op::Lmcp &&
               insn.op != mep::Op::Smcpi && insn.op != mep::Op::Lmcpi) {
        disp = mep::field_value(mep::Field::FCdisp10, word, address);
    }
    const u32 target = (r[static_cast<size_t>(rm)] + disp) & 0xFFFFFFF8u;
    if (!load) cop_access("smcp", 8, target, r[static_cast<size_t>(crn)], false);

    switch (insn.op) {
        case mep::Op::Smcpa:
        case mep::Op::Lmcpa:
        case mep::Op::Smcpm0:
        case mep::Op::Lmcpm0:
        case mep::Op::Smcpm1:
        case mep::Op::Lmcpm1:
        case mep::Op::Smcpi:
        case mep::Op::Lmcpi:
            r[static_cast<size_t>(rm)] = r[static_cast<size_t>(rm)] + 8;
            break;
        default: break;
    }
}

// ---------------------------------------------------------------------------
// Step
// ---------------------------------------------------------------------------

std::string MePCore::mark_undefined(const std::string& what, u32 address) {
    undefined_instruction = true;
    // The PC is deliberately left pointing at the offending instruction so the
    // debugger can stop on it and the caller can fix up the memory image.
    set_pc(address);
    const std::string fault = format("%s at 0x%08X", what.c_str(), address);
    ZLB_LOG_WARN("cpu", "[CMeP] %s", fault.c_str());
    return fault;
}

StepResult MePCore::step() {
    StepResult out;
    const u32 address = pc;
    out.address = address;

    // Machine hook (see MePCore::pc_hook): stop before executing this PC so the
    // caller can act on it.  The PC is left untouched and the reason is recorded
    // in halt_reason, so a debugger session sees why the core stopped.
    if (pc_hook && pc_hook(address)) {
        out.length = 0;
        out.was_branch = false;
        halt(format("machine hook at 0x%05X", address));
        return out;
    }

    set_context(address);
    trace_watched_pc();
    const u32 word = (bus->fetch16(address) | (static_cast<u32>(bus->fetch16(address + 2)) << 16));

    const mep::Insn* insn = mep::decode(word);
    out.length = insn->len;
    out.text = mep::format(*insn, word, address);

    if (insn->op == mep::Op::None || mep::is_reserved(insn->op)) {
        out.faulted = true;
        out.text = "*unknown*";
        out.length = 4;
        out.fault = mark_undefined(format("undefined instruction 0x%08X", word), address);
        return out;
    }

    // The PC is advanced first and branches overwrite it, which matches the
    // delay free MeP branches.
    set_pc(address + insn->len);

    execute(*insn, word, address);
    if (undefined_instruction) {
        out.faulted = true;
        out.fault = format("unimplemented instruction %s at 0x%08X", insn->mnem, address);
        return out;
    }

    ++cycles;
    ++instructions;
    cbus.advance();
    // A taken branch during a repeat block escapes the loop (see the comment on
    // repeat_step_end): the caller reports it because a branch to the *next*
    // instruction leaves no trace in the PC.
    repeat_step_end(address, branch_taken_);

    out.was_branch = pc != address + insn->len;
    return out;
}

/// The MeP hardware loop unit repeats the block [RPB, RPE] once per RPC count.
/// The instruction *after* RPE still retires before the branch back, so the loop
/// body the linker emits is [RPB, RPE] plus one trailing slot; `erepeat` keeps
/// going until that trailing slot is a *taken branch*, which is the loop's only
/// exit. Detecting it from the PC is not enough: the second loader's SC handshake
/// at 0x49540 is `beqi $2,0x1,0x49544`, whose target *is* the next instruction, so
/// the PC looks like a plain fall-through and the loop unit looped forever. The
/// caller therefore reports whether a branch was taken (`branch_taken_`).
void MePCore::repeat_step_end(u32 address, bool branch_taken) {
    if (!rep_active_) return;

    if (rep_pending_back_) {
        rep_pending_back_ = false;
        if (branch_taken) {
            rep_active_ = false;
            rep_endless_ = false;
            return;
        }
        if (rep_endless_) {
            set_pc(rpb);
        } else if (rpc != 0) {
            --rpc;
            set_pc(rpb);
        } else {
            rep_active_ = false;
        }
        return;
    }

    if (address == (rpe & 0xFFFFFFFEu)) rep_pending_back_ = true;
}

// ---------------------------------------------------------------------------
// Execute
// ---------------------------------------------------------------------------

void MePCore::branch_to(u32 target) {
    branch_taken_ = true;
    set_pc(target);
}

void MePCore::execute(const mep::Insn& insn, u32 word, u32 address) {
    using mep::Field;
    using mep::Op;

    branch_taken_ = false;
    const int rn = static_cast<int>(mep::raw(word, 4, 4));
    const int rm = static_cast<int>(mep::raw(word, 8, 4));
    auto& R = r;

    switch (insn.op) {
        // -------------------------------------------------------- loads/stores
        case Op::Sb: store8(R[static_cast<size_t>(rm)], static_cast<u8>(R[static_cast<size_t>(rn)])); break;
        case Op::Sh:
            store16(R[static_cast<size_t>(rm)] & 0xFFFFFFFEu,
                    static_cast<u16>(R[static_cast<size_t>(rn)]));
            break;
        case Op::Sw: store32(R[static_cast<size_t>(rm)] & 0xFFFFFFFCu, R[static_cast<size_t>(rn)]); break;
        case Op::Lb:
            R[static_cast<size_t>(rn)] =
                static_cast<u32>(static_cast<s32>(static_cast<s8>(load8(R[static_cast<size_t>(rm)]))));
            break;
        case Op::Lh:
            R[static_cast<size_t>(rn)] = static_cast<u32>(static_cast<s32>(
                static_cast<s16>(load16(R[static_cast<size_t>(rm)] & 0xFFFFFFFEu))));
            break;
        case Op::Lw:
            R[static_cast<size_t>(rn)] = load32(R[static_cast<size_t>(rm)] & 0xFFFFFFFCu);
            break;
        case Op::Lbu: R[static_cast<size_t>(rn)] = load8(R[static_cast<size_t>(rm)]); break;
        case Op::Lhu:
            R[static_cast<size_t>(rn)] = load16(R[static_cast<size_t>(rm)] & 0xFFFFFFFEu);
            break;

        case Op::SwSp: {
            const u32 d = mep::field_value(Field::F7u9a4, word, address);
            store32((R[15] + d) & 0xFFFFFFFCu, R[static_cast<size_t>(rn)]);
            break;
        }
        case Op::LwSp: {
            const u32 d = mep::field_value(Field::F7u9a4, word, address);
            R[static_cast<size_t>(rn)] = load32((R[15] + d) & 0xFFFFFFFCu);
            break;
        }
        case Op::SbTp: {
            const int n = static_cast<int>(mep::raw(word, 5, 3));
            const u32 d = mep::field_value(Field::F7u9, word, address);
            store8(R[13] + d, static_cast<u8>(R[static_cast<size_t>(n)]));
            break;
        }
        case Op::ShTp: {
            const int n = static_cast<int>(mep::raw(word, 5, 3));
            const u32 d = mep::field_value(Field::F7u9a2, word, address);
            store16((R[13] + d) & 0xFFFFFFFEu, static_cast<u16>(R[static_cast<size_t>(n)]));
            break;
        }
        case Op::SwTp: {
            const int n = static_cast<int>(mep::raw(word, 5, 3));
            const u32 d = mep::field_value(Field::F7u9a4, word, address);
            store32((R[13] + d) & 0xFFFFFFFCu, R[static_cast<size_t>(n)]);
            break;
        }
        case Op::LbTp: {
            const int n = static_cast<int>(mep::raw(word, 5, 3));
            const u32 d = mep::field_value(Field::F7u9, word, address);
            R[static_cast<size_t>(n)] = static_cast<u32>(static_cast<s32>(static_cast<s8>(load8(R[13] + d))));
            break;
        }
        case Op::LhTp: {
            const int n = static_cast<int>(mep::raw(word, 5, 3));
            const u32 d = mep::field_value(Field::F7u9a2, word, address);
            R[static_cast<size_t>(n)] = static_cast<u32>(
                static_cast<s32>(static_cast<s16>(load16((R[13] + d) & 0xFFFFFFFEu))));
            break;
        }
        case Op::LwTp: {
            const int n = static_cast<int>(mep::raw(word, 5, 3));
            const u32 d = mep::field_value(Field::F7u9a4, word, address);
            R[static_cast<size_t>(n)] = load32((R[13] + d) & 0xFFFFFFFCu);
            break;
        }
        case Op::LbuTp: {
            const int n = static_cast<int>(mep::raw(word, 5, 3));
            const u32 d = mep::field_value(Field::F7u9, word, address);
            R[static_cast<size_t>(n)] = load8(R[13] + d);
            break;
        }
        case Op::LhuTp: {
            const int n = static_cast<int>(mep::raw(word, 5, 3));
            const u32 d = mep::field_value(Field::F7u9a2, word, address);
            R[static_cast<size_t>(n)] = load16((R[13] + d) & 0xFFFFFFFEu);
            break;
        }

        case Op::Sb16:
        case Op::Sh16:
        case Op::Sw16:
        case Op::Lb16:
        case Op::Lh16:
        case Op::Lw16:
        case Op::Lbu16:
        case Op::Lhu16: {
            const u32 d = mep::field_value(Field::F16s16, word, address);
            const u32 target = R[static_cast<size_t>(rm)] + d;
            switch (insn.op) {
                case Op::Sb16: store8(target, static_cast<u8>(R[static_cast<size_t>(rn)])); break;
                case Op::Sh16:
                    store16(target & 0xFFFFFFFEu, static_cast<u16>(R[static_cast<size_t>(rn)]));
                    break;
                case Op::Sw16: store32(target & 0xFFFFFFFCu, R[static_cast<size_t>(rn)]); break;
                case Op::Lb16:
                    R[static_cast<size_t>(rn)] =
                        static_cast<u32>(static_cast<s32>(static_cast<s8>(load8(target))));
                    break;
                case Op::Lh16:
                    R[static_cast<size_t>(rn)] = static_cast<u32>(
                        static_cast<s32>(static_cast<s16>(load16(target & 0xFFFFFFFEu))));
                    break;
                case Op::Lw16: R[static_cast<size_t>(rn)] = load32(target & 0xFFFFFFFCu); break;
                case Op::Lbu16: R[static_cast<size_t>(rn)] = load8(target); break;
                default: R[static_cast<size_t>(rn)] = load16(target & 0xFFFFFFFEu); break;
            }
            break;
        }

        case Op::Sw24: {
            const u32 a = mep::field_value(Field::F24u8a4n, word, address);
            store32(a, R[static_cast<size_t>(rn)]);
            break;
        }
        case Op::Lw24: {
            const u32 a = mep::field_value(Field::F24u8a4n, word, address);
            R[static_cast<size_t>(rn)] = load32(a);
            break;
        }

        // ------------------------------------------------------ move/extend
        case Op::Extb: R[static_cast<size_t>(rn)] = static_cast<u32>(static_cast<s32>(static_cast<s8>(R[static_cast<size_t>(rn)]))); break;
        case Op::Exth: R[static_cast<size_t>(rn)] = static_cast<u32>(static_cast<s32>(static_cast<s16>(R[static_cast<size_t>(rn)]))); break;
        case Op::Extub: R[static_cast<size_t>(rn)] &= 0xFFu; break;
        case Op::Extuh: R[static_cast<size_t>(rn)] &= 0xFFFFu; break;

        case Op::Ssarb: {
            const u32 d = mep::field_value(Field::F2u6, word, address);
            sar = 32u - (((R[static_cast<size_t>(rm)] + d) & 3u) * 8u);
            break;
        }

        case Op::Mov: R[static_cast<size_t>(rn)] = R[static_cast<size_t>(rm)]; break;
        case Op::Movi8:
            R[static_cast<size_t>(rn)] = mep::sext(mep::raw(word, 8, 8), 8);
            break;
        case Op::Movi16:
            R[static_cast<size_t>(rn)] = mep::sext(mep::raw(word, 16, 16), 16);
            break;
        case Op::Movu24: {
            const int n = static_cast<int>(mep::raw(word, 5, 3));
            R[static_cast<size_t>(n)] = mep::field_value(Field::F24u8n, word, address);
            break;
        }
        case Op::Movu16: R[static_cast<size_t>(rn)] = mep::raw(word, 16, 16); break;
        case Op::Movh: R[static_cast<size_t>(rn)] = mep::raw(word, 16, 16) << 16; break;

        // -------------------------------------------------------- arithmetic
        case Op::Add3: {
            const int rl = static_cast<int>(mep::raw(word, 12, 4));
            R[static_cast<size_t>(rl)] = R[static_cast<size_t>(rn)] + R[static_cast<size_t>(rm)];
            break;
        }
        case Op::Add:
            R[static_cast<size_t>(rn)] += mep::sext(mep::raw(word, 8, 6), 6);
            break;
        case Op::Add3i: {
            const u32 d = mep::field_value(Field::F7u9a4, word, address);
            R[static_cast<size_t>(rn)] = R[15] + d;
            break;
        }
        case Op::Advck3:
            R[0] = signed_add_overflow(R[static_cast<size_t>(rn)], R[static_cast<size_t>(rm)]) ? 1u : 0u;
            break;
        case Op::Sub: R[static_cast<size_t>(rn)] -= R[static_cast<size_t>(rm)]; break;
        case Op::Sbvck3:
            R[0] = signed_sub_overflow(R[static_cast<size_t>(rn)], R[static_cast<size_t>(rm)]) ? 1u : 0u;
            break;
        case Op::Neg: R[static_cast<size_t>(rn)] = 0u - R[static_cast<size_t>(rm)]; break;
        case Op::Slt3:
            R[0] = (static_cast<s32>(R[static_cast<size_t>(rn)]) < static_cast<s32>(R[static_cast<size_t>(rm)])) ? 1u : 0u;
            break;
        case Op::Sltu3:
            R[0] = (R[static_cast<size_t>(rn)] < R[static_cast<size_t>(rm)]) ? 1u : 0u;
            break;
        case Op::Slt3i: {
            const u32 v = mep::raw(word, 8, 5);
            R[0] = (static_cast<s32>(R[static_cast<size_t>(rn)]) < static_cast<s32>(v)) ? 1u : 0u;
            break;
        }
        case Op::Sltu3i: {
            const u32 v = mep::raw(word, 8, 5);
            R[0] = (R[static_cast<size_t>(rn)] < v) ? 1u : 0u;
            break;
        }
        case Op::Sl1ad3:
            R[0] = (R[static_cast<size_t>(rn)] << 1) + R[static_cast<size_t>(rm)];
            break;
        case Op::Sl2ad3:
            R[0] = (R[static_cast<size_t>(rn)] << 2) + R[static_cast<size_t>(rm)];
            break;
        case Op::Add3x:
            R[static_cast<size_t>(rn)] = R[static_cast<size_t>(rm)] + mep::field_value(Field::F16s16, word, address);
            break;
        case Op::Slt3x:
            R[static_cast<size_t>(rn)] =
                (static_cast<s32>(R[static_cast<size_t>(rm)]) <
                 static_cast<s32>(mep::field_value(Field::F16s16, word, address)))
                    ? 1u
                    : 0u;
            break;
        case Op::Sltu3x: {
            const u32 v = mep::raw(word, 16, 16);
            R[static_cast<size_t>(rn)] = (R[static_cast<size_t>(rm)] < v) ? 1u : 0u;
            break;
        }

        // ----------------------------------------------------------- logical
        case Op::Or: R[static_cast<size_t>(rn)] |= R[static_cast<size_t>(rm)]; break;
        case Op::And: R[static_cast<size_t>(rn)] &= R[static_cast<size_t>(rm)]; break;
        case Op::Xor: R[static_cast<size_t>(rn)] ^= R[static_cast<size_t>(rm)]; break;
        case Op::Nor: R[static_cast<size_t>(rn)] = ~(R[static_cast<size_t>(rn)] | R[static_cast<size_t>(rm)]); break;
        case Op::Or3: R[static_cast<size_t>(rn)] = R[static_cast<size_t>(rm)] | mep::raw(word, 16, 16); break;
        case Op::And3: R[static_cast<size_t>(rn)] = R[static_cast<size_t>(rm)] & mep::raw(word, 16, 16); break;
        case Op::Xor3: R[static_cast<size_t>(rn)] = R[static_cast<size_t>(rm)] ^ mep::raw(word, 16, 16); break;

        // ------------------------------------------------------------ shifts
        case Op::Sra:
            R[static_cast<size_t>(rn)] = static_cast<u32>(
                static_cast<s32>(R[static_cast<size_t>(rn)]) >> (R[static_cast<size_t>(rm)] & 0x1Fu));
            break;
        case Op::Srl: R[static_cast<size_t>(rn)] >>= (R[static_cast<size_t>(rm)] & 0x1Fu); break;
        case Op::Sll: R[static_cast<size_t>(rn)] <<= (R[static_cast<size_t>(rm)] & 0x1Fu); break;
        case Op::Srai:
            R[static_cast<size_t>(rn)] =
                static_cast<u32>(static_cast<s32>(R[static_cast<size_t>(rn)]) >> mep::raw(word, 8, 5));
            break;
        case Op::Srli: R[static_cast<size_t>(rn)] >>= mep::raw(word, 8, 5); break;
        case Op::Slli: R[static_cast<size_t>(rn)] <<= mep::raw(word, 8, 5); break;
        case Op::Sll3: R[0] = R[static_cast<size_t>(rn)] << mep::raw(word, 8, 5); break;
        case Op::Fsft: {
            // Funnel shift: SAR selects how far the {rn:rm} pair is shifted.
            const unsigned sh = sar & 0x3Fu;
            const u64 joined = (static_cast<u64>(R[static_cast<size_t>(rn)]) << 32) |
                               R[static_cast<size_t>(rm)];
            const u64 shifted = sh >= 64 ? 0 : (joined << sh);
            R[static_cast<size_t>(rn)] = static_cast<u32>(shifted >> 32);
            break;
        }

        // ----------------------------------------------------------- branches
        case Op::Bra: branch_to(mep::field_value(Field::F12s4a2, word, address) & 0xFFFFFFFEu); break;
        case Op::Beqz:
            if (R[static_cast<size_t>(rn)] == 0) {
                branch_to(mep::field_value(Field::F8s8a2, word, address) & 0xFFFFFFFEu);
            }
            break;
        case Op::Bnez:
            if (R[static_cast<size_t>(rn)] != 0) {
                branch_to(mep::field_value(Field::F8s8a2, word, address) & 0xFFFFFFFEu);
            }
            break;
        // NOTE: the immediate-compare branches take their register from bits [7:4]
        // and the immediate from bits [11:8] *in this core's numbering*, which is
        // the same MSB-relative numbering the ISA table and the disassembler use
        // (`mep::raw` is `raw_impl`), so both agree: for word 0x0002E210 the
        // register is $2 and the immediate 0x1, exactly as printed.
        case Op::Beqi:
            if (R[static_cast<size_t>(rn)] == mep::raw(word, 8, 4)) {
                branch_to(mep::field_value(Field::F17s16a2, word, address) & 0xFFFFFFFEu);
            }
            break;
        case Op::Bnei:
            if (R[static_cast<size_t>(rn)] != mep::raw(word, 8, 4)) {
                branch_to(mep::field_value(Field::F17s16a2, word, address) & 0xFFFFFFFEu);
            }
            break;
        case Op::Blti:
            if (static_cast<s32>(R[static_cast<size_t>(rn)]) < static_cast<s32>(mep::raw(word, 8, 4))) {
                branch_to(mep::field_value(Field::F17s16a2, word, address) & 0xFFFFFFFEu);
            }
            break;
        case Op::Bgei:
            if (static_cast<s32>(R[static_cast<size_t>(rn)]) >= static_cast<s32>(mep::raw(word, 8, 4))) {
                branch_to(mep::field_value(Field::F17s16a2, word, address) & 0xFFFFFFFEu);
            }
            break;
        case Op::Beq:
            if (R[static_cast<size_t>(rn)] == R[static_cast<size_t>(rm)]) {
                branch_to(mep::field_value(Field::F17s16a2, word, address) & 0xFFFFFFFEu);
            }
            break;
        case Op::Bne:
            if (R[static_cast<size_t>(rn)] != R[static_cast<size_t>(rm)]) {
                branch_to(mep::field_value(Field::F17s16a2, word, address) & 0xFFFFFFFEu);
            }
            break;
        case Op::Bsr12:
            lp = (address + 2) | 1u;
            branch_to(mep::field_value(Field::F12s4a2, word, address) & 0xFFFFFFFEu);
            break;
        case Op::Bsr24:
            lp = (address + 4) | 1u;
            branch_to(mep::field_value(Field::F24s5a2n, word, address) & 0xFFFFFFFEu);
            break;
        case Op::Jmp: {
            const u32 target = R[static_cast<size_t>(rm)];
            // A target with bit 0 set toggles between core and VLIW mode, but
            // only the Venezia (IVC2) profile implements the second mode.
            branch_to(target & 0xFFFFFFFEu);
            break;
        }
        case Op::Jmp24:
            branch_to((address & 0xF0000000u) |
                      (mep::field_value(Field::F24u5a2n, word, address) & 0xFFFFFFFEu));
            break;
        case Op::Jsr:
            lp = (address + 2) | 1u;
            branch_to(R[static_cast<size_t>(rm)] & 0xFFFFFFFEu);
            break;
        case Op::Ret: branch_to(lp & 0xFFFFFFFEu); break;
        case Op::Jsrv:
            lp = (address + 2) | 1u;
            psw |= kPswOperatingMode;
            vliw_mode = true;
            branch_to(R[static_cast<size_t>(rm)] & 0xFFFFFFFCu);
            break;
        case Op::Bsrv:
            lp = (address + 4) | 1u;
            psw |= kPswOperatingMode;
            vliw_mode = true;
            branch_to(mep::field_value(Field::F24s5a2n, word, address) & 0xFFFFFFFCu);
            break;

        // -------------------------------------------------------------- repeat
        case Op::Repeat:
            rpb = address + 4;
            rpe = mep::field_value(Field::F17s16a2, word, address) & 0xFFFFFFFEu;
            rpc = R[static_cast<size_t>(rn)];
            rep_active_ = true;
            rep_pending_back_ = false;
            rep_endless_ = false;
            break;
        case Op::Erepeat:
            rpb = address + 4;
            rpe = (mep::field_value(Field::F17s16a2, word, address) & 0xFFFFFFFEu);
            rpc = 0;
            rep_active_ = true;
            rep_pending_back_ = false;
            rep_endless_ = true;
            break;

        // ------------------------------------------------------- control regs
        case Op::Stc:
        case Op::StcLp:
        case Op::StcHi:
        case Op::StcLo:
            set_csr(stc_ldc_csr(insn, word), R[static_cast<size_t>(rn)]);
            break;
        case Op::Ldc:
        case Op::LdcLp:
        case Op::LdcHi:
        case Op::LdcLo: {
            const int csr = stc_ldc_csr(insn, word);
            // Reading the PC through `ldc $rn,$pc` yields the address of the
            // following instruction.
            R[static_cast<size_t>(rn)] = (csr == 0) ? (address + 4) : get_csr(csr);
            break;
        }
        case Op::Di: psw &= ~kPswInterruptEnable; break;
        case Op::Ei: psw |= kPswInterruptEnable; break;
        case Op::Reti:
            if ((psw & kPswException) != 0) {
                set_pc(npc & 0xFFFFFFFEu);
                psw &= ~kPswException;
            } else {
                set_pc(epc & 0xFFFFFFFEu);
            }
            break;
        case Op::Halt:
        case Op::Sleep:
            psw |= kPswHalt;
            halt(insn.op == Op::Halt ? "halt instruction" : "sleep instruction");
            break;
        case Op::Swi: {
            const u32 level = mep::raw(word, 10, 2);
            exc |= (1u << (4 + static_cast<int>(level)));
            break;
        }
        case Op::Break:
            psw |= kPswHalt;
            halt("break exception");
            break;
        case Op::Syncm:
        case Op::Synccp:
        case Op::Dbreak: break;
        case Op::Dret: set_pc(depc & 0xFFFFFFFEu); break;

        // -------------------------------------------------------- control bus
        case Op::Stcb: {
            const u32 a = mep::raw(word, 16, 16);
            cop_access("stcb", 1, a, R[static_cast<size_t>(rn)], false);
            break;
        }
        case Op::Ldcb: {
            const u32 a = mep::raw(word, 16, 16);
            R[static_cast<size_t>(rn)] = cop_access("ldcb", 1, a, 0, true);
            break;
        }
        case Op::StcbR:
            cop_access("stcb", 1, R[static_cast<size_t>(rm)] & 0xFFFFu, R[static_cast<size_t>(rn)], false);
            break;
        case Op::LdcbR:
            R[static_cast<size_t>(rn)] = cop_access("ldcb", 1, R[static_cast<size_t>(rm)] & 0xFFFFu, 0, true);
            break;

        // ------------------------------------------------------------- bit ops
        case Op::Bsetm:
        case Op::Bclrm:
        case Op::Bnotm:
        case Op::Btstm: {
            const u32 bit = mep::raw(word, 5, 3);
            const u32 target = R[static_cast<size_t>(rm)];
            const u8 value = load8(target);
            if (insn.op == Op::Bsetm) {
                store8(target, static_cast<u8>(value | (1u << bit)));
            } else if (insn.op == Op::Bclrm) {
                store8(target, static_cast<u8>(value & ~(1u << bit)));
            } else if (insn.op == Op::Bnotm) {
                store8(target, static_cast<u8>(value ^ (1u << bit)));
            } else {
                R[0] = value & (1u << bit);
            }
            break;
        }
        case Op::Tas: {
            const u32 target = R[static_cast<size_t>(rm)];
            const u8 value = load8(target);
            store8(target, 1);
            R[static_cast<size_t>(rn)] = value;
            break;
        }
        case Op::Cache:
        case Op::Pref:
        case Op::Prefd:
            // No cache model on CMeP; the prefetch is an architectural no-op.
            cop_access("cache", 4, address, 0, false);
            break;

        // ---------------------------------------------------------- multiply
        case Op::Mul: {
            const u64 p = static_cast<u64>(static_cast<s64>(static_cast<s32>(R[static_cast<size_t>(rn)])) *
                                           static_cast<s64>(static_cast<s32>(R[static_cast<size_t>(rm)])));
            hi = static_cast<u32>(p >> 32);
            lo = static_cast<u32>(p);
            break;
        }
        case Op::Mulu: {
            const u64 p = static_cast<u64>(R[static_cast<size_t>(rn)]) * R[static_cast<size_t>(rm)];
            hi = static_cast<u32>(p >> 32);
            lo = static_cast<u32>(p);
            break;
        }
        case Op::Mulr: {
            const u64 p = static_cast<u64>(static_cast<s64>(static_cast<s32>(R[static_cast<size_t>(rn)])) *
                                           static_cast<s64>(static_cast<s32>(R[static_cast<size_t>(rm)])));
            hi = static_cast<u32>(p >> 32);
            lo = static_cast<u32>(p);
            R[static_cast<size_t>(rn)] = lo;
            break;
        }
        case Op::Mulru: {
            const u64 p = static_cast<u64>(R[static_cast<size_t>(rn)]) * R[static_cast<size_t>(rm)];
            hi = static_cast<u32>(p >> 32);
            lo = static_cast<u32>(p);
            R[static_cast<size_t>(rn)] = lo;
            break;
        }
        case Op::Madd:
        case Op::Maddu:
        case Op::Maddr:
        case Op::Maddru: {
            const bool is_signed = insn.op == Op::Madd || insn.op == Op::Maddr;
            u64 acc = (static_cast<u64>(hi) << 32) | lo;
            if (is_signed) {
                acc += static_cast<u64>(static_cast<s64>(static_cast<s32>(R[static_cast<size_t>(rn)])) *
                                        static_cast<s64>(static_cast<s32>(R[static_cast<size_t>(rm)])));
            } else {
                acc += static_cast<u64>(R[static_cast<size_t>(rn)]) * R[static_cast<size_t>(rm)];
            }
            hi = static_cast<u32>(acc >> 32);
            lo = static_cast<u32>(acc);
            if (insn.op == Op::Maddr || insn.op == Op::Maddru) R[static_cast<size_t>(rn)] = lo;
            break;
        }
        case Op::Div: {
            if (R[static_cast<size_t>(rm)] == 0) {
                halt("divide by zero");
                break;
            }
            if (R[static_cast<size_t>(rn)] == 0x80000000u && R[static_cast<size_t>(rm)] == 0xFFFFFFFFu) {
                lo = 0x80000000u;
                hi = 0;
                break;
            }
            lo = static_cast<u32>(static_cast<s32>(R[static_cast<size_t>(rn)]) /
                                  static_cast<s32>(R[static_cast<size_t>(rm)]));
            hi = static_cast<u32>(static_cast<s32>(R[static_cast<size_t>(rn)]) %
                                  static_cast<s32>(R[static_cast<size_t>(rm)]));
            break;
        }
        case Op::Divu: {
            if (R[static_cast<size_t>(rm)] == 0) {
                halt("divide by zero");
                break;
            }
            lo = R[static_cast<size_t>(rn)] / R[static_cast<size_t>(rm)];
            hi = R[static_cast<size_t>(rn)] % R[static_cast<size_t>(rm)];
            break;
        }

        // -------------------------------------------------------- dsp/misc
        case Op::Ldz: {
            u32 v = R[static_cast<size_t>(rm)];
            int n = 32;
            while (n > 0 && (v & 0x80000000u) == 0) {
                v <<= 1;
                --n;
            }
            R[static_cast<size_t>(rn)] = static_cast<u32>(n);
            break;
        }
        case Op::Abs: {
            const s32 d = static_cast<s32>(R[static_cast<size_t>(rn)]) - static_cast<s32>(R[static_cast<size_t>(rm)]);
            R[static_cast<size_t>(rn)] = static_cast<u32>(d < 0 ? -d : d);
            break;
        }
        case Op::Ave:
            R[static_cast<size_t>(rn)] = static_cast<u32>(
                (static_cast<s32>(R[static_cast<size_t>(rn)]) + static_cast<s32>(R[static_cast<size_t>(rm)]) + 1) >> 1);
            break;
        case Op::Min:
            if (static_cast<s32>(R[static_cast<size_t>(rn)]) > static_cast<s32>(R[static_cast<size_t>(rm)])) {
                R[static_cast<size_t>(rn)] = R[static_cast<size_t>(rm)];
            }
            break;
        case Op::Max:
            if (static_cast<s32>(R[static_cast<size_t>(rn)]) < static_cast<s32>(R[static_cast<size_t>(rm)])) {
                R[static_cast<size_t>(rn)] = R[static_cast<size_t>(rm)];
            }
            break;
        case Op::Minu:
            if (R[static_cast<size_t>(rn)] > R[static_cast<size_t>(rm)]) {
                R[static_cast<size_t>(rn)] = R[static_cast<size_t>(rm)];
            }
            break;
        case Op::Maxu:
            if (R[static_cast<size_t>(rn)] < R[static_cast<size_t>(rm)]) {
                R[static_cast<size_t>(rn)] = R[static_cast<size_t>(rm)];
            }
            break;
        case Op::Clip: {
            const u32 n = mep::raw(word, 24, 5);
            if (n == 0) {
                R[static_cast<size_t>(rn)] = 0;
                break;
            }
            const s32 max = static_cast<s32>((1u << (n - 1)) - 1u);
            const s32 min = -static_cast<s32>(1u << (n - 1));
            s32 v = static_cast<s32>(R[static_cast<size_t>(rn)]);
            if (v > max) v = max;
            else if (v < min) v = min;
            R[static_cast<size_t>(rn)] = static_cast<u32>(v);
            break;
        }
        case Op::Clipu: {
            const u32 n = mep::raw(word, 24, 5);
            if (n == 0) {
                R[static_cast<size_t>(rn)] = 0;
                break;
            }
            const u32 max = (n >= 32) ? 0xFFFFFFFFu : ((1u << n) - 1u);
            if (R[static_cast<size_t>(rn)] > max) R[static_cast<size_t>(rn)] = max;
            break;
        }
        case Op::Sadd: {
            const s64 s = static_cast<s64>(static_cast<s32>(R[static_cast<size_t>(rn)])) +
                          static_cast<s32>(R[static_cast<size_t>(rm)]);
            if (s > 2147483647LL) R[static_cast<size_t>(rn)] = 0x7FFFFFFFu;
            else if (s < -2147483648LL) R[static_cast<size_t>(rn)] = 0x80000000u;
            else R[static_cast<size_t>(rn)] = static_cast<u32>(static_cast<s32>(s));
            break;
        }
        case Op::Ssub: {
            const s64 s = static_cast<s64>(static_cast<s32>(R[static_cast<size_t>(rn)])) -
                          static_cast<s32>(R[static_cast<size_t>(rm)]);
            if (s > 2147483647LL) R[static_cast<size_t>(rn)] = 0x7FFFFFFFu;
            else if (s < -2147483648LL) R[static_cast<size_t>(rn)] = 0x80000000u;
            else R[static_cast<size_t>(rn)] = static_cast<u32>(static_cast<s32>(s));
            break;
        }
        case Op::Saddu: {
            const u64 s = static_cast<u64>(R[static_cast<size_t>(rn)]) + R[static_cast<size_t>(rm)];
            R[static_cast<size_t>(rn)] = (s > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<u32>(s);
            break;
        }
        case Op::Ssubu:
            R[static_cast<size_t>(rn)] = (R[static_cast<size_t>(rn)] < R[static_cast<size_t>(rm)])
                                             ? 0u
                                             : R[static_cast<size_t>(rn)] - R[static_cast<size_t>(rm)];
            break;

        // -------------------------------------------------------- conditional
        case Op::Bcpeq:
        case Op::Bcpne:
        case Op::Bcpat:
        case Op::Bcpaf: {
            const u32 mask = mep::raw(word, 8, 4);
            bool take = false;
            if (insn.op == Op::Bcpeq) take = ((mask ^ cr0) == 0);
            else if (insn.op == Op::Bcpne) take = ((mask ^ cr0) != 0);
            else if (insn.op == Op::Bcpat) take = ((mask & cr0) != 0);
            else take = ((mask & cr0) == 0);
            if (take) set_pc(mep::field_value(Field::F17s16a2, word, address) & 0xFFFFFFFEu);
            break;
        }

        case Op::SimSyscall:
            ZLB_LOG_INFO("cpu", "[CMeP] simulator syscall 0x%x at 0x%08X", mep::raw(word, 5, 4), address);
            break;

        // -------------------------------------------------------- cp word ops
        case Op::Swcp:
        case Op::Lwcp:
        case Op::Swcpi:
        case Op::Lwcpi:
        case Op::Swcp16:
        case Op::Lwcp16:
        case Op::Sbcpa:
        case Op::Lbcpa:
        case Op::Shcpa:
        case Op::Lhcpa:
        case Op::Swcpa:
        case Op::Lwcpa:
        case Op::Sbcpm0:
        case Op::Lbcpm0:
        case Op::Shcpm0:
        case Op::Lhcpm0:
        case Op::Swcpm0:
        case Op::Lwcpm0:
        case Op::Sbcpm1:
        case Op::Lbcpm1:
        case Op::Shcpm1:
        case Op::Lhcpm1:
        case Op::Swcpm1:
        case Op::Lwcpm1:
        case Op::Sbcp:
        case Op::Lbcp:
        case Op::Lbucp:
        case Op::Shcp:
        case Op::Lhcp:
        case Op::Lhucp:
        case Op::Lbucpa:
        case Op::Lhucpa:
        case Op::Lbucpm0:
        case Op::Lhucpm0:
        case Op::Lbucpm1:
        case Op::Lhucpm1:
            cop_word(insn, word, address);
            break;

        // ------------------------------------------------------ cp 64 bit ops
        case Op::Smcp:
        case Op::Lmcp:
        case Op::Smcpi:
        case Op::Lmcpi:
        case Op::Smcp16:
        case Op::Lmcp16:
        case Op::Smcpa:
        case Op::Lmcpa:
        case Op::Smcpm0:
        case Op::Lmcpm0:
        case Op::Smcpm1:
        case Op::Lmcpm1:
            cop_word64(insn, word, address);
            break;

        // -------------------------------------------------------- compare/sw
        case Op::Casb3:
        case Op::Cash3:
        case Op::Casw3: {
            const int rl = static_cast<int>(mep::field_value(Field::FRl5, word, address));
            const u32 target = R[static_cast<size_t>(rm)];
            if (insn.op == Op::Casb3) {
                const u8 old = load8(target);
                if (old == static_cast<u8>(R[static_cast<size_t>(rn)] & 0xFF)) {
                    store8(target, static_cast<u8>(R[static_cast<size_t>(rl)] & 0xFF));
                } else {
                    R[static_cast<size_t>(rl)] = old;
                }
            } else if (insn.op == Op::Cash3) {
                const u16 old = load16(target & 0xFFFFFFFEu);
                if (old == static_cast<u16>(R[static_cast<size_t>(rn)] & 0xFFFF)) {
                    store16(target & 0xFFFFFFFEu, static_cast<u16>(R[static_cast<size_t>(rl)] & 0xFFFF));
                } else {
                    R[static_cast<size_t>(rl)] = old;
                }
            } else {
                const u32 old = load32(target & 0xFFFFFFFCu);
                if (old == R[static_cast<size_t>(rn)]) {
                    store32(target & 0xFFFFFFFCu, R[static_cast<size_t>(rl)]);
                } else {
                    R[static_cast<size_t>(rl)] = old;
                }
            }
            break;
        }

        // ------------------------------------------- IVC2 / DSP escape opcodes
        case Op::Uci:
        case Op::Dsp:
        case Op::Dsp0:
        case Op::Dsp1:
            // These escape to the IVC2 DSP coprocessor, which only exists on
            // the Venezia profile; on the CMeP they are undefined.
            mark_undefined(format("%s (IVC2/DSP coprocessor)", insn.mnem), address);
            break;

        default:
            mark_undefined(insn.mnem, address);
            break;
    }
}

// ---------------------------------------------------------------------------
// Debugger interface
// ---------------------------------------------------------------------------

u32 MePCore::cond_packed() const {
    u32 value = 0;
    for (size_t i = 0; i < cond.size(); ++i) {
        if (cond[i]) value |= (1u << i);
    }
    return value;
}

std::string MePCore::disassemble(u32 address, unsigned& length) {
    set_context(address);
    const u32 word = bus->fetch16(address) | (static_cast<u32>(bus->fetch16(address + 2)) << 16);
    const mep::Insn* insn = mep::decode(word);
    length = insn->len;
    return mep::format(*insn, word, address);
}

void MePCore::registers(std::vector<RegValue>& out) const {
    for (int i = 0; i < 16; ++i) {
        out.emplace_back("GPR", mep::register_name(i), r[static_cast<size_t>(i)]);
    }
    out.emplace_back("Core", "$pc", pc);
    out.emplace_back("Core", "$hi", hi);
    out.emplace_back("Core", "$lo", lo);
    out.emplace_back("Core", "$sar", sar);
    out.emplace_back("Core", "$lp", lp);
    out.emplace_back("Core", "$epc", epc);
    out.emplace_back("Core", "$npc", npc);
    out.emplace_back("Core", "$rpb", rpb);
    out.emplace_back("Core", "$rpe", rpe);
    out.emplace_back("Core", "$rpc", rpc);
    out.emplace_back("Core", "$tmp", tmp);
    out.emplace_back("Core", "$psw", psw);
    out.emplace_back("Core", "$cond", cond_packed());
    out.emplace_back("Core", "$mb0", mb0);
    out.emplace_back("Core", "$me0", me0);
    out.emplace_back("Core", "$mb1", mb1);
    out.emplace_back("Core", "$me1", me1);
    out.emplace_back("Core", "$exc", exc);
    out.emplace_back("Core", "$opt", opt);
}

std::string MePCore::status_line() const {
    std::string line = format("pc=0x%08X psw=0x%08X cond=0x%02X", pc, psw, cond_packed());
    if (vliw_mode) line += " vliw";
    if (halted) {
        line += " HALTED(";
        line += halt_reason;
        line += ')';
    }
    if (undefined_instruction) line += " UNDEFINED";
    return line;
}

void MePCore::describe_state(std::vector<std::string>& lines) const {
    lines.push_back(format("rpb=0x%08X rpe=0x%08X rpc=%u %s", rpb, rpe, rpc,
                           rep_active_ ? (rep_endless_ ? "(endless)" : "(active)") : "(idle)"));
    lines.push_back(format("mb0=0x%08X me0=0x%08X mb1=0x%08X me1=0x%08X", mb0, me0, mb1, me1));
    lines.push_back(format("exc=0x%08X cfg=0x%08X vid=0x%08X id=0x%08X", exc, cfg, vid, id));
    lines.push_back(format("cbus[0x400..0x401]=%u timer=%s%s remaining=%u", cbus.count,
                           cbus.busy() ? "running" : "idle", cbus.done ? " done" : "",
                           cbus.remaining));
}

namespace {

/// Register index for the names the debugger and tests use ("$sp", "sp", "15").
int gpr_index(const std::string& name) {
    if (name == "sp") return 15;
    if (name == "tp") return 13;
    if (name == "gp") return 14;
    if (name == "fp") return 8;
    if (name.empty() || name.size() > 2) return -1;
    for (char c : name) {
        if (!std::isdigit(static_cast<unsigned char>(c))) return -1;
    }
    const int value = std::atoi(name.c_str());
    return (value >= 0 && value <= 15) ? value : -1;
}

}  // namespace

bool MePCore::set_register(const std::string& name, u64 value) {
    std::string key = to_lower(trim(name));
    if (!key.empty() && key[0] == '$') key.erase(0, 1);

    const int index = gpr_index(key);
    if (index >= 0) {
        r[static_cast<size_t>(index)] = static_cast<u32>(value);
        return true;
    }
    if (key == "pc") { set_pc(static_cast<u32>(value)); return true; }
    if (key == "hi") { hi = static_cast<u32>(value); return true; }
    if (key == "lo") { lo = static_cast<u32>(value); return true; }
    if (key == "sar") { sar = static_cast<u32>(value); return true; }
    if (key == "lp") { lp = static_cast<u32>(value); return true; }
    if (key == "epc") { epc = static_cast<u32>(value); return true; }
    if (key == "npc") { npc = static_cast<u32>(value); return true; }
    if (key == "rpb") { rpb = static_cast<u32>(value); return true; }
    if (key == "rpe") { rpe = static_cast<u32>(value); return true; }
    if (key == "rpc") { rpc = static_cast<u32>(value); return true; }
    if (key == "tmp") { tmp = static_cast<u32>(value); return true; }
    if (key == "psw") {
        psw = static_cast<u32>(value);
        vliw_mode = (psw & kPswOperatingMode) != 0;
        return true;
    }
    if (key == "cond") {
        for (size_t i = 0; i < cond.size(); ++i) cond[i] = ((value >> i) & 1u) != 0;
        return true;
    }
    return false;
}

bool MePCore::get_register(const std::string& name, u64& value) const {
    std::string key = to_lower(trim(name));
    if (!key.empty() && key[0] == '$') key.erase(0, 1);

    const int index = gpr_index(key);
    if (index >= 0) {
        value = r[static_cast<size_t>(index)];
        return true;
    }
    if (key == "pc") { value = pc; return true; }
    if (key == "hi") { value = hi; return true; }
    if (key == "lo") { value = lo; return true; }
    if (key == "sar") { value = sar; return true; }
    if (key == "lp") { value = lp; return true; }
    if (key == "epc") { value = epc; return true; }
    if (key == "npc") { value = npc; return true; }
    if (key == "rpb") { value = rpb; return true; }
    if (key == "rpe") { value = rpe; return true; }
    if (key == "rpc") { value = rpc; return true; }
    if (key == "tmp") { value = tmp; return true; }
    if (key == "psw") { value = psw; return true; }
    if (key == "cond") { value = cond_packed(); return true; }
    return false;
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

std::unique_ptr<Cpu> create_mep_core(Bus& bus) { return std::make_unique<MePCore>(bus); }

}  // namespace zlb
