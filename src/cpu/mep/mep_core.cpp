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
#include "event/providers.h"

namespace zlb {
namespace {

constexpr u32 kPswInterruptEnable = 1u << 0;
constexpr u32 kPswNmi = 1u << 9;
constexpr u32 kPswHardwareInterruptEnable = 1u << 8;
constexpr u32 kExcHardwarePending = 1u << 8;
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

/// Number of leading zero bits: the `ldz` result and the bit count the modulo
/// addressing unit derives its mask from.  The reference description computes
/// that mask as `(srl (const SI -1) (do_ldz (or mb me)))`, which is the smallest
/// mask covering both MB and ME - e.g. MB = 0x50000, ME = 0x5003f gives
/// `0xFFFFFFFF >> 13 == 0x7ffff`.  Counting *significant* bits instead (the
/// value the core used to return) produces 0x1fff there and the wrap can never
/// trigger, so `do_ldz` must be a leading-zero count; the instruction's own
/// comment in cpu/mep-core.cpu is "leading zeroes".
int leading_zeros(u32 value) {
    int n = 0;
    while (n < 32 && (value & 0x80000000u) == 0) {
        value <<= 1;
        ++n;
    }
    return n;
}

/// CGEN `mod0`/`mod1`: post-modify the base register of a coprocessor
/// addressing-unit transfer, wrapping at the end address of the modulo window.
u32 modulo_update(u32 base, u32 mb, u32 me, u32 displacement) {
    const u32 range = mb | me;
    const u32 mask = (range == 0) ? 0u : (0xFFFFFFFFu >> leading_zeros(range));
    if ((base & mask) == (me & mask)) return (base & ~mask) | mb;
    return base + displacement;
}

}  // namespace

// ---------------------------------------------------------------------------
// Control bus model
// ---------------------------------------------------------------------------

void MePCore::ControlBus::reset() {
    regs.fill(0);
    irq_levels = irq_edges = 0;
    count = 0;
    remaining = 0;
    running = false;
    done = false;
    force_expired = false;
}

bool MePCore::ControlBus::busy() const { return running && !force_expired; }

u32 MePCore::ControlBus::read(unsigned address) const {
    if (address >= regs.size()) return 0;
    // The secure module kprx_auth_sm configures control registers 0x410..0x415 and
    // then polls 0x412 for completion. Logging that window shows the whole protocol:
    // who writes what and what is read back.
    {
        static const bool cb_log = [] {
            const char* v = std::getenv("ZLB_CB_LOG");
            return v != nullptr && v[0] != '0';
        }();
        if (cb_log && address >= 0x410u && address < 0x420u) {
            static u32 n = 0;
            if (n < 60u) {
                ++n;
                ZLB_LOG_INFO("mep", "cb read  0x%03X -> 0x%X", address, regs[address]);
            }
        }
    }
    if (address == 1) return (irq_levels & ~regs[3]) | (irq_edges & regs[3]);
    u32 value = regs[address];
    if (address == kCbTimerStatus) {
        // Bit 0 is the "the count reached zero" latch, *not* a busy flag.  Both
        // poll loops in the boot chain wait for it to become 1:
        //   first loader  0x5E686: ldcb / and $0,$2 / beqz  -> loop while bit0 == 0
        //   second loader 0x45516: ldcb / and3 $3,$3,1 / bnez -> escape when bit0 != 0
        // Reading the status never changes it; software clears it by writing 0.
        value = (value & ~1u) | ((done || force_expired) ? 1u : 0u);
    }
    return value;
}

void MePCore::ControlBus::write(unsigned address, u32 value) {
    if (address >= regs.size()) return;
    {
        static const bool cb_log = [] {
            const char* v = std::getenv("ZLB_CB_LOG");
            return v != nullptr && v[0] != '0';
        }();
        if (cb_log && address >= 0x410u && address < 0x420u) {
            static u32 n = 0;
            if (n < 80u) {
                ++n;
                ZLB_LOG_INFO("mep", "cb write 0x%03X = 0x%X", address, value);
            }
        }
    }
    if (address == 0) {
        // IVR.ICN and ILV describe the last vector fetch; only IML is writable.
        regs[0] = (regs[0] & ~0xF00u) | (value & 0xF00u);
        return;
    }
    if (address == 1) {
        // ISR is write-zero-to-clear for edge inputs and read-only for levels.
        irq_edges &= value;
        return;
    }
    regs[address] = value;
    if (address == kCbTimerCount || address == kCbTimerCount + 1) {
        // The reload value is 16 bit at 0x400..0x401, big endian: the first
        // loader writes the low byte to 0x401 (0x5E67A) while the second loader
        // writes the high byte to 0x400 (0x45502).
        count = ((regs[kCbTimerCount] & 0xFFu) << 8) | (regs[kCbTimerCount + 1] & 0xFFu);
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

void MePCore::ControlBus::set_irq_level(unsigned source, bool asserted) {
    if (source >= 32) return;
    const u32 bit = 1u << source;
    if (asserted) {
        if ((irq_levels & bit) == 0 && (regs[3] & bit) != 0) irq_edges |= bit;
        irq_levels |= bit;
    } else {
        irq_levels &= ~bit;
    }
}

int MePCore::ControlBus::pending_irq() const {
    const u32 pending = read(1) & regs[2];
    unsigned highest = (regs[0] >> 8) & 0xFu;
    int source = -1;
    for (unsigned channel = 0; channel < 32; ++channel) {
        if ((pending & (1u << channel)) == 0) continue;
        const unsigned level = (regs[4 + channel / 8] >> ((channel % 8) * 4)) & 0xFu;
        if (level > highest || (level == highest && source >= 0)) {
            highest = level;
            source = static_cast<int>(channel);
        }
    }
    return source;
}

void MePCore::ControlBus::acknowledge_irq(unsigned source) {
    const u32 level = (regs[4 + source / 8] >> ((source % 8) * 4)) & 0xFu;
    regs[0] = (regs[0] & 0xF00u) | (level << 12) | (source << 3);
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
    cfg = 1u << 3; // CFG.IVM resets to separate vectors (MeP architecture p42).
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

    events().event(EventProvider::Cpu, ev::cpu::kCoreReset)
        .field("core", static_cast<u64>(0))
        .field("name", std::string(core_name()))
        .address("entry", entry)
        .emit();
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
            rep_endless_ = (rpe & 1u) != 0;
            rep_active_ = rpb != 0 && (rep_endless_ || rpc != 0);
            break;
        case 5:
            rpe = value;
            rep_endless_ = (rpe & 1u) != 0;
            rep_active_ = rpb != 0 && (rep_endless_ || rpc != 0);
            break;
        case 6:
            rpc = value;
            rep_active_ = rpb != 0 && ((rpe & 1u) != 0 || rpc != 0);
            break;
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
    // The coprocessor registers are 64 bit wide, but the model only holds the
    // low 32 bits (the `$cN` file is the GPR file), so an eight byte store is
    // the register zero extended.  Shifting the 32 bit `value` by 32 or more
    // would be undefined and on x86/ARM64 repeats the low word.
    for (unsigned i = 0; i < width; ++i) {
        const u8 byte = (i < 4) ? static_cast<u8>((value >> (8 * i)) & 0xFFu) : 0u;
        cbus.write(address + i, byte);
    }
    return value;
}

void MePCore::cop_word(const mep::Insn& insn, u32 word, u32 address) {
    const int crn = static_cast<int>(mep::raw(word, 4, 4));
    const int rm = static_cast<int>(mep::raw(word, 8, 4));
    const u32 base = r[static_cast<size_t>(rm)];

    u32 disp = 0;
    u32 update = 0;
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
        default:
            // The addressing-unit forms (`sbcpa`, `shcpm0`, ...) transfer
            // through $rm itself - the reference body is
            // `(set (mem QI rma) (and crn #xff))` followed by
            // `(set rma (add rma (ext SI cdisp10)))` - so cdisp10 is only the
            // post-modify amount and must not be added to the address.
            update = mep::field_value(mep::Field::FCdisp10, word, address);
            break;
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

    // The a/m forms are the addressing unit variants.  Their base register is
    // post-modified by the `cdisp10` operand (`update`), not by the access size:
    // the reference description for the plain forms is
    // `(set rma (add rma (ext SI cdisp10)))` (cpu/mep-core.cpu sbcpa/shcpa/...)
    // and the m0/m1 siblings replace that add with `(mod0 cdisp10)` /
    // `(mod1 cdisp10)`.  The plain post-increment forms (`swcpi`) do add the
    // access size.
    switch (insn.op) {
        case mep::Op::Swcpi:
        case mep::Op::Lwcpi:
            r[static_cast<size_t>(rm)] = base + 4;
            break;
        case mep::Op::Swcpa:
        case mep::Op::Lwcpa:
        case mep::Op::Sbcpa:
        case mep::Op::Lbcpa:
        case mep::Op::Shcpa:
        case mep::Op::Lhcpa:
        case mep::Op::Lbucpa:
        case mep::Op::Lhucpa:
            r[static_cast<size_t>(rm)] = base + update;
            break;
        case mep::Op::Swcpm0:
        case mep::Op::Lwcpm0:
        case mep::Op::Sbcpm0:
        case mep::Op::Lbcpm0:
        case mep::Op::Shcpm0:
        case mep::Op::Lhcpm0:
        case mep::Op::Lbucpm0:
        case mep::Op::Lhucpm0:
            r[static_cast<size_t>(rm)] = modulo_update(base, mb0, me0, update);
            break;
        case mep::Op::Swcpm1:
        case mep::Op::Lwcpm1:
        case mep::Op::Sbcpm1:
        case mep::Op::Lbcpm1:
        case mep::Op::Shcpm1:
        case mep::Op::Lhcpm1:
        case mep::Op::Lbucpm1:
        case mep::Op::Lhucpm1:
            r[static_cast<size_t>(rm)] = modulo_update(base, mb1, me1, update);
            break;
        default: break;
    }
}

void MePCore::cop_word64(const mep::Insn& insn, u32 word, u32 address) {
    const int crn = static_cast<int>(mep::raw(word, 4, 4));
    const int rm = static_cast<int>(mep::raw(word, 8, 4));
    const bool load = insn.mnem[0] == 'l';

    u32 disp = 0;
    u32 update = 0;
    if (insn.op == mep::Op::Smcp16 || insn.op == mep::Op::Lmcp16) {
        disp = mep::field_value(mep::Field::F16s16, word, address);
    } else if (insn.op != mep::Op::Smcp && insn.op != mep::Op::Lmcp &&
               insn.op != mep::Op::Smcpi && insn.op != mep::Op::Lmcpi) {
        // The `a`/`m` forms transfer through $rm and use cdisp10a8 as the
        // post-modify amount only (see the comment in cop_word).
        update = mep::field_value(mep::Field::FCdisp10, word, address);
    }
    const u32 target = (r[static_cast<size_t>(rm)] + disp) & 0xFFFFFFF8u;
    if (!load) cop_access("smcp", 8, target, r[static_cast<size_t>(crn)], false);

    switch (insn.op) {
        case mep::Op::Smcpi:
        case mep::Op::Lmcpi:
            r[static_cast<size_t>(rm)] = r[static_cast<size_t>(rm)] + 8;
            break;
        case mep::Op::Smcpa:
        case mep::Op::Lmcpa:
            r[static_cast<size_t>(rm)] = r[static_cast<size_t>(rm)] + update;
            break;
        case mep::Op::Smcpm0:
        case mep::Op::Lmcpm0:
            r[static_cast<size_t>(rm)] =
                modulo_update(r[static_cast<size_t>(rm)], mb0, me0, update);
            break;
        case mep::Op::Smcpm1:
        case mep::Op::Lmcpm1:
            r[static_cast<size_t>(rm)] =
                modulo_update(r[static_cast<size_t>(rm)], mb1, me1, update);
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
    // The secure kernel keeps its state in a global written through 0x801d54
    // ("sw $1,-32748($gp)"). The idle loop waits for the value 9. Trace every call to
    // that setter with its value and return address: this shows whether 9 is ever set
    // and, if so, from where.
    {
        static const bool st_log = [] {
            const char* v = std::getenv("ZLB_STATE_LOG");
            return v != nullptr && v[0] != '0';
        }();
        // 0x800A56 is "lw $5,($3)" with $3 = 0xE0000010 (the ARM->CMeP mailbox), so $5 on
        // the next instruction is the command the ARM sent. Logging it shows which
        // commands actually arrive - in particular whether 0x101 (the one that sets
        // state 9) ever does.
        if (st_log && (pc & ~1u) == 0x800A58u) {
            static u32 m = 0;
            if (m < 60u) {
                ++m;
                ZLB_LOG_INFO("mep", "mailbox cmd: $5=0x%X (%u)", r[5], r[5]);
            }
        }
        if (st_log && (pc & ~1u) == 0x801D54u) {
            static u32 n = 0;
            if (n < 80u) {
                ++n;
                ZLB_LOG_INFO("mep", "state set: value=%u (0x%X) lr=0x%X pc=0x%X",
                             r[1], r[1], lp, pc);
            }
        }
    }
    StepResult out;
    const u32 address = pc;
    out.address = address;
    refresh_irq_line();
    // Architecture 3.6.6/7.13: SWI or STC requests a pending interrupt, which
    // enters before the next instruction once IEC and its SIE bit are set.
    // Table 17 gives software interrupts priority over hardware interrupts.
    // Keep SIP pending until the native handler clears it with STC EXC.
    if ((psw & kPswInterruptEnable) != 0 && (psw & kPswNmi) == 0 &&
        (exc & psw & 0xF0u) != 0) {
        u32 vector_base = boot_vector_base_;
        if ((cfg & (1u << 4)) != 0) {
            vector_base = (cfg & (1u << 23)) != 0 ? 0x00800000u : 0x00200000u;
        }
        epc = (pc & ~1u) | (vliw_mode ? 1u : 0u);
        const u32 previous = psw;
        psw = (previous & ~0x100Fu) | ((previous & 5u) << 1);
        vliw_mode = false;
        exc = (exc & ~0xFu) | 5u;
        // RPB/RPE/RPC remain available for the handler's native context save.
        // SWI in either of the last two repeat slots is prohibited by 7.13.
        rep_pending_back_ = false;
        set_pc(vector_base + 0x14u);
        out.was_branch = true;
        out.text = "software interrupt";
        ++cycles;
        return out;
    }
    if (take_pending_irq()) {
        out.was_branch = true;
        out.text = "hardware interrupt";
        ++cycles;
        return out;
    }

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
    // See Cpu::step_text: formatting the listing for every instruction is pure
    // per-instruction overhead when nothing reads StepResult::text.
    if (step_text) out.text = mep::format(*insn, word, address);

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
    refresh_irq_line();

    out.was_branch = pc != address + insn->len;
    return out;
}

u32 MePCore::interrupt_flag_register() const { return cbus.read(1); }
u32 MePCore::interrupt_mask_register() const { return cbus.read(2); }

void MePCore::set_irq_level(unsigned source, bool asserted) {
    cbus.set_irq_level(source, asserted);
    refresh_irq_line();
}

void MePCore::refresh_irq_line() {
    const bool requested = cbus.pending_irq() >= 0;
    exc = requested ? exc | kExcHardwarePending : exc & ~kExcHardwarePending;
    // The INTC wakes HALT/SLEEP regardless of the core's IEC/HIE masks. A
    // debugger or machine-hook stop is not an architectural sleep state.
    if (requested && halted &&
        (halt_reason == "sleep instruction" || halt_reason == "halt instruction")) {
        psw &= ~kPswHalt;
        halted = false;
        halt_reason.clear();
    }
}

bool MePCore::take_pending_irq() {
    refresh_irq_line();
    const int source = cbus.pending_irq();
    if (source >= 0) {
        ++irq_sources_seen;
        irq_last_source = source;
    }
    if (source < 0 || (psw & (kPswInterruptEnable | kPswHardwareInterruptEnable)) !=
                          (kPswInterruptEnable | kPswHardwareInterruptEnable) ||
        (psw & kPswNmi) != 0) return false;
    // The final repeat slot cannot be interrupted while another iteration
    // remains (Architecture 7.12). Its branch-back retires before IRQ entry.
    if (rep_active_ && rep_pending_back_ && (rep_endless_ || rpc != 0)) return false;

    // MeP Core Architecture 3.6.2/3.6.5: CFG controls ROM/RAM vector banks,
    // independently of the image entry address used by the loader.
    u32 vector = boot_vector_base_ + 0x30u;
    if ((cfg & (1u << 4)) != 0) {
        const bool eva = (cfg & (1u << 23)) != 0;
        const bool iva = (cfg & (1u << 22)) != 0;
        vector = iva ? 0x00800000u : 0x00200000u;
        if (eva == iva) vector += 0x30u;
    }
    if ((cfg & (1u << 3)) != 0) vector += static_cast<u32>(source) * 4u;
    epc = (pc & ~1u) | (vliw_mode ? 1u : 0u);
    const u32 previous = psw;
    psw = (previous & ~0x100Fu) | ((previous & 5u) << 1);
    vliw_mode = false;
    exc &= ~0xFu; // EXC.EXC=0 identifies a hardware interrupt.
    cbus.acknowledge_irq(static_cast<unsigned>(source));
    // The loop registers are preserved, and a handler can save/restore them
    // with STC/LDC. No pending trailing slot remains at an interrupt boundary.
    rep_pending_back_ = false;
    ++irq_sources_taken;
    set_pc(vector);

    events().event(EventProvider::Interrupt, ev::interrupt::kDeliver)
        .field("core", static_cast<u64>(0))
        .field("line", static_cast<u64>(source))
        .address("vector", vector)
        .field("name", std::string(core_name()))
        .emit();
    return true;
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
            rpe = (mep::field_value(Field::F17s16a2, word, address) & 0xFFFFFFFEu) | 1u;
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
            // following instruction.  The reference description is
            // `(set-vliw-modified-pcrel-offset rn 2 4 8)`, i.e. `rn = pc + 2` in
            // core operating mode (only the Venezia VLIW modes add 4 / 8), and
            // `ldc` is a 16 bit instruction - this used to add 4 unconditionally.
            R[static_cast<size_t>(rn)] = (csr == 0) ? (address + 2) : get_csr(csr);
            break;
        }
        case Op::Di: psw &= ~kPswInterruptEnable; break;
        case Op::Ei: psw |= kPswInterruptEnable; break;
        case Op::Reti:
            if ((psw & kPswNmi) != 0) {
                set_pc(npc & 0xFFFFFFFEu);
                psw &= ~kPswNmi;
            } else {
                set_pc(epc & 0xFFFFFFFEu);
                // IEC<-IEP and UMC<-UMP; previous bits remain unchanged.
                psw = (psw & ~5u) | ((psw >> 1) & 5u);
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
            cbus.write(a, R[static_cast<size_t>(rn)]);
            break;
        }
        case Op::Ldcb: {
            const u32 a = mep::raw(word, 16, 16);
            R[static_cast<size_t>(rn)] = cbus.read(a);
            break;
        }
        case Op::StcbR:
            cbus.write(R[static_cast<size_t>(rm)] & 0xFFFFu, R[static_cast<size_t>(rn)]);
            break;
        case Op::LdcbR:
            R[static_cast<size_t>(rn)] = cbus.read(R[static_cast<size_t>(rm)] & 0xFFFFu);
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
            // "leading zeroes": the count of zero bits above the most
            // significant one (`ldz $rn,0` is 32), not the number of
            // significant bits.
            R[static_cast<size_t>(rn)] = static_cast<u32>(leading_zeros(R[static_cast<size_t>(rm)]));
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
            // The reference description clamps with a *signed* upper compare and
            // then clamps negatives to zero:
            //   (if (gt rn max) (set rn max)) (if (lt rn 0) (set rn 0))
            const s32 max = static_cast<s32>((1u << n) - 1u);
            const s32 v = static_cast<s32>(R[static_cast<size_t>(rn)]);
            if (v > max) R[static_cast<size_t>(rn)] = static_cast<u32>(max);
            else if (v < 0) R[static_cast<size_t>(rn)] = 0;
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

// ---------------------------------------------------------------------------
// Save states
// ---------------------------------------------------------------------------

void MePCore::ControlBus::save_state(StateWriter& writer) const {
    writer.fixed(regs, [&](const u32& value) { writer.put_u32(value); });
    writer.put_u32(irq_levels);
    writer.put_u32(irq_edges);
    writer.put_u32(count);
    writer.put_u32(remaining);
    writer.put_bool(running);
    writer.put_bool(done);
    writer.put_bool(force_expired);
}

void MePCore::ControlBus::load_state(StateReader& reader) {
    reader.fixed(regs, [&](u32& value) { value = reader.get_u32(); });
    irq_levels = reader.get_u32();
    irq_edges = reader.get_u32();
    count = reader.get_u32();
    remaining = reader.get_u32();
    running = reader.get_bool();
    done = reader.get_bool();
    force_expired = reader.get_bool();
}

void MePCore::save_state(StateWriter& writer) const {
    // Shared core state first: instructions/cycles, halted, halt_reason, pc and
    // undefined_instruction.
    Cpu::save_state(writer);

    writer.fixed(r, [&](const u32& value) { writer.put_u32(value); });
    writer.put_u32(hi);
    writer.put_u32(lo);
    writer.put_u32(sar);
    writer.put_u32(lp);
    writer.put_u32(epc);
    writer.put_u32(npc);
    writer.put_u32(tmp);
    writer.put_u32(psw);
    writer.put_u32(exc);
    writer.put_u32(cfg);
    writer.put_u32(vid);
    writer.put_u32(id);
    writer.put_u32(dbg);
    writer.put_u32(depc);
    writer.put_u32(opt);
    writer.put_u32(rcfg);
    writer.put_u32(ccfg);
    writer.put_u32(cr0);
    writer.fixed(cond, [&](bool value) { writer.put_bool(value); });
    writer.put_u32(rpb);
    writer.put_u32(rpe);
    writer.put_u32(rpc);
    writer.put_u32(mb0);
    writer.put_u32(me0);
    writer.put_u32(mb1);
    writer.put_u32(me1);
    // The INTC registers, edge/level inputs and the delay timer live in the
    // control bus model; its own section keeps the layout name-checked.
    writer.begin("cbus");
    cbus.save_state(writer);
    writer.end();
    writer.put_bool(vliw_mode);
    writer.put_u32(reset_vector);
    writer.put_bool(rep_active_);
    writer.put_bool(rep_pending_back_);
    writer.put_bool(rep_endless_);
    writer.put_bool(branch_taken_);
    writer.put_u32(boot_vector_base_);
    // pc_hook is host wiring (the boot chain installs it), never state.
}

void MePCore::load_state(StateReader& reader) {
    Cpu::load_state(reader);

    // Read back in exactly the order save_state wrote; restore fields directly
    // and never recompute derived state (`vliw_mode`, the repeat flags and
    // `exc`'s hardware-pending bit are all restored from the stream).
    reader.fixed(r, [&](u32& value) { value = reader.get_u32(); });
    hi = reader.get_u32();
    lo = reader.get_u32();
    sar = reader.get_u32();
    lp = reader.get_u32();
    epc = reader.get_u32();
    npc = reader.get_u32();
    tmp = reader.get_u32();
    psw = reader.get_u32();
    exc = reader.get_u32();
    cfg = reader.get_u32();
    vid = reader.get_u32();
    id = reader.get_u32();
    dbg = reader.get_u32();
    depc = reader.get_u32();
    opt = reader.get_u32();
    rcfg = reader.get_u32();
    ccfg = reader.get_u32();
    cr0 = reader.get_u32();
    reader.fixed(cond, [&](bool& value) { value = reader.get_bool(); });
    rpb = reader.get_u32();
    rpe = reader.get_u32();
    rpc = reader.get_u32();
    mb0 = reader.get_u32();
    me0 = reader.get_u32();
    mb1 = reader.get_u32();
    me1 = reader.get_u32();
    reader.begin("cbus");
    cbus.load_state(reader);
    reader.end();
    vliw_mode = reader.get_bool();
    reset_vector = reader.get_u32();
    rep_active_ = reader.get_bool();
    rep_pending_back_ = reader.get_bool();
    rep_endless_ = reader.get_bool();
    branch_taken_ = reader.get_bool();
    boot_vector_base_ = reader.get_u32();
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
