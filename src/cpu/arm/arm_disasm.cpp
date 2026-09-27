// zeliboba - ARMv7-A (Cortex-A9) A32 / T16 / T32 disassembler.
//
// This is a faithful port of the reference disassembler that ships with the
// VitaTestSuite C# core, with the same classification rules and the same textual
// conventions so that listings from either tool can be diffed.
#include "cpu/arm/arm_disasm.h"

#include <cstdio>
#include <cstring>

#include "cpu/arm/arm_defs.h"
#include "cpu/arm/arm_vfp.h"

namespace zlb {

namespace {

// ---------------------------------------------------------------------------
// Output buffer
// ---------------------------------------------------------------------------

struct DState {
    std::string buf;
    Bus* bus = nullptr;
    bool thumb = false;
    u32 address = 0;
    u32 cond = 0xEu;
    bool has_cond = false;
    u32 insn = 0;

    void reset() {
        buf.clear();
        address = 0;
        cond = 0xEu;
        has_cond = false;
        insn = 0;
    }

    void s(const char* text) { buf += text; }
    void c(char ch) { buf += ch; }
    void u(u32 v) { buf += std::to_string(v); }

    void hex(u32 v) {
        char tmp[16];
        std::snprintf(tmp, sizeof(tmp), "0x%X", v);
        buf += tmp;
    }

    /// `#0x..` for non-negative values, `#-0x..` for values with bit 31 set.
    void imm(u32 v) {
        buf += '#';
        if (v >= 0x80000000u) {
            buf += "-0x";
            char tmp[16];
            std::snprintf(tmp, sizeof(tmp), "%X", static_cast<u32>(-static_cast<s32>(v)));
            buf += tmp;
        } else {
            char tmp[16];
            std::snprintf(tmp, sizeof(tmp), "%X", v);
            buf += tmp;
        }
    }

    void reg(int n) {
        buf += 'r';
        buf += std::to_string(n);
    }

    void vreg(bool dbl, int n) {
        buf += dbl ? 'd' : 's';
        buf += std::to_string(n);
    }

    void reg_list(u32 mask) {
        buf += '{';
        bool first = true;
        int i = 0;
        while (i < 16) {
            if ((mask & (1u << i)) == 0) {
                ++i;
                continue;
            }
            int j = i;
            while (j + 1 < 16 && (mask & (1u << (j + 1))) != 0) ++j;
            if (!first) buf += ", ";
            first = false;
            buf += 'r';
            buf += std::to_string(i);
            if (j > i) {
                buf += "-r";
                buf += std::to_string(j);
            }
            i = j + 1;
        }
        buf += '}';
    }

    void shift(int type, int amount) {
        if (type == 0 && amount == 0) return;
        switch (type) {
            case 0: buf += ", lsl #"; buf += std::to_string(amount); break;
            case 1: buf += ", lsr #"; buf += std::to_string(amount == 0 ? 32 : amount); break;
            case 2: buf += ", asr #"; buf += std::to_string(amount == 0 ? 32 : amount); break;
            default:
                if (amount == 0) buf += ", rrx";
                else { buf += ", ror #"; buf += std::to_string(amount); }
                break;
        }
    }

    void shift_reg(int type, int rs) {
        switch (type) {
            case 0: buf += ", lsl "; break;
            case 1: buf += ", lsr "; break;
            case 2: buf += ", asr "; break;
            default: buf += ", ror "; break;
        }
        reg(rs);
    }

    void s_flag(bool v) { if (v) buf += 's'; }
    void cond_suffix() { if (has_cond) buf += arm::condition_name(cond); }
};

// Scratch state; the disassembler is single threaded and never re-entered.
DState g_state;

u32 fetch_word(DState& s, u32 addr) {
    if (s.bus == nullptr) return 0;
    s.bus->context.pc = addr;
    s.bus->context.core = "arm-disasm";
    return s.bus->read32(addr & 0xFFFFFFFCu);
}

u32 fetch_half(DState& s, u32 addr) {
    if (s.bus == nullptr) return 0;
    s.bus->context.pc = addr;
    s.bus->context.core = "arm-disasm";
    return s.bus->read16(addr & 0xFFFFFFFEu);
}

u32 imm12_of(u32 instr) { return arm::decode_imm12(instr); }

void neon_transfer(DState& s, u32 instr);
void neon_data_processing(DState& s, u32 instr);

// ---------------------------------------------------------------------------
// A32
// ---------------------------------------------------------------------------

bool is_arm_misc_encoding(u32 instr) {
    if ((instr & 0x0FBF0FFFu) == 0x010F0000u) return true;  // MRS
    if ((instr & 0x0FB0FFF0u) == 0x0120F000u) return true;  // MSR (register)
    if ((instr & 0x0FFFFFF0u) == 0x012FFF10u) return true;  // BX
    if ((instr & 0x0FFFFFF0u) == 0x012FFF20u) return true;  // BXJ
    if ((instr & 0x0FFFFFF0u) == 0x012FFF30u) return true;  // BLX
    return false;
}

void fields(DState& s, u32 instr) {
    const bool spsr = (instr & (1u << 22)) != 0;
    const u32 f = (instr >> 16) & 0xFu;
    s.s(spsr ? "spsr_" : "cpsr_");
    if ((f & 8u) != 0) s.c('f');
    if ((f & 4u) != 0) s.c('s');
    if ((f & 2u) != 0) s.c('x');
    if ((f & 1u) != 0) s.c('c');
}

void operand2(DState& s, u32 instr, bool immediate) {
    if (immediate) {
        s.imm(imm12_of(instr));
        return;
    }
    s.reg(static_cast<int>(instr & 0xFu));
    if ((instr & 0x10u) == 0) s.shift(static_cast<int>((instr >> 5) & 3u), static_cast<int>((instr >> 7) & 0x1Fu));
    else s.shift_reg(static_cast<int>((instr >> 5) & 3u), static_cast<int>((instr >> 8) & 0xFu));
}

void arm_data_processing(DState& s, u32 addr, u32 instr, bool immediate) {
    const u32 op = (instr >> 21) & 0xFu;
    const bool sf = (instr & (1u << 20)) != 0;
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const int rd = static_cast<int>((instr >> 12) & 0xFu);

    const char* name;
    switch (op) {
        case 0x0u: name = "and"; break;
        case 0x1u: name = "eor"; break;
        case 0x2u: name = "sub"; break;
        case 0x3u: name = "rsb"; break;
        case 0x4u: name = "add"; break;
        case 0x5u: name = "adc"; break;
        case 0x6u: name = "sbc"; break;
        case 0x7u: name = "rsc"; break;
        case 0x8u: name = "tst"; break;
        case 0x9u: name = "teq"; break;
        case 0xAu: name = "cmp"; break;
        case 0xBu: name = "cmn"; break;
        case 0xCu: name = "orr"; break;
        case 0xDu: name = "mov"; break;
        case 0xEu: name = "bic"; break;
        default: name = "mvn"; break;
    }

    const bool test = op >= 8u && op <= 0xBu;
    const bool no_rn = op == 0xDu || op == 0xFu;
    const bool adr = (op == 4u || op == 2u) && !sf && rn == 15 && immediate;

    // A32 MOVW / MOVT: opcode 1000 / 1010 with S == 0.
    if (immediate && !sf && (op == 0x8u || op == 0xAu) && rd != 15) {
        const u32 imm16 = (static_cast<u32>(rn) << 12) | (instr & 0xFFFu);
        s.s(op == 0x8u ? "movw" : "movt");
        s.cond_suffix();
        s.c(' ');
        s.reg(rd);
        s.s(", ");
        if (op == 0x8u) s.imm(imm16);
        else s.hex(imm16);
        return;
    }

    s.s(name);
    if (sf && !test) s.s_flag(true);
    s.cond_suffix();
    s.c(' ');

    if (adr) {
        const u32 imm = imm12_of(instr);
        s.reg(rd);
        s.s(", #");
        s.hex(op == 4u ? addr + 8u + imm : addr + 8u - imm);
        return;
    }
    if (test) {
        s.reg(rn);
        s.s(", ");
        operand2(s, instr, immediate);
        return;
    }
    s.reg(rd);
    if (!no_rn) {
        s.s(", ");
        s.reg(rn);
    }
    s.s(", ");
    operand2(s, instr, immediate);
}

void arm_msr_imm(DState& s, u32 instr) {
    s.s("msr");
    s.cond_suffix();
    s.c(' ');
    fields(s, instr);
    s.s(", ");
    s.imm(imm12_of(instr));
}

// ---- multiply / swap -------------------------------------------------------

void arm_mul(DState& s, u32 addr, u32 instr) {
    (void)addr;
    const u32 op = (instr >> 21) & 0xFu;
    const bool sf = (instr & (1u << 20)) != 0;
    const int rd_lo = static_cast<int>((instr >> 12) & 0xFu);
    const int rd_hi = static_cast<int>((instr >> 16) & 0xFu);
    const int rs = static_cast<int>((instr >> 8) & 0xFu);
    const int rm = static_cast<int>(instr & 0xFu);

    // SWP / SWPB share the MUL encoding space.
    if ((instr & 0x0FB00F90u) == 0x01000090u) {
        s.s(((instr & (1u << 22)) != 0) ? "swpb" : "swp");
        s.cond_suffix();
        s.c(' ');
        s.reg(rd_lo);
        s.s(", ");
        s.reg(rm);
        s.s(", [");
        s.reg(rd_hi);
        s.c(']');
        return;
    }

    switch (op) {
        case 0x0u:
            s.s("mul"); s.s_flag(sf); s.cond_suffix(); s.c(' ');
            s.reg(rd_hi); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
            return;
        case 0x1u:
            s.s("mla"); s.s_flag(sf); s.cond_suffix(); s.c(' ');
            s.reg(rd_hi); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs); s.s(", "); s.reg(rd_lo);
            return;
        case 0x2u:
            s.s("umaal"); s.cond_suffix(); s.c(' ');
            s.reg(rd_lo); s.s(", "); s.reg(rd_hi); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
            return;
        case 0x3u:
            s.s("mls"); s.cond_suffix(); s.c(' ');
            s.reg(rd_hi); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs); s.s(", "); s.reg(rd_lo);
            return;
        case 0x4u:
            s.s("umull"); s.s_flag(sf); s.cond_suffix(); s.c(' ');
            s.reg(rd_lo); s.s(", "); s.reg(rd_hi); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
            return;
        case 0x5u:
            s.s("umlal"); s.s_flag(sf); s.cond_suffix(); s.c(' ');
            s.reg(rd_lo); s.s(", "); s.reg(rd_hi); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
            return;
        case 0x6u:
            s.s("smull"); s.s_flag(sf); s.cond_suffix(); s.c(' ');
            s.reg(rd_lo); s.s(", "); s.reg(rd_hi); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
            return;
        case 0x7u:
            s.s("smlal"); s.s_flag(sf); s.cond_suffix(); s.c(' ');
            s.reg(rd_lo); s.s(", "); s.reg(rd_hi); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
            return;
        default:
            break;
    }
    char tmp[16];
    std::snprintf(tmp, sizeof(tmp), "%08X", instr);
    s.s(".word 0x");
    s.s(tmp);
}

// ---- media / DSP -----------------------------------------------------------

bool rd_lo_is_zero(u32 instr) {
    return ((instr >> 8) & 0xFu) == 0u && ((instr >> 5) & 7u) == 0u && ((instr >> 12) & 0xFu) != 15u;
}

void arm_sat_extend(DState& s, u32 instr, int rd, int rn, int rm, u32 g, u32 o2) {
    (void)o2;
    const bool is_unsigned = (g & 4u) != 0u;
    const bool half = (g & 1u) != 0u;
    const u32 sel = (instr >> 5) & 3u;

    if (sel == 3u) {
        const int rot = static_cast<int>((instr >> 10) & 3u) * 8;
        const char* nm;
        if (rn == 15) nm = is_unsigned ? (half ? "uxth" : "uxtb") : (half ? "sxth" : "sxtb");
        else nm = is_unsigned ? (half ? "uxtah" : "uxtab") : (half ? "sxtah" : "sxtab");
        s.s(nm);
        s.cond_suffix();
        s.c(' ');
        s.reg(rd);
        s.s(", ");
        if (rn != 15) {
            s.reg(rn);
            s.s(", ");
        }
        s.reg(rm);
        if (rot != 0) {
            s.s(", ror #");
            s.u(static_cast<u32>(rot));
        }
        return;
    }
    if ((sel & 1u) != 0u && ((instr >> 7) & 0x1Fu) == 0x1Eu) {
        u32 sat16 = (instr >> 16) & 0xFu;
        if (!is_unsigned) sat16 += 1u;
        s.s(is_unsigned ? "usat16" : "ssat16");
        s.cond_suffix();
        s.c(' ');
        s.reg(rd);
        s.s(", #");
        s.u(sat16);
        s.s(", ");
        s.reg(rm);
        return;
    }
    if ((instr & 0x20u) != 0u) {
        s.s(".word 0x");
        char tmp[16];
        std::snprintf(tmp, sizeof(tmp), "%08X", instr);
        s.s(tmp);
        return;
    }
    {
        const u32 sat_imm = (instr >> 16) & 0x1Fu;
        const u32 sat = is_unsigned ? sat_imm : sat_imm + 1u;
        const u32 shift = (instr >> 7) & 0x1Fu;
        const bool asr = (instr & 0x40u) != 0u;
        s.s(is_unsigned ? "usat" : "ssat");
        s.cond_suffix();
        s.c(' ');
        s.reg(rd);
        s.s(", #");
        s.u(sat);
        s.s(", ");
        s.reg(rm);
        if (shift != 0 || asr) {
            s.s(asr ? ", asr #" : ", lsl #");
            s.u(asr && shift == 0 ? 32u : shift);
        }
    }
}

void arm_bitfield(DState& s, u32 instr, int rd) {
    const int lsb = static_cast<int>((instr >> 7) & 0x1Fu);
    const int field = static_cast<int>((instr >> 16) & 0x1Fu);
    const int rm = static_cast<int>(instr & 0xFu);
    const u32 g = (instr >> 20) & 0xFFu;
    const u32 sel = (instr >> 4) & 7u;

    if (sel == 1u) {
        s.s(rm == 15 ? "bfc" : "bfi");
        s.cond_suffix();
        s.c(' ');
        s.reg(rd);
        if (rm != 15) {
            s.s(", ");
            s.reg(rm);
        }
        s.s(", #");
        s.u(static_cast<u32>(lsb));
        s.s(", #");
        s.u(static_cast<u32>(field - lsb + 1));
        return;
    }
    if (sel == 5u) {
        s.s((g & 4u) == 0u ? "sbfx" : "ubfx");
        s.cond_suffix();
        s.c(' ');
        s.reg(rd);
        s.s(", ");
        s.reg(rm);
        s.s(", #");
        s.u(static_cast<u32>(lsb));
        s.s(", #");
        s.u(static_cast<u32>(field + 1));
        return;
    }
    s.s(".word 0x");
    char tmp[16];
    std::snprintf(tmp, sizeof(tmp), "%08X", instr);
    s.s(tmp);
}

void arm_media(DState& s, u32 addr, u32 instr) {
    (void)addr;
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const int rs = static_cast<int>((instr >> 8) & 0xFu);
    const int rm = static_cast<int>(instr & 0xFu);
    const u32 g = (instr >> 20) & 0xFFu;
    const u32 o2 = (instr >> 5) & 7u;
    const bool bit20 = (instr & (1u << 20)) != 0;
    const bool bit22 = (instr & (1u << 22)) != 0;
    const bool bit5 = (instr & 0x20u) != 0;
    const bool bit6 = (instr & 0x40u) != 0;

    if (g == 0x10u && bit5 && o2 == 0u && (instr & 0x80u) != 0u && rd_lo_is_zero(instr)) {
        s.s(bit22 ? "swpb" : "swp");
        s.cond_suffix();
        s.c(' ');
        s.reg(rd);
        s.s(", ");
        s.reg(rm);
        s.s(", [");
        s.reg(rn);
        s.c(']');
        return;
    }

    const u32 nib_a = instr & 0xF0u;
    if (g >= 0x10u && g <= 0x17u && (g & 1u) == 0u) {
        const bool xb = (instr & 0x20u) != 0u;
        const bool yb = (instr & 0x40u) != 0u;
        if (nib_a == 0x50u) {
            const char* q = g == 0x10u ? "qadd" : (g == 0x12u ? "qsub" : (g == 0x14u ? "qdadd" : "qdsub"));
            s.s(q);
            s.cond_suffix();
            s.c(' ');
            s.reg(rd);
            s.s(", ");
            s.reg(rm);
            s.s(", ");
            s.reg(rn);
            return;
        }
        if ((nib_a & 0x90u) == 0x80u) {
            const char* suf = (xb ? "t" : "b");
            const char* suf2 = (yb ? "t" : "b");
            if (g == 0x10u) {
                s.s("smla"); s.s(suf); s.s(suf2); s.cond_suffix(); s.c(' ');
                s.reg(rn); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs); s.s(", "); s.reg(rd);
                return;
            }
            if (g == 0x12u) {
                s.s(xb ? "smulw" : "smlaw"); s.s(yb ? "t" : "b"); s.cond_suffix(); s.c(' ');
                s.reg(rn); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
                if (!xb) { s.s(", "); s.reg(rd); }
                return;
            }
            if (g == 0x14u) {
                s.s("smlal"); s.s(suf); s.s(suf2); s.cond_suffix(); s.c(' ');
                s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
                return;
            }
            s.s("smul"); s.s(suf); s.s(suf2); s.cond_suffix(); s.c(' ');
            s.reg(rn); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
            return;
        }
    }

    switch (g) {
        case 0x60u:
            break;
        case 0x61u: case 0x62u: case 0x63u:
        case 0x65u: case 0x66u: case 0x67u: {
            const char* base_name;
            switch (o2) {
                case 0u: base_name = "add16"; break;
                case 1u: base_name = "asx"; break;
                case 2u: base_name = "sax"; break;
                case 3u: base_name = "sub16"; break;
                case 4u: base_name = "add8"; break;
                case 7u: base_name = "sub8"; break;
                default: base_name = nullptr; break;
            }
            if (base_name == nullptr) break;
            const char* pre;
            if (g == 0x62u) pre = "q";
            else if (g == 0x63u) pre = "sh";
            else if (g == 0x65u) pre = "u";
            else if (g == 0x66u) pre = "uq";
            else if (g == 0x67u) pre = "uh";
            else pre = "s";
            s.s(pre); s.s(base_name); s.cond_suffix(); s.c(' ');
            s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.reg(rm);
            return;
        }
        case 0x68u:
            if ((instr & 0xFF0u) == 0xFB0u) {
                s.s("sel"); s.cond_suffix(); s.c(' ');
                s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.reg(rm);
                return;
            }
            if ((o2 & 3u) == 0u) {
                s.s("pkhbt"); s.cond_suffix(); s.c(' ');
                s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.reg(rm);
                const u32 sh = (instr >> 7) & 0x1Fu;
                if (sh != 0) { s.s(", lsl #"); s.u(sh); }
                return;
            }
            if ((o2 & 3u) == 2u) {
                s.s("pkhtb"); s.cond_suffix(); s.c(' ');
                s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.reg(rm);
                u32 sh = (instr >> 7) & 0x1Fu;
                if (sh == 0) sh = 32;
                s.s(", asr #"); s.u(sh);
                return;
            }
            if ((o2 & 3u) == 3u) {
                const int rot = static_cast<int>((instr >> 10) & 3u) * 8;
                const bool low = rn == 15;
                s.s(bit22 ? (low ? "uxtb16" : "uxtab16") : (low ? "sxtb16" : "sxtab16"));
                s.cond_suffix();
                s.c(' ');
                s.reg(rd);
                if (!low) { s.s(", "); s.reg(rn); }
                s.s(", "); s.reg(rm);
                if (rot != 0) { s.s(", ror #"); s.u(static_cast<u32>(rot)); }
                return;
            }
            break;
        case 0x6Au: case 0x6Bu:
        case 0x6Eu: case 0x6Fu:
            arm_sat_extend(s, instr, rd, rn, rm, g, o2);
            return;
        case 0x69u: case 0x6Du:
            break;
        case 0x6Cu:
            if ((o2 & 3u) == 3u) {
                const int rot = static_cast<int>((instr >> 10) & 3u) * 8;
                const bool low = rn == 15;
                s.s(low ? "uxtb16" : "uxtab16"); s.cond_suffix(); s.c(' ');
                s.reg(rd);
                if (!low) { s.s(", "); s.reg(rn); }
                s.s(", "); s.reg(rm);
                if (rot != 0) { s.s(", ror #"); s.u(static_cast<u32>(rot)); }
                return;
            }
            break;
        case 0x70u: case 0x74u: {
            const bool q = (o2 & 1u) != 0;
            const char* nm;
            if (g == 0x70u) nm = q ? (bit20 ? "smlsdx" : "smusdx") : (bit20 ? "smladx" : "smuadx");
            else nm = q ? (bit20 ? "smlsd" : "smusd") : (bit20 ? "smlad" : "smuad");
            s.s(nm); s.cond_suffix(); s.c(' ');
            s.reg(rd); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
            if (bit20) { s.s(", "); s.reg(rn); }
            return;
        }
        case 0x78u:
            if (o2 == 0u) {
                s.s(bit20 ? "usada8" : "usad8"); s.cond_suffix(); s.c(' ');
                s.reg(rd); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
                if (bit20) { s.s(", "); s.reg(rn); }
                return;
            }
            if (o2 == 2u) {
                s.s(bit20 ? "smlsld" : "smlald"); s.cond_suffix(); s.c(' ');
                s.reg(rn); s.s(", "); s.reg(rd); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
                return;
            }
            break;
        case 0x7Au: case 0x7Bu:
        case 0x7Cu: case 0x7Du: case 0x7Eu: case 0x7Fu:
            arm_bitfield(s, instr, rd);
            return;
        case 0x71u: case 0x72u: case 0x73u:
        case 0x75u: case 0x76u: case 0x77u: {
            const u32 kind = (g >> 1) & 3u;
            const char* nm = kind == 0u ? "smmul" : (kind == 2u ? "smmla" : "smmls");
            s.s(nm);
            if (bit5) s.c('r');
            s.cond_suffix(); s.c(' ');
            s.reg(rd); s.s(", "); s.reg(rm); s.s(", "); s.reg(rs);
            if (kind != 0u) { s.s(", "); s.reg(rn); }
            return;
        }
        default:
            break;
    }

    char tmp[16];
    std::snprintf(tmp, sizeof(tmp), "%08X", instr);
    s.s(".word 0x");
    s.s(tmp);
}

// ---- misc ------------------------------------------------------------------

void arm_misc(DState& s, u32 addr, u32 instr) {
    (void)addr;
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    const int rm = static_cast<int>(instr & 0xFu);
    const u32 op2 = (instr >> 5) & 7u;

    if ((instr & 0x0FFFFFF0u) == 0x012FFF10u) { s.s("bx"); s.cond_suffix(); s.c(' '); s.reg(rm); return; }
    if ((instr & 0x0FFFFFF0u) == 0x012FFF20u) { s.s("bxj"); s.cond_suffix(); s.c(' '); s.reg(rm); return; }
    if ((instr & 0x0FFFFFF0u) == 0x012FFF30u) { s.s("blx"); s.cond_suffix(); s.c(' '); s.reg(rm); return; }

    const u32 o1m = (instr >> 20) & 0xFFu;
    if ((instr & 0xFFFu) == 0u && (o1m == 0x10u || o1m == 0x14u)) {
        const bool spsr = (instr & (1u << 22)) != 0;
        s.s("mrs"); s.cond_suffix(); s.c(' ');
        s.reg(rd); s.s(", ");
        s.s(spsr ? "spsr" : "cpsr");
        return;
    }
    if ((instr & 0xFF0u) == 0u && (o1m == 0x12u || o1m == 0x16u)) {
        s.s("msr"); s.cond_suffix(); s.c(' ');
        fields(s, instr); s.s(", "); s.reg(rm);
        return;
    }
    if (op2 == 1u) {
        s.s("clz"); s.cond_suffix(); s.c(' ');
        s.reg(rd); s.s(", "); s.reg(rm);
        return;
    }
    char tmp[16];
    std::snprintf(tmp, sizeof(tmp), "%08X", instr);
    s.s(".word 0x");
    s.s(tmp);
}

void arm_clz(DState& s, u32 instr) {
    s.s("clz"); s.cond_suffix(); s.c(' ');
    s.reg(static_cast<int>((instr >> 12) & 0xFu));
    s.s(", ");
    s.reg(static_cast<int>(instr & 0xFu));
}

// ---- extra load/store ------------------------------------------------------

void arm_mem_operand(DState& s, u32 instr, int rn, bool p, bool u, bool w, bool immediate, u32 imm) {
    s.s(", [");
    s.reg(rn);
    if (p) {
        if (immediate) {
            s.s(", ");
            s.imm(u ? imm : static_cast<u32>(-static_cast<s32>(imm)));
        } else {
            s.s(", ");
            if (!u) s.c('-');
            s.reg(static_cast<int>(instr & 0xFu));
        }
        s.c(']');
        if (w) s.c('!');
    } else {
        s.c(']');
        s.s(", ");
        if (immediate) s.imm(u ? imm : static_cast<u32>(-static_cast<s32>(imm)));
        else {
            if (!u) s.c('-');
            s.reg(static_cast<int>(instr & 0xFu));
        }
    }
}

// ---- exclusive load/store --------------------------------------------------

/// LDREX/STREX share the multiply space (bits [7:4] = 1001) and are told apart
/// by bits [27:20]: 8/9 word, C/D byte, E/F halfword.  They have to be decoded
/// before the multiplies, otherwise the loop that kernel_boot_loader uses to
/// post and wait for completion of a job (a decrementing counter at
/// 0x4005C008, see docs/KBL.md) disassembles as `smlal r12,r0,r3,r15`.
bool is_arm_exclusive(u32 instr) {
    const u32 excl = instr & 0x0FF00FF0u;
    return excl == 0x01900F90u || excl == 0x01800F90u ||   // LDREX  / STREX
           excl == 0x01D00F90u || excl == 0x01C00F90u ||   // LDREXB / STREXB
           excl == 0x01F00F90u || excl == 0x01E00F90u;     // LDREXH / STREXH
}

void arm_exclusive(DState& s, u32 instr) {
    const u32 kind = (instr >> 20) & 0xFu;
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    const int rt = static_cast<int>(instr & 0xFu);

    const char* nm = "undef";
    bool load = true;
    switch (kind) {
        case 0x9u: nm = "ldrex"; break;
        case 0x8u: nm = "strex"; load = false; break;
        case 0xDu: nm = "ldrexb"; break;
        case 0xCu: nm = "strexb"; load = false; break;
        case 0xFu: nm = "ldrexh"; break;
        case 0xEu: nm = "strexh"; load = false; break;
        default: break;
    }
    s.s(nm);
    s.cond_suffix();
    s.c(' ');
    s.reg(rd);
    if (!load) {
        s.s(", ");
        s.reg(rt);
    }
    s.s(", [");
    s.reg(rn);
    s.c(']');
}

void arm_extra_load_store(DState& s, u32 addr, u32 instr) {
    (void)addr;
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const bool p = (instr & (1u << 24)) != 0;
    const bool u = (instr & (1u << 23)) != 0;
    const bool immediate = (instr & (1u << 22)) != 0;
    const bool w = (instr & (1u << 21)) != 0;
    const bool l = (instr & (1u << 20)) != 0;
    const u32 sel = (instr >> 5) & 3u;

    const char* nm;
    bool pair = false;
    switch (sel) {
        case 1u: nm = l ? "ldrh" : "strh"; break;
        case 2u: nm = l ? "ldrsb" : "ldrd"; pair = !l; break;
        case 3u: nm = l ? "ldrsh" : "strd"; pair = !l; break;
        default: nm = "undef"; break;
    }
    if (std::strcmp(nm, "undef") == 0) {
        char tmp[16];
        std::snprintf(tmp, sizeof(tmp), "%08X", instr);
        s.s(".word 0x");
        s.s(tmp);
        return;
    }

    s.s(nm);
    if (!p && w) s.c('t');
    s.cond_suffix();
    s.c(' ');
    s.reg(rd);
    if (pair) {
        s.s(", r");
        s.u(static_cast<u32>(rd + 1));
    }
    const u32 imm = immediate ? ((((instr >> 8) & 0xFu) << 4) | (instr & 0xFu)) : 0u;
    arm_mem_operand(s, instr, rn, p, u, w, immediate, imm);
}

// ---- load/store ------------------------------------------------------------

void arm_load_store(DState& s, u32 addr, u32 instr, bool immediate) {
    const int rd = static_cast<int>((instr >> 12) & 0xFu);
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const bool p = (instr & (1u << 24)) != 0;
    const bool u = (instr & (1u << 23)) != 0;
    const bool b = (instr & (1u << 22)) != 0;
    const bool w = (instr & (1u << 21)) != 0;
    const bool l = (instr & (1u << 20)) != 0;

    s.s(l ? "ldr" : "str");
    if (b) s.c('b');
    if (!p && w) s.c('t');
    s.cond_suffix();
    s.c(' ');
    s.reg(rd);

    if (rn == 15 && p && !w && immediate && (instr & 0xFFFu) != 0) {
        const u32 off = instr & 0xFFFu;
        s.s(", #");
        s.hex(u ? addr + 8u + off : addr + 8u - off);
        return;
    }

    s.s(", [");
    s.reg(rn);
    if (!p) {
        s.c(']');
        s.s(", ");
    } else {
        s.s(", ");
    }
    if (immediate) {
        const u32 off = instr & 0xFFFu;
        s.imm(u ? off : static_cast<u32>(-static_cast<s32>(off)));
    } else {
        if (!u) s.c('-');
        s.reg(static_cast<int>(instr & 0xFu));
        s.shift(static_cast<int>((instr >> 5) & 3u), static_cast<int>((instr >> 7) & 0x1Fu));
    }
    if (p) {
        s.c(']');
        if (w) s.c('!');
    }
}

void arm_load_store_multiple(DState& s, u32 addr, u32 instr) {
    (void)addr;
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const bool p = (instr & (1u << 24)) != 0;
    const bool u = (instr & (1u << 23)) != 0;
    const bool sf = (instr & (1u << 22)) != 0;
    const bool w = (instr & (1u << 21)) != 0;
    const bool l = (instr & (1u << 20)) != 0;
    const u32 list = instr & 0xFFFFu;

    std::string nm = l ? "ldm" : "stm";
    bool alias = false;
    if (w && rn == 13) {
        if (l && u) {
            nm = "pop";
            alias = true;
        } else if (!l && !u) {
            nm = "push";
            alias = true;
        }
    }
    if (!alias) {
        if (u) nm += p ? "ib" : "ia";
        else nm += p ? "db" : "da";
    }

    s.s(nm.c_str());
    s.cond_suffix();
    s.c(' ');
    if (alias) {
        if (!l) s.c(' ');
    } else {
        s.reg(rn);
        if (w) s.c('!');
        s.s(", ");
    }
    s.reg_list(list);
    if (sf) s.s("^");
}

void arm_branch(DState& s, u32 addr, u32 instr) {
    const bool link = (instr & (1u << 24)) != 0;
    s32 imm = static_cast<s32>(instr & 0xFFFFFFu);
    if ((imm & 0x800000) != 0) imm |= static_cast<s32>(0xFF000000u);
    s.s(link ? "bl" : "b");
    s.cond_suffix();
    s.s(" #");
    s.hex(addr + 8u + static_cast<u32>(imm << 2));
}

// ---- coprocessor -----------------------------------------------------------

void vfp_load_store(DState& s, u32 addr, u32 instr) {
    (void)addr;
    const bool load = (instr & (1u << 20)) != 0;
    const bool single = (instr & 0x100u) == 0u;  // coprocessor 10 = single
    const bool multiple = !((instr & (1u << 24)) != 0u && (instr & (1u << 21)) == 0u);
    const bool p = (instr & (1u << 24)) != 0;
    const bool u = (instr & (1u << 23)) != 0;
    const bool w = (instr & (1u << 21)) != 0;
    const int rn = static_cast<int>((instr >> 16) & 0xFu);
    const int vd = static_cast<int>((instr >> 12) & 0xFu);
    const bool d_bit = (instr & (1u << 22)) != 0;
    const int d = single ? ((vd << 1) | (d_bit ? 1 : 0)) : (vd | (d_bit ? 0x10 : 0));
    const u32 imm8 = instr & 0xFFu;

    if (multiple) {
        const int count = single ? static_cast<int>(imm8) : static_cast<int>(imm8 / 2u);
        s.s(load ? "vldm" : "vstm");
        s.s(u ? (p ? "ib " : "ia ") : (p ? "db " : "da "));
        s.reg(rn);
        if (w) s.c('!');
        s.s(", {");
        for (int k = 0; k < count; ++k) {
            if (k != 0) s.s(", ");
            s.c(single ? 's' : 'd');
            s.u(static_cast<u32>(d + k));
        }
        s.c('}');
        return;
    }

    s.s(load ? "vldr" : "vstr");
    s.s(single ? ".f32 " : ".f64 ");
    s.c(single ? 's' : 'd');
    s.u(static_cast<u32>(d));
    s.s(", [");
    s.reg(rn);
    s.s(", ");
    if (!u) s.c('-');
    s.imm(imm8 * 4u);
    s.c(']');
    if (p && w) s.c('!');
}

/// Advanced SIMD (NEON) instruction names. Only the subset this core actually
/// executes is named precisely; everything else gets an explicit `neon.<opc>`
/// name rather than a bare `.word`, so a listing never lies about the class.
void neon_transfer(DState& s, u32 instr) {
    // VDUP (ARM core register to all lanes): bits [27:20] == 1110 1010, size in
    // {bits[22:21], bit 5}, destination in imm4 (bits [19:16]).
    if ((instr & 0x0FB00FF0u) == 0x0EA00B10u) {
        s.s("vdup.32 q");
        s.u(((instr >> 16) & 0xFu) / 2u);
        s.s(", ");
        s.reg(static_cast<int>((instr >> 12) & 0xFu));
        return;
    }
    if ((instr & 0x0FB00FF0u) == 0x0EA00B30u) {
        s.s("vdup.16 q");
        s.u(((instr >> 16) & 0xFu) / 2u);
        s.s(", ");
        s.reg(static_cast<int>((instr >> 12) & 0xFu));
        return;
    }
    if ((instr & 0x0FB00FF0u) == 0x0EE00B10u) {
        s.s("vdup.8 q");
        s.u(((instr >> 16) & 0xFu) / 2u);
        s.s(", ");
        s.reg(static_cast<int>((instr >> 12) & 0xFu));
        return;
    }
    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "neon.transfer 0x%08X", instr);
    s.s(tmp);
}

/// Advanced SIMD data processing. The 3-same-length group is named from the
/// (U, size, opc1, opc2) fields; anything else is reported by class.
void neon_data_processing(DState& s, u32 instr) {
    const u32 opc1 = (instr >> 8) & 0xFu;
    const bool opc2 = (instr & 0x10u) != 0u;
    const u32 size = (instr >> 20) & 3u;
    const bool u_bit = (instr & (1u << 24)) != 0u;
    const int d = (static_cast<int>((instr >> 22) & 1u) << 4) | static_cast<int>((instr >> 12) & 0xFu);
    const int n = (static_cast<int>((instr >> 7) & 1u) << 4) | static_cast<int>((instr >> 16) & 0xFu);
    const int m = (static_cast<int>((instr >> 5) & 1u) << 4) | static_cast<int>(instr & 0xFu);
    const bool quad = (instr & 0x40u) != 0u;
    const char* name = nullptr;
    if (opc1 == 0x1u && opc2 && !u_bit) {
        name = size == 0u ? "vand" : (size == 1u ? "vbic" : (size == 2u ? "vorr" : "vorn"));
    } else if (opc1 == 0x3u && opc2 && !u_bit) {
        name = size == 0u ? "veor" : (size == 1u ? "vbsl" : (size == 2u ? "vbit" : "vbif"));
    }
    if (name != nullptr) {
        s.s(name);
        s.c(' ');
        s.c(quad ? 'q' : 'd');
        s.u(static_cast<u32>(quad ? d / 2 : d));
        s.s(", ");
        s.c(quad ? 'q' : 'd');
        s.u(static_cast<u32>(quad ? n / 2 : n));
        s.s(", ");
        s.c(quad ? 'q' : 'd');
        s.u(static_cast<u32>(quad ? m / 2 : m));
        return;
    }
    char tmp[48];
    std::snprintf(tmp, sizeof(tmp), "neon.dp 0x%08X (opc1=%X opc2=%u size=%u)", instr, opc1,
                  opc2 ? 1u : 0u, size);
    s.s(tmp);
}
void vfp_data_processing(DState& s, u32 addr, u32 instr) {
    (void)addr;
    const bool sz = (instr & 0x100u) != 0;
    const u32 opc1 = (instr >> 20) & 0xFu;
    const u32 opc2 = (instr >> 6) & 0xFu;
    const u32 g = (instr >> 20) & 0xFFu;
    const int rd_core = static_cast<int>((instr >> 12) & 0xFu);

    const u32 f_vd = (instr >> 12) & 0xFu;
    const u32 f_vn = (instr >> 16) & 0xFu;
    const u32 f_vm = instr & 0xFu;
    const bool b_d = (instr & (1u << 22)) != 0u;
    const bool b_n = (instr & 0x80u) != 0u;
    const bool b_m = (instr & 0x20u) != 0u;
    const int vd = sz ? static_cast<int>(f_vd | (b_d ? 16u : 0u)) : static_cast<int>((f_vd << 1) | (b_d ? 1u : 0u));
    const int vn = sz ? static_cast<int>(f_vn | (b_n ? 16u : 0u)) : static_cast<int>((f_vn << 1) | (b_n ? 1u : 0u));
    const int vm = sz ? static_cast<int>(f_vm | (b_m ? 16u : 0u)) : static_cast<int>((f_vm << 1) | (b_m ? 1u : 0u));
    const char* prec = sz ? ".f64 " : ".f32 ";

    // VMOV between a core register and a single/double register.
    if ((instr & 0x0F800E1Fu) == 0x0E000A10u) {
        const bool to_core = (instr & (1u << 20)) != 0;
        if (sz) {
            const int lane = static_cast<int>((instr >> 21) & 1u);
            s.s("vmov.32 ");
            if (to_core) {
                s.reg(rd_core); s.s(", d"); s.u(static_cast<u32>(vn)); s.c('['); s.u(static_cast<u32>(lane)); s.c(']');
            } else {
                s.s("d"); s.u(static_cast<u32>(vn)); s.c('['); s.u(static_cast<u32>(lane)); s.s("], "); s.reg(rd_core);
            }
        } else {
            if (to_core) { s.s("vmov "); s.reg(rd_core); s.s(", s"); s.u(static_cast<u32>(vn)); }
            else { s.s("vmov s"); s.u(static_cast<u32>(vn)); s.s(", "); s.reg(rd_core); }
        }
        return;
    }

    // VMRS / VMSR.
    if ((instr & 0x0FE00F10u) == 0x0EE00A10u) {   // VMSR (1110) and VMRS (1111)
        const bool to_core = (instr & (1u << 20)) != 0;
        const u32 reg = (instr >> 16) & 0xFu;
        const char* sysreg;
        switch (reg) {
            case 0u: sysreg = "fpsid"; break;
            case 1u: sysreg = "fpscr"; break;
            case 5u: sysreg = "fpsr"; break;
            case 6u: sysreg = "mvfr0"; break;
            case 7u: sysreg = "mvfr1"; break;
            case 8u: sysreg = "fpexc"; break;
            default: sysreg = "fpsr"; break;
        }
        // ARM syntax: `VMRS <Rt>, <sysreg>` but `VMSR <sysreg>, <Rt>`.
        if (to_core) {
            s.s("vmrs ");
            if (rd_core == 15) s.s("apsr_nzcv, ");
            else { s.reg(rd_core); s.s(", "); }
            s.s(sysreg);
        } else {
            s.s("vmsr ");
            s.s(sysreg);
            s.s(", ");
            s.reg(rd_core);
        }
        return;
    }
    if ((g & 0xEFu) == 0xC4u && (instr & 0x10u) != 0u) {
        const bool to_double = (instr & 0x100u) != 0;
        const int d2 = static_cast<int>(((instr >> 12) & 0xFu) | ((instr & (1u << 22)) != 0 ? 0x10u : 0u));
        const int rt2 = static_cast<int>((instr >> 16) & 0xFu);
        if (to_double) { s.s("vmov d"); s.u(static_cast<u32>(d2)); s.s(", "); s.reg(rd_core); s.s(", "); s.reg(rt2); }
        else { s.s("vmov "); s.reg(rd_core); s.s(", "); s.reg(rt2); s.s(", d"); s.u(static_cast<u32>(d2)); }
        return;
    }

    if ((instr & 0x0FB00E50u) == 0x0EB00A00u) {
        const u32 imm4_h = (instr >> 16) & 0xFu;
        const u32 imm4_l = instr & 0xFu;
        const u32 cmode = (instr >> 8) & 0xFu;
        if ((cmode & 0xEu) == 0xAu) {
            const u32 imm8 = (imm4_h << 4) | imm4_l;
            s.s("vmov."); s.s(sz ? "f64 " : "f32 ");
            s.c(sz ? 'd' : 's');
            s.u(static_cast<u32>(sz ? (vd / 2) : vd));
            s.s(", #");
            if (sz) {
                const u64 bits = ArmVfp::expand_immediate64(imm8);
                f64 d = 0.0;
                std::memcpy(&d, &bits, sizeof(d));
                char tmp[48];
                std::snprintf(tmp, sizeof(tmp), "%.17g", d);
                s.s(tmp);
            } else {
                const f32 f = ArmVfp::bits_to_float(ArmVfp::expand_immediate(imm8));
                char tmp[48];
                std::snprintf(tmp, sizeof(tmp), "%.9g", static_cast<double>(f));
                s.s(tmp);
            }
            return;
        }
    }

    if (opc1 == 0xBu && (opc2 & 5u) == 4u) {
        if (opc2 == 0x4u || opc2 == 0x5u) {
            const bool zero = (instr & 0x40u) != 0;
            s.s("vcmp");
            if ((opc2 & 1u) != 0) s.c('e');
            s.s(prec);
            s.vreg(sz, vd);
            if (zero) s.s(", #0.0");
            else { s.s(", "); s.vreg(sz, vm); }
            return;
        }
        if (opc2 == 0x7u) {
            const bool to_int = (instr & 0x80u) == 0u;
            const u32 ty = (instr >> 16) & 3u;
            const char* t1 = (ty & 1u) != 0 ? "u" : "s";
            const char* t2 = (ty & 2u) != 0 ? "32" : "16";
            s.s("vcvt");
            if (to_int) {
                s.s("."); s.s(t1); s.s(t2); s.s(sz ? ".f64 " : ".f32 ");
                s.vreg(false, vd); s.s(", "); s.vreg(sz, vm);
            } else {
                s.s(sz ? ".f64." : ".f32."); s.s(t1); s.s(t2); s.s(" ");
                s.vreg(sz, vd); s.s(", "); s.vreg(false, vm);
            }
            return;
        }
        char tmp[16];
        std::snprintf(tmp, sizeof(tmp), "%08X", instr);
        s.s(".word 0x");
        s.s(tmp);
        return;
    }

    // Bit 4 set with bits [23:20] == 1010 or 1011 is the Advanced SIMD (NEON)
    // transfer space, not VFP: 0xEEA00B10 is `vdup.32 q0, r0`. Without this
    // split the sel-based VFMA switch below swallows the whole class and prints
    // every one of them as a floating-point multiply-accumulate.
    {
        const u32 tag = (instr >> 20) & 0xFu;
        if ((instr & 0x10u) != 0u && (tag == 0xAu || tag == 0xBu)) {
            neon_transfer(s, instr);
            return;
        }
    }
    const u32 sel = (((instr >> 23) & 1u) << 2) | (((instr >> 21) & 1u) << 1) | ((instr >> 20) & 1u);
    const bool opc3 = (instr & 0x40u) != 0;
    if (sel == 7u) {
        if (!opc3) {
            char tmp[16];
            std::snprintf(tmp, sizeof(tmp), "%08X", instr);
            s.s(".word 0x");
            s.s(tmp);
            return;
        }
        const u32 opc2v = (instr >> 16) & 0xFu;
        const bool n_bit = (instr & 0x80u) != 0;
        if (opc2v == 4u) {
            s.s(n_bit ? "vcmpe" : "vcmp"); s.s(prec);
            s.vreg(sz, vd);
            if (vm == 0) s.s(", #0.0");
            else { s.s(", "); s.vreg(sz, vm); }
            return;
        }
        if (opc2v == 8u || opc2v == 0xCu || opc2v == 0xDu) {
            const int sd = static_cast<int>((((instr >> 12) & 0xFu) << 1) | ((instr >> 22) & 1u));
            const int sm = static_cast<int>(((instr & 0xFu) << 1) | ((instr >> 5) & 1u));
            const char* ty = (opc2v == 8u) ? (n_bit ? "s32" : "u32") : (opc2v == 0xDu ? "s32" : "u32");
            if (opc2v == 8u) {
                s.s("vcvt."); s.s(sz ? "f64." : "f32."); s.s(ty); s.c(' ');
                s.vreg(sz, vd); s.s(", s"); s.u(static_cast<u32>(sm));
            } else {
                s.s(opc2v == 0xCu ? "vcvtr." : "vcvt.");
                s.s(ty); s.s(prec);
                s.s("s"); s.u(static_cast<u32>(sd)); s.s(", "); s.vreg(sz, vm);
            }
            return;
        }
        if (opc2v == 1u) {
            s.s(n_bit ? "vsqrt" : "vneg"); s.s(prec);
            s.vreg(sz, vd); s.s(", "); s.vreg(sz, vm);
            return;
        }
        if (opc2v == 0u) {
            s.s(n_bit ? "vabs" : "vmov"); s.s(prec);
            s.vreg(sz, vd); s.s(", "); s.vreg(sz, vm);
            return;
        }
        char tmp[16];
        std::snprintf(tmp, sizeof(tmp), "%08X", instr);
        s.s(".word 0x");
        s.s(tmp);
        return;
    }

    const char* nm;
    switch (sel) {
        case 0x0u: nm = opc3 ? "vmls" : "vmla"; break;
        case 0x1u: nm = opc3 ? "vnmla" : "vnmls"; break;
        case 0x2u: nm = opc3 ? "vnmul" : "vmul"; break;
        case 0x3u: nm = opc3 ? "vsub" : "vadd"; break;
        case 0x4u:
            if (opc3) {
                char tmp[16];
                std::snprintf(tmp, sizeof(tmp), "%08X", instr);
                s.s(".word 0x");
                s.s(tmp);
                return;
            }
            nm = "vdiv";
            break;
        case 0x5u: nm = opc3 ? "vfnma" : "vfnms"; break;
        case 0x6u: nm = opc3 ? "vfms" : "vfma"; break;
        default: {
            char tmp[16];
            std::snprintf(tmp, sizeof(tmp), "%08X", instr);
            s.s(".word 0x");
            s.s(tmp);
            return;
        }
    }

    s.s(nm); s.s(prec); s.vreg(sz, vd);
    s.s(", "); s.vreg(sz, vn);
    s.s(", "); s.vreg(sz, vm);
}

void arm_coprocessor(DState& s, u32 addr, u32 instr) {
    const u32 cp = (instr >> 8) & 0xFu;
    const u32 blk = (instr >> 25) & 7u;

    if (blk == 6u) {
        if ((instr & 0x01E00000u) == 0x00400000u) {
            const bool l = (instr & (1u << 20)) != 0;
            s.s(l ? "mrrc" : "mcrr"); s.cond_suffix(); s.c(' ');
            s.s("p"); s.u(cp); s.s(", #"); s.u((instr >> 4) & 0xFu);
            s.s(", "); s.reg(static_cast<int>((instr >> 12) & 0xFu));
            s.s(", "); s.reg(static_cast<int>((instr >> 16) & 0xFu));
            s.s(", c"); s.u(instr & 0xFu);
            return;
        }
        if (cp == 10u || cp == 11u) {
            vfp_load_store(s, addr, instr);
            return;
        }
        const bool ldc = (instr & (1u << 20)) != 0;
        const bool long_form = (instr & (1u << 22)) != 0;
        s.s(ldc ? "ldc" : "stc");
        if (long_form) s.c('l');
        s.cond_suffix(); s.c(' ');
        s.s("p"); s.u(cp); s.s(", c"); s.u((instr >> 12) & 0xFu);
        arm_mem_operand(s, instr, static_cast<int>((instr >> 16) & 0xFu), (instr & (1u << 24)) != 0,
                        (instr & (1u << 23)) != 0, (instr & (1u << 21)) != 0, true, (instr & 0xFFu) * 4u);
        return;
    }

    if (cp == 10u || cp == 11u) {
        vfp_data_processing(s, addr, instr);
        return;
    }
    if ((instr & 0x10u) == 0u) {
        s.s("cdp"); s.cond_suffix(); s.c(' ');
        s.s("p"); s.u(cp); s.s(", #"); s.u((instr >> 20) & 0xFu);
        s.s(", c"); s.u((instr >> 12) & 0xFu);
        s.s(", c"); s.u((instr >> 16) & 0xFu);
        s.s(", c"); s.u(instr & 0xFu);
        s.s(", #"); s.u((instr >> 5) & 7u);
        return;
    }

    const u32 op1 = (instr >> 21) & 7u;
    const bool load = (instr & (1u << 20)) != 0;
    s.s(load ? "mrc" : "mcr"); s.cond_suffix(); s.c(' ');
    s.s("p"); s.u(cp); s.s(", #"); s.u(op1);
    s.s(", "); s.reg(static_cast<int>((instr >> 12) & 0xFu));
    s.s(", c"); s.u((instr >> 16) & 0xFu);
    s.s(", c"); s.u(instr & 0xFu);
    s.s(", #"); s.u((instr >> 5) & 7u);
}

// ---- unconditional space ---------------------------------------------------

void arm_unconditional(DState& s, u32 addr, u32 instr) {
    // 1111 101H: BLX (immediate); H is bit 24 and contributes offset bit 1.
    // Advanced SIMD data processing (1111 001x) and element/structure
    // load-store (1111 0100) are NEON, not the A32 unconditional misc space.
    if ((instr & 0x0E000000u) == 0x02000000u || (instr & 0x0F000000u) == 0x04000000u) {
        neon_data_processing(s, instr);
        return;
    }

    if ((instr & 0xFE000000u) == 0xFA000000u) {
        const u32 h = (instr >> 24) & 1u;
        u32 imm = ((instr & 0xFFFFFFu) << 2) | (h << 1);
        const s32 offset = arm::sign_extend32(imm, 26);
        s.s("blx #");
        s.hex(addr + 8u + static_cast<u32>(offset));
        return;
    }

    const u32 cp = (instr >> 8) & 0xFu;
    const u32 blk = (instr >> 25) & 7u;

    if (cp == 10u || cp == 11u) {
        if (blk == 6u) {
            vfp_load_store(s, addr, instr);
            return;
        }
        vfp_data_processing(s, addr, instr);
        return;
    }

    if ((instr & 0xFFFFFF00u) == 0xF57FF000u || (instr & 0xFFFFFFF0u) == 0xF57FF010u) {
        const u32 opc = instr & 0xFFu;
        if (opc == 0x1Fu) {
            s.s("clrex");
            return;
        }
        s.s((opc & 0xF0u) == 0x40u ? "dsb" : (opc & 0xF0u) == 0x50u ? "dmb" : "isb");
        if ((opc & 0xFu) == 0xFu) s.s(" sy");
        else { s.s(" #"); s.u(opc & 0xFu); }
        return;
    }

    if ((instr & 0xFFFFFDFFu) == 0xF1010000u) {
        s.s("setend ");
        s.s((instr & (1u << 9)) != 0 ? "be" : "le");
        return;
    }

    if ((instr & 0xFF300020u) == 0xF1000000u) {
        const u32 imod = (instr >> 18) & 3u;
        s.s(imod == 0u ? "cps" : (imod == 2u ? "cpsie" : "cpsid"));
        s.c(' ');
        const u32 aif = (instr >> 6) & 7u;
        bool any = false;
        if ((aif & 4u) != 0) { s.c('a'); any = true; }
        if ((aif & 2u) != 0) { s.c('i'); any = true; }
        if ((aif & 1u) != 0) { s.c('f'); any = true; }
        if (!any) s.s("none");
        if ((instr & (1u << 17)) != 0) { s.s(", #"); s.u(instr & 0x1Fu); }
        return;
    }

    if ((instr & 0xFFFFFFF0u) == 0xE320F000u) {
        switch (instr & 0xFu) {
            case 0u: s.s("nop"); return;
            case 1u: s.s("yield"); return;
            case 2u: s.s("wfe"); return;
            case 3u: s.s("wfi"); return;
            case 4u: s.s("sev"); return;
            default: s.s("hint #"); s.u(instr & 0xFu); return;
        }
    }

    if ((instr & 0x0FFFFFF0u) == 0x012FFF10u) { s.s("bx "); s.reg(static_cast<int>(instr & 0xFu)); return; }
    if ((instr & 0x0FFFFFF0u) == 0x012FFF20u) { s.s("blx "); s.reg(static_cast<int>(instr & 0xFu)); return; }
    if ((instr & 0x0FFFFFF0u) == 0x012FFF30u) { s.s("bxj "); s.reg(static_cast<int>(instr & 0xFu)); return; }

    if ((instr & 0x0FF000F0u) == 0x01600070u) { s.s("smc #"); s.u(instr & 0xFu); return; }
    if ((instr & 0x0FF000F0u) == 0x01400070u) { s.s("hvc #"); s.u(instr & 0xFu); return; }
    if ((instr & 0x0FF000F0u) == 0x01200070u) {
        s.s("bkpt #");
        s.u((((instr >> 4) & 0xFFFu) << 4) | (instr & 0xFu));
        return;
    }

    if ((instr & 0x0FE00000u) == 0x08100000u) {
        const int rn = static_cast<int>((instr >> 16) & 0xFu);
        const bool p = (instr & (1u << 24)) != 0;
        const bool u = (instr & (1u << 23)) != 0;
        const bool w = (instr & (1u << 21)) != 0;
        s.s("rfe");
        s.s(u ? (p ? "ib " : "ia ") : (p ? "db " : "da "));
        s.reg(rn);
        if (w) s.c('!');
        return;
    }
    if ((instr & 0x0FE00000u) == 0x08C00000u || (instr & 0x0FE00000u) == 0x08E00000u) {
        const bool p = (instr & (1u << 24)) != 0;
        const bool u = (instr & (1u << 23)) != 0;
        const bool w = (instr & (1u << 21)) != 0;
        s.s("srs");
        s.s(u ? (p ? "ib " : "ia ") : (p ? "db " : "da "));
        s.c('#');
        s.u(instr & 0x1Fu);
        if (w) s.c('!');
        return;
    }

    if ((instr & 0x0D700000u) == 0x05500000u) { s.s("pld"); return; }
    if ((instr & 0x0D200000u) == 0x05000000u) { s.s("pli"); return; }

    char tmp[16];
    std::snprintf(tmp, sizeof(tmp), "%08X", instr);
    s.s(".word 0x");
    s.s(tmp);
}

void arm_decode(DState& s, u32 addr, unsigned& length) {
    length = 4;
    const u32 instr = fetch_word(s, addr);
    s.insn = instr;
    const u32 cond = instr >> 28;
    s.cond = cond;
    s.has_cond = cond != 0xEu && cond != 0xFu;

    if (cond == 0xFu) {
        arm_unconditional(s, addr, instr);
        return;
    }

    const u32 blk = (instr >> 25) & 7u;
    const u32 op = (instr >> 21) & 0xFu;

    if (blk == 5u) {
        arm_branch(s, addr, instr);
        return;
    }

    // Bits [27:24] == 1111 with cond != 1111 is SVC.
    if ((instr & 0x0F000000u) == 0x0F000000u) {
        s.s("svc");
        s.cond_suffix();
        s.c(' ');
        s.imm(instr & 0xFFFFFFu);
        return;
    }

    if (blk == 6u || blk == 7u) {
        arm_coprocessor(s, addr, instr);
        return;
    }
    if (blk == 4u) {
        arm_load_store_multiple(s, addr, instr);
        return;
    }
    if (blk == 2u || blk == 3u) {
        if (blk == 3u && (instr & 0x10u) != 0u) {
            arm_media(s, addr, instr);
            return;
        }
        arm_load_store(s, addr, instr, blk == 2u);
        return;
    }
    if (blk == 1u) {
        // Hints (ARM ARM A8.8.16): cond 0011 0010 0000 1111 0000 0000 oooo. They
        // share bits [27:20] with MSR (immediate) and used to print as
        // `teq rN, #imm`, which made the kernel boot loader's `sev` look like a
        // skipped data-processing instruction.
        if ((instr & 0x0FFF0F00u) == 0x03200F00u) {
            switch (instr & 0xFFu) {
                case 0u: s.s("nop"); break;
                case 1u: s.s("yield"); break;
                case 2u: s.s("wfe"); break;
                case 3u: s.s("wfi"); break;
                case 4u: s.s("sev"); break;
                case 0xF0u: s.s("dbg #0"); break;
                default:
                    s.s("hint #");
                    s.u(instr & 0xFFu);
                    break;
            }
            s.cond_suffix();
            return;
        }
        if (op >= 0xAu && ((instr >> 12) & 0xFu) == 0xFu) {
            arm_msr_imm(s, instr);
            return;
        }
        arm_data_processing(s, addr, instr, true);
        return;
    }

    const u32 ga = (instr >> 20) & 0xFFu;
    const u32 nib = instr & 0xF0u;
    if (nib == 0xB0u || nib == 0xD0u || nib == 0xF0u) { arm_extra_load_store(s, addr, instr); return; }
    if (nib == 0x90u) {
        if (is_arm_exclusive(instr)) { arm_exclusive(s, instr); return; }
        arm_mul(s, addr, instr);
        return;
    }
    if (ga >= 0x10u && ga <= 0x17u && (ga & 1u) == 0u && (nib == 0x50u || (nib & 0x90u) == 0x80u)) {
        arm_media(s, addr, instr);
        return;
    }
    if ((instr & 0x0FFF0FF0u) == 0x016F0F10u) { arm_clz(s, instr); return; }
    if (ga >= 0x10u && ga <= 0x17u && is_arm_misc_encoding(instr)) { arm_misc(s, addr, instr); return; }
    arm_data_processing(s, addr, instr, false);
}

// ---------------------------------------------------------------------------
// Thumb-16
// ---------------------------------------------------------------------------

void t16_shift(DState& s, u32 i) {
    const u32 op = (i >> 11) & 3u;
    const u32 imm5 = (i >> 6) & 0x1Fu;
    const int rm = static_cast<int>((i >> 3) & 7u);
    const int rd = static_cast<int>(i & 7u);
    if (op == 3u) {
        char tmp[8];
        std::snprintf(tmp, sizeof(tmp), "%04X", i);
        s.s(".hword 0x");
        s.s(tmp);
        return;
    }
    s.s(op == 0u ? "lsls " : (op == 1u ? "lsrs " : "asrs "));
    s.reg(rd); s.s(", "); s.reg(rm);
    if (op == 0u) {
        if (imm5 != 0) { s.s(", #"); s.u(imm5); }
    } else {
        s.s(", #");
        s.u(imm5 == 0 ? 32u : imm5);
    }
}

void t16_add_sub(DState& s, u32 i) {
    const bool immediate = (i & 0x400u) != 0;
    const bool sub = (i & 0x200u) != 0;
    const int rn = static_cast<int>((i >> 6) & 7u);
    const int rm = static_cast<int>((i >> 3) & 7u);
    const int rd = static_cast<int>(i & 7u);
    s.s(sub ? "subs " : "adds ");
    s.reg(rd); s.s(", "); s.reg(rm); s.s(", ");
    if (immediate) s.imm(static_cast<u32>(rn));
    else s.reg(rn);
}

void t16_mov_cmp(DState& s, u32 i) {
    const u32 op = (i >> 11) & 3u;
    const int rd = static_cast<int>((i >> 8) & 7u);
    const u32 imm8 = i & 0xFFu;
    switch (op) {
        case 0u: s.s("movs "); s.reg(rd); s.s(", "); s.imm(imm8); return;
        case 1u: s.s("cmp "); s.reg(rd); s.s(", "); s.imm(imm8); return;
        case 2u: s.s("adds "); s.reg(rd); s.s(", "); s.imm(imm8); return;
        default: s.s("subs "); s.reg(rd); s.s(", "); s.imm(imm8); return;
    }
}

void t16_alu(DState& s, u32 i) {
    const u32 op = (i >> 6) & 0xFu;
    const int rm = static_cast<int>((i >> 3) & 7u);
    const int rd = static_cast<int>(i & 7u);
    switch (op) {
        case 0x0u: s.s("ands "); break;
        case 0x1u: s.s("eors "); break;
        case 0x2u: s.s("lsls "); break;
        case 0x3u: s.s("lsrs "); break;
        case 0x4u: s.s("asrs "); break;
        case 0x5u: s.s("adcs "); break;
        case 0x6u: s.s("sbcs "); break;
        case 0x7u: s.s("rors "); break;
        case 0x8u: s.s("tst "); break;
        case 0x9u: s.s("rsbs "); break;
        case 0xAu: s.s("cmp "); break;
        case 0xBu: s.s("cmn "); break;
        case 0xCu: s.s("orrs "); break;
        case 0xDu: s.s("muls "); break;
        case 0xEu: s.s("bics "); break;
        default: s.s("mvns "); break;
    }
    s.reg(rd); s.s(", "); s.reg(rm);
    if (op == 0x9u) s.s(", #0x0");
}

void t16_special_data(DState& s, u32 i) {
    const u32 op = (i >> 8) & 3u;
    const int rm = static_cast<int>((i >> 3) & 0xFu);
    const int rd = static_cast<int>((i & 7u) | ((i >> 4) & 8u));
    switch (op) {
        case 0u: s.s("add "); s.reg(rd); s.s(", "); s.reg(rm); return;
        case 1u: s.s("cmp "); s.reg(rd); s.s(", "); s.reg(rm); return;
        case 2u: s.s("mov "); s.reg(rd); s.s(", "); s.reg(rm); return;
        default: s.s((i & 0x0080u) != 0 ? "blx " : "bx "); s.reg(rm); return;
    }
}

void t16_literal(DState& s, u32 addr, u32 i) {
    const int rd = static_cast<int>((i >> 8) & 7u);
    const u32 imm8 = static_cast<u32>(i & 0xFFu) * 4u;
    s.s("ldr "); s.reg(rd); s.s(", [pc, #"); s.u(imm8); s.c(']');
    s.s("  ; ");
    s.hex(((addr + 4u) & ~3u) + imm8);
}

void t16_ldr_str_reg(DState& s, u32 i) {
    const u32 op = (i >> 9) & 7u;
    const int rm = static_cast<int>((i >> 6) & 7u);
    const int rn = static_cast<int>((i >> 3) & 7u);
    const int rd = static_cast<int>(i & 7u);
    switch (op) {
        case 0u: s.s("str "); break;
        case 1u: s.s("strh "); break;
        case 2u: s.s("strb "); break;
        case 3u: s.s("ldrsb "); break;
        case 4u: s.s("ldr "); break;
        case 5u: s.s("ldrh "); break;
        case 6u: s.s("ldrb "); break;
        default: s.s("ldrsh "); break;
    }
    s.reg(rd); s.s(", ["); s.reg(rn); s.s(", "); s.reg(rm); s.c(']');
}

void t16_ldr_str_imm(DState& s, u32 i) {
    const bool byte_op = (i & 0x1000u) != 0;
    const bool load = (i & 0x0800u) != 0;
    const u32 imm5 = (i >> 6) & 0x1Fu;
    const int rn = static_cast<int>((i >> 3) & 7u);
    const int rd = static_cast<int>(i & 7u);
    s.s(load ? "ldr" : "str");
    if (byte_op) s.c('b');
    s.c(' ');
    s.reg(rd); s.s(", ["); s.reg(rn);
    if (imm5 != 0) { s.s(", #"); s.u(byte_op ? imm5 : imm5 * 4u); }
    s.c(']');
}

void t16_ldr_str_half_imm(DState& s, u32 i) {
    const bool load = (i & 0x0800u) != 0;
    const u32 imm5 = (i >> 6) & 0x1Fu;
    const int rn = static_cast<int>((i >> 3) & 7u);
    const int rd = static_cast<int>(i & 7u);
    s.s(load ? "ldrh " : "strh ");
    s.reg(rd); s.s(", ["); s.reg(rn);
    if (imm5 != 0) { s.s(", #"); s.u(imm5 * 2u); }
    s.c(']');
}

void t16_sp_rel(DState& s, u32 i) {
    const bool load = (i & 0x0800u) != 0;
    const int rd = static_cast<int>((i >> 8) & 7u);
    const u32 imm8 = static_cast<u32>(i & 0xFFu) * 4u;
    s.s(load ? "ldr " : "str ");
    s.reg(rd); s.s(", [sp");
    if (imm8 != 0) { s.s(", #"); s.u(imm8); }
    s.c(']');
}

void t16_adr(DState& s, u32 addr, u32 i) {
    const bool sp = (i & 0x0800u) != 0;
    const int rd = static_cast<int>((i >> 8) & 7u);
    const u32 imm8 = static_cast<u32>(i & 0xFFu) * 4u;
    if (sp) {
        s.s("add "); s.reg(rd); s.s(", sp, #"); s.u(imm8);
    } else {
        s.s("adr "); s.reg(rd); s.s(", #"); s.hex(((addr + 4u) & ~3u) + imm8);
    }
}

void t16_ldm_stm(DState& s, u32 i) {
    const bool load = (i & 0x0800u) != 0;
    const int rn = static_cast<int>((i >> 8) & 7u);
    const u32 list = i & 0xFFu;
    s.s(load ? "ldm" : "stm");
    if (load) s.s("ia");
    s.c(' ');
    s.reg(rn);
    if ((list & (1u << rn)) == 0) s.c('!');
    s.s(", ");
    s.reg_list(list);
}

void t16_cond_branch(DState& s, u32 addr, u32 i) {
    const u32 cond = (i >> 8) & 0xFu;
    if (cond == 0xFu) { s.s("svc #"); s.u(i & 0xFFu); return; }
    if (cond == 0xEu) { s.s("udf #"); s.u(i & 0xFFu); return; }
    s32 imm = static_cast<s32>(i & 0xFFu);
    if ((imm & 0x80) != 0) imm |= static_cast<s32>(0xFFFFFF00u);
    s.s("b");
    s.s(arm::condition_name(cond));
    s.s(" #");
    s.hex(addr + 4u + static_cast<u32>(imm << 1));
}

void t16_uncond_branch(DState& s, u32 addr, u32 i) {
    s32 imm = static_cast<s32>(i & 0x7FFu);
    if ((imm & 0x400) != 0) imm |= static_cast<s32>(0xFFFFF800u);
    s.s("b #");
    s.hex(addr + 4u + static_cast<u32>(imm << 1));
}

void t16_misc(DState& s, u32 addr, u32 i) {
    const u32 sub = (i >> 8) & 0xFu;

    if (sub == 0u) {
        const bool s2 = (i & 0x80u) != 0;
        const u32 imm = static_cast<u32>(i & 0x7Fu) * 4u;
        s.s(s2 ? "sub sp, #" : "add sp, #");
        s.u(imm);
        return;
    }
    if ((i & 0xF500u) == 0xB100u) {
        const bool nz = (i & 0x800u) != 0;
        const u32 high = (i >> 9) & 1u;
        const u32 imm5 = (i >> 3) & 0x1Fu;
        const u32 offset = ((high << 5) | imm5) << 1;
        s.s(nz ? "cbnz " : "cbz ");
        s.reg(static_cast<int>(i & 7u));
        s.s(", #");
        s.hex(addr + 4u + offset);
        return;
    }
    if ((i & 0xFF00u) == 0xB200u) {
        const u32 op2 = (i >> 6) & 3u;
        const int rm = static_cast<int>((i >> 3) & 7u);
        const int rd = static_cast<int>(i & 7u);
        s.s(op2 == 0u ? "sxth " : (op2 == 1u ? "sxtb " : (op2 == 2u ? "uxth " : "uxtb ")));
        s.reg(rd); s.s(", "); s.reg(rm);
        return;
    }
    if ((i & 0xFE00u) == 0xB400u) {
        u32 list = i & 0xFFu;
        if ((i & 0x100u) != 0) list |= 0x4000u;
        s.s("push "); s.reg_list(list);
        return;
    }
    if ((i & 0xFF00u) == 0xB600u) {
        if ((i & 0x00F0u) == 0x0060u) { s.s("cpsie i"); return; }
        if ((i & 0x00F0u) == 0x0070u) { s.s("cpsid i"); return; }
        if ((i & 0x00FFu) == 0x0002u) { s.s("setend be"); return; }
        if ((i & 0x00FFu) == 0x0000u) { s.s("setend le"); return; }
        s.s("cps");
        return;
    }
    if ((i & 0xFF00u) == 0xBA00u) {
        const u32 a = (i >> 6) & 3u;
        const int rm = static_cast<int>((i >> 3) & 7u);
        const int rd = static_cast<int>(i & 7u);
        s.s(a == 0u ? "rev " : (a == 1u ? "rev16 " : "revsh "));
        s.reg(rd); s.s(", "); s.reg(rm);
        return;
    }
    if ((i & 0xFE00u) == 0xBC00u) {
        u32 list = i & 0xFFu;
        if ((i & 0x100u) != 0) list |= 0x8000u;
        s.s("pop "); s.reg_list(list);
        return;
    }
    if ((i & 0xFF00u) == 0xBE00u) { s.s("bkpt #"); s.u(i & 0xFFu); return; }
    if ((i & 0xFF00u) == 0xBF00u) {
        const u32 mask = i & 0xFu;
        const u32 firstcond = (i >> 4) & 0xFu;
        if (mask == 0u) {
            switch (firstcond) {
                case 0u: s.s("nop"); return;
                case 1u: s.s("yield"); return;
                case 2u: s.s("wfe"); return;
                case 3u: s.s("wfi"); return;
                case 4u: s.s("sev"); return;
                default: s.s("hint #"); s.u(firstcond); return;
            }
        }
        s.s("it");
        s.s(arm::condition_name(firstcond));
        s.c(' ');
        int n = 0;
        for (int b = 3; b >= 0; --b) {
            if ((mask & (1u << b)) != 0) ++n;
        }
        for (int b = 0; b < n; ++b) s.c((mask & (1u << (3 - b))) != 0 ? 't' : 'e');
        return;
    }
    char tmp[8];
    std::snprintf(tmp, sizeof(tmp), "%04X", i);
    s.s(".hword 0x");
    s.s(tmp);
}

void thumb16(DState& s, u32 addr, u32 i) {
    const u32 top = (i >> 11) & 0x1Fu;
    switch (top) {
        case 0u: case 1u: case 2u: t16_shift(s, i); return;
        case 3u: t16_add_sub(s, i); return;
        case 4u: case 5u: case 6u: case 7u: t16_mov_cmp(s, i); return;
        case 8u:
            if ((i & 0x400u) != 0u) t16_special_data(s, i);
            else t16_alu(s, i);
            return;
        case 9u: t16_literal(s, addr, i); return;
        case 10u: case 11u: t16_ldr_str_reg(s, i); return;
        case 12u: case 13u: case 14u: case 15u: t16_ldr_str_imm(s, i); return;
        case 16u: case 17u: t16_ldr_str_half_imm(s, i); return;
        case 18u: case 19u: t16_sp_rel(s, i); return;
        case 20u: case 21u: t16_adr(s, addr, i); return;
        case 22u: case 23u: t16_misc(s, addr, i); return;
        case 24u: case 25u: t16_ldm_stm(s, i); return;
        case 26u: case 27u: t16_cond_branch(s, addr, i); return;
        case 28u: t16_uncond_branch(s, addr, i); return;
        default: {
            char tmp[8];
            std::snprintf(tmp, sizeof(tmp), "%04X", i);
            s.s(".hword 0x");
            s.s(tmp);
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// Thumb-32
// ---------------------------------------------------------------------------

void t32_coprocessor(DState& s, u32 addr, u32 instr) {
    const u32 cp = (instr >> 8) & 0xFu;
    if (cp == 10u || cp == 11u) {
        if ((instr & 0x0E000000u) == 0x0C000000u) {
            vfp_load_store(s, addr, instr);
            return;
        }
        vfp_data_processing(s, addr, instr);
        return;
    }
    if (cp == 15u) {
        const bool load = (instr & (1u << 20)) != 0;
        const u32 o1 = (instr >> 21) & 7u;
        const u32 o2 = (instr >> 5) & 7u;
        s.s(load ? "mrc" : "mcr");
        s.s(".w p15, #"); s.u(o1);
        s.s(", r"); s.u((instr >> 12) & 0xFu);
        s.s(", c"); s.u((instr >> 16) & 0xFu);
        s.s(", c"); s.u(instr & 0xFu);
        if (o2 != 0u) { s.s(", #"); s.u(o2); }
        return;
    }
    char tmp[16];
    std::snprintf(tmp, sizeof(tmp), "%08X", instr);
    s.s(".word 0x");
    s.s(tmp);
}

void t32_data_processing_shifted(DState& s, u32 instr) {
    const u32 hw1 = (instr >> 16) & 0xFFFFu;
    const u32 hw2 = instr & 0xFFFFu;
    const u32 opc = (hw1 >> 5) & 0xFu;
    const bool sf = (hw1 & 0x10u) != 0u;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rd = static_cast<int>((hw2 >> 8) & 0xFu);
    const int rm = static_cast<int>(hw2 & 0xFu);
    const int type = static_cast<int>((hw2 >> 4) & 3u);
    int amount = static_cast<int>(((hw2 >> 12) & 7u) << 2 | ((hw2 >> 6) & 3u));

    const char* nm;
    switch (opc) {
        case 0x0u: nm = "and"; break;
        case 0x1u: nm = "bic"; break;
        case 0x2u: nm = rn == 15 ? "mov" : "orr"; break;
        case 0x3u: nm = rn == 15 ? "mvn" : "orn"; break;
        case 0x4u: nm = rn == 15 ? "mov" : "eor"; break;
        case 0x6u: nm = "pkh"; break;
        case 0x8u: nm = "add"; break;
        case 0xAu: nm = "adc"; break;
        case 0xBu: nm = "sbc"; break;
        case 0xDu: nm = "sub"; break;
        case 0xEu: nm = "rsb"; break;
        default: {
            char tmp[16];
            std::snprintf(tmp, sizeof(tmp), "%08X", instr);
            s.s(".word 0x");
            s.s(tmp);
            return;
        }
    }

    if (opc == 0x6u) {
        const bool tb = (hw2 & 0x20u) != 0u;
        s.s(tb ? "pkhtb " : "pkhbt ");
        s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.reg(rm);
        if (tb) {
            const int a = amount == 0 ? 32 : amount;
            s.s(", asr #"); s.u(static_cast<u32>(a));
        } else if (amount != 0) {
            s.s(", lsl #"); s.u(static_cast<u32>(amount));
        }
        return;
    }

    s.s(nm);
    if (!sf) s.s(".w");
    s.c(' ');
    s.reg(rd); s.s(", ");
    if (!(rn == 15 && (opc == 2u || opc == 3u || opc == 4u))) {
        s.reg(rn);
        s.s(", ");
    }
    s.reg(rm);
    if (type == 0 && amount == 0) return;
    if (type != 0 && amount == 0) amount = 32;
    s.shift(type, amount);
}

void t32_data_processing_modified(DState& s, u32 instr) {
    const u32 hw1 = (instr >> 16) & 0xFFFFu;
    const u32 hw2 = instr & 0xFFFFu;
    const u32 i = (hw1 >> 10) & 1u;
    const u32 opc = (hw1 >> 5) & 0xFu;
    const bool sf = (hw1 & 0x10u) != 0u;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rd = static_cast<int>((hw2 >> 8) & 0xFu);
    const u32 imm12v = (i << 11) | (((hw2 >> 12) & 7u) << 8) | (hw2 & 0xFFu);
    const u32 imm = arm::thumb_expand_imm(imm12v);

    const bool alias = rd == 15 && (opc == 0x0u || opc == 0x4u || opc == 0x8u || opc == 0xDu);
    const char* nm;
    switch (opc) {
        case 0x0u: nm = alias ? "tst" : "and"; break;
        case 0x1u: nm = "bic"; break;
        case 0x2u: nm = rn == 15 ? "mov" : "orr"; break;
        case 0x3u: nm = rn == 15 ? "mvn" : "orn"; break;
        case 0x4u: nm = alias ? "teq" : "eor"; break;
        case 0x8u: nm = alias ? "cmn" : "add"; break;
        case 0xAu: nm = "adc"; break;
        case 0xBu: nm = "sbc"; break;
        case 0xDu: nm = alias ? "cmp" : "sub"; break;
        case 0xEu: nm = "rsb"; break;
        default: {
            char tmp[16];
            std::snprintf(tmp, sizeof(tmp), "%08X", instr);
            s.s(".word 0x");
            s.s(tmp);
            return;
        }
    }
    s.s(nm);
    if (alias) {
        s.s(".w ");
        s.reg(rn); s.s(", "); s.imm(imm);
        return;
    }
    if (sf) s.c('s');
    s.s(".w ");
    s.reg(rd); s.s(", ");
    if (!(rn == 15 && (opc == 2u || opc == 4u || opc == 0u))) {
        s.reg(rn);
        s.s(", ");
    }
    s.imm(imm);
}

void t32_data_processing_plain(DState& s, u32 instr) {
    const u32 hw1 = (instr >> 16) & 0xFFFFu;
    const u32 hw2 = instr & 0xFFFFu;
    const u32 i = (hw1 >> 10) & 1u;
    const u32 opc = (hw1 >> 4) & 0x1Fu;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rd = static_cast<int>((hw2 >> 8) & 0xFu);
    const u32 imm3v = (hw2 >> 12) & 7u;
    const u32 imm8 = hw2 & 0xFFu;
    const u32 imm12v = (i << 11) | (imm3v << 8) | imm8;
    const u32 imm16 = ((hw1 & 0xFu) << 12) | (i << 11) | (imm3v << 8) | imm8;
    const int lsb = static_cast<int>(imm3v << 2 | ((hw2 >> 6) & 3u));
    const int shift_type = static_cast<int>((hw2 >> 4) & 3u);

    switch (opc) {
        case 0x00u: s.s("addw "); s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.imm(imm12v); return;
        case 0x04u: s.s("movw "); s.reg(rd); s.s(", "); s.imm(imm16); return;
        case 0x0Au: s.s("subw "); s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.imm(imm12v); return;
        case 0x0Cu: s.s("movt "); s.reg(rd); s.s(", "); s.hex(imm16); return;
        case 0x10u:
            s.s("ssat "); s.reg(rd); s.s(", #"); s.u(hw1 & 0xFu); s.s(", "); s.reg(rn);
            if (lsb != 0) { s.s(shift_type == 2 ? ", asr #" : ", lsl #"); s.u(static_cast<u32>(lsb)); }
            return;
        case 0x12u:
            s.s("ssat16 "); s.reg(rd); s.s(", #"); s.u(hw1 & 0xFu); s.s(", "); s.reg(rn);
            return;
        case 0x14u:
            s.s("sbfx "); s.reg(rd); s.s(", "); s.reg(rn);
            s.s(", #"); s.u(static_cast<u32>(lsb));
            s.s(", #"); s.u((hw2 >> 12) + 1u);
            return;
        case 0x16u:
            if (rn == 15) {
                s.s("bfc "); s.reg(rd); s.s(", #"); s.u(static_cast<u32>(lsb));
                s.s(", #"); s.u((hw2 >> 12) + 1u);
            } else {
                s.s("bfi "); s.reg(rd); s.s(", "); s.reg(rn);
                s.s(", #"); s.u(static_cast<u32>(lsb));
                s.s(", #"); s.u((hw2 >> 12) + 1u);
            }
            return;
        case 0x18u:
            s.s("usat "); s.reg(rd); s.s(", #"); s.u(hw1 & 0xFu); s.s(", "); s.reg(rn);
            if (lsb != 0) { s.s(shift_type == 2 ? ", asr #" : ", lsl #"); s.u(static_cast<u32>(lsb)); }
            return;
        case 0x1Au:
            s.s("usat16 "); s.reg(rd); s.s(", #"); s.u(hw1 & 0xFu); s.s(", "); s.reg(rn);
            return;
        case 0x1Cu:
            s.s("ubfx "); s.reg(rd); s.s(", "); s.reg(rn);
            s.s(", #"); s.u(static_cast<u32>(lsb));
            s.s(", #"); s.u((hw2 >> 12) + 1u);
            return;
        default: {
            char tmp[16];
            std::snprintf(tmp, sizeof(tmp), "%08X", instr);
            s.s(".word 0x");
            s.s(tmp);
            return;
        }
    }
}

void t32_load_store_dual(DState& s, u32 addr, u32 instr) {
    (void)addr;
    const u32 hw1 = (instr >> 16) & 0xFFFFu;
    const u32 hw2 = instr & 0xFFFFu;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rt = static_cast<int>((hw2 >> 12) & 0xFu);
    const bool db = (hw1 & 0x100u) != 0u;
    const bool u = (hw1 & 0x080u) != 0u;
    const bool fixed6 = (hw1 & 0x040u) != 0u;
    const bool w = (hw1 & 0x020u) != 0u;
    const bool l = (hw1 & 0x010u) != 0u;

    if (!fixed6) {
        const u32 list = hw2;
        if (l && w && rn == 13) s.s("pop.w ");
        else if (!l && w && rn == 13) s.s("push.w ");
        else {
            s.s(l ? "ldm" : "stm");
            s.s(db ? "db " : "ia ");
            s.reg(rn);
            if (w) s.c('!');
            s.s(", ");
        }
        s.reg_list(list);
        return;
    }

    if (hw1 == 0xE8C0u || hw1 == 0xE8D0u) {
        s.s(hw1 == 0xE8C0u ? "strexb " : "ldrexb ");
        if (hw1 == 0xE8C0u) {
            s.reg(static_cast<int>((hw2 >> 8) & 0xFu)); s.s(", "); s.reg(rt); s.s(", [");
            s.reg(rn); s.c(']');
        } else {
            s.reg(rt); s.s(", ["); s.reg(rn); s.c(']');
        }
        return;
    }

    if (!u && !db && !w) {
        const u32 imm = (hw2 & 0xFFu) * 4u;
        if (!l) {
            s.s("strex "); s.reg(static_cast<int>((hw2 >> 8) & 0xFu)); s.s(", "); s.reg(rt);
            s.s(", ["); s.reg(rn);
            if (imm != 0) { s.s(", #"); s.u(imm); }
            s.c(']');
        } else {
            s.s("ldrex "); s.reg(rt); s.s(", ["); s.reg(rn);
            if (imm != 0) { s.s(", #"); s.u(imm); }
            s.c(']');
        }
        return;
    }

    const int rt2 = static_cast<int>((hw2 >> 8) & 0xFu);
    const u32 imm8 = (hw2 & 0xFFu) * 4u;
    s.s(l ? "ldrd " : "strd ");
    s.reg(rt); s.s(", "); s.reg(rt2); s.s(", [");
    s.reg(rn);
    if (db) {
        s.s(", ");
        if (!u) s.c('-');
        s.imm(imm8);
        s.c(']');
        if (w) s.c('!');
    } else {
        s.c(']');
        s.s(", ");
        if (!u) s.c('-');
        s.imm(imm8);
    }
}

void t32_load_store_single(DState& s, u32 addr, u32 instr) {    (void)addr;
    const u32 hw1 = (instr >> 16) & 0xFFFFu;
    const u32 hw2 = instr & 0xFFFFu;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rt = static_cast<int>((hw2 >> 12) & 0xFu);
    const bool signed_space = (hw1 & 0x100u) != 0u;
    const bool imm12_form = (hw1 & 0x80u) != 0u;
    const bool is_word = (hw1 & 0x40u) != 0u;
    const bool is_half = (hw1 & 0x20u) != 0u;
    const bool l = (hw1 & 0x10u) != 0u;
    // hw2[11:6] == 0 marks the register-offset form; hw2[11] alone does not,
    // because in the 12-bit immediate forms hw2[11:0] is the offset itself.
    const bool register_form = (hw2 & 0x0FC0u) == 0u;

    if (signed_space) {
        const u32 size_sel = hw1 & 0x30u;
        if (size_sel != 0x10u && size_sel != 0x30u) {
            char tmp[16];
            std::snprintf(tmp, sizeof(tmp), "%08X", instr);
            s.s(".word 0x");
            s.s(tmp);
            return;
        }
        const bool half = size_sel == 0x30u;
        s.s(half ? "ldrsh.w " : "ldrsb.w ");
        s.reg(rt); s.s(", [");
        s.reg(rn);
        if (imm12_form) {
            s.s(", #"); s.u(hw2 & 0xFFFu); s.c(']');
        } else if (register_form) {
            s.s(", ");
            s.reg(static_cast<int>(hw2 & 0xFu));
            const u32 shift = (hw2 >> 4) & 3u;
            if (shift != 0u) { s.s(", lsl #"); s.u(shift); }
            s.c(']');
        } else {
            const bool p = (hw2 & 0x400u) != 0u;
            const bool u = (hw2 & 0x200u) != 0u;
            const bool w = (hw2 & 0x100u) != 0u;
            const u32 imm8 = hw2 & 0xFFu;
            if (p) {
                s.s(", ");
                if (!u) s.c('-');
                s.imm(imm8); s.c(']');
                if (w) s.c('!');
            } else {
                s.c(']');
                s.s(", ");
                if (!u) s.c('-');
                s.imm(imm8);
            }
        }
        return;
    }

    if (l && rt == 15) { s.s("pld"); return; }

    s.s(l ? "ldr" : "str");
    s.s(is_word ? "" : (is_half ? "h" : "b"));
    s.s(".w ");
    s.reg(rt); s.s(", [");
    s.reg(rn);
    if (imm12_form) {
        s.s(", #"); s.u(hw2 & 0xFFFu); s.c(']');
        return;
    }
    if (register_form) {
        s.s(", ");
        s.reg(static_cast<int>(hw2 & 0xFu));
        const u32 shift = (hw2 >> 4) & 3u;
        if (shift != 0u) { s.s(", lsl #"); s.u(shift); }
        s.c(']');
        return;
    }
    {
        const bool p = (hw2 & 0x400u) != 0u;
        const bool u = (hw2 & 0x200u) != 0u;
        const bool w = (hw2 & 0x100u) != 0u;
        const u32 imm8 = hw2 & 0xFFu;
        if (p) {
            s.s(", ");
            if (!u) s.c('-');
            s.imm(imm8); s.c(']');
            if (w) s.c('!');
        } else {
            s.c(']');
            s.s(", ");
            if (!u) s.c('-');
            s.imm(imm8);
        }
    }
}

void t32_misc(DState& s, u32 addr, u32 instr);

void t32_branch_misc(DState& s, u32 addr, u32 instr) {
    const u32 hw1 = (instr >> 16) & 0xFFFFu;
    const u32 hw2 = instr & 0xFFFFu;
    const u32 sv = (hw1 >> 10) & 1u;
    const u32 j1 = (hw2 >> 13) & 1u;
    const u32 j2 = (hw2 >> 11) & 1u;
    const u32 imm11 = hw2 & 0x7FFu;

    if ((hw2 & 0x4000u) == 0u) {
        if ((hw2 & 0x1000u) != 0u) {
            const u32 imm10 = hw1 & 0x3FFu;
            const u32 i1 = (~(j1 ^ sv)) & 1u;
            const u32 i2 = (~(j2 ^ sv)) & 1u;
            const s32 off = arm::sign_extend32((sv << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1), 25);
            s.s("b.w #");
            s.hex(addr + 4u + static_cast<u32>(off));
            return;
        }
        const u32 cond = (hw1 >> 6) & 0xFu;
        if (cond <= 0xDu) {
            const u32 imm6 = hw1 & 0x3Fu;
            const s32 off = arm::sign_extend32((sv << 20) | (j2 << 19) | (j1 << 18) | (imm6 << 12) | (imm11 << 1), 21);
            s.s("b");
            s.s(arm::condition_name(cond));
            s.s(".w #");
            s.hex(addr + 4u + static_cast<u32>(off));
            return;
        }
        t32_misc(s, addr, instr);
        return;
    }

    const u32 imm10 = hw1 & 0x3FFu;
    const u32 i1 = (~(j1 ^ sv)) & 1u;
    const u32 i2 = (~(j2 ^ sv)) & 1u;
    if ((hw2 & 0x1000u) != 0u) {
        const s32 off = arm::sign_extend32((sv << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1), 25);
        s.s("bl #");
        s.hex(addr + 4u + static_cast<u32>(off));
        return;
    }
    const u32 imm10l = (hw2 >> 1) & 0x3FFu;
    const s32 offx = arm::sign_extend32((sv << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm10l << 2), 25);
    s.s("blx #");
    s.hex(((addr + 4u) & ~3u) + static_cast<u32>(offx));
}

void t32_multiply(DState& s, u32 instr) {
    const u32 hw1 = (instr >> 16) & 0xFFFFu;
    const u32 hw2 = instr & 0xFFFFu;
    const u32 op1 = (hw1 >> 4) & 0xFu;
    const int rn = static_cast<int>(hw1 & 0xFu);
    const int rd = static_cast<int>((hw2 >> 8) & 0xFu);
    const int ra = static_cast<int>((hw2 >> 12) & 0xFu);
    const int rm = static_cast<int>(hw2 & 0xFu);
    const u32 op2 = (hw2 >> 4) & 0xFu;

    if (((hw1 >> 8) & 0xFu) == 0xAu) {
        static const char* names[6] = {"add16", "asx", "sax", "sub16", "add8", "sub8"};
        const char* base = names[(op2 >> 1) & 7u];
        if (op1 >= 4u) s.s("u");
        else if ((op1 & 3u) == 2u) s.s("q");
        else if ((op1 & 3u) == 3u) s.s("sh");
        else s.s("s");
        s.s(base);
        s.c(' ');
        s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.reg(rm);
        return;
    }

    if (op1 == 0u && op2 == 0u) {
        if (ra == 15) {
            s.s("mul "); s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.reg(rm);
        } else {
            s.s("mla "); s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.reg(rm); s.s(", "); s.reg(ra);
        }
        return;
    }
    if (op1 == 1u && op2 == 0u) {
        s.s("mls "); s.reg(rd); s.s(", "); s.reg(rn); s.s(", "); s.reg(rm); s.s(", "); s.reg(ra);
        return;
    }
    char tmp[16];
    std::snprintf(tmp, sizeof(tmp), "%08X", instr);
    s.s(".word 0x");
    s.s(tmp);
}

void t32_misc(DState& s, u32 addr, u32 instr) {
    (void)addr;
    const u32 hw1 = (instr >> 16) & 0xFFFFu;
    const u32 hw2 = instr & 0xFFFFu;
    const int rd = static_cast<int>((hw2 >> 8) & 0xFu);
    const int rm = static_cast<int>(hw2 & 0xFu);

    if (hw1 == 0xF3AFu && (hw2 & 0x8000u) != 0u) {
        switch (hw2 & 0xFFu) {
            case 0x00u: s.s("nop.w"); return;
            case 0x01u: s.s("yield.w"); return;
            case 0x02u: s.s("wfe.w"); return;
            case 0x03u: s.s("wfi.w"); return;
            case 0x04u: s.s("sev.w"); return;
            default: s.s("hint.w #"); s.u(hw2 & 0xFFu); return;
        }
    }
    if (hw1 == 0xF3BFu && (hw2 & 0xFF00u) == 0x8F00u) {
        s.s((hw2 & 0xF0u) == 0x10u ? "clrex"
            : (hw2 & 0xF0u) == 0x40u ? "dsb sy"
            : (hw2 & 0xF0u) == 0x50u ? "dmb sy"
                                     : "isb sy");
        return;
    }
    if (hw1 == 0xF3EFu && (hw2 & 0xF000u) == 0x8000u) {
        const bool spsr = (hw2 & 0x100u) != 0u;
        s.s("mrs "); s.reg(rd); s.s(", ");
        s.s(spsr ? "spsr" : "cpsr");
        return;
    }
    if ((hw1 & 0xFFF0u) == 0xF380u && (hw2 & 0xF000u) == 0x8000u) {
        const u32 sysm = (hw2 >> 8) & 0xFu;
        const bool spsr = (hw2 & 0x100u) != 0u;
        s.s("msr ");
        s.s(spsr ? "spsr_" : "cpsr_");
        if (sysm == 0u) s.c('c');
        else if (sysm == 1u) s.c('x');
        else if (sysm == 2u) s.c('s');
        else if (sysm == 3u) s.c('f');
        else s.s("fsxc");
        s.s(", "); s.reg(rm);
        return;
    }
    if ((hw1 & 0xFFF0u) == 0xF7F0u) { s.s("svc.w #"); s.u(hw2 & 0xFFu); return; }
    if ((hw1 & 0xFFE0u) == 0xF7E0u) { s.s("hvc.w #"); s.u(hw2 & 0xFFu); return; }
    if (hw1 == 0xF3AFu && (hw2 & 0xFFF0u) == 0x80F0u) { s.s("dbg #"); s.u(hw2 & 0xFu); return; }

    char tmp[16];
    std::snprintf(tmp, sizeof(tmp), "%08X", instr);
    s.s(".word 0x");
    s.s(tmp);
}

void thumb32(DState& s, u32 addr, u32 instr) {
    const u32 hw1 = (instr >> 16) & 0xFFFFu;
    const u32 t1 = (hw1 >> 11) & 3u;
    const u32 top = (hw1 >> 4) & 0x7Fu;

    if (t1 == 1u) {
        if ((top & 0x60u) == 0x00u) { t32_load_store_dual(s, addr, instr); return; }
        if ((top & 0x60u) == 0x20u) { t32_data_processing_shifted(s, instr); return; }
        t32_coprocessor(s, addr, instr);
        return;
    }
    if (t1 == 2u) {
        if ((instr & 0x8000u) != 0u) { t32_branch_misc(s, addr, instr); return; }
        if ((hw1 & 0x200u) == 0u) { t32_data_processing_modified(s, instr); return; }
        t32_data_processing_plain(s, instr);
        return;
    }
    if ((top & 0x60u) == 0x20u) { t32_multiply(s, instr); return; }
    t32_load_store_single(s, addr, instr);
}

void thumb_decode(DState& s, u32 addr, unsigned& length) {
    const u32 hw1 = fetch_half(s, addr);
    if ((hw1 & 0xF800u) >= 0xE800u) {
        const u32 hw2 = fetch_half(s, addr + 2u);
        const u32 instr = (hw1 << 16) | hw2;
        s.insn = instr;
        length = 4;
        thumb32(s, addr, instr);
        return;
    }
    s.insn = hw1;
    length = 2;
    thumb16(s, addr, hw1);
}

}  // namespace

std::string arm_disassemble(Bus& bus, u32 address, bool thumb, unsigned& length) {
    DState& s = g_state;
    s.reset();
    s.bus = &bus;
    s.thumb = thumb;
    s.address = address;
    length = thumb ? 2u : 4u;

    if (thumb) thumb_decode(s, address, length);
    else arm_decode(s, address, length);

    if (s.buf.empty()) {
        char tmp[16];
        std::snprintf(tmp, sizeof(tmp), "%08X", s.insn);
        s.buf = ".word 0x";
        s.buf += tmp;
    }
    if (length != 2u && length != 4u) length = thumb ? 2u : 4u;
    s.bus = nullptr;
    return s.buf;
}

std::string arm_mnemonic(Bus& bus, u32 address, bool thumb) {
    unsigned length = 0;
    const std::string text = arm_disassemble(bus, address, thumb, length);
    const size_t space = text.find(' ');
    return space == std::string::npos ? text : text.substr(0, space);
}

}  // namespace zlb
