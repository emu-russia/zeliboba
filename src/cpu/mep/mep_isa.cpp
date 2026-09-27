// zeliboba - MeP-c5 opcode table, decoder and objdump compatible formatter.
#include "cpu/mep/mep_isa.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

namespace zlb::mep {
namespace {

// A concise table entry constructor: the operand list is brace initialised and
// the count is derived so that a hand edit cannot desynchronise the two.
struct TableEntry {
    const char* mnem;
    const char* fmt;
    u32 mask;
    u32 value;
    u8 len;
    Op op;
    std::array<Opnd, 3> ops;
};

/// Number of live operands in a padded operand list.  Only the first operand
/// may use `Field::Const` with a non zero value (the "$0" of `slt3`), so the
/// first padding slot is always recognisable.
u8 count_operands(const std::array<Opnd, 3>& ops) {
    for (u8 i = 0; i < ops.size(); ++i) {
        if (ops[i].field == Field::Const && ops[i].constant == 0) return i;
    }
    return static_cast<u8>(ops.size());
}

/// Number of set bits in a mask; used to order the table by specificity.
int mask_bits(u32 mask) {
    int bits = 0;
    while (mask != 0) {
        bits += static_cast<int>(mask & 1u);
        mask >>= 1;
    }
    return bits;
}

/// Build the table once: the encoding source is a compile time constant list so
/// there is no file parsing and no ordering ambiguity at run time.
const std::array<Insn, 225>& instruction_table() {
    static std::array<Insn, 225> table = {{
#define I(mnem, fmt, mask, value, len, op, ...) \
    Insn{mnem, fmt, mask, value, static_cast<u8>(len), op, __VA_ARGS__}
#include "cpu/mep/mep_isa_table.inc"
#undef I
    }};

    static bool prepared = false;
    if (!prepared) {
        prepared = true;
        for (Insn& insn : table) insn.op_count = count_operands(insn.ops);

        // Two entries in the CGEN derived table describe their register operand
        // with the wrong field: the encodings are (+ MAJ_1 rn (f-sub4 14/15)) so
        // the register lives in bits 4..7 (the `rn` field), but the operand list
        // says `rma` (bits 8..11), which the mask 0xFF0F forces to zero.  Fix
        // them up so that `jmp $rm` / `jsr $rm` print the register they encode.
        for (Insn& insn : table) {
            if (insn.op == Op::Jmp || insn.op == Op::Jsr) insn.ops[0].field = Field::Rn;
        }

        // The CGEN lookup order groups by major opcode and takes the most
        // specific mask first; the generated list is in architecture manual
        // order, so reproduce the ordering here.  Entries that are equally
        // specific keep their original relative order (stable sort).
        std::stable_sort(table.begin(), table.end(), [](const Insn& a, const Insn& b) {
            const u32 major_a = (a.value >> 12) & 0xFu;
            const u32 major_b = (b.value >> 12) & 0xFu;
            if (major_a != major_b) return major_a < major_b;
            return mask_bits(a.mask) > mask_bits(b.mask);
        });
    }
    return table;
}

// ---------------------------------------------------------------------------
// Field extraction
// ---------------------------------------------------------------------------

u32 raw_impl(u32 word, int start, int length) {
    if (length <= 0 || length > 32 || start < 0) return 0;
    const int base = start & ~15;
    const int local = start - base;
    const u32 chunk = (base >= 32) ? 0u : ((word >> base) & 0xFFFFu);
    const int shift = 16 - local - length;
    if (shift < 0) return 0;
    return (chunk >> shift) & ((1u << length) - 1u);
}

/// Sign extend the low `bits` bits of `value`.
u32 sext_impl(u32 value, int bits) {
    if (bits <= 0 || bits >= 32) return value;
    const u32 m = 1u << (bits - 1);
    return (value ^ m) - m;
}

/// Branch targets are PC relative *to the address of the branch*, which is what
/// the annotated listings show ("bsr 0x5c4fe" at 0x5c040).
u32 rel(u32 word, u32 pc, int start, int length, int shift) {
    return pc + (sext_impl(raw_impl(word, start, length), length) << shift);
}

}  // namespace

u32 raw(u32 word, int start, int length) { return raw_impl(word, start, length); }

u32 sext(u32 value, int bits) { return sext_impl(value, bits); }

u32 field_value(Field field, u32 word, u32 pc) {
    switch (field) {
        case Field::Const: return 0;
        case Field::Rn: return raw_impl(word, 4, 4);
        case Field::Rm: return raw_impl(word, 8, 4);
        case Field::Rl: return raw_impl(word, 12, 4);
        case Field::Rn3: return raw_impl(word, 5, 3);
        case Field::F8s8a2: return rel(word, pc, 8, 7, 1);
        case Field::F12s4a2: return rel(word, pc, 4, 11, 1);
        case Field::F17s16a2: return rel(word, pc, 16, 16, 1);
        case Field::F24s5a2n: {
            const u32 hi = raw_impl(word, 16, 16);
            const u32 lo = raw_impl(word, 5, 7);
            u32 disp = (hi << 8) | (lo << 1);
            if ((disp & 0x800000u) != 0) disp |= 0xFF000000u;
            return pc + disp;
        }
        case Field::F24u5a2n: {
            const u32 hi = raw_impl(word, 16, 16);
            const u32 lo = raw_impl(word, 5, 7);
            return (hi << 8) | (lo << 1);
        }
        case Field::F16s16: return sext_impl(raw_impl(word, 16, 16), 16);
        case Field::F16u16: return raw_impl(word, 16, 16);
        case Field::F2u6: return sext_impl(raw_impl(word, 6, 2), 2);
        case Field::F2u10: return raw_impl(word, 10, 2);
        case Field::F6s8: return sext_impl(raw_impl(word, 8, 6), 6);
        case Field::F8s8: return sext_impl(raw_impl(word, 8, 8), 8);
        case Field::F24u8a4n: {
            const u32 hi = raw_impl(word, 16, 16);
            const u32 lo = raw_impl(word, 8, 6);
            return (hi << 8) | (lo << 2);
        }
        case Field::F24u4n: {
            const u32 hi = raw_impl(word, 4, 8);
            const u32 lo = raw_impl(word, 16, 16);
            return (hi << 16) | lo;
        }
        case Field::FCallnum:
            return (raw_impl(word, 5, 1) << 3) | (raw_impl(word, 6, 1) << 2) |
                   (raw_impl(word, 7, 1) << 1) | raw_impl(word, 11, 1);
        case Field::F3u5: return raw_impl(word, 5, 3);
        case Field::F4u8: return raw_impl(word, 8, 4);
        case Field::F5u8: return raw_impl(word, 8, 5);
        case Field::F7u9: return raw_impl(word, 9, 7);
        case Field::F7u9a2: return raw_impl(word, 9, 6) << 1;
        case Field::F7u9a4: return raw_impl(word, 9, 5) << 2;
        case Field::F24u8n: {
            const u32 hi = raw_impl(word, 16, 16);
            const u32 lo = raw_impl(word, 8, 8);
            return (hi << 8) | lo;
        }
        case Field::F5u24: return raw_impl(word, 24, 5);
        case Field::FCdisp10: {
            // 10 bit coprocessor displacement with an odd sign encoding: the
            // CGEN description XORs bit 7 into the top bits before extending.
            const u32 value = raw_impl(word, 22, 10);
            const u32 t = ((value & 0x80u) != 0) ? (value ^ 0x300u) : value;
            const int c = ((t & 0x200u) != 0) ? static_cast<int>(t) - 0x400 : static_cast<int>(t);
            if ((c & 0x200) != 0) return static_cast<u32>((c & 0x3FF) - 0x400);
            return static_cast<u32>(c & 0x3FF);
        }
        case Field::FCsrn: return (raw_impl(word, 15, 1) << 4) | raw_impl(word, 8, 4);
        case Field::FCrn: return raw_impl(word, 4, 4);
        case Field::FCrnx: return (raw_impl(word, 28, 1) << 4) | raw_impl(word, 4, 4);
        case Field::FCcrn: return (raw_impl(word, 28, 2) << 4) | raw_impl(word, 4, 4);
        case Field::FRl5: return raw_impl(word, 20, 4);
        case Field::F12s20: return sext_impl(raw_impl(word, 20, 12), 12);
        case Field::FC5Rm: return (raw_impl(word, 8, 4) << 16) | raw_impl(word, 16, 16);
        case Field::FC5Rnm: return (raw_impl(word, 4, 8) << 16) | raw_impl(word, 16, 16);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Decode
// ---------------------------------------------------------------------------

const Insn* decode(u32 word) {
    static const Insn unknown{"*unknown*", "*unknown*", 0, 0xFFFFFFFFu, 4, Op::None, {}, 0};

    // The generated table is already ordered most-specific-first, so the first
    // matching entry is the right one.
    for (const Insn& insn : instruction_table()) {
        if (insn.matches(word)) return &insn;
    }
    return &unknown;
}

// ---------------------------------------------------------------------------
// Formatting
// ---------------------------------------------------------------------------

const char* register_name(int index) {
    static const char* const names[16] = {"$0",  "$1",  "$2",  "$3",  "$4",  "$5",  "$6",  "$7",
                                          "$8",  "$9",  "$10", "$11", "$12", "$tp", "$gp", "$sp"};
    if (index < 0 || index > 15) return "$?";
    return names[index];
}

const char* control_register_name(int index) {
    static const char* const names[32] = {
        "$pc",  "$lp",  "$sar", "$???", "$rpb", "$rpe", "$rpc", "$hi",
        "$lo",  "$???", "$???", "$???", "$mb0", "$me0", "$mb1", "$me1",
        "$psw", "$id",  "$tmp", "$epc", "$exc", "$cfg", "$vid", "$npc",
        "$dbg", "$depc", "$opt", "$rcfg", "$ccfg", "$???", "$???", "$???"};
    if (index < 0 || index > 31) return "$???";
    return names[index];
}

namespace {

/// Append a rendered operand.  `op` remembers whether the field was signed so
/// that `add $3,-1` and `add3 $gp,$gp,-5504` print the way objdump prints them.
void append_operand(std::string& out, const Opnd& opnd, u32 value) {
    char buffer[64];
    switch (opnd.print) {
        case Print::Reg:
            out += register_name(static_cast<int>(value & 0xF));
            break;
        case Print::CpReg:
            std::snprintf(buffer, sizeof(buffer), "$c%u", value);
            out += buffer;
            break;
        case Print::Csrn:
            out += control_register_name(static_cast<int>(value));
            break;
        case Print::SDec:
        case Print::Cdisp10:
            std::snprintf(buffer, sizeof(buffer), "%d", static_cast<int>(value));
            out += buffer;
            break;
        default:
            std::snprintf(buffer, sizeof(buffer), "0x%x", value);
            out += buffer;
            break;
    }
}

}  // namespace

std::string format(const Insn& insn, u32 word, u32 pc) {
    // "mov $0,$0" is the architectural nop; objdump prints it as such and the
    // annotated listings rely on it.
    if (insn.op == Op::Mov && raw_impl(word, 4, 4) == 0 && raw_impl(word, 8, 4) == 0) return "nop";
    if (insn.op_count == 0 || std::strchr(insn.fmt, '{') == nullptr) return insn.fmt;

    std::string out;
    out.reserve(48);
    const char* cursor = insn.fmt;
    for (u8 i = 0; i < insn.op_count; ++i) {
        const char* brace = std::strchr(cursor, '{');
        if (brace == nullptr) {
            out += cursor;
            cursor = nullptr;
            break;
        }
        out.append(cursor, static_cast<size_t>(brace - cursor));
        const Opnd& opnd = insn.ops[i];
        const u32 value =
            (opnd.field == Field::Const) ? opnd.constant : field_value(opnd.field, word, pc);
        append_operand(out, opnd, value);
        cursor = brace + 3;  // skip "{N}"
    }
    if (cursor != nullptr) out += cursor;
    return out;
}

}  // namespace zlb::mep
