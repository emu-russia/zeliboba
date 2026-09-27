// zeliboba - RL78 instruction decoder (see rl78_decode.h).
#include "cpu/rl78/rl78_decode.h"

#include <array>
#include <vector>

namespace zlb {
namespace {

/// Row indices bucketed by the byte the lookup starts from.  Rows are inserted
/// in table order so that ties - two rows fixing the same number of bits - keep
/// the order binutils' own table has.
struct RowIndex {
    // u16, not u8: the table has more than 256 rows.
    std::array<std::vector<u16>, 256> by_byte;
    std::array<bool, 256> prefix{};

    RowIndex() {
        for (int row = 0; row < kRl78RowCount; ++row) {
            const Rl78Row& entry = kRl78Rows[row];
            if (entry.q == 2) {
                // Two-byte opcode: selected by the prefix byte, matched on the
                // whole 16-bit word.
                prefix[entry.prefix] = true;
                by_byte[entry.prefix].push_back(static_cast<u16>(row));
                continue;
            }
            // One-byte opcode: the fixed value may leave variable field bits, so
            // register the row under every byte value it matches.
            const u16 mask = static_cast<u16>(rl78_row_mask(entry) & 0xFFu);
            for (int byte = 0; byte < 256; ++byte) {
                if ((byte & mask) != (entry.value & mask)) continue;
                by_byte[static_cast<size_t>(byte)].push_back(static_cast<u16>(row));
            }
        }
    }
};

const RowIndex& row_index() {
    static const RowIndex index;
    return index;
}

int pop_count(u32 value) {
    int count = 0;
    while (value != 0) {
        count += static_cast<int>(value & 1u);
        value >>= 1;
    }
    return count;
}

/// Read `count` little-endian bytes of instruction data at `position`.
u32 read_bytes(Bus& bus, u32 base, unsigned position, unsigned count) {
    u32 value = 0;
    for (unsigned i = 0; i < count; ++i) {
        value |= static_cast<u32>(bus.fetch8((base + position + i) & 0xFFFFFu)) << (8 * i);
    }
    return value;
}

u8 read_byte(Bus& bus, u32 base, unsigned position) {
    return bus.fetch8((base + position) & 0xFFFFFu);
}

Rl78Operand decode_operand(Bus& bus, u32 address, const Rl78Decoded& insn, int slot,
                           int switch_byte, int header, bool es) {
    const Rl78Desc& desc = kRl78Descs[insn.row * 2 + slot];
    Rl78Operand op;
    op.type = static_cast<Rl78OpType>(desc.type);
    op.condition = static_cast<Rl78Cond>(desc.cn);
    if (op.type == Rl78OpType::None) return op;

    // ---- register / base register ----
    switch (desc.rm) {
        case kRl78RmFixed:
            op.reg = static_cast<Rl78Reg>(desc.rf);
            break;
        case kRl78RmField8:
            // The 3-bit field selects X..H, i.e. RL78_Reg_X + field.
            op.reg = static_cast<Rl78Reg>(static_cast<int>(Rl78Reg::X) +
                                          ((switch_byte >> desc.rs) & desc.rx));
            break;
        case kRl78RmField16:
            // The 2-bit field selects AX..HL, i.e. RL78_Reg_AX + field.
            op.reg = static_cast<Rl78Reg>(static_cast<int>(Rl78Reg::AX) +
                                          ((switch_byte >> desc.rs) & desc.rx));
            break;
        default:
            op.reg = Rl78Reg::None;
            break;
    }
    op.reg2 = static_cast<Rl78Reg>(desc.r2);

    // ---- bit number ----
    if (desc.bs == kRl78BsFixed) {
        op.bit = desc.bf;
    } else if (desc.bs != kRl78BsNone) {
        op.bit = static_cast<u8>((switch_byte >> desc.bs) & desc.bx);
    }

    op.es = es && (op.type == Rl78OpType::Ind || op.type == Rl78OpType::BitInd);
    op.add_kind = desc.ra;
    op.offset = static_cast<u8>(header + kRl78Rows[insn.row].q + desc.off);

    // ---- addend ----
    switch (desc.ra) {
        case kRl78AkNone:
            break;
        case kRl78AkLit:
            op.addend = desc.rp;
            break;
        case kRl78AkField:
            op.addend = (switch_byte >> desc.rs) & desc.rx;
            break;
        case kRl78AkImmu1:
            op.addend = static_cast<s32>(read_bytes(bus, address, op.offset, 1));
            op.size = 1;
            break;
        case kRl78AkImmu2:
            op.addend = static_cast<s32>(read_bytes(bus, address, op.offset, 2));
            op.size = 2;
            break;
        case kRl78AkImmu3:
            op.addend = static_cast<s32>(read_bytes(bus, address, op.offset, 3));
            op.size = 3;
            break;
        case kRl78AkImms1:
            op.addend = static_cast<s8>(read_byte(bus, address, op.offset));
            op.size = 1;
            break;
        case kRl78AkImms2:
            op.addend = static_cast<s16>(read_bytes(bus, address, op.offset, 2));
            op.size = 2;
            break;
        case kRl78AkSfr: {
            const int number = read_byte(bus, address, op.offset);
            op.addend = static_cast<s32>(rl78_sfr(number));
            op.address = static_cast<u32>(op.addend);
            op.size = 1;
            break;
        }
        case kRl78AkSaddr: {
            const int number = read_byte(bus, address, op.offset);
            op.addend = static_cast<s32>(rl78_short_direct(number));
            op.address = static_cast<u32>(op.addend);
            op.size = 1;
            break;
        }
        case kRl78AkRel8: {
            const int displacement = static_cast<s8>(read_byte(bus, address, op.offset));
            op.addend = static_cast<s32>(address) + static_cast<s32>(insn.length) + displacement;
            op.address = static_cast<u32>(op.addend);
            op.size = 1;
            break;
        }
        case kRl78AkRel16: {
            const int displacement = static_cast<s16>(read_bytes(bus, address, op.offset, 2));
            op.addend = static_cast<s32>(address) + static_cast<s32>(insn.length) + displacement;
            op.address = static_cast<u32>(op.addend);
            op.size = 2;
            break;
        }
        case kRl78AkCallt: {
            // 0x80 + mm * 16 + nnn * 2 : the CALLT table entry address.
            const int nnn = (switch_byte >> desc.rs) & desc.rx;
            const int mm = (switch_byte >> desc.bs) & desc.bx;
            op.addend = 0x80 + mm * 16 + nnn * 2;
            op.address = static_cast<u32>(op.addend);
            break;
        }
        default:
            break;
    }

    // For an address operand the decoder can resolve alone (SFR, saddr,
    // !addr16, CALLT) precompute the linear address; the register based forms
    // are resolved by the core against the current register file.
    if ((op.type == Rl78OpType::Ind || op.type == Rl78OpType::BitInd) &&
        op.reg == Rl78Reg::None &&
        (desc.ra == kRl78AkImmu2 || desc.ra == kRl78AkImmu3 || desc.ra == kRl78AkRel8 ||
         desc.ra == kRl78AkRel16)) {
        op.address = static_cast<u32>(op.addend);
    }
    return op;
}

}  // namespace

int rl78_row_count() { return kRl78RowCount; }

bool rl78_byte_is_prefix(u8 byte) { return row_index().prefix[byte]; }

int rl78_find_row(u8 op0, u8 op1) {
    const RowIndex& index = row_index();
    const bool two_byte = index.prefix[op0];
    const u32 word = two_byte ? ((static_cast<u32>(op0) << 8) | op1) : static_cast<u32>(op0);
    const u8 want_q = two_byte ? 2 : 1;

    int best = -1;
    int best_bits = -1;
    for (u16 candidate : index.by_byte[op0]) {
        const Rl78Row& entry = kRl78Rows[candidate];
        if (entry.q != want_q) continue;
        const u16 mask = rl78_row_mask(entry);
        const u32 value = two_byte ? ((static_cast<u32>(entry.prefix) << 8) | entry.value)
                                   : static_cast<u32>(entry.value);
        if ((word & mask) != (value & mask)) continue;
        // binutils spells out more bits first; ties keep table order.
        const int bits = pop_count(mask);
        if (bits > best_bits) {
            best_bits = bits;
            best = static_cast<int>(candidate);
        }
    }
    return best;
}

bool rl78_decode(Bus& bus, u32 address, Rl78Decoded& out) {
    address &= 0xFFFFFu;

    bool es = false;
    unsigned header = 0;
    u8 op0 = bus.fetch8(address);
    if (op0 == 0x11) {
        // 0x11 is the ES: prefix; the decoder never selects its "es:" row.
        es = true;
        header = 1;
        op0 = bus.fetch8((address + 1) & 0xFFFFFu);
    }

    int row;
    if (row_index().prefix[op0]) {
        const u8 op1 = bus.fetch8((address + header + 1) & 0xFFFFFu);
        row = rl78_find_row(op0, op1);
    } else {
        row = rl78_find_row(op0, 0);
    }
    if (row < 0) return false;

    const Rl78Row& entry = kRl78Rows[row];
    out.row = row;
    out.has_es_prefix = es;
    out.length = static_cast<unsigned>(entry.length) + header;
    out.id = static_cast<Rl78Id>(entry.id);
    out.flags = entry.flags;
    out.is_word = entry.is_word != 0;
    out.syntax = kRl78Syntaxes[entry.syntax];
    out.mnemonic = kRl78Mnemonics[entry.mnemonic];

    // The switch byte: for a two-byte opcode the byte after the prefix, else the
    // opcode byte itself (after an ES: prefix).
    const int switch_byte =
        entry.q == 2 ? bus.fetch8((address + header + 1) & 0xFFFFFu) : static_cast<int>(op0);

    out.ops[0] = decode_operand(bus, address, out, 0, switch_byte, static_cast<int>(header), es);
    out.ops[1] = decode_operand(bus, address, out, 1, switch_byte, static_cast<int>(header), es);
    return true;
}

unsigned rl78_instruction_length(Bus& bus, u32 address) {
    Rl78Decoded insn;
    if (!rl78_decode(bus, address, insn)) return 1;
    return insn.length;
}

}  // namespace zlb
