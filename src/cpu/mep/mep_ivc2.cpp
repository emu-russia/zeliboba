// zeliboba - MeP IVC2 slot decoder and formatter.  See mep_ivc2.h.
#include "cpu/mep/mep_ivc2.h"

#include <array>
#include <cstdio>
#include <cstring>

#include "cpu/mep/mep_isa.h"

namespace zlb::mep {
namespace {

struct TableEntry {
    const char* mnem;
    const char* fmt;
    u32 mask;
    u32 value;
    Ivc2Slot slot;
    u8 op_count;
    Ivc2Op ops[6];
};

/// The generated table, one entry per `(dni ...)` form of mep-ivc2.cpu.
const std::array<TableEntry, 688>& table() {
    static const std::array<TableEntry, 688> entries = {{
#define IVC2_SLOT(mnem, fmt, mask, value, slot, opcount, ...) \
    TableEntry{mnem, fmt, mask, value, Ivc2Slot::slot, opcount, {__VA_ARGS__}},
#include "cpu/mep/mep_ivc2_table.inc"
#undef IVC2_SLOT
    }};
    return entries;
}

/// Read one operand part.  `start` counts the part's last bit the CGEN way, so
/// `raw()` performs the byte swapped extraction for both halfwords.
u32 part_value(const Ivc2Part& part, u32 word) {
    if (part.length == 0) return 0;
    u32 value = raw(word, static_cast<int>(part.start), static_cast<int>(part.length));
    if (part.shift != 0) value <<= part.shift;
    return value;
}

u32 operand_value(const Ivc2Op& op, u32 word) {
    u32 value = 0;
    for (u8 i = 0; i < op.part_count; ++i) value |= part_value(op.parts[i], word);
    return value;
}
int operand_bits(const Ivc2Op& op) {
    int bits = 0;
    for (u8 i = 0; i < op.part_count; ++i) bits += op.parts[i].length;
    return bits;
}

const char* ivc2_cr_name(u32 index) {
    static char names[32][8];
    static bool once = false;
    if (!once) {
        once = true;
        for (int i = 0; i < 32; ++i) std::snprintf(names[i], sizeof(names[i]), "$cr%d", i);
    }
    return names[index & 31];
}

const char* ivc2_ccr_name(u32 index) {
    static char names[64][10];
    static bool once = false;
    if (!once) {
        once = true;
        for (int i = 0; i < 64; ++i) {
            if (i == 0) std::snprintf(names[i], sizeof(names[i]), "$csar0");
            else if (i == 0x1) std::snprintf(names[i], sizeof(names[i]), "$cc");
            else if (i == 0x4) std::snprintf(names[i], sizeof(names[i]), "$cofr0");
            else if (i == 0x5) std::snprintf(names[i], sizeof(names[i]), "$cofr1");
            else if (i == 0x6) std::snprintf(names[i], sizeof(names[i]), "$cofa0");
            else if (i == 0x7) std::snprintf(names[i], sizeof(names[i]), "$cofa1");
            else if (i == 0xF) std::snprintf(names[i], sizeof(names[i]), "$csar1");
            else if (i >= 16 && i <= 23)
                std::snprintf(names[i], sizeof(names[i]), "$acc0_%d", i - 16);
            else if (i >= 24 && i <= 31)
                std::snprintf(names[i], sizeof(names[i]), "$acc1_%d", i - 24);
            else std::snprintf(names[i], sizeof(names[i]), "$ccr%d", i);
        }
    }
    return names[index & 63];
}

void append_operand(std::string& out, const Ivc2Op& op, u32 word) {
    char buffer[64];
    const u32 value = operand_value(op, word);

    switch (op.print) {
        case Ivc2Print::Reg:
            out += register_name(static_cast<int>(value & 0xF));
            return;
        case Ivc2Print::Cp64:
        case Ivc2Print::Cp32:
            out += ivc2_cr_name(value);
            return;
        case Ivc2Print::Ivc2Ccr:
            out += ivc2_ccr_name(value);
            return;
        case Ivc2Print::Csr:
            out += control_register_name(static_cast<int>(value));
            return;
        case Ivc2Print::Name:
            out += op.name;
            return;
        case Ivc2Print::SImm: {
            const u32 signed_value = sext(value, operand_bits(op));
            std::snprintf(buffer, sizeof(buffer), "%d", static_cast<int>(signed_value));
            out += buffer;
            return;
        }
        case Ivc2Print::Imm:
        default:
            std::snprintf(buffer, sizeof(buffer), "0x%x", value);
            out += buffer;
            return;
    }
}

/// Split a little endian 64 bit packet into its two 32 bit halves.
void packet_words(const u8 bytes[8], u32& w0, u32& w1) {
    w0 = static_cast<u32>(bytes[0]) | (static_cast<u32>(bytes[1]) << 8) |
         (static_cast<u32>(bytes[2]) << 16) | (static_cast<u32>(bytes[3]) << 24);
    w1 = static_cast<u32>(bytes[4]) | (static_cast<u32>(bytes[5]) << 8) |
         (static_cast<u32>(bytes[6]) << 16) | (static_cast<u32>(bytes[7]) << 24);
}

/// The 16 bit slot word of a V1 packet: bytes 2..5 walked the CGEN way.  The
/// slot contributes up to 26 bits, but a P0S encoding never uses more than 16,
/// so the low 16 bits of this value are the slot word.
u32 p0s_word(const u8 bytes[8]) {
    return static_cast<u32>(bytes[2]) | (static_cast<u32>(bytes[3]) << 8) |
           (static_cast<u32>(bytes[4]) << 16) | (static_cast<u32>(bytes[5]) << 24);
}

}  // namespace

bool ivc2_packet_present(const u8 bytes[8]) {
    // binutils looks at the *second* memory byte on a little endian target
    // (`buf[1 ^ e]` with e = 1): below 0xc0 means the V1/V2 shapes, and the
    // 0xF. .. .7 pattern means V3.
    const u32 high = bytes[1] >> 4;
    return high < 0xCu || (high == 0xFu && (bytes[0] & 0x0Fu) == 0x07u);
}

Ivc2Packet ivc2_split_packet(const u8 bytes[8]) {
    Ivc2Packet packet;
    u32 w0 = 0;
    u32 w1 = 0;
    packet_words(bytes, w0, w1);

    // binutils' shape dispatch (mep_examine_ivc2_insns), for a little endian
    // target: the selector byte is memory byte 1.
    const u32 high = bytes[1] >> 4;

    if (high < 0xCu) {
        // V1: [core 16][p0s 16][p1 32]
        packet.valid = true;
        packet.has_core = true;
        packet.core_length = 2;
        packet.pieces[0] = {Ivc2Slot::P0S, p0s_word(bytes)};
        packet.pieces[1] = {Ivc2Slot::P1, w1};
        packet.piece_count = 2;
        return packet;
    }
    if (high == 0xFu && (bytes[0] & 0x0Fu) == 0x07u) {
        // V3: [p0 32][p1 32] - the whole packet is coprocessor.
        packet.valid = true;
        packet.has_core = false;
        packet.core_length = 0;
        packet.pieces[0] = {Ivc2Slot::P0, w0};
        packet.pieces[1] = {Ivc2Slot::P1, w1};
        packet.piece_count = 2;
        return packet;
    }
    // V2: [core 32]xxxx[p1 32]
    packet.valid = true;
    packet.has_core = true;
    packet.core_length = 4;
    packet.pieces[0] = {Ivc2Slot::P1, w1};
    packet.piece_count = 1;
    return packet;
}

const Ivc2Insn* ivc2_decode_slot(Ivc2Slot slot, u32 word) {
    for (const TableEntry& entry : table()) {
        if (entry.slot != slot) continue;
        if ((word & entry.mask) != entry.value) continue;
        return reinterpret_cast<const Ivc2Insn*>(&entry);
    }
    return nullptr;
}

std::string ivc2_format(const Ivc2Insn& insn, u32 word, u32) {
    std::string out;
    u8 next = 0;
    const char* s = insn.fmt;

    while (*s != '\0') {
        if (*s != '$') {
            out += *s++;
            continue;
        }
        // `$name`: consume the next operand.  The generated table lists the
        // operands in syntax order, so the index lines up.
        ++s;
        while ((*s >= 'a' && *s <= 'z') || (*s >= 'A' && *s <= 'Z') ||
               (*s >= '0' && *s <= '9') || *s == '_') {
            ++s;
        }
        if (next < insn.op_count) {
            append_operand(out, insn.ops[next], word);
            ++next;
        } else {
            out += "$?";
        }
    }
    return out;
}

std::string ivc2_disassemble_slot(Ivc2Slot slot, u32 word, u32 pc) {
    const Ivc2Insn* insn = ivc2_decode_slot(slot, word);
    if (insn == nullptr) {
        switch (slot) {
            case Ivc2Slot::P0S: return "*unknown-p0s*";
            case Ivc2Slot::P0: return "*unknown-p0*";
            case Ivc2Slot::P1: return "*unknown-p1*";
            case Ivc2Slot::C3:
            default: return "*unknown-c3*";
        }
    }
    return ivc2_format(*insn, word, pc);
}

}  // namespace zlb::mep
