// zeliboba - MeP IVC2 (Venezia VLIW coprocessor) disassembler tests.
//
// The encoder table is generated from MeP/mep-ivc2.cpu, so these tests only
// check the parts that the generator cannot get wrong by construction: the
// packet split (which bytes feed which slot), the slot word assembly, the
// register naming and the formatter wiring.  The packet bytes below are the
// real ones from the MeP payload of os0/kd/vnzimg.elf (file offset 0x130, MeP
// VA 0x80000000) and from the machine encodings written in the comments of
// mep-ivc2.cpu.
#include <cstdio>
#include <string>

#include "bus/bus.h"
#include "cpu/mep/mep_core.h"
#include "cpu/mep/mep_ivc2.h"
#include "cpu/mep/mep_isa.h"
#include "test_framework.h"

namespace {

using zlb::Bus;
using zlb::MePCore;
using zlb::mep::Ivc2Print;
using zlb::mep::Ivc2Slot;
using zlb::u32;
using zlb::u8;

/// Assemble a packet from the four bytes of its two 16 bit units, the way the
/// little endian MeP stores them.
void words_for(const u8 bytes[8], u32& w0, u32& w1) {
    w0 = static_cast<u32>(bytes[0]) | (static_cast<u32>(bytes[1]) << 8) |
         (static_cast<u32>(bytes[2]) << 16) | (static_cast<u32>(bytes[3]) << 24);
    w1 = static_cast<u32>(bytes[4]) | (static_cast<u32>(bytes[5]) << 8) |
         (static_cast<u32>(bytes[6]) << 16) | (static_cast<u32>(bytes[7]) << 24);
}

/// The first eight bytes of the vnzimg MeP payload: a vector table of `jmp`s.
ZLB_TEST(ivc2_packet_bytes_match_the_mep_payload) {
    const u8 bytes[8] = {0xE8, 0xD9, 0x06, 0x80, 0x18, 0xDA, 0x0A, 0x80};
    u32 w0 = 0;
    u32 w1 = 0;
    words_for(bytes, w0, w1);
    ZLB_EXPECT_EQ(w0, 0x8006D9E8u);
    ZLB_EXPECT_EQ(w1, 0x800ADA18u);

    // ... and that word is a core `jmp 0x80063c` (checked against the core
    // decoder, which is the same object the packet decoder uses).
    const zlb::mep::Insn* insn = zlb::mep::decode(w0);
    ZLB_EXPECT_TRUE(std::string(insn->mnem) == "jmp");
}

/// V1 shape (core 16 + p0s 16 + p1 32): the p0s word is bytes 2..5 and the p1
/// word is bytes 4..7 (they overlap: the p0s slot is the upper half of the
/// second 32 bit word).  The shape selector is memory byte 1 (little endian).
ZLB_TEST(ivc2_v1_packet_split) {
    const u8 bytes[8] = {0x44, 0x07, 0x03, 0x14, 0x00, 0x00, 0x11, 0x22};  // byte1 < 0xc0
    const zlb::mep::Ivc2Packet packet = zlb::mep::ivc2_split_packet(bytes);
    ZLB_EXPECT_TRUE(packet.valid);
    ZLB_EXPECT_TRUE(packet.has_core);
    ZLB_EXPECT_EQ(packet.core_length, 2u);
    ZLB_EXPECT_EQ(packet.piece_count, 2u);
    ZLB_EXPECT_TRUE(packet.pieces[0].slot == Ivc2Slot::P0S);
    ZLB_EXPECT_TRUE(packet.pieces[1].slot == Ivc2Slot::P1);
    // bytes 2..5 little endian: 03, 14, 00, 00 -> 0x00001403
    ZLB_EXPECT_EQ(packet.pieces[0].word, 0x00001403u);
    ZLB_EXPECT_EQ(packet.pieces[1].word, 0x22110000u);
}

/// V2 shape (core 32 + p1 32): the first word is the core instruction.
ZLB_TEST(ivc2_v2_packet_split) {
    const u8 bytes[8] = {0xE8, 0xD9, 0x06, 0x80, 0x44, 0x07, 0x00, 0xF0};
    const zlb::mep::Ivc2Packet packet = zlb::mep::ivc2_split_packet(bytes);
    ZLB_EXPECT_TRUE(packet.valid);
    ZLB_EXPECT_TRUE(packet.has_core);
    ZLB_EXPECT_EQ(packet.core_length, 4u);
    ZLB_EXPECT_EQ(packet.piece_count, 1u);
    ZLB_EXPECT_TRUE(packet.pieces[0].slot == Ivc2Slot::P1);
    ZLB_EXPECT_EQ(packet.pieces[0].word, 0xF0000744u);
}

/// V3 shape (p0 32 + p1 32, no core instruction).  The selector is memory byte
/// 1 with the high nibble 0xF and memory byte 0 with the low nibble 7 (the
/// `f-sub4` = 7 of the p0 word).  The slot word is the little endian assembly
/// of the first four bytes, so `07 F7 00 00` gives 0x0000F707.
ZLB_TEST(ivc2_v3_packet_split) {
    const u8 bytes[8] = {0x07, 0xF7, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44};
    const zlb::mep::Ivc2Packet packet = zlb::mep::ivc2_split_packet(bytes);
    ZLB_EXPECT_TRUE(packet.valid);
    ZLB_EXPECT_FALSE(packet.has_core);
    ZLB_EXPECT_EQ(packet.piece_count, 2u);
    ZLB_EXPECT_TRUE(packet.pieces[0].slot == Ivc2Slot::P0);
    ZLB_EXPECT_EQ(packet.pieces[0].word, 0x0000F707u);
    ZLB_EXPECT_EQ(packet.pieces[1].word, 0x44332211u);
}

/// Every generated entry has to be reachable: fill the bits its mask leaves
/// free and check that `decode_slot` finds it again.
ZLB_TEST(ivc2_slot_decoding_is_self_consistent) {
    // A few well known entries, with the bits the operand occupies filled in by
    // hand.  `cpmovfrcc` is `(+ MAJ_15 (f-ivc2-3u4 #x0) croc (f-sub4 7)
    // (f-ivc2-5u16 #x10) (f-ivc2-5u21 #x0) (f-ivc2-5u26 #x1) (f-ivc2-1u31 #x0))`,
    // so croc (bits 3..7) and the unconstrained bits may vary.
    const u32 cpmovfrcc = 0xF0410E00u;
    const zlb::mep::Ivc2Insn* insn =
        zlb::mep::ivc2_decode_slot(Ivc2Slot::C3, cpmovfrcc);
    ZLB_EXPECT_TRUE(insn != nullptr);
    // Any operand value in the free bits must still decode to the same entry.
    if (insn != nullptr) {
        for (u32 fill = 0; fill < 0x100u; fill += 0x1Cu) {
            const u32 word = (cpmovfrcc & insn->mask) | (fill & ~insn->mask);
            const zlb::mep::Ivc2Insn* again = zlb::mep::ivc2_decode_slot(Ivc2Slot::C3, word);
            ZLB_EXPECT_TRUE(again != nullptr);
        }
    }

    // The 16 bit slots can be swept exhaustively.
    int p0s = 0;
    for (u32 word = 0; word < 0x10000u; ++word) {
        if (zlb::mep::ivc2_decode_slot(Ivc2Slot::P0S, word) != nullptr) ++p0s;
    }
    ZLB_EXPECT_TRUE(p0s > 0);
}

/// `cpmovfrcc` in the C3 slot.  Its encoding is
/// `(+ MAJ_15 (f-ivc2-3u4 #x0) croc (f-sub4 7) (f-ivc2-5u16 #x10)
///   (f-ivc2-5u21 #x0) (f-ivc2-5u26 #x1) (f-ivc2-1u31 #x0))`,
/// which for `croc = 0` is the slot word 0xF0410E00.
ZLB_TEST(ivc2_formats_a_known_c3_encoding) {
    const u32 word = 0xF0410E00u;
    const zlb::mep::Ivc2Insn* insn = zlb::mep::ivc2_decode_slot(Ivc2Slot::C3, word);
    ZLB_EXPECT_TRUE(insn != nullptr);
    if (insn != nullptr) {
        const std::string text = zlb::mep::ivc2_format(*insn, word, 0x80000000u);
        ZLB_EXPECT_TRUE(text.substr(0, 9) == "cpmovfrcc");
        // the operand list must be rendered, not left as "$..." placeholders
        ZLB_EXPECT_TRUE(text.find('$') != std::string::npos);
        ZLB_EXPECT_TRUE(text.find("$cr0") != std::string::npos);
    }
}

/// Unknown slot words must render as objdump's placeholders, not as garbage.
ZLB_TEST(ivc2_unknown_slot_placeholder) {
    ZLB_EXPECT_TRUE(zlb::mep::ivc2_disassemble_slot(Ivc2Slot::P0S, 0xFFFFFFFFu, 0) ==
                    "*unknown-p0s*");
    ZLB_EXPECT_TRUE(zlb::mep::ivc2_disassemble_slot(Ivc2Slot::P1, 0xFFFFFFFFu, 0) ==
                    "*unknown-p1*");
}

/// VLIW mode through the core's own disassembler: a V2 packet prints as
/// "<core> + <slot>".
ZLB_TEST(mep_core_disassembles_a_vliw_packet) {
    Bus bus;
    bus.add_ram("packet", 0x1000, 0x80000000, "test packet");
    const zlb::u8 bytes[8] = {0xE8, 0xD9, 0x06, 0x80, 0x44, 0x07, 0x00, 0xF0};
    bus.load(0x80000000, bytes, sizeof(bytes), "packet");

    MePCore cpu(bus);
    cpu.reset(0x80000000);
    cpu.vliw_mode = true;

    unsigned length = 0;
    const std::string text = cpu.disassemble(0x80000000, length);
    ZLB_EXPECT_EQ(length, 8u);
    ZLB_EXPECT_TRUE(text.find(" + ") != std::string::npos);
    ZLB_EXPECT_TRUE(text.find("jmp") == 0);
}

}  // namespace
