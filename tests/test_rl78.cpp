// zeliboba - RL78 core tests.
//
// Covers the operand model, the reset prologue of the real Ernie firmware, the
// four defects the RL78 rework fixed (SFR mirror, bit-instruction lengths, the
// missing STOP row and the 0x31 page switch mask), one arithmetic and one
// branch case per instruction family, the stack, the interrupt vector table and
// the register interface.
#include "test_framework.h"

#include <string>

#include "bus/bus.h"
#include "common/util.h"
#include "cpu/rl78/rl78_core.h"
#include "cpu/rl78/rl78_decode.h"
#include "cpu/rl78/rl78_disasm.h"

namespace {

using namespace zlb;

/// Code base for the hand encoded tests.
constexpr u32 kBase = 0x1000;

/// The first 33 bytes of the ErnIE reset handler (USS-1001, 0xE000).
const char* const kPrologue =
    "61CFCBF820FEF5F002CFF50080F57800FCF5040151E02C2073F693935820FEDFF9";

int hex_nibble(char character) {
    if (character >= '0' && character <= '9') return character - '0';
    if (character >= 'a' && character <= 'f') return character - 'a' + 10;
    if (character >= 'A' && character <= 'F') return character - 'A' + 10;
    return -1;
}

/// Plain RAM for the whole 1 MiB code space.  There is no SFR device model yet,
/// so everything that is not one of the five control registers the core mirrors
/// (SP/PSW/CS/ES/PMC) lands in RAM.
struct Bench {
    Bus bus;
    Rl78Core cpu;

    Bench() : cpu(bus) {
        bus.add_ram("rl78_test", 0x100000, 0, "plain RAM");
        bus.memset_bytes(0, 0, 0x100000);
    }

    /// Write the hex digits of `hex` (spaces are ignored) at `address`.
    void put(u32 address, const char* hex) {
        std::string digits;
        for (const char* cursor = hex; *cursor != '\0'; ++cursor) {
            if (hex_nibble(*cursor) >= 0) digits.push_back(*cursor);
        }
        for (size_t i = 0; i + 1 < digits.size(); i += 2) {
            const u8 byte =
                static_cast<u8>((hex_nibble(digits[i]) << 4) | hex_nibble(digits[i + 1]));
            bus.write8(address + static_cast<u32>(i / 2), byte);
        }
    }

    /// Clear 32 bytes at the code base, drop `hex` there and reset onto it.
    void code(const char* hex) {
        for (u32 i = 0; i < 32; ++i) bus.write8(kBase + i, 0);
        put(kBase, hex);
        cpu.reset(kBase);
    }

    std::string dis(u32 address, unsigned& length) { return rl78_disassemble(bus, address, length); }
};

/// Disassemble `hex` at the code base and compare text and length.
bool expect_disasm(Bench& bench, const char* hex, const std::string& want, int want_length) {
    bench.code(hex);
    unsigned length = 0;
    const std::string text = bench.dis(kBase, length);
    if (text == want && length == static_cast<unsigned>(want_length)) return true;
    ZLB_FAIL(zlb::format("disasm %s -> \"%s\" (%u bytes), want \"%s\" (%d bytes)", hex, text.c_str(),
                         length, want.c_str(), want_length));
    return false;
}

// ---------------------------------------------------------------------------
// 1. reset, the control register mirror and the register interface
// ---------------------------------------------------------------------------

ZLB_TEST(rl78_reset_reads_the_vector_table) {
    Bench bench;
    bench.bus.write16(0x0000, 0xE000);
    bench.cpu.reset();
    ZLB_EXPECT_EQ(0xE000u, bench.cpu.pc);
    ZLB_EXPECT_EQ(0xE000u, bench.cpu.reset_vector);

    // reset(entry) overrides the vector.
    bench.cpu.reset(0x0DC00);
    ZLB_EXPECT_EQ(0x0DC00u, bench.cpu.pc);

    // ... and the 20-bit mask is applied.
    bench.cpu.reset(0x123456);
    ZLB_EXPECT_EQ(0x23456u, bench.cpu.pc);
}

ZLB_TEST(rl78_reset_state_matches_the_hardware_manual) {
    Bench bench;
    bench.bus.write16(0x0000, 0xE000);
    bench.cpu.reset();
    ZLB_EXPECT_EQ(0x06, bench.cpu.psw);   // ISP0 = ISP1 = 1
    ZLB_EXPECT_EQ(0x0F, bench.cpu.es);    // reset value of ES
    ZLB_EXPECT_EQ(0x00, bench.cpu.cs);
    ZLB_EXPECT_EQ(0x0000, bench.cpu.sp);
    ZLB_EXPECT_EQ(3, bench.cpu.isp_level());  // ISP0 = ISP1 = 1
    ZLB_EXPECT_EQ(0, bench.cpu.bank());
    ZLB_EXPECT_EQ(0xFFEE0u, bench.cpu.bank_base());
    ZLB_EXPECT_FALSE(bench.cpu.flag_ie());
    ZLB_EXPECT_FALSE(bench.cpu.undefined_instruction);
}

ZLB_TEST(rl78_control_registers_live_in_the_core) {
    Bench bench;
    bench.code("CBF820FE");  // movw sp, #0xFE20
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xFE20, bench.cpu.sp);
    // The store must not have gone to RAM at 0xFFFF8.
    ZLB_EXPECT_EQ(0x00, bench.bus.read8(0xFFFF8));
    ZLB_EXPECT_EQ(0x00, bench.bus.read8(0xFFFF9));

    bench.code("0C5A9EF8");  // mov a, #0x5A ; mov 0xFFF8, a
    bench.cpu.step();
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x5Au, bench.cpu.sp & 0xFF);

    bench.code("0C019EFA");  // mov a, #1 ; mov 0xFFFA, a
    bench.cpu.step();
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x01u, bench.cpu.psw & 0xFF);

    bench.code("0C019EFD");  // mov a, #1 ; mov 0xFFFD, a  (ES)
    bench.cpu.step();
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x01, bench.cpu.es);

    // Reading the mirrored SFRs back sees the core, not RAM.
    bench.code("8EF8");  // mov a, 0xFFF8 (saddr 0xF8 -> SFR window)
    bench.cpu.sp = 0x1234;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x34, bench.cpu.a);

    bench.code("8EFA");  // mov a, 0xFFFA
    bench.cpu.psw = 0x00AB;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xAB, bench.cpu.a);

    // PMC (0xFFFFE) is mirrored too.
    bench.bus.write8(0xFFFFE, 0x11);  // RAM copy only
    bench.code("8EFE");               // mov a, 0xFFFFE
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x00, bench.cpu.a);  // the core's PMC, not the RAM byte
}

ZLB_TEST(rl78_control_register_word_access) {
    Bench bench;
    // 0xCB carries an SFR *number*, 0xC9 a short-direct address.  Both print
    // "0xFFFF8"/"0xFFEF8" respectively for n = 0xF8, but only 0xCB reaches SP -
    // that distinction is exactly defect #1.
    Rl78Decoded insn;
    bench.code("CBF820FE");  // movw sp, #0xFE20
    ZLB_EXPECT_TRUE(rl78_decode(bench.bus, kBase, insn));
    ZLB_EXPECT_EQ(kRl78AkSfr, insn.ops[0].add_kind);
    ZLB_EXPECT_EQ(kRl78SfrSp, insn.ops[0].address);
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xFE20, bench.cpu.sp);
    ZLB_EXPECT_EQ(0x0000, bench.bus.read16(0xFFFF8));

    bench.code("C9F820FE");  // movw 0xFFEF8, #0xFE20 (short direct: plain RAM)
    ZLB_EXPECT_TRUE(rl78_decode(bench.bus, kBase, insn));
    ZLB_EXPECT_EQ(kRl78AkSaddr, insn.ops[0].add_kind);
    ZLB_EXPECT_EQ(0xFFEF8u, insn.ops[0].address);
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x0000, bench.cpu.sp);
    ZLB_EXPECT_EQ(0xFE20, bench.bus.read16(0xFFEF8));

    // The 16-bit accessors mirror SP, PSW and ES/PMC.
    bench.cpu.write_data16(kRl78SfrSp, 0x1234);
    ZLB_EXPECT_EQ(0x1234, bench.cpu.sp);
    ZLB_EXPECT_EQ(0x1234u, bench.cpu.read_data16(kRl78SfrSp));
    // ES is the low byte at 0xFFFFD, PMC the high one at 0xFFFFE.
    bench.cpu.write_data16(kRl78SfrEs, 0x0A0B);
    ZLB_EXPECT_EQ(0x0B, bench.cpu.es);
    ZLB_EXPECT_EQ(0x0A, bench.cpu.pmc);
    ZLB_EXPECT_EQ(0x0A0Bu, bench.cpu.read_data16(kRl78SfrEs));
    bench.cpu.write_data16(kRl78SfrPsw, 0x00C1);
    ZLB_EXPECT_EQ(0x00C1u, bench.cpu.psw);
}

ZLB_TEST(rl78_register_interface) {
    Bench bench;
    bench.cpu.reset(kBase);
    std::vector<RegValue> regs;
    bench.cpu.registers(regs);
    ZLB_EXPECT_EQ(17u, regs.size());
    ZLB_EXPECT_TRUE(regs[0].group == "general" && regs[0].name == "PC");
    ZLB_EXPECT_EQ(kBase, regs[0].value);
    ZLB_EXPECT_TRUE(regs[6].name == "PSW");
    ZLB_EXPECT_TRUE(regs[6].note.find("bank0") != std::string::npos);
    ZLB_EXPECT_TRUE(regs[7].name == "ES");

    u64 value = 0;
    ZLB_EXPECT_TRUE(bench.cpu.set_register("AX", 0x1234));
    ZLB_EXPECT_TRUE(bench.cpu.get_register("ax", value));
    ZLB_EXPECT_EQ(0x1234u, value);
    ZLB_EXPECT_TRUE(bench.cpu.get_register("A", value));
    ZLB_EXPECT_EQ(0x12u, value);
    ZLB_EXPECT_TRUE(bench.cpu.get_register("X", value));
    ZLB_EXPECT_EQ(0x34u, value);
    ZLB_EXPECT_TRUE(bench.cpu.set_register("SP", 0xFE20));
    ZLB_EXPECT_EQ(0xFE20u, bench.cpu.sp);
    ZLB_EXPECT_TRUE(bench.cpu.set_register("PSW", 0x07));
    ZLB_EXPECT_EQ(0x07u, bench.cpu.psw);
    ZLB_EXPECT_TRUE(bench.cpu.set_register("PC", 0x3005E));
    ZLB_EXPECT_EQ(0x3005Eu, bench.cpu.pc);
    ZLB_EXPECT_FALSE(bench.cpu.set_register("r0", 1));
    ZLB_EXPECT_FALSE(bench.cpu.get_register("r0", value));
}

ZLB_TEST(rl78_status_line) {
    Bench bench;
    bench.cpu.reset(kBase);
    ZLB_EXPECT_TRUE(bench.cpu.status_line() ==
                    "RL78 PC=01000 AX=0000 BC=0000 DE=0000 HL=0000 SP=0000 PSW=06 [----] "
                    "ES=0F CS=00 bank=0");

    bench.cpu.pc = 0x3005E;
    bench.cpu.set_reg16(Rl78Reg::AX, 0x0000);
    bench.cpu.sp = 0xFE1C;
    bench.cpu.psw = 0x07;
    ZLB_EXPECT_TRUE(bench.cpu.status_line() ==
                    "RL78 PC=3005E AX=0000 BC=0000 DE=0000 HL=0000 SP=FE1C PSW=07 [--C-] "
                    "ES=0F CS=00 bank=0");

    std::vector<std::string> lines;
    bench.cpu.describe_state(lines);
    ZLB_EXPECT_TRUE(!lines.empty());
}

ZLB_TEST(rl78_step_result_reports_length_and_text) {
    Bench bench;
    bench.code("CBF820FE");
    const StepResult result = bench.cpu.step();
    ZLB_EXPECT_EQ(kBase, result.address);
    ZLB_EXPECT_EQ(4u, result.length);
    ZLB_EXPECT_TRUE(result.text == "movw sp, #0xFE20");
    ZLB_EXPECT_FALSE(result.faulted);
    ZLB_EXPECT_FALSE(result.was_branch);
    ZLB_EXPECT_EQ(kBase + 4, bench.cpu.pc);
    ZLB_EXPECT_EQ(1u, bench.cpu.instructions);

    bench.code("FC F5 04 01");  // call !0x104F5
    const StepResult call = bench.cpu.step();
    ZLB_EXPECT_TRUE(call.was_branch);
    ZLB_EXPECT_EQ(0x104F5u, bench.cpu.pc);
}

ZLB_TEST(rl78_undefined_encoding_faults) {
    Bench bench;
    // 0x31 0x06 has no row (the 0x31 page has no `0bit 0110` case).
    bench.code("3106");
    const StepResult result = bench.cpu.step();
    ZLB_EXPECT_TRUE(bench.cpu.undefined_instruction);
    ZLB_EXPECT_TRUE(result.faulted);
    ZLB_EXPECT_EQ(1u, result.length);
    ZLB_EXPECT_TRUE(result.text == "??");
    ZLB_EXPECT_TRUE(bench.cpu.halted);
    ZLB_EXPECT_EQ(1u, bench.cpu.unknown_instructions);
    // One byte was skipped so a caller can resynchronise.
    ZLB_EXPECT_EQ(kBase + 1, bench.cpu.pc);
}

// ---------------------------------------------------------------------------
// 2. the reset prologue and the four defect regressions
// ---------------------------------------------------------------------------

ZLB_TEST(rl78_reset_prologue_tiles_exactly) {
    Bench bench;
    bench.put(0xE000, kPrologue);
    bench.cpu.reset(0xE000);

    const char* const expected[] = {
        "sel rb0",            "movw sp, #0xFE20",   "clrb !0x02F0",       "mov !0x00F5, #0x80",
        "clrb !0x0078",       "call !0x104F5",      "mov a, #0xE0",       "sub a, #0x20",
        "mov b, a",           "clrw ax",            "dec b",              "dec b",
        "movw 0xFE20[b], ax", "bnz $0x0E01A",
    };
    const unsigned expected_length[] = {2, 4, 3, 4, 3, 4, 2, 2, 1, 1, 1, 1, 3, 2};

    u32 pc = 0xE000;
    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); ++i) {
        unsigned length = 0;
        const std::string text = bench.dis(pc, length);
        if (text != expected[i] || length != expected_length[i]) {
            ZLB_FAIL(zlb::format("prologue line %d at 0x%05X is \"%s\" (%u bytes), want \"%s\" (%u)",
                                 static_cast<int>(i), pc, text.c_str(), length, expected[i],
                                 expected_length[i]));
        }
        pc += length;
    }
    ZLB_EXPECT_EQ(0xE021u, pc);
    ZLB_EXPECT_EQ(33u, pc - 0xE000u);
}

ZLB_TEST(rl78_defect1_sfr_operands_are_registers_not_memory) {
    // CB F8 20 FE is `movw %s0, #%1` with %s0 = sfr(IMMU(1)) = 0xFFFF8 = SP.
    // Decoding it as `movw 0xFFF00, #0xFE20` (a plain 0xFFF00-based store, from
    // the old 0xFFF00 + n model) left the reset handler without a stack.
    Bench bench;
    bench.code("CBF820FE");
    Rl78Decoded insn;
    ZLB_EXPECT_TRUE(rl78_decode(bench.bus, kBase, insn));
    ZLB_EXPECT_EQ(4u, insn.length);
    ZLB_EXPECT_TRUE(insn.ops[0].type == Rl78OpType::Ind);
    ZLB_EXPECT_TRUE(insn.ops[0].reg == Rl78Reg::None);
    ZLB_EXPECT_EQ(kRl78AkSfr, insn.ops[0].add_kind);
    ZLB_EXPECT_EQ(kRl78SfrSp, insn.ops[0].address);
    ZLB_EXPECT_EQ(0xFE20, insn.ops[1].addend);
    ZLB_EXPECT_TRUE(insn.id == Rl78Id::Mov);
    ZLB_EXPECT_TRUE(insn.is_word);
    ZLB_EXPECT_TRUE(insn.mnemonic == std::string("movw"));

    bench.cpu.step();
    ZLB_EXPECT_EQ(0xFE20, bench.cpu.sp);
    ZLB_EXPECT_EQ(0x0000, bench.bus.read16(0xFFFF8));
}

ZLB_TEST(rl78_defect2_bit_instruction_lengths) {
    // set1/clr1 sfr.bit (`71 0A sfr`, `71 0B sfr`) are 3 bytes, not 2; the
    // CE-page `mov %s0,#%1` mul/div multiplex is 3 bytes, not 1; BR/CALL $rel16
    // are 3 bytes, not 2.
    Bench bench;
    expect_disasm(bench, "710A10", "set1 0xFFF10.0", 3);
    expect_disasm(bench, "710B10", "clr1 0xFFF10.0", 3);
    expect_disasm(bench, "CEFB03", "mov 0xFFFFB, #0x03", 3);
    expect_disasm(bench, "FE0002", "call $0x01203", 3);
    expect_disasm(bench, "EE0002", "br $0x01203", 3);

    bench.code("710A10");
    bench.bus.write8(0xFFF10, 0x00);
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x01, bench.bus.read8(0xFFF10));
    ZLB_EXPECT_EQ(kBase + 3, bench.cpu.pc);

    // The mul/div multiplex really is a DIVHU when the immediate is 0x03.
    bench.code("CEFB03");
    bench.cpu.a = 0x00;
    bench.cpu.x = 0x10;
    bench.cpu.d = 0x00;
    bench.cpu.e = 0x02;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x0008u, bench.cpu.get_reg16(Rl78Reg::AX));
    ZLB_EXPECT_EQ(0x0000u, bench.cpu.get_reg16(Rl78Reg::DE));
    ZLB_EXPECT_EQ(kBase + 3, bench.cpu.pc);
}

ZLB_TEST(rl78_defect3_stop_row_exists) {
    Bench bench;
    expect_disasm(bench, "61FD", "stop", 2);
    bench.code("61FD");
    bench.cpu.step();
    ZLB_EXPECT_TRUE(bench.cpu.halted);
    ZLB_EXPECT_TRUE(bench.cpu.halt_reason == "STOP");
    // A halted instruction does not advance PC.
    ZLB_EXPECT_EQ(kBase, bench.cpu.pc);

    expect_disasm(bench, "61ED", "halt", 2);
    bench.code("61ED");
    bench.cpu.step();
    ZLB_EXPECT_TRUE(bench.cpu.halted);
    ZLB_EXPECT_TRUE(bench.cpu.halt_reason == "HALT");
}

ZLB_TEST(rl78_defect4_switchmask_on_the_031_page) {
    // The 0x31 page switches on `op[1] & 0x8f`; each row only fixes the bits its
    // own pattern spells out.  With the old `0xFFFF & ~var` mask a row whose
    // `0bit`/`1bit` field is variable ended up pinned to a single encoding.
    Bench bench;
    for (int bit = 0; bit < 8; ++bit) {
        const int low = 0x02 | (bit << 4);   // "0bit 0010" = bt  saddr.bit
        const int high = 0x82 | (bit << 4);  // "1bit 0010" = bt  saddr.bit (ES page)
        bench.bus.write8(kBase + 0, 0x31);
        bench.bus.write8(kBase + 1, static_cast<u8>(low));
        bench.bus.write8(kBase + 2, 0x00);
        bench.bus.write8(kBase + 3, 0x00);
        Rl78Decoded insn;
        if (!rl78_decode(bench.bus, kBase, insn)) {
            ZLB_FAIL(zlb::format("0x31 %02X (bt with bit %d) does not decode", low, bit));
            continue;
        }
        if (insn.ops[1].bit != bit || insn.length != 4) {
            ZLB_FAIL(zlb::format("0x31 %02X decodes with bit %d length %u", low, insn.ops[1].bit,
                                 insn.length));
        }
        bench.bus.write8(kBase + 1, static_cast<u8>(high));
        if (!rl78_decode(bench.bus, kBase, insn) || insn.ops[1].bit != bit) {
            ZLB_FAIL(zlb::format("0x31 %02X (bit %d) does not decode", high, bit));
        }
    }
    expect_disasm(bench, "31120002", "bt 0xFFF00.1, $0x01006", 4);
    expect_disasm(bench, "31142002", "bf 0xFFE20.1, $0x01006", 4);
    expect_disasm(bench, "31170002", "shl c, 1", 2);
    // The whole 0x31 page: every bit-field combination must be known.
    for (int op1 = 0; op1 < 256; ++op1) {
        bench.bus.write8(kBase, 0x31);
        bench.bus.write8(kBase + 1, static_cast<u8>(op1));
        if (rl78_find_row(0x31, static_cast<u8>(op1)) < 0) continue;
        unsigned length = 0;
        const std::string text = bench.dis(kBase, length);
        if (text == "??") ZLB_FAIL(zlb::format("0x31 %02X finds a row but prints ??", op1));
    }
}

// ---------------------------------------------------------------------------
// 3. operand model / disassembly text
// ---------------------------------------------------------------------------

ZLB_TEST(rl78_disassembly_of_every_operand_kind) {
    Bench bench;
    // SFR and bit forms.
    expect_disasm(bench, "717BFA", "di", 3);
    expect_disasm(bench, "717AFA", "ei", 3);
    expect_disasm(bench, "7108F002", "clr1 !0x02F0.0", 4);
    expect_disasm(bench, "7138F002", "clr1 !0x02F0.3", 4);
    expect_disasm(bench, "710320", "clr1 0xFFE20.0", 3);
    expect_disasm(bench, "711320", "clr1 0xFFE20.1", 3);
    expect_disasm(bench, "718B", "clr1 a.0", 2);
    expect_disasm(bench, "7183", "clr1 [hl].0", 2);
    expect_disasm(bench, "7188", "clr1 cy", 2);
    expect_disasm(bench, "718A", "set1 a.0", 2);
    expect_disasm(bench, "7199", "mov1 a.1, cy", 2);
    expect_disasm(bench, "71CC", "mov1 cy, a.4", 2);
    expect_disasm(bench, "71C0", "not1 cy", 2);
    expect_disasm(bench, "710520", "and1 cy, 0xFFE20.0", 3);
    expect_disasm(bench, "710D20", "and1 cy, 0xFFF20.0", 3);
    expect_disasm(bench, "718D", "and1 cy, a.0", 2);
    expect_disasm(bench, "117108F002", "clr1 es:!0x02F0.0", 5);
    expect_disasm(bench, "F5F002", "clrb !0x02F0", 3);
    expect_disasm(bench, "CFF50080", "mov !0x00F5, #0x80", 4);

    // Register operands.
    expect_disasm(bench, "73", "mov b, a", 1);
    expect_disasm(bench, "93", "dec b", 1);
    expect_disasm(bench, "F6", "clrw ax", 1);
    expect_disasm(bench, "F7", "clrw bc", 1);
    expect_disasm(bench, "51E0", "mov a, #0xE0", 2);
    expect_disasm(bench, "50E0", "mov x, #0xE0", 2);
    expect_disasm(bench, "2C20", "sub a, #0x20", 2);
    expect_disasm(bench, "0C20", "add a, #0x20", 2);
    expect_disasm(bench, "3119", "shl a, 1", 2);
    expect_disasm(bench, "311A", "shr a, 1", 2);
    expect_disasm(bench, "311B", "sar a, 1", 2);
    expect_disasm(bench, "303412", "movw ax, #0x1234", 3);

    // Memory operands.
    expect_disasm(bench, "8B", "mov a, [hl]", 1);
    expect_disasm(bench, "8C10", "mov a, [hl+0x10]", 2);
    expect_disasm(bench, "8D20", "mov a, 0xFFE20", 2);
    expect_disasm(bench, "8E00", "mov a, 0xFFF00", 2);
    expect_disasm(bench, "8FF500", "mov a, !0x00F5", 3);
    expect_disasm(bench, "B810", "movw [sp+0x10], ax", 2);
    expect_disasm(bench, "9B", "mov [hl], a", 1);
    expect_disasm(bench, "BB", "movw [hl], ax", 1);
    expect_disasm(bench, "618B", "xch a, b", 2);
    expect_disasm(bench, "6180", "add a, [hl+b]", 2);
    expect_disasm(bench, "6182", "add a, [hl+c]", 2);
    expect_disasm(bench, "610905", "addw ax, [hl+0x05]", 3);
    expect_disasm(bench, "090001", "mov a, 0x0100[b]", 3);
    expect_disasm(bench, "490001", "mov a, 0x0100[bc]", 3);
    expect_disasm(bench, "290001", "mov a, 0x0100[c]", 3);

    // Stack.
    expect_disasm(bench, "C130", "push ax", 1);
    expect_disasm(bench, "C030", "pop ax", 1);
    expect_disasm(bench, "61DD", "push psw", 2);
    expect_disasm(bench, "61CD", "pop psw", 2);

    // Branches and calls.
    expect_disasm(bench, "FE0002", "call $0x01203", 3);
    expect_disasm(bench, "EE0002", "br $0x01203", 3);
    expect_disasm(bench, "FCF50401", "call !0x104F5", 4);
    expect_disasm(bench, "61CB", "br ax", 2);
    expect_disasm(bench, "61DA", "call bc", 2);
    expect_disasm(bench, "31042002", "bf 0xFFE20.0, $0x01006", 4);
    expect_disasm(bench, "310502", "bf a.0, $0x01005", 3);
    expect_disasm(bench, "31120002", "bt 0xFFF00.1, $0x01006", 4);

    // SEL, SKIP aliases and 16-bit register pairs.
    expect_disasm(bench, "61CF", "sel rb0", 2);
    expect_disasm(bench, "61FF", "sel rb3", 2);
    expect_disasm(bench, "61E8", "skz", 2);
    expect_disasm(bench, "61F8", "sknz", 2);
    expect_disasm(bench, "61C8", "skc", 2);
    expect_disasm(bench, "61E3", "skh", 2);
    expect_disasm(bench, "DF02", "bnz $0x01004", 2);
    expect_disasm(bench, "DD02", "bz $0x01004", 2);
}

ZLB_TEST(rl78_operand_addend_kinds) {
    Bench bench;
    // bt 0xFFF00.0, +2 : pc-relative source, short-direct destination.
    bench.code("31020002");
    Rl78Decoded insn;
    ZLB_EXPECT_TRUE(rl78_decode(bench.bus, kBase, insn));
    ZLB_EXPECT_TRUE(insn.ops[1].type == Rl78OpType::BitInd);
    ZLB_EXPECT_EQ(0u, insn.ops[1].bit);
    ZLB_EXPECT_EQ(0xFFF00u, insn.ops[1].address);
    ZLB_EXPECT_TRUE(insn.ops[0].type == Rl78OpType::Imm);
    ZLB_EXPECT_EQ(kRl78AkRel8, insn.ops[0].add_kind);
    ZLB_EXPECT_EQ(kBase + 6, static_cast<u32>(insn.ops[0].addend));
    ZLB_EXPECT_TRUE(insn.ops[1].condition == Rl78Cond::T);

    // bf a.0, +2 : register bit with the F condition.
    bench.code("310502");
    ZLB_EXPECT_TRUE(rl78_decode(bench.bus, kBase, insn));
    ZLB_EXPECT_TRUE(insn.ops[1].type == Rl78OpType::Bit);
    ZLB_EXPECT_TRUE(insn.ops[1].reg == Rl78Reg::A);
    ZLB_EXPECT_TRUE(insn.ops[1].condition == Rl78Cond::F);

    // A 16-bit absolute immediate (IMMU2) and the ES: page.
    bench.code("118F0001");
    ZLB_EXPECT_TRUE(rl78_decode(bench.bus, kBase, insn));
    ZLB_EXPECT_TRUE(insn.has_es_prefix);
    ZLB_EXPECT_EQ(4u, insn.length);
    ZLB_EXPECT_TRUE(insn.ops[1].es);
    ZLB_EXPECT_EQ(kRl78AkImmu2, insn.ops[1].add_kind);
    ZLB_EXPECT_EQ(0x0100, insn.ops[1].addend);

    // A 20-bit absolute target (IMMU3).
    bench.code("FCF50401");
    ZLB_EXPECT_TRUE(rl78_decode(bench.bus, kBase, insn));
    ZLB_EXPECT_EQ(kRl78AkImmu3, insn.ops[0].add_kind);
    ZLB_EXPECT_EQ(0x104F5, insn.ops[0].addend);
    ZLB_EXPECT_EQ(3u, insn.ops[0].size);

    // A sign-extended byte displacement and the CALLT table entry.
    bench.code("FE0002");
    ZLB_EXPECT_TRUE(rl78_decode(bench.bus, kBase, insn));
    ZLB_EXPECT_EQ(kRl78AkRel16, insn.ops[0].add_kind);
    ZLB_EXPECT_EQ(kBase + 3 + 0x200, static_cast<u32>(insn.ops[0].addend));

    bench.code("6184");
    ZLB_EXPECT_TRUE(rl78_decode(bench.bus, kBase, insn));
    ZLB_EXPECT_EQ(kRl78AkCallt, insn.ops[0].add_kind);
    ZLB_EXPECT_EQ(0x80u, insn.ops[0].address);
}

ZLB_TEST(rl78_instruction_fetch_goes_through_the_bus) {
    Bench bench;
    bench.code("CBF820FE");
    const u64 before = bench.bus.stats.fetches;
    unsigned length = 0;
    bench.dis(kBase, length);
    // 1 opcode byte + the switch byte + 2 operand bytes.
    ZLB_EXPECT_EQ(4u, bench.bus.stats.fetches - before);
    ZLB_EXPECT_TRUE(bench.bus.context.pc == kBase);
    ZLB_EXPECT_TRUE(bench.bus.context.core == "RL78");
}

// ---------------------------------------------------------------------------
// 4. one case per instruction family
// ---------------------------------------------------------------------------

ZLB_TEST(rl78_arithmetic_flags) {
    Bench bench;
    bench.code("5112");  // mov a, #0x12
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x12, bench.cpu.a);
    ZLB_EXPECT_EQ(kBase + 2, bench.cpu.pc);

    bench.code("510F");  // mov a, #0x0F ; add a, #1
    bench.cpu.step();
    bench.put(kBase, "0C01");
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x10, bench.cpu.a);
    ZLB_EXPECT_TRUE(bench.cpu.flag_ac());
    ZLB_EXPECT_FALSE(bench.cpu.flag_cy());
    ZLB_EXPECT_FALSE(bench.cpu.flag_z());

    bench.code("0C01");  // add a, #1
    bench.cpu.step();
    bench.put(kBase, "0CFF");
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x00, bench.cpu.a);
    ZLB_EXPECT_TRUE(bench.cpu.flag_cy());
    ZLB_EXPECT_TRUE(bench.cpu.flag_z());

    bench.code("0C00");  // add a, #0 ; sub a, #1
    bench.cpu.step();
    bench.put(kBase, "2C01");
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xFF, bench.cpu.a);
    ZLB_EXPECT_TRUE(bench.cpu.flag_cy());

    // ADDC uses CY.
    bench.code("0C00");
    bench.cpu.psw |= kRl78FlagCy;
    bench.put(kBase, "1C00");  // addc a, #0
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x01, bench.cpu.a);

    // INC does not touch CY (the row's flag mask is Fza).
    bench.code("51FF");
    bench.cpu.step();
    bench.cpu.psw |= kRl78FlagCy;
    bench.put(kBase, "8100");  // inc a
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x00, bench.cpu.a);
    ZLB_EXPECT_TRUE(bench.cpu.flag_z());
    ZLB_EXPECT_TRUE(bench.cpu.flag_cy());

    // CMP leaves the destination alone.
    bench.code("5105");  // mov a, #5 ; cmp a, #5
    bench.cpu.step();
    bench.put(kBase, "4C05");
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x05, bench.cpu.a);
    ZLB_EXPECT_TRUE(bench.cpu.flag_z());
    ZLB_EXPECT_FALSE(bench.cpu.flag_cy());

    // CMPW AX, #imm16 (the word form keeps Z/CY/AC in step with the byte form).
    bench.code("303412");  // movw ax, #0x1234
    bench.cpu.step();
    bench.put(kBase, "443412");  // cmpw ax, #0x1234
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_TRUE(bench.cpu.flag_z());
    ZLB_EXPECT_FALSE(bench.cpu.flag_cy());

    // 16-bit stack adjustment.
    bench.code("1010");  // addw sp, #0x10
    bench.cpu.sp = 0x100;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x110, bench.cpu.sp);
    ZLB_EXPECT_EQ(kBase + 2, bench.cpu.pc);

    bench.code("2010");  // subw sp, #0x10
    bench.cpu.sp = 0x100;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x0F0, bench.cpu.sp);

    // MOVW BC, AX
    bench.code("30341212");
    bench.cpu.step();
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x1234u, bench.cpu.get_reg16(Rl78Reg::BC));
    ZLB_EXPECT_EQ(kBase + 4, bench.cpu.pc);
}

ZLB_TEST(rl78_logic_family) {
    Bench bench;
    bench.code("51F0");  // mov a, #0xF0
    bench.cpu.step();
    bench.put(kBase, "5CF0");  // and a, #0xF0
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xF0, bench.cpu.a);

    bench.put(kBase, "6C0F");  // or a, #0x0F
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xFF, bench.cpu.a);

    bench.put(kBase, "7CFF");  // xor a, #0xFF
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x00, bench.cpu.a);
    ZLB_EXPECT_TRUE(bench.cpu.flag_z());
}

ZLB_TEST(rl78_shifts_and_rotates) {
    Bench bench;
    bench.code("0C81");  // mov a,#0x81 ; shl a,1
    bench.cpu.step();
    bench.put(kBase, "3119");
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x02, bench.cpu.a);
    ZLB_EXPECT_TRUE(bench.cpu.flag_cy());

    bench.code("0C81");
    bench.cpu.step();
    bench.put(kBase, "311A");  // shr a,1
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x40, bench.cpu.a);
    ZLB_EXPECT_TRUE(bench.cpu.flag_cy());

    bench.code("0C81");
    bench.cpu.step();
    bench.put(kBase, "311B");  // sar a,1
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xC0, bench.cpu.a);

    bench.code("0C81");
    bench.cpu.step();
    bench.put(kBase, "3179");  // shl a,7 (the 0x31 page bit-field form)
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x80, bench.cpu.a);

    // ROL/ROR through CY.
    bench.code("0C80");  // mov a,#0x80
    bench.cpu.step();
    bench.put(kBase, "61EB");  // rol a,1
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x01, bench.cpu.a);
    ZLB_EXPECT_TRUE(bench.cpu.flag_cy());

    bench.code("0C01");
    bench.cpu.step();
    bench.put(kBase, "61DB");  // ror a,1
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x80, bench.cpu.a);

    // Word shift.
    bench.code("303412");  // movw ax,#0x1234
    bench.cpu.step();
    bench.put(kBase, "311D");  // shlw ax,1
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x2468u, bench.cpu.get_reg16(Rl78Reg::AX));
}

ZLB_TEST(rl78_multiply_and_divide) {
    Bench bench;
    bench.code("D6");  // mulu x
    bench.cpu.a = 0x10;
    bench.cpu.x = 0x11;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x0110u, bench.cpu.get_reg16(Rl78Reg::AX));

    bench.code("CEFB01");  // mulhu
    bench.cpu.a = 0x02;
    bench.cpu.x = 0x00;
    bench.cpu.b = 0x03;
    bench.cpu.c = 0x00;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x0000u, bench.cpu.get_reg16(Rl78Reg::AX));
    ZLB_EXPECT_EQ(0x0006u, bench.cpu.get_reg16(Rl78Reg::BC));

    bench.code("CEFB02");  // mulh (signed)
    bench.cpu.a = 0xFF;
    bench.cpu.x = 0xFE;
    bench.cpu.b = 0x00;
    bench.cpu.c = 0x02;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xFFFCu, bench.cpu.get_reg16(Rl78Reg::AX));

    bench.code("CEFB03");  // divhu
    bench.cpu.a = 0x00;
    bench.cpu.x = 0x10;
    bench.cpu.d = 0x00;
    bench.cpu.e = 0x02;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x0008u, bench.cpu.get_reg16(Rl78Reg::AX));
    ZLB_EXPECT_EQ(0x0000u, bench.cpu.get_reg16(Rl78Reg::DE));

    // DIVWU: the dividend is BC:AX (BC high) and the quotient comes back in
    // BC:AX, with the remainder in DE:HL.
    bench.code("CEFB0B");  // divwu
    bench.cpu.a = 0x00;
    bench.cpu.x = 0x00;
    bench.cpu.b = 0x00;
    bench.cpu.c = 0x03;
    bench.cpu.d = 0x00;
    bench.cpu.e = 0x00;
    bench.cpu.h = 0x00;
    bench.cpu.l = 0x02;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x0001u, bench.cpu.get_reg16(Rl78Reg::AX));  // 0x30000 / 2 = 0x18000
    ZLB_EXPECT_EQ(0x8000u, bench.cpu.get_reg16(Rl78Reg::BC));
    ZLB_EXPECT_EQ(0x0000u, bench.cpu.get_reg16(Rl78Reg::DE));
    ZLB_EXPECT_EQ(0x0000u, bench.cpu.get_reg16(Rl78Reg::HL));

    // Division by zero returns all ones.
    bench.code("CEFB03");
    bench.cpu.a = 0x10;
    bench.cpu.x = 0x00;
    bench.cpu.d = 0x00;
    bench.cpu.e = 0x00;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xFFFFu, bench.cpu.get_reg16(Rl78Reg::AX));
    ZLB_EXPECT_EQ(0xFFFFu, bench.cpu.get_reg16(Rl78Reg::DE));

    // MACH writes the 32-bit result to 0xFFFF0..0xFFFF3.
    bench.code("CEFB05");  // machu
    bench.cpu.a = 0x00;
    bench.cpu.x = 0x02;
    bench.cpu.b = 0x00;
    bench.cpu.c = 0x03;
    bench.cpu.d = 0x00;
    bench.cpu.e = 0x00;
    bench.cpu.h = 0x00;
    bench.cpu.l = 0x00;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x06u, bench.bus.read32(0xFFFF0));
}

ZLB_TEST(rl78_branch_family) {
    Bench bench;
    // BT taken / not taken.
    bench.bus.write8(0xFFF00, 0x02);
    bench.code("31120002");  // bt 0xFFF00.1, +2
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 6, bench.cpu.pc);
    bench.bus.write8(0xFFF00, 0x00);
    bench.code("31120002");
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 4, bench.cpu.pc);

    bench.code("310302");  // bt a.0, +2
    bench.cpu.a = 0x01;
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 5, bench.cpu.pc);

    bench.code("310502");  // bf a.0, +2
    bench.cpu.a = 0x01;
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 3, bench.cpu.pc);

    // BTCLR branches and clears.
    bench.bus.write8(0xFFF00, 0x02);
    bench.code("31100002");  // btclr 0xFFF00.1, +2
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 6, bench.cpu.pc);
    ZLB_EXPECT_EQ(0x00, bench.bus.read8(0xFFF00));

    // BC / BNC / BZ / BNZ / BH / BNH.
    bench.code("DC01");  // bc +1
    bench.cpu.psw |= kRl78FlagCy;
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 3, bench.cpu.pc);

    bench.code("DD01");  // bz +1
    bench.cpu.psw |= kRl78FlagZ;
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 3, bench.cpu.pc);

    bench.code("DE01");  // bnc +1
    bench.cpu.psw &= static_cast<u16>(~kRl78FlagCy);
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 3, bench.cpu.pc);

    bench.code("DF01");  // bnz +1
    bench.cpu.psw &= static_cast<u16>(~kRl78FlagZ);
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 3, bench.cpu.pc);

    // BR $rel16 jumps to pc + 3 + disp.
    bench.code("EE0002");  // br $0x01203
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x1203u, bench.cpu.pc);

    // BR AX takes its page from CS.
    bench.code("61CB");  // br ax
    bench.cpu.set_reg16(Rl78Reg::AX, 0x1234);
    bench.cpu.cs = 0x02;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x21234u, bench.cpu.pc);

    // SKIP: sknz skips the following 2-byte instruction, skz does not.
    bench.code("61F80C12");
    bench.cpu.psw = 0x06;
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 4, bench.cpu.pc);

    bench.code("61E80C12");
    bench.cpu.psw = 0x06;
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 2, bench.cpu.pc);
}

ZLB_TEST(rl78_call_ret_and_callt) {
    Bench bench;
    bench.code("FD3412");  // call !0x1234
    bench.cpu.sp = 0x100;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x1234u, bench.cpu.pc);
    ZLB_EXPECT_EQ(0x00FE, bench.cpu.sp);
    ZLB_EXPECT_EQ(kBase + 3, bench.bus.read16(0xF0000u | 0x00FE));

    bench.put(0x1234, "D7");  // ret
    bench.cpu.step();
    ZLB_EXPECT_EQ(kBase + 3, bench.cpu.pc);
    ZLB_EXPECT_EQ(0x0100, bench.cpu.sp);

    // CALLT reads the table at 0x80 + n.
    bench.bus.write16(0x80, 0x1234);
    bench.code("6184");  // callt [0x80]
    bench.cpu.sp = 0x100;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x1234u, bench.cpu.pc);
    ZLB_EXPECT_EQ(kBase + 2, bench.bus.read16(0xF0000u | 0x00FE));

    // PUSH / POP round trip.
    bench.code("C130");  // push ax
    bench.cpu.sp = 0x100;
    bench.cpu.set_reg16(Rl78Reg::AX, 0xBEEF);
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xBEEFu, bench.bus.read16(0xF0000u | 0x00FE));
    bench.put(kBase, "C030");  // pop ax
    bench.cpu.pc = kBase;
    bench.cpu.set_reg16(Rl78Reg::AX, 0);
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xBEEFu, bench.cpu.get_reg16(Rl78Reg::AX));

    // PUSH PSW / POP PSW use the word SFR operand path.
    bench.code("61DD");  // push psw
    bench.cpu.sp = 0x100;
    bench.cpu.psw = 0x06;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x00FE, bench.cpu.sp);
    ZLB_EXPECT_EQ(0x06, bench.bus.read8(0xF0000u | 0x00FE));

    bench.code("61CD");  // pop psw
    bench.cpu.sp = 0x00FE;
    bench.cpu.psw = 0x06;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x0100, bench.cpu.sp);
    ZLB_EXPECT_EQ(0x06, bench.cpu.psw);
}

ZLB_TEST(rl78_bit_operations_on_registers_and_memory) {
    Bench bench;
    bench.code("713200");  // set1 0xFFF00.3
    bench.bus.write8(0xFFF00, 0x00);
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x08, bench.bus.read8(0xFFF00));

    bench.put(kBase, "713300");  // clr1 0xFFF00.3
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x00, bench.bus.read8(0xFFF00));

    bench.bus.write8(0xFFF00, 0x08);
    bench.code("713C00");  // mov1 cy, 0xFFF00.3
    bench.cpu.step();
    ZLB_EXPECT_TRUE(bench.cpu.flag_cy());

    bench.code("711900");  // mov1 0xFFF00.1, cy
    bench.bus.write8(0xFFF00, 0x00);
    bench.cpu.psw |= kRl78FlagCy;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x02, bench.bus.read8(0xFFF00));

    bench.bus.write8(0xFFF00, 0x01);
    bench.code("710500");  // and1 cy, 0xFFF00.0
    bench.cpu.psw |= kRl78FlagCy;
    bench.cpu.step();
    ZLB_EXPECT_TRUE(bench.cpu.flag_cy());

    bench.bus.write8(0xFFF00, 0x01);
    bench.code("710700");  // xor1 cy, 0xFFF00.0
    bench.cpu.psw &= static_cast<u16>(~kRl78FlagCy);
    bench.cpu.step();
    ZLB_EXPECT_TRUE(bench.cpu.flag_cy());

    bench.code("710600");  // or1 cy, 0xFFF00.0
    bench.bus.write8(0xFFF00, 0x00);
    bench.cpu.psw &= static_cast<u16>(~kRl78FlagCy);
    bench.cpu.step();
    ZLB_EXPECT_FALSE(bench.cpu.flag_cy());

    bench.code("71C0");  // not1 cy
    bench.cpu.psw |= kRl78FlagCy;
    bench.cpu.step();
    ZLB_EXPECT_FALSE(bench.cpu.flag_cy());

    bench.code("718A");  // set1 a.0
    bench.cpu.a = 0x00;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x01, bench.cpu.a);

    bench.code("718B");  // clr1 a.0
    bench.cpu.a = 0xFF;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xFE, bench.cpu.a);

    bench.code("7182");  // set1 [hl].0
    bench.cpu.h = 0x00;
    bench.cpu.l = 0x10;
    bench.bus.write8(0xF0010, 0x00);
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x01, bench.bus.read8(0xF0010));

    // DI / EI through the PSW mirror.
    bench.code("717BFA");  // clr1 psw.7
    bench.cpu.psw = 0xFF;
    bench.cpu.step();
    ZLB_EXPECT_FALSE(bench.cpu.flag_ie());

    bench.code("717AFA");  // set1 psw.7
    bench.cpu.psw = 0x06;
    bench.cpu.step();
    ZLB_EXPECT_TRUE(bench.cpu.flag_ie());

    // MOVS / CMPS.
    bench.code("61CE05");  // movs [hl+5], x
    bench.cpu.h = 0x00;
    bench.cpu.l = 0x10;
    bench.cpu.x = 0x9A;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x9A, bench.bus.read8(0xF0015));
    ZLB_EXPECT_FALSE(bench.cpu.flag_z());

    bench.code("61DE00");  // cmps x, [hl]
    bench.cpu.h = 0x00;
    bench.cpu.l = 0x10;
    bench.cpu.x = 0x9A;
    bench.bus.write8(0xF0010, 0x9A);
    bench.cpu.step();
    ZLB_EXPECT_TRUE(bench.cpu.flag_z());
}

ZLB_TEST(rl78_addressing_modes_execute) {
    Bench bench;
    // [HL] and [HL+byte]
    bench.code("5100");
    bench.cpu.step();
    bench.cpu.h = 0x00;
    bench.cpu.l = 0x10;
    bench.bus.write8(0xF0010, 0x5A);
    bench.put(kBase, "8B");
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x5A, bench.cpu.a);

    bench.cpu.a = 0x77;
    bench.put(kBase, "9B");
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x77, bench.bus.read8(0xF0010));

    bench.code("BB");
    bench.cpu.h = 0x00;
    bench.cpu.l = 0x10;
    bench.cpu.set_reg16(Rl78Reg::AX, 0xABCD);
    bench.cpu.step();
    ZLB_EXPECT_EQ(0xABCD, bench.bus.read16(0xF0010));

    // [HL+B] and [HL+C]
    bench.code("61C9");
    bench.cpu.h = 0x00;
    bench.cpu.l = 0x10;
    bench.cpu.b = 0x05;
    bench.bus.write8(0xF0015, 0x9A);
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x9A, bench.cpu.a);

    // Short-direct (0xFFE20 + n) and the SFR window (0xFFF00 + n).
    bench.bus.write8(0xFFE20, 0x33);
    bench.code("8D20");
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x33, bench.cpu.a);

    bench.bus.write8(0xFFF05, 0x44);
    bench.code("8E05");
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x44, bench.cpu.a);

    // !addr16 lives on the 0xF0000 data page.
    bench.bus.write8(0xF0100, 0x88);
    bench.code("8F0001");
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x88, bench.cpu.a);

    // [SP+n]
    bench.code("9805");
    bench.cpu.sp = 0xFE00;
    bench.cpu.a = 0x5A;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x5A, bench.bus.read8(0xF0000 + 0xFE05));

    // Base register forms: the immediate is the page offset and the register is
    // the index, so the address is 0xF0000 + imm16 + reg.
    bench.bus.write8(0xF0103, 0x11);
    bench.code("090001");  // mov a, 0x0100[b]
    bench.cpu.b = 0x03;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x11, bench.cpu.a);

    bench.bus.write8(0xF0102, 0x22);
    bench.code("290001");  // mov a, 0x0100[c]
    bench.cpu.c = 0x02;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x22, bench.cpu.a);

    bench.bus.write8(0xF0300, 0x33);
    bench.code("490001");  // mov a, 0x0100[bc]
    bench.cpu.set_reg16(Rl78Reg::BC, 0x0200);
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x33, bench.cpu.a);

    // ES: pages the direct access.
    bench.bus.write8(0x10100, 0x99);
    bench.code("4101");  // mov es, #1
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x01, bench.cpu.es);
    ZLB_EXPECT_EQ(kBase + 2, bench.cpu.pc);

    bench.put(kBase, "118F0001");  // es: mov a, !0x0100
    bench.cpu.pc = kBase;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x99, bench.cpu.a);
    ZLB_EXPECT_EQ(kBase + 4, bench.cpu.pc);
}

ZLB_TEST(rl78_sel_selects_the_register_bank) {
    Bench bench;
    bench.code("61CF");  // sel rb0
    bench.cpu.psw = 0x06;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0, bench.cpu.bank());
    ZLB_EXPECT_EQ(0xFFEE0u, bench.cpu.bank_base());
    ZLB_EXPECT_EQ(kBase + 2, bench.cpu.pc);

    bench.code("61FF");  // sel rb3
    bench.cpu.psw = 0x06;
    bench.cpu.step();
    ZLB_EXPECT_EQ(3, bench.cpu.bank());
    ZLB_EXPECT_EQ(0xFFEE0u + 24, bench.cpu.bank_base());
    // RBS0 is PSW.3 and RBS1 is PSW.5.
    ZLB_EXPECT_EQ(0x08u | 0x20u, bench.cpu.psw & 0x28u);
}

ZLB_TEST(rl78_interrupt_vector_table_and_reti) {
    Bench bench;
    // Vector 2 lives at 0x0004; the handler is `nop ; reti`.
    bench.bus.write16(0x0004, 0x1234);
    bench.code("00");  // nop at the code base
    bench.put(0x1234, "0061FC");
    bench.cpu.sp = 0x0200;
    bench.cpu.psw = 0x06;

    bench.cpu.request_interrupt(2);
    ZLB_EXPECT_FALSE(bench.cpu.interrupt_pending());  // IE = 0

    bench.cpu.psw |= kRl78FlagIe;
    ZLB_EXPECT_TRUE(bench.cpu.interrupt_pending());

    // The interrupt is taken at the start of the step and the handler's first
    // instruction is executed by the same call, exactly like the C# reference
    // core (which is what produced the verified 300000-instruction trace).
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x1235u, bench.cpu.pc);
    ZLB_EXPECT_FALSE(bench.cpu.flag_ie());
    ZLB_EXPECT_EQ(0x01FC, bench.cpu.sp);
    ZLB_EXPECT_EQ(kBase, bench.bus.read16(0xF0000u | 0x01FC));
    ZLB_EXPECT_EQ(0x86, bench.bus.read8(0xF0000u | 0x01FF));

    bench.cpu.step();  // reti
    ZLB_EXPECT_EQ(kBase, bench.cpu.pc);
    ZLB_EXPECT_EQ(0x0200, bench.cpu.sp);
    ZLB_EXPECT_EQ(0x86, bench.cpu.psw);
    ZLB_EXPECT_FALSE(bench.cpu.interrupt_pending());
}

ZLB_TEST(rl78_brk_takes_the_0x7e_vector) {
    Bench bench;
    bench.bus.write16(0x007E, 0x2000);
    bench.code("61CC");  // brk
    bench.cpu.sp = 0x0200;
    bench.cpu.psw = 0x86;
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x2000u, bench.cpu.pc);
    ZLB_EXPECT_FALSE(bench.cpu.flag_ie());
    ZLB_EXPECT_EQ(0x01FC, bench.cpu.sp);
    ZLB_EXPECT_EQ(kBase + 2, bench.bus.read16(0xF0000u | 0x01FC));
}

ZLB_TEST(rl78_irq_lines_are_programmable) {
    Bench bench;
    bench.code("00");  // nop
    bench.cpu.set_irq_vector(static_cast<int>(IrqLine::Syscon), 3);
    bench.bus.write16(6, 0x3000);
    bench.bus.write8(0x3000, 0x00);  // nop
    bench.cpu.psw |= kRl78FlagIe;
    bench.cpu.set_irq(static_cast<int>(IrqLine::Syscon), true);
    ZLB_EXPECT_TRUE(bench.cpu.interrupt_pending());
    bench.cpu.step();
    ZLB_EXPECT_EQ(0x3001u, bench.cpu.pc);
    bench.cpu.set_irq(static_cast<int>(IrqLine::Syscon), false);
    ZLB_EXPECT_EQ(3, bench.cpu.irq_vector(static_cast<int>(IrqLine::Syscon)));
    // Unprogrammed lines default to vector 0.
    ZLB_EXPECT_EQ(0, bench.cpu.irq_vector(static_cast<int>(IrqLine::Dma)));
}

// ---------------------------------------------------------------------------
// 5. table invariants
// ---------------------------------------------------------------------------

ZLB_TEST(rl78_table_shape) {
    ZLB_EXPECT_EQ(312, rl78_row_count());
    ZLB_EXPECT_EQ(312, kRl78RowCount);
    ZLB_EXPECT_EQ(624, 2 * kRl78RowCount);
    int two_byte = 0;
    int one_byte = 0;
    for (int row = 0; row < kRl78RowCount; ++row) {
        const Rl78Row& entry = kRl78Rows[row];
        if (entry.q == 2) {
            ++two_byte;
            // The 0x31 page switches on `op[1] & 0x8f`, every other two-byte
            // page on the whole byte.
            ZLB_EXPECT_EQ(entry.prefix == 0x31 ? 0x8Fu : 0xFFu,
                          static_cast<unsigned>(entry.switchmask));
            ZLB_EXPECT_TRUE(entry.prefix == 0x31 || entry.prefix == 0x61 || entry.prefix == 0x71);
        } else {
            ++one_byte;
            ZLB_EXPECT_TRUE(entry.prefix == 0);
        }
        ZLB_EXPECT_TRUE(entry.operands <= 2);
        ZLB_EXPECT_TRUE(entry.length >= 1 && entry.length <= kRl78MaxInstructionLength);
        ZLB_EXPECT_TRUE(entry.mnemonic < kRl78MnemonicCount);
        ZLB_EXPECT_TRUE(entry.syntax < kRl78SyntaxCount);
        ZLB_EXPECT_TRUE(entry.id <= static_cast<u8>(Rl78Id::Xor));
        // The switch mask formula must reproduce the fixed bits of the opcode.
        const u16 mask = rl78_row_mask(entry);
        ZLB_EXPECT_TRUE((entry.value & mask) == entry.value);
    }
    ZLB_EXPECT_EQ(134, two_byte);
    ZLB_EXPECT_EQ(178, one_byte);
}

ZLB_TEST(rl78_every_row_decodes_one_of_its_own_encodings) {
    Bench bench;
    int shadowed = 0;
    for (int row = 0; row < kRl78RowCount; ++row) {
        const Rl78Row& entry = kRl78Rows[row];
        if (entry.id == static_cast<u8>(Rl78Id::Unknown)) continue;  // the "es:" pseudo row
        bool found = false;
        for (int f = 0; f < 256 && !found; ++f) {
            const u16 mask = static_cast<u16>(rl78_row_mask(entry) & 0xFFu);
            if ((f & mask) != (entry.value & mask)) continue;
            int probe;
            if (entry.q == 2) {
                bench.bus.write8(kBase, entry.prefix);
                bench.bus.write8(kBase + 1, static_cast<u8>(f));
                probe = rl78_find_row(entry.prefix, static_cast<u8>(f));
            } else {
                bench.bus.write8(kBase, static_cast<u8>(f));
                bench.bus.write8(kBase + 1, 0);
                probe = rl78_find_row(static_cast<u8>(f), 0);
            }
            if (probe != row) continue;
            for (u32 i = 2; i < 24; ++i) bench.bus.write8(kBase + i, 0);
            unsigned length = 0;
            const std::string text = bench.dis(kBase, length);
            if (text == "??") continue;
            if (length != entry.length) {
                ZLB_FAIL(zlb::format("row %d (%s) f=%02X length %u, table says %u", row,
                                     kRl78Mnemonics[entry.mnemonic], f, length, entry.length));
            }
            found = true;
        }
        if (!found) {
            ++shadowed;
            ZLB_FAIL(zlb::format("row %d (%s) is never selected", row,
                                 kRl78Mnemonics[entry.mnemonic]));
        }
    }
    ZLB_EXPECT_EQ(0, shadowed);
}

ZLB_TEST(rl78_prefix_pages_decode_densely) {
    Bench bench;
    const u8 pages[] = {0x31, 0x61, 0x71};
    const int expected_known[] = {200, 232, 243};
    for (size_t page_index = 0; page_index < 3; ++page_index) {
        int known = 0;
        for (int op1 = 0; op1 < 256; ++op1) {
            for (u32 i = 0; i < 8; ++i) bench.bus.write8(kBase + i, 0);
            bench.bus.write8(kBase, pages[page_index]);
            bench.bus.write8(kBase + 1, static_cast<u8>(op1));
            unsigned length = 0;
            const std::string text = bench.dis(kBase, length);
            if (text != "??") ++known;
        }
        // 93 of the 256 `op[1] & 0x8f` combinations have no row (the .opc has no
        // `0bit/1bit 0110` case, for instance); the other two pages are denser.
        if (known != expected_known[page_index]) {
            ZLB_FAIL(zlb::format("page %02X decodes %d of 256 encodings, want %d",
                                 pages[page_index], known, expected_known[page_index]));
        }
    }
}

}  // namespace
