// zeliboba - MeP-c5 (CMeP) core self tests.
//
// Every opword in this file is a literal copied out of the prototype CMeP boot
// ROM (dumps/vita_prototype_bootrom.bin, base address 0x5C000); the comment
// gives the ROM address it came from.  The words are literals rather than
// formulas because MeP packs its operand fields in unusual places and the CGEN
// table reproduces that faithfully:
//
//   * MAJ_0 (`sub $3,$2` = 0x0324) takes its first syntax operand from bits
//     8..11 and its address/other operand from bits 4..7;
//   * `add3 $1,$2,$3` (0x00009231) writes the result to the field at bits 4..7
//     while the formatter lists it first;
//   * the 12 bit branches (`beqz`) fold the register and the pc relative
//     displacement into the same bits, and their displacement is measured from
//     the instruction *after* the branch and scaled by 2.
//
// Building these words from an assembler style formula is therefore a good way
// to test the test rather than the core, which is what earlier iterations of
// this file kept doing.
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "bus/device.h"
#include "common/util.h"
#include "cpu/mep/mep_core.h"
#include "cpu/mep/mep_disasm.h"
#include "test_framework.h"

namespace {

using zlb::Bus;
using zlb::MePCore;
using zlb::StepResult;
using zlb::u8;
using zlb::u16;
using zlb::u32;
using zlb::u64;

constexpr u32 kRamBase = 0x40000;
constexpr u32 kRamSize = 0x20000;
constexpr u32 kCode = 0x40000;

// ---------------------------------------------------------------------------
// Boot ROM instructions, quoted with their address in the image.
// ---------------------------------------------------------------------------

constexpr u32 kMov1Zero = 0x5100u;        ///< 0x5C00C: mov $1,0
constexpr u32 kSub32 = 0x0324u;           ///< 0x5C00E: sub $3,$2
constexpr u32 kSrl3By3 = 0x631Au;         ///< 0x5C010: srl $3,0x3
constexpr u32 kAdd3Minus1 = 0x63FCu;      ///< 0x5C012: add $3,-1
constexpr u32 kRepeat = 0x0003E309u;      ///< 0x5C014: repeat $3,0x5c01a
constexpr u32 kSwIndirect = 0x012Au;      ///< 0x5C018: sw $1,($2)
constexpr u32 kSwDisp4 = 0x0004C12Au;     ///< 0x5C01A: sw $1,4($2)
constexpr u32 kAdd2By8 = 0x6220u;         ///< 0x5C01E: add $2,8
constexpr u32 kMovSp0 = 0x0F00u;          ///< 0x5C024: mov $sp,$0
constexpr u32 kStc0Hi = 0x7078u;          ///< 0x5C038: stc $0,$hi
constexpr u32 kStc0Lo = 0x7088u;          ///< 0x5C03A: stc $0,$lo
constexpr u32 kStc3Sar = 0x7328u;         ///< 0x5C05E: stc $3,$sar
constexpr u32 kStc0Lp = 0x7018u;          ///< 0x5C06E: stc $0,$lp
constexpr u32 kLdc0Lp = 0x701Au;          ///< 0x5C500: ldc $0,$lp
constexpr u32 kLwIndirect = 0x07AEu;      ///< 0x5C57A: lw $7,($10)
constexpr u32 kRet = 0x7002u;             ///< 0x5E628: ret
constexpr u32 kJmpReg = 0x101Eu;          ///< 0x5C50A: jmp $1
constexpr u32 kAdd3Three = 0x00009231u;   ///< 0x5C0EA: add3 $1,$2,$3
constexpr u32 kBreak = 0x7032u;
constexpr u32 kHalt = 0x7022u;
constexpr u32 kUnknown16 = 0x101Du;       ///< no table entry

struct Fixture {
    Bus bus;
    MePCore cpu;

    Fixture() : cpu(bus) {
        bus.add_ram("cmep_ram", kRamSize, kRamBase, "test RAM");
        cpu.reset(kCode);
    }

    /// Write one instruction word.  A 16 bit form must use a 16 bit store or it
    /// would clobber the instruction that follows.
    void word(u32 address, u32 value) {
        if ((value >> 16) == 0) {
            bus.write16(address, static_cast<u16>(value));
        } else {
            bus.write32(address, value);
        }
    }

    void run(int count = 64) {
        for (int i = 0; i < count; ++i) {
            const StepResult result = cpu.step();
            if (result.faulted || cpu.halted) break;
        }
    }
};

void show(const Fixture& f, const char* tag) {
    if (std::getenv("ZLB_MEP_VERBOSE") == nullptr) return;
    std::printf("      [%s] pc=0x%08X", tag, f.cpu.pc);
    for (int i = 0; i < 8; ++i) std::printf(" $%d=%08X", i, f.cpu.r[i]);
    std::printf("\n");
}

}  // namespace

// ---------------------------------------------------------------------------
// Instruction semantics (all opwords from the boot ROM)
// ---------------------------------------------------------------------------

ZLB_TEST(mep_alu_immediates) {
    // sub $3,$2 (0x0324): the destination is the field at bits 4..7, so with
    // $2 = 10 and $3 = 40 this computes $3 = 40 - 10.
    Fixture f;
    f.cpu.r[2] = 10;
    f.cpu.r[3] = 40;
    f.word(kCode, kSub32);
    f.run(1);
    show(f, "sub");
    ZLB_EXPECT_EQ(f.cpu.r[3], 30u);
    ZLB_EXPECT_EQ(f.cpu.r[2], 10u);
    ZLB_EXPECT_EQ(f.cpu.pc, kCode + 2u);

    // add $2,8 (0x6220).
    Fixture g;
    g.cpu.r[2] = 40;
    g.word(kCode, kAdd2By8);
    g.run(1);
    show(g, "add");
    ZLB_EXPECT_EQ(g.cpu.r[2], 48u);

    // srl $3,0x3 (0x631A): 0x40 >> 3 = 8.
    Fixture h;
    h.cpu.r[3] = 0x40;
    h.word(kCode, kSrl3By3);
    h.run(1);
    show(h, "srl");
    ZLB_EXPECT_EQ(h.cpu.r[3], 8u);

    // add $3,-1 (0x63FC) sign extends the six bit immediate field.
    Fixture i;
    i.cpu.r[3] = 100;
    i.word(kCode, kAdd3Minus1);
    i.run(1);
    show(i, "add -1");
    ZLB_EXPECT_EQ(i.cpu.r[3], 99u);
}

ZLB_TEST(mep_add3) {
    // add3 $1,$2,$3 (0x00009231 at 0x5C0EA): the result lands in the field the
    // formatter names first, and the two operands are the other two registers.
    Fixture f;
    f.cpu.r[1] = 1;
    f.cpu.r[2] = 2;
    f.cpu.r[3] = 3;
    f.word(kCode, kAdd3Three);
    f.run(1);
    show(f, "add3");
    ZLB_EXPECT_EQ(f.cpu.r[1], 5u);
    ZLB_EXPECT_EQ(f.cpu.r[2], 2u);
    ZLB_EXPECT_EQ(f.cpu.r[3], 3u);
}

ZLB_TEST(mep_loads_and_stores) {
    // sw $1,($2) then lw $7,($10), both pointing at the same RAM.
    Fixture f;
    f.cpu.r[1] = 0xCAFEBABEu;
    f.cpu.r[2] = 0x50000;
    f.cpu.r[10] = 0x50000;
    f.word(kCode, kSwIndirect);
    f.word(kCode + 2, kLwIndirect);
    f.run(2);
    show(f, "sw/lw");
    ZLB_EXPECT_EQ(f.bus.read32(0x50000), 0xCAFEBABEu);
    ZLB_EXPECT_EQ(f.cpu.r[7], 0xCAFEBABEu);

    // The 16 bit displacement form: sw $1,4($2) writes at 0x50004.
    Fixture g;
    g.cpu.r[1] = 0x11223344u;
    g.cpu.r[2] = 0x50000;
    g.word(kCode, kSwDisp4);
    g.run(1);
    show(g, "sw 4(r2)");
    ZLB_EXPECT_EQ(g.bus.read32(0x50004), 0x11223344u);
}

ZLB_TEST(mep_control_registers_stc_ldc) {
    // stc $3,$sar (0x7328 at 0x5C05E) writes control register 2.
    Fixture f;
    f.cpu.r[3] = 0x99;
    f.word(kCode, kStc3Sar);
    f.run(1);
    show(f, "stc $3,$sar");
    ZLB_EXPECT_EQ(f.cpu.sar, 0x99u);

    // stc $0,$hi and stc $0,$lo (0x5C038 / 0x5C03A).
    Fixture g;
    g.cpu.r[0] = 0xDEADBEEF;
    g.word(kCode, kStc0Hi);
    g.word(kCode + 2, kStc0Lo);
    g.run(2);
    show(g, "stc hi/lo");
    ZLB_EXPECT_EQ(g.cpu.hi, 0xDEADBEEFu);
    ZLB_EXPECT_EQ(g.cpu.lo, 0xDEADBEEFu);

    // stc $0,$lp (0x5C06E) then ldc $0,$lp (0x5C500).
    Fixture h;
    h.cpu.r[0] = 0x00040000u;
    h.word(kCode, kStc0Lp);
    h.word(kCode + 2, kLdc0Lp);
    h.run(2);
    show(h, "stc/ldc lp");
    ZLB_EXPECT_EQ(h.cpu.lp, 0x00040000u);
    ZLB_EXPECT_EQ(h.cpu.r[0], 0x00040000u);
}

ZLB_TEST(mep_repeat_hardware_loop) {
    // repeat $3,<end> (0x0003E309) relocated: it names the end of the block and
    // the loop unit repeats [$rpb, $rpe] plus one trailing instruction.
    Fixture f;
    f.cpu.r[3] = 3;
    f.word(kCode, kRepeat);
    f.word(kCode + 4, 0x0000u);
    f.word(kCode + 6, 0x0000u);
    f.word(kCode + 8, 0x0000u);
    f.word(kCode + 0x0A, 0x0000u);
    f.word(kCode + 0x0C, 0x0000u);
    f.run(24);
    show(f, "repeat");
    ZLB_EXPECT_EQ(f.cpu.rpb, kCode + 4u);          // the instruction after repeat
    ZLB_EXPECT_EQ(f.cpu.rpe, kCode + 0x06u);       // 0x5c01a relocated
    ZLB_EXPECT_EQ(f.cpu.rpc, 0u);
    ZLB_EXPECT_FALSE(f.cpu.halted);
}

ZLB_TEST(mep_jmp_and_ret) {
    // jmp $1 (0x101E) is a tail call: $lp must be untouched.
    Fixture f;
    f.cpu.lp = 0xDEADBEEFu;
    f.cpu.r[1] = kCode + 0x10;
    f.word(kCode, kJmpReg);
    f.run(1);
    show(f, "jmp");
    ZLB_EXPECT_EQ(f.cpu.pc, kCode + 0x10u);
    ZLB_EXPECT_EQ(f.cpu.lp, 0xDEADBEEFu);

    // ret jumps to $lp with bit 0 masked off.
    Fixture g;
    g.cpu.lp = (kCode + 0x20) | 1u;
    g.word(kCode, kRet);
    g.run(1);
    show(g, "ret");
    ZLB_EXPECT_EQ(g.cpu.pc, kCode + 0x20u);
}

ZLB_TEST(mep_control_bus_delay_timer) {
    // The delay blocks drive the built in one shot timer.  Both of them wait for
    // status bit 0 to become 1, i.e. bit 0 is the completion latch:
    //   first loader  0x5E686: ldcb $0,0x404 / and $0,$2 / beqz $0,0x5E686
    //   second loader 0x45516: ldcb $3,0x404 / and3 $3,$3,1 / bnez $3,0x45520
    Fixture f;
    f.word(kCode, 0x0402F004u);            // stcb $0,0x402 (disable)
    f.word(kCode + 4, 0x0400F004u);        // stcb $0,0x400 (reload high byte = 0)
    f.word(kCode + 8, 0x0404F014u);        // ldcb $0,0x404 -> still counting
    f.run(3);
    show(f, "delay start");
    ZLB_EXPECT_EQ(f.cpu.r[0] & 1u, 0u);
    ZLB_EXPECT_FALSE(f.cpu.cbus.busy());
    ZLB_EXPECT_EQ(f.cpu.cbus.count, 0u);

    // Starting the timer over 0x402 and then letting the counter run out sets
    // the latch, which is what both poll loops wait for.
    Fixture h;
    h.cpu.cbus.write(0x400, 1);
    h.cpu.cbus.write(0x402, 1);
    ZLB_EXPECT_TRUE(h.cpu.cbus.busy());
    ZLB_EXPECT_EQ(h.cpu.cbus.count, 0x100u);   // 0x400 is the high byte
    ZLB_EXPECT_EQ(h.cpu.cbus.read(0x404) & 1u, 0u);
    for (int i = 0; i < 512; ++i) h.cpu.cbus.advance();
    ZLB_EXPECT_FALSE(h.cpu.cbus.busy());
    ZLB_EXPECT_EQ(h.cpu.cbus.read(0x404) & 1u, 1u);

    // Disabling the timer stops the count, and writing zero to the status
    // register clears the latch again.
    Fixture g;
    g.cpu.cbus.write(0x401, 4);            // low byte
    g.cpu.cbus.write(0x402, 1);
    ZLB_EXPECT_EQ(g.cpu.cbus.count, 4u);
    g.cpu.cbus.write(0x402, 0);            // disable
    ZLB_EXPECT_FALSE(g.cpu.cbus.busy());
    g.cpu.cbus.write(0x404, 0);            // clear the latch
    ZLB_EXPECT_EQ(g.cpu.cbus.read(0x404) & 1u, 0u);
}

ZLB_TEST(mep_coprocessor_word_moves) {
    // swcpi $c5,($6+) (0x3560) is a coprocessor store that post-increments the
    // base register by four.
    Fixture f;
    f.cpu.r[6] = 0x50000;
    f.word(kCode, 0x3560u);
    f.run(1);
    show(f, "swcpi");
    ZLB_EXPECT_EQ(f.cpu.r[6], 0x50004u);

    // swcp $c5,($6) (0x3568) does not.
    Fixture g;
    g.cpu.r[6] = 0x50000;
    g.word(kCode, 0x3568u);
    g.run(1);
    show(g, "swcp");
    ZLB_EXPECT_EQ(g.cpu.r[6], 0x50000u);
}

ZLB_TEST(mep_halt_and_undefined) {
    Fixture f;
    f.word(kCode, kHalt);
    StepResult result = f.cpu.step();
    ZLB_EXPECT_TRUE(f.cpu.halted);
    ZLB_EXPECT_FALSE(result.faulted);
    ZLB_EXPECT_TRUE((f.cpu.psw & (1u << 11)) != 0);
    ZLB_EXPECT_TRUE(f.cpu.halt_reason == std::string("halt instruction"));

    // `break` also stops the core.
    Fixture b;
    b.word(kCode, kBreak);
    StepResult broken = b.cpu.step();
    ZLB_EXPECT_TRUE(b.cpu.halted);
    ZLB_EXPECT_FALSE(broken.faulted);
    ZLB_EXPECT_TRUE(b.cpu.halt_reason == std::string("break exception"));

    // An encoding with no table entry faults and does not advance pc.
    Fixture g;
    g.word(kCode, kUnknown16);
    StepResult bad = g.cpu.step();
    ZLB_EXPECT_TRUE(bad.faulted);
    ZLB_EXPECT_TRUE(g.cpu.undefined_instruction);
    ZLB_EXPECT_EQ(g.cpu.pc, kCode);
    ZLB_EXPECT_EQ(bad.address, kCode);
    ZLB_EXPECT_EQ(bad.length, 4u);
    ZLB_EXPECT_EQ(g.cpu.instructions, 0u);
    ZLB_EXPECT_FALSE(g.cpu.halted);
}

ZLB_TEST(mep_register_access_and_status) {
    Fixture f;
    ZLB_EXPECT_TRUE(f.cpu.set_register("$0", 0x60000));
    ZLB_EXPECT_TRUE(f.cpu.set_register("sp", 0x5FFEC));
    ZLB_EXPECT_TRUE(f.cpu.set_register("$tp", 0x5E820));
    ZLB_EXPECT_TRUE(f.cpu.set_register("$gp", 0x5EB00));
    ZLB_EXPECT_TRUE(f.cpu.set_register("hi", 0xDEADBEEF));

    u64 value = 0;
    ZLB_EXPECT_TRUE(f.cpu.get_register("$0", value));
    ZLB_EXPECT_EQ(value, 0x60000u);
    ZLB_EXPECT_TRUE(f.cpu.get_register("$15", value));
    ZLB_EXPECT_EQ(value, 0x5FFECu);
    ZLB_EXPECT_TRUE(f.cpu.get_register("$13", value));
    ZLB_EXPECT_EQ(value, 0x5E820u);
    ZLB_EXPECT_TRUE(f.cpu.get_register("$14", value));
    ZLB_EXPECT_EQ(value, 0x5EB00u);
    ZLB_EXPECT_TRUE(f.cpu.get_register("hi", value));
    ZLB_EXPECT_EQ(value, 0xDEADBEEFu);
    ZLB_EXPECT_FALSE(f.cpu.get_register("nonsense", value));
    ZLB_EXPECT_FALSE(f.cpu.set_register("nonsense", 1));

    std::vector<zlb::RegValue> regs;
    f.cpu.registers(regs);
    int gpr = 0;
    int core = 0;
    for (const auto& reg : regs) {
        if (reg.group == "GPR") ++gpr;
        if (reg.group == "Core") ++core;
    }
    ZLB_EXPECT_EQ(gpr, 16);
    ZLB_EXPECT_EQ(core, 19);

    f.cpu.pc = 0x5C018;
    ZLB_EXPECT_TRUE(f.cpu.status_line() == std::string("pc=0x0005C018 psw=0x00000000 cond=0x00"));
    ZLB_EXPECT_TRUE(std::string(f.cpu.core_name()) == std::string("CMeP"));
    ZLB_EXPECT_TRUE(f.cpu.arch() == zlb::Arch::MeP);
}

ZLB_TEST(mep_disassembler_matches_the_boot_rom) {
    // The first instructions of the prototype first loader, verbatim from the
    // image and _scratch/mep_first_loader.log lines 10-26.  The stream is
    // contiguous, so the offsets account for mixed 16/32 bit lengths.
    Fixture f;
    // The stream is contiguous (mixed 16/32 bit lengths), so build it as a byte
    // image and load it in one go: writing the 32 bit words individually would
    // clobber the 16 bit instruction that follows each of them.
    static const u8 image[0x20] = {
        0x28, 0xD8, 0xC0, 0x05,  // 0x00: jmp 0x5c004
        0x00, 0xD2, 0xEB, 0x05,  // 0x04: movu $2,0x5eb00
        0x3C, 0xD3, 0xEE, 0x05,  // 0x08: movu $3,0x5ee3c
        0x00, 0x51,              // 0x0C: mov $1,0
        0x24, 0x03,              // 0x0E: sub $3,$2
        0x1A, 0x63,              // 0x10: srl $3,0x3
        0xFC, 0x63,              // 0x12: add $3,-1
        0x09, 0xE3, 0x03, 0x00,  // 0x14: repeat $3,0x5c01a
        0x2A, 0x01,              // 0x18: sw $1,($2)
        0x2A, 0xC1, 0x04, 0x00,  // 0x1A: sw $1,4($2)
        0x20, 0x62,              // 0x1E: add $2,8
    };
    f.bus.load(kCode, image, sizeof(image), "rom-head");

    struct Expect {
        u32 offset;
        unsigned length;
        const char* text;
    };
    // Branch targets are absolute, so the listing relocates with the image.
    static const Expect expected[] = {
        {0x00, 4, "jmp 0x5c004"},   // jmp is absolute
        {0x04, 4, "movu $2,0x5eb00"},
        {0x08, 4, "movu $3,0x5ee3c"},
        {0x0C, 2, "mov $1,0"},
        {0x0E, 2, "sub $3,$2"},
        {0x10, 2, "srl $3,0x3"},
        {0x12, 2, "add $3,-1"},
        {0x14, 4, "repeat $3,0x4001a"},
        {0x18, 2, "sw $1,($2)"},
        {0x1A, 4, "sw $1,4($2)"},
        {0x1E, 2, "add $2,8"},
    };
    for (const Expect& item : expected) {
        unsigned length = 0;
        const std::string text = mep_disassemble(f.bus, kCode + item.offset, length);
        if (text != std::string(item.text)) {
            std::printf("      +0x%02X: got '%s' want '%s'\n", item.offset, text.c_str(), item.text);
        }
        ZLB_EXPECT_EQ(length, item.length);
        ZLB_EXPECT_TRUE(text == std::string(item.text));
    }

    // The opwords themselves must be the ones the listing shows.
    ZLB_EXPECT_EQ(f.bus.read16(kCode + 0x18), 0x012Au);
    ZLB_EXPECT_EQ(f.bus.read32(kCode + 0x1A), 0x0004C12Au);

    // "mov $0,$0" is the architectural nop.
    Fixture g;
    g.word(kCode, 0x0000u);
    unsigned length = 0;
    ZLB_EXPECT_TRUE(mep_disassemble(g.bus, kCode, length) == std::string("nop"));
    ZLB_EXPECT_EQ(length, 2u);

    // Unknown encodings render as *unknown* and report 4 bytes.
    Fixture h;
    h.word(kCode, kUnknown16);
    ZLB_EXPECT_TRUE(mep_disassemble(h.bus, kCode, length) == std::string("*unknown*"));
    ZLB_EXPECT_EQ(length, 4u);
}

ZLB_TEST(mep_reset_and_reset_context) {
    Fixture f;
    f.cpu.r[0] = 0x1111;
    f.cpu.r[5] = 0x2222;
    f.cpu.hi = 0x3333;
    f.cpu.instructions = 42;
    f.cpu.reset(0x5C000);
    ZLB_EXPECT_EQ(f.cpu.pc, 0x5C000u);
    ZLB_EXPECT_EQ(f.cpu.r[5], 0u);
    ZLB_EXPECT_EQ(f.cpu.hi, 0u);
    ZLB_EXPECT_EQ(f.cpu.instructions, 0u);
    ZLB_EXPECT_FALSE(f.cpu.halted);

    f.cpu.prepare_reset_context(0x60000, 0x40000, 0, 0);
    ZLB_EXPECT_EQ(f.cpu.r[0], 0x60000u);   // $0 carries the boot stack pointer
    f.cpu.prepare_reset_context(0x5FFEC, 0, 0, 0);
    ZLB_EXPECT_EQ(f.cpu.r[0], 0x5FFECu);

    // reset() with no argument uses the architectural CMeP reset vector.
    f.cpu.reset();
    ZLB_EXPECT_EQ(f.cpu.pc, 0x00040000u);
}

ZLB_TEST(mep_reset_does_not_touch_devices) {
    // A core reset must not power-cycle the board: a register file standing in
    // for the ARM->CMeP mailbox keeps its value across reset(entry).
    Fixture f;
    auto device = std::make_unique<zlb::RegisterFile>("MailboxArmToCmep", 0xE0000010, 4);
    device->define(0xE0000010, "command", 0);
    zlb::RegisterFile* mailbox = device.get();
    f.bus.add_device(std::move(device));

    mailbox->poke(0xE0000010, 0x1234);
    f.cpu.reset(0x5C000);
    ZLB_EXPECT_EQ(mailbox->peek(0xE0000010), 0x1234u);
}
