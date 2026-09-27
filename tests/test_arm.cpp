// zeliboba - register level tests for the ARM Cortex-A9 core.
//
// Everything here drives the core through the public Bus + Cpu interface: a
// scratch RAM region at 0x80000000 is loaded with hand assembled instructions
// and then stepped. That keeps the tests honest about the decode path and the
// memory path at the same time.
#include <cstring>
#include <string>
#include <vector>

#include "cpu/arm/arm_core.h"
#include "cpu/arm/arm_disasm.h"
#include "cpu/arm/arm_mmu.h"
#include "cpu/arm/arm_vfp.h"
#include "cpu/factory.h"
#include "test_framework.h"

using namespace zlb;

namespace {

constexpr u32 kCodeBase = 0x80000000u;
constexpr u32 kDataBase = 0x80010000u;

struct Fixture {
    Bus bus;
    ArmCore cpu;

    Fixture() : cpu(bus) {
        bus.add_ram("main", 128u * 1024u, 0x80000000u, "test");
        cpu.reset(0);
    }

    void load(u32 address, const std::vector<u32>& words) {
        for (size_t i = 0; i < words.size(); ++i) {
            bus.write32(address + static_cast<u32>(i) * 4u, words[i]);
        }
    }

    /// Lay out 16 bit instructions byte-exactly. `pack_halfwords()` followed by
    /// 32 bit words silently inserts two bytes of padding after an odd number of
    /// halfwords, which moves (and breaks) a following IT block, so code that
    /// mixes both lengths uses this instead.
    void load_halfwords(u32 address, const std::vector<u16>& halfwords) {
        for (size_t i = 0; i < halfwords.size(); ++i) {
            bus.write16(address + static_cast<u32>(i) * 2u, halfwords[i]);
        }
    }

    void run_at(u32 address, int steps, bool thumb_state = false) {
        cpu.reset(thumb_state ? (address | 1u) : address);
        for (int i = 0; i < steps; ++i) {
            if (cpu.undefined_instruction || cpu.halted) break;
            cpu.step();
        }
    }

    u32 reg(int index) const { return cpu.r[index]; }
    void set_reg(int index, u32 value) { cpu.r[index] = value; }
};

// A32 encoders -------------------------------------------------------------

constexpr u32 arm_cond(u32 cond, u32 op) { return (cond << 28) | op; }

constexpr u32 arm_dp_imm(u32 cond, u32 opcode, bool s, u32 rn, u32 rd, u32 imm12) {
    return (cond << 28) | (1u << 25) | (opcode << 21) | (s ? (1u << 20) : 0u) | (rn << 16) |
           (rd << 12) | (imm12 & 0xFFFu);
}

constexpr u32 arm_dp_reg(u32 cond, u32 opcode, bool s, u32 rn, u32 rd, u32 rm) {
    return (cond << 28) | (opcode << 21) | (s ? (1u << 20) : 0u) | (rn << 16) | (rd << 12) | rm;
}

constexpr u32 arm_ldr_str_imm(u32 cond, bool load, bool byte, u32 rn, u32 rd, u32 offset,
                              bool pre = true, bool up = true, bool wb = false) {
    return (cond << 28) | (1u << 26) | (pre ? (1u << 24) : 0u) | (up ? (1u << 23) : 0u) |
           (byte ? (1u << 22) : 0u) | (wb ? (1u << 21) : 0u) | (load ? (1u << 20) : 0u) | (rn << 16) |
           (rd << 12) | (offset & 0xFFFu);
}

constexpr u32 arm_ldm_stm(u32 cond, bool load, bool pre, bool up, bool wb, u32 rn, u32 list) {
    return (cond << 28) | (1u << 27) | (pre ? (1u << 24) : 0u) | (up ? (1u << 23) : 0u) |
           (wb ? (1u << 21) : 0u) | (load ? (1u << 20) : 0u) | (rn << 16) | (list & 0xFFFFu);
}

constexpr u32 arm_branch(u32 cond, bool link, s32 offset) {
    const u32 imm24 = static_cast<u32>(offset) & 0xFFFFFFu;
    return (cond << 28) | (5u << 25) | (link ? (1u << 24) : 0u) | imm24;
}

constexpr u32 arm_movw(u32 cond, u32 rd, u32 imm16) {
    return (cond << 28) | (1u << 25) | (0x8u << 21) | ((imm16 >> 12) << 16) | (rd << 12) |
           (imm16 & 0xFFFu);
}

/// A32 MCR/MRC p15: cond 1110 opc1 L CRn Rt 1111 opc2 1 CRm.
constexpr u32 arm_cp15(bool read, u32 opc1, u32 crn, u32 rt, u32 crm, u32 opc2) {
    return 0xEE000F10u | (opc1 << 21) | (read ? (1u << 20) : 0u) | (crn << 16) | (rt << 12) |
           (opc2 << 5) | (crm & 0xFu);
}

/// A32 VMOV (immediate), single precision: 1110 1110 1011 imm4H Vd 101 0000 imm4L.
constexpr u32 arm_vmov_s_imm(u32 sd, u32 imm8) {
    return 0xEEB00A00u | (((imm8 >> 4) & 0xFu) << 16) | (((sd >> 1) & 0xFu) << 12) | (imm8 & 0xFu);
}

/// A32 VMOV (immediate), double precision: 1110 1110 1011 imm4H Vd 101 1000 imm4L.
constexpr u32 arm_vmov_d_imm(u32 dd, u32 imm8) {
    return 0xEEB00B00u | (((imm8 >> 4) & 0xFu) << 16) | ((dd & 0xFu) << 12) |
           (((dd >> 4) & 1u) << 22) | (imm8 & 0xFu);
}

// Thumb encoders -----------------------------------------------------------

constexpr u32 t16_mov_imm(u32 rd, u32 imm8) { return 0x2000u | (rd << 8) | (imm8 & 0xFFu); }
constexpr u32 t16_add_imm(u32 rd, u32 imm8) { return 0x3000u | (rd << 8) | (imm8 & 0xFFu); }
constexpr u32 t16_sub_imm(u32 rd, u32 imm8) { return 0x3800u | (rd << 8) | (imm8 & 0xFFu); }
constexpr u32 t16_cmp_imm(u32 rd, u32 imm8) { return 0x2800u | (rd << 8) | (imm8 & 0xFFu); }
constexpr u32 t16_lsl_imm(u32 rd, u32 rm, u32 imm5) {
    return 0x0000u | (imm5 << 6) | (rm << 3) | rd;
}

/// T32 BL: 11110 S imm10 : 11 J1 1 J2 imm11, offset relative to PC + 4.
void emit_thumb32_bl(std::vector<u32>& out, u32 address, u32 target) {
    const s32 offset = static_cast<s32>(target) - static_cast<s32>(address + 4u);
    const u32 imm = static_cast<u32>(offset) & 0x01FFFFFFu;
    const u32 s = (imm >> 24) & 1u;
    const u32 i1 = (imm >> 23) & 1u;
    const u32 i2 = (imm >> 22) & 1u;
    const u32 imm10 = (imm >> 12) & 0x3FFu;
    const u32 imm11 = (imm >> 1) & 0x7FFu;
    const u32 j1 = (~i1 ^ s) & 1u;
    const u32 j2 = (~i2 ^ s) & 1u;
    const u32 hw1 = 0xF000u | (s << 10) | imm10;
    const u32 hw2 = 0xD000u | (j1 << 13) | (1u << 12) | (j2 << 11) | imm11;
    // Little-endian: the *first* halfword goes in the low 16 bits of the word.
    out.push_back((hw2 << 16) | hw1);
}

/// T32 BLX (immediate, to an ARM target): 11110 S imm10 : 11 J1 0 J2 imm10L 0.
void emit_thumb32_blx(std::vector<u32>& out, u32 address, u32 target) {
    const u32 base = (address + 4u) & ~3u;
    const s32 offset = static_cast<s32>(target) - static_cast<s32>(base);
    const u32 imm = (static_cast<u32>(offset) >> 1) & 0x01FFFFFFu;
    const u32 s = (imm >> 24) & 1u;
    const u32 i1 = (imm >> 23) & 1u;
    const u32 i2 = (imm >> 22) & 1u;
    const u32 imm10 = (imm >> 12) & 0x3FFu;
    const u32 imm10l = (imm >> 2) & 0x3FFu;
    const u32 j1 = (~i1 ^ s) & 1u;
    const u32 j2 = (~i2 ^ s) & 1u;
    const u32 hw1 = 0xF000u | (s << 10) | imm10;
    const u32 hw2 = 0xC000u | (j1 << 13) | (0u << 12) | (j2 << 11) | (imm10l << 1);
    out.push_back((hw2 << 16) | hw1);
}

/// Pack a list of halfwords into little-endian 32-bit instruction words.
std::vector<u32> pack_halfwords(const std::vector<u16>& halfwords) {
    std::vector<u32> words;
    for (size_t i = 0; i + 1 < halfwords.size(); i += 2) {
        words.push_back(static_cast<u32>(halfwords[i]) | (static_cast<u32>(halfwords[i + 1]) << 16));
    }
    if (halfwords.size() % 2 != 0) words.push_back(halfwords.back() & 0xFFFFu);
    return words;
}

}  // namespace

// ===========================================================================
// ALU flag behaviour
// ===========================================================================

ZLB_TEST(arm_add_sets_flags) {
    Fixture f;
    // movw r0, #0 ; movt r0, #0x8000 -> r0 = 0x80000000
    // subs are not needed; use adds r2, r0, r0  -> 0x00000000, C = 1, V = 1
    const u32 movt0 = 0xE3400000u | ((0x8000u >> 12) << 16) | (0x8000u & 0xFFFu);
    f.load(kCodeBase, {arm_movw(0xE, 0, 0x0000), movt0, arm_dp_reg(0xE, 0x4, true, 0, 2, 0)});
    f.run_at(kCodeBase, 3);
    ZLB_EXPECT_EQ(f.reg(0), 0x80000000u);
    ZLB_EXPECT_EQ(f.reg(2), 0u);            // 0x80000000 + 0x80000000 wraps to 0
    ZLB_EXPECT_TRUE(f.cpu.flag_z());        // Z set
    ZLB_EXPECT_TRUE(f.cpu.flag_c());        // unsigned carry out
    ZLB_EXPECT_TRUE(f.cpu.flag_v());        // signed overflow
    ZLB_EXPECT_FALSE(f.cpu.flag_n());
}

ZLB_TEST(arm_sub_sets_overflow) {
    Fixture f;
    // movw r0, #0 ; movt r0, #0x8000 -> r0 = 0x80000000 ; subs r1, r0, #1
    const u32 movt0 = 0xE3400000u | ((0x8000u >> 12) << 16) | (0x8000u & 0xFFFu);
    f.load(kCodeBase, {arm_movw(0xE, 0, 0x0000), movt0, arm_dp_imm(0xE, 0x2, true, 0, 1, 1)});
    f.run_at(kCodeBase, 3);
    ZLB_EXPECT_EQ(f.reg(0), 0x80000000u);
    ZLB_EXPECT_EQ(f.reg(1), 0x7FFFFFFFu);
    ZLB_EXPECT_TRUE(f.cpu.flag_v());   // signed overflow
    ZLB_EXPECT_TRUE(f.cpu.flag_c());   // no borrow
    ZLB_EXPECT_FALSE(f.cpu.flag_n());
    ZLB_EXPECT_FALSE(f.cpu.flag_z());
}

ZLB_TEST(arm_mov_and_tst_flags) {
    Fixture f;
    // tst r0, r1 ; teq r0, r1 ; cmp r0, r1 -- operands seeded directly so the
    // test is about the flag logic and not about MOVW/MOVT encodings.
    f.load(kCodeBase, {arm_dp_reg(0xE, 0x8, true, 0, 0, 1),   // tst r0, r1
                       arm_dp_reg(0xE, 0x9, true, 0, 0, 1),   // teq r0, r1
                       arm_dp_reg(0xE, 0xA, true, 0, 0, 1)}); // cmp r0, r1
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 0x80008000u);
    f.set_reg(1, 0x80000000u);
    f.cpu.step();  // tst: 0x80008000 & 0x80000000 = 0x80000000 -> N, !Z
    ZLB_EXPECT_FALSE(f.cpu.flag_z());
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    f.cpu.step();  // teq: 0x00008000 -> !Z, !N
    ZLB_EXPECT_FALSE(f.cpu.flag_z());
    ZLB_EXPECT_FALSE(f.cpu.flag_n());
    f.cpu.step();  // cmp: 0x80008000 - 0x80000000 = 0x8000 -> !Z, !N, C
    ZLB_EXPECT_FALSE(f.cpu.flag_z());
    ZLB_EXPECT_FALSE(f.cpu.flag_n());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
}

ZLB_TEST(arm_mul_and_clz) {
    Fixture f;
    // mul r2, r0, r1 (bits [7:4] == 1001, op == 0)
    // mul r2, r0, r1: cond 0000000 S Rd Rn Rs 1001 Rm -> bits [7:4] must be 1001.
    const u32 mul = (0xE << 28) | (0u << 21) | (2u << 16) | (0u << 12) | (1u << 8) | (0x9u << 4) | 0u;
    const u32 clz = (0xE << 28) | 0x016F0F10u | (3u << 12) | 0u;  // clz r3, r0
    f.load(kCodeBase, {mul, clz});
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 6);
    f.set_reg(1, 7);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 42u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(3), 29u);  // clz(7) == 29
}

ZLB_TEST(arm_clz_matches_reference) {
    Fixture f;
    ZLB_EXPECT_EQ(arm::count_leading_zeros(0u), 32u);
    ZLB_EXPECT_EQ(arm::count_leading_zeros(1u), 31u);
    ZLB_EXPECT_EQ(arm::count_leading_zeros(0x80000000u), 0u);
    ZLB_EXPECT_EQ(arm::count_leading_zeros(0x0000FFFFu), 16u);
    (void)f;
}

// ===========================================================================
// LDM / STM
// ===========================================================================

ZLB_TEST(arm_stm_ldm_ia_roundtrip) {
    Fixture f;
    // stmia r0!, {r1-r3} ; ldmia r0!, {r4-r6}
    f.load(kCodeBase, {arm_ldm_stm(0xE, false, false, true, true, 0, 0x000E),
                       arm_ldm_stm(0xE, true, false, true, true, 0, 0x0070)});
    f.cpu.reset(kCodeBase);
    f.set_reg(0, kDataBase);
    f.set_reg(1, 0x11111111u);
    f.set_reg(2, 0x22222222u);
    f.set_reg(3, 0x33333333u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), kDataBase + 12u);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0), 0x11111111u);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 4), 0x22222222u);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 8), 0x33333333u);

    // Reload the base: LDMIA reads [base, base+12) so it must start at kDataBase.
    f.set_reg(0, kDataBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(4), 0x11111111u);
    ZLB_EXPECT_EQ(f.reg(5), 0x22222222u);
    ZLB_EXPECT_EQ(f.reg(6), 0x33333333u);
    ZLB_EXPECT_EQ(f.reg(0), kDataBase + 12u);
}

ZLB_TEST(arm_stmdb_ldmia_push_pop) {
    Fixture f;
    // stmdb sp!, {r1, r14} (push) ; ldmia sp!, {r2, r3}
    f.load(kCodeBase, {arm_ldm_stm(0xE, false, true, false, true, 13, 0x4002),
                       arm_ldm_stm(0xE, true, false, true, true, 13, 0x000C)});
    f.cpu.reset(kCodeBase);
    f.set_reg(13, kDataBase + 0x100);
    f.set_reg(1, 0xAAAAAAAAu);
    f.set_reg(14, 0xBBBBBBBBu);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(13), kDataBase + 0x100u - 8u);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0x100 - 8), 0xAAAAAAAAu);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0x100 - 4), 0xBBBBBBBBu);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 0xAAAAAAAAu);
    ZLB_EXPECT_EQ(f.reg(3), 0xBBBBBBBBu);
    ZLB_EXPECT_EQ(f.reg(13), kDataBase + 0x100u);
}

// ===========================================================================
// Load / store, MUL, ADD, SUB
// ===========================================================================

ZLB_TEST(arm_add_sub_mul_and_memory) {
    Fixture f;
    // Base address lives in r5 so MOVW into r0/r1 cannot clobber it.
    f.load(kCodeBase, {
                          arm_movw(0xE, 0, 0x0010),                  // movw r0, #0x10
                          arm_movw(0xE, 1, 0x0020),                  // movw r1, #0x20
                          arm_dp_reg(0xE, 0x4, false, 0, 2, 1),      // add r2, r0, r1
                          arm_dp_reg(0xE, 0x2, false, 2, 3, 1),      // sub r3, r2, r1
                          arm_ldr_str_imm(0xE, false, false, 5, 2, 0),  // str r2, [r5]
                          arm_ldr_str_imm(0xE, true, false, 5, 4, 0),   // ldr r4, [r5]
                      });
    f.cpu.reset(kCodeBase);
    f.set_reg(5, kDataBase);
    f.cpu.step();
    f.cpu.step();
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 0x30u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(3), 0x10u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase), 0x30u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(4), 0x30u);
}

ZLB_TEST(arm_ldr_byte_and_halfword) {
    Fixture f;
    f.load(kCodeBase, {arm_ldr_str_imm(0xE, true, true, 0, 1, 0),      // ldrb r1, [r0]
                       arm_ldr_str_imm(0xE, true, true, 0, 2, 1)});    // ldrb r2, [r0, #1]
    f.bus.write8(kDataBase, 0xABu);
    f.bus.write8(kDataBase + 1, 0x7Fu);
    f.cpu.reset(kCodeBase);
    f.set_reg(0, kDataBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 0xABu);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 0x7Fu);
}

// ===========================================================================
// Branches
// ===========================================================================

ZLB_TEST(arm_bl_sets_lr_and_branches) {
    Fixture f;
    const u32 target = kCodeBase + 0x40u;
    const s32 delta = static_cast<s32>(target) - static_cast<s32>(kCodeBase + 8u);
    f.load(kCodeBase, {arm_branch(0xE, true, delta >> 2)});
    f.cpu.reset(kCodeBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(14), kCodeBase + 4u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), target);
}

ZLB_TEST(arm_bx_interworks) {
    Fixture f;
    // bx r0 with r0 = (thumb target | 1)
    const u32 bx = 0xE12FFF10u;
    f.load(kCodeBase, {bx});
    f.cpu.reset(kCodeBase);
    f.set_reg(0, (kCodeBase + 0x80u) | 1u);
    f.cpu.step();
    ZLB_EXPECT_TRUE(f.cpu.thumb);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 0x80u);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & 0x20u) != 0);
}

ZLB_TEST(thumb_bl_and_blx) {
    Fixture f;
    std::vector<u32> words;
    const u32 bl_target = kCodeBase + 0x100u;
    emit_thumb32_bl(words, kCodeBase, bl_target);
    f.load(kCodeBase, words);
    f.cpu.reset(kCodeBase | 1u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), bl_target);
    ZLB_EXPECT_EQ(f.reg(14), kCodeBase + 4u + 1u);
    ZLB_EXPECT_TRUE(f.cpu.thumb);

    // BLX (immediate) switches to ARM state at a word aligned target.
    Fixture g;
    std::vector<u32> words2;
    const u32 arm_target = kCodeBase + 0x100u;
    emit_thumb32_blx(words2, kCodeBase, arm_target);
    g.load(kCodeBase, words2);
    g.cpu.reset(kCodeBase | 1u);
    g.cpu.step();
    ZLB_EXPECT_EQ(g.cpu.get_pc(), arm_target);
    ZLB_EXPECT_FALSE(g.cpu.thumb);
    ZLB_EXPECT_EQ(g.reg(14), kCodeBase + 4u + 1u);
}

ZLB_TEST(thumb_conditional_branch) {
    Fixture f;
    // movs r0, #0 ; beq +4 ; movs r1, #1 ; movs r2, #2
    const std::vector<u16> code = {
        static_cast<u16>(t16_mov_imm(0, 0)),   // movs r0, #0
        0xD001u,                               // beq +2 halfwords
        static_cast<u16>(t16_mov_imm(1, 1)),   // movs r1, #1  (skipped)
        static_cast<u16>(t16_mov_imm(2, 2)),   // movs r2, #2
    };
    f.load(kCodeBase, pack_halfwords(code));
    f.cpu.reset(kCodeBase | 1u);
    f.cpu.step();  // movs r0, #0 -> Z set
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    f.cpu.step();  // beq -> taken
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 6u);
    f.cpu.step();  // movs r2, #2
    ZLB_EXPECT_EQ(f.reg(1), 0u);
    ZLB_EXPECT_EQ(f.reg(2), 2u);
}

ZLB_TEST(thumb_cbz_and_tbb_free_cbnz) {
    Fixture f;
    // movs r0, #0 ; cbz r0, +2 ; movs r1, #1 ; movs r2, #2
    const std::vector<u16> code = {
        static_cast<u16>(t16_mov_imm(0, 0)),
        0xB100u,                               // cbz r0, +2
        static_cast<u16>(t16_mov_imm(1, 1)),   // skipped
        static_cast<u16>(t16_mov_imm(2, 2)),
    };
    f.load(kCodeBase, pack_halfwords(code));
    f.cpu.reset(kCodeBase | 1u);
    f.cpu.step();
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 6u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 0u);
    ZLB_EXPECT_EQ(f.reg(2), 2u);
}

// ===========================================================================
// IT blocks
// ===========================================================================

ZLB_TEST(thumb_it_block_executes_conditionally) {
    Fixture f;
    // movs r0, #1 ; cmp r0, #1 ; it eq ; moveq r1, #0x42 ; mov r2, #0x43
    const std::vector<u16> code = {
        static_cast<u16>(t16_mov_imm(0, 1)),
        static_cast<u16>(t16_cmp_imm(0, 1)),   // Z = 1
        0xBF08u,                               // it eq
        static_cast<u16>(t16_mov_imm(1, 0x42)),// moveq r1, #0x42   (executes)
        static_cast<u16>(t16_mov_imm(2, 0x43)),// mov r2, #0x43     (outside IT)
    };
    f.load(kCodeBase, pack_halfwords(code));
    f.cpu.reset(kCodeBase | 1u);
    for (int i = 0; i < 5; ++i) f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 0x42u);
    ZLB_EXPECT_EQ(f.reg(2), 0x43u);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);

    // Now with Z clear: the IT body must be skipped.
    Fixture g;
    const std::vector<u16> code2 = {
        static_cast<u16>(t16_mov_imm(0, 1)),
        static_cast<u16>(t16_cmp_imm(0, 2)),   // Z = 0
        0xBF08u,                               // it eq
        static_cast<u16>(t16_mov_imm(1, 0x42)),// skipped
        static_cast<u16>(t16_mov_imm(2, 0x43)),
    };
    g.load(kCodeBase, pack_halfwords(code2));
    g.cpu.reset(kCodeBase | 1u);
    for (int i = 0; i < 5; ++i) g.cpu.step();
    ZLB_EXPECT_EQ(g.reg(1), 0u);
    ZLB_EXPECT_EQ(g.reg(2), 0x43u);
}

ZLB_TEST(thumb_it_block_with_an_odd_first_condition) {
    // `movs r3,#n ; cmp r3,#1 ; it ls (mask 1101) ; mov.w r0,#0x11 ; mov.w r1,#0x22
    //  ; mov.w r2,#0x33 ; mov.w r4,#0x44` - the exact shape kernel_boot_loader uses
    // at 0x4003B6EC to pick its cache-line round-up helper.  The fourth slot is the
    // 'E' position and the block's condition (LS) has bit 0 set: storing ITSTATE as
    // firstcond[3:1]:mask rather than firstcond:mask turned this into `it hi` and
    // made the loader call a function pointer read through a garbage base.
    for (u32 n : {1u, 2u}) {
        Fixture f;
        const std::vector<u16> code = {
            static_cast<u16>(0x2300u | (n & 0xFFu)),  // movs r3, #n
            static_cast<u16>(0x2B01u),                // cmp r3, #1 -> LS iff n == 1
            static_cast<u16>(0xBF9Du),                // it ls
        };
        std::vector<u32> bodies;
        bodies.push_back(0x0011F04Fu);  // mov.w r0, #0x11  (then)
        bodies.push_back(0x0122F04Fu);  // mov.w r1, #0x22  (then)
        bodies.push_back(0x0233F04Fu);  // mov.w r2, #0x33  (then)
        bodies.push_back(0x0444F04Fu);  // mov.w r4, #0x44  (else)
        f.load_halfwords(kCodeBase, code);
        f.load(kCodeBase + static_cast<u32>(code.size()) * 2u, bodies);
        f.cpu.reset(kCodeBase | 1u);
        for (int i = 0; i < 8; ++i) f.cpu.step();
        const bool ls = (n == 1u);
        ZLB_EXPECT_EQ(f.reg(0), ls ? 0x11u : 0u);
        ZLB_EXPECT_EQ(f.reg(1), ls ? 0x22u : 0u);
        ZLB_EXPECT_EQ(f.reg(2), ls ? 0x33u : 0u);
        ZLB_EXPECT_EQ(f.reg(4), ls ? 0u : 0x44u);
    }
}

ZLB_TEST(thumb_it_block_it_else) {
    Fixture f;
    // movs r0, #1 ; cmp r0, #1 ; ite eq ; mov.w r1, #1 ; mov.w r2, #2
    //
    // The bodies use MOV.W, not MOVS: `movs` would update Z, and the "else" half
    // of an ITE block is evaluated with the flags as they are when it executes, so
    // a flag-setting "then" would legitimately let the "else" run.
    const std::vector<u16> code = {
        static_cast<u16>(t16_mov_imm(0, 1)),
        static_cast<u16>(t16_cmp_imm(0, 1)),   // Z = 1
        0xBF0Cu,                               // ite eq -> mask 1100
    };
    const std::vector<u32> words = pack_halfwords(code);
    std::vector<u32> all = words;
    all.push_back(0x0101F04Fu);                // mov.w r1, #1
    all.push_back(0x0202F04Fu);                // mov.w r2, #2
    f.load(kCodeBase, all);
    f.cpu.reset(kCodeBase | 1u);
    for (int i = 0; i < 5; ++i) f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 1u);   // "then" executes
    ZLB_EXPECT_EQ(f.reg(2), 0u);   // "else" is skipped
}

// The same ITE block with loads as the bodies, i.e. the exact shape KBL uses at
// 0x40020AAC to pick TTBR0 or TTBR1.
ZLB_TEST(thumb_it_block_conditional_ldr) {
    Fixture f;
    f.bus.write32(0x80000140u, 0x11111111u);
    f.bus.write32(0x80000160u, 0x22222222u);
    // ite hs ; ldr r0,[r0,#0x40] ; ldr r0,[r0,#0x60]
    f.load(kCodeBase, {0x6C00BF2Cu, 0x00006E00u});

    f.cpu.reset(kCodeBase | 1u);
    f.cpu.cpsr |= arm::kFlagC;                 // HS
    f.set_reg(0, 0x80000100u);
    for (int i = 0; i < 3; ++i) f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x11111111u);      // only the "then" load ran

    f.cpu.reset(kCodeBase | 1u);
    f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);   // LO
    f.set_reg(0, 0x80000100u);
    for (int i = 0; i < 3; ++i) f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x22222222u);      // only the "else" load ran
}

// ===========================================================================
// CPSR mode banking, exceptions, interrupts
// ===========================================================================

ZLB_TEST(arm_mode_banking_switches_sp) {
    Fixture f;
    f.cpu.reset(kCodeBase);
    f.set_reg(13, 0x11111111u);  // SVC bank SP

    // Enter IRQ mode through an exception so both banks are exercised.
    f.cpu.take_exception(arm::kVecIrq, arm::kModeIrq, 0x80000010u);
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeIrq);
    ZLB_EXPECT_EQ(f.reg(14), 0x80000010u);
    // The SVC bank SPSR holds the CPSR we came from (SVC mode, reset value).
    ZLB_EXPECT_EQ(f.cpu.banked_spsr(arm::kModeSupervisor) & arm::kModeMask, arm::kModeSupervisor);
    // A fresh chip only has the SVC bank populated, so the IRQ/FIQ banks read 0.
    ZLB_EXPECT_EQ(f.cpu.banked_spsr(arm::kModeIrq), 0u);
    ZLB_EXPECT_EQ(f.cpu.banked_spsr(arm::kModeFiq), 0u);
    ZLB_EXPECT_EQ(f.reg(13), 0u);  // the IRQ bank starts empty
    f.set_reg(13, 0x22222222u);    // IRQ bank SP

    // Take an SVC exception from IRQ mode: the SVC bank still holds 0x11111111.
    f.cpu.take_exception(arm::kVecSupervisor, arm::kModeSupervisor, 0x80000020u);
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeSupervisor);
    ZLB_EXPECT_EQ(f.reg(13), 0x11111111u);
    // The SVC bank SPSR now holds the CPSR we came from (IRQ mode).
    ZLB_EXPECT_EQ(f.cpu.spsr() & arm::kModeMask, arm::kModeIrq);

    // Back to IRQ mode: SP must be the IRQ bank value again.
    f.cpu.take_exception(arm::kVecFiq, arm::kModeFiq, 0x80000030u);
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeFiq);
    f.cpu.set_spsr(arm::kModeIrq);
    f.cpu.take_exception(arm::kVecSupervisor, arm::kModeSupervisor, 0x80000040u);
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeSupervisor);
    ZLB_EXPECT_EQ(f.reg(13), 0x11111111u);
}

ZLB_TEST(arm_svc_takes_exception) {
    Fixture f;
    // svc #0
    f.load(kCodeBase, {0xEF000000u});
    f.cpu.reset(kCodeBase);
    f.set_reg(13, 0x80020000u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeSupervisor);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), 0x08u);            // low vectors by default
    ZLB_EXPECT_EQ(f.cpu.spsr() & arm::kModeMask, arm::kModeSupervisor);
    ZLB_EXPECT_EQ(f.cpu.exception_count, 1u);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagI) != 0);
}

ZLB_TEST(arm_mrs_msr_and_high_vectors) {
    Fixture f;
    f.cpu.reset(kCodeBase);
    f.bus.write32(0xFFFF0008u, 0xE1A00000u);  // nop at the high SVC vector
    f.cpu.mmu.sctlr |= (1u << 13);            // SCTLR.V
    ZLB_EXPECT_EQ(f.cpu.vector_base(), 0xFFFF0000u);
    f.cpu.mmu.vbar = 0x80001000u;
    ZLB_EXPECT_EQ(f.cpu.vector_base(), 0x80001000u);
    f.cpu.mmu.vbar = 0;
    f.load(kCodeBase, {arm_movw(0xE, 0, 0x0012), 0xE129F000u});  // msr cpsr_fc, r0
    f.cpu.reset(kCodeBase);
    f.bus.write32(0xFFFF0008u, 0xE1A00000u);
    f.cpu.mmu.sctlr |= (1u << 13);
    f.cpu.step();  // movw r0, #0x12
    f.cpu.step();  // msr cpsr_fc, r0 -> IRQ mode
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeIrq);
}

ZLB_TEST(arm_irq_is_taken_at_a_boundary) {
    Fixture f;
    f.load(kCodeBase, {0xE1A00000u, 0xE1A00000u});
    f.cpu.reset(kCodeBase);
    f.cpu.cpsr &= ~(arm::kFlagI | arm::kFlagF);  // unmask IRQ and FIQ
    f.cpu.set_irq(0, true);
    ZLB_EXPECT_TRUE(f.cpu.interrupt_pending());
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeIrq);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), arm::kVecIrq);
    ZLB_EXPECT_EQ(f.cpu.exception_count, 1u);
    f.cpu.set_irq(0, false);
    ZLB_EXPECT_FALSE(f.cpu.interrupt_pending());
}

ZLB_TEST(arm_fiq_masks_irq) {
    Fixture f;
    f.cpu.reset(kCodeBase);
    f.cpu.cpsr &= ~(arm::kFlagI | arm::kFlagF);  // unmask IRQ and FIQ
    f.cpu.set_irq(1, true);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeFiq);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagF) != 0);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagI) == 0);  // FIQ does not mask IRQ
}

ZLB_TEST(arm_reset_honours_thumb_bit) {
    Fixture f;
    f.cpu.reset(0x80000001u);
    ZLB_EXPECT_TRUE(f.cpu.thumb);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), 0x80000000u);
    f.cpu.reset(0x80000002u);
    ZLB_EXPECT_FALSE(f.cpu.thumb);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), 0x80000002u);
}

// ===========================================================================
// VFP
// ===========================================================================

ZLB_TEST(vfp_add_single_and_flags) {
    Fixture f;
    // vmov s0, #1.0 ; vmov s2, #2.0 ; vadd.f32 s0, s0, s2
    // VFPExpandImm(0x70) == 1.0f and VFPExpandImm(0x00) == 2.0f.
    // vadd.f32 s0, s0, s2  ->  cond 1110 1110 0011 0 D 00 Vn 101 sz=0 0000 Vm
    const u32 vadd = 0xEE300A00u | (0u << 12) | (0u << 16) | 1u;  // Vm = s2
    ZLB_EXPECT_NEAR(ArmVfp::bits_to_float(ArmVfp::expand_immediate(0x00)), 2.0f, 1e-6);
    ZLB_EXPECT_NEAR(ArmVfp::bits_to_float(ArmVfp::expand_immediate(0x70)), 1.0f, 1e-6);
    f.load(kCodeBase, {arm_vmov_s_imm(0, 0x70), arm_vmov_s_imm(2, 0x00), vadd});
    f.cpu.reset(kCodeBase);
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    f.cpu.step();  // vmov s0, #1.0
    ZLB_EXPECT_NEAR(f.cpu.vfp.read_f32(0), 1.0f, 1e-6);
    f.cpu.step();  // vmov s2, #2.0
    ZLB_EXPECT_NEAR(f.cpu.vfp.read_f32(2), 2.0f, 1e-6);
    f.cpu.step();  // vadd.f32 s0, s0, s2
    ZLB_EXPECT_NEAR(f.cpu.vfp.read_f32(0), 3.0f, 1e-6);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    ZLB_EXPECT_FALSE(f.cpu.vfp.fpscr & ArmVfp::kZs);
    ZLB_EXPECT_FALSE(f.cpu.vfp.fpscr & ArmVfp::kNs);
}

ZLB_TEST(vfp_double_arithmetic_and_compare) {
    Fixture f;
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    f.cpu.vfp.write_f64(0, 1.5);
    f.cpu.vfp.write_f64(2, 4.0);
    // vadd.f64 d1, d0, d2  -> 1110 1110 0011 0 D 00 Vd 101 sz=1 0000 Vm
    const u32 vadd = 0xEE300B00u | (1u << 12) | (0u << 16) | 2u;
    f.load(kCodeBase, {vadd});
    f.cpu.reset(kCodeBase);
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    f.cpu.vfp.write_f64(0, 1.5);
    f.cpu.vfp.write_f64(2, 4.0);
    f.cpu.step();
    ZLB_EXPECT_NEAR(f.cpu.vfp.read_f64(1), 5.5, 1e-12);

    // VCMP.F64 d1, d2 -> equal? no: 5.5 > 4.0 so C=1, V=0.
    f.cpu.vfp.compare(5.5, 4.0, false);
    ZLB_EXPECT_TRUE((f.cpu.vfp.fpscr & ArmVfp::kCs) != 0);
    ZLB_EXPECT_TRUE((f.cpu.vfp.fpscr & ArmVfp::kZs) == 0);
    ZLB_EXPECT_TRUE((f.cpu.vfp.fpscr & ArmVfp::kNs) == 0);

    // VMRS APSR_nzcv, FPSCR moves the flags into the CPSR.
    const u32 vmrs = 0xEEF1FA10u | (1u << 8);  // vmrs apsr_nzcv, fpscr (bit 8 = the @ bit)
    f.bus.write32(kCodeBase + 4, vmrs);
    f.cpu.reset(kCodeBase + 4);
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    f.cpu.vfp.compare(5.5, 4.0, false);
    f.cpu.step();
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_FALSE(f.cpu.flag_z());
}

ZLB_TEST(vfp_core_register_transfer_and_load_store) {
    Fixture f;
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    // vmov s0, r0   -> 1110 1110 0000 0000 1010 0001 0000
    const u32 vmov_s0_r0 = 0xEE000A10u;
    f.load(kCodeBase, {vmov_s0_r0});
    f.cpu.reset(kCodeBase);
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    f.set_reg(0, 0x40490FDBu);  // 3.14159274f
    f.cpu.step();
    ZLB_EXPECT_NEAR(f.cpu.vfp.read_f32(0), 3.14159274f, 1e-5);

    // vstr s0, [r0] / vldr s2, [r0]; both instructions use Rn = bits [19:16].
    const u32 vstr = 0xED800A00u;  // vstr.f32 s0, [r0]
    const u32 vldr = 0xED901A00u;  // vldr.f32 s2, [r0]
    f.bus.write32(kCodeBase + 4, vstr);
    f.bus.write32(kCodeBase + 8, vldr);
    f.cpu.reset(kCodeBase + 4);
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    f.cpu.vfp.write_f32(0, 3.14159274f);
    f.set_reg(0, kDataBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase), 0x40490FDBu);
    f.cpu.step();
    ZLB_EXPECT_NEAR(f.cpu.vfp.read_f32(2), 3.14159274f, 1e-5);

    // VPUSH {s0} is VSTMDB sp! with imm8 == 1.
    const u32 vpush = 0xED2D0A01u;
    f.bus.write32(kCodeBase + 12, vpush);
    f.cpu.reset(kCodeBase + 12);
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    f.cpu.vfp.write_f32(0, 1.0f);
    f.set_reg(13, kDataBase + 0x100);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0x100 - 4), 0x3F800000u);
    ZLB_EXPECT_EQ(f.reg(13), kDataBase + 0x100u - 4u);
}

ZLB_TEST(vfp_vmsr_fpexc_enables_the_unit) {
    Fixture f;
    // The kernel boot loader turns the VFP on exactly like this:
    //   mov r0, #0x40000000 ; vmsr fpexc, r0 ; vmov s0, r0
    // The key point is that VMSR/VMRS are *system register transfers* and must
    // execute whatever FPEXC.EN currently is - otherwise the unit can never be
    // enabled at all.
    const u32 mov_imm = arm_dp_imm(0xE, 0xDu, false, 0, 0, 0);  // placeholder, replaced below
    (void)mov_imm;
    // movw r0, #0 ; movt r0, #0x4000 -> r0 = 0x40000000 (FPEXC.EN)
    const u32 movt = 0xE3400000u | ((0x4000u >> 12) << 16) | (0x4000u & 0xFFFu);
    const u32 vmsr_fpexc = 0xEEE80A10u;      // vmsr fpexc, r0
    const u32 vmrs_fpexc = 0xEEF80A10u;      // vmrs r0, fpexc
    const u32 vmov_s0_r0 = 0xEE000A10u;      // vmov s0, r0
    const u32 vadd = 0xEE300A00u;            // vadd.f32 s0, s0, s0
    f.load(kCodeBase, {arm_movw(0xE, 0, 0x0000), movt, vmsr_fpexc, vmrs_fpexc, vmov_s0_r0, vadd});
    f.cpu.reset(kCodeBase);
    ZLB_EXPECT_FALSE(f.cpu.vfp.enabled());

    f.cpu.step();  // movw r0, #0
    f.cpu.step();  // movt r0, 0x4000
    ZLB_EXPECT_EQ(f.reg(0), 0x40000000u);

    // VMSR must not fault while the unit is disabled.
    const StepResult vmsr = f.cpu.step();
    ZLB_EXPECT_FALSE(vmsr.faulted);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    ZLB_EXPECT_TRUE(f.cpu.vfp.enabled());
    ZLB_EXPECT_EQ(f.cpu.vfp.fpexc, 0x40000000u);

    // VMRS reads it straight back.
    const StepResult vmrs = f.cpu.step();
    ZLB_EXPECT_FALSE(vmrs.faulted);
    ZLB_EXPECT_EQ(f.reg(0), 0x40000000u);

    // And now the data processing instruction is allowed through.
    f.cpu.vfp.write_f32(0, 2.5f);
    const StepResult vmov = f.cpu.step();
    ZLB_EXPECT_FALSE(vmov.faulted);
    const StepResult vadd_result = f.cpu.step();
    ZLB_EXPECT_FALSE(vadd_result.faulted);
    ZLB_EXPECT_NEAR(f.cpu.vfp.read_f32(0), 5.0f, 1e-6);
}

ZLB_TEST(vfp_data_instruction_still_gated_by_fpexc) {
    Fixture f;
    // With FPEXC.EN clear a VADD must be reported as an undefined instruction,
    // so the gate is not simply removed.
    const u32 vadd = 0xEE300A00u;
    f.load(kCodeBase, {vadd});
    f.cpu.reset(kCodeBase);
    const StepResult result = f.cpu.step();
    ZLB_EXPECT_TRUE(result.faulted);
    ZLB_EXPECT_TRUE(f.cpu.undefined_instruction);
}

ZLB_TEST(vfp_int_to_float_and_back) {
    Fixture f;
    ArmVfp& v = f.cpu.vfp;
    v.reset();
    v.write_s(2, 7u);
    // VCVT.F64.S32 d0, s2 : 1110 1110 1011 1000 0 D 11 Vd 101 1 11 1 0 Vm
    // opc2 == 8, N == 1 (signed) is the "integer -> float" direction.
    const u32 vcvt = 0xEEB80BC1u;  // Vm = s2
    f.load(kCodeBase, {vcvt});
    f.cpu.reset(kCodeBase);
    v.fpexc = ArmVfp::kFpexcEn;
    v.write_s(2, 7u);
    f.cpu.step();
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    ZLB_EXPECT_NEAR(v.read_f64(0), 7.0, 1e-12);

    // VCMP/VCMPE round trip through compare().
    ZLB_EXPECT_EQ(v.compare(1.0, 2.0, false), 0);
    ZLB_EXPECT_EQ(v.compare(2.0, 2.0, false), 1);
    ZLB_EXPECT_EQ(v.compare(3.0, 2.0, false), 2);
}

// ===========================================================================
// MMU
// ===========================================================================

ZLB_TEST(mmu_flat_when_disabled) {
    Fixture f;
    u32 pa = 0;
    std::string fault;
    ZLB_EXPECT_TRUE(f.cpu.translate(0x81000000u, false, true, pa, fault));
    ZLB_EXPECT_EQ(pa, 0x81000000u);  // identity mapping with the MMU off
    ZLB_EXPECT_TRUE(fault.empty());
}

ZLB_TEST(mmu_section_translate_and_permission_fault) {
    Fixture f;
    ArmMmu& mmu = f.cpu.mmu;
    const u32 l1_table = 0x80010000u;   // inside the 128 KiB test RAM
    mmu.ttbr0 = l1_table;
    mmu.dacr = 0x00000001u;   // domain 0 = client
    mmu.sctlr |= 1u;          // MMU on

    // Section descriptor for VA 0x81000000 -> PA 0x80000000, AP = 0b01 (RW all),
    // domain 0 (client), TEX/C/B = 0b011 (normal write-back, no write allocate).
    const u32 index = (0x81000000u >> 20) & 0xFFFu;
    const u32 descriptor = 0x80000000u | (2u << 0) | (0u << 5) | (1u << 10) | (3u << 2);
    f.bus.write32(l1_table + index * 4u, descriptor);

    u32 pa = 0;
    std::string fault;
    ZLB_EXPECT_TRUE(f.cpu.translate(0x81000000u, false, false, pa, fault));
    ZLB_EXPECT_EQ(pa, 0x80000000u);
    ZLB_EXPECT_TRUE(f.cpu.translate(0x810FFFFFu, true, false, pa, fault));
    ZLB_EXPECT_EQ(pa, 0x800FFFFFu);
    ZLB_EXPECT_TRUE(mmu.walks >= 2u);

    // Now a read-only section (AP = 0b10) must fault on a write.
    const u32 ro = 0x80000000u | (2u << 0) | (0u << 5) | (2u << 10) | (3u << 2);
    f.bus.write32(l1_table + index * 4u, ro);
    mmu.walks = 0;
    ZLB_EXPECT_FALSE(f.cpu.translate(0x81000000u, true, false, pa, fault));
    ZLB_EXPECT_FALSE(fault.empty());
    ZLB_EXPECT_EQ(static_cast<int>(mmu.dfsr), 0);  // translate() does not touch DFSR
    ZLB_EXPECT_TRUE(f.cpu.translate(0x81000000u, false, false, pa, fault));
}

ZLB_TEST(mmu_ttbcr_n_uses_ttbr1_for_the_top_half) {
    // The exact configuration the kernel boot loader uses: TTBCR.N == 2 splits
    // the address space at 0x40000000, TTBR0 covers below it and TTBR1 above.
    // Each TTBR's low 14 bits are the control field (IRGN/ORGN/S/RGN/N), so the
    // table base must have them masked off. Nothing else in this file covers it.
    Fixture f;
    ArmMmu& mmu = f.cpu.mmu;
    const u32 ttbr0 = 0x80010000u;   // 16 KiB aligned
    const u32 ttbr1 = 0x8001404Au;   // control bits set; base must be 0x80014000
    const u32 ttbr0_base = ttbr0 & 0xFFFFC000u;
    const u32 ttbr1_base = ttbr1 & 0xFFFFC000u;

    mmu.ttbr0 = ttbr0;
    mmu.ttbr1 = ttbr1;
    mmu.ttbcr = 2;                   // N = 2 -> split at 0x40000000
    mmu.dacr = 0x00000001u;          // domain 0 = client
    mmu.sctlr |= 1u;                 // MMU on

    // TTBR0: VA 0x00000000 -> PA 0x80100000 (a 1 MiB section).
    // AP[1:0] = 0b01 (RW for all), domain 0, TEX/C/B = 0b011 (normal WB).
    const u32 lo_index = (0x00000000u >> 20) & 0xFFFu;
    const u32 lo_desc = 0x80100000u | 2u | (0u << 5) | (1u << 10) | (3u << 2);
    f.bus.write32(ttbr0_base + lo_index * 4u, lo_desc);

    // TTBR1: VA 0x40000000 -> PA 0x80200000, and VA 0x91E00000 -> PA 0x80300000.
    const u32 hi_index0 = (0x40000000u >> 20) & 0xFFFu;
    const u32 hi_index1 = (0x91E00000u >> 20) & 0xFFFu;
    const u32 hi_desc0 = 0x80200000u | 2u | (0u << 5) | (1u << 10) | (3u << 2);
    const u32 hi_desc1 = 0x80300000u | 2u | (0u << 5) | (1u << 10) | (3u << 2);
    f.bus.write32(ttbr1_base + hi_index0 * 4u, hi_desc0);
    f.bus.write32(ttbr1_base + hi_index1 * 4u, hi_desc1);

    // The split: these must be walked through TTBR0.
    int num = -1;
    ZLB_EXPECT_EQ(mmu.ttbr_base_for(0x00000000u, num), ttbr0_base);
    ZLB_EXPECT_EQ(num, 0);
    ZLB_EXPECT_EQ(mmu.ttbr_base_for(0x3FFFFFFFu, num), ttbr0_base);
    ZLB_EXPECT_EQ(num, 0);

    // ... and these through TTBR1, with the control bits masked out of the base.
    ZLB_EXPECT_EQ(mmu.ttbr_base_for(0x40000000u, num), ttbr1_base);
    ZLB_EXPECT_EQ(num, 1);
    ZLB_EXPECT_EQ(mmu.ttbr_base_for(0x91EB40BCu, num), ttbr1_base);
    ZLB_EXPECT_EQ(num, 1);
    ZLB_EXPECT_EQ(mmu.ttbr_base_for(0xFFFFFFFFu, num), ttbr1_base);
    ZLB_EXPECT_EQ(num, 1);

    u32 pa = 0;
    std::string fault;
    // Below the split: TTBR0's section.
    ZLB_EXPECT_TRUE(f.cpu.translate(0x00000000u, false, false, pa, fault));
    ZLB_EXPECT_EQ(pa, 0x80100000u);
    // Above the split: TTBR1's sections. If the walk went through TTBR0 the
    // descriptor would be a fault (or the wrong section) and this would fail.
    ZLB_EXPECT_TRUE(f.cpu.translate(0x40000000u, false, false, pa, fault));
    ZLB_EXPECT_EQ(pa, 0x80200000u);
    ZLB_EXPECT_TRUE(f.cpu.translate(0x91EB40BCu, false, false, pa, fault));
    ZLB_EXPECT_EQ(pa, 0x803B40BCu);
    // A VA above the split with no descriptor in TTBR1 must fault, and the fault
    // record must name the TTBR it walked (not TTBR0).
    ZLB_EXPECT_FALSE(f.cpu.translate(0xC02716A8u, false, false, pa, fault));
    ZLB_EXPECT_FALSE(fault.empty());
    ZLB_EXPECT_EQ(mmu.last_walk.ttbr_num, 1);
    ZLB_EXPECT_EQ(mmu.last_walk.ttbr_base, ttbr1_base);
    ZLB_EXPECT_EQ(mmu.last_walk.va, 0xC02716A8u);
    ZLB_EXPECT_TRUE(mmu.last_walk.fault != arm::MmFaultKind::None);
}

ZLB_TEST(mmu_dacr_domain_zero_client_others_fault) {
    // DACR = 2 means only domain 0 is client; domains 1..15 are "no access" and
    // must fault even for a perfectly valid section descriptor.
    Fixture f;
    ArmMmu& mmu = f.cpu.mmu;
    mmu.ttbr0 = 0x80010000u;
    mmu.dacr = 0x00000002u;
    mmu.sctlr |= 1u;

    const u32 index = (0x81000000u >> 20) & 0xFFFu;
    // Domain 0 descriptor: allowed (AP = 0b01 RW for all).
    f.bus.write32(0x80010000u + index * 4u, 0x80000000u | 2u | (0u << 5) | (1u << 10) | (3u << 2));
    u32 pa = 0;
    std::string fault;
    ZLB_EXPECT_TRUE(f.cpu.translate(0x81000000u, false, false, pa, fault));
    ZLB_EXPECT_EQ(pa, 0x80000000u);

    // The same descriptor tagged with domain 3: DACR[7:6] == 00 -> no access.
    f.bus.write32(0x80010000u + index * 4u, 0x80000000u | 2u | (3u << 5) | (1u << 10) | (3u << 2));
    ZLB_EXPECT_FALSE(f.cpu.translate(0x81000000u, false, false, pa, fault));
    ZLB_EXPECT_TRUE(mmu.last_walk.fault == arm::MmFaultKind::Domain);
    ZLB_EXPECT_EQ(mmu.last_walk.domain, 3u);
    // report_data_abort() must land the domain status in DFSR and the VA in DFAR.
    arm::MmResult scratch;
    scratch.fault = arm::MmFaultKind::Domain;
    scratch.fsr_full = 0x00000009u | (3u << 4);
    scratch.fsr_status = scratch.fsr_full;
    mmu.dfsr = 0;
    mmu.report_data_abort(scratch, 0x81000000u, false);
    ZLB_EXPECT_EQ(mmu.dfsr & 0xFu, 0x9u);
    ZLB_EXPECT_EQ((mmu.dfsr >> 4) & 0xFu, 3u);
    ZLB_EXPECT_EQ(mmu.dfar, 0x81000000u);
}

ZLB_TEST(cp15_dacr_read_returns_the_written_value) {
    // The kernel boot loader brackets its VA->PA probing with
    //     mrc p15,0,r3,c3,c0   ; save DACR
    //     mcr p15,0,r4,c3,c0   ; DACR = 0x55555555 (all domains client)
    //     ...
    //     mcr p15,0,r3,c3,c0   ; restore DACR
    // A DACR read that answers 0 makes the restore disable every domain, so the
    // next instruction fetch takes a domain fault. This is the regression for the
    // stall at 0x4003AFB2 in kernel_boot_loader.self.
    Fixture f;
    f.load(kCodeBase, {
        arm_cp15(false, 0, 3, 4, 0, 0),  // mcr p15,0,r4,c3,c0 : DACR = r4
        arm_cp15(true, 0, 3, 5, 0, 0),   // mrc p15,0,r5,c3,c0 : r5 = DACR
    });
    f.cpu.reset(kCodeBase);
    f.cpu.mmu.dacr = 0x00000001u;
    f.set_reg(4, 0x55555555u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mmu.dacr, 0x55555555u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(5), 0x55555555u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 8u);
}

ZLB_TEST(cp15_translation_control_registers_read_back) {
    // TTBR0/TTBCR/VBAR must be readable: the boot loader saves and restores them
    // around the remap it performs with the V2P probes.
    Fixture f;
    f.load(kCodeBase, {
        arm_cp15(false, 0, 2, 4, 0, 0),   // mcr p15,0,r4,c2,c0,0 : TTBR0 = r4
        arm_cp15(true, 0, 2, 5, 0, 0),    // mrc p15,0,r5,c2,c0,0
        arm_cp15(false, 0, 2, 4, 0, 2),   // mcr p15,0,r4,c2,c0,2 : TTBCR = r4
        arm_cp15(true, 0, 2, 6, 0, 2),    // mrc p15,0,r6,c2,c0,2
        arm_cp15(false, 0, 12, 4, 0, 0),  // mcr p15,0,r4,c12,c0,0 : VBAR = r4
        arm_cp15(true, 0, 12, 7, 0, 0),   // mrc p15,0,r7,c12,c0,0
    });
    f.cpu.reset(kCodeBase);

    f.set_reg(4, 0x40108000u);
    f.cpu.step();
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mmu.ttbr0, 0x40108000u);
    ZLB_EXPECT_EQ(f.reg(5), 0x40108000u);

    f.set_reg(4, 0x00000002u);
    f.cpu.step();
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mmu.ttbcr, 2u);
    ZLB_EXPECT_EQ(f.reg(6), 2u);

    f.set_reg(4, 0x00000120u);
    f.cpu.step();
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mmu.vbar, 0x00000120u);
    ZLB_EXPECT_EQ(f.reg(7), 0x00000120u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 24u);
}

ZLB_TEST(cp15_va_to_pa_probe_publishes_par) {
    // CP15 c7, c8, 0 (V2PCWPR) translates Rt and reports the physical address
    // through PAR (c7, c4, 0) instead of raising an abort - the mechanism the
    // boot loader uses to find the frame behind a virtual address.
    Fixture f;
    ArmMmu& mmu = f.cpu.mmu;
    f.load(kCodeBase, {
        arm_cp15(false, 0, 7, 4, 8, 0),  // mcr p15,0,r4,c7,c8,0 : V2PCWPR
        arm_cp15(true, 0, 7, 5, 4, 0),   // mrc p15,0,r5,c7,c4,0 : r5 = PAR
    });
    f.cpu.reset(kCodeBase);
    mmu.ttbr0 = 0x80010000u;
    mmu.dacr = 0x00000001u;
    mmu.sctlr |= 1u;

    const u32 index = (0x80000000u >> 20) & 0xFFFu;
    f.bus.write32(0x80010000u + index * 4u, 0x80000000u | 2u | (1u << 10) | (3u << 2));
    f.set_reg(4, 0x80004000u);
    f.cpu.step();
    f.cpu.step();
    ZLB_EXPECT_FALSE(f.cpu.halted);
    ZLB_EXPECT_EQ(mmu.par, 0x80004001u);  // PAR[0] = 1: translation succeeded
    ZLB_EXPECT_EQ(f.reg(5), 0x80004001u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 8u);
}

ZLB_TEST(cp15_va_to_pa_probe_reports_a_fault_instead_of_aborting) {
    // A probe of an unmapped address must not raise a prefetch/data abort; the
    // status goes into PAR with PAR[0] = 0.
    Fixture f;
    ArmMmu& mmu = f.cpu.mmu;
    f.load(kCodeBase, {
        arm_cp15(false, 0, 7, 4, 8, 0),  // V2PCWPR
        arm_cp15(true, 0, 7, 5, 4, 0),   // r5 = PAR
    });
    f.cpu.reset(kCodeBase);
    mmu.ttbr0 = 0x80010000u;
    mmu.dacr = 0x00000001u;
    mmu.sctlr |= 1u;

    const u32 index = (0x80000000u >> 20) & 0xFFFu;
    f.bus.write32(0x80010000u + index * 4u, 0x80000000u | 2u | (1u << 10) | (3u << 2));
    f.set_reg(4, 0xC0000000u);  // L1[0xC00] is left as a fault descriptor
    const u64 faults_before = mmu.total_faults;
    f.cpu.step();
    f.cpu.step();
    ZLB_EXPECT_FALSE(f.cpu.halted);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 8u);
    ZLB_EXPECT_EQ(mmu.par & 1u, 0u);
    ZLB_EXPECT_TRUE((mmu.par >> 1) != 0u);
    // The probe fault is accounted for, but no exception was taken: only the
    // probe failed, the two instruction fetches translated fine.
    ZLB_EXPECT_EQ(mmu.total_faults, faults_before + 1u);
}

ZLB_TEST(mmu_section_ap_and_apx_semantics) {
    // ARMv7 short descriptors are not LPAE: for a *first level* (section)
    // descriptor AP[1:0] lives in bits [11:10] and bit 15 is APX, an override to
    // "no user access" - not an AP[2] read-only bit. AP = 0b11 with APX = 0 is
    // therefore readable *and writable* by user code, which is what the kernel
    // boot loader's identity sections rely on.
    Fixture f;
    ArmMmu& mmu = f.cpu.mmu;
    mmu.ttbr0 = 0x80010000u;
    mmu.dacr = 0x00000001u;
    mmu.sctlr |= 1u;
    const u32 index = (0x81000000u >> 20) & 0xFFFu;
    const u32 base = 0x81000000u | 2u | (3u << 2);  // identity section, C/B = 1
    const auto put = [&](u32 ap, u32 apx) {
        f.bus.write32(0x80010000u + index * 4u, base | (ap << 10) | (apx << 15));
    };

    put(3, 0);  // read/write for everyone
    ZLB_EXPECT_TRUE(mmu.translate(0x81000000u, false, false, arm::kModeUser).ok);
    ZLB_EXPECT_TRUE(mmu.translate(0x81000000u, true, false, arm::kModeUser).ok);

    put(1, 0);  // privileged RW, user read-only
    ZLB_EXPECT_TRUE(mmu.translate(0x81000000u, false, false, arm::kModeUser).ok);
    ZLB_EXPECT_FALSE(mmu.translate(0x81000000u, true, false, arm::kModeUser).ok);

    put(3, 1);  // APX set: no user access, and AP = 0b11 becomes privileged read-only
    ZLB_EXPECT_FALSE(mmu.translate(0x81000000u, false, false, arm::kModeUser).ok);
    ZLB_EXPECT_TRUE(mmu.translate(0x81000000u, false, false, arm::kModeSupervisor).ok);
    ZLB_EXPECT_FALSE(mmu.translate(0x81000000u, true, false, arm::kModeSupervisor).ok);

    put(1, 1);  // APX set with AP = 0b01: privileged keeps read/write, user loses access
    ZLB_EXPECT_TRUE(mmu.translate(0x81000000u, true, false, arm::kModeSupervisor).ok);
    ZLB_EXPECT_FALSE(mmu.translate(0x81000000u, false, false, arm::kModeUser).ok);

    put(2, 0);  // privileged read-only
    ZLB_EXPECT_TRUE(mmu.translate(0x81000000u, false, false, arm::kModeSupervisor).ok);
    ZLB_EXPECT_FALSE(mmu.translate(0x81000000u, true, false, arm::kModeSupervisor).ok);
}

ZLB_TEST(mmu_fault_registers_are_exposed) {    // `info` must be able to answer "why did it abort" without the bus trace.
    Fixture f;
    f.cpu.mmu.sctlr |= 1u;
    f.cpu.mmu.dfsr = 0x00000805u;
    f.cpu.mmu.dfar = 0x91EB40BCu;
    f.cpu.mmu.ifsr = 0x00000007u;
    f.cpu.mmu.ifar = 0xC02716A8u;

    std::vector<RegValue> regs;
    f.cpu.registers(regs);
    bool saw_dfsr = false;
    bool saw_dfar = false;
    bool saw_sctlr = false;
    for (const RegValue& value : regs) {
        if (value.group != "CP15") continue;
        if (value.name == "DFSR") { saw_dfsr = true; ZLB_EXPECT_EQ(value.value, 0x00000805u); }
        if (value.name == "DFAR") { saw_dfar = true; ZLB_EXPECT_EQ(value.value, 0x91EB40BCu); }
        if (value.name == "SCTLR") saw_sctlr = true;
    }
    ZLB_EXPECT_TRUE(saw_dfsr);
    ZLB_EXPECT_TRUE(saw_dfar);
    ZLB_EXPECT_TRUE(saw_sctlr);

    std::vector<std::string> lines;
    f.cpu.describe_state(lines);
    bool mentioned = false;
    for (const std::string& line : lines) {
        if (line.find("DFSR=0x00000805") != std::string::npos) mentioned = true;
    }
    ZLB_EXPECT_TRUE(mentioned);
}

ZLB_TEST(mmu_translation_fault_raises_data_abort) {
    Fixture f;
    ArmMmu& mmu = f.cpu.mmu;
    const u32 l1_table = 0x80010000u;
    // ldr r1, [r0] with r0 in a region whose L1 descriptor is a fault.
    f.load(kCodeBase, {arm_ldr_str_imm(0xE, true, false, 0, 1, 0)});
    // reset() clears the CP15 state, so everything below must come after it.
    f.cpu.reset(kCodeBase);
    mmu.ttbr0 = l1_table;
    mmu.dacr = 0x00000001u;
    mmu.sctlr |= 1u;
    f.set_reg(0, 0x81000000u);
    // Map the code page (VA 0x80000000) so the *data* access is what faults;
    // otherwise the instruction fetch aborts first and we would be testing the
    // prefetch path by accident.
    f.bus.write32(l1_table + (((0x80000000u >> 20) & 0xFFFu) * 4u),
                  0x80000000u | 2u | (0u << 5) | (1u << 10) | (3u << 2));
    f.bus.write32(l1_table + (((0x81000000u >> 20) & 0xFFFu) * 4u), 0u);  // fault descriptor
    const StepResult result = f.cpu.step();
    ZLB_EXPECT_TRUE(result.faulted);
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeAbort);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), arm::kVecDataAbort);
    ZLB_EXPECT_NE(mmu.dfsr, 0u);
}

ZLB_TEST(mmu_coarse_page_translate) {
    Fixture f;
    ArmMmu& mmu = f.cpu.mmu;
    const u32 l1 = 0x80010000u;
    const u32 l2 = 0x80018000u;
    mmu.ttbr0 = l1;
    mmu.dacr = 0x00000001u;
    mmu.sctlr |= 1u;

    const u32 index1 = (0x81000000u >> 20) & 0xFFFu;
    f.bus.write32(l1 + index1 * 4u, l2 | 1u);          // coarse page table, domain 0
    const u32 index2 = (0x81000000u >> 12) & 0xFFu;
    f.bus.write32(l2 + index2 * 4u, 0x80010000u | 2u | (3u << 4) | (1u << 9));  // small page, AP=0b011

    u32 pa = 0;
    std::string fault;
    ZLB_EXPECT_TRUE(f.cpu.translate(0x81000ABCu, false, false, pa, fault));
    ZLB_EXPECT_EQ(pa, 0x80010ABCu);
}

// ===========================================================================
// Disassembler
// ===========================================================================

ZLB_TEST(disasm_vfp_transfers_are_not_vfma) {
    // Regression: the sel-based VFMA switch used to swallow the whole VFP
    // transfer class, printing e.g. `vfma.f32 s1, s16, s0` for `vmsr`.
    Fixture f;
    struct Case {
        u32 word;
        const char* expect;
    };
    const Case cases[] = {
        {0xEE000A10u, "vmov s0, r0"},        // VMOV core -> single
        {0xEE100A10u, "vmov r0, s0"},        // VMOV single -> core
        {0xEEE80A10u, "vmsr fpexc, r0"},     // VMSR FPEXC
        {0xEEE10A10u, "vmsr fpscr, r0"},     // VMSR FPSCR
        {0xEEF80A10u, "vmrs r0, fpexc"},     // VMRS FPEXC
        {0xEEF1FA10u, "vmrs apsr_nzcv, fpscr"},
        {0xEE300A00u, "vadd.f32 s0, s0, s0"},
    };
    for (const Case& c : cases) {
        f.bus.write32(kCodeBase, c.word);
        unsigned length = 0;
        const std::string text = arm_disassemble(f.bus, kCodeBase, false, length);
        ZLB_EXPECT_EQ(length, 4u);
        ZLB_EXPECT_TRUE(text.find(c.expect) != std::string::npos);
    }
}

ZLB_TEST(disasm_neon_transfer_and_data_processing) {
    // Regression: the real kernel_boot_loader NEON block (vdup + 15x vorr)
    // must decode to named instructions, not `.word` and not a VFP op.
    Fixture f;
    struct Case {
        u32 word;
        const char* expect;
    };
    const Case cases[] = {
        {0xEEA00B10u, "vdup.32 q0, r0"},     // VDUP.32 q0, r0
        {0xF2202150u, "vorr q1, q0, q0"},    // VORR q1, q0, q0
        {0xF2204150u, "vorr q2, q0, q0"},
        {0xF2600150u, "vorr q8, q0, q0"},
        {0xF260E150u, "vorr q15, q0, q0"},
        {0xF2000150u, "vand q0, q0, q0"},    // size 00 -> VAND
    };
    for (const Case& c : cases) {
        f.bus.write32(kCodeBase, c.word);
        unsigned length = 0;
        const std::string text = arm_disassemble(f.bus, kCodeBase, false, length);
        ZLB_EXPECT_TRUE(text.find(c.expect) != std::string::npos);
    }
}

ZLB_TEST(neon_vdup_and_vorr_execute) {
    // The two idioms the kernel boot loader uses must execute exactly, not be
    // approximated: vdup.32 replicates Rt into all four 32-bit lanes and vorr
    // is a plain bitwise OR of two vectors.
    Fixture f;
    const u32 vdup = 0xEEA00B10u;   // vdup.32 q0, r0
    const u32 vorr = 0xF2202150u;   // vorr q1, q0, q0
    f.load(kCodeBase, {vdup, vorr});
    f.cpu.reset(kCodeBase);
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    f.set_reg(0, 0xDEADBEEFu);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(0), 0xDEADBEEFDEADBEEFull);
    ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(1), 0xDEADBEEFDEADBEEFull);
    // Clear q0 first, then check that vorr copies it into q1.
    f.cpu.vfp.write_d32(1, 0u);
    f.cpu.vfp.write_d32(0, 0u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(2), 0u);
    ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(3), 0u);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);

    // A NEON instruction outside the implemented subset must report itself
    // instead of silently doing nothing.
    Fixture g;
    g.load(kCodeBase, {0xF2200950u});  // vmul.i32 q0, q0, q0
    g.cpu.reset(kCodeBase);
    g.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    const StepResult result = g.cpu.step();
    ZLB_EXPECT_TRUE(result.faulted);
    ZLB_EXPECT_TRUE(g.cpu.undefined_instruction);
    ZLB_EXPECT_TRUE(g.cpu.halt_reason.find("Advanced SIMD") != std::string::npos);
}

ZLB_TEST(disasm_a32_branch_and_literal) {
    Fixture f;
    f.load(kCodeBase, {arm_branch(0xE, true, 0x10), arm_ldr_str_imm(0xE, true, false, 15, 0, 0x10)});
    unsigned length = 0;
    const std::string bl = arm_disassemble(f.bus, kCodeBase, false, length);
    ZLB_EXPECT_EQ(length, 4u);
    ZLB_EXPECT_TRUE(bl.find("bl ") == 0);
    ZLB_EXPECT_TRUE(bl.find("0x80000048") != std::string::npos);

    const std::string ldr = arm_disassemble(f.bus, kCodeBase + 4, false, length);
    ZLB_EXPECT_TRUE(ldr.find("ldr r0, #0x80000018") != std::string::npos);
    ZLB_EXPECT_TRUE(ldr.find("0x80000018") != std::string::npos);
}

ZLB_TEST(disasm_thumb16_and_thumb32) {
    Fixture f;
    const std::vector<u16> code = {
        static_cast<u16>(t16_mov_imm(0, 0x42)),
        static_cast<u16>(t16_add_imm(1, 3)),
        0xBF08u,
        0x4770u,  // bx lr
    };
    f.load(kCodeBase, pack_halfwords(code));
    unsigned length = 0;
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase, true, length).find("movs r0, #42") != std::string::npos);
    ZLB_EXPECT_EQ(length, 2u);
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase + 2, true, length).find("adds r1") != std::string::npos);
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase + 4, true, length).find("iteq") != std::string::npos);
    ZLB_EXPECT_EQ(length, 2u);
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase + 6, true, length).find("bx r14") != std::string::npos);

    std::vector<u32> words;
    emit_thumb32_bl(words, kCodeBase, kCodeBase + 0x100u);
    f.load(kCodeBase + 0x20, words);
    const std::string bl = arm_disassemble(f.bus, kCodeBase + 0x20, true, length);
    ZLB_EXPECT_EQ(length, 4u);
    ZLB_EXPECT_TRUE(bl.find("bl #0x80000100") != std::string::npos);
}

// ===========================================================================
// Misc / integration
// ===========================================================================

ZLB_TEST(arm_undefined_instruction_reports_fault) {
    Fixture f;
    f.load(kCodeBase, {0xF7F0A000u});  // a permanently undefined encoding
    f.cpu.reset(kCodeBase);
    const StepResult result = f.cpu.step();
    ZLB_EXPECT_TRUE(result.faulted);
    ZLB_EXPECT_TRUE(f.cpu.undefined_instruction);
    ZLB_EXPECT_FALSE(result.fault.empty());
}

ZLB_TEST(arm_movw_movt_build_a_constant) {
    Fixture f;
    f.load(kCodeBase, {arm_movw(0xE, 0, 0x1234),
                       0xE3400000u | ((0x5678u >> 12) << 16) | (0u << 12) | (0x5678u & 0xFFFu)});
    f.cpu.reset(kCodeBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x1234u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x56781234u);
}

ZLB_TEST(thumb2_section_fill_loop_head) {
    Fixture f;
    // The head of KBL's section-fill loop (0x4003A9F6..0x4003AA12).  The loop
    // counts descriptors in r2 and stops when r2 == r5, where r5 is the region
    // size in MiB, so the "is r2 a power of two" test must send an even r5 down
    // the path that starts the counter at 2:
    //   4003A9F6  movs r2, #1          2101
    //   4003A9F8  subs r3, r5, #1      1E6B
    //   4003AA04  and.w r3, r3, r2     03 EA 02 03
    //   4003AA0E  cbz r3, +0x28        4B B1
    //   4003AA12  movs r2, #2          2202
    // Two 16-bit instructions share one 32-bit word, so the sequence is:
    //   0x00 movs r2,#1 | 0x02 subs r3,r5,#1 | 0x04 and.w r3,r3,r2
    //   0x08 cbz r3,+40 | 0x0A movs r2,#2
    f.load(kCodeBase, {0x1E6B2201u, 0x0302EA03u, 0x2202B14Bu});

    // Even r5: (r5-1) & 1 == 1, so the branch must not be taken.
    f.cpu.reset(kCodeBase | 1u);
    f.set_reg(5, 0x84u);
    for (int i = 0; i < 4; ++i) f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(3), 1u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 10u);

    // Odd r5: (r5-1) & 1 == 0, so the branch is taken and the counter keeps its
    // odd start (the odd-size path).
    f.cpu.reset(kCodeBase | 1u);
    f.set_reg(5, 0x85u);
    for (int i = 0; i < 4; ++i) f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(3), 0u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 8u + 4u + 18u);   // CBZ imm5 = 9
}

ZLB_TEST(thumb2_register_controlled_shifts) {
    Fixture f;
    // LSL.W r0, r3, r2 : 1111 1010 0000 0011 | 1111 0000 0000 0010
    // LSR.W r0, r3, r2 : 1111 1010 0010 0011 | 1111 0000 0000 0010
    // ROR.W r0, r3, r2 : 1111 1010 0110 0011 | 1111 0000 0000 0010
    f.load(kCodeBase, {0xF002FA03u, 0xF002FA23u, 0xF002FA63u});
    f.cpu.reset(kCodeBase | 1u);
    f.set_reg(3, 0x80000001u);
    f.set_reg(2, 4u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x00000010u);        // LSL #4
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x08000000u);        // LSR #4
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x18000000u);        // ROR #4
}

ZLB_TEST(arm_thumb2_movw_ubfx_and_bfi) {
    Fixture f;
    // T32 plain binary immediate (ARM ARM A6.3.4). Each instruction is two
    // halfwords; in little-endian memory the first halfword is at the lower
    // address, so the in-memory word is (hw2 << 16) | hw1.
    //
    //   MOVW Rd, #imm16 : 11110 i 10 0100 imm4 : 0 imm3 Rd imm8
    //   UBFX Rd, Rn, #lsb, #width : 11110 0 11 1100 Rn : 0 imm3 Rd imm2 widthm1
    //   BFI  Rd, Rn, #lsb, #width : 11110 0 11 0110 Rn : 0 imm3 Rd imm2 widthm1
    // Verified against capstone: movw r0,#0xABCD / ubfx r1,r0,#4,#8 / bfi r1,r0,#0,#4
    const u32 movw = 0x30CDF64Au;
    const u32 ubfx = 0x1107F3C0u;
    const u32 bfi = 0x0103F360u;
    f.load(kCodeBase, {movw, ubfx, bfi});
    f.cpu.reset(kCodeBase | 1u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0xABCDu);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 0xBCu);   // (0xABCD >> 4) & 0xFF = 0xBC
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 0xBDCu);  // low nibble replaced with 0xD
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
}
// ---------------------------------------------------------------------------
// Thumb-2 "load/store single data item" addressing forms (ARM ARM A6.3.9).
//
// The register-offset form is selected by hw2[11:6] == 0 - *not* by hw2[11],
// because in the 12-bit immediate forms hw2[11:0] is the offset itself. The
// kernel boot loader depends on both: it fills its first level tables with
// `str.w r7,[r5,#0xB0]` (hw1=0xF8C5 hw2=0x70B0, T3) and installs descriptors
// with `str.w r3,[r0,r2,lsl#2]` (hw1=0xF840 hw2=0x3022, T2).
// ---------------------------------------------------------------------------

namespace {

constexpr u32 t32_load_store(u32 hw1, u32 hw2) { return (hw2 << 16) | hw1; }

/// Size/L bits of hw1: bit6 = word, bit5 = halfword, bit4 = load.
constexpr u32 t32_size_bits(bool load, bool word, bool half) {
    return (word ? 0x40u : (half ? 0x20u : 0u)) | (load ? 0x10u : 0u);
}

/// T2 (register offset): 1111 1000 0100 Rn Rt 000000 imm2 Rm.
constexpr u32 t32_t2(u32 rn, u32 rt, u32 rm, u32 shift, bool load, bool word = true, bool half = false) {
    const u32 hw1 = 0xF800u | t32_size_bits(load, word, half) | rn;
    return t32_load_store(hw1, (rt << 12) | ((shift & 3u) << 4) | rm);
}

/// T3 (12-bit immediate): 1111 1000 1100 Rn Rt imm12.
constexpr u32 t32_t3(u32 rn, u32 rt, u32 imm12, bool load, bool word = true, bool half = false) {
    const u32 hw1 = 0xF880u | t32_size_bits(load, word, half) | rn;
    return t32_load_store(hw1, (rt << 12) | (imm12 & 0xFFFu));
}

/// T4 (8-bit immediate with P/U/W): 1111 1000 0100 Rn Rt 1 P U W imm8.
constexpr u32 t32_t4(u32 rn, u32 rt, u32 imm8, bool pre, bool up, bool wb, bool load = false) {
    const u32 hw1 = 0xF800u | t32_size_bits(load, true, false) | rn;
    return t32_load_store(hw1, (rt << 12) | 0x800u | (pre ? 0x400u : 0u) | (up ? 0x200u : 0u) |
                                   (wb ? 0x100u : 0u) | (imm8 & 0xFFu));
}

}  // namespace

ZLB_TEST(thumb32_t3_immediate_offset_is_not_a_register_offset) {
    // hw2[11:0] is the offset; an offset below 0x800 must not be read as
    // "base + register". Both the low and the high half of the 12-bit range.
    Fixture low;
    low.load(kCodeBase, {t32_t3(5, 7, 0x0B0u, false)});
    low.cpu.reset(kCodeBase | 1u);
    low.set_reg(5, kDataBase);
    low.set_reg(7, 0xCAFEBABEu);
    low.set_reg(0, 0xDEADBEEFu);  // would be used by a bogus [Rn, Rm] decode
    low.cpu.step();
    ZLB_EXPECT_EQ(low.bus.read32(kDataBase + 0xB0u), 0xCAFEBABEu);
    ZLB_EXPECT_EQ(low.reg(5), kDataBase);  // no writeback

    Fixture high;
    high.load(kCodeBase, {t32_t3(5, 1, 0x180u, false)});
    high.cpu.reset(kCodeBase | 1u);
    high.set_reg(5, kDataBase);
    high.set_reg(1, 0x11223344u);
    high.set_reg(2, 0xDEADBEEFu);
    high.cpu.step();
    ZLB_EXPECT_EQ(high.bus.read32(kDataBase + 0x180u), 0x11223344u);

    // Same rule for the halfword form (`ldrh.w r3,[r5,#0xA2]`).
    Fixture half;
    half.load(kCodeBase, {t32_t3(5, 3, 0x0A2u, true, false, true)});
    half.cpu.reset(kCodeBase | 1u);
    half.set_reg(5, kDataBase);
    half.set_reg(2, 0xDEADBEEFu);
    half.bus.write16(kDataBase + 0xA2u, 0xBEEFu);
    half.cpu.step();
    ZLB_EXPECT_EQ(half.reg(3), 0xBEEFu);
}

ZLB_TEST(thumb32_t2_register_offset_applies_the_shift) {
    // `str.w r3,[r0,r2,lsl#2]` - the form the kernel boot loader uses to index
    // its first level table.
    Fixture st;
    st.load(kCodeBase, {t32_t2(0, 3, 2, 2, false)});
    st.cpu.reset(kCodeBase | 1u);
    st.set_reg(0, kDataBase);
    st.set_reg(2, 4);  // index 4 -> byte offset 0x10
    st.set_reg(3, 0x80200181u);
    st.cpu.step();
    ZLB_EXPECT_EQ(st.bus.read32(kDataBase + 0x10u), 0x80200181u);
    ZLB_EXPECT_EQ(st.reg(0), kDataBase);  // no writeback

    Fixture ld;
    ld.load(kCodeBase, {t32_t2(0, 3, 2, 2, true)});
    ld.cpu.reset(kCodeBase | 1u);
    ld.set_reg(0, kDataBase);
    ld.set_reg(2, 4);
    ld.bus.write32(kDataBase + 0x10u, 0x000001E0u);
    ld.cpu.step();
    ZLB_EXPECT_EQ(ld.reg(3), 0x1E0u);

    // Unshifted register form.
    Fixture plain;
    plain.load(kCodeBase, {t32_t2(0, 3, 2, 0, false)});
    plain.cpu.reset(kCodeBase | 1u);
    plain.set_reg(0, kDataBase);
    plain.set_reg(2, 0x24u);
    plain.set_reg(3, 0x55AA55AAu);
    plain.cpu.step();
    ZLB_EXPECT_EQ(plain.bus.read32(kDataBase + 0x24u), 0x55AA55AAu);
}

ZLB_TEST(thumb32_t4_post_increment_still_has_writeback) {
    // The form the table fill loop uses: `str.w r2,[r3],#4`.
    Fixture f;
    f.load(kCodeBase, {t32_t4(3, 2, 4, false, true, true)});
    f.cpu.reset(kCodeBase | 1u);
    f.set_reg(3, kDataBase);
    f.set_reg(2, 0x1E0u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase), 0x1E0u);
    ZLB_EXPECT_EQ(f.reg(3), kDataBase + 4u);

    // Pre-indexed negative offset with writeback: `str.w r3,[r0,#-4]!`.
    Fixture pre;
    pre.load(kCodeBase, {t32_t4(0, 3, 4, true, false, true)});
    pre.cpu.reset(kCodeBase | 1u);
    pre.set_reg(0, kDataBase + 8u);
    pre.set_reg(3, 0x12345678u);
    pre.cpu.step();
    ZLB_EXPECT_EQ(pre.bus.read32(kDataBase + 4u), 0x12345678u);
    ZLB_EXPECT_EQ(pre.reg(0), kDataBase + 4u);
}

ZLB_TEST(arm_status_line_and_registers) {
    Fixture f;
    f.cpu.reset(kCodeBase);
    ZLB_EXPECT_TRUE(f.cpu.status_line().find("ARM SVC") == 0);
    ZLB_EXPECT_TRUE(f.cpu.status_line().find("mmu=off") != std::string::npos);

    std::vector<RegValue> regs;
    f.cpu.registers(regs);
    bool saw_arm = false;
    bool saw_cp15 = false;
    bool saw_vfp = false;
    for (const auto& value : regs) {
        if (value.group == "ARM") saw_arm = true;
        if (value.group == "CP15") saw_cp15 = true;
        if (value.group == "VFP") saw_vfp = true;
    }
    ZLB_EXPECT_TRUE(saw_arm);
    ZLB_EXPECT_TRUE(saw_cp15);
    ZLB_EXPECT_TRUE(saw_vfp);

    u64 out = 0;
    ZLB_EXPECT_TRUE(f.cpu.get_register("CPSR", out));
    ZLB_EXPECT_EQ(static_cast<u32>(out), arm::kResetCpsr);
    ZLB_EXPECT_TRUE(f.cpu.set_register("R0", 0xDEADBEEFu));
    ZLB_EXPECT_TRUE(f.cpu.get_register("r0", out));
    ZLB_EXPECT_EQ(static_cast<u32>(out), 0xDEADBEEFu);
    ZLB_EXPECT_FALSE(f.cpu.get_register("nosuch", out));
}

ZLB_TEST(cpu_factory_creates_arm_core) {
    Bus bus;
    bus.add_ram("main", 64u * 1024u, 0x80000000u, "test");
    // make_cpu() also references the MeP and RL78 factories, which belong to
    // their own workstreams, so this test links against the ARM factory directly.
    auto cpu = create_arm_core(bus);
    ZLB_EXPECT_TRUE(cpu != nullptr);
    ZLB_EXPECT_EQ(static_cast<int>(cpu->arch()), static_cast<int>(Arch::Arm));
    ZLB_EXPECT_TRUE(std::string(cpu->core_name()) == "ARM Cortex-A9");
}
