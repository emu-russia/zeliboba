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

// ARM ARM Table A7-9: AND, BIC, ORR, ORN, EOR, BSL, BIT, BIF.
constexpr u32 arm_neon_logical(u32 op, bool quad, u32 d, u32 n, u32 m) {
    return 0xF2000110u | ((op >> 2) << 24) | ((op & 3u) << 20) |
           ((d >> 4) << 22) | ((d & 15u) << 12) | ((n >> 4) << 7) |
           ((n & 15u) << 16) | ((m >> 4) << 5) | (m & 15u) | (quad ? 0x40u : 0u);
}

constexpr u32 arm_neon_mov_imm(u32 d, bool quad, u32 cmode, bool op, u32 imm8) {
    return 0xF2800010u | ((d >> 4) << 22) | ((d & 15u) << 12) |
           ((imm8 >> 7) << 24) | (((imm8 >> 4) & 7u) << 16) | (imm8 & 15u) |
           (cmode << 8) | (quad ? 0x40u : 0u) | (op ? 0x20u : 0u);
}

constexpr u32 arm_neon_vst1(u32 d, u32 type, u32 size, u32 align, u32 n, u32 m) {
    return 0xF4000000u | ((d >> 4) << 22) | ((d & 15u) << 12) | (n << 16) |
           (type << 8) | (size << 6) | (align << 4) | m;
}

constexpr u32 arm_neon_vld1(u32 d, u32 type, u32 size, u32 align, u32 n, u32 m) {
    return arm_neon_vst1(d, type, size, align, n, m) | 0x00200000u;
}

constexpr u32 arm_neon_movl(u32 width, bool uns, u32 d, u32 m) {
    return 0xF2800A10u | ((width / 8u) << 19) | (uns ? 0x01000000u : 0u) |
           ((d >> 4) << 22) | ((d & 15u) << 12) | ((m >> 4) << 5) | (m & 15u);
}

constexpr u32 arm_neon_add(u32 size, bool quad, u32 d, u32 n, u32 m) {
    return 0xF2000800u | (size << 20) | ((d >> 4) << 22) | ((d & 15u) << 12) |
           ((n >> 4) << 7) | ((n & 15u) << 16) | ((m >> 4) << 5) | (m & 15u) |
           (quad ? 0x40u : 0u);
}

constexpr u32 arm_neon_padd(u32 size, u32 d, u32 n, u32 m) {
    return (arm_neon_add(size,false,d,n,m)&~0x00000F00u)|0x00000B10u;
}

constexpr u32 arm_vmov_scalar32(u32 rt, u32 d, bool upper, bool to_core) {
    return 0xEE000B10u | ((d & 15u) << 16) | ((d >> 4) << 7) | (rt << 12) |
           (upper ? 0x00200000u : 0u) | (to_core ? 0x00100000u : 0u);
}

void load_neon(Fixture& f, u32 instr, bool thumb) {
    if (thumb) {
        const u32 t32 = (instr & 0x0F000000u) == 0x04000000u ?
            0xF9000000u | (instr & 0x00FFFFFFu) :
            0xEF000000u | (instr & 0x00FFFFFFu) | ((instr & 0x01000000u) << 4);
        f.load_halfwords(kCodeBase, {static_cast<u16>(t32 >> 16), static_cast<u16>(t32)});
    } else f.load(kCodeBase, {instr});
    f.cpu.reset(kCodeBase | (thumb ? 1u : 0u));
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
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
/// The packed immediate is the branch offset itself: S = offset[24], I1 =
/// offset[23], I2 = offset[22], imm10 = offset[21:12] and imm10L = offset[11:2],
/// with offset[1:0] == 0 because the BLX target is word aligned (ARM ARM
/// A7.7.18; the BL form instead carries offset[24:1] because bit 0 of its
/// target is the Thumb bit). J1/J2 store the inverted-interworking form:
/// J1 == NOT(I1 EOR S), J2 == NOT(I2 EOR S).
void emit_thumb32_blx(std::vector<u32>& out, u32 address, u32 target) {
    const u32 base = (address + 4u) & ~3u;
    const s32 offset = static_cast<s32>(target) - static_cast<s32>(base);
    const u32 imm = static_cast<u32>(offset) & 0x01FFFFFFu;
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

ZLB_TEST(thumb32_blx_immediate_carries_the_whole_offset) {
    // The BLX immediate form packs offset[24:2] into S:I1:I2:imm10:imm10L, so a
    // target beyond imm10L's own 12-bit reach exercises imm10, I1 and the sign
    // bit. emitters that shifted the offset before splitting it produced a
    // target 2x or 4x off (or dropped the high bits entirely).
    const u32 targets[] = {
        kCodeBase + 0x100u,        // offset 0x0000FC: imm10L only
        kCodeBase + 0x1000u,       // offset 0x000FFC: imm10 = 1
        kCodeBase + 0x200000u,     // offset 0x1FFFFC: I1 set
        kCodeBase + 4u - 0x200u,   // offset -0x200: S = 1
    };
    for (u32 target : targets) {
        Fixture f;
        std::vector<u32> words;
        emit_thumb32_blx(words, kCodeBase, target);
        f.load(kCodeBase, words);
        f.cpu.reset(kCodeBase | 1u);
        f.cpu.step();
        ZLB_EXPECT_EQ(f.cpu.get_pc(), target);
        ZLB_EXPECT_FALSE(f.cpu.thumb);
        ZLB_EXPECT_FALSE((f.cpu.cpsr & arm::kFlagT) != 0);
        ZLB_EXPECT_EQ(f.reg(14), kCodeBase + 4u + 1u);
        ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    }
}

ZLB_TEST(thumb_conditional_branch) {
    Fixture f;
    // movs r0, #0 ; beq +4 ; movs r1, #1 ; movs r2, #2
    //
    // T16 B<c> is `1101 cond imm8` with PC = PC + 4 + (imm8 << 1), so imm8 = 0
    // branches to the instruction *after* the next one (the old encoding 0xD001
    // asked for PC + 6 and could never land on base + 6, which is what the test
    // body expects).
    const std::vector<u16> code = {
        static_cast<u16>(t16_mov_imm(0, 0)),   // movs r0, #0
        0xD000u,                               // beq +4 (imm8 = 0)
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

    // A non-zero imm8 (1 -> PC + 6) has to skip two halfwords instead of one.
    Fixture g;
    const std::vector<u16> wider = {
        static_cast<u16>(t16_mov_imm(0, 0)),   // base + 0
        0xD001u,                               // beq +6
        static_cast<u16>(t16_mov_imm(1, 1)),   // skipped
        static_cast<u16>(t16_mov_imm(3, 3)),   // skipped
        static_cast<u16>(t16_mov_imm(2, 4)),   // base + 8
    };
    g.load(kCodeBase, pack_halfwords(wider));
    g.cpu.reset(kCodeBase | 1u);
    g.cpu.step();
    g.cpu.step();
    ZLB_EXPECT_EQ(g.cpu.get_pc(), kCodeBase + 8u);
    g.cpu.step();
    ZLB_EXPECT_EQ(g.reg(1), 0u);
    ZLB_EXPECT_EQ(g.reg(3), 0u);
    ZLB_EXPECT_EQ(g.reg(2), 4u);

    // The same branch with the condition false falls through.
    Fixture h;
    const std::vector<u16> not_taken = {
        static_cast<u16>(t16_mov_imm(0, 1)),   // movs r0, #1 -> Z clear
        0xD001u,                               // beq +6 (not taken)
        static_cast<u16>(t16_mov_imm(1, 1)),
        static_cast<u16>(t16_mov_imm(3, 3)),
        static_cast<u16>(t16_mov_imm(2, 4)),
    };
    h.load(kCodeBase, pack_halfwords(not_taken));
    h.cpu.reset(kCodeBase | 1u);
    h.cpu.step();
    h.cpu.step();
    ZLB_EXPECT_EQ(h.cpu.get_pc(), kCodeBase + 4u);
    h.cpu.step();
    ZLB_EXPECT_EQ(h.reg(1), 1u);
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

ZLB_TEST(thumb_tbb_table_branch_byte) {
    Fixture f;
    // tbb [pc, r0] at kCodeBase; the byte table follows at kCodeBase+4.
    f.load_halfwords(kCodeBase, { 0xE8DFu, 0xF000u });   // tbb [pc, r0]
    f.bus.write8(kCodeBase + 4u, 2);                     // entry[0] = 2
    f.bus.write8(kCodeBase + 5u, 3);                     // entry[1] = 3
    f.cpu.reset(kCodeBase | 1u);
    f.set_reg(0, 0);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 8u);       // +4 + 2*2
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);

    Fixture g;
    g.load_halfwords(kCodeBase, { 0xE8DFu, 0xF000u });
    g.bus.write8(kCodeBase + 4u, 2);
    g.bus.write8(kCodeBase + 5u, 3);
    g.cpu.reset(kCodeBase | 1u);
    g.set_reg(0, 1);
    g.cpu.step();
    ZLB_EXPECT_EQ(g.cpu.get_pc(), kCodeBase + 10u);      // +4 + 2*3
}

ZLB_TEST(thumb_tbh_table_branch_halfword) {
    Fixture f;
    // tbh [pc, r0, lsl #1] at kCodeBase; the halfword table follows.
    f.load_halfwords(kCodeBase, { 0xE8DFu, 0xF010u });   // tbh [pc, r0, lsl #1]
    f.bus.write16(kCodeBase + 4u, 2);                    // entry[0] = 2
    f.bus.write16(kCodeBase + 6u, 3);                    // entry[1] = 3
    f.cpu.reset(kCodeBase | 1u);
    f.set_reg(0, 0);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 8u);       // +4 + 2*2

    Fixture g;
    g.load_halfwords(kCodeBase, { 0xE8DFu, 0xF010u });
    g.bus.write16(kCodeBase + 4u, 2);
    g.bus.write16(kCodeBase + 6u, 3);
    g.cpu.reset(kCodeBase | 1u);
    g.set_reg(0, 1);
    g.cpu.step();
    ZLB_EXPECT_EQ(g.cpu.get_pc(), kCodeBase + 10u);      // +4 + 2*3
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
    // This checks the explicit non-flag-setting wide encoding. The narrow MOV
    // encoding also preserves flags inside IT, as the validator test below checks.
    const std::vector<u16> code = {
        static_cast<u16>(t16_mov_imm(0, 1)),
        static_cast<u16>(t16_cmp_imm(0, 1)),   // Z = 1
        0xBF0Cu,                               // ite eq -> mask 1100
        0xF04Fu, 0x0101u,                      // mov.w r1, #1
        0xF04Fu, 0x0202u,                      // mov.w r2, #2
    };
    f.load_halfwords(kCodeBase, code);
    f.cpu.reset(kCodeBase | 1u);
    for (int i = 0; i < 5; ++i) f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 1u);   // "then" executes
    ZLB_EXPECT_EQ(f.reg(2), 0u);   // "else" is skipped
}

ZLB_TEST(thumb_it_nskbl_sce_validator_returns_two) {
    // NSKBL 0x5101A4D2: BF0C / 2002 / 2000 selects 2 for a zero fourth
    // header byte. Narrow MOV does not set flags in IT, so the EQ half must
    // not clear Z and accidentally enable the NE half. The last slot also
    // preserves flags even though the core has advanced ITSTATE to zero.
    for (u32 fourth_byte : {0u, 1u}) {
        Fixture f;
        f.load_halfwords(kCodeBase, {
            0x2B00u, // cmp r3, #0
            0xBF0Cu, // ite eq
            0x2002u, // moveq r0, #2
            0x2000u, // movne r0, #0
            0xB900u, // cbnz r0, .+4 (the caller's nonzero-result continuation)
            0xBF00u,
            0xBF00u,
        });
        f.cpu.reset(kCodeBase | 1u);
        f.set_reg(0, 0xFFFFFFFFu);
        f.set_reg(3, fourth_byte);
        f.cpu.step();
        const u32 flags_after_cmp = f.cpu.cpsr & 0xF0000000u;
        for (int step = 0; step < 3; ++step) f.cpu.step();
        ZLB_EXPECT_EQ(f.reg(0), fourth_byte == 0u ? 2u : 0u);
        ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, flags_after_cmp);
        f.cpu.step();
        ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + (fourth_byte == 0u ? 12u : 10u));
        ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    }
}

ZLB_TEST(thumb_it_narrow_data_processing_preserves_flags) {
    struct Case { u16 instruction; u32 result; };
    // r0=0x80000000, r1=1, C=1. Check results as well as all four flags:
    // ADC/SBC must still consume C, and register shifts must suppress the
    // shifter's separate carry update along with their N/Z update.
    const Case cases[] = {
        {0x0008u, 1u},           // MOV register (LSL #0 alias)
        {0x0048u, 2u},           // LSL immediate
        {0x0848u, 0u},           // LSR immediate
        {0x1048u, 0u},           // ASR immediate
        {0x1840u, 0x80000001u},  // ADD register
        {0x1A40u, 0x7FFFFFFFu},  // SUB register
        {0x1C40u, 0x80000001u},  // ADD immediate (imm3)
        {0x1E40u, 0x7FFFFFFFu},  // SUB immediate (imm3)
        {0x2002u, 2u},           // MOV immediate
        {0x3001u, 0x80000001u},  // ADD immediate (imm8)
        {0x3801u, 0x7FFFFFFFu},  // SUB immediate (imm8)
        {0x4008u, 0u},           // AND
        {0x4048u, 0x80000001u},  // EOR
        {0x4088u, 0u},           // LSL register
        {0x40C8u, 0x40000000u},  // LSR register
        {0x4108u, 0xC0000000u},  // ASR register
        {0x4148u, 0x80000002u},  // ADC
        {0x4188u, 0x7FFFFFFFu},  // SBC
        {0x41C8u, 0x40000000u},  // ROR register
        {0x4248u, 0xFFFFFFFFu},  // RSB #0
        {0x4308u, 0x80000001u},  // ORR
        {0x4348u, 0x80000000u},  // MUL
        {0x4388u, 0x80000000u},  // BIC
        {0x43C8u, 0xFFFFFFFEu},  // MVN
    };
    constexpr u32 flags = arm::kFlagZ | arm::kFlagC | arm::kFlagV;
    for (const Case& test : cases) {
        for (u16 it : {0xBF08u, 0xBF04u}) { // single/final slot, first slot of ITT EQ
            Fixture f;
            f.load_halfwords(kCodeBase, {it, test.instruction, 0x2242u, 0x2301u});
            f.cpu.reset(kCodeBase | 1u);
            f.set_reg(0, 0x80000000u);
            f.set_reg(1, 1u);
            f.cpu.cpsr = (f.cpu.cpsr & ~0xF0000000u) | flags;
            f.cpu.step();
            f.cpu.step();
            ZLB_EXPECT_EQ(f.reg(0), test.result);
            ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, flags);
            f.cpu.step(); // mov r2,#0x42: inside ITT, outside single-slot IT
            ZLB_EXPECT_EQ(f.reg(2), 0x42u);
            ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u,
                          it == 0xBF04u ? flags : (arm::kFlagC | arm::kFlagV));
            f.cpu.step(); // MOV outside either block must set N/Z normally
            ZLB_EXPECT_EQ(f.reg(3), 1u);
            ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, arm::kFlagC | arm::kFlagV);
            ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
        }
    }
}

ZLB_TEST(thumb_it_comparisons_and_explicit_wide_s_still_set_flags) {
    struct Case { u16 instruction; u32 r0; u32 flags; };
    const Case cases[] = {
        {0x2801u, 2u, arm::kFlagC},                 // CMP immediate
        {0x4288u, 2u, arm::kFlagC},                 // CMP register
        {0x4588u, 2u, arm::kFlagC},                 // CMP high register (r8,r1)
        {0x42C8u, 0x7FFFFFFFu, arm::kFlagN | arm::kFlagV}, // CMN register
        {0x4208u, 2u, arm::kFlagZ | arm::kFlagC | arm::kFlagV}, // TST
    };
    for (const Case& test : cases) {
        Fixture f;
        f.load_halfwords(kCodeBase, {0xBF08u, test.instruction}); // IT EQ
        f.cpu.reset(kCodeBase | 1u);
        f.set_reg(0, test.r0);
        f.set_reg(1, 1u);
        f.set_reg(8, test.r0);
        f.cpu.cpsr |= 0xF0000000u;
        f.cpu.step();
        f.cpu.step();
        ZLB_EXPECT_EQ(f.reg(0), test.r0);
        ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, test.flags);
    }
    Fixture f;
    f.load_halfwords(kCodeBase, {0xBF08u, 0xF05Fu, 0x0002u}); // IT EQ; MOVS.W r0,#2
    f.cpu.reset(kCodeBase | 1u);
    f.cpu.cpsr |= 0xF0000000u;
    f.cpu.step();
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 2u);
    ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, arm::kFlagC | arm::kFlagV);
}

ZLB_TEST(thumb_lsl_immediate_carry_feeds_adc_and_is_preserved_in_it) {
    struct Case { u32 source; u32 amount; u32 result; bool carry; };
    const Case cases[] = {
        {0x80000000u, 1u, 0u, true},
        {1u, 1u, 2u, false},
        {0x08000001u, 5u, 0x20u, true},
        {0x80000010u, 5u, 0x200u, false},
        {3u, 31u, 0x80000000u, true},
        {0x40000000u, 31u, 0u, false},
        {0x80000001u, 0u, 0x80000001u, false}, // MOV alias preserves incoming C
    };
    for (const Case& test : cases) {
        for (bool incoming_carry : {false, true}) {
            for (bool in_it : {false, true}) {
                Fixture f;
                const u16 lsl = static_cast<u16>(t16_lsl_imm(0, 1, test.amount));
                if (in_it) f.load_halfwords(kCodeBase, {0xBF08u, lsl, 0x415Au});
                else f.load_halfwords(kCodeBase, {lsl, 0x415Au});
                f.cpu.reset(kCodeBase | 1u);
                f.set_reg(1, test.source);
                f.set_reg(2, 10u);
                f.set_reg(3, 0u); // following ADCS r2,r3 adds only the carry
                const u32 initial_flags = arm::kFlagZ | arm::kFlagV |
                                          (incoming_carry ? arm::kFlagC : 0u);
                f.cpu.cpsr = (f.cpu.cpsr & ~0xF0000000u) | initial_flags;
                if (in_it) f.cpu.step();
                f.cpu.step();
                const bool carry = (in_it || test.amount == 0u) ? incoming_carry : test.carry;
                const u32 shifted_flags = (test.result & arm::kFlagN) |
                                          (test.result == 0u ? arm::kFlagZ : 0u) |
                                          (carry ? arm::kFlagC : 0u) | arm::kFlagV;
                ZLB_EXPECT_EQ(f.reg(0), test.result);
                ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, in_it ? initial_flags : shifted_flags);
                f.cpu.step();
                ZLB_EXPECT_EQ(f.reg(2), carry ? 11u : 10u);
                ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, 0u);
                ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
            }
        }
    }
}

ZLB_TEST(thumb_modified_immediate_logical_carry_feeds_adc) {
    struct Immediate { u16 encoding; u32 value; bool rotated; bool carry; };
    // ThumbExpandImm_C (ARM ARM A6-231): zero extension and all three byte
    // replication forms preserve incoming C, regardless of the byte's bit 7.
    // The final two cases rotate 0x80 by 8/9 and must instead replace C.
    const Immediate immediates[] = {
        {0x0080u, 0x00000080u, false, false},
        {0x1080u, 0x00800080u, false, false},
        {0x2080u, 0x80008000u, false, false},
        {0x3080u, 0x80808080u, false, false},
        {0x007Fu, 0x0000007Fu, false, false},
        {0x107Fu, 0x007F007Fu, false, false},
        {0x207Fu, 0x7F007F00u, false, false},
        {0x307Fu, 0x7F7F7F7Fu, false, false},
        {0x4000u, 0x80000000u, true, true},
        {0x4080u, 0x40000000u, true, false},
    };
    struct Instruction { u16 hw1; u16 hw2; u32 source; bool writes_result; };
    const Instruction instructions[] = {
        {0xF05Fu, 0x0000u, 0u, true},           // MOVS.W r0,#imm
        {0xF010u, 0x0000u, 0xFFFFFFFFu, true},  // ANDS.W r0,r0,#imm
        {0xF010u, 0x0F00u, 0xFFFFFFFFu, false}, // TST.W r0,#imm
        {0xF090u, 0x0F00u, 0u, false},          // TEQ.W r0,#imm
    };
    for (const Immediate& imm : immediates) {
        for (const Instruction& instruction : instructions) {
            for (bool incoming_carry : {false, true}) {
                Fixture f;
                f.load_halfwords(kCodeBase, {
                    instruction.hw1,
                    static_cast<u16>(instruction.hw2 | imm.encoding),
                    0x415Au, // ADCS r2,r3 consumes the immediate's carry result
                });
                f.cpu.reset(kCodeBase | 1u);
                f.set_reg(0, instruction.source);
                f.set_reg(2, 10u);
                f.set_reg(3, 0u);
                f.cpu.cpsr = (f.cpu.cpsr & ~0xF0000000u) |
                             arm::kFlagN | arm::kFlagZ | arm::kFlagV |
                             (incoming_carry ? arm::kFlagC : 0u);
                f.cpu.step();
                const bool carry = imm.rotated ? imm.carry : incoming_carry;
                ZLB_EXPECT_EQ(f.reg(0), instruction.writes_result ? imm.value : instruction.source);
                ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u,
                              (imm.value & arm::kFlagN) | arm::kFlagV |
                              (carry ? arm::kFlagC : 0u));
                f.cpu.step();
                ZLB_EXPECT_EQ(f.reg(2), carry ? 11u : 10u);
                ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, 0u);
                ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
            }
        }
    }
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
    // SPSR_<mode> = CPSR on entry (ARM ARM B1.8.3): the IRQ bank now holds the
    // SVC CPSR we came from, and the untouched SVC/FIQ SPSRs stay 0.
    ZLB_EXPECT_EQ(f.cpu.banked_spsr(arm::kModeIrq) & arm::kModeMask, arm::kModeSupervisor);
    ZLB_EXPECT_EQ(f.cpu.banked_spsr(arm::kModeSupervisor), 0u);
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
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagI) != 0);  // FIQ entry masks IRQ
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
    //
    // The encoding is 0xEEF1FA10 (coproc = 0b1010).  The test used to OR in bit 8,
    // which changes the coprocessor field to 0b1011 (CP11) - a different
    // instruction that the core correctly rejects, so the flags were never moved.
    const u32 vmrs = 0xEEF1FA10u;  // vmrs apsr_nzcv, fpscr
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

    // And now the data processing instruction is allowed through.  VMOV moves
    // 2.5f (0x40200000) from r0 into s0 and the VADD doubles it; the test used to
    // poke s0 directly and then let the VMOV overwrite it with r0 = 0x40000000
    // (= 2.0f), so 5.0f could never appear.
    f.set_reg(0, 0x40200000u);  // 2.5f
    const StepResult vmov = f.cpu.step();
    ZLB_EXPECT_FALSE(vmov.faulted);
    ZLB_EXPECT_NEAR(f.cpu.vfp.read_f32(0), 2.5f, 1e-6);
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

ZLB_TEST(vfp_vcvt_between_single_and_double) {
    // VCVT.F64.F32 / VCVT.F32.F64 (ARM ARM A8.8.314, opc2 == 7): the sz bit names
    // the *source* type, so the D and M bits belong to the other operand.  This
    // form was decoded as an undefined instruction (audit L2) - a core halt in the
    // middle of any float routine that widens or narrows a value.
    // Encodings verified with capstone: EE B7 0A C0 / EE B7 0B C0 / EE B7 1A C0.
    Fixture f;
    ArmVfp& v = f.cpu.vfp;
    const u32 vcvt_f64_f32 = 0xEEB70AC0u;  // vcvt.f64.f32 d0, s0
    const u32 vcvt_f32_f64 = 0xEEB70BC0u;  // vcvt.f32.f64 s0, d0
    const u32 vcvt_f64_f32_d1 = 0xEEB71AC0u;  // vcvt.f64.f32 d1, s0
    const u32 vcvt_f32_f64_s2 = 0xEEB71BC0u;  // vcvt.f32.f64 s2, d0
    f.load(kCodeBase, {vcvt_f64_f32, vcvt_f32_f64, vcvt_f64_f32_d1, vcvt_f32_f64_s2});

    f.cpu.reset(kCodeBase);
    v.fpexc = ArmVfp::kFpexcEn;
    // NB: write_s/write_f32 differ - write_s stores a *raw* 32-bit pattern (the
    // integer converters use it), write_f32 stores a float.
    v.write_f32(0, 1.5f);
    f.cpu.step();
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    ZLB_EXPECT_NEAR(v.read_f64(0), 1.5, 1e-12);

    // Widening keeps the value, narrowing rounds to single precision.
    v.write_f64(0, 1.0 / 3.0);
    f.cpu.step();
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    ZLB_EXPECT_NEAR(static_cast<double>(v.read_f32(0)), 1.0 / 3.0, 1e-7);
    ZLB_EXPECT_NE(static_cast<double>(v.read_f32(0)), 1.0 / 3.0);

    // The destination's D bit (bit 22) is the *high* bit of the double register...
    v.write_f32(0, -2.25f);
    f.cpu.step();
    ZLB_EXPECT_NEAR(v.read_f64(1), -2.25, 1e-12);
    // ...and the low bit of the single register in the other direction.
    v.write_f64(0, 4.5);
    f.cpu.step();
    ZLB_EXPECT_NEAR(static_cast<double>(v.read_f32(2)), 4.5, 1e-7);
}

ZLB_TEST(vfp_vcvt_vs_vcvtr_rounding) {
    // ARM ARM A8.8.314 (VCVT / VCVTR between floating-point and integer): opc2
    // 1100 is the unsigned and 1101 the signed float -> integer converter, and
    // the N bit chooses the rounding: N == 1 is VCVT (round towards zero),
    // N == 0 is VCVTR (FPSCR.RMode). Ignoring N made VCVT follow RMode.
    Fixture f;
    ArmVfp& v = f.cpu.vfp;
    const u32 vcvtr_s = 0xEEBD0A42u;  // vcvtr.s32.f32 s0, s4
    const u32 vcvt_s = 0xEEBD0AC2u;   // vcvt.s32.f32  s0, s4
    const u32 vcvtr_u = 0xEEBC0A42u;  // vcvtr.u32.f32 s0, s4
    const u32 vcvt_u = 0xEEBC0AC2u;   // vcvt.u32.f32  s0, s4

    const auto run = [&](u32 instr, f32 value, u32 rmode) {
        v.reset();
        f.bus.write32(kCodeBase, instr);
        f.cpu.reset(kCodeBase);
        v.fpexc = ArmVfp::kFpexcEn;
        v.fpscr = (v.fpscr & ~(3u << 22)) | ((rmode & 3u) << 22);
        v.write_f32(4, value);
        f.cpu.step();
        return v.read_s(0);
    };

    // 2.5 with RMode = RP: VCVTR rounds away to 3, VCVT truncates to 2.
    ZLB_EXPECT_EQ(run(vcvtr_s, 2.5f, 1u), 3u);
    ZLB_EXPECT_EQ(run(vcvt_s, 2.5f, 1u), 2u);
    // RMode = RZ pins VCVTR to the same answer as VCVT: -2.5 truncates to -2.
    ZLB_EXPECT_EQ(run(vcvtr_s, -2.5f, 3u), 0xFFFFFFFEu);
    ZLB_EXPECT_EQ(run(vcvt_s, -2.5f, 3u), 0xFFFFFFFEu);
    // RMode = RP rounds a negative value towards zero here as well.
    ZLB_EXPECT_EQ(run(vcvtr_s, -2.5f, 1u), 0xFFFFFFFEu);
    // The unsigned converter differs only in the saturation behaviour, so the
    // same N bit rule applies. (N == 1 is VCVT, not "unsigned".)
    ZLB_EXPECT_EQ(run(vcvtr_u, 2.5f, 1u), 3u);
    ZLB_EXPECT_EQ(run(vcvt_u, 2.5f, 1u), 2u);
    // RMode = RN (the reset value) rounds to nearest, ties to even.
    ZLB_EXPECT_EQ(run(vcvtr_s, 2.5f, 0u), 2u);
    ZLB_EXPECT_EQ(run(vcvtr_s, 3.5f, 0u), 4u);
    // VCVT is unaffected by RMode and always truncates towards zero.
    ZLB_EXPECT_EQ(run(vcvt_s, 2.5f, 2u), 2u);
    ZLB_EXPECT_EQ(run(vcvt_s, -2.5f, 2u), 0xFFFFFFFEu);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
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

    // Section descriptor for VA 0x81000000 -> PA 0x80000000, AP = 0b001 (privileged RW),
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

    // AP=0b101 is privileged read-only: AP[2] is section descriptor bit 15.
    const u32 ro = 0x80000000u | 2u | (1u << 10) | (1u << 15) | (3u << 2);
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
    // AP[2:0] = 0b001 (privileged RW), domain 0, TEX/C/B = 0b011 (normal WB).
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
    // DACR = 1 means only domain 0 is client; domains 1..15 are "no access" and
    // must fault even for a perfectly valid section descriptor.
    Fixture f;
    ArmMmu& mmu = f.cpu.mmu;
    mmu.ttbr0 = 0x80010000u;
    mmu.dacr = 0x00000001u;
    mmu.sctlr |= 1u;

    const u32 index = (0x81000000u >> 20) & 0xFFFu;
    // Domain 0 descriptor: allowed (AP = 0b001 privileged RW).
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

ZLB_TEST(cp15_native_threadmgr_context_gate_preserves_per_core_thread_pointer) {
    Fixture f;
    // Genuine ThreadMgr writer at 4A1DE8, followed by an architectural URO
    // setup. All four writes precede every read to detect shared-core state.
    f.load_halfwords(kCodeBase, {0xEE0Du, 0x9F90u, 0xEE0Du, 0x7F70u});
    constexpr u32 gate = kCodeBase + 0x100u;
    // Unchanged mutex context check from linked 8100C3A0..8100C3BA. It reads
    // PRW/URO, masks URO, rereads PRW, and skips ILLEGAL_CONTEXT on nonzero PRW.
    f.load_halfwords(gate, {
        0xEE1Du, 0x8F90u, 0xEE1Du, 0x4F70u, 0x0420u, 0xEE0Du, 0x0F70u,
        0xEE1Du, 0x2F90u, 0xF247u, 0x1901u, 0xF2C8u, 0x0902u, 0xB92Au,
    });
    const u32 pointers[] = {0x6BA28u, 0x6BC28u, 0x6BE28u, 0x6B828u};
    std::vector<std::unique_ptr<ArmCore>> cores;
    for (u32 core = 0; core < 4; ++core) {
        cores.push_back(std::make_unique<ArmCore>(f.bus));
        ArmCore& cpu = *cores.back();
        cpu.core_id_ = core;
        cpu.reset(kCodeBase | 1u);
        cpu.set_register("CPSR", arm::kModeSystem | arm::kFlagT);
        cpu.set_register("SCR", 5);
        cpu.r[9] = pointers[core];
        cpu.r[7] = 0x12340056u + core;
        ZLB_EXPECT_FALSE(cpu.step().faulted);
        ZLB_EXPECT_FALSE(cpu.step().faulted);
    }
    for (u32 core = 0; core < 4; ++core) {
        ArmCore& cpu = *cores[core];
        cpu.set_pc(gate);
        for (int i = 0; i < 8; ++i) ZLB_EXPECT_FALSE(cpu.step().faulted);
        ZLB_EXPECT_EQ(cpu.r[8], pointers[core]);
        ZLB_EXPECT_EQ(cpu.r[2], pointers[core]);
        ZLB_EXPECT_EQ(cpu.r[4], 0x12340056u + core);
        ZLB_EXPECT_EQ(cpu.get_pc(), gate + 0x28u); // native nonzero-context branch
        u64 uro = 0;
        ZLB_EXPECT_TRUE(cpu.get_register("TPIDRURO", uro));
        ZLB_EXPECT_EQ(uro, (0x12340056u + core) << 16);
    }
}

ZLB_TEST(cp15_thread_ids_enforce_user_read_write_permissions_a32_and_thumb32) {
    const char* names[] = {"TPIDRURW", "TPIDRURO", "TPIDRPRW"};
    for (bool thumb : {false, true}) {
        for (bool nonsecure : {false, true}) {
            for (u32 op = 2; op <= 4; ++op) {
                for (bool read : {false, true}) {
                    Fixture f;
                    f.cpu.reset(kCodeBase + 0x100u);
                    f.cpu.set_register("CPSR", arm::kModeSystem);
                    f.cpu.set_register("SCR", nonsecure ? 1u : 0u);
                    // Seed all three using privileged instructions, not a
                    // public register shortcut; forbidden writes must retain it.
                    f.load(kCodeBase + 0x100u, {
                        arm_cp15(false, 0, 13, 0, 0, 2),
                        arm_cp15(false, 0, 13, 0, 0, 3),
                        arm_cp15(false, 0, 13, 0, 0, 4),
                    });
                    for (u32 seed = 2; seed <= 4; ++seed) {
                        f.set_reg(0, 0x12340000u + seed);
                        ZLB_EXPECT_FALSE(f.cpu.step().faulted);
                    }
                    const u32 instr = arm_cp15(read, 0, 13, read ? 2u : 0u, 0, op);
                    if (thumb) f.load_halfwords(kCodeBase, {static_cast<u16>(instr >> 16), static_cast<u16>(instr)});
                    else f.load(kCodeBase, {instr});
                    f.cpu.set_register("CPSR", arm::kModeUser | (thumb ? arm::kFlagT : 0u));
                    f.cpu.set_pc(kCodeBase);
                    f.set_reg(0, 0x56780000u + op);
                    f.set_reg(2, 0xDEADC0DEu);
                    const StepResult result = f.cpu.step();
                    const bool allowed = op == 2u || (op == 3u && read);
                    ZLB_EXPECT_EQ(result.faulted, !allowed);
                    ZLB_EXPECT_EQ(f.cpu.undefined_instruction, !allowed);
                    if (read) {
                        ZLB_EXPECT_EQ(f.reg(2), allowed ? 0x12340000u + op : 0xDEADC0DEu);
                    }
                    for (u32 stored = 2; stored <= 4; ++stored) {
                        u64 value = 0;
                        ZLB_EXPECT_TRUE(f.cpu.get_register(names[stored - 2], value));
                        const u32 expected = allowed && !read && stored == op ? 0x56780000u + op :
                                                                                 0x12340000u + stored;
                        ZLB_EXPECT_EQ(value, expected);
                    }
                }
            }
        }
    }
}

ZLB_TEST(cp15_thread_id_security_banks_monitor_access_roundtrip_and_reset) {
    Fixture f;
    f.cpu.reset(kCodeBase);
    f.cpu.set_register("CPSR", arm::kModeSystem);
    const auto seed = [&](u32 base) {
        f.load(kCodeBase + 0x400u, {
            arm_cp15(false, 0, 13, 0, 0, 2),
            arm_cp15(false, 0, 13, 0, 0, 3),
            arm_cp15(false, 0, 13, 0, 0, 4),
        });
        f.cpu.set_pc(kCodeBase + 0x400u);
        for (u32 op = 2; op <= 4; ++op) {
            f.set_reg(0, base + op);
            ZLB_EXPECT_FALSE(f.cpu.step().faulted);
        }
    };
    seed(0x12340000u);
    f.cpu.set_register("SCR", 1);
    seed(0x56780000u);
    f.cpu.mvbar = kCodeBase + 0x100u;
    f.load(kCodeBase, {
        0xE1600070u,                         // SMC #0 from Non-secure SYS
        arm_cp15(true, 0, 13, 6, 0, 2),
        arm_cp15(true, 0, 13, 7, 0, 3),
        arm_cp15(true, 0, 13, 8, 0, 4),
    });
    f.load(kCodeBase + 0x108u, {
        arm_cp15(true, 0, 13, 4, 0, 4),       // Monitor, SCR.NS=1: PRW_NS
        0xE3A00000u, arm_cp15(false, 0, 1, 0, 1, 0), // SCR.NS=0
        arm_cp15(true, 0, 13, 5, 0, 4),       // PRW_S
        0xE3A00001u, arm_cp15(false, 0, 1, 0, 1, 0), // SCR.NS=1
        0xE1B0F00Eu,                         // MOVS PC,LR: return to Non-secure
    });
    f.cpu.set_pc(kCodeBase);
    ZLB_EXPECT_FALSE(f.cpu.step().faulted);
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeMonitor);
    ZLB_EXPECT_TRUE(f.cpu.secure_state());
    for (int i = 0; i < 7; ++i) ZLB_EXPECT_FALSE(f.cpu.step().faulted);
    ZLB_EXPECT_EQ(f.reg(4), 0x56780004u);
    ZLB_EXPECT_EQ(f.reg(5), 0x12340004u);
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeSystem);
    ZLB_EXPECT_FALSE(f.cpu.secure_state());
    for (int i = 0; i < 3; ++i) ZLB_EXPECT_FALSE(f.cpu.step().faulted);
    ZLB_EXPECT_EQ(f.reg(6), 0x56780002u);
    ZLB_EXPECT_EQ(f.reg(7), 0x56780003u);
    ZLB_EXPECT_EQ(f.reg(8), 0x56780004u);

    // Other Op1/CRm combinations are not aliases for these registers.
    f.load(kCodeBase + 0x200u, {arm_cp15(false, 1, 13, 0, 0, 4),
                              arm_cp15(false, 0, 13, 0, 1, 4)});
    f.cpu.set_pc(kCodeBase + 0x200u);
    f.set_reg(0, 0xDEADBEEFu);
    ZLB_EXPECT_FALSE(f.cpu.step().faulted);
    ZLB_EXPECT_FALSE(f.cpu.step().faulted);
    u64 value = 0;
    ZLB_EXPECT_TRUE(f.cpu.get_register("TPIDRPRW", value));
    ZLB_EXPECT_EQ(value, 0x56780004u);

    // UNKNOWN hardware reset values use the emulator's documented zero choice.
    f.cpu.reset(kCodeBase);
    for (u32 ns : {0u, 1u}) {
        f.cpu.set_register("SCR", ns);
        for (const char* name : {"TPIDRURW", "TPIDRURO", "TPIDRPRW"}) {
            ZLB_EXPECT_TRUE(f.cpu.get_register(name, value));
            ZLB_EXPECT_EQ(value, 0u);
        }
    }
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
    ZLB_EXPECT_EQ(mmu.par, 0x80004000u);  // PAR.F=0: translation succeeded
    ZLB_EXPECT_EQ(f.reg(5), 0x80004000u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 8u);
}

ZLB_TEST(cp15_va_to_pa_probe_reports_a_fault_instead_of_aborting) {
    // A probe of an unmapped address must not raise a prefetch/data abort; the
    // status goes into PAR with PAR.F=1.
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
    ZLB_EXPECT_EQ(mmu.par, 0xBu);  // Section translation fault FS=5, F=1
    ZLB_EXPECT_TRUE((mmu.par >> 1) != 0u);
    // The probe fault is accounted for, but no exception was taken: only the
    // probe failed, the two instruction fetches translated fine.
    ZLB_EXPECT_EQ(mmu.total_faults, faults_before + 1u);
}

ZLB_TEST(cp15_va_to_pa_access_encodings_and_fault_status_match_armv7) {
    struct Permission {
        u32 ap;
        bool allowed[4];  // ATS1CPR, ATS1CPW, ATS1CUR, ATS1CUW
    };
    const Permission permissions[] = {
        {1u, {true, true, false, false}},
        {2u, {true, true, true, false}},
        {3u, {true, true, true, true}},
        {5u, {true, false, false, false}},
        {6u, {true, false, true, false}},
    };
    for (const Permission& permission : permissions) {
        for (u32 operation = 0; operation < 4; ++operation) {
            Fixture f;
            f.load(kCodeBase, {
                arm_cp15(false, 0, 7, 4, 8, operation),
                arm_cp15(true, 0, 7, 5, 4, 0),
            });
            f.cpu.reset(kCodeBase);
            f.cpu.mmu.ttbr0 = kDataBase;
            f.cpu.mmu.dacr = 1u | (1u << 6);  // client domains 0 and 3
            f.cpu.mmu.sctlr |= 1u;
            f.bus.write32(kDataBase + 0x800u * 4u, kCodeBase | (3u << 10) | 2u);
            f.bus.write32(kDataBase + 0x900u * 4u,
                          kCodeBase | ((permission.ap & 3u) << 10) |
                          ((permission.ap >> 2) << 15) | (3u << 5) | 2u);
            f.set_reg(4, 0x90001000u);
            f.cpu.step();
            f.cpu.step();
            // FS=13 for permission fault; PAR excludes DFSR domain and WnR.
            ZLB_EXPECT_EQ(f.reg(5), permission.allowed[operation] ? 0x80001000u : 0x1Bu);
            ZLB_EXPECT_EQ(f.cpu.exception_count, 0u);
            ZLB_EXPECT_FALSE(f.cpu.halted);
        }
    }
}

ZLB_TEST(cp15_va_to_pa_supersection_sets_ss_and_returns_supersection_address) {
    Fixture f;
    f.load(kCodeBase, {
        arm_cp15(false, 0, 7, 4, 8, 0),
        arm_cp15(true, 0, 7, 5, 4, 0),
    });
    f.cpu.reset(kCodeBase);
    f.cpu.mmu.ttbr0 = kDataBase;
    f.cpu.mmu.dacr = 1u;
    f.cpu.mmu.sctlr |= 1u;
    f.bus.write32(kDataBase + 0x800u * 4u, kCodeBase | (3u << 10) | 2u);
    f.bus.write32(kDataBase + 0x905u * 4u, kCodeBase | (1u << 18) | (3u << 10) | 2u);
    f.set_reg(4, 0x90567000u);
    f.cpu.step();
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(5), 0x80000002u); // SS=1, PA[23:12] comes from the input VA
}

ZLB_TEST(mmu_short_descriptor_ap_permissions_match_armv7) {
    // ARM ARM table B3-8 applies to sections, small pages and entire large
    // pages. Each mask below is {user W, user R, privileged W, privileged R}.
    // 0b100 is reserved, so this table asserts only architecturally defined APs.
    struct Case { u32 ap; u32 permissions; };
    const Case cases[] = {{0u, 0u}, {1u, 3u}, {2u, 7u}, {3u, 15u},
                          {5u, 1u}, {6u, 5u}, {7u, 5u}};
    constexpr u32 l1 = 0x80010000u;
    constexpr u32 l2 = 0x80018000u;
    constexpr u32 va = 0x81000000u;
    for (u32 format : {0u, 1u, 2u}) { // section, small, large
        for (const Case& test : cases) {
            Fixture f;
            ArmMmu& mmu = f.cpu.mmu;
            mmu.ttbr0 = l1;
            mmu.dacr = 1u;
            mmu.sctlr |= 1u; // AFE=0: full AP[2:0] model
            const u32 l1_address = l1 + (va >> 20) * 4u;
            if (format == 0u) {
                f.bus.write32(l1_address, 0x80000002u |
                              ((test.ap & 3u) << 10) | ((test.ap >> 2) << 15));
            } else {
                f.bus.write32(l1_address, l2 | 1u);
                for (u32 index = 0; index < 16u; ++index) {
                    const u32 base = format == 2u ? 0x80000000u : 0x80000000u + index * 0x1000u;
                    f.bus.write32(l2 + index * 4u, base | (format == 2u ? 1u : 2u) |
                                  ((test.ap & 3u) << 4) | ((test.ap >> 2) << 9));
                }
            }
            // The old large-page check invented different permissions in each
            // 16KiB quarter. Every quarter must obey the same AP field.
            for (u32 offset : {0u, 0x4000u, 0x8000u, 0xC000u}) {
                for (u32 mode : {arm::kModeSystem, arm::kModeUser}) {
                    for (bool write : {false, true}) {
                        const u32 permission_bit = (mode == arm::kModeUser ? 2u : 0u) + (write ? 1u : 0u);
                        const bool allowed = (test.permissions & (1u << permission_bit)) != 0u;
                        const auto result = mmu.translate(va + offset, write, false, mode);
                        ZLB_EXPECT_EQ(result.ok, allowed);
                        if (allowed) ZLB_EXPECT_EQ(result.phys_addr, 0x80000000u + offset);
                        else ZLB_EXPECT_TRUE(result.fault == arm::MmFaultKind::Permission);
                    }
                }
            }
        }
    }
}

ZLB_TEST(mmu_nskbl_large_page_allows_genuine_privileged_clear) {
    // The actual NSKBL STRD fault walk at 0x51013548. Its large-page AP=001
    // permits privileged writes across all 64KiB; XN prohibits only fetches.
    Fixture f;
    f.bus.add_ram("nskbl", 0x70000u, 0x40300000u, "genuine walk");
    ArmMmu& mmu = f.cpu.mmu;
    mmu.sctlr = 0x20005805u; // AFE=1, AF already set in the descriptor
    mmu.ttbr0 = 0x40308000u;
    mmu.ttbr1 = 0x4030C04Au;
    mmu.ttbcr = 2u;
    mmu.dacr = 0x55555555u;
    f.bus.write32(0x40308C00u, 0x40316181u); // coarse table, domain12
    for (u32 index = 0; index < 16u; ++index)
        f.bus.write32(0x40316000u + index * 4u, 0x40359011u);
    for (u32 offset : {0u, 0x3FF8u, 0x4000u, 0x8000u, 0xC000u, 0xFFF8u}) {
        const auto store = mmu.translate(0x30000000u + offset, true, false, arm::kModeSystem);
        ZLB_EXPECT_TRUE(store.ok);
        ZLB_EXPECT_EQ(store.phys_addr, 0x40350000u + offset);
        ZLB_EXPECT_TRUE(store.normal); // TEX=001,C=B=0: Normal, non-cacheable
        ZLB_EXPECT_FALSE(mmu.translate(0x30000000u + offset, false, true, arm::kModeSystem).ok);
    }
    ZLB_EXPECT_FALSE(mmu.translate(0x30000000u, false, false, arm::kModeUser).ok);
    ZLB_EXPECT_EQ(f.bus.read32(0x40316000u), 0x40359011u);
}

ZLB_TEST(mmu_afe_access_flag_is_ap_zero_for_all_short_descriptors) {
    constexpr u32 l1 = 0x80010000u;
    constexpr u32 l2 = 0x80018000u;
    constexpr u32 va = 0x81000000u;
    for (u32 format : {0u, 1u, 2u}) {
        Fixture f;
        ArmMmu& mmu = f.cpu.mmu;
        mmu.ttbr0 = l1;
        mmu.dacr = 1u;
        mmu.sctlr |= 1u;
        const u32 l1_address = l1 + (va >> 20) * 4u;
        const u32 address = format == 0u ? l1_address : l2;
        // AP=010 grants privileged RW when AFE=0. AFE=1 instead makes
        // AP[0]=0 an access-flag fault, even though AP[2:1] permits writing.
        const u32 descriptor = format == 0u ? (0x80000002u | (2u << 10))
                                             : (0x80000000u | (format == 2u ? 1u : 2u) | (2u << 4));
        if (format != 0u) f.bus.write32(l1_address, l2 | 1u);
        f.bus.write32(address, descriptor);
        ZLB_EXPECT_TRUE(mmu.translate(va, true, false, arm::kModeSystem).ok);
        mmu.sctlr |= 1u << 29;
        const auto fault = mmu.translate(va, true, false, arm::kModeSystem);
        ZLB_EXPECT_FALSE(fault.ok);
        ZLB_EXPECT_TRUE(fault.fault == arm::MmFaultKind::AccessFlag);
        ZLB_EXPECT_EQ(fault.fsr_full, format == 0u ? 0x803u : 0x806u);
        ZLB_EXPECT_EQ(f.bus.read32(address), descriptor); // software must set AF
        f.bus.write32(address, descriptor | (1u << (format == 0u ? 10u : 4u)));
        ZLB_EXPECT_TRUE(mmu.translate(va, true, false, arm::kModeUser).ok);
    }
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
    f.bus.write32(l2 + index2 * 4u, 0x80010000u | 2u | (3u << 4) | (1u << 9));  // small page, AP=0b111 (RO all)

    u32 pa = 0;
    std::string fault;
    ZLB_EXPECT_TRUE(f.cpu.translate(0x81000ABCu, false, false, pa, fault));
    ZLB_EXPECT_EQ(pa, 0x80010ABCu);
}

ZLB_TEST(arm_thumb2_bitfield_matches_reference) {
    // The kernel's object dispatch reads an object's type with `ubfx r5, r1, #24, #4`
    // (0x5100A546) and compares it against 10, so a wrong UBFX would send NSKBL down
    // the wrong branch.  Expected values come from Unicorn (a real ARMv7 core model)
    // running the same encodings, produced by keystone.
    Fixture f;
    struct Case {
        u32 word;       // little-endian halfword pair: hw1 | (hw2 << 16)
        u32 r3_in;
        u32 r5_in;
        u32 expect;
    };
    const Case cases[] = {
        {0x2307F3C5u, 0u, 0x12345678u, 0x00000056u},  // ubfx r3, r5, #8, #8
        {0x2307F345u, 0u, 0x12345678u, 0x00000056u},  // sbfx r3, r5, #8, #8
        {0x230FF365u, 0xDEADBEEFu, 0x12345678u, 0xDEAD78EFu},  // bfi r3, r5, #8, #8
    };
    for (const Case& c : cases) {
        f.load(kCodeBase, {c.word});
        f.cpu.reset(kCodeBase | 1u);
        f.set_reg(3, c.r3_in);
        f.set_reg(5, c.r5_in);
        f.cpu.step();
        ZLB_EXPECT_TRUE(!f.cpu.undefined_instruction);
        ZLB_EXPECT_EQ(f.reg(3), c.expect);
    }
}

ZLB_TEST(arm_thumb2_ssat_usat_saturate_immediate) {
    // Round 226, found by the differential sweep against Unicorn.  The saturate
    // immediate of SSAT lives in hw2<4:0> as width - 1 (USAT stores the width itself),
    // hw1 bit 5 selects the shift type (0 = LSL, 1 = ASR) and hw2<14:12>:<7:6> the
    // amount; with a zero amount and opc 0x12/0x1A the same word is SSAT16/USAT16.
    // Before the fix `ssat #8` saturated to 6 bits (0x1F instead of 0x7F) and every
    // shifted form returned its input unchanged.  Expected values: Unicorn.
    Fixture f;
    struct Case {
        u32 word;
        int r5;
        u32 r5_in;
        u32 expect;
    };
    const Case cases[] = {
        {0x0307F305u, 5, 0x7FFFFFFFu, 0x0000007Fu},  // ssat r3, #8, r5
        {0x0307F305u, 5, 0xFFFFFF80u, 0xFFFFFF80u},  // ssat r3, #8, r5 (in range)
        {0x130FF325u, 5, 0x0F0E0D0Cu, 0x00007FFFu},  // ssat r3, #16, r5, asr #4
        {0x0307F325u, 5, 0x0F0E0D0Cu, 0x007F007Fu},  // ssat16 r3, #8, r5
        {0x0384F385u, 5, 0x80018002u, 0x0000000Fu},  // usat r3, #4, r5, lsl #2
        {0x0308F3A5u, 5, 0x0000FFFFu, 0x00000000u},  // usat16 r3, #8, r5 (signed halves)
        {0x0308F3A5u, 5, 0x00010002u, 0x00010002u},  // usat16 r3, #8, r5
    };
    for (const Case& c : cases) {
        f.load(kCodeBase, {c.word});
        f.cpu.reset(kCodeBase | 1u);
        f.set_reg(c.r5, c.r5_in);
        f.cpu.step();
        ZLB_EXPECT_TRUE(!f.cpu.undefined_instruction);
        ZLB_EXPECT_EQ(f.reg(3), c.expect);
    }
}

ZLB_TEST(arm_multiply_mls_operand_order) {
    // Round 226: MLS is Ra - Rn x Rm; both the ARM and the Thumb decoder computed
    // Rn x Rm - Ra, i.e. the negated result.  Unicorn for `mls r3, r5, r6, r7` with
    // r5 = 0xFFFFFFFF, r6 = 1, r7 = 1 gives 2 (1 - (-1)).
    // Thumb encoding `mls r3, r5, r6, r7` = hw1 0xFB05, hw2 0x7316; the ARM one is
    // 0xE0673596 (`mls r3, r5, r6, r7`).
    Fixture f;
    struct Case {
        u32 word;
        bool thumb;
        u32 expect;
    };
    const Case cases[] = {
        {0x7316FB05u, true, 0x00000002u},
        {0xE0637695u, false, 0x00000002u},
    };
    for (const Case& c : cases) {
        f.load(kCodeBase, {c.word});
        f.cpu.reset(c.thumb ? (kCodeBase | 1u) : kCodeBase);
        f.set_reg(5, 0xFFFFFFFFu);
        f.set_reg(6, 1u);
        f.set_reg(7, 1u);
        f.cpu.step();
        ZLB_EXPECT_TRUE(!f.cpu.undefined_instruction);
        ZLB_EXPECT_EQ(f.reg(3), c.expect);
    }
}

ZLB_TEST(arm_dp_register_shift_without_s_leaves_flags_alone) {
    // Round 225: the carry-out of a register-specified shift only reaches C when the
    // instruction has S == 1.  `shift_reg` used to write C unconditionally, so
    // `mov r3, r5, lsl r6` (encoding 0xE1A03615, no S) set C.  Reference: Unicorn runs
    // the same word with r5 = 0xFFFFFFFF, r6 = 1 and reports r3 = 0xFFFFFFFE with the
    // flags unchanged; `movs` (0xE1B03615) must set C.
    Fixture f;
    f.load(kCodeBase, {0xE1A03615u});
    f.cpu.reset(kCodeBase);
    f.set_reg(5, 0xFFFFFFFFu);
    f.set_reg(6, 1u);
    f.cpu.cpsr &= ~(arm::kFlagN | arm::kFlagZ | arm::kFlagC | arm::kFlagV);
    f.cpu.step();
    ZLB_EXPECT_TRUE(!f.cpu.undefined_instruction);
    ZLB_EXPECT_EQ(f.reg(3), 0xFFFFFFFEu);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagC) == 0u);

    // The flag-setting form still has to update C.
    f.load(kCodeBase, {0xE1B03615u});
    f.cpu.reset(kCodeBase);
    f.set_reg(5, 0xFFFFFFFFu);
    f.set_reg(6, 1u);
    f.cpu.cpsr &= ~(arm::kFlagN | arm::kFlagZ | arm::kFlagC | arm::kFlagV);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(3), 0xFFFFFFFEu);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagC) != 0u);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagN) != 0u);
}

ZLB_TEST(arm_thumb32_multiply_long_accumulate) {
    // Round 225: the differential sweep against Unicorn (tools/arm_probe + one
    // instruction per case) found that SMLAL and UMLAL were reported undefined - the
    // decoder only knew the plain SMULL (op1 = 0x8) and UMULL (op1 = 0xA) slots,
    // while the accumulating forms are op1 = 0xC and 0xE.  Expected values come from
    // Unicorn running the same encodings (keystone: `smlal r3, r4, r5, r6` is
    // hw1 = 0xFBC5, hw2 = 0x3406).
    Fixture f;
    struct Case {
        u32 word;
        u32 r3_in;
        u32 r4_in;
        u32 r5_in;
        u32 r6_in;
        u32 expect_lo;
        u32 expect_hi;
    };
    const Case cases[] = {
        // smlal r3, r4, r5, r6  - signed product added to RdHi:RdLo
        {0x3406FBC5u, 0x11111111u, 0x22222222u, 0x00010002u, 0xFFFFFFFFu, 0x1110110Fu,
         0x22222222u},
        // umlal r3, r4, r5, r6  - unsigned counterpart
        {0x3406FBE5u, 0x11111111u, 0x22222222u, 0x00010002u, 0xFFFFFFFFu, 0x1110110Fu,
         0x22232224u},
        // smlal with a negative product (-0x20000) added to zero
        {0x3406FBC5u, 0x00000000u, 0x00000000u, 0xFFFF0000u, 0x00000002u, 0xFFFE0000u,
         0xFFFFFFFFu},
    };
    for (const Case& c : cases) {
        f.load(kCodeBase, {c.word});
        f.cpu.reset(kCodeBase | 1u);
        f.set_reg(3, c.r3_in);
        f.set_reg(4, c.r4_in);
        f.set_reg(5, c.r5_in);
        f.set_reg(6, c.r6_in);
        f.cpu.step();
        ZLB_EXPECT_TRUE(!f.cpu.undefined_instruction);
        ZLB_EXPECT_EQ(f.reg(3), c.expect_lo);
        ZLB_EXPECT_EQ(f.reg(4), c.expect_hi);
    }
}

ZLB_TEST(mmu_small_page_xn_is_bit_zero) {    // Round 216: XN lives in bit 0 of a small-page descriptor, not bit 15 - bit 15
    // is part of PA[31:12] for a 4 KiB page.  Reading bit 15 there made every page
    // whose physical address has bit 15 set execute-never, and PA 0x40118000 (where
    // the model backs the ARM low window) is one of them: the first instruction
    // fetch of a vector there raised IFSR = 0xF and NSKBL's abort handler could
    // never run.
    Fixture f;
    ArmMmu& mmu = f.cpu.mmu;
    const u32 l1 = 0x80010000u;
    const u32 l2 = 0x80018000u;
    mmu.ttbr0 = l1;
    mmu.dacr = 0x00000001u;
    mmu.sctlr |= 1u;

    const u32 index1 = (0x81000000u >> 20) & 0xFFFu;
    const u32 index2 = (0x81000000u >> 12) & 0xFFu;
    f.bus.write32(l1 + index1 * 4u, l2 | 1u);        // coarse table, domain 0
    // PA 0x80010000 has bit 15 set; AP = 0b011 (privileged RW); bit 0 = 0 -> XN = 0.
    f.bus.write32(l2 + index2 * 4u, 0x80010000u | 2u | (3u << 4));

    u32 pa = 0;
    std::string fault;
    ZLB_EXPECT_TRUE(f.cpu.translate(0x81000ABCu, false, false, pa, fault));
    ZLB_EXPECT_EQ(pa, 0x80010ABCu);
    pa = 0;
    ZLB_EXPECT_TRUE(f.cpu.translate(0x81000ABCu, false, true, pa, fault));  // fetch
    ZLB_EXPECT_EQ(pa, 0x80010ABCu);

    // Setting bit 0 marks the page execute-never: the fetch faults, the read does not.
    f.bus.write32(l2 + index2 * 4u, 0x80010000u | 3u | (3u << 4));
    pa = 0;
    ZLB_EXPECT_FALSE(f.cpu.translate(0x81000ABCu, false, true, pa, fault));
    pa = 0;
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

ZLB_TEST(neon_native_threadmgr_register_initializer) {
    // Firmware 1.04 ThreadMgr PT_LOAD0, linked 81000000..81000044. The native
    // boot stopped at F2201110: Q=0 legally copies D0 to the odd register D1.
    // Keep the instruction bytes unchanged and relocate only the data pointer.
    Fixture f;
    f.load(kCodeBase, {
        0xE59F009Cu, 0xED900B00u, 0xF2201110u, 0xF2204150u,
        0xF2206150u, 0xF2208150u, 0xF220A150u, 0xF220C150u,
        0xF220E150u, 0xF2600150u, 0xF2602150u, 0xF2604150u,
        0xF2606150u, 0xF2608150u, 0xF260A150u, 0xF260C150u,
        0xF260E150u, 0xE12FFF1Eu, 0x7F80DEADu, 0x7FF8DEADu,
    });
    f.bus.write32(kCodeBase + 0xA4u, kCodeBase + 0x48u);
    f.cpu.reset(kCodeBase);
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    f.cpu.vfp.fpscr = 0xA1000015u;
    f.cpu.cpsr |= 0xA0000000u;
    const u32 cpsr_before = f.cpu.cpsr;
    for (int d = 0; d < 32; ++d) f.cpu.vfp.write_d32(d, 0x0123456789ABCDEFull + d);
    f.set_reg(14, kCodeBase + 0x200u);
    for (int i = 0; i < 18; ++i) {
        const StepResult result = f.cpu.step();
        ZLB_EXPECT_FALSE(result.faulted);
        if (result.faulted) return;
    }
    for (int d = 0; d < 32; ++d) {
        const u64 expected = d == 2 || d == 3 ? 0x0123456789ABCDEFull + d :
                                               0x7FF8DEAD7F80DEADull;
        ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d), expected);
    }
    ZLB_EXPECT_EQ(f.reg(15), kCodeBase + 0x200u);
    ZLB_EXPECT_EQ(f.cpu.cpsr, cpsr_before);
    ZLB_EXPECT_EQ(f.cpu.vfp.fpscr, 0xA1000015u);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
}

ZLB_TEST(neon_native_syscon_command_padding) {
    // Unchanged Syscon PT_LOAD0 81001BEA..81001CA0. Actual ARM3 stops on
    // FFC70E1F at VA4F9BEA with R2=28,R7=3,R12=24,LR=4. The vector fill
    // and byte tail must fill exactly buffer[20..47], leaving its header intact.
    Fixture f;
    f.load_halfwords(kCodeBase, {
        0xFFC7,0x0E1F,0x2001,0xF106,0x0913,0x4287,0xF107,0x38FF,
        0xF106,0x031B,0xF008,0x0803,0xF949,0x070F,0xD927,0xF1B8,
        0x0F00,0xD012,0xF1B8,0x0F01,0xD00A,0xF1B8,0x0F02,0xD004,
        0xF943,0x070F,0x2002,0xF106,0x0323,0x3001,0xF943,0x070D,
        0x3001,0xF943,0x070D,0x4287,0xD911,0x461E,0xF103,0x0908,
        0x3004,0xF946,0x070D,0x3608,0xF949,0x070F,0xF946,0x070F,
        0xF103,0x0618,0x3320,0x4287,0xF946,0x070F,0xD8ED,0x4594,
        0x44E6,0xD021,0xF10E,0x0301,0x22FF,0x2B1F,0xF1CE,0x071F,
        0xF801,0x200E,0xF007,0x0003,0xDC16,0xB158,0x2801,0xD005,
        0x2802,0xBF1C,0x54CA,0x3301,0x54CA,0x3301,0x54CA,0x3301,
        0x2B1F,0xDC09,0x54CA,0x1C58,0x1CDE,0x1C9F,0x3304,0x540A,
        0x2B1F,0x55CA,0x558A,0xDDF5,
    });
    f.cpu.reset(kCodeBase | 1u);
    f.cpu.cpsr = 0x6000003Fu;
    f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
    f.cpu.vfp.fpscr = 0xA1000015u;
    f.set_reg(0, 0xFFFFFFFDu); f.set_reg(1, kDataBase + 16u);
    f.set_reg(2, 28u); f.set_reg(3, 0u); f.set_reg(4, kDataBase);
    f.set_reg(6, kDataBase + 1u); f.set_reg(7, 3u);
    f.set_reg(8, 2u); f.set_reg(12, 24u); f.set_reg(14, 4u);
    for (u32 byte = 0; byte < 56u; ++byte) f.bus.write8(kDataBase + byte, static_cast<u8>(byte));
    for (int d = 0; d < 32; ++d) f.cpu.vfp.write_d32(d, 0x0123456789ABCDEFull + d);
    constexpr u32 end = kCodeBase + 0xB8u; // native 81001CA2
    for (int step = 0; step < 200 && f.cpu.get_pc() != end; ++step) {
        const StepResult result = f.cpu.step();
        ZLB_EXPECT_FALSE(result.faulted);
        if (result.faulted) return;
    }
    ZLB_EXPECT_EQ(f.cpu.get_pc(), end);
    for (u32 byte = 0; byte < 56u; ++byte) {
        ZLB_EXPECT_EQ(f.bus.read8(kDataBase + byte), byte >= 20u && byte < 48u ? 0xFFu : byte);
    }
    for (int d = 0; d < 32; ++d) {
        ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d), d == 16 ? ~u64{0} : 0x0123456789ABCDEFull + d);
    }
    ZLB_EXPECT_EQ(f.cpu.vfp.fpscr, 0xA1000015u);
    unsigned length = 0;
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase, true, length) == "vmov.i8 d16, #FF");
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase + 0x18u, true, length) == "vst1.8 {d16}, [r9]");
}

ZLB_TEST(neon_modified_immediate_vmoves_and_thumb_class_selection) {
    struct Case { u32 cmode; bool op; u32 imm8; u64 expected; };
    const Case cases[] = {
        {0,false,0xA5,0x000000A5000000A5ull}, {2,false,0xA5,0x0000A5000000A500ull},
        {4,false,0xA5,0x00A5000000A50000ull}, {6,false,0xA5,0xA5000000A5000000ull},
        {8,false,0xA5,0x00A500A500A500A5ull}, {10,false,0xA5,0xA500A500A500A500ull},
        {12,false,0xA5,0x0000A5FF0000A5FFull}, {13,false,0xA5,0x00A5FFFF00A5FFFFull},
        {14,false,0xA5,0xA5A5A5A5A5A5A5A5ull}, {14,true,0xA5,0xFF00FF0000FF00FFull},
        {15,false,0x70,0x3F8000003F800000ull}, {14,false,0,0}, {0,false,0,0},
    };
    for (bool thumb : {false, true}) {
        for (const Case& test : cases) {
            for (u32 d : {1u, 16u, 30u, 31u}) {
                const bool quad = d == 30u;
                Fixture f;
                load_neon(f, arm_neon_mov_imm(d, quad, test.cmode, test.op, test.imm8), thumb);
                f.cpu.cpsr |= 0xA8000000u;
                const u32 psr = f.cpu.cpsr;
                for (int reg = 0; reg < 32; ++reg) f.cpu.vfp.write_d32(reg, 0x1122334455667788ull + reg);
                const StepResult result = f.cpu.step();
                ZLB_EXPECT_FALSE(result.faulted);
                for (int reg = 0; reg < 32; ++reg) {
                    const bool written = reg == static_cast<int>(d) || (quad && reg == static_cast<int>(d + 1u));
                    ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(reg), written ? test.expected : 0x1122334455667788ull + reg);
                }
                ZLB_EXPECT_EQ(f.cpu.cpsr, psr);
                ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 4u);
            }
        }
        for (u32 instr : {arm_neon_mov_imm(17,true,14,false,0xFF),
                         arm_neon_mov_imm(16,false,15,true,0xA5),
                         arm_neon_mov_imm(16,false,1,false,0xA5),
                         arm_neon_mov_imm(16,false,2,false,0)}) {
            Fixture f; load_neon(f, instr, thumb);
            f.cpu.vfp.write_d32(16, 0xABCDEF1234567890ull);
            ZLB_EXPECT_TRUE(f.cpu.step().faulted);
            ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(16), 0xABCDEF1234567890ull);
        }
    }
    // EF and FF also carry the existing register logical families, with U
    // preserved at its architectural A32 position rather than dropped.
    for (u32 op : {2u, 4u}) {
        Fixture f; load_neon(f, arm_neon_logical(op,false,17,1,31),true);
        f.cpu.vfp.write_d32(1, 0x0F0F0F0F0F0F0F0Full);
        f.cpu.vfp.write_d32(31, 0x3333333333333333ull);
        ZLB_EXPECT_FALSE(f.cpu.step().faulted);
        ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(17), op == 2u ? 0x3F3F3F3F3F3F3F3Full : 0x3C3C3C3C3C3C3C3Cull);
    }
}

ZLB_TEST(neon_vst1_element_endian_register_lists_and_writeback) {
    constexpr u32 types[] = {7u,10u,6u,2u};
    for (bool thumb : {false,true}) for (bool big : {false,true}) {
        for (u32 size = 0; size < 4u; ++size) for (u32 count = 1; count <= 4u; ++count) {
            for (u32 m : {15u,13u,2u,0u}) {
                Fixture f; load_neon(f,arm_neon_vst1(32u-count,types[count-1],size,0,0,m),thumb);
                // Unaligned base exercises MemU. Register-index writeback must
                // use the old Rm, including Rm == Rn.
                const u32 address = kDataBase + 3u;
                f.set_reg(0,address); f.set_reg(2,37u);
                if (big) f.cpu.cpsr |= arm::kFlagE;
                const u32 psr = f.cpu.cpsr;
                for (u32 reg = 0; reg < count; ++reg) f.cpu.vfp.write_d32(static_cast<int>(32u-count+reg),0x0123456789ABCDEFull + reg);
                f.bus.write8(address-1u,0xAAu); f.bus.write8(address+8u*count,0xBBu);
                ZLB_EXPECT_FALSE(f.cpu.step().faulted);
                const u32 width = 1u << size;
                for (u32 reg = 0; reg < count; ++reg) for (u32 byte = 0; byte < 8u; ++byte) {
                    const u32 source = (byte / width) * width + (big ? width-1u-byte%width : byte%width);
                    const u32 expected = static_cast<u32>(((0x0123456789ABCDEFull+reg) >> (source*8u)) & 0xFFu);
                    ZLB_EXPECT_EQ(f.bus.read8(address+reg*8u+byte),expected);
                }
                const u32 increment = m == 13u ? 8u*count : m == 0u ? address : 37u;
                ZLB_EXPECT_EQ(f.reg(0),m == 15u ? address : address+increment);
                ZLB_EXPECT_EQ(f.bus.read8(address-1u),0xAAu);
                ZLB_EXPECT_EQ(f.bus.read8(address+8u*count),0xBBu);
                ZLB_EXPECT_EQ(f.cpu.cpsr,psr);
            }
        }
    }
}

ZLB_TEST(neon_vst1_alignment_translation_fault_and_it_suppression) {
    // Crossing two independently translated pages must never write the
    // physically adjacent decoy; a second-page fault leaves Rn unchanged.
    for (bool fault : {false,true}) {
        Fixture f; load_neon(f,arm_neon_vst1(31,7,0,0,0,13),true);
        ArmMmu& mmu=f.cpu.mmu; mmu.ttbr0=kDataBase; mmu.dacr=1u; mmu.sctlr|=1u;
        const u32 l2=kDataBase+0x8000u;
        f.bus.write32(kDataBase+0x800u*4u,0x80000C0Eu);
        f.bus.write32(kDataBase+0x810u*4u,l2|1u);
        f.bus.write32(l2,0x8001A01Eu); f.bus.write32(l2+4u,fault ? 0u : 0x8001C01Eu);
        f.bus.write32(0x8001B000u,0xA5A5A5A5u);
        f.set_reg(0,0x81000FFCu); f.cpu.vfp.write_d32(31,0x0123456789ABCDEFull);
        ZLB_EXPECT_EQ(f.cpu.step().faulted,fault);
        ZLB_EXPECT_EQ(f.bus.read32(0x8001AFFCu),0x89ABCDEFu);
        ZLB_EXPECT_EQ(f.bus.read32(0x8001B000u),0xA5A5A5A5u);
        if (fault) {
            ZLB_EXPECT_EQ(f.reg(0),0x81000FFCu); ZLB_EXPECT_EQ(mmu.dfar,0x81001000u);
        } else {
            ZLB_EXPECT_EQ(f.reg(0),0x81001004u); ZLB_EXPECT_EQ(f.bus.read32(0x8001C000u),0x01234567u);
        }
    }
    for (u32 size : {1u,2u,3u}) {
        Fixture f; load_neon(f,arm_neon_vst1(16,7,size,0,0,13),true);
        f.cpu.mmu.sctlr|=2u; f.set_reg(0,kDataBase+1u); f.bus.write32(kDataBase,0xA5A5A5A5u);
        ZLB_EXPECT_TRUE(f.cpu.step().faulted); ZLB_EXPECT_EQ(f.reg(0),kDataBase+1u);
        ZLB_EXPECT_EQ(f.cpu.mmu.dfsr,0x801u); ZLB_EXPECT_EQ(f.bus.read32(kDataBase),0xA5A5A5A5u);
    }
    for (u32 cb : {0u,4u}) for (u32 size : {1u,2u,3u}) {
        Fixture f; load_neon(f,arm_neon_vst1(16,7,size,0,0,13),true);
        ArmMmu& mmu=f.cpu.mmu; mmu.ttbr0=kDataBase; mmu.dacr=1u; mmu.sctlr|=1u;
        const u32 l2=kDataBase+0x8000u;
        f.bus.write32(kDataBase+0x800u*4u,0x80000C0Eu);
        f.bus.write32(kDataBase+0x810u*4u,l2|1u);
        f.bus.write32(l2,0x8001A012u|cb); // Strongly-ordered or Device, A=0
        f.bus.write32(0x8001A000u,0xA5A5A5A5u); f.set_reg(0,0x81000001u);
        ZLB_EXPECT_TRUE(f.cpu.step().faulted); ZLB_EXPECT_EQ(f.reg(0),0x81000001u);
        ZLB_EXPECT_EQ(mmu.dfar,0x81000001u); ZLB_EXPECT_EQ(mmu.dfsr,0x801u);
        ZLB_EXPECT_EQ(f.bus.read32(0x8001A000u),0xA5A5A5A5u);
    }
    Fixture byte_store; load_neon(byte_store,arm_neon_vst1(31,7,0,0,0,13),true);
    byte_store.cpu.mmu.sctlr|=2u; byte_store.set_reg(0,kDataBase+1u);
    byte_store.cpu.vfp.write_d32(31,0x0123456789ABCDEFull);
    ZLB_EXPECT_FALSE(byte_store.cpu.step().faulted); // bytes retain alignment 1 with A=1
    ZLB_EXPECT_EQ(byte_store.bus.read64(kDataBase+1u),0x0123456789ABCDEFull);
    ZLB_EXPECT_EQ(byte_store.reg(0),kDataBase+9u);
    // Explicit :64 alignment faults with A=0, while illegal list/alignment
    // encodings are undefined before any write or register update.
    for (u32 instr : {arm_neon_vst1(16,7,0,1,0,13),arm_neon_vst1(31,10,0,0,0,13),
                     arm_neon_vst1(16,7,0,2,0,13),arm_neon_vst1(16,7,0,0,15,13)}) {
        Fixture f; load_neon(f,instr,true); f.set_reg(0,kDataBase+1u); f.bus.write32(kDataBase,0xA5A5A5A5u);
        ZLB_EXPECT_TRUE(f.cpu.step().faulted); ZLB_EXPECT_EQ(f.reg(0),kDataBase+1u);
        ZLB_EXPECT_EQ(f.bus.read32(kDataBase),0xA5A5A5A5u);
    }
    Fixture skipped;
    skipped.load_halfwords(kCodeBase,{0xBF08u,0xFFC7u,0x0E1Fu,0xBF08u,0xF940u,0x070Du});
    skipped.cpu.reset(kCodeBase|1u); skipped.cpu.cpsr &= ~arm::kFlagZ;
    skipped.set_reg(0,kDataBase); skipped.cpu.vfp.write_d32(16,0x0123456789ABCDEFull);
    for (int step=0;step<4;++step) ZLB_EXPECT_FALSE(skipped.cpu.step().faulted);
    ZLB_EXPECT_EQ(skipped.cpu.vfp.read_d32(16),0x0123456789ABCDEFull);
    ZLB_EXPECT_EQ(skipped.bus.read32(kDataBase),0u); ZLB_EXPECT_EQ(skipped.reg(0),kDataBase);
}

ZLB_TEST(neon_vld1_exact_native_syscon_halfword_instruction_and_neighbors) {
    Fixture f;
    // Genuine Syscon4F8B4C..4F8B5E: R7 is the response buffer, LR is the
    // eight-byte block count. The VLD1 itself starts at a halfword-only PC.
    f.load_halfwords(kCodeBase + 2u, {0x463Au,0x2101u,0x458Eu,0xF10Eu,0x3CFFu,
                                     0xF962u,0x070Du,0xF00Cu,0x0C03u});
    f.cpu.reset((kCodeBase + 2u) | 1u);
    f.set_reg(7,kDataBase + 3u); f.set_reg(14,1u); f.set_reg(13,kDataBase + 0xF00u);
    for (int d=0;d<32;++d) f.cpu.vfp.write_d32(d,0x8877665544332211ull + d);
    constexpr u8 response[] = {4u,0u,6u,0u,0u,0x60u,0x40u,0u,0x55u,0u};
    for (u32 i=0;i<sizeof(response);++i) f.bus.write8(kDataBase + 3u + i,response[i]);
    for (unsigned step=0;step<6;++step) ZLB_EXPECT_FALSE(f.cpu.step().faulted);
    ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(16),0x0040600000060004ull);
    ZLB_EXPECT_EQ(f.reg(2),kDataBase + 11u); ZLB_EXPECT_EQ(f.reg(7),kDataBase + 3u);
    ZLB_EXPECT_EQ(f.reg(13),kDataBase + 0xF00u); ZLB_EXPECT_EQ(f.reg(14),1u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(),kCodeBase + 20u); ZLB_EXPECT_EQ(f.reg(12),0u);
    for (int d=0;d<32;++d) if (d !=16) ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d),0x8877665544332211ull + d);
    for (u32 i=0;i<sizeof(response);++i) ZLB_EXPECT_EQ(f.bus.read8(kDataBase+3u+i),response[i]);
}

ZLB_TEST(neon_vld1_elements_endian_lists_and_postindex) {
    constexpr u32 types[] = {7u,10u,6u,2u};
    for (bool thumb : {false,true}) for (bool big : {false,true}) {
        for (u32 size=0;size<4u;++size) for (u32 count=1;count<=4u;++count) {
            for (u32 m : {15u,13u,2u,0u}) {
                Fixture f; load_neon(f,arm_neon_vld1(32u-count,types[count-1],size,0,0,m),thumb);
                const u32 address=kDataBase+3u;
                f.set_reg(0,address); f.set_reg(2,37u); f.set_reg(13,kDataBase+0xF00u);
                if (big) f.cpu.cpsr |=arm::kFlagE;
                const u32 psr=f.cpu.cpsr;
                for (int d=0;d<32;++d) f.cpu.vfp.write_d32(d,0x8877665544332211ull+d);
                for (u32 byte=0;byte<8u*count;++byte) f.bus.write8(address+byte,static_cast<u8>(byte*29u+7u));
                ZLB_EXPECT_FALSE(f.cpu.step().faulted);
                const u32 width=1u<<size;
                for (u32 reg=0;reg<count;++reg) {
                    u64 expected=0;
                    for (u32 byte=0;byte<8u;++byte) {
                        const u32 source=(byte/width)*width+(big ? width-1u-byte%width : byte%width);
                        expected |=static_cast<u64>(static_cast<u8>((reg*8u+source)*29u+7u)) << (byte*8u);
                    }
                    ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(static_cast<int>(32u-count+reg)),expected);
                }
                for (u32 d=0;d<32u-count;++d) ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(static_cast<int>(d)),0x8877665544332211ull+d);
                const u32 increment=m==13u ? 8u*count : m==0u ? address : 37u;
                ZLB_EXPECT_EQ(f.reg(0),m==15u ? address : address+increment);
                ZLB_EXPECT_EQ(f.reg(13),kDataBase+0xF00u); ZLB_EXPECT_EQ(f.cpu.cpsr,psr);
            }
        }
    }
}

ZLB_TEST(neon_vld1_alignment_page_fault_and_it_suppression) {
    // For a 64-bit element, the ARM pseudocode reads the low result word
    // first: address+4 for BE, address for LE. Both pages are absent here.
    for (bool big : {false,true}) {
        Fixture f; load_neon(f,arm_neon_vld1(31,7,3,0,0,13),true);
        if (big) f.cpu.cpsr|=arm::kFlagE;
        ArmMmu& mmu=f.cpu.mmu; mmu.ttbr0=kDataBase; mmu.dacr=1u; mmu.sctlr|=1u;
        const u32 l2=kDataBase+0x8000u;
        f.bus.write32(kDataBase+0x800u*4u,0x80000C0Eu); f.bus.write32(kDataBase+0x810u*4u,l2|1u);
        f.set_reg(0,0x81000FFCu); f.cpu.vfp.write_d32(31,0x0123456789ABCDEFull);
        ZLB_EXPECT_TRUE(f.cpu.step().faulted); ZLB_EXPECT_EQ(mmu.dfar,big ? 0x81001000u : 0x81000FFCu);
        ZLB_EXPECT_EQ(f.reg(0),0x81000FFCu); ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(31),0x0123456789ABCDEFull);
    }
    for (bool fault : {false,true}) {
        Fixture f; load_neon(f,arm_neon_vld1(31,7,0,0,0,13),true);
        ArmMmu& mmu=f.cpu.mmu; mmu.ttbr0=kDataBase; mmu.dacr=1u; mmu.sctlr|=1u;
        const u32 l2=kDataBase+0x8000u;
        f.bus.write32(kDataBase+0x800u*4u,0x80000C0Eu); f.bus.write32(kDataBase+0x810u*4u,l2|1u);
        f.bus.write32(l2,0x8001A01Eu); f.bus.write32(l2+4u,fault ? 0u : 0x8001C01Eu);
        f.bus.write32(0x8001AFFCu,0x89ABCDEFu); f.bus.write32(0x8001B000u,0xA5A5A5A5u);
        f.bus.write32(0x8001C000u,0x01234567u); f.set_reg(0,0x81000FFCu);
        f.cpu.vfp.write_d32(31,0x8877665544332211ull);
        ZLB_EXPECT_EQ(f.cpu.step().faulted,fault);
        if (fault) {
            ZLB_EXPECT_EQ(f.reg(0),0x81000FFCu); ZLB_EXPECT_EQ(mmu.dfar,0x81001000u);
            ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(31),0x8877665589ABCDEFull);
        } else {
            ZLB_EXPECT_EQ(f.reg(0),0x81001004u); ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(31),0x0123456789ABCDEFull);
        }
        ZLB_EXPECT_EQ(f.bus.read32(0x8001B000u),0xA5A5A5A5u);
    }
    for (u32 size : {1u,2u,3u}) {
        Fixture f; load_neon(f,arm_neon_vld1(16,7,size,0,0,13),true);
        f.cpu.mmu.sctlr|=2u; f.set_reg(0,kDataBase+1u); f.cpu.vfp.write_d32(16,0xAABBCCDDEEFF0011ull);
        ZLB_EXPECT_TRUE(f.cpu.step().faulted); ZLB_EXPECT_EQ(f.cpu.mmu.dfsr,1u);
        ZLB_EXPECT_EQ(f.reg(0),kDataBase+1u); ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(16),0xAABBCCDDEEFF0011ull);
    }
    for (u32 cb : {0u,4u}) for (u32 size : {1u,2u,3u}) {
        Fixture f; load_neon(f,arm_neon_vld1(16,7,size,0,0,13),true);
        ArmMmu& mmu=f.cpu.mmu; mmu.ttbr0=kDataBase; mmu.dacr=1u; mmu.sctlr|=1u;
        const u32 l2=kDataBase+0x8000u;
        f.bus.write32(kDataBase+0x800u*4u,0x80000C0Eu); f.bus.write32(kDataBase+0x810u*4u,l2|1u);
        f.bus.write32(l2,0x8001A012u|cb); f.set_reg(0,0x81000001u);
        ZLB_EXPECT_TRUE(f.cpu.step().faulted); ZLB_EXPECT_EQ(f.reg(0),0x81000001u);
        ZLB_EXPECT_EQ(mmu.dfar,0x81000001u); ZLB_EXPECT_EQ(mmu.dfsr,1u);
    }
    for (u32 instr : {arm_neon_vld1(16,7,0,1,0,13),arm_neon_vld1(31,10,0,0,0,13),
                     arm_neon_vld1(16,7,0,2,0,13),arm_neon_vld1(16,7,0,0,15,13)}) {
        Fixture f; load_neon(f,instr,true); f.set_reg(0,kDataBase+1u);
        f.cpu.vfp.write_d32(16,0xAABBCCDDEEFF0011ull);
        ZLB_EXPECT_TRUE(f.cpu.step().faulted); ZLB_EXPECT_EQ(f.reg(0),kDataBase+1u);
        ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(16),0xAABBCCDDEEFF0011ull);
    }
    Fixture skipped; skipped.load_halfwords(kCodeBase,{0xBF08u,0xF962u,0x070Du});
    skipped.cpu.reset(kCodeBase|1u); skipped.cpu.cpsr &= ~arm::kFlagZ;
    skipped.set_reg(2,0xF0000000u); skipped.cpu.vfp.write_d32(16,0xAABBCCDDEEFF0011ull);
    ZLB_EXPECT_FALSE(skipped.cpu.step().faulted); ZLB_EXPECT_FALSE(skipped.cpu.step().faulted);
    ZLB_EXPECT_EQ(skipped.reg(2),0xF0000000u); ZLB_EXPECT_EQ(skipped.cpu.vfp.read_d32(16),0xAABBCCDDEEFF0011ull);
}

ZLB_TEST(neon_movl_signed_unsigned_widths_and_source_overlap) {
    struct Registers {u32 d,m;};
    for (bool thumb : {false,true}) for (bool uns : {false,true}) for (u32 width : {8u,16u,32u}) {
        for (const Registers registers : {Registers{16,16},Registers{16,17},Registers{30,31},Registers{0,31}}) {
            Fixture f; load_neon(f,arm_neon_movl(width,uns,registers.d,registers.m),thumb);
            u64 before[32],expected[32];
            for (u32 d=0;d<32;++d) before[d]=expected[d]=0x8034FE91A5FF7E00ull+d;
            for (int d=0;d<32;++d) f.cpu.vfp.write_d32(d,before[d]);
            expected[registers.d]=expected[registers.d+1]=0;
            const u64 source=before[registers.m];
            const u64 mask=(1ull<<width)-1u;
            for (u32 e=0;e<64u/width;++e) {
                const u64 element=(source>>(e*width))&mask;
                const s64 signed_element=(element&(1ull<<(width-1u))) ?
                    static_cast<s64>(element)-(1ll<<width) : static_cast<s64>(element);
                const u64 value=uns ? element : static_cast<u64>(signed_element);
                const u32 output_width=width*2u,offset=e*output_width;
                const u64 output_mask=output_width==64u ? ~0ull : (1ull<<output_width)-1u;
                expected[registers.d+offset/64u] |= (value&output_mask)<<(offset%64u);
            }
            f.set_reg(13,kDataBase+0xF00u); f.cpu.cpsr|=0xA0000000u;
            const u32 psr=f.cpu.cpsr; f.cpu.vfp.fpscr=0xA1000015u;
            ZLB_EXPECT_FALSE(f.cpu.step().faulted);
            for (int d=0;d<32;++d) ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d),expected[d]);
            ZLB_EXPECT_EQ(f.reg(13),kDataBase+0xF00u); ZLB_EXPECT_EQ(f.cpu.cpsr,psr);
            ZLB_EXPECT_EQ(f.cpu.vfp.fpscr,0xA1000015u);
        }
    }
}

ZLB_TEST(neon_movl_rejects_invalid_quad_and_related_shifts) {
    for (bool thumb : {false,true}) {
        for (u32 instruction : {arm_neon_movl(8,true,17,16),arm_neon_movl(16,false,31,0),
                                arm_neon_movl(8,true,16,16)|0x10000u}) {
            Fixture f; load_neon(f,instruction,thumb);
            for (int d=0;d<32;++d) f.cpu.vfp.write_d32(d,0xAABBCCDDEEFF0000ull+d);
            ZLB_EXPECT_TRUE(f.cpu.step().faulted); ZLB_EXPECT_TRUE(f.cpu.undefined_instruction);
            for (int d=0;d<32;++d) ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d),0xAABBCCDDEEFF0000ull+d);
        }
    }
    Fixture skipped; skipped.load_halfwords(kCodeBase,{0xBF08u,0xFFC8u,0x0A30u});
    skipped.cpu.reset(kCodeBase|1u); skipped.cpu.cpsr &=~arm::kFlagZ;
    skipped.cpu.vfp.write_d32(16,0xAABBCCDDEEFF0011ull);
    ZLB_EXPECT_FALSE(skipped.cpu.step().faulted); ZLB_EXPECT_FALSE(skipped.cpu.step().faulted);
    ZLB_EXPECT_EQ(skipped.cpu.vfp.read_d32(16),0xAABBCCDDEEFF0011ull);
}

ZLB_TEST(neon_add_integer_lane_wrap_and_register_aliases) {
    struct Registers {bool quad;u32 d,n,m;};
    const Registers registers[]={{false,17,31,19},{false,31,31,17},{false,17,31,17},
                                 {true,30,16,30},{true,16,16,30}};
    for (bool thumb : {false,true}) for (u32 size=0;size<4u;++size) for (const Registers& regs : registers) {
        Fixture f; load_neon(f,arm_neon_add(size,regs.quad,regs.d,regs.n,regs.m),thumb);
        u64 before[32],expected[32];
        for (int d=0;d<32;++d) {
            before[d]=expected[d]=0xFEFF8000FF7FFFFFull+(d&1);
            f.cpu.vfp.write_d32(d,before[d]);
        }
        const u32 width=8u<<size;
        const u64 mask=width==64u ? ~0ull : (1ull<<width)-1u;
        for (u32 reg=0;reg<(regs.quad?2u:1u);++reg) {
            u64 result=0;
            for (u32 offset=0;offset<64u;offset+=width) {
                const u64 left=(before[regs.n+reg]>>offset)&mask;
                const u64 right=(before[regs.m+reg]>>offset)&mask;
                result |= ((left+right)&mask)<<offset;
            }
            expected[regs.d+reg]=result;
        }
        f.set_reg(13,kDataBase+0xF00u); f.cpu.cpsr|=0xA0000000u;
        const u32 psr=f.cpu.cpsr; f.cpu.vfp.fpscr=0xA1000015u;
        ZLB_EXPECT_FALSE(f.cpu.step().faulted);
        for (int d=0;d<32;++d) ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d),expected[d]);
        ZLB_EXPECT_EQ(f.reg(13),kDataBase+0xF00u); ZLB_EXPECT_EQ(f.cpu.cpsr,psr);
        ZLB_EXPECT_EQ(f.cpu.vfp.fpscr,0xA1000015u);
    }
}

ZLB_TEST(neon_add_rejects_odd_quad_and_preserves_other_families) {
    for (bool thumb : {false,true}) {
        for (u32 instruction : {arm_neon_add(2,true,17,16,30),arm_neon_add(2,true,16,17,30),
                                arm_neon_add(2,true,16,30,31),arm_neon_add(2,false,16,16,16)|0x01000000u}) {
            Fixture f; load_neon(f,instruction,thumb);
            for (int d=0;d<32;++d) f.cpu.vfp.write_d32(d,0xAABBCCDDEEFF0000ull+d);
            ZLB_EXPECT_TRUE(f.cpu.step().faulted); ZLB_EXPECT_TRUE(f.cpu.undefined_instruction);
            for (int d=0;d<32;++d) ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d),0xAABBCCDDEEFF0000ull+d);
        }
    }
    Fixture skipped; skipped.load_halfwords(kCodeBase,{0xBF08u,0xEF62u,0x28A4u});
    skipped.cpu.reset(kCodeBase|1u); skipped.cpu.cpsr &=~arm::kFlagZ;
    skipped.cpu.vfp.write_d32(18,0xAABBCCDDEEFF0011ull);
    ZLB_EXPECT_FALSE(skipped.cpu.step().faulted); ZLB_EXPECT_FALSE(skipped.cpu.step().faulted);
    ZLB_EXPECT_EQ(skipped.cpu.vfp.read_d32(18),0xAABBCCDDEEFF0011ull);
}

ZLB_TEST(neon_pairwise_add_integer_reduction_and_source_overlap) {
    struct Registers {u32 d,n,m;};
    const Registers registers[]={{24,24,24},{31,17,31},{17,17,31},{19,17,31}};
    for (bool thumb : {false,true}) for (u32 size=0;size<3u;++size) for (const Registers& regs : registers) {
        Fixture f; load_neon(f,arm_neon_padd(size,regs.d,regs.n,regs.m),thumb);
        u64 before[32];
        for (int d=0;d<32;++d) {
            before[d]=0xFF00807FFFFF0001ull+d; f.cpu.vfp.write_d32(d,before[d]);
        }
        const u32 width=8u<<size;
        const u64 mask=(1ull<<width)-1u;
        u64 expected=0;
        for (u32 lane=0;lane<64u/width;++lane) {
            const bool second=lane>=32u/width;
            const u32 pair=lane%(32u/width);
            const u64 source=before[second ? regs.m : regs.n];
            const u64 left=(source>>(pair*2u*width))&mask;
            const u64 right=(source>>((pair*2u+1u)*width))&mask;
            expected |= ((left+right)&mask)<<(lane*width);
        }
        f.set_reg(13,kDataBase+0xF00u); f.cpu.cpsr|=0xA0000000u;
        const u32 psr=f.cpu.cpsr; f.cpu.vfp.fpscr=0xA1000015u;
        ZLB_EXPECT_FALSE(f.cpu.step().faulted);
        for (u32 d=0;d<32;++d) ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d),d==regs.d ? expected : before[d]);
        ZLB_EXPECT_EQ(f.reg(13),kDataBase+0xF00u); ZLB_EXPECT_EQ(f.cpu.cpsr,psr);
        ZLB_EXPECT_EQ(f.cpu.vfp.fpscr,0xA1000015u);
    }
}

ZLB_TEST(neon_pairwise_add_rejects_invalid_size_and_quad) {
    for (bool thumb : {false,true}) for (u32 instruction : {
        arm_neon_padd(3,24,24,24),arm_neon_padd(2,24,24,24)|0x40u,
        arm_neon_padd(2,24,24,24)|0x01000000u}) {
        Fixture f; load_neon(f,instruction,thumb);
        f.cpu.vfp.write_d32(24,0x0123456789ABCDEFull);
        ZLB_EXPECT_TRUE(f.cpu.step().faulted); ZLB_EXPECT_TRUE(f.cpu.undefined_instruction);
        ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(24),0x0123456789ABCDEFull);
    }
    Fixture skipped; skipped.load_halfwords(kCodeBase,{0xBF08u,0xEF68u,0x8BB8u});
    skipped.cpu.reset(kCodeBase|1u); skipped.cpu.cpsr &=~arm::kFlagZ;
    skipped.cpu.vfp.write_d32(24,0x0123456789ABCDEFull);
    ZLB_EXPECT_FALSE(skipped.cpu.step().faulted); ZLB_EXPECT_FALSE(skipped.cpu.step().faulted);
    ZLB_EXPECT_EQ(skipped.cpu.vfp.read_d32(24),0x0123456789ABCDEFull);
}

ZLB_TEST(neon_scalar_word_high_d_read_write_preserves_neighbors) {
    for (bool thumb : {false,true}) for (bool upper : {false,true}) for (bool to_core : {false,true}) {
        for (u32 target : {0u,15u,16u,24u,31u}) {
            Fixture f; const u32 instruction=arm_vmov_scalar32(2,target,upper,to_core);
            if (thumb) f.load_halfwords(kCodeBase+2u,{static_cast<u16>(instruction>>16),static_cast<u16>(instruction)});
            // A32 instructions require word alignment; Thumb starts at +2.
            else f.load(kCodeBase,{instruction});
            f.cpu.reset(thumb ? (kCodeBase+2u)|1u : kCodeBase);
            f.cpu.vfp.fpexc=ArmVfp::kFpexcEn;
            u64 before[32];
            for (int d=0;d<32;++d) {before[d]=0xAABBCCDD10203000ull+d;f.cpu.vfp.write_d32(d,before[d]);}
            f.set_reg(2,0xFE234567u); f.set_reg(13,kDataBase+0xF00u);
            f.cpu.cpsr|=0xA0000000u; const u32 psr=f.cpu.cpsr; f.cpu.vfp.fpscr=0xA1000015u;
            ZLB_EXPECT_FALSE(f.cpu.step().faulted);
            const u32 shift=upper ? 32u : 0u;
            ZLB_EXPECT_EQ(f.reg(2),to_core ? static_cast<u32>(before[target]>>shift) : 0xFE234567u);
            for (u32 d=0;d<32;++d) {
                const u64 mask=0xFFFFFFFFull<<shift;
                const u64 expected=!to_core && d==target ?
                    (before[d]&~mask)|(0xFE234567ull<<shift) : before[d];
                ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d),expected);
            }
            ZLB_EXPECT_EQ(f.reg(13),kDataBase+0xF00u); ZLB_EXPECT_EQ(f.cpu.cpsr,psr);
            ZLB_EXPECT_EQ(f.cpu.vfp.fpscr,0xA1000015u);
            ZLB_EXPECT_EQ(f.cpu.get_pc(),kCodeBase+(thumb ? 6u : 4u));
        }
    }
}

ZLB_TEST(neon_scalar_word_rejects_invalid_core_register_and_other_sizes) {
    for (bool thumb : {false,true}) for (u32 instruction : {
        arm_vmov_scalar32(15,24,false,true),arm_vmov_scalar32(2,24,false,true)|0x00400000u,
        arm_vmov_scalar32(2,24,false,true)|0x20u}) {
        Fixture f;
        if (thumb) f.load_halfwords(kCodeBase,{static_cast<u16>(instruction>>16),static_cast<u16>(instruction)});
        else f.load(kCodeBase,{instruction});
        f.cpu.reset(kCodeBase|(thumb ? 1u : 0u)); f.cpu.vfp.fpexc=ArmVfp::kFpexcEn;
        f.cpu.vfp.write_d32(24,0x0123456789ABCDEFull); f.set_reg(2,0xF1234567u);
        ZLB_EXPECT_TRUE(f.cpu.step().faulted); ZLB_EXPECT_TRUE(f.cpu.undefined_instruction);
        ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(24),0x0123456789ABCDEFull); ZLB_EXPECT_EQ(f.reg(2),0xF1234567u);
    }
    Fixture invalid_sp; const u32 instruction=arm_vmov_scalar32(13,24,false,true);
    invalid_sp.load_halfwords(kCodeBase,{static_cast<u16>(instruction>>16),static_cast<u16>(instruction)});
    invalid_sp.cpu.reset(kCodeBase|1u); invalid_sp.cpu.vfp.fpexc=ArmVfp::kFpexcEn;
    invalid_sp.set_reg(13,kDataBase+0xF00u);
    ZLB_EXPECT_TRUE(invalid_sp.cpu.step().faulted); ZLB_EXPECT_EQ(invalid_sp.reg(13),kDataBase+0xF00u);
}

ZLB_TEST(neon_unchanged_native_syscon_checksum_matches_scalar_oracle) {
    // Original 1.04 Syscon PT_LOAD bytes +B26..DB4, an inline checksum block
    // (linked 81000B26, captured at 004F8B26). Relative branches are retained.
    // The short scalar initialization at +165C is its other basic block.
    // Success rejoins the callback at +DB4; failure branches to +372.
    constexpr char code[] =
        "bb78012b7ff622acd81c8045fff41eac981c4fead00e07284feace0340f28b85"
        "002b00f088853a4601218e450ef1ff3c62f90d070cf0030cc8ff300a60efb021"
        "61efb101d0ff322ad0ff300a62efb24163efb32162efa42860efb03163efa228"
        "61efb10160efa28840f2d780bcf1000f5ed0bcf1010f3cd0bcf1020f1cd062f9"
        "0d070221c8ff300a60efb02161efb101d0ff322ad0ff300a62efb24164efa888"
        "63efb32162efa82860efb03163efa22861efb10160efa28862f90d070131c8ff"
        "300a60efb02161efb101d0ff322ad0ff300a62efb24164efa88863efb32162ef"
        "a82860efb03163efa22861efb10160efa28862f90d0701318e45c8ff300a60ef"
        "b02161efb101d0ff322ad0ff300a62efb24164efa88863efb32162efa82860ef"
        "b03163efa22861efb10160efa28874d9944602f11808043120326cf90d478e45"
        "c8ff344a6cf90d2764efb46165efb541d0ff366ac8ff322ad0ff344a66efb691"
        "69efa88867efb76166efa88862efb26164efb491d0ff366a69efa8886cf90f47"
        "65efa88863efb32166efb691c8ff344ad0ff322a69efa88867efb76166efa888"
        "64efb46162efb291d0ff366a69efa88863efb32168f90f0762efa82865efb541"
        "66efb631c8ff300ad0ff344a63efa22867efb76166efa26860efb02164efb471"
        "d0ff322a67efa66865efb54164efa64861efb10162efb251d0ff300a65efa448"
        "63efb32162efa42860efb03163efa22861efb10160efa2888ad868efb88b8342"
        "18ee902b2cd0d94317f803e001330918984201f00301724422d971b1012907d0"
        "022902d0f95c01335218f95c01335218f95c013398420a4412d9591c17f803a0"
        "17f801c0991c03f1030817f801e0524417f8081004336244984272440a44ecd8"
        "3b5cd04300f0ff079f427ff4dfaa";
    const auto nibble=[](char ch) -> u8 {return static_cast<u8>(ch>='a' ? ch-'a'+10 : ch-'0');};
    const auto load_code=[&](Fixture& f) {
        for (u32 i=0;i<(sizeof(code)-1u)/2u;++i)
            f.bus.write8(kCodeBase+0xB26u+i,static_cast<u8>((nibble(code[i*2u])<<4)|nibble(code[i*2u+1u])));
        f.load_halfwords(kCodeBase+0x165Cu,{0x2200u,0x4613u,0xF7FFu,0xBB74u});
        f.cpu.reset((kCodeBase+0xB26u)|1u); f.cpu.vfp.fpexc=ArmVfp::kFpexcEn;
        for (int d=0;d<32;++d) f.cpu.vfp.write_d32(d,0xAABBCCDDEEFF0000ull+d);
    };
    const auto run_block=[](Fixture& f) {
        unsigned steps=0;
        while (steps<2000u && !f.cpu.halted && f.cpu.get_pc()!=kCodeBase+0xDB4u &&
               f.cpu.get_pc()!=kCodeBase+0x372u) {f.cpu.step();++steps;}
        ZLB_EXPECT_FALSE(f.cpu.halted); ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
        ZLB_EXPECT_TRUE(steps<2000u);
    };
    // These lengths exercise the scalar path, every short vector unroll and
    // the long four-block loop, including 255-byte length and all alignments.
    for (u32 length : {2u,3u,4u,5u,6u,7u,8u,9u,14u,15u,22u,23u,30u,31u,38u,39u,62u,63u,126u,127u,254u,255u}) {
        for (u32 offset=0;offset<8u;++offset) for (bool valid : {false,true}) {
            Fixture f; load_code(f); const u32 source=kDataBase+offset;
            std::vector<u8> packet(length+3u);
            for (u32 byte=0;byte<length+2u;++byte) packet[byte]=static_cast<u8>(byte*73u+length*11u);
            packet[2]=static_cast<u8>(length);
            u32 sum=0; for (u32 byte=0;byte<length+2u;++byte) sum+=packet[byte];
            const u8 checksum=static_cast<u8>(~sum);
            packet.back()=static_cast<u8>(checksum^(valid ? 0u : 0x80u));
            for (u32 byte=0;byte<packet.size();++byte) f.bus.write8(source+byte,packet[byte]);
            f.set_reg(7,source); f.set_reg(8,length+3u); f.set_reg(13,kDataBase+0xF00u);
            f.set_reg(4,0xC1234567u); f.set_reg(5,0xD2345678u); f.set_reg(6,0xE3456789u);
            f.cpu.vfp.fpscr=0xA1000015u; run_block(f);
            ZLB_EXPECT_EQ(f.cpu.get_pc(),kCodeBase+(valid ? 0xDB4u : 0x372u));
            ZLB_EXPECT_EQ(f.reg(2),sum); ZLB_EXPECT_EQ(f.reg(7),checksum);
            ZLB_EXPECT_EQ(f.reg(13),kDataBase+0xF00u);
            ZLB_EXPECT_EQ(f.reg(4),0xC1234567u); ZLB_EXPECT_EQ(f.reg(5),0xD2345678u); ZLB_EXPECT_EQ(f.reg(6),0xE3456789u);
            for (int d=0;d<32;++d) if (d<16 || d>25)
                ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d),0xAABBCCDDEEFF0000ull+d);
            ZLB_EXPECT_EQ(f.cpu.vfp.fpscr,0xA1000015u);
            for (u32 byte=0;byte<packet.size();++byte) ZLB_EXPECT_EQ(f.bus.read8(source+byte),packet[byte]);
        }
    }
    for (u32 length : {0u,1u,6u}) {
        Fixture f; load_code(f); f.bus.write8(kDataBase+2u,static_cast<u8>(length));
        f.set_reg(7,kDataBase); f.set_reg(8,length==6u ? 8u : 32u);
        run_block(f); ZLB_EXPECT_EQ(f.cpu.get_pc(),kCodeBase+0x372u);
        ZLB_EXPECT_EQ(f.reg(7),kDataBase); // rejected before checksum processing
        for (int d=0;d<32;++d) ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d),0xAABBCCDDEEFF0000ull+d);
    }
}

ZLB_TEST(neon_logical_exact_d_q_registers_and_bit_selection) {
    // Truth-table bit index is (old destination, N, M). This checks the
    // architectural bit definitions independently of the implementation's
    // algebra, including VBSL's old-destination mask and operand overlap.
    constexpr u32 truth_tables[] = {0x88u, 0x44u, 0xEEu, 0xDDu,
                                    0x66u, 0xCAu, 0xD8u, 0xE4u};
    constexpr const char* names[] = {"vand", "vbic", "vorr", "vorn",
                                    "veor", "vbsl", "vbit", "vbif"};
    struct Layout { bool quad; u32 d, n, m; };
    const Layout layouts[] = {
        {false, 1, 17, 31}, {false, 31, 5, 19},
        {false, 17, 17, 31}, {false, 31, 17, 31},
        {true, 30, 16, 2}, {true, 16, 16, 30}, {true, 30, 16, 30},
    };
    for (u32 op = 0; op < 8; ++op) {
        for (const Layout& layout : layouts) {
            Fixture f;
            f.load(kCodeBase, {arm_neon_logical(op, layout.quad, layout.d, layout.n, layout.m)});
            unsigned length = 0;
            const char* prefix = layout.quad ? "q" : "d";
            const std::string expected_disasm = std::string(names[op]) + " " + prefix +
                std::to_string(layout.quad ? layout.d / 2 : layout.d) + ", " + prefix +
                std::to_string(layout.quad ? layout.n / 2 : layout.n) + ", " + prefix +
                std::to_string(layout.quad ? layout.m / 2 : layout.m);
            ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase, false, length) == expected_disasm);
            ZLB_EXPECT_EQ(length, 4u);
            f.cpu.reset(kCodeBase);
            f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
            f.cpu.vfp.fpscr = 0xA1000015u;
            f.cpu.cpsr |= 0xA0000000u;
            const u32 cpsr_before = f.cpu.cpsr;
            std::vector<u64> before;
            for (int d = 0; d < 32; ++d) {
                before.push_back(0xC53AF096E18D274Bull * (d + 1));
                f.cpu.vfp.write_d32(d, before.back());
            }
            std::vector<u64> expected = before;
            for (u32 lane = 0; lane < (layout.quad ? 2u : 1u); ++lane) {
                u64 result = 0;
                for (u32 bit = 0; bit < 64; ++bit) {
                    const u32 index = static_cast<u32>(((before[layout.d + lane] >> bit) & 1u) << 2) |
                                      static_cast<u32>(((before[layout.n + lane] >> bit) & 1u) << 1) |
                                      static_cast<u32>((before[layout.m + lane] >> bit) & 1u);
                    result |= static_cast<u64>((truth_tables[op] >> index) & 1u) << bit;
                }
                expected[layout.d + lane] = result;
            }
            const StepResult result = f.cpu.step();
            ZLB_EXPECT_FALSE(result.faulted);
            if (result.faulted) continue;
            for (int d = 0; d < 32; ++d) ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d), expected[d]);
            ZLB_EXPECT_EQ(f.reg(15), kCodeBase + 4u);
            ZLB_EXPECT_EQ(f.cpu.cpsr, cpsr_before);
            ZLB_EXPECT_EQ(f.cpu.vfp.fpscr, 0xA1000015u);
        }
    }
}

ZLB_TEST(neon_logical_rejects_odd_q_fields_and_other_operation_encodings) {
    struct Layout { u32 d, n, m; };
    const Layout invalid_q[] = {{1, 16, 30}, {0, 17, 30}, {0, 16, 31}};
    for (u32 op = 0; op < 8; ++op) {
        for (const Layout& layout : invalid_q) {
            Fixture f;
            f.load(kCodeBase, {arm_neon_logical(op, true, layout.d, layout.n, layout.m)});
            f.cpu.reset(kCodeBase);
            f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
            for (int d = 0; d < 32; ++d) f.cpu.vfp.write_d32(d, 0x0123456789ABCDEFull + d);
            const StepResult result = f.cpu.step();
            ZLB_EXPECT_TRUE(result.faulted);
            ZLB_EXPECT_TRUE(f.cpu.undefined_instruction);
            for (int d = 0; d < 32; ++d) {
                ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d), 0x0123456789ABCDEFull + d);
            }
        }
    }
    // opc1=3 is a different operation family, not EOR/BSL/BIT/BIF. Until that
    // family is implemented it must fault without a spurious logical write.
    for (u32 size = 0; size < 4; ++size) {
        Fixture f;
        f.load(kCodeBase, {arm_neon_logical(size, false, 2, 4, 6) | 0x200u});
        f.cpu.reset(kCodeBase);
        f.cpu.vfp.fpexc = ArmVfp::kFpexcEn;
        for (int d = 0; d < 32; ++d) f.cpu.vfp.write_d32(d, 0x0123456789ABCDEFull + d);
        const StepResult result = f.cpu.step();
        ZLB_EXPECT_TRUE(result.faulted);
        ZLB_EXPECT_TRUE(f.cpu.undefined_instruction);
        for (int d = 0; d < 32; ++d) {
            ZLB_EXPECT_EQ(f.cpu.vfp.read_d32(d), 0x0123456789ABCDEFull + d);
        }
    }
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
    // LDR r0, [pc, #0x10] with PC = instruction + 8: 0x80000004 + 8 + 0x10.
    ZLB_EXPECT_TRUE(ldr.find("ldr r0, #0x8000001C") != std::string::npos);
    ZLB_EXPECT_TRUE(ldr.find("0x8000001C") != std::string::npos);
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
    // BL carries a PC-relative offset, so the instruction has to be emitted for
    // the address it is going to live at (the old test emitted it for kCodeBase
    // and then loaded it at +0x20, which shifts the decoded target by 0x20).
    emit_thumb32_bl(words, kCodeBase + 0x20u, kCodeBase + 0x100u);
    f.load(kCodeBase + 0x20, words);
    const std::string bl = arm_disassemble(f.bus, kCodeBase + 0x20, true, length);
    ZLB_EXPECT_EQ(length, 4u);
    ZLB_EXPECT_TRUE(bl.find("bl #0x80000100") != std::string::npos);
}

ZLB_TEST(disasm_a32_multiply_family) {
    // Byte-exact operand order for the whole multiply space (capstone 5.0.9 was
    // the reference for docs/CPU_ARM_AUDIT.md). The mul/mla/... table was
    // shifted by one opcode and the SMMUL family was read out of bits [21:20],
    // so SDIV printed as a multiply and every SMMUL printed as an SMMLA.
    Fixture f;
    struct Case {
        u32 word;
        const char* expect;
    };
    const Case cases[] = {
        {0xE0001293u, "mul r0, r3, r2"},
        {0xE0201293u, "mla r0, r3, r2, r1"},
        {0xE0401293u, "umaal r1, r0, r3, r2"},
        {0xE0601293u, "mls r0, r3, r2, r1"},
        {0xE0801293u, "umull r1, r0, r3, r2"},
        {0xE0A01293u, "umlal r1, r0, r3, r2"},
        {0xE0C01293u, "smull r1, r0, r3, r2"},
        {0xE0E01293u, "smlal r1, r0, r3, r2"},
        {0xE750F211u, "smmul r0, r1, r2"},
        {0xE750F231u, "smmulr r0, r1, r2"},
        {0xE7503211u, "smmla r0, r1, r2, r3"},
        {0xE7503231u, "smmlar r0, r1, r2, r3"},
        {0xE75032D1u, "smmls r0, r1, r2, r3"},
        {0xE75032F1u, "smmlsr r0, r1, r2, r3"},
        {0xE710F211u, "sdiv r0, r1, r2"},
        {0xE730F211u, "udiv r0, r1, r2"},
    };
    for (const Case& c : cases) {
        f.bus.write32(kCodeBase, c.word);
        unsigned length = 0;
        const std::string text = arm_disassemble(f.bus, kCodeBase, false, length);
        ZLB_EXPECT_EQ(length, 4u);
        ZLB_EXPECT_TRUE(text.find(c.expect) != std::string::npos);
    }
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
    ZLB_EXPECT_EQ(f.reg(1), 0xBDu);   // low nibble replaced with 0xD
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

ZLB_TEST(thumb32_native_deflate_header_uses_the_unaligned_postindex_address) {
    Fixture f;
    // Genuine inflater sequence at NSKBL 0x51024E9E/EA8/EBA. Reading +2
    // from this valid zlib stream must identify dynamic DEFLATE block type 2.
    f.load_halfwords(kCodeBase, {0xF858u, 0x9B02u, // LDR r9,[r8],#2
                                0xFA69u, 0xF306u, // ROR.W r3,r9,r6
                                0xF3C3u, 0x4381u}); // UBFX r3,r3,#18,#2
    f.bus.write32(kDataBase, 0x93CD9C78u);
    f.bus.write32(kDataBase + 4u, 0x5153485Fu);
    f.cpu.reset(kCodeBase | 1u);
    f.set_reg(8, kDataBase + 2u);
    f.set_reg(6, static_cast<u32>(-17));
    const u32 flags = arm::kFlagN | arm::kFlagC | arm::kFlagV;
    f.cpu.cpsr = (f.cpu.cpsr & ~0xF0000000u) | flags;
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(9), 0x485F93CDu);
    ZLB_EXPECT_EQ(f.reg(8), kDataBase + 4u);
    ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, flags);
    f.cpu.step();
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(3), 2u);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
}

ZLB_TEST(thumb32_unaligned_word_load_store_addressing_forms) {
    struct Case { u32 load; u32 store; u32 base; u32 address; u32 updated_base; };
    const Case cases[] = {
        {t32_t3(8, 9, 2, true), t32_t3(8, 9, 2, false),
         kDataBase, kDataBase + 2u, kDataBase},
        {t32_t2(8, 9, 12, 1, true), t32_t2(8, 9, 12, 1, false),
         kDataBase, kDataBase + 2u, kDataBase},
        {t32_t4(8, 9, 2, false, true, true, true), t32_t4(8, 9, 2, false, true, true),
         kDataBase + 2u, kDataBase + 2u, kDataBase + 4u},
        {t32_t4(8, 9, 2, true, false, true, true), t32_t4(8, 9, 2, true, false, true),
         kDataBase + 4u, kDataBase + 2u, kDataBase + 2u},
    };
    for (const Case& test : cases) {
        for (bool load : {false, true}) {
            Fixture f;
            f.load(kCodeBase, {load ? test.load : test.store});
            f.bus.write32(kDataBase, 0xA5A5A5A5u);
            f.bus.write32(kDataBase + 4u, 0xA5A5A5A5u);
            if (load) f.bus.write32(test.address, 0x11223344u);
            f.cpu.reset(kCodeBase | 1u);
            f.set_reg(8, test.base);
            f.set_reg(9, load ? 0xDEADBEEFu : 0x11223344u);
            f.set_reg(12, 1u);
            const StepResult result = f.cpu.step();
            ZLB_EXPECT_FALSE(result.faulted);
            ZLB_EXPECT_EQ(f.reg(8), test.updated_base);
            if (load) ZLB_EXPECT_EQ(f.reg(9), 0x11223344u);
            else {
                ZLB_EXPECT_EQ(f.bus.read32(test.address), 0x11223344u);
                ZLB_EXPECT_EQ(f.bus.read8(test.address - 1u), 0xA5u);
                ZLB_EXPECT_EQ(f.bus.read8(test.address + 4u), 0xA5u);
            }
            ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
        }
    }
    for (bool add : {false, true}) {
        Fixture f;
        const u32 pc = kCodeBase + 0x42u;
        const u32 base = (pc + 4u) & ~3u;
        const u32 address = add ? base + 0x21u : base - 0x21u;
        f.load(pc, {t32_load_store(add ? 0xF8DFu : 0xF85Fu, 0x9021u)});
        f.bus.write32(address, 0x11223344u);
        f.cpu.reset(pc | 1u);
        f.cpu.step();
        ZLB_EXPECT_EQ(f.reg(9), 0x11223344u);
        ZLB_EXPECT_EQ(f.cpu.get_pc(), pc + 4u);
    }
}

ZLB_TEST(arm_single_word_respects_data_endianness_in_all_instruction_sets) {
    for (bool big_endian : {false, true}) {
        for (u32 alignment = 0; alignment < 4; ++alignment) {
            for (unsigned isa = 0; isa < 3; ++isa) {
                Fixture f;
                if (isa == 0) f.load(kCodeBase, {arm_ldr_str_imm(14, true, false, 0, 3, 0),
                                               arm_ldr_str_imm(14, false, false, 0, 3, 8)});
                else if (isa == 1) f.load_halfwords(kCodeBase, {0x6803u, 0x6083u});
                else f.load(kCodeBase, {t32_t3(0, 3, 0, true), t32_t3(0, 3, 8, false)});
                f.bus.write32(kDataBase + alignment, 0x11223344u);
                f.cpu.reset(kCodeBase | (isa == 0 ? 0u : 1u));
                f.set_reg(0, kDataBase + alignment);
                if (big_endian) f.cpu.cpsr |= arm::kFlagE;
                f.cpu.step();
                ZLB_EXPECT_EQ(f.reg(3), big_endian ? 0x44332211u : 0x11223344u);
                f.cpu.step();
                ZLB_EXPECT_EQ(f.bus.read32(kDataBase + alignment + 8u), 0x11223344u);
                ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
            }
        }
    }
}

ZLB_TEST(arm_single_word_crosses_noncontiguous_normal_pages) {
    for (bool load : {false, true}) {
        for (unsigned isa = 0; isa < 3; ++isa) {
            Fixture f;
            if (isa == 0) f.load(kCodeBase, {arm_ldr_str_imm(14, load, false, 0, 3, 2, false)});
            else if (isa == 1) f.load_halfwords(kCodeBase, {load ? u16{0x6803} : u16{0x6003}});
            else f.load(kCodeBase, {t32_t4(0, 3, 2, false, true, true, load)});
            f.cpu.reset(kCodeBase | (isa == 0 ? 0u : 1u));
            ArmMmu& mmu = f.cpu.mmu;
            mmu.ttbr0 = kDataBase;
            mmu.dacr = 1u;
            mmu.sctlr |= 1u;
            const u32 l2 = kDataBase + 0x8000u;
            f.bus.write32(kDataBase + 0x800u * 4u, 0x80000C0Eu); // Normal code section
            f.bus.write32(kDataBase + 0x810u * 4u, l2 | 1u);
            f.bus.write32(l2, 0x8001A01Eu); // VA 81000000 -> PA 8001A000
            f.bus.write32(l2 + 4u, 0x8001C01Eu); // next VA -> nonadjacent PA
            const u32 address = 0x81000FFFu;
            f.bus.write8(0x8001AFFFu, 0x44u);
            f.bus.write32(0x8001B000u, 0xA5A5A5A5u); // adjacent physical page is a decoy
            f.bus.write32(0x8001C000u, load ? 0xA5112233u : 0xA5A5A5A5u);
            f.set_reg(0, address);
            f.set_reg(3, load ? 0u : 0x11223344u);
            const StepResult result = f.cpu.step();
            ZLB_EXPECT_FALSE(result.faulted);
            ZLB_EXPECT_EQ(f.reg(0), address + (isa == 1 ? 0u : 2u));
            if (load) ZLB_EXPECT_EQ(f.reg(3), 0x11223344u);
            else {
                ZLB_EXPECT_EQ(f.bus.read8(0x8001AFFFu), 0x44u);
                ZLB_EXPECT_EQ(f.bus.read32(0x8001C000u), 0xA5112233u);
            }
            ZLB_EXPECT_EQ(f.bus.read32(0x8001B000u), 0xA5A5A5A5u);
            ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
        }
    }
}

ZLB_TEST(arm_single_word_cross_page_fault_preserves_destination_and_writeback) {
    for (bool load : {false, true}) {
        for (unsigned isa = 0; isa < 3; ++isa) {
            Fixture f;
            if (isa == 0) f.load(kCodeBase, {arm_ldr_str_imm(14, load, false, 0, 3, 4, false)});
            else if (isa == 1) f.load_halfwords(kCodeBase, {load ? u16{0x6803} : u16{0x6003}});
            else f.load(kCodeBase, {t32_t4(0, 3, 4, false, true, true, load)});
            f.cpu.reset(kCodeBase | (isa == 0 ? 0u : 1u));
            ArmMmu& mmu = f.cpu.mmu;
            mmu.ttbr0 = kDataBase;
            mmu.dacr = 1u;
            mmu.sctlr |= 1u;
            const u32 l2 = kDataBase + 0x8000u;
            f.bus.write32(kDataBase + 0x800u * 4u, 0x80000C0Eu);
            f.bus.write32(kDataBase + 0x810u * 4u, l2 | 1u);
            f.bus.write32(l2, 0x8001A01Eu);
            f.bus.write32(l2 + 4u, 0u); // next virtual page must fault
            f.bus.write8(0x8001AFFFu, 0x77u);
            f.bus.write32(0x8001B000u, 0xA5A5A5A5u); // adjacent PA is not the next VA
            constexpr u32 address = 0x81000FFFu;
            f.set_reg(0, address);
            f.set_reg(3, load ? 0xDEADBEEFu : 0x11223344u);
            const StepResult result = f.cpu.step();
            ZLB_EXPECT_TRUE(result.faulted);
            ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeAbort);
            ZLB_EXPECT_EQ(mmu.dfar, address + 1u);
            ZLB_EXPECT_EQ(mmu.dfsr, load ? 7u : 0x807u);
            ZLB_EXPECT_EQ(f.reg(0), address);
            ZLB_EXPECT_EQ(f.reg(3), load ? 0xDEADBEEFu : 0x11223344u);
            ZLB_EXPECT_EQ(f.bus.read32(0x8001B000u), 0xA5A5A5A5u);
        }
    }
}

ZLB_TEST(arm_single_word_alignment_fault_preserves_writeback_and_memory) {
    // A=1 faults regardless of M. With M=1, Device/Strongly-ordered mappings
    // fault even when A=0. Neither the destination nor post-index base commits.
    struct Case { bool mmu_enabled; bool alignment_check; u32 data_cb; };
    const Case cases[] = {{false, true, 0xCu}, {true, true, 0xCu},
                          {true, false, 4u}, {true, false, 0u}};
    for (const Case& test : cases) {
        for (bool load : {false, true}) {
            for (unsigned isa = 0; isa < 3; ++isa) {
                Fixture f;
                if (isa == 0) f.load(kCodeBase, {arm_ldr_str_imm(14, load, false, 0, 3, 2, false)});
                else if (isa == 1) f.load_halfwords(kCodeBase, {load ? u16{0x6803} : u16{0x6003}});
                else f.load(kCodeBase, {t32_t4(0, 3, 2, false, true, true, load)});
                f.bus.write32(0x8001A000u, 0x93CD9C78u);
                f.bus.write32(0x8001A004u, 0x5153485Fu);
                f.cpu.reset(kCodeBase | (isa == 0 ? 0u : 1u));
                const u32 address = test.mmu_enabled ? 0x81000002u : 0x8001A002u;
                ArmMmu& mmu = f.cpu.mmu;
                if (test.mmu_enabled) {
                    mmu.ttbr0 = kDataBase;
                    mmu.dacr = 1u;
                    const u32 l2 = kDataBase + 0x8000u;
                    f.bus.write32(kDataBase + 0x800u * 4u, 0x80000C0Eu);
                    f.bus.write32(kDataBase + 0x810u * 4u, l2 | 1u);
                    f.bus.write32(l2, 0x8001A012u | test.data_cb);
                    mmu.sctlr |= 1u;
                }
                if (test.alignment_check) mmu.sctlr |= 2u;
                f.set_reg(0, address);
                f.set_reg(3, 0xDEADBEEFu);
                const StepResult result = f.cpu.step();
                ZLB_EXPECT_TRUE(result.faulted);
                ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeAbort);
                ZLB_EXPECT_EQ(mmu.dfar, address);
                ZLB_EXPECT_EQ(mmu.dfsr, load ? 1u : 0x801u);
                ZLB_EXPECT_EQ(f.reg(0), address);
                ZLB_EXPECT_EQ(f.reg(3), 0xDEADBEEFu);
                ZLB_EXPECT_EQ(f.bus.read32(0x8001A000u), 0x93CD9C78u);
                ZLB_EXPECT_EQ(f.bus.read32(0x8001A004u), 0x5153485Fu);
            }
        }
    }
}

ZLB_TEST(thumb32_ldr_pc_dispatches_the_real_skbl_relocation_case) {
    Fixture f;
    f.bus.add_ram("skbl", 0x10000u, 0x40020000u, "test");
    // Genuine SKBL jump-table load at 0x4002474E. Treating Rt == PC as PLD
    // falls through into table data instead of reaching the MOVW relocation.
    f.load_halfwords(0x4002474Eu, {0xF853u, 0xF02Cu, 0x2099u});
    f.bus.write32(0x40024810u, 0x4002489Du);
    f.load_halfwords(0x4002489Cu, {0x202Au});
    f.cpu.reset(0x4002474Fu);
    f.set_reg(3, 0x40024754u);
    f.set_reg(12, 0x2Fu);
    const u32 flags = arm::kFlagZ | arm::kFlagC | arm::kFlagV;
    f.cpu.cpsr = (f.cpu.cpsr & ~0xF0000000u) | flags;
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), 0x4002489Cu);
    ZLB_EXPECT_TRUE(f.cpu.thumb);
    ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, flags);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x2Au);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
}

ZLB_TEST(thumb32_word_loads_to_pc_interwork_and_preserve_writeback) {
    struct Case { u32 instruction; u32 source; u32 base; u32 address; u32 updated_base; };
    const Case cases[] = {
        {t32_t3(3, 15, 0x20u, true), kCodeBase + 2u,
         kDataBase, kDataBase + 0x20u, kDataBase},
        {t32_t2(3, 15, 12, 2, true), kCodeBase + 2u,
         kDataBase, kDataBase + 0x20u, kDataBase},
        {t32_t4(3, 15, 0x20u, true, false, false, true), kCodeBase + 2u,
         kDataBase + 0x40u, kDataBase + 0x20u, kDataBase + 0x40u},
        {t32_t4(3, 15, 0x20u, true, true, true, true), kCodeBase + 2u,
         kDataBase, kDataBase + 0x20u, kDataBase + 0x20u},
        {t32_t4(3, 15, 0x20u, true, false, true, true), kCodeBase + 2u,
         kDataBase + 0x40u, kDataBase + 0x20u, kDataBase + 0x20u},
        {t32_t4(3, 15, 0x20u, false, true, true, true), kCodeBase + 2u,
         kDataBase + 0x20u, kDataBase + 0x20u, kDataBase + 0x40u},
        {t32_t4(3, 15, 0x20u, false, false, true, true), kCodeBase + 2u,
         kDataBase + 0x20u, kDataBase + 0x20u, kDataBase},
        // Halfword-aligned instruction addresses prove literal PC alignment.
        {t32_load_store(0xF8DFu, 0xF020u), kCodeBase + 2u,
         kDataBase, kCodeBase + 0x24u, kDataBase},
        {t32_load_store(0xF85Fu, 0xF020u), kCodeBase + 0x42u,
         kDataBase, kCodeBase + 0x24u, kDataBase},
    };
    for (const Case& test : cases) {
        for (bool thumb_target : {false, true}) {
            Fixture f;
            f.load_halfwords(test.source, {
                static_cast<u16>(test.instruction),
                static_cast<u16>(test.instruction >> 16),
                0x2099u, // a fallthrough must not execute this MOVS r0,#0x99
            });
            const u32 target = kCodeBase + (thumb_target ? 0x202u : 0x300u);
            f.bus.write32(test.address, target | (thumb_target ? 1u : 0u));
            if (thumb_target) f.load_halfwords(target - 2u, {0x2099u, 0x202Au});
            else f.load(target, {arm_dp_imm(0xE, 0xD, false, 0, 0, 0x2Au)});
            f.cpu.reset(test.source | 1u);
            f.set_reg(3, test.base);
            f.set_reg(12, 8u);
            const u32 flags = arm::kFlagZ | arm::kFlagC | arm::kFlagV;
            f.cpu.cpsr = (f.cpu.cpsr & ~0xF0000000u) | flags;
            f.cpu.step();
            ZLB_EXPECT_EQ(f.cpu.get_pc(), target);
            ZLB_EXPECT_EQ(f.cpu.thumb, thumb_target);
            ZLB_EXPECT_EQ(f.reg(3), test.updated_base);
            ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, flags);
            f.cpu.step();
            ZLB_EXPECT_EQ(f.reg(0), 0x2Au);
            ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
        }
    }
}

ZLB_TEST(thumb32_preload_hints_do_not_load_pc_or_fault_on_the_hint_address) {
    const u32 hints[] = {
        t32_load_store(0xF893u, 0xF020u), // PLD [r3,#0x20]
        t32_load_store(0xF813u, 0xF02Cu), // PLD [r3,r12,LSL#2]
        t32_load_store(0xF813u, 0xFC20u), // PLD [r3,#-0x20]
        t32_load_store(0xF8B3u, 0xF020u), // PLDW [r3,#0x20]
        t32_load_store(0xF89Fu, 0xF020u), // PLD literal, positive
        t32_load_store(0xF81Fu, 0xF020u), // PLD literal, negative
        t32_load_store(0xF993u, 0xF020u), // PLI [r3,#0x20]
        t32_load_store(0xF913u, 0xF02Cu), // PLI [r3,r12,LSL#2]
        t32_load_store(0xF913u, 0xFC20u), // PLI [r3,#-0x20]
        t32_load_store(0xF99Fu, 0xF020u), // PLI literal, positive
    };
    for (u32 hint : hints) {
        Fixture f;
        f.load(kCodeBase, {hint});
        f.cpu.reset(kCodeBase | 1u);
        // Only code is mapped. A data load through r3 would abort.
        f.cpu.mmu.sctlr = 1u;
        f.cpu.mmu.ttbr0 = kDataBase;
        f.cpu.mmu.dacr = 3u;
        f.bus.write32(kDataBase + 0x800u * 4u, 0x80000C02u);
        f.set_reg(0, 0x12345678u);
        f.set_reg(3, 0x30000000u);
        f.set_reg(12, 0x2Fu);
        const u32 flags = arm::kFlagZ | arm::kFlagC | arm::kFlagV;
        f.cpu.cpsr = (f.cpu.cpsr & ~0xF0000000u) | flags;
        f.cpu.step();
        ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 4u);
        ZLB_EXPECT_TRUE(f.cpu.thumb);
        ZLB_EXPECT_EQ(f.reg(0), 0x12345678u);
        ZLB_EXPECT_EQ(f.reg(3), 0x30000000u);
        ZLB_EXPECT_EQ(f.reg(12), 0x2Fu);
        ZLB_EXPECT_EQ(f.cpu.cpsr & 0xF0000000u, flags);
        ZLB_EXPECT_EQ(f.cpu.mmu.total_faults, 0u);
        ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    }
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

// ===========================================================================
// A32 data processing: shifter operands, the S bit and flag rules
// ===========================================================================

namespace {

// A32 opcodes (bits [24:21]).
constexpr u32 kOpAnd = 0x0u, kOpEor = 0x1u, kOpSub = 0x2u, kOpRsb = 0x3u, kOpAdd = 0x4u,
                kOpAdc = 0x5u, kOpSbc = 0x6u, kOpRsc = 0x7u, kOpTst = 0x8u, kOpTeq = 0x9u,
                kOpCmp = 0xAu, kOpCmn = 0xBu, kOpOrr = 0xCu, kOpMov = 0xDu, kOpBic = 0xEu,
                kOpMvn = 0xFu;

// Shift types of the A32 shifter operand.
constexpr u32 kLsl = 0u, kLsr = 1u, kAsr = 2u, kRor = 3u;

/// Data processing with a shifted register operand: bits [11:4] carry imm5:type.
constexpr u32 a32_shift(u32 cond, u32 opcode, bool s, u32 rn, u32 rd, u32 rm, u32 type, u32 amount) {
    return (cond << 28) | (opcode << 21) | (s ? (1u << 20) : 0u) | (rn << 16) | (rd << 12) |
           ((amount & 0x1Fu) << 7) | ((type & 3u) << 5) | rm;
}

/// Data processing with a *register controlled* shift (bit 4 selects the form,
/// bits [11:8] = Rs).
constexpr u32 a32_shift_rs(u32 cond, u32 opcode, bool s, u32 rn, u32 rd, u32 rm, u32 type, u32 rs) {
    return (cond << 28) | (opcode << 21) | (s ? (1u << 20) : 0u) | (rn << 16) | (rd << 12) |
           (rs << 8) | ((type & 3u) << 5) | (1u << 4) | rm;
}

/// MSR (immediate): cond 0011 0R10 mask 1111 rotate imm8.
constexpr u32 a32_msr_imm(u32 cond, bool spsr, u32 mask, u32 rot, u32 imm8) {
    return (cond << 28) | (1u << 25) | (1u << 24) | (spsr ? (1u << 22) : 0u) | (1u << 21) |
           (mask << 16) | (15u << 12) | ((rot & 0xFu) << 8) | (imm8 & 0xFFu);
}

constexpr u32 a32_mrs(u32 cond, bool spsr, u32 rd) {
    return (cond << 28) | (1u << 24) | (spsr ? (1u << 22) : 0u) | (0xFu << 16) | (rd << 12);
}

constexpr u32 a32_msr_reg(u32 cond, bool spsr, u32 mask, u32 rm) {
    return (cond << 28) | (1u << 24) | (spsr ? (1u << 22) : 0u) | (1u << 21) | (mask << 16) |
           (0xFu << 12) | rm;
}

constexpr u32 a32_clz(u32 cond, u32 rd, u32 rm) {
    return (cond << 28) | 0x016F0F10u | (rd << 12) | rm;
}

/// MUL: Rd = Rm * Rs, S sets N/Z only. Bits [15:12] are zero by definition.
constexpr u32 a32_mul(u32 cond, bool s, u32 rd, u32 rm, u32 rs) {
    return (cond << 28) | (s ? (1u << 20) : 0u) | (rd << 16) | (rs << 8) | 0x90u | rm;
}

/// MOVT Rd, #imm16 (MOVW is arm_movw() above).
constexpr u32 a32_movt(u32 cond, u32 rd, u32 imm16) {
    return (cond << 28) | (1u << 25) | (0xAu << 21) | ((imm16 >> 12) << 16) | (rd << 12) |
           (imm16 & 0xFFFu);
}

}  // namespace

ZLB_TEST(arm_dp_logical_ops) {
    Fixture f;
    // ands/eors/orrs/bics r3, r1, r2 and mvns r3, r2 - all S, all shall write N/Z
    // only (C comes from the shifter, V is untouched).
    f.load(kCodeBase, {
                          a32_shift(0xE, kOpAnd, true, 1, 3, 2, kLsl, 0),
                          a32_shift(0xE, kOpEor, true, 1, 3, 2, kLsl, 0),
                          a32_shift(0xE, kOpOrr, true, 1, 3, 2, kLsl, 0),
                          a32_shift(0xE, kOpBic, true, 1, 3, 2, kLsl, 0),
                          a32_shift(0xE, kOpMvn, true, 0, 3, 2, kLsl, 0),
                      });
    f.cpu.reset(kCodeBase);
    f.set_reg(1, 0xF0F0F0F0u);
    f.set_reg(2, 0x0FF00FF0u);

    f.cpu.step();  // and
    ZLB_EXPECT_EQ(f.reg(3), 0x00F000F0u);
    ZLB_EXPECT_FALSE(f.cpu.flag_n());
    ZLB_EXPECT_FALSE(f.cpu.flag_z());

    f.cpu.step();  // eor
    ZLB_EXPECT_EQ(f.reg(3), 0xFF00FF00u);
    ZLB_EXPECT_TRUE(f.cpu.flag_n());

    f.cpu.step();  // orr
    ZLB_EXPECT_EQ(f.reg(3), 0xFFF0FFF0u);
    ZLB_EXPECT_TRUE(f.cpu.flag_n());

    f.cpu.step();  // bic
    ZLB_EXPECT_EQ(f.reg(3), 0xF000F000u);
    ZLB_EXPECT_TRUE(f.cpu.flag_n());

    f.cpu.step();  // mvn
    ZLB_EXPECT_EQ(f.reg(3), 0xF00FF00Fu);
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    ZLB_EXPECT_FALSE(f.cpu.flag_z());
}

ZLB_TEST(arm_dp_adc_sbc_rsc_carry_in) {
    Fixture f;
    const auto run = [&](u32 instr, u32 rn_value, u32 rm_value, bool carry) {
        f.bus.write32(kCodeBase, instr);
        f.cpu.reset(kCodeBase);
        f.set_reg(1, rn_value);
        f.set_reg(2, rm_value);
        if (carry) f.cpu.cpsr |= arm::kFlagC;
        else f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);
        f.cpu.step();
    };

    // ADC adds the carry in: 0xFFFFFFFF + 0 + 0 = 0xFFFFFFFF with C = 0 ...
    run(a32_shift(0xE, kOpAdc, true, 1, 0, 2, kLsl, 0), 0xFFFFFFFFu, 0u, false);
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFFu);
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    ZLB_EXPECT_FALSE(f.cpu.flag_z());
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    // ... and + 1 carries out to 0 with Z set.
    run(a32_shift(0xE, kOpAdc, true, 1, 0, 2, kLsl, 0), 0xFFFFFFFFu, 0u, true);
    ZLB_EXPECT_EQ(f.reg(0), 0u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    ZLB_EXPECT_FALSE(f.cpu.flag_n());
    // ADC overflow: 0x7FFFFFFF + 0 + 1 = 0x80000000.
    run(a32_shift(0xE, kOpAdc, true, 1, 0, 2, kLsl, 0), 0x7FFFFFFFu, 0u, true);
    ZLB_EXPECT_EQ(f.reg(0), 0x80000000u);
    ZLB_EXPECT_TRUE(f.cpu.flag_v());
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_n());

    // SBC subtracts NOT(C): C = 1 means "no borrow".
    run(a32_shift(0xE, kOpSbc, true, 1, 0, 2, kLsl, 0), 1u, 1u, true);
    ZLB_EXPECT_EQ(f.reg(0), 0u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    run(a32_shift(0xE, kOpSbc, true, 1, 0, 2, kLsl, 0), 1u, 1u, false);
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFFu);
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    // SBC overflow: 0x80000000 - 1 - 0 = 0x7FFFFFFF.
    run(a32_shift(0xE, kOpSbc, true, 1, 0, 2, kLsl, 0), 0x80000000u, 1u, true);
    ZLB_EXPECT_EQ(f.reg(0), 0x7FFFFFFFu);
    ZLB_EXPECT_TRUE(f.cpu.flag_v());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_FALSE(f.cpu.flag_n());

    // RSC computes op2 - rn - NOT(C).
    run(a32_shift(0xE, kOpRsc, true, 1, 0, 2, kLsl, 0), 5u, 0u, true);
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFBu);
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    run(a32_shift(0xE, kOpRsc, true, 1, 0, 2, kLsl, 0), 5u, 0u, false);
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFAu);

    // RSB (no carry in) computes op2 - rn: 0 - 1 = -1.
    run(a32_shift(0xE, kOpRsb, true, 1, 0, 2, kLsl, 0), 1u, 0u, true);
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFFu);
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    ZLB_EXPECT_FALSE(f.cpu.flag_v());
}

ZLB_TEST(arm_dp_cmp_cmn_flags) {
    Fixture f;
    const auto run = [&](u32 instr, u32 rn_value, u32 rm_value) {
        f.bus.write32(kCodeBase, instr);
        f.cpu.reset(kCodeBase);
        f.set_reg(1, rn_value);
        f.set_reg(2, rm_value);
        f.cpu.step();
    };

    // CMP is a subtract that writes no register; CMP/CMN set all four flags
    // (unlike TST/TEQ, which only produce N/Z and the shifter carry).
    run(a32_shift(0xE, kOpCmp, true, 1, 0, 2, kLsl, 0), 0x80000000u, 1u);
    ZLB_EXPECT_EQ(f.reg(0), 0u);  // CMP must not write Rd
    ZLB_EXPECT_EQ(f.reg(2), 1u);
    ZLB_EXPECT_FALSE(f.cpu.flag_n());
    ZLB_EXPECT_TRUE(f.cpu.flag_v());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_FALSE(f.cpu.flag_z());

    run(a32_shift(0xE, kOpCmp, true, 1, 0, 2, kLsl, 0), 5u, 5u);
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_FALSE(f.cpu.flag_v());

    run(a32_shift(0xE, kOpCmp, true, 1, 0, 2, kLsl, 0), 0u, 1u);
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_n());

    // CMN: 0x7FFFFFFF + 1 overflows to 0x80000000.
    run(a32_shift(0xE, kOpCmn, true, 1, 0, 2, kLsl, 0), 0x7FFFFFFFu, 1u);
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    ZLB_EXPECT_TRUE(f.cpu.flag_v());
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    ZLB_EXPECT_FALSE(f.cpu.flag_z());

    // CMN of two values that carry out: 0xFFFFFFFF + 1 = 0.
    run(a32_shift(0xE, kOpCmn, true, 1, 0, 2, kLsl, 0), 0xFFFFFFFFu, 1u);
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_FALSE(f.cpu.flag_v());
}

ZLB_TEST(arm_dp_shift_lsl_immediate) {
    Fixture f;
    // `movs r0, r1, lsl #n`: LSL #0 leaves C alone, LSL #1..31 shifts bit
    // (32-n) into C, LSL #32 (register form) shifts bit 0 into C.
    const auto run = [&](u32 amount, u32 value, bool carry_in) {
        f.bus.write32(kCodeBase, a32_shift(0xE, kOpMov, true, 0, 0, 1, kLsl, amount));
        f.cpu.reset(kCodeBase);
        f.set_reg(1, value);
        if (carry_in) f.cpu.cpsr |= arm::kFlagC;
        else f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);
        f.cpu.step();
    };

    run(0, 0x80000001u, false);  // LSL #0 - result unchanged, C unchanged
    ZLB_EXPECT_EQ(f.reg(0), 0x80000001u);
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_n());

    run(1, 0x80000000u, false);
    ZLB_EXPECT_EQ(f.reg(0), 0u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_z());

    run(31, 0x00000003u, false);
    ZLB_EXPECT_EQ(f.reg(0), 0x80000000u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());  // bit 1 of the source
    ZLB_EXPECT_TRUE(f.cpu.flag_n());

    run(16, 0x00008000u, false);
    ZLB_EXPECT_EQ(f.reg(0), 0x80000000u);
    ZLB_EXPECT_FALSE(f.cpu.flag_c());  // bit 16 of 0x00008000 is clear

    // LSL by a register amount: 32 shifts bit 0 out into C, 33 shifts C to 0.
    const auto run_reg = [&](u32 amount, u32 value) {
        f.bus.write32(kCodeBase, a32_shift_rs(0xE, kOpMov, true, 0, 0, 1, kLsl, 2));
        f.cpu.reset(kCodeBase);
        f.set_reg(1, value);
        f.set_reg(2, amount);
        f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);
        f.cpu.step();
    };
    run_reg(32, 1u);
    ZLB_EXPECT_EQ(f.reg(0), 0u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    run_reg(33, 1u);
    ZLB_EXPECT_EQ(f.reg(0), 0u);
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    // Only the low byte of Rs is used: 0x104 means a shift of 4.
    run_reg(0x104u, 1u);
    ZLB_EXPECT_EQ(f.reg(0), 0x10u);
}

ZLB_TEST(arm_dp_shift_lsr_asr_immediate) {
    Fixture f;
    const auto run = [&](u32 opcode, u32 type, u32 amount, u32 value, bool carry_in) {
        f.bus.write32(kCodeBase, a32_shift(0xE, opcode, true, 0, 0, 1, type, amount));
        f.cpu.reset(kCodeBase);
        f.set_reg(1, value);
        if (carry_in) f.cpu.cpsr |= arm::kFlagC;
        else f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);
        f.cpu.step();
    };

    run(kOpMov, kLsr, 1, 0x00000003u, false);
    ZLB_EXPECT_EQ(f.reg(0), 1u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());

    // LSR #32: the imm5 field of 0 encodes 32, result 0 and C = bit 31.
    run(kOpMov, kLsr, 0, 0x80000000u, false);
    ZLB_EXPECT_EQ(f.reg(0), 0u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_z());

    run(kOpMov, kLsr, 4, 0x0000000Fu, false);
    ZLB_EXPECT_EQ(f.reg(0), 0u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());

    run(kOpMov, kAsr, 1, 0x80000001u, false);
    ZLB_EXPECT_EQ(f.reg(0), 0xC0000000u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_n());

    // ASR #32 sign fills and shifts bit 31 into C.
    run(kOpMov, kAsr, 0, 0x80000000u, false);
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFFu);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    run(kOpMov, kAsr, 0, 0x7FFFFFFFu, false);
    ZLB_EXPECT_EQ(f.reg(0), 0u);
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
}

ZLB_TEST(arm_dp_shift_ror_and_rrx) {
    Fixture f;
    const auto run = [&](u32 type, u32 amount, u32 value, bool carry_in) {
        f.bus.write32(kCodeBase, a32_shift(0xE, kOpMov, true, 0, 0, 1, type, amount));
        f.cpu.reset(kCodeBase);
        f.set_reg(1, value);
        if (carry_in) f.cpu.cpsr |= arm::kFlagC;
        else f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);
        f.cpu.step();
    };

    run(kRor, 1, 0x00000001u, false);
    ZLB_EXPECT_EQ(f.reg(0), 0x80000000u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());

    run(kRor, 4, 0x0000000Fu, false);
    ZLB_EXPECT_EQ(f.reg(0), 0xF0000000u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());

    // ROR #0 in the immediate form is RRX: a one bit rotate through carry.
    run(kRor, 0, 0x00000001u, true);
    ZLB_EXPECT_EQ(f.reg(0), 0x80000000u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    run(kRor, 0, 0x00000001u, false);
    ZLB_EXPECT_EQ(f.reg(0), 0u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());  // the bit that was shifted out
}

ZLB_TEST(arm_dp_shifter_carry_for_logical_ops_with_s) {
    Fixture f;
    // ARM ARM A8.8.2: with S set, the C flag comes from the shifter for
    // AND/EOR/ORR/MOV/BIC/MVN (and TST/TEQ), and from the adder for the
    // arithmetic opcodes.
    const auto run = [&](u32 opcode, u32 rn) {
        f.bus.write32(kCodeBase, a32_shift(0xE, opcode, true, rn, 3, 1, kLsl, 1));
        f.cpu.reset(kCodeBase);
        f.set_reg(1, 0x80000000u);  // LSL #1 shifts the top bit out -> C = 1
        f.set_reg(rn, 0xFFFFFFFFu);
        f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);
        f.cpu.step();
    };

    run(kOpMov, 0);
    ZLB_EXPECT_EQ(f.reg(3), 0u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_z());

    run(kOpMvn, 0);
    ZLB_EXPECT_EQ(f.reg(3), 0xFFFFFFFFu);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());

    run(kOpOrr, 4);  // orrs r3, r4, r1, lsl #1 : r4 = 0xFFFFFFFF
    ZLB_EXPECT_EQ(f.reg(3), 0xFFFFFFFFu);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());

    run(kOpBic, 4);  // bics r3, r4, r1, lsl #1 : r4 = 0xFFFFFFFF, r1 = 0x80000000
    ZLB_EXPECT_EQ(f.reg(3), 0xFFFFFFFFu);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());

    // TST/TEQ also take C from the shifter.
    f.bus.write32(kCodeBase, a32_shift(0xE, kOpTst, true, 4, 0, 1, kLsl, 1));
    f.cpu.reset(kCodeBase);
    f.set_reg(4, 0xFFFFFFFFu);
    f.set_reg(1, 0x80000000u);
    f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);
    f.cpu.step();
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_EQ(f.reg(0), 0u);  // TST writes nothing
}

ZLB_TEST(arm_dp_arithmetic_flags_use_the_adder_carry) {
    Fixture f;
    // For the arithmetic opcodes the shifter still shifts, but C must come from
    // the addition, not from the shifted-out bit.
    f.bus.write32(kCodeBase, a32_shift(0xE, kOpAdd, true, 1, 0, 1, kLsl, 1));
    f.cpu.reset(kCodeBase);
    f.set_reg(1, 0x80000000u);  // r1 lsl #1 = 0 with shifter carry 1
    f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x80000000u);  // 0x80000000 + 0
    ZLB_EXPECT_FALSE(f.cpu.flag_c());      // adder carry is 0
    ZLB_EXPECT_FALSE(f.cpu.flag_v());
    ZLB_EXPECT_TRUE(f.cpu.flag_n());

    // 0xFFFFFFFF + (1 lsr #1 = 0): the shifter carry is 1 but the sum does not
    // carry out either.
    f.bus.write32(kCodeBase, a32_shift(0xE, kOpAdd, true, 1, 0, 2, kLsr, 1));
    f.cpu.reset(kCodeBase);
    f.set_reg(1, 0xFFFFFFFFu);
    f.set_reg(2, 0x00000001u);
    f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFFu);
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_n());

    // SUBS with C = 0 must still report the no-borrow result of the subtraction:
    // 5 - 5 = 0 sets C.
    f.bus.write32(kCodeBase, a32_shift(0xE, kOpSub, true, 1, 0, 2, kLsl, 0));
    f.cpu.reset(kCodeBase);
    f.set_reg(1, 5u);
    f.set_reg(2, 5u);
    f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);
    f.cpu.step();
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
}

ZLB_TEST(arm_dp_immediate_rotated_operand) {
    Fixture f;
    // MOV with a rotated 8 bit immediate: 0xFF rotated right by 8 (rotate field
    // 4) is 0xFF000000, and for the logical opcodes the S bit then takes C from
    // the last bit rotated out (bit 31 of the expanded immediate).
    f.load(kCodeBase, {
                          arm_dp_imm(0xE, kOpMov, false, 0, 0, (4u << 8) | 0xFFu),
                          arm_dp_imm(0xE, kOpAdd, true, 0, 0, 1u),
                          arm_dp_imm(0xE, kOpSub, true, 0, 0, 1u),
                          arm_dp_imm(0xE, kOpMov, true, 0, 0, 0u),
                          arm_dp_imm(0xE, kOpMov, true, 1, 1, (4u << 8) | 0x0Fu),
                      });
    f.cpu.reset(kCodeBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0xFF000000u);
    f.cpu.step();  // adds r0, r0, #1 -> 0xFF000001, N set, no carry out
    ZLB_EXPECT_EQ(f.reg(0), 0xFF000001u);
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    f.cpu.step();  // subs r0, r0, #1 -> 0xFF000000, C = 1
    ZLB_EXPECT_EQ(f.reg(0), 0xFF000000u);
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    f.cpu.step();  // movs r0, #0 -> Z, rotate 0 leaves C as it was
    ZLB_EXPECT_EQ(f.reg(0), 0u);
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    f.cpu.step();  // movs r1, #0x0F000000 -> C = bit 31 of the expanded immediate
    ZLB_EXPECT_EQ(f.reg(1), 0x0F000000u);
    ZLB_EXPECT_FALSE(f.cpu.flag_c());
    ZLB_EXPECT_FALSE(f.cpu.flag_n());
}

ZLB_TEST(arm_dp_write_pc_with_s_is_an_exception_return) {
    Fixture f;
    // `movs pc, lr` restores CPSR from SPSR and branches to LR (ARM ARM
    // A8.8.102: if S == 1 and Rd == 15 the instruction is an exception return).
    f.load(kCodeBase, {a32_shift(0xE, kOpMov, true, 0, 15, 14, kLsl, 0)});
    f.cpu.reset(kCodeBase);
    f.cpu.set_spsr(arm::kModeIrq | arm::kFlagT | arm::kFlagZ);
    f.set_reg(14, kCodeBase + 0x40u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeIrq);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 0x40u);
    ZLB_EXPECT_TRUE(f.cpu.thumb);
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    ZLB_EXPECT_FALSE(f.cpu.flag_c());

    // Without S, writing the PC does not change the instruction set or CPSR.
    Fixture g;
    g.load(kCodeBase, {a32_shift(0xE, kOpAdd, false, 1, 15, 2, kLsl, 0)});
    g.cpu.reset(kCodeBase);
    g.set_reg(1, kCodeBase + 0x80u);
    g.set_reg(2, 4u);
    g.cpu.step();
    ZLB_EXPECT_EQ(g.cpu.get_pc(), kCodeBase + 0x84u);
    ZLB_EXPECT_EQ(g.cpu.mode(), arm::kModeSupervisor);
    ZLB_EXPECT_FALSE(g.cpu.thumb);
}

ZLB_TEST(arm_dp_write_pc_interworks) {
    // ARM ARM A8.8.1 (ADC/ADD/.../MOV): with Rd == 15 and S == 0 the write is
    // an ALUWritePC, which for A32 is exactly BXWritePC - bit 0 of the result
    // is the new T bit and is not part of the address. Forcing ARM state here
    // broke every computed jump into Thumb code (`add pc, ...` in a veneer).
    Fixture f;
    f.bus.write32(kCodeBase, a32_shift(0xE, kOpAdd, false, 1, 15, 2, kLsl, 0));  // add pc, r1, r2
    f.cpu.reset(kCodeBase);
    f.cpu.cpsr |= arm::kFlagN | arm::kFlagC;  // S == 0: the flags must survive
    f.set_reg(1, (kCodeBase + 0x200u) | 1u);
    f.set_reg(2, 0u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 0x200u);
    ZLB_EXPECT_TRUE(f.cpu.thumb);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagT) != 0);
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_FALSE(f.cpu.flag_z());

    // Bit 0 clear -> ARM state and the bit is stripped from the address.
    f.cpu.reset(kCodeBase);
    f.set_reg(1, kCodeBase + 0x300u);
    f.set_reg(2, 0u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 0x300u);
    ZLB_EXPECT_FALSE(f.cpu.thumb);
    ZLB_EXPECT_FALSE((f.cpu.cpsr & arm::kFlagT) != 0);

    // MOV pc, Rm is the same ALUWritePC case (no Rn operand).
    Fixture g;
    g.bus.write32(kCodeBase, a32_shift(0xE, kOpMov, false, 0, 15, 1, kLsl, 0));  // mov pc, r1
    g.cpu.reset(kCodeBase);
    g.set_reg(1, (kCodeBase + 0x400u) | 1u);
    g.cpu.step();
    ZLB_EXPECT_EQ(g.cpu.get_pc(), kCodeBase + 0x400u);
    ZLB_EXPECT_TRUE(g.cpu.thumb);
    ZLB_EXPECT_TRUE((g.cpu.cpsr & arm::kFlagT) != 0);
}

ZLB_TEST(arm_condition_codes_full_table) {
    // Every condition (except the unconditional space) against every NZCV
    // combination: B<cond> #8 is taken exactly when the ARM ARM A8.3 predicate
    // holds.
    Fixture f;
    for (u32 cond = 0; cond <= 0xEu; ++cond) {
        f.bus.write32(kCodeBase, arm_branch(cond, false, 2));
        for (u32 flags = 0; flags < 16u; ++flags) {
            f.cpu.reset(kCodeBase);
            f.cpu.cpsr = (f.cpu.cpsr & 0x0FFFFFFFu) | (flags << 28);
            f.cpu.step();
            const bool n = (flags & 8u) != 0;
            const bool z = (flags & 4u) != 0;
            const bool c = (flags & 2u) != 0;
            const bool v = (flags & 1u) != 0;
            bool taken = false;
            switch (cond) {
                case 0x0u: taken = z; break;
                case 0x1u: taken = !z; break;
                case 0x2u: taken = c; break;
                case 0x3u: taken = !c; break;
                case 0x4u: taken = n; break;
                case 0x5u: taken = !n; break;
                case 0x6u: taken = v; break;
                case 0x7u: taken = !v; break;
                case 0x8u: taken = c && !z; break;
                case 0x9u: taken = !c || z; break;
                case 0xAu: taken = (n == v); break;
                case 0xBu: taken = (n != v); break;
                case 0xCu: taken = !z && (n == v); break;
                case 0xDu: taken = z || (n != v); break;
                default: taken = true; break;  // AL
            }
            ZLB_EXPECT_EQ(f.cpu.get_pc(), taken ? kCodeBase + 16u : kCodeBase + 4u);
        }
    }
}

ZLB_TEST(arm_psr_transfer_mrs) {
    Fixture f;
    f.load(kCodeBase, {a32_mrs(0xE, false, 0), a32_mrs(0xE, true, 1)});
    f.cpu.reset(kCodeBase);
    f.cpu.cpsr |= arm::kFlagN;
    f.cpu.set_spsr(0xDEADBEEFu);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), f.cpu.cpsr);
    ZLB_EXPECT_TRUE((f.reg(0) & arm::kFlagN) != 0);
    ZLB_EXPECT_EQ(f.reg(0) & arm::kModeMask, arm::kModeSupervisor);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 0xDEADBEEFu);
}

ZLB_TEST(arm_psr_transfer_msr_register_fields) {
    Fixture f;
    // MSR writes only the fields selected by the mask: _f sets NZCV, _c the
    // control byte (including the mode), _fsxc everything.
    f.load(kCodeBase, {a32_msr_reg(0xE, false, 0x8u, 0), a32_msr_reg(0xE, true, 0x8u, 0),
                       a32_msr_reg(0xE, false, 0x1u, 0)});
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 0xF0000000u);
    f.cpu.step();
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_v());
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeSupervisor);  // control byte untouched

    f.set_reg(0, 0x00000000u);
    f.cpu.step();  // msr spsr_f, r0
    ZLB_EXPECT_EQ(f.cpu.spsr(), 0u);

    f.set_reg(0, static_cast<u32>(arm::kModeIrq));
    f.cpu.step();  // msr cpsr_c, r0 -> switch to IRQ mode
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeIrq);
    ZLB_EXPECT_TRUE(f.cpu.flag_n());  // the flags survived the field write
}

ZLB_TEST(arm_psr_transfer_msr_immediate) {
    Fixture f;
    // MSR (immediate): cond 0011 0R10 mask 1111 rotate imm8. 0xF0000000 is
    // 0xF0 rotated right by 8, i.e. rotate field 4 (the rotate field counts
    // 2-bit steps).
    f.load(kCodeBase, {a32_msr_imm(0xE, false, 0x8u, 4u, 0xF0u),
                       a32_msr_imm(0xE, true, 0x8u, 4u, 0xF0u)});
    f.cpu.reset(kCodeBase);
    f.cpu.step();
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
    ZLB_EXPECT_TRUE(f.cpu.flag_v());
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeSupervisor);
    f.cpu.step();  // msr spsr_f, #0xF0
    ZLB_EXPECT_EQ(f.cpu.spsr() & 0xF0000000u, 0xF0000000u);
}

ZLB_TEST(arm_movw_movt_semantics) {
    Fixture f;
    // MOVW zeroes the top half, MOVT replaces only the top half.
    f.load(kCodeBase, {arm_movw(0xE, 0, 0xFFFFu), a32_movt(0xE, 0, 0x1234u),
                       arm_movw(0xE, 1, 0x0001u), a32_movt(0xE, 1, 0xABCDu)});
    f.cpu.reset(kCodeBase);
    f.set_reg(1, 0xAAAA5555u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x0000FFFFu);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x1234FFFFu);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 0x00000001u);  // MOVW clears everything above bit 15
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 0xABCD0001u);
}

ZLB_TEST(arm_clz_instruction) {
    Fixture f;
    f.load(kCodeBase, {a32_clz(0xE, 0, 1), a32_clz(0xE, 2, 3), a32_clz(0xE, 4, 5)});
    f.cpu.reset(kCodeBase);
    f.set_reg(1, 0x00000001u);
    f.set_reg(3, 0x0000FFFFu);
    f.set_reg(5, 0u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 31u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 16u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(4), 32u);  // CLZ of zero is 32
}

ZLB_TEST(arm_mul_s_flag_updates_nz_only) {
    Fixture f;
    // MUL{S} sets N/Z from the 32 bit product and leaves C/V alone.
    f.bus.write32(kCodeBase, a32_mul(0xE, true, 2, 0, 1));
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 0xFFFFFFFFu);
    f.set_reg(1, 1u);
    f.cpu.cpsr |= arm::kFlagC | arm::kFlagV;
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 0xFFFFFFFFu);
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    ZLB_EXPECT_FALSE(f.cpu.flag_z());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());  // untouched by MUL
    ZLB_EXPECT_TRUE(f.cpu.flag_v());

    f.bus.write32(kCodeBase, a32_mul(0xE, true, 3, 4, 5));
    f.cpu.reset(kCodeBase);
    f.set_reg(4, 0u);
    f.set_reg(5, 0x12345678u);
    f.cpu.cpsr |= arm::kFlagC;
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(3), 0u);
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    ZLB_EXPECT_FALSE(f.cpu.flag_n());
    ZLB_EXPECT_TRUE(f.cpu.flag_c());
}

// ===========================================================================
// A32 multiply / multiply accumulate / divide
//
// Field layout of the multiply space (ARM ARM A8.8.72 etc., verified with
// capstone): [19:16] = Rd (RdHi for the long forms), [15:12] = Ra (RdLo),
// [11:8] = Rm, [3:0] = Rn, bits [7:4] = 1001, bits [24:21] = opcode with
// 0 MUL, 1 MLA, 2 UMAAL, 3 MLS, 4 UMULL, 5 UMLAL, 6 SMULL, 7 SMLAL.
// ===========================================================================

namespace {

constexpr u32 a32_mla(u32 cond, bool s, u32 rd, u32 rn, u32 rm, u32 ra) {
    return (cond << 28) | (1u << 21) | (s ? (1u << 20) : 0u) | (rd << 16) | (ra << 12) |
           (rm << 8) | 0x90u | rn;
}

constexpr u32 a32_mls(u32 cond, u32 rd, u32 rn, u32 rm, u32 ra) {
    return (cond << 28) | (3u << 21) | (rd << 16) | (ra << 12) | (rm << 8) | 0x90u | rn;
}

constexpr u32 a32_umaal(u32 cond, u32 rdlo, u32 rdhi, u32 rn, u32 rm) {
    return (cond << 28) | (2u << 21) | (rdhi << 16) | (rdlo << 12) | (rm << 8) | 0x90u | rn;
}

/// op: 4 UMULL, 5 UMLAL, 6 SMULL, 7 SMLAL.
constexpr u32 a32_long_mul(u32 cond, u32 op, bool s, u32 rdlo, u32 rdhi, u32 rn, u32 rm) {
    return (cond << 28) | (op << 21) | (s ? (1u << 20) : 0u) | (rdhi << 16) | (rdlo << 12) |
           (rm << 8) | 0x90u | rn;
}

/// SDIV/UDIV: cond 0111 0001 / 0111 0011 Rd 1111 Rm 0001 Rn.
constexpr u32 a32_divide(u32 cond, bool udiv, u32 rd, u32 rn, u32 rm) {
    return (cond << 28) | ((udiv ? 0x73u : 0x71u) << 20) | (rd << 16) | (0xFu << 12) | (rm << 8) |
           (1u << 4) | rn;
}

/// SMMUL/SMMULR/SMMLA/SMMLAR/SMMLS/SMMLSR: cond 0111 0101 Rd Ra Rm nibble Rn
/// with nibble = 0001 (SMMLA), 0011 (SMMLAR), 1101 (SMMLS), 1111 (SMMLSR);
/// Ra == 1111 in the 0001/0011 forms is the no-accumulate SMMUL/SMMULR.
constexpr u32 a32_smmul(u32 rd, u32 ra, u32 rn, u32 rm, u32 nibble) {
    return 0xE7500000u | (rd << 16) | (ra << 12) | (rm << 8) | ((nibble & 0xFu) << 4) | rn;
}

}  // namespace

ZLB_TEST(arm_multiply_mla_and_mls) {
    Fixture f;
    // mul r2, r0, r1 ; mla r3, r0, r1, r4 ; mls r5, r0, r1, r4
    f.load(kCodeBase, {a32_mul(0xE, false, 2, 0, 1), a32_mla(0xE, false, 3, 0, 1, 4),
                       a32_mls(0xE, 5, 0, 1, 4)});
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 6u);
    f.set_reg(1, 7u);
    f.set_reg(4, 100u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 42u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(3), 142u);  // 6*7 + 100
    f.cpu.step();
    // Round 226: MLS is Ra - Rn x Rm, so 100 - 42 = 58.  This expectation used to be
    // 0xFFFFFFC6 (42 - 100), i.e. it had been written from the implementation, which
    // had the operands the other way round; Unicorn gives 0x3A for the same encoding.
    ZLB_EXPECT_EQ(f.reg(5), 0x3Au);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);

    // MLAS sets N/Z from the 32 bit result.
    f.bus.write32(kCodeBase, a32_mla(0xE, true, 2, 0, 1, 4));
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 0x40000000u);
    f.set_reg(1, 4u);      // product 0x00000000 (low 32 bits)
    f.set_reg(4, 0u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 0x00000000u);
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
}

ZLB_TEST(arm_multiply_long_forms) {
    Fixture f;
    // UMULL r0, r1, r2, r3 : RdLo = r0, RdHi = r1.
    f.bus.write32(kCodeBase, a32_long_mul(0xE, 4, false, 0, 1, 2, 3));
    f.cpu.reset(kCodeBase);
    f.set_reg(2, 0xFFFFFFFFu);
    f.set_reg(3, 2u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFEu);
    ZLB_EXPECT_EQ(f.reg(1), 0x00000001u);

    // UMLAL accumulates into RdHi:RdLo.
    f.bus.write32(kCodeBase, a32_long_mul(0xE, 5, false, 0, 1, 2, 3));
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 1u);
    f.set_reg(1, 1u);
    f.set_reg(2, 0xFFFFFFFFu);
    f.set_reg(3, 2u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFFu);  // 0x00000001_00000001 + 0x1_FFFFFFFE
    ZLB_EXPECT_EQ(f.reg(1), 0x00000002u);

    // SMULL is signed: -2 * 3 = -6 = 0xFFFFFFFF_FFFFFFFA.
    f.bus.write32(kCodeBase, a32_long_mul(0xE, 6, false, 0, 1, 2, 3));
    f.cpu.reset(kCodeBase);
    f.set_reg(2, 0xFFFFFFFEu);
    f.set_reg(3, 3u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFAu);
    ZLB_EXPECT_EQ(f.reg(1), 0xFFFFFFFFu);

    // SMLAL: -(1<<20) * 4 + 8 = -4194296.
    f.bus.write32(kCodeBase, a32_long_mul(0xE, 7, false, 0, 1, 2, 3));
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 8u);
    f.set_reg(1, 0u);
    f.set_reg(2, 0xFFF00000u);  // -1048576
    f.set_reg(3, 4u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0xFFC00008u);  // 8 - 4194304
    ZLB_EXPECT_EQ(f.reg(1), 0xFFFFFFFFu);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
}

ZLB_TEST(arm_multiply_long_sets_nz_from_the_64_bit_result) {
    Fixture f;
    // UMULLS: N comes from bit 63 (RdHi bit 31), Z from the whole 64 bit value.
    f.bus.write32(kCodeBase, a32_long_mul(0xE, 4, true, 0, 1, 2, 3));
    f.cpu.reset(kCodeBase);
    f.set_reg(2, 0xFFFFFFFFu);
    f.set_reg(3, 0xFFFFFFFFu);  // 0xFFFFFFFE_00000001 -> N set, Z clear
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x00000001u);
    ZLB_EXPECT_EQ(f.reg(1), 0xFFFFFFFEu);
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    ZLB_EXPECT_FALSE(f.cpu.flag_z());

    // A zero product sets Z (and clears N).
    f.bus.write32(kCodeBase, a32_long_mul(0xE, 4, true, 0, 1, 2, 3));
    f.cpu.reset(kCodeBase);
    f.set_reg(2, 0u);
    f.set_reg(3, 0x12345678u);
    f.cpu.step();
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
    ZLB_EXPECT_FALSE(f.cpu.flag_n());
}

ZLB_TEST(arm_multiply_umaal) {
    Fixture f;
    // UMAAL RdLo, RdHi, Rn, Rm: {RdHi:RdLo} += Rn * Rm (unsigned, no flags).
    f.bus.write32(kCodeBase, a32_umaal(0xE, 0, 1, 2, 3));
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 1u);
    f.set_reg(1, 2u);  // accumulator 0x00000002_00000001
    f.set_reg(2, 0xFFFFFFFFu);
    f.set_reg(3, 4u);
    f.cpu.cpsr |= arm::kFlagN | arm::kFlagZ;  // must survive: UMAAL has no S bit
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFDu);  // 0x2_00000001 + 0x3_FFFFFFFC
    ZLB_EXPECT_EQ(f.reg(1), 0x00000005u);
    ZLB_EXPECT_TRUE(f.cpu.flag_n());
    ZLB_EXPECT_TRUE(f.cpu.flag_z());
}

ZLB_TEST(arm_divide_sdiv_udiv) {
    Fixture f;
    const auto run = [&](u32 instr, u32 rn_value, u32 rm_value, u32& out) {
        f.bus.write32(kCodeBase, instr);
        f.cpu.reset(kCodeBase);
        f.set_reg(1, rn_value);
        f.set_reg(2, rm_value);
        f.cpu.step();
        out = f.reg(0);
    };
    u32 result = 0;

    run(a32_divide(0xE, false, 0, 1, 2), 20u, 3u, result);
    ZLB_EXPECT_EQ(result, 6u);
    // SDIV truncates towards zero: -7 / 2 = -3.
    run(a32_divide(0xE, false, 0, 1, 2), 0xFFFFFFF9u, 2u, result);
    ZLB_EXPECT_EQ(result, 0xFFFFFFFDu);
    // Division by zero yields 0.
    run(a32_divide(0xE, false, 0, 1, 2), 100u, 0u, result);
    ZLB_EXPECT_EQ(result, 0u);
    // INT32_MIN / -1 overflows to INT32_MIN.
    run(a32_divide(0xE, false, 0, 1, 2), 0x80000000u, 0xFFFFFFFFu, result);
    ZLB_EXPECT_EQ(result, 0x80000000u);
    // UDIV is unsigned: 0xFFFFFFFF / 2.
    run(a32_divide(0xE, true, 0, 1, 2), 0xFFFFFFFFu, 2u, result);
    ZLB_EXPECT_EQ(result, 0x7FFFFFFFu);
    run(a32_divide(0xE, true, 0, 1, 2), 7u, 0u, result);
    ZLB_EXPECT_EQ(result, 0u);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
}

ZLB_TEST(arm_media_smmul_smmla_smmls) {
    // SMMUL/SMMLA/SMMLS take the most significant word of the 64-bit product
    // and put the accumulate operand at bit 32 (ARM ARM A8.8.166-A8.8.169).
    Fixture f;
    const auto run = [&](u32 instr, u32 r1, u32 r2, u32 r3) {
        f.bus.write32(kCodeBase, instr);
        f.cpu.reset(kCodeBase);
        f.set_reg(0, 0xDEADBEEFu);
        f.set_reg(1, r1);
        f.set_reg(2, r2);
        f.set_reg(3, r3);
        f.cpu.step();
    };

    // SMMUL Rd, Rm, Rn: 0x00010000 * 0x00008000 = 2^31, top word 0.
    run(a32_smmul(0, 15, 1, 2, 0x1u), 0x00010000u, 0x00008000u, 0u);
    ZLB_EXPECT_EQ(f.reg(0), 0u);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    ZLB_EXPECT_EQ(f.reg(3), 0u);  // Ra == 1111: no register is accumulated

    // A negative product keeps its sign: -1 * 2 = -2 -> 0xFFFFFFFF.
    run(a32_smmul(0, 15, 1, 2, 0x1u), 0xFFFFFFFFu, 2u, 0u);
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFFFFu);

    // SMMULR adds 0x80000000 before the shift: 2^31 + 2^31 -> 1.
    run(a32_smmul(0, 15, 1, 2, 0x3u), 0x00010000u, 0x00008000u, 0u);
    ZLB_EXPECT_EQ(f.reg(0), 1u);

    // SMMLA Rd, Rm, Rn, Ra: (2 << 32) + 2^31 >> 32 = 2 - the accumulate sits
    // at bit 32, it is not added to the low word of the product.
    run(a32_smmul(0, 3, 1, 2, 0x1u), 0x00010000u, 0x00008000u, 2u);
    ZLB_EXPECT_EQ(f.reg(0), 2u);
    ZLB_EXPECT_EQ(f.reg(3), 2u);  // Ra is a source only

    // SMMLAR: (1 << 32) + 2^31 + 0x80000000 = 2 << 32 -> 2.
    run(a32_smmul(0, 3, 1, 2, 0x3u), 0x00010000u, 0x00008000u, 1u);
    ZLB_EXPECT_EQ(f.reg(0), 2u);

    // SMMLS subtracts the product from the shifted accumulator:
    // -2^31 * 4 = -2^33, so (3 << 32) - (-2^33) = 5 << 32 -> 5.
    run(a32_smmul(0, 3, 1, 2, 0xDu), 0x80000000u, 4u, 3u);
    ZLB_EXPECT_EQ(f.reg(0), 5u);

    // SMMLSR: (1 << 32) - 2^31 + 0x80000000 = 1 << 32 -> 1.
    run(a32_smmul(0, 3, 1, 2, 0xFu), 0x00010000u, 0x00008000u, 1u);
    ZLB_EXPECT_EQ(f.reg(0), 1u);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);

    // Rd is bits [19:16] and Ra is bits [15:12]; with them pointing at
    // different registers neither may be read from the other's field. 2 * 3 = 6
    // so the product's top word is 0 and only Ra can produce a non-zero result:
    // with Ra = r3 = 7, SMMLA r4, r1, r2, r3 must write r4 and leave r3 alone
    // (writing the Ra slot is exactly what the old decoder did).
    run(a32_smmul(4, 3, 1, 2, 0x1u), 2u, 3u, 0u);
    ZLB_EXPECT_EQ(f.reg(4), 0u);
    ZLB_EXPECT_EQ(f.reg(3), 0u);
    run(a32_smmul(4, 3, 1, 2, 0x1u), 2u, 3u, 7u);
    ZLB_EXPECT_EQ(f.reg(4), 7u);
    ZLB_EXPECT_EQ(f.reg(3), 7u);  // Ra is a source only
}

// ===========================================================================
// A32 load/store: addressing modes, byte/halfword/dual, unaligned behaviour
// ===========================================================================

namespace {

/// Register-offset form: cond 011 P U 0 W L Rn Rd imm5 type 0 Rm.
constexpr u32 a32_ldr_str_reg(u32 cond, bool load, bool byte, u32 rn, u32 rd, u32 rm, u32 type,
                              u32 amount, bool pre = true, bool up = true, bool wb = false) {
    return (cond << 28) | (3u << 25) | (pre ? (1u << 24) : 0u) | (up ? (1u << 23) : 0u) |
           (byte ? (1u << 22) : 0u) | (wb ? (1u << 21) : 0u) | (load ? (1u << 20) : 0u) |
           (rn << 16) | (rd << 12) | ((amount & 0x1Fu) << 7) | ((type & 3u) << 5) | rm;
}

/// Halfword / signed / doubleword form (ARM ARM A8.8.66 / A8.8.76 / A8.8.79).
/// bits [6:5] = S:H select LDRH (01), LDRSB (10), LDRSH (11), LDRD/STRD.
constexpr u32 a32_extra_ls(u32 cond, bool load, bool signed_op, bool half, bool immediate, u32 rn,
                           u32 rd, u32 offset, bool pre = true, bool up = true, bool wb = false) {
    return (cond << 28) | (pre ? (1u << 24) : 0u) | (up ? (1u << 23) : 0u) |
           (immediate ? (1u << 22) : 0u) | (wb ? (1u << 21) : 0u) | (load ? (1u << 20) : 0u) |
           (rn << 16) | (rd << 12) | (((offset >> 4) & 0xFu) << 8) | (1u << 7) |
           (signed_op ? (1u << 6) : 0u) | (half ? (1u << 5) : 0u) | (1u << 4) | (offset & 0xFu);
}

/// Block transfer with the S bit (user bank / exception return forms).
constexpr u32 a32_ldm_stm_s(u32 cond, bool load, bool pre, bool up, bool wb, bool s, u32 rn,
                            u32 list) {
    return (cond << 28) | (1u << 27) | (pre ? (1u << 24) : 0u) | (up ? (1u << 23) : 0u) |
           (s ? (1u << 22) : 0u) | (wb ? (1u << 21) : 0u) | (load ? (1u << 20) : 0u) | (rn << 16) |
           (list & 0xFFFFu);
}

/// BLX (immediate): 1111 101H imm24, offset = imm24:H:'0' relative to PC + 8.
constexpr u32 a32_blx_imm(u32 address, u32 target) {
    const u32 offset = target - (address + 8u);
    return 0xFA000000u | ((offset & 2u) << 23) | ((offset >> 2) & 0x00FFFFFFu);
}

constexpr u32 a32_bx(u32 rm) { return 0xE12FFF10u | rm; }
constexpr u32 a32_blx_reg(u32 rm) { return 0xE12FFF30u | rm; }

/// Exclusive access (ARM ARM A8.8.83 / A8.8.157): the byte and halfword forms
/// differ only in opcode bits [23:20].
constexpr u32 a32_ldrex(u32 rd, u32 rn) { return 0xE1900F9Fu | (rn << 16) | (rd << 12); }
constexpr u32 a32_strex(u32 rd, u32 rm, u32 rn) { return 0xE1800F90u | (rn << 16) | (rd << 12) | rm; }
constexpr u32 a32_ldrexb(u32 rd, u32 rn) { return 0xE1D00F9Fu | (rn << 16) | (rd << 12); }
constexpr u32 a32_strexb(u32 rd, u32 rm, u32 rn) { return 0xE1C00F90u | (rn << 16) | (rd << 12) | rm; }
constexpr u32 a32_ldrexh(u32 rd, u32 rn) { return 0xE1F00F9Fu | (rn << 16) | (rd << 12); }
constexpr u32 a32_strexh(u32 rd, u32 rm, u32 rn) { return 0xE1E00F90u | (rn << 16) | (rd << 12) | rm; }
/// LDREXD/STREXD (ARM ARM A8.8.67 / A8.8.158, layout confirmed against QEMU's
/// target/arm/a32.decode): the opcode carries a single register field - Rt for
/// LDREXD (bits [15:12]) and Rt for STREXD (bits [3:0]) - and the second register
/// is always Rt + 1.
constexpr u32 a32_ldrexd(u32 rd, u32 rn) { return 0xE1B00F9Fu | (rn << 16) | (rd << 12); }
constexpr u32 a32_strexd(u32 rd, u32 rt, u32 rn) { return 0xE1A00F90u | (rn << 16) | (rd << 12) | rt; }

}  // namespace

ZLB_TEST(arm_ldr_str_addressing_modes) {
    Fixture f;
    // Pre-indexed positive, pre-indexed negative, post-indexed and writeback.
    f.load(kCodeBase, {
                          arm_ldr_str_imm(0xE, false, false, 1, 0, 8),                     // str r0, [r1, #8]
                          arm_ldr_str_imm(0xE, true, false, 1, 2, 8, false, false, false), // ldr r2, [r1], #-8 (post-index: P=0, W=0)
                          arm_ldr_str_imm(0xE, true, false, 1, 3, 4, true, false, true),   // ldr r3, [r1, #-4]!
                          arm_ldr_str_imm(0xE, true, false, 1, 4, 16),                     // ldr r4, [r1, #16]
                      });
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 0xCAFEBABEu);
    f.set_reg(1, kDataBase + 0x40u);
    f.bus.write32(kDataBase + 0x48u, 0x11111111u);
    f.bus.write32(kDataBase + 0x34u, 0x22222222u);
    f.bus.write32(kDataBase + 0x40u, 0x33333333u);
    f.bus.write32(kDataBase + 0x50u, 0x44444444u);

    f.cpu.step();  // str r0, [r1, #8] - no writeback, base unchanged
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0x48u), 0xCAFEBABEu);
    ZLB_EXPECT_EQ(f.reg(1), kDataBase + 0x40u);

    f.cpu.step();  // ldr r2, [r1], #-8 : loads [r1], then r1 -= 8
    ZLB_EXPECT_EQ(f.reg(2), 0x33333333u);
    ZLB_EXPECT_EQ(f.reg(1), kDataBase + 0x38u);

    f.cpu.step();  // ldr r3, [r1, #-4]! : r1 -= 4 then loads [r1]
    ZLB_EXPECT_EQ(f.reg(3), 0x22222222u);
    ZLB_EXPECT_EQ(f.reg(1), kDataBase + 0x34u);

    f.cpu.step();  // ldr r4, [r1, #16]
    ZLB_EXPECT_EQ(f.reg(4), 0u);  // nothing was written there
    ZLB_EXPECT_EQ(f.reg(1), kDataBase + 0x34u);
}

ZLB_TEST(arm_ldr_str_register_offset) {
    Fixture f;
    // ldr r0, [r1, r2, lsl #2] ; str r3, [r1, r6, asr #1] ; ldrb r4, [r1, -r2]
    f.load(kCodeBase, {a32_ldr_str_reg(0xE, true, false, 1, 0, 2, kLsl, 2),
                       a32_ldr_str_reg(0xE, false, false, 1, 3, 6, kAsr, 1),
                       a32_ldr_str_reg(0xE, true, true, 1, 4, 2, kLsl, 0, true, false)});
    f.cpu.reset(kCodeBase);
    f.set_reg(1, kDataBase + 0x100u);
    f.set_reg(2, 4u);
    f.set_reg(6, 0x20u);
    f.set_reg(3, 0x99887766u);
    f.bus.write32(kDataBase + 0x110u, 0xDEADBEEFu);
    f.bus.write8(kDataBase + 0xFCu, 0x5Au);

    f.cpu.step();  // ldr r0, [r1, r2, lsl #2] -> [base + 0x10]
    ZLB_EXPECT_EQ(f.reg(0), 0xDEADBEEFu);

    f.cpu.step();  // str r3, [r1, r6, asr #1] -> 0x20 asr 1 = 0x10
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0x110u), 0x99887766u);

    f.cpu.step();  // ldrb r4, [r1, -r2] -> [base - 4]
    ZLB_EXPECT_EQ(f.reg(4), 0x5Au);
}

ZLB_TEST(arm_ldr_unaligned_reads_the_addressed_bytes) {
    Fixture f;
    // ARMv7 MemU with SCTLR.A=0 reads four bytes at the address. Legacy
    // pre-ARMv7 aligned-word rotation corrupts this value across the boundary.
    f.load(kCodeBase, {arm_ldr_str_imm(0xE, true, false, 0, 1, 0),
                       arm_ldr_str_imm(0xE, true, false, 0, 2, 1)});
    f.bus.write32(kDataBase, 0x11223344u);
    f.bus.write8(kDataBase + 4u, 0x55u);
    f.cpu.reset(kCodeBase);
    f.set_reg(0, kDataBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 0x11223344u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 0x55112233u);
}

ZLB_TEST(arm_str_unaligned_writes_the_addressed_bytes) {
    Fixture f;
    // ARMv7 STR uses MemU, preserving bytes on either side of the transfer.
    f.load(kCodeBase, {arm_ldr_str_imm(0xE, false, false, 0, 1, 0)});
    f.bus.write8(kDataBase, 0xAAu);
    f.bus.write8(kDataBase + 1, 0xBBu);
    f.bus.write8(kDataBase + 2, 0xCCu);
    f.bus.write8(kDataBase + 3, 0xDDu);
    f.bus.write8(kDataBase + 6, 0xEEu);
    f.cpu.reset(kCodeBase);
    f.set_reg(0, kDataBase + 2u);
    f.set_reg(1, 0x11223344u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 2u), 0x11223344u);
    ZLB_EXPECT_EQ(f.bus.read16(kDataBase), 0xBBAAu);
    ZLB_EXPECT_EQ(f.bus.read8(kDataBase + 6u), 0xEEu);
}

ZLB_TEST(arm_native_strncpy_preserves_os0_module_paths) {
    // Unchanged A32 helper at NSKBL 51013730. The live path normalizer calls
    // it with source 7FC53 containing /kd/sysmem.skprx. Legacy aligned+ROR
    // loads copied /os0 and skd/eysmkm.s, so native FAT lookup failed.
    const std::vector<u32> native_copy = {
        0xE1A0C000u, 0xE3520000u, 0x0A000028u, 0xE3520004u, 0xBA00000Bu,
        0xE4913004u, 0xE31300FFu, 0x0A000015u, 0xE3130CFFu, 0x0A000010u,
        0xE31308FFu, 0x0A00000Bu, 0xE31304FFu, 0xE48C3004u, 0xE2422004u,
        0x0A00000Fu, 0xEAFFFFEFu, 0xE4D13001u, 0xE4CC3001u, 0xE2522001u,
        0x0A000016u, 0xE3530000u, 0x0A00000Au, 0xEAFFFFF8u, 0xE4CC3001u,
        0xE2422001u, 0xE1A03463u, 0xE4CC3001u, 0xE2422001u, 0xE1A03463u,
        0xE4CC3001u, 0xE2422001u, 0xE3520000u, 0x0A000009u, 0xE3A03000u,
        0xE3520004u, 0xBA000003u, 0xE48C3004u, 0xE2522004u, 0x0A000003u,
        0xEAFFFFF8u, 0xE4CC3001u, 0xE2522001u, 0x1AFFFFFCu, 0xE12FFF1Eu,
    };
    constexpr char path[] = "/kd/sysmem.skprx";
    constexpr u32 capacity = 32u;
    for (u32 source_alignment = 0; source_alignment < 4; ++source_alignment) {
        for (u32 destination_alignment = 0; destination_alignment < 4; ++destination_alignment) {
            Fixture f;
            f.load(kCodeBase, native_copy);
            const u32 source = kDataBase + source_alignment;
            const u32 destination = kDataBase + 0x100u + destination_alignment;
            for (u32 byte = 0; byte < source_alignment; ++byte) {
                f.bus.write8(kDataBase + byte, static_cast<u8>("os0"[byte]));
            }
            for (u32 byte = 0; byte < sizeof(path); ++byte) f.bus.write8(source + byte, path[byte]);
            for (u32 byte = 0; byte < capacity + 2u; ++byte) {
                f.bus.write8(destination - 1u + byte, 0xA5u);
            }
            f.cpu.reset(kCodeBase);
            f.set_reg(0, destination);
            f.set_reg(1, source);
            f.set_reg(2, capacity);
            f.set_reg(14, kCodeBase + 0x100u);
            unsigned count = 0;
            while (f.cpu.get_pc() != kCodeBase + 0x100u && count++ < 500u) {
                const StepResult result = f.cpu.step();
                ZLB_EXPECT_FALSE(result.faulted);
                if (result.faulted) break;
            }
            ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 0x100u);
            ZLB_EXPECT_EQ(f.reg(0), destination);
            for (u32 byte = 0; byte < capacity; ++byte) {
                const u8 expected = byte < sizeof(path) ? static_cast<u8>(path[byte]) : 0u;
                ZLB_EXPECT_EQ(f.bus.read8(destination + byte), expected);
            }
            ZLB_EXPECT_EQ(f.bus.read8(destination - 1u), 0xA5u);
            ZLB_EXPECT_EQ(f.bus.read8(destination + capacity), 0xA5u);
            ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
        }
    }
}

ZLB_TEST(arm_unaligned_word_addressing_forms_preserve_actual_addresses) {
    struct Case { u32 load; u32 store; u32 base; u32 address; u32 updated_base; };
    const Case cases[] = {
        {arm_ldr_str_imm(14, true, false, 0, 3, 2), arm_ldr_str_imm(14, false, false, 0, 3, 2),
         kDataBase, kDataBase + 2u, kDataBase},
        {a32_ldr_str_reg(14, true, false, 0, 3, 2, 0, 1),
         a32_ldr_str_reg(14, false, false, 0, 3, 2, 0, 1),
         kDataBase, kDataBase + 2u, kDataBase},
        {arm_ldr_str_imm(14, true, false, 0, 3, 4, false),
         arm_ldr_str_imm(14, false, false, 0, 3, 4, false),
         kDataBase + 3u, kDataBase + 3u, kDataBase + 7u},
        {arm_ldr_str_imm(14, true, false, 0, 3, 2, true, false, true),
         arm_ldr_str_imm(14, false, false, 0, 3, 2, true, false, true),
         kDataBase + 3u, kDataBase + 1u, kDataBase + 1u},
    };
    for (const Case& test : cases) {
        for (const bool load : {false, true}) {
            Fixture f;
            f.load(kCodeBase, {load ? test.load : test.store});
            f.bus.write32(kDataBase, 0xA5A5A5A5u);
            f.bus.write32(kDataBase + 4u, 0xA5A5A5A5u);
            if (load) f.bus.write32(test.address, 0x11223344u);
            f.cpu.reset(kCodeBase);
            f.set_reg(0, test.base);
            f.set_reg(2, 1u);
            f.set_reg(3, load ? 0xDEADBEEFu : 0x11223344u);
            const StepResult result = f.cpu.step();
            ZLB_EXPECT_FALSE(result.faulted);
            ZLB_EXPECT_EQ(f.reg(0), test.updated_base);
            if (load) ZLB_EXPECT_EQ(f.reg(3), 0x11223344u);
            else {
                ZLB_EXPECT_EQ(f.bus.read32(test.address), 0x11223344u);
                ZLB_EXPECT_EQ(f.bus.read8(test.address - 1u), 0xA5u);
                ZLB_EXPECT_EQ(f.bus.read8(test.address + 4u), 0xA5u);
            }
        }
    }
}

ZLB_TEST(thumb16_single_word_register_immediate_and_sp_addressing) {
    struct Case { u16 load; u16 store; u32 base; u32 offset; u32 address; };
    const Case cases[] = {
        {0x5883u, 0x5083u, kDataBase, 1u, kDataBase + 1u}, // [r0,r2]
        {0x6843u, 0x6043u, kDataBase + 2u, 0u, kDataBase + 6u}, // [r0,#4]
        {0x9B01u, 0x9301u, kDataBase + 3u, 0u, kDataBase + 7u}, // [sp,#4]
    };
    for (const Case& test : cases) {
        for (const bool load : {false, true}) {
            Fixture f;
            f.load_halfwords(kCodeBase, {load ? test.load : test.store});
            for (u32 byte = 0; byte < 12u; ++byte) f.bus.write8(kDataBase + byte, 0xA5u);
            if (load) f.bus.write32(test.address, 0x11223344u);
            f.cpu.reset(kCodeBase | 1u);
            f.set_reg(0, test.base);
            f.set_reg(13, test.base);
            f.set_reg(2, test.offset);
            f.set_reg(3, load ? 0xDEADBEEFu : 0x11223344u);
            const StepResult result = f.cpu.step();
            ZLB_EXPECT_FALSE(result.faulted);
            if (load) ZLB_EXPECT_EQ(f.reg(3), 0x11223344u);
            else {
                ZLB_EXPECT_EQ(f.bus.read32(test.address), 0x11223344u);
                ZLB_EXPECT_EQ(f.bus.read8(test.address - 1u), 0xA5u);
                ZLB_EXPECT_EQ(f.bus.read8(test.address + 4u), 0xA5u);
            }
            ZLB_EXPECT_EQ(f.reg(0), test.base);
            ZLB_EXPECT_EQ(f.reg(13), test.base);
        }
    }
}

ZLB_TEST(arm_ldrb_strb) {
    Fixture f;
    f.load(kCodeBase, {arm_ldr_str_imm(0xE, true, true, 0, 1, 3),      // ldrb r1, [r0, #3]
                       arm_ldr_str_imm(0xE, true, true, 0, 2, 0),      // ldrb r2, [r0]
                       arm_ldr_str_imm(0xE, false, true, 0, 4, 1)});   // strb r4, [r0, #1]
    f.cpu.reset(kCodeBase);
    f.set_reg(0, kDataBase + 1u);  // deliberately unaligned
    f.set_reg(4, 0x12345678u);
    f.bus.write8(kDataBase + 1, 0x99u);
    f.bus.write8(kDataBase + 4, 0x80u);
    f.cpu.step();  // ldrb r1, [r0, #3] -> byte at kDataBase+4 = 0x80, zero extended
    ZLB_EXPECT_EQ(f.reg(1), 0x80u);
    f.cpu.step();  // ldrb r2, [r0] -> 0x99
    ZLB_EXPECT_EQ(f.reg(2), 0x99u);
    f.cpu.step();  // strb r4, [r0, #1] -> byte at kDataBase+2 = 0x78
    ZLB_EXPECT_EQ(f.bus.read8(kDataBase + 2), 0x78u);
    ZLB_EXPECT_EQ(f.bus.read8(kDataBase + 1), 0x99u);
}

ZLB_TEST(arm_ldm_with_pc_in_list_still_writes_back) {
    Fixture f;
    // ldmia sp!, {r4-r11, pc} - the encoding kernel_boot_loader's arzl decoder
    // returns with (0x4003C484).  ARM applies the base write-back even when r15 is
    // in the list; round 159: skipping it left the decoder's caller with sp 0x24
    // too low, and its `pop {…,pc}` then returned into garbage.
    f.load(kCodeBase, {0xE8BD8FF0u});
    f.cpu.reset(kCodeBase);
    const u32 sp = kDataBase + 0x40u;
    f.set_reg(13, sp);
    for (u32 i = 0; i < 8u; ++i) f.bus.write32(sp + i * 4u, 0x1000u + i);
    f.bus.write32(sp + 32u, kCodeBase + 0x100u);

    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(13), sp + 36u);          // the write-back happened
    ZLB_EXPECT_EQ(f.reg(4), 0x1000u);
    ZLB_EXPECT_EQ(f.reg(11), 0x1007u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 0x100u);
}

ZLB_TEST(arm_extra_load_store_halfword) {
    Fixture f;
    // strh r0, [r1, #4] ; ldrh r2, [r1], #2 ; ldrh r3, [r4, r5] ; strh r6, [r1, #-2]!
    f.load(kCodeBase, {a32_extra_ls(0xE, false, false, true, true, 1, 0, 4),
                       a32_extra_ls(0xE, true, false, true, true, 1, 2, 2, false, true, false),
                       a32_extra_ls(0xE, true, false, true, false, 4, 3, 5),
                       a32_extra_ls(0xE, false, false, true, true, 1, 6, 2, true, false, true)});
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 0xABCDEF01u);
    f.set_reg(1, kDataBase + 0x20u);
    f.set_reg(4, kDataBase + 0x40u);
    f.set_reg(5, 2u);
    f.set_reg(6, 0x5555u);
    f.bus.write16(kDataBase + 0x22u, 0x1234u);
    f.bus.write16(kDataBase + 0x42u, 0x8888u);

    f.cpu.step();
    ZLB_EXPECT_EQ(f.bus.read16(kDataBase + 0x24u), 0xEF01u);  // low half of r0
    ZLB_EXPECT_EQ(f.reg(1), kDataBase + 0x20u);

    f.cpu.step();  // ldrh r2, [r1], #2
    ZLB_EXPECT_EQ(f.reg(2), 0u);
    ZLB_EXPECT_EQ(f.reg(1), kDataBase + 0x22u);

    f.cpu.step();  // ldrh r3, [r4, r5]
    ZLB_EXPECT_EQ(f.reg(3), 0x8888u);

    f.cpu.step();  // strh r6, [r1, #-2]! -> address base+0x20
    ZLB_EXPECT_EQ(f.bus.read16(kDataBase + 0x20u), 0x5555u);
    ZLB_EXPECT_EQ(f.reg(1), kDataBase + 0x20u);
}

ZLB_TEST(arm_extra_load_store_signed) {
    Fixture f;
    // ldrsb r0, [r1] ; ldrsh r2, [r1, #2] ; ldrsb r3, [r1, #-1]
    f.load(kCodeBase, {a32_extra_ls(0xE, true, true, false, true, 1, 0, 0),
                       a32_extra_ls(0xE, true, true, true, true, 1, 2, 2),
                       a32_extra_ls(0xE, true, true, false, true, 1, 3, 1, true, false)});
    f.bus.write8(kDataBase, 0x80u);
    f.bus.write16(kDataBase + 2, 0x8001u);
    f.bus.write8(kDataBase - 1, 0xFFu);
    f.cpu.reset(kCodeBase);
    f.set_reg(1, kDataBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0xFFFFFF80u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 0xFFFF8001u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(3), 0xFFFFFFFFu);
}

ZLB_TEST(arm_ldrd_strd) {
    Fixture f;
    // strd r0, r1, [r2, #8] ; ldrd r4, r5, [r2], #8. LDRD/STRD encode L = 0 and
    // are selected by bits [6:5] == 10 (LDRD) / 11 (STRD).
    f.load(kCodeBase, {a32_extra_ls(0xE, false, true, true, true, 2, 0, 8),
                       a32_extra_ls(0xE, false, true, false, true, 2, 4, 8, true, true, true)});
    f.cpu.reset(kCodeBase);
    f.set_reg(0, 0x11111111u);
    f.set_reg(1, 0x22222222u);
    f.set_reg(2, kDataBase + 0x10u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0x18u), 0x11111111u);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0x1Cu), 0x22222222u);
    ZLB_EXPECT_EQ(f.reg(2), kDataBase + 0x10u);  // no writeback for STRD here
    f.cpu.step();  // ldrd r4, r5, [r2, #8]!
    ZLB_EXPECT_EQ(f.reg(4), 0x11111111u);
    ZLB_EXPECT_EQ(f.reg(5), 0x22222222u);
    ZLB_EXPECT_EQ(f.reg(2), kDataBase + 0x18u);  // writeback
}

ZLB_TEST(arm_ldr_pc_branches_and_interworks) {
    Fixture f;
    // ldr pc, [r0] with the loaded value carrying the Thumb bit.
    f.load(kCodeBase, {arm_ldr_str_imm(0xE, true, false, 0, 15, 0)});
    f.bus.write32(kDataBase, (kCodeBase + 0x100u) | 1u);
    f.cpu.reset(kCodeBase);
    f.set_reg(0, kDataBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 0x100u);
    ZLB_EXPECT_TRUE(f.cpu.thumb);
}

ZLB_TEST(arm_ldm_stm_all_addressing_modes) {
    // IA/IB/DA/DB for the store side, checking the exact addresses each mode
    // touches (ARM ARM A8.8.91: the four modes differ only in P/U). The
    // descending modes write *below* the base register.
    const struct {
        bool pre, up;
        s32 first;  // offset of the first (lowest numbered) register
    } modes[] = {{false, true, 0}, {true, true, 4}, {false, false, -12}, {true, false, -12}};
    for (const auto& mode : modes) {
        Fixture f;
        f.bus.write32(kCodeBase, arm_ldm_stm(0xE, false, mode.pre, mode.up, false, 0, 0x000E));
        f.cpu.reset(kCodeBase);
        f.set_reg(0, kDataBase + 0x20u);
        f.set_reg(1, 0x11111111u);
        f.set_reg(2, 0x22222222u);
        f.set_reg(3, 0x33333333u);
        f.cpu.step();
        const u32 at = static_cast<u32>(static_cast<s32>(kDataBase + 0x20u) + mode.first);
        ZLB_EXPECT_EQ(f.bus.read32(at), 0x11111111u);
        ZLB_EXPECT_EQ(f.bus.read32(at + 4u), 0x22222222u);
        ZLB_EXPECT_EQ(f.bus.read32(at + 8u), 0x33333333u);
        ZLB_EXPECT_EQ(f.reg(0), kDataBase + 0x20u);  // no writeback
    }

    // With writeback the base moves by +/- 12 in the same direction.
    Fixture g;
    g.bus.write32(kCodeBase, arm_ldm_stm(0xE, false, true, false, true, 0, 0x000E));
    g.cpu.reset(kCodeBase);
    g.set_reg(0, kDataBase + 0x20u);
    g.set_reg(1, 0xAAAA0001u);
    g.set_reg(2, 0xAAAA0002u);
    g.set_reg(3, 0xAAAA0003u);
    g.cpu.step();  // STMDB r0!, {r1-r3}
    ZLB_EXPECT_EQ(g.bus.read32(kDataBase + 0x14u), 0xAAAA0001u);
    ZLB_EXPECT_EQ(g.bus.read32(kDataBase + 0x1Cu), 0xAAAA0003u);
    ZLB_EXPECT_EQ(g.reg(0), kDataBase + 0x14u);

    // ... and an LDM in the matching mode reads them back.
    Fixture h;
    h.bus.write32(kCodeBase, arm_ldm_stm(0xE, true, false, true, true, 0, 0x000E));
    h.cpu.reset(kCodeBase);
    h.set_reg(0, kDataBase + 0x40u);
    h.bus.write32(kDataBase + 0x40u, 0x01020304u);
    h.bus.write32(kDataBase + 0x44u, 0x05060708u);
    h.bus.write32(kDataBase + 0x48u, 0x090A0B0Cu);
    h.cpu.step();  // LDMIA r0!, {r1-r3}
    ZLB_EXPECT_EQ(h.reg(1), 0x01020304u);
    ZLB_EXPECT_EQ(h.reg(2), 0x05060708u);
    ZLB_EXPECT_EQ(h.reg(3), 0x090A0B0Cu);
    ZLB_EXPECT_EQ(h.reg(0), kDataBase + 0x4Cu);
}

ZLB_TEST(arm_ldm_stm_pc_and_user_bank) {
    Fixture f;
    // STM stores PC + 8 (ARM ARM A8.8.91: the stored value of R15 is the
    // address of the instruction + 8).
    f.load(kCodeBase, {arm_ldm_stm(0xE, false, false, true, false, 0, 0x8000)});
    f.cpu.reset(kCodeBase);
    f.set_reg(0, kDataBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase), kCodeBase + 8u);

    // LDM with PC in the list branches.
    Fixture g;
    g.load(kCodeBase, {arm_ldm_stm(0xE, true, false, true, false, 0, 0x8002)});
    g.bus.write32(kDataBase, 0x11111111u);
    g.bus.write32(kDataBase + 4u, (kCodeBase + 0x200u) | 1u);
    g.cpu.reset(kCodeBase);
    g.set_reg(0, kDataBase);
    g.cpu.step();
    ZLB_EXPECT_EQ(g.reg(1), 0x11111111u);
    ZLB_EXPECT_EQ(g.cpu.get_pc(), kCodeBase + 0x200u);
    ZLB_EXPECT_TRUE(g.cpu.thumb);

    // `ldmia r0, {r13, r14}^` writes the *user* bank, not the current one.
    Fixture h;
    h.load(kCodeBase, {a32_ldm_stm_s(0xE, true, false, true, false, true, 0, 0x6000),
                       a32_msr_reg(0xE, false, 0x1u, 1)});
    h.bus.write32(kDataBase, 0x12345678u);
    h.bus.write32(kDataBase + 4u, 0x9ABCDEF0u);
    h.cpu.reset(kCodeBase);
    h.set_reg(0, kDataBase);
    h.set_reg(13, 0xDEAD0000u);  // SVC bank SP must survive
    h.set_reg(1, static_cast<u32>(arm::kModeUser));
    h.cpu.step();  // ldmia r0, {sp, lr}^
    ZLB_EXPECT_EQ(h.reg(13), 0xDEAD0000u);
    h.cpu.step();  // msr cpsr_c, r1 -> user mode
    ZLB_EXPECT_EQ(h.cpu.mode(), arm::kModeUser);
    ZLB_EXPECT_EQ(h.reg(13), 0x12345678u);
    ZLB_EXPECT_EQ(h.reg(14), 0x9ABCDEF0u);
}

ZLB_TEST(arm_stm_user_bank_s_bit) {
    // `stmia r0, {r13, r14}^` stores the user bank registers even from SVC mode.
    // The user bank is populated with `ldmia r1, {r13, r14}^` first, which is the
    // documented way to load it from privileged code.
    Fixture f;
    f.load(kCodeBase, {a32_ldm_stm_s(0xE, true, false, true, false, true, 1, 0x6000),
                       a32_ldm_stm_s(0xE, false, false, true, false, true, 0, 0x6000)});
    f.bus.write32(kDataBase, 0x22220000u);
    f.bus.write32(kDataBase + 4u, 0xBBBB0000u);
    f.cpu.reset(kCodeBase);
    f.set_reg(0, kDataBase + 0x40u);
    f.set_reg(1, kDataBase);
    f.set_reg(13, 0x11110000u);  // SVC bank SP
    f.set_reg(14, 0xAAAA0000u);  // SVC bank LR
    f.cpu.step();                // ldmia r1, {sp, lr}^ -> load the user bank
    ZLB_EXPECT_EQ(f.reg(13), 0x11110000u);  // the SVC bank is untouched
    ZLB_EXPECT_EQ(f.reg(14), 0xAAAA0000u);
    f.cpu.step();  // stmia r0, {sp, lr}^ -> store the user bank
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0x40u), 0x22220000u);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0x44u), 0xBBBB0000u);
    ZLB_EXPECT_EQ(f.reg(13), 0x11110000u);
    ZLB_EXPECT_EQ(f.reg(14), 0xAAAA0000u);
}

ZLB_TEST(arm_branch_and_link) {
    Fixture f;
    // bl forward, b backward, and a conditional branch that is not taken.
    f.bus.write32(kCodeBase, arm_branch(0xE, true, 0x10));         // bl +0x40
    f.bus.write32(kCodeBase + 4, arm_branch(0x1, false, -4));      // bne -16 (not taken)
    f.cpu.reset(kCodeBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(14), kCodeBase + 4u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 8u + 0x40u);

    // BL from an address where the backward offset is negative.
    f.bus.write32(kCodeBase + 8, arm_branch(0xE, true, -4));
    f.cpu.reset(kCodeBase + 8);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 8u + 8u - 16u);
    ZLB_EXPECT_EQ(f.reg(14), kCodeBase + 12u);

    // A not-taken conditional branch still writes PC + 4 (ARM state).
    f.cpu.reset(kCodeBase + 4);
    f.cpu.cpsr |= arm::kFlagZ;  // makes the BNE at kCodeBase + 4 not taken
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 8u);
}

ZLB_TEST(arm_blx_immediate_enters_thumb) {
    Fixture f;
    const u32 target = kCodeBase + 0x101u;
    f.bus.write32(kCodeBase, a32_blx_imm(kCodeBase, target & ~1u));
    f.cpu.reset(kCodeBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), target & ~1u);
    ZLB_EXPECT_TRUE(f.cpu.thumb);
    ZLB_EXPECT_EQ(f.reg(14), kCodeBase + 4u);
}

ZLB_TEST(arm_blx_immediate_h_bit_reaches_the_first_thumb_halfword) {
    for (const u32 source : {kCodeBase, kCodeBase + 0x400u}) {
        for (const u32 halfword_offset : {0u, 2u}) {
            Fixture f;
            const u32 target = kCodeBase + 0x100u + halfword_offset;
            f.load(source, {
                a32_blx_imm(source, target),
                arm_dp_imm(14, 13, false, 0, 2, 7), // MOV r2,#7 on return to ARM
            });
            // H=1 targets the second halfword of a word. Landing two bytes too
            // early executes MOVS r0,#0x99 instead of the intended MOVS r0,#42.
            f.load_halfwords(target - 2u, {0x2099u, 0x202Au, 0x4770u}); // BX LR
            f.cpu.reset(source);
            f.cpu.step();
            ZLB_EXPECT_EQ(f.cpu.get_pc(), target);
            ZLB_EXPECT_TRUE(f.cpu.thumb);
            ZLB_EXPECT_EQ(f.reg(14), source + 4u);
            f.cpu.step();
            ZLB_EXPECT_EQ(f.reg(0), 42u);
            f.cpu.step();
            ZLB_EXPECT_EQ(f.cpu.get_pc(), source + 4u);
            ZLB_EXPECT_FALSE(f.cpu.thumb);
            f.cpu.step();
            ZLB_EXPECT_EQ(f.reg(2), 7u);
            ZLB_EXPECT_FALSE(f.cpu.halted);
        }
    }
}

ZLB_TEST(arm_bx_blx_register) {
    Fixture f;
    // BX Rm does not touch LR; BLX Rm does.
    f.load(kCodeBase, {a32_bx(0), a32_blx_reg(1)});
    f.cpu.reset(kCodeBase);
    f.set_reg(0, (kCodeBase + 0x80u) | 1u);
    f.set_reg(14, 0x11111111u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 0x80u);
    ZLB_EXPECT_TRUE(f.cpu.thumb);
    ZLB_EXPECT_EQ(f.reg(14), 0x11111111u);

    f.cpu.reset(kCodeBase + 4);
    f.set_reg(1, kCodeBase + 0x100u);  // even -> ARM state
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 0x100u);
    ZLB_EXPECT_FALSE(f.cpu.thumb);
    ZLB_EXPECT_EQ(f.reg(14), kCodeBase + 8u);
}

ZLB_TEST(arm_svc_immediate) {
    Fixture f;
    f.load(kCodeBase, {0xEF000042u});  // svc #0x42
    f.cpu.reset(kCodeBase);
    f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagI);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeSupervisor);
    ZLB_EXPECT_EQ(f.reg(14), kCodeBase + 4u);  // the preferred return address
    ZLB_EXPECT_EQ(f.cpu.get_pc(), arm::kVecSupervisor);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagI) != 0);
    ZLB_EXPECT_EQ(f.cpu.exception_count, 1u);
}

ZLB_TEST(arm_exclusive_word_byte_half) {
    Fixture f;
    // LDREX/STREX succeed as a pair and fail without a reservation.
    f.load(kCodeBase, {a32_ldrex(0, 1), a32_strex(2, 3, 1)});
    f.bus.write32(kDataBase, 0x12345678u);
    f.cpu.reset(kCodeBase);
    f.set_reg(1, kDataBase);
    f.set_reg(3, 0xCAFEF00Du);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x12345678u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 0u);  // success
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase), 0xCAFEF00Du);

    // A second STREX without a new LDREX must fail and leave memory alone.
    f.bus.write32(kCodeBase + 8, a32_strex(2, 3, 1));
    f.cpu.reset(kCodeBase + 8);
    f.set_reg(1, kDataBase);
    f.set_reg(3, 0xDEADBEEFu);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 1u);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase), 0xCAFEF00Du);

    // A plain store between LDREX and STREX drops the reservation.
    f.load(kCodeBase, {a32_ldrex(0, 1), arm_ldr_str_imm(0xE, false, false, 1, 4, 0),
                       a32_strex(2, 3, 1)});
    f.cpu.reset(kCodeBase);
    f.set_reg(1, kDataBase);
    f.set_reg(3, 0xB16B00B5u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0xCAFEF00Du);
    f.cpu.step();  // STR clobbers the reservation
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 1u);

    // LDREXB/LDREXH and STREXB/STREXH use the byte/halfword monitor.
    f.bus.write32(kCodeBase, a32_ldrexb(0, 1));
    f.bus.write8(kDataBase + 3, 0x7Fu);
    f.cpu.reset(kCodeBase);
    f.set_reg(1, kDataBase + 3u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x7Fu);
    f.bus.write32(kCodeBase, a32_strexb(2, 3, 1));
    f.cpu.reset(kCodeBase);
    f.set_reg(1, kDataBase + 4u);  // a different address: the LDREXB monitor misses
    f.set_reg(3, 0x12345678u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2), 1u);  // no reservation held: fails
    f.bus.write32(kCodeBase, a32_ldrexh(0, 1));
    f.bus.write16(kDataBase + 2, 0xABCDu);
    f.cpu.reset(kCodeBase);
    f.set_reg(1, kDataBase + 2u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0xABCDu);
}

ZLB_TEST(arm_exclusive_doubleword) {
    // LDREXD/STREXD (audit L3): Rt must be even and the pair is Rt (low) / Rt+1
    // (high).  Both forms used to be decoded as a multiply, so the core clobbered
    // two registers instead of touching the monitored address.
    Fixture f;
    f.load(kCodeBase, {a32_ldrexd(0, 1), a32_strexd(4, 2, 1)});
    f.bus.write32(kDataBase, 0x11223344u);
    f.bus.write32(kDataBase + 4, 0x55667788u);
    f.cpu.reset(kCodeBase);
    f.set_reg(1, kDataBase);
    f.set_reg(2, 0xAAAABBBBu);
    f.set_reg(3, 0xCCCCDDDDu);
    f.cpu.step();  // LDREXD r0, r1, [r1]
    ZLB_EXPECT_EQ(f.reg(0), 0x11223344u);
    ZLB_EXPECT_EQ(f.reg(1), 0x55667788u);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    f.set_reg(1, kDataBase);
    f.cpu.step();  // STREXD r4, r2, r3, [r1]
    ZLB_EXPECT_EQ(f.reg(4), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase), 0xAAAABBBBu);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 4), 0xCCCCDDDDu);

    // Without a reservation the store must fail and leave memory alone.
    f.bus.write32(kCodeBase + 8, a32_strexd(4, 2, 1));
    f.cpu.reset(kCodeBase + 8);
    f.set_reg(1, kDataBase);
    f.set_reg(2, 0x11111111u);
    f.set_reg(3, 0x22222222u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(4), 1u);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase), 0xAAAABBBBu);
}

ZLB_TEST(arm_hints_and_barriers_are_noops) {
    Fixture f;
    f.load(kCodeBase, {0xE320F000u,  // nop
                       0xE320F001u,  // yield
                       0xE320F002u,  // wfe
                       0xE320F003u,  // wfi
                       0xE320F004u,  // sev
                       0xE320F0F0u,  // dbg #0
                       0xF57FF04Fu,  // dsb sy
                       0xF57FF05Fu,  // dmb sy
                       0xF57FF06Fu,  // isb sy
                       0xF57FF01Fu}); // clrex
    f.cpu.reset(kCodeBase);
    for (int i = 0; i < 10; ++i) {
        f.cpu.step();
    }
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 40u);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    ZLB_EXPECT_EQ(f.reg(0), 0u);

    // Conditional hints: the kernel boot loader's spin barrier is
    // `wfe` with cond == NE (0x1320F002), so it must be skipped when Z is set
    // and executed otherwise; `sev` (0xE320F004) is an AL no-op.
    Fixture g;
    g.load(kCodeBase, {0x1320F002u, 0xE320F004u});
    g.cpu.reset(kCodeBase);
    g.cpu.cpsr |= arm::kFlagZ;
    g.cpu.step();  // wfe (ne) is skipped
    ZLB_EXPECT_EQ(g.cpu.get_pc(), kCodeBase + 4u);
    g.cpu.step();  // sev executes and advances by 4
    ZLB_EXPECT_EQ(g.cpu.get_pc(), kCodeBase + 8u);
    ZLB_EXPECT_FALSE(g.cpu.undefined_instruction);

    Fixture h;
    h.load(kCodeBase, {0x1320F002u});
    h.cpu.reset(kCodeBase);
    h.cpu.cpsr &= ~static_cast<u32>(arm::kFlagZ);
    h.cpu.step();  // Z clear: the conditional wfe runs (as a no-op)
    ZLB_EXPECT_EQ(h.cpu.get_pc(), kCodeBase + 4u);
    ZLB_EXPECT_FALSE(h.cpu.undefined_instruction);
}

ZLB_TEST(arm_cps_and_setend) {
    Fixture f;
    // CPS: 1111 0001 0000 imod(2) M(1) 0 AIF(3) mode(5) - A = bit 8, I = bit 7,
    // F = bit 6 of the AIF field.
    const auto a32_cps = [](u32 imod, u32 aif, bool m, u32 mode) {
        return 0xF1000000u | (imod << 18) | (m ? (1u << 17) : 0u) | (aif << 6) | (mode & 0x1Fu);
    };
    f.load(kCodeBase, {a32_cps(3, 3, false, 0),  // cpsid if
                       a32_cps(2, 4, false, 0),  // cpsie a
                       0xF1010200u,              // setend be
                       0xF1010000u});            // setend le
    f.cpu.reset(kCodeBase);
    f.cpu.cpsr &= ~(arm::kFlagI | arm::kFlagF);
    f.cpu.cpsr |= arm::kFlagA;
    f.cpu.step();  // cpsid if -> I and F masked
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagI) != 0);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagF) != 0);
    f.cpu.step();  // cpsie a -> A cleared, I/F unchanged
    ZLB_EXPECT_FALSE((f.cpu.cpsr & arm::kFlagA) != 0);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagI) != 0);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagF) != 0);
    f.cpu.step();  // setend be
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagE) != 0);
    f.cpu.step();  // setend le
    ZLB_EXPECT_FALSE((f.cpu.cpsr & arm::kFlagE) != 0);

    // CPS with a mode change: `cpsid if, #0x1B` switches to Undefined mode.
    f.bus.write32(kCodeBase, a32_cps(3, 3, true, arm::kModeUndefined));
    f.cpu.reset(kCodeBase);
    f.cpu.cpsr &= ~(arm::kFlagI | arm::kFlagF);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeUndefined);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagI) != 0);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagF) != 0);
}

ZLB_TEST(arm_rfe_restores_cpsr_and_pc) {
    Fixture f;
    // RFEIA r0 loads PC from [r0] and CPSR from [r0 + 4].  Round 161: the two were
    // the other way round, and kernel_boot_loader's non-secure entry
    // (0x40021AFC, `rfeia sp!` with the target 0x51000000 at [sp] and CPSR 0x93 at
    // [sp+4]) then loaded CPSR = 0x51000000 and PC = 0x93 instead of handing over to
    // NSKBL - the real firmware is the evidence for this word order.
    f.load(kCodeBase, {0xF8900A00u});
    f.bus.write32(kDataBase, kCodeBase + 0x80u);
    f.bus.write32(kDataBase + 4u, 0x0000001Fu);            // System mode, no flags
    f.cpu.reset(kCodeBase);
    f.set_reg(0, kDataBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeSystem);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 0x80u);
    ZLB_EXPECT_FALSE(f.cpu.thumb);

    // SRSDB sp!, #0x1F stores the current (System mode encoding -> the active)
    // CPSR and LR.  Note (round 161): this still uses the order the model always
    // had - CPSR at base-4, LR at base, SP = base-8 - while RFE now loads PC from
    // the addressed word.  The two are therefore *not* a round-tripping pair any
    // more; SRS is not exercised anywhere in the 1.04 boot (the non-secure entry
    // pushes its frame by hand), so it is left as it is rather than re-guessed.
    f.load(kCodeBase, {0xF96D001Fu});
    f.cpu.reset(kCodeBase);
    f.set_reg(13, kDataBase + 0x100u);
    f.set_reg(14, 0x11223344u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(13), kDataBase + 0xF8u);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0xFCu) & arm::kModeMask, arm::kModeSupervisor);
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0x100u), 0x11223344u);
}

ZLB_TEST(arm_exception_returns_use_restored_state_alignment) {
    // RFE, LDM^ with PC, and MOVS PC all restore the ISA before BranchWritePC.
    // Address bit 0 never chooses the ISA; bit 1 survives only in Thumb state.
    for (const u32 instruction : {0xF8900A00u, 0xE8D08000u, 0xE1B0F00Eu}) {
        for (const bool restored_thumb : {false, true}) {
            for (u32 low_bits = 0; low_bits < 4; ++low_bits) {
                Fixture f;
                const u32 target = kCodeBase + 0x80u + low_bits;
                const u32 saved_cpsr = arm::kModeSystem | arm::kFlagN | arm::kFlagC |
                                       (restored_thumb ? arm::kFlagT : 0u);
                f.load(kCodeBase, {instruction});
                if (restored_thumb) f.load_halfwords(kCodeBase + 0x80u, {0x2500u, 0x255Au});
                else f.load(kCodeBase + 0x80u, {0xE3A0505Au});
                f.bus.write32(kDataBase, target);
                f.bus.write32(kDataBase + 4u, saved_cpsr);
                f.cpu.reset(kCodeBase);
                f.cpu.set_spsr(saved_cpsr);
                f.set_reg(0, kDataBase);
                f.set_reg(14, target);
                f.cpu.step();
                ZLB_EXPECT_EQ(f.cpu.get_pc(), target & (restored_thumb ? ~1u : ~3u));
                ZLB_EXPECT_EQ(f.cpu.thumb, restored_thumb);
                ZLB_EXPECT_EQ(f.cpu.cpsr, saved_cpsr);
                f.cpu.step();
                ZLB_EXPECT_EQ(f.reg(5), restored_thumb && low_bits < 2u ? 0u : 0x5Au);
                ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
            }
        }
    }
}

ZLB_TEST(arm_irq_rfe_preserves_halfword_oled_instruction_boundary) {
    Fixture f;
    const u32 interrupted_pc = kCodeBase + 0x86u;
    const u32 vector_base = kCodeBase + 0x200u;
    const u32 irq_sp = kDataBase + 0x100u;
    const u32 system_sp = kDataBase + 0x200u;
    // Genuine OLED 81000286..28C: LSL.W R0,R8,LR; CMP R5,#15; STR R5,[R4,#12].
    // An IRQ after the LSL returns to the halfword-only CMP boundary. The old
    // word-aligned RFE returned into F00E, the LSL's second halfword, and the
    // resulting F00E2D0F instruction zeroed SP in the unchanged native helper.
    f.load_halfwords(kCodeBase + 0x82u, {0xFA08u, 0xF00Eu, 0x2D0Fu, 0x60E5u});
    // Native IntrMgr entry/exit instructions: subtract IRQ LR, save PC/CPSR,
    // preserve R0, and return through its genuine A32 RFEIA SP! (F8BD0A00).
    f.load(vector_base + arm::kVecIrq, {0xE24EE004u, 0xE24DD030u, 0xE58DE028u,
             0xE58D0000u, 0xE14FE000u, 0xE58DE02Cu, 0xE59D0000u,
             0xE28DD028u, 0xF8BD0A00u});
    f.cpu.set_register("CPSR", arm::kModeIrq);
    f.set_reg(13, irq_sp);
    f.cpu.set_register("CPSR", arm::kModeSystem | arm::kFlagT | arm::kFlagC);
    f.set_reg(13, system_sp);
    f.set_reg(14, 7u);  // OLED deliberately uses LR as scratch during packing.
    f.set_reg(0, 0x12345678u);
    f.set_reg(4, kDataBase + 0x300u);
    f.set_reg(5, 18u);
    f.cpu.set_pc(interrupted_pc);
    f.cpu.mmu.vbar = vector_base;
    const u32 saved_cpsr = f.cpu.cpsr;
    f.cpu.set_irq(0, true);
    f.cpu.step();
    f.cpu.set_irq(0, false);
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeIrq);
    for (unsigned i = 0; i < 9; ++i) f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), interrupted_pc);
    ZLB_EXPECT_EQ(f.cpu.cpsr, saved_cpsr);
    ZLB_EXPECT_EQ(f.reg(13), system_sp);
    ZLB_EXPECT_EQ(f.reg(14), 7u);
    ZLB_EXPECT_EQ(f.reg(0), 0x12345678u);
    f.cpu.step();  // The intended CMP, not an instruction made of two tails.
    ZLB_EXPECT_EQ(f.cpu.get_pc(), interrupted_pc + 2u);
    ZLB_EXPECT_EQ(f.reg(13), system_sp);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 0x30Cu), 18u);
    f.cpu.set_register("CPSR", arm::kModeIrq);
    ZLB_EXPECT_EQ(f.reg(13), irq_sp);  // RFE writeback applies to the IRQ bank.
}

ZLB_TEST(arm_user_system_share_sp_lr_across_privileged_transfers) {
    // Both Security states share the same core register banks (only Monitor
    // has an additional bank). CPSR.M distinguishes privilege, not SYS SP/LR.
    for (const bool nonsecure : {false, true}) {
        Fixture f;
        f.load(kCodeBase, {a32_msr_reg(0xE, false, 1u, 0)});
        f.load(kCodeBase + 0x40u, {
            a32_ldm_stm_s(0xE, false, false, true, false, true, 0, 0x6000),
            a32_ldm_stm_s(0xE, true, false, true, false, true, 1, 0x6000),
            0xE1B0F00Eu});
        f.cpu.set_register("SCR", nonsecure ? 5u : 4u);
        f.cpu.set_register("CPSR", arm::kModeIrq);
        f.set_reg(13, 0x11110000u);
        f.cpu.set_register("CPSR", arm::kModeFiq);
        f.set_reg(13, 0x22220000u);
        f.set_reg(14, 0x22220004u);
        f.cpu.set_register("CPSR", arm::kModeSystem);
        f.set_reg(13, 0x33330000u);
        f.set_reg(14, 0x33330004u);
        f.set_reg(0, arm::kModeUser);
        f.cpu.set_pc(kCodeBase);
        f.cpu.step();  // Actual MSR SYS -> USR preserves the shared registers.
        ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeUser);
        ZLB_EXPECT_EQ(f.reg(13), 0x33330000u);
        ZLB_EXPECT_EQ(f.reg(14), 0x33330004u);
        f.set_reg(13, 0x44440000u);
        f.set_reg(14, 0x44440004u);
        f.cpu.take_exception(arm::kVecIrq, arm::kModeIrq, kCodeBase + 0x80u);
        f.cpu.set_pc(kCodeBase + 0x40u);
        f.set_reg(0, kDataBase);
        f.set_reg(1, kDataBase + 0x20u);
        f.bus.write32(kDataBase + 0x20u, 0x55550000u);
        f.bus.write32(kDataBase + 0x24u, 0x55550004u);
        f.cpu.step();  // STM^ captures the shared User/System bank from IRQ.
        ZLB_EXPECT_EQ(f.bus.read32(kDataBase), 0x44440000u);
        ZLB_EXPECT_EQ(f.bus.read32(kDataBase + 4u), 0x44440004u);
        f.cpu.step();  // LDM^ replaces it without modifying the IRQ SP.
        ZLB_EXPECT_EQ(f.reg(13), 0x11110000u);
        f.cpu.set_spsr(arm::kModeSystem);
        f.cpu.step();  // Exception return accesses those values as System.
        ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeSystem);
        ZLB_EXPECT_EQ(f.reg(13), 0x55550000u);
        ZLB_EXPECT_EQ(f.reg(14), 0x55550004u);
        f.cpu.set_register("CPSR", arm::kModeFiq);
        ZLB_EXPECT_EQ(f.reg(13), 0x22220000u);
        ZLB_EXPECT_EQ(f.reg(14), 0x22220004u);
        f.cpu.set_register("CPSR", arm::kModeIrq);
        ZLB_EXPECT_EQ(f.reg(13), 0x11110000u);
        f.cpu.set_register("CPSR", arm::kModeSystem);
        ZLB_EXPECT_EQ(f.reg(13), 0x55550000u);
        f.cpu.reset(kCodeBase);
        f.cpu.set_register("CPSR", arm::kModeSystem);
        ZLB_EXPECT_EQ(f.reg(13), 0u);
        ZLB_EXPECT_EQ(f.reg(14), 0u);
    }
}

ZLB_TEST(arm_smc_and_hvc) {
    Fixture f;
    // SMC #0 enters monitor mode through MVBAR + 8.
    f.load(kCodeBase, {0xE1600070u});
    f.cpu.reset(kCodeBase);
    f.cpu.mvbar = 0x80001000u;
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeMonitor);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), 0x80001008u);
    ZLB_EXPECT_EQ(f.reg(14), kCodeBase + 4u);

    // HVC #0 enters hypervisor mode at the HVC vector.
    Fixture g;
    g.load(kCodeBase, {0xE1400070u});
    g.cpu.reset(kCodeBase);
    g.cpu.step();
    ZLB_EXPECT_EQ(g.cpu.mode(), arm::kModeHyp);
    ZLB_EXPECT_EQ(g.cpu.get_pc(), arm::kVecHyp);
}

ZLB_TEST(arm_trustzone_smc_uses_secure_translation_and_round_trips_nonsecure_bank) {
    // Monitor remains Secure when SCR.NS=1, but MRC/MCR select the NS CP15
    // bank. Exercise both the genuine Secure-MMU-off/NS-MMU-on case and two
    // enabled translation regimes mapping the same VA to different RAM.
    for (const bool secure_mmu_on : {false, true}) {
        Fixture f;
        constexpr u32 ns_ram = 0x80100000u;
        constexpr u32 secure_table = 0x80010000u;
        constexpr u32 ns_table = 0x80014000u;
        constexpr u32 monitor = kCodeBase + 0x400u;
        constexpr u32 secure_control = 0x00C50078u;
        constexpr u32 ns_control = secure_control | (1u << 29) | 1u;
        f.bus.add_ram("nonsecure", 0x2000u, ns_ram, "test");
        f.bus.write32(secure_table + 0x800u * 4u, kCodeBase | (2u << 10) | 2u);
        f.bus.write32(ns_table + 0x800u * 4u, ns_ram | (3u << 10) | (3u << 5) | 2u);
        f.bus.write32(kCodeBase + 0x1000u, 0xA55AA55Au);
        f.bus.write32(ns_ram + 0x1000u, 0x5AA55AA5u);
        f.load(kCodeBase, {
            arm_cp15(false, 0, 2, 0, 0, 0),  // Secure TTBR0
            arm_cp15(false, 0, 3, 1, 0, 0),  // Secure DACR
            arm_cp15(false, 0, 1, 2, 0, 0),  // Secure SCTLR
            0xE1600070u,                    // SMC enters Secure Monitor
        });
        f.load(monitor + 8u, {
            arm_cp15(false, 0, 1, 3, 1, 0),  // SCR.NS=1; execution stays Secure
            arm_cp15(false, 0, 2, 4, 0, 0),  // Nonsecure TTBR0
            arm_cp15(false, 0, 3, 5, 0, 0),  // Nonsecure DACR
            arm_cp15(false, 0, 12, 6, 0, 0), // Nonsecure VBAR
            arm_cp15(false, 0, 1, 7, 0, 0),  // Nonsecure SCTLR, AFE=1
            0xE1B0F00Eu,                    // MOVS PC,LR -> Nonsecure SVC
        });
        f.load(ns_ram + 0x10u, {
            arm_ldr_str_imm(14, true, false, 9, 8, 0), // Nonsecure data
            0xE1600070u,                              // SMC from NS with MMU on
            arm_ldr_str_imm(14, true, false, 9, 7, 0), // NS data after return
        });
        f.cpu.reset(kCodeBase);
        f.cpu.mvbar = monitor;
        f.set_reg(0, secure_table);
        f.set_reg(1, 1u);
        f.set_reg(2, secure_control | (secure_mmu_on ? 1u : 0u));
        f.set_reg(3, 1u);
        f.set_reg(4, ns_table);
        f.set_reg(5, 1u << 6);
        f.set_reg(6, kCodeBase + 0x800u);
        f.set_reg(7, ns_control);
        f.set_reg(9, kCodeBase + 0x1000u);
        for (int i = 0; i < 4; ++i) f.cpu.step();
        ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeMonitor);
        for (int i = 0; i < 5; ++i) {
            const StepResult step = f.cpu.step();
            ZLB_EXPECT_FALSE(step.faulted);
            ZLB_EXPECT_TRUE(f.cpu.secure_state());
        }
        f.cpu.step();
        ZLB_EXPECT_FALSE(f.cpu.secure_state());
        ZLB_EXPECT_EQ(f.cpu.mmu.ttbr0, ns_table);
        ZLB_EXPECT_EQ(f.cpu.mmu.sctlr, ns_control);
        f.cpu.step();
        ZLB_EXPECT_EQ(f.reg(8), 0x5AA55AA5u);
        f.cpu.mmu.par = 0x22200000u; // NS PAR is not overwritten by Monitor operations

        // The second SMC probes NS registers while reading Secure memory. The
        // Secure descriptor deliberately has AP=010, AF=0: it is legal because
        // Secure SCTLR.AFE=0, independent of Nonsecure SCTLR.AFE=1.
        f.load(monitor + 8u, {
            arm_cp15(true, 0, 1, 0, 0, 0),  // NS SCTLR
            arm_cp15(true, 0, 2, 1, 0, 0),  // NS TTBR0
            arm_ldr_str_imm(14, true, false, 9, 2, 0),
            arm_cp15(true, 0, 12, 3, 0, 0), // NS VBAR
            arm_cp15(false, 0, 7, 9, 8, 0), // ATS1CPR uses Secure translation
            arm_cp15(true, 0, 7, 12, 4, 0), // NS PAR retains its previous value
            arm_cp15(false, 0, 1, 10, 1, 0), // select Secure CP15 registers
            arm_cp15(true, 0, 7, 8, 4, 0),  // ATS1CPR result from Secure PAR
            arm_cp15(false, 0, 1, 11, 1, 0), // select NS registers
            arm_cp15(false, 0, 7, 9, 8, 4), // ATS12NSOPR uses NS translation
            arm_cp15(true, 0, 7, 6, 4, 0),  // NS PAR again remains unchanged
            arm_cp15(false, 0, 1, 10, 1, 0), // SCR.NS=0 is legal in Monitor
            arm_cp15(true, 0, 2, 4, 0, 0),  // Secure TTBR0
            arm_cp15(true, 0, 7, 5, 4, 0),  // NS translation result in Secure PAR
            arm_cp15(false, 0, 1, 11, 1, 0), // SCR.NS=1, execution still Secure
            0xE1B0F00Eu,                    // MOVS PC,LR back to NS
        });
        f.set_reg(10, 0u);
        f.set_reg(11, 1u);
        f.cpu.cpsr |= arm::kFlagC | arm::kFlagZ;
        const u32 ns_cpsr = f.cpu.cpsr;
        f.cpu.step();
        ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeMonitor);
        ZLB_EXPECT_TRUE(f.cpu.secure_state());
        ZLB_EXPECT_EQ(f.cpu.cpsr & (arm::kFlagC | arm::kFlagZ), arm::kFlagC | arm::kFlagZ);
        ZLB_EXPECT_EQ(f.cpu.mmu.ttbr0, secure_table);
        ZLB_EXPECT_EQ(f.cpu.mmu.enabled(), secure_mmu_on);
        for (int i = 0; i < 16; ++i) {
            const StepResult step = f.cpu.step();
            ZLB_EXPECT_FALSE(step.faulted);
        }
        ZLB_EXPECT_EQ(f.reg(0), ns_control);
        ZLB_EXPECT_EQ(f.reg(1), ns_table);
        ZLB_EXPECT_EQ(f.reg(2), 0xA55AA55Au);
        ZLB_EXPECT_EQ(f.reg(3), kCodeBase + 0x800u);
        ZLB_EXPECT_EQ(f.reg(4), secure_table);
        ZLB_EXPECT_EQ(f.reg(12), 0x22200000u);
        ZLB_EXPECT_EQ(f.reg(6), 0x22200000u);
        ZLB_EXPECT_EQ(f.reg(8), kCodeBase + 0x1000u);
        ZLB_EXPECT_EQ(f.reg(5), ns_ram + 0x1000u);
        ZLB_EXPECT_EQ(f.cpu.cpsr, ns_cpsr);
        ZLB_EXPECT_FALSE(f.cpu.secure_state());
        ZLB_EXPECT_EQ(f.cpu.mmu.ttbr0, ns_table);
        ZLB_EXPECT_EQ(f.cpu.mmu.dacr, 1u << 6);
        ZLB_EXPECT_EQ(f.cpu.mmu.par, 0x22200000u);
        f.cpu.step();
        ZLB_EXPECT_EQ(f.reg(7), 0x5AA55AA5u);
        ZLB_EXPECT_EQ(f.cpu.mmu.ifsr, 0u);
        ZLB_EXPECT_FALSE(f.cpu.halted);
    }
}

ZLB_TEST(arm_trustzone_exception_from_monitor_clears_scr_ns_and_uses_secure_vectors) {
    Fixture f;
    f.load(kCodeBase, {arm_cp15(false, 0, 1, 0, 1, 0)});
    f.cpu.reset(kCodeBase);
    f.cpu.set_register("CPSR", arm::kModeMonitor | arm::kFlagI | arm::kFlagF);
    f.cpu.mmu.vbar = kCodeBase + 0x200u;
    f.set_reg(0, 1u);
    f.cpu.step();
    ZLB_EXPECT_TRUE(f.cpu.secure_state());
    ZLB_EXPECT_EQ(f.cpu.scr & arm::kScrNs, 1u);
    f.cpu.take_exception(arm::kVecDataAbort, arm::kModeAbort, kCodeBase + 8u);
    ZLB_EXPECT_TRUE(f.cpu.secure_state());
    ZLB_EXPECT_EQ(f.cpu.scr & arm::kScrNs, 0u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 0x200u + arm::kVecDataAbort);
}

ZLB_TEST(arm_trustzone_cp15_register_banks_remain_independent) {
    struct Register {
        u32 crn, crm, opc2, secure, nonsecure;
    };
    const Register registers[] = {
        {1, 0, 0, 0x00C50078u, 0x20C50078u}, // SCTLR (MMU stays off)
        {2, 0, 0, 0x80010000u, 0x80014000u}, // TTBR0
        {2, 0, 1, 0x8001004Au, 0x8001404Au}, // TTBR1
        {2, 0, 2, 1u, 2u},                 // TTBCR
        {3, 0, 0, 1u, 0x40u},              // DACR
        {5, 0, 0, 0x805u, 0x80Du},         // DFSR
        {5, 1, 0, 5u, 7u},                 // IFSR
        {6, 0, 0, 0xDEAD0000u, 0xBEEF0000u}, // DFAR
        {6, 0, 2, 0xDEAD0004u, 0xBEEF0004u}, // IFAR
        {7, 4, 0, 0x80001000u, 0x80101000u}, // PAR
        {10, 2, 0, 0x98E8E8E8u, 0x98E0E0E0u}, // PRRR
        {10, 2, 1, 0x00009898u, 0x00008888u}, // NMRR
        {12, 0, 0, kCodeBase + 0x400u, kCodeBase + 0x800u}, // VBAR
        {13, 0, 1, 0x123401u, 0x567802u},   // CONTEXTIDR
    };
    for (const Register& reg : registers) {
        Fixture f;
        f.load(kCodeBase, {
            arm_cp15(false, 0, reg.crn, 1, reg.crm, reg.opc2),
            arm_cp15(false, 0, 1, 2, 1, 0), // SCR.NS=1
            arm_cp15(false, 0, reg.crn, 3, reg.crm, reg.opc2),
            arm_cp15(true, 0, reg.crn, 4, reg.crm, reg.opc2),
            arm_cp15(false, 0, 1, 5, 1, 0), // SCR.NS=0
            arm_cp15(true, 0, reg.crn, 6, reg.crm, reg.opc2),
        });
        f.cpu.reset(kCodeBase);
        f.cpu.set_register("CPSR", arm::kModeMonitor | arm::kFlagI | arm::kFlagF);
        f.set_reg(1, reg.secure);
        f.set_reg(2, 1u);
        f.set_reg(3, reg.nonsecure);
        f.set_reg(5, 0u);
        for (int i = 0; i < 6; ++i) {
            const StepResult step = f.cpu.step();
            ZLB_EXPECT_FALSE(step.faulted);
            ZLB_EXPECT_TRUE(f.cpu.secure_state());
        }
        ZLB_EXPECT_EQ(f.reg(4), reg.nonsecure);
        ZLB_EXPECT_EQ(f.reg(6), reg.secure);
    }
}

ZLB_TEST(arm_exception_entry_preserves_flags_and_applies_architectural_masks) {
    constexpr u32 flags = arm::kFlagN | arm::kFlagZ | arm::kFlagC | arm::kFlagV |
                          arm::kFlagQ | arm::kFlagGE;
    constexpr u32 it_mask = (3u << 25) | (0x3Fu << 10);
    struct Exception {
        u32 mode;
        u32 masks;
    };
    const Exception exceptions[] = {
        {arm::kModeMonitor, arm::kFlagA | arm::kFlagI | arm::kFlagF},
        {arm::kModeIrq, arm::kFlagA | arm::kFlagI},
        {arm::kModeFiq, arm::kFlagA | arm::kFlagI | arm::kFlagF},
        {arm::kModeAbort, arm::kFlagA | arm::kFlagI},
        {arm::kModeSupervisor, arm::kFlagI},
        {arm::kModeUndefined, arm::kFlagI},
    };
    for (const Exception& exception : exceptions) {
        for (const bool thumb_vectors : {false, true}) {
            Fixture f;
            f.cpu.reset(kCodeBase);
            f.cpu.cpsr = arm::kModeSystem | flags | it_mask | arm::kFlagJ | arm::kFlagT |
                         arm::kFlagE;
            f.cpu.mmu.sctlr = thumb_vectors ? ((1u << 30) | (1u << 25)) : 0u;
            const u32 saved = f.cpu.cpsr;
            f.cpu.take_exception(arm::kVecSupervisor, exception.mode, kCodeBase + 4u);
            ZLB_EXPECT_EQ(f.cpu.cpsr & flags, flags);
            ZLB_EXPECT_EQ(f.cpu.cpsr & (arm::kFlagJ | it_mask), 0u);
            ZLB_EXPECT_EQ(f.cpu.cpsr & (arm::kFlagA | arm::kFlagI | arm::kFlagF), exception.masks);
            ZLB_EXPECT_EQ(f.cpu.thumb, thumb_vectors);
            ZLB_EXPECT_EQ((f.cpu.cpsr & arm::kFlagE) != 0u, thumb_vectors);
            ZLB_EXPECT_EQ(f.cpu.banked_spsr(exception.mode), saved);
        }
    }
    // NS handlers cannot set CPSR.A unless SCR.AW permits it, and preserve F.
    for (const bool allow_abort_mask : {false, true}) {
        Fixture f;
        f.cpu.reset(kCodeBase);
        f.cpu.set_register("SCR", arm::kScrNs | (allow_abort_mask ? arm::kScrAw : 0u));
        f.cpu.cpsr = arm::kModeSystem | flags | arm::kFlagF;
        f.cpu.take_exception(arm::kVecIrq, arm::kModeIrq, kCodeBase + 4u);
        ZLB_EXPECT_FALSE(f.cpu.secure_state());
        ZLB_EXPECT_EQ(f.cpu.cpsr & (arm::kFlagA | arm::kFlagI | arm::kFlagF),
                      arm::kFlagI | arm::kFlagF | (allow_abort_mask ? arm::kFlagA : 0u));
    }
    for (const bool allow_fiq_mask : {false, true}) {
        Fixture f;
        f.cpu.reset(kCodeBase);
        f.cpu.set_register("SCR", arm::kScrNs | (allow_fiq_mask ? arm::kScrFw : 0u));
        f.cpu.cpsr = arm::kModeSystem | flags;
        f.cpu.take_exception(arm::kVecFiq, arm::kModeFiq, kCodeBase + 4u);
        ZLB_EXPECT_EQ(f.cpu.cpsr & (arm::kFlagA | arm::kFlagI | arm::kFlagF),
                      arm::kFlagI | (allow_fiq_mask ? arm::kFlagF : 0u));
    }
}

ZLB_TEST(arm_trustzone_physical_interrupts_follow_scr_routing) {
    for (const bool nonsecure : {false, true}) {
        for (const bool fiq : {false, true}) {
            for (const bool routed : {false, true}) {
                Fixture f;
                const u32 offset = fiq ? arm::kVecFiq : arm::kVecIrq;
                const u32 normal_mode = fiq ? arm::kModeFiq : arm::kModeIrq;
                const u32 target_mode = routed ? arm::kModeMonitor : normal_mode;
                const u32 route_bit = fiq ? arm::kScrFiq : arm::kScrIrq;
                constexpr u32 secure_vectors = kCodeBase + 0x400u;
                constexpr u32 ns_vectors = kCodeBase + 0x800u;
                constexpr u32 monitor_vectors = kCodeBase + 0xC00u;
                f.cpu.reset(kCodeBase);
                f.cpu.mmu.vbar = secure_vectors;
                f.cpu.vbar_nonsecure = ns_vectors;
                f.cpu.mvbar = monitor_vectors;
                f.cpu.set_register("SCR", (nonsecure ? arm::kScrNs : 0u) |
                                         (routed ? route_bit : 0u));
                f.cpu.cpsr = arm::kModeSystem | arm::kFlagZ | arm::kFlagQ;
                const u32 saved = f.cpu.cpsr;
                f.cpu.set_irq(fiq ? 1 : 0, true);
                f.cpu.step();
                const u32 vectors = routed ? monitor_vectors : (nonsecure ? ns_vectors : secure_vectors);
                ZLB_EXPECT_EQ(f.cpu.mode(), target_mode);
                ZLB_EXPECT_EQ(f.cpu.get_pc(), vectors + offset);
                ZLB_EXPECT_EQ(f.cpu.banked_spsr(target_mode), saved);
                ZLB_EXPECT_EQ(f.reg(14), kCodeBase + 4u);
                ZLB_EXPECT_EQ(f.cpu.secure_state(), routed || !nonsecure);
                ZLB_EXPECT_EQ(f.cpu.scr & arm::kScrNs, nonsecure ? arm::kScrNs : 0u);
                ZLB_EXPECT_EQ(f.cpu.exception_count, 1u);
                f.cpu.set_irq(fiq ? 1 : 0, false);
                f.bus.write32(vectors + offset, arm_dp_imm(14, 13, false, 0, 7, 0x42));
                ZLB_EXPECT_FALSE(f.cpu.step().faulted);
                ZLB_EXPECT_EQ(f.reg(7), 0x42u);
            }
        }
    }
}

ZLB_TEST(arm_trustzone_routed_interrupts_still_obey_cpsr_masks) {
    for (const bool fiq : {false, true}) {
        Fixture f;
        f.load(kCodeBase, {arm_dp_imm(14, 13, false, 0, 0, 0x42)});
        f.cpu.reset(kCodeBase);
        f.cpu.set_register("SCR", arm::kScrNs | (fiq ? arm::kScrFiq : arm::kScrIrq));
        f.cpu.cpsr = arm::kModeSystem | (fiq ? arm::kFlagF : arm::kFlagI);
        unsigned wakeups = 0;
        f.cpu.irq_hook = [&]() { ++wakeups; };
        f.cpu.set_irq(fiq ? 1 : 0, true);
        ZLB_EXPECT_FALSE(f.cpu.interrupt_pending());
        if (fiq) ZLB_EXPECT_EQ(wakeups, 0u);
        f.cpu.step();
        ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeSystem);
        ZLB_EXPECT_EQ(f.cpu.exception_count, 0u);
        ZLB_EXPECT_EQ(f.reg(0), 0x42u);
        if (fiq) {
            f.cpu.set_irq(1, false);
            f.cpu.cpsr &= ~arm::kFlagF;
            f.cpu.set_irq(1, true);
            ZLB_EXPECT_EQ(wakeups, 1u);
            f.cpu.set_irq(1, true);
            ZLB_EXPECT_EQ(wakeups, 1u);
            ZLB_EXPECT_TRUE(f.cpu.interrupt_pending());
        }
    }
}

ZLB_TEST(arm_trustzone_nskbl_cpsid_if_preserves_secure_fiq_enable) {
    Fixture f;
    // Native handoff: SCR=5 and CPSR=0x93. The actual first NSKBL instruction
    // is CPSID IF; SCR.FW=0 forbids its Nonsecure F update.
    f.load(kCodeBase, {0xF10C00C0u, 0xF10800C0u});
    f.cpu.reset(kCodeBase);
    f.cpu.set_register("SCR", 5u);
    f.cpu.cpsr = 0x93u;
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.cpsr, 0x93u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.cpsr, 0x13u);
}

ZLB_TEST(arm_trustzone_cps_filters_a_and_f_in_all_encodings) {
    for (unsigned world = 0; world < 3; ++world) {
        for (unsigned permissions = 0; permissions < 4; ++permissions) {
            for (const bool initially_masked : {false, true}) {
                for (unsigned encoding = 0; encoding < 3; ++encoding) {
                    Fixture f;
                    if (encoding == 0) f.load(kCodeBase, {0xF10C01C0u, 0xF10801C0u});
                    if (encoding == 1) f.load_halfwords(kCodeBase, {0xB677u, 0xB667u});
                    if (encoding == 2) f.load_halfwords(kCodeBase, {0xF3AFu, 0x86E0u, 0xF3AFu, 0x84E0u});
                    f.cpu.reset(kCodeBase | (encoding != 0 ? 1u : 0u));
                    f.cpu.cpsr = (f.cpu.cpsr & arm::kFlagT) | arm::kModeSystem | arm::kFlagN;
                    if (world == 2) f.cpu.set_register("CPSR", (f.cpu.cpsr & ~arm::kModeMask) | arm::kModeMonitor);
                    f.cpu.set_register("SCR", (world != 0 ? arm::kScrNs : 0u) |
                                              ((permissions & 1u) ? arm::kScrFw : 0u) |
                                              ((permissions & 2u) ? arm::kScrAw : 0u));
                    const u32 initial = initially_masked ? arm::kFlagA | arm::kFlagF : 0u;
                    f.cpu.cpsr |= initial;
                    const u32 allowed = (world != 1 || (permissions & 1u) ? arm::kFlagF : 0u) |
                                        (world != 1 || (permissions & 2u) ? arm::kFlagA : 0u);
                    f.cpu.step();
                    ZLB_EXPECT_EQ(f.cpu.cpsr & (arm::kFlagA | arm::kFlagI | arm::kFlagF),
                                  initial | allowed | arm::kFlagI);
                    ZLB_EXPECT_EQ(f.cpu.cpsr & arm::kFlagN, arm::kFlagN);
                    f.cpu.step();
                    ZLB_EXPECT_EQ(f.cpu.cpsr & (arm::kFlagA | arm::kFlagI | arm::kFlagF), initial & ~allowed);
                    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
                }
            }
        }
    }
}

ZLB_TEST(arm_trustzone_msr_filters_masks_and_decodes_thumb_source_register) {
    for (const bool thumb_code : {false, true}) {
        for (unsigned permissions = 0; permissions < 4; ++permissions) {
            for (const bool initially_masked : {false, true}) {
                Fixture f;
                if (thumb_code) f.load_halfwords(kCodeBase, {0xF385u, 0x8300u, 0xF386u, 0x8300u});
                else f.load(kCodeBase, {a32_msr_reg(14, false, 3u, 5u), a32_msr_reg(14, false, 3u, 6u)});
                f.cpu.reset(kCodeBase | (thumb_code ? 1u : 0u));
                f.cpu.set_register("SCR", arm::kScrNs |
                                          ((permissions & 1u) ? arm::kScrFw : 0u) |
                                          ((permissions & 2u) ? arm::kScrAw : 0u));
                const u32 state = arm::kModeSupervisor | (thumb_code ? arm::kFlagT : 0u);
                const u32 initial = initially_masked ? arm::kFlagA | arm::kFlagF : 0u;
                f.cpu.cpsr = state | initial | arm::kFlagZ;
                f.set_reg(0, state); // The old T32 decoder incorrectly reads this.
                f.set_reg(5, state | arm::kFlagA | arm::kFlagI | arm::kFlagF);
                f.set_reg(6, state);
                const u32 allowed = ((permissions & 1u) ? arm::kFlagF : 0u) |
                                    ((permissions & 2u) ? arm::kFlagA : 0u);
                f.cpu.step();
                ZLB_EXPECT_EQ(f.cpu.cpsr & (arm::kFlagA | arm::kFlagI | arm::kFlagF), initial | allowed | arm::kFlagI);
                ZLB_EXPECT_EQ(f.cpu.cpsr & arm::kFlagZ, arm::kFlagZ);
                f.cpu.step();
                ZLB_EXPECT_EQ(f.cpu.cpsr & (arm::kFlagA | arm::kFlagI | arm::kFlagF), initial & ~allowed);
                ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
            }
        }
    }
    // The same Rn and R fields select a non-r0 SPSR write in Thumb state.
    Fixture f;
    f.load_halfwords(kCodeBase, {0xF397u, 0x8F00u}); // MSR SPSR_fsxc,r7
    f.cpu.reset(kCodeBase | 1u);
    f.set_reg(0, 0xAAAAAAAAu);
    f.set_reg(7, arm::kModeSystem | arm::kFlagN | arm::kFlagF);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.spsr(), f.reg(7));
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
}

ZLB_TEST(arm_trustzone_exception_return_filters_masks_in_source_world) {
    for (unsigned world = 0; world < 3; ++world) {
        for (unsigned permissions = 0; permissions < 4; ++permissions) {
            for (const bool initially_masked : {false, true}) {
                for (const bool rfe : {false, true}) {
                    Fixture f;
                    constexpr u32 target = kCodeBase + 0x80u;
                    f.load(kCodeBase, {rfe ? 0xF8900A00u : 0xE1B0F00Eu});
                    f.cpu.reset(kCodeBase);
                    if (world == 2) f.cpu.set_register("CPSR", arm::kModeMonitor);
                    f.cpu.set_register("SCR", (world != 0 ? arm::kScrNs : 0u) |
                                              ((permissions & 1u) ? arm::kScrFw : 0u) |
                                              ((permissions & 2u) ? arm::kScrAw : 0u));
                    const u32 initial = initially_masked ? arm::kFlagA | arm::kFlagF : 0u;
                    f.cpu.cpsr = f.cpu.mode() | initial;
                    const u32 return_psr = arm::kModeSystem | arm::kFlagT | arm::kFlagZ |
                                          (initially_masked ? 0u : arm::kFlagA | arm::kFlagF);
                    f.cpu.set_spsr(return_psr);
                    f.set_reg(14, target);
                    f.set_reg(0, kDataBase);
                    f.bus.write32(kDataBase, target);
                    f.bus.write32(kDataBase + 4u, return_psr);
                    const u32 allowed = (world != 1 || (permissions & 1u) ? arm::kFlagF : 0u) |
                                        (world != 1 || (permissions & 2u) ? arm::kFlagA : 0u);
                    f.cpu.step();
                    ZLB_EXPECT_EQ(f.cpu.get_pc(), target);
                    ZLB_EXPECT_EQ(f.cpu.mode(), arm::kModeSystem);
                    ZLB_EXPECT_TRUE(f.cpu.thumb);
                    ZLB_EXPECT_EQ(f.cpu.cpsr & (arm::kFlagA | arm::kFlagF),
                                  (initial & ~allowed) | (return_psr & allowed));
                    ZLB_EXPECT_EQ(f.cpu.cpsr & arm::kFlagZ, arm::kFlagZ);
                    ZLB_EXPECT_EQ(f.cpu.secure_state(), world == 0);
                }
            }
        }
    }
}

ZLB_TEST(arm_thumb_mov_pc_branches_without_exchanging_instruction_set) {
    for (const bool tagged_target : {false, true}) {
        Fixture f;
        // Actual panic helper sequence at VA C1C20: MOV PC,R3; NOP; NOP.W.
        // The computed Thumb target is even; its bit zero is not an ISA tag.
        f.load_halfwords(kCodeBase, {0x469Fu, 0xBF00u, 0xF3AFu, 0x8000u});
        constexpr u32 target = kCodeBase + 0x42u; // Halfword-only alignment.
        f.load_halfwords(target, {0x2542u}); // MOVS r5,#0x42
        f.cpu.reset(kCodeBase | 1u);
        f.cpu.cpsr |= arm::kFlagN | arm::kFlagC | arm::kFlagQ;
        const u32 saved = f.cpu.cpsr;
        f.set_reg(3, target | (tagged_target ? 1u : 0u));
        f.cpu.step();
        ZLB_EXPECT_EQ(f.cpu.get_pc(), target);
        ZLB_EXPECT_EQ(f.cpu.cpsr, saved);
        ZLB_EXPECT_TRUE(f.cpu.thumb);
        f.cpu.step();
        ZLB_EXPECT_EQ(f.reg(5), 0x42u);
        ZLB_EXPECT_EQ(f.cpu.get_pc(), target + 2u);
        ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
    }
}

ZLB_TEST(arm_thumb_bx_still_exchanges_to_arm_for_an_even_target) {
    Fixture f;
    f.load_halfwords(kCodeBase, {0x4718u}); // BX r3
    constexpr u32 target = kCodeBase + 0x40u;
    f.load(target, {arm_dp_imm(14, 13, false, 0, 5, 0x42)});
    f.cpu.reset(kCodeBase | 1u);
    f.set_reg(3, target);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), target);
    ZLB_EXPECT_FALSE(f.cpu.thumb);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(5), 0x42u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), target + 4u);
}

ZLB_TEST(arm_memory_context_marks_monitor_secure_and_identifies_core) {
    Fixture f;
    f.cpu.core_id_ = 3;
    f.load(kCodeBase, {arm_ldr_str_imm(14, true, false, 0, 1, 0), 0xE1600070u});
    constexpr u32 monitor = kCodeBase + 0x400u;
    f.load(monitor + 8u, {arm_ldr_str_imm(14, false, false, 0, 1, 0)});
    f.cpu.reset(kCodeBase);
    f.cpu.mvbar = monitor;
    f.cpu.set_register("SCR", arm::kScrNs);
    f.set_reg(0, kDataBase);
    f.cpu.step();
    ZLB_EXPECT_TRUE(f.bus.context.nonsecure);
    ZLB_EXPECT_EQ(f.bus.context.core_id, 3u);
    f.cpu.step(); // SMC enters Secure Monitor while preserving SCR.NS.
    f.cpu.step(); // The Monitor store must carry a Secure access attribute.
    ZLB_EXPECT_FALSE(f.bus.context.nonsecure);
    ZLB_EXPECT_EQ(f.bus.context.core_id, 3u);
    ZLB_EXPECT_EQ(f.cpu.scr & arm::kScrNs, arm::kScrNs);
}

ZLB_TEST(arm_coprocessor_space) {
    Fixture f;
    // MCR/MRC and CDP for a coprocessor with no hook are architecturally
    // accepted by this core (it logs and moves on), and MRC p15 returns the
    // identification registers.
    f.load(kCodeBase, {
                          arm_cp15(true, 0, 0, 0, 0, 0),   // mrc p15,0,r0,c0,c0,0 : MIDR
                          arm_cp15(true, 0, 0, 1, 0, 1),   // mrc p15,0,r1,c0,c0,1 : CTR
                          arm_cp15(true, 0, 0, 2, 0, 5),   // mrc p15,0,r2,c0,c0,5 : MPIDR
                          0xEE010100u,                     // cdp p1,#0,c0,c1,c0,#0
                          0xEC410100u,                     // mcrr p1,#0,r0,r1,c0
                          0xEC510100u,                     // mrrc p1,#0,r0,r1,c0
                          0xEC300101u,                     // ldc p1,c0,[r0],#-4
                      });
    f.cpu.reset(kCodeBase);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x410FC090u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(1), 0x8444C003u);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(2) & 3u, 0u);
    for (int i = 0; i < 4; ++i) f.cpu.step();
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 28u);
    ZLB_EXPECT_FALSE(f.cpu.undefined_instruction);
}

// ===========================================================================
// A32 DSP/media instructions the core implements
// ===========================================================================

ZLB_TEST(arm_media_qadd_family) {
    Fixture f;
    // QADD/QSUB/QDADD/QDSUB saturate to 32 bits and set the Q flag.
    const auto run = [&](u32 instr, u32 rm, u32 rn, u32& out) {
        f.bus.write32(kCodeBase, instr);
        f.cpu.reset(kCodeBase);
        f.set_reg(5, rm);  // the first operand is in bits [3:0] for QADD Rd, Rm, Rn
        f.set_reg(2, rn);
        f.cpu.step();
        out = f.reg(3);
    };
    u32 out = 0;
    // qadd r3, r5, r2 : Rd = sat(Rm + Rn)
    run(0xE1023155u, 0x7FFFFFFFu, 1u, out);
    ZLB_EXPECT_EQ(out, 0x7FFFFFFFu);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagQ) != 0);
    run(0xE1023155u, 0x00000001u, 0x00000002u, out);
    ZLB_EXPECT_EQ(out, 3u);
    ZLB_EXPECT_FALSE((f.cpu.cpsr & arm::kFlagQ) != 0);
    // qsub r3, r5, r2 : Rd = sat(Rm - Rn)
    run(0xE1223155u, 0x80000000u, 1u, out);
    ZLB_EXPECT_EQ(out, 0x80000000u);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagQ) != 0);
    // qdadd r3, r5, r2 : Rd = sat(Rm + sat(2*Rn)) - doubling overflows here, so
    // the doubled value saturates to INT32_MAX and the sum is that value.
    run(0xE1423155u, 0u, 0x40000000u, out);
    ZLB_EXPECT_EQ(out, 0x7FFFFFFFu);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagQ) != 0);
    // qdsub r3, r5, r2 : sat(Rm - sat(2*Rn)) = 0 - INT32_MAX = 0x80000001.
    run(0xE1623155u, 0u, 0x40000000u, out);
    ZLB_EXPECT_EQ(out, 0x80000001u);
}

ZLB_TEST(arm_media_halfword_multiply) {
    Fixture f;
    // smlabb r2, r5, r1, r3 : Rd[19:16], Ra[15:12], Rm[11:8], Rn[3:0].
    const auto run = [&](u32 instr, u32 r5, u32 r1, u32 r3, u32& out) {
        f.bus.write32(kCodeBase, instr);
        f.cpu.reset(kCodeBase);
        f.set_reg(5, r5);
        f.set_reg(1, r1);
        f.set_reg(3, r3);
        f.cpu.step();
        out = f.reg(2);
    };
    u32 out = 0;
    run(0xE1023185u, 0x0002FFFFu, 0x00030004u, 10u, out);  // SMLABB: (-1 * 4) + 10
    ZLB_EXPECT_EQ(out, 6u);
    run(0xE10231A5u, 0x0002FFFFu, 0x00030004u, 10u, out);  // SMLATB: (2 * 4) + 10
    ZLB_EXPECT_EQ(out, 18u);
    run(0xE10231C5u, 0x0002FFFFu, 0x00030004u, 10u, out);  // SMLABT: (-1 * 3) + 10
    ZLB_EXPECT_EQ(out, 7u);
    run(0xE10231E5u, 0x0002FFFFu, 0x00030004u, 10u, out);  // SMLATT: (2 * 3) + 10
    ZLB_EXPECT_EQ(out, 16u);

    // SMLALBB r3, r2, r5, r1 accumulates a 64 bit signed halfword product.
    f.bus.write32(kCodeBase, 0xE1423185u);
    f.cpu.reset(kCodeBase);
    f.set_reg(5, 0x0000FFFFu);  // -1
    f.set_reg(1, 0x00000004u);
    f.set_reg(3, 0x00000010u);  // RdLo
    f.set_reg(2, 0x00000000u);  // RdHi
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(3), 0x0000000Cu);
    ZLB_EXPECT_EQ(f.reg(2), 0x00000000u);
}

ZLB_TEST(arm_media_smulw_and_smlaw) {
    Fixture f;
    // SMLAWy/SMULWy (ARM ARM A8.8.152/A8.8.163): the 32 bit operand is Rn
    // (bits [3:0] of the encoding, r5 here) and the halfword operand is Rm
    // (bits [11:8], r1 here); "B"/"T" pick its bottom/top half. SMLAW adds the
    // accumulator *before* taking bits [47:16] of the sum.
    const auto run = [&](u32 instr, u32 rm, u32 rs, u32 ra, u32& out) {
        f.bus.write32(kCodeBase, instr);
        f.cpu.reset(kCodeBase);
        f.set_reg(5, rm);
        f.set_reg(1, rs);
        f.set_reg(3, ra);
        f.cpu.step();
        out = f.reg(2);
    };
    u32 out = 0;
    // smulwb r2, r5, r1 : (0x00030000 * 4) >> 16 = 12
    run(0xE12231A5u, 0x00030000u, 0x00000004u, 0u, out);
    ZLB_EXPECT_EQ(out, 12u);
    // smulwt r2, r5, r1 : uses the top half of r1 = 4 -> 12 as well
    run(0xE12231E5u, 0x00030000u, 0x00040000u, 0u, out);
    ZLB_EXPECT_EQ(out, 12u);
    // smlawb r2, r5, r1, r3 : ((0x00030000 * 4) + 0x00010000) >> 16 = 13
    run(0xE1223185u, 0x00030000u, 0x00000004u, 0x00010000u, out);
    ZLB_EXPECT_EQ(out, 13u);
    // smlawb with a different word operand proves the operands are not swapped:
    // 0x00050000 (r5, the word) with the halfword 8 gives (0x280000)>>16 = 40.
    run(0xE1223185u, 0x00050000u, 0x00000008u, 0u, out);
    ZLB_EXPECT_EQ(out, 40u);
    // smulbb r2, r5, r1
    run(0xE1623185u, 0x00000003u, 0x00000004u, 0u, out);
    ZLB_EXPECT_EQ(out, 12u);
}

ZLB_TEST(arm_media_parallel_add_sub) {
    Fixture f;
    // SADD16/SSUB16/SADD8/SSUB8 and the GE flags of the 8 bit forms.
    const auto run = [&](u32 instr, u32 rn, u32 rm, u32& out) {
        f.bus.write32(kCodeBase, instr);
        f.cpu.reset(kCodeBase);
        f.set_reg(2, rn);
        f.set_reg(5, rm);
        f.cpu.step();
        out = f.reg(3);
    };
    u32 out = 0;
    run(0xE6123115u, 0x00010002u, 0x00030004u, out);  // sadd16 r3, r2, r5
    ZLB_EXPECT_EQ(out, 0x00040006u);
    run(0xE6123175u, 0x00010002u, 0x00030004u, out);  // ssub16 r3, r2, r5
    ZLB_EXPECT_EQ(out, 0xFFFEFFFEu);
    run(0xE6123195u, 0x01020304u, 0x01020304u, out);  // sadd8 r3, r2, r5
    ZLB_EXPECT_EQ(out, 0x02040608u);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagGE) == arm::kFlagGE);  // all lanes >= 0
    run(0xE61231F5u, 0x00000000u, 0x01000000u, out);  // ssub8 r3, r2, r5
    ZLB_EXPECT_EQ(out, 0xFF000000u);
    ZLB_EXPECT_EQ(f.cpu.cpsr & arm::kFlagGE, 0x7u << 16);  // GE[3] clear
    // QADD16 saturates each halfword.
    run(0xE6223115u, 0x7FFF7FFFu, 0x7FFF7FFFu, out);
    ZLB_EXPECT_EQ(out, 0x7FFF7FFFu);
    // SHADD16 halves the sum (rounding down).
    run(0xE6323115u, 0x00040006u, 0x00020002u, out);
    ZLB_EXPECT_EQ(out, 0x00030004u);
    // UADD16 is the unsigned form.
    run(0xE6523115u, 0xFFFF0000u, 0x00010002u, out);
    ZLB_EXPECT_EQ(out, 0x00000002u);
}

ZLB_TEST(arm_media_saturate) {
    Fixture f;
    // SSAT r3, #sat, r5 {, shift} - the sat_imm field is bits [20:16] and holds
    // sat - 1 for the signed form.  Capstone: ssat r3, #3, r5, lsl #2.
    const auto run = [&](u32 instr, u32 value, u32& out) {
        f.bus.write32(kCodeBase, instr);
        f.cpu.reset(kCodeBase);
        f.set_reg(5, value);
        f.cpu.step();
        out = f.reg(3);
    };
    u32 out = 0;
    run(0xE6A23115u, 0x00000010u, out);  // 0x10 << 2 = 0x40 -> saturates to 3
    ZLB_EXPECT_EQ(out, 3u);
    ZLB_EXPECT_TRUE((f.cpu.cpsr & arm::kFlagQ) != 0);
    run(0xE6A23115u, 0xFFFFFFFCu, out);  // -4 << 2 = -16 -> saturates to -4
    ZLB_EXPECT_EQ(out, 0xFFFFFFFCu);
    // usat r3, #2, r5, lsl #2 : USAT saturates into 0..(2^sat - 1)
    run(0xE6E23115u, 0x00000010u, out);
    ZLB_EXPECT_EQ(out, 3u);
    run(0xE6E23115u, 0xFFFFFFFCu, out);  // negative saturates to 0
    ZLB_EXPECT_EQ(out, 0u);
}

ZLB_TEST(arm_media_extend) {
    Fixture f;
    const auto run = [&](u32 instr, u32 value, u32 rn_value, u32& out) {
        f.bus.write32(kCodeBase, instr);
        f.cpu.reset(kCodeBase);
        f.set_reg(5, value);
        f.set_reg(2, rn_value);
        f.cpu.step();
        out = f.reg(3);
    };
    u32 out = 0;
    run(0xE6AF3075u, 0x00000080u, 0u, out);  // sxtb r3, r5 (rot 0)
    ZLB_EXPECT_EQ(out, 0xFFFFFF80u);
    run(0xE6BF3075u, 0x00008000u, 0u, out);  // sxth r3, r5
    ZLB_EXPECT_EQ(out, 0xFFFF8000u);
    run(0xE6EF3075u, 0x00000080u, 0u, out);  // uxtb r3, r5
    ZLB_EXPECT_EQ(out, 0x00000080u);
    run(0xE6FF3075u, 0x00008000u, 0u, out);  // uxth r3, r5
    ZLB_EXPECT_EQ(out, 0x00008000u);
    // SXTB with a rotate: ror #8 first (0x00008000 ror 8 = 0x80000080).
    run(0xE6AF3475u, 0x00008000u, 0u, out);  // sxtb r3, r5, ror #8
    ZLB_EXPECT_EQ(out, 0xFFFFFF80u);
    // SXTAB adds Rn.
    run(0xE6A23075u, 0x00000080u, 5u, out);  // sxtab r3, r2, r5
    ZLB_EXPECT_EQ(out, 0xFFFFFF85u);
}

ZLB_TEST(arm_media_sel) {
    Fixture f;
    // SEL Rd, Rn, Rm takes its bytes from Rn where the corresponding GE bit is
    // set and from Rm otherwise (ARM ARM A8.8.157).  The operand order used to be
    // the other way round, which every `usub8`/`sadd8` + `sel` idiom (byte-wise
    // min/max in the firmware) ran backwards.  Reference: a real ARMv7 core.
    f.load(kCodeBase, {0xE6810FB2u});  // sel r0, r1, r2
    f.cpu.reset(kCodeBase);
    f.set_reg(1, 0x11111111u);
    f.set_reg(2, 0x22222222u);
    f.cpu.cpsr = (f.cpu.cpsr & ~arm::kFlagGE) | (0x5u << 16);  // GE0 and GE2 set
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x22112211u);  // GE set -> Rn = r1
}

ZLB_TEST(arm_thumb2_data_processing_register_family) {
    // 1111 1010 op1 Rn | 1111 Rd op2 Rm - data-processing (register).  Every
    // instruction below used to run as an arbitrary parallel add/sub because the
    // decoder read the ARM (32-bit) selector layout out of the Thumb fields.  CLZ
    // in particular broke NSKBL's size classifier (`clz r3, r5` returned 0 instead
    // of 15), so no 64 KiB allocation ever succeeded and the kernel heap was never
    // created (docs/NSKBL.md round 212).  All expected values, GE and Q bits come
    // from Unicorn (a real ARMv7 core model) run on the same encoding.
    Fixture f;
    struct Case {
        u32 hw1;
        u32 hw2;
        u32 r5;
        u32 r6;
        u32 expect;
        u32 ge;       // expected CPSR GE nibble
        bool q;       // expected CPSR Q flag
    };
    const Case cases[] = {
        {0xFAB5u, 0xF385u, 0x00010002u, 0u, 0x0000000Fu, 0u, false},  // clz r3, r5
        {0xFAB5u, 0xF385u, 0x00000000u, 0u, 0x00000020u, 0u, false},  // clz of zero = 32
        {0xFAB5u, 0xF385u, 0x80000000u, 0u, 0x00000000u, 0u, false},
        {0xFA95u, 0xF385u, 0x12345678u, 0u, 0x78563412u, 0u, false},  // rev.w
        {0xFA95u, 0xF395u, 0x12345678u, 0u, 0x34127856u, 0u, false},  // rev16.w
        {0xFA95u, 0xF3B5u, 0x00001234u, 0u, 0x00003412u, 0u, false},  // revsh.w
        {0xFA95u, 0xF3B5u, 0x0000ABCDu, 0u, 0xFFFFCDABu, 0u, false},  // revsh sign extends
        // parallel add/sub with the operand in r6
        {0xFA95u, 0xF306u, 0x00010002u, 0x00030004u, 0x00040006u, 0xFu, false},  // sadd16
        {0xFAC5u, 0xF306u, 0x00000000u, 0x01000000u, 0xFF000000u, 0x7u, false},  // ssub8
        {0xFA85u, 0xF346u, 0x80018002u, 0x80018002u, 0x00020004u, 0xAu, false},  // uadd8 carry
        {0xFAC5u, 0xF346u, 0x10203040u, 0x01020304u, 0x0F1E2D3Cu, 0xFu, false},  // usub8
        {0xFAA5u, 0xF306u, 0x80018002u, 0x80018002u, 0x00030001u, 0x3u, false},  // sasx
        {0xFAE5u, 0xF306u, 0x00010002u, 0x00030004u, 0xFFFD0005u, 0x3u, false},  // ssax
        {0xFA95u, 0xF315u, 0x12345678u, 0x12345678u, 0x24687FFFu, 0u, false},    // qadd16
        {0xFA95u, 0xF325u, 0x00040006u, 0x00020002u, 0x00040006u, 0u, false},    // shadd16
        {0xFA85u, 0xF386u, 0x40000000u, 0x40000000u, 0x7FFFFFFFu, 0u, true},     // qadd
        {0xFA85u, 0xF396u, 0x00000001u, 0x00000000u, 0x00000002u, 0u, false},    // qdadd doubles Rn
        {0xFA85u, 0xF3A6u, 0x00000001u, 0x00000000u, 0xFFFFFFFFu, 0u, false},    // qsub is Rm - Rn
        {0xFA85u, 0xF3B6u, 0x40000000u, 0x40000000u, 0xC0000001u, 0u, true},     // qdsub
        // extend-and-add
        {0xFA05u, 0xF386u, 0x00010002u, 0x00008001u, 0x00008003u, 0u, false},    // sxtah
        {0xFA55u, 0xF386u, 0x00010002u, 0x000000F0u, 0x000100F2u, 0u, false},    // uxtab
        {0xFA45u, 0xF386u, 0x00010002u, 0x00000080u, 0x0000FF82u, 0u, false},    // sxtab
        {0xFA25u, 0xF386u, 0x00010002u, 0x00008080u, 0x0001FF82u, 0u, false},    // sxtab16
        {0xFA05u, 0xF3A6u, 0x00010002u, 0x80010000u, 0x00008003u, 0u, false},    // sxtah ror #16
        {0xFA1Fu, 0xF385u, 0x00008001u, 0u, 0x00008001u, 0u, false},             // uxth (Rn == 15)
        {0xFA2Fu, 0xF385u, 0x00008080u, 0u, 0x0000FF80u, 0u, false},             // sxtb16
    };
    for (const Case& c : cases) {
        // `load` writes a little-endian word, so hw1 (the halfword at the lower
        // address) is the low half of the stored value.
        f.load(kCodeBase, {c.hw1 | (c.hw2 << 16)});
        f.cpu.reset(kCodeBase | 1u);
        f.set_reg(5, c.r5);
        f.set_reg(6, c.r6);
        f.cpu.cpsr = (f.cpu.cpsr & ~arm::kFlagGE) & ~arm::kFlagQ;
        f.cpu.step();
        ZLB_EXPECT_TRUE(!f.cpu.undefined_instruction);
        ZLB_EXPECT_EQ(f.reg(3), c.expect);
        ZLB_EXPECT_EQ((f.cpu.cpsr >> 16) & 0xFu, c.ge);
        ZLB_EXPECT_EQ((f.cpu.cpsr & arm::kFlagQ) != 0u, c.q);
    }

    // SEL (op1 = 0xA, op2 = 8) reads Rn = r5 and Rm = r6 and picks Rn where GE is
    // set: GE = 0101 -> bytes 0 and 2 come from r5 (0xAA), the rest from r6 (0xBB).
    f.load(kCodeBase, {0xF386FAA5u});
    f.cpu.reset(kCodeBase | 1u);
    f.set_reg(5, 0xAAAAAAAAu);
    f.set_reg(6, 0xBBBBBBBBu);
    f.cpu.cpsr = (f.cpu.cpsr & ~arm::kFlagGE) | (0x5u << 16);
    f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(3), 0xBBAABBAAu);
}

ZLB_TEST(disasm_thumb2_data_processing_register) {
    // The disassembler shared the interpreter's wrong table and named all of these
    // after a parallel add/sub (an `sadd16`-based name for `revsh`).
    Fixture f;
    f.load(kCodeBase, {0xF385FAB5u, 0xF395FA95u, 0xF3B5FA95u, 0xF386FAA5u,
                       0xF386FA85u, 0xF386FA45u, 0xF346FAC5u, 0xF306FA95u});
    unsigned length = 0;
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase, true, length).find("clz r3, r5") != std::string::npos);
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase + 4, true, length).find("rev16.w r3, r5") != std::string::npos);
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase + 8, true, length).find("revsh.w r3, r5") != std::string::npos);
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase + 12, true, length).find("sel r3, r5, r6") != std::string::npos);
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase + 16, true, length).find("qadd r3, r6, r5") != std::string::npos);
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase + 20, true, length).find("sxtab r3, r5, r6") != std::string::npos);
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase + 24, true, length).find("usub8 r3, r5, r6") != std::string::npos);
    ZLB_EXPECT_TRUE(arm_disassemble(f.bus, kCodeBase + 28, true, length).find("sadd16 r3, r5, r6") != std::string::npos);
}

// `step()` used to disassemble every instruction into StepResult::text. Nothing
// outside the core reads it, and for any code whose PA differs from its VA the
// listing was produced from an unmapped read of the *virtual* address (the bus
// resolves an unmapped byte by scanning every device) - so it was both garbage
// and the single most expensive thing per guest instruction. It is opt-in now.
ZLB_TEST(arm_step_only_disassembles_when_a_tool_asks_for_it) {
    Fixture f;
    f.load(kCodeBase, {arm_dp_imm(0xEu, 0x4u, true, 1u, 0u, 1u)});   // add r0, r1, #1
    f.cpu.reset(kCodeBase);

    f.cpu.step_text = false;
    StepResult plain = f.cpu.step();
    ZLB_EXPECT_TRUE(plain.text.empty());
    ZLB_EXPECT_EQ(plain.address, kCodeBase);
    ZLB_EXPECT_EQ(plain.length, 4u);

    f.cpu.reset(kCodeBase);
    f.cpu.step_text = true;
    StepResult listed = f.cpu.step();
    unsigned length = 0;
    const std::string expected = arm_disassemble(f.bus, kCodeBase, false, length);
    ZLB_EXPECT_TRUE(!listed.text.empty());
    ZLB_EXPECT_TRUE(listed.text == expected);
}
