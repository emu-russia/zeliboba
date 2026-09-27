p = 'tests/test_arm.cpp'
s = open(p, encoding='utf-8').read()

# The ITE test used `movs` bodies, which set the flags and therefore (correctly)
# let the NE half run.  Use flag-preserving MOV.W instead so the block's condition
# is the only thing under test.
start = s.index('ZLB_TEST(thumb_it_block_it_else) {')
end = s.index('// ===========================================================================\n// CPSR mode banking, exceptions, interrupts', start)
new_test = '''ZLB_TEST(thumb_it_block_it_else) {
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

'''
s = s[:start] + new_test + s[end:]
open(p, 'w', encoding='utf-8').write(s)
print('IT tests rewritten')
