p = 'tests/test_arm.cpp'
s = open(p, encoding='utf-8').read()

anchor = """ZLB_TEST(thumb2_register_controlled_shifts) {"""
assert anchor in s

extra = '''ZLB_TEST(thumb_it_block_conditional_ldr) {
    Fixture f;
    // KBL 0x40020AAC..0x40020AB0:
    //   ite hs
    //   ldrhs r0, [r0, #0x40]     ; TTBR0 when the VA is at or below the split
    //   ldrlo r0, [r0, #0x60]     ; TTBR1 otherwise
    // Both loads are plain Thumb-16 LDRs; only the IT state picks one of them.
    f.bus.write32(0x80000140u, 0x11111111u);
    f.bus.write32(0x80000160u, 0x22222222u);
    f.load(kCodeBase, {0x6C00BF28u, 0x00006E00u});

    // HS: carry set -> the first (then) instruction runs.
    f.cpu.reset(kCodeBase | 1u);
    f.cpu.cpsr |= arm::kFlagC;
    f.set_reg(0, 0x80000100u);
    for (int i = 0; i < 3; ++i) f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x11111111u);

    // LO: carry clear -> the second (else) instruction runs.
    f.cpu.reset(kCodeBase | 1u);
    f.cpu.cpsr &= ~static_cast<u32>(arm::kFlagC);
    f.set_reg(0, 0x80000100u);
    for (int i = 0; i < 3; ++i) f.cpu.step();
    ZLB_EXPECT_EQ(f.reg(0), 0x22222222u);
}

''' + anchor
s = s.replace(anchor, extra, 1)
open(p, 'w', encoding='utf-8').write(s)
print('IT test added')
