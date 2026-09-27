p = 'tests/test_arm.cpp'
s = open(p, encoding='utf-8').read()

old = """    f.load(kCodeBase, {0x00002101u, 0x00001E6Bu, 0x0302EA03u, 0x0000B14Bu, 0x00002202u});"""
new = """    // Two 16-bit instructions share one 32-bit word, so the sequence is:
    //   0x00 movs r2,#1 | 0x02 subs r3,r5,#1 | 0x04 and.w r3,r3,r2
    //   0x08 cbz r3,+40 | 0x0A movs r2,#2
    f.load(kCodeBase, {0x1E6B2101u, 0x0302EA03u, 0x2202B14Bu});"""
assert old in s
s = s.replace(old, new, 1)

# add a register-shift test right after the loop-head test
anchor = """ZLB_TEST(arm_thumb2_movw_ubfx_and_bfi) {"""
extra = '''ZLB_TEST(thumb2_register_controlled_shifts) {
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

''' + anchor
assert anchor in s
s = s.replace(anchor, extra, 1)
open(p, 'w', encoding='utf-8').write(s)
print('tests fixed/added')
