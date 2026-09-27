p = 'tests/test_arm.cpp'
s = open(p, encoding='utf-8').read()

anchor = """ZLB_TEST(arm_thumb2_movw_ubfx_and_bfi) {"""
assert anchor in s

new_test = '''ZLB_TEST(thumb2_section_fill_loop_head) {
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
    f.load(kCodeBase, {0x00002101u, 0x00001E6Bu, 0x0302EA03u, 0x0000B14Bu, 0x00002202u});
    f.set_reg(5, 0x84u);                       // even: (r5-1) & 1 == 1
    f.run_at(kCodeBase, 4, true);
    ZLB_EXPECT_EQ(f.reg(3), 1u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 10u);   // cbz not taken -> movs r2,#2

    // Odd r5 makes (r5-1) & 1 == 0, so the branch is taken and the counter keeps
    // its odd start (the odd-size path).
    f.run_at(kCodeBase, 4, true);
    f.set_reg(5, 0x85u);
    f.run_at(kCodeBase, 4, true);
    ZLB_EXPECT_EQ(f.reg(3), 0u);
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 6u + 40u);
}

''' + anchor
s = s.replace(anchor, new_test, 1)
open(p, 'w', encoding='utf-8').write(s)
print('test added')
