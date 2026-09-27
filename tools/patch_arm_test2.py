p = 'tests/test_arm.cpp'
s = open(p, encoding='utf-8').read()

old = """    f.load(kCodeBase, {0x00002101u, 0x00001E6Bu, 0x0302EA03u, 0x0000B14Bu, 0x00002202u});
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
}"""
new = """    f.load(kCodeBase, {0x00002101u, 0x00001E6Bu, 0x0302EA03u, 0x0000B14Bu, 0x00002202u});

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
    ZLB_EXPECT_EQ(f.cpu.get_pc(), kCodeBase + 8u + 4u + 40u);
}"""
assert old in s
s = s.replace(old, new, 1)
open(p, 'w', encoding='utf-8').write(s)
print('test fixed')
