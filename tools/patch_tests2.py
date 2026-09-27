p = 'tests/test_cmep.cpp'
s = open(p, encoding='utf-8').read()

old = """    f.cmep.reset();
    ZLB_EXPECT_EQ(f.cmep.keyring_writes(), 0u);
    ZLB_EXPECT_TRUE(f.cmep.captured_keyrings().empty());"""
new = """    f.cmep.reset();
    ZLB_EXPECT_EQ(f.cmep.keyring_writes(), 0u);
    // Only the fused boot slot survives a reset; the loader-programmed slots do not.
    const auto& after_reset = f.cmep.captured_keyrings();
    ZLB_EXPECT_EQ(after_reset.size(), static_cast<size_t>(1));
    ZLB_EXPECT_TRUE(after_reset.count(cmep::kBootKeyring) == 1);"""
assert old in s
s = s.replace(old, new, 1)
open(p, 'w', encoding='utf-8').write(s)
print('reset test updated')
