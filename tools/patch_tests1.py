p = 'tests/test_cmep.cpp'
s = open(p, encoding='utf-8').read()

old = """    ZLB_EXPECT_EQ(f.cmep.keyring_writes(), 1u);
    const auto& slots = f.cmep.captured_keyrings();
    ZLB_EXPECT_EQ(slots.size(), static_cast<size_t>(1));
    ZLB_EXPECT_TRUE(slots.count(0x501) == 1);"""
new = """    ZLB_EXPECT_EQ(f.cmep.keyring_writes(), 1u);
    const auto& slots = f.cmep.captured_keyrings();
    // Slot 10 is the fused boot key and is always present (see
    // KeyringDevice::fused_boot_key); the capture adds one more.
    ZLB_EXPECT_EQ(slots.size(), static_cast<size_t>(2));
    ZLB_EXPECT_TRUE(slots.count(cmep::kBootKeyring) == 1);
    ZLB_EXPECT_TRUE(slots.count(0x501) == 1);"""
assert old in s
s = s.replace(old, new, 1)

old = """    // The SceKeys keyring mirrors the capture: later stages read it from there.
    ZLB_EXPECT_EQ(f.keys.keyring().size(), static_cast<size_t>(1));"""
new = """    // The SceKeys keyring mirrors the capture: later stages read it from there.
    ZLB_EXPECT_EQ(f.keys.keyring().size(), static_cast<size_t>(2));"""
assert old in s
s = s.replace(old, new, 1)

open(p, 'w', encoding='utf-8').write(s)
print('keyring capture test updated')
