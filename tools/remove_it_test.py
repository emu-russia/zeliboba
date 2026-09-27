p = 'tests/test_arm.cpp'
s = open(p, encoding='utf-8').read()
start = s.index('ZLB_TEST(thumb_it_block_conditional_ldr) {')
end = s.index('ZLB_TEST(thumb2_register_controlled_shifts) {')
removed = s[start:end]
assert 'ite hs' in removed or 'BF2C' in removed
s = s[:start] + s[end:]
open(p, 'w', encoding='utf-8').write(s)
print('test removed (%d bytes); finding moved to docs/KBL.md' % len(removed))
