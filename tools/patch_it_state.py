p = 'src/cpu/arm/arm_core.cpp'
s = open(p, encoding='utf-8').read()

# 1. IT instruction: ITSTATE = firstcond[3:1] : mask[4:0].
old = """        it_state_ = ((firstcond << 4) | mask) & 0xFFu;
        it_state_valid_ = true;"""
new = """        // ITSTATE = firstcond[3:1] : mask[4:0] (ARM ARM A7.3.5).  The mask is kept
        // as a five bit field because the low bit of each instruction's condition
        // comes out of the shift below - that is what encodes the 'E' positions.
        it_state_ = ((firstcond & 0xEu) << 4) | (mask & 0x1Fu);
        it_state_valid_ = true;"""
assert old in s
s = s.replace(old, new, 1)

# 2. thumb_condition.
start = s.index('u32 ArmCore::thumb_condition(bool& in_it_block, bool& last_in_it) {')
end = s.index('\n}\n', start) + 3
new_fn = """u32 ArmCore::thumb_condition(bool& in_it_block, bool& last_in_it) {
    if (!it_state_valid_) {
        in_it_block = false;
        last_in_it = false;
        return 0xEu;
    }
    in_it_block = true;
    const u32 state = it_state_ & 0xFFu;
    // ITSTATE[7:4] is the condition of the instruction about to execute.  Bit 4 is
    // fed by the shift below, so an 'E' position inverts the block's first
    // condition - without this every instruction of an ITE block ran on the same
    // condition (KBL 0x40020AAC: `ite hs / ldrhs r0,[r0,#0x40] / ldrlo r0,[r0,#0x60]`).
    const u32 cond = (state >> 4) & 0xFu;
    if ((state & 0x7u) == 0u) {
        last_in_it = true;
        it_state_valid_ = false;
        it_state_ = 0;
    } else {
        last_in_it = false;
        it_state_ = (state & 0xE0u) | ((state << 1) & 0x1Fu);
    }
    return cond;
}
"""
s = s[:start] + new_fn + s[end:]

# 3. peek must agree.
old3 = """    const u32 state = it_state_ & 0xFFu;
    const u32 cond = state >> 4;
    const u32 mask = state & 0xFu;
    in_it_block = true;
    last_in_it = (mask & 7u) == 0u;
    return cond;"""
new3 = """    const u32 state = it_state_ & 0xFFu;
    in_it_block = true;
    last_in_it = (state & 0x7u) == 0u;
    return (state >> 4) & 0xFu;"""
assert old3 in s
s = s.replace(old3, new3, 1)
open(p, 'w', encoding='utf-8').write(s)
print('IT state machine fixed (ITSTATE = cond[3:1]:mask[4:0])')
