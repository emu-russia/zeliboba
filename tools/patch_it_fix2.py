p = 'src/cpu/arm/arm_core.cpp'
s = open(p, encoding='utf-8').read()

# 1. IT instruction: keep the mask as a 5-bit field so the low condition bit can
#    come out of the shift (ARM ARM ITSTATE = cond[3:1]:mask[4:0]).
old = """        it_state_ = ((firstcond << 4) | mask) & 0xFFu;
        it_state_valid_ = true;"""
new = """        // ITSTATE = firstcond[3:1] : mask[4:0].  The low bit of each instruction's
        // condition comes out of the mask shift, which is what encodes the 'E'
        // positions (ARM ARM A7.3.5).
        it_state_ = ((firstcond & 0xEu) << 4) | (mask & 0x1Fu);
        it_state_valid_ = true;"""
assert old in s
s = s.replace(old, new, 1)

# 2. Condition/advance.
old2 = s[s.index('u32 ArmCore::thumb_condition(bool& in_it_block, bool& last_in_it) {'):]
old2 = old2[:old2.index('\n}\n') + 3]
new2 = """u32 ArmCore::thumb_condition(bool& in_it_block, bool& last_in_it) {
    if (!it_state_valid_) {
        in_it_block = false;
        last_in_it = false;
        return 0xEu;
    }
    in_it_block = true;
    const u32 state = it_state_ & 0xFFu;
    // ITSTATE[7:4] is the condition of the instruction about to execute; bit 4 is
    // fed by the mask shift below, which is how the 'E' of an ITE block inverts
    // the first condition.
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
s = s.replace(old2, new2, 1)

# 3. Peek variant must agree.
old3 = """    const u32 state = it_state_ & 0xFFu;
    const u32 firstcond = state >> 4;
    const u32 mask = state & 0xFu;
    in_it_block = true;
    last_in_it = (mask & 7u) == 0u;
    return firstcond ^ ((mask & 0x8u) != 0u ? 1u : 0u);"""
new3 = """    const u32 state = it_state_ & 0xFFu;
    in_it_block = true;
    last_in_it = (state & 0x7u) == 0u;
    return (state >> 4) & 0xFu;"""
assert old3 in s
s = s.replace(old3, new3, 1)
open(p, 'w', encoding='utf-8').write(s)
print('IT state machine fixed')
