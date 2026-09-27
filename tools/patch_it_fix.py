p = 'src/cpu/arm/arm_core.cpp'
s = open(p, encoding='utf-8').read()

old = """u32 ArmCore::thumb_condition(bool& in_it_block, bool& last_in_it) {
    if (!it_state_valid_) {
        in_it_block = false;
        last_in_it = false;
        return 0xEu;
    }
    const u32 state = it_state_ & 0xFFu;
    const u32 cond = state >> 4;
    const u32 mask = state & 0xFu;
    in_it_block = true;

    if ((mask & 7u) == 0u) {
        last_in_it = true;
        it_state_valid_ = false;
        it_state_ = 0;
    } else {
        last_in_it = false;
        u32 new_mask = (mask << 1) & 0xFu;
        if ((mask & 0x10u) != 0) new_mask |= 1u;
        it_state_ = (cond << 4) | (new_mask & 0xFu);
    }
    return cond;
}"""
new = """u32 ArmCore::thumb_condition(bool& in_it_block, bool& last_in_it) {
    if (!it_state_valid_) {
        in_it_block = false;
        last_in_it = false;
        return 0xEu;
    }
    const u32 state = it_state_ & 0xFFu;
    const u32 firstcond = state >> 4;
    const u32 mask = state & 0xFu;
    in_it_block = true;

    // The mask's top bit is the 'T'/'E' selector: 'T' keeps the first condition,
    // 'E' inverts its lowest bit (ARM ARM A7.3.5, ITSTATE[7:5] and the mask).  The
    // KBL depends on it: 0x40020AAC is `ite hs / ldrhs r0,[r0,#0x40] /
    // ldrlo r0,[r0,#0x60]`, and without the inversion the second load ran too.
    const u32 cond = firstcond ^ ((mask & 0x8u) != 0u ? 1u : 0u);

    if ((mask & 7u) == 0u) {
        last_in_it = true;
        it_state_valid_ = false;
        it_state_ = 0;
    } else {
        last_in_it = false;
        it_state_ = (firstcond << 4) | ((mask << 1) & 0xFu);
    }
    return cond;
}"""
assert old in s
s = s.replace(old, new, 1)

old2 = """    const u32 state = it_state_ & 0xFFu;
    const u32 cond = state >> 4;
    const u32 mask = state & 0xFu;
    in_it_block = true;
    last_in_it = (mask & 7u) == 0u;
    return cond;"""
new2 = """    const u32 state = it_state_ & 0xFFu;
    const u32 firstcond = state >> 4;
    const u32 mask = state & 0xFu;
    in_it_block = true;
    last_in_it = (mask & 7u) == 0u;
    return firstcond ^ ((mask & 0x8u) != 0u ? 1u : 0u);"""
assert old2 in s
s = s.replace(old2, new2, 1)
open(p, 'w', encoding='utf-8').write(s)
print('IT condition fix applied')
