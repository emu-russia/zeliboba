p = 'src/cpu/arm/arm_core.cpp'
s = open(p, encoding='utf-8').read()

# 1. Remove the bogus "register-controlled shift" special case: EA0x is AND.W etc.
old_special = """    // Register-controlled shift: opc == 0000, type == 00, imm3:imm2 == 0.
    if (opc == 0u && type == 0 && amount == 0 && (hw2 & 0x00F0u) == 0u) {
        const int amount_reg = static_cast<int>((hw2 >> 8) & 0xFu);
        const u32 value = shift_reg(r[rm], 0, r[amount_reg] & 0xFFu);
        r[rn] = value;
        if (sf) set_nz(value);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

"""
assert old_special in s
s = s.replace(old_special, "", 1)

# 2. Add the real register-shift encodings (0xFA0x..0xFA7x, hw2[15:12] == 1111).
anchor = """    if (((hw1 >> 8) & 0xFu) == 0xAu) {
        parallel_add_sub32(op1, op2, rd, rn, rm);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }"""
assert anchor in s
new_case = """    // LSL/LSR/ASR/ROR (register) T2: 1111 1010 op1(4) Rn | 1111 Rd 0000 00 Rm.
    // op1 = 0/1 LSL, 2/3 LSR, 4/5 ASR, 6/7 ROR (odd = set flags).  These share the
    // 1111 1010 prefix with the parallel add/sub group, but only op1 <= 7 are
    // shifts, and an AND.W Rd, Rn, Rm (1110 1010 0000 Rn ...) must NOT land here.
    if (ra == 15 && op2 == 0u && op1 <= 7u && ((hw1 >> 8) & 0xFu) == 0xAu) {
        const u32 shifted = shift_reg(r[rn], static_cast<int>(op1 >> 1), r[rm] & 0xFFu);
        r[rd] = shifted;
        if ((op1 & 1u) != 0u) set_nz(shifted);
        write_r15(cur_instr_addr_ + 4u);
        return;
    }

""" + anchor
s = s.replace(anchor, new_case, 1)
open(p, 'w', encoding='utf-8').write(s)
print('core patched')
