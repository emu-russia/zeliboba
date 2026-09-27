"""Find the call sites of a function in a decrypted kernel module.

Usage: python tools/an_callers.py <module.elf> <target-hex> [context]
"""
import sys

import capstone

sys.path.insert(0, r'C:\Work\PSVita\_scratch_soc')
import kdis

path = sys.argv[1]
target = int(sys.argv[2], 16)
context = int(sys.argv[3]) if len(sys.argv) > 3 else 0x30

e = kdis.load(path)
seg = e.segs[0]
b = seg.data
VB = seg.vaddr
md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB)
md.detail = True

callers = []
for insn in md.disasm(b, VB):
    if insn.mnemonic not in ('bl', 'blx'):
        continue
    for op in insn.operands:
        if op.type == capstone.arm.ARM_OP_IMM and op.imm == target:
            callers.append(insn.address)

print('callers of 0x%08X: %s' % (target, [hex(c) for c in callers]))
for c in callers[:4]:
    print('--- around 0x%08X ---' % c)
    start = c - context
    for insn in md.disasm(b[start - VB:start - VB + context * 2], start):
        mark = '  <== call' if insn.address == c else ''
        print('%08X  %-8s %s %s%s' % (insn.address, insn.bytes.hex(' '), insn.mnemonic, insn.op_str, mark))
