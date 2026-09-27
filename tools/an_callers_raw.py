"""Find the call sites of a function in a raw ARM segment.

Usage: python tools/an_callers_raw.py <file> <base-hex> <target-hex> [context]
"""
import sys

import capstone

path = sys.argv[1]
base = int(sys.argv[2], 16)
target = int(sys.argv[3], 16)
context = int(sys.argv[4]) if len(sys.argv) > 4 else 0x30

b = open(path, 'rb').read()
md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB)
md.detail = True

callers = []
for insn in md.disasm(b, base):
    if insn.mnemonic not in ('bl', 'blx'):
        continue
    for op in insn.operands:
        if op.type == capstone.arm.ARM_OP_IMM and op.imm == target:
            callers.append(insn.address)

print('callers of 0x%08X: %s' % (target, [hex(c) for c in callers]))
for c in callers[:3]:
    print('--- around 0x%08X ---' % c)
    start = c - context
    for insn in md.disasm(b[start - base:start - base + context * 2], start):
        mark = '  <== call' if insn.address == c else ''
        print('%08X  %-8s %s %s%s' % (insn.address, insn.bytes.hex(' '), insn.mnemonic, insn.op_str, mark))
