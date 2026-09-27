import sys

import capstone

sys.path.insert(0, r'C:\Work\PSVita\_scratch_soc')
import kdis

p = r'C:\Work\PSVita\Vita_104_Firmware\Out\fs_dec\os0\kd\bootimage.elf'
e = kdis.load(p)
seg = e.segs[0]
b = seg.data
VB = seg.vaddr
lit = VB + 0x1C7CFE
print('seg 0x%08X..0x%08X literal 0x%08X' % (VB, VB + seg.filesz, lit))

off = lit - VB
for row in range(-16, 48, 16):
    chunk = b[off + row: off + row + 16]
    print('  %08X: %s' % (lit + row, chunk.hex(' ')))

md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB)
refs = []
for insn in md.disasm(b, VB):
    if insn.mnemonic.startswith('ldr') and '[pc,' in insn.op_str:
        # compute the literal address
        try:
            imm = int(insn.op_str.split('#')[-1].rstrip(']'), 0)
        except ValueError:
            continue
        if insn.address % 4 == 0:
            base = insn.address + 4
        else:
            base = (insn.address + 4) & ~3
        if base + imm == lit:
            refs.append(insn.address)
print('refs:', [hex(r) for r in refs])
for r in refs[:3]:
    print('--- around 0x%08X ---' % r)
    start = r - 0x20
    for insn in md.disasm(b[start - VB:start - VB + 0x80], start):
        print('%08X  %-8s %s %s' % (insn.address, insn.bytes.hex(' '), insn.mnemonic, insn.op_str))
