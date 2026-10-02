import sys

import capstone

sys.path.insert(0, r'..\_scratch_soc')
import kdis

p = r'..\Vita_104_Firmware\Out\fs_dec\os0\kd\sysmem.elf'
e = kdis.load(p)
seg = e.segs[0]
b = seg.data
VB = seg.vaddr
print('seg 0x%08X..0x%08X' % (VB, VB + seg.filesz))

lo, hi = 0x810314C0, 0x81031560
off = lo - VB
for row in range(0, hi - lo, 16):
    chunk = b[off + row: off + row + 16]
    words = ' '.join('%08X' % int.from_bytes(chunk[i:i + 4], 'little') for i in range(0, 16, 4))
    print('  %08X: %s' % (lo + row, words))

# find code that references the table: look for movw/movt pairs producing 0x810314/0x810315
targets = {0x810314C0 >> 12, 0x81031500 >> 12}
md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB)
hits = []
for insn in md.disasm(b, VB):
    if insn.mnemonic in ('movw', 'movt') and insn.op_str:
        hits.append((insn.address, insn.mnemonic, insn.op_str))
print('movw/movt count', len(hits))
for a, m, o in hits:
    if '0x1500' in o or '0x14c0' in o or '0x14' in o and '0x81' in o:
        print('%08X %s %s' % (a, m, o))
