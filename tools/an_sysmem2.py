import struct
import sys

import capstone

sys.path.insert(0, r'C:\Work\PSVita\_scratch_soc')
import kdis

p = r'C:\Work\PSVita\Vita_104_Firmware\Out\fs_dec\os0\kd\sysmem.elf'
e = kdis.load(p)
seg = e.segs[0]
b = seg.data
VB = seg.vaddr
print('seg 0x%08X..0x%08X' % (VB, VB + seg.filesz))

# 1. find literals pointing at the device table
for value in (0x810314C0, 0x810314D0, 0x81031500, 0x81031530):
    pat = struct.pack('<I', value)
    hits = []
    i = b.find(pat)
    while i >= 0:
        hits.append(VB + i)
        i = b.find(pat, i + 1)
    print('literal 0x%08X: %s' % (value, [hex(h) for h in hits]))

# 2. disassemble thumb+arm and find PC-relative loads to the table
md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB)
refs = []
for insn in md.disasm(b, VB):
    if not insn.mnemonic.startswith('ldr') or '[pc' not in insn.op_str:
        continue
    try:
        imm = int(insn.op_str.split('#')[-1].rstrip(']'), 0)
    except ValueError:
        continue
    base = (insn.address + 4) & ~3
    target = base + imm
    if 0x810314C0 <= target <= 0x81031540:
        refs.append((insn.address, target))
print('table refs:', [(hex(a), hex(t)) for a, t in refs[:20]])
