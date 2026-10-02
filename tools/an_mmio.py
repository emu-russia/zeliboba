"""Scan the real Vita kernel modules for MMIO accesses in the peripheral windows.

Usage: python tools/an_mmio.py <module.elf> <lo> <hi>
"""
import io
import struct
import sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
sys.path.insert(0, r'..\_scratch_soc')
import kdis
import kdis2

path = sys.argv[1]
lo = int(sys.argv[2], 0)
hi = int(sys.argv[3], 0)

e = kdis.load(path)
seg = e.segs[0]
VB = seg.vaddr
b = seg.data
print('module %s seg 0x%08X..0x%08X' % (path.split('\\')[-1], VB, VB + seg.filesz))

roots = set()
for i in range(0, len(b) - 2, 2):
    v = struct.unpack_from('<H', b, i)[0]
    if (v & 0xFF00) == 0xB500:
        roots.add((VB + i, 'thumb'))
for i in range(0, len(b) - 4, 4):
    w = struct.unpack_from('<I', b, i)[0]
    if (w & 0xFFFF0000) == 0xE92D0000 and (w & 0x4000):
        roots.add((VB + i, 'arm'))

d = kdis2.Dis(e, code_lo=VB, code_hi=VB + seg.filesz)
d.run(sorted(roots))

seen = {}
for ac in d.accesses:
    if lo <= ac.addr < hi:
        seen.setdefault(ac.addr, []).append(ac)
for a in sorted(seen):
    lst = seen[a]
    print('MMIO 0x%08X x%-3d e.g. 0x%08X %s' % (a, len(lst), lst[0].pc, lst[0].mnem))
print('total distinct: %d' % len(seen))
