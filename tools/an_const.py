"""Which kernel modules reference a given 32-bit MMIO constant?

Usage: python tools/an_const.py E2040000 [more constants...]
"""
import glob
import os
import struct
import sys

sys.path.insert(0, r'..\_scratch_soc')
import kdis

KD = r'..\Vita_104_Firmware\Out\fs_dec\os0\kd'
constants = [int(a, 16) for a in sys.argv[1:]] or [0xE2040000]

for path in sorted(glob.glob(os.path.join(KD, '*.elf'))):
    try:
        e = kdis.load(path)
    except Exception as exc:
        continue
    seg = e.segs[0]
    b = seg.data
    for value in constants:
        pat = struct.pack('<I', value)
        pat_movw = struct.pack('<I', value & 0xFFFF)
        hits = []
        i = b.find(pat)
        while i >= 0:
            hits.append(seg.vaddr + i)
            i = b.find(pat, i + 1)
        if hits:
            print('%-22s 0x%08X literal x%d at %s' %
                  (os.path.basename(path), value, len(hits), ', '.join(hex(h) for h in hits[:4])))
