import sys

path = r'..\_scratch\kbl.bin'
base = 0x40020000
targets = [int(x, 16) for x in sys.argv[1:]] or [0x4002B048]

data = open(path, 'rb').read()
n = len(data)

def thumb_bl_target(off):
    if off + 4 > n:
        return None
    hw1 = data[off] | (data[off + 1] << 8)
    hw2 = data[off + 2] | (data[off + 3] << 8)
    if (hw1 & 0xF800) != 0xF000:
        return None
    S = (hw1 >> 10) & 1
    imm10 = hw1 & 0x3FF
    J1 = (hw2 >> 13) & 1
    J2 = (hw2 >> 11) & 1
    imm11 = hw2 & 0x7FF
    I1 = (~(J1 ^ S)) & 1
    I2 = (~(J2 ^ S)) & 1
    off_v = (S << 24) | (I1 << 23) | (I2 << 22) | (imm10 << 12) | (imm11 << 1)
    pc = base + off + 4
    blx = (hw2 & 0x1000) == 0
    tgt = pc + off_v
    return (tgt, blx)

for want in targets:
    hits = []
    for off in range(0, n - 4, 2):
        r = thumb_bl_target(off)
        if not r:
            continue
        tgt, blx = r
        if (tgt & ~1) == (want & ~1) or tgt == want:
            hits.append(('BLX' if blx else 'BL', base + off))
    print('callers of 0x%08X: %d' % (want, len(hits)))
    for kind, addr in hits:
        print('   %s from 0x%08X' % (kind, addr))
