import sys

sys.path.insert(0, r'C:\Work\PSVita\_scratch_soc')
import kdis

p = r'C:\Work\PSVita\Vita_104_Firmware\Out\fs_dec\os0\kd\sdif.elf'
e = kdis.load(p)
seg = e.segs[0]
print('seg vaddr 0x%08X filesz 0x%X' % (seg.vaddr, seg.filesz))
b = seg.data

for name, addr in (('table@81009FA8', 0x81009FA8),):
    off = addr - seg.vaddr
    print(name, 'offset 0x%X' % off)
    for row in range(8):
        chunk = b[off + row * 16: off + row * 16 + 16]
        words = ' '.join('%08X' % int.from_bytes(chunk[i:i + 4], 'little') for i in range(0, 16, 4))
        print('  +0x%02X: %s' % (row * 16, words))

for pat, label in ((b'\x00\x00\x04\xe2', '0xE2040000'), (b'\x00\x00\x0b\xe0', '0xE00B0000'),
                   (b'\x00\x00\x00\xe0', '0xE0000000')):
    hits = []
    i = b.find(pat)
    while i >= 0:
        hits.append(i)
        i = b.find(pat, i + 1)
    print('%s occurrences: %s' % (label, [hex(seg.vaddr + h) for h in hits[:10]]))
