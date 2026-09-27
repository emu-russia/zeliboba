import os

files = [
    r'C:\Work\PSVita\dumps\pch-5c-cold_first_loader.bin',
    r'C:\Work\PSVita\_scratch\second_loader.bin',
    r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2_dec\secure_kernel.bin',
    r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2_dec\second_loader.bin',
    r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2_dec\nsbl.bin',
]
for path in files:
    if not os.path.exists(path):
        continue
    data = open(path, 'rb').read()
    hits = []
    for i in range(len(data) - 4):
        if data[i] == 0xCE and data[i + 1] == 0x61:
            window = data[max(0, i - 16):i + 24]
            if b'\x49\x66' in window:
                hits.append(i)
    print('%s: %s' % (os.path.basename(path), ', '.join('0x%X' % h for h in hits[:8]) or '-'))
