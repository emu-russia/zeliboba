import glob
import os

pats = {
    'movt E588 (thumb-2)': bytes.fromhex('cef288'),
    'movt E588 var': bytes.fromhex('cff288'),
    'movw 8000': bytes.fromhex('48f20080'),
    'literal E5888000': bytes.fromhex('008088e5'),
    'literal E5888020': bytes.fromhex('208088e5'),
    'literal E5888100': bytes.fromhex('008188e5'),
}
for base in (r'..\Vita_104_Firmware\Out\fs_dec\os0\kd',
             r'..\Vita_104_Firmware\Out\fs_dec\os0',
             r'..\Vita_104_Firmware\Out\fs_dec\os0\kd'):
    for path in sorted(glob.glob(os.path.join(base, '*.elf'))):
        try:
            b = open(path, 'rb').read()
        except Exception:
            continue
        for name, pat in pats.items():
            i = b.find(pat)
            n = 0
            while i >= 0 and n < 3:
                print('%-24s %-20s +0x%X' % (os.path.basename(path), name, i))
                n += 1
                i = b.find(pat, i + 1)
