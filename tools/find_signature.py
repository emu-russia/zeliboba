import os, sys

roots = [r'C:\Work\PSVita\Vita_104_Firmware', r'C:\Work\PSVita\dumps', r'C:\Work\PSVita\_scratch']
pats = {
    'sig_le (49 66 CE 61)': bytes([0x49, 0x66, 0xCE, 0x61]),
    'sig_be (61 CE 66 49)': bytes([0x61, 0xCE, 0x66, 0x49]),
    'imm 61CE (CE 61)': bytes([0xCE, 0x61]),
}
for root in roots:
    for dirpath, dirnames, filenames in os.walk(root):
        for name in filenames:
            path = os.path.join(dirpath, name)
            try:
                if os.path.getsize(path) > 64 * 1024 * 1024:
                    continue
                with open(path, 'rb') as fh:
                    data = fh.read()
            except OSError:
                continue
            hits = []
            for label, pat in pats.items():
                idx = data.find(pat)
                if idx >= 0:
                    hits.append('%s@0x%X' % (label, idx))
            if hits:
                print(os.path.relpath(path, r'C:\Work\PSVita'), '->', ', '.join(hits))
