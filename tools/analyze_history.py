import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else r'.\build\hist.txt'
lines = open(path, encoding='utf-8', errors='replace').read()

# The history command prints "last N PCs of ARM:" followed by wrapped hex tokens.
pcs = re.findall(r'(?<![0-9A-Fa-f])[0-9A-Fa-f]{8}(?![0-9A-Fa-f])', lines)
pcs = [p.upper() for p in pcs]
print('tokens:', len(pcs))
if not pcs:
    sys.exit(0)

# Find the transition into the zero region at the top of RAM.
kbl_lo, kbl_hi = 0x40020000, 0x400B4000
first_out = None
for i, p in enumerate(pcs):
    value = int(p, 16)
    if not (kbl_lo <= value < kbl_hi):
        first_out = i
        break

print('first PC outside the KBL image at token', first_out)
if first_out is not None:
    lo = max(0, first_out - 40)
    print('context (last KBL instructions, then where it went):')
    for i in range(lo, min(len(pcs), first_out + 12)):
        marker = '  <-- first outside' if i == first_out else ''
        print('   %5d  %s%s' % (i, pcs[i], marker))
else:
    print('the whole history is inside the KBL image; tail:')
    for p in pcs[-20:]:
        print('   ', p)

# Region histogram, so a long zero-walk is obvious.
from collections import Counter
buckets = Counter((int(p, 16) >> 20) << 20 for p in pcs)
print('region histogram (1 MiB buckets):')
for base, count in sorted(buckets.items(), key=lambda kv: -kv[1])[:10]:
    print('   0x%08X  %d' % (base, count))
