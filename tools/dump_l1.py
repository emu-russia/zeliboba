"""Dump the KBL's L1 translation tables and report the non-fault entries."""
import re
import subprocess

EXE = r'C:\Work\PSVita\zeliboba\build\bin\zeliboba.exe'


def dump(addr, rows):
    cmd = [EXE, '-q', '--stage', 'kbl', '-ex', 'core arm', '-ex', 'runm 6000',
           '-ex', 'mem 0x%X %d' % (addr, rows), '-ex', 'quit']
    out = subprocess.run(cmd, capture_output=True, text=True, errors='replace').stdout
    data = {}
    for line in out.splitlines():
        m = re.match(r'^([0-9A-F]{8})\s+([0-9A-F ]+?)\s*\|', line)
        if not m:
            continue
        base = int(m.group(1), 16)
        blob = bytes.fromhex(m.group(2).replace(' ', ''))
        for i in range(len(blob) // 4):
            data[base + i * 4] = int.from_bytes(blob[i * 4:i * 4 + 4], 'little')
    return data


for name, base, rows in (('TTBR0', 0x40108000, 256), ('TTBR1', 0x4010C000, 512)):
    table = dump(base, rows)
    print('%s table @0x%08X: %d entries read' % (name, base, len(table)))
    for addr in sorted(table):
        word = table[addr]
        if word == 0x000001E0 or word == 0:
            continue
        index = (addr - base) // 4
        print('  index 0x%03X (VA 0x%08X): 0x%08X  kind=%s' %
              (index, index << 20, word, {0: 'fault', 1: 'page-table', 2: 'section', 3: 'reserved'}.get(word & 3, '?')))
