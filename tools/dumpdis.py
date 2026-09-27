"""Dump a memory range out of the running emulator and disassemble it as Thumb.

Usage: python tools/dumpdis.py <addr> <rows> <base> [--thumb] [--stage kbl]
"""
import re
import subprocess
import sys
import os

addr = int(sys.argv[1], 0)
rows = int(sys.argv[2]) if len(sys.argv) > 2 else 8
base = int(sys.argv[3], 0) if len(sys.argv) > 3 else addr
thumb = '--thumb' in sys.argv
stage = 'kbl' if 'kbl' in sys.argv else None

exe = r'C:\Work\PSVita\zeliboba\build\bin\zeliboba.exe'
cmd = [exe, '-q']
if stage:
    cmd += ['--stage', stage]
cmd += ['-ex', 'core arm', '-ex', 'mem 0x%X %d' % (addr, rows), '-ex', 'quit']
out = subprocess.run(cmd, capture_output=True, text=True, errors='replace').stdout

data = bytearray()
for line in out.splitlines():
    m = re.match(r'^([0-9A-F]{8})\s+((?:[0-9A-F]{2} ){1,16})\s', line)
    if not m:
        continue
    start = int(m.group(1), 16)
    if start < addr:
        continue
    if not data:
        data = bytearray()
        while len(data) < (start - addr):
            data.append(0)
    for byte in m.group(2).split():
        data.append(int(byte, 16))

path = os.path.join(r'C:\Work\PSVita\zeliboba\build', 'dump_%X.bin' % addr)
open(path, 'wb').write(bytes(data))
print('dumped %d bytes -> %s' % (len(data), path))

zdis = r'C:\Work\PSVita\zeliboba\build\bin\zdis.exe'
args = [zdis, 'arm', path, '--base', hex(base), '--count', str(rows * 8)]
if thumb:
    args.append('--thumb')
print(subprocess.run(args, capture_output=True, text=True).stdout)
