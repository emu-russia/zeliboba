"""Find every literal word that points at one of the given guest addresses.

Boot-path step tables in the KBL are literal pools of function pointers, so a
linear scan of the image finds who references a handler that has no static
caller (for example the fixed-heap init step at 0x4002C5F0).

Usage: python tools/kbl_findptr.py <addr> [<addr> ...]
"""
import struct
import sys

IMAGE = r"C:\Work\PSVita\_scratch\kbl.bin"
BASE = 0x40020000

data = open(IMAGE, "rb").read()
for arg in sys.argv[1:]:
    value = int(arg, 0)
    for cand, label in ((value, "exact"), (value | 1, "thumb"),
                        (value | 2, "arm"), (value & ~3, "aligned")):
        needle = struct.pack("<I", cand)
        offs = []
        i = data.find(needle)
        while i >= 0:
            offs.append(BASE + i)
            i = data.find(needle, i + 1)
        if offs:
            print("0x%08X as %-8s (0x%08X): %s"
                  % (value, label, cand, " ".join("0x%X" % o for o in offs[:24])))
