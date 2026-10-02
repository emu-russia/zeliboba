"""Search the loaded NSKBL image for a 32-bit constant in any of the forms the
compiler uses: a literal-pool word, or movs/movt (+ shift) pieces.

Usage: python tools/nskbl_find_value.py 0x80024300
"""
import struct
import sys

IMAGE = r"..\_scratch\nskbl_mem.bin"
BASE = 0x51000000


def main():
    value = int(sys.argv[1], 0)
    data = open(IMAGE, "rb").read()
    literal = struct.pack("<I", value)
    print("literal words:")
    start = 0
    while True:
        at = data.find(literal, start)
        if at < 0:
            break
        print("  0x%08X" % (BASE + at))
        start = at + 1
    high = (value >> 16) & 0xFFFF
    if (value & 0xFF) == 0:
        # value = (low8 << 8) | high16 : movs rN,#low8 ; movt rN,#high16 ; lsls rN,#8
        low8 = (value >> 8) & 0xFF
        print("movs #0x%02X + movt #0x%04X pattern:" % (low8, high))
        for register in range(16):
            movs = bytes([low8, 0x20 | register])
            movt = bytes([0xC8, 0xF2, 0x02, register])
            start = 0
            while True:
                at = data.find(movs, start)
                if at < 0:
                    break
                start = at + 1
                for offset in range(2, 15, 2):
                    if data[at + offset:at + offset + 4] == movt:
                        print("  0x%08X  movs r%d,#0x%02X + movt r%d,#0x%04X (+%d)"
                              % (BASE + at, register, low8, register, high, offset))
                        break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
