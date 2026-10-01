"""Locate the builders of a 32-bit constant in the loaded NSKBL image.

The image builds its error codes with a `movw rN, #low` / `movt rN, #high` pair, so
this scans the raw bytes for that pair (properly decoded, unlike a nibble guess):

    movw rN, #imm16 : 1111 0 i 10 0100 imm4 | 0 imm3 Rd imm8
    movt rN, #imm16 : 1111 0 i 10 1100 imm4 | 0 imm3 Rd imm8

Usage: python tools/nskbl_find_const.py 0x4300 0x8002 [--image FILE] [--base 0x...]
"""
import argparse
import sys

DEFAULT_IMAGE = r"C:\Work\PSVita\_scratch\nskbl_mem.bin"
DEFAULT_BASE = 0x51000000


def encode(value, base_halfword):
    imm4 = (value >> 12) & 0xF
    imm3 = (value >> 8) & 0x7
    imm8 = value & 0xFF
    return base_halfword | imm4, imm3, imm8


def pair_bytes(value, base_halfword, register):
    half1, imm3, imm8 = encode(value, base_halfword)
    half2 = (imm3 << 12) | (register << 8) | imm8
    return bytes([half1 & 0xFF, half1 >> 8, half2 & 0xFF, half2 >> 8])


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("low", type=lambda v: int(v, 0))
    ap.add_argument("high", type=lambda v: int(v, 0))
    ap.add_argument("--image", default=DEFAULT_IMAGE)
    ap.add_argument("--base", type=lambda v: int(v, 0), default=DEFAULT_BASE)
    args = ap.parse_args(argv)

    data = open(args.image, "rb").read()
    hits = []
    for register in range(16):
        movw = pair_bytes(args.low, 0xF240, register)
        movt = pair_bytes(args.high, 0xF2C0, register)
        start = 0
        while True:
            at = data.find(movw, start)
            if at < 0:
                break
            start = at + 1
            for offset in range(4, 13, 2):
                if data[at + offset:at + offset + 4] == movt:
                    hits.append((args.base + at, register, offset))
                    break
    for address, register, offset in sorted(set(hits)):
        print("0x%08X  movw r%d + movt r%d (+%d)" % (address, register, register, offset))
    print("%d site(s)" % len(set(hits)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
