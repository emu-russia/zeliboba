"""Enumerate the 0x8032xxxx error codes the NSKBL image can build.

capstone 5.0.7 mis-decodes the MOVT encoding used here (`C8 F2 32 09` is
`movt r9, #0x8032`, but capstone reports `movt r8, #0x9032`), so this scans the
raw bytes instead: a 4-byte `movt rN, #0x8032` followed by looking back for the
`mov.w rN, #imm8` that supplies the low half of the code.

Usage: python tools/nskbl_error_codes.py [--image FILE] [--base 0x...]
"""
import argparse
import sys

DEFAULT_IMAGE = r"..\Vita_104_Firmware\Out\SLB2_dec\nsbl.bin"
DEFAULT_BASE = 0x51000000

# movt rN, #0x8032 -> halfwords 0xF2C8, (N << 8) | 0x32
MOVT = {name: bytes([0xC8, 0xF2, 0x32, register])
        for name, register in (("r8", 0x08), ("r9", 0x09), ("r10", 0x0A), ("r11", 0x0B))}
# mov.w rN, #imm8 -> halfwords 0xF04F, (N << 8) | imm8
MOVW_PREFIX = bytes([0x4F, 0xF0])


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default=DEFAULT_IMAGE)
    ap.add_argument("--base", type=lambda v: int(v, 0), default=DEFAULT_BASE)
    args = ap.parse_args(argv)

    data = open(args.image, "rb").read()
    rows = []
    for name, pattern in MOVT.items():
        start = 0
        while True:
            pos = data.find(pattern, start)
            if pos < 0:
                break
            start = pos + 1
            low = None
            for back in (4, 8):                     # the immediate may not be adjacent
                prev = data[pos - back:pos - back + 4]
                if len(prev) == 4 and prev[0:2] == MOVW_PREFIX and prev[3] == int(name[1:]):
                    low = prev[2] if back == 4 else low
                    if back == 4:
                        break
            where = args.base + pos
            code = f"0x8032{low:04X}" if low is not None else "0x8032????"
            rows.append((where, name, code))
    for where, name, code in sorted(rows):
        print(f"0x{where:08X}: movt {name}, #0x8032 -> {code}")
    print(f"{len(rows)} site(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
