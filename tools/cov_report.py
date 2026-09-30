"""Summarise a `cov save` bitmap: how much of each image actually executed.

The debugger writes one bit per `gran` bytes of VA space (`cov save <file>`), and
prints the base/gran it used.  This script turns that bitmap into the numbers a
"did the boot path really run" question needs: executed instruction sites per
image region, the share of the image that executed, and the longest unexecuted
runs (candidates for code that never ran).

    python tools/cov_report.py --preset mep --bitmap _scratch/mep_cov.bin
    python tools/cov_report.py --preset arm --bitmap _scratch/arm_cov_nskbl.bin
    python tools/cov_report.py --bitmap f.bin --base 0x40020000 --gran 2 \
        --region "KBL=0x40020000-0x4005BFC0"

Bitmaps written for one window cannot be read for another: pass the same base the
run used (`ZLB_ARM_COV_BASE` / `ZLB_MEP_COV_BASE`, printed in the log line
"coverage armed").
"""
import argparse
from pathlib import Path

# Image extents (label, first byte, one past the last byte).  They come from the
# loaders: the MeP images are the SLB2 payloads, the ARM ones are ELF segments.
PRESETS = {
    "mep": dict(base=0x00040000, gran=2, regions=[
        ("second_loader", 0x00040000, 0x00056600),   # 0x16600 bytes
        ("first_loader", 0x0005C000, 0x00060000),    # 16 KiB window
        ("secure_kernel", 0x00800000, 0x00807C00),   # 31744 bytes
    ]),
    "arm": dict(base=0x40020000, gran=2, regions=[
        ("kbl_code", 0x40020000, 0x4005BFC0),        # KBL ELF segment 2
        ("kbl_data", 0x4005C000, 0x4005C0F0),        # segment 3 (240 bytes)
    ]),
    "nskbl": dict(base=0x51000000, gran=2, regions=[
        ("nskbl", 0x51000000, 0x5102B08C),           # decoded NSKBL image
    ]),
}


def parse_region(text):
    name, _, span = text.partition("=")
    lo, _, hi = span.partition("-")
    return name, int(lo, 0), int(hi, 0)


def parse_query(text):
    name, _, addr = text.partition("=")
    return name, int(addr, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bitmap", required=True)
    ap.add_argument("--preset", choices=sorted(PRESETS))
    ap.add_argument("--base", type=lambda v: int(v, 0))
    ap.add_argument("--gran", type=int)
    ap.add_argument("--region", action="append", default=[])
    ap.add_argument("--query", action="append", default=[],
                    help="label=addr: print whether that exact site executed")
    ap.add_argument("--top", type=int, default=6, help="unexecuted runs to show per region")
    args = ap.parse_args()

    preset = PRESETS.get(args.preset, dict(base=0, gran=2, regions=[]))
    base = args.base if args.base is not None else preset["base"]
    gran = args.gran if args.gran is not None else preset["gran"]
    regions = list(preset["regions"]) + [parse_region(r) for r in args.region]

    bits = Path(args.bitmap).read_bytes()
    span = len(bits) * 8 * gran

    def executed(addr):
        if addr < base or addr >= base + span:
            return False
        index = (addr - base) // gran
        return bool(bits[index >> 3] & (1 << (index & 7)))

    print(f"bitmap {args.bitmap}: {len(bits)} bytes, base 0x{base:08X}, "
          f"{gran} bytes/bit, window 0x{base:08X}-0x{base + span:08X}")
    for name, lo, hi in regions:
        if hi > base + span:
            hi = base + span
        sites = hit = 0
        holes = []
        run_start = None
        for a in range(lo, hi, gran):
            sites += 1
            if executed(a):
                hit += 1
                if run_start is not None:
                    holes.append((run_start, a))
                    run_start = None
            elif run_start is None:
                run_start = a
        if run_start is not None:
            holes.append((run_start, hi))
        share = (100.0 * hit / sites) if sites else 0.0
        print(f"  {name:<14} 0x{lo:08X}-0x{hi:08X}  {hit}/{sites} sites executed "
              f"({share:.1f}% of the image bytes, {hi - lo} bytes)")
        holes.sort(key=lambda h: h[1] - h[0], reverse=True)
        for a, b in holes[:args.top]:
            print(f"      never executed: 0x{a:08X}-0x{b:08X}  {b - a} bytes")

    if args.query:
        print("  entry points:")
        misses = 0
        for text in args.query:
            name, addr = parse_query(text)
            ok = executed(addr)
            misses += 0 if ok else 1
            print(f"      {'EXECUTED  ' if ok else 'NOT RUN   '} {name:<44} 0x{addr:08X}")
        print(f"      {len(args.query) - misses}/{len(args.query)} executed")


if __name__ == "__main__":
    main()
