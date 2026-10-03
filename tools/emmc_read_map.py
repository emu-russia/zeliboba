#!/usr/bin/env python3
"""Map the eMMC LBAs a run actually read onto os0/vs0 files.

Why: "how far did the boot get" is otherwise a matter of reading hundreds of
megabytes of log by eye.  The console prints one line per SDIF data command
(`ZLB_EMMC_LOG=1`, capped at 400 commands), and this script turns those LBAs
into the FAT16 files they belong to, for both partitions of
`build/emmc.img`: os0 (slot04, LBA 65536) and vs0 (slot07, LBA 360448).

Usage:
    ZLB_EMMC_LOG=1 ./run-wsl.sh -q -ex "boot" -ex "runm 1000000" -ex "quit" > run.log
    python3 tools/emmc_read_map.py run.log

    python3 tools/emmc_read_map.py run.log --image build/emmc.img --last 20

The interesting signal for the Live Area goal is the **partition**: a run that
only ever touches os0 has not started userland yet, and the first LBAs at or
beyond 360448 are the first vs0 access.
"""
import argparse
import collections
import re
import struct
import sys

BLOCK = 512
PARTITIONS = [
    ("os0", 65536),
    ("vs0", 360448),
]


class Fat16:
    """Minimal read-only FAT16 reader over one partition of an image file."""

    def __init__(self, handle, part_lba):
        self.handle = handle
        self.part_lba = part_lba
        boot = self._lba(part_lba)
        self.bps = struct.unpack_from("<H", boot, 11)[0]
        self.spc = boot[13]
        reserved = struct.unpack_from("<H", boot, 14)[0]
        fats = boot[16]
        root_entries = struct.unpack_from("<H", boot, 17)[0]
        fat_sectors = struct.unpack_from("<H", boot, 22)[0]
        self.root_lba = reserved + fats * fat_sectors
        self.root_sectors = (root_entries * 32 + self.bps - 1) // self.bps
        self.data_lba = self.root_lba + self.root_sectors
        self.fat = self._lba(part_lba + reserved, fat_sectors)
        self.owner = {}
        self.files = 0
        self._walk(self._entries(self._lba(part_lba + self.root_lba, self.root_sectors)), "")

    def _lba(self, lba, count=1):
        self.handle.seek(lba * BLOCK)
        return self.handle.read(count * BLOCK)

    def _entries(self, blob):
        return [blob[i * 32:(i + 1) * 32] for i in range(len(blob) // 32)]

    def _next(self, cluster):
        return struct.unpack_from("<H", self.fat, cluster * 2)[0]

    def _chain(self, cluster, limit=200000):
        out = []
        while 2 <= cluster < 0xFFF8 and len(out) < limit:
            out.append(cluster)
            cluster = self._next(cluster)
        return out

    def _walk(self, entries, prefix):
        for entry in entries:
            if len(entry) < 32 or entry[0] in (0x00, 0xE5):
                continue
            attr = entry[11]
            name = entry[:8].decode("latin1").rstrip()
            ext = entry[8:11].decode("latin1").rstrip()
            full = prefix + name + ("." + ext if ext else "")
            start = struct.unpack_from("<H", entry, 26)[0]
            if not start:
                continue
            if attr & 0x10:
                if name in (".", ".."):
                    continue
                blob = self._lba(self.part_lba + self.data_lba + (start - 2) * self.spc,
                                 self.spc)
                self._walk(self._entries(blob), full + "/")
            else:
                self.files += 1
                for cluster in self._chain(start):
                    self.owner[cluster] = full

    def file_of_lba(self, lba):
        rel = (lba - self.part_lba) - self.data_lba
        if rel < 0:
            return None, None
        cluster = rel // self.spc + 2
        return self.owner.get(cluster), cluster


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log", help="captured emulator output with `mmc read lba=...` lines")
    parser.add_argument("--image", default="build/emmc.img")
    parser.add_argument("--last", type=int, default=0,
                        help="only list the last N files first touched")
    args = parser.parse_args()

    reads = []
    pattern = re.compile(r"mmc\s+read lba=(\d+) count=(\d+)")
    with open(args.log, "r", errors="ignore") as handle:
        for line in handle:
            match = pattern.search(line)
            if match:
                reads.append((int(match.group(1)), int(match.group(2))))
    if not reads:
        print("no `mmc read lba=...` lines - run with ZLB_EMMC_LOG=1 (and WSLENV through run-wsl.sh)")
        return 1

    print(f"{len(reads)} data command(s), {sum(c for _, c in reads)} sector(s) requested")
    print("  lowest lba :", min(lba for lba, _ in reads))
    print("  highest lba:", max(lba + count for lba, count in reads) - 1)

    with open(args.image, "rb") as image:
        for name, part_lba in PARTITIONS:
            volume = Fat16(image, part_lba)
            touched = collections.OrderedDict()
            for lba, count in reads:
                for sector in range(lba, lba + count):
                    if sector < part_lba:
                        continue
                    full, _ = volume.file_of_lba(sector)
                    if full and full not in touched:
                        touched[full] = sector
            print(f"\n{name} (partition at LBA {part_lba}, {volume.files} files): "
                  f"{len(touched)} file(s) touched")
            items = list(touched.items())
            if args.last and len(items) > args.last:
                print(f"  ... {len(items) - args.last} earlier file(s) omitted")
                items = items[-args.last:]
            for full, sector in items:
                print(f"   {full:44s} first at lba {sector}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
