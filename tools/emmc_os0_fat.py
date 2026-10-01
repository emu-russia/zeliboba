"""List the os0 FAT16 volume inside the reconstructed eMMC image.

The volume starts at LBA 65536 (measured: its boot sector is EB FE 90 "SCEI").
Prints the BPB geometry and the directory entries, so the first cluster of
psp2bootconfig.skprx can be converted into the LBA the loader must read.
"""
import struct

IMAGE = r"C:\Work\PSVita\zeliboba\build\emmc.img"
PART_LBA = 65536
BLOCK = 512


def read_lba(handle, lba, count=1):
    handle.seek(lba * BLOCK)
    return handle.read(count * BLOCK)


def main():
    handle = open(IMAGE, "rb")
    boot = read_lba(handle, PART_LBA)
    bytes_per_sector = struct.unpack_from("<H", boot, 11)[0]
    sectors_per_cluster = boot[13]
    reserved = struct.unpack_from("<H", boot, 14)[0]
    fat_count = boot[16]
    root_entries = struct.unpack_from("<H", boot, 17)[0]
    fat_sectors = struct.unpack_from("<H", boot, 22)[0]
    total_sectors = struct.unpack_from("<I", boot, 32)[0] or struct.unpack_from("<H", boot, 19)[0]
    oem = boot[3:11].decode("latin-1").strip()
    print(f"OEM '{oem}'  {bytes_per_sector} B/sector  {sectors_per_cluster} sectors/cluster  "
          f"reserved {reserved}  FATs {fat_count} x {fat_sectors}  root {root_entries}  "
          f"total {total_sectors} sectors")

    root_start = PART_LBA + reserved + fat_count * fat_sectors
    root_sectors = (root_entries * 32 + bytes_per_sector - 1) // bytes_per_sector
    data_start = root_start + root_sectors
    print(f"root dir at LBA {root_start} ({root_sectors} sectors), data at LBA {data_start}")

    entries = []
    for sector in range(root_sectors):
        block = read_lba(handle, root_start + sector)
        for off in range(0, len(block), 32):
            entry = block[off:off + 32]
            if entry[0] in (0x00, 0xE5):
                continue
            name = "".join(chr(c) if 32 <= c < 127 else "?" for c in entry[0:8]).rstrip()
            ext = "".join(chr(c) if 32 <= c < 127 else "?" for c in entry[8:11]).rstrip()
            attr = entry[11]
            cluster = struct.unpack_from("<H", entry, 26)[0] | (struct.unpack_from("<H", entry, 20)[0] << 16)
            size = struct.unpack_from("<I", entry, 28)[0]
            long_name = entry[0] == 0x0F
            if long_name:
                continue
            name = name if ext in ("", "SKPRX", "SKP") else f"{name}.{ext}"
            if attr & 0x10:
                continue
            entries.append((name, ext, attr, cluster, size))
            lba = data_start + (cluster - 2) * sectors_per_cluster
            print(f"  {name:<15} attr=0x{attr:02X} cluster={cluster:<6} size={size:<8} -> LBA {lba}")
    if not entries:
        print("  (no short-name file entries found)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
