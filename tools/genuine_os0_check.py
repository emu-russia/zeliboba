"""Compare psp2bootconfig.skprx in the genuine os0 partition image with the
extracted filesystem tree the model currently builds its volume from.

The loader's format validator (0x5101A4B0) only accepts 0x7F 'E' 'L' 'F', i.e. a
SELF/ELF; the extracted tree holds a decrypted module starting with "SCE\\0".
"""
import glob
import struct

GENUINE = r"C:\Work\PSVita\Vita_104_Firmware\Out\PUP_dec\os0.bin"
TREE = r"C:\Work\PSVita\Vita_104_Firmware\Out\fs\os0\psp2bootconfig.skprx"


def head(path, count=16):
    with open(path, "rb") as handle:
        data = handle.read(count)
    return " ".join("%02X" % b for b in data)


print("extracted tree  :", head(TREE))
print("genuine os0.bin :", end=" ")

boot = open(GENUINE, "rb").read(512)
rsv = struct.unpack_from("<H", boot, 14)[0]
nfats = boot[16]
fat_sectors = struct.unpack_from("<H", boot, 22)[0]
root_entries = struct.unpack_from("<H", boot, 17)[0]
spc = boot[13]
root = rsv + nfats * fat_sectors
root_sectors = (root_entries * 32 + 511) // 512
data = root + root_sectors
print(f"BPB spc={spc} reserved={rsv} fats={nfats}x{fat_sectors} root={root_entries} "
      f"-> root LBA {root}, data LBA {data}")

handle = open(GENUINE, "rb")
found = 0
for sector in range(root_sectors):
    handle.seek((root + sector) * 512)
    block = handle.read(512)
    for off in range(0, 512, 32):
        entry = block[off:off + 32]
        if entry[0] in (0x00, 0xE5) or entry[11] in (0x0F, 0x10):
            continue
        name = "".join(chr(c) if 32 <= c < 127 else "?" for c in entry[0:8]).strip()
        ext = "".join(chr(c) if 32 <= c < 127 else "?" for c in entry[8:11]).strip()
        if not (name.startswith("PSP2") or ext.startswith("SKP")):
            continue
        cluster = struct.unpack_from("<H", entry, 26)[0]
        size = struct.unpack_from("<I", entry, 28)[0]
        lba = data + (cluster - 2) * spc
        handle.seek(lba * 512)
        first = handle.read(16)
        hexs = " ".join("%02X" % b for b in first)
        text = "".join(chr(b) if 32 <= b < 127 else "." for b in first)
        print(f"  {name}.{ext:<4} cluster={cluster:<6} size={size:<8} LBA {lba:<7} {hexs} {text}")
        found += 1
        if found >= 8:
            break
    if found >= 8:
        break
