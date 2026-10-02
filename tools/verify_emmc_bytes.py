#!/usr/bin/env python3
"""Read-only, independent byte checks of a reconstructed firmware 1.04 eMMC.

Uses only Python's standard library; does not import the emulator's parsers.
Firmware inputs are raw PUP_dec images from an extracted Out directory or ZIP.
"""

import argparse
import contextlib
import hashlib
import json
from pathlib import Path
import struct
import sys
import zipfile


CHUNK_BYTES = 1024 * 1024
BOOT_BYTES = 0x200000
RPMB_BYTES = 0x80000
PARTITIONS = (
    ("os0_0", "os0.bin", 0x01000000, 0x01000000, 3, 6),
    ("os0_1", "os0.bin", 0x02000000, 0x01000000, 3, 6),
    ("vs0_0", "vs0.bin", 0x0B000000, 0x10000000, 4, 6),
)
BLS_PARTITIONS = (("bls0", 0x00800000), ("bls1", 0x00C00000))
SLB2_SOURCE = "boot_slb2-00.pkg.seg02"


def read_exact(stream, count):
    data = stream.read(count)
    if len(data) != count:
        raise ValueError(f"short read: expected {count} bytes, got {len(data)}")
    return data


def region_bytes(image, offset, length):
    image.seek(offset)
    return read_exact(image, length)


def compare_region(image, source_open, source_bytes, offset, slot_bytes, tail_byte):
    if source_bytes > slot_bytes:
        raise ValueError(f"source exceeds slot at 0x{offset:X}")
    image.seek(offset)
    source_hash = hashlib.sha256()
    image_hash = hashlib.sha256()
    mismatches = 0
    first_mismatch = None
    compared = 0
    with source_open() as source:
        while compared < source_bytes:
            count = min(CHUNK_BYTES, source_bytes - compared)
            expected = read_exact(source, count)
            actual = read_exact(image, count)
            source_hash.update(expected)
            image_hash.update(actual)
            if expected != actual:
                mismatches += sum(a != b for a, b in zip(expected, actual))
                if first_mismatch is None:
                    first_mismatch = compared + next(
                        i for i, (a, b) in enumerate(zip(expected, actual)) if a != b
                    )
            compared += count
        if source.read(1):
            raise ValueError("source grew during verification")

    tail_bytes = slot_bytes - source_bytes
    remaining = tail_bytes
    tail_mismatches = 0
    first_tail_mismatch = None
    while remaining:
        chunk = read_exact(image, min(CHUNK_BYTES, remaining))
        unexpected = len(chunk) - chunk.count(tail_byte)
        if unexpected and first_tail_mismatch is None:
            first_tail_mismatch = source_bytes + tail_bytes - remaining + next(
                i for i, value in enumerate(chunk) if value != tail_byte
            )
        tail_mismatches += unexpected
        remaining -= len(chunk)

    return {
        "offset": offset,
        "slot_bytes": slot_bytes,
        "source_bytes": source_bytes,
        "image_prefix_bytes": compared,
        "source_sha256": source_hash.hexdigest(),
        "image_prefix_sha256": image_hash.hexdigest(),
        "prefix_equal": mismatches == 0,
        "mismatched_bytes": mismatches,
        "first_mismatch_image_offset": (
            offset + first_mismatch if first_mismatch is not None else None
        ),
        "tail_bytes": tail_bytes,
        "tail_expected_byte": tail_byte,
        "tail_equal": tail_mismatches == 0,
        "tail_mismatched_bytes": tail_mismatches,
        "first_tail_mismatch_image_offset": (
            offset + first_tail_mismatch if first_tail_mismatch is not None else None
        ),
        "ok": mismatches == 0 and tail_mismatches == 0,
    }


def check_slb2(image, source_open, source_bytes, offset, total_mismatches):
    with source_open() as source:
        raw = read_exact(source, source_bytes)
    if len(raw) < 512 or raw[:4] != b"SLB2":
        raise ValueError("firmware source has an invalid SLB2 header")
    version, table_bytes, count = struct.unpack_from("<III", raw, 4)
    if version != 1 or table_bytes != 512 or not 0 < count <= (table_bytes - 32) // 48:
        raise ValueError("firmware source has an unsupported SLB2 table")
    actual = region_bytes(image, offset, source_bytes)
    table_mismatches = sum(a != b for a, b in zip(raw[:table_bytes], actual[:table_bytes]))
    payload_mismatches = 0
    entries = []
    ranges = []
    for index in range(count):
        entry = 32 + index * 48
        block, size = struct.unpack_from("<II", raw, entry)
        start = block * 512
        end = start + size
        if not size or start < table_bytes or end > len(raw):
            raise ValueError("firmware SLB2 entry points outside its payload area")
        ranges.append((start, end))
        name = raw[entry + 16:entry + 48].split(b"\0", 1)[0].decode("ascii", errors="replace")
        mismatches = sum(a != b for a, b in zip(raw[start:end], actual[start:end]))
        payload_mismatches += mismatches
        entries.append({"name": name, "offset": start, "bytes": size,
                        "equal": mismatches == 0, "mismatched_bytes": mismatches})
    for (_, previous_end), (next_start, _) in zip(sorted(ranges), sorted(ranges)[1:]):
        if previous_end > next_start:
            raise ValueError("firmware SLB2 payload entries overlap")
    return {
        "header_equal": raw[:32] == actual[:32],
        "table_equal": table_mismatches == 0,
        "table_mismatched_bytes": table_mismatches,
        "payloads_equal": payload_mismatches == 0,
        "payload_mismatched_bytes": payload_mismatches,
        "padding_mismatched_bytes": total_mismatches - table_mismatches - payload_mismatches,
        "entries": entries,
    }


def check_fats(image, offset, slot_bytes):
    bpb = region_bytes(image, offset, 512)
    bytes_per_sector = struct.unpack_from("<H", bpb, 11)[0]
    sectors_per_cluster = bpb[13]
    reserved = struct.unpack_from("<H", bpb, 14)[0]
    fat_count = bpb[16]
    root_entries = struct.unpack_from("<H", bpb, 17)[0]
    total_sectors = struct.unpack_from("<H", bpb, 19)[0]
    sectors_per_fat = struct.unpack_from("<H", bpb, 22)[0]
    total_sectors = total_sectors or struct.unpack_from("<I", bpb, 32)[0]
    valid = (
        bpb[510:512] == b"\x55\xaa"
        and bytes_per_sector in (512, 1024, 2048, 4096)
        and sectors_per_cluster != 0
        and sectors_per_cluster & (sectors_per_cluster - 1) == 0
        and reserved > 0 and fat_count == 2
        and root_entries > 0 and sectors_per_fat > 0
        and 0 < total_sectors * bytes_per_sector <= slot_bytes
    )
    result = {
        "bytes_per_sector": bytes_per_sector,
        "sectors_per_cluster": sectors_per_cluster,
        "reserved_sectors": reserved,
        "fat_count": fat_count,
        "sectors_per_fat": sectors_per_fat,
        "total_sectors": total_sectors,
        "bpb_valid": bool(valid),
        "ok": False,
    }
    if not valid:
        result["error"] = "invalid or unsupported FAT16 BPB"
        return result
    fat_bytes = sectors_per_fat * bytes_per_sector
    fat_offset = offset + reserved * bytes_per_sector
    root_sectors = (root_entries * 32 + bytes_per_sector - 1) // bytes_per_sector
    data_sectors = total_sectors - reserved - fat_count * sectors_per_fat - root_sectors
    cluster_count = data_sectors // sectors_per_cluster
    if not (4085 <= cluster_count < 65525) or fat_bytes < (cluster_count + 2) * 2:
        result["error"] = "BPB does not describe a FAT16 volume with sufficient FAT entries"
        return result
    first = region_bytes(image, fat_offset, fat_bytes)
    second = region_bytes(image, fat_offset + fat_bytes, fat_bytes)
    result.update({
        "cluster_count": cluster_count,
        "fat_bytes": fat_bytes,
        "fat_offsets": [fat_offset, fat_offset + fat_bytes],
        "fat_sha256": [hashlib.sha256(first).hexdigest(), hashlib.sha256(second).hexdigest()],
        "copies_equal": first == second,
        "ok": first == second,
    })
    return result


def verify(args):
    report = {"ok": False, "image": str(args.image.resolve()), "regions": [], "errors": []}
    with contextlib.ExitStack() as stack:
        image = stack.enter_context(args.image.open("rb"))
        image_stat = args.image.stat()
        report["image_bytes"] = image_stat.st_size
        sources = {}
        if args.firmware_zip:
            archive = stack.enter_context(zipfile.ZipFile(args.firmware_zip))
            report["firmware_zip"] = str(args.firmware_zip.resolve())
            for name in ("os0.bin", "vs0.bin", SLB2_SOURCE):
                matches = [item for item in archive.infolist()
                           if item.filename.endswith("PUP_dec/" + name)]
                if len(matches) != 1:
                    raise ValueError(f"expected one ZIP member ending PUP_dec/{name}, found {len(matches)}")
                item = matches[0]
                sources[name] = (lambda item=item: archive.open(item), item.file_size, item.filename)
        else:
            report["firmware"] = str(args.firmware.resolve())
            for name in ("os0.bin", "vs0.bin", SLB2_SOURCE):
                path = args.firmware / "PUP_dec" / name
                sources[name] = (lambda path=path: path.open("rb"), path.stat().st_size, str(path))

        master = region_bytes(image, 0, 512)
        version, user_blocks = struct.unpack_from("<II", master, 0x20)
        user_bytes = user_blocks * 512
        expected_bytes = user_bytes + 2 * BOOT_BYTES + RPMB_BYTES
        master_ok = (master[:32] == b"Sony Computer Entertainment Inc."
                     and version == 3 and master[510:512] == b"\x55\xaa"
                     and user_blocks > 0 and image_stat.st_size == expected_bytes)
        records = []
        for i in range(16):
            start, size, code, kind = struct.unpack_from("<IIBB", master, 0x50 + i * 17)
            if size:
                records.append((start * 512, size * 512, code, kind))
        needed = [(offset, size, code, kind) for _, _, offset, size, code, kind in PARTITIONS]
        needed.extend((offset, 0x400000, 2, 0xDA) for _, offset in BLS_PARTITIONS)
        missing = [record for record in needed if record not in records]
        report["master"] = {
            "version": version, "user_blocks": user_blocks, "user_bytes": user_bytes,
            "expected_image_bytes": expected_bytes,
            "boot_offsets": [user_bytes, user_bytes + BOOT_BYTES],
            "required_partition_records_present": not missing,
            "missing_partition_records": missing,
            "ok": master_ok and not missing,
        }
        if not report["master"]["ok"]:
            report["errors"].append("master block or image geometry differs from the documented layout")

        slots = [(label, name, offset, size) for label, name, offset, size, _, _ in PARTITIONS]
        slots.extend((label, SLB2_SOURCE, offset, 0x400000) for label, offset in BLS_PARTITIONS)
        slots.extend((label, SLB2_SOURCE, offset, BOOT_BYTES)
                     for label, offset in (("boot0", user_bytes), ("boot1", user_bytes + BOOT_BYTES)))
        for label, name, offset, size in slots:
            if offset + size > image_stat.st_size:
                report["errors"].append(f"{label} extends beyond image")
                continue
            source_open, source_size, source_path = sources[name]
            tail_byte = 0xFF if label in ("bls0", "bls1") else 0
            region = compare_region(image, source_open, source_size, offset, size, tail_byte)
            region.update({"name": label, "source": source_path})
            if name != SLB2_SOURCE:
                region["fat"] = check_fats(image, offset, size)
                region["ok"] = region["ok"] and region["fat"]["ok"]
            else:
                region["slb2"] = check_slb2(image, source_open, source_size, offset,
                                             region["mismatched_bytes"])
            report["regions"].append(region)
        if args.image_sha256:
            image.seek(0)
            digest = hashlib.sha256()
            for chunk in iter(lambda: image.read(CHUNK_BYTES), b""):
                digest.update(chunk)
            report["image_sha256"] = digest.hexdigest()
        after = args.image.stat()
        if (after.st_size, after.st_mtime_ns) != (image_stat.st_size, image_stat.st_mtime_ns):
            report["errors"].append("image changed during verification; rerun while the emulator is stopped")
        report["ok"] = not report["errors"] and all(region["ok"] for region in report["regions"])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path,
                        default=Path(__file__).resolve().parents[1] / "build" / "emmc.img")
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--firmware-zip", type=Path, help="original firmware ZIP; no extraction required")
    inputs.add_argument("--firmware", type=Path, help="extracted Out directory containing PUP_dec")
    parser.add_argument("--image-sha256", action="store_true", help="also hash the complete image")
    parser.add_argument("--json", action="store_true", help="emit the detailed report as JSON")
    args = parser.parse_args()
    try:
        report = verify(args)
    except (OSError, ValueError, zipfile.BadZipFile, RuntimeError, EOFError) as error:
        report = {"ok": False, "errors": [str(error)]}
        status = 2
    else:
        status = 0 if report["ok"] else 1
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print("verify OK" if report["ok"] else "verify FAILED")
        for error in report["errors"]:
            print(f"  error: {error}")
        for region in report.get("regions", []):
            tail_status = "OK" if region["tail_equal"] else "DIFFERS"
            print(f"  {region['name']}: prefix {'equal' if region['prefix_equal'] else 'DIFFERS'}, "
                  f"{region['image_prefix_bytes']}/{region['source_bytes']} bytes; "
                  f"tail 0x{region['tail_expected_byte']:02X} {tail_status}")
            print(f"    source SHA256: {region['source_sha256']}")
            print(f"    image  SHA256: {region['image_prefix_sha256']}")
            if region["first_mismatch_image_offset"] is not None:
                print(f"    {region['mismatched_bytes']} mismatched bytes; "
                      f"first mismatch: 0x{region['first_mismatch_image_offset']:X}")
            if region["first_tail_mismatch_image_offset"] is not None:
                print(f"    first tail mismatch: 0x{region['first_tail_mismatch_image_offset']:X}")
            if "fat" in region:
                fat = region["fat"]
                print(f"    FAT copies: {'OK' if fat['ok'] else 'FAILED'}" +
                      (f" ({fat['error']})" if "error" in fat else ""))
            if "slb2" in region:
                slb2 = region["slb2"]
                print(f"    SLB2 header/table: {'equal' if slb2['table_equal'] else 'DIFFERS'}; "
                      f"payloads: {'equal' if slb2['payloads_equal'] else 'DIFFERS'}; "
                      f"padding mismatches: {slb2['padding_mismatched_bytes']}")
        if "image_sha256" in report:
            print(f"  complete image SHA256: {report['image_sha256']}")
    return status


if __name__ == "__main__":
    sys.exit(main())
