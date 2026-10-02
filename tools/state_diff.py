#!/usr/bin/env python3
"""Compare two zeliboba save states and report the first differing section.

    python tools/state_diff.py a.state b.state

Save states are length-prefixed, named sections (see src/common/state.h). This
walks both files' section trees together and prints the path of the first
section whose payload differs, so a determinism failure points at the device
that was not restored instead of at a byte offset.
"""
import struct
import sys

MAGIC = b"ZLBSTATE"


def parse_sections(data, begin, end):
    out = []
    p = begin
    while p < end:
        if p + 4 > end or data[p:p + 4] != b"SEC ":
            return None
        p += 4
        if p + 2 > end:
            return None
        (name_len,) = struct.unpack_from("<H", data, p)
        p += 2
        if p + name_len > end:
            return None
        name = data[p:p + name_len].decode("utf-8", "replace")
        p += name_len
        if p + 8 > end:
            return None
        (length,) = struct.unpack_from("<Q", data, p)
        p += 8
        if p + length > end:
            return None
        out.append((name, p, p + length))
        p += length
    return out


def parse(data, begin, end):
    """Parse [begin, end) as a section sequence.

    A section payload may start with a small raw prologue (Vita writes the
    format version before its first nested section), so up to 16 leading bytes
    are reported as a `<prologue>` pseudo-section. Returns None when the range is
    a leaf payload.
    """
    direct = parse_sections(data, begin, end)
    if direct is not None:
        return direct
    for skip in (4, 8, 12, 16):
        if begin + skip >= end:
            break
        nested = parse_sections(data, begin + skip, end)
        if nested is not None:
            return [("<prologue>", begin, begin + skip)] + nested
    return None



def first_difference(a, b, abegin, aend, bbegin, bend, path):
    """Return a human readable description of the first difference, or None."""
    sa = parse(a, abegin, aend)
    sb = parse(b, bbegin, bend)
    if sa is None or sb is None:
        if a[abegin:aend] == b[bbegin:bend]:
            return None
        limit = min(aend - abegin, bend - bbegin)
        for i in range(limit):
            if a[abegin + i] != b[bbegin + i]:
                return f"{path}: leaf differs at +0x{i:X} (sizes {aend - abegin} vs {bend - bbegin})"
        return f"{path}: sizes differ ({aend - abegin} vs {bend - bbegin})"
    names_a = [name for name, _, _ in sa]
    names_b = [name for name, _, _ in sb]
    if names_a != names_b:
        return f"{path}: section names differ: {names_a} vs {names_b}"
    for (name, a0, a1), (_, b0, b1) in zip(sa, sb):
        difference = first_difference(a, b, a0, a1, b0, b1, f"{path}/{name}")
        if difference:
            return difference
    return None


def load(path):
    with open(path, "rb") as handle:
        raw = handle.read()
    if raw[:8] != MAGIC:
        raise SystemExit(f"{path}: not a save state")
    version, body_size = struct.unpack_from("<IQ", raw, 8)
    body = raw[20:20 + body_size]
    if len(body) != body_size:
        raise SystemExit(f"{path}: truncated")
    return version, body


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    version_a, a = load(sys.argv[1])
    version_b, b = load(sys.argv[2])
    if version_a != version_b:
        print(f"format versions differ: {version_a} vs {version_b}")
        return 1
    difference = first_difference(a, b, 0, len(a), 0, len(b), "")
    if difference is None:
        print("identical")
        return 0
    print(difference)
    return 1


if __name__ == "__main__":
    sys.exit(main())
