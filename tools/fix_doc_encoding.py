"""Normalise a docs/*.md file that was written in more than one encoding.

History: docs/KBL.md and docs/STAGE1_HANDOVER.md ended up with blocks of cp1251
text inside an otherwise UTF-8 file (one-off patch scripts opened the file with
`encoding='utf-8'` and rewrote it, while an earlier tool had saved part of it as
cp1251).  A single mixed file cannot be decoded with either codec, and my own
`read` tooling refused it.

Line level detection alone is ambiguous - cp1251 bytes `D0 B0` are valid UTF-8
for U+0430 - so the blocks are located by the lines that are *not* valid UTF-8
and the whole block between the first and the last such line is then decoded as
cp1251.  The script prints how many mixed lines it found and how many mojibake
bigrams the block contains: a block with zero mojibake hits really was cp1251.

    python tools/fix_doc_encoding.py docs/KBL.md            # report only
    python tools/fix_doc_encoding.py docs/KBL.md --write    # rewrite as UTF-8
"""
import re
import sys
from pathlib import Path

# Bigrams that only appear when UTF-8 Cyrillic is decoded as cp1251.
MOJIBAKE = ("Р°", "Рµ", "Рѕ", "РЅ", "СЃ", "СЂ", "С‚", "РІ", "Р»", "Рє", "Рґ",
            "Рј", "Рї", "Сѓ", "С‡", "С‹", "Р·", "СЏ", "Р¶", "С…", "Р±", "С—")


def split_lines(raw: bytes):
    out = []
    start = 0
    for i, b in enumerate(raw):
        if b == 0x0A:
            end = i
            if end > start and raw[end - 1] == 0x0D:
                end -= 1
            out.append(raw[start:end])
            start = i + 1
    if start < len(raw):
        out.append(raw[start:])
    return out


def valid_utf8(seg: bytes) -> bool:
    try:
        seg.decode("utf-8")
        return True
    except UnicodeDecodeError:
        return False


def convert(path: Path, write: bool = False) -> dict:
    raw = path.read_bytes()
    lines = split_lines(raw)
    bad = [i for i, seg in enumerate(lines)
           if any(b >= 0x80 for b in seg) and not valid_utf8(seg)]
    report = {"file": path.name, "lines": len(lines), "non_utf8": len(bad), "block": None}
    if not bad:
        return report

    lo, hi = bad[0], bad[-1]
    block = "\n".join(seg.decode("cp1251") for seg in lines[lo:hi + 1])
    report["block"] = (lo + 1, hi + 1)
    report["block_mojibake_hits"] = sum(
        len(re.findall(re.escape(sig), block)) for sig in MOJIBAKE)

    text = []
    for i, seg in enumerate(lines):
        enc = "cp1251" if lo <= i <= hi else "utf-8"
        try:
            text.append(seg.decode(enc))
        except UnicodeDecodeError:
            text.append(seg.decode("cp1251", errors="replace"))
    out = "\n".join(text)
    if not out.endswith("\n"):
        out += "\n"
    report["out_bytes"] = len(out.encode("utf-8"))
    if write:
        path.write_bytes(out.encode("utf-8"))
    return report


def main(argv):
    write = "--write" in argv
    names = [a for a in argv if not a.startswith("--")]
    if not names:
        print(__doc__)
        return 2
    for name in names:
        print(convert(Path(name), write))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
