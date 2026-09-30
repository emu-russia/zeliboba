"""Check the documentation tree (docs/*.md).

Run:  python tools/check_docs.py

Checks:
* every doc is valid UTF-8 (no U+FFFD, no mixed cp1251 blocks - the failure mode
  that `tools/fix_doc_encoding.py` repairs);
* every `docs/<name>.md` a document refers to exists (catches links to removed
  files);
* every `_scratch/...` or `tools/...` path a document names exists (catches the
  "the artifact was never committed" class of stale reference).

Exit code is non-zero when anything is wrong.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
WORKSPACE = ROOT.parent

DOC_REF = re.compile(r"docs[/\\]([A-Za-z0-9_\-]+\.md)")
PATH_REF = re.compile(r"`((?:_scratch|tools)[/\\][A-Za-z0-9_\-./\\]+)`")


def main() -> int:
    problems = []
    docs = sorted(DOCS.glob("*.md"))
    for md in docs:
        raw = md.read_bytes()
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            problems.append(f"{md.name}: не UTF-8: {exc}")
            continue
        if "\ufffd" in text:
            problems.append(f"{md.name}: содержит U+FFFD (повреждённая кодировка)")
        for name in set(DOC_REF.findall(text)):
            if not (DOCS / name).exists():
                problems.append(f"{md.name}: ссылка на несуществующий docs/{name}")
        for ref in set(PATH_REF.findall(text)):
            target = ref.replace("\\", "/")
            if not (ROOT / target).exists() and not (WORKSPACE / target).exists():
                problems.append(f"{md.name}: ссылка на несуществующий путь {target}")

    print(f"docs/*.md проверено: {len(docs)}")
    if problems:
        print(f"замечаний: {len(problems)}")
        for p in sorted(set(problems)):
            print("  -", p)
        return 1
    print("замечаний нет")
    return 0


if __name__ == "__main__":
    sys.exit(main())
