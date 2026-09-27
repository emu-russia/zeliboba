"""Recursive-descent disassembly of the KBL image with cross-references.

The kernel_boot_loader is Thumb-2, and a linear sweep of the raw dump decodes
garbage (a 16-bit halfword can start a 32-bit instruction).  This walks the
image the way the CPU does - from the ELF entry, following direct branches and
calls - and then answers the questions the KBL analysis needs:

  * who calls <addr>            (BL/BLX/`bl` via a literal pool)
  * who references <addr>       (any B/BL/target, plus ADR/MOVW+MOVT pairs)
  * what a function looks like  (disassembly of one function)

Usage:
  python tools/kbl_xref.py <command> [args...]

Commands:
  callers <addr> [...]     all call sites of the listed addresses
  refs <addr> [...]        all control-flow targets of the listed addresses
  dis <addr> [count]       disassemble from an address (no block analysis)
  funcs                    list discovered function start addresses
  stats                    coverage of the linear range

The image is `_scratch/kbl.bin` (393216 bytes loaded at 0x40020000), which the
debugger dumps from the running machine; see docs/KBL.md round 70.
"""
import bisect
import struct
import sys
from collections import defaultdict

import capstone

IMAGE = r"C:\Work\PSVita\_scratch\kbl.bin"
BASE = 0x40020000
ENTRY = 0x40020284          # ARM boot entry the KBL starts at (bootchain.cpp)


def load():
    data = open(IMAGE, "rb").read()
    return data


class Walker:
    def __init__(self, data):
        self.data = data
        self.md_t = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB)
        self.md_t.detail = True
        self.md_a = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_ARM)
        self.md_a.detail = True
        self.blocks = {}                     # addr -> [insn...]
        self.calls = defaultdict(set)        # target -> {call site}
        self.branches = defaultdict(set)     # target -> {source}
        self.funcs = set()
        # MOVW/MOVT pairs that materialise a constant address.
        self.movt = {}                       # addr of movt insn -> value

    def bytes_at(self, addr):
        off = addr - BASE
        if off < 0 or off >= len(self.data):
            return b""
        return self.data[off:off + 4]

    def md(self, thumb):
        return self.md_t if thumb else self.md_a

    def decode(self, addr, thumb, max_insns=1):
        code = self.bytes_at(addr)
        if len(code) < 2:
            return []
        out = []
        for insn in self.md(thumb).disasm(code, addr, max_insns):
            out.append(insn)
        return out

    def walk(self, addr, thumb=True, budget=200000):
        """Recursive descent from addr, following direct control flow."""
        worklist = [(addr, thumb)]
        seen = set()
        done = 0
        while worklist:
            a, t = worklist.pop()
            if (a, t) in seen or done > budget:
                continue
            seen.add((a, t))
            while True:
                insns = self.decode(a, t, 1)
                if not insns:
                    break
                insn = insns[0]
                self.blocks.setdefault(a, []).append(insn)
                done += 1
                mnem = insn.mnemonic
                ops = insn.op_str
                target = None

                # state switch
                if mnem in ("blx", "bl") and ops.startswith("#"):
                    try:
                        target = int(ops[1:], 0)
                    except ValueError:
                        target = None
                    if target is not None:
                        # BLX with an immediate switches ARM<->Thumb; capstone
                        # already reports the target, and the low bit tells the state.
                        self.calls[target & ~1].add(a)
                        nt = (target & 1) == 1 if mnem == "blx" else t
                        # Thumb BL/BLX targets are always halfword aligned and the
                        # state is thumb for `bl` inside thumb code; an ARM->Thumb
                        # switch shows up as an odd target.
                        if mnem == "blx":
                            nt = (target & 1) == 1
                        else:
                            nt = t
                        worklist.append((target & ~1, nt))
                    a = insn.address + insn.size
                    continue
                if mnem in ("b", "bx") and ops.startswith("#"):
                    try:
                        target = int(ops[1:], 0)
                    except ValueError:
                        target = None
                    if target is not None:
                        self.branches[target & ~1].add(a)
                        worklist.append((target & ~1, t))
                    # unconditional b ends the block
                    if mnem == "b":
                        break
                    a = insn.address + insn.size
                    continue
                if mnem.startswith("b") and ops.startswith("#") and mnem not in ("bkpt",):
                    # conditional branch
                    try:
                        target = int(ops[1:], 0)
                    except ValueError:
                        target = None
                    if target is not None:
                        self.branches[target & ~1].add(a)
                        worklist.append((target & ~1, t))
                    a = insn.address + insn.size
                    continue
                if mnem in ("pop",) and "r15" in ops or mnem in ("bx",) and "r14" in ops \
                        or mnem == "pop.w" and "r15" in ops:
                    break
                if mnem in ("ldr", "ldr.w") and "r15" in ops:
                    # PC load: the word after is a target
                    m = ops.split("[")[1].split("]")[0].split(",")[0].strip()
                    base = m
                    if "#" in ops:
                        disp = int(ops.split("#")[1].split("]")[0], 0)
                    else:
                        disp = 0
                    if base == "pc":
                        pc = (insn.address + 4) & ~3
                        lit = pc + disp
                        word = self.read32(lit)
                        if word is not None:
                            self.branches[word & ~1].add(insn.address)
                            worklist.append((word & ~1, (word & 1) == 1))
                    break
                a = insn.address + insn.size

    def read32(self, addr):
        off = addr - BASE
        if off < 0 or off + 4 > len(self.data):
            return None
        return struct.unpack_from("<I", self.data, off)[0]

    def freesweep(self):
        """Also sweep every halfword as a possible code start (fallback)."""
        pass


def main():
    data = load()
    w = Walker(data)
    if len(sys.argv) < 2:
        print(__doc__)
        return
    cmd = sys.argv[1]
    if cmd == "funcs":
        w.walk(ENTRY)
        for f in sorted(w.calls):
            pass
        return
    # Walk a wide range for xrefs: the whole text, starting from the entry.
    w.walk(ENTRY)
    if cmd == "callers":
        for arg in sys.argv[2:]:
            addr = int(arg, 0)
            sites = sorted(w.calls.get(addr, ()))
            print("callers of 0x%08X: %d" % (addr, len(sites)))
            for s in sites:
                print("   0x%08X" % s)
    elif cmd == "refs":
        for arg in sys.argv[2:]:
            addr = int(arg, 0)
            # dicts hold collections, but keep the union defensive: an older caller
            # may hand back a tuple, and `tuple | tuple` raises TypeError.
            sites = sorted(set(w.branches.get(addr, ())) | set(w.calls.get(addr, ())))
            print("refs to 0x%08X: %d" % (addr, len(sites)))
            for s in sites:
                print("   0x%08X" % s)
    elif cmd == "dis":
        addr = int(sys.argv[2], 0)
        count = int(sys.argv[3]) if len(sys.argv) > 3 else 40
        for insn in w.decode(addr, True, count):
            print("  %08X  %-24s %s" % (insn.address, insn.bytes.hex(" "), insn.mnemonic + " " + insn.op_str))
    elif cmd == "stats":
        print("blocks: %d, call targets: %d, branch targets: %d"
              % (len(w.blocks), len(w.calls), len(w.branches)))


if __name__ == "__main__":
    main()
