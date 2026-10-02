"""Recursive-descent analysis of a raw ARM/Thumb image with cross-references.

The firmware images here mix ARM and Thumb, so a linear sweep decodes garbage (a
16-bit halfword can start a 32-bit instruction).  This walks the image the way the
CPU does - from an entry point, following direct branches and calls, switching to
ARM when a `blx` immediate says so - and then answers:

  * callers <addr> ...   who calls the address (BL/BLX sites)
  * refs <addr> ...      any control-flow target or literal-pool word equal to it
  * consts <value> ...   instructions whose immediate/operand equals the value
                         (e.g. `consts 0x2430` finds `add.w rX,rY,#0x2430`)
  * dis <addr> [count]   disassemble from an address
  * funcs                discovered function entries
  * stats                what was walked

Usage:
  python tools/arm_xref.py [--image FILE] [--base 0x...] [--entry 0x...] [--arm]
                           <command> [args...]

Defaults target the decoded NSKBL image (Vita_104_Firmware/Out/SLB2_dec/nsbl.bin,
base 0x51000000, entry 0x51000100 - the reset vector of its own vector table).

capstone is required (it is installed in this environment; `import capstone`).
"""
import argparse
import struct
import sys
from collections import defaultdict

try:
    import capstone
except ImportError:                                    # pragma: no cover
    print("capstone is required: pip install capstone", file=sys.stderr)
    sys.exit(2)

DEFAULT_IMAGE = r"..\Vita_104_Firmware\Out\SLB2_dec\nsbl.bin"
DEFAULT_BASE = 0x51000000
DEFAULT_ENTRY = 0x51000100


class Walker:
    def __init__(self, data, base, entry, start_arm=False):
        self.data = data
        self.base = base
        self.md = {
            False: capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB),
            True: capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_ARM),
        }
        for engine in self.md.values():
            engine.detail = True
        self.insns = {}                    # addr -> (size, mnemonic, op_str, is_arm)
        self.calls = defaultdict(set)      # target -> {call site}
        self.refs = defaultdict(set)       # value -> {insn address}
        self.funcs = set()
        self.pending = [(entry, start_arm)]

    def bytes_at(self, addr, count=4):
        off = addr - self.base
        if off < 0 or off >= len(self.data):
            return b""
        return self.data[off:off + count]

    def add_ref(self, value, at):
        if self.base <= value < self.base + len(self.data):
            self.refs[value].add(at)

    def walk(self, limit=400000):
        while self.pending and len(self.insns) < limit:
            addr, is_arm = self.pending.pop()
            if addr in self.insns:
                continue
            code = self.bytes_at(addr, 4)
            if len(code) < 4 and not is_arm:
                code = self.bytes_at(addr, 2)
            if not code:
                continue
            found = None
            for insn in self.md[is_arm].disasm(code, addr, count=1):
                found = insn
                break
            if found is None:
                continue
            size = found.size
            self.insns[addr] = (size, found.mnemonic, found.op_str, is_arm)
            self._follow(found, addr, is_arm)
        return self

    def _follow(self, insn, addr, is_arm):
        groups = insn.groups
        mnemonic = insn.mnemonic
        # Literal pool reads: ldr rX, [pc, #imm] -> record the word as a reference.
        for op in insn.operands:
            if op.type == capstone.arm.ARM_OP_MEM and op.mem.base == capstone.arm.ARM_REG_PC:
                target = addr + 4 + op.mem.disp if not is_arm else addr + 8 + op.mem.disp
                word = self.bytes_at(target & ~3, 4)
                if len(word) == 4:
                    self.add_ref(struct.unpack("<I", word)[0], addr)
            if op.type == capstone.arm.ARM_OP_IMM:
                value = op.imm & 0xFFFFFFFF
                self.add_ref(value, addr)

        is_branch = (capstone.arm.ARM_GRP_JUMP in groups or capstone.arm.ARM_GRP_CALL in groups)
        # Fall through to the next instruction unless this one always transfers control.
        # NOTE: only a plain `b` is unconditional - a conditional branch (`bne`, `beq`)
        # must keep its fall-through, or whole blocks after a loop test are lost.
        unconditional = mnemonic == "b" or mnemonic.startswith("b.w")
        if not (is_branch and unconditional) and not mnemonic.startswith(("bx", "pop", "ret")):
            self.pending.append((addr + insn.size, is_arm))

        if not is_branch:
            return
        # Direct branch: follow the target in the same mode; blx switches mode.
        for op in insn.operands:
            if op.type != capstone.arm.ARM_OP_IMM:
                continue
            target = op.imm & 0xFFFFFFFF
            if capstone.arm.ARM_GRP_CALL in groups:
                self.calls[target].add(addr)
                self.funcs.add(target)
            switch = mnemonic.startswith("blx")
            self.pending.append((target, (not is_arm) if switch else is_arm))


def load_image(path):
    with open(path, "rb") as handle:
        return handle.read()


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default=DEFAULT_IMAGE)
    ap.add_argument("--base", type=lambda v: int(v, 0), default=DEFAULT_BASE)
    ap.add_argument("--entry", type=lambda v: int(v, 0), default=DEFAULT_ENTRY)
    ap.add_argument("--arm", action="store_true", help="the entry is ARM code")
    ap.add_argument("command")
    ap.add_argument("args", nargs="*")
    args = ap.parse_args(argv)

    walker = Walker(load_image(args.image), args.base, args.entry, args.arm).walk()

    def addrs(values):
        return [int(v, 0) for v in values]

    if args.command == "callers":
        for target in addrs(args.args):
            sites = sorted(walker.calls.get(target, ()))
            print(f"callers of 0x{target:08X}: {len(sites)}")
            for site in sites:
                size, mnemonic, op_str, is_arm = walker.insns.get(site, (0, "?", "?", False))
                print(f"  0x{site:08X}  {mnemonic} {op_str}{'' if is_arm else '  (thumb)'}")
    elif args.command == "refs":
        for target in addrs(args.args):
            sites = sorted(walker.refs.get(target, ()))
            print(f"references to 0x{target:08X}: {len(sites)}")
            for site in sites:
                size, mnemonic, op_str, is_arm = walker.insns.get(site, (0, "?", "?", False))
                print(f"  0x{site:08X}  {mnemonic} {op_str}{'' if is_arm else '  (thumb)'}")
    elif args.command == "consts":
        # Linear scan of both modes: a search must not depend on the walk's reach.
        wanted = {int(v, 0) & 0xFFFFFFFF for v in args.args}
        hits = defaultdict(list)
        for mode, step, engine in ((False, 2, walker.md[False]), (True, 4, walker.md[True])):
            addr = walker.base if step == 2 else (walker.base + 3) & ~3
            while addr < walker.base + len(walker.data):
                code = walker.bytes_at(addr, 4)
                if len(code) < 4 and not mode:
                    code = walker.bytes_at(addr, 2)
                if not code:
                    break
                insn = next(engine.disasm(code, addr, count=1), None)
                if insn is not None and insn.mnemonic not in ("udf", ".byte", "b"):
                    for op in insn.operands:
                        value = None
                        if op.type == capstone.arm.ARM_OP_IMM:
                            value = op.imm & 0xFFFFFFFF
                        elif op.type == capstone.arm.ARM_OP_MEM:
                            # `str rX,[rY,#0x2430]` carries the field as a displacement.
                            value = op.mem.disp & 0xFFFFFFFF
                        if value in wanted:
                            hits[value].append((addr, insn.mnemonic, insn.op_str, mode))
                            break
                addr += step
        for value in sorted(hits):
            print(f"immediate 0x{value:08X}: {len(hits[value])} site(s)")
            for addr, mnemonic, op_str, mode in sorted(hits[value])[:40]:
                print(f"  0x{addr:08X}  {mnemonic} {op_str}  ({'arm' if mode else 'thumb'})")
    elif args.command == "dis":
        addr = addrs(args.args[:1])[0]
        count = int(args.args[1], 0) if len(args.args) > 1 else 16
        is_arm = args.arm
        printed = 0
        while printed < count:
            size, mnemonic, op_str, mode = walker.insns.get(addr, (0, None, None, is_arm))
            if mnemonic is None:
                code = walker.bytes_at(addr, 4)
                insns = list(walker.md[is_arm].disasm(code, addr, count=1))
                if not insns:
                    code = walker.bytes_at(addr, 2)
                    insns = list(walker.md[False].disasm(code, addr, count=1))
                if not insns:
                    break
                insn = insns[0]
                print(f"  0x{addr:08X}  {insn.mnemonic} {insn.op_str}")
                addr += insn.size
            else:
                print(f"  0x{addr:08X}  {mnemonic} {op_str}")
                addr += size
            printed += 1
    elif args.command == "pool":
        # Who loads this *data* address as a literal (`ldr rX,[pc,#imm]`)?  Scans every
        # 2-byte offset in both modes - the walk above stops at indirect calls, and the
        # question here is exactly "which code touches this table".
        wanted = set(addrs(args.args))
        hits = []
        for mode, step, engine in ((False, 2, walker.md[False]), (True, 4, walker.md[True])):
            addr = walker.base if step == 2 else (walker.base + 3) & ~3
            while addr < walker.base + len(walker.data):
                code = walker.bytes_at(addr, 4)
                if len(code) < 4 and not mode:
                    code = walker.bytes_at(addr, 2)
                if not code:
                    break
                insn = next(engine.disasm(code, addr, count=1), None)
                if insn is not None:
                    for op in insn.operands:
                        if op.type == capstone.arm.ARM_OP_MEM and op.mem.base == capstone.arm.ARM_REG_PC:
                            target = (addr + 4 + op.mem.disp) if not mode else (addr + 8 + op.mem.disp)
                            if (target & ~3) in wanted:
                                hits.append((addr, insn.mnemonic, insn.op_str, mode, target & ~3))
                            break
                addr += step
        for value in sorted(wanted):
            found = [h for h in hits if h[4] == value]
            print(f"literal loads of 0x{value:08X}: {len(found)}")
            for addr, mnemonic, op_str, mode, target in found[:40]:
                print(f"  0x{addr:08X}  {mnemonic} {op_str}  ({'arm' if mode else 'thumb'})")
    elif args.command == "pcrange":
        # Which instructions compute an address in [lo,hi) from the PC?  This is how the
        # images reach a table the linker placed nearby (`adr`, or `ldr rX,[pc,#imm]`
        # with a *page* base then an offset), which a plain pointer search misses.
        lo, hi = addrs(args.args[:2])
        for mode, step, engine in ((False, 2, walker.md[False]), (True, 4, walker.md[True])):
            addr = walker.base if step == 2 else (walker.base + 3) & ~3
            while addr < walker.base + len(walker.data):
                code = walker.bytes_at(addr, 4)
                if len(code) < 4 and not mode:
                    code = walker.bytes_at(addr, 2)
                if not code:
                    break
                insn = next(engine.disasm(code, addr, count=1), None)
                if insn is not None and insn.mnemonic.startswith(("adr", "add", "ldr", "sub")):
                    for op in insn.operands:
                        if op.type == capstone.arm.ARM_OP_MEM and op.mem.base == capstone.arm.ARM_REG_PC:
                            target = (addr + 4 + op.mem.disp) if not mode else (addr + 8 + op.mem.disp)
                            if lo <= target < hi:
                                print(f"  0x{addr:08X}  {insn.mnemonic} {insn.op_str} -> "
                                      f"0x{target:08X}  ({'arm' if mode else 'thumb'})")
                            break
                elif insn is not None and insn.mnemonic.startswith("adr"):
                    for op in insn.operands:
                        if op.type == capstone.arm.ARM_OP_IMM and lo <= (op.imm & 0xFFFFFFFF) < hi:
                            print(f"  0x{addr:08X}  {insn.mnemonic} {insn.op_str}  "
                                  f"({'arm' if mode else 'thumb'})")
                            break
                addr += step
    elif args.command == "funcs":
        for addr in sorted(walker.funcs):
            print(f"0x{addr:08X}")
    elif args.command == "stats":
        print(f"image {args.image}: {len(walker.data)} bytes at 0x{args.base:08X}")
        print(f"instructions walked: {len(walker.insns)}")
        print(f"functions discovered: {len(walker.funcs)}")
        print(f"distinct reference values: {len(walker.refs)}")
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
