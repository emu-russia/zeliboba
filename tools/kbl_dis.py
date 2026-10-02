"""Linear Thumb-2 disassembly of a byte range of the KBL image.

zdis is convenient but its output addresses depend on --base; this prints the
guest address directly, which is what cross-referencing needs.

Usage: python tools/kbl_dis.py <addr> <count> [--arm]
"""
import sys
import capstone

IMAGE = r"..\_scratch\kbl.bin"
BASE = 0x40020000

addr = int(sys.argv[1], 0)
count = int(sys.argv[2], 0) if len(sys.argv) > 2 else 40
mode = capstone.CS_MODE_ARM if "--arm" in sys.argv else capstone.CS_MODE_THUMB

data = open(IMAGE, "rb").read()
off = addr - BASE
md = capstone.Cs(capstone.CS_ARCH_ARM, mode)
md.detail = False
for insn in md.disasm(data[off:off + count * 4], addr):
    print("  %08X  %-12s %s %s" % (insn.address, insn.bytes.hex(" "), insn.mnemonic, insn.op_str))
