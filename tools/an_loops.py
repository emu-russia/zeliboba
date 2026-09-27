import sys

import capstone

b = open(r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2_dec\kernel_boot_loader.self.seg01', 'rb').read()
base = 0x40020000
md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB)
md.detail = True

lo, hi = 0x40020E00, 0x40021100
print('backward branches into [0x%X, 0x%X):' % (lo, hi))
for insn in md.disasm(b, base):
    if not insn.mnemonic.startswith('b'):
        continue
    if insn.mnemonic in ('bl', 'blx', 'bic', 'bics', 'bfi', 'bfc', 'bkpt'):
        continue
    for op in insn.operands:
        if op.type == capstone.arm.ARM_OP_IMM and lo <= op.imm < hi and op.imm <= insn.address:
            print('  %08X %-6s %s -> %08X' % (insn.address, insn.mnemonic, insn.op_str, op.imm))
