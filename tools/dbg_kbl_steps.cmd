# zeliboba - where the boot init steps are entered.
#
#   zeliboba.exe --script tools/dbg_kbl_steps.cmd --log off
#
# 0x4002C602 calls the object-manager builder 0x4002BAF8; 0x4002C5F0 calls the
# fixed-heap init step 0x40031D58 (see tools/kbl_xref.py callers).  Both are
# reached only from the load-time relocation table, so this shows the step order.
bp arm 0x4002C602
bp arm 0x4002C5F0
bp arm 0x40031D58
bp arm 0x40031AC8
bp arm 0x4002BBD6
runm 20000
bpl
regs
history on
runm 200
history arm 120
quit
