# zeliboba - does the fixed-heap init step get entered, and with what argument?
#
#   zeliboba.exe --script tools/dbg_kbl_heapstep.cmd --log off
#
# 0x4002C5F0 is the only static caller of 0x40031D58 (tools/kbl_xref.py), and
# 0x40031D58 is what calls the class constructor 0x40031410 and then
# FixedHeapInit 0x40031AC8.  The step is reached from the boot-init walk at
# 0x4002C594 only for cores with MPIDR id <= 2 (cmp r0,#2 / bhi), so the base
# register tells which core eats it.
core arm0
bp arm 0x4002C5F0
bp arm 0x40031D58
bp arm 0x40031AC8
bp arm 0x4002BBD6
runm 60000
core arm0
regs
runm 60000
regs
runm 60000
regs
runm 60000
regs
quit
