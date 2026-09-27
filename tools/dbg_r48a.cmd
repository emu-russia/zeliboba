; Round 47: the exact fault chain of kernel_boot_loader.
; Run: .\build\bin\zeliboba.exe --script <abs path to this file>
; Machine slices (runm) are required: `run` never lets the other three cores
; past the four-core barrier, and the KBL's console output comes from core 0
; only after the barrier.
bp arm 0x1610C
runm 2000000
core arm0
faults all
regs
console
quit
