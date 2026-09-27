; Round 47: proof that only the low-VA mappings are missing.
; Stop the KBL just before its first unmapped access (VA 0x40000), give the two
; page-table entries it never installs, and let it continue: the data abort
; handler at 0x16100 then runs and returns (LR=0x400321C2) instead of dying in
; the exception loop.
bp arm 0x400321B6
runm 2000000
core arm0
poke 0x80200058 0x0000065E 32
poke 0x80200100 0x0004065E 32
bpc 0x400321B6
runm 100000
core arm0
regs
faults
quit
