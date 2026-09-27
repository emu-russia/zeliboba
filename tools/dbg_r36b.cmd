stage kbl
core arm
poke 0x40020000 0xE1600070
reg PC 0x40020000
reg MVBAR 0x40021000
reg SCR 0
reg CPSR 0x1D3
regs
step
regs
quit
