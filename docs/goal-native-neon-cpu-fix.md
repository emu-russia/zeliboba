# Native ThreadMgr NEON logical-register fault

Firmware 1.04 `Out/fs_dec/os0/kd/threadmgr.elf` PT_LOAD0 starts at file offset
`0xA0`, linked VA `0x81000000`. File offset `0xA8` contains `F2201110`,
`VORR D1,D0,D0` (the VMOV alias), with Q=0. The native module relocation
places it at VA `0x004A0008`. All four cores enter the genuine ThreadMgr
extension at `0x004A19E4`, call `0x004A0000`, load D0 from `0x004A0048`,
and stopped at this instruction with D0=`0x7FF8DEAD7F80DEAD`.

Evidence: `goal-native-memu-full.log`, `goal-arm-native-threadmgr-initializer.bin`,
and `goal-arm-native-threadmgr-extension.bin`. ThreadMgr text is `0x004A0000`
with size `0x2DEB0`; its BSS is separately relocated at `0x00435000`, size `0xE8`.

ARM ARM DDI0406C.d A8.8.361 (A8-977..978) requires even D-register indices
only for Q=1. Q=0 uses the exact D index, including D1 and D31. The emulator
had reversed this check and rounded all D indices down to even indices.
The local primary manual is `build/research/zeliboba-review-thumb-it/arm-ddi0406cd.txt`:
VORR encoding starts at line 48544, the alignment/index rules at line 48565.

The existing logical family also had two directly verified mistakes:
Table A7-9 (local line 15020 onward) encodes EOR/BSL/BIT/BIF as opc1=1,
opc2=1,U=1, rather than opc1=3,U=0. A8.8.291 (A8-843..844, local line 41850
onward) defines VBSL's mask as the old destination value. These are corrected
in the same execution path without adding other NEON operation families.

## Verification

The new native initializer regression executes all unchanged instructions
from linked `0x81000000` through `BX LR` at `0x81000044`. Only the relocated
data-pointer literal is adjusted. It checks D0/D1 and Q2..Q15 against the
genuine loaded pattern, preserves Q1, CPSR and FPSCR, and returns to LR.

Two further tests cover all eight logical instructions using independent
per-bit truth tables, odd and high D registers, high Q registers, operand
overlap, neighboring-register preservation, and architecturally invalid odd
Q indices. Encodings from the previously misidentified opc1=3 family fault
without a spurious logical write.

Before the fix, all three new cases fail (103 assertion failures):
`goal-native-neon-cpu-before.log`. With the fixed CPU object, the complete
isolated ARM suite passes **164 tests / 0 failures**:
`goal-native-neon-cpu-tests.log`. The expected integrated suite count is
**548**, up from 545. No shared build or full boot was run for this report.

Sources: `src/cpu/arm/arm_core.cpp`, `src/cpu/arm/arm_disasm.cpp`,
`tests/test_arm.cpp`. The disassembler uses the corrected U/opc1 selector too;
the logical-family tests check its exact mnemonic and high D/Q register
operands for every valid layout. Other unsupported Advanced SIMD instructions
remain explicit faults.
