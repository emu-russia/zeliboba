# Native bootconfig DEFLATE: proved CPU address defect

Source: firmware 1.04 `psp2bootconfig.skprx` first authenticated section. Its 733-byte decrypted input starts `78 9C CD 93 5F 48 53 51` and is identical through the ARM VA, ARM PA and MeP PA views. SHA256: `3f2043fbfda85346d7c9c638dd30cf081e521a3c31742a0eb030c879e95921bd`.

At NSKBL `0x510172DC`, native zlib decoder `0x51024E18` receives R0=`0xCC000`, R1=`0x658`, R2=`0x5113C000`, R3=0. SCTLR=`0x20005805` has alignment checking clear. The source VA translates identically; output VA maps to PA `0x40373000`.

The raw inflater instruction at `0x51024E9E` is `F858 9B02`, `LDR r9,[r8],#2`, with R8=`0x5113C002`. Correct bytes produce `0x485F93CD`; the former T32 decoder rounded the address down and loaded `0x93CD9C78`. The following ROR/UBFX identifies the wrong block type 0 instead of dynamic type 2, causing native return `0x80560100`, mapped by its caller to `0x800F0516`.

Primary ARM ARM DDI0406C.d A8.8.63 (LDR Thumb), A8 STR immediate/register, B2.4.5 MemU and D16.7.30 specify byte-exact ARMv7 unaligned accesses with SCTLR.A=0, four independently translated bytes, and alignment faults with A=1. Device/Strongly-ordered translations remain alignment checked.

The fix is confined to T32 single word load/store paths (immediate, register, indexed, literal). It preserves other transfer paths. Regressions execute the genuine inflater header sequence; all supported addressing forms; noncontiguous Normal pages; A=1 with MMU both off/on; Device/Strongly-ordered faults and unchanged writeback/memory; and CPSR.E byte order.

Unchanged native inflater isolated validation: before fix returns `0x80560100` after 100 instructions. After fix returns `0x658` after 28,130 instructions. Its 1,624 output bytes equal the genuine decoded ELF PT_LOAD0 (file `Out/fs_dec/os0/psp2bootconfig.elf`, offset A0, length 658) byte for byte. Output SHA256: `7d92a892f9c72a9e15fff70b4ffcfc174984f03f802e710e87d861b8468a93c2`.

Artifacts: `goal-native-deflate-isolated-trace.log`, `goal-native-deflate-isolated-fixed-trace.log`, `goal-native-deflate-isolated-output.bin`, `goal-native-deflate-arm-tests.log`. Independent fixture source: `build/research/zeliboba-native-deflate-cpu-probe.cpp`. No firmware decompressor or output bytes were patched; all execution used genuine NSKBL instructions and the captured guest stream. Integrated full-boot verification remains with the parent agent.
