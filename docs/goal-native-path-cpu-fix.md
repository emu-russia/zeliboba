# Native os0 module-path corruption: ARMv7 single-word MemU fix

The real native FAT open receives a valid `/kd/sysmem.skprx` input but the ARM path normalizer corrupted it before directory lookup. This is an architectural CPU defect, not a missing firmware filename or a directory service result to fabricate.

## Exact live instruction

`goal-arm-native-bootconfig-path-word.log` captures A32 `E4913004` at VA 51013744 (`LDR r3,[r1],#4`). Before execution R1=0007FC53 -> PA40319C53, Normal cacheable, SCTLR=20005805 (A=0). Bytes at the source are `2F 6B 64 2F 73 79 73 6D`, beginning `/kd/sysm`. Correct little-endian word is 2F646B2F. The old CPU produced R3=30736F2F (`/os0`) by rounding the source to 7FC50 and rotating the aligned word right 24 bits. R1 then advanced to 7FC57; no translation fault explained the mismatch.

The caller 51024854 is the genuine Thumb path normalizer; its 510248CE/510248E2 BLX calls enter the ARM strncpy helper 51013730. Therefore the actual failing word transfer is A32. The remaining T16 single-word address truncation violated the same ARMv7 MemU rule and was corrected under the same explicitly authorized scope.

## Independent unchanged native execution

A standalone CPU/Bus probe executes all unchanged code of the native 51024854 normalizer with a source at 7FC53, source-prefix bytes `os0`, and normal RAM buffers. It uses a byte-string oracle and preserves all firmware bytes. Both versions execute 1,475 instructions:

Before:
```text
first native E4913004 LDR: R1=0007FC53 exact-word=2F646B2F aligned-word=2F30736F SCTLR=00C50078
after native LDR: R3=30736F2F R1=0007FC57
RETURN after 1475 instructions prefix='/os0' suffix='skd/eysmkm.s'
byte oracle prefix='/kd/' suffix='sysmem.skprx': MISMATCH
```

After:
```text
first native E4913004 LDR: R1=0007FC53 exact-word=2F646B2F aligned-word=2F30736F SCTLR=00C50078
after native LDR: R3=2F646B2F R1=0007FC57
RETURN after 1475 instructions prefix='/kd/' suffix='sysmem.skprx'
byte oracle prefix='/kd/' suffix='sysmem.skprx': MATCH
```

Probe source: `build/research/zeliboba-native-path-copy-probe.cpp`; isolated fixed CPU object `build/research/zeliboba-native-path-arm-fixed.o`. These builds did not modify the shared build tree.

## Implementation and limits

`arm_core.cpp/.h` now provide a common private single-word MemU helper for A32, T16 and T32 LDR/STR. Aligned words retain a bus word transaction; unaligned Normal words transfer the exact four bytes, translating each byte independently. SCTLR.A faults irrespective of MMU enable; Device and Strongly-ordered translations reject unaligned transfer even with A clear. CPSR.E applies to aligned and unaligned words. Destination/writeback commit only after the transfer succeeds. Split stores retain the previous T32 per-byte fault behavior; atomic completion of a split transfer is not claimed.

Fetch, multiple-register, doubleword, exclusive and VFP paths retain their separate helpers and are outside this change. No firmware file, path, FAT lookup, module result, MMU mapping or host boot hook changed.

ARM ARM DDI0406C.d A8.8.63 and B2.4.5 specify LDR MemU byte addressing. The local primary manual `build/research/zeliboba-review-thumb-it/arm-ddi0406cd.txt` has LDR pseudocode at line 22307, unaligned MemU semantics around line 66642, and ARMv7 UnalignedSupport always true around line 143763. The removed rotate/aligned-store behavior is an older ISA rule; two existing A32 tests were corrected because they asserted that obsolete behavior.

## Validation

Focused CPU suite: 161 tests, zero failed cases or assertions (`goal-native-path-cpu-tests.log`). This adds four tests to the previous integrated 541, yielding an expected full suite of 545.

- Complete genuine A32 strncpy instruction sequence across every source/destination word alignment, checked against byte copying and NUL padding with guard bytes.
- A32 immediate/register/pre/post-index and T16 register/immediate/SP single-word paths.
- All three instruction sets: aligned/unaligned little/big-endian data, noncontiguous Normal pages, missing second page fault, preserved destination/writeback, SCTLR.A with MMU on/off, Device and Strongly-ordered faults.
- Previously fixed genuine Thumb inflater header and load-PC execution regressions remain passing.

The unchanged native inflater was independently rerun with the new common helper. It returns 658 after 28,130 instructions and outputs exactly the genuine first PT_LOAD of psp2bootconfig.elf: 1,624 bytes, SHA256 `7d92a892f9c72a9e15fff70b4ffcfc174984f03f802e710e87d861b8468a93c2`. Artifacts: `goal-native-path-deflate-isolated.log`, `goal-native-path-deflate-isolated-output.bin`.

The full ordinary boot after integration remains the next runtime check. This report proves correction of the observed CPU path corruption; it does not claim later dependencies or the guest logo have completed.
