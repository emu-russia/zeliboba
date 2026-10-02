# Genuine 1.04 Syscon NEON command padding

Source is frozen; no shared build or full-machine run with the modified CPU was performed here. The complete isolated ARM suite passes **171 tests / 0 failures**, including four new execution regressions. Parent integration can proceed independently of this report.

## Native stop and exact bytes

The integrated 560-test binary halts ARM3 at Syscon VA **0x004F9BEA**, raw Thumb instruction **0xFFC70E1F**, with misleading reason `T32 NEON load/store`. Supplied `Out/fs_dec/os0/kd/syscon.elf` PT_LOAD0 begins at file offset0xA0, linked0x81000000; segment+0x1BEA is byte-exact **C7 FF 1F 0E = VMOV.I8 D16,#0xFF**. Its equivalent A32 instruction is0xF3C70E1F.

Fresh env-clean capture on private APFS image clone `goal-arm-syscon-neon-clone.img` reproduces the actual instruction before execution: ARM3 SYS Thumb, CPSR6000003F, FPEXC40000000, SCTLR20005805 (A=0), no live MMU fault. Code VA4F9BEA translates to PA40749BEA; buffer VAA6D48 translates to PA41035D48, both Normal cacheable. Actual inputs are R4=A6D48, R6=A6D49, R1=A6D58, R2=28, R7=3, R12=24, LR=4. Artifacts: `goal-arm-native-syscon-neon.py/.log/.json`. The first300k-slice window did not reach this worker; a second window reaches it. It is not an MMU, source-buffer, or Thumb-state error.

The unchanged block next executes **F949070F = VST1.8 {D16},[R9]** at0x4F9C02, then **F943070D = VST1.8 {D16},[R3]!** at0x4F9C26/2C. Those instructions were also missing. The decoder sent FF to scalar signed-load decoding and would do the same for F9 structure stores.

## Architectural basis and correction

Primary source: [ARM Architecture Reference Manual ARMv7-A/R, DDI0406C.d](https://developer.arm.com/documentation/ddi0406/cd), cached full text `build/research/zeliboba-review-thumb-it/arm-ddi0406cd.txt`.

- **A7.4, A7-259**, local line14951 onward: Thumb Advanced SIMD data-processing prefix EF/FF corresponds to A32 F2/F3, retaining U/immediate-high bit in its proper position.
- **A7.4.6, A7-267..269**, local15426..15604: modified-immediate allocation and `AdvSIMDExpandImm`; shifted/repeated integer bytes, bit-per-byte I64 constants, and repeated F32 constants. Related VORR/VBIC/VMVN immediates are separate operations.
- **A8.8.340, A8-937..938**, local46697 onward: VMOV T1/A1, exact D/Q destination, Q requires even D index, one/two register writes, no condition/FPSCR changes.
- **A7.7, A7-273**, local15797 onward: Thumb F9 structure transfer prefix maps to A32 F4 while retaining all variable fields. Valid scalar signed loads/hints have a different fixed bit and stay on their existing path.
- **A8.8.405, A8-1065..1066**, local52860 onward: VST1 multiple-single-elements stores all elements of one to four consecutive exact D registers, applies endian interpretation per element, explicit alignment and SCTLR.A checks, and commits writeback after successful transfers. Rm15 means no writeback; Rm13 adds the list byte count; other Rm uses the old register value. Eight-byte elements use two MemU words.

`arm_core.cpp` now normalizes only the fixed Thumb class bits and executes VMOV modified immediates and the reached VST1 multiple-single-elements store family. It rejects invalid Q/list/alignment encodings before register or memory changes. UNPREDICTABLE zero encodings in specified shifted-immediate forms deterministically fault. The store path uses the existing per-access translation/MemU helpers, with a local endian/alignment-safe halfword path, and preserves the base on a fault. `arm_disasm.cpp` names the genuine I8 VMOV and VST1 forms and uses the same Thumb class normalization. Existing Advanced SIMD control coverage is unchanged; VLD and interleaved/single-lane structure families remain explicit unsupported faults.

## Meaningful execution verification

The new unchanged native Syscon regression executes **linked81001BEA..81001CA0** with the captured live register inputs, relocating only code and RAM addresses. It reaches native81001CA2, sets D16 to all ones, executes the actual vector stores plus scalar tail, and fills **only buffer bytes20..47** withFF. Header/neighbor bytes and every other D register remain intact; FPSCR remains unchanged.

Three further regression cases cover:

- Independent expected constants for every VMOV immediate expansion form, including immediate-high bit1/0, A32/T32, odd/high D and high Q destinations, neighboring register preservation and invalid/related encodings. Thumb register VORR and VEOR prove U0/U1 normalization too.
- VST1 sizes8/16/32/64, list counts1..4 ending atD31, per-element little/big endian, unaligned Normal RAM, no/register/immediate writeback including Rm=Rn, and guard bytes.
- Nonadjacent physical pages; a genuine second-page permission/translation failure with no base update; SCTLR.A and explicit alignment faults; unaligned Device/Strongly-ordered rejection; byte alignment1 under A=1; invalid register/alignment encodings; skipped IT conditions with no vector or memory update.

Old560 archive with the new tests fails all four new cases (**5,856 assertion failures**) in `goal-native-syscon-neon-before.log`. The corrected source first passes all nine NEON cases, then the full isolated ARM suite **171/0** in `goal-native-syscon-neon-cpu-tests.log`. Only `src/cpu/arm/arm_core.cpp`, `src/cpu/arm/arm_disasm.cpp`, and `tests/test_arm.cpp` changed. Integrated count increment is **+4**; with parent CDRAM1 and EMC7 the expected full suite is572. Genuine hardware SPI/OLED progress after the fix remains for the integrated native run to establish.
