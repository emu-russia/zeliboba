# Native Syscon checksum SIMD execution — frozen CPU correction

## Reached defect and provenance

The ordinary 599-test native run reached SceSyscon's real response checksum block and halted ARM0 on Thumb-2 instruction `F962070D` at VA `004F8B56`, bytes `62 F9 0D 07`: `VLD1.8 {d16},[r2]!`. The original 1.04 `Out/fs_dec/os0/kd/syscon.elf` has the same bytes at first PT_LOAD file offset `A0+B56`, linked VA `81000B56`. The register dump has SYS/Thumb CPSR `600000BF`, SP `005840F8`, R2/R7 `000A6D78`, and R0 `8`; this is the inline packet checksum computation. The actual captured response bytes are not available, so the isolated probes use explicitly supplied test packets rather than claiming a replay of the live response.

Runtime evidence: `goal-native-early-ready-full.log` and `goal-arm-native-syscon-oled-early-ready.log/.json`. Native disassembly: `goal-native-syscon-checksum-neon-dis.txt`.

## Architectural basis

Primary source: [ARM Architecture Reference Manual ARMv7-A and ARMv7-R, DDI0406C.d](https://developer.arm.com/documentation/ddi0406/cd), cached text `build/research/zeliboba-review-thumb-it/arm-ddi0406cd.txt`.

| Instruction | Primary section/pages | Implemented rule relevant to the guest |
| --- | --- | --- |
| VLD1 multiple single elements | A8.8.321, A8-899–900; cached lines 44515–44630 | Types 7/10/6/2 transfer 1/2/3/4 D registers using element-sized MemU accesses, explicit alignment checks, endian interpretation and final writeback. Rm=13 increments by list size; Rm=15 has no writeback; another Rm supplies the original offset. |
| VMOVL | A8.8.347, A8-951–952; lines 47314–47400 | Signed/unsigned 8/16/32-bit source elements widen into an even-D Q destination. Read the complete input D register before either destination half is written, including `q8,d16` overlap. |
| VADD integer | A8.8.283, A8-829–830; lines 41229–41308 | 8/16/32/64-bit lane-local modular addition. Q operands require even D indices; D operations permit odd/high indices. Sources are captured before writes. |
| VPADD integer | A8.8.363, A8-981–982; lines 48721–48820 | Pairwise reduction of two D registers with 8/16/32-bit lane arithmetic; Q=1 and size=3 are undefined. Read both source vectors before an overlapping destination write. |
| VMOV word scalar ↔ ARM register | A8.8.342/343, A8-941–944; lines 46921–47100 | D index uses the full high register bit, and lane index selects the lower/upper 32-bit element. Word transfer is distinct from byte/halfword encodings. Rt=15 and Thumb Rt=13 are unpredictable and rejected by this model. |

VLD1's 64-bit big-endian case follows the manual's explicit word order: load `address+4` into result bits 31:0 first, then `address` into bits 63:32. A both-pages-unmapped test checks the resulting first DFAR. Alignment faults and later translation faults preserve Rn; completed earlier elements remain loaded as in the pseudocode. Unaligned half/word accesses retain existing Normal-memory checks and independent translations; Device and Strongly-ordered unaligned accesses fault.

## Unchanged native continuation before and after

`goal-native-syscon-checksum-probe.cpp` loads the original ELF bytes and enters the original inline block at `004F8B26`. The ten-byte supplied protocol test packet is `04 00 06 00 00 60 40 00 55 00` at an unaligned Normal-RAM address; the first eight bytes sum to `AA`, whose one's complement is `55`. Its SP and unrelated D registers are seeded with visible sentinels. No firmware instruction, result, callback or global is patched.

| CPU support present during replay | Next genuine boundary/result | Evidence log |
| --- | --- | --- |
| Added VLD1 only | D16=`0040600000060004`, R2 advances by eight, SP intact; undefined VMOVL.U8 at `4F8B5E` | `goal-native-syscon-vld1-continuation.log` |
| Added VMOVL | Undefined VADD.I32 at `4F8B7A` | `goal-native-syscon-movl-continuation.log` |
| Added VADD | D24=`000000600000004A`; undefined VPADD.I32 at `4F8D40` | `goal-native-syscon-add-continuation.log` |
| Added VPADD, old scalar extraction | D24=`000000AA000000AA`, but `VMOV.32 r2,d24[0]` at `4F8D46` reads seeded D8 and yields R2=`EEFF0008`; takes native failure `4F8372` | `goal-native-syscon-padd-continuation.log` |
| Corrected high-D word scalar extraction | R2=`000000AA`; reaches native success rejoin `4F8DB4` in 41 instructions, SP=`8001F000` unchanged | `goal-native-syscon-checksum-after.log` |

The success point is the checksum block's continuation in a larger callback, not a claimed return from the full callback or a new integrated boot success.

## Regression and validation

Source changes are limited to `src/cpu/arm/arm_core.cpp` and `tests/test_arm.cpp`; headers, the disassembler, device sources and guest firmware remain untouched.

Twelve new tests cover exact native halfword-aligned VLD1 execution; ARM/Thumb and little/big-endian element loads; lists ending at D31; old-Rm writeback including Rm=Rn; page crossing and partial fault commit; SCTLR.A, memory attributes and illegal encodings; IT suppression; signed/unsigned VMOVL widening and overlapping source; lane wrap and aliasing in VADD/VPADD; high-D scalar read/write with neighbor preservation; and the portable unchanged native checksum block against an independent scalar byte-sum oracle.

The checksum fixture embeds all 654 original bytes at PT_LOAD `+B26..DB4`, SHA256 `6d47bf269cd0b476e5f92a0cb0dc679e744828cbfbe7e3bbc1225d29fb743568`, plus its original eight-byte scalar initialization at `+165C`. Relative branches and success/failure destinations are retained. It executes 22 packet lengths × 8 alignments × valid/invalid checksums, plus three malformed-length/capacity controls: 355 native block executions. Short scalar cases, each vector unroll, the long four-block loop, maximum 255-byte lengths and high-D decoys are included. The test checks the complete byte sum, chosen native branch, SP, untouched registers/D neighbors, FPSCR and input RAM.

Isolated compilation links one newly compiled `arm_core.cpp` object ahead of the existing frozen core archive. No shared build or new full boot probe was run.

```sh
/usr/bin/c++ -std=c++20 -O2 -Isrc -c src/cpu/arm/arm_core.cpp -o build/research/zeliboba-syscon-checksum.o
/usr/bin/c++ -std=c++20 -O2 -Isrc -Itests tests/test_arm.cpp tests/test_main.cpp build/research/zeliboba-syscon-checksum.o build/libzeliboba_core.a -o build/research/zeliboba-syscon-checksum-tests
build/research/zeliboba-syscon-checksum-tests neon_
build/research/zeliboba-syscon-checksum-tests
```

Results: focused NEON/disassembly **21 tests / 0 failures**, complete isolated ARM/disassembly **186 tests / 0 failures**. Logs: `goal-native-syscon-checksum-focused-tests.log`, `goal-native-syscon-checksum-arm-tests.log`. Twelve added cases imply **611** integrated tests from the frozen 599 baseline.

Independent read-only review by `sdif_review` verified exact fixture bytes against the supplied ELF, primary encodings/pseudocode, overlap constraints, memory faults/writeback and portable arithmetic. The review's sole BE64 access-order item was corrected and tested before freeze; no remaining material blocker was found.

## Scope limits

Other structure/lane load forms, nonzero VSHLL, byte/halfword scalar VMOV, and unrelated SIMD operation families remain unsupported. Existing broader CP10/VFP access-control behavior is unchanged. This correction proves the native checksum computation and architectural instruction boundaries; it does not claim new live Syscon callback completion, framebuffer presentation or the guest PlayStation logo. Those require the next root-controlled integrated run.
