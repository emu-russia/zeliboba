# Native bootconfig dependency-open and ARM rendezvous evidence

Frozen read-only diagnosis against the native firmware 1.04 path, completed 2026-10-01 before the architectural unaligned-access fix. Latest shared binary: 541 passing tests. Captures use the absolute `build/goal-arm-rendezvous-clone.img`, with all `ZLB_*` variables removed except the explicitly noted observational write trap in the gate capture. The clone remains clean: 8,345 reads, zero writes. No path, file, marker, callback, register, or return-value replacement was made.

## First cause: an actual unaligned A32 LDR corrupts the path

`goal-arm-native-bootconfig-path-word.log` captures the first prefix-copy call from native path normalizer `0x51024854`, at call site `0x510248CE`. The called routine `0x51013730` is an A32 bounded string-copy routine (word loads with per-byte NUL detection and zero padding).

At the genuine instruction `0x51013744`, raw word `0xE4913004` is `LDR R3, [R1], #4`:

| Observation | Before | After one native instruction |
| --- | --- | --- |
| ARM0 PC | `0x51013744` | `0x51013748` |
| R1 | `0x0007FC53` | `0x0007FC57` |
| R3 | `0` | `0x30736F2F` (bytes `/os0`) |
| CPSR | `0x2000001F`, SYS ARM | unchanged |
| SCTLR | `0x20005805`, MMU enabled, A=0 | unchanged |
| MMU fault count | 8 | 8 |

VA `0x0007FC53` translates to PA `0x40319C53`, Normal cacheable guest RAM. The bytes there are `2F 6B 64 2F 73 79 73 6D ...`, the correct input `/kd/sysmem.skprx`. The correct little-endian LDR result is therefore `0x2F646B2F` (`/kd/`). The observed word instead equals `ROR(0x2F30736F, 24)`, from the aligned address `0x0007FC50`, whose bytes are `6F 73 30 2F` (`os0/`). This is legacy aligned-and-rotated behavior applied to the ARMv7 Cortex-A9.

The primary architectural contract is ARM DDI 0406C.d: `LDR (immediate)` operation uses `MemU[address,4]`; `UnalignedSupport()` is always true in ARMv7 (D16-2649); `SCTLR.U` is RAO/SBOP in ARMv7, so a cleared stored bit22 cannot select the old ARMv6/earlier access model (SCTLR description, `Alignment support` A3-106). Local primary manual: `build/research/zeliboba-review-thumb-it/arm-ddi0406cd.pdf` and `.txt`.

`goal-arm-native-bootconfig-fat-lookup.log` captures the effect at `0x5102398E`, immediately after normalizer return: prefix buffer VA `0x7FAE0` contains `/os0` rather than `/kd/`; suffix buffer VA `0x7F9E0` contains `skd/eysmkm.s` rather than `sysmem.skprx`. Input at VA `0x7FC53` is still correct. The native KD-component lookup is consequently skipped and the final lookup runs against root with cluster sentinel `0xFFFFFFFE`, returning zero at `0x51023A20`.

An isolated unchanged-native-code probe owned by the CPU agent independently reproduced the same prefix, suffix, pointer and incorrect word. The proposed fix is architectural single-word `MemU` behavior, not a firmware or FAT workaround.

## The load gates and the image are valid

`goal-arm-native-bootconfig-sysmem-gate.log` reaches the actual dependency path `os0:kd/sysmem.skprx`. Both bootconfig gates pass:

- `0x51018F9E`: R0=1 after checking bootconfig bit0.
- `0x51018FAA`: R0=1 after checking bootconfig bit2.
- Bootconfig VA `0x47C0`, physical `0x403047C0`, field +0x6C is 5. Native A32 copy `0x51011DA8` supplies bit2 (value4); the historical marker adds bit0. Setting more marker bits is not justified.

`goal-arm-native-bootconfig-fat-open.log` catches actual native FAT open after `0x51023970`: R0=0 at `0x5102415E`; the caller constructs `0x803FF007` at `0x51024160..164`, received at frontend return `0x510015F0`. Frontend input and stack remain the correct `os0/kd/sysmem.skprx` before path splitting. There are no extra MMC data reads at this failure.

Independent raw FAT audit by the storage agent (`goal-independent-os0-fat-check.json/.txt` and parser) proves both os0 partitions match all original 6,594,560 source bytes, with zero tails, equal FAT copies, valid LFN checksum/order, and complete `kd/sysmem.skprx` and `kd/stdio.skprx` contents matching the extracted genuine files. All 28 listed dependencies exist. The KD root entry is cluster3, SFN `KD         `, attributes `0x16` in both source and image. No missing/corrupt file is evidenced at this frontier.

## Why bootconfig reports success and later branches to zero

Native bootconfig first segment is VA `0xCC000` (PA `0x40373000`), module UID `0x20075`. The CPU agent's separate integrated capture proves all three segment loads return0, relocation returns0 at `0x5101A3E2`, and first callback `0xCC0B4` reaches `0xCC0BE` with R0=0, followed by UART `ScePsp2BootConfig Starting... OK`. Both encrypted sections and native decompression outputs match the genuine ELF.

The dependency-load results are separate: `goal-arm-native-bootconfig-list-results.log` captures `0x803FF007` at both `0xCC0018` (first fourteen core modules) and `0xCC002E` (second fourteen drivers). Both UID arrays at `0xCD038` and `0xCD000` remain zero. The module's first callback ignores these failures and returns0.

The second callback `0xCC0C0` enters on all four cores. Its bound import at `0xCC0084` correctly interworks to native NSKBL `0x51010710` (library SceKblForKernel NID `0xD0FC2991`, function NID `0x1DB28F02`). All twelve imports are genuinely linked; this is not an unresolved stub.

`goal-arm-native-zero-entry-capture.log` stops the first branch to VA0 before its fetch:

- ARM0 PC0, LR `0x51010721`, MMU still enabled, fault count8.
- `[0x5113B61C] = 0x4000` is the sysroot pointer.
- Native `0x5101071C` loads `[sysroot+0x40]` into R3; that slot (VA `0x4040`, PA `0x40304040`) is zero.
- `BLX R3` at `0x5101071E` calls zero. The stack return address is `0xCC0089`.

The first dependency modules never load/register the required callbacks. A null guard or fabricated callback would hide the earlier load failure and would encounter other still-unregistered sysroot callbacks afterward.

## The final WFE is a consequence, not the first blocker

The latest full fill-corrected run still reaches the same empty callback, primary restart and A4 rendezvous; CMeP cleanup now correctly returns to native sleep `0x80048A`, so CMeP memory corruption is independent of this path error.

`goal-arm-native-rendezvous-capture.log` distinguishes the final cores:

- ARM0 WFE helper PC `0x510147DC`, LR `0x510158FF`, SCTLR=0, barrier at physical `0x5102B000`: count3, owner1. Stack callers `0x51000337 -> 0x510003A5 -> 0x51000250` identify re-entered NSKBL startup. The legacy low-window fetch mapping after VA0 allows this restart.
- ARM1..3 PC `0x510147DC`, LR `0x51015893`, MMU enabled. They wait on sysroot barrier VA `0x4034` (PA `0x40304034`), count4, owner0; caller chain `0x510109A9 -> 0x51001481 -> 0x510014CD -> 0x5100153B -> 0x51000D0D` identifies module batch completion.

Independent no-watchdog run has the same frontier with zero forced cluster wakeups/counter patches. A4 here is a restarted initial checkpoint, not a kernel handoff or a guest logo. Retest native dependency loading/authentication/registration after the architectural CPU fix before changing any machine hook.
