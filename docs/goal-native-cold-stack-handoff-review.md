# FW 1.04 cold KBL stack handoff review — 2026-10-01

The existing machine stack substitution applied to native allocated stacks as well as the older shared `0x4000` stack. The limited `original_top == 0x4000` condition now in `src/machine/bootchain.cpp:2861` preserves the native allocator's stack tops. This is a review of that source change and the prior failed cold run; it does not report the outcome of the rebuilt binary.

## Genuine allocation and handoff

Primary firmware evidence is the supplied `kernel_boot_loader.self`, examined through its prior live runtime snapshot `build/goal-kbl-live-bank.bin` at VA `0x40020000`. Snapshot size is `0x60000`, SHA256 `6699e2f8f04c7e31019ab0d002d7cdc48f58304b01591bd3c66b116f315eef97`. Its saved code contains the following unchanged native sequence:

| Runtime instruction | Meaning |
| --- | --- |
| `40020784 LSL.W R6,R8,#5` | Per-core record stride is `0x20`. |
| `40020788 MOV.W R9,#4000` | Requested stack size is `0x4000` bytes. |
| `40020796 ADDS R5,R4,R6` | Select this core's record within the context. |
| `400207B0 STR.W R9,[R5,#1B4]` | Record requested size. |
| `400207BC MOV R2,R9`; `400207C0 MOV.W R3,#1000` | Allocate a 16 KiB block with the native `0x1000` argument. |
| `400207C6 BL 40030CDC`; `400207CC STR.W R0,[R5,#1B8]` | Allocate and retain the native handle. |
| `400207CA ADD R1,SP,#14`; `400207D0 BL 4002A3E0` | Obtain the block's base through an output slot. |
| `400207D4 LDR.W R1,[R5,#1B4]`; `400207DA LDR R2,[SP,#14]` | Load size and base. |
| `400207DC ADDS R3,R2,R1`; `400207DE STR.W R3,[R5,#1BC]` | Record stack top `base + size`. |
| `40020802 LDR.W R2,[R6,#1BC]`; `40020806 BLX 4003A140` | Pass the recorded native top to the trampoline. |
| `4003A140 MOV SP,R2`; `4003A144 BX R1` | Install that exact stack top and enter the next stage. |

`4002A3E0` looks up the allocation and copies its object field `+0x14` to the caller's output slot (`4002A3F2..F8`). The trampoline target is `400204A9`, whose prologue pushes five registers then reserves `0xB4` bytes. A whole-snapshot Thumb call scan found only `40020806` calling this trampoline. The scan supplements the actual decoded caller; data bytes elsewhere in the snapshot are not treated as code evidence.

## Historical failed cold run

Both `build/goal-native-cold-full.log:135` onward and the independent graphics capture `build/goal-arm-native-syscon-oled-cold.log:138` onward used the previous unconditional bias. That source added `core * 0x1000` before the trampoline. Its log accidentally printed R2 after changing it, so the two displayed values are identical. Reversing that known addition yields these virtual allocation ranges:

| Core | Native top inferred from log and old hook | Native 16 KiB VA interval | Modified top actually installed |
| --- | --- | --- | --- |
| ARM1 | `00050000` | `[0004C000,00050000)` | `00051000` |
| ARM2 | `00084000` | `[00080000,00084000)` | `00086000` |
| ARM3 | `00040000` | `[0003C000,00040000)` | `00043000` |

These ranges are arithmetic reconstructions from the genuine size and old hook, not fresh dumps of allocation records or physical mappings. They establish that the modified stack tops lie beyond their own native allocations. The changed prologue accesses produced extra model mappings at VA `00050FEC` and `00085FEC`. ARM2 later fetched VA zero with LR `4002F805`. Native `4002F800` calls the IRQ-mask unlock helper, then `4002F806` restores the local frame and `4002F808` pops `{R4-R11,PC}`. The CPU reviewer's separate unchanged old-binary capture stops at that POP with SP `00085CF4`, saved PC slot `00085D14` equal to zero, and all nine saved registers zero; that slot translates to PA `4012FD14` through L2 entry `4012F45F`. CPSR remains SYS Thumb, so no register-bank switch explains the lost frame. The exact instruction that replaced its backing mapping is not established by these captures.

All four cores ultimately wait at `4003A01C`. This is the instruction *after* `4003A018 WFE`: `4003A01C BX LR`. ARM0's LR `4003B3C9` and ARM1/2's LR `4003B3D7` belong to the genuine four-core barrier at `4003B384`. Thus the final stop is a barrier wait after the secondary failure, not evidence of a new WFI device requirement. No os0 module or Display startup was reached in this cold capture.

Earlier successful warm-shaped input runs also used the overbroad hook: `goal-native-early-ready-full.log:141..145` installed tops `4E000`, `53000`, `41000`, implying original distinct native tops `4C000`, `50000`, `40000`. Passing those runs did not establish that biasing native stacks was correct.

## Cold parameter consumers and source review

The corrected NVS-derived board construction is documented in `build/goal-native-kbl-param-provenance.md`. This correction does not modify eMMC bytes. In the secure KBL, the native parameter copy is selected at `400211E8..40021210`. At `40021246`, parameter byte `+31` is compared with `FF`; the FF path branches to `40021922`, invokes the native `4003B89C` operation for DIP indices `C4` and `D3`, and rejoins at `4002125C`. The old non-FF path invokes the separate native `4003B860` operation for those indices. This is a real earlier path difference, not a reason to restore fabricated update/external flags.

The new narrow hook checks the original R2 before changing it and preserves every allocated top above. It also fixes the diagnostic's before/after values. The existing development fallback remains available only for the historically measured raw shared top `4000`; `ZLB_KBL_CORE_STACK=0` and global substitution disabling retain their existing meaning. No stack deduplication, guest predicate changes, new flag writes, forced barrier release or exception-return patch is part of this change. Parent owns the rebuilt tests and fresh runtime comparison.

The later fixed two 14-entry lists in supplied `psp2bootconfig.elf`, nonexternal NSKBL open-path rejoin, and cold Display predicate outcomes remain prospective until this earlier native handoff succeeds. They are not cold-run milestones.

## eMMC verification scope

`tools/verify_emmc_bytes.py` opens the image read-only and compares seven firmware placements and expected tails against `Vita_104_Firmware.zip`. The prior result `build/macos-emmc-bytes.json` reports zero differing bytes for those placements, matching duplicate FATs, and whole-image SHA256 `f36b4612f3e7fe497d7d9899b16434a9208b8a0cdee06f260af3ff5109264534`. `build/macos-emmc-negative-check.json` demonstrates detection of a deliberately changed byte. `build/goal-independent-os0-fat-check.json` independently confirms nested `kd/sysmem.skprx` and `kd/stdio.skprx` bytes in the source and both os0 copies.

This verifies the reconstructed image's supplied firmware content. No original Windows image was available, and these checks cannot establish whether that original image was damaged, whether Windows caused damage, or whether reconstructed console-specific/RPMB/idstorage data match a real device. Cold board NVS fields and the stack model defect are separate from those image byte comparisons.
