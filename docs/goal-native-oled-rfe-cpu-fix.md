# Native OLED IRQ return and User/System register-bank correction

2026-10-01. Source frozen in src/cpu/arm/arm_core.cpp/.h and tests/test_arm.cpp. Three added tests; isolated ARM suite **174 tests / 0 failures**. Parent integrates the full suite; expected total **582** from the last579 baseline. No shared build or board/device source change was made by this agent.

## Actual frontier and reproducible cause

The579 private native capture `goal-arm-native-syscon-oled-timers.log/.json` enters genuine OLED A1/length5 helper at runtime4F41EC with SP AADF0, then executes its push and SPI2 start at4F477A/477E with SP AADD0. Final drain4F4324 has SP0/LR7 and74 queued bytes. LR7 is deliberate helper scratch use and is normal; the zero stack is abnormal. Original full helper without interrupts already produces nine words/18 bytes and preserves its pushed stack.

`decode_arm_rfe_srs` previously passed `new_pc & ~3` to its exception-return helper even when restored CPSR.T=1. This loses a valid Thumb halfword boundary. At genuine linked81000286/runtime4F4286, OLED executes `FA08 F00E` (LSL.W R0,R8,LR), followed at8100028A/runtime4F428A by `2D0F` (CMP R5,#15). An IRQ at that CMP boundary incorrectly returns to81000288, the previous instruction's second halfword. The accidental word F00E2D0F sets SP0 on the current decoder. This is an execution-address error; the original valid packer instruction is supported.

`goal-oled-a1-rfe-boundary-probe.cpp` executes unchanged original OLED PT_LOAD bytes with an architectural IRQ frame/preservation fixture ending in the exact A32 native RFEIA SP! opcode F8BD0A00. Trial54 interrupts at8100028A. Before: resumes81000288, zeroes SP and reaches drain with SP0/LR7 (`goal-oled-a1-rfe-boundary-before-54.log`). After: resumes8100028A and reaches drain with exact18 TX bytes/nine words/SP81004FD0/LR7 (`goal-oled-a1-rfe-boundary-after-54.log`). Other old-boundary trials produce missing/extra FIFO words or undefined instructions.

After the correction, **all378 helper instruction-boundary IRQ trials** preserve the exact original transfer and stack (`goal-oled-a1-rfe-boundaries-after.txt`). The precise live interrupt cycle was not captured, so this isolates and reproduces a mechanism matching the live symptom; it does not claim a trace of the exact live return. Full native integration is still required.

## Architectural basis and changes

[ARM Architecture Reference Manual ARMv7-A/R, DDI0406C.d](https://developer.arm.com/documentation/ddi0406/cd), B9.3.13 RFE, printedB9-1986/1987, restores CPSR before BranchWritePC. A2.3.2 BranchWritePC, printedA2-48, preserves bit1 for Thumb and clears bits1:0 for ARM. The RFE caller now passes its complete loaded PC; both exception-return helpers align only after selecting restored CPSR.T. Existing ALU/MOVS-PC and LDM-with-PC-and-S return callers had no earlier word mask; they now also use the restored-state alignment, preserving Thumb+2 and aligning ARM. The sole `exception_return_with_cpsr` caller is the A32 RFE decoder; ordinary LDM/PC interworking paths retain their own behavior.

The same manual B1.3.2, FigureB1-2 and RBankSelect, printedB1-1144/1145, maps both User and System to SP_usr/LR_usr. Existing backing arrays incorrectly used separate entries for mode16 and31. A canonical SP/LR index now maps System to User in mode switches, exception entry and both return paths. Mode encodings, privilege, SPSR, IRQ/FIQ/Monitor banks and Security selection remain distinct. The existing privileged STM/LDM user-register path consequently reads/writes the same bank that System accesses. This is independently proven architectural correctness; the live OLED SP0 reproduction depends on the RFE alignment defect, not on the alias change.

## Regression and provenance

* `arm_exception_returns_use_restored_state_alignment`: executes RFE, LDM-with-PC-and-S and MOVS-PC, each with all four low address-bit combinations and ARM/Thumb restored states. It executes the selected target instruction, proving address bit0 does not select the ISA and a halfword-only Thumb target is preserved.
* `arm_irq_rfe_preserves_halfword_oled_instruction_boundary`: real IRQ entry into a fixture assembled from supplied IntrMgr entry/exit instructions, with native frame order and actual RFEIA SP!. It returns to byte-exact genuine OLED halfwords, checks intended CMP/STR execution, restored SYS scratch LR/SP/CPSR and IRQ stack writeback. Old source fails by returning into F00E and zeroing SP.
* `arm_user_system_share_sp_lr_across_privileged_transfers`: executes SYS-to-USR MSR and privileged STM/LDM user-bank transfers, then returns into System. It checks alias continuity, distinct IRQ/FIQ SP/LR and reset in both Secure and Nonsecure states. Old source loses User SP/LR and later restores stale System entries.

Before-fix architectural tests fail12,2,10 assertions respectively against `build/research/zeliboba-rfe-user-system-before.cpp`. After-fix full isolated ARM suite passes174/0 (`goal-native-rfe-user-system-isolated-tests.log`). Existing TrustZone and exception-mask tests also pass. Compiler emits only pre-existing unused-symbol and VFP-mask warnings.

Original supplied oled.elf SHA256: `ddafc3324ee69e817f198c63363eadb58c26ec5dd916777b55c88c1a38635851`. Its unchanged PT_LOAD begins fileA0, linked81000000; runtime OLED text4F4000. Native InstrMgr nonsecure executable PT_LOAD starts fileC0, linked81000000; SHA256: `c967807ee961f9ec7d192f8fb0d3859842ff36ba92f5866220f9ba462d136c00`. Its IRQ wrapper contains RFEIA SP! at linked81000844,81000860 and8100089C. The isolated handler is a reduced register/frame fixture, not a claim to execute the entire native dispatcher/scheduler.

Valid privileged user-register transfers in the regression run from IRQ mode. ARM DDI0406C.d B9.3.5 and B9.3.16 classify the corresponding LDM/STM user-register forms executed in User or System mode as UNPREDICTABLE; those cases were not added to the test or changed in production. Existing FIQ User-R8..12 transfer handling remains outside this fix's coverage.

The pre-existing A32 SRS decoder does not implement the architectural current LR/SPSR to target-mode SP contract. No supplied native wrapper observed here uses SRS, so that separate limitation is deferred. Unsupported T32 RFE/SRS families and unrelated RFE addressing variants were not expanded. No guest firmware bytes, callback, service result or RTOS return was patched. Actual native continuation and guest PlayStation logo remain unverified after this source freeze.
