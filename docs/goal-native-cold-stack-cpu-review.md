# Cold KBL stage stack: CPU and boundary review

Source/profile: corrected board-NVS cold parameters (`FF FF 00 FF`, product word4), old binary retaining unconditional stage-stack bias. No CPU source changes or shared builds were made for this review.

## Exact native failure boundary

`goal-cold-arm2-epilogue.py` used a private APFS eMMC clone, removed inherited ZLB variables, and set only an ARM breakpoint at native `4002F808`; no watchpoints changed the ordinary machine budget. The capture is `goal-cold-arm2-epilogue.log`.

ARM2 reaches the genuine Thumb `POP.W {r4-r11,pc}` at 0x4002F808 with SP=0x85CF4, LR=0x4002F805, CPSR=0x2000003F (SYS Thumb). VA 0x85CF4 translates to PA 0x4012FCF4, normal cacheable, L2[0x85]=0x4012F45F. The complete saved-register frame is zero, including saved return VA 0x85D14 -> PA 0x4012FD14. The subsequent normal runm records a fetch at VA 0 and ARM2 restarts the KBL reset path while the other cores wait at their later barrier.

The script's `step1` stays on the breakpoint; its after-register snapshots therefore do not claim single-instruction execution. Normal runm resumes the instruction before the next stop. No direct post-POP zero-PC snapshot is claimed. The actual zero frame and following zero fetch are directly observed.

## CPU semantics

Native unlock 0x4003A300 clears the lock, performs barriers/SEV, preserves CPSR outside I/F via MRS/AND/BIC/ORR, and executes MSRc at 0x4003A324. Its value retains mode SYS. `write_cpsr_masked` switches register banks only when the mode actually changes; SYS->SYS preserves live SP/LR. User/System share a canonical SP/LR bank through sp_lr_bank. There is no evidenced CPU bank-switch defect at this boundary. The fault-hook PC=8 is the ARM fetch pipeline value for an ordinary VA 0 fetch, and does not establish IRQ/vector entry.

## Model defect and narrow correction

Native caller 0x400207B0..0x40020806 allocates 0x4000-byte per-core stacks, saves base+size at record+0x1BC, and passes the resulting top to trampoline 0x4003A140 (`MOV SP,r2; BX r1`). In this cold run original secondary tops 0x50000/0x84000/0x40000 were blindly biased to 0x51000/0x86000/0x43000. Each moves outside its own allocation. The root-owned correction restricts the fallback to the historically measured shared original top 0x4000 and preserves native allocated tops. This is coherent with native ownership; it does not alter boot flags or guest instructions. Positive corrected-runtime progression remains a separate fresh capture.

Docs updated: KBL current boundary statement, NSKBL trampoline/history qualification, DEBUGGER stack/BP inspection guide. No other docs or source changed in this task.
