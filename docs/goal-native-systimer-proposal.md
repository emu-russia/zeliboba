# Native FW1.04 LT5 / WT7 timer evidence and bounded model proposal

2026-10-01, current integrated baseline: 572 tests / 0 failures. This is a read-only proposal. No timer source change, handler unlock, counter bias, service return or guest patch has been applied. The actual guest PlayStation logo has not been observed.

## Reached failure and division of ownership

`build/goal-native-emc-syscon-full.log` proves only the first EMC payload E0000 was submitted and completed. Secure ARM2 then waits at A32 CD024, LR 3BF1D3, lock 548728, for the genuine delay 0x30D40. Native Syscon/OLED workers separately read a permanently zero ThreadMgr time counter before any SPI submission. CPU-agent provider evidence is in `build/goal-native-clock-provider-evidence.md`; its fresh current-binary capture is `build/goal-native-clock-audit-capture.log`.

The time provider is GetSystemTimeLow NID47F6DE49, runtime 4B7D5D. Wide is a separate function at 4B7D34. Neither divides or converts the hardware count. ThreadMgr maps LT5 PA E20B6000. Secure delay NIDC0908EA9 resolves 3BF149 and maps WT7 PA E20BE000. These are distinct peripherals with distinct register layouts and real GIC interrupt IDs141 and135.

Current `src/hw/soc/kermit.cpp` installs LT5 as `Kermit.UnkE20B6`, an empty RegisterBlock. With no defined registers it drops even the captured KBL control/counter writes. WT7 is absent. Adding tick registration to the empty block alone cannot fix either device.

## Primary supplied firmware and addresses

`Out/fs_dec/os0/kd/systimer.elf` has executable PT_LOAD file offset C0, VA81000000, length1290; its ordinary runtime text is EA000. `build/goal-native-systimer-thumb.txt` has been regenerated using this actual offset. Earlier output generated with A0 was incorrectly displaced by20 bytes and must not be used.

* Generic IRQ entry 81000000 acknowledges word-timer +14 with3 (8100000E), or long-timer +18 with3 (8100004A). With no software period, type2 increments the software high counter at 8100003A..42; this is consistent with a word-counter wrap interrupt, but does not establish individual status bits by itself.
* Reset 81000050..EA writes zero word-timer config+8/counter+4/deadline+0/unknown+C and ACK3 at+14. Long reset writes config+1C zero, zero doublewords at+0/+8/+10 and ACK3 at+18. IRQ setup accepts word IDs128..134 and long IDs136..141; reserved WT7 ID135 is handled separately by the secure module.
* Set period 810003A4 writes word deadline+0 or long deadline doubleword+8.
* Start 8100051C sets config enable bit0. A word timer without a software period uses OR3; with a period it uses OR1D. Long timer without a period uses OR1; with a period OR1D. Stop 810005CC clears bit0. These paths establish enable bit0 but leave the individual meanings of bits1..4 only partially determined.
* Read counter 8100074C reads word+4 or long high+4 / low+0 / high+4 with rollover retry. Clear/restart 81000640 writes these same counter fields.

`Out/fs_dec/os0/kd/threadmgr.elf` has executable PT_LOAD file offset A0, VA81000000, runtime4A0000. The following addresses are linked / runtime:

* 81017DB2..7E4E / 4B7DB2..7E4E maps E20B6000/1000 and registers IRQ141 with handler81017A1D / 4B7A1D.
* 8101762C..6C4 / 4B762C..76C4 chooses a 64-bit next deadline, at least current+200, and writes LT5 +8 then+C. With no deadline it writes both wordsFFFFFFFF.
* 81017A1C..B36 / 4B7A1C..7B36 executes due callback/list processing using current time. At81017B18 / 4B7B18 it writes exactly2 to LT5 status+18, then recomputes the deadline. This supports interpreting status bit1 as the comparison-event acknowledgement; it is not a general-purpose status injection register.
* KBL4002158E..1598 writes stopped LT5 config2F345008, zero counter low/high, comparisonFFFFFFFF/FFFFFFFF, then starts with2F34500D. See the CPU-agent report/capture for exact source bytes.

The cached genuine secure SceKernelIntrMgr image is `build/research/zeliboba-tz-intrmgr-physical.bin`, base3BE000. Current-runtime delay binding and globals were revalidated by the fresh clock capture. `build/goal-native-wt7-delay-thumb.txt` contains the relevant unchanged instructions:

* Init3BF230..286 sets PA E20BE000, registers GIC ID135 with handler3BF001, priority80, initial targetF, then enables it. Alternate3BF290..2E0 maps the same physical page.
* Delay3BF148..214 maintains active-core mask at548700, selected core at548704, deadlines at548708..14, common lock548718, per-core wait locks548720..2C and timer pointer548730. It uses a maximum1,000,000-count chunk and targets IRQ135 to the selected core via3BEFE0..FFE / 3BEC2C. The first active delay writes deadline+0=duration, counter+4=0 and control+8=DD00000D. A later delay computes current+duration and may replace the earlier comparison.
* Handler3BF000..136 saves config+8, writes config0, acknowledges+14 with3 and reads current+4. For every active core it releases its actual wait lock only when **current > that core deadline**. It selects the next remaining deadline and restores the saved config if work remains. Equality is an explicit native rearm case; it must not be defeated by counter+1 or an injected unlock. The mapped lock/unlock import helpers are CD014/CD038 and irq-save variants CD050/CD0C4.

## Independent hardware reference and limits

[Vita Development Wiki Hardware Timers](https://www.psdevwiki.com/vita/Hardware_Timers) revision21127 corroborates all timer bases, 1000-byte apertures and IRQs; configurable enable bit0, prescale[31:24], source[23:20], and the different word/long layouts. The same page explicitly marks low mode bits and status meanings as uncertain. This is reverse-engineering documentation, not a manufacturer register manual. The supplied, reached firmware above is the primary behavioral evidence.

The wiki gives source3=48MHz and divider `(prescale+1)`. Thus LT5's captured prescale47 is exactly1MHz and its native raw counter is microseconds. WT7's source0 uses SysClock (190 or222MHz), with captured prescale221. A fixed222MHz board SysClock would give1MHz; the actual physical platform clock input has not been captured, so selecting222MHz is an explicitly modeled clock input, not a proven register value. CPU333MHz and PERIPHCLK1MHz are existing emulator clock conventions. No hardware-tested baremetal timer implementation was found in xerpi/vita-libbaremetal; its tree contains no timer/clock files. The Linux Vita DTS uses Cortex-A9 MPCore timers instead and does not establish E20B timer modes.

## Concrete narrow implementation proposal

Own `src/hw/soc/timers.cpp`, `soc_internal.h`, `kermit.cpp` and focused timer tests after root authorization. Leave the existing Cortex-A9 global/private timer models unchanged. Add a Vita timer device capable of the two reached layouts and install **only LT5 E20B6000/1000 IRQ141 and WT7 E20BE000/1000 IRQ135**; do not map unobserved timer slots or TMBERR/GT hardware.

Registers: word deadline00, current04, config08, unknown0C/10 retained as snapshots, status14; long current00/04, deadline08/0C, unknown10/14 snapshots, status18, config1C. Reset all counter/deadline/config/status/fractional state and deassert each physical IRQ. Unknown offsets readFFFFFFFF and writes do not create state or events. Counter/deadline/config writes and reads are consistent for byte, halfword and word lanes. Status is hardware-owned; writes acknowledge only supplied low status bits, zero ACK preserves, upper-lane ACK cannot clear low pending bits, and no write creates an event.

Clock profile support is deliberately scoped: source3 and opaque captured middle fields345000 for LT5; source0 and captured middle fields0 for WT7. Preserve raw config snapshots. Support enable bit0 and prescale[31:24] as documented, but accept low mode patterns only from the reached profiles: long stop0/345008 or34500C, active34500D; word stop0/00000C, active00000D. Unknown active source/mode/reserved combinations remain visibly unsupported/stopped with named diagnostics and no fabricated IRQ or completion. Do not guess capture-output, auto-reload or unsupported generic SceSystimer modes.

Elapsed emulated time is the sole source of counter advance. The existing Kermit tick accumulator handles sub-microsecond CPU-cycle fractions; the timer adds a retained integer-rational phase for source/(prescale+1), including non-integral per-PERIPHCLK increments. Use48MHz input for LT5 and explicitly modeled222MHz SysClock for WT7; avoid floating point, host wall clock, read-driven increments and catch-up patches. Configuration changes may reset the divider phase as an explicit emulator choice, while stopping otherwise preserves the counter.

Support a latched comparison event only, projected in status bit1 from the native LT5 ACK2 evidence; ACK3 clears it as native WT7 requires. Keep unproven overflow/status bit0 inert for these comparison-only modes, while counter arithmetic still rolls normally (32-bit WT and64-bit LT). Do not invent the separate overflow interrupt mode or claim that every unknown mode is enabled. Fire on the next genuine counting tick that reaches/crosses a newly armed comparison, including one that starts equal/already overdue, and disarm the event after it fires. ACK alone must not create repeated events from a still-past deadline. Counter/deadline writes or an actual stop-to-start transition rearm. The precise physical equality/crossing phase is not measured; this latch model is an explicit choice consistent with native handling, and allows equality to rearm until elapsed time makes current>deadline. IRQ outputs route through the existing GIC callback with IDs135/141, honoring native targets/enables and without an SGI or direct CPU wake shortcut.

Tests required before freeze: actual KBL LT5 writes plus unchanged native GetTimeLow/Wide instructions and rollover; rational divider and split-cycle equivalence; zero/partial/full W1C and hardware-owned pending status; reset and independent ports; unknown active clock/mode rejection; next-event deadline/rearm/no repeated IRQ from ACK; WT7 programmed with30D40 and physical GIC135 delivery to selected Secure core, then an unchanged native expiry/unlock path (including equality/rearm before greater-than). Preserve genuine ISR result and lock writes; never replace the guest routine with a test-induced success hook. Root runs shared all-target/full-suite/ordinary boot once source and isolated checks are ready.

## Source identity

SHA256 of supplied systimer.elf: `5cdccb5c10a5470c39098b13fcb80d09054b643b51343f77e5a4fb830cf9c72d`; threadmgr.elf: `b67f1f96e390dfe8df1af3b1889d364f7b5dfe36c07fe3c2ed4768f87e9a050f`; cached 3BE000 runtime image: `1cc1e3d6b860692f2869e592cf06d11d5c09cd15db6ad03fb4fb32f17bb23638`. The cached image is identified as cached rather than represented as a fresh full-memory dump. The native binding, deadline call and wait registers were revalidated in the fresh 572 capture.

## Implemented source and isolated validation (2026-10-01)

Root authorized the narrow model after reviewing the proposal. Source is now frozen in `src/hw/soc/timers.cpp`, `soc_internal.h`, `kermit.cpp`, and `tests/test_systimer.cpp`. The existing Cortex-A9 timer code is unchanged. Only the two reached Vita timers are installed, owned and advanced by the existing Kermit time accumulator. No machine, CPU, firmware or service code was edited.

Portable quotient/remainder arithmetic replaces any dependency on compiler-specific 128-bit integers: split PERIPHCLK ticks by the frequency denominator, multiply only the bounded remainder, retain fractional phase, compute the modular count and separately detect a full64-bit count span. CPU-agent independently checked 24,576 combinations against a Python arbitrary-precision oracle, including all256 prescalers, huge batches and fractional states. A huge batch fires an armed comparison even if its modular final counter is below the deadline.

Clarified model policies: any counter/deadline programming access, including an identical value, rearms; ACK alone and an identical already-active config write do not. A real stop/start rearms. Config changes reset divider phase; unchanged config does not. Newly armed equal/past unsigned absolute deadlines expire only on a genuine counting tick. This is a comparison-phase model choice, not a captured electrical measurement or signed half-range wrap scheduling contract. Batch-crossing detection happens before modular counter addition, so it cannot lose a comparison crossed before32/64-bit wrap. IRQ output gating by captured low pattern0xC is explicitly inferred: stoppedC retains pending compare output, stopped8/zero disable output while preserving pending status until W1C. Unsupported active profiles preserve raw diagnostics and neither count nor create events. Word status bit1 is the stated same-family inference from LT5 ACK2; overflow mode/events remain unimplemented.

The isolated command is saved at `build/goal-native-systimer-isolated/build-command.txt`, compile output at `build.log`, and test output at `test_systimer.log`: **7 tests, 0 failed cases, 0 failed assertions**. Compile has only the two pre-existing unused-symbol warnings in timers.cpp/kermit.cpp. No shared build or ordinary boot run was started by this agent.

The tests replay actual KBL LT5 register programming and unchanged ThreadMgr Low/Wide instructions, including an actual high/low/high rollover retry. They validate fractional CPU time, known divider ratios, split versus batch phase, UINT64_MAX batches, crossings before32/64-bit wrap, hardware-owned status and W1C lanes, explicit rearm/ACK behavior, unknown-mode rejection, reset and independent ports. Physical IRQ141 is enabled/targeted through actual GIC registers and confirms retained pending/masked and stopped-profile output behavior.

The physical IRQ135 test executes the full unchanged native WT7 ISR3BF000..147, native target helper3BEFE0..FFE, target-register helper3BEC2C..CC3, actual{1,2,4,8} table, and genuine CD014/CD038/CD050/CD0C4 lock helpers from supplied sysmem.elf. Its test-only architectural IRQ dispatcher acknowledges real GIC135, invokes the native handler and restores SPSR/returns. The native current==deadline path keeps the per-core lock held, acknowledges and rewrites the same deadline, then restores the timer profile; the next real tick advances the counter and raises135 again. The guest's own unlock/SEV instructions release the wait lock; the interrupted genuine LDREX/STREX waiter resumes, reacquires it and returns. No host lock write, fake handler or service-success patch is used.

These are source/isolated-test results. The actual shared-build/full-suite/ordinary-boot outcome remains pending root's integrated probe and must be recorded separately once observed.
