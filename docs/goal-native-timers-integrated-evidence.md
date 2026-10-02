# Native macOS LT5/WT7 integration, 2026-10-01

All CLI, tools, tests and SDL3 targets build successfully. The complete suite
passes **579 tests, zero failed cases and zero assertion failures**. Logs:
`goal-native-timers-build.log` and `goal-native-timers-tests.log`.

The focused model and explicit clock/compare assumptions are recorded in
`goal-native-systimer-proposal.md`; independent architectural/arithmetic review
is `goal-native-systimer-review.md`. This is a bounded comparison-timer model,
not a recovered specification of every Vita timer mode.

## Actual firmware progress

`goal-native-timer-frontier-probe.py --label timers` ran the integrated CLI
with every inherited `ZLB_*` override removed and a fresh APFS image clone.
Only breakpoints and nondestructive observations were used. Capture:
`goal-native-timer-frontier-timers.log/.json`.

The unchanged firmware reached these milestones:

* KBL's actual LT5 start completed at ARM0 PC `4002159A`.
* GetSystemTimeLow's actual load completed at `004B7D6E`, returning
  `R0=00037D36`, rather than the former permanently zero counter.
* The Secure per-core delay entered its wait at `003BF1CE` on ARM2.
* Physical IRQ135 reached the actual native handler at `003BF000`, with
  interrupt ID `87` in R0.
* The actual wait returned at `003BF1D2` after guest interrupt handling.
* Lowio's actual SMC117 returned at `005A8098` with **R0=0**. Controller
  diagnostics show **six commands submitted, six completed, zero rejected**,
  last payload `00200000` / launch3; no calibration IRQ34 was manufactured.

The clone remained clean: **10,753 sector reads, zero writes**. The supplied
firmware archive and original reconstructed `build/emmc.img` were untouched.
No host code released the guest delay lock or replaced a service return.

## Newly reached boundary

The independent graphics capture `goal-arm-native-syscon-oled-timers.log/.json`
records 21 milestones: actual Syscon SPI0 submission and GPIO3 rise, and OLED's
actual A1/length5 helper with SPI2 start before TX. SPI0 has a queued ten-byte
reply but no GPIO4 response interrupt. SPI2 has a TX-drain wait and no RX.

The live OLED execution also diverges before controller completion: its SP is
zero and it has emitted 74 bytes. An unchanged isolated helper emits the
expected nine words / 18 bytes with its stack intact. LR7 is intentional
scratch in this helper and is not itself evidence of corruption. The separate
CPU investigation is checking native interrupt context save/restore; this
capture does not establish its final cause or a successful display frame.

The ordinary env-clean `runm 1000000` result is saved separately in
`goal-native-timers-full.cmd/.log`, with only observational fault logging
enabled. Timer-driven progress is genuine, but guest logo generation,
SetFrameBuf, DSI timing enable and presentation have not yet been observed.
