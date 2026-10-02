# GPIO0 / SPI0 ready implementation — FW1.04, 2026-10-01

Sources frozen after coordinated integration. This implements the actual reached
Syscon SPI0/GPIO248 boundary. The fresh native capture now proves the full
Syscon receive/ACK/stop route and restored loader handoff, as recorded below.
It does not establish accepted OLED identity, scanout, or the PlayStation logo.

## Actual baseline 579 evidence

`goal-native-timer-frontier-timers.log/.json` proves all six EMC commands,
physical IRQ135 and unchanged Secure timer handler, native delay return, and
Lowio SMC117 return0. `goal-arm-native-syscon-oled-timers.log/.json` and compact
`goal-native-timers-graphics-evidence.md` then prove Syscon GPIO248/sub4
registration/enable return0, SPI0 transfer32, ten response bytes queued, and
actual GPIO3 request rise; no native GPIO sub4 callback was reached.

Before GPIO3 rise: direction/readback00=A90001, input04=10, SET08=1,
CLEAR0C=8, opaque10=FF0000, mode14=300, gate0 mask1C=0; status38/3C/40/44/48=10.
After rise old shim changes00=A90009 and08=8. The old shim conflates direction
and latch. +18/+20/+24/+28/+2C/+34 were undefined, not measured zero.
GIC248 group0, enablewordFF000000, priority50, target0F, config55555555;
CPU interface0B and PMRFF. No response pin transition was inferred from these
old shim snapshots.

## GPIO implementation and explicit limits

One existing CMeP.GPIO object at E20A0000/1000 remains mirrored into ARM.
+00 is direction (hardware1=output), +34 is independent output latch;
SET08/CLEAR0C apply supplied lane bits only. Opaque10 remains a snapshot.
+04 samples driven output pins or board-provided input pins nondestructively.
Guest input/output-readback writes do not inject board levels. Named peek and
reads agree, statuses are hardware-owned W1C with exact byte lanes, and unknown
registers stay unavailable (FF) rather than accumulating writes.

Five active-high masks1C..2C resetFFFFFFFF, pending38..48 reset0. Only the
reached physical input4 falling/mode3 path latches events. Other pins/modes are
not claimed implemented. Masked retention and fanout into all five pending
latches are explicit model policies, not recovered electrical reset facts;
separate masks derive physical parent248+gate. Reached native gate0 is unmasked,
unused gates remain masked. Mode/mask/direction writes do not invent edges.

The optional development JIG is absent by default. Enabling it explicitly
asserts input4; a real driven output3 rise releases it, never a read. Native
phase disables/rejects JIG reenable until reset. Parent-owned CmepBlock wrapper
and genuine secure-kernel handoff call activate native phase only after early
loaders finish. This reuses the already modeled vector/SRAM handoff boundary;
its real board control register remains unknown. This phase retires JIG only; it does not suppress genuine SPI response-ready.
Before an actual qualified valid SPI exchange, SPI ready supplies no pin4 peer;
configuration alone does not assert one. Early polling frame bytes are unchanged.
Debugger gpo now reads actual checkpoint output latch+34.

## Native polarity correction and SPI0 board wire

`goal-native-gpio-input-candidate-review.md` and
`goal-lowio-gpio-candidate-probe.cpp/.log` supersede the prospective
persistent-low assumption in `goal-native-gpio-spi0-ready-proposal.md` step4.
Unchanged Lowio mode3 setter clears shadow+28 bit4. Unchanged parent candidate
810022A4..22CC computes ((sample04 XOR shadow28) AND NOT shadow24) AND gate
shadow. Samplelow rejects sub4; samplehigh branches810022D6 with candidate10.
PortRead81002780..27BA independently reads+34 for output/+04 for input.

Only SPI0 CTL0 fresh complete request/reply frames can schedule a ready edge,
in both the real early second-loader phase and later ARM-native phase.
Both length and one's-complement checksum must validate, optional one-byte
low16 wire padding is accepted, stale undrained RX cannot be attributed to a
new generation, and the associated request must start while GPIO3 is low then
rise while direction3 is output. Pin4 must be input/mode3 and gate0 unmasked.
A valid framed error response is transport-ready; its result bytes are not
changed or interpreted as service success. Existing Ernie opcode/result
behavior is unchanged.

After one elapsed PERIPHCLK tick, the real board input API drives pin4 low
then idle-high within that coarse tick. Delay and sub-tick pulse width are
explicit emulator timing choices compatible with unchanged native candidate
bytes, not recovered electrical measurements. Ordinary GPIO logic latches
the falling edge and derives actual GIC248; no synthetic subcallback, event,
SPI IRQ64, guest result, IRQ target, polarity override, or wake is introduced.
Reads/W1C do not change input, consume the reply or generate another edge.
Stop, RX drain, output3 low, CTL/mode/mask qualifier loss and reset cancel
scheduled work; one generation pulses once. Restoring CTL0 or another qualifier
cannot resurrect a canceled generation. Once a valid exchange establishes
idle-high, cancellation and partial SPI reset retain high, avoiding a fabricated
fall while mode3 remains active. Full GPIO reset clears modes/status/input first,
then invokes the board-wire reset callback to disconnect the peer without an
edge. Initial input absence and idle-high after exchange are explicitly modeled
board levels, not an electrical pull-up measurement. SPI0 alone is ticked by Kermit, not its mirror.
SPI2's consume_oled_words body is byte-for-byte unchanged from the frozen
pre-wire source; all six SPI2 native/transport regressions pass.

## Isolated validation

`goal-native-gpio-wire-isolated/build-command.txt` records an isolated compile
of changed GPIO/SPI/Kermit and CmepBlock allocation/phase-wrapper source
against the stable library. No shared target was built by this agent.

- 6 new GPIO tests: unchanged Lowio PortRead/checkpoints; byte lanes;
  physical pin4/masks/W1C; reset/unsupported modes; opt-in JIG phase;
  cross-bus physical GIC248 route.
- Updated obsolete CMeP GPIO test checks absent default JIG and independent
  direction/latch. It no longer requires fabricated read-consuming handshake.
- 5 new SPI ready tests: unconfigured/no-peer byte polling plus actual early
  second-loader GPIO QueryIntr/ACK and later native exchange/tick phase;
  cancellation;
  malformed/no reply/unknown mode/unassociated rise; error bytes/stale
  generation; actual GPIO248→GIC plus unchanged candidate high acceptance and
  persistent-low rejection. Candidate fixture proves eligibility only, not
  completion of the full native parent/sub4 handler.
- All 6 unchanged OLED SPI2 tests pass.
- Remaining 21 CMeP device tests pass after class/allocation ABI rebuild.
- debugger.cpp syntax check passes.

39 unique executed cases, 0 failed cases/assertions. Filter logs:
`goal-native-gpio-wire-isolated/test_gpio-early-ready-final.log` (8 including
one overlapping wire), `test_spi-early-ready-final.log` (11 including same wire),
`test_cmep-early-ready-final.log` (22 including updated legacy GPIO case).
Final compile is `build-early-ready-final.log`; initial phase-gated and narrow
CTL-cancel isolated logs are preserved separately. Initial 7-case GPIO-only proof also retained under
`goal-native-gpio-isolated`. Build warnings are preexisting unused fields,
unused GIC name and a CMeP signedness comparison. Root subsequently built all
Mac targets and ran all 599 shared cases with zero failures; the exact logs are
`goal-native-early-ready-build.log` and `goal-native-early-ready-tests.log`.

## Preserved initial integrated regression and corrective evidence

Root's initial GPIO/SPI2/CPU build passed all599 tests, then actual full run
`goal-native-rfe-gpio-spi-full.log` exposed an early-phase regression. First
loader reports SUCCESS at line46, stage2 handoff402FC at47, but ARM releases at
budget400000 at48 before real secure-kernel load. Nativephase0, GPIOdirFF0008,
out880008, mode300, gate0maskFFFFFFEF, allpending0; SPI0request4/reply10 and
GPIO3high; MeP stopped497FC polling status via497CA..E0. This proves first-loader
success only for that binary; it does not preserve native secure-kernel/auth
startup. Failed runtime remains preserved in `goal-native-rfe-gpio-spi-regression.md`
and graphics probe logs. Do not reuse its later ARM state as current boot progress.

The initial native-phase gating assumption was wrong: unchanged second loader
43896 configures pin3 output;438A0 configures pin4 input;438AA mode3;438BA unmasks
pin4;438C2 ACK. Real receive43B5E..66 polls QueryIntr49790 until pending4 exists,
then ACK49806 at43B92 writes4 to allfive statuses, drains RX43C1E..38,
stopsSPI43C62 and clearsGPIO3 at43C72. Query never reads input04. Exact static
artifacts: `goal-native-early-gpio-secondloader-dis.txt`,
`goal-native-early-gpio-caller-dis.txt`, `goal-native-early-gpio-ack-dis.txt`.

Final correction removes CPU phase from real reply qualification. Only an
actual valid fresh framed request/reply initializes idle-high, before associated
output3 rise; then the same tick-driven physical fall/high pulse latches status.
No debug-JIG read effect, guest status injection or service return is introduced.
The regression fixture executes unchanged49790..49894 bytes, verifies Query0
before elapsed tick, Query1 after pulse, unchanged ACK then Query0; byte response
remains10 until drain, and a later native exchange repeats normally. The final
isolated39/0 proof includes this missing early caller contract.

## Fresh integrated result after the early correction

The final coordinated all-target Mac build and full suite pass 599/0. Root's
fresh ordinary clone log `goal-native-early-ready-full.log` reports first-loader
SUCCESS at line46, real second-loader 5FF00 service handoff at lines50/51 and
secure-kernel entry 00800000 at line52, followed by native NSKBL Starting at73.
There is no ARM budget-release fallback in this run. All 21 named module starts
are reached, including the 14 core modules, Lowio, Syscon, OLED and Display.

The independent graphics capture `goal-arm-native-syscon-oled-early-ready.log`
and `.json`, summarized in `goal-native-early-ready-graphics-evidence.md`,
proves actual mode3 setup, parent IRQ248 at5AA298, sub4 dispatch at5AA310,
Syscon callback4F8160, ACK4F819A/19E and SPI0 stop4F8350/52. The ten queued
response bytes drain to zero. At entry input4 is high, all ordinary falling-edge
latches contain10 and only gate0 is unmasked; the native handler supplies the
candidate/dispatch without a host callback shortcut. The full-run diagnostics
record32 physical ready edges and SPI0 RXpending0.

The native OLED A1 helper completes exactly18 TX/RX bytes and keeps SP intact,
but supplierFFFF is rejected and the worker stores ready2. Display ignores the
subsequent native GetDDB803F0A03 error, follows its unchanged table path, enables
IFTU A/B/sharedbus modeA and DSI0, and modeled DSI frames advance. No producer,
gzip, SetFrameBuf or supported valid guest-RAM IFTU scanout is observed. The
fresh run stops at the separately owned Syscon SIMD decode F962070D/4F8B56.
This establishes restored native transport and head setup, not accepted panel
identity or logo presentation. Earlier phase-gated failure artifacts remain
preserved above and are not current success evidence.

First-loader baseline exact JIG bypass proof is
`goal-native-gpio-firstloader-check.py/.log`: prototype5C510 r0=21 skips5C514..56A,
SUCCESS and staged handoff,4110 reads/0writes. Prior retail541 regression is
`goal-native-retail-fill-regression-full.log`, meta JSON/result TXT, using
`../dumps/pch-5c-cold_first_loader.bin` on an isolated clone.
It proves that historical fill/cleanup baseline only, not the final GPIO binary.

## Primary independent hardware-tested references

- https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/gpio.c
- https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/include/gpio.h
- https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/syscon.c
- https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/spi.c
- https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/drivers/gpio/gpio-vita.c
- https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/arch/arm/boot/dts/vita.dtsi

These are primary hardware-tested reverse-engineering implementations, not
manufacturer electrical specifications. Supplied genuine firmware and live
native captures establish the reached 1.04 call/handler path.
