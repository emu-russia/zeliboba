# Genuine FW1.04 SceEmcTop / CDRAM initialization — 2026-10-01

Initial read-only diagnosis against the integrated 560-test binary. The
captures below precede controller implementation; they contain no SMC return
replacement, controller success injection, shared build or original-image
changes. The authorized bounded implementation is described separately below;
its eventual ordinary-boot outcome is not yet captured here.

## Reached state and exact identification

`build/goal-native-thread-context-full.log` reports Lowio start OK. ARM2
then executes Secure SYS Thumb at `0054BB8C..92`, polling `E8200024` bit0.
This is the next reached hardware boundary, not the prospective GPIO/SPI
boundary.

An env-clean ordinary run on APFS clone
`build/goal-native-e820-clone.img` reproduces it. Captures:

- `build/goal-native-e820-final-capture.py/.log`: final registers, translated
  code/global/stack pages, physical register reads, and clone eMMC state.
- `build/goal-native-e820-command-capture.py/.log`: six genuine breakpoints
  from SMC entry through first command launch, preserving actual bus traces.
- `build/goal-native-e820-VA00549000.bin` and text pages at VA54A000,
  VA54B000, VA54C000, VA54D000: physical saves obtained after Secure translation.
- `build/goal-native-e820-driver-tzs.elf`: exact ELF extracted from supplied
  `Out/SLB2_dec/kernel_boot_loader.self.seg01`, offset36574, length3CA0.
- `build/goal-native-e820-driver-tzs-selected.txt`: known native function
  entries decoded independently; no reliance on mixed ARM import/data decoding.

At the first stop: `SCR=4`, `CPSR=2000013F` (Secure SYS Thumb), MMU on,
`TTBR0=40108000`, `TTBR1=4010C04A`, no MMU fault. Actual translations:

| VA | PA | Meaning |
| --- | --- | --- |
| `0054BB8C` | `401C3B8C` | SceDriverTzs native command poll |
| `280C1000` | `E8200000` | Device page, L2 descriptor E8200417 |
| `00549140` | `401CA140` | SceDriverTzs EMC globals |
| `00546290` | `4019E290` | Actual Secure stack |

The bytes at VA54BB8C are `51 6A 11 F0 01 0F FB D1`; they match the
supplied segment at file381A0, inside genuine **SceDriverTzs**. Its text is
linked81000000, relocated54A000; data linked81003000, relocated549000.
The poll is linked81001B8C. ELF SHA256:
`30b85f52e414a44452a5cb89d0379908d5ae7dac72539184bdfd11948c0b8c9c`.

Lowio's linked810039D4 queues callback8100398D named `SceCdramInit` via
ThreadMgr. Callback8100398C calls linked81000090, which executes SMC117
at81000094. At runtime these are5AB9D4,5AB98C,5A8090. DriverTzs native
initializer maps **E8200000/1000**, label **SceEmcTop**, and registers:

| Service | Native linked handler | Actual runtime handler |
| --- | --- | --- |
| SMC117 | 81001C69 | 0054BC69 |
| SMC118 | 81001911 | 0054B911 |
| SMC119 | 810018F1 | 0054B8F1 |

Live SMC117 entry54BC68 receives R0=0 and reaches init54BA8C with R0=0.
It sets native once-only global `[549140+20]=1` before initializing. Do not
overwrite this state or replace the service return.

## Controller register evidence

The controller mapping is absent from current `kermit.cpp`. Thus existing
physical reads return FFFFFFFF and writes are dropped; this is sufficient to
explain the spin without an MMU/decode fault.

The init body begins linked81001A8C / runtime54BA8C. It disables IRQ34,
loads genuine global configuration, programs the following values, then
executes its native command sequence. Actual write traces are at capture
log lines3747–3820:

| Offset | Actual initial write | Native instruction runtime PC |
| --- | --- | --- |
| `00` | `00002051` | 54BABE |
| `04` | `00072233` | 54BAC4 |
| `08` | `07725245` | 54BAD2 |
| `0C` | `00001414` | 54BAD8 |
| `10` | `001F0704` | 54BADC |
| `14` | `001F0C0F` | 54BAE2 |
| `18` | `0000020B` | 54BAE4 |
| `1C` | `00000006` | 54BAE6 |
| `38` | `0000008C` | 54BAE8 |
| `3C` | `69462300` | 54BAEC |

Timing/configuration meanings beyond these native writes are opaque; preserve
readback and diagnostics without inventing full bitfields. The value2051 is
global2251 with bit200 cleared; final common path ORs600 into it and writes
2651 at54BB46.

### Command and busy registers

Every command writes payload to **+28**, reads **+24**, preserves bits30,
ORs the launch bits, writes +24, executes DMB, and waits while read bit0 is1.
This is a command engine with a self-clearing busy bit, not an all-zero
register file.

SMC117 receives cold-path argument0 in the live capture. Native code's
subsequent exact payload/launch sequence is:

| Payload at +28 | Launch bits ORed at +24 | Busy-poll runtime PC |
| --- | --- | --- |
| `000E0000` | 1 | 54BB8C |
| `00040400` | 1 | 54BBCC |
| `00020000` | 1 | 54BBF0 |
| `00020000` | 1 | 54BC14 |
| `00000031` | 1 | 54BC36 |
| `00200000` | 3 | 54BC5C |

An alternate argument1 branch executes the first payload only, before the
common final-enable path. There is a conditional JEDEC interpretation: if
bits19:17 encode RAS/CAS/WE, the sequence is NOP (7), precharge (2) with A10=1,
refresh (1) twice, then mode-register (0) values. Candidate bank bit21 makes
the last command select MR1. The primary DDR2 command truth table linked below
supports these names, but it does not establish Vita's EMC field layout or
its custom DRAM electrical configuration. The implementation accepts the
exact captured payloads, without a general JEDEC decoder or guessed MR fields.

**Actual first command is reached:** writeE0000 at54BB7A, readFFFFFFFF at
54BB7C, write31 at54BB86, then poll54BB8C. The launch31 derives from
`FFFFFFFF & 30 | 1`. With a real reset-idle +24=0 the same unchanged guest
will write **1**. Do not hardcode31 as the only acceptable launch control.
Guest code preserves30, so those bits must remain coherent across commands.

### SMC118 / SMC119 controller mode handshakes

These routines are registered genuinely, but their execution is still
prospective in this capture:

- SMC118 linked81001910 checks +24 bit20; if clear, writes10 to +24 and
  polls until read bit20 becomes1 at81001930. This establishes request10 /
  acknowledged-mode20 as a controller transition. A self-refresh/power mode
  is plausible but its label is not proved here.
- SMC119 linked810018F0, if bit20 is set, writes0 to +24 and returns0.
  This is the reverse native mode transition, distinct from command launch.

### Calibration IRQ path is separate

Native init81001A3C registers hardware IRQ **34** with callback8100193D,
priority50, target CPU maskF, type0; it does not use a SPI/GPIO IRQ. The init
disables IRQ34 first, then enables it in the common return path.

Common final setup writes `+230=0,+244=1,+234=0,+230=1` before enabling IRQ34
and returning0. Genuine handler8100193C reads **+240 bit0**, writes **+244=1**
to acknowledge, and if bit0 was set copies native global+10/+14 into
controller+38/+3C, then clears+230. This looks like calibration state, but
its trigger, latency and precise interrupt enable meaning remain unproven.
Do not synthesize IRQ34 or calibration-success status to unblock a command
busy poll. Keep those registers diagnostic and unsupported until evidence
requires a coherent calibration implementation.

## Authorized bounded implementation, ordinary-boot result pending

Production source: `src/hw/soc/emc.cpp`, `soc_internal.h` declarations and
`kermit.cpp` install/ownership/tick integration. CMake already discovers new
source and test files through CONFIGURE_DEPENDS globs. No machine-source edit
is part of this controller change.

- Aperture E8200000/1000. Defined native words preserve opaque snapshots;
  undefined words read FFFFFFFF and ignore writes. Reset control/status is0
  because there is no outstanding modeled command or acknowledged mode.
  This is a controller-reset model choice, not a measured hardware reset dump.
- Only payloads E0000,40400,20000,31 with launch1, and200000 with launch3,
  are accepted. Control bits outside33 or mismatched launch/payload pairs are
  rejected. Preserve modifier bit1 and controller-owned mode bits30 across
  completion; clear only busy bit0. No dependency on the order of already
  known commands is invented.
- Accepted commands become busy immediately, latch the payload/control for
  diagnosis, and complete after **one PERIPHCLK tick**. This deterministic
  latency is an explicit emulator assumption, not a physical timing claim.
  Polling without a tick cannot complete a command. Command/completion/reject
  counters, last payload/control, and reset count are host diagnostics.
- Unknown operations enter a sticky unsupported/busy state. Later known
  submissions cannot erase the unknown operation; explicit control0 cancels
  it. This fail-closed policy is deliberate, not a claim about real hardware
  timeout or recovery behavior. Power reset clears all pending state/counters.
- Native mode request10 (including the native write30 echo) sets request
  state; a peripheral tick derives hardware acknowledgement20. Reads expose
  10 while pending and30 after acknowledgement. Command echoes of20 cannot
  inject acknowledged mode. Native control0 reverses the mode/cancels pending
  work immediately. The electrical name of this mode remains unproved.
- Byte/halfword accesses merge lanes. Only an access containing the low
  control byte submits/cancels; upper-byte writes stage input but cannot clear
  busy, request mode or set hardware status. A later low-byte submission
  rejects previously staged unknown flags. Whole-word writes are atomic.
- Calibration +230/+234 remain snapshots; +240 is hardware-owned status0;
  +244 acknowledges supplied bits W1C, never sets status. There is no modeled
  calibration event or IRQ34 assertion. Final native configuration writes can
  be retained without pretending calibration occurred. Command completion
  is independent of this calibration path.

Focused tests in `tests/test_emc.cpp` replay the actual first-command ARM
instructions and unchanged SMC118/119 handlers, the exact cold six-command
sequence, mode-bit ownership, lane boundaries, unsupported refusal, pending
reset cancellation, diagnostic mapping and absence of calibration IRQ. These
tests do not establish that native SMC117 or Lowio returned in ordinary boot;
that requires the parent's fresh integrated live capture.

Isolated validation passed **7 tests / 0 failures**, saved in
`build/goal-native-emc-isolated/test_emc.log`. The isolated compile command is
saved beside it and links against the frozen shared library without rebuilding
shared artifacts. The sole compiler warning is the existing unused
`kGicDeviceName` in kermit.cpp. Controller sources and tests are now frozen for
the parent's coordinated integrated build.

Subsequent integrated572 ordinary run (parent, runm1M) observes EMC
`commands=1 completed=1`, and no CPU fault. This proves the **first** exact
native command completed. The native delay/provider/lock path now stalls
before the remaining command sequence; it does not prove SMC117 returned,
all six commands completed, calibration succeeded or CDRAM data was accessed.
The CPU owner is investigating that independently; no GPIO/IRQ workaround
belongs in this controller to bypass the native delay.

Separate backing limitation in the captured 560-test binary: the board had
**no RAM at physical20000000..27FFFFFF**, the public 128MiB CDRAM aperture.
The root agent has separately added independent ARM physical RAM in
`Vita::build_board()`; integration is pending. See `goal-cdram-board-mapping.md`.
Controller busy completion alone does not establish a native CDRAM memory
test or rendered frame.

## Later integrated timer and GPIO revisions

The initial capture above is historical. Integrated579 adds native LT5/WT7
time and physical IRQ135, so the unchanged Secure delay handler releases its
per-core wait and SMC117 returns0. That dedicated capture is
`goal-native-timer-frontier-timers.log/.json`.

The later integrated599 ordinary capture `goal-native-early-ready-full.log`
retains all6 completed commands, zero rejected commands, one reset, final
payload00200000/control00000003/status00000002 and calibration IRQ34 low.
Independent128MiB CDRAM backing is installed and its board regression passes.
SPI0/GPIO248, OLED A1 and native Display/DSI advance; the next stop is a missing
Syscon checksum SIMD instruction. This proves completion of the observed
protocol and native delay, not physical DDR calibration, a CDRAM memory test
or a rendered guest frame.

## Primary references

- [Hardware-tested baremetal CDRAM enable](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/cdram.c)
  directly executes SMC117 with four zero arguments, corroborating the exact
  observed Lowio service path.
- [Reverse-engineered physical memory map](https://www.psdevwiki.com/vita/Memory_Mapping)
  names E8200000..E8200FFF Secure SceEmcTop and 20000000..27FFFFFF VRAM.
- [Primary DDR2 manufacturer command truth table](https://www.issi.com/WW/pdf/43-46DR81280-16640.pdf)
  gives the RAS/CAS/WE NOP, precharge, refresh and mode-register commands;
  this supports the conditional interpretation only, not Vita EMC bitfields.
- The supplied genuine FW1.04 SceDriverTzs ELF is the primary authority for
  the register protocol and runtime command sequence above. Public sources
  located during this audit do not document its complete command bitfields.
