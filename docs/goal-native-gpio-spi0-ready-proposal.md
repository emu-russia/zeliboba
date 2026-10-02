# GPIO0 / SPI0 response-ready proposal — FW1.04, 2026-10-01

Read-only proposal. No GPIO/SPI/Ernie/debugger/machine production edits or
shared build were performed. EMC sources remain frozen. The prior static
evidence is `goal-native-spi-gpio-evidence.md`; graphics owns the fresh
post-NEON/EMC native request probe. A native SPI submission/response wait must
be captured before promoting this proposal to production.

## Current frontier and additional genuine evidence

The 560 graphics capture reaches Syscon packet construction, then stops on
the unsupported VMOV at Syscon VA4F9BEA. The graphics agent's new 572 probe
passes that instruction and observes GPIO248/sub4 registration and enable
return0, but has not yet reported an actual SPI submission breakpoint. Thus
the following is a device contract/probe proposal, not a Syscon success claim.

Parent's integrated572 runm1M subsequently completed without CPU faults. EMC
diagnostics establish only the first command completion, not SMC117 return or
all six commands. Secure ARM2 is now waiting in an exclusive-lock helper after
the native delay call, and other workers poll a zero ThreadMgr clock provider
before SPI0 start. CPU owner is auditing that timer/provider/lock path. This
proposal remains prospective; do not implement GPIO ready to bypass it.

GPIO0 is the existing `CMeP.GPIO` at E20A0000, mirrored into ARM. Expand/replace
that single object in place, not by adding overlapping MMIO devices. Native
Lowio requests a1000-byte window; current shim size100 covers used registers
but not the entire resource. GPIO1 E0100000 is a separate future task.

New native output-latch proof: Lowio PortRead at linked810027A6..27B0 loads
its hardware base and direction shadow, then tests the selected direction bit.
For hardware direction1 it reads **+34**, for direction0 **+04**. This is
explicit evidence that +34 supplies the output latch, in addition to its use
as a barrier/readback register after set/clear. Sources:
`build/goal-display-lowio-thumb.txt:3633` and supplied lowio.elf file2846..2850.
This permits gpo to read+34 rather than introducing a synthetic guest register.

The genuine MeP second loader independently initializes GPIO0/1 tables at
4950A..495EA, snapshots direction+00, modes+14/+18 and inverted gate masks
+1C..2C, and echoes +38..48 to acknowledge status. Its direction API4966A
also XORs API mode with1. SPI transmit43716..4371A clears GPIO3 through4962E;
43790 starts SPI, and4379C sets GPIO3 through495F2. These mirror the native
ARM request sequence. Its checkpoint writer4897E..48998 uses only +0C/+08.

## Current shim audit

| Behavior | Exact current implementation | Consequence |
| --- | --- | --- |
| Direction versus output | mailbox.cpp645 defines+00 as data;675..688 set/clear mutate it; +10 is mislabeled direction | Genuine direction and checkpoint latch are conflated |
| Pin read | read exactly+04 returns full stored value and consumes bit4 once | Poll/debug reads change a physical line; byte size is not masked |
| Byte lanes | RegisterFile accesses exact addresses; write ignores size | +01/+09 become unrelated map entries, rather than lanes of their words |
| Set/clear masks | Special handling only at exact+08/+0C; value is not lane shifted | Upper-byte output operations do not modify the correct pins |
| Status | Modes/masks/status are undefined ordinary storage | No input edge, masking, W1C or parent IRQ248..252 |
| Debugger peek/poke | inherited named access calls raw peek/poke | Bypasses native pin/edge/set-clear semantics; peek differs from guest read |
| Reset | RegisterFile resets only defined defaults, not its whole map | Unknown/lane entries from earlier runs survive reset |
| Checkpoint diagnostic | debugger.cpp1039..1045 reads+00 | Must move to proven output latch+34 |

Locations: `src/hw/cmep/mailbox.cpp:645`, `cmep_internal.h:461`,
`src/bus/device.cpp:58..126`. Do not change common RegisterFile behavior for
this device task; GPIO should implement its own coherent storage/semantics.

## Legacy debugger/JIG path: actually bypassed in ordinary boot

Fresh env-clean capture: `build/goal-native-gpio-firstloader-check.py/.log`,
using the existing APFS clone, no GPIO pin read added by the capture. Genuine
prototype first loader stops at5C510 with r0=21. One native BGE instruction
branches to5C56C, bypassing the negative-mode fallback5C514..56A. Its debug
call5C54A→5E4E4 is not reached before first-loader SUCCESS and second-loader
handoff4CBE6. Clone4110 reads/0 writes, dirty=no. Existing boot substitutions
remain transparently logged; this capture proves only the branch/JIG usage.

The native mailbox_debug_sc body at5E4E4 first writes direction8 at5E4FA,
reads pin4 at5E4FC and, if high, publishes E0000020/24, raises output3 at5E558,
then polls pin4 until low at5E560..566. It later consumes debugger mailbox
data and lowers output3. Pin4 absence takes a native no-peer return, rather
than requiring every GPIO read to behave as a debugger transaction.

Proposed legacy policy: retain an explicitly enabled development JIG peer,
separate from the Ernie peer, and only during pre-native-ARM boot. When enabled,
its pin transition follows the actual output3/request-mailbox transaction;
ordinary reads and debugger peek never consume it. A board phase setter at
the already-modeled secure-kernel/ARM handoff would disable the JIG peer, or
default-disabled standalone tools can enable it explicitly. Any required
machine-phase call is a separate parent-owned integration change, not an edit
authorized in this read-only task. Do not infer the producer from the target
bus.context: ARM DeviceMirror accesses retain the CMeP target bus's context.

## Bounded GPIO0 contract

- +00 direction, hardware1=output/0=input, as established by genuine1.04 API
  inversion and PortRead. +10 remains an opaque boot-written snapshot; no
  unknown direction/ready effect is assigned to it.
- Independent 32-bit output latch: supplied +08 bits set, +0C bits clear.
  +34 reads the latch nondestructively. Checkpoints16..23 retain the actual
  guest writes; pin3 signaling cannot corrupt direction or checkpoints.
- +04 reads sampled pins nondestructively: driven outputs from latch/direction,
  inputs from board peers. Ernie response-ready pin4 is an external input;
  GPIO register writes cannot invent its level or a ready response. Idle-high
  pin4 is a board-wire model choice to make the first active-low reply edge
  possible, not a dumped pull-up/reset-state measurement. Keep legacy peer
  selection explicit where the physical line is reused during boot.
- +14/+18 store packed two-bit interrupt modes. Primary hardware-tested header
  gives0=high-level,1=low-level,2=rising,3=falling. Initial bounded support can
  require the actually observed **pin4/input/mode3** path; other modes must be
  diagnostic/unsupported until their reassertion behavior is included and tested.
- Five masks +1C..2C are active-high: bit1 blocks that pin. Five status words
  +38..48 are W1C, supplied byte lanes only, hardware-owned on writes/peek.
  Each parent line248+gate derives from `(pending[gate] & ~mask[gate]) !=0`.
  Recompute on edges, mask changes, W1C and reset. A masked event can remain
  latched and become deliverable upon unmask; mask changes cannot create an
  input transition. Multi-gate edge fanout is not established by the capture:
  for the initial pin4 route, set only the enabled native gate(s), and label
  whether masked-event retention is a model assumption. Capture the actual
  masks before deciding that routing policy; do not assert all five IRQs.
- Reset clears latch, pending edges, modes and peer transaction state; chosen
  reset masks must be documented. All-masked is a conservative model reset
  default until native reset values are recovered. No old unknown-register
  map entries may survive. Unknown registers stay unavailable/snapshot-only.
- Native reads, named peek and summary must agree without draining state.
  Poke routes through the same register rules; input/pending injection is a
  separate explicit board/debug API, never an ordinary status write. Byte,
  halfword and word lane access must be coherent at aligned word boundaries.

## Board wire: valid SPI0 reply plus output3 request/release

Retain current SPI0 control0, prequeued complete frame, one Ernie callback.
Do not send individual FIFO words through the Ernie packet parser or change
proved first-loader crypto transport. SPI2 streaming stays a separate mode-
gated task after an actual OLED drain wait; SPI1 remains unattached.

Native Syscon starts SPI at linked8100011E **before** raising GPIO3 at12A;
current Spi::start_transfer already creates RX synchronously at start. A
response callback at that point must not prematurely call the GPIO handler
before the guest's request state/pin3 is ready.

Proposed endpoint state machine:

1. A real framed SPI0 transaction latches its response generation. Use actual
   bytes returned by the existing responder; a nonempty structurally valid
   reply may carry an error result and still means transport-ready. No reply,
   malformed request or unsupported mode cannot create a successful reply.
   Never infer service success from byte availability, GPIO level or status200.
2. Wait until the associated output3 request/release rise (while direction3 is
   output) and valid response exists. Only then schedule pin4 high→low after
   **one PERIPHCLK tick**, an explicit emulator latency. Tick before output3
   rises does nothing; GPIO reads do not advance the state.
3. The ordinary GPIO edge path latches pin4 and derives only the unmasked
   native gate IRQ. GIC248 dispatches the registered Lowio gate0 handler,
   which W1C-acks and invokes subinterrupt4. No direct guest callback/event.
4. Keep pin4 low through repeated reads and W1C acknowledgement. Do not
   regenerate an edge merely because the handler clears pending status.
5. RX drain, native stop+10=0 and output3 low cancel delayed work/rearm the
   line high for the next valid generation. Capture establishes exact native
   order; treat early abort/drain as cancellation, not a second completion.
   Returning high must not itself raise a falling-edge event. GPIO pending
   bits still require normal W1C acknowledgement; draining SPI is not GPIOACK.
6. Reset cancels generation/timer and lowers parent levels coherently. A
   partial reset or stale response from before reset must never assert248.

Delayed ownership can live in **SPI0::tick**, called once from Kermit by the
owned device name, with callbacks for output3 and external input4; the GPIO
object already mirrors across buses. Kermit's current needs_tick excludes SPI
and CMeP.GPIO, and DeviceMirror does not forward tick. Add SPI0 explicitly;
do not tick the same object through both mirrors. Existing attach_syscon_spi
already sees both buses and can find the single GPIO object for wiring, so
normal endpoint wiring need not require a new machine-side device creation.
GPIO IRQ callbacks connect to actual Kermit GIC IDs248..252, not SPI's currently
unused callback ID0 and not64+port. Existing GIC configured target/priority
continues to select the core; no forced target or thread/event wake.

Ernie limitation: ernie.cpp630..634 already returns unknown-command error01,
but known unmodeled commands receive an empty OK record. This proposal must
not broaden that fallback or claim such replies prove hardware success.
Malformed requests also currently get an ACK80 envelope in ernie_spi.cpp;
do not turn this existing fallback into newly claimed valid execution. A
future missing Ernie command needs its own genuine opcode/input evidence.

## Required capture and test boundaries

Graphics owns the fresh actual gate capture. Need before/after: mapped GPIO
direction/+34/+04 without read consumption, pin4 mode3, all five masks/status,
GIC248 configuration, valid packet RAM, SPI0 counts/status, first start+10,
output3 rise, actual parent/subinterrupt handler, final native request result.
In particular recover the native gate masks to settle event fanout/retention.

Focused future tests should exercise:

- Genuine NSKBL/CMeP checkpoint clear/set sequence and Lowio output/input
  PortRead; direction and checkpoints remain independent; gpo uses+34.
- Repeated byte/halfword/word reads and named peek cause no pin transition;
  set/clear/W1C lane isolation and reset restoration, including old stray words.
- Pin4 falling edge, masking/unmasking, W1C zero/partial writes, absence of
  duplicate edge while low, true parentIRQ248 and independently bounded gates.
- SPI0 native frame/start-then-output3 ordering, response before/after rise,
  no edge until the peripheral tick, RX/error response integrity, malformed/
  absent slave/unsupported mode refusal, drain/stop/low cancellation and repeat.
- Explicit legacy JIG output3-triggered peer release only while enabled in
  the boot phase; ordinary native/ARM reads cannot consume Ernie ready state.
- Both bus endpoints share exactly one GPIO/SPI object; IRQ and reset callbacks
  remain coherent. Full ordinary boot must preserve first-loader milestones.

These tests model register/transport behavior, not duplicate the firmware's
request parser or synthesize successful Syscon service results.

## Primary citations

- [Hardware-tested GPIO register and interrupt implementation](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/gpio.c)
  supplies offsets, active-high masks and status acknowledgement.
- [Primary GPIO encoding header](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/include/gpio.h)
  defines mode3 falling edge and physical pins3/4. Genuine1.04 determines its
  own API-to-hardware direction inversion.
- [Hardware-tested Syscon transfer sequence](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/syscon.c)
  raises output3 after write completion, acquires input4 interrupt before RX,
  and lowers output3 after receive/stop.
- [Hardware-tested SPI framing register path](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/spi.c)
  corroborates queued TX/start, count-based RX and write0 stop.
- [Primary Vita Linux GPIO driver](https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/drivers/gpio/gpio-vita.c)
  implements mask-bit clearing for unmask and status W1C; unused direction
  labels conflict with its own native path and do not override genuine1.04.
- [Primary board device tree](https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/arch/arm/boot/dts/vita.dtsi)
  specifies GPIO0 parents GIC_SPI216..220 (hardware248..252), level-high, and
  Syscon pin4 falling-edge/pin3 transmit wiring.
- Supplied genuine first_loader/second_loader/Lowio/Syscon code and the actual
  captures above remain primary for FW1.04 behavior and current reached state.
