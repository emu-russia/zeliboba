# IFTU0 frame-completion proposal: independent review

2026-10-01. Read-only native/source review, no emulator source edits, builds or new boot probes. Current cold-stack graphics capture proves native producer 0x005B7048, gzip output 0x1FE000 and SetFrameBuf return0. Guest logo pixels reside in actual RAM, but the explicit-only IFTU leaves active bank0 blank and prepares the logo in bank1.

## Exact contract

Supplied Lowio1.04 text SHA256 8d6ff20781db00725bb7feb313f80f892a58ac04e947df4dbdaebaee37be881b, PT_LOAD file+0xA0, linked0x81000000, runtime0x005A8000. Cached static instruction bytes are in goal-native-iftu-599-static.txt.

- Enable 0x8100690C writes plane+0x50=1, +0x58=0x108, +0x180=cached optional flag (default1), initializes both banks, selects0 and bus-enable1. Ordinary output flag0 sets shared mode pairs01 (A bits2:1; B bits4:3), then unmasks each native plane IRQ204/205.
- SetInput 0x810065E0 snapshots current bank from plane+4 bit1 at0x81006680..82. Sync-nonzero branch0x81006754..6E fills the opposite bank via unchanged helper0x810058DC and writes software pending0x80000000|old. It does not write shared select or plane+180.
- Handler0x8100599C reads plane+40 at0x810059B8, writes0 at59BA, reads again at59BC, and writes cached optional flag to+180 at59BE. Neither status value controls its branches. If software pending is nonzero, it clears the token and replays the cached descriptor to the old bank at59C4..D8. Thus raw+40=0 is accepted; no nonzero hardware status encoding is recovered.
- Disable0x81006B6C blanks both banks, sets+50=0, zero-ACKs+40 and masks its native IRQ (6BBE..6BD8).
- Fresh capture has +180=1, ordinary mode pairs01/01, active index0, opposite bank1 PA0x1C000000/format0x10/unblanked and bank0 blank. The native software pending is0x80000000.

## Alpha mode correction

Shared mode0xA->0xB changes bit0 only, not the plane mode pairs. Native Display merge call0x005B70EE reaches Lowio0x8100688C and stores shared+20=4, ORs shared+4 bit0 (store0x810068F4). Fade setting0x80 clears that bit through the same helper. The hardware-tested primary [vita-libbaremetal source](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/iftu.c) independently names shared+4 bit0 alpha enable and +10/+18 explicit config selects. It does not establish automatic mode01, +180 or status+40 semantics. The A->B transition is not a commit signal.

## Bounded model review: clear

A narrowly labeled board model may arm ordinary plane mode01 with +50=1/+58=0x108/+180=1, flip one coherent active-bank index at the next real DSI0 frame boundary, and latch a physical IRQ204/205 internally. Raw+40 can remain0; a guest zero ACK clears the internal level, and +180write1 rearms for a later frame. This fits native prepare/old-bank-replay ordering without reading guest software pending, inventing a descriptor-store commit or choosing particular framebuffer content. Manual mode00 retains explicit selects; mode11 stays outside the supported bound.

One-frame-per-arm is an explicit model approximation. Native evidence does not prove or exclude continuous bank alternation or fully identify +180. No hardware status bit is claimed. The +180 write inside the handler precedes old-bank replay, so rearm must never flip synchronously on that write/ACK; only a later real frame can consume it. Active index must consistently drive plane+4 and scanout, while stored manual-select requests remain separate in automatic mode. Device reset/disable must deassert pending IRQ, masked delivery must retain a pending level, and neither blank banks nor non-logo frames should be producer-specific gates.

This review supports protocol implementation, not completion of the overall goal. macOS SDL visibility is still a separate validation. CPU audit of the ordinary cold-stack log so far finds no new undefined/unimplemented instruction or Kernel Panic; that file was still streaming at review time.

## Implementation review (owner source, before integration)

Reviewed IFTU write/ACK/rearm/frame/reset/scanout, DSI callback delivery and Kermit/GIC wiring, plus the new physical IRQ/native-handler fixture. The core behavior matches the bounded proposal: full-word controls only, coherent active bank and +4 readback, no producer/descriptor-validity timing gate, one arm consumed per real frame batch, native zero ACK deasserts, and reset clears events. DSI's fractional advancement and mask independence remain intact. Handler fixture copies unchanged 58DC..59EE, checks both old-bank cases, physical IAR204 and EOI, guest pending clear/replay, SP and subsequent-frame rearm; it explicitly does not claim full OS vector execution.

One actionable diagnostic-path defect was sent to the owner/root: named IftuController::poke_register added base twice, because RegisterBlock::enumerate_registers already returns absolute addresses. Named writes must use entry.address directly; numeric-offset writes still add base. The owner isolated fixture also needed the existing resolve_workspace_path declaration include. No additional material production issue found in this review. Final integration clearance depends on these owner corrections and focused validation.

## Frozen-source verdict

Clear for integration. The named-address correction is present at iftu.cpp150 and meaningful named ACK/rearm assertions cover it. The joint-plane fixture preserves both native mode pairs; IRQ204 and205 are independently acknowledged and only rearmedA turns again. Hardware current-bit ownership survives register pre-storage and partial lanes. Embedded native code is exactly276bytes from supplied Lowio58DC..59EF, independently byte-verified and SHA256 a6b196d03dc1df2ce6b06b3dd993f815acfc040288c982022ea9c1777760e21a.

Owner isolated results: IFTU8/0 and DSI9/0, one overlapping filter match,16unique tests/0. This reviewer ran no duplicate compile/test. No remaining material implementation issue found within the declared native ordinary-profile scope. Actual native IRQ handler/scanout and SDL visibility remain runtime validation, not implied by these isolated tests.
