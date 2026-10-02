# Firmware 1.04 display and boot-logo contract

Recovered 2026-10-01 from the supplied `display.elf` and `lowio.elf`, following
the [Graphics PDF review](GRAPHICS_REFERENCES.md). This contract describes the
native guest path. Ordinary cold boot now executes its logo producer, gzip and
SetFrameBuf; exported guest pixels match the embedded asset. Native IFTU
publication and the actual SDL capture now show the white PlayStation logo on
black. Timing/rearm/status choices are bounded below; full hardware behavior is
still being recovered.

## Inputs and address convention

| Input under `Vita_104_Firmware/Out/fs_dec/os0/kd` | SHA256 |
|---|---|
| display.elf | `86bbe8a1944f7657de70253c9a96c71b56bc442f70dc49c7dfe2390b6141ed91` |
| lowio.elf | `8d6ff20781db00725bb7feb313f80f892a58ac04e947df4dbdaebaee37be881b` |

Addresses `0x8100....` are linked addresses within the named module, not asserted
runtime VAs. Both first segments begin at VA `0x81000000`, file offset `0xA0`.
The supplied `db.yml` names firmware 3.60; its NID names are labels only. All
instructions, tables and constants here come from the actual 1.04 binaries.

Detailed recovery and selected disassembly are retained in
[`build/goal-firmware104-display-contract.md`](../build/goal-firmware104-display-contract.md)
and [`build/goal-firmware104-dsi0-timing-contract.md`](../build/goal-firmware104-dsi0-timing-contract.md).

## Native logo producer

The gzip asset at `display:0x81004A60` / file+`0x4B00` consumes `0xE85` bytes
and expands to `0x1FE000` = 960 × 544 × 4 bytes. It contains the centered white
PlayStation symbol on black. Its first three channels are identical and the
fourth byte is always zero; this monochrome image cannot establish channel order.
Raw SHA256: `80c43bdc43d6fcc7f1419960726e6dc3e6be906ae0faac2cdf060204717a6979`.

The real routine at `display:0x81003048` allocates a 2 MiB memblock of type
`0x6020D006`, requesting physical address `0x1C000000`. It obtains the guest VA,
calls the imported gzip decoder at `0x810030AC`, flushes caches, and calls
`SetFrameBufInternal` at `0x810030DE` with
`{size=0x18, base=guestVA, pitch=960 pixels, format=0, width=960, height=544}`.
It then fades through alpha 0,2,...,254, waiting for a vblank each time, and sets
merge mode `0x80`. The ordinary display init thread calls this routine at
`display:0x81001000`; boot predicates can skip it.

This bitmap uses CPU decompression and Display/Lowio scanout. It does not require
SGX shader execution. The emulator must execute the producer and read the guest's
pixels; copying the offline asset into guest RAM would not verify the boot path.

## IFTU mapping and scanout

Lowio's table at `0x81009498` identifies these physical windows:

| Plane | Registers | Shared control | IRQ |
|---|---|---|---:|
| IFTU0 A / B | `E5020000` / `E5021000` | `E5022000` | 204 / 205 |
| IFTU1 A / B | `E5030000` / `E5031000` | `E5032000` | 206 / 207 |
| IFTU2 | `E5040000` | none in this table | 255 |

Each plane has descriptor banks at +`0x200` and +`0x300`. Native helper
`lowio:0x810058DC` writes the framebuffer PA at bank+0, input format at +`0x40`,
width/height at +`0x44/+0x48`, blank state at +`0x4C`, and row-padding bytes
at +`0x54`. API format 0 becomes IFTU format `0x10` (RGBA bytes). The preliminary
SDK API's format 3 is a different namespace. **Bank+4 is not stride**:
row stride for this subset is `width*4 + padding`. The logo uses 3840 bytes.

Native Lowio Enable writes plane+`0x50`=1 and Disable writes zero; the model uses
this observed pair as a conservative scanout gate without assigning a proven
hardware bit name. Shared+0 bit0 enables the bus; explicit bank selects are
shared+`0x10` (A) and +`0x18` (B). Native deferred-update code reads plane+4 bit1
as the active-bank index. The public
[vita-libbaremetal IFTU code](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/iftu.c)
corroborates bus enable, explicit selection, format and bank blanking; it does
not establish the meaning of plane+`0x50`.

Deferred `SetInputFrameBuffer` writes the opposite bank and records a pending
descriptor. Native IFTU handler `0x8100599C` reads plane+`0x40`, writes zero there,
writes its cached flag to +`0x180`, and replays the descriptor into the old bank.
Automatic turnover at a frame boundary is consistent with this sequence.
The reached ordinary per-plane mode pair is binary01; shared mode A→B sets
alpha-enable bit0, not a bank commit. Exact physical status encoding and the
complete role of +`0x180` remain unresolved. Blend/fade
writes target B-plane local+`0x8C/+0xA0` and shared+`0x20/+4`; the full blend
equation is not recovered.

The bounded [native IFTU0 model](../src/hw/soc/iftu.cpp) reads validated guest RAM
directly from its active bank, supports format `0x10`, dimensions and byte
padding, and respects bus/plane/blank enables. DSI0 supplies real modeled frame
boundaries independently of IRQ213 masking. With the reached ordinary mode01,
bus=1, run+50=1, setup+58=108 and flag+180=1, a full-word +180=1 arms one future
frame. At that boundary the hardware current bank changes before physical
IRQ204/205. A frame batch consumes one arm once; native rearm applies to a later
boundary, allowing old-bank replay to finish first.

**One frame per arm is an explicit model choice.** Descriptor validity, RAM
pixels and guest software pending never trigger or gate these hardware events.
Raw plane+40 reads zero because its encoding is unknown; an internal latch
drives the GIC until a full-word zero ACK. Partial ACK/rearm stores do not issue
these operations. Manual mode00 keeps explicit selection; unknown10/11 are
inert. The stored manual selector remains separate from hardware current bit1.
Stop, cancellation and reset behavior are deliberately bounded. Scaling,
blending and special modes remain unsupported. Six new regressions cover
boundary timing, physical IRQs, partial accesses, reset/stop and actual native
handler/writer bytes. See the
[implementation and model limits](../build/goal-native-iftu-boundary-implementation.md).
The old assumed `E2100000` controller remains a synthetic test fixture.

## DSI0 timing and guest vblank route

Lowio maps DSI0 at **`E5050000`, size `0x1000`, IRQ213**; DSI1 at `E5060000`
uses IRQ210. Native StartDisplay (`lowio:0x8100517C`) for head0/VIC0 writes
control+0=1, progressive+4=0, horizontal+8=`0xC4E`, vertical-total+`0xC`=`0x252`.
It reads status+`0x50` and writes the same word to ACK, then writes mask+`0x54`=2.
An isolated execution of these actual instructions returned zero after 205
instructions; only synchronization imports were stubbed. Startup has no DSI
readiness polling loop.

The native top handler `0x81003AE0` acknowledges status with the same W1C pattern
and triggers enabled subinterrupts. Display registers IRQ213/sub1 with callback
`display:0x81002940`, establishing **status bit1 as vblank**. This callback
increments the guest counter, pulses its wait event, and requests SGI8. Hardware
timing must raise IRQ213 through the ordinary GIC so these guest instructions run.

Display stores refresh-rate float `0x426FC29E` = 59.94005585 Hz. A bounded VIC0
model can consume the existing peripheral microsecond clock using phase
`ticks*60`, threshold `1001000`, retaining the remainder (60000/1001 Hz).
First-frame phase and status latching while masked are explicit model choices.
Control modes 2/3/4 and arbitrary PHY clocks are not established by mode1 evidence.

The [native DSI0 model](../src/hw/soc/dsi.cpp) now implements this progressive
VIC0 subset, fractional timing, coalesced status, byte-lane W1C and mask-controlled
IRQ213. Focused tests include the physical board/GIC ACK/EOI route and the
frame callback's independence from IRQ213 masking.
The integrated suite passes 617 tests; ordinary cold boot starts all 14 core modules,
Stdio, Lowio, Syscon, OLED, Display and SblSsSmComm. All six SceEmcTop commands
complete; SMC117 returns zero. The genuine GPIO248/sub4 receive/ACK/stop path
now executes. The RFE Thumb return correction preserves the OLED stack and its
exact18-byte A1 transfer. SPI2 samples a chosen undriven-high input; the guest
rejects supplierFFFF and stores ready2, preserving API error803F0A03.
Display WaitReady and OLED script return0, despite that native rejection.
Native head setup enables both IFTU planes/bus and DSI0 control1/mask2, and
modeled frames advance. Syscon's native checksum SIMD now executes correctly.
NVS-derived early boot flags and preservation of native allocated per-core
stacks let all four Display predicates return0. Actual producer005B7048 calls
gzip, which returns1FE000 into VA10000000/PA1C000000; SetFrameBuf and vblank wait
return0. Exported guest pixels have the asset's exact SHA256 above.
The actual first SetInput is deferred: it prepares bank1 while currentbank0
is still blank. The next modeled DSI frame changes current to1 and dispatches
physical IRQ204 into Lowio at005AD99C. The native zero ACK clears IRQ204 pending
word6 bit12 (00001000→0); +180 rearms, guest software pending80000000 clears,
and native replay at005AD9DC fills old bank0 with PA1C000000. The first
post-submission vblank wait returns0. No guest service or handler was invoked
by the host. The ordinary runm1000000 capture exports bank1 scanout at960×544,
pitch3840, after23 DSI frames. A separate native SDL capture visibly shows the
white PlayStation logo on black. See
[integrated evidence](../build/goal-native-iftu-arm-integrated-evidence.md),
[native trace](../build/goal-native-iftu-arm-graphics-evidence.md) and
SDL screenshot (image omitted from source delivery).
The older blank cold-stack captures are preserved. These observations do not
establish measured physical panel timing or complete fade behavior.

Shutdown queues packets at +`0x500` and polls +`0x414` bit24; DisableHead polls
the low nibble of +`0x48`. Packet completion remains unsupported. Do not synthesize
these readiness results or claim accurate shutdown from the startup probe.

## Evidence still needed from ordinary boot

- Full logo fade completion and native producer return; IRQ213/sub1, first
  wait release and actual SDL guest pixels are already observed.
- Physical IFTU status encoding, continuous-versus-rearmed timing and blend equation.
- Other display modes, scaling, packets/shutdown and complete kernel startup.

See [STATUS.md](STATUS.md) for the current boot frontier. The PDF references do
not provide a binary SGX command stream or USSE ISA specification.
