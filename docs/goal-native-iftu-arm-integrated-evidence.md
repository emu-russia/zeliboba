# Native os0 boot and macOS PlayStation logo — 2026-10-01

The native macOS CLI and SDL3 build now executes firmware 1.04 os0 modules and
presents the guest's white PlayStation logo on black. The actual SDL screenshot
was independently inspected by the root, hardware, graphics and CPU reviewers:
SDL screenshot (image omitted from source delivery).

## Build and input verification

All CLI, SDL3, tool and test targets build successfully:
[build log](goal-native-iftu-arm-build.log). The clean-environment full suite
passes **617 tests, zero case failures and zero assertion failures**:
[test log](goal-native-iftu-arm-tests.log). Six new peripheral tests cover DSI
frame callbacks, deferred IFTU timing, physical IRQ204/205 and unchanged native
handler/writer bytes. The separate CPU fixes cover the actual Syscon checksum
instruction sequence.

Inputs are the supplied firmware ZIP, prototype first-loader and USS-1001 Ernie
firmware. The additional retail `pch-5c-cold_first_loader.bin` also passes a
fresh bounded 300,000-slice boot, first-loader SUCCESS and os0 execution:
[retail evidence](goal-native-iftu-arm-retail-evidence.md). That bounded retail
probe does not establish retail SDL presentation.

The rebuilt image matches all **992 source files** (os0 63, vs0 929), both os0
copies, vs0 and all four SLB2 source prefixes; FAT checks pass. The independently
compared rebuild's whole-file SHA256 is
`f36b4612f3e7fe497d7d9899b16434a9208b8a0cdee06f260af3ff5109264534`.
This identifies this rebuild, not every valid console image. The suspected
original Windows image is unavailable and has not been validated. Procedure,
source comparison and read-only filesystem checks are in [EMMC.md](../docs/EMMC.md)
and [byte-verifier result](macos-emmc-bytes.json).

## Ordinary native boot

[Ordinary commands](goal-native-iftu-arm-full.cmd) and
[ordinary log](goal-native-iftu-arm-full.log) use `runm 1000000` on a fresh APFS
clone of `goal-emmc.img`. Inherited `ZLB_*` variables are removed; only
observational fault logging is enabled. No watchpoints or guest-memory edits
are used. Genuine os0 module loading and driver starts execute, reaching the
native Display producer. The final device snapshot records:

- IFTU0 A bank1, PA `1C000000`, 960×544, pitch3840, RGBA8888.
- DSI0 progressive VIC0 at the modeled 60000/1001 Hz, 23 frames.
- SPI0 299 transfers/ready edges; OLED A1 exactly18 transmitted/received bytes.
- Image clean, **10,753 sector reads, zero writes**; UART idle.

The CP15 accumulated fault counter is not claimed to be zero. No undefined
instruction halt or panic occurs in the captured run. The stage label remains
the loader's diagnostic stage; the actual native os0 PCs and display output
establish the reached milestone.

## Genuine producer, publication and guest acknowledgement

The separate no-watch [native trace](goal-arm-native-syscon-oled-iftu-arm.log)
validates the actual module names before setting breakpoints. All four native
Display mode predicates return zero. Native producer `005B7048` calls the
genuine gzip decoder; return `005B70B0` has R0=`001FE000`. Its output resides at
VA `10000000` → PA `1C000000`. Native SetFrameBuf returns zero.

The exported **2,088,960 guest bytes** exactly match the independently decoded
firmware logo, SHA256
`80c43bdc43d6fcc7f1419960726e6dc3e6be906ae0faac2cdf060204717a6979`.
Guest RAM PNG (image omitted from source delivery) is a
visualization of those bytes, separate from the actual SDL screenshot. The
offline reference was used only for comparison; it was never copied into guest
memory.

Native deferred SetInput prepares A bank1 while active bank0 is blank. At the
next DSI frame (8→9 in this trace), hardware current becomes1 before physical
IRQ204 enters the actual Lowio handler at `005AD99C` on ARM0. Native zero ACK
clears physical GIC pending word6 bit12 (`00001000`→0). The genuine `+180`
store rearms a later frame; guest code clears software pending `80000000` and
replays the same PA into old bank0 at `005AD9DC`. The first post-submission
vblank wait returns zero. Both banks then contain the guest logo descriptor.

The trace stops after 106 checkpoints and one fixed 300,000-slice no-hit window;
final DSI count 22 and active A bank0 are coherent. This independent breakpoint
schedule differs from the ordinary capture's 23 frames/bank1. No guest handler,
service result, wait release or pending token was supplied by the host.
[Detailed trace and limits](goal-native-iftu-arm-graphics-evidence.md),
[machine-readable checkpoints](goal-native-iftu-arm-graphics-evidence.json).

## Actual macOS SDL presentation

A separate fresh clone runs the native SDL executable with
`--no-rebuild --run 0 -ex "runm 1000000"`, then captures the F7 display panel
after two SDL frames. The actual 1280×720 screenshot visibly contains the
centered white PlayStation logo on the black guest framebuffer. The header
reports **960×544, stride3840, four-byte RGBA8888**. SDL initializes Metal/vsync
and a 48 kHz stereo S16 audio stream; guest audio playback is not claimed.

The UI exits zero after **108.528 seconds**, with 10,753 reads/zero writes.
Source image size and modification time are unchanged. The previously running
UI process remains untouched. [Command and metadata](goal-native-iftu-arm-macos-ui.json),
[SDL log](goal-native-iftu-arm-macos-ui.log), BMP (image omitted from source delivery).
The prior blank cold-stack screenshot and logs are preserved as pre-turnover
negative evidence.

Reproduce from `.`:

```bash
./run-macos.sh --run 0 -ex "runm 1000000"
```

Press F7 to view the guest display. On this host the bounded boot capture takes
about 109 seconds. `./run-macos.sh --cli` starts the CLI debugger.

## Explicit limits

IFTU implements only the reached ordinary mode01, exact run/setup/flag subset.
One future frame per full-word `+180=1` is an explicit model choice. Raw `+40`
stays zero because physical completion encoding is unrecovered; an internal
latch drives the physical IRQ until the native full-word zero ACK. DSI frame
events never depend on valid pixels, guest software pending or an invented
descriptor commit. Manual selection and hardware current remain distinct.
[Implementation and review](goal-native-iftu-boundary-implementation.md).

The host presents guest RGB opaquely; the source logo's alpha bytes are zero.
Full A/B blending, physical fade/timing, other modes and complete producer
return have not been established. Missing-ROM, identity/context and secure
boundary substitutions documented in [STATUS.md](../docs/STATUS.md) remain.
Full kernel startup, LiveArea and guest SGX rendering are still unfinished.

All 15 Graphics PDFs (800 pages) are indexed in
[GRAPHICS_REFERENCES.md](../docs/GRAPHICS_REFERENCES.md). They support API and
layout analysis but do not provide binary GPU command, MMU or USSE instruction
encodings. This achieved logo executes the genuine CPU/decompression/display
path and does not establish a complete SGX backend.
