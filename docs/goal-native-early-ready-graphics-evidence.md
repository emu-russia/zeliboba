# Native graphics frontier after early SPI0 readiness (599 tests)

The fresh ordinary-default capture now passes the CMeP second-loader GPIO query, genuinely executes the native ARM Syscon GPIO248/sub4 response callback, completes the OLED A1 read with the correct stack and 18 transmitted / received bytes, and lets Display enable DSI0 despite the naturally rejected panel identity. It does not reach the firmware logo producer, gzip or SetFrameBuf in this bounded run.

Artifacts: `goal-arm-native-syscon-oled-early-ready.log`, `.json` (60 native checkpoints), and `-clone.img`. Invocation: `python3 build/goal-arm-native-syscon-oled-postneon.py --label early-ready --slices 300000 --empty-windows 1`. The script uses a fresh private clone, removes inherited `ZLB_*` overrides, and writes only debugger breakpoints / host capture files. Existing machine boot stand-ins and the explicitly chosen undriven-high OLED input remain part of the model; this is not a claim that all boot hardware or panel identity is recovered. No guest RAM, API return, ready flag or logo pixels are supplied by the probe.

## Restored handoff and native Syscon completion

The real second loader calls `0x5FF00`; the existing stage-boundary stand-in stages secure kernel `0x800000`. Native NSKBL starts and reaches the successful driver-list return at `0xCC02E`. The prior budget-release fallback does not occur in this capture.

Native Lowio interrupt-mode entry `0x5AA970` receives `{port0,pin4,mode3}`. Its GPIO0 software record is VA `0x4F2120` (PA `0x4073C120`); the mode3 store clears the polarity-shadow bit at record `+0x28`. Captured IRQ248 entry `0x5AA298` receives this record, dispatches sub4 at `0x5AA310`, and reaches the genuine Syscon callback `0x4F8160`, ACK `0x4F819A/0x4F819E`, and SPI0 stop `0x4F8350/0x4F8352`. Ten actual queued response bytes become zero pending bytes.

At parent IRQ entry, GPIO direction is `FF0009`, output `A90009`, sampled input `A90019`, mode0–15 `300`, gate0 mask `FFFFFFEF`, other masks `FFFFFFFF`, and status words `10`. The modeled ready pulse has returned idle high before software dispatch, while its ordinary falling-edge latch remains pending. Native phase is1, the development JIG peer is absent, and edge count is32. These are observations of the documented board-wire model, not measured physical pulse timing or reset wiring.

## OLED rejection and Display continuation are now observed

The native A1/len5 helper returns zero at `0x4F47AA` on ARM3 with SP `AADF0` intact. SPI2 reports exactly one transfer, 18 TX bytes, 18 sampled RX bytes, and zero remaining RX after genuine close. Native BSS receives supplier / elective words `FFFF`; the worker stores ready2 at `0x4F47E0/0x4F47E2` and returns1. This is the expected rejection of the explicit undriven-high input, not panel recognition.

Display's native WaitReady call returns zero at `0x5B4F1E`. Its weak OLED-import presence check executes at `0x5B7470`, both native IFTU-plane enable calls return zero, and the real OLED initialization script enters `0x4F4B00`.

At script GetDDB call `0x4F4B74`, output points to SP+6, with SP `0x0110AD50` mapped to PA `0x41037D50`. GetDDB returns the genuine `803F0A03` at `0x4F4B78` and leaves the stack halfword **0058** unchanged (from the saved Display-BSS pointer). The script reads that existing halfword; it was not zero or a supplied revision ID. The unchanged script ignores the API error, chooses its native table path and returns zero at `0x4F4B3A`; Display's OLED helper subsequently returns zero at `0x5B45BC`.

The native Lowio DSI status read / echo-ACK executes at `0x5AD3D8`; control1 is written at `0x5AD404/0x5AD406`. Display's DSI start returns zero at `0x5B45D6` and head-enabled store executes at `0x5B45E4`. Final DSI0 is enabled in its recovered progressive VIC0 timing with13 modeled frames, status0 and mask2. IFTU0 shared bus control is1 and modeA, but no supported enabled guest-RAM scanout exists yet.

## Bounded stop and next observations

The last no-hit window is exactly 300,000 slices. Final stage is ARM KBL, CMeP sleeps at `0x80048A`, ARM0 stops at `0x4F8B5A` after the separately diagnosed Syscon T32 SIMD `F962070D` at `0x4F8B56`, and the other ARM PCs are ThreadMgr idle `0x47969C`. The independent CPU owner is fixing that instruction family. This capture does not establish whether a Display logo predicate rejects the producer or whether subsequent work is delayed by the halt; it does not infer the reason from absent breakpoints.

The clone remains clean with10,753 reads and zero writes; Bigmac count remains1081. No logo producer / gzip / SetFrameBuf breakpoint fires and no framebuffer file is exported. The next prepared probe adds each native Display logo-predicate call / return (`FDA/FDE`, `FE8/FEC`, `FF0/FF4`, `FF8/FFC`), work return, vblank callback, IFTU SetInput/IRQ, both complete bank descriptions and Lowio plane software state. All four predicate returns must be zero before the actual `+1000` invocation can reach producer `+3048`; their semantics must be recovered from their real bound providers, not guessed or forced. No extra pre-fix boot window was run.
