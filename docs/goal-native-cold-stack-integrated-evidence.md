# Native cold boot and guest logo buffer — 2026-10-01

All macOS CLI, SDL3 and tool targets build. The clean-environment full suite
passes **611 tests, zero case/assertion failures**:
`goal-native-cold-stack-build.log`, `goal-native-cold-stack-tests.log`.

## Corrections at their source

The genuine Syscon response checksum now executes its VLD1/VMOVL/VADD/VPADD and
high-D scalar VMOV sequence. Twelve architectural tests and an independent
scalar oracle cover the actual unchanged firmware block; see
`goal-native-syscon-checksum-cpu-fix.md`. Native received flags/result return0.

The existing modeled second-loader handoff now obtains boot-mode bytes from
the same Ernie NVS store used by the SC protocol: NVS4A0/481/483 go to parameter
+30/+31/+33; the native cold initializer supplies +32=0. Fresh modeled NVS
therefore yields **FF FF 00 FF**. This is an early board-input model based on
the actual builder, not a claim that the complete native per-console builder
ran. The unsupported late external-boot marker was removed; product profile
word+6C remains4. Actual copied Sysmem parameters retain these values.
`goal-native-kbl-param-provenance.md` records all four native copy paths.

This exposed a separate overbroad stack workaround: it shifted genuinely
allocated 16KiB per-core stack tops beyond their allocations. The fallback is
now restricted to the historically measured shared top4000; native allocated
tops are preserved. The old failed cold capture is retained separately in
`goal-native-cold-full.log` and `goal-arm-native-syscon-oled-cold.log`.
`goal-cold-arm2-epilogue.log` observes the zero saved-PC frame before the VA0
fetch; it does not establish the exact later remapping writer. Static caller
and independent CPU review are in `goal-native-cold-stack-handoff-review.md`
and `goal-native-cold-stack-cpu-review.md`.

## Ordinary os0 execution restored

`goal-native-cold-stack-full.cmd/.log/.img` is a fresh APFS clone, ordinary
runm1000000, with all inherited ZLB overrides removed. Only observational fault
logging is enabled. The first/second loaders, modeled secure boundary, native
101/102 handshake, authenticated os0 loads and both28-module lists complete.
All14 core-module starts and Stdio/Lowio/Syscon/OLED/Display/SblSsSmComm return0.
All6 EMC commands complete. No undefined halt or ARM fault is logged in this
capture. CMeP ends in native sleep80048A. SPI0 records299 transfers/ready edges,
1438 output and3794 input bytes, RX empty. OLED A1 is exactly18TX/RX bytes;
the modeled undriven-high input is rejected with ready2/API803F0A03 retained.
The image remains clean: **10753 sector reads, zero writes, dirty=no**.

## Genuine logo producer and remaining presentation work

The independent no-watch breakpoint capture
`goal-arm-native-syscon-oled-cold-stack.log/.json` validates all four actual
module-header names before installing graphics breakpoints. All four native
Display mode predicates return0. Actual parameter VA47C0 has byte30FF,
word6C4, wordC060 and wordC4FF14.

The guest executes producer005B7048. Its actual gzip call005B70AC returns
**1FE000** at005B70B0 into **VA10000000 → PA1C000000**. Native SetFrameBuf
005B70DE returns0 at005B70E2; the native vblank wait also returns0.
The exported2088960 guest bytes exactly match the independently decoded
genuine display.elf asset, SHA256
`80c43bdc43d6fcc7f1419960726e6dc3e6be906ae0faac2cdf060204717a6979`.
Artifacts: `goal-arm-native-syscon-oled-cold-stack-guest-framebuffer.rgba/.png`.
The PNG renders these exported guest RGB bytes for inspection; it is not an
SDL screenshot. No offline asset was copied into guest memory.

Native first SetInput uses the deferred path: A bank1 has PA1C000000,
format10, width960, height544, blank0; active bank0 retains PA0/blank1.
DSI0 delivers native vblank callbacks and23 frames in the ordinary capture,
but the current IFTU implementation lacks frame-boundary turnover/IRQ204.
It consequently exports no supported enabled scanout. The shared mode A→B
change sets alpha enable, not a recovered commit bit. The next hardware work
must preserve these distinctions and let the genuine Lowio handler ACK/replay
the old bank through a physical GIC interrupt.

This is guest-produced logo-buffer evidence, not yet a native SDL logo, complete
kernel startup, measured OLED hardware or SGX execution. Existing missing-ROM,
identity/context and security substitutions remain documented in docs/STATUS.md.
