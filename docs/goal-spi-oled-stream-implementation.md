# SPI2 native OLED streaming: source-ready verification

Owned edits: `src/hw/soc/spi.cpp`, only the SPI declaration / fields in `src/hw/soc/soc_internal.h`, and new `tests/test_spi_oled.cpp`. No Kermit integration, GPIO, board-response wire, CPU, firmware bytes, frontend, DSI or IFTU change was made. The shared binary / build remain untouched; compilation was isolated against the existing 579 core archive.

## Implemented controller subset and explicit assumption

Only port2 with exact observed `CTL=0x30001` enters the new stream on a guest start1. An empty start clocks nothing. Prequeued low16 words are consumed at that start; subsequent actual FIFO writes are consumed synchronously. Each consumed word samples sixteen high input bits and adds two `FF` bytes to the existing RX queue. Start0 disarms future clocks while retaining captured input, as the genuine read helper stops before reading it. Reset clears both queues, counters and armed state. Changing CTL away from the supported value cancels the stream; restoring CTL alone does not arm it again. Other ports / modes retain whole-frame callbacks.

This is a deliberately chosen undriven pulled-high board input, **not** a recovered Vita pull value, attached OLED panel, valid supplier ID or command-specific response. Source comments, the SPI2 summary and detailed diagnostics disclose that assumption. No input profile / environment / CLI option was added. Nonzero FIFO counts reuse the existing byte queue; the native OLED code establishes only empty/nonempty and drained-zero behavior, not units, capacity, rate or overflow. Per-word completion is synchronous, with no guessed clock divisor or completion IRQ. Only the existing RX-not-empty latch / mask / W1C mechanism is used. Native INTCTL3 leaves that bit9 masked.

Diagnostic request / sampled-response snapshots retain their first 64 bytes per stream start, with full byte counters. That bound is a host diagnostic limit, not a FIFO capacity. No peripheral read fabricates progress. The model does not mutate guest RAM, ready state, imports, script returns, framebuffer state, or panel-recognition predicates.

## Unchanged guest fixture and baseline failure

The portable test embeds unchanged firmware 1.04 `oled.elf` regions linked at `0x81000160..0x81000980` (close, read packer/unpacker, worker, ready/error APIs) and its original unbound provider stubs `0x810014A0..0x81001570`. Source SHA-256 and per-region SHA-256 are recorded beside the hex fixtures. No external firmware file is required to run tests.

The isolated fixture maps ordinary code, BSS, stack and caller output, then supplies A1 / output / length5 and a return sentinel. Its existing unbound clock/reset/GPIO imports return errors; ignored clock errors remain ignored by unchanged helper instructions. The original GPIO error chooses the worker's native pin-high branch, and that boundary is explicit. This test does not claim to validate Lowio clock setup or GPIO wiring. No guest executable byte or API return is patched.

Before changing SPI, both the untouched A1 helper and worker fail to return within a bounded 5,000 instructions, while their emitted nine TX words already match the golden stream. `goal-spi-oled-baseline-tests.log` records 2 cases / 2 expected failures, using `build/research/zeliboba-spi-oled-baseline` against the original 579 library.

## Passing isolated checks

`goal-spi-oled-stream-tests.log`: 6 new cases / 0 failures, executable `build/research/zeliboba-spi-oled-tests`, compiling current `spi.cpp` separately and linking the existing shared archive without rebuilding it.

- Unchanged A1 helper returns with five `FF` payload bytes, exact TX words `030A 0804 2010 0040 0000 0000 0000 0000 0000`, 18 transmitted and sampled bytes, preserved stack / output guards, and genuine close draining the surplus RX words. At its native stop, all 18 captured bytes remain available.
- Unchanged worker copies the all-FF DDB words, rejects supplier `FFFF`, and publishes ready2. Native GetDDB, read and write APIs retain `803F0A03`, preserve caller output, and make no new transfer. Native WaitReady returns zero for completed failure while ready remains2.
- Empty start clocks no input; only actual low16 writes sample bytes. Stop retains RX, word pops decrement availability by two, and a subsequent real start consumes prequeued TX exactly once.
- RX latch / W1C and reset cancel observable pending input / IRQ state. CTL after reset requires a new guest start; no stale stream resurrects.
- Changing CTL retains already captured RX but cancels future word clocks until a new start.
- SPI0 / SPI1 and unsupported SPI2 modes retain one whole-frame callback, byte-exact native request ordering, original odd-length reply pops, and one-shot start behavior.

`goal-spi-oled-legacy-tests.log`: both existing `kermit_spi_*` SoC regressions pass against the separately compiled new SPI object. This includes the original Syscon low-byte-first four-byte request / five-byte reply path and the unattached framed port's completion behavior.

The unchanged helper fixture has no interrupts; the separate CPU owner has proven and fixed the live RFE Thumb-alignment / SYS-USR SP alias faults and separately swept interrupt boundaries. The next integrated native boot must verify the expected 18-byte transfer, ready2, actual Display continuation, real gzip / SetFrameBuf calls and guest-RAM scanout. Source-ready tests do not establish a guest logo.
