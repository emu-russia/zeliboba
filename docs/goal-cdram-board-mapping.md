# CDRAM board aperture

Added independent ARM physical RAM at `0x20000000..0x27FFFFFF` (128 MiB). The GPU and IFTU share the ARM physical bus, so they observe the same bytes. This is separate from main DRAM at `0x40000000` and the logo work SRAM at `0x1C000000`. No CMeP alias is introduced.

Primary references: [hardware-tested framebuffer code](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/display.c) uses physical framebuffer base `0x20000000`; [physical memory map](https://www.psdevwiki.com/vita/Memory_Mapping) specifies the 128 MiB CDRAM range. [CDRAM enable](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/cdram.c) uses SMC117, matching the genuine FW1.04 Lowio/SceDriverTzs capture.

The board uses the existing eager RAM allocation and zero-on-reset policy. Electrical power gating, RAM training, cache coherency and retention are outside this change. Controller completion does not prove a native CDRAM memory test or rendered frame. The observed boot logo producer requests SRAM, so adding CDRAM alone does not establish logo output.

Source: `src/machine/vita.h`, `src/machine/vita.cpp`; meaningful aperture/independence/direct-RAM/reset regression in `tests/test_machine.cpp`. Integrated CLI/SDL3 build passes; full suite 572 tests / 0 failures in `goal-native-emc-syscon-tests.log`. The new aperture/independence/direct-RAM/reset case passes. Ordinary `runm 1000000` preserves the eMMC image and completes the first EMC command, but does not yet demonstrate guest CDRAM data access or a frame.

Later579 supplies the native timer delay/IRQ135 and proves SMC117 return0/all6
EMC commands. Integrated599 retains the aperture regression and six-command
completion, and enables native DSI after GPIO/OLED progress. No guest CDRAM
memory test or framebuffer has yet been observed; the new stop is a missing
Syscon checksum SIMD instruction. Current evidence:
`goal-native-early-ready-integrated-evidence.md`.
