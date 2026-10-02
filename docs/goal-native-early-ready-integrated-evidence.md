# Native GPIO/OLED/DSI integration — 2026-10-01

All macOS CLI, SDL3 and tool targets build. The complete clean-environment suite
passes **599 tests, zero cases/assertions failed**. Logs:
`goal-native-early-ready-build.log`, `goal-native-early-ready-tests.log`.

The ordinary `runm 1000000` capture in `goal-native-early-ready-full.cmd/.log`
uses a fresh APFS image clone and removes every inherited ZLB override. Only
observational ARM/Secure fault logging is enabled. First-loader SUCCESS, second
loader completion and the existing modeled secure-kernel boundary are restored;
there is no ARM context budget-release timeout. Both os0 lists and all28 modules
load, and all14 core-module starts plus Stdio/Lowio/Syscon/OLED/Display/SblSsSmComm
return zero. All6 EMC commands complete; the prior native SMC117 result0 proof
remains consistent. The image records10753 sector reads, zero writes, dirty=no.

## Real received reply and intact OLED transfer

The independent clean breakpoint capture
`goal-arm-native-syscon-oled-early-ready.log/.json` records60 milestones:
GPIO248 parent, native sub4 callback, Syscon receive/ACK/stop, then OLED and
Display setup. SPI0 has32 transfers,368 output bytes,588 input bytes,32 real
ready pulses and no remaining RX. Ready derives from a validated frame, driven
output3 rise and an elapsed peripheral tick, through ordinary GPIO/GIC state;
there is no guest completion/return/event patch. Genuine unchanged second-loader
QueryIntr/ACK tests cover the early exchange that the initial599 revision missed.

The corrected ARM RFE return preserves the Thumb halfword address. Native OLED
A1 returns0 with its SP restored toAADF0 and exact18 TX/RX bytes; the previous
74-byte/SP0 divergence and later false VA24/1CDA280 writes are absent. SPI2's
input is an explicitly chosen undriven high wire, not a recovered panel/pull.
The native worker rejects supplierFFFF, stores ready2 and preserves API error
803F0A03. WaitReady returns0 for this completed failure.

## Native Display head startup, still no logo

The real OLED script GetDDB returns803F0A03. Its stack revision word atSP+6
is0058, unchanged by the error (saved upper half of a pointer, not assumedzero).
The script returns0; Display OLED setup and DSI start also return0. Actual
guest stores enable both IFTU planes/bus and DSI control1/mask2, followed by
the Display head-enabled store. The ordinary capture reports23 modeled DSI
frames, the breakpoint capture13; both retain status0 and mask2 after ACK.

No native logo producer, gzip, SetFrameBuf or valid IFTU framebuffer is reached.
The new concrete stop is **T32 F962070D / VLD1.8 {D16}, [R2]!** at Syscon4F8B56,
in its native response checksum helper. ARM0 halts at4F8B5A, other cores idle
at47969C, CMeP sleeps at80048A. The only logged memory fault is the historical
early boot-window mapping at52FEC; the later OLED context-corruption faults
are gone. Missing checksum SIMD support is being implemented separately.

This proves restored os0 bring-up and native hardware-driver execution within
the documented board/boot substitutions. It does not prove a physical OLED ID,
electrical timing, pure ROM boot, full kernel startup, SGX shaders or a guest logo.
The offline logo asset remains reference evidence and has not been injected.
