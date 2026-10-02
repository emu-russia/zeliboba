# Integrated GPIO early-loader regression — 2026-10-01

The first integrated ARM RFE/GPIO/SPI2 revision builds every macOS target and
passes all 599 tests (`goal-native-rfe-gpio-spi-build.log`,
`goal-native-rfe-gpio-spi-tests.log`). The ordinary fresh-clone boot exposed a
coverage gap: real SPI0 response-ready signaling was incorrectly gated until
the secure-kernel handoff, although the second loader needs it to reach that
handoff. Passing first-loader SUCCESS alone did not preserve the boot chain.

`goal-native-rfe-gpio-spi-full.cmd/.log/.img` records a clean-environment
`runm 1000000`; only observational ARM/Secure fault logging was enabled.
The first loader reports SUCCESS and enters the second loader, but ARM release
hits its 400000-slice context budget. The final CMeP PC is `0x497FC`, in the
second loader's GPIO interrupt polling path. The trace reads status words
`E20A0038..48` at `497CA..497E0`; all are zero.

The actual configuration is direction `00FF0008`, output `00880008`, input
`00880008`, pin4 mode `00000300`, gate0 mask `FFFFFFEF`, other masks `FFFFFFFF`,
all pending status zero, zero falling edges and native phase zero. SPI0 has one
valid four-byte request and ten-byte response, with ten RX bytes still queued.
The output3 request is driven high. This is a real early response generation,
not a development JIG exchange. The newly added phase gate excluded it.

Unchanged second-loader disassembly corroborates the sequence: clear output3
at `4371A`, SPI start at `43790`, set output3 at `4379C`, then helper `49790`
queries latched GPIO pending against the native enable shadows. The separate
bounded breakpoint capture `goal-arm-native-syscon-oled-rfe-gpio-spi.log` also
ends in that second-loader poll, before os0 driver-list completion.

The corrective scope is the real validated SPI reply wire in both phases:
no peer input before a valid framed response; a real driven request rise and
elapsed peripheral tick produce an ordinary pin4 falling pulse and pending
latch. The explicit handoff phase retires the optional JIG only. No pending
status, subcallback, guest return or thread wake should be forced. These files
preserve the failed revision's evidence; later corrected captures require new
names and a new full-suite result.
