# Next integrated native Syscon/OLED capture

Prepared after the frozen 560 graphics probe. `goal-arm-native-syscon-oled-postneon.py` is syntax-checked and has not run against an integrated EMC/NEON binary. Parent must signal that binary and full suite are ready. Default output suffix is `emc-neon`; existing outputs are never overwritten. The probe makes a fresh APFS clone, removes every `ZLB_*` variable, and validates real native Syscon/Lowio/OLED/Display module headers before installing recovered runtime addresses.

## Observations that distinguish the next real gate

| Native checkpoint | Runtime VA | What it establishes |
|---|---|---|
| Syscon packet padding VMOV pre/post | `4F9BEA` / `4F9BEE` | Corrected instruction really continues in the native worker; RAM packet object is R4. |
| Syscon transmit/start pre/post | `4F803C`, `4F811E` / `4F8120` | Real packet R0/R5, SPI0 configuration, RX/TX counts and transfer snapshot. |
| Syscon GPIO3 set pre/post | `4F812A` / `4F812E` | Guest submits the notification after SPI0 start, rather than a host-injected request. |
| GPIO parent / response subhandler | `5AA298`, `4F8160` | Actual IRQ248 and pin4 subinterrupt dispatch execute. Registration uses native `(248,4)`, not a guessed SPI IRQ. |
| GPIO4 ACK pre/post / SPI0 stop | `4F819A` / `4F819E`, `4F8350` / `4F8352` | Real native acknowledgement and packet completion lifecycle. |
| Syscon native result mask | `4FA728`, `4FA73A` | Packet flags decide its result: mask `B00000` excludes the standalone SPI status200 flag. |
| OLED SPI2 start before first TX | `4F477A` / `4F477E` | Native control30001/start1 precedes streaming words, distinct from SPI0 request framing. |
| OLED TX drain / passed | `4F41C6` / `4F41D4` | The real driver tests SPI2 +2C; nonzero pending count can prevent readiness. |
| OLED ready1/ready2 store | `4F47CE` / `4F47D0`, `4F47E0` / `4F47E2` | Readiness VA4F7004 is produced by guest code. |
| OLED readiness / Display continuation | `4F48D2`, `4F48EC`, `5B4F1E` | The actual Display worker crosses its observed dependency. |
| Guest logo/gzip/SetFrameBuf | `5B7048`, `5B70AC`, `5B70DE` | Later producer and presentation calls are reached, without injecting offline image bytes. |

Every breakpoint is one-shot to bound polling captures. Two extra one-million-slice windows permit the observed secure EMC initialization's native 200ms delay. A missing breakpoint establishes only that it was not reached within the capture; runtime core PCs, work contexts, queues and device state determine the wait.

SPI reads deliberately start at physical +08 and omit RX +00, so diagnostics cannot drain replies. RX/TX counts at +28/+2C are read live: current generic named register peeks expose stored reset images and would misreport those queues. GPIO is inspected with nondestructive `RegisterFile.peek` via `devget`; physical GPIO+04 reads currently consume a legacy debugger handshake and are excluded. A `no such register` response is retained as an unrepresented backing offset, rather than converted into a hardware zero. GIC diagnostics omit IAR and all acknowledgements. No status, GPIO input, IRQ, register value, packet result, worker wake or framebuffer is fabricated.

Static grounding remains `goal-native-spi-gpio-evidence.md`, `goal-native-spi-syscon-thumb.txt`, and `goal-native-spi-oled-thumb.txt`. Next model changes must follow the actual reached state; this plan does not authorize a slave response or controller extension.
