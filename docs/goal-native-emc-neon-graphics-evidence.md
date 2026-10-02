# Native Syscon/OLED after EMC and NEON: integrated 572

Frozen private capture from the integrated all-target build with full suite 572/0. Command: `python3 build/goal-arm-native-syscon-oled-postneon.py --label emc-neon`. The script removes every `ZLB_*` override, makes a fresh APFS clone of `goal-emmc.img`, validates real Syscon/Lowio/OLED/Display headers, and performs no source, firmware or guest-register writes. It exits cleanly after 12 real milestones and two additional one-million-slice no-hit windows. The original image is untouched by this probe; its private clone remains dirty=no, 10,753 reads, zero writes.

## Proven progress

The native Syscon subinterrupt registration `(IRQ248, pin4)` and its enable call return0. Genuine packet construction crosses the corrected `VMOV.I8 D16,#0xFF` at VA4F9BEA→4F9BEE on ARM3. No ARM undefined-instruction stop occurs in this capture. OLED work and Display work execute in distinct native thread contexts; Display reaches its OLED readiness poll, while ready VA4F7004 remains zero.

EMC hardware diagnostics record command000E0000/control1, commands1/completed1, idle status0 and IRQ34 low. This establishes completion of the modeled first controller command; it does not establish return of the secure driver callback. ARM2 later remains in its native Secure WFE at VA000CD024 with R0=00548728, R4=00548700, LR003BF1D3 and TPIDRPRW=0, matching the parent’s ordinary capture.

## The reached frontier precedes SPI submission

No native SPI0 transmit/start, GPIO parent/subinterrupt response handler, OLED SPI2 start/TX-drain, readiness return, logo producer, gzip, SetFrameBuf or vblank breakpoint is reached. SPI0 remains at its 31 earlier boot-chain transfers, 364 bytes out/578 in, RX pending0. SPI2 has control30001 but zero transfers/queued TX bytes, so a streaming/drain failure has not yet been demonstrated.

Both no-hit snapshots place Syscon in its native elapsed-time call/loop, with LR4F9DBD and threshold R4=00000FA0. First snapshot: ARM3 PC4FD9A8, R0/R1/R2=0. Final snapshot: ARM3 PC4B7D6E, R0=0, R1=00E6C000, R2=00435000, R3=2802D000, TPIDRPRW=0006BE28. This is immediately after the provider counter read identified independently by the CPU agent at4B7D6C. The parent/CPU audit establishes that import4FD9A8 is genuinely bound to ThreadMgr4B7D5D and reads physicalE20B6000, currently the unticked Kermit.UnkE20B6 backing block. This independent native hardware finding, rather than a fabricated packet/IRQ success, explains the next counter implementation task.

Display’s worker is also executing that time provider from OLED readiness: first snapshot ARM1 PC4B7D5C/R0=0/LR4F48E1; final PC4B7D6A/R1=00E6C000/R3=2802D000 with native context0044B228. ARM0 is normal idle47969C. Total executed instructions grow to1,436,036,622 while `info` still reports emulated t≈0.276s. Native DSI0 control/mask/status and IFTU0 bus control/mode remain zero; no guest framebuffer/logo is produced.

## Captured GPIO backing is not a hardware completion claim

Nondestructive peeks record legacy GPIO dataA90009, input backing10, +10=FF0000, packed pin modes+14=300 and gate0 mask+1C=0. Backing +38..48 retain10 from native acknowledgement writes; the present RegisterFile implementation does not implement native W1C gate semantics. Several offsets, including output latch+34, have no defined/written backing value. These values identify future model gaps but do not prove an asserted native ready IRQ: no response handler is reached and the SPI request has not been submitted.

SPI reads omit RX FIFO+00; GPIO input+04 is inspected only by nondestructive peek because current Bus.read consumes a legacy debugger handshake. GIC IAR/ACK accesses are omitted. Thus the probe does not consume a pending response or manufacture progress while observing counts/masks/status.

## Actual milestones

| Native checkpoint | Core | VA PC | R0 |
|---|---|---|---|
| Syscon module_start | arm0 | `004F9DF4` | `00000004` |
| Syscon GPIO248 sub4 registration call | arm0 | `004F97C8` | `000000F8` |
| Syscon registration result | arm0 | `004F97CC` | `00000000` |
| Syscon GPIO248 sub4 enable call | arm0 | `004F97D0` | `000000F8` |
| Syscon sub4 enable result | arm0 | `004F97D4` | `00000000` |
| Syscon initialization returned | arm0 | `004F9DFA` | `00000000` |
| Syscon packet VMOV before | arm3 | `004F9BEA` | `FFFFFFFD` |
| Syscon packet VMOV after | arm3 | `004F9BEE` | `FFFFFFFD` |
| OLED module_start | arm0 | `004F4834` | `00000004` |
| OLED work callback | arm1 | `004F4780` | `00000000` |
| Display init work | arm1 | `005B4F14` | `00000000` |
| OLED readiness poll | arm1 | `004F48D2` | `00000000` |

## Artifacts

- `goal-arm-native-syscon-oled-emc-neon.log`, `goal-arm-native-syscon-oled-emc-neon.json`, `goal-arm-native-syscon-oled-emc-neon-clone.img`.
- Prepared script `goal-arm-native-syscon-oled-postneon.py`; interpretation/guard plan `goal-native-syscon-oled-postneon-plan.md`.
- Frozen previous frontier `goal-native-graphics-560-evidence.md`.
- Static genuine register/driver contract `goal-native-spi-gpio-evidence.md` and Syscon/OLED byte listings.

SHA256 `goal-arm-native-syscon-oled-emc-neon.log`: `532bd1200b402a9ce39d216474dd0464346c4ba693348563fd6f9b8c95bfa62c`.

SHA256 `goal-arm-native-syscon-oled-emc-neon.json`: `ff412055f3d89adc99a6b34770db98d187254942dc28bfef7a1c43115a9a7e21`.

SHA256 `goal-arm-native-syscon-oled-postneon.py`: `14b55d147d1499c3c6c5ed839c107490d03252b13638bd0c113adb2ca138f878`.

