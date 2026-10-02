# Genuine 1.04 post-timer graphics capture (579-test binary)

The timer-integrated native run reaches both the Syscon SPI0 submission and the OLED SPI2 start-before-TX path. It does not yet reach a guest framebuffer: OLED remains in its A1 command helper, Display waits for OLED readiness, and DSI0 / IFTU0 controls remain zero. A live stack-pointer disturbance must be resolved before treating the final OLED TX length as a hardware requirement.

## Reproducible evidence

`python3 build/goal-arm-native-syscon-oled-postneon.py --label timers --slices300000 --empty-windows1`

The script uses a fresh private eMMC clone and clears inherited `ZLB_*` environment overrides. It adds debugger breakpoints and read-only inspection, with no register, RAM, guest return or forced wake changes. Artifacts are `goal-arm-native-syscon-oled-timers.log`, `.json`, and `-clone.img`. All captures use the integrated binary whose full suite passed 579 tests. The JSON contains CPU registers for the 21 breakpoint milestones; device snapshots are in the log.

The final no-hit window runs 300,000 slices. The clone remains clean, with 10,753 sector reads and zero writes. No ARM core has an undefined-instruction halt. The final PCs are ARM0 `0x004F4324` (OLED TX drain), ARM1 `0x004F48DC` (OLED readiness wait), and ARM2 / ARM3 `0x0047969C` (ThreadMgr idle).

## Syscon submission and GPIO observations

The genuine Syscon GPIO248/sub4 registration at `0x004F97C8` returns zero at `0x004F97CC`; its enable at `0x004F97D0` returns zero at `0x004F97D4`. The native worker enters transmit at `0x004F803C` on ARM3 with packet VA `0x000A6D48` (PA `0x41035D48`). The packet begins `01 00 01 FD` at packet offset `+0x10`, with native `FF` padding; packet flags at `+0x0C` are `0x11`.

SPI0 start at `0x004F811E` advances its device counters from 31 to 32 transfers, 578 to 588 RX bytes, and zero to ten pending RX bytes. Its TX total is 368 bytes at both sides of this start. GPIO3 is set by the following native call at `0x004F812A`, returning zero at `0x004F812E`. The Syscon GPIO-sub4 callback breakpoint is never reached during this capture. SPI0 still has ten RX bytes pending at the final snapshot.

These are nondestructive register views of the **old GPIO shim**, not a hardware interpretation:

| Milestone | +00 | +04 | +08 | +0C | +10 | +14 | +1C | +38 / +3C / +40 / +44 / +48 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Registration / enable, points 01–04 | A90008 | 10 | A90000 | 8 | FF0000 | 300 | 0 | 10 each |
| SPI0 start before / after and GPIO3-set entry, points 13–15 | A90001 | 10 | 1 | 8 | FF0000 | 300 | 0 | 10 each |
| GPIO3-set return and later, points 16–20 / final | A90009 | 10 | 8 | 8 | FF0000 | 300 | 0 | 10 each |

Offsets `+18`, `+20`, `+24`, `+28`, `+2C`, and `+34` return `no such register` through this diagnostic interface. They must not be reported as proven zero. The old shim folds output writes into `+00`; its observed `+00` transition does not establish a real direction-register transition.

Final GIC state for IRQ248: distributor group word at `0x1A00109C` is zero, enable word at `0x1A00111C` is `FF000000`, pending word at `0x1A00121C` is zero, priority byte at `0x1A0014F8` is `50`, target byte at `0x1A0018F8` is `0F`, and configuration word at `0x1A001C3C` is `55555555`. All four CPU interfaces have `ICCICR=B`, `ICCPMR=FF`, and deasserted IRQ / FIQ outputs. These observations locate the missing GPIO event delivery without asserting a board-ready input pulse.

## OLED reached path and live divergence

The OLED worker runs on ARM0 in this capture. At its A1 call `0x004F47A6`, and helper entry `0x004F41EC`, arguments are command `A1`, output VA `0x000AADF0`, length five, with entry SP `0x000AADF0`. The prologue reserves 32 bytes. SPI2 start before / after at `0x004F477A` / `0x004F477E` has SP `0x000AADD0`, and stores control `00030001` followed by start `1`, with an empty TX FIFO. This proves the start-before-TX controller path is genuinely exercised.

After the final window, ARM0 is at the native drain loop `0x004F431E..0x004F4324`. SPI2 has one transfer, 74 TX bytes, zero RX bytes; `+2C=4A`, `+28=0`, control `30001`, interrupt control `3`, and DMA control `F`. Native BSS is coherent at VA `0x004F7000` (PA `0x40753000`): open count `1`, ready `0`, partial-bit count `0`, partial word `0`, mapped SPI VA `280DF000`, and event UID `10A95`.

The final helper SP is **zero**, although SP was correct immediately after SPI2 start. The ARM0 MMU-fault counter increases from eight to nine; the log records an intervening model boot-window mapping supplied for a write to VA `0x24`. This capture alone does not identify the instruction or context switch that lost SP. **LR=7 is expected helper scratch state**, not evidence of corruption: the unmodified packer uses LR to count residual bits. The extra TX words therefore remain unexplained, and no SPI2 production change is justified by the live length alone.

## Unchanged helper control experiment

`goal-oled-a1-unchanged-probe.cpp`, executable `build/research/zeliboba-oled-a1-unchanged-probe`, and `goal-oled-a1-unchanged-probe.log` form an isolated control experiment. The harness copies the original `oled.elf` first PT_LOAD (`file+0xA0`, size `0x23C0`) to its genuine linked address `0x81000000`, supplies ordinary BSS / stack / output RAM and the actual existing SPI2 controller, and executes unchanged helper bytes from `0x810001EC`. The original unbound clock-import stubs remain in place; their error returns are ignored by the helper just as its code specifies. No scheduler or external interrupt runs in this fixture.

The same 579 CPU produces exactly nine low-16-bit TX words:

`030A 0804 2010 0040 0000 0000 0000 0000 0000`

It reaches `0x8100031E` after 378 instructions with 18 TX bytes, SP `0x81004FD0` (entry `0x81004FF0` minus 32), LR `7`, and partial-bit count zero. The prologue is the only SP change. This proves the native packer and current CPU decoder can execute the expected A1/len5 emission without the live scheduling / exception environment. The experiment stops at the real current controller's drain wait; it supplies no RX response and does not claim that the entire OLED helper returns.

## Frontend and pending work

DSI0 control / status / mask and IFTU0 bus control / mode remain zero, with no supported enabled guest-RAM scanout. Display is genuinely waiting in OLED readiness; a successful module start does not establish a framebuffer or logo.

The read-only transport proposal in `goal-native-oled-spi2-streaming-proposal.md` remains pending. It bounds SPI2 `CTL=30001` streaming and explicitly documents a chosen undriven pulled-high input that would sample `FFFF` per actually consumed 16-bit word. This is an unverified board-wire assumption, not a valid panel identity; the native all-FF A1 path should reject the panel as ready `2` and retain `803F0A03` API errors. No response, readiness, script return, framebuffer or IFTU turn-over is injected in this capture or control experiment. The CPU owner is auditing the live SP disturbance before transport implementation.
