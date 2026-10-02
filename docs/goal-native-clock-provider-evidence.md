# Native 1.04 time providers and the 572 delay frontier

Read-only investigation. No source edits, forced SMC, fabricated completion or shared build. The current integrated suite is 572/0. One fresh env-clean APFS clone ran the genuine boot through four exact breakpoints; its capture is `goal-native-clock-audit-capture.log` and script `goal-native-clock-audit-capture.py`. The original eMMC remained untouched; the private clone has 10,753 reads, zero writes and is clean.

## Reached shared clock: LT5, not the Cortex-A9 GlobalTimer

The genuine 1.04 ELF import/export tables establish these bindings:

| Consumer | Import library / NID | Native stub / provider |
|---|---|---|
| SceSyscon | SceThreadmgrForDriver `E2C40624`, `47F6DE49` | `004FD9A8` -> Thumb `004B7D5D` |
| SceOled | same library and NID | `004F54A0` -> Thumb `004B7D5D` |
| SceSyscon / SceOled delay | same library, `4B675D05` | `004FD9B8` / `004F54B0` -> Thumb `004A2FC9` |

The current VitaSDK primary NID database names `47F6DE49` **ksceKernelGetSystemTimeLow** and `4B675D05` **ksceKernelDelayThread**. These names agree with the older 1.04 tables; the database itself is for 3.60, so the local 1.04 address binding is separately established from its ELF tables and live patched stubs. The primary kernel header documents DelayThread's argument in microseconds.

At genuine linked `81017D5C` (runtime `004B7D5C`), ThreadMgr loads its state pointer, obtains the mapped timer pointer at state `+2280+54`, reads a word at timer `+0`, and returns that word unchanged:

```
004B7D5C  movw r2,#E000
004B7D60  movt r2,#8102         ; relocated global is 00435000
004B7D64  ldr r1,[r2]
004B7D66  add.w r0,r1,#2280
004B7D6A  ldr r3,[r0,#54]
004B7D6C  ldr r0,[r3]
004B7D6E  bx lr
```

The actual mapped timer VA is `2802D000`, physically **E20B6000**. The completed parent 1M-slice run and graphics agent's independent two-1M-window run both repeatedly read `E20B6000 = 0` at `004B7D6C`; graphics stops immediately after this load with R0=0. Adjacent GetSystemTimeWide at `004B7D34` reads high `+4`, low `+0`, high `+4`, retries on changed high, then returns the raw pair in R1:R0. Neither provider divides or rescales the counter.

ThreadMgr init `004B7DB2..004B7E4E` maps physical `E20B6000`, length `1000`, using native Sysmem imports. It registers interrupt **141/0x8D**, native handler Thumb `004B7A1D`. Its native deadline scheduler `004B762C..004B76C4` writes the 64-bit absolute deadline to timer `+8/+C`; no queued timers writes both words `FFFFFFFF`. IRQ handler at `004B7B10..18` writes **2** at timer `+18` before updating the next deadline. SDIF agent independently owns the detailed deadline/IRQ protocol recovery.

## Frequency and initial enable

The actual KBL instructions in `goal-kbl-live-bank.bin`, base `40020000`, establish:

```
4002158E  str r7,[r3,#1C]      ; r3=E20B6000, r7=2F345008 (stop)
40021590  str r5,[r3]          ; r5=0, current low
40021592  str r5,[r3,#4]       ; current high
40021594  str r1,[r3,#8]       ; r1=FFFFFFFF, compare low
40021596  str r1,[r3,#C]       ; compare high
40021598  str r2,[r3,#1C]      ; r2=2F34500D (start)
```

The fresh private capture stops at `4002159A`, after the start write. Its reads of the entire LT5 register range are all zero. This is a model defect, not an absent guest enable: `Kermit.UnkE20B6` is constructed as a plain RegisterBlock with **no defined registers**, so RegisterBlock::write ignores all those writes. RegisterBlock::read returns zero for each undefined register. It is also absent from `needs_tick()`.

The hardware-tested/reverse-engineering wiki identifies this aperture as SceLT5/System Time and explains its native configuration `2F34500D`: source selector 3 is **48 MHz**, high-byte divider is **47+1**, so `48,000,000/48 = 1,000,000 Hz`, i.e. **one count per microsecond**. This is corroborating board evidence rather than an official SoC manual. It agrees with the native raw GetSystemTimeLow/ Wide API and delay/deadline use. Cached full page is `build/research/zeliboba-hardware-timers.html`; plain text is `goal-native-clock-hardware-reference.txt`.

The existing Kermit device tick units already are microseconds (CPU 333 MHz -> peripheral 1 MHz conversion in `KermitBlock::tick`). The reached LT5 therefore needs one counter increment per device tick for this proven configuration. Changing A9 GlobalTimer frequency would not affect the reached provider. The UI's frozen `emulated time: 0.276` is CPU instruction-counter based and does not establish that the peripheral tick dispatcher stopped; root separately verified peripheral ticks continue during WFE.

Syscon currently uses elapsed low-counter subtraction in native loops `004F9D78..8C` and `004F9DB8..CC`; the reached latter loop requests 4000 microseconds. OLED's ready wait `004F48D2..E4` also calls GetSystemTimeLow, while its worker `004F479C` requests 20,000 microseconds through DelayThread before issuing its A1 command. No new CPU condition/load defect is indicated by these native loops.

## Independent Secure EMC delay: WT7 and IRQ135

`goal-native-e820-driver-tzs.elf` PT_LOAD0 file offset `A0`, linked `81000000`, native `0054A000`. Its genuine 1.04 import table maps linked stub `81001328` to library **SceIntrmgrForTZS**, library NID `EC3056FE`, function NID **C0908EA9**. No authoritative function name was found; describe it as the native Secure delay function rather than guessing DelayThread.

Fresh current-572 capture reaches `0054BB9C` after the genuine first EMC command has completed; R0 is `30D40` (200,000). Patched A32 stub bytes at `0054B328` are `E30FC149 / E340C03B / E12FFF1C`, which branch to Thumb **003BF149**. The targeted `dis` output for this A32 stub inherited the caller's Thumb state; decode its bytes as A32, not the misleading displayed Thumb mnemonics. Captured runtime Secure delay bytes match the saved genuine `build/research/zeliboba-tz-intrmgr-physical.bin` (base `003BE000`).

Secure delay `003BF148..214` obtains the current MPIDR core, clamps duration to <=1,000,000, builds per-core deadlines in `00548700`, and waits on a native exclusive lock at `00548720 + 4*core`. Timer pointer is global `00548730`. Initialization `003BF290..2E0` maps physical **E20BE000**, length `1000` (SceWT7); native IRQ **135/0x87**, handler Thumb **003BF001**. First delay writes compare timer `+0=duration`, current `+4=0`, configuration `+8=DD00000D`. Subsequent deadlines add to current `+4`. Handler writes 3 at `+14` to ACK and releases the due per-core lock via `CD038` (store zero, DSB, SEV).

The capture reaches `003BF1CE` with R0=`00548728` and R8/R9=`30D40`; it then naturally enters the same A32 lock seen in the completed parent run, PC `000CD024`, LR `003BF1D3`. That is the genuine interrupt-delivery wait, not an LDREX/STREX failure. **WT7 is wholly unmapped** in the current board, so its reads are FFFFFFFF and writes cannot arm a deadline or IRQ. The SDIF agent owns exact WT7 config, comparator and IRQ delivery recovery/proposal.

The corroborating hardware wiki describes WT7 config `DD00000D` as SysClock/(221+1), yielding 1 MHz at 222 MHz SysClock or ~0.856 MHz at 190 MHz. The actual platform source frequency must be established or explicitly documented as an approximation; do not silently treat every timer configuration as a 1 MHz clock. This uncertainty does not affect LT5's separately fixed 48 MHz / 48 configuration.

## Minimal coherent proposal (no implementation here)

Replace the placeholder LT5 and missing WT7 with proper MMIO timer devices driven by the same Kermit microsecond ticks. LT5 must preserve 64-bit programmable counter halves, stopped/start configuration, and the native 64-bit compare/deadline plus level IRQ141/ACK; WT7 must preserve its distinct 32-bit layout and deliver actual IRQ135 so the genuine Secure handler releases its own lock. Use the existing GIC level callbacks and real guest handlers. Do not patch GetSystemTimeLow, release locks from the host, or synthesize DelayThread returns. Unsupported source selections/configuration/auxiliary fields should remain stored and explicitly bounded rather than inventing behavior from register names. Tests should include the unchanged native KBL LT5 start sequence, rollover read protocol, disable/reset, programmed fractional rate as applicable, masked deadline latching/rearm/ACK, IRQ135/141 wiring and an unchanged native Secure lock handler return.

Primary NID/API references:
- https://raw.githubusercontent.com/vitasdk/vita-headers/master/db/360/SceKernelThreadMgr.yml
- https://raw.githubusercontent.com/vitasdk/vita-headers/master/include/psp2kern/kernel/threadmgr/thread.h

Corroborating board references (reverse-engineering wiki, not an official SoC manual):
- https://wiki.henkaku.xyz/vita/Hardware_Timers
- https://wiki.henkaku.xyz/vita/Physical_Memory
