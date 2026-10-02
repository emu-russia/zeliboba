# Native cold 1.04 logo producer and deferred IFTU frontier (611 tests)

Fresh private APFS image clone, every inherited ZLB variable removed, ordinary per-core execution budget, no watches, guest writes, instruction/API patches or injected pixels. The main probe records 83 native checkpoints and one final 300,000-slice no-hit window; its artifacts are `goal-arm-native-syscon-oled-cold-stack.log/.json`. The supplied boot chain still uses documented model stand-ins, including native-kernel staging and a chosen undriven-high SPI2 input; this is not a claim of complete console hardware fidelity.

## Actual cold inputs and producer

Native module-list return CC02E is zero. Actual Syscon/Oled/Lowio/Display module-info names validate their previous text placements. Syscon GPIO248/sub4/response/checksum/transport executes without the earlier stack/CPU faults. OLED A1 still samples all-FF through the explicit undriven wire model, publishes rejected-panel ready2, and returns the original GetDDB error803F0A03; native Display accepts that state.

All four actual Display logo predicates now return zero:

| Native return PC | Predicate | R0 |
| --- | --- | --- |
| 5B4FDE | B0E1FC67 UpdateMode | 0 |
| 5B4FEC | 89D19090 ExternalBootMode | 0 |
| 5B4FF4 | 7918D44E parameter+6C bit16 | 0 |
| 5B4FFC | 79C9AE10 UsbEnumWakeup | 0 |

The provider's real parameter VA47C0 maps PA403047C0, with bytes+30..33=`FF FF 00 FF`, word+6C=4, +C0=60,+C4=FF14. The observed chain is Sysmem globalCE328→sysroot4000→sysroot+3C=47C0. Sysroot+254 points at original context51184C00; context+2C points at source51184E98. The source's complete100 bytes are independently inspected in the same capture and retain the cold values.

Display calls its real producer5B7048. At5B70AC, actual guest allocator VA10000000 maps PA1C000000 and is passed to the native gzip routine; at5B70B0 it returns1FE000. Those 2,088,960 guest-produced bytes were exported only after that return:

- `goal-arm-native-syscon-oled-cold-stack-guest-framebuffer.rgba`
- `goal-arm-native-syscon-oled-cold-stack-guest-framebuffer.png` (RGB rendering of the exported four-byte pixels; alpha byte is zero throughout)
- SHA256 `80c43bdc43d6fcc7f1419960726e6dc3e6be906ae0faac2cdf060204717a6979`

The raw bytes are exactly equal to the independently decompressed supplied display.elf asset, not merely a visual resemblance. The PNG visibly contains the white PlayStation logo on black. Native SetFrameBufInternal call5B70DE returns0 at5B70E2, and its first native vblank wait returns0 at5B70F6. No extracted asset was written into guest memory.

## Actual deferred register transition

The first logo submission is {head0,buffer0,descriptorVA110AD6C,sync1}; descriptor size18/base10000000/pitch960/format0/960x544. The actual registers and native supplied code prove the deferred opposite-bank path.

| State | Before native SFB | After native SFB / first vblank |
| --- | --- | --- |
| planeA+4 current-bank bit1 | 0 | 0 in current static model |
| planeA+40 raw | 0 | 0 |
| planeA+50 / +58 / +180 | 1 /108 /1 | unchanged |
| shared+0 / +4 | 1 /A | 1/A immediately after SFB;1/B after merge call |
| shared A+10/B+18 selects | 0/0 | 0/0 |
| A software+1E0/+1E4/+1E8/+1EC | 0/1/0/1 | 0/1/80000000/1 |
| A bank0 | PA0,blank1 | PA0,blank1 |
| A bank1 | PA0,blank1 | PA1C000000,blank0,format10,960x544,padding0 |
| B banks | PA0,blank1 | unchanged |

Software planeA record is VA4F2298, its fields+1E0/+1E4/+1E8/+1EC are4F2478/7C/80/84. Hardware plane register+180 is distinct from the software descriptor cache+180.

A→B is only shared alpha-enable bit0: after SFB, Display30EE/runtime5B70EE calls SetMergeConf1548 with head0/setting4/alpha0. Lowio688C/runtime5AE88C stores shared+20=4 and ORs bit0 of shared+4; actual STR is linked810068F4/runtime5AE8F4. Both per-plane mode pairs remain binary01. Endfade setting80 later clears that same bit. This is not a bank-selection/flip/rearm store.

The native full-bank helper58DC first clears bank+4C at5940, then writes format/geometry/padding/addresses/scaling/crop; its last full-bank store is+D4 at5994/runtime5AD994. Nothing identifies+D4 as a hardware commit. Deferred SetInput writes no hardware+180 or shared select; it stores pendingoldindex|80000000 only in guest software. Current static IFTU therefore does not present the new bank, despite valid guest pixels.

## Narrow frame-publication model proposed for implementation by the hardware owner

The firmware and public manual-select code do not recover the automatic-mode name, +58 interrupt bits, +180 enable/rearm semantics, or a nonzero+40 status encoding. The bounded proposal is an explicit model choice for the reached ordinary subset, not a claimed hardware register specification:

1. Keep manual modepair00 and public shared selects. Support automatic modepair01 only with bus1, plane50=1, plane58=108 and a native write of180=1; mode11 and unknown setups remain unsupported.
2. Each actual180=1 write arms one frame publication. A real DSI0 frame boundary consumes that arm, changes the coherent active-bank index and plane+4 bit1 **before** asserting normal GIC204/205. DSI is the only frame clock. No guest software-pending read, framebuffer-PA heuristic, finalD4 commit, API-return override or extra timer is used.
3. Track completion internally, leaving raw+40 at its recovered value0. Native write0 ACK clears that level event; the real Lowio handler writes cached180=1 to rearm, then replays the descriptor into the old bank and clears its own pending word. Do not invent a status bit to describe this internal latch.
4. Keep active-bank frontend selection consistent with native+4, while preserving stored manual-select registers. Validate actual guest RAM/format/geometry for export independently of frame/IRQ delivery; a blank/invalid input is not successful picture data and must never inject pixels.
5. Reset/disable removes the model's pending event/arm and drives the IRQ low. Masked GIC delivery must not undo the frame change or require a forced enable. Unsupported interlace/alpha composition/scaling semantics remain explicit.

This corresponds to continuous native frame cadence only because the **guest handler** rearms180=1. An empty/blank first frame may still raise a frame completion; descriptor validity is frontend memory safety rather than a guessed completion gate. Native handler599C reads+40 twice without branching on either value, zero-ACKs, writes cached180, then replays pending; no nonzero status pattern is established by those bytes. Primary [manual IFTU code](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/iftu.c) confirms explicit selectors/blanking; its [header](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/include/iftu.h) confirms format10. These sources do not prove automatic turnover/rearm timing.

Meaningful tests should cover one boundary per actualarm, coherent status/frontendbank selection before IRQ, normal ACK/rearm and native old-bank replay, both starting indices and both planes, unsupported modes, blank/invalid memory guards, disable/reset/masking, and preserved manual selection. The unchanged native helper/handler fixture can verify replay/clear/rearm behavior using ordinary test-RAM patterns. This report author edits no production source.

## Final bounded state

DSI0 has22 modeled frames; its real IRQ213/parent/Display callback/counter and first logo vblankwait execute. IFTU IRQ204/205 and native old-bank replay do not occur in the current static model; the frontend reports no supported enabled scanout. The fade/producer-final return is therefore not yet observed. CMeP sleeps80048A; ARM0/1/2 idle47969C, ARM3 remains a live Syscon timed worker4F9DC0, no undefined halt. Clone remains clean10,753 cardreads/0writes. SDL presentation is a separate pending validation; the raw guest-generated logo is now proven.
