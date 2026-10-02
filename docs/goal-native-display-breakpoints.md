# Minimal native display/logo capture plan (firmware 1.04)

Prepared read-only from the already recovered actual ELF bytes and reports. No new guest run, source change, shared build, guessed flip/status or logo injection. This is a future runtime capture plan, not evidence that these routines have executed in ordinary boot.

## Resolve relocated segments first

Let `D`, `L`, `O` be the observed runtime first-PT_LOAD VAs of Display, Lowio and Oled, respectively. Every code offset below is relative to linked `0x81000000`: breakpoint PC = observed module code base + offset, with Thumb bit cleared. Do not set literal `0x8100....` breakpoints without verifying the actual runtime placement.

Their ELF `e_entry` values (`Display 0x3EC0`, Lowio `0x76D0`, Oled `0x1570`) point to module-info data, not executable start functions. The actual noname export NID `0x935CD196` identifies callbacks at Display `D+0xCF8`, Lowio `L+0xB0`, Oled `O+0x834` (export pointers have Thumb bit1). Capture the genuine module loader's segment VA/PA and callback bindings; avoid assuming an unresolved import or direct ELF entry.

Each also has a separate zero-file-size BSS PT_LOAD: Display linked `0x81007000`, memsize0x200; Lowio linked `0x8100B000`, memsize0xDBC; Oled linked `0x81003000`, memsize0x13C. Let `Dd`, `Ld`, `Od` be their actual runtime data-segment VAs. Code and data need not share a guessed fixed relocation delta. Handler R1 and relocated MOVW/MOVT pairs provide direct live state pointers if loader records are inconvenient.

## Native prerequisites and hidden return values

Bootconfig's second fourteen-driver table is, in order, `stdio, lowio, syscon, oled, display, sm_comm, ss_mgr, sdif, msif, gcauthmgr, sdstor, rtc, exfatfs, sysstatemgr`. Core dependencies already precede it: Sysmem, IntrMgr, ThreadMgr, DmacMgr, ProcessMgr, ModuleMgr and related providers. Display imports Lowio's DSI/IFTU/Pervasive and Oled APIs plus ThreadMgr work/event/fast-mutex functions, Sysmem physical memblocks/address conversion, IntrMgr subinterrupt/SGI services, and Utils gzip decompression. Bound imports and successful native starts should be proved rather than supplied.

Display module start does **not directly create a thread**. At `D+0xD1A` it calls its ThreadMgr import `D+0x3B9C` (NID E50E1185, named EnqueueWorkQueue), with:

- R0=`0x10023`, R1=actual relocated `SceDisplayInit` string (linked+0x5920), R2=`D+0xF15`, R3=0.
- Work callback code begins `D+0xF14`.
- At `D+0xD1E`, the routine unconditionally overwrites the queue result with R0=0. Capture R0 **before** executing that MOVS, and separately prove callback entry; module-start return0 alone proves neither.

Oled module start similarly queues its initialization callback `O+0x780` at `O+0x86A`. Display callback immediately calls Oled WaitReady. If it parks there, the actual wait at `O+0x8D2..0x8EA` polls `[Od+4]` and obtains native system time; capture the real producer before considering display MMIO. No readiness value should be manufactured.

Lowio module callback calls genuine DSI init `L+0x406C` and IFTU init `L+0x62C8`; the map/setup bodies are `L+0x3EA4` and `L+0x5D3C`. These register IRQ213 and IRQ204/205 with the native IntrMgr.

## Small sequential breakpoint set

Set only the next needed points after obtaining actual module bases; remove completed points using `bpc <PC>` to avoid repeated stops. `bp arm <PC>` covers all four cores; identify the stopped core from the CLI and inspect that core. Do not single-step one core through waits that need the native scheduler/other cores.

| Stage | Relative native PC | Capture before continuing |
| --- | --- | --- |
| Module start and work request | Display `+CF8`, `+D1A`, `+D1E` | Init result, four queue arguments and queue result before overwritten0 |
| Real work begins | Display `+F14`, `+F1E` | Callback entry and Oled WaitReady return; `+F1E` proves it got past readiness |
| Setup return / skip predicates | Display `+FDA`, `+FDE`, `+FEC`, `+FF4`, `+FFC` | Update-mode, external-mode, unknown Sysroot NID7918D44E, USB-enum-wakeup predicate returns; any nonzero can skip logo |
| Logo producer | Display `+3048` | Native caller and allocation options are about to be built |
| Physical memblock result | Display `+3088`, `+309C` | Alloc UID/error; actual GetMemBlockBase result and guest VA at `[SP+4C]`. Options request PA1C000000, type6020D006, size200000 |
| Actual gzip call/result | Display `+30AC`, `+30B0` | R0 destination VA, R1 capacity1FE000, R2 relocated asset VA (`D+4A60`), R3=0; returned byte count/error; translate VA then dump **actual PA** |
| Actual framebuffer call/result | Display `+30DE`, `+30E2` | R0=head0, R1=plane0, R2=`SP+34` descriptor, R3=sync1; return/error before branch |
| First fade wait and release | Display `+30F2`, `+30F6` | First vblank wait call and actual return; if parked inspect DSI delivery rather than modify wait field |
| Native vblank chain | Lowio `+3AE0`, Display `+2940`, `+295C` | DSI top handler status/mask and W1C; Display R1 head state, counter at+0C and event UID at+10; new counter after callback |
| Fade finished | Display `+3130`, `+3134`, `+1004` | Merge-mode80 call, producer result, init result; successful producer sets `[Dd+178]=1` at+1012 |

Ordinary head0 setup `D+0x4C4` provides a narrower diagnostic if setup fails: it calls DSI SetVic, clocks/reset, EnableHead, both IFTU output-format/Enable functions, then DSI StartDisplay. IFTU Enable returns are at `D+556` and `D+598`; DSI StartDisplay return is `D+5D6`. Native head0 ordinary path supplies control argument0; Lowio produces control1/progressive0/timingC4E/252. Capture actual values if it takes another branch.

## Logo byte validation and frontend evidence

The API descriptor at `D+30DE` is `{size18, guestVA, pixel_pitch3C0, format0, width3C0, height220}`. Lowio must translate that VA to actual PA; requested1C000000 is not proof of allocation. At successful gzip return, translate destination with the active core's `vpa`, then physical `save <PA> 0x1FE000 <unique.bin>`. Compare the captured guest bytes read-only with the offline reference hash `80c43bdc43d6fcc7f1419960726e6dc3e6be906ae0faac2cdf060204717a6979`; do not copy reference bytes into guest RAM. A decode match proves the guest producer, not bank selection or presentation.

IFTU bank must then contain PA, format10, width3C0, height220, padding0, blank0; stride is width*4+padding=`0xF00`, not bank+4. Frontend upload intentionally treats these RGB bytes as opaque; asset byte3=0 does not make the panel transparent. Verify native enabled scanout from `devices`/named IFTU peeks and a fresh native SDL Display-panel capture, with synthetic E210 framebuffer disabled. An offline PNG, fixture color bars, or producer-only dump cannot count as an observed guest logo.

## Minimal physical MMIO observations

Low-overhead initial write log can use `ZLB_WTRAP=E5020000-E5050FFF`; it observes actual PA writes without changing registers. Once initialization is close, more focused CLI write watchpoints may use the following actual addresses. They force ARM slice size1, so do not keep many active throughout the entire boot.

| Purpose | Physical address(es) | Proven expected ordinary store / read |
| --- | --- | --- |
| Shared IFTU enable/mode/select | E5022000, E5022004, E5022010, E5022018 | Bus1; A mode2 then bothA; selects0. Shared mode1/3 bits are not named hardware enables |
| A/B observed run/cached control | E5020050/E5021050, E5020180/E5021180 | Lowio writes run1, cached flag1; +180 meaning remains unresolved |
| Native bank PA | E5020200/E5020300, E5021200/E5021300 | Actual guest PA after SetInput; capture full descriptor around+240/+340 |
| Current-bank state | E5020004, E5021004 | Guest reads bit1; inspect without assuming autonomous toggle |
| IFTU event ACK | E5020040, E5021040 | Native handler writes **zero**, then cached flag at+180. No nonzero status bit established |
| Fade/merge | E502108C, E50210A0, E5022020, E5022004 | B alpha0,2,...FE; localA0=0; shared+20=4 and mode bit0; final+20=0/bit0clear |
| DSI startup | E5050000, E5050004, E5050008, E505000C | 1,0,C4E,252 |
| DSI native vblank IRQ | E5050050, E5050054 | Latched bit1, mask2; handler writes same status back W1C |

Useful non-acknowledging peeks: `devget Kermit.IFTU0.A.BANK0.ADDRESS`, `.A.BANK1.ADDRESS`, `.A.BANK_SELECT`, `.A.REG_004`, `.BUS_CONTROL`, `.BUS_MODE`, and `devget Kermit.DSI0.CONTROL`, `.STATUS`, `.IRQ_MASK`. Never read raw GIC IAR merely to inspect delivery; that acknowledges an interrupt. Capture per-core GIC diagnostics and native IAR/EOI tokens from actual handlers.

GIC distributor word6 holds IRQ204/205/213: groupPA1A001098, enablePA1A001118, pendingPA1A001218, activePA1A001318; bits12,13,21 respectively. Their priority/target bytes are 1A0014CC/4CD/4D5 and 1A0018CC/8CD/8D5. Do not force native enables, targets or security groups. A vblank callback may trigger SGI8 at Display `+2970` only when its head mask test passes; native event delivery, not SGI8 alone, releases the logo wait.

## Unresolved hardware behavior to keep separate

Current DSI0 supports native mode1/VIC0 timing and level IRQ213, but does not implement packet/PHY readiness, +414 shutdown completion, +48 idle or +4C scanline semantics. A later live need must be diagnosed separately; native startup itself has no DSI readiness poll.

Current IFTU0 exports only the explicitly selected valid enabled guest-RAM bank. It does not generate IFTU IRQ204/205, autonomously turn banks, scale or blend. Native deferred `SetInput` (`L+65E0`, sync R3!=0) writes the opposite bank and software pending old index (`L+6754..676E`); no select/+180/commit write is emitted then. Its IRQ handler (`L+599C`) zero-ACKs+40, writes cached+180 and replays the new descriptor to the old bank. If startup selected bank0 and the logo is prepared only in bank1, static scanout can remain blank even with perfectly decoded guest pixels and real DSI vblanks. Record both banks, current-bit1, shared selects/mode and actual handler execution; do not manufacture a flip, status or callback to satisfy the test.

The sources prove the prepare/replay protocol and explicit selects, not continuous versus rearmed hardware turnover, the exact +180 function, an IFTU completion status bit, or the blending equation. Fade completion proves guest waits progress; the current raw-plane frontend does not reproduce physical OLED blending.

Source references: `goal-firmware104-display-contract.md`, `goal-firmware104-dsi0-timing-contract.md`, `goal-firmware104-iftu-deferred-turnover.md`, and their actual-byte listings. Only callsite/workqueue/Oled-ready sequencing and future breakpoint selection were added here.
