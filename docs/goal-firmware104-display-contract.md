# Firmware 1.04 native boot-logo/display contract

Read-only recovery, 2026-10-01. No emulator source changed, no shared build run,
and no logo injected into the guest. The extracted image below is an offline
check of a real firmware asset; guest presentation has not been observed.

## Sources and address convention

Primary inputs are the supplied decoded `os0/kd/display.elf` and `lowio.elf`
under `../Vita_104_Firmware/Out/fs_dec/os0/kd`.

| Input | SHA256 |
|---|---|
| display.elf | `86bbe8a1944f7657de70253c9a96c71b56bc442f70dc49c7dfe2390b6141ed91` |
| lowio.elf | `8d6ff20781db00725bb7feb313f80f892a58ac04e947df4dbdaebaee37be881b` |

All `0x8100....` addresses below are **linked addresses in the named module**,
not asserted runtime addresses. Both first segments start at linked VA
`0x81000000` and file offset `0xA0`: file offset = VA − `0x81000000` + `0xA0`.
Physical MMIO addresses are unaffected by module relocation. NID matches name
the import/export functions; the supplied `db.yml` labels itself firmware 3.60,
so its names are labels only. All instruction, table, and layout evidence comes
from the actual 1.04 ELF files.

Readable instruction/byte evidence: [goal-display-contract-evidence.txt](goal-display-contract-evidence.txt).
Import/export identities and hashes: [goal-display-module-tables.txt](goal-display-module-tables.txt).
Extended DSI setup: [goal-display-dsi-enable-evidence.txt](goal-display-dsi-enable-evidence.txt).
These are selected disassembly ranges; embedded jump tables/literal data are not
function bodies even when a linear decoder prints them as instructions.

## The actual white PlayStation asset and its native producer

`display.elf` has a gzip stream at **linked VA `0x81004A60`, file offset
`0x4B00`**, consuming `0xE85` bytes and decompressing to **`0x1FE000` bytes =
960 × 544 × 4**. It is the centered white PlayStation symbol on black, including
gray antialiasing. All pixels have identical first three bytes; the fourth byte
is always zero. The monochrome asset therefore does not independently prove
R/G/B channel order. An opaque panel upload must not turn these pixels
transparent merely because their fourth byte is zero.

Artifacts: raw four-byte pixels (image omitted from source delivery),
offline decoded PNG (image omitted from source delivery).
Raw SHA256: `80c43bdc43d6fcc7f1419960726e6dc3e6be906ae0faac2cdf060204717a6979`.

Native routine **`display:0x81003048`**:

1. Allocates a `0x200000`-byte block, type `0x6020D006`, with options requesting
   **physical address `0x1C000000`**. Gets its guest VA using the real Sysmem API.
2. At `0x810030AC`, calls `ksceGzipDecompress` (import NID `0x367EE3DF`) with
   destination = allocated VA, capacity `0x1FE000`, source `0x81004A60`.
3. At `0x810030C0`, calls the imported D-cache/L2 writeback-invalidate-range
   routine (`0x364E68A4`) over the result.
4. At `0x810030DE`, calls `ksceDisplaySetFrameBufInternal` at `0x81001708` with
   head 0, plane 0, sync/update argument 1, and this actual structure:

   | API descriptor offset | Value |
   |---|---|
   | `+0x00` | size `0x18` |
   | `+0x04` | allocated framebuffer guest VA |
   | `+0x08` | pitch `0x3C0` **pixels** |
   | `+0x0C` | pixel-format selector **0** |
   | `+0x10` | width `0x3C0` = 960 |
   | `+0x14` | height `0x220` = 544 |

5. `0x810030E6..0x81003126` fades using real `SetMergeConf(0,4,alpha)` for
   alpha values 0,2,...,254, waiting for a vblank after each value. At
   `0x81003130` it calls `SetMergeConf(0,0x80,0)`.

The init thread `display:0x81000F14` configures and enables the display before
calling this routine at `0x81001000` in the ordinary boot path. Update/external
boot and Sysroot predicates can skip that call. This producer uses CPU gzip
decompression and the Display/Lowio presentation API; no SGX command execution
is required to create this particular bitmap.

## Actual MMIO windows and IRQ registration

Lowio's six-word, 24-byte descriptor records at **`0x81009498` / file+`0x9538`**
contain `{IRQ, name, register-name, register-PA, control-name, control-PA}`:

| Plane | Register PA, size | Shared control PA | IRQ |
|---|---|---|---|
| IFTU0 A | `0xE5020000`, `0x1000` | `0xE5022000` | 204 (`0xCC`) |
| IFTU0 B | `0xE5021000`, `0x1000` | `0xE5022000` | 205 (`0xCD`) |
| IFTU1 A | `0xE5030000`, `0x1000` | `0xE5032000` | 206 (`0xCE`) |
| IFTU1 B | `0xE5031000`, `0x1000` | `0xE5032000` | 207 (`0xCF`) |
| IFTU2 | `0xE5040000`, `0x1000` | none in this table | 255 (`0xFF`) |

`lowio:0x81005D3C` maps these physical windows using Sysmem type `0x20100806`
and registers the ordinary IFTU IRQ handler `0x8100599C`. The plane state stride
is `0x20C`; offset `+0` holds mapped plane-register VA, `+4` holds mapped shared
control VA. B reuses A's shared-control mapping. Fifth-plane behavior is separate
and is not needed to establish the head-0 logo contract.

DSI init **`lowio:0x81003EA4`** maps **DSI0 `0xE5050000`** and **DSI1
`0xE5060000`**, each `0x1000` bytes. IRQs are **213 (`0xD5`)** and **210
(`0xD2`)**; the top-level handler is `lowio:0x81003AE0`.

## Ordinary four-byte framebuffer descriptors

The actual logo's API format **0** becomes **IFTU input format `0x10`** at
`display:0x81001A6A`. The external/common validator independently makes the same
mapping at `0x81000A16`. These are different encodings from the preliminary SDK
API value 3 in `docs/GRAPHICS_REFERENCES.md`, and from the emulator's assumed
hardware format values 0/1. The native ordinary path validates the base against
`0xFF`, pitch against `0x3F`, and computes framebuffer capacity as
`pitch × height × 4` (`0x810019EA`, `0x81001A12`, later validation).

Independent public code corroborates **`0x10` = A8B8G8R8**, which corresponds
to byte order R,G,B,A on this little-endian guest. The white/gray asset alone
could not establish that channel order.
[vita-libbaremetal IFTU header](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/include/iftu.h)

Display helper **`0x81000130`** passes Lowio an input structure of `0x54` bytes:

| Lowio descriptor offset | Ordinary meaning |
|---|---|
| `+0x00` | hardware input format (`0x10` for the logo) |
| `+0x04` / `+0x08` | width / height |
| `+0x0C` | row padding **bytes**, `4 × (pixel_pitch − width)` |
| `+0x10` | zero in this path |
| `+0x14` | framebuffer **PA**, resolved from the API VA |
| `+0x18`, `+0x1C` | zero in this path; other address fields |
| `+0x20`, `+0x24`, `+0x28` | zero in this path |
| `+0x2C`, `+0x30` | scaling values; ordinary full-size default `0x10000` |
| `+0x34`, `+0x38` | viewport/placement values |
| `+0x3C`, `+0x40` | zero in ordinary path |
| `+0x44..+0x50` | additional flags/values, zero here |

Exact padding calculation is `SUBS r2,r2,r3; LSLS r6,r2,#2` at
`display:0x8100015E..0x81000162`. The descriptor starts at that routine's
`SP+8`; format/width/height/padding/PA are stored at `0x81000168..0x81000174`.

Each plane has two configuration banks, **plane base +`0x200` and +`0x300`**.
Helper **`lowio:0x810058DC`**, non-null-format path `0x81005922`, copies:

| Bank-relative register | Source descriptor field | Exact store VA |
|---|---|---|
| `+0x00` | `+0x14` framebuffer PA | `0x81005956` |
| `+0x04` | `+0x18`, zero for ordinary logo | `0x81005958` |
| `+0x08` | `+0x1C`, zero for ordinary logo | `0x8100595A` |
| `+0x20`, `+0x24`, `+0x28` | `+0x20`, `+0x24`, `+0x28` | `0x81005960..68` |
| `+0x40` | `+0x00` input format | `0x81005944` |
| `+0x44` / `+0x48` | width / height | `0x81005946` / `4C` |
| `+0x4C` | zero for an ordinary framebuffer; one for blank/null configuration | `0x81005940` / `58EA` |
| `+0x54` | `+0x0C` row-padding bytes | `0x81005950` |
| `+0x58` | `+0x10`, zero here | `0x81005954` |
| `+0x60..+0x6C` | `+0x44..+0x50` | `0x8100596A..76` |
| `+0xC0`, `+0xC4` | scale fields `+0x2C`, `+0x30` | `0x81005978..7C` |
| `+0xC8..+0xD4` | `+0x34..+0x40` | `0x81005988..94` |

**Correction preserved: bank+`0x04` is not stride.** In this ordinary format,
row stride is `width × 4 + bank[0x54]`, equivalently `API pixel_pitch × 4`.
For the logo it is `0xF00` = 3840 bytes and bank+`0x54` is zero.

Lowio output-format state is independently initialized by `0x8100651C`; Enable
copies it into each bank at `+0xA0` format, `+0xA4` output width, `+0xA8` output
height (`0x81006A80..88`, `0x81006AA4..AC`). Do not confuse bank+`0xA0` with
the distinct **plane-local** blend-control register +`0xA0`.

## Enables, deferred selection, and status ACK

Observed enables in **`lowio:0x8100690C`**:

| Destination | Actual write |
|---|---|
| plane+`0x58` | `0x108` at `0x8100696E` |
| plane+`0x50` | 1 at `0x81006972`; Disable writes 0 |
| plane+`0x180` | cached flag at `0x81006974`, ordinary flag 1 |
| both banks | cached input/output configuration at `0x81006A80..0x81006AB0` |
| shared-control+`0x04` | OR bit 1 for A or bit 3 for B; extra output-mode bits 2/4 |
| shared-control+`0x00` | 1 at `0x81006B0A` after shared+`0x04` is stored |

The public bare-metal implementation independently identifies shared+`0x00`
bit 0 as bus enable, shared+`0x04` bit 0 as alpha enable, shared+`0x10` (A) and
`+0x18` (B) as **explicit configuration selects 0/1**, and bank+`0x4C` as
inverted bank enable. Native Enable writes shared+`0x10` = 0 for A at
`0x81006B04`, shared+`0x18` = 0 for B at `0x81006B54`. Its shared+`0x04`
bits 1/3 must not be called proven plane-enable gates: bare-metal static
scanout sets only bit 0 there.
[vita-libbaremetal IFTU implementation](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/iftu.c)

Lowio unmasks the actual plane IRQ at `0x81006B1C`. Disable
`0x81006B6C..0x81006BDC` makes both banks blank (`+0x24C` / `+0x34C` = 1),
writes plane+`0x50` = 0 and plane+`0x40` = 0, and masks the plane IRQ.

**Directly observed bank-selection protocol:** `SetInputFrameBuffer` at
`lowio:0x810065E0` reads **plane+`0x04`, bit 1** (`0x81006680..82`) as the
current-bank index. Immediate argument `r3=0` updates address-related fields in
both banks. Deferred argument `r3!=0` writes the **opposite** bank using
`((current XOR 1) + 2) << 8`, then stores `current | 0x80000000` in its software
pending word (`0x81006754..0x8100676E`). Disabled planes only cache the structure.

On the real IFTU IRQ, **`0x8100599C`**:

1. Reads plane+`0x40`, **writes zero** to plane+`0x40`, then reads it again
   (`0x810059B8..BC`). This is the observed ACK pattern, unlike DSI's W1C ACK.
2. Writes the cached optional flag to plane+`0x180` (`0x810059BE`).
3. If software pending exists, clears pending and copies the cached new
   descriptor into the bank that was current before the deferred update.
4. Executes DSB and returns through the native IntrMgr handler.

**Unresolved:** no explicit write selecting plane+`0x04` bit 1 or a standalone
FLIP command was found outside Enable in these paths. Autonomous bank turnover
at the frame boundary is consistent with the opposite-bank/replay protocol,
but is an inference. Shared+`0x04` bits `[2:1]` / `[4:3]` appear to control
per-plane configuration mode: native ordinary head 0 uses pair value 1, while
bare-metal explicit selection uses pair value 0. The extra bits 2/4 follow a
native output-mode flag, set for particular head-1 VICs; head-0 output setup
`display:0x81000D24` sets that flag to zero (`0x81000D4E`, `0x81000DA2`).
Manual versus automatic versus interlaced interpretation is still **inferred**,
not an independently proved register specification.

The precise hardware role of plane+`0x180` and the status bit value
at plane+`0x40` remain unproven. Preserve the actual stores and IRQ order; do not
invent a synthetic commit register from this evidence.

## Logo fade and DSI vblank delivery

`display:0x81001548` translates head 0 to **B plane index 1** for merge/fade
control. The native fade setting 4 writes **shared-control+`0x20` = 4** and sets
shared-control+`0x04` bit 0 in `lowio:0x8100688C`. Its alpha calls
`lowio:0x810067FC`: **B plane-local+`0x8C` = alpha, +`0xA0` = 0** for values
below `0x100`; alpha `0x100` instead sets local+`0xA0` = 1. Final setting `0x80`
clears shared+`0x04` bit 0 and shared+`0x20` to zero. These are observed fade
writes; the full hardware blending equation is not recovered.

DSI StartDisplay **`lowio:0x8100517C`** programs timing registers and performs:

| DSI-local offset | Observed action |
|---|---|
| `+0x50` | read status, write the same word to ACK (`0x810053D8`, `53E0`) |
| `+0x54` | cached enabled-subinterrupt mask (`0x810053E2`) |
| `+0x838` | head 0 writes 0 (`0x810053F0`); head 1 writes 1 |
| `+0x508` | `0xFFFFFFFF` (`0x810053F4`) |
| `+0x00` | head 0 ordinary mode 0 writes **1** (`0x81005404`), from table `0x81009230[0]` |

Top IRQ handler **`lowio:0x81003AE0`** reads DSI+`0x50`, writes the **same
value** back, reads back, then triggers each enabled subinterrupt among bits
0..4. Subinterrupt-enable/disable callbacks at `0x81003B84` / `0x81003BD0`
maintain the mask and write DSI+`0x54` when the head is enabled.

Display registers **DSI0 IRQ 213, subinterrupt 1** with callback
**`display:0x81002940`** at `0x81002B9A`; DSI1 IRQ 210 uses subinterrupt 1 too.
Thus **DSI status bit 1** is the observed vblank source. The callback increments
the native head counter, pulses its native wait event, and triggers **SGI 8**
for selected-head presentation work. The logo's repeated waits depend on that
native delivery. A host frame callback alone does not substitute for it.

## Concrete mismatch with the current emulator

`src/hw/soc/soc_internal.h:137` places its assumed display at **`0xE2100000`**,
size `0x1000`. `display.cpp:9..29` explicitly documents an invented single
register window. The 1.04 driver instead maps the IFTU/DSI windows above.
Its buffer addresses, separate width/height, padding, format `0x10`, blend state,
and timing IRQs do not match that assumed map. Merely changing its base is
insufficient.

`display.cpp:199` copies guest pixels only when the invented DMA_CONTROL is
written; `tick()` at line 219 advances counters/callbacks without refreshing
pixels. None of the recovered native presentation paths writes that register.
`kermit.cpp:717` installs only the assumed DisplayController and does not
connect native IFTU204/205 or DSI213 delivery. The color-bar self-test verifies
the host upload path but does not validate this firmware contract.

A subsequent implementation can read the framebuffer PA and descriptor written
by genuine guest Lowio, use 4-byte pixels/pixel pitch for the native logo subset,
respect observed enable/blank/fade state, and deliver recovered timing/IFTU IRQs.
It should expose unresolved bank timing and blending choices explicitly and
validate them against a live native driver trace before claiming full fidelity.
It must not copy this offline asset to guest RAM or fabricate a logo result.

## Next live evidence to capture

- Actual relocated logo producer call and its returned memblock VA/PA; successful
  decompression and real SetFrameBufInternal result.
- Lowio writes at E5020000/E5021000/E5022000 and E5050000 during native init.
- Plane+4 bit 1 before/after deferred update; plane+180 stores; IFTU status+40
  bit value and order relative to DSI status bit 1.
- Native DSI213 sub1 callback, SGI8 presentation callback, and logo wait release.
- Final guest RAM pixels and native scanout presented by the host frontend.

This report establishes the real asset, native producer, physical map, descriptor
layout, format encoding, and ACK sequences. It does not establish that os0 has
loaded this module or that the actual guest white logo has been displayed.
