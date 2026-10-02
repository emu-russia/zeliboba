# Firmware 1.04 DSI0 timing contract and bounded model proposal

Read-only recovery, 2026-10-01. No emulator source edits or shared build.
This extends [the native Display/IFTU report](goal-firmware104-display-contract.md).
It proposes timing hardware that produces the guest's genuine interrupt;
it does not call native callbacks, supply a logo, or select an IFTU bank itself.

## Evidence and reproducibility

Primary binary: decoded firmware 1.04 `os0/kd/lowio.elf`, SHA256
`8d6ff20781db00725bb7feb313f80f892a58ac04e947df4dbdaebaee37be881b`.
Display: SHA256 `86bbe8a1944f7657de70253c9a96c71b56bc442f70dc49c7dfe2390b6141ed91`.
Addresses `0x8100....` are linked VAs within the named module; first-segment
file offset = VA − `0x81000000` + `0xA0`.

- [Selected byte/VA/file-offset disassembly and VIC0 table](goal-display-dsi0-contract-evidence.txt).
- [Native StartDisplay execution log](goal-display-dsi-native-start-probe.log).
- [Isolated probe source](goal-display-dsi-native-start-probe.cpp).

The isolated probe loads the actual Lowio instructions and actual VIC0 timing
record into scratch RAM, initializes the caller's head state for 24-bit DSI
output/two lanes/subinterrupt 1, and executes the real StartDisplay routine on
the existing ARM core. Only imported synchronization lock/resume routines are
return stubs. A recording device captures MMIO; no device timing or guest boot
is simulated. The routine returns zero after 205 instructions. Compile/run:

```sh
c++ -std=c++17 -O2 -Isrc build/goal-display-dsi-native-start-probe.cpp \
  build/libzeliboba_core.a -o build/research/zeliboba-display-dsi-probe
build/research/zeliboba-display-dsi-probe
```

Public primary project code corroborates the base, VIC0 progressive geometry,
24-bit/two-lane timing calculation, status read/write-back and mask 2, and its
stop routine polls the idle low nibble. Its register-array indices are words:
`dsi_regs[0x14]` is byte offset `0x50`. This is corroboration, not permission to
import later-version behaviors into 1.04.
[vita-libbaremetal dsi.c](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/dsi.c)

## Mapping, progressive mode, and actual startup writes

`lowio:0x81003EA4` maps DSI0 **physical `0xE5050000`, size `0x1000`** and
registers **IRQ213 (`0xD5`)** with top-level handler `0x81003AE0`.
DSI1 is `0xE5060000`/IRQ210 and is outside this proposed initial subset.

VIC0 lookup record at `lowio:0x81008F4C` points to **`0x81009024`, file+`0x90C4`**:

| Field offset | Native value | Recovered role |
|---|---|---|
| `+0x00` | 5 | timing flags |
| `+0x04` | `0x223A1C` = 2243100 | clock-configuration selector/value |
| `+0x08` | `0x81009338` | PHY subtable |
| `+0x14` | 1050 | horizontal total |
| `+0x18` | 594 | vertical total |
| `+0x1C` | 0 | progressive mode |
| `+0x20`, `+0x24`, `+0x28` | 20,66,4 | horizontal front/sync/back periods |
| `+0x2C`, `+0x30`, `+0x34` | 4,4,42 | vertical front/sync/back periods |

Active geometry is 1050−20−66−4 = 960, 594−4−4−42 = 544.
Framebuffer input remains four bytes/pixel; **DSI link output 24 bits** is a
separate parameter and does not make the framebuffer a three-byte format.

`ksceDsiStartDisplay`, **Lowio `0x8100517C`**, head0/VIC0/control argument 0:

| DSI0 byte offset | Actual native write | Store linked VA |
|---|---|---|
| `+0x004` | `0x00000000` progressive control | `0x8100522A` |
| `+0x008` | `0x00000C4E` = 3150 horizontal timing units | `0x8100526A` |
| `+0x00C` | `0x00000252` = 594 vertical total | `0x81005272` |
| `+0x010` | `0x00010001` | `0x81005346` |
| `+0x014` | `0x00D20C13` | `0x8100534A` |
| `+0x01C` | `0x002E024F` | `0x81005350` |
| `+0x024` | `0x00C60001` | `0x81005354` |
| `+0x02C` | `0x00010001` | `0x81005358` |
| `+0x030` | `0x00040C4E` | `0x8100535A` |
| `+0x03C` | `0x00000001` | `0x81005398` |
| `+0x040` | `0x014F0026` | `0x810053A2` |
| `+0x05C` | `0x0001024F` | `0x810053CC` |
| `+0x060` | `0x0001002E` | `0x810053CE` |
| `+0x06C` | `0x40010157` | `0x810053D2` |
| `+0x070` | zero | `0x810053D4` |
| `+0x050` | reads status then writes same word | `0x810053D8`, `53E0` |
| `+0x054` | **2**, enabled subinterrupt-1 mask | `0x810053E2` |
| `+0x838` | zero for head 0 | `0x810053F0` |
| `+0x508` | `0xFFFFFFFF` | `0x810053F4` |
| `+0x000` | **1**, ordinary running/start control | `0x81005404` |

Final control values come from table `lowio:0x81009230`, words `{1,2,2,3,4}`.
Values 2/3/4 are distinct supported driver modes, so a generic “bit0 enables
everything” interpretation is unjustified. A bounded initial model can support
**control 1 / progressive VIC0 timing** and store other modes without pretending
to understand their trigger/interlace behavior.

## Native vblank status, ACK, mask, and callback route

Top handler **`lowio:0x81003AE0`**:

- `0x81003AEC`: reads DSI+`0x50` into `r4`.
- `0x81003AEE`: writes **that same word** to DSI+`0x50`, DMB, read-back, DSB.
- `0x81003AFA`: loads the cached enabled-subinterrupt mask.
- `0x81003B0C..16`: tests status bit **1** and enabled mask bit **1**.
- `0x81003B5E..64`: calls native `TriggerSubIntr(IRQ213,1,1)` through import
  NID `0xCC94B294` when both tests pass.

This supports **latched status with W1C ACK at +`0x50`**, interrupt mask
at +`0x54`, and a device IRQ level based on pending enabled status. Masking
delivery must not fabricate consumption of a pending status. The precise
set-while-masked hardware behavior was not independently measured; retaining
raw pending while masked is the conventional explicit model choice.

Subinterrupt callbacks `lowio:0x81003B84` / `0x81003BD0` set/clear bit `1<<sub`
in the software mask and write that mask to hardware+`0x54` when the head is
enabled (`0x81003BB8` / `0x81003C00`). StartDisplay publishes the mask after
clearing old status. DsiEnableHead initially writes +`0x54` = 0.

Display registers IRQ213 **subinterrupt1** with callback **`display:0x81002940`**
at `0x81002B9A`. That callback increments its native head counter and pulses
the native wait event; selected-head work also requests **SGI8**. These native
handlers and scheduler events release the boot-logo routine's repeated vblank
waits. A model must raise IRQ213 through the normal GIC and allow these guest
instructions to run; it must not call Display callbacks or set wait words.

## Time/rate evidence and clock limits

`display:0x81000D8C..0x81000D96` stores **float bits `0x426FC29E`** as the
ordinary head refresh rate: **59.94005584716797 Hz**. This is the float-rounded
form of the approximately `60000/1001` Hz rate stated in the local preliminary
SDK Display Overview. A period near **16683.333 microseconds**, not 16667,
is justified for the recovered ordinary head0 mode.

VIC0's raw clock value **2243100** is passed to Lowio's clock-configuration
selector at `0x81001808` by EnableHead. That routine chooses PLL/divider data,
programs a separate Pervasive clock block, and uses 1000-microsecond delays.
Public Pervasive code corroborates the same clock selector and delays. The
physical link clock's exact units/divisors are not established here, and the
timing proposal does not pretend the raw value is the pixel frequency.
[vita-libbaremetal pervasive.c](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/pervasive.c)

Emulator time is explicit: `KermitBlock::tick` converts accumulated A9 cycles
to **1MHz peripheral ticks = microseconds** before invoking device tick
(`kermit.cpp` near lines 531 and 794, `soc_internal.h` clock constants).
The DSI device must consume these ticks directly; a second CPU-cycle conversion
would give the wrong rate. Devices continue advancing with emulated machine
time while a core waits.

## Readiness and intentionally unsupported packet behavior

There is **no DSI readiness polling loop** in the observed DsiEnableHead
`lowio:0x81004460` or StartDisplay path. EnableHead programs PHY control and a
finite command script through **+`0x500`**; it sets +`0x518` = 1, writes
+`0x940`/`944`/`948` = `0x200`, then unmasks the native head IRQ. Clock setup
outside DSI uses its real guest delay API. No fake ready bit is required by
the reached ordinary StartDisplay/vblank subset.

Shutdown/reconfiguration does have concrete polls:

- **SendBlanking `lowio:0x810057A8`** queues words at +`0x500`, writes
  +`0x50C` = `0x01000000` and +`0x508` = `0xFFFFFFFF`, then **waits for
  +`0x414` bit24** at `0x8100584A..52`. It acknowledges through +`0x514`,
  finally writes **+`0x000` = 0** at `0x8100586C`.
- **DisableHead `lowio:0x810050D0`** waits until **+`0x48` low nibble is zero**
  at `0x8100510E..14`, then deinitializes PHY registers and clock gating.
- The getter at **`lowio:0x810058A4`** reads **+`0x4C` bits[28:16] and
  subtracts 1**, supporting a one-based scanline field. Exact other bits and
  phase relative to the vblank interrupt are not recovered.

A timing-only DSI model must identify packet execution/blanking completion as
unsupported. Storing command writes is honest; unconditionally supplying
+`0x414` bit24 would be a fabricated completion. An idle reset value of zero
at +`0x48` is a bounded idle model, not a packet-engine implementation. If the
native boot reaches these polls, capture the real command submission first and
implement its completion separately. Public code has a later +`0x51C` write
absent from the recovered 1.04 head0 path; do not require it for native startup.

## Minimal proposed DSI0 model

1. Install one `0x1000`-byte device at E5050000. Preserve ordinary register
   writes; implement W1C status at +50 and enable-mask at +54 with correct byte
   access semantics. Reset status, mask, control, and phase to zero.
2. Support the observed progressive head0 subset: control **1**, progressive
   +4=0, horizontal timing +8=`0xC4E`, vertical total +C=`0x252`. Other modes
   remain stored/unsupported; no invented readiness or event synthesis.
3. Use an integer phase accumulator for elapsed **microseconds**:
   `phase += ticks * 60`; every **1,001,000 phase units** represents one frame.
   Keep fractional remainder. This yields 60 frames per 1.001 seconds and
   preserves chunking independence. Use a quotient/remainder implementation
   to avoid overflow on large tick arguments.
4. At each frame boundary latch **status bit1 (`2`)**. Multiple unacknowledged
   frames coalesce in that status bit, rather than queueing artificial guest
   interrupts. Drive **IRQ213 level = (status & mask & 2) != 0** through the
   existing GIC route; recompute immediately on ACK and mask writes. No forced
   GIC enables/targets, unmasking, CPU wake bypass, or native callback call.
5. Control zero stops future frame generation without clearing a previously
   latched status by itself. W1C and mask determine pending delivery. A fresh
   stopped→supported-running transition starts a documented phase; the exact
   physical line of the first interrupt is a model approximation.
6. A host scanout refresh may inspect the real guest-written IFTU configuration
   at frame time. It must not alter native wait state or IFTU bank selection.
   Automatic IFTU turnover and blend equations remain separate, labeled gaps.
7. If a native caller needs the +4C getter, derive its one-based line field
   from the same progressive phase/594-line geometry; do not claim the unknown
   low bits or exact line/interrupt phase are implemented.

Useful focused regressions: no event while stopped; no event at 16683us but one
after the next microsecond; phase stable across split tick calls; native mask2
asserts GIC213; zero status write retains pending, W1C2 clears it; mask0 deasserts
without consuming pending and re-enabling mask2 restores delivery. EOI must
not manufacture another pending event after the device ACK. A native driver
trace should then verify DSI213→sub1→Display wait event/SGI8→logo fade progress.

This is a bounded timing/interrupt proposal, not a DSI PHY/packet engine, full
display specification, or evidence that the real guest logo has been shown.
