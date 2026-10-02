# Prospective FW1.04 SPI / GPIO boundary — 2026-10-01

This is read-only preparation while the latest reached native boundary is
Lowio's I2C idle poll. It does not claim Syscon or OLED startup was reached.
No production changes, shared build, slave ACK, or interrupt injection were
performed for this audit.

## Inputs and reproducibility

The supplied `Vita_104_Firmware/Out/fs_dec/os0/kd/{lowio,syscon,oled}.elf`
images contain the actual register sequences. Their text starts at linked VA
`0x81000000`, ELF file offset `0xA0`. Addresses below are linked addresses;
live Syscon/OLED relocation bases must be recovered after those modules load.
Lowio is already observed at text VA `0x5A8000`, BSS VA `0x4F2000`, so a
Lowio linked address `0x8100xxxx` maps to `0x5A8000 + xxxx` for text.

Saved disassemblies:

- `build/goal-display-lowio-thumb.txt`
- `build/goal-native-spi-syscon-thumb.txt`
- `build/goal-native-spi-oled-thumb.txt`
- `build/goal-nskbl.bin`: raw live NSKBL, base `0x51000000`.

The Thumb disassemblies also run across data and ARM import stubs. Findings
use actual Thumb driver bodies and genuine SCE import/export NID tables,
rather than treating those mixed regions as decoded Thumb instructions.

## Existing model versus genuine usage

| Register | Native evidence | Current implementation |
| --- | --- | --- |
| SPI RX `+00` / TX `+04` | 16-bit FIFO words, low byte first in Syscon | Pops/pushes two bytes (`spi.cpp:55`, `:87`) |
| SPI control `+08` | Syscon `0`; OLED `0x30001` | Stored only (`spi.cpp:93`) |
| SPI `+10` | Syscon starts after queued packet; OLED starts before FIFO writes | One synchronous snapshot on write1; later FIFO writes never consumed (`:103`, `:114`) |
| SPI status `+24` | Syscon clears `0x600`; separately diagnoses bit `0x200` | Reply unconditionally sets `0x200`, named RX-not-empty (`:127`) |
| SPI RX count `+28` | Native reads to decide whether receive data exists | Queued byte count (`:76`) |
| SPI TX `+2C` | OLED waits while full `0x7F`, then waits until0 | Unbounded queued byte count (`:77`) |

SPI0/1/2 bases are `E0A00000`, `E0A10000`, `E0A20000`. Supplied Syscon/OLED
mapping requests are `0x1000`; the model attaches `0x10000` windows
(`kermit.cpp:689`). SPI1's native controller IRQ is not recovered from these
images, and the supplied module set contains no motion driver.

`Spi::update_irq()` emits ID 0 (`spi.cpp:133–137`), but **no SPI IRQ callback is
attached** in `kermit.cpp` or elsewhere in production. There is no existing
ID 64+port remap. `kermit_attach_syscon_spi()` only attaches SPI0's Ernie frame
callback (`kermit.cpp:621–624`) and mirrors controllers across buses. SPI1/2
have no slave. Mask writes do not recompute the optional IRQ, and reset does
not notify a previously asserted callback; these are latent model concerns,
not evidence for a dedicated native SPI interrupt ID or mask formula.

## Syscon response-ready is GPIO0 IRQ 248 / subinterrupt4

Native Syscon initializer `81001674` maps `E0A00000/1000` at `8100179E`.
The mapped pointer is stored at linked BSS `81008098`. At `810017C8` it
registers subinterrupt `(248,4)` with callback `81000161`; `810017D0` enables
that subinterrupt. Genuine import NIDs identify these as IntrMgr
RegisterSubIntrHandler (`96576C18`) and EnableSubIntr (`901E41D8`).

The native transmit routine `8100003C` clears GPIO pin3, drains RX, W1C-clears
SPI `+24=0x600`, queues the request/checksum through `+04`, writes `+08=0`,
then `+10=1` at `8100011E`, and sets GPIO pin3 at `8100012A`.

The genuine response handler `81000160` acknowledges GPIO pin4 at `8100019A`.
It flags empty RX (`+28=0`) as request flag `100000`, nonempty TX (`+2C!=0`)
as `200000`, and SPI `+24` bit 0x200 as `400000` while clearing that bit
(`810001F0–206`). It drains reply words, stops `+10=0` at `81000350`, and
clears GPIO pin3 at `8100035A`. **Do not assert bit 0x200 means RX readiness:**
availability is separately checked through `+28`. Conversely, bit 0x200 alone
is not proven fatal: native synchronous result helper `81002728` uses error
mask `B00000`, which excludes request flag 0x400000.

Native Lowio parent registration at `81002480` sets IRQ 248, name
`SceGpio0Gate0`, handler `81002299`, type0, priority 0x50, target_cpu maskF.
Gates1–4 register IRQ 249–252, priorities70/A0/C0/D0, target 0xF. Parent handler
`81002298` uses the pin read register plus native state to select a pin,
W1C-clears its gate status at `+38 + gate*4` (`810022FA`) and dispatches the
actual pin subinterrupt (`81002310`). This GPIO gate path, rather than SPI
ID 0 or an assumed64+port, is the proven native Syscon completion route.

## GPIO register overlap and boot checkpoints

GPIO0 `E20A0000/1000` and GPIO1 `E0100000/1000` are the native maps. GPIO0
is currently the single `CMeP.GPIO` device, mirrored to ARM at the same PA
(`vita.cpp:467–472`); do not add a competing overlapping device.

| Offset | Native/primary contract | Existing shim |
| --- | --- | --- |
| `00` | Direction | Data/checkpoint accumulator |
| `04` | Read pin state | Bit4 debugger handshake, cleared by first read |
| `08/0C` | Output set/clear | Modify stored `+00` |
| `10` | Boot writes `FF0000`; exact meaning still unresolved | Named direction |
| `14/18` | Packed two-bit interrupt modes | Undefined stored register words |
| `1C..2C` | Five active-high gate masks | No IRQ behavior |
| `34` | Read latch/barrier read | No native behavior |
| `38..48` | Five W1C gate status words | Ordinary storage |

Native Lowio SetPortMode export NID 372022A4 points at `810028A5`. Its body
XORs API mode with 1 at `810028FA`, updates direction at `+00` (`81002918`),
and saves the native shadow. Thus Syscon's API mode 0 for pin3 writes hardware
direction 1 (output); API mode 1 for pin4 writes direction 0 (input). Pin4
interrupt mode 3 is programmed at `8100176A`; Lowio updates its packed mode
bits at `+14` (`810029DE`). This ABI inversion resolves the apparent mode
constant discrepancy; preserve it rather than guessing from symbol names.

Genuine NSKBL init `51010FCC` reads `E20A0000`, ORs `00FF0000`, writes `+00`
at `51010FE4`, then writes `00FF0000` to `+10` at `51010FE6`. The checkpoint
writer `51010F98` writes complement/code masks only to clear/set `+0C/+08`
at `51010FC4/FC6`. Therefore checkpoint bits 16–23 are output-latch state,
separate from direction. `debugger.cpp:1039–1045` currently reads `+00` and
depends on the conflation. A later GPIO change must preserve a real output
latch for checkpoints and adjust that debugger observation accordingly.

The existing CMeP debug handshake (`mailbox.cpp:660–670`, tests:571) consumes
pin4 on read. That is a development external-agent substitution, not native
GPIO behavior. The same physical pin is Syscon response-ready later. Before
replacement, capture whether that debug path executes in the selected boot,
then gate any retained debugger/JIG peer behavior by actual transaction or
boot stage; ordinary pin reads must not consume an Ernie falling-edge event.
The existing E010 `CmepStorageDevice` is also GPIO1 according to both the
primary map and matching boot offsets, but extending it is separate work.

## OLED streaming differs from SPI0 framing

OLED maps `E0A20000/1000`, configures `+08=30001,+14=F,+0C=3,+20=0`.
It does not import IntrMgr or register a SPI completion interrupt.
The first-open branch `81000750` drains RX, writes control 0x30001 at
`81000776`, then start 1 at `8100077A` **before** the body emits TX words.
The body packs reversed-byte command/data into nine-bit wire tokens and
queues 16-bit words, waiting while `+2C==7F`. Close `81000160` waits for
`+2C==0` at `810001C6–CE` and drains RX. The current model consumes the empty
FIFO once at start, then leaves subsequent words queued forever.

A future bounded streaming subset can initially be gated by **port 2 and
exact captured control 0x30001 plus start-active state**. Preserve SPI0's
native control 0, prequeued packet, one complete Ernie-frame callback.
Do not send each OLED word to the Ernie packet parser or fabricate receive
data/panel readiness. Full mode bitfield, timing, FIFO count units, reset
behavior, and an OLED slave response are not yet proved.

## Next live probe plan

1. After I2C integration, identify the actual next module/wait before edits.
2. Dump GPIO0 physical `E20A0000..48`, native Lowio state at current
   `4F2120..4F216B`, and GIC 248 enable/target/priority/pending before/after
   native pin4 mode and subinterrupt enable. Record checkpoints separately.
3. Once Syscon text relocation is known, break at linked offsets `17C8`,
   `11E`, `12A`, and `160`; capture actual command packet, GPIO 3 transition,
   RX/TX counts, bit 0x200, GPIO gate masks/status, and native completion result.
4. If OLED actually starts, break at offsets `77A`, first TX write, and `1C6`;
   capture control/start and queued/drained words. This validates the mode
   gate before any streaming implementation.
5. Future regressions should replay these native sequences, prove Syscon
   frame/checkpoint preservation, input-edge latch/masking/W1C behavior,
   and SPI2 TX drain independently. No synthetic successful slave response.

## Primary reference links

- [Vita baremetal SPI driver](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/spi.c)
  corroborates the three bases, port 2 configuration, Syscon framed sequence.
- [Vita baremetal Syscon driver](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/syscon.c)
  corroborates GPIO 3 output and GPIO 4 falling response-ready handling.
- [Vita baremetal GPIO implementation](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/gpio.c)
  corroborates register offsets, inverted gate masks and W1C status.
- [Vita baremetal OLED driver](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/oled.c)
  corroborates port 2 start-before-TX, nine-bit packing and TX drain polls.
- [Vita Linux SPI driver](https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/drivers/spi/spi-vita.c)
  corroborates polling register use; it does not prove dedicated SPI IRQs.
- [Vita Linux device tree](https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/arch/arm/boot/dts/vita.dtsi)
  gives GPIO0 parent GIC_SPI 216..220 = hardware IDs 248..252, level-high,
  and Syscon GPIO 4 falling-edge with GPIO 3 transmit line.
- [IntrMgr kernel header](https://github.com/vitasdk/vita-headers/blob/master/include/psp2kern/kernel/intrmgr.h)
  gives registration stack arguments, including priority and target_cpu.

Conflicting unused direction constants in the later Linux GPIO driver are
not used to override the genuine FW1.04 code and its explicit API inversion.
