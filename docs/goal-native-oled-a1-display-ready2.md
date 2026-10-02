# Native OLED A1 identification and Display's rejected-panel path

Read-only recovery from the genuine supplied 1.04 `oled.elf` and `display.elf`, the existing native listings, and primary public driver/protocol sources. No guest memory, return value, readiness flag, source or build was changed; no extra one-million-slice run was launched. Integrated572 has not yet executed this path: its OLED worker remains behind the native zero timer before SPI2 TX. These are static predictions for the next genuine capture.

## Five returned bytes and GPIO branch

The worker is OLED+780 (validated runtime004F4780). It reads GPIO0 pin0 through import129DF5AC. Low pin: native PortSet D454A584 at+792 raises it, waits20ms at+79C, then reads commandA1 length5 at+7A6 and restores low through PortClear F6310435 at+7EC. High pin: reads the same A1/len5 at+7FA and leaves it high. Correcting the GPIO+34 output latch can change which branch/delay is taken, but neither branch skips RX. Earlier descriptions reversing these Set/Clear imports should be corrected; the genuine NIDs match the primary Lowio names.

At+7AA/+7FE the last byte must equalFF. If so, +806..82A saves byte0|byte1<<8 at OLED BSS+8 and byte2|byte3<<8 at BSS+A. Then +7C0..7CA additionally requires the first saved word to equal0. Only that case publishes ready1 at BSS+4 (+7CE). Bytes2/3 do not participate in this startup acceptance predicate. A completed other reply disables the SPI2 clock and asserts its reset at+7D2..7DA, then publishes ready2 at+7E0. Worker returns1 either way.

Thus a hypothetical continuous high input unpacking to FF,FF,FF,FF,FF is rejected (supplier wordFFFF), stores both wordsFFFF and yields ready2. Continuous low input unpacking to five00 bytes is also rejected (last byte is notFF). Neither is a valid panel identity. An empty RX queue leaves the read helper waiting at+336; that is current device-model behavior, not proof that disconnected physical hardware has an empty receive FIFO.

OLED+8F0 (`GetDDB`, NIDC9D5987C) returns803F0A04 while ready0,803F0A03 for ready2 and only writes output words for ready1. The second word later selects native initialization/revision tables at+B78..C9E: low byte0 with full word0,0200 or0300 has dedicated paths; low bytes1..5 have others, and remaining values select the default table. Do not select a reply just to choose a desired revision table.

## Defaults do not avoid the first read

OLED module_start+834 calls importBBE1771C (`SearchModuleByName`) with string `SceSamantha` at+1950. If another module is found, it returns2 before OLED initialization/worker queuing; if not found it initializes and queues worker+781. This is module-presence dispatch, not a disconnected-panel test. The genuine572 boot already queues/executes the OLED worker, so this skip is not the current default and must not be forced.

## Actual Display behavior for ready2

Display worker+F14 calls OLED WaitReady at+F1A. OLED+8C4 waits only for BSS+4 !=0 and returns0 for ready1 orready2, so rejection is a completed initialization state.

Display+3470 contains a weak imported-function-address presence check, not a call to GetDDB. The genuine import stub+3E2C has reference-list pointer810063E0; that list carries MOVW relocation47 at+3470 and MOVT48 at+3476 for the imported GetDDB address. The presence check initializes Display BSS+1D8 to1 when that function exists. It does not inspect the A1 supplier/revision result. No GetDDB call appears in Display's startup listing.

Head setup+4C4 configures/enables IFTU A andB at+552/+594, before OLED helper+3538 at+5B8. The helper calls OLED NID9F4ABDDC -> OLED+8B0, which reads GPIO0pin0 and returns1 iff it islow; this is not a readiness getter. If pin0 is alreadyhigh, helper+3538 returns0 without initializing the OLED scripts. If low (as after the default worker branch), it sets OLED script-mode flag viaNIDED2D6F19 ->+1300, runsNIDDDB1412B ->+B00, then clears script-mode.

OLED+B00 raises GPIO0pin0, waits120ms, runs its first command table and calls GetDDB with second output atSP+6. It ignores GetDDB's return at+B74 and reads the unchanged stack halfword at+B78 even if GetDDB returned803F0A03. Consequently the exact later revision/default table under ready2 depends on real stack bytes, and must be captured instead of assumed zero. Its command-table parsers+A64/+1324/+13C8 ignore each command-write return while advancing pointers/delays; ready2 write+954 returns803F0A03 without clocking SPI. Normal completed table paths in+B00 return0, while an already-running script can return803F0A05. Native time must work for these script waits to terminate.

If helper+3538 returns0, head setup calls DSI Start at+5D2 and marks head enabled at+5E4. If any head-setup call fails, Display worker's +101E branches back to+FCE (unlock), then still evaluates the boot predicates at+FDA/+FE8/+FF0/+FF8. Provided those actual predicates allow it, +1000 calls the native logo producer+3048 regardless of that head-setup error. There is no ready2 check on this route.

Producer+3048 allocates its genuine framebuffer at requestedPA1C000000, gzip-decompresses its own embedded asset at+30AC and calls SetFrameBufInternal+1708 at+30DE. The latter validates descriptor/format/ownership and reaches helper+130 -> native IFTU SetInput+3D8C; its recovered body has no OLED-ready or head-enabled rejection. It does not inject host/extracted pixels. The native producer then waits real vblank events during merging/fade. Whether the exact rejected-panel path reaches each milestone is a live prediction, not an observed logo.

## Controller and physical input evidence bounds

Native command-read+1EC uses exactCTL30001, starts before FIFO writes at+77A, emits genuine low16 words and padding, drains TX at+31E, writes start0 at+326, then consumes retained RX at+336. It discards two9-bit tokens before the five payload bytes for A1. FIFO words are appended least-significant byte first and the parser reverses each8-bit value from token bits1..8. A1/len5 emits nine16-bit words (18bytes) including padding; a sampled constant-high18byte RX stream therefore yields fiveFF bytes naturally, without a command-specific reply.

The primary baremetal OLED implementation independently uses the sameCTL30001/start-before-TX/low16/RX-drain sequence, but its initialization is write-only and supplies no A1 identity or disconnected-input level. The primary Linux Vita SPI driver corroborates low-byte-first16-bit FIFO access and RX availability counted against byte-buffer lengths in its control0/eight-bit path; it does not decode30001 or prove the OLED FIFO capacity/count units. The native30001 read routine itself proves that this mode must capture receive data for clocked TX before stop. It does not establish an exact control bitfield, sample edge, separate MISO versus shared SDA, or undriven pull level.

Public Linux's AMS495QA01/Magnachip panel driver is primary source code with matching960x544 timings and a similar command family, but it does not read any panel ID/DDB. Its existence does not supply the actual Vita response. Himax's authored HX8357-D datasheet (a different panel/controller) gives the same A1 schema on physical PDFpage193: after a dummy, two supplier-ID bytes, two supplier-elective bytes, then an exit/escape code; FF means the block ends. This supports DDB interpretation, not Samsung identity bytes or Vita wiring.

## Bounded proposal and meaningful tests, after genuine TX is reached

A defensible explicit board stand-in could attach an undriven input-level peer to SPI2 rather than a panel-ID responder: each actually consumed16 TX wirebits produces the sampled16 RX bits, with defaultlevel chosen and clearly labeled as an unverified board pull assumption. Keep command parsing out of this peer. A high-level stand-in would receiveFFFF per word, and native firmware would reject that naturally. This is acceptable evidence for controller clocks/data flow, but cannot be called connected-panel emulation or recovered Vita pull-up behavior.

Scope only the native port2/exact30001/gueststart1 streaming subset. Arm on an empty start, consume actual queued/subsequent low16 TX words once, append two RX bytes per consumed word in existing low-byte-first order, and retain RX across gueststart0. Reset clears engine and FIFOs; a stopped engine creates no new RX. Keep SPI0's whole-Ernie-frame callback and other modes separate. Do not synthesize an OLED-success IRQ or replace driver errors; existing RX interrupt-latch behavior can apply to actual queued bits through the existing mask. Do not infer127-word capacity or clock rate from the native7F guard.

Tests should replay the unchanged native A1 packing/unpacking and worker with an explicitly constant-high wordpeer: actual18 TX bytes, nineFFFF words sampled, FIFO+28 initially18 under the existing byte-queue model, +00 popsFFFF and decrements count2, stop retainsRX, actual destination fiveFF, ready2 and nativeGetDDB/write803F0A03. Closing drains surplus RX. A constant-low peer should also reject, proving that the wire stand-in is not tailored to a valid identity. Mixed input-word test1234 should pop1234 from queued bytes34,12, independently checking endian behavior. No clocks before start, no clocks afterstop, reset cancellation, wrongCTL/port and existing SPI0 native-frame tests bound lifecycle and regression scope. A full ordinary boot must then observe the exact +B74 return/SP+6/table, head return, DSI control, genuine gzip/SetFrameBuf and guest-RAM pixel digest before claiming framebuffer/logo progress.

## Primary links

- [Vita baremetal OLED source](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/oled.c): matching transport/configuration, no ID response.
- [Vita Linux SPI source](https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/drivers/spi/spi-vita.c): byte-order/eight-bit-mode receive contract.
- [Linux AMS495QA01 panel source](https://github.com/torvalds/linux/blob/master/drivers/gpu/drm/panel/panel-magnachip-d53e6ea8966.c): panel init/timing, no ID reads.
- [Authored Himax HX8357-D datasheet](https://cdn-shop.adafruit.com/datasheets/HX8357-D_DS_April2012.pdf), physicalPDFpage193: A1 DDB schema for a different controller.
- [Primary Modulemgr NID names](https://github.com/vitasdk/vita-headers/blob/master/db/360/SceKernelModulemgr.yml), [OLED names](https://github.com/vitasdk/vita-headers/blob/master/db/360/SceOled.yml), [Lowio names](https://github.com/vitasdk/vita-headers/blob/master/db/360/SceLowio.yml): only names with exact matching1.04NIDs are used, not3.60addresses.

Evidence inputs: `build/goal-native-spi-oled-thumb.txt`, `goal-display-display-thumb.txt`, `goal-display-module-tables.txt`; compact independently parsed imports are `build/goal-native-oled-display-imports.txt`. Prior572 live bounds remain `build/goal-native-emc-neon-graphics-evidence.md`.
