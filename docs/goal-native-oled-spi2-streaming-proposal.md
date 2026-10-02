# Bounded OLED SPI2 transport proposal before the next native gate

Read-only review against actual1.04 `oled.elf` and the primary project implementations linked below. Integrated572 never reached SPI2 start/TX; no additional one-million-slice probe or source edit was made for this proposal. All addresses below are linked81000000-relative; the validated current OLED text relocation is004F4000 and BSS004F7000.

## Native FIFO/wire contract

Initialization at8100009C..A2 writes SPI2 physicalE0A20000 configuration: +08=30001, +14=F, +0C=3, +20=0. The command-read helper810001EC first opens at81000750: drains RX, writes control30001 at776, then start1 at77A **before** any TX word. The command-write helper81000954 has the same first-open ordering at81000A5E/ A60. Native word writes use UXTH and store low16 bits at FIFO+04; they are not packets for Ernie.

The command/data token is `(reverse8(byte)<<1)|dc`, where dc=0 for command and1 for data. The native driver appends each nine-bit token at the current low-bit position, emits the accumulator's low16 bits through +04, shifts right16, and preserves leftover bits across words. The packing implies a low-word/low-bit serial order; that inference follows the software bit reversal and is not a measured wire trace. Interpreting each emitted low byte as an eight-bit panel command would lose both the bit reversal and command/data bit. Nine-bit tokens can cross FIFO-word boundaries.

Native write helper81000986..A30 packs caller command/data. Native close81000184..1C4 repeatedly adds a zero command token until the bit buffer becomes empty, emitting further low16-bit words. This aligns the padded stream to both9 and16 bits: nonempty commands can end at a multiple of144 wire bits, not merely a rounded byte boundary. This is emitted guest data; the transport must neither remove nor invent these padding words.

Independent arithmetic examples from the recovered packing algorithm:

| Guest request | FIFO low16 words, in order | Little-endian word bytes |
|---|---|---|
| WriteF0,data5A5A | `6A1E,02D5` then seven `0000` | `1E6AD502` followed fourteen zero bytes |
| Write11,no data | `0110` then eight `0000` | `1001` followed sixteen zero bytes |
| Native ReadA1,length5 | `030A,0804,2010,0040` then five `0000` | `0A03040810204000` followed ten zero bytes |

These examples are static expected outputs, not a claim that any integrated guest has transmitted them yet. They are useful golden vectors for later unchanged-driver/Bus regression replay.

Each enqueue polls +2C until it differs from7F (81000240,298,2F8, etc.; write helper9B4/A08). Close waits for +2C==0 at810001C6..CE. Both are hardware-queue observations, not panel acknowledgements. The native code does not prove whether the nonzero count is bytes, sixteen-bit FIFO entries, or mode-dependent. The current model's two-byte increment means its count is always even after word writes, so simply capping its unbounded byte queue at127 or calling127 a word capacity would introduce an unverified rule. Initial transport work can model immediate active-word consumption and expose only the proven drained zero without inventing FIFO capacity/rate. Backpressure/fullness remains explicitly unsupported pending live evidence.

## Native startup does require receive data

The actual first worker command at810007A6 is A1, destinationSP, requested length5. The same call is present at7FA in the alternate GPIO branch. The helper for command<=AF sends command plus length+1 zero-data tokens, pads, waits TX drain at8100031E, writes start0 at326, then **waits for RX count+28 nonzero** at336. It consumes sixteen-bit FIFO words, discards two initial nine-bit tokens and reconstructs five reversed-byte payload tokens into the caller's buffer. For commands>AF, the native read helper emits the FB/command/FC sequence and discards four initial tokens before requested payload; that vendor extension is not generic SPI framing.

The worker inspects response byte4 forFF at7AA..7B0/7FE..804. Only the FF branch copies four returned bytes into BSS+8/+A and sets its local recognition flag at806..82A. Its further GPIO/identity checks choose readiness1 or2 at7CE/7E0. The later write API81000954 rejects readiness0 and2. Neither five zero bytes, a fabricated FF marker, loopback, nor an invented panel ID is an acceptable transport completion. A native zero-length write/read path may drain RX without needing its contents; the startup A1 length5 path does not.

A TX-only implementation will therefore expose a subsequent legitimate unsupported RX/panel-input wait. It must report that limitation and must not increment RX count, set a successful response bit, write the destination or readiness global, raise an OLED IRQ, or invoke an Ernie reply to advance the worker. Physical full-duplex clocks may shift an input level into RX in real hardware, but the actual panel MISO/pull state has not been established here.

## Bounded transport design, only after the live TX gate is reached

1. Select the streaming subset only for port2 with exact observed CTL30001 and an explicit guest start1. Keep a separate armed-engine state from any instantaneous busy flag. An empty start arms subsequent writes rather than closing an empty transaction. Before start, and outside this subset, preserve the existing framed path.
2. On a guest +04 write while armed, consume exactly its low16 wire bits in order. A coarse synchronous hardware stand-in can complete that one word immediately, so +2C reaches zero because data was actually consumed, not because a read forcibly clears the queue. It must retain pending pre-start words and consume them at actual start. Do not guess a divisor, clock rate, capacity or interrupt source.
3. Preserve raw transmitted words/byte totals and bounded diagnostic samples. Future panel input should use an explicitly attached wire/word peer that returns supplied input; do not reuse SPI0's whole-Ernie-frame callback per OLED word. With no panel peer, TX completion alone produces no RX, status success or IRQ. Any future nine-bit parser must retain partial tokens across sixteen-bit words and consume actual guest padding.
4. Guest start0 and reset disarm the engine. Reset also clears queues, residual transport bits and diagnostic session state; later writes require a new start. A CTL change leaves the supported subset and must not silently continue its streaming behavior. Avoid changing status+10 semantics beyond the observed start/stop sequence until a live read establishes them.
5. SPI0 control0/prequeued packet behavior stays byte-for-byte compatible, with one Ernie callback per actual frame start. SPI1 remains separate. GPIO response-ready semantics/IRQ248 belong to the proven Syscon route and are not inferred for OLED.

This is a controller data-path proposal, not a claim to emulate an OLED panel, satisfy the A1 query, enable a framebuffer, or create a guest logo. No production change is authorized by this report alone; the parent will use the actual post-counter frontier to scope implementation.

## Meaningful regression plan

- Replay start-before-TX into actual Bus/SPI2 using the static native low16 golden sequence. Verify every emitted word/order and exactly18 accepted TX bytes, drained TX count, empty RX and no fabricated status/IRQ. Where practical run the unchanged native packing/close block rather than a copied host packer.
- Replay the native A1 length5 sequence with no panel peer: TX can drain, but RX count stays zero and the firmware destination/readiness remains unchanged. A bounded instruction execution should stop at the real receive wait, not return success. A fixture-only supplied RX stream can separately verify transport ordering/unpacking without becoming a runtime panel-response substitution.
- Exercise stop/reset between start and writes, then a fresh start: stale armed state/words cannot leak into the next request. Use observable TX sink/queue output, not internal flag assertions.
- Change port or CTL and verify the streaming rule is inactive. Replay existing SPI0 native framed four-byte round trip and ensure one callback, same low-byte ordering and response consumption; preserve the established byte-count contract in that mode.
- Do not add a7F capacity/fullness test until the count units/overflow policy are established. Do not add success-shaped synthetic panel bytes to a boot test.

## Primary corroboration and its bounds

The primary baremetal OLED source independently uses the same SPI2 configuration, start-before-write ordering, nine-bit reversed-byte packing, low16-bit emissions, zero-token padding,7F full check and zero TX drain. Its initialization only writes panel commands; it does not perform the native A1 query or prove a startup reply. [Baremetal OLED source](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/oled.c).

The primary generic SPI source establishes SPI bases and the separate prequeued control0 framed start/stop API. It does not decode the CTL bitfield, wire clock or FIFO capacity. [Baremetal SPI source](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/spi.c).

The primary Linux SPI implementation programs eight-bit framed transfers, packs two byte-buffer entries into each low16 write, and interprets RX availability against byte lengths. That corroborates current byte counts in control0; it is not a hardware specification of nonzero OLED CTL30001 TX count units. [Linux Vita SPI source](https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/drivers/spi/spi-vita.c).

Native byte evidence: `goal-native-spi-oled-thumb.txt`; frozen reached-state boundary: `goal-native-emc-neon-graphics-evidence.md`. No extra guest run or source/build mutation was performed for this proposal.


## Extension: chosen undriven high input, pending real post-timer gate

The original TX-only proposal above records the review before full-duplex/input analysis. This extension proposes a bounded RX implementation, subject to the real post-timer TX wait being captured. It does not assert the actual Vita pull network or connect a valid OLED panel. The complete static firmware reasoning is `build/goal-native-oled-a1-display-ready2.md`.

Exact observed port2/CTL30001/gueststart1 is the only new streaming subset. An empty start arms the engine. Each actually consumed low16 TX word clocks16 wirebits and enqueues sampledFFFF as byteFF thenbyteFF. This is an explicit chosen pulled-up undriven board input, independently of command contents; no command decoder, valid-panel ID, additional MISO profile or CLI/environment option is proposed. Record the board assumption in source comments and device diagnostics. Avoid labels suggesting a recovered electrical reset value. The native firmware itself demonstrates RX capture under this configuration; public primary sources do not decode the individual30001 control bits, input topology or pull level.

Queue counts reuse the existing RX byte queue: one consumed16-bit word adds2 bytes, one FIFO+00 word read pops2, and +28 reports those pending bytes. This reuses the control0 primary-driver byte contract; nonzero30001 native reads only test empty/nonempty, so do not claim a newly proven OLED count unit/capacity. A stop0 disarms further clocks but retains already captured RX, because genuine read+1EC stops at+326 before consuming it. Native next-open/close drain the remaining words themselves. Reset clears both FIFOs and the armed stream. Pending TX is consumed exactly once; no clocks occur merely on empty start or afterstop. A change away from supportedCTL disarms the streaming rule. Preserve SPI0 whole-frame Ernie behavior and other ports/modes.

The actual native A1/len5 packer emits9 words/144bits, so this input produces9 sampledFFFF words/18 pending bytes. The unchanged native read helper discards2 initial9-bit tokens, reconstructs its fiveFF payload bytes and drains surplus words onclose. Worker+780 recognizes the exitbyteFF but rejects supplierwordFFFF; it publishes ready2, notready1. GetDDB+8F0 and command-write+954 keep their genuine803F0A03 errors. No readiness, script, SetFrameBuf, DSI, IRQ status or return value is forced. Queue-derived existing RX latch/mask handling can remain; do not introduce an OLED-success interrupt.

The rejected-panel route can still naturally proceed through Display: WaitReady acceptsnonzero2, head setup enablesIFTU before its OLED call, and Display continues to actuallogo predicates even after a setup error. Its script parser ignores the nativewriteAPI errors. Actual +B74 error/SP+6/table selection, eventual headreturn, DSI start, gzip and SetFrameBuf must be captured before asserting success. No extracted logo bytes enter the model.

A meaningful unchanged-helper regression should embed the small genuine executable read/close regions with provenance/hash, preserving linked81000000 addresses and original instructions (including native low16 packing and9-bit unpacking), then execute an ArmCore against the actual SPI2 bus device. Set only ordinary isolated test input state: mapped linked code, BSS+14 peripheralpointer, stack/output buffers, argumentsA1/destination/5 andreturn sentinel. Clock-enable/disable imports can execute their original unbound MVN/BX stubs, whose ignored errors do not affect this helper; explicitly label that isolated fixture boundary instead of patching guesthelper returns. First run the same untouched region against the pre-change model to prove a drain/RX wait, then against the new model to require finite return0, exactfiveFF output,18 realTX bytes and exhaustedRX. This test is portable and must not require external proprietary firmware files at test time.

Additional public-bus tests check start-beforeTX, no spontaneousRX, nativeA1 TXwordsequence030A/0804/2010/0040 plusfivezeros, count18 andstopretention, low-byte-first wordpops/countminus2, stop/reset cancellation andwrongmode/port exclusion. A worker/API fixture can additionally execute the genuine supplier/exitchecks and error paths, proving ready2/803F0A03 without forcing any global. Existing nativeSPI0 request/reply test must remain byteexact. No production source or test change is authorized by this document alone.
