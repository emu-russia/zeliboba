# Firmware 1.04 IFTU deferred-buffer follow-up

Read-only recovery after the DSI0 integration, 2026-10-01. Emulator sources
remain frozen. The ordinary boot has not reached Display/Lowio initialization.
This extends [the recovered native contract](goal-firmware104-display-contract.md),
not a claim that a hardware flip or guest logo has been observed.

## Additional exact native evidence

- [Selected actual 1.04 instructions, bytes, and linked VAs](goal-display-iftu-turnover-evidence.txt).
- [Isolated native-routine execution log](goal-display-iftu-native-contract-probe.log).
- [Probe source](goal-display-iftu-native-contract-probe.cpp).
- [SetFrameBuf callers](goal-display-iftu-set-framebuf-callers.txt).

The isolated probe loads the supplied Lowio code into scratch RAM, clears its
plane-state array, initializes ordinary format/geometry and test PAs, and runs
the real SetOutputFormat, Enable, deferred SetInput, and IRQ-handler routines.
Synchronization/unmask imports return from local stubs; an ordinary memcpy
import stub copies the scratch descriptor. Recording MMIO supplies current-bank
0 or 1 as explicit test inputs. **No hardware IRQ, automatic movement or actual
boot is simulated.** The plane IRQ handler is invoked only to recover its stores.

### Output-mode fields and enables

SetOutputFormat at **Lowio `0x8100651C`** stores its fifth argument at software
plane-state **`+0x1E0`** (`0x810065AA`, `0x810065B4`). This is an output-mode
flag supplied by Display; it is not the deferred sync argument of SetInput.

Enable at **`0x8100690C`** produces these shared-control mode pairs:

| Software output flag | A pair shared+4 bits[2:1] | B pair bits[4:3] | Additional writes |
|---|---:|---:|---|
| 0, ordinary progressive head0 | `01` | `01` | initial config-select A+10 / B+18 = 0 |
| 1, special output mode | `11` | `11` | shared A+14 / B+1C = 0, plus initial selects 0 |

The A bit operations are at `0x81006AEA..0x81006B04`; B at
`0x81006B40..0x81006B6A`. The native-routine probe independently records:

- A Enable returns zero after 254 instructions and writes shared+4 = **2**,
  shared+10 = 0, shared+0 = 1.
- B Enable returns zero after 263 instructions and writes shared+4 = **0xA**,
  shared+18 = 0, shared+0 = 1, retaining A's mode.
- Each plane's +50 = 1, +58 = 0x108, and **+180 = 1** precede these stores.
- SetOutputFormat ordinary flag0 returns zero after 49 instructions.

Public [vita-libbaremetal IFTU code](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/iftu.c)
uses mode pairs `00` while writing shared+10/+18 as explicit configuration
selects. It establishes the manual path's select addresses, but does not implement
or document the native `01` and `11` behavior. Naming `01` as automatic alternation
is therefore an **inference**, not a documented mode enum.

### Opposite-bank prepare and old-bank replay

SetInputFrameBuffer **`0x810065E0`**:

1. Reads plane+4 bit1 at `0x81006680..82` as current bank.
2. Deferred sync argument `r3 != 0` writes the complete descriptor into the
   opposite bank at `0x81006754..62`.
3. Sets software pending to `0x80000000 | old_current_bank` at `0x81006766..6A`.
4. Makes no MMIO config-select write, +180 write, or explicit commit/flip write
   on this deferred path.

Native execution confirms both index cases on both head0 planes:

| Supplied current-bank test input | After deferred call | Software pending | After native handler |
|---|---|---|---|
| 0 | bank0 retains old PA; bank1 gets new PA | `80000000` | both banks have new PA |
| 1 | bank0 gets next PA; bank1 retains previous PA | `80000001` | both banks have next PA |

Each deferred call returns zero after 464 instructions. Handler execution clears
pending and copies into the old bank. It **does not read current bank again**.
An actual hardware bank change before the handler is consistent with this
prepare/replay protocol, but the protocol alone cannot establish its timing.

### IRQ204/205 status and rearm uncertainty

Lowio registers IRQ204 (A) and IRQ205 (B) using handler **`0x8100599C`**.
At `0x810059B8..BE` the handler reads plane+40, writes **zero** there, reads
back, then writes the cached optional flag to **+180**. The status value is
never tested or used to select a branch. Thus:

- Zero-write ACK is directly evidenced. This is not the DSI status W1C pattern.
- **No exact nonzero status-bit pattern is established by these instructions.**
- Default/cached +180 flag is 1; SetInput's third argument updates this cache
  only if it is nonnegative. A negative argument leaves the prior value intact.
- +180 is written at Enable and after each ACK, so interrupt enable/rearm or
  recurring trigger control is plausible, but its precise meaning is unresolved.

The recording probe intentionally leaves status at zero. The invoked handler
still replays the pending descriptor and returns `FFFFFFFF` after 82 instructions.
That is evidence about guest handler control flow, **not permission to raise an
IRQ without a hardware event**.

## Bound for a future model, after an actual native capture

A small model could associate native mode pair `01` with frame-boundary bank
alternation on DSI0, restricted to enabled bus/plane and the recovered +180=1
ordinary subset. Each frame would change current-bank state before latching a
per-plane completion event, let IRQ204/205 reach the ordinary GIC, and allow the
guest handler to ACK/rearm and replay into the old bank. All pixels would still
come from valid guest-written descriptors and RAM.

This remains a **proposed modeled behavior**. The firmware evidence does not
distinguish continuous alternation from a +180-triggered/rearmed transfer, nor
prove that each frame raises a completion when no buffer changed. A nonzero
completion token would have to be labeled model bookkeeping, not a recovered
hardware status bit. IRQ status coalescing and +180 gating also need explicit
model choices or further evidence; +180 must not become an invented FLIP register.

The current static IFTU model obtains its frontend scanout bank from shared
manual config-select. A future automatic mode must use one coherent active-bank
state for plane+4 bit1 and frontend scanout. Keeping shared+10/+18 as stored
manual requests while automatic scanout follows an explicit active state avoids
pretending the guest wrote a select. Whether hardware reads those selects back
as active state is not established. Mode `00` should preserve the proven explicit
select behavior; `11` remains unsupported until its timing is recovered.

No host implementation should inspect the guest's software pending word, call
the native handler, overwrite cached descriptors, release Display wait words,
or fabricate a completion solely because a new PA was seen. DSI0's vblank IRQ
and IFTU's plane-completion IRQ are distinct hardware paths even when a future
model relates them to the same frame boundary.

## Capture required once Display is reached

1. Log real Enable stores, shared mode/select values, plane+50/+58/+180, and
   actual GIC configuration for 204/205/213.
2. At real deferred SetInput, capture old plane+4 bit1 and both complete banks;
   confirm the requested logo PA/geometry and pending old index.
3. Observe any subsequent native writes before the first completion. Capture
   handler +40 reads, zero ACK and +180 rearm; do not assume status bits.
4. Verify current-bank/scanout consistency and old-bank replay across consecutive
   frames, plus behavior when IRQ is masked or +180 differs.

The captured guest producer/IRQ route, rather than the isolated test or a
plausible automatic mode label, remains the criterion for genuine logo progress.
