# Native IFTU follow-up — frozen read-only review, 2026-10-01

The fresh 599-test binary reaches native IFTU0 A/B Enable returns 0,
shared bus control 1/mode 0xA and plane run+0x50=1, followed by native DSI0 start 0.
The graphics probe records 13 modeled DSI frames, but no logo producer, gzip,
SetFrameBuf or supported valid guest-RAM scanout. Root's longer ordinary run
records 23 frames and the same absence of valid scanout. The current stop is the
separately owned Syscon SIMD decode 0xF962070D at 0x4F8B56. **No IFTU turnover or
completion implementation is authorized or added by this review.**

Current live evidence: `goal-native-early-ready-full.log`,
`goal-arm-native-syscon-oled-early-ready.log/.json` and compact
`goal-native-early-ready-graphics-evidence.md`. Sources remain frozen.

## Exact supplied firmware and address mapping

`Vita_104_Firmware/Out/fs_dec/os0/kd/lowio.elf` SHA256 is
`8d6ff20781db00725bb7feb313f80f892a58ac04e947df4dbdaebaee37be881b`.
Its text PT_LOAD is linked 0x81000000/file+0xA0. Current native text is VA 0x5A8000;
separate BSS linked 0x8100B000 is VA 0x4F2000, as independently captured. Static bytes
are retained in `goal-native-iftu-599-static.txt` and
`goal-native-iftu-599-initializer-static.txt`; the latter uses skipdata across
literal pools, so the confirmed entry/path at81005D3C, not decoded pool words,
is the initializer evidence.

Lowio's physical table 0x81009498 gives IFTU0 A 0xE5020000/IRQ204,
B 0xE5021000/IRQ205 and shared 0xE5022000. IFTU1 uses
0xE5030000/0xE5031000/0xE5032000, IRQ206/207;
IFTU2 0xE5040000 has no shared window in this table and IRQ255.

Software plane array is linked 0x8100B298, stride 0x20C. For current IFTU0:

| Field | A VA | B VA | Meaning from actual instructions |
|---|---|---|---|
| Record | 4F2298 | 4F24A4 | +0 plane MMIO VA; +4 shared MMIO VA |
| +180 | 4F2418 | 4F2624 | Cached input descriptor, 54 bytes |
| +1E0 | 4F2478 | 4F2684 | SetOutputFormat fifth argument |
| +1E4 | 4F247C | 4F2688 | Native enabled field |
| +1E8 | 4F2480 | 4F268C | Pending old-bank token |
| +1EC | 4F2484 | 4F2690 | Cached optional flag replayed to hardware +180 |

**Software descriptor +180 and hardware plane register +180 are different
objects.** Do not treat either as a proven commit/flip register.

## Initializer and modes now established

Entry 0x81005D3C zeroes 0xA3C bytes (five 0x20C records) at 0x8100B298.
Instructions 0x81005D64/68 then store 1 at A+0x1EC;
0x81005E2E/36 store 1 at B+0x1EC (=array+0x3F8).
Thus the genuine default cached optional flag is 1, while +0x1E0, +0x1E4 and +0x1E8
start 0. The earlier isolated routine fixture also used 1, but this direct
initializer evidence removes dependence on that fixture seed.

SetOutputFormat 0x8100651C stores its fifth argument at software+0x1E0.
Enable 0x8100690C writes run+0x50=1, setup+0x58=0x108,
hardware+0x180=cached+0x1EC, both descriptor banks, shared selection 0 and
bus control 1. For output flag 0, A's shared+4 bits[2:1] are binary 01 and
B's bits[4:3] are binary 01, yielding mode 0xA when both are enabled.
Flag 1 selects binary 11 and also writes shared+0x14/+0x1C=0. This is exact native
arithmetic, not a recovered mode-name enum. Fresh live mode 0xA agrees with flag 0
arithmetic; direct software fields remain capture targets.

The primary hardware-tested [vita-libbaremetal IFTU source](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/iftu.c)
and [header](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/include/iftu.h)
corroborate the E502 windows, explicit shared+10/+18 selects, bank+200/+300,
input format10 and blanking. That code uses mode pairs00 and does not establish
the native01/11 meanings, plane status+40, or hardware+180 behavior.

## Deferred path and what the handler actually accepts

SetInputFramebuffer 0x810065E0 receives {R0=plane,R1=descriptor,R2=optionalflag,
R3=sync}. Nonnegative R2 replaces cached+1EC; negative R2 retains it. With
sync nonzero it reads current-bank bit1 from plane+4, writes the opposite bank,
and stores 0x80000000|oldindex at software+0x1E8. It makes no explicit shared select
or hardware+180 commit write on that branch. Sync0 uses a different immediate
path that writes relevant fields in both banks. **The first actual logo buffer
has not been submitted; it must not be presumed to use the deferred branch.**

Handler 0x8100599C/runtime 0x5AD99C reads plane+0x40 at 0x810059B8,
writes 0 at 0x810059BA, reads again at 0x810059BC, then writes cached+0x1EC
to hardware+0x180 at 0x810059BE. Neither status
read participates in its branch decisions. If software+1E8 is nonzero, it clears
that token and replays the cached descriptor into the old bank selected by the
token's low bit, without rereading current bank. It returns 0xFFFFFFFF.

Therefore any sampled +0x40 value, including 0, is accepted by these unchanged
handler instructions. W0ACK is evidenced; **no legitimate nonzero status-bit
encoding is recoverable here**. +0x180=1 is native, but whether it enables,
triggers or rearms an event is unresolved. The scratch probe invokes the handler
without a hardware event and stubs only its synchronization imports; that is
control-flow recovery, not proof of delivered IRQ or permission to synthesize it.

## Next capture and model boundary

Graphics has prepared full non-ACK snapshots: SetInput R0–R3/R1 descriptor;
plane+4/+40/+50/+58/+180; shared+0/+4/+10/+14/+18/+1C; all four complete256-byte
bank records; software+180 and +1E0/+1E4/+1E8/+1EC; GIC204/205/213 fields.
At handler entry R1 supplies the actual record. Capture a genuine submission's
branch, native result, descriptor validity and subsequent wait before choosing
any turnover behavior. Existing static model correctly remains explicit-select
only; an automatic mode would need coherent active-bank readback and frontend
selection, with event timing, status encoding and +180 semantics explicitly
bounded by further evidence. Do not inspect/patch guest software pending,
invoke its callback, invent a completion bit, force a logo predicate or supply
framebuffer pixels. No new boot run or shared build was performed for this review.
