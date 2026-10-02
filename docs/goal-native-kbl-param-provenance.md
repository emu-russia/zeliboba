# 1.04 cold boot parameter provenance and Display predicates

Read-only recovery against the supplied firmware and the integrated 611-test
binary. No firmware bytes, predicates, guest state, or production source were
changed by this audit.

## Actual reached failure

`goal-arm-native-syscon-oled-checksum.log` lines 43361–43392 identify the object:

* Sysmem relocated global `VA 0x000CE328` contains sysroot `VA 0x4000`.
* Sysroot `+8` is size `0x4BC`; `+0x3C` points to `VA 0x47C0`.
* Param `VA 0x47C0 -> PA 0x403047C0`; bytes `+0x30..33` are `00 00 00 FF`,
  word `+0x6C` is `5`, `+0xC0` is `0x60`, and `+0xC4` is `0xFF14`.
* Genuine Sysmem `IsUpdateMode` returns `1` at native Display return site
  `0x005B4FDE`. Display takes its exit at `0x005B4FE0` (lines 43781–43846).
  Later predicates were not called; their outcomes are static predictions only.

The unchanged 1.04 getter at linked `0x8101CA7C` loads param byte `+0x30`,
subtracts `0xFF`, and returns `1` if unequal. Thus the zero supplied by the board
builder has the exact architectural/native consequence seen in the capture.
`IsExternalBootMode` at `0x8101C9BC` returns param `+0x6C & 1`; the marker's
`|= 1` would also report external boot if this later predicate is reached.

The exact four getter listings are in
`goal-native-display-predicate-static.txt`. These are native 1.04 functions,
not definitions imported from a later sysroot layout.

## Genuine second-loader authors of boot flags

Raw supplied `SLB2_dec/second_loader.bin`, loaded at `0x40000`, SHA256
`cf01888929c3d7c49efba0fb1ae2950857acf8dd86f3960472191ff165821b1b`:

| Param field | Actual native producer | Cold model result |
| --- | --- | --- |
| `+0x30` | `0x41D1C..41D32`: destination `0x1F000130`, NVS offset `0x4A0`, length `1`, call `0x443EA` | `FF` |
| `+0x31` | `0x41CB4..41CB8` calls `0x42174`, cached NVS `0x480+1` | `FF` |
| `+0x32` | `0x41CAC..41CB0` calls `0x4215A`, reads cache `0x55328+4`; `0x42010..42022` explicitly sets that byte to zero | `00` |
| `+0x33` | `0x41CBC..41CC0` calls `0x42188`, cached NVS `0x480+3` | `FF` |

`0x41FE8..4200C` first fills both eight-byte caches (`0x55328`, `0x55330`)
with `FF`. `0x42010..4202C` clears the special `+4` byte and reads NVS
`0x480`, length `8`, into `0x55330`. NVS reader `0x443EA` builds the actual
command `0x1082`, offset/length payload; its small-read success path copies the
response data to the requested destination at `0x444D8..444E0`.

The existing `ernie::NvsStore` initializes its whole store to `FF`, and the
existing wire test `ernie_nvs_and_scratchpad_are_offset_length_stores` verifies
this for NVS `0x480`. Therefore the internally consistent unprovisioned cold
board profile is **`FF FF 00 FF` / little-endian `0xFF00FFFF`**, derived from
the current NVS contents and the native cold initializer. Four `FF` bytes would
misrepresent the native `+0x32` author. A valid user-written update/safe byte
must remain observable rather than be overwritten later.

Precise listings: `goal-native-kbl-param-builder-dis.txt`,
`goal-native-kbl-param-flags-dis.txt`, `goal-native-kbl-param-nvs-dis.txt`;
the full read-only second-loader listing supplies cache initialization and
callers in `goal-native-secondloader-boot-param-full-dis.txt`.

## Boot type author and unsupported marker

Builder `0x41BB2..41BB6` initializes param `+0x6C` to zero. It sets bit 0
only after `0x41BBA` calls `0x41A4E` and receives a nonzero result. In this
supplied second loader, `0x41A4E` is exactly `mov $0,0; ret`, so this path
cannot set the external-boot bit. The independent bit-16 producer `0x41A52`
is also `mov $0,0; ret` in the supplied bytes.

The product bit 2 is independently read from `0x42074` / global `0x55764`;
`0x420E2..4213E` can set it from the authenticated configuration result. Keeping
the existing board profile's product value `4` is a bounded existing model
policy; this audit does not claim every real cold console has identical bits.

The historical `bootcfg_marker` block in `bootchain.cpp` at the 611 baseline
writes both `+0x33=FF` and `+0x6C|=1` into the already copied heap record at
`PA 0x403047C0`. It would overwrite a meaningful NVS safe-mode request and
contaminates the same parameter later read by Sysmem's external-boot getter.

Its justification cites a native store at `0x5101587C`. Actual Thumb bytes
show `0x5101587A..7C` are one `BLX 0x51014528` (core-ID helper) inside the
four-core barrier routine. This is not a parameter store. The nearby store of
`1` at `0x5101586C` writes an argument-supplied barrier object, followed by
halfword counts of `4`; it is unrelated to boot type.

## Native file dispatcher remains available with bit 0 clear

`0x51018F6C` dispatches service `0x10005` for path open. Its checks at
`0x51018F9A..18FAA` read boot-type bit 0 and bit 2. If either is zero, the
branch to `0x51019116` is **not a failure exit**:

1. It calls `0x51017304`, which returns `[GetSysroot()+0x2E] & 1`.
2. Zero rejoins `0x51018FAE`, selecting a single attempt.
3. Nonzero selects up to `0x186A0` attempts unless global `0x5113BAD0` is
   already set, then rejoins `0x51018FB2`.
4. Both paths call the genuine opener `0x51001548` at `0x51018FC6` and use
   its actual result. The retry logic recognizes `0x8008000A` / `0x8009000A`.

No forced flag, returned success, directory lookup, or firmware patch is needed
to retain this route. Dynamic boot after removing the marker must still check
actual opener results; static control flow does not promise all dependencies
will succeed. The saved listing is
`goal-native-nskbl-service-boot-gate-dis.txt`.

## Object copies and kernel provenance

| Stage | Source and destination | Native proof |
| --- | --- | --- |
| second loader | native param base `0x1F000100` in SPAD | builder `0x41B50..41B54`, initial clear `0x41B38..41B46` |
| board handoff | modeled SPAD param mirrored to ARM `PA 0x100` | `build_kbl_param` then `mirror_cmep_scratch_to_arm` |
| secure KBL | `PA 0x100 -> 0x400B2DC8`, size `0x100`; context `0x400B2B30+0x2C` points to this copy | `0x400211E8..40021210`, literal `0x400213F4=0x400B2E08` minus `0x40` |
| secure-to-NS handoff | `0x400B2DC8 -> PA 0x40300100`, size `0x100` | `0x400215A6..400215BA` |
| NSKBL temp | `0x40300100 -> 0x51184E98`, size `0x100`; context `0x51184C00+0x2C` points here | `0x51000900..51000926` |
| NSKBL sysroot | context `+0x2C -> allocated param`; store pointer in sysroot `+0x3C` | `0x51010C1C..51010C38` |
| kernel Sysmem | `0x8101BFA8..8101BFBE` selects sysroot `VA 0x4000` and stores it in linked global `0x8103A328` | native checks sysroot size `0x4BC`; matching size follows direct object path |
| actual Display | same sysroot `VA 0x4000`, size `0x4BC`, param `VA 0x47C0` | fresh 611 capture above |

Exact ARM copy and Sysmem setter listings are
`goal-native-kbl-param-arm-copy-dis.txt` and
`goal-native-sysmem-sysroot-global-dis.txt`.

The old claim that NSKBL intentionally zeroes this parameter at
`0x51011D5C..51011DA8` is unsupported: these addresses are inside the genuine
**A32 memcpy** `0x51011C80`, including `STRD` stores of loaded source words.
The parameter is copied, not reconstructed as an empty boot mode object.

## Bounded implementation proposal and source limitations

Correct the existing early board parameter constructor to read current NVS
bytes `0x4A0`, `0x481`, `0x483` into `+0x30`, `+0x31`, `+0x33`, with native
cold initializer `+0x32=0`. Use an inspection-only NVS API: calling public
`dispatch_command` would publish a reply and increment command counters.
Retire the entire historical late marker block; retain the existing product
profile `+0x6C=4`. Keep native service dispatch and Display predicates intact.

[VitaSDK SceKblParam header](https://github.com/vitasdk/vita-headers/blob/master/include/psp2kern/kernel/kbl/kbl.h)
corroborates field layout and names; it is an independently recovered primary
project source, not a Sony specification. Cold sentinels and 1.04 offsets above
come from the supplied native firmware and existing modeled NVS, not a later
3.60 sysroot header. The Ernie wiki was attempted via browsing but returned
an access-denied page and is not used as new evidence here.

This report does not claim successful logo presentation. The post-change run
must first show unchanged native authentication/loading, then actual Display
predicate returns, producer execution, framebuffer setup, and guest scanout.
