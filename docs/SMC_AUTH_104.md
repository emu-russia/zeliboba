# Firmware 1.04 secure-module scheduler and authentication ABI

Research date: 2026-10-01. This contains captured native TrustZone startup
observations and prospective scheduler/authentication ABI guidance. Native
kprx secure-module authentication, entry and cleanup are now observed, as are
native authentication/decrypt of both bootconfig segments and native
ScePsp2BootConfig relocation, initialization and callback entry. Native os0
kernel modules now load and start; the guest white PlayStation logo is observed
in native macOS SDL. Full kernel startup remains unfinished. Consult
[STATUS.md](STATUS.md) and the latest boot traces for the reached state.

The NSKBL ABI addresses in the later sections were checked in the guest-unpacked,
ARM-filtered NSKBL image `build/goal-nskbl.bin`, based at `0x51000000`, size
`0x40000`, SHA-256
`8f23de3903b4b23d07b9940be44b902d3ca8fffd2a09976a5acac15262c789a7`.
This runtime image matters: an offline unpacked image that has not received
the ARM instruction filter can contain misleading MOVT immediates.

## Primary references and version boundaries

- [VitaSDK scheduler header](https://github.com/vitasdk/vita-headers/blob/master/include/psp2kern/kernel/sm_sched.h)
  and [SM communication header](https://github.com/vitasdk/vita-headers/blob/master/include/psp2kern/kernel/sm_comm.h):
  named APIs and later firmware contracts, mainly 3.60.
- [Reversed scheduler proxy](https://github.com/motoharu-gosuto/psvcmd56/blob/master/src/CMD56Reversed/SceSblSmSchedProxy.cpp)
  and [internal layouts](https://github.com/motoharu-gosuto/psvcmd56/blob/master/src/CMD56Reversed/SceSblSmSchedProxyInternalTypes.h):
  useful cross-checks for SMC arguments and event fields. The 1.04 slot consumer
  differs from the later shared-buffer descriptor mechanism.
- [xyzz F00D scheduler](https://github.com/xyzz/f00d/blob/master/smsched.c)
  and [its structures](https://github.com/xyzz/f00d/blob/master/smsched.h):
  hardware mailbox submission/completion behavior.
- [reF00D authentication implementation](https://github.com/dots-tb/reF00D/blob/master/src/ref00d_kprx_auth.c)
  and [metadata structures](https://github.com/dots-tb/reF00D/blob/master/src/ref00d_kprx_auth.h):
  working cryptographic algorithms and segment selection, rather than a
  complete native command dispatcher. The corresponding `10001` and `20001`
  services in [psvcmd56's KprxAuthService](https://github.com/motoharu-gosuto/psvcmd56/blob/master/src/CMD56Reversed/F00D/KprxAuthService.cpp)
  are return-minus-one stubs and cannot establish their crypto behavior.

The runtime 1.04 disassembly is the authority for the firmware-specific
addresses and command layouts below; later headers corroborate meanings.

## Native TrustZone startup observed on 2026-10-01

The rebuilt CLI, with all inherited `ZLB_*` variables removed and only diagnostic
logging enabled, now executes the native page allocator, module loader, and
module starts. Captures are `build/goal-tz-native-return-capture.log` and
`build/goal-native-smsched-handshake.log`; their corresponding `.py` scripts
preserve the setup and select the ARM core that actually hit each breakpoint.
These captures use the ordinary boot path and the absolute `build/goal-emmc.img`,
with the remaining development substitutions still enabled by default.

At load return `0x40021B50` and start return `0x40021B68`:

| Module | Genuine load UID | Start result |
|---|---:|---:|
| SceSysmem | `0x20077` | `0` |
| SceExcpmgr | `0x20089` | `0` |
| SceKernelIntrMgr | `0x2008F` | `0` |
| SceKernelBusError | `0x200A9` | `0` |
| SceSblSmsched | `0x200AF` | `0` in the later native IRQ capture below |

All four cores reach the real IntrMgr callback `0x003BEFDD` and BusError
callback `0x0051A069` through `0x40021B92`. IntrMgr's module object is VA
`0x71A90` / PA `0x40131A90`, with text at VA `0x003BE000` and data at
`0x00540000`. `MVBAR` remains `0x16140`, but the vector page has changed:
VA `0x16168` / PA `0x40000168` contains native SMC entry `0x003BE1C8`, and
the IRQ target at `0x1617C` contains `0x003BE008`. Therefore the old SKBL
dispatcher table pointer at `0x400B3024`, still zero, is no longer the active
SMC table. Native IntrMgr uses `[VA 0x5486C8] = 0x513000`; that table maps to
PA `0x40196000` and includes `0x101 -> 0x003BE759` and
`0x103 -> 0x003BE711`.

The first capture's blocker was Smsched start, at VA `0x0051E364` / PA `0x401C6364`,
linked address `0x81002364`. Its first mailbox read waits for status bit
`0x100` at `0xE0000000`. Inputs are `r0=0x00800000`, `r1=0x7C00`,
`r2=0xE0000000`, `lr=0x0051DB6B`. At that revision a full run remained at
checkpoint `0x88`; CMeP is halted at `0x40002`, status is zero, and its command
register retains `1`. This does not establish a successful CMeP firmware
return: the host hook currently classifies the low-window entry that way.

### Native initialization handshake

The filtered, relocated guest instructions at `0x0051E35C..0x0051E43C`
specify these branches. The independent
[xyzz implementation](https://github.com/xyzz/f00d/blob/master/smsched.c#L44-L104)
corroborates the status/address protocol.

| Condition or step | Actual guest behavior |
|---|---|
| Status bit `0x100` absent | Poll `0xE0000000` at `0x0051E364`; this was the first capture's wait. |
| `(status & 0x600) == 0x400` | Return `0x800F0427` at `0x0051E3EC`. |
| `(status & 0x600) == 0x200` | Read and acknowledge current status by writing it back, then take the common handshake. |
| `(status & 0x600) == 0x600` | Write `1`, then `0`, to `0xE0010000`; wait for zero there and bit 31 at `0xE0010004`. Submit `(image_address & ~3) | 1` to `0xE0000010`. Wait for status `1` or `2`; `2` returns the error above, while `1` is acknowledged before the common handshake. |
| Common handshake | Wait for the low 16 status bits to equal `0x101`, then write `0x101` to acknowledge them. |
| Shared-buffer publication | Translate the buffer at `shared_state + 8` through the native Sysmem API; write its PA masked with `~3` to `0xE0000010`, then write literal `1` separately at `0x0051E3C4`. Wait for the command register to clear. |
| Completion | Wait for low 16 status bits `0x102`, acknowledge by writing `0x102`, then return `0`. |

The two address/trigger writes require retaining the posted address bits until
the receiving endpoint acknowledges them. Status writes from ARM acknowledge
selected CMeP-produced bits; they are not replacement status values. In the
first capture, the boot ROM stand-in consumed `0x9/0x101/0x102/0x106`, wrote
command `1`, and injected CMeP state `9` before native Smsched ran. The later
handoff correction below removes that shutdown-state injection and broad
low-window completion assumption.

Smsched globals `[0x519090]` and `[0x519094]` both point to native shared
state VA `0x5501F0` / PA `0x401D01F0`. Its initialized fields are:

| Offset | Captured value and purpose |
|---|---|
| `+00` | Heap UID `0x200B1`. |
| `+04` | VA `0x30000200`, requested `0x80`-byte buffer, PA `0x40140200`. |
| `+08` | VA `0x300002C0`, requested `0x100`-byte shared command buffer, PA `0x401402C0`; this PA is the later handshake submission. |
| `+0C` | VA `0x30000400`, eight-byte auth-image range: `{0x40000500, 0x88F8}`. |
| `+10` | Physical address `0x40140400` of the range above, returned by native Sysmem. |

The image input `0x00800000` comes from the model's documented SceKblParam
development substitution: `src/machine/bootchain.cpp`, `build_kbl_param()`,
stores `board::kCmepSecureKernelBase` at parameter `+0x80` and the loaded
`secure_kernel.bin` size at `+0x84`. `load_cmep_secure_kernel()` loads that
`0x7C00`-byte plaintext image into the CMeP private address window at
`0x00800000`. This is a CMeP image address; it is not a newly allocated ARM
RAM frame. The ARM physical mirror of CMeP SRAM begins at `0x00040000`.
The captured native getter `0x000CE09C` resolves `[[0x000B2298] + 0x3C]`:
`[0xB2298]=0x4000`, `[0x403C]=0x47C0`. The guest-copied parameter at VA
`0x47C0` / PA `0x401047C0` contains `{0x00800000, 0x7C00}` at `+0x80`,
matching the original parameter copies at PA `0x4001FD00` and `0x1F000100`.

### Follow-up: native handshake completed, RVK request pending

The later ordinary run in `build/goal-native-cmep-full.log` preserves the live
CMeP `0x101` status for native ARM Smsched. ARM publishes PA `0x401402C0`
and trigger bit one; CMeP reads `0x401402C1`, acknowledges it, publishes
`0x102`, and ARM acknowledges that status. No scheduler completion or CMeP
shutdown state is fabricated. This advances the reached frontier to the
native revocation-list request, captured separately in
`build/goal-native-rvk-submission.log` using its `.py` script.

ARM VA `0x0051E4C8` waits for command `0x80A01` at `0xE0000010` to clear.
CMeP is asleep at `0x0080048A`, with state `3`, and retains the real bootstrap
shared address. The shared physical buffer at `0x401402C0` starts with
`{1, 0x40140440}`; the latter physical address contains the single image range
`{0x40008F00, 0x6C0}`. The native submission sequence is at
`0x0051E496..0x0051E4C4`, followed by command-clear polling and then a
nonzero low-half status wait at `0x0051E4D0`. ARM acknowledges that result;
if bit `0x8000` is present, it maps the low byte to `0x800F0300 | low_byte`.
The [xyzz RVK submission](https://github.com/xyzz/f00d/blob/master/smsched.c#L106-L128)
and [shared-buffer layout](https://github.com/xyzz/f00d/blob/master/smsched.h#L34-L38)
corroborate this request format and completion handling.

`build/goal-native-rvk-staged.bin` is the `0x6C0` bytes read from PA
`0x40008F00`, byte-for-byte equal to firmware `Out/SLB2/prog_rvk.srvk`.
SHA-256: `caba020624b014bc277b731286ec23f1d2437a4edb4a9e45adbbe7856d75d081`.
It is the real encrypted SRVK container: SCE version `3`, flags/key field
`0x40`, type `2`, header length `0x400`, data length `0x2C0`, and payload
header system version `0x10400000000`. It is not the decrypted `.seg01`
payload. At that capture no native RVK validation/decryption result had been
observed. The next behavior required actual mailbox interrupt delivery and
native CMeP command handling; the following capture observes that execution.

### Native IRQ servicing and scheduler startup, with RVK validation still failing

`build/goal-native-rvk-result-capture.log` and its `.py` script capture the
genuine RVK callback at MeP `0x008006BE`, buffer-registration completion at
`0x00800722`, and module-start return at ARM `0x40021B68`. The ordinary full
run `build/goal-native-irq-full.log` corroborates the device operations. These
captures use native mailbox source-8 IRQ delivery and real Bigmac DMA and
AES-256-CBC; no RVK status or scheduler success is injected.

The RVK callback returns **`0x800F0616`**, then native `0x008004DE` posts
mailbox status **`0x8016`**. Its observed operation sequence is:

| Function | Source and destination | Length | Result in this capture |
|---|---|---:|---|
| `0` | PA `0x40008F00` to CMeP `0x808FF0` | `0x30` | Real SCE header copied. |
| `0` | PA `0x40008F30` to CMeP `0x809020` | `0x3D0` | Remaining SCE header copied. |
| `0x238A` | In place at `0x809020` | `0x40` | Real AES-256-CBC produces metadata key/IV and zero padding. |
| `0x218A` | In place at `0x809060` | `0x390` | Old broad Bigmac development fallback reports OK but leaves ciphertext; native parsing returns the error above. |

The key at native `0x8074F0` and IV at `0x807510` already exist in the guest
secure kernel. An independent offline OpenSSL check decrypts the staged
SRVK with these values, then its metadata-provided AES-128 key and IV. The
metadata describes a plain `0x20`-byte header at image `+0x400` and encrypted
`0x2A0`-byte entries at `+0x420`. AES-128-CTR using metadata key indices
`8` and `9` reproduces the existing `prog_rvk.srvk.seg01` exactly. This is
offline evidence for available key material and future device operations,
**not an observed successful guest RVK result**. The exact `0x218A` model
implementation and known-answer regression were added after this capture;
its integrated boot result must be captured separately.

The optional RVK error does not stop native Smsched initialization. Its next
command `0x80901` supplies `{PA 0x40140200, length 0x80}`. Native handler
`0x008006D6` checks four-byte address alignment and the physical range via
`0x00801CE0`, stores the buffer PA at `0x8076A4` and length at `0x8076A0`,
then posts status `1`. Invalid input would post `0x8016`. The current range
is accepted by the native `0x40000000..0x402FFFFF` predicate. Its subsequent
revision-log notification is skipped because fuse/keyslot `0x50A`, byte 13
bit 0, reads zero at `0xE006214D`.

At ARM `0x40021B68`, Smsched UID `0x200AF` returns **`r0=0`**. The genuine
startup field `[VA 0x5190A0]` is **`1`**, and all sixteen service slots
`0x12D..0x13C` at VA `0x5130B4` / PA `0x401960B4` contain their native
targets. The ordinary run then reaches checkpoint `0xA1` and stops at
Monitor PC `0x000162A8`; that later stop is distinct from RVK initialization.

### AES-128 metadata decrypt reached; next missing operation is SHA-256

The subsequent integrated AES-128-CBC and Monitor-guard run is preserved in
`build/goal-native-rvk-cbc128-capture.log`, with its matching `.py` script.
Function `0x218A` now decrypts the native `0x390`-byte metadata in place at
`0x809060`. The RVK callback at `0x8006BE` returns **`0x800F0024`** rather
than the preceding metadata-parse error. The native buffer-registration result
is still `1`, and Smsched UID `0x200AF` still returns `r0=0` at `0x40021B68`.
Its service table and startup-complete field remain genuine and populated.

The capture reaches exact Bigmac function **`0x2093`** at MeP `0x806176`:
source `0x808FF0`, length `0x300`, digest destination `0x808C7C`. At this
revision the broad fallback still reports OK without producing the SHA-256
digest. The later ordinary trace `build/goal-native-cbc128-full.log` reaches
checkpoint `0xA9` and also requests this same function for a kprx SCE header:
source `0x81F000`, length `0x700`, digest destination `0x808B0C`. It then
remains in the native authentication wait with CMeP posting status `0x8024`.

The exact `0x2093` implementation was added after these captures. Native wrapper
`0x805EF2 -> 0x806110` uses `+00` as absolute source, `+04` as a full 32-byte
digest destination, and `+08` as message length; the retained `+10` key selector
is not a digest length. Register-level known-answer tests include padding across
two blocks and the RVK-sized 768-byte message. Its integrated guest result still
requires a new capture; successful RVK validation is not established here.

The independent RSA audit in `build/goal-rvk-rsa-audit.log` recovers the signature
digest using the native secure-kernel modulus and exponent. It equals SHA-256
of the captured decrypted header's first `0x300` bytes:
`0da6768c779b8eeedc7c53a0503f2c3f0f793699b0978ee77cc9a9fbe8635093`.
The corresponding binaries are `goal-rvk-rsa-decrypted-header.bin` and
`goal-rvk-rsa-recovered-digest.bin`. The guest's digest buffer still held stale
stack data under the SHA fallback. These artifacts support fixing SHA output;
they do not establish a need to bypass or alter native RSA verification.

### SHA passes the native RSA check; section transfer and HMAC are reached

`build/goal-native-rvk-sha-capture.log` records the integrated SHA/GIC/cache
revision. Its native RVK callback at `0x8006BE` returns **`0x800F0627`**,
so execution has passed the earlier RSA-digest failure and reached the first
section's integrity check. Native buffer registration and Smsched startup still
complete normally. The next exact operations observed in this capture are:

| Function | Source | Destination | Length | Behavior at capture time |
|---|---|---|---:|---|
| `0x2080` | PA `0x40009300`, image `+0x400` | CMeP `0x8093F0` | `0x20` | Broad fallback leaves the plain-section destination zero. |
| `0x20B3` | CMeP `0x8093F0` | CMeP `0x808CDC` | `0x20` | Broad fallback leaves the HMAC digest zero. |

The wrapper at `0x8062CE` retains key selector `0x216` at `+10` and points
`+14` at `0x808CB4`, context `+4`. It stages a complete 64-byte key block
from context `+0x90` at `0x808D40` into `+200..+23F`; the first 32 bytes
are the section's real metadata key and the remaining 32 are zero padding.
The single-shot final call sets the software context mode word to `2` and
emits `0x20B3`, with no incremental `0x0400/0800/0C00` modifier. The preserved
context and key are `goal-native-rvk-sha-hmac-context.bin` and
`goal-native-rvk-sha-hmac-key.bin`. Offline HMAC-SHA256 of the real plain
section with that staged key matches its metadata digest exactly.

Exact `0x2080` transfer and `0x20B3` HMAC implementations and register-level
regressions were added after this capture. The HMAC key window now retains all
64 staged bytes, while AES continues to select its first 16 or 32 bytes. A
successful integrated RVK callback and successful SM launch remain unobserved
in this capture. The subsequent revision reaches native CTR as recorded below.

The separate `build/goal-native-kprx-launch-sha-capture.log` identifies the next
launch failure without substituting its result. Native SCE parser return
`0x8026AE` and policy return `0x8026E8` both have `$0=0`. At `0x802706`,
the RVK-entry pointer in `$2` is `0xFFFFFFFF`; the native availability check
branches to failure, and launch return `0x80091C` has **`$0=0x800F0326`**.
The subsequent Bigmac `0x000C` zero-fill request is failure cleanup. The launch
has passed header authentication but cannot proceed while RVK initialization
has failed; it does not require a fabricated kprx-authentication result.

### Plain transfer and HMAC pass; native AES-CTR is reached

The integrated 506-test revision is captured in
`build/goal-native-rvk-transfer-hmac-capture.log`. Native `0x2080` transfers
the plain first section and `0x20B3` computes its matching HMAC. The guest then
issues **`0x21A1`** at MeP `0x805E62`: source PA `0x40009320`, destination
CMeP `0x809410`, length `0x2A0`. At capture time this exact CTR operation
still used the broad fallback. The next section's HMAC consequently fails;
the actual RVK callback at `0x8006BE` remains **`0x800F0627`**. Native buffer
registration returns `1`, and Smsched's actual start return at ARM
`0x40021B68`, UID `0x200AF`, remains `0`.

`build/goal-native-rvk-ctr-input-retry.log` captures the live CTR operands.
The first 16 bytes staged at `+200` equal metadata key index 8. Register
`+14=0x808D88` points at metadata IV index 9 with **all sixteen bytes
reversed**. The native wrapper reverses it at `0x8045D8` before issuing
hardware and reverses the hardware-updated counter at `0x804620` afterward.
Stream callers use the same context IV at `+0x150` as input and output;
counter writeback is required to continue successive chunks.

Independent OpenSSL AES-128-CTR using the captured key and canonical IV
decrypts the actual 672-byte source exactly to the extracted `prog_rvk.srvk`
second section, SHA-256
`388e8457398138c628330786b3ff019d82fbb0b582d4176675e570f9d8ec682e`.
It consumes 42 counter blocks. The exact `0x21A1` implementation now performs
this transform and writes the advanced 128-bit counter back in hardware
byte order, with NIST and counter-wrap register regressions. A live callback
after this implementation is recorded below; no successful result was inferred
from the offline decryption.

### Native RVK validation succeeds after CTR integration

`build/goal-native-rvk-ctr-result-capture.log` records the next integrated
revision on 2026-10-01. The genuine RVK callback at MeP `0x8006BE` now returns
**`0`** after both sections pass their native HMAC checks. The guest publishes
21 revocation entries: `[0x807698]=0x15`, `[0x80769C]=0x809410`, replacing the
earlier unavailable values `0xFFFFFFFF`. Registration callback `0x800722`
returns `1`, and Smsched UID `0x200AF` again returns `0` at its actual module
start return, ARM `0x40021B68`.

The post-command guest section at `0x809410` was saved as
`goal-native-rvk-ctr-decrypted-section.bin`; it matches the independent
extracted RVK section byte for byte, with the SHA-256 above. The saved
`goal-native-rvk-ctr-updated-hardware-iv.bin` equals the canonical metadata IV
advanced by exactly 42 blocks and reversed back to hardware order. This is
observed native RVK success, with real AES and HMAC outputs. Successful secure
module launch, os0-kernel loading, and the guest logo remain unobserved in
this startup capture.

### Native kprx payload contract after RVK availability

The payload path is independently checked and is now reached in the public
platform profile capture below. The authenticated kprx header saved in the
earlier parser capture describes one section at image `+0x1800`, length
`0x6FB4` (28,596 bytes), encryption type `3` (AES-128-CTR), key index `4`,
IV index `5`, and hash type `6` (HMAC-SHA256), hash index `0`. Compression
type is `1`. Independent OpenSSL decryption matches the extracted ELF's
`PT_LOAD` bytes at file `+0x1000`; their SHA-256 is
`4f0f27ec3bd12d76db3a9b1ce799147aa2069eebff35ff04fda275668cc9a0f3`.
Their HMAC also matches the authenticated metadata. Missing key material is
not indicated by these checks.

The native loader checks RVK availability at `0x802706`, then calls its
revocation policy at `0x802712` (return `0x802716`). Payload loading goes
through `0x8039EC` and returns at `0x8027EA`. Native CTR calls use the same
`0x21A1` wrapper. For the final four-byte fragment, `0x802C8C` explicitly
processes a full 16-byte scratch block, then the software consumes the valid
fragment; hardware CTR lengths remain aligned.

For payload streaming across multiple hardware hash calls, the native HMAC
wrapper at `0x806258..0x80627A` selects `0x24B3` for the first non-final call,
`0x2CB3` for continuation, and `0x28B3` for finalization after prior calls.
These retain the 64-byte staged key, `+14` context state, and digest output
at context `+0x2C`. The first and final forms are now observed; continuation
is recovered from the same wrapper and covered by the state protocol below.

The outer launch caller considers **return `1`**, tested at `0x80091E`, to
mean successful loading. It then sets EPC to `0x0080B000` at
`0x800966..0x80096E` and executes `RETI`; that matches the independent ELF
entry. Parser or policy return `0` alone does not establish a launched SM.

### RVK is available; kprx rejects an unavailable platform identity

The GIC reset correction restores ordinary boot to NSKBL checkpoint `0xA9`,
with 513 tests passing. `build/goal-native-kprx-launch-rvk-ok-capture.log`
records parser return `0` at `0x8026AE`, initial policy return `0` at
`0x8026E8`, and the valid RVK pointer/count at `0x802706`. Subsequent policy
return `0x802716` and actual launch return `0x80091C` both have
**`0x800F0B31`**. Payload loading and incremental HMAC have not been reached.

`build/goal-native-kprx-product-class-capture.log` locates the first rejection.
Native `0x805774` copies identity slot `0x509`, using the address arithmetic
in `0x80571A`: `0xE0058000 + (0x509 << 5) = 0xE0062120`. The 16-byte saved
identity is all zero. Product bytes `+4/+5` are interpreted big endian;
the class getter at `0x804B58` recognizes `0x0100` as test, `0x0101` as tool,
`0x0102` as DEX, and ordinary `0x0103..0x0111` products as CEX. The captured
predicate count and class at `0x804BA8` are both zero. Getter return
`0x804820` is zero; at `0x804840`, `$1=0`, `$3=0xFFFFFFFF`, and `$0=1`, so
the guest correctly branches to `0x804A50`, constructing `0x800F0B31`.

The existing USS-1001 retail model now supplies the **public PCH-1001 prefix**
`00 00 00 01 01 04 00 10` at slot `0x509`. This prefix is supported by the
[console ID tool's published hardware samples](https://github.com/Freakler/vita-ConsoleID).
It is modeled platform identity, **not a dumped console identity**. The
console-unique remainder is unavailable and remains zero; no console keys
are invented. Word, halfword, and byte reads expose the same data. Overrides
are retained verbatim, so unknown products remain subject to the guest's
policy. Reset restores the public prefix. The actual class and policy results
after this input change are recorded next; no successful launch is claimed.

### Public platform profile accepted; native payload reaches streamed HMAC

The public identity revision passes 514 tests. On 2026-10-01,
`build/goal-native-kprx-public-profile-class-capture.log` observes the copied
prefix at `0x804B6A`, the retail predicate returning `1` at `0x804B70`, and
predicate count/class both `1` at `0x804BA8`. The actual platform getter
returns `1` at `0x804820`. This is the guest classifying the modeled public
profile as CEX, without modifying its classifier or policy.

`build/goal-native-kprx-public-profile-launch-capture.log` then observes
parser and initial policy returns `0`, 21 available RVK entries, and the
post-RVK policy returning `0` at `0x802716`. Payload CTR decryption executes
before the following genuine streamed HMAC requests at `0x8062CE`:

| Function | Source | Length | Context state (`+14`) | Digest destination |
|---|---|---:|---|---|
| `0x24B3`, first non-final | `0x80B000` | 28,544 | `0x808B44` | `0x808B6C` |
| `0x28B3`, final | `0x808B8C` | 52 | `0x808B44` | `0x808B6C` |

The saved `goal-native-kprx-public-profile-stream-hmac-context-06.bin` and
`-07.bin` show one context at `0x808B40`, mode changing from `1` to `2`,
and buffered-byte count changing from `0` to `48`. The final source combines
those 48 buffered bytes and the last four payload bytes. Both saved staged
keys are the same 64-byte padded key at context `+0x90`. The first request
contains exactly 446 complete SHA-256 message blocks.

At this capture the two exact functions still use the broad fallback and
produce no digest. The guest therefore returns `0x800F0627` at payload
return `0x8027EA`; the outer launch returns `0x800F0324` at `0x80091C`.
Successful SM launch, os0 loading, and the logo remain unobserved.

Exact streamed HMAC implementations were added after this capture, preserving
the existing `0x20B3` one-shot form. First/continue requests compress complete
blocks; finalization computes the real inner and outer SHA-256 with the
accumulated length. Tests compare independent Python/OpenSSL answers for the
observed 28,544 + 52 split and interleaved contexts, including key mismatch,
invalid ranges, overwritten state, finalization retirement, and reset.
The isolated CMeP suite passes 50 tests. Integrated native verification follows.

**State-model limit:** the native wrapper reserves 40 opaque bytes at context
`+4`, but the hardware's serialization has not been recovered. The model
stores canonical SHA-256 chaining words and a little-endian byte count there,
retaining pointer/key-bound state on the host. It verifies the guest image
before continuation, replaces it on a new first request, and retires it on
finalization/reset. Copying the opaque image to a different guest pointer is
unsupported. This describes the model's state representation, not verified
hardware bytes or a fabricated integrity result.

### Native kprx authentication and ELF entry observed

The integrated streamed-HMAC/IFTU revision passes 520 tests. On 2026-10-01,
`build/goal-native-kprx-stream-hmac-result-launch-capture.log` observes both
exact incremental operations completing with real output. The saved final
context's digest at `+0x2C` equals an independent HMAC-SHA256 of the saved
guest plaintext and the authenticated header's expected digest at `+0x430`.
The entire 28,596-byte guest payload in
`goal-native-kprx-stream-hmac-result-payload.bin` equals the independently
extracted ELF `PT_LOAD` bytes. Its SHA-256 is the value recorded above.

The genuine payload routine returns **`0` at `0x8027EA`**, then the outer
loader returns **`1` at `0x80091C`**. The next breakpoint actually executes
at **MeP `0x80B000`**, the module's ELF entry, with `$0=0x401402CC`,
`$epc=0x80B000`, and `$psw=0x113`. The saved entry bytes match the decrypted
payload. This establishes native authenticated kprx loading and entry,
without replacing guest policy, integrity comparisons, or return values.

The ordinary run `build/goal-native-stream-iftu-full.log` later waits with
the module at MeP `0x80B050`, halted by a genuine sleep instruction, while
ARM0 is at `0x51016ABE` and NSKBL checkpoint remains `0xA9`. No subsequent
Bigmac operation appears in that run. Native os0 authentication/loading and
a guest logo remain unobserved; the module's request/event handoff is the
next boundary to inspect.

### First native os0 authentication/decrypt capture (533-test revision)

The integrated MeP SWI revision reaches the real module registration syscall:
at `0x80E8C6`, `$r4=4`, `$r1=9`, and `$r2=0x80E55E`. Software exception
entry is `0x800014`, with EPC `0x80E8C8`, PSW `0x112`, and EXC `0x15`.
The native syscall returns `1` to `0x80E8C8`, stores/enables the IRQ 9
callback, and keeps the module in its worker loop at `0x80E84A`. Captures
in `build/goal-native-kprx-swi-native-return-launch-capture.log` distinguish
the pending exception at the debugger's pre-delivery boundary from the
actual native return after the exception handler has run.

The next genuine validator, `0x80E59E..0x80E5D8`, reads four bytes at
`0xE0062260..0xE0062263` as a little-endian DRAM byte size, then checks
the posted command's end against `0x40000000 + size`, including 33-bit
carry. Previously zero capacity rejected both the first request and its
cleanup sentinel with transport status `5`; the service result stayed
`0xFFFFFFFF`. The board model now publishes its existing `0x20000000`
capacity in platform slot `0x513`, using the same `kermit::kScuSize`
constant as DRAM backing and KBL parameters. This is an explicitly modeled
public board input, not a dumped console identity or configuration. Raw
register overrides are retained; reset restores the modeled capacity.

The resulting 533-test revision is captured, with mutation flags unset,
in `build/goal-native-os0-auth-crypto-capture.log`. At the real private
callback return `0x80E68E`, request result `[0x812FC0 + 8]` is **zero for
`0x10001`, `0x20001`, and `0x30001`**. Incidental `$r0` at that point may
be the unlock syscall result and is not the command's service result.
The first command authenticates the genuine 4 KiB header of
`Out/fs/os0/psp2bootconfig.skprx`; the next two select/decrypt its first
segment. ARM independently receives transport status `1` and service result
`0` for all three requests. The later stop sentinel also reaches native
worker exit and the kernel's `0x10000` exit notification.

The captured plaintext at `0x5113C000` is `0x2DD` (733) bytes. Exact native
CTR executes 720 bytes, then a 16-byte tail scratch operation. Streamed
HMAC issues `0x24B3` for 704 bytes and `0x28B3` for the final 29 bytes,
with context `0x81FE40`, `+0x14` register pointing at `0x81FE44`, and
digest output `0x81FE6C`. Its final digest is
`58e35397b92d04a16e02938b822284f4b01060b56e791022690e582f8675750f`:
it equals independent Python HMAC-SHA256 of all 733 saved bytes and the
expected digest at authenticated header `0x811FC0 + 0x720`.

`build/goal-native-os0-auth-crypto-oracle.py` independently decrypts the
original SELF section at file `0x10A0`, length `0x2DD`, using OpenSSL
AES-128-CTR and the authenticated metadata key/IV at `+0x760/+0x770`.
It reproduces the guest bytes exactly, SHA-256
`3f2043fbfda85346d7c9c638dd30cf081e521a3c31742a0eb030c879e95921bd`.
Python zlib then yields `0x658` (1,624) bytes, exactly the first `PT_LOAD`
of `Out/fs_dec/os0/psp2bootconfig.elf`, SHA-256
`7d92a892f9c72a9e15fff70b4ffcfc174984f03f802e710e87d861b8468a93c2`.
At this historical capture the guest's subsequent decompressor had not yet
completed successfully. This capture establishes crypto and input validity;
the later native loading and cleanup observations follow below.

### Native bootconfig loading and CMeP cleanup (541-test revision, 2026-10-01)

The Thumb word-access alignment correction allows the unchanged guest inflater
to produce the expected `0x658`-byte first ELF segment. The genuine bootconfig
relocation returns zero at `0x5101A3E2`, with module UID `0x20075` and first
segment VA `0xCC000` backed at PA `0x40373000`. Its initialization callback
at `0xCC0B4` returns zero at `0xCC0BE`, printing
`ScePsp2BootConfig Starting... OK`. Its second callback at `0xCC0C0` also
executes natively on all four ARM cores. Evidence is in
`build/goal-integrated-bootconfig-relocation-capture.log` and the ARM captures
`build/goal-arm-native-bootconfig-list-results.log` and
`build/goal-arm-native-zero-entry-capture.log`.

The two dependency-list imports return `0x803FF007` at `0xCC0018` and
`0xCC002E`, leaving their UID arrays at `0xCD000` and `0xCD038` zero.
Initialization ignores these errors. The primary callback subsequently calls
`0x51010710`, which branches through the null callback in sysroot `+0x40`
at `0x5101071E`, reaching PC zero with LR `0x51010721`. The resulting ARM
restart/checkpoint `0xA4` is a distinct remaining boundary; these captures
do not establish the list-load error's cause or further kernel entry.

`build/goal-native-cmep-fill-cleanup-v2-capture.log` independently captures
the native MeP lifecycle with mutation flags unset. At `0x80E68E`, the private
command result is zero for the sequence `10001`, `20001`, `30001`, `20001`,
`30001`: one header authentication and both selected bootconfig segments.
The module handles the stop sentinel, unregisters its IRQ and takes terminal
SWI 1 into `0x800C30`; the native kernel sends the `0x10000` exit notification
before calling its module wipe at `0x80056E`.

The genuine wipe wrapper `0x8055A0` writes zero to Bigmac `+0x34`, then
submits source zero, destination `0x80A000`, byte length `0x16000`, flags zero
and function `0x000C`. The old emulator's small-opcode AES decoder incorrectly
resolved source zero to low SRAM `0x40000`; after the correct SRAM alias was
installed, that AES write corrupted the secure kernel at `0x800000`. The
scoped native zero-fill handler now clears all `90,112` bytes of the requested
module arena without changing the captured kernel vectors. Nonzero fill
pattern serialization is unverified and reports an error. The first-loader
nonzero-source `0x000C` form retains its explicitly unverified legacy emulator
interpretation; no general hardware meaning is inferred from that encoding.

At the secure-kernel handoff, low CMeP SRAM `0x40000..0x60000` and private
SRAM `0x800000..0x820000` share backing. The register that controls this board
remap remains unknown; this boundary is modeled explicitly, and both cold
and warm reset restore separate boot SRAM before clearing memory. No copied
helper bytes or synthetic helper returns are used. The live cleanup executes
the actual uncached helper `0x400CE`: CFG is `8` before and after its mask
clears `0x402`, then it returns to `0x80020E` with `$0=0`. The genuine cache
initializer reads CCFG zero at `0x8001C2` and branches to `0x8001F8` without
calling the nonzero-cache-size helper at `0x400B0`. The complete `31,744`-byte
kernel snapshots at `0x800C3E` and `0x800C42` are identical.

Native `0x8001A8` then sets EPC `0x800100` and executes RETI. After the real
kernel reinitialization it publishes `0x102`, retains its persistent RVK table
(`count=0x15`, pointer `0x809410`) and shared PA `0x401402C0`, sets state `3`,
and reaches genuine sleep at `0x80048A`. There is no MeP undefined instruction
in the corresponding ordinary run `build/goal-native-fill-full.log`. In this
541-test revision, repeated secure-module launches had not yet been observed
because the ARM dependency-list/callback boundary prevented further requests.
That revision reports **541 tests, zero failures**; no guest logo is observed.

### Native sysmem/stdio authentication and repeated launches (545-test revision)

The corrected ARM single-word MemU accesses preserve the actual bytes at
unaligned addresses. The unchanged guest path normalizer can now copy `/kd/`
from its unaligned source, resolving the earlier dependency-list failure.
The independent FAT16 audit `build/goal-independent-os0-fat-check.py` confirms
both clone os0 partitions contain the supplied `PUP_dec/os0.bin` verbatim,
with valid short/LFN entries and complete sysmem/stdio file chains. No image
rebuild or path substitution is involved.

Fresh capture `build/goal-native-memu-os0-crypto-v2-capture.log`, with all inherited
mutation flags removed and a separate APFS image clone, observes sysmem on the
second genuine kprx entry and stdio on the sixteenth. Before the latter, fifteen
native module cleanups have returned through the real cache helper to
`0x800C42`, followed by real restart and re-entry at `0x800972` / `0x80B000`.
Their `10001` private results at `0x80E68E` are zero. The original clear header
prefixes match the supplied SELF files, and the full decrypted `0x1000`-byte
headers match an independent OpenSSL oracle:

| Module | Decrypted header SHA-256 |
|---|---|
| `kd/sysmem.skprx` | `fc3d53e9d260e2466dfa34003204f213fe8b7f1c63ac73c3460000be56efa43d` |
| `kd/stdio.skprx` | `1432967822bff099cfaee97f4010d39bbe0a484d3df5d7ae0962cfde13b2bc7e` |

Every selected section's `20001` and `30001` private result is also zero.
All eight actual compressed plaintext outputs match OpenSSL AES-128-CTR
byte for byte, and the actual native HMAC outputs match both the metadata's
expected digest and independent Python HMAC-SHA256. Independently inflating
these saved guest outputs yields the supplied decoded ELF segments exactly:

| Module | Segment | Native compressed bytes | Decoded ELF bytes |
|---|---:|---:|---:|
| sysmem | 0 | `0x24018` | `0x393A0` |
| sysmem | 1 | `0x49` | `0x6C` |
| sysmem | 2 | `0x314D` | `0x9450` |
| sysmem | 3 | `0x1C` | `0x18` |
| stdio | 0 | `0x4C5` | `0x8E0` |
| stdio | 1 | `0x44` | `0xD4` |
| stdio | 2 | `0x174` | `0x384` |
| stdio | 3 | `0x70` | `0xD8` |

Segments 2 and 3 have ELF program type `0x60000000` and contain relocations.
Exact input/output hashes, expected HMAC digests and comparison binaries are
in `build/goal-next-os0-crypto-reference.json`; actual private results and
output ranges are in `build/goal-native-memu-os0-crypto-v2-results.json`.
The latter capture ends immediately after stdio's final successful section,
before its cleanup. After secure-kernel handoff, every reached Bigmac function
in this capture is handled by the exact native DMA, CBC, CTR, SHA, HMAC or
verified zero-fill implementation; no broad report-success fallback or crypto
error appears. Earlier boot-stage development substitutions remain explicitly
logged, so this does not claim an unsubstituted ROM boot.

The corresponding ordinary run `build/goal-native-memu-full.log` records
`10,753` eMMC reads, zero writes and `1,081` Bigmac operations. After bootconfig,
native startup prints `OK` for SceSysmem, SceExcpmgr, SceKernelIntrMgr,
SceKernelBusError, SceSystimer, SceSblACMgr and SceKernelThreadMgr. CMeP returns
to genuine sleep at `0x80048A`. All ARM cores then stop on decoder rejection
of NEON instruction `0xF2201110` at `0x004A0008` (reported PC `0x004A0010`),
with checkpoint `0xA9`; this is the new reached CPU boundary. The integrated
suite reports **545 tests, zero failures**. No guest PlayStation logo is observed.

### Native ARM progress after the NEON correction (548-test revision)

The subsequent ordinary run `build/goal-native-neon-full.log` starts all 14
core modules and Stdio. The unchanged ThreadMgr initializer now executes its
legal `VORR D1,D0,D0`; no further crypto change was needed. The same `10,753`
sector reads, zero writes, `1,081` Bigmac operations and native CMeP sleep
`0x80048A` remain. ARM0 waits in Lowio at `0x005AB1DC`, polling unmapped
`0xE050001C` as `0xFFFFFFFF`; secondary cores wait at `0x51000D0C`.
The integrated suite passes **548 tests, zero failures**. DSI/IFTU guest
startup and the PlayStation logo remain unobserved.

### Native threads and Lowio after TPIDR/I2C fixes (560-test revision)

`build/goal-native-thread-context-full.log` starts all 14 core modules, Stdio
and Lowio. Architectural TPIDR registers retain the guest's context pointers;
native threads now execute, with ARM1/3 in idle and ARM0 in Syscon initialization.
The I2C0/1 controllers each receive exact reset7 and report idle; unsupported
slave transactions cannot create completion. ARM2 executes Secure work at
`0x0054BB8C..94`, polling unmapped `0xE8200024` as `0xFFFFFFFF`.
The image remains clean: `10,753` sector reads and zero writes. Native IRQ
wakeups increase to252, while artificial WFE ticks, counter patches and forced
cluster wakes remain zero. The integrated suite passes **560 tests, zero
failures**. No guest DSI timing enable, framebuffer scanout or logo is claimed.

### Syscon padding and the first EMC command (572-test revision)

`build/goal-native-emc-syscon-full.log` runs one million ordinary slices with
the corrected Thumb NEON VMOV/VST1 padding sequence. All 14 core modules and
Stdio, Lowio, Syscon, OLED, Display, SblSsSmComm start successfully. SceEmcTop
completes the first native initialization command `0x000E0000`; all six
commands and SMC117/callback completion are not yet observed. Independent
128 MiB CDRAM backing is installed; no native CDRAM data test is claimed.

Syscon/OLED read zero through actual GetSystemTimeLow `0x004B7D5D` from LT5
`0xE20B6000`, whose placeholder drops configuration writes. The separate
Secure EMC delay uses missing WT7 `0xE20BE000` and IRQ135; native ARM2 waits
on its per-core lock at `0x000CD024`. These are hardware timer boundaries,
not an auth-service or CPU failure. No forced wake or lock/result injection
is used. The suite passes **572 tests / zero failures**; eMMC still records
10,753 reads, zero writes, dirty=no. Native DSI timing and the logo producer
remain unreached. Fresh macOS SDL3 screenshot:
`build/goal-native-emc-syscon-macos-ui.png` (real Syscon instruction bytes).

### Native timer completion (579-test revision)

LT5 now supplies the actual ThreadMgr raw microsecond count. WT7 drives physical
IRQ135 into the genuine Secure handler, which releases the per-core delay wait.
The fresh `goal-native-timer-frontier-timers.log/.json` captures native SMC117
return at `005A8098` with R0=0 and all six EMC commands completed. The ordinary
`goal-native-timers-full.log` independently records six completions. No auth
result, delay lock or SMC return is supplied by a host hook.

The complete suite passes **579 tests / zero failures**. eMMC still records
10,753 reads, zero writes, dirty=no. Syscon reaches SPI0/GPIO3 and OLED reaches
A1/length5/SPI2 TX drain. Native interrupt context divergence and GPIO/SPI
transport are the next boundary; guest logo presentation remains unreached.
The earlier 572-test section above is retained as historical evidence.

### Observed native service registration

Static native startup order is: initial handshake (`0x0051DB66`), optional
revocation-list setup (`0x0051DB7A`, command `0x80A01`), `0x80`-byte buffer
setup (`0x0051DB8E`, command `0x80901`), IRQ 200 registration and enable,
then IRQs 201 through 203. It finally registers the sixteen SMC/function
pairs at VA `0x519000`, in ascending order `0x12D..0x13C`, using the loop
`0x0051DC5E..0x0051DC74`. Selected relocated targets are `0x12D=0x51CCCD`,
`0x133=0x51E0B9`, `0x136=0x51E159`, `0x137=0x51E171`,
`0x138=0x51E295`, `0x139=0x51E345`, and `0x13A=0x51C9A5`.
Only after these registrations does `0x0051DC78` set `[0x5190A0]=1`.

At the two earlier startup waits the service slots were zero. After the
genuine `0x80901` completion their native registration is observed, including
`0x12D=0x51CCCD`, `0x133=0x51E0B9`, `0x136=0x51E159`,
`0x137=0x51E171`, `0x138=0x51E295`, and `0x139=0x51E345`.
Scheduler startup completion and native RVK validation are now observed.
Native SM authentication, os0-kernel loading, and a guest logo remain
unobserved in these startup captures.

## SMC boundary and shared slots

ARM stub `0x510169E0` loads the SMC API number from `[sp]` into `r12`, executes
`smc #0`, then returns through `lr`.

| SMC | Registers at the secure boundary | Effect |
|---|---|---|
| `0x12D` | `r0=priority`, `r1=physical range-list address`, `r2=count`, `r3=bank 0 slot index` | Launch request; return secure handle through slot `+0x14`. |
| `0x133` | `r0=handle`, `r1=mailbox 1..3`, `r2=set mask`, `r3=0` | Set ARM-to-Cry mailbox bits. |
| `0x136` | Same, with clear mask | Clear Cry-to-ARM mailbox bits. |
| `0x137` | `r0=handle`, `r1=mailbox 1..3`, `r2=bank 0 slot index`, `r3=0` | Return Cry-to-ARM mailbox value in slot word zero. |
| `0x138` | `r0=handle`, `r1=interrupt index 0..3`, `r2=r3=0` | Enable notification delivery. |
| `0x139` | Same | Disable notification delivery. |
| `0x13A` | `r0=handle`, `r1=interrupt index`, `r2=notification token`, `r3=0` | Acknowledge delivered notification. |

These are secure-boundary handles. The public request ID is a different value
stored by the guest proxy; notifications use the public ID. A callback address
is stored locally by the guest and is not passed to SMC `0x138`.

The shared-base getter `0x510106F0` returns `[[0x5113B61C] + 0x80]`.
Initialization at `0x51016738` stores base in `[0x5113B74C]` and base plus
`0x400` in `[0x5113B748]`. Banks 0 and 1 each contain eight `0x80`-byte slots.
These are CPU/context slots, not an inferred producer/consumer ring. Bank 0
allocation at `0x510167CC` uses the CPU index, adding four in the alternate
context detected by `0x51002CD4`; bank 1 consumption uses `MPIDR & 0xF`.
Resolve guest virtual addresses before accessing these globals in a host model.

Invoke at `0x51016180` fills a `0x48`-byte bank 0 payload, calls SMC at
`0x510162CC`, then consumes the returned secure handle at `0x510162EA`:

| Offset | Field |
|---|---|
| `+00`, `+04`, `+08`, `+0C` | Four startup words |
| `+10` | Public request ID |
| `+14` | Secure handle output |
| `+18` | Context self type |
| `+1C` | Media/path ID |
| `+20` | Caller program authority ID, 64 bits |
| `+28` | Caller capability, 32 bytes |

Startup at `0x51016BCC` supplies self type `0x1002`, `r1=0`, and `r2=0`:
this launch must work without an image range list. It stores its public ID
at `0x5113B794`, registers callback `0x510169ED` for interrupt index zero,
and waits on `0x5113BAC0`. The callback at `0x510169EC` accepts scheduler
status `2` or `0xA`, returning startup success through that wait word; other
statuses supply the callback's fifth argument as the result.

## Notifications and completion

Initialization at `0x510160F4` registers interrupt ID **4**, named
`SceSblSmSchedProxySGI`, with handler `0x51015F89`; the registration call is
at `0x5101614A`. A model that substitutes for the secure scheduler must
deliver the proxy notification via this SGI, rather than handing a raw F00D
mailbox IRQ to NSKBL.

Dispatcher `0x51015F88` obtains the current CPU's bank 1 slot through
`0x510168FC` / `0x5101690E`. The event layout is:

| Offset | Field |
|---|---|
| `+00` | Interrupt index |
| `+04` | Public request ID |
| `+08` | Delivery flags: `1` event flag, `2` extended callback, `4` simple callback |
| `+0C` | Event bits |
| `+10` | Notification acknowledgement token |
| `+14` | Result |
| `+18` | Scheduler status |

The extended callback at `0x51015FFA` receives
`(public_id, interrupt_index, registered_arg, status, result)`, with result
on the stack. The dispatcher acknowledges through SMC `0x13A` at
`0x51016012`. The guest stores pending notifications when a handler has not
yet been registered; `0x51016574` can invoke a stored callback before
enabling secure delivery. An implementation needs notification ordering and
acknowledgement state, not just a success return.

Generic command submitter `0x51016A5C` translates the command buffer to a
physical address, registers callback `0x51016B29` for mailbox 1, sets the
wait word to minus one, then calls `0x133` with `command_PA | 1` at
`0x51016AB4`. Callback `0x51016B28` reads mailbox 1 through `0x137` and clears
the returned bits through `0x136`. It requires bit zero to be set and accepts
mailbox value **1** as transport success. The command's service result is
read separately from `command + 8` at `0x51016B00`.

The hardware source uses completion values `1` for success, `3` for invalid
command, `5` for invalid secure buffer, and `9` for I/O error; the error bits
correspond to `0x800F0002`, `0x800F0001`, and `0x800F0005`. A command can have
successful transport and a negative service result. Returning zero from the
submission SMC without publishing completion leaves the guest spinning.

## Actual 1.04 authentication commands

Command buffers use total size at `+00`, function ID at `+04`, service result
at `+08`, and payload at `+40`. Builders zero the buffer and initialize the
result to `0xFFFFFFFF`. Physical range entries are address/length pairs.

| Command | Builder | Size | Payload and result |
|---|---|---:|---|
| `0x10001` | `0x51016E9C` | `0x190` | Context `0x130` at `+40..+16F`; header range count at `+170`, range-list PA at `+174`. Returns called auth info at `+D8`. |
| `0x20001` | `0x51017028` | `0x50` | Segment index at `+40`; returns compression mode at `+44`. |
| `0x30001` | `0x510170D8` | `0x50` | Input count/list PA at `+40/+44`; output count/list PA at `+48/+4C`. Decrypts selected segment data. |

For `10001`, success copies the returned `0x90` bytes from command `+D8`
to the input context `+98` at `0x51016FF8`. Context layout is reserved word
`+0`, self type `+4`, caller auth info `+8`, called auth info `+98`, media/path
ID `+128`, reserved word `+12C`. Auth info contains authority ID at `+0`,
capability at `+10`, attributes at `+30`, and shared secrets at `+50`;
see [VitaSDK types](https://github.com/vitasdk/vita-headers/blob/master/include/psp2kern/types.h).

The meaning of `20001 +44` is corroborated twice: reF00D's
`ref00d_setup_segment` selects by metadata `section_idx`, initializes key/IV,
and returns `section_compression`; the actual guest consumer `0x51017250`
reads that mode from `0x5113B800`, treating `1` as copy and `2` as
decompression through `0x510172CC` / `0x51024E18`. Thus `30001` should return
decrypted compressed bytes and let the guest decompress them.

## Crypto implementation guidance and remaining fidelity gaps

reF00D implements header authentication by decrypting the `0x40`-byte
metadata key block with AES-256-CBC, then the metadata body with its recovered
AES-128 key/IV. It verifies SHA-256/RSA-2048, extracts authority/capabilities/
attributes, selects each segment's key and IV, and maintains AES-CTR state
across block submissions. Its built-in `REF00D_KERNEL=0x10007` keys are
custom keys, not established firmware keys.

Existing local building blocks:

- `src/loader/self.cpp:1106`, `sce_decrypt_metadata_in_place`: decrypts a
  supplied header with the existing key tables. Plaintext metadata starts
  at `metadata_offset + 0x30`, followed by the `0x40`-byte key block,
  `0x20`-byte metadata info, `0x30`-byte segment records, and 16-byte vault
  entries. This helper does not verify the RSA signature.
- `src/loader/keys.cpp:985`, `aes_ctr_crypt`: increments a big-endian 128-bit
  counter, but starts from the provided IV on every invocation. Preserve
  stream position, including any partial block, across `30001` submissions.
- `src/loader/keys.h:76`: SHA-256, HMAC-SHA256, and RSA public primitives are
  available. Their availability does not establish a complete native
  authorization-policy implementation.

A prospective HLE should consume the guest's physical range lists, retain
per-request authentication/segment state, write actual decrypted data and
auth outputs, and publish transport and service results separately. Exact
native authorization-policy checks, segment-integrity enforcement, lifecycle
edge cases, and notification timing still need validation against the real
1.04 secure implementation. None of these research findings demonstrates
that execution has reached the corresponding SMC or that os0 has booted.
