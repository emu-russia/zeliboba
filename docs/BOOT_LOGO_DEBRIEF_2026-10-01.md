# Root-cause debrief: macOS bring-up to the native PlayStation logo

The gap was between reaching a loader checkpoint and executing a correct cold
boot all the way through firmware authentication, os0 module loading, native
threads, device initialization, logo production and display publication. Each
stage exposed requirements that the previous stage had never exercised.

We started with **438 passing internal tests**, a working host debugger, and
NSKBL checkpoint **A9**, while ARM0 polled `5101EDE8` and the kernel had not
loaded. We finished with **617 passing tests**, genuine os0 execution and an
actual macOS SDL screenshot of the guest logo (image omitted from source delivery).
The net test-case count grew by 179; this is not a count of 179 independent root
causes. Full kernel startup, LiveArea and guest SGX rendering remain unfinished.

The tracked boot objective ran from **13:50:32 to 19:28:16 PDT on 2026-10-01**:
5 hours 37 minutes 44 seconds, matching the reported 5h37m. Initial Mac setup
artifacts precede that interval, around 13:31–13:39. Times below are saved
milestone/build timestamps, not a measured allocation of labor: agents worked
in parallel, and some reports were written after the underlying experiment.

**Starting state and the first useful distinction.** The screenshots established
that the Windows build could initialize the debugger and that its input layout
was available. Porting host paths, file access and SDL startup made that setup
usable on macOS. AppleClang/CMake and Homebrew SDL3 then built the CLI, tools,
tests and UI; Metal/vsync and 48 kHz stereo audio initialized. VS Code was optional.
Portable filesystem paths, POSIX large-file access, and platform guards for
Windows resources/DLL copying were part of this work. Startup outside the source
directory was checked. Initial host startup log (local capture omitted from source delivery).

That host success did not establish firmware boot progress. The
first Mac boot log (local capture omitted from source delivery) shows automatic mailbox replies,
23 counter patches and 8 forced cluster wakeups, with CMeP halted at 40002.
A9 meant that NSKBL had reached a checkpoint; it did not prove that the native
secure scheduler, authentication services or os0 dependencies were operating.
The original 438-test baseline is retained in [STATUS.md](STATUS.md).

**Chronology of the gap.** This is the causal order visible in the saved runs;
some peripheral and graphics recovery happened in parallel with CPU repairs.

| Saved milestone, PDT | What blocked further progress | Correction and newly reachable behavior |
|---|---|---|
| Initial Mac setup,13:31–13:39 | Windows-specific host assumptions and unresolved input/image confidence | Native CLI/SDL builds, source-relative paths and verified firmware reconstruction; the initial A9 wait remains |
|13:52–14:49;440→468 tests | SDIF event acknowledgement/probe timeouts, incorrect Thumb flags/carry, ARM permissions/security state, Monitor/vector handling and old loader-memory substitutions | Native card probes, validation, allocation, relocation and TrustZone module starts execute; the secure-service path becomes the next blocker |
|14:58–16:28;471→533 tests | CMeP handshake/interrupt behavior, missing crypto operations, GIC/FIQ delivery, stale pending state, identity policy and DRAM-capacity input | Actual scheduler/RVK/kprx service code authenticates and decrypts the first os0 module |
|16:40–17:01;539→545 tests | Correct compressed data and filenames are misread by unaligned ARM/Thumb word accesses | Genuine DEFLATE output matches the ELF; `/kd/sysmem.skprx` survives normalization and both 14-entry module lists load (28 modules total) |
|17:07–18:04;548→579 tests | ThreadMgr/Syscon NEON gaps, discarded thread-ID registers, absent I2C reset/idle windows, EMC/CDRAM and timer behavior | Native threads/objects and Lowio initializers work; all six EMC commands complete and real timer IRQ135 releases SMC117 |
|18:24–18:38;599 tests | Missing SPI response-ready interrupt, incorrect Thumb exception return and incomplete OLED stream behavior; first GPIO revision regresses early boot | Native receive/ACK/stop and exact 18-byte OLED A1 transfer work; early second-loader reply is restored; Display enables DSI |
|18:48;611 tests | Syscon response checksum uses unsupported/misdecoded SIMD operations | Genuine VLD1/VMOVL/VADD/VPADD/high-D VMOV checksum code completes |
|19:05;611 tests | Wrong boot-mode inputs skip the logo; corrected cold inputs expose a legacy stack hook | NVS-derived cold fields and preservation of allocated stacks let the genuine logo producer run |
|19:19;617 tests | Deferred buffer is populated but never becomes active; native IFTU completion interrupt is missing | DSI frame boundary switches active bank, raises IRQ204, and the actual handler ACKs/rearms/replays |
|19:22 screenshot;19:28 sign-off | Need proof that actual host presentation consumes those guest pixels | Independent SDL capture visibly shows the white PS logo; final logs, image state and documentation are checked |

**The initial card wait required SDIF protocol corrections.** The register
block merged writes into stored status before SDIF applied write-one-to-clear
(W1C). Acknowledging one event could therefore erase another pending event.
Byte/halfword/word acknowledgement had to preserve the unwritten lanes and
events. The guest's no-response CMD8/CMD5 probes also needed a real modeled
command timeout, error-summary/signal behavior and reset cancellation to proceed
through card detection. Neither valid FAT bytes nor first-loader SUCCESS proved
this later controller path. W1C tests (local capture omitted from source delivery),
post-timeout native run (local capture omitted from source delivery).

**CPU and loader defects were the largest foundation gap.** Early native
validator behavior depended on Thumb IT flag suppression, shift carry and
replicated-immediate carry. Page access depended on correct AP/AFE decoding,
separate Secure/Nonsecure MMU state, and actual Monitor exception/SMC semantics.
The model also needed to preserve firmware-built vectors and keep authentication
input buffers outside scratch space that IdStorage reused.

The historical claim that the SCE validator accepted only ELF was caused by
incorrect IT semantics: its unchanged SCE sequence correctly returns 2 after
the repair. That sequence was not reached by the original ordinary baseline,
so this correction alone does not explain its card wait. Once MMU security
banking worked, a vector-restoration shim overwrote native Monitor code with
ELF bytes, yielding an apparent undefined instruction on ELF magic 464C457F.
Preserving the guest's vectors removed that host-induced failure.
Vector-overwrite capture (local capture omitted from source delivery).

Old hooks then became a separate obstacle. Some reseeded a live allocator heap
or replaced guest-populated MemBlock/class state; they were disabled or narrowed
once the actual native path worked. A Thumb32 `LDR.W PC` had been treated as a
preload, losing real relocation control flow. Fixing these defects let the guest
allocate, relocate and start Sysmem, Excpmgr and IntrMgr. Merely increasing run
slices or forcing a later stage could not establish those behaviors.
[Current foundation findings](STATUS.md), loader-PC regression log (local capture omitted from source delivery).

Two later CPU defects explain why storage and decompression looked suspect.
The authenticated 733-byte bootconfig stream was valid and independently
decompressed to the expected ELF. The native Thumb inflater loaded a word from
an unaligned address, but the emulator rounded the address down. It saw the
wrong DEFLATE block type and returned80560100/800F0516. Correct MemU addressing
let the unchanged decoder produce the exact1,624-byte segment.
[Before/after proof](../build/goal-native-deflate-cpu-fix.md).

The path normalizer exposed the same architectural error in A32. At 51013744,
`LDR r3,[r1],#4` with source 7FC53 should read `/kd/`. Legacy aligned-and-rotated
behavior produced `/os0`; the suffix was also corrupted. FAT lookup failed,
the module lists stayed empty, a later sysroot callback remained zero, and the
cores eventually waited after a restart. The null callback and WFE were
downstream symptoms. A common ARMv7 single-word helper fixed A32, Thumb16 and
Thumb32 without changing filenames or lookup results.
[Path-copy proof](../build/goal-native-path-cpu-fix.md),
[dependency failure and rendezvous](../build/goal-arm-native-rendezvous-evidence.md).

**Authentication required working services and interrupt transport.** The
initial model's automatic replies did not replace a functioning CMeP scheduler.
Native 101/102 handshaking, mailbox endpoint ACK, MeP IRQ/RETI/SWI and wakeup
behavior were needed before real service processing could continue. Bigmac
fallback paths had reported success without producing plaintext or digests,
so genuine metadata/RSA/HMAC comparisons failed. Real operation outputs let
the native RVK callback return zero; no new signature bypass was needed there.
Bigmac
needed the reached DMA, AES-CBC/CTR, SHA-256 and streamed-HMAC operations with
correct state and buffer handling. GIC mapping, security groups, target cores,
FIQ delivery and level deassertion had to match the guest's actual setup.

An explicitly modeled public retail identity prefix removed a platform-policy
rejection; the board's DRAM-capacity slot was made consistent with 512 MiB. These
are documented board inputs, not recovered console-specific identity. Native
header/segment/decrypt service returns then became zero, and their resulting
bytes matched the supplied ELF. That established where authentication ended
and the ARM decoder failure began.
[Native service result proof](../build/goal-native-kprx-service-decompress-evidence.md),
[remaining substitutions](STATUS.md).

**Module start success concealed unfinished workers.** After os0 dependencies
loaded, ThreadMgr and Syscon exercised additional NEON forms. More subtly,
ThreadMgr wrote real current-thread pointers to TPIDRPRW, but the CPU ignored
those CP15 registers and returned zero. Mutex/event creation therefore rejected
the context correctly for an architecturally wrong input. Implementing per-core,
Secure/Nonsecure TPIDR registers preserved the guest's own pointers and produced
real object UIDs. Both I2C windows also needed the exact reached reset7/idle
subset; an absent device read FFFFFFFF kept Lowio polling forever.
[Thread-context proof](../build/goal-native-tpidr-context-evidence.md),
[I2C poll proof](../build/goal-native-lowio-i2c-evidence.md).

Queued CDRAM initialization needed EMC command completion and Secure delay
completion. Independently, the physical board map needed CDRAM RAM backing;
no captured guest CPU/DMA access to that data aperture established it as the
cause of this wait. LT5 supplies Nonsecure ThreadMgr/Syscon/OLED elapsed time
and deadlines. The SecureDelay/SMC117 path required the WT7 comparison timer
and its interrupt. Physical IRQ135 reached the real handler; the guest
released its delay and SMC117 returned zero. Driver wrappers often returned
zero after queuing work, so we traced individual callbacks and completions
instead of treating `Starting... OK` as the end of initialization.
[Timer and EMC completion](../build/goal-native-timers-integrated-evidence.md).

The next waits crossed SPI, GPIO, interrupt return and OLED. A queued SPI reply
needed a response-ready GPIO pulse, mask/pending behavior and native IRQ248/sub4
handling. RFE incorrectly cleared a valid Thumb halfword address bit, returning
into the previous instruction and corrupting the OLED stack/transfer. An
unchanged-helper fixture reproduced that mechanism; all 378 instruction-boundary
IRQ trials passed after correction. User/System SP/LR aliasing was also corrected.
The live path subsequently retained the stack and exact 18-byte A1 transfer.
[Exception-return proof](../build/goal-native-oled-rfe-cpu-fix.md).

Finally, Syscon's real checksum used a sequence of SIMD operations that exposed
several successive missing/misdecoded forms. High-D scalar extraction could
compute the right vector sum yet read the wrong register. The repaired native
block was checked against an independent scalar checksum oracle across 355
executions before ordinary boot was retried.
[Checksum proof](../build/goal-native-syscon-checksum-cpu-fix.md).

**The last three gaps were boot inputs, stack ownership and publication.**
Once the device work ran, Display legitimately skipped the logo: the modeled
parameter byte+30 was zero, so native `IsUpdateMode` returned1. A historical
late marker also set the external-boot bit without a valid native author.
Following the actual second-loader producers back to Ernie NVS established the
consistent cold fields **FF FF 00 FF**, with the retained product profile+6C=4.
Correcting the early constructor and removing the late marker let all four
native Display predicates return zero.
[Parameter provenance](../build/goal-native-kbl-param-provenance.md).

That cold path exposed another inherited workaround: the stage hook added
`core * 0x1000` to every stack top, including genuinely allocated 16 KiB stacks.
The shifted tops were outside their allocations. ARM2 eventually reached a
POP with a zero saved-register/return frame. The exact backing-remap writer
was not established; the allocation/hook mismatch was established. Restricting
the fallback to the original shared top 4000 preserved native allocated tops
and restored progression.
[Independent stack review](../build/goal-native-cold-stack-handoff-review.md).

The genuine gzip producer then created all **2,088,960 bytes** of the 960×544
logo at PA 1C000000, exactly matching the firmware asset. SetFrameBuf returned
zero. Yet the old IFTU model displayed active bank0, which was blank, while
native deferred SetInput had filled bank1. The RAM-export PNG proved production;
the preserved blank SDL capture (image omitted from source delivery)
proved that presentation was still missing.

The final bounded IFTU implementation consumes DSI frame boundaries, changes
hardware current before physical IRQ204, and lets the unchanged native Lowio
handler acknowledge, rearm and replay into the old bank. It never reads the
guest's pending token as a hardware trigger or supplies logo pixels. The
actual SDL screenshot then showed those guest bytes.
[Native publication trace](../build/goal-native-iftu-arm-graphics-evidence.md),
[controller model and limits](../build/goal-native-iftu-boundary-implementation.md).

**What the eMMC and Graphics investigations resolved.** Independent ZIP byte
comparison, both FAT copies, filesystem checks and 992 file comparisons validated
the reconstructed image's firmware content. A deliberately corrupted clone
failed the checker. Preserving original SLB2 padding improved source fidelity,
but the A9 wait was unchanged before/after that correction. No damaged firmware
region explained the observed path lookup failure. The original suspected
Windows image was unavailable, so its condition remains unknown; console-specific
IdStorage/RPMB/layout is not validated by the firmware comparisons.
[eMMC procedure and scope](EMMC.md).

All 15 Graphics PDFs/800 pages were inventoried and indexed, with focused review
of relevant sections. They explained API pitch, formats, ownership, refresh,
texture layouts and shader patching. Actual 1.04 Lowio/Display instructions
supplied the IFTU/DSI register contract. The logo used CPU gzip and scanout;
the PDFs did not provide binary GPU command, MMU or USSE encodings, and a
complete shader backend was unnecessary for this milestone.
[Graphics findings](GRAPHICS_REFERENCES.md).

**Why this took 5h37m.** The main cost was serial discovery: later firmware
instructions and peripheral states became observable only after earlier
authentication, allocation and execution worked. We repeatedly captured the
next exact failure, reproduced it in a smaller unchanged-code fixture, corrected
the responsible layer, integrated and checked ordinary cold boot. Even the final
independent SDL boot/capture took **108.528 seconds**; repeated interpreter boots
and more instrumented runs added substantial turnaround time. There is no reliable
per-category timer, so assigning percentages to reverse engineering, compilation,
testing or agent coordination would be speculation.

Some cost was avoidable. Old hooks could make a checkpoint look healthier than
the underlying native path. Existing tests missed native access forms and some
asserted obsolete ARM behavior. During this session, the first GPIO revision
passed 599 tests but accidentally withheld an early second-loader reply until
handoff; ordinary boot caught the regression and a native early QueryIntr/ACK
case was added. That failed full run took 103.263 seconds.
[Regression record](../build/goal-native-rfe-gpio-spi-regression.md).

Probe setup also mattered: `run` bypasses boot hooks that `runm` executes; raw
`save` reads physical addresses, whereas ARM code/data observations often need
virtual translation; linked firmware addresses differ from relocated runtime
bases; and watchpoints reduce the ARM scheduling budget, making some timing
comparisons invalid. Those mistakes and negative captures were retained and
excluded from positive claims. We improved ARM instruction inspection and used
bounded no-watch captures for the final evidence. These lessons can shorten
the next iteration; they cannot remove the dependency order of a cold boot.

**What to carry forward.** Use a verified immutable input set and a fresh image
clone for each boot. Record success per native operation: allocation, authentication,
load, relocation, callback, device completion, pixel production and presentation.
Keep every remaining hook's ownership/activation condition explicit. Use small
native fixtures with independent oracles, then an ordinary boot smoke covering
the earlier stages as well as the new device. Freeze one integrated build for
parallel probes, and keep failed captures under distinct names.

The achieved result has four independent checks: guest-produced pixels match
the source asset; current-bank readback and scanout agree; real IRQ204 dispatch
and guest ACK/rearm/replay occur; actual SDL shows the white logo. The final
suite is 617/0 and captures report 10,753 reads/zero writes. The original input
image and previously running UI were preserved.
[Integrated result](../build/goal-native-iftu-arm-integrated-evidence.md).
No runtime undefined-instruction halt or panic was logged. ARM0's accumulated
CP15 fault counter remains 8; passing tests and a visible logo do not establish
zero diagnostic counters.

Exact IFTU physical status/rearm semantics, full blending/fade, complete kernel
startup, LiveArea and SGX remain open. One frame per full-word arm, raw status 0
with an internal completion latch, undriven-high SPI2 input and remaining
missing-ROM/security/identity substitutions are documented model choices.
The visible logo establishes the reached guest CPU/display path within those
limits.
