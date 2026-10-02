# Native I2C object errors trace to missing architectural thread-ID registers — 548

Read-only ordinary CLI capture with inherited `ZLB_*` variables removed, absolute `goal-arm-lowio-clone.img`, and `--no-rebuild`. No context pointer, UID, return value, thread state, interrupt status, or peripheral completion was supplied. The captured binary is the integrated548 build.

## Exact public definitions and genuine 1.04 bindings

`80027101` is `SCE_KERNEL_ERROR_ILLEGAL_CONTEXT` in the primary [VitaSDK error header](https://github.com/vitasdk/vita-headers/blob/master/include/psp2/kernel/error.h). The published [ThreadMgr NID table](https://github.com/vitasdk/vita-headers/blob/master/db/360/SceKernelThreadMgr.yml) identifies `FBAA026E` as `ksceKernelCreateMutex` and `4336BAA4` as `ksceKernelCreateEventFlag`. That table is labelled firmware3.60; the actual1.04 ELF import/export tables independently contain the identical NIDs and library `SceThreadmgrForDriver/E2C40624`, so those names are used only for the matched bindings.

| I2C call | Genuine Lowio import stub | NID | Genuine ThreadMgr provider | Result slot |
| --- | --- | --- | --- | --- |
| Create mutex | linked `81004440`, native `005AC440` | `FBAA026E` | linked `8100C395`, native Thumb `004AC395` | I2C state+1C |
| Create event flag | linked `81004370`, native `005AC370` | `4336BAA4` | linked `8100DB09`, native Thumb `004ADB09` | I2C state+18 |

All four actual calls pass R0=`SceI2c0` or `SceI2c1`, R1(attr)=0, R2(initial count/pattern)=0, R3(options)=0. Their failure happens before argument-dependent object allocation. It is not a fabricated error inferred from the idle-poll result.

## Native current-thread pointers are already produced

The genuine per-core ThreadMgr extension executes `MCR p15,0,R9,c13,c0,4` at `004A1DE8` and reaches `004A1DEC` normally. This encoding writes TPIDRPRW, not CONTEXTIDR. Raw bytes are `0D EE 90 9F`. The source R9/SB and readable guest object are:

| Core | Native pointer in R9/SB | Actual writer PC / next PC |
| --- | --- | --- |
| ARM3 | `0006B828` | `004A1DE8 -> 004A1DEC` |
| ARM0 | `0006BA28` | `004A1DE8 -> 004A1DEC` |
| ARM1 | `0006BC28` | `004A1DE8 -> 004A1DEC` |
| ARM2 | `0006BE28` | `004A1DE8 -> 004A1DEC` |

The capture saves all before/after registers, `vpa` and guest-RAM reads. The writers execute in SYS/Thumb with SCR=5 (Nonsecure), MMU enabled. Thus there is actual native per-core context initialization before Lowio module_start; no pre-scheduler absence should be assumed.

## Actual zero read and early return

Mutex provider `004AC394` first reads TPIDRPRW into R8 at `004AC3A0`, TPIDRURO into R4 at `004AC3A4`, temporarily changes TPIDRURO, then reads TPIDRPRW into R2 at `004AC3AE`. At the live `004AC3BA` CBNZ gate, R2=0. It therefore restores TPIDRURO and immediately returns the preloaded illegal-context error without allocating a mutex. Actual caller returns:

- I2c0 at `005AB0F6`: R0=`80027101`, LR=`005AB0F7`.
- I2c1 at `005AB10A`: R0=`80027101`, LR=`005AB10B`.

Event-flag provider `004ADB08` reads TPIDRPRW into R1 at `004ADB14`, TPIDRURO at `004ADB18`, and TPIDRPRW into R2 at `004ADB22`. Live `004ADB26` again has R2=0. It constructs the same error, restores TPIDRURO and returns immediately:

- I2c0 at `005AB11E`: R0=`80027101`, LR=`005AB11F`.
- I2c1 at `005AB132`: R0=`80027101`, LR=`005AB133`.

The final native controller records contain these actual returns in their mutex/event slots. Their separate first-controller hardware idle poll then reaches the already-confirmed unmapped `E050001C` wait. Fixing the I2C idle/reset model alone would preserve the object errors.

At capture548, `ArmCore::cp15_read/write` case13 implements only opc2=1 CONTEXTIDR; opc2=2/3/4 read zero and their writes are ignored. Therefore the genuine nonzero current-thread write is lost by the CPU model. The observed native illegal-context rejection is correct for the value the model provides, but the provided value is architecturally wrong.

## Required architectural contract

Cached primary ARM ARM DDI0406C.d, **B4.1.150/151/152**, physical PDF pages1714/1715 (`build/research/zeliboba-review-thumb-it/arm-ddi0406cd.pdf`), specifies:

| Register | CP15 encoding | Access | Security Extensions |
| --- | --- | --- | --- |
| TPIDRPRW | opc1=0,c13,c0,opc2=4 | PL1+ read/write; inaccessible atPL0 | Secure/Nonsecure banked |
| TPIDRURO | opc1=0,c13,c0,opc2=3 | PL0 read-only; PL1+ read/write | Secure/Nonsecure banked |
| TPIDRURW | opc1=0,c13,c0,opc2=2 | All privilege levels read/write | Secure/Nonsecure banked |

These are 32-bit software-owned storage with UNKNOWN reset values; processor hardware never updates them. A deterministic reset choice for the emulator must be described as model policy. Genuine guest writes, per-core ownership, security banking and illegal-access behavior must be retained; no boot-specific pointer seed or fake successful UID is required. The CPU owner has the exact capture and primary sections and owns any implementation/tests.

## Frozen artifacts and validation boundary

- `goal-arm-native-tpidr-context.py/.log/.json`: all four actual producer writes, all four provider gates/errors and final native I2C poll.
- `goal-native-lowio-i2c-evidence.md`, `goal-arm-native-lowio-start.log`, `goal-arm-native-lowio-map.log`: independent native module sequence and both physical I2C mappings.
- A separate initial `goal-arm-native-tpidr-context-step-bp-attempt.log/.json` stopped again on the same writer breakpoint when `step` was issued, so it is **not** used as an executed-write proof. The corrected capture uses writer and next-PC breakpoints and records actual execution.

This evidence identifies the lost-register cause. It does not claim that object allocation, workqueue execution, all scheduler state or display initialization succeeds after the correction; those require the next ordinary integrated run. The prepared graphics capture script remains unrun pending the parent's new-binary signal.
