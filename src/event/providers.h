// zeliboba - the event provider manifests.
//
// Every provider is declared once, with a stable GUID, a Graph Explorer area and
// a table of its events.  Instrumentation refers to an event by
// `EventProvider::X` + `ev::x::kY`, and the level/opcode/task/keyword defaults
// come from this table, exactly like an ETW manifest: a call site cannot forget
// to declare what it emits, and the debugger can list the vocabulary.
#pragma once

#include "event/event.h"

namespace zlb {
namespace ev {

// ---------------------------------------------------------------------------
// Machine - board lifecycle
// ---------------------------------------------------------------------------
namespace machine {
constexpr u16 kBuild = 1;        ///< build=ok
constexpr u16 kReset = 2;        ///< cold=1
constexpr u16 kPartsFitted = 3;  ///< emmc=, syscon=, first_loader=
constexpr u16 kSaveState = 4;    ///< path=, bytes=
constexpr u16 kLoadState = 5;    ///< path=, bytes=
}  // namespace machine

// ---------------------------------------------------------------------------
// Boot - the firmware boot chain
// ---------------------------------------------------------------------------
namespace boot {
constexpr u16 kStageBegin = 1;   ///< stage=, detail=      (activity Start)
constexpr u16 kStageEnd = 2;     ///< stage=, result=      (activity Stop)
constexpr u16 kMilestone = 3;    ///< text=
constexpr u16 kHandshake = 4;    ///< status=, side=
constexpr u16 kArmReleased = 5;  ///< entry=, cores=
constexpr u16 kComplete = 6;     ///< stage=
constexpr u16 kFailure = 7;      ///< stage=, reason=
constexpr u16 kStage = 8;        ///< stage=, index=  (plain transition marker)
}  // namespace boot

// ---------------------------------------------------------------------------
// CPU - core lifecycle and faults
// ---------------------------------------------------------------------------
namespace cpu {
constexpr u16 kCoreReset = 1;     ///< core=, entry=
constexpr u16 kCoreHalt = 2;      ///< core=, reason=
constexpr u16 kCoreWake = 3;      ///< core=, reason=
constexpr u16 kException = 4;     ///< core=, vector=, cause=
constexpr u16 kUndefined = 5;     ///< core=, pc=, reason=
constexpr u16 kSlice = 6;         ///< core=, cycles=   (Verbose, opt-in)
}  // namespace cpu

// ---------------------------------------------------------------------------
// Interrupt - raise / deliver / EOI
// ---------------------------------------------------------------------------
namespace interrupt {
constexpr u16 kRaise = 1;      ///< line=, source=
constexpr u16 kDeliver = 2;    ///< core=, line=, vector=
constexpr u16 kEoi = 3;        ///< line=, source=
constexpr u16 kEnable = 4;     ///< line=, enabled=
constexpr u16 kPending = 5;    ///< line=, source=
constexpr u16 kSpurious = 6;   ///< line=
}  // namespace interrupt

// ---------------------------------------------------------------------------
// Timer
// ---------------------------------------------------------------------------
namespace timer {
constexpr u16 kArm = 1;      ///< timer=, ticks=
constexpr u16 kExpire = 2;   ///< timer=, count=
constexpr u16 kStop = 3;     ///< timer=
constexpr u16 kReset = 4;    ///< timer=
}  // namespace timer

// ---------------------------------------------------------------------------
// Memory
// ---------------------------------------------------------------------------
namespace memory {
constexpr u16 kMapRegion = 1;   ///< name=, base=, size=
constexpr u16 kUnmapped = 2;    ///< kind=, address=, pc=
constexpr u16 kAllocate = 3;    ///< size=, address=, tag=
constexpr u16 kAperture = 4;    ///< name=, base=, size=
constexpr u16 kWrite = 5;       ///< address=, value=, size=
}  // namespace memory

// ---------------------------------------------------------------------------
// eMMC card
// ---------------------------------------------------------------------------
namespace emmc {
constexpr u16 kTransferBegin = 1;  ///< dir=, lba=, blocks=
constexpr u16 kTransferEnd = 2;    ///< bytes=, result=
constexpr u16 kCommand = 3;        ///< cmd=, arg=
constexpr u16 kError = 4;          ///< op=, detail=
constexpr u16 kAttach = 5;         ///< path=, capacity=
constexpr u16 kReset = 6;          ///< reason=
}  // namespace emmc

// ---------------------------------------------------------------------------
// SDIF host controller
// ---------------------------------------------------------------------------
namespace sdif {
constexpr u16 kCommand = 1;        ///< cmd=, arg=
constexpr u16 kTransferBegin = 2;  ///< lba=, blocks=, dir=
constexpr u16 kTransferEnd = 3;    ///< bytes=, result=
constexpr u16 kError = 4;          ///< op=, detail=
constexpr u16 kClock = 5;          ///< hz=
}  // namespace sdif

// ---------------------------------------------------------------------------
// DMA
// ---------------------------------------------------------------------------
namespace dma {
constexpr u16 kTransferBegin = 1;  ///< channel=, src=, dst=, bytes=
constexpr u16 kTransferEnd = 2;    ///< channel=, bytes=, result=
constexpr u16 kError = 3;          ///< channel=, detail=
}  // namespace dma

// ---------------------------------------------------------------------------
// Display / DSI / IFTU
// ---------------------------------------------------------------------------
namespace display {
constexpr u16 kModeSet = 1;        ///< width=, height=, format=
constexpr u16 kFrameBuffer = 2;    ///< address=, stride=, format=
constexpr u16 kFramePresent = 3;   ///< frames=, address=
constexpr u16 kVblank = 4;         ///< count=
constexpr u16 kTurnoverBegin = 5;  ///< bank=
constexpr u16 kTurnoverEnd = 6;    ///< bank=
constexpr u16 kDsiCommand = 7;     ///< cmd=, length=
constexpr u16 kReset = 8;          ///< reason=
}  // namespace display

// ---------------------------------------------------------------------------
// GPU
// ---------------------------------------------------------------------------
namespace gpu {
constexpr u16 kSubmit = 1;  ///< cmd=, bytes=
constexpr u16 kFrame = 2;   ///< frames=
}  // namespace gpu

// ---------------------------------------------------------------------------
// Syscon / Ernie
// ---------------------------------------------------------------------------
namespace syscon {
constexpr u16 kScCommand = 1;   ///< cmd=, payload=
constexpr u16 kScReply = 2;     ///< cmd=, result=
constexpr u16 kPowerState = 3;  ///< from=, to=
constexpr u16 kReset = 4;       ///< reason=
constexpr u16 kRtc = 5;         ///< value=
constexpr u16 kEmmcHost = 6;    ///< cmd=
constexpr u16 kError = 7;       ///< op=, detail=
}  // namespace syscon

// ---------------------------------------------------------------------------
// CMeP (F00D) - services and hardware engines
// ---------------------------------------------------------------------------
namespace cmep {
constexpr u16 kServiceCall = 1;     ///< entry=
constexpr u16 kSecureKernel = 2;    ///< base=, size=
constexpr u16 kKeyring = 3;         ///< slot=, size=
constexpr u16 kBigmacBegin = 4;     ///< op=, bytes=
constexpr u16 kBigmacEnd = 5;       ///< bytes=, result=
constexpr u16 kBignum = 6;          ///< op=, bits=
constexpr u16 kSleep = 7;           ///< pc=
constexpr u16 kCryptoBegin = 8;     ///< algo=, bytes=
constexpr u16 kCryptoEnd = 9;       ///< bytes=, result=
constexpr u16 kError = 10;          ///< op=, detail=
}  // namespace cmep

// ---------------------------------------------------------------------------
// Mailbox - the ARM <-> CMeP channels
// ---------------------------------------------------------------------------
namespace mailbox {
constexpr u16 kSend = 1;       ///< channel=, value=
constexpr u16 kReceive = 2;    ///< channel=, value=
constexpr u16 kIrq = 3;        ///< channel=, target=
constexpr u16 kHandshake = 4;  ///< status=, side=
constexpr u16 kClear = 5;      ///< channel=
}  // namespace mailbox

// ---------------------------------------------------------------------------
// Crypto
// ---------------------------------------------------------------------------
namespace crypto {
constexpr u16 kAesBegin = 1;     ///< mode=, bytes=
constexpr u16 kAesEnd = 2;       ///< bytes=, result=
constexpr u16 kSha256Begin = 3;  ///< bytes=
constexpr u16 kSha256End = 4;    ///< digest=
constexpr u16 kHmacBegin = 5;    ///< bytes=
constexpr u16 kHmacEnd = 6;      ///< digest=
constexpr u16 kRsaBegin = 7;     ///< bits=
constexpr u16 kRsaEnd = 8;       ///< result=
constexpr u16 kError = 9;        ///< algo=, detail=
}  // namespace crypto

// ---------------------------------------------------------------------------
// Loader - SLB2 / SELF / ELF / PUP
// ---------------------------------------------------------------------------
namespace loader {
constexpr u16 kLoadBegin = 1;     ///< name=, size=, kind=, address=
constexpr u16 kLoadEnd = 2;       ///< name=, entry=, bytes=, result=
constexpr u16 kDecryptBegin = 3;  ///< name=, segments=
constexpr u16 kDecryptEnd = 4;    ///< name=, bytes=
constexpr u16 kVerify = 5;        ///< name=, result=
constexpr u16 kSlb2 = 6;          ///< entry=, size=, name=
constexpr u16 kError = 7;         ///< name=, detail=
}  // namespace loader

// ---------------------------------------------------------------------------
// Guest kernel
// ---------------------------------------------------------------------------
namespace kernel {
constexpr u16 kEntry = 1;         ///< entry=, path=
constexpr u16 kModuleBegin = 2;   ///< name=, address=
constexpr u16 kModuleEnd = 3;     ///< name=, result=
constexpr u16 kThread = 4;        ///< id=, entry=
constexpr u16 kFault = 5;         ///< address=, detail=
constexpr u16 kRunning = 6;       ///< insns=
constexpr u16 kMilestone = 7;     ///< text=
}  // namespace kernel

// ---------------------------------------------------------------------------
// Bus
// ---------------------------------------------------------------------------
namespace bus {
constexpr u16 kMmioRead = 1;    ///< address=, value=, device=   (Verbose)
constexpr u16 kMmioWrite = 2;   ///< address=, value=, device=   (Verbose)
constexpr u16 kUnmapped = 3;    ///< kind=, address=, pc=
constexpr u16 kFetchFault = 4;  ///< address=, pc=
constexpr u16 kRamAccess = 5;   ///< kind=, address=, size=      (Verbose)
}  // namespace bus

// ---------------------------------------------------------------------------
// Debugger
// ---------------------------------------------------------------------------
namespace debugger {
constexpr u16 kBreakpoint = 1;  ///< arch=, address=, core=
constexpr u16 kWatchpoint = 2;  ///< address=, kind=, id=
constexpr u16 kStop = 3;        ///< reason=, address=
constexpr u16 kCommand = 4;     ///< text=
constexpr u16 kStep = 5;        ///< arch=, address=, count=
}  // namespace debugger

// ---------------------------------------------------------------------------
// Test provider (unit tests emit here)
// ---------------------------------------------------------------------------
namespace test {
constexpr u16 kProbe = 1;  ///< name=, value=
constexpr u16 kActivity = 2;  ///< name=  (Begin/End pair)
}  // namespace test

}  // namespace ev
}  // namespace zlb
