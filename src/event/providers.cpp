// zeliboba - the event provider manifest tables.
//
// One table per provider plus the index that maps an EventProvider to its info.
// Adding a provider means: an enum value in event.h, an entry in kProviders, and
// a table here.  A unit test walks the whole index and asserts that every provider
// is described and every event id is unique.
#include "event/providers.h"

#include <cstring>

namespace zlb {
namespace {

using namespace ev;

// -- tasks (ETW Task) --------------------------------------------------------
namespace task {
constexpr u16 kBoard = 1;
constexpr u16 kLifecycle = 1;
constexpr u16 kStage = 2;
constexpr u16 kMilestone = 3;
constexpr u16 kCore = 1;
constexpr u16 kFault = 2;
constexpr u16 kController = 1;
constexpr u16 kTimer = 1;
constexpr u16 kMapping = 1;
constexpr u16 kAccess = 2;
constexpr u16 kCard = 1;
constexpr u16 kTransfer = 2;
constexpr u16 kHost = 1;
constexpr u16 kChannel = 1;
constexpr u16 kEngine = 2;
constexpr u16 kService = 1;
constexpr u16 kHash = 1;
constexpr u16 kCipher = 2;
constexpr u16 kImage = 1;
constexpr u16 kModule = 2;
constexpr u16 kStop = 1;
constexpr u16 kCommand = 2;
constexpr u16 kProbe = 1;
constexpr u16 kPresent = 2;
constexpr u16 kPower = 2;
}  // namespace task

using L = EventLevel;
using O = EventOpcode;
using K = EventKeyword;
constexpr EventKeyword kState = event_keyword::kState;
constexpr EventKeyword kBoot = event_keyword::kBoot;
constexpr EventKeyword kCpu = event_keyword::kCpu;
constexpr EventKeyword kIrq = event_keyword::kInterrupt;
constexpr EventKeyword kTimer = event_keyword::kTimer;
constexpr EventKeyword kMem = event_keyword::kMemory;
constexpr EventKeyword kStor = event_keyword::kStorage;
constexpr EventKeyword kDisp = event_keyword::kDisplay;
constexpr EventKeyword kPow = event_keyword::kPower;
constexpr EventKeyword kSec = event_keyword::kSecurity;
constexpr EventKeyword kLd = event_keyword::kLoader;
constexpr EventKeyword kKern = event_keyword::kKernel;
constexpr EventKeyword kComm = event_keyword::kComm;
constexpr EventKeyword kDbg = event_keyword::kDebug;
constexpr EventKeyword kBus = event_keyword::kBusAccess;

// -- Machine ----------------------------------------------------------------
constexpr EventMetadata kMachineEvents[] = {
    {.id = machine::kBuild, .level = L::Informational, .opcode = O::Info, .task = task::kBoard,
     .keyword = kState, .name = "Build", .fields = "detail", .description = "board built"},
    {.id = machine::kReset, .level = L::Informational, .opcode = O::Info, .task = task::kBoard,
     .keyword = kState, .name = "Reset", .fields = "cold", .description = "power-on reset"},
    {.id = machine::kPartsFitted, .level = L::Informational, .opcode = O::Info, .task = task::kBoard,
     .keyword = kState, .name = "PartsFitted", .fields = "emmc,syscon,first_loader",
     .description = "firmware parts attached"},
    {.id = machine::kSaveState, .level = L::Informational, .opcode = O::Info, .task = task::kBoard,
     .keyword = kState, .name = "SaveState", .fields = "path,bytes", .description = "state written"},
    {.id = machine::kLoadState, .level = L::Informational, .opcode = O::Info, .task = task::kBoard,
     .keyword = kState, .name = "LoadState", .fields = "path,bytes", .description = "state restored"},
};

// -- Boot -------------------------------------------------------------------
constexpr EventMetadata kBootEvents[] = {
    {.id = boot::kStageBegin, .level = L::Informational, .opcode = O::Start, .task = task::kStage,
     .keyword = kBoot, .name = "Stage", .fields = "stage,detail", .description = "boot stage entered"},
    {.id = boot::kStageEnd, .level = L::Informational, .opcode = O::Stop, .task = task::kStage,
     .keyword = kBoot, .name = "StageEnd", .fields = "stage,result", .description = "boot stage left"},
    {.id = boot::kMilestone, .level = L::Informational, .opcode = O::Info, .task = task::kMilestone,
     .keyword = kBoot, .name = "Milestone", .fields = "text", .description = "boot chain milestone"},
    {.id = boot::kHandshake, .level = L::Informational, .opcode = O::Info, .task = task::kLifecycle,
     .keyword = kComm, .name = "Handshake", .fields = "status,side", .description = "mailbox handshake"},
    {.id = boot::kArmReleased, .level = L::Informational, .opcode = O::Info, .task = task::kLifecycle,
     .keyword = kBoot, .name = "ArmReleased", .fields = "entry,cores", .description = "ARM cluster released"},
    {.id = boot::kComplete, .level = L::Informational, .opcode = O::Info, .task = task::kLifecycle,
     .keyword = kBoot, .name = "Complete", .fields = "stage", .description = "boot chain finished"},
    {.id = boot::kFailure, .level = L::Error, .opcode = O::Info, .task = task::kLifecycle,
     .keyword = kBoot, .name = "Failure", .fields = "stage,reason", .description = "boot chain failed"},
    {.id = boot::kStage, .level = L::Informational, .opcode = O::Info, .task = task::kStage,
     .keyword = kBoot, .name = "StageChange", .fields = "stage,index", .description = "stage transition"},
};

// -- CPU --------------------------------------------------------------------
constexpr EventMetadata kCpuEvents[] = {
    {.id = cpu::kCoreReset, .level = L::Informational, .opcode = O::Info, .task = task::kCore,
     .keyword = kCpu, .name = "CoreReset", .fields = "core,entry", .description = "core reset"},
    {.id = cpu::kCoreHalt, .level = L::Informational, .opcode = O::Info, .task = task::kCore,
     .keyword = kCpu, .name = "CoreHalt", .fields = "core,reason", .description = "core halted"},
    {.id = cpu::kCoreWake, .level = L::Informational, .opcode = O::Info, .task = task::kCore,
     .keyword = kCpu, .name = "CoreWake", .fields = "core,reason", .description = "core left halt"},
    {.id = cpu::kException, .level = L::Warning, .opcode = O::Info, .task = task::kFault,
     .keyword = kCpu, .name = "Exception", .fields = "core,vector,cause", .description = "exception taken"},
    {.id = cpu::kUndefined, .level = L::Error, .opcode = O::Info, .task = task::kFault,
     .keyword = kCpu, .name = "Undefined", .fields = "core,pc,reason", .description = "undefined instruction"},
    {.id = cpu::kSlice, .level = L::Verbose, .opcode = O::Info, .task = task::kCore,
     .keyword = kCpu, .name = "Slice", .fields = "core,cycles", .description = "scheduler slice"},
};

// -- Interrupt --------------------------------------------------------------
constexpr EventMetadata kInterruptEvents[] = {
    {.id = interrupt::kRaise, .level = L::Informational, .opcode = O::Info, .task = task::kController,
     .keyword = kIrq, .name = "Raise", .fields = "line,source", .description = "interrupt raised"},
    {.id = interrupt::kDeliver, .level = L::Informational, .opcode = O::Info, .task = task::kController,
     .keyword = kIrq, .name = "Deliver", .fields = "core,line,vector", .description = "interrupt delivered"},
    {.id = interrupt::kEoi, .level = L::Informational, .opcode = O::Info, .task = task::kController,
     .keyword = kIrq, .name = "Eoi", .fields = "line,source", .description = "end of interrupt"},
    {.id = interrupt::kEnable, .level = L::Verbose, .opcode = O::Info, .task = task::kController,
     .keyword = kIrq, .name = "Enable", .fields = "line,enabled", .description = "line mask change"},
    {.id = interrupt::kPending, .level = L::Verbose, .opcode = O::Info, .task = task::kController,
     .keyword = kIrq, .name = "Pending", .fields = "line,source", .description = "line left pending"},
    {.id = interrupt::kSpurious, .level = L::Warning, .opcode = O::Info, .task = task::kController,
     .keyword = kIrq, .name = "Spurious", .fields = "line", .description = "spurious interrupt"},
};

// -- Timer ------------------------------------------------------------------
constexpr EventMetadata kTimerEvents[] = {
    {.id = timer::kArm, .level = L::Verbose, .opcode = O::Start, .task = task::kTimer,
     .keyword = kTimer, .name = "Arm", .fields = "timer,ticks", .description = "timer armed"},
    {.id = timer::kExpire, .level = L::Informational, .opcode = O::Stop, .task = task::kTimer,
     .keyword = kTimer, .name = "Expire", .fields = "timer,count", .description = "timer expired"},
    {.id = timer::kStop, .level = L::Verbose, .opcode = O::Info, .task = task::kTimer,
     .keyword = kTimer, .name = "Stop", .fields = "timer", .description = "timer stopped"},
    {.id = timer::kReset, .level = L::Informational, .opcode = O::Info, .task = task::kTimer,
     .keyword = kTimer, .name = "Reset", .fields = "timer", .description = "timer reset"},
};

// -- Memory -----------------------------------------------------------------
constexpr EventMetadata kMemoryEvents[] = {
    {.id = memory::kMapRegion, .level = L::Informational, .opcode = O::Info, .task = task::kMapping,
     .keyword = kMem, .name = "MapRegion", .fields = "name,base,size", .description = "region mapped"},
    {.id = memory::kUnmapped, .level = L::Warning, .opcode = O::Info, .task = task::kAccess,
     .keyword = kMem, .name = "Unmapped", .fields = "kind,address,pc", .description = "unmapped access"},
    {.id = memory::kAllocate, .level = L::Verbose, .opcode = O::Info, .task = task::kMapping,
     .keyword = kMem, .name = "Allocate", .fields = "size,address,tag", .description = "allocation"},
    {.id = memory::kAperture, .level = L::Informational, .opcode = O::Info, .task = task::kMapping,
     .keyword = kMem, .name = "Aperture", .fields = "name,base,size", .description = "memory aperture"},
    {.id = memory::kWrite, .level = L::Verbose, .opcode = O::Info, .task = task::kAccess,
     .keyword = kMem, .name = "Write", .fields = "address,value,size", .description = "sensitive write"},
};

// -- eMMC -------------------------------------------------------------------
constexpr EventMetadata kEmmcEvents[] = {
    {.id = emmc::kTransferBegin, .level = L::Informational, .opcode = O::Start, .task = task::kTransfer,
     .keyword = kStor, .name = "Transfer", .fields = "dir,lba,blocks", .description = "card transfer started"},
    {.id = emmc::kTransferEnd, .level = L::Informational, .opcode = O::Stop, .task = task::kTransfer,
     .keyword = kStor, .name = "TransferEnd", .fields = "bytes,result", .description = "card transfer finished"},
    {.id = emmc::kCommand, .level = L::Verbose, .opcode = O::Info, .task = task::kCard,
     .keyword = kStor, .name = "Command", .fields = "cmd,arg", .description = "card command"},
    {.id = emmc::kError, .level = L::Error, .opcode = O::Info, .task = task::kCard,
     .keyword = kStor, .name = "Error", .fields = "op,detail", .description = "card error"},
    {.id = emmc::kAttach, .level = L::Informational, .opcode = O::Info, .task = task::kCard,
     .keyword = kStor, .name = "Attach", .fields = "path,capacity", .description = "image attached"},
    {.id = emmc::kReset, .level = L::Informational, .opcode = O::Info, .task = task::kCard,
     .keyword = kStor, .name = "Reset", .fields = "reason", .description = "card reset"},
};

// -- SDIF -------------------------------------------------------------------
constexpr EventMetadata kSdifEvents[] = {
    {.id = sdif::kCommand, .level = L::Verbose, .opcode = O::Info, .task = task::kHost,
     .keyword = kStor, .name = "Command", .fields = "cmd,arg", .description = "command issued"},
    {.id = sdif::kTransferBegin, .level = L::Informational, .opcode = O::Start, .task = task::kTransfer,
     .keyword = kStor, .name = "Transfer", .fields = "lba,blocks,dir", .description = "data transfer started"},
    {.id = sdif::kTransferEnd, .level = L::Informational, .opcode = O::Stop, .task = task::kTransfer,
     .keyword = kStor, .name = "TransferEnd", .fields = "bytes,result", .description = "data transfer finished"},
    {.id = sdif::kError, .level = L::Error, .opcode = O::Info, .task = task::kHost,
     .keyword = kStor, .name = "Error", .fields = "op,detail", .description = "host error"},
    {.id = sdif::kClock, .level = L::Verbose, .opcode = O::Info, .task = task::kHost,
     .keyword = kStor, .name = "Clock", .fields = "hz", .description = "clock change"},
};

// -- DMA --------------------------------------------------------------------
constexpr EventMetadata kDmaEvents[] = {
    {.id = dma::kTransferBegin, .level = L::Informational, .opcode = O::Start, .task = task::kChannel,
     .keyword = kStor, .name = "Transfer", .fields = "channel,src,dst,bytes", .description = "DMA started"},
    {.id = dma::kTransferEnd, .level = L::Informational, .opcode = O::Stop, .task = task::kChannel,
     .keyword = kStor, .name = "TransferEnd", .fields = "channel,bytes,result", .description = "DMA finished"},
    {.id = dma::kError, .level = L::Error, .opcode = O::Info, .task = task::kChannel,
     .keyword = kStor, .name = "Error", .fields = "channel,detail", .description = "DMA error"},
};

// -- Display ----------------------------------------------------------------
constexpr EventMetadata kDisplayEvents[] = {
    {.id = display::kModeSet, .level = L::Informational, .opcode = O::Info, .task = task::kController,
     .keyword = kDisp, .name = "ModeSet", .fields = "width,height,format", .description = "display mode set"},
    {.id = display::kFrameBuffer, .level = L::Informational, .opcode = O::Info, .task = task::kController,
     .keyword = kDisp, .name = "FrameBuffer", .fields = "address,stride,format", .description = "framebuffer set"},
    {.id = display::kFramePresent, .level = L::Informational, .opcode = O::Info, .task = task::kPresent,
     .keyword = kDisp, .name = "FramePresent", .fields = "frames,address", .description = "frame presented"},
    {.id = display::kVblank, .level = L::Verbose, .opcode = O::Info, .task = task::kPresent,
     .keyword = kDisp, .name = "Vblank", .fields = "count", .description = "vblank"},
    {.id = display::kTurnoverBegin, .level = L::Informational, .opcode = O::Start, .task = task::kPresent,
     .keyword = kDisp, .name = "Turnover", .fields = "bank", .description = "IFTU turnover started"},
    {.id = display::kTurnoverEnd, .level = L::Informational, .opcode = O::Stop, .task = task::kPresent,
     .keyword = kDisp, .name = "TurnoverEnd", .fields = "bank", .description = "IFTU turnover finished"},
    {.id = display::kDsiCommand, .level = L::Verbose, .opcode = O::Info, .task = task::kController,
     .keyword = kDisp, .name = "DsiCommand", .fields = "cmd,length", .description = "DSI command"},
    {.id = display::kReset, .level = L::Informational, .opcode = O::Info, .task = task::kController,
     .keyword = kDisp, .name = "Reset", .fields = "reason", .description = "display reset"},
};

// -- GPU --------------------------------------------------------------------
constexpr EventMetadata kGpuEvents[] = {
    {.id = gpu::kSubmit, .level = L::Verbose, .opcode = O::Info, .task = task::kEngine,
     .keyword = kDisp, .name = "Submit", .fields = "cmd,bytes", .description = "command submitted"},
    {.id = gpu::kFrame, .level = L::Informational, .opcode = O::Info, .task = task::kPresent,
     .keyword = kDisp, .name = "Frame", .fields = "frames", .description = "frame completed"},
};

// -- Syscon -----------------------------------------------------------------
constexpr EventMetadata kSysconEvents[] = {
    {.id = syscon::kScCommand, .level = L::Verbose, .opcode = O::Send, .task = task::kController,
     .keyword = kPow, .name = "ScCommand", .fields = "cmd,payload", .description = "SC command sent"},
    {.id = syscon::kScReply, .level = L::Verbose, .opcode = O::Receive, .task = task::kController,
     .keyword = kPow, .name = "ScReply", .fields = "cmd,result", .description = "SC reply received"},
    {.id = syscon::kPowerState, .level = L::Informational, .opcode = O::Info, .task = task::kPower,
     .keyword = kPow, .name = "PowerState", .fields = "from,to", .description = "power state change"},
    {.id = syscon::kReset, .level = L::Warning, .opcode = O::Info, .task = task::kPower,
     .keyword = kPow, .name = "Reset", .fields = "reason", .description = "reset requested"},
    {.id = syscon::kRtc, .level = L::Verbose, .opcode = O::Info, .task = task::kController,
     .keyword = kPow, .name = "Rtc", .fields = "value", .description = "RTC access"},
    {.id = syscon::kEmmcHost, .level = L::Verbose, .opcode = O::Info, .task = task::kController,
     .keyword = kStor, .name = "EmmcHost", .fields = "cmd", .description = "eMMC host command"},
    {.id = syscon::kError, .level = L::Error, .opcode = O::Info, .task = task::kController,
     .keyword = kPow, .name = "Error", .fields = "op,detail", .description = "syscon error"},
};

// -- CMeP -------------------------------------------------------------------
constexpr EventMetadata kCmepEvents[] = {
    {.id = cmep::kServiceCall, .level = L::Informational, .opcode = O::Info, .task = task::kService,
     .keyword = kSec, .name = "ServiceCall", .fields = "entry", .description = "first-loader service called"},
    {.id = cmep::kSecureKernel, .level = L::Informational, .opcode = O::Info, .task = task::kService,
     .keyword = kSec, .name = "SecureKernel", .fields = "base,size", .description = "secure kernel entered"},
    {.id = cmep::kKeyring, .level = L::Informational, .opcode = O::Info, .task = task::kEngine,
     .keyword = kSec, .name = "Keyring", .fields = "slot,size", .description = "key loaded"},
    {.id = cmep::kBigmacBegin, .level = L::Informational, .opcode = O::Start, .task = task::kEngine,
     .keyword = kSec, .name = "Bigmac", .fields = "op,bytes", .description = "crypto engine started"},
    {.id = cmep::kBigmacEnd, .level = L::Informational, .opcode = O::Stop, .task = task::kEngine,
     .keyword = kSec, .name = "BigmacEnd", .fields = "bytes,result", .description = "crypto engine finished"},
    {.id = cmep::kBignum, .level = L::Verbose, .opcode = O::Info, .task = task::kEngine,
     .keyword = kSec, .name = "Bignum", .fields = "op,bits", .description = "bignum operation"},
    {.id = cmep::kSleep, .level = L::Informational, .opcode = O::Info, .task = task::kService,
     .keyword = kSec, .name = "Sleep", .fields = "pc", .description = "CMeP idle"},
    {.id = cmep::kCryptoBegin, .level = L::Verbose, .opcode = O::Start, .task = task::kEngine,
     .keyword = kSec, .name = "Crypto", .fields = "algo,bytes", .description = "crypto operation started"},
    {.id = cmep::kCryptoEnd, .level = L::Verbose, .opcode = O::Stop, .task = task::kEngine,
     .keyword = kSec, .name = "CryptoEnd", .fields = "bytes,result", .description = "crypto operation finished"},
    {.id = cmep::kError, .level = L::Error, .opcode = O::Info, .task = task::kService,
     .keyword = kSec, .name = "Error", .fields = "op,detail", .description = "CMeP error"},
};

// -- Mailbox ----------------------------------------------------------------
constexpr EventMetadata kMailboxEvents[] = {
    {.id = mailbox::kSend, .level = L::Verbose, .opcode = O::Send, .task = task::kChannel,
     .keyword = kComm, .name = "Send", .fields = "channel,value", .description = "mailbox write"},
    {.id = mailbox::kReceive, .level = L::Verbose, .opcode = O::Receive, .task = task::kChannel,
     .keyword = kComm, .name = "Receive", .fields = "channel,value", .description = "mailbox read"},
    {.id = mailbox::kIrq, .level = L::Informational, .opcode = O::Info, .task = task::kChannel,
     .keyword = kIrq, .name = "Irq", .fields = "channel,target", .description = "mailbox interrupt"},
    {.id = mailbox::kHandshake, .level = L::Informational, .opcode = O::Info, .task = task::kChannel,
     .keyword = kComm, .name = "Handshake", .fields = "status,side", .description = "handshake observed"},
    {.id = mailbox::kClear, .level = L::Verbose, .opcode = O::Info, .task = task::kChannel,
     .keyword = kComm, .name = "Clear", .fields = "channel", .description = "mailbox cleared"},
};

// -- Crypto -----------------------------------------------------------------
constexpr EventMetadata kCryptoEvents[] = {
    {.id = crypto::kAesBegin, .level = L::Verbose, .opcode = O::Start, .task = task::kCipher,
     .keyword = kSec, .name = "Aes", .fields = "mode,bytes", .description = "AES started"},
    {.id = crypto::kAesEnd, .level = L::Verbose, .opcode = O::Stop, .task = task::kCipher,
     .keyword = kSec, .name = "AesEnd", .fields = "bytes,result", .description = "AES finished"},
    {.id = crypto::kSha256Begin, .level = L::Verbose, .opcode = O::Start, .task = task::kHash,
     .keyword = kSec, .name = "Sha256", .fields = "bytes", .description = "SHA-256 started"},
    {.id = crypto::kSha256End, .level = L::Verbose, .opcode = O::Stop, .task = task::kHash,
     .keyword = kSec, .name = "Sha256End", .fields = "digest", .description = "SHA-256 finished"},
    {.id = crypto::kHmacBegin, .level = L::Verbose, .opcode = O::Start, .task = task::kHash,
     .keyword = kSec, .name = "Hmac", .fields = "bytes", .description = "HMAC started"},
    {.id = crypto::kHmacEnd, .level = L::Verbose, .opcode = O::Stop, .task = task::kHash,
     .keyword = kSec, .name = "HmacEnd", .fields = "digest", .description = "HMAC finished"},
    {.id = crypto::kRsaBegin, .level = L::Verbose, .opcode = O::Start, .task = task::kCipher,
     .keyword = kSec, .name = "Rsa", .fields = "bits", .description = "RSA started"},
    {.id = crypto::kRsaEnd, .level = L::Verbose, .opcode = O::Stop, .task = task::kCipher,
     .keyword = kSec, .name = "RsaEnd", .fields = "result", .description = "RSA finished"},
    {.id = crypto::kError, .level = L::Error, .opcode = O::Info, .task = task::kHash,
     .keyword = kSec, .name = "Error", .fields = "algo,detail", .description = "crypto error"},
};

// -- Loader -----------------------------------------------------------------
constexpr EventMetadata kLoaderEvents[] = {
    {.id = loader::kLoadBegin, .level = L::Informational, .opcode = O::Start, .task = task::kImage,
     .keyword = kLd, .name = "Load", .fields = "name,size,kind,address", .description = "image load started"},
    {.id = loader::kLoadEnd, .level = L::Informational, .opcode = O::Stop, .task = task::kImage,
     .keyword = kLd, .name = "LoadEnd", .fields = "name,entry,bytes,result", .description = "image load finished"},
    {.id = loader::kDecryptBegin, .level = L::Informational, .opcode = O::Start, .task = task::kImage,
     .keyword = kSec, .name = "Decrypt", .fields = "name,segments", .description = "SELF decrypt started"},
    {.id = loader::kDecryptEnd, .level = L::Informational, .opcode = O::Stop, .task = task::kImage,
     .keyword = kSec, .name = "DecryptEnd", .fields = "name,bytes", .description = "SELF decrypt finished"},
    {.id = loader::kVerify, .level = L::Informational, .opcode = O::Info, .task = task::kImage,
     .keyword = kSec, .name = "Verify", .fields = "name,result", .description = "signature check"},
    {.id = loader::kSlb2, .level = L::Informational, .opcode = O::Info, .task = task::kImage,
     .keyword = kLd, .name = "Slb2", .fields = "entry,size,name", .description = "SLB2 entry read"},
    {.id = loader::kError, .level = L::Error, .opcode = O::Info, .task = task::kImage,
     .keyword = kLd, .name = "Error", .fields = "name,detail", .description = "load failure"},
};

// -- Kernel -----------------------------------------------------------------
constexpr EventMetadata kKernelEvents[] = {
    {.id = kernel::kEntry, .level = L::Informational, .opcode = O::Info, .task = task::kLifecycle,
     .keyword = kKern, .name = "Entry", .fields = "entry,path", .description = "kernel entry"},
    {.id = kernel::kModuleBegin, .level = L::Informational, .opcode = O::Start, .task = task::kModule,
     .keyword = kKern, .name = "Module", .fields = "name,address", .description = "module start"},
    {.id = kernel::kModuleEnd, .level = L::Informational, .opcode = O::Stop, .task = task::kModule,
     .keyword = kKern, .name = "ModuleEnd", .fields = "name,result", .description = "module finished"},
    {.id = kernel::kThread, .level = L::Verbose, .opcode = O::Info, .task = task::kModule,
     .keyword = kKern, .name = "Thread", .fields = "id,entry", .description = "guest thread"},
    {.id = kernel::kFault, .level = L::Error, .opcode = O::Info, .task = task::kLifecycle,
     .keyword = kKern, .name = "Fault", .fields = "address,detail", .description = "guest fault"},
    {.id = kernel::kRunning, .level = L::Informational, .opcode = O::Info, .task = task::kLifecycle,
     .keyword = kKern, .name = "Running", .fields = "insns", .description = "guest kernel running"},
    {.id = kernel::kMilestone, .level = L::Informational, .opcode = O::Info, .task = task::kLifecycle,
     .keyword = kKern, .name = "Milestone", .fields = "text", .description = "guest milestone"},
};

// -- Bus --------------------------------------------------------------------
constexpr EventMetadata kBusEvents[] = {
    {.id = bus::kMmioRead, .level = L::Verbose, .opcode = O::Info, .task = task::kAccess,
     .keyword = kBus, .name = "MmioRead", .fields = "address,value,device", .description = "MMIO read"},
    {.id = bus::kMmioWrite, .level = L::Verbose, .opcode = O::Info, .task = task::kAccess,
     .keyword = kBus, .name = "MmioWrite", .fields = "address,value,device", .description = "MMIO write"},
    {.id = bus::kUnmapped, .level = L::Warning, .opcode = O::Info, .task = task::kAccess,
     .keyword = kMem, .name = "Unmapped", .fields = "kind,address,pc", .description = "unmapped access"},
    {.id = bus::kFetchFault, .level = L::Warning, .opcode = O::Info, .task = task::kAccess,
     .keyword = kMem, .name = "FetchFault", .fields = "address,pc", .description = "fetch fault"},
    {.id = bus::kRamAccess, .level = L::Verbose, .opcode = O::Info, .task = task::kAccess,
     .keyword = kBus, .name = "RamAccess", .fields = "kind,address,size", .description = "RAM access"},
};

// -- Debugger ---------------------------------------------------------------
constexpr EventMetadata kDebuggerEvents[] = {
    {.id = debugger::kBreakpoint, .level = L::Informational, .opcode = O::Info, .task = task::kStop,
     .keyword = kDbg, .name = "Breakpoint", .fields = "arch,address,core", .description = "breakpoint hit"},
    {.id = debugger::kWatchpoint, .level = L::Informational, .opcode = O::Info, .task = task::kStop,
     .keyword = kDbg, .name = "Watchpoint", .fields = "address,kind,id", .description = "watchpoint hit"},
    {.id = debugger::kStop, .level = L::Informational, .opcode = O::Info, .task = task::kStop,
     .keyword = kDbg, .name = "Stop", .fields = "reason,address", .description = "machine stopped"},
    {.id = debugger::kCommand, .level = L::Verbose, .opcode = O::Info, .task = task::kCommand,
     .keyword = kDbg, .name = "Command", .fields = "text", .description = "debugger command"},
    {.id = debugger::kStep, .level = L::Verbose, .opcode = O::Info, .task = task::kStop,
     .keyword = kDbg, .name = "Step", .fields = "arch,address,count", .description = "single step"},
};

// -- Test -------------------------------------------------------------------
constexpr EventMetadata kTestEvents[] = {
    {.id = test::kProbe, .level = L::Informational, .opcode = O::Info, .task = task::kProbe,
     .keyword = event_keyword::kNone, .name = "Probe", .fields = "name,value", .description = "test probe"},
    {.id = test::kActivity, .level = L::Informational, .opcode = O::Start, .task = task::kProbe,
     .keyword = event_keyword::kNone, .name = "Activity", .fields = "name", .description = "test activity"},
};

// -- provider table ---------------------------------------------------------
constexpr EventProviderInfo kProviders[] = {
    {EventProvider::Machine, "Zeliboba-Machine", "7c1b0d01-0000-4000-8000-000000000001", EventArea::Other,
     "board build, reset, fitted parts and save states", kMachineEvents, std::size(kMachineEvents)},
    {EventProvider::Boot, "Zeliboba-Boot", "7c1b0d02-0000-4000-8000-000000000002", EventArea::Other,
     "the firmware boot chain: stages, handshakes and milestones", kBootEvents, std::size(kBootEvents)},
    {EventProvider::Cpu, "Zeliboba-CPU", "7c1b0d03-0000-4000-8000-000000000003", EventArea::Computation,
     "core reset, halts, wakes and exceptions", kCpuEvents, std::size(kCpuEvents)},
    {EventProvider::Interrupt, "Zeliboba-Interrupt", "7c1b0d04-0000-4000-8000-000000000004",
     EventArea::Computation, "IRQ/FIQ raise, delivery and EOI", kInterruptEvents, std::size(kInterruptEvents)},
    {EventProvider::Timer, "Zeliboba-Timer", "7c1b0d05-0000-4000-8000-000000000005", EventArea::Computation,
     "timers and the systimer", kTimerEvents, std::size(kTimerEvents)},
    {EventProvider::Memory, "Zeliboba-Memory", "7c1b0d06-0000-4000-8000-000000000006", EventArea::Memory,
     "memory regions, apertures and unmapped accesses", kMemoryEvents, std::size(kMemoryEvents)},
    {EventProvider::Emmc, "Zeliboba-eMMC", "7c1b0d07-0000-4000-8000-000000000007", EventArea::Storage,
     "the eMMC card: transfers, commands and errors", kEmmcEvents, std::size(kEmmcEvents)},
    {EventProvider::Sdif, "Zeliboba-SDIF", "7c1b0d08-0000-4000-8000-000000000008", EventArea::Storage,
     "the SDIF host controller", kSdifEvents, std::size(kSdifEvents)},
    {EventProvider::Dma, "Zeliboba-DMA", "7c1b0d09-0000-4000-8000-000000000009", EventArea::Storage,
     "DMA channel transfers", kDmaEvents, std::size(kDmaEvents)},
    {EventProvider::Display, "Zeliboba-Display", "7c1b0d0a-0000-4000-8000-00000000000a", EventArea::Video,
     "DSI, IFTU and framebuffer turnover", kDisplayEvents, std::size(kDisplayEvents)},
    {EventProvider::Gpu, "Zeliboba-GPU", "7c1b0d0b-0000-4000-8000-00000000000b", EventArea::Video,
     "the SGX command path", kGpuEvents, std::size(kGpuEvents)},
    {EventProvider::Syscon, "Zeliboba-Syscon", "7c1b0d0c-0000-4000-8000-00000000000c", EventArea::Power,
     "Ernie: SC protocol, power and reset", kSysconEvents, std::size(kSysconEvents)},
    {EventProvider::Cmep, "Zeliboba-CMeP", "7c1b0d0d-0000-4000-8000-00000000000d",
     EventArea::Communications, "the F00D security core, its services and engines", kCmepEvents,
     std::size(kCmepEvents)},
    {EventProvider::Mailbox, "Zeliboba-Mailbox", "7c1b0d0e-0000-4000-8000-00000000000e",
     EventArea::Communications, "the ARM <-> CMeP mailbox channels", kMailboxEvents, std::size(kMailboxEvents)},
    {EventProvider::Crypto, "Zeliboba-Crypto", "7c1b0d0f-0000-4000-8000-00000000000f", EventArea::Other,
     "AES, SHA-256, HMAC and RSA operations", kCryptoEvents, std::size(kCryptoEvents)},
    {EventProvider::Loader, "Zeliboba-Loader", "7c1b0d10-0000-4000-8000-000000000010", EventArea::Other,
     "SLB2, SELF, ELF and PUP image loading", kLoaderEvents, std::size(kLoaderEvents)},
    {EventProvider::Kernel, "Zeliboba-Kernel", "7c1b0d11-0000-4000-8000-000000000011", EventArea::Computation,
     "the guest kernel: entry, modules and faults", kKernelEvents, std::size(kKernelEvents)},
    {EventProvider::Bus, "Zeliboba-Bus", "7c1b0d12-0000-4000-8000-000000000012", EventArea::Other,
     "raw bus accesses (opt-in, high volume)", kBusEvents, std::size(kBusEvents)},
    {EventProvider::Debugger, "Zeliboba-Debugger", "7c1b0d13-0000-4000-8000-000000000013", EventArea::Other,
     "breakpoints, watchpoints and stops", kDebuggerEvents, std::size(kDebuggerEvents)},
    {EventProvider::Test, "Zeliboba-Test", "7c1b0d14-0000-4000-8000-000000000014", EventArea::Other,
     "probes emitted by the unit tests", kTestEvents, std::size(kTestEvents)},
};
static_assert(std::size(kProviders) == static_cast<size_t>(kEventProviderCount),
              "every EventProvider needs a manifest entry");

const EventProviderInfo kUnknownProvider = {EventProvider::Test, "unknown", "", EventArea::Other,
                                            "unknown provider", nullptr, 0};

}  // namespace

const EventProviderInfo& event_provider_info(EventProvider provider) {
    const int index = static_cast<int>(provider);
    if (index < 0 || index >= kEventProviderCount) return kUnknownProvider;
    return kProviders[index];
}

const EventMetadata* event_metadata(EventProvider provider, u16 id) {
    const EventProviderInfo& info = event_provider_info(provider);
    for (size_t i = 0; i < info.event_count; ++i) {
        if (info.events[i].id == id) return &info.events[i];
    }
    return nullptr;
}

const char* event_name(EventProvider provider, u16 id) {
    const EventMetadata* metadata = event_metadata(provider, id);
    return metadata ? metadata->name : "Event";
}

}  // namespace zlb
