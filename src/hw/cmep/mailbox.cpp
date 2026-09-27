// zeliboba - mailboxes, straps, GPIO and the SC bridge of the CMeP block.
//
// ---------------------------------------------------------------------------
// Register protocol - evidence
// ---------------------------------------------------------------------------
// Mailboxes (ANALYSIS.md appendix A, annotated listing):
//
//   0xE0000000  CMeP -> ARM status.  main writes 1 on success (0x5C5F0) and 2
//               on the failure path (0x5C616).  ARM polls it to learn that the
//               first loader is done.
//   0xE0000010  ARM -> CMeP command / physical address.  main polls it until
//               bit 0 is set (0x5C57A `lw $7,($10)` / 0x5C580 `beqz`), then
//               takes bits 2+ as a 4-byte aligned PA (`mov $12,-4; and $7,$12`,
//               0x5C586) and copies 64 bytes from it to 0x40000 (0x5C58C).
//   0xE0000020/0x24  CMeP -> debugger (two 32-bit words, written at 0x5E54C
//               and 0x5E556 from the live value of 0xE005003C).
//   0xE0000028/0x2C  debugger -> CMeP.  mailbox_debug_sc reads both (0x5E58C,
//               0x5E59A) and then writes 0xFFFFFFFF back into all four
//               registers (0x5E5AA, 0x5E5B4, 0x5E5B6, 0x5E5B8) - the ack.
//   0xE0000060/0x64  second debug mailbox pair, same protocol.
//
// Strap / GPIO / SC bridge:
//
//   0xE0062020  the 32-byte strap block.  check_boot_mode reads only word 0
//               (0x5C116) and tests bit 0 (0x5C118) to pick 'A' vs '!';
//               keyring_set_value copies all 32 bytes into keyring 0x501 when
//               boot-mode bit 0 is set (loop at 0x5C164..0x5C178).
//   0xE0064060  read three times (0x5C2BC, 0x5C416, 0x5C808); img_proc_5C798
//               uses the low 16 bits as an "eMMC key index select" mask.
//   0xE20A0000  GPIO output register (written with 8 at 0x5E4FA).
//   0xE20A0004  GPIO state/handshake.  mailbox_debug_sc waits for bit 4 to
//               become *set* (0x5E4FC + 0x5E502) and, after signalling on
//               0xE20A0008, waits for it to become *clear* (0x5E560/0x5E566).
//   0xE20A0008  GPIO set (bit 3 = 8, 0x5E558).
//   0xE20A000C  GPIO clear (bit 3 = 8, 0x5E5BA).
//   0xE310xxxx  SC registers.  The boot paths program a request bit into
//               0xE31010A0 / 0xE31020A0 and poll the same address until it is
//               non-zero, then write the same value to 0xE31010A4 /
//               0xE31020A4 and wait for it to clear (0x5C246..0x5C28C and
//               0x5C3BC..0x5C3E6).
#include <cstring>

#include "common/log.h"
#include "hw/cmep/cmep_internal.h"
#include "hw/syscon.h"

namespace zlb {
namespace cmep_detail {

// ---------------------------------------------------------------------------
// MailboxDevice
// ---------------------------------------------------------------------------

MailboxDevice::MailboxDevice(Bus& bus, CmepBlock& owner)
    : RegisterFile("CMeP.Mailbox", cmep::kMailboxBase, 0x100), bus_(bus), owner_(owner) {
    define(kCmepToArm, "MailboxCmepToArm.Status", 0);
    define(0xE0000004, "MailboxCmepToArm.Status2", 0);
    define(0xE0000008, "MailboxCmepToArm.Status3", 0);
    define(0xE000000C, "MailboxCmepToArm.Status4", 0);
    define(kArmToCmep, "MailboxArmToCmep.Command (bit0 = present)", 0);
    define(0xE0000014, "MailboxArmToCmep.Func0", 0);
    define(0xE0000018, "MailboxArmToCmep.Func1", 0);
    define(0xE000001C, "MailboxArmToCmep.Func2", 0);
    define(kCmepToDebugger, "MailboxCmepToDebugger", 0);
    define(0xE0000024, "MailboxCmepToDebugger+0x4", 0);
    define(kDebuggerToCmep, "MailboxDebuggerToCmep", 0xFFFFFFFF);
    define(0xE000002C, "MailboxDebuggerToCmep+0x4", 0xFFFFFFFF);
    define(kDebuggerToCmep2, "MailboxDebuggerToCmep2", 0xFFFFFFFF);
    define(0xE0000064, "MailboxDebuggerToCmep2+0x4", 0xFFFFFFFF);
}

u64 MailboxDevice::read(u32 address, unsigned size) {
    return RegisterFile::read(address, size);
}

void MailboxDevice::write(u32 address, unsigned size, u64 value) {
    if (address == kCmepToArm) {
        // 0x5C5F0 (1 = success) and 0x5C616 (2 = the failure path).
        RegisterFile::write(address, size, value);
        return;
    }
    if (address == kDebuggerToCmep || address == 0xE000002C || address == kDebuggerToCmep2 ||
        address == 0xE0000064) {
        if (static_cast<u32>(value) == 0xFFFFFFFFu) ++debug_acks_;
        RegisterFile::write(address, size, value);
        return;
    }
    if (address == kCmepToDebugger || address == 0xE0000024) ++debug_posts_;
    RegisterFile::write(address, size, value);
}

std::string MailboxDevice::summary() const {
    return format("cmep->arm=0x%X arm->cmep=0x%08X posts=%llu acks=%llu",
                  static_cast<u32>(peek(kCmepToArm)), static_cast<u32>(peek(kArmToCmep)),
                  static_cast<unsigned long long>(debug_posts_),
                  static_cast<unsigned long long>(debug_acks_));
}

void MailboxDevice::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("Mailbox CMeP->ARM   = 0x%X (%s)", static_cast<u32>(peek(kCmepToArm)),
                           owner_.reported_success() ? "success"
                                                     : (owner_.reported_failure() ? "failure"
                                                                                  : "pending")));
    lines.push_back(format("Mailbox ARM->CMeP   = 0x%08X", static_cast<u32>(peek(kArmToCmep))));
    lines.push_back(format("Debugger mailbox    = 0x%08X/0x%08X (posts %llu, acks %llu)",
                           static_cast<u32>(peek(kDebuggerToCmep)),
                           static_cast<u32>(peek(0xE000002C)),
                           static_cast<unsigned long long>(debug_posts_),
                           static_cast<unsigned long long>(debug_acks_)));
}

// ---------------------------------------------------------------------------
// SceBlockDevice (0x5FFC0000)
// ---------------------------------------------------------------------------

SceBlockDevice::SceBlockDevice()
    : RegisterFile("CMeP.SceBlock", kBase, kSize) {
    // The answer the loader checks for (0x4A774..0x4A796): magic "SCE\0", word 1
    // == 3 and bit 0x40 of byte 8.  The block starts as plain scratch memory -
    // the loader fills the request itself and reads it back - and the header is
    // only pre-set until the first write, so that an exchange which never writes
    // the magic still answers with a valid block.
    define(kBase + 0x00, "SCE block magic", 0x00454353u);   // "SCE\0"
    define(kBase + 0x04, "SCE block status", 0x00000003u);
    define(kBase + 0x08, "SCE block flags", 0x00010140u);
    for (u32 offset = 0x0C; offset < 0x30; offset += 4) {
        define(kBase + offset, format("SCE block +0x%02X", offset), 0);
    }
}

// ---------------------------------------------------------------------------
// CmepStorageDevice (0xE0100000)
// ---------------------------------------------------------------------------

CmepStorageDevice::CmepStorageDevice()
    : RegisterFile("CMeP.Storage", kBase, kSize) {
    // The second loader's controller init (0x49568..0x495E0) reads the id at
    // +0x00, the banks at +0x14..+0x2C and the interrupt/status words at
    // +0x38..+0x48 (writing those back to clear them).  The controller is idle
    // at boot, so every register resets to zero instead of the 0xFF an unmapped
    // window would hand back.
    define(kBase + 0x00, "CMeP storage controller id/status", 0);
    for (u32 offset = 0x14; offset <= 0x2C; offset += 4) {
        define(kBase + offset, format("CMeP storage controller +0x%02X", offset), 0);
    }
    for (u32 offset = 0x38; offset <= 0x48; offset += 4) {
        define(kBase + offset, format("CMeP storage controller irq/status +0x%02X", offset), 0);
    }
}

// ---------------------------------------------------------------------------
// CmepFlagsDevice
// ---------------------------------------------------------------------------

namespace {

/// Highest set bit of a 3 bit command, as the status nibble reports it
/// (5 -> 4, 2 -> 2, 1 -> 1; the init at 0x4840E..0x484B0 waits for exactly that).
u8 highest_bit(u8 value) {
    u8 out = 0;
    for (u8 bit = 0; bit < 4; ++bit) {
        if ((value & (1u << bit)) != 0) out = static_cast<u8>(1u << bit);
    }
    return out;
}

}  // namespace

CmdBlockDevice::CmdBlockDevice() : RegisterFile("CMeP.CmdBlock", cmep::kCmdBlockBase, cmep::kCmdBlockSize) {
    for (u32 i = 0; i < kSlots; ++i) {
        define(cmep::kCmdBlockBase + 0x8000 + i * 8, format("CMeP cmd block slot %u command", i), 0);
        define(cmep::kCmdBlockBase + 0x8100 + i * 8, format("CMeP cmd block slot %u value", i), 0);
        define(cmep::kCmdBlockBase + 0x8120 + i * 8, format("CMeP cmd block slot %u byte port", i), 0);
    }
    define(cmep::kCmdBlockBase + 0x8020, "CMeP cmd block status nibbles", 0);
    define(cmep::kCmdBlockBase + 0x8180, "CMeP cmd block commit", 0);
}

u64 CmdBlockDevice::read(u32 address, unsigned size) {
    if (address == cmep::kCmdBlockBase + 0x8020) {
        u32 packed = 0;
        for (u32 i = 0; i < kSlots; ++i) {
            packed |= static_cast<u32>(highest_bit(command_[i])) << (i * 4);
        }
        return packed;
    }
    if (address >= cmep::kCmdBlockBase + 0x8120 && address < cmep::kCmdBlockBase + 0x8120 + kSlots * 8) {
        const u32 slot = (address - (cmep::kCmdBlockBase + 0x8120)) / 8;
        // The low byte keeps the command that was written (the descriptor walker
        // at 0x4827E writes a value and waits until it reads it back), the answer
        // sits in bits 8..15 (0x48370: `and $0,$0,0xff00` / `srl $0,0x8`).
        return (peek(address) & 0xFFull) | (static_cast<u64>(reply_[slot]) << 8);
    }
    return RegisterFile::read(address, size);
}

void CmdBlockDevice::write(u32 address, unsigned size, u64 value) {
    RegisterFile::write(address, size, value);
    if (address >= cmep::kCmdBlockBase + 0x8000 && address < cmep::kCmdBlockBase + 0x8000 + kSlots * 8) {
        const u32 slot = (address - (cmep::kCmdBlockBase + 0x8000)) / 8;
        command_[slot] = static_cast<u8>(value & 0x7u);
        return;
    }
    if (address >= cmep::kCmdBlockBase + 0x8120 && address < cmep::kCmdBlockBase + 0x8120 + kSlots * 8) {
        const u32 slot = (address - (cmep::kCmdBlockBase + 0x8120)) / 8;
        const u8 byte = static_cast<u8>(value & 0xFFu);
        // Device identification answers (ASSUMPTION - see the class comment):
        // command 5 reports "type present" and command 8 reports device type 4.
        if (byte == 5) {
            reply_[slot] = 0x01;
        } else if (byte == 8) {
            reply_[slot] = 0x10;
        } else {
            reply_[slot] = 0x00;
        }
    }
}

void CmdBlockDevice::reset() {
    RegisterFile::reset();
    command_.fill(0);
    reply_.fill(0);
}

SysCtlDevice::SysCtlDevice() : RegisterFile("CMeP.SysCtl", cmep::kSysCtlBase, 0x1000) {
    // Bits 0 and 2 are what the second loader's configuration check wants
    // (0x40DB6: ($2 & 5) == 5 and ($2 & 8) == 0); bit 3 must stay clear.  The
    // secure kernel's start-up check (0x80567C) compares 0xE0010004 against
    // 0x80000005 exactly, so bit 31 belongs to the same register value.
    define(0xE0010000, "Cmep system config", 0x80000005);
    define(0xE0010004, "Cmep strap inputs", 0x80000005);
}

CmepFlagsDevice::CmepFlagsDevice() : RegisterFile("CMeP.Flags", cmep::kFlagsBase, 0x1000) {
    define(0xE0020000, "Cmep flags (arm2cry)", 0);
    define(0xE0020004, "Cmep flags+0x4", 0);
    define(0xE0020020, "Cmep bignum sync/work state", 0);
    define(0xE0020024, "Cmep work state+0x4", 0);
}

u64 CmepFlagsDevice::read(u32 address, unsigned size) {
    // 0xE0020020 is the Bignum completion semaphore: the engine raises it when
    // an operation finishes and the loaders clear it by writing the value back.
    return RegisterFile::read(address, size);
}

void CmepFlagsDevice::write(u32 address, unsigned size, u64 value) {
    if (address == kWorkState) {
        // engine_wait (0x4BBB0) and the first loader (0x5CF12/0x5CF14) both spin
        // on a non-zero value and then write it back to release the engine.
        ++sync_pulses_;
        RegisterFile::write(address, size, 0);
        return;
    }
    RegisterFile::write(address, size, value);
}

// ---------------------------------------------------------------------------
// StrapDevice
// ---------------------------------------------------------------------------

StrapDevice::StrapDevice(Bus& bus, CmepBlock& owner)
    : RegisterFile("CMeP.Strap", cmep::kStrapBase, 0x10000), bus_(bus), owner_(owner) {
    for (u32 i = 0; i < kStrapSize; i += 4) {
        define(kStrap + i, i == 0 ? "JP strap / JIG detect (bit0 = 'A')" : "Strap block word",
               0);
    }
    define(kScState, "MMIO_E0064060 (eMMC key index select)", 0);
    define(kScState + 4, "MMIO_E0064064", 0);
    // 0xE00621C0..0xE00621DF: the eight words the CMeP secure kernel reads in its
    // start-up check (0x800FA4 reads 0xE00621C0, C4, C8, CC, D0, D4, D8, DC, keeps
    // the last one and demands 0x01040000 - i.e. "system software 01.04.0000" -
    // at 0x80569A).  Only the last word is compared, so the others stay zero.
    for (u32 offset = 0xE00621C0; offset <= 0xE00621DC; offset += 4) {
        define(offset, offset == 0xE00621DC ? "firmware version (checked = 01.04.0000)" : "unverified",
               0x01040000);
    }
}

u64 StrapDevice::read(u32 address, unsigned size) {
    return RegisterFile::read(address, size);
}

void StrapDevice::write(u32 address, unsigned size, u64 value) {
    RegisterFile::write(address, size, value);
}

void StrapDevice::set_bit0(bool set) {
    u64 value = peek(kStrap);
    if (set) {
        value |= 1u;
    } else {
        value &= ~1ull;
    }
    poke(kStrap, value);
}

std::string StrapDevice::summary() const {
    return format("strap=0x%08X mode='%c' sc_state=0x%08X", static_cast<u32>(peek(kStrap)),
                  bit0() ? 'A' : '!', static_cast<u32>(peek(kScState)));
}

// ---------------------------------------------------------------------------
// EmmcCryptoDevice
// ---------------------------------------------------------------------------

EmmcCryptoDevice::EmmcCryptoDevice()
    : RegisterFile("CMeP.EmmcCrypto", cmep::kEmmcCryptoBase, 0x100) {
    define(kToggle, "EmmcCryptoToggle", 0);
    define(kIndexes, "EmmcCrypto keyring indexes (hi:lo)", 0);
    define(0xE0070010, "EmmcCrypto status", 0);
}

u64 EmmcCryptoDevice::read(u32 address, unsigned size) {
    return RegisterFile::read(address, size);
}

void EmmcCryptoDevice::write(u32 address, unsigned size, u64 value) {
    if (address == kToggle) {
        enabled_ = (value & 1u) != 0;
    }
    RegisterFile::write(address, size, value);
}

std::string EmmcCryptoDevice::summary() const {
    return format("enabled=%d keyring_hi=0x%03X keyring_lo=0x%03X", enabled_ ? 1 : 0, keyring_hi(),
                  keyring_lo());
}

// ---------------------------------------------------------------------------
// GpioDevice
// ---------------------------------------------------------------------------

GpioDevice::GpioDevice() : RegisterFile("CMeP.GPIO", cmep::kGpioBase, 0x100) {
    define(kData, "GPIO data", 0);
    define(kState, "GPIO state/handshake (bit4 = debugger alive)", 0x10);
    define(kSet, "GPIO set", 0);
    define(kClear, "GPIO clear", 0);
    define(0xE20A0010, "GPIO direction", 0);
}

void GpioDevice::reset() {
    RegisterFile::reset();
    handshakes_ = 0;
    // The debugger/JIG line is presented as asserted after power-on.
    poke(kState, 0x10);
}

u64 GpioDevice::read(u32 address, unsigned size) {
    if (address == kState) {
        const u64 value = peek(kState);
        // 0x5E4FC expects bit 4 set, 0x5E560 expects it clear.  Modelling the
        // external agent as "releases the line on the first read" satisfies
        // both loops without an external debugger attached.
        if ((value & 0x10u) != 0) {
            poke(kState, value & ~0x10ull);
            ++handshakes_;
        }
        return value;
    }
    return RegisterFile::read(address, size);
}

void GpioDevice::write(u32 address, unsigned size, u64 value) {
    if (address == kSet) {
        poke(kData, peek(kData) | value);
        RegisterFile::write(address, size, value);
        return;
    }
    if (address == kClear) {
        poke(kData, peek(kData) & ~value);
        RegisterFile::write(address, size, value);
        return;
    }
    RegisterFile::write(address, size, value);
}

// ---------------------------------------------------------------------------
// ScBridgeDevice
// ---------------------------------------------------------------------------

ScBridgeDevice::ScBridgeDevice()
    // The shared SC window is 64 KiB (Ernie's kScWindowSize): the second loader
    // touches 0xE31040A4 and 0xE3105000 while it brings the SC engine up
    // (0x48794, 0x48848), so the CMeP side must cover the whole window instead
    // of the first 16 KiB.
    : RegisterFile("CMeP.SecureCtl", cmep::kScBase, 0x10000) {
    define(kCmd124, "SC 0xE3100124", 0);
    define(kReqA0, "SC 0xE31010A0 request", 0);
    define(kAckA4, "SC 0xE31010A4 ack", 0);
    define(kReg1100, "SC 0xE3101100", 0);
    define(kReg1190, "SC 0xE3101190", 0);
    define(kReq20A0, "SC 0xE31020A0 request", 0);
    define(kAck20A4, "SC 0xE31020A4 ack", 0);
    define(kReg2100, "SC 0xE3102100", 0);
    define(kReg3040, "SC 0xE3103040", 0);
    define(kReg3050, "SC 0xE3103050", 0);
    // Interface straps: the type at 0xE3100000 and the status at 0xE3101000 are
    // fixed, so the CMeP sees them even when the shared Ernie window is not
    // attached (a standalone CmepBlock, or the tests).
    define(zlb::ernie::kScIfType, "SC 0xE3100000 interface type", zlb::ernie::kScIfTypeValue);
    define(zlb::ernie::kScIfStatus, "SC 0xE3101000 interface status", zlb::ernie::kScIfStatusValue);
}

void ScBridgeDevice::reset() {
    RegisterFile::reset();
    requests_ = 0;
    have_a0_ = false;
    have_20a0_ = false;
}

void ScBridgeDevice::attach_shared_sc(Device* sc) {
    // The SC registers are one hardware block: the CMeP posts commands and reads
    // replies through exactly the same registers the ARM polls and Ernie serves.
    // This device used to keep a private copy that only *faked* the acknowledgements,
    // so a command posted from one side was invisible to the other and both ended up
    // in retry loops ("SC commands: 0 served"). Forward to the real block instead.
    shared_sc_ = sc;
}

u64 ScBridgeDevice::read(u32 address, unsigned size) {
    if (shared_sc_ != nullptr && shared_sc_->handles(address)) {
        return shared_sc_->read(address, size);
    }
    if (address == kReqA0 || address == kReq20A0) {
        // The program-and-poll loops (0x5C24A..0x5C254 and 0x5C266..0x5C270)
        // exit as soon as the request register reads non-zero.
        const u64 value = RegisterFile::read(address, size);
        return (value == 0) ? 1u : value;
    }
    if (address == kAckA4 || address == kAck20A4) {
        // ... and the ack registers are polled until they read zero.
        return 0;
    }
    if (address == kReg2100) {
        // 0xE3102100 is the "start/step the SC" strobe.  The first loader writes 1
        // and moves on (0x5C536).  The second loader writes 1 and then polls the
        // register until it *reads back 1* - not 0:
        //     00049536  sw   $2,($3)          ; *(0xE3102100) = 1
        //     00049538  erepeat 0x4953e
        //     0004953E  lw   $2,($3)
        //     00049540  beqi $2,0x1,0x49544   ; proceed once it reads 1
        // Forcing 0 here left the CMeP spinning in that hardware loop forever, so
        // the value the guest wrote is simply held (the SC accepts the strobe).
        return RegisterFile::read(address, size);
    }
    return RegisterFile::read(address, size);
}

void ScBridgeDevice::write(u32 address, unsigned size, u64 value) {
    if (shared_sc_ != nullptr && shared_sc_->handles(address)) {
        // The real block is the single source of truth for the SC state; it also
        // runs the command dispatcher when the command word is written.
        shared_sc_->write(address, size, value);
        if (address == kReqA0 || address == kReq20A0) {
            ++requests_;
            if (address == kReqA0) have_a0_ = true;
            else have_20a0_ = true;
        }
        return;
    }
    if (address == kReqA0 || address == kReq20A0) {
        ++requests_;
        if (address == kReqA0) have_a0_ = true;
        else have_20a0_ = true;
    }
    RegisterFile::write(address, size, value);
}

// ---------------------------------------------------------------------------
// ScXferDevice
// ---------------------------------------------------------------------------

ScXferDevice::ScXferDevice() : Device("CMeP.ScXfer", kBase, kSize) {
    descriptor_.assign(0x2C, 0);
}

void ScXferDevice::reset() {
    descriptor_.assign(0x2C, 0);
    window_base_ = kCmdWindow;
    pending_ = false;
}

u32 ScXferDevice::descriptor_address(u32 channel) {
    // sc_xfer (0x5CF58): `movh $6,0xe0b0` then, when ($1 & 0xFF) != 0,
    // `movh $11,0xe0bf; sll3 $0,$tp,0x10; add3 $6,$0,$11`, so the window is
    // 0xE0B00000 | (channel << 16) | 0x0000FF00 for a 64 KiB page (the first
    // page is 0xE0B0FF00, the response page for channel 1 is 0xE0BFFF00).
    const u32 channel_bits = (channel & 0xFFu) << 16;
    return kCmdWindow + channel_bits + kDescriptorOffset;
}

u64 ScXferDevice::read(u32 address, unsigned size) {
    const u32 offset = address - base();
    u64 value = 0;
    for (unsigned i = 0; i < size; ++i) {
        u8 byte = 0;
        const u32 index = offset + i;
        if (index >= kDescriptorOffset && index < kDescriptorOffset + descriptor_.size()) {
            byte = descriptor_[index - kDescriptorOffset];
        }
        value |= static_cast<u64>(byte) << (8 * i);
    }
    return value;
}

void ScXferDevice::write(u32 address, unsigned size, u64 value) {
    const u32 offset = address - base();
    if (offset >= kDescriptorOffset) {
        const u32 index = offset - kDescriptorOffset;
        if (descriptor_.size() < kDescriptorSize) descriptor_.resize(kDescriptorSize, 0);
        if (index < kDescriptorSize) {
            descriptor_[index] = static_cast<u8>(value);
            pending_ = true;
        }
    }
}

void ScXferDevice::post_descriptor(const std::vector<u8>& bytes, u32 window_base) {
    // The descriptor is queued at +0xB0FF00 of the selected window (sc_xfer
    // builds it in RAM at 0x5EE20 and hands that address to sc_read, so the
    // window copy is the debugger/test entry point).
    descriptor_.assign(kDescriptorSize, 0);
    window_base_ = window_base;
    const size_t n = bytes.size() < descriptor_.size() ? bytes.size() : descriptor_.size();
    std::memcpy(descriptor_.data(), bytes.data(), n);
    pending_ = true;
}

std::string ScXferDevice::summary() const {
    return format("window=0x%08X pending=%d mode=0x%02X channel=%u", window_base_,
                  pending_ ? 1 : 0, descriptor_.empty() ? 0 : descriptor_[0],
                  descriptor_.size() > 0x0E ? descriptor_[0x0E] : 0);
}

void ScXferDevice::describe(std::vector<std::string>& lines) const {
    if (descriptor_.empty()) {
        lines.push_back("SC xfer: no descriptor queued");
        return;
    }
    const u32 chunk = 0;
    lines.push_back(format("SC xfer window     = 0x%08X", window_base_ + chunk));
    lines.push_back(format("SC descriptor mode = 0x%02X channel = %u", descriptor_[0],
                           descriptor_.size() > 0x0E ? descriptor_[0x0E] : 0));
    lines.push_back(format("SC descriptor to   = 0x%08X @0x18 = 0x%08X", descriptor_[4],
                           static_cast<u32>(descriptor_[0x18] | (descriptor_[0x19] << 8) |
                                            (descriptor_[0x1A] << 16) | (descriptor_[0x1B] << 24))));
}

}  // namespace cmep_detail
}  // namespace zlb
