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
//   Each CPU sets bits in its outgoing words and clears acknowledged bits in
//               its incoming words. FW1.04 secure_kernel clears the incoming
//               command with -1 at 0x8003FC and 0x80042E. Native ARM Smsched
//               acknowledges status 101/102 by writing those values to +0,
//               and submits the shared-buffer PA then a separate 1 to +0x10
//               at 0x0051E3C2/0x0051E3C4. These accesses match xyzz/f00d
//               smsched.c set_kernel_enp:
//               https://github.com/xyzz/f00d/blob/master/smsched.c
//               Sender writes must accumulate PA and doorbell; ACK writes must
//               clear only the requested bits. Host set_* helpers use poke.
//   0xE0000040..0x5C are W1C aliases of the eight words at +0..+0x1C.
//               CMeP clears its outgoing auxiliary words through +0x44/48/4C
//               at 0x8005FE..0x80061E and 0x8023F8..0x802416. ARM clears its
//               outgoing auxiliary words through +0x54/58/5C in smsched.c
//               smsched_load_task. These aliases clear from either endpoint.
//   Pending words drive level interrupts. Command channel 0 uses bit 0 as its
//               doorbell, so submitting a PA alone does not interrupt CMeP.
//               ARM status notifications include high-half-only values such
//               as 0x10000 (0x8009CE), so any pending status bit asserts ARM.
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
//   0xE20A0000  GPIO direction register (written with 8 at 0x5E4FA).
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
#include <utility>

#include "common/log.h"
#include "event/providers.h"
#include "hw/cmep/cmep_internal.h"
#include "hw/soc.h"
#include "hw/syscon.h"
#include "loader/loader.h"

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
    for (u32 offset = 0; offset < 0x20u; offset += 4u) {
        define(kCmepToArm + 0x40u + offset, "Mailbox.Clear", 0);
    }
    define(kCmepToDebugger, "MailboxCmepToDebugger", 0);
    define(0xE0000024, "MailboxCmepToDebugger+0x4", 0);
    define(kDebuggerToCmep, "MailboxDebuggerToCmep", 0xFFFFFFFF);
    define(0xE000002C, "MailboxDebuggerToCmep+0x4", 0xFFFFFFFF);
    define(kDebuggerToCmep2, "MailboxDebuggerToCmep2", 0xFFFFFFFF);
    define(0xE0000064, "MailboxDebuggerToCmep2+0x4", 0xFFFFFFFF);
}

void MailboxDevice::set_irq_callbacks(CmepBlock::MailboxIrqCallback to_cmep,
                                     CmepBlock::MailboxIrqCallback to_arm) {
    to_cmep_irq_ = std::move(to_cmep);
    to_arm_irq_ = std::move(to_arm);
    refresh_irqs(true);
}

void MailboxDevice::refresh_irqs(bool force) {
    for (unsigned channel = 0; channel < 4u; ++channel) {
        const u32 command = static_cast<u32>(peek(kArmToCmep + channel * 4u));
        const bool to_cmep = channel == 0u ? (command & 1u) != 0u : command != 0u;
        const bool to_arm = peek(kCmepToArm + channel * 4u) != 0u;
        if (force || to_cmep_levels_[channel] != to_cmep) {
            const bool to_cmep_edge = to_cmep && !to_cmep_levels_[channel];
            to_cmep_levels_[channel] = to_cmep;
            if (to_cmep_irq_) to_cmep_irq_(channel, to_cmep);
            if (to_cmep_edge &&
                events().should_record(EventProvider::Mailbox, EventLevel::Informational,
                                       event_keyword::kInterrupt)) {
                events().event(EventProvider::Mailbox, ev::mailbox::kIrq)
                    .field("channel", (u64)channel)
                    .field("target", (u64)0)
                    .emit();
            }
        }
        if (force || to_arm_levels_[channel] != to_arm) {
            const bool to_arm_edge = to_arm && !to_arm_levels_[channel];
            to_arm_levels_[channel] = to_arm;
            if (to_arm_irq_) to_arm_irq_(channel, to_arm);
            if (to_arm_edge &&
                events().should_record(EventProvider::Mailbox, EventLevel::Informational,
                                       event_keyword::kInterrupt)) {
                events().event(EventProvider::Mailbox, ev::mailbox::kIrq)
                    .field("channel", (u64)channel)
                    .field("target", (u64)1)
                    .emit();
            }
        }
    }
}

void MailboxDevice::reset() {
    RegisterFile::reset();
    debug_posts_ = 0;
    debug_acks_ = 0;
    refresh_irqs();
}

void MailboxDevice::set_arm_to_cmep(u32 value) {
    poke(kArmToCmep, value);
    refresh_irqs();
    if (events().should_record(EventProvider::Mailbox, EventLevel::Informational,
                               event_keyword::kComm)) {
        events().event(EventProvider::Mailbox, ev::mailbox::kHandshake)
            .field("status", (u64)value)
            .field("side", (u64)1)
            .emit();
    }
}

void MailboxDevice::set_cmep_to_arm(u32 value) {
    poke(kCmepToArm, value);
    refresh_irqs();
    if (events().should_record(EventProvider::Mailbox, EventLevel::Informational,
                               event_keyword::kComm)) {
        events().event(EventProvider::Mailbox, ev::mailbox::kHandshake)
            .field("status", (u64)value)
            .field("side", (u64)0)
            .emit();
    }
}

u64 MailboxDevice::read(u32 address, unsigned size) {
    if (address >= kCmepToArm && size <= 8u &&
        static_cast<u64>(address) + size <= static_cast<u64>(kArmToCmep) + 0x10u) {
        u64 value = 0;
        for (unsigned i = 0; i < size; ++i) {
            const u32 byte_address = address + i;
            const unsigned shift = (byte_address & 3u) * 8u;
            const u64 byte = (peek(byte_address & ~3u) >> shift) & 0xFFu;
            value |= byte << (i * 8u);
        }
        if (events().should_record(EventProvider::Mailbox, EventLevel::Verbose, event_keyword::kComm)) {
            events().event(EventProvider::Mailbox, ev::mailbox::kReceive)
                .field("channel", (u64)(((address - kCmepToArm) / 4u) & 3u))
                .field("value", value)
                .emit();
        }
        return value;
    }
    return RegisterFile::read(address, size);
}

bool MailboxDevice::write_data_mailbox(u32 address, unsigned size, u64 value, bool arm_port) {
    if (address < kCmepToArm || size > 8u ||
        static_cast<u64>(address) + size > static_cast<u64>(kArmToCmep) + 0x10u) {
        return false;
    }
    for (unsigned i = 0; i < size; ++i) {
        const u32 byte_address = address + i;
        const u32 word_address = byte_address & ~3u;
        const unsigned shift = (byte_address & 3u) * 8u;
        const u32 mask = static_cast<u32>((value >> (i * 8u)) & 0xFFu) << shift;
        const bool outgoing = (word_address < kArmToCmep) != arm_port;
        set_bits(word_address, mask, outgoing);
    }
    refresh_irqs();
    if (events().should_record(EventProvider::Mailbox, EventLevel::Verbose, event_keyword::kComm)) {
        events().event(EventProvider::Mailbox, ev::mailbox::kSend)
            .field("channel", (u64)0)
            .field("value", value)
            .field("direction", (u64)(arm_port ? 1 : 0))
            .emit();
    }
    return true;
}

bool MailboxDevice::write_clear_alias(u32 address, unsigned size, u64 value) {
    constexpr u32 begin = kCmepToArm + 0x40u;
    constexpr u32 end = begin + 0x20u;
    if (address < begin || size > 8u || static_cast<u64>(address) + size > end) return false;
    for (unsigned i = 0; i < size; ++i) {
        const u32 byte_address = address - 0x40u + i;
        const unsigned shift = (byte_address & 3u) * 8u;
        const u32 mask = static_cast<u32>((value >> (i * 8u)) & 0xFFu) << shift;
        set_bits(byte_address & ~3u, mask, false);
    }
    refresh_irqs();
    return true;
}

void MailboxDevice::write_arm(u32 address, unsigned size, u64 value) {
    if (!write_data_mailbox(address, size, value, true)) write(address, size, value);
}

void MailboxDevice::write(u32 address, unsigned size, u64 value) {
    if (write_clear_alias(address, size, value)) return;
    if (write_data_mailbox(address, size, value, false)) return;
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

u64 SceBlockDevice::traced_accesses_ = 0;

namespace {

/// Little endian readers over a byte buffer.
u32 le_u32(const u8* bytes) {
    return static_cast<u32>(bytes[0]) | (static_cast<u32>(bytes[1]) << 8) |
           (static_cast<u32>(bytes[2]) << 16) | (static_cast<u32>(bytes[3]) << 24);
}

u64 le_u64(const u8* bytes) {
    return static_cast<u64>(le_u32(bytes)) | (static_cast<u64>(le_u32(bytes + 4)) << 32);
}

}  // namespace

bool SceBlockDevice::trace_enabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("ZLB_SCEBLOCK_TRACE");
        return value != nullptr && value[0] != '0';
    }();
    return enabled;
}

u64 SceBlockDevice::read_register(u32 address, unsigned size) {
    const u32 offset = address - kBase;
    u64 value = 0;
    for (unsigned i = 0; i < size && offset + i < kSize; ++i) {
        value |= static_cast<u64>(memory_[offset + i]) << (8 * i);
    }
    return value;
}

void SceBlockDevice::write_register(u32 address, unsigned size, u64 value) {
    const u32 offset = address - kBase;
    for (unsigned i = 0; i < size && offset + i < kSize; ++i) {
        memory_[offset + i] = static_cast<u8>((value >> (8 * i)) & 0xFF);
    }
}

void SceBlockDevice::process_job() {
    job_done_ = true;
    ++jobs_;
    if (trace_enabled()) {
        ZLB_LOG_INFO("sceblock", "job %llu fired (keys=%s)", static_cast<unsigned long long>(jobs_),
                     keys_ != nullptr ? "present" : "missing");
    }
    if (keys_ == nullptr) return;

    const u64 header_length = le_u64(prologue_.data() + 0x10);
    if (header_length <= 0x30 || header_length > kSize || (header_length % 4) != 0) {
        ZLB_LOG_WARN("sceblock", "job %llu: implausible header_length 0x%llX",
                     static_cast<unsigned long long>(jobs_),
                     static_cast<unsigned long long>(header_length));
        return;
    }
    const size_t payload = static_cast<size_t>(header_length) - 0x30;

    // The loader posted SELF[0..0x30) as the prologue and SELF[0x30..) into the
    // window; together they are the SELF header area the engine has to decrypt.
    std::vector<u8> image(static_cast<size_t>(header_length), 0);
    std::copy(prologue_.begin(), prologue_.end(), image.begin());
    for (size_t i = 0; i < payload; ++i) {
        image[0x30 + i] = memory_[i];
    }

    std::string why;
    if (!sce_decrypt_metadata_in_place(image, *keys_, &why)) {
        ZLB_LOG_INFO("sceblock", "job %llu: metadata decryption failed (%s)",
                     static_cast<unsigned long long>(jobs_), why.c_str());
        return;
    }

    for (size_t i = 0; i < payload; ++i) {
        memory_[i] = image[0x30 + i];
    }
    ZLB_LOG_INFO("sceblock",
                 "job %llu: SELF header area decrypted (header_length 0x%llX, metadata at 0x%X), "
                 "plaintext exposed in the window",
                 static_cast<unsigned long long>(jobs_),
                 static_cast<unsigned long long>(header_length), le_u32(prologue_.data() + 0x0C));
}

u64 SceBlockDevice::read(u32 address, unsigned size) {
    if (payload_written_ && !job_done_) process_job();
    const u64 value = RegisterFile::read(address, size);
    if (trace_enabled() && traced_accesses_ < 4000) {
        ++traced_accesses_;
        ZLB_LOG_INFO("sceblock", "r +0x%04X size=%u -> 0x%llX", address - kBase, size,
                     static_cast<unsigned long long>(value));
    }
    return value;
}

void SceBlockDevice::write(u32 address, unsigned size, u64 value) {
    const u32 offset = address - kBase;
    // Phase 1 opens with the SCE magic; the prologue is the twelve words that
    // follow (SCE header + the first two u64 of the SELF header = SELF[0..0x30)).
    if (size == 4 && offset == 0 && (value & 0xFFFFFFFFull) == 0x00454353ull) {
        prologue_.fill(0);
        // The magic is the first prologue word, so it counts towards the twelve.
        for (unsigned byte = 0; byte < 4; ++byte) {
            prologue_[byte] = static_cast<u8>((value >> (8 * byte)) & 0xFF);
        }
        prologue_words_ = 1;
        prologue_seen_ = true;
        payload_written_ = false;
        job_done_ = false;
    } else if (prologue_seen_ && prologue_words_ < 12 && offset < 0x30) {
        for (unsigned byte = 0; byte < 4; ++byte) {
            prologue_[offset + byte] = static_cast<u8>((value >> (8 * byte)) & 0xFF);
        }
        ++prologue_words_;
    } else if (prologue_seen_) {
        // Everything after the prologue is the header area of the job.
        payload_written_ = true;
    }

    if (trace_enabled() && traced_accesses_ < 4000) {
        ++traced_accesses_;
        ZLB_LOG_INFO("sceblock", "w +0x%04X size=%u <- 0x%llX", offset, size,
                     static_cast<unsigned long long>(value));
    }
    RegisterFile::write(address, size, value);
}

SceBlockDevice::SceBlockDevice(SceKeys* keys)
    : RegisterFile("CMeP.SceBlock", kBase, kSize) {
    // The engine's key table: the job the second loader posts is the decryption
    // of the SELF header area (see process_job()).
    keys_ = keys;
    // The prologue the loader checks for (0x4A774..0x4A796) is magic "SCE\0",
    // word 1 == 3 and bit 0x40 of byte 8.  The block starts as plain scratch
    // memory - the loader fills it itself and reads the answer back - and the
    // header is only pre-set until the first write, so that an exchange which
    // never writes the magic still answers with a valid block.
    define(kBase + 0x00, "SCE block magic", 0x00454353u);   // "SCE\0"
    define(kBase + 0x04, "SCE block status", 0x00000003u);
    define(kBase + 0x08, "SCE block flags", 0x00010140u);
    for (u32 offset = 0x0C; offset < 0x30; offset += 4) {
        define(kBase + offset, format("SCE block +0x%02X", offset), 0);
    }
    // The window itself is byte addressed memory; the pre-set prologue lives there.
    memory_.fill(0);
    const u32 preset[3] = {0x00454353u, 0x00000003u, 0x00010140u};
    for (u32 word = 0; word < 3; ++word) {
        for (unsigned byte = 0; byte < 4; ++byte) {
            memory_[word * 4 + byte] = static_cast<u8>((preset[word] >> (8 * byte)) & 0xFF);
        }
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

namespace {
constexpr u32 kConsoleIdentityBase = 0xE0062120u;
constexpr u32 kConsoleIdentitySlotSize = 32u;
constexpr u32 kDramCapacityBase = 0xE0062260u;

bool public_platform_slot(u32 address, unsigned size) {
    if (size > sizeof(u64)) return false;
    const u64 end = static_cast<u64>(address) + size;
    return (address >= kConsoleIdentityBase && end <= kConsoleIdentityBase + 32u) ||
           (address >= kDramCapacityBase && end <= kDramCapacityBase + 32u);
}
}

StrapDevice::StrapDevice(Bus& bus, CmepBlock& owner)
    : RegisterFile("CMeP.Strap", cmep::kStrapBase, 0x10000), bus_(bus), owner_(owner) {
    for (u32 i = 0; i < kStrapSize; i += 4) {
        define(kStrap + i, i == 0 ? "JP strap / JIG detect (bit0 = 'A')" : "Strap block word",
               0);
    }
    define(kScState, "MMIO_E0064060 (eMMC key index select)", 0);
    define(kScState + 4, "MMIO_E0064064", 0);
    // FW1.04 0x805774 reads slot 0x509 via 0x80571A: E0058000 + (509<<5)
    // = E0062120. Its first 16 bytes are the console identity used by native
    // 0x804B58 to classify the board; an all-zero product is rightly rejected.
    // Supply the public PCH-1001 prefix for the existing USS-1001 retail model:
    // 00 00 00 01 01 04 00 10. This is modeled platform input, not a dumped
    // console identity or a console key. The unique remainder is unavailable
    // and stays zero. Public prefix evidence: https://github.com/Freakler/vita-ConsoleID
    for (u32 word = 0; word < kConsoleIdentitySlotSize / 4u; ++word) {
        const char* kind = word < 2u ? "modeled public PCH-1001 prefix" :
                           (word < 4u ? "unique identity unavailable" : "slot padding");
        const u32 value = word == 0u ? 0x01000000u : (word == 1u ? 0x10000401u : 0u);
        define(kConsoleIdentityBase + word * 4u,
               format("Console identity slot 0x509 word %u (%s)", word, kind), value);
    }
    // Native FW1.04 kprx 0x80E59E..0x80E5D8 reads slot 0x513's first four
    // bytes as a little-endian DRAM aperture size, then validates mailbox
    // buffers against [0x40000000, 0x40000000+size) with carry accounting.
    // This is the existing modeled board capacity, shared with Vita's DRAM
    // aliases and SceKblParam+0x64, not dumped per-console configuration.
    for (u32 word = 0; word < 8u; ++word) {
        define(kDramCapacityBase + word * 4u,
               word == 0u ? "DRAM capacity slot 0x513 (modeled board byte size)" :
                            "DRAM capacity slot 0x513 (unavailable padding)",
               word == 0u ? kermit::kScuSize : 0u);
    }
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
    if (public_platform_slot(address, size)) {
        // Identity copies use words and the native capacity validator uses
        // bytes. Both must see the same explicitly modeled platform input.
        u64 value = 0;
        for (unsigned i = 0; i < size; ++i) {
            const u32 byte_address = address + i;
            const u64 byte = (peek(byte_address & ~3u) >> ((byte_address & 3u) * 8u)) & 0xFFu;
            value |= byte << (i * 8u);
        }
        return value;
    }
    return RegisterFile::read(address, size);
}

void StrapDevice::write(u32 address, unsigned size, u64 value) {
    if (public_platform_slot(address, size)) {
        // Keep explicit overrides verbatim. Malformed/unknown products must
        // remain visible to the native classifier and aperture validator.
        for (unsigned i = 0; i < size; ++i) {
            const u32 byte_address = address + i;
            const unsigned shift = (byte_address & 3u) * 8u;
            const u64 byte = (value >> (i * 8u)) & 0xFFu;
            poke(byte_address & ~3u, (peek(byte_address & ~3u) & ~(0xFFull << shift)) |
                                      (byte << shift));
        }
        return;
    }
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
    const u32 product_word = static_cast<u32>(peek(kConsoleIdentityBase + 4u));
    const u32 product = ((product_word & 0xFFu) << 8u) | ((product_word >> 8u) & 0xFFu);
    return format("strap=0x%08X mode='%c' sc_state=0x%08X console_product=0x%04X "
                  "identity=modeled public platform (not a console dump) dram_capacity=0x%08X",
                  static_cast<u32>(peek(kStrap)), bit0() ? 'A' : '!',
                  static_cast<u32>(peek(kScState)), product,
                  static_cast<u32>(peek(kDramCapacityBase)));
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

// Supplied FW1.04 Lowio PortRead810027A6..27B0 reads output+34 when
// direction+00 selects output, otherwise input+04. Checkpoints use SET08 /
// CLEAR0C. The reached Syscon driver sets pin4 mode3 at+14=300, unmasks gate0
// at+1C=0, and registers GPIO248/sub4. Other gates were undefined in the old
// shim, not observed unmasked. Primary hardware-tested register encoding:
// https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/gpio.c
// https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/include/gpio.h
//
// Bounded model: only physical input4 falling/mode3 generates edges. Reset
// masks all gates; pin levels have no fabricated pull-up/ready source. Pending
// edges fan out into all five status latches even while masked; this retention
// and fanout are explicit model choices, not captured electrical reset facts.
// Each parent248+gate is separately mask-gated, so the reached setup raises248
// only. Unsupported mode/slave work never receives a completion here.
GpioDevice::GpioDevice() : Device("CMeP.GPIO", cmep::kGpioBase, 0x1000) {
    const std::pair<u32, const char*> names[] = {
        {0x00, "DIRECTION"}, {0x04, "INPUT"}, {0x08, "SET"}, {0x0C, "CLEAR"},
        {0x10, "OPAQUE_10"}, {0x14, "MODE_0_15"}, {0x18, "MODE_16_31"},
        {0x1C, "MASK0"}, {0x20, "MASK1"}, {0x24, "MASK2"}, {0x28, "MASK3"}, {0x2C, "MASK4"},
        {0x34, "OUTPUT"}, {0x38, "STATUS0"}, {0x3C, "STATUS1"}, {0x40, "STATUS2"},
        {0x44, "STATUS3"}, {0x48, "STATUS4"},
    };
    for (const auto& name : names) register_name_entry(base_ + name.first, name.second);
    reset();
}

bool GpioDevice::known_offset(u32 offset) const {
    return (offset & 3u) == 0 && offset <= 0x48 && offset != 0x30;
}

u32 GpioDevice::raw_input() const {
    return external_input_ | (legacy_jig_enabled_ && jig_asserted_ ? 0x10u : 0u);
}

u32 GpioDevice::sampled_input() const {
    return (output_latch() & direction()) | (raw_input() & ~direction());
}

u32 GpioDevice::word_value(u32 offset) const {
    return offset == 4 ? sampled_input() : registers_[offset / 4];
}

void GpioDevice::refresh_irqs() {
    for (unsigned gate = 0; gate < kGateCount; ++gate) {
        const u32 pending = registers_[(0x38 + gate * 4) / 4];
        const u32 mask = registers_[(0x1C + gate * 4) / 4];
        const bool level = (pending & ~mask) != 0;
        if (level == irq_states_[gate]) continue;
        irq_states_[gate] = level;
        if (irq_) irq_(kParentIrqBase + gate, level);
    }
}

void GpioDevice::set_irq_callback(std::function<void(u32, bool)> callback) {
    irq_ = std::move(callback);
    if (irq_) {
        for (unsigned gate = 0; gate < kGateCount; ++gate) irq_(kParentIrqBase + gate, irq_states_[gate]);
    }
}

void GpioDevice::set_output_callback(std::function<void(u32, u32)> callback) {
    output_callback_ = std::move(callback);
    if (output_callback_) output_callback_(direction(), output_latch());
}

void GpioDevice::input_changed(u32 before) {
    const u32 falling = before & ~raw_input() & ~direction();
    if ((falling & 0x10u) == 0) return;
    const unsigned pin4_mode = (registers_[0x14 / 4] >> 8) & 3u;
    if (pin4_mode == 3) {
        ++falling_edges_;
        // Masking affects delivery, not physical edge capture. All-gate fanout
        // is an explicit model choice; all-masked reset bounds unused parents.
        for (unsigned gate = 0; gate < kGateCount; ++gate) registers_[(0x38 + gate * 4) / 4] |= 0x10u;
    } else {
        // Retain readable mode snapshots, but never invent unmodeled level,
        // rising, capture or other-pin events to satisfy a guest wait.
        ++unsupported_edges_;
    }
    refresh_irqs();
}

void GpioDevice::set_external_input(u32 mask, u32 levels) {
    const u32 before = raw_input();
    external_input_ = (external_input_ & ~mask) | (levels & mask);
    input_changed(before);
}

bool GpioDevice::set_legacy_jig_enabled(bool enabled) {
    if (enabled && native_phase_) return false;
    const u32 before = raw_input();
    legacy_jig_enabled_ = enabled;
    jig_asserted_ = enabled;
    input_changed(before);
    return true;
}

void GpioDevice::enter_native_phase() {
    native_phase_ = true;
    set_legacy_jig_enabled(false);
    if (output_callback_) output_callback_(direction(), output_latch());
}

void GpioDevice::reset() {
    registers_.fill(0);
    for (unsigned gate = 0; gate < kGateCount; ++gate) registers_[(0x1C + gate * 4) / 4] = 0xFFFFFFFF;
    external_input_ = 0;
    handshakes_ = falling_edges_ = unsupported_edges_ = 0;
    legacy_jig_enabled_ = jig_asserted_ = native_phase_ = false;
    refresh_irqs(); // deassert live parents; retain installed board callbacks
    if (board_reset_) board_reset_(); // modes are reset before wire disconnect; no reset edge
    if (output_callback_) output_callback_(direction(), output_latch());
}

u64 GpioDevice::read(u32 address, unsigned size) {
    u64 value = 0;
    for (unsigned i = 0; i < size && i < 8; ++i) {
        const u64 current = static_cast<u64>(address) + i;
        u32 byte = 0xFF;
        if (current >= base_ && current < static_cast<u64>(base_) + size_) {
            const u32 offset = static_cast<u32>(current - base_);
            if (known_offset(offset & ~3u)) byte = (word_value(offset & ~3u) >> ((offset & 3u) * 8)) & 0xFFu;
        }
        value |= static_cast<u64>(byte) << (i * 8);
    }
    return value;
}

void GpioDevice::write(u32 address, unsigned size, u64 value) {
    const bool was_output3_high = (direction() & output_latch() & 8u) != 0;
    u32 supplied_set = 0, supplied_clear = 0;
    bool set_written = false, clear_written = false;
    for (unsigned i = 0; i < size && i < 8; ++i) {
        const u64 current = static_cast<u64>(address) + i;
        if (current < base_ || current >= static_cast<u64>(base_) + size_) continue;
        const u32 offset = static_cast<u32>(current - base_);
        const u32 word = offset & ~3u;
        if (!known_offset(word) || word == 4 || word == 0x34) continue; // hardware input/output readback
        const u32 shift = (offset & 3u) * 8;
        const u32 supplied = static_cast<u32>((value >> (i * 8)) & 0xFFu) << shift;
        if (word == 8) { supplied_set |= supplied; set_written = true; }
        else if (word == 0xC) { supplied_clear |= supplied; clear_written = true; }
        else if (word >= 0x38) registers_[word / 4] &= ~supplied; // only supplied W1C bits
        else registers_[word / 4] = (registers_[word / 4] & ~(0xFFu << shift)) | supplied;
    }
    // SET/CLEAR reads expose the last supplied command mask as a diagnostic
    // snapshot; unwritten lanes are never reapplied to the output latch.
    if (set_written) {
        registers_[8 / 4] = supplied_set;
        registers_[0x34 / 4] |= supplied_set;
    }
    if (clear_written) {
        registers_[0xC / 4] = supplied_clear;
        registers_[0x34 / 4] &= ~supplied_clear;
    }
    const bool output3_high = (direction() & output_latch() & 8u) != 0;
    if (legacy_jig_enabled_ && jig_asserted_ && !was_output3_high && output3_high) {
        const u32 before = raw_input();
        jig_asserted_ = false;
        ++handshakes_;
        input_changed(before); // explicit development peer release, never read-driven
    }
    refresh_irqs();
    if (output_callback_) output_callback_(direction(), output_latch());
}

void GpioDevice::enumerate_registers(std::vector<RegisterInfo>& out) const {
    for (const auto& entry : names_) {
        const u32 offset = entry.first - base_;
        out.push_back({entry.first, entry.second, offset >= 0x1C && offset <= 0x2C ? 0xFFFFFFFFu : 0u, 4});
    }
}

bool GpioDevice::peek_register(const std::string& name, u64& out) const {
    if (name == "FALLING_EDGES") { out = falling_edges_; return true; }
    if (name == "UNSUPPORTED_EDGES") { out = unsupported_edges_; return true; }
    if (name == "JIG_ENABLED") { out = legacy_jig_enabled_; return true; }
    if (name == "NATIVE_PHASE") { out = native_phase_; return true; }
    for (const auto& entry : names_) {
        if (entry.second == name) { out = word_value(entry.first - base_); return true; }
    }
    return false;
}

bool GpioDevice::poke_register(const std::string& name, u64 value) {
    for (const auto& entry : names_) {
        if (entry.second == name) { write(entry.first, 4, value); return true; }
    }
    return false;
}

std::string GpioDevice::summary() const {
    return format("direction=%08X output=%08X input=%08X edges=%llu unsupported=%llu JIG=%s releases=%llu",
                  direction(), output_latch(), sampled_input(), static_cast<unsigned long long>(falling_edges_),
                  static_cast<unsigned long long>(unsupported_edges_), legacy_jig_enabled_ ? "enabled" : "absent",
                  static_cast<unsigned long long>(handshakes_));
}

void GpioDevice::describe(std::vector<std::string>& lines) const {
    lines.push_back("  " + summary());
    for (unsigned gate = 0; gate < kGateCount; ++gate) {
        lines.push_back(format("    gate%u IRQ%u mask=%08X pending=%08X level=%u", gate, kParentIrqBase + gate,
                               registers_[(0x1C + gate * 4) / 4], registers_[(0x38 + gate * 4) / 4], irq_states_[gate]));
    }
}

// ---------------------------------------------------------------------------
// ScBridgeDevice
// ---------------------------------------------------------------------------

ScBridgeDevice::ScBridgeDevice()
    // The shared SC window is 128 KiB (Ernie's kScWindowSize): the second loader
    // touches 0xE31040A4, 0xE3105000 and 0xE3110C00 while it brings the SC engine
    // up (0x48794, 0x48848, 0x4864C), so the CMeP side must cover the whole
    // window - a narrower one left 0xE3110C00 unmapped, its write-then-poll loop
    // read back 0 and the loader spun there forever.
    : RegisterFile("CMeP.SecureCtl", cmep::kScBase, cmep::kScWindowSize) {
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
    events_c0_ = 1;   // one SC event is pending for the CMeP from power-on
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
    // 0xE31000C0 is read per side (see the note on events_c0_): the CMeP's own
    // pending SC event is bit 0, and the shared window serves the ARM's bits.
    if (address == kEventsC0) {
        const u32 value = events_c0_;
        return size >= 4 ? value : (value & ((1u << (size * 8)) - 1u));
    }
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
    if (address == kEventsC0) {
        events_c0_ &= ~static_cast<u32>(value);   // write-1-to-clear
        return;
    }
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

const char* ScXferDevice::register_name(u32 address) const {
    // The window is 0xE0B00000 | (channel << 16); the only populated bytes are
    // the 0x2C-byte descriptor sc_xfer leaves at +0xFF00 of the selected window
    // (see the layout comment on the class in cmep_internal.h).
    if (address < kBase || address >= kBase + kSize) return nullptr;
    const u32 offset = address - kBase;
    const u32 in_window = offset & 0xFFFFu;
    if (in_window < kDescriptorOffset) {
        return in_window == 0 ? "SC window base" : nullptr;
    }
    const u32 field = in_window - kDescriptorOffset;
    if (field == 0x00) return "SC descriptor mode/flags";
    if (field >= 0x04 && field < 0x08) return "SC descriptor timeout";
    if (field >= 0x08 && field < 0x0C) return "SC descriptor timeout2";
    if (field == 0x0E) return "SC descriptor channel";
    if (field >= 0x18 && field < 0x1C) return "SC descriptor window base";
    if (field >= 0x24 && field < 0x28) return "SC descriptor control pointer";
    if (field < kDescriptorSize) return "SC descriptor byte";
    return nullptr;
}

void ScXferDevice::enumerate_registers(std::vector<RegisterInfo>& out) const {
    // Every SC transfer window: entries for the base, the mode/channel bytes and
    // the two 32-bit pointers the engine actually consumes.
    static const u32 kChannels[] = {0x00, 0x01};
    static const u32 kFields[] = {0x00, 0x04, 0x08, 0x0E, 0x18, 0x24};
    for (u32 channel : kChannels) {
        const u32 window = kCmdWindow + (channel << 16);
        RegisterInfo base;
        base.address = window;
        base.name = "SC window base";
        out.push_back(base);
        for (u32 field : kFields) {
            const u32 address = window + kDescriptorOffset + field;
            const char* name = register_name(address);
            RegisterInfo info;
            info.address = address;
            info.name = name ? name : "SC descriptor";
            info.width = (field == 0x00 || field == 0x0E) ? 1u : 4u;
            out.push_back(std::move(info));
        }
    }
}

void ScXferDevice::post_descriptor(const std::vector<u8>& bytes, u32 window_base) {    // The descriptor is queued at +0xB0FF00 of the selected window (sc_xfer
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

// ---------------------------------------------------------------------------
// Save states
// ---------------------------------------------------------------------------
// Every device writes its mutable fields in one fixed order. Register-file
// devices start with RegisterFile::save_state() (the register image holds the
// pending/arbitration words and the SC register bank), then add whatever their
// overridden read/write keep outside that image. Pointers, references and
// std::function wiring are rebuilt by the machine and stay in the build.

void MailboxDevice::save_state(StateWriter& writer) const {
    RegisterFile::save_state(writer);
    // Per-direction IRQ latches: they decide whether the next refresh_irqs()
    // raises an edge, so they are state, not wiring.
    writer.fixed(to_cmep_levels_, [&](bool level) { writer.put_bool(level); });
    writer.fixed(to_arm_levels_, [&](bool level) { writer.put_bool(level); });
    writer.put_u64(debug_posts_);
    writer.put_u64(debug_acks_);
}

void MailboxDevice::load_state(StateReader& reader) {
    RegisterFile::load_state(reader);
    reader.fixed(to_cmep_levels_, [&](bool& level) { level = reader.get_bool(); });
    reader.fixed(to_arm_levels_, [&](bool& level) { level = reader.get_bool(); });
    debug_posts_ = reader.get_u64();
    debug_acks_ = reader.get_u64();
    // The machine's interrupt lines are not part of this stream, so re-announce
    // the restored levels once the read is known good. The call is idempotent
    // and keeps the live lines in step with the restored latches.
    if (reader.ok()) refresh_irqs(true);
}

void EmmcCryptoDevice::save_state(StateWriter& writer) const {
    RegisterFile::save_state(writer);
    writer.put_bool(enabled_);
}

void EmmcCryptoDevice::load_state(StateReader& reader) {
    RegisterFile::load_state(reader);
    enabled_ = reader.get_bool();
}

void CmdBlockDevice::save_state(StateWriter& writer) const {
    RegisterFile::save_state(writer);
    writer.fixed(command_, [&](u8 value) { writer.put_u8(value); });
    writer.fixed(reply_, [&](u8 value) { writer.put_u8(value); });
}

void CmdBlockDevice::load_state(StateReader& reader) {
    RegisterFile::load_state(reader);
    reader.fixed(command_, [&](u8& value) { value = reader.get_u8(); });
    reader.fixed(reply_, [&](u8& value) { value = reader.get_u8(); });
}

void SceBlockDevice::save_state(StateWriter& writer) const {
    RegisterFile::save_state(writer);
    writer.begin("SceBlock.memory");
    state_write_pages(writer, memory_.data(), memory_.size());
    writer.end();
    writer.fixed(prologue_, [&](u8 byte) { writer.put_u8(byte); });
    writer.put_u32(prologue_words_);
    writer.put_bool(prologue_seen_);
    writer.put_bool(payload_written_);
    writer.put_bool(job_done_);
    writer.put_u64(jobs_);
}

void SceBlockDevice::load_state(StateReader& reader) {
    RegisterFile::load_state(reader);
    reader.begin("SceBlock.memory");
    state_read_pages(reader, memory_.data(), memory_.size());
    reader.end();
    reader.fixed(prologue_, [&](u8& byte) { byte = reader.get_u8(); });
    prologue_words_ = reader.get_u32();
    prologue_seen_ = reader.get_bool();
    payload_written_ = reader.get_bool();
    job_done_ = reader.get_bool();
    jobs_ = reader.get_u64();
}

void CmepFlagsDevice::save_state(StateWriter& writer) const {
    RegisterFile::save_state(writer);
    writer.put_u64(sync_pulses_);
}

void CmepFlagsDevice::load_state(StateReader& reader) {
    RegisterFile::load_state(reader);
    sync_pulses_ = reader.get_u64();
}

void GpioDevice::save_state(StateWriter& writer) const {
    writer.fixed(registers_, [&](u32 value) { writer.put_u32(value); });
    writer.fixed(irq_states_, [&](bool state) { writer.put_bool(state); });
    writer.put_u32(external_input_);
    writer.put_u64(handshakes_);
    writer.put_u64(falling_edges_);
    writer.put_u64(unsupported_edges_);
    writer.put_bool(legacy_jig_enabled_);
    writer.put_bool(jig_asserted_);
    writer.put_bool(native_phase_);
}

void GpioDevice::load_state(StateReader& reader) {
    reader.fixed(registers_, [&](u32& value) { value = reader.get_u32(); });
    reader.fixed(irq_states_, [&](bool& state) { state = reader.get_bool(); });
    external_input_ = reader.get_u32();
    handshakes_ = reader.get_u64();
    falling_edges_ = reader.get_u64();
    unsupported_edges_ = reader.get_u64();
    legacy_jig_enabled_ = reader.get_bool();
    jig_asserted_ = reader.get_bool();
    native_phase_ = reader.get_bool();
    // The board's IRQ gate state is not part of the stream (the callbacks are
    // wiring): re-announce the restored gate levels so an IRQ line that was
    // asserted when the state was taken is asserted again after the load.
    if (reader.ok() && irq_) {
        for (unsigned gate = 0; gate < kGateCount; ++gate) {
            irq_(kParentIrqBase + gate, irq_states_[gate]);
        }
    }
    // The output notification is a pure function of the restored registers and
    // is idempotent, so it is safe to re-drive the board peer (Kermit.Spi0's
    // syscon GPIO state) after a load.
    if (reader.ok() && output_callback_) output_callback_(direction(), output_latch());
}

void ScBridgeDevice::save_state(StateWriter& writer) const {
    RegisterFile::save_state(writer);
    writer.put_u64(requests_);
    writer.put_bool(have_a0_);
    writer.put_bool(have_20a0_);
    writer.put_u32(events_c0_);
}

void ScBridgeDevice::load_state(StateReader& reader) {
    RegisterFile::load_state(reader);
    requests_ = reader.get_u64();
    have_a0_ = reader.get_bool();
    have_20a0_ = reader.get_bool();
    events_c0_ = reader.get_u32();
}

void ScXferDevice::save_state(StateWriter& writer) const {
    writer.list(descriptor_, [&](u8 byte) { writer.put_u8(byte); });
    writer.put_u32(window_base_);
    writer.put_bool(pending_);
}

void ScXferDevice::load_state(StateReader& reader) {
    reader.list(descriptor_, [&](u8& byte) { byte = reader.get_u8(); });
    window_base_ = reader.get_u32();
    pending_ = reader.get_bool();
}

}  // namespace cmep_detail
}  // namespace zlb
