// zeliboba - private device classes of the CMeP ("F00D") block.
//
// Everything in this header is an implementation detail of cmep_block.cpp and
// the device translation units (keyring.cpp, bigmac.cpp, bignum.cpp,
// mailbox.cpp).  The public interface is src/hw/cmep.h.
//
// ---------------------------------------------------------------------------
// Register protocol - evidence
// ---------------------------------------------------------------------------
// The addresses below were reverse engineered from the annotated first-loader
// listing dumps/bootrom_analysis/vita_prototype_bootrom.annotated.asm.  Every
// claim carries the instruction address of the code that proves it.
//
//   bigmac_cmd            0x5CD20  writes +0x04 <- $1, +0x00 <- $2,
//                                  +0x08 <- $3, +0x14 <- stack[0] (only when
//                                  non-zero), +0x10 <- $4, +0x0C <- $5,
//                                  +0x1C <- 1, then polls +0x24 for bit 0 to
//                                  clear and returns -1 when (+0x24 & 0x78000).
//   keyring_write1        0x5CCA8  cmd = (0x0301 << 16) | 8   (keyring 8)
//   keyring_write2        0x5CCC0  cmd = (0x010A << 16) | $1  (keyring index)
//   keyring_read1         0x5CCD8  cmd = (0x030A << 16) | $1
//   keyring_read2         0x5CCF0  cmd = (0x0033 << 16) | 0
//   keyring_op_5CD06      0x5CD06  cmd = 12, trigger payload = $1 (0x1C000-...)
//   bigmac_rng            0x5CDD6  +0x00/+0x04/+0x08/+0x0C/+0x14 <- 0 (a plain
//                                  quiesce: no command, no start bit)
#pragma once

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "bus/device.h"
#include "hw/cmep.h"
#include "loader/keys.h"

namespace zlb {

class CmepBlock;
class ErnieBlock;

namespace cmep_detail {

// ---------------------------------------------------------------------------
// Small helpers shared by the devices
// ---------------------------------------------------------------------------

/// 64-bit rotate-left, used by the SHA-1 message schedule.
inline u32 rol32(u32 value, unsigned bits) { return (value << bits) | (value >> (32 - bits)); }
/// 64-bit rotate-right, used by SHA-256.
inline u32 ror32(u32 value, unsigned bits) { return (value >> bits) | (value << (32 - bits)); }

// ---------------------------------------------------------------------------
// Development substitutions in the decrypted second loader
// ---------------------------------------------------------------------------

/// One entry of the patch table `apply_development_substitutions()` writes into
/// the decrypted second loader (see bigmac.cpp for the reasoning behind every
/// entry).  The table is exposed so that a test can prove each entry lands
/// *inside* the staged image: round 91 found an entry written as an absolute
/// address (`0x40850` instead of offset `0x850`), which the loop turned into a
/// write to `0x80850` - unmapped memory, so the "substitution" was a no-op that
/// still announced itself in the log.
struct SecondLoaderPatch {
    u32 offset;      ///< from the staged image base (0x40000 == cmep::kRamBase)
    u32 length;      ///< bytes touched
    u32 word;        ///< `forced` entries install this instruction, little-endian
    bool forced;     ///< false: the bytes are cleared to zero (a disabled check)
    const char* what;
};

/// The patch table itself, so tests can check it without applying it.
const SecondLoaderPatch* second_loader_patches(size_t& count);

/// The boot chain hashes 0x16600 bytes of the staged second loader (the SLB2
/// entry is 93184 bytes); every patch has to stay below this bound.
inline constexpr u32 kSecondLoaderStagedBytes = 0x16600;

/// Deterministic 64-bit LCG.  The hardware RNG is seeded from a fixed constant
/// so that a given run always produces the same stream (documented in
/// bigmac.cpp).
struct DeterministicRng {
    u64 state = 0;
    void seed(u64 value) { state = value | 1u; }
    u32 next() {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        u32 v = static_cast<u32>(state >> 32);
        v ^= v >> 15;
        v *= 0x2545F491u;
        return v ^ (v >> 13);
    }
};

// ---------------------------------------------------------------------------
// 0xE0000000 - ARM <-> CMeP and debug mailboxes
// ---------------------------------------------------------------------------

/// The mailbox block.  Register names and the ack protocol come from
/// ANALYSIS.md appendix A and mailbox_debug_sc (0x5E4E4).
class MailboxDevice : public RegisterFile {
public:
    MailboxDevice(Bus& bus, CmepBlock& owner);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;

    u32 cmep_to_arm() const { return static_cast<u32>(peek(kCmepToArm)); }
    u32 arm_to_cmep() const { return static_cast<u32>(peek(kArmToCmep)); }
    void set_arm_to_cmep(u32 value) { poke(kArmToCmep, value); }
    void set_cmep_to_arm(u32 value) { poke(kCmepToArm, value); }

    u64 debug_posts() const { return debug_posts_; }
    u64 debug_acks() const { return debug_acks_; }
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    /// 0xE0000000 CMeP -> ARM status.
    static constexpr u32 kCmepToArm = 0xE0000000;
    /// 0xE0000010 ARM -> CMeP command / physical address (bit 0 = present).
    static constexpr u32 kArmToCmep = 0xE0000010;
    static constexpr u32 kCmepToDebugger = 0xE0000020;
    static constexpr u32 kDebuggerToCmep = 0xE0000028;
    static constexpr u32 kDebuggerToCmep2 = 0xE0000060;

private:
    Bus& bus_;
    CmepBlock& owner_;
    u64 debug_posts_ = 0;
    u64 debug_acks_ = 0;
};

// ---------------------------------------------------------------------------
// 0xE0010000 - CMeP system / strap inputs
// ---------------------------------------------------------------------------

/// Read-only configuration inputs the CMeP second loader validates before it
/// hands anything to the ARM:
///
///   0x40DB6  `lw $2,(0xE0010004)`, then `($2 & 5) == 5` and `($2 & 8) == 0`
///            have to hold or the loader aborts (error 0x800F002F) and reports
///            the 0x500 failure code to the ARM mailbox.
///
/// The model answers with 0x00000005: the two "present" bits the check asks for
/// and nothing else. (ASSUMPTION: the individual bits are not named by any
/// workspace material; the constraint comes from the check itself.)
class SysCtlDevice : public RegisterFile {
public:
    SysCtlDevice();
};

// ---------------------------------------------------------------------------
// 0xE6000000 - CMeP command/status block (identity unknown)
// ---------------------------------------------------------------------------

/// The second loader's stage E (0x40590 -> 0x483da -> 0x482a2) drives a block at
/// 0xE6008000 with a per-slot register set, and whatever it talks to is not
/// described by any material in the workspace yet. The register semantics below
/// are read straight off the disassembly; the *answers* are an ASSUMPTION fitted
/// to the three checks the loader performs, and are meant to be replaced once the
/// block is identified:
///
///   0xE6008000 + i*8   write: the slot command (only the low 3 bits are used)
///   0xE6008020         read:  4 bit status per slot; the init at 0x4840E..0x484B0
///                             writes 5/2/1 and then waits for 4/2/1, i.e. the
///                             status reports the highest set bit of the command
///   0xE6008100 + i*8   write: 16 bit value
///   0xE6008120 + i*8   write a byte, read the answer in bits 8..15; the
///                             identification at 0x4838A needs (answer & 7) == 1
///                             after writing 5 and (answer & 3) == 0 with
///                             ((answer >> 2) & 0xf) == 4 or 5 after writing 8
///   0xE6008180         write 1, reads back 1 (a commit strobe)
class CmdBlockDevice : public RegisterFile {
public:
    CmdBlockDevice();

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;

    /// Slots the block answers for (the loader programs five).
    static constexpr u32 kSlots = 8;

private:
    std::array<u8, kSlots> command_{};  ///< last command written per slot
    std::array<u8, kSlots> reply_{};    ///< answer the byte port hands back
};

// ---------------------------------------------------------------------------
// 0x5FFC0000 - "SCE" command block (secure engine mailbox)
// ---------------------------------------------------------------------------

/// The second loader copies a 48 byte request to 0x5FFC0000 and reads the answer
/// back from the same block (0x40B10 / 0x4A774): the magic must be `SCE\0`, the
/// second word 3 and bit 0x40 of byte 8 set.  On hardware the secure engine
/// fills the answer; the model powers the block up with exactly such an answer
/// and keeps whatever the loader writes afterwards, which is the documented
/// development substitution from docs/KBL.md rounds 28/29 (no dump in the
/// workspace carries the engine's keys).
class SceBlockDevice : public RegisterFile {
public:
    /// `keys` is the machine's SCE key table: the block needs it to run the
    /// decryption job the second loader posts (see process_job()).
    explicit SceBlockDevice(SceKeys* keys = nullptr);

    /// Diagnostic trace of the request/answer traffic (`ZLB_SCEBLOCK_TRACE=1`).
    /// Added in round 153-6 to recover the engine's protocol from a real run:
    /// the loader fills the window and reads an answer back, and the model has
    /// to learn which offsets the answer is expected at.
    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;

    static constexpr u32 kBase = 0x5FFC0000;
    static constexpr u32 kSize = 0x00010000;

protected:
    /// The window is plain byte addressed memory, not a scalar register file:
    /// the second loader fills the SELF header area with *byte* stores (3584 of
    /// them in the 1.04 run) and reads the answer back with *word* loads, which
    /// RegisterFile's per-address storage cannot represent (round 153-8).
    u64 read_register(u32 address, unsigned size) override;
    void write_register(u32 address, unsigned size, u64 value) override;

private:
    static bool trace_enabled();
    static u64 traced_accesses_;

    /// The engine's job (round 153-7/153-8): the second loader first posts the
    /// 48 byte prologue (SCE header + the first two u64 of the SELF header) and
    /// then the SELF header area `SELF[0x30 .. header_length)`; it reads the same
    /// 4048 bytes back expecting the metadata decrypted in place.  Rebuild the
    /// image from the prologue plus the window and decrypt its metadata.
    void process_job();

    SceKeys* keys_ = nullptr;
    std::array<u8, kSize> memory_{};  ///< the window as byte addressed memory
    std::array<u8, 48> prologue_{};   ///< SELF[0x00 .. 0x30)
    unsigned prologue_words_ = 0;    ///< words of the prologue captured so far
    bool prologue_seen_ = false;     ///< a SCE magic write opened a new job
    bool payload_written_ = false;   ///< the header area has been posted
    bool job_done_ = false;
    u64 jobs_ = 0;
};

// ---------------------------------------------------------------------------
// 0xE0100000 - CMeP storage controller (not yet identified)
// ---------------------------------------------------------------------------

/// The second loader initialises a second controller right after the GPIO block
/// (0x49568..0x495E0): it walks a table of two descriptors at 0x561A4 - entry 0
/// is the GPIO block at 0xE20A0000 and entry 1 this one - reads +0x00 and the
/// register banks at +0x14..+0x2C and +0x38..+0x48 and writes the latter back to
/// clear them.  Nothing else touches the window on the boot path we can reach,
/// so the model provides a plain register file (reads must not come back as
/// 0xFF, which is what an unmapped access returns and which the loader then
/// complements and stores).  See docs/KBL.md round 26 and docs/HARDWARE.md.
class CmepStorageDevice : public RegisterFile {
public:
    CmepStorageDevice();

    static constexpr u32 kBase = 0xE0100000;
    static constexpr u32 kSize = 0x1000;
};

// ---------------------------------------------------------------------------
// 0xE0020000 - CMeP flags / bignum synchronisation
// ---------------------------------------------------------------------------

class CmepFlagsDevice : public RegisterFile {
public:
    CmepFlagsDevice();

    /// 0xE0020020 is the Bignum completion semaphore.  The engine raises it
    /// when an operation finishes; both loaders collect the result and then
    /// write the value back to release it (first loader 0x5CF12/0x5CF14,
    /// second loader engine_wait() at 0x4BBB0, which spins until the register
    /// is non-zero and clears it).  The second loader's bignum wrapper also
    /// refuses to program the engine while a completion is still pending
    /// (0x4BA26..0x4BA38 -> 0x800F0010), so the caller retries through the wait.
    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;

    u64 sync_pulses() const { return sync_pulses_; }
    u32 work_state() const { return static_cast<u32>(peek(kWorkState)); }
    void set_work_state(u32 value) { poke(kWorkState, value); }

    static constexpr u32 kWorkState = 0xE0020020;

private:
    u64 sync_pulses_ = 0;
};

// ---------------------------------------------------------------------------
// 0xE0030000 - keyring controller
// ---------------------------------------------------------------------------

/// One entry of the write log, for the debugger and the tests.
struct KeyringWriteRecord {
    u32 raw_trigger = 0;  ///< value written to 0xE0030020
    u32 index = 0;        ///< bits 0-12 of the trigger
    u32 flags = 0;        ///< value written to KeyringNewValue[0] (low 16 bits)
    std::array<u8, 32> value{};
    bool had_payload = false;  ///< a device write delivered KeyringNewValue[0]
};

class KeyringDevice : public RegisterFile {
public:
    KeyringDevice(Bus& bus, SceKeys& keys, CmepBlock& owner);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;

    /// 8 x 32-bit staged "new value" registers captured into SceKeys::keyring().
    const std::map<u32, KeyringSlot>& slots() const { return slots_; }
    const std::vector<std::pair<u32, u32>>& clear_history() const { return clear_history_; }
    const std::vector<KeyringWriteRecord>& writes() const { return writes_; }

    /// Index of a raw ClearFlags word (low 13 bits, e.g. 0x1C0F020E -> 0x20E).
    static u32 clear_index(u32 raw) { return raw & 0x1FFFu; }
    /// Flag mask of a raw ClearFlags word (0x1C0F020E -> 0x1C0F0000).
    static u32 clear_mask(u32 raw) { return raw & ~0x1FFFu; }

    /// Index last written to 0xE0030028.
    u32 last_query() const { return last_query_; }
    /// Value the model answers at 0xE003002C (3 = "all flags clear").
    u32 flags_response() const { return flags_response_; }
    /// Force the answer of the 0x501 boot-mode query (CmepBlock::set_keyring_flags).
    void set_flags_response(u32 value) {
        flags_response_ = kValidBit | (value & 0xFFu);
        poke(kQueryResponse, flags_response_);
    }

    u64 set_value_triggers() const { return set_value_triggers_; }
    u64 clear_flags_writes() const { return clear_flags_writes_; }

    /// Install a slot without going through the bus (used by reset / tests).
    void install_slot(u32 index, u32 flags, const std::array<u8, 32>& value);

    /// Bigmac keyring transfer targets: stage a payload written by a keyring
    /// write command, and read a slot back for a keyring read command.
    void stage_keyring(u32 index, u32 flags, const std::vector<u8>& payload);
    bool read_keyring(u32 index, std::vector<u8>& out) const;
    void copy_to_bus(u32 address, const std::vector<u8>& bytes);

    /// Build the fused boot-slot material (AES-128 key || IV) from the key set.
    static std::array<u8, 32> fused_boot_key(const SceKeys& keys);

    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    static constexpr u32 kNewValue = 0xE0030000;
    static constexpr u32 kSetValueTrigger = 0xE0030020;
    static constexpr u32 kClearFlags = 0xE0030024;
    static constexpr u32 kQueryRequest = 0xE0030028;
    static constexpr u32 kQueryResponse = 0xE003002C;
    /// "Keyring present/valid" bit of the query response word: the CMeP secure
    /// kernel compares the whole word against 0x10000003 (image 0x801A10/0x801A24,
    /// `bne $10,$1` at 0x801A4C), while the first loader only reads the low bits.
    static constexpr u32 kValidBit = 0x10000000u;

private:
    Bus& bus_;
    SceKeys& keys_;
    CmepBlock& owner_;
    std::map<u32, KeyringSlot> slots_;
    std::vector<std::pair<u32, u32>> clear_history_;
    std::vector<KeyringWriteRecord> writes_;
    u32 last_query_ = 0;
    std::array<u8, 32> fused_value_{};
    u32 flags_response_ = 3;
    u64 set_value_triggers_ = 0;
    u64 clear_flags_writes_ = 0;
};

// ---------------------------------------------------------------------------
// 0xE0060000 - boot strap / JIG
// ---------------------------------------------------------------------------

class StrapDevice : public RegisterFile {
public:
    StrapDevice(Bus& bus, CmepBlock& owner);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;

    /// 0xE0062020 bit 0 selects 'A' (normal) vs '!' (service) at 0x5C116.
    void set_bit0(bool set);
    bool bit0() const { return (peek(kStrap) & 1u) != 0; }

    std::string summary() const override;

    static constexpr u32 kStrap = 0xE0062020;
    static constexpr u32 kStrapSize = 0x20;  ///< 32-byte block, 0x5C164..0x5C178
    static constexpr u32 kScState = 0xE0064060;

private:
    Bus& bus_;
    CmepBlock& owner_;
};

// ---------------------------------------------------------------------------
// 0xE0070000 - eMMC crypto window
// ---------------------------------------------------------------------------

class EmmcCryptoDevice : public RegisterFile {
public:
    EmmcCryptoDevice();

    /// 0xE0070000 <- 1 enables the engine (0x5C0EA); 0xE0070008 carries the two
    /// keyring indexes as 0x020E020F (0x5C0E8).
    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;

    bool enabled() const { return enabled_; }
    u32 keyring_hi() const { return (peek(kIndexes) >> 16) & 0xFFFFu; }
    u32 keyring_lo() const { return peek(kIndexes) & 0xFFFFu; }

    std::string summary() const override;

    static constexpr u32 kToggle = 0xE0070000;
    static constexpr u32 kIndexes = 0xE0070008;

private:
    bool enabled_ = false;
};

// ---------------------------------------------------------------------------
// 0xE20A0000 - GPIO / external-agent handshake
// ---------------------------------------------------------------------------

class GpioDevice : public RegisterFile {
public:
    GpioDevice();

    /// mailbox_debug_sc polls +0x04 twice at 0x5E4FC and 0x5E560.  Bit 4 is
    /// modelled as the debugger handshake: it reads set once and then clears,
    /// so the polling loop terminates (documented in cmep_block.cpp).
    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;

    u64 handshakes() const { return handshakes_; }

    static constexpr u32 kData = 0xE20A0000;
    static constexpr u32 kState = 0xE20A0004;
    static constexpr u32 kSet = 0xE20A0008;
    static constexpr u32 kClear = 0xE20A000C;

private:
    u64 handshakes_ = 0;
};

// ---------------------------------------------------------------------------
// 0xE3100000 - SC / syscon register window
// ---------------------------------------------------------------------------

class ScBridgeDevice : public RegisterFile {
public:
    ScBridgeDevice();

    /// The first loader uses a program-and-poll pattern: it writes a request
    /// bit to 0xE31010A0 / 0xE31020A0 and polls the same register, then writes
    /// the same bit to 0xE31010A4 / 0xE31020A4 (0x5C246..0x5C260 for the
    /// bit5 path, 0x5C3BC..0x5C3D8 for bit6).  The model acknowledges a
    /// request immediately so the loops exit.
    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;

    /// Forward the SC registers to the real block (Ernie's 0xE3100000 window).
    void attach_shared_sc(Device* sc);

    u64 requests() const { return requests_; }

    static constexpr u32 kCmd124 = 0xE3100124;
    static constexpr u32 kReqA0 = 0xE31010A0;
    static constexpr u32 kAckA4 = 0xE31010A4;
    static constexpr u32 kReg1100 = 0xE3101100;
    static constexpr u32 kReg1190 = 0xE3101190;
    static constexpr u32 kReq20A0 = 0xE31020A0;
    static constexpr u32 kAck20A4 = 0xE31020A4;
    static constexpr u32 kReg2100 = 0xE3102100;
    static constexpr u32 kReg3040 = 0xE3103040;
    static constexpr u32 kReg3050 = 0xE3103050;
    static constexpr u32 kEventsC0 = 0xE31000C0;  ///< SC event register (write-1-to-clear)

private:
    u64 requests_ = 0;
    bool have_a0_ = false;
    bool have_20a0_ = false;
    Device* shared_sc_ = nullptr;

    /// 0xE31000C0 seen from the CMeP: the second loader's SC bring-up waits for
    /// *exactly* 1 (bit 0) here and acknowledges it by writing the same value
    /// back, then waits for the register to clear:
    ///
    ///     000485D6  movh $2,0xE310 / or3 $2,$2,0xC0
    ///     000485DE  erepeat 0x485E8
    ///     000485E2  lw   $0,($2)
    ///     000485EC  beqi $0,0x1,0x485F0     ; loops until it reads 1
    ///     000485F0  sw   $0,($3)            ; write-1-to-clear
    ///     000485FA  bnei $2,0x1,0x485FE     ; wait for the bit to go away
    ///
    /// The ARM's kernel boot loader polls the *same* register for bits 3 and 4
    /// (0x4003C02E / 0x4003C066), so the two sides have their own pending-event
    /// bits; handing the CMeP the ARM's mask (0x18) made it spin forever.
    u32 events_c0_ = 1;
};

// ---------------------------------------------------------------------------
// 0xE0B00000 / 0xE0BF0000 - SC command / response transfer window
// ---------------------------------------------------------------------------

/// The 28-byte SC command descriptor built at 0x5EE20 by sc_xfer (0x5CF58):
///
///   +0x00  u8   mode/flags       (0x30, 0x80 or 0x101; loc_5CFB0..loc_5CFE4)
///   +0x04  u32  timeout          (0x47C0C000 = 0x0B71B000 << 2)
///   +0x08  u32  timeout2         (0x0B71B000 << 1 when $2 != 0)
///   +0x0E  u8   channel          ($tp = $1 & 0xFF, stored by `sb $tp,14($7)`)
///   +0x18  u32  window base      0xE0B00000, or 0xE0BF0000 + (channel << 16)
///   +0x24  u32  control pointer  (the descriptor is handed to sc_read; the
///                                engine touches +0x2C..+0x3A of that buffer)
///
/// sc_read (0x5CFFE) then: writes 1 to control+0x2F when mode bit 0 is set and
/// polls it clear; when mode bit 1 is set it programs +0x2E/+0x34/+0x36/+0x38/
/// +0x3A, waits for control+0x24 bit 1 and then writes the two 16-bit values
/// back to +0x30/+0x32 and waits for them to become zero.
struct ScDescriptor {
    u8 mode = 0;
    u32 timeout = 0;
    u32 timeout2 = 0;
    u8 channel = 0;
    u32 window = 0;
    u32 control = 0;
    u32 command = 0;  ///< the SC command derived from the channel/descriptor
    std::vector<u8> payload;
    std::vector<u8> reply;
    bool valid = false;
};

class ScXferDevice : public Device {
public:
    ScXferDevice();

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;

    const std::vector<u8>& descriptor() const { return descriptor_; }
    bool pending() const { return pending_; }
    void clear_pending() { pending_ = false; }

    const char* register_name(u32 address) const override;
    void enumerate_registers(std::vector<RegisterInfo>& out) const override;

    /// Poke a descriptor into the window (tests / the SC init path).
    void post_descriptor(const std::vector<u8>& bytes, u32 window_base);

    /// Absolute address where sc_xfer finds the descriptor for `channel`.
    static u32 descriptor_address(u32 channel);

    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    static constexpr u32 kBase = 0xE0B00000;
    /// 0xE0B00000..0xE0C0FFFF: the command window plus the first response slot
    /// (0xE0BF0000 + (channel << 16), 0x5CF76..0x5CF7C).
    static constexpr u32 kSize = 0x00110000;
    static constexpr u32 kCmdWindow = 0xE0B00000;
    static constexpr u32 kRespWindow = 0xE0BF0000;
    /// Where sc_xfer places the descriptor inside the selected window.
    static constexpr u32 kDescriptorOffset = 0x00B0FF00;
    /// Size of the queueable descriptor/control block.
    static constexpr u32 kDescriptorSize = 0x2C;

private:
    std::vector<u8> descriptor_;
    u32 window_base_ = kCmdWindow;
    bool pending_ = false;
};

// ---------------------------------------------------------------------------
// 0xE0040000 - Bignum / RSA engine
// ---------------------------------------------------------------------------

/// The Bignum / modular-exponentiation engine (0xE0040000, mirrored every
/// 0x1000).  Recovered from both loaders:
///
///   first loader  bignum_op 0x5CE04          second loader 0x4BA20 wrapper
///   -------------                            -----------------------------
///   0x5CE0C 64 words from $2 -> 0x108        0x4BA4E/0x4BA5A -> 0x108 (base)
///   0x5CE28 64 words from $3 -> 0x400        0x4BA6A/0x4BA72 -> 0x400 (modulus)
///   0x5CE50 0x808 <- exponent word 0         0x4BAF6 streams 0x808 words
///   0x5CE5C 0x800 <- 0x91000000|n1<<18|n2<<9 0x4BAA0 the same control word
///   0x5CE68 ..&0x02000000 error              0x4BAC2 ..&0x02000000 error
///   0x5CE7A ..&0x04000000 "engine is ready"  0x4BAA6 waits for 0x04000000
///   0x5CE8C ..&0x08000000 "wants a word"     0x4BADE feeds while 0x04000000
///   0x5CECA 64 words 0x508 -> byte-reversed  0x4BB60 the same copy-out
///   0x5CF12/0x5CF14 E0020020 collect        0x4BBB0 engine_wait()
///
/// So the control word's bit 18..27 field is the modulus word count (n1) and
/// bits 9..17 the exponent word count (n2); the base and modulus sit in 64-word
/// windows, the exponent arrives one word at a time through 0xE0040808 in
/// big-endian order, and the result window holds the byte-reversed result.  The
/// status word is:
///
///   bit 31     busy (an operation is in flight)
///   bit 25     error
///   bit 26     ready - the engine can take another word (set while idle too)
///   bit 27     wants a word - the exponent stream is incomplete
///   bits 16-23 result length in words (equal to the modulus word count)
///
/// The first loader checks *both* 26 and 27 before it feeds another word, which
/// is exactly the "idle vs needs-data" split: after the last exponent word 26
/// stays set (the engine is idle again) while 27 drops, so the loop stops.  The
/// second loader feeds while 26 is set but counts the words itself.
class BignumDevice : public Device {
public:
    BignumDevice(Bus& bus, CmepBlock& owner);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;

    const char* register_name(u32 address) const override;
    void enumerate_registers(std::vector<RegisterInfo>& out) const override;
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    /// Big-endian modulus / exponent actually used by the engine.
    std::vector<u8> modulus() const;
    std::vector<u8> exponent() const;
    /// Read a 64-word window (0xE0040400 modulus, 0xE0040108 input) as a
    /// big-endian integer: the windows hold the little-endian word encoding of
    /// the value, so the walk reverses the word order and byte-swaps each word.
    std::vector<u8> read_window(u32 base) const;
    void set_result(const std::vector<u8>& big_endian);
    /// Finish an operation: run the engine and publish the result.
    void finish_operation();

    /// Substitute the modular exponentiation (the shared rsa_public() from
    /// loader/keys.h is used by default; tests may install their own).
    using EngineFn = std::vector<u8> (*)(const std::vector<u8>&, const std::vector<u8>&,
                                         const std::vector<u8>&);
    void set_engine(EngineFn fn) { engine_ = fn; }

    u64 operations() const { return operations_; }
    u32 result_words() const { return result_words_; }
    bool busy() const { return busy_; }
    bool wants_word() const { return want_word_; }

    static constexpr u32 kBaseAddr = 0xE0040000;
    /// 64 KiB so the per-0x1000 mirrors (0xE0041800, 0xE0042800, ...) decode to
    /// the same registers; the window is selected with the 0x1000 mask.
    static constexpr u32 kSizeAddr = 0x10000;
    /// 0x5CE0C / 0x4BA52: `or3 $9,$9,0x108` - the base/input window.
    static constexpr u32 kInput = 0xE0040108;
    /// 0x5CE28 / 0x4BA6E: `or3 $9,$9,0x400` - the modulus window.
    static constexpr u32 kModulus = 0xE0040400;
    /// 0x5CEC8..0x5CF04 (and 0x4BB52) walk 64 words here.
    static constexpr u32 kOutput = 0xE0040508;
    static constexpr u32 kStatus = 0xE0040804;
    static constexpr u32 kControl = 0xE0040800;
    static constexpr u32 kData = 0xE0040808;
    static constexpr u32 kMirrorStride = 0x1000;

    static constexpr u32 kStatusBusy = 0x80000000;   ///< bit 31
    static constexpr u32 kStatusError = 0x02000000;  ///< bit 25 (0x5CE6A, 0x4BAC2)
    static constexpr u32 kStatusReady = 0x04000000;  ///< bit 26 (0x5CE7A, 0x4BAA6)
    static constexpr u32 kStatusFetch = 0x08000000;  ///< bit 27 (0x5CE8E)

private:
    Bus& bus_;
    CmepBlock& owner_;
    std::map<u32, u64> words_;
    std::vector<u8> result_;
    /// Every word handed to the engine through 0xE0040808 since the last
    /// completed operation, in arrival order (most significant word first for a
    /// multi-word exponent).  The first loader writes word 0 *before* the
    /// control store, which is why the stream is kept outside the operation.
    std::vector<u32> port_words_;
    u32 control_ = 0;
    u32 mod_words_ = 0;
    u32 exp_words_ = 0;
    u32 result_words_ = 0;
    u64 operations_ = 0;
    bool error_ = false;
    bool busy_ = false;
    bool want_word_ = false;
    EngineFn engine_ = nullptr;
};

// ---------------------------------------------------------------------------
// 0xE0050000 - Bigmac (AES / SHA / HMAC / RNG / keyring DMA)
// ---------------------------------------------------------------------------

/// Operation codes issued through 0xE005000C, reconstructed from the wrappers
/// (see the header comment of this file).  Only the keyring/transfer codes are
/// attested by the first loader; the rest are named after the engines they
/// select and are configurable so that a mis-decoded opcode can be corrected by
/// the debugger instead of a rebuild.
enum class BigmacFunction : u32 {
    KeyringWrite1 = 0x0301,  ///< 0x5CCA8
    KeyringWrite2 = 0x010A,  ///< 0x5CCC0
    KeyringRead1 = 0x030A,   ///< 0x5CCD8
    Sha256 = 0x0033,         ///< 0x5CCF0
    AesCbcDecrypt = 0x000C,  ///< 0x5CD06 (keyring_op_5CD06)
    AesCbcEncrypt = 0x000D,
    AesEcbDecrypt = 0x000E,
    AesEcbEncrypt = 0x000F,
    Sha1 = 0x0010,
    HmacSha256 = 0x0012,
    Rng = 0x0013,
};

/// The full register image bigmac_cmd (0x5CD20) leaves behind, in the roles the
/// listing proves:
///
///   0x5CD32  +0x04 <- $1   keyring index / argument 0
///   0x5CD34  +0x00 <- $2   memory pointer (source for writes, destination for
///                          reads) / argument 1
///   0x5CD36  +0x08 <- $3   length / argument 2
///   0x5CD42  +0x14 <- the extra stack argument (key material pointer or a
///                          second destination buffer)
///   0x5CD5C  +0x10 <- $4   flags; when it is >= 0x1000 the eight words are also
///                          copied into the +0x200 window by 0x5CD64
///   0x5CDAA  +0x0C <- $5   function word (modifiers OR-ed in at 0x5CD4E:
///                          bit 28 when the keyring index is < 0x1000, and at
///                          0x5CD98: bit 7 when the key material came from a
///                          memory pointer)
///   0x5CDAC  +0x1C <- 1    start
struct BigmacCommand {
    u32 raw_command = 0;  ///< +0x04 : keyring index (transfers) / argument 0
    u32 pointer = 0;      ///< +0x00 : memory pointer
    u32 length = 0;       ///< +0x08
    u32 extra = 0;        ///< +0x14 : stack argument
    u32 flags = 0;        ///< +0x10
    u32 function = 0;     ///< +0x0C : raw function word with modifiers

    /// Keyring index (transfers) - bits 0-15 of +0x04.
    u32 index() const { return raw_command & 0xFFFFu; }
    /// Function word with the modifier bits stripped (bit 28, bit 7).
    u32 opcode() const { return function & 0x0FFFFFFFu & ~0x80u; }
    bool used_window_key() const { return (function & 0x80u) != 0; }
    bool small_index() const { return (function & 0x10000000u) != 0; }
};

class BigmacDevice : public Device {
public:
    BigmacDevice(Bus& bus, CmepBlock& owner);
    ~BigmacDevice() override;

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;

    const char* register_name(u32 address) const override;
    void enumerate_registers(std::vector<RegisterInfo>& out) const override;
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    /// Run one command directly (the self test uses this without a bus).
    /// Returns the value bigmac_cmd would return: 0 on success, -1 on error.
    /// `key_material` (when given) overrides the +0x00/+0x14 key source; `iv` is
    /// used by the CBC functions, otherwise the model falls back to the second
    /// half of a 256-bit key (the layout keyring_write1/2 produce: key in the
    /// low words, IV in the high ones).
    s32 execute(const BigmacCommand& cmd, const std::vector<u8>* key_material,
                const u8* iv = nullptr);

    /// Convenience overload for the keyring transfer wrappers.
    s32 execute(u32 command, u32 pointer, u32 length, const std::vector<u8>* key_material,
                u32 function, const u8* iv = nullptr);

    /// The IV the last CBC operation used.
    const std::array<u8, 16>& last_iv() const { return last_iv_; }

    /// Last key/IV handed to the engine (for the debugger).
    const std::array<u8, 32>& last_key() const { return last_key_; }
    unsigned last_key_bits() const { return last_key_bits_; }

    BigmacOp last_op() const { return last_op_; }
    u32 last_function() const { return last_function_; }
    u32 last_command() const { return last_command_; }
    u64 operations() const { return operations_; }
    u64 keyring_transfers() const { return keyring_transfers_; }
    u64 aes_operations() const { return aes_operations_; }
    u64 hash_operations() const { return hash_operations_; }
    u64 rng_operations() const { return rng_operations_; }

    /// Register opcodes (see BigmacFunction).  The keyring TransferMap holds
    /// the "classic" Bigmac keyring transfer ops used by the *second* loader
    /// (0x11 = transfer cmd+0x04 words to a keyring); the first loader uses the
    /// "new" form (0x0301 / 0x010A) captured by the wrappers above.
    void set_keyring_transfer_op(u32 op) { keyring_transfer_op_ = op; }
    u32 keyring_transfer_op() const { return keyring_transfer_op_; }

    DeterministicRng& rng() { return rng_; }
    const DeterministicRng& rng() const { return rng_; }

    static constexpr u32 kBaseAddr = 0xE0050000;
    static constexpr u32 kSizeAddr = 0x1000;
    static constexpr u32 kCmd = 0xE0050000;
    static constexpr u32 kArg0 = 0xE0050004;
    static constexpr u32 kArg1 = 0xE0050008;
    static constexpr u32 kFunction = 0xE005000C;
    static constexpr u32 kArg2 = 0xE0050010;
    static constexpr u32 kArg3 = 0xE0050014;
    static constexpr u32 kStart = 0xE005001C;
    static constexpr u32 kStatus = 0xE0050024;
    static constexpr u32 kException = 0xE005003C;
    static constexpr u32 kDataWindow = 0xE0050200;
    static constexpr u32 kDataWords = 8;

    static constexpr u32 kStatusBusy = 1u << 0;
    static constexpr u32 kStatusErrorMask = 0x78000u;
    static constexpr u32 kCmdUseWindow = 0x8000u;  ///< key lives in +0x200..+0x21F

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    static BigmacOp hash_op(u32 function);

    Bus& bus_;
    CmepBlock& owner_;
    BigmacOp last_op_ = BigmacOp::None;
    u32 last_function_ = 0;
    u32 last_command_ = 0;
    std::array<u8, 32> last_key_{};
    std::array<u8, 16> last_iv_{};
    unsigned last_key_bits_ = 0;
    u32 keyring_transfer_op_ = 0x11;
    u64 operations_ = 0;
    u64 keyring_transfers_ = 0;
    u64 aes_operations_ = 0;
    u64 hash_operations_ = 0;
    u64 rng_operations_ = 0;
    DeterministicRng rng_;
    bool initialized_ = false;
};

// ---------------------------------------------------------------------------
// Free helpers implemented in bigmac.cpp, shared with cmep_block.cpp / tests
// ---------------------------------------------------------------------------

/// AES block encryption for 128/192/256-bit keys (raw FIPS-197, no padding).
void aes_encrypt_block(const u8* key, unsigned key_bits, const u8 in[16], u8 out[16]);
/// AES block decryption.
void aes_decrypt_block(const u8* key, unsigned key_bits, const u8 in[16], u8 out[16]);

/// CBC/ECB over `length` bytes (must be a multiple of 16).
void aes_cbc_crypt(const u8* key, unsigned key_bits, const u8 iv[16], u8* data, size_t length,
                   bool encrypt);
void aes_ecb_crypt(const u8* key, unsigned key_bits, u8* data, size_t length, bool encrypt);

/// SHA-1 (20 bytes) and SHA-256 (32 bytes) over a byte range.
void sha1_digest(const u8* data, size_t length, u8 out[20]);
void sha256_digest(const u8* data, size_t length, u8 out[32]);
void hmac_sha256_digest(const u8* key, size_t key_length, const u8* data, size_t length, u8 out[32]);

/// Little-endian key material of `bits`/8 bytes as the Bigmac register window
/// consumes it (the loop at 0x5CD64 reads four bytes and builds a big-endian
/// word, so the memory layout is little-endian).
std::vector<u8> read_key_material(Bus& bus, u32 address, unsigned bits, bool* ok);

}  // namespace cmep_detail

}  // namespace zlb
