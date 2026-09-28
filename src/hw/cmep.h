// zeliboba - CMeP ("F00D") hardware block.
//
// Everything the MeP-c5 security core can reach: its RAM window, the keyring
// controller, the Bigmac crypto engine, the Bignum engine, the eMMC crypto
// window, the ARM/CMeP mailboxes, the boot strap and the SC bridge to Ernie.
#pragma once

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "loader/keys.h"

namespace zlb {

class EmmcCard;
class ErnieBlock;

/// Implementation details of this block live in src/hw/cmep/cmep_internal.h
/// (included *after* this header by the block's own translation units, so the
/// devices can use CmepBlock without a circular include).
namespace cmep_detail {
class MailboxDevice;
class KeyringDevice;
class StrapDevice;
class BigmacDevice;
class BignumDevice;
class ScXferDevice;
class CmepFlagsDevice;
}  // namespace cmep_detail

// ---------------------------------------------------------------------------
// Address map (see docs/bootrom_analysis and the CMeP register wiki)
// ---------------------------------------------------------------------------
namespace cmep {
/// sc_xfer (0x5CF58) builds its 28-byte SC command descriptor here: the memset
/// at 0x5CF8A clears 28 bytes at 0x5EE20, the fields are written at
/// 0x5CF8E..0x5CFA0, and sc_read (0x5CFFE) is handed that address as its
/// control pointer.
constexpr u32 kScDescriptorAddress = 0x0005EE20;
constexpr u32 kMailboxBase = 0xE0000000;
/// 0xE0010000 - CMeP system/strap inputs. The second loader reads +0x04 while it
/// validates the chip configuration (0x40DB6: `lw $2,(0xE0010004)` and then
/// requires `(value & 5) == 5` with bit 3 clear) and +0x04 of 0xE0020000, which
/// only has to have bits 27..31 and 0..4 clear.
constexpr u32 kSysCtlBase = 0xE0010000;
constexpr u32 kFlagsBase = 0xE0020000;
constexpr u32 kKeyringBase = 0xE0030000;
constexpr u32 kBignumBase = 0xE0040000;
constexpr u32 kBigmacBase = 0xE0050000;
constexpr u32 kStrapBase = 0xE0060000;
constexpr u32 kEmmcCryptoBase = 0xE0070000;
constexpr u32 kGpioBase = 0xE20A0000;
/// 0xE6000000 - CMeP command/status block of unknown identity (see
/// CmdBlockDevice in cmep_internal.h). Only the window 0xE6008000..0xE60081FF is
/// used by the boot chain.
constexpr u32 kCmdBlockBase = 0xE6000000;
constexpr u32 kCmdBlockSize = 0x10000;
constexpr u32 kScBase = 0xE3100000;
/// The SC register window is 128 KiB wide: both processors see one block, and
/// Ernie's SC device answers exactly this span (ernie_internal.h
/// kScWindowSize).  The second loader reaches 0xE3110C00 through it (0x4864C).
constexpr u32 kScWindowSize = 0x20000;
constexpr u32 kScXferBase = 0xE0B00000;
/// Keyring slot that holds the chip's fused boot key (AES-128 key || IV): the
/// first loader loads it from image+0xE0 but the hardware value survives.
constexpr u32 kBootKeyring = 10;
constexpr u32 kRamBase = 0x00040000;
constexpr u32 kRamSize = 0x20000;      // 128 KiB window 0x40000-0x5FFFF
constexpr u32 kPrivateBase = 0x00800000;  // secure kernel / private RAM
constexpr u32 kPrivateSize = 0x200000;
/// 0x5FFC0000: the command block the second loader hands to the "SCE" engine
/// (0x40B10 copies the 48 byte request there, 0x4A774 reads the answer back and
/// checks the `SCE\0` magic).  On hardware this is the secure engine's mailbox.
constexpr u32 kSceBlockBase = 0x5FFC0000;
constexpr u32 kSceBlockSize = 0x00010000;
}  // namespace cmep

/// Kinds of work the Bigmac engine can be asked to do. The first loader uses
/// keyring transfers, the RNG and AES/HMAC/SHA commands.
enum class BigmacOp : u32 {
    None = 0,
    KeyringWrite = 1,
    KeyringRead = 2,
    Rng = 3,
    Aes = 4,
    Sha = 5,
    Hmac = 6,
};

/// The CMeP side of the board.
class CmepBlock {
public:
    CmepBlock(Bus& bus, SceKeys& keys);
    ~CmepBlock();

    /// Install every device into the bus and map the RAM windows.
    void install();

    void reset();


    /// The boot strap block 0xE0062020 that selects 'A' (normal) vs '!' (service).
    void set_strap_bit0(bool set);
    /// Value the model returns for keyring 0x501 flag queries.
    void set_keyring_flags(u32 flags) { keyring_flags_ = flags; }
    u32 keyring_flags() const { return keyring_flags_; }

    /// Boot mode reported to the next stage ('A' = 0x41, '!' = 0x21).
    void set_boot_mode(u8 mode);
    u8 boot_mode() const { return boot_mode_; }

    /// Mailbox helpers used by the boot chain and the debugger.
    u32 arm_to_cmep_command() const;
    void set_arm_to_cmep_command(u32 value);
    u32 cmep_status() const;
    void set_cmep_status(u32 value);

    /// Attach the storage path: SC commands issued by the first loader are
    /// forwarded to Ernie, which drives the card.
    void attach_storage(ErnieBlock* ernie, EmmcCard* card);
    /// Complete an SC transfer that the first loader queued at 0xE0B00000.
    void service_sc_transfer();

    /// Keyring access for the debugger.
    const std::map<u32, KeyringSlot>& captured_keyrings() const;
    const std::vector<std::pair<u32, u32>>& clear_flags_history() const;

    u64 keyring_writes() const;
    u64 bigmac_operations() const;
    u64 bignum_operations() const;
    u64 sc_transfers() const;

    /// True when the first loader has written "success" to the ARM mailbox.
    bool reported_success() const;
    /// True when the first loader took the failure path (mailbox = 2).
    bool reported_failure() const;

    std::string summary() const;
    void describe(std::vector<std::string>& lines) const;

    /// Access to the devices for the debugger's device list.
    std::vector<Device*> devices() const;

    /// Point the CMeP's SC register window (0xE3100000) at the *shared* SC block -
    /// the same registers the ARM polls and Ernie serves. Without this the CMeP had
    /// a private copy that only faked acknowledgements, so a command posted by one
    /// side never reached the other.
    void attach_shared_sc(Device* sc);

    // ------------------------------------------------------------------
    // Wiring used by the block's own devices (and read-only for the machine).
    // ------------------------------------------------------------------

    Bus& bus() const { return *bus_; }
    SceKeys& keys() const { return *keys_; }
    cmep_detail::KeyringDevice& keyring_device() const;
    cmep_detail::BigmacDevice& bigmac_device() const;
    cmep_detail::BignumDevice& bignum_device() const;
    cmep_detail::MailboxDevice& mailbox_device() const;
    cmep_detail::StrapDevice& strap_device() const;
    cmep_detail::ScXferDevice& sc_xfer_device() const;
    /// 0xE0020000 block - the Bignum completion semaphore lives here.
    cmep_detail::CmepFlagsDevice& flags_device() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    /// Look a device up in the bus (install() hands ownership of the devices to
    /// the bus, so raw member pointers would dangle).
    Device* device_by_name(const char* name) const;
    Bus* bus_ = nullptr;
    SceKeys* keys_ = nullptr;
    u32 keyring_flags_ = 3;  // "all flags clear" -> boot mode query succeeds
    u8 boot_mode_ = 0x41;
};

}  // namespace zlb
