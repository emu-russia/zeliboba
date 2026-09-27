// zeliboba - CMeP ("F00D") hardware block: installation and the block level
// boot protocol (strap, boot mode, SC transfers, mailbox reporting).
//
// The device models live next to this file:
//   keyring.cpp  0xE0030000  keyring controller
//   bigmac.cpp   0xE0050000  Bigmac AES/SHA/HMAC/RNG engine
//   bignum.cpp   0xE0040000  Bignum / RSA engine
//   mailbox.cpp  0xE0000000, 0xE0020000, 0xE0060000, 0xE0070000, 0xE20A0000,
//                0xE3100000 and the 0xE0B00000 SC transfer window
//
// ---------------------------------------------------------------------------
// Boot protocol this block implements (annotated listing evidence)
// ---------------------------------------------------------------------------
//   main (0x5C4FE)
//     init_emmc_keyrings (0x5C0B8)
//         0xE0030024 <- 0x1C0F020E / 0x1C0F020F   (0x5C0DC, 0x5C0E6)
//         0xE0070008 <- 0x020E020F               (0x5C0E8)
//         0xE0070000 <- 1                        (0x5C0EA)
//     check_boot_mode (0x5C0EE)
//         0xE0030028 <- 0x501                    (0x5C102)
//         $12 = 0xE003002C; ($12 & 3) != 3 -> -1 (0x5C104..0x5C10A)
//         $12 = *(0xE0062020); bit0 ? 'A'(65) : '!'(33)  (0x5C116..0x5C124)
//     ARM image fetch: poll 0xE0000010 bit0 (0x5C57A), PA = value & ~3
//         (0x5C586), copy 64 bytes to 0x40000 (0x5C58C), validate (0x5C592)
//     process_image (0x5CC00), then keyring_set_value (0x5C12A) and finally
//         0xE0000000 <- 1 on success (0x5C5F0) or <- 2 and hang on failure
//         (0x5C610..0x5C618).
#include <cstring>
#include <cstring>

#include "common/log.h"
#include "hw/cmep.h"
#include "hw/cmep/cmep_internal.h"
#include "hw/syscon.h"

namespace zlb {

using namespace cmep_detail;

// Device accessors declared in cmep.h (the definitions need the internal
// device types, which is why they are not inline there).
Device* CmepBlock::device_by_name(const char* name) const {
    if (bus_ == nullptr) return nullptr;
    for (const auto& device : bus_->devices()) {
        if (device->name() == name) return device.get();
    }
    return nullptr;
}

cmep_detail::KeyringDevice& CmepBlock::keyring_device() const {
    return *static_cast<cmep_detail::KeyringDevice*>(device_by_name("CMeP.Keyring"));
}
cmep_detail::BigmacDevice& CmepBlock::bigmac_device() const {
    return *static_cast<cmep_detail::BigmacDevice*>(device_by_name("CMeP.Bigmac"));
}
cmep_detail::BignumDevice& CmepBlock::bignum_device() const {
    return *static_cast<cmep_detail::BignumDevice*>(device_by_name("CMeP.Bignum"));
}
cmep_detail::MailboxDevice& CmepBlock::mailbox_device() const {
    return *static_cast<cmep_detail::MailboxDevice*>(device_by_name("CMeP.Mailbox"));
}
cmep_detail::StrapDevice& CmepBlock::strap_device() const {
    return *static_cast<cmep_detail::StrapDevice*>(device_by_name("CMeP.Strap"));
}
cmep_detail::ScXferDevice& CmepBlock::sc_xfer_device() const {
    return *static_cast<cmep_detail::ScXferDevice*>(device_by_name("CMeP.ScXfer"));
}
cmep_detail::CmepFlagsDevice& CmepBlock::flags_device() const {
    return *static_cast<cmep_detail::CmepFlagsDevice*>(device_by_name("CMeP.Flags"));
}

// ---------------------------------------------------------------------------
// Impl - owns every device while install() has not handed it to the bus
// ---------------------------------------------------------------------------

struct CmepBlock::Impl {
    Impl(Bus& bus, SceKeys& keys, CmepBlock& owner)
        : mailbox(new MailboxDevice(bus, owner)),
          sysctl(new SysCtlDevice()),
          flags(new CmepFlagsDevice()),
          keyring(new KeyringDevice(bus, keys, owner)),
          strap(new StrapDevice(bus, owner)),
          emmc_crypto(new EmmcCryptoDevice()),
          gpio(new GpioDevice()),
          cmd_block(new CmdBlockDevice()),
          sc_bridge(new ScBridgeDevice()),
          sc_xfer(new ScXferDevice()),
          storage(new CmepStorageDevice()),
          sce_block(new SceBlockDevice()),
          bignum(new BignumDevice(bus, owner)),
          bigmac(new BigmacDevice(bus, owner)),
          bigmac_mirror(new DeviceMirror(*bigmac, cmep::kBigmacBase + 0x1000, 0x3000)) {}

    std::unique_ptr<MailboxDevice> mailbox;
    std::unique_ptr<SysCtlDevice> sysctl;
    std::unique_ptr<CmepFlagsDevice> flags;
    std::unique_ptr<KeyringDevice> keyring;
    std::unique_ptr<StrapDevice> strap;
    std::unique_ptr<EmmcCryptoDevice> emmc_crypto;
    std::unique_ptr<GpioDevice> gpio;
    std::unique_ptr<CmdBlockDevice> cmd_block;
    std::unique_ptr<ScBridgeDevice> sc_bridge;
    std::unique_ptr<ScXferDevice> sc_xfer;
    std::unique_ptr<CmepStorageDevice> storage;
    std::unique_ptr<SceBlockDevice> sce_block;
    std::unique_ptr<BignumDevice> bignum;
    std::unique_ptr<BigmacDevice> bigmac;
    /// The wiki map mirrors the Bigmac window every 0x1000; the mirror keeps
    /// the same register names in the debugger's MMIO view.
    std::unique_ptr<DeviceMirror> bigmac_mirror;

    bool installed = false;
    ErnieBlock* ernie = nullptr;
    EmmcCard* card = nullptr;
    u64 sc_transfers = 0;
    u64 sc_failures = 0;
    u32 last_sc_command = 0;
    std::vector<u8> last_sc_reply;
};

// ---------------------------------------------------------------------------
// Construction / installation
// ---------------------------------------------------------------------------

CmepBlock::CmepBlock(Bus& bus, SceKeys& keys) : impl_(new Impl(bus, keys, *this)) {
    bus_ = &bus;
    keys_ = &keys;
}

CmepBlock::~CmepBlock() = default;

void CmepBlock::install() {
    // RAM windows first: the first loader is staged at 0x40000..0x5FFFF
    // (ANALYSIS.md section 3 - the image contains its own .bss at 0x5EB00 and
    // the hand-off stub writes 0 to 0x5C000, proving the window is RAM).
    bus_->ensure_ram(cmep::kRamBase, cmep::kRamSize, "cmep");
    if (MemRegion* ram = bus_->region_at(cmep::kRamBase)) ram->name = "CMeP.RAM";

    // Secure kernel / private RAM window.
    bus_->ensure_ram(cmep::kPrivateBase, cmep::kPrivateSize, "cmep_private");
    if (MemRegion* priv = bus_->region_at(cmep::kPrivateBase)) priv->name = "CMeP.Private";

    // The second loader hands its 48 byte "SCE" request to the secure engine by
    // copying it to 0x5FFC0000 and then reads the answer back from the same
    // block (0x4A774 checks the `SCE\0` magic, word 1 == 3 and bit 0x40 of byte
    // 8).  The block is a device that powers up with a valid answer and keeps
    // the loader's writes - documented as a development substitution in
    // docs/KBL.md round 28.

    if (!impl_->installed) {
        bus_->add_device(std::move(impl_->sce_block));
        bus_->add_device(std::move(impl_->mailbox));
        bus_->add_device(std::move(impl_->sysctl));
        bus_->add_device(std::move(impl_->flags));
        bus_->add_device(std::move(impl_->keyring));
        bus_->add_device(std::move(impl_->strap));
        bus_->add_device(std::move(impl_->emmc_crypto));
        bus_->add_device(std::move(impl_->gpio));
        bus_->add_device(std::move(impl_->cmd_block));
        bus_->add_device(std::move(impl_->sc_bridge));
        bus_->add_device(std::move(impl_->sc_xfer));
        bus_->add_device(std::move(impl_->storage));
        bus_->add_device(std::move(impl_->bignum));
        bus_->add_device(std::move(impl_->bigmac));
        bus_->add_device(std::move(impl_->bigmac_mirror));
        impl_->installed = true;
    }
    bus_->rebuild_map();
    reset();
}

void CmepBlock::reset() {
    for (const auto& device : bus_->devices()) {
        if (device->name().rfind("CMeP.", 0) == 0) device->reset();
    }
    impl_->sc_transfers = 0;
    impl_->sc_failures = 0;
    impl_->last_sc_command = 0;
    impl_->last_sc_reply.clear();
    boot_mode_ = 0x41;
    // The strap reports a normal, non-JIG console: 0xE0062020 must have bits 0..2
    // clear, because the second loader validates the chip configuration at
    // 0x40DB6 and aborts with 0x800F0033 otherwise (`and3 $3,$3,0x7` /
    // `beqz $3,<ok>`, where $3 = [0xE0062020]).  The first loader only uses bit 0
    // to pick the banner character ('A' when set, '!' when clear, 0x5C116..0x5C124)
    // and accepts either (0x5C510: `bgei $0,0x0,0x5c56c`), so the retail value is
    // the one that lets the whole chain run.  Keyring 0x501 answers 3 ("all flags
    // clear") so check_boot_mode (0x5C10A) succeeds.
    if (impl_->installed) {
        strap_device().set_bit0(false);
        keyring_device().set_flags_response(keyring_flags_);
        // KeyringDevice::reset() has already programmed slot 10 with the chip's
        // fused boot key (see KeyringDevice::fused_boot_key).
    }
}

// ---------------------------------------------------------------------------
// Strap / boot mode
// ---------------------------------------------------------------------------

void CmepBlock::set_strap_bit0(bool set) {
    if (impl_->installed) strap_device().set_bit0(set);
}

void CmepBlock::set_boot_mode(u8 mode) {
    boot_mode_ = mode;
    set_strap_bit0(mode == 0x41);
}

// ---------------------------------------------------------------------------
// Mailboxes
// ---------------------------------------------------------------------------

u32 CmepBlock::arm_to_cmep_command() const {
    return mailbox_device().arm_to_cmep();
}

void CmepBlock::set_arm_to_cmep_command(u32 value) {
    mailbox_device().set_arm_to_cmep(value);
}

u32 CmepBlock::cmep_status() const {
    return mailbox_device().cmep_to_arm();
}

void CmepBlock::set_cmep_status(u32 value) {
    mailbox_device().set_cmep_to_arm(value);
}

bool CmepBlock::reported_success() const {
    return cmep_status() == 1u;
}

bool CmepBlock::reported_failure() const {
    return cmep_status() == 2u;
}

// ---------------------------------------------------------------------------
// SC bridge
// ---------------------------------------------------------------------------

void CmepBlock::attach_storage(ErnieBlock* ernie, EmmcCard* card) {
    impl_->ernie = ernie;
    impl_->card = card;
}

/// Decode the 28-byte descriptor sc_xfer (0x5CF58) builds at 0x5EE20:
///
///   +0x00  u8   mode      (0x30 / 0x80 / 0x101; loc_5CFB0..loc_5CFE4)
///   +0x04  u32  timeout   (0x0B71B000 << 2 - 0x5CF98/0x5CF9C)
///   +0x08  u32  timeout2  (<< 2 normally, << 1 when the 5th argument is set;
///                          0x5CFA4..0x5CFAC)
///   +0x0E  u8   channel   (`sb $tp,14($7)` at 0x5CF8E, $tp = command & 0xFF)
///   +0x18  u32  window    (0x5CFA0 stores $6, the 0xE0Bxxxxx base; sc_read
///                          dereferences desc+0x18 as the control buffer)
///
/// sc_read (0x5CFFE) then drives the transfer through that control block;
/// the two call sites enter sc_xfer with $1 = 0x101 ("main init started"), so
/// the channel stored at +0x0E is 0x01 and the command is (mode << 8) | 1.
void CmepBlock::service_sc_transfer() {
    if (!impl_->installed) return;
    ScXferDevice& xfer = sc_xfer_device();
    ScDescriptor desc;
    // sc_xfer (0x5CF58) builds the descriptor in CMeP RAM at 0x5EE20: the memset
    // at 0x5CF8A clears 28 bytes, the fields are written at 0x5CF8E..0x5CFA0 and
    // sc_read (0x5CFFE) is handed that same address as its control pointer.
    std::vector<u8> raw(0x2C, 0);
    for (size_t i = 0; i < raw.size(); ++i) {
        const u32 address = cmep::kScDescriptorAddress + static_cast<u32>(i);
        if (!bus_->is_mapped(address)) break;
        raw[i] = bus_->read8(address);
    }
    bool ram_descriptor = false;
    for (u8 byte : raw) {
        if (byte != 0) ram_descriptor = true;
    }
    if (!ram_descriptor) {
        // Fall back to the copy queued inside the transfer window (debugger and
        // tests); when neither exists there is nothing to service.
        const std::vector<u8>& staged = xfer.descriptor();
        if (staged.size() < 0x2C) {
            xfer.clear_pending();
            return;
        }
        raw.assign(staged.begin(), staged.begin() + 0x2C);
    }
    desc.mode = raw[0];
    desc.timeout = static_cast<u32>(raw[4] | (raw[5] << 8) | (raw[6] << 16) | (raw[7] << 24));
    desc.timeout2 = static_cast<u32>(raw[8] | (raw[9] << 8) | (raw[0xA] << 16) | (raw[0xB] << 24));
    desc.channel = raw[0x0E];
    desc.window = static_cast<u32>(raw[0x18] | (raw[0x19] << 8) | (raw[0x1A] << 16) |
                                   (raw[0x1B] << 24));
    desc.control = static_cast<u32>(raw[0x24] | (raw[0x25] << 8) | (raw[0x26] << 16) |
                                    (raw[0x27] << 24));
    desc.command = (static_cast<u32>(desc.mode) << 8) | desc.channel;
    desc.valid = true;
    impl_->last_sc_command = desc.command;
    // sc_xfer's own window computation (0x5CF6C..0x5CF7C): 0xE0B00000 for
    // channel 0 and 0xE0BF0000 + (channel << 16) otherwise.
    if (desc.window == 0) {
        desc.window = (desc.channel == 0)
                          ? ScXferDevice::kCmdWindow
                          : ScXferDevice::kRespWindow + ((desc.channel & 0xFFu) << 16);
    }

    if (!impl_->ernie) {
        // No syscon attached: still complete the transfer so the boot chain does
        // not spin, and record the miss for the debugger.
        ++impl_->sc_failures;
        xfer.clear_pending();
        return;
    }

    // The command payload follows the descriptor's control block inside the
    // selected window.
    std::vector<u8> payload;
    const u32 payload_address = desc.window + 0x100;
    for (size_t i = 0; i < 16; ++i) {
        const u32 address = payload_address + static_cast<u32>(i);
        if (!bus_->is_mapped(address)) break;
        payload.push_back(bus_->read8(address));
    }
    if (payload.empty() && desc.channel != 0) payload.assign(16, 0);

    const std::vector<u8> reply = impl_->ernie->dispatch_command(desc.command, payload);
    impl_->last_sc_reply = reply;
    ++impl_->sc_transfers;

    // sc_read reads the answer back through the control block it was handed
    // (the engine programs +0x2E..+0x3A and polls +0x30/+0x32), so the reply is
    // written after the descriptor; the transfer window is updated as well.
    const u32 reply_address = desc.control != 0 ? desc.control + 0x2C : desc.window + 0x100;
    for (size_t i = 0; i < reply.size(); ++i) {
        const u32 address = reply_address + static_cast<u32>(i);
        if (bus_->is_mapped(address)) bus_->write8(address, reply[i]);
    }
    for (size_t i = 0; i < reply.size(); ++i) {
        const u32 address = desc.window + 0x100 + static_cast<u32>(i);
        if (bus_->is_mapped(address)) bus_->write8(address, reply[i]);
    }
    xfer.clear_pending();
}

// ---------------------------------------------------------------------------
// Debugger helpers
// ---------------------------------------------------------------------------

const std::map<u32, KeyringSlot>& CmepBlock::captured_keyrings() const {
    static const std::map<u32, KeyringSlot> empty;
    return impl_->installed ? keyring_device().slots() : empty;
}

const std::vector<std::pair<u32, u32>>& CmepBlock::clear_flags_history() const {
    static const std::vector<std::pair<u32, u32>> empty;
    return impl_->installed ? keyring_device().clear_history() : empty;
}

u64 CmepBlock::keyring_writes() const {
    return impl_->installed ? keyring_device().set_value_triggers() : 0;
}

u64 CmepBlock::bigmac_operations() const {
    return impl_->installed ? bigmac_device().operations() : 0;
}

u64 CmepBlock::bignum_operations() const {
    return impl_->installed ? bignum_device().operations() : 0;
}

u64 CmepBlock::sc_transfers() const { return impl_->sc_transfers; }

std::string CmepBlock::summary() const {
    const u32 strap = impl_->installed
                          ? static_cast<u32>(strap_device().peek(StrapDevice::kStrap))
                          : 0;
    return format("CMeP mode='%c' strap=0x%X keyrings=%zu bigmac=%llu bignum=%llu sc=%llu mb=%u",
                  boot_mode_ ? static_cast<char>(boot_mode_) : '?', strap, captured_keyrings().size(),
                  static_cast<unsigned long long>(bigmac_operations()),
                  static_cast<unsigned long long>(bignum_operations()),
                  static_cast<unsigned long long>(sc_transfers()), cmep_status());
}

void CmepBlock::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("CMeP boot mode       = '%c' (0x%02X)", static_cast<char>(boot_mode_),
                           boot_mode_));
    if (impl_->installed) {
        lines.push_back(format("CMeP strap 0xE0062020= 0x%08X",
                               static_cast<u32>(strap_device().peek(StrapDevice::kStrap))));
    }
    lines.push_back(format("CMeP mailbox status  = %u (%s)", cmep_status(),
                           reported_success() ? "reported success"
                                              : (reported_failure() ? "reported failure"
                                                                    : "in progress")));
    lines.push_back(format("CMeP keyring flags   = 0x%X (query 0x501)", keyring_flags_));
    if (impl_->installed) {
        lines.push_back(keyring_device().summary());
        lines.push_back(bigmac_device().summary());
        lines.push_back(bignum_device().summary());
        lines.push_back(strap_device().summary());
        lines.push_back(mailbox_device().summary());
        lines.push_back(sc_xfer_device().summary());
    }
    for (const char* name : {"CMeP.EmmcCrypto", "CMeP.GPIO", "CMeP.SecureCtl"}) {
        if (Device* device = device_by_name(name)) lines.push_back(device->summary());
    }
    lines.push_back(format("CMeP SC transfers    = %llu (failures %llu, last cmd 0x%X)",
                           static_cast<unsigned long long>(impl_->sc_transfers),
                           static_cast<unsigned long long>(impl_->sc_failures),
                           impl_->last_sc_command));
    for (const auto& entry : captured_keyrings()) {
        lines.push_back(format("CMeP keyring[0x%03X] flags=0x%04X locked=%d", entry.first,
                               entry.second.flags, entry.second.locked ? 1 : 0));
    }
}

void CmepBlock::attach_shared_sc(Device* sc) {
    // install() hands every device to the bus, which nulls the Impl pointers, so
    // the device has to be looked up by name here - the first version of this
    // function used the (now null) impl pointer and silently did nothing, which
    // is why the CMeP saw a private copy of the SC registers.
    ScBridgeDevice* bridge = impl_->sc_bridge.get();
    if (bridge == nullptr) bridge = static_cast<ScBridgeDevice*>(device_by_name("CMeP.SecureCtl"));
    if (bridge != nullptr) bridge->attach_shared_sc(sc);
}

std::vector<Device*> CmepBlock::devices() const {
    static const char* kNames[] = {"CMeP.Mailbox",    "CMeP.Flags",      "CMeP.Keyring",
                                   "CMeP.Strap",      "CMeP.EmmcCrypto", "CMeP.GPIO",
                                   "CMeP.SecureCtl",  "CMeP.ScXfer",     "CMeP.Bignum",
                                   "CMeP.Bigmac",     "CMeP.Bigmac@mirror"};
    std::vector<Device*> out;
    for (const char* wanted : kNames) {
        for (const auto& device : bus_->devices()) {
            if (device->name() == wanted) {
                out.push_back(device.get());
                break;
            }
        }
    }
    return out;
}

}  // namespace zlb
