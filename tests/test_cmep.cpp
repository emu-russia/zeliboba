// zeliboba - unit tests for the CMeP ("F00D") security block.
//
// The tests drive the block the way the first loader does: through the memory
// mapped registers on a real Bus, using the register sequences documented in
// src/hw/cmep/*.cpp (every claim there carries the instruction address of the
// annotated boot ROM listing that proves it).
#include <array>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/log.h"
#include "common/util.h"
#include "cpu/mep/mep_core.h"
#include "hw/cmep.h"
#include "hw/cmep/cmep_internal.h"
#include "hw/soc.h"
#include "machine/bootkeys.h"
#include "machine/bootkeys_data.h"
#include "machine/vita.h"
#include "test_framework.h"

using namespace zlb;

namespace {

/// One scratch machine per test: bus + keys + the block, installed.
struct Fixture {
    std::unique_ptr<Bus> bus_owner = std::make_unique<Bus>();
    Bus& bus = *bus_owner;
    SceKeys keys;
    CmepBlock cmep;

    Fixture() : cmep(bus, keys) {
        bus.unmapped_reads_zero = true;
        cmep.install();
    }
};

/// pup_fiction keys.py ENC_KEY / ENC_IV (AES-128-CBC over the .enc body).
const u8 kEncKey[16] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
                        0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
const u8 kEncIv[16] = {0xAF, 0x5F, 0x2C, 0xB0, 0x4A, 0xC1, 0x75, 0x1A,
                       0xBF, 0x51, 0xCE, 0xF1, 0xC8, 0x09, 0x62, 0x10};

void issue_native_bigmac(Fixture& f, u32 source, u32 destination, u32 length,
                        u32 function, u32 iv = 0) {
    f.bus.write32(0xE0050000u, source);
    f.bus.write32(0xE0050004u, destination);
    f.bus.write32(0xE0050008u, length);
    f.bus.write32(0xE005000Cu, function);
    f.bus.write32(0xE0050014u, iv);
    f.bus.write32(0xE0050104u, 3u);
    f.bus.write32(0xE005001Cu, 1u);
}

void stage_bigmac_key(Fixture& f, const u8* key, size_t length) {
    for (size_t i = 0; i < length; i += 4) {
        const u32 word = static_cast<u32>(key[i]) |
                         (static_cast<u32>(key[i + 1]) << 8) |
                         (static_cast<u32>(key[i + 2]) << 16) |
                         (static_cast<u32>(key[i + 3]) << 24);
        f.bus.write32(0xE0050200u + static_cast<u32>(i), word);
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Installation / register decode
// ---------------------------------------------------------------------------

ZLB_TEST(cmep_devices_installed) {
    Fixture f;
    // Every window the first loader touches must resolve to a device.
    const u32 addresses[] = {0xE0000000, 0xE0000010, 0xE0000020, 0xE0000028, 0xE0020000,
                             0xE0020020, 0xE0030000, 0xE0030020, 0xE0030024, 0xE0030028,
                             0xE003002C, 0xE0040800, 0xE0040804, 0xE0040808, 0xE0050000,
                             0xE005000C, 0xE005001C, 0xE0050024, 0xE005003C, 0xE0062020,
                             0xE0064060, 0xE0070000, 0xE0070008, 0xE20A0000, 0xE20A0004,
                             0xE31010A0, 0xE31010A4, 0xE31020A0, 0xE31020A4, 0xE0B00000,
                             0xE0BF0000};
    for (u32 address : addresses) {
        Device* device = f.bus.find_device(address);
        ZLB_EXPECT_TRUE(device != nullptr);
        if (device != nullptr) {
            ZLB_EXPECT_TRUE(device->name().rfind("CMeP.", 0) == 0);
            // The debugger's MMIO view needs a name at the exact address.
            if (device->register_name(address) == nullptr && device->base() == address) {
                ZLB_FAIL(format("device %s has no register name for its own base 0x%08X",
                                device->name().c_str(), address));
            }
        }
    }
    // RAM windows: the first loader image lives at 0x40000 (ANALYSIS.md §3).
    ZLB_EXPECT_TRUE(f.bus.is_ram(0x40000));
    ZLB_EXPECT_TRUE(f.bus.is_ram(0x5FFFF));
    ZLB_EXPECT_TRUE(f.bus.is_ram(cmep::kPrivateBase));
    ZLB_EXPECT_EQ(f.cmep.devices().size(), static_cast<size_t>(11));
}

ZLB_TEST(cmep_register_names_are_readable) {
    Fixture f;
    for (Device* device : f.cmep.devices()) {
        ZLB_EXPECT_TRUE(device != nullptr);
        if (device == nullptr) continue;
        std::vector<RegisterInfo> regs;
        device->enumerate_registers(regs);
        if (regs.empty()) {
            ZLB_FAIL(format("device %s enumerates no registers", device->name().c_str()));
        }
        for (const RegisterInfo& info : regs) ZLB_EXPECT_FALSE(info.name.empty());
    }
}

// ---------------------------------------------------------------------------
// Keyring controller (0xE0030000)
// ---------------------------------------------------------------------------

ZLB_TEST(cmep_keyring_capture_and_flags) {
    Fixture f;
    // keyring_set_value (0x5C12A): eight words then the trigger.
    for (u32 i = 0; i < 8; ++i) f.bus.write32(0xE0030000 + 4 * i, 0x11u * (i + 1));
    f.bus.write32(0xE0030020, 0x501);

    ZLB_EXPECT_EQ(f.cmep.keyring_writes(), 1u);
    const auto& slots = f.cmep.captured_keyrings();
    // Slot 10 is the fused boot key and is always present (see
    // KeyringDevice::fused_boot_key); the capture adds one more.
    ZLB_EXPECT_EQ(slots.size(), static_cast<size_t>(2));
    ZLB_EXPECT_TRUE(slots.count(cmep::kBootKeyring) == 1);
    ZLB_EXPECT_TRUE(slots.count(0x501) == 1);
    if (slots.count(0x501) != 0) {
        const KeyringSlot& slot = slots.find(0x501)->second;
        // 0x5CD64 builds big-endian words from four consecutive bytes, i.e. the
        // staged words are little-endian; word 0 holds the flags.  The 32-byte
        // value therefore has word n in bytes 4n..4n+3 with the low byte first:
        // word 0 (0x11) at byte 0 and word 7 (0x88) at byte 28.  (The old
        // expectation `value[31] == 0x88` could never hold for any uniform word
        // order - it asked for word 0 little-endian and word 7 big-endian.)
        ZLB_EXPECT_EQ(slot.value[0], 0x11);
        ZLB_EXPECT_EQ(slot.value[4], 0x22);
        ZLB_EXPECT_EQ(slot.value[8], 0x33);
        ZLB_EXPECT_EQ(slot.value[28], 0x88);
        ZLB_EXPECT_EQ(slot.value[29], 0x00);
        ZLB_EXPECT_EQ(slot.value[31], 0x00);
        ZLB_EXPECT_EQ(slot.flags, 0x11);
        ZLB_EXPECT_TRUE(slot.present);
    }
    // The SceKeys keyring mirrors the capture: later stages read it from there.
    ZLB_EXPECT_EQ(f.keys.keyring().size(), static_cast<size_t>(2));

    // ClearFlags (0x5C0DC: 0x1C0F020E, 0x5C0E6: 0x1C0F020F).
    f.bus.write32(0xE0030024, 0x1C0F020Eu);
    f.bus.write32(0xE0030024, 0x1C0F020Fu);
    ZLB_EXPECT_EQ(f.cmep.clear_flags_history().size(), static_cast<size_t>(2));
    ZLB_EXPECT_EQ(f.cmep.clear_flags_history()[0].first, 0x20Eu);
    ZLB_EXPECT_EQ(f.cmep.clear_flags_history()[1].first, 0x20Fu);
    ZLB_EXPECT_EQ(cmep_detail::KeyringDevice::clear_mask(f.cmep.clear_flags_history()[0].second), 0x1C0F0000u);
    f.bus.write32(0xE0030024, 0x08000501u);
    ZLB_EXPECT_EQ(cmep_detail::KeyringDevice::clear_index(f.cmep.clear_flags_history()[2].second), 0x501u);

    // QueryFlags (0x5C102/0x5C104): 0x501 must answer 0x10000003 - the low bits
    // are what check_boot_mode tests (`($12 & 3) != 3`), bit 28 is the
    // "keyring valid" bit the CMeP secure kernel compares against (0x801A4C).
    f.bus.write32(0xE0030028, 0x501);
    ZLB_EXPECT_EQ(f.bus.read32(0xE003002C), 0x10000003u);
    f.cmep.set_keyring_flags(1);
    f.bus.write32(0xE0030028, 0x501);
    ZLB_EXPECT_EQ(f.bus.read32(0xE003002C), 0x10000001u);
}

ZLB_TEST(cmep_keyring_locking) {
    Fixture f;
    for (u32 i = 0; i < 8; ++i) f.bus.write32(0xE0030000 + 4 * i, 0xA5A5A5A5u);
    f.bus.write32(0xE0030020, 0x123);
    ZLB_EXPECT_TRUE(f.cmep.captured_keyrings().count(0x123) == 1);
    // 0x5C200 writes 0x08000501: WriteByCmepAllowed is dropped -> locked.
    f.bus.write32(0xE0030024, 0x08000501u);
    const auto& slot = f.cmep.captured_keyrings().find(0x123)->second;
    ZLB_EXPECT_TRUE(slot.locked);
}

// ---------------------------------------------------------------------------
// eMMC crypto window (0xE0070000), strap and boot mode
// ---------------------------------------------------------------------------

ZLB_TEST(cmep_emmc_crypto_and_strap) {
    Fixture f;
    // init_emmc_keyrings (0x5C0E8 / 0x5C0EA).
    f.bus.write32(0xE0070008, 0x020E020Fu);
    f.bus.write32(0xE0070000, 1);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0070008), 0x020E020Fu);
    Device* crypto = f.bus.find_device(0xE0070000);
    ZLB_EXPECT_TRUE(crypto != nullptr && crypto->name() == "CMeP.EmmcCrypto");

    // check_boot_mode (0x5C116): strap bit 0 selects 'A' vs '!'.
    f.cmep.set_strap_bit0(true);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0062020) & 1u, 1u);
    ZLB_EXPECT_EQ(f.cmep.boot_mode(), 0x41);
    f.cmep.set_strap_bit0(false);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0062020) & 1u, 0u);
    f.cmep.set_boot_mode(0x21);
    ZLB_EXPECT_EQ(f.cmep.boot_mode(), 0x21);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0062020) & 1u, 0u);

    // 0xE0064060 is a plain state register (0x5C2BC/0x5C416/0x5C808).
    f.bus.write32(0xE0064060, 0x00010002u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0064060), 0x00010002u);
}

ZLB_TEST(cmep_public_retail_identity_slot_registers) {
    Fixture f;
    const u32 identity = 0xE0058000u + (0x509u << 5u);
    // Public PCH-1001 prefix only; the console-unique remainder is unavailable.
    const u8 prefix[8] = {0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x10};
    ZLB_EXPECT_EQ(identity, 0xE0062120u);
    ZLB_EXPECT_EQ(f.bus.read32(identity), 0x01000000u);
    ZLB_EXPECT_EQ(f.bus.read32(identity + 4u), 0x10000401u);
    for (u32 i = 0; i < sizeof(prefix); ++i) ZLB_EXPECT_EQ(f.bus.read8(identity + i), prefix[i]);
    for (u32 i = sizeof(prefix); i < 32u; ++i) ZLB_EXPECT_EQ(f.bus.read8(identity + i), 0u);
    ZLB_EXPECT_EQ(f.bus.read16(identity + 4u), 0x0401u);
    Device* device = f.bus.find_device(identity);
    ZLB_EXPECT_TRUE(device != nullptr);
    if (device != nullptr) {
        const char* name = device->register_name(identity + 4u);
        ZLB_EXPECT_TRUE(name != nullptr && std::string(name).find("modeled public") != std::string::npos);
        ZLB_EXPECT_TRUE(device->summary().find("console_product=0x0104") != std::string::npos);
    }
    // Overrides are never normalized into a valid product. The saved native
    // all-zero classifier capture proves guest rejection; don't duplicate it.
    f.bus.write32(identity + 4u, 0u);
    ZLB_EXPECT_EQ(f.bus.read32(identity + 4u), 0u);
    f.bus.write16(identity + 4u, 0xFFFFu);
    ZLB_EXPECT_EQ(f.bus.read32(identity + 4u), 0x0000FFFFu);
    f.bus.write8(identity + 7u, 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read32(identity + 4u), 0xA500FFFFu);
    f.bus.write8(identity + 15u, 0x5Au);
    ZLB_EXPECT_EQ(f.bus.read8(identity + 15u), 0x5Au);
    f.cmep.reset();
    for (u32 i = 0; i < sizeof(prefix); ++i) ZLB_EXPECT_EQ(f.bus.read8(identity + i), prefix[i]);
    for (u32 i = sizeof(prefix); i < 32u; ++i) ZLB_EXPECT_EQ(f.bus.read8(identity + i), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0062020u), 0u);
}

ZLB_TEST(cmep_modeled_dram_capacity_slot_registers) {
    Fixture f;
    constexpr u32 base = 0xE0062260u;
    ZLB_EXPECT_EQ(f.bus.read32(base), kermit::kScuSize);
    ZLB_EXPECT_EQ(f.bus.read64(base), static_cast<u64>(kermit::kScuSize));
    ZLB_EXPECT_EQ(f.bus.read16(base), 0u);
    ZLB_EXPECT_EQ(f.bus.read16(base + 2u), kermit::kScuSize >> 16u);
    for (u32 i = 0; i < 32u; ++i) {
        const u32 expected = i < 4u ? ((kermit::kScuSize >> (i * 8u)) & 0xFFu) : 0u;
        ZLB_EXPECT_EQ(f.bus.read8(base + i), expected);
    }
    ZLB_EXPECT_TRUE(std::string(f.cmep.strap_device().register_name(base)).find("0x513") != std::string::npos);
    ZLB_EXPECT_TRUE(f.cmep.strap_device().summary().find("dram_capacity=0x20000000") != std::string::npos);
    // Unaligned partial overrides preserve other lanes, crossing a word edge.
    f.bus.write16(base + 3u, 0xABCDu);
    ZLB_EXPECT_EQ(f.bus.read32(base), 0xCD000000u);
    ZLB_EXPECT_EQ(f.bus.read32(base + 4u), 0xABu);
    ZLB_EXPECT_EQ(f.bus.read16(base + 3u), 0xABCDu);
    f.bus.write32(base, 0xFFFFFFFFu);
    ZLB_EXPECT_EQ(f.bus.read32(base), 0xFFFFFFFFu); // No normalization of raw inputs.
    f.bus.write8(base + 31u, 0x5Au);
    f.cmep.reset();
    ZLB_EXPECT_EQ(f.bus.read32(base), kermit::kScuSize);
    for (u32 i = 4; i < 32u; ++i) ZLB_EXPECT_EQ(f.bus.read8(base + i), 0u);
}

ZLB_TEST(cmep_dram_capacity_drives_native_kprx_buffer_validator) {
    // Genuine FW1.04 kprx bytes 0x80E55E..0x80E6E5 from the authenticated
    // guest payload; see docs/SMC_AUTH_104.md. Run the actual byte loads,
    // 33-bit aperture/end arithmetic and failure response, without copying
    // the guest validator into C++. Stop a valid request before payload copy.
    const u8 validator[] = {
        0x80, 0x6F, 0x21, 0xC3, 0x00, 0xE0, 0x1A, 0x7B, 0x34, 0xC3, 0x14, 0x00, 0x16, 0x45, 0x12, 0x46,
        0x0E, 0x47, 0x0A, 0x4B, 0x3E, 0x05, 0x55, 0xC2, 0x01, 0x00, 0x00, 0xE2, 0xB1, 0x00, 0xFE, 0x50,
        0x3A, 0x05, 0x01, 0x15, 0x5E, 0x06, 0x63, 0xC3, 0x40, 0x00, 0x04, 0xE3, 0xA7, 0x00, 0x01, 0xC0,
        0x00, 0x10, 0x63, 0x00, 0x04, 0xE0, 0xA2, 0x00, 0x21, 0xC3, 0x06, 0xE0, 0x34, 0xC3, 0x60, 0x22,
        0x3C, 0x03, 0xF8, 0xC3, 0x04, 0x00, 0x21, 0xC3, 0x06, 0xE0, 0x34, 0xC3, 0x61, 0x22, 0x3C, 0x03,
        0xF8, 0xC3, 0x05, 0x00, 0x21, 0xC3, 0x06, 0xE0, 0x34, 0xC3, 0x62, 0x22, 0x3C, 0x03, 0xF8, 0xC3,
        0x06, 0x00, 0x21, 0xC3, 0x06, 0xE0, 0x34, 0xC3, 0x63, 0x22, 0x3C, 0x03, 0xF8, 0xC3, 0x07, 0x00,
        0x21, 0xC3, 0x2F, 0x40, 0x34, 0xC3, 0xFF, 0xFF, 0x53, 0x03, 0x07, 0x4A, 0x00, 0xE0, 0x6F, 0x00,
        0x21, 0xC2, 0x00, 0x40, 0x23, 0x9A, 0xA3, 0x03, 0x00, 0x0A, 0x07, 0xA0, 0x33, 0x05, 0x14, 0xA0,
        0x62, 0x95, 0x53, 0x02, 0x00, 0x01, 0x03, 0x0A, 0x0B, 0xA0, 0xA5, 0xE1, 0x27, 0x00, 0x23, 0x03,
        0x48, 0xA0, 0x21, 0xC3, 0x00, 0x1F, 0x34, 0xC3, 0xFF, 0x7F, 0x53, 0x03, 0x04, 0xE0, 0x5F, 0x00,
        0x62, 0x95, 0x53, 0x02, 0x04, 0xE0, 0x5B, 0x00, 0x21, 0xC3, 0x00, 0x1F, 0x34, 0xC3, 0x00, 0x80,
        0x23, 0x03, 0x04, 0xE0, 0x54, 0x00, 0x22, 0xB0, 0x21, 0xC3, 0x85, 0x1F, 0x34, 0xC3, 0xFF, 0xFF,
        0x53, 0x03, 0x04, 0xE0, 0x53, 0x00, 0x63, 0x95, 0x53, 0x03, 0x04, 0xE0, 0x4F, 0x00, 0x21, 0xC0,
        0x86, 0x1F, 0x33, 0x00, 0x04, 0xE0, 0x4A, 0x00, 0xC0, 0xD1, 0x2F, 0x81, 0x50, 0x02, 0x60, 0x03,
        0xE9, 0xDF, 0x0A, 0x00, 0x55, 0xA0, 0xC3, 0xE7, 0x2F, 0x81, 0x65, 0xE7, 0x3F, 0x00, 0xC7, 0xE2,
        0x2F, 0x81, 0xFF, 0x51, 0xAF, 0xE3, 0x1F, 0x81, 0x15, 0xE2, 0x04, 0x00, 0x3A, 0x00, 0x6E, 0xB0,
        0x3E, 0xC1, 0x04, 0x00, 0x22, 0xB0, 0xFE, 0x53, 0x31, 0x11, 0x1E, 0x03, 0x25, 0xE3, 0x0B, 0x00,
        0x1E, 0xC3, 0x08, 0x00, 0xC0, 0xD2, 0x2F, 0x81, 0x20, 0x61, 0x00, 0x56, 0x3E, 0x03, 0x3F, 0x10,
        0x0A, 0xB0, 0x1E, 0xC1, 0x04, 0x00, 0xE1, 0xA1, 0x02, 0x56, 0x50, 0x01, 0xC0, 0xD2, 0x2F, 0x81,
        0x70, 0x03, 0x59, 0xDD, 0x0A, 0x00, 0x04, 0xA0, 0x08, 0x56, 0x21, 0xC3, 0x00, 0xE0, 0x34, 0xC3,
        0x04, 0x00, 0x64, 0xC6, 0x01, 0x00, 0x3A, 0x06, 0x24, 0xB0, 0x21, 0xC3, 0xFF, 0x1E, 0x34, 0xC3,
        0xFF, 0xFF, 0x53, 0x03, 0x04, 0xE0, 0x9F, 0xFF, 0x10, 0xB0, 0x21, 0xC3, 0x83, 0x1F, 0x34, 0xC3,
        0xFF, 0xFF, 0x53, 0x03, 0x04, 0xE0, 0xAA, 0xFF, 0x04, 0x56, 0xD0, 0xBF, 0x0F, 0x47, 0x13, 0x46,
        0x17, 0x45, 0x0B, 0x4B, 0x20, 0x4F, 0xBE, 0x10,
    };
    struct Request { u32 pa, length, capacity; bool accepted; };
    const Request requests[] = {
        {0x40350200u, 0x190u, kermit::kScuSize, true}, // Captured command10001.
        {0x5FFFFE70u, 0x190u, kermit::kScuSize, true}, // End equals aperture end.
        {0x5FFFFE74u, 0x190u, kermit::kScuSize, false},
        {0x60000000u, 0x190u, kermit::kScuSize, false},
        {0x402FFFF0u, 0x190u, kermit::kScuSize, false}, // Protected first3MiB.
        {0x40350200u, 0x190u, 0u, false},             // Previous model input.
        // Raw overrides exercise carry at the 32-bit address-space boundary.
        {0xFFFFFFF0u, 0x190u, 0xC0000000u, false},
        {0xFFFFFE70u, 0x190u, 0xC0000000u, true},
        {0x40350200u, 0x3Fu, kermit::kScuSize, false},
        {0x40350200u, 0x1004u, kermit::kScuSize, false},
    };
    for (const Request& request : requests) {
        Fixture f;
        auto arm = std::make_unique<Bus>();
        f.cmep.install_arm_mailbox(*arm);
        f.bus.load(0x80E55Eu, validator, sizeof(validator), "native kprx PA validator");
        f.bus.add_ram("native request header", 0x1000u, request.pa & ~0xFFFu, "validator input");
        f.bus.write32(request.pa, request.length);
        f.bus.write32(0xE0062260u, request.capacity);
        arm->write32(0xE0000014u, request.pa | 1u);
        MePCore cpu(f.bus);
        cpu.reset(0x80E55Eu);
        cpu.r[15] = 0x81E000u;
        for (unsigned step = 0; step < 200u; ++step) {
            if (cpu.pc == 0x80E646u || cpu.pc == 0x80E6B6u) break;
            const StepResult result = cpu.step();
            ZLB_EXPECT_FALSE(result.faulted);
            if (result.faulted) break;
        }
        ZLB_EXPECT_EQ(cpu.pc, request.accepted ? 0x80E646u : 0x80E6B6u);
        ZLB_EXPECT_EQ(f.bus.read32(0xE0000014u), 0u); // Genuine incoming ACK.
        ZLB_EXPECT_EQ(f.bus.read32(0xE0000004u), request.accepted ? 0u : 5u);
        ZLB_EXPECT_EQ(f.bus.read32(request.pa), request.length);
    }
}

// ---------------------------------------------------------------------------
// Mailboxes (0xE0000000)
// ---------------------------------------------------------------------------

ZLB_TEST(cmep_mailboxes) {
    Fixture f;
    // ARM -> CMeP image hand-off (0x5C57A): bit 0 is "data present".
    f.cmep.set_arm_to_cmep_command(0x1F000001u);
    ZLB_EXPECT_EQ(f.cmep.arm_to_cmep_command(), 0x1F000001u);
    ZLB_EXPECT_EQ(f.cmep.arm_to_cmep_command() & ~3u, 0x1F000000u);

    // CMeP -> ARM status: 1 = success (0x5C5F0), 2 = failure (0x5C616).
    ZLB_EXPECT_FALSE(f.cmep.reported_success());
    ZLB_EXPECT_FALSE(f.cmep.reported_failure());
    f.cmep.set_cmep_status(1);
    ZLB_EXPECT_TRUE(f.cmep.reported_success());
    f.cmep.set_cmep_status(2);
    ZLB_EXPECT_TRUE(f.cmep.reported_failure());

    // Debug mailboxes, acknowledged with 0xFFFFFFFF (0x5E5AA..0x5E5B8).
    ZLB_EXPECT_EQ(f.bus.read32(0xE0000028), 0xFFFFFFFFu);
    f.bus.write32(0xE0000020, 0x12345678u);
    f.bus.write32(0xE0000024, 0x9ABCDEF0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0000020), 0x12345678u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0000024), 0x9ABCDEF0u);
}

ZLB_TEST(cmep_mailbox_native_shared_buffer_handshake) {
    Fixture f;
    auto arm = std::make_unique<Bus>();
    f.cmep.install_arm_mailbox(*arm);
    constexpr u32 status = 0xE0000000u;
    constexpr u32 command = 0xE0000010u;
    constexpr u32 shared_pa = 0x40378000u;

    // Genuine 1.04 secure_kernel posts101; native ARM Smsched acknowledges
    // that status before submitting a shared-buffer PA and a separate doorbell.
    f.bus.write32(status, 0x101u);
    ZLB_EXPECT_EQ(arm->read32(status), 0x101u);
    arm->write32(status, 0x101u);
    ZLB_EXPECT_EQ(f.bus.read32(status), 0u);
    arm->write32(command, shared_pa);
    arm->write32(command, 1u);
    ZLB_EXPECT_EQ(f.bus.read32(command), shared_pa | 1u);
    ZLB_EXPECT_EQ(f.bus.read32(command) & ~3u, shared_pa);

    // The CMeP consumes the request and clears the incoming word with -1
    // (80042E), then posts102, which ARM acknowledges through its own port.
    f.bus.write32(command, 0xFFFFFFFFu);
    ZLB_EXPECT_EQ(arm->read32(command), 0u);
    f.bus.write32(status, 0x102u);
    ZLB_EXPECT_EQ(arm->read32(status), 0x102u);
    arm->write32(status, 0x102u);
    ZLB_EXPECT_EQ(f.bus.read32(status), 0u);

    // A second submission must not retain the earlier address or its doorbell.
    constexpr u32 next_pa = 0x40500000u;
    arm->write32(command, next_pa);
    arm->write32(command, 1u);
    ZLB_EXPECT_EQ(f.bus.read32(command), next_pa | 1u);
}

ZLB_TEST(cmep_mailbox_endpoints_ack_only_selected_bits) {
    Fixture f;
    auto arm = std::make_unique<Bus>();
    f.cmep.install_arm_mailbox(*arm);

    // Scheduler mailboxes1..3 use the same directional set/clear contract.
    // A receiver ACK must leave separately pending notifications intact.
    for (u32 i = 0; i < 4u; ++i) {
        const u32 incoming_arm = 0xE0000000u + i * 4u;
        const u32 incoming_cmep = 0xE0000010u + i * 4u;
        f.bus.write32(incoming_arm, 0x101u);
        f.bus.write32(incoming_arm, 0x10000u);
        arm->write32(incoming_arm, 0x101u);
        ZLB_EXPECT_EQ(arm->read32(incoming_arm), 0x10000u);
        arm->write32(incoming_arm, 0x10000u);
        ZLB_EXPECT_EQ(f.bus.read32(incoming_arm), 0u);

        arm->write32(incoming_cmep, 0xA5000101u);
        f.bus.write32(incoming_cmep, 0x101u);
        ZLB_EXPECT_EQ(arm->read32(incoming_cmep), 0xA5000000u);
        f.bus.write32(incoming_cmep, 0xFFFFFFFFu);
        ZLB_EXPECT_EQ(arm->read32(incoming_cmep), 0u);
    }

    // Byte ACKs clear their own lane, preserving both adjacent bytes and bits
    // not included in the mask. Both buses still read the same register word.
    f.bus.write32(0xE0000000u, 0xA5C30007u);
    arm->write8(0xE0000002u, 0xC1u);
    ZLB_EXPECT_EQ(arm->read32(0xE0000000u), 0xA5020007u);
    ZLB_EXPECT_EQ(f.bus.read8(0xE0000002u), 2u);

    // Boot-ROM stand-in setters explicitly assign values, rather than adopting
    // a guest endpoint's set/clear behavior.
    f.cmep.set_cmep_status(1u);
    ZLB_EXPECT_EQ(arm->read32(0xE0000000u), 1u);
    f.cmep.set_arm_to_cmep_command(0x40378001u);
    f.cmep.set_arm_to_cmep_command(1u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0000010u), 1u);
    f.bus.reset();
    ZLB_EXPECT_EQ(arm->read32(0xE0000000u), 0u);
    ZLB_EXPECT_EQ(arm->read32(0xE0000010u), 0u);
}

ZLB_TEST(cmep_mailbox_doorbell_and_status_irq_levels) {
    Fixture f;
    auto arm = std::make_unique<Bus>();
    f.cmep.install_arm_mailbox(*arm);
    std::array<bool, 4> to_cmep{};
    std::array<bool, 4> to_arm{};
    unsigned cmep_edges = 0;
    f.cmep.set_mailbox_irq_callbacks(
        [&](unsigned channel, bool asserted) {
            to_cmep[channel] = asserted;
            ++cmep_edges;
        },
        [&](unsigned channel, bool asserted) { to_arm[channel] = asserted; });

    // Native Smsched submits PA first, then bit 0. Only the completed request
    // may interrupt CMeP; clearing the doorbell drops the level even while the
    // address bits remain readable until the consumer finishes its ACK.
    const unsigned initial_edges = cmep_edges;
    arm->write32(0xE0000010u, 0x401402C0u);
    ZLB_EXPECT_FALSE(to_cmep[0]);
    ZLB_EXPECT_EQ(cmep_edges, initial_edges);
    arm->write32(0xE0000010u, 1u);
    ZLB_EXPECT_TRUE(to_cmep[0]);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0000010u), 0x401402C1u);
    f.bus.write32(0xE0000010u, 1u);
    ZLB_EXPECT_FALSE(to_cmep[0]);
    ZLB_EXPECT_EQ(arm->read32(0xE0000010u), 0x401402C0u);
    f.bus.write32(0xE0000010u, 0xFFFFFFFFu);
    ZLB_EXPECT_EQ(arm->read32(0xE0000010u), 0u);

    // The subsequent genuine RVK command asserts the same interrupt without
    // a fabricated response. Its consumer ACK clears the pending request.
    arm->write32(0xE0000010u, 0x80A01u);
    ZLB_EXPECT_TRUE(to_cmep[0]);
    f.bus.write32(0xE0000010u, 0xFFFFFFFFu);
    ZLB_EXPECT_FALSE(to_cmep[0]);

    // ARM status has pending notifications above bit 0 too. A partial ACK
    // leaves the line asserted until every remaining pending bit is consumed.
    f.bus.write32(0xE0000000u, 0x101u);
    ZLB_EXPECT_TRUE(to_arm[0]);
    arm->write32(0xE0000000u, 1u);
    ZLB_EXPECT_TRUE(to_arm[0]);
    ZLB_EXPECT_EQ(arm->read32(0xE0000000u), 0x100u);
    arm->write32(0xE0000000u, 0x100u);
    ZLB_EXPECT_FALSE(to_arm[0]);
    f.bus.write32(0xE0000000u, 0x10000u);
    ZLB_EXPECT_TRUE(to_arm[0]);
    arm->write32(0xE0000000u, 0x10000u);
    ZLB_EXPECT_FALSE(to_arm[0]);
}

ZLB_TEST(cmep_mailbox_clear_aliases_and_irq_lifecycle) {
    Fixture f;
    auto arm = std::make_unique<Bus>();
    f.cmep.install_arm_mailbox(*arm);
    std::array<bool, 4> to_cmep{};
    std::array<bool, 4> to_arm{};

    // Installing the CPU wiring after a pending request must publish its
    // current levels, including host-assigned boot-ROM handshake values.
    f.cmep.set_arm_to_cmep_command(1u);
    f.cmep.set_cmep_status(0x101u);
    f.cmep.set_mailbox_irq_callbacks(
        [&](unsigned channel, bool asserted) { to_cmep[channel] = asserted; },
        [&](unsigned channel, bool asserted) { to_arm[channel] = asserted; });
    ZLB_EXPECT_TRUE(to_cmep[0]);
    ZLB_EXPECT_TRUE(to_arm[0]);
    f.cmep.set_arm_to_cmep_command(0u);
    f.cmep.set_cmep_status(0u);
    ZLB_EXPECT_FALSE(to_cmep[0]);
    ZLB_EXPECT_FALSE(to_arm[0]);

    // Producers cancel their own outstanding word through the +0x40 W1C
    // alias: CMeP uses +0x44/48/4C, ARM uses +0x54/58/5C. Every channel must
    // share its data and pending interrupt state across the two bus endpoints.
    for (unsigned channel = 0; channel < 4u; ++channel) {
        const u32 status = 0xE0000000u + channel * 4u;
        const u32 command = 0xE0000010u + channel * 4u;
        f.bus.write32(status, 0x10000u);
        arm->write32(command, 0x80A01u);
        ZLB_EXPECT_TRUE(to_arm[channel]);
        ZLB_EXPECT_TRUE(to_cmep[channel]);
        f.bus.write32(status + 0x40u, 0xFFFFFFFFu);
        arm->write32(command + 0x40u, 0xFFFFFFFFu);
        ZLB_EXPECT_EQ(arm->read32(status), 0u);
        ZLB_EXPECT_EQ(f.bus.read32(command), 0u);
        ZLB_EXPECT_FALSE(to_arm[channel]);
        ZLB_EXPECT_FALSE(to_cmep[channel]);
    }

    // Clear aliases acknowledge selected bits and byte lanes from either
    // endpoint. Cancelling one notification preserves the other pending bit.
    f.bus.write32(0xE0000008u, 0xA5C30007u);
    arm->write8(0xE000004Au, 0xC1u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0000008u), 0xA5020007u);
    ZLB_EXPECT_TRUE(to_arm[2]);
    f.bus.write32(0xE0000048u, 0xFFFFFFFFu);
    ZLB_EXPECT_FALSE(to_arm[2]);

    // Bus reset clears the observable words and every asserted direction,
    // while retaining the installed callbacks for a later cold boot.
    for (unsigned channel = 0; channel < 4u; ++channel) {
        f.bus.write32(0xE0000000u + channel * 4u, 0x101u);
        arm->write32(0xE0000010u + channel * 4u, 1u);
    }
    f.bus.reset();
    for (unsigned channel = 0; channel < 4u; ++channel) {
        ZLB_EXPECT_FALSE(to_arm[channel]);
        ZLB_EXPECT_FALSE(to_cmep[channel]);
        ZLB_EXPECT_EQ(arm->read32(0xE0000000u + channel * 4u), 0u);
        ZLB_EXPECT_EQ(f.bus.read32(0xE0000010u + channel * 4u), 0u);
    }
    arm->write32(0xE0000010u, 1u);
    ZLB_EXPECT_TRUE(to_cmep[0]);
}

// ---------------------------------------------------------------------------
// GPIO (0xE20A0000) - mailbox_debug_sc (0x5E4E4)
// ---------------------------------------------------------------------------

ZLB_TEST(cmep_gpio_default_has_no_jig_peer_and_separates_direction_from_output) {
    Fixture f;
    f.cmep.reset();
    f.bus.write32(0xE20A0000, 8);  // genuine mailbox_debug_sc direction setup
    ZLB_EXPECT_EQ(f.bus.read32(0xE20A0004) & 0x10u, 0u); // absent peer, native no-JIG path
    ZLB_EXPECT_EQ(f.bus.read32(0xE20A0004) & 0x10u, 0u); // reads never consume a line
    f.bus.write32(0xE20A0008, 8);
    ZLB_EXPECT_EQ(f.bus.read32(0xE20A0034) & 8u, 8u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE20A0000), 8u);
    f.bus.write32(0xE20A000C, 8);
    ZLB_EXPECT_EQ(f.bus.read32(0xE20A0034) & 8u, 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE20A0000), 8u);
}

// ---------------------------------------------------------------------------
// SC bridge registers (0xE3100000) - the program-and-poll boot paths
// ---------------------------------------------------------------------------

ZLB_TEST(cmep_sc_bridge_polling) {
    Fixture f;
    // boot_path_bit5 (0x5C246..0x5C28C) and bit6 (0x5C3BC..0x5C3E6).
    f.bus.write32(0xE31020A0, 1);
    ZLB_EXPECT_NE(f.bus.read32(0xE31020A0), 0u);
    f.bus.write32(0xE31010A0, 0);
    ZLB_EXPECT_NE(f.bus.read32(0xE31010A0), 0u);
    f.bus.write32(0xE31010A4, 1);
    ZLB_EXPECT_EQ(f.bus.read32(0xE31010A4), 0u);
    f.bus.write32(0xE31020A4, 1);
    ZLB_EXPECT_EQ(f.bus.read32(0xE31020A4), 0u);
}


// ---------------------------------------------------------------------------
// Bigmac (0xE0050000)
// ---------------------------------------------------------------------------

ZLB_TEST(bigmac_aes_vectors) {
    // FIPS-197 C.1 (AES-128) and C.3 (AES-256) plus an SP 800-38A CBC block.
    u8 key128[16];
    u8 block[16];
    for (int i = 0; i < 16; ++i) key128[i] = static_cast<u8>(i);
    // The C.1 *plaintext* is 00112233445566778899aabbccddeeff, not 00..0f: the
    // expected ciphertext below belongs to that input (the test used to feed the
    // key bytes as the plaintext, so it could never match).
    for (int i = 0; i < 16; ++i) block[i] = static_cast<u8>(i * 0x11);
    const u8 want128[16] = {0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
                            0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a};
    u8 out[16];
    u8 plain128[16];
    std::memcpy(plain128, block, 16);
    cmep_detail::aes_encrypt_block(key128, 128, block, out);
    ZLB_EXPECT_EQ(std::memcmp(out, want128, 16), 0);
    cmep_detail::aes_decrypt_block(key128, 128, out, block);
    for (int i = 0; i < 16; ++i) ZLB_EXPECT_EQ(block[i], plain128[i]);

    u8 key256[32];
    for (int i = 0; i < 32; ++i) key256[i] = static_cast<u8>(i);
    const u8 plain256[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                             0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
    const u8 want256[16] = {0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf,
                            0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49, 0x60, 0x89};
    cmep_detail::aes_encrypt_block(key256, 256, plain256, out);
    ZLB_EXPECT_EQ(std::memcmp(out, want256, 16), 0);
}

ZLB_TEST(bigmac_hash_vectors) {
    const u8 abc[3] = {'a', 'b', 'c'};
    u8 sha1_out[20];
    u8 sha256_out[32];
    cmep_detail::sha1_digest(abc, 3, sha1_out);
    cmep_detail::sha256_digest(abc, 3, sha256_out);
    const u8 want_sha1[20] = {0xA9, 0x99, 0x3E, 0x36, 0x47, 0x06, 0x81, 0x6A, 0xBA, 0x3E,
                              0x25, 0x71, 0x78, 0x50, 0xC2, 0x6C, 0x9C, 0xD0, 0xD8, 0x9D};
    const u8 want_sha256[32] = {0xBA, 0x78, 0x16, 0xBF, 0x8F, 0x01, 0xCF, 0xEA, 0x41, 0x41, 0x40,
                                0xDE, 0x5D, 0xAE, 0x22, 0x23, 0xB0, 0x03, 0x61, 0xA3, 0x96, 0x17,
                                0x7A, 0x9C, 0xB4, 0x10, 0xFF, 0x61, 0xF2, 0x00, 0x15, 0xAD};
    ZLB_EXPECT_EQ(std::memcmp(sha1_out, want_sha1, 20), 0);
    ZLB_EXPECT_EQ(std::memcmp(sha256_out, want_sha256, 32), 0);
}

ZLB_TEST(bigmac_keyring_write1_register_protocol) {
    Fixture f;
    // img_proc_5C67E calls keyring_write1 at 0x5C69E with $1 = 8 (keyring),
    // $2 = 0x5E704 (source), $3 = 32 (length), $4 = 0x206 (flags).  bigmac_cmd
    // maps them to +0x04/+0x00/+0x08/+0x10 and ORs bit 28 into the function
    // because the keyring index is below 0x1000 (0x5CD4E).
    std::vector<u8> key32(32);
    for (int i = 0; i < 32; ++i) key32[i] = static_cast<u8>(0x40 + i);
    f.bus.load(0x5E704, key32.data(), 32, "key");
    f.bus.write32(0xE0050004, 8);            // arg0 = keyring index
    f.bus.write32(0xE0050000, 0x0005E704u);  // pointer = source
    f.bus.write32(0xE0050008, 32);           // length
    f.bus.write32(0xE0050014, 0);            // extra
    f.bus.write32(0xE0050010, 0x206);        // flags
    f.bus.write32(0xE005000C, 0x10000301u);  // keyring_write1 | bit 28
    f.bus.write32(0xE005001C, 1);            // start

    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024) & 1u, 0u);
    const auto& slots = f.cmep.captured_keyrings();
    ZLB_EXPECT_TRUE(slots.count(8) == 1);
    if (slots.count(8) != 0) {
        const KeyringSlot& slot = slots.find(8)->second;
        for (int i = 0; i < 32; ++i) ZLB_EXPECT_EQ(slot.value[i], static_cast<u8>(0x40 + i));
        ZLB_EXPECT_EQ(slot.flags, 0x0206u);
    }
    ZLB_EXPECT_EQ(f.cmep.bigmac_operations(), 1u);
}

ZLB_TEST(bigmac_rng_is_deterministic) {
    Fixture f;
    const u64 first_state = f.cmep.bigmac_device().rng().state;
    for (int i = 0; i < 8; ++i) {
        ZLB_EXPECT_EQ(f.cmep.bigmac_device().execute(
                          0, 0, 0, nullptr,
                          static_cast<u32>(cmep_detail::BigmacFunction::Rng)),
                      0);
    }
    const u64 after = f.cmep.bigmac_device().rng().state;
    ZLB_EXPECT_NE(after, first_state);
    // Two blocks seeded identically produce the same stream (documented model).
    Fixture g;
    ZLB_EXPECT_EQ(g.cmep.bigmac_device().rng().state, first_state);
    // `f` has already consumed eight RNG commands above; give `g` the same
    // history, otherwise the two streams are simply at different offsets.
    for (int i = 0; i < 8; ++i) {
        g.cmep.bigmac_device().execute(0, 0, 0, nullptr,
                                       static_cast<u32>(cmep_detail::BigmacFunction::Rng));
    }
    Bus& bus_a = f.bus;
    Bus& bus_b = g.bus;
    for (int i = 0; i < 4; ++i) {
        f.cmep.bigmac_device().execute(0, 0, 0, nullptr,
                                       static_cast<u32>(cmep_detail::BigmacFunction::Rng));
        g.cmep.bigmac_device().execute(0, 0, 0, nullptr,
                                       static_cast<u32>(cmep_detail::BigmacFunction::Rng));
        ZLB_EXPECT_EQ(bus_a.read32(0xE0050200), bus_b.read32(0xE0050200));
    }
}

ZLB_TEST(bigmac_aes_cbc_through_registers) {
    Fixture f;
    // Encrypt a 32-byte block with the known key, then decrypt it through the
    // register protocol and compare.  The key and IV are staged in the +0x200
    // window: key in words 0-3, IV in words 4-7 (bigmac_cmd stores +0x10 last).
    u8 key[16];
    u8 iv[16];
    std::memcpy(key, kEncKey, 16);
    std::memcpy(iv, kEncIv, 16);
    std::vector<u8> plain(32);
    for (int i = 0; i < 32; ++i) plain[i] = static_cast<u8>(i * 3 + 1);
    std::vector<u8> cipher = plain;
    cmep_detail::aes_cbc_crypt(key, 128, iv, cipher.data(), cipher.size(), true);
    f.bus.load(0x41000, cipher.data(), cipher.size(), "cipher");

    for (int i = 0; i < 4; ++i) {
        const u32 word = static_cast<u32>(key[4 * i]) | (static_cast<u32>(key[4 * i + 1]) << 8) |
                         (static_cast<u32>(key[4 * i + 2]) << 16) |
                         (static_cast<u32>(key[4 * i + 3]) << 24);
        f.bus.write32(0xE0050200 + 4 * i, word);
    }
    for (int i = 0; i < 4; ++i) {
        const u32 word = static_cast<u32>(iv[4 * i]) | (static_cast<u32>(iv[4 * i + 1]) << 8) |
                         (static_cast<u32>(iv[4 * i + 2]) << 16) |
                         (static_cast<u32>(iv[4 * i + 3]) << 24);
        f.bus.write32(0xE0050210 + 4 * i, word);
    }
    f.bus.write32(0xE0050004, 0x0100u);
    f.bus.write32(0xE0050000, 0x00041000u);
    f.bus.write32(0xE0050008, 32);
    f.bus.write32(0xE0050014, 0);
    f.bus.write32(0xE0050010, 128);
    f.bus.write32(0xE005000C, static_cast<u32>(cmep_detail::BigmacFunction::AesCbcDecrypt));
    f.bus.write32(0xE005001C, 1);

    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024) & 1u, 0u);
    std::vector<u8> decoded(32);
    f.bus.read_bytes(0x41000, decoded.data(), decoded.size());
    for (size_t i = 0; i < plain.size(); ++i) ZLB_EXPECT_EQ(decoded[i], plain[i]);
}

ZLB_TEST(bigmac_native_dma_copies_arm_ram_to_private_ram) {
    Fixture f;
    // 0x80287C -> 0x8055D6 reads the first 48 bytes of the staged RVK image
    // into the native secure-kernel header buffer through function zero.
    std::array<u8, 48> source{};
    for (size_t i = 0; i < source.size(); ++i) source[i] = static_cast<u8>(i * 7u + 3u);
    f.bus.load(0x40008F00u, source.data(), source.size(), "native_dma_source");
    f.bus.memset_bytes(0x808FEFu, 0xA5u, source.size() + 2u);
    issue_native_bigmac(f, 0x40008F00u, 0x808FF0u, source.size(), 0u);
    std::array<u8, 48> copied{};
    f.bus.read_bytes(0x808FF0u, copied.data(), copied.size());
    ZLB_EXPECT_EQ(std::memcmp(copied.data(), source.data(), source.size()), 0);
    ZLB_EXPECT_EQ(f.bus.read8(0x808FEFu), 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read8(0x809020u), 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005001Cu), 0u);
    ZLB_EXPECT_EQ(f.cmep.bigmac_operations(), 1u);
}

ZLB_TEST(bigmac_native_zero_fill_executes_genuine_module_wipe_wrapper) {
    Fixture f;
    const auto image = read_file(resolve_workspace_path(
        "Vita_104_Firmware/Out/SLB2_dec/secure_kernel.bin"));
    ZLB_EXPECT_TRUE(image.has_value());
    if (!image) return;
    ZLB_EXPECT_TRUE(f.bus.load(cmep::kPrivateBase, image->data(), image->size(),
                               "native_secure_kernel"));
    auto* low = f.bus.region_at(cmep::kRamBase, cmep::kRamSize);
    auto* priv = f.bus.region_at(cmep::kPrivateBase);
    ZLB_EXPECT_TRUE(low != nullptr && priv != nullptr);
    if (!low || !priv) return;
    low->external = priv->bytes();
    f.bus.rebuild_map();

    // Execute the unchanged 0x8055A0 wrapper with the live module-cleanup
    // arguments. Its source=0 is not a boot-SRAM offset. With the real SRAM
    // alias, the old AES interpretation destroys code and its return stack.
    constexpr u32 destination = 0x80A000u;
    constexpr u32 length = 0x16000u;
    constexpr u32 returned = 0x808000u;
    f.bus.memset_bytes(destination - 1u, 0xA5u, length + 2u);
    std::array<u8, 32> stale_key{};
    stale_key.fill(0x5Au);
    stage_bigmac_key(f, stale_key.data(), stale_key.size());
    f.bus.write32(0xE0050014u, 0xFFFFFFF0u);  // stale, unmapped IV is ignored
    MePCore cpu(f.bus);
    cpu.reset(0x8055A0u);
    cpu.psw = 0;
    cpu.r[1] = destination;
    cpu.r[2] = 0;
    cpu.r[3] = length;
    cpu.r[4] = 0;
    cpu.r[15] = 0x808F80u;
    cpu.lp = returned;
    for (unsigned i = 0; i < 300 && cpu.get_pc() != returned; ++i) {
        const auto step = cpu.step();
        ZLB_EXPECT_FALSE(step.faulted);
        if (step.faulted) break;
    }
    ZLB_EXPECT_EQ(cpu.get_pc(), returned);
    ZLB_EXPECT_EQ(cpu.r[0], 0u);
    ZLB_EXPECT_EQ(cpu.r[15], 0x808F80u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050000u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050004u), destination);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050008u), length);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005000Cu), 0xCu);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050034u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
    ZLB_EXPECT_EQ(f.cmep.bigmac_operations(), 1u);
    ZLB_EXPECT_EQ(f.cmep.bigmac_device().last_op(), BigmacOp::None);
    std::vector<u8> kernel(image->size());
    f.bus.read_bytes(cmep::kPrivateBase, kernel.data(), kernel.size());
    ZLB_EXPECT_EQ(std::memcmp(kernel.data(), image->data(), kernel.size()), 0);
    ZLB_EXPECT_EQ(f.bus.read32(cmep::kRamBase), f.bus.read32(cmep::kPrivateBase));
    std::vector<u8> arena(length);
    f.bus.read_bytes(destination, arena.data(), arena.size());
    ZLB_EXPECT_TRUE(arena == std::vector<u8>(length, 0));
    ZLB_EXPECT_EQ(f.bus.read8(destination - 1u), 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read8(destination + length), 0xA5u);
}

ZLB_TEST(bigmac_native_zero_fill_byte_ranges_and_errors) {
    Fixture f;
    // An odd byte count crossing a page must clear exactly the requested
    // bytes; neither AES alignment nor source/IV mapping applies to a fill.
    constexpr u32 destination = 0x50FF3u;
    constexpr u32 length = 37u;
    f.bus.memset_bytes(destination - 1u, 0xA5u, length + 2u);
    f.bus.write32(0xE0050034u, 0u);
    issue_native_bigmac(f, 0u, destination, length, 0xCu, 0xFFFFFFF0u);
    std::array<u8, length> cleared{};
    std::array<u8, length> result{};
    f.bus.read_bytes(destination, result.data(), result.size());
    ZLB_EXPECT_EQ(std::memcmp(result.data(), cleared.data(), result.size()), 0);
    ZLB_EXPECT_EQ(f.bus.read8(destination - 1u), 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read8(destination + length), 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);

    f.bus.add_ram("native_fill_destination", 16u, 0x10000000u, "partial fill range");
    f.bus.memset_bytes(0x10000000u, 0xA5u, 16u);
    issue_native_bigmac(f, 0u, 0x10000000u, 32u, 0xCu);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0x10000000u), 0xA5A5A5A5u);
    ZLB_EXPECT_EQ(f.bus.read32(0x1000000Cu), 0xA5A5A5A5u);
    issue_native_bigmac(f, 0u, 0xFFFFFFF0u, 32u, 0xCu);
    ZLB_EXPECT_NE(f.bus.read32(0xE005003Cu), 0u);
    issue_native_bigmac(f, 0u, 0x100u, 16u, 0xCu);
    ZLB_EXPECT_NE(f.bus.read32(0xE005003Cu), 0u);

    // No verified nonzero pattern protocol is claimed; reject before writing.
    f.bus.memset_bytes(destination, 0xA5u, length);
    f.bus.write32(0xE0050034u, 0x12345678u);
    issue_native_bigmac(f, 0u, destination, length, 0xCu);
    ZLB_EXPECT_NE(f.bus.read32(0xE005003Cu), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(destination), 0xA5A5A5A5u);
    f.bus.write32(0xE0050034u, 0u);
    issue_native_bigmac(f, 0u, 0x100u, 0u, 0xCu);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
}

ZLB_TEST(bigmac_native_dma_rejects_unmapped_and_wrapping_ranges) {
    Fixture f;
    const std::array<u8, 32> source{};
    const std::array<u8, 16> destination = {0xA5u};
    f.bus.load(0x50000u, source.data(), source.size(), "native_dma_source");
    f.bus.add_ram("native_dma_destination", 16u, 0x10000000u, "partial DMA range");
    f.bus.load(0x10000000u, destination.data(), destination.size(), "native_dma_destination");
    // The mapped prefix must remain intact when the remainder is unmapped.
    issue_native_bigmac(f, 0x50000u, 0x10000000u, 32u, 0u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    std::array<u8, 16> unchanged{};
    f.bus.read_bytes(0x10000000u, unchanged.data(), unchanged.size());
    ZLB_EXPECT_EQ(std::memcmp(unchanged.data(), destination.data(), unchanged.size()), 0);
    issue_native_bigmac(f, 0x100u, 0x50000u, 16u, 0u);
    ZLB_EXPECT_NE(f.bus.read32(0xE005003Cu), 0u);
    // Native addresses are absolute: low unmapped addresses must not be
    // redirected into the legacy boot SRAM staging buffer.
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    issue_native_bigmac(f, 0xFFFFFFF0u, 0x50000u, 32u, 0u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
}

ZLB_TEST(bigmac_native_aes256_cbc_register_protocol) {
    Fixture f;
    // NIST SP 800-38A F.2.6: independent AES-256-CBC known-answer bytes.
    // This verifies all 32 key bytes, a separate IV, CBC chaining, and the
    // native source/destination layout rather than the old in-place protocol.
    const u8 key[32] = {
        0x60, 0x3D, 0xEB, 0x10, 0x15, 0xCA, 0x71, 0xBE,
        0x2B, 0x73, 0xAE, 0xF0, 0x85, 0x7D, 0x77, 0x81,
        0x1F, 0x35, 0x2C, 0x07, 0x3B, 0x61, 0x08, 0xD7,
        0x2D, 0x98, 0x10, 0xA3, 0x09, 0x14, 0xDF, 0xF4};
    const u8 iv[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                       0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
    const u8 cipher[32] = {
        0xF5, 0x8C, 0x4C, 0x04, 0xD6, 0xE5, 0xF1, 0xBA,
        0x77, 0x9E, 0xAB, 0xFB, 0x5F, 0x7B, 0xFB, 0xD6,
        0x9C, 0xFC, 0x4E, 0x96, 0x7E, 0xDB, 0x80, 0x8D,
        0x67, 0x9F, 0x77, 0x7B, 0xC6, 0x70, 0x2C, 0x7D};
    const u8 plain[32] = {
        0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96,
        0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A,
        0xAE, 0x2D, 0x8A, 0x57, 0x1E, 0x03, 0xAC, 0x9C,
        0x9E, 0xB7, 0x6F, 0xAC, 0x45, 0xAF, 0x8E, 0x51};
    f.bus.load(0x40008F00u, cipher, sizeof(cipher), "native_aes_source");
    f.bus.load(0x806000u, iv, sizeof(iv), "native_aes_iv");
    stage_bigmac_key(f, key, sizeof(key));
    issue_native_bigmac(f, 0x40008F00u, 0x808FF0u, sizeof(cipher), 0x238Au, 0x806000u);
    u8 decoded[32];
    f.bus.read_bytes(0x808FF0u, decoded, sizeof(decoded));
    ZLB_EXPECT_EQ(std::memcmp(decoded, plain, sizeof(plain)), 0);
    u8 retained_cipher[32];
    f.bus.read_bytes(0x40008F00u, retained_cipher, sizeof(retained_cipher));
    ZLB_EXPECT_EQ(std::memcmp(retained_cipher, cipher, sizeof(cipher)), 0);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
    ZLB_EXPECT_EQ(f.cmep.bigmac_device().last_key_bits(), 256u);
    ZLB_EXPECT_EQ(std::memcmp(f.cmep.bigmac_device().last_key().data(), key, sizeof(key)), 0);
    ZLB_EXPECT_EQ(std::memcmp(f.cmep.bigmac_device().last_iv().data(), iv, sizeof(iv)), 0);
    // The next native decrypt may be in-place, using the same staged key.
    issue_native_bigmac(f, 0x40008F00u, 0x40008F00u, sizeof(cipher), 0x238Au, 0x806000u);
    f.bus.read_bytes(0x40008F00u, decoded, sizeof(decoded));
    ZLB_EXPECT_EQ(std::memcmp(decoded, plain, sizeof(plain)), 0);
    ZLB_EXPECT_EQ(f.cmep.bigmac_device().aes_operations(), 2u);
}

ZLB_TEST(bigmac_native_aes256_cbc_accepts_zero_window_key) {
    Fixture f;
    // AES-256 with an all-zero key/IV encrypts one zero block to these fixed
    // bytes. Bit 7 selects a valid window key even when every key word is zero.
    const u8 cipher[16] = {0xDC, 0x95, 0xC0, 0x78, 0xA2, 0x40, 0x89, 0x89,
                           0xAD, 0x48, 0xA2, 0x14, 0x92, 0x84, 0x20, 0x87};
    const u8 zero_iv[16] = {};
    f.bus.load(0x50000u, cipher, sizeof(cipher), "native_zero_key_cipher");
    f.bus.load(0x50100u, zero_iv, sizeof(zero_iv), "native_zero_key_iv");
    issue_native_bigmac(f, 0x50000u, 0x50000u, sizeof(cipher), 0x238Au, 0x50100u);
    for (u32 i = 0; i < 16u; ++i) ZLB_EXPECT_EQ(f.bus.read8(0x50000u + i), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
}

ZLB_TEST(bigmac_native_aes128_cbc_register_protocol) {
    Fixture f;
    // NIST SP 800-38A F.2.2. Native RVK function 0x218A uses only the first
    // 16 window bytes, even if a prior AES-256 operation left a nonzero tail.
    const u8 key_window[32] = {
        0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
        0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C,
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    const u8 iv[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                       0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
    const u8 cipher[32] = {
        0x76, 0x49, 0xAB, 0xAC, 0x81, 0x19, 0xB2, 0x46,
        0xCE, 0xE9, 0x8E, 0x9B, 0x12, 0xE9, 0x19, 0x7D,
        0x50, 0x86, 0xCB, 0x9B, 0x50, 0x72, 0x19, 0xEE,
        0x95, 0xDB, 0x11, 0x3A, 0x91, 0x76, 0x78, 0xB2};
    const u8 plain[32] = {
        0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96,
        0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A,
        0xAE, 0x2D, 0x8A, 0x57, 0x1E, 0x03, 0xAC, 0x9C,
        0x9E, 0xB7, 0x6F, 0xAC, 0x45, 0xAF, 0x8E, 0x51};
    f.bus.load(0x809060u, cipher, sizeof(cipher), "native_aes128_source");
    f.bus.load(0x806000u, iv, sizeof(iv), "native_aes128_iv");
    stage_bigmac_key(f, key_window, sizeof(key_window));
    issue_native_bigmac(f, 0x809060u, 0x809060u, sizeof(cipher), 0x218Au, 0x806000u);
    u8 decoded[32];
    f.bus.read_bytes(0x809060u, decoded, sizeof(decoded));
    ZLB_EXPECT_EQ(std::memcmp(decoded, plain, sizeof(plain)), 0);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.cmep.bigmac_device().last_key_bits(), 128u);
    ZLB_EXPECT_EQ(std::memcmp(f.cmep.bigmac_device().last_key().data(), key_window, 16u), 0);
    ZLB_EXPECT_EQ(std::memcmp(f.cmep.bigmac_device().last_iv().data(), iv, sizeof(iv)), 0);
    // Malformed input must error instead of reaching the old 0x20xx fallback.
    issue_native_bigmac(f, 0x809060u, 0x809060u, 31u, 0x218Au, 0x806000u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
}

ZLB_TEST(bigmac_native_aes256_cbc_rejects_bad_inputs) {
    Fixture f;
    const u8 cipher[32] = {};
    const u8 iv[16] = {};
    f.bus.load(0x50000u, cipher, sizeof(cipher), "native_aes_invalid_source");
    f.bus.load(0x50100u, iv, sizeof(iv), "native_aes_invalid_iv");
    issue_native_bigmac(f, 0x50000u, 0x50000u, 31u, 0x238Au, 0x50100u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    issue_native_bigmac(f, 0x50000u, 0x50000u, 32u, 0x238Au, 0x100u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    issue_native_bigmac(f, 0x10000000u, 0x50000u, 32u, 0x238Au, 0x50100u);
    ZLB_EXPECT_NE(f.bus.read32(0xE005003Cu), 0u);
    const u8 prefix[16] = {0xA5u};
    f.bus.add_ram("native_aes_invalid_destination", 16u, 0x10000000u, "partial AES range");
    f.bus.load(0x10000000u, prefix, sizeof(prefix), "native_aes_invalid_destination");
    issue_native_bigmac(f, 0x50000u, 0x10000000u, 32u, 0x238Au, 0x50100u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    u8 unchanged[16];
    f.bus.read_bytes(0x10000000u, unchanged, sizeof(unchanged));
    ZLB_EXPECT_EQ(std::memcmp(unchanged, prefix, sizeof(prefix)), 0);
    // An error must not poison the next valid request.
    issue_native_bigmac(f, 0x50000u, 0x50000u, 32u, 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
}

ZLB_TEST(bigmac_native_aes128_ctr_register_protocol) {
    Fixture f;
    // NIST SP 800-38A F.5.1/F.5.2, with the native wrapper's full 16-byte
    // IV reversal. The poisoned window tail must not become AES-256 material.
    const u8 key_window[32] = {
        0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
        0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C,
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    const u8 hardware_iv[16] = {
        0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8,
        0xF7, 0xF6, 0xF5, 0xF4, 0xF3, 0xF2, 0xF1, 0xF0};
    const u8 after_four[16] = {
        0x03, 0xFF, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8,
        0xF7, 0xF6, 0xF5, 0xF4, 0xF3, 0xF2, 0xF1, 0xF0};
    const u8 cipher[64] = {
        0x87, 0x4D, 0x61, 0x91, 0xB6, 0x20, 0xE3, 0x26, 0x1B, 0xEF, 0x68, 0x64, 0x99, 0x0D, 0xB6, 0xCE,
        0x98, 0x06, 0xF6, 0x6B, 0x79, 0x70, 0xFD, 0xFF, 0x86, 0x17, 0x18, 0x7B, 0xB9, 0xFF, 0xFD, 0xFF,
        0x5A, 0xE4, 0xDF, 0x3E, 0xDB, 0xD5, 0xD3, 0x5E, 0x5B, 0x4F, 0x09, 0x02, 0x0D, 0xB0, 0x3E, 0xAB,
        0x1E, 0x03, 0x1D, 0xDA, 0x2F, 0xBE, 0x03, 0xD1, 0x79, 0x21, 0x70, 0xA0, 0xF3, 0x00, 0x9C, 0xEE};
    const u8 plain[64] = {
        0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A,
        0xAE, 0x2D, 0x8A, 0x57, 0x1E, 0x03, 0xAC, 0x9C, 0x9E, 0xB7, 0x6F, 0xAC, 0x45, 0xAF, 0x8E, 0x51,
        0x30, 0xC8, 0x1C, 0x46, 0xA3, 0x5C, 0xE4, 0x11, 0xE5, 0xFB, 0xC1, 0x19, 0x1A, 0x0A, 0x52, 0xEF,
        0xF6, 0x9F, 0x24, 0x45, 0xDF, 0x4F, 0x9B, 0x17, 0xAD, 0x2B, 0x41, 0x7B, 0xE6, 0x6C, 0x37, 0x10};
    stage_bigmac_key(f, key_window, sizeof(key_window));
    f.bus.load(0x50000u, cipher, sizeof(cipher), "native_ctr_source");
    f.bus.load(0x50100u, hardware_iv, sizeof(hardware_iv), "native_ctr_iv");
    f.bus.memset_bytes(0x502FFu, 0xA5u, 66u);
    issue_native_bigmac(f, 0x50000u, 0x50300u, sizeof(cipher), 0x21A1u, 0x50100u);
    u8 decoded[64], updated[16];
    f.bus.read_bytes(0x50300u, decoded, sizeof(decoded));
    f.bus.read_bytes(0x50100u, updated, sizeof(updated));
    ZLB_EXPECT_EQ(std::memcmp(decoded, plain, sizeof(plain)), 0);
    ZLB_EXPECT_EQ(std::memcmp(updated, after_four, sizeof(updated)), 0);
    ZLB_EXPECT_EQ(std::memcmp(f.cmep.bigmac_device().last_iv().data(), hardware_iv, 16u), 0);
    ZLB_EXPECT_EQ(f.cmep.bigmac_device().last_key_bits(), 128u);
    ZLB_EXPECT_EQ(f.bus.read8(0x502FFu), 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read8(0x50340u), 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005001Cu), 0u);
    for (size_t i = 0; i < sizeof(cipher); ++i) ZLB_EXPECT_EQ(f.bus.read8(0x50000u + i), cipher[i]);
    // In-place encryption must be symmetric, and consume the same counter.
    f.bus.write_bytes(0x50100u, hardware_iv, sizeof(hardware_iv));
    issue_native_bigmac(f, 0x50300u, 0x50300u, sizeof(plain), 0x21A1u, 0x50100u);
    f.bus.read_bytes(0x50300u, decoded, sizeof(decoded));
    ZLB_EXPECT_EQ(std::memcmp(decoded, cipher, sizeof(cipher)), 0);
    // Two calls continue through the hardware-updated IV, as native streams do.
    f.bus.write_bytes(0x50100u, hardware_iv, sizeof(hardware_iv));
    issue_native_bigmac(f, 0x50000u, 0x50300u, 16u, 0x21A1u, 0x50100u);
    issue_native_bigmac(f, 0x50010u, 0x50310u, 48u, 0x21A1u, 0x50100u);
    f.bus.read_bytes(0x50300u, decoded, sizeof(decoded));
    f.bus.read_bytes(0x50100u, updated, sizeof(updated));
    ZLB_EXPECT_EQ(std::memcmp(decoded, plain, sizeof(plain)), 0);
    ZLB_EXPECT_EQ(std::memcmp(updated, after_four, sizeof(updated)), 0);
}

ZLB_TEST(bigmac_native_aes128_ctr_zero_key_counter_wrap) {
    Fixture f;
    // Independent OpenSSL AES-128-CTR answer, zero key/plaintext and initial
    // counter 2^128-1. The second block uses counter zero, then writes back 1.
    const u8 expected[32] = {
        0x3F, 0x5B, 0x8C, 0xC9, 0xEA, 0x85, 0x5A, 0x0A, 0xFA, 0x73, 0x47, 0xD2, 0x3E, 0x8D, 0x66, 0x4E,
        0x66, 0xE9, 0x4B, 0xD4, 0xEF, 0x8A, 0x2C, 0x3B, 0x88, 0x4C, 0xFA, 0x59, 0xCA, 0x34, 0x2B, 0x2E};
    f.bus.memset_bytes(0x50000u, 0u, 32u);
    f.bus.memset_bytes(0x50100u, 0xFFu, 16u);
    issue_native_bigmac(f, 0x50000u, 0x50000u, 32u, 0x21A1u, 0x50100u);
    u8 result[32];
    f.bus.read_bytes(0x50000u, result, sizeof(result));
    ZLB_EXPECT_EQ(std::memcmp(result, expected, sizeof(expected)), 0);
    ZLB_EXPECT_EQ(f.bus.read8(0x50100u), 1u);
    for (u32 i = 1; i < 16; ++i) ZLB_EXPECT_EQ(f.bus.read8(0x50100u + i), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    // A zero-length aligned request consumes no counter blocks.
    issue_native_bigmac(f, 0x100u, 0x100u, 0u, 0x21A1u, 0x50100u);
    ZLB_EXPECT_EQ(f.bus.read8(0x50100u), 1u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
}

ZLB_TEST(bigmac_native_aes128_ctr_rejects_bad_ranges_without_writeback) {
    Fixture f;
    f.bus.memset_bytes(0x50000u, 0u, 32u);
    f.bus.memset_bytes(0x50100u, 0xFFu, 16u);
    f.bus.memset_bytes(0x50200u, 0xA5u, 32u);
    f.bus.add_ram("native_ctr_partial_destination", 16u, 0x10000000u, "partial CTR output");
    f.bus.memset_bytes(0x10000000u, 0xA5u, 16u);
    const u32 requests[][4] = {
        {0x50000u, 0x50200u, 31u, 0x50100u},
        {0x100u, 0x50200u, 32u, 0x50100u},
        {0xFFFFFFF0u, 0x50200u, 32u, 0x50100u},
        {0x50000u, 0x10000000u, 32u, 0x50100u},
        {0x50000u, 0xFFFFFFF0u, 32u, 0x50100u},
        {0x50000u, 0x50200u, 32u, 0x100u},
        {0x50000u, 0x50200u, 32u, 0xFFFFFFF8u}};
    for (const auto& request : requests) {
        issue_native_bigmac(f, request[0], request[1], request[2], 0x21A1u, request[3]);
        ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
        ZLB_EXPECT_NE(f.bus.read32(0xE005003Cu), 0u);
        for (u32 i = 0; i < 16u; ++i) {
            ZLB_EXPECT_EQ(f.bus.read8(0x50100u + i), 0xFFu);
            ZLB_EXPECT_EQ(f.bus.read8(0x10000000u + i), 0xA5u);
        }
        for (u32 i = 0; i < 32u; ++i) ZLB_EXPECT_EQ(f.bus.read8(0x50200u + i), 0xA5u);
    }
    issue_native_bigmac(f, 0x50000u, 0x50200u, 32u, 0x21A1u, 0x50100u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
    ZLB_EXPECT_EQ(f.bus.read8(0x50100u), 1u);
}

ZLB_TEST(bigmac_native_sha256_register_protocol) {
    Fixture f;
    // FIPS 180-4 SHA-256 examples, passed through the native 0x806110 layout.
    // Fixed expected bytes verify the device's digest and byte order without
    // using the SHA helper under test to generate the expected answer.
    const u8 abc[3] = {'a', 'b', 'c'};
    const u8 abc_digest[32] = {
        0xBA, 0x78, 0x16, 0xBF, 0x8F, 0x01, 0xCF, 0xEA,
        0x41, 0x41, 0x40, 0xDE, 0x5D, 0xAE, 0x22, 0x23,
        0xB0, 0x03, 0x61, 0xA3, 0x96, 0x17, 0x7A, 0x9C,
        0xB4, 0x10, 0xFF, 0x61, 0xF2, 0x00, 0x15, 0xAD};
    const char two_block_message[] = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    const u8 two_block_digest[32] = {
        0x24, 0x8D, 0x6A, 0x61, 0xD2, 0x06, 0x38, 0xB8,
        0xE5, 0xC0, 0x26, 0x93, 0x0C, 0x3E, 0x60, 0x39,
        0xA3, 0x3C, 0xE4, 0x59, 0x64, 0xFF, 0x21, 0x67,
        0xF6, 0xEC, 0xED, 0xD4, 0x19, 0xDB, 0x06, 0xC1};
    f.bus.load(0x808FF0u, abc, sizeof(abc), "native_sha_source");
    f.bus.memset_bytes(0x808C7Bu, 0xA5u, 34u);
    // The native caller can leave a nonzero key selector from prior crypto.
    // It must still write the entire 32-byte digest, with no output truncation.
    f.bus.write32(0xE0050010u, 0x216u);
    issue_native_bigmac(f, 0x808FF0u, 0x808C7Cu, sizeof(abc), 0x2093u);
    u8 digest[32];
    f.bus.read_bytes(0x808C7Cu, digest, sizeof(digest));
    ZLB_EXPECT_EQ(std::memcmp(digest, abc_digest, sizeof(digest)), 0);
    ZLB_EXPECT_EQ(f.bus.read8(0x808C7Bu), 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read8(0x808C9Cu), 0xA5u);
    for (size_t i = 0; i < sizeof(abc); ++i) ZLB_EXPECT_EQ(f.bus.read8(0x808FF0u + i), abc[i]);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005001Cu), 0u);

    // 56 input bytes require two final compression blocks after SHA padding.
    f.bus.load(0x808FF0u, reinterpret_cast<const u8*>(two_block_message),
               sizeof(two_block_message) - 1u, "native_sha_padding_source");
    issue_native_bigmac(f, 0x808FF0u, 0x808C7Cu, sizeof(two_block_message) - 1u, 0x2093u);
    f.bus.read_bytes(0x808C7Cu, digest, sizeof(digest));
    ZLB_EXPECT_EQ(std::memcmp(digest, two_block_digest, sizeof(digest)), 0);
    ZLB_EXPECT_EQ(f.cmep.bigmac_device().hash_operations(), 2u);
}

ZLB_TEST(bigmac_native_sha256_full_blocks_and_empty_message) {
    Fixture f;
    // Independent hashlib/OpenSSL answer for a 768-byte, 12-block source, the
    // size used by the native RVK signature check. Each byte is (i*7+3)&255.
    std::array<u8, 768> source{};
    for (size_t i = 0; i < source.size(); ++i) source[i] = static_cast<u8>(i * 7u + 3u);
    const u8 expected[32] = {
        0xAC, 0x31, 0x5A, 0x0B, 0xE0, 0x63, 0xFF, 0x33,
        0x1C, 0x93, 0x91, 0x3F, 0x8B, 0x57, 0x30, 0x05,
        0xD4, 0x8F, 0x9E, 0x62, 0xC0, 0x92, 0x26, 0xC0,
        0x52, 0x7C, 0x96, 0x34, 0x9C, 0x2C, 0xA3, 0x4F};
    const u8 empty_digest[32] = {
        0xE3, 0xB0, 0xC4, 0x42, 0x98, 0xFC, 0x1C, 0x14,
        0x9A, 0xFB, 0xF4, 0xC8, 0x99, 0x6F, 0xB9, 0x24,
        0x27, 0xAE, 0x41, 0xE4, 0x64, 0x9B, 0x93, 0x4C,
        0xA4, 0x95, 0x99, 0x1B, 0x78, 0x52, 0xB8, 0x55};
    f.bus.load(0x808FF0u, source.data(), source.size(), "native_sha_blocks");
    issue_native_bigmac(f, 0x808FF0u, 0x808C7Cu, source.size(), 0x2093u);
    u8 digest[32];
    f.bus.read_bytes(0x808C7Cu, digest, sizeof(digest));
    ZLB_EXPECT_EQ(std::memcmp(digest, expected, sizeof(digest)), 0);
    // Empty hashing does not dereference the source, even when unmapped.
    issue_native_bigmac(f, 0x100u, 0x808C7Cu, 0u, 0x2093u);
    f.bus.read_bytes(0x808C7Cu, digest, sizeof(digest));
    ZLB_EXPECT_EQ(std::memcmp(digest, empty_digest, sizeof(digest)), 0);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
}

ZLB_TEST(bigmac_native_sha256_rejects_invalid_ranges) {
    Fixture f;
    const u8 source[64] = {};
    f.bus.load(0x50000u, source, sizeof(source), "native_sha_invalid_source");
    f.bus.memset_bytes(0x808C7Cu, 0xA5u, 32u);
    issue_native_bigmac(f, 0x100u, 0x808C7Cu, 16u, 0x2093u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    for (u32 i = 0; i < 32u; ++i) ZLB_EXPECT_EQ(f.bus.read8(0x808C7Cu + i), 0xA5u);
    issue_native_bigmac(f, 0xFFFFFFF0u, 0x808C7Cu, 32u, 0x2093u);
    ZLB_EXPECT_NE(f.bus.read32(0xE005003Cu), 0u);
    f.bus.add_ram("native_sha_partial_destination", 16u, 0x10000000u, "partial digest range");
    f.bus.memset_bytes(0x10000000u, 0xA5u, 16u);
    issue_native_bigmac(f, 0x50000u, 0x10000000u, sizeof(source), 0x2093u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    for (u32 i = 0; i < 16u; ++i) ZLB_EXPECT_EQ(f.bus.read8(0x10000000u + i), 0xA5u);
    issue_native_bigmac(f, 0x50000u, 0xFFFFFFF0u, sizeof(source), 0x2093u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    // A subsequent valid operation clears the hardware error indication.
    issue_native_bigmac(f, 0x50000u, 0x808C7Cu, sizeof(source), 0x2093u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
}

ZLB_TEST(bigmac_native_stream_hmac_matches_kprx_chunk_sizes) {
    Fixture f;
    std::array<u8, 64> key{};
    for (size_t i = 0; i < key.size(); ++i) key[i] = static_cast<u8>(i);
    std::vector<u8> source(28596u);
    for (size_t i = 0; i < source.size(); ++i) source[i] = static_cast<u8>(i * 7u + 3u);
    // Python hashlib/OpenSSL known answer; exactly the live kprx 28544+52 split.
    const std::array<u8, 32> expected = {
        0x25, 0xBF, 0x34, 0x9E, 0x49, 0xF1, 0x74, 0xD8, 0x42, 0x07, 0x25, 0xEB, 0x94, 0x29, 0x78, 0xDE,
        0x25, 0xC4, 0xD4, 0x69, 0x8F, 0x6B, 0xDA, 0xD9, 0x95, 0x53, 0xDF, 0x94, 0xD8, 0xA3, 0x64, 0xD8};
    f.bus.load(0x40001D00u, source.data(), source.size(), "native_stream_hmac_source");
    f.bus.memset_bytes(0x808B40u, 0xA5u, 0xD4u);
    stage_bigmac_key(f, key.data(), key.size());
    issue_native_bigmac(f, 0x40001D00u, 0x808B6Cu, 28544u, 0x24B3u, 0x808B44u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    for (u32 i = 0; i < 32u; ++i) ZLB_EXPECT_EQ(f.bus.read8(0x808B6Cu + i), 0xA5u);
    // The first chunk is already compressed; later mutation must not rehash it.
    f.bus.memset_bytes(0x40001D00u, 0xFEu, 16u);
    issue_native_bigmac(f, 0x40001D00u + 28544u, 0x808B6Cu, 52u, 0x28B3u, 0x808B44u);
    std::array<u8, 32> digest{};
    f.bus.read_bytes(0x808B6Cu, digest.data(), digest.size());
    ZLB_EXPECT_EQ(std::memcmp(digest.data(), expected.data(), digest.size()), 0);
    ZLB_EXPECT_EQ(f.cmep.bigmac_device().last_key_bits(), 512u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
    // Finalization retires the context instead of accepting a stale second tail.
    issue_native_bigmac(f, 0x40001D00u + 28544u, 0x808B6Cu, 52u, 0x28B3u, 0x808B44u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    f.bus.write_bytes(0x40001D00u, source.data(), 16u);
    issue_native_bigmac(f, 0x40001D00u, 0x808B6Cu, source.size(), 0x20B3u, 0x808B44u);
    f.bus.read_bytes(0x808B6Cu, digest.data(), digest.size());
    ZLB_EXPECT_EQ(std::memcmp(digest.data(), expected.data(), digest.size()), 0);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
}

ZLB_TEST(bigmac_native_stream_hmac_continuation_and_context_isolation) {
    Fixture f;
    std::array<u8, 64> key_a{}, key_b{};
    for (size_t i = 0; i < key_a.size(); ++i) key_a[i] = static_cast<u8>(i);
    key_b.fill(0xA5u);
    std::array<u8, 193> source_a{};
    std::array<u8, 129> source_b{};
    for (size_t i = 0; i < source_a.size(); ++i) source_a[i] = static_cast<u8>(i * 13u + 5u);
    for (size_t i = 0; i < source_b.size(); ++i) source_b[i] = static_cast<u8>(255u - i);
    const std::array<u8, 32> expected_a = {
        0x5A, 0x05, 0x4B, 0x0A, 0xE2, 0xDE, 0xD8, 0x68, 0x1A, 0x0B, 0x58, 0x8D, 0xB3, 0x8C, 0x0A, 0x5D,
        0xAD, 0x9F, 0x80, 0x0E, 0xC8, 0x82, 0x5D, 0xF7, 0xCA, 0x11, 0xE1, 0xA4, 0x28, 0xDA, 0x08, 0x44};
    const std::array<u8, 32> expected_b = {
        0xA4, 0xB2, 0x26, 0xE0, 0x02, 0x7F, 0xC0, 0xE2, 0x82, 0x29, 0x7B, 0x6A, 0xA0, 0x1D, 0xA3, 0x08,
        0x07, 0x87, 0x97, 0xF5, 0x6E, 0x20, 0x0E, 0xC1, 0xDE, 0xC7, 0x5E, 0xAF, 0xE5, 0x0F, 0x40, 0x68};
    f.bus.load(0x50000u, source_a.data(), source_a.size(), "native_stream_hmac_a");
    f.bus.load(0x51000u, source_b.data(), source_b.size(), "native_stream_hmac_b");
    stage_bigmac_key(f, key_a.data(), key_a.size());
    issue_native_bigmac(f, 0x50000u, 0x52100u, 64u, 0x24B3u, 0x52000u);
    stage_bigmac_key(f, key_b.data(), key_b.size());
    issue_native_bigmac(f, 0x51000u, 0x52300u, 128u, 0x24B3u, 0x52200u);
    // A different staged key cannot accidentally continue the first context.
    issue_native_bigmac(f, 0x50040u, 0x52100u, 64u, 0x2CB3u, 0x52000u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    stage_bigmac_key(f, key_a.data(), key_a.size());
    issue_native_bigmac(f, 0x50040u, 0x52100u, 64u, 0x2CB3u, 0x52000u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    stage_bigmac_key(f, key_b.data(), key_b.size());
    issue_native_bigmac(f, 0x51080u, 0x52300u, 1u, 0x28B3u, 0x52200u);
    std::array<u8, 32> digest{};
    f.bus.read_bytes(0x52300u, digest.data(), digest.size());
    ZLB_EXPECT_EQ(std::memcmp(digest.data(), expected_b.data(), digest.size()), 0);
    stage_bigmac_key(f, key_a.data(), key_a.size());
    issue_native_bigmac(f, 0x50080u, 0x52100u, 65u, 0x28B3u, 0x52000u);
    f.bus.read_bytes(0x52100u, digest.data(), digest.size());
    ZLB_EXPECT_EQ(std::memcmp(digest.data(), expected_a.data(), digest.size()), 0);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
}

ZLB_TEST(bigmac_native_stream_hmac_errors_reset_and_reinitialization) {
    Fixture f;
    std::array<u8, 64> key{};
    for (size_t i = 0; i < key.size(); ++i) key[i] = static_cast<u8>(i);
    std::array<u8, 65> source{};
    for (size_t i = 0; i < source.size(); ++i) source[i] = static_cast<u8>(i * 13u + 5u);
    const std::array<u8, 32> expected = {
        0x22, 0xEE, 0xB9, 0x2A, 0xD1, 0xAB, 0x42, 0xDF, 0x31, 0x1C, 0x65, 0xE7, 0xFF, 0x03, 0x64, 0xDA,
        0x4E, 0x60, 0xC7, 0x53, 0x26, 0x2A, 0x33, 0x44, 0xAA, 0x01, 0xEB, 0x78, 0x3C, 0x49, 0x22, 0xA4};
    f.bus.load(0x50000u, source.data(), source.size(), "native_stream_hmac_errors");
    f.bus.memset_bytes(0x50500u, 0xA5u, 32u);
    f.bus.add_ram("native_stream_hmac_partial_destination", 16u, 0x10000000u, "partial HMAC digest");
    f.bus.memset_bytes(0x10000000u, 0xA5u, 16u);
    stage_bigmac_key(f, key.data(), key.size());
    issue_native_bigmac(f, 0x50000u, 0x50500u, 64u, 0x24B3u, 0x50400u);
    std::array<u8, 40> state{};
    f.bus.read_bytes(0x50400u, state.data(), state.size());
    const u32 requests[][5] = {
        {0x50040u, 0x10000000u, 1u, 0x28B3u, 0x50400u},
        {0x50000u, 0x50500u, 63u, 0x2CB3u, 0x50400u},
        {0x100u, 0x50500u, 1u, 0x28B3u, 0x50400u},
        {0x50040u, 0x50500u, 1u, 0x28B3u, 0x100u},
        {0x50000u, 0x50500u, 64u, 0x24B3u, 0xFFFFFFF0u}};
    for (const auto& request : requests) {
        issue_native_bigmac(f, request[0], request[1], request[2], request[3], request[4]);
        ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
        std::array<u8, 40> unchanged{};
        f.bus.read_bytes(0x50400u, unchanged.data(), unchanged.size());
        ZLB_EXPECT_EQ(std::memcmp(state.data(), unchanged.data(), state.size()), 0);
        for (u32 i = 0; i < 32u; ++i) ZLB_EXPECT_EQ(f.bus.read8(0x50500u + i), 0xA5u);
        for (u32 i = 0; i < 16u; ++i) ZLB_EXPECT_EQ(f.bus.read8(0x10000000u + i), 0xA5u);
    }
    // A relocated opaque state image cannot silently inherit another pointer's
    // host context; this limit is explicit until hardware serialization is known.
    f.bus.write_bytes(0x50600u, state.data(), state.size());
    issue_native_bigmac(f, 0x50040u, 0x50500u, 1u, 0x28B3u, 0x50600u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    f.bus.write8(0x50400u, state[0] ^ 0xFFu);
    issue_native_bigmac(f, 0x50040u, 0x50500u, 1u, 0x28B3u, 0x50400u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    f.bus.write_bytes(0x50400u, state.data(), state.size());
    issue_native_bigmac(f, 0x50040u, 0x50500u, 1u, 0x28B3u, 0x50400u);
    std::array<u8, 32> digest{};
    f.bus.read_bytes(0x50500u, digest.data(), digest.size());
    ZLB_EXPECT_EQ(std::memcmp(digest.data(), expected.data(), digest.size()), 0);
    // A new first chunk reinitializes a reused pointer. Reset retires it.
    issue_native_bigmac(f, 0x50000u, 0x50500u, 64u, 0x24B3u, 0x50400u);
    f.cmep.reset();
    stage_bigmac_key(f, key.data(), key.size());
    issue_native_bigmac(f, 0x50040u, 0x50500u, 1u, 0x28B3u, 0x50400u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    issue_native_bigmac(f, 0x50000u, 0x50500u, 64u, 0x24B3u, 0x50400u);
    issue_native_bigmac(f, 0x50040u, 0x50500u, 1u, 0x28B3u, 0x50400u);
    f.bus.read_bytes(0x50500u, digest.data(), digest.size());
    ZLB_EXPECT_EQ(std::memcmp(digest.data(), expected.data(), digest.size()), 0);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
}

ZLB_TEST(bigmac_native_context_transfer_register_protocol) {
    Fixture f;
    // Native 0x806C78 retains the context's 0x2080 bits for a plain transfer.
    // Its RVK caller reads the 32-byte section at image +0x400.
    const u32 section[8] = {5u, 1u, 0u, 0x104u, 0x15u, 0u, 0u, 0u};
    f.bus.load(0x40009300u, section, sizeof(section), "native_rvk_plain_section");
    f.bus.memset_bytes(0x8093EFu, 0xA5u, sizeof(section) + 2u);
    issue_native_bigmac(f, 0x40009300u, 0x8093F0u, sizeof(section), 0x2080u);
    u32 copied[8];
    f.bus.read_bytes(0x8093F0u, copied, sizeof(copied));
    ZLB_EXPECT_EQ(std::memcmp(copied, section, sizeof(section)), 0);
    ZLB_EXPECT_EQ(f.bus.read8(0x8093EFu), 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read8(0x809410u), 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
    // Reject the entire request without changing a mapped destination prefix.
    f.bus.add_ram("native_context_partial_destination", 16u, 0x10000000u, "partial transfer");
    f.bus.memset_bytes(0x10000000u, 0xA5u, 16u);
    issue_native_bigmac(f, 0x40009300u, 0x10000000u, sizeof(section), 0x2080u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    for (u32 i = 0; i < 16u; ++i) ZLB_EXPECT_EQ(f.bus.read8(0x10000000u + i), 0xA5u);
    issue_native_bigmac(f, 0xFFFFFFF0u, 0x8093F0u, sizeof(section), 0x2080u);
    ZLB_EXPECT_NE(f.bus.read32(0xE005003Cu), 0u);
}

ZLB_TEST(bigmac_native_hmac_sha256_register_protocol) {
    Fixture f;
    // RFC 4231 test case 1, staged as the native padded 64-byte key block.
    std::array<u8, 64> key{};
    for (size_t i = 0; i < 20u; ++i) key[i] = 0x0Bu;
    const u8 message[8] = {'H', 'i', ' ', 'T', 'h', 'e', 'r', 'e'};
    const u8 expected[32] = {
        0xB0, 0x34, 0x4C, 0x61, 0xD8, 0xDB, 0x38, 0x53,
        0x5C, 0xA8, 0xAF, 0xCE, 0xAF, 0x0B, 0xF1, 0x2B,
        0x88, 0x1D, 0xC2, 0x00, 0xC9, 0x83, 0x3D, 0xA7,
        0x26, 0xE9, 0x37, 0x6C, 0x2E, 0x32, 0xCF, 0xF7};
    f.bus.load(0x8093F0u, message, sizeof(message), "native_hmac_source");
    f.bus.memset_bytes(0x808CB4u, 0xA5u, 40u);
    f.bus.memset_bytes(0x808CDBu, 0xA5u, 34u);
    stage_bigmac_key(f, key.data(), key.size());
    f.bus.write32(0xE0050010u, 0x216u);
    issue_native_bigmac(f, 0x8093F0u, 0x808CDCu, sizeof(message), 0x20B3u, 0x808CB4u);
    u8 digest[32];
    f.bus.read_bytes(0x808CDCu, digest, sizeof(digest));
    ZLB_EXPECT_EQ(std::memcmp(digest, expected, sizeof(digest)), 0);
    ZLB_EXPECT_EQ(f.bus.read8(0x808CDBu), 0xA5u);
    ZLB_EXPECT_EQ(f.bus.read8(0x808CFCu), 0xA5u);
    for (size_t i = 0; i < sizeof(message); ++i) {
        ZLB_EXPECT_EQ(f.bus.read8(0x8093F0u + i), message[i]);
    }
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
    ZLB_EXPECT_EQ(f.cmep.bigmac_device().last_op(), BigmacOp::Hmac);
    ZLB_EXPECT_EQ(f.cmep.bigmac_device().hash_operations(), 1u);

    // A nonzero upper half with zero lower half proves words +220..+23C are
    // staged and consumed. Fixed empty-message answer from hashlib/OpenSSL.
    key.fill(0);
    for (size_t i = 32; i < key.size(); ++i) key[i] = 0xA5u;
    const u8 tail_key_digest[32] = {
        0xB9, 0x40, 0x0B, 0x93, 0xAB, 0xF0, 0x22, 0xFC,
        0x74, 0x12, 0x8B, 0x60, 0x33, 0xB9, 0x52, 0x27,
        0x20, 0x5F, 0xE5, 0x68, 0xF1, 0x77, 0xBB, 0x41,
        0x38, 0xFB, 0x05, 0x19, 0x37, 0x01, 0xDE, 0xDC};
    stage_bigmac_key(f, key.data(), key.size());
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050220u), 0xA5A5A5A5u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005023Cu), 0xA5A5A5A5u);
    issue_native_bigmac(f, 0x100u, 0x808CDCu, 0u, 0x20B3u, 0x808CB4u);
    f.bus.read_bytes(0x808CDCu, digest, sizeof(digest));
    ZLB_EXPECT_EQ(std::memcmp(digest, tail_key_digest, sizeof(digest)), 0);
}

ZLB_TEST(bigmac_native_hmac_sha256_full_blocks_and_invalid_ranges) {
    Fixture f;
    std::array<u8, 64> key{};
    for (size_t i = 0; i < key.size(); ++i) key[i] = static_cast<u8>(i);
    std::array<u8, 768> source{};
    for (size_t i = 0; i < source.size(); ++i) source[i] = static_cast<u8>(i * 7u + 3u);
    const u8 expected[32] = {
        0x98, 0xDC, 0xB9, 0xA1, 0x45, 0x64, 0x84, 0xD8,
        0xB3, 0xCC, 0xA2, 0xDA, 0xA2, 0xF2, 0xDE, 0x9B,
        0x6E, 0xD8, 0xAA, 0x19, 0xA7, 0xB0, 0xD0, 0x4E,
        0x78, 0x1C, 0x71, 0x58, 0xC2, 0x4A, 0xE2, 0xD2};
    f.bus.load(0x808FF0u, source.data(), source.size(), "native_hmac_blocks");
    stage_bigmac_key(f, key.data(), key.size());
    issue_native_bigmac(f, 0x808FF0u, 0x808CDCu, source.size(), 0x20B3u, 0x808CB4u);
    u8 digest[32];
    f.bus.read_bytes(0x808CDCu, digest, sizeof(digest));
    ZLB_EXPECT_EQ(std::memcmp(digest, expected, sizeof(digest)), 0);
    issue_native_bigmac(f, 0x100u, 0x808CDCu, 16u, 0x20B3u, 0x808CB4u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    f.bus.read_bytes(0x808CDCu, digest, sizeof(digest));
    ZLB_EXPECT_EQ(std::memcmp(digest, expected, sizeof(digest)), 0);
    f.bus.add_ram("native_hmac_partial_digest", 16u, 0x10000000u, "partial HMAC output");
    f.bus.memset_bytes(0x10000000u, 0xA5u, 16u);
    issue_native_bigmac(f, 0x808FF0u, 0x10000000u, source.size(), 0x20B3u, 0x808CB4u);
    ZLB_EXPECT_NE(f.bus.read32(0xE005003Cu), 0u);
    for (u32 i = 0; i < 16u; ++i) ZLB_EXPECT_EQ(f.bus.read8(0x10000000u + i), 0xA5u);
    issue_native_bigmac(f, 0xFFFFFFF0u, 0x808CDCu, 32u, 0x20B3u, 0x808CB4u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024u) & 0x78000u, 0u);
    issue_native_bigmac(f, 0x808FF0u, 0x808CDCu, source.size(), 0x20B3u, 0x808CB4u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0050024u), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE005003Cu), 0u);
}

ZLB_TEST(bigmac_unknown_function_raises_exception) {
    Fixture f;
    f.bus.write32(0xE0050004, 0);
    f.bus.write32(0xE0050000, 0);
    f.bus.write32(0xE0050008, 0);
    f.bus.write32(0xE0050010, 0);
    f.bus.write32(0xE005000C, 0x7F7F);
    f.bus.write32(0xE005001C, 1);
    // The failure path at 0x5C5FE reads the exception status back.
    ZLB_EXPECT_NE(f.bus.read32(0xE0050024) & 0x78000u, 0u);
    ZLB_EXPECT_NE(f.bus.read32(0xE005003C), 0u);
}

// ---------------------------------------------------------------------------
// Bignum (0xE0040000) - bignum_op (0x5CE04)
// ---------------------------------------------------------------------------

ZLB_TEST(bignum_register_protocol) {
    Fixture f;
    // modulus window 0xE0040400, base window 0xE0040108, exponent stream
    // 0xE0040808, control 0xE0040800 (bignum_op).
    // Use modulus 0x10001 (65537) and exponent 65537 so the check is exact:
    // 2^65537 mod 65537 == 2 for a prime modulus.
    // The control word is 0x91000000 | (modulus words << 18) | (exponent words
    // << 9), exactly what 0x5CE58 builds and what the second loader's wrapper
    // stores at 0x4BAA0.
    const u32 control = 0x90000000u | (1u << 18) | (1u << 9);
    // Write the operands as the loader does: `lw` from the blob, i.e.
    // little-endian words from the end of the big-endian value.
    const u32 mod_word0 = 0x00010001u;
    f.bus.write32(0xE0040400, mod_word0);
    f.bus.write32(0xE0040108, 2);
    f.bus.write32(0xE0040808, 65537);  // the first exponent word (0x5CE50)
    f.bus.write32(0xE0040800, control);

    ZLB_EXPECT_EQ(f.cmep.bignum_operations(), 1u);
    const u32 status = f.bus.read32(0xE0040804);
    ZLB_EXPECT_EQ(status & 0x02000000u, 0u);              // no error (bit 25)
    ZLB_EXPECT_EQ(status & 0x80000000u, 0u);              // not busy (bit 31)
    ZLB_EXPECT_EQ(status & 0x08000000u, 0u);              // no word wanted (bit 27)
    ZLB_EXPECT_NE(status & 0x04000000u, 0u);              // idle/ready (bit 26)
    ZLB_EXPECT_EQ((status >> 16) & 0xFFu, 1u);            // result = 1 word
    // 2^65537 mod 65537 == 2.  The window holds the little-endian encoding of the
    // result and the loader's byte-swap loop (0x5CECA..0x5CF04) rebuilds the
    // big-endian block, so the value 2 shows up as 0x00000002 in window word 0
    // and the top word is zero.
    ZLB_EXPECT_EQ(f.bus.read32(0xE0040508), 0x00000002u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0040604), 0u);
    // The 0x1000 mirror (0xE0041804) decodes to the same register.
    ZLB_EXPECT_EQ(f.bus.read32(0xE0041804), status);
    ZLB_EXPECT_EQ(f.cmep.bignum_operations(), 1u);
    // The engine raises the E0020020 completion semaphore, which engine_wait()
    // (0x4BBB0) spins on and clears by writing it back.
    ZLB_EXPECT_EQ(f.bus.read32(0xE0020020), 1u);
    f.bus.write32(0xE0020020, 1);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0020020), 0u);
}

ZLB_TEST(bignum_verifies_the_development_boot_key) {
    Fixture f;
    // Drive the engine exactly like the first loader: `lw` one little-endian word
    // of each operand blob and `sw` it into the 64-word windows (0x5CE06..0x5CE3A
    // for the base, 0x5CE22..0x5CE3A for the modulus), then stream the exponent
    // (0x5CE50) and start (0x5CE5C).
    auto load_window = [&f](u32 window, const unsigned char* blob) {
        for (u32 word = 0; word < 64; ++word) {
            const u32 value = static_cast<u32>(blob[4 * word]) |
                              (static_cast<u32>(blob[4 * word + 1]) << 8) |
                              (static_cast<u32>(blob[4 * word + 2]) << 16) |
                              (static_cast<u32>(blob[4 * word + 3]) << 24);
            f.bus.write32(window + 4 * word, value);
        }
    };
    load_window(0xE0040108, bootkeys::kSignatureWindow);
    load_window(0xE0040400, bootkeys::kModulusWindow);
    f.bus.write32(0xE0040808, 0x00010001u);  // e = 65537, the ROM's first word
    f.bus.write32(0xE0040800, 0x90000000u | (64u << 18) | (1u << 9));  // start

    ZLB_EXPECT_EQ(f.bus.read32(0xE0040804) & 0x02000000u, 0u);  // no engine error

    // Rebuild the 256-byte big-endian block the loader compares: its byte-swap
    // loop (0x5CECA..0x5CF04) byte-swaps window word i into destination word
    // 63 - i.
    std::vector<u8> block(256, 0);
    for (u32 word = 0; word < 64; ++word) {
        const u32 value = f.bus.read32(0xE0040508 + 4 * word);
        for (u32 byte = 0; byte < 4; ++byte) {
            block[4 * (63 - word) + byte] =
                static_cast<u8>((value >> (8 * (3 - byte))) & 0xFF);
        }
    }
    // A correct 2048-bit RSA verification: 00 01 FF*203 00 || DigestInfo || key.
    ZLB_EXPECT_EQ(block[0], 0x00u);
    ZLB_EXPECT_EQ(block[1], 0x01u);
    for (int i = 2; i < 205; ++i) ZLB_EXPECT_EQ(block[static_cast<size_t>(i)], 0xFFu);
    ZLB_EXPECT_EQ(block[205], 0x00u);

    u8 digest[32] = {};
    sha256(block.data(), block.size(), digest);
    ZLB_EXPECT_EQ(std::memcmp(digest, bootkeys::kExpectedMessageDigest, 32), 0);
}

ZLB_TEST(first_loader_layout_detection_matches_both_dumps) {
    // The prototype image and the retail PCH RAM snapshot are the same program
    // with `.data`/`.bss` shifted by -0x80, so provisioning has to locate the ROM
    // constants instead of hard-coding them.  The marker is easy to get wrong by
    // four bytes (the blob is preceded by a `00 00 C0 E0` filler word that also
    // satisfies an 8-byte prefix match), and a four-byte error is invisible: the
    // only symptom is a failed RSA comparison at boot.
    struct Build {
        const char* path;
        int shift;
        bool prototype;
        bool present;
    };
    Build builds[] = {
        {"dumps/vita_prototype_bootrom.bin", 0, true, false},
        {"dumps/pch-5c-cold_first_loader.bin", -0x80, false, false},
    };
    for (Build& build : builds) {
        Fixture f;
        const std::string path = resolve_workspace_path(build.path);
        auto data = read_file(path);
        if (!data) continue;
        build.present = true;
        ZLB_EXPECT_TRUE(f.bus.load(0x5C000, data->data(), data->size(), "first_loader"));

        const FirstLoaderLayout layout = detect_first_loader_layout(f.bus, *data);
        ZLB_EXPECT_TRUE(layout.known);
        ZLB_EXPECT_EQ(layout.shift, build.shift);
        ZLB_EXPECT_EQ(layout.prototype, build.prototype);
        ZLB_EXPECT_EQ(layout.verified, 16u);  // the full build marker
        // The three constants the RSA path needs, at their documented addresses
        // for the prototype build (docs/BOOT.md 3).
        if (build.prototype) {
            ZLB_EXPECT_EQ(layout.parameter_blob, 0x5E764u);
            ZLB_EXPECT_EQ(layout.tail_key, 0x5E744u);
        }
        ZLB_EXPECT_EQ(layout.digest_info, layout.parameter_blob + 0x10u);
        ZLB_EXPECT_EQ(layout.tail_key, layout.parameter_blob - 0x20u);
        // The digest reference the loader memcmps must be the DER prefix in both.
        ZLB_EXPECT_EQ(f.bus.read8(layout.digest_info), 0x30u);
        ZLB_EXPECT_EQ(f.bus.read8(layout.digest_info + 3), 0x0Cu);
    }
    // Both dumps exist in this workspace; a missing one means the checkout is
    // incomplete rather than that the detection is wrong.
    ZLB_EXPECT_TRUE(builds[0].present && builds[1].present);
}

ZLB_TEST(bignum_streams_the_exponent) {
    Fixture f;
    // Two-word exponent: the stream is a big-endian word sequence, so 0x00000000
    // followed by 0x00000002 is the exponent 2 (the first loader writes word 0
    // before the control store and the rest while status bit 27 asks for them).
    f.bus.write32(0xE0040400 + 4 * 0, 0x00000005u);  // modulus = 5
    f.bus.write32(0xE0040108, 3);                    // base = 3
    f.bus.write32(0xE0040808, 0);                    // exponent word 0 = 0
    f.bus.write32(0xE0040800, 0x90000000u | (1u << 18) | (2u << 9));
    // The engine asks for the missing word through status bits 26/27.
    ZLB_EXPECT_NE(f.bus.read32(0xE0040804) & 0x08000000u, 0u);
    ZLB_EXPECT_NE(f.bus.read32(0xE0040804) & 0x80000000u, 0u);
    f.bus.write32(0xE0040808, 0x00000002u);          // exponent word 1 = 2
    ZLB_EXPECT_EQ(f.bus.read32(0xE0040804) & 0x80000000u, 0u);
    // 3^2 mod 5 == 4 -> the little-endian window word 0 holds the value 4.
    ZLB_EXPECT_EQ(f.bus.read32(0xE0040508), 0x00000004u);
}

ZLB_TEST(bignum_reset_clears_state) {
    Fixture f;
    f.bus.write32(0xE0040400, 7);
    f.bus.write32(0xE0040108, 3);
    f.bus.write32(0xE0040808, 2);
    f.bus.write32(0xE0040800, 0x90000000u | (1u << 18) | (1u << 9));
    ZLB_EXPECT_EQ(f.cmep.bignum_operations(), 1u);
    f.cmep.reset();
    ZLB_EXPECT_EQ(f.cmep.bignum_operations(), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0040804) & 0x02000000u, 0u);
}

// ---------------------------------------------------------------------------
// SC transfer window (0xE0B00000 / 0xE0BF0000)
// ---------------------------------------------------------------------------

ZLB_TEST(cmep_sc_descriptor_decode) {
    Fixture f;
    // sc_xfer (0x5CF58) builds a 28-byte descriptor at 0xE0B0FF00:
    //   +0x00 mode, +0x04 timeout, +0x08 timeout2, +0x0E channel,
    //   +0x18 window base, +0x24 control pointer.
    std::vector<u8> desc(0x2C, 0);
    desc[0x00] = 0x80;
    desc[0x0E] = 0x01;
    const u32 window = cmep_detail::ScXferDevice::kCmdWindow;
    desc[0x18] = static_cast<u8>(window);
    desc[0x19] = static_cast<u8>(window >> 8);
    desc[0x1A] = static_cast<u8>(window >> 16);
    desc[0x1B] = static_cast<u8>(window >> 24);
    f.cmep.sc_xfer_device().post_descriptor(desc, window);
    ZLB_EXPECT_TRUE(f.cmep.sc_xfer_device().pending());

    // Without storage attached the transfer still completes (it records the
    // miss) so the boot chain cannot spin forever.
    f.cmep.attach_storage(nullptr, nullptr);
    f.cmep.service_sc_transfer();
    ZLB_EXPECT_FALSE(f.cmep.sc_xfer_device().pending());
    ZLB_EXPECT_EQ(f.cmep.sc_transfers(), 0u);

    // The descriptor sc_xfer really builds lives in CMeP RAM (0x5EE20).
    ZLB_EXPECT_EQ(cmep::kScDescriptorAddress, 0x5EE20u);

    // The descriptor address helper matches sc_xfer's own computation.
    ZLB_EXPECT_EQ(cmep_detail::ScXferDevice::descriptor_address(0x00),
                  cmep_detail::ScXferDevice::kCmdWindow +
                      cmep_detail::ScXferDevice::kDescriptorOffset);
    ZLB_EXPECT_EQ(cmep_detail::ScXferDevice::descriptor_address(0x01),
                  cmep_detail::ScXferDevice::kCmdWindow + 0x10000 +
                      cmep_detail::ScXferDevice::kDescriptorOffset);
}

// ---------------------------------------------------------------------------
// reset / summary / describe
// ---------------------------------------------------------------------------

ZLB_TEST(cmep_reset_restores_power_on_state) {
    Fixture f;
    for (u32 i = 0; i < 8; ++i) f.bus.write32(0xE0030000 + 4 * i, 0xDEADBEEFu);
    f.bus.write32(0xE0030020, 0x501);
    f.bus.write32(0xE0030024, 0x1C0F020Eu);
    f.cmep.set_cmep_status(1);
    f.bus.write32(0xE005000C, 0x0301);
    f.bus.write32(0xE005001C, 1);
    ZLB_EXPECT_NE(f.cmep.keyring_writes(), 0u);
    ZLB_EXPECT_NE(f.cmep.bigmac_operations(), 0u);

    f.cmep.reset();
    ZLB_EXPECT_EQ(f.cmep.keyring_writes(), 0u);
    // Only the fused boot slot survives a reset; the loader-programmed slots do not.
    const auto& after_reset = f.cmep.captured_keyrings();
    ZLB_EXPECT_EQ(after_reset.size(), static_cast<size_t>(1));
    ZLB_EXPECT_TRUE(after_reset.count(cmep::kBootKeyring) == 1);
    ZLB_EXPECT_TRUE(f.cmep.clear_flags_history().empty());
    ZLB_EXPECT_EQ(f.cmep.cmep_status(), 0u);
    ZLB_EXPECT_FALSE(f.cmep.reported_success());
    ZLB_EXPECT_FALSE(f.cmep.reported_failure());
    ZLB_EXPECT_EQ(f.cmep.bigmac_operations(), 0u);
    ZLB_EXPECT_EQ(f.cmep.bignum_operations(), 0u);
    ZLB_EXPECT_EQ(f.cmep.sc_transfers(), 0u);
    // The strap reports the retail configuration: 0xE0062020 bits 0..2 clear.
    // The second loader validates exactly that (0x40DB6: `and3 $3,$3,0x7` then
    // `beqz $3,<ok>`, error 0x800F0033 otherwise), while the first loader only
    // uses bit 0 to pick the banner character and accepts both values.
    ZLB_EXPECT_EQ(f.bus.read32(0xE0062020) & 7u, 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE003002C), 0x10000003u);
}

ZLB_TEST(cmep_summary_and_describe) {
    Fixture f;
    const std::string summary = f.cmep.summary();
    ZLB_EXPECT_FALSE(summary.empty());
    std::vector<std::string> lines;
    f.cmep.describe(lines);
    ZLB_EXPECT_TRUE(lines.size() >= 8);
    for (Device* device : f.cmep.devices()) {
        if (device == nullptr) continue;
        ZLB_EXPECT_FALSE(device->summary().empty());
    }
}

ZLB_TEST(cmep_sysctl_and_sc_window_inputs) {
    // The second loader validates the chip configuration before it hands
    // anything to the ARM (0x40DB6), and both inputs it reads have to answer:
    //
    //   [0xE3101000] == 0x0001000F   (SC interface status)
    //   ([0xE0010004] & 5) == 5, ([0xE0010004] & 8) == 0
    //   ([0xE0062020] & 7) == 0
    Fixture f;
    ZLB_EXPECT_EQ(f.bus.read32(0xE0010004) & 5u, 5u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0010004) & 8u, 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0062020) & 7u, 0u);

    // The CMeP reaches the SC window through CMeP.SecureCtl, which forwards to
    // Ernie's shared window: the interface status has to come back from there.
    ZLB_EXPECT_EQ(f.bus.read32(0xE3101000), 0x0001000Fu);

    // 0xE3101000 is a *latch*, not a constant: the second loader writes the strap
    // value into it and polls it back (0x48516), and later zeroes it and polls
    // that back as well (0x486BA).  A read-only constant made it spin at 0x486C2.
    f.bus.write32(0xE3101000, 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE3101000), 0u);
    f.bus.write32(0xE3101000, 0x0001000Fu);
    ZLB_EXPECT_EQ(f.bus.read32(0xE3101000), 0x0001000Fu);

    // Unknown SC registers behave like storage because the CMeP uses them as
    // write-then-poll strobes (0xE3102120 in the second loader at 0x487E8).
    f.bus.write32(0xE3102120, 1u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE3102120), 1u);
    f.bus.write32(0xE3101120, 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE3101120), 0u);

    // 0xE3110C00 is the same write-then-poll pattern (second loader 0x4864C) but
    // sits above the first 64 KiB, so the CMeP's SC window has to cover Ernie's
    // full 128 KiB span.
    f.bus.write32(0xE3110C00, 1u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE3110C00), 1u);
    f.bus.write32(0xE3110C00, 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE3110C00), 0u);
}

ZLB_TEST(cmep_sc_event_register_serves_each_side_its_own_bits) {
    // 0xE31000C0 is the SC event register (write-1-to-clear) and the two
    // processors wait for different bits in it: the second loader's SC bring-up
    // needs *exactly* 1 (0x485D6..0x485F8) while the ARM's kernel boot loader
    // needs 0x8/0x10 (0x4003C02E, 0x4003C066).  The CMeP therefore sees its own
    // pending bit and acknowledges it with a write of 1.
    Fixture f;

    ZLB_EXPECT_EQ(f.bus.read32(0xE31000C0), 1u);
    f.bus.write32(0xE31000C0, 0u);   // write-1-to-clear: writing 0 changes nothing
    ZLB_EXPECT_EQ(f.bus.read32(0xE31000C0), 1u);
    f.bus.write32(0xE31000C0, 1u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE31000C0), 0u);
}

ZLB_TEST(cmep_secure_ctl_forwards_to_the_shared_sc_window) {
    // Regression guard for the SC unification bug: CmepBlock::attach_shared_sc
    // used the Impl pointer, which install() had already handed to the bus (and
    // therefore nulled), so the attachment silently did nothing and the CMeP
    // kept reading a private copy of the SC registers.
    Fixture f;

    RegisterFile shared("Test.SC", 0xE3100000, 0x10000);
    shared.define(0xE3100200, "TEST_SC_REGISTER", 0x5A5A0000);
    f.cmep.attach_shared_sc(&shared);

    ZLB_EXPECT_EQ(f.bus.read32(0xE3100200), 0x5A5A0000u);

    // While a shared window is attached it answers the whole 64 KiB range, so an
    // address it does not define reads as zero instead of falling back to the
    // bridge's private copy - that is what makes the two processors see one block.
    ZLB_EXPECT_EQ(f.bus.read32(0xE3101000), 0u);
}

ZLB_TEST(cmep_cmd_block_status_and_byte_port) {
    // The block at 0xE6008000 is not identified yet; what the model has to
    // satisfy is the contract the second loader checks:
    //   * the status nibble reports the highest set bit of the slot command
    //     (the init at 0x4840E..0x484B0 writes 5/2/1 and waits for 4/2/1);
    //   * the byte port keeps the written command in the low byte and puts the
    //     device answer in bits 8..15 (0x4827E write-verify, 0x48370 read).
    Fixture f;
    f.bus.write32(0xE6008000, 5);
    f.bus.write32(0xE6008008, 2);
    f.bus.write32(0xE6008010, 1);
    ZLB_EXPECT_EQ(f.bus.read32(0xE6008020) & 0xFu, 4u);
    ZLB_EXPECT_EQ((f.bus.read32(0xE6008020) >> 4) & 0xFu, 2u);
    ZLB_EXPECT_EQ((f.bus.read32(0xE6008020) >> 8) & 0xFu, 1u);

    f.bus.write32(0xE6008120, 2);
    ZLB_EXPECT_EQ(f.bus.read32(0xE6008120), 2u);          // write-verify path
    f.bus.write32(0xE6008120, 5);
    ZLB_EXPECT_EQ((f.bus.read32(0xE6008120) >> 8) & 0xFFu, 1u);   // device answer
    f.bus.write32(0xE6008120, 8);
    const u32 answer = (f.bus.read32(0xE6008120) >> 8) & 0xFFu;
    ZLB_EXPECT_EQ(answer & 3u, 0u);
    ZLB_EXPECT_EQ((answer >> 2) & 0xFu, 4u);

    f.bus.write32(0xE6008180, 1);
    ZLB_EXPECT_EQ(f.bus.read32(0xE6008180), 1u);
}

// ---------------------------------------------------------------------------
// Development substitutions in the second loader
// ---------------------------------------------------------------------------

ZLB_TEST(second_loader_substitutions_stay_inside_the_staged_image) {
    // Round 91: the "ARM boot-context gate" entry was written as the absolute
    // address 0x40850 while the apply loop adds the image base 0x40000, so the
    // patch went to 0x80850 - unmapped memory - and silently did nothing.  Every
    // entry has to land inside the payload the chain hashes (0x16600 bytes), and
    // each one needs a name so the log can identify it.
    size_t count = 0;
    const cmep_detail::SecondLoaderPatch* patches = cmep_detail::second_loader_patches(count);
    ZLB_EXPECT_TRUE(patches != nullptr);
    ZLB_EXPECT_TRUE(count > 0);
    for (size_t i = 0; i < count; ++i) {
        const cmep_detail::SecondLoaderPatch& patch = patches[i];
        ZLB_EXPECT_TRUE(patch.what != nullptr && patch.what[0] != '\0');
        ZLB_EXPECT_TRUE(patch.length == 2 || patch.length == 4);
        const u32 end = patch.offset + patch.length;
        ZLB_EXPECT_TRUE(end <= cmep_detail::kSecondLoaderStagedBytes);
        // The patch has to be reachable at the image base the chain stages it at.
        ZLB_EXPECT_TRUE(!patch.forced || patch.word != 0u);
    }
}
