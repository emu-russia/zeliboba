// zeliboba - unit tests for the CMeP ("F00D") security block.
//
// The tests drive the block the way the first loader does: through the memory
// mapped registers on a real Bus, using the register sequences documented in
// src/hw/cmep/*.cpp (every claim there carries the instruction address of the
// annotated boot ROM listing that proves it).
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/log.h"
#include "hw/cmep.h"
#include "hw/cmep/cmep_internal.h"
#include "machine/bootkeys_data.h"
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
                ZLB_EXPECT_TRUE(device->register_name(address) != nullptr);
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
        ZLB_EXPECT_FALSE(regs.empty());
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
        // staged words are little-endian; word 0 holds the flags.
        ZLB_EXPECT_EQ(slot.value[0], 0x11);
        ZLB_EXPECT_EQ(slot.value[31], 0x88);
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

// ---------------------------------------------------------------------------
// GPIO (0xE20A0000) - mailbox_debug_sc (0x5E4E4)
// ---------------------------------------------------------------------------

ZLB_TEST(cmep_gpio_handshake) {
    Fixture f;
    f.cmep.reset();
    f.bus.write32(0xE20A0000, 8);  // 0x5E4FA
    const u32 first = f.bus.read32(0xE20A0004);
    ZLB_EXPECT_TRUE((first & 0x10u) != 0);  // 0x5E4FC expects bit 4 set
    const u32 second = f.bus.read32(0xE20A0004);
    ZLB_EXPECT_TRUE((second & 0x10u) == 0);  // 0x5E560 waits for it to clear
    f.bus.write32(0xE20A0008, 8);            // GPIO set (0x5E558)
    ZLB_EXPECT_EQ(f.bus.read32(0xE20A0000) & 8u, 8u);
    f.bus.write32(0xE20A000C, 8);  // GPIO clear (0x5E5BA)
    ZLB_EXPECT_EQ(f.bus.read32(0xE20A0000) & 8u, 0u);
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
    for (int i = 0; i < 16; ++i) {
        key128[i] = static_cast<u8>(i);
        block[i] = static_cast<u8>(i);
    }
    const u8 want128[16] = {0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
                            0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a};
    u8 out[16];
    cmep_detail::aes_encrypt_block(key128, 128, block, out);
    ZLB_EXPECT_EQ(std::memcmp(out, want128, 16), 0);
    cmep_detail::aes_decrypt_block(key128, 128, out, block);
    for (int i = 0; i < 16; ++i) ZLB_EXPECT_EQ(block[i], static_cast<u8>(i));

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

    // Unknown SC registers behave like storage because the CMeP uses them as
    // write-then-poll strobes (0xE3102120 in the second loader at 0x487E8).
    f.bus.write32(0xE3102120, 1u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE3102120), 1u);
    f.bus.write32(0xE3101120, 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE3101120), 0u);
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

