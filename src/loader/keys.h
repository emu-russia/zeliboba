// zeliboba - cryptographic material used by the boot chain.
//
// The tables are the ones recovered from the firmware itself: the SceKeys set
// used to decrypt SELF/PUP containers (internal + external keyrings) and the
// per-model keyring layout of the eMMC.
#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include "common/types.h"

namespace zlb {

/// A named 128/256-bit key.
struct Key {
    std::string name;
    std::vector<u8> bytes;

    size_t size() const { return bytes.size(); }
};

/// One keyring slot as the hardware sees it: index, flags and 256 bits of data.
struct KeyringSlot {
    u32 index = 0;
    u32 flags = 0;
    std::array<u8, 32> value{};
    bool locked = false;
    bool present = false;
};

/// Key set used to unwrap the SCE containers found in the firmware.
class SceKeys {
public:
    SceKeys();

    static const SceKeys& default_keys();
    static SceKeys load_from_file(const std::string& path);

    /// Keyring 0x501 style meta-key : used for the CMAC of the SELF header.
    const std::vector<u8>& get(const std::string& name) const;
    bool has(const std::string& name) const;
    void set(const std::string& name, std::vector<u8> bytes);

    /// Key selection by SELF header fields, as the hardware keyring would do.
    const std::vector<u8>& select_self_key(u16 platform, u16 key_revision, u32 sce_type) const;

    std::vector<std::string> names() const;

    /// F00D keyring slots: they are initialised by the CMeP first loader and can
    /// be overwritten by the emulated hardware as the boot progresses.
    std::map<u32, KeyringSlot>& keyring() { return keyring_; }
    const std::map<u32, KeyringSlot>& keyring() const { return keyring_; }

    /// Keyring 0x501 record (boot mode passed from the first loader to stage 2).
    std::array<u8, 32> keyring_0501{};
    bool keyring_0501_valid = false;

private:
    std::map<std::string, std::vector<u8>> keys_;
    std::map<u32, KeyringSlot> keyring_;
};

// ---------------------------------------------------------------------------
// Primitive crypto used by the loaders
// ---------------------------------------------------------------------------

void aes128_cbc_decrypt(const u8* key, const u8* iv, const u8* input, size_t length, u8* output);
void aes128_ecb_encrypt_block(const u8* key, const u8* input, u8* output);
void aes128_ecb_decrypt_block(const u8* key, const u8* input, u8* output);
void aes256_cbc_decrypt(const u8* key, const u8* iv, const u8* input, size_t length, u8* output);
void aes256_ecb_decrypt_block(const u8* key, const u8* input, u8* output);

void sha256(const u8* data, size_t length, u8 out[32]);
void sha1(const u8* data, size_t length, u8 out[20]);
void hmac_sha256(const u8* key, size_t key_length, const u8* data, size_t length, u8 out[32]);

/// Big-endian arbitrary precision modular exponentiation (Bignum engine).
std::vector<u8> rsa_public(const std::vector<u8>& modulus, const std::vector<u8>& exponent,
                           const std::vector<u8>& input);

}  // namespace zlb
