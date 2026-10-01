#include "machine/bootkeys.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "common/log.h"
#include "common/util.h"
#include "loader/keys.h"
#include "machine/bootkeys_data.h"

namespace zlb {

namespace {


/// Build marker shared by both dumps of the resident first loader:
/// `A7 E6 05 63 01 00 01 00` (magic 0x6305E6A7, then e = 65537) followed by
/// twelve zero bytes.  It sits exactly four bytes *before* the DER/RSA parameter
/// blob the loader streams (the four bytes are the tail of the `00 00 C0 E0`
/// filler word), so it pins the layout without having to guess the blob offset.
///
/// The full 16 bytes matter.  Matching only `A7 E6 05 63 01 00 01 00` also hits
/// the marker's own start in the *other* dump's address order and, worse, the
/// prefix of the blob itself is enough to match 8 bytes 4 bytes late - a hit
/// that moves every derived constant by 4 and makes the RSA comparison fail with
/// no other symptom.
const u8 kParameterBlobMarker[16] = {0xA7, 0xE6, 0x05, 0x63, 0x01, 0x00, 0x01, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/// Distance from the marker to the parameter blob it introduces.
constexpr int kMarkerToBlob = 4;

/// Offsets inside the first loader's `.data`, measured from the parameter blob:
/// the 18-byte DigestInfo reference follows the 16-byte blob, and the keyring
/// 0x20 key material sits 0x20 bytes before it.
constexpr int kBlobToDigestInfo = 0x10;
constexpr int kBlobToTailKey = -0x20;

/// How far either build's `.data` may sit from the prototype layout: the retail
/// PCH snapshot is shifted by exactly -0x80 and nothing else is expected, but the
/// scan is written as a search rather than as a table so that it also answers
/// "this is some third build" instead of silently using wrong addresses.
constexpr int kLayoutScanLow = -0x180;
constexpr int kLayoutScanHigh = 0x180;

/// Lower-case hex of a byte range, for the log lines.
std::string hex16(const u8* data, size_t length) {
    std::string out;
    for (size_t i = 0; i < length; ++i) out += format("%02X", data[i]);
    return out;
}

/// How many marker bytes the fitted loader carries at `address` (reported by the
/// log and `--bootkeys`).
size_t verify_parameter_blob(Bus& bus, u32 address) {
    size_t matched = 0;
    for (size_t i = 0; i < sizeof(kParameterBlobMarker); ++i) {
        if (bus.read8(address + static_cast<u32>(i)) != kParameterBlobMarker[i]) break;
        ++matched;
    }
    return matched;
}

/// The block the loader's RSA verification must produce, built from the fitted
/// first loader's own ROM constants plus the digest the loader computes over the
/// staged image, so it always matches the image in use.
std::vector<u8> build_expected_message(Bus& cmep_bus, const std::vector<u8>& image,
                                      const FirstLoaderLayout& layout) {
    std::vector<u8> message;
    message.reserve(256);
    message.push_back(0x00);
    message.push_back(0x01);
    message.insert(message.end(), 203, 0xFF);
    message.push_back(0x00);

    std::vector<u8> tail(18, 0);
    cmep_bus.read_bytes(layout.digest_info, tail.data(), tail.size());
    message.insert(message.end(), tail.begin(), tail.end());

    // 0x5C898 memcmps the last 32 bytes of the block against 0x5EDC0, which
    // 0x5C7F6 filled with SHA-256(image[0:0x1C0]) - the image header up to (but
    // not including) the 256-byte signature at 0x1C0.  So the tail is not a ROM
    // constant: it is the digest of the very image being verified.
    u8 digest[32] = {};
    if (image.size() >= bootkey_addrs::kSignatureOffset) {
        sha256(image.data(), bootkey_addrs::kSignatureOffset, digest);
    }
    message.insert(message.end(), digest, digest + 32);

    return message;
}

}  // namespace

FirstLoaderLayout detect_first_loader_layout(Bus& cmep_bus, const std::vector<u8>& image) {
    // Where the marker sits in the prototype build; the retail snapshot puts the
    // same bytes 0x80 lower, so the scan only has to find that one distance.
    constexpr int kPrototypeMarker = static_cast<int>(bootkey_addrs::kExpectedHead) - kMarkerToBlob;
    FirstLoaderLayout layout;
    int marker = kPrototypeMarker;
    layout.known = (verify_parameter_blob(cmep_bus, static_cast<u32>(marker)) ==
                    sizeof(kParameterBlobMarker));
    if (!layout.known) {
        for (int candidate = kPrototypeMarker + kLayoutScanLow;
             candidate <= kPrototypeMarker + kLayoutScanHigh; ++candidate) {
            if (verify_parameter_blob(cmep_bus, static_cast<u32>(candidate)) !=
                sizeof(kParameterBlobMarker)) {
                continue;
            }
            layout.known = true;
            marker = candidate;
            break;
        }
    }
    if (layout.known) {
        layout.shift = marker - kPrototypeMarker;
        layout.prototype = (layout.shift == 0);
        layout.parameter_blob = static_cast<u32>(marker + kMarkerToBlob);
        layout.digest_info = static_cast<u32>(static_cast<int>(layout.parameter_blob) + kBlobToDigestInfo);
        layout.tail_key = static_cast<u32>(static_cast<int>(layout.parameter_blob) + kBlobToTailKey);
        layout.verified = sizeof(kParameterBlobMarker);
    } else {
        // No build marker: fall back to the prototype addresses so the checks
        // below report a mismatch instead of reading a random window.
        layout.parameter_blob = bootkey_addrs::kExpectedHead;
        layout.digest_info = bootkey_addrs::kDigestInfo;
        layout.tail_key = bootkey_addrs::kTailKey;
    }
    (void)image;
    return layout;
}

std::string describe_boot_keys(Bus& cmep_bus, const std::vector<u8>& image) {
    const FirstLoaderLayout layout = detect_first_loader_layout(cmep_bus, image);
    const std::vector<u8> message = build_expected_message(cmep_bus, image, layout);
    u8 digest[32] = {};
    sha256(message.data(), message.size(), digest);

    std::string out = format("fitted build    : %s (shift %d, blob 0x%05X, digest ref 0x%05X)\n",
                             layout.known ? layout.name() : "unknown build", layout.shift,
                             layout.parameter_blob, layout.digest_info);
    out += format("expected block  : %s ... %s\n", hex16(message.data(), 8).c_str(),
                  hex16(message.data() + 248, 8).c_str());
    out += format("sha256(block)   : %s\n", hex16(digest, 32).c_str());
    out += format("signed digest   : %s\n", hex16(bootkeys::kExpectedMessageDigest, 32).c_str());
    out += format("key table       : 0x%08X, strap 0x%08X, signature at image+0x%X\n",
                  bootkey_addrs::kKeyTable, bootkey_addrs::kStrap,
                  static_cast<unsigned>(bootkey_addrs::kSignatureOffset));
    if (image.empty()) {
        // Without the staged image the last 32 bytes of the block are unknown
        // (they are SHA-256 of its header), so the digest above is not the one the
        // loader will check - report the layout and say so instead of a verdict.
        out += "state           : give the staged image to check the signed block "
               "(the debugger's `boot` stages one)\n";
        return out;
    }
    out += std::memcmp(digest, bootkeys::kExpectedMessageDigest, 32) == 0
               ? "state           : the fitted first loader matches this development key\n"
               : "state           : MISMATCH - the development key was made for another loader build\n";
    return out;
}

bool provision_boot_keys(Bus& cmep_bus, std::vector<u8>& image, std::string& why) {
    if (image.size() < bootkey_addrs::kSignatureOffset + 256) {
        why = format("the staged payload is only %zu bytes, too small for a signature at 0x%X", image.size(),
                     bootkey_addrs::kSignatureOffset);
        return false;
    }

    // Which build is fitted decides where its ROM constants live (the retail PCH
    // snapshot is the same program with .data/.bss shifted by -0x80).
    const FirstLoaderLayout layout = detect_first_loader_layout(cmep_bus, image);
    ZLB_LOG_INFO("boot", "fitted first loader: %s (constant shift %d, blob at 0x%05X, %zu/%zu marker bytes verified)",
                 layout.known ? layout.name() : "unknown build", layout.shift, layout.parameter_blob,
                 layout.verified, sizeof(kParameterBlobMarker));

    // 1. Place the payload digest in the header's digest table.  0x5C9BA hands the
    //    32 bytes at image+0x1A0 to the keyring wrapper and 0x5CA00 memcmps the
    //    result against SHA-256 of the payload at image+0x2C0 with the length from
    //    image+0x10.  On hardware that block is unwrapped with a chip-resident
    //    keyring key that appears in no dump, so the model's unwrap is a copy (see
    //    BigmacDevice::execute) and the plaintext has to be placed here.  It sits
    //    inside image[0:0x1C0] and therefore inside the block the RSA signature
    //    covers, so it has to happen before the message below is built - the
    //    loader's own SHA-256/memcmp sequence then still runs unchanged.
    const size_t digest_at = bootkey_addrs::kDigestTableOffset;
    const size_t payload_at = bootkey_addrs::kPayloadOffset;
    if (image.size() >= payload_at && image.size() >= digest_at + 32) {
        u32 payload_length = 0;
        for (unsigned b = 0; b < 4; ++b) {
            payload_length |= static_cast<u32>(image[0x10 + b]) << (8 * b);
        }
        if (payload_length != 0 && payload_at + payload_length <= image.size()) {
            u8 payload_digest[32] = {};
            sha256(image.data() + payload_at, payload_length, payload_digest);
            std::memcpy(image.data() + digest_at, payload_digest, 32);
            ZLB_LOG_INFO("boot", "provisioned the payload digest at image+0x%zX (sha256 of 0x%X bytes)",
                         digest_at, payload_length);
        }
    }

    // 2. Build the block the engine must return.  It uses the loader's own ROM
    //    constants and the SHA-256 of the image header, so it only matches when the
    //    fitted first loader is the build this development key was made for.
    const std::vector<u8> message = build_expected_message(cmep_bus, image, layout);
    u8 digest[32] = {};
    sha256(message.data(), message.size(), digest);
    if (std::getenv("ZLB_BOOTKEY_TRACE") != nullptr) {
        u8 header_digest[32] = {};
        sha256(image.data(), bootkey_addrs::kSignatureOffset, header_digest);
        std::fprintf(stderr, "[bootkeys] layout blob=0x%05X digest_info=0x%05X tail_key=0x%05X shift=%d\n",
                     layout.parameter_blob, layout.digest_info, layout.tail_key, layout.shift);
        std::fprintf(stderr, "[bootkeys] digest_info = %s\n", hex16(message.data() + 223, 18).c_str());
        std::fprintf(stderr, "[bootkeys] header_sha  = %s\n", hex16(header_digest, 32).c_str());
        std::fprintf(stderr, "[bootkeys] block_sha   = %s\n", hex16(digest, 32).c_str());
    }
    if (std::memcmp(digest, bootkeys::kExpectedMessageDigest, 32) != 0) {
        why = format("the fitted first loader expects a different signed block (sha256 %s, "
                     "development key was made for %s)",
                     hex16(digest, 32).c_str(), hex16(bootkeys::kExpectedMessageDigest, 32).c_str());
        return false;
    }

    // 3. The hardware key table: 64 words, little-endian, word 0 least significant
    //    (the loader copies 64 words from 0xE0066000 + i*4 into 0xE0040400 + i*4).
    for (size_t i = 0; i < 256; i += 4) {
        u32 word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= static_cast<u32>(bootkeys::kModulusWindow[i + b]) << (8 * b);
        cmep_bus.write32(bootkey_addrs::kKeyTable + static_cast<u32>(i), word);
    }

    // 4. The strap: 0x5C808 only uses the table when its low 16 bits are non-zero,
    //    and 0x5C81A requires bit image[0x16] of it to be set. image[0x16] is 0 for
    //    the 1.04 second loader, so bit 0 selects key index 0.
    cmep_bus.write32(bootkey_addrs::kStrap, 1);

    // 5. Re-sign the staged payload: the loader copies the 64 words at image+0x1C0
    //    into the engine's base window.
    std::memcpy(image.data() + bootkey_addrs::kSignatureOffset, bootkeys::kSignatureWindow, 256);

    ZLB_LOG_INFO("boot", "provisioned the CMeP key table at 0x%08X and re-signed the staged payload",
                 bootkey_addrs::kKeyTable);
    return true;
}

}  // namespace zlb
