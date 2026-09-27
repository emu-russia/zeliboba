// zeliboba - MeP image header ("ENP" container, magic 0x64B2C8E5).
//
// second_loader.enp / secure_kernel.enp are the CMeP ("F00D") stages stored in
// the SLB2 block. The header layout and its validation are the checks the CMeP
// boot ROM performs before it jumps into the image
// (dumps/bootrom_analysis/ANALYSIS.md 4.3 "Header validation of the loaded
// image (validate_header, 0x5C61C)", header at 0x40000):
//
//   +0x00  u32 magic  == 0x64B2C8E5
//   +0x04  u32 size     >= 0x2B0 and < 0x10000
//   +0x08  u32 offset   )  size == offset + length + 0x2B0
//   +0x0C  u32 length   )
//   +0x10  u32 field    field + size < 0x1C000
//   +0x16  u16 field    < 0x10
//
// The same field the ANALYSIS calls `size` (+0x04) is what
// pup_fiction.enc_decrypt() reads as the *offset* of the payload and
// field +0x10 as the payload length; both describe the real files:
//
//   second_loader.enp (93184 = 0x16C00 bytes)
//      magic=0x64B2C8E5 size=0x2C0 offset=0x10 length=0 field_0x10=0x16600
//      u16@0x16=0   -> 0x2C0 == 0x10 + 0 + 0x2B0 and 0x16600 + 0x2C0 < 0x1C000
//      payload = [0x2C0, 0x2C0 + 0x16600) -> the same 0x16600 = 91648 bytes as
//      the extracted Out/SLB2_dec/second_loader.bin
//   secure_kernel.enp (33280 = 0x8200 bytes)
//      size=0x2C0 offset=0x10 length=0 field_0x10=0x7C00 = 31744 =
//      Out/SLB2_dec/secure_kernel.bin
//
// The payload is AES-128-CBC encrypted with ENC_KEY/ENC_IV for the ".enc"
// variant and already decrypted for the ".enp" variant.
#include <string>
#include <vector>

#include "loader/loader.h"
#include "loader/loader_extra.h"

namespace zlb {

namespace {

constexpr u32 kMepMagic = 0x64B2C8E5u;
constexpr u32 kMepHeaderBase = 0x2B0u;      // fixed part of the header
constexpr u32 kMepSizeMin = 0x2B0u;
constexpr u32 kMepSizeMax = 0x10000u;
constexpr u32 kMepWindowEnd = 0x1C000u;     // 0x40000 + 0x1C000 covers the window

}  // namespace

bool is_mep_image(const std::vector<u8>& data) {
    return data.size() >= 0x18 && ld::read_u32(data, 0) == kMepMagic;
}

bool parse_mep_header(const std::vector<u8>& data, MepImageHeader& out) {
    out = MepImageHeader{};
    if (data.size() < 0x18) return false;

    out.magic = ld::read_u32(data, 0x00);
    out.size = ld::read_u32(data, 0x04);
    out.offset = ld::read_u32(data, 0x08);
    out.length = ld::read_u32(data, 0x0C);
    out.field_0x10 = ld::read_u32(data, 0x10);
    out.field_0x16 = ld::read_u16(data, 0x16);

    if (out.magic != kMepMagic) return false;
    if (out.size < kMepSizeMin || out.size >= kMepSizeMax) return false;
    if (out.size != out.offset + out.length + kMepHeaderBase) return false;
    if (static_cast<u64>(out.field_0x10) + out.size >= kMepWindowEnd) return false;
    if (out.field_0x16 >= 0x10) return false;

    out.valid = true;
    return true;
}

std::optional<std::vector<u8>> mep_payload(const std::vector<u8>& data,
                                           const MepImageHeader& header) {
    if (!header.valid) return std::nullopt;
    const size_t start = header.size;   // the payload starts right after the header
    if (start > data.size()) return std::nullopt;
    size_t length = header.field_0x10;
    if (start + length > data.size()) length = data.size() - start;
    if (length == 0) return std::nullopt;
    return ld::slice(data, start, length);
}

std::string describe_mep_header(const MepImageHeader& header) {
    if (!header.valid) return "MeP image header (invalid)";
    return format("MeP image header magic=0x%08X size=0x%X offset=0x%X length=0x%X "
                  "field_0x10=0x%X field_0x16=%u payload=0x%X bytes at 0x%X",
                  header.magic, header.size, header.offset, header.length, header.field_0x10,
                  header.field_0x16, header.field_0x10, header.size);
}

}  // namespace zlb
