// zeliboba - SCE container (SELF/SRVK/SPKG) parsing and decryption.
//
// Struct definitions follow pup_fiction/scetypes.py, the pipeline follows
// pup_fiction/sceutils.py + self2elf.py + scedecrypt.py:
//
//   SceHeader (32)  -> SelfHeader (88) -> AppInfo (32) -> SceVersionInfo (16)
//   -> ControlInfo / ControlInfoDigest256 / ControlInfoDRM (NPDRM)
//   -> ELF header (52, stored in the clear) -> e_phnum program headers (32 each)
//   -> the same number of SegmentInfo (32)
//   -> [if any SegmentInfo.plaintext == NO] metadata decryption:
//        AES-CBC(metadata key, iv) over SceHeader.metadata_offset+48
//        (the first 64 bytes must decrypt to a MetadataInfo whose padding -
//         bytes 16..31 and 48..63 - is zero, which is how the right key out of
//         the candidate list is identified; APP SELFs additionally XOR the first
//         64 bytes through the NPDRM key and the RIF klicensee)
//        -> MetadataInfo(key, iv) -> AES-CBC(key, iv) over the rest
//        -> MetadataHeader -> MetadataSection[] -> 16 byte key vault
//        every section with encryption == AES128CTR contributes (key, iv)
//   -> for each metadata section: AES-128-CTR the segment bytes and (when
//      SegmentInfo.compressed == YES) inflate them with zlib/DEFLATE
//
// Layout of a retail SELF (verified on Out/SLB2/kernel_boot_loader.self and on
// the devkit SDK .suprx samples):
//   * the inner ELF header and the program header table are stored *in the
//     clear* at self_header.elf_offset / phdr_offset, so the arch is known
//     without any key;
//   * SegmentInfo.offset == program_header.p_offset + SceHeader.header_length
//     and shdr_offset == e_shoff + header_length: the whole data area
//     [header_length, header_length + data_length) is the original ELF image,
//     with the encrypted segments encrypted in place. self_to_elf therefore uses
//     that region as the base and overwrites the segments it could decrypt.
//
// Deviation from the C#/python reference (documented, deliberate): the
// reference keys the metadata sections by *section index* and then uses
// `scesegs[i].idx` as the segment index, which only works while section i
// describes segment i+1. kernel_boot_loader.self has 5 program headers and 4
// metadata sections (segment 1..4; segment 0 is PT_NULL with SecureBool UNUSED
// and has no section at all), so the reference raises KeyError / "no AES128CTR
// key" there. We key the sections by their own seg_idx field instead, which
// decrypts that file correctly.
#include <algorithm>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "common/log.h"
#include "common/util.h"
#include "loader/loader.h"
#include "loader/loader_extra.h"

namespace zlb {

namespace {

// ---------------------------------------------------------------------------
// SCE structures (scetypes.py)
// ---------------------------------------------------------------------------

constexpr u32 kSceMagic = 0x00454353u;         // "SCE\0"
constexpr u32 kSceHeaderVersion = 3;
constexpr size_t kSceHeaderSize = 32;
constexpr size_t kSelfHeaderSize = 88;
constexpr size_t kAppInfoSize = 32;
constexpr size_t kSegmentInfoSize = 32;
constexpr size_t kElfHeaderSize = 52;
constexpr size_t kMetadataInfoSize = 64;
constexpr size_t kMetadataHeaderSize = 32;
constexpr size_t kMetadataSectionSize = 48;

constexpr int kSecureBoolUnused = 0;
constexpr int kSecureBoolNo = 1;
constexpr int kSecureBoolYes = 2;

constexpr int kEncryptionAes128Ctr = 3;
constexpr int kCompressionDeflate = 2;

struct SceHeaderFields {
    u32 magic = 0;
    u32 version = 0;
    u8 platform = 0;
    u8 key_revision = 0;
    u16 sce_type = 0;
    u32 metadata_offset = 0;
    u64 header_length = 0;
    u64 data_length = 0;
};

struct SelfHeaderFields {
    u64 file_length = 0;
    u64 field_8 = 0;
    u64 self_offset = 0;
    u64 appinfo_offset = 0;
    u64 elf_offset = 0;
    u64 phdr_offset = 0;
    u64 shdr_offset = 0;
    u64 segment_info_offset = 0;
    u64 sceversion_offset = 0;
    u64 controlinfo_offset = 0;
    u64 controlinfo_length = 0;
};

struct AppInfoFields {
    u64 auth_id = 0;
    u32 vendor_id = 0;
    u32 self_type = 0;
    u64 sys_version = 0;
    u64 field_18 = 0;
};

struct SegmentInfoFields {
    u64 offset = 0;
    u64 size = 0;
    int compressed = kSecureBoolUnused;
    int plaintext = kSecureBoolUnused;
};

struct MetadataSectionFields {
    u64 offset = 0;
    u64 size = 0;
    int type = 0;
    int seg_idx = 0;
    int hash_type = 0;
    int hash_idx = 0;
    int encryption = 0;
    int key_idx = 0;
    int iv_idx = 0;
    int compression = 0;
};

bool parse_sce_header_fields(const std::vector<u8>& data, SceHeaderFields& out) {
    if (data.size() < kSceHeaderSize) return false;
    out.magic = ld::read_u32(data, 0);
    if (out.magic != kSceMagic) return false;
    out.version = ld::read_u32(data, 4);
    out.platform = ld::read_u8(data, 8);
    out.key_revision = ld::read_u8(data, 9);
    out.sce_type = ld::read_u16(data, 10);
    out.metadata_offset = ld::read_u32(data, 12);
    out.header_length = ld::read_u64(data, 16);
    out.data_length = ld::read_u64(data, 24);
    return true;
}

bool parse_self_header_fields(const std::vector<u8>& data, SelfHeaderFields& out) {
    if (data.size() < kSceHeaderSize + kSelfHeaderSize) return false;
    const size_t base = kSceHeaderSize;
    out.file_length = ld::read_u64(data, base + 0x00);
    out.field_8 = ld::read_u64(data, base + 0x08);
    out.self_offset = ld::read_u64(data, base + 0x10);
    out.appinfo_offset = ld::read_u64(data, base + 0x18);
    out.elf_offset = ld::read_u64(data, base + 0x20);
    out.phdr_offset = ld::read_u64(data, base + 0x28);
    out.shdr_offset = ld::read_u64(data, base + 0x30);
    out.segment_info_offset = ld::read_u64(data, base + 0x38);
    out.sceversion_offset = ld::read_u64(data, base + 0x40);
    out.controlinfo_offset = ld::read_u64(data, base + 0x48);
    out.controlinfo_length = ld::read_u64(data, base + 0x50);
    return true;
}

bool parse_app_info_fields(const std::vector<u8>& data, u64 offset, AppInfoFields& out) {
    if (offset + kAppInfoSize > data.size()) return false;
    const size_t base = static_cast<size_t>(offset);
    out.auth_id = ld::read_u64(data, base + 0);
    out.vendor_id = ld::read_u32(data, base + 8);
    out.self_type = ld::read_u32(data, base + 12);
    out.sys_version = ld::read_u64(data, base + 16);
    out.field_18 = ld::read_u64(data, base + 24);
    return true;
}

bool parse_segment_info(const std::vector<u8>& data, size_t offset, SegmentInfoFields& out) {
    if (offset + kSegmentInfoSize > data.size()) return false;
    out.offset = ld::read_u64(data, offset + 0);
    out.size = ld::read_u64(data, offset + 8);
    out.compressed = static_cast<int>(ld::read_u32(data, offset + 16));
    out.plaintext = static_cast<int>(ld::read_u32(data, offset + 24));
    return true;
}

// ---------------------------------------------------------------------------
// DEFLATE (RFC 1951) / zlib (RFC 1950)
// ---------------------------------------------------------------------------

constexpr size_t kDefaultInflateLimit = 64u * 1024u * 1024u;

struct BitReader {
    const u8* data = nullptr;
    size_t size = 0;
    size_t byte_pos = 0;
    u32 buffer = 0;
    unsigned count = 0;
    bool failed = false;

    u32 bits(unsigned wanted) {
        while (count < wanted) {
            if (byte_pos >= size) {
                failed = true;
                return 0;
            }
            buffer |= static_cast<u32>(data[byte_pos++]) << count;
            count += 8;
        }
        const u32 value = buffer & ((1u << wanted) - 1u);
        buffer >>= wanted;
        count -= wanted;
        return value;
    }

    void align() {
        buffer = 0;
        count = 0;
    }
};

struct Huffman {
    u16 counts[16] = {};
    std::vector<u16> symbols;

    bool build(const u8* lengths, size_t count) {
        for (int i = 0; i < 16; ++i) counts[i] = 0;
        for (size_t i = 0; i < count; ++i) ++counts[lengths[i]];
        if (counts[0] == count) return true;   // complete but empty code

        int left = 1;
        for (int length = 1; length < 16; ++length) {
            left <<= 1;
            left -= counts[length];
            if (left < 0) return false;        // over subscribed
        }

        u16 offsets[16] = {};
        for (int length = 1; length < 15; ++length)
            offsets[length + 1] = static_cast<u16>(offsets[length] + counts[length]);
        symbols.assign(count, 0);
        for (size_t symbol = 0; symbol < count; ++symbol) {
            if (lengths[symbol] != 0) symbols[offsets[lengths[symbol]]++] = static_cast<u16>(symbol);
        }
        return true;
    }
};

int decode_symbol(BitReader& reader, const Huffman& table) {
    int code = 0;
    int first = 0;
    int index = 0;
    for (int length = 1; length < 16; ++length) {
        code |= static_cast<int>(reader.bits(1));
        if (reader.failed) return -1;
        const int count = table.counts[length];
        if (code - first < count) return table.symbols[index + (code - first)];
        index += count;
        first = (first + count) << 1;
        code <<= 1;
    }
    return -1;
}

constexpr u16 kLengthBase[29] = {3,  4,  5,  6,  7,  8,  9,  10,  11,  13,  15,  17,  19, 23, 27,
                                 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
constexpr u8 kLengthExtra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
                                 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
constexpr u16 kDistanceBase[30] = {1,    2,    3,    4,    5,    7,     9,     13,    17,   25,
                                   33,   49,   65,   97,   129,  193,   257,   385,   513,  769,
                                   1025, 1537, 2049, 3073, 4097, 6145,  8193,  12289, 16385, 24577};
constexpr u8 kDistanceExtra[30] = {0, 0, 0,  0,  1,  1,  2,  2,  3,  3,  4,  4,  5,  5,  6,
                                   6, 7, 7,  8,  8,  9,  9,  10, 10, 11, 11, 12, 12, 13, 13};
constexpr u8 kCodeLengthOrder[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};

bool inflate_block(BitReader& reader, std::vector<u8>& out, size_t limit) {
    const u32 final_block = reader.bits(1);
    const u32 type = reader.bits(2);
    if (reader.failed) return false;

    if (type == 0) {   // stored
        reader.align();
        if (reader.byte_pos + 4 > reader.size) return false;
        const u32 length = static_cast<u32>(reader.data[reader.byte_pos]) |
                           (static_cast<u32>(reader.data[reader.byte_pos + 1]) << 8);
        const u32 complement = static_cast<u32>(reader.data[reader.byte_pos + 2]) |
                               (static_cast<u32>(reader.data[reader.byte_pos + 3]) << 8);
        reader.byte_pos += 4;
        if ((length ^ 0xFFFFu) != complement) return false;
        if (reader.byte_pos + length > reader.size) return false;
        if (out.size() + length > limit) return false;
        out.insert(out.end(), reader.data + reader.byte_pos, reader.data + reader.byte_pos + length);
        reader.byte_pos += length;
        return final_block != 0;
    }

    Huffman literal;
    Huffman distance;
    if (type == 1) {   // fixed
        u8 literal_lengths[288];
        for (int i = 0; i < 144; ++i) literal_lengths[i] = 8;
        for (int i = 144; i < 256; ++i) literal_lengths[i] = 9;
        for (int i = 256; i < 280; ++i) literal_lengths[i] = 7;
        for (int i = 280; i < 288; ++i) literal_lengths[i] = 8;
        u8 distance_lengths[30];
        for (int i = 0; i < 30; ++i) distance_lengths[i] = 5;
        if (!literal.build(literal_lengths, 288)) return false;
        if (!distance.build(distance_lengths, 30)) return false;
    } else if (type == 2) {   // dynamic
        const unsigned hlit = reader.bits(5) + 257;
        const unsigned hdist = reader.bits(5) + 1;
        const unsigned hclen = reader.bits(4) + 4;
        if (reader.failed) return false;

        u8 code_lengths[19] = {};
        for (unsigned i = 0; i < hclen; ++i) code_lengths[kCodeLengthOrder[i]] =
            static_cast<u8>(reader.bits(3));
        if (reader.failed) return false;

        Huffman code_table;
        if (!code_table.build(code_lengths, 19)) return false;

        std::vector<u8> lengths(hlit + hdist, 0);
        size_t index = 0;
        while (index < lengths.size()) {
            const int symbol = decode_symbol(reader, code_table);
            if (symbol < 0) return false;
            if (symbol < 16) {
                lengths[index++] = static_cast<u8>(symbol);
            } else if (symbol == 16) {
                if (index == 0) return false;
                const unsigned repeat = 3 + reader.bits(2);
                const u8 previous = lengths[index - 1];
                for (unsigned i = 0; i < repeat && index < lengths.size(); ++i)
                    lengths[index++] = previous;
            } else if (symbol == 17) {
                const unsigned repeat = 3 + reader.bits(3);
                for (unsigned i = 0; i < repeat && index < lengths.size(); ++i) lengths[index++] = 0;
            } else {
                const unsigned repeat = 11 + reader.bits(7);
                for (unsigned i = 0; i < repeat && index < lengths.size(); ++i) lengths[index++] = 0;
            }
            if (reader.failed) return false;
        }

        if (!literal.build(lengths.data(), hlit)) return false;
        if (!distance.build(lengths.data() + hlit, hdist)) return false;
    } else {
        return false;
    }

    for (;;) {
        const int symbol = decode_symbol(reader, literal);
        if (symbol < 0) return false;
        if (symbol < 256) {
            if (out.size() + 1 > limit) return false;
            out.push_back(static_cast<u8>(symbol));
            continue;
        }
        if (symbol == 256) break;   // end of block
        const int length_index = symbol - 257;
        if (length_index >= 29) return false;
        const unsigned length =
            kLengthBase[length_index] + reader.bits(kLengthExtra[length_index]);
        const int distance_index = decode_symbol(reader, distance);
        if (distance_index < 0 || distance_index >= 30) return false;
        const unsigned copy_distance =
            kDistanceBase[distance_index] + reader.bits(kDistanceExtra[distance_index]);
        if (reader.failed || copy_distance == 0 || copy_distance > out.size()) return false;
        if (out.size() + length > limit) return false;
        const size_t start = out.size() - copy_distance;
        for (unsigned i = 0; i < length; ++i) out.push_back(out[start + i]);
    }
    return final_block != 0;
}

bool inflate_raw(const u8* data, size_t size, std::vector<u8>& out, size_t limit) {
    BitReader reader;
    reader.data = data;
    reader.size = size;
    for (;;) {
        const bool final_block = inflate_block(reader, out, limit);
        if (reader.failed) return false;
        if (final_block) break;
        if (out.size() > limit) return false;
    }
    return !out.empty();
}

}  // namespace

bool has_zlib_header(const u8* data, size_t length) {
    if (data == nullptr || length < 2) return false;
    const unsigned cmf = data[0];
    const unsigned flg = data[1];
    return (cmf & 0x0F) == 8 && ((cmf << 8) | flg) % 31 == 0;
}

std::optional<std::vector<u8>> inflate_zlib_or_raw(const u8* input, size_t length, std::string* mode,
                                                   size_t max_output) {
    if (input == nullptr || length == 0) return std::nullopt;
    const size_t limit = max_output != 0 ? max_output : kDefaultInflateLimit;
    const bool zlib = has_zlib_header(input, length);

    std::vector<u8> out;
    const size_t first_skip = zlib ? 2 : 0;
    out.reserve(length * 4);
    if (inflate_raw(input + first_skip, length - first_skip, out, limit)) {
        if (mode != nullptr) *mode = zlib ? "zlib (2 byte header stripped)" : "raw deflate";
        return out;
    }

    // Retry the other interpretation (also covers a short/odd stream).
    out.clear();
    if (inflate_raw(input, length, out, limit)) {
        if (mode != nullptr) *mode = "raw deflate (zlib autodetect retry)";
        return out;
    }
    if (length > 2) {
        out.clear();
        if (inflate_raw(input + 2, length - 2, out, limit)) {
            if (mode != nullptr) *mode = "zlib (2 byte header stripped, autodetect retry)";
            return out;
        }
    }
    if (mode != nullptr) *mode = "failed";
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// Metadata decryption
// ---------------------------------------------------------------------------

namespace {

struct MetadataResult {
    std::vector<MetadataSectionFields> sections;
    std::vector<std::vector<u8>> vault;
    std::string key_source;
    int section_count = 0;
    int key_count = 0;
};

/// sceutils.get_key_type(): the (sys_version, self_type) pair used to pick the
/// metadata key. SRVK and SPKG containers carry their system version in their
/// own header *after* the SCE header.
void key_type_of(const std::vector<u8>& data, const SceHeaderFields& sce, s64& sys_version,
                 int& self_type) {
    sys_version = -1;
    self_type = static_cast<int>(SceSelfKind::None);
    if (sce.sce_type == static_cast<u16>(SceContainerKind::Self)) {
        SelfHeaderFields self;
        AppInfoFields app;
        if (parse_self_header_fields(data, self) &&
            parse_app_info_fields(data, self.appinfo_offset, app)) {
            sys_version = static_cast<s64>(app.sys_version);
            self_type = static_cast<int>(app.self_type);
        }
    } else if (sce.sce_type == static_cast<u16>(SceContainerKind::Srvk)) {
        // SrvkHeader (32 bytes): u32 field_0, u32 field_4, u64 sys_version, ...
        const size_t base = static_cast<size_t>(sce.header_length);
        if (base + 32 <= data.size())
            sys_version = static_cast<s64>(ld::read_u64(data, base + 8));
    } else if (sce.sce_type == static_cast<u16>(SceContainerKind::Spkg)) {
        // SpkgHeader (128 bytes): u32 field_0, u32 pkg_type, u32 flags, u32 field_C,
        // u64 update_version, ...; python uses update_version << 16.
        const size_t base = static_cast<size_t>(sce.header_length);
        if (base + 128 <= data.size()) {
            const u64 update_version = ld::read_u64(data, base + 0x10);
            sys_version = static_cast<s64>(update_version << 16);
        }
    }
}

/// The service the metadata key selection needs from the SceKeys store.
std::vector<SceKeyCandidate> metadata_candidates(const SceKeys& keys, const SceHeaderFields& sce,
                                                 s64 sys_version, int self_type) {
    std::vector<SceKeyCandidate> candidates = sce_key_candidates(
        keys, static_cast<int>(SceKeyKind::Metadata), sce.sce_type, false, sys_version,
        sce.key_revision, self_type);
    // Fallback of the reference implementation: ignore the system version
    // (self2elf.py's ignore_sysver=True).
    if (sys_version >= 0) {
        for (SceKeyCandidate& extra : sce_key_candidates(
                 keys, static_cast<int>(SceKeyKind::Metadata), sce.sce_type, true, sys_version,
                 sce.key_revision, self_type)) {
            const bool duplicate =
                std::any_of(candidates.begin(), candidates.end(),
                            [&extra](const SceKeyCandidate& existing) {
                                return existing.key == extra.key && existing.iv == extra.iv;
                            });
            if (!duplicate) candidates.push_back(std::move(extra));
        }
    }
    return candidates;
}

std::vector<SceKeyCandidate> npdrm_candidates(const SceKeys& keys, const SceHeaderFields& sce,
                                              s64 sys_version, int self_type) {
    const int key_index = sce.key_revision >= 2 ? 1 : 0;
    std::vector<SceKeyCandidate> candidates =
        sce_key_candidates(keys, static_cast<int>(SceKeyKind::Npdrm), sce.sce_type, false,
                           sys_version, key_index, self_type);
    if (sys_version >= 0) {
        for (SceKeyCandidate& extra : sce_key_candidates(
                 keys, static_cast<int>(SceKeyKind::Npdrm), sce.sce_type, true, sys_version,
                 key_index, self_type)) {
            candidates.push_back(std::move(extra));
        }
    }
    return candidates;
}

bool all_zero(const u8* data, size_t offset, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        if (data[offset + i] != 0) return false;
    }
    return true;
}

bool decrypt_metadata(const std::vector<u8>& data, const SceKeys& keys,
                      const std::vector<u8>& klicense, MetadataResult& out,
                      SelfDecryptReport& report) {
    SceHeaderFields sce;
    if (!parse_sce_header_fields(data, sce)) {
        report.stage = "sce header";
        report.message = "not an SCE container (magic 0x00454353 missing or file too small)";
        return false;
    }
    if (sce.header_length <= static_cast<u64>(sce.metadata_offset) + kMetadataInfoSize) {
        report.stage = "metadata";
        report.message = format("metadata region is empty (header_length 0x%llX <= "
                                "metadata_offset 0x%X + 64)",
                                static_cast<unsigned long long>(sce.header_length),
                                sce.metadata_offset);
        return false;
    }

    const size_t data_offset = static_cast<size_t>(sce.metadata_offset) + 48;
    const size_t data_length =
        static_cast<size_t>(sce.header_length - sce.metadata_offset - 48);
    if (data_offset + data_length > data.size() || data_length < kMetadataInfoSize) {
        report.stage = "metadata";
        report.message = format("metadata blob 0x%zX+0x%zX is outside the file (size 0x%zX)",
                                data_offset, data_length, data.size());
        return false;
    }

    s64 sys_version = -1;
    int self_type = 0;
    key_type_of(data, sce, sys_version, self_type);

    const std::vector<SceKeyCandidate> candidates =
        metadata_candidates(keys, sce, sys_version, self_type);
    if (candidates.empty()) {
        report.stage = "metadata key";
        report.message = format("no metadata key for sce_type=%u sys_version=0x%llX key_revision=%u "
                                "self_type=0x%02X",
                                sce.sce_type, static_cast<unsigned long long>(sys_version),
                                sce.key_revision, self_type);
        return false;
    }

    std::vector<SceKeyCandidate> npdrm;
    if (self_type == static_cast<int>(SceSelfKind::App)) {
        npdrm = npdrm_candidates(keys, sce, sys_version, self_type);
        if (npdrm.empty()) {
            report.stage = "npdrm key";
            report.message = format("no NPDRM key for sce_type=%u key_index=%d self_type=APP",
                                    sce.sce_type, sce.key_revision >= 2 ? 1 : 0);
            return false;
        }
        if (klicense.size() != 16) {
            report.stage = "klicensee";
            report.message = "SELF type APP is NPDRM encrypted: a 16 byte klicensee (from the RIF) "
                             "is required and self_to_elf() has no way to pass one";
            return false;
        }
    }

    u8 block[kMetadataInfoSize];
    std::memcpy(block, data.data() + data_offset, kMetadataInfoSize);
    u8 plain[kMetadataInfoSize];
    std::string used;
    bool decrypted = false;

    for (const SceKeyCandidate& candidate : candidates) {
        if (npdrm.empty()) {
            aes_cbc_decrypt_any(candidate.key, candidate.iv, block, kMetadataInfoSize, plain);
        } else {
            for (const SceKeyCandidate& np : npdrm) {
                u8 pre[16];
                aes_cbc_decrypt_any(np.key, np.iv, klicense.data(), 16, pre);
                u8 stage1[kMetadataInfoSize];
                aes_cbc_decrypt_any(std::vector<u8>(pre, pre + 16), np.iv, block, kMetadataInfoSize,
                                    stage1);
                aes_cbc_decrypt_any(candidate.key, candidate.iv, stage1, kMetadataInfoSize, plain);
                if (all_zero(plain, 16, 16) && all_zero(plain, 48, 16)) {
                    used = format("%s + %s", candidate.source.c_str(), np.source.c_str());
                    decrypted = true;
                    break;
                }
            }
        }
        if (!decrypted && all_zero(plain, 16, 16) && all_zero(plain, 48, 16)) {
            used = candidate.source;
            decrypted = true;
        }
        if (decrypted) break;
    }

    if (!decrypted) {
        report.stage = "metadata key";
        report.message = format("%zu candidate metadata key(s) did not verify (the decrypted "
                                "MetadataInfo padding is not zero) for key_revision=%u "
                                "self_type=0x%02X sys_version=0x%llX",
                                candidates.size(), sce.key_revision, self_type,
                                static_cast<unsigned long long>(sys_version));
        return false;
    }
    report.metadata_key = used;
    out.key_source = used;

    const std::vector<u8> metadata_key(plain + 0, plain + 16);
    const std::vector<u8> metadata_iv(plain + 32, plain + 48);

    const size_t body_length = data_length - kMetadataInfoSize;
    std::vector<u8> body(body_length);
    aes_cbc_decrypt_any(metadata_key, metadata_iv, data.data() + data_offset + kMetadataInfoSize,
                        body_length, body.data());

    if (body.size() < kMetadataHeaderSize) {
        report.stage = "metadata body";
        report.message = "decrypted metadata body is smaller than the metadata header";
        return false;
    }
    const int section_count = static_cast<int>(ld::read_u32(body, 12));
    const int key_count = static_cast<int>(ld::read_u32(body, 16));
    out.section_count = section_count;
    out.key_count = key_count;
    report.metadata_sections = section_count;
    report.metadata_keys = key_count;

    if (section_count < 0 ||
        kMetadataHeaderSize + static_cast<size_t>(section_count) * kMetadataSectionSize >
            body.size()) {
        report.stage = "metadata body";
        report.message = format("metadata section table (%d x 48) does not fit in 0x%zX bytes",
                                section_count, body.size());
        return false;
    }
    const size_t vault_offset =
        kMetadataHeaderSize + static_cast<size_t>(section_count) * kMetadataSectionSize;
    if (key_count < 0 || vault_offset + static_cast<size_t>(key_count) * 16 > body.size()) {
        report.stage = "metadata body";
        report.message = format("metadata key vault (%d keys) does not fit", key_count);
        return false;
    }

    for (int i = 0; i < key_count; ++i) {
        out.vault.push_back(
            ld::slice(body, vault_offset + static_cast<size_t>(i) * 16, 16));
    }

    for (int i = 0; i < section_count; ++i) {
        const size_t offset = kMetadataHeaderSize + static_cast<size_t>(i) * kMetadataSectionSize;
        MetadataSectionFields section;
        section.offset = ld::read_u64(body, offset + 0);
        section.size = ld::read_u64(body, offset + 8);
        section.type = static_cast<int>(ld::read_u32(body, offset + 16));
        section.seg_idx = static_cast<int>(ld::read_u32(body, offset + 20));
        section.hash_type = static_cast<int>(ld::read_u32(body, offset + 24));
        section.hash_idx = static_cast<int>(ld::read_u32(body, offset + 28));
        section.encryption = static_cast<int>(ld::read_u32(body, offset + 32));
        section.key_idx = static_cast<int>(ld::read_u32(body, offset + 36));
        section.iv_idx = static_cast<int>(ld::read_u32(body, offset + 40));
        section.compression = static_cast<int>(ld::read_u32(body, offset + 44));
        out.sections.push_back(section);
    }
    return true;
}

}  // namespace

std::vector<SceSegmentData> sce_decrypt_segments(const std::vector<u8>& data, const SceKeys& keys,
                                                 const std::vector<u8>& klicense,
                                                 SelfDecryptReport* report) {
    SelfDecryptReport local;
    SelfDecryptReport& state = report != nullptr ? *report : local;
    state = SelfDecryptReport{};

    SceHeaderFields sce;
    if (!parse_sce_header_fields(data, sce)) {
        state.stage = "sce header";
        state.message = "not an SCE container";
        return {};
    }

    MetadataResult metadata;
    if (!decrypt_metadata(data, keys, klicense, metadata, state)) return {};

    std::vector<SceSegmentData> out;
    for (size_t i = 0; i < metadata.sections.size(); ++i) {
        const MetadataSectionFields& section = metadata.sections[i];
        SceSegmentData segment;
        segment.index = static_cast<u32>(i);
        segment.segment_index = static_cast<u32>(section.seg_idx);
        segment.offset = static_cast<u32>(section.offset);
        segment.size = static_cast<u32>(section.size);
        segment.compressed = section.compression == kCompressionDeflate;
        segment.encrypted = section.encryption == kEncryptionAes128Ctr;
        if (!segment.encrypted) continue;

        if (section.key_idx < 0 || section.key_idx >= static_cast<int>(metadata.vault.size()) ||
            section.iv_idx < 0 || section.iv_idx >= static_cast<int>(metadata.vault.size())) {
            state.stage = format("segment %zu", i);
            state.message = format("metadata section references key/iv index %d/%d but the vault "
                                   "has %zu entries",
                                   section.key_idx, section.iv_idx, metadata.vault.size());
            return {};
        }

        const size_t offset = static_cast<size_t>(section.offset);
        size_t size = static_cast<size_t>(section.size);
        if (offset > data.size()) size = 0;
        if (offset + size > data.size()) size = data.size() - offset;

        const std::vector<u8>& key = metadata.vault[static_cast<size_t>(section.key_idx)];
        const std::vector<u8>& iv = metadata.vault[static_cast<size_t>(section.iv_idx)];
        std::vector<u8> plain(size);
        if (size > 0) aes_ctr_crypt(key.data(), iv.data(), data.data() + offset, size, plain.data());

        if (segment.compressed) {
            std::string mode;
            auto inflated = inflate_zlib_or_raw(plain.data(), plain.size(), &mode);
            if (!inflated) {
                state.stage = format("segment %zu", i);
                state.message = format("segment %zu (0x%zX bytes) is marked DEFLATE but does not "
                                       "inflate", i, plain.size());
                return {};
            }
            state.inflate_mode = mode;
            segment.notes = format("deflate %s 0x%zX -> 0x%zX", mode.c_str(), plain.size(),
                                   inflated->size());
            plain = std::move(*inflated);
        }
        segment.data = std::move(plain);
        out.push_back(std::move(segment));
    }
    state.ok = true;
    state.stage = "done";
    state.message = format("decrypted %zu segment(s)", out.size());
    return out;
}

SceInnerElfInfo sce_inner_elf_info(const std::vector<u8>& data) {
    SceInnerElfInfo info;
    SceHeaderFields sce;
    if (!parse_sce_header_fields(data, sce)) return info;
    if (sce.sce_type != static_cast<u16>(SceContainerKind::Self)) return info;

    SelfHeaderFields self;
    if (!parse_self_header_fields(data, self)) return info;
    if (!ld::in_range(data, static_cast<size_t>(self.elf_offset), kElfHeaderSize)) return info;

    const size_t elf = static_cast<size_t>(self.elf_offset);
    if (!(data[elf] == 0x7F && data[elf + 1] == 'E' && data[elf + 2] == 'L' && data[elf + 3] == 'F'))
        return info;

    info.present = true;
    info.machine = ld::read_u16(data, elf + 0x12);
    info.entry_raw = ld::read_u32(data, elf + 0x18);
    info.phnum = ld::read_u16(data, elf + 0x2C);
    const u16 ph_entry_size = ld::read_u16(data, elf + 0x2A);
    const size_t entry_size = ph_entry_size != 0 ? ph_entry_size : 32;

    u64 low = ~static_cast<u64>(0);
    u64 high = 0;
    for (u16 i = 0; i < info.phnum; ++i) {
        const size_t offset = static_cast<size_t>(self.phdr_offset) + i * entry_size;
        if (offset + 32 > data.size()) break;
        if (ld::read_u32(data, offset) != 1) continue;   // PT_LOAD
        const u64 vaddr = ld::read_u32(data, offset + 0x08);
        const u64 memsz = ld::read_u32(data, offset + 0x14);
        low = std::min(low, vaddr);
        high = std::max(high, vaddr + memsz);
    }
    if (high > low) {
        info.load_address = static_cast<u32>(low);
        info.span = high - low;
        info.entry_resolved = (info.entry_raw >= low && info.entry_raw < high)
                                  ? info.entry_raw
                                  : static_cast<u32>(low + info.entry_raw);
    } else {
        info.entry_resolved = info.entry_raw;
    }
    return info;
}

bool is_self(const std::vector<u8>& data) {
    return data.size() >= 4 && ld::read_u32(data, 0) == kSceMagic;
}

bool parse_self_header(const std::vector<u8>& data, SelfHeader& out) {
    out = SelfHeader{};
    SceHeaderFields sce;
    if (!parse_sce_header_fields(data, sce)) return false;

    out.platform = sce.platform;
    out.key_revision = sce.key_revision;
    out.header_len = static_cast<u32>(sce.header_length);
    out.sce_type = sce.sce_type;
    out.self_version = static_cast<u16>(sce.version);   // the SCE container version (3)
    out.elf_size = static_cast<u32>(sce.data_length);

    SelfHeaderFields self;
    if (!parse_self_header_fields(data, self)) return false;
    out.elf_offset = static_cast<u32>(self.elf_offset);

    AppInfoFields app;
    if (parse_app_info_fields(data, self.appinfo_offset, app)) {
        out.authid = static_cast<u32>(app.auth_id);
        out.vendor_id = app.vendor_id;
        out.self_type = static_cast<u16>(app.self_type);
        out.sys_version = app.sys_version;
    }

    // Raw segment info table (32 bytes per program header), for diagnostics.
    const u16 phnum = ld::read_u16(data, static_cast<size_t>(self.elf_offset) + 0x2C);
    for (u16 i = 0; i < phnum; ++i) {
        const size_t offset = static_cast<size_t>(self.segment_info_offset) + i * kSegmentInfoSize;
        if (offset + kSegmentInfoSize > data.size()) break;
        for (size_t b = 0; b < kSegmentInfoSize; ++b) out.segment_data.push_back(data[offset + b]);
    }
    return true;
}

std::optional<std::vector<u8>> self_to_elf_report(const std::vector<u8>& data, const SceKeys& keys,
                                                  const std::vector<u8>& klicense,
                                                  SelfDecryptReport& report) {
    report = SelfDecryptReport{};

    SceHeaderFields sce;
    if (!parse_sce_header_fields(data, sce)) {
        report.stage = "sce header";
        report.message = "not an SCE container (expected magic 0x00454353 \"SCE\\0\")";
        return std::nullopt;
    }
    if (sce.sce_type != static_cast<u16>(SceContainerKind::Self)) {
        report.stage = "sce header";
        report.message = format("SCE type %u is not SELF (1); SELF->ELF extraction does not apply "
                                "(use sce_decrypt_segments() for SRVK/SPKG)",
                                sce.sce_type);
        return std::nullopt;
    }

    SelfHeaderFields self;
    if (!parse_self_header_fields(data, self)) {
        report.stage = "self header";
        report.message = "file is smaller than the 88 byte SELF header";
        return std::nullopt;
    }
    AppInfoFields app;
    const bool have_app = parse_app_info_fields(data, self.appinfo_offset, app);

    if (!ld::in_range(data, static_cast<size_t>(self.elf_offset), kElfHeaderSize)) {
        report.stage = "inner elf";
        report.message = format("inner ELF header at 0x%llX is outside the file",
                                static_cast<unsigned long long>(self.elf_offset));
        return std::nullopt;
    }
    const u8* elf_magic = data.data() + static_cast<size_t>(self.elf_offset);
    if (!(elf_magic[0] == 0x7F && elf_magic[1] == 'E' && elf_magic[2] == 'L' &&
          elf_magic[3] == 'F')) {
        report.stage = "inner elf";
        report.message = format("no ELF magic at the inner ELF offset 0x%llX",
                                static_cast<unsigned long long>(self.elf_offset));
        return std::nullopt;
    }

    const u16 ph_entry_size = ld::read_u16(data, static_cast<size_t>(self.elf_offset) + 0x2A);
    const u16 phnum = ld::read_u16(data, static_cast<size_t>(self.elf_offset) + 0x2C);
    const u32 elf_phoff = ld::read_u32(data, static_cast<size_t>(self.elf_offset) + 0x1C);
    const size_t entry_size = ph_entry_size != 0 ? ph_entry_size : 32;

    struct ProgramHeader {
        u32 type = 0;
        u32 offset = 0;
        u32 vaddr = 0;
        u32 filesz = 0;
        u32 memsz = 0;
        u32 flags = 0;
    };
    std::vector<ProgramHeader> phdrs;
    std::vector<SegmentInfoFields> segment_infos;
    for (u16 i = 0; i < phnum; ++i) {
        const size_t po = static_cast<size_t>(self.phdr_offset) + i * entry_size;
        const size_t so = static_cast<size_t>(self.segment_info_offset) + i * kSegmentInfoSize;
        if (po + 32 > data.size()) {
            report.stage = "program headers";
            report.message = format("program header %u at 0x%zX is outside the file", i, po);
            return std::nullopt;
        }
        ProgramHeader phdr;
        phdr.type = ld::read_u32(data, po + 0x00);
        phdr.offset = ld::read_u32(data, po + 0x04);
        phdr.vaddr = ld::read_u32(data, po + 0x08);
        phdr.filesz = ld::read_u32(data, po + 0x10);
        phdr.memsz = ld::read_u32(data, po + 0x14);
        phdr.flags = ld::read_u32(data, po + 0x18);
        phdrs.push_back(phdr);

        SegmentInfoFields info;
        parse_segment_info(data, so, info);
        segment_infos.push_back(info);
    }

    // Any segment with SecureBool plaintext == NO needs the metadata pipeline.
    bool any_encrypted = false;
    for (const SegmentInfoFields& info : segment_infos) {
        if (info.plaintext == kSecureBoolNo) any_encrypted = true;
    }

    std::map<int, size_t> section_of_segment;   // seg_idx -> section index
    MetadataResult metadata;
    if (any_encrypted) {
        if (!decrypt_metadata(data, keys, klicense, metadata, report)) return std::nullopt;
        for (size_t i = 0; i < metadata.sections.size(); ++i) {
            const MetadataSectionFields& section = metadata.sections[i];
            if (section.encryption == kEncryptionAes128Ctr)
                section_of_segment[section.seg_idx] = i;
        }
    }

    // Base image: for a retail SELF the data area [header_length, +data_length)
    // is the original ELF, so it is used verbatim and the decrypted segments are
    // written over it.
    size_t base_offset = 0;
    bool have_base = false;
    for (size_t i = 0; i < phdrs.size() && i < segment_infos.size(); ++i) {
        if (phdrs[i].type != 1) continue;   // PT_LOAD
        if (segment_infos[i].offset == static_cast<u64>(phdrs[i].offset) + sce.header_length) {
            base_offset = static_cast<size_t>(sce.header_length);
            have_base = base_offset + sce.data_length <= data.size();
        }
        break;
    }

    std::vector<u8> out;
    if (have_base) {
        out = ld::slice(data, base_offset, static_cast<size_t>(sce.data_length));
    }
    // Make sure the ELF header and the program headers fit.
    const size_t header_span = std::max<size_t>(kElfHeaderSize, elf_phoff + phdrs.size() * entry_size);
    if (out.size() < header_span) out.resize(header_span, 0);

    // Decrypt / copy the segments.
    for (size_t i = 0; i < phdrs.size() && i < segment_infos.size(); ++i) {
        const ProgramHeader& phdr = phdrs[i];
        const SegmentInfoFields& info = segment_infos[i];
        const bool encrypted = info.plaintext == kSecureBoolNo;
        if (info.size == 0) continue;   // empty (BSS only) segment: nothing to decrypt

        const size_t source_offset = static_cast<size_t>(info.offset);
        size_t source_size = static_cast<size_t>(info.size);
        if (source_offset > data.size()) source_size = 0;
        if (source_offset + source_size > data.size()) source_size = data.size() - source_offset;

        std::vector<u8> plain;
        size_t write_size = 0;
        if (encrypted) {
            const auto it = section_of_segment.find(static_cast<int>(i));
            if (it == section_of_segment.end()) {
                report.stage = format("segment %zu", i);
                report.message = format("segment %zu is encrypted (SegmentInfo plaintext=NO) but no "
                                        "AES128CTR metadata section has seg_idx=%zu (%zu sections "
                                        "decrypted, vault=%zu)",
                                        i, i, metadata.sections.size(), metadata.vault.size());
                return std::nullopt;
            }
            const MetadataSectionFields& section = metadata.sections[it->second];
            if (section.key_idx < 0 || section.key_idx >= static_cast<int>(metadata.vault.size()) ||
                section.iv_idx < 0 || section.iv_idx >= static_cast<int>(metadata.vault.size())) {
                report.stage = format("segment %zu", i);
                report.message = format("metadata section %zu references key/iv index %d/%d but the "
                                        "vault has %zu entries",
                                        it->second, section.key_idx, section.iv_idx,
                                        metadata.vault.size());
                return std::nullopt;
            }
            const std::vector<u8>& key = metadata.vault[static_cast<size_t>(section.key_idx)];
            const std::vector<u8>& iv = metadata.vault[static_cast<size_t>(section.iv_idx)];
            plain.resize(source_size);
            if (source_size > 0)
                aes_ctr_crypt(key.data(), iv.data(), data.data() + source_offset, source_size,
                              plain.data());
            std::string note = format("segment %zu: AES128CTR key=vault[%d] iv=vault[%d] 0x%zX bytes",
                                      i, section.key_idx, section.iv_idx, source_size);
            if (info.compressed == kSecureBoolYes) {
                std::string mode;
                auto inflated = inflate_zlib_or_raw(plain.data(), plain.size(), &mode);
                if (!inflated) {
                    report.stage = format("segment %zu", i);
                    report.message = format("segment %zu (0x%zX encrypted bytes) is marked DEFLATE "
                                            "but does not inflate",
                                            i, plain.size());
                    return std::nullopt;
                }
                report.inflate_mode = mode;
                note += format(" deflate %s -> 0x%zX bytes", mode.c_str(), inflated->size());
                plain = std::move(*inflated);
            }
            report.segment_notes.push_back(note);
            write_size = plain.size();
        } else if (!have_base) {
            // No verbatim base image: copy the plaintext bytes out of the
            // container (clipped to the segment's file size).
            write_size = std::min<size_t>(source_size, phdr.filesz);
        }

        if (write_size == 0) continue;
        const size_t destination = phdr.offset;
        if (destination + write_size > out.size()) out.resize(destination + write_size, 0);
        if (encrypted) {
            std::memcpy(out.data() + destination, plain.data(), write_size);
        } else if (!have_base) {
            if (source_offset + write_size <= data.size())
                std::memcpy(out.data() + destination, data.data() + source_offset, write_size);
        }
    }

    // The ELF header and the program header table are stored in the clear in
    // the SELF; write them last so a PT_NULL segment cannot clobber them.
    std::memcpy(out.data(), data.data() + static_cast<size_t>(self.elf_offset), kElfHeaderSize);
    for (size_t i = 0; i < phdrs.size(); ++i) {
        const size_t po = static_cast<size_t>(self.phdr_offset) + i * entry_size;
        if (elf_phoff + (i + 1) * entry_size > out.size()) break;
        std::memcpy(out.data() + elf_phoff + i * entry_size, data.data() + po, entry_size);
    }

    if (!is_elf(out)) {
        report.stage = "output";
        report.message = "the reconstructed image does not start with the ELF magic";
        return std::nullopt;
    }

    if (!elf_section_headers_sane(out)) {
        // The section header table of a retail SELF is encrypted outside the
        // metadata sections and cannot be recovered; drop it rather than point
        // e_shoff at garbage (the phdr view stays complete).
        out[0x20] = out[0x21] = out[0x22] = out[0x23] = 0;   // e_shoff
        out[0x2E] = out[0x2F] = 0;                           // e_shentsize
        out[0x30] = out[0x31] = 0;                           // e_shnum
        out[0x32] = out[0x33] = 0;                           // e_shstrndx
        report.section_headers_dropped = true;
    }

    report.ok = true;
    report.stage = "done";
    report.message = format("decrypted %zu segment(s), output 0x%zX bytes", report.segment_notes.size(),
                            out.size());
    if (have_app && app.self_type == static_cast<u32>(SceSelfKind::App))
        report.message += " (SELF type APP: a RIF klicensee was required and supplied)";
    return out;
}

std::optional<std::vector<u8>> self_to_elf(const std::vector<u8>& data, const SceKeys& keys) {
    SelfDecryptReport report;
    auto out = self_to_elf_report(data, keys, {}, report);
    if (!out) ZLB_LOG_WARN("loader", "self_to_elf failed at stage '%s': %s", report.stage.c_str(),
                           report.message.c_str());
    return out;
}

std::string describe_self(const std::vector<u8>& data) {
    SceHeaderFields sce;
    if (!parse_sce_header_fields(data, sce)) return "SCE container (unparsable)";
    SelfHeaderFields self;
    AppInfoFields app;
    const bool have_self = parse_self_header_fields(data, self);
    const bool have_app = have_self && parse_app_info_fields(data, self.appinfo_offset, app);

    std::string text = format("SCE type=%u platform=0x%02X keyrev=%u version=%u metadata=0x%X "
                              "header_len=0x%llX data_len=0x%llX",
                              sce.sce_type, sce.platform, sce.key_revision, sce.version,
                              sce.metadata_offset, static_cast<unsigned long long>(sce.header_length),
                              static_cast<unsigned long long>(sce.data_length));
    if (have_app) {
        text += format(" self_type=0x%02X sys_version=0x%llX authid=0x%llX", app.self_type,
                       static_cast<unsigned long long>(app.sys_version),
                       static_cast<unsigned long long>(app.auth_id));
    }
    if (have_self && ld::in_range(data, static_cast<size_t>(self.elf_offset), 0x34)) {
        const u16 machine = ld::read_u16(data, static_cast<size_t>(self.elf_offset) + 0x12);
        const u32 entry = ld::read_u32(data, static_cast<size_t>(self.elf_offset) + 0x18);
        const u16 phnum = ld::read_u16(data, static_cast<size_t>(self.elf_offset) + 0x2C);
        text += format(" inner=ELF %s entry=0x%X phnum=%u", elf_machine_name(machine), entry, phnum);
    }
    return text;
}

}  // namespace zlb
