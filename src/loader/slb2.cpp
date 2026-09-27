// zeliboba - SLB2 container ("second loader block 2").
//
// The SLB2 partition of the eMMC (and the decrypted inside of
// boot_slb2-00.pkg from the PUP) is one contiguous blob: a 0x200 byte header
// block, a table of 0x30 byte entries and the entry payloads at 512 byte block
// boundaries.
//
//   0x00  char[4] "SLB2"                     (u32 LE 0x32424C53)
//   0x04  u32     version        = 1
//   0x08  u32     header_size    = 0x200
//   0x0C  u32     entry_count
//   0x10  u32     total_size     (capacity in 512 byte blocks; the 1.04
//                                 container says 0x2000 = 4 MiB while only
//                                 0x4EE blocks are used)
//   0x14  u8[0x0C] zero
//   entries from 0x20, stride 0x30:
//     +0x00 u32 offset_in_512_byte_blocks
//     +0x04 u32 size_in_bytes
//     +0x08 u32 flags
//     +0x0C u32 reserved
//     +0x10 char[32] name, NUL padded
//
// Everything beyond the used blocks is 0xFF (the erased eMMC pattern).
//
// Evidence — Out/PUP_dec/boot_slb2-00.pkg.seg02 (647168 = 0x9E000 bytes), the
// decrypted boot_slb2 SPKG segment, parsed with the layout above:
//   magic "SLB2" version=1 header_size=0x200 count=7 total=0x2000, and the 7
//   entries reproduce the sizes of the files pup_fiction extracted into
//   Out/SLB2/ exactly:
//     [0] blk=0x001 off=0x00200 size=93184  second_loader.enp
//     [1] blk=0x0B7 off=0x16E00 size=93184  second_loader.enc
//     [2] blk=0x16D off=0x2DA00 size=33280  secure_kernel.enp
//     [3] blk=0x1AE off=0x35C00 size=33280  secure_kernel.enc
//     [4] blk=0x1EF off=0x3DE00 size=355220 kernel_boot_loader.self
//     [5] blk=0x4A5 off=0x94A00 size=35064  kprx_auth_sm.self
//     [6] blk=0x4EA off=0x9D400 size=1728   prog_rvk.srvk
//   The block offsets are exactly the running sum of
//   ceil(size / 0x200) starting at block 1, and the data of every entry starts
//   with the expected magic (0x64B2C8E5 for the .enp files, 0x00454353 for the
//   .self files), which is what fixes the layout. The same container layout is
//   present in the raw eMMC dump (_scratch/emmcdump.bin) as the SLB2 partition.
#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "common/util.h"
#include "loader/loader.h"
#include "loader/loader_extra.h"

namespace zlb {

namespace {

constexpr u32 kSlb2Magic = 0x32424C53u;   // "SLB2"
constexpr u32 kSlb2HeaderSize = 0x200u;
constexpr u32 kSlb2BlockSize = 0x200u;
constexpr u32 kSlb2EntrySize = 0x30u;
constexpr u32 kSlb2EntryBase = 0x20u;
constexpr u32 kSlb2NameSize = 0x20u;
constexpr u8 kSlb2ErasedFill = 0xFFu;

}  // namespace

bool is_slb2(const std::vector<u8>& data) {
    return data.size() >= 0x20 && ld::read_u32(data, 0) == kSlb2Magic;
}

std::optional<Slb2Image> parse_slb2(const std::vector<u8>& data) {
    if (!is_slb2(data)) return std::nullopt;

    Slb2Image image;
    image.version = ld::read_u32(data, 0x04);
    u32 count = ld::read_u32(data, 0x0C);

    // The entry table has to fit in the file; a bogus count is clamped instead
    // of failing the whole parse (the container may have been truncated).
    const u32 max_entries =
        static_cast<u32>((data.size() - kSlb2EntryBase) / kSlb2EntrySize);
    if (count > max_entries) count = max_entries;

    for (u32 i = 0; i < count; ++i) {
        const size_t offset = kSlb2EntryBase + static_cast<size_t>(i) * kSlb2EntrySize;
        if (offset + kSlb2EntrySize > data.size()) break;

        Slb2Entry entry;
        const u32 block = ld::read_u32(data, offset + 0x00);
        entry.size = ld::read_u32(data, offset + 0x04);
        entry.name = ld::read_cstr(data, offset + 0x10, kSlb2NameSize);
        entry.offset = block * kSlb2BlockSize;
        entry.data = ld::slice(data, entry.offset, entry.size);
        image.entries.push_back(std::move(entry));
    }

    image.valid = true;
    return image;
}

std::vector<u8> build_slb2(const std::vector<Slb2Entry>& entries, u32 version, u32 total_blocks,
                           u8 fill) {
    // Lay the payloads out on 512 byte block boundaries starting at block 1.
    std::vector<u32> blocks;
    blocks.reserve(entries.size());
    u32 block = 1;
    for (const Slb2Entry& entry : entries) {
        blocks.push_back(block);
        block += static_cast<u32>((entry.data.size() + kSlb2BlockSize - 1) / kSlb2BlockSize);
    }
    const u32 used_blocks = block;

    size_t payload_end = static_cast<size_t>(used_blocks) * kSlb2BlockSize;
    for (size_t i = 0; i < entries.size(); ++i) {
        payload_end = std::max(payload_end, static_cast<size_t>(blocks[i]) * kSlb2BlockSize +
                                                entries[i].data.size());
    }

    std::vector<u8> out(payload_end, fill);

    // Header block: the fixed 0x20 byte header and the entry records are zero
    // padded (the retail container stores zeros there and only fills the unused
    // tail of the block, "0x170..0x200", with the erased 0xFF pattern).
    const size_t table_end = kSlb2EntryBase + entries.size() * kSlb2EntrySize;
    if (table_end <= out.size()) std::memset(out.data(), 0, table_end);

    const u32 total = total_blocks != 0 ? total_blocks : used_blocks;
    auto put_u32 = [&out](size_t offset, u32 value) {
        out[offset + 0] = static_cast<u8>(value);
        out[offset + 1] = static_cast<u8>(value >> 8);
        out[offset + 2] = static_cast<u8>(value >> 16);
        out[offset + 3] = static_cast<u8>(value >> 24);
    };
    put_u32(0x00, kSlb2Magic);
    put_u32(0x04, version);
    put_u32(0x08, kSlb2HeaderSize);
    put_u32(0x0C, static_cast<u32>(entries.size()));
    put_u32(0x10, total);

    for (size_t i = 0; i < entries.size(); ++i) {
        const Slb2Entry& entry = entries[i];
        const size_t offset = kSlb2EntryBase + i * kSlb2EntrySize;
        // The records themselves are zero padded (only the unused tail of the
        // header block carries the erased 0xFF pattern, as the retail container
        // shows: "5365636f6e645f6c6f616465722e656e70000000...").
        std::memset(out.data() + offset, 0, kSlb2EntrySize);
        put_u32(offset + 0x00, blocks[i]);
        put_u32(offset + 0x04, static_cast<u32>(entry.data.size()));
        put_u32(offset + 0x08, 0);   // flags
        put_u32(offset + 0x0C, 0);   // reserved
        const size_t name_length = std::min<size_t>(entry.name.size(), kSlb2NameSize - 1);
        if (name_length > 0)
            std::memcpy(out.data() + offset + 0x10, entry.name.data(), name_length);
    }

    for (size_t i = 0; i < entries.size(); ++i) {
        const size_t offset = static_cast<size_t>(blocks[i]) * kSlb2BlockSize;
        if (entries[i].data.empty()) continue;
        std::memcpy(out.data() + offset, entries[i].data.data(), entries[i].data.size());
    }
    return out;
}

std::vector<u8> build_slb2(const std::vector<Slb2Entry>& entries) {
    return build_slb2(entries, 1, 0, kSlb2ErasedFill);
}

std::string describe_slb2(const Slb2Image& image) {
    if (!image.valid) return "SLB2 (invalid)";
    std::string text = format("SLB2 version=%u entries=%u", image.version,
                              static_cast<unsigned>(image.entries.size()));
    for (const Slb2Entry& entry : image.entries)
        text += format(" %s@0x%X(%u)", entry.name.c_str(), entry.offset,
                       static_cast<unsigned>(entry.size));
    return text;
}

}  // namespace zlb
