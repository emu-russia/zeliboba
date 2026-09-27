// zeliboba - eMMC image reconstruction for firmware 1.04.
//
// The console image we have is not an eMMC dump, so the user area is rebuilt out
// of the firmware itself:
//
//   * LBA 0            the console's own master block ("Sony Computer
//                      Entertainment Inc.", version 3) with the real 1.04
//                      partition table, see docs/EMMC.md for the evidence,
//   * the idstorage    reconstructed leaf index (personal per console),
//   * 8 MiB / 12 MiB   the two SLB2 copies (bls0 / bls1) carrying
//                      second_loader.enp + .enc, secure_kernel.enp + .enc,
//                      kernel_boot_loader.self, kprx_auth_sm.self and
//                      prog_rvk.srvk,
//   * 16 MiB / 32 MiB  os0 and its backup, FAT16 from out/fs/os0,
//   * 176 MiB          vs0, FAT16 from out/fs/vs0,
//   * the remaining    declared as in the real table but erased (we have no
//                      content for sa0/tm0/vd0/ud0/pd0/ur0).
//
// The SLB2 copies and the two boot partitions hold byte-identical containers.
#include "hw/emmc/emmc.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>

namespace zlb {
namespace emmc {

namespace {

namespace fs = std::filesystem;

u64 align_up(u64 value, u64 alignment) { return (value + alignment - 1) / alignment * alignment; }

// ---------------------------------------------------------------------------
// The 1.04 partition table (see docs/EMMC.md)
// ---------------------------------------------------------------------------

struct PartitionSpec {
    const char* name;
    VitaPartCode code;
    VitaPartType type;
    u64 offset_bytes;
    u64 size_bytes;
    u8 active;
    u32 flags;
    const char* payload;  ///< what the builder writes there
};

constexpr PartitionSpec kPartitions[] = {
    {"idstorage", VitaPartCode::IdStorage, VitaPartType::Raw, 0x00040000ull, 0x00080000ull, 0, 0x001F0F00,
     "reconstructed leaf index"},
    {"bls0", VitaPartCode::Slb2, VitaPartType::Raw, 0x00800000ull, 0x00400000ull, 0, 0x000F0F00,
     "SLB2 container"},
    {"bls1", VitaPartCode::Slb2, VitaPartType::Raw, 0x00C00000ull, 0x00400000ull, 1, 0x000F0F01,
     "SLB2 container (backup)"},
    {"os0_0", VitaPartCode::Os0, VitaPartType::Fat16, 0x01000000ull, 0x01000000ull, 0, 0x000F0F00,
     "FAT16 from fs/os0"},
    {"os0_1", VitaPartCode::Os0, VitaPartType::Fat16, 0x02000000ull, 0x01000000ull, 1, 0x000F0F01,
     "FAT16 from fs/os0 (backup)"},
    {"sa0", VitaPartCode::Sa0, VitaPartType::Fat16, 0x03000000ull, 0x06000000ull, 0, 0x000F0F00,
     "erased"},
    {"tm0", VitaPartCode::Tm0, VitaPartType::Fat16, 0x09000000ull, 0x02000000ull, 0, 0x000F0F00,
     "erased"},
    {"vs0_0", VitaPartCode::Vs0, VitaPartType::Fat16, 0x0B000000ull, 0x10000000ull, 0, 0x000F0F00,
     "FAT16 from fs/vs0"},
    {"vd0", VitaPartCode::Vd0, VitaPartType::Fat16, 0x1B000000ull, 0x02000000ull, 0, 0x000F0F00,
     "erased"},
    {"ud0", VitaPartCode::Ud0, VitaPartType::Fat16, 0x1D000000ull, 0x10000000ull, 0, 0x000F0F00,
     "erased"},
    {"pd0", VitaPartCode::Pd0, VitaPartType::ExFat, 0x2D000000ull, 0x13000000ull, 0, 0x000F0F00,
     "erased (exFAT on hardware)"},
    {"ur0", VitaPartCode::Ur0, VitaPartType::ExFat, 0x40000000ull, 0x0A340000ull, 0, 0x000F0F00,
     "erased (exFAT on hardware)"},
};

const PartitionSpec* find_partition(const char* name) {
    for (const auto& spec : kPartitions) {
        if (std::strcmp(spec.name, name) == 0) return &spec;
    }
    return nullptr;
}

std::vector<VitaPartition> make_partition_table(u64 limit_bytes, std::vector<std::string>* notes) {
    std::vector<VitaPartition> table;
    for (const auto& spec : kPartitions) {
        if (spec.offset_bytes + spec.size_bytes > limit_bytes) {
            if (notes) {
                notes->push_back(format("partition %s (0x%llX..0x%llX) does not fit in this %s image "
                                        "and is not declared",
                                        spec.name, static_cast<unsigned long long>(spec.offset_bytes),
                                        static_cast<unsigned long long>(spec.offset_bytes + spec.size_bytes),
                                        human_size(limit_bytes).c_str()));
            }
            continue;
        }
        VitaPartition partition;
        partition.offset_blocks = spec.offset_bytes / kBlockSize;
        partition.size_blocks = spec.size_bytes / kBlockSize;
        partition.code = spec.code;
        partition.type = spec.type;
        partition.active = spec.active;
        partition.flags = spec.flags;
        table.push_back(partition);
    }
    return table;
}

// ---------------------------------------------------------------------------
// Source payloads
// ---------------------------------------------------------------------------

/// The seven files of a 1.04 SLB2, in table order.
const char* const kSlb2Names[] = {
    "second_loader.enp", "second_loader.enc", "secure_kernel.enp", "secure_kernel.enc",
    "kernel_boot_loader.self", "kprx_auth_sm.self", "prog_rvk.srvk",
};

struct Slb2Source {
    std::vector<Slb2File> files;
    std::string description;
};

/// Preferred source: the authentic container inside the 1.04 PUP
/// (boot_slb2-00.pkg.seg02), sliced by its own entry table. Falls back to the
/// individual files in Out/SLB2 so the tool still works if the extracted
/// container is missing.
bool collect_slb2_source(const std::string& firmware_root, Slb2Source& out, std::string& message) {
    const std::string container_path = path_join(firmware_root, "PUP_dec/boot_slb2-00.pkg.seg02");
    if (file_exists(container_path)) {
        auto data = read_file(container_path);
        if (data) {
            Slb2View view;
            if (parse_slb2_container(*data, view) && view.valid) {
                std::vector<Slb2File> files;
                bool ok = true;
                std::string detail;
                for (const auto& entry : view.entries) {
                    if (entry.offset + entry.size > data->size()) {
                        ok = false;
                        detail = format("entry %s points past the container", entry.name.c_str());
                        break;
                    }
                    Slb2File file;
                    file.name = entry.name;
                    file.path = format("%s#%llu+%llu", container_path.c_str(),
                                       static_cast<unsigned long long>(entry.offset),
                                       static_cast<unsigned long long>(entry.size));
                    files.push_back(file);
                }
                if (ok && !files.empty()) {
                    out.files = files;
                    out.description = format("retail 1.04 SLB2 (%s, %u entries)",
                                             path_filename(container_path).c_str(), view.count);
                    message = out.description;
                    return true;
                }
                if (!detail.empty()) message = detail;
            } else {
                message = view.message;
            }
        }
    }

    // Fallback: assemble from Out/SLB2.
    const std::string slb2_dir = path_join(firmware_root, "SLB2");
    std::vector<Slb2File> files;
    for (const char* name : kSlb2Names) {
        const std::string path = path_join(slb2_dir, name);
        if (file_exists(path)) files.push_back(Slb2File{name, path});
    }
    if (files.size() < 5) {
        message = format("no SLB2 payloads found (looked at %s and %s)", container_path.c_str(),
                         slb2_dir.c_str());
        return false;
    }
    out.files = files;
    out.description = format("Out/SLB2 (%u entries)", static_cast<unsigned>(files.size()));
    message = out.description;
    return true;
}

/// Read a payload that may be a plain file or a "path#offset+size" slice.
struct SourceFile {
    std::string name;
    std::string path;
    u64 offset = 0;
    u64 size = 0;
    bool slice = false;
};

bool read_source_file(const SourceFile& source, std::vector<u8>& out) {
    auto data = read_file(source.path);
    if (!data) return false;
    if (!source.slice) {
        out = std::move(*data);
        return true;
    }
    if (source.offset + source.size > data->size()) return false;
    out.assign(data->begin() + static_cast<std::ptrdiff_t>(source.offset),
               data->begin() + static_cast<std::ptrdiff_t>(source.offset + source.size));
    return true;
}

SourceFile resolve_source(const std::string& reference) {
    SourceFile file;
    const size_t marker = reference.find('#');
    if (marker == std::string::npos) {
        file.path = reference;
        file.size = file_size(reference);
        return file;
    }
    file.path = reference.substr(0, marker);
    const std::string spec = reference.substr(marker + 1);
    const size_t plus = spec.find('+');
    if (plus == std::string::npos) {
        file.size = file_size(file.path);
        return file;
    }
    parse_u64(spec.substr(0, plus), file.offset);
    parse_u64(spec.substr(plus + 1), file.size);
    file.slice = true;
    return file;
}

/// First bytes of a payload, used to sanity check what we are about to write.
u32 payload_magic(const std::vector<u8>& data) {
    if (data.size() < 4) return 0;
    return read_le32(data.data());
}

/// Copy a genuine partition image (os0.bin / vs0.bin) into a partition slot,
/// zero filling the rest of the slot. Returns the number of bytes copied, or 0
/// on failure. The image is streamed, so a 256 MiB vs0 costs one 1 MiB buffer.
u64 copy_partition_image(EmmcCard& card, u64 partition_offset, u64 partition_bytes,
                         const std::string& image_path) {
    auto image = read_file(image_path);
    if (!image) return 0;
    if (image->size() > partition_bytes) {
        // A partition image can never be larger than the slot it belongs in.
        return 0;
    }
    if (!card.write_bytes(EmmcPartition::User, partition_offset, image->data(), image->size())) {
        return 0;
    }
    // Clear the tail so stale bytes from a previous build cannot leak through.
    const u64 remaining = partition_bytes - image->size();
    if (remaining == 0) return image->size();

    static const std::vector<u8> zeros(1u << 20, 0);
    u64 offset = partition_offset + image->size();
    u64 left = remaining;
    while (left > 0) {
        const size_t chunk = static_cast<size_t>(left > zeros.size() ? zeros.size() : left);
        if (!card.write_bytes(EmmcPartition::User, offset, zeros.data(), chunk)) return 0;
        offset += chunk;
        left -= chunk;
    }
    return image->size();
}

/// Size of the FAT16 volume inside a partition slot, taken from its BPB. The
/// console's os0 lives in a 16 MiB slot but the volume itself is 8 MiB (0x8000
/// sectors) on the real hardware, so the mount extent must come from the BPB.
u64 fat_volume_bytes(EmmcCard& card, u64 partition_offset) {
    std::vector<u8> boot(512, 0);
    if (!card.read_bytes(EmmcPartition::User, partition_offset, boot.data(), boot.size())) return 0;
    if (read_le16(boot.data() + 510) != 0xAA55) return 0;
    const u32 bytes_per_sector = read_le16(boot.data() + 11);
    if (bytes_per_sector == 0) return 0;
    const u32 total16 = read_le16(boot.data() + 19);
    const u32 total32 = read_le32(boot.data() + 32);
    const u32 total = total16 ? total16 : total32;
    return static_cast<u64>(total) * bytes_per_sector;
}

}  // namespace

// ---------------------------------------------------------------------------
// Master block
// ---------------------------------------------------------------------------

const char* to_string(VitaPartCode code) {
    switch (code) {
        case VitaPartCode::Empty: return "EMPTY";
        case VitaPartCode::IdStorage: return "IDSTORAGE";
        case VitaPartCode::Slb2: return "SLB2";
        case VitaPartCode::Os0: return "OS0";
        case VitaPartCode::Vs0: return "VS0";
        case VitaPartCode::Vd0: return "VD0";
        case VitaPartCode::Tm0: return "TM0";
        case VitaPartCode::Ur0: return "UR0";
        case VitaPartCode::Ux0: return "UX0";
        case VitaPartCode::Gro0: return "GRO0";
        case VitaPartCode::Grw0: return "GRW0";
        case VitaPartCode::Ud0: return "UD0";
        case VitaPartCode::Sa0: return "SA0";
        case VitaPartCode::UnknownMc: return "UNK_MC";
        case VitaPartCode::Pd0: return "PD0";
    }
    return "?";
}

const char* to_string(VitaPartType type) {
    switch (type) {
        case VitaPartType::Unknown0: return "UNKNOWN_0";
        case VitaPartType::Fat16: return "FAT16";
        case VitaPartType::ExFat: return "EXFAT";
        case VitaPartType::UnknownB: return "UNKNOWN_B";
        case VitaPartType::Raw: return "RAW";
    }
    return "?";
}

std::vector<u8> build_master_block(const std::vector<VitaPartition>& partitions, u32 total_blocks) {
    std::vector<u8> block(512, 0);
    std::memcpy(block.data(), kMasterMagic, 32);
    write_le32(block.data() + 0x20, kMasterVersion);
    write_le32(block.data() + kMasterSizeField, total_blocks);
    // 0x28 / 0x2C are zero on the real console; 0x30.. are constants seen in the
    // reference dump and are reproduced verbatim.
    write_le32(block.data() + 0x30, 0x0000606A);
    write_le32(block.data() + 0x34, 0x00000069);
    write_le32(block.data() + 0x38, 0x00006000);
    write_le32(block.data() + 0x3C, 0x00004000);
    write_le32(block.data() + 0x40, 0x00006000);
    write_le32(block.data() + 0x44, 0x00010000);

    for (size_t i = 0; i < partitions.size() && i < kPartitionTableSlots; ++i) {
        const size_t offset = kPartitionTableOffset + i * kPartitionRecordSize;
        const VitaPartition& partition = partitions[i];
        write_le32(block.data() + offset + 0, static_cast<u32>(partition.offset_blocks));
        write_le32(block.data() + offset + 4, static_cast<u32>(partition.size_blocks));
        block[offset + 8] = static_cast<u8>(partition.code);
        block[offset + 9] = static_cast<u8>(partition.type);
        block[offset + 10] = partition.active;
        write_le32(block.data() + offset + 11, partition.flags);
        block[offset + 15] = 0x0F;
    }
    write_le16(block.data() + 0x1FE, 0xAA55);
    return block;
}

bool parse_master_block(const u8* data, size_t length, std::vector<VitaPartition>& partitions,
                        u32& total_blocks, std::string& message) {
    partitions.clear();
    total_blocks = 0;
    if (!data || length < 512) {
        message = "master block is smaller than 512 bytes";
        return false;
    }
    if (std::memcmp(data, kMasterMagic, 32) != 0) {
        message = format("bad master block magic (\"%s\")",
                         std::string(reinterpret_cast<const char*>(data), 31).c_str());
        return false;
    }
    const u32 version = read_le32(data + 0x20);
    if (version != kMasterVersion) {
        message = format("master block version %u (expected %u)", version, kMasterVersion);
        return false;
    }
    if (read_le16(data + 0x1FE) != 0xAA55) {
        message = "master block has no 0xAA55 signature";
        return false;
    }
    total_blocks = read_le32(data + kMasterSizeField);
    for (u32 i = 0; i < kPartitionTableSlots; ++i) {
        const size_t offset = kPartitionTableOffset + i * kPartitionRecordSize;
        VitaPartition partition;
        partition.offset_blocks = read_le32(data + offset + 0);
        partition.size_blocks = read_le32(data + offset + 4);
        partition.code = static_cast<VitaPartCode>(data[offset + 8]);
        partition.type = static_cast<VitaPartType>(data[offset + 9]);
        partition.active = data[offset + 10];
        partition.flags = read_le32(data + offset + 11);
        if (partition.offset_blocks == 0 && partition.size_blocks == 0) break;
        partitions.push_back(partition);
    }
    message = format("version %u, total %u blocks (%s), %u partition records", version, total_blocks,
                     human_size(static_cast<u64>(total_blocks) * kBlockSize).c_str(),
                     static_cast<unsigned>(partitions.size()));
    return true;
}

// ---------------------------------------------------------------------------
// SLB2 container
// ---------------------------------------------------------------------------

std::vector<u8> build_slb2_container(const std::vector<Slb2File>& files, bool verbose) {
    std::vector<u8> container;
    // The retail container fills the unused part of the entry table with 0xFF
    // (verified byte for byte against boot_slb2-00.pkg.seg02), so do the same.
    std::vector<u8> table(kSlb2TableSize, 0xFF);
    // ... but the 12 reserved bytes of the header itself (0x14..0x1F) are zero in
    // both the retail container and the two eMMC dumps. They used to be left at
    // the 0xFF fill, which made the assembled header differ from the console's in
    // twelve bytes - and the CMeP second loader validates the container header it
    // reads from the card (it decrypts/authenticates the first 512 bytes before
    // parsing the entry table), so the difference is not cosmetic.
    std::memset(table.data() + 0x14, 0, kSlb2HeaderSize - 0x14);
    write_le32(table.data() + 0, kSlb2Magic);
    write_le32(table.data() + 4, kSlb2Version);
    write_le32(table.data() + 8, kSlb2TableSize);
    write_le32(table.data() + 12, static_cast<u32>(files.size()));
    write_le32(table.data() + 16, kSlb2DataHint);

    u64 cursor = kSlb2TableSize;
    u32 index = 0;
    for (const auto& file : files) {
        const SourceFile source = resolve_source(file.path);
        std::vector<u8> payload;
        if (!read_source_file(source, payload)) {
            if (verbose) std::printf("  ! cannot read SLB2 payload %s\n", file.path.c_str());
            return {};
        }
        cursor = align_up(cursor, kBlockSize);
        const u64 first_block = cursor / kBlockSize;
        const size_t entry = kSlb2HeaderSize + static_cast<size_t>(index) * kSlb2EntryStride;
        write_le32(table.data() + entry + 0x00, static_cast<u32>(first_block));
        write_le32(table.data() + entry + 0x04, static_cast<u32>(payload.size()));
        write_le32(table.data() + entry + 0x08, 0);
        write_le32(table.data() + entry + 0x0C, 0);
        // The 32 byte name field is NUL padded even though the rest of the unused
        // table area is 0xFF: the boot chain's parser reads it as a C string and
        // would otherwise see "second_loader.enp<0xFF...>" and reject the entry.
        std::memset(table.data() + entry + 0x10, 0, 32);
        std::memcpy(table.data() + entry + 0x10, file.name.c_str(),
                    std::min<size_t>(32, file.name.size()));
        if (verbose) {
            std::printf("  SLB2 [%u] %-24s block 0x%-6llX size 0x%-8llX\n", index, file.name.c_str(),
                        static_cast<unsigned long long>(first_block),
                        static_cast<unsigned long long>(payload.size()));
        }
        ++index;
        cursor += payload.size();
    }
    cursor = align_up(cursor, kBlockSize);

    container = std::move(table);
    container.resize(static_cast<size_t>(cursor), 0);

    cursor = kSlb2TableSize;
    for (const auto& file : files) {
        const SourceFile source = resolve_source(file.path);
        std::vector<u8> payload;
        if (!read_source_file(source, payload)) return {};
        cursor = align_up(cursor, kBlockSize);
        std::memcpy(container.data() + cursor, payload.data(), payload.size());
        cursor += payload.size();
    }
    return container;
}

bool parse_slb2_container(const std::vector<u8>& data, Slb2View& out, bool require_payload) {
    out = Slb2View{};
    if (data.size() < kSlb2HeaderSize) {
        out.message = "container is smaller than the SLB2 header";
        return false;
    }
    if (read_le32(data.data()) != kSlb2Magic) {
        out.message = format("bad magic 0x%08X (expected 0x%08X \"SLB2\")", read_le32(data.data()),
                             kSlb2Magic);
        return false;
    }
    out.version = read_le32(data.data() + 4);
    out.table_size = read_le32(data.data() + 8);
    out.count = read_le32(data.data() + 12);
    out.data_hint = read_le32(data.data() + 16);
    if (out.count > 64) {
        out.message = format("implausible entry count %u", out.count);
        return false;
    }
    if (out.version != kSlb2Version) {
        out.message = format("unexpected version %u", out.version);
        return false;
    }

    u64 end = kSlb2HeaderSize + static_cast<u64>(out.count) * kSlb2EntryStride;
    for (u32 i = 0; i < out.count; ++i) {
        const size_t entry = kSlb2HeaderSize + static_cast<size_t>(i) * kSlb2EntryStride;
        if (entry + kSlb2EntryStride > data.size()) {
            out.message = format("entry %u is outside the container", i);
            return false;
        }
        Slb2EntryView view;
        view.first_block = read_le32(data.data() + entry + 0x00);
        view.size = read_le32(data.data() + entry + 0x04);
        view.flags = read_le32(data.data() + entry + 0x08);
        view.offset = static_cast<u64>(view.first_block) * kBlockSize;
        const char* name = reinterpret_cast<const char*>(data.data() + entry + 0x10);
        size_t name_length = 0;
        // The retail container NUL terminates names inside a table whose unused
        // space is filled with 0xFF, so stop at either byte.
        while (name_length < 32 && name[name_length] != '\0' &&
               static_cast<u8>(name[name_length]) != 0xFF) {
            ++name_length;
        }
        view.name.assign(name, name_length);
        if (view.offset + view.size > data.size() && require_payload) {
            out.message = format("entry %u (%s) is outside the container (offset 0x%llX size 0x%llX)",
                                 i, view.name.c_str(), static_cast<unsigned long long>(view.offset),
                                 static_cast<unsigned long long>(view.size));
            return false;
        }
        end = std::max(end, view.offset + view.size);
        out.entries.push_back(view);
    }
    out.total_bytes = end;
    out.valid = true;
    out.message = format("SLB2 v%u, %u entries, %s", out.version, out.count,
                         human_size(out.total_bytes).c_str());
    return true;
}

// ---------------------------------------------------------------------------
// idstorage
// ---------------------------------------------------------------------------

std::vector<u8> build_idstorage_image() {
    constexpr u32 kLeafCount = 256;
    constexpr u32 kLeafSize = 512;
    std::vector<u8> image(static_cast<size_t>(kLeafCount) * kLeafSize, 0xFF);

    // Leaf 0 is what the boot chain actually reads: the CMeP second loader reads
    // block 0x200 (byte 0x40000) with CMD17 and 0x46632 refuses to go on unless
    // the first 64 bytes of the block are 0xFFF5 repeated.  The reference dump
    // (dumps/emmcdump.zip, byte 0x40000) carries exactly that layout, so leaf 0
    // reproduces it: a 64 byte 0xFFF5 marker, the u16 leaf index 0x0000..0x007F,
    // 0xFFFF filler up to +0x180 and the short leaf list the dump shows there.
    u8* leaf0 = image.data();
    for (u32 i = 0; i < 32; ++i) write_le16(leaf0 + i * 2, 0xFFF5);
    for (u32 i = 0; i < 128; ++i) write_le16(leaf0 + 0x40 + i * 2, static_cast<u16>(i));
    for (u32 offset = 0x140; offset < 0x180; offset += 2) write_le16(leaf0 + offset, 0xFFFF);
    static constexpr u16 kLeafList[] = {0x0080, 0x0100, 0x0102, 0x0103, 0x0110,
                                        0x0111, 0x0112, 0x0113, 0x0114, 0x0115};
    u32 cursor = 0x180;
    for (u16 id : kLeafList) {
        write_le16(leaf0 + cursor, id);
        cursor += 2;
    }
    for (; cursor < kLeafSize; cursor += 2) write_le16(leaf0 + cursor, 0xFFFF);

    // Leaves.  The mapping table (slot 0) points at the leaves the console has:
    // slots 32..159 hold leaf IDs 0x0000..0x007F (the idps certificates, all
    // console unique and not in any dump) and slots 192..201 the IDs the dump
    // lists.  Only the SMI leaf (ID 0x80, slot 192) is needed by the boot chain:
    // the CMeP second loader searches the mapping table for 0x0080 (0x4678C) and
    // then reads that slot (block 0x2C0) and checks the structure documented for
    // it - "SMI\0", version 1 and a plaintext area that must be zeroes.  The
    // payload (0x80..0xFF) and signature (0x100..0x1FF) areas are encrypted with a
    // per-unit key (keyslot 0x213) that no dump contains, so they are left as
    // zeroes here; see docs/KBL.md, round 21.
    constexpr u32 kSmiSlot = 192;
    u8* smi = image.data() + static_cast<size_t>(kSmiSlot) * kLeafSize;
    smi[0] = 'S';
    smi[1] = 'M';
    smi[2] = 'I';
    smi[3] = 0x00;
    write_le32(smi + 0x04, 1);  // version
    write_le32(smi + 0x08, 0);  // minfw_plaintext (ignored by the second loader)
    std::memset(smi + 0x0C, 0, 0x80 - 0x0C);   // must_be_zero, checked
    std::memset(smi + 0x80, 0, 0x100 - 0x80);  // payload area (encrypted on hardware)
    std::memset(smi + 0x100, 0xFF, 0x100);     // signature area (encrypted on hardware)
    return image;
}

}  // namespace emmc

// ---------------------------------------------------------------------------
// build_emmc_image
// ---------------------------------------------------------------------------

EmmcImagePlan build_emmc_image(const std::string& firmware_root, const std::string& output_path,
                               bool verbose, u64 size_bytes, bool from_tree) {
    namespace fs = std::filesystem;
    using namespace emmc;

    EmmcImagePlan plan;

    const fs::path root(firmware_root);
    if (!fs::is_directory(root)) {
        plan.message = format("firmware root '%s' is not a directory", firmware_root.c_str());
        return plan;
    }

    const u64 user_area = size_bytes ? size_bytes : kUserAreaBytes;
    if (user_area < kMinImageBytes) {
        plan.message = format("image size %s is too small; at least %s is required to hold the master "
                              "block, idstorage, both SLB2 copies and the os0/vs0 partitions",
                              human_size(user_area).c_str(), human_size(kMinImageBytes).c_str());
        return plan;
    }
    plan.total_size = user_area + kBootAreaSize + kRpmbSize;

    // ---- gather the sources ---------------------------------------------
    Slb2Source slb2_source;
    std::string slb2_message;
    if (!collect_slb2_source(firmware_root, slb2_source, slb2_message)) {
        plan.message = format("cannot assemble the SLB2 boot stage: %s", slb2_message.c_str());
        return plan;
    }
    plan.notes.push_back(format("SLB2 source: %s", slb2_source.description.c_str()));

    const std::string fs_os0 = path_join(firmware_root, "fs/os0");
    const std::string fs_vs0 = path_join(firmware_root, "fs/vs0");
    if (!fs::is_directory(fs_os0) || !fs::is_directory(fs_vs0)) {
        plan.message = format("the extracted filesystem tree is incomplete: need %s and %s",
                              fs_os0.c_str(), fs_vs0.c_str());
        return plan;
    }

    // The genuine 1.04 partition images are the geometry template *and* the
    // default payload.
    FatGeometry os0_geometry = default_fat_geometry();
    FatGeometry vs0_geometry = default_fat_geometry();
    const std::string os0_image = path_join(firmware_root, "PUP_dec/os0.bin");
    const std::string vs0_image = path_join(firmware_root, "PUP_dec/vs0.bin");
    {
        auto data = read_file(os0_image);
        if (data && read_fat_geometry(*data, os0_geometry)) {
            plan.notes.push_back(format("os0 geometry from %s: %s", path_filename(os0_image).c_str(),
                                        os0_geometry.message.c_str()));
        } else {
            plan.notes.push_back("os0 geometry: built-in 1.04 default (PUP_dec/os0.bin unavailable)");
        }
        data = read_file(vs0_image);
        if (data && read_fat_geometry(*data, vs0_geometry)) {
            plan.notes.push_back(format("vs0 geometry from %s: %s", path_filename(vs0_image).c_str(),
                                        vs0_geometry.message.c_str()));
        } else {
            plan.notes.push_back("vs0 geometry: built-in 1.04 default (PUP_dec/vs0.bin unavailable)");
        }
    }
    if (!from_tree && (!file_exists(os0_image) || !file_exists(vs0_image))) {
        plan.message = format("the real 1.04 partition images are required for the default mode: "
                              "need %s and %s (use --from-tree to synthesise instead)",
                              os0_image.c_str(), vs0_image.c_str());
        return plan;
    }
    vs0_geometry.volume_id = os0_geometry.volume_id ? os0_geometry.volume_id : vs0_geometry.volume_id;
    os0_geometry.volume_label = "NO NAME";
    vs0_geometry.volume_label = "NO NAME";

    // ---- partition table -------------------------------------------------
    std::vector<VitaPartition> partition_table = make_partition_table(user_area, &plan.notes);
    if (partition_table.empty()) {
        plan.message = "no partition fits this image size";
        return plan;
    }

    // ---- open the image --------------------------------------------------
    EmmcCard card;
    if (!card.attach(output_path, true, plan.total_size)) {
        plan.message = format("cannot create '%s'", output_path.c_str());
        return plan;
    }
    if (verbose) {
        std::printf("image    : %s (%s user area + %s boot + %s RPMB)\n", output_path.c_str(),
                    human_size(user_area).c_str(), human_size(kBootAreaSize).c_str(),
                    human_size(kRpmbSize).c_str());
    }

    auto add_entry = [&](const std::string& name, EmmcPartition partition, u64 offset, u64 size,
                         const std::string& source) {
        EmmcLayoutEntry entry;
        entry.name = name;
        entry.partition = partition;
        entry.offset = offset;
        entry.size = size;
        entry.source = source;
        plan.entries.push_back(entry);
    };

    // 1) master block
    const std::vector<u8> master =
        build_master_block(partition_table, static_cast<u32>(kUserAreaBlocks));
    if (!card.write_bytes(EmmcPartition::User, 0, master.data(), master.size())) {
        plan.message = "failed to write the master block";
        return plan;
    }
    add_entry("master_block", EmmcPartition::User, 0, master.size(),
              "Sony master block + 1.04 partition table (see docs/EMMC.md)");

    // 2) SLB2 container, written to both boot partitions and both bls copies.
    const std::vector<u8> slb2 = build_slb2_container(slb2_source.files, verbose);
    if (slb2.empty()) {
        plan.message = "failed to assemble the SLB2 container";
        return plan;
    }
    Slb2View slb2_view;
    if (!parse_slb2_container(slb2, slb2_view)) {
        plan.message = format("assembled SLB2 container is invalid: %s", slb2_view.message.c_str());
        return plan;
    }
    plan.notes.push_back(format("SLB2 container: %s (%s, %u entries)", slb2_view.message.c_str(),
                                human_size(slb2.size()).c_str(), slb2_view.count));

    for (EmmcPartition boot : {EmmcPartition::Boot0, EmmcPartition::Boot1}) {
        if (!card.write_bytes(boot, 0, slb2.data(), slb2.size())) {
            plan.message = "failed to write a boot partition";
            return plan;
        }
        add_entry(boot == EmmcPartition::Boot0 ? "boot0/slb2" : "boot1/slb2", boot, 0, slb2.size(),
                  slb2_source.description);
    }

    // 3) user area: idstorage, bls0/bls1, os0 (x2), vs0.
    const std::vector<u8> idstorage = build_idstorage_image();
    const PartitionSpec* idstorage_spec = find_partition("idstorage");
    if (idstorage_spec &&
        idstorage_spec->offset_bytes + idstorage_spec->size_bytes <= user_area) {
        std::vector<u8> padded(idstorage_spec->size_bytes, 0xFF);
        std::memcpy(padded.data(), idstorage.data(), idstorage.size());
        if (!card.write_bytes(EmmcPartition::User, idstorage_spec->offset_bytes, padded.data(),
                              padded.size())) {
            plan.message = "failed to write the idstorage partition";
            return plan;
        }
        add_entry("idstorage", EmmcPartition::User, idstorage_spec->offset_bytes,
                  idstorage_spec->size_bytes,
                  format("reconstructed leaf index (%u leaves, %s index table)",
                         static_cast<unsigned>(idstorage.size() / 512),
                         human_size(idstorage.size()).c_str()));
    }

    for (const char* name : {"bls0", "bls1"}) {
        const PartitionSpec* spec = find_partition(name);
        if (!spec || spec->offset_bytes + spec->size_bytes > user_area) continue;
        std::vector<u8> region(spec->size_bytes, 0xFF);
        std::memcpy(region.data(), slb2.data(), std::min<u64>(slb2.size(), spec->size_bytes));
        if (!card.write_bytes(EmmcPartition::User, spec->offset_bytes, region.data(), region.size())) {
            plan.message = format("failed to write partition %s", name);
            return plan;
        }
        add_entry(name, EmmcPartition::User, spec->offset_bytes, spec->size_bytes,
                  format("SLB2 container (%s)", slb2_source.description.c_str()));
    }

    struct FatTarget {
        const char* partition;
        const char* tree;    ///< extracted fs/ subtree (used by the from-tree mode)
        const char* image;   ///< real 1.04 partition image (used by the default mode)
        FatGeometry geometry;
    };
    const FatTarget targets[] = {
        {"os0_0", fs_os0.c_str(), os0_image.c_str(), os0_geometry},
        {"os0_1", fs_os0.c_str(), os0_image.c_str(), os0_geometry},
        {"vs0_0", fs_vs0.c_str(), vs0_image.c_str(), vs0_geometry},
    };

    for (const auto& target : targets) {
        const PartitionSpec* spec = find_partition(target.partition);
        if (!spec || spec->offset_bytes + spec->size_bytes > user_area) {
            plan.notes.push_back(format("%s does not fit in this image and was skipped",
                                        target.partition));
            continue;
        }

        FatVolumeStats stats;
        std::string source;
        if (!from_tree) {
            // Default: lay the genuine 1.04 partition image down verbatim. It is
            // what the console actually has there and every file it contains is
            // byte identical to the extracted tree (verified, see docs/EMMC.md).
            const u64 written =
                copy_partition_image(card, spec->offset_bytes, spec->size_bytes, target.image);
            if (written == 0) {
                plan.message = format("cannot lay down the partition image '%s' for %s",
                                      target.image, target.partition);
                return plan;
            }
            if (verbose) {
                std::printf("  %s <- %s (%s, tail zero filled to %s)\n", target.partition,
                            path_filename(target.image).c_str(), human_size(written).c_str(),
                            human_size(spec->size_bytes).c_str());
            }
            if (!mount_fat_volume(card, EmmcPartition::User, spec->offset_bytes / kBlockSize,
                                  spec->size_bytes / kBlockSize, stats)) {
                plan.message = format("the partition image written to %s is not mountable: %s",
                                      target.partition, stats.message.c_str());
                return plan;
            }
            source = format("%s -> %s verbatim (%s of %s, tail zero filled)",
                            path_filename(target.image).c_str(), stats.volume_label.c_str(),
                            human_size(written).c_str(), human_size(spec->size_bytes).c_str());
        } else {
            if (!build_fat_volume(card, EmmcPartition::User, spec->offset_bytes / kBlockSize,
                                  spec->size_bytes / kBlockSize, target.tree, target.geometry,
                                  verbose, stats)) {
                plan.message = format("FAT16 build for %s failed: %s", target.partition,
                                      stats.message.c_str());
                return plan;
            }
            source = format("%s -> %s (%llu files, %llu dirs, %s data)", target.tree,
                            stats.volume_label.c_str(), static_cast<unsigned long long>(stats.files),
                            static_cast<unsigned long long>(stats.directories),
                            human_size(stats.data_bytes).c_str());
        }

        add_entry(target.partition, EmmcPartition::User, spec->offset_bytes, spec->size_bytes, source);
        plan.notes.push_back(format("%s: %s, %u sectors/FAT, %u clusters, %llu used (%s), "
                                    "%u free, root %u/%u entries",
                                    target.partition, stats.template_note.c_str(),
                                    stats.sectors_per_fat, stats.cluster_count,
                                    static_cast<unsigned long long>(stats.clusters_used),
                                    human_size(stats.bytes_used).c_str(), stats.free_clusters,
                                    stats.root_entries_used, stats.root_entry_count));
    }

    // 4) the remaining declared partitions exist only as "don't care" regions.
    for (const auto& spec : kPartitions) {
        if (std::strcmp(spec.payload, "erased") == 0 ||
            std::strcmp(spec.payload, "erased (exFAT on hardware)") == 0) {
            if (spec.offset_bytes + spec.size_bytes > user_area) continue;
            add_entry(spec.name, EmmcPartition::User, spec.offset_bytes, spec.size_bytes,
                      spec.payload);
        }
    }

    if (!card.flush()) {
        plan.message = "flush failed";
        return plan;
    }

    // 5) RPMB is left blank but recorded.
    add_entry("rpmb", EmmcPartition::Rpmb, 0, kRpmbSize, "blank (no RPMB content available)");

    plan.ok = true;
    plan.message = format("wrote %s to %s (%u regions)", human_size(plan.total_size).c_str(),
                          output_path.c_str(), static_cast<unsigned>(plan.entries.size()));
    return plan;
}

// ---------------------------------------------------------------------------
// inspect_emmc_image
// ---------------------------------------------------------------------------

EmmcImagePlan inspect_emmc_image(const std::string& path) {
    using namespace emmc;

    EmmcImagePlan plan;
    EmmcCard card;
    if (!card.attach(path, false)) {
        plan.message = format("cannot open '%s'", path.c_str());
        return plan;
    }
    plan.total_size = card.capacity_bytes() + kBootAreaSize + kRpmbSize;    plan.notes.push_back(format("card: %s user area, %llu x %u byte blocks, boot %s, RPMB %s",
                                human_size(card.capacity_bytes()).c_str(),
                                static_cast<unsigned long long>(card.block_count()),
                                card.block_size(), human_size(kBootAreaSize).c_str(),
                                human_size(kRpmbSize).c_str()));

    std::vector<u8> master(512, 0);
    if (!card.read_bytes(EmmcPartition::User, 0, master.data(), master.size())) {
        plan.message = "cannot read LBA 0";
        return plan;
    }

    std::vector<VitaPartition> partitions;
    u32 total_blocks = 0;
    std::string master_message;
    if (parse_master_block(master.data(), master.size(), partitions, total_blocks, master_message)) {
        plan.notes.push_back(format("master block: %s", master_message.c_str()));
        for (size_t i = 0; i < partitions.size(); ++i) {
            const VitaPartition& partition = partitions[i];
            const u64 offset = partition.offset_blocks * kBlockSize;
            const u64 size = partition.size_blocks * kBlockSize;
            EmmcLayoutEntry entry;
            entry.name = format("slot%02u/%s", static_cast<unsigned>(i), to_string(partition.code));
            entry.partition = EmmcPartition::User;
            entry.offset = offset;
            entry.size = size;
            entry.source = format("type=%s active=%u flags=0x%08X", to_string(partition.type),
                                  partition.active, partition.flags);
            plan.entries.push_back(entry);
        }
    } else {
        plan.notes.push_back(format("master block: NOT FOUND (%s)", master_message.c_str()));
    }

    // Mount whatever FAT16 volumes we recognise. The partition is bigger than the
    // volume it holds (16 MiB slot, 8 MiB FAT16 image), so the size is read from
    // the BPB rather than from the partition table.
    auto mount_named = [&](const char* name, const char* source) {
        const PartitionSpec* spec = find_partition(name);
        if (!spec) return;
        if (spec->offset_bytes + spec->size_bytes > card.capacity_bytes()) return;
        const u64 volume_bytes = fat_volume_bytes(card, spec->offset_bytes);
        FatVolumeStats stats;
        if (mount_fat_volume(card, EmmcPartition::User, spec->offset_bytes / kBlockSize,
                             volume_bytes / kBlockSize, stats)) {
            plan.notes.push_back(format("%s: FAT16 %u B/cluster, %u clusters, %llu files, %llu dirs, "
                                        "%s data, %u/%u clusters used (volume %s in a %s slot)",
                                        name, stats.cluster_bytes, stats.cluster_count,
                                        static_cast<unsigned long long>(stats.files),
                                        static_cast<unsigned long long>(stats.directories),
                                        human_size(stats.data_bytes).c_str(),
                                        static_cast<unsigned>(stats.cluster_count - stats.free_clusters),
                                        stats.cluster_count, human_size(volume_bytes).c_str(),
                                        human_size(spec->size_bytes).c_str()));
            for (auto& entry : plan.entries) {
                if (entry.name == name) {
                    entry.source = format("FAT16 label=%s files=%llu dirs=%llu data=%s",
                                          stats.volume_label.c_str(),
                                          static_cast<unsigned long long>(stats.files),
                                          static_cast<unsigned long long>(stats.directories),
                                          human_size(stats.data_bytes).c_str());
                }
            }
        } else if (source) {
            plan.notes.push_back(format("%s: not a mountable FAT16 volume (%s)", name,
                                        stats.message.c_str()));
        }
    };
    mount_named("os0_0", "fs/os0");
    mount_named("vs0_0", "fs/vs0");

    // SLB2 containers in the boot partitions and the two bls copies. The table
    // lives in the first 0x200 bytes, so a couple of KiB is enough to read it.
    auto report_slb2 = [&](const char* label, EmmcPartition partition, u64 offset) {
        std::vector<u8> data(4 * KB, 0);
        if (!card.read_bytes(partition, offset, data.data(), data.size())) return;
        Slb2View view;
        if (parse_slb2_container(data, view, false)) {
            plan.notes.push_back(format("%s: %s", label, view.message.c_str()));
        } else {
            plan.notes.push_back(format("%s: not an SLB2 container (%s)", label, view.message.c_str()));
        }
    };
    report_slb2("boot0", EmmcPartition::Boot0, 0);
    report_slb2("boot1", EmmcPartition::Boot1, 0);
    for (const char* name : {"bls0", "bls1"}) {
        const PartitionSpec* spec = find_partition(name);
        if (!spec || spec->offset_bytes >= card.capacity_bytes()) continue;
        report_slb2(name, EmmcPartition::User, spec->offset_bytes);
    }

    plan.ok = true;
    plan.message = format("inspected %s", path.c_str());
    return plan;
}

// ---------------------------------------------------------------------------
// verify_emmc_image
// ---------------------------------------------------------------------------

EmmcVerifyReport verify_emmc_image(const std::string& path, const std::string& firmware_root) {
    namespace fs = std::filesystem;
    using namespace emmc;

    EmmcVerifyReport report;
    EmmcCard card;
    if (!card.attach(path, false)) {
        report.message = format("cannot open '%s'", path.c_str());
        return report;
    }

    std::vector<u8> master(512, 0);
    if (!card.read_bytes(EmmcPartition::User, 0, master.data(), master.size())) {
        report.message = "cannot read LBA 0";
        return report;
    }
    std::vector<VitaPartition> partitions;
    u32 total_blocks = 0;
    if (!parse_master_block(master.data(), master.size(), partitions, total_blocks,
                            report.master_block_message)) {
        report.problems.push_back(format("master block: %s", report.master_block_message.c_str()));
    }

    // STAGE 1: the SLB2 boot containers.
    u32 slb2_seen = 0;
    auto check_slb2 = [&](const char* label, EmmcPartition partition, u64 offset, u64 size,
                          std::string* detail) {
        std::vector<u8> data(static_cast<size_t>(std::min<u64>(size, 1u << 20)), 0);
        if (!card.read_bytes(partition, offset, data.data(), data.size())) {
            report.problems.push_back(format("%s: cannot read", label));
            return false;
        }
        Slb2View view;
        if (!parse_slb2_container(data, view, false)) {
            report.problems.push_back(format("%s: %s", label, view.message.c_str()));
            return false;
        }
        // Every entry must start with a magic we recognise:
        //   second_loader.*  = MeP image header 0x64B2C8E5
        //   *.self / *.srvk  = SCE header "SCE\0" (0x00454353)
        std::string entries;
        for (const auto& entry : view.entries) {
            std::vector<u8> head(4, 0);
            if (!card.read_bytes(partition, offset + entry.offset, head.data(), head.size())) {
                report.problems.push_back(format("%s: cannot read %s", label, entry.name.c_str()));
                return false;
            }
            const u32 magic = read_le32(head.data());
            const bool is_mep = magic == 0x64B2C8E5u;
            const bool is_sce = magic == 0x00454353u;
            if (!is_mep && !is_sce) {
                report.problems.push_back(format("%s: %s has magic 0x%08X (expected 0x64B2C8E5 or "
                                                 "0x00454353)",
                                                 label, entry.name.c_str(), magic));
                return false;
            }
            if (!entries.empty()) entries += ", ";
            entries += format("%s@0x%llX+0x%llX(%s)", entry.name.c_str(),
                              static_cast<unsigned long long>(entry.offset),
                              static_cast<unsigned long long>(entry.size), is_mep ? "MeP" : "SCE");
        }
        if (detail) {
            *detail = format("%u entries: %s", view.count, entries.c_str());
        }
        ++slb2_seen;
        return true;
    };

    std::string boot0_detail;
    std::string bls0_detail;
    bool slb2_ok = check_slb2("boot0", EmmcPartition::Boot0, 0, kBootPartitionSize, &boot0_detail);
    slb2_ok = check_slb2("boot1", EmmcPartition::Boot1, 0, kBootPartitionSize, nullptr) && slb2_ok;
    const PartitionSpec* bls0 = find_partition("bls0");
    const PartitionSpec* bls1 = find_partition("bls1");
    if (bls0 && bls0->offset_bytes < card.capacity_bytes()) {
        slb2_ok = check_slb2("bls0", EmmcPartition::User, bls0->offset_bytes, bls0->size_bytes,
                             &bls0_detail) &&
                  slb2_ok;
    } else {
        report.problems.push_back("bls0 does not fit in this image");
        slb2_ok = false;
    }
    if (bls1 && bls1->offset_bytes < card.capacity_bytes()) {
        slb2_ok = check_slb2("bls1", EmmcPartition::User, bls1->offset_bytes, bls1->size_bytes,
                             nullptr) &&
                  slb2_ok;
    }
    report.slb2_ok = slb2_ok;
    report.slb2_message = boot0_detail.empty() ? "no SLB2 container found" : boot0_detail;
    report.slb2_entries = slb2_seen;

    // STAGE 2: the FAT16 volumes, mounted back and compared file by file.
    struct FatCheck {
        const char* name;
        const char* tree;
    };
    const FatCheck checks[] = {{"os0_0", "fs/os0"}, {"vs0_0", "fs/vs0"}};
    for (const auto& check : checks) {
        const PartitionSpec* spec = find_partition(check.name);
        if (!spec || spec->offset_bytes + spec->size_bytes > card.capacity_bytes()) {
            report.problems.push_back(format("%s does not fit in this image", check.name));
            continue;
        }
        EmmcPartitionReport entry;
        entry.name = check.name;
        entry.payload = format("FAT16 from %s", check.tree);
        entry.offset = spec->offset_bytes;
        entry.size = spec->size_bytes;
        entry.present = true;

        const u64 volume_bytes = fat_volume_bytes(card, spec->offset_bytes);
        if (volume_bytes == 0 || volume_bytes > spec->size_bytes) {
            entry.note = "no readable FAT16 BPB";
            report.problems.push_back(format("%s: cannot read the FAT16 BPB", check.name));
            report.partitions.push_back(format("%-8s %s", entry.name.c_str(), entry.note.c_str()));
            continue;
        }
        const u64 volume_blocks = volume_bytes / kBlockSize;

        if (!mount_fat_volume(card, EmmcPartition::User, spec->offset_bytes / kBlockSize,
                              volume_blocks, entry.fat)) {
            entry.note = format("mount failed: %s", entry.fat.message.c_str());
            report.problems.push_back(format("%s: %s", check.name, entry.fat.message.c_str()));
            report.partitions.push_back(format("%-8s %s", entry.name.c_str(), entry.note.c_str()));
            continue;
        }
        entry.mounted = true;
        entry.note = format("mounted: %u B/cluster, %u clusters, %llu files, %llu dirs, label \"%s\"",
                            entry.fat.cluster_bytes, entry.fat.cluster_count,
                            static_cast<unsigned long long>(entry.fat.files),
                            static_cast<unsigned long long>(entry.fat.directories),
                            entry.fat.volume_label.c_str());

        const std::string tree = path_join(firmware_root, check.tree);
        if (fs::is_directory(tree)) {
            u64 checked = 0;
            u64 mismatches = 0;
            std::vector<std::string> problems;
            verify_fat_volume(card, EmmcPartition::User, spec->offset_bytes / kBlockSize,
                              volume_blocks, tree, checked, mismatches, problems);
            report.files_checked += checked;
            report.file_mismatches += mismatches;
            entry.note += format(", %llu/%llu files verified",
                                 static_cast<unsigned long long>(checked - mismatches),
                                 static_cast<unsigned long long>(checked));
            for (const auto& problem : problems) {
                report.problems.push_back(format("%s: %s", check.name, problem.c_str()));
            }
        } else {
            report.problems.push_back(format("%s: source tree %s is missing, cannot compare",
                                             check.name, tree.c_str()));
        }
        report.partitions.push_back(format("%-8s %s", entry.name.c_str(), entry.note.c_str()));
    }

    report.ok = report.problems.empty() && report.slb2_ok;
    report.message = report.ok
                         ? format("%llu files verified, %u SLB2 copies checked",
                                  static_cast<unsigned long long>(report.files_checked), slb2_seen)
                         : format("%llu files checked, %llu mismatches, %u problems",
                                  static_cast<unsigned long long>(report.files_checked),
                                  static_cast<unsigned long long>(report.file_mismatches),
                                  static_cast<unsigned>(report.problems.size()));
    return report;
}

}  // namespace zlb
