// zeliboba - eMMC private helpers.
//
// Internal to src/hw/emmc/. Everything shared by emmc_card.cpp, emmc_fat.cpp and
// emmc_image.cpp lives here:
//
//   * the Toshiba THGBM3G5D1FBAIE geometry and the partition/register layout,
//   * the Vita master block (LBA 0 of the user area) and its 17 byte partition
//     records,
//   * the SLB2 boot container format,
//   * the FAT16 volume builder / reader.
//
// The FAT16 helpers follow the *real* 1.04 volumes: `os0.bin` and `vs0.bin` in
// out/PUP_dec are the genuine partition images (512 B sectors, 8 sectors per
// cluster, reserved 2, 2 FATs, 512 root entries) and are used as the geometry
// template. See docs/EMMC.md.
#pragma once

#include <array>
#include <cstddef>
#include <cstring>
#include <string>
#include <vector>

#include "common/types.h"
#include "hw/emmc.h"

namespace zlb {
namespace emmc {

// ---------------------------------------------------------------------------
// Card geometry (Toshiba THGBM3G5D1FBAIE, 4 GiB, eMMC 4.41)
// ---------------------------------------------------------------------------

constexpr u32 kBlockSize = 512;

constexpr u64 kBootPartitionSize = 2ull * MB;  ///< EXT_CSD BOOT_SIZE_MULTI = 0x10 * 128 KiB
constexpr u64 kBootAreaSize = 2 * kBootPartitionSize;
constexpr u64 kRpmbSize = 512ull * KB;  ///< datasheet RPMB_SIZE_MULT = 0x01 (128 KiB unit) rounded up

/// Total capacity of the part: 4 GiB (user + boot + RPMB).
constexpr u64 kPartTotalBytes = 4ull * GB;

/// User area capacity. The datasheet (page 4, "Density Specifications") gives the
/// THGBM3G5D1FBAIE user area as 3,997,171,712 bytes = 0xEE3C0000, SEC_COUNT
/// 0x00772000 blocks. The console's own master block instead declares 0x71A000
/// blocks (3,812,622,336 bytes = 3.55 GiB), i.e. ~184 MiB less - that is the
/// table a factory partitioner wrote, not the raw card capacity. The default
/// reconstructed image uses the console's own figure so the tables the tools
/// print match the hardware; both numbers are in docs/EMMC.md.
constexpr u64 kPartDatasheetUserBytes = 3997171712ull;  // 0xEE3C0000
constexpr u64 kPartTableTotalBytes = 0xE3400000ull;     // what the console's master block declares
constexpr u64 kPartTableTotalBlocks = kPartTableTotalBytes / kBlockSize;

constexpr u64 kUserAreaBytes = kPartTableTotalBytes;
constexpr u64 kUserAreaBlocks = kPartTableTotalBlocks;

/// The whole image file is user area + both boot partitions + RPMB.
constexpr u64 kImageBytes = kUserAreaBytes + kBootAreaSize + kRpmbSize;

/// The smallest image the builder can produce: everything up to and including
/// the second vs0 partition (0x1B000000 = 432 MiB) plus one sector of slack and
/// the boot/RPMB areas that live behind the user area.
constexpr u64 kMinImageBytes = 0x1B000000ull + 512ull * KB;

// ---------------------------------------------------------------------------
// FAT16 layout
// ---------------------------------------------------------------------------

/// Geometry read out of a genuine 1.04 partition image (os0.bin / vs0.bin).
struct FatGeometry {
    bool valid = false;
    std::string message;
    u32 bytes_per_sector = 512;
    u32 sectors_per_cluster = 8;
    u32 reserved_sectors = 2;
    u32 fat_count = 2;
    u32 root_entry_count = 512;
    u32 sectors_per_fat = 0;   ///< as found in the template (0 = recompute)
    u32 total_sectors = 0;     ///< as found in the template
    u32 media_descriptor = 0xF8;
    std::string oem_name = "SCEI";
    std::string volume_label = "NO NAME";
    u32 volume_id = 0;
    u32 boot_sector_copy = 0;  ///< first sector of the template; kept for the report
};

/// Read the BPB of a FAT16 partition image. Used to mirror real 1.04 volumes.
bool read_fat_geometry(const std::vector<u8>& image, FatGeometry& out);

/// The canonical 1.04 geometry, used when no template is available.
FatGeometry default_fat_geometry();

struct FatVolumeStats {
    bool ok = false;
    std::string message;

    u32 bytes_per_sector = 0;
    u32 sectors_per_cluster = 0;
    u32 cluster_bytes = 0;
    u32 reserved_sectors = 0;
    u32 fat_count = 0;
    u32 sectors_per_fat = 0;
    u32 root_entry_count = 0;
    u32 total_sectors = 0;
    u32 cluster_count = 0;
    u32 free_clusters = 0;
    u32 root_entries_used = 0;
    u32 root_entries_free = 0;
    u32 data_start_sector = 0;
    u32 data_sectors = 0;
    std::string oem_name;
    std::string volume_label;
    u32 volume_id = 0;
    std::string template_note;

    u64 files = 0;
    u64 directories = 0;
    u64 data_bytes = 0;   ///< sum of file sizes
    u64 clusters_used = 0;
    u64 largest_file = 0;
    u64 bytes_used = 0;   ///< clusters_used * cluster_bytes

    u64 data_end_sector() const { return static_cast<u64>(data_start_sector) + data_sectors; }
};

/// Build a FAT16 volume into `card` at [start_block, start_block + blocks), using
/// the *contents* of `source_root` as the volume root.
///
/// The file data is copied from the filesystem tree; the on-disk geometry (sector
/// size, cluster size, reserved sectors, FAT count, root entries) comes from
/// `geometry`, which is normally read from the genuine os0.bin/vs0.bin images.
bool build_fat_volume(EmmcCard& card, EmmcPartition partition, u64 start_block, u64 blocks,
                      const std::string& source_root, const FatGeometry& geometry, bool verbose,
                      FatVolumeStats& out);

/// Parse the FAT16 volume back out of the card and collect `out`.
bool mount_fat_volume(EmmcCard& card, EmmcPartition partition, u64 start_block, u64 blocks,
                      FatVolumeStats& out);

/// One file/directory found while walking a mounted volume.
struct FatDirEntry {
    std::string path;  ///< "kd/sysmem.skprx", directories end with '/'
    bool directory = false;
    u64 size = 0;
    u32 first_cluster = 0;
};

/// Walk the whole volume (used by --verify and inspect).
bool list_fat_volume(EmmcCard& card, EmmcPartition partition, u64 start_block, u64 blocks,
                     std::vector<FatDirEntry>& out, std::string& message);

/// Compare the mounted volume against the filesystem tree it was built from.
/// Every file must exist, be the right size and be byte identical.
bool verify_fat_volume(EmmcCard& card, EmmcPartition partition, u64 start_block, u64 blocks,
                       const std::string& source_root, u64& checked, u64& mismatches,
                       std::vector<std::string>& problems);

// ---------------------------------------------------------------------------
// SLB2 boot container
// ---------------------------------------------------------------------------
//
// Recovered from the retail 1.04 container (boot_slb2-00.pkg.seg02) and confirmed
// against the first eMMC dump:
//
//   0x00 char[4] "SLB2"
//   0x04 u32     version = 1
//   0x08 u32     header (table) size = 0x200
//   0x0C u32     entry count
//   0x10 u32     data offset hint = 0x2000
//   0x14 u32     reserved
//   0x20 ...     entries, stride 0x30:
//                  +0x00 u32 first 512 byte block of the payload
//                  +0x04 u32 payload size in bytes
//                  +0x08 u32 flags (0)
//                  +0x0C u32 reserved (0)
//                  +0x10 char[32] NUL padded name
//
// The payloads follow the table, each aligned to a 512 byte block.

constexpr u32 kSlb2Magic = 0x32424C53;  ///< "SLB2" little endian
constexpr u32 kSlb2Version = 1;
constexpr u32 kSlb2TableSize = 0x200;
constexpr u32 kSlb2HeaderSize = 0x20;
constexpr u32 kSlb2EntryStride = 0x30;
constexpr u32 kSlb2DataHint = 0x2000;

struct Slb2File {
    std::string name;
    std::string path;
};

std::vector<u8> build_slb2_container(const std::vector<Slb2File>& files, bool verbose);

struct Slb2EntryView {
    std::string name;
    u64 offset = 0;
    u64 size = 0;
    u32 first_block = 0;
    u32 flags = 0;
};

struct Slb2View {
    bool valid = false;
    std::string message;
    u32 version = 0;
    u32 table_size = 0;
    u32 count = 0;
    u32 data_hint = 0;
    u64 total_bytes = 0;
    std::vector<Slb2EntryView> entries;
};

/// Parse an SLB2 container. When `require_payload` is false only the table is
/// validated (the caller passed a short read of the first blocks, which is all
/// the table needs).
bool parse_slb2_container(const std::vector<u8>& data, Slb2View& out, bool require_payload = true);

// ---------------------------------------------------------------------------
// Vita master block (LBA 0 of the user area)
// ---------------------------------------------------------------------------

constexpr char kMasterMagic[33] = "Sony Computer Entertainment Inc.";  ///< 32 bytes + NUL
constexpr u32 kMasterVersion = 3;
constexpr u32 kMasterSizeField = 0x24;  ///< u32 total user area blocks
constexpr u32 kPartitionTableOffset = 0x50;
constexpr u32 kPartitionRecordSize = 17;
constexpr u32 kPartitionTableSlots = 16;

/// Partition code byte (values read straight out of the reference dump).
enum class VitaPartCode : u8 {
    Empty = 0x00,
    IdStorage = 0x01,
    Slb2 = 0x02,
    Os0 = 0x03,
    Vs0 = 0x04,
    Vd0 = 0x05,
    Tm0 = 0x06,
    Ur0 = 0x07,
    Ux0 = 0x08,
    Gro0 = 0x09,
    Grw0 = 0x0A,
    Ud0 = 0x0B,
    Sa0 = 0x0C,
    UnknownMc = 0x0D,
    Pd0 = 0x0E,
};

/// Partition type byte; a coarse filesystem hint (FAT16 / exFAT / raw).
enum class VitaPartType : u8 {
    Unknown0 = 0x00,
    Fat16 = 0x06,
    ExFat = 0x07,
    UnknownB = 0x0B,
    Raw = 0xDA,
};

const char* to_string(VitaPartCode code);
const char* to_string(VitaPartType type);

struct VitaPartition {
    u64 offset_blocks = 0;
    u64 size_blocks = 0;
    VitaPartCode code = VitaPartCode::Empty;
    VitaPartType type = VitaPartType::Unknown0;
    u8 active = 0;
    u32 flags = 0;
};

/// Serialise a 512 byte master block.
std::vector<u8> build_master_block(const std::vector<VitaPartition>& partitions, u32 total_blocks);

/// Parse a 512 byte master block. False when the magic / version / signature
/// do not match.
bool parse_master_block(const u8* data, size_t length, std::vector<VitaPartition>& partitions,
                        u32& total_blocks, std::string& message);

// ---------------------------------------------------------------------------
// Small shared helpers
// ---------------------------------------------------------------------------

inline u16 read_le16(const u8* p) { return static_cast<u16>(p[0] | (p[1] << 8)); }
inline u32 read_le32(const u8* p) {
    return static_cast<u32>(p[0]) | (static_cast<u32>(p[1]) << 8) | (static_cast<u32>(p[2]) << 16) |
           (static_cast<u32>(p[3]) << 24);
}
inline void write_le16(u8* p, u16 v) {
    p[0] = static_cast<u8>(v & 0xFF);
    p[1] = static_cast<u8>(v >> 8);
}
inline void write_le32(u8* p, u32 v) {
    p[0] = static_cast<u8>(v & 0xFF);
    p[1] = static_cast<u8>((v >> 8) & 0xFF);
    p[2] = static_cast<u8>((v >> 16) & 0xFF);
    p[3] = static_cast<u8>((v >> 24) & 0xFF);
}

/// Build the (reconstructed) idstorage partition image: a 512 byte index table
/// of leaf numbers followed by 256 leaves. See docs/EMMC.md - the real ID
/// storage content is personal to each console and is not in the PUP.
std::vector<u8> build_idstorage_image();

/// Detailed per-partition result used while verifying (see verify_emmc_image in
/// the public header, which flattens this into display strings).
struct EmmcPartitionReport {
    std::string name;
    std::string payload;
    u64 offset = 0;
    u64 size = 0;
    bool present = false;
    bool mounted = false;
    std::string note;
    FatVolumeStats fat;
};

}  // namespace emmc
}  // namespace zlb
