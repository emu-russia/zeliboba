// zeliboba - FAT16 volume builder and reader for the eMMC user area.
//
// The volume this writes is the console's own `os0` / `vs0` partition: 512 byte
// sectors, 8 sectors (4 KiB) per cluster, 2 reserved sectors, 2 FATs, 512 root
// directory entries, OEM "SCEI", label "NO NAME" - exactly the geometry of the
// genuine 1.04 partition images out/PUP_dec/os0.bin and out/PUP_dec/vs0.bin.
// The *content* is written from the extracted filesystem tree (out/fs/os0,
// out/fs/vs0), which carries every file those images carry (63 files /
// 6,405,765 bytes and 929 files / 84,375,093 bytes - verified, see docs/EMMC.md).
//
// Written here: boot sector, FSInfo, both FATs, the root directory, every
// subdirectory, long file name entries and the file data. mount_fat_volume(),
// list_fat_volume() and verify_fat_volume() parse the volume straight back out
// of the card so the tool can prove the result is readable.
#include "hw/emmc/emmc.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <map>
#include <set>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace zlb {
namespace emmc {

namespace {

namespace fs = std::filesystem;

constexpr u32 kDirEntrySize = 32;
constexpr u16 kFatEoc = 0xFFFF;
constexpr u16 kFatReserved1 = 0xFFF8;  // FAT[0]: media descriptor + all ones
constexpr u16 kFatReserved2 = 0xFFFF;

/// Cluster numbers of a FAT16 chain are 16 bit; anything at or above 0xFFF8 ends
/// the chain.
inline bool fat_is_end(u16 value) { return value >= 0xFFF8; }

// ---------------------------------------------------------------------------
// Sector level writers / readers
// ---------------------------------------------------------------------------

struct VolumeWriter {
    EmmcCard* card = nullptr;
    EmmcPartition partition = EmmcPartition::User;
    u64 start_block = 0;
    u64 blocks = 0;
    u32 sector_size = 512;
    bool failed = false;

    bool write_at(u64 byte_offset, const void* data, size_t length) {
        if (failed) return false;
        if (byte_offset + length > blocks * emmc::kBlockSize) {
            failed = true;
            return false;
        }
        if (!card->write_bytes(partition, start_block * emmc::kBlockSize + byte_offset, data, length)) {
            failed = true;
            return false;
        }
        return true;
    }

    bool write_sector(u64 sector, const void* data) {
        return write_at(sector * sector_size, data, sector_size);
    }
};

// ---------------------------------------------------------------------------
// Directory tree collected from the source filesystem
// ---------------------------------------------------------------------------

struct TreeNode {
    std::string name;       ///< name as it appears in the directory (UTF-8)
    std::string path;       ///< absolute source path (files only)
    std::string short_name; ///< 11 byte 8.3 name
    std::string lfn_utf16;  ///< UTF-16LE bytes, empty when no LFN entries are needed
    bool directory = false;
    u64 size = 0;
    u8 attributes = 0x20;
    u32 first_cluster = 0;
    u32 clusters = 0;
    u64 dir_bytes = 0;
    std::vector<TreeNode> children;
};

std::string utf8_to_ansi(const std::string& text) {
#if defined(_WIN32)
    if (text.empty()) return {};
    const int wide_len =
        MultiByteToWideChar(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), nullptr, 0);
    if (wide_len <= 0) return text;
    std::wstring wide(static_cast<size_t>(wide_len), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), wide.data(), wide_len);
    const int ansi_len =
        WideCharToMultiByte(CP_ACP, 0, wide.data(), wide_len, nullptr, 0, nullptr, nullptr);
    if (ansi_len <= 0) return text;
    std::string ansi(static_cast<size_t>(ansi_len), '\0');
    WideCharToMultiByte(CP_ACP, 0, wide.data(), wide_len, ansi.data(), ansi_len, nullptr, nullptr);
    return ansi;
#else
    return text;
#endif
}

/// ANSI (the console's Shift-JIS) -> UTF-16LE bytes for the LFN entries.
std::string ansi_to_utf16le(const std::string& text) {
    std::string out;
#if defined(_WIN32)
    if (text.empty()) return out;
    const int wide_len =
        MultiByteToWideChar(CP_ACP, 0, text.c_str(), static_cast<int>(text.size()), nullptr, 0);
    if (wide_len <= 0) return out;
    std::wstring wide(static_cast<size_t>(wide_len), L'\0');
    MultiByteToWideChar(CP_ACP, 0, text.c_str(), static_cast<int>(text.size()), wide.data(), wide_len);
    out.resize(wide.size() * 2);
    std::memcpy(out.data(), wide.data(), out.size());
#else
    for (char c : text) {
        out.push_back(c);
        out.push_back('\0');
    }
#endif
    return out;
}

std::string utf16_preview(const std::string& utf16) {
    std::string out;
    for (size_t i = 0; i + 1 < utf16.size(); i += 2) {
        const u16 value = static_cast<u16>(static_cast<u8>(utf16[i]) | (static_cast<u8>(utf16[i + 1]) << 8));
        if (value == 0) break;
        out.push_back(value < 0x80 ? static_cast<char>(value) : '?');
    }
    return out;
}

bool is_valid_sfn_char(char c) {
    if (static_cast<unsigned char>(c) <= 0x20) return false;
    switch (c) {
        case '"':
        case '*':
        case '+':
        case ',':
        case '/':
        case ':':
        case ';':
        case '<':
        case '=':
        case '>':
        case '?':
        case '[':
        case '\\':
        case ']':
        case '|':
            return false;
        default:
            return static_cast<unsigned char>(c) < 0x80;
    }
}

/// Plain 8.3 conversion. Returns false when the name does not fit.
bool make_sfn(const std::string& ansi_name, bool directory, std::string& out) {
    std::string base = ansi_name;
    std::string ext;
    if (!directory) {
        const size_t dot = ansi_name.rfind('.');
        if (dot != std::string::npos && dot != 0) {
            base = ansi_name.substr(0, dot);
            ext = ansi_name.substr(dot + 1);
        }
    }

    std::string upper_base;
    std::string upper_ext;
    for (char c : base) {
        if (c == '.' || !is_valid_sfn_char(c)) return false;
        upper_base.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    }
    for (char c : ext) {
        if (c == '.' || !is_valid_sfn_char(c)) return false;
        upper_ext.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    }
    if (upper_base.empty() || upper_base.size() > 8 || upper_ext.size() > 3) return false;

    upper_base.resize(8, ' ');
    upper_ext.resize(3, ' ');
    out = upper_base + upper_ext;
    return true;
}

/// Generated ~N name for names that cannot be represented in 8.3.
std::string make_generated_sfn(const std::string& ansi_name, u32 index) {
    std::string base;
    for (char c : ansi_name) {
        if (c == '.' || !is_valid_sfn_char(c)) continue;
        base.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
        if (base.size() >= 6) break;
    }
    if (base.empty()) base = "FILE";
    std::string name = base + "~" + format("%u", index);
    name.resize(8, ' ');
    return name + "   ";
}

std::string sfn_display(const std::string& sfn) {
    const std::string base = trim(sfn.substr(0, 8));
    const std::string ext = trim(sfn.substr(8, 3));
    return ext.empty() ? base : base + "." + ext;
}

u8 sfn_checksum(const std::string& sfn) {
    u8 sum = 0;
    for (size_t i = 0; i < 11; ++i) {
        const u8 byte = i < sfn.size() ? static_cast<u8>(sfn[i]) : static_cast<u8>(' ');
        sum = static_cast<u8>(((sum & 1) ? 0x80 : 0x00) + (sum >> 1) + byte);
    }
    return sum;
}

/// Fill a fixed width ASCII field, space padded (as the console's own volumes do).
void pad_field(std::vector<u8>& sector, size_t offset, size_t width, const std::string& text) {
    for (size_t i = 0; i < width; ++i) {
        sector[offset + i] = i < text.size() ? static_cast<u8>(text[i]) : static_cast<u8>(' ');
    }
}

/// Fill the 32 byte directory entry.
void fill_entry(u8* entry, const std::string& sfn, u8 attributes, u32 first_cluster, u64 size) {
    std::memset(entry, 0, kDirEntrySize);
    for (size_t i = 0; i < 11; ++i) {
        entry[i] = i < sfn.size() ? static_cast<u8>(sfn[i]) : static_cast<u8>(' ');
    }
    entry[11] = attributes;
    write_le16(entry + 20, static_cast<u16>((first_cluster >> 16) & 0xFFFF));
    write_le16(entry + 26, static_cast<u16>(first_cluster & 0xFFFF));
    write_le32(entry + 28, static_cast<u32>(size));
}

/// Append the long file name entries for `utf16`, highest sequence number first.
void emit_lfn(std::vector<u8>& out, const std::string& utf16, u8 checksum) {
    const size_t chars = utf16.size() / 2;
    if (chars == 0) return;
    const size_t parts = (chars + 12) / 13;
    static const size_t kOffsets[13] = {1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30};

    for (size_t n = parts; n-- > 0;) {
        u8 entry[kDirEntrySize];
        std::memset(entry, 0, sizeof(entry));
        entry[0] = static_cast<u8>((n + 1) | ((n + 1 == parts) ? 0x40 : 0x00));
        entry[11] = 0x0F;
        entry[12] = 0;
        entry[13] = checksum;
        for (size_t i = 0; i < 13; ++i) {
            const size_t index = n * 13 + i;
            u16 value = 0xFFFF;
            if (index < chars) {
                value = static_cast<u16>(static_cast<u8>(utf16[index * 2]) |
                                         (static_cast<u8>(utf16[index * 2 + 1]) << 8));
            } else if (index == chars) {
                value = 0x0000;
            }
            write_le16(entry + kOffsets[i], value);
        }
        out.insert(out.end(), entry, entry + kDirEntrySize);
    }
}

/// Number of 32 byte slots a child occupies in its parent directory.
u32 entry_slots(const TreeNode& node) {
    if (node.lfn_utf16.empty()) return 1;
    return static_cast<u32>((node.lfn_utf16.size() / 2 + 12) / 13) + 1;
}

/// Slots a directory needs: its children plus "." and ".." unless it is the root.
u64 directory_slots(const TreeNode& node, bool is_root) {
    u64 slots = is_root ? 0 : 2;
    for (const auto& child : node.children) slots += entry_slots(child);
    return slots;
}

// ---------------------------------------------------------------------------
// Reading a FAT16 volume back
// ---------------------------------------------------------------------------

struct VolumeReader {
    EmmcCard* card = nullptr;
    EmmcPartition partition = EmmcPartition::User;
    u64 start_block = 0;
    u64 blocks = 0;
    u32 sector_size = 512;

    bool read_at(u64 byte_offset, void* out, size_t length) const {
        if (byte_offset + length > blocks * emmc::kBlockSize) return false;
        return card->read_bytes(partition, start_block * emmc::kBlockSize + byte_offset, out, length);
    }
};

struct MountedFat {
    bool valid = false;
    std::string message;
    u32 bps = 0;
    u32 spc = 0;
    u32 reserved = 0;
    u32 fat_count = 0;
    u32 spf = 0;
    u32 root_entries = 0;
    u32 total_sectors = 0;
    u32 cluster_count = 0;
    u32 free_clusters = 0;
    u32 root_dir_sectors = 0;
    u32 first_data_sector = 0;
    u32 first_root_sector = 0;
    u32 data_sectors = 0;
    u64 fat_size_bytes = 0;
    std::string oem;
    std::string label;
    u32 volume_id = 0;
    std::vector<u16> fat;
    VolumeReader reader;

    u64 cluster_offset(u32 cluster) const {
        return (static_cast<u64>(first_data_sector) + static_cast<u64>(cluster - 2) * spc) * bps;
    }
    bool cluster_valid(u32 cluster) const { return cluster >= 2 && cluster < cluster_count + 2; }
    u16 fat_entry(u32 cluster) const {
        return cluster < fat.size() ? fat[cluster] : static_cast<u16>(kFatEoc);
    }
};

bool mount_volume(const VolumeReader& reader_in, MountedFat& out) {
    out = MountedFat{};
    VolumeReader reader = reader_in;
    out.reader = reader;

    std::vector<u8> boot(512, 0);
    if (!reader.read_at(0, boot.data(), boot.size())) {
        out.message = "cannot read the boot sector";
        return false;
    }
    if (read_le16(boot.data() + 510) != 0xAA55) {
        out.message = "no FAT boot signature (0xAA55) at offset 510";
        return false;
    }
    out.bps = read_le16(boot.data() + 11);
    out.spc = boot[13];
    out.reserved = read_le16(boot.data() + 14);
    out.fat_count = boot[16];
    out.root_entries = read_le16(boot.data() + 17);
    const u32 total16 = read_le16(boot.data() + 19);
    const u32 total32 = read_le32(boot.data() + 32);
    out.total_sectors = total16 ? total16 : total32;
    out.spf = read_le16(boot.data() + 22);
    out.oem.assign(reinterpret_cast<const char*>(boot.data() + 3), 8);
    out.oem = trim(out.oem);
    out.label.assign(reinterpret_cast<const char*>(boot.data() + 43), 11);
    out.label = trim(out.label);
    out.volume_id = read_le32(boot.data() + 39);

    if (out.bps != 512 || out.spc == 0 || out.spf == 0 || out.fat_count == 0 || out.reserved == 0) {
        out.message = format("implausible BPB (bps=%u spc=%u spf=%u fats=%u reserved=%u)", out.bps,
                             out.spc, out.spf, out.fat_count, out.reserved);
        return false;
    }

    // The caller may hand us the whole partition slot (which is larger than the
    // volume) or the exact volume size; clamp to whatever the BPB says so the
    // geometry always matches the filesystem that was written.
    const u64 volume_bytes = static_cast<u64>(out.total_sectors) * out.bps;
    if (volume_bytes == 0) {
        out.message = "BPB total sector count is zero";
        return false;
    }
    VolumeReader clamped = reader;
    if (volume_bytes < reader.blocks * emmc::kBlockSize) {
        clamped.blocks = volume_bytes / emmc::kBlockSize;
    } else if (volume_bytes > reader.blocks * emmc::kBlockSize) {
        out.message = format("volume claims %s but the partition only has %s",
                             human_size(volume_bytes).c_str(),
                             human_size(reader.blocks * emmc::kBlockSize).c_str());
        return false;
    }
    out.reader = clamped;
    reader = clamped;

    out.root_dir_sectors = (out.root_entries * kDirEntrySize + out.bps - 1) / out.bps;
    out.first_root_sector = out.reserved + out.fat_count * out.spf;
    out.first_data_sector = out.first_root_sector + out.root_dir_sectors;
    const u64 data_sectors = static_cast<u64>(out.total_sectors) - out.first_data_sector;
    out.cluster_count = static_cast<u32>(data_sectors / out.spc);
    out.data_sectors = out.cluster_count * out.spc;
    if (out.cluster_count < 1) {
        out.message = "no data clusters";
        return false;
    }

    // The FAT only has to describe `cluster_count` clusters (the two reserved
    // entries live in the first two slots of the same table).
    out.fat.assign(out.cluster_count + 2, 0);
    const u64 fat_bytes = static_cast<u64>(out.spf) * out.bps;
    if (fat_bytes < static_cast<u64>(out.cluster_count) * 2) {
        out.message = format("FAT is too small for the volume (%s for %u clusters)",
                             human_size(fat_bytes).c_str(), out.cluster_count);
        return false;
    }
    out.fat_size_bytes = fat_bytes;
    std::vector<u8> raw(fat_bytes);
    if (!reader.read_at(static_cast<u64>(out.reserved) * out.bps, raw.data(), raw.size())) {
        out.message = "cannot read the FAT";
        return false;
    }
    for (u32 i = 0; i < out.fat.size(); ++i) out.fat[i] = read_le16(raw.data() + i * 2);

    // FSInfo (informational only; the console's own volumes do not fill it in,
    // so fall back to counting the free entries in the FAT).
    out.free_clusters = 0;
    for (u32 cluster = 2; cluster < out.cluster_count + 2; ++cluster) {
        if (out.fat[cluster] == 0) ++out.free_clusters;
    }
    if (out.reserved > 1) {
        std::vector<u8> fsinfo(512, 0);
        if (reader.read_at(static_cast<u64>(out.bps), fsinfo.data(), fsinfo.size())) {
            if (read_le32(fsinfo.data() + 0) == 0x41615252 && read_le32(fsinfo.data() + 484) == 0x61417272) {
                const u32 reported = read_le32(fsinfo.data() + 488);
                if (reported != 0 && reported <= out.cluster_count) out.free_clusters = reported;
            }
        }
    }

    out.valid = true;
    out.message = "ok";
    return true;
}

/// Read a cluster chain, guarding against loops and runaway chains.
std::vector<u32> cluster_chain(const MountedFat& volume, u32 first, size_t max_clusters = 200000) {
    std::vector<u32> chain;
    std::set<u32> seen;
    u32 cluster = first;
    while (volume.cluster_valid(cluster) && chain.size() < max_clusters) {
        if (!seen.insert(cluster).second) break;
        chain.push_back(cluster);
        const u16 next = volume.fat_entry(cluster);
        if (fat_is_end(next) || next == 0) break;
        cluster = next;
    }
    return chain;
}

std::vector<u8> read_chain(const MountedFat& volume, u32 first, u64 size) {
    std::vector<u8> out;
    if (size == 0 || first < 2) return out;
    out.reserve(static_cast<size_t>(std::min<u64>(size, 1u << 20)));
    const u32 cluster_bytes = volume.spc * volume.bps;
    for (u32 cluster : cluster_chain(volume, first)) {
        std::vector<u8> buffer(cluster_bytes, 0);
        if (!volume.reader.read_at(volume.cluster_offset(cluster), buffer.data(), buffer.size())) break;
        const size_t take = static_cast<size_t>(std::min<u64>(cluster_bytes, size - out.size()));
        out.insert(out.end(), buffer.begin(), buffer.begin() + take);
        if (out.size() >= size) break;
    }
    return out;
}

/// Parse one directory. Handles long file names, returns entries in order.
struct RawDirEntry {
    std::string name;  ///< long name when present, otherwise the 8.3 name
    std::string short_name;
    u8 attributes = 0;
    u32 first_cluster = 0;
    u64 size = 0;
    bool deleted = false;
    bool end = false;
};

std::vector<RawDirEntry> parse_directory(const std::vector<u8>& data) {
    std::vector<RawDirEntry> out;
    std::map<size_t, std::string> pending;  // reversed LFN parts
    std::string pending_name;

    for (size_t offset = 0; offset + kDirEntrySize <= data.size(); offset += kDirEntrySize) {
        const u8* entry = data.data() + offset;
        if (entry[0] == 0x00) {
            RawDirEntry end;
            end.end = true;
            out.push_back(end);
            break;
        }
        if (entry[0] == 0xE5) {
            pending_name.clear();
            continue;
        }
        if (entry[11] == 0x0F) {
            // Long file name part: collect and concatenate in sequence order.
            const u32 sequence = entry[0] & 0x3F;
            static const size_t kOffsets[13] = {1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30};
            std::vector<u16> chars;
            for (size_t i = 0; i < 13; ++i) chars.push_back(read_le16(entry + kOffsets[i]));
            std::string piece;
            for (u16 value : chars) {
                if (value == 0x0000) break;
                if (value == 0xFFFF) continue;
                piece.push_back(static_cast<char>(value & 0xFF));
                piece.push_back(static_cast<char>(value >> 8));
            }
            (void)sequence;
            // Parts arrive last-to-first, so prepending reconstructs the name.
            pending_name = piece + pending_name;
            continue;
        }

        RawDirEntry result;
        const std::string base = trim(std::string(reinterpret_cast<const char*>(entry), 8));
        const std::string ext = trim(std::string(reinterpret_cast<const char*>(entry) + 8, 3));
        result.short_name = ext.empty() ? base : base + "." + ext;
        result.attributes = entry[11];
        result.first_cluster =
            (static_cast<u32>(read_le16(entry + 20)) << 16) | read_le16(entry + 26);
        result.size = read_le32(entry + 28);
        if (!pending_name.empty()) {
            std::string decoded;
            for (size_t i = 0; i + 1 < pending_name.size(); i += 2) {
                const u16 value =
                    static_cast<u16>(static_cast<u8>(pending_name[i]) |
                                     (static_cast<u8>(pending_name[i + 1]) << 8));
                if (value == 0) break;
                decoded.push_back(value < 0x80 ? static_cast<char>(value) : '?');
            }
            result.name = decoded.empty() ? result.short_name : decoded;
        } else {
            result.name = result.short_name;
        }
        pending_name.clear();
        out.push_back(result);
    }
    return out;
}

std::vector<u8> read_directory_data(const MountedFat& volume, u32 first_cluster, bool is_root) {
    std::vector<u8> data;
    if (is_root) {
        const u64 bytes = static_cast<u64>(volume.root_dir_sectors) * volume.bps;
        data.resize(static_cast<size_t>(bytes));
        if (!volume.reader.read_at(static_cast<u64>(volume.first_root_sector) * volume.bps, data.data(),
                                   data.size())) {
            data.clear();
        }
        return data;
    }
    const u32 cluster_bytes = volume.spc * volume.bps;
    for (u32 cluster : cluster_chain(volume, first_cluster)) {
        std::vector<u8> buffer(cluster_bytes, 0);
        if (!volume.reader.read_at(volume.cluster_offset(cluster), buffer.data(), buffer.size())) break;
        data.insert(data.end(), buffer.begin(), buffer.end());
        if (data.size() > (4u << 20)) break;
    }
    return data;
}

}  // namespace

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

FatGeometry default_fat_geometry() {
    FatGeometry geometry;
    geometry.valid = true;
    geometry.message = "built-in 1.04 default (matches out/PUP_dec/os0.bin)";
    geometry.bytes_per_sector = 512;
    geometry.sectors_per_cluster = 8;
    geometry.reserved_sectors = 2;
    geometry.fat_count = 2;
    geometry.root_entry_count = 512;
    geometry.sectors_per_fat = 0;
    geometry.total_sectors = 0;
    geometry.media_descriptor = 0xF8;
    geometry.oem_name = "SCEI";
    geometry.volume_label = "NO NAME";
    geometry.volume_id = 0x40EA3B2A;
    return geometry;
}

bool read_fat_geometry(const std::vector<u8>& image, FatGeometry& out) {
    out = FatGeometry{};
    if (image.size() < 512) {
        out.message = "image is smaller than one sector";
        return false;
    }
    const u8* sector = image.data();
    if (read_le16(sector + 510) != 0xAA55) {
        out.message = "no 0xAA55 boot signature";
        return false;
    }
    out.bytes_per_sector = read_le16(sector + 11);
    out.sectors_per_cluster = sector[13];
    out.reserved_sectors = read_le16(sector + 14);
    out.fat_count = sector[16];
    out.root_entry_count = read_le16(sector + 17);
    const u32 total16 = read_le16(sector + 19);
    const u32 total32 = read_le32(sector + 32);
    out.total_sectors = total16 ? total16 : total32;
    const u32 spf16 = read_le16(sector + 22);
    const u32 spf32 = read_le32(sector + 36);
    out.sectors_per_fat = spf16 ? spf16 : spf32;
    out.media_descriptor = sector[21];
    out.oem_name = trim(std::string(reinterpret_cast<const char*>(sector + 3), 8));
    out.volume_label = trim(std::string(reinterpret_cast<const char*>(sector + 43), 11));
    out.volume_id = read_le32(sector + 39);

    if (out.bytes_per_sector == 0 || out.sectors_per_cluster == 0 || out.reserved_sectors == 0 ||
        out.fat_count == 0 || out.sectors_per_fat == 0) {
        out.message = "incomplete BPB";
        return false;
    }
    if (spf16 == 0) {
        out.message = "not a FAT16 volume (16 bit sectors-per-FAT field is zero)";
        return false;
    }
    out.valid = true;
    out.message = format("FAT16 %u B/sector, %u sectors/cluster, %u reserved, %u FATs, %u root entries, "
                         "OEM \"%s\", label \"%s\"",
                         out.bytes_per_sector, out.sectors_per_cluster, out.reserved_sectors,
                         out.fat_count, out.root_entry_count, out.oem_name.c_str(),
                         out.volume_label.c_str());
    return true;
}

// ---------------------------------------------------------------------------
// Builder
// ---------------------------------------------------------------------------

bool build_fat_volume(EmmcCard& card, EmmcPartition partition, u64 start_block, u64 blocks,
                      const std::string& source_root, const FatGeometry& geometry, bool verbose,
                      FatVolumeStats& out) {
    out = FatVolumeStats{};

    const u32 bps = geometry.bytes_per_sector ? geometry.bytes_per_sector : 512;
    const u32 spc = geometry.sectors_per_cluster ? geometry.sectors_per_cluster : 8;
    const u32 reserved = geometry.reserved_sectors ? geometry.reserved_sectors : 2;
    const u32 fat_count = geometry.fat_count ? geometry.fat_count : 2;
    const u32 root_entries = geometry.root_entry_count ? geometry.root_entry_count : 512;

    if (bps != 512) {
        out.message = "only 512 byte sectors are supported";
        return false;
    }
    if (spc == 0 || (spc & (spc - 1)) != 0) {
        out.message = "sectors per cluster must be a power of two";
        return false;
    }
    if (!fs::is_directory(source_root)) {
        out.message = format("source tree '%s' does not exist", source_root.c_str());
        return false;
    }

    const u64 total_sectors = blocks * emmc::kBlockSize / bps;
    const u32 root_dir_sectors = (root_entries * kDirEntrySize + bps - 1) / bps;
    const u32 cluster_bytes = spc * bps;

    // ---- collect the source tree ----------------------------------------
    u64 file_count = 0;
    u64 dir_count = 0;
    u64 data_bytes = 0;
    u64 largest_file = 0;

    std::function<bool(const fs::path&, TreeNode&)> collect = [&](const fs::path& path,
                                                                  TreeNode& node) -> bool {
        std::error_code ec;
        fs::directory_iterator iterator(path, fs::directory_options::skip_permission_denied, ec);
        if (ec) {
            out.message = format("cannot list %s: %s", path.string().c_str(), ec.message().c_str());
            return false;
        }
        std::vector<fs::directory_entry> entries;
        for (const auto& entry : iterator) entries.push_back(entry);
        std::sort(entries.begin(), entries.end(),
                  [](const fs::directory_entry& a, const fs::directory_entry& b) {
                      return a.path().filename().string() < b.path().filename().string();
                  });

        std::map<std::string, u32> used_short_names;
        for (const auto& entry : entries) {
            std::error_code type_ec;
            const bool is_dir = entry.is_directory(type_ec);
            if (type_ec) continue;
            if (!is_dir && !entry.is_regular_file(type_ec)) continue;  // skip symlinks, devices

            TreeNode child;
            child.name = entry.path().filename().string();
            child.directory = is_dir;

            const std::string ansi = utf8_to_ansi(child.name);
            std::string sfn;
            const bool plain = make_sfn(ansi, is_dir, sfn);
            bool needs_lfn = !plain || sfn_display(sfn) != to_upper(ansi);
            if (plain && used_short_names.count(sfn) != 0) {
                needs_lfn = true;
                sfn.clear();
            }
            if (sfn.empty()) {
                u32 index = 1;
                std::string candidate;
                do {
                    candidate = make_generated_sfn(ansi, index++);
                } while (used_short_names.count(candidate) != 0 && index < 100000);
                sfn = candidate;
                needs_lfn = true;
            }
            used_short_names[sfn] = 1;
            child.short_name = sfn;

            if (needs_lfn) {
                child.lfn_utf16 = ansi_to_utf16le(child.name);
                if (child.lfn_utf16.empty()) child.lfn_utf16 = ansi_to_utf16le(to_upper(ansi));
            }

            if (is_dir) {
                child.attributes = 0x10;
                ++dir_count;
                if (!collect(entry.path(), child)) return false;
            } else {
                child.attributes = 0x20;
                child.path = entry.path().string();
                child.size = entry.file_size(type_ec);
                if (type_ec) child.size = 0;
                ++file_count;
                data_bytes += child.size;
                largest_file = std::max(largest_file, child.size);
            }
            node.children.push_back(std::move(child));
        }
        return true;
    };

    TreeNode root;
    root.name = "/";
    root.directory = true;
    if (!collect(fs::path(source_root), root)) return false;

    out.files = file_count;
    out.directories = dir_count;
    out.data_bytes = data_bytes;
    out.largest_file = largest_file;

    // ---- geometry and cluster count -------------------------------------
    // `sectors_per_fat` is a fixed point of the FAT/root/data split. When the
    // template's own value is large enough for the volume we keep it (that is
    // what the console shipped: 19 sectors for os0, 259 for vs0), otherwise the
    // minimal self-consistent value is computed.
    u32 sectors_per_fat = 0;
    u32 cluster_count = 0;
    for (int attempt = 0; attempt < 6; ++attempt) {
        const u32 minimal =
            static_cast<u32>((cluster_count + 2 + (bps / 2 - 1)) / (bps / 2));
        sectors_per_fat = geometry.sectors_per_fat > minimal ? geometry.sectors_per_fat : minimal;
        const u64 data_sectors =
            total_sectors - reserved - static_cast<u64>(fat_count) * sectors_per_fat - root_dir_sectors;
        if (data_sectors == 0) {
            out.message = "partition is too small for the FAT and root directory";
            return false;
        }
        const u32 next = static_cast<u32>(data_sectors / spc);
        if (next == cluster_count) break;
        cluster_count = next;
    }
    if (sectors_per_fat < static_cast<u32>((cluster_count + 2 + (bps / 2 - 1)) / (bps / 2))) {
        out.message = format("template FAT size %u sectors cannot describe %u clusters",
                             sectors_per_fat, cluster_count);
        return false;
    }
    if (cluster_count < 4085 || cluster_count > 65525) {
        out.message = format("FAT16 needs 4085..65525 clusters; this geometry yields %u for a %s "
                             "partition",
                             cluster_count, human_size(blocks * emmc::kBlockSize).c_str());
        return false;
    }

    const u32 first_data_sector = reserved + fat_count * sectors_per_fat + root_dir_sectors;

    const u64 root_slots_used = directory_slots(root, true);
    out.root_entries_used = static_cast<u32>(root_slots_used);
    out.root_entries_free = root_entries > root_slots_used
                                ? static_cast<u32>(root_entries - root_slots_used)
                                : 0;
    if (root_slots_used > root_entries) {
        out.message = format("the root directory needs %llu entries but a FAT16 root holds %u",
                             static_cast<unsigned long long>(root_slots_used), root_entries);
        return false;
    }

    std::vector<u8> fat(static_cast<size_t>(cluster_count) + 2, 0);
    write_le16(fat.data() + 0, kFatReserved1);
    write_le16(fat.data() + 2, kFatReserved2);

    u32 next_free = 2;
    auto allocate = [&](u64 count, u32& first) -> bool {
        if (count == 0) {
            first = 0;
            return true;
        }
        if (static_cast<u64>(next_free - 2) + count > cluster_count) return false;
        first = next_free;
        for (u64 i = 0; i < count; ++i) {
            const u32 cluster = next_free + static_cast<u32>(i);
            const u32 next = (i + 1 == count) ? kFatEoc : cluster + 1;
            write_le16(fat.data() + static_cast<size_t>(cluster) * 2, static_cast<u16>(next));
        }
        next_free += static_cast<u32>(count);
        return true;
    };

    // Directories first: allocate a chain for every subdirectory in breadth
    // first order, then write the contents.
    std::vector<TreeNode*> queue;
    for (auto& child : root.children) {
        if (child.directory) queue.push_back(&child);
    }
    std::vector<TreeNode*> all_dirs;
    for (size_t index = 0; index < queue.size(); ++index) {
        TreeNode* dir = queue[index];
        all_dirs.push_back(dir);
        const u64 slots = directory_slots(*dir, false);
        const u64 bytes = slots * kDirEntrySize;
        const u64 clusters = (bytes + cluster_bytes - 1) / cluster_bytes;
        u32 first = 0;
        if (!allocate(clusters, first)) {
            out.message = format("out of clusters allocating directory '%s'", dir->name.c_str());
            return false;
        }
        dir->first_cluster = first;
        dir->clusters = static_cast<u32>(clusters);
        dir->dir_bytes = bytes;
        for (auto& child : dir->children) {
            if (child.directory) queue.push_back(&child);
        }
    }

    // File data, one sequential pass. Directory clusters are already taken, so
    // the cursor starts at the first free cluster and skips the per-file slack:
    // the unused tail of each file's last cluster, and the gaps between files,
    // are never touched and the image stays sparse.
    VolumeWriter writer;
    writer.card = &card;
    writer.partition = partition;
    writer.start_block = start_block;
    writer.blocks = blocks;
    writer.sector_size = bps;

    u64 file_cursor =
        (static_cast<u64>(first_data_sector) + static_cast<u64>(next_free - 2) * spc) * bps;
    u64 files_bytes_written = 0;
    std::vector<u8> buffer(1u << 20);

    std::function<bool(TreeNode&)> write_files = [&](TreeNode& node) -> bool {
        for (auto& child : node.children) {
            if (child.directory) {
                if (!write_files(child)) return false;
                continue;
            }
            if (child.size == 0) {
                child.first_cluster = 0;
                child.clusters = 0;
                continue;
            }
            // Substitution (round 395): the workspace holds only the *decrypted* os0
            // content - fs/os0/*.skprx start with the SCE module magic 53 43 45 00 - while
            // NSKBL's format validator (0x5101A4B0) accepts only 7F 45 4C 46 = "\x7FELF"
            // (see docs/NSKBL.md round 394).  The same extraction also provides the ELF form
            // (fs_dec/os0/psp2bootconfig.elf), so put that on the card instead when
            // ZLB_OS0_ELF=1.  Gated: by default the volume stays byte-identical to fs/.
            static const bool os0_elf = [] {
                const char* on = std::getenv("ZLB_OS0_ELF");
                return on != nullptr && on[0] != '0';
            }();
            if (os0_elf && !child.directory) {
                std::string candidate = child.path;
                bool swapped = false;
                for (const char* pair : {"fs/os0", "fs\\os0"}) {
                    const std::string from = pair;
                    const std::string to = from.substr(0, 3) + "_dec" + from.substr(3);
                    const size_t at = candidate.find(from);
                    if (at != std::string::npos) {
                        candidate.replace(at, from.size(), to);
                        swapped = true;
                        break;
                    }
                }
                const size_t dot = candidate.rfind('.');
                if (swapped && dot != std::string::npos) {
                    candidate.replace(dot, std::string::npos, ".elf");
                    std::error_code size_ec;
                    const auto elf_size = fs::file_size(candidate, size_ec);
                    if (!size_ec && elf_size != 0) {
                        child.path = candidate;
                        child.size = elf_size;
                    }
                }
            }
            const u64 needed =
                (child.size + cluster_bytes - 1) / static_cast<u64>(cluster_bytes);
            u32 first = 0;
            if (!allocate(needed, first)) {
                out.message = format("out of clusters while writing '%s'", child.name.c_str());
                return false;
            }
            child.first_cluster = first;
            child.clusters = static_cast<u32>(needed);

            std::FILE* file = std::fopen(child.path.c_str(), "rb");
            if (!file) {
                out.message = format("cannot open %s", child.path.c_str());
                return false;
            }
            u64 remaining = child.size;
            u64 offset = file_cursor;
            while (remaining > 0) {
                const size_t chunk =
                    static_cast<size_t>(remaining > buffer.size() ? buffer.size() : remaining);
                if (std::fread(buffer.data(), 1, chunk, file) != chunk) {
                    std::fclose(file);
                    out.message = format("short read on %s", child.path.c_str());
                    return false;
                }
                if (!writer.write_at(offset, buffer.data(), chunk)) {
                    std::fclose(file);
                    out.message = format("write failed for %s (image too small?)", child.name.c_str());
                    return false;
                }
                offset += chunk;
                remaining -= chunk;
            }
            std::fclose(file);
            files_bytes_written += child.size;
            file_cursor += needed * static_cast<u64>(cluster_bytes);
        }
        return true;
    };
    if (!write_files(root)) return false;
    (void)files_bytes_written;

    const u32 free_clusters = cluster_count + 2 - next_free;

    // ---- boot sector and FSInfo -----------------------------------------
    std::vector<u8> boot(bps, 0);
    boot[0] = 0xEB;
    boot[1] = 0xFE;
    boot[2] = 0x90;
    pad_field(boot, 3, 8, geometry.oem_name);  // OEM name, space padded like the console's
    write_le16(boot.data() + 11, static_cast<u16>(bps));
    boot[13] = static_cast<u8>(spc);
    write_le16(boot.data() + 14, static_cast<u16>(reserved));
    boot[16] = static_cast<u8>(fat_count);
    write_le16(boot.data() + 17, static_cast<u16>(root_entries));
    if (total_sectors < 0x10000) {
        write_le16(boot.data() + 19, static_cast<u16>(total_sectors));
    } else {
        write_le32(boot.data() + 32, static_cast<u32>(total_sectors));
    }
    boot[21] = static_cast<u8>(geometry.media_descriptor ? geometry.media_descriptor : 0xF8);
    write_le16(boot.data() + 22, static_cast<u16>(sectors_per_fat));
    // 63 sectors / 255 heads: what the console's own 1.04 volumes carry.
    write_le16(boot.data() + 24, 63);
    write_le16(boot.data() + 26, 255);
    write_le32(boot.data() + 28, 0);     // hidden sectors
    boot[36] = 0x80;                     // drive number
    boot[38] = 0x29;                     // extended boot signature
    write_le32(boot.data() + 39, geometry.volume_id);
    pad_field(boot, 43, 11, geometry.volume_label);  // volume label, space padded
    std::memcpy(boot.data() + 54, "FAT16   ", 8);
    write_le16(boot.data() + 510, 0xAA55);
    if (!writer.write_sector(0, boot.data())) {
        out.message = "failed to write the boot sector";
        return false;
    }

    if (reserved > 1) {
        std::vector<u8> fsinfo(bps, 0);
        write_le32(fsinfo.data() + 0, 0x41615252);
        write_le32(fsinfo.data() + 484, 0x61417272);
        write_le32(fsinfo.data() + 488, free_clusters);
        write_le32(fsinfo.data() + 492, next_free);
        write_le16(fsinfo.data() + 510, 0xAA55);
        if (!writer.write_sector(1, fsinfo.data())) {
            out.message = "failed to write the FSInfo sector";
            return false;
        }
        std::vector<u8> blank(bps, 0);
        for (u32 sector = 2; sector < reserved; ++sector) {
            if (!writer.write_sector(sector, blank.data())) {
                out.message = "failed to clear the reserved area";
                return false;
            }
        }
    }

    for (u32 copy = 0; copy < fat_count; ++copy) {
        const u64 base = (reserved + static_cast<u64>(copy) * sectors_per_fat) * bps;
        if (!writer.write_at(base, fat.data(), fat.size())) {
            out.message = "failed to write a FAT copy";
            return false;
        }
    }

    // ---- directory contents ---------------------------------------------
    std::function<bool(TreeNode&, u32, bool)> write_directory =
        [&](TreeNode& node, u32 parent, bool is_root) -> bool {
        const u64 slots = directory_slots(node, is_root);
        const u64 bytes = std::max<u64>(slots * kDirEntrySize, is_root ? 0 : kDirEntrySize);
        std::vector<u8> content;
        content.reserve(static_cast<size_t>(bytes));

        if (!is_root) {
            u8 dot[kDirEntrySize];
            u8 dotdot[kDirEntrySize];
            fill_entry(dot, ".          ", 0x10, node.first_cluster, 0);
            fill_entry(dotdot, "..         ", 0x10, parent, 0);
            content.insert(content.end(), dot, dot + kDirEntrySize);
            content.insert(content.end(), dotdot, dotdot + kDirEntrySize);
        }
        for (const auto& child : node.children) {
            if (!child.lfn_utf16.empty()) emit_lfn(content, child.lfn_utf16,
                                                   sfn_checksum(child.short_name));
            u8 entry[kDirEntrySize];
            fill_entry(entry, child.short_name, child.attributes, child.first_cluster,
                       child.directory ? 0 : child.size);
            content.insert(content.end(), entry, entry + kDirEntrySize);        }

        const u64 capacity = is_root
                                 ? static_cast<u64>(root_dir_sectors) * bps
                                 : static_cast<u64>(std::max<u32>(node.clusters, 1)) * cluster_bytes;
        if (content.size() > capacity) {
            out.message = format("directory '%s' does not fit its allocation", node.name.c_str());
            return false;
        }
        content.resize(static_cast<size_t>(capacity), 0);

        u64 sector = is_root
                         ? static_cast<u64>(reserved) + static_cast<u64>(fat_count) * sectors_per_fat
                         : static_cast<u64>(first_data_sector) +
                               static_cast<u64>(node.first_cluster - 2) * spc;
        u64 written = 0;
        while (written < content.size()) {
            const size_t chunk = std::min<size_t>(bps, static_cast<size_t>(content.size() - written));
            std::vector<u8> sector_buffer(bps, 0);
            std::memcpy(sector_buffer.data(), content.data() + written, chunk);
            if (!writer.write_sector(sector, sector_buffer.data())) {
                out.message = "failed to write a directory sector (image too small?)";
                return false;
            }
            ++sector;
            written += chunk;
        }

        for (auto& child : node.children) {
            if (!child.directory) continue;
            if (!write_directory(child, node.first_cluster, false)) return false;
        }
        return true;
    };

    if (!write_directory(root, 0, true)) return false;
    if (!card.flush()) {
        out.message = "flush failed";
        return false;
    }

    out.ok = true;
    out.message = verbose ? format("built %s from %s", geometry.volume_label.c_str(),
                                   source_root.c_str())
                          : "ok";
    out.bytes_per_sector = bps;
    out.sectors_per_cluster = spc;
    out.cluster_bytes = cluster_bytes;
    out.reserved_sectors = reserved;
    out.fat_count = fat_count;
    out.sectors_per_fat = sectors_per_fat;
    out.root_entry_count = root_entries;
    out.total_sectors = static_cast<u32>(total_sectors);
    out.cluster_count = cluster_count;
    out.free_clusters = free_clusters;
    out.data_start_sector = first_data_sector;
    out.data_sectors = cluster_count * spc;
    out.oem_name = geometry.oem_name;
    out.volume_label = geometry.volume_label;
    out.volume_id = geometry.volume_id;
    out.template_note = geometry.message;
    out.clusters_used = next_free - 2;
    out.bytes_used = static_cast<u64>(out.clusters_used) * cluster_bytes;
    return true;
}

// ---------------------------------------------------------------------------
// Reader / verifier
// ---------------------------------------------------------------------------

bool mount_fat_volume(EmmcCard& card, EmmcPartition partition, u64 start_block, u64 blocks,
                      FatVolumeStats& out) {
    out = FatVolumeStats{};
    VolumeReader reader;
    reader.card = &card;
    reader.partition = partition;
    reader.start_block = start_block;
    reader.blocks = blocks;
    reader.sector_size = 512;

    MountedFat volume;
    if (!mount_volume(reader, volume)) {
        out.message = volume.message;
        return false;
    }

    std::vector<FatDirEntry> entries;
    std::string message;
    if (!list_fat_volume(card, partition, start_block, blocks, entries, message)) {
        out.message = message;
        return false;
    }

    out.ok = true;
    out.message = "ok";
    out.bytes_per_sector = volume.bps;
    out.sectors_per_cluster = volume.spc;
    out.cluster_bytes = volume.spc * volume.bps;
    out.reserved_sectors = volume.reserved;
    out.fat_count = volume.fat_count;
    out.sectors_per_fat = volume.spf;
    out.root_entry_count = volume.root_entries;
    out.total_sectors = volume.total_sectors;
    out.cluster_count = volume.cluster_count;
    out.free_clusters = volume.free_clusters;
    out.data_start_sector = volume.first_data_sector;
    out.data_sectors = volume.data_sectors;
    out.oem_name = volume.oem;
    out.volume_label = volume.label;
    out.volume_id = volume.volume_id;

    u32 root_used = 0;
    for (const auto& entry : entries) {
        const size_t slash = entry.path.find('/');
        if (slash == std::string::npos && entry.directory) ++root_used;
        if (entry.directory) {
            ++out.directories;
        } else {
            ++out.files;
            out.data_bytes += entry.size;
            out.largest_file = std::max(out.largest_file, entry.size);
        }
    }
    out.root_entries_used = static_cast<u32>(root_used);
    out.root_entries_free =
        volume.root_entries > root_used ? volume.root_entries - root_used : 0;
    return true;
}

bool list_fat_volume(EmmcCard& card, EmmcPartition partition, u64 start_block, u64 blocks,
                     std::vector<FatDirEntry>& out, std::string& message) {
    out.clear();
    message.clear();

    VolumeReader reader;
    reader.card = &card;
    reader.partition = partition;
    reader.start_block = start_block;
    reader.blocks = blocks;
    reader.sector_size = 512;

    MountedFat volume;
    if (!mount_volume(reader, volume)) {
        message = volume.message;
        return false;
    }

    std::set<u32> visited;
    std::function<bool(const std::string&, u32, bool, size_t)> walk =
        [&](const std::string& prefix, u32 first_cluster, bool is_root, size_t depth) -> bool {
        if (depth > 32) return true;
        if (!is_root && !visited.insert(first_cluster).second) return true;  // cycle guard

        const std::vector<u8> data = read_directory_data(volume, first_cluster, is_root);
        if (data.empty() && !is_root) return true;

        for (const RawDirEntry& entry : parse_directory(data)) {
            if (entry.end) break;
            if (entry.attributes & 0x08) continue;  // volume label
            if (entry.name == "." || entry.name == "..") continue;
            FatDirEntry item;
            item.path = prefix + entry.name;
            item.directory = (entry.attributes & 0x10) != 0;
            item.size = item.directory ? 0 : entry.size;
            item.first_cluster = entry.first_cluster;
            if (item.directory) {
                item.path.push_back('/');
                out.push_back(item);
                if (!walk(item.path, entry.first_cluster, false, depth + 1)) return false;
            } else {
                out.push_back(item);
            }
        }
        return true;
    };

    if (!walk("", 0, true, 0)) {
        message = "directory walk failed";
        return false;
    }
    return true;
}

bool verify_fat_volume(EmmcCard& card, EmmcPartition partition, u64 start_block, u64 blocks,
                       const std::string& source_root, u64& checked, u64& mismatches,
                       std::vector<std::string>& problems) {
    checked = 0;
    mismatches = 0;

    VolumeReader reader;
    reader.card = &card;
    reader.partition = partition;
    reader.start_block = start_block;
    reader.blocks = blocks;
    reader.sector_size = 512;

    MountedFat volume;
    if (!mount_volume(reader, volume)) {
        problems.push_back(format("mount failed: %s", volume.message.c_str()));
        return false;
    }

    // Map the volume contents by 8.3-equivalent path so the comparison does not
    // depend on how long file names were rendered.
    std::map<std::string, RawDirEntry> by_path;
    std::set<u32> visited;
    std::function<void(const std::string&, u32, bool, size_t)> index =
        [&](const std::string& prefix, u32 first_cluster, bool is_root, size_t depth) {
        if (depth > 32) return;
        if (!is_root && !visited.insert(first_cluster).second) return;
        const std::vector<u8> data = read_directory_data(volume, first_cluster, is_root);
        for (const RawDirEntry& entry : parse_directory(data)) {
            if (entry.end) break;
            if (entry.attributes & 0x08) continue;
            if (entry.name == "." || entry.name == "..") continue;
            const std::string path = prefix + entry.name;
            if (entry.attributes & 0x10) {
                index(path + "/", entry.first_cluster, false, depth + 1);
            } else {
                by_path[to_lower(path)] = entry;
                by_path[to_lower(prefix + entry.short_name)] = entry;
            }
        }
    };
    index("", 0, true, 0);

    std::error_code ec;
    if (!fs::is_directory(source_root, ec)) {
        problems.push_back(format("source tree '%s' missing", source_root.c_str()));
        return false;
    }
    for (const auto& entry : fs::recursive_directory_iterator(source_root, ec)) {
        std::error_code type_ec;
        if (!entry.is_regular_file(type_ec) || type_ec) continue;
        const std::string relative = fs::relative(entry.path(), source_root, ec).generic_string();
        if (ec) continue;
        ++checked;

        const auto hit = by_path.find(to_lower(relative));
        if (hit == by_path.end()) {
            ++mismatches;
            if (problems.size() < 40) {
                problems.push_back(format("missing in image: %s", relative.c_str()));
            }
            continue;
        }
        const u64 expected = entry.file_size(type_ec);
        if (hit->second.size != expected) {
            ++mismatches;
            if (problems.size() < 40) {
                problems.push_back(format("wrong size for %s: image=%llu tree=%llu", relative.c_str(),
                                          static_cast<unsigned long long>(hit->second.size),
                                          static_cast<unsigned long long>(expected)));
            }
            continue;
        }
        if (expected > 0) {
            const std::vector<u8> data = read_chain(volume, hit->second.first_cluster, expected);
            if (data.size() != expected) {
                ++mismatches;
                if (problems.size() < 40) {
                    problems.push_back(format("short cluster chain for %s (%llu of %llu bytes)",
                                              relative.c_str(),
                                              static_cast<unsigned long long>(data.size()),
                                              static_cast<unsigned long long>(expected)));
                }
                continue;
            }
            std::FILE* file = std::fopen(entry.path().string().c_str(), "rb");
            if (!file) {
                ++mismatches;
                if (problems.size() < 40) {
                    problems.push_back(format("cannot reopen %s", relative.c_str()));
                }
                continue;
            }
            std::vector<u8> reference(static_cast<size_t>(expected));
            const size_t got = std::fread(reference.data(), 1, reference.size(), file);
            std::fclose(file);
            if (got != reference.size() || std::memcmp(reference.data(), data.data(), reference.size()) != 0) {
                ++mismatches;
                if (problems.size() < 40) {
                    problems.push_back(format("content differs for %s", relative.c_str()));
                }
            }
        }
    }
    return mismatches == 0;
}

}  // namespace emmc
}  // namespace zlb
