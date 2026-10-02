// zeliboba - eMMC device model (Toshiba THGBM3G5D1FBAIE).
//
// The card is a plain file:
//
//     [ user area ][ boot0 2 MiB ][ boot1 2 MiB ][ RPMB 512 KiB ]
//       0x00000000   0xE3400000      0xE3600000     0xE3800000
//
// Blocks are always 512 bytes, offsets are byte offsets into the file and
// nothing is cached in RAM: reads and writes go straight through std::FILE*
// with 64 bit seeks, so a 4 GiB image costs one file cursor, not 4 GiB of RAM.
//
// The register contents come from the datasheet in the workspace
// (datasheets/thgbm3g5d1fbaie32nm4gbe-mmc_e_rev0.3_100917.pdf). Every field
// value carries the CSD/EXT_CSD slice it was taken from in a comment.
#include "hw/emmc/emmc.h"

#include <cstdio>
#include <cstring>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <io.h>
#include <windows.h>
#include <winioctl.h>
#else
#include <unistd.h>
#endif

namespace zlb {

namespace {

const char* partition_name(EmmcPartition partition) {
    switch (partition) {
        case EmmcPartition::User: return "user";
        case EmmcPartition::Boot0: return "boot0";
        case EmmcPartition::Boot1: return "boot1";
        case EmmcPartition::Rpmb: return "rpmb";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// CRC7 (MMC polynomial x^7 + x^3 + 1 = 0x09, MSB first)
// ---------------------------------------------------------------------------

u8 crc7(const u8* data, size_t length) {
    u8 crc = 0;
    for (size_t i = 0; i < length; ++i) {
        const u8 byte = data[i];
        for (int bit = 7; bit >= 0; --bit) {
            crc = static_cast<u8>(crc << 1);
            if (((byte >> bit) & 1) ^ ((crc >> 7) & 1)) crc ^= 0x09;
        }
    }
    return static_cast<u8>((crc >> 1) & 0x3F);
}

/// Write `value` into [hi:lo] of a big-endian 128 bit register. Bit 127 is the
/// most significant bit of byte 0, so bit N lives in byte 15 - N/8 at bit
/// position N%8 counted from the LSB of that byte.
void put_bits(std::array<u8, 16>& reg, unsigned hi, unsigned lo, u64 value) {
    for (unsigned bit = lo; bit <= hi; ++bit) {
        const unsigned index = 15 - (bit / 8);
        const u8 mask = static_cast<u8>(1u << (bit % 8));
        if ((value >> (bit - lo)) & 1ull) {
            reg[index] = static_cast<u8>(reg[index] | mask);
        } else {
            reg[index] = static_cast<u8>(reg[index] & static_cast<u8>(~mask));
        }
    }
}

u64 get_bits(const std::array<u8, 16>& reg, unsigned hi, unsigned lo) {
    u64 value = 0;
    for (unsigned bit = lo; bit <= hi; ++bit) {
        const unsigned index = 15 - (bit / 8);
        value |= static_cast<u64>((reg[index] >> (bit % 8)) & 1u) << (bit - lo);
    }
    return value;
}

bool seek64(std::FILE* file, u64 offset) {
#if defined(_WIN32)
    return _fseeki64(file, static_cast<long long>(offset), SEEK_SET) == 0;
#else
    return fseeko(file, static_cast<off_t>(offset), SEEK_SET) == 0;
#endif
}

u64 tell64(std::FILE* file) {
#if defined(_WIN32)
    const long long position = _ftelli64(file);
#else
    const off_t position = ftello(file);
#endif
    return position < 0 ? 0ull : static_cast<u64>(position);
}

u64 file_length(std::FILE* file) {
    if (!file) return 0;
    const u64 saved = tell64(file);
    if (!seek64(file, 0)) return 0;
#if defined(_WIN32)
    const long long size = _filelengthi64(_fileno(file));
#else
    std::fseek(file, 0, SEEK_END);
    const long size = std::ftell(file);
#endif
    seek64(file, saved);
    return size < 0 ? 0ull : static_cast<u64>(size);
}

/// A 4 GiB card that only holds ~300 MiB of firmware should not cost 4 GiB of
/// disk, so the image is created sparse when the filesystem supports it.
void make_sparse(const std::string& path) {
#if defined(_WIN32)
    HANDLE handle = CreateFileA(path.c_str(), GENERIC_WRITE | GENERIC_READ,
                                FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle == INVALID_HANDLE_VALUE) return;
    DWORD bytes = 0;
    DeviceIoControl(handle, FSCTL_SET_SPARSE, nullptr, 0, nullptr, 0, &bytes, nullptr);
    CloseHandle(handle);
#else
    (void)path;
#endif
}

void resize_file(std::FILE* file, u64 size) {
    if (!seek64(file, size)) return;
    std::fputc(0, file);
    std::fflush(file);
#if defined(_WIN32)
    _chsize_s(_fileno(file), static_cast<long long>(size));
#else
    if (ftruncate(fileno(file), static_cast<off_t>(size)) != 0) {
        // best effort
    }
#endif
}

std::string hex_dump(const u8* data, size_t length) {
    std::string out;
    out.reserve(length * 3);
    for (size_t i = 0; i < length; ++i) {
        if (i) out.push_back(' ');
        out += hex(data[i], 2);
    }
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// EmmcCard
// ---------------------------------------------------------------------------

EmmcCard::EmmcCard() {
    ext_csd_.assign(512, 0);
    build_registers();
}

EmmcCard::~EmmcCard() { detach(); }

bool EmmcCard::attach(const std::string& path, bool create, u64 size_bytes) {
    detach();

    if (create) {
        std::FILE* probe = std::fopen(path.c_str(), "rb");
        if (probe) {
            std::fclose(probe);
        } else {
            std::FILE* maker = std::fopen(path.c_str(), "wb");
            if (!maker) return false;
            std::fclose(maker);
            make_sparse(path);
        }
    }

    // Open read/write when the file allows it so a rebuilt image can be written
    // back (set_readonly() still blocks writes); fall back to read-only for an
    // image that really is write protected.
    file_ = std::fopen(path.c_str(), "r+b");
    if (!file_) file_ = std::fopen(path.c_str(), "rb");
    if (!file_) return false;

    if (create) {
        const u64 existing = file_length(file_);
        if (existing < size_bytes) resize_file(file_, size_bytes);
    }

    const u64 total = file_length(file_);
    const u64 overhead = emmc::kBootAreaSize + emmc::kRpmbSize;
    if (total <= overhead) {
        std::fclose(file_);
        file_ = nullptr;
        return false;
    }

    path_ = path;
    user_blocks_ = (total - overhead) / emmc::kBlockSize;
    boot0_offset_ = user_blocks_ * emmc::kBlockSize;
    boot1_offset_ = boot0_offset_ + emmc::kBootPartitionSize;
    rpmb_offset_ = boot0_offset_ + emmc::kBootAreaSize;
    boot_blocks_ = emmc::kBootPartitionSize / emmc::kBlockSize;
    rpmb_blocks_ = emmc::kRpmbSize / emmc::kBlockSize;

    attached_ = true;
    dirty_ = false;
    reads_ = 0;
    writes_ = 0;
    build_registers();
    return true;
}

void EmmcCard::detach() {
    if (file_) {
        if (dirty_ && !readonly_) flush();
        std::fclose(file_);
        file_ = nullptr;
    }
    attached_ = false;
    dirty_ = false;
    path_.clear();
}

u64 EmmcCard::block_count() const {
    switch (partition_) {
        case EmmcPartition::Boot0:
        case EmmcPartition::Boot1:
            return boot_blocks_;
        case EmmcPartition::Rpmb:
            return rpmb_blocks_;
        case EmmcPartition::User:
        default:
            return user_blocks_;
    }
}

u64 EmmcCard::capacity_bytes() const { return user_blocks_ * emmc::kBlockSize; }

bool EmmcCard::flush() {
    if (!file_) return false;
    if (readonly_) {
        dirty_ = false;
        return true;
    }
    std::fflush(file_);
    dirty_ = false;
    return true;
}

bool EmmcCard::region_offset(EmmcPartition partition, u64& offset, u64& blocks) const {
    switch (partition) {
        case EmmcPartition::User:
            offset = 0;
            blocks = user_blocks_;
            return true;
        case EmmcPartition::Boot0:
            offset = boot0_offset_;
            blocks = boot_blocks_;
            return true;
        case EmmcPartition::Boot1:
            offset = boot1_offset_;
            blocks = boot_blocks_;
            return true;
        case EmmcPartition::Rpmb:
            offset = rpmb_offset_;
            blocks = rpmb_blocks_;
            return true;
    }
    return false;
}

bool EmmcCard::read_blocks(EmmcPartition partition, u64 lba, u32 count, u8* out) {
    if (!file_ || !out || count == 0) return false;

    u64 base = 0;
    u64 blocks = 0;
    if (!region_offset(partition, base, blocks)) return false;
    if (lba >= blocks || (lba + count) > blocks) return false;

    if (!seek64(file_, base + lba * emmc::kBlockSize)) return false;
    const size_t wanted = static_cast<size_t>(count) * emmc::kBlockSize;
    const size_t got = std::fread(out, 1, wanted, file_);
    if (got != wanted) {
        // Past the end of a short image reads as erased flash.
        std::memset(out + got, 0xFF, wanted - got);
    }
    reads_ += count;
    return true;
}

bool EmmcCard::write_blocks(EmmcPartition partition, u64 lba, u32 count, const u8* data) {
    if (!file_ || !data || count == 0 || readonly_) return false;

    u64 base = 0;
    u64 blocks = 0;
    if (!region_offset(partition, base, blocks)) return false;
    if (lba >= blocks || (lba + count) > blocks) return false;

    if (!seek64(file_, base + lba * emmc::kBlockSize)) return false;
    const size_t wanted = static_cast<size_t>(count) * emmc::kBlockSize;
    if (std::fwrite(data, 1, wanted, file_) != wanted) return false;
    writes_ += count;
    dirty_ = true;
    return true;
}

bool EmmcCard::erase(u64 lba, u32 count) {
    if (!file_ || readonly_ || count == 0) return false;
    if (lba >= user_blocks_ || (lba + count) > user_blocks_) return false;

    static const std::vector<u8> erased(emmc::kBlockSize * 64, 0x00);
    u64 remaining = count;
    u64 cursor = lba;
    while (remaining > 0) {
        const u32 chunk = static_cast<u32>(remaining > 64 ? 64 : remaining);
        if (!write_blocks(EmmcPartition::User, cursor, chunk, erased.data())) return false;
        cursor += chunk;
        remaining -= chunk;
    }
    return true;
}

bool EmmcCard::read_bytes(EmmcPartition partition, u64 offset, void* out, size_t length) {
    if (!out) return false;
    if (length == 0) return true;

    u64 base = 0;
    u64 blocks = 0;
    if (!region_offset(partition, base, blocks)) return false;
    if (offset + length > blocks * emmc::kBlockSize) return false;
    return read_image_bytes(base + offset, out, length);
}

bool EmmcCard::write_bytes(EmmcPartition partition, u64 offset, const void* data, size_t length) {
    if (!data) return false;
    if (length == 0) return true;

    u64 base = 0;
    u64 blocks = 0;
    if (!region_offset(partition, base, blocks)) return false;
    if (offset + length > blocks * emmc::kBlockSize) return false;
    return write_image_bytes(base + offset, data, length);
}

u64 EmmcCard::image_size() const { return file_length(file_); }

bool EmmcCard::read_image_bytes(u64 file_offset, void* out, size_t length) {
    if (!file_ || !out) return false;
    if (length == 0) return true;
    if (!seek64(file_, file_offset)) return false;
    const size_t got = std::fread(out, 1, length, file_);
    if (got != length) {
        // Past the end of a short image reads as erased flash.
        std::memset(static_cast<u8*>(out) + got, 0xFF, length - got);
    }
    reads_ += (length + emmc::kBlockSize - 1) / emmc::kBlockSize;
    return true;
}

bool EmmcCard::write_image_bytes(u64 file_offset, const void* data, size_t length) {
    if (!file_ || !data || readonly_) return false;
    if (length == 0) return true;
    if (!seek64(file_, file_offset)) return false;
    if (std::fwrite(data, 1, length, file_) != length) return false;
    writes_ += (length + emmc::kBlockSize - 1) / emmc::kBlockSize;
    dirty_ = true;
    return true;
}

void EmmcCard::select_partition(EmmcPartition partition) {
    partition_ = partition;
    // EXT_CSD[179] PARTITION_CONFIG carries the access bits in [2:0].
    if (ext_csd_.size() >= 180) {
        ext_csd_[179] = static_cast<u8>((ext_csd_[179] & 0xF8) | (static_cast<u32>(partition) & 0x07));
    }
}

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

void EmmcCard::set_cid(const EmmcCid& cid) {
    cid_source_ = cid;
    build_registers();
}

u32 EmmcCard::csize() const {
    // CSD version 1.0: capacity = (C_SIZE + 1) * 512 KiB.
    const u64 user = user_blocks_ ? user_blocks_ * emmc::kBlockSize : emmc::kUserAreaBytes;
    const u64 units = user / (512ull * 1024ull);
    return static_cast<u32>(units ? units - 1 : 0);
}

void EmmcCard::build_registers() {
    cid_.fill(0);

    // CID, big-endian 128 bit (datasheet page 4):
    //   [127:120] MID = 0x11 "0001 0001b"
    //   [119:114] reserved, [113:112] CBX = 01b, [111:104] OID = 0x00
    //   [103:56]  PNM (48 bit ASCII), [55:48] PRV, [47:16] PSN, [15:8] MDT
    //   [7:1] CRC7, [0] always 1
    put_bits(cid_, 127, 120, cid_source_.manufacturer_id);
    put_bits(cid_, 113, 112, 0x1);  // CBX: BGA package
    put_bits(cid_, 111, 104, cid_source_.oem_id & 0xFF);

    std::string name = cid_source_.product_name;
    if (name.size() < 6) name.resize(6, ' ');
    if (name.size() > 6) name.resize(6);
    for (int i = 0; i < 6; ++i) {
        put_bits(cid_, 103 - i * 8, 96 - i * 8, static_cast<u8>(name[static_cast<size_t>(i)]));
    }
    put_bits(cid_, 55, 48, cid_source_.revision);
    put_bits(cid_, 47, 16, cid_source_.serial);
    put_bits(cid_, 15, 8, cid_source_.manufacturing_date);
    put_bits(cid_, 0, 0, 1);
    cid_[15] = static_cast<u8>((crc7(cid_.data(), 15) << 1) | 1);

    // CSD version 1.0 (CSD_STRUCTURE = 3, high capacity), datasheet page 5.
    csd_.fill(0);
    put_bits(csd_, 127, 126, 0x3);    // CSD_STRUCTURE
    put_bits(csd_, 125, 122, 0x4);    // SPEC_VERS
    put_bits(csd_, 119, 112, 0x0E);   // TAAC
    put_bits(csd_, 111, 104, 0x00);   // NSAC
    put_bits(csd_, 103, 96, 0x32);    // TRAN_SPEED 26 MHz
    put_bits(csd_, 95, 84, 0x0F5);    // CCC
    put_bits(csd_, 83, 80, 0x9);      // READ_BL_LEN = 512
    put_bits(csd_, 76, 76, 0x0);      // DSR_IMP
    put_bits(csd_, 73, 62, csize());  // C_SIZE
    put_bits(csd_, 61, 59, 0x7);      // VDD_R_CURR_MIN
    put_bits(csd_, 58, 56, 0x7);      // VDD_R_CURR_MAX
    put_bits(csd_, 55, 53, 0x7);      // VDD_W_CURR_MIN
    put_bits(csd_, 52, 50, 0x7);      // VDD_W_CURR_MAX
    put_bits(csd_, 49, 47, 0x7);      // C_SIZE_MULT (unused for CSD 1.0)
    put_bits(csd_, 46, 42, 0x1F);     // ERASE_GRP_SIZE
    put_bits(csd_, 41, 37, 0x1F);     // ERASE_GRP_MULT
    put_bits(csd_, 36, 32, 0x03);     // WP_GRP_SIZE
    put_bits(csd_, 31, 31, 0x1);      // WP_GRP_ENABLE
    put_bits(csd_, 28, 26, 0x5);      // R2W_FACTOR
    put_bits(csd_, 25, 22, 0x9);      // WRITE_BL_LEN = 512
    put_bits(csd_, 0, 0, 1);
    csd_[15] = static_cast<u8>((crc7(csd_.data(), 15) << 1) | 1);

    ext_csd_.assign(512, 0);
    auto& e = ext_csd_;

    // Every value below is the datasheet's EXT_CSD slice (pages 6-9).
    e[192] = 0x01;  // [192] HPI_FEATURES
    e[194] = 0x00;  // [194] PWR_CL_DDR_52_195
    e[196] = 0x00;  // [196] PWR_CL_52_195
    e[205] = 0x1E;  // [205] MIN_PERF_R_4_26
    e[207] = 0x3C;  // [207] MIN_PERF_R_8_26_4_52
    e[209] = 0x3C;  // [209] MIN_PERF_R_8_52
    e[212] = static_cast<u8>(user_blocks_);         // [215:212] SEC_COUNT
    e[213] = static_cast<u8>(user_blocks_ >> 8);
    e[214] = static_cast<u8>(user_blocks_ >> 16);
    e[215] = static_cast<u8>(user_blocks_ >> 24);
    e[217] = 0x10;  // [217] S_A_TIMEOUT
    e[219] = 0x07;  // [219] S_C_VCCQ
    e[220] = 0x06;  // [220] S_C_VCC
    e[221] = 0x01;  // [221] HC_WP_GRP_SIZE
    e[222] = 0x10;  // [222] REL_WR_SEC_C
    e[223] = 0x02;  // [223] ERASE_TIMEOUT_MULT
    e[224] = 0x04;  // [224] HC_ERASE_GRP_SIZE
    e[225] = 0x06;  // [225] ACC_SIZE
    e[226] = 0x10;  // [226] BOOT_SIZE_MULTI = 16 * 128 KiB = 2 MiB per boot partition
    e[231] = 0x15;  // [231] SEC_FEATURE_SUPPORT
    e[232] = 0x04;  // [232] TRIM_MULT
    e[241] = 0x19;  // [241] INI_TIMEOUT_AP
    e[160] = 0x03;  // [160] PARTITIONING_SUPPORT
    e[156] = 0x03;  // [159:157] MAX_ENH_SIZE_MULT
    e[157] = 0xB1;
    e[158] = 0x00;
    e[166] = 0x05;  // [166] WR_REL_PARAM
    e[168] = 0x01;  // [168] RPMB_SIZE_MULT (1 * 128 KiB)
    e[173] = 0x00;  // [173] BUS_WIDTH (1 bit at power on)
    e[175] = 0x01;  // [175] ERASED_MEM_CONT
    e[176] = 0x00;  // [176] BOOT_CONFIG
    e[177] = 0x01;  // [177] BOOT_BUS_WIDTH
    e[179] = 0x00;  // [179] PARTITION_CONFIG: user area, no boot enable
    e[181] = 0x00;  // [181] ERASE_GROUP_DEF: use ERASE_GRP_SIZE/MULT
    e[182] = 0x1F;  // [182] ERASE_GRP_MULT
    e[183] = 0x1F;  // [183] ERASE_GRP_SIZE
    e[152] = 0x02;  // [152] EXT_CSD_REV: 1.5.1 (what an eMMC 4.41 part reports)
    e[228] = 0x07;  // [228] BOOT_INFO: boot partitions supported, DDR boot
    e[246] = 0x00;  // [246] BKOPS_STATUS
    e[503 - 256] = 0x00;  // [504] S_CMD_SET
    e[502 - 256] = 0x01;  // [502] BKOPS_SUPPORT
}

// ---------------------------------------------------------------------------
// Debugger helpers
// ---------------------------------------------------------------------------

std::string EmmcCard::summary() const {
    if (!attached_) return "eMMC: not attached";
    return format("eMMC %s: %s, %llu blocks, partition=%s%s", path_filename(path_).c_str(),
                  human_size(capacity_bytes()).c_str(), static_cast<unsigned long long>(user_blocks_),
                  partition_name(partition_), readonly_ ? " (read-only)" : "");
}

void EmmcCard::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("file        : %s", path_.empty() ? "(none)" : path_.c_str()));
    lines.push_back(format("attached    : %s", attached_ ? "yes" : "no"));
    lines.push_back(format("user area   : %s (%llu x 512)", human_size(capacity_bytes()).c_str(),
                           static_cast<unsigned long long>(user_blocks_)));
    lines.push_back(format("boot0       : file offset 0x%llX, %s",
                           static_cast<unsigned long long>(boot0_offset_),
                           human_size(emmc::kBootPartitionSize).c_str()));
    lines.push_back(format("boot1       : file offset 0x%llX, %s",
                           static_cast<unsigned long long>(boot1_offset_),
                           human_size(emmc::kBootPartitionSize).c_str()));
    lines.push_back(format("rpmb        : file offset 0x%llX, %s",
                           static_cast<unsigned long long>(rpmb_offset_),
                           human_size(emmc::kRpmbSize).c_str()));
    lines.push_back(format("partition   : %s", partition_name(partition_)));
    lines.push_back(format("rca         : %u", rca_));
    lines.push_back(format("read-only   : %s", readonly_ ? "yes" : "no"));
    lines.push_back(format("dirty       : %s", dirty_ ? "yes" : "no"));
    lines.push_back(format("transfers   : %llu reads, %llu writes", static_cast<unsigned long long>(reads_),
                           static_cast<unsigned long long>(writes_)));
    lines.push_back(format("CID         : %s", hex_dump(cid_.data(), cid_.size()).c_str()));
    lines.push_back(format("CSD         : %s", hex_dump(csd_.data(), csd_.size()).c_str()));
    lines.push_back(format("CSD C_SIZE  : 0x%X (%llu x 512 KiB = %s)", csize(),
                           static_cast<unsigned long long>(csize()) + 1,
                           human_size((static_cast<u64>(csize()) + 1) * 512ull * 1024ull).c_str()));
    lines.push_back(format("CID decoded : MID=%02X OID=%02X PNM=%s PRV=%02X PSN=%08X MDT=%04X",
                           static_cast<unsigned>(get_bits(cid_, 127, 120)),
                           static_cast<unsigned>(get_bits(cid_, 111, 104)),
                           cid_source_.product_name.c_str(),
                           static_cast<unsigned>(get_bits(cid_, 55, 48)),
                           static_cast<unsigned>(get_bits(cid_, 47, 16)),
                           static_cast<unsigned>(get_bits(cid_, 15, 8))));
    lines.push_back(format("EXT_CSD[215:212] SEC_COUNT        : 0x%08X (%llu blocks)",
                           (static_cast<u32>(ext_csd_[215]) << 24) | (static_cast<u32>(ext_csd_[214]) << 16) |
                               (static_cast<u32>(ext_csd_[213]) << 8) | ext_csd_[212],
                           static_cast<unsigned long long>(user_blocks_)));
    lines.push_back(format("EXT_CSD[226] BOOT_SIZE_MULT     : %u (%u KiB per boot partition)",
                           ext_csd_[226], static_cast<unsigned>(ext_csd_[226]) * 128));
    lines.push_back(format("EXT_CSD[168] RPMB_SIZE_MULT     : %u (%u KiB)", ext_csd_[168],
                           static_cast<unsigned>(ext_csd_[168]) * 128));
    lines.push_back(format("EXT_CSD[160] PARTITIONING_SUPPORT: 0x%02X", ext_csd_[160]));
    lines.push_back(format("EXT_CSD[152] EXT_CSD_REV        : 0x%02X", ext_csd_[152]));
    lines.push_back(format("EXT_CSD[179] PARTITION_CONFIG   : 0x%02X", ext_csd_[179]));
}

}  // namespace zlb
