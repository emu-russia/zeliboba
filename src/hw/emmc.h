// zeliboba - eMMC device model.
//
// The console has no eMMC dump, so the card is reconstructed from the firmware
// itself: the boot partitions carry the SLB2 stage (second loader + kernel boot
// loader + secure kernel) and the user area carries an exFAT image with os0/vs0
// taken from the 1.04 PUP.
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/types.h"

namespace zlb {

/// eMMC partitions, as selected by EXT_CSD PARTITION_CONFIG.
enum class EmmcPartition : u32 {
    User = 0,
    Boot0 = 1,
    Boot1 = 2,
    Rpmb = 3,
};

struct EmmcCid {
    u8 manufacturer_id = 0x11;   // Toshiba
    u16 oem_id = 0x0100;
    std::string product_name = "THGBM3G5D1FBAIE";  // 4 GiB part from the datasheet folder
    u8 revision = 0x03;
    u32 serial = 0x5A1B0B00;
    u16 manufacturing_date = 0x0130;  // 2011-03
};

class EmmcCard {
public:
    EmmcCard();
    ~EmmcCard();

    EmmcCard(const EmmcCard&) = delete;
    EmmcCard& operator=(const EmmcCard&) = delete;

    /// Attach an image file. When `create` is true a new image of `size_bytes`
    /// is created if the file does not exist.
    bool attach(const std::string& path, bool create = false, u64 size_bytes = 4ull * GB);
    bool attached() const { return attached_; }
    void detach();
    const std::string& path() const { return path_; }

    u32 block_size() const { return 512; }
    u64 block_count() const;
    u64 capacity_bytes() const;
    bool flush();

    // ------------------------------------------------------------------
    // Bulk transfers (the host controllers call these)
    // ------------------------------------------------------------------

    bool read_blocks(EmmcPartition partition, u64 lba, u32 count, u8* out);
    bool write_blocks(EmmcPartition partition, u64 lba, u32 count, const u8* data);
    bool erase(u64 lba, u32 count);

    /// Byte oriented access used by the image builder. `offset` is a byte offset
    /// from the start of the selected partition.
    bool read_bytes(EmmcPartition partition, u64 offset, void* out, size_t length);
    bool write_bytes(EmmcPartition partition, u64 offset, const void* data, size_t length);

    /// Raw image file access (no partition mapping); used by the tools and the
    /// image builder to reach regions that are not part of the user area.
    u64 image_size() const;
    bool read_image_bytes(u64 file_offset, void* out, size_t length);
    bool write_image_bytes(u64 file_offset, const void* data, size_t length);

    /// File offset and block count of a partition inside the image file. The
    /// image tools use this to translate the partition table's byte offsets.
    bool region_offset(EmmcPartition partition, u64& offset, u64& blocks) const;

    // ------------------------------------------------------------------
    // Card registers
    // ------------------------------------------------------------------

    const std::array<u8, 16>& cid() const { return cid_; }
    const std::array<u8, 16>& csd() const { return csd_; }
    const std::vector<u8>& ext_csd() const { return ext_csd_; }
    void set_cid(const EmmcCid& cid);

    u32 relative_address() const { return rca_; }
    void set_relative_address(u32 rca) { rca_ = rca; }

    EmmcPartition current_partition() const { return partition_; }
    void select_partition(EmmcPartition partition);

    bool readonly() const { return readonly_; }
    void set_readonly(bool value) { readonly_ = value; }

    bool dirty() const { return dirty_; }

    std::string summary() const;
    void describe(std::vector<std::string>& lines) const;

    /// Statistics for the debugger.
    u64 reads() const { return reads_; }
    u64 writes() const { return writes_; }

private:
    void build_registers();

    /// CSD C_SIZE for the current user area size.
    u32 csize() const;

    std::string path_;
    std::FILE* file_ = nullptr;
    bool attached_ = false;
    bool readonly_ = false;
    bool dirty_ = false;

    u64 user_blocks_ = 0;
    u64 boot0_offset_ = 0;
    u64 boot1_offset_ = 0;
    u64 rpmb_offset_ = 0;
    u64 boot_blocks_ = 0;
    u64 rpmb_blocks_ = 0;

    std::array<u8, 16> cid_{};
    std::array<u8, 16> csd_{};
    std::vector<u8> ext_csd_;
    EmmcCid cid_source_{};
    u32 rca_ = 1;
    EmmcPartition partition_ = EmmcPartition::User;
    u64 reads_ = 0;
    u64 writes_ = 0;
};

// ---------------------------------------------------------------------------
// eMMC image reconstruction
// ---------------------------------------------------------------------------

struct EmmcLayoutEntry {
    std::string name;
    EmmcPartition partition;
    u64 offset = 0;
    u64 size = 0;
    std::string source;
};

/// Description of a reconstructed image, returned by the builder so that the
/// debugger and the docs can show exactly what was written where.
struct EmmcImagePlan {
    std::vector<EmmcLayoutEntry> entries;
    u64 total_size = 0;
    bool ok = false;
    std::string message;
    std::vector<std::string> notes;
};

/// Build an eMMC image for firmware 1.04.
///
/// `firmware_root` is the `Vita_104_Firmware/Out` directory: it must contain
/// `SLB2/` (the boot stage), `PUP_dec/` (the real `os0.bin` / `vs0.bin`
/// partition images) and the extracted `fs/` tree.
///
/// By default the two FAT16 partitions are the genuine 1.04 partition images
/// that the PUP ships, laid down verbatim (and zero filled to the end of their
/// slot). Every file those images contain is byte identical to the extracted
/// `fs/` tree, so this is both the most faithful and the fastest option.
///
/// With `from_tree` the volumes are synthesised from `fs/os0` and `fs/vs0`
/// instead, using the geometry read out of the real images.
///
/// `size_bytes` is the size of the *user area* to create; it defaults to the
/// console's real 3.55 GiB. Anything below 432 MiB cannot hold os0 + vs0 and is
/// rejected. Returns a detailed plan of every region written.
EmmcImagePlan build_emmc_image(const std::string& firmware_root, const std::string& output_path,
                               bool verbose = false, u64 size_bytes = 0, bool from_tree = false);

/// Inspect an existing image and describe its partitions.
EmmcImagePlan inspect_emmc_image(const std::string& path);

/// Verification report for a built image (used by `--verify`).
struct EmmcVerifyReport {
    bool ok = false;
    std::string message;
    std::vector<std::string> problems;
    u64 files_checked = 0;
    u64 file_mismatches = 0;
    bool slb2_ok = false;
    std::string slb2_message;
    u32 slb2_entries = 0;
    std::string master_block_message;
    std::vector<std::string> partitions;
};

/// Re-read a built image: master block, SLB2 copies in both boot partitions and
/// both bls copies, and the FAT16 volumes mounted back and compared file by
/// file against `<firmware_root>/fs/os0` and `<firmware_root>/fs/vs0`.
EmmcVerifyReport verify_emmc_image(const std::string& path, const std::string& firmware_root);

/// Human readable name of an eMMC partition ("user", "boot0", ...).
const char* to_string(EmmcPartition partition);

/// Default layout constants (see docs/EMMC.md).
namespace emmc_layout {
constexpr u64 kUserAreaOffset = 0x00000000ull;
constexpr u64 kBoot0Offset = 0x00000000ull;  // inside the boot0 area of the file
constexpr u64 kBootAreaSize = 4ull * MB;     // 2 x 2 MiB boot partitions
constexpr u64 kRpmbSize = 512ull * KB;
constexpr u64 kSlb2Offset = 0x00000000ull;   // start of the SLB2 partition (boot0)
constexpr u32 kSlb2Magic = 0x32424C53;       // "SLB2"
}  // namespace emmc_layout

}  // namespace zlb
