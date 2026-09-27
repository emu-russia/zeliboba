// zeliboba - eMMC device model and image builder tests.
//
// Covers the card geometry and registers, partition selection and block
// read/write round-trips on a small image, the SLB2 boot container, the master
// block partition table and the FAT16 builder's on-disk structures.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "common/util.h"
#include "hw/emmc/emmc.h"
#include "test_framework.h"

/// ZLB_EXPECT_EQ prints its operands as integers, so strings get their own
/// helper here.
#define ZLB_EXPECT_STREQ(a, b)                                                     \
    do {                                                                           \
        const std::string zlb_sa = (a);                                            \
        const std::string zlb_sb = (b);                                            \
        if (zlb_sa != zlb_sb) {                                                    \
            ::zlb::test::report_failure(__FILE__, __LINE__,                        \
                                        "expected \"" + zlb_sa + "\" == \"" + zlb_sb + "\""); \
        }                                                                          \
    } while (0)

using namespace zlb;

namespace {

namespace fs = std::filesystem;

/// A scratch directory that removes itself, so a failing test never leaves files
/// behind in the source tree.
struct ScratchDir {
    fs::path path;

    explicit ScratchDir(const std::string& name) {
        path = fs::temp_directory_path() / ("zlb_emmc_" + name + "_" + std::to_string(::rand()));
        std::error_code ec;
        fs::remove_all(path, ec);
        fs::create_directories(path, ec);
    }
    ~ScratchDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }

    std::string file(const std::string& name) const { return (path / name).string(); }
};

/// Small card: 8 MiB user area + the fixed 4 MiB boot area + 512 KiB RPMB.
constexpr u64 kSmallUserArea = 8ull * MB;

std::string make_card(const ScratchDir& scratch, const char* name = "card.img") {
    EmmcCard card;
    const bool ok = card.attach(scratch.file(name), true, kSmallUserArea + emmc::kBootAreaSize +
                                                              emmc::kRpmbSize);
    (void)ok;
    return scratch.file(name);
}

}  // namespace

// ---------------------------------------------------------------------------
// Card geometry
// ---------------------------------------------------------------------------

ZLB_TEST(emmc_card_geometry) {
    ScratchDir scratch("geometry");
    EmmcCard card;

    ZLB_EXPECT_FALSE(card.attached());
    ZLB_EXPECT_TRUE(card.attach(scratch.file("card.img"), true, kSmallUserArea + emmc::kBootAreaSize +
                                                                      emmc::kRpmbSize));
    ZLB_EXPECT_TRUE(card.attached());
    ZLB_EXPECT_EQ(card.block_size(), 512u);

    // The file carries the user area, both boot partitions and RPMB.
    ZLB_EXPECT_EQ(card.image_size(), kSmallUserArea + emmc::kBootAreaSize + emmc::kRpmbSize);
    ZLB_EXPECT_EQ(card.capacity_bytes(), kSmallUserArea);
    ZLB_EXPECT_EQ(card.block_count(), kSmallUserArea / 512);
    ZLB_EXPECT_EQ(file_size(scratch.file("card.img")),
                  kSmallUserArea + emmc::kBootAreaSize + emmc::kRpmbSize);

    // The boot partitions are 2 MiB each and RPMB is 512 KiB, exactly as the
    // EXT_CSD describes them.
    u64 offset = 0;
    u64 blocks = 0;
    ZLB_EXPECT_TRUE(card.region_offset(EmmcPartition::User, offset, blocks));
    ZLB_EXPECT_EQ(offset, 0ull);
    ZLB_EXPECT_EQ(blocks, kSmallUserArea / 512);
    ZLB_EXPECT_TRUE(card.region_offset(EmmcPartition::Boot0, offset, blocks));
    ZLB_EXPECT_EQ(offset, kSmallUserArea);
    ZLB_EXPECT_EQ(blocks, 2 * MB / 512);
    ZLB_EXPECT_TRUE(card.region_offset(EmmcPartition::Boot1, offset, blocks));
    ZLB_EXPECT_EQ(offset, kSmallUserArea + 2 * MB);
    ZLB_EXPECT_TRUE(card.region_offset(EmmcPartition::Rpmb, offset, blocks));
    ZLB_EXPECT_EQ(offset, kSmallUserArea + 4 * MB);
    ZLB_EXPECT_EQ(blocks, 512 * KB / 512);

    card.detach();
    ZLB_EXPECT_FALSE(card.attached());

    // A file that is too small to hold the fixed areas must be rejected. The
    // existing file is 12.5 MiB, so make a genuinely tiny one.
    std::vector<u8> stub(1024, 0);
    ZLB_EXPECT_TRUE(write_file(scratch.file("stub.img"), stub));
    EmmcCard tiny;
    ZLB_EXPECT_FALSE(tiny.attach(scratch.file("stub.img"), false));
}

ZLB_TEST(emmc_card_read_only_and_dirty) {
    ScratchDir scratch("readonly");
    const std::string path = scratch.file("card.img");
    ZLB_EXPECT_TRUE(make_card(scratch) == path);

    EmmcCard card;
    ZLB_EXPECT_TRUE(card.attach(path));
    ZLB_EXPECT_FALSE(card.readonly());
    ZLB_EXPECT_FALSE(card.dirty());

    u8 block[512];
    std::memset(block, 0xAB, sizeof(block));
    ZLB_EXPECT_TRUE(card.write_blocks(EmmcPartition::User, 4, 1, block));
    ZLB_EXPECT_TRUE(card.dirty());
    ZLB_EXPECT_TRUE(card.flush());
    ZLB_EXPECT_FALSE(card.dirty());

    card.set_readonly(true);
    ZLB_EXPECT_TRUE(card.readonly());
    ZLB_EXPECT_FALSE(card.write_blocks(EmmcPartition::User, 4, 1, block));
    ZLB_EXPECT_FALSE(card.write_bytes(EmmcPartition::User, 0, block, sizeof(block)));
    ZLB_EXPECT_FALSE(card.erase(0, 1));

    // Reads still work and see the flushed value.
    u8 readback[512];
    std::memset(readback, 0, sizeof(readback));
    ZLB_EXPECT_TRUE(card.read_blocks(EmmcPartition::User, 4, 1, readback));
    ZLB_EXPECT_EQ(readback[0], 0xABu);

    // Out of range requests are refused rather than silently truncated.
    ZLB_EXPECT_FALSE(card.read_blocks(EmmcPartition::User, card.block_count(), 1, readback));
    ZLB_EXPECT_FALSE(card.read_blocks(EmmcPartition::Boot0, 2 * MB / 512, 1, readback));
    ZLB_EXPECT_FALSE(card.read_blocks(EmmcPartition::Rpmb, 512 * KB / 512, 1, readback));
    ZLB_EXPECT_FALSE(card.read_blocks(EmmcPartition::User, 0, 0, readback));
}

ZLB_TEST(emmc_card_partition_selection) {
    ScratchDir scratch("partition");
    EmmcCard card;
    ZLB_EXPECT_TRUE(card.attach(scratch.file("card.img"), true,
                                kSmallUserArea + emmc::kBootAreaSize + emmc::kRpmbSize));

    ZLB_EXPECT_EQ(static_cast<u32>(card.current_partition()), static_cast<u32>(EmmcPartition::User));

    const std::vector<u8> user(512, 0x11);
    const std::vector<u8> boot0(512, 0x22);
    const std::vector<u8> boot1(512, 0x33);
    ZLB_EXPECT_TRUE(card.write_blocks(EmmcPartition::User, 0, 1, user.data()));
    ZLB_EXPECT_TRUE(card.write_blocks(EmmcPartition::Boot0, 0, 1, boot0.data()));
    ZLB_EXPECT_TRUE(card.write_blocks(EmmcPartition::Boot1, 0, 1, boot1.data()));

    // The three areas must not alias: the boot partition lives *behind* the user
    // area in the file, so a bug in the offset maths shows up here.
    u8 probe[512];
    card.select_partition(EmmcPartition::User);
    ZLB_EXPECT_EQ(static_cast<u32>(card.current_partition()), static_cast<u32>(EmmcPartition::User));
    ZLB_EXPECT_TRUE(card.read_blocks(EmmcPartition::Boot0, 0, 1, probe));
    ZLB_EXPECT_EQ(probe[0], 0x22u);
    card.select_partition(EmmcPartition::Boot1);
    ZLB_EXPECT_TRUE(card.read_blocks(EmmcPartition::Boot1, 0, 1, probe));
    ZLB_EXPECT_EQ(probe[0], 0x33u);
    ZLB_EXPECT_TRUE(card.read_blocks(EmmcPartition::User, 0, 1, probe));
    ZLB_EXPECT_EQ(probe[0], 0x11u);

    // EXT_CSD[179] PARTITION_CONFIG follows the selection in its low three bits.
    ZLB_EXPECT_EQ(card.ext_csd()[179] & 0x07u, static_cast<u32>(EmmcPartition::Boot1));
    card.select_partition(EmmcPartition::Rpmb);
    ZLB_EXPECT_EQ(card.ext_csd()[179] & 0x07u, static_cast<u32>(EmmcPartition::Rpmb));
    ZLB_EXPECT_EQ(card.ext_csd()[179] & 0xF8u, 0u);
}

ZLB_TEST(emmc_card_block_round_trip) {
    ScratchDir scratch("roundtrip");
    EmmcCard card;
    ZLB_EXPECT_TRUE(card.attach(scratch.file("card.img"), true,
                                kSmallUserArea + emmc::kBootAreaSize + emmc::kRpmbSize));

    // Deterministic pattern across several blocks.
    std::vector<u8> written(512 * 8);
    for (size_t i = 0; i < written.size(); ++i) {
        written[i] = static_cast<u8>((i * 37 + i / 512) & 0xFF);
    }
    ZLB_EXPECT_TRUE(card.write_blocks(EmmcPartition::User, 100, 8, written.data()));

    std::vector<u8> readback(written.size(), 0);
    ZLB_EXPECT_TRUE(card.read_blocks(EmmcPartition::User, 100, 8, readback.data()));
    ZLB_EXPECT_TRUE(readback == written);

    // Byte granularity reads see the same bytes...
    std::vector<u8> slice(100, 0);
    ZLB_EXPECT_TRUE(card.read_bytes(EmmcPartition::User, 100 * 512 + 400, slice.data(), slice.size()));
    ZLB_EXPECT_EQ(std::memcmp(slice.data(), written.data() + 400, slice.size()), 0);

    // ... and byte granularity writes land where the block reads expect them.
    const std::string marker = "zeliboba-emmc";
    ZLB_EXPECT_TRUE(card.write_bytes(EmmcPartition::User, 105 * 512 + 17, marker.data(), marker.size()));
    ZLB_EXPECT_TRUE(card.read_blocks(EmmcPartition::User, 105, 1, readback.data()));
    ZLB_EXPECT_EQ(std::memcmp(readback.data() + 17, marker.data(), marker.size()), 0);

    // Erase clears whole blocks.
    ZLB_EXPECT_TRUE(card.erase(100, 8));
    ZLB_EXPECT_TRUE(card.read_blocks(EmmcPartition::User, 100, 8, readback.data()));
    bool all_zero = true;
    for (u8 byte : readback) all_zero = all_zero && byte == 0;
    ZLB_EXPECT_TRUE(all_zero);

    // Erase only works on the user area.
    ZLB_EXPECT_FALSE(card.erase(card.block_count() - 1, 2));

    const u64 reads_before = card.reads();
    const u64 writes_before = card.writes();
    ZLB_EXPECT_TRUE(card.read_blocks(EmmcPartition::User, 0, 1, readback.data()));
    ZLB_EXPECT_EQ(card.reads(), reads_before + 1);
    ZLB_EXPECT_TRUE(card.write_blocks(EmmcPartition::User, 0, 1, readback.data()));
    ZLB_EXPECT_EQ(card.writes(), writes_before + 1);
}

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

ZLB_TEST(emmc_card_cid_csd) {
    ScratchDir scratch("registers");
    EmmcCard card;
    ZLB_EXPECT_TRUE(card.attach(scratch.file("card.img"), true,
                                kSmallUserArea + emmc::kBootAreaSize + emmc::kRpmbSize));

    std::vector<std::string> lines;
    card.describe(lines);
    ZLB_EXPECT_TRUE(lines.size() >= 15);

    // CID: manufacturer 0x11 (Toshiba), PNM "THGBM3", CRC7 in [7:1], bit 0 set.
    const auto& cid = card.cid();
    ZLB_EXPECT_EQ(cid[0], 0x11u);
    // cid[1] holds [119:112]: reserved [119:114] in bits 7..2, CBX in bits 1..0.
    ZLB_EXPECT_EQ(cid[1] & 0x3Cu, 0u);           // reserved [119:114]
    ZLB_EXPECT_EQ(cid[1] & 0x03u, 0x01u);        // CBX = BGA, datasheet "01b"
    ZLB_EXPECT_EQ(cid[2], 0x00u);                 // OID
    ZLB_EXPECT_EQ(cid[3], 'T');
    ZLB_EXPECT_EQ(cid[4], 'H');
    ZLB_EXPECT_EQ(cid[5], 'G');
    ZLB_EXPECT_EQ(cid[6], 'B');
    ZLB_EXPECT_EQ(cid[7], 'M');
    ZLB_EXPECT_EQ(cid[8], '3');
    ZLB_EXPECT_EQ(cid[15] & 0x01u, 0x01u);  // "not used, always 1"
    ZLB_EXPECT_NE(cid[15] & 0xFEu, 0u);     // a real CRC7, not zero

    // CSD: version 1.0 (structure 3), 512 byte blocks, CCC 0x0F5 and
    // READ_BL_LEN 0x9 as the datasheet specifies. The exact byte layout of the
    // produced register is pinned here as well.
    const auto& csd = card.csd();
    ZLB_EXPECT_EQ(csd[0] >> 6, 0x03u);              // CSD_STRUCTURE
    ZLB_EXPECT_EQ((csd[0] >> 2) & 0x0Fu, 0x04u);    // SPEC_VERS
    ZLB_EXPECT_EQ(csd[1], 0x0Eu);                   // TAAC
    ZLB_EXPECT_EQ(csd[2], 0x00u);                   // NSAC
    ZLB_EXPECT_EQ(csd[3], 0x32u);                   // TRAN_SPEED 26 MHz
    ZLB_EXPECT_EQ(csd[4], 0x0Fu);                   // CCC [95:88]
    ZLB_EXPECT_EQ(csd[5] >> 4, 0x05u);              // CCC [87:84]
    ZLB_EXPECT_EQ(csd[5] & 0x0Fu, 0x09u);           // READ_BL_LEN = 512 bytes
    ZLB_EXPECT_EQ(csd[15] & 0x01u, 0x01u);
    ZLB_EXPECT_NE(csd[15] & 0xFEu, 0u);

    // C_SIZE must describe the user area: (C_SIZE + 1) * 512 KiB == capacity.
    const u32 c_size = (static_cast<u32>(csd[6] & 0x03) << 10) | (static_cast<u32>(csd[7]) << 2) |
                       (csd[8] >> 6);
    ZLB_EXPECT_EQ((static_cast<u64>(c_size) + 1) * 512 * KB, card.capacity_bytes());

    const auto& ext = card.ext_csd();
    ZLB_EXPECT_EQ(ext.size(), 512u);
    const u32 sec_count = (static_cast<u32>(ext[215]) << 24) | (static_cast<u32>(ext[214]) << 16) |
                          (static_cast<u32>(ext[213]) << 8) | ext[212];
    ZLB_EXPECT_EQ(static_cast<u64>(sec_count) * 512, card.capacity_bytes());
    ZLB_EXPECT_EQ(ext[226], 0x10u);              // BOOT_SIZE_MULTI -> 2 MiB
    ZLB_EXPECT_EQ(static_cast<u32>(ext[226]) * 128 * KB, emmc::kBootPartitionSize);
    ZLB_EXPECT_EQ(ext[168], 0x01u);              // RPMB_SIZE_MULT
    ZLB_EXPECT_EQ(ext[160], 0x03u);              // PARTITIONING_SUPPORT
    ZLB_EXPECT_EQ(ext[152], 0x02u);              // EXT_CSD_REV 1.5.1
    ZLB_EXPECT_EQ(ext[228], 0x07u);              // BOOT_INFO

    // A custom CID must be reflected in the register.
    EmmcCid custom;
    custom.serial = 0xDEADBEEF;
    custom.product_name = "TESTPART";  // only the first six characters fit
    card.set_cid(custom);
    ZLB_EXPECT_EQ(card.cid()[3], 'T');
    ZLB_EXPECT_EQ(card.cid()[4], 'E');
    ZLB_EXPECT_EQ(card.cid()[5], 'S');
    ZLB_EXPECT_EQ(card.cid()[6], 'T');
    ZLB_EXPECT_EQ(card.cid()[7], 'P');
    ZLB_EXPECT_EQ(card.cid()[8], 'A');     // PNM is six bytes; "RT" does not fit
    ZLB_EXPECT_EQ(card.cid()[10], 0xDEu);  // PSN[31:24]
    ZLB_EXPECT_EQ(card.cid()[13], 0xEFu);  // PSN[7:0]
}

// ---------------------------------------------------------------------------
// SLB2 container
// ---------------------------------------------------------------------------

ZLB_TEST(emmc_slb2_container) {
    ScratchDir scratch("slb2");
    const std::vector<u8> first(1500, 0xA5);
    const std::vector<u8> second(700, 0x5A);
    ZLB_EXPECT_TRUE(write_file(scratch.file("second_loader.enp"), first));
    ZLB_EXPECT_TRUE(write_file(scratch.file("kernel_boot_loader.self"), second));

    std::vector<emmc::Slb2File> files;
    files.push_back({"second_loader.enp", scratch.file("second_loader.enp")});
    files.push_back({"kernel_boot_loader.self", scratch.file("kernel_boot_loader.self")});

    const std::vector<u8> container = emmc::build_slb2_container(files, false);
    ZLB_EXPECT_FALSE(container.empty());
    // Table 0x200, first payload at 0x200 (1500 bytes -> ends 0x7DC), second
    // payload aligned to 0x800, running to 0xACC and the container rounded up
    // to 0xC00.
    ZLB_EXPECT_EQ(container.size(), 0xC00u);
    ZLB_EXPECT_EQ(emmc::read_le32(container.data()), 0x32424C53u);
    ZLB_EXPECT_EQ(emmc::read_le32(container.data() + 4), 1u);
    ZLB_EXPECT_EQ(emmc::read_le32(container.data() + 8), 0x200u);
    ZLB_EXPECT_EQ(emmc::read_le32(container.data() + 12), 2u);

    emmc::Slb2View view;
    ZLB_EXPECT_TRUE(emmc::parse_slb2_container(container, view));
    ZLB_EXPECT_EQ(view.count, 2u);
    ZLB_EXPECT_EQ(view.entries.size(), 2u);
    ZLB_EXPECT_STREQ(view.entries[0].name, "second_loader.enp");
    ZLB_EXPECT_EQ(view.entries[0].offset, 0x200ull);
    ZLB_EXPECT_EQ(view.entries[0].size, 1500ull);
    ZLB_EXPECT_STREQ(view.entries[1].name, "kernel_boot_loader.self");
    ZLB_EXPECT_EQ(view.entries[1].offset, 0x800ull);  // 0x200 + align512(1500)
    ZLB_EXPECT_EQ(view.entries[1].size, 700ull);

    // The payloads must round-trip byte for byte.
    ZLB_EXPECT_EQ(std::memcmp(container.data() + 0x200, first.data(), first.size()), 0);
    ZLB_EXPECT_EQ(std::memcmp(container.data() + view.entries[1].offset, second.data(), second.size()), 0);

    // A corrupt magic must be rejected, and a table-only parse must work on a
    // read that stops before the payloads.
    std::vector<u8> broken = container;
    broken[0] = 'X';
    emmc::Slb2View broken_view;
    ZLB_EXPECT_FALSE(emmc::parse_slb2_container(broken, broken_view));

    std::vector<u8> head(container.begin(), container.begin() + 0x200);
    emmc::Slb2View head_view;
    ZLB_EXPECT_TRUE(emmc::parse_slb2_container(head, head_view, false));
    ZLB_EXPECT_EQ(head_view.count, 2u);
    ZLB_EXPECT_FALSE(emmc::parse_slb2_container(head, head_view, true));
}

// ---------------------------------------------------------------------------
// Master block
// ---------------------------------------------------------------------------

ZLB_TEST(emmc_master_block) {
    std::vector<emmc::VitaPartition> table;
    emmc::VitaPartition idstorage;
    idstorage.offset_blocks = 0x200;
    idstorage.size_blocks = 0x400;
    idstorage.code = emmc::VitaPartCode::IdStorage;
    idstorage.type = emmc::VitaPartType::Raw;
    idstorage.flags = 0x001F0F00;
    table.push_back(idstorage);

    emmc::VitaPartition os0;
    os0.offset_blocks = 0x8000;
    os0.size_blocks = 0x8000;
    os0.code = emmc::VitaPartCode::Os0;
    os0.type = emmc::VitaPartType::Fat16;
    os0.active = 1;
    os0.flags = 0x000F0F01;
    table.push_back(os0);

    const std::vector<u8> block = emmc::build_master_block(table, 0x71A000);
    ZLB_EXPECT_EQ(block.size(), 512u);
    ZLB_EXPECT_EQ(std::memcmp(block.data(), emmc::kMasterMagic, 32), 0);
    ZLB_EXPECT_EQ(emmc::read_le32(block.data() + 0x20), 3u);
    ZLB_EXPECT_EQ(emmc::read_le32(block.data() + 0x24), 0x71A000u);
    ZLB_EXPECT_EQ(emmc::read_le16(block.data() + 0x1FE), 0xAA55u);
    // Records are 17 bytes: 4 + 4 + 1 + 1 + 1 + 4 + 2 pad (the last byte 0x0F is
    // a constant seen on the console).
    ZLB_EXPECT_EQ(emmc::read_le32(block.data() + 0x50), 0x200u);
    ZLB_EXPECT_EQ(emmc::read_le32(block.data() + 0x54), 0x400u);
    ZLB_EXPECT_EQ(block[0x58], static_cast<u8>(emmc::VitaPartCode::IdStorage));
    ZLB_EXPECT_EQ(block[0x59], static_cast<u8>(emmc::VitaPartType::Raw));
    ZLB_EXPECT_EQ(block[0x5A], 0u);
    ZLB_EXPECT_EQ(emmc::read_le32(block.data() + 0x5B), 0x001F0F00u);
    ZLB_EXPECT_EQ(block[0x5F], 0x0Fu);
    ZLB_EXPECT_EQ(emmc::read_le32(block.data() + 0x61), 0x8000u);  // next record = 0x50 + 17
    ZLB_EXPECT_EQ(block[0x69], static_cast<u8>(emmc::VitaPartCode::Os0));
    ZLB_EXPECT_EQ(block[0x6A], static_cast<u8>(emmc::VitaPartType::Fat16));
    ZLB_EXPECT_EQ(block[0x6B], 1u);  // os0 active

    std::vector<emmc::VitaPartition> parsed;
    u32 total_blocks = 0;
    std::string message;
    ZLB_EXPECT_TRUE(emmc::parse_master_block(block.data(), block.size(), parsed, total_blocks, message));
    ZLB_EXPECT_EQ(total_blocks, 0x71A000u);
    ZLB_EXPECT_EQ(parsed.size(), 2u);
    ZLB_EXPECT_EQ(parsed[0].offset_blocks, 0x200ull);
    ZLB_EXPECT_EQ(parsed[0].size_blocks, 0x400ull);
    ZLB_EXPECT_EQ(static_cast<u32>(parsed[0].code), static_cast<u32>(emmc::VitaPartCode::IdStorage));
    ZLB_EXPECT_EQ(static_cast<u32>(parsed[1].type), static_cast<u32>(emmc::VitaPartType::Fat16));
    ZLB_EXPECT_EQ(parsed[1].active, 1u);

    std::vector<u8> broken = block;
    broken[0] = 'x';
    ZLB_EXPECT_FALSE(emmc::parse_master_block(broken.data(), broken.size(), parsed, total_blocks, message));
    broken = block;
    emmc::write_le16(broken.data() + 0x1FE, 0);
    ZLB_EXPECT_FALSE(emmc::parse_master_block(broken.data(), broken.size(), parsed, total_blocks, message));
}

ZLB_TEST(emmc_idstorage_image) {
    const std::vector<u8> image = emmc::build_idstorage_image();
    ZLB_EXPECT_EQ(image.size(), 256ull * 512);
    // Leaf 0 must reproduce what the reference dump carries at byte 0x40000:
    // 64 bytes of 0xFFF5, then the u16 leaf index, then 0xFFFF filler.  The CMeP
    // second loader (0x46632) refuses to go on unless the first 32 halfwords of
    // the block it reads there are 0xFFF5, which is what makes the boot chain
    // continue past its first data read.
    for (u32 i = 0; i < 32; ++i) {
        ZLB_EXPECT_EQ(emmc::read_le16(image.data() + i * 2), 0xFFF5u);
    }
    for (u32 i = 0; i < 128; ++i) {
        ZLB_EXPECT_EQ(emmc::read_le16(image.data() + 0x40 + i * 2), i);
    }
    ZLB_EXPECT_EQ(emmc::read_le16(image.data() + 0x140), 0xFFFFu);
    // The short leaf list the dump shows at +0x180: it starts with leaf 0x80,
    // which is the value the loader later searches a decrypted record for.
    ZLB_EXPECT_EQ(emmc::read_le16(image.data() + 0x180), 0x0080u);
    ZLB_EXPECT_EQ(emmc::read_le16(image.data() + 0x182), 0x0100u);
    // Leaves 1..255 are 0xFF, except the SMI leaf the second loader checks: the
    // mapping table lists ID 0x0080 for slot 192, and that slot must carry the
    // documented SMI structure ("SMI\0", version 1, zeroed plaintext area).
    ZLB_EXPECT_EQ(image[512], 0xFFu);
    ZLB_EXPECT_EQ(image[512 * 191 + 17], 0xFFu);
    const u8* smi = image.data() + 512 * 192;
    ZLB_EXPECT_EQ(std::memcmp(smi, "SMI\0", 4), 0);
    ZLB_EXPECT_EQ(emmc::read_le32(smi + 4), 1u);
    for (u32 i = 0x0C; i < 0x80; ++i) ZLB_EXPECT_EQ(smi[i], 0);
}

// ---------------------------------------------------------------------------
// FAT16 builder
// ---------------------------------------------------------------------------

ZLB_TEST(emmc_fat_geometry_read) {
    // The boot sector of a genuine 1.04 os0 volume (as found in the reference
    // dump): 512 B sectors, 8 sectors/cluster, 2 reserved, 2 FATs, 512 root
    // entries, 19 sectors per FAT, 32768 sectors, media 0xF8.
    std::vector<u8> boot(512, 0);
    boot[0] = 0xEB;
    boot[1] = 0xFE;
    boot[2] = 0x90;
    std::memcpy(boot.data() + 3, "SCEI    ", 8);
    emmc::write_le16(boot.data() + 11, 512);
    boot[13] = 8;
    emmc::write_le16(boot.data() + 14, 2);
    boot[16] = 2;
    emmc::write_le16(boot.data() + 17, 512);
    emmc::write_le16(boot.data() + 19, 0x8000);
    boot[21] = 0xF8;
    emmc::write_le16(boot.data() + 22, 19);
    boot[36] = 0x80;
    boot[38] = 0x29;
    emmc::write_le32(boot.data() + 39, 0x3F5A3D45);
    std::memcpy(boot.data() + 43, "NO NAME    ", 11);
    std::memcpy(boot.data() + 54, "FAT16   ", 8);
    emmc::write_le16(boot.data() + 510, 0xAA55);

    emmc::FatGeometry geometry;
    ZLB_EXPECT_TRUE(emmc::read_fat_geometry(boot, geometry));
    ZLB_EXPECT_EQ(geometry.bytes_per_sector, 512u);
    ZLB_EXPECT_EQ(geometry.sectors_per_cluster, 8u);
    ZLB_EXPECT_EQ(geometry.reserved_sectors, 2u);
    ZLB_EXPECT_EQ(geometry.fat_count, 2u);
    ZLB_EXPECT_EQ(geometry.root_entry_count, 512u);
    ZLB_EXPECT_EQ(geometry.sectors_per_fat, 19u);
    ZLB_EXPECT_EQ(geometry.total_sectors, 0x8000u);
    ZLB_EXPECT_EQ(geometry.media_descriptor, 0xF8u);
    ZLB_EXPECT_STREQ(geometry.oem_name, "SCEI");
    ZLB_EXPECT_STREQ(geometry.volume_label, "NO NAME");

    std::vector<u8> broken = boot;
    emmc::write_le16(broken.data() + 510, 0);
    ZLB_EXPECT_FALSE(emmc::read_fat_geometry(broken, geometry));
    ZLB_EXPECT_FALSE(emmc::read_fat_geometry(std::vector<u8>(100), geometry));

    // The built-in default describes the same console geometry.
    const emmc::FatGeometry fallback = emmc::default_fat_geometry();
    ZLB_EXPECT_TRUE(fallback.valid);
    ZLB_EXPECT_EQ(fallback.bytes_per_sector, 512u);
    ZLB_EXPECT_EQ(fallback.sectors_per_cluster, 8u);
    ZLB_EXPECT_EQ(fallback.root_entry_count, 512u);
    ZLB_EXPECT_STREQ(fallback.oem_name, "SCEI");
}

ZLB_TEST(emmc_fat_builder_small_tree) {
    ScratchDir scratch("fat");
    // A tiny tree: two files at the root, one subdirectory with a file, one
    // empty file, one long name that needs LFN entries, and a sizeable file
    // that spans several clusters.
    const fs::path tree = scratch.path / "tree";
    fs::create_directories(tree / "kd");
    ZLB_EXPECT_TRUE(write_file((tree / "psp2config.skprx").string(), std::vector<u8>(5165, 0x42)));
    ZLB_EXPECT_TRUE(write_file((tree / "empty.bin").string(), std::vector<u8>()));
    ZLB_EXPECT_TRUE(write_file((tree / "a_very_long_file_name.skprx").string(),
                               std::vector<u8>(9000, 0x17)));
    ZLB_EXPECT_TRUE(write_file((tree / "kd" / "sysmem.skprx").string(), std::vector<u8>(20000, 0x99)));

    EmmcCard card;
    // FAT16 needs at least 4085 clusters; 16 MiB with 4 KiB clusters works out at
    // 4087, exactly like the console's own os0 partition.
    const u64 user_bytes = 16ull * MB;
    ZLB_EXPECT_TRUE(card.attach(scratch.file("fat.img"), true,
                                user_bytes + emmc::kBootAreaSize + emmc::kRpmbSize));

    emmc::FatGeometry geometry = emmc::default_fat_geometry();
    emmc::FatVolumeStats stats;
    const bool built = emmc::build_fat_volume(card, EmmcPartition::User, 0,
                                              user_bytes / 512, tree.string(), geometry,
                                              false, stats);
    if (!built) std::printf("  build_fat_volume failed: %s\n", stats.message.c_str());
    ZLB_EXPECT_TRUE(built);
    ZLB_EXPECT_TRUE(stats.ok);
    ZLB_EXPECT_EQ(stats.files, 4ull);
    ZLB_EXPECT_EQ(stats.directories, 1ull);
    ZLB_EXPECT_EQ(stats.data_bytes, 5165ull + 0ull + 9000ull + 20000ull);
    ZLB_EXPECT_EQ(stats.bytes_per_sector, 512u);
    ZLB_EXPECT_EQ(stats.sectors_per_cluster, 8u);
    ZLB_EXPECT_EQ(stats.cluster_bytes, 4096u);
    ZLB_EXPECT_TRUE(stats.cluster_count >= 4085 && stats.cluster_count <= 65525);
    ZLB_EXPECT_TRUE(stats.free_clusters > 0);
    // 5 child entries plus the LFN slots for the two names that do not fit 8.3.
    ZLB_EXPECT_EQ(stats.root_entries_used, 9u);

    // ---- on-disk checks -------------------------------------------------
    std::vector<u8> boot(512, 0);
    ZLB_EXPECT_TRUE(card.read_bytes(EmmcPartition::User, 0, boot.data(), boot.size()));
    ZLB_EXPECT_EQ(emmc::read_le16(boot.data() + 510), 0xAA55u);
    ZLB_EXPECT_EQ(boot[0], 0xEBu);
    ZLB_EXPECT_EQ(boot[1], 0xFEu);
    ZLB_EXPECT_EQ(boot[2], 0x90u);
    ZLB_EXPECT_EQ(std::memcmp(boot.data() + 3, "SCEI    ", 8), 0);
    ZLB_EXPECT_EQ(emmc::read_le16(boot.data() + 11), 512u);
    ZLB_EXPECT_EQ(boot[13], 8u);
    ZLB_EXPECT_EQ(emmc::read_le16(boot.data() + 14), 2u);
    ZLB_EXPECT_EQ(boot[16], 2u);
    ZLB_EXPECT_EQ(emmc::read_le16(boot.data() + 17), 512u);
    ZLB_EXPECT_EQ(emmc::read_le16(boot.data() + 19), static_cast<u16>(user_bytes / 512));
    ZLB_EXPECT_EQ(boot[21], 0xF8u);
    ZLB_EXPECT_EQ(emmc::read_le16(boot.data() + 22), static_cast<u16>(stats.sectors_per_fat));
    ZLB_EXPECT_EQ(boot[36], 0x80u);
    ZLB_EXPECT_EQ(boot[38], 0x29u);
    ZLB_EXPECT_EQ(std::memcmp(boot.data() + 54, "FAT16   ", 8), 0);

    // FSInfo in sector 1, both FAT copies begin with the reserved entries.
    std::vector<u8> fsinfo(512, 0);
    ZLB_EXPECT_TRUE(card.read_bytes(EmmcPartition::User, 512, fsinfo.data(), fsinfo.size()));
    ZLB_EXPECT_EQ(emmc::read_le32(fsinfo.data() + 0), 0x41615252u);
    ZLB_EXPECT_EQ(emmc::read_le32(fsinfo.data() + 484), 0x61417272u);
    ZLB_EXPECT_EQ(emmc::read_le32(fsinfo.data() + 488), stats.free_clusters);
    ZLB_EXPECT_EQ(emmc::read_le16(fsinfo.data() + 510), 0xAA55u);

    std::vector<u8> fat0(512, 0);
    std::vector<u8> fat1(512, 0);
    ZLB_EXPECT_TRUE(card.read_bytes(EmmcPartition::User, 2 * 512, fat0.data(), fat0.size()));
    ZLB_EXPECT_TRUE(card.read_bytes(EmmcPartition::User,
                                    (2 + stats.sectors_per_fat) * 512, fat1.data(), fat1.size()));
    ZLB_EXPECT_EQ(emmc::read_le16(fat0.data() + 0), 0xFFF8u);
    ZLB_EXPECT_EQ(emmc::read_le16(fat0.data() + 2), 0xFFFFu);
    ZLB_EXPECT_EQ(std::memcmp(fat0.data(), fat1.data(), fat0.size()), 0);  // mirrored FATs
    ZLB_EXPECT_EQ(emmc::read_le16(fat0.data() + 4), 0xFFFFu);              // cluster 2 = directory

    // The root directory holds the four entries plus the LFN run.
    const u64 root_offset =
        static_cast<u64>(stats.reserved_sectors + stats.fat_count * stats.sectors_per_fat) * 512;
    std::vector<u8> root(512, 0);
    ZLB_EXPECT_TRUE(card.read_bytes(EmmcPartition::User, root_offset, root.data(), root.size()));

    bool saw_kd = false;
    bool saw_config = false;
    bool saw_empty = false;
    bool saw_lfn = false;
    u32 config_cluster = 0;
    u64 config_size = 0;
    u32 kd_cluster = 0;
    for (size_t i = 0; i + 32 <= root.size(); i += 32) {
        const u8* entry = root.data() + i;
        if (entry[0] == 0x00) break;
        if (entry[11] == 0x0F) {
            saw_lfn = true;
            continue;
        }
        const std::string name(reinterpret_cast<const char*>(entry), 8);
        if (name == "KD      ") {
            saw_kd = true;
            ZLB_EXPECT_EQ(entry[11], 0x10u);  // directory
            ZLB_EXPECT_EQ(emmc::read_le32(entry + 28), 0u);
            kd_cluster = emmc::read_le16(entry + 26);
        } else if (name == "PSP2CO~1") {
            saw_config = true;
            ZLB_EXPECT_EQ(entry[11], 0x20u);  // archive
            config_cluster = emmc::read_le16(entry + 26);
            config_size = emmc::read_le32(entry + 28);
        } else if (name == "EMPTY   ") {
            saw_empty = true;
            ZLB_EXPECT_EQ(emmc::read_le16(entry + 26), 0u);  // no cluster for an empty file
            ZLB_EXPECT_EQ(emmc::read_le32(entry + 28), 0u);
        }
    }
    if (!saw_kd || !saw_config || !saw_empty) {
        std::printf("  root scan: kd=%d config=%d empty=%d  used=%u\n", saw_kd ? 1 : 0,
                    saw_config ? 1 : 0, saw_empty ? 1 : 0, stats.root_entries_used);
        for (size_t i = 0; i + 32 <= root.size(); i += 32) {
            const u8* e = root.data() + i;
            if (e[0] == 0x00) { std::printf("  [%02u] end\n", (unsigned)(i / 32)); break; }
            std::printf("  [%02u] %.8s%.3s attr=%02X first=%u size=%u (oem=%.8s)\n",
                        (unsigned)(i / 32), reinterpret_cast<const char*>(e),
                        reinterpret_cast<const char*>(e) + 8, e[11],
                        emmc::read_le16(e + 26), emmc::read_le32(e + 28),
                        reinterpret_cast<const char*>(boot.data() + 3));
        }
    }
    ZLB_EXPECT_TRUE(saw_kd);
    ZLB_EXPECT_TRUE(saw_config);
    ZLB_EXPECT_TRUE(saw_empty);
    ZLB_EXPECT_TRUE(saw_lfn);
    ZLB_EXPECT_EQ(config_size, 5165ull);

    // The 5165 byte file needs two clusters and the chain must end with an EOC.
    ZLB_EXPECT_TRUE(config_cluster >= 2);
    std::vector<u8> fat_entry(2, 0);
    ZLB_EXPECT_TRUE(card.read_bytes(EmmcPartition::User, 2 * 512 + config_cluster * 2,
                                    fat_entry.data(), fat_entry.size()));
    const u16 config_next = emmc::read_le16(fat_entry.data());
    ZLB_EXPECT_EQ(config_next, static_cast<u16>(config_cluster + 1));
    std::vector<u8> config_last(2, 0);
    ZLB_EXPECT_TRUE(card.read_bytes(EmmcPartition::User, 2 * 512 + (config_cluster + 1) * 2,
                                    config_last.data(), config_last.size()));
    ZLB_EXPECT_TRUE(emmc::read_le16(config_last.data()) >= 0xFFF8u);

    // The 20 KiB file in kd must have a real chain: 20000 bytes / 4096 = 5 clusters.
    std::vector<u8> kd_dir(4096, 0);
    ZLB_EXPECT_TRUE(card.read_bytes(EmmcPartition::User,
                                    static_cast<u64>(stats.data_start_sector) * 512 +
                                        static_cast<u64>(kd_cluster - 2) * 4096,
                                    kd_dir.data(), kd_dir.size()));
    ZLB_EXPECT_EQ(kd_dir[0], '.');
    ZLB_EXPECT_EQ(kd_dir[32], '.');
    ZLB_EXPECT_EQ(kd_dir[33], '.');
    u32 sysmem_cluster = 0;
    for (size_t i = 64; i + 32 <= kd_dir.size(); i += 32) {
        const u8* entry = kd_dir.data() + i;
        if (entry[0] == 0x00) break;
        if (entry[11] == 0x0F) continue;
        const std::string name(reinterpret_cast<const char*>(entry), 8);
        if (name == "SYSMEM~1") {
            sysmem_cluster = emmc::read_le16(entry + 26);
            ZLB_EXPECT_EQ(emmc::read_le32(entry + 28), 20000u);
        }
    }
    if (sysmem_cluster < 2) {
        std::printf("  kd cluster=%u dir head:", kd_cluster);
        for (size_t i = 0; i < 160; ++i) std::printf(" %02X", kd_dir[i]);
        std::printf("\n");
    }
    ZLB_EXPECT_TRUE(sysmem_cluster >= 2);
    std::vector<u8> chain_entry(2, 0);
    ZLB_EXPECT_TRUE(card.read_bytes(EmmcPartition::User, 2 * 512 + sysmem_cluster * 2,
                                    chain_entry.data(), chain_entry.size()));
    ZLB_EXPECT_EQ(emmc::read_le16(chain_entry.data()), static_cast<u16>(sysmem_cluster + 1));

    // ---- mount it back and walk it --------------------------------------
    emmc::FatVolumeStats mounted;
    ZLB_EXPECT_TRUE(emmc::mount_fat_volume(card, EmmcPartition::User, 0, user_bytes / 512, mounted));
    ZLB_EXPECT_EQ(mounted.files, 4ull);
    ZLB_EXPECT_EQ(mounted.data_bytes, stats.data_bytes);
    ZLB_EXPECT_EQ(mounted.cluster_count, stats.cluster_count);
    ZLB_EXPECT_EQ(mounted.sectors_per_fat, stats.sectors_per_fat);
    ZLB_EXPECT_STREQ(mounted.volume_label, "NO NAME");

    std::vector<emmc::FatDirEntry> entries;
    std::string message;
    ZLB_EXPECT_TRUE(emmc::list_fat_volume(card, EmmcPartition::User, 0, user_bytes / 512, entries, message));
    ZLB_EXPECT_EQ(entries.size(), 5u);  // 4 files + the kd directory
    bool found_kd = false;
    bool found_sysmem = false;
    bool found_long = false;
    bool found_config = false;
    for (const auto& entry : entries) {
        // "kd" only gets long file name entries when its 8.3 form differs from the
        // source name, so compare case insensitively.
        if (entry.directory && to_lower(entry.path) == "kd/") found_kd = true;
        if (to_lower(entry.path) == "kd/sysmem.skprx") {
            found_sysmem = true;
            ZLB_EXPECT_EQ(entry.size, 20000ull);
        }
        if (entry.path == "psp2config.skprx") {
            found_config = true;
            ZLB_EXPECT_EQ(entry.size, 5165ull);
        }
        if (entry.path == "a_very_long_file_name.skprx") {
            found_long = true;
            ZLB_EXPECT_EQ(entry.size, 9000ull);
        }
    }
    ZLB_EXPECT_TRUE(found_kd);
    ZLB_EXPECT_TRUE(found_sysmem);
    ZLB_EXPECT_TRUE(found_config);
    ZLB_EXPECT_TRUE(found_long);

    // ---- full byte comparison against the source tree -------------------
    u64 checked = 0;
    u64 mismatches = 0;
    std::vector<std::string> problems;
    ZLB_EXPECT_TRUE(emmc::verify_fat_volume(card, EmmcPartition::User, 0, user_bytes / 512,
                                            tree.string(), checked, mismatches, problems));
    ZLB_EXPECT_EQ(checked, 4ull);
    ZLB_EXPECT_EQ(mismatches, 0ull);
    ZLB_EXPECT_TRUE(problems.empty());
}

ZLB_TEST(emmc_fat_builder_rejects_bad_input) {
    ScratchDir scratch("fatbad");
    EmmcCard card;
    const u64 user_bytes = 8ull * MB;
    ZLB_EXPECT_TRUE(card.attach(scratch.file("fat.img"), true,
                                user_bytes + emmc::kBootAreaSize + emmc::kRpmbSize));

    emmc::FatGeometry geometry = emmc::default_fat_geometry();
    emmc::FatVolumeStats stats;

    // A missing source tree is an error, not a crash.
    ZLB_EXPECT_FALSE(emmc::build_fat_volume(card, EmmcPartition::User, 0, user_bytes / 512,
                                            scratch.file("nope"), geometry, false, stats));
    ZLB_EXPECT_FALSE(stats.ok);
    ZLB_EXPECT_TRUE(!stats.message.empty());

    // A partition too small to be FAT16 is refused.
    const fs::path tree = scratch.path / "tree";
    fs::create_directories(tree);
    ZLB_EXPECT_TRUE(write_file((tree / "a.bin").string(), std::vector<u8>(16, 1)));
    ZLB_EXPECT_FALSE(emmc::build_fat_volume(card, EmmcPartition::User, 0, 2048, tree.string(),
                                            geometry, false, stats));
    ZLB_EXPECT_FALSE(stats.ok);

    // Mounting something that is not a volume reports a reason.
    emmc::FatVolumeStats mounted;
    ZLB_EXPECT_FALSE(emmc::mount_fat_volume(card, EmmcPartition::User, 4096, 2048, mounted));
    ZLB_EXPECT_FALSE(mounted.ok);
    ZLB_EXPECT_TRUE(!mounted.message.empty());
}

// ---------------------------------------------------------------------------
// Full image build (only when the workspace firmware is present)
// ---------------------------------------------------------------------------

namespace {

/// ZLB_WORKSPACE_DIR is defined by the build system; fall back to a relative
/// path so a hand-compiled test still finds the firmware next to the checkout.
std::string workspace_firmware_root() {
#ifdef ZLB_WORKSPACE_DIR
    const std::string base = ZLB_WORKSPACE_DIR;
#else
    const std::string base = "..";
#endif
    return base + "/Vita_104_Firmware/Out";
}

}  // namespace

ZLB_TEST(emmc_build_and_inspect_round_trip) {
    const std::string firmware = workspace_firmware_root();
    if (!fs::is_directory(firmware)) {
        std::printf("  (skipped: %s not present)\n", firmware.c_str());
        return;
    }

    ScratchDir scratch("image");
    const std::string image = scratch.file("image.img");

    // Smallest image the builder accepts: everything up to the end of vs0.
    const u64 size = emmc::kMinImageBytes;
    const EmmcImagePlan plan = build_emmc_image(firmware, image, false, size);
    ZLB_EXPECT_TRUE(plan.ok);
    ZLB_EXPECT_TRUE(plan.entries.size() >= 10);
    ZLB_EXPECT_EQ(plan.total_size, size + emmc::kBootAreaSize + emmc::kRpmbSize);
    ZLB_EXPECT_EQ(file_size(image), plan.total_size);

    // The plan must name every region we care about.
    auto has_entry = [&](const std::string& name) {
        for (const auto& entry : plan.entries) {
            if (entry.name == name) return true;
        }
        return false;
    };
    ZLB_EXPECT_TRUE(has_entry("master_block"));
    ZLB_EXPECT_TRUE(has_entry("boot0/slb2"));
    ZLB_EXPECT_TRUE(has_entry("boot1/slb2"));
    ZLB_EXPECT_TRUE(has_entry("idstorage"));
    ZLB_EXPECT_TRUE(has_entry("bls0"));
    ZLB_EXPECT_TRUE(has_entry("bls1"));
    ZLB_EXPECT_TRUE(has_entry("os0_0"));
    ZLB_EXPECT_TRUE(has_entry("os0_1"));
    ZLB_EXPECT_TRUE(has_entry("vs0_0"));

    // Inspecting the very same file must describe the same partition table.
    const EmmcImagePlan inspected = inspect_emmc_image(image);
    ZLB_EXPECT_TRUE(inspected.ok);
    ZLB_EXPECT_EQ(inspected.total_size, plan.total_size);
    ZLB_EXPECT_TRUE(inspected.entries.size() >= 8);

    // ... and verifying it must find every file and both SLB2 copies.
    const EmmcVerifyReport report = verify_emmc_image(image, firmware);
    ZLB_EXPECT_TRUE(report.ok);
    ZLB_EXPECT_EQ(report.file_mismatches, 0ull);
    ZLB_EXPECT_EQ(report.files_checked, 992ull);
    ZLB_EXPECT_TRUE(report.slb2_ok);
    ZLB_EXPECT_EQ(report.slb2_entries, 4u);
    ZLB_EXPECT_TRUE(report.problems.empty());
}
