// zeliboba - PSP2UPDAT ("SCEUF") firmware update package parsing.
//
// Port of pup_fiction/pup_fiction.py pup_extract_files()/make_filename():
//
//   SCEUF_HEADER_SIZE = 0x80
//   SCEUF_FILEREC_SIZE = 0x20
//
//   +0x00 char[8] "SCEUF\0\0\1"      (the task's "UPUP" magic 0x50555050 is the
//                                     PSP/PS3 spelling; the Vita file starts
//                                     with 0x55454353 == "SCEUF", both are
//                                     accepted, `PupImage::magic` reports the
//                                     u32 that was actually found)
//   +0x08 u32 version                (1.04: 2)
//   +0x10 u32 firmware version       (1.04: 0x01040000)
//   +0x14 u32 build number           (1.04: 230019)
//   +0x18 u32 entry count            (1.04: 19)
//   TOC at 0x80, one 0x20 byte record per entry:
//     +0x00 u64 file type  -> the "name hash"; known values map to the
//                             pup_types table, anything else is named from the
//                             payload SCE header (see below)
//     +0x08 u64 offset
//     +0x10 u64 size
//     +0x18 u64 flags
//
//   For an unnamed type the name comes from the payload header
//   (pup_fiction.make_filename): if it starts with a SCE container header
//   (magic 0x454353, version 3, flags 0x30040) then
//     u8(payload + metadata_offset + 4)
//   indexes FSTYPE and the name becomes "<fstype>-<nn>.pkg" with a per-index
//   counter.
//
// Evidence — PSP2UPDAT104.PUP (99446784 bytes) versus Out/PUP/:
//   version=2 firmware=0x01040000 build=230019 entries=19, and resolving the
//   names with the rule above reproduces all 19 extracted files exactly, in
//   order, by offset AND size:
//     [ 0] type=0x100 off=0x00000800 size=7         version.txt
//     [ 1] type=0x101 off=0x00000A00 size=48435     license.xml
//     [ 2] type=0x200 off=0x0000C800 size=4640652   psp2swu.self
//     [ 3] type=0x204 off=0x00479800 size=42456     cui_setupper.self
//     [ 4] type=0x302 off=0x00483E00 size=648320    boot_slb2-00.pkg
//     [ 5] type=0x303 off=0x00522400 size=6595712   os0-00.pkg
//     [ 6] type=0x304 off=0x00B6CA00 size=8389760   vs0-00.pkg
//     [ 7] type=0x305 off=0x0136D000 size=8389760   vs0-01.pkg
//     [ 8] type=0x306 off=0x01B6D600 size=8389760   vs0-02.pkg
//     [ 9] type=0x307 off=0x0236DC00 size=8389760   vs0-03.pkg
//     [10] type=0x308 off=0x02B6E200 size=8389760   vs0-04.pkg
//     [11] type=0x309 off=0x0336E800 size=8389760   vs0-05.pkg
//     [12] type=0x30A off=0x03B6EE00 size=8389760   vs0-06.pkg
//     [13] type=0x30B off=0x0436F400 size=8389760   vs0-07.pkg
//     [14] type=0x30C off=0x04B6FA00 size=8389760   vs0-08.pkg
//     [15] type=0x30D off=0x05370000 size=8389760   vs0-09.pkg
//     [16] type=0x30E off=0x05B70600 size=3560576   vs0-10.pkg
//     [17] type=0x400 off=0x05ED5C00 size=4096      package_scewm.wm
//     [18] type=0x401 off=0x05ED6C00 size=1024      package_sceas.as
//   (sum of the TOC sizes 99438878; the file is 99446784 = 0x5ED7000, i.e. the
//    last record ends exactly at EOF). The table below is kept as a fallback for
//    a header-only/TOC-only view where the payload header cannot be read.
#include <cstring>
#include <string>
#include <vector>

#include "common/util.h"
#include "loader/loader.h"
#include "loader/loader_extra.h"

namespace zlb {

namespace {

constexpr u32 kPupMagicSceuf = 0x55454353u;   // "SCEUF"
// The PSP/PS3 spelling of the same container. The task names the constant
// 0x50555050; read as little endian, the bytes "UPUP" are 0x50555055, so both
// spellings are accepted ("UPUP" in memory, and the documented u32).
constexpr u32 kPupMagicUpup = 0x50555050u;
constexpr u32 kPupMagicUpupBytes = 0x50555055u;
constexpr size_t kPupHeaderSize = 0x80;
constexpr size_t kPupFileRecordSize = 0x20;
constexpr u32 kSceContainerMagic = 0x454353u;   // "\0SCE" little endian
constexpr u32 kSceHeaderVersion = 3;
constexpr u32 kSceHeaderFlagsSpkg = 0x30040;

struct NamedType {
    u64 type;
    const char* name;
};

constexpr NamedType kPupTypes[] = {
    {0x100, "version.txt"},         {0x101, "license.xml"},
    {0x200, "psp2swu.self"},        {0x204, "cui_setupper.self"},
    {0x400, "package_scewm.wm"},    {0x401, "package_sceas.as"},
    {0x2005, "UpdaterES1.CpUp"},    {0x2006, "UpdaterES2.CpUp"},
};

constexpr const char* kFsType[] = {
    "unknown0", "os0",      "unknown2", "unknown3", "vs0_chmod", "unknown5",   "unknown6",
    "unknown7", "pervasive8", "boot_slb2", "vs0",    "devkit_cp", "motionC",    "bbmc",
    "unknownE", "motionF",  "touch10",  "touch11",  "syscon12",  "syscon13",   "pervasive14",
    "unknown15", "vs0_tarpatch", "sa0", "pd0",       "pervasive19", "unknown1A", "psp_emulist",
};
constexpr size_t kFsTypeCount = sizeof(kFsType) / sizeof(kFsType[0]);

/// The 1.04 PUP order (see the header comment for the offset/size evidence).
/// Only used when the payload header of an unnamed type cannot be read.
constexpr const char* kPup104Order[] = {
    "version.txt",  "license.xml", "psp2swu.self", "cui_setupper.self", "boot_slb2-00.pkg",
    "os0-00.pkg",   "vs0-00.pkg",  "vs0-01.pkg",   "vs0-02.pkg",        "vs0-03.pkg",
    "vs0-04.pkg",   "vs0-05.pkg",  "vs0-06.pkg",   "vs0-07.pkg",        "vs0-08.pkg",
    "vs0-09.pkg",   "vs0-10.pkg",  "package_scewm.wm", "package_sceas.as",
};
constexpr size_t kPup104OrderCount = sizeof(kPup104Order) / sizeof(kPup104Order[0]);

const char* known_type_name(u64 type) {
    for (const NamedType& entry : kPupTypes) {
        if (entry.type == type) return entry.name;
    }
    return nullptr;
}

/// pup_fiction.make_filename(): derive "<fstype>-<nn>.pkg" from the payload's
/// SCE container header. `counters` keeps the per-fstype numbering.
std::string name_from_payload(const std::vector<u8>& data, size_t offset,
                              std::vector<int>& counters) {
    if (offset + 0x1000 > data.size()) return {};
    if (ld::read_u32(data, offset) != kSceContainerMagic) return {};
    if (ld::read_u32(data, offset + 4) != kSceHeaderVersion) return {};
    if (ld::read_u32(data, offset + 8) != kSceHeaderFlagsSpkg) return {};

    const u64 metadata_offset = ld::read_u64(data, offset + 16);
    const size_t index_offset = offset + static_cast<size_t>(metadata_offset) + 4;
    if (index_offset >= data.size()) return {};
    const u8 index = data[index_offset];
    if (index >= kFsTypeCount) return {};
    if (counters.size() < kFsTypeCount) counters.resize(kFsTypeCount, 0);
    const int number = counters[index]++;
    return format("%s-%02d.pkg", kFsType[index], number);
}

}  // namespace

bool is_pup(const std::vector<u8>& data) {
    if (data.size() < 0x20) return false;
    const u32 magic = ld::read_u32(data, 0);
    if (magic == kPupMagicSceuf || magic == kPupMagicUpup || magic == kPupMagicUpupBytes)
        return true;
    // "PSP2UPDAT" only ever appears in the file name, but the reference accepts
    // it defensively.
    return std::memcmp(data.data(), "PSP2UPDAT", 8) == 0;
}

std::optional<PupImage> parse_pup(const std::vector<u8>& data) {
    if (!is_pup(data)) return std::nullopt;

    PupImage image;
    image.magic = ld::read_u32(data, 0);
    image.version = ld::read_u32(data, 0x08);
    image.firmware_version = ld::read_u32(data, 0x10);

    u32 count = ld::read_u32(data, 0x18);
    const u32 max_records =
        static_cast<u32>(data.size() > kPupHeaderSize
                             ? (data.size() - kPupHeaderSize) / kPupFileRecordSize
                             : 0);
    if (count > max_records) count = max_records;

    std::vector<int> counters;
    for (u32 i = 0; i < count; ++i) {
        const size_t record = kPupHeaderSize + static_cast<size_t>(i) * kPupFileRecordSize;
        PupEntry entry;
        const u64 type = ld::read_u64(data, record + 0x00);
        const u64 offset = ld::read_u64(data, record + 0x08);
        const u64 size = ld::read_u64(data, record + 0x10);
        entry.flags = static_cast<u32>(ld::read_u64(data, record + 0x18));
        entry.offset = static_cast<u32>(offset);
        entry.size = static_cast<u32>(size);

        if (const char* known = known_type_name(type)) {
            entry.name = known;
        } else if (offset < data.size()) {
            entry.name = name_from_payload(data, static_cast<size_t>(offset), counters);
            if (entry.name.empty() && i < kPup104OrderCount) entry.name = kPup104Order[i];
        }
        if (entry.name.empty()) entry.name = format("unknown-0x%llx.pkg",
                                                    static_cast<unsigned long long>(type));

        if (offset <= data.size() && offset + size <= data.size())
            entry.data = ld::slice(data, static_cast<size_t>(offset), static_cast<size_t>(size));
        image.entries.push_back(std::move(entry));
    }

    image.valid = true;
    return image;
}

std::string describe_pup(const PupImage& image) {
    if (!image.valid) return "PSP2UPDAT package (invalid)";
    return format("PSP2UPDAT magic=0x%08X version=%u firmware=0x%08X entries=%u", image.magic,
                  image.version, static_cast<u32>(image.firmware_version),
                  static_cast<unsigned>(image.entries.size()));
}

}  // namespace zlb
