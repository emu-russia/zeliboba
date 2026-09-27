// zeliboba - ELF32/ELF64 parsing.
//
// This is a straight struct decode of the on-disk headers (never relying on the
// host endianness), plus the Vita specific quirk that e_entry of a *module* is
// not a virtual address but the offset of the SceModuleInfo structure from the
// start of segment 0:
//
//     modinfo_file_offset = segment0.p_offset + e_entry
//
// (verified against real firmware: Out/fs_dec/os0/kd/threadmgr.elf has
// e_type=0xFE04, e_entry=0x27AA0 and "SceKernelThreadMgr" at 0x27B40 =
// 0xA0 + 0x27AA0, where 0xA0 is segment 0's p_offset). For every other image
// e_entry is a normal virtual address; `elf_resolved_entry()` decides which one
// it is by checking whether e_entry falls inside a PT_LOAD segment.
#include <algorithm>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "common/util.h"
#include "loader/loader.h"
#include "loader/loader_extra.h"

namespace zlb {

namespace {

constexpr u16 kElfMachineArm = 0x28;      // EM_ARM
constexpr u16 kElfMachineMep = 0xF00D;    // EM_MEP (Toshiba MeP)
constexpr u16 kElfMachineRl78 = 0xF3;     // EM_RL78

constexpr u32 kPtNull = 0;
constexpr u32 kPtLoad = 1;
constexpr u32 kPtSceRela = 0x60000000;
constexpr u32 kPtSceComment = 0x6FFFFF00;

}  // namespace

bool is_elf(const std::vector<u8>& data) {
    return data.size() >= 16 && data[0] == 0x7F && data[1] == 0x45 && data[2] == 0x4C &&
           data[3] == 0x46;
}

const char* elf_segment_type_name(u32 p_type) {
    switch (p_type) {
        case kPtNull: return "null";
        case kPtLoad: return "load";
        case 2: return "dynamic";
        case 3: return "interp";
        case 4: return "note";
        case 6: return "phdr";
        case 7: return "tls";
        case kPtSceRela: return "sce_rela";
        case kPtSceComment: return "sce_comment";
        default: return "type_other";
    }
}

const char* elf_machine_name(u16 machine) {
    switch (machine) {
        case kElfMachineArm: return "ARM";
        case kElfMachineMep: return "MeP";
        case kElfMachineRl78: return "RL78";
        case 0x03: return "x86";
        case 0x3E: return "x86-64";
        case 0xB7: return "AArch64";
        case 0x14: return "PowerPC";
        default: return "machine_unknown";
    }
}

Arch elf_arch(u16 machine) {
    switch (machine) {
        case kElfMachineArm: return Arch::Arm;
        case kElfMachineMep: return Arch::MeP;
        case kElfMachineRl78: return Arch::Rl78;
        default: return Arch::Unknown;
    }
}

const Segment* elf_first_load(const ElfImage& image) {
    for (const Segment& segment : image.segments) {
        if (segment.name == "load") return &segment;
    }
    return image.segments.empty() ? nullptr : &image.segments.front();
}

u32 elf_resolved_entry(const ElfImage& image) {
    const u32 raw = image.entry;
    for (const Segment& segment : image.segments) {
        if (segment.name != "load" || segment.memsz == 0) continue;
        if (raw >= segment.vaddr && static_cast<u64>(raw) < static_cast<u64>(segment.vaddr) + segment.memsz)
            return raw;
    }
    if (const Segment* segment0 = elf_first_load(image)) return segment0->vaddr + raw;
    return raw;
}

std::optional<ElfImage> parse_elf(const std::vector<u8>& data) {
    if (data.size() < 52) return std::nullopt;
    if (!is_elf(data)) return std::nullopt;

    ElfImage image;
    image.data = data;

    const u8 elf_class = data[4];
    if (elf_class == 1) {
        image.is64 = false;
    } else if (elf_class == 2) {
        image.is64 = true;
    } else {
        return std::nullopt;   // neither ELF32 nor ELF64
    }

    image.type = ld::read_u16(data, 0x10);
    image.machine = ld::read_u16(data, 0x12);

    u64 entry64 = 0;
    u16 ph_entry_size = 0;
    if (!image.is64) {
        entry64 = ld::read_u32(data, 0x18);
        image.phoff = ld::read_u32(data, 0x1C);
        ph_entry_size = ld::read_u16(data, 0x2A);
        image.phnum = ld::read_u16(data, 0x2C);
    } else {
        entry64 = ld::read_u64(data, 0x18);
        image.phoff = ld::read_u32(data, 0x20);   // low 32 bits; Vita images are 32 bit
        ph_entry_size = ld::read_u16(data, 0x36);
        image.phnum = ld::read_u16(data, 0x38);
    }
    image.entry = static_cast<u32>(entry64);

    u32 ph_entry = ph_entry_size;
    if (image.phnum > 0 && ph_entry == 0) ph_entry = image.is64 ? 56u : 32u;

    for (u32 i = 0; i < image.phnum; ++i) {
        const u64 offset = static_cast<u64>(image.phoff) + static_cast<u64>(i) * ph_entry;
        if (offset + ph_entry > data.size() || ph_entry < 32) break;

        Segment segment;
        u32 p_type = 0;
        if (!image.is64) {
            p_type = ld::read_u32(data, static_cast<size_t>(offset) + 0x00);
            segment.offset = ld::read_u32(data, static_cast<size_t>(offset) + 0x04);
            segment.vaddr = ld::read_u32(data, static_cast<size_t>(offset) + 0x08);
            segment.filesz = ld::read_u32(data, static_cast<size_t>(offset) + 0x10);
            segment.memsz = ld::read_u32(data, static_cast<size_t>(offset) + 0x14);
            segment.flags = ld::read_u32(data, static_cast<size_t>(offset) + 0x18);
        } else {
            p_type = ld::read_u32(data, static_cast<size_t>(offset) + 0x00);
            segment.flags = ld::read_u32(data, static_cast<size_t>(offset) + 0x04);
            segment.offset = static_cast<u32>(ld::read_u64(data, static_cast<size_t>(offset) + 0x08));
            segment.vaddr = static_cast<u32>(ld::read_u64(data, static_cast<size_t>(offset) + 0x10));
            segment.filesz = static_cast<u32>(ld::read_u64(data, static_cast<size_t>(offset) + 0x20));
            segment.memsz = static_cast<u32>(ld::read_u64(data, static_cast<size_t>(offset) + 0x28));
        }
        segment.name = elf_segment_type_name(p_type);
        image.segments.push_back(std::move(segment));
    }

    image.valid = true;
    return image;
}

int64_t ElfImage::module_info_offset() const {
    // The raw e_entry is a virtual address when it lands inside a PT_LOAD
    // segment; otherwise it is the SceModuleInfo offset relative to segment 0.
    if (elf_resolved_entry(*this) == entry) return -1;
    const Segment* segment0 = elf_first_load(*this);
    if (segment0 == nullptr) return -1;
    return static_cast<int64_t>(segment0->offset) + static_cast<int64_t>(entry);
}

bool elf_section_headers_sane(const std::vector<u8>& elf) {
    if (elf.size() < 52 || !is_elf(elf)) return false;
    if (elf[4] != 1) return false;   // only the 32 bit layout is checked/supported here
    const u32 shoff = ld::read_u32(elf, 0x20);
    const u16 shentsize = ld::read_u16(elf, 0x2E);
    const u16 shnum = ld::read_u16(elf, 0x30);
    const u16 shstrndx = ld::read_u16(elf, 0x32);
    if (shoff == 0 || shnum == 0 || shentsize != 40) return false;
    if (shstrndx >= shnum) return false;
    if (static_cast<u64>(shoff) + static_cast<u64>(shnum) * shentsize > elf.size()) return false;

    for (u16 i = 0; i < shnum; ++i) {
        const size_t offset = shoff + static_cast<size_t>(i) * shentsize;
        const u32 sh_type = ld::read_u32(elf, offset + 4);
        const u32 sh_offset = ld::read_u32(elf, offset + 16);
        const u32 sh_size = ld::read_u32(elf, offset + 20);
        const u32 sh_entsize = ld::read_u32(elf, offset + 36);
        if (sh_type > 0x15 && sh_type < 0x60000000) return false;   // unknown section type
        if (sh_type == 8 && sh_entsize != 0 && sh_size % sh_entsize != 0) return false;
        if (sh_type != 8 && sh_type != 0 && static_cast<u64>(sh_offset) + sh_size > elf.size())
            return false;
    }
    return true;
}

std::optional<u32> elf_section_offset(const std::vector<u8>& elf, const std::string& name) {
    if (!elf_section_headers_sane(elf)) return std::nullopt;
    const u32 shoff = ld::read_u32(elf, 0x20);
    const u16 shnum = ld::read_u16(elf, 0x30);
    const u16 shstrndx = ld::read_u16(elf, 0x32);

    const u32 names_offset =
        ld::read_u32(elf, shoff + static_cast<size_t>(shstrndx) * 40 + 16);
    for (u16 i = 0; i < shnum; ++i) {
        const size_t offset = shoff + static_cast<size_t>(i) * 40;
        const u32 sh_name = ld::read_u32(elf, offset + 0);
        if (sh_name == 0) continue;
        const std::string section_name =
            ld::read_cstr(elf, static_cast<size_t>(names_offset) + sh_name, 64);
        if (section_name == name) return ld::read_u32(elf, offset + 16);
    }
    return std::nullopt;
}

std::string describe_elf(const ElfImage& image) {
    if (!image.valid) return "ELF (not parsed)";
    u32 low = 0xFFFFFFFFu;
    u64 high = 0;
    int loads = 0;
    for (const Segment& segment : image.segments) {
        if (segment.name != "load" || segment.memsz == 0) continue;
        ++loads;
        low = std::min(low, segment.vaddr);
        high = std::max(high, static_cast<u64>(segment.vaddr) + segment.memsz);
    }
    const u32 resolved = elf_resolved_entry(image);
    std::string text = format("ELF%s %s e_type=0x%04X entry=0x%X", image.is64 ? "64" : "32",
                              elf_machine_name(image.machine), image.type, resolved);
    if (resolved != image.entry)
        text += format(" (module info offset 0x%X from segment 0)", image.entry);
    text += format(" phnum=%u loadaddr=0x%X span=0x%llX", image.phnum, loads > 0 ? low : 0u,
                   static_cast<unsigned long long>(loads > 0 ? high - low : 0));
    return text;
}

}  // namespace zlb
