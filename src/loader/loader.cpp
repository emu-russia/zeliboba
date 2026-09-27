// zeliboba - image identification and loading into the bus.
//
// Detection rules (VitaTestSuite/Core/ImageLoader.cs):
//   * 7F 45 4C 46            -> ELF; e_machine 0x28 ARM, 0xF00D MeP, 0xF3 RL78
//   * "SCE\0" (0x00454353)   -> SELF
//   * "SLB2" (0x32424C53)    -> SLB2 container
//   * "SCEUF"/"UPUP"         -> PSP2UPDAT package
//   * 0x64B2C8E5             -> MeP image header (ENP)
//   * otherwise a raw binary, decided from the file name:
//       *bootrom* *first_loader* *secure_kernel* *second_loader* -> MeP boot image
//       USS-* *ernie* *IRT-*                                     -> RL78
//       *.bin whose first words look like ARM                    -> ARM
//       anything else                                            -> Unknown
//
// This file also implements the `ld::` byte helpers declared in loader_extra.h.
#include "loader/loader.h"

#include <algorithm>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "common/log.h"
#include "common/util.h"
#include "loader/loader_extra.h"
#include "loader/nid.h"

namespace zlb {

// ===========================================================================
//  Byte helpers (loader_extra.h, namespace ld)
// ===========================================================================

namespace ld {

bool in_range(const std::vector<u8>& data, size_t offset, size_t length) {
    return offset <= data.size() && length <= data.size() - offset;
}

u8 read_u8(const std::vector<u8>& data, size_t offset) {
    return in_range(data, offset, 1) ? data[offset] : 0;
}

u16 read_u16(const std::vector<u8>& data, size_t offset) {
    if (!in_range(data, offset, 2)) return 0;
    return static_cast<u16>(data[offset] | (static_cast<u16>(data[offset + 1]) << 8));
}

u32 read_u32(const std::vector<u8>& data, size_t offset) {
    if (!in_range(data, offset, 4)) return 0;
    return static_cast<u32>(data[offset]) | (static_cast<u32>(data[offset + 1]) << 8) |
           (static_cast<u32>(data[offset + 2]) << 16) | (static_cast<u32>(data[offset + 3]) << 24);
}

u64 read_u64(const std::vector<u8>& data, size_t offset) {
    return static_cast<u64>(read_u32(data, offset)) |
           (static_cast<u64>(read_u32(data, offset + 4)) << 32);
}

std::string read_cstr(const std::vector<u8>& data, size_t offset, size_t max_length) {
    std::string out;
    if (offset >= data.size()) return out;
    const size_t end = std::min(data.size(), offset + max_length);
    for (size_t i = offset; i < end; ++i) {
        if (data[i] == 0) break;
        out.push_back(static_cast<char>(data[i]));
    }
    return out;
}

std::string hex_bytes(const u8* data, size_t length) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    out.reserve(length * 2);
    for (size_t i = 0; i < length; ++i) {
        out.push_back(digits[data[i] >> 4]);
        out.push_back(digits[data[i] & 0x0F]);
    }
    return out;
}

std::vector<u8> slice(const std::vector<u8>& data, size_t offset, size_t length) {
    if (offset >= data.size()) return {};
    const size_t available = std::min(length, data.size() - offset);
    return std::vector<u8>(data.begin() + static_cast<std::ptrdiff_t>(offset),
                           data.begin() + static_cast<std::ptrdiff_t>(offset + available));
}

}  // namespace ld

// ===========================================================================
//  Identification
// ===========================================================================

const char* to_string(ImageKind kind) {
    switch (kind) {
        case ImageKind::Unknown: return "unknown";
        case ImageKind::RawBinary: return "raw";
        case ImageKind::Elf: return "elf";
        case ImageKind::SceSelf: return "self";
        case ImageKind::Slb2: return "slb2";
        case ImageKind::Pup: return "pup";
        case ImageKind::MePEnp: return "mep-enp";
        case ImageKind::BootImage: return "boot-image";
    }
    return "unknown";
}

namespace {

bool contains_any(const std::string& text, std::initializer_list<const char*> needles) {
    for (const char* needle : needles) {
        if (text.find(needle) != std::string::npos) return true;
    }
    return false;
}

/// ARM opcode words that show up at the start of a real ARM boot image: an LDR
/// from the literal pool (e.g. 18 F0 9F E5), a B/BL or a PUSH {..., lr}.
bool looks_like_arm(const std::vector<u8>& data, std::string& what) {
    if (data.size() < 4) return false;
    const size_t limit = std::min<size_t>(data.size() & ~static_cast<size_t>(3), 0x40);
    for (size_t offset = 0; offset + 4 <= limit; offset += 4) {
        const u32 word = ld::read_u32(data, offset);
        if (word == 0xE59FF018u) {
            what = "18 F0 9F E5 (ldr pc, [pc, #0x18])";
            return true;
        }
        if ((word & 0xFFFFF000u) == 0xE59F0000u) {
            what = format("ldr rX, [pc, #imm] (0x%08X)", word);
            return true;
        }
        if ((word & 0x0F000000u) == 0x0A000000u) {
            what = format("b (0x%08X)", word);
            return true;
        }
        if ((word & 0x0F000000u) == 0x0B000000u) {
            what = format("bl (0x%08X)", word);
            return true;
        }
        if ((word & 0xFFFF0000u) == 0xE92D0000u) {
            what = format("push {...} (0x%08X)", word);
            return true;
        }
    }
    return false;
}

ImageInfo identify_raw(const std::vector<u8>& data, const std::string& name) {
    ImageInfo info;
    info.kind = ImageKind::RawBinary;
    info.size = static_cast<u32>(data.size());
    info.load_address = 0;
    info.entry_point = 0;

    const std::string lower = to_lower(name);
    if (contains_any(lower, {"bootrom", "first_loader", "secure_kernel", "second_loader"})) {
        // The CMeP boot ROM / first loader images live in the CMeP RAM window
        // (ANALYSIS.md 4.1/4.2: the next stage is staged at 0x40000).
        info.kind = ImageKind::BootImage;
        info.arch = Arch::MeP;
        info.load_address = kMepBootWindow;
        info.entry_point = kMepBootWindow;
        info.describe = "raw MeP boot image (name hint), staged at 0x40000";
        return info;
    }
    if (contains_any(lower, {"uss-", "ernie", "irt-"})) {
        info.arch = Arch::Rl78;
        info.describe = "raw RL78 image (name hint: ErnIE syscon)";
        return info;
    }

    std::string pattern;
    if (looks_like_arm(data, pattern)) {
        info.arch = Arch::Arm;
        info.describe = format("raw ARM image (%s)", pattern.c_str());
        return info;
    }
    info.arch = Arch::Unknown;
    info.describe = "raw binary (arch unknown: no name hint and no ARM opcode pattern)";
    return info;
}

ImageInfo identify_elf(const std::vector<u8>& data) {
    ImageInfo info;
    info.kind = ImageKind::Elf;
    info.size = static_cast<u32>(data.size());
    auto image = parse_elf(data);
    if (!image) {
        info.describe = "ELF (header parse failed)";
        return info;
    }
    info.arch = elf_arch(image->machine);
    info.entry_point = elf_resolved_entry(*image);
    u32 low = 0xFFFFFFFFu;
    u64 high = 0;
    int loads = 0;
    for (const Segment& segment : image->segments) {
        if (segment.name != "load" || segment.memsz == 0) continue;
        ++loads;
        low = std::min(low, segment.vaddr);
        high = std::max(high, static_cast<u64>(segment.vaddr) + segment.memsz);
    }
    info.load_address = loads > 0 ? low : 0;
    info.describe = describe_elf(*image);
    return info;
}

ImageInfo identify_self(const std::vector<u8>& data) {
    ImageInfo info;
    info.kind = ImageKind::SceSelf;
    info.size = static_cast<u32>(data.size());
    info.describe = describe_self(data);

    // The inner ELF header and its program header table are in the clear even
    // in an encrypted SELF, so the architecture, the load range and the entry
    // point are known without any key.
    const SceInnerElfInfo inner = sce_inner_elf_info(data);
    if (inner.present) {
        info.arch = elf_arch(inner.machine);
        info.load_address = inner.load_address;
        info.entry_point = inner.entry_resolved;
        info.describe += format(" loadaddr=0x%X span=0x%llX", inner.load_address,
                                static_cast<unsigned long long>(inner.span));
    }
    return info;
}

/// Map every PT_LOAD segment of an ELF image into the bus (auto-mapping RAM)
/// and zero the BSS tail. Returns the entry point or 0 on failure.
bool map_elf_into(Bus& bus, const std::vector<u8>& image, const std::string& tag, u32& entry,
                  std::optional<ElfImage>& out_elf, std::optional<ModuleInfo>& out_module,
                  std::string& error) {
    auto parsed = parse_elf(image);
    if (!parsed) {
        error = "ELF header parse failed";
        return false;
    }

    int index = 0;
    for (const Segment& segment : parsed->segments) {
        if (segment.name != "load" || segment.memsz == 0) continue;
        const std::string segment_tag = format("%s%d", tag.c_str(), index);
        ++index;

        if (segment.filesz > 0) {
            size_t file_size = segment.filesz;
            if (segment.offset >= image.size()) {
                file_size = 0;
            } else if (segment.offset + file_size > image.size()) {
                file_size = image.size() - segment.offset;
            }
            if (file_size > 0) {
                bus.load(segment.vaddr, image.data() + segment.offset, file_size, segment_tag);
            }
        }
        if (segment.memsz > segment.filesz) {
            bus.memset_bytes(segment.vaddr + segment.filesz, 0, segment.memsz - segment.filesz);
        }
    }

    entry = elf_resolved_entry(*parsed);
    out_module = parse_module_info(image);
    out_elf = std::move(parsed);
    return true;
}

}  // namespace

ImageInfo identify(const std::vector<u8>& data, const std::string& name) {
    ImageInfo info;
    if (data.empty()) {
        info.kind = ImageKind::Unknown;
        info.describe = "empty file";
        return info;
    }

    if (is_elf(data)) return identify_elf(data);
    if (is_self(data)) return identify_self(data);
    if (is_pup(data)) {
        info.kind = ImageKind::Pup;
        info.size = static_cast<u32>(data.size());
        auto pup = parse_pup(data);
        info.describe = pup ? describe_pup(*pup) : "PSP2UPDAT package (TOC parse failed)";
        return info;
    }
    if (is_slb2(data)) {
        info.kind = ImageKind::Slb2;
        info.size = static_cast<u32>(data.size());
        auto slb2 = parse_slb2(data);
        info.describe = slb2 ? describe_slb2(*slb2) : "SLB2 container (entry table parse failed)";
        return info;
    }
    if (is_mep_image(data)) {
        MepImageHeader header;
        info.kind = ImageKind::MePEnp;
        info.arch = Arch::MeP;
        info.size = static_cast<u32>(data.size());
        info.load_address = kMepBootWindow;
        info.entry_point = kMepBootWindow;
        info.describe = parse_mep_header(data, header)
                            ? describe_mep_header(header)
                            : std::string("MeP image header (magic 0x64B2C8E5 but the checks fail)");
        return info;
    }
    return identify_raw(data, name);
}

// ===========================================================================
//  Loading
// ===========================================================================

LoadResult load_image(Bus& bus, const std::vector<u8>& data, const std::string& name,
                      const SceKeys& keys, u32 address) {
    LoadResult result;
    result.info = identify(data, name);
    const bool explicit_address = address != kAutoAddress;

    switch (result.info.kind) {
        case ImageKind::Elf: {
            if (explicit_address) {
                bus.load(address, data.data(), data.size(), "elf");
                result.entry = address;
                result.ok = true;
                result.message = format("ELF copied verbatim to 0x%X", address);
                return result;
            }
            std::string error;
            if (!map_elf_into(bus, data, "elf", result.entry, result.elf, result.module, error)) {
                result.message = format("ELF load failed: %s", error.c_str());
                return result;
            }
            result.ok = true;
            result.message = format("loaded %s, entry=0x%X", result.info.describe.c_str(),
                                    result.entry);
            return result;
        }

        case ImageKind::SceSelf: {
            SelfDecryptReport report;
            auto elf = self_to_elf_report(data, keys, {}, report);
            if (!elf) {
                result.message = format("SELF -> ELF failed at stage '%s': %s", report.stage.c_str(),
                                        report.message.c_str());
                ZLB_LOG_WARN("loader", "%s", result.message.c_str());
                return result;
            }
            result.info.plain = *elf;

            if (explicit_address) {
                bus.load(address, elf->data(), elf->size(), "self");
                result.entry = address;
                result.ok = true;
                result.message = format("decrypted SELF (0x%zX bytes) copied verbatim to 0x%X",
                                        elf->size(), address);
                return result;
            }

            std::string error;
            if (!map_elf_into(bus, *elf, "self", result.entry, result.elf, result.module, error)) {
                result.message = format("decrypted SELF does not load: %s", error.c_str());
                return result;
            }
            result.ok = true;
            result.message = format("SELF -> ELF 0x%zX bytes, entry=0x%X", elf->size(),
                                    result.entry);
            return result;
        }

        case ImageKind::MePEnp: {
            MepImageHeader header;
            if (!parse_mep_header(data, header)) {
                result.message = "MeP image header validation failed";
                return result;
            }
            auto body = mep_payload(data, header);
            if (!body) {
                result.message = format("MeP image has no payload (size=0x%X field_0x10=0x%X)",
                                        header.size, header.field_0x10);
                return result;
            }
            const u32 destination = explicit_address ? address : kMepBootWindow;
            bus.load(destination, body->data(), body->size(), "mep");
            result.entry = destination;
            result.ok = true;
            result.info.load_address = destination;
            result.info.entry_point = destination;
            result.message = format("MeP payload 0x%zX bytes loaded at 0x%X", body->size(),
                                    destination);
            return result;
        }

        case ImageKind::BootImage:
        case ImageKind::RawBinary: {
            const u32 destination = explicit_address ? address : result.info.load_address;
            bus.load(destination, data.data(), data.size(), "raw");
            result.entry = destination;
            result.ok = true;
            result.message = format("raw image 0x%zX bytes loaded at 0x%X", data.size(), destination);
            return result;
        }

        case ImageKind::Pup: {
            auto pup = parse_pup(data);
            result.ok = false;
            result.message =
                pup ? format("PSP2UPDAT container with %zu entries: extract them first (parse_pup)",
                             pup->entries.size())
                    : std::string("PSP2UPDAT container: table of contents parse failed");
            return result;
        }

        case ImageKind::Slb2: {
            auto slb2 = parse_slb2(data);
            result.ok = false;
            result.message =
                slb2 ? format("SLB2 container with %zu entries: load one entry instead",
                              slb2->entries.size())
                     : std::string("SLB2 container: entry table parse failed");
            return result;
        }

        case ImageKind::Unknown:
        default:
            result.message = "empty or unrecognised image";
            return result;
    }
}

LoadResult load_file(Bus& bus, const std::string& path, const SceKeys& keys, u32 address) {
    LoadResult result;
    auto data = read_file(path);
    if (!data) {
        result.message = format("cannot read '%s'", path.c_str());
        result.info.describe = result.message;
        return result;
    }
    result = load_image(bus, *data, path, keys, address);
    if (result.message.empty()) result.message = format("loaded '%s'", path.c_str());
    return result;
}

}  // namespace zlb
