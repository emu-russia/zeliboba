// zeliboba - image identification and loading.
//
// Handles the container formats of the Vita boot chain:
//
//   PUP      PSP2UPDAT.PUP    - the firmware update package (retail, encrypted)
//   SLB2     SLB2             - second loader block, stored in an eMMC partition
//   SELF     *.self / *.suprx - signed ELF wrapper
//   ELF      *.elf / *.skprx  - decrypted modules
//   ENP      second_loader.enp- MeP image header (magic 0x64B2C8E5)
//   raw      everything else
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/types.h"
#include "cpu/cpu.h"  // Arch
#include "loader/keys.h"

namespace zlb {

enum class ImageKind { Unknown, RawBinary, Elf, SceSelf, Slb2, Pup, MePEnp, BootImage };

const char* to_string(ImageKind kind);

struct ImageInfo {
    ImageKind kind = ImageKind::Unknown;
    Arch arch = Arch::Unknown;
    u32 load_address = 0;
    u32 entry_point = 0;
    u32 size = 0;
    std::string describe;

    /// Decrypted payload (SELF -> ELF, PUP entry -> body). Empty when the image
    /// is already plain.
    std::vector<u8> plain;
};

struct Segment {
    u32 vaddr = 0;
    u32 memsz = 0;
    u32 filesz = 0;
    u32 flags = 0;
    u32 offset = 0;
    std::string name;
};

struct ExportEntry {
    u32 address = 0;
    u32 nid = 0;
    u16 library = 0;
    std::string name;  // resolved through the NID database when available
};

struct ImportEntry {
    u32 address = 0;   // slot address in the importing module
    u32 nid = 0;
    u16 library = 0;
    std::string name;
};

struct ModuleInfo {
    std::string name;
    u32 type = 0;
    u32 module_nid = 0;
    u32 entry = 0;
    u32 exports_start = 0;
    u32 exports_end = 0;
    u32 imports_start = 0;
    u32 imports_end = 0;
    std::string library_name;
    std::vector<ExportEntry> exports;
    std::vector<ImportEntry> imports;
};

struct ElfImage {
    bool valid = false;
    bool is64 = false;
    u32 entry = 0;
    u32 phoff = 0;
    u32 phnum = 0;
    u16 machine = 0;
    u16 type = 0;
    std::vector<Segment> segments;
    std::vector<u8> data;

    /// File offset of the SceModuleInfo structure, or -1.
    int64_t module_info_offset() const;
};

/// Result of loading something into a bus.
struct LoadResult {
    bool ok = false;
    std::string message;
    u32 entry = 0;
    ImageInfo info;
    std::optional<ElfImage> elf;
    std::optional<ModuleInfo> module;
};

// ---------------------------------------------------------------------------
// Format parsers
// ---------------------------------------------------------------------------

bool is_elf(const std::vector<u8>& data);
bool is_self(const std::vector<u8>& data);
bool is_pup(const std::vector<u8>& data);
bool is_slb2(const std::vector<u8>& data);

std::optional<ElfImage> parse_elf(const std::vector<u8>& data);

/// Parse a SELF container header (the signature/trailer metadata).
struct SelfHeader {
    u16 platform = 0;
    u16 key_revision = 0;
    u32 header_len = 0;
    u16 self_type = 0;
    u16 self_version = 0;
    u64 sys_version = 0;
    u32 elf_offset = 0;
    u32 elf_size = 0;
    u32 authid = 0;
    u32 vendor_id = 0;
    u32 sce_type = 0;
    std::vector<u8> segment_data;
};
bool parse_self_header(const std::vector<u8>& data, SelfHeader& out);

/// Decrypt the SELF body into an ELF image (uses the SceKeys tables; the
/// hardware keyring can be substituted through `keys`).
std::optional<std::vector<u8>> self_to_elf(const std::vector<u8>& data, const SceKeys& keys);

/// SLB2 container: second_loader.enp + kernel_boot_loader.self (+ extra entries).
struct Slb2Entry {
    std::string name;
    u32 offset = 0;
    u32 size = 0;
    std::vector<u8> data;
};
struct Slb2Image {
    bool valid = false;
    u32 version = 0;
    std::vector<Slb2Entry> entries;
};
std::optional<Slb2Image> parse_slb2(const std::vector<u8>& data);

/// Build an SLB2 container from a list of entries (the eMMC image builder and
/// the round trip tests need the inverse of parse_slb2). Layout, block
/// alignment and the 0xFF fill of the unused space are documented in slb2.cpp;
/// `version` 1, a `total_blocks` of 0 meaning "as many blocks as the payloads
/// need" and `fill` 0xFF reproduce the retail 1.04 container.
/// [ADDED for the eMMC workstream - this is the only addition to loader.h.]
std::vector<u8> build_slb2(const std::vector<Slb2Entry>& entries);

/// PUP package: a list of PKG entries (each an SCE container).
struct PupEntry {
    std::string name;
    u32 offset = 0;
    u32 size = 0;
    u32 flags = 0;
    std::vector<u8> data;
};
struct PupImage {
    bool valid = false;
    u32 magic = 0;
    u32 version = 0;
    u64 firmware_version = 0;
    std::vector<PupEntry> entries;
};
std::optional<PupImage> parse_pup(const std::vector<u8>& data);

/// MeP image header (`second_loader.enp`, magic 0x64B2C8E5).
struct MepImageHeader {
    u32 magic = 0;
    u32 size = 0;
    u32 offset = 0;
    u32 length = 0;
    u32 field_0x10 = 0;
    u16 field_0x16 = 0;
    bool valid = false;
};
bool parse_mep_header(const std::vector<u8>& data, MepImageHeader& out);

// ---------------------------------------------------------------------------
// High level
// ---------------------------------------------------------------------------

ImageInfo identify(const std::vector<u8>& data, const std::string& name);

/// Identify, decrypt and map an image into `bus`. When `address` is not
/// `kAutoAddress` the payload is copied verbatim to that address.
constexpr u32 kAutoAddress = 0xFFFFFFFFu;

LoadResult load_image(Bus& bus, const std::vector<u8>& data, const std::string& name,
                      const SceKeys& keys, u32 address = kAutoAddress);

LoadResult load_file(Bus& bus, const std::string& path, const SceKeys& keys, u32 address = kAutoAddress);

/// Extract the SceModuleInfo export/import tables of an ELF image.
std::optional<ModuleInfo> parse_module_info(const std::vector<u8>& elf);

}  // namespace zlb
