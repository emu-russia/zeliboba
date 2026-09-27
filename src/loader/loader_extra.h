// zeliboba - loader internals shared between the loader translation units.
//
// loader.h is the public contract; this header only carries the helpers that
// more than one loader .cpp needs (little-endian readers, the AES-CTR and
// DEFLATE primitives that keys.h does not declare, and a few ELF/MeP
// conventions). Everything here is still `namespace zlb` and dependency free.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "common/types.h"
#include "loader/keys.h"
#include "loader/loader.h"

namespace zlb {

class NidDatabase;

// ---------------------------------------------------------------------------
// Byte readers / string helpers.
//
// These live in their own namespace so that a same-named helper in another
// workstream's translation unit can never be picked up by the linker instead.
// Out of range reads return 0 exactly like the C# reference's BinUtil, so the
// container parsers can stay defensive without exceptions.
// ---------------------------------------------------------------------------

namespace ld {

bool in_range(const std::vector<u8>& data, size_t offset, size_t length);
u8 read_u8(const std::vector<u8>& data, size_t offset);
u16 read_u16(const std::vector<u8>& data, size_t offset);
u32 read_u32(const std::vector<u8>& data, size_t offset);
u64 read_u64(const std::vector<u8>& data, size_t offset);
/// NUL terminated Latin-1 string, never throws.
std::string read_cstr(const std::vector<u8>& data, size_t offset, size_t max_length);
/// Lower case hex without separators.
std::string hex_bytes(const u8* data, size_t length);
/// Keeps `length` bytes of `data` starting at `offset` (shorter when clipped).
std::vector<u8> slice(const std::vector<u8>& data, size_t offset, size_t length);

}  // namespace ld

// ---------------------------------------------------------------------------
// Crypto helpers that keys.h does not declare.
// ---------------------------------------------------------------------------

/// AES-CTR (the SCE segment encryption "AES128CTR"): the 16 byte IV is the
/// initial 128 bit big-endian counter, the keystream is E(counter) with a
/// 128 bit AES key. `length` may be any size.
void aes_ctr_crypt(const u8* key128, const u8* iv, const u8* input, size_t length, u8* output);

/// AES-CBC decryption with a 16/24/32 byte key (the SCE metadata keys are 32
/// bytes = AES-256, the NPDRM and vault keys are 16 bytes). A trailing partial
/// block is zero padded, exactly like the C# reference's CoreAes.CbcDecrypt.
void aes_cbc_decrypt_any(const std::vector<u8>& key, const std::vector<u8>& iv, const u8* input,
                         size_t length, u8* output);

// ---------------------------------------------------------------------------
// DEFLATE (RFC 1951) with the RFC 1950 zlib wrapper auto detected.
//
// The SCE segment compressor emits zlib streams; SELF segments and PUP/SRVK/
// SPKG payloads are routinely compressed (every retail 1.04 .skprx has
// compressed segments). zlib is not a dependency we are allowed to take, so the
// decompressor lives here.
// ---------------------------------------------------------------------------

/// Inflate `length` bytes at `input`. `mode`, when given, receives a short
/// description of what was detected. Returns nullopt when the stream is
/// malformed or the output would exceed `max_output` (0 = 64 MiB).
std::optional<std::vector<u8>> inflate_zlib_or_raw(const u8* input, size_t length,
                                                   std::string* mode = nullptr,
                                                   size_t max_output = 0);

/// zlib header check: CM == 8 (deflate) and (CMF * 256 + FLG) % 31 == 0.
bool has_zlib_header(const u8* data, size_t length);

// ---------------------------------------------------------------------------
// SCE key table access (the KeyStore semantics of pup_fiction's scetypes.py).
// keys.h itself only exposes a named byte store; keys.cpp keeps the three
// pup_fiction tables inside it (see the comment at the top of keys.cpp).
// ---------------------------------------------------------------------------

enum class SceKeyKind : int { Metadata = 0, Npdrm = 1 };
enum class SceContainerKind : int { Self = 1, Srvk = 2, Spkg = 3, Dev = 0xC0 };
enum class SceSelfKind : int {
    None = 0x00,
    Kernel = 0x07,
    App = 0x08,
    Boot = 0x09,
    Secure = 0x0B,
    User = 0x0D,
};

/// One registered key/IV pair with its system version window.
struct SceKeyCandidate {
    std::vector<u8> key;
    std::vector<u8> iv;
    int key_revision = 0;
    u64 min_version = 0;
    u64 max_version = 0;
    std::string source;   // "pup_fiction/keys.py:73", or "<file>:<line>" for a key file
};

/// KeyStore.get() as a list: every entry of (key kind, SCE type, SELF type) that
/// matches the version window (unless `ignore_sys_version`, or `sys_version` is
/// negative) and the key revision (unless `key_revision` is negative), in
/// registration order. `selftype`/`sce_type` are the raw header values.
/// `sys_version` is 64 bit (the SCE version numbers are 44 bit wide, e.g.
/// 0x10400000000, so a 32 bit `long` would truncate them).
std::vector<SceKeyCandidate> sce_key_candidates(const SceKeys& keys, int key_kind, int sce_type,
                                                bool ignore_sys_version, s64 sys_version,
                                                int key_revision, int self_type);

/// KeyStore.get(): the first matching candidate. False when there is none (the
/// python raises KeyError there, this API never throws).
bool sce_keys_lookup(const SceKeys& keys, int key_kind, int sce_type, s64 sys_version,
                     int key_revision, int self_type, SceKeyCandidate& out);

/// Human readable description of every registered table entry (diagnostics).
std::vector<std::string> describe_key_table(const SceKeys& keys);

// ---------------------------------------------------------------------------
// ELF helpers.
// ---------------------------------------------------------------------------

const char* elf_segment_type_name(u32 p_type);
const char* elf_machine_name(u16 machine);
Arch elf_arch(u16 machine);

/// First PT_LOAD segment, or nullptr.
const Segment* elf_first_load(const ElfImage& image);

/// e_entry of a Vita module is *not* a virtual address but the file offset of
/// the SceModuleInfo structure relative to segment 0, so the address is
/// `segment0.p_vaddr + e_entry` whenever the raw value does not already land
/// inside a PT_LOAD segment (ElfImage.cs ResolveEntry).
u32 elf_resolved_entry(const ElfImage& image);

/// The section header table of a *reconstructed* SELF is not always present
/// (retail SELFs encrypt it outside the metadata sections); this returns true
/// when e_shoff/e_shnum point at a table that is fully inside the image and
/// passes a basic type/offset sanity check.
bool elf_section_headers_sane(const std::vector<u8>& elf);

/// File offset of the ELF section named `name`, or nullopt when the image has
/// no usable section header table.
std::optional<u32> elf_section_offset(const std::vector<u8>& elf, const std::string& name);

/// One line summary of a parsed ELF (used by identify()).
std::string describe_elf(const ElfImage& image);

// ---------------------------------------------------------------------------
// MeP images.
// ---------------------------------------------------------------------------

/// The CMeP first loader stages the next stage at 0x40000 and jumps there
/// (dumps/bootrom_analysis/ANALYSIS.md 4.1/4.2: `$lp = 0x40000`, the stub's
/// `ret` jumps into the image; 4.3 validates the image header at 0x40000), so
/// that is the default load address for a headerless MeP image or an ENP body.
constexpr u32 kMepBootWindow = 0x40000;

/// Magic sniff for the MeP image header (`0x64B2C8E5` at offset 0).
bool is_mep_image(const std::vector<u8>& data);

/// The image body of an ENP container: `header.field_0x10` bytes starting at
/// `header.size` (see the comment at the top of mep_image.cpp for the evidence).
std::optional<std::vector<u8>> mep_payload(const std::vector<u8>& data,
                                           const MepImageHeader& header);

std::string describe_mep_header(const MepImageHeader& header);

/// SLB2 builder (declared next to parse_slb2 in loader.h). The three argument
/// overload lets the eMMC image builder reproduce the retail container exactly:
/// the 1.04 container has version 1, header_size 0x200 and reports 0x2000
/// (4 MiB) in the "+0x10 total size" field even though only 0x4EE blocks are
/// used, and the unused space is 0xFF (erased eMMC flash).
std::vector<u8> build_slb2(const std::vector<Slb2Entry>& entries, u32 version, u32 total_blocks,
                           u8 fill);

/// One line summary of a parsed SLB2 container.
std::string describe_slb2(const Slb2Image& image);

/// Alternative to `parse_module_info()` for callers that already have a NID
/// database (the tests and tools); `parse_module_info()` uses the lazily loaded
/// default database.
std::optional<ModuleInfo> parse_module_info_with(const std::vector<u8>& elf, const NidDatabase* database);

// ---------------------------------------------------------------------------
// Full SceModuleInfo detail (library records with their NID, name and
// functions). ModuleInfo only carries a u16 `library` ordinal per entry; the
// import stub resolver needs the complete records.
// ---------------------------------------------------------------------------

struct ModuleLibraryFunction {
    u32 nid = 0;
    u32 address = 0;
    std::string name;   // NID database name, empty when unknown
};

struct ModuleLibrary {
    std::string name;
    u32 nid = 0;
    u16 index = 0;          // ordinal inside its own table (export or import)
    u16 version = 0;
    u16 attribute = 0;
    u32 size = 0;
    bool is_export = false;
    std::vector<ModuleLibraryFunction> functions;
};

std::vector<ModuleLibrary> parse_module_libraries(const std::vector<u8>& elf,
                                                  const NidDatabase* database);

/// File offset of the SceModuleInfo structure, or 0 when there is none.
u64 module_info_file_offset(const std::vector<u8>& elf);

/// One line summary of a parsed module info.
std::string describe_module_info(const ModuleInfo& info);

/// One line summary of a parsed PUP image.
std::string describe_pup(const PupImage& image);

// ---------------------------------------------------------------------------
// SCE container decryption detail (see self.cpp).
// ---------------------------------------------------------------------------

/// Why a SELF/SRVK/SPKG decryption did or did not work: `stage` names the exact
/// step that failed ("sce header", "self header", "inner elf", "metadata key",
/// "metadata body", "segment N", "klicensee", "output") and `message` carries
/// the numbers (key revision, SELF type, system version, vault size, ...).
struct SelfDecryptReport {
    bool ok = false;
    std::string stage = "not started";
    std::string message;
    std::string metadata_key;        // the table entry that verified the metadata
    std::string inflate_mode;        // what the DEFLATE auto detection settled on
    int metadata_sections = 0;
    int metadata_keys = 0;
    bool section_headers_dropped = false;   // e_shoff/e_shnum could not be recovered
    std::vector<std::string> segment_notes;
};

/// One decrypted (and inflated) metadata segment, as scedecrypt.py writes them.
struct SceSegmentData {
    u32 index = 0;           // metadata section index (the "segNN" of scedecrypt)
    u32 segment_index = 0;   // MetadataSection.seg_idx (the program header index)
    u32 offset = 0;          // offset inside the container
    u32 size = 0;            // size inside the container
    bool encrypted = false;
    bool compressed = false;
    std::string notes;       // "deflate zlib (2 byte header stripped) 0x.. -> 0x.."
    std::vector<u8> data;
};

/// self_to_elf() with the failure detail kept. `klicense` is the 16 byte
/// klicensee from a RIF file (only needed for SELF type APP); pass an empty
/// vector when there is none.
std::optional<std::vector<u8>> self_to_elf_report(const std::vector<u8>& data, const SceKeys& keys,
                                                  const std::vector<u8>& klicense,
                                                  SelfDecryptReport& report);

/// scedecrypt.py: decrypt every AES128CTR metadata segment of a SELF/SRVK/SPKG
/// container (and inflate the compressed ones). Returns the segments in section
/// order; an empty vector with `report->ok == false` means the stage in the
/// report failed.
std::vector<SceSegmentData> sce_decrypt_segments(const std::vector<u8>& data, const SceKeys& keys,
                                                 const std::vector<u8>& klicense,
                                                 SelfDecryptReport* report);

/// One line summary of an SCE container header.
std::string describe_self(const std::vector<u8>& data);

/// What the *clear* part of a SELF says about the inner ELF: the ELF header and
/// the program header table are stored unencrypted, so the arch, the load range
/// and the entry point are available without any key.
struct SceInnerElfInfo {
    bool present = false;
    u16 machine = 0;
    u32 entry_raw = 0;
    u32 entry_resolved = 0;
    u16 phnum = 0;
    u32 load_address = 0;
    u64 span = 0;
};

SceInnerElfInfo sce_inner_elf_info(const std::vector<u8>& data);

}  // namespace zlb
