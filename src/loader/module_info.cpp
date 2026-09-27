// zeliboba - SceModuleInfo: the export/import (NID) tables of a Vita module.
//
// Port of pup_fiction/vita_loader/vita_loader.py (ELFHeader/phdr/Modinfo/
// Modexport/Modimport/parse_impexp) and of VitaTestSuite/Core/ElfImage.cs
// (VitaModule.Parse). The struct:
//
//   +0x00 u16 modattribute
//   +0x02 u8  modversion[2]
//   +0x04 char modname[27]
//   +0x1F u8  type
//   +0x20 u32 gp_value
//   +0x24 u32 export_top
//   +0x28 u32 export_end
//   +0x2C u32 import_top
//   +0x30 u32 import_end
//   +0x34 u32 module_nid          (present from the SDK generation that the
//                                  retail 1.04 modules use; read when it fits)
//
// export_top/end and import_top/end are *segment 0 relative* offsets of the
// library tables. Every library entry stores its own size, so the walk is
// `cur += entry.size`:
//
//   export (SceLibEntTable, 0x20): u8 size@0, u16 version@2, u16 attribute@4,
//                                  u16 num_functions@6, u16 num_vars@8,
//                                  u16 num_tls_vars@0xA, u32 lib_nid@0x10,
//                                  u32 lib_name@0x14, u32 nid_table@0x18,
//                                  u32 entry_table@0x1C
//   import (SceLibStubTable):      u16 size@0; two known layouts:
//                                  0x34 (firmware):  num_functions@4,
//                                                    lib_nid@0x10, lib_name@0x14,
//                                                    nid_table@0x1C, entry_table@0x20
//                                  0x24 (SDK):       num_functions@6,
//                                                    lib_nid@0xC, lib_name@0x10,
//                                                    nid_table@0x14, entry_table@0x18
//
// `ExportEntry::library` / `ImportEntry::library` carry the 0 based ordinal of
// the library inside its own table (loader.h declares that field as u16), see
// parse_module_libraries() in loader_extra.h for the full library records
// (name + NID + functions), which is what an import stub resolver needs.
#include <algorithm>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "common/util.h"
#include "loader/loader.h"
#include "loader/loader_extra.h"
#include "loader/nid.h"

namespace zlb {

namespace {

constexpr size_t kModuleInfoSize = 0x34;
constexpr size_t kModuleNameMax = 27;
constexpr u32 kMaxLibraryEntries = 4096;

struct LibraryTable {
    bool is_export = false;
    u16 index = 0;          // ordinal inside its own table
    u32 nid = 0;
    u16 version = 0;
    u16 attribute = 0;
    std::string name;
    u32 entry_offset = 0;   // segment 0 relative offset of the entry
    u32 size = 0;
    std::vector<std::pair<u32, u32>> functions;   // (nid, address)
};

struct ModuleTables {
    bool valid = false;
    u64 module_offset = 0;
    u16 attribute = 0;
    u16 version = 0;
    std::string name;
    u32 type = 0;
    u32 gp_value = 0;
    u32 export_top = 0;
    u32 export_end = 0;
    u32 import_top = 0;
    u32 import_end = 0;
    u32 module_nid = 0;
    std::vector<LibraryTable> libraries;
    std::string warning;
};

bool looks_like_module_info(const std::vector<u8>& data, size_t offset, size_t segment_end) {
    if (offset + kModuleInfoSize > data.size() || offset + kModuleInfoSize > segment_end)
        return false;
    // A printable module name of at least 3 characters.
    int printable = 0;
    for (size_t i = 0; i < kModuleNameMax; ++i) {
        const u8 c = data[offset + 4 + i];
        if (c == 0) break;
        if (c < 0x20 || c > 0x7E) return false;
        ++printable;
    }
    if (printable < 3) return false;
    // The export/import windows have to be inside segment 0 and well ordered.
    const u32 export_top = ld::read_u32(data, offset + 0x24);
    const u32 export_end = ld::read_u32(data, offset + 0x28);
    const u32 import_top = ld::read_u32(data, offset + 0x2C);
    const u32 import_end = ld::read_u32(data, offset + 0x30);
    auto sane = [](u32 top, u32 end, size_t limit) {
        if (top == 0 && end == 0) return true;
        if (top == 0 || end == 0 || end <= top) return false;
        return end <= limit;
    };
    const size_t limit = segment_end;
    return sane(export_top, export_end, limit) && sane(import_top, import_end, limit);
}

/// Walk one library table (export_top..export_end) at `top` (segment 0
/// relative) and collect the library records.
void walk_table(const std::vector<u8>& data, const Segment& segment0, u32 top, u32 end,
                bool is_export, u16& ordinal, std::vector<LibraryTable>& out) {
    if (top == 0 || end == 0 || end <= top) return;
    const u64 segment_end = static_cast<u64>(segment0.offset) + segment0.filesz;
    if (static_cast<u64>(top) + segment0.offset > segment_end) return;

    u32 current = top;
    for (u32 guard = 0; guard < kMaxLibraryEntries && current < end; ++guard) {
        const size_t entry = static_cast<size_t>(segment0.offset) + current;
        const size_t minimum = is_export ? 0x20u : 0x18u;
        if (!ld::in_range(data, entry, minimum)) break;

        LibraryTable library;
        library.is_export = is_export;
        library.entry_offset = current;
        u32 num_functions = 0;
        u32 lib_name_pointer = 0;
        u32 nid_table = 0;
        u32 entry_table = 0;
        u32 advance = 0;

        if (is_export) {
            advance = ld::read_u8(data, entry + 0);
            library.size = advance;
            library.version = ld::read_u16(data, entry + 2);
            library.attribute = ld::read_u16(data, entry + 4);
            num_functions = ld::read_u16(data, entry + 6);
            library.nid = ld::read_u32(data, entry + 0x10);
            lib_name_pointer = ld::read_u32(data, entry + 0x14);
            nid_table = ld::read_u32(data, entry + 0x18);
            entry_table = ld::read_u32(data, entry + 0x1C);
        } else {
            const u32 size = ld::read_u16(data, entry + 0);
            library.size = size;
            library.version = ld::read_u16(data, entry + 2);
            library.attribute = ld::read_u16(data, entry + 4);
            if (size >= 0x34) {
                advance = size;
                // Firmware 1.04 puts num_functions at +6 ("34 00 01 00 00 00 NN 00",
                // verified on all 42 Out/fs_dec/os0/kd modules: u16@+4 is always 0
                // and u16@+6 holds the real count). The generation the C# reference
                // describes uses +4; fall back to it when +6 is zero.
                num_functions = ld::read_u16(data, entry + 6);
                if (num_functions == 0) num_functions = ld::read_u16(data, entry + 4);
                library.nid = ld::read_u32(data, entry + 0x10);
                lib_name_pointer = ld::read_u32(data, entry + 0x14);
                nid_table = ld::read_u32(data, entry + 0x1C);
                entry_table = ld::read_u32(data, entry + 0x20);
            } else {
                // SDK / devkit layout: num_functions@6, lib_nid@0xC, lib_name@0x10,
                // nid_table@0x14, entry_table@0x18.
                advance = size > 0 ? size : 0x24;
                num_functions = ld::read_u16(data, entry + 6);
                library.nid = ld::read_u32(data, entry + 0xC);
                lib_name_pointer = ld::read_u32(data, entry + 0x10);
                nid_table = ld::read_u32(data, entry + 0x14);
                entry_table = ld::read_u32(data, entry + 0x18);
            }
        }

        if (lib_name_pointer != 0) {
            const int64_t name_offset = static_cast<int64_t>(segment0.offset) +
                                        static_cast<int64_t>(lib_name_pointer) -
                                        static_cast<int64_t>(segment0.vaddr);
            if (name_offset >= 0 && static_cast<size_t>(name_offset) < data.size())
                library.name = ld::read_cstr(data, static_cast<size_t>(name_offset), 256);
        }
        if (library.name.empty()) library.name = "noname";

        if (num_functions > 0 && nid_table != 0 && entry_table != 0) {
            const int64_t nid_offset = static_cast<int64_t>(segment0.offset) +
                                       static_cast<int64_t>(nid_table) -
                                       static_cast<int64_t>(segment0.vaddr);
            const int64_t entry_offset = static_cast<int64_t>(segment0.offset) +
                                         static_cast<int64_t>(entry_table) -
                                         static_cast<int64_t>(segment0.vaddr);
            for (u32 f = 0; f < num_functions; ++f) {
                const u64 nid_at = static_cast<u64>(nid_offset) + f * 4;
                const u64 function_at = static_cast<u64>(entry_offset) + f * 4;
                if (nid_at + 4 > segment_end || function_at + 4 > segment_end) break;
                if (nid_at + 4 > data.size() || function_at + 4 > data.size()) break;
                const u32 nid = ld::read_u32(data, static_cast<size_t>(nid_at));
                const u32 address = ld::read_u32(data, static_cast<size_t>(function_at));
                library.functions.emplace_back(nid, address);
            }
        }

        library.index = ordinal++;
        out.push_back(std::move(library));
        if (advance == 0) break;
        current += advance;
    }
}

/// Locate and decode the SceModuleInfo of an ELF image.
ModuleTables read_module_tables(const std::vector<u8>& data, const ElfImage& image) {
    ModuleTables tables;
    const Segment* segment0 = elf_first_load(image);
    if (segment0 == nullptr) {
        tables.warning = "ELF has no PT_LOAD segment";
        return tables;
    }
    const u64 segment_end = static_cast<u64>(segment0->offset) + segment0->filesz;

    // 1) the Vita module rule: e_entry is the module info offset from segment 0.
    int64_t module_offset = image.module_info_offset();
    // 2) the named section from an ELF with a section header table.
    if (module_offset < 0) {
        if (auto section = elf_section_offset(data, ".sceModuleInfo.rodata"))
            module_offset = static_cast<int64_t>(*section);
    }
    // 3) last resort: scan segment 0 for a plausible SceModuleInfo (a printable
    //    name plus well ordered export/import windows). Only for images that
    //    declare themselves a Vita module: kernel_boot_loader.self is a plain
    //    ET_EXEC (e_type 0x0002) ARM kernel image whose e_entry is a real
    //    virtual address, and it must not be reported as one of the module
    //    headers embedded in its data segment.
    const bool is_module_type = image.type == 0xFE00 || image.type == 0xFE01 ||
                                image.type == 0xFE04 || image.type == 0xFE05;
    if (module_offset < 0 && is_module_type) {
        for (size_t offset = static_cast<size_t>(segment0->offset);
             offset + kModuleInfoSize <= segment_end && offset + kModuleInfoSize <= data.size();
             offset += 4) {
            if (looks_like_module_info(data, offset, static_cast<size_t>(segment_end))) {
                module_offset = static_cast<int64_t>(offset);
                break;
            }
        }
    }
    if (module_offset < 0 || !ld::in_range(data, static_cast<size_t>(module_offset), kModuleInfoSize)) {
        tables.warning = "no SceModuleInfo found (e_entry is a virtual address and there is no "
                         ".sceModuleInfo.rodata section)";
        return tables;
    }

    tables.module_offset = static_cast<u64>(module_offset);
    tables.attribute = ld::read_u16(data, static_cast<size_t>(module_offset) + 0x00);
    tables.version = ld::read_u16(data, static_cast<size_t>(module_offset) + 0x02);
    tables.name = ld::read_cstr(data, static_cast<size_t>(module_offset) + 4, kModuleNameMax);
    tables.type = ld::read_u8(data, static_cast<size_t>(module_offset) + 0x1F);
    tables.gp_value = ld::read_u32(data, static_cast<size_t>(module_offset) + 0x20);
    tables.export_top = ld::read_u32(data, static_cast<size_t>(module_offset) + 0x24);
    tables.export_end = ld::read_u32(data, static_cast<size_t>(module_offset) + 0x28);
    tables.import_top = ld::read_u32(data, static_cast<size_t>(module_offset) + 0x2C);
    tables.import_end = ld::read_u32(data, static_cast<size_t>(module_offset) + 0x30);
    if (ld::in_range(data, static_cast<size_t>(module_offset) + 0x34, 4))
        tables.module_nid = ld::read_u32(data, static_cast<size_t>(module_offset) + 0x34);

    u16 ordinal = 0;
    walk_table(data, *segment0, tables.export_top, tables.export_end, true, ordinal, tables.libraries);
    ordinal = 0;
    walk_table(data, *segment0, tables.import_top, tables.import_end, false, ordinal,
               tables.libraries);
    tables.valid = true;
    return tables;
}

std::string resolve(const NidDatabase* database, u32 nid) {
    if (database == nullptr) return {};
    return database->lookup(nid);
}

}  // namespace

std::optional<ModuleInfo> parse_module_info_with(const std::vector<u8>& elf,
                                                 const NidDatabase* database) {
    auto image = parse_elf(elf);
    if (!image) return std::nullopt;

    const ModuleTables tables = read_module_tables(elf, *image);
    if (!tables.valid) return std::nullopt;

    ModuleInfo info;
    info.name = tables.name;
    info.type = tables.type;
    info.module_nid = tables.module_nid;
    info.entry = elf_resolved_entry(*image);
    info.exports_start = tables.export_top;
    info.exports_end = tables.export_end;
    info.imports_start = tables.import_top;
    info.imports_end = tables.import_end;

    for (const LibraryTable& library : tables.libraries) {
        if (library.is_export) {
            if (info.library_name.empty()) info.library_name = library.name;
            for (const auto& function : library.functions) {
                ExportEntry entry;
                entry.address = function.second;
                entry.nid = function.first;
                entry.library = library.index;
                entry.name = resolve(database, function.first);
                info.exports.push_back(std::move(entry));
            }
        } else {
            for (const auto& function : library.functions) {
                ImportEntry entry;
                entry.address = function.second;
                entry.nid = function.first;
                entry.library = library.index;
                entry.name = resolve(database, function.first);
                info.imports.push_back(std::move(entry));
            }
        }
    }
    return info;
}

std::optional<ModuleInfo> parse_module_info(const std::vector<u8>& elf) {
    return parse_module_info_with(elf, default_nid_database());
}

std::vector<ModuleLibrary> parse_module_libraries(const std::vector<u8>& elf,
                                                  const NidDatabase* database) {
    std::vector<ModuleLibrary> out;
    auto image = parse_elf(elf);
    if (!image) return out;
    const ModuleTables tables = read_module_tables(elf, *image);
    if (!tables.valid) return out;

    for (const LibraryTable& library : tables.libraries) {
        ModuleLibrary record;
        record.name = library.name;
        record.nid = library.nid;
        record.index = library.index;
        record.is_export = library.is_export;
        record.size = library.size;
        record.attribute = library.attribute;
        record.version = library.version;
        for (const auto& function : library.functions) {
            ModuleLibraryFunction entry;
            entry.nid = function.first;
            entry.address = function.second;
            entry.name = resolve(database, function.first);
            record.functions.push_back(std::move(entry));
        }
        out.push_back(std::move(record));
    }
    return out;
}

u64 module_info_file_offset(const std::vector<u8>& elf) {
    auto image = parse_elf(elf);
    if (!image) return 0;
    const ModuleTables tables = read_module_tables(elf, *image);
    return tables.valid ? tables.module_offset : 0;
}

std::string describe_module_info(const ModuleInfo& info) {
    std::string text = format("module '%s' type=0x%02X nid=0x%08X entry=0x%X exports=%zu imports=%zu",
                              info.name.c_str(), info.type, info.module_nid, info.entry,
                              info.exports.size(), info.imports.size());
    if (!info.library_name.empty()) text += format(" library=%s", info.library_name.c_str());
    return text;
}

}  // namespace zlb
