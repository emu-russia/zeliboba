// zeliboba - NID -> symbol name database.
//
// The 32 bit NID is the SCE "name id" (the low 32 bits of a SHA-1 of the symbol
// name); the loader resolves the export/import tables of a module with it so the
// debugger can print names instead of numbers. See nid.cpp for the file format.
#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/types.h"

namespace zlb {

class NidDatabase {
public:
    /// Parse `path`. Returns false when the file cannot be read or contains no
    /// usable entry; the object is left empty in that case. Never throws.
    bool load(const std::string& path);

    /// Symbol name for `nid`, or an empty string when it is unknown.
    std::string lookup(u32 nid) const;
    bool has(u32 nid) const;
    size_t size() const { return map_.size(); }
    const std::string& path() const { return path_; }
    const std::vector<std::string>& warnings() const { return warnings_; }
    const std::unordered_map<u32, std::string>& entries() const { return map_; }

private:
    std::unordered_map<u32, std::string> map_;
    std::vector<std::string> warnings_;
    std::string path_;
};

/// Load the database, or nullopt when `path` has no usable entry.
std::optional<NidDatabase> load_nid_database(const std::string& path);

/// Every candidate path that exists (relative to the current directory and up
/// to two levels above it), most specific first.
std::vector<std::string> find_nid_databases();

/// Lazily loaded shared instance for callers without a database of their own
/// (parse_module_info). Null when none of the candidate paths can be read.
const NidDatabase* default_nid_database();

}  // namespace zlb
