#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// NID -> name database, generated from vita_loader's db.yml by
// tools/gen_nid_table.py.  A NID is the 32-bit hash a PS Vita module stores in its
// import table instead of a symbol name, so without this table every import the
// guest resolves shows up as a bare hex value in diagnostics.
namespace zlb::debug {

struct NidEntry {
    std::uint32_t nid;
    const char* function;
    const char* library;
    const char* module;
};

// The whole table, sorted by NID (the data file provides these two).
const NidEntry* nid_entries();
std::size_t nid_entry_count();

// Every known function with this NID.  NIDs are not unique across libraries, so a
// lookup can legitimately return several rows.
std::vector<const NidEntry*> nid_lookup(std::uint32_t nid);

// Substring search over function names, case sensitive, in table order.
std::vector<const NidEntry*> nid_search(const std::string& fragment);

// "SceThreadmgr::sceKernelWaitThreadEnd", or an empty string when the NID is
// unknown.
std::string nid_name(std::uint32_t nid);

}  // namespace zlb::debug
