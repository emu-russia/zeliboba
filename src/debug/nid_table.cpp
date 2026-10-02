#include "debug/nid_table.h"

namespace zlb::debug {

std::vector<const NidEntry*> nid_lookup(std::uint32_t nid) {
    std::vector<const NidEntry*> matches;
    const NidEntry* entries = nid_entries();
    const std::size_t count = nid_entry_count();
    // The table is sorted by NID, so seek to the first hit and take the run.
    std::size_t low = 0;
    std::size_t high = count;
    while (low < high) {
        const std::size_t mid = low + (high - low) / 2;
        if (entries[mid].nid < nid) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    for (std::size_t index = low; index < count && entries[index].nid == nid; ++index) {
        matches.push_back(&entries[index]);
    }
    return matches;
}

std::vector<const NidEntry*> nid_search(const std::string& fragment) {
    std::vector<const NidEntry*> matches;
    if (fragment.empty()) return matches;
    const NidEntry* entries = nid_entries();
    const std::size_t count = nid_entry_count();
    for (std::size_t index = 0; index < count; ++index) {
        const std::string name = entries[index].function;
        if (name.find(fragment) != std::string::npos) matches.push_back(&entries[index]);
    }
    return matches;
}

std::string nid_name(std::uint32_t nid) {
    const std::vector<const NidEntry*> matches = nid_lookup(nid);
    if (matches.empty()) return {};
    return std::string(matches.front()->library) + "::" + matches.front()->function;
}

}  // namespace zlb::debug
