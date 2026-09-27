// zeliboba - NID (32 bit SCE name id) to symbol name database.
//
// The database shipped with the workspace is the "nid_db.yml" YAML file
// (VitaTestSuite/Docs/nid_db.yml, 434 KiB, identical to
// pup_fiction/vita_loader/db.yml). Its interesting lines are the function
// tables:
//
//     modules:
//       SceAVConfig:
//         nid: 0x222DDEB1                  <- skipped, it is a *module* nid
//         libraries:
//           SceAVConfig:
//             kernel: false
//             nid: 0x79E0F03F              <- skipped (contains "nid: ")
//             functions:
//               sceAVConfigChangeReg: 0xD0595CE4   <- taken
//
// The reader reproduces the reference implementation
// (pup_fiction/vita_loader/vita_loader.py VitaElf.load_nids, ported in
// VitaTestSuite/Core/NidDatabase.cs):
//
//     for line in data:
//         if "0x" in line and "nid: " not in line:
//             name, nid = line.strip().split(":")
//             nid_to_name[int(nid, 16)] = name
//
// i.e. any line that mentions "0x", is not a "nid: ..." line and splits into
// name/hex becomes an entry; later lines win. A plain "<name>: 0xNID" or
// "- name: 0xNID" list is accepted by the same rule, so both the YAML tree and
// the flat list form load.
#include "loader/nid.h"

#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/util.h"

namespace zlb {

namespace {

bool parse_hex_nid(const std::string& text, u32& out) {
    std::string value = trim(text);
    if (value.size() > 2 && value[0] == '0' && (value[1] == 'x' || value[1] == 'X'))
        value = value.substr(2);
    if (value.empty() || value.size() > 8) return false;
    u64 parsed = 0;
    for (char c : value) {
        int digit = -1;
        if (c >= '0' && c <= '9') digit = c - '0';
        else if (c >= 'a' && c <= 'f') digit = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') digit = c - 'A' + 10;
        if (digit < 0) return false;
        parsed = (parsed << 4) | static_cast<u64>(digit);
    }
    out = static_cast<u32>(parsed);
    return true;
}

/// Strip a leading "- " list marker and any trailing comment.
std::string clean_line(const std::string& raw) {
    std::string line = raw;
    const size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    line = trim(line);
    if (!line.empty() && line[0] == '-') {
        line = trim(line.substr(1));
    }
    return line;
}

}  // namespace

bool NidDatabase::load(const std::string& path) {
    map_.clear();
    path_ = path;
    warnings_.clear();

    auto data = read_file(path);
    if (!data) return false;

    const std::string text(reinterpret_cast<const char*>(data->data()), data->size());
    int line_number = 0;
    for (const std::string& raw_line : split(text, '\n')) {
        ++line_number;
        const std::string line = clean_line(raw_line);
        if (line.find("0x") == std::string::npos) continue;          // python: if "0x" in line
        if (line.find("nid: ") != std::string::npos) continue;       // ... and "nid: " not in line
        if (line.find(": ") == std::string::npos) {
            warnings_.push_back(format("line %d: no 'name: 0xNID' pair in '%s'", line_number,
                                       line.c_str()));
            continue;
        }

        const size_t colon = line.rfind(':');
        const std::string name = trim(line.substr(0, colon));
        u32 nid = 0;
        if (name.empty() || !parse_hex_nid(line.substr(colon + 1), nid)) {
            warnings_.push_back(format("line %d: cannot parse NID from '%s'", line_number,
                                       line.c_str()));
            continue;
        }
        map_[nid] = name;   // later lines win, exactly like the python dict
    }
    return !map_.empty();
}

std::string NidDatabase::lookup(u32 nid) const {
    const auto it = map_.find(nid);
    return it == map_.end() ? std::string() : it->second;
}

bool NidDatabase::has(u32 nid) const { return map_.find(nid) != map_.end(); }

std::optional<NidDatabase> load_nid_database(const std::string& path) {
    NidDatabase database;
    if (!database.load(path)) return std::nullopt;
    return database;
}

std::vector<std::string> find_nid_databases() {
    // The database lives next to the extracted firmware modules
    // (Out/fs_dec/os0/kd/db.yml) and in the reference docs tree
    // (VitaTestSuite/Docs/nid_db.yml, pup_fiction/vita_loader/db.yml).
    static const char* kCandidates[] = {
        "Vita_104_Firmware/Out/fs_dec/os0/kd/db.yml",
        "VitaTestSuite/Docs/nid_db.yml",
        "pup_fiction/vita_loader/db.yml",
        "../Vita_104_Firmware/Out/fs_dec/os0/kd/db.yml",
        "../VitaTestSuite/Docs/nid_db.yml",
        "../pup_fiction/vita_loader/db.yml",
        "../../Vita_104_Firmware/Out/fs_dec/os0/kd/db.yml",
        "../../VitaTestSuite/Docs/nid_db.yml",
        "../../pup_fiction/vita_loader/db.yml",
    };
    std::vector<std::string> found;
    for (const char* candidate : kCandidates) {
        if (file_exists(candidate)) found.push_back(candidate);
    }
    return found;
}

const NidDatabase* default_nid_database() {
    // Lazily loaded read-only cache: the file is only parsed on first use, and
    // the same pointer is handed out afterwards. This is the only long lived
    // state the loader keeps besides Log::instance().
    static const NidDatabase* cached = []() -> const NidDatabase* {
        static NidDatabase database;
        for (const std::string& path : find_nid_databases()) {
            if (database.load(path)) return &database;
        }
        return nullptr;
    }();
    return cached;
}

}  // namespace zlb
