// zeliboba - file and string helpers shared by the tools, loaders and tests.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "common/types.h"

namespace zlb {

// ---------------------------------------------------------------------------
// Files
// ---------------------------------------------------------------------------

std::optional<std::vector<u8>> read_file(const std::string& path);
bool write_file(const std::string& path, const void* data, size_t size);
inline bool write_file(const std::string& path, const std::vector<u8>& data) {
    return write_file(path, data.data(), data.size());
}
bool file_exists(const std::string& path);
u64 file_size(const std::string& path);
bool make_directories(const std::string& path);
std::string path_join(const std::string& a, const std::string& b);
std::string path_directory(const std::string& path);
std::string path_filename(const std::string& path);
std::string path_extension(const std::string& path);
std::string path_stem(const std::string& path);

// ---------------------------------------------------------------------------
// Strings / numbers
// ---------------------------------------------------------------------------

std::string format(const char* fmt, ...);
std::string hex(u64 value, int width = 0);
std::string to_lower(std::string s);
std::string to_upper(std::string s);
std::vector<std::string> split(const std::string& text, char separator);
std::string trim(const std::string& text);
bool starts_with(const std::string& text, const std::string& prefix);
bool ends_with(const std::string& text, const std::string& suffix);
std::string replace_all(std::string text, const std::string& from, const std::string& to);

// Parse a number written in decimal ("1234"), hex ("0x5C000", "$5C000") or
// binary ("0b1010"). Returns false when the text is malformed.
bool parse_u64(const std::string& text, u64& out);
bool parse_u32(const std::string& text, u32& out);

// Human readable byte count: "1.44 MiB".
std::string human_size(u64 bytes);

}  // namespace zlb
