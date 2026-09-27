#include "common/util.h"

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <filesystem>

namespace zlb {

namespace fs = std::filesystem;

std::optional<std::vector<u8>> read_file(const std::string& path) {
    std::FILE* file = std::fopen(path.c_str(), "rb");
    if (!file) return std::nullopt;

    std::fseek(file, 0, SEEK_END);
    long size = std::ftell(file);
    std::fseek(file, 0, SEEK_SET);
    if (size < 0) {
        std::fclose(file);
        return std::nullopt;
    }

    std::vector<u8> data(static_cast<size_t>(size));
    if (size > 0 && std::fread(data.data(), 1, data.size(), file) != data.size()) {
        std::fclose(file);
        return std::nullopt;
    }
    std::fclose(file);
    return data;
}

bool write_file(const std::string& path, const void* data, size_t size) {
    std::error_code ec;
    fs::path p(path);
    if (p.has_parent_path()) fs::create_directories(p.parent_path(), ec);

    std::FILE* file = std::fopen(path.c_str(), "wb");
    if (!file) return false;
    bool ok = size == 0 || std::fwrite(data, 1, size, file) == size;
    std::fclose(file);
    return ok;
}

bool file_exists(const std::string& path) {
    std::error_code ec;
    return fs::exists(path, ec);
}

u64 file_size(const std::string& path) {
    std::error_code ec;
    auto size = fs::file_size(path, ec);
    return ec ? 0 : static_cast<u64>(size);
}

bool make_directories(const std::string& path) {
    std::error_code ec;
    fs::create_directories(path, ec);
    return !ec;
}

std::string path_join(const std::string& a, const std::string& b) {
    return (fs::path(a) / b).string();
}

std::string path_directory(const std::string& path) {
    return fs::path(path).parent_path().string();
}

std::string path_filename(const std::string& path) {
    return fs::path(path).filename().string();
}

std::string path_extension(const std::string& path) {
    return fs::path(path).extension().string();
}

std::string path_stem(const std::string& path) {
    return fs::path(path).stem().string();
}

std::string format(const char* fmt, ...) {
    char buffer[2048];
    va_list args;
    va_start(args, fmt);
    int needed = std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (needed < 0) return {};
    if (static_cast<size_t>(needed) < sizeof(buffer)) return std::string(buffer, static_cast<size_t>(needed));

    std::string big(static_cast<size_t>(needed) + 1, '\0');
    va_start(args, fmt);
    std::vsnprintf(big.data(), big.size(), fmt, args);
    va_end(args);
    big.resize(static_cast<size_t>(needed));
    return big;
}

std::string hex(u64 value, int width) {
    if (width <= 0) return format("%llX", static_cast<unsigned long long>(value));
    return format("%0*llX", width, static_cast<unsigned long long>(value));
}

std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string to_upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

std::vector<std::string> split(const std::string& text, char separator) {
    std::vector<std::string> out;
    std::string current;
    for (char c : text) {
        if (c == separator) {
            out.push_back(current);
            current.clear();
        } else {
            current.push_back(c);
        }
    }
    out.push_back(current);
    return out;
}

std::string trim(const std::string& text) {
    size_t begin = 0;
    size_t end = text.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(text[begin]))) ++begin;
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1]))) --end;
    return text.substr(begin, end - begin);
}

bool starts_with(const std::string& text, const std::string& prefix) {
    return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

bool ends_with(const std::string& text, const std::string& suffix) {
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string replace_all(std::string text, const std::string& from, const std::string& to) {
    if (from.empty()) return text;
    size_t pos = 0;
    while ((pos = text.find(from, pos)) != std::string::npos) {
        text.replace(pos, from.size(), to);
        pos += to.size();
    }
    return text;
}

bool parse_u64(const std::string& text, u64& out) {
    std::string s = trim(text);
    if (s.empty()) return false;

    int base = 10;
    size_t start = 0;
    if (s.size() > 2 && (s[0] == '0') && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        start = 2;
    } else if (s.size() > 2 && s[0] == '0' && (s[1] == 'b' || s[1] == 'B')) {
        base = 2;
        start = 2;
    } else if (s.size() > 1 && s[0] == '$') {
        base = 16;
        start = 1;
    }

    u64 value = 0;
    bool any = false;
    for (size_t i = start; i < s.size(); ++i) {
        char c = s[i];
        if (c == '_' || c == '\'') continue;
        int digit;
        if (c >= '0' && c <= '9') digit = c - '0';
        else if (c >= 'a' && c <= 'f') digit = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') digit = c - 'A' + 10;
        else return false;
        if (digit >= base) return false;
        value = value * static_cast<u64>(base) + static_cast<u64>(digit);
        any = true;
    }
    if (!any) return false;
    out = value;
    return true;
}

bool parse_u32(const std::string& text, u32& out) {
    u64 value = 0;
    if (!parse_u64(text, value)) return false;
    out = static_cast<u32>(value);
    return true;
}

std::string human_size(u64 bytes) {
    static const char* units[] = {"B", "KiB", "MiB", "GiB", "TiB"};
    double value = static_cast<double>(bytes);
    int unit = 0;
    while (value >= 1024.0 && unit < 4) {
        value /= 1024.0;
        ++unit;
    }
    if (unit == 0) return format("%llu B", static_cast<unsigned long long>(bytes));
    return format("%.2f %s", value, units[unit]);
}

}  // namespace zlb
