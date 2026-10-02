// zeliboba - save-state stream and file helpers. See common/state.h.
#include "common/state.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>

#include "common/util.h"

namespace zlb {

namespace {

constexpr char kSectionTag[4] = {'S', 'E', 'C', ' '};
constexpr char kFileMagic[8] = {'Z', 'L', 'B', 'S', 'T', 'A', 'T', 'E'};
constexpr size_t kFileHeaderSize = 8 + 4 + 8;  // magic + version + body size
constexpr size_t kFileTrailerSize = 8;         // FNV-1a of the body

u64 fnv1a(const u8* data, size_t size) {
    u64 hash = 1469598103934665603ull;
    for (size_t i = 0; i < size; ++i) {
        hash ^= data[i];
        hash *= 1099511628211ull;
    }
    return hash;
}

}  // namespace

// ---------------------------------------------------------------------------
// StateWriter
// ---------------------------------------------------------------------------

void StateWriter::begin(const char* section) {
    bytes(kSectionTag, sizeof(kSectionTag));
    const size_t length = section != nullptr ? std::strlen(section) : 0;
    put_u16(static_cast<u16>(length));
    bytes(section, length);
    open_.push_back(out_.size());
    put_u64(0);  // patched by end()
}

void StateWriter::end() {
    if (open_.empty()) return;
    const size_t at = open_.back();
    open_.pop_back();
    const u64 length = static_cast<u64>(out_.size() - (at + 8));
    for (int i = 0; i < 8; ++i) {
        out_[at + static_cast<size_t>(i)] = static_cast<u8>(length >> (8 * i));
    }
}

// ---------------------------------------------------------------------------
// StateReader
// ---------------------------------------------------------------------------

void StateReader::fail(const std::string& message) {
    if (ok_) {
        ok_ = false;
        error_ = message;
    }
}

bool StateReader::begin(const char* section) {
    if (!ok_) return false;
    if (!have(sizeof(kSectionTag))) return false;
    if (std::memcmp(cursor_, kSectionTag, sizeof(kSectionTag)) != 0) {
        fail("state file: expected a section tag");
        return false;
    }
    cursor_ += sizeof(kSectionTag);
    const u16 name_length = get_u16();
    if (!ok_ || !have(name_length)) return false;
    std::string name(reinterpret_cast<const char*>(cursor_), name_length);
    cursor_ += name_length;
    const u64 length = get_u64();
    if (!ok_ || !have(static_cast<size_t>(length))) return false;
    if (section == nullptr || name != section) {
        // A different section: skip it and report the mismatch. The caller's
        // error message names what it wanted, which is what a user needs.
        cursor_ += static_cast<size_t>(length);
        fail(format("state file: expected section '%s', found '%s'", section != nullptr ? section : "",
                    name.c_str()));
        return false;
    }
    section_ends_.push_back(cursor_ + length);
    return true;
}

void StateReader::end() {
    if (!ok_ || section_ends_.empty()) return;
    cursor_ = section_ends_.back();
    section_ends_.pop_back();
}

// ---------------------------------------------------------------------------
// RAM pages
// ---------------------------------------------------------------------------

size_t state_write_pages(StateWriter& writer, const u8* data, size_t size) {
    const size_t pages = (size + kStatePageSize - 1) / kStatePageSize;
    std::vector<u8> bitmap(pages, 0);
    size_t written = 0;
    for (size_t page = 0; page < pages; ++page) {
        const size_t offset = page * kStatePageSize;
        const size_t length = std::min(kStatePageSize, size - offset);
        bool nonzero = false;
        for (size_t i = 0; i < length; ++i) {
            if (data[offset + i] != 0) {
                nonzero = true;
                break;
            }
        }
        if (nonzero) {
            bitmap[page] = 1;
            written += length;
        }
    }
    writer.put_u64(static_cast<u64>(size));
    writer.put_u64(static_cast<u64>(pages));
    writer.bytes(bitmap.data(), bitmap.size());
    for (size_t page = 0; page < pages; ++page) {
        if (bitmap[page] == 0) continue;
        const size_t offset = page * kStatePageSize;
        const size_t length = std::min(kStatePageSize, size - offset);
        writer.bytes(data + offset, length);
    }
    return written;
}

void state_read_pages(StateReader& reader, u8* data, size_t size) {
    const u64 stored_size = reader.get_u64();
    const u64 pages = reader.get_u64();
    if (!reader.ok()) return;
    if (stored_size != size) {
        reader.fail(format("state file: RAM region size %llu does not match this build's %llu",
                           static_cast<unsigned long long>(stored_size),
                           static_cast<unsigned long long>(size)));
        return;
    }
    std::vector<u8> bitmap(static_cast<size_t>(pages), 0);
    reader.bytes(bitmap.data(), bitmap.size());
    if (!reader.ok()) return;
    std::memset(data, 0, size);
    for (size_t page = 0; page < bitmap.size(); ++page) {
        if (bitmap[page] == 0) continue;
        const size_t offset = page * kStatePageSize;
        if (offset >= size) {
            reader.fail("state file: RAM page bitmap is out of range");
            return;
        }
        const size_t length = std::min(kStatePageSize, size - offset);
        reader.bytes(data + offset, length);
        if (!reader.ok()) return;
    }
}

// ---------------------------------------------------------------------------
// Files
// ---------------------------------------------------------------------------

bool state_write_file(const std::string& path, const StateWriter& body, std::string& error) {
    const std::vector<u8>& payload = body.data();
    std::vector<u8> file;
    file.reserve(kFileHeaderSize + payload.size() + kFileTrailerSize);
    file.insert(file.end(), kFileMagic, kFileMagic + sizeof(kFileMagic));
    StateWriter header;
    header.put_u32(kStateFormatVersion);
    header.put_u64(static_cast<u64>(payload.size()));
    file.insert(file.end(), header.data().begin(), header.data().end());
    file.insert(file.end(), payload.begin(), payload.end());
    StateWriter trailer;
    trailer.put_u64(fnv1a(payload.data(), payload.size()));
    file.insert(file.end(), trailer.data().begin(), trailer.data().end());

    const std::string temporary = path + ".tmp";
    if (!write_file(temporary, file)) {
        error = format("cannot write '%s'", temporary.c_str());
        return false;
    }
    std::error_code ec;
    std::filesystem::remove(path, ec);
    std::filesystem::rename(temporary, path, ec);
    if (ec) {
        error = format("cannot move '%s' to '%s': %s", temporary.c_str(), path.c_str(),
                       ec.message().c_str());
        return false;
    }
    return true;
}

bool state_read_file(const std::string& path, std::vector<u8>& body, std::string& error) {
    auto data = read_file(path);
    if (!data) {
        error = format("cannot read '%s'", path.c_str());
        return false;
    }
    if (data->size() < kFileHeaderSize + kFileTrailerSize) {
        error = format("'%s' is too small to be a save state", path.c_str());
        return false;
    }
    if (std::memcmp(data->data(), kFileMagic, sizeof(kFileMagic)) != 0) {
        error = format("'%s' is not a zeliboba save state", path.c_str());
        return false;
    }
    StateReader header(data->data() + sizeof(kFileMagic), 12);
    const u32 version = header.get_u32();
    const u64 body_size = header.get_u64();
    if (version != kStateFormatVersion) {
        error = format("state format version %u, this build writes %u", version, kStateFormatVersion);
        return false;
    }
    if (body_size != data->size() - kFileHeaderSize - kFileTrailerSize) {
        error = format("'%s' is truncated or has trailing data", path.c_str());
        return false;
    }
    const u8* body_data = data->data() + kFileHeaderSize;
    const u8* trailer = body_data + body_size;
    const u64 expected = static_cast<u64>(trailer[0]) | (static_cast<u64>(trailer[1]) << 8) |
                         (static_cast<u64>(trailer[2]) << 16) | (static_cast<u64>(trailer[3]) << 24) |
                         (static_cast<u64>(trailer[4]) << 32) | (static_cast<u64>(trailer[5]) << 40) |
                         (static_cast<u64>(trailer[6]) << 48) | (static_cast<u64>(trailer[7]) << 56);
    if (fnv1a(body_data, static_cast<size_t>(body_size)) != expected) {
        error = format("'%s' failed its checksum", path.c_str());
        return false;
    }
    body.assign(body_data, body_data + body_size);
    return true;
}

}  // namespace zlb
