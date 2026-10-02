// zeliboba - binary save-state stream.
//
// A save state is a length-checked, sectioned byte stream. Every object that
// owns mutable state implements `save_state(StateWriter&)` / `load_state(
// StateReader&)`; the machine walks the object graph and writes one section per
// object. Sections are length prefixed, so a reader can detect a layout
// mismatch (an old state file against a new build) before it has applied
// anything to the live machine.
//
// The format is little endian and deliberately dumb: no compression, no
// pointers. A writer and a reader of the same build always agree on the order,
// and a mismatch is a hard error rather than a silent misread. (The scalar
// helpers are named put_*/get_* rather than u8/u16/... because a member called
// `u8` would hide the `zlb::u8` type inside the class body.)
//
// RAM is written with a zero-page bitmap (see `state_write_pages`): a freshly
// reset machine has almost no non-zero pages, so states taken early in a boot
// are a few kilobytes instead of hundreds of megabytes.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "common/types.h"

namespace zlb {

/// Format version of the whole state file. Bump when a section layout changes
/// in a way an older reader cannot detect by name alone.
constexpr u32 kStateFormatVersion = 1;

// ---------------------------------------------------------------------------
// Writer
// ---------------------------------------------------------------------------

class StateWriter {
public:
    // ---- scalars ---------------------------------------------------------

    void put_u8(u8 value) { out_.push_back(value); }
    void put_u16(u16 value) {
        put_u8(static_cast<u8>(value));
        put_u8(static_cast<u8>(value >> 8));
    }
    void put_u32(u32 value) {
        put_u16(static_cast<u16>(value));
        put_u16(static_cast<u16>(value >> 16));
    }
    void put_u64(u64 value) {
        put_u32(static_cast<u32>(value));
        put_u32(static_cast<u32>(value >> 32));
    }
    void put_i32(int value) { put_u32(static_cast<u32>(value)); }
    void put_bool(bool value) { put_u8(value ? 1 : 0); }
    void put_f32(float value) {
        u32 bits = 0;
        std::memcpy(&bits, &value, 4);
        put_u32(bits);
    }
    void put_f64(double value) {
        u64 bits = 0;
        std::memcpy(&bits, &value, 8);
        put_u64(bits);
    }

    /// Raw POD copy (trivially copyable only).
    template <typename T>
    void put_pod(const T& value) {
        bytes(&value, sizeof(T));
    }

    void bytes(const void* data, size_t size) {
        if (size == 0) return;
        const u8* p = static_cast<const u8*>(data);
        out_.insert(out_.end(), p, p + size);
    }

    void str(const std::string& text) {
        put_u32(static_cast<u32>(text.size()));
        bytes(text.data(), text.size());
    }

    /// Any container with size() and range-for (std::vector, std::array, ...).
    template <typename C, typename F>
    void list(const C& container, F each) {
        put_u32(static_cast<u32>(container.size()));
        for (const auto& element : container) each(element);
    }

    /// A fixed-size container (std::array, C array): no length is written, the
    /// reader knows the size from the build.
    template <typename C, typename F>
    void fixed(const C& container, F each) {
        for (const auto& element : container) each(element);
    }

    template <typename M, typename F>
    void map(const M& container, F each) {
        put_u32(static_cast<u32>(container.size()));
        for (const auto& entry : container) each(entry.first, entry.second);
    }

    // ---- sections --------------------------------------------------------

    /// Open a named section whose length is patched by the matching `end()`.
    void begin(const char* section);
    void end();

    // ---- output ----------------------------------------------------------

    const std::vector<u8>& data() const { return out_; }
    std::vector<u8>& data() { return out_; }
    size_t size() const { return out_.size(); }

private:
    std::vector<u8> out_;
    std::vector<size_t> open_;
};

// ---------------------------------------------------------------------------
// Reader
// ---------------------------------------------------------------------------

class StateReader {
public:
    StateReader(const u8* data, size_t size) : begin_(data), cursor_(data), end_(data + size) {}
    explicit StateReader(const std::vector<u8>& data) : StateReader(data.data(), data.size()) {}

    // ---- scalars ---------------------------------------------------------

    u8 get_u8() {
        if (!have(1)) return 0;
        return *cursor_++;
    }
    u16 get_u16() {
        const u16 lo = get_u8();
        const u16 hi = get_u8();
        return static_cast<u16>(lo | (hi << 8));
    }
    u32 get_u32() {
        const u32 lo = get_u16();
        const u32 hi = get_u16();
        return lo | (hi << 16);
    }
    u64 get_u64() {
        const u64 lo = get_u32();
        const u64 hi = get_u32();
        return lo | (hi << 32);
    }
    int get_i32() { return static_cast<int>(get_u32()); }
    bool get_bool() { return get_u8() != 0; }
    float get_f32() {
        const u32 bits = get_u32();
        float value = 0;
        std::memcpy(&value, &bits, 4);
        return value;
    }
    double get_f64() {
        const u64 bits = get_u64();
        double value = 0;
        std::memcpy(&value, &bits, 8);
        return value;
    }

    template <typename T>
    T get_pod() {
        T value{};
        bytes(&value, sizeof(T));
        return value;
    }

    void bytes(void* out, size_t size) {
        if (!have(size)) return;
        std::memcpy(out, cursor_, size);
        cursor_ += size;
    }

    std::string str() {
        const u32 size = get_u32();
        if (!ok_ || !have(size)) return {};
        std::string text(reinterpret_cast<const char*>(cursor_), size);
        cursor_ += size;
        return text;
    }

    /// Read a container back. `C` needs clear() and push_back().
    template <typename C, typename F>
    void list(C& container, F each) {
        const u32 count = get_u32();
        container.clear();
        for (u32 i = 0; i < count && ok_; ++i) {
            typename C::value_type element{};
            each(element);
            container.push_back(std::move(element));
        }
    }

    /// Read a fixed-size container element by element (no length on the wire).
    template <typename C, typename F>
    void fixed(C& container, F each) {
        for (auto& element : container) {
            if (!ok_) return;
            each(element);
        }
    }

    template <typename M, typename F>
    void map(M& container, F each) {
        const u32 count = get_u32();
        container.clear();
        for (u32 i = 0; i < count && ok_; ++i) {
            typename M::key_type key{};
            typename M::mapped_type value{};
            each(key, value);
            container.emplace(std::move(key), std::move(value));
        }
    }

    // ---- sections --------------------------------------------------------

    /// Open `section`; fails (and sets `ok()` false) when the stream holds a
    /// different section or is truncated. A successful call must be matched by
    /// `end()`, which skips any bytes the loader did not consume.
    bool begin(const char* section);
    /// Close the current section, skipping unread bytes.
    void end();

    // ---- errors ----------------------------------------------------------

    bool ok() const { return ok_; }
    const std::string& error() const { return error_; }
    bool at_end() const { return cursor_ >= end_; }
    size_t remaining() const { return static_cast<size_t>(end_ - cursor_); }

    /// Record the first error (later errors are ignored so the message names the
    /// cause, not the symptom).
    void fail(const std::string& message);

private:
    bool have(size_t size) {
        if (!ok_) return false;
        if (static_cast<size_t>(end_ - cursor_) < size) {
            fail("state file is truncated");
            return false;
        }
        return true;
    }

    const u8* begin_;
    const u8* cursor_;
    const u8* end_;
    /// End offset of every currently open section, innermost last.
    std::vector<const u8*> section_ends_;
    bool ok_ = true;
    std::string error_;
};

// ---------------------------------------------------------------------------
// RAM helper
// ---------------------------------------------------------------------------

constexpr size_t kStatePageSize = 4096;

/// Write `size` bytes as a zero-page bitmap plus the non-zero pages only.
/// Returns the number of payload bytes written (not counting the bitmap).
size_t state_write_pages(StateWriter& writer, const u8* data, size_t size);
/// Read the inverse of `state_write_pages` into `data` (which must have the same
/// size the writer saw; a mismatch is a load error).
void state_read_pages(StateReader& reader, u8* data, size_t size);

// ---------------------------------------------------------------------------
// File helpers
// ---------------------------------------------------------------------------

/// Serialise `body` (which must already contain its own header/sections) into
/// `path` with the shared file header, atomically (write + rename).
bool state_write_file(const std::string& path, const StateWriter& body, std::string& error);
/// Read a state file written by `state_write_file`, verifying magic/version.
bool state_read_file(const std::string& path, std::vector<u8>& body, std::string& error);

}  // namespace zlb
