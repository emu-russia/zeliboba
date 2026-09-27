// zeliboba - common integer types and small helpers.
//
// The emulator targets three very different instruction sets at once (ARMv7-A,
// Toshiba MeP-c5 and Renesas RL78), so every core is written against this tiny
// set of fixed width aliases instead of the platform's int/long.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace zlb {

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using s8 = std::int8_t;
using s16 = std::int16_t;
using s32 = std::int32_t;
using s64 = std::int64_t;

using f32 = float;
using f64 = double;

constexpr u32 KB = 1024u;
constexpr u32 MB = 1024u * 1024u;
constexpr u32 GB = 1024u * 1024u * 1024u;

// ---------------------------------------------------------------------------
// Bit helpers
// ---------------------------------------------------------------------------

template <typename T>
constexpr bool test_bit(T value, unsigned bit) {
    return ((value >> bit) & T(1)) != T(0);
}

template <typename T>
constexpr T bit_mask(unsigned hi, unsigned lo) {
    const unsigned width = hi - lo + 1;
    if (width >= sizeof(T) * 8) return static_cast<T>(~T(0));
    return static_cast<T>(((static_cast<u64>(1) << width) - 1) << lo);
}

template <typename T>
constexpr T extract_bits(T value, unsigned hi, unsigned lo) {
    return static_cast<T>((value >> lo) & (bit_mask<T>(hi, lo) >> lo));
}

template <typename T>
constexpr T set_bits(T value, unsigned hi, unsigned lo, T bits) {
    const T mask = bit_mask<T>(hi, lo);
    return static_cast<T>((value & ~mask) | ((bits << lo) & mask));
}

template <typename T>
constexpr T sign_extend(T value, unsigned bits) {
    const T m = T(1) << (bits - 1);
    return static_cast<T>((value ^ m) - m);
}

template <typename T>
constexpr T rotl(T value, unsigned n) {
    const unsigned width = sizeof(T) * 8;
    n %= width;
    if (n == 0) return value;
    return static_cast<T>((value << n) | (value >> (width - n)));
}

template <typename T>
constexpr T rotr(T value, unsigned n) {
    const unsigned width = sizeof(T) * 8;
    n %= width;
    if (n == 0) return value;
    return static_cast<T>((value >> n) | (value << (width - n)));
}

template <typename T>
constexpr T arithmetic_shift_right(T value, unsigned n) {
    using S = std::make_signed_t<T>;
    return static_cast<T>(static_cast<S>(value) >> n);
}

// ---------------------------------------------------------------------------
// Endian helpers (all target machines are little endian except none - but the
// bus keeps the flag so that big endian experiments stay possible).
// ---------------------------------------------------------------------------

inline u16 bswap16(u16 v) { return static_cast<u16>((v >> 8) | (v << 8)); }
inline u32 bswap32(u32 v) {
    return ((v & 0xFF000000u) >> 24) | ((v & 0x00FF0000u) >> 8) | ((v & 0x0000FF00u) << 8) |
           ((v & 0x000000FFu) << 24);
}
inline u64 bswap64(u64 v) {
    return (static_cast<u64>(bswap32(static_cast<u32>(v))) << 32) | bswap32(static_cast<u32>(v >> 32));
}

}  // namespace zlb
