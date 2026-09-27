// zeliboba - embedded 8x8 bitmap font used by the SDL3 frontend.
//
// The UI deliberately has no font library dependency: 95 glyphs (ASCII 32..126)
// are stored as eight bit-packed rows each, taken from the public domain
// "font8x8_basic" table. Row 0 is the top scanline, bit 0 of a row byte is the
// leftmost pixel, so a glyph can be blitted by testing bits from low to high.
#pragma once

#include "common/types.h"

namespace zlb {
namespace font {

constexpr int kGlyphWidth = 8;
constexpr int kGlyphHeight = 8;
constexpr int kFirstChar = 32;   ///< ' '
constexpr int kLastChar = 126;   ///< '~'
constexpr int kGlyphCount = kLastChar - kFirstChar + 1;

/// Eight bit-packed scanlines of `c`, or nullptr when it has no glyph
/// (everything outside 32..126). Substitutes a solid block for unknown chars.
const u8* glyph(char c);

/// True when `c` has a real glyph (rather than the fallback box).
bool has_glyph(char c);

}  // namespace font
}  // namespace zlb
