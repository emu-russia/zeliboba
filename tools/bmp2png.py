"""Minimal BMP -> PNG converter (no third party modules).

Handles the 24-bit and 32-bit uncompressed BMPs that SDL_SaveBMP writes.
Usage: python bmp2png.py input.bmp output.png
"""
import struct
import sys
import zlib


def read_bmp(path):
    data = open(path, 'rb').read()
    if data[:2] != b'BM':
        raise ValueError('not a BMP')
    pixel_offset = struct.unpack_from('<I', data, 10)[0]
    header_size = struct.unpack_from('<I', data, 14)[0]
    width, height = struct.unpack_from('<ii', data, 18)
    bpp = struct.unpack_from('<H', data, 28)[0]
    compression = struct.unpack_from('<I', data, 30)[0]
    masks = None
    if compression == 3:  # BI_BITFIELDS, what SDL_SaveBMP writes for ARGB8888
        masks = struct.unpack_from('<III', data, 54)
    elif compression != 0:
        raise ValueError('unsupported BMP compression %d' % compression)
    if bpp not in (24, 32):
        raise ValueError('unsupported bpp %d' % bpp)

    def shift_of(mask):
        if mask == 0:
            return 0, 0
        shift = 0
        while (mask & 1) == 0:
            mask >>= 1
            shift += 1
        return shift, mask

    bottom_up = height > 0
    height = abs(height)
    stride = ((width * bpp // 8) + 3) & ~3
    rows = []
    for y in range(height):
        src_y = (height - 1 - y) if bottom_up else y
        base = pixel_offset + src_y * stride
        row = bytearray()
        for x in range(width):
            off = base + x * (bpp // 8)
            if masks:
                value = int.from_bytes(data[off:off + bpp // 8], 'little')
                rs, rm = shift_of(masks[0])
                gs, gm = shift_of(masks[1])
                bs, bm = shift_of(masks[2])
                r = ((value >> rs) & rm) * 255 // rm if rm else 0
                g = ((value >> gs) & gm) * 255 // gm if gm else 0
                b = ((value >> bs) & bm) * 255 // bm if bm else 0
            else:
                b, g, r = data[off], data[off + 1], data[off + 2]
            row += bytes((r, g, b))
        rows.append(bytes(row))
    return width, height, rows


def write_png(path, width, height, rows):
    raw = bytearray()
    for row in rows:
        raw.append(0)  # filter: none
        raw += row

    def chunk(tag, payload):
        out = struct.pack('>I', len(payload)) + tag + payload
        return out + struct.pack('>I', zlib.crc32(tag + payload) & 0xFFFFFFFF)

    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(bytes(raw), 9))
    png += chunk(b'IEND', b'')
    open(path, 'wb').write(png)


if __name__ == '__main__':
    w, h, rows = read_bmp(sys.argv[1])
    write_png(sys.argv[2], w, h, rows)
    print('%s -> %s (%dx%d)' % (sys.argv[1], sys.argv[2], w, h))
