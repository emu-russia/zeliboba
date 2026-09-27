"""Find the callers of an address in a decrypted Vita code segment.

The segments scedecrypt.py wrote next to the firmware (SLB2_dec/*.segNN) are raw
load images: seg01 of kernel_boot_loader.self is the 0x3BFE0 byte RX segment that
loads at 0x40020000.  Cross references are the missing piece when the debugger can
only tell you *where* something faulted, so this scans the image for the Thumb
BL/BLX encodings whose target is the address of interest and prints the few
instructions before each call, which is usually where the arguments come from.

usage: thumb_xref.py <image> <base-hex> <target-hex> [context]
"""
import struct
import sys

path = sys.argv[1]
base = int(sys.argv[2], 0)
target = int(sys.argv[3], 0)
context = int(sys.argv[4], 0) if len(sys.argv) > 4 else 8
data = open(path, "rb").read()


def halfword(offset):
    if offset + 2 > len(data):
        return None
    return struct.unpack_from("<H", data, offset)[0]


def sign_extend(value, bits):
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value


hits = []
offset = 0
while offset + 4 <= len(data):
    hw1 = halfword(offset)
    hw2 = halfword(offset + 2)
    pc = base + offset
    # Thumb-32 BL/BLX: 11110 S imm10 : 11 J1 1 J2 imm11
    if (hw1 & 0xF800) == 0xF000 and (hw2 & 0x8000) == 0x8000:
        s = (hw1 >> 10) & 1
        imm10 = hw1 & 0x3FF
        j1 = (hw2 >> 13) & 1
        j2 = (hw2 >> 11) & 1
        imm11 = hw2 & 0x7FF
        i1 = (~(j1 ^ s)) & 1
        i2 = (~(j2 ^ s)) & 1
        imm32 = (s << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1)
        imm32 = sign_extend(imm32, 25)
        dest = (pc + 4 + imm32) & 0xFFFFFFFF
        if dest == target:
            hits.append((pc, "bl/blx", hw2 & 0x1000))
        offset += 4
        continue
    # Thumb-16 conditional/unconditional branch (b<cond> / b) is 2 bytes.
    offset += 2

names = {
    0x0: "beq", 0x1: "bne", 0x2: "bcs", 0x3: "bcc", 0x4: "bmi", 0x5: "bpl", 0x6: "bvs",
    0x7: "bvc", 0x8: "bhi", 0x9: "bls", 0xA: "bge", 0xB: "blt", 0xC: "bgt", 0xD: "ble",
}
print(f"{len(hits)} call(s) to 0x{target:08X} in {path}")
for pc, kind, is_blx in hits:
    print(f"\n--- call at 0x{pc:08X} ({kind}{' BLX' if is_blx else ''}) ---")
    start = max(0, pc - base - context * 4)
    for off in range(start, pc - base + 4, 4):
        words = data[off:off + 4]
        if len(words) < 4:
            continue
        hw_a, hw_b = struct.unpack_from("<HH", data, off)
        note = ""
        if 0x2000 <= hw_a <= 0x2FFF:
            note = f"movs r{hw_a >> 8}, #{hw_a & 0xFF}"
        elif hw_a == 0x4FF0 or (hw_a & 0xFFF0) == 0xF040:
            rd = hw_b & 0xF
            i = (hw_a >> 10) & 1
            imm3 = (hw_b >> 12) & 7
            imm8 = hw_b & 0xFF
            imm12 = (i << 11) | (imm3 << 8) | imm8
            note = f"mov.w r{rd}, #0x{imm12:X}? (imm12={imm12:#x})"
        elif (hw_a & 0xF800) == 0x4800:
            note = f"ldr r{(hw_a >> 8) & 7}, [pc, #{(hw_a & 0xFF) * 4}]"
        elif (hw_a & 0xF800) == 0x6800:
            note = f"ldr r{(hw_a >> 0) & 7}, [r{(hw_a >> 3) & 7}, #0]"
        elif (hw_a & 0xF800) == 0x9000:
            note = f"str r{(hw_a >> 8) & 7}, [sp, #{(hw_a & 0xFF) * 4}]"
        elif hw_a == 0xB500 or hw_a == 0xB5F0 or (hw_a & 0xFF80) == 0xB080:
            note = "push/sub sp"
        print(f"  0x{base + off:08X}  {hw_a:04X} {hw_b:04X}   {note}")
