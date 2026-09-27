p = 'src/hw/soc/uart.cpp'
s = open(p, encoding='utf-8').read()

old = """    define(kScr, "UART_SCR", 0);
}"""
new = """    define(kScr, "UART_SCR", 0);
    // SoC glue in the same window: KBL's peripheral bring-up helper (0x4003BBCC)
    // treats every block in its table the same way - it writes +0x04/+0x10/+0x20/
    // +0x30/+0x40/+0x50/+0x60/+0x64, waits for a ready bit at +0x28 (bit 8 at
    // 0x4003BC98, bit 9 at 0x4003BCCC) and then writes +0x70.  For this block that
    // is the UART's own bring-up (it writes a divisor of 0x77E and 8N1), so the
    // status reports "powered and clocked".
    define(kPeripheralStatus, "UART_PERIPH_STATUS", kPeripheralReady, 4);
}"""
assert old in s
s = s.replace(old, new, 1)

old2 = """constexpr u8 kMcrLoopback = 0x10;"""
new2 = """constexpr u8 kMcrLoopback = 0x10;

// Peripheral glue (see the constructor): +0x28 is the block's ready status.
constexpr u32 kPeripheralStatus = 0x28;
constexpr u32 kPeripheralReady = 0x0300;  // bit 8 = powered, bit 9 = clocked"""
assert old2 in s
s = s.replace(old2, new2, 1)

# Make the read path return it explicitly (read_word's switch might not know 0x28).
old3 = """u64 Uart::read_word(u32 offset, u64 stored) {"""
new3 = """u64 Uart::read_word(u32 offset, u64 stored) {
    if (offset == kPeripheralStatus) return kPeripheralReady;"""
assert old3 in s
s = s.replace(old3, new3, 1)
open(p, 'w', encoding='utf-8').write(s)
print('UART glue status added')
