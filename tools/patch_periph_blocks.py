p = 'src/hw/soc/kermit.cpp'
s = open(p, encoding='utf-8').read()

old = """    // Peripheral power/clock glue and the boot handshake window: both are windows
    // KBL only polls, see PeripheralPower / BootHandshake.
    d.bus.add_device(std::make_unique<kermit::PeripheralPower>("Kermit.PeriphPower", kermit::kPeriphPowerBase,
                                                              kermit::kPeriphPowerSize));"""
new = """    // Peripheral power/clock glue and the boot handshake window: both are windows
    // KBL only polls, see PeripheralPower / BootHandshake.  One instance per 64 KiB
    // block, so the register offsets stay the block-relative ones KBL uses.
    for (u32 i = 0; i < kermit::kPeriphPowerCount; ++i) {
        d.bus.add_device(std::make_unique<kermit::PeripheralPower>(
            format("Kermit.PeriphPower%u", i), kermit::kPeriphPowerBase + i * kermit::kPeriphPowerBlock,
            kermit::kPeriphPowerBlock));
    }"""
assert old in s
s = s.replace(old, new, 1)

old_owned = """           name == "Kermit.PeriphPower" || name == "Kermit.BootHandshake";"""
new_owned = """           name.compare(0, 18, "Kermit.PeriphPower") == 0 || name == "Kermit.BootHandshake";"""
assert old_owned in s
s = s.replace(old_owned, new_owned, 1)

open(p, 'w', encoding='utf-8').write(s)
print('per-block instances')

p2 = 'src/hw/soc/soc_internal.h'
t = open(p2, encoding='utf-8').read()
old2 = """constexpr u32 kPeriphPowerBase = 0xE2030000;
constexpr u32 kPeriphPowerSize = 0x00070000;"""
new2 = """constexpr u32 kPeriphPowerBase = 0xE2030000;
constexpr u32 kPeriphPowerBlock = 0x00010000;   ///< one 64 KiB window per device
constexpr u32 kPeriphPowerCount = 7;            ///< 0xE2030000 .. 0xE2090000"""
assert old2 in t
t = t.replace(old2, new2, 1)
open(p2, 'w', encoding='utf-8').write(t)
print('constants updated')
