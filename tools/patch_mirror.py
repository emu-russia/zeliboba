p = 'src/hw/soc/kermit.cpp'
s = open(p, encoding='utf-8').read()
old = """    // The SDIF0 alias has to be built after the base device exists, because
    // DeviceMirror holds a reference to it.
    d.bus.add_device(std::make_unique<DeviceMirror>(*d.sdif0, kermit::kSdif0ArmMirror, kermit::kSdifSize));
    d.sdif0_alias = kermit::detail::find_device(d.bus, "Kermit.Sdif0@mirror");"""
new = """    // There is no SDIF0 alias at 0xE2040000: that window is the peripheral
    // power/clock glue KBL configures (0x4003BBCC), while sdif.elf itself uses
    // 0xE0B00000/0xE0C00000/0xE0C10000 (its base table at 0x81009FA8)."""
assert old in s
s = s.replace(old, new, 1)
open(p, 'w', encoding='utf-8').write(s)
print('mirror removed')
