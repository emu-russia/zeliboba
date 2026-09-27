p = r'src\bus\bus.cpp'
s = open(p, encoding='utf-8').read()
subs = [
    ("void Bus::write8(u32 address, u8 value) {\n    stats.writes++;",
     "void Bus::write8(u32 address, u8 value) {\n    stats.writes++;\n    note_write_trap(address, 1, value);"),
    ("void Bus::write16(u32 address, u16 value) {\n    stats.writes++;",
     "void Bus::write16(u32 address, u16 value) {\n    stats.writes++;\n    note_write_trap(address, 2, value);"),
    ("void Bus::write32(u32 address, u32 value) {\n    stats.writes++;",
     "void Bus::write32(u32 address, u32 value) {\n    stats.writes++;\n    note_write_trap(address, 4, value);"),
    ("void Bus::write64(u32 address, u64 value) {\n    stats.writes++;",
     "void Bus::write64(u32 address, u64 value) {\n    stats.writes++;\n    note_write_trap(address, 8, value);"),
    ("bool Bus::load(u32 address, const void* data, size_t length, const std::string& tag) {\n    if (length == 0) return true;",
     "bool Bus::load(u32 address, const void* data, size_t length, const std::string& tag) {\n    if (length == 0) return true;\n    for (size_t i = 0; i < length; ++i) note_write_trap(address + static_cast<u32>(i), 0, 0);"),
]
for old, new in subs:
    if old not in s:
        print('MISS:', old.splitlines()[0])
        continue
    s = s.replace(old, new, 1)
open(p, 'w', encoding='utf-8').write(s)
print('ok')
