p = 'src/debug/debugger.cpp'
s = open(p, encoding='utf-8').read()

old = """            out += format("%-5s device=%-28s region=%-16s read32=0x%08X\\n", names[i],
                          device ? device->name().c_str() : "-", region ? region->name.c_str() : "-",
                          static_cast<unsigned>(bus->read32(address)));"""
new = """            out += format("%-5s device=%-28s region=%-16s read32=0x%08X direct=0x%08X\\n", names[i],
                          device ? device->name().c_str() : "-", region ? region->name.c_str() : "-",
                          static_cast<unsigned>(bus->read32(address)),
                          device ? static_cast<unsigned>(device->read(address, 4)) : 0u);"""
assert old in s
s = s.replace(old, new, 1)

old2 = """        // Every device that claims the address: the fast page table keeps the last
        // one added, while find_device() keeps the smallest window, so listing both
        // makes a shadowing window obvious."""
new2 = """        // A RAM region marks whole pages, and it may cover the page without covering
        // the address, so report those too.
        for (int i = 0; i < 3; ++i) {
            const u32 index = address >> 12;
            for (const auto& region : buses[i]->regions()) {
                if (!region.mapped || region.size == 0) continue;
                const u32 first = region.base >> 12;
                const u32 last = (region.base + region.size - 1) >> 12;
                if (index < first || index > last) continue;
                emit(format("%-5s page-region %-20s base=0x%08X size=0x%X", names[i], region.name.c_str(),
                            region.base, region.size));
            }
        }
        // Every device that claims the address: the fast page table keeps the last
        // one added, while find_device() keeps the smallest window, so listing both
        // makes a shadowing window obvious."""
assert old2 in s
s = s.replace(old2, new2, 1)
open(p, 'w', encoding='utf-8').write(s)
print('map diagnostics extended')
