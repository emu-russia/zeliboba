p = 'src/debug/debugger.cpp'
s = open(p, encoding='utf-8').read()

old = """        for (const std::string& line : split(out, '\\n')) {
            if (!line.empty()) emit(line);
        }
        return true;
    }"""
new = """        for (const std::string& line : split(out, '\\n')) {
            if (!line.empty()) emit(line);
        }
        // Every device that claims the address: the fast page table keeps the last
        // one added, while find_device() keeps the smallest window, so listing both
        // makes a shadowing window obvious.
        for (int i = 0; i < 3; ++i) {
            for (const auto& device : buses[i]->devices()) {
                if (!device->handles(address)) continue;
                emit(format("%-5s candidate %-28s base=0x%08X size=0x%X", names[i], device->name().c_str(),
                            device->base(), device->size()));
            }
        }
        return true;
    }"""
assert old in s
s = s.replace(old, new, 1)
open(p, 'w', encoding='utf-8').write(s)
print('map extended')
