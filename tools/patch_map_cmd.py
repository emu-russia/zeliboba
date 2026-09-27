p = 'src/debug/debugger.cpp'
s = open(p, encoding='utf-8').read()

anchor = """    if (command == "devices") {
        emit(cmd_devices(args));
        return true;
    }"""
assert anchor in s

new = """    if (command == "devices") {
        emit(cmd_devices(args));
        return true;
    }
    if (command == "map") {
        // Which device/region does an address actually resolve to, and what does
        // it read?  The fast page table and Bus::find_device() can disagree, so
        // this prints the page kind as well.
        const u32 address = arg_address(args, 0, active_core()->get_pc());
        Bus* buses[3] = {&vita_.cmep_bus(), &vita_.arm_bus(), &vita_.syscon_bus()};
        const char* names[3] = {"mep", "arm", "rl78"};
        std::string out;
        for (int i = 0; i < 3; ++i) {
            Bus* bus = buses[i];
            Device* device = bus->find_device(address);
            const MemRegion* region = bus->region_at(address, 1);
            out += format("%-5s device=%-28s region=%-16s read32=0x%08X\\n", names[i],
                          device ? device->name().c_str() : "-", region ? region->name.c_str() : "-",
                          static_cast<unsigned>(bus->read32(address)));
        }
        for (const std::string& line : split(out, '\\n')) {
            if (!line.empty()) emit(line);
        }
        return true;
    }"""
s = s.replace(anchor, new, 1)

old_help = '"devices", "devget", "devset", "emmc",'
new_help = '"devices", "map", "devget", "devset", "emmc",'
assert old_help in s
s = s.replace(old_help, new_help, 1)

old_usage = '  devices                list MMIO devices'
new_usage = '''  devices                list MMIO devices
  map <addr>             which device/region an address resolves to'''
if old_usage in s:
    s = s.replace(old_usage, new_usage, 1)
else:
    print('warning: usage line not found')

open(p, 'w', encoding='utf-8').write(s)
print('map command added')
