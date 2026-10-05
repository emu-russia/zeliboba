// zeliboba - zdis: standalone disassembler for the three Vita processors.
//
// It reuses the same cores as the emulator, so what you see here is exactly what
// the interpreter will execute. Handy when a register's meaning has to be
// recovered from a firmware module.
//
//   zdis <mep|arm|rl78> <file> [--base 0x...] [--offset 0x...] [--count N]
//                               [--thumb] [--quiet]
#include <cstdio>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/util.h"
#include "cpu/cpu.h"
#include "cpu/factory.h"
#include "cpu/mep/mep_core.h"

using namespace zlb;

namespace {

void usage() {
    std::printf(
        "usage: zdis <mep|arm|rl78> <file> [options]\n"
        "  --base <addr>     load address (default depends on the architecture)\n"
        "  --offset <n>      skip n bytes into the file\n"
        "  --count <n>       number of instructions (default 64)\n"
        "  --thumb           start in Thumb state (ARM only)\n"
        "  --vliw            start in MeP VLIW mode (Venezia IVC2 packets)\n"
        "  --arch-raw        print raw words instead of disassembling\n");
}

u32 default_base(Arch arch) {
    switch (arch) {
        case Arch::MeP: return 0x5C000;
        case Arch::Arm: return 0x80000000;
        case Arch::Rl78: return 0x00000000;
        default: return 0x80000000;
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        usage();
        return 2;
    }

    Arch arch = Arch::Unknown;
    if (!parse_arch(argv[1], arch) || arch == Arch::Unknown) {
        std::fprintf(stderr, "zdis: unknown architecture '%s'\n", argv[1]);
        return 2;
    }

    std::string path = argv[2];
    u32 base = default_base(arch);
    u32 offset = 0;
    int count = 64;
    bool thumb = false;
    bool raw = false;
    bool vliw = false;

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        auto value = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "zdis: %s needs a value\n", arg.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--base") parse_u32(value(), base);
        else if (arg == "--offset") parse_u32(value(), offset);
        else if (arg == "--count") count = std::atoi(value().c_str());
        else if (arg == "--thumb") thumb = true;
        else if (arg == "--vliw") vliw = true;
        else if (arg == "--arch-raw") raw = true;
        else {
            std::fprintf(stderr, "zdis: unknown option '%s'\n", arg.c_str());
            return 2;
        }
    }

    auto data = read_file(path);
    if (!data) {
        std::fprintf(stderr, "zdis: cannot read %s\n", path.c_str());
        return 1;
    }
    if (offset >= data->size()) {
        std::fprintf(stderr, "zdis: offset 0x%X is past the end of the file\n", offset);
        return 1;
    }

    const size_t length = data->size() - offset;
    const u32 load_base = base;
    const u32 map_size = static_cast<u32>((length + 0xFFFF) & ~0xFFFFull);

    Bus bus;
    bus.add_ram("image", map_size, load_base, "zdis image");
    bus.load(load_base, data->data() + offset, length, "image");

    auto cpu = make_cpu(arch, bus);
    if (!cpu) {
        std::fprintf(stderr, "zdis: the %s core is not built into this binary\n", to_string(arch));
        return 1;
    }

    if (arch == Arch::Arm && thumb) {
        // Entering Thumb through the reset vector keeps the core's convention.
        cpu->reset(load_base | 1u);
    } else {
        cpu->reset(load_base);
    }

    if (vliw && arch == Arch::MeP) {
        // Venezia's VLIW operating mode: the fetch unit pulls 64 bit packets and
        // issues a core instruction together with the IVC2 slots.
        if (auto* mep = dynamic_cast<MePCore*>(cpu.get())) {
            mep->vliw_mode = true;
        } else {
            std::fprintf(stderr, "zdis: --vliw is only implemented for the MeP core\n");
            return 2;
        }
    }

    std::printf("; %s: %s, %zu bytes at 0x%08X (%s)\n", path_filename(path).c_str(), to_string(arch), length,
                load_base, thumb ? "thumb" : (vliw ? "vliw" : "default state"));

    if (raw) {
        for (size_t i = 0; i + 4 <= length; i += 4) {
            std::printf("%08X  %02X %02X %02X %02X\n", load_base + static_cast<u32>(i), (*data)[offset + i],
                        (*data)[offset + i + 1], (*data)[offset + i + 2], (*data)[offset + i + 3]);
        }
        return 0;
    }

    u32 address = load_base;
    for (int i = 0; i < count; ++i) {
        unsigned instruction_length = 0;
        std::string text = cpu->disassemble(address, instruction_length);
        if (instruction_length == 0) instruction_length = 4;

        std::string bytes;
        for (unsigned b = 0; b < instruction_length && b < 8; ++b) {
            bytes += format("%02X ", bus.read8(address + b));
        }
        std::printf("%08X  %-20s %s\n", address, bytes.c_str(), text.c_str());

        if (address - load_base + instruction_length >= length) break;
        address += instruction_length;
    }
    return 0;
}
