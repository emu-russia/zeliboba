// zeliboba - little ARM/Thumb probe used to inspect instruction behaviour that the
// unit tests only assert indirectly.  Prints the registers after every step.
//
// Usage: arm_probe <hex-bytes> [reset-address] [steps] [--reg rN=V]... [--cpsr V] [--one]
//
//   <hex-bytes>      instruction bytes, e.g. "c5f30723"
//   [reset-address]  default 0x80000001 (Thumb); pass 0x80000000 for ARM
//   [steps]          default 8
//   --reg rN=V       seed a register (hex or 0x-prefixed), repeatable
//   --cpsr V         seed the flags/mode word (default 0x1F, user mode, ARM state off)
//   --one            print only the machine-readable RESULT line
//
// The RESULT line is what tools/../scratch sweep scripts compare against Unicorn:
//   RESULT r0=... r1=... ... r15=... cpsr=... undef=<0|1>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "cpu/arm/arm_core.h"

using namespace zlb;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: arm_probe <hex-bytes> [reset-address] [steps] [--reg rN=V]... "
                     "[--cpsr V] [--one]\n");
        return 2;
    }
    std::vector<u8> code;
    for (const char* p = argv[1]; *p;) {
        while (*p == ' ') ++p;
        if (!*p) break;
        char pair[3] = {p[0], p[1], 0};
        code.push_back(static_cast<u8>(std::strtoul(pair, nullptr, 16)));
        p += 2;
    }
    const u32 base = 0x80000000u;
    u32 reset = base | 1u;
    int steps = 8;
    u32 seed_cpsr = 0x1Fu;
    bool one = false;
    std::vector<std::pair<int, u32>> seeds;

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--one") {
            one = true;
        } else if (arg == "--cpsr" && i + 1 < argc) {
            seed_cpsr = static_cast<u32>(std::strtoul(argv[++i], nullptr, 0));
        } else if (arg == "--reg" && i + 1 < argc) {
            const char* spec = argv[++i];
            const char* eq = std::strchr(spec, '=');
            if (eq == nullptr || (spec[0] != 'r' && spec[0] != 'R')) continue;
            const int index = std::atoi(spec + 1);
            if (index < 0 || index > 15) continue;
            seeds.emplace_back(index, static_cast<u32>(std::strtoul(eq + 1, nullptr, 0)));
        } else if (arg.rfind("0x", 0) == 0 || arg.rfind("0X", 0) == 0 ||
                   (arg[0] >= '0' && arg[0] <= '9')) {
            if (i == 2) reset = static_cast<u32>(std::strtoul(arg.c_str(), nullptr, 0));
            else if (i == 3) steps = std::atoi(arg.c_str());
        }
    }
    if (steps < 1) steps = 1;

    Bus bus;
    bus.add_ram("main", 1024u * 1024u, base, "probe");
    ArmCore cpu(bus);
    for (size_t i = 0; i < code.size(); ++i) bus.write8(base + static_cast<u32>(i), code[i]);

    cpu.reset(reset);
    cpu.cpsr = (cpu.cpsr & ~0x1Fu) | (seed_cpsr & 0x1Fu);
    cpu.thumb = (reset & 1u) != 0u;
    for (const auto& seed : seeds) cpu.r[seed.first] = seed.second;
    if (!one) {
        std::printf("reset: pc=%08X thumb=%d cpsr=%08X\n", cpu.get_pc(), cpu.thumb ? 1 : 0,
                    cpu.cpsr);
    }
    for (int i = 0; i < steps; ++i) {
        cpu.step();
        if (!one) {
            std::printf("step %2d: pc=%08X r0=%08X r1=%08X r2=%08X r3=%08X r4=%08X cpsr=%08X "
                        "[%c%c%c%c]%s\n",
                        i + 1, cpu.get_pc(), cpu.r[0], cpu.r[1], cpu.r[2], cpu.r[3], cpu.r[4],
                        cpu.cpsr, (cpu.cpsr & (1u << 31)) ? 'N' : 'n',
                        (cpu.cpsr & (1u << 30)) ? 'Z' : 'z', (cpu.cpsr & (1u << 29)) ? 'C' : 'c',
                        (cpu.cpsr & (1u << 28)) ? 'V' : 'v',
                        cpu.undefined_instruction ? "  <undefined>" : "");
        }
        if (cpu.undefined_instruction || cpu.halted) break;
    }
    std::printf("RESULT");
    for (int i = 0; i < 16; ++i) std::printf(" r%d=%08X", i, cpu.r[i]);
    std::printf(" cpsr=%08X undef=%d\n", cpu.cpsr, cpu.undefined_instruction ? 1 : 0);
    return 0;
}
