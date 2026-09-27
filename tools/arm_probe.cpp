// zeliboba - little ARM/Thumb probe used to inspect instruction behaviour that the
// unit tests only assert indirectly.  Prints the registers after every step.
//
// Usage: arm_probe <hex-bytes> [reset-address] [steps]
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
        std::fprintf(stderr, "usage: arm_probe <hex-bytes> [reset-address] [steps]\n");
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
    const u32 reset = argc > 2 ? static_cast<u32>(std::strtoul(argv[2], nullptr, 0)) : (base | 1u);
    const int steps = argc > 3 ? std::atoi(argv[3]) : 8;

    Bus bus;
    bus.add_ram("main", 128u * 1024u, base, "probe");
    ArmCore cpu(bus);
    for (size_t i = 0; i < code.size(); ++i) bus.write8(base + static_cast<u32>(i), code[i]);

    cpu.reset(reset);
    std::printf("reset: pc=%08X thumb=%d cpsr=%08X\n", cpu.get_pc(), cpu.thumb ? 1 : 0, cpu.cpsr);
    for (int i = 0; i < steps; ++i) {
        cpu.step();
        std::printf("step %2d: pc=%08X r0=%08X r1=%08X r2=%08X r3=%08X r4=%08X cpsr=%08X "
                    "[%c%c%c%c]%s\n",
                    i + 1, cpu.get_pc(), cpu.r[0], cpu.r[1], cpu.r[2], cpu.r[3], cpu.r[4], cpu.cpsr,
                    (cpu.cpsr & (1u << 31)) ? 'N' : 'n', (cpu.cpsr & (1u << 30)) ? 'Z' : 'z',
                    (cpu.cpsr & (1u << 29)) ? 'C' : 'c', (cpu.cpsr & (1u << 28)) ? 'V' : 'v',
                    cpu.undefined_instruction ? "  <undefined>" : "");
        if (cpu.undefined_instruction || cpu.halted) break;
    }
    return 0;
}
