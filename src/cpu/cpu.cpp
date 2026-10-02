#include "cpu/cpu.h"

#include "common/log.h"

namespace zlb {

const char* to_string(Arch arch) {
    switch (arch) {
        case Arch::Arm: return "ARM";
        case Arch::MeP: return "MeP";
        case Arch::Rl78: return "RL78";
        default: return "Unknown";
    }
}

bool parse_arch(const std::string& text, Arch& out) {
    std::string lowered;
    lowered.reserve(text.size());
    for (char c : text) lowered.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    if (lowered == "arm" || lowered == "a9" || lowered == "cortex-a9") { out = Arch::Arm; return true; }
    if (lowered == "mep" || lowered == "cmep" || lowered == "f00d" || lowered == "venezia") { out = Arch::MeP; return true; }
    if (lowered == "rl78" || lowered == "ernie" || lowered == "syscon") { out = Arch::Rl78; return true; }
    if (lowered == "none" || lowered == "unknown") { out = Arch::Unknown; return true; }
    return false;
}

int Cpu::run(int64_t max_steps, const std::function<bool()>& abort) {
    int64_t done = 0;
    while (done < max_steps) {
        if (halted || undefined_instruction) break;
        if (abort && abort()) break;
        if (hit_breakpoint()) break;
        step();
        ++done;
    }
    return static_cast<int>(done);
}

void Cpu::save_state(StateWriter& writer) const {
    writer.put_u64(instructions);
    writer.put_u64(cycles);
    writer.put_bool(halted);
    writer.str(halt_reason);
    writer.put_u32(get_pc());
    writer.put_bool(undefined_instruction);
}

void Cpu::load_state(StateReader& reader) {
    instructions = reader.get_u64();
    cycles = reader.get_u64();
    halted = reader.get_bool();
    halt_reason = reader.str();
    set_pc(reader.get_u32());
    undefined_instruction = reader.get_bool();
}

}  // namespace zlb
