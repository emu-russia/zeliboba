// zeliboba - CPU core factories.
//
// Each architecture implements its factory in its own translation unit, so the
// machine layer can construct cores without knowing how they are built. Adding
// a fourth core means adding one factory function and one case in `make_cpu`.
#pragma once

#include <memory>
#include <string>

#include "bus/bus.h"
#include "cpu/cpu.h"

namespace zlb {

std::unique_ptr<Cpu> create_arm_core(Bus& bus);
std::unique_ptr<Cpu> create_mep_core(Bus& bus);
std::unique_ptr<Cpu> create_rl78_core(Bus& bus);

inline std::unique_ptr<Cpu> make_cpu(Arch arch, Bus& bus) {
    switch (arch) {
        case Arch::Arm: return create_arm_core(bus);
        case Arch::MeP: return create_mep_core(bus);
        case Arch::Rl78: return create_rl78_core(bus);
        default: return nullptr;
    }
}

}  // namespace zlb
