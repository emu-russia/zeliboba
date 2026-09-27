// zeliboba - boot chain state machine.
//
// Power-on sequence being modelled (firmware 1.04):
//
//   1. Ernie (syscon) comes up, releases the CMeP.
//   2. The ARM boot ROM reads the SLB2 container from the eMMC boot partition and
//      stages `second_loader.enp` in the shared boot SRAM, then tells the CMeP
//      through mailbox 0xE0000010 (bit 0 = "image present", bits 2+ = address).
//      We do not have the ARM boot ROM dump, so this step is modelled in C++.
//   3. The CMeP first loader validates the header, runs `process_image` (keyring
//      + Bigmac AES + Bignum RSA) and jumps to the staged image at 0x40000.
//   4. `second_loader` (CMeP) brings up the secure kernel and the ARM kernel boot
//      loader, reading the rest of SLB2 over the SC/eMMC path.
//   5. The ARM starts `kernel_boot_loader`, which mounts the user area and loads
//      the kernel ELF (`os0`), then transfers control to it.
#pragma once

#include <string>
#include <vector>

#include "common/types.h"

namespace zlb {

enum class BootStage {
    PowerOn = 0,
    ArmBootRom,
    CmepFirstLoader,
    CmepSecondLoader,
    CmepSecureKernel,
    ArmKernelBootLoader,
    KernelEntry,
    KernelRunning,
    Failed,
};

const char* to_string(BootStage stage);

struct BootStatus {
    BootStage stage = BootStage::PowerOn;
    std::string detail;
    u64 steps_in_stage = 0;
    u64 cmep_status = 0;      // last value the first loader wrote to 0xE0000000
    u32 arm_entry = 0;
    bool arm_released = false;
};

}  // namespace zlb
