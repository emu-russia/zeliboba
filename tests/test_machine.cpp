// zeliboba - machine wiring self tests.
//
// These do not need a working CPU core: they verify the board description
// (memory windows, shared SRAM, fitted parts, eMMC) which is what the boot chain
// depends on.
#include "test_framework.h"

#include "bus/device.h"
#include "common/util.h"
#include "hw/cmep.h"
#include "machine/vita.h"

using namespace zlb;

namespace {

Vita& shared_machine() {
    static Vita vita;
    static bool built = false;
    if (!built) {
        VitaConfig config;
        config.rebuild_emmc = false;  // never write a multi-hundred-MiB file in tests
        vita.build(config);
        vita.reset(false);
        built = true;
    }
    return vita;
}

}  // namespace

ZLB_TEST(machine_memory_map) {
    Vita& vita = shared_machine();

    // CMeP RAM window holds the first loader and the staging buffer.
    ZLB_EXPECT_TRUE(vita.cmep_bus().is_ram(board::kCmepRamBase, board::kCmepRamSize));
    ZLB_EXPECT_TRUE(vita.cmep_bus().is_mapped(board::kCmepStackTop));
    ZLB_EXPECT_TRUE(vita.cmep_bus().is_ram(0x5C000, 16));

    // ARM DRAM, SRAM and private region.
    ZLB_EXPECT_TRUE(vita.arm_bus().is_ram(0x80000000, 0x1000));
    ZLB_EXPECT_TRUE(vita.arm_bus().is_ram(0x83FFF000, 0x1000));
    ZLB_EXPECT_TRUE(vita.arm_bus().is_ram(0x1F000000, 0x1000));
    ZLB_EXPECT_TRUE(vita.arm_bus().is_ram(0x40000000, 0x1000));

    // The syscon keeps its 1 MiB flash.
    ZLB_EXPECT_TRUE(vita.syscon_bus().is_ram(0x00000000, 0x1000));
    ZLB_EXPECT_TRUE(vita.syscon_bus().is_ram(0x000FF000, 0x1000));
}

ZLB_TEST(machine_shared_boot_sram_is_aliased) {
    Vita& vita = shared_machine();

    vita.cmep_bus().write32(board::kSharedSramBase + 0x40, 0x5A5A1234);
    ZLB_EXPECT_EQ(vita.arm_bus().read32(board::kSharedSramBase + 0x40), 0x5A5A1234u);

    vita.arm_bus().write8(board::kSharedSramBase + 0x44, 0x99);
    ZLB_EXPECT_EQ(vita.cmep_bus().read8(board::kSharedSramBase + 0x44), 0x99u);
}

ZLB_TEST(machine_cores_exist) {
    Vita& vita = shared_machine();
    ZLB_EXPECT_TRUE(vita.cmep() != nullptr);
    ZLB_EXPECT_TRUE(vita.arm() != nullptr);
    // The RL78 core is created by the Ernie block; it is optional while the
    // syscon firmware path is being brought up.
    if (vita.cmep()) ZLB_EXPECT_TRUE(vita.cmep()->arch() == Arch::MeP);
    if (vita.arm()) ZLB_EXPECT_TRUE(vita.arm()->arch() == Arch::Arm);
}

ZLB_TEST(machine_first_loader_is_fitted) {
    Vita& vita = shared_machine();
    // The prototype first loader starts with `jmp 0x5C004` (0x05C0D828).
    const u32 first_word = vita.cmep_bus().read32(board::kFirstLoaderBase);
    ZLB_EXPECT_EQ(first_word, 0x05C0D828u);
}

ZLB_TEST(machine_cmep_devices_present) {
    Vita& vita = shared_machine();
    Bus& bus = vita.cmep_bus();

    struct Expectation {
        const char* name;
        u32 address;
    };
    const Expectation expected[] = {
        {"CMeP.Keyring", cmep::kKeyringBase},
        {"CMeP.Mailbox", cmep::kMailboxBase},
        {"CMeP.Bigmac", cmep::kBigmacBase},
        {"CMeP.Bignum", cmep::kBignumBase},
        {"CMeP.Strap", cmep::kStrapBase},
    };

    for (const Expectation& item : expected) {
        Device* device = bus.find_device(item.address);
        ZLB_EXPECT_TRUE(device != nullptr);
        if (device) {
            // The smallest matching window must be the device we asked for.
            std::string name = device->name();
            ZLB_EXPECT_TRUE(name == item.name || bus.find_device(item.address)->handles(item.address));
        }
    }
}

ZLB_TEST(machine_boot_stage_progresses) {
    Vita& vita = shared_machine();
    // After a reset the machine is at the ARM boot ROM model stage, and the CMeP
    // first loader is ready to run.
    vita.reset(false);
    ZLB_EXPECT_TRUE(vita.stage() == BootStage::ArmBootRom || vita.stage() == BootStage::CmepFirstLoader);
    ZLB_EXPECT_FALSE(vita.kernel_started());
    ZLB_EXPECT_TRUE(!vita.plan_boot().empty());
}

ZLB_TEST(machine_workspace_paths_resolve) {
    const std::string resolved = resolve_workspace_path("dumps/vita_prototype_bootrom.bin");
    ZLB_EXPECT_TRUE(file_exists(resolved));
    ZLB_EXPECT_TRUE(resolve_workspace_path("C:/absolute/path") == "C:/absolute/path");
}

// A breakpoint has to be exact even though a scheduler slice runs a whole budget
// of instructions per core: the debugger installs `pc_hook` for that, and the
// slice must end *before* the matching instruction executes.
ZLB_TEST(machine_pc_hook_stops_the_slice_before_the_instruction) {
    Vita& vita = shared_machine();
    vita.reset(false);

    Cpu* arm0 = vita.arm_core(0);
    ZLB_EXPECT_TRUE(arm0 != nullptr);
    if (!arm0) return;

    // Let the core run a little so its PC is inside real code, then arm the hook
    // on the instruction it is about to execute.
    for (int i = 0; i < 8; ++i) vita.run_slice();
    const u32 armed_pc = arm0->get_pc();
    const u64 insns_before = arm0->instructions;

    vita.pc_hook = [armed_pc](Arch arch, int, u32 pc) {
        return arch == Arch::Arm && pc == armed_pc;
    };
    vita.run_slice();

    ZLB_EXPECT_TRUE(vita.pc_hook_stopped());
    ZLB_EXPECT_EQ(static_cast<u32>(vita.pc_hook_pc()), armed_pc);
    ZLB_EXPECT_EQ(static_cast<u32>(arm0->get_pc()), armed_pc);
    ZLB_EXPECT_TRUE(arm0->instructions == insns_before);

    // With the hook gone the same slice makes progress again.
    vita.clear_pc_hook_stop();
    vita.pc_hook = nullptr;
    vita.run_slice();
    ZLB_EXPECT_FALSE(vita.pc_hook_stopped());
    ZLB_EXPECT_TRUE(arm0->instructions > insns_before);
}
