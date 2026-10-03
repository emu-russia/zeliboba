// zeliboba - machine wiring self tests.
//
// These do not need a working CPU core: they verify the board description
// (memory windows, shared SRAM, fitted parts, eMMC) which is what the boot chain
// depends on.
#include "test_framework.h"

#include <cstdlib>
#include <filesystem>

#include "bus/device.h"
#include "common/util.h"
#include "cpu/arm/arm_core.h"
#include "cpu/mep/mep_core.h"
#include "hw/cmep.h"
#include "hw/emmc.h"
#include "hw/soc.h"
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
        // Cold reset: `Bus::reset()` clears every RAM region, so a *warm* reset
        // straight after build() would drop the fitted first loader (and the
        // tests below check exactly that it is there).
        vita.reset(true);
        built = true;
    }
    return vita;
}

struct TemporarySlb2Card {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("zlb_machine_slb2_" + std::to_string(::rand()) + ".img");

    explicit TemporarySlb2Card(const std::vector<Slb2Entry>& entries) {
        EmmcCard card;
        ZLB_EXPECT_TRUE(card.attach(path.string(), true, 2ull * MB + emmc_layout::kBootAreaSize +
                                                            emmc_layout::kRpmbSize));
        const auto container = build_slb2(entries);
        ZLB_EXPECT_TRUE(card.write_bytes(EmmcPartition::Boot0, 0, container.data(), container.size()));
    }

    ~TemporarySlb2Card() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

void check_secure_module_staging(const std::vector<u8>& context, bool context_fits,
                                 bool secure_kernel_handoff = true) {
    const auto kprx = read_file(resolve_workspace_path("Vita_104_Firmware/Out/SLB2/kprx_auth_sm.self"));
    const auto rvk = read_file(resolve_workspace_path("Vita_104_Firmware/Out/SLB2/prog_rvk.srvk"));
    const auto kbl = read_file(resolve_workspace_path("Vita_104_Firmware/Out/SLB2/kernel_boot_loader.self"));
    ZLB_EXPECT_TRUE(kprx.has_value() && rvk.has_value() && kbl.has_value());
    if (!kprx || !rvk || !kbl) return;
    ZLB_EXPECT_EQ(kprx->size(), 0x88F8u);

    // Reverse the container order to exercise the firmware's name-based load
    // order. A missing/failed context entry must leave the first slot available.
    std::vector<Slb2Entry> entries = {
        {"prog_rvk.srvk", 0, 0, *rvk},
        {"kernel_boot_loader.self", 0, 0, *kbl},
        {"kprx_auth_sm.self", 0, 0, *kprx},
        {"second_loader.enp", 0, 0, std::vector<u8>(4, 0)},
    };
    if (!context.empty()) entries.push_back({"context_auth_sm.self", 0, 0, context});
    TemporarySlb2Card card(entries);
    Vita vita;
    VitaConfig config;
    config.emmc_image = card.path.string();
    config.rebuild_emmc = false;
    config.provision_keys = false;
    vita.build(config);
    vita.reset(true);

    const u32 kprx_pa = context_fits ? 0x40000900u : 0x40000500u;
    const u32 rvk_pa = context_fits ? 0x40009300u : 0x40008F00u;
    auto expect_payload = [&](u32 address, const std::vector<u8>& expected) {
        std::vector<u8> actual(expected.size());
        vita.arm_bus().read_bytes(address, actual.data(), actual.size());
        ZLB_EXPECT_TRUE(actual == expected);
        vita.cmep_bus().read_bytes(address, actual.data(), actual.size());
        ZLB_EXPECT_TRUE(actual == expected);
    };
    // The real second loader uses this window as IdStorage scratch before its
    // final module reads. Emulate that overwrite after the boot ROM has retained
    // the source files, so the handoff must restore the original bytes.
    for (u32 address = board::kSecureModuleStagingBase; address < board::kSecureModuleStagingEnd;
         address += 4) {
        vita.cmep_bus().write32(address, 0xF5FFF5FFu);
    }
    ZLB_EXPECT_EQ(vita.arm_bus().read32(kprx_pa), 0xF5FFF5FFu);

    if (secure_kernel_handoff) {
        // The secure kernel loads into private SRAM and must leave the final
        // shared-DRAM auth files readable by both processors.
        ZLB_EXPECT_TRUE(vita.enter_stage(BootStage::CmepSecureKernel));
        expect_payload(kprx_pa, *kprx);
        expect_payload(rvk_pa, *rvk);
        if (context_fits) expect_payload(0x40000500u, context);
    }

    auto expect_modules_and_params = [&] {
        for (u32 base : {board::kKblParamBase, board::kKblParamDram}) {
            ZLB_EXPECT_EQ(vita.arm_bus().read32(base + 0x88), context_fits ? 0x40000500u : 0u);
            ZLB_EXPECT_EQ(vita.arm_bus().read32(base + 0x8C), context_fits ? context.size() : 0u);
            ZLB_EXPECT_EQ(vita.arm_bus().read32(base + 0x90), kprx_pa);
            ZLB_EXPECT_EQ(vita.arm_bus().read32(base + 0x94), kprx->size());
            ZLB_EXPECT_EQ(vita.arm_bus().read32(base + 0x98), rvk_pa);
            ZLB_EXPECT_EQ(vita.arm_bus().read32(base + 0x9C), rvk->size());
        }
        expect_payload(kprx_pa, *kprx);
        expect_payload(rvk_pa, *rvk);
        if (context_fits) expect_payload(0x40000500u, context);
    };
    ZLB_EXPECT_TRUE(vita.enter_stage(BootStage::ArmKernelBootLoader));
    expect_modules_and_params();

    // A warm reset clears both RAM buses without rereading SLB2. Direct stage
    // entry must republish the retained source bytes and the matching record.
    vita.reset(false);
    ZLB_EXPECT_EQ(vita.arm_bus().read32(kprx_pa), 0u);
    ZLB_EXPECT_TRUE(vita.enter_stage(BootStage::ArmKernelBootLoader));
    expect_modules_and_params();
}

}  // namespace

ZLB_TEST(machine_memory_map) {
    Vita& vita = shared_machine();

    // CMeP RAM window holds the first loader and the staging buffer.
    ZLB_EXPECT_TRUE(vita.cmep_bus().is_ram(board::kCmepRamBase, board::kCmepRamSize));
    // kCmepStackTop is the *first byte past* the window (the stack grows down
    // from it), so the last usable byte is the mapped one.
    ZLB_EXPECT_TRUE(vita.cmep_bus().is_ram(board::kCmepStackTop - 4));
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

ZLB_TEST(machine_cdram_aperture_is_independent_and_resets) {
    Vita vita;
    VitaConfig config;
    config.rebuild_emmc = false;
    config.provision_keys = false;
    vita.build(config);
    Bus& bus = vita.arm_bus();

    // The complete hardware aperture, including the last byte, must be RAM
    // that DMA/scanout can read directly. A short framebuffer-only allocation
    // or a main-DRAM/SRAM alias would fail these checks.
    ZLB_EXPECT_TRUE(bus.is_ram(0x20000000, 0x08000000));
    ZLB_EXPECT_FALSE(bus.is_ram(0x1FFFFFFF));
    ZLB_EXPECT_FALSE(bus.is_ram(0x28000000));
    bus.write32(0x40000020, 0x55667788);
    bus.write32(0x1C000020, 0x11223344);
    bus.write32(0x20000020, 0xAABBCCDD);
    bus.write32(0x27FFFFFC, 0x12345678);
    bus.write8(0x20000021, 0xEF);
    ZLB_EXPECT_EQ(bus.read32(0x20000020), 0xAABBEFDDu);
    ZLB_EXPECT_EQ(bus.read32(0x27FFFFFC), 0x12345678u);
    ZLB_EXPECT_EQ(bus.read32(0x40000020), 0x55667788u);
    ZLB_EXPECT_EQ(bus.read32(0x1C000020), 0x11223344u);
    const auto* region = bus.region_at(0x20000020, 4);
    const u8* bytes = region ? region->bytes() + (0x20000020 - region->base) : nullptr;
    ZLB_EXPECT_TRUE(bytes != nullptr);
    if (bytes) {
        ZLB_EXPECT_EQ(bytes[0], 0xDDu);
        ZLB_EXPECT_EQ(bytes[1], 0xEFu);
        ZLB_EXPECT_EQ(bytes[2], 0xBBu);
        ZLB_EXPECT_EQ(bytes[3], 0xAAu);
    }

    vita.reset(false);
    ZLB_EXPECT_EQ(bus.read32(0x20000020), 0u);
    ZLB_EXPECT_EQ(bus.read32(0x27FFFFFC), 0u);
    ZLB_EXPECT_TRUE(bus.is_ram(0x20000000, 0x08000000));
}

ZLB_TEST(machine_secure_modules_survive_secure_kernel_handoff) {
    check_secure_module_staging({}, false);
    // A 513-byte optional context rounds to 1024 bytes before the auth SELF.
    check_secure_module_staging(std::vector<u8>(513, 0xC7), true);
}

ZLB_TEST(machine_oversized_secure_module_does_not_consume_a_slot) {
    check_secure_module_staging(
        std::vector<u8>(board::kSecureModuleStagingEnd - board::kSecureModuleStagingBase + 1, 0xC7),
        false);
}

ZLB_TEST(machine_direct_arm_stage_has_secure_module_sources) {
    check_secure_module_staging({}, false, false);
}

ZLB_TEST(machine_secure_kernel_uncached_sram_remap_and_reset) {
    TemporarySlb2Card card({{"second_loader.enp", 0, 0, std::vector<u8>(4, 0)}});
    Vita vita;
    VitaConfig config;
    config.emmc_image = card.path.string();
    config.rebuild_emmc = false;
    config.provision_keys = false;
    vita.build(config);
    vita.reset(true);
    Bus& bus = vita.cmep_bus();
    constexpr u32 low = board::kCmepRamBase;
    constexpr u32 cached = board::kCmepSecureKernelBase;
    constexpr u32 offset = 0x12000u;
    const u32 first_loader_word = bus.fetch32(config.first_loader_base);

    for (bool cold : {false, true}) {
        // Exercise a populated fast-path page before the handoff. Boot storage
        // is independent of the private SRAM; the ARM mirror already sees SRAM.
        bus.write32(low + offset, 0x12345678u);
        bus.write32(cached + offset, 0xA5A55A5Au);
        ZLB_EXPECT_EQ(bus.fetch32(low + offset), 0x12345678u);
        ZLB_EXPECT_EQ(vita.arm_bus().read32(low + offset), 0xA5A55A5Au);
        auto* boot_region = bus.region_at(low, board::kCmepRamSize);
        ZLB_EXPECT_TRUE(boot_region != nullptr);
        if (!boot_region) return;
        u8* boot_storage = boot_region->data.data();

        ZLB_EXPECT_TRUE(vita.enter_stage(BootStage::CmepSecureKernel));
        boot_region = bus.region_at(low, board::kCmepRamSize);
        ZLB_EXPECT_TRUE(boot_region && boot_region->data.data() == boot_storage);
        ZLB_EXPECT_EQ(boot_storage[offset], 0x78u);
        ZLB_EXPECT_EQ(bus.fetch32(low + offset), 0xA5A55A5Au);
        // Byte, halfword, cross-page word, instruction fetch and ARM writes all
        // observe one backing store in both CMeP address windows.
        bus.write8(low + offset, 0x19u);
        ZLB_EXPECT_EQ(bus.read8(cached + offset), 0x19u);
        bus.write16(cached + offset + 2u, 0xABCDu);
        ZLB_EXPECT_EQ(bus.fetch16(low + offset + 2u), 0xABCDu);
        bus.write32(low + 0xFFEu, 0xCAFEBABEu);
        ZLB_EXPECT_EQ(bus.read32(cached + 0xFFEu), 0xCAFEBABEu);
        vita.arm_bus().write32(low + offset + 4u, 0x76543210u);
        ZLB_EXPECT_EQ(bus.fetch32(low + offset + 4u), 0x76543210u);
        ZLB_EXPECT_EQ(bus.fetch32(cached + offset + 4u), 0x76543210u);

        // Execute the actual firmware cache-disable helper at its uncached
        // address, including the real RET. No helper bytes or result are seeded.
        auto* mep = dynamic_cast<MePCore*>(vita.cmep());
        ZLB_EXPECT_TRUE(mep != nullptr);
        if (!mep) return;
        mep->cfg = 0xAA55FFFFu;
        mep->lp = 0x80020Fu;
        mep->r[0] = 0xDEADBEEFu;
        mep->set_pc(low + 0xCEu);
        for (unsigned i = 0; i < 6u; ++i) mep->step();
        ZLB_EXPECT_FALSE(mep->undefined_instruction);
        ZLB_EXPECT_FALSE(mep->halted);
        ZLB_EXPECT_EQ(mep->cfg, 0xAA55FBFDu);
        ZLB_EXPECT_EQ(mep->r[0], 0u);
        ZLB_EXPECT_EQ(mep->get_pc(), 0x80020Eu);

        vita.reset(cold);
        // Clearing the retained owned bytes proves reset detached the alias
        // before Bus::reset(), not merely before the first subsequent access.
        ZLB_EXPECT_EQ(boot_storage[offset], 0u);
        ZLB_EXPECT_EQ(bus.read32(low + offset), 0u);
        ZLB_EXPECT_EQ(bus.read32(cached + offset), 0u);
        bus.write32(cached + offset, 0x11223344u);
        ZLB_EXPECT_EQ(bus.fetch32(low + offset), 0u);
        bus.write32(low + offset, 0x99887766u);
        ZLB_EXPECT_EQ(bus.read32(cached + offset), 0x11223344u);
        ZLB_EXPECT_EQ(vita.arm_bus().read32(low + offset), 0x11223344u);
        if (cold) ZLB_EXPECT_EQ(bus.fetch32(config.first_loader_base), first_loader_word);
    }
}

ZLB_TEST(machine_cmep_startup_waits_for_native_scheduler_handshake) {
    TemporarySlb2Card card({{"second_loader.enp", 0, 0, std::vector<u8>(4, 0)}});
    Vita vita;
    VitaConfig config;
    config.emmc_image = card.path.string();
    config.rebuild_emmc = false;
    config.provision_keys = false;
    vita.build(config);
    vita.reset(true);
    ZLB_EXPECT_TRUE(vita.enter_stage(BootStage::CmepSecureKernel));

    // The board remaps EVM=0 vectors into secure SRAM. A real ARM mailbox
    // doorbell must reach INTC source 8 and fetch the guest's channel-8 vector,
    // whose jump enters the native command dispatcher wrapper.
    auto* mep = dynamic_cast<MePCore*>(vita.cmep());
    ZLB_EXPECT_TRUE(mep != nullptr);
    if (!mep) return;
    vita.cmep_block().set_arm_to_cmep_command(0);
    mep->cfg = 8;
    mep->psw = 0x101;
    mep->cbus.write(0, 0x600);
    mep->cbus.write(2, 0x100);
    mep->cbus.write(5, 0xF);
    vita.arm_bus().write32(0xE0000010, 0x80A01);
    mep->step();
    ZLB_EXPECT_EQ(mep->get_pc(), 0x800050u);
    mep->step();
    ZLB_EXPECT_EQ(mep->get_pc(), 0x80035Cu);
    vita.cmep_bus().write32(0xE0000010, 0xFFFFFFFFu);
    mep->reset(board::kCmepSecureKernelBase);

    // A PC in the second-loader window is not proof of a secure-kernel return.
    // The former broad hook stopped before this harmless instruction and
    // advertised success even after an invalid fetch path escaped the image.
    vita.cmep_block().set_cmep_status(0);
    vita.cmep_bus().write16(0x40002, 0x0000);  // MeP NOP
    vita.cmep()->set_register("pc", 0x40002);
    const u64 before = vita.cmep()->instructions;
    vita.cmep()->step();
    ZLB_EXPECT_EQ(vita.cmep()->instructions, before + 1u);
    ZLB_EXPECT_FALSE(vita.cmep()->halted);
    ZLB_EXPECT_EQ(vita.cmep()->get_pc(), 0x40004u);

    // The boot ROM may acknowledge the residual second-loader status, but must
    // leave the secure kernel's live state and command word to its native peer.
    constexpr u32 gp = 0x0080F698u;
    constexpr u32 state = gp - 32748u;
    vita.cmep()->set_register("pc", 0x800410);
    vita.cmep()->set_register("gp", gp);
    vita.cmep_bus().write32(state, 1u);
    vita.cmep_block().set_arm_to_cmep_command(0);
    vita.cmep_block().set_cmep_status(9);
    vita.poll_boot_chain();
    ZLB_EXPECT_EQ(vita.cmep_block().cmep_status(), 0u);
    ZLB_EXPECT_EQ(vita.cmep_bus().read32(state), 1u);
    ZLB_EXPECT_EQ(vita.cmep_block().arm_to_cmep_command(), 0u);
    ZLB_EXPECT_FALSE(vita.boot_status().arm_released);

    // Real firmware publishes 0x101 and polls for PA | 1. Releasing the ARM must
    // preserve that pending status, its waiting PC, and the initialized state.
    vita.cmep_block().set_cmep_status(0x101);
    vita.poll_boot_chain();
    vita.poll_boot_chain();
    ZLB_EXPECT_TRUE(vita.boot_status().arm_released);
    ZLB_EXPECT_EQ(vita.cmep_block().cmep_status(), 0x101u);
    ZLB_EXPECT_EQ(vita.cmep_block().arm_to_cmep_command(), 0u);
    ZLB_EXPECT_EQ(vita.cmep_bus().read32(state), 1u);
    ZLB_EXPECT_EQ(vita.cmep()->get_pc(), 0x800410u);
    ZLB_EXPECT_FALSE(vita.cmep()->halted);

    // Later status 0x102 is also native scheduler traffic, not an automatic
    // shutdown request or a boot-ROM reply that overwrites the command buffer.
    vita.cmep_block().set_cmep_status(0x102);
    vita.cmep_block().set_arm_to_cmep_command(0x40123401);
    vita.poll_boot_chain();
    ZLB_EXPECT_EQ(vita.cmep_block().cmep_status(), 0x102u);
    ZLB_EXPECT_EQ(vita.cmep_block().arm_to_cmep_command(), 0x40123401u);
    ZLB_EXPECT_EQ(vita.cmep_bus().read32(state), 1u);

    // A reset must discard the previous boot's ready/release state. A stale
    // externally observed 0x101 alone cannot release a fresh first-loader boot.
    vita.reset(false);
    vita.cmep_block().set_cmep_status(0x101);
    vita.poll_boot_chain();
    ZLB_EXPECT_FALSE(vita.boot_status().arm_released);
    ZLB_EXPECT_TRUE(vita.arm()->halted);
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

ZLB_TEST(machine_native_mailbox_irq_does_not_overlap_l2_cache) {
    Vita& vita = shared_machine();
    vita.reset(true);
    Bus& bus = vita.arm_bus();

    // The native firmware uses PERIPHBASE 0x1A000000 and PL310 at +0x2000.
    // Check the assembled board, where the former broad cache window used to
    // intercept the GIC CPU-interface accesses despite a working GIC fixture.
    Device* cpuif = bus.find_device(0x1A000104u);
    Device* cache = bus.find_device(0x1A00277Cu);
    ZLB_EXPECT_TRUE(cpuif && cpuif->name() == "Kermit.GIC");
    ZLB_EXPECT_TRUE(cache && cache->name() == "Kermit.L2CC@1A002000");
    for (u32 command : {0x1A002730u, 0x1A00277Cu, 0x1A0027FCu}) {
        bus.write32(command, 0xFFFFu);
        ZLB_EXPECT_EQ(bus.read32(command), 0u);
    }

    // Program and acknowledge the real CMeP->ARM channel-0 interrupt through
    // the guest's register addresses, including distributor word 6 (IRQ200).
    bus.write32(0x1A001000u, 1u);
    bus.write32(0x1A000100u, 1u);
    bus.write32(0x1A000104u, 0xFFu);
    bus.write32(0x1A001118u, 1u << 8u);
    bus.write8(0x1A0014C8u, 0x20u);
    bus.write8(0x1A0018C8u, 1u);
    vita.cmep_block().set_cmep_status(1u);
    ZLB_EXPECT_EQ(bus.read32(0x1A00010Cu) & 0x3FFu, 200u);
    vita.cmep_block().set_cmep_status(0u);
    bus.write32(0x1A000110u, 200u);
    ZLB_EXPECT_EQ(bus.read32(0x1A00010Cu) & 0x3FFu, 1023u);
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
    vita.reset(true);

    Cpu* arm0 = vita.arm_core(0);
    ZLB_EXPECT_TRUE(arm0 != nullptr);
    if (!arm0) return;

    // The ARM cores stay halted until the CMeP has handed the boot context over
    // (round 92), so enter the kernel-boot-loader stage explicitly: this test is
    // about the debugger's exact breakpoints, not about the boot timing.
    ZLB_EXPECT_TRUE(vita.enter_stage(BootStage::ArmKernelBootLoader));
    ZLB_EXPECT_FALSE(arm0->halted);

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

ZLB_TEST(machine_monitor_keeps_guest_runtime_vectors) {
    Vita& vita = shared_machine();
    const CoreBudget saved_budget = vita.budget();
    vita.budget() = {1, 0, 0};
    for (const auto scenario : {std::pair{true, false}, std::pair{false, false},
                                std::pair{true, true}}) {
        const bool mapped = scenario.first;
        const bool relocated_handler = scenario.second;
        vita.reset(true);
        ZLB_EXPECT_TRUE(vita.enter_stage(BootStage::ArmKernelBootLoader));
        auto* core = dynamic_cast<ArmCore*>(vita.arm_core(0));
        ZLB_EXPECT_TRUE(core != nullptr);
        if (!core) continue;
        for (int i = 1; i < Vita::kArmCoreCount; ++i) vita.arm_core(i)->halted = true;

        // The table installed by genuine FW1.04 SKBL before NSKBL's first
        // SMC103, including its updated SMC dispatcher and adjacent boot data.
        constexpr u32 runtime = 0x40000100u;
        for (u32 i = 0; i < 0x100u; i += 4) vita.arm_bus().write32(runtime + i, 0u);
        constexpr std::array<u32, 8> handlers = {
            0x4002826C, 0x400288C4, 0x40028294, 0x400286C4,
            0x40028460, 0x400282C4, 0x400282DC, 0x40028308,
        };
        for (u32 i = 0; i < handlers.size(); ++i) {
            vita.arm_bus().write32(runtime + i * 4u, 0xE59FF018u);
            vita.arm_bus().write32(runtime + 0x20u + i * 4u, handlers[i]);
        }
        for (u32 offset : {0x48u, 0x4Cu, 0x50u, 0x58u, 0x5Cu}) {
            vita.arm_bus().write32(runtime + offset, 0xE59FF018u);
        }
        const u32 smc_handler = relocated_handler ? 0x003BE1C8u : 0x40029788u;
        vita.arm_bus().write32(runtime + 0x68u, smc_handler);
        for (u32 offset : {0x6Cu, 0x70u, 0x78u}) {
            vita.arm_bus().write32(runtime + offset, 0x4002831Cu);
        }
        vita.arm_bus().write32(runtime + 0x7Cu, 0x400295C8u);
        vita.arm_bus().write32(runtime + 0x90u, 0x80000001u);
        vita.arm_bus().write32(runtime + 0xFCu, 0xA5A55A5Au);
        std::vector<u8> before(0x100u), after(0x100u);
        vita.arm_bus().read_bytes(runtime, before.data(), before.size());

        // Monitor executes Secure even while SCR.NS names the caller's world.
        core->cpsr = arm::kModeMonitor | arm::kFlagA | arm::kFlagI | arm::kFlagF;
        core->thumb = false;
        core->ns_ = true;
        core->scr = 5u;
        core->mmu.sctlr = mapped ? 1u : 0u;
        core->mmu.ttbcr = 0u;
        core->mmu.ttbr0 = 0x40100000u;
        core->mmu.dacr = 3u;
        if (mapped) {
            // VA0x16000 -> PA0x40000000, so the vector at VA16148 reads
            // the runtime monitor table directly at physical40000148.
            vita.arm_bus().write32(0x40100000u, 0x40101001u);
            vita.arm_bus().write32(0x40101000u + 0x16u * 4u, 0x40000032u);
            if (relocated_handler) {
                // The native IntrMgr handler is a VA in a separately allocated
                // page, unlike the original KBL's identity-mapped address.
                vita.arm_bus().write32(0x40100000u + 3u * 4u, 0x40102001u);
                vita.arm_bus().write32(0x40102000u + 0xBEu * 4u, 0x40194032u);
            }
        }
        core->set_pc(0x16148u);
        vita.run_slice();

        // Execute the actual LDR-PC vector, rather than asserting only the
        // copy predicate: the guest-installed dispatcher must be reached.
        ZLB_EXPECT_EQ(static_cast<u32>(core->get_pc()), smc_handler);
        vita.arm_bus().read_bytes(runtime, after.data(), after.size());
        ZLB_EXPECT_TRUE(after == before);
        if (!mapped) {
            vita.arm_bus().read_bytes(0x16100u, after.data(), after.size());
            ZLB_EXPECT_TRUE(after == before);
        }
    }
    vita.budget() = saved_budget;
}

ZLB_TEST(machine_allocator_preserves_native_heap) {
    Vita& vita = shared_machine();
    const CoreBudget saved_budget = vita.budget();
    vita.budget() = {1, 0, 0};
    vita.reset(true);
    ZLB_EXPECT_TRUE(vita.enter_stage(BootStage::ArmKernelBootLoader));
    auto* core = dynamic_cast<ArmCore*>(vita.arm_core(0));
    ZLB_EXPECT_TRUE(core != nullptr);
    if (!core) {
        vita.budget() = saved_budget;
        return;
    }
    for (int i = 1; i < Vita::kArmCoreCount; ++i) vita.arm_core(i)->halted = true;

    // Genuine FW1.04's internal dlmalloc heap lives at VA70008, above the
    // legacy routing threshold. Exercise the machine hook with its translated
    // header spanning two physical pages, while executing the guest's actual
    // allocator prologue. The cookie and initialized bins must survive intact.
    constexpr u32 heap_va = 0x00070F08u;
    constexpr u32 header_bytes = 0x1DCu;
    constexpr u32 cookie = 0x19442EA8u;
    constexpr u32 heap_pa = 0x40130F08u;
    constexpr u32 second_pa = 0x40150000u;
    constexpr u32 l1 = 0x40100000u;
    constexpr u32 l2 = 0x40101000u;
    vita.arm_bus().write32(l1, l2 | 1u);
    vita.arm_bus().write32(l1 + 0x400u * 4u, 0x40000C02u);
    vita.arm_bus().write32(l2 + 0x70u * 4u, 0x40130032u);
    vita.arm_bus().write32(l2 + 0x71u * 4u, second_pa | 0x32u);
    vita.arm_bus().write32(0x400B2974u, cookie);
    std::vector<u8> before(header_bytes), after(header_bytes);
    for (u32 offset = 0; offset < header_bytes; offset += 4u) {
        const u32 value = offset == 0x24u ? cookie : 0x00070000u + offset;
        const u32 pa = offset < 0xF8u ? heap_pa + offset : second_pa + offset - 0xF8u;
        vita.arm_bus().write32(pa, value);
    }
    vita.arm_bus().read_bytes(heap_pa, before.data(), 0xF8u);
    vita.arm_bus().read_bytes(second_pa, before.data() + 0xF8u, header_bytes - 0xF8u);
    core->mmu.sctlr = 1u;
    core->mmu.ttbcr = 0u;
    core->mmu.ttbr0 = l1;
    core->mmu.dacr = 3u;
    core->set_register("CPSR", arm::kModeSystem | arm::kFlagT);
    core->set_register("r0", heap_va);
    core->set_register("r1", 0xCu);
    core->set_register("SP", 0x40061000u);
    core->set_pc(0x40034A00u);
    vita.run_slice();

    ZLB_EXPECT_EQ(core->r[0], heap_va);
    ZLB_EXPECT_EQ(static_cast<u32>(core->get_pc()), 0x40034A04u);
    ZLB_EXPECT_EQ(vita.arm_bus().read32(heap_pa + 0x24u), cookie);
    vita.arm_bus().read_bytes(heap_pa, after.data(), 0xF8u);
    vita.arm_bus().read_bytes(second_pa, after.data() + 0xF8u, header_bytes - 0xF8u);
    ZLB_EXPECT_TRUE(after == before);
    vita.budget() = saved_budget;
}

ZLB_TEST(machine_memblock_pool_preserves_native_table) {
    Vita& vita = shared_machine();
    const CoreBudget saved_budget = vita.budget();
    vita.budget() = {1, 0, 0};
    vita.reset(true);
    ZLB_EXPECT_TRUE(vita.enter_stage(BootStage::ArmKernelBootLoader));
    auto* core = dynamic_cast<ArmCore*>(vita.arm_core(0));
    ZLB_EXPECT_TRUE(core != nullptr);
    if (!core) {
        vita.budget() = saved_budget;
        return;
    }
    for (int i = 1; i < Vita::kArmCoreCount; ++i) vita.arm_core(i)->halted = true;

    constexpr u32 global = 0x400B291Cu;
    constexpr u32 pool = 0x400A0000u;
    constexpr u32 fallback = 0x4008F000u;
    constexpr u32 native_va = 0x00070000u;
    constexpr u32 native_pa = 0x40190000u;
    constexpr u32 l1 = 0x40100000u;
    constexpr u32 l2 = 0x40101000u;
    vita.arm_bus().write32(l1, l2 | 1u);
    vita.arm_bus().write32(l1 + 0x400u * 4u, 0x40000C02u);
    vita.arm_bus().write32(l2 + 0x70u * 4u, native_pa | 0x32u);
    core->mmu.sctlr = 1u;
    core->mmu.ttbcr = 0u;
    core->mmu.ttbr0 = l1;
    core->mmu.dacr = 3u;
    core->set_register("CPSR", arm::kModeSystem | arm::kFlagT);

    // First remember the legacy fallback through its normal early hook. Its
    // ninth record must also be initialized: type00100808 selects index8.
    vita.arm_bus().write32(global, 0u);
    vita.arm_bus().write32(0x400B294Cu, pool);
    vita.arm_bus().write32(pool + 0x18u, 0x40080000u);
    vita.arm_bus().write32(pool + 0x1Cu, 0x10000u);
    for (u32 offset = 0; offset < 9u * 0x14u; offset += 4u) {
        vita.arm_bus().write32(fallback + offset, 0xA5A55A5Au);
    }
    core->set_register("r1", 0u);
    core->set_register("r5", 0x400B2910u);
    core->set_pc(0x4002BB32u);
    vita.run_slice();
    ZLB_EXPECT_EQ(vita.arm_bus().read32(global), fallback);
    ZLB_EXPECT_EQ(static_cast<u32>(core->get_pc()), 0x4002BB34u);
    for (u32 offset = 0; offset < 9u * 0x14u; offset += 4u) {
        ZLB_EXPECT_EQ(vita.arm_bus().read32(fallback + offset), 0u);
    }

    // FW1.04 subsequently installs its own nine-record table and populates
    // class8 before the NameHeap constructor. Execute the genuine LDR at the
    // later hook: it must observe that native VA, retaining its free object.
    std::array<u32, 9u * 5u> native{};
    native[8u * 5u + 1u] = 1u;              // free count
    native[8u * 5u + 2u] = 1u;              // low-water count
    native[8u * 5u + 3u] = native_va + 0xC0u;
    native[8u * 5u + 4u] = native_va + 0xC0u;
    for (u32 i = 0; i < native.size(); ++i) {
        vita.arm_bus().write32(native_pa + i * 4u, native[i]);
    }
    vita.arm_bus().write32(native_pa + 0xF0u, 0x00100808u);
    vita.arm_bus().write32(global, native_va);
    core->set_register("r2", global);
    core->set_pc(0x4002BFB8u);
    vita.run_slice();
    ZLB_EXPECT_EQ(core->r[2], native_va);
    ZLB_EXPECT_EQ(static_cast<u32>(core->get_pc()), 0x4002BFBAu);
    ZLB_EXPECT_EQ(vita.arm_bus().read32(global), native_va);
    for (u32 i = 0; i < native.size(); ++i) {
        ZLB_EXPECT_EQ(vita.arm_bus().read32(native_pa + i * 4u), native[i]);
    }
    ZLB_EXPECT_EQ(vita.arm_bus().read32(native_pa + 0xF0u), 0x00100808u);

    // The remembered fallback remains available if the slot actually becomes
    // zero, without touching the previously populated native table.
    vita.arm_bus().write32(global, 0u);
    core->set_register("r2", global);
    core->set_pc(0x4002BFB8u);
    vita.run_slice();
    ZLB_EXPECT_EQ(core->r[2], fallback);
    ZLB_EXPECT_EQ(vita.arm_bus().read32(global), fallback);
    for (u32 i = 0; i < native.size(); ++i) {
        ZLB_EXPECT_EQ(vita.arm_bus().read32(native_pa + i * 4u), native[i]);
    }
    vita.budget() = saved_budget;
}

// The non-secure kernel boot loader (NSKBL) is the last segment of
// kernel_boot_loader.self: an ARZL stream staged at PA 0x50000000, which SKBL
// decodes to 0x51000000 and enters in the non-secure world.  The stage runs the
// KBL's *own* sceArlzDecode (0x4003C330) and sceArlzArmFilter (0x4003CB40) on
// the emulated core, so this test pins the whole decode: the reset vector, the
// startup string of the decoded image and the state the core is left in.
ZLB_TEST(machine_nskbl_stage_decodes_the_non_secure_loader) {
    Vita& vita = shared_machine();
    vita.reset(true);

    ZLB_EXPECT_TRUE(vita.enter_stage(BootStage::NskblEntry));
    ZLB_EXPECT_TRUE(vita.stage() == BootStage::NskblEntry);

    Bus& arm = vita.arm_bus();
    // The compressed stream is where the KBL stages it, and the decoder read it
    // from there (the "ARZL" magic survives in DRAM).
    ZLB_EXPECT_EQ(arm.read32(0x50000000), 0x4C5A5241u);

    // NSKBL's reset vector: eight `ldr pc, [pc, #0x18]` entries whose pointer
    // table names the reset handler (0x51000100) first.
    ZLB_EXPECT_EQ(arm.read32(0x51000000), 0xE59FF018u);
    ZLB_EXPECT_EQ(arm.read32(0x51000020), 0x51000100u);

    // The startup message of the decoded image.  It sits at 0x51027F66 and is
    // what the loader prints through SceKernelPrintf once its boot() is done.
    std::string text;
    for (u32 i = 0; i < 0x3000; ++i) text.push_back(static_cast<char>(arm.read8(0x51027000 + i)));
    ZLB_EXPECT_TRUE(text.find("Starting PSP2 Kernel Boot Loader") != std::string::npos);
    ZLB_EXPECT_TRUE(text.find("psp2bootconfig.skprx") != std::string::npos);

    // Every core starts at the reset vector, in the non-secure world, with the
    // MMU off (wiki NSKBL#Reset).
    for (int i = 0; i < Vita::kArmCoreCount; ++i) {
        Cpu* core = vita.arm_core(i);
        ZLB_EXPECT_TRUE(core != nullptr);
        if (!core) continue;
        ZLB_EXPECT_EQ(static_cast<u32>(core->get_pc()), 0x51000000u);
        ZLB_EXPECT_FALSE(core->halted);
    }
}

// The machine's clock is Kermit's tick counter, not a core's private `cycles`.
// It used to be read from arm0, so the moment arm0 parked in WFE the reported
// emulated time froze for the rest of the run (and `run_for` could never reach
// its target).  Halt arm0 deliberately and check the clock still moves.
ZLB_TEST(machine_emulated_time_keeps_advancing_while_arm0_is_halted) {
    Vita& vita = shared_machine();
    vita.reset(true);
    Cpu* arm0 = vita.arm_core(0);
    ZLB_EXPECT_TRUE(arm0 != nullptr);
    if (arm0 == nullptr) return;

    vita.run_slice();
    const double before = vita.emulated_seconds();
    const u64 arm0_cycles_before = arm0->cycles;

    arm0->halted = true;
    arm0->halt_reason = "test: parked in WFE";
    for (int i = 0; i < 4; ++i) vita.run_slice();
    const double after = vita.emulated_seconds();

    arm0->halted = false;
    arm0->halt_reason.clear();

    ZLB_EXPECT_TRUE(after > before);
    ZLB_EXPECT_EQ(arm0->cycles, arm0_cycles_before);   // the core itself never ran
    ZLB_EXPECT_TRUE(vita.kermit().total_cycles() >= 5u * 256u);
}

// A normal cold boot never calls the debug-only `stage nskbl` / `stage kernel`
// entry points, so the boot report used to stay on `arm-kernel-boot-loader` for
// the whole run even after NSKBL had loaded os0 and the kernel was executing.
// The state machine now discovers both transitions from the cluster's PCs.
ZLB_TEST(machine_boot_stage_follows_the_normal_chain_into_nskbl_and_kernel) {
    Vita& vita = shared_machine();
    vita.reset(true);
    ZLB_EXPECT_TRUE(vita.stage() == BootStage::ArmBootRom ||
                    vita.stage() == BootStage::CmepFirstLoader);
    auto& arm0 = *dynamic_cast<ArmCore*>(vita.arm_core(0));

    // The debug entries are *not* used: the stage has to come from the PC only.
    arm0.halted = false;
    arm0.reset(0x51016AC2u);              // inside the NSKBL window, non-secure
    vita.poll_boot_chain();
    ZLB_EXPECT_TRUE(vita.stage() == BootStage::NskblEntry);

    arm0.reset(0x0047969Cu);              // os0 kernel below VA 0x00800000
    vita.poll_boot_chain();
    ZLB_EXPECT_TRUE(vita.stage() == BootStage::KernelEntry ||
                    vita.stage() == BootStage::KernelRunning);

    // A power-on reset forgets the observation, so a fresh boot re-discovers it.
    vita.reset(true);
    ZLB_EXPECT_TRUE(vita.stage() == BootStage::ArmBootRom);
}

// The boot-stage fault substitutions repair the low window the missing ARM boot ROM
// would have left in the KBL's tables.  They must never run for a user-mode context:
// a user address space is per process and randomised (user-mode ASLR), so a mapping
// built from the boot rule would alias an unrelated physical page into a random VA
// and swallow the fault the guest's own demand paging wanted.  The gate is installed
// while userland is still out of reach, so this pins it before it can matter.
ZLB_TEST(machine_boot_fault_substitution_refuses_user_mode) {
    Vita& vita = shared_machine();
    vita.reset(true);
    auto& arm = *dynamic_cast<ArmCore*>(vita.arm_core(0));
    ZLB_EXPECT_TRUE(arm.fault_hook != nullptr);
    if (!arm.fault_hook) return;

    Bus& bus = vita.arm_bus();
    // A coarse table whose entries all fault, so the only thing that can satisfy an
    // access is the substitution itself.  VA 0x00200000 is inside the range the boot
    // rule maps (0x00100000..0x40000000, PA = 0x40000000 + VA).
    constexpr u32 l1 = 0x42000000u;
    constexpr u32 l2 = 0x42008000u;
    constexpr u32 kernel_va = 0x00200000u;   // patched by the privileged call
    constexpr u32 user_va = 0x00201000u;     // same 1 MiB section, different page
    bus.memset_bytes(l1, 0, 0x4000u);
    bus.memset_bytes(l2, 0, 0x400u);
    bus.write32(l1 + (kernel_va >> 20) * 4u, l2 | 1u);
    arm.mmu.ttbr0 = l1;
    arm.mmu.ttbcr = 0;
    arm.mmu.dacr = 1;
    arm.mmu.sctlr = 1;

    arm.set_register("CPSR", arm::kModeSystem);
    ZLB_EXPECT_TRUE(arm.fault_hook(0, kernel_va, false, false));   // privileged: repaired

    arm.set_register("CPSR", arm::kModeUser);
    ZLB_EXPECT_FALSE(arm.fault_hook(0, user_va, false, false));    // user: refused
    ZLB_EXPECT_EQ(bus.read32(l2 + ((user_va >> 12) & 0xFFu) * 4u), 0u);
}
