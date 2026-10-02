// Native FW1.04 GPIO0 input/output, W1C and physical IRQ boundaries.
#include <array>
#include <memory>
#include <string>

#include "bus/bus.h"
#include "cpu/arm/arm_core.h"
#include "hw/cmep/cmep_internal.h"
#include "hw/soc/soc_internal.h"
#include "test_framework.h"

using namespace zlb;
using zlb::cmep_detail::GpioDevice;

namespace {
struct GpioFixture {
    std::unique_ptr<Bus> owner = std::make_unique<Bus>();
    Bus& bus = *owner;
    GpioDevice* gpio;
    std::array<bool, 5> levels{};
    std::array<unsigned, 5> rises{};
    GpioFixture() {
        auto device = std::make_unique<GpioDevice>(); gpio = device.get();
        bus.add_device(std::move(device));
        gpio->set_irq_callback([this](u32 id, bool level) {
            ZLB_EXPECT_TRUE(id >= 248 && id <= 252);
            const unsigned gate = id - 248;
            if (level && !levels[gate]) ++rises[gate];
            levels[gate] = level;
        });
    }
    u32 read(u32 offset) { return bus.read32(cmep::kGpioBase + offset); }
    void write(u32 offset, u32 value) { bus.write32(cmep::kGpioBase + offset, value); }
    void native_pin4() {
        write(0, 0xFF000008); // checkpoint/output3 directions, pin4 input
        write(0x14, 0x300);  // actual native Syscon mode3/falling pin4
        write(0x1C, 0);      // actual native gate0 unmask; other masks unknown in old shim
    }
    void fall() { gpio->set_external_input(0x10, 0x10); gpio->set_external_input(0x10, 0); }
};

void write_hex(Bus& bus, u32 address, const char* hex) {
    auto digit = [](char c) -> u8 { return static_cast<u8>(c <= '9' ? c - '0' : c - 'a' + 10); };
    for (unsigned i = 0; hex[i] && hex[i + 1]; i += 2)
        bus.write8(address + i / 2, static_cast<u8>((digit(hex[i]) << 4) | digit(hex[i + 1])));
}
}

ZLB_TEST(gpio_native_checkpoint_and_lowio_port_read_keep_direction_output_and_input_separate) {
    GpioFixture f;
    f.write(0, 0x00FF0008);
    f.write(0x10, 0x00FF0000); // opaque boot word, independent of direction
    f.write(0xC, 0x00FF0000); f.write(8, 0x00A90000);
    ZLB_EXPECT_EQ(f.read(0), 0x00FF0008u);
    ZLB_EXPECT_EQ(f.read(0x34), 0x00A90000u);
    f.write(8, 8); f.write(0xC, 8);
    ZLB_EXPECT_EQ(f.read(0), 0x00FF0008u);
    ZLB_EXPECT_EQ(f.read(0x34), 0x00A90000u);
    f.write(0xC, 0x00FF0000); f.write(8, 0x00A40000);
    ZLB_EXPECT_EQ((f.read(0x34) >> 16) & 0xFFu, 0xA4u); // actual checkpoint latch/gpo source
    ZLB_EXPECT_EQ((f.read(0) >> 16) & 0xFFu, 0xFFu);
    f.bus.add_ram("NativeLowioPortRead", 0x10000, 0x81000000, "unchanged supplied Lowio81002780");
    // Supplied lowio.elf PT_LOAD fileA0, linked81002780..27BA. Actual native
    // hardware1 selects output+34, hardware0 selects nondestructive input+04.
    write_hex(f.bus, 0x81002780,
        "012810b484bf4ff48070c8f23f0013d84c224bf2201302fb00f0c8f20013012414fa01f21c18185863681a4214bf406b4068c84000f0010010bc7047");
    f.bus.write32(0x8100B120, cmep::kGpioBase); f.bus.write32(0x8100B124, f.read(0));
    ArmCore cpu(f.bus);
    auto native_read = [&](unsigned pin) {
        cpu.reset(0x81002781); cpu.r[0] = 0; cpu.r[1] = pin;
        cpu.r[13] = 0x8100FFF0; cpu.r[14] = 0x81002801;
        unsigned steps = 0;
        while (cpu.get_pc() != 0x81002800 && steps++ < 32) ZLB_EXPECT_FALSE(cpu.step().faulted);
        ZLB_EXPECT_EQ(cpu.get_pc(), 0x81002800u);
        return cpu.r[0];
    };
    ZLB_EXPECT_EQ(native_read(18), 1u); // actual output A4 bit2
    ZLB_EXPECT_EQ(native_read(16), 0u); // actual output, direction bit is still1
    f.gpio->set_external_input(0x10, 0x10);
    ZLB_EXPECT_EQ(native_read(4), 1u);
    ZLB_EXPECT_EQ(native_read(4), 1u); // input reads cannot consume peer state
    f.gpio->set_external_input(0x10, 0);
    ZLB_EXPECT_EQ(native_read(4), 0u);
}

ZLB_TEST(gpio_byte_lanes_set_clear_and_readback_do_not_merge_direction_or_reapply_old_masks) {
    GpioFixture f;
    f.write(0, 0xFF000008);
    f.bus.write8(cmep::kGpioBase + 1, 0xAA);
    ZLB_EXPECT_EQ(f.read(0), 0xFF00AA08u);
    f.bus.write8(GpioDevice::kSet + 2, 0xC3);
    f.bus.write8(GpioDevice::kSet, 8);
    ZLB_EXPECT_EQ(f.read(0x34), 0x00C30008u);
    f.bus.write16(GpioDevice::kClear + 2, 0x00C1);
    ZLB_EXPECT_EQ(f.read(0x34), 0x00020008u);
    ZLB_EXPECT_EQ(f.bus.read8(GpioDevice::kOutput + 2), 2u);
    ZLB_EXPECT_EQ(f.bus.read16(GpioDevice::kOutput), 8u);
    ZLB_EXPECT_EQ(f.read(0), 0xFF00AA08u);
    f.gpio->set_external_input(0xFFFFFFFF, 0x12345678);
    const u32 expected = (0x00020008u & 0xFF00AA08u) | (0x12345678u & ~0xFF00AA08u);
    ZLB_EXPECT_EQ(f.read(4), expected);
    f.write(4, 0); f.write(0x34, 0xFFFFFFFF);
    ZLB_EXPECT_EQ(f.read(4), expected); ZLB_EXPECT_EQ(f.read(0x34), 0x00020008u);
    u64 input = 0, output = 0;
    ZLB_EXPECT_TRUE(f.gpio->peek_register("INPUT", input));
    ZLB_EXPECT_TRUE(f.gpio->peek_register("OUTPUT", output));
    ZLB_EXPECT_EQ(input, expected); ZLB_EXPECT_EQ(output, 0x00020008u);
    ZLB_EXPECT_TRUE(f.gpio->poke_register("INPUT", 0));
    ZLB_EXPECT_TRUE(f.gpio->poke_register("OUTPUT", 0));
    ZLB_EXPECT_EQ(f.read(4), expected); ZLB_EXPECT_EQ(f.read(0x34), 0x00020008u);
}

ZLB_TEST(gpio_native_pin4_falling_edge_delivers_only_unmasked_gate_and_w1c_is_lane_exact) {
    GpioFixture f; f.native_pin4(); f.fall();
    ZLB_EXPECT_TRUE(f.levels[0]); ZLB_EXPECT_EQ(f.rises[0], 1u);
    for (unsigned gate = 1; gate < 5; ++gate) ZLB_EXPECT_FALSE(f.levels[gate]);
    for (unsigned gate = 0; gate < 5; ++gate) ZLB_EXPECT_EQ(f.read(0x38 + gate * 4), 0x10u);
    // All-gate masked retention/fanout is a documented model choice, not a
    // measured reset value. W1C never consumes/changes the physical input.
    f.write(0x38, 0); f.bus.write8(cmep::kGpioBase + 0x39, 0xFF);
    f.bus.write16(cmep::kGpioBase + 0x3A, 0xFFFF); f.write(0x38, 8);
    ZLB_EXPECT_TRUE(f.levels[0]); ZLB_EXPECT_EQ(f.read(0x38), 0x10u);
    for (unsigned i = 0; i < 16; ++i) {
        ZLB_EXPECT_EQ(f.bus.read8(GpioDevice::kState) & 0x10u, 0u);
        ZLB_EXPECT_EQ(f.bus.read16(GpioDevice::kState) & 0x10u, 0u);
        u64 pending = 0; ZLB_EXPECT_TRUE(f.gpio->peek_register("STATUS0", pending));
        ZLB_EXPECT_EQ(pending, 0x10u);
    }
    f.bus.write8(cmep::kGpioBase + 0x38, 0x10);
    ZLB_EXPECT_FALSE(f.levels[0]); ZLB_EXPECT_EQ(f.read(4) & 0x10u, 0u);
    f.gpio->set_external_input(0x10, 0);
    ZLB_EXPECT_EQ(f.read(0x38), 0u); ZLB_EXPECT_EQ(f.rises[0], 1u); // no duplicate low-level edge
    f.fall(); ZLB_EXPECT_TRUE(f.levels[0]); ZLB_EXPECT_EQ(f.rises[0], 2u);
    f.write(0x1C, 0xFFFFFFFF);
    ZLB_EXPECT_FALSE(f.levels[0]); ZLB_EXPECT_EQ(f.read(0x38), 0x10u);
    f.write(0x1C, 0); ZLB_EXPECT_TRUE(f.levels[0]);
    f.write(0x24, 0xFFFFFFEF); // previously masked pending gate2 explicitly unmasked
    ZLB_EXPECT_TRUE(f.levels[2]); ZLB_EXPECT_FALSE(f.levels[1]);
    ZLB_EXPECT_EQ(f.rises[2], 1u);
    f.write(0x40, 0x10); ZLB_EXPECT_FALSE(f.levels[2]);
}

ZLB_TEST(gpio_masked_edge_retention_reset_and_unknown_modes_do_not_fabricate_events) {
    GpioFixture f;
    f.write(0x14, 0x300); f.fall(); // physical edge while every gate is masked
    for (bool level : f.levels) ZLB_EXPECT_FALSE(level);
    f.write(0x1C, 0xFFFFFFEF); ZLB_EXPECT_TRUE(f.levels[0]);
    f.bus.reset_devices();
    for (unsigned gate = 0; gate < 5; ++gate) {
        ZLB_EXPECT_FALSE(f.levels[gate]);
        ZLB_EXPECT_EQ(f.read(0x1C + gate * 4), 0xFFFFFFFFu);
        ZLB_EXPECT_EQ(f.read(0x38 + gate * 4), 0u);
    }
    ZLB_EXPECT_EQ(f.read(0), 0u); ZLB_EXPECT_EQ(f.read(4), 0u); ZLB_EXPECT_EQ(f.read(0x34), 0u);
    f.write(0x1C, 0);
    for (u32 mode : {0u, 0x100u, 0x200u}) {
        f.write(0x14, mode); f.fall();
        ZLB_EXPECT_EQ(f.read(0x38), 0u); ZLB_EXPECT_FALSE(f.levels[0]);
    }
    f.write(0x14, 0x300); f.write(0, 0x10); f.fall(); //pin4 output: external fall not accepted
    ZLB_EXPECT_EQ(f.read(0x38), 0u);
    f.write(0, 0); f.gpio->set_external_input(8, 8); f.gpio->set_external_input(8, 0);
    ZLB_EXPECT_EQ(f.read(0x38), 0u); //other pins remain unmodeled
    f.write(0x38, 0xFFFFFFFF); ZLB_EXPECT_EQ(f.read(0x38), 0u);
    f.write(0x100, 0x12345678); f.write(0x30, 0x10);
    ZLB_EXPECT_EQ(f.read(0x100), 0xFFFFFFFFu); ZLB_EXPECT_EQ(f.read(0x30), 0xFFFFFFFFu);
    f.bus.reset_devices();
    ZLB_EXPECT_EQ(f.read(0x100), 0xFFFFFFFFu); //no stray/lane map entry survives reset
    u64 unsupported = 1; ZLB_EXPECT_TRUE(f.gpio->peek_register("UNSUPPORTED_EDGES", unsupported));
    ZLB_EXPECT_EQ(unsupported, 0u);
}

ZLB_TEST(gpio_legacy_jig_is_opt_in_transaction_driven_and_disabled_in_native_phase) {
    GpioFixture f;
    ZLB_EXPECT_EQ(f.read(4), 0u); //ordinary first loader takes genuine no-peer branch
    ZLB_EXPECT_TRUE(f.gpio->set_legacy_jig_enabled(true));
    f.write(0, 8); //genuine mailbox_debug_sc direction write at5E4FA
    for (unsigned i = 0; i < 12; ++i) {
        ZLB_EXPECT_EQ(f.read(4) & 0x10u, 0x10u);
        ZLB_EXPECT_EQ(f.bus.read8(GpioDevice::kState), 0x10u);
        u64 input = 0; ZLB_EXPECT_TRUE(f.gpio->peek_register("INPUT", input));
        ZLB_EXPECT_EQ(input, 0x10u);
    }
    ZLB_EXPECT_EQ(f.gpio->handshakes(), 0u);
    f.write(8, 8); //actual output3 request rise5E558 releases explicit development peer
    ZLB_EXPECT_EQ(f.read(4) & 0x10u, 0u); ZLB_EXPECT_EQ(f.gpio->handshakes(), 1u);
    ZLB_EXPECT_EQ(f.read(0), 8u); ZLB_EXPECT_EQ(f.read(0x34), 8u);
    f.write(8, 8); ZLB_EXPECT_EQ(f.gpio->handshakes(), 1u);
    f.write(0xC, 8); f.gpio->enter_native_phase();
    ZLB_EXPECT_FALSE(f.gpio->set_legacy_jig_enabled(true));
    f.gpio->set_external_input(0x10, 0x10);
    for (unsigned i = 0; i < 12; ++i) ZLB_EXPECT_EQ(f.read(4) & 0x10u, 0x10u);
    ZLB_EXPECT_EQ(f.gpio->handshakes(), 1u); //native reads cannot consume Ernie input
    f.gpio->reset(); ZLB_EXPECT_EQ(f.gpio->handshakes(), 0u);
    ZLB_EXPECT_EQ(f.read(4), 0u); //reset restores absent peer, no asserted ready line
}

ZLB_TEST(gpio_shared_bus_endpoints_deliver_physical_gpio248_through_native_gic_fields) {
    GpioFixture f;
    auto arm_owner = std::make_unique<Bus>(); Bus& arm_bus = *arm_owner;
    KermitBlock soc(arm_bus, nullptr); soc.install(); soc.reset();
    arm_bus.add_device(std::make_unique<DeviceMirror>(*f.gpio, cmep::kGpioBase, f.gpio->size()));
    f.gpio->set_irq_callback([&soc](u32 id, bool level) { soc.raise_irq(id, level); });
    ArmCore cpu(arm_bus); cpu.core_id_ = 3; cpu.reset(); cpu.set_register("CPSR", 0x13);
    kermit_set_cpu(arm_bus, &cpu, 3);
    const u32 dist = kermit::kScuBase + kermit::kGicDistOffset;
    const u32 icc = kermit::kScuBase + kermit::kIccOffset;
    arm_bus.context.core_id = 3; arm_bus.context.nonsecure = false;
    arm_bus.write32(dist, 1); arm_bus.write32(icc + 4, 0xFF); arm_bus.write32(icc, 0xB);
    // Actual579 snapshot: group0, enabled248, priority50, target0F,
    // level-triggered55555555 and native CPU interface0B/FIQ enabled.
    arm_bus.write32(dist + 0x100 + (248 / 32) * 4, 1u << (248 % 32));
    arm_bus.write8(dist + 0x400 + 248, 0x50); arm_bus.write8(dist + 0x800 + 248, 0xF);
    arm_bus.write32(cmep::kGpioBase, 8); arm_bus.write32(cmep::kGpioBase + 0x14, 0x300);
    arm_bus.write32(cmep::kGpioBase + 0x1C, 0);
    arm_bus.write32(GpioDevice::kSet, 0x00A90008);
    ZLB_EXPECT_EQ(f.read(0x34), 0x00A90008u); ZLB_EXPECT_EQ(f.read(0), 8u);
    ZLB_EXPECT_FALSE(cpu.interrupt_pending());
    f.gpio->set_external_input(0x10, 0x10);
    ZLB_EXPECT_EQ(arm_bus.read32(GpioDevice::kState) & 0x10u, 0x10u);
    f.gpio->set_external_input(0x10, 0);
    ZLB_EXPECT_TRUE(cpu.interrupt_pending());
    ZLB_EXPECT_EQ(arm_bus.read32(icc + 0xC) & 0x3FFu, 248u);
    ZLB_EXPECT_EQ(arm_bus.read32(cmep::kGpioBase + 0x38), 0x10u);
    arm_bus.write8(cmep::kGpioBase + 0x38, 0x10);
    arm_bus.write32(icc + 0x10, 248);
    ZLB_EXPECT_FALSE(cpu.interrupt_pending());
    ZLB_EXPECT_EQ(f.read(0x38), 0u); ZLB_EXPECT_EQ(f.read(4) & 0x10u, 0u);
    f.bus.reset_devices();
    ZLB_EXPECT_EQ(arm_bus.read32(GpioDevice::kOutput), 0u);
    ZLB_EXPECT_EQ(arm_bus.read32(cmep::kGpioBase + 0x1C), 0xFFFFFFFFu);
    f.gpio->set_irq_callback({}); kermit_set_cpu(arm_bus, nullptr, 3);
}
