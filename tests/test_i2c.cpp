// Native Lowio reset protocol and explicitly unsupported I2C transfers.
#include <memory>
#include <string>

#include "bus/bus.h"
#include "cpu/arm/arm_core.h"
#include "hw/soc/soc_internal.h"
#include "test_framework.h"

using namespace zlb;

namespace {

struct I2cFixture {
    std::unique_ptr<Bus> owner = std::make_unique<Bus>();
    Bus& bus = *owner;
    KermitBlock soc{bus, nullptr};

    I2cFixture() { soc.install(); soc.reset(); }
    u32 base(unsigned port) const { return port == 0 ? kermit::kI2c0Base : kermit::kI2c1Base; }
    u32 read(unsigned port, u32 offset) { return bus.read32(base(port) + offset); }
    void write(unsigned port, u32 offset, u32 value) { bus.write32(base(port) + offset, value); }
    Device& device(unsigned port) { return *bus.find_device(base(port)); }

    void native_init(unsigned port) {
        // Lowio 0x810031CA..321A, also used by the native reset/resume exports.
        write(port, 0x2C, 0x0100F70F);
        write(port, 0x08, 1);
        write(port, 0x0C, 1);
        write(port, 0x14, 7);
        ZLB_EXPECT_EQ(read(port, 0x1C), 0u);
        write(port, 0x28, read(port, 0x28));
        write(port, 0x2C, 0x01000000);
        write(port, 0x18, 5);
    }
};

}  // namespace

ZLB_TEST(i2c_native_lowio_reset_instructions_reach_idle_on_both_ports) {
    I2cFixture fx;
    constexpr u32 text = 0x80000000;
    constexpr u32 state = 0x80000400;
    fx.bus.add_ram("I2cReplay", 0x2000, text, "native Lowio replay");
    // Exact original Lowio bytes at linked 0x810031C4..31FA. This includes
    // the native MMIO stores, DSB, busy loop, status echo and running control.
    // Stop before the imported ClearEventFlag call, which is not a device op.
    constexpr u8 code[] = {
        0x58, 0xF8, 0x18, 0x3C, 0x07, 0x22, 0xDF, 0x62,
        0xC3, 0xF8, 0x08, 0x90, 0xC3, 0xF8, 0x0C, 0x90,
        0x5A, 0x61, 0xBF, 0xF3, 0x4F, 0x8F, 0xDC, 0x69,
        0x00, 0x2C, 0xFC, 0xD1, 0x9A, 0x6A, 0x4F, 0xF0,
        0x80, 0x7A, 0x4F, 0xF4, 0x9E, 0x61, 0xD8, 0xF8,
        0x00, 0x00, 0xCF, 0xF6, 0xFF, 0x71, 0x01, 0x36,
        0x9A, 0x62, 0xC3, 0xF8, 0x2C, 0xA0,
    };
    fx.bus.write_bytes(text, code, sizeof(code));
    ArmCore cpu(fx.bus);
    for (unsigned port = 0; port < 2; ++port) {
        // Reset must recover a busy unsupported transfer, not only an
        // initially zero status register that happens to pass the loop.
        fx.write(port, 0x14, 0x213);
        ZLB_EXPECT_EQ(fx.read(port, 0x1C), 1u);
        fx.bus.write32(state, fx.base(port));
        fx.bus.write32(state + 0x18, 0x10001);  // event UID is never called here
        cpu.reset(text | 1u);
        cpu.r[6] = port;
        cpu.r[7] = 0x0100F70F;
        cpu.r[8] = state + 0x18;
        cpu.r[9] = 1;
        unsigned steps = 0;
        while (cpu.get_pc() < text + sizeof(code) && steps++ < 64) {
            ZLB_EXPECT_FALSE(cpu.step().faulted);
            if (cpu.undefined_instruction || cpu.halted) break;
        }
        ZLB_EXPECT_EQ(cpu.get_pc(), text + sizeof(code));
        ZLB_EXPECT_EQ(cpu.r[4], 0u); // native CMP/BNE observed actual idle
        ZLB_EXPECT_EQ(cpu.r[6], port + 1);
        ZLB_EXPECT_EQ(fx.read(port, 0x08), 1u);
        ZLB_EXPECT_EQ(fx.read(port, 0x0C), 1u);
        ZLB_EXPECT_EQ(fx.read(port, 0x14), 7u);
        ZLB_EXPECT_EQ(fx.read(port, 0x1C), 0u);
        ZLB_EXPECT_EQ(fx.read(port, 0x28), 0u);
        ZLB_EXPECT_EQ(fx.read(port, 0x2C), 0x01000000u);
        fx.write(port, 0x18, 5); // native store after ClearEventFlag
    }
    ZLB_EXPECT_EQ(fx.soc.pending_irq_count(), 0u);
}

ZLB_TEST(i2c_native_windows_and_register_diagnostics_are_installed) {
    I2cFixture fx;
    for (unsigned port = 0; port < 2; ++port) {
        Device& dev = fx.device(port);
        ZLB_EXPECT_EQ(dev.base(), fx.base(port));
        ZLB_EXPECT_EQ(dev.size(), 0x1000u);
        ZLB_EXPECT_EQ(fx.bus.find_device(fx.base(port) + 0xFFF), &dev);
        ZLB_EXPECT_TRUE(fx.bus.find_device(fx.base(port) + 0x1000) == nullptr);
        bool listed = false;
        for (Device* owned : fx.soc.devices()) if (owned == &dev) listed = true;
        ZLB_EXPECT_TRUE(listed);
        ZLB_EXPECT_TRUE(std::string(dev.register_name(fx.base(port) + 0x1D)) == "BUSY");
        ZLB_EXPECT_TRUE(dev.register_name(fx.base(port) + 0x20) == nullptr);
        ZLB_EXPECT_EQ(fx.read(port, 0x20), 0xFFFFFFFFu); // unrecovered reserved word
        fx.native_init(port);
        u64 value = 0;
        ZLB_EXPECT_TRUE(dev.peek_register("IRQ_CONTROL", value));
        ZLB_EXPECT_EQ(value, 0x01000000u);
        ZLB_EXPECT_FALSE(dev.peek_register("UNKNOWN", value));
        std::vector<std::string> details;
        dev.describe(details);
        ZLB_EXPECT_EQ(details.size(), 3u);
        ZLB_EXPECT_TRUE(details[1].find("unsupported") != std::string::npos);
    }
}

ZLB_TEST(i2c_byte_lanes_preserve_opaque_control_and_command_length) {
    I2cFixture fx;
    const u32 base = fx.base(0);
    fx.bus.write8(base + 0x2C, 0x0F);
    fx.bus.write8(base + 0x2D, 0xF7);
    fx.bus.write16(base + 0x2E, 0x0100);
    ZLB_EXPECT_EQ(fx.read(0, 0x2C), 0x0100F70Fu);
    ZLB_EXPECT_EQ(fx.bus.read8(base + 0x2D), 0xF7u);
    ZLB_EXPECT_EQ(fx.bus.read16(base + 0x2E), 0x0100u);
    fx.bus.write8(base + 0x2D, 0);
    ZLB_EXPECT_EQ(fx.read(0, 0x2C), 0x0100000Fu);
    fx.bus.write16(base + 0x10, 0x69);
    fx.bus.write16(base + 0x12, 0xA55A);
    ZLB_EXPECT_EQ(fx.read(0, 0x10), 0xA55A0069u);
    fx.bus.write8(base + 0x15, 3); // length-only write never starts a command
    ZLB_EXPECT_EQ(fx.read(0, 0x14), 0x300u);
    ZLB_EXPECT_EQ(fx.read(0, 0x1C), 0u);
    fx.bus.write8(base + 0x14, 0x13);
    ZLB_EXPECT_EQ(fx.read(0, 0x14), 0x313u);
    ZLB_EXPECT_EQ(fx.read(0, 0x1C), 1u);
    fx.bus.write16(base + 0x16, 0xCAFE); // upper lanes cannot complete it
    ZLB_EXPECT_EQ(fx.read(0, 0x14), 0xCAFE0313u);
    ZLB_EXPECT_EQ(fx.read(0, 0x1C), 1u);
    fx.bus.write8(base + 0x14, 7); // high unknown flags must not fake reset
    ZLB_EXPECT_EQ(fx.read(0, 0x14), 0xCAFE0307u);
    ZLB_EXPECT_EQ(fx.read(0, 0x1C), 1u);
    fx.bus.write16(base + 0x16, 0);
    fx.bus.write8(base + 0x15, 0);
    ZLB_EXPECT_EQ(fx.read(0, 0x14), 7u);
    ZLB_EXPECT_EQ(fx.read(0, 0x1C), 1u); // upper-lane edits do not execute
    fx.bus.write8(base + 0x14, 7); // exact reset assembled with byte lanes
    ZLB_EXPECT_EQ(fx.read(0, 0x1C), 0u);
    ZLB_EXPECT_EQ(fx.read(0, 0x10), 0xA55A0069u);
    ZLB_EXPECT_EQ(fx.read(0, 0x2C), 0x0100000Fu);
}

ZLB_TEST(i2c_unsupported_transfers_never_complete_or_supply_slave_data) {
    I2cFixture fx;
    for (unsigned port = 0; port < 2; ++port) {
        for (u32 command : {0x0402u, 0x0313u, 5u, 4u, 0u, 0xFFu, 0x107u, 0x80000007u}) {
            fx.native_init(port);
            fx.write(port, 0x10, 0x69);
            fx.write(port, 0x00, 0x81);
            fx.write(port, 0x00, 0x08);
            fx.write(port, 0x14, command);
            ZLB_EXPECT_EQ(fx.read(port, 0x1C), 1u);
            fx.write(port, 0x2C, 0xFFFFFFFF); // opaque mask never generates IRQ
            fx.soc.tick(333000000);
            for (unsigned read = 0; read < 32; ++read)
                ZLB_EXPECT_EQ(fx.read(port, 0x1C), 1u);
            ZLB_EXPECT_EQ(fx.read(port, 0x28), 0u);
            ZLB_EXPECT_EQ(fx.read(port, 0x04), 0xFFFFFFFFu);
            ZLB_EXPECT_EQ(fx.bus.read8(fx.base(port) + 4), 0xFFu);
            ZLB_EXPECT_EQ(fx.bus.read16(fx.base(port) + 4), 0xFFFFu);
            ZLB_EXPECT_EQ(fx.soc.pending_irq_count(), 0u);
            ZLB_EXPECT_TRUE(fx.device(port).summary().find("unsupported transfer") != std::string::npos);
            fx.write(port, 0x1C, 0); // busy is hardware-owned
            fx.write(port, 0x04, 0); // writing RX cannot invent a response
            ZLB_EXPECT_FALSE(fx.device(port).poke_register("BUSY", 0));
            ZLB_EXPECT_FALSE(fx.device(port).poke_register("RX_FIFO", 0));
            ZLB_EXPECT_EQ(fx.read(port, 0x1C), 1u);
            ZLB_EXPECT_EQ(fx.read(port, 0x04), 0xFFFFFFFFu);
            fx.write(port, 0x14, 7);
            ZLB_EXPECT_EQ(fx.read(port, 0x1C), 0u);
            ZLB_EXPECT_TRUE(fx.device(port).summary().find("idle/reset only") != std::string::npos);
        }
    }
}

ZLB_TEST(i2c_two_ports_and_bus_reset_are_independent) {
    I2cFixture fx;
    fx.native_init(0);
    fx.native_init(1);
    fx.write(0, 0x10, 0x1A);
    fx.write(1, 0x10, 0x69);
    fx.write(0, 0x00, 0xE2);
    fx.write(0, 0x14, 0x102);
    ZLB_EXPECT_EQ(fx.read(1, 0x1C), 0u);
    fx.write(1, 0x14, 0x413);
    fx.write(0, 0x14, 7);
    ZLB_EXPECT_EQ(fx.read(0, 0x1C), 0u);
    ZLB_EXPECT_EQ(fx.read(1, 0x1C), 1u);
    ZLB_EXPECT_EQ(fx.read(0, 0x10), 0x1Au);
    ZLB_EXPECT_EQ(fx.read(1, 0x10), 0x69u);
    ZLB_EXPECT_EQ(fx.read(1, 0x14), 0x413u);
    ZLB_EXPECT_TRUE(fx.device(0).summary().find("tx-writes=0") != std::string::npos);
    fx.write(1, 0x14, 7);
    ZLB_EXPECT_EQ(fx.read(1, 0x1C), 0u);
}

ZLB_TEST(i2c_status_writes_do_not_set_events_and_power_reset_restores_idle) {
    I2cFixture fx;
    fx.native_init(0);
    fx.native_init(1);
    // IRQ status is W1C, not writable event injection. There are no modeled
    // transactions that can supply pending bits in this bounded subset.
    fx.write(0, 0x28, 0xFFFFFFFF);
    fx.bus.write8(fx.base(0) + 0x29, 0xFF);
    fx.bus.write16(fx.base(0) + 0x2A, 0xFFFF);
    ZLB_EXPECT_EQ(fx.read(0, 0x28), 0u);
    ZLB_EXPECT_EQ(fx.read(0, 0x2C), 0x01000000u);
    fx.write(0, 0x14, 0x113);
    fx.write(1, 0x00, 0xA5);
    fx.write(1, 0x14, 0x102);
    fx.soc.reset();
    for (unsigned port = 0; port < 2; ++port) {
        for (u32 offset : {8u, 0xCu, 0x10u, 0x14u, 0x18u, 0x1Cu, 0x28u, 0x2Cu})
            ZLB_EXPECT_EQ(fx.read(port, offset), 0u);
        ZLB_EXPECT_EQ(fx.read(port, 0x04), 0xFFFFFFFFu);
        ZLB_EXPECT_TRUE(fx.device(port).summary().find("tx-writes=0 resets=0") != std::string::npos);
        fx.native_init(port); // reset keeps the ordinary guest bring-up usable
    }
    ZLB_EXPECT_EQ(fx.soc.pending_irq_count(), 0u);
}
