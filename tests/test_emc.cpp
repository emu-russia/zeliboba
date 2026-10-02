// Genuine SceDriverTzs command and mode handshakes, with unsupported boundaries.
#include <memory>
#include <string>

#include "bus/bus.h"
#include "cpu/arm/arm_core.h"
#include "hw/soc/soc_internal.h"
#include "test_framework.h"

using namespace zlb;

namespace {

struct EmcFixture {
    std::unique_ptr<Bus> owner = std::make_unique<Bus>();
    Bus& bus = *owner;
    KermitBlock soc{bus, nullptr};

    EmcFixture() { soc.install(); soc.reset(); }
    u32 read(u32 offset) { return bus.read32(kermit::kEmcTopBase + offset); }
    void write(u32 offset, u32 value) { bus.write32(kermit::kEmcTopBase + offset, value); }
    Device& device() { return *bus.find_device(kermit::kEmcTopBase); }
    void tick() { soc.tick(kermit::cycles_from_periph_ticks(1)); }

    void native_config() {
        // Captured SceDriverTzs 0x54BABE..54BAEC; meanings remain opaque.
        write(0x00, 0x2051);
        write(0x04, 0x00072233);
        write(0x08, 0x07725245);
        write(0x0C, 0x00001414);
        write(0x10, 0x001F0704);
        write(0x14, 0x001F0C0F);
        write(0x18, 0x0000020B);
        write(0x1C, 6);
        write(0x38, 0x8C);
        write(0x3C, 0x69462300);
    }

    void command(u32 payload, u32 launch) {
        write(0x28, payload);
        const u32 preserved = read(0x24) & 0x30;
        write(0x24, preserved | launch);
    }
};

}  // namespace

ZLB_TEST(emc_native_first_command_instructions_poll_busy_until_peripheral_tick) {
    EmcFixture fx;
    constexpr u32 text = 0x80000000;
    fx.bus.add_ram("EmcReplay", 0x1000, text, "original SceDriverTzs command loop");
    // Exact linked 0x81001B76..1B94; this constructs the command, preserves30,
    // launches, executes DMB and loops on bit0, rather than requiring word0.
    constexpr u8 code[] = {
        0x4F, 0xF4, 0x60, 0x23, 0x93, 0x62, 0x50, 0x6A,
        0x00, 0xF0, 0x30, 0x01, 0x41, 0xF0, 0x01, 0x03,
        0x53, 0x62, 0xBF, 0xF3, 0x5F, 0x8F, 0x51, 0x6A,
        0x11, 0xF0, 0x01, 0x0F, 0xFB, 0xD1,
    };
    fx.bus.write_bytes(text, code, sizeof(code));
    ArmCore cpu(fx.bus);
    cpu.reset(text | 1u);
    cpu.r[2] = kermit::kEmcTopBase;
    for (unsigned step = 0; step < 7; ++step) ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_EQ(cpu.get_pc(), text + 22);
    ZLB_EXPECT_EQ(fx.read(0x28), 0x000E0000u);
    ZLB_EXPECT_EQ(fx.read(0x24), 1u);
    for (unsigned step = 0; step < 3; ++step) ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_EQ(cpu.get_pc(), text + 22); // one genuine loop while busy
    for (unsigned read = 0; read < 32; ++read) ZLB_EXPECT_EQ(fx.read(0x24), 1u);
    fx.soc.tick(kermit::cycles_from_periph_ticks(1) - 1);
    ZLB_EXPECT_EQ(fx.read(0x24), 1u); // fractional CPU time is insufficient
    fx.soc.tick(1);
    for (unsigned step = 0; step < 3; ++step) ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_EQ(cpu.get_pc(), text + sizeof(code));
    ZLB_EXPECT_EQ(fx.read(0x24), 0u);
    ZLB_EXPECT_EQ(cpu.r[1], 0u);
    ZLB_EXPECT_TRUE(fx.device().summary().find("commands=1 completed=1") != std::string::npos);
    ZLB_EXPECT_EQ(fx.soc.pending_irq_count(), 0u);
}

ZLB_TEST(emc_native_cold_sequence_retains_configuration_and_clears_only_busy) {
    EmcFixture fx;
    fx.native_config();
    fx.command(0x000E0000, 1);
    ZLB_EXPECT_EQ(fx.read(0x24), 1u);
    fx.tick();
    // The original driver delays, writes30 then0, then issues the five below.
    fx.soc.tick(kermit::cycles_from_periph_ticks(200000));
    fx.write(0x24, 0x30);
    ZLB_EXPECT_EQ(fx.read(0x24), 0x10u); // echoed20 did not inject acknowledgement
    fx.write(0x24, 0);
    fx.tick();
    ZLB_EXPECT_EQ(fx.read(0x24), 0u); // cancellation cannot produce a stale ack
    for (u32 payload : {0x00040400u, 0x00020000u, 0x00020000u, 0x00000031u}) {
        fx.command(payload, 1);
        ZLB_EXPECT_EQ(fx.read(0x24), 1u);
        fx.tick();
        ZLB_EXPECT_EQ(fx.read(0x24), 0u);
    }
    fx.command(0x00200000, 3);
    ZLB_EXPECT_EQ(fx.read(0x24), 3u);
    fx.tick();
    ZLB_EXPECT_EQ(fx.read(0x24), 2u); // only busy cleared; native TST #1 exits
    fx.write(0x00, fx.read(0x00) | 0x600);
    fx.write(0x230, 0);
    fx.write(0x244, 1);
    fx.write(0x234, 0);
    fx.write(0x230, 1);
    ZLB_EXPECT_EQ(fx.read(0x00), 0x2651u);
    ZLB_EXPECT_EQ(fx.read(0x04), 0x00072233u);
    ZLB_EXPECT_EQ(fx.read(0x08), 0x07725245u);
    ZLB_EXPECT_EQ(fx.read(0x14), 0x001F0C0Fu);
    ZLB_EXPECT_EQ(fx.read(0x38), 0x8Cu);
    ZLB_EXPECT_EQ(fx.read(0x3C), 0x69462300u);
    ZLB_EXPECT_EQ(fx.read(0x240), 0u);
    ZLB_EXPECT_TRUE(fx.device().summary().find("commands=6 completed=6 rejected=0") != std::string::npos);
    ZLB_EXPECT_TRUE(fx.device().summary().find("last-command=00200000 last-control=00000003") != std::string::npos);
    ZLB_EXPECT_EQ(fx.soc.pending_irq_count(), 0u);
}

ZLB_TEST(emc_native_smc118_and119_instructions_request_and_release_mode) {
    EmcFixture fx;
    constexpr u32 text = 0x81000000;
    constexpr u32 globals = text + 0x3140;
    constexpr u32 returned = text + 0x3F00;
    fx.bus.add_ram("EmcModeReplay", 0x4000, text, "original SceDriverTzs mode services");
    // Unchanged native handlers, including literal native globals addresses.
    constexpr u8 enter[] = {
        0x43, 0xF2, 0x40, 0x13, 0xC8, 0xF2, 0x00, 0x13,
        0x1B, 0x68, 0x1B, 0xB1, 0x5A, 0x6A, 0x12, 0xF0,
        0x20, 0x0F, 0x01, 0xD0, 0x00, 0x20, 0x70, 0x47,
        0x10, 0x20, 0x58, 0x62, 0xBF, 0xF3, 0x5F, 0x8F,
        0x59, 0x6A, 0x11, 0xF0, 0x20, 0x0F, 0xFB, 0xD0,
        0x00, 0x20, 0x70, 0x47,
    };
    constexpr u8 leave[] = {
        0x43, 0xF2, 0x40, 0x13, 0xC8, 0xF2, 0x00, 0x13,
        0x1B, 0x68, 0x3B, 0xB1, 0x5A, 0x6A, 0x12, 0xF0,
        0x20, 0x0F, 0x03, 0xD0, 0x00, 0x20, 0x58, 0x62,
        0xBF, 0xF3, 0x5F, 0x8F, 0x00, 0x20, 0x70, 0x47,
    };
    fx.bus.write_bytes(text + 0x1910, enter, sizeof(enter));
    fx.bus.write_bytes(text + 0x18F0, leave, sizeof(leave));
    fx.bus.write32(globals, kermit::kEmcTopBase);
    ArmCore cpu(fx.bus);
    cpu.reset((text + 0x1910) | 1u);
    cpu.r[14] = returned | 1u;
    unsigned steps = 0;
    while (cpu.get_pc() != text + 0x1930 && steps++ < 32) ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_EQ(cpu.get_pc(), text + 0x1930);
    ZLB_EXPECT_EQ(fx.read(0x24), 0x10u);
    for (unsigned step = 0; step < 3; ++step) ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_EQ(cpu.get_pc(), text + 0x1930); // wait for hardware bit20
    fx.tick();
    ZLB_EXPECT_EQ(fx.read(0x24), 0x30u);
    for (unsigned step = 0; step < 5; ++step) ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_EQ(cpu.get_pc(), returned);
    ZLB_EXPECT_EQ(cpu.r[0], 0u); // actual unchanged handler returns zero

    // Commands preserve30, while completion clears only bit0.
    fx.command(0x00000031, 1);
    ZLB_EXPECT_EQ(fx.read(0x24), 0x31u);
    fx.tick();
    ZLB_EXPECT_EQ(fx.read(0x24), 0x30u);
    cpu.set_pc(text + 0x18F0); // set_pc takes the actual aligned Thumb PC
    cpu.r[14] = returned | 1u;
    steps = 0;
    while (cpu.get_pc() != returned && steps++ < 32) ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_EQ(cpu.get_pc(), returned);
    ZLB_EXPECT_EQ(cpu.r[0], 0u);
    ZLB_EXPECT_EQ(fx.read(0x24), 0u);
    fx.tick();
    ZLB_EXPECT_EQ(fx.read(0x24), 0u);
    ZLB_EXPECT_EQ(fx.soc.pending_irq_count(), 0u);
}

ZLB_TEST(emc_unknown_payloads_and_control_flags_remain_busy_until_explicit_reset) {
    EmcFixture fx;
    struct Request { u32 payload; u32 control; };
    constexpr Request rejected[] = {
        {0xDEADBEEF, 1}, {0x000E0000, 3}, {0x00200000, 1},
        {0x00000031, 2}, {0x00040400, 5}, {0x00020000, 0x80000001},
        {0x000E0000, 0x20}, {0x000E0000, 0x40},
    };
    for (const auto& request : rejected) {
        fx.write(0x24, 0);
        fx.write(0x28, request.payload);
        fx.write(0x24, request.control);
        ZLB_EXPECT_EQ(fx.read(0x24) & 1u, 1u);
        fx.soc.tick(kermit::cycles_from_periph_ticks(1000000));
        for (unsigned read = 0; read < 16; ++read) ZLB_EXPECT_EQ(fx.read(0x24) & 1u, 1u);
        ZLB_EXPECT_TRUE(fx.device().summary().find("unsupported") != std::string::npos);
        fx.command(0x000E0000, 1); // a new command cannot erase an unknown operation
        fx.tick();
        ZLB_EXPECT_EQ(fx.read(0x24) & 1u, 1u);
        fx.bus.write16(kermit::kEmcTopBase + 0x26, 0); // high lanes cannot reset
        ZLB_EXPECT_EQ(fx.read(0x24) & 1u, 1u);
        fx.write(0x24, 0);
        ZLB_EXPECT_EQ(fx.read(0x24), 0u);
        fx.command(0x000E0000, 1);
        fx.tick();
        ZLB_EXPECT_EQ(fx.read(0x24), 0u);
    }
    ZLB_EXPECT_EQ(fx.soc.pending_irq_count(), 0u);
}

ZLB_TEST(emc_byte_lanes_do_not_submit_cancel_or_inject_hardware_status) {
    EmcFixture fx;
    const u32 base = kermit::kEmcTopBase;
    fx.bus.write8(base + 0x04, 0x33);
    fx.bus.write8(base + 0x05, 0x22);
    fx.bus.write16(base + 0x06, 7);
    ZLB_EXPECT_EQ(fx.read(0x04), 0x00072233u);
    ZLB_EXPECT_EQ(fx.bus.read16(base + 0x05), 0x0722u);
    fx.bus.write16(base + 0x28, 0);
    fx.bus.write16(base + 0x2A, 0xE);
    ZLB_EXPECT_EQ(fx.read(0x28), 0x000E0000u);
    ZLB_EXPECT_EQ(fx.read(0x24), 0u);
    fx.bus.write8(base + 0x24, 1);
    ZLB_EXPECT_EQ(fx.read(0x24), 1u);
    fx.bus.write8(base + 0x25, 0x20); // not a low-lane mode request
    fx.bus.write16(base + 0x26, 0);
    ZLB_EXPECT_EQ(fx.read(0x24), 1u);
    fx.tick();
    ZLB_EXPECT_EQ(fx.read(0x24), 0u);
    fx.bus.write8(base + 0x24, 1); // staged unknown high flag must now reject
    fx.tick();
    ZLB_EXPECT_EQ(fx.read(0x24), 1u);
    fx.bus.write8(base + 0x25, 0);
    ZLB_EXPECT_EQ(fx.read(0x24), 1u); // a high-lane zero cannot clear busy
    fx.bus.write8(base + 0x24, 0);
    ZLB_EXPECT_EQ(fx.read(0x24), 0u);

    fx.write(0x28, 0x00000031);
    fx.write(0x24, 0x21); // echoed acknowledgement20 is not a request10
    ZLB_EXPECT_EQ(fx.read(0x24), 1u);
    fx.tick();
    ZLB_EXPECT_EQ(fx.read(0x24), 0u);
    fx.bus.write8(base + 0x25, 0x10); // high byte cannot request native mode
    fx.tick();
    ZLB_EXPECT_EQ(fx.read(0x24), 0u);
    fx.write(0x24, 0x10);
    ZLB_EXPECT_EQ(fx.read(0x24), 0x10u);
    fx.tick();
    ZLB_EXPECT_EQ(fx.read(0x24), 0x30u);
    fx.bus.write16(base + 0x26, 0);
    ZLB_EXPECT_EQ(fx.read(0x24), 0x30u);
    fx.write(0x24, 0);
    ZLB_EXPECT_EQ(fx.read(0x24), 0u);
}

ZLB_TEST(emc_calibration_registers_are_snapshots_without_fabricated_events) {
    EmcFixture fx;
    fx.write(0x230, 0);
    fx.write(0x244, 1);
    fx.write(0x234, 0xCAFEBABE);
    fx.write(0x230, 1);
    fx.write(0x240, 0xFFFFFFFF); // status is hardware-owned
    ZLB_EXPECT_FALSE(fx.device().poke_register("CAL_STATUS", 1));
    fx.bus.write8(kermit::kEmcTopBase + 0x245, 0xFF);
    fx.bus.write16(kermit::kEmcTopBase + 0x246, 0xFFFF);
    fx.soc.tick(kermit::cycles_from_periph_ticks(1000000));
    ZLB_EXPECT_EQ(fx.read(0x230), 1u);
    ZLB_EXPECT_EQ(fx.read(0x234), 0xCAFEBABEu);
    ZLB_EXPECT_EQ(fx.read(0x240), 0u);
    ZLB_EXPECT_EQ(fx.read(0x244), 0u);
    ZLB_EXPECT_EQ(fx.soc.pending_irq_count(), 0u);
}

ZLB_TEST(emc_mapping_diagnostics_and_power_reset_cancel_all_pending_work) {
    EmcFixture fx;
    Device& device = fx.device();
    ZLB_EXPECT_EQ(device.base(), kermit::kEmcTopBase);
    ZLB_EXPECT_EQ(device.size(), 0x1000u);
    ZLB_EXPECT_EQ(fx.bus.find_device(kermit::kEmcTopBase + 0xFFF), &device);
    ZLB_EXPECT_TRUE(fx.bus.find_device(kermit::kEmcTopBase + 0x1000) == nullptr);
    bool listed = false;
    for (Device* owned : fx.soc.devices()) if (owned == &device) listed = true;
    ZLB_EXPECT_TRUE(listed);
    ZLB_EXPECT_TRUE(std::string(device.register_name(kermit::kEmcTopBase + 0x25)) == "CONTROL_STATUS");
    ZLB_EXPECT_TRUE(device.register_name(kermit::kEmcTopBase + 0x20) == nullptr);
    ZLB_EXPECT_EQ(fx.read(0x20), 0xFFFFFFFFu);
    ZLB_EXPECT_EQ(fx.read(0x100), 0xFFFFFFFFu);
    fx.write(0x100, 0); // reserved storage is not universal success/readback
    ZLB_EXPECT_EQ(fx.read(0x100), 0xFFFFFFFFu);
    for (u32 control : {0x11u, 0x80000001u}) {
        fx.native_config();
        fx.write(0x28, 0x000E0000);
        fx.write(0x24, control);
        fx.soc.reset();
        fx.tick();
        for (u32 offset : {0u, 4u, 8u, 0x24u, 0x28u, 0x230u, 0x234u, 0x240u, 0x244u})
            ZLB_EXPECT_EQ(fx.read(offset), 0u);
        ZLB_EXPECT_TRUE(device.summary().find("commands=0 completed=0 rejected=0 resets=0") != std::string::npos);
        u64 value = 0;
        ZLB_EXPECT_TRUE(device.peek_register("CONTROL_STATUS", value));
        ZLB_EXPECT_EQ(value, 0u);
        ZLB_EXPECT_FALSE(device.peek_register("UNKNOWN", value));
        fx.command(0x000E0000, 1);
        fx.tick();
        ZLB_EXPECT_EQ(fx.read(0x24), 0u);
    }
    std::vector<std::string> details;
    device.describe(details);
    ZLB_EXPECT_EQ(details.size(), 4u);
    ZLB_EXPECT_TRUE(details[1].find("emulator latency") != std::string::npos);
    ZLB_EXPECT_TRUE(details[3].find("no calibration event") != std::string::npos);
    ZLB_EXPECT_EQ(fx.soc.pending_irq_count(), 0u);
}
