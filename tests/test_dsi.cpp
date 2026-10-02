// Firmware 1.04 DSI0 timing tests drive native register writes, not callbacks.
#include <limits>
#include <memory>

#include "bus/bus.h"
#include "hw/soc/soc_internal.h"
#include "test_framework.h"

using namespace zlb;

namespace {

struct DsiFixture {
    std::unique_ptr<Bus> owner = std::make_unique<Bus>();
    Bus& bus = *owner;
    kermit::DsiController* dsi = nullptr;
    bool irq = false;
    unsigned assertions = 0;

    DsiFixture() {
        auto device = std::make_unique<kermit::DsiController>();
        dsi = device.get();
        bus.add_device(std::move(device));
        dsi->set_irq_callback([this](u32 id, bool asserted) {
            ZLB_EXPECT_EQ(id, 213u);
            irq = asserted;
            if (asserted) ++assertions;
        });
    }

    void write(u32 offset, u32 value) { bus.write32(kermit::kDsi0Base + offset, value); }
    u32 read(u32 offset) { return bus.read32(kermit::kDsi0Base + offset); }
    void start(u32 mask = 2) {
        // Actual Lowio StartDisplay progressive VIC0 subset, control last.
        write(0x04, 0);
        write(0x08, 0xC4E);
        write(0x0C, 0x252);
        write(0x50, read(0x50));
        write(0x54, mask);
        write(0x00, 1);
    }
};

}  // namespace

ZLB_TEST(dsi_no_vblank_until_native_progressive_timing_is_enabled) {
    DsiFixture fx;
    fx.write(0x54, 2);
    fx.dsi->tick(1001000);
    ZLB_EXPECT_EQ(fx.read(0x50), 0u);
    ZLB_EXPECT_FALSE(fx.irq);

    fx.write(0x00, 1);  // enable alone is not a configured native head
    fx.dsi->tick(1001000);
    ZLB_EXPECT_EQ(fx.dsi->frame_counter(), 0u);
    fx.write(0x08, 0xC4E);
    fx.write(0x0C, 0x252);
    fx.write(0x04, 1);  // interlaced/other progressive field is unsupported
    fx.dsi->tick(1001000);
    ZLB_EXPECT_EQ(fx.read(0x50), 0u);
    fx.write(0x04, 0);
    for (u32 other_mode : {2u, 3u, 4u}) {
        fx.write(0, other_mode);
        fx.dsi->tick(1001000);
        ZLB_EXPECT_EQ(fx.dsi->frame_counter(), 0u);
        ZLB_EXPECT_FALSE(fx.irq);
        ZLB_EXPECT_EQ(fx.read(0), other_mode);
    }

    fx.write(0, 1);
    fx.dsi->tick(16683);
    ZLB_EXPECT_EQ(fx.dsi->frame_counter(), 0u);
    fx.dsi->tick(1);
    ZLB_EXPECT_EQ(fx.dsi->frame_counter(), 1u);
    ZLB_EXPECT_TRUE(fx.irq);
    ZLB_EXPECT_EQ(fx.assertions, 1u);
}

ZLB_TEST(dsi_vblank_latches_while_masked_and_ack_deasserts_level) {
    DsiFixture fx;
    fx.start(0);
    fx.dsi->tick(1001000);  // 60 pending frames coalesce in one status bit
    ZLB_EXPECT_EQ(fx.dsi->frame_counter(), 60u);
    ZLB_EXPECT_EQ(fx.read(0x50), 2u);
    ZLB_EXPECT_FALSE(fx.irq);
    fx.write(0x54, 2);
    ZLB_EXPECT_TRUE(fx.irq);
    ZLB_EXPECT_EQ(fx.assertions, 1u);
    fx.write(0x50, 0);  // native W1C must not consume pending on zero
    ZLB_EXPECT_TRUE(fx.irq);
    fx.write(0x54, 0);
    ZLB_EXPECT_FALSE(fx.irq);
    ZLB_EXPECT_EQ(fx.read(0x50), 2u);
    fx.write(0x54, 2);
    ZLB_EXPECT_TRUE(fx.irq);
    fx.write(0x50, fx.read(0x50));  // native handler read/write-back
    ZLB_EXPECT_EQ(fx.read(0x50), 0u);
    ZLB_EXPECT_FALSE(fx.irq);
    fx.dsi->tick(16683);
    ZLB_EXPECT_FALSE(fx.irq);
    fx.dsi->tick(1);
    ZLB_EXPECT_TRUE(fx.irq);
}

ZLB_TEST(dsi_fractional_frame_phase_is_exact_and_chunk_independent) {
    DsiFixture batch;
    DsiFixture sliced;
    batch.start();
    sliced.start();
    batch.dsi->tick(1000000);
    for (unsigned i = 0; i < 1000; ++i) sliced.dsi->tick(1000);
    ZLB_EXPECT_EQ(batch.dsi->frame_counter(), 59u);
    ZLB_EXPECT_EQ(sliced.dsi->frame_counter(), 59u);
    batch.write(0x50, 2);
    sliced.write(0x50, 2);
    batch.dsi->tick(999);
    sliced.dsi->tick(999);
    ZLB_EXPECT_EQ(batch.dsi->frame_counter(), 59u);
    ZLB_EXPECT_FALSE(batch.irq);
    ZLB_EXPECT_FALSE(sliced.irq);
    batch.dsi->tick(1);
    sliced.dsi->tick(1);
    ZLB_EXPECT_EQ(batch.dsi->frame_counter(), 60u);
    ZLB_EXPECT_EQ(sliced.dsi->frame_counter(), 60u);
    batch.write(0x50, 2);
    sliced.write(0x50, 2);
    batch.dsi->tick(16683);
    sliced.dsi->tick(8000);
    sliced.dsi->tick(8683);
    ZLB_EXPECT_EQ(batch.dsi->frame_counter(), 60u);
    ZLB_EXPECT_EQ(sliced.dsi->frame_counter(), 60u);
    batch.dsi->tick(1);
    sliced.dsi->tick(1);
    ZLB_EXPECT_EQ(batch.dsi->frame_counter(), 61u);
    ZLB_EXPECT_EQ(sliced.dsi->frame_counter(), 61u);
}

ZLB_TEST(dsi_large_tick_does_not_overflow_or_iterate_per_frame) {
    DsiFixture fx;
    fx.start();
    fx.dsi->tick(1);  // begin with a nonzero fractional phase
    fx.dsi->tick(std::numeric_limits<u64>::max());
    // Exact floor((2^64 microseconds)*60/1001000), remainder 960.
    ZLB_EXPECT_EQ(fx.dsi->frame_counter(), 1105698945477096ull);
    ZLB_EXPECT_EQ(fx.read(0x50), 2u);
    ZLB_EXPECT_EQ(fx.assertions, 1u);
    fx.write(0x50, 2);
    fx.dsi->tick(16667);  // remainder becomes1000980, still short of1001000
    ZLB_EXPECT_EQ(fx.dsi->frame_counter(), 1105698945477096ull);
    ZLB_EXPECT_FALSE(fx.irq);
    fx.dsi->tick(1);
    ZLB_EXPECT_EQ(fx.dsi->frame_counter(), 1105698945477097ull);
    ZLB_EXPECT_TRUE(fx.irq);
}

ZLB_TEST(dsi_byte_mask_and_w1c_writes_only_touch_supplied_lanes) {
    DsiFixture fx;
    fx.start(0);
    fx.dsi->tick(16684);
    fx.bus.write8(kermit::kDsi0Base + 0x55, 0xA5);
    ZLB_EXPECT_EQ(fx.read(0x54), 0xA500u);
    ZLB_EXPECT_FALSE(fx.irq);
    fx.bus.write8(kermit::kDsi0Base + 0x54, 2);
    ZLB_EXPECT_EQ(fx.read(0x54), 0xA502u);
    ZLB_EXPECT_TRUE(fx.irq);
    fx.bus.write8(kermit::kDsi0Base + 0x51, 0xFF);
    fx.bus.write16(kermit::kDsi0Base + 0x52, 0xFFFF);
    ZLB_EXPECT_EQ(fx.read(0x50), 2u);
    ZLB_EXPECT_TRUE(fx.irq);
    fx.bus.write8(kermit::kDsi0Base + 0x50, 0);
    ZLB_EXPECT_TRUE(fx.irq);
    fx.bus.write8(kermit::kDsi0Base + 0x50, 2);
    ZLB_EXPECT_FALSE(fx.irq);
    ZLB_EXPECT_EQ(fx.read(0x50), 0u);
}

ZLB_TEST(dsi_stop_and_reset_clear_timing_without_fake_packet_readiness) {
    DsiFixture fx;
    fx.start();
    fx.dsi->tick(8000);
    fx.write(0, 0);
    fx.dsi->tick(1001000);
    ZLB_EXPECT_EQ(fx.dsi->frame_counter(), 0u);
    fx.write(0, 1);
    fx.dsi->tick(16683);
    ZLB_EXPECT_FALSE(fx.irq);  // documented fresh-start phase
    fx.dsi->tick(1);
    fx.write(0, 0);
    ZLB_EXPECT_TRUE(fx.irq);  // stopping is not a status ACK
    fx.dsi->tick(1001000);
    ZLB_EXPECT_EQ(fx.dsi->frame_counter(), 1u);

    // Real SendBlanking queues a packet and polls +414 bit24. Timing alone
    // must not invent packet completion or a scanline-ready return value.
    fx.write(0x500, 0xA30000A4);
    fx.write(0x50C, 0x01000000);
    fx.write(0x508, 0xFFFFFFFF);
    fx.dsi->tick(1001000);
    ZLB_EXPECT_EQ(fx.read(0x500), 0xA30000A4u);
    ZLB_EXPECT_EQ(fx.read(0x50C), 0x01000000u);
    ZLB_EXPECT_EQ(fx.read(0x414), 0u);
    ZLB_EXPECT_EQ(fx.read(0x48), 0u);
    ZLB_EXPECT_EQ(fx.read(0x4C), 0u);

    fx.bus.reset_devices();
    ZLB_EXPECT_FALSE(fx.irq);
    ZLB_EXPECT_EQ(fx.dsi->frame_counter(), 0u);
    for (u32 offset : {0u, 4u, 8u, 0xCu, 0x50u, 0x54u, 0x500u, 0x50Cu})
        ZLB_EXPECT_EQ(fx.read(offset), 0u);
    fx.dsi->tick(1001000);
    ZLB_EXPECT_EQ(fx.read(0x50), 0u);
    fx.start();
    fx.dsi->tick(16683);
    ZLB_EXPECT_FALSE(fx.irq);
    fx.dsi->tick(1);
    ZLB_EXPECT_TRUE(fx.irq);
}

ZLB_TEST(kermit_dsi_native_irq213_wiring_uses_microsecond_time_and_level_ack) {
    auto owner = std::make_unique<Bus>();
    Bus& bus = *owner;
    KermitBlock soc(bus, nullptr);
    soc.install();
    soc.reset();
    constexpr u32 dist = kermit::kScuBase + kermit::kGicDistOffset;
    constexpr u32 icc = kermit::kScuBase + kermit::kIccOffset;
    constexpr u32 irq = 213;
    ZLB_EXPECT_TRUE(bus.find_device(kermit::kDsi0Base) != nullptr);
    bool listed = false;
    for (Device* device : soc.devices()) if (device->name() == "Kermit.DSI0") listed = true;
    ZLB_EXPECT_TRUE(listed);

    // The guest enables a real level-sensitive SPI; the DSI model never
    // supplies distributor/CPU-interface enables or changes the target CPU.
    bus.write32(dist, 1);
    bus.write32(icc, 1);
    bus.write32(icc + 4, 0xFF);
    bus.write32(dist + 0x100 + (irq / 32) * 4, 1u << (irq % 32));
    bus.write8(dist + 0x400 + irq, 0xA0);
    bus.write8(dist + 0x800 + irq, 1);
    bus.write32(kermit::kDsi0Base + 4, 0);
    bus.write32(kermit::kDsi0Base + 8, 0xC4E);
    bus.write32(kermit::kDsi0Base + 0xC, 0x252);
    bus.write32(kermit::kDsi0Base + 0x54, 2);
    bus.write32(kermit::kDsi0Base, 1);
    soc.tick(16683ull * 333);  // 333 A9 cycles per peripheral microsecond
    ZLB_EXPECT_FALSE(kermit_irq_line(bus));
    soc.tick(333);
    ZLB_EXPECT_EQ(bus.read32(kermit::kDsi0Base + 0x50), 2u);
    ZLB_EXPECT_TRUE(kermit_irq_line(bus));
    ZLB_EXPECT_EQ(bus.read32(icc + 0x0C) & 0x3FFu, irq);
    bus.write32(kermit::kDsi0Base + 0x50, 2);
    bus.write32(icc + 0x10, irq);
    ZLB_EXPECT_FALSE(kermit_irq_line(bus));
    ZLB_EXPECT_EQ(soc.pending_irq_count(), 0u);
    ZLB_EXPECT_EQ(bus.read32(icc + 0x0C) & 0x3FFu, 1023u);
    soc.reset();
    ZLB_EXPECT_EQ(bus.read32(kermit::kDsi0Base), 0u);
    ZLB_EXPECT_EQ(bus.read32(kermit::kDsi0Base + 0x50), 0u);
    ZLB_EXPECT_FALSE(kermit_irq_line(bus));
}

ZLB_TEST(dsi_board_frame_boundaries_are_batched_and_independent_of_irq_mask) {
    DsiFixture fx;
    std::vector<u64> boundaries;
    fx.dsi->set_frame_callback([&](u64 count) { boundaries.push_back(count); });
    fx.dsi->tick(1001000);
    ZLB_EXPECT_TRUE(boundaries.empty());
    fx.start(0);
    fx.dsi->tick(16683);
    ZLB_EXPECT_TRUE(boundaries.empty());
    fx.dsi->tick(1);
    ZLB_EXPECT_EQ(boundaries.size(), 1u);
    if (!boundaries.empty()) ZLB_EXPECT_EQ(boundaries[0], 1u);
    ZLB_EXPECT_FALSE(fx.irq);
    fx.dsi->tick(1001000);
    ZLB_EXPECT_EQ(boundaries.size(), 2u);
    if (boundaries.size() > 1) ZLB_EXPECT_EQ(boundaries[1], 60u);
    fx.write(0, 0);
    fx.dsi->tick(1001000);
    ZLB_EXPECT_EQ(boundaries.size(), 2u);
    fx.dsi->reset();
    fx.start(0);
    fx.dsi->tick(16683);
    ZLB_EXPECT_EQ(boundaries.size(), 2u);
    fx.dsi->tick(1);
    ZLB_EXPECT_EQ(boundaries.size(), 3u);
}
