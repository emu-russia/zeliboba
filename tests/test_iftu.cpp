// Native IFTU0 scanout tests use guest-created patterns, not firmware assets.
#include <memory>

#include "bus/bus.h"
#include "cpu/arm/arm_core.h"
#include "hw/soc/soc_internal.h"
#include "test_framework.h"

using namespace zlb;

namespace {

struct IftuFixture {
    std::unique_ptr<Bus> owner = std::make_unique<Bus>();
    Bus& bus = *owner;
    KermitBlock kermit{bus, nullptr};

    IftuFixture() {
        bus.unmapped_reads_zero = true;
        bus.add_ram("display guest SRAM", kermit::kScratchpadSramSize,
                    kermit::kScratchpadSramBase, "native scanout test");
        kermit.install();
        kermit.reset();
    }

    void bank(unsigned plane, unsigned index, u32 address, u32 width, u32 height, u32 padding = 0) {
        const u32 start = kermit::kIftu0Base + plane * 0x1000 + 0x200 + index * 0x100;
        // Stores recovered from lowio.elf 0x810058DC and Enable 0x8100690C.
        bus.write32(start + 0x00, address);
        bus.write32(start + 0x04, 0);
        bus.write32(start + 0x08, 0);
        bus.write32(start + 0x40, 0x10);
        bus.write32(start + 0x44, width);
        bus.write32(start + 0x48, height);
        bus.write32(start + 0x4C, 0);
        bus.write32(start + 0x54, padding);
        bus.write32(start + 0x58, 0);
        bus.write32(start + 0xA0, 0x2000);
        bus.write32(start + 0xA4, width);
        bus.write32(start + 0xA8, height);
        bus.write32(start + 0xC0, 0x10000);
        bus.write32(start + 0xC4, 0x10000);
    }

    void enable(unsigned plane) {
        const u32 local = kermit::kIftu0Base + plane * 0x1000;
        bus.write32(local + 0x58, 0x108);
        bus.write32(local + 0x50, 1);
        bus.write32(local + 0x180, 1);
        bus.write32(kermit::kIftu0Base + 0x2010 + plane * 8, 0);
        const u32 shift = 1 + plane * 2;
        const u32 modes = bus.read32(kermit::kIftu0Base + 0x2004);
        bus.write32(kermit::kIftu0Base + 0x2004, (modes & ~(3u << shift)) | (1u << shift));
        bus.write32(kermit::kIftu0Base + 0x2000, 1);
    }

    void dsi_start() {
        bus.write32(kermit::kDsi0Base + 4, 0);
        bus.write32(kermit::kDsi0Base + 8, 0xC4E);
        bus.write32(kermit::kDsi0Base + 0xC, 0x252);
        bus.write32(kermit::kDsi0Base + 0x54, 0);  // independent of IRQ213
        bus.write32(kermit::kDsi0Base, 1);
    }

    void enable_irq(unsigned irq) {
        constexpr u32 dist = kermit::kScuBase + kermit::kGicDistOffset;
        constexpr u32 icc = kermit::kScuBase + kermit::kIccOffset;
        bus.write32(dist, 1);
        bus.write32(icc, 1);
        bus.write32(icc + 4, 0xFF);
        bus.write32(dist + 0x100 + (irq / 32) * 4, 1u << (irq % 32));
        bus.write8(dist + 0x400 + irq, 0xA0);
        bus.write8(dist + 0x800 + irq, 1);
    }

    void frame() { kermit.tick(16684ull * 333); }
    u32 current(unsigned plane = 0) { return (bus.read32(kermit::kIftu0Base + plane * 0x1000 + 4) >> 1) & 1; }
};

}  // namespace

ZLB_TEST(kermit_iftu_native_enable_exports_guest_rgba_with_padded_rows) {
    IftuFixture fx;
    constexpr u32 address = kermit::kScratchpadSramBase;
    constexpr int stride = 512 * 4;
    fx.bus.write32(address, 0x00332211);
    fx.bus.write32(address + stride, 0x00665544);
    fx.bus.write8(address + 480 * 4, 0xCA);  // row padding must not become a pixel
    fx.bank(0, 0, address, 480, 272, 4 * (512 - 480));
    fx.bank(0, 1, address, 480, 272, 4 * (512 - 480));

    int width = -1, height = -1, pitch = -1, bpp = -1;
    ZLB_EXPECT_TRUE(fx.kermit.framebuffer(width, height, pitch, &bpp) == nullptr);
    ZLB_EXPECT_EQ(bpp, 0);
    fx.enable(0);
    const u8* pixels = fx.kermit.framebuffer(width, height, pitch, &bpp);
    ZLB_EXPECT_TRUE(pixels != nullptr);
    ZLB_EXPECT_EQ(width, 480);
    ZLB_EXPECT_EQ(height, 272);
    ZLB_EXPECT_EQ(pitch, stride);
    ZLB_EXPECT_EQ(bpp, 4);
    if (pixels) {
        ZLB_EXPECT_EQ(pixels[0], 0x11);
        ZLB_EXPECT_EQ(pixels[1], 0x22);
        ZLB_EXPECT_EQ(pixels[2], 0x33);
        ZLB_EXPECT_EQ(pixels[3], 0);  // genuine display pixels can have zero alpha
        ZLB_EXPECT_EQ(pixels[stride], 0x44);
        ZLB_EXPECT_EQ(pixels[480 * 4], 0xCA);
    }

    // Bank +4 is another address field, not row stride. Shared mode bits are
    // stored without inventing enable gates; only established controls govern.
    fx.bus.write32(kermit::kIftu0Base + 0x204, 0xDEADBEEF);
    fx.bus.write32(kermit::kIftu0Base + 0x2004, 1);
    pixels = fx.kermit.framebuffer(width, height, pitch, &bpp);
    ZLB_EXPECT_TRUE(pixels != nullptr);
    ZLB_EXPECT_EQ(pitch, stride);
    fx.bus.write8(address, 0x7A);
    if (pixels) ZLB_EXPECT_EQ(pixels[0], 0x7A);  // only guest memory supplies pixels

    // The native logo producer's complete 960x544 four-byte span fits the
    // already mapped 2 MiB SRAM; no additional framebuffer RAM is created.
    fx.bank(0, 0, address, 960, 544);
    fx.bank(0, 1, address, 960, 544);
    fx.bus.write8(address + 0x1FE000 - 4, 0x5B);
    pixels = fx.kermit.framebuffer(width, height, pitch, &bpp);
    ZLB_EXPECT_EQ(width, 960);
    ZLB_EXPECT_EQ(height, 544);
    ZLB_EXPECT_EQ(pitch, 3840);
    if (pixels) ZLB_EXPECT_EQ(pixels[0x1FE000 - 4], 0x5B);
    else ZLB_FAIL("native full panel guest buffer missing");

    // Enabling the old fixture cannot hide an already valid native scanout.
    fx.bus.write32(kermit::kDisplayBase, 1);
    ZLB_EXPECT_EQ(fx.kermit.framebuffer(width, height, pitch), pixels);
    ZLB_EXPECT_EQ(fx.kermit.frame_counter(), 0u);  // no fabricated native completion
}

ZLB_TEST(kermit_iftu_explicit_bank_selection_has_no_invented_deferred_flip) {
    IftuFixture fx;
    constexpr u32 first = kermit::kScratchpadSramBase;
    constexpr u32 second = first + 0x1000;
    constexpr u32 third = first + 0x2000;
    fx.bus.write8(first, 0x11);
    fx.bus.write8(second, 0x22);
    fx.bus.write8(third, 0x33);
    fx.bank(0, 0, first, 4, 2);
    fx.bank(0, 1, second, 4, 2);
    fx.enable(0);
    fx.kermit.tick(333000000);  // elapsed time does not invent a bank turnover
    int width = 0, height = 0, pitch = 0;
    const u8* pixels = fx.kermit.framebuffer(width, height, pitch);
    if (pixels) ZLB_EXPECT_EQ(pixels[0], 0x11);
    else ZLB_FAIL("selected first guest bank missing");
    ZLB_EXPECT_EQ(fx.bus.read32(kermit::kIftu0Base + 4) & 2u, 0u);
    ZLB_EXPECT_EQ(fx.bus.read32(kermit::kIftu0Base + 0x40), 0u);
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 0u);

    fx.bus.write32(kermit::kIftu0Base + 0x2010, 1);  // public native explicit select
    pixels = fx.kermit.framebuffer(width, height, pitch);
    if (pixels) ZLB_EXPECT_EQ(pixels[0], 0x22);
    else ZLB_FAIL("explicitly selected second guest bank missing");
    ZLB_EXPECT_EQ(fx.bus.read8(kermit::kIftu0Base + 4) & 2u, 2u);

    // The B plane uses its own banks and shared selector; no A/B blending is
    // inferred when the frontend exports the first supported active input.
    fx.bank(1, 1, third, 4, 2);
    fx.enable(1);
    fx.bus.write32(kermit::kIftu0Base + 0x2018, 1);
    fx.bus.write32(kermit::kIftu0Base + 0x50, 0);
    pixels = fx.kermit.framebuffer(width, height, pitch);
    if (pixels) ZLB_EXPECT_EQ(pixels[0], 0x33);
    else ZLB_FAIL("B plane explicit selection missing");
    ZLB_EXPECT_EQ(fx.bus.read32(kermit::kIftu0Base + 0x1004) & 2u, 2u);
}

ZLB_TEST(kermit_iftu_rejects_invalid_guest_span_and_reset_hides_pixels) {
    IftuFixture fx;
    constexpr u32 base = kermit::kIftu0Base;
    constexpr u32 address = kermit::kScratchpadSramBase;
    int width = 0, height = 0, pitch = 0, bpp = 0;
    fx.bank(0, 0, address, 4, 2);
    fx.enable(0);
    ZLB_EXPECT_TRUE(fx.kermit.framebuffer(width, height, pitch, &bpp) != nullptr);

    auto absent = [&] {
        ZLB_EXPECT_TRUE(fx.kermit.framebuffer(width, height, pitch, &bpp) == nullptr);
        ZLB_EXPECT_EQ(width, 0);
        ZLB_EXPECT_EQ(height, 0);
        ZLB_EXPECT_EQ(pitch, 0);
        ZLB_EXPECT_EQ(bpp, 0);
    };
    fx.bus.write32(base + 0x200, base);  // MMIO must never be read as framebuffer RAM
    const u64 reads = fx.bus.stats.reads;
    const u64 mmio = fx.bus.stats.mmio;
    absent();
    ZLB_EXPECT_EQ(fx.bus.stats.reads, reads);
    ZLB_EXPECT_EQ(fx.bus.stats.mmio, mmio);
    fx.bus.write32(base + 0x200, address + kermit::kScratchpadSramSize - 8);
    absent();  // the full rows cross the mapped region
    fx.bus.write32(base + 0x200, 0xFFFFFFF0);
    absent();  // PA addition would wrap
    fx.bus.write32(base + 0x200, address);
    fx.bus.write32(base + 0x254, 0xFFFFFFFF);
    absent();  // padding/stride cannot overflow the exported integer dimensions
    fx.bus.write32(base + 0x254, 0);
    fx.bus.write32(base + 0x244, 0xFFFFFFFF);
    absent();
    fx.bus.write32(base + 0x244, 4);
    fx.bus.write32(base + 0x248, 0);
    absent();
    fx.bus.write32(base + 0x248, 2);
    fx.bus.write32(base + 0x240, 0x20);
    absent();  // unsupported formats do not borrow a guessed layout
    fx.bus.write32(base + 0x240, 0x10);
    fx.bus.write32(base + 0x24C, 1);
    absent();
    fx.bus.write32(base + 0x24C, 0);
    fx.bus.write32(base + 0x2010, 2);
    absent();
    fx.bus.write32(base + 0x2010, 0);
    fx.bus.write32(base + 0x2000, 0);
    absent();
    fx.bus.write32(base + 0x2000, 1);
    fx.bus.write32(base + 0x50, 0);
    absent();
    fx.bus.write32(base + 0x50, 1);
    ZLB_EXPECT_TRUE(fx.kermit.framebuffer(width, height, pitch) != nullptr);

    // Invalid/disabled native state may fall back only to an explicitly enabled
    // synthetic fixture. Reset disables both and retains no stale host pixels.
    fx.bus.write32(kermit::kDisplayBase, 1);
    fx.bus.write32(base + 0x24C, 1);
    ZLB_EXPECT_TRUE(fx.kermit.framebuffer(width, height, pitch, &bpp) != nullptr);
    ZLB_EXPECT_EQ(bpp, 2);
    fx.kermit.reset();
    absent();
    ZLB_EXPECT_EQ(fx.bus.read32(base + 0x200), 0u);
    ZLB_EXPECT_EQ(fx.bus.read32(base + 0x2010), 0u);
    ZLB_EXPECT_EQ(fx.kermit.frame_counter(), 0u);
}

ZLB_TEST(iftu_cold_deferred_bank_moves_at_dsi_boundary_before_physical_irq204) {
    IftuFixture fx;
    constexpr u32 base = kermit::kIftu0Base;
    constexpr u32 guest = kermit::kScratchpadSramBase;
    fx.bus.write32(guest, 0x007B3519);  // ordinary guest-created pixels
    fx.bank(0, 0, 0, 960, 544);
    fx.bus.write32(base + 0x24C, 1);
    fx.bank(0, 1, guest, 960, 544);
    fx.enable(0);
    fx.bus.write32(base + 0x2004, 3);  // mode01 plus alpha bit, not a commit
    fx.enable_irq(204);
    int width = 0, height = 0, stride = 0;
    ZLB_EXPECT_TRUE(fx.kermit.framebuffer(width, height, stride) == nullptr);
    fx.kermit.tick(1001000ull * 333);  // no DSI start, no physical boundary
    ZLB_EXPECT_EQ(fx.current(), 0u);
    ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
    fx.dsi_start();
    fx.kermit.tick(16683ull * 333);
    ZLB_EXPECT_EQ(fx.current(), 0u);
    fx.kermit.tick(333);
    ZLB_EXPECT_EQ(fx.current(), 1u);
    ZLB_EXPECT_EQ(fx.bus.read32(base + 0x2010), 0u);  // manual request retained
    ZLB_EXPECT_EQ(fx.bus.read32(base + 0x40), 0u);  // no invented status bits
    ZLB_EXPECT_TRUE(kermit_irq_line(fx.bus));
    constexpr u32 icc = kermit::kScuBase + kermit::kIccOffset;
    ZLB_EXPECT_EQ(fx.bus.read32(icc + 0xC) & 0x3FF, 204u);
    const u8* pixels = fx.kermit.framebuffer(width, height, stride);
    ZLB_EXPECT_EQ(width, 960);
    ZLB_EXPECT_EQ(height, 544);
    ZLB_EXPECT_EQ(stride, 3840);
    if (pixels) ZLB_EXPECT_EQ(pixels[0], 0x19);
    else ZLB_FAIL("guest bank did not become active at the frame boundary");
    fx.bus.write32(base + 0x40, 0);
    fx.bus.write32(icc + 0x10, 204);
    ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
    fx.kermit.tick(1001000ull * 333);
    ZLB_EXPECT_EQ(fx.current(), 1u);  // one arm, not continuous unarmed swapping
    ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
}

ZLB_TEST(iftu_frame_events_do_not_depend_on_pixels_and_coalesce_per_native_rearm) {
    IftuFixture fx;
    constexpr u32 base = kermit::kIftu0Base;
    fx.enable(0);  // both descriptors are empty: timing still exists
    fx.enable(1);
    fx.enable_irq(204);
    fx.enable_irq(205);
    fx.dsi_start();
    fx.kermit.tick(1001000ull * 333);  // sixty frames, only one armed turnover
    ZLB_EXPECT_EQ(fx.current(0), 1u);
    ZLB_EXPECT_EQ(fx.current(1), 1u);
    constexpr u32 icc = kermit::kScuBase + kermit::kIccOffset;
    ZLB_EXPECT_EQ(fx.bus.read32(icc + 0xC) & 0x3FF, 204u);
    fx.bus.write32(base + 0x40, 0);
    fx.bus.write32(icc + 0x10, 204);
    ZLB_EXPECT_EQ(fx.bus.read32(icc + 0xC) & 0x3FF, 205u);
    fx.bus.write32(base + 0x1040, 0);
    fx.bus.write32(icc + 0x10, 205);
    ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
    fx.frame();
    ZLB_EXPECT_EQ(fx.current(0), 1u);
    fx.bus.write32(base + 0x180, 1);  // genuine ISR order: ACK, then rearm
    ZLB_EXPECT_EQ(fx.current(0), 1u);
    ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
    fx.frame();
    ZLB_EXPECT_EQ(fx.current(0), 0u);
    ZLB_EXPECT_EQ(fx.current(1), 1u);  // B was not rearmed
    ZLB_EXPECT_TRUE(kermit_irq_line(fx.bus));
    int width = 1, height = 1, stride = 1;
    ZLB_EXPECT_TRUE(fx.kermit.framebuffer(width, height, stride) == nullptr);
}

ZLB_TEST(iftu_partial_control_lanes_cannot_ack_or_rearm_and_stop_reset_cancel) {
    IftuFixture fx;
    constexpr u32 base = kermit::kIftu0Base;
    fx.enable(0);
    fx.enable_irq(204);
    fx.dsi_start();
    fx.frame();
    ZLB_EXPECT_TRUE(kermit_irq_line(fx.bus));
    for (unsigned lane = 0; lane < 4; ++lane) fx.bus.write8(base + 0x40 + lane, 0);
    fx.bus.write16(base + 0x40, 0);
    fx.bus.write16(base + 0x42, 0);
    fx.bus.write32(base + 0x40, 1);
    ZLB_EXPECT_TRUE(kermit_irq_line(fx.bus));
    ZLB_EXPECT_EQ(fx.bus.read32(base + 0x40), 0u);
    Device* iftu = fx.bus.find_device(base);
    ZLB_EXPECT_TRUE(iftu != nullptr);
    if (iftu) ZLB_EXPECT_TRUE(iftu->poke_register("A.REG_040", 0));
    ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
    fx.bus.write8(base + 0x180, 1);
    fx.bus.write16(base + 0x180, 1);
    fx.bus.write16(base + 0x182, 0);
    fx.frame();
    ZLB_EXPECT_EQ(fx.current(), 1u);
    ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
    fx.bus.write32(base + 0x180, 1);
    fx.bus.write32(base + 0x2004, 6);  // mode11 loss cancels the future arm
    fx.bus.write32(base + 0x2004, 2);
    fx.frame();
    ZLB_EXPECT_EQ(fx.current(), 1u);
    fx.bus.write32(base + 0x180, 1);
    fx.bus.write8(base + 0x50, 0);  // real low-lane stop cancels
    fx.bus.write8(base + 0x50, 1);
    fx.frame();
    ZLB_EXPECT_EQ(fx.current(), 1u);
    if (iftu) ZLB_EXPECT_TRUE(iftu->poke_register("A.REG_180", 1));
    fx.frame();
    ZLB_EXPECT_EQ(fx.current(), 0u);
    ZLB_EXPECT_TRUE(kermit_irq_line(fx.bus));
    fx.bus.write32(base + 0x2000, 0);
    ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
    fx.bus.write32(base + 0x2000, 1);
    fx.frame();
    ZLB_EXPECT_EQ(fx.current(), 0u);
    fx.kermit.reset();
    fx.dsi_start();
    fx.frame();
    ZLB_EXPECT_EQ(fx.current(), 0u);
    ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
}

ZLB_TEST(iftu_manual_selection_and_unknown_profiles_do_not_generate_frame_events) {
    IftuFixture fx;
    constexpr u32 base = kermit::kIftu0Base;
    constexpr u32 guest = kermit::kScratchpadSramBase;
    fx.bank(0, 0, guest, 4, 2);
    fx.bank(0, 1, guest + 0x100, 4, 2);
    fx.bus.write8(guest + 0x100, 0x6D);
    fx.enable(0);
    fx.enable_irq(204);
    fx.dsi_start();
    for (u32 mode : {0u, 4u, 6u}) {
        fx.bus.write32(base + 0x2004, mode);  // manual00, unknown10/11
        fx.bus.write32(base + 0x180, 1);
        fx.bus.write8(base + 0x2010, 1);
        fx.frame();
        ZLB_EXPECT_EQ(fx.current(), 1u);
        ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
    }
    fx.bus.write32(base + 0x2004, 0);
    fx.bus.write32(base + 4, 0xFFFF0000);  // hardware current bit cannot be forced
    ZLB_EXPECT_EQ(fx.current(), 1u);
    int width = 0, height = 0, stride = 0;
    const u8* pixels = fx.kermit.framebuffer(width, height, stride);
    if (pixels) ZLB_EXPECT_EQ(pixels[0], 0x6D);
    else ZLB_FAIL("manual selection lost its independent active bank");
    fx.bus.write32(base + 0x2004, 2);
    fx.bus.write32(base + 0x58, 0x100);  // unknown setup is not ordinary
    fx.bus.write32(base + 0x180, 1);
    fx.frame();
    ZLB_EXPECT_EQ(fx.current(), 1u);
    ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
}

ZLB_TEST(iftu_physical_irq_and_unchanged_native_handler_ack_rearm_and_replay_old_bank) {
    // This test executes the actual handler and descriptor writer, after a
    // real board/GIC event. Only imported synchronization wrappers are local
    // no-op fixture returns; this is not a full OS interrupt-vector test.
    // Supplied lowio.elf text PT_LOAD file+A0, linked810058DC..59EF.
    // Only the unchanged descriptor writer and IRQ handler code is embedded.
    // SHA256 a6b196d03dc1df2ce6b06b3dd993f815acfc040288c982022ea9c1777760e21a
    static constexpr u8 native_code[] = {
        0x13, 0x68, 0x2d, 0xe9, 0xf0, 0x0f, 0xf3, 0xb9, 0xd0, 0xf8, 0xdc, 0x41, 0x01, 0x22, 0xca, 0x64,
        0x4f, 0xf4, 0x80, 0x32, 0x14, 0xf4, 0xf8, 0x3f, 0x14, 0xbf, 0x4f, 0xf4, 0x80, 0x54, 0x10, 0x24,
        0x0c, 0x64, 0x00, 0x23, 0xd0, 0xf8, 0xd4, 0x41, 0xd0, 0xf8, 0xd8, 0x01, 0x4c, 0x64, 0x88, 0x64,
        0x0b, 0x66, 0x4b, 0x66, 0x8b, 0x66, 0xcb, 0x66, 0xc1, 0xf8, 0xc0, 0x20, 0xc1, 0xf8, 0xc4, 0x20,
        0xbd, 0xe8, 0xf0, 0x0f, 0x70, 0x47, 0x95, 0x69, 0x4f, 0xf0, 0x00, 0x0a, 0xd4, 0x69, 0xd2, 0xf8,
        0x04, 0x90, 0xd2, 0xf8, 0x08, 0x80, 0xd2, 0xf8, 0x0c, 0xc0, 0x17, 0x69, 0x56, 0x69, 0xd2, 0xf8,
        0x24, 0xb0, 0x10, 0x6a, 0xc1, 0xf8, 0x4c, 0xa0, 0x0b, 0x64, 0xc1, 0xf8, 0x44, 0x90, 0x93, 0x6a,
        0xc1, 0xf8, 0x48, 0x80, 0xc1, 0xf8, 0x54, 0xc0, 0x8f, 0x65, 0x0e, 0x60, 0x4d, 0x60, 0x8c, 0x60,
        0x95, 0x6c, 0x54, 0x6c, 0x08, 0x62, 0xc1, 0xf8, 0x24, 0xb0, 0xd0, 0x6c, 0x8b, 0x62, 0x0c, 0x66,
        0x13, 0x6d, 0x4d, 0x66, 0xd4, 0x6a, 0x15, 0x6b, 0x88, 0x66, 0xcb, 0x66, 0xc1, 0xf8, 0xc0, 0x40,
        0xc1, 0xf8, 0xc4, 0x50, 0x55, 0x6b, 0x94, 0x6b, 0xd0, 0x6b, 0x13, 0x6c, 0xc1, 0xf8, 0xc8, 0x50,
        0xc1, 0xf8, 0xcc, 0x40, 0xc1, 0xf8, 0xd0, 0x00, 0xc1, 0xf8, 0xd4, 0x30, 0xc0, 0xe7, 0x00, 0xbf,
        0xf8, 0xb5, 0x01, 0xf5, 0xfc, 0x74, 0x0f, 0x46, 0x20, 0x46, 0x0e, 0x68, 0xfe, 0xf7, 0x6a, 0xec,
        0xd7, 0xf8, 0xec, 0x21, 0x00, 0x23, 0xd7, 0xf8, 0xe8, 0x11, 0x05, 0x46, 0x30, 0x6c, 0x33, 0x64,
        0x30, 0x6c, 0xc6, 0xf8, 0x80, 0x21, 0x59, 0xb1, 0x01, 0xf0, 0x01, 0x01, 0x38, 0x46, 0x8a, 0x1c,
        0xc7, 0xf8, 0xe8, 0x31, 0x11, 0x02, 0x07, 0xf5, 0xc0, 0x72, 0x71, 0x18, 0xff, 0xf7, 0x80, 0xff,
        0xbf, 0xf3, 0x4f, 0x8f, 0x20, 0x46, 0x29, 0x46, 0xfe, 0xf7, 0x3c, 0xec, 0x4f, 0xf0, 0xff, 0x30,
        0xf8, 0xbd, 0x00, 0xbf,
    };
    static_assert(sizeof(native_code) == 0x114);
    for (unsigned old_bank : {0u, 1u}) {
        IftuFixture fx;
        constexpr u32 base = kermit::kIftu0Base;
        constexpr u32 guest = kermit::kScratchpadSramBase;
        constexpr u32 record = 0x8100B298;
        constexpr u32 stack = 0x8100F000;
        constexpr u32 exit = 0x8100F100;
        fx.bus.add_ram("native Lowio handler fixture", 0x10000, 0x81000000, "genuine handler and writer bytes");
        // Lowio text PT_LOAD is file+A0/linked81000000; copy only unchanged
        // writer58DC..599A and handler599C..59EE, not firmware data or assets.
        fx.bus.write_bytes(0x810058DC, native_code, sizeof(native_code));
        for (u32 imported_sync : {0x81004280u, 0x81004260u}) fx.bus.write32(imported_sync, 0xE12FFF1E);
        fx.bus.write32(record, base);
        fx.bus.write32(record + 4, base + 0x2000);
        fx.bus.write32(record + 0x1E4, 1);
        fx.bus.write32(record + 0x1E8, 0x80000000u | old_bank);
        fx.bus.write32(record + 0x1EC, 1);
        fx.bus.write32(record + 0x180, 0x10);
        fx.bus.write32(record + 0x184, 4);
        fx.bus.write32(record + 0x188, 2);
        fx.bus.write32(record + 0x194, guest);
        fx.bus.write32(record + 0x1AC, 0x10000);
        fx.bus.write32(record + 0x1B0, 0x10000);
        fx.bank(0, old_bank, guest + 0x100, 4, 2);
        fx.bank(0, old_bank ^ 1, guest, 4, 2);
        fx.bus.write8(guest, 0x79);
        fx.enable(0);
        fx.bus.write32(base + 0x2010, old_bank);
        fx.enable_irq(204);
        fx.dsi_start();
        fx.frame();
        ZLB_EXPECT_EQ(fx.current(), old_bank ^ 1);
        ZLB_EXPECT_TRUE(kermit_irq_line(fx.bus));
        constexpr u32 icc = kermit::kScuBase + kermit::kIccOffset;
        ZLB_EXPECT_EQ(fx.bus.read32(icc + 0xC) & 0x3FF, 204u);
        ArmCore cpu(fx.bus);
        cpu.reset(0x8100599D);
        cpu.r[0] = 204;
        cpu.r[1] = record;
        cpu.r[13] = stack;
        cpu.r[14] = exit | 1;
        for (unsigned count = 0; count < 200 && cpu.get_pc() != exit; ++count) {
            cpu.step();
            if (cpu.halted || cpu.undefined_instruction) break;
        }
        ZLB_EXPECT_EQ(cpu.get_pc(), exit);
        ZLB_EXPECT_EQ(cpu.r[13], stack);
        ZLB_EXPECT_EQ(cpu.r[0], 0xFFFFFFFFu);
        ZLB_EXPECT_EQ(fx.bus.read32(record + 0x1E8), 0u);  // guest handler cleared it
        ZLB_EXPECT_EQ(fx.bus.read32(base + 0x200), guest);  // guest writer replayed it
        ZLB_EXPECT_EQ(fx.bus.read32(base + 0x300), guest);
        ZLB_EXPECT_EQ(fx.current(), old_bank ^ 1);  // rearm did not flip immediately
        fx.bus.write32(icc + 0x10, 204);
        ZLB_EXPECT_FALSE(kermit_irq_line(fx.bus));
        fx.frame();
        ZLB_EXPECT_EQ(fx.current(), old_bank);  // next actual frame consumes rearm
        ZLB_EXPECT_TRUE(kermit_irq_line(fx.bus));
        int width = 0, height = 0, stride = 0;
        const u8* pixels = fx.kermit.framebuffer(width, height, stride);
        if (pixels) ZLB_EXPECT_EQ(pixels[0], 0x79);
        else ZLB_FAIL("native old-bank replay failed to retain guest scanout");
    }
}
