// zeliboba - unit tests for the Kermit (ARM SoC) block.
//
// The tests build a Bus, install the block on it and drive the devices through
// their memory mapped registers, exactly like the kernel boot loader does. They
// deliberately use the real EmmcCard (attached to a throw away image) so that
// the SDIF data path is exercised end to end.
#include <cstdio>
#include <memory>
#include <string>

#include "bus/bus.h"
#include "common/log.h"
#include "hw/emmc.h"
#include "hw/soc.h"
#include "hw/soc/soc_internal.h"
#include "test_framework.h"

using namespace zlb;

namespace {

/// One scratch machine per test: the bus, the eMMC image and the block.
struct Fixture {
    std::unique_ptr<Bus> bus_owner = std::make_unique<Bus>();
    Bus& bus = *bus_owner;
    EmmcCard card;
    KermitBlock kermit;
    std::string image;

    explicit Fixture(const char* image_name) : kermit(bus, &card), image(image_name) {
        bus.unmapped_reads_zero = true;
        bus.add_ram("dram", 0x100000, kermit::kDramBase, "soc test dram");
        bus.add_ram("sram", kermit::kSramSize, kermit::kSramBase, "soc test sram");
        std::remove(image_name);
        card.attach(image_name, true, 8ull * MB);
        kermit.install();
        kermit.reset();
    }

    ~Fixture() {
        card.detach();
        std::remove(image.c_str());
    }

    u32 gic(u32 offset) const { return kermit::kScuBase + kermit::kGicDistOffset + offset; }
    u32 icc(u32 offset) const { return kermit::kScuBase + kermit::kIccOffset + offset; }

    /// Enable an interrupt line through the distributor's set-enable register.
    void enable_irq(u32 id) {
        bus.write32(gic(0x100 + (id / 32) * 4), 1u << (id % 32));
        bus.write32(gic(0x400 + id / 4), 0xA0A0A0A0u);
    }

    /// Power the interrupt controller up the way a driver would.
    void gic_start() {
        bus.write32(gic(0x000), 1);
        bus.write32(icc(0x004), 0xFF);
        bus.write32(icc(0x000), 1);
    }

    /// Acknowledge and EOI every pending interrupt (driver style drain).
    void drain_irqs() {
        for (int i = 0; i < 64 && kermit.pending_irq_count() != 0; ++i) {
            const u32 id = bus.read32(icc(0x00C)) & 0x3FF;
            if (id == 1023) break;
            bus.write32(icc(0x010), id);
        }
    }
};

}  // namespace

ZLB_TEST(kermit_installs_every_device) {
    Fixture fx("build/soc_test_install.img");
    const std::vector<Device*> devices = fx.kermit.devices();
    ZLB_EXPECT_TRUE(devices.size() >= 10);
    bool have_gic = false;
    bool have_uart = false;
    bool have_sdif = false;
    bool have_dma = false;
    bool have_display = false;
    for (Device* device : devices) {
        if (device->name() == "Kermit.GIC") have_gic = true;
        if (device->name() == "Kermit.Uart") have_uart = true;
        if (device->name() == "Kermit.Sdif0") have_sdif = true;
        if (device->name() == "Kermit.DMA") have_dma = true;
        if (device->name() == "Kermit.Display") have_display = true;
    }
    ZLB_EXPECT_TRUE(have_gic);
    ZLB_EXPECT_TRUE(have_uart);
    ZLB_EXPECT_TRUE(have_sdif);
    ZLB_EXPECT_TRUE(have_dma);
    ZLB_EXPECT_TRUE(have_display);

    // Every device must be reachable through the bus.
    ZLB_EXPECT_TRUE(fx.bus.find_device(kermit::kUartBase) != nullptr);
    ZLB_EXPECT_TRUE(fx.bus.find_device(kermit::kSdif0Base) != nullptr);
    ZLB_EXPECT_TRUE(fx.bus.find_device(kermit::kDmaBase) != nullptr);
    ZLB_EXPECT_TRUE(fx.bus.find_device(kermit::kDisplayBase) != nullptr);
    ZLB_EXPECT_TRUE(fx.bus.find_device(kermit::kScuBase + kermit::kIccOffset) != nullptr);

    // KermitBlock::describe()/summary() must not crash and must say something.
    std::vector<std::string> lines;
    fx.kermit.describe(lines);
    ZLB_EXPECT_TRUE(lines.size() > 6);
    ZLB_EXPECT_TRUE(!fx.kermit.summary().empty());
}

ZLB_TEST(kermit_gic_acknowledge_and_eoi) {
    Fixture fx("build/soc_test_gic.img");
    const u32 spi = static_cast<u32>(kermit::Irq::Emmc);
    fx.gic_start();
    fx.enable_irq(spi);

    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 0u);
    fx.kermit.pulse_irq(spi, 64);
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 1u);
    ZLB_EXPECT_TRUE(kermit_irq_line(fx.bus));

    const u32 id = fx.bus.read32(fx.icc(0x00C)) & 0x3FF;
    std::fprintf(stderr, "  gic test: id=%u line=%d pending=%u\n", id, kermit_irq_line(fx.bus) ? 1 : 0,
                 fx.kermit.pending_irq_count());
    ZLB_EXPECT_EQ(id, spi);
    ZLB_EXPECT_TRUE(!kermit_irq_line(fx.bus));

    fx.bus.write32(fx.icc(0x010), spi);
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 0u);
    ZLB_EXPECT_TRUE(!kermit_irq_line(fx.bus));

    // Draining an idle controller must report the spurious id.
    ZLB_EXPECT_EQ(fx.bus.read32(fx.icc(0x00C)) & 0x3FF, 1023u);
}

ZLB_TEST(kermit_gic_level_and_edge_triggering) {
    Fixture fx("build/soc_test_gic2.img");
    const u32 spi = static_cast<u32>(kermit::Irq::Emmc);
    fx.gic_start();
    fx.enable_irq(spi);

    // A level sensitive line is pending while it is asserted. Once the interrupt
    // is acknowledged (active) it stays hidden until the EOI, and dropping the
    // line clears it.
    const u32 config_word = fx.gic(0xC20) + ((spi - 32) / 16) * 4;
    const u32 config_shift = ((spi - 32) % 16) * 2;
    fx.bus.write32(config_word, fx.bus.read32(config_word) & ~(3u << config_shift));
    fx.kermit.raise_irq(spi, true);
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 1u);
    ZLB_EXPECT_EQ(fx.bus.read32(fx.icc(0x00C)) & 0x3FF, spi);
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 0u);
    // Re-asserting a line that is already sampled high must not latch a new
    // interrupt while the first one is still active.
    fx.kermit.raise_irq(spi, true);
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 0u);
    fx.kermit.raise_irq(spi, false);
    fx.bus.write32(fx.icc(0x010), spi);
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 0u);

    // An edge sensitive line latches once and stays gone after the EOI.
    fx.bus.write32(config_word, fx.bus.read32(config_word) | (2u << config_shift));
    fx.kermit.raise_irq(spi, true);
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 1u);
    fx.bus.write32(fx.icc(0x010), fx.bus.read32(fx.icc(0x00C)) & 0x3FF);
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 0u);
    fx.kermit.raise_irq(spi, false);
}

ZLB_TEST(kermit_gic_priority_masks_low_priority_lines) {
    Fixture fx("build/soc_test_gic3.img");
    const u32 spi = static_cast<u32>(kermit::Irq::Emmc);
    fx.gic_start();
    fx.enable_irq(spi);
    // A priority mask below the interrupt's priority must hide it. The
    // distributor's priority registers are byte addressable; the model feeds
    // every interrupt in the word from the same value.
    const u32 priority_word = fx.gic(0x400) + spi / 4;
    fx.bus.write32(priority_word, 0x000000F0u);
    fx.bus.write32(fx.icc(0x004), 0x80);  // mask every priority >= 0x80
    fx.kermit.pulse_irq(spi, 64);
    ZLB_EXPECT_TRUE(!kermit_irq_line(fx.bus));
    // Let the pulse expire, then open the mask and pulse again: now the CPU
    // must see it.
    fx.kermit.tick(1000 * 333);
    fx.bus.write32(fx.icc(0x004), 0xFF);
    fx.kermit.pulse_irq(spi, 64);
    ZLB_EXPECT_TRUE(kermit_irq_line(fx.bus));
    ZLB_EXPECT_EQ(fx.bus.read32(fx.icc(0x00C)) & 0x3FF, spi);
}

ZLB_TEST(kermit_global_timer_counts_microseconds) {
    Fixture fx("build/soc_test_gt.img");
    const u32 control = kermit::kScuBase + kermit::kGlobalTimerOffset + 0x08;
    const u32 counter_lo = kermit::kScuBase + kermit::kGlobalTimerOffset + 0x00;
    const u32 comparator_lo = kermit::kScuBase + kermit::kGlobalTimerOffset + 0x10;
    const u32 status = kermit::kScuBase + kermit::kGlobalTimerOffset + 0x0C;

    ZLB_EXPECT_EQ(fx.bus.read32(counter_lo), 0u);
    fx.bus.write32(control, 1);
    fx.bus.write32(comparator_lo, 1000);
    fx.kermit.tick(1000000);  // 1e6 CPU cycles == 3003 PERIPHCLK ticks
    ZLB_EXPECT_EQ(fx.bus.read32(counter_lo), 3003u);
    ZLB_EXPECT_EQ(fx.bus.read32(status) & 1u, 1u);

    // The counter is free running: clearing the control bit stops the
    // comparator/interrupt logic, not the count.  The old test pinned the opposite
    // (frozen while disabled), which made GTCNT stand still for a guest that reads
    // it without programming the timer - and left the model with no timer that could
    // ever be a WFE wake-up source (docs/KBL.md 7.1.40).
    fx.bus.write32(control, 0);
    const u32 before = fx.bus.read32(counter_lo);
    fx.kermit.tick(1000000);
    ZLB_EXPECT_EQ(fx.bus.read32(counter_lo), before + 3003u);

    // Reset returns it to zero.
    fx.kermit.reset();
    ZLB_EXPECT_EQ(fx.bus.read32(counter_lo), 0u);
}

// The global timer is the only interrupt source a core in WFE can be woken by in
// the boot chain (the kernel boot loader never programs the MPCore timers), so the
// whole path "comparator -> distributor -> cpu interface -> core IRQ line" is pinned
// here: assert, stay asserted while the event bit is set, and drop on write-1-to-clear.
ZLB_TEST(kermit_global_timer_reaches_the_core_irq_line) {
    Fixture fx("build/soc_test_gt_irq.img");
    const u32 base = kermit::kScuBase + kermit::kGlobalTimerOffset;
    const u32 control = base + 0x08;
    const u32 status = base + 0x0C;
    const u32 comparator_lo = base + 0x10;

    fx.gic_start();
    fx.enable_irq(kermit::kIrqPpiGlobalTimer);
    ZLB_EXPECT_TRUE(!kermit_irq_line(fx.bus));

    fx.bus.write32(control, 1);
    fx.bus.write32(comparator_lo, 1000);
    fx.kermit.tick(1000000);   // 3003 PERIPHCLK ticks, past the comparator
    ZLB_EXPECT_EQ(fx.bus.read32(status) & 1u, 1u);
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 1u);
    ZLB_EXPECT_TRUE(kermit_irq_line(fx.bus));

    // The event is a crossing, not the level `counter >= comparator`: clearing the
    // status register must stick (the old level-derived check re-asserted it on the
    // next evaluation, so the interrupt could never be acknowledged).
    fx.bus.write32(status, 1);
    ZLB_EXPECT_EQ(fx.bus.read32(status) & 1u, 0u);
    fx.kermit.tick(1000000);
    ZLB_EXPECT_EQ(fx.bus.read32(status) & 1u, 0u);

    // Acknowledge and EOI the interrupt the crossing latched: now the line drops,
    // because the source is no longer asserting.
    fx.drain_irqs();
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 0u);
    ZLB_EXPECT_TRUE(!kermit_irq_line(fx.bus));
}

ZLB_TEST(kermit_private_timer_and_watchdog) {
    Fixture fx("build/soc_test_pt.img");
    const u32 base = kermit::kScuBase + kermit::kPrivateTimerOffset;
    const u32 control = base + 0x08;
    const u32 status = base + 0x0C;
    const u32 counter = base + 0x04;
    const u32 watchdog_control = base + 0x28;
    const u32 watchdog_counter = base + 0x24;

    fx.bus.write32(base + 0x00, 500);  // load
    fx.bus.write32(control, 0x7);      // enable | auto reload | irq enable
    fx.kermit.tick(100000);            // 300 us
    ZLB_EXPECT_EQ(fx.bus.read32(counter), 200u);
    ZLB_EXPECT_EQ(fx.bus.read32(status) & 1u, 0u);
    fx.kermit.tick(100000);
    ZLB_EXPECT_EQ(fx.bus.read32(status) & 1u, 1u);
    fx.bus.write32(status, 1);
    ZLB_EXPECT_EQ(fx.bus.read32(status) & 1u, 0u);

    // The watchdog is the second counter in the same block.
    fx.bus.write32(base + 0x20, 100);
    fx.bus.write32(watchdog_control, 0x1);
    fx.kermit.tick(100000);
    ZLB_EXPECT_TRUE(fx.bus.read32(watchdog_counter) <= 100u);
    fx.bus.write32(watchdog_control, 0);
}

ZLB_TEST(kermit_uart_captures_output_and_raises_rx_irq) {
    Fixture fx("build/soc_test_uart.img");
    const u32 spi = static_cast<u32>(kermit::Irq::Uart0);
    fx.gic_start();
    fx.enable_irq(spi);

    const char* text = "hello kermit\n";
    for (const char* p = text; *p != '\0'; ++p) fx.bus.write8(kermit::kUartBase, static_cast<u8>(*p));
    ZLB_EXPECT_TRUE(fx.kermit.uart_has_output());
    ZLB_EXPECT_TRUE(fx.kermit.take_uart_output() == text);
    ZLB_EXPECT_TRUE(!fx.kermit.uart_has_output());

    // Receive: inject a byte and check the interrupt and the data register.
    // The UART line is level sensitive, so configure it that way (the reset
    // default is edge triggered for every interrupt).
    const u32 config_word = fx.gic(0xC20) + ((spi - 32) / 16) * 4;
    const u32 config_shift = ((spi - 32) % 16) * 2;
    fx.bus.write32(config_word, fx.bus.read32(config_word) & ~(3u << config_shift));
    fx.bus.write32(kermit::kUartBase + 0x04, 0x01);
    fx.kermit.uart_write("xy");
    std::fprintf(stderr, "  uart test: pending=%u line=%d iir=0x%02X lsr=0x%02X\n", fx.kermit.pending_irq_count(),
                 kermit_irq_line(fx.bus) ? 1 : 0, static_cast<unsigned>(fx.bus.read32(kermit::kUartBase + 0x08) & 0xFF),
                 static_cast<unsigned>(fx.bus.read32(kermit::kUartBase + 0x14) & 0xFF));
    ZLB_EXPECT_EQ(fx.bus.read32(kermit::kUartBase + 0x14) & 1u, 1u);
    ZLB_EXPECT_EQ(fx.bus.read32(kermit::kUartBase + 0x08) & 0x0F, 0x04u);
    ZLB_EXPECT_TRUE(kermit_irq_line(fx.bus));
    ZLB_EXPECT_EQ(fx.bus.read32(fx.icc(0x00C)) & 0x3FF, spi);
    ZLB_EXPECT_EQ(fx.bus.read8(kermit::kUartBase), static_cast<u8>('x'));
    ZLB_EXPECT_EQ(fx.bus.read8(kermit::kUartBase), static_cast<u8>('y'));

    fx.bus.write32(fx.icc(0x010), spi);
    fx.kermit.reset();
    ZLB_EXPECT_TRUE(!fx.kermit.uart_has_output());
}

ZLB_TEST(kermit_dma_copies_through_the_bus) {
    Fixture fx("build/soc_test_dma.img");
    const u32 spi = static_cast<u32>(kermit::Irq::Dma0);
    fx.gic_start();
    fx.enable_irq(spi);

    const u32 source = kermit::kDramBase + 0x1000;
    const u32 dest = kermit::kDramBase + 0x2000;
    for (u32 i = 0; i < 64; ++i) fx.bus.write8(source + i, static_cast<u8>(i + 1));
    fx.bus.memset_bytes(dest, 0, 64);

    fx.bus.write32(kermit::kDmaBase + 0x100 + 0x00, source);
    fx.bus.write32(kermit::kDmaBase + 0x100 + 0x04, dest);
    fx.bus.write32(kermit::kDmaBase + 0x100 + 0x08, 64);
    fx.bus.write32(kermit::kDmaBase + 0x100 + 0x0C, 0x01 | 0x04 | 0x100);  // enable | irq | start
    ZLB_EXPECT_EQ(fx.bus.read32(kermit::kDmaBase + 0x100 + 0x10) & 1u, 1u);

    for (int i = 0; i < 16; ++i) fx.kermit.tick(333000);
    bool copied = true;
    for (u32 i = 0; i < 64; ++i) {
        if (fx.bus.read8(dest + i) != static_cast<u8>(i + 1)) copied = false;
    }
    ZLB_EXPECT_TRUE(copied);
    ZLB_EXPECT_EQ(fx.bus.read32(kermit::kDmaBase + 0x100 + 0x10) & 2u, 2u);

    // A transfer to unmapped memory must report the error instead of hanging.
    fx.bus.write32(kermit::kDmaBase + 0x120 + 0x00, source);
    fx.bus.write32(kermit::kDmaBase + 0x120 + 0x04, 0xF0000000u);
    fx.bus.write32(kermit::kDmaBase + 0x120 + 0x08, 16);
    fx.bus.write32(kermit::kDmaBase + 0x120 + 0x0C, 0x01 | 0x100);
    fx.kermit.tick(333000);
    ZLB_EXPECT_EQ(fx.bus.read32(kermit::kDmaBase + 0x120 + 0x10) & 4u, 4u);
}

ZLB_TEST(kermit_display_framebuffer_and_flip) {
    Fixture fx("build/soc_test_display.img");
    int width = -1;
    int height = -1;
    int stride = -1;
    ZLB_EXPECT_TRUE(fx.kermit.framebuffer(width, height, stride) == nullptr);
    ZLB_EXPECT_EQ(fx.kermit.frame_counter(), 0u);

    fx.bus.write32(kermit::kDisplayBase + 0x08, kermit::kDramBase + 0x10000);
    fx.bus.write32(kermit::kDisplayBase + 0x10, 960 * 2);
    fx.bus.write32(kermit::kDisplayBase + 0x14, (544u << 16) | 960u);
    fx.bus.write32(kermit::kDisplayBase + 0x18, 0);
    fx.bus.write32(kermit::kDisplayBase + 0x00, 0x05);  // enable | vsync

    const u8* pixels = fx.kermit.framebuffer(width, height, stride);
    ZLB_EXPECT_TRUE(pixels != nullptr);
    ZLB_EXPECT_EQ(width, 960);
    ZLB_EXPECT_EQ(height, 544);
    ZLB_EXPECT_EQ(stride, 960 * 2);

    u8* writable = const_cast<u8*>(pixels);
    writable[0] = 0x1F;
    writable[1] = 0x00;  // blue in RGB565
    fx.kermit.tick(5550111 * 3);
    ZLB_EXPECT_TRUE(fx.kermit.frame_counter() >= 3);
    pixels = fx.kermit.framebuffer(width, height, stride);
    ZLB_EXPECT_EQ(pixels[0], 0x1F);
    ZLB_EXPECT_EQ(pixels[1], 0x00);

    // Powering the display down hides the buffer again.
    fx.bus.write32(kermit::kDisplayBase + 0x00, 0);
    ZLB_EXPECT_TRUE(fx.kermit.framebuffer(width, height, stride) == nullptr);
}

ZLB_TEST(kermit_sdif_programs_a_block_read) {
    Fixture fx("build/soc_test_sdif.img");
    const u32 base = kermit::kSdif0Base;
    fx.bus.write16(base + 0x04, 512);  // block size
    fx.bus.write16(base + 0x06, 1);    // block count
    fx.bus.write32(base + 0x08, 0);    // argument
    fx.bus.write16(base + 0x0C, 0x10); // transfer mode: read
    fx.bus.write16(base + 0x0E, (17u << 8) | 0x02 | 0x10 | 0x20);

    // The command was decoded as CMD17 with one block at LBA 0, whatever the
    // card made of the (empty) scratch image.
    ZLB_EXPECT_EQ(fx.kermit.last_emmc_count(), 1u);
    ZLB_EXPECT_EQ(fx.kermit.last_emmc_lba(), 0u);

    // The data port is readable and the interrupt logic runs.
    fx.bus.read8(base + 0x20);
    ZLB_EXPECT_EQ(fx.bus.read32(base + 0x24) & (1u << 16), 1u << 16);  // card inserted

    // A write command goes the other way: 512 bytes pushed through the port.
    fx.bus.write16(base + 0x0C, 0x00);
    fx.bus.write16(base + 0x0E, (24u << 8) | 0x02 | 0x10 | 0x20);
    for (u32 i = 0; i < 512; ++i) fx.bus.write8(base + 0x20, static_cast<u8>(i));
    ZLB_EXPECT_EQ(fx.kermit.last_emmc_count(), 1u);

    // Reset clears the statistics.
    fx.kermit.reset();
    ZLB_EXPECT_EQ(fx.kermit.emmc_transfers(), 0u);
    ZLB_EXPECT_EQ(fx.kermit.last_emmc_count(), 0u);
}

ZLB_TEST(kermit_reset_powers_every_device_down) {
    Fixture fx("build/soc_test_reset.img");
    const u32 spi = static_cast<u32>(kermit::Irq::Emmc);
    fx.gic_start();
    fx.enable_irq(spi);
    fx.kermit.pulse_irq(spi, 64);
    for (const char* p = "abc"; *p != '\0'; ++p) fx.bus.write8(kermit::kUartBase, static_cast<u8>(*p));
    fx.bus.write32(kermit::kDisplayBase + 0x00, 0x05);
    fx.kermit.tick(5550111);

    fx.kermit.reset();
    ZLB_EXPECT_EQ(fx.kermit.pending_irq_count(), 0u);
    ZLB_EXPECT_TRUE(!fx.kermit.uart_has_output());
    ZLB_EXPECT_EQ(fx.kermit.frame_counter(), 0u);
    ZLB_EXPECT_EQ(fx.kermit.emmc_transfers(), 0u);
    ZLB_EXPECT_EQ(fx.bus.read32(kermit::kScuBase + kermit::kGlobalTimerOffset + 0x00), 0u);
    ZLB_EXPECT_EQ(fx.bus.read32(fx.icc(0x000)) & 1u, 0u);
    ZLB_EXPECT_TRUE(!kermit_irq_line(fx.bus));
}

ZLB_TEST(kermit_register_names_are_exposed) {
    Fixture fx("build/soc_test_names.img");
    // The debugger relies on named registers: every register a device reports
    // through enumerate_registers() has to carry a non-empty name, and a device
    // that can name its registers must accept a named poke/peek.
    for (Device* device : fx.kermit.devices()) {
        std::vector<RegisterInfo> regs;
        device->enumerate_registers(regs);
        if (!regs.empty() && device->register_name(regs.front().address) == nullptr) {
            std::printf("  unnamed: %s @ 0x%08X (first reg 0x%08X %s)\n", device->name().c_str(), device->base(),
                        regs.front().address, regs.front().name.c_str());
        }
        ZLB_EXPECT_TRUE(regs.empty() || device->register_name(regs.front().address) != nullptr);
        for (const RegisterInfo& info : regs) ZLB_EXPECT_TRUE(!info.name.empty());
    }

    // A named access round trips through poke/peek.
    Device* gic = fx.bus.find_device(fx.gic(0x000));
    ZLB_EXPECT_TRUE(gic != nullptr);
    if (gic != nullptr) {
        u64 value = 0;
        ZLB_EXPECT_TRUE(gic->peek_register("ICDDCR", value));
        ZLB_EXPECT_TRUE(gic->poke_register("ICDDCR", 1));
        ZLB_EXPECT_TRUE(gic->peek_register("ICDDCR", value));
        ZLB_EXPECT_EQ(value, 1u);
        // ... and the same register is readable by address.
        ZLB_EXPECT_EQ(fx.bus.read32(fx.gic(0x000)), 1u);
    }

    Device* uart = fx.bus.find_device(kermit::kUartBase);
    ZLB_EXPECT_TRUE(uart != nullptr);
    if (uart != nullptr) {
        u64 value = 0;
        ZLB_EXPECT_TRUE(uart->peek_register("UART_LCR", value));
        ZLB_EXPECT_EQ(value, 0x03u);
    }
}

ZLB_TEST(kermit_spi_syscon_link_round_trip) {
    // SPI0 is the syscon link the boot chain drives. This test walks the exact
    // register sequence the CMeP second loader uses (0x436E4): drain the receive
    // FIFO, push the request words, clear the interrupt status, start the
    // transfer, then read the reply through 0x28/0x00.
    Fixture f("spi.img");

    kermit::Spi* spi = nullptr;
    for (const auto& device : f.bus.devices()) {
        if (device->name() == "Kermit.Spi0") spi = static_cast<kermit::Spi*>(device.get());
    }
    ZLB_EXPECT_TRUE(spi != nullptr);
    if (spi == nullptr) return;

    // A slave that echoes the request with one extra byte, so the data path is
    // observable without depending on Ernie.
    spi->set_slave([](const std::vector<u8>& request) {
        std::vector<u8> reply = request;
        reply.push_back(0x5A);
        return reply;
    });

    const u32 base = f.bus.find_device(0xE0A00000) ? 0xE0A00000u : 0u;
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x28), 0u);   // RX FIFO empty
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x2C), 0u);   // TX FIFO empty

    // The request words are written low byte first, so the four byte frame
    // `01 00 01 FD` (CMD 0x0001, the second loader's first packet) is two
    // 16 bit writes.
    f.bus.write32(base + 0x04, 0x00000001u);        // bytes 01 00
    f.bus.write32(base + 0x04, 0x0000FD01u);        // bytes 01 FD
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x2C), 4u);
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x10) & 1u, 0u);  // idle before the start
    f.bus.write32(base + 0x24, 0x600u);             // clear interrupt status
    f.bus.write32(base + 0x10, 1u);                 // start

    // 4 + 1 = 5 bytes came back; the receive count is in bytes and the FIFO
    // hands out 16 bit words.
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x28), 5u);
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x00), 0x00000001u);
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x28), 3u);
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x00), 0x0000FD01u);
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x28), 1u);
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x00) & 0xFFu, 0x5Au);
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x28), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x00), 0u);

    ZLB_EXPECT_TRUE((f.bus.read32(base + 0x24) & 0x200u) != 0u);
    f.bus.write32(base + 0x24, 0x200u);             // write one to clear
    ZLB_EXPECT_EQ(f.bus.read32(base + 0x24) & 0x200u, 0u);

    ZLB_EXPECT_EQ(spi->transfers(), 1u);
    ZLB_EXPECT_EQ(spi->bytes_tx(), 4u);
    ZLB_EXPECT_EQ(spi->bytes_rx(), 5u);
    ZLB_EXPECT_EQ(spi->last_request().size(), size_t(4));
    ZLB_EXPECT_EQ(spi->last_response().size(), size_t(5));

    const char* name = f.bus.find_device(base + 0x28)->register_name(base + 0x28);
    ZLB_EXPECT_TRUE(name != nullptr && std::string(name) == "SPI_RXFIFO_STATUS");
}

ZLB_TEST(kermit_spi_without_a_slave_still_completes) {
    // A driver whose poll loop waits for the transfer to finish must not spin
    // forever just because nothing is attached to the link.
    Fixture f("spi-empty.img");
    f.bus.write32(0xE0A10000 + 0x04, 0x00000011u);
    f.bus.write32(0xE0A10000 + 0x10, 1u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0A10000 + 0x10) & 1u, 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0A10000 + 0x28), 0u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0A10000 + 0x2C), 0u);
}

ZLB_TEST(sdif_command_registers_are_decoded_from_the_shared_word) {
    // The command register (0x0E) shares its 32 bit word with the transfer mode
    // register (0x0C) and the block count (0x06) shares one with the block size
    // (0x04), so both are stored under the word offset. A driver that writes the
    // command with a 16 bit access (the CMeP second loader does: `W2 0x0E`) used
    // to have it decoded as GO_IDLE_STATE and its response came back as zero.
    Fixture f("sdif-cmd.img");

    // CMD0 has index 0 and no flags, so the command half is zero: the command
    // still has to run.
    f.bus.write16(0xE0B00008, 0x00000000u);          // argument
    f.bus.write16(0xE0B0000C, 0x0000u);              // transfer mode
    f.bus.write16(0xE0B0000E, 0x0000u);              // command: CMD0
    ZLB_EXPECT_TRUE((f.bus.read16(0xE0B00030) & 0x1u) != 0u);   // command complete

    // CMD1 (SEND_OP_COND): index 1, no data. The response has to come back from
    // the card, not as the GO_IDLE_STATE zero response. Like a real card the
    // first call reports "power up in progress" (OCR bit 31) and the next one
    // reports ready.
    f.bus.write32(0xE0B00030, 0x1u);                 // clear status
    f.bus.write16(0xE0B0000E, 0x0102u);              // command: CMD1
    ZLB_EXPECT_EQ(f.bus.read32(0xE0B00010), 0xC0FF8000u);
    f.bus.write32(0xE0B00030, 0x1u);
    f.bus.write16(0xE0B0000E, 0x0102u);
    ZLB_EXPECT_EQ(f.bus.read32(0xE0B00010), 0x40FF8000u);

    // CMD3 in MMC mode: the RCA travels in the argument and the answer is an R1
    // status, not the RCA (the CMeP loader's error decoder treats bits 13..23 of
    // the response as error bits).
    f.bus.write32(0xE0B00008, 0x00010000u);          // argument: RCA 1
    f.bus.write16(0xE0B0000E, 0x031Au);              // command: CMD3, R1 response
    ZLB_EXPECT_EQ(f.bus.read32(0xE0B00010), 0x00000900u);

    // The block size / block count pair is written as one 32 bit word and read
    // back the same way.
    f.bus.write32(0xE0B00004, 0x00020100u);          // 512 byte blocks, 2 of them
    ZLB_EXPECT_EQ(f.bus.read32(0xE0B00004), 0x00020100u);
}

ZLB_TEST(sdif_buffer_port_pops_a_word_per_word_access) {
    // The buffer data port is 32 bit wide: a 4 byte access has to pop four bytes
    // (and a 1 byte access one). Returning a single byte for every size made the
    // CMeP second loader's PIO loop advance four destination bytes per FIFO byte.
    Fixture f("sdif-fifo.img");

    // SEND_EXT_CSD hands out the card's Extended CSD over the data lines.
    f.bus.write32(0xE0B00004, 0x00000200u);          // block size 512
    f.bus.write16(0xE0B0000C, 0x0010u);              // transfer mode
    f.bus.write16(0xE0B0000E, 0x083Au);              // command: CMD8
    f.bus.write32(0xE0B00030, 0x1u);                 // acknowledge command complete

    // Present State reports "buffer read enable" while data is pending.
    ZLB_EXPECT_TRUE((f.bus.read32(0xE0B00024) & 0x800u) != 0u);

    // Count how many word accesses it takes to drain the card: SEND_EXT_CSD hands
    // out one 512 byte block, so 128 word reads must exhaust it. With the old
    // one-byte-per-access model this took four times as many.
    int reads = 0;
    while ((f.bus.read32(0xE0B00024) & 0x800u) != 0u && reads < 5000) {
        f.bus.read32(0xE0B00020);
        ++reads;
    }
    ZLB_EXPECT_EQ(reads, 128);
    // Transfer Complete is reported a few clocks after the last byte.
    f.kermit.tick(20000);
    ZLB_EXPECT_TRUE((f.bus.read32(0xE0B00030) & 0x2u) != 0u);
}
