// Native Syscon SPI0 / GPIO0 board wire. Service bytes are provided by the
// existing responder; tests never inject a guest success or subinterrupt call.
#include <memory>
#include <vector>

#include "cpu/arm/arm_core.h"
#include "cpu/mep/mep_core.h"
#include "hw/cmep/cmep_internal.h"
#include "hw/soc/soc_internal.h"
#include "test_framework.h"

using namespace zlb;

namespace {
struct SysconWireFixture {
    std::unique_ptr<Bus> arm_owner = std::make_unique<Bus>();
    std::unique_ptr<Bus> cmep_owner = std::make_unique<Bus>();
    std::unique_ptr<Bus> ernie_owner = std::make_unique<Bus>();
    Bus& arm = *arm_owner;
    Bus& cmep = *cmep_owner;
    ErnieBlock ernie{*ernie_owner, nullptr};
    KermitBlock soc{arm, nullptr};
    cmep_detail::GpioDevice* gpio = nullptr;
    kermit::Spi* spi = nullptr;
    static constexpr u32 kSpi = 0xE0A00000;
    static constexpr u32 kGpio = cmep::kGpioBase;

    SysconWireFixture() {
        auto device = std::make_unique<cmep_detail::GpioDevice>(); gpio = device.get();
        cmep.add_device(std::move(device));
        ernie.install(); ernie.reset(); soc.install(); soc.reset();
        soc.attach_syscon_spi(cmep, &ernie);
        arm.add_device(std::make_unique<DeviceMirror>(*gpio, kGpio, gpio->size()));
        spi = static_cast<kermit::Spi*>(arm.find_device(kSpi));
    }
    u32 g(u32 offset) { return arm.read32(kGpio + offset); }
    void g(u32 offset, u32 value) { arm.write32(kGpio + offset, value); }
    u32 s(u32 offset) { return arm.read32(kSpi + offset); }
    void s(u32 offset, u32 value) { arm.write32(kSpi + offset, value); }
    void native() {
        gpio->enter_native_phase();
        g(0, 8); g(0x14, 0x300); g(0x1C, 0); g(0xC, 8);
    }
    void queue(const std::vector<u8>& bytes) {
        for (size_t i = 0; i < bytes.size(); i += 2)
            s(4, bytes[i] | (i + 1 < bytes.size() ? static_cast<u32>(bytes[i + 1]) << 8 : 0));
    }
    void start() { queue(ernie::make_spi_request(5, {})); s(0x10, 1); }
    void release() { g(8, 8); }
    void drain() { while (s(0x28)) s(0); }
    void ack() { for (u32 offset = 0x38; offset <= 0x48; offset += 4) g(offset, 0x10); }
    void next() { s(0x10, 0); g(0xC, 8); drain(); ack(); }
};

void write_hex(Bus& bus, u32 address, const char* hex) {
    auto digit = [](char c) -> u8 { return static_cast<u8>(c <= '9' ? c - '0' : c - 'a' + 10); };
    for (unsigned i = 0; hex[i] && hex[i + 1]; i += 2)
        bus.write8(address + i / 2, static_cast<u8>((digit(hex[i]) << 4) | digit(hex[i + 1])));
}
}

ZLB_TEST(spi_syscon_ready_preserves_unconfigured_polling_and_services_real_secondloader_before_native_phase) {
    SysconWireFixture f;
    ZLB_EXPECT_EQ(f.g(4) & 0x10u, 0u); // no early JIG/ready peer is synthesized
    f.start();
    ZLB_EXPECT_EQ(f.spi->last_request().size(), size_t(4));
    ZLB_EXPECT_EQ(f.spi->last_response().size(), size_t(10));
    ZLB_EXPECT_EQ(f.s(0x28), 10u); // existing unconfigured byte-polling transport unchanged
    f.soc.tick(333000); ZLB_EXPECT_EQ(f.g(4) & 0x10u, 0u);
    ZLB_EXPECT_EQ(f.g(0x38), 0u); ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 0u);
    f.next();
    // Genuine second_loader43896..438C2 configures pin3 output, pin4 input,
    // mode3 and enables pin4 before secure-kernel/native-phase handoff.
    f.g(0, 8); f.g(0x14, 0x300); f.g(0x1C, 0xFFFFFFEF);
    ZLB_EXPECT_FALSE(f.gpio->native_phase());
    ZLB_EXPECT_EQ(f.g(4) & 0x10u, 0u); // configuration alone is not a reply
    f.cmep.add_ram("unchanged MeP GPIO query/ACK", 0x20000, 0x40000, "supplied second_loader bytes plus caller RAM");
    // second_loader.bin49790..49894: the actual43B62 polling callee reads
    // status38..48 against software enables;49806 W1C-ACKs all five latches.
    write_hex(f.cmep, 0x49790, "01541304000321c03f8004c0000167a31f532303000321c03f8004c0010157a3276136610401a4d36105119340001e092e201ec214009ecc38009ec33c009ecb40009eca440021131ec210009ec94800211c1ec21800c013211b1ec21c00b013211a1ec22000a0132119901331100d6006c00100027001541304000321c03f8004c000017fa31f532303000321c03f8004c001016fa3276136610401a4d3610511932e241e0340021ec414003ecc38003ec03c003ecb40003eca440041101ec410003ec94800411cc0101ecc1800c11bb0101ecb1c001ec120003ac23800b11aa01011193ac23c0090103ac240003ac244003ac2480021103ec348000d6006c001000270");
    f.cmep.write32(0x561A4, f.kGpio); f.cmep.write32(0x561A4 + 16, 0x10);
    MePCore polling(f.cmep);
    auto native_call = [&](u32 entry) {
        polling.reset(entry); polling.r[1] = 0; polling.r[2] = 4; polling.lp = 0x4A000;
        for (unsigned n = 0; n < 96 && polling.get_pc() != 0x4A000; ++n)
            ZLB_EXPECT_FALSE(polling.step().faulted);
        ZLB_EXPECT_EQ(polling.get_pc(), 0x4A000u);
        return polling.r[0];
    };
    ZLB_EXPECT_EQ(native_call(0x49790), 0u);
    f.start(); f.soc.tick(333);
    ZLB_EXPECT_EQ(f.g(4) & 0x10u, 0x10u); // real valid frame establishes idle-high before rise
    ZLB_EXPECT_EQ(native_call(0x49790), 0u);
    for (unsigned n = 0; n < 16; ++n) {
        ZLB_EXPECT_EQ(f.g(4) & 0x10u, 0x10u); ZLB_EXPECT_EQ(f.s(0x28), 10u);
    }
    f.release(); f.soc.tick(332);
    ZLB_EXPECT_EQ(native_call(0x49790), 0u); // fractional CPU cycles do not advance wire
    f.soc.tick(1);
    ZLB_EXPECT_EQ(native_call(0x49790), 1u); // genuine second-loader poll sees physical edge
    ZLB_EXPECT_EQ(f.g(4) & 0x10u, 0x10u); ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 1u);
    native_call(0x49806); ZLB_EXPECT_EQ(native_call(0x49790), 0u);
    ZLB_EXPECT_EQ(f.s(0x28), 10u); // genuine GPIO ACK is not an RX drain
    f.next(); f.native(); f.start(); f.release(); f.soc.tick(333);
    ZLB_EXPECT_EQ(f.g(0x38), 0x10u); ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 2u);
    ZLB_EXPECT_EQ(f.spi->transfers(), 3u); ZLB_EXPECT_EQ(f.ernie.spi_transfers(), 3u);
    f.ack(); f.soc.tick(333000);
    ZLB_EXPECT_EQ(f.g(0x38), 0u); ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 2u);
    ZLB_EXPECT_EQ(f.s(0x28), 10u);
}

ZLB_TEST(spi_syscon_ready_stop_drain_output_low_reset_and_control_cancel_delayed_generation) {
    SysconWireFixture f; f.native();
    for (unsigned cancellation = 0; cancellation < 7; ++cancellation) {
        f.start(); f.release();
        if (cancellation == 0) f.s(0x10, 0);
        if (cancellation == 1) f.drain();
        if (cancellation == 2) f.g(0xC, 8);
        if (cancellation == 3) f.spi->reset();
        if (cancellation == 4) f.s(8, 1); // exact CTL0 qualifier lost before scheduled tick
        if (cancellation == 5) f.g(0x1C, 0xFFFFFFFF); // mode3 still live: no artificial low on masking
        if (cancellation == 6) f.g(0x14, 0x200);
        f.soc.tick(333000);
        ZLB_EXPECT_EQ(f.g(0x38), 0u); ZLB_EXPECT_EQ(f.g(4) & 0x10u, 0x10u);
        ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 0u);
        if (cancellation >= 4) {
            if (cancellation == 4) f.s(8, 0);
            if (cancellation == 5) f.g(0x1C, 0);
            if (cancellation == 6) f.g(0x14, 0x300);
            // Restoring a qualifier cannot resurrect canceled frame/rise.
            f.soc.tick(333000);
            ZLB_EXPECT_EQ(f.g(0x38), 0u); ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 0u);
        }
        f.next(); // drain stale response before the next complete generation
    }
    f.start(); f.release(); f.soc.tick(333);
    ZLB_EXPECT_EQ(f.g(0x38), 0x10u); ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 1u);
    f.next(); f.soc.reset(); f.cmep.reset_devices();
    ZLB_EXPECT_FALSE(f.gpio->native_phase()); ZLB_EXPECT_EQ(f.g(4), 0u);
    ZLB_EXPECT_EQ(f.g(0x38), 0u); ZLB_EXPECT_EQ(f.s(0x28), 0u);
    f.soc.tick(333000); ZLB_EXPECT_EQ(f.g(0x38), 0u);
}

ZLB_TEST(spi_syscon_ready_refuses_bad_frames_missing_reply_wrong_mode_and_unassociated_rise) {
    SysconWireFixture f; f.native();
    auto bad = ernie::make_spi_request(5, {}); bad.back() ^= 1;
    f.queue(bad); f.s(0x10, 1); f.release(); f.soc.tick(333);
    ZLB_EXPECT_TRUE(f.s(0x28) != 0); // existing responder error bytes are retained
    ZLB_EXPECT_EQ(f.g(0x38), 0u); f.next();
    f.spi->set_slave([](const std::vector<u8>&) { return std::vector<u8>{}; });
    f.start(); f.release(); f.soc.tick(333); ZLB_EXPECT_EQ(f.g(0x38), 0u); f.next();
    f.spi->set_slave([](const std::vector<u8>&) {
        auto answer = ernie::make_spi_response(4, 0, {}); answer[4] ^= 1; return answer;
    });
    f.start(); f.release(); f.soc.tick(333); ZLB_EXPECT_EQ(f.g(0x38), 0u); f.next();
    f.spi->set_slave([&](const std::vector<u8>& request) { return f.ernie.spi_transfer(request); });
    f.s(8, 0x30001); // port2 OLED control is not SPI0's framed native mode
    f.start(); f.release(); f.soc.tick(333); ZLB_EXPECT_EQ(f.g(0x38), 0u); f.next(); f.s(8, 0);
    f.release(); f.start(); // GPIO3 already high before this generation
    f.soc.tick(333); ZLB_EXPECT_EQ(f.g(0x38), 0u); f.next();
    f.g(0, 0); f.start(); f.release(); f.soc.tick(333); //output3 direction is not driven
    ZLB_EXPECT_EQ(f.g(0x38), 0u); f.next(); f.g(0, 8);
    f.g(0x14, 0x200); f.start(); f.release(); f.soc.tick(333);
    ZLB_EXPECT_EQ(f.g(0x38), 0u); f.next(); f.g(0x14, 0x300);
    f.g(0x1C, 0xFFFFFFFF); f.start(); f.release(); f.soc.tick(333);
    ZLB_EXPECT_EQ(f.g(0x38), 0u); ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 0u);
}

ZLB_TEST(spi_syscon_ready_error_reply_is_transport_only_and_cannot_reuse_a_stale_generation) {
    SysconWireFixture f; f.native();
    const auto error = ernie::make_spi_response(4, 0, {0x42});
    f.spi->set_slave([&](const std::vector<u8>&) { return error; });
    f.start(); f.release(); f.soc.tick(333);
    ZLB_EXPECT_EQ(f.g(0x38), 0x10u); ZLB_EXPECT_TRUE(f.spi->last_response() == error);
    f.ack(); f.g(0xC, 8); f.release(); f.soc.tick(333);
    ZLB_EXPECT_EQ(f.g(0x38), 0u); ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 1u);
    // A second start without draining old captured bytes cannot attribute the
    // old response to a new ready pulse, even with another valid packet.
    f.g(0xC, 8); f.start(); f.release(); f.soc.tick(333);
    ZLB_EXPECT_EQ(f.g(0x38), 0u); ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 1u);
    f.next(); f.start(); f.release(); f.soc.tick(333);
    ZLB_EXPECT_EQ(f.g(0x38), 0x10u); ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 2u);
}

ZLB_TEST(spi_syscon_ready_real_gpio248_gic_and_unchanged_lowio_candidate_accept_the_pulse) {
    SysconWireFixture f; f.native();
    ArmCore receiver(f.arm); receiver.core_id_ = 3; receiver.reset(); receiver.set_register("CPSR", 0x13);
    kermit_set_cpu(f.arm, &receiver, 3);
    const u32 dist = kermit::kScuBase + kermit::kGicDistOffset;
    const u32 icc = kermit::kScuBase + kermit::kIccOffset;
    f.arm.context.core_id = 3; f.arm.context.nonsecure = false;
    f.arm.write32(dist, 1); f.arm.write32(icc, 0xB); f.arm.write32(icc + 4, 0xFF);
    f.arm.write32(dist + 0x100 + (248 / 32) * 4, 1u << (248 % 32));
    f.arm.write8(dist + 0x400 + 248, 0x50); f.arm.write8(dist + 0x800 + 248, 0xF);
    f.start(); f.release(); ZLB_EXPECT_FALSE(receiver.interrupt_pending()); f.soc.tick(333);
    ZLB_EXPECT_TRUE(receiver.interrupt_pending());
    ZLB_EXPECT_EQ(f.arm.read32(icc + 0xC) & 0x3FFu, 248u);
    // Exact supplied Lowio PT_LOADA0 linked810022A4..22CC. Its mode3 setter
    // clears shadow bit4, so candidate qualification requires sampled pin4
    // high after the saved falling edge. This tests candidate eligibility,
    // not a fabricated sub4 call or completion of the full interrupt handler.
    f.arm.add_ram("unchanged Lowio GPIO candidate", 0x10000, 0x81000000, "supplied bytes and fixture RAM");
    write_hex(f.arm, 0x810022A4,
        "f72c42ddfc2cd8bfa4f1f80138dcd7f804e00b1db26a756a56f823308eea020222ea0502134004d1");
    const u32 record = 0x8100B120;
    f.arm.write32(record, f.kGpio); f.arm.write32(record + 0x10, 0x10);
    f.arm.write32(record + 0x24, 0); f.arm.write32(record + 0x28, 0xFFFFFFEF);
    ArmCore native(f.arm); native.reset(0x810022A5);
    native.r[4] = 248; native.r[6] = record; native.r[7] = f.kGpio;
    for (unsigned step = 0; step < 32 && native.get_pc() != 0x810022CA; ++step)
        ZLB_EXPECT_FALSE(native.step().faulted);
    ZLB_EXPECT_EQ(native.get_pc(), 0x810022CAu); ZLB_EXPECT_EQ(native.r[3], 0x10u);
    native.step(); ZLB_EXPECT_EQ(native.get_pc(), 0x810022D6u);
    // Negative control: persistent low is rejected by the same unchanged
    // candidate bytes despite latched pending status. No polarity override.
    f.gpio->set_external_input(0x10, 0);
    native.reset(0x810022A5); native.r[4] = 248; native.r[6] = record; native.r[7] = f.kGpio;
    for (unsigned step = 0; step < 32 && native.get_pc() != 0x810022CA; ++step)
        ZLB_EXPECT_FALSE(native.step().faulted);
    ZLB_EXPECT_EQ(native.r[3], 0u); native.step();
    ZLB_EXPECT_EQ(native.get_pc(), 0x810022CCu);
    f.gpio->set_external_input(0x10, 0x10);
    f.arm.context.core_id = 3; f.arm.context.nonsecure = false;
    f.ack(); f.arm.write32(icc + 0x10, 248);
    ZLB_EXPECT_FALSE(receiver.interrupt_pending()); ZLB_EXPECT_EQ(f.g(4) & 0x10u, 0x10u);
    ZLB_EXPECT_EQ(f.s(0x28), 10u); ZLB_EXPECT_EQ(f.spi->syscon_ready_edges(), 1u);
    kermit_set_cpu(f.arm, nullptr, 3);
}
