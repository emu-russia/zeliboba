// Native firmware 1.04 OLED transport tests. These portable fixtures execute
// unchanged guest instructions; they do not patch helper returns or readiness.
#include <algorithm>
#include <cstring>
#include <memory>

#include "cpu/arm/arm_core.h"
#include "hw/soc/soc_internal.h"
#include "test_framework.h"

using namespace zlb;

namespace {

// Genuine firmware 1.04 oled.elf SHA-256 ddafc3324ee69e817f198c63363eadb58c26ec5dd916777b55c88c1a38635851
// Embedded original error-return import stubs are an explicit isolated boundary:
// clock/reset/GPIO providers are absent here; no executable byte is substituted.
// oled.elf linked 0x81000000+0x160..+0x980, file +0x200, unchanged.
// SHA-256 9b2a6b47a51df62edfa48799d6658aabb5dd3960c1f696225afab1cd9558a659
constexpr const char kNativeOledCode[] =
    "f8b543f20003c8f200131a68013a1a60002a39d192faa2f0de684fea106e5a69"
    "4fea4e0efeb1df681969586907f109060efa07f40f2ede6044ea01041c61f1dd"
    "3546c16a7f29fcd0103da1b2240c0f2d4160f6dcf91f103e21f00f001c61361a"
    "de60002edfd1d36a23b1d36a002bfad100e01368906a0028fbd101210220c0f2"
    "0301916001f08ce9f8bd00bf2de9f84343f20004c8f2001405460e4617462368"
    "002b00f0a5820133af2d43f200002360c8f2001000f2e580eab2d0f80ce00369"
    "92faa2f54269290e0ef10905c56049000f2d01fa0ef141ea0301016111dd2846"
    "d36a7f2bfcd010388bb2090c0f285360f6dcaef10703103d23f00f022161a81a"
    "e06017f101092cd04ff0000c9cfaacf54fea156e4fea4e0848f00108d4f80ce0"
    "62690ef1090508fa0ef00f2de56041ea0001216111dd2846d36a7f2bfcd01038"
    "8bb2090c0f285360f6dcaef10703103d23f00f0021612a1ae2600cf1010ccc45"
    "dcd34ff0000ed4f814c09efaaef8e5684fea18614fea41080db3d4f80ce02369"
    "62690ef1090508fa0ef00f2de56040ea03012161f0dd2846d36a7f2bfcd01038"
    "8bb2090c0f285360f6dcaef10700103d20f00f022161ad1ae560002dddd1dcf8"
    "2c50002dfbd1ccf810506469002f53d0022328462946a26a002afcd022688a40"
    "1031082940ea020044dd48f63962a1f1090cc3f6e302a2fb0cc2c2f34102002b"
    "00f0aa81013b0939400a082932ddbab1012a0dd0022a05d0002b00f0df81400a"
    "013b0939002b00f0cf81400a013b0939002b00f0c181013b0939400a082919dd"
    "002b00f06081013b4fea502e0939002b00f05181013b4fea5e22002b00f04381"
    "013b500a002b00f03781013b1b39400a0829e5dcaf42aed8fff7c2fe0020bde8"
    "f883fb21d0f80cc091faa1f20369110e42690cf1090e4900bef10f0fc0f80ce0"
    "01fa0cf141ea0301016113dd7046d36a7f2bfcd010388bb2090c0f285360f6dc"
    "acf10703aef1100923f00f022161c2eb0900e060eab2d4f80ce0206992faa2f5"
    "62692b0e0ef10905e56059000f2d41f0010303fa0ef141ea0001216111dd2846"
    "d36a7f2bfcd010388bb2090c0f285360f6dcaef10703103d23f00f022161a81a"
    "e060fc22d4f80ce092faa2f52069290e62690ef109054b000f2de56003fa0ef1"
    "41ea0001216111dd2846d36a7f2bfcd010388bb2090c0f285360f6dcaef10703"
    "103d23f00f022161a81ae06017f101092cd04ff0000c9cfaacf54fea156e4fea"
    "4e0848f00108d4f80ce062690ef1090508fa0ef00f2de56041ea0001216111dd"
    "2846d36a7f2bfcd010388bb2090c0f285360f6dcaef10703103d23f00f002161"
    "2a1ae2600cf1010ccc45dcd34ff0000ed4f814c09efaaef8e5684fea18614fea"
    "41080db3d4f80ce0236962690ef1090508fa0ef00f2de56040ea03012161f0dd"
    "2846d36a7f2bfcd010388bb2090c0f285360f6dcaef10700103d20f00f022161"
    "ad1ae560002dddd1dcf82c50002dfbd1ccf810506469002f3ff41eaf04232846"
    "2946a26a002afcd022688a401031082940ea02003ddd48f63962a1f1090cc3f6"
    "e302a2fb0cc2c2f34102002b7ed0013b0939400a08292cddb2b1012a0dd0022a"
    "05d0002b00f08480400a013b0939002b00f08880400a013b0939002b70d0013b"
    "0939400a082914dd002b4dd0013b4fea502e0939002b3fd0013b4fea5e2253b3"
    "013b500a002b30d0013b1b39400a0829eadcaf42b5d8cfe6920a92faa2f2120e"
    "72550135c2e6cef3872090faa0f0000e70550135b5e6c0f3872090faa0f2100e"
    "70550135a7e6c0f3470292faa2f2120e7255013598e6cef3872090faa0f0000e"
    "70550135500a002bced1920a92faa2f2120e72550135c8e7c0f3872090faa0f2"
    "100e70550135b8e7c0f3470292faa2f2120e72550135aae7c0f3470e9efaaefc"
    "4fea1c6e06f805e001354ce6c0f3470c9cfaacfe4fea1e6c06f805c0013577e7"
    "c0f3470292faa2f2120e7255013587e7c0f34702093992faa2f2400a120e7255"
    "013574e7c0f34702093992faa2f2400a120e7255013570e7c0f3470292faa2f2"
    "120e7255013537e6c0f34702093992faa2f2400a120e7255013529e6c0f34702"
    "093992faa2f2400a120e7255013519e6d4f81480022000f0dceed8f828302bb1"
    "d8f80030d8f828000028f9d101210122c0f203012368c8f80810c8f8102042e5"
    "002070b5014682b000f0d2ee014600282fd100f0deee44f62060012400f088ee"
    "a12005226946fff721fd9df80430ff2b29d043f200050026c8f20015a4b946b1"
    "298943f20003c8f2001311b901225a6007e0022000f094ee022000f08aee0223"
    "6b60012002b070bd0020014600f0b8eee5e7a120052269460024fff7f7fc9df8"
    "0430ff2bd5d19df8010043f200059df80020c8f200159df8031001269df80230"
    "42ea002243ea01202a816881002cc6d0dae700bf41f6501008b5c8f2001000f0"
    "48ee002801db022008bdfff7d9fb00280fdb232041f65c1140f28172c0f20100"
    "c8f20011c8f20012002300f02aee002008bd012008bd00bf38b5054600f010ee"
    "044600f00eee001ba842fad338bd00bf002008b5014600f064ee002008bd00bf"
    "002008b5014600f054ee002008bd00bf002008b5014600f03ceed0f1010038bf"
    "002008bd70b543f20006c8f2001640f2e735736853b900f0e4ed044600f0e0ed"
    "001ba842fad97368002bf4d0002070bd43f20003c8f200135a6822b940f60420"
    "c8f23f007047022a07d008b11a89028041b15b8900200b80704740f60320c8f2"
    "3f0070470846704708b543f20003c8f200135b6823b940f60420c8f23f0008bd"
    "022b02d0fff752fc08bd40f60320c8f23f0008bdf0b543f20004c8f200140546"
    "40f6042083b06368c8f23f000bb903b0f0bd022b04bf40f60320c8f23f00f6d0";

// oled.elf linked 0x81000000+0x14A0..+0x1570, file +0x1540, unchanged.
// SHA-256 2002468cb438e64fc9282efa8c6ed6d9f662910809f518b47c8b04ca647776d7
constexpr const char kNativeOledImports[] =
    "0000e0e31eff2fe10000a0e1000000000000e0e31eff2fe10000a0e100000000"
    "0000e0e31eff2fe10000a0e1000000000000e0e31eff2fe10000a0e100000000"
    "0000e0e31eff2fe10000a0e1000000000000e0e31eff2fe10000a0e100000000"
    "0000e0e31eff2fe10000a0e1000000000000e0e31eff2fe10000a0e100000000"
    "0000e0e31eff2fe10000a0e1000000000000e0e31eff2fe10000a0e100000000"
    "0000e0e31eff2fe10000a0e1000000000000e0e31eff2fe10000a0e100000000"
    "0000e0e31eff2fe10000a0e100000000";

void copy_hex(u8* destination, const char* hex) {
    auto nibble = [](char c) -> unsigned { return c <= '9' ? c - '0' : c - 'a' + 10; };
    for (size_t i = 0; hex[i * 2]; ++i)
        destination[i] = static_cast<u8>((nibble(hex[i * 2]) << 4) | nibble(hex[i * 2 + 1]));
}

struct ObservedSpi : kermit::Spi {
    ObservedSpi(u32 port = 2) : Spi("SPI native fixture", 0xE0A20000, 0x1000, port) {}
    std::vector<u16> words;
    std::vector<size_t> rx_at_stop;

    void write_word(u32 offset, u64 value) override {
        if (offset == kTxFifo) words.push_back(static_cast<u16>(value));
        if (offset == kStatus && (value & 1u) == 0) rx_at_stop.push_back(rx_pending());
        Spi::write_word(offset, value);
    }
};

struct NativeOledFixture {
    static constexpr u32 kCode = 0x81000000;
    static constexpr u32 kBss = 0x81003000;
    static constexpr u32 kOutput = 0x81004000;
    static constexpr u32 kStack = 0x81004FF0;
    static constexpr u32 kReturn = 0x81002000;
    static constexpr u32 kSpi = 0xE0A20000;

    std::unique_ptr<Bus> owner = std::make_unique<Bus>();
    Bus& bus = *owner;
    ArmCore cpu{bus};
    ObservedSpi* spi = nullptr;
    bool irq = false;

    NativeOledFixture() {
        auto& code = bus.add_ram("unchanged OLED code", 0x3000, kCode, "portable native instruction fixture");
        copy_hex(code.bytes() + 0x160, kNativeOledCode);
        copy_hex(code.bytes() + 0x14A0, kNativeOledImports);
        bus.add_ram("native fixture BSS, output and stack", 0x3000, kBss, "ordinary caller RAM");
        auto device = std::make_unique<ObservedSpi>();
        spi = device.get();
        bus.add_device(std::move(device));
        spi->set_irq_callback([this](u32, bool asserted) { irq = asserted; });
        bus.rebuild_map();
        spi->reset();
        bus.write32(kBss + 0x14, kSpi);
    }

    bool call(u32 offset, u32 r0 = 0, u32 r1 = 0, u32 r2 = 0) {
        cpu.reset(kCode + offset + 1);
        cpu.r[0] = r0;
        cpu.r[1] = r1;
        cpu.r[2] = r2;
        cpu.r[13] = kStack;
        cpu.r[14] = kReturn | 1u;
        for (unsigned step = 0; step < 5000; ++step) {
            if (cpu.get_pc() == kReturn) return true;
            cpu.step();
            if (cpu.undefined_instruction || cpu.halted) return false;
        }
        return false;
    }
};

void expect_a1_words(const ObservedSpi& spi) {
    static constexpr u16 expected[] = {0x030A, 0x0804, 0x2010, 0x0040, 0, 0, 0, 0, 0};
    ZLB_EXPECT_EQ(spi.words.size(), std::size(expected));
    for (size_t i = 0; i < std::min(spi.words.size(), std::size(expected)); ++i)
        ZLB_EXPECT_EQ(spi.words[i], expected[i]);
}

}  // namespace

ZLB_TEST(spi_oled_unchanged_a1_reads_undriven_high_and_closes) {
    NativeOledFixture fx;
    fx.bus.write8(fx.kOutput - 1, 0x35);
    fx.bus.write8(fx.kOutput + 5, 0x53);
    fx.bus.write32(fx.kStack, 0xA55AA55A);
    const bool returned = fx.call(0x1EC, 0xA1, fx.kOutput, 5);
    ZLB_EXPECT_TRUE(returned);
    expect_a1_words(*fx.spi);
    if (!returned) return;

    for (unsigned i = 0; i < 5; ++i) ZLB_EXPECT_EQ(fx.bus.read8(fx.kOutput + i), 0xFFu);
    ZLB_EXPECT_EQ(fx.bus.read8(fx.kOutput - 1), 0x35u);
    ZLB_EXPECT_EQ(fx.bus.read8(fx.kOutput + 5), 0x53u);
    ZLB_EXPECT_EQ(fx.cpu.r[13], fx.kStack);
    ZLB_EXPECT_EQ(fx.bus.read32(fx.kStack), 0xA55AA55Au);
    ZLB_EXPECT_EQ(fx.spi->transfers(), 1u);
    ZLB_EXPECT_EQ(fx.spi->bytes_tx(), 18u);
    ZLB_EXPECT_EQ(fx.spi->bytes_rx(), 18u);
    ZLB_EXPECT_EQ(fx.spi->rx_pending(), size_t(0));
    ZLB_EXPECT_EQ(fx.bus.read32(fx.kBss), 0u);  // genuine close decremented open count
    ZLB_EXPECT_EQ(fx.bus.read32(fx.kBss + 0x0C), 0u);
    ZLB_EXPECT_EQ(fx.spi->rx_at_stop.size(), size_t(1));
    if (!fx.spi->rx_at_stop.empty()) ZLB_EXPECT_EQ(fx.spi->rx_at_stop.front(), size_t(18));
    ZLB_EXPECT_FALSE(fx.irq);  // native INTCTL3 does not unmask existing RX bit9
}

ZLB_TEST(spi_oled_unchanged_worker_rejects_high_input_and_keeps_api_errors) {
    NativeOledFixture fx;
    // Original unbound GPIO import returns -1, selecting the native pin-high
    // branch. This fixture tests the unchanged response predicate, not GPIO.
    const bool returned = fx.call(0x780);
    ZLB_EXPECT_TRUE(returned);
    expect_a1_words(*fx.spi);
    if (!returned) return;
    ZLB_EXPECT_EQ(fx.cpu.r[0], 1u);  // native work-item return, not panel readiness
    ZLB_EXPECT_EQ(fx.cpu.r[13], fx.kStack);
    ZLB_EXPECT_EQ(fx.bus.read32(fx.kBss + 4), 2u);
    ZLB_EXPECT_EQ(fx.bus.read16(fx.kBss + 8), 0xFFFFu);
    ZLB_EXPECT_EQ(fx.bus.read16(fx.kBss + 0x0A), 0xFFFFu);

    fx.bus.write32(fx.kOutput, 0x12345678);
    ZLB_EXPECT_TRUE(fx.call(0x8F0, fx.kOutput, fx.kOutput + 2));  // GetDDB
    ZLB_EXPECT_EQ(fx.cpu.r[0], 0x803F0A03u);
    ZLB_EXPECT_EQ(fx.bus.read32(fx.kOutput), 0x12345678u);
    ZLB_EXPECT_TRUE(fx.call(0x928, 0xA1, fx.kOutput, 5));  // read API
    ZLB_EXPECT_EQ(fx.cpu.r[0], 0x803F0A03u);
    ZLB_EXPECT_TRUE(fx.call(0x954, 0x29));  // write API
    ZLB_EXPECT_EQ(fx.cpu.r[0], 0x803F0A03u);
    ZLB_EXPECT_EQ(fx.spi->transfers(), 1u);
    ZLB_EXPECT_EQ(fx.spi->bytes_tx(), 18u);
    ZLB_EXPECT_TRUE(fx.call(0x8C4));  // WaitReady accepts completed failure, returns zero
    ZLB_EXPECT_EQ(fx.cpu.r[0], 0u);
    ZLB_EXPECT_EQ(fx.bus.read32(fx.kBss + 4), 2u);
}

ZLB_TEST(spi_oled_clocks_only_consumed_words_and_stop_retains_input) {
    NativeOledFixture fx;
    auto write = [&](u32 offset, u32 value) { fx.bus.write32(fx.kSpi + offset, value); };
    auto read = [&](u32 offset) { return fx.bus.read32(fx.kSpi + offset); };
    write(0x08, 0x30001);
    write(0x0C, 3);
    write(0x10, 1);  // the genuine empty start clocks no bits
    ZLB_EXPECT_EQ(read(0x28), 0u);
    ZLB_EXPECT_EQ(read(0x2C), 0u);
    ZLB_EXPECT_EQ(read(0x24), 0u);
    write(0x04, 0xDEAD1234);  // only the actual low16 word enters the wire path
    write(0x04, 0xBEEFABCD);
    ZLB_EXPECT_EQ(read(0x2C), 0u);
    ZLB_EXPECT_EQ(read(0x28), 4u);
    ZLB_EXPECT_EQ(fx.spi->bytes_rx(), 4u);
    ZLB_EXPECT_EQ(fx.spi->last_request().size(), size_t(4));
    if (fx.spi->last_request().size() == 4) {
        ZLB_EXPECT_EQ(fx.spi->last_request()[0], 0x34u);
        ZLB_EXPECT_EQ(fx.spi->last_request()[1], 0x12u);
        ZLB_EXPECT_EQ(fx.spi->last_request()[2], 0xCDu);
        ZLB_EXPECT_EQ(fx.spi->last_request()[3], 0xABu);
    }
    ZLB_EXPECT_FALSE(fx.irq);
    write(0x10, 0);  // firmware stops first, then receives
    ZLB_EXPECT_EQ(read(0x28), 4u);
    write(0x04, 0x9876);  // stopped words wait; they do not sample input yet
    ZLB_EXPECT_EQ(read(0x2C), 2u);
    ZLB_EXPECT_EQ(read(0x28), 4u);
    ZLB_EXPECT_EQ(read(0), 0xFFFFu);
    ZLB_EXPECT_EQ(read(0x28), 2u);
    ZLB_EXPECT_EQ(read(0), 0xFFFFu);
    ZLB_EXPECT_EQ(read(0x28), 0u);
    write(0x10, 1);  // prequeued word is consumed exactly once at a real start
    ZLB_EXPECT_EQ(read(0x2C), 0u);
    ZLB_EXPECT_EQ(read(0x28), 2u);
    ZLB_EXPECT_EQ(fx.spi->bytes_tx(), 6u);
    ZLB_EXPECT_EQ(fx.spi->bytes_rx(), 6u);
    ZLB_EXPECT_EQ(fx.spi->transfers(), 2u);
    ZLB_EXPECT_EQ(read(0), 0xFFFFu);
    ZLB_EXPECT_EQ(read(0), 0u);  // no spontaneous input after all clocked bits
}

ZLB_TEST(spi_oled_reset_cancels_stream_and_clears_pending_level) {
    NativeOledFixture fx;
    auto write = [&](u32 offset, u32 value) { fx.bus.write32(fx.kSpi + offset, value); };
    auto read = [&](u32 offset) { return fx.bus.read32(fx.kSpi + offset); };
    write(0x08, 0x30001);
    write(0x10, 1);
    write(0x04, 0x1234);
    ZLB_EXPECT_FALSE(fx.irq);  // queued input alone is masked
    write(0x0C, 0x200);  // existing RX latch, no invented OLED completion bit
    ZLB_EXPECT_TRUE(fx.irq);
    write(0x24, 0x400);  // unrelated W1C does not acknowledge RX
    ZLB_EXPECT_TRUE(fx.irq);
    write(0x24, 0x200);
    ZLB_EXPECT_FALSE(fx.irq);
    ZLB_EXPECT_EQ(read(0x28), 2u);  // interrupt acknowledgement is not a FIFO flush
    write(0x04, 0xABCD);
    ZLB_EXPECT_TRUE(fx.irq);

    fx.spi->reset();
    ZLB_EXPECT_FALSE(fx.irq);
    ZLB_EXPECT_EQ(read(0x28), 0u);
    ZLB_EXPECT_EQ(read(0x2C), 0u);
    ZLB_EXPECT_EQ(read(0x24), 0u);
    ZLB_EXPECT_EQ(fx.spi->bytes_rx(), 0u);
    write(0x08, 0x30001);
    write(0x04, 0x5678);  // setting CTL after reset cannot resurrect the old start
    ZLB_EXPECT_EQ(read(0x2C), 2u);
    ZLB_EXPECT_EQ(read(0x28), 0u);
    write(0x10, 1);
    ZLB_EXPECT_EQ(read(0x2C), 0u);
    ZLB_EXPECT_EQ(read(0x28), 2u);
    ZLB_EXPECT_EQ(read(0), 0xFFFFu);
    ZLB_EXPECT_EQ(fx.spi->transfers(), 1u);
    ZLB_EXPECT_EQ(fx.spi->bytes_rx(), 2u);
}

ZLB_TEST(spi_oled_control_change_requires_new_start) {
    NativeOledFixture fx;
    auto write = [&](u32 offset, u32 value) { fx.bus.write32(fx.kSpi + offset, value); };
    auto read = [&](u32 offset) { return fx.bus.read32(fx.kSpi + offset); };
    write(0x08, 0x30001);
    write(0x10, 1);
    write(0x04, 0x1234);
    write(0x08, 0);
    write(0x04, 0xABCD);  // change cancels the engine but retains already captured RX
    ZLB_EXPECT_EQ(read(0x28), 2u);
    ZLB_EXPECT_EQ(read(0x2C), 2u);
    write(0x08, 0x30001);
    write(0x04, 0x5678);  // mode selection alone does not restart
    ZLB_EXPECT_EQ(read(0x28), 2u);
    ZLB_EXPECT_EQ(read(0x2C), 4u);
    write(0x10, 1);
    ZLB_EXPECT_EQ(read(0x2C), 0u);
    ZLB_EXPECT_EQ(read(0x28), 6u);
    ZLB_EXPECT_EQ(fx.spi->bytes_rx(), 6u);
}

ZLB_TEST(spi_oled_subset_preserves_other_ports_and_framed_modes) {
    // Native SPI0's request must invoke one whole-frame peer and keep the
    // original low-byte-first reply contract. Unsupported ports/modes cannot
    // acquire an undriven-high reply or start-before-TX stream behavior.
    for (const auto& config : {std::pair{0u, 0u}, std::pair{0u, 0x30001u},
                               std::pair{1u, 0x30001u}, std::pair{2u, 0u},
                               std::pair{2u, 0x30000u}, std::pair{2u, 0x30003u}}) {
        auto owner = std::make_unique<Bus>();
        Bus& bus = *owner;
        auto device = std::make_unique<ObservedSpi>(config.first);
        auto* spi = device.get();
        unsigned callbacks = 0;
        spi->set_slave([&](const std::vector<u8>& request) {
            ++callbacks;
            ZLB_EXPECT_EQ(request.size(), size_t(4));
            if (request.size() == 4) {
                ZLB_EXPECT_EQ(request[0], 1u);
                ZLB_EXPECT_EQ(request[1], 0u);
                ZLB_EXPECT_EQ(request[2], 1u);
                ZLB_EXPECT_EQ(request[3], 0xFDu);
            }
            return std::vector<u8>{0xC1, 0x02, 0x5A};
        });
        bus.add_device(std::move(device));
        bus.rebuild_map();
        bus.write32(0xE0A20008, config.second);
        bus.write32(0xE0A20004, 1);
        bus.write32(0xE0A20004, 0xFD01);
        ZLB_EXPECT_EQ(bus.read32(0xE0A2002C), 4u);
        ZLB_EXPECT_EQ(bus.read32(0xE0A20028), 0u);
        ZLB_EXPECT_EQ(callbacks, 0u);
        bus.write32(0xE0A20010, 1);
        ZLB_EXPECT_EQ(callbacks, 1u);
        ZLB_EXPECT_EQ(bus.read32(0xE0A2002C), 0u);
        ZLB_EXPECT_EQ(bus.read32(0xE0A20028), 3u);
        ZLB_EXPECT_EQ(bus.read32(0xE0A20000), 0x02C1u);
        ZLB_EXPECT_EQ(bus.read32(0xE0A20028), 1u);
        ZLB_EXPECT_EQ(bus.read32(0xE0A20000), 0x5Au);
        bus.write32(0xE0A20004, 0xABCD);  // legacy one-shot start does not stay armed
        ZLB_EXPECT_EQ(bus.read32(0xE0A2002C), 2u);
        ZLB_EXPECT_EQ(bus.read32(0xE0A20028), 0u);
        ZLB_EXPECT_EQ(callbacks, 1u);
    }
}
