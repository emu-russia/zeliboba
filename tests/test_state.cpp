// Deterministic save states: stream layout, RAM pages, file container and a
// full device+core resume check.
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/state.h"
#include "common/util.h"
#include "cpu/arm/arm_core.h"
#include "hw/soc/soc_internal.h"
#include "test_framework.h"

using namespace zlb;

namespace {

/// A small Kermit SoC plus one ARM core running a self-contained loop. The loop
/// touches a device register (the private timer) and RAM, so a resume that
/// misses either CPU or device state diverges immediately.
struct StateFixture {
    std::unique_ptr<Bus> owner = std::make_unique<Bus>();
    Bus& bus = *owner;
    KermitBlock soc{bus, nullptr};
    std::unique_ptr<ArmCore> cpu;

    static constexpr u32 kCode = 0x00100000;
    static constexpr u32 kData = 0x00100800;

    StateFixture() {
        soc.install();
        soc.reset();
        bus.add_ram("state_test_ram", 0x2000, kCode, "save-state test program and data");
        // add r0, r0, #1 ; str r0, [r1] ; str r0, [r2] (timer load) ; b start
        bus.write32(kCode + 0x0, 0xE2800001u);
        bus.write32(kCode + 0x4, 0xE5810000u);
        bus.write32(kCode + 0x8, 0xE5820000u);
        bus.write32(kCode + 0xC, 0xEAFFFFFBu);
        cpu = std::make_unique<ArmCore>(bus);
        cpu->reset(kCode);
        cpu->r[0] = 0;
        cpu->r[1] = kData;
        cpu->r[2] = kermit::kLt5Base;   // a device register: write state too
    }

    void run(int steps) {
        for (int i = 0; i < steps; ++i) cpu->step();
    }

    /// Snapshot the bus and the core into one comparable byte string.
    std::vector<u8> snapshot() const {
        StateWriter writer;
        writer.begin("bus");
        bus.save_state(writer);
        writer.end();
        writer.begin("cpu");
        cpu->save_state(writer);
        writer.end();
        return writer.data();
    }

    void restore(const std::vector<u8>& bytes) {
        StateReader reader(bytes);
        reader.begin("bus");
        bus.load_state(reader);
        reader.end();
        reader.begin("cpu");
        cpu->load_state(reader);
        reader.end();
        ZLB_EXPECT_TRUE(reader.ok());
    }
};

}  // namespace

ZLB_TEST(state_stream_sections_and_scalars_round_trip) {
    StateWriter writer;
    writer.begin("outer");
    writer.put_u8(0x12);
    writer.put_u16(0x3456);
    writer.put_u32(0x789ABCDEu);
    writer.put_u64(0x0123456789ABCDEFull);
    writer.put_i32(-7);
    writer.put_bool(true);
    writer.str("hello");
    writer.begin("inner");
    writer.put_u32(42);
    writer.end();
    writer.put_f64(1.5);
    writer.end();

    StateReader reader(writer.data());
    ZLB_EXPECT_TRUE(reader.begin("outer"));
    ZLB_EXPECT_EQ(reader.get_u8(), 0x12);
    ZLB_EXPECT_EQ(reader.get_u16(), 0x3456);
    ZLB_EXPECT_EQ(reader.get_u32(), 0x789ABCDEu);
    ZLB_EXPECT_EQ(reader.get_u64(), 0x0123456789ABCDEFull);
    ZLB_EXPECT_EQ(reader.get_i32(), -7);
    ZLB_EXPECT_TRUE(reader.get_bool());
    ZLB_EXPECT_TRUE(reader.str() == "hello");
    ZLB_EXPECT_TRUE(reader.begin("inner"));
    ZLB_EXPECT_EQ(reader.get_u32(), 42u);
    reader.end();
    ZLB_EXPECT_NEAR(reader.get_f64(), 1.5, 1e-9);
    reader.end();
    ZLB_EXPECT_TRUE(reader.ok());
    ZLB_EXPECT_TRUE(reader.at_end());
}

ZLB_TEST(state_reader_rejects_a_wrong_section) {
    StateWriter writer;
    writer.begin("expected");
    writer.put_u32(1);
    writer.end();
    StateReader reader(writer.data());
    ZLB_EXPECT_FALSE(reader.begin("other"));
    ZLB_EXPECT_FALSE(reader.ok());
    ZLB_EXPECT_TRUE(!reader.error().empty());
}

ZLB_TEST(state_zero_pages_round_trip_sparsely) {
    std::vector<u8> data(64 * 1024, 0);
    data[100] = 0xAB;
    data[4096 * 10 + 3] = 0xCD;
    data[data.size() - 1] = 0xEF;

    StateWriter writer;
    const size_t written = state_write_pages(writer, data.data(), data.size());
    // Two non-zero pages only (page 0 and page 10), plus the last byte in page 15.
    ZLB_EXPECT_EQ(written, 3u * 4096u);
    ZLB_EXPECT_TRUE(writer.size() < data.size() / 2);

    std::vector<u8> restored(data.size(), 0x55);
    StateReader reader(writer.data());
    state_read_pages(reader, restored.data(), restored.size());
    ZLB_EXPECT_TRUE(reader.ok());
    ZLB_EXPECT_TRUE(restored == data);
}

ZLB_TEST(state_file_round_trip_and_corruption_check) {
    StateWriter body;
    body.begin("payload");
    body.put_u32(0xDEADBEEFu);
    body.str("save state");
    body.end();

    const std::string path = "state-test.zlbstate";
    std::string error;
    ZLB_EXPECT_TRUE(state_write_file(path, body, error));
    std::vector<u8> file;
    ZLB_EXPECT_TRUE(state_read_file(path, file, error));
    ZLB_EXPECT_TRUE(file == body.data());

    // Flip one payload byte: the checksum has to reject it.
    auto data = read_file(path);
    ZLB_EXPECT_TRUE(data.has_value());
    if (data && data->size() > 32) {
        (*data)[32] ^= 0xFF;
        ZLB_EXPECT_TRUE(write_file(path, *data));
        std::vector<u8> ignored;
        ZLB_EXPECT_FALSE(state_read_file(path, ignored, error));
    }
    std::remove(path.c_str());
}

ZLB_TEST(state_arm_and_kermit_resume_is_deterministic) {
    StateFixture fixture;
    fixture.run(400);
    const std::vector<u8> checkpoint = fixture.snapshot();
    const u64 checkpoint_insns = fixture.cpu->instructions;

    // Continue without interruption: this is the reference.
    fixture.run(300);
    const std::vector<u8> reference = fixture.snapshot();
    const u64 reference_insns = fixture.cpu->instructions;
    ZLB_EXPECT_TRUE(reference_insns > checkpoint_insns);

    // Rewind and run the same 300 steps again.
    fixture.restore(checkpoint);
    ZLB_EXPECT_EQ(fixture.cpu->instructions, checkpoint_insns);
    fixture.run(300);
    const std::vector<u8> resumed = fixture.snapshot();
    ZLB_EXPECT_EQ(fixture.cpu->instructions, reference_insns);
    ZLB_EXPECT_TRUE(resumed == reference);
}

ZLB_TEST(state_region_repointed_at_an_external_buffer_round_trips) {
    // The boot chain re-points the CMeP RAM window at the private-SRAM buffer
    // while it runs; a state saved after that must still carry the bytes, and a
    // later load must put them back where the region currently reads from.
    Bus bus;
    std::vector<u8> shared(0x1000, 0);
    bus.add_ram("plain", 0x1000, 0x00100000, "test region");
    bus.add_ram_alias("alias", 0x00200000, 0x1000, shared.data(), "test alias");
    // `add_ram_alias` may reallocate `regions_`, so resolve the reference after
    // both regions exist.
    MemRegion* ram = bus.region_at(0x00100000);
    ZLB_EXPECT_TRUE(ram != nullptr);
    bus.write8(0x00100000, 0x11);
    ram->external = shared.data();   // the runtime re-point
    bus.rebuild_map();               // the machine rebuilds the cached page pointers
    bus.write8(0x00100000, 0x22);

    StateWriter writer;
    writer.begin("bus");
    bus.save_state(writer);
    writer.end();

    shared[0] = 0x33;
    StateReader reader(writer.data());
    reader.begin("bus");
    bus.load_state(reader);
    reader.end();
    ZLB_EXPECT_TRUE(reader.ok());
    ZLB_EXPECT_EQ(bus.read8(0x00100000), 0x22);
    ZLB_EXPECT_EQ(shared[0], 0x22);
}

ZLB_TEST(state_device_registers_are_restored) {
    StateFixture fixture;
    fixture.run(120);
    const u32 lt_before = fixture.bus.read32(kermit::kLt5Base);
    const u32 wt_before = fixture.bus.read32(kermit::kWt7Base);
    const std::vector<u8> checkpoint = fixture.snapshot();

    // Change device state behind the snapshot's back, then restore.
    fixture.bus.write32(kermit::kLt5Base, lt_before ^ 0x5A5A5A5Au);
    fixture.bus.write32(kermit::kWt7Base, wt_before ^ 0xA5A5A5A5u);
    ZLB_EXPECT_NE(fixture.bus.read32(kermit::kLt5Base), lt_before);
    fixture.restore(checkpoint);
    ZLB_EXPECT_EQ(fixture.bus.read32(kermit::kLt5Base), lt_before);
    ZLB_EXPECT_EQ(fixture.bus.read32(kermit::kWt7Base), wt_before);
}
