// zeliboba - bus, trace and device self tests.
#include "test_framework.h"

#include "bus/bus.h"
#include "bus/device.h"
#include "common/util.h"

using namespace zlb;

namespace {

class TinyDevice : public RegisterFile {
public:
    TinyDevice() : RegisterFile("Tiny", 0xF0000000, 0x100) {
        define(0xF0000000, "CTRL", 0x1234);
        define(0xF0000004, "STATUS", 0);
    }
};

}  // namespace

ZLB_TEST(bus_ram_read_write) {
    Bus bus;
    bus.add_ram("ram", 0x1000, 0x80000000, "test");

    bus.write32(0x80000000, 0xDEADBEEF);
    ZLB_EXPECT_EQ(bus.read32(0x80000000), 0xDEADBEEFu);
    bus.write16(0x80000004, 0xBEEF);
    ZLB_EXPECT_EQ(bus.read16(0x80000004), 0xBEEFu);
    bus.write8(0x80000006, 0x42);
    ZLB_EXPECT_EQ(bus.read8(0x80000006), 0x42u);
    bus.write64(0x80000008, 0x0123456789ABCDEFull);
    ZLB_EXPECT_EQ(bus.read64(0x80000008), 0x0123456789ABCDEFull);
}

ZLB_TEST(bus_unaligned_ram_access) {
    Bus bus;
    bus.add_ram("ram", 0x1000, 0x80000000, "test");
    bus.write32(0x80000001, 0x11223344);
    ZLB_EXPECT_EQ(bus.read32(0x80000001), 0x11223344u);
    ZLB_EXPECT_EQ(bus.read8(0x80000001), 0x44u);
    ZLB_EXPECT_EQ(bus.read8(0x80000004), 0x11u);
}

ZLB_TEST(bus_mmio_routing_and_naming) {
    Bus bus;
    bus.add_device(std::make_unique<TinyDevice>());

    ZLB_EXPECT_EQ(bus.read32(0xF0000000), 0x1234u);
    bus.write32(0xF0000000, 0xABCD);
    ZLB_EXPECT_EQ(bus.read32(0xF0000000), 0xABCDu);

    Device* device = bus.find_device(0xF0000000);
    ZLB_EXPECT_TRUE(device != nullptr);
    ZLB_EXPECT_TRUE(device->register_name(0xF0000000) != nullptr);
    ZLB_EXPECT_TRUE(std::string(device->register_name(0xF0000000)) == "CTRL");
    if (device) {
        u64 value = 0;
        ZLB_EXPECT_TRUE(device->peek_register("CTRL", value));
        ZLB_EXPECT_EQ(value, 0xABCDu);
    }
}

ZLB_TEST(bus_smallest_device_wins) {
    Bus bus;
    bus.add_device(std::make_unique<RegisterFile>("Big", 0xE0000000, 0x1000));
    auto small = std::make_unique<RegisterFile>("Small", 0xE0000100, 0x10);
    small->define(0xE0000100, "X", 0x55);
    bus.add_device(std::move(small));

    Device* device = bus.find_device(0xE0000100);
    ZLB_EXPECT_TRUE(device != nullptr);
    if (device) ZLB_EXPECT_TRUE(device->name() == "Small");
}

ZLB_TEST(bus_unmapped_reads) {
    Bus bus;
    bus.add_ram("ram", 0x1000, 0x80000000, "test");
    ZLB_EXPECT_FALSE(bus.is_mapped(0x90000000));
    ZLB_EXPECT_EQ(bus.read32(0x90000000), 0xFFFFFFFFu);
    ZLB_EXPECT_TRUE(bus.last_unmapped);
}

ZLB_TEST(bus_trace_records_mmio) {
    Bus bus;
    bus.add_device(std::make_unique<TinyDevice>());
    bus.context.core = "test";
    bus.context.pc = 0x1234;

    bus.trace.clear();
    bus.read32(0xF0000000);
    bus.write32(0xF0000000, 7);

    auto records = bus.trace.tail(8);
    ZLB_EXPECT_EQ(records.size(), static_cast<size_t>(2));
    if (records.size() == 2) {
        ZLB_EXPECT_TRUE(records[0].kind == AccessKind::Read);
        ZLB_EXPECT_EQ(records[0].address, 0xF0000000u);
        ZLB_EXPECT_EQ(records[0].pc, 0x1234u);
        ZLB_EXPECT_TRUE(records[1].kind == AccessKind::Write);
        ZLB_EXPECT_EQ(records[1].value, 7u);
    }
}

ZLB_TEST(bus_shared_alias) {
    Bus first;
    Bus second;
    std::vector<u8> shared(0x1000, 0);

    first.add_ram_alias("shared_a", 0x1F000000, 0x1000, shared.data(), "shared");
    second.add_ram_alias("shared_b", 0x1F000000, 0x1000, shared.data(), "shared");

    first.write32(0x1F000000, 0xCAFEBABE);
    ZLB_EXPECT_EQ(second.read32(0x1F000000), 0xCAFEBABEu);
    second.write8(0x1F000004, 0x77);
    ZLB_EXPECT_EQ(first.read8(0x1F000004), 0x77u);
}

ZLB_TEST(bus_load_and_ensure_ram) {
    Bus bus;
    std::vector<u8> blob = {1, 2, 3, 4, 5, 6, 7, 8};
    ZLB_EXPECT_TRUE(bus.load(0x00040000, blob.data(), blob.size(), "blob"));
    ZLB_EXPECT_TRUE(bus.is_ram(0x00040000, blob.size()));
    ZLB_EXPECT_EQ(bus.read8(0x00040000), 1u);
    ZLB_EXPECT_EQ(bus.read8(0x00040007), 8u);
}

ZLB_TEST(util_parse_numbers) {
    u64 value = 0;
    ZLB_EXPECT_TRUE(parse_u64("0x5C000", value));
    ZLB_EXPECT_EQ(value, 0x5C000u);
    ZLB_EXPECT_TRUE(parse_u64("$5C000", value));
    ZLB_EXPECT_EQ(value, 0x5C000u);
    ZLB_EXPECT_TRUE(parse_u64("1234", value));
    ZLB_EXPECT_EQ(value, 1234u);
    ZLB_EXPECT_TRUE(parse_u64("0b1011", value));
    ZLB_EXPECT_EQ(value, 11u);
    ZLB_EXPECT_FALSE(parse_u64("nonsense", value));
}

ZLB_TEST(util_bit_helpers) {
    ZLB_EXPECT_EQ(extract_bits<u32>(0xDEADBEEFu, 15, 8), 0xBEu);
    ZLB_EXPECT_EQ(set_bits<u32>(0, 7, 4, 0xA), 0xA0u);
    ZLB_EXPECT_EQ(sign_extend<u32>(0xFF, 8), 0xFFFFFFFFu);
    ZLB_EXPECT_TRUE(test_bit<u32>(0x80000000u, 31));
    ZLB_EXPECT_EQ(rotr<u32>(0x80000001u, 1), 0xC0000000u);
    ZLB_EXPECT_EQ(rotl<u32>(0x80000001u, 1), 0x00000003u);
}
