// zeliboba - unit tests for the modelled PowerVR SGX window (src/hw/soc/sgx.cpp).
//
// The device is the register interface the GPU driver will talk to (task part 2
// of the Live Area goal, docs/GPU.md).  The tests pin down the contract the rest
// of the model relies on: identification, the command-queue handshake, the
// completion events and the interrupt line.
#include "bus/bus.h"
#include "hw/soc/soc_internal.h"
#include "test_framework.h"

using namespace zlb;

namespace {

struct Fixture {
    std::unique_ptr<Bus> bus_owner = std::make_unique<Bus>();
    Bus& bus = *bus_owner;
    kermit::Sgx sgx{"SGX", kermit::kSgxBase, kermit::kSgxSize, bus};
    bool irq_seen = false;
    bool irq_level = false;

    Fixture() {
        bus.unmapped_reads_zero = true;
        bus.add_ram("dram", 0x100000, kermit::kDramBase, "sgx test dram");
        sgx.reset();
        sgx.set_irq_callback([this](u32 id, bool level) {
            (void)id;
            irq_seen = true;
            irq_level = level;
        });
    }

    void write(u32 offset, u32 value) { sgx.write(kermit::kSgxBase + offset, 4, value); }
    u32 read(u32 offset) { return static_cast<u32>(sgx.read(kermit::kSgxBase + offset, 4)); }
};

}  // namespace

ZLB_TEST(sgx_window_identifies_itself) {
    Fixture f;
    ZLB_EXPECT_EQ(f.read(kermit::kSgxCoreId), kermit::kSgxCoreIdValue);
    ZLB_EXPECT_EQ(f.read(kermit::kSgxCoreRevision), kermit::kSgxCoreRevisionValue);
    ZLB_EXPECT_EQ(f.read(kermit::kSgxCoreStatus), 0u);
}

ZLB_TEST(sgx_queue_registers_read_back) {
    Fixture f;
    f.write(kermit::kSgxQueueBase, 0x40300000u);
    f.write(kermit::kSgxQueueSize, 0x1000u);
    f.write(kermit::kSgxQueueWrite, 0x40u);
    ZLB_EXPECT_EQ(f.sgx.queue_base(), 0x40300000u);
    ZLB_EXPECT_EQ(f.sgx.queue_size(), 0x1000u);
    ZLB_EXPECT_EQ(f.sgx.queue_write_offset(), 0x40u);
    ZLB_EXPECT_EQ(f.read(kermit::kSgxQueueBase), 0x40300000u);
    ZLB_EXPECT_EQ(f.read(kermit::kSgxQueueSize), 0x1000u);
}

ZLB_TEST(sgx_kick_reports_completion_and_counts_commands) {
    Fixture f;
    f.write(kermit::kSgxQueueBase, 0x40000000u);
    f.write(kermit::kSgxQueueSize, 0x1000u);
    f.write(kermit::kSgxQueueWrite, 0x30u);   // three 16-byte commands
    ZLB_EXPECT_EQ(f.sgx.kicks(), 0u);

    f.write(kermit::kSgxQueueControl, 1u);

    ZLB_EXPECT_EQ(f.sgx.kicks(), 1u);
    ZLB_EXPECT_EQ(f.sgx.commands(), 3u);
    ZLB_EXPECT_EQ(f.sgx.queue_read_offset(), 0x30u);
    // The completion events are raised and visible in EVENT_STATUS.
    ZLB_EXPECT_TRUE((f.read(kermit::kSgxEventStatus) & kermit::kSgxEventTa) != 0u);
    ZLB_EXPECT_TRUE((f.read(kermit::kSgxEventStatus) & kermit::kSgxEvent3d) != 0u);
}

ZLB_TEST(sgx_kick_raises_the_irq_line_only_when_enabled) {
    Fixture f;
    f.write(kermit::kSgxQueueBase, 0x40000000u);
    f.write(kermit::kSgxQueueSize, 0x1000u);
    f.write(kermit::kSgxQueueWrite, 0x10u);

    // Without the enable mask a kick stays silent.
    f.write(kermit::kSgxQueueControl, 1u);
    ZLB_EXPECT_FALSE(f.irq_level);
    ZLB_EXPECT_EQ(f.read(kermit::kSgxIrqStatus) & kermit::kSgxEventTa, kermit::kSgxEventTa);

    // Enable the completion event: the already pending status must assert.
    f.irq_seen = false;
    f.write(kermit::kSgxEventEnable, kermit::kSgxEventTa);
    ZLB_EXPECT_TRUE(f.irq_seen);
    ZLB_EXPECT_TRUE(f.irq_level);
    ZLB_EXPECT_TRUE(f.sgx.irq_line());

    // Clearing the status drops the line again.
    f.write(kermit::kSgxIrqClear, kermit::kSgxEventTa | kermit::kSgxEvent3d);
    ZLB_EXPECT_FALSE(f.irq_level);
    ZLB_EXPECT_FALSE(f.sgx.irq_line());
    ZLB_EXPECT_EQ(f.read(kermit::kSgxIrqStatus), 0u);
}

ZLB_TEST(sgx_kick_reads_the_command_queue_from_guest_memory) {
    Fixture f;
    // Put a recognisable command stream in DRAM: four 16-byte command units.
    const u32 queue = kermit::kDramBase + 0x8000u;
    const u32 words[4] = {0xDEADBEEFu, 0x00000011u, 0x12345678u, 0xA5A5A5A5u};
    for (u32 i = 0; i < 4u; ++i) {
        f.bus.write32(queue + i * 4u, words[i]);
    }

    f.write(kermit::kSgxQueueBase, queue);
    f.write(kermit::kSgxQueueSize, 0x1000u);
    f.write(kermit::kSgxQueueWrite, 0x10u);   // one 16-byte command unit published
    f.write(kermit::kSgxQueueControl, 1u);

    ZLB_EXPECT_EQ(f.sgx.commands(), 1u);
    ZLB_EXPECT_EQ(f.sgx.queue_bytes(), 0x10u);
    ZLB_EXPECT_EQ(f.sgx.last_words().size(), 4u);
    ZLB_EXPECT_EQ(f.sgx.last_words()[0], 0xDEADBEEFu);
    ZLB_EXPECT_EQ(f.sgx.last_words()[2], 0x12345678u);
    ZLB_EXPECT_EQ(f.sgx.queue_read_offset(), 0x10u);
}

ZLB_TEST(sgx_mmu_translates_through_the_modelled_page_table) {
    Fixture f;
    const u32 table = kermit::kDramBase + 0x4000u;
    const u32 va = 0x00123000u;
    const u32 pa_page = 0x00555000u;

    // Disabled (or without a table) every lookup faults.
    ZLB_EXPECT_EQ(f.sgx.translate(va), kermit::kSgxMmuFault);
    ZLB_EXPECT_EQ(f.sgx.faults(), 1u);

    // Install one valid entry: {page, flags} indexed by va >> 12.
    f.bus.write32(table + (va >> 12) * 8u, pa_page);
    f.bus.write32(table + (va >> 12) * 8u + 4u, kermit::kSgxMmuEntryValid);
    f.write(kermit::kSgxMmuDirBase, table);
    f.write(kermit::kSgxMmuControl, 1u);

    ZLB_EXPECT_EQ(f.sgx.translate(va + 0x234u), pa_page + 0x234u);
    ZLB_EXPECT_EQ(f.sgx.faults(), 1u);   // still only the first lookup faulted

    // An unmapped page in the same table still faults.
    ZLB_EXPECT_EQ(f.sgx.translate(va + 0x2000u), kermit::kSgxMmuFault);
    ZLB_EXPECT_EQ(f.sgx.faults(), 2u);

    // Invalidation is counted and STATUS reports {faults, translations}.
    f.write(kermit::kSgxMmuInvalidate, 1u);
    ZLB_EXPECT_EQ(f.sgx.invalidations(), 1u);
    const u32 status = f.read(kermit::kSgxMmuStatus);
    ZLB_EXPECT_EQ(status & 0xFFFFu, 2u);
    ZLB_EXPECT_EQ(status >> 16, static_cast<u32>(f.sgx.translations()));
}

ZLB_TEST(sgx_reset_returns_the_window_to_power_on) {
    Fixture f;
    f.write(kermit::kSgxQueueBase, 0x40000000u);
    f.write(kermit::kSgxQueueSize, 0x1000u);
    f.write(kermit::kSgxEventEnable, kermit::kSgxEventTa);
    f.write(kermit::kSgxQueueWrite, 0x20u);
    f.write(kermit::kSgxQueueControl, 1u);
    ZLB_EXPECT_TRUE(f.sgx.kicks() == 1u);

    f.sgx.reset();

    ZLB_EXPECT_EQ(f.sgx.kicks(), 0u);
    ZLB_EXPECT_EQ(f.sgx.commands(), 0u);
    ZLB_EXPECT_EQ(f.read(kermit::kSgxEventStatus), 0u);
    ZLB_EXPECT_EQ(f.read(kermit::kSgxIrqStatus), 0u);
    ZLB_EXPECT_EQ(f.read(kermit::kSgxQueueBase), 0u);
    ZLB_EXPECT_FALSE(f.sgx.irq_line());
}
