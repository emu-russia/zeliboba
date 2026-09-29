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
    kermit::Sgx sgx{"SGX", kermit::kSgxBase, kermit::kSgxSize};
    bool irq_seen = false;
    bool irq_level = false;

    Fixture() {
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
