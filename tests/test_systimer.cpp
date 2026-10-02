// Reached FW1.04 LT5 time provider and WT7 Secure-delay hardware contracts.
#include <limits>
#include <memory>
#include <string>

#include "bus/bus.h"
#include "cpu/arm/arm_core.h"
#include "hw/soc/soc_internal.h"
#include "test_framework.h"

using namespace zlb;

namespace {
struct TimerFixture {
    std::unique_ptr<Bus> owner = std::make_unique<Bus>();
    Bus& bus = *owner;
    KermitBlock soc{bus, nullptr};
    TimerFixture() { soc.install(); soc.reset(); }
    u32 lt(u32 offset) { return bus.read32(kermit::kLt5Base + offset); }
    u32 wt(u32 offset) { return bus.read32(kermit::kWt7Base + offset); }
    void lt(u32 offset, u32 value) { bus.write32(kermit::kLt5Base + offset, value); }
    void wt(u32 offset, u32 value) { bus.write32(kermit::kWt7Base + offset, value); }
    Device& lt_device() { return *bus.find_device(kermit::kLt5Base); }
    Device& wt_device() { return *bus.find_device(kermit::kWt7Base); }
    void ticks(u64 count) { soc.tick(kermit::cycles_from_periph_ticks(count)); }
    void native_lt_start() {
        // Actual KBL4002158E..1598 stopped/start sequence.
        lt(0x1C, 0x2F345008); lt(0, 0); lt(4, 0);
        lt(8, 0xFFFFFFFF); lt(0xC, 0xFFFFFFFF); lt(0x1C, 0x2F34500D);
    }
    void native_wt_start(u32 deadline) {
        // Actual first Secure delay3BF1EA..214.
        wt(0, deadline); wt(4, 0); wt(8, 0xDD00000D);
    }
};

void write_hex(Bus& bus, u32 address, const char* hex) {
    auto digit = [](char c) -> u8 { return static_cast<u8>(c <= '9' ? c - '0' : c - 'a' + 10); };
    for (unsigned i = 0; hex[i] && hex[i + 1]; i += 2)
        bus.write8(address + i / 2, static_cast<u8>((digit(hex[i]) << 4) | digit(hex[i + 1])));
}

void run_to(ArmCore& cpu, u32 returned, unsigned maximum = 256) {
    unsigned steps = 0;
    while (cpu.get_pc() != returned && steps++ < maximum && !cpu.halted)
        ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_EQ(cpu.get_pc(), returned);
}

void load_native_provider(TimerFixture& fx) {
    fx.bus.add_ram("NativeTimeText", 0x2000, 0x81017000, "unchanged FW1.04 ThreadMgr providers");
    fx.bus.add_ram("NativeTimeState", 0x5000, 0x8102E000, "native global/pointer layout");
    // Supplied threadmgr.elf PT_LOAD fileA0, linked81017D34..6E. No host hook.
    write_hex(fx.bus, 0x81017D34,
        "4ef20001c8f20211086800f50a535b6d596818685a689142fad10b46002141ea00001946704700bf");
    write_hex(fx.bus, 0x81017D5C, "4ef20002c8f20212116801f50a50436d18687047");
    fx.bus.write32(0x8102E000, 0x8102F000);
    fx.bus.write32(0x8102F000 + 0x2280 + 0x54, kermit::kLt5Base);
}
} // namespace

ZLB_TEST(systimer_native_kbl_start_and_time_providers_use_elapsed_ticks_and_rollover) {
    TimerFixture fx;
    fx.native_lt_start();
    ZLB_EXPECT_EQ(fx.lt(0x1C), 0x2F34500Du);
    for (unsigned read = 0; read < 20; ++read) ZLB_EXPECT_EQ(fx.lt(0), 0u);
    fx.soc.tick(kermit::cycles_from_periph_ticks(1) - 1);
    ZLB_EXPECT_EQ(fx.lt(0), 0u);
    fx.soc.tick(1);
    ZLB_EXPECT_EQ(fx.lt(0), 1u);
    fx.ticks(12345);
    load_native_provider(fx);
    ArmCore cpu(fx.bus);
    cpu.reset(0x81017D5D);
    cpu.r[14] = 0x81017E01;
    run_to(cpu, 0x81017E00);
    ZLB_EXPECT_EQ(cpu.r[0], 12346u); // unchanged native Low returns raw microseconds

    fx.lt(0, 0xFFFFFFFE); fx.lt(4, 0x12345);
    fx.ticks(4);
    cpu.reset(0x81017D35); cpu.r[14] = 0x81017E01;
    run_to(cpu, 0x81017E00);
    ZLB_EXPECT_EQ(cpu.r[0], 2u);
    ZLB_EXPECT_EQ(cpu.r[1], 0x12346u); // unchanged native high/low/high protocol
    ZLB_EXPECT_EQ(fx.wt(4), 0u); // independent timer has not been started
    fx.lt(0, 0xFFFFFFFF); fx.lt(4, 0x12345);
    cpu.reset(0x81017D35); cpu.r[14] = 0x81017E01;
    for (unsigned step = 0; step < 6; ++step) ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_EQ(cpu.get_pc(), 0x81017D46u); // first high word read, before low
    fx.ticks(1); // genuine high-word rollover between provider reads
    for (unsigned step = 0; step < 4; ++step) ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_EQ(cpu.get_pc(), 0x81017D44u); // unchanged branch retries
    run_to(cpu, 0x81017E00);
    ZLB_EXPECT_EQ(cpu.r[0], 0u); ZLB_EXPECT_EQ(cpu.r[1], 0x12346u);
}

ZLB_TEST(systimer_prescaler_phase_matches_known_ratios_and_large_batches_portably) {
    TimerFixture fx;
    // Source3/(255+1): 48MHz/256 = 3 counts per16 microseconds.
    fx.lt(0x1C, 0xFF34500D); fx.lt(8, 0xFFFFFFFF); fx.lt(0xC, 0xFFFFFFFF);
    for (unsigned i = 0; i < 15; ++i) fx.lt_device().tick(1);
    ZLB_EXPECT_EQ(fx.lt(0), 2u);
    fx.lt_device().tick(1);
    ZLB_EXPECT_EQ(fx.lt(0), 3u);
    // Modeled222MHz/(255+1): exactly111 counts per128 microseconds.
    fx.wt(0, 0xFFFFFFFF); fx.wt(8, 0xFF00000D);
    for (unsigned i = 0; i < 1000; ++i) fx.wt_device().tick(1);
    ZLB_EXPECT_EQ(fx.wt(4), 867u);
    fx.wt_device().reset(); fx.wt(0, 0xFFFFFFFF); fx.wt(8, 0xFF00000D);
    fx.wt_device().tick(1000);
    ZLB_EXPECT_EQ(fx.wt(4), 867u);
    fx.wt_device().tick(24);
    ZLB_EXPECT_EQ(fx.wt(4), 888u); // eight complete111-count intervals

    fx.lt_device().reset(); fx.lt(8, 17); fx.lt(0x1C, 0x0034500D);
    fx.lt_device().tick(std::numeric_limits<u64>::max()); //48*(2^64-1) counts
    ZLB_EXPECT_EQ(fx.lt(0), 0xFFFFFFD0u);
    ZLB_EXPECT_EQ(fx.lt(4), 0xFFFFFFFFu);
    ZLB_EXPECT_EQ(fx.lt(0x18), 2u); // a full counter span cannot miss comparison
    fx.wt_device().reset(); fx.wt(0, 19); fx.wt(8, 0x0000000D);
    fx.wt_device().tick(std::numeric_limits<u64>::max()); //222*(2^64-1) counts
    ZLB_EXPECT_EQ(fx.wt(4), 0xFFFFFF22u);
    ZLB_EXPECT_EQ(fx.wt(0x14), 2u);
}

ZLB_TEST(systimer_comparison_latches_before_wrap_and_rearming_is_explicit) {
    TimerFixture fx;
    fx.wt(0, 0xFFFFFFFF); fx.wt(4, 0xFFFFFFFE); fx.wt(8, 0xDD00000D);
    fx.wt_device().tick(5);
    ZLB_EXPECT_EQ(fx.wt(4), 3u);
    ZLB_EXPECT_EQ(fx.wt(0x14), 2u); // batch crossed target before wrapping
    fx.wt(0x14, 3); fx.wt_device().tick(10);
    ZLB_EXPECT_EQ(fx.wt(0x14), 0u); // ACK alone does not rearm
    fx.wt(0, 13); // equal current target, IRQ waits for genuine count
    ZLB_EXPECT_EQ(fx.wt(4), 13u);
    ZLB_EXPECT_EQ(fx.wt(0x14), 0u);
    fx.wt_device().tick(1);
    ZLB_EXPECT_EQ(fx.wt(4), 14u);
    ZLB_EXPECT_EQ(fx.wt(0x14), 2u);
    fx.wt(0x14, 2); fx.wt(8, 0xDD00000D); fx.wt_device().tick(1);
    ZLB_EXPECT_EQ(fx.wt(0x14), 0u); // identical active config cannot rearm
    fx.wt(0, 13); fx.wt_device().tick(1);
    ZLB_EXPECT_EQ(fx.wt(0x14), 2u); // identical deadline explicitly rearms overdue
    fx.wt(0x14, 2); fx.wt(4, fx.wt(4)); fx.wt_device().tick(1);
    ZLB_EXPECT_EQ(fx.wt(0x14), 2u); // identical counter write also rearms
    fx.wt(0x14, 2); fx.wt(8, 0); fx.wt(8, 0xDD00000D);
    ZLB_EXPECT_EQ(fx.wt(0x14), 0u);
    fx.wt_device().tick(1);
    ZLB_EXPECT_EQ(fx.wt(0x14), 2u); // native stop/start rearms, ACK alone does not

    fx.native_lt_start(); fx.lt(0, 0xFFFFFFFD); fx.lt(4, 0xFFFFFFFF);
    fx.lt(8, 0xFFFFFFFE); fx.lt(0xC, 0xFFFFFFFF);
    fx.lt_device().tick(5);
    ZLB_EXPECT_EQ(fx.lt(0), 2u); ZLB_EXPECT_EQ(fx.lt(4), 0u);
    ZLB_EXPECT_EQ(fx.lt(0x18), 2u); //64-bit crossing before final wrapped counter
}

ZLB_TEST(systimer_status_is_hardware_owned_and_acknowledges_only_written_lanes) {
    TimerFixture fx;
    fx.native_lt_start(); fx.lt(8, 2); fx.lt(0xC, 0);
    fx.native_wt_start(2); fx.ticks(2);
    for (auto item : {std::pair<u32,u32>{kermit::kLt5Base, 0x18}, {kermit::kWt7Base, 0x14}}) {
        const u32 status = item.first + item.second;
        ZLB_EXPECT_EQ(fx.bus.read32(status), 2u);
        fx.bus.write32(status, 0); fx.bus.write8(status + 1, 0xFF); fx.bus.write16(status + 2, 0xFFFF);
        ZLB_EXPECT_EQ(fx.bus.read32(status), 2u);
        fx.bus.write8(status, 1); //unmodeled overflow ACK does not clear compare
        ZLB_EXPECT_EQ(fx.bus.read32(status), 2u);
        fx.bus.write8(status, 2);
        ZLB_EXPECT_EQ(fx.bus.read32(status), 0u);
        fx.bus.write32(status, 0xFFFFFFFF);
        ZLB_EXPECT_EQ(fx.bus.read32(status), 0u); // cannot inject events through writes
    }
    fx.ticks(10);
    ZLB_EXPECT_EQ(fx.lt(0x18), 0u); ZLB_EXPECT_EQ(fx.wt(0x14), 0u);
}

ZLB_TEST(systimer_control_lanes_unknown_modes_reset_and_stopped_output_are_bounded) {
    TimerFixture fx;
    fx.bus.write8(kermit::kWt7Base + 3, 0x12);
    fx.bus.write16(kermit::kWt7Base, 0x3456);
    ZLB_EXPECT_EQ(fx.wt(0), 0x12003456u);
    fx.bus.write8(kermit::kWt7Base + 9, 0);
    fx.bus.write16(kermit::kWt7Base + 10, 0xDD00);
    fx.bus.write8(kermit::kWt7Base + 8, 0x0D);
    ZLB_EXPECT_EQ(fx.wt(8), 0xDD00000Du);
    fx.wt(0, 1); fx.wt_device().tick(1);
    ZLB_EXPECT_EQ(fx.wt(0x14), 2u);
    fx.wt(8, 0xDD00000C); // inferred stopped compare profile retains pending output
    fx.wt_device().tick(10);
    ZLB_EXPECT_EQ(fx.wt(4), 1u); ZLB_EXPECT_EQ(fx.wt(0x14), 2u);
    fx.wt(8, 0); // ISR stops count/output; event awaits ACK
    ZLB_EXPECT_EQ(fx.wt(0x14), 2u);
    fx.wt(0x14, 3);
    for (u32 config : {0xDD00001Du, 0xDD10000Du, 0xDD00080Du, 0xDD00400Du}) {
        fx.wt(8, config); fx.wt_device().tick(100);
        ZLB_EXPECT_EQ(fx.wt(8), config); ZLB_EXPECT_EQ(fx.wt(4), 1u);
        ZLB_EXPECT_EQ(fx.wt(0x14), 0u);
        u64 unsupported = 0; ZLB_EXPECT_TRUE(fx.wt_device().peek_register("UNSUPPORTED", unsupported));
        ZLB_EXPECT_EQ(unsupported, 1u);
    }
    fx.wt(0xC, 0xCAFEBABE); fx.wt(0x10, 0x10203040);
    fx.bus.write32(kermit::kWt7Base + 0x100, 0x12345678);
    ZLB_EXPECT_EQ(fx.bus.read32(kermit::kWt7Base + 0x100), 0xFFFFFFFFu);
    ZLB_EXPECT_EQ(fx.bus.read32(kermit::kWt7Base + 0x1C), 0xFFFFFFFFu);
    fx.native_lt_start(); fx.ticks(9);
    fx.wt_device().reset();
    ZLB_EXPECT_EQ(fx.lt(0), 9u);
    for (u32 offset : {0u, 4u, 8u, 0xCu, 0x10u, 0x14u}) ZLB_EXPECT_EQ(fx.wt(offset), 0u);
    fx.soc.reset();
    ZLB_EXPECT_EQ(fx.lt(0), 0u); ZLB_EXPECT_EQ(fx.lt(0x1C), 0u);
    ZLB_EXPECT_TRUE(fx.wt_device().summary().find("comparisons=0 unsupported=0") != std::string::npos);
}

ZLB_TEST(systimer_physical_irq135_native_handler_rearms_equality_and_unlocks_after_elapsed_time) {
    TimerFixture fx;
    fx.bus.add_ram("NativeSecureIntrMgr", 0x2000, 0x3BE000, "original WT7 IRQ135 handler/imports");
    fx.bus.add_ram("NativeSecureLocks", 0x1000, 0xCD000, "original FW1.04 A32 lock helpers");
    fx.bus.add_ram("NativeSecureIrqRecords", 0x8000, 0x540000, "native IntrMgr interrupt records");
    fx.bus.add_ram("NativeSecureState", 0x2000, 0x548000, "native delay globals/stacks");
    fx.bus.add_ram("TimerIrqVector", 0x1000, 0x80000000, "test GIC dispatcher and exception return");
    // Actual cached3BE000 image, exact3BF000..147, including native global literals.
    write_hex(fx.bus, 0x3BF000,
        "2de9f84348f20074c0f2540448f23075c0f2540504f1180000f044eb2b6803224ff00008d3f80890c3f808805a615f68266816f0010605d0a068874264d8a668c4f80480226848f20073c0f2540312f0020f07d0d868874264d8002e34d10121e6686160206848f20073c0f2540310f0040f06d019698f4232d84ebb022326696360206848f20073c0f2540310f0080f06d05a6997422dd8f6b9032066696060226848f20073c0f2540332b15868fff797ff29680e60c1f808901f4800f00aeb4ff0ff30bde8f883db689e42c7d8c9e71a699642d2d8d4e75b69b342e0d2dce71968164821f0040200211a60196100f0f2eac6e71968124821f0080200211a60596100f0e8eacbe7236848f22070c0f25400464623f001012160c4f8088000f0daea8fe71a68074822f0020100221960da6000f0d0ea95e718875400288754002c87540024875400");
    write_hex(fx.bus, 0x3BF6A4, "14c00de30cc040e31cff2fe100000000");
    write_hex(fx.bus, 0x3BF6D4, "38c00de30cc040e31cff2fe100000000");
    // Genuine sysmem.elf fileD114..D14F, including SEV E320F004.
    write_hex(fx.bus, 0xCD014,
        "0120a0e39f1f90e1000031e302f02013921f800100003103f9ffff1a5ff07ff51eff2fe10010a0e35ff07ff5001080e54ff07ff504f020e31eff2fe1");
    write_hex(fx.bus, 0x3BEC2C, "b0f5807f2de9f041044605d947f20210c8f20200bde8f08140f200068701c0f25406f3592bb947f20a10c8f20200bde8f08121b311f4702f1ed111f00f0f17d10d0c07f11008b044f619404600f034ed014648f2c460c0f25400f58002684046141984f8005800f010ed0020bde8f08100290adb0d46e4e711f00f0fe0d047f20510c8f20200bde8f08147f20510c8f20200bde8f08100bf");
    write_hex(fx.bus, 0x3BEFE0, "032808b59dbf4ff60043c0f23b0353f8201000218720fff719fe00eae07008bd");
    write_hex(fx.bus, 0x3BFC00, "01000000020000000400000008000000");
    write_hex(fx.bus, 0xCD050, "00300fe1c01083e301f021e10120a0e39f1f90e1000031e303f0211102f02013c010831301f02111921f800100003103f6ffff1a5ff07ff5c00003e21eff2fe100300fe1c02083e302f021e10120a0e39f1f90e1000031e3921f80015ff07ff5000031e3c00003020000e01303f021111eff2fe10020a0e35ff07ff5002080e54ff07ff504f020e300200fe1c01001e2c020c2e3012082e102f021e11eff2fe1");
    write_hex(fx.bus, 0x3BF6B4, "c4c00de30cc040e31cff2fe100000000");
    write_hex(fx.bus, 0x3BF6E4, "50c00de30cc040e31cff2fe100000000");
    const u32 icc = kermit::kScuBase + kermit::kIccOffset;
    const u32 dist = kermit::kScuBase + kermit::kGicDistOffset;
    // Test-only architectural IRQ dispatcher: real IAR/EOIR, native handler,
    // saved SPSR/registers and SUBS exception return. No firmware service hook.
    const u32 dispatcher[] = {
        0xE92D500F, 0xE14F0000, 0xE92D0001, 0xE59FC028,
        0xE59C0000, 0xE92D0001, 0xE59FC020, 0xE12FFF3C,
        0xE8BD0001, 0xE59FC018, 0xE58C0000, 0xE8BD0001,
        0xE16FF000, 0xE8BD500F, 0xE25EF004, icc + 0xC, 0x3BF001, icc + 0x10,
    };
    for (unsigned i = 0; i < sizeof(dispatcher) / sizeof(u32); ++i)
        fx.bus.write32(0x80000100 + i * 4, dispatcher[i]);
    fx.bus.write32(0x80000018, 0xE59FF000); fx.bus.write32(0x80000020, 0x80000100);
    fx.bus.write32(0x540000 + 135 * 0x40, 0x3BF001);
    fx.bus.write32(0x5486C4, dist);
    fx.bus.write32(0x548700, 4); fx.bus.write32(0x548704, 2);
    fx.bus.write32(0x548710, 0x30D40); fx.bus.write32(0x548728, 1);
    fx.bus.write32(0x548730, kermit::kWt7Base);
    ArmCore cpu(fx.bus);
    cpu.core_id_ = 2;
    cpu.reset(0xCD014);
    cpu.set_register("VBAR", 0x80000000);
    cpu.set_register("CPSR", 0x92); cpu.r[13] = 0x549000; // IRQ stack
    cpu.set_register("CPSR", 0x13); cpu.r[13] = 0x548F00; // Secure SVC, IRQ unmasked
    cpu.r[0] = 0x548728; cpu.r[14] = 0x80000200;
    // Ordinary architectural board wiring: actual IRQ/SEV can leave WFE.
    // These hooks never inspect or write either guest lock.
    auto wake = [&cpu]() {
        cpu.wfe_waiting_ = false;
        if (cpu.halt_reason == "wfe") { cpu.halted = false; cpu.halt_reason.clear(); }
    };
    cpu.irq_hook = wake;
    cpu.sev_hook = [&cpu, &wake]() { cpu.event_pending_ = true; wake(); };
    kermit_set_cpu(fx.bus, &cpu, 2);
    fx.bus.context.core_id = 2; fx.bus.context.nonsecure = false;
    fx.bus.write32(dist, 1); fx.bus.write32(icc + 4, 0xFF); fx.bus.write32(icc, 1);
    fx.bus.write32(dist + 0x100 + (135 / 32) * 4, 1u << (135 % 32));
    fx.bus.write8(dist + 0x400 + 135, 0x80); fx.bus.write8(dist + 0x800 + 135, 4);
    fx.native_wt_start(0x30D40);
    for (unsigned i = 0; i < 4; ++i) ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_TRUE(cpu.halted); ZLB_EXPECT_EQ(cpu.get_pc(), 0xCD024u);
    fx.ticks(0x30D40);
    ZLB_EXPECT_EQ(fx.wt(4), 0x30D40u);
    ZLB_EXPECT_TRUE(cpu.interrupt_pending());
    // At equality the real handler must keep the per-core lock and rearm.
    unsigned steps = 0;
    while (!(cpu.halted && !cpu.interrupt_pending()) && steps++ < 512)
        ZLB_EXPECT_FALSE(cpu.step().faulted);
    ZLB_EXPECT_TRUE(cpu.halted);
    ZLB_EXPECT_EQ(fx.bus.read32(0x548728), 1u);
    ZLB_EXPECT_EQ(fx.bus.read32(0x548700), 4u);
    ZLB_EXPECT_EQ(fx.wt(0x14), 0u); ZLB_EXPECT_EQ(fx.wt(8), 0xDD00000Du);
    fx.ticks(1);
    ZLB_EXPECT_EQ(fx.wt(4), 0x30D41u);
    ZLB_EXPECT_TRUE(cpu.interrupt_pending());
    run_to(cpu, 0x80000200, 512); // genuine wait helper returns after native unlock
    ZLB_EXPECT_EQ(fx.bus.read32(0x548700), 0u);
    // The IRQ genuinely cleared the lock; the resumed LDREX/STREX then reacquired it.
    ZLB_EXPECT_EQ(fx.bus.read32(0x548728), 1u);
    ZLB_EXPECT_EQ(fx.bus.read32(0x548710), 0u);
    ZLB_EXPECT_EQ(fx.wt(0x14), 0u); ZLB_EXPECT_EQ(fx.wt(8), 0u);
    ZLB_EXPECT_EQ(fx.soc.pending_irq_count(), 0u);
    ZLB_EXPECT_EQ(cpu.mode(), arm::kModeSupervisor);
    ZLB_EXPECT_TRUE(fx.wt_device().summary().find("comparisons=2") != std::string::npos);
    kermit_set_cpu(fx.bus, nullptr, 2);
}

ZLB_TEST(systimer_physical_irq141_target_mask_and_stopped_profile_preserve_pending_event) {
    TimerFixture fx;
    ArmCore cpu0(fx.bus), cpu2(fx.bus);
    cpu0.core_id_ = 0; cpu2.core_id_ = 2;
    cpu0.reset(); cpu2.reset();
    cpu0.set_register("CPSR", 0x13); cpu2.set_register("CPSR", 0x13);
    kermit_set_cpu(fx.bus, &cpu0, 0); kermit_set_cpu(fx.bus, &cpu2, 2);
    const u32 dist = kermit::kScuBase + kermit::kGicDistOffset;
    const u32 icc = kermit::kScuBase + kermit::kIccOffset;
    fx.native_lt_start(); fx.lt(8, 1); fx.lt(0xC, 0); fx.ticks(1);
    ZLB_EXPECT_FALSE(cpu0.interrupt_pending()); ZLB_EXPECT_FALSE(cpu2.interrupt_pending());
    ZLB_EXPECT_EQ(fx.lt(0x18), 2u); // deadline latches even before GIC enable
    fx.bus.context.core_id = 2;
    fx.bus.write32(dist, 1); fx.bus.write32(icc + 4, 0xFF); fx.bus.write32(icc, 1);
    fx.bus.write32(dist + 0x100 + (141 / 32) * 4, 1u << (141 % 32));
    fx.bus.write8(dist + 0x400 + 141, 0x80); fx.bus.write8(dist + 0x800 + 141, 4);
    ZLB_EXPECT_FALSE(cpu0.interrupt_pending()); ZLB_EXPECT_TRUE(cpu2.interrupt_pending());
    ZLB_EXPECT_EQ(fx.bus.read32(icc + 0xC), 141u);
    // Lower output pattern is an explicit unmeasured model policy: stopped C
    // retains compare output; stopped8 drops it without clearing the event.
    fx.lt(0x1C, 0x2F34500C);
    ZLB_EXPECT_TRUE(fx.lt_device().summary().find("irq141=asserted") != std::string::npos);
    fx.ticks(10); ZLB_EXPECT_EQ(fx.lt(0), 1u);
    fx.lt(0x1C, 0x2F345008);
    ZLB_EXPECT_TRUE(fx.lt_device().summary().find("irq141=idle") != std::string::npos);
    ZLB_EXPECT_EQ(fx.lt(0x18), 2u);
    fx.bus.write32(icc + 0x10, 141);
    ZLB_EXPECT_FALSE(cpu2.interrupt_pending());
    fx.lt(0x1C, 0x2F34500C);
    ZLB_EXPECT_TRUE(cpu2.interrupt_pending());
    fx.lt(0x18, 2);
    ZLB_EXPECT_FALSE(cpu2.interrupt_pending());
    ZLB_EXPECT_EQ(fx.bus.read32(icc + 0xC), 1023u);
    fx.soc.reset();
    ZLB_EXPECT_FALSE(cpu0.interrupt_pending()); ZLB_EXPECT_FALSE(cpu2.interrupt_pending());
    kermit_set_cpu(fx.bus, nullptr, 0); kermit_set_cpu(fx.bus, nullptr, 2);
}
