#include "test_framework.h"

#include <array>
#include "bus/device.h"
#include "cpu/arm/arm_core.h"
#include "debug/debugger.h"
#include "hw/soc.h"

using namespace zlb;

namespace {

Vita& debugger_test_vita() {
    // Keep the debugger alive for its process-wide log sink. No firmware or
    // eMMC attachment is needed to inspect the installed board devices.
    static Vita vita;
    static bool built = false;
    if (!built) {
        VitaConfig config;
        config.rebuild_emmc = false;
        vita.build(config);
        // This shorter device name is encountered on the CMeP bus first.
        // Selecting it would silently return the wrong CPU register value.
        auto prefix = std::make_unique<RegisterFile>("Kermit", 0xF0000000u, 4);
        prefix->define(0, "GIC.CPU3.ICCICR", 0xAAu);
        vita.cmep_bus().add_device(std::move(prefix));
        built = true;
    }
    return vita;
}

Debugger& debugger_test_instance() {
    static Debugger debugger(debugger_test_vita());
    return debugger;
}

void configure_inspection_map(ArmCore& cpu, u32 l1, u32 l2, u32 va, u32 pa) {
    Bus& bus = *cpu.bus;
    bus.memset_bytes(l1, 0, 0x4000u);
    bus.memset_bytes(l2, 0, 0x400u);
    bus.write32(l1 + (va >> 20) * 4u, l2 | 1u);
    bus.write32(l2 + ((va >> 12) & 0xFFu) * 4u, pa | 0x3Eu);
    cpu.mmu.ttbr0 = l1;
    cpu.mmu.ttbcr = 0;
    cpu.mmu.dacr = 1;
    cpu.mmu.sctlr = 1;
}

struct InspectionState {
    ArmMmu mmu;
    BusStats stats;
    BusContext context;
    u64 trace_total;
    bool last_unmapped;
    std::array<u32, 16> r{};
    u32 cpsr;
    u32 scr;
    bool thumb;
    bool nonsecure;
    u64 instructions;
    u64 exceptions;
    bool halted;

    explicit InspectionState(const ArmCore& cpu)
        : mmu(cpu.inspection_mmu()), stats(cpu.bus->stats), context(cpu.bus->context),
          trace_total(cpu.bus->trace.total()), last_unmapped(cpu.bus->last_unmapped),
          cpsr(cpu.cpsr), scr(cpu.scr), thumb(cpu.thumb), nonsecure(cpu.ns_),
          instructions(cpu.instructions), exceptions(cpu.exception_count), halted(cpu.halted) {
        for (int i = 0; i < 16; ++i) r[i] = cpu.r[i];
    }

    void expect_unchanged(const ArmCore& cpu) const {
        const ArmMmu& live = cpu.inspection_mmu();
        ZLB_EXPECT_EQ(live.walks, mmu.walks);
        ZLB_EXPECT_EQ(live.total_faults, mmu.total_faults);
        ZLB_EXPECT_EQ(live.fault_count, mmu.fault_count);
        ZLB_EXPECT_EQ(live.last_walk.va, mmu.last_walk.va);
        ZLB_EXPECT_EQ(live.last_walk.l1_addr, mmu.last_walk.l1_addr);
        ZLB_EXPECT_EQ(live.last_walk.l2_addr, mmu.last_walk.l2_addr);
        ZLB_EXPECT_EQ(live.last_walk.repeats, mmu.last_walk.repeats);
        for (int i = 0; i < ArmMmu::kFaultLogSize; ++i) {
            ZLB_EXPECT_EQ(live.faults[i].va, mmu.faults[i].va);
            ZLB_EXPECT_EQ(live.faults[i].repeats, mmu.faults[i].repeats);
        }
        ZLB_EXPECT_EQ(live.dfsr, mmu.dfsr);
        ZLB_EXPECT_EQ(live.dfar, mmu.dfar);
        ZLB_EXPECT_EQ(live.ifsr, mmu.ifsr);
        ZLB_EXPECT_EQ(live.ifar, mmu.ifar);
        ZLB_EXPECT_EQ(live.par, mmu.par);
        ZLB_EXPECT_EQ(live.replaced_section_hits, mmu.replaced_section_hits);
        ZLB_EXPECT_EQ(cpu.bus->stats.reads, stats.reads);
        ZLB_EXPECT_EQ(cpu.bus->stats.writes, stats.writes);
        ZLB_EXPECT_EQ(cpu.bus->stats.fetches, stats.fetches);
        ZLB_EXPECT_EQ(cpu.bus->stats.ram, stats.ram);
        ZLB_EXPECT_EQ(cpu.bus->stats.mmio, stats.mmio);
        ZLB_EXPECT_EQ(cpu.bus->stats.unmapped, stats.unmapped);
        ZLB_EXPECT_EQ(cpu.bus->trace.total(), trace_total);
        ZLB_EXPECT_TRUE(cpu.bus->context.core == context.core);
        ZLB_EXPECT_EQ(cpu.bus->context.pc, context.pc);
        ZLB_EXPECT_EQ(cpu.bus->context.core_id, context.core_id);
        ZLB_EXPECT_EQ(cpu.bus->context.nonsecure, context.nonsecure);
        ZLB_EXPECT_EQ(cpu.bus->last_unmapped, last_unmapped);
        for (int i = 0; i < 16; ++i) ZLB_EXPECT_EQ(cpu.r[i], r[i]);
        ZLB_EXPECT_EQ(cpu.cpsr, cpsr);
        ZLB_EXPECT_EQ(cpu.scr, scr);
        ZLB_EXPECT_EQ(cpu.thumb, thumb);
        ZLB_EXPECT_EQ(cpu.ns_, nonsecure);
        ZLB_EXPECT_EQ(cpu.instructions, instructions);
        ZLB_EXPECT_EQ(cpu.exception_count, exceptions);
        ZLB_EXPECT_EQ(cpu.halted, halted);
    }
};

class InspectionReadDevice final : public Device {
public:
    InspectionReadDevice() : Device("inspection read trap", 0x4FFFF000u, 0x1000u) {}
    u64 read(u32, unsigned) override { ++reads; return 0xFFFFFFFFu; }
    void write(u32, unsigned, u64) override {}
    unsigned reads = 0;
};

} // namespace

ZLB_TEST(debugger_dotted_device_cpu_registers_do_not_ack_interrupts) {
    Vita& vita = debugger_test_vita();
    Debugger& debugger = debugger_test_instance();
    std::string output;
    debugger.set_output([&output](const std::string& text) { output = text; });

    Bus& bus = vita.arm_bus();
    bus.context.core_id = 3;
    bus.context.nonsecure = false;
    const u32 cpuif = kermit::kScuBase + 0x100u;
    const u32 distributor = kermit::kScuBase + 0x1000u;
    bus.write32(distributor, 1u);
    bus.write32(cpuif, 9u);
    bus.write32(cpuif + 4u, 0xF8u);
    bus.write32(distributor + 0x4C8u, 0x40404040u);
    bus.write32(distributor + 0x8C8u, 0x08080808u);
    bus.write32(distributor + 0x118u, 1u << 8);
    bus.write32(distributor + 0x218u, 1u << 8);

    debugger.execute("devget Kermit.GIC.CPU3.ICCICR");
    ZLB_EXPECT_TRUE(output == "Kermit.GIC.CPU3.ICCICR = 0x9");
    debugger.execute("devget Kermit.GIC.CPU3.FIQ");
    ZLB_EXPECT_TRUE(output == "Kermit.GIC.CPU3.FIQ = 0x1");
    // The named diagnostic reads the register snapshot, whereas an MMIO IAR
    // read would acknowledge IRQ 200 and deassert its FIQ output.
    debugger.execute("devget Kermit.GIC.CPU3.ICCIAR");
    ZLB_EXPECT_TRUE(output == "Kermit.GIC.CPU3.ICCIAR = 0x3FF");
    ZLB_EXPECT_EQ(bus.read32(distributor + 0x218u) & (1u << 8), 1u << 8);
    debugger.execute("devget Kermit.GIC.CPU3.FIQ");
    ZLB_EXPECT_TRUE(output == "Kermit.GIC.CPU3.FIQ = 0x1");

    debugger.execute("devset Kermit.GIC.CPU3.ICCPMR 0x40");
    ZLB_EXPECT_TRUE(output == "Kermit.GIC.CPU3.ICCPMR <- 0x40");
    debugger.execute("devget Kermit.GIC.CPU3.ICCPMR");
    ZLB_EXPECT_TRUE(output == "Kermit.GIC.CPU3.ICCPMR = 0x40");
    debugger.execute("devget Kermit.GIC.CPU3.FIQ");
    ZLB_EXPECT_TRUE(output == "Kermit.GIC.CPU3.FIQ = 0x0");
    debugger.execute("devget Kermit.GIC.CPU0.ICCPMR");
    ZLB_EXPECT_TRUE(output == "Kermit.GIC.CPU0.ICCPMR = 0x0");
    debugger.set_output({});
}

ZLB_TEST(debugger_arm_instruction_view_selects_core_and_execution_security_bank) {
    Vita& vita = debugger_test_vita();
    Debugger& debugger = debugger_test_instance();
    Bus& bus = vita.arm_bus();
    auto& arm0 = *dynamic_cast<ArmCore*>(vita.arm_core(0));
    auto& arm3 = *dynamic_cast<ArmCore*>(vita.arm_core(3));
    constexpr u32 pc = 0x005AB1DAu;
    constexpr u32 offset = pc & 0xFFFu;
    arm0.reset((pc + 0x100u) | 1u);
    arm0.set_register("CPSR", arm::kModeSystem | arm::kFlagT);
    arm0.set_register("SCR", 1);
    configure_inspection_map(arm0, 0x42000000u, 0x42008000u, pc, 0x43000000u);
    bus.write16(0x43000000u + offset, 0xBF00u);

    arm3.reset(pc | 1u);
    arm3.set_register("CPSR", arm::kModeSystem | arm::kFlagT);
    configure_inspection_map(arm3, 0x42010000u, 0x42018000u, pc, 0x43020000u);
    bus.write16(0x43020000u + offset, 0x2C07u); // Secure CMP R4,#7
    arm3.set_register("SCR", 1);
    configure_inspection_map(arm3, 0x42020000u, 0x42028000u, pc, 0x43010000u);
    // Exact reached Lowio polling loop. Its branch target must remain a VA.
    bus.write16(0x43010000u + offset, 0x69DCu);
    bus.write16(0x43010000u + offset + 2, 0x2C00u);
    bus.write16(0x43010000u + offset + 4, 0xD1FCu);
    bus.context = {"guest context", 0x12345678u, true, 2};
    bus.trace.set_trace_ram(true);
    debugger.set_active_arch(Arch::Arm);
    debugger.set_arm_core_index(3);
    debugger.add_breakpoint(Arch::Arm, pc + 2);

    InspectionState before3(arm3);
    InspectionState before0(arm0);
    const auto lines = debugger.disassemble(pc, 3);
    ZLB_EXPECT_EQ(lines.size(), 3u);
    if (lines.size() == 3) {
        ZLB_EXPECT_TRUE(lines[0].bytes == std::vector<u8>({0xDCu, 0x69u}));
        ZLB_EXPECT_TRUE(lines[0].text == "ldr r4, [r3, #28]");
        ZLB_EXPECT_TRUE(lines[0].is_pc);
        ZLB_EXPECT_EQ(lines[0].length, 2u);
        ZLB_EXPECT_EQ(lines[1].address, pc + 2);
        ZLB_EXPECT_TRUE(lines[1].bytes == std::vector<u8>({0x00u, 0x2Cu}));
        ZLB_EXPECT_TRUE(lines[1].text == "cmp r4, #0");
        ZLB_EXPECT_TRUE(lines[1].has_breakpoint);
        ZLB_EXPECT_TRUE(lines[2].bytes == std::vector<u8>({0xFCu, 0xD1u}));
        ZLB_EXPECT_TRUE(lines[2].text == "bne #0x5AB1DA");
    }
    before3.expect_unchanged(arm3);
    before0.expect_unchanged(arm0);

    debugger.set_arm_core_index(0);
    const auto core0_lines = debugger.disassemble(pc, 1);
    ZLB_EXPECT_EQ(core0_lines.size(), 1u);
    if (!core0_lines.empty()) {
        ZLB_EXPECT_TRUE(core0_lines[0].text == "nop");
        ZLB_EXPECT_FALSE(core0_lines[0].is_pc);
    }
    before0.expect_unchanged(arm0);

    debugger.set_arm_core_index(3);
    // Monitor executes Secure even with SCR.NS=1. Inspecting must not switch
    // that register bank to the Non-secure CP15 MRC/MCR view.
    arm3.set_register("CPSR", arm::kModeMonitor | arm::kFlagT);
    InspectionState before_monitor(arm3);
    const auto secure_lines = debugger.disassemble(pc, 1);
    ZLB_EXPECT_EQ(secure_lines.size(), 1u);
    if (!secure_lines.empty()) {
        ZLB_EXPECT_TRUE(secure_lines[0].bytes == std::vector<u8>({0x07u, 0x2Cu}));
        ZLB_EXPECT_TRUE(secure_lines[0].text == "cmp r4, #7");
    }
    before_monitor.expect_unchanged(arm3);

    // The byte-fed A32 decoder retains virtual branch targets too.
    arm3.set_register("CPSR", arm::kModeMonitor);
    arm3.set_pc(pc + 6);
    bus.write32(0x43020000u + offset + 6, 0xEAFFFFFEu);
    InspectionState before_a32(arm3);
    const auto arm_lines = debugger.disassemble(pc + 6, 1);
    ZLB_EXPECT_EQ(arm_lines.size(), 1u);
    if (!arm_lines.empty()) {
        ZLB_EXPECT_TRUE(arm_lines[0].bytes == std::vector<u8>({0xFEu, 0xFFu, 0xFFu, 0xEAu}));
        ZLB_EXPECT_TRUE(arm_lines[0].text == "b #0x5AB1E0");
        ZLB_EXPECT_EQ(arm_lines[0].length, 4u);
    }
    before_a32.expect_unchanged(arm3);
    debugger.clear_breakpoints();
}

ZLB_TEST(debugger_arm_thumb32_instruction_crosses_noncontiguous_physical_pages) {
    Vita& vita = debugger_test_vita();
    Debugger& debugger = debugger_test_instance();
    Bus& bus = vita.arm_bus();
    auto& cpu = *dynamic_cast<ArmCore*>(vita.arm_core(0));
    constexpr u32 pc = 0x00600FFEu;
    cpu.reset(pc | 1u);
    configure_inspection_map(cpu, 0x42000000u, 0x42008000u, pc, 0x43030000u);
    bus.write32(0x42008000u + 4, 0x43042000u | 0x3Eu);
    bus.write16(0x43030FFEu, 0xF000u); // B.W pc+6
    bus.write16(0x43042000u, 0xB801u);
    bus.write16(0x43042002u, 0xBF00u);
    cpu.mmu.walks = 123;
    cpu.mmu.total_faults = 4;
    cpu.mmu.fault_count = 1;
    cpu.mmu.last_walk.va = 0xDEADBEEFu;
    cpu.mmu.faults[0].va = 0xBADCAFEu;
    cpu.mmu.faults[0].repeats = 42;
    cpu.mmu.dfsr = 0x123;
    cpu.mmu.dfar = 0x456;
    cpu.mmu.ifsr = 0x789;
    cpu.mmu.ifar = 0xABC;
    cpu.mmu.par = 0xDEF;
    cpu.mmu.record_walks = true;
    debugger.set_active_arch(Arch::Arm);
    debugger.set_arm_core_index(0);
    InspectionState before(cpu);
    const auto lines = debugger.disassemble(pc, 2);
    ZLB_EXPECT_EQ(lines.size(), 2u);
    if (lines.size() == 2) {
        ZLB_EXPECT_TRUE(lines[0].bytes == std::vector<u8>({0x00u, 0xF0u, 0x01u, 0xB8u}));
        ZLB_EXPECT_TRUE(lines[0].text.find("0x601004") != std::string::npos);
        ZLB_EXPECT_EQ(lines[0].length, 4u);
        ZLB_EXPECT_EQ(lines[1].address, pc + 4);
        ZLB_EXPECT_TRUE(lines[1].text == "nop");
    }
    before.expect_unchanged(cpu);
    // An inaccessible second half is unavailable, never invented FF bytes.
    bus.write32(0x42008000u + 4, 0);
    unsigned hook_calls = 0;
    cpu.fault_hook = [&hook_calls](u32, u32, bool, bool) { ++hook_calls; return true; };
    InspectionState before_fault(cpu);
    const auto incomplete = debugger.disassemble(pc, 1);
    ZLB_EXPECT_EQ(incomplete.size(), 1u);
    if (!incomplete.empty()) {
        ZLB_EXPECT_TRUE(incomplete[0].text.find("<unavailable: page translation fault") == 0);
        ZLB_EXPECT_TRUE(incomplete[0].bytes == std::vector<u8>({0x00u, 0xF0u}));
        ZLB_EXPECT_EQ(incomplete[0].length, 4u);
    }
    ZLB_EXPECT_EQ(hook_calls, 0u);
    before_fault.expect_unchanged(cpu);
    cpu.fault_hook = {};
}

ZLB_TEST(debugger_arm_instruction_inspection_never_reads_mmio_or_repairs_faults) {
    Vita& vita = debugger_test_vita();
    Debugger& debugger = debugger_test_instance();
    Bus& bus = vita.arm_bus();
    auto& cpu = *dynamic_cast<ArmCore*>(vita.arm_core(0));
    constexpr u32 va = 0x00700000u;
    cpu.reset(va | 1u);
    configure_inspection_map(cpu, 0x42000000u, 0x42008000u, va, 0x4FFFF000u);
    auto device = std::make_unique<InspectionReadDevice>();
    InspectionReadDevice* read_trap = device.get();
    bus.add_device(std::move(device)); // deliberately overlays mapped RAM
    debugger.set_active_arch(Arch::Arm);
    debugger.set_arm_core_index(0);
    unsigned hook_calls = 0;
    cpu.fault_hook = [&hook_calls](u32, u32, bool, bool) { ++hook_calls; return true; };
    InspectionState before_device(cpu);
    const auto mmio = debugger.disassemble(va, 1);
    ZLB_EXPECT_EQ(mmio.size(), 1u);
    if (!mmio.empty()) {
        ZLB_EXPECT_TRUE(mmio[0].text == "<unavailable: PA 0x4FFFF000 is not RAM>");
        ZLB_EXPECT_TRUE(mmio[0].bytes.empty());
    }
    before_device.expect_unchanged(cpu);
    // The TTBR itself points into MMIO. A copied MMU must also avoid invoking
    // the device during its descriptor reads, even though RAM lies underneath.
    cpu.mmu.ttbr0 = 0x4FFFF000u;
    InspectionState before_table(cpu);
    const auto bad_table = debugger.disassemble(va, 1);
    ZLB_EXPECT_EQ(bad_table.size(), 1u);
    if (!bad_table.empty()) {
        ZLB_EXPECT_TRUE(bad_table[0].text.find("<unavailable: section translation fault") == 0);
        ZLB_EXPECT_TRUE(bad_table[0].bytes.empty());
    }
    before_table.expect_unchanged(cpu);
    ZLB_EXPECT_EQ(read_trap->reads, 0u);
    ZLB_EXPECT_EQ(hook_calls, 0u);

    // MMU-off listings inspect physical RAM without introducing bus traffic.
    cpu.mmu.sctlr = 0;
    cpu.set_pc(0x43050000u);
    bus.write16(0x43050000u, 0xBF00u);
    InspectionState before_off(cpu);
    const auto physical = debugger.disassemble(0x43050000u, 1);
    ZLB_EXPECT_EQ(physical.size(), 1u);
    if (!physical.empty()) ZLB_EXPECT_TRUE(physical[0].text == "nop");
    before_off.expect_unchanged(cpu);
    cpu.fault_hook = {};
}
