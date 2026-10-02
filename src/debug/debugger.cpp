// zeliboba - debugger command implementation.
#include "debug/debugger.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>

#include "common/log.h"
#include "common/util.h"
#include "cpu/arm/arm_core.h"
#include "cpu/arm/arm_disasm.h"
#include "cpu/arm/arm_mmu.h"
#include "hw/cmep.h"
#include "hw/emmc.h"
#include "hw/soc.h"
#include "hw/syscon.h"
#include "machine/bootkeys.h"

namespace zlb {

namespace {

/// Boot checkpoint decode.  The ARM boot stages drive the GPO lines through
/// 0xE20A000C (clear bits) and 0xE20A0008 (set bits), and the wiki's Boot
/// Sequence page lists what each value means
/// (wiki.henkaku.xyz/vita/Boot_Sequence#Boot_Debug_Checkpoint_Codes): the
/// second loader's codes start at 0x40 (GPO 0x52 == second loader code 0x12,
/// i.e. the code is the value minus 0x40), while the ARM stages report their
/// code as-is - 0x81..0x8F is the secure kernel bootloader (TrustZone) and
/// 0xA1..0xAF the non-secure kernel bootloader.
std::string boot_checkpoint_text(u32 value) {
    switch (value) {
        case 0x81: return "secure kernel bootloader: core 0 pre-init complete";
        case 0x82: return "secure kernel bootloader: secure world interrupts registered";
        case 0x83: return "secure kernel bootloader: serial console ready";
        case 0x84: return "secure kernel bootloader: some device init";
        case 0x85: return "secure kernel bootloader: co-processor init, other cores start";
        case 0x86: return "secure kernel bootloader: MMU enabled, VBAR/MVBAR set up";
        case 0x87: return "secure kernel bootloader: nothing since 0x86";
        case 0x88: return "secure kernel bootloader: boot setup complete, secure kernel load";
        case 0x89: return "secure kernel bootloader: secure kernel loaded, loading NSKBL";
        case 0x8A: return "secure kernel bootloader: resuming context at 0x1F000000";
        case 0xA1: return "NSKBL: core 0 (non-secure) pre-init complete";
        case 0xA2: return "NSKBL: interrupts registered";
        case 0xA3: return "NSKBL: serial console ready";
        case 0xA4: return "NSKBL: device buffer initialised";
        case 0xA5: return "NSKBL: co-processor init, other cores start";
        case 0xA6: return "NSKBL: MMU enabled, VBAR set up";
        case 0xA7: return "NSKBL: nothing since 0xA6";
        case 0xA8: return "NSKBL: boot setup complete, kernel loading";
        case 0xA9: return "NSKBL: kernel pre-init done, before first external load";
        // 0xAA..0xAF are the NSKBL *fatal* checkpoints: the panic handlers sit in a
        // cluster at 0x51000C3C..0x51000C98, each doing `movs r0,#code`,
        // `bl 0x51010F98` (the GPO writer) and then `b .`.  Measured dumps before the
        // hang: 0xAB reads SPSR, 0xAC reads IFSR/DFAR/IFAR, 0xAD reads DFSR (this is
        // the one a data abort reaches - the model hit it at 0x51000C80 with
        // DFAR = 0x4B656350), 0xAA/0xAE/0xAF dump nothing.
        case 0xAA: return "NSKBL: fatal checkpoint (panic handler 0x51000C3E)";
        case 0xAB: return "NSKBL: fatal checkpoint (reads SPSR)";
        case 0xAC: return "NSKBL: fatal checkpoint (reads IFSR/DFAR/IFAR)";
        case 0xAD: return "NSKBL: fatal checkpoint: data abort (reads DFSR)";
        case 0xAE: return "NSKBL: fatal checkpoint (panic handler 0x51000C86)";
        case 0xAF: return "NSKBL: fatal checkpoint (panic handler 0x51000C92)";
        default: break;
    }
    if (value >= 0x40 && value < 0x80) {
        // The wiki lists the second-loader rows of the 1.04 firmware by GPO value
        // and the 0.931.010 rows as "GPO value 0x52 == code 0x12", so the value is
        // printed as-is first and the offset reading second.
        switch (value) {
            case 0x40: return "second loader: SBL finished successfully (0.931.010)";
            case 0x41: return "second loader: hardware info / GPO init check done";
            case 0x43: return "second loader: bigmac keyring 0x508/0x51B written";
            case 0x46: return "second loader: QA flags written to bigmac keyring";
            case 0x48: return "second loader: SD/eMMC initialised";
            case 0x49: return "second loader: loading kernel_boot_loader.self";
            case 0x4A: return "second loader: SD/eMMC I/O error";
            case 0x4B: return "second loader: reading ConsoleID from eMMC failed";
            case 0x50: return "second loader: copying keyrings 0x602/0x601";
            case 0x52: return "second loader: eMMC is not available";
            case 0x53: return "second loader: reading OpenPSID from eMMC failed";
            case 0x54: return "second loader: minimal firmware version read failed";
            case 0x55: return "second loader: factory firmware version -> bigmac keyring";
            case 0x5A: return "second loader: about to write SceKblParam to SPAD32K";
            case 0x60: return "second loader: SceKblParam done, setting device clocks";
            default: break;
        }
        return format("second loader checkpoint 0x%02X (0.931.010 code 0x%02X)", value, value - 0x40);
    }
    return "unknown";
}

const char* watch_kind_text(unsigned kind) {
    switch (kind & 7) {
        case 1: return "r";
        case 2: return "w";
        case 4: return "x";
        case 7: return "rwx";
        default: return "?";
    }
}

bool parse_watch_kind(const std::string& text, unsigned& kind) {
    kind = 0;
    for (char c : to_lower(text)) {
        if (c == 'r') kind |= 1;
        else if (c == 'w') kind |= 2;
        else if (c == 'x') kind |= 4;
        else return false;
    }
    if (kind == 0) kind = 7;
    return true;
}

bool inspect_arm_instruction_byte(const ArmCore& cpu, u32 va, u8& value, std::string& reason) {
    const arm::MmResult result = cpu.inspection_mmu().inspect_translation(va, true, cpu.mode());
    if (!result.ok) {
        reason = arm::fault_text(result.fault, va, false, true);
        return false;
    }
    const Bus& bus = *cpu.bus;
    const MemRegion* region = bus.region_at(result.phys_addr);
    if (!region || bus.find_device(result.phys_addr)) {
        reason = format("PA 0x%08X is not RAM", result.phys_addr);
        return false;
    }
    value = region->bytes()[result.phys_addr - region->base];
    return true;
}

}  // namespace

Debugger::Debugger(Vita& vita) : vita_(vita) { hook_log(); }

void Debugger::print(const std::string& text) {
    if (output_) output_(text);
    else std::printf("%s\n", text.c_str());
}

void Debugger::hook_log() {
    Log::instance().add_sink([this](const LogRecord& record) {
        log_records_.push_back(record);
        if (log_records_.size() > 4000) log_records_.erase(log_records_.begin(), log_records_.begin() + 1000);
    });
}

// ---------------------------------------------------------------------------
// Cores
// ---------------------------------------------------------------------------

Cpu* Debugger::active_core() const {
    // Kermit is a quad core cluster and its cores run different code (each reads
    // MPIDR), so "the ARM" is the selected core, not always core 0.
    if (active_arch_ == Arch::Arm) {
        if (Cpu* core = const_cast<Vita&>(vita_).arm_core(arm_core_index_)) return core;
    }
    return const_cast<Vita&>(vita_).core(active_arch_);
}

void Debugger::set_arm_core_index(int index) {
    const int count = Vita::kArmCoreCount;
    arm_core_index_ = (index < 0) ? 0 : (index >= count ? count - 1 : index);
}

void Debugger::set_active_arch(Arch arch) {
    active_arch_ = arch;
    // Memory watchpoints need RAM accesses in the trace; only turn tracing off
    // again when nothing is watching memory.
    if (watchpoints_.empty()) {
        if (Cpu* cpu = active_core()) cpu->bus->trace.set_trace_ram(false);
    }
}

// ---------------------------------------------------------------------------
// Breakpoints / watchpoints
// ---------------------------------------------------------------------------

void Debugger::add_breakpoint(Arch arch, u32 address) {
    breakpoints_[arch].insert(address);
    if (Cpu* cpu = vita_.core(arch)) cpu->breakpoints.insert(address);
}

void Debugger::remove_breakpoint(Arch arch, u32 address) {
    breakpoints_[arch].erase(address);
    if (Cpu* cpu = vita_.core(arch)) cpu->breakpoints.erase(address);
}

void Debugger::clear_breakpoints() {
    for (auto& entry : breakpoints_) {
        if (Cpu* cpu = vita_.core(entry.first)) cpu->breakpoints.clear();
        entry.second.clear();
    }
}

const std::set<u32>& Debugger::breakpoints(Arch arch) const {
    static const std::set<u32> empty;
    auto it = breakpoints_.find(arch);
    return it == breakpoints_.end() ? empty : it->second;
}

int Debugger::add_watchpoint(u32 address, unsigned kind, u32 mask, const std::string& note) {
    Watchpoint watch;
    watch.id = next_watchpoint_id_++;
    watch.address = address;
    watch.mask = mask;
    watch.kind = kind;
    watch.note = note;
    watchpoints_.push_back(watch);
    // Memory watchpoints only work if plain RAM accesses reach the trace log.
    for (Bus* bus : {&vita_.cmep_bus(), &vita_.arm_bus(), &vita_.syscon_bus()}) {
        bus->trace.set_trace_ram(true);
    }
    return watch.id;
}

bool Debugger::remove_watchpoint(int id) {
    for (auto it = watchpoints_.begin(); it != watchpoints_.end(); ++it) {
        if (it->id == id) {
            watchpoints_.erase(it);
            return true;
        }
    }
    return false;
}

void Debugger::clear_watchpoints() { watchpoints_.clear(); }

void Debugger::check_watchpoints(u64 trace_from) {
    if (watchpoints_.empty()) return;

    Bus* buses[3] = {&vita_.cmep_bus(), &vita_.arm_bus(), &vita_.syscon_bus()};
    for (Bus* bus : buses) {
        if (bus->trace.total() <= trace_from) continue;
        for (const AccessRecord& record : bus->trace.since(trace_from, 512)) {
            if (record.sequence < trace_from) continue;
            unsigned bit = 0;
            if (record.kind == AccessKind::Read) bit = 1;
            else if (record.kind == AccessKind::Write) bit = 2;
            else bit = 4;

            for (const Watchpoint& watch : watchpoints_) {
                if (!watch.enabled) continue;
                if ((watch.kind & bit) == 0) continue;
                if ((record.address & watch.mask) != (watch.address & watch.mask)) continue;

                last_stop_.stopped = true;
                last_stop_.address = record.address;
                last_stop_.reason = format("watchpoint #%d (%s) at 0x%08X %s value=0x%llX pc=0x%08X", watch.id,
                                           watch_kind_text(watch.kind), record.address, to_string(record.kind),
                                           static_cast<unsigned long long>(record.value), record.pc);
                print("[stop] " + last_stop_.reason);
                stop_requested_ = true;
                return;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------

namespace {
struct MachineStepResult {
    bool stopped = false;
    Arch arch = Arch::Unknown;
    u32 address = 0;
    std::string reason;
};
}  // namespace

void Debugger::step(int count) {
    stop_requested_ = false;
    for (int i = 0; i < count && !stop_requested_; ++i) {
        for (Arch arch : {Arch::MeP, Arch::Arm, Arch::Rl78}) {
            Cpu* cpu = vita_.core(arch);
            if (!cpu || cpu->halted) continue;
            if (breakpoints_.count(arch) && breakpoints_[arch].count(cpu->get_pc())) {
                last_stop_.stopped = true;
                last_stop_.arch = arch;
                last_stop_.address = cpu->get_pc();
                last_stop_.reason = format("%s breakpoint at 0x%08X", to_string(arch), cpu->get_pc());
                print("[stop] " + last_stop_.reason);
                return;
            }
        }

        u64 trace_from = 0;
        Bus* buses[3] = {&vita_.cmep_bus(), &vita_.arm_bus(), &vita_.syscon_bus()};
        for (Bus* bus : buses) trace_from = std::max(trace_from, bus->trace.total());

        for (Arch arch : {Arch::MeP, Arch::Rl78, Arch::Arm}) {
            Cpu* cpu = vita_.core(arch);
            if (!cpu || cpu->halted) continue;
            if (history_enabled_) {
                auto& ring = history_[arch];
                ring.push_back(cpu->get_pc());
                while (ring.size() > history_limit_) ring.pop_front();
            }
            cpu->step();
            ++last_stop_.steps;
        }

        vita_.kermit().tick(1);
        vita_.poll_boot_chain();
        check_watchpoints(trace_from);
    }
}

void Debugger::run(int64_t count) {
    stop_requested_ = false;
    last_stop_.stopped = false;
    int64_t done = 0;
    while (done < count && !stop_requested_) {
        step(1);
        last_stop_.steps = static_cast<u64>(done);
        ++done;
        // step() reports a breakpoint or a watchpoint here; without this the loop
        // kept re-hitting the same breakpoint until the count ran out.
        if (last_stop_.stopped) break;
        if (vita_.stage() == BootStage::Failed) break;
    }
}

void Debugger::run_until(u32 address, int64_t limit) {
    stop_requested_ = false;
    Cpu* cpu = active_core();
    if (!cpu) return;
    int64_t done = 0;
    while (done < limit && !stop_requested_) {
        if (cpu->get_pc() == address) {
            last_stop_.stopped = true;
            last_stop_.arch = active_arch_;
            last_stop_.address = address;
            last_stop_.reason = format("reached 0x%08X", address);
            print("[stop] " + last_stop_.reason);
            return;
        }
        step(1);
        ++done;
    }
    if (done >= limit) print(format("run_until: limit of %lld steps reached", static_cast<long long>(limit)));
}

void Debugger::run_machine(int64_t slices) {
    stop_requested_ = false;
    // A machine slice runs `budget().arm` instructions on *every* core (256 by
    // default).  An ARM breakpoint no longer needs a one-instruction budget:
    // `pc_hook` is consulted before each Kermit instruction, so the stop is exact
    // while the machine keeps running at full speed - which matters, because the
    // kernel boot loader needs hundreds of millions of instructions to reach the
    // code under study.  Watchpoints are still found by diffing the bus traces at
    // slice boundaries, so they keep the slow but exact one-instruction path.
    CoreBudget& budget = vita_.budget();
    const int saved_arm_budget = budget.arm;
    if (!watchpoints_.empty()) budget.arm = 1;
    struct Restore {
        CoreBudget& budget;
        int value;
        std::function<bool(Arch, int, u32)>& hook;
        Vita& vita;
        std::array<std::set<u32>, Vita::kArmCoreCount>* parked = nullptr;
        std::array<std::set<u32>, Vita::kArmCoreCount>& saved;
        ~Restore() {
            budget.arm = value;
            hook = nullptr;
            if (!parked) return;
            // Put the cores' own breakpoint sets back.
            for (int i = 0; i < Vita::kArmCoreCount; ++i) {
                Cpu* core = vita.arm_core(i);
                if (core) core->breakpoints = saved[static_cast<size_t>(i)];
            }
        }
    } restore{budget, saved_arm_budget, vita_.pc_hook, vita_, nullptr, parked_arm_breaks_};

    auto arm_breaks = breakpoints_.find(Arch::Arm);
    if (arm_breaks != breakpoints_.end() && !arm_breaks->second.empty()) {
        const auto& set = arm_breaks->second;
        // Resuming from an ARM breakpoint has to execute the instruction under
        // it exactly once, on the core that stopped - otherwise the hook fires
        // again on the very same PC and `runm` never gets past the breakpoint.
        const u32 resume_pc = last_stop_.stopped && last_stop_.arch == Arch::Arm ? last_stop_.address : 0;
        int resume_core = resume_pc ? last_stop_.core : -1;
        vita_.pc_hook = [&set, resume_pc, resume_core](Arch arch, int core, u32 pc) mutable {
            if (arch != Arch::Arm) return false;
            if (core == resume_core && pc == resume_pc) {
                resume_core = -1;  // one free step over the breakpoint
                return false;
            }
            return set.count(pc) != 0;
        };
        // Cpu::run() returns *before* executing an instruction whose address is in
        // the core's own breakpoint set - and that is exactly the instruction the
        // hook just waved through, so the machine would never get past a
        // breakpoint.  The hook owns ARM breakpoints during a whole-machine run,
        // so park the per-core sets and put them back when the run returns.
        for (int i = 0; i < Vita::kArmCoreCount; ++i) {
            Cpu* core = vita_.arm_core(i);
            if (!core) continue;
            parked_arm_breaks_[static_cast<size_t>(i)] = core->breakpoints;
            core->breakpoints.clear();
        }
        restore.parked = &parked_arm_breaks_;
    }

    for (int64_t i = 0; i < slices && !stop_requested_; ++i) {
        if (vita_.stage() == BootStage::Failed) break;
        u64 trace_from = 0;
        for (Bus* bus : {&vita_.cmep_bus(), &vita_.arm_bus(), &vita_.syscon_bus()}) {
            trace_from = std::max(trace_from, bus->trace.total());
        }
        vita_.run_slice();
        if (vita_.pc_hook_stopped()) {
            vita_.clear_pc_hook_stop();
            last_stop_.stopped = true;
            last_stop_.arch = vita_.pc_hook_arch();
            last_stop_.address = vita_.pc_hook_pc();
            last_stop_.core = vita_.pc_hook_core();
            last_stop_.reason = format("arm%d breakpoint at 0x%08X", vita_.pc_hook_core(),
                                       vita_.pc_hook_pc());
            print("[stop] " + last_stop_.reason);
            return;
        }
        // Slices bypass step(), so record the PC history here as well - "how did
        // this core get here" is the question a whole-machine run raises most.
        if (history_enabled_) {
            for (Arch arch : {Arch::MeP, Arch::Rl78, Arch::Arm}) {
                Cpu* cpu = arch == Arch::Arm ? vita_.arm_core(arm_core_index_) : vita_.core(arch);
                if (!cpu || cpu->halted) continue;
                auto& ring = history_[arch];
                ring.push_back(cpu->get_pc());
                while (ring.size() > history_limit_) ring.pop_front();
            }
        }
        // Slices bypass step(), so check the watchpoints here as well.
        check_watchpoints(trace_from);

        // ... and the breakpoints: a slice runs *all four* Kermit cores, so a
        // breakpoint set on the cluster has to be tested against every core.
        // Without this the boot core cannot be halted at all once
        // kernel_boot_loader reaches its four-core barrier (0x4003B384): a
        // per-core `run` never lets the other three cores arrive, and the
        // interesting per-core init code sits *behind* that barrier.
        if (machine_breakpoint_check()) return;
    }
}

bool Debugger::machine_breakpoint_check() {
    if (breakpoints_.empty()) return false;
    for (Arch arch : {Arch::MeP, Arch::Rl78, Arch::Arm}) {
        auto it = breakpoints_.find(arch);
        if (it == breakpoints_.end() || it->second.empty()) continue;
        if (arch == Arch::Arm) {
            for (int i = 0; i < Vita::kArmCoreCount; ++i) {
                Cpu* cpu = vita_.arm_core(i);
                if (!cpu || cpu->halted) continue;
                if (it->second.count(cpu->get_pc())) {
                    last_stop_.stopped = true;
                    last_stop_.arch = Arch::Arm;
                    last_stop_.address = cpu->get_pc();
                    last_stop_.core = i;
                    last_stop_.reason = format("arm%d breakpoint at 0x%08X", i, cpu->get_pc());
                    print("[stop] " + last_stop_.reason);
                    return true;
                }
            }
            continue;
        }
        Cpu* cpu = vita_.core(arch);
        if (!cpu || cpu->halted) continue;
        if (it->second.count(cpu->get_pc())) {
            last_stop_.stopped = true;
            last_stop_.arch = arch;
            last_stop_.address = cpu->get_pc();
            last_stop_.reason = format("%s breakpoint at 0x%08X", to_string(arch), cpu->get_pc());
            print("[stop] " + last_stop_.reason);
            return true;
        }
    }
    return false;
}

void Debugger::clear_history() { history_.clear(); }

std::vector<u32> Debugger::history(Arch arch, size_t count) const {
    if (arch == Arch::Unknown) arch = active_arch_;
    auto it = history_.find(arch);
    if (it == history_.end()) return {};
    const auto& ring = it->second;
    const size_t take = (count == 0 || count > ring.size()) ? ring.size() : count;
    return std::vector<u32>(ring.end() - static_cast<std::ptrdiff_t>(take), ring.end());
}

// ---------------------------------------------------------------------------
// Views
// ---------------------------------------------------------------------------

std::vector<Debugger::DisassemblyLine> Debugger::disassemble(u32 address, int count, Arch arch) {
    std::vector<DisassemblyLine> lines;
    if (arch == Arch::Unknown) arch = active_arch_;
    Cpu* cpu = arch == Arch::Arm ? vita_.arm_core(arm_core_index_) : vita_.core(arch);
    if (!cpu) return lines;

    u32 pc = cpu->get_pc();
    for (int i = 0; i < count; ++i) {
        unsigned length = 0;
        DisassemblyLine line;
        line.address = address;
        if (const auto* arm = dynamic_cast<const ArmCore*>(cpu)) {
            std::array<u8, 4> bytes{};
            std::string reason;
            length = arm->thumb ? 2u : 4u;
            bool available = true;
            for (unsigned b = 0; b < length; ++b) {
                if (static_cast<u64>(address) + b > 0xFFFFFFFFull) {
                    reason = "instruction crosses address-space end";
                    available = false;
                    break;
                }
                if (!inspect_arm_instruction_byte(*arm, address + b, bytes[b], reason)) {
                    available = false;
                    break;
                }
                line.bytes.push_back(bytes[b]);
                if (arm->thumb && b == 1u) {
                    const u32 hw1 = static_cast<u32>(bytes[0]) | (static_cast<u32>(bytes[1]) << 8);
                    if ((hw1 & 0xF800u) >= 0xE800u) length = 4u;
                }
            }
            line.text = available ? arm_disassemble_bytes(bytes.data(), address, arm->thumb, length) :
                                    format("<unavailable: %s>", reason.c_str());
        } else {
            line.text = cpu->disassemble(address, length);
            for (unsigned b = 0; b < length && b < 8; ++b) line.bytes.push_back(cpu->bus->read8(address + b));
        }
        if (length == 0) length = 4;
        line.length = length;
        line.is_pc = (address == pc);
        line.has_breakpoint = breakpoints(arch).count(address) != 0;
        lines.push_back(std::move(line));
        address += length;
    }
    return lines;
}

std::vector<std::string> Debugger::memory_dump(u32 address, int rows, int bytes_per_row) {
    std::vector<std::string> out;
    Bus& bus = *active_core()->bus;
    for (int row = 0; row < rows; ++row) {
        u32 base = address + static_cast<u32>(row * bytes_per_row);
        std::string line = format("%08X  ", base);
        std::string ascii;
        for (int i = 0; i < bytes_per_row; ++i) {
            u8 value = bus.read8(base + static_cast<u32>(i));
            line += format("%02X ", value);
            ascii.push_back(value >= 32 && value < 127 ? static_cast<char>(value) : '.');
            if (i == bytes_per_row / 2 - 1) line += ' ';
        }
        line += " |" + ascii + "|";
        out.push_back(line);
    }
    return out;
}

std::vector<RegValue> Debugger::registers_of(Cpu& cpu) {
    std::vector<RegValue> out;
    cpu.registers(out);
    return out;
}

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

void Debugger::set_log_level(LogLevel level) { Log::instance().set_level(level); }
void Debugger::set_category_level(const std::string& category, LogLevel level) {
    Log::instance().set_category_level(category, level);
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

namespace {

std::vector<std::string> tokenize(const std::string& line) {
    std::vector<std::string> tokens;
    std::string current;
    for (char c : line) {
        if (c == ' ' || c == '\t' || c == ',') {
            if (!current.empty()) {
                tokens.push_back(current);
                current.clear();
            }
        } else {
            current.push_back(c);
        }
    }
    if (!current.empty()) tokens.push_back(current);
    return tokens;
}

u32 arg_address(const std::vector<std::string>& args, size_t index, u32 fallback) {
    if (index >= args.size()) return fallback;
    u32 value = fallback;
    parse_u32(args[index], value);
    return value;
}

int arg_int(const std::vector<std::string>& args, size_t index, int fallback) {
    if (index >= args.size()) return fallback;
    u32 value = static_cast<u32>(fallback);
    parse_u32(args[index], value);
    return static_cast<int>(value);
}

}  // namespace

bool Debugger::execute(const std::string& line) {
    std::string trimmed = trim(line);
    if (trimmed.empty()) return true;

    std::vector<std::string> tokens = tokenize(trimmed);
    if (tokens.empty()) return true;
    std::string command = to_lower(tokens[0]);
    std::vector<std::string> args(tokens.begin() + 1, tokens.end());

    auto emit = [this](const std::string& text) { print(text); };

    if (command == "echo") {
        emit(trimmed.size() > 4 ? trimmed.substr(4) : std::string());
        return true;
    }

    // --- execution -----------------------------------------------------
    if (command == "step" || command == "s") {
        step(arg_int(args, 0, 1));
        emit(format("%s pc=%s", to_string(active_arch()), hex(active_core()->get_pc(), 8).c_str()));
        return true;
    }
    if (command == "next" || command == "n") {
        step(arg_int(args, 0, 1));
        emit(format("%s pc=%s", to_string(active_arch()), hex(active_core()->get_pc(), 8).c_str()));
        return true;
    }
    if (command == "run" || command == "r") {
        run(arg_int(args, 0, 100000));
        emit(format("executed %llu machine steps", static_cast<unsigned long long>(last_stop_.steps)));
        emit(vita_.status_line());
        return true;
    }
    if (command == "runm" || command == "slices") {
        const int slices = arg_int(args, 0, 1000);
        const auto started = std::chrono::steady_clock::now();
        run_machine(slices);
        const double elapsed =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
        emit(format("%s  [%d slices in %.0f ms]", vita_.status_line().c_str(), slices, elapsed));
        return true;
    }
    if (command == "until") {
        run_until(arg_address(args, 0, active_core()->get_pc()));
        return true;
    }
    if (command == "reset") {
        vita_.reset(true);
        emit("machine reset");
        return true;
    }
    if (command == "core" || command == "cores") {
        if (args.empty()) {
            // All four Kermit cores plus the other two processors: on a quad core
            // machine "the ARM" is a cluster, and its cores run different code.
            for (int i = 0; i < Vita::kArmCoreCount; ++i) {
                Cpu* cpu = vita_.arm_core(i);
                if (!cpu) continue;
                emit(format("%-5s %-18s pc=%s %s %s%s", format("arm%d", i).c_str(), cpu->core_name(),
                            hex(cpu->get_pc(), 8).c_str(), cpu->halted ? "HALTED " : "running",
                            cpu->status_line().c_str(),
                            (active_arch_ == Arch::Arm && arm_core_index_ == i) ? "  <- active" : ""));
            }
            for (Arch arch : {Arch::MeP, Arch::Rl78}) {
                Cpu* cpu = vita_.core(arch);
                emit(format("%-5s %-18s pc=%s %s %s%s", to_string(arch), cpu ? cpu->core_name() : "(absent)",
                            cpu ? hex(cpu->get_pc(), 8).c_str() : "--------",
                            cpu && cpu->halted ? "HALTED " : "running",
                            cpu ? cpu->status_line().c_str() : "",
                            (active_arch_ == arch) ? "  <- active" : ""));
            }
            return true;
        }
        // `core arm2` selects a core of the cluster; `core arm` keeps the current
        // one so that `core arm` / `core arm2` / `core arm` round-trips.
        std::string name = to_lower(args[0]);
        if (name.size() > 3 && starts_with(name, "arm") && isdigit(static_cast<unsigned char>(name[3]))) {
            set_active_arch(Arch::Arm);
            set_arm_core_index(std::atoi(name.c_str() + 3));
            emit(format("active core: arm%d", arm_core_index_));
            return true;
        }
        Arch arch = Arch::Unknown;
        if (!parse_arch(args[0], arch) || arch == Arch::Unknown) {
            emit("usage: core <mep|arm[0..3]|rl78>");
            return true;
        }
        set_active_arch(arch);
        emit(format("active core: %s", arch == Arch::Arm ? format("arm%d", arm_core_index_).c_str()
                                                         : to_string(arch)));
        return true;
    }
    if (command == "history" || command == "hist") {
        if (!args.empty() && (args[0] == "on" || args[0] == "off" || args[0] == "clear")) {
            if (args[0] == "on") set_history_enabled(true);
            else if (args[0] == "off") set_history_enabled(false);
            else clear_history();
            emit(format("history %s (%zu entries)", history_enabled() ? "on" : "off", history_.size()));
            return true;
        }
        Arch arch = Arch::Unknown;
        size_t index = 0;
        if (!args.empty()) {
            Arch parsed = Arch::Unknown;
            if (parse_arch(args[0], parsed) && parsed != Arch::Unknown) {
                arch = parsed;
                index = 1;
            }
        }
        const int count = arg_int(args, index, 64);
        auto entries = history(arch, static_cast<size_t>(count < 0 ? 0 : count));
        if (entries.empty()) {
            emit("(history empty - enable it with 'history on' before running)");
            return true;
        }
        const Arch shown = (arch == Arch::Unknown) ? active_arch_ : arch;
        std::string line;
        for (size_t i = 0; i < entries.size(); ++i) {
            line += format("%s%08X", (i % 6 == 0) ? "\n   " : " ", entries[i]);
        }
        emit(format("last %zu PCs of %s:%s", entries.size(), to_string(shown), line.c_str()));
        return true;
    }
    if (command == "stop") {
        stop();
        emit("stop requested");
        return true;
    }

    // --- breakpoints ---------------------------------------------------
    if (command == "bp" || command == "break") {
        Arch arch = active_arch();
        if (!args.empty() && (args[0] == "mep" || args[0] == "arm" || args[0] == "rl78")) {
            parse_arch(args[0], arch);
            args.erase(args.begin());
        }
        u32 address = arg_address(args, 0, 0);
        add_breakpoint(arch, address);
        emit(format("breakpoint set at %s 0x%08X", to_string(arch), address));
        return true;
    }
    if (command == "bpc") {
        u32 address = arg_address(args, 0, 0);
        for (Arch arch : {Arch::MeP, Arch::Arm, Arch::Rl78}) remove_breakpoint(arch, address);
        emit(format("breakpoint cleared at 0x%08X", address));
        return true;
    }
    if (command == "bpl") {
        bool any = false;
        for (Arch arch : {Arch::MeP, Arch::Arm, Arch::Rl78}) {
            for (u32 address : breakpoints(arch)) {
                emit(format("%-4s 0x%08X", to_string(arch), address));
                any = true;
            }
        }
        if (!any) emit("(no breakpoints)");
        return true;
    }
    if (command == "bpcall") {
        clear_breakpoints();
        emit("all breakpoints cleared");
        return true;
    }
    if (command == "watch") {
        unsigned kind = 7;
        if (!args.empty() && (args.back() == "r" || args.back() == "w" || args.back() == "x" ||
                              args.back() == "rw" || args.back() == "rx" || args.back() == "wx" ||
                              args.back() == "rwx")) {
            parse_watch_kind(args.back(), kind);
            args.pop_back();
        }
        u32 address = arg_address(args, 0, 0);
        int id = add_watchpoint(address, kind);
        emit(format("watchpoint #%d at 0x%08X [%s]", id, address, watch_kind_text(kind)));
        return true;
    }
    if (command == "watchc") {
        int id = arg_int(args, 0, -1);
        if (id < 0) {
            clear_watchpoints();
            emit("all watchpoints cleared");
        } else {
            emit(remove_watchpoint(id) ? format("watchpoint #%d removed", id) : "no such watchpoint");
        }
        return true;
    }
    if (command == "wpl") {
        if (watchpoints_.empty()) {
            emit("(no watchpoints)");
        } else {
            for (const Watchpoint& watch : watchpoints_) {
                emit(format("#%-3d 0x%08X mask=0x%08X [%s]%s", watch.id, watch.address, watch.mask,
                            watch_kind_text(watch.kind), watch.enabled ? "" : " (disabled)"));
            }
        }
        return true;
    }

    // --- inspection ----------------------------------------------------
    if (command == "regs" || command == "reg") {
        if (command == "reg" && args.size() >= 2) {
            u64 value = 0;
            if (parse_u64(args[1], value) && active_core()->set_register(args[0], static_cast<u32>(value))) {
                emit(format("%s <- 0x%llX", args[0].c_str(), static_cast<unsigned long long>(value)));
            } else {
                emit("cannot set that register");
            }
            return true;
        }
        for (const RegValue& reg : registers()) {
            emit(format("%-8s %-10s = 0x%016llX %s", reg.group.c_str(), reg.name.c_str(),
                        static_cast<unsigned long long>(reg.value), reg.note.c_str()));
        }
        return true;
    }
    if (command == "dis" || command == "u") {
        u32 address = args.empty() ? active_core()->get_pc() : arg_address(args, 0, 0);
        int count = arg_int(args, 1, 16);
        for (const auto& line : disassemble(address, count)) {
            std::string bytes;
            for (u8 b : line.bytes) bytes += format("%02X ", b);
            emit(format("%c%c %08X  %-18s %s", line.is_pc ? '>' : ' ', line.has_breakpoint ? '*' : ' ',
                        line.address, bytes.c_str(), line.text.c_str()));
        }
        return true;
    }
    if (command == "mem" || command == "x") {
        u32 address = arg_address(args, 0, active_core()->get_pc());
        int rows = arg_int(args, 1, 16);
        for (const std::string& line : memory_dump(address, rows)) emit(line);
        return true;
    }
    if (command == "vpa" || command == "vmem") {
        // `mem` reads *physical* memory.  That is a trap for anyone inspecting the
        // kernel boot loader, which rebuilds its page tables as it goes: the same
        // object VA lives at a different PA before and after that (round 57 in
        // docs/KBL.md lost several rounds to exactly this).  `vpa` translates a VA
        // through the active core's tables - and prints the descriptor chain it
        // used - while `vmem` also dumps the bytes at the resulting PA.
        Cpu* cpu = active_core();
        auto* arm = dynamic_cast<ArmCore*>(cpu);
        if (!arm) {
            emit("vpa: the active core has no MMU (select an ARM core first)");
            return true;
        }
        const u32 va = arg_address(args, 0, cpu->get_pc());
        // Turn the walk recording on just for this query: the walk itself keeps
        // last_walk up to date, but the hot path only publishes it when asked.
        const bool saved_walks = arm->mmu.record_walks;
        arm->mmu.record_walks = true;
        const arm::MmResult result = arm->mmu.translate(va, false, false, arm->mode());
        const ArmMmu::WalkRecord walk = arm->mmu.last_walk;
        arm->mmu.record_walks = saved_walks;

        if (!result.ok) {
            emit(format("VA 0x%08X -> %s (fsr 0x%X)  L1[0x%03X]@0x%08X=0x%08X", va,
                        arm::fault_name(result.fault), result.fsr_status, (va >> 20) & 0xFFFu,
                        walk.l1_addr, walk.l1_desc));
            if (walk.used_l2) {
                emit(format("   L2[0x%02X]@0x%08X=0x%08X domain=%u", (va >> 12) & 0xFFu, walk.l2_addr,
                            walk.l2_desc, walk.domain));
            }
            return true;
        }
        emit(format("VA 0x%08X -> PA 0x%08X  %s%s  (ttbr%d base=0x%08X L1[0x%03X]=0x%08X)", va,
                    result.phys_addr, result.device ? "device" : (result.strongly_ordered ? "strongly-ordered" : "normal"),
                    result.normal && !result.device && !result.strongly_ordered ? " cacheable" : "",
                    walk.ttbr_num, walk.ttbr_base, (va >> 20) & 0xFFFu, walk.l1_desc));
        if (walk.used_l2) {
            emit(format("   L2[0x%02X]@0x%08X=0x%08X domain=%u", (va >> 12) & 0xFFu, walk.l2_addr, walk.l2_desc,
                        walk.domain));
        }
        if (command == "vmem") {
            const int rows = arg_int(args, 1, 4);
            for (const std::string& line : memory_dump(result.phys_addr, rows)) emit(line);
        }
        return true;
    }
    if (command == "save") {
        // Dump a range of the active core's address space to a file.  The
        // decrypted stages only exist inside the machine, and reversing them is
        // far easier with tools/zdis than with one `dis` window at a time, so
        // this is the way to get a stage out for offline analysis.
        if (args.size() < 3) {
            emit("usage: save <addr> <length> <file>");
            return true;
        }
        Bus& bus = *active_core()->bus;
        const u32 address = arg_address(args, 0, 0);
        const u32 length = arg_address(args, 1, 0);
        std::vector<u8> data(static_cast<size_t>(length));
        for (u32 i = 0; i < length; ++i) data[i] = bus.read8(address + i);
        const std::string path = resolve_workspace_path(args[2]);
        if (!write_file(path, data)) {
            emit("cannot write " + path);
            return true;
        }
        emit(format("saved %u bytes from %08X to %s", length, address, path.c_str()));
        return true;
    }
    if (command == "savestate") {
        // Whole-machine snapshot: RAM, every device, every core and the boot
        // chain. `loadstate` restores it and the run continues from exactly this
        // point, which is what makes a 2.5-minute cold boot cheap to re-enter.
        if (args.empty()) {
            emit("usage: savestate <file>");
            return true;
        }
        const std::string path = resolve_workspace_path(args[0]);
        std::string error;
        if (!vita_.save_state(path, error)) {
            emit("savestate failed: " + error);
            return true;
        }
        emit(format("save state written: %s (stage %s)", path.c_str(), to_string(vita_.stage())));
        return true;
    }
    if (command == "loadstate") {
        if (args.empty()) {
            emit("usage: loadstate <file>");
            return true;
        }
        const std::string path = resolve_workspace_path(args[0]);
        std::string error;
        if (!vita_.load_state(path, error)) {
            emit("loadstate failed: " + error);
            return true;
        }
        // The loaded state replaces every register, so a pending hook stop from
        // the interrupted run no longer describes the machine.
        vita_.clear_pc_hook_stop();
        emit(format("save state loaded: %s (stage %s)", path.c_str(), to_string(vita_.stage())));
        return true;
    }
    if (command == "vpoke") {
        // Virtual-address counterpart of `poke`: translate the VA through the
        // active core's tables and write the resulting PA.  Patching a kernel
        // object whose address moves with the page tables (see `vpa`) is the main
        // use, so the translation is printed along with the write.
        if (args.size() < 2) return "usage: vpoke <va> <value> [8|16|32]";
        Cpu* cpu = active_core();
        auto* arm = dynamic_cast<ArmCore*>(cpu);
        if (!arm) {
            emit("vpoke: the active core has no MMU (use poke)");
            return true;
        }
        const u32 va = arg_address(args, 0, 0);
        u64 value = 0;
        if (!parse_u64(args[1], value)) return "cannot parse the value";
        const int bits = args.size() > 2 ? arg_int(args, 2, 32) : 32;
        const bool saved_walks = arm->mmu.record_walks;
        arm->mmu.record_walks = true;
        const arm::MmResult result = arm->mmu.translate(va, true, false, arm->mode());
        const ArmMmu::WalkRecord walk = arm->mmu.last_walk;
        arm->mmu.record_walks = saved_walks;
        if (!result.ok) {
            emit(format("vpoke: VA 0x%08X does not translate (%s, fsr 0x%X, L1[0x%03X]@0x%08X=0x%08X)", va,
                        arm::fault_name(result.fault), result.fsr_status, (va >> 20) & 0xFFFu, walk.l1_addr,
                        walk.l1_desc));
            return true;
        }
        Bus& bus = *cpu->bus;
        switch (bits) {
            case 8: bus.write8(result.phys_addr, static_cast<u8>(value)); break;
            case 16: bus.write16(result.phys_addr, static_cast<u16>(value)); break;
            case 32: bus.write32(result.phys_addr, static_cast<u32>(value)); break;
            default: return "size must be 8, 16 or 32";
        }
        emit(format("vpoke VA 0x%08X -> PA 0x%08X <- 0x%llX (%d bit)  reads back 0x%08X", va, result.phys_addr,
                    static_cast<unsigned long long>(value), bits, bus.read32(result.phys_addr & ~3u)));
        return true;
    }
    if (command == "poke") {
        // Write a value to physical memory through the active core's bus. This is
        // the escape hatch for experiments the device models cannot express yet -
        // installing a translation table entry by hand, patching a stage's
        // parameter block, forcing a device register - without rebuilding.
        if (args.size() < 2) return "usage: poke <addr> <value> [8|16|32]";
        Bus& bus = *active_core()->bus;
        const u32 address = arg_address(args, 0, 0);
        u64 value = 0;
        if (!parse_u64(args[1], value)) return "cannot parse the value";
        const int bits = args.size() > 2 ? arg_int(args, 2, 32) : 32;
        switch (bits) {
            case 8: bus.write8(address, static_cast<u8>(value)); break;
            case 16: bus.write16(address, static_cast<u16>(value)); break;
            case 32: bus.write32(address, static_cast<u32>(value)); break;
            default: return "size must be 8, 16 or 32";
        }
        emit(format("poke %08X <- 0x%llX (%d bit)  reads back 0x%08X", address,
                    static_cast<unsigned long long>(value), bits, bus.read32(address & ~3u)));
        return true;
    }
    if (command == "trace") {
        emit(cmd_trace(args));
        return true;
    }
    if (command == "devices") {
        emit(cmd_devices(args));
        return true;
    }
    if (command == "map") {
        // Which device/region does an address actually resolve to, and what does
        // it read?  The fast page table and Bus::find_device() can disagree, so
        // this prints the page kind as well.
        const u32 address = arg_address(args, 0, active_core()->get_pc());
        Bus* buses[3] = {&vita_.cmep_bus(), &vita_.arm_bus(), &vita_.syscon_bus()};
        const char* names[3] = {"mep", "arm", "rl78"};
        std::string out;
        for (int i = 0; i < 3; ++i) {
            Bus* bus = buses[i];
            Device* device = bus->find_device(address);
            const MemRegion* region = bus->region_at(address, 1);
            out += format("%-5s device=%-28s region=%-16s read32=0x%08X direct=0x%08X\n", names[i],
                          device ? device->name().c_str() : "-", region ? region->name.c_str() : "-",
                          static_cast<unsigned>(bus->read32(address)),
                          device ? static_cast<unsigned>(device->read(address, 4)) : 0u);
        }
        for (const std::string& line : split(out, '\n')) {
            if (!line.empty()) emit(line);
        }
        // A RAM region marks whole pages, and it may cover the page without covering
        // the address, so report those too.
        for (int i = 0; i < 3; ++i) {
            const u32 index = address >> 12;
            for (const auto& region : buses[i]->regions()) {
                if (!region.mapped || region.size == 0) continue;
                const u32 first = region.base >> 12;
                const u32 last = (region.base + region.size - 1) >> 12;
                if (index < first || index > last) continue;
                emit(format("%-5s page-region %-20s base=0x%08X size=0x%X", names[i], region.name.c_str(),
                            region.base, region.size));
            }
        }
        // Every device that claims the address: the fast page table keeps the last
        // one added, while find_device() keeps the smallest window, so listing both
        // makes a shadowing window obvious.
        for (int i = 0; i < 3; ++i) {
            for (const auto& device : buses[i]->devices()) {
                if (!device->handles(address)) continue;
                emit(format("%-5s candidate %-28s base=0x%08X size=0x%X", names[i], device->name().c_str(),
                            device->base(), device->size()));
            }
        }
        return true;
    }
    if (command == "devget" || command == "devset") {
        emit(cmd_dev(args, command == "devset"));
        return true;
    }
    if (command == "gpo") {
        // Boot checkpoint: the stages clear bits in 0xE20A000C and set the next
        // code in 0xE20A0008, so output latch+34 holds it in bits 16..23
        // (KBL writes 0x00840000 for code 0x84).  Both processors drive the same
        // lines, so the value is whichever stage wrote last.
        const u32 raw = vita_.arm_bus().read32(0xE20A0034);
        const u32 value = (raw >> 16) & 0xFFu;
        emit(format("GPO raw=0x%08X checkpoint=0x%02X - %s", raw, value,
                    boot_checkpoint_text(value).c_str()));
        return true;
    }
    if (command == "console" || command == "uart") {
        // Whatever the firmware has written to its console block (+0x70 of
        // 0xE2030000/0xE2040000) since the last call - including a partial line,
        // which take_uart_output() flushes.  The boot loaders print their
        // progress here, so this is the cheapest window into a boot failure.
        const std::string text = vita_.kermit().take_uart_output();
        if (text.empty()) {
            emit("(console idle)");
            return true;
        }
        std::string line;
        for (char c : text) {
            if (c == '\r') continue;
            if (c == '\n') {
                emit(line);
                line.clear();
                continue;
            }
            line.push_back(c);
        }
        if (!line.empty()) emit(line);
        return true;
    }
    if (command == "bootctx" || command == "context") {
        // The ARM boot context is the piece the CMeP is supposed to hand over and
        // that the kernel boot loader reads (docs/KBL.md rounds 41-43).  Two copies
        // matter: the CMeP's own DRAM source (0x40000000) and the scratch that the
        // ARM sees at PA 0.  Print both so "empty context" is one command away.
        Bus& cmep = vita_.cmep_bus();
        Bus& arm = vita_.arm_bus();
        emit("CMeP DRAM source 0x40000000 (boot context, gate at second-loader 0x40850):");
        for (u32 row = 0; row < 4; ++row) {
            emit(format("  %08X  %08X %08X %08X %08X", 0x40000000u + row * 16,
                        cmep.read32(0x40000000u + row * 16), cmep.read32(0x40000004u + row * 16),
                        cmep.read32(0x40000008u + row * 16), cmep.read32(0x4000000Cu + row * 16)));
        }
        emit("ARM PA 0 (mirror of the CMeP scratch, first 32 KiB):");
        for (u32 row = 0; row < 4; ++row) {
            emit(format("  %08X  %08X %08X %08X %08X", row * 16, arm.read32(row * 16),
                        arm.read32(row * 16 + 4), arm.read32(row * 16 + 8), arm.read32(row * 16 + 12)));
        }
        const u32 gate_reply = cmep.read32(0xE0010004);
        emit(format("CMeP status 0xE0010004 = 0x%08X (secure kernel wants exactly 0x80000005)", gate_reply));
        return true;
    }
    if (command == "emmc") {
        emit(cmd_emmc(args));
        return true;
    }
    if (command == "boot" || command == "stage") {
        if (command == "stage" && !args.empty()) {
            BootStage target = BootStage::PowerOn;
            std::string name = to_lower(args[0]);
            if (name == "first" || name == "firstloader") target = BootStage::CmepFirstLoader;
            else if (name == "second" || name == "secondloader") target = BootStage::CmepSecondLoader;
            else if (name == "secure" || name == "securekernel") target = BootStage::CmepSecureKernel;
            else if (name == "kbl" || name == "arm") target = BootStage::ArmKernelBootLoader;
            else if (name == "nskbl") target = BootStage::NskblEntry;
            else if (name == "kernel") target = BootStage::KernelEntry;
            else {
                emit("usage: stage <first|second|secure|kbl|nskbl|kernel>");
                return true;
            }
            emit(vita_.enter_stage(target) ? format("entered stage %s", to_string(target))
                                           : format("could not enter stage %s", to_string(target)));
            return true;
        }
        emit(cmd_boot(args));
        return true;
    }
    if (command == "keyring") {
        emit(cmd_keyring(args));
        return true;
    }
    if (command == "bootkeys") {
        // Which resident first-loader build is fitted and which signed block its
        // RSA check expects (see machine/bootkeys.cpp).  Useful when a differently
        // built first loader is loaded with --first-loader.
        emit(describe_boot_keys(vita_.cmep_bus()));
        return true;
    }
    if (command == "faults") {
        emit(cmd_faults(args));
        return true;
    }
    if (command == "cov" || command == "coverage") {
        // `cov [mep] [start] [bytes]` prints the map, `cov save <file> [mep]` writes the
        // raw bitmap so external tools can join it with the disassembly.
        std::vector<std::string> rest = args;
        bool mep = false;
        if (!rest.empty() && (rest[0] == "mep" || rest[0] == "cmep")) {
            mep = true;
            rest.erase(rest.begin());
        } else if (!rest.empty() && rest[0] == "arm") {
            rest.erase(rest.begin());
        }
        if (!rest.empty() && rest[0] == "save") {
            if (rest.size() < 2) {
                emit("usage: cov save <file> [mep]");
                return true;
            }
            // `cov save <file> mep` and `cov mep save <file>` are both accepted.
            if (rest.size() > 2 && (rest[2] == "mep" || rest[2] == "cmep")) mep = true;
            std::ofstream out(rest[1], std::ios::binary);
            if (!out) {
                emit("cannot write " + rest[1]);
                return true;
            }
            const std::vector<u8>& bits = mep ? vita_.mep_cov_bytes() : vita_.arm_cov_bytes();
            if (bits.empty()) {
                emit(format("coverage bitmap is empty (%s map armed, but no instruction of that core ran)",
                            mep ? "CMeP" : "ARM"));
                return true;
            }
            out.write(reinterpret_cast<const char*>(bits.data()),
                      static_cast<std::streamsize>(bits.size()));
            emit(format("coverage bitmap written: %s (%zu bytes, base 0x%08X, %u bytes/bit, %s)",
                        rest[1].c_str(), bits.size(),
                        mep ? vita_.mep_cov_base() : vita_.arm_cov_base(),
                        mep ? vita_.mep_cov_granularity() : vita_.arm_cov_granularity(),
                        mep ? "CMeP" : "ARM"));
            return true;
        }
        emit(cmd_cov(rest, mep));
        return true;
    }
    if (command == "info") {
        emit(cmd_info(args));
        return true;
    }
    if (command == "image" || command == "load") {
        emit(cmd_image(args));
        return true;
    }
    if (command == "log") {
        emit(cmd_log(args));
        return true;
    }
    if (command == "help" || command == "?") {
        emit(cmd_help(args));
        return true;
    }
    if (command == "quit" || command == "q" || command == "exit") {
        quit_ = true;
        return false;
    }

    emit(format("unknown command: %s (try 'help')", command.c_str()));
    return true;
}

// ---------------------------------------------------------------------------
// Individual commands
// ---------------------------------------------------------------------------

std::string Debugger::cmd_help(const std::vector<std::string>& args) {
    (void)args;
    return
        "execution\n"
        "  step [n] | s [n]       step the active core n instructions (all cores advance 1:1)\n"
        "  run [n] | r [n]        run n machine steps, stopping on breakpoints/watchpoints\n"
        "  runm [n]               run n whole machine slices (fast; ARM breakpoints stay exact)\n"
        "  until <addr>           run until the active core reaches addr\n"
        "  reset                  power-on reset\n"
        "  core [mep|arm|rl78]    show or select the active core\n"
        "inspection\n"
        "  regs | reg <name> <v>  registers of the active core\n"
        "  dis [addr] [count]     disassemble\n"
        "  mem [addr] [rows]      hex dump (physical address)\n"
        "  vpa <va>               translate a VA through the active core's MMU and show the walk\n"
        "  vmem <va> [rows]       like vpa, then hex dump the bytes at the resulting PA\n"
        "  poke <addr> <val> [s]  write physical memory (s = 8|16|32)\n"
        "  vpoke <va> <val> [s]   translate a VA through the active core's MMU, then write\n"
        "  save <addr> <len> <f>  dump memory to a file (feed it to tools/zdis)\n"
        "  savestate <file>       write the whole machine (RAM, devices, cores) to a file\n"
        "  loadstate <file>       restore a machine written by savestate and continue from it\n"
        "  trace [n]              last n bus accesses\n"
        "  trace find <addr>      accesses to an address\n"
        "  devices                list MMIO devices\n"
        "  map <addr>             which device/region an address resolves to\n"
        "  devget <dev>.<reg>     read a named register\n"
        "  devset <dev>.<reg> <v> write a named register\n"
        "  emmc info|read|write   inspect the attached card\n"
        "  gpo                    boot checkpoint from the GPIO output latch (0xE20A0034)\n"
        "  console                pending firmware console output (UART +0x70)\n"
        "  bootctx                ARM boot context: CMeP DRAM source and the PA 0 mirror\n"
        "  faults [all]           MMU fault ring: faulting pc, VA and page-table entry\n"
        "  keyring                captured CMeP keyring state\n"
        "  bootkeys               fitted first-loader build and the block its RSA check expects\n"
        "  cov [mep] [start] [n]  PC coverage map (ZLB_ARM_COV / ZLB_MEP_COV);\n"
        "                         cov save <file> [mep] writes the raw bitmap\n"
        "  boot                   boot chain report and plan\n"
        "  info                   image / core / access statistics\n"
        "breakpoints / watchpoints\n"
        "  bp [core] <addr>       set a breakpoint\n"
        "  bpc <addr> | bpl | bpcall\n"
        "  watch <addr> [rwx]     set a watchpoint\n"
        "  watchc [id] | wpl\n"
        "misc\n"
        "  log <level>            trace|debug|info|warn|error|off\n"
        "  stage <name>           jump the boot chain to first|second|secure|kbl|nskbl|kernel\n"
        "  load <file> [addr]     load an image into the active core's bus\n"
        "  quit";
}

std::string Debugger::cmd_cov(const std::vector<std::string>& args, bool mep) {
    const bool armed = mep ? vita_.mep_cov_armed() : vita_.arm_cov_armed();
    if (!armed) {
        return mep ? "CMeP coverage not armed (start the model with ZLB_MEP_COV=1)"
                   : "ARM coverage not armed (start the model with ZLB_ARM_COV=1)";
    }
    const u32 gran = mep ? vita_.mep_cov_granularity() : vita_.arm_cov_granularity();
    const u32 base = mep ? vita_.mep_cov_base() : vita_.arm_cov_base();
    const u32 size = mep ? vita_.mep_cov_size() : vita_.arm_cov_size();
    const auto executed = [&](u32 addr) {
        return mep ? vita_.mep_cov_executed(addr) : vita_.arm_cov_executed(addr);
    };
    u32 start = args.empty() ? base : static_cast<u32>(std::strtoul(args[0].c_str(), nullptr, 0));
    u32 bytes = args.size() > 1 ? static_cast<u32>(std::strtoul(args[1].c_str(), nullptr, 0)) : size;
    if (start < base) start = base;
    if (start + bytes > base + size) bytes = (base + size > start) ? (base + size - start) : 0u;

    std::string out = format("coverage %s 0x%08X-0x%08X (%u bytes per bit)\n",
                             mep ? "CMeP" : "ARM", start, start + bytes, gran);
    // Per-page hit counts, so a page that was only brushed through stands out.
    for (u32 page = start; page < start + bytes; page += 0x1000u) {
        const u32 end = (page + 0x1000u < start + bytes) ? page + 0x1000u : start + bytes;
        u32 hit = 0, total = 0;
        for (u32 a = page; a < end; a += gran) {
            ++total;
            if (executed(a)) ++hit;
        }
        out += format("  0x%08X  %3u/%3u %s\n", page, hit, total,
                      hit == 0u ? "  <- never executed" : (hit == total ? "  (fully covered)" : ""));
    }
    // Longest unexecuted runs.
    struct Hole { u32 from; u32 to; };
    std::vector<Hole> holes;
    u32 run_start = 0;
    bool in_run = false;
    for (u32 a = start; a < start + bytes; a += gran) {
        if (!executed(a)) {
            if (!in_run) { run_start = a; in_run = true; }
        } else if (in_run) {
            holes.push_back({run_start, a});
            in_run = false;
        }
    }
    if (in_run) holes.push_back({run_start, start + bytes});
    std::sort(holes.begin(), holes.end(),
              [](const Hole& a, const Hole& b) { return (a.to - a.from) > (b.to - b.from); });
    out += format("unexecuted runs (longest %zu, showing up to 24):\n", holes.size());
    for (size_t i = 0; i < holes.size() && i < 24u; ++i) {
        out += format("  0x%08X-0x%08X  %u bytes\n", holes[i].from, holes[i].to,
                      holes[i].to - holes[i].from);
    }
    return out;
}

std::string Debugger::cmd_info(const std::vector<std::string>& args) {
    (void)args;
    std::string out = vita_.status_line() + "\n";
    Cpu* cpu = active_core();
    if (cpu) {
        out += format("core %s (%s) pc=0x%08X insns=%llu cycles=%llu halted=%d\n", cpu->core_name(),
                      to_string(cpu->arch()), cpu->get_pc(), (unsigned long long)cpu->instructions,
                      (unsigned long long)cpu->cycles, cpu->halted ? 1 : 0);
        out += cpu->status_line() + "\n";
        const BusStats& stats = cpu->bus->stats;
        out += format("bus: reads=%llu writes=%llu fetches=%llu mmio=%llu ram=%llu unmapped=%llu trace=%llu\n",
                      (unsigned long long)stats.reads, (unsigned long long)stats.writes,
                      (unsigned long long)stats.fetches, (unsigned long long)stats.mmio,
                      (unsigned long long)stats.ram, (unsigned long long)stats.unmapped,
                      (unsigned long long)cpu->bus->trace.total());
        std::vector<std::string> lines;
        cpu->describe_state(lines);
        for (const auto& line : lines) out += "  " + line + "\n";
    }
    return out;
}

std::string Debugger::cmd_regs(const std::vector<std::string>& args) {
    (void)args;
    std::string out;
    for (const RegValue& reg : registers()) {
        out += format("%-8s %-10s = 0x%016llX %s\n", reg.group.c_str(), reg.name.c_str(),
                      static_cast<unsigned long long>(reg.value), reg.note.c_str());
    }
    return out;
}

std::string Debugger::cmd_dis(const std::vector<std::string>& args) {
    u32 address = args.empty() ? active_core()->get_pc() : arg_address(args, 0, 0);
    int count = arg_int(args, 1, 16);
    std::string out;
    for (const auto& line : disassemble(address, count)) {
        std::string bytes;
        for (u8 b : line.bytes) bytes += format("%02X ", b);
        out += format("%c%c %08X  %-18s %s\n", line.is_pc ? '>' : ' ', line.has_breakpoint ? '*' : ' ',
                      line.address, bytes.c_str(), line.text.c_str());
    }
    return out;
}

std::string Debugger::cmd_mem(const std::vector<std::string>& args) {
    u32 address = arg_address(args, 0, active_core()->get_pc());
    int rows = arg_int(args, 1, 16);
    std::string out;
    for (const std::string& line : memory_dump(address, rows)) out += line + "\n";
    return out;
}

std::string Debugger::cmd_trace(const std::vector<std::string>& args) {
    Bus& bus = *active_core()->bus;
    if (!args.empty() && args[0] == "find") {
        u32 address = arg_address(args, 1, 0);
        u32 mask = arg_address(args, 2, 0xFFFFFFFFu);
        std::string out;
        for (const AccessRecord& record : bus.trace.find(address, mask)) {
            out += format("%s%u  %08X = %08llX  pc=%08X %s\n", to_string(record.kind), record.size, record.address,
                          static_cast<unsigned long long>(record.value), record.pc,
                          record.unmapped ? "UNMAPPED" : "");
        }
        return out.empty() ? "(no accesses)" : out;
    }
    int count = arg_int(args, 0, 32);
    std::string out;
    for (const AccessRecord& record : bus.trace.tail(static_cast<size_t>(count))) {
        const char* name = "";
        if (record.device >= 0 && record.device < static_cast<int>(bus.devices().size()))
            name = bus.devices()[record.device]->name().c_str();
        out += format("%s%u  %08X = %08llX  pc=%08X %s%s\n", to_string(record.kind), record.size, record.address,
                      static_cast<unsigned long long>(record.value), record.pc, name,
                      record.unmapped ? " (unmapped)" : "");
    }
    return out.empty() ? "(trace empty)" : out;
}

std::string Debugger::cmd_devices(const std::vector<std::string>& args) {
    (void)args;
    std::string out;
    struct Entry {
        const char* bus;
        Bus* ptr;
    };
    Entry buses[3] = {{"mep", &vita_.cmep_bus()}, {"arm", &vita_.arm_bus()}, {"rl78", &vita_.syscon_bus()}};
    for (const Entry& entry : buses) {
        out += format("--- %s bus ---\n", entry.bus);
        for (const auto& device : entry.ptr->devices()) {
            out += format("%-34s %s\n", device->name().c_str(), device->summary().c_str());
        }
    }
    return out;
}

std::string Debugger::cmd_dev(const std::vector<std::string>& args, bool write) {
    if (args.size() < (write ? 2u : 1u))
        return "usage: devget <device>.<register> | devset <device>.<register> <value>";
    const std::string& spec = args[0];
    if (spec.find('.') == std::string::npos || spec.back() == '.')
        return "expected <device>.<register>";

    // Both device and register names can contain dots. Match the longest
    // installed device name, leaving e.g. CPU3.ICCICR intact as the register.
    Device* selected = nullptr;
    Bus* buses[3] = {&vita_.cmep_bus(), &vita_.arm_bus(), &vita_.syscon_bus()};
    for (Bus* bus : buses) {
        for (const auto& device : bus->devices()) {
            const std::string& name = device->name();
            if (spec.size() <= name.size() + 1 || spec.compare(0, name.size(), name) != 0 ||
                spec[name.size()] != '.') continue;
            if (!selected || name.size() > selected->name().size()) selected = device.get();
        }
    }
    if (!selected) return "no such device";
    const std::string register_name = spec.substr(selected->name().size() + 1);
    u64 value = 0;
    if (write) {
        if (!parse_u64(args[1], value)) return "invalid value";
        if (!selected->poke_register(register_name, value)) return "device rejected the write";
        return format("%s.%s <- 0x%llX", selected->name().c_str(), register_name.c_str(),
                      static_cast<unsigned long long>(value));
    }
    if (!selected->peek_register(register_name, value)) return "no such register";
    return format("%s.%s = 0x%llX", selected->name().c_str(), register_name.c_str(),
                  static_cast<unsigned long long>(value));
}

std::string Debugger::cmd_emmc(const std::vector<std::string>& args) {
    EmmcCard& card = vita_.emmc();
    if (args.empty() || args[0] == "info") {
        std::string out = card.summary() + "\n";
        std::vector<std::string> lines;
        card.describe(lines);
        for (const auto& line : lines) out += "  " + line + "\n";
        return out;
    }
    if (args[0] == "read") {
        u32 lba = arg_address(args, 1, 0);
        int count = arg_int(args, 2, 1);
        std::vector<u8> buffer(static_cast<size_t>(count) * 512);
        if (!card.read_blocks(EmmcPartition::User, lba, static_cast<u32>(count), buffer.data()))
            return "read failed";
        std::string out;
        for (int row = 0; row < count * 32; ++row) {
            u32 base = static_cast<u32>(row * 16);
            out += format("%08X  ", static_cast<u32>(lba) * 512 + base);
            for (int i = 0; i < 16; ++i) out += format("%02X ", buffer[base + i]);
            out += "\n";
        }
        return out;
    }
    if (args[0] == "write") {
        return "use a tool to modify an image; writing through the debugger is not implemented";
    }
    return "usage: emmc info|read <lba> <count>";
}

std::string Debugger::cmd_boot(const std::vector<std::string>& args) {
    (void)args;
    std::string out = vita_.boot_report();
    out += "plan:\n";
    for (const std::string& line : vita_.plan_boot()) out += "  " + line + "\n";
    return out;
}

/// MMU fault ring of the ARM cores: which instruction faulted, at which virtual
/// address, and which descriptor the walk used. Without this the only trace of a
/// translation fault is the abort vector the core ends up in, which is exactly
/// the case that stalled kernel_boot_loader (VBAR 0x16100 is unmapped there, so
/// the *first* data abort and the vector fetch that follows it look alike).
std::string Debugger::cmd_faults(const std::vector<std::string>& args) {
    const bool all = !args.empty() && (to_lower(args[0]) == "all");
    std::string out;
    for (int i = 0; i < Vita::kArmCoreCount; ++i) {
        Cpu* cpu = vita_.arm_core(i);
        if (!cpu) continue;
        if (!all && i != arm_core_index_) continue;
        auto* arm = dynamic_cast<ArmCore*>(cpu);
        if (!arm) continue;
        const ArmMmu& mmu = arm->mmu;
        out += format("arm%d: %llu fault(s), %llu walk(s)  VBAR=0x%08X TTBR0=0x%08X TTBR1=0x%08X TTBCR=0x%X\n", i,
                      static_cast<unsigned long long>(mmu.total_faults), static_cast<unsigned long long>(mmu.walks),
                      mmu.vbar, mmu.ttbr0, mmu.ttbr1, mmu.ttbcr);
        if (mmu.fault_count == 0) {
            out += "  (no faults recorded)\n";
            continue;
        }
        for (int f = 0; f < mmu.fault_count; ++f) {
            const ArmMmu::WalkRecord& walk = mmu.faults[f];
            out += format("  #%d VA=0x%08X pc=0x%08X %-5s %-38s x%d\n", f, walk.va, walk.pc,
                          walk.fetch ? "fetch" : (walk.write ? "write" : "read"),
                          arm::fault_name(walk.fault), walk.repeats);
            out += format("     ttbr%d base=0x%08X L1[0x%03X]@0x%08X=0x%08X", walk.ttbr_num, walk.ttbr_base,
                          (walk.va >> 20) & 0xFFFu, walk.l1_addr, walk.l1_desc);
            if (walk.used_l2) {
                out += format(" L2[0x%02X]@0x%08X=0x%08X", (walk.va >> 12) & 0xFFu, walk.l2_addr, walk.l2_desc);
            }
            out += format(" domain=%u\n", walk.domain);
        }
    }
    return out;
}

std::string Debugger::cmd_keyring(const std::vector<std::string>& args) {
    (void)args;
    std::string out;
    out += format("keyring writes : %llu\n", static_cast<unsigned long long>(vita_.cmep_block().keyring_writes()));
    out += format("bigmac ops     : %llu\n", static_cast<unsigned long long>(vita_.cmep_block().bigmac_operations()));
    out += format("bignum ops     : %llu\n", static_cast<unsigned long long>(vita_.cmep_block().bignum_operations()));
    out += format("sc transfers   : %llu\n", static_cast<unsigned long long>(vita_.cmep_block().sc_transfers()));
    out += format("boot mode      : 0x%02X\n", vita_.cmep_block().boot_mode());
    const auto& slots = vita_.cmep_block().captured_keyrings();
    if (slots.empty()) {
        out += "(no keyring values captured)\n";
    } else {
        for (const auto& slot : slots) {
            out += format("keyring 0x%03X flags=0x%08X locked=%d\n  ", slot.first, slot.second.flags,
                          slot.second.locked ? 1 : 0);
            for (u8 byte : slot.second.value) out += format("%02X", byte);
            out += "\n";
        }
    }
    out += "clear flags history:\n";
    for (const auto& entry : vita_.cmep_block().clear_flags_history()) {
        out += format("  0x%08X (keyring 0x%03X)\n", entry.second, entry.first);
    }
    return out;
}

std::string Debugger::cmd_image(const std::vector<std::string>& args) {
    if (args.empty()) return "usage: load <file> [address]";
    Cpu* cpu = active_core();
    std::string path = resolve_workspace_path(args[0]);
    u32 address = args.size() > 1 ? arg_address(args, 1, kAutoAddress) : kAutoAddress;

    if (auto data = read_file(path)) {
        LoadResult result = load_image(*cpu->bus, *data, path, SceKeys::default_keys(), address);
        if (!result.ok) return "load failed: " + result.message;
        cpu->reset(result.entry);
        return format("loaded %s: entry 0x%08X", path_filename(path).c_str(), result.entry);
    }
    return "cannot read " + path;
}

std::string Debugger::cmd_log(const std::vector<std::string>& args) {
    if (args.empty()) return format("log level: %s", to_string(Log::instance().level()));
    LogLevel level = LogLevel::Info;
    if (!parse_log_level(args[0], level)) return "usage: log <trace|debug|info|warn|error|off>";
    set_log_level(level);
    return format("log level: %s", to_string(level));
}

std::vector<std::string> Debugger::complete(const std::string& prefix) const {
    static const std::vector<std::string> commands = {
        "step", "run", "runm", "until", "reset", "core", "bp", "bpc", "bpl", "watch", "watchc",
        "wpl", "regs", "reg", "dis", "mem", "poke", "save", "trace", "devices", "map", "devget",
        "devset", "emmc", "gpo", "console", "uart", "bootctx", "boot", "faults", "stage", "keyring",
        "bootkeys", "info", "load",
        "log", "help", "quit"};
    std::vector<std::string> out;
    for (const auto& command : commands) {
        if (starts_with(command, prefix)) out.push_back(command);
    }
    return out;
}

}  // namespace zlb
