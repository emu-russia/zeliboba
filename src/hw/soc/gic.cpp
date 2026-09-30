// zeliboba - Cortex-A9 MPCore interrupt controller (GIC).
//
// The ARM Generic Interrupt Controller in the Cortex-A9 is a "PL390"-style
// implementation split in two halves (see the Cortex-A9 MPCore TRM, chapters
// 3.3 and 3.4):
//
//   * the distributor, a 4 KiB page at PERIPHBASE + 0x1000, owns the enable,
//     pending, active, priority, target and configuration state of all 32
//     software interrupts per core, 16 private peripheral interrupts and up to
//     224 shared peripheral interrupts. TRM 3.3.1 (page 3-51) gives the
//     register offsets used below.
//   * one per-core CPU interface, a 4 KiB page at PERIPHBASE + 0x1000 + 0x100
//     * n (the TRM's own layout, table 1-3 on page 1-17) and the generic GIC
//     architecture's 0x2000 + 0x1000 * n page. TRM 3.4.1 (page 3-60) lists
//     ICCICR/ICCPMR/ICCBPR/ICCIAR/ICCEOIR/ICCRPR/ICCHPIR.
//
// This model implements CPU0's interface and enough of the distributor for the
// kernel's interrupt manager (intrmgr.elf) and the timers to work: level and
// edge triggered SPIs, priorities, target masks and a correct
// acknowledge/running-priority/EOI state machine.
#include "hw/soc/soc_internal.h"

namespace zlb::kermit {
namespace {

constexpr u32 kIcdDcr = 0x000;
constexpr u32 kIcdIctr = 0x004;
constexpr u32 kIcdIidr = 0x008;
constexpr u32 kIcdIsEnable = 0x100;   // 5 words
constexpr u32 kIcdIcEnable = 0x180;   // 5 words
constexpr u32 kIcdIsPending = 0x200;  // 5 words
constexpr u32 kIcdIcPending = 0x280;  // 5 words
constexpr u32 kIcdIsActive = 0x300;   // 5 words
constexpr u32 kIcdIcActive = 0x380;   // 5 words
constexpr u32 kIcdIPriority = 0x400;  // 32 bytes
constexpr u32 kIcdITargets = 0x800;   // 32 bytes
constexpr u32 kIcdIConfig = 0xC00;    // 32 PPI bytes (ICDICFR0-7)
constexpr u32 kSpiConfigBase = 0xC20; // 32 SPI bytes (ICDICFR8-15)
constexpr u32 kIcdIPpidr = 0xD00;     // PPI status: bit set == level high

constexpr u32 kSpurious = 1023;

// Round 340 diagnostic: nothing in the guest ever writes the GIC, so both the
// distributor's ICDDCR enable and the CPU interface's ICCICR/ICCPMR stay zero and
// the CPU line can never be asserted (highest_pending() needs ICDDCR bit 0, and
// Gic::refresh_line() needs ICCICR bit 0 plus a priority passing ICCPMR).  NSKBL
// still reaches its "interrupts registered" checkpoint (GPO A2), which suggests the
// secure world this model only partly stages leaves the controller enabled before
// handing over.  ZLB_GIC_CPUIF=1 models that: the distributor and the CPU interface
// count as enabled.  The default keeps the measured behaviour.
bool gic_left_enabled_by_secure_world() {
    static const bool forced = [] {
        const char* value = std::getenv("ZLB_GIC_CPUIF");
        return value != nullptr && value[0] == '1';
    }();
    return forced;
}

constexpr u32 kIccIcr = 0x000;
constexpr u32 kIccPmr = 0x004;
constexpr u32 kIccBpr = 0x008;
constexpr u32 kIccIar = 0x00C;
constexpr u32 kIccEoir = 0x010;
constexpr u32 kIccRpr = 0x014;
constexpr u32 kIccHpir = 0x018;
constexpr u32 kIccAbpr = 0x01C;
constexpr u32 kIccIdr = 0x0FC;

constexpr u32 kIccIdrValue = 0x3901243B;  // TRM 3.4.2, page 3-60
constexpr u32 kIcdIctrValue = 0x0000FC21; // 8 lines (32 SGI + 32 PPI + 128 SPI) ... see note below

constexpr const char* kIrqNames[] = {
    "SGI0",  "SGI1",  "SGI2",  "SGI3",  "SGI4",  "SGI5",  "SGI6",  "SGI7",
    "SGI8",  "SGI9",  "SGI10", "SGI11", "SGI12", "SGI13", "SGI14", "SGI15",
    "PPI16", "PPI17", "PPI18", "PPI19", "PPI20", "PPI21", "PPI22", "PPI23",
    "PPI24", "PPI25", "PPI26", "GlobalTimer", "PPI28", "PrivateTimer", "Watchdog", "PPI31",
};

const char* irq_name(u32 id) {
    if (id < sizeof(kIrqNames) / sizeof(kIrqNames[0])) return kIrqNames[id];
    return nullptr;
}

std::string irq_label(u32 id) {
    if (const char* name = irq_name(id)) return format("%s (%u)", name, id);
    return format("SPI%u (%u)", id - 32, id);
}

}  // namespace

// ---------------------------------------------------------------------------
// GicCpuInterface
// ---------------------------------------------------------------------------

GicCpuInterface::GicCpuInterface(std::string name, u32 base, u32 size) : RegisterBlock(std::move(name), base, size) {
    define(kIccIcr, "ICCICR", 0x00000000);
    define(kIccPmr, "ICCPMR", 0x00000000);
    define(kIccBpr, "ICCBPR", 0x00000003);
    define(kIccIar, "ICCIAR", kSpurious);
    define(kIccEoir, "ICCEOIR", 0);
    define(kIccRpr, "ICCRPR", 0x000000FF);
    define(kIccHpir, "ICCHPIR", kSpurious);
    define(kIccAbpr, "ICCABPR", 0x00000003);
    define(kIccIdr, "ICCIDR", kIccIdrValue);
}

u64 GicCpuInterface::read_word(u32 offset, u64 stored) {
    switch (offset) {
        case kIccIar:
            return acknowledge_ ? acknowledge_() : kSpurious;
        case kIccRpr:
            return running_priority();
        case kIccHpir:
            return highest_pending_ ? highest_pending_() : kSpurious;
        case kIccIcr:
        case kIccPmr:
        case kIccBpr:
        case kIccAbpr:
        case kIccIdr:
        default:
            return stored;
    }
}

void GicCpuInterface::write_word(u32 offset, u64 value) {
    switch (offset) {
        case kIccEoir:
            if (eoi_) eoi_(static_cast<u32>(value & 0x3FF));
            poke(kIccEoir, value & 0x3FF);
            return;
        case kIccIar:
        case kIccRpr:
        case kIccHpir:
        case kIccIdr:
            return;  // read only
        default:
            poke(offset, value);
            return;
    }
}

u32 GicCpuInterface::acknowledge() { return acknowledge_ ? acknowledge_() : kSpurious; }

void GicCpuInterface::end_of_interrupt(u32 id) {
    if (eoi_) eoi_(id);
}

u32 GicCpuInterface::running_priority() const {
    // 0xFF means "no active interrupt". The model keeps the per-id active
    // state in the distributor; the running priority is therefore reported by
    // the source (see Gic::refresh_line for how it is used).
    return 0x000000FF;
}

u32 GicCpuInterface::highest_pending() const { return highest_pending_ ? highest_pending_() : kSpurious; }

void GicCpuInterface::set_sources(std::function<u32()> highest_pending_source) {
    highest_pending_ = std::move(highest_pending_source);
}

void GicCpuInterface::set_ack_handler(std::function<u32()> handler) { acknowledge_ = std::move(handler); }

void GicCpuInterface::set_eoi_handler(std::function<void(u32)> handler) { eoi_ = std::move(handler); }

std::string GicCpuInterface::summary() const {
    return format("%s irq=%s pmr=0x%02X pending=%u", name_.c_str(), (peek(kIccIcr) & 1) ? "on" : "off",
                  static_cast<unsigned>(peek(kIccPmr) & 0xFF),
                  highest_pending_ ? highest_pending_() : kSpurious);
}

// ---------------------------------------------------------------------------
// GicDistributor
// ---------------------------------------------------------------------------

GicDistributor::GicDistributor(std::string name, u32 base, u32 size, u32 max_irq)
    : RegisterBlock(std::move(name), base, size), max_irq_(max_irq) {
    const u32 words = (max_irq_ + 31) / 32;
    enabled_.assign(max_irq_, false);
    pending_.assign(max_irq_, false);
    active_.assign(max_irq_, false);
    level_.assign(max_irq_, true);
    sampled_.assign(max_irq_, false);
    priority_.assign(max_irq_, 0xA0);
    targets_.assign(max_irq_, 0x01);
    pulse_left_.assign(max_irq_, 0);

    // SGIs are always enabled and edge triggered. The PPIs and SPIs are edge
    // triggered by default too, which is what the 0xAAAAAAAA reset value of the
    // ICDICFR registers means.
    level_.assign(max_irq_, false);
    pending_after_eoi_.assign(max_irq_, false);
    for (u32 id = 0; id < 16; ++id) enabled_[id] = true;

    define(kIcdDcr, "ICDDCR", 0);
    define(kIcdIctr, "ICDICTR", kIcdIctrValue);
    define(kIcdIidr, "ICDIIDR", 0x0102043B);
    for (u32 i = 0; i < words; ++i) {
        define(kIcdIsEnable + i * 4, format("ICDISER%u", i), 0);
        define(kIcdIcEnable + i * 4, format("ICDICER%u", i), 0);
        define(kIcdIsPending + i * 4, format("ICDISPR%u", i), 0);
        define(kIcdIcPending + i * 4, format("ICDICPR%u", i), 0);
        define(kIcdIsActive + i * 4, format("ICDISAR%u", i), 0);
        define(kIcdIcActive + i * 4, format("ICDICAR%u", i), 0);
    }
    for (u32 i = 0; i < 32; ++i) {
        define(kIcdIPriority + i, format("ICDIPR%02u", i), 0xA0A0A0A0u);
        define(kIcdITargets + i, format("ICDIPTR%02u", i), 0x01010101u);
    }
    for (u32 i = 0; i < 16; ++i) {
        // The PPIs and the SPIs own one configuration word each; a set bit means
        // edge triggered, so 0xAAAAAAAA makes everything edge triggered, which
        // is what a driver that never writes the register expects.
        define(kIcdIConfig + i * 4, format("ICDICFR%02u", i), 0xAAAAAAAAu);
        define(kSpiConfigBase + i * 4, format("ICDICFR%02u", i + 8), 0xAAAAAAAAu);
    }
    define(kIcdIPpidr, "ICDPPISR", 0);
}

bool GicDistributor::enabled(u32 id) const { return id < max_irq_ ? enabled_[id] : false; }
bool GicDistributor::pending(u32 id) const { return id < max_irq_ ? pending_[id] : false; }
bool GicDistributor::active(u32 id) const { return id < max_irq_ ? active_[id] : false; }
bool GicDistributor::edge_triggered(u32 id) const { return id < max_irq_ ? !level_[id] : true; }

u16 GicDistributor::priority(u32 id) const {
    return id < max_irq_ ? priority_[id] : 0xFF;
}

u8 GicDistributor::targets(u32 id) const { return id < max_irq_ ? targets_[id] : 0; }

void GicDistributor::set_default_priority(u32 id, u8 value) {
    if (id >= max_irq_) return;
    priority_[id] = value;
    const u32 byte = kIcdIPriority + id / 4;
    const u32 shift = (id % 4) * 8;
    poke(byte, (peek(byte) & ~(0xFFull << shift)) | (static_cast<u64>(value) << shift));
}

void GicDistributor::set_level(u32 id, bool assertion) {    if (id >= max_irq_) return;
    const bool rising = assertion && !sampled_[id];
    sampled_[id] = assertion;

    if (rising) {
        pulse_left_[id] = 0;
        if (!active_[id]) {
            pending_[id] = true;
        } else {
            // The GIC does not latch a line whose interrupt is already active;
            // remember that a new edge arrived so the EOI can make it pending
            // again (a real distributor does exactly that through the pending
            // latch, which is hidden while the interrupt is active).
            pending_after_eoi_[id] = true;
        }
        return;
    }

    if (!assertion) {
        pulse_left_[id] = 0;
        if (level_[id]) {
            // A level sensitive source that dropped its line has nothing left to
            // service; an edge sensitive one keeps the "an edge arrived while
            // the interrupt was active" flag, because the EOI still has to make
            // that edge pending.
            pending_[id] = false;
            pending_after_eoi_[id] = false;
        }
    }
}

void GicDistributor::pulse(u32 id, u64 ticks) {
    if (id >= max_irq_) return;
    // A pulse is a fresh edge by definition: forget the sampled level so that
    // set_level() sees a rising edge even when the previous pulse left the line
    // sampled high (which happens when a driver re-pulses an interrupt that is
    // still active).
    sampled_[id] = false;
    pulse_left_[id] = static_cast<u32>(ticks == 0 ? 1 : ticks);
    set_level(id, true);
}

void GicDistributor::clear_pending(u32 id) {
    if (id >= max_irq_) return;
    pending_[id] = false;
}

bool GicDistributor::line_asserted(u32 core, u8 priority_mask) const {
    return highest_pending(core, priority_mask) != kSpurious;
}

u32 GicDistributor::highest_pending(u32 core, u8 priority_mask) const {
    if ((peek(kIcdDcr) & 1) == 0 && !gic_left_enabled_by_secure_world()) return kSpurious;

    u32 best = kSpurious;
    u8 best_priority = 0xFF;
    // SGIs/PPIs (0-31) then SPIs. Lower priority value == more important.
    for (u32 id = 0; id < max_irq_; ++id) {
        if (!pending_[id]) continue;
        if (active_[id]) continue;  // already being serviced
        if (!enabled_[id] && !gic_left_enabled_by_secure_world()) continue;
        if (id >= 32) {
            const u8 target = targets_[id];
            if ((target & (1u << core)) == 0) continue;
        }
        const u8 prio = priority_[id];
        if (prio > priority_mask) continue;
        if (prio < best_priority) {
            best_priority = prio;
            best = id;
        }
    }
    return best;
}

u32 GicDistributor::acknowledge(u32 core) {
    const u32 id = highest_pending(core, 0xFF);
    if (id == kSpurious) return kSpurious;
    pending_[id] = false;
    active_[id] = true;
    if (level_[id]) {
        // Level sensitive: the line keeps the pending bit set while asserted.
        pending_[id] = sampled_[id];
    }
    return id;
}

void GicDistributor::end_of_interrupt(u32 core, u32 id) {
    (void)core;
    if (id >= max_irq_) return;
    active_[id] = false;
    // A level sensitive interrupt whose line is still asserted is pending
    // again; an edge sensitive one only if a new edge arrived while it was
    // active (a real distributor's pending latch does that automatically).
    if ((level_[id] && sampled_[id]) || pending_after_eoi_[id]) {
        pending_[id] = true;
    }
    pending_after_eoi_[id] = false;
    // A pulse that has already expired is gone: EOI must not resurrect it.
    pulse_left_[id] = 0;
}

u32 GicDistributor::pending_count() const {
    u32 count = 0;
    for (u32 id = 0; id < max_irq_; ++id) {
        if (pending_[id] && enabled_[id]) ++count;
    }
    return count;
}

bool GicDistributor::tick_pulses(u64 ticks) {
    bool changed = false;
    for (u32 id = 0; id < max_irq_; ++id) {
        const u32 left = pulse_left_[id];
        if (left == 0) continue;
        if (left <= ticks) {
            pulse_left_[id] = 0;
            set_level(id, false);
        } else {
            pulse_left_[id] = static_cast<u32>(left - ticks);
        }
        changed = true;
    }
    return changed;
}

u64 GicDistributor::read_word(u32 offset, u64 stored) { return stored; }

void GicDistributor::reset() {
    RegisterBlock::reset();
    enabled_.assign(max_irq_, false);
    pending_.assign(max_irq_, false);
    active_.assign(max_irq_, false);
    sampled_.assign(max_irq_, false);
    pulse_left_.assign(max_irq_, 0);
    pending_after_eoi_.assign(max_irq_, false);
    level_.assign(max_irq_, false);
    // SGIs are always enabled and edge triggered.
    for (u32 id = 0; id < 16; ++id) enabled_[id] = true;
}

void GicDistributor::sync_status_registers() {
    auto sync = [this](u32 base, const std::vector<bool>& bits) {
        for (u32 i = 0; i * 32 < max_irq_; ++i) {
            u32 value = 0;
            for (u32 b = 0; b < 32 && i * 32 + b < max_irq_; ++b) {
                if (bits[i * 32 + b]) value |= 1u << b;
            }
            poke(base + i * 4, value);
        }
    };
    sync(kIcdIsEnable, enabled_);
    sync(kIcdIsPending, pending_);
    sync(kIcdIsActive, active_);

    u32 ppi = 0;
    for (u32 id = 16; id < 32 && id < max_irq_; ++id) {
        if (sampled_[id]) ppi |= 1u << (id - 16);
    }
    poke(kIcdIPpidr, ppi);
}

u64 GicDistributor::read(u32 address, unsigned size) {
    // The status registers are a view of the internal state, so they have to be
    // rebuilt even for a single byte access (which is how the bus reaches a
    // device when the access straddles two pages).
    sync_status_registers();
    return RegisterBlock::read(address, size);
}

void GicDistributor::write(u32 address, unsigned size, u64 value) {
    const u32 offset = address - base_;
    // Bus splits sub-word MMIO into byte accesses, so a 32 bit store arrives
    // here as four 1 byte calls (and as four separate calls to write_word when
    // the register is word aligned). Several distributor registers need the
    // whole word to be interpreted at once (ICDIPR and ICDIPTR carry four
    // different fields, ICDICFR four two-bit fields), so a 4 byte access is
    // routed to write_word directly.
    if (size == 4 && (offset & 3u) == 0) {
        write_word(offset, value);
        return;
    }
    RegisterBlock::write(address, size, value);
}

void GicDistributor::write_word(u32 offset, u64 value) {
    const u32 value32 = static_cast<u32>(value);

    auto set_bits = [&](u32 base, std::vector<bool>& target, bool state) {
        for (u32 i = 0; i * 32 < max_irq_; ++i) {
            if (offset != base + i * 4) continue;
            for (u32 b = 0; b < 32 && i * 32 + b < max_irq_; ++b) {
                if ((value32 >> b) & 1u) target[i * 32 + b] = state;
            }
            poke(offset, value32);
            return true;
        }
        return false;
    };

    if (offset == kIcdDcr) {
        poke(offset, value32 & 1u);
        return;
    }
    if (offset == kIcdIctr || offset == kIcdIidr) return;
    if (set_bits(kIcdIsEnable, enabled_, true)) return;
    if (set_bits(kIcdIcEnable, enabled_, false)) return;
    if (set_bits(kIcdIsPending, pending_, true)) return;
    if (set_bits(kIcdIcPending, pending_, false)) return;
    if (set_bits(kIcdIsActive, active_, true)) return;
    if (set_bits(kIcdIcActive, active_, false)) return;

    if (offset >= kIcdIPriority && offset < kIcdIPriority + 32) {
        for (u32 i = 0; i < 4; ++i) {
            const u32 id = (offset - kIcdIPriority) * 4 + i;
            if (id >= max_irq_) break;
            priority_[id] = static_cast<u8>((value32 >> (i * 8)) & 0xFF);
        }
        poke(offset, value32);
        return;
    }
    if (offset >= kIcdITargets && offset < kIcdITargets + 32) {
        for (u32 i = 0; i < 4; ++i) {
            const u32 id = (offset - kIcdITargets) * 4 + i;
            if (id >= max_irq_) break;
            targets_[id] = static_cast<u8>((value32 >> (i * 8)) & 0xFF);
        }
        poke(offset, value32);
        return;
    }
    // Version 1 of the Cortex-A9 GIC packs two bits per interrupt into
    // ICDICFR0..ICDICFR7 for the PPIs and ICDICFR8..ICDICFR15 for the SPIs.
    // They sit right after the 0x0C00 window this model uses for ICDICFR, so
    // the SPI half is at kSpiConfigBase. Bit 1 of each pair: 1 = edge, 0 = level.
    if (offset >= kIcdIConfig && offset < kIcdIConfig + 32) {
        for (u32 i = 0; i < 16; ++i) {
            const u32 id = (offset - kIcdIConfig) * 16 + i;
            if (id >= max_irq_) break;
            level_[id] = ((value32 >> (i * 2 + 1)) & 1u) == 0;
        }
        poke(offset, value32);
        return;
    }
    if (offset >= kSpiConfigBase && offset < kSpiConfigBase + 32) {
        for (u32 i = 0; i < 16; ++i) {
            const u32 id = 32 + (offset - kSpiConfigBase) * 16 + i;
            if (id >= max_irq_) break;
            level_[id] = ((value32 >> (i * 2 + 1)) & 1u) == 0;
        }
        poke(offset, value32);
        return;
    }
    poke(offset, value32);
}

std::string GicDistributor::summary() const {
    return format("%s lines=%u pending=%u", name_.c_str(), max_irq_, pending_count());
}

void GicDistributor::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  distributor %s: %s, %u pending", name_.c_str(),
                           (peek(kIcdDcr) & 1) ? "enabled" : "disabled", pending_count()));
    u32 shown = 0;
    for (u32 id = 0; id < max_irq_ && shown < 12; ++id) {
        if (!pending_[id] && !active_[id]) continue;
        lines.push_back(format("    irq %-20s prio=0x%02X target=0x%02X %s%s", irq_label(id).c_str(),
                               priority_[id], targets_[id], pending_[id] ? "PENDING " : "",
                               active_[id] ? "ACTIVE" : ""));
        ++shown;
    }
}

// ---------------------------------------------------------------------------
// Gic
// ---------------------------------------------------------------------------

// The GIC device covers the whole MPCore private region. That is deliberate:
// `Bus::find_device` picks the *smallest* matching window, so the SCU, the
// timers and the distributor (which are installed as their own devices) still
// win for their own pages, while the per-core interface pages at
// PERIPHBASE + 0x0100 and PERIPHBASE + 0x2000 - which are inside the private
// region but outside both the distributor and the CPU interface windows - are
// still routed here instead of falling through to unmapped memory.
Gic::Gic(u32 region_base, u32 region_size, u32 distrib_base, u32 distrib_size, u32 cpuif_base, u32 cpuif_size,
         std::unique_ptr<Device> cpuif_alias, std::unique_ptr<GicCpuInterface> cpuif)
    : Device("Kermit.GIC", region_base, region_size), cpu_interface_alias_(std::move(cpuif_alias)) {
    distributor_ = std::make_unique<GicDistributor>("Kermit.GIC.distributor", distrib_base, distrib_size, kGicMaxIrq);
    cpu_interface_ = std::move(cpuif);
    (void)cpuif_base;
    (void)cpuif_size;

    cpu_interface_->set_sources([this]() { return distributor_->highest_pending(0, cpu_interface_->priority_mask()); });
    cpu_interface_->set_ack_handler([this]() {
        if ((cpu_interface_->peek(kIccIcr) & 1) == 0) return kSpurious;
        const u32 id = distributor_->acknowledge(0);
        active_.push_back(id);
        // Taking the interrupt usually lowers the line; refresh it here rather
        // than waiting for the Gic::read() epilogue, because acknowledge() can
        // also run from the debugger.
        refresh_line();
        return id;
    });
    cpu_interface_->set_eoi_handler([this](u32 id) {
        if (!active_.empty()) active_.pop_back();
        distributor_->end_of_interrupt(0, id);
    });
}

u64 Gic::read(u32 address, unsigned size) {
    Device* device = distributor_->handles(address) ? static_cast<Device*>(distributor_.get()) : nullptr;
    if (device == nullptr) device = cpu_interface_->handles(address) ? static_cast<Device*>(cpu_interface_.get()) : nullptr;
    if (device == nullptr && cpu_interface_alias_ && cpu_interface_alias_->handles(address)) device = cpu_interface_alias_.get();
    if (device == nullptr) return 0;
    const u64 value = device->read(address, size);
    if (device == distributor_.get()) refresh_line();
    return value;
}

void Gic::write(u32 address, unsigned size, u64 value) {
    Device* device = distributor_->handles(address) ? static_cast<Device*>(distributor_.get()) : nullptr;
    if (device == nullptr) device = cpu_interface_->handles(address) ? static_cast<Device*>(cpu_interface_.get()) : nullptr;
    if (device == nullptr && cpu_interface_alias_ && cpu_interface_alias_->handles(address)) device = cpu_interface_alias_.get();
    if (device == nullptr) return;
    device->write(address, size, value);
    refresh_line();
}

void Gic::reset() {
    distributor_->reset();
    cpu_interface_->reset();
    active_.clear();
    line_ = false;
    if (line_callback_) line_callback_(false);
}

void Gic::tick(u64 cycles) {
    // The caller (KermitBlock::tick) already converted CPU cycles into the
    // PERIPHCLK ticks the pulse counters are expressed in.
    if (distributor_->tick_pulses(cycles)) refresh_line();
}

const char* Gic::register_name(u32 address) const {
    if (const char* name = distributor_->register_name(address)) return name;
    if (cpu_interface_alias_ && cpu_interface_alias_->handles(address)) return cpu_interface_alias_->register_name(address);
    return cpu_interface_->register_name(address);
}

void Gic::enumerate_registers(std::vector<RegisterInfo>& out) const {
    // Only the two windows that belong to CPU0's interface: the alias is the
    // same registers at a second address, and listing them twice would make the
    // debugger's address/name mapping ambiguous.
    distributor_->enumerate_registers(out);
    cpu_interface_->enumerate_registers(out);
}

bool Gic::peek_register(const std::string& name, u64& out) const {
    if (distributor_->peek_register(name, out)) return true;
    return cpu_interface_->peek_register(name, out);
}

bool Gic::poke_register(const std::string& name, u64 value) {
    if (distributor_->poke_register(name, value)) return true;
    return cpu_interface_->poke_register(name, value);
}

void Gic::set_cpu(Cpu* cpu) {
    cpu_ = cpu;
    if (cpu != nullptr) cpu->set_irq(static_cast<int>(IrqLine::Irq), line_);
}

void Gic::refresh_line() {
    // Round 339 diagnostic: the CPU interface's ICCICR.Enable bit gates the whole
    // CPU line, and measurement says nothing in the guest ever writes the GIC at
    // all - a write trap over the full block (0x1E000000-0x1E004000) records zero
    // writes, ICCICR reads back 0, and a breakpoint on the non-secure IRQ vector
    // (VA 0x40118) never fires in a complete run.  NSKBL still reaches its
    // "interrupts registered" checkpoint (GPO A2), so on hardware the interface is
    // presumably left enabled by the secure world this model only partly stages.
    // This knob treats it as enabled so the effect can be measured; the default
    // behaviour is unchanged.
    static const bool cpuif_forced = gic_left_enabled_by_secure_world();
    const bool cpuif_enabled = (cpu_interface_->peek(kIccIcr) & 1u) != 0u || cpuif_forced;
    // ICCPMR is likewise never written (it reads back 0, which masks every
    // priority), so the forced path also has to open the priority mask.
    const u8 mask = cpuif_forced ? 0xFFu : cpu_interface_->priority_mask();
    const u32 winning = cpuif_enabled ? distributor_->highest_pending(0, mask) : kSpurious;
    const bool asserted = winning != kSpurious;
    // Round 341 diagnostic: the SDIF asserts its line (measured with
    // ZLB_SDIF_IRQ_LOG) but ArmCore::set_irq never fires (ZLB_ARM_IRQ_LOG), so the
    // question is what this function computes.  ZLB_GIC_LOG=1 prints every change
    // together with the distributor's gate for the winning id.
    static const bool log_gic = [] {
        const char* value = std::getenv("ZLB_GIC_LOG");
        return value != nullptr && value[0] != '0';
    }();
    if (log_gic && asserted != line_) {
        ZLB_LOG_INFO("kermit",
                     "GIC line %s: icddcr=0x%08X iccicr=0x%08X pmr=0x%02X id=%u (cpu=%s)",
                     asserted ? "assert" : "clear", static_cast<unsigned>(distributor_->peek(kIcdDcr)),
                     static_cast<unsigned>(cpu_interface_->peek(kIccIcr)), mask, winning,
                     cpu_ != nullptr ? "set" : "null");
    }
    if (asserted == line_) return;
    line_ = asserted;
    if (line_callback_) line_callback_(line_);
    if (cpu_ != nullptr) cpu_->set_irq(static_cast<int>(IrqLine::Irq), line_);
}

std::string Gic::summary() const {
    return format("%s %s pending=%u ack=0x%03X", name_.c_str(), line_ ? "IRQ" : "idle",
                  distributor_->pending_count(),
                  cpu_interface_->peek(kIccIar) & 0x3FF);
}

void Gic::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s (%s)", name_.c_str(), line_ ? "asserted" : "idle"));
    lines.push_back(format("    ICCICR=0x%08X ICCPMR=0x%02X active=%u", static_cast<unsigned>(cpu_interface_->peek(kIccIcr)),
                           static_cast<unsigned>(cpu_interface_->peek(kIccPmr) & 0xFF),
                           static_cast<unsigned>(active_.size())));
    distributor_->describe(lines);
    for (u32 id : active_) lines.push_back(format("    servicing %s", irq_label(id).c_str()));
}

}  // namespace zlb::kermit
