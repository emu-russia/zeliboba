// zeliboba - Cortex-A9 MPCore interrupt controller (GIC).
//
// The ARM Generic Interrupt Controller in the Cortex-A9 is a "PL390"-style
// implementation split in two halves (see the Cortex-A9 MPCore TRM, chapters
// 3.3 and 3.4):
//
//   * the distributor, a 4 KiB page at PERIPHBASE + 0x1000, owns the enable,
//     pending, active, priority, target and configuration state of the 16
//     software interrupts per core, 16 private peripheral interrupts and up to
//     224 shared peripheral interrupts. TRM 3.3.1 (page 3-51) gives the
//     register offsets used below.
//   * a banked CPU interface at PERIPHBASE + 0x100. TRM 3.4.1 lists
//     ICCICR/ICCPMR/ICCBPR/ICCIAR/ICCEOIR/ICCRPR/ICCHPIR.
//
// This model implements four banked CPU interfaces and the distributor for the
// kernel's interrupt manager (intrmgr.elf) and the timers to work: level and
// edge triggered SPIs, priorities, target masks and a correct
// acknowledge/running-priority/EOI state machine.
#include "hw/soc/soc_internal.h"

namespace zlb::kermit {
namespace {

constexpr u32 kIcdDcr = 0x000;
constexpr u32 kIcdIctr = 0x004;
constexpr u32 kIcdIidr = 0x008;
constexpr u32 kIcdIsr = 0x080;  // ICDISR / GICD_IGROUPR: 0=Secure Group0.
constexpr u32 kIcdIsEnable = 0x100;
constexpr u32 kIcdIcEnable = 0x180;
constexpr u32 kIcdIsPending = 0x200;
constexpr u32 kIcdIcPending = 0x280;
constexpr u32 kIcdIsActive = 0x300;
constexpr u32 kIcdIcActive = 0x380;
constexpr u32 kIcdIPriority = 0x400;  // One byte per interrupt.
constexpr u32 kIcdITargets = 0x800;   // One byte per interrupt.
constexpr u32 kIcdIConfig = 0xC00;    // Two bits per interrupt.
constexpr u32 kIcdIPpidr = 0xD00;     // PPI status: bit set == level high
constexpr u32 kIcdSgir = 0xF00;

constexpr u32 kSpurious = 1023;
// Cortex-A9 MPCore DDI0407G table3-1 and section3.3.7: SGIs are fixed
// edge-sensitive; PPIs have a fixed per-CPU configuration; SPI bit0 is fixed
// to one and bit1 resets to zero (level-sensitive).
constexpr u32 kSgiConfig = 0xAAAAAAAAu;
constexpr u32 kPpiConfig = 0x7DC00000u;
constexpr u32 kSpiConfig = 0x55555555u;

u32 private_irq_config(u32 id) {
    return id < 16u ? 2u : (kPpiConfig >> ((id - 16u) * 2u)) & 3u;
}

// Round 343: the policy itself is set by the machine (`Gic::set_secure_world_left_enabled`,
// which the Vita turns on because its secure stages are staged by substitutions that
// reproduce their checkpoints - "secure world interrupts registered", GPO 0x82 -
// without running their register programming).  These two environment variables
// remain for experiments: ZLB_GIC_STRICT=1 (or ZLB_GIC_CPUIF=0) forces the strict
// hardware behaviour, ZLB_GIC_CPUIF=1 forces the substitution.  Measured effect of
// the policy on the Vita: NSKBL's storage stack goes from 3 to 19 eMMC reads
// including real file data (docs/NSKBL.md 10.47/10.48).
bool gic_effective_left_enabled(bool configured) {
    static const int override = [] {
        const char* strict = std::getenv("ZLB_GIC_STRICT");
        if (strict != nullptr && strict[0] != '0') return 0;
        const char* value = std::getenv("ZLB_GIC_CPUIF");
        if (value != nullptr && value[0] == '0') return 0;
        if (value != nullptr && value[0] == '1') return 1;
        return -1;
    }();
    if (override >= 0) return override == 1;
    return configured;
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
// ITLinesNumber describes implemented groups of 32; CPUNumber is four cores.
constexpr u32 kIcdIctrValue = (1u << 10) | (3u << 5) | (kGicMaxIrq / 32u - 1u);

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
            return nonsecure_access_ ? (running_priority() < 0x80u ? 0u : (running_priority() << 1) & 0xFFu)
                                     : running_priority();
        case kIccHpir:
            return highest_pending_ ? highest_pending_() : kSpurious;
        case kIccIcr:
            return nonsecure_access_ ? (stored >> 1) & 1u : stored;
        case kIccPmr:
            return nonsecure_access_ ? (stored < 0x80u ? 0u : (stored << 1) & 0xFFu) : stored;
        case kIccBpr:
            return nonsecure_access_ ? peek(kIccAbpr) : stored;
        case kIccAbpr:
            return nonsecure_access_ ? 0u : stored;
        case kIccIdr:
        default:
            return stored;
    }
}

void GicCpuInterface::write_word(u32 offset, u64 value) {
    switch (offset) {
        case kIccIcr:
            // The NS view exposes only EnableGrp1 as bit0. In particular an
            // NSKBL store of zero cannot disable Secure Group0 or FIQEn.
            poke(offset, nonsecure_access_ ? (peek(offset) & ~2ull) | ((value & 1u) << 1) : value);
            return;
        case kIccPmr:
            if (!nonsecure_access_) poke(offset, value & 0xFFu);
            else if (peek(offset) >= 0x80u) poke(offset, 0x80u | ((value & 0xFFu) >> 1));
            return;
        case kIccBpr:
            poke(nonsecure_access_ ? kIccAbpr : kIccBpr, value & 7u);
            return;
        case kIccAbpr:
            if (!nonsecure_access_) poke(offset, value & 7u);
            return;
        case kIccEoir:
            if (eoi_) eoi_(static_cast<u32>(value & 0x1FFF));
            poke(kIccEoir, value & 0x1FFF);
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

void GicCpuInterface::write(u32 address, unsigned size, u64 value) {
    const u32 offset = address - base_;
    // Native GIC accesses are word stores. Avoid RegisterBlock's pre-store:
    // the NS view needs the previous Secure register value to retain its bits.
    if (size == 4 && (offset & 3u) == 0) write_word(offset, value);
    else RegisterBlock::write(address, size, value);
}

u32 GicCpuInterface::acknowledge() { return acknowledge_ ? acknowledge_() : kSpurious; }

void GicCpuInterface::end_of_interrupt(u32 id) {
    if (eoi_) eoi_(id);
}

u32 GicCpuInterface::running_priority() const {
    return running_priority_ ? running_priority_() : 0xFFu;
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

void GicCpuInterface::save_state(StateWriter& writer) const {
    // Register image first (RegisterBlock), then the access view. The four
    // handler hooks are host wiring and are never written.
    RegisterBlock::save_state(writer);
    writer.put_bool(nonsecure_access_);
}

void GicCpuInterface::load_state(StateReader& reader) {
    RegisterBlock::load_state(reader);
    nonsecure_access_ = reader.get_bool();
}

// ---------------------------------------------------------------------------
// GicDistributor
// ---------------------------------------------------------------------------

GicDistributor::GicDistributor(std::string name, u32 base, u32 size, u32 max_irq)
    : RegisterBlock(std::move(name), base, size), max_irq_(max_irq) {
    const u32 words = (max_irq_ + 31) / 32;
    const size_t states = max_irq_ + 32u * (kGicCoreCount - 1u);
    group1_.assign(states, false);
    enabled_.assign(states, false);
    pending_.assign(states, false);
    active_.assign(states, false);
    level_.assign(states, true);
    sampled_.assign(states, false);
    priority_.assign(states, 0xA0);
    targets_.assign(states, 0x01);
    pulse_left_.assign(states, 0);

    // SGI/PPI configuration is read-only and banked with the private state.
    // Shared interrupts start level-sensitive, including mailbox 200..203.
    pending_after_eoi_.assign(states, false);
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        for (u32 id = 0; id < 32u; ++id) {
            level_[state_index(id, core)] = (private_irq_config(id) & 2u) == 0u;
            if (id < 16u) enabled_[state_index(id, core)] = true;
        }
    }

    define(kIcdDcr, "ICDDCR", 0);
    define(kIcdIctr, "ICDICTR", kIcdIctrValue);
    define(kIcdIidr, "ICDIIDR", 0x0102043B);
    for (u32 i = 0; i < words; ++i) {
        define(kIcdIsr + i * 4, format("ICDISR%u", i), 0);
        define(kIcdIsEnable + i * 4, format("ICDISER%u", i), 0);
        define(kIcdIcEnable + i * 4, format("ICDICER%u", i), 0);
        define(kIcdIsPending + i * 4, format("ICDISPR%u", i), 0);
        define(kIcdIcPending + i * 4, format("ICDICPR%u", i), 0);
        define(kIcdIsActive + i * 4, format("ICDISAR%u", i), 0);
        define(kIcdIcActive + i * 4, format("ICDICAR%u", i), 0);
    }
    for (u32 i = 0; i < (max_irq_ + 3u) / 4u; ++i) {
        define(kIcdIPriority + i * 4u, format("ICDIPR%02u", i), 0xA0A0A0A0u);
        define(kIcdITargets + i * 4u, format("ICDIPTR%02u", i), 0x01010101u);
    }
    for (u32 i = 0; i < (max_irq_ + 15u) / 16u; ++i) {
        // Each word covers sixteen two-bit interrupt configuration fields.
        define(kIcdIConfig + i * 4, format("ICDICFR%02u", i),
               i == 0u ? kSgiConfig : (i == 1u ? kPpiConfig : kSpiConfig));
    }
    define(kIcdIPpidr, "ICDPPISR", 0);
    define(kIcdSgir, "ICDSGIR", 0);
}

size_t GicDistributor::state_index(u32 id, unsigned core) const {
    return id < 32u && core != 0u ? max_irq_ + (core - 1u) * 32u + id : id;
}

bool GicDistributor::enabled(u32 id, unsigned core) const { return id < max_irq_ && enabled_[state_index(id, core)]; }
bool GicDistributor::pending(u32 id, unsigned core) const { return id < max_irq_ && pending_[state_index(id, core)]; }
bool GicDistributor::active(u32 id, unsigned core) const { return id < max_irq_ && active_[state_index(id, core)]; }
bool GicDistributor::edge_triggered(u32 id, unsigned core) const { return id >= max_irq_ || !level_[state_index(id, core)]; }
bool GicDistributor::group1(u32 id, unsigned core) const { return id < max_irq_ && group1_[state_index(id, core)]; }

u16 GicDistributor::priority(u32 id, unsigned core) const {
    return id < max_irq_ ? priority_[state_index(id, core)] : 0xFF;
}

u8 GicDistributor::targets(u32 id, unsigned core) const {
    return id < max_irq_ ? (id < 32u ? static_cast<u8>(1u << core) : targets_[id]) : 0;
}

void GicDistributor::set_default_priority(u32 id, u8 value) {
    if (id >= max_irq_) return;
    priority_[id] = value;
    const u32 byte = kIcdIPriority + (id / 4u) * 4u;
    const u32 shift = (id % 4) * 8;
    poke(byte, (peek(byte) & ~(0xFFull << shift)) | (static_cast<u64>(value) << shift));
}

void GicDistributor::set_level(u32 id, bool assertion, unsigned core) {
    if (id >= max_irq_) return;
    const size_t index = state_index(id, core);
    const bool rising = assertion && !sampled_[index];
    sampled_[index] = assertion;

    if (rising) {
        pulse_left_[index] = 0;
        if (!active_[index]) {
            pending_[index] = true;
        } else {
            // The GIC does not latch a line whose interrupt is already active;
            // remember that a new edge arrived so the EOI can make it pending
            // again (a real distributor does exactly that through the pending
            // latch, which is hidden while the interrupt is active).
            pending_after_eoi_[index] = true;
        }
        return;
    }

    if (!assertion) {
        pulse_left_[index] = 0;
        if (level_[index]) {
            // A level sensitive source that dropped its line has nothing left to
            // service; an edge sensitive one keeps the "an edge arrived while
            // the interrupt was active" flag, because the EOI still has to make
            // that edge pending.
            pending_[index] = false;
            pending_after_eoi_[index] = false;
        }
    }
}

void GicDistributor::pulse(u32 id, u64 ticks, unsigned core) {
    if (id >= max_irq_) return;
    // A pulse is a fresh edge by definition: forget the sampled level so that
    // set_level() sees a rising edge even when the previous pulse left the line
    // sampled high (which happens when a driver re-pulses an interrupt that is
    // still active).
    const size_t index = state_index(id, core);
    sampled_[index] = false;
    set_level(id, true, core);
    pulse_left_[index] = static_cast<u32>(ticks == 0 ? 1 : ticks);
}

void GicDistributor::clear_pending(u32 id, unsigned core) {
    if (id >= max_irq_) return;
    pending_[state_index(id, core)] = false;
}

bool GicDistributor::line_asserted(u32 core, u8 priority_mask) const {
    return highest_pending(core, priority_mask) != kSpurious;
}

u32 GicDistributor::highest_pending(u32 core, u8 priority_mask) const {
    if (core >= kGicCoreCount) return kSpurious;
    const bool forced = gic_effective_left_enabled(secure_world_left_enabled_);
    const u32 groups = forced ? 3u : static_cast<u32>(peek(kIcdDcr)) & 3u;

    u32 best = kSpurious;
    u8 best_priority = 0xFF;
    // SGIs/PPIs (0-31) then SPIs. Lower priority value == more important.
    for (u32 id = 0; id < max_irq_; ++id) {
        const size_t index = state_index(id, core);
        if ((groups & (group1_[index] ? 2u : 1u)) == 0u) continue;
        if (!pending_[index]) continue;
        if (active_[index]) continue;  // already being serviced
        if (!enabled_[index] && !forced) continue;
        if (id >= 32) {
            const u8 target = targets_[id];
            if ((target & (1u << core)) == 0) continue;
        }
        const u8 prio = priority_[index];
        if (prio >= priority_mask) continue;
        if (prio < best_priority) {
            best_priority = prio;
            best = id;
        }
    }
    return best;
}

u32 GicDistributor::acknowledge(u32 core, u8 priority_mask) {
    const u32 id = highest_pending(core, priority_mask);
    if (id == kSpurious) return kSpurious;
    const size_t index = state_index(id, core);
    pending_[index] = false;
    active_[index] = true;
    u32 token = id;
    if (id < 16u && sgi_sources_[core][id] != 0u) {
        unsigned source = 0;
        while ((sgi_sources_[core][id] & (1u << source)) == 0u) ++source;
        token |= source << 10;
        sgi_sources_[core][id] &= static_cast<u8>(~(1u << source));
        pending_[index] = sgi_sources_[core][id] != 0;
    }
    if (level_[index]) {
        // Level sensitive: the line keeps the pending bit set while asserted.
        pending_[index] = sampled_[index];
    }
    return token;
}

void GicDistributor::end_of_interrupt(u32 core, u32 id) {
    id &= 0x3FFu;
    if (id >= max_irq_) return;
    const size_t index = state_index(id, core);
    active_[index] = false;
    // A level sensitive interrupt whose line is still asserted is pending
    // again; an edge sensitive one only if a new edge arrived while it was
    // active (a real distributor's pending latch does that automatically).
    if ((level_[index] && sampled_[index]) || pending_after_eoi_[index] ||
        (id < 16u && sgi_sources_[core][id] != 0u)) {
        pending_[index] = true;
    }
    pending_after_eoi_[index] = false;
    // A pulse that has already expired is gone: EOI must not resurrect it.
    pulse_left_[index] = 0;
}

u32 GicDistributor::pending_count() const {
    u32 count = 0;
    for (size_t index = 0; index < pending_.size(); ++index) {
        if (pending_[index] && enabled_[index]) ++count;
    }
    return count;
}

bool GicDistributor::tick_pulses(u64 ticks) {
    bool changed = false;
    for (size_t index = 0; index < pulse_left_.size(); ++index) {
        const u32 left = pulse_left_[index];
        if (left == 0) continue;
        if (left <= ticks) {
            pulse_left_[index] = 0;
            const u32 id = index < max_irq_ ? static_cast<u32>(index) : (index - max_irq_) % 32u;
            const unsigned core = index < max_irq_
                                      ? 0u
                                      : 1u + static_cast<unsigned>((index - max_irq_) / 32u);
            set_level(id, false, core);
        } else {
            pulse_left_[index] = static_cast<u32>(left - ticks);
        }
        changed = true;
    }
    return changed;
}

u32 GicDistributor::visible_word(u32 word, const std::vector<bool>& bits) const {
    u32 value = 0;
    for (u32 bit = 0; bit < 32u && word * 32u + bit < max_irq_; ++bit) {
        const size_t index = state_index(word * 32u + bit, access_core_);
        if (bits[index] && (!nonsecure_access_ || group1_[index])) value |= 1u << bit;
    }
    return value;
}

u64 GicDistributor::read_word(u32 offset, u64 stored) {
    if (offset == kIcdDcr) return nonsecure_access_ ? (stored >> 1) & 1u : stored;
    const u32 word_bytes = ((max_irq_ + 31u) / 32u) * 4u;
    if (offset >= kIcdIsr && offset < kIcdIsr + word_bytes)
        return nonsecure_access_ ? 0u : visible_word((offset - kIcdIsr) / 4u, group1_);
    const struct { u32 base; const std::vector<bool>* bits; } ranges[] = {
        {kIcdIsEnable, &enabled_}, {kIcdIcEnable, &enabled_},
        {kIcdIsPending, &pending_}, {kIcdIcPending, &pending_},
        {kIcdIsActive, &active_}, {kIcdIcActive, &active_},
    };
    for (const auto& range : ranges)
        if (offset >= range.base && offset < range.base + word_bytes)
            return visible_word((offset - range.base) / 4u, *range.bits);
    if ((offset >= kIcdIPriority && offset < kIcdIPriority + max_irq_) ||
        (offset >= kIcdITargets && offset < kIcdITargets + max_irq_)) {
        const bool priority_field = offset < kIcdITargets;
        const u32 first = offset - (priority_field ? kIcdIPriority : kIcdITargets);
        u32 value = 0;
        for (u32 lane = 0; lane < 4u && first + lane < max_irq_; ++lane) {
            const u32 id = first + lane;
            const size_t index = state_index(id, access_core_);
            if (nonsecure_access_ && !group1_[index]) continue;
            const u8 byte = priority_field ? (nonsecure_access_ ? static_cast<u8>(priority_[index] << 1) : priority_[index])
                                           : targets(id, access_core_);
            value |= static_cast<u32>(byte) << (lane * 8u);
        }
        return value;
    }
    if (offset >= kIcdIConfig && offset < kIcdIConfig + ((max_irq_ + 15u) / 16u) * 4u) {
        u32 value = 0;
        const u32 first = ((offset - kIcdIConfig) / 4u) * 16u;
        for (u32 field = 0; field < 16u && first + field < max_irq_; ++field) {
            const u32 id = first + field;
            const size_t index = state_index(id, access_core_);
            if (nonsecure_access_ && !group1_[index]) continue;
            const u32 config = id < 32u ? private_irq_config(id) : (level_[index] ? 1u : 3u);
            value |= config << (field * 2u);
        }
        return value;
    }
    if (offset == kIcdSgir) return 0;  // write only
    return stored;
}

void GicDistributor::reset() {
    RegisterBlock::reset();
    const size_t states = enabled_.size();
    group1_.assign(states, false);
    enabled_.assign(states, false);
    pending_.assign(states, false);
    active_.assign(states, false);
    sampled_.assign(states, false);
    pulse_left_.assign(states, 0);
    pending_after_eoi_.assign(states, false);
    level_.assign(states, true);
    priority_.assign(states, 0xA0);
    targets_.assign(states, 0x01);
    sgi_sources_ = {};
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        for (u32 id = 0; id < 32u; ++id) {
            level_[state_index(id, core)] = (private_irq_config(id) & 2u) == 0u;
            if (id < 16u) enabled_[state_index(id, core)] = true;
        }
    }
}

void GicDistributor::sync_status_registers() {
    auto sync = [this](u32 base, const std::vector<bool>& bits) {
        for (u32 i = 0; i * 32 < max_irq_; ++i) {
            u32 value = 0;
            for (u32 b = 0; b < 32 && i * 32 + b < max_irq_; ++b) {
                if (bits[state_index(i * 32 + b, access_core_)]) value |= 1u << b;
            }
            poke(base + i * 4, value);
        }
    };
    sync(kIcdIsEnable, enabled_);
    sync(kIcdIsPending, pending_);
    sync(kIcdIsActive, active_);

    u32 ppi = 0;
    for (u32 id = 16; id < 32 && id < max_irq_; ++id) {
        if (sampled_[state_index(id, access_core_)]) ppi |= 1u << (id - 16);
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
    // different fields, ICDICFR sixteen two-bit fields), so a 4 byte access is
    // routed to write_word directly.
    if (size == 4 && (offset & 3u) == 0) {
        write_word(offset, value);
        return;
    }
    if (size == 1 && ((offset >= kIcdIPriority && offset < kIcdIPriority + max_irq_) ||
                      (offset >= kIcdITargets && offset < kIcdITargets + max_irq_))) {
        const u32 word = offset & ~3u;
        const u32 shift = (offset & 3u) * 8u;
        const u32 previous_view = static_cast<u32>(read_word(word, peek(word)));
        write_word(word, (previous_view & ~(0xFFu << shift)) | ((static_cast<u32>(value) & 0xFFu) << shift));
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
                const u32 id = i * 32u + b;
                const size_t index = state_index(id, access_core_);
                if (nonsecure_access_ && !group1_[index]) continue;
                if ((value32 >> b) & 1u) {
                    if (&target != &enabled_ || id >= 16u) target[index] = state;
                    if (&target == &pending_ && !state && id < 16u) sgi_sources_[access_core_][id] = 0;
                }
            }
            poke(offset, value32);
            return true;
        }
        return false;
    };

    if (offset == kIcdDcr) {
        poke(offset, nonsecure_access_ ? (peek(offset) & ~2ull) | ((value32 & 1u) << 1) : value32 & 3u);
        return;
    }
    if (offset >= kIcdIsr && offset < kIcdIsr + ((max_irq_ + 31u) / 32u) * 4u) {
        if (!nonsecure_access_) {
            const u32 first = ((offset - kIcdIsr) / 4u) * 32u;
            for (u32 bit = 0; bit < 32u && first + bit < max_irq_; ++bit)
                group1_[state_index(first + bit, access_core_)] = ((value32 >> bit) & 1u) != 0u;
            poke(offset, value32);
        }
        return;
    }
    if (offset == kIcdIctr || offset == kIcdIidr) return;
    if (set_bits(kIcdIsEnable, enabled_, true)) return;
    if (set_bits(kIcdIcEnable, enabled_, false)) return;
    if (set_bits(kIcdIsPending, pending_, true)) return;
    if (set_bits(kIcdIcPending, pending_, false)) return;
    if (set_bits(kIcdIsActive, active_, true)) return;
    if (set_bits(kIcdIcActive, active_, false)) return;

    if (offset >= kIcdIPriority && offset < kIcdIPriority + max_irq_) {
        for (u32 i = 0; i < 4; ++i) {
            const u32 id = offset - kIcdIPriority + i;
            if (id >= max_irq_) break;
            const size_t index = state_index(id, access_core_);
            if (nonsecure_access_ && !group1_[index]) continue;
            const u8 written = static_cast<u8>((value32 >> (i * 8)) & 0xFF);
            priority_[index] = nonsecure_access_ ? 0x80u | (written >> 1) : written;
        }
        u32 canonical = 0;
        for (u32 lane = 0; lane < 4u && offset - kIcdIPriority + lane < max_irq_; ++lane)
            canonical |= static_cast<u32>(priority_[state_index(offset - kIcdIPriority + lane, access_core_)]) << (lane * 8u);
        poke(offset, canonical);
        return;
    }
    if (offset >= kIcdITargets && offset < kIcdITargets + max_irq_) {
        for (u32 i = 0; i < 4; ++i) {
            const u32 id = offset - kIcdITargets + i;
            if (id >= max_irq_) break;
            if (id < 32u || (nonsecure_access_ && !group1(id, access_core_))) continue;
            targets_[id] = static_cast<u8>((value32 >> (i * 8)) & ((1u << kGicCoreCount) - 1u));
        }
        poke(offset, value32);
        return;
    }
    // ICDICFRn covers IDs 16*n..16*n+15. SGIs occupy word zero, PPIs
    // word one, and SPIs start at word two. Bit 1 selects edge sensitivity.
    if (offset >= kIcdIConfig && offset < kIcdIConfig + ((max_irq_ + 15u) / 16u) * 4u) {
        if (offset < kIcdIConfig + 8u) return;  // SGI/PPI fields are read-only.
        u32 canonical = 0;
        for (u32 i = 0; i < 16; ++i) {
            const u32 id = ((offset - kIcdIConfig) / 4u) * 16u + i;
            if (id >= max_irq_) break;
            const size_t index = state_index(id, access_core_);
            if (!nonsecure_access_ || group1_[index])
                level_[index] = ((value32 >> (i * 2 + 1)) & 1u) == 0;
            canonical |= (level_[index] ? 1u : 3u) << (i * 2u);
        }
        poke(offset, canonical);
        return;
    }
    if (offset == kIcdSgir) {
        const u32 id = value32 & 15u;
        const bool requested_group1 = nonsecure_access_ || (value32 & (1u << 15)) != 0u;
        const u32 filter = (value32 >> 24) & 3u;
        u32 targets = (value32 >> 16) & 0xFu;
        if (filter == 1u) targets = 0xFu & ~(1u << access_core_);
        else if (filter == 2u) targets = 1u << access_core_;
        else if (filter == 3u) return;
        for (unsigned core = 0; core < kGicCoreCount; ++core) {
            if ((targets & (1u << core)) == 0u || group1(id, core) != requested_group1) continue;
            sgi_sources_[core][id] |= static_cast<u8>(1u << access_core_);
            pending_[state_index(id, core)] = true;
        }
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

void GicDistributor::save_state(StateWriter& writer) const {
    // Register image (the enable/pending/active status words) first.
    RegisterBlock::save_state(writer);
    // `max_irq_` is construction-time configuration (it sizes every vector
    // below); `access_core_`/`nonsecure_access_` are the per-access view.
    writer.put_u32(access_core_);
    writer.put_bool(nonsecure_access_);
    writer.put_bool(secure_world_left_enabled_);
    writer.list(group1_, [&](bool bit) { writer.put_bool(bit); });
    writer.list(enabled_, [&](bool bit) { writer.put_bool(bit); });
    writer.list(pending_, [&](bool bit) { writer.put_bool(bit); });
    writer.list(active_, [&](bool bit) { writer.put_bool(bit); });
    writer.list(level_, [&](bool bit) { writer.put_bool(bit); });
    writer.list(sampled_, [&](bool bit) { writer.put_bool(bit); });
    writer.list(pending_after_eoi_, [&](bool bit) { writer.put_bool(bit); });
    writer.list(priority_, [&](u8 value) { writer.put_u8(value); });
    writer.list(targets_, [&](u8 value) { writer.put_u8(value); });
    writer.list(pulse_left_, [&](u32 value) { writer.put_u32(value); });
    writer.fixed(sgi_sources_, [&](const std::array<u8, 16>& sources) {
        writer.fixed(sources, [&](u8 source) { writer.put_u8(source); });
    });
}

void GicDistributor::load_state(StateReader& reader) {
    RegisterBlock::load_state(reader);
    access_core_ = reader.get_u32();
    nonsecure_access_ = reader.get_bool();
    secure_world_left_enabled_ = reader.get_bool();
    reader.list(group1_, [&](bool& bit) { bit = reader.get_bool(); });
    reader.list(enabled_, [&](bool& bit) { bit = reader.get_bool(); });
    reader.list(pending_, [&](bool& bit) { bit = reader.get_bool(); });
    reader.list(active_, [&](bool& bit) { bit = reader.get_bool(); });
    reader.list(level_, [&](bool& bit) { bit = reader.get_bool(); });
    reader.list(sampled_, [&](bool& bit) { bit = reader.get_bool(); });
    reader.list(pending_after_eoi_, [&](bool& bit) { bit = reader.get_bool(); });
    reader.list(priority_, [&](u8& value) { value = reader.get_u8(); });
    reader.list(targets_, [&](u8& value) { value = reader.get_u8(); });
    reader.list(pulse_left_, [&](u32& value) { value = reader.get_u32(); });
    reader.fixed(sgi_sources_, [&](std::array<u8, 16>& sources) {
        reader.fixed(sources, [&](u8& source) { source = reader.get_u8(); });
    });
}

// ---------------------------------------------------------------------------
// Gic
// ---------------------------------------------------------------------------

// The GIC device covers the whole MPCore private region. That is deliberate:
// `Bus::find_device` picks the *smallest* matching window, so the SCU and
// timers still win for their own pages. The adjacent PL310 page is outside
// this window.
Gic::Gic(u32 region_base, u32 region_size, u32 distrib_base, u32 distrib_size, u32 cpuif_base, u32 cpuif_size,
         std::unique_ptr<Device> cpuif_alias, std::unique_ptr<GicCpuInterface> cpuif)
    : Device("Kermit.GIC", region_base, region_size), cpu_interface_alias_(std::move(cpuif_alias)) {
    distributor_ = std::make_unique<GicDistributor>("Kermit.GIC.distributor", distrib_base, distrib_size, kGicMaxIrq);
    cpu_interfaces_[0] = std::move(cpuif);
    for (unsigned core = 1; core < kGicCoreCount; ++core)
        cpu_interfaces_[core] = std::make_unique<GicCpuInterface>(format("Kermit.GIC.cpu%u", core), cpuif_base, cpuif_size);
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        auto& interface = *cpu_interfaces_[core];
        interface.set_sources([this, core]() { return highest_pending(core, true); });
        interface.set_running_priority_source([this, core]() { return running_priority(core); });
        interface.set_ack_handler([this, core]() {
            const u32 eligible = highest_pending(core, true);
            if (eligible >= 1022u) return eligible;
            const bool forced = gic_effective_left_enabled(secure_world_left_enabled_);
            const u32 token = distributor_->acknowledge(core, forced ? 0xFFu : cpu_interfaces_[core]->priority_mask());
            if ((token & 0x3FFu) != kSpurious) active_[core].push_back(token);
            refresh_line();
            return token;
        });
        interface.set_eoi_handler([this, core](u32 token) {
            const u32 id = token & 0x3FFu;
            if (id >= kGicMaxIrq) return;
            auto& stack = active_[core];
            if (stack.empty() || (stack.back() & 0x3FFu) != id) return;
            if (cpu_interfaces_[core]->nonsecure_access() && !distributor_->group1(id, core)) return;
            stack.pop_back();
            distributor_->end_of_interrupt(core, id);
            refresh_line();
        });
    }
}

u8 Gic::running_priority(unsigned core) const {
    u8 result = 0xFF;
    for (u32 token : active_[core])
        result = std::min(result, static_cast<u8>(distributor_->priority(token & 0x3FFu, core)));
    return result;
}

u32 Gic::highest_pending(unsigned core, bool acknowledge_view) const {
    const auto& interface = *cpu_interfaces_[core];
    const bool forced = gic_effective_left_enabled(secure_world_left_enabled_);
    const u32 control = static_cast<u32>(interface.peek(kIccIcr));
    const u32 id = distributor_->highest_pending(core, forced ? 0xFFu : interface.priority_mask());
    if (id == kSpurious) return id;
    const bool group1 = distributor_->group1(id, core);
    if (!forced && (control & (group1 ? 2u : 1u)) == 0u) return kSpurious;
    if (!active_[core].empty()) {
        unsigned binary_point = static_cast<unsigned>(interface.peek(kIccBpr) & 7u);
        if (group1 && (control & (1u << 4)) == 0u) {
            binary_point = static_cast<unsigned>(interface.peek(kIccAbpr) & 7u);
            if (binary_point != 0u) --binary_point;
        }
        const u8 group_mask = static_cast<u8>(0xFFu << (binary_point + 1u));
        if (distributor_->priority(id, core) >= (running_priority(core) & group_mask)) return kSpurious;
    }
    if (acknowledge_view) {
        if (interface.nonsecure_access() && !group1) return kSpurious;
        if (!interface.nonsecure_access() && group1 && (control & (1u << 2)) == 0u) return 1022u;
    }
    return id;
}

u64 Gic::read(u32 address, unsigned size) {
    const unsigned core = access_context_ && access_context_->core_id < kGicCoreCount ? access_context_->core_id : 0u;
    const bool nonsecure = access_context_ && access_context_->nonsecure;
    if (distributor_->handles(address)) {
        distributor_->set_access_context(core, nonsecure);
        return distributor_->read(address, size);
    }
    auto& interface = *cpu_interfaces_[core];
    interface.set_nonsecure_access(nonsecure);
    if (interface.handles(address)) return interface.read(address, size);
    if (cpu_interface_alias_ && cpu_interface_alias_->handles(address))
        return interface.read(interface.base() + address - cpu_interface_alias_->base(), size);
    return 0;
}

void Gic::write(u32 address, unsigned size, u64 value) {
    const unsigned core = access_context_ && access_context_->core_id < kGicCoreCount ? access_context_->core_id : 0u;
    const bool nonsecure = access_context_ && access_context_->nonsecure;
    if (distributor_->handles(address)) {
        distributor_->set_access_context(core, nonsecure);
        distributor_->write(address, size, value);
    } else {
        auto& interface = *cpu_interfaces_[core];
        interface.set_nonsecure_access(nonsecure);
        if (interface.handles(address)) interface.write(address, size, value);
        else if (cpu_interface_alias_ && cpu_interface_alias_->handles(address))
            interface.write(interface.base() + address - cpu_interface_alias_->base(), size, value);
        else return;
    }
    refresh_line();
}

void Gic::reset() {
    distributor_->reset();
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        cpu_interfaces_[core]->reset();
        cpu_interfaces_[core]->set_nonsecure_access(false);
        active_[core].clear();
        irq_lines_[core] = false;
        fiq_lines_[core] = false;
        if (cpus_[core]) {
            cpus_[core]->set_irq(static_cast<int>(IrqLine::Irq), false);
            cpus_[core]->set_irq(static_cast<int>(IrqLine::FiQ), false);
        }
    }
    line_ = false;
    if (line_callback_) line_callback_(false);
}

void Gic::tick(u64 cycles) {
    if (distributor_->tick_pulses(cycles)) refresh_line();
}

const char* Gic::register_name(u32 address) const {
    if (const char* name = distributor_->register_name(address)) return name;
    return cpu_interfaces_[0]->register_name(address);
}

void Gic::enumerate_registers(std::vector<RegisterInfo>& out) const {
    distributor_->enumerate_registers(out);
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        std::vector<RegisterInfo> registers;
        cpu_interfaces_[core]->enumerate_registers(registers);
        for (auto& reg : registers) {
            reg.name = format("CPU%u.%s", core, reg.name.c_str());
            out.push_back(std::move(reg));
        }
    }
}

bool Gic::peek_register(const std::string& name, u64& out) const {
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        const std::string prefix = format("CPU%u.", core);
        if (name.compare(0, prefix.size(), prefix) != 0) continue;
        const std::string field = name.substr(prefix.size());
        if (field == "IRQ") { out = irq_lines_[core]; return true; }
        if (field == "FIQ") { out = fiq_lines_[core]; return true; }
        return cpu_interfaces_[core]->peek_register(field, out);
    }
    if (distributor_->peek_register(name, out)) return true;
    return cpu_interfaces_[0]->peek_register(name, out);
}

bool Gic::poke_register(const std::string& name, u64 value) {
    bool found = false;
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        const std::string prefix = format("CPU%u.", core);
        if (name.compare(0, prefix.size(), prefix) == 0)
            found = cpu_interfaces_[core]->poke_register(name.substr(prefix.size()), value);
    }
    if (!found) found = distributor_->poke_register(name, value) || cpu_interfaces_[0]->poke_register(name, value);
    if (found) refresh_line();
    return found;
}

void Gic::set_cpu(Cpu* cpu, unsigned core) {
    if (core >= kGicCoreCount) return;
    cpus_[core] = cpu;
    if (cpu) {
        cpu->set_irq(static_cast<int>(IrqLine::Irq), irq_lines_[core]);
        cpu->set_irq(static_cast<int>(IrqLine::FiQ), fiq_lines_[core]);
    }
}

void Gic::refresh_line() {
    static const bool log_gic = [] {
        const char* value = std::getenv("ZLB_GIC_LOG");
        return value && value[0] != '0';
    }();
    // IHI0048B.b section3.7: classify the highest pending interrupt first, then
    // choose IRQ or FIQ with that CPU interface's Secure FIQEn bit.
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        const u32 winning = highest_pending(core);
        const bool asserted = winning != kSpurious;
        const bool fiq = asserted && !distributor_->group1(winning, core) &&
                         (cpu_interfaces_[core]->peek(kIccIcr) & (1u << 3)) != 0u;
        const bool irq = asserted && !fiq;
        if (log_gic && (irq != irq_lines_[core] || fiq != fiq_lines_[core]))
            ZLB_LOG_INFO("kermit", "GIC cpu%u IRQ=%u FIQ=%u icddcr=%08X iccicr=%08X pmr=%02X id=%u",
                         core, irq, fiq, static_cast<unsigned>(distributor_->peek(kIcdDcr)),
                         static_cast<unsigned>(cpu_interfaces_[core]->peek(kIccIcr)),
                         cpu_interfaces_[core]->priority_mask(), winning);
        if (irq != irq_lines_[core]) {
            irq_lines_[core] = irq;
            if (core == 0 && line_callback_) line_callback_(irq);
            if (cpus_[core]) cpus_[core]->set_irq(static_cast<int>(IrqLine::Irq), irq);
        }
        if (fiq != fiq_lines_[core]) {
            fiq_lines_[core] = fiq;
            if (cpus_[core]) cpus_[core]->set_irq(static_cast<int>(IrqLine::FiQ), fiq);
        }
    }
    line_ = irq_lines_[0];
}

std::string Gic::summary() const {
    unsigned irq_mask = 0, fiq_mask = 0;
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        if (irq_lines_[core]) irq_mask |= 1u << core;
        if (fiq_lines_[core]) fiq_mask |= 1u << core;
    }
    return format("%s IRQ=%X FIQ=%X pending=%u", name_.c_str(), irq_mask, fiq_mask, distributor_->pending_count());
}

void Gic::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s", summary().c_str()));
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        lines.push_back(format("    cpu%u Secure ICCICR=%08X PMR=%02X IRQ=%u FIQ=%u active=%zu", core,
                              static_cast<unsigned>(cpu_interfaces_[core]->peek(kIccIcr)),
                              cpu_interfaces_[core]->priority_mask(), irq_lines_[core], fiq_lines_[core], active_[core].size()));
        for (u32 token : active_[core]) lines.push_back(format("      servicing %s", irq_label(token & 0x3FFu).c_str()));
    }
    distributor_->describe(lines);
}

void Gic::save_state(StateWriter& writer) const {
    // The distributor and the CPU interfaces are owned here, not registered on
    // the bus, so each travels in its own named section.
    writer.begin("Gic.distributor");
    distributor_->save_state(writer);
    writer.end();
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        writer.begin("Gic.cpu_interface");
        cpu_interfaces_[core]->save_state(writer);
        writer.end();
    }
    // The optional generic CPU-interface alias is wiring, but it is a Device
    // that is never walked by the bus, so its state (if fitted) is written too.
    writer.put_bool(cpu_interface_alias_ != nullptr);
    if (cpu_interface_alias_) {
        writer.begin("Gic.cpu_interface_alias");
        cpu_interface_alias_->save_state(writer);
        writer.end();
    }
    writer.fixed(active_, [&](const std::vector<u32>& stack) {
        writer.list(stack, [&](u32 token) { writer.put_u32(token); });
    });
    writer.fixed(irq_lines_, [&](bool asserted) { writer.put_bool(asserted); });
    writer.fixed(fiq_lines_, [&](bool asserted) { writer.put_bool(asserted); });
    writer.put_bool(line_);
    writer.put_bool(secure_world_left_enabled_);
    // `cpus_` and `line_callback_` are host wiring, `access_context_` points at
    // the bus context: none of them is serialised.
}

void Gic::load_state(StateReader& reader) {
    reader.begin("Gic.distributor");
    distributor_->load_state(reader);
    reader.end();
    for (unsigned core = 0; core < kGicCoreCount; ++core) {
        reader.begin("Gic.cpu_interface");
        cpu_interfaces_[core]->load_state(reader);
        reader.end();
    }
    const bool has_alias = reader.get_bool();
    if (has_alias) {
        reader.begin("Gic.cpu_interface_alias");
        if (cpu_interface_alias_) cpu_interface_alias_->load_state(reader);
        reader.end();
    }
    reader.fixed(active_, [&](std::vector<u32>& stack) {
        reader.list(stack, [&](u32& token) { token = reader.get_u32(); });
    });
    reader.fixed(irq_lines_, [&](bool& asserted) { asserted = reader.get_bool(); });
    reader.fixed(fiq_lines_, [&](bool& asserted) { asserted = reader.get_bool(); });
    line_ = reader.get_bool();
    secure_world_left_enabled_ = reader.get_bool();
    // `line_` is derived from core 0's IRQ record; keep the two consistent with
    // the value the state was taken with. The callback is not fired here - the
    // CPU cores restore their own interrupt inputs right after this.
    irq_lines_[0] = line_;
}

}  // namespace zlb::kermit
