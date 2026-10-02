// zeliboba - Cortex-A9 MPCore global timer, private timers and watchdog.
//
// Register offsets and semantics come straight from the Cortex-A9 MPCore TRM
// (datasheets/cortex_a9_mpcore_trm_100486_0401_10_en.pdf):
//
//   * chapter 4.2 "Private timer and watchdog registers" (page 4-64),
//     table 4-1: 0x00 load, 0x04 counter, 0x08 control, 0x0C interrupt status,
//     0x20 watchdog load, 0x24 watchdog counter, 0x28 watchdog control,
//     0x2C watchdog interrupt status, 0x30 watchdog reset status,
//     0x34 watchdog disable;
//   * chapter 4.4 "Global timer registers" (page 4-71), table 4-4:
//     0x00/0x04 64 bit counter, 0x08 control, 0x0C interrupt status,
//     0x10/0x14 comparator, 0x18 auto-increment;
//   * the region offsets inside the private memory region come from table 1-3
//     (page 1-17): global timer at +0x0200, private timer/watchdog at +0x0600.
//
// Clocking: the timers are clocked by PERIPHCLK. KermitBlock::tick converts the
// CPU cycle count into PERIPHCLK ticks (see kCpuClockHz / kPeriphClockHz in
// soc_internal.h) and hands those to these devices, so everything below counts
// microseconds.
//
// The private timer raises PPI 29 (TRM 4.2.3, page 4-65), the watchdog raises
// PPI 30 and the global timer comparator raises PPI 27 (TRM 4.3, page 4-70).
#include "hw/soc/soc_internal.h"

#include <limits>

namespace zlb::kermit {

namespace {

// Private timer / watchdog register offsets (TRM table 4-1).
constexpr u32 kTimerLoad = 0x00;
constexpr u32 kTimerCounter = 0x04;
constexpr u32 kTimerControl = 0x08;
constexpr u32 kTimerStatus = 0x0C;
constexpr u32 kWatchdogLoad = 0x20;
constexpr u32 kWatchdogCounter = 0x24;
constexpr u32 kWatchdogControl = 0x28;
constexpr u32 kWatchdogStatus = 0x2C;
constexpr u32 kWatchdogResetStatus = 0x30;
constexpr u32 kWatchdogDisable = 0x34;

constexpr u32 kTimerEnable = 1u << 0;
constexpr u32 kTimerAutoReload = 1u << 1;
constexpr u32 kTimerIrqEnable = 1u << 2;

// Global timer register offsets (TRM table 4-4).
constexpr u32 kGlobalCounterLow = 0x00;
constexpr u32 kGlobalCounterHigh = 0x04;
constexpr u32 kGlobalControl = 0x08;
constexpr u32 kGlobalStatus = 0x0C;
constexpr u32 kGlobalComparatorLow = 0x10;
constexpr u32 kGlobalComparatorHigh = 0x14;
constexpr u32 kGlobalAutoIncrement = 0x18;

/// Convert PERIPHCLK ticks into CPU cycles. Both are compile time constants.
inline u64 periph_to_cycles(u64 ticks) { return cycles_from_periph_ticks(ticks); }

}  // namespace

// ---------------------------------------------------------------------------
// GlobalTimer
// ---------------------------------------------------------------------------

GlobalTimer::GlobalTimer(std::string name, u32 base, u32 size) : RegisterBlock(std::move(name), base, size) {
    define(kGlobalCounterLow, "GTCNT_LO", 0);
    define(kGlobalCounterHigh, "GTCNT_HI", 0);
    define(kGlobalControl, "GTCTRL", 0);
    define(kGlobalStatus, "GTSTATUS", 0);
    define(kGlobalComparatorLow, "GTCMP_LO", 0);
    define(kGlobalComparatorHigh, "GTCMP_HI", 0);
    define(kGlobalAutoIncrement, "GTAUTOINC", 0);
}

void GlobalTimer::reset() {
    RegisterBlock::reset();
    counter_ = 0;
    comparator_ = 0;
    accumulator_ = 0;
    enabled_ = false;
    if (irq_state_) {
        irq_state_ = false;
        if (irq_) irq_(kIrqPpiGlobalTimer, false);
    }
}

u64 GlobalTimer::read_word(u32 offset, u64 stored) {
    switch (offset) {
        case kGlobalCounterLow:
            return counter_ & 0xFFFFFFFFull;
        case kGlobalCounterHigh:
            return (counter_ >> 32) & 0xFFFFFFFFull;
        case kGlobalControl:
            return enabled_ ? 1u : 0u;
        case kGlobalStatus:
            return irq_state_ ? 1u : 0u;
        case kGlobalComparatorLow:
            return comparator_ & 0xFFFFFFFFull;
        case kGlobalComparatorHigh:
            return (comparator_ >> 32) & 0xFFFFFFFFull;
        default:
            return stored;
    }
}

void GlobalTimer::write_word(u32 offset, u64 value) {
    switch (offset) {        case kGlobalCounterLow:
            counter_ = (counter_ & 0xFFFFFFFF00000000ull) | (value & 0xFFFFFFFFull);
            return;
        case kGlobalCounterHigh:
            counter_ = (counter_ & 0xFFFFFFFFull) | ((value & 0xFFFFFFFFull) << 32);
            return;
        case kGlobalControl:
            enabled_ = (value & 1u) != 0;
            poke(kGlobalControl, value & 1u);
            refresh_irq();
            return;
        case kGlobalStatus:
            // Write one to clear.  The latch has to go with it, otherwise the next
            // refresh_irq() re-derives "fired" from counter >= comparator and puts the
            // bit (and the line) straight back, so the interrupt could never be
            // acknowledged - measured: the status bit stayed 1 after the clear and the
            // core's IRQ line never dropped (docs/KBL.md 7.1.40).
            if (value & 1u) {
                fired_ = false;
                poke(kGlobalStatus, 0);
                refresh_irq();
            }
            return;
        case kGlobalComparatorLow:
            comparator_ = (comparator_ & 0xFFFFFFFF00000000ull) | (value & 0xFFFFFFFFull);
            refresh_irq();
            return;
        case kGlobalComparatorHigh:
            comparator_ = (comparator_ & 0xFFFFFFFFull) | ((value & 0xFFFFFFFFull) << 32);
            refresh_irq();
            return;
        default:
            poke(offset, value);
            return;
    }
}

void GlobalTimer::refresh_irq() {
    // TRM 4.3: "the comparators for each processor with the global timer fire
    // when the timer value is greater than or equal to [the comparator]".  The
    // event is a *crossing*, latched in the status register and cleared by the
    // guest; the counter staying past the comparator must not re-assert it, or the
    // interrupt can never be acknowledged (the core would take it forever on an
    // edge-triggered line and spin in a level-triggered one).
    const bool reached = enabled_ && comparator_ != 0 && counter_ >= comparator_;
    if (!reached) {
        armed_ = true;    // below the comparator again: ready for the next crossing
    } else if (armed_) {
        armed_ = false;
        fired_ = true;
        poke(kGlobalStatus, 1);
    }
    const bool want = fired_ || (peek(kGlobalStatus) & 1u) != 0;
    if (want == irq_state_) return;
    irq_state_ = want;
    if (irq_) irq_(kIrqPpiGlobalTimer, irq_state_);
}

void GlobalTimer::tick(u64 cycles) {
    // KermitBlock::tick hands out PERIPHCLK ticks, which is what the global
    // timer counts.
    const u64 ticks = cycles;
    if (ticks == 0) return;
    // The counter is free running: on the Cortex-A9 the global timer's GTCNT
    // counts PERIPHCLK as long as the block is clocked, and the enable bit only
    // gates the comparator/interrupt logic - not the count itself.  Gating the
    // increment on `enabled_` made GTCNT stand still for a guest that reads it
    // without programming the control register (measured: the boot chain never
    // writes the MPCore timers at all, so the counter stayed 0 for the whole run
    // and no timer could ever be a wake-up source - docs/KBL.md 7.1.40).
    counter_ += ticks;
    if (!enabled_) return;
    if (comparator_ != 0 && counter_ >= comparator_) refresh_irq();
}

std::string GlobalTimer::summary() const {
    return format("%s counter=%llu%s cmp=%llu", name_.c_str(), static_cast<unsigned long long>(counter_),
                  enabled_ ? "" : " (stopped)", static_cast<unsigned long long>(comparator_));
}

void GlobalTimer::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s: %s counter=%llu us cmp=%llu irq=%s", name_.c_str(),
                           enabled_ ? "running" : "stopped", static_cast<unsigned long long>(counter_),
                           static_cast<unsigned long long>(comparator_), irq_state_ ? "asserted" : "idle"));
}

// ---------------------------------------------------------------------------
// PrivateTimer
// ---------------------------------------------------------------------------

PrivateTimer::PrivateTimer(std::string name, u32 base, u32 size, u32 core, u32 timer_irq, u32 watchdog_irq)
    : RegisterBlock(std::move(name), base, size), core_(core), timer_irq_(timer_irq), watchdog_irq_(watchdog_irq) {
    define(kTimerLoad, "PTIMER_LOAD", 0);
    define(kTimerCounter, "PTIMER_COUNTER", 0);
    define(kTimerControl, "PTIMER_CONTROL", 0);
    define(kTimerStatus, "PTIMER_STATUS", 0);
    define(kWatchdogLoad, "WDT_LOAD", 0);
    define(kWatchdogCounter, "WDT_COUNTER", 0);
    define(kWatchdogControl, "WDT_CONTROL", 0);
    define(kWatchdogStatus, "WDT_STATUS", 0);
    define(kWatchdogResetStatus, "WDT_RESET_STATUS", 0);
    define(kWatchdogDisable, "WDT_DISABLE", 0);
}

void PrivateTimer::reset() {
    RegisterBlock::reset();
    timer_ = Counter{};
    watchdog_ = Counter{};
    watchdog_reset_status_ = false;
    watchdog_disabled_ = false;
    accumulator_ = 0;
    if (irq_) {
        irq_(timer_irq_, false);
        irq_(watchdog_irq_, false);
    }
}

void PrivateTimer::refresh(Counter& counter, u32 id) {
    if (counter.event) poke(&counter == &timer_ ? kTimerStatus : kWatchdogStatus, 1);
    const bool want = counter.event && counter.irq_enable;
    if (irq_) irq_(id, want);
}

void PrivateTimer::step(Counter& counter, u32 id, u64 periph_ticks) {
    if (!counter.running) return;
    u64 left = periph_ticks;
    while (left > 0) {
        const u32 per_tick = static_cast<u32>(counter.prescaler) + 1;
        const u32 need = per_tick - counter.prescale_left;
        if (left < need) {
            counter.prescale_left += static_cast<u32>(left);
            return;
        }
        left -= need;
        counter.prescale_left = 0;
        if (counter.value == 0) {
            counter.event = true;
            ++expiries_;
            refresh(counter, id);
            if (counter.auto_reload) {
                counter.value = counter.load;
            } else {
                counter.running = false;
                return;
            }
        }
        if (counter.value > 0) --counter.value;
    }
}

void PrivateTimer::tick(u64 cycles) {
    // KermitBlock::tick hands out PERIPHCLK ticks, which is what the private
    // timer's prescaler divides.
    const u64 ticks = cycles;
    if (ticks == 0) return;
    step(timer_, timer_irq_, ticks);
    step(watchdog_, watchdog_irq_, ticks);
}

u64 PrivateTimer::read_word(u32 offset, u64 stored) {
    switch (offset) {
        case kTimerCounter:
            return timer_.value;
        case kTimerStatus:
            return timer_.event ? 1u : 0u;
        case kWatchdogCounter:
            return watchdog_.value;
        case kWatchdogStatus:
            return watchdog_.event ? 1u : 0u;
        case kWatchdogResetStatus:
            return watchdog_reset_status_ ? 1u : 0u;
        default:
            return stored;
    }
}

void PrivateTimer::write_word(u32 offset, u64 value) {
    const u32 value32 = static_cast<u32>(value);
    switch (offset) {
        case kTimerLoad:
            timer_.load = value32;
            timer_.value = value32;
            poke(kTimerLoad, value32);
            return;
        case kTimerCounter:
            timer_.value = value32;
            poke(kTimerCounter, value32);
            return;
        case kTimerControl:
            timer_.prescaler = static_cast<u8>((value32 >> 8) & 0xFF);
            timer_.irq_enable = (value32 & kTimerIrqEnable) != 0;
            timer_.auto_reload = (value32 & kTimerAutoReload) != 0;
            timer_.running = (value32 & kTimerEnable) != 0;
            timer_.prescale_left = 0;
            poke(kTimerControl, value32);
            refresh(timer_, timer_irq_);
            return;
        case kTimerStatus:
            if (value32 & 1u) {
                timer_.event = false;
                poke(kTimerStatus, 0);
                refresh(timer_, timer_irq_);
            }
            return;
        case kWatchdogLoad:
            watchdog_.load = value32;
            watchdog_.value = value32;
            poke(kWatchdogLoad, value32);
            return;
        case kWatchdogCounter:
            watchdog_.value = value32;
            poke(kWatchdogCounter, value32);
            return;
        case kWatchdogControl:
            watchdog_.prescaler = static_cast<u8>((value32 >> 8) & 0xFF);
            watchdog_.irq_enable = (value32 & kTimerIrqEnable) != 0;
            watchdog_.auto_reload = (value32 & kTimerAutoReload) != 0;
            watchdog_.running = (value32 & kTimerEnable) != 0;
            watchdog_.prescale_left = 0;
            poke(kWatchdogControl, value32);
            refresh(watchdog_, watchdog_irq_);
            return;
        case kWatchdogStatus:
            if (value32 & 1u) {
                watchdog_.event = false;
                poke(kWatchdogStatus, 0);
                refresh(watchdog_, watchdog_irq_);
            }
            return;
        case kWatchdogResetStatus:
            // nWDRESET resets this register only; writes are ignored.
            return;
        case kWatchdogDisable:
            // 0x12345678 (software lock) then 0x87654321 (disable), per the
            // TRM 4.2.11 description.
            if (watchdog_disabled_ || value32 == 0x87654321u) {
                watchdog_.running = false;
                watchdog_disabled_ = true;
                poke(kWatchdogDisable, 1);
            } else if (value32 == 0x12345678u) {
                watchdog_disabled_ = false;
                poke(kWatchdogDisable, 0);
            }
            return;
        default:
            poke(offset, value32);
            return;
    }
}

std::string PrivateTimer::summary() const {
    return format("%s core%u timer=%s(%u) wdt=%s(%u) expiries=%llu", name_.c_str(), core_,
                  timer_.running ? "run" : "stop", timer_.value, watchdog_.running ? "run" : "stop",
                  watchdog_.value, static_cast<unsigned long long>(expiries_));
}

void PrivateTimer::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s (core %u): private timer %s load=%u counter=%u irq=%s", name_.c_str(), core_,
                           timer_.running ? "running" : "stopped", timer_.load, timer_.value,
                           timer_.event ? "flag" : "-"));
    lines.push_back(format("    watchdog: %s load=%u counter=%u reset_status=%u disabled=%u",
                           watchdog_.running ? "running" : "stopped", watchdog_.load, watchdog_.value,
                           watchdog_reset_status_ ? 1u : 0u, watchdog_disabled_ ? 1u : 0u));
}

// ---------------------------------------------------------------------------
// Vita LT5 / WT7, outside the Cortex-A9 private peripheral region.
// ---------------------------------------------------------------------------
// Supplied FW1.04: KBL4002158E..1598 programs LT5; ThreadMgr4B762C..76C4
// writes deadlines and IRQ141 handler4B7B18 acknowledges2 at+18. Secure
// IntrMgr3BF148..214 programs WT7, and IRQ135 handler3BF000..136 stops it,
// acknowledges3 at+14 and releases genuine per-core locks when current>due.
// https://www.psdevwiki.com/vita/Hardware_Timers corroborates layouts,
// enable bit0 and source/prescaler fields, but leaves lower mode bits uncertain.
// See build/goal-native-systimer-proposal.md for the complete evidence/limits.
//
// Model choices: fixed222 MHz WT input; comparison on elapsed counting ticks,
// with a latch and explicit rearming; only compare status bit1 is projected
// (proven by LT5 ACK2, inferred for the same family's WT ACK3). Overflow event
// bit0, auto-reload/capture modes and auxiliary counter behavior remain inert.
// Nothing increments on reads or changes a guest lock/service return.

VitaSystemTimer::VitaSystemTimer(std::string name, u32 base, bool longrange, u32 irq_id)
    : Device(std::move(name), base, kVitaTimerSize), longrange_(longrange), irq_id_(irq_id) {
    if (longrange_) {
        register_name_entry(base_ + 0x00, "COUNTER_LO");
        register_name_entry(base_ + 0x04, "COUNTER_HI");
        register_name_entry(base_ + 0x08, "DEADLINE_LO");
        register_name_entry(base_ + 0x0C, "DEADLINE_HI");
        register_name_entry(base_ + 0x10, "AUX_LO");
        register_name_entry(base_ + 0x14, "AUX_HI");
    } else {
        register_name_entry(base_ + 0x00, "DEADLINE");
        register_name_entry(base_ + 0x04, "COUNTER");
        register_name_entry(base_ + 0x0C, "AUX_0C");
        register_name_entry(base_ + 0x10, "AUX_10");
    }
    register_name_entry(base_ + config_offset(), "CONFIG");
    register_name_entry(base_ + status_offset(), "STATUS");
}

bool VitaSystemTimer::known_offset(u32 offset) const {
    return (offset & 3u) == 0 && offset <= (longrange_ ? 0x1Cu : 0x14u);
}

bool VitaSystemTimer::supported_config(u32 config) const {
    // Prescale[31:24] is documented; all other fields must match a captured
    // profile. Retain opaque LT5 fields345000 without guessing their meaning.
    const u32 body = config & 0x00FFFFFFu;
    if (body == 0) return true;  // native ISR/reset stop; divider is irrelevant
    if (longrange_) return body == 0x00345008 || body == 0x0034500C || body == 0x0034500D;
    return body == 0x0000000C || body == 0x0000000D;
}

bool VitaSystemTimer::running() const {
    return !unsupported_ && (registers_[config_offset() / 4] & 1u) != 0;
}

u64 VitaSystemTimer::counter() const {
    const u32 slot = counter_offset() / 4;
    return registers_[slot] | (longrange_ ? static_cast<u64>(registers_[slot + 1]) << 32 : 0);
}

u64 VitaSystemTimer::deadline() const {
    const u32 slot = deadline_offset() / 4;
    return registers_[slot] | (longrange_ ? static_cast<u64>(registers_[slot + 1]) << 32 : 0);
}

void VitaSystemTimer::refresh_irq() {
    // Only a reached comparison profile drives the line. Gating by the
    // captured 0xC mode pattern (not individual bit names) is an inference;
    // Stop0 disables output but retains the event until native W1C follows.
    const bool want = !unsupported_ && (registers_[config_offset() / 4] & 0xCu) == 0xCu &&
                      (registers_[status_offset() / 4] & 2u) != 0;
    if (want == irq_state_) return;
    irq_state_ = want;
    if (irq_) irq_(irq_id_, want);
}

u64 VitaSystemTimer::read(u32 address, unsigned size) {
    u64 value = 0;
    for (unsigned i = 0; i < size && i < 8; ++i) {
        const u64 current = static_cast<u64>(address) + i;
        u32 byte = 0xFF;
        if (current >= base_ && current < static_cast<u64>(base_) + size_) {
            const u32 offset = static_cast<u32>(current - base_);
            if (known_offset(offset & ~3u)) {
                byte = (registers_[offset / 4] >> ((offset & 3u) * 8)) & 0xFFu;
            }
        }
        value |= static_cast<u64>(byte) << (i * 8);
    }
    return value;
}

void VitaSystemTimer::write(u32 address, unsigned size, u64 value) {
    const u32 prior_config = registers_[config_offset() / 4];
    const bool was_running = running();
    bool config_written = false;
    bool counter_or_deadline_written = false;
    for (unsigned i = 0; i < size && i < 8; ++i) {
        const u64 current = static_cast<u64>(address) + i;
        if (current < base_ || current >= static_cast<u64>(base_) + size_) continue;
        const u32 offset = static_cast<u32>(current - base_);
        const u32 word = offset & ~3u;
        if (!known_offset(word)) continue;
        const u32 shift = (offset & 3u) * 8;
        const u32 supplied = static_cast<u32>((value >> (i * 8)) & 0xFFu) << shift;
        u32& stored = registers_[word / 4];
        if (word == status_offset()) {
            stored &= ~(supplied & 3u); // zero/upper lanes never clear low pending bits
            continue;
        }
        stored = (stored & ~(0xFFu << shift)) | supplied;
        config_written |= word == config_offset();
        counter_or_deadline_written |= word == counter_offset() || word == deadline_offset() ||
                                      (longrange_ && (word == counter_offset() + 4 || word == deadline_offset() + 4));
    }
    if (config_written) {
        const u32 config = registers_[config_offset() / 4];
        unsupported_ = !supported_config(config);
        if (config != prior_config) fractional_ = 0; // explicit divider-phase reset choice
        if (unsupported_ && config != prior_config) {
            ++unsupported_configs_;
            ZLB_LOG_WARN("timer", "%s unsupported configuration %08X; count/output stopped",
                         name_.c_str(), config);
        }
        if (!was_running && running()) armed_ = true;
    }
    // Even identical counter/deadline writes are explicit programming and
    // rearm. An identical active config write and W1C alone do not rearm.
    if (counter_or_deadline_written) armed_ = true;
    refresh_irq();
}

void VitaSystemTimer::tick(u64 periph_ticks) {
    if (!running() || periph_ticks == 0) return;
    const u32 config = registers_[config_offset() / 4];
    const u64 input_hz = longrange_ ? 48000000u : kVitaTimerSysClockHz;
    const u64 divisor = static_cast<u64>((config >> 24) + 1) * kPeriphClockHz;
    // Portable quotient/remainder arithmetic also works on MSVC. The partial
    // product is bounded by 256MHz*222MHz; the whole-count product may wrap,
    // so retain its low modular value and separately detect a full64-bit span.
    const u64 whole = periph_ticks / divisor;
    const u64 remainder = periph_ticks % divisor;
    const u64 partial = remainder * input_hz + fractional_;
    const u64 extra = partial / divisor;
    fractional_ = partial % divisor;
    const bool full_span = whole > (std::numeric_limits<u64>::max() - extra) / input_hz;
    const u64 counts = whole * input_hz + extra;
    if (counts == 0 && !full_span) return;
    const u64 before = counter();
    const u64 target = deadline();
    // Unsigned absolute comparison is a model choice. A newly armed equal or
    // overdue target expires on the next actual count. Otherwise measure the
    // distance before adding, so a batch that wraps past the target still fires.
    const u64 distance = before >= target ? 1 : target - before;
    if (armed_ && (full_span || counts >= distance)) {
        armed_ = false;
        registers_[status_offset() / 4] |= 2u;
        ++comparisons_;
    }
    const u64 after = before + counts; // modular64/32 arithmetic
    const u32 slot = counter_offset() / 4;
    registers_[slot] = static_cast<u32>(after);
    if (longrange_) registers_[slot + 1] = static_cast<u32>(after >> 32);
    refresh_irq();
}

void VitaSystemTimer::reset() {
    registers_.fill(0);
    fractional_ = comparisons_ = unsupported_configs_ = 0;
    armed_ = unsupported_ = false;
    refresh_irq();
}

void VitaSystemTimer::enumerate_registers(std::vector<RegisterInfo>& out) const {
    for (const auto& entry : names_) out.push_back({entry.first, entry.second, 0, 4});
}

bool VitaSystemTimer::peek_register(const std::string& name, u64& out) const {
    if (name == "UNSUPPORTED") { out = unsupported_; return true; }
    if (name == "COMPARISONS") { out = comparisons_; return true; }
    if (name == "UNSUPPORTED_CONFIGS") { out = unsupported_configs_; return true; }
    for (const auto& entry : names_) {
        if (entry.second == name) { out = registers_[(entry.first - base_) / 4]; return true; }
    }
    return false;
}

bool VitaSystemTimer::poke_register(const std::string& name, u64 value) {
    for (const auto& entry : names_) {
        if (entry.second == name) { write(entry.first, 4, value); return true; }
    }
    return false;
}

std::string VitaSystemTimer::summary() const {
    return format("%s counter=%llu deadline=%llu config=%08X %s irq%u=%s comparisons=%llu unsupported=%llu",
                  name_.c_str(), static_cast<unsigned long long>(counter()), static_cast<unsigned long long>(deadline()),
                  registers_[config_offset() / 4], unsupported_ ? "unsupported" : running() ? "running" : "stopped",
                  irq_id_, irq_state_ ? "asserted" : "idle", static_cast<unsigned long long>(comparisons_),
                  static_cast<unsigned long long>(unsupported_configs_));
}

void VitaSystemTimer::describe(std::vector<std::string>& lines) const {
    lines.push_back("  " + summary());
    lines.push_back(format("    input=%u Hz divider=%u; compare-only model%s",
                           longrange_ ? 48000000u : kVitaTimerSysClockHz,
                           (registers_[config_offset() / 4] >> 24) + 1,
                           longrange_ ? "" : "; SysClock222 MHz is a modeled board input"));
}

}  // namespace zlb::kermit
