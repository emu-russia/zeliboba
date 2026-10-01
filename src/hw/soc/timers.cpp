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

}  // namespace zlb::kermit
