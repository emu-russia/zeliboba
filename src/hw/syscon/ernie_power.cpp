// zeliboba - Ernie power / reset / button / fuel gauge / RTC state.
//
// Everything in this file is the "board" side of the syscon: the panel buttons
// and their power-off / power-on state machine, the bq27520 fuel gauge, the
// real time clock the SC commands import and export, and the SoC release
// handshake the CMeP first loader performs (0x5C206 boot_path_bit5 and the
// 0x30000118 / 0x30000208 pair).
//
// The gauge datasheet (Abby/bq27520-g4.pdf) describes the standard command set
// the syscon firmware has to speak:
//
//   0x00 Control()      0x01 DeviceType()   0x02 FirmwareVersion()
//   0x04 SpecVersion()  0x06 Flags()        0x08 NominalAvailableCapacity()
//   0x0A FullAvailableCapacity()            0x0C RemainingCapacity()
//   0x0E FullChargeCapacity()               0x10 AverageCurrent()
//   0x12 StandbyCurrent()                   0x14 MaxLoadCurrent()
//   0x16 AveragePower()                     0x18 StateOfCharge()   <- percent
//   0x1A InternalTemperature()              0x1C Voltage()
// Those are the registers the 0x0891/0x088F/0x088E SC commands shuttle between
// the gauge and the caller, so the model has to answer them live.
#include <algorithm>
#include <cstring>

#include "common/util.h"
#include "hw/syscon.h"
#include "hw/syscon/ernie_internal.h"
#include "hw/syscon/ernie_power.h"

namespace zlb {
namespace ernie {

namespace {

/// Rated capacity of the Vita's cell in mAh (2500 mAh class pack).
constexpr u16 kDesignCapacityMah = 2500;
/// Room temperature baseline the InternalTemperature() model reports in 0.1 K.
constexpr u16 kBaseTemperatureDeciKelvin = 2981;  // 25.0 C

}  // namespace

const std::vector<GaugeRegister>& gauge_registers() {
    static const std::vector<GaugeRegister> table = {
        {0x00, "Control", "-"},
        {0x01, "DeviceType", "-"},
        {0x02, "FirmwareVersion", "-"},
        {0x04, "SpecVersion", "-"},
        {0x06, "Flags", "bitfield"},
        {0x08, "DesignCapacity", "mAh"},
        {0x0A, "FullAvailableCapacity", "mAh"},
        {0x0C, "RemainingCapacity", "mAh"},
        {0x0E, "FullChargeCapacity", "mAh"},
        {0x10, "AverageCurrent", "mA"},
        {0x12, "StandbyCurrent", "mA"},
        {0x14, "MaxLoadCurrent", "mA"},
        {0x16, "AveragePower", "mW"},
        {0x18, "StateOfCharge", "%"},
        {0x1A, "InternalTemperature", "0.1 K"},
        {0x1C, "Voltage", "mV"},
    };
    return table;
}

const GaugeRegister* gauge_register(u8 command) {
    for (const GaugeRegister& reg : gauge_registers()) {
        if (reg.command == command) return &reg;
    }
    return nullptr;
}

PowerState::PowerState() {
    sfr = nullptr;
    emmc = nullptr;
}

void PowerState::bind(ErnieSfr* sfr_block, EmmcHost* host) {
    sfr = sfr_block;
    emmc = host;
    apply();
}

void PowerState::apply() {
    if (sfr == nullptr) return;
    sfr->set_milliseconds(milliseconds);
    sfr->set_rtc_seconds(rtc_seconds);
    sfr->set_battery_percent(battery_percent);
    sfr->set_charger_state(charger_state);
    sfr->set_battery_millivolts(voltage_mv);
    sfr->set_power_button(power_button);
    sfr->set_ps_button(ps_button);
    sfr->set_volume_up(volume_up);
    sfr->set_volume_down(volume_down);
}

void PowerState::reset() {
    milliseconds = 0;
    rtc_seconds = 0x4E24E1C0;  // 2011-07-29, a plausible manufacturing date
    battery_percent = 87;
    charger_state = 0;
    voltage_mv = 3820;
    current_ma = -240;  // on battery
    design_capacity_mah = kDesignCapacityMah;
    temperature_dk = kBaseTemperatureDeciKelvin;
    power_button = false;
    ps_button = false;
    volume_up = false;
    volume_down = false;
    soc_released = false;
    soc_release_count = 0;
    power_button_events = 0;
    power_button_held_ms = 0;
    shutdown_requested = false;
    apply();
}

void PowerState::advance_milliseconds(u64 delta) {
    milliseconds += delta;
    rtc_seconds += delta / 1000ull;
    if (power_button) power_button_held_ms += delta;
    // A modelled charger tops the pack up slowly; this keeps the reported
    // percent monotone while a long run is in progress.
    if (charger_state == 1 && battery_percent < 100 && milliseconds % 60000ull < delta) {
        ++battery_percent;
        voltage_mv = static_cast<u32>(std::min<u64>(4300, static_cast<u64>(voltage_mv) + 6));
    } else if (charger_state == 2) {
        battery_percent = 100;
        voltage_mv = 4200;
    }
    if (sfr != nullptr) apply();
}

void PowerState::set_power_button(bool pressed) {
    if (pressed && !power_button) {
        ++power_button_events;
        power_button_held_ms = 0;
    }
    power_button = pressed;
    if (sfr != nullptr) sfr->set_power_button(pressed);
}

void PowerState::set_ps_button(bool pressed) {
    ps_button = pressed;
    if (sfr != nullptr) sfr->set_ps_button(pressed);
}

void PowerState::set_volume_up(bool pressed) {
    volume_up = pressed;
    if (sfr != nullptr) sfr->set_volume_up(pressed);
}

void PowerState::set_volume_down(bool pressed) {
    volume_down = pressed;
    if (sfr != nullptr) sfr->set_volume_down(pressed);
}

void PowerState::set_battery_percent(int percent) {
    battery_percent = std::max(0, std::min(100, percent));
    // Loose Li-ion curve: 3.0 V empty, 4.2 V full.
    voltage_mv = static_cast<u32>(3000 + (1200 * battery_percent) / 100);
    if (sfr != nullptr) apply();
}

void PowerState::set_charger_state(int state) {
    charger_state = state;
    if (state == 1) current_ma = 850;
    else if (state == 2) current_ma = 0;
    else current_ma = -240;
    if (sfr != nullptr) apply();
}

void PowerState::release_soc() {
    if (soc_released) return;
    soc_released = true;
    ++soc_release_count;
}

bool run_power_state_machine(PowerState& state, const PowerPolicy& policy) {
    bool changed = false;
    if (!state.soc_released) {
        // Power-on: the syscon releases the SoC as soon as the reset sequencing
        // is done.  The machine layer's boot chain watches this flag, and the
        // CMeP first loader programs 0x30000208 itself on its own path.
        state.release_soc();
        changed = true;
    }
    if (policy.power_off_on_hold && state.power_button &&
        state.power_button_held_ms >= policy.power_off_hold_ms && !state.shutdown_requested) {
        state.shutdown_requested = true;
        changed = true;
    }
    if (policy.reset_on_ps_hold && state.ps_button && !state.soc_released) {
        state.release_soc();
        changed = true;
    }
    return changed;
}

// ---------------------------------------------------------------------------
// Fuel gauge (bq27520)
// ---------------------------------------------------------------------------

std::string gauge_summary(const PowerState& state) {
    return format("bq27520: %d%% %umV %dmA charger=%s", state.battery_percent, state.voltage_mv,
                  state.current_ma, charger_name(state.charger_state));
}

std::string power_summary(const PowerState& state) {
    return format("rtc=%llu ms=%llu %s soc=%s btn(p=%d ps=%d v+=%d v-=%d)",
                  static_cast<unsigned long long>(state.rtc_seconds),
                  static_cast<unsigned long long>(state.milliseconds), gauge_summary(state).c_str(),
                  state.soc_released ? "released" : "held", state.power_button ? 1 : 0,
                  state.ps_button ? 1 : 0, state.volume_up ? 1 : 0, state.volume_down ? 1 : 0);
}

void describe_power_state(const PowerState& state, std::vector<std::string>& lines) {
    lines.push_back("Ernie power / RTC / fuel gauge");
    lines.push_back(format("  milliseconds : %llu", static_cast<unsigned long long>(state.milliseconds)));
    lines.push_back(format("  RTC          : unix %llu", static_cast<unsigned long long>(state.rtc_seconds)));
    lines.push_back("  " + gauge_summary(state));
    lines.push_back(format("  charger      : %s, current %d mA, design %u mAh",
                           charger_name(state.charger_state), state.current_ma,
                           static_cast<unsigned>(state.design_capacity_mah)));
    lines.push_back(format("  buttons      : power=%d (held %llu ms, %llu events) ps=%d vol+=%d vol-=%d",
                           state.power_button ? 1 : 0,
                           static_cast<unsigned long long>(state.power_button_held_ms),
                           static_cast<unsigned long long>(state.power_button_events),
                           state.ps_button ? 1 : 0, state.volume_up ? 1 : 0,
                           state.volume_down ? 1 : 0));
    lines.push_back(format("  SoC release  : %s (%llu times)", state.soc_released ? "released" : "held",
                           static_cast<unsigned long long>(state.soc_release_count)));
    if (state.shutdown_requested) lines.push_back("  power off    : requested by the power button");
}

const char* charger_name(int state) {
    switch (state) {
        case 0: return "on battery";
        case 1: return "charging";
        case 2: return "charged";
        default: return "unknown";
    }
}

}  // namespace ernie
}  // namespace zlb
