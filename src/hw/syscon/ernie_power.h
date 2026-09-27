// zeliboba - Ernie power / reset / button / fuel gauge / RTC state.
//
// The declarations live in this private header so that ernie.cpp can own the
// state while ernie_power.cpp keeps the bq27520 command model and the power
// state machine.
#pragma once

#include <string>
#include <vector>

#include "common/types.h"

namespace zlb {

class EmmcCard;
class ErnieSfr;

namespace ernie {

class EmmcHost;

/// A single fuel gauge register: 16-bit command address, name and unit.
struct GaugeRegister {
    u8 command = 0;
    const char* name = "";
    const char* unit = "";
};

/// The bq27520 standard command set (Abby/bq27520-g4.pdf section 7.1).
const std::vector<GaugeRegister>& gauge_registers();
const GaugeRegister* gauge_register(u8 command);

/// Whatever the board state is: this is the syscon's model of the console, not
/// of the SoC.
struct PowerState {
    PowerState();

    /// Bind the SFR file and the storage host; also pushes the current state
    /// into the SFR block (ports, RTC registers, panel lines).
    void bind(ErnieSfr* sfr_block, EmmcHost* host);

    /// Push the current state into the SFR file.
    void apply();

    /// Power-on state.
    void reset();

    void advance_milliseconds(u64 delta);

    void set_power_button(bool pressed);
    void set_ps_button(bool pressed);
    void set_volume_up(bool pressed);
    void set_volume_down(bool pressed);
    void set_battery_percent(int percent);
    void set_charger_state(int state);
    void release_soc();

    // --- state ---------------------------------------------------------
    u64 milliseconds = 0;
    u64 rtc_seconds = 0x4E24E1C0;  // 2011-07-29, a plausible manufacturing date
    int battery_percent = 87;
    int charger_state = 0;
    u32 voltage_mv = 3820;
    int current_ma = -240;  ///< negative = discharging (bq27520 AverageCurrent)
    u16 design_capacity_mah = 2500;
    u16 temperature_dk = 2981;  ///< 25.0 C in 0.1 K

    bool power_button = false;
    bool ps_button = false;
    bool volume_up = false;
    bool volume_down = false;

    bool soc_released = false;
    u64 soc_release_count = 0;

    u64 power_button_events = 0;
    u64 power_button_held_ms = 0;
    bool shutdown_requested = false;

    ErnieSfr* sfr = nullptr;
    EmmcHost* emmc = nullptr;
};

/// Knobs for the (small) power state machine the block runs on `tick`.
struct PowerPolicy {
    /// Milliseconds the power button must be held before a power-off is posted.
    u64 power_off_hold_ms = 3000;
    bool power_off_on_hold = true;
    bool reset_on_ps_hold = false;
};

/// Advance the state machine.  Returns true when anything changed.
bool run_power_state_machine(PowerState& state, const PowerPolicy& policy);

std::string gauge_summary(const PowerState& state);
std::string power_summary(const PowerState& state);
void describe_power_state(const PowerState& state, std::vector<std::string>& lines);
const char* charger_name(int state);

}  // namespace ernie
}  // namespace zlb
