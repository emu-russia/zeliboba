// zeliboba - Ernie SFR register file (RL78/G1x special function registers).
//
// Ernie is a Renesas RL78 (G1x class: the SFR map matches RL78/G13
// R01UH0146EJ0380 table 3-5, and RL78/G23/G24 use the same addresses for every
// register the firmware touches).  The RL78 core in src/cpu/rl78/ maps every
// "sfr" operand and every `ES:!addr16` access through the Bus, so this device is
// the whole peripheral side of the microcontroller: ports, timers, serial,
// A/D, RTC, clock generator, watchdog, interrupt flags and the SC command port.
//
// Evidence for the map (the disassembly citations below are offsets into
// ernie-master/USS-1001.bin, which loads at 0x00000):
//
//   * the boot ROM at 0x30035 (USS-1001) programs the clock generator and then
//     polls OSTC and CKC:
//         3003B: mov 0xFFFA0, #0x72      ; CMC  = 0x72 (OSCSEL=1, AMPH=1)
//         3003E: mov 0xFFFA1, #0xC0      ; CSC  = 0xC0 (MSTOP=1, XTSTOP=1)
//         30051: oneb !0xFFFA3           ; OSTS = 0x01
//         30054: clr1 0xFFFA1.7          ; CSC.MSTOP = 0 -> start X1 oscillation
//         3005A: cmp  !0xFFFA2, #0xC0    ; OSTC oscillation-stabilisation status
//         3005E: bnz  $0x3005A           ; 0xC0 == MOST9|MOST10 stable
//         30063: mov 0xFFFA4, #0x10      ; CKC.MCM0 = 1 -> main system clock = fMX
//         30066: bt   0xFFFA4.5, ...     ; CKC.MCS  = fMX selected
//   * 0xFFFE0..0xFFFE7 are the interrupt request / mask registers: 0x30080
//     clears IF0H.TMIF01H (0xFFFE1.6), 0x30083 sets MK1H.TMIF01H (0xFFFE5.6)
//     and 0x3008A spins on 0xFFFE1.6 waiting for the interval timer to fire.
//   * the RTC block at 0xFFF90 is what the SC RTC commands read/write (the
//     firmware keeps a BCD copy of SEC/MIN/HOUR/WEEK/DAY/MONTH/YEAR).
#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include "bus/device.h"
#include "common/types.h"

namespace zlb {
namespace ernie {

// ---------------------------------------------------------------------------
// RL78 address windows (the block itself is declared in hw/syscon.h).
// ---------------------------------------------------------------------------

constexpr u32 kSfrWindowBase = 0x000FFF00;
constexpr u32 kSfrWindowSize = 0x100;
constexpr u32 kSfrWindowMirror = 0x0FFFFF00;
constexpr u32 kFlashWindowBase = 0x00000000;
constexpr u32 kFlashWindowSize = 0x100000;


// ---------------------------------------------------------------------------
// SFR addresses (RL78/G13 table 3-5; the numbers are the low 16 bits, the RL78
// data page is 0xF0000 so `mov 0xFFFA0, #0x72` is a byte store to 0x000FFFA0).
// ---------------------------------------------------------------------------

// Ports
constexpr u32 kSfrP0 = 0xFFF00;
constexpr u32 kSfrP1 = 0xFFF01;
constexpr u32 kSfrP2 = 0xFFF02;
constexpr u32 kSfrP3 = 0xFFF03;
constexpr u32 kSfrP4 = 0xFFF04;
constexpr u32 kSfrP5 = 0xFFF05;
constexpr u32 kSfrP6 = 0xFFF06;
constexpr u32 kSfrP7 = 0xFFF07;
constexpr u32 kSfrP12 = 0xFFF0C;
constexpr u32 kSfrP13 = 0xFFF0D;
constexpr u32 kSfrP14 = 0xFFF0E;
constexpr u32 kSfrP15 = 0xFFF0F;
constexpr u32 kSfrPm0 = 0xFFF20;
constexpr u32 kSfrPm1 = 0xFFF21;
constexpr u32 kSfrPm2 = 0xFFF22;
constexpr u32 kSfrPm3 = 0xFFF23;
constexpr u32 kSfrPm4 = 0xFFF24;
constexpr u32 kSfrPm5 = 0xFFF25;
constexpr u32 kSfrPm6 = 0xFFF26;
constexpr u32 kSfrPm7 = 0xFFF27;
constexpr u32 kSfrPm12 = 0xFFF2C;
constexpr u32 kSfrPm14 = 0xFFF2E;
constexpr u32 kSfrPm15 = 0xFFF2F;

// Serial array unit 0/1/2/3 data registers
constexpr u32 kSfrSdr00 = 0xFFF10;
constexpr u32 kSfrSdr01 = 0xFFF12;
constexpr u32 kSfrSdr02 = 0xFFF44;
constexpr u32 kSfrSdr03 = 0xFFF46;
constexpr u32 kSfrSdr10 = 0xFFF48;
constexpr u32 kSfrSdr11 = 0xFFF4A;
constexpr u32 kSfrSdr12 = 0xFFF14;
constexpr u32 kSfrSdr13 = 0xFFF16;

// Timers
constexpr u32 kSfrTdr00 = 0xFFF18;
constexpr u32 kSfrTdr01 = 0xFFF1A;
constexpr u32 kSfrTdr02 = 0xFFF64;
constexpr u32 kSfrTdr03 = 0xFFF66;
constexpr u32 kSfrTdr04 = 0xFFF68;
constexpr u32 kSfrTdr05 = 0xFFF6A;
constexpr u32 kSfrTdr06 = 0xFFF6C;
constexpr u32 kSfrTdr07 = 0xFFF6E;

// A/D converter
constexpr u32 kSfrAdcr = 0xFFF1E;   ///< 10-bit conversion result
constexpr u32 kSfrAdcrh = 0xFFF1F;  ///< 8-bit conversion result
constexpr u32 kSfrAdm0 = 0xFFF30;
constexpr u32 kSfrAds = 0xFFF31;
constexpr u32 kSfrAdm1 = 0xFFF32;

// Key return / external interrupts
constexpr u32 kSfrKrm = 0xFFF37;
constexpr u32 kSfrEgp0 = 0xFFF38;
constexpr u32 kSfrEgn0 = 0xFFF39;
constexpr u32 kSfrEgp1 = 0xFFF3A;
constexpr u32 kSfrEgn1 = 0xFFF3B;

// I2C (the bq27520 fuel gauge sits on IICA0)
constexpr u32 kSfrIica0 = 0xFFF50;
constexpr u32 kSfrIics0 = 0xFFF51;
constexpr u32 kSfrIicf0 = 0xFFF52;
constexpr u32 kSfrIica1 = 0xFFF54;
constexpr u32 kSfrIics1 = 0xFFF55;
constexpr u32 kSfrIicf1 = 0xFFF56;

// Real time clock
constexpr u32 kSfrItmc = 0xFFF90;
constexpr u32 kSfrSec = 0xFFF92;
constexpr u32 kSfrMin = 0xFFF93;
constexpr u32 kSfrHour = 0xFFF94;
constexpr u32 kSfrWeek = 0xFFF95;
constexpr u32 kSfrDay = 0xFFF96;
constexpr u32 kSfrMonth = 0xFFF97;
constexpr u32 kSfrYear = 0xFFF98;
constexpr u32 kSfrSubcud = 0xFFF99;
constexpr u32 kSfrAlarmwm = 0xFFF9A;
constexpr u32 kSfrAlarmwh = 0xFFF9B;
constexpr u32 kSfrAlarmww = 0xFFF9C;
constexpr u32 kSfrRtcc0 = 0xFFF9D;
constexpr u32 kSfrRtcc1 = 0xFFF9E;

// Clock generator
constexpr u32 kSfrCmc = 0xFFFA0;
constexpr u32 kSfrCsc = 0xFFFA1;
constexpr u32 kSfrOstc = 0xFFFA2;
constexpr u32 kSfrOsts = 0xFFFA3;
constexpr u32 kSfrCkc = 0xFFFA4;
constexpr u32 kSfrCks0 = 0xFFFA5;
constexpr u32 kSfrCks1 = 0xFFFA6;

// Reset / voltage / watchdog / CRC
constexpr u32 kSfrResf = 0xFFFA8;
constexpr u32 kSfrLvim = 0xFFFA9;
constexpr u32 kSfrLvis = 0xFFFAA;
constexpr u32 kSfrWdte = 0xFFFAB;
constexpr u32 kSfrCrcin = 0xFFFAC;

// DMA
constexpr u32 kSfrDsa0 = 0xFFFB0;
constexpr u32 kSfrDsa1 = 0xFFFB1;
constexpr u32 kSfrDra0 = 0xFFFB2;
constexpr u32 kSfrDra1 = 0xFFFB4;
constexpr u32 kSfrDbc0 = 0xFFFB6;
constexpr u32 kSfrDbc1 = 0xFFFB8;
constexpr u32 kSfrDmc0 = 0xFFFBA;
constexpr u32 kSfrDmc1 = 0xFFFBB;
constexpr u32 kSfrDrc0 = 0xFFFBC;
constexpr u32 kSfrDrc1 = 0xFFFBD;

// Interrupt request / mask / priority
constexpr u32 kSfrIf0 = 0xFFFE0;
constexpr u32 kSfrIf1 = 0xFFFE2;
constexpr u32 kSfrMk0 = 0xFFFE4;
constexpr u32 kSfrMk1 = 0xFFFE6;
constexpr u32 kSfrIf2 = 0xFFFD0;
constexpr u32 kSfrMk2 = 0xFFFD4;
constexpr u32 kSfrPr0 = 0xFFFE8;
constexpr u32 kSfrPr1 = 0xFFFEA;
constexpr u32 kSfrPr2 = 0xFFFEC;

// ---------------------------------------------------------------------------
// Bit helpers
// ---------------------------------------------------------------------------

// CMC (0xFFFA0)
constexpr u8 kCmcAmph0 = 0x01;
constexpr u8 kCmcAmph1 = 0x02;
constexpr u8 kCmcOscsels = 0x04;
constexpr u8 kCmcExclks = 0x08;
constexpr u8 kCmcOscsel = 0x40;
constexpr u8 kCmcExclk = 0x80;

// CSC (0xFFFA1)
constexpr u8 kCscHiostop = 0x01;  ///< 0 = high speed on-chip oscillator running
constexpr u8 kCscXtstop = 0x40;   ///< 0 = XT1 (subsystem) oscillator running
constexpr u8 kCscMstop = 0x80;    ///< 0 = X1 (high speed system) oscillator running

// OSTC (0xFFFA2): one bit per 2^n / fX stabilisation window
constexpr u8 kOstcM8 = 0x01;
constexpr u8 kOstcM9 = 0x02;
constexpr u8 kOstcM10 = 0x04;
constexpr u8 kOstcM11 = 0x08;
constexpr u8 kOstcM13 = 0x10;
constexpr u8 kOstcM15 = 0x20;
constexpr u8 kOstcM17 = 0x40;
constexpr u8 kOstcM18 = 0x80;

// CKC (0xFFFA4)
constexpr u8 kCkcMcm0 = 0x10;  ///< W: main system clock = fMX when set
constexpr u8 kCkcMcs = 0x20;   ///< R: main system clock is fMX
constexpr u8 kCkcCss = 0x40;   ///< W: fCLK = subsystem clock when set
constexpr u8 kCkcCls = 0x80;   ///< R: fCLK is the subsystem clock

// Interrupt flag / mask bit names for IF0H / MK0H (0xFFFE1 / 0xFFFE5)
constexpr u8 kIf0hSreif0 = 0x01;
constexpr u8 kIf0hTmif01h = 0x40;
constexpr u8 kIf0hSrIf0 = 0x02;
constexpr u8 kIf0hCsiif01 = 0x04;
constexpr u8 kIf0hIicif01 = 0x08;
constexpr u8 kIf0hStif0 = 0x10;
constexpr u8 kIf0hCsiif00 = 0x20;
constexpr u8 kIf0hIicif00 = 0x40;

/// Which interrupt vector the interval timer (ITMC) raises.  Vector 20 is
/// TMIF01H in the RL78/G13 vector table and the firmware's handler table
/// confirms it (0xE4BA; see tests/test_syscon.cpp).
constexpr int kIntervalTimerVector = 20;

/// How the register behaves on read/write; used by the debugger and by the
/// firmware-facing handshakes.
enum class SfrKind : u8 {
    ReadWrite = 0,
    ReadOnly = 1,
    WriteOnly = 2,
    /// Value the hardware derives from the current state (OSTC, P4, ...).
    Derived = 3,
};

struct SfrDesc {
    u32 address = 0;
    unsigned width = 1;
    const char* name = "";
    u16 reset = 0;
    SfrKind kind = SfrKind::ReadWrite;
    const char* note = "";
};

/// The name table: every address this block models, with its reset value.
const SfrDesc* sfr_table(size_t& count);
const SfrDesc* sfr_lookup(u32 address);
const char* sfr_name(u32 address);

// ---------------------------------------------------------------------------
// Clock generator state (CMC/CSC/OSTC/OSTS/CKC)
// ---------------------------------------------------------------------------

struct ClockState {
    u8 cmc = 0x00;
    u8 csc = 0xC0;  ///< reset: MSTOP = 1, XTSTOP = 1, HIOSTOP = 0
    u8 osts = 0x07;
    u8 ckc = 0x00;

    bool x1_running = false;   ///< CSC.MSTOP == 0
    bool xt1_running = false;  ///< CSC.XTSTOP == 0
    bool hio_running = true;   ///< CSC.HIOSTOP == 0 (on-chip oscillator on after reset)
    /// The boot sequence clears MSTOP (0x30054) to start X1 oscillation; the
    /// stabilisation counter is only reset by the STOP instruction or by
    /// MSTOP = 1, so this flag models the re-arm edge that restarts the count.
    bool mstop_high = true;

    /// Cycle at which X1 oscillation was started (0 = never).
    u64 x1_start_cycle = 0;
    /// Stabilisation window selected by OSTS, in cycles of the CPU clock.
    /// The RL78/G13 counts 2^n / fX; we scale the count to the emulated clock.
    u32 x1_window = 0;

    bool x1_stable = false;
    bool main_is_fmx = false;  ///< CKC.MCS
    bool clk_is_sub = false;   ///< CKC.CLS
};

/// OSTS -> stabilisation window multiplier.  OSTS[2:0] maps to 2^8 .. 2^18 / fX
/// (RL78/G13 5.3.5); we return the exponent.
int osts_exponent(u8 osts);

std::string clock_summary(const ClockState& state);

}  // namespace ernie

/// The SFR window (0x000FFF00..0x000FFFFF) plus its mirror at 0x0FFFFF00.
///
/// It is a Device so the debugger can name every register, and so the RL78 core
/// (which routes `sfr`/`ES:` operands through the Bus) sees a coherent
/// peripheral file.
class ErnieSfr : public Device {
public:
    explicit ErnieSfr(u32 base = ernie::kSfrWindowBase, u32 size = ernie::kSfrWindowSize);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;

    const char* register_name(u32 address) const override;
    void enumerate_registers(std::vector<RegisterInfo>& out) const override;
    bool peek_register(const std::string& name, u64& out) const override;
    bool poke_register(const std::string& name, u64 value) override;
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;
    void tick(u64 cycles) override;

    // ------------------------------------------------------------------
    // Firmware-visible peripheral state
    // ------------------------------------------------------------------

    /// Milliseconds since power-on, as the RTC counter would see them.
    void set_milliseconds(u64 ms) { milliseconds_ = ms; }
    u64 milliseconds() const { return milliseconds_; }

    /// BCD wall clock the RTC registers are derived from.
    void set_rtc_seconds(u64 unix_seconds);
    u64 rtc_seconds() const { return rtc_seconds_; }

    /// Panel lines.  A line reads as 0 when the button is pressed (the syscon
    /// firmware enables the internal pull-up and sees an active low input).
    void set_power_button(bool pressed);
    void set_ps_button(bool pressed);
    void set_volume_up(bool pressed);
    void set_volume_down(bool pressed);
    bool power_button() const { return power_button_; }
    bool ps_button() const { return ps_button_; }
    bool volume_up() const { return volume_up_; }
    bool volume_down() const { return volume_down_; }

    /// Fuel gauge (TI bq27520 over IICA0).
    void set_battery_percent(int percent) { battery_percent_ = percent; }
    int battery_percent() const { return battery_percent_; }
    /// 0 = on battery, 1 = charging, 2 = charged.
    void set_charger_state(int state) { charger_state_ = state; }
    int charger_state() const { return charger_state_; }
    /// Pack voltage in millivolts (bq27520 "Voltage()").
    void set_battery_millivolts(u32 mv) { battery_mv_ = mv; }
    u32 battery_millivolts() const { return battery_mv_; }

    /// Cycle counter the clock/interval-timer model advances with.
    void set_cycle_source(u64 cycles) { cycles_ = cycles; }

    /// Interval timer (KK1 = the deep-sleep wake-up timer in the syscon
    /// firmware) is armed when ITMC is written.
    bool interval_timer_armed() const { return interval_armed_; }
    u64 interval_timer_fires() const { return interval_fires_; }
    /// Period in CPU cycles; the real period is 2^15 / fIL (see the note in the
    /// .cpp) and is not knowable from the dump, so the machine layer overrides
    /// it for bring-up.
    void set_interval_timer_cycles(u64 cycles) { interval_period_ = cycles ? cycles : 1; }
    u64 interval_timer_cycles() const { return interval_period_; }

    /// Interrupt vectors the SFR block wants raised.  The caller (`ErnieBlock`)
    /// forwards them to the RL78 core through `Cpu::set_irq`.
    const std::vector<int>& pending_vectors() const { return pending_vectors_; }
    void clear_pending_vectors() { pending_vectors_.clear(); }

    /// Number of writes to the watchdog enable register seen so far.
    u64 watchdog_writes() const { return watchdog_writes_; }
    /// True when the firmware has refreshed the watchdog with a legal value.
    bool watchdog_fed() const { return watchdog_fed_; }

    /// Named raw access, used by the tests and the debugger.
    u8 peek8(u32 address) const;
    void poke8(u32 address, u8 value);

    const ernie::ClockState& clock() const { return clock_; }
    std::string describe_sfr(u32 address) const;

private:
    u8 read_byte(u32 address);
    void write_byte(u32 address, u8 value);
    /// Pure value computation (no access counters), shared by read_byte and
    /// peek8 so the debugger sees derived registers such as ADCRH.
    u8 compute_byte(u32 address) const;

    void note_access(u32 address, bool write);
    void refresh_clock(u64 cycles);
    void refresh_rtc();
    void update_port_inputs();
    void poke_watchdog(u8 value);
    void arm_interval_timer(u16 value);

    std::map<u32, u8> regs_;
    ernie::ClockState clock_;

    u64 milliseconds_ = 0;
    u64 rtc_seconds_ = 0x4E24E1C0;  // 2011-07-29
    u64 cycles_ = 0;

    bool power_button_ = false;
    bool ps_button_ = false;
    bool volume_up_ = false;
    bool volume_down_ = false;

    int battery_percent_ = 87;
    int charger_state_ = 0;
    u32 battery_mv_ = 3820;

    bool interval_armed_ = false;
    u64 interval_period_ = 2097152;
    u64 interval_next_ = 0;
    u64 interval_fires_ = 0;

    u64 watchdog_writes_ = 0;
    bool watchdog_fed_ = false;

    std::vector<int> pending_vectors_;

    /// Last 16 distinct SFR accesses, for `describe()` and the selftest.
    std::vector<std::pair<u32, u8>> recent_;
    u64 reads_ = 0;
    u64 writes_ = 0;
};

}  // namespace zlb

