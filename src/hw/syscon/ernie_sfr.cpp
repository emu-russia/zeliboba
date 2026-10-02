// zeliboba - Ernie SFR register file implementation.
//
// Layout note: the command table and the clock/RTC helpers live in an anonymous
// namespace at `zlb` scope (so they are file local), while `zlb::ernie` only
// holds declarations.  `ErnieSfr` is declared in hw/syscon/ernie_sfr.h at `zlb`
// scope, so its member definitions must be at that scope too.
#include "hw/syscon/ernie_sfr.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "common/util.h"

namespace zlb {

namespace {

using namespace zlb::ernie;

// The table below is the RL78/G13 SFR list (R01UH0146EJ0380 table 3-5) filtered
// down to the addresses the USS-1001/USS-1002 firmware actually touches.  The
// "note" column records the firmware evidence for the modelled behaviour.
const SfrDesc kTable[] = {
    // ---- ports ----------------------------------------------------------
    {kSfrP0, 1, "P0", 0x00, SfrKind::Derived, "port 0 output latch"},
    {kSfrP1, 1, "P1", 0x00, SfrKind::Derived, "port 1 output latch"},
    {kSfrP2, 1, "P2", 0x00, SfrKind::Derived, "port 2 output latch"},
    {kSfrP3, 1, "P3", 0x00, SfrKind::Derived, "port 3 output latch"},
    {kSfrP4, 1, "P4", 0x00, SfrKind::Derived, "port 4 latch; 0x3004B/0x30057 drives P4.0"},
    {kSfrP5, 1, "P5", 0x00, SfrKind::Derived, "port 5 output latch"},
    {kSfrP6, 1, "P6", 0x00, SfrKind::Derived, "port 6 latch (charger/USB sense)"},
    {kSfrP7, 1, "P7", 0x00, SfrKind::Derived, "port 7 output latch"},
    {kSfrP12, 1, "P12", 0x00, SfrKind::Derived, "port 12 output latch"},
    {kSfrP13, 1, "P13", 0x00, SfrKind::Derived, "port 13 output latch"},
    {kSfrP14, 1, "P14", 0x00, SfrKind::ReadWrite, "port 14 output latch"},
    {kSfrP15, 1, "P15", 0x00, SfrKind::ReadWrite, "port 15 output latch"},

    {kSfrPm0, 1, "PM0", 0xFF, SfrKind::ReadWrite, "port mode (1 = input)"},
    {kSfrPm1, 1, "PM1", 0xFF, SfrKind::ReadWrite, "port mode"},
    {kSfrPm2, 1, "PM2", 0xFF, SfrKind::ReadWrite, "port mode"},
    {kSfrPm3, 1, "PM3", 0xFF, SfrKind::ReadWrite, "port mode"},
    {kSfrPm4, 1, "PM4", 0xFF, SfrKind::ReadWrite, "port mode"},
    {kSfrPm5, 1, "PM5", 0xFF, SfrKind::ReadWrite, "port mode"},
    {kSfrPm6, 1, "PM6", 0xFF, SfrKind::ReadWrite, "port mode"},
    {kSfrPm7, 1, "PM7", 0xFF, SfrKind::ReadWrite, "port mode"},
    {kSfrPm12, 1, "PM12", 0xFF, SfrKind::ReadWrite, "port mode"},
    {kSfrPm14, 1, "PM14", 0xFF, SfrKind::ReadWrite, "port mode"},
    {kSfrPm15, 1, "PM15", 0xFF, SfrKind::ReadWrite, "port mode"},

    // ---- serial (SAU) ---------------------------------------------------
    {kSfrSdr00, 2, "SDR00", 0x0000, SfrKind::ReadWrite, "SCI0 TXD/SIO00 data"},
    {kSfrSdr01, 2, "SDR01", 0x0000, SfrKind::ReadWrite, "SCI0 RXD/SIO01 data"},
    {kSfrSdr02, 2, "SDR02", 0x0000, SfrKind::ReadWrite, "SCI1 TXD/SIO10 data"},
    {kSfrSdr03, 2, "SDR03", 0x0000, SfrKind::ReadWrite, "SCI1 RXD/SIO11 data"},
    {kSfrSdr10, 2, "SDR10", 0x0000, SfrKind::ReadWrite, "SCI2 data"},
    {kSfrSdr11, 2, "SDR11", 0x0000, SfrKind::ReadWrite, "SCI2 data"},
    {kSfrSdr12, 2, "SDR12", 0x0000, SfrKind::ReadWrite, "SCI3 TXD/SIO30 data"},
    {kSfrSdr13, 2, "SDR13", 0x0000, SfrKind::ReadWrite, "SCI3 RXD/SIO31 data"},

    // ---- timers ---------------------------------------------------------
    {kSfrTdr00, 2, "TDR00", 0x0000, SfrKind::ReadWrite, "timer 00 counter/compare"},
    {kSfrTdr01, 2, "TDR01", 0x0000, SfrKind::ReadWrite, "timer 01 counter/compare"},
    {kSfrTdr02, 2, "TDR02", 0x0000, SfrKind::ReadWrite, "timer 02"},
    {kSfrTdr03, 2, "TDR03", 0x0000, SfrKind::ReadWrite, "timer 03"},
    {kSfrTdr04, 2, "TDR04", 0x0000, SfrKind::ReadWrite, "timer 04"},
    {kSfrTdr05, 2, "TDR05", 0x0000, SfrKind::ReadWrite, "timer 05"},
    {kSfrTdr06, 2, "TDR06", 0x0000, SfrKind::ReadWrite, "timer 06"},
    {kSfrTdr07, 2, "TDR07", 0x0000, SfrKind::ReadWrite, "timer 07"},

    // ---- A/D ------------------------------------------------------------
    {kSfrAdcr, 2, "ADCR", 0x0000, SfrKind::Derived, "10-bit A/D result (battery divider)"},
    {kSfrAdcrh, 1, "ADCRH", 0x00, SfrKind::Derived, "8-bit A/D result"},
    {kSfrAdm0, 1, "ADM0", 0x00, SfrKind::ReadWrite, "A/D mode 0 (ADCS = conversion start)"},
    {kSfrAds, 1, "ADS", 0x00, SfrKind::ReadWrite, "A/D channel select"},
    {kSfrAdm1, 1, "ADM1", 0x00, SfrKind::ReadWrite, "A/D mode 1"},
    {0xFFF35, 1, "ADUL", 0x00, SfrKind::ReadWrite, "A/D upper limit"},
    {0xFFF36, 1, "ADLL", 0x00, SfrKind::ReadWrite, "A/D lower limit"},

    // ---- key return / external interrupt edges --------------------------
    {kSfrKrm, 1, "KRM", 0x00, SfrKind::ReadWrite, "key return mode (PS/home button)"},
    {kSfrEgp0, 1, "EGP0", 0x00, SfrKind::ReadWrite, "external interrupt rising edge 0"},
    {kSfrEgn0, 1, "EGN0", 0x00, SfrKind::ReadWrite, "external interrupt falling edge 0"},
    {kSfrEgp1, 1, "EGP1", 0x00, SfrKind::ReadWrite, "external interrupt rising edge 1"},
    {kSfrEgn1, 1, "EGN1", 0x00, SfrKind::ReadWrite, "external interrupt falling edge 1"},

    // ---- I2C (fuel gauge) ----------------------------------------------
    {kSfrIica0, 1, "IICA0", 0x00, SfrKind::ReadWrite, "IICA0 shift register (bq27520)"},
    {kSfrIics0, 1, "IICS0", 0x00, SfrKind::Derived, "IICA0 status"},
    {kSfrIicf0, 1, "IICF0", 0x00, SfrKind::ReadWrite, "IICA0 flag register"},
    {kSfrIica1, 1, "IICA1", 0x00, SfrKind::ReadWrite, "IICA1 shift register"},
    {kSfrIics1, 1, "IICS1", 0x00, SfrKind::Derived, "IICA1 status"},
    {kSfrIicf1, 1, "IICF1", 0x00, SfrKind::ReadWrite, "IICA1 flag register"},

    // ---- RTC ------------------------------------------------------------
    {kSfrItmc, 2, "ITMC", 0x00FF, SfrKind::Derived, "interval timer control (write arms it)"},
    {kSfrSec, 1, "SEC", 0x00, SfrKind::Derived, "RTC second (BCD)"},
    {kSfrMin, 1, "MIN", 0x00, SfrKind::Derived, "RTC minute (BCD)"},
    {kSfrHour, 1, "HOUR", 0x12, SfrKind::Derived, "RTC hour (BCD)"},
    {kSfrWeek, 1, "WEEK", 0x00, SfrKind::Derived, "RTC day of week"},
    {kSfrDay, 1, "DAY", 0x01, SfrKind::Derived, "RTC day (BCD)"},
    {kSfrMonth, 1, "MONTH", 0x01, SfrKind::Derived, "RTC month (BCD)"},
    {kSfrYear, 1, "YEAR", 0x00, SfrKind::Derived, "RTC year (BCD, 00 = 2000)"},
    {kSfrSubcud, 1, "SUBCUD", 0x00, SfrKind::ReadWrite, "watch error correction"},
    {kSfrAlarmwm, 1, "ALARMWM", 0x00, SfrKind::ReadWrite, "alarm minute"},
    {kSfrAlarmwh, 1, "ALARMWH", 0x12, SfrKind::ReadWrite, "alarm hour"},
    {kSfrAlarmww, 1, "ALARMWW", 0x00, SfrKind::ReadWrite, "alarm week"},
    {kSfrRtcc0, 1, "RTCC0", 0x00, SfrKind::ReadWrite, "RTC control 0"},
    {kSfrRtcc1, 1, "RTCC1", 0x00, SfrKind::ReadWrite, "RTC control 1 (RWSTP/RWAIT)"},

    // ---- clock generator ------------------------------------------------
    {kSfrCmc, 1, "CMC", 0x00, SfrKind::ReadWrite, "clock mode control; 0x3003B writes 0x72"},
    {kSfrCsc, 1, "CSC", 0xC0, SfrKind::ReadWrite, "clock status control; bit7 MSTOP starts X1"},
    {kSfrOstc, 1, "OSTC", 0x00, SfrKind::Derived, "oscillation stabilisation status; polled at 0x3005A"},
    {kSfrOsts, 1, "OSTS", 0x07, SfrKind::ReadWrite, "oscillation stabilisation time select"},
    {kSfrCkc, 1, "CKC", 0x00, SfrKind::Derived, "clock control; bit4 MCM0 = fMX, bit5 MCS reports"},
    {kSfrCks0, 1, "CKS0", 0x00, SfrKind::ReadWrite, "clock output select 0"},
    {kSfrCks1, 1, "CKS1", 0x00, SfrKind::ReadWrite, "clock output select 1"},

    // ---- reset / voltage / watchdog -------------------------------------
    {kSfrResf, 1, "RESF", 0x00, SfrKind::Derived, "reset control flag"},
    {kSfrLvim, 1, "LVIM", 0x00, SfrKind::ReadWrite, "voltage detection register"},
    {kSfrLvis, 1, "LVIS", 0x00, SfrKind::ReadWrite, "voltage detection level"},
    {kSfrWdte, 1, "WDTE", 0x1A, SfrKind::WriteOnly, "watchdog enable (0xAC feeds it)"},
    {kSfrCrcin, 1, "CRCIN", 0x00, SfrKind::ReadWrite, "CRC input"},

    // ---- DMA ------------------------------------------------------------
    {kSfrDsa0, 1, "DSA0", 0x00, SfrKind::ReadWrite, "DMA SFR address 0"},
    {kSfrDsa1, 1, "DSA1", 0x00, SfrKind::ReadWrite, "DMA SFR address 1"},
    {kSfrDra0, 2, "DRA0", 0x0000, SfrKind::ReadWrite, "DMA RAM address 0"},
    {kSfrDra1, 2, "DRA1", 0x0000, SfrKind::ReadWrite, "DMA RAM address 1"},
    {kSfrDbc0, 2, "DBC0", 0x0000, SfrKind::ReadWrite, "DMA byte count 0"},
    {kSfrDbc1, 2, "DBC1", 0x0000, SfrKind::ReadWrite, "DMA byte count 1"},
    {kSfrDmc0, 1, "DMC0", 0x00, SfrKind::ReadWrite, "DMA mode 0"},
    {kSfrDmc1, 1, "DMC1", 0x00, SfrKind::ReadWrite, "DMA mode 1"},
    {kSfrDrc0, 1, "DRC0", 0x00, SfrKind::ReadWrite, "DMA operation control 0"},
    {kSfrDrc1, 1, "DRC1", 0x00, SfrKind::ReadWrite, "DMA operation control 1"},

    // ---- interrupts -----------------------------------------------------
    {kSfrIf0, 1, "IF0L", 0x00, SfrKind::ReadWrite, "interrupt request flags"},
    {kSfrIf0 + 1, 1, "IF0H", 0x00, SfrKind::Derived, "bit6 TMIF01H is the boot's interval flag"},
    {kSfrIf1, 1, "IF1L", 0x00, SfrKind::ReadWrite, "interrupt request flags"},
    {kSfrIf1 + 1, 1, "IF1H", 0x00, SfrKind::Derived, "bit7 RTCIF / bit6 ITIF / bit0 ADIF"},
    {kSfrMk0, 1, "MK0L", 0xFF, SfrKind::ReadWrite, "interrupt mask flags"},
    {kSfrMk0 + 1, 1, "MK0H", 0xFF, SfrKind::ReadWrite, "interrupt mask (0 enables)"},
    {kSfrMk1, 1, "MK1L", 0xFF, SfrKind::ReadWrite, "interrupt mask"},
    {kSfrMk1 + 1, 1, "MK1H", 0xFF, SfrKind::ReadWrite, "bit6 masks TMIF01H"},
    {kSfrPr0, 2, "PR0", 0xFFFF, SfrKind::ReadWrite, "interrupt priority 0"},
    {kSfrPr1, 2, "PR1", 0xFFFF, SfrKind::ReadWrite, "interrupt priority 1"},
    {kSfrIf2, 2, "IF2", 0x0000, SfrKind::ReadWrite, "interrupt request flags 2"},
    {kSfrMk2, 2, "MK2", 0xFFFF, SfrKind::ReadWrite, "interrupt mask 2"},
    {kSfrPr2, 2, "PR2", 0xFFFF, SfrKind::ReadWrite, "interrupt priority 2"},

    // ---- multiplier / divider -------------------------------------------
    {0xFFFF0, 1, "MD0", 0x00, SfrKind::ReadWrite, "multiplier/divider operand A"},
    {0xFFFF1, 1, "MD1", 0x00, SfrKind::ReadWrite, "multiplier/divider operand B"},
    {0xFFFF2, 1, "MD2", 0x00, SfrKind::ReadWrite, "multiplier/divider operand C"},
    {0xFFFF3, 1, "MD3", 0x00, SfrKind::ReadWrite, "multiplier/divider operand D"},
    {0xFFFF4, 1, "MD4", 0x00, SfrKind::ReadWrite, "multiplier/divider operand E"},
    {0xFFFF5, 1, "MD5", 0x00, SfrKind::ReadWrite, "multiplier/divider result 0"},
    {0xFFFF6, 1, "MD6", 0x00, SfrKind::ReadWrite, "multiplier/divider result 1"},
    {0xFFFF7, 1, "MD7", 0x00, SfrKind::ReadWrite, "multiplier/divider result 2"},
};

const SfrDesc* const kTableEnd = kTable + (sizeof(kTable) / sizeof(kTable[0]));

// ---------------------------------------------------------------------------
// RTC calendar conversion
// ---------------------------------------------------------------------------

/// Converted RL78 RTC registers (years count from 2000, all fields BCD).
struct BcdTime {
    u8 sec = 0, min = 0, hour = 0, week = 0, day = 1, month = 1, year = 0;
};

int days_in_month(int year, int month) {
    static const int table[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month < 1 || month > 12) return 30;
    if (month == 2) {
        const bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
        return leap ? 29 : 28;
    }
    return table[month - 1];
}

u8 to_bcd(int value) {
    const int clamped = value < 0 ? 0 : (value > 99 ? 99 : value);
    return static_cast<u8>(((clamped / 10) << 4) | (clamped % 10));
}

int from_bcd(u8 value) { return ((value >> 4) & 0x0F) * 10 + (value & 0x0F); }

BcdTime from_unix(u64 seconds) {
    BcdTime out;
    const u64 days_total = seconds / 86400ull;
    const u64 rem = seconds % 86400ull;
    const int hour = static_cast<int>(rem / 3600);
    const int minute = static_cast<int>((rem % 3600) / 60);
    const int second = static_cast<int>(rem % 60);

    // 1970-01-01 was a Thursday; the RL78 WEEK register counts 0 = Sunday.
    const int weekday = static_cast<int>((days_total + 4) % 7);

    // Civil-from-days (Howard Hinnant), then back to a calendar year.
    const s64 z = static_cast<s64>(days_total) + 719468;
    const s64 era = (z >= 0 ? z : z - 146096) / 146097;
    const u64 doe = static_cast<u64>(z - era * 146097);
    const u64 yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    s64 year = static_cast<s64>(yoe) + era * 400;
    const u64 doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const u64 mp = (5 * doy + 2) / 153;
    const int day = static_cast<int>(doy - (153 * mp + 2) / 5 + 1);
    const int month = static_cast<int>(mp < 10 ? mp + 3 : mp - 9);
    year += (month <= 2) ? 1 : 0;

    const int yy = static_cast<int>((year - 2000) % 100);
    out.sec = to_bcd(second);
    out.min = to_bcd(minute);
    out.hour = to_bcd(hour);
    out.day = to_bcd(day);
    out.month = to_bcd(month);
    out.year = to_bcd(yy);
    out.week = static_cast<u8>(weekday & 0x07);
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Table accessors (declared in zlb::ernie)
// ---------------------------------------------------------------------------

namespace ernie {

const SfrDesc* sfr_table(size_t& count) {
    count = static_cast<size_t>(kTableEnd - kTable);
    return kTable;
}

const SfrDesc* sfr_lookup(u32 address) {
    for (const SfrDesc* it = kTable; it != kTableEnd; ++it) {
        if (address >= it->address && address < it->address + it->width) return it;
    }
    return nullptr;
}

const char* sfr_name(u32 address) {
    const SfrDesc* desc = sfr_lookup(address);
    return desc ? desc->name : nullptr;
}

int osts_exponent(u8 osts) {
    // RL78/G13 figure 5-6: OSTS[2:0] selects 2^8 .. 2^18 / fX.
    static const int table[8] = {8, 9, 10, 11, 13, 15, 17, 18};
    return table[osts & 0x07];
}

std::string clock_summary(const ClockState& state) {
    return format("CMC=%02X CSC=%02X OSTS=%02X CKC=%02X fCLK=%s%s", state.cmc, state.csc, state.osts,
                  state.ckc, state.clk_is_sub ? "fSUB/" : "fMAIN/",
                  state.main_is_fmx ? "fMX" : "fIH");
}

void ClockState::save_state(StateWriter& writer) const {
    writer.put_u8(cmc);
    writer.put_u8(csc);
    writer.put_u8(osts);
    writer.put_u8(ckc);
    writer.put_bool(x1_running);
    writer.put_bool(xt1_running);
    writer.put_bool(hio_running);
    writer.put_bool(mstop_high);
    writer.put_u64(x1_start_cycle);
    writer.put_u32(x1_window);
    writer.put_bool(x1_stable);
    writer.put_bool(main_is_fmx);
    writer.put_bool(clk_is_sub);
}

void ClockState::load_state(StateReader& reader) {
    cmc = reader.get_u8();
    csc = reader.get_u8();
    osts = reader.get_u8();
    ckc = reader.get_u8();
    x1_running = reader.get_bool();
    xt1_running = reader.get_bool();
    hio_running = reader.get_bool();
    mstop_high = reader.get_bool();
    x1_start_cycle = reader.get_u64();
    x1_window = reader.get_u32();
    x1_stable = reader.get_bool();
    main_is_fmx = reader.get_bool();
    clk_is_sub = reader.get_bool();
}

}  // namespace ernie

// ---------------------------------------------------------------------------
// ErnieSfr
// ---------------------------------------------------------------------------

ErnieSfr::ErnieSfr(u32 base, u32 size) : Device("Ernie.SFR", base, size) {
    for (const ernie::SfrDesc* it = kTable; it != kTableEnd; ++it) {
        register_name_entry(it->address, it->name);
        regs_[it->address] = static_cast<u8>(it->reset & 0xFF);
    }
    refresh_clock(0);
    refresh_rtc();
    update_port_inputs();
}

void ErnieSfr::reset() {
    for (const ernie::SfrDesc* it = kTable; it != kTableEnd; ++it) {
        regs_[it->address] = static_cast<u8>(it->reset & 0xFF);
    }
    clock_ = ernie::ClockState{};
    milliseconds_ = 0;
    cycles_ = 0;
    interval_armed_ = false;
    interval_next_ = 0;
    interval_fires_ = 0;
    watchdog_writes_ = 0;
    watchdog_fed_ = false;
    pending_vectors_.clear();
    recent_.clear();
    reads_ = 0;
    writes_ = 0;
    refresh_rtc();
    update_port_inputs();
    refresh_clock(0);
}

void ErnieSfr::save_state(StateWriter& writer) const {
    // The register image (all SFRs, including the derived ones' latched values),
    // then everything the model keeps outside it.
    writer.map(regs_, [&](u32 address, u8 value) {
        writer.put_u32(address);
        writer.put_u8(value);
    });
    writer.begin("clock");
    clock_.save_state(writer);
    writer.end();
    writer.put_u64(milliseconds_);
    writer.put_u64(rtc_seconds_);
    writer.put_u64(cycles_);
    writer.put_bool(power_button_);
    writer.put_bool(ps_button_);
    writer.put_bool(volume_up_);
    writer.put_bool(volume_down_);
    writer.put_i32(battery_percent_);
    writer.put_i32(charger_state_);
    writer.put_u32(battery_mv_);
    writer.put_bool(interval_armed_);
    writer.put_u64(interval_period_);
    writer.put_u64(interval_next_);
    writer.put_u64(interval_fires_);
    writer.put_u64(watchdog_writes_);
    writer.put_bool(watchdog_fed_);
    writer.list(pending_vectors_, [&](int vector) { writer.put_i32(vector); });
    // The recent-access ring is a bounded runtime log, so a length-prefixed list
    // is the right shape for it.
    writer.list(recent_, [&](const std::pair<u32, u8>& entry) {
        writer.put_u32(entry.first);
        writer.put_u8(entry.second);
    });
    writer.put_u64(reads_);
    writer.put_u64(writes_);
}

void ErnieSfr::load_state(StateReader& reader) {
    reader.map(regs_, [&](u32& address, u8& value) {
        address = reader.get_u32();
        value = reader.get_u8();
    });
    reader.begin("clock");
    clock_.load_state(reader);
    reader.end();
    milliseconds_ = reader.get_u64();
    rtc_seconds_ = reader.get_u64();
    cycles_ = reader.get_u64();
    power_button_ = reader.get_bool();
    ps_button_ = reader.get_bool();
    volume_up_ = reader.get_bool();
    volume_down_ = reader.get_bool();
    battery_percent_ = reader.get_i32();
    charger_state_ = reader.get_i32();
    battery_mv_ = reader.get_u32();
    interval_armed_ = reader.get_bool();
    interval_period_ = reader.get_u64();
    interval_next_ = reader.get_u64();
    interval_fires_ = reader.get_u64();
    watchdog_writes_ = reader.get_u64();
    watchdog_fed_ = reader.get_bool();
    reader.list(pending_vectors_, [&](int& vector) { vector = reader.get_i32(); });
    reader.list(recent_, [&](std::pair<u32, u8>& entry) {
        entry.first = reader.get_u32();
        entry.second = reader.get_u8();
    });
    reads_ = reader.get_u64();
    writes_ = reader.get_u64();
}

u8 ErnieSfr::peek8(u32 address) const {
    return compute_byte(address & 0xFFFFFFu);
}

void ErnieSfr::poke8(u32 address, u8 value) { regs_[address & 0xFFFFFFu] = value; }

// ---------------------------------------------------------------------------
// Bus access
// ---------------------------------------------------------------------------

u64 ErnieSfr::read(u32 address, unsigned size) {
    // A 16-bit register read must be served as a unit: reading it byte by byte
    // would lose the high lane of SDR/TDR/DRA/RTC-pair registers.
    if (size == 2) {
        const ernie::SfrDesc* desc = ernie::sfr_lookup(address);
        if (desc != nullptr && desc->width == 2 && address == desc->address) {
            refresh_clock(cycles_);
            const u8 low = read_byte(address);
            const u8 high = read_byte(address + 1);
            return static_cast<u64>(low) | (static_cast<u64>(high) << 8);
        }
    }

    u64 value = 0;
    for (unsigned i = 0; i < size; ++i) {
        value |= static_cast<u64>(read_byte(address + i)) << (8 * i);
    }
    return value;
}

void ErnieSfr::write(u32 address, unsigned size, u64 value) {
    // Same reasoning as read(): a 16-bit store to a 16-bit register must land as
    // one value, otherwise the second byte iteration would clobber the first
    // (that is what made ITMC read back as 0x0000 after `movw ITMC,#0x8000`).
    if (size == 2) {
        const ernie::SfrDesc* desc = ernie::sfr_lookup(address);
        if (desc != nullptr && desc->width == 2 && address == desc->address) {
            note_access(address, true);
            if (address == ernie::kSfrItmc) {
                arm_interval_timer(static_cast<u16>(value));
                return;
            }
            auto store = [this, address](unsigned offset, u8 byte) {
                auto it = regs_.find(address + offset);
                if (it != regs_.end()) it->second = byte;
            };
            store(0, static_cast<u8>(value & 0xFF));
            store(1, static_cast<u8>((value >> 8) & 0xFF));
            return;
        }
    }

    for (unsigned i = 0; i < size; ++i) {
        write_byte(address + i, static_cast<u8>((value >> (8 * i)) & 0xFF));
    }
}

void ErnieSfr::note_access(u32 address, bool write) {
    if (write) {
        ++writes_;
    } else {
        ++reads_;
    }
    recent_.push_back({address, write ? 1 : 0});
    if (recent_.size() > 16) recent_.erase(recent_.begin());
}

void ErnieSfr::update_port_inputs() {
    // The syscon reads the panel lines back through its port registers.  A
    // pressed button shorts the line to ground, so the input bit reads 0.
    auto set_bit = [this](u32 address, u8 mask, bool high) {
        u8& value = regs_[address];
        if (high) {
            value = static_cast<u8>(value | mask);
        } else {
            value = static_cast<u8>(value & ~mask);
        }
    };
    // P1.0 = power button, P1.1 = PS (home) button, P1.2/P1.3 = volume.
    set_bit(ernie::kSfrP1, 0x01, !power_button_);
    set_bit(ernie::kSfrP1, 0x02, !ps_button_);
    set_bit(ernie::kSfrP1, 0x04, !volume_up_);
    set_bit(ernie::kSfrP1, 0x08, !volume_down_);
    // P6.0 = charger present (bq27520 PGOOD), P6.1 = USB VBUS.
    set_bit(ernie::kSfrP6, 0x01, charger_state_ != 0);
    set_bit(ernie::kSfrP6, 0x02, charger_state_ == 1);
}

void ErnieSfr::refresh_rtc() {
    const BcdTime t = from_unix(rtc_seconds_);
    regs_[ernie::kSfrSec] = t.sec;
    regs_[ernie::kSfrMin] = t.min;
    regs_[ernie::kSfrHour] = t.hour;
    regs_[ernie::kSfrWeek] = t.week;
    regs_[ernie::kSfrDay] = t.day;
    regs_[ernie::kSfrMonth] = t.month;
    regs_[ernie::kSfrYear] = t.year;
}

void ErnieSfr::refresh_clock(u64 cycles) {
    // OSTS is a plain register but the clock model owns its live value, so it is
    // pushed back here (a reset leaves the table's 0x07 in place).
    regs_[ernie::kSfrOsts] = clock_.osts;

    // --- X1 oscillation stabilisation ------------------------------------
    // RL78/G13 5.3.4: OSTC is cleared to 00H by a reset, by STOP and by
    // CSTP (MSTOP) = 1; it then counts 2^8 .. 2^18 / fX while the X1 oscillator
    // runs.  The boot ROM starts the oscillator (0x30054 clears MSTOP) and waits
    // for `cmp !0xFFFA2, #0xC0` (0x3005A), i.e. exactly MOST9 | MOST10: the two
    // bits that the Renesas startup example tests for.  Hold the register at
    // 0xC0 from that point on rather than running on into MOST11/MOST13, which
    // is what a real part does while `fX` keeps counting but the startup code
    // has already left the loop.
    if (clock_.mstop_high || !clock_.x1_running) {
        clock_.x1_stable = false;
        clock_.x1_start_cycle = 0;
        clock_.main_is_fmx = false;
        regs_[ernie::kSfrOstc] = 0x00;
    } else {
        if (clock_.x1_start_cycle == 0) clock_.x1_start_cycle = cycles;
        const u64 elapsed = cycles - clock_.x1_start_cycle;
        const u64 window = 1ull << ernie::osts_exponent(clock_.osts);
        u8 ostc;
        if (elapsed >= window + (window >> 1)) {
            ostc = 0xC0;  // MOST9 | MOST10: stabilised, hold here
        } else if (elapsed >= window) {
            ostc = 0x80;  // MOST9 only
        } else {
            ostc = 0x00;
        }
        regs_[ernie::kSfrOstc] = ostc;
        clock_.x1_stable = ostc == 0xC0;
    }
    // --- main system clock selection -------------------------------------
    // CKC.MCM0 = 1 asks for fMX (the X1 oscillator).  MCS reports the clock
    // actually feeding fMAIN: it only follows once the oscillator is stable.
    const bool want_fmx = (clock_.ckc & ernie::kCkcMcm0) != 0;
    clock_.main_is_fmx = want_fmx && clock_.x1_stable;
    clock_.clk_is_sub = (clock_.ckc & ernie::kCkcCss) != 0;

    u8 ckc = clock_.ckc;
    if (clock_.main_is_fmx) {
        ckc = static_cast<u8>(ckc | ernie::kCkcMcs);
    } else {
        ckc = static_cast<u8>(ckc & ~ernie::kCkcMcs);
    }
    if (clock_.clk_is_sub) {
        ckc = static_cast<u8>(ckc | ernie::kCkcCls);
    } else {
        ckc = static_cast<u8>(ckc & ~ernie::kCkcCls);
    }
    regs_[ernie::kSfrCkc] = ckc;
    regs_[ernie::kSfrCsc] = clock_.csc;
}

void ErnieSfr::poke_watchdog(u8 value) {
    ++watchdog_writes_;
    // RL78/G13 15.3: the high nibble of WDTE selects the mode, writing 0xAC
    // clears the counter (the firmware's standard "feed"), 0x1A restarts it.
    const u8 high = static_cast<u8>(value & 0xF0);
    if (high == 0xA0) {
        watchdog_fed_ = true;  // 0xAC: clear and restart the counter
    } else if (value == 0x1A) {
        watchdog_fed_ = false;  // stop
    }
    regs_[ernie::kSfrWdte] = value;
}

void ErnieSfr::arm_interval_timer(u16 value) {
    // ITMC is a 16-bit register: store both lanes so a read-back matches what the
    // firmware wrote (the previous byte-wise path clobbered the high byte).
    regs_[ernie::kSfrItmc] = static_cast<u8>(value & 0xFF);
    regs_[ernie::kSfrItmc + 1] = static_cast<u8>((value >> 8) & 0xFF);
    // RL78/G13 22.2: ITMC holds the 15-bit reload value of the interval timer
    // down-counter.  Writing any non-zero value starts the counter; the firmware
    // writes 0x8000, i.e. bit 14 clear with a large reload, which is a legal
    // (if unusual) reload.  A write of 0 stops the counter.
    const bool armed = (value & 0x7FFF) != 0;
    interval_armed_ = armed;
    interval_fires_ = 1;
    interval_next_ = cycles_ + interval_period_;
}

u8 ErnieSfr::compute_byte(u32 address) const {
    switch (address) {
        // A/D converter: the result registers only hold something meaningful
        // once ADM0.ADCS has been set.  The divider is fed from the battery, so
        // report a plausible 10-bit value derived from the modelled cell
        // voltage; a zero here would look like a dead battery to the firmware.
        case ernie::kSfrAdcrh: {
            auto adm0 = regs_.find(ernie::kSfrAdm0);
            if (adm0 != regs_.end() && (adm0->second & 0x80) != 0) {
                const u32 span = battery_mv_ > 3000 ? (battery_mv_ - 3000) : 0;
                const u32 counts = std::min<u32>(span / 2, 1023);
                return static_cast<u8>(counts >> 2);
            }
            return 0;
        }
        // IICA channel 0: the bq27520 answers with a live status byte.  The value
        // follows the SMBus status layout: transfer complete, no error.
        case ernie::kSfrIics0: {
            u8 status = 0x10;  // SPD0: transfer complete, no error
            if (charger_state_ != 0) status |= 0x08;
            return status;
        }
        case ernie::kSfrIics1: return 0x10;
        case ernie::kSfrOstc: return regs_.count(address) ? regs_.at(address) : 0;
        default: break;
    }
    auto it = regs_.find(address);
    return it == regs_.end() ? 0 : it->second;
}

u8 ErnieSfr::read_byte(u32 address) {
    note_access(address, false);
    refresh_clock(cycles_);
    return compute_byte(address);
}

void ErnieSfr::write_byte(u32 address, u8 value) {
    note_access(address, true);
    switch (address) {
        case ernie::kSfrCmc:
            clock_.cmc = static_cast<u8>(value & 0xC7);  // bits 5..3 read as 0
            refresh_clock(cycles_);
            return;

        case ernie::kSfrCsc:
            clock_.csc = value;
            clock_.mstop_high = (value & ernie::kCscMstop) != 0;
            clock_.x1_running = (value & ernie::kCscMstop) == 0;
            clock_.xt1_running = (value & ernie::kCscXtstop) == 0;
            clock_.hio_running = (value & ernie::kCscHiostop) == 0;
            refresh_clock(cycles_);
            return;

        case ernie::kSfrOsts:
            clock_.osts = static_cast<u8>(value & 0x07);
            refresh_clock(cycles_);
            return;

        case ernie::kSfrCkc:
            // MCM0 is bit 4 and CSS is bit 6; MCS/CLS are read only.
            clock_.ckc = static_cast<u8>(value & (ernie::kCkcMcm0 | ernie::kCkcCss));
            refresh_clock(cycles_);
            return;

        case ernie::kSfrOstc:
            // OSTC is read only on silicon; accept and ignore the store so a
            // firmware that writes it does not disturb the status the boot ROM
            // polls at 0x3005A.
            return;

        case ernie::kSfrWdte:
            poke_watchdog(value);
            return;

        case ernie::kSfrItmc:
            arm_interval_timer(value);
            return;

        case ernie::kSfrIf0 + 1: {
            // IF0H is a request flag register: writing 0 clears, writing 1 is
            // ignored for the timer flags (the hardware sets them).
            u8& flags = regs_[ernie::kSfrIf0 + 1];
            flags = static_cast<u8>(flags & value);
            return;
        }
        case ernie::kSfrIf1 + 1: {
            u8& flags = regs_[ernie::kSfrIf1 + 1];
            flags = static_cast<u8>(flags & value);
            return;
        }
        default: break;
    }

    // RTC registers: a write moves the modelled wall clock so a firmware
    // RTC-set command sticks, everything else is plain storage.
    switch (address) {
        case ernie::kSfrSec:
        case ernie::kSfrMin:
        case ernie::kSfrHour:
        case ernie::kSfrWeek:
        case ernie::kSfrDay:
        case ernie::kSfrMonth:
        case ernie::kSfrYear: {
            auto it = regs_.find(address);
            if (it != regs_.end()) it->second = value;
            BcdTime t;
            t.sec = regs_[ernie::kSfrSec];
            t.min = regs_[ernie::kSfrMin];
            t.hour = regs_[ernie::kSfrHour];
            t.week = regs_[ernie::kSfrWeek];
            t.day = regs_[ernie::kSfrDay];
            t.month = regs_[ernie::kSfrMonth];
            t.year = regs_[ernie::kSfrYear];
            const int yy = from_bcd(t.year);
            int month = from_bcd(t.month);
            int day = from_bcd(t.day);
            if (month < 1 || month > 12) month = 1;
            if (day < 1) day = 1;
            if (day > days_in_month(2000 + yy, month)) day = days_in_month(2000 + yy, month);
            // Days from 2000-01-01 to the target date, on top of 946684800.
            static const int ydays[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
            int days = yy * 365 + (yy + 3) / 4;
            days += ydays[month - 1];
            if (month > 2 && (yy % 4 == 3) && ((2000 + yy) % 100 != 0 || (2000 + yy) % 400 == 0)) ++days;
            days += day - 1;
            const u64 base = 946684800ull;  // 2000-01-01T00:00:00Z
            rtc_seconds_ = base + static_cast<u64>(days) * 86400ull +
                           static_cast<u64>(from_bcd(t.hour)) * 3600ull +
                           static_cast<u64>(from_bcd(t.min)) * 60ull + from_bcd(t.sec);
            return;
        }
        default: break;
    }

    auto it = regs_.find(address);
    if (it != regs_.end()) it->second = value;
}

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

void ErnieSfr::tick(u64 cycles) {
    cycles_ = cycles;
    refresh_clock(cycles_);
    refresh_rtc();

    if (!interval_armed_) return;
    // The interval timer counts on the low speed on-chip oscillator (15 kHz on a
    // G13, so 2^15 counts is one second).  The emulated period is a machine
    // layer knob: what matters for bring-up is that the flag is raised at all,
    // because 0x30080..0x3008F is the first place the firmware blocks on an
    // interrupt (IF0H.TMIF01H at 0xFFFE1.6 with MK1H.6 clear).
    if (cycles_ < interval_next_) return;
    do {
        interval_next_ += interval_period_;
        ++interval_fires_;
    } while (interval_next_ <= cycles_);
    regs_[ernie::kSfrIf0 + 1] = static_cast<u8>(regs_[ernie::kSfrIf0 + 1] | ernie::kIf0hTmif01h);
    if ((regs_[ernie::kSfrMk1 + 1] & ernie::kIf0hTmif01h) == 0) {
        pending_vectors_.push_back(ernie::kIntervalTimerVector);
    }
}

// ---------------------------------------------------------------------------
// Panel lines / gauge / RTC
// ---------------------------------------------------------------------------

void ErnieSfr::set_power_button(bool pressed) {
    power_button_ = pressed;
    if (pressed) regs_[ernie::kSfrIf0] = static_cast<u8>(regs_[ernie::kSfrIf0] | 0x80);  // PIF5
    update_port_inputs();
}

void ErnieSfr::set_ps_button(bool pressed) {
    ps_button_ = pressed;
    if (pressed) regs_[ernie::kSfrIf0] = static_cast<u8>(regs_[ernie::kSfrIf0] | 0x40);  // PIF4
    update_port_inputs();
}

void ErnieSfr::set_volume_up(bool pressed) {
    volume_up_ = pressed;
    update_port_inputs();
}

void ErnieSfr::set_volume_down(bool pressed) {
    volume_down_ = pressed;
    update_port_inputs();
}

void ErnieSfr::set_rtc_seconds(u64 unix_seconds) {
    rtc_seconds_ = unix_seconds;
    refresh_rtc();
}

// ---------------------------------------------------------------------------
// Debugger
// ---------------------------------------------------------------------------

const char* ErnieSfr::register_name(u32 address) const {
    if (const char* name = ernie::sfr_name(address)) return name;
    return Device::register_name(address);
}

void ErnieSfr::enumerate_registers(std::vector<RegisterInfo>& out) const {
    for (const ernie::SfrDesc* it = kTable; it != kTableEnd; ++it) {
        RegisterInfo info;
        info.address = it->address;
        info.name = it->name;
        info.reset_value = it->reset;
        info.width = it->width;
        out.push_back(info);
    }
}

bool ErnieSfr::peek_register(const std::string& name, u64& out) const {
    for (const ernie::SfrDesc* it = kTable; it != kTableEnd; ++it) {
        if (name != it->name) continue;
        u64 value = 0;
        for (unsigned i = 0; i < it->width; ++i) {
            value |= static_cast<u64>(peek8(it->address + i)) << (8 * i);
        }
        out = value;
        return true;
    }
    return Device::peek_register(name, out);
}

bool ErnieSfr::poke_register(const std::string& name, u64 value) {
    for (const ernie::SfrDesc* it = kTable; it != kTableEnd; ++it) {
        if (name != it->name) continue;
        write(it->address, it->width, value);
        return true;
    }
    return Device::poke_register(name, value);
}

std::string ErnieSfr::describe_sfr(u32 address) const {
    const ernie::SfrDesc* desc = ernie::sfr_lookup(address);
    if (desc == nullptr) return {};
    u64 value = 0;
    for (unsigned i = 0; i < desc->width; ++i) {
        value |= static_cast<u64>(peek8(desc->address + i)) << (8 * i);
    }
    return format("%-8s = 0x%0*llX   %s", desc->name, static_cast<int>(desc->width * 2),
                  static_cast<unsigned long long>(value), desc->note);
}

std::string ErnieSfr::summary() const {
    return format("%s + %llu reads / %llu writes", ernie::clock_summary(clock_).c_str(),
                  static_cast<unsigned long long>(reads_),
                  static_cast<unsigned long long>(writes_));
}

void ErnieSfr::describe(std::vector<std::string>& lines) const {
    lines.push_back("Ernie SFR file (RL78/G1x map)");
    lines.push_back("  " + ernie::clock_summary(clock_));
    lines.push_back(format("  X1 oscillation: %s, stable=%s, OSTC=0x%02X",
                           clock_.x1_running ? "on" : "off", clock_.x1_stable ? "yes" : "no",
                           peek8(ernie::kSfrOstc)));
    lines.push_back(format("  RTC            : %02X-%02X-%02X %02X:%02X:%02X (unix %llu)",
                           peek8(ernie::kSfrYear), peek8(ernie::kSfrMonth), peek8(ernie::kSfrDay),
                           peek8(ernie::kSfrHour), peek8(ernie::kSfrMin), peek8(ernie::kSfrSec),
                           static_cast<unsigned long long>(rtc_seconds_)));
    lines.push_back(format("  panel          : power=%d ps=%d vol+=%d vol-=%d P1=0x%02X P6=0x%02X",
                           power_button_ ? 1 : 0, ps_button_ ? 1 : 0, volume_up_ ? 1 : 0,
                           volume_down_ ? 1 : 0, peek8(ernie::kSfrP1), peek8(ernie::kSfrP6)));
    lines.push_back(format("  fuel gauge     : %d%% %umV charger=%d", battery_percent_, battery_mv_,
                           charger_state_));
    lines.push_back(format("  interval timer : %s fires=%llu period=%llu cycles (IF0H.6=%d)",
                           interval_armed_ ? "armed" : "stopped",
                           static_cast<unsigned long long>(interval_fires_),
                           static_cast<unsigned long long>(interval_period_),
                           (peek8(ernie::kSfrIf0 + 1) & ernie::kIf0hTmif01h) ? 1 : 0));
    lines.push_back(format("  watchdog       : %llu writes, last fed=%s",
                           static_cast<unsigned long long>(watchdog_writes_),
                           watchdog_fed_ ? "yes" : "no"));
    lines.push_back(format("  accesses       : %llu reads, %llu writes",
                           static_cast<unsigned long long>(reads_),
                           static_cast<unsigned long long>(writes_)));
}

}  // namespace zlb

