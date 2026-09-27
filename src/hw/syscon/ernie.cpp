// zeliboba - Ernie (Renesas RL78 syscon) hardware block.
//
// See src/hw/syscon/ernie_internal.h for the reconstructed SC protocol and
// src/hw/syscon/ernie_sfr.h for the SFR map and its firmware evidence.
#include "hw/syscon.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "bus/bus.h"
#include "common/log.h"
#include "common/util.h"
#include "cpu/cpu.h"
#include "cpu/factory.h"
#include "cpu/rl78/rl78_core.h"
#include "hw/emmc.h"
#include "hw/syscon/ernie_internal.h"
#include "hw/syscon/ernie_power.h"
#include "hw/syscon/ernie_sfr.h"

namespace zlb {
namespace ernie {

/// Flash backed by a Bus RAM region.  ErnieBlock::install() re-uses the
/// "ernie_flash" region when the machine layer already mapped one (Vita::build_buses
/// does) and allocates it otherwise.
class FlashDevice : public Device {
public:
    explicit FlashDevice(Bus& bus)
        : Device("Ernie.Flash", kFlashBase, kFlashSize), bus_(bus) {}

    u8* data() {
        if (region_ == nullptr) region_ = bus_.region_at(kFlashBase, kFlashSize);
        return region_ ? region_->bytes() : nullptr;
    }

    size_t size() const { return region_ ? region_->size : 0; }

    /// True once the backing region covers the whole 1 MiB flash window.
    bool complete() const {
        return region_ != nullptr && region_->base <= kFlashBase &&
               static_cast<u64>(region_->base) + region_->size >=
                   static_cast<u64>(kFlashBase) + kFlashSize;
    }

    u64 read(u32 address, unsigned size) override {
        u8* bytes = data();
        if (bytes == nullptr || !complete()) return 0;
        const u32 offset = address - kFlashBase;
        if (static_cast<u64>(offset) + size > kFlashSize) return 0;
        u64 value = 0;
        for (unsigned i = 0; i < size; ++i) value |= static_cast<u64>(bytes[offset + i]) << (8 * i);
        return value;
    }

    void write(u32 address, unsigned size, u64 value) override {
        u8* bytes = data();
        if (bytes == nullptr || !complete()) return;
        const u32 offset = address - kFlashBase;
        if (static_cast<u64>(offset) + size > kFlashSize) return;
        // The RL78 programs flash through a boot routine; a plain data store is
        // honoured here so RAM-clear loops and the debugger behave.
        for (unsigned i = 0; i < size; ++i) {
            bytes[offset + i] = static_cast<u8>((value >> (8 * i)) & 0xFF);
        }
    }

    std::string summary() const override {
        const size_t bytes = size();
        return format("%s of flash fitted", human_size(bytes ? bytes : kFlashSize).c_str());
    }

private:
    Bus& bus_;
    MemRegion* region_ = nullptr;
};

}  // namespace ernie

// ---------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------

struct ErnieBlock::Impl {
    Impl(ErnieBlock& owner, Bus& bus, EmmcCard* card)
        : bus(bus), card(card), channel(owner), emmc_host(card) {}

    Bus& bus;
    EmmcCard* card = nullptr;

    ErnieSfr* sfr = nullptr;  ///< owned by the bus once installed
    ernie::FlashDevice* flash = nullptr;
    ernie::ScWindowDevice* sc_window = nullptr;
    ernie::ScMessageWindow* sc_cmd_window = nullptr;
    ernie::ScMessageWindow* sc_resp_window = nullptr;
    ernie::SocGateDevice* soc_gate = nullptr;
    ernie::ScStrapDevice* strap = nullptr;

    ernie::ScRegs sc_regs;
    ernie::ScChannel channel;
    ernie::EmmcHost emmc_host;
    ernie::PowerState power;
    ernie::PowerPolicy policy;

    bool installed = false;
    u64 cycles = 0;  ///< absolute cycle count fed to the clock model
    u64 ticks = 0;
    u64 irqs_raised = 0;

    /// (command, reply size) of the last commands the dispatcher answered.
    std::vector<std::pair<u32, u32>> recent_commands;

    /// How often each per-console record (0x00A0) has been served.  The boot
    /// chain reads the same record twice and expects the syscon's record
    /// lifecycle: version 1 on the first read and version 3 once the record has
    /// been used (0x48E24 then 0x4906C).
    std::map<u8, unsigned> record_reads;

    void forward_interrupts(Cpu* cpu);
    std::vector<u8> dispatch(u32 command, const std::vector<u8>& payload);
    std::vector<u8> ok_reply(const std::vector<u8>& payload, u8 status = 0) const;
    std::vector<u8> error_reply(u8 error, u8 detail = 0) const;
    void note_command(u32 command, size_t reply_size);
};

namespace {

/// Build the 4 + 32 byte response record the Ernie SC handlers write at 0x0DD98:
///   +0 result (0 = ok, 0xFE = busy, anything else = error)
///   +1 status byte (panel / card status)
///   +2 payload length (the handlers store 0x02..0x24 here)
///   +3 payload
std::vector<u8> build_reply(u8 result, u8 status, const u8* payload, size_t length) {
    std::vector<u8> reply(4 + 32, 0);
    reply[0] = result;
    reply[1] = status;
    reply[2] = static_cast<u8>(std::min<size_t>(length, 32));
    for (size_t i = 0; i < length && i < 32; ++i) reply[3 + i] = payload[i];
    return reply;
}

std::vector<u8> u32_payload(u32 value) {
    std::vector<u8> out(4);
    ernie::store_le32(out.data(), value);
    return out;
}

std::vector<u8> u16_payload(u16 value) {
    std::vector<u8> out(2);
    out[0] = static_cast<u8>(value);
    out[1] = static_cast<u8>(value >> 8);
    return out;
}

/// bq27520 standard command addresses (Abby/bq27520-g4.pdf 7.1).  The syscon
/// firmware only ever shuttles these 16-bit words between the gauge and the SC
/// caller, so the model answers them directly.
enum GaugeCommand : u8 {
    kGaugeControl = 0x00,
    kGaugeDeviceType = 0x01,
    kGaugeFirmwareVersion = 0x02,
    kGaugeSpecVersion = 0x04,
    kGaugeFlags = 0x06,
    kGaugeDesignCapacity = 0x08,
    kGaugeFullAvailableCapacity = 0x0A,
    kGaugeRemainingCapacity = 0x0C,
    kGaugeFullChargeCapacity = 0x0E,
    kGaugeAverageCurrent = 0x10,
    kGaugeStandbyCurrent = 0x12,
    kGaugeMaxLoadCurrent = 0x14,
    kGaugeAveragePower = 0x16,
    kGaugeStateOfCharge = 0x18,
    kGaugeInternalTemperature = 0x1A,
    kGaugeVoltage = 0x1C,
};

u16 gauge_read_word(const ernie::PowerState& state, u8 command) {
    switch (command) {
        case kGaugeControl: return 0x0000;
        case kGaugeDeviceType: return 0x0520;       // bq27520-G4 device type
        case kGaugeFirmwareVersion: return 0x0107;  // v1.07
        case kGaugeSpecVersion: return 0x0021;      // bq27xxx spec 2.1
        case kGaugeFlags: {
            // Flags(): bit 0 DSG (discharging), bit 4 CHG (charging), bit 5 FC
            // (fully charged), bit 8 OCVTAKEN, bit 15 BAT_DET.
            u16 flags = 0x8100;
            if (state.charger_state == 0) flags |= 0x0001;
            else if (state.charger_state == 1) flags |= 0x0010;
            else flags |= 0x0020;
            return flags;
        }
        case kGaugeDesignCapacity:
        case kGaugeFullAvailableCapacity:
        case kGaugeFullChargeCapacity: return state.design_capacity_mah;
        case kGaugeRemainingCapacity:
            return static_cast<u16>((static_cast<u32>(state.design_capacity_mah) *
                                     static_cast<u32>(state.battery_percent)) / 100u);
        case kGaugeAverageCurrent: return static_cast<u16>(state.current_ma);
        case kGaugeStandbyCurrent: return 0x0000;
        case kGaugeMaxLoadCurrent: return static_cast<u16>(-2400);
        case kGaugeAveragePower: return static_cast<u16>(state.current_ma < 0 ? -900 : 3200);
        case kGaugeStateOfCharge: return static_cast<u16>(state.battery_percent);
        case kGaugeInternalTemperature: return state.temperature_dk;
        case kGaugeVoltage: return static_cast<u16>(state.voltage_mv);
        default: return 0xFFFF;  // the gauge's own "not implemented" encoding
    }
}

bool gauge_write_word(ernie::PowerState& state, u8 command, u16 value) {
    switch (command) {
        case kGaugeControl: return true;  // SEALED / IT_ENABLE / RESET: accepted
        case kGaugeDesignCapacity:
            state.design_capacity_mah = value;
            return true;
        default: return false;
    }
}

/// Distinguish the two boards by their command table location.  The scripts
/// vita_command_table_uss1001.py (offset 0x26BE, 70 entries) and
/// vita_command_table_uss1002.py (offset 0x3096, 73 entries) both start with
/// command 0x0000 / flags 0x0000, so the presence of a valid entry at 0x3096 is
/// the discriminator.
bool looks_like_uss1002(const std::vector<u8>& dump) {
    if (dump.size() < 0x3096 + 8) return false;
    const u16 number = static_cast<u16>(dump[0x3096] | (dump[0x3097] << 8));
    const u16 flags = static_cast<u16>(dump[0x3098] | (dump[0x3099] << 8));
    return number == 0x0000 && flags == 0x0000;
}

}  // namespace

void ErnieBlock::Impl::note_command(u32 command, size_t reply_size) {
    recent_commands.push_back({command, static_cast<u32>(reply_size)});
    if (recent_commands.size() > 32) recent_commands.erase(recent_commands.begin());
}

std::vector<u8> ErnieBlock::Impl::ok_reply(const std::vector<u8>& payload, u8 status) const {
    return build_reply(ernie::kScResultOk, status, payload.data(), payload.size());
}

std::vector<u8> ErnieBlock::Impl::error_reply(u8 error, u8 detail) const {
    const u8 payload[1] = {detail};
    return build_reply(error, detail, payload, 1);
}

void ErnieBlock::Impl::forward_interrupts(Cpu* cpu) {
    if (cpu == nullptr || sfr == nullptr) return;
    // The SFR file is the INTC: it knows which vector fired and whether the mask
    // bit let it through.  The RL78 core has no INTC detail of its own, so the
    // vector is programmed into its IRQ slot 0 before the line is pulsed.
    const std::vector<int>& pending = sfr->pending_vectors();
    for (int vector : pending) {
        if (auto* rl78 = dynamic_cast<Rl78Core*>(cpu)) {
            rl78->set_irq_vector(0, vector);
            rl78->set_irq(0, true);
            rl78->set_irq(0, false);
        } else {
            cpu->set_irq(static_cast<int>(IrqLine::Syscon), true);
            cpu->set_irq(static_cast<int>(IrqLine::Syscon), false);
        }
        ++irqs_raised;
    }
    sfr->clear_pending_vectors();
}

/// The SoC side can also post a command by writing a descriptor into the command
/// window; the CMeP's sc_xfer path goes through CmepBlock instead, but a
/// debugger or the SoC-side bridge can fill the window directly.

// ---------------------------------------------------------------------------
// The functional SC dispatcher
// ---------------------------------------------------------------------------

std::vector<u8> ErnieBlock::Impl::dispatch(u32 command, const std::vector<u8>& payload) {
    // Bring-up trace: `log debug` prints every SC command with its payload size,
    // which is how the second loader's storage traffic is followed.
    ZLB_LOG_DBG("ernie.sc", "cmd 0x%04X payload=%u", command, static_cast<unsigned>(payload.size()));
    switch (command) {
        // --- identity / status ------------------------------------------
        case 0x0000: {  // get_status (handler 0x35C19, reply length 0x02)
            // The second loader polls this command while it brings its SC engine
            // up. The reply is the two status bytes the firmware handler builds
            // (SoC release flag, charger state); the SPI call path caps the
            // reply frame length byte at 6 (second loader 0x442AE), so two bytes
            // is the largest payload this path accepts.
            return ok_reply({static_cast<u8>(power.soc_released ? 1 : 0),
                             static_cast<u8>(power.charger_state)},
                            static_cast<u8>(power.power_button ? 0x01 : 0x00));
        }
        case 0x0001: {  // get_boot_info / "Ernie version" (0x35C41)
            // The SPI call path that submits this command caps the reply frame
            // length byte at 6 (second loader 0x442AE: `len - 3 > 3` is an
            // error), i.e. a 4 byte payload.  The wiki's boot trace shows the
            // same shape: CMD 0x0001 is answered with `04 00 06 00 0D 06 00 01
            // E1`, so the payload is the Ernie firmware version as four bytes.
            return ok_reply({0x0D, 0x06, 0x00, 0x01});
        }
        case 0x0002: {  // get_model_id / firmware timestamp (0x35C78, length 0x0E)
            // The wrapper copies exactly 12 bytes (second loader 0x44678), and
            // the wiki's trace shows an ASCII build stamp here.
            static const char kStamp[] = "201211081704";
            return ok_reply(std::vector<u8>(kStamp, kStamp + 12));
        }
        case 0x0005: {  // get_boot_mode / hardware info (0x35E83, length 0x06)
            // Wiki: `> 0x0005` -> `< 0x0004 payload=[00 60 40 00]`. The second
            // loader feeds these bits into its boot parameter table (0x4CF00),
            // so the values become part of the ARM's boot context.
            return ok_reply({0x00, 0x60, 0x40, 0x00});
        }
        case 0x0010: {  // get_version / wakeup factor (0x35EF1, length 0x04)
            // Wiki: `> 0x0010` -> `< 0x0004 payload=[14 FF]` - the boot type,
            // 0xFF14 cold boot and 0xFF80 resume. The second loader submits this
            // through the small-reply API (0x446F2: `mov $2,16; jmp 0x44274`),
            // which rejects any reply longer than four bytes, so the payload
            // has to stay this short.
            return ok_reply({0x14, 0xFF});
        }
        case 0x0003:    // get_model_string (0x35CAF)
        case 0x0004:    // get_serial (0x35D49)
        case 0x0006:    // get_uid (0x35EBA, length 0x12)
        case 0x0011:    // get_version_ext (0x35F43)
        case 0x0012: {  // get_firmware_version (0x35FE6)
            std::vector<u8> data(6, 0);
            data[0] = 0x01;  // USS-1001
            data[1] = 0x00;
            data[2] = power.soc_released ? 1 : 0;
            data[3] = static_cast<u8>(power.battery_percent);
            data[4] = static_cast<u8>(power.charger_state);
            data[5] = 0x41;  // boot mode 'A' (normal), cmep::CmepBlock default
            return ok_reply(data);
        }
        case 0x0013:    // get_last_result (0x360A1, length 0x04)
            return ok_reply(u32_payload(0));
        case 0x0015:    // get_error_detail (0x360FE)
            return ok_reply(std::vector<u8>(2, 0));

        // --- per-console records ------------------------------------------
        case 0x00A0: {
            // The second loader keeps its 40 byte configuration records in the
            // syscon's non-volatile storage.  0x48D28 builds the "absent"
            // template {0x30, 0, kind, index, ...}, the loader hands it to the
            // syscon (this command, a 40 byte payload) and 0x48E24 then insists
            // on version 1 ("record present", byte 1) with the same index in
            // byte 3.  A console that owns the record always answers with the
            // stored version, so the model echoes the template with version 1
            // set and its payload zeroed - the same development substitution
            // machine/bootkeys.cpp applies to the per-console keyring, because
            // no dump in the workspace carries the syscon NVS.
            // `ZLB_NO_SUBSTITUTION=1` answers with the empty record instead.
            static const bool disabled = [] {
                const char* value = std::getenv("ZLB_NO_SUBSTITUTION");
                return value != nullptr && value[0] != '0';
            }();
            // Bring-up knob: the version the synthesised record reports.  The
            // boot chain validates the same request with version 1 (0x48E24) and
            // later with version 3 (0x4906C, the "record was updated" state), so
            // this makes the two stages distinguishable from the console.
            static const u8 forced_version = [] {
                const char* value = std::getenv("ZLB_RECORD_VERSION");
                if (value == nullptr) return static_cast<u8>(0);
                return static_cast<u8>(std::strtoul(value, nullptr, 0));
            }();
            std::vector<u8> record = payload;
            if (record.size() >= 4 && record[0] == 0x30) {
                if (!disabled) {
                    // version 1 while the record is "fresh", 3 once the boot
                    // chain has read it before (the two validators).
                    const u8 index = record[3];
                    const unsigned reads = ++record_reads[index];
                    record[1] = forced_version != 0 ? forced_version : (reads <= 2 ? 1 : 3);
                    record[2] = 0;  // kind 0 = plain record
                    for (size_t i = 4; i < record.size(); ++i) record[i] = 0;
                    ZLB_LOG_DBG("ernie.sc", "record index=%u read=%u -> version %u", index, reads,
                                record[1]);
                }
            } else {
                // A request without the record header is answered with a valid,
                // zeroed template so the caller's state machine still advances.
                record.assign(40, 0);
            }
            // build_reply caps the payload at the firmware's 32 bytes, so the 40
            // byte record is framed by hand here.
            std::vector<u8> framed(4 + record.size(), 0);
            framed[0] = 0;  // result: ok
            framed[2] = static_cast<u8>(record.size());
            std::copy(record.begin(), record.end(), framed.begin() + 3);
            return framed;
        }

        // --- panel / UART state -----------------------------------------
        case 0x0100:    // get_panel_state (0x36132, length 0x06)
        case 0x0103:    // get_panel_state2 (0x361C5, length 0x06)
        case 0x0130: {  // get_uart_state (0x36266, length 0x06)
            std::vector<u8> data(6, 0);
            data[0] = sfr ? sfr->peek8(ernie::kSfrP7) : 0;
            data[1] = sfr ? sfr->peek8(ernie::kSfrP5) : 0;
            data[2] = static_cast<u8>(power.power_button ? 0 : 1);
            data[3] = static_cast<u8>(power.ps_button ? 0 : 1);
            data[4] = static_cast<u8>(power.volume_up ? 0 : 1);
            data[5] = static_cast<u8>(power.volume_down ? 0 : 1);
            return ok_reply(data);
        }

        // --- RTC ---------------------------------------------------------
        case 0x0080:    // get_rtc (0x36378, length 0x04)
        case 0x0081:    // set_rtc (0x36448)
        case 0x0082: {  // set_rtc_alarm (0x364B8)
            if (payload.size() >= 4 && (command == 0x0081 || command == 0x0082)) {
                const u64 seconds = ernie::load_le32(payload.data());
                power.rtc_seconds = seconds;
                if (sfr) sfr->set_rtc_seconds(seconds);
            }
            return ok_reply(u32_payload(static_cast<u32>(power.rtc_seconds)));
        }
        case 0x0083:    // get_rtc_alarm
        case 0x0084:    // get_rtc_alarm2
        case 0x0085:    // get_rtc_alarm3
            return ok_reply(u32_payload(static_cast<u32>(power.rtc_seconds)));

        // --- power / reset -----------------------------------------------
        case 0x0900:    // reset_device (0x373CF)
        case 0x0901:    // suspend (0x37437)
        case 0x0902:    // resume (0x3749F)
            if (command == 0x0900) power.release_soc();
            return ok_reply(u32_payload(0));

        // --- fuel gauge ---------------------------------------------------
        case 0x0800: {  // get_battery (0x37253, length 0x06)
            std::vector<u8> data(6, 0);
            data[0] = static_cast<u8>(power.battery_percent);
            data[1] = static_cast<u8>(power.charger_state);
            data[2] = static_cast<u8>(power.voltage_mv & 0xFF);
            data[3] = static_cast<u8>(power.voltage_mv >> 8);
            const u16 current = static_cast<u16>(power.current_ma);
            data[4] = static_cast<u8>(current & 0xFF);
            data[5] = static_cast<u8>(current >> 8);
            return ok_reply(data);
        }
        case 0x0806:    // get_charger_state (0x372DA, length 0x03)
        case 0x0820:    // get_charge_current (0x37319, length 0x02)
        case 0x0821:    // get_charge_voltage (0x37347, length 0x02)
        case 0x0822: {  // get_charge_temp (0x3737A, length 0x02)
            std::vector<u8> data(2, 0);
            // When a charger is present the gauge reports the charge current
            // (percent of full), otherwise the remaining charge.
            const int charge = power.charger_state == 0 ? power.battery_percent
                                                        : 100 - power.battery_percent;
            data[0] = static_cast<u8>(charge);
            data[1] = static_cast<u8>(power.voltage_mv >> 4);
            return ok_reply(data);
        }
        case 0x0891:    // gauge_read_word (0x36ED3)
        case 0x088E: {  // gauge_command (0x36DD4)
            if (payload.empty()) return error_reply(0x02, 0x00);
            const u8 reg = payload[0];
            const u16 value = gauge_read_word(power, reg);
            return ok_reply(u16_payload(value));
        }        case 0x088F: {  // gauge_write_word (0x36CE0)
            if (payload.size() < 3) return error_reply(0x02, 0x00);
            const u8 reg = payload[0];
            const u16 value = static_cast<u16>(payload[1] | (payload[2] << 8));
            (void)gauge_write_word(power, reg, value);
            return ok_reply(u16_payload(value));
        }
        case 0x089B: {  // gauge_read_block (0x370EC)
            if (payload.size() < 3) return error_reply(0x02, 0x00);
            const u8 length = static_cast<u8>(std::min<unsigned>(payload[2], 32));
            std::vector<u8> block(length, 0);
            if (length >= 2) {
                block[0] = 0x20;  // bq27520 device type low byte
                block[1] = 0x05;
            }
            if (length >= 3) block[2] = static_cast<u8>(power.battery_percent);
            return ok_reply(block);
        }

        // --- eMMC ---------------------------------------------------------
        case 0x1100: {  // storage_read (0x35B57, length 0x06 without data)
            if (payload.size() < 8) {
                std::vector<u8> data(6, 0);
                if (card != nullptr && card->attached()) {
                    data[0] = static_cast<u8>(emmc_host.partition());
                    const u64 blocks = card->block_count();
                    for (int i = 0; i < 4; ++i) data[1 + i] = static_cast<u8>((blocks >> (8 * i)) & 0xFF);
                    data[5] = 0x01;
                }
                return ok_reply(data);
            }
            const u64 lba = ernie::load_le32(payload.data());
            const u32 count = ernie::load_le32(payload.data() + 4);
            std::vector<u8> data;
            if (!emmc_host.read_blocks(lba, count, data)) return error_reply(0x03, 0x00);
            ZLB_LOG_DBG("ernie.sc", "storage_read lba=%llu count=%u -> %u bytes, first=%02X %02X %02X %02X",
                        static_cast<unsigned long long>(lba), count, static_cast<unsigned>(data.size()),
                        data.size() > 0 ? data[0] : 0, data.size() > 1 ? data[1] : 0,
                        data.size() > 2 ? data[2] : 0, data.size() > 3 ? data[3] : 0);
            return ok_reply(data, static_cast<u8>(emmc_host.card_status() & 0xFF));
        }
        case 0x1101: {  // storage_write (0x35B8E)
            if (payload.size() < 8) return error_reply(0x02, 0x00);
            const u64 lba = ernie::load_le32(payload.data());
            const u32 count = ernie::load_le32(payload.data() + 4);
            std::vector<u8> data(payload.begin() + 8, payload.end());
            if (!emmc_host.write_blocks(lba, count, data)) return error_reply(0x03, 0x00);
            return ok_reply(u32_payload(0));
        }
        case 0x1080:  // emmc_init (0x375CF)
            return emmc_host.init_card() ? ok_reply(u32_payload(emmc_host.card_status()))
                                         : error_reply(0x04, 0x00);
        case 0x1081: {  // emmc_get_cid (0x375FC)
            if (card == nullptr || !card->attached()) return error_reply(0x04, 0x00);
            const std::array<u8, 16>& cid = card->cid();
            return ok_reply(std::vector<u8>(cid.begin(), cid.end()));
        }
        case 0x1082: {  // emmc_get_csd (0x3764D)
            if (card == nullptr || !card->attached()) return error_reply(0x04, 0x00);
            const std::array<u8, 16>& csd = card->csd();
            return ok_reply(std::vector<u8>(csd.begin(), csd.end()));
        }
        case 0x1083: {  // emmc_get_ext_csd (0x37722)
            if (card == nullptr || !card->attached()) return error_reply(0x04, 0x00);
            const std::vector<u8>& ext = card->ext_csd();
            return ok_reply(std::vector<u8>(ext.begin(), ext.begin() + std::min<size_t>(ext.size(), 32)));
        }
        case 0x1180: {  // emmc_read (0x35749, length 0x06)
            if (payload.size() >= 8) {
                const u64 lba = ernie::load_le32(payload.data());
                const u32 count = ernie::load_le32(payload.data() + 4);
                std::vector<u8> data;
                if (!emmc_host.read_blocks(lba, count, data)) return error_reply(0x03, 0x00);
                return ok_reply(data, static_cast<u8>(emmc_host.card_status() & 0xFF));
            }
            std::vector<u8> data(6, 0);
            if (card != nullptr && card->attached()) {
                data[0] = static_cast<u8>(emmc_host.partition());
                data[1] = 0x01;  // card initialised
            }
            return ok_reply(data);
        }
        case 0x1181: {  // emmc_write (0x3578B)
            if (payload.size() < 8) return error_reply(0x02, 0x00);
            const u64 lba = ernie::load_le32(payload.data());
            const u32 count = ernie::load_le32(payload.data() + 4);
            std::vector<u8> data(payload.begin() + 8, payload.end());
            if (!emmc_host.write_blocks(lba, count, data)) return error_reply(0x03, 0x00);
            return ok_reply(u32_payload(0));
        }
        case 0x1182: {  // emmc_erase (0x35864)
            if (payload.size() < 8) return error_reply(0x02, 0x00);
            const u64 lba = ernie::load_le32(payload.data());
            const u32 count = ernie::load_le32(payload.data() + 4);
            if (!emmc_host.erase_blocks(lba, count)) return error_reply(0x03, 0x00);
            return ok_reply(u32_payload(0));
        }
        case 0x1183: {  // emmc_switch_partition (0x35A8C)
            if (payload.size() < 4) return error_reply(0x02, 0x00);
            if (!emmc_host.select_partition(ernie::load_le32(payload.data()))) {
                return error_reply(0x03, 0x00);
            }
            return ok_reply(u32_payload(static_cast<u32>(emmc_host.partition())));
        }
        case 0x1184:    // emmc_boot_select (0x35961)
        case 0x1185: {  // emmc_boot_read (0x35B36, length 0x06)
            if (payload.size() >= 8) {
                const u64 lba = ernie::load_le32(payload.data());
                const u32 count = ernie::load_le32(payload.data() + 4);
                std::vector<u8> data;
                if (!emmc_host.read_blocks(lba, count, data)) return error_reply(0x03, 0x00);
                return ok_reply(data, static_cast<u8>(emmc_host.card_status() & 0xFF));
            }
            std::vector<u8> data(6, 0);
            if (card != nullptr && card->attached()) {
                const u64 blocks = card->block_count();
                for (int i = 0; i < 4; ++i) data[i] = static_cast<u8>((blocks >> (8 * i)) & 0xFF);
                data[4] = static_cast<u8>(emmc_host.partition());
                data[5] = 0x01;
            }
            return ok_reply(data);
        }

        // --- SoC control ---------------------------------------------------
        case 0x2080:  // soc_release (0x377CA, length 0x02)
            power.release_soc();
            if (soc_gate) soc_gate->set_released(true);
            return ok_reply(u32_payload(1));
        case 0x2081:  // soc_hold (0x37800)
            power.soc_released = false;
            if (soc_gate) soc_gate->set_released(false);
            return ok_reply(u32_payload(0));
        case 0x2082:  // soc_reset (0x37836)
            return ok_reply(u32_payload(0));
        case 0x2085:  // soc_status (0x37B04)
            return ok_reply(u32_payload(power.soc_released ? 1u : 0u));

        default: break;
    }

    // The Ernie dispatcher looks the command up in the table at 0x26BE and posts
    // error 0x01 ("unknown command") when it misses.  A known command with no
    // functional model is acknowledged with an empty payload so the caller does
    // not stall.
    if (ernie::sc_command_info(command) == nullptr) return error_reply(0x01, 0x00);
    return ok_reply({});
}

// ---------------------------------------------------------------------------
// ErnieBlock
// ---------------------------------------------------------------------------

ErnieBlock::ErnieBlock(Bus& bus, EmmcCard* card)
    : impl_(new Impl(*this, bus, card)), cpu_(nullptr) {}

ErnieBlock::~ErnieBlock() = default;

void ErnieBlock::install() {
    Impl& impl = *impl_;
    if (impl.installed) return;

    // Flash: re-use the machine layer's "ernie_flash" region when it exists (the
    // Vita builder maps one), otherwise allocate it here so a standalone Ernie
    // bus works too.
    if (impl.bus.region_at(ernie::kFlashBase, ernie::kFlashSize) == nullptr) {
        impl.bus.add_ram("ernie_flash", ernie::kFlashSize, ernie::kFlashBase, "Ernie code/data flash");
    }
    impl.flash = new ernie::FlashDevice(impl.bus);
    impl.bus.add_device(std::unique_ptr<Device>(impl.flash));

    // SFR window plus its 32-bit mirror (the RL78 forms an address by masking to
    // 20 bits, but the CMeP/SoC side sees the mirrored page).
    impl.sfr = new ErnieSfr();
    impl.bus.add_device(std::unique_ptr<Device>(impl.sfr));
    impl.bus.add_device(std::unique_ptr<Device>(
        new DeviceMirror(*impl.sfr, ernie::kSfrMirrorBase, ernie::kSfrSize)));

    // SC registers, message windows and the SoC boot gate.  The window device
    // reads through the channel so there is exactly one ScRegs instance.
    impl.sc_window = new ernie::ScWindowDevice();
    impl.sc_window->set_channel(&impl.channel);
    impl.bus.add_device(std::unique_ptr<Device>(impl.sc_window));

    impl.sc_cmd_window = new ernie::ScMessageWindow(ernie::kScCmdWindow, 0x200, false);
    impl.bus.add_device(std::unique_ptr<Device>(impl.sc_cmd_window));
    impl.sc_resp_window = new ernie::ScMessageWindow(ernie::kScRespWindow, 0x200, true);
    impl.bus.add_device(std::unique_ptr<Device>(impl.sc_resp_window));

    impl.soc_gate = new ernie::SocGateDevice();
    impl.bus.add_device(std::unique_ptr<Device>(impl.soc_gate));
    impl.strap = new ernie::ScStrapDevice();
    impl.bus.add_device(std::unique_ptr<Device>(impl.strap));

    impl.installed = true;
    impl.power.bind(impl.sfr, &impl.emmc_host);

    // The RL78 core fetches from the flash device through the bus.
    cpu_ = create_rl78_core(impl.bus);
    if (cpu_) {
        cpu_->name = "Ernie";
        if (firmware_loaded_) cpu_->reset(reset_vector_);
    } else {
        ZLB_LOG_WARN("ernie", "no RL78 core available, the functional SC model answers instead");
        run_firmware_ = false;
    }

    size_t sfr_count = 0;
    ernie::sfr_table(sfr_count);
    ZLB_LOG_INFO("ernie", "installed: %s flash, %u SFRs, SC window 0x%08X, %s",
                 human_size(ernie::kFlashSize).c_str(), static_cast<unsigned>(sfr_count),
                 ernie::kScWindowBase,
                 cpu_ ? "RL78 core ready" : "functional SC model only");
}

void ErnieBlock::reset() {
    Impl& impl = *impl_;
    milliseconds_ = 0;
    rtc_seconds_ = 0x4E24E1C0;
    battery_percent_ = 87;
    charger_state_ = 0;
    soc_released_ = false;
    response_ready_ = false;
    busy_ = false;
    response_.clear();
    commands_served_ = 0;

    impl.power.reset();
    impl.channel.reset();
    impl.emmc_host.reset();
    impl.power.bind(impl.sfr, &impl.emmc_host);
    impl.cycles = 0;
    impl.ticks = 0;
    impl.irqs_raised = 0;
    impl.recent_commands.clear();

    impl.bus.reset_devices();
    impl.power.apply();

    if (cpu_) {
        cpu_->reset(reset_vector_);
        cpu_->halted = false;
        cpu_->halt_reason.clear();
    }
    ZLB_LOG_INFO("ernie", "reset: RL78 at 0x%05X (%s)", reset_vector_,
                 firmware_loaded_ ? "firmware fitted" : "no firmware");
}

bool ErnieBlock::load_firmware(const std::vector<u8>& dump, bool run_firmware) {
    Impl& impl = *impl_;
    run_firmware_ = run_firmware;
    if (dump.empty()) {
        firmware_loaded_ = false;
        ZLB_LOG_WARN("ernie", "no firmware dump supplied, the functional SC model answers");
        if (cpu_) {
            cpu_->halted = true;
            cpu_->halt_reason = "no firmware";
        }
        return false;
    }
    if (dump.size() > ernie::kFlashSize) {
        ZLB_LOG_ERROR("ernie", "dump is %s, the flash is only %s", human_size(dump.size()).c_str(),
                      human_size(ernie::kFlashSize).c_str());
        return false;
    }
    if (impl.flash == nullptr) {
        ZLB_LOG_ERROR("ernie", "install() must run before load_firmware()");
        return false;
    }
    u8* flash = impl.flash->data();
    if (flash == nullptr || !impl.flash->complete()) {
        ZLB_LOG_ERROR("ernie", "flash region is not mapped or too small");
        return false;
    }

    std::memcpy(flash, dump.data(), dump.size());
    if (dump.size() < ernie::kFlashSize) {
        std::memset(flash + dump.size(), 0xFF, ernie::kFlashSize - dump.size());
    }

    // Both dumps keep the RL78 reset vector at offset 0 (0xE000 for USS-1001,
    // 0x0DC00 for USS-1002).  Read it back so the fitted image always decides,
    // and fall back to the board constant only when the vector is blank.
    reset_vector_ = static_cast<u32>(flash[0]) | (static_cast<u32>(flash[1]) << 8);
    if (reset_vector_ == 0x0000 || reset_vector_ == 0xFFFF) {
        reset_vector_ = looks_like_uss1002(dump) ? ernie::kResetVector1002 : ernie::kResetVector1001;
    }

    firmware_loaded_ = true;
    if (cpu_) {
        cpu_->reset(reset_vector_);
        cpu_->halted = !run_firmware;
        cpu_->halt_reason = run_firmware ? std::string() : std::string("functional SC model");
    }
    ZLB_LOG_INFO("ernie", "firmware %s fitted, reset vector 0x%05X, %s", human_size(dump.size()).c_str(),
                 reset_vector_, run_firmware ? "RL78 core active" : "functional SC model active");
    return true;
}

std::vector<u8> ErnieBlock::dispatch_command(u32 command, const std::vector<u8>& payload) {
    Impl& impl = *impl_;
    // The fuel gauge / panel / RTC values are mirrored in ErnieBlock's own
    // members (the public API's setters are inline there), so fold them back
    // into the power model before it answers.
    impl.power.battery_percent = battery_percent_;
    impl.power.charger_state = charger_state_;
    impl.power.rtc_seconds = rtc_seconds_;

    std::vector<u8> reply = impl.dispatch(command, payload);
    ++commands_served_;
    impl.note_command(command, reply.size());
    impl.channel.publish_reply(command, reply);
    response_ = reply;
    response_ready_ = !reply.empty();
    busy_ = false;
    if (impl.sc_resp_window != nullptr) impl.sc_resp_window->load(reply);

    // Reflect anything the command changed (an SC "set RTC", a power-off).
    rtc_seconds_ = impl.power.rtc_seconds;
    battery_percent_ = impl.power.battery_percent;
    charger_state_ = impl.power.charger_state;
    soc_released_ = impl.power.soc_released;
    return reply;
}

void ErnieBlock::clear_response() {
    response_.clear();
    response_ready_ = false;
    impl_->channel.on_reply_cleared();
}

bool ErnieBlock::firmware_loaded() const { return firmware_loaded_; }

void ErnieBlock::command_posted(const u8* descriptor, size_t length) {
    Impl& impl = *impl_;
    if (descriptor == nullptr || length == 0) return;
    impl.channel.post_descriptor(descriptor, length, ernie::kScCmdWindow);
    const ernie::ScRegs& regs = impl.channel.regs();
    commands_served_ = regs.commands;
    response_ = regs.response;
    response_ready_ = !response_.empty();
    busy_ = false;
    impl.note_command(regs.command, response_.size());
}

void ErnieBlock::advance_milliseconds(u64 delta) {    Impl& impl = *impl_;
    impl.power.advance_milliseconds(delta);
    ernie::run_power_state_machine(impl.power, impl.policy);
    milliseconds_ = impl.power.milliseconds;
    rtc_seconds_ = impl.power.rtc_seconds;
    battery_percent_ = impl.power.battery_percent;
    charger_state_ = impl.power.charger_state;
    soc_released_ = impl.power.soc_released;
}

void ErnieBlock::tick(u64 cycles) {
    Impl& impl = *impl_;
    if (impl.sfr != nullptr) impl.sfr->tick(cycles);
}

void ErnieBlock::set_power_button(bool pressed) {
    impl_->power.set_power_button(pressed);
    if (pressed) impl_->power.release_soc();
    soc_released_ = impl_->power.soc_released;
}

void ErnieBlock::set_ps_button(bool pressed) { impl_->power.set_ps_button(pressed); }
void ErnieBlock::set_volume_up(bool pressed) { impl_->power.set_volume_up(pressed); }
void ErnieBlock::set_volume_down(bool pressed) { impl_->power.set_volume_down(pressed); }

void ErnieBlock::release_soc() {
    impl_->power.release_soc();
    if (impl_->soc_gate != nullptr) impl_->soc_gate->set_released(true);
    soc_released_ = true;
}

// ---------------------------------------------------------------------------
// Debugger surface
// ---------------------------------------------------------------------------

std::string ErnieBlock::summary() const {
    const Impl& impl = *impl_;
    std::string core = "no core";
    if (cpu_ != nullptr) {
        core = format("pc=0x%05X insns=%llu", cpu_->get_pc(),
                      static_cast<unsigned long long>(cpu_->instructions));
    }
    return format("%s | %s | %s | %llu SC commands | %llu IRQs",
                  firmware_loaded_ ? (run_firmware_ ? "RL78 firmware running" : "RL78 firmware idle")
                                   : "functional SC model",
                  core.c_str(), impl.emmc_host.summary().c_str(),
                  static_cast<unsigned long long>(commands_served_),
                  static_cast<unsigned long long>(impl.irqs_raised));
}

void ErnieBlock::describe(std::vector<std::string>& lines) const {
    const Impl& impl = *impl_;
    lines.push_back("Ernie (Renesas RL78 syscon)");
    lines.push_back(format("  firmware   : %s, reset vector 0x%05X, mode %s",
                           firmware_loaded_ ? "fitted" : "absent", reset_vector_,
                           run_firmware_ ? "firmware" : "functional"));
    if (cpu_ != nullptr) {
        lines.push_back(format("  RL78 core  : pc=0x%05X insns=%llu cycles=%llu",
                               cpu_->get_pc(), static_cast<unsigned long long>(cpu_->instructions),
                               static_cast<unsigned long long>(cpu_->cycles)));
        cpu_->describe_state(lines);
    } else {
        lines.push_back("  RL78 core  : not created");
    }
    lines.push_back("  SC channel : " + impl.channel.regs().summary());
    lines.push_back(format("  last SC    : 0x%X answered with %u bytes, ready=%d",
                           impl.channel.regs().command, static_cast<unsigned>(response_.size()),
                           response_ready_ ? 1 : 0));
    ernie::describe_power_state(impl.power, lines);
    impl.emmc_host.describe(lines);
    if (impl.sfr != nullptr) impl.sfr->describe(lines);
}

std::vector<Device*> ErnieBlock::devices() const {
    std::vector<Device*> out;
    for (const auto& device : impl_->bus.devices()) {
        const std::string& name = device->name();
        if (name.rfind("Ernie", 0) == 0) out.push_back(device.get());
    }
    return out;
}

}  // namespace zlb


