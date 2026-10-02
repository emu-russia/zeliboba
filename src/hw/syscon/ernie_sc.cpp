// zeliboba - Ernie SC command channel.
//
// This file implements the protocol documented at the top of ernie_internal.h:
// the SC register file the SoC side polls, the two message windows the CMeP uses
// and the functional (C++) model of the command dispatcher.
#include <algorithm>
#include <cstdio>
#include <cstring>

#include "common/log.h"
#include "common/util.h"
#include "hw/syscon.h"
#include "hw/syscon/ernie_internal.h"

namespace zlb {
namespace ernie {

namespace {

/// The Ernie command table, dumped from USS-1001.bin at 0x26BE (70 entries,
/// eight bytes each: u16 number, u16 flags, u32 handler).  `flags` bit 0 marks a
/// command whose handler consumes a payload, which is why the CMeP only posts
/// descriptors with a control block for those.  The names are the functional
/// model's identification; the handler addresses are evidence.
///
/// The names are *not* evidence.  Several of them were inferred from how the
/// command numbers group, and three of those inferences are now contradicted by
/// the wire traffic of the 1.04 second loader and by the wiki:
///
///   * 0x1082 is an NVS read (`u16 offset` + `u8 length`), not an eMMC CSD read
///     -- the loader sends [80 04 08] and the decompiled 3.60 second loader
///     calls `syscon_read_cmd_0x1082_ptr_0x480_into_gbuf`;
///   * 0x1083 carries the same header followed by data (NVS write);
///   * 0x0090/0x0091 are the Syscon scratch pad, not the fuel gauge -- the
///     loader sends [E0 00 20], i.e. the `SceDIPSW dipsw` field;
///   * 0x1100 takes no payload at all (flags = 0x0000 in this very table) and
///     answers with the Ernie DL version, so it cannot be a block read.
///
/// The remaining members of the 0x1080/0x1101/0x1180..0x1185 group keep their
/// old labels, but nothing in the dumps or the wiki supports them: the eMMC is
/// wired to Kermit's SDIO0 controller and the second loader drives it directly,
/// so treat those names as unverified.
const ScCommandInfo kCommandTable[] = {
    {0x0000, 0x0000, 0x35C19, "get_status"},
    {0x0001, 0x0000, 0x35C41, "get_boot_info"},
    {0x0002, 0x0000, 0x35C78, "get_model_id"},
    {0x0003, 0x0001, 0x35CAF, "get_model_string"},
    {0x0004, 0x0000, 0x35D49, "get_serial"},
    {0x0005, 0x0000, 0x35E83, "get_boot_mode"},
    {0x0006, 0x0000, 0x35EBA, "get_uid"},
    {0x0010, 0x0000, 0x35EF1, "get_version"},
    {0x0011, 0x0001, 0x35F43, "get_version_ext"},
    {0x0012, 0x0001, 0x35FE6, "get_firmware_version"},
    {0x0013, 0x0000, 0x360A1, "get_last_result"},
    {0x0015, 0x0001, 0x360FE, "get_error_detail"},
    {0x0800, 0x0001, 0x37253, "get_battery"},
    {0x0806, 0x0000, 0x372DA, "get_charger_state"},
    {0x0820, 0x0000, 0x37319, "get_charge_current"},
    {0x0821, 0x0000, 0x37347, "get_charge_voltage"},
    {0x0822, 0x0000, 0x3737A, "get_charge_temp"},
    {0x0900, 0x0001, 0x373CF, "reset_device"},
    {0x0901, 0x0001, 0x37437, "suspend"},
    {0x0902, 0x0001, 0x3749F, "resume"},
    {0x1100, 0x0000, 0x35B57, "get_ernie_dl_version"},
    {0x1101, 0x0001, 0x35B8E, "storage_write"},
    {0x0100, 0x0001, 0x36132, "get_panel_state"},
    {0x0103, 0x0001, 0x361C5, "get_panel_state2"},
    {0x0130, 0x0001, 0x36266, "get_uart_state"},
    {0x0080, 0x0000, 0x36378, "get_rtc"},
    {0x0081, 0x0001, 0x36448, "set_rtc"},
    {0x0082, 0x0001, 0x364B8, "set_rtc_alarm"},
    {0x0083, 0x0000, 0x36689, "get_rtc_alarm"},
    {0x0084, 0x0000, 0x366D4, "get_rtc_alarm2"},
    {0x0085, 0x0000, 0x3670C, "get_rtc_alarm3"},
    {0x00C1, 0x0001, 0x368DB, "set_led"},
    {0x0090, 0x0001, 0x36540, "scratchpad_read"},
    {0x0091, 0x0001, 0x365E0, "scratchpad_write"},
    {0x00A0, 0x0000, 0x36744, "get_gauge_status"},
    {0x00B2, 0x0001, 0x367E4, "set_power_hold"},
    {0x00C0, 0x0000, 0x36881, "get_power_state"},
    {0x00D0, 0x0001, 0x36967, "set_port"},
    {0x00D1, 0x0001, 0x369F1, "get_port"},
    {0x00D2, 0x0001, 0x36A32, "set_port_mode"},
    {0x00D3, 0x0001, 0x36AA2, "get_port_mode"},
    {0x0190, 0x0001, 0x36B12, "get_adc"},
    {0x01B0, 0x0001, 0x36B95, "get_temp"},
    {0x0886, 0x0001, 0x37002, "charge_control"},
    {0x0888, 0x0001, 0x37024, "charge_set_limit"},
    {0x0889, 0x0001, 0x37088, "charge_set_current"},
    {0x0891, 0x0001, 0x36ED3, "gauge_read_word"},
    {0x088F, 0x0001, 0x36CE0, "gauge_write_word"},
    {0x088E, 0x0001, 0x36DD4, "gauge_command"},
    {0x089B, 0x0001, 0x370EC, "gauge_read_block"},
    {0x089C, 0x0001, 0x37150, "gauge_write_block"},
    {0x08C5, 0x0001, 0x373AD, "gauge_control"},
    {0x0982, 0x0001, 0x37507, "sleep_enter"},
    {0x098A, 0x0001, 0x3756B, "sleep_wake_reason"},
    {0x1080, 0x0001, 0x375CF, "emmc_init"},
    {0x1081, 0x0001, 0x375FC, "emmc_get_cid"},
    {0x1082, 0x0001, 0x3764D, "nvs_read"},
    {0x1083, 0x0001, 0x37722, "nvs_write"},
    {0x1180, 0x0001, 0x35749, "emmc_read"},
    {0x1181, 0x0001, 0x3578B, "emmc_write"},
    {0x1182, 0x0001, 0x35864, "emmc_erase"},
    {0x1183, 0x0001, 0x35A8C, "emmc_switch_partition"},
    {0x1184, 0x0001, 0x35961, "emmc_boot_select"},
    {0x1185, 0x0001, 0x35B36, "emmc_boot_read"},
    {0x2080, 0x0001, 0x377CA, "soc_release"},
    {0x2081, 0x0001, 0x37800, "soc_hold"},
    {0x2082, 0x0001, 0x37836, "soc_reset"},
    {0x2083, 0x0001, 0x3794B, "soc_power_control"},
    {0x2084, 0x0001, 0x37A0A, "soc_clock_control"},
    {0x2085, 0x0001, 0x37B04, "soc_status"},
};

constexpr size_t kCommandCount = sizeof(kCommandTable) / sizeof(kCommandTable[0]);

}  // namespace

const std::vector<ScCommandInfo>& sc_command_table() {
    static const std::vector<ScCommandInfo> table(kCommandTable, kCommandTable + kCommandCount);
    return table;
}

const ScCommandInfo* sc_command_info(u32 number) {
    for (const ScCommandInfo& info : kCommandTable) {
        if (info.number == number) return &info;
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// ScRegs
// ---------------------------------------------------------------------------

void ScRegs::reset() {
    cmd10A0 = 0;
    ack10A4 = 0;
    resp20A0 = 0;
    resp20A4 = 0;
    reg1190 = 0;
    stat24 = 0;
    stat36 = 0;
    req122 = 0;
    irq124 = 0;
    reg1100 = 0;
    reg2100 = 0;
    reg3040 = 0;
    reg3050 = 0;
    reg0194 = 0;
    engine30 = 0;
    engine32 = 0;
    command = 0;
    error = 0;
    status_byte = 0;
    payload_length = 0;
    commands = 0;
    request_polls = 0;
    ack_polls = 0;
}

std::string ScRegs::summary() const {
    return format("SC cmd=0x%X served=%llu stat24=0x%X irq124=0x%X req1190=%u", command,
                  static_cast<unsigned long long>(commands), stat24, irq124, reg1190);
}

void ScRegs::save_state(StateWriter& writer) const {
    writer.put_u32(cmd10A0);
    writer.put_u32(ack10A4);
    writer.put_u32(resp20A0);
    writer.put_u32(resp20A4);
    writer.put_u32(reg1190);
    writer.put_u32(stat24);
    writer.put_u32(stat36);
    writer.put_u32(req122);
    writer.put_u32(irq124);
    writer.put_u32(reg1100);
    writer.put_u32(reg2100);
    writer.put_u32(reg3040);
    writer.put_u32(reg3050);
    writer.put_u32(reg0194);
    writer.put_u32(engine30);
    writer.put_u32(engine32);
    writer.put_u32(stat1000);
    writer.put_u32(command);
    writer.put_u32(error);
    writer.put_u8(status_byte);
    writer.put_u8(payload_length);
    // The response is a runtime buffer (never longer than the 32 byte record),
    // so a length prefix is the right shape; the read cursor follows it.
    writer.list(response, [&](u8 byte) { writer.put_u8(byte); });
    writer.put_u64(static_cast<u64>(response_byte));
    writer.put_u64(commands);
    writer.put_u64(request_polls);
    writer.put_u64(ack_polls);
}

void ScRegs::load_state(StateReader& reader) {
    cmd10A0 = reader.get_u32();
    ack10A4 = reader.get_u32();
    resp20A0 = reader.get_u32();
    resp20A4 = reader.get_u32();
    reg1190 = reader.get_u32();
    stat24 = reader.get_u32();
    stat36 = reader.get_u32();
    req122 = reader.get_u32();
    irq124 = reader.get_u32();
    reg1100 = reader.get_u32();
    reg2100 = reader.get_u32();
    reg3040 = reader.get_u32();
    reg3050 = reader.get_u32();
    reg0194 = reader.get_u32();
    engine30 = reader.get_u32();
    engine32 = reader.get_u32();
    stat1000 = reader.get_u32();
    command = reader.get_u32();
    error = reader.get_u32();
    status_byte = reader.get_u8();
    payload_length = reader.get_u8();
    reader.list(response, [&](u8& byte) { byte = reader.get_u8(); });
    response_byte = static_cast<size_t>(reader.get_u64());
    commands = reader.get_u64();
    request_polls = reader.get_u64();
    ack_polls = reader.get_u64();
}

// ---------------------------------------------------------------------------
// ScWindowDevice
// ---------------------------------------------------------------------------

ScWindowDevice::ScWindowDevice() : Device("Ernie.SC", kScWindowBase, kScWindowSize) {
    register_name_entry(kScStat24, "SC_STATUS");
    register_name_entry(kScIfType, "SC_IF_TYPE");
    register_name_entry(kScStat1000, "SC_IF_STATUS");
    register_name_entry(kScEngineCmd, "SC_DATA0");
    register_name_entry(kScEngineAux, "SC_DATA1");
    register_name_entry(kScStat36, "SC_STATUS36");
    register_name_entry(kScReq122, "SC_REQ122");
    register_name_entry(kScIrq124, "SC_ENABLE124");
    register_name_entry(kScReg0194, "SC_REG0194");
    register_name_entry(kScCmd10A0, "SC_CMD10A0");
    register_name_entry(kScAck10A4, "SC_ACK10A4");
    register_name_entry(kScReg1100, "SC_REG1100");
    register_name_entry(kScReply1190, "SC_REPLY1190");
    register_name_entry(kScResp20A0, "SC_RESP20A0");
    register_name_entry(kScResp20A4, "SC_RESP20A4");
    register_name_entry(kScReg2100, "SC_REG2100");
    register_name_entry(kScReg3040, "SC_REG3040");
    register_name_entry(kScReg3050, "SC_REG3050");
}

ScRegs& ScWindowDevice::regs() { return channel_ ? channel_->regs() : fallback_; }
const ScRegs& ScWindowDevice::regs() const { return channel_ ? channel_->regs() : fallback_; }

u64 ScWindowDevice::read(u32 address, unsigned size) {
    u32 value = 0;
    switch (address) {
        case kScStat24:
            value = regs().stat24;
            ++regs().request_polls;
            break;
        case kScEngineCmd:
        case kScEngineAux:
            // sc_read reads its eight reply bytes out of control+0x30..0x37
            // (0x5C302..0x5C338) and then waits for both ports to read back zero
            // (loc_5D088..loc_5D08E).  Serving the reply byte by byte and then
            // zero makes both loops terminate exactly as the hardware does.
            value = channel_ ? static_cast<u32>(channel_->read_data_port()) : 0;
            ++regs().ack_polls;
            break;
        case kScStat36: value = regs().stat36; break;
        case kScIfType: value = kScIfTypeValue; break;
        case kScStat1000: value = regs().stat1000; break;
        case kScReq122: value = regs().req122; break;
        case kScIrq124: value = regs().irq124; break;
        case kScReg0194: value = regs().reg0194; break;
        case kScEventsC0: {
            // Hand out the currently pending events, then re-arm: the syscon
            // raises the next event as soon as the guest has consumed this one.
            value = events_c0_;
            events_c0_ = 0x18u;
            break;
        }
        case kScCmd10A0: value = regs().cmd10A0; break;
        case kScAck10A4: value = regs().ack10A4; break;
        case kScReg1100: value = regs().reg1100; break;
        case kScReply1190: value = regs().reg1190; break;
        case kScResp20A0: value = regs().resp20A0; break;
        case kScResp20A4: value = regs().resp20A4; break;
        case kScReg2100: value = regs().reg2100; break;
        case kScReg3040: value = regs().reg3040; break;
        case kScReg3050: value = regs().reg3050; break;
        default:
            // Unknown SC registers behave like storage: the CMeP uses several of
            // them as write-then-poll strobes (0xE3101120 / 0xE3102120), so a
            // register that was written reads back its value.
            {
                const auto it = scratch_.find(address);
                value = it == scratch_.end() ? 0 : it->second;
            }
            break;
    }
    if (size >= 4) return value;
    return value & ((1u << (size * 8)) - 1u);
}

void ScWindowDevice::write(u32 address, unsigned size, u64 raw) {
    const u32 value = static_cast<u32>(raw);
    switch (address) {
        case kScCmd10A0:
            regs().cmd10A0 = value;
            if (channel_) channel_->on_command_word_written(value);
            break;
        case kScAck10A4:
            regs().ack10A4 = value;
            break;
        case kScResp20A0:
            regs().resp20A0 = value;
            if (channel_) channel_->on_command_word_written(value);
            break;
        case kScResp20A4: regs().resp20A4 = value; break;
        case kScReg1100: regs().reg1100 = value; break;
        case kScReply1190:
            regs().reg1190 = value;
            if (value == 0 && channel_) channel_->on_reply_cleared();
            break;
        case kScReg2100: regs().reg2100 = value; break;
        case kScReg3040: regs().reg3040 = value; break;
        case kScReg3050: regs().reg3050 = value; break;
        case kScIrq124: regs().irq124 = value; break;
        case kScReq122: regs().req122 = value; break;
        case kScStat24: regs().stat24 = value; break;
        case kScStat36: regs().stat36 = value; break;
        case kScStat1000: regs().stat1000 = value; break;
        case kScReg0194: regs().reg0194 = value; break;
        case kScEventsC0:
            // Write-1-to-clear acknowledgement.
            events_c0_ &= ~value;
            break;
        case kScReq5C:
        case kScReq60:
            // Service-request latches: KBL writes 1 to 0xE310005C and then to
            // 0xE3100060, polling each until it reads back 0
            // (0x4003C014..0x4003C05E).  The syscon clears them once the request
            // has been taken; the model acknowledges immediately (the registers
            // are not stored, so reads return 0) - the same substitution the
            // request registers 0xE31010A0/0xE31020A0 already use for the CMeP.
            break;
        case kScEngineCmd: regs().engine30 = value; break;
        case kScEngineAux: regs().engine32 = value; break;
        default: scratch_[address] = value; break;
    }
}

void ScWindowDevice::reset() {
    regs().reset();
    scratch_.clear();
}

const char* ScWindowDevice::register_name(u32 address) const {
    const char* name = lookup_name(address);
    return name ? name : Device::register_name(address);
}

void ScWindowDevice::enumerate_registers(std::vector<RegisterInfo>& out) const {
    for (const auto& entry : names_) {
        RegisterInfo info;
        info.address = entry.first;
        info.name = entry.second;
        info.width = 4;
        out.push_back(info);
    }
}

std::string ScWindowDevice::summary() const { return regs().summary(); }

void ScWindowDevice::describe(std::vector<std::string>& lines) const {
    lines.push_back("Ernie SC bridge (0xE3100000)");
    lines.push_back(format("  command         : 0x%X (%s)", regs().command,
                           sc_command_info(regs().command) ? sc_command_info(regs().command)->name
                                                          : "unknown"));
    lines.push_back(format("  status          : 0x%X (busy=%d ready=%d error=%d)", regs().stat24,
                           (regs().stat24 & 1) ? 1 : 0, (regs().stat24 & 2) ? 1 : 0,
                           (regs().stat24 & 4) ? 1 : 0));
    lines.push_back(format("  enable (0x124)  : 0x%X, reply request (0x1190) = %u", regs().irq124,
                           regs().reg1190));
    lines.push_back(format("  posted words    : cmd10A0=0x%X ack10A4=0x%X resp20A0=0x%X resp20A4=0x%X",
                           regs().cmd10A0, regs().ack10A4, regs().resp20A0, regs().resp20A4));
    lines.push_back(format("  served          : %llu commands, status byte 0x%02X, payload %u",
                           static_cast<unsigned long long>(regs().commands), regs().status_byte,
                           static_cast<unsigned>(regs().payload_length)));
}

void ScWindowDevice::save_state(StateWriter& writer) const {
    // Only the standalone (unbound) register file lives here; the bound channel
    // is serialised by ErnieBlock, which owns it.
    writer.begin("fallback");
    fallback_.save_state(writer);
    writer.end();
    writer.map(scratch_, [&](u32 address, u32 value) {
        writer.put_u32(address);
        writer.put_u32(value);
    });
    writer.put_u32(events_c0_);
}

void ScWindowDevice::load_state(StateReader& reader) {
    reader.begin("fallback");
    fallback_.load_state(reader);
    reader.end();
    reader.map(scratch_, [&](u32& address, u32& value) {
        address = reader.get_u32();
        value = reader.get_u32();
    });
    events_c0_ = reader.get_u32();
}

// ---------------------------------------------------------------------------
// SocGateDevice
// ---------------------------------------------------------------------------

SocGateDevice::SocGateDevice() : Device("Ernie.SocGate", kBase, kSize) {
    register_name_entry(kBootState, "SOC_BOOT_STATE");
    register_name_entry(kRelease, "SOC_RELEASE");
}

u64 SocGateDevice::read(u32 address, unsigned size) {
    u32 value = 0;
    if (address == kBootState) value = boot_state_;
    else if (address == kRelease) value = release_;
    if (size >= 4) return value;
    return value & ((1u << (size * 8)) - 1u);
}

void SocGateDevice::write(u32 address, unsigned size, u64 value) {
    if (address == kBootState) {
        boot_state_ = static_cast<u32>(value);
    } else if (address == kRelease) {
        release_ = static_cast<u32>(value);
        // 0x5C2A6..0x5C2B4: the CMeP stores 0x8001 here once bit 1 of
        // 0x30000118 is set.  That is the "ARM may start" release.
        if ((release_ & 0x8001u) == 0x8001u) released_ = true;
    }
}

void SocGateDevice::reset() {
    boot_state_ = 0x00000002;
    release_ = 0;
    released_ = false;
}

const char* SocGateDevice::register_name(u32 address) const {
    const char* name = lookup_name(address);
    return name ? name : Device::register_name(address);
}

std::string SocGateDevice::summary() const {
    return format("boot state 0x%08X, release 0x%08X (%s)", boot_state_, release_,
                  released_ ? "SoC released" : "SoC held");
}

void SocGateDevice::save_state(StateWriter& writer) const {
    writer.put_u32(boot_state_);
    writer.put_u32(release_);
    writer.put_bool(released_);
}

void SocGateDevice::load_state(StateReader& reader) {
    boot_state_ = reader.get_u32();
    release_ = reader.get_u32();
    released_ = reader.get_bool();
}

// ---------------------------------------------------------------------------
// ScStrapDevice
// ---------------------------------------------------------------------------

ScStrapDevice::ScStrapDevice() : Device("Ernie.ScStrap", kBase, kSize) {
    register_name_entry(kBase, "SC_STATE_STRAP");
}

u64 ScStrapDevice::read(u32 address, unsigned size) {
    (void)address;
    if (size >= 4) return value_;
    return value_ & ((1u << (size * 8)) - 1u);
}

void ScStrapDevice::write(u32 address, unsigned size, u64 value) {
    (void)address;
    (void)size;
    value_ = static_cast<u32>(value);
}

void ScStrapDevice::reset() { value_ = 0x00010000; }

const char* ScStrapDevice::register_name(u32 address) const {
    const char* name = lookup_name(address);
    return name ? name : Device::register_name(address);
}

std::string ScStrapDevice::summary() const {
    return format("state strap 0x%08X (JIG=%d)", value_, (value_ & 0x10000u) ? 0 : 1);
}

void ScStrapDevice::save_state(StateWriter& writer) const { writer.put_u32(value_); }
void ScStrapDevice::load_state(StateReader& reader) { value_ = reader.get_u32(); }

// ---------------------------------------------------------------------------
// ScMessageWindow
// ---------------------------------------------------------------------------

ScMessageWindow::ScMessageWindow(u32 base, u32 size, bool response)
    : Device(response ? "Ernie.ScReply" : "Ernie.ScCmd", base, size),
      bytes_(size, 0),
      response_(response) {}

u64 ScMessageWindow::read(u32 address, unsigned size) {
    const u32 offset = address - base_;
    u64 value = 0;
    for (unsigned i = 0; i < size; ++i) {
        const u32 index = offset + i;
        const u8 byte = index < bytes_.size() ? bytes_[index] : 0;
        value |= static_cast<u64>(byte) << (8 * i);
    }
    return value;
}

void ScMessageWindow::write(u32 address, unsigned size, u64 value) {
    const u32 offset = address - base_;
    for (unsigned i = 0; i < size; ++i) {
        const u32 index = offset + i;
        if (index < bytes_.size()) bytes_[index] = static_cast<u8>((value >> (8 * i)) & 0xFF);
    }
    if (!response_) dirty_ = true;
    // Round 245: byte 0x2F bit 0 is the command window's start bit.  On hardware the
    // block clears it once the descriptor has been handed to the syscon, and the poster
    // polls exactly that bit (NSKBL: byte write 1 then a poll at pc 0x5101D788..0x5101D790,
    // after reaching stage 0xA9).  Dispatch the descriptor and clear the bit so the poll
    // completes, instead of leaving it set forever.
    if (!response_ && bytes_.size() > 0x2Fu && (bytes_[0x2F] & 0x01u) != 0u) {
        bytes_[0x2F] = static_cast<u8>(bytes_[0x2F] & ~0x01u);
        if (command_posted_) command_posted_(bytes_.data(), bytes_.size());
    }
    // Round 248: only a write *to the two data ports* means "the poster consumed the
    // reply" - the earlier version checked the ports after every write, so the trigger
    // write itself (offset 0x2F, with an empty reply and therefore zero ports) cleared
    // the "reply available" bit in the same call that had just set it, and the guest
    // never saw bit 17 (measured: pc 0x5101D7C6 is reached zero times).
    const bool touches_data_ports = offset <= 0x33u && offset + size > 0x30u;
    if (!response_ && bytes_.size() >= 0x34u && touches_data_ports) {
        const u16 data0 = static_cast<u16>(bytes_[0x30] | (bytes_[0x31] << 8));
        const u16 data1 = static_cast<u16>(bytes_[0x32] | (bytes_[0x33] << 8));
        if (data0 == 0u && data1 == 0u) bytes_[0x26] = static_cast<u8>(bytes_[0x26] & ~0x02u);
    }
}

void ScMessageWindow::reset() {
    std::fill(bytes_.begin(), bytes_.end(), 0);
    dirty_ = false;
}

const char* ScMessageWindow::register_name(u32 address) const {
    const u32 offset = address - base_;
    switch (offset) {
        case 0x00: return "SC_DESC_MODE";
        case 0x04: return "SC_DESC_TIMEOUT";
        case 0x08: return "SC_DESC_TIMEOUT2";
        case 0x0E: return "SC_DESC_CHANNEL";
        case 0x18: return "SC_DESC_WINDOW";
        case 0x24: return "SC_DESC_CONTROL";
        default: return "SC_DESC_BYTES";
    }
}

std::string ScMessageWindow::summary() const {
    return format("%s: %u bytes%s, mode=0x%02X channel=%u command=0x%X",
                  response_ ? "reply" : "command", static_cast<unsigned>(bytes_.size()),
                  dirty_ ? " (dirty)" : "", bytes_.empty() ? 0 : bytes_[0],
                  bytes_.size() > 0x0E ? bytes_[0x0E] : 0, load_le32(bytes_, 0x24));
}

void ScMessageWindow::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s window @0x%08X, %zu bytes", response_ ? "reply" : "command", base_,
                           bytes_.size()));
    for (size_t i = 0; i < bytes_.size() && i < 0x30; i += 16) {
        std::string row = format("    +%02X:", static_cast<unsigned>(i));
        for (size_t j = 0; j < 16 && i + j < bytes_.size(); ++j) row += format(" %02X", bytes_[i + j]);
        lines.push_back(row);
    }
}

void ScMessageWindow::load(const std::vector<u8>& data) {
    std::fill(bytes_.begin(), bytes_.end(), 0);
    const size_t count = std::min(data.size(), bytes_.size());
    std::memcpy(bytes_.data(), data.data(), count);
    dirty_ = false;
}

void ScMessageWindow::save_state(StateWriter& writer) const {
    // The window size is a construction-time constant, so the byte image is
    // written raw (no length): load_state must not resize it.
    writer.bytes(bytes_.data(), bytes_.size());
    writer.put_bool(response_);
    writer.put_bool(dirty_);
}

void ScMessageWindow::load_state(StateReader& reader) {
    reader.bytes(bytes_.data(), bytes_.size());
    response_ = reader.get_bool();
    dirty_ = reader.get_bool();
}

void ScMessageWindow::store(u32 offset, const u8* data, size_t length) {
    if (offset >= bytes_.size()) return;
    const size_t count = std::min(length, bytes_.size() - offset);
    std::memcpy(bytes_.data() + offset, data, count);
}

// ---------------------------------------------------------------------------
// ScChannel
// ---------------------------------------------------------------------------

ScChannel::ScChannel(ErnieBlock& owner) : owner_(owner) {}

void ScChannel::reset() { regs_.reset(); }

void ScChannel::post_descriptor(const u8* descriptor, size_t length, u32 window_base) {
    if (descriptor == nullptr || length == 0) return;
    const u8 mode = descriptor[0];
    const u8 channel = length > 0x0E ? descriptor[0x0E] : 0;
    u32 command = (static_cast<u32>(mode) << 8) | channel;
    // sc_read also keeps the command word at the head of the selected window, so
    // accept a descriptor that carries an explicit command word at +0x24 when
    // the mode/channel pair looks empty.
    if (command == 0 && length >= 0x28) command = load_le32(descriptor + 0x24);

    std::vector<u8> payload;
    const size_t payload_offset = 0x100;
    if (length > payload_offset) {
        payload.assign(descriptor + payload_offset, descriptor + length);
    }
    (void)window_base;

    regs_.command = command;
    const std::vector<u8> reply = owner_.dispatch_command(command, payload);
    publish_reply(command, reply);
}

void ScChannel::on_command_word_written(u32 value) {
    if (value == 0) return;
    regs_.command = value;
    // A bare command word (no descriptor) is answered with an empty payload,
    // which is what the CMeP's program-and-poll path expects: it only wants the
    // bridge to acknowledge.  ErnieBlock::dispatch_command already publishes the
    // reply into this channel, so nothing else is needed here.
    (void)owner_.dispatch_command(value, {});
}

void ScChannel::on_reply_cleared() {
    regs_.stat24 &= ~0x02u;
    regs_.response.clear();
    regs_.response_byte = 0;
}

u64 ScChannel::read_data_port() {
    if ((regs_.stat24 & 0x02u) == 0) return 0;
    if (regs_.response_byte >= regs_.response.size()) return 0;
    return regs_.response[regs_.response_byte++];
}

void ScChannel::set_reply_request(u32 value) {
    regs_.reg1190 = value;
    if (value == 0) on_reply_cleared();
}

void ScChannel::publish_reply(u32 command, const std::vector<u8>& reply) {
    regs_.command = command;
    regs_.commands++;
    regs_.response = reply;
    regs_.response_byte = 0;
    regs_.stat24 |= 0x02u;  // reply ready (sc_read polls bit 1)
    regs_.reg1190 = 1;      // reply request asserted for the SoC side
    // The two data ports carry the first two bytes of the response record; a
    // non-zero value means "more data in the window" (loc_5D088).
    regs_.engine30 = reply.size() > 0 ? reply[0] : 0;
    regs_.engine32 = reply.size() > 1 ? reply[1] : 0;
    if (reply.size() >= 3) {
        regs_.status_byte = reply[1];
        regs_.payload_length = reply[2];
    }
}

// ---------------------------------------------------------------------------
// Scratch pad
// ---------------------------------------------------------------------------

const std::array<u8, ScratchPad::kDipSwitchSize>& ScratchPad::retail_dip_switches() {
    // 0x00..0x0F  CP DIP switches: unset on Retail/TestKit (wiki "Syscon Scratch
    //             Pad": the field is only set on a DevKit that has a CP).
    // 0x10..0x1F  the release mode values the wiki lists for SceKblParam +0x50..
    //             (+0x54 SDK, +0x58 shell, +0x5C debug, +0x60 system).
    static const std::array<u8, kDipSwitchSize> block = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,  // SDK flags       = 0x00000000
        0x00, 0x00, 0x00, 0x00,  // shell flags     = 0x00000000
        0x02, 0x00, 0x08, 0x00,  // debug control   = 0x00080002
        0x00, 0x00, 0x00, 0x20,  // system control  = 0x20000000
    };
    return block;
}

void ScratchPad::reset() {
    bytes_.fill(0);
    std::copy(retail_dip_switches().begin(), retail_dip_switches().end(),
              bytes_.begin() + kDipSwitchOffset);
}

}  // namespace ernie
}  // namespace zlb









