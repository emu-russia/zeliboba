// Genuine firmware 1.04 SceDriverTzs / SceEmcTop initialization subset.
//
// The native driver maps E8200000/1000, registers SMC117/118/119, and submits
// payloads at +28 with launch/status at +24. SMC117's cold path uses the exact
// payload/launch pairs below; it preserves bits30 and polls only busy bit0.
// SMC118 writes10 and waits for hardware acknowledgement20; SMC119 writes0.
// The apparent DDR command fields and electrical behavior are not recovered.
// See build/goal-native-e820-emctop-evidence.md for actual code/capture evidence,
// and https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/cdram.c
// for independent hardware-tested use of SMC117.
//
// Completion after one PERIPHCLK tick is an emulator choice, not a measured
// physical latency. Configuration words are opaque snapshots. IRQ34 belongs
// to the separate +230/+234/+240/+244 calibration path; no event is generated
// until its source contract is known. This device neither patches services nor
// performs a DRAM memory test, electrical calibration or address-space gating.
#include "hw/soc/soc_internal.h"

namespace zlb::kermit {
namespace {

constexpr u32 kControl = 0x24;
constexpr u32 kPayload = 0x28;
constexpr u32 kCalibrationStatus = 0x240;
constexpr u32 kCalibrationAck = 0x244;
constexpr u32 kBusy = 1;
constexpr u32 kModifier = 2;
constexpr u32 kModeRequest = 0x10;
constexpr u32 kModeAcknowledged = 0x20;
constexpr u32 kKnownControlBits = kBusy | kModifier | kModeRequest | kModeAcknowledged;

struct NamedRegister { u32 offset; const char* name; };
constexpr NamedRegister kRegisters[] = {
    {0x00, "REG_000"}, {0x04, "REG_004"}, {0x08, "REG_008"},
    {0x0C, "REG_00C"}, {0x10, "REG_010"}, {0x14, "REG_014"},
    {0x18, "REG_018"}, {0x1C, "REG_01C"}, {kControl, "CONTROL_STATUS"},
    {kPayload, "COMMAND_PAYLOAD"}, {0x2C, "REG_02C"},
    {0x38, "REG_038"}, {0x3C, "REG_03C"},
    {0x230, "CAL_CONTROL"}, {0x234, "CAL_CONFIG"},
    {kCalibrationStatus, "CAL_STATUS"}, {kCalibrationAck, "CAL_ACK"},
};

bool known_register(u32 offset) {
    for (const auto& reg : kRegisters) if (reg.offset == offset) return true;
    return false;
}

bool supported_command(u32 payload, u32 launch) {
    if (launch == 1) {
        return payload == 0x000E0000 || payload == 0x00040400 ||
               payload == 0x00020000 || payload == 0x00000031;
    }
    return launch == 3 && payload == 0x00200000;
}

}  // namespace

EmcTopController::EmcTopController() : Device("Kermit.EmcTop", kEmcTopBase, kEmcTopSize) {
    for (const auto& reg : kRegisters) register_name_entry(base_ + reg.offset, reg.name);
}

u32 EmcTopController::control_status() const {
    return (busy_ ? kBusy : 0) | modifier_ |
           (mode_request_ ? kModeRequest : 0) |
           (mode_acknowledged_ ? kModeAcknowledged : 0);
}

u64 EmcTopController::read(u32 address, unsigned size) {
    u64 result = 0;
    for (unsigned i = 0; i < size && i < 8; ++i) {
        const u64 current = static_cast<u64>(address) + i;
        u32 byte = 0xFF;
        if (current >= base_ && current < static_cast<u64>(base_) + size_) {
            const u32 offset = static_cast<u32>(current - base_);
            const u32 word = offset & ~3u;
            if (known_register(word)) {
                const u32 stored = word == kControl ? control_status() : registers_[word / 4];
                byte = (stored >> ((offset & 3) * 8)) & 0xFFu;
            }
        }
        result |= static_cast<u64>(byte) << (i * 8);
    }
    return result;
}

void EmcTopController::write(u32 address, unsigned size, u64 value) {
    bool control_written = false;
    for (unsigned i = 0; i < size && i < 8; ++i) {
        const u64 current = static_cast<u64>(address) + i;
        if (current < base_ || current >= static_cast<u64>(base_) + size_) continue;
        const u32 offset = static_cast<u32>(current - base_);
        const u32 word = offset & ~3u;
        if (!known_register(word) || word == kCalibrationStatus) continue;
        const u32 shift = (offset & 3) * 8;
        const u32 supplied = static_cast<u32>((value >> (i * 8)) & 0xFFu) << shift;
        if (word == kCalibrationAck) {
            // Acknowledge only supplied bits, never inject calibration status.
            registers_[kCalibrationStatus / 4] &= ~supplied;
        } else {
            registers_[word / 4] = (registers_[word / 4] & ~(0xFFu << shift)) | supplied;
        }
        control_written |= offset == kControl;
    }
    // Only an access containing the low control lane submits/cancels. Merge
    // all lanes first so unknown word flags are rejected atomically. Upper
    // control lanes stage input for a later low-lane access; they never change
    // hardware busy/mode status or create a completion themselves.
    if (control_written) submit_control();
}

void EmcTopController::reject_control() {
    busy_ = true;
    command_pending_ = mode_pending_ = false;
    unsupported_ = true;
    ++rejected_count_;
    ZLB_LOG_WARN("emc", "unsupported EMC payload=%08X control=%08X; busy until control0",
                 last_payload_, last_control_);
}

void EmcTopController::submit_control() {
    const u32 control = registers_[kControl / 4];
    if (control == 0) {
        // The native cold sequence writes30 then0; SMC119 writes0 to release
        // the acknowledged mode. Also cancel an unsupported/pending command.
        busy_ = command_pending_ = mode_request_ = mode_acknowledged_ = mode_pending_ = unsupported_ = false;
        modifier_ = 0;
        ++control_resets_;
        return;
    }

    last_payload_ = registers_[kPayload / 4];
    last_control_ = control;
    if ((control & ~kKnownControlBits) != 0 || unsupported_ || command_pending_) {
        reject_control();
        return;
    }

    const u32 launch = control & (kBusy | kModifier);
    if (launch != 0) {
        if (!supported_command(last_payload_, launch)) {
            reject_control();
            return;
        }
        modifier_ = launch & kModifier;
        busy_ = command_pending_ = true;
        ++command_count_;
        // Mask0x20 (bit5) is a read-only acknowledgement. An echoed20 cannot activate
        // mode by itself. The explicit request10 may accompany a command.
        if ((control & kModeRequest) != 0) {
            mode_request_ = true;
            mode_pending_ = !mode_acknowledged_;
        }
        ZLB_LOG_TRACE("emc", "EMC command=%08X launch=%X pending (one PERIPHCLK tick)",
                      last_payload_, launch);
        return;
    }

    if (control == kModeRequest || control == (kModeRequest | kModeAcknowledged)) {
        mode_request_ = true;
        mode_pending_ = !mode_acknowledged_;
        return;
    }
    // Standalone guest acknowledgement20 or other unobserved control values
    // cannot manufacture mode state or silently report a successful operation.
    reject_control();
}

void EmcTopController::tick(u64 peripheral_ticks) {
    if (peripheral_ticks == 0 || unsupported_) return;
    if (mode_pending_) {
        mode_pending_ = false;
        mode_acknowledged_ = mode_request_;
    }
    if (command_pending_) {
        command_pending_ = false;
        busy_ = false;
        ++completion_count_;
    }
}

void EmcTopController::reset() {
    registers_.fill(0);
    modifier_ = last_payload_ = last_control_ = 0;
    command_count_ = completion_count_ = rejected_count_ = control_resets_ = 0;
    busy_ = command_pending_ = mode_request_ = mode_acknowledged_ = mode_pending_ = unsupported_ = false;
}

const char* EmcTopController::register_name(u32 address) const {
    return handles(address) ? lookup_name(address & ~3u) : nullptr;
}

void EmcTopController::enumerate_registers(std::vector<RegisterInfo>& out) const {
    for (const auto& reg : kRegisters) out.push_back({base_ + reg.offset, reg.name, 0, 4});
}

bool EmcTopController::peek_register(const std::string& name, u64& out) const {
    for (const auto& reg : kRegisters) {
        if (name != reg.name) continue;
        out = reg.offset == kControl ? control_status() : registers_[reg.offset / 4];
        return true;
    }
    return false;
}

bool EmcTopController::poke_register(const std::string& name, u64 value) {
    for (const auto& reg : kRegisters) {
        if (name != reg.name) continue;
        if (reg.offset == kCalibrationStatus) return false;
        write(base_ + reg.offset, 4, value);
        return true;
    }
    return false;
}

std::string EmcTopController::summary() const {
    return format("EmcTop: %s status=%08X last-command=%08X last-control=%08X commands=%llu completed=%llu rejected=%llu resets=%llu IRQ%u=low",
                  unsupported_ ? "unsupported" : busy_ ? "command pending" : mode_pending_ ? "mode pending" : "idle",
                  control_status(), last_payload_, last_control_,
                  static_cast<unsigned long long>(command_count_), static_cast<unsigned long long>(completion_count_),
                  static_cast<unsigned long long>(rejected_count_), static_cast<unsigned long long>(control_resets_),
                  kIrqEmcCalibration);
}

void EmcTopController::describe(std::vector<std::string>& lines) const {
    lines.push_back(summary());
    lines.push_back("  exact native initialization commands; one PERIPHCLK tick completion (emulator latency)");
    lines.push_back("  mode request10/ack20/release0; unsupported operations stay busy until control0");
    lines.push_back("  opaque config/calibration snapshots; no calibration event or electrical DRAM behavior");
}

void EmcTopController::save_state(StateWriter& writer) const {
    // The whole register array plus the last command snapshot, the mode/command
    // latches and every counter.
    writer.fixed(registers_, [&](u32 value) { writer.put_u32(value); });
    writer.put_u32(modifier_);
    writer.put_u32(last_payload_);
    writer.put_u32(last_control_);
    writer.put_u64(command_count_);
    writer.put_u64(completion_count_);
    writer.put_u64(rejected_count_);
    writer.put_u64(control_resets_);
    writer.put_bool(busy_);
    writer.put_bool(command_pending_);
    writer.put_bool(mode_request_);
    writer.put_bool(mode_acknowledged_);
    writer.put_bool(mode_pending_);
    writer.put_bool(unsupported_);
}

void EmcTopController::load_state(StateReader& reader) {
    reader.fixed(registers_, [&](u32& value) { value = reader.get_u32(); });
    modifier_ = reader.get_u32();
    last_payload_ = reader.get_u32();
    last_control_ = reader.get_u32();
    command_count_ = reader.get_u64();
    completion_count_ = reader.get_u64();
    rejected_count_ = reader.get_u64();
    control_resets_ = reader.get_u64();
    busy_ = reader.get_bool();
    command_pending_ = reader.get_bool();
    mode_request_ = reader.get_bool();
    mode_acknowledged_ = reader.get_bool();
    mode_pending_ = reader.get_bool();
    unsupported_ = reader.get_bool();
}

}  // namespace zlb::kermit
