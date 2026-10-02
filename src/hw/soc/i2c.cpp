// Native firmware 1.04 Lowio I2C idle/reset subset.
//
// Lowio linked 0x81003024 maps E0500000/E0510000 with 0x1000 apertures and
// IRQ142/143. At 0x810031C4 it writes +2C=0100F70F, +08/+0C=1, +14=7,
// polls +1C until zero, acknowledges +28 by echoing it, then writes
// +2C=01000000 and +18=5. Its IRQ handler 0x81002F18 also echoes +28 (W1C).
//
// Independent primary references reproduce this register protocol:
// https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/i2c.c
// https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/drivers/i2c/busses/i2c-vita.c
// https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/arch/arm/boot/dts/vita.dtsi
//
// +1C is Busy, +00/+04 are TX/RX FIFO, +10 is the 7-bit slave address,
// +14 encodes command and length, +18 selects speed, +28 is IRQ status,
// and +2C is IRQ control. +08/+0C and individual IRQ-control masks remain
// unknown. Native commands are write2, stop4, repeated-start5, reset7,
// read13, with length in bits15:8. Only reset7 is implemented here.
//
// Reset finishes synchronously (timing is not recovered). Other commands
// remain explicitly unsupported/busy until reset, and FIFO reads are
// unavailable. No slave presence, NACK/ACK, data or completion IRQ is invented.
// The hardware-tested driver's NACK bit15 is known, but an interruptible
// transaction model needs the IRQ-control masks and actual slave contracts.
#include "hw/soc/soc_internal.h"

namespace zlb::kermit {
namespace {

constexpr u32 kTxFifo = 0x00;
constexpr u32 kRxFifo = 0x04;
constexpr u32 kAddress = 0x10;
constexpr u32 kCommand = 0x14;
constexpr u32 kBusy = 0x1C;
constexpr u32 kIrqStatus = 0x28;
constexpr u32 kIrqControl = 0x2C;

struct NamedRegister { u32 offset; const char* name; };
constexpr NamedRegister kRegisters[] = {
    {kTxFifo, "TX_FIFO"}, {kRxFifo, "RX_FIFO"}, {0x08, "REG_008"},
    {0x0C, "REG_00C"}, {kAddress, "SLAVE_ADDRESS"}, {kCommand, "COMMAND"},
    {0x18, "SPEED"}, {kBusy, "BUSY"}, {kIrqStatus, "IRQ_STATUS"},
    {kIrqControl, "IRQ_CONTROL"},
};

bool known_register(u32 offset) {
    for (const auto& reg : kRegisters) if (reg.offset == offset) return true;
    return false;
}

u32 reset_value(u32 offset) {
    // All-ones FIFO reads explicitly represent unavailable data, rather than
    // a synthetic zero byte that could be mistaken for a slave response.
    return offset == kTxFifo || offset == kRxFifo ? 0xFFFFFFFFu : 0;
}

}  // namespace

I2cController::I2cController(std::string name, u32 base, unsigned port)
    : Device(std::move(name), base, kI2cSize), port_(port) {
    for (const auto& reg : kRegisters) register_name_entry(base_ + reg.offset, reg.name);
}

u64 I2cController::read(u32 address, unsigned size) {
    u64 result = 0;
    for (unsigned i = 0; i < size && i < 8; ++i) {
        const u64 current = static_cast<u64>(address) + i;
        u32 byte = 0xFF;
        if (current >= base_ && current < static_cast<u64>(base_) + size_) {
            const u32 offset = static_cast<u32>(current - base_);
            const u32 word = offset & ~3u;
            if (known_register(word) && word != kTxFifo && word != kRxFifo) {
                byte = (registers_[word / 4] >> ((offset & 3) * 8)) & 0xFFu;
            }
        }
        result |= static_cast<u64>(byte) << (i * 8);
    }
    return result;
}

void I2cController::write(u32 address, unsigned size, u64 value) {
    bool command_written = false;
    bool tx_written = false;
    for (unsigned i = 0; i < size && i < 8; ++i) {
        const u64 current = static_cast<u64>(address) + i;
        if (current < base_ || current >= static_cast<u64>(base_) + size_) continue;
        const u32 offset = static_cast<u32>(current - base_);
        const u32 word = offset & ~3u;
        if (!known_register(word) || word == kBusy || word == kRxFifo) continue;
        const u32 shift = (offset & 3) * 8;
        const u32 supplied = static_cast<u32>((value >> (i * 8)) & 0xFFu) << shift;
        if (word == kIrqStatus) {
            // W1C acknowledges only the actual supplied byte lanes, never
            // previously stored bits in other bytes of the status word.
            registers_[word / 4] &= ~supplied;
        } else {
            registers_[word / 4] = (registers_[word / 4] & ~(0xFFu << shift)) | supplied;
        }
        command_written |= offset == kCommand;
        tx_written |= offset == kTxFifo;
    }
    // Count port writes for diagnosis only. There is no implemented FIFO or
    // slave that consumes these bytes; this counter is not a guest register.
    if (tx_written) ++tx_writes_;
    // Length-only lane writes do not start a transaction. Submit after all
    // lanes in this access are merged so a native word write is atomic.
    if (command_written) submit_command();
}

void I2cController::submit_command() {
    const u32 command = registers_[kCommand / 4];
    // Only the exact native reset word is evidenced. Do not silently accept
    // reset-like low bytes with unknown length/high control flags.
    if (command == 7) {
        registers_[kBusy / 4] = 0;
        registers_[kIrqStatus / 4] = 0;
        registers_[kTxFifo / 4] = 0;
        tx_writes_ = 0;
        unsupported_ = false;
        ++bus_resets_;
        ZLB_LOG_TRACE("i2c", "I2C%u bus reset: idle, IRQ control=%08X", port_,
                      registers_[kIrqControl / 4]);
        return;
    }
    // Staying busy is a deliberate unsupported-operation boundary, not a
    // claim about physical hardware timeout behavior. In particular, merely
    // polling or writing an opaque IRQ mask cannot manufacture a completion.
    registers_[kBusy / 4] = 1;
    unsupported_ = true;
    ZLB_LOG_WARN("i2c", "I2C%u unsupported command=%08X slave=%02X tx-writes=%llu; busy until reset",
                 port_, command, registers_[kAddress / 4],
                 static_cast<unsigned long long>(tx_writes_));
}

void I2cController::reset() {
    registers_.fill(0);
    tx_writes_ = bus_resets_ = 0;
    unsupported_ = false;
}

const char* I2cController::register_name(u32 address) const {
    return handles(address) ? lookup_name(address & ~3u) : nullptr;
}

void I2cController::enumerate_registers(std::vector<RegisterInfo>& out) const {
    for (const auto& reg : kRegisters) out.push_back({base_ + reg.offset, reg.name, reset_value(reg.offset), 4});
}

bool I2cController::peek_register(const std::string& name, u64& out) const {
    for (const auto& reg : kRegisters) {
        if (name != reg.name) continue;
        // Use the same availability/read-only rules as a guest register read.
        out = reg.offset == kTxFifo || reg.offset == kRxFifo
                  ? 0xFFFFFFFFu : registers_[reg.offset / 4];
        return true;
    }
    return false;
}

bool I2cController::poke_register(const std::string& name, u64 value) {
    for (const auto& reg : kRegisters) {
        if (name != reg.name) continue;
        if (reg.offset == kBusy || reg.offset == kRxFifo) return false;
        write(base_ + reg.offset, 4, value);
        return true;
    }
    return false;
}

std::string I2cController::summary() const {
    return format("I2C%u: %s busy=%u command=%08X slave=%02X tx-writes=%llu resets=%llu IRQ%u=low",
                  port_, unsupported_ ? "unsupported transfer" : "idle/reset only",
                  registers_[kBusy / 4], registers_[kCommand / 4], registers_[kAddress / 4],
                  static_cast<unsigned long long>(tx_writes_), static_cast<unsigned long long>(bus_resets_),
                  port_ == 0 ? kIrqI2c0 : kIrqI2c1);
}

void I2cController::describe(std::vector<std::string>& lines) const {
    lines.push_back(summary());
    lines.push_back("  synchronous bus reset; transfer/FIFO execution and slave responses unsupported");
    lines.push_back("  IRQ-control bits opaque; no ACK/NACK, successful completion or interrupt generated");
}

void I2cController::save_state(StateWriter& writer) const {
    // `port_` is construction-time configuration; everything the guest can
    // program plus the diagnostic counters travels here.
    writer.fixed(registers_, [&](u32 value) { writer.put_u32(value); });
    writer.put_u64(tx_writes_);
    writer.put_u64(bus_resets_);
    writer.put_bool(unsupported_);
}

void I2cController::load_state(StateReader& reader) {
    reader.fixed(registers_, [&](u32& value) { value = reader.get_u32(); });
    tx_writes_ = reader.get_u64();
    bus_resets_ = reader.get_u64();
    unsupported_ = reader.get_bool();
}

}  // namespace zlb::kermit
