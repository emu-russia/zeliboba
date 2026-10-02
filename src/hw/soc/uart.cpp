// zeliboba - Kermit debug UART.
//
// The register set is the classic 16550 one, which is what the small serial
// blocks used for a kernel console implement; the base address (0xE2030000)
// is the one the existing C# device model reserves ("ARM.Uart" in
// VitaTestSuite/Core/Devices/VitaDevices.cs). (ASSUMPTION: the exact register
// semantics have not been verified against lowio.elf - see the final report.)
//
// What matters for the emulator is the data path, not the baud rate:
//
//   * every byte written to the transmit register is appended to a string that
//     take_uart_output() hands to the debugger, logged line by line, and that
//     the kernel's console output ends up in;
//   * uart_write() injects bytes into the receive FIFO and raises the line
//     status / receive-data-available interrupt, which is SPI 44 (see the
//     kermit::Irq enum in hw/soc.h).
#include "hw/soc/soc_internal.h"

namespace zlb::kermit {
namespace {

// 16550 register offsets, with a 4 byte stride between the eight registers.
constexpr u32 kRxTx = 0x00;    // RBR / THR (DLL when DLAB=1)
constexpr u32 kIer = 0x04;     // IER (DLM when DLAB=1)
constexpr u32 kIirFcr = 0x08;  // IIR (read) / FCR (write)
constexpr u32 kLcr = 0x0C;
constexpr u32 kMcr = 0x10;
constexpr u32 kLsr = 0x14;
constexpr u32 kMsr = 0x18;
constexpr u32 kScr = 0x1C;

// The same registers expressed as the indices reg_offset() returns. Keeping
// both forms spelled out avoids repeating the (offset >> 2) & 7 arithmetic in
// every switch arm.
constexpr u32 kRegRxTx = kRxTx >> 2;
constexpr u32 kRegIer = kIer >> 2;
constexpr u32 kRegIirFcr = kIirFcr >> 2;
constexpr u32 kRegLcr = kLcr >> 2;
constexpr u32 kRegMcr = kMcr >> 2;
constexpr u32 kRegLsr = kLsr >> 2;
constexpr u32 kRegMsr = kMsr >> 2;
constexpr u32 kRegScr = kScr >> 2;

constexpr u8 kLsrDataReady = 0x01;
constexpr u8 kLsrOverrun = 0x02;
constexpr u8 kLsrThrEmpty = 0x20;
constexpr u8 kLsrTransmitterEmpty = 0x40;

constexpr u8 kIerRxData = 0x01;
constexpr u8 kIerTxEmpty = 0x02;
constexpr u8 kIerLineStatus = 0x04;
constexpr u8 kIerModemStatus = 0x08;

constexpr u8 kIirNoInterrupt = 0x01;
constexpr u8 kIirTxEmpty = 0x02;
constexpr u8 kIirRxData = 0x04;
constexpr u8 kIirLineStatus = 0x06;

constexpr u8 kMcrLoopback = 0x10;

// Peripheral glue (see the constructor): +0x28 is the block's ready status.
constexpr u32 kPeripheralStatus = 0x28;
constexpr u32 kPeripheralReady = 0x0300;  // bit 8 = powered, bit 9 = clocked

// The firmware's own console register: kernel_boot_loader's putchar is
//     ldr  r0, [r2, #40]      ; +0x28 status
//     tst  r0, #0x100         ; bit 8 = transmitter ready
//     beq  <poll>
//     str  r1, [r2, #112]     ; +0x70 = transmit data
// (0x4003BC98..0x4003BCA2, table of console bases at 0x4005C058 = {0xE2030000,
// 0xE2040000, ...}). The +0x70 data register is therefore part of the real
// block layout, not the 16550 stride the rest of this file assumes.
constexpr u32 kConsoleData = 0x70;

constexpr size_t kRxCapacity = 4096;

}  // namespace


Uart::Uart(std::string name, u32 base, u32 size) : RegisterBlock(std::move(name), base, size) {
    // The transmit/receive register is a FIFO port: every byte written goes out
    // (and every byte read pops the receive queue). Uart::read_word/write_word
    // decode it, so it is registered as an ordinary register here.
    define(kRxTx, "UART_RBR_THR_DLL", 0);
    define(kIer, "UART_IER_DLM", 0);
    define(kIirFcr, "UART_IIR_FCR", kIirNoInterrupt);
    define(kLcr, "UART_LCR", 0x03);  // 8 bits, no parity, one stop bit
    define(kMcr, "UART_MCR", 0);
    define(kLsr, "UART_LSR", kLsrThrEmpty | kLsrTransmitterEmpty);
    define(kMsr, "UART_MSR", 0xB0);
    define(kScr, "UART_SCR", 0);
    // SoC glue in the same window: KBL's peripheral bring-up helper (0x4003BBCC)
    // treats every block in its table the same way - it writes +0x04/+0x10/+0x20/
    // +0x30/+0x40/+0x50/+0x60/+0x64, waits for a ready bit at +0x28 (bit 8 at
    // 0x4003BC98, bit 9 at 0x4003BCCC) and then writes +0x70.  For this block that
    // is the UART's own bring-up (it writes a divisor of 0x77E and 8N1), so the
    // status reports "powered and clocked".
    define(kPeripheralStatus, "UART_PERIPH_STATUS", kPeripheralReady, 4);
    // The firmware console's data register (see kConsoleData): defined so that
    // RegisterBlock::write()/read() actually calls write_word()/read_word() for
    // it - an undefined offset is only kept in the byte image and the character
    // would be dropped.
    define(kConsoleData, "UART_CONSOLE_DATA", 0);
}

void Uart::reset() {
    RegisterBlock::reset();
    r_ = Regs{};
    rx_.clear();
    output_.clear();
    log_line_.clear();
    irq_line_ = false;
    bytes_tx_ = 0;
    bytes_rx_ = 0;
    update_status();
    if (irq_callback_) irq_callback_(static_cast<u32>(Irq::Uart0), false);
}

void Uart::update_status() {
    u32 value = kLsrThrEmpty | kLsrTransmitterEmpty;
    if (!rx_.empty()) value |= kLsrDataReady;
    if (overrun_) value |= kLsrOverrun;
    poke(kLsr, value);
}

u64 Uart::read_word(u32 offset, u64 stored) {
    if (offset == kPeripheralStatus) return kPeripheralReady;
    if (offset == kConsoleData) return read_data_register();
    const u32 reg = reg_offset(offset);
    const bool dlab = (r_.lcr & 0x80) != 0;
    switch (reg) {
        case kRegRxTx:
            return read_data_register();
        case kRegIer:
            return dlab ? ((r_.divisor >> 8) & 0xFF) : r_.ier;
        case kRegIirFcr: {
            u8 value = kIirNoInterrupt;
            if (r_.ier & kIerLineStatus) {
                value = kIirLineStatus;
            } else if ((r_.ier & kIerRxData) && !rx_.empty()) {
                value = kIirRxData;
            } else if (r_.ier & kIerTxEmpty) {
                value = kIirTxEmpty;
            }
            return value;
        }
        case kRegLcr:
            return r_.lcr;
        case kRegMcr:
            return r_.mcr;
        case kRegLsr:
            update_status();
            return peek(kLsr);
        case kRegMsr:
            return r_.msr;
        case kRegScr:
            return r_.scr;
        default:
            return stored;
    }
}

void Uart::write_word(u32 offset, u64 value) {
    if (offset == kConsoleData) {
        write_data_register(static_cast<u8>(value & 0xFF));
        return;
    }
    const u32 reg = reg_offset(offset);
    const u8 value8 = static_cast<u8>(value & 0xFF);
    switch (reg) {
        case kRegRxTx:
            write_data_register(value8);
            return;
        case kRegIer:
            // The divisor latch shares this address when LCR.DLAB is set.
            if (r_.lcr & 0x80) {
                r_.divisor = static_cast<u16>((r_.divisor & 0x00FF) | (static_cast<u16>(value8) << 8));
                return;
            }
            r_.ier = value8;
            poke(kIer, value8);
            update_irq();
            return;
        case kRegIirFcr:
            if (value8 & 0x02) rx_.clear();
            if (value8 & 0x01) r_.fcr = value8;
            update_status();
            update_irq();
            return;
        case kRegLcr:
            r_.lcr = value8;
            return;
        case kRegMcr:
            r_.mcr = value8;
            return;
        case kRegLsr:
            overrun_ = false;
            update_status();
            return;
        case kRegMsr:
            return;
        case kRegScr:
            r_.scr = value8;
            return;
        default:
            return;
    }
}

u8 Uart::read_data_register() {
    if (r_.lcr & 0x80) return static_cast<u8>(r_.divisor & 0xFF);
    if (rx_.empty()) {
        update_status();
        return 0;
    }
    const u8 byte = rx_.front();
    rx_.pop_front();
    update_status();
    update_irq();
    return byte;
}

void Uart::write_data_register(u8 value) {
    if (r_.lcr & 0x80) {
        r_.divisor = static_cast<u16>((r_.divisor & 0xFF00) | value);
        return;
    }

    // Transmit. Capture the byte, echo it to the debug log line by line so the
    // 'uart' output is readable, and never block.
    output_.push_back(static_cast<char>(value));
    ++bytes_tx_;
    if (mirror_) {
        update_status();
        return;
    }
    if (value == '\n' || value == '\r') {
        if (!log_line_.empty()) ZLB_LOG_INFO("uart", "%s", log_line_.c_str());
        log_line_.clear();
    } else if (value >= 0x20 && value < 0x7F) {
        log_line_.push_back(static_cast<char>(value));
        if (log_line_.size() > 240) {
            ZLB_LOG_INFO("uart", "%s", log_line_.c_str());
            log_line_.clear();
        }
    }
    update_status();
}

void Uart::push_rx(u8 byte) {
    if (rx_.size() >= kRxCapacity) {
        rx_.pop_front();
        overrun_ = true;
    }
    rx_.push_back(byte);
    ++bytes_rx_;
    update_status();
    update_irq();
}

void Uart::push_rx(const std::string& text) {
    for (char c : text) push_rx(static_cast<u8>(c));
}

std::string Uart::take_output() {
    std::string out;
    out.swap(output_);
    if (!log_line_.empty()) {
        if (!mirror_) ZLB_LOG_INFO("uart", "%s", log_line_.c_str());
        log_line_.clear();
    }
    return out;
}

void Uart::update_irq() {
    const bool want =
        ((r_.ier & kIerRxData) && !rx_.empty()) || ((r_.ier & kIerLineStatus) && overrun_);
    if (want == irq_line_) return;
    irq_line_ = want;
    if (irq_callback_) irq_callback_(static_cast<u32>(Irq::Uart0), irq_line_);
}

std::string Uart::summary() const {
    const u32 id = static_cast<u32>(Irq::Uart0);
    return format("%s tx=%llu rx=%llu fifo=%llu irq=%s(%u)", name_.c_str(),
                  static_cast<unsigned long long>(bytes_tx_), static_cast<unsigned long long>(bytes_rx_),
                  static_cast<unsigned long long>(rx_.size()), irq_line_ ? "on" : "off", id);
}

void Uart::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s: 16550-ish console, %llu bytes transmitted, %llu received, %llu queued",
                           name_.c_str(), static_cast<unsigned long long>(bytes_tx_),
                           static_cast<unsigned long long>(bytes_rx_), static_cast<unsigned long long>(rx_.size())));
    lines.push_back(format("    LCR=0x%02X IER=0x%02X MCR=0x%02X divisor=%u output_pending=%llu", r_.lcr, r_.ier, r_.mcr,
                           r_.divisor, static_cast<unsigned long long>(output_.size())));
    if (!log_line_.empty()) lines.push_back(format("    partial line: \"%s\"", log_line_.c_str()));
}

void Uart::save_state(StateWriter& writer) const {
    RegisterBlock::save_state(writer);
    writer.begin("Uart.regs");
    writer.put_u8(r_.ier);
    writer.put_u8(r_.fcr);
    writer.put_u8(r_.lcr);
    writer.put_u8(r_.mcr);
    writer.put_u8(r_.scr);
    writer.put_u8(r_.msr);
    writer.put_u16(r_.divisor);
    writer.end();
    // The receive FIFO is a deque, so its contents travel explicitly as a
    // length followed by the bytes.
    writer.put_u32(static_cast<u32>(rx_.size()));
    for (u8 byte : rx_) writer.put_u8(byte);
    writer.str(output_);
    writer.str(log_line_);
    writer.put_bool(irq_line_);
    writer.put_bool(overrun_);
    writer.put_bool(mirror_);
    writer.put_u64(bytes_tx_);
    writer.put_u64(bytes_rx_);
    // `irq_callback_` is host wiring: never serialised.
}

void Uart::load_state(StateReader& reader) {
    RegisterBlock::load_state(reader);
    reader.begin("Uart.regs");
    r_.ier = reader.get_u8();
    r_.fcr = reader.get_u8();
    r_.lcr = reader.get_u8();
    r_.mcr = reader.get_u8();
    r_.scr = reader.get_u8();
    r_.msr = reader.get_u8();
    r_.divisor = reader.get_u16();
    reader.end();
    const u32 rx_size = reader.get_u32();
    rx_.clear();
    for (u32 i = 0; i < rx_size && reader.ok(); ++i) rx_.push_back(reader.get_u8());
    output_ = reader.str();
    log_line_ = reader.str();
    irq_line_ = reader.get_bool();
    overrun_ = reader.get_bool();
    mirror_ = reader.get_bool();
    bytes_tx_ = reader.get_u64();
    bytes_rx_ = reader.get_u64();
}

}  // namespace zlb::kermit
