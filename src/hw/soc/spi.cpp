// zeliboba - SPI master blocks (syscon / motion / OLED).
//
// The register map is documented in soc_internal.h; this file is the data path.
//
// Why the boot chain needs it: the CMeP second loader does not talk to Ernie
// through the CMeP's own SC mailbox (0xE0B00000), it drives the SoC SPI block at
// 0xE0A00000 instead - a transfer is
//
//   drain the RX FIFO            (0x436E4: lw 0x28 / lw 0x00 until 0x28 reads 0)
//   push the request words       (sw to 0x04, low byte first)
//   clear the interrupt status   (sw 0x600 to 0x24, bits 9 and 10)
//   start the transfer           (sw 1 to 0x10)
//   wait for the reply           (0x28 = byte count, 0x00 = reply words)
//
// and the ARM syscon.elf driver (0x8100003C) uses exactly the same sequence. The
// protocol that travels over the link is implemented in hw/syscon/ernie_spi.cpp.
#include "hw/soc/soc_internal.h"

namespace zlb::kermit {

Spi::Spi(std::string name, u32 base, u32 size, u32 port)
    : RegisterBlock(std::move(name), base, size), port_(port) {
    define(kRxFifo, "SPI_RXFIFO", 0);
    define(kTxFifo, "SPI_TXFIFO", 0);
    define(kCtl, "SPI_CTL", 0);
    define(kIntCtl, "SPI_INTCTL", 0);
    define(kStatus, "SPI_STATUS", 0);
    define(kDmaCtl, "SPI_DMACTL", 0);
    define(kReg18, "SPI_REG18", 0);
    define(kReg20, "SPI_REG20", 0);
    define(kIntStatus, "SPI_INT_STATUS", 0);
    define(kRxFifoStatus, "SPI_RXFIFO_STATUS", 0);
    define(kTxFifoStatus, "SPI_TXFIFO_STATUS", 0);
}

void Spi::reset() {
    RegisterBlock::reset();
    tx_.clear();
    rx_.clear();
    ctl_ = 0;
    int_ctl_ = 0;
    dma_ctl_ = 0;
    reg18_ = 0;
    reg20_ = 0;
    int_status_ = 0;
    busy_ = false;
    irq_line_ = false;
    transfers_ = 0;
    bytes_tx_ = 0;
    bytes_rx_ = 0;
    last_request_.clear();
    last_response_.clear();
}

u64 Spi::read_word(u32 offset, u64 stored) {
    switch (offset) {
        case kRxFifo: {
            // The FIFO is 16 bit wide: one access pops up to two bytes, low byte
            // first, which is what both drivers reassemble into a word.
            u32 value = 0;
            if (!rx_.empty()) {
                value = rx_.front();
                rx_.pop_front();
            }
            if (!rx_.empty()) {
                value |= static_cast<u32>(rx_.front()) << 8;
                rx_.pop_front();
            }
            if (rx_.empty() && (int_status_ & kIntStatusRxNotEmpty)) {
                // Last word handed out: the "RX FIFO not empty" flag drops, but
                // it is a latch the guest clears by writing the interrupt status.
            }
            update_irq();
            return value;
        }
        case kRxFifoStatus: return rx_.size();
        case kTxFifoStatus: return tx_.size();
        case kStatus: return busy_ ? 1u : 0u;
        case kIntStatus: return int_status_;
        default: return stored;
    }
}

void Spi::write_word(u32 offset, u64 value) {
    const u32 word = static_cast<u32>(value);
    switch (offset) {
        case kTxFifo: {
            tx_.push_back(static_cast<u8>(word & 0xFF));
            tx_.push_back(static_cast<u8>((word >> 8) & 0xFF));
            bytes_tx_ += 2;
            break;
        }
        case kCtl: ctl_ = word; break;
        case kIntCtl: int_ctl_ = word; break;
        case kDmaCtl: dma_ctl_ = word; break;
        case kReg18: reg18_ = word; break;
        case kReg20: reg20_ = word; break;
        case kIntStatus:
            // Write one to clear.
            int_status_ &= ~word;
            update_irq();
            break;
        case kStatus:
            if ((word & 1u) != 0) {
                start_transfer();
            } else {
                busy_ = false;
            }
            break;
        default: break;
    }
}

void Spi::start_transfer() {
    busy_ = true;
    last_request_ = tx_;
    if (slave_) {
        last_response_ = slave_(tx_);
    } else {
        // No device on the link: answer with an empty frame so a driver waiting
        // for the receive count still terminates instead of spinning forever.
        last_response_.clear();
    }
    tx_.clear();
    for (u8 byte : last_response_) rx_.push_back(byte);
    bytes_rx_ += last_response_.size();
    if (!last_response_.empty()) int_status_ |= kIntStatusRxNotEmpty;
    ++transfers_;
    busy_ = false;
    update_irq();
}

void Spi::update_irq() {
    const bool line = (int_status_ & int_ctl_) != 0;
    if (line == irq_line_) return;
    irq_line_ = line;
    if (irq_callback_) irq_callback_(0, line);
}

std::string Spi::summary() const {
    return format("SPI%u: %llu transfers, %llu bytes out, %llu in, rx pending %zu, ctl=0x%X",
                  port_, static_cast<unsigned long long>(transfers_),
                  static_cast<unsigned long long>(bytes_tx_),
                  static_cast<unsigned long long>(bytes_rx_), rx_.size(), ctl_);
}

void Spi::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("SPI%u master @0x%08X", port_, base_));
    lines.push_back(format("  control        : CTL=0x%X INTCTL=0x%X DMACTL=0x%X reg18=0x%X reg20=0x%X",
                           ctl_, int_ctl_, dma_ctl_, reg18_, reg20_));
    lines.push_back(format("  interrupt      : STATUS=0x%X (rx not empty=%d), enabled mask 0x%X",
                           int_status_, (int_status_ & kIntStatusRxNotEmpty) ? 1 : 0, int_ctl_));
    lines.push_back(format("  transfer       : %llu started, %llu bytes out, %llu bytes in",
                           static_cast<unsigned long long>(transfers_),
                           static_cast<unsigned long long>(bytes_tx_),
                           static_cast<unsigned long long>(bytes_rx_)));
    lines.push_back(format("  last request   : %zu bytes, first byte 0x%02X", last_request_.size(),
                           last_request_.empty() ? 0 : last_request_[0]));
    lines.push_back(format("  last response  : %zu bytes, first byte 0x%02X", last_response_.size(),
                           last_response_.empty() ? 0 : last_response_[0]));
    std::string request;
    for (size_t i = 0; i < last_request_.size() && i < 16; ++i) request += format(" %02X", last_request_[i]);
    if (!request.empty()) lines.push_back("  request bytes  :" + request);
    std::string response;
    for (size_t i = 0; i < last_response_.size() && i < 16; ++i) response += format(" %02X", last_response_[i]);
    if (!response.empty()) lines.push_back("  response bytes :" + response);
}

}  // namespace zlb::kermit
