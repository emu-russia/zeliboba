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

namespace {
constexpr u32 kOledStreamCtl = 0x30001;
constexpr size_t kOledDiagnosticBytes = 64;  // a diagnostic bound, not FIFO capacity

// Framing is independent of the responder's result byte. An error packet can
// be physically ready, but malformed/absent bytes cannot generate GPIO ready.
// SPI clocks low16 words: the single optional trailing byte is wire padding.
bool framed_packet(const std::vector<u8>& packet, u8 minimum_length) {
    if (packet.size() < 4 || packet[2] < minimum_length) return false;
    const size_t length = static_cast<size_t>(packet[2]) + 3;
    if (packet.size() != length && packet.size() != (length + (length & 1u))) return false;
    unsigned sum = 0;
    for (size_t i = 0; i < length; ++i) sum += packet[i];
    return static_cast<u8>(sum) == 0xFF;
}
}  // namespace

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
    cancel_syscon_ready();
    syscon_ready_edges_ = 0;
    const bool was_asserted = irq_line_;
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
    oled_stream_armed_ = false;
    irq_line_ = false;
    transfers_ = 0;
    bytes_tx_ = 0;
    bytes_rx_ = 0;
    last_request_.clear();
    last_response_.clear();
    if (was_asserted && irq_callback_) irq_callback_(0, false);
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
            if (rx_.empty()) cancel_syscon_ready();
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
            if (oled_stream_armed_) consume_oled_words();
            break;
        }
        case kCtl:
            ctl_ = word;
            if (port_ == 0 && word != 0) cancel_syscon_ready();
            if (word != kOledStreamCtl) oled_stream_armed_ = false;
            break;
        case kIntCtl:
            int_ctl_ = word;
            update_irq();
            break;
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
                // The native read helper stops before reading captured input.
                // Stop disarms future clocks but does not flush the RX FIFO.
                oled_stream_armed_ = false;
                busy_ = false;
                cancel_syscon_ready();
            }
            break;
        default: break;
    }
}

void Spi::start_transfer() {
    cancel_syscon_ready();
    const bool fresh_rx = rx_.empty();
    if (port_ == 2 && ctl_ == kOledStreamCtl) {
        // Genuine 1.04 OLED open starts an empty engine, then writes low16
        // words. Word consumption is synchronous; no divisor, FIFO capacity,
        // clock rate or transfer-complete interrupt is inferred here.
        oled_stream_armed_ = true;
        last_request_.clear();
        last_response_.clear();
        ++transfers_;
        consume_oled_words();
        return;
    }
    oled_stream_armed_ = false;
    busy_ = true;
    last_request_ = tx_;
    if (slave_) {
        last_response_ = slave_(tx_);
    } else {
        // No framed peer supplies a reply. Transfer completion and receive
        // availability are separate: an RX poll can still wait indefinitely.
        last_response_.clear();
    }
    tx_.clear();
    for (u8 byte : last_response_) rx_.push_back(byte);
    bytes_rx_ += last_response_.size();
    if (!last_response_.empty()) int_status_ |= kIntStatusRxNotEmpty;
    ++transfers_;
    busy_ = false;
    // The reached native Syscon start precedes GPIO3 rise. Only this fresh,
    // complete framed generation may be associated with a later driven rise.
    // The genuine second loader also polls this GPIO latch before secure-
    // kernel handoff. CPU phase must not suppress a real qualified reply.
    syscon_valid_generation_ = port_ == 0 && ctl_ == 0 && syscon_gpio_qualified_ &&
        !syscon_request_high_ && fresh_rx && framed_packet(last_request_, 1) &&
        framed_packet(last_response_, 2);
    if (syscon_valid_generation_) {
        syscon_peer_active_ = true;
        drive_syscon_ready(false); // first real exchange establishes idle-high
    }
    update_irq();
}

void Spi::set_syscon_ready_callback(std::function<void(bool)> callback) {
    syscon_ready_callback_ = std::move(callback);
    if (syscon_ready_callback_)
        syscon_ready_callback_(syscon_peer_active_ && !syscon_ready_asserted_);
}

void Spi::drive_syscon_ready(bool asserted) {
    syscon_ready_asserted_ = asserted;
    if (syscon_ready_callback_)
        syscon_ready_callback_(syscon_peer_active_ && !asserted);
}

void Spi::cancel_syscon_ready() {
    syscon_valid_generation_ = false;
    syscon_ready_scheduled_ = false;
    drive_syscon_ready(false);
}

void Spi::reset_syscon_ready_wire() {
    syscon_peer_active_ = false;
    cancel_syscon_ready();
}

void Spi::set_syscon_gpio(bool edge_qualified, bool driven_request_high) {
    if (port_ != 0) return;
    syscon_gpio_qualified_ = edge_qualified;
    const bool was_high = syscon_request_high_;
    syscon_request_high_ = driven_request_high;
    if (!edge_qualified || !driven_request_high) {
        if (was_high || !edge_qualified) cancel_syscon_ready();
        return;
    }
    if (!was_high && syscon_valid_generation_) syscon_ready_scheduled_ = true;
}

void Spi::tick(u64 ticks) {
    // One PERIPHCLK tick is an explicit emulator latency, not recovered wire
    // timing. Reads cannot advance it; mirrors never tick this object twice.
    if (ticks == 0 || !syscon_ready_scheduled_) return;
    syscon_ready_scheduled_ = false;
    if (ctl_ != 0 || !syscon_peer_active_ || !syscon_gpio_qualified_ || !syscon_request_high_ || rx_.empty()) return;
    ++syscon_ready_edges_;
    drive_syscon_ready(true);
    // Unchanged Lowio mode3 records shadow bit4=0, and its parent handler
    // qualifies candidate pins from sampled input XOR that shadow. A low
    // pulse returning idle-high before handler dispatch is compatible with
    // the exact native bytes; holding low excludes sub4. Sub-tick pulse width
    // is an explicit board-model choice, not an electrical measurement.
    drive_syscon_ready(false);
}

void Spi::consume_oled_words() {
    busy_ = !tx_.empty();
    for (u8 byte : tx_) {
        if (last_request_.size() < kOledDiagnosticBytes) last_request_.push_back(byte);
        // Chosen board-wire assumption: an undriven input is pulled high.
        // Every actually consumed low16 TX word clocks sixteen input bits,
        // sampled as FFFF. This does not decode commands or emulate a panel;
        // the unchanged A1 worker rejects these bytes as readiness 2.
        rx_.push_back(0xFF);
        if (last_response_.size() < kOledDiagnosticBytes) last_response_.push_back(0xFF);
    }
    bytes_rx_ += tx_.size();
    if (!tx_.empty()) int_status_ |= kIntStatusRxNotEmpty;
    tx_.clear();
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
    std::string result = format("SPI%u: %llu transfers, %llu bytes out, %llu in, rx pending %zu, ctl=0x%X",
                  port_, static_cast<unsigned long long>(transfers_),
                  static_cast<unsigned long long>(bytes_tx_),
                  static_cast<unsigned long long>(bytes_rx_), rx_.size(), ctl_);
    if (port_ == 0)
        result += format(" GPIO ready=%s scheduled=%u edges=%llu", syscon_ready_asserted_ ? "low" : syscon_peer_active_ ? "idle-high" : "absent",
                         syscon_ready_scheduled_, static_cast<unsigned long long>(syscon_ready_edges_));
    if (port_ == 2)
        result += format(" OLED stream=%s input=modeled undriven high (not panel ID)",
                         oled_stream_armed_ ? "armed" : "stopped");
    return result;
}

void Spi::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("SPI%u master @0x%08X", port_, base_));
    lines.push_back(format("  control        : CTL=0x%X INTCTL=0x%X DMACTL=0x%X reg18=0x%X reg20=0x%X",
                           ctl_, int_ctl_, dma_ctl_, reg18_, reg20_));
    lines.push_back(format("  interrupt      : STATUS=0x%X (rx not empty=%d), enabled mask 0x%X",
                           int_status_, (int_status_ & kIntStatusRxNotEmpty) ? 1 : 0, int_ctl_));
    if (port_ == 2) {
        lines.push_back(format("  OLED subset    : CTL30001 stream %s, synchronous low16 word consumption",
                               oled_stream_armed_ ? "armed" : "stopped"));
        lines.push_back("  modeled input  : chosen undriven pulled-high wire; not a recovered pull or panel ID");
        lines.push_back("  count limits   : inherited byte queue; OLED FIFO rate/capacity/count units unverified");
        lines.push_back("  diagnostics    : first 64 bytes per OLED start, not a FIFO-capacity limit");
    }
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
