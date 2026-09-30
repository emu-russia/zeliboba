// zeliboba - Kermit SD/eMMC host controller (SDIF).
//
// Base address
// ------------
// sdif.elf carries a descriptor table at VA 0x81009FA8 whose entries are
// 0xE0B00000, 0xE0C00000 and 0xE0C10000 (32 bit literals), and the same module
// contains the strings "SceSdif0", "SceSdif1", "SceSdif2" (VA 0x8100A0C8,
// 0x8100A0D4, 0x8100A0E0) plus the format string
// "%d:0x%08x:0x%08x:%d:Skip sdif port[%d] reset" (VA 0x8100A068). sysmem.elf
// repeats the same three values at VA 0x810314E0. That is the evidence for the
// three port bases used in soc_internal.h (kSdif0Base .. kSdif2Base); only port
// 0 (the eMMC port) is wired to an EmmcCard here.
//
// Register layout
// ---------------
// The offsets below are the SD Host Controller Simplified Specification ones
// (SDMA 0x00 ... slot interrupt status 0xFC, plus the vendor window at 0x100).
// (ASSUMPTION: the offset-by-offset meaning was not confirmed instruction by
// instruction against sdif.elf; the block is however register-compatible with
// the SDHCI family, which is what a 4 KiB port window of this shape is.)
//
// Data path: both PIO (the 0x20 buffer data port) and ADMA2 (a descriptor table
// in guest memory) are implemented, because a driver that uses 0x20 must not
// hang and one that programs ADMA must still move data. Every transfer goes
// through EmmcCard::read_blocks / write_blocks, so the host never touches the
// image file directly.
#include "hw/soc/soc_internal.h"

#include <cstdio>
#include <cstdlib>

namespace zlb::kermit {

namespace {

// Temporary bring-up trace: ZLB_SDIF_TRACE=1 prints every register write that
// takes part in a transfer and every command the model executes. It exists to
// recover what the CMeP second loader programs without guessing at its stack
// frames; it costs nothing when the variable is unset.
bool sdif_trace() {
    static const bool on = [] {
        const char* value = std::getenv("ZLB_SDIF_TRACE");
        return value != nullptr && value[0] != '0';
    }();
    return on;
}

void sdif_trace_write(const char* what, u32 offset, u64 value, unsigned size) {
    if (!sdif_trace()) return;
    std::fprintf(stderr, "[sdif] w %-18s +0x%02X size=%u value=0x%llX\n", what, offset, size,
                 static_cast<unsigned long long>(value));
}

// SDMA system address / argument 2
constexpr u32 kSdmaAddr = 0x00;
constexpr u32 kBlockSize = 0x04;
constexpr u32 kBlockCount = 0x06;
constexpr u32 kArgument = 0x08;
constexpr u32 kTransferMode = 0x0C;
constexpr u32 kCommand = 0x0E;
constexpr u32 kResponse0 = 0x10;
constexpr u32 kResponse1 = 0x14;
constexpr u32 kResponse2 = 0x18;
constexpr u32 kResponse3 = 0x1C;
constexpr u32 kBufferDataPort = 0x20;
constexpr u32 kPresentState = 0x24;
constexpr u32 kHostControl1 = 0x28;
constexpr u32 kPowerControl = 0x29;
constexpr u32 kBlockGapControl = 0x2A;
constexpr u32 kWakeupControl = 0x2B;
constexpr u32 kClockControl = 0x2C;
constexpr u32 kTimeoutControl = 0x2E;
constexpr u32 kSoftwareReset = 0x2F;
constexpr u32 kNormalIntStatus = 0x30;
constexpr u32 kErrorIntStatus = 0x32;
constexpr u32 kNormalIntStatusEnable = 0x34;
constexpr u32 kErrorIntStatusEnable = 0x36;
constexpr u32 kNormalIntSignalEnable = 0x38;
constexpr u32 kErrorIntSignalEnable = 0x3A;
constexpr u32 kAutoCmdErrorStatus = 0x3C;
constexpr u32 kHostControl2 = 0x3E;
constexpr u32 kCapabilities = 0x40;
constexpr u32 kCapabilities1 = 0x44;
constexpr u32 kMaxCurrentCapabilities = 0x48;
constexpr u32 kAdmaErrorStatus = 0x54;
constexpr u32 kAdmaSystemAddress = 0x58;  // 0x58..0x5F, 64 bit
constexpr u32 kSlotIntStatus = 0xFC;
constexpr u32 kHostControllerVersion = 0xFE;
constexpr u32 kVendorBase = 0x100;

// Present state bits (SDHCI 2.0, chapter 2.2.10).
constexpr u32 kPsCmdInhibit = 1u << 0;
constexpr u32 kPsCmdInhibitDat = 1u << 1;
constexpr u32 kPsDatLineActive = 1u << 2;
constexpr u32 kPsBufferReadEnable = 1u << 11;
constexpr u32 kPsBufferWriteEnable = 1u << 10;
constexpr u32 kPsCardInserted = 1u << 16;
constexpr u32 kPsCardStable = 1u << 17;

// Normal interrupt status bits.
constexpr u16 kIntCmdComplete = 1u << 0;
constexpr u16 kIntTransferComplete = 1u << 1;
constexpr u16 kIntBlockGap = 1u << 2;
constexpr u16 kIntDma = 1u << 3;
constexpr u16 kIntBufferWriteReady = 1u << 4;
constexpr u16 kIntBufferReadReady = 1u << 5;
constexpr u16 kIntCardInsertion = 1u << 6;
constexpr u16 kIntBufferReadOverrun = 1u << 7;
constexpr u16 kIntCommandTimeout = 1u << 0;  // error interrupt status

// Transfer mode bits.
constexpr u16 kTmDmaEnable = 1u << 0;
constexpr u16 kTmBlockCountEnable = 1u << 1;
constexpr u16 kTmAutoCmd12 = 1u << 2;
constexpr u16 kTmAutoCmd23 = 1u << 3;
constexpr u16 kTmRead = 1u << 4;
constexpr u16 kTmMultiBlock = 1u << 5;

// Command register bits.
constexpr u16 kCmdResponseMask = 0x3;
constexpr u16 kCmdResponseLong = 0x1;
constexpr u16 kCmdResponseShort = 0x2;
constexpr u16 kCmdCrcCheck = 1u << 3;
constexpr u16 kCmdIndexCheck = 1u << 4;
constexpr u16 kCmdDataPresent = 1u << 5;
constexpr u16 kCmdIndexShift = 8;

constexpr u32 kCommandTimeoutTicks = 1000000;  // 1 s at PERIPHCLK = 1 MHz

std::string cmd_name(u8 index) {
    switch (index) {
        case 0: return "GO_IDLE_STATE";
        case 1: return "SEND_OP_COND";
        case 2: return "ALL_SEND_CID";
        case 3: return "SEND_RELATIVE_ADDR";
        case 6: return "SWITCH";
        case 7: return "SELECT_CARD";
        case 8: return "SEND_EXT_CSD";
        case 9: return "SEND_CSD";
        case 12: return "STOP_TRANSMISSION";
        case 13: return "SEND_STATUS";
        case 16: return "SET_BLOCKLEN";
        case 17: return "READ_SINGLE_BLOCK";
        case 18: return "READ_MULTIPLE_BLOCK";
        case 24: return "WRITE_BLOCK";
        case 25: return "WRITE_MULTIPLE_BLOCK";
        case 35: return "ERASE";
        case 55: return "APP_CMD";
        case 51: return "SEND_SCR";
        default: return format("CMD%u", index);
    }
}

std::string acmd_name(u8 index) {
    switch (index) {
        case 6: return "ACMD6 SET_BUS_WIDTH";
        case 13: return "ACMD13 SD_STATUS";
        case 23: return "ACMD23 SET_WR_BLK_ERASE_COUNT";
        case 41: return "ACMD41 SD_APP_OP_COND";
        case 51: return "ACMD51 SEND_SCR";
        default: return format("ACMD%u", index);
    }
}

}  // namespace

Sdif::Sdif(std::string name, u32 base, u32 size, EmmcCard* card, u32 port)
    : RegisterBlock(std::move(name), base, size), card_(card), port_(port) {
    define(kSdmaAddr, "SDMA_SYS_ADDR", 0);
    define(kBlockSize, "BLOCK_SIZE", 0, 2);
    define(kBlockCount, "BLOCK_COUNT", 0, 2);
    define(kArgument, "ARGUMENT", 0);
    define(kTransferMode, "TRANSFER_MODE", 0, 2);
    define(kCommand, "COMMAND", 0, 2);
    define(kResponse0, "RESPONSE0", 0);
    define(kResponse1, "RESPONSE1", 0);
    define(kResponse2, "RESPONSE2", 0);
    define(kResponse3, "RESPONSE3", 0);
    define(kBufferDataPort, "BUFFER_DATA_PORT", 0);
    define(kPresentState, "PRESENT_STATE", 0);
    define(kHostControl1, "HOST_CONTROL1", 0, 1);
    define(kPowerControl, "POWER_CONTROL", 0, 1);
    define(kBlockGapControl, "BLOCK_GAP_CONTROL", 0, 1);
    define(kWakeupControl, "WAKEUP_CONTROL", 0, 1);
    define(kClockControl, "CLOCK_CONTROL", 0, 2);
    define(kTimeoutControl, "TIMEOUT_CONTROL", 0, 1);
    define(kSoftwareReset, "SOFTWARE_RESET", 0, 1);
    define(kNormalIntStatus, "NORMAL_INT_STATUS", 0, 2);
    define(kErrorIntStatus, "ERROR_INT_STATUS", 0, 2);
    define(kNormalIntStatusEnable, "NORMAL_INT_STATUS_ENABLE", 0, 2);
    define(kErrorIntStatusEnable, "ERROR_INT_STATUS_ENABLE", 0, 2);
    define(kNormalIntSignalEnable, "NORMAL_INT_SIGNAL_ENABLE", 0, 2);
    define(kErrorIntSignalEnable, "ERROR_INT_SIGNAL_ENABLE", 0, 2);
    define(kAutoCmdErrorStatus, "AUTO_CMD_ERROR_STATUS", 0, 2);
    define(kHostControl2, "HOST_CONTROL2", 0, 2);
    define(kCapabilities, "CAPABILITIES", 0x00037F77u);
    define(kCapabilities1, "CAPABILITIES1", 0x00000000u);
    define(kMaxCurrentCapabilities, "MAX_CURRENT_CAPABILITIES", 0x00000000u);
    define(kAdmaErrorStatus, "ADMA_ERROR_STATUS", 0, 1);
    define(kAdmaSystemAddress, "ADMA_SYSTEM_ADDRESS", 0);
    define(kAdmaSystemAddress + 4, "ADMA_SYSTEM_ADDRESS_HI", 0);
    define(kSlotIntStatus, "SLOT_INT_STATUS", 0, 2);
    define(kHostControllerVersion, "HOST_CONTROLLER_VERSION", 0x0002u, 2);
    define(kVendorBase + 0x00, "VENDOR_CAPS", 0);
    define(kVendorBase + 0x04, "VENDOR_CTRL", 0);
}

void Sdif::reset() {
    RegisterBlock::reset();
    const bool was_asserted = irq_line_;
    irq_line_ = false;
    data_.clear();
    data_index_ = 0;
    data_pending_ = false;
    read_direction_ = false;
    command_ok_ = true;
    app_cmd_ = false;
    rca_ = 0;
    busy_left_ = 0;
    command_error_in_ = 0;
    error_status_ = 0;
    last_command_register_ = 0;
    last_lba_ = 0;
    last_count_ = 0;
    transfers_ = 0;
    last_command_.clear();
    response_.fill(0);
    poke(kPresentState, kPsCardInserted | kPsCardStable);
    if (was_asserted && irq_callback_) irq_callback_(static_cast<u32>(Irq::Emmc), false);
}

u64 Sdif::read(u32 address, unsigned size) {
    // The buffer data port is a FIFO: reading it pops as many bytes as the access
    // is wide. It used to hand back a single byte for every access size, so the
    // CMeP second loader's PIO loop (0x47B5C: `lw $0,32($10)`) advanced four bytes
    // of destination per byte of FIFO and walked off the end of the transfer.
    if (address >= base_ + kBufferDataPort && address < base_ + kBufferDataPort + 4) {
        u64 out = 0;
        for (unsigned i = 0; i < size; ++i) {
            out |= static_cast<u64>(read_data_port()) << (8 * i);
        }
        update_irq();
        return out;
    }
    const u64 value = RegisterBlock::read(address, size);
    return value;
}

void Sdif::write(u32 address, unsigned size, u64 value) {
    const u32 offset = address - base_;
    if (offset >= kBufferDataPort && offset < kBufferDataPort + 4) {
        write_data_port(static_cast<u8>(value & 0xFF));
        update_irq();
        return;
    }
    // A command is issued by *writing* the 16 bit command register at 0x0E, which
    // shares its 32 bit word with the transfer mode register at 0x0C. The trigger
    // used to be "the command half is non zero", which silently dropped CMD0:
    // GO_IDLE_STATE has index 0 and no flags, so the whole register is zero and
    // the CMeP second loader (which clears the status, writes 0 to 0x0E at
    // 0x47912 and then waits for Command Complete) span forever.
    const bool touches_command = offset <= kCommand && offset + size > kCommand;
    if (sdif_trace() && offset != kNormalIntStatus && offset != kErrorIntStatus &&
        offset != kNormalIntStatusEnable && offset != kErrorIntStatusEnable &&
        offset != kNormalIntSignalEnable && offset != kErrorIntSignalEnable) {
        sdif_trace_write(name_.c_str(), offset, value, size);
    }
    RegisterBlock::write(address, size, value);
    if (!touches_command) return;
    const u32 command_bits = (static_cast<u32>(store(kTransferMode)) >> 16) & 0x3FFF;
    // A write that lands exactly on the command register is always a command
    // (a driver may re-issue the same one); a wider write only starts one when
    // the command half actually changed, so programming the transfer mode does
    // not re-run the previous command.
    if (offset != kCommand && command_bits == last_command_register_) return;
    last_command_register_ = command_bits;
    execute_command();
}

u64 Sdif::read_word(u32 offset, u64 stored) {
    if (offset == kPresentState) {
        u32 state = kPsCardInserted | kPsCardStable;
        if (data_pending_) {
            state |= kPsDatLineActive;
            state |= read_direction_ ? kPsBufferReadEnable : kPsBufferWriteEnable;
            state |= kPsCmdInhibitDat;
        }
        if (busy_left_ != 0) state |= kPsCmdInhibit | kPsCmdInhibitDat;
        return state;
    }
    if (offset == kClockControl) {
        // Bit 1 of the clock control register is the read only "internal clock
        // stable" flag. The CMeP second loader enables the clock (writes 0x8001:
        // internal clock enable plus a divider) and then spins until the flag
        // reads back set (0x47626..0x47632), so the model raises it as soon as
        // the clock is enabled.
        u32 value = static_cast<u32>(stored);
        if ((value & 0x1u) != 0) value |= 0x2u;
        return value;
    }
    if (offset == kNormalIntStatus) {
        // Buffer Read/Write Ready is a *level* flag on real hardware: it stays
        // asserted while the data FIFO is not empty, so acknowledging it (write
        // one to clear) makes it come straight back. The CMeP second loader reads
        // the Extended CSD in 128 byte chunks and waits for the flag to reappear
        // after each chunk (its PIO loop is 0x47B32..0x47B88; the wait for the
        // next chunk is at 0x47B94), so a one-shot model stalls it after the
        // first chunk.
        u32 value = static_cast<u32>(stored);
        if (data_pending_ && data_ready_in_ == 0 && data_index_ < data_.size()) {
            value |= read_direction_ ? kIntBufferReadReady : kIntBufferWriteReady;
        }
        // The ERROR_INT_STATUS register is the high half of this word; a read of
        // 0x32 is decoded here through the word offset, so the error half has to be
        // merged in (it does not live in the byte image - see error_status_).
        value |= static_cast<u32>(error_status_) << 16;
        return value;
    }
    if (offset >= kResponse0 && offset <= kResponse3) {
        return response_[(offset - kResponse0) / 4];
    }
    return stored;
}

void Sdif::write_word(u32 offset, u64 value) {
    // RegisterBlock has already stored the merged word; these are the registers
    // with side effects. Registers that share a word with another register (the
    // transfer mode / command pair at 0x0C) must not call store() here, or the
    // sibling bits would be lost; the command register is handled in write().
    const u32 value32 = static_cast<u32>(value);
    if (sdif_trace() && offset == kNormalIntStatus) {
        std::fprintf(stderr, "[sdif] ack INT_STATUS word 0x%08X (normal was 0x%04X, error 0x%04X)\n",
                     value32, static_cast<unsigned>(peek(kNormalIntStatus) & 0xFFFF),
                     static_cast<unsigned>(error_status_));
    }
    // The command register is decoded in write(), which knows whether the access
    // actually covered 0x0E (a zero command half is a valid CMD0).
    switch (offset) {
        case kNormalIntStatus:
            // The word at 0x30 carries NORMAL_INT_STATUS in its low half and
            // ERROR_INT_STATUS (0x32) in its high half, and write_word() is only
            // ever handed the *word* offset - so a `case kErrorIntStatus` could
            // never match (the same trap the software-reset byte above fell into).
            // Write one to clear: the caller's low bytes clear the normal status,
            // its high bytes clear the error status.
            store(kNormalIntStatus,
                  (peek(kNormalIntStatus) & 0xFFFFull) & ~static_cast<u64>(value32 & 0xFFFFu));
            error_status_ =
                static_cast<u16>(error_status_ & ~static_cast<u16>((value32 >> 16) & 0xFFFFu));
            update_irq();
            return;
        case kClockControl: {
            // 0x2C..0x2F share this 32 bit word: clock control (16 bit at 0x2C),
            // timeout control (0x2E) and software reset (0x2F). RegisterBlock
            // only ever hands over the word aligned offset, so the byte registers
            // have to be decoded from the merged value here - the earlier
            // `case kSoftwareReset` could never match and the reset bit stayed
            // set forever, which hangs the CMeP second loader (it writes 1 to
            // 0x2F and spins on the read at 0x47496..0x474A6).
            // SDHCI 2.0: bit 0 = reset the whole host, bit 1 = reset the command
            // circuit, bit 2 = reset the data circuit.  The model executes commands
            // synchronously, so every one of them completes immediately; each bit
            // must therefore self-clear, or a driver that polls it spins forever.
            // NSKBL only sets bit 0 for its first command (0x5101D788..0x5101D790,
            // round 241) but its error path resets the *command* circuit with bit 1
            // (0x5101ED04, `strb r1,#2` / poll at 0x5101ED0A..0x5101ED12) and parked
            // there once the error status became visible (round 303).
            const u32 reset = (value32 >> 24) & 0x07u;
            if (reset != 0u) {
                if ((reset & 0x05u) != 0u) {   // the whole host or the data circuit
                    data_.clear();
                    data_index_ = 0;
                    data_pending_ = false;
                }
                // Self clearing: the bits read back zero once the reset is done - and
                // they have to be cleared in the *stored* word as well.  The write that
                // triggered the reset is merged into this word, and re-registering the
                // reset byte set the very bit the caller polls.
                RegisterBlock::write(base_ + kSoftwareReset, 0, 0);
                store(kClockControl, peek(kClockControl) & ~(static_cast<u64>(0x07u) << 24));
            }
            return;
        }
        case kPowerControl:
            store(kPowerControl, value32 & 0x0F);
            return;
        default:
            return;
    }
}

u8 Sdif::read_data_port() {
    if (!data_pending_ || data_index_ >= data_.size()) return 0;
    const u8 byte = data_[data_index_++];
    if (data_index_ >= data_.size()) {
        if (sdif_trace()) {
            std::fprintf(stderr, "[sdif] port drained %zu bytes (read)\n", data_.size());
        }
        data_pending_ = false;
        data_ready_in_ = 0;  // cancel a data-ready event that has not fired yet
        // Transfer Complete follows the last data byte by a few clock cycles.
        // The CMeP second loader acknowledges the interrupt status by writing
        // 0x32 to 0x30, which clears bits 1, 4 and 5 in one go: raising Transfer
        // Complete in the same instant as the last data-ready flag meant the
        // driver wiped it before it could ever see it and then waited forever for
        // a transfer end that had already been reported.
        transfer_complete_in_ = 16;
    }
    return byte;
}

void Sdif::write_data_port(u8 value) {
    if (!data_pending_) return;
    if (data_index_ < data_.size()) data_[data_index_++] = value;
    if (data_index_ >= data_.size()) {
        // Push whatever the driver wrote into the card.
        // The block count lives in the high half of the block size word (see the
        // note in execute_command()).
        const u32 block_word = static_cast<u32>(peek(kBlockSize));
        const u32 blocks = (block_word >> 16) & 0xFFFF;
        const bool multi = (peek(kTransferMode) & kTmMultiBlock) != 0;
        const u32 count = multi ? blocks : 1;
        const bool ok = card_ ? card_->write_blocks(EmmcPartition::User, current_lba(), count, data_.data()) : false;
        data_pending_ = false;
        last_count_ = count;
        ++transfers_;
        store(kNormalIntStatus, peek(kNormalIntStatus) | (ok ? (kIntTransferComplete | kIntBufferWriteReady) : 0));
    }
}

// ---------------------------------------------------------------------------
// Command execution
// ---------------------------------------------------------------------------

u64 Sdif::current_lba() const {
    const u32 raw = static_cast<u32>(peek(kArgument));
    // The eMMC reports OCR bit 30 (card capacity status), so per the SD/MMC
    // specification the CMD17/18/24/25 argument *is* the block address, not a
    // byte address: the CMeP second loader reads ARG = 0x200 and the block it
    // then validates (0x46632 requires the first 64 bytes to be 0xFFF5 repeated)
    // is the idstorage leaf at byte 0x40000 = block 0x200 - exactly what the
    // reference eMMC dump carries at that offset. ZLB_SDIF_BYTE_ARG=1 restores
    // the byte addressed reading for experiments.
    static const bool byte_argument = [] {
        const char* value = std::getenv("ZLB_SDIF_BYTE_ARG");
        return value != nullptr && value[0] != '0';
    }();
    if (byte_argument) return raw / 512u;
    return raw;
}

bool Sdif::use_adma() const {
    return (peek(kTransferMode) & kTmDmaEnable) != 0;
}

void Sdif::execute_command() {
    // Registers that share a 32 bit word with a sibling are stored under the
    // *word* offset (RegisterBlock merges a sub-word write into the word), so
    // they have to be read back through peek() at the word offset: the command
    // register lives in the high half of the transfer mode word at 0x0C and the
    // block count in the high half of the block size word at 0x04. Reading
    // peek(kCommand) returned zero for every driver that writes the command with
    // a 16 bit access, which made the CMeP second loader's CMD1 decode as
    // GO_IDLE_STATE and its response come back as zero.
    const u32 transfer_word = static_cast<u32>(peek(kTransferMode));
    const u32 block_word = static_cast<u32>(peek(kBlockSize));
    const u16 command = static_cast<u16>((transfer_word >> 16) & 0x3FFF);
    const u8 index = static_cast<u8>((command >> kCmdIndexShift) & 0x3F);
    const u32 argument = static_cast<u32>(peek(kArgument));
    const bool has_data = (command & kCmdDataPresent) != 0;
    const bool read = (transfer_word & kTmRead) != 0;
    const u32 block_size = static_cast<u32>(block_word & 0x0FFF);
    u32 blocks = static_cast<u32>((block_word >> 16) & 0xFFFF);
    const bool multi = (transfer_word & kTmMultiBlock) != 0;
    if (!multi || blocks == 0) blocks = 1;
    if (block_size == 0) blocks = 0;

    command_ok_ = true;
    response_.fill(0);
    last_lba_ = current_lba();
    last_count_ = blocks;

    if (app_cmd_) {
        last_command_ = acmd_name(index);
    } else {
        last_command_ = cmd_name(index);
    }

    if (sdif_trace()) {
        const u32 cmep_pc = dma_secondary_ != nullptr ? dma_secondary_->context.pc : 0;
        const u32 arm_pc = dma_primary_ != nullptr ? dma_primary_->context.pc : 0;
        std::fprintf(stderr,
                     "[sdif] pc=%08X CMD idx=%u arg=0x%08X tm=0x%04X block=0x%08X size=%u count=%u "
                     "data=%d read=%d multi=%d dma=%d adma=0x%08X\n",
                     cmep_pc, index, argument, transfer_word & 0xFFFF, block_word, block_size, blocks,
                     has_data ? 1 : 0, read ? 1 : 0, multi ? 1 : 0, use_adma() ? 1 : 0,
                     static_cast<unsigned>(peek(kAdmaSystemAddress)));
    }

    auto respond_short = [this](u32 value) { response_[0] = value; };

    switch (index) {
        case 0:  // GO_IDLE_STATE: reset the card state machine.
            app_cmd_ = false;
            op_cond_polls_ = 0;
            respond_short(0x00000000);
            break;
        case 1:  // SEND_OP_COND / ACMD41
            // Bit 31 of the OCR is "power up still in progress". The CMeP second
            // loader polls SEND_OP_COND and stops as soon as the answer has bit 31
            // set (0x46F70: `bgei $3,0x0,<retry>` keeps asking while the value is
            // positive), so the card reports busy on the first call and ready on
            // every later one - which is what a real MMC does anyway.
            if (op_cond_polls_++ == 0) {
                respond_short(0xC0FF8000u);
            } else {
                respond_short(0x40FF8000u);  // ready, high capacity
            }
            break;
        case 5: {  // IO_SEND_OP_COND - CMD5 only exists on an SDIO card.
            // Round 303 (measured): NSKBL's storage driver probes with CMD5 after
            // CMD8 and only *falls back* to the MMC/SD identification when the probe
            // fails: its wrapper 0x51022DE4 loops while the answer is negative and
            // 0x51023012 takes the `cbnz r0` error branch, which leads to
            // 0x51022ED8 (the CID/relative-address stage) and finally sets the
            // "identified" flag [dev+0x2410] = 1 that the open checks at 0x5102074A.
            // Answering instead (an earlier guess, 0xC0FF8000) made 0x51023034 see
            // bit 27 clear, take 0x51023040 and leave [dev+0x2410] = 3, so the open
            // failed with 0x80320017.  The card behind this controller is an eMMC,
            // i.e. not SDIO, so it must not answer.
            command_ok_ = false;
            break;
        }
        case 2: {  // ALL_SEND_CID
            if (card_) {
                const auto& cid = card_->cid();
                for (u32 i = 0; i < 4; ++i) {
                    response_[i] = (static_cast<u32>(cid[i * 4 + 0]) << 24) |
                                   (static_cast<u32>(cid[i * 4 + 1]) << 16) |
                                   (static_cast<u32>(cid[i * 4 + 2]) << 8) | static_cast<u32>(cid[i * 4 + 3]);
                }
            }
            break;
        }
        case 3:  // SEND_RELATIVE_ADDR (SD) / SET_RELATIVE_ADDR (MMC)
            // MMC puts the RCA it wants in the argument and expects an R1 status
            // back; SD sends a zero argument and expects R6 with the card's RCA.
            // The CMeP boot chain is MMC: its CMD3 carries 0x00010000 (the trace
            // shows Argument>>9 = 128) and its error decoder (0x47DE2, mask
            // 0xFDFFE008 - bits 13..23 are status error bits) rejected the RCA we
            // used to return in the high half as error 0x80320110.
            if ((argument & 0xFFFF0000u) != 0) {
                rca_ = argument & 0xFFFF0000u;
                respond_short(0x00000900u);  // READY_FOR_DATA | CURRENT_STATE=transfer
            } else {
                rca_ = 0x00010000u;
                respond_short(rca_);
            }
            if (card_) card_->set_relative_address(static_cast<u16>(rca_ >> 16));
            break;
        case 7:  // SELECT_CARD
            rca_ = argument & 0xFFFF0000u;
            respond_short(0x00000900u);  // READY_FOR_DATA | CURRENT_STATE=transfer
            break;
        case 8: {  // CMD8 is *two different commands* depending on the bus mode.
            // Round 302 (measured): MMC uses CMD8 = SEND_EXT_CSD (argument 0, a 512
            // byte data phase), which is what the CMeP boot chain issues
            // (`CMD idx=8 arg=0x0 tm=0x0010 data=1`).  SD uses CMD8 = SEND_IF_COND
            // (no data phase, R7 echoes the 0x1AA-style check pattern), which is how
            // NSKBL starts its card: its trace is `CMD idx=0 arg=0x0` then
            // `CMD idx=8 arg=0xAA tm=0x0000 data=0`, repeated three times, and then
            // the storage open fails with 0x80320160 (the SD init gives up).
            // With the ext-CSD path chosen for both, the SD caller never received the
            // R7 echo, so `os0:psp2bootconfig.skprx` was never opened.
            if (!has_data) {
                // SD SEND_IF_COND.  A card only answers when it accepts the voltage
                // range the host asks for (VHS = argument bits 11:8); NSKBL sends
                // 0xAA (VHS = 0) for a device whose identify left the flags at 0x80
                // and 0x1AA (VHS = 1) for an SD one.  The two cases have to differ:
                // its wrapper 0x51022D84 turns any answer that is not the pure echo
                // into 0x80320161, while the caller 0x51022FF6 accepts only 0 (SD,
                // echo answered) or 0x80320002 (engine error, i.e. no response) and
                // continues to the op-cond phase for both.  So the VHS-less request
                // times out instead of being answered, exactly like the MMC card
                // behind this controller - and once the SDHCI error interrupt status
                // became readable (see error_status_) the driver's error mapper
                // 0x5101EBE4 turns that timeout into the 0x80320002 its caller wants.
                // Measured effect: the ARM's SD command trace grows from 6 commands
                // (CMD0/CMD8 x3, then the open gives up) to 24 (CMD0, five CMD8
                // retries, CMD5 op-cond ...) - the default run is unchanged.
                // ZLB_SDIF_CMD8_NORESP=0 restores the old "always echo" answer.
                static const bool cmd8_no_response = [] {
                    const char* value = std::getenv("ZLB_SDIF_CMD8_NORESP");
                    return value == nullptr || value[0] != '0';
                }();
                if (cmd8_no_response && (argument & 0xF00u) == 0u) {
                    command_ok_ = false;
                    break;
                }
                respond_short(argument);   // R7: echo the check pattern
                break;
            }
            if (card_) {
                const auto& ext = card_->ext_csd();
                data_.assign(ext.begin(), ext.end());
            }
            if (data_.size() < 512) data_.resize(512, 0);
            data_.resize(512);  // exactly one block: the driver's buffer is sized for it
            data_index_ = 0;
            data_pending_ = true;
            read_direction_ = true;
            break;
        }
        case 9: {  // SEND_CSD
            if (card_) {
                const auto& csd = card_->csd();
                for (u32 i = 0; i < 4; ++i) {
                    response_[i] = (static_cast<u32>(csd[i * 4 + 0]) << 24) |
                                   (static_cast<u32>(csd[i * 4 + 1]) << 16) |
                                   (static_cast<u32>(csd[i * 4 + 2]) << 8) | static_cast<u32>(csd[i * 4 + 3]);
                }
            }
            break;
        }
        case 12:  // STOP_TRANSMISSION
            respond_short(0x00000900u);
            break;
        case 13:  // SEND_STATUS
            respond_short(0x00000900u);  // READY_FOR_DATA, TRAN state
            break;
        case 16:  // SET_BLOCKLEN
            respond_short(0x00000900u);
            break;
        case 17:  // READ_SINGLE_BLOCK
        case 18:  // READ_MULTIPLE_BLOCK
            if (has_data && card_) {
                const u32 count = index == 17 ? 1 : blocks;
                data_.assign(static_cast<size_t>(count) * 512u, 0);
                const bool ok = card_->read_blocks(EmmcPartition::User, current_lba(), count, data_.data());
                // Round 277 diagnostic: NSKBL's file lookup (0x510232EC) reports "not found"
                // for os0:psp2bootconfig.skprx although the entry is in the image, so log the
                // sectors the guest actually reads while it searches.  ZLB_EMMC_LOG=1.
                static const bool log_reads = [] {
                    const char* value = std::getenv("ZLB_EMMC_LOG");
                    return value != nullptr && value[0] != '0';
                }();
                if (log_reads) {
                    static unsigned logged = 0;
                    if (logged < 400u) {
                        ++logged;
                        ZLB_LOG_INFO("mmc", "read lba=%u count=%u ok=%d", current_lba(), count, ok ? 1 : 0);                    }
                }

                command_ok_ = ok;
                data_index_ = 0;
                data_pending_ = true;
                read_direction_ = true;
                last_count_ = count;
                ++transfers_;
                if (use_adma() && adma_transfer(true, data_, nullptr)) {
                    // The card's bytes are already in guest memory: nothing is
                    // left in the FIFO, and Transfer Complete follows the table
                    // walk instead of a data port drain (see tick()).
                    data_pending_ = false;
                    dma_complete_ = true;
                    transfer_complete_in_ = 32;
                }
            }
            respond_short(0x00000900u);
            break;
        case 24:  // WRITE_BLOCK
        case 25:  // WRITE_MULTIPLE_BLOCK
            if (has_data) {
                const u32 count = index == 24 ? 1 : blocks;
                data_.assign(static_cast<size_t>(count) * 512u, 0);
                data_index_ = 0;
                data_pending_ = true;
                read_direction_ = false;
                last_count_ = count;
                if (use_adma() && card_) {
                    // ADMA writes: the descriptor table points at the data in
                    // guest memory, so pull it out and push it to the card.
                    std::vector<u8> staging;
                    if (adma_transfer(false, data_, &staging) && staging.size() >= 512u) {
                        staging.resize(static_cast<size_t>(count) * 512u, 0);
                        command_ok_ = card_->write_blocks(EmmcPartition::User, current_lba(), count,
                                                          staging.data());
                        data_ = staging;
                        data_pending_ = false;
                        ++transfers_;
                        dma_complete_ = true;
                        transfer_complete_in_ = 32;
                    }
                }
            }
            respond_short(0x00000900u);
            break;
        case 55:  // APP_CMD
            app_cmd_ = true;
            respond_short(0x00000900u);
            break;
        case 51:  // SEND_SCR (and ACMD51)
            data_.assign(8, 0);
            data_[0] = 0x02;
            data_[1] = 0x25;
            data_index_ = 0;
            data_pending_ = true;
            read_direction_ = true;
            respond_short(0x00000900u);
            break;
        default:
            // Unknown commands complete with a generic "ready" status so a
            // polling driver does not spin forever.
            respond_short(0x00000900u);
            break;
    }

    if (index != 55) app_cmd_ = false;

    u16 status = kIntCmdComplete;
    // The data phase follows the command by a few clock cycles on real hardware:
    // the driver sees Command Complete first, acknowledges it, and only then
    // "Buffer Read/Write Ready". Raising both at the same instant let the CMeP
    // second loader acknowledge Command Complete and then wait forever for a data
    // event that had already been consumed (its command engine polls 0x30/0x32 in
    // a loop at 0x47914).
    if (has_data && data_pending_) {
        data_ready_in_ = 8;
    }
    // An R1b command (the response type is "48 bit with busy", command-register
    // bits 0..1 == 3) leaves the card driving DAT0 low after the response.  There is
    // no real busy time in the model, but the driver still waits for the *second*
    // event that ends it: NSKBL's request engine completes a request only when the
    // SDHCI reports another interrupt after Command Complete (its status mapper
    // 0x5101EBE4 returns early while bit 3 of the request status word is set, which
    // is exactly the R1b case), so CMD6 (SWITCH, `0x61B`) parked the whole storage
    // open at the status poll 0x5101EDE4.  Report Transfer Complete once the busy
    // would have ended.
    const bool r1b_busy = (command & kCmdResponseMask) == kCmdResponseMask;
    if (command_ok_ && r1b_busy && !has_data && transfer_complete_in_ == 0) {
        transfer_complete_in_ = 8;
    }
    if (sdif_trace()) {
        std::fprintf(stderr, "[sdif] raise CmdComplete ok=%d data_pending=%d tc_in=%u\n", command_ok_ ? 1 : 0,
                     data_pending_ ? 1 : 0, transfer_complete_in_);
    }
    if (command_ok_) {
        poke(kNormalIntStatus, peek(kNormalIntStatus) | status);
    } else {
        // Defer the error the way the data phase is deferred: on hardware the
        // interrupt arrives after the driver has posted the request it belongs to,
        // and the driver's completion path (0x5101EBE4) only maps the error status
        // into its own code while [dev+0x2428] still names that request.
        command_error_in_ = [] {
            const char* value = std::getenv("ZLB_SDIF_ERR_DELAY");
            return value != nullptr ? static_cast<u32>(std::strtoul(value, nullptr, 10)) : 8u;
        }();
    }
    // The model executes a command synchronously, so the command inhibit bits
    // have to clear immediately: the CMeP second loader polls Present State bit 0
    // between command issues (0x477E8) and a fixed inhibit window longer than its
    // retry loop left the bit set forever. The DAT line stays "active" while a
    // data phase is still waiting to be drained (data_pending_).

    busy_left_ = 0;
    update_irq();
}

bool Sdif::adma_transfer(bool read, const std::vector<u8>& payload, std::vector<u8>* sink) {
    // ADMA2 (SD Host Controller Simplified Specification 2.0, section 2.2.13).
    // The CMeP second loader builds this table itself - 0x48094 fills 8 byte
    // records with attribute 0x21 ("valid, transfer") and marks the last one
    // 0x23 - and then programs the table base into ADMA_SYSTEM_ADDRESS (0x58)
    // through the path at 0x48158, which also sets transfer mode bit 0 (DMA
    // enable). A driver that uses ADMA never touches the 0x20 data port, so the
    // host has to move the bytes itself.
    const u32 table = dma_translate(static_cast<u32>(peek(kAdmaSystemAddress)));
    adma_address_ = table;
    adma_bytes_ = 0;
    adma_ok_ = false;

    Bus* bus = dma_bus_for(table, 8);
    if (bus == nullptr || table == 0) {
        if (sdif_trace()) {
            std::fprintf(stderr, "[sdif] adma: no bus for table 0x%08X (primary=%p secondary=%p)\n", table,
                         static_cast<void*>(dma_primary_), static_cast<void*>(dma_secondary_));
        }
        return false;
    }

    u32 consumed = 0;
    u32 record = table;
    for (int guard = 0; guard < 4096; ++guard) {
        if (bus->first_unmapped(record, 8) != 0) {
            if (sdif_trace()) std::fprintf(stderr, "[sdif] adma: record 0x%08X unmapped\n", record);
            return false;
        }
        u8 desc[8];
        bus->read_bytes(record, desc, 8);
        const u16 attribute = static_cast<u16>(desc[0] | (desc[1] << 8));
        const u16 length = static_cast<u16>(desc[2] | (desc[3] << 8));
        const u32 target = static_cast<u32>(desc[4]) | (static_cast<u32>(desc[5]) << 8) |
                           (static_cast<u32>(desc[6]) << 16) | (static_cast<u32>(desc[7]) << 24);

        if (sdif_trace()) {
            std::fprintf(stderr, "[sdif] adma rec @0x%08X attr=0x%04X len=%u target=0x%08X (%u of %u moved)\n",
                         record, attribute, length, target, adma_bytes_,
                         static_cast<unsigned>(payload.size()));
        }
        if ((attribute & 0x1u) == 0) {
            // No valid bit: the walk stops and the error status latches.
            if (sdif_trace()) {
                std::fprintf(stderr, "[sdif] adma: record 0x%08X not valid (attr=0x%04X)\n", record, attribute);
            }
            return false;
        }
        // Bits [5:4] are ACT: 0b10 transfers data, 0b00 is a no-op (and 0b11 a
        // link to another table, which no Vita driver uses).
        //
        // Length 0 means a full 64 KiB block, not an empty record: the second
        // loader's multi-block read of the SELF header area builds the table
        // `{0x21, 0, 0x5FFC0000} {0x21, 0, 0x5FFD0000} {0x21, 0, 0x5FFE0000}
        // {0x23, 48640, 0x5FFF0000}` whose targets are exactly 64 KiB apart, and
        // 3*65536 + 48640 equals the transfer size CMD23 asked for (479 blocks =
        // 245248 bytes).  Treating those records as empty stopped the walk after
        // 48640 bytes, the short transfer made the driver retry forever (round
        // 153-10 in docs/SYSCON.md).
        const u32 transfer_length = length == 0 ? 0x10000u : length;
        const u32 physical_target = dma_translate(target);
        if (((attribute >> 4) & 0x3u) == 0x2u && transfer_length != 0) {
            if (bus->first_unmapped(physical_target, transfer_length) != 0) {
                if (sdif_trace()) {
                    std::fprintf(stderr, "[sdif] adma: target 0x%08X len %u unmapped\n", target,
                                 transfer_length);
                }
                return false;
            }
            if (read) {
                if (static_cast<size_t>(consumed) + transfer_length > payload.size()) return false;
                bus->write_bytes(physical_target, payload.data() + consumed, transfer_length);
            } else {
                const size_t base = sink->size();
                sink->resize(base + transfer_length);
                bus->read_bytes(physical_target, sink->data() + base, transfer_length);
            }
            consumed += transfer_length;
            adma_bytes_ += transfer_length;
        }
        if ((attribute & 0x2u) != 0) break;  // END: last record of the table
        record += 8;
    }
    adma_ok_ = read ? consumed == payload.size() : adma_bytes_ != 0;
    if (sdif_trace()) {
        std::fprintf(stderr, "[sdif] adma table=0x%08X moved=%u/%u ok=%d\n", table, adma_bytes_,
                     static_cast<unsigned>(payload.size()), adma_ok_ ? 1 : 0);
    }
    return adma_ok_;
}

u32 Sdif::dma_translate(u32 address) const {
    // Round 305-307 (measured): NSKBL's non-secure window is mapped identity in the guest's
    // own page tables (`L1[0x510] = 0x5111158E`, VA 0x51000000 -> PA 0x51000000), but its
    // SD driver hands the DMA engine the *bus* address of that window, which is the address
    // with the window byte dropped: the request object at VA 0x5117D400 carries
    // `[request+0x7C] = 0x0017D480` for its ADMA table at VA 0x5117D480 and builds its
    // records with targets 0x0017D5C0 and 0x00033140 for the buffers at VA 0x5117D5C0 and
    // VA 0x51033140 - i.e. `address & 0x00FFFFFF`.
    //
    // Everything below 0x01000000 is window relative, and that has to be decided by the
    // *address*, not by whether a bus happens to serve it: 0x00033140 lies inside the model's
    // own low boot window (the ARM boot ROM alias at PA 0..0x3FFFF), so asking the bus first
    // left that record untranslated and the driver's cache line stayed zero while its
    // partition scan read no records at all (the open then failed with 0x803FF007 again).
    // Both the table base and every record target go through here; addresses at or above
    // 0x01000000 are used as they are (the CMeP's tables live at 0x40000400).
    if (address < 0x01000000u) {
        const u32 windowed = address + 0x51000000u;
        if (dma_bus_for(windowed, 8) != nullptr) return windowed;
    }
    return address;
}

Bus* Sdif::dma_bus_for(u32 address, u32 length) const {
    // "Usable" means plain memory on this bus: mapped, and not covered by a
    // device window. The ARM bus maps the whole 128 MiB DRAM window at
    // 0x40000000 as RAM, but its first 64 KiB is the MPCore peripheral block
    // (SCU/GIC), and Bus::find_device prefers a device over RAM - so the CMeP
    // second loader's ADMA2 table at 0x40000400 reads back as zero there while
    // the CMeP's own window returns what the loader wrote. Either bus moves the
    // same DRAM bytes, so the check only picks the one that can see them.
    auto usable = [address, length](Bus* bus) {
        if (bus == nullptr) return false;
        if (bus->first_unmapped(address, length) != 0) return false;
        for (u32 offset = 0; offset < length; ++offset) {
            if (bus->find_device(address + offset) != nullptr) return false;
        }
        return true;
    };
    if (usable(dma_primary_)) return dma_primary_;
    if (usable(dma_secondary_)) return dma_secondary_;
    return nullptr;
}

void Sdif::finish_transfer(bool ok) {
    data_pending_ = false;
    poke(kNormalIntStatus, peek(kNormalIntStatus) |
                                (ok ? (kIntTransferComplete | kIntDma) : 0));
    update_irq();
}

void Sdif::tick(u64 cycles) {
    if (command_error_in_ != 0) {
        // A failed command reports its error interrupt status a few ticks after the
        // command register write, so the driver has its request in place when the
        // error is mapped (see execute_command()).
        if (cycles >= command_error_in_) {
            command_error_in_ = 0;
            error_status_ = static_cast<u16>(error_status_ | 0x0001);
            if (sdif_trace()) {
                std::fprintf(stderr, "[sdif] %s raise CommandError -> error_status_=0x%04X\n",
                             name_.c_str(), static_cast<unsigned>(error_status_));
            }
            update_irq();
        } else {
            command_error_in_ -= static_cast<u32>(cycles);
        }
    }
    if (transfer_complete_in_ != 0) {
        // The data phase finished; report it a few ticks later (see
        // read_data_port()).
        if (cycles >= transfer_complete_in_) {
            transfer_complete_in_ = 0;
            if (sdif_trace()) {
                std::fprintf(stderr, "[sdif] raise TransferComplete dma=%d\n", dma_complete_ ? 1 : 0);
            }
            poke(kNormalIntStatus,
                 peek(kNormalIntStatus) | kIntTransferComplete | (dma_complete_ ? kIntDma : 0));
            dma_complete_ = false;
            data_ready_in_ = 0;
            update_irq();
        } else {
            transfer_complete_in_ -= static_cast<u32>(cycles);
        }
    }
    if (data_ready_in_ != 0) {
        // The data phase becomes visible a few PERIPHCLK ticks after the command
        // completed (see execute_command()).
        if (cycles >= data_ready_in_) {
            data_ready_in_ = 0;
            // Only announce the data phase while there is data left: the driver
            // may already have read the whole buffer by the time this fires.
            if (data_pending_) {
                const u16 ready =
                    static_cast<u16>(read_direction_ ? kIntBufferReadReady : kIntBufferWriteReady);
                poke(kNormalIntStatus, peek(kNormalIntStatus) | ready);
            }
            update_irq();
        } else {
            data_ready_in_ -= static_cast<u32>(cycles);
        }
    }
    if (busy_left_ == 0) return;
    if (cycles >= busy_left_) {
        busy_left_ = 0;
    } else {
        busy_left_ -= static_cast<u32>(cycles);
    }
}

void Sdif::update_irq() {
    const u16 normal = static_cast<u16>(peek(kNormalIntStatus));
    const u16 normal_signal = static_cast<u16>(peek(kNormalIntSignalEnable));
    const u16 error = error_status_;
    const u16 error_signal = static_cast<u16>(peek(kErrorIntSignalEnable));
    const bool want = ((normal & normal_signal) != 0) || ((error & error_signal) != 0);
    // Round 340 diagnostic: nothing in the guest enables the GIC, so the ARM never
    // takes an interrupt; this log shows whether the controller asserts its own line
    // at all (and with which status/enable words).  ZLB_SDIF_IRQ_LOG=1.
    static const bool log_irq = [] {
        const char* value = std::getenv("ZLB_SDIF_IRQ_LOG");
        return value != nullptr && value[0] != '0';
    }();
    if (log_irq && want != irq_line_) {
        std::fprintf(stderr,
                     "[sdif] irq %s: normal=0x%04X n_enable=0x%04X error=0x%04X e_enable=0x%04X\n",
                     want ? "assert" : "drop", normal, normal_signal, error, error_signal);
    }
    if (want == irq_line_) return;
    irq_line_ = want;
    if (irq_callback_) irq_callback_(static_cast<u32>(Irq::Emmc), irq_line_);
}

std::string Sdif::summary() const {
    return format("%s port%u %s cmd=%s lba=%llu count=%u transfers=%llu", name_.c_str(), port_,
                  card_ && card_->attached() ? "card" : "no card", last_command_.empty() ? "-" : last_command_.c_str(),
                  static_cast<unsigned long long>(last_lba_), last_count_,
                  static_cast<unsigned long long>(transfers_));
}

void Sdif::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s (SDIF port %u): %llu transfers, last %s lba=%llu blocks=%u", name_.c_str(), port_,
                           static_cast<unsigned long long>(transfers_), last_command_.c_str(),
                           static_cast<unsigned long long>(last_lba_), last_count_));
    lines.push_back(format("    PRESENT_STATE=0x%08X NORMAL_INT=0x%04X ERROR_INT=0x%04X irq=%s",
                           static_cast<unsigned>(peek(kPresentState)), static_cast<unsigned>(peek(kNormalIntStatus) & 0xFFFF),
                           static_cast<unsigned>(error_status_), irq_line_ ? "asserted" : "idle"));
    if (adma_address_ != 0 || adma_bytes_ != 0) {
        lines.push_back(format("    ADMA table=0x%08X moved=%u bytes %s", adma_address_, adma_bytes_,
                               adma_ok_ ? "ok" : "FAILED"));
    }
    if (card_) lines.push_back(format("    card: %s", card_->summary().c_str()));
}

}  // namespace zlb::kermit
