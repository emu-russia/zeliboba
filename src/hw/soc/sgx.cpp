// zeliboba - PowerVR SGX543MP4+ register window (model side).
//
// Task part 2 of the Live Area goal: give the machine the SGX window the GPU
// driver talks to.  The base address and the register offsets are the model's
// own contract (see the comment on kSgxBase in soc_internal.h): libgpu_es4.elf
// is a user library that reaches the GPU through kernel services, and the
// kernel side of that interface was not recovered from the firmware dump.  The
// behaviour modelled here is the part every PowerVR SGX driver uses:
//
//   * the window identifies itself (CORE_ID / CORE_REVISION),
//   * the driver points the block at a command queue (QUEUE_BASE / QUEUE_SIZE),
//   * the driver publishes how far it filled the queue (QUEUE_WRITE),
//   * a kick (write 1 into QUEUE_CONTROL) makes the block consume the queue and
//     report completion through EVENT_STATUS/IRQ_STATUS, which the enable mask
//     turns into the interrupt line.
//
// Completion is synchronous: the model has no shader pipeline yet, so a kick
// drains whatever the driver published immediately.  That is enough for the
// display/kernel bring-up path, and it is the hook the GXM command parser and
// the OpenGL translation will hang off (docs/GPU.md steps 3 and 4).
#include "hw/soc/soc_internal.h"

namespace zlb::kermit {

Sgx::Sgx(std::string name, u32 base, u32 size, Bus& bus)
    : RegisterBlock(std::move(name), base, size), bus_(bus) {
    define(kSgxCoreId, "SGX_CORE_ID", kSgxCoreIdValue);
    define(kSgxCoreRevision, "SGX_CORE_REVISION", kSgxCoreRevisionValue);
    define(kSgxCoreStatus, "SGX_CORE_STATUS", 0);
    define(kSgxEventStatus, "SGX_EVENT_STATUS", 0);
    define(kSgxEventClear, "SGX_EVENT_CLEAR", 0);
    define(kSgxEventEnable, "SGX_EVENT_ENABLE", 0);
    define(kSgxQueueBase, "SGX_QUEUE_BASE", 0);
    define(kSgxQueueSize, "SGX_QUEUE_SIZE", 0);
    define(kSgxQueueControl, "SGX_QUEUE_CONTROL", 0);
    define(kSgxQueueWrite, "SGX_QUEUE_WRITE", 0);
    define(kSgxQueueRead, "SGX_QUEUE_READ", 0);
    define(kSgxIrqStatus, "SGX_IRQ_STATUS", 0);
    define(kSgxIrqClear, "SGX_IRQ_CLEAR", 0);
}

void Sgx::reset() {
    RegisterBlock::reset();
    events_ = 0;
    event_enable_ = 0;
    irq_status_ = 0;
    queue_base_ = 0;
    queue_size_ = 0;
    queue_control_ = 0;
    queue_write_ = 0;
    queue_read_ = 0;
    kicks_ = 0;
    commands_ = 0;
    irq_line_ = false;
    if (irq_) irq_(0, false);
}

void Sgx::tick(u64 cycles) {
    (void)cycles;   // kicks complete synchronously for now (see the file header)
}

void Sgx::refresh_irq() {
    const bool level = (irq_status_ & event_enable_ & (kSgxEventTa | kSgxEvent3d | kSgxEventErr)) != 0;
    if (level != irq_line_) {
        irq_line_ = level;
        if (irq_) irq_(0, level);
    }
}

void Sgx::complete_kick() {
    // The driver published QUEUE_WRITE bytes of commands; consume them.  The bytes
    // are read out of guest memory here (the queue lives in DRAM), which is what
    // makes this a command-queue model rather than a set of counters: the first
    // words of every kick are kept for the debugger and for the GXM parser that
    // comes next (docs/GPU.md step 3).
    const u32 published = queue_write_;
    last_words_.clear();
    if (queue_size_ != 0u) {
        const u32 consumed = (published >= queue_read_) ? (published - queue_read_) : 0u;
        const u32 usable = (consumed > queue_size_) ? queue_size_ : consumed;
        queue_bytes_ += usable;
        commands_ += usable / 16u;   // the model counts 16-byte command units
        for (u32 offset = 0; offset + 4u <= usable && last_words_.size() < 8u; offset += 4u) {
            last_words_.push_back(bus_.read32(queue_base_ + queue_read_ + offset));
        }
    }
    queue_read_ = published;
    poke(kSgxQueueRead, queue_read_);
    events_ = kSgxEventTa | kSgxEvent3d;
    irq_status_ |= kSgxEventTa | kSgxEvent3d;
    poke(kSgxEventStatus, events_);
    poke(kSgxIrqStatus, irq_status_);
    refresh_irq();
}

u64 Sgx::read_word(u32 offset, u64 stored) {
    switch (offset) {
        case kSgxCoreId: return kSgxCoreIdValue;
        case kSgxCoreRevision: return kSgxCoreRevisionValue;
        case kSgxCoreStatus: return 0;
        case kSgxEventStatus: return events_;
        case kSgxQueueRead: return queue_read_;
        case kSgxIrqStatus: return irq_status_;
        default: return stored;
    }
}

void Sgx::write_word(u32 offset, u64 value) {
    switch (offset) {
        case kSgxEventClear:
            events_ &= ~static_cast<u32>(value);
            irq_status_ &= ~static_cast<u32>(value);
            poke(kSgxEventStatus, events_);
            poke(kSgxIrqStatus, irq_status_);
            refresh_irq();
            return;
        case kSgxEventEnable:
            event_enable_ = static_cast<u32>(value);
            refresh_irq();
            return;
        case kSgxQueueBase:
            queue_base_ = static_cast<u32>(value);
            return;
        case kSgxQueueSize:
            queue_size_ = static_cast<u32>(value);
            return;
        case kSgxQueueWrite:
            queue_write_ = static_cast<u32>(value);
            return;
        case kSgxQueueControl: {
            queue_control_ = static_cast<u32>(value);
            if ((queue_control_ & 1u) != 0u) {
                ++kicks_;
                complete_kick();
            }
            return;
        }
        case kSgxIrqClear:
            irq_status_ &= ~static_cast<u32>(value);
            poke(kSgxIrqStatus, irq_status_);
            refresh_irq();
            return;
        default:
            return;
    }
}

std::string Sgx::summary() const {
    return format("PowerVR SGX window: %llu kick(s), %llu command(s), %u event(s), IRQ %s",
                  static_cast<unsigned long long>(kicks_),
                  static_cast<unsigned long long>(commands_), events_,
                  irq_line_ ? "asserted" : "clear");
}

void Sgx::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("core id 0x%08X revision 0x%08X", kSgxCoreIdValue,
                           kSgxCoreRevisionValue));
    lines.push_back(format("queue base 0x%08X size %u write %u read %u", queue_base_,
                           queue_size_, queue_write_, queue_read_));
    lines.push_back(format("kicks %llu commands %llu events 0x%X enable 0x%X irq 0x%X",
                           static_cast<unsigned long long>(kicks_),
                           static_cast<unsigned long long>(commands_), events_,
                           event_enable_, irq_status_));
}

}  // namespace zlb::kermit
