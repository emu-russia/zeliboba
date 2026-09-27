// zeliboba - Kermit DMA controller.
//
// Base address: ASSUMPTION. dmacmgr.elf did not yield a register level
// description, so the controller lives in the free part of the ARM peripheral
// window that the C# reference model already reserves (0xE2000000-0xE20A0000).
// The base is a named constant in soc_internal.h (kDmaBase) so it is a one line
// change once the real offsets are recovered.
//
// Register layout (per channel, stride 0x20, 4 channels, block at 0x100):
//
//   0x00  SRC        source address
//   0x04  DST        destination address
//   0x08  COUNT      number of bytes
//   0x0C  CONTROL    bit0 enable, bit1 direction (1 = memory to peripheral),
//                    bits[3:2] unit size, bit4 interrupt enable, bit8 start
//   0x10  STATUS     bit0 busy, bit1 done, bit2 error, bit3 interrupt
//   0x14  NEXT       chained descriptor (not used by the model, kept in sync)
//   0x18  RESERVED
//   0x1C  RESERVED
//
// The globals are at 0x00: GLOBAL_CONTROL (bit0 enable, bit1 halt), 0x04
// GLOBAL_STATUS (bit0 any busy, bits[7:4] per channel done), 0x08 IRQ_STATUS
// and 0x0C IRQ_MASK.
//
// Every access goes through Bus::readN/writeN, so a channel whose source or
// destination is a device window talks to that device instead of the RAM
// behind it; the DMA engine never dereferences a host pointer. A transfer is
// executed in `tick()` at a documented rate (one unit per tick, 16 bytes per
// call) so that it is deterministic and interruptible.
#include "hw/soc/soc_internal.h"

namespace zlb::kermit {
namespace {

constexpr u32 kChSrc = 0x00;
constexpr u32 kChDst = 0x04;
constexpr u32 kChCount = 0x08;
constexpr u32 kChControl = 0x0C;
constexpr u32 kChStatus = 0x10;
constexpr u32 kChNext = 0x14;

constexpr u32 kCtlEnable = 1u << 0;
constexpr u32 kCtlDirection = 1u << 1;  // 1 = memory to peripheral
constexpr u32 kCtlUnitShift = 2;
constexpr u32 kCtlUnitMask = 0x3u << kCtlUnitShift;
constexpr u32 kCtlIrqEnable = 1u << 4;
constexpr u32 kCtlStart = 1u << 8;

constexpr u32 kStBusy = 1u << 0;
constexpr u32 kStDone = 1u << 1;
constexpr u32 kStError = 1u << 2;
constexpr u32 kStIrq = 1u << 3;

constexpr u32 kGlobalControl = 0x00;
constexpr u32 kGlobalStatus = 0x04;
constexpr u32 kIrqStatus = 0x08;
constexpr u32 kIrqMask = 0x0C;

constexpr u32 kBytesPerTick = 16;

}  // namespace

DmaController::DmaController(std::string name, u32 base, u32 size, Bus& bus, u32 channels)
    : Device(std::move(name), base, size), bus_(bus) {
    channels_.resize(channels);
    for (u32 i = 0; i < channels; ++i) {
        const u32 block = kChannelOffset + i * kChannelStride;
        names_[block + kChSrc] = format("CH%u_SRC", i);
        names_[block + kChDst] = format("CH%u_DST", i);
        names_[block + kChCount] = format("CH%u_COUNT", i);
        names_[block + kChControl] = format("CH%u_CONTROL", i);
        names_[block + kChStatus] = format("CH%u_STATUS", i);
        names_[block + kChNext] = format("CH%u_NEXT", i);
    }
    names_[kGlobalControl] = "GLOBAL_CONTROL";
    names_[kGlobalStatus] = "GLOBAL_STATUS";
    names_[kIrqStatus] = "IRQ_STATUS";
    names_[kIrqMask] = "IRQ_MASK";
}

void DmaController::reset() {
    for (auto& channel : channels_) channel = Channel{};
    transfers_ = 0;
    bytes_copied_ = 0;
    irq_ = false;
    irq_mask_ = 0;
    global_control_ = 0;
    if (irq_callback_) irq_callback_(static_cast<u32>(Irq::Dma0), false);
}

void DmaController::add_target(u32 base, u32 size, Device* device) {
    targets_.push_back(Target{base, size, device});
}

u64 DmaController::read(u32 address, unsigned size) {
    const u32 offset = address - base_;
    u32 value = 0;
    if (offset >= kChannelOffset && offset < kChannelOffset + kDmaChannels * kChannelStride) {
        const u32 index = (offset - kChannelOffset) / kChannelStride;
        const u32 reg = (offset - kChannelOffset) % kChannelStride;
        if (index >= channels_.size()) return 0;
        const Channel& channel = channels_[index];
        switch (reg & ~3u) {
            case kChSrc: value = channel.source; break;
            case kChDst: value = channel.dest; break;
            case kChCount: value = channel.count; break;
            case kChControl: value = channel.control; break;
            case kChStatus: value = channel.status | (channel.active ? kStBusy : 0u); break;
            default: value = 0; break;
        }
    } else {
        switch (offset & ~3u) {
            case kGlobalControl:
                value = global_control_;
                break;
            case kGlobalStatus: {
                u32 status = 0;
                for (u32 i = 0; i < channels_.size(); ++i) {
                    if (channels_[i].active) status |= 1u;
                    if (channels_[i].status & kStDone) status |= 1u << (4 + i);
                }
                value = status;
                break;
            }
            case kIrqStatus:
                value = irq_status();
                break;
            case kIrqMask:
                value = irq_mask_;
                break;
            default:
                value = 0;
                break;
        }
    }
    return extract_register_bytes(value, offset, 0, size);
}

void DmaController::write(u32 address, unsigned size, u64 value) {
    const u32 offset = address - base_;
    if (offset >= kChannelOffset && offset < kChannelOffset + kDmaChannels * kChannelStride) {
        const u32 index = (offset - kChannelOffset) / kChannelStride;
        const u32 reg = (offset - kChannelOffset) % kChannelStride;
        if (index >= channels_.size()) return;
        Channel& channel = channels_[index];
        u32 current = 0;
        switch (reg & ~3u) {
            case kChSrc: current = channel.source; break;
            case kChDst: current = channel.dest; break;
            case kChCount: current = channel.count; break;
            case kChControl: current = channel.control; break;
            case kChStatus: current = channel.status; break;
            default: current = 0; break;
        }
        const u32 merged = static_cast<u32>(merge_register_bytes(current, value, offset, 0, size));
        switch (reg & ~3u) {
            case kChSrc: channel.source = merged; break;
            case kChDst: channel.dest = merged; break;
            case kChCount: channel.count = merged; break;
            case kChControl:
                channel.control = merged;
                if (merged & kCtlStart) start(index);
                break;
            case kChStatus:
                // Write one to clear done/error/interrupt.
                channel.status &= ~(merged & (kStDone | kStError | kStIrq));
                update_irq();
                break;
            default: break;
        }
        return;
    }

    u32 merged = static_cast<u32>(merge_register_bytes(
        offset == kGlobalControl ? global_control_
                                 : (offset == kIrqMask ? irq_mask_ : 0u),
        value, offset, 0, size));
    switch (offset & ~3u) {
        case kGlobalControl:
            global_control_ = merged;
            break;
        case kGlobalStatus:
            for (u32 i = 0; i < channels_.size(); ++i) {
                if (value & (1ull << (4 + i))) channels_[i].status &= ~kStDone;
            }
            break;
        case kIrqStatus:
            for (u32 i = 0; i < channels_.size(); ++i) {
                if (value & (1ull << i)) channels_[i].status &= ~kStIrq;
            }
            update_irq();
            break;
        case kIrqMask:
            irq_mask_ = merged;
            update_irq();
            break;
        default:
            break;
    }
}

const char* DmaController::register_name(u32 address) const {
    if (!handles(address)) return nullptr;
    auto it = names_.find(address - base_);
    return it == names_.end() ? nullptr : it->second.c_str();
}

void DmaController::enumerate_registers(std::vector<RegisterInfo>& out) const {
    for (const auto& entry : names_) {
        RegisterInfo info;
        info.address = base_ + entry.first;
        info.name = entry.second;
        out.push_back(std::move(info));
    }
}

bool DmaController::peek_register(const std::string& name, u64& out) const {
    for (const auto& entry : names_) {
        if (entry.second != name) continue;
        out = const_cast<DmaController*>(this)->read(base_ + entry.first, 4);
        return true;
    }
    return false;
}

bool DmaController::poke_register(const std::string& name, u64 value) {
    for (const auto& entry : names_) {
        if (entry.second != name) continue;
        write(base_ + entry.first, 4, value);
        return true;
    }
    return false;
}

void DmaController::start(u32 index) {
    Channel& channel = channels_[index];
    if ((channel.control & kCtlEnable) == 0 || channel.count == 0) {
        channel.status = (channel.status | kStError) & ~kStBusy;
        update_irq();
        return;
    }
    channel.active = true;
    channel.status = (channel.status & ~(kStDone | kStError)) | kStBusy;
}

void DmaController::step(u32 index) {
    Channel& channel = channels_[index];
    if (!channel.active) return;

    const u32 unit = 1u << ((channel.control & kCtlUnitMask) >> kCtlUnitShift);
    u32 chunk = channel.count;
    if (chunk > kBytesPerTick) chunk = kBytesPerTick;
    chunk = (chunk + unit - 1) & ~(unit - 1);
    if (chunk > channel.count) chunk = channel.count;

    std::vector<u8> data;
    if (!read_source(data, channel.source, chunk)) {
        complete(index, false);
        return;
    }
    if (!write_dest(data, channel.dest)) {
        complete(index, false);
        return;
    }

    channel.source += chunk;
    channel.dest += chunk;
    channel.count -= chunk;
    bytes_copied_ += chunk;

    if (channel.count == 0) complete(index, true);
}

void DmaController::complete(u32 index, bool ok) {
    Channel& channel = channels_[index];
    channel.active = false;
    const bool notify = (channel.control & 0x10u) != 0;  // CTL interrupt enable
    if (ok) {
        channel.status = (channel.status & ~(kStBusy | kStError)) | kStDone;
        if (notify) channel.status |= kStIrq;
        ++transfers_;
    } else {
        channel.status = (channel.status & ~kStBusy) | kStError;
        if (notify) channel.status |= kStIrq;
    }
    update_irq();
}

u32 DmaController::irq_status() const {
    u32 status = 0;
    for (u32 i = 0; i < channels_.size(); ++i) {
        if (channels_[i].status & kStIrq) status |= 1u << i;
    }
    return status;
}

bool DmaController::read_source(std::vector<u8>& out, u32 address, u32 length) {
    out.resize(length);
    // A device window is read through the device so that a peripheral source
    // (the SDIF data port, the UART receive register) behaves correctly.
    for (const auto& target : targets_) {
        if (address < target.base || address + length > target.base + target.size) continue;
        for (u32 i = 0; i < length; ++i) {
            out[i] = static_cast<u8>(target.device->read(address + i, 1) & 0xFF);
        }
        return true;
    }
    const u32 first = bus_.first_unmapped(address, length);
    if (first != 0) return false;
    bus_.read_bytes(address, out.data(), length);
    return true;
}

bool DmaController::write_dest(const std::vector<u8>& data, u32 address) {
    for (const auto& target : targets_) {
        if (address < target.base || address + data.size() > target.base + target.size) continue;
        for (size_t i = 0; i < data.size(); ++i) target.device->write(address + static_cast<u32>(i), 1, data[i]);
        return true;
    }
    const u32 first = bus_.first_unmapped(address, data.size());
    if (first != 0) return false;
    bus_.write_bytes(address, data.data(), data.size());
    return true;
}

void DmaController::update_irq() {
    // The mask register suppresses the interrupt line only: the per channel
    // status bits stay visible in IRQ_STATUS either way.
    const bool want = (irq_status() & ~irq_mask_) != 0;
    if (want == irq_) return;
    irq_ = want;
    if (irq_callback_) irq_callback_(static_cast<u32>(Irq::Dma0), irq_);
}

void DmaController::tick(u64 cycles) {
    for (u32 i = 0; i < channels_.size(); ++i) {
        if (!channels_[i].active) continue;
        u64 budget = cycles;
        while (budget > 0) {
            step(i);
            --budget;
            if (!channels_[i].active) break;
        }
    }
}

std::string DmaController::summary() const {
    u32 active = 0;
    for (const auto& channel : channels_) {
        if (channel.active) ++active;
    }
    return format("%s %zu channels, %u active, %llu transfers, %s", name_.c_str(), channels_.size(), active,
                  static_cast<unsigned long long>(transfers_), irq_ ? "IRQ" : "idle");
}

void DmaController::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s: %llu channels, %llu transfers, %s copied", name_.c_str(),
                           static_cast<unsigned long long>(channels_.size()),
                           static_cast<unsigned long long>(transfers_),
                           human_size(bytes_copied_).c_str()));
    for (u32 i = 0; i < channels_.size(); ++i) {
        const Channel& channel = channels_[i];
        if (!channel.active && channel.status == 0 && channel.source == 0 && channel.dest == 0) continue;
        lines.push_back(format("    ch%u %s src=0x%08X dst=0x%08X left=%u status=0x%X", i,
                               channel.active ? "run " : "stop", channel.source, channel.dest, channel.count,
                               channel.status));
    }
}

}  // namespace zlb::kermit
