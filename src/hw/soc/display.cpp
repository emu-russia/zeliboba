// zeliboba - Kermit display controller.
//
// The panel is a Sony CXD5315GG OLED, 960x544 (see
// VitaTestSuite/Docs/cxd5315gg/). The SDL3 frontend only needs two things from
// this block: the currently scanned out framebuffer and a frame counter, so the
// model keeps a single active buffer in host memory and hands a pointer out
// through KermitBlock::framebuffer().
//
// Base address: ASSUMPTION. display.elf/oled.elf did not yield a register
// offset table, so the window lives in the free part of the ARM peripheral
// space (kDisplayBase in soc_internal.h).
//
// Register map (all 32 bit, offsets from the base):
//
//   0x00 DISPLAY_CONTROL   bit0 enable, bit1 frame start, bit2 vsync enable,
//                          bits[5:4] trigger source (0 = software)
//   0x04 DISPLAY_STATUS    bit0 ready, bit1 vsync pending, bit2 buffered flip
//   0x08 FRAMEBUFFER0      physical address of buffer 0 (DRAM)
//   0x0C FRAMEBUFFER1      physical address of buffer 1
//   0x10 STRIDE            bytes per line
//   0x14 SIZE              width in bits[15:0], height in bits[31:16]
//   0x18 FORMAT            0 = RGB565, 1 = RGBA8888
//   0x1C ACTIVE            buffer index in bits[1:0]
//   0x20 FLIP              write the buffer index to latch a flip
//   0x24 VSYNC_PERIOD      PERIPHCLK ticks between frames (default 16667 = 60 Hz)
//   0x28 FRAME_COUNTER     number of completed frames
//   0x2C LAST_SYNC         frame counter value of the last vsync
//   0x30 DMA_SOURCE        address the display engine reads from (debug)
//   0x34 DMA_CONTROL       bit0 start a fill/copy from DMA_SOURCE
#include "hw/soc/soc_internal.h"

namespace zlb::kermit {
namespace {

constexpr u32 kControl = 0x00;
constexpr u32 kStatus = 0x04;
constexpr u32 kBuffer0 = 0x08;
constexpr u32 kBuffer1 = 0x0C;
constexpr u32 kStride = 0x10;
constexpr u32 kSize = 0x14;
constexpr u32 kFormat = 0x18;
constexpr u32 kActive = 0x1C;
constexpr u32 kFlip = 0x20;
constexpr u32 kVsyncPeriod = 0x24;
constexpr u32 kFrameCounter = 0x28;
constexpr u32 kLastSync = 0x2C;
constexpr u32 kDmaSource = 0x30;
constexpr u32 kDmaControl = 0x34;

constexpr u32 kCtlEnable = 1u << 0;
constexpr u32 kCtlFrameStart = 1u << 1;
constexpr u32 kCtlVsyncEnable = 1u << 2;

constexpr u32 kStReady = 1u << 0;
constexpr u32 kStVsync = 1u << 1;
constexpr u32 kStFlipped = 1u << 2;

}  // namespace

DisplayController::DisplayController(std::string name, u32 base, u32 size, int width, int height)
    : Device(std::move(name), base, size), panel_width_(width), panel_height_(height) {
    names_[kControl] = "DISPLAY_CONTROL";
    names_[kStatus] = "DISPLAY_STATUS";
    names_[kBuffer0] = "FRAMEBUFFER0";
    names_[kBuffer1] = "FRAMEBUFFER1";
    names_[kStride] = "STRIDE";
    names_[kSize] = "SIZE";
    names_[kFormat] = "FORMAT";
    names_[kActive] = "ACTIVE";
    names_[kFlip] = "FLIP";
    names_[kVsyncPeriod] = "VSYNC_PERIOD";
    names_[kFrameCounter] = "FRAME_COUNTER";
    names_[kLastSync] = "LAST_SYNC";
    names_[kDmaSource] = "DMA_SOURCE";
    names_[kDmaControl] = "DMA_CONTROL";
    for (auto& buffer : buffers_) {
        buffer.width = panel_width_;
        buffer.height = panel_height_;
        buffer.stride = panel_width_ * 2;
        buffer.format = Format::Rgb565;
    }
    update_layout();
}

void DisplayController::update_layout() {
    for (auto& buffer : buffers_) {
        const size_t needed = static_cast<size_t>(buffer.stride) * static_cast<size_t>(buffer.height);
        if (buffer.pixels.size() != needed) buffer.pixels.assign(needed, 0);
    }
}

void DisplayController::reset() {
    control_ = 0;
    status_ = 0;
    active_ = 0;
    vsync_left_ = 0;
    frame_counter_ = 0;
    frames_ = 0;
    writes_ = 0;
    for (auto& buffer : buffers_) {
        buffer.address = 0;
        buffer.format = Format::Rgb565;
        buffer.width = panel_width_;
        buffer.height = panel_height_;
        buffer.stride = panel_width_ * 2;
        buffer.pixels.assign(static_cast<size_t>(buffer.stride) * buffer.height, 0);
    }
    update_layout();
}

u64 DisplayController::read(u32 address, unsigned size) {
    const u32 offset = (address - base_) & ~3u;
    u32 value = 0;
    switch (offset) {
        case kControl: value = control_; break;
        case kStatus: value = status_ | (control_ & kCtlEnable ? kStReady : 0u); break;
        case kBuffer0: value = buffers_[0].address; break;
        case kBuffer1: value = buffers_[1].address; break;
        case kStride: value = static_cast<u32>(buffers_[active_].stride); break;
        case kSize:
            value = (static_cast<u32>(buffers_[active_].width) & 0xFFFFu) |
                    ((static_cast<u32>(buffers_[active_].height) & 0xFFFFu) << 16);
            break;
        case kFormat: value = static_cast<u32>(buffers_[active_].format); break;
        case kActive: value = active_; break;
        case kVsyncPeriod: value = vsync_period_; break;
        case kFrameCounter: value = static_cast<u32>(frame_counter_); break;
        case kLastSync: value = static_cast<u32>(frames_); break;
        default: value = 0; break;
    }
    return extract_register_bytes(value, address, base_, size);
}

void DisplayController::write(u32 address, unsigned size, u64 value) {
    const u32 offset = (address - base_) & ~3u;
    const u32 value32 = static_cast<u32>(value);
    ++writes_;
    switch (offset) {
        case kControl:
            control_ = value32;
            if ((control_ & kCtlEnable) == 0) status_ &= ~kStReady;
            else status_ |= kStReady;
            if (control_ & kCtlVsyncEnable) vsync_left_ = vsync_period_;
            return;
        case kStatus:
            status_ &= ~value32;
            return;
        case kBuffer0:
        case kBuffer1: {
            Buffer& buffer = buffers_[offset == kBuffer0 ? 0 : 1];
            buffer.address = value32;
            update_layout();
            return;
        }
        case kStride: {
            const int stride = static_cast<int>(value32 == 0 ? panel_width_ * 2 : value32);
            buffers_[active_].stride = stride;
            update_layout();
            return;
        }
        case kSize: {
            const int width = static_cast<int>(value32 & 0xFFFFu);
            const int height = static_cast<int>((value32 >> 16) & 0xFFFFu);
            Buffer& buffer = buffers_[active_];
            buffer.width = width > 0 ? width : panel_width_;
            buffer.height = height > 0 ? height : panel_height_;
            update_layout();
            return;
        }
        case kFormat:
            buffers_[active_].format = value32 == 1 ? Format::Rgba8888 : Format::Rgb565;
            update_layout();
            return;
        case kActive:
            active_ = value32 & 1u;
            return;
        case kFlip:
            active_ = value32 & 1u;
            status_ |= kStFlipped;
            return;
        case kVsyncPeriod:
            vsync_period_ = value32 == 0 ? 16667u : value32;
            return;
        case kDmaSource:
            /* informational: the block reads guest memory only through the
               machine's DMA engine, never through a host pointer. */
            return;
        case kDmaControl:
            if (value32 & 1u) {
                status_ |= kStFlipped;
                scanout_from_guest();
            }
            return;
        default:
            return;
    }
}

void DisplayController::scanout_from_guest() {
    if (bus_ == nullptr) return;
    Buffer& buffer = buffers_[active_];
    if (buffer.address == 0u || buffer.pixels.empty()) return;
    const size_t stride = static_cast<size_t>(buffer.stride);
    const size_t needed = stride * static_cast<size_t>(buffer.height);
    if (buffer.pixels.size() < needed) buffer.pixels.resize(needed, 0);
    for (size_t row = 0; row < static_cast<size_t>(buffer.height); ++row) {
        const u32 source = buffer.address + static_cast<u32>(row * stride);
        u8* destination = buffer.pixels.data() + row * stride;
        for (size_t column = 0; column < stride; ++column) {
            destination[column] = bus_->read8(source + static_cast<u32>(column));
        }
    }
    ++scanouts_;
    ++frame_counter_;
    status_ |= kStVsync;
    if (frame_callback_) frame_callback_();
}

void DisplayController::tick(u64 cycles) {
    if ((control_ & kCtlEnable) == 0) return;
    if (vsync_period_ == 0) return;

    u64 ticks = cycles;
    while (ticks > 0) {
        if (vsync_left_ == 0) vsync_left_ = vsync_period_;
        if (ticks < vsync_left_) {
            vsync_left_ -= static_cast<u32>(ticks);
            return;
        }
        ticks -= vsync_left_;
        vsync_left_ = vsync_period_;
        ++frame_counter_;
        ++frames_;
        status_ |= kStVsync;
        if (frame_callback_) frame_callback_();
    }
}

const u8* DisplayController::framebuffer(int& width, int& height, int& stride,
                                       int* bytes_per_pixel) const {
    if ((control_ & kCtlEnable) == 0) {
        width = height = stride = 0;
        if (bytes_per_pixel) *bytes_per_pixel = 0;
        return nullptr;
    }
    const Buffer& buffer = buffers_[active_];
    width = buffer.width;
    height = buffer.height;
    stride = buffer.stride;
    // Row padding is independent of the pixel format. In particular, stride /
    // width can describe neither RGB565 nor RGBA8888 for a padded framebuffer.
    if (bytes_per_pixel) {
        *bytes_per_pixel = static_cast<int>(DisplayController::bytes_per_pixel(buffer.format));
    }
    return buffer.pixels.empty() ? nullptr : buffer.pixels.data();
}

const char* DisplayController::register_name(u32 address) const {
    if (!handles(address)) return nullptr;
    auto it = names_.find((address - base_) & ~3u);
    return it == names_.end() ? nullptr : it->second.c_str();
}

void DisplayController::enumerate_registers(std::vector<RegisterInfo>& out) const {
    for (const auto& entry : names_) {
        RegisterInfo info;
        info.address = base_ + entry.first;
        info.name = entry.second;
        out.push_back(std::move(info));
    }
}

bool DisplayController::peek_register(const std::string& name, u64& out) const {
    for (const auto& entry : names_) {
        if (entry.second != name) continue;
        out = const_cast<DisplayController*>(this)->read(base_ + entry.first, 4);
        return true;
    }
    return false;
}

bool DisplayController::poke_register(const std::string& name, u64 value) {
    for (const auto& entry : names_) {
        if (entry.second != name) continue;
        write(base_ + entry.first, 4, value);
        return true;
    }
    return false;
}

std::string DisplayController::summary() const {
    const Buffer& buffer = buffers_[active_];
    return format("%s %dx%d %s %s frames=%llu", name_.c_str(), buffer.width, buffer.height,
                  buffer.format == Format::Rgb565 ? "RGB565" : "RGBA8888",
                  (control_ & kCtlEnable) ? "on" : "off", static_cast<unsigned long long>(frame_counter_));
}

void DisplayController::describe(std::vector<std::string>& lines) const {
    const Buffer& buffer = buffers_[active_];
    lines.push_back(format("  %s: panel %dx%d, active buffer %u %s %dx%d stride=%d", name_.c_str(), panel_width_,
                           panel_height_, active_, buffer.format == Format::Rgb565 ? "RGB565" : "RGBA8888",
                           buffer.width, buffer.height, buffer.stride));
    lines.push_back(format("    buffers: [0]=0x%08X [1]=0x%08X, %llu frames, %llu register writes",
                           buffers_[0].address, buffers_[1].address,
                           static_cast<unsigned long long>(frame_counter_),
                           static_cast<unsigned long long>(writes_)));
}

void DisplayController::save_state(StateWriter& writer) const {
    // panel_width_/panel_height_ are construction-time configuration; `bus_`
    // and the `names_` table are wiring, and `frame_callback_` is a host hook.
    writer.fixed(buffers_, [&](const Buffer& buffer) {
        writer.begin("Display.buffer");
        writer.put_u32(buffer.address);
        writer.put_u32(static_cast<u32>(buffer.format));
        writer.put_i32(buffer.stride);
        writer.put_i32(buffer.width);
        writer.put_i32(buffer.height);
        // The host pixel image is a raw buffer: its length is written first so
        // the reader can size the vector before the page bitmap is consumed.
        writer.put_u32(static_cast<u32>(buffer.pixels.size()));
        state_write_pages(writer, buffer.pixels.data(), buffer.pixels.size());
        writer.end();
    });
    writer.put_u32(active_);
    writer.put_u32(control_);
    writer.put_u32(status_);
    writer.put_u32(vsync_period_);
    writer.put_u32(vsync_left_);
    writer.put_u64(frame_counter_);
    writer.put_u64(frames_);
    writer.put_u64(writes_);
    writer.put_u64(scanouts_);
}

void DisplayController::load_state(StateReader& reader) {
    reader.fixed(buffers_, [&](Buffer& buffer) {
        reader.begin("Display.buffer");
        buffer.address = reader.get_u32();
        buffer.format = static_cast<Format>(reader.get_u32());
        buffer.stride = reader.get_i32();
        buffer.width = reader.get_i32();
        buffer.height = reader.get_i32();
        const u32 pixel_size = reader.get_u32();
        buffer.pixels.resize(pixel_size);
        if (pixel_size != 0) state_read_pages(reader, buffer.pixels.data(), buffer.pixels.size());
        reader.end();
    });
    active_ = reader.get_u32();
    control_ = reader.get_u32();
    status_ = reader.get_u32();
    vsync_period_ = reader.get_u32();
    vsync_left_ = reader.get_u32();
    frame_counter_ = reader.get_u64();
    frames_ = reader.get_u64();
    writes_ = reader.get_u64();
    scanouts_ = reader.get_u64();
}

}  // namespace zlb::kermit
