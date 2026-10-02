// Native firmware 1.04 IFTU0 scanout.
//
// lowio.elf's table at linked VA 0x81009498 maps the A/B planes at
// E5020000/E5021000 and shared control at E5022000. Its helper 0x810058DC
// writes the two input banks; display.elf 0x8100015E computes row padding as
// 4*(pixel_pitch-width). xerpi/vita-libbaremetal's iftu.c independently names
// shared +10/+18 as explicit bank selects and format 0x10 as A8B8G8R8.
//
// The reached cold boot prepares the opposite bank without an explicit select,
// then the native IRQ handler zero-ACKs +40 and writes +180=1 before replaying
// the old bank. Model the ordinary mode as one turnover at the next DSI frame
// per full-word +180=1 rearm. Continuous versus rearmed hardware behavior is
// unrecovered: this one-shot timing is an explicit bounded model choice.
// Raw +40 status encoding remains unknown and reads zero; an internal latch
// drives physical IRQ204/205 until the native full-word zero ACK. No guest
// pending object, descriptor validity or pixels participate in event timing.
#include "hw/soc/soc_internal.h"

#include <limits>

namespace zlb::kermit {
namespace {

constexpr u32 kPlaneStep = 0x1000;
constexpr u32 kShared = 0x2000;
constexpr u32 kBusControl = kShared + 0x00;
constexpr u32 kPlaneState = 0x04;
constexpr u32 kPlaneRun = 0x50;
constexpr u32 kPlaneSetup = 0x58;
constexpr u32 kPlaneArm = 0x180;
constexpr u32 kPlaneAck = 0x40;
constexpr u32 kBankBase = 0x200;
constexpr u32 kBankStep = 0x100;
constexpr u32 kBankAddress = 0x00;
constexpr u32 kBankFormat = 0x40;
constexpr u32 kBankWidth = 0x44;
constexpr u32 kBankHeight = 0x48;
constexpr u32 kBankBlank = 0x4C;
constexpr u32 kBankPadding = 0x54;
constexpr u32 kFormatAbgr8888 = 0x10;

u32 selector_offset(unsigned plane) { return kShared + 0x10 + plane * 8; }

}  // namespace

IftuController::IftuController(std::string name, u32 base, u32 size, Bus& bus)
    : RegisterBlock(std::move(name), base, size), bus_(bus) {
    // Keep the recovered stores available for native readback and diagnostics.
    // Names describing unknown registers deliberately carry their offset.
    for (unsigned plane = 0; plane < 2; ++plane) {
        const u32 local = plane * kPlaneStep;
        const char* label = plane == 0 ? "A" : "B";
        for (u32 offset : {0x04u, 0x40u, 0x50u, 0x58u, 0x80u, 0x84u, 0x88u,
                           0x8Cu, 0xA0u, 0x100u, 0x180u}) {
            define(local + offset, format("%s.REG_%03X", label, offset));
        }
        for (u32 offset = 0x104; offset <= 0x168; offset += 4) {
            define(local + offset, format("%s.REG_%03X", label, offset));
        }
        for (unsigned bank = 0; bank < 2; ++bank) {
            const u32 start = local + kBankBase + bank * kBankStep;
            const std::string prefix = format("%s.BANK%u.", label, bank);
            define(start + kBankAddress, prefix + "ADDRESS");
            define(start + 0x04, prefix + "ADDRESS1");
            define(start + 0x08, prefix + "ADDRESS2");
            define(start + kBankFormat, prefix + "FORMAT");
            define(start + kBankWidth, prefix + "WIDTH");
            define(start + kBankHeight, prefix + "HEIGHT");
            define(start + kBankBlank, prefix + "BLANK");
            define(start + kBankPadding, prefix + "ROW_PADDING");
            for (u32 offset : {0x20u, 0x24u, 0x28u, 0x58u, 0x60u, 0x64u,
                               0x68u, 0x6Cu, 0xA0u, 0xA4u, 0xA8u, 0xC0u,
                               0xC4u, 0xC8u, 0xCCu, 0xD0u, 0xD4u}) {
                define(start + offset, prefix + format("REG_%02X", offset));
            }
        }
    }
    define(kBusControl, "BUS_CONTROL");
    define(kShared + 0x04, "BUS_MODE");
    define(selector_offset(0), "A.BANK_SELECT");
    define(kShared + 0x14, "A.REG_14");
    define(selector_offset(1), "B.BANK_SELECT");
    define(kShared + 0x1C, "B.REG_1C");
    define(kShared + 0x20, "BLEND_CONTROL");
}

void IftuController::write(u32 address, unsigned size, u64 value) {
    // The only evidenced ACK/rearm accesses are full native word stores.
    // Byte/halfword writes retain storage where applicable, but cannot turn a
    // partial lane of an opaque control into an ACK or a frame submission.
    const std::array<bool, 2> was_ordinary = {ordinary_profile(0), ordinary_profile(1)};
    auto overlaps = [&](u32 offset) {
        const u64 start = static_cast<u64>(base_) + offset;
        return size != 0 && static_cast<u64>(address) < start + 4 &&
               static_cast<u64>(address) + size > start;
    };
    RegisterBlock::write(address, size, value);
    for (unsigned plane = 0; plane < 2; ++plane) {
        const u32 local = plane * kPlaneStep;
        if (overlaps(selector_offset(plane))) {
            const u64 selected = peek(selector_offset(plane));
            if (selected <= 1) active_bank_[plane] = static_cast<u32>(selected);
        }
        // These views stay hardware-owned even for cross-word byte accesses.
        store(local + kPlaneState, (peek(local + kPlaneState) & ~2ull) |
              (static_cast<u64>(active_bank_[plane]) << 1));
        store(local + kPlaneAck, 0);
        if (address == base_ + local + kPlaneAck && size == 4 && value == 0)
            pending_[plane] = false;
        if (overlaps(local + kPlaneArm) && peek(local + kPlaneArm) != 1) armed_[plane] = false;
        if (address == base_ + local + kPlaneArm && size == 4 && value == 1) {
            // Arm before the rest of Enable has been programmed; qualification
            // is checked at the later physical frame, never at this store.
            armed_[plane] = true;
        }
        // A stopped bus/plane cancels pending output. Losing an already valid
        // mode/setup cancels its future arm; restoring a qualifier alone does
        // not resurrect it. Initial Enable may arm before finishing setup.
        if ((overlaps(kBusControl) && peek(kBusControl) != 1) ||
            (overlaps(local + kPlaneRun) && peek(local + kPlaneRun) != 1)) {
            armed_[plane] = pending_[plane] = false;
        }
        if (was_ordinary[plane] && !ordinary_profile(plane)) armed_[plane] = false;
        update_irq(plane);
    }
}

void IftuController::write_word(u32 offset, u64 value) {
    for (unsigned plane = 0; plane < 2; ++plane) {
        const u32 local = plane * kPlaneStep;
        if (offset == local + kPlaneAck) {
            store(offset, 0);  // status encoding unrecovered, not guest-injected
            return;
        }
        if (offset == local + kPlaneState) {
            // Current-bank bit is hardware-owned. Other opaque bits retain
            // their existing register storage behavior.
            store(offset, (value & ~2ull) | (static_cast<u64>(active_bank_[plane]) << 1));
            return;
        }
    }
    store(offset, value);
}

bool IftuController::poke_register(const std::string& name, u64 value) {
    std::vector<RegisterInfo> registers;
    enumerate_registers(registers);
    for (const auto& entry : registers) {
        if (entry.name != name) continue;
        write(entry.address, 4, value);
        return true;
    }
    u64 offset = 0;
    if (parse_u64(name, offset) && offset <= std::numeric_limits<u32>::max() &&
        is_defined(static_cast<u32>(offset))) {
        write(base_ + static_cast<u32>(offset), 4, value);
        return true;
    }
    return false;
}

bool IftuController::ordinary_profile(unsigned plane) const {
    const u32 local = plane * kPlaneStep;
    // shared mode bit0 is alpha merge, not a bank commit. Only the reached
    // binary01 mode pair is automatic; manual00 and unknown11 stay inert.
    return peek(kBusControl) == 1 && peek(local + kPlaneRun) == 1 &&
           peek(local + kPlaneSetup) == 0x108 && peek(local + kPlaneArm) == 1 &&
           ((peek(kShared + 4) >> (1 + plane * 2)) & 3) == 1 && peek(selector_offset(plane)) <= 1;
}

void IftuController::frame_boundary(u64 frames) {
    if (frames == 0) return;
    for (unsigned plane = 0; plane < 2; ++plane) {
        if (!armed_[plane] || !ordinary_profile(plane)) continue;
        // A batch represents elapsed frames without intervening guest stores.
        // Consume the one arm once; ACK/rearm can affect only a later boundary.
        armed_[plane] = false;
        active_bank_[plane] ^= 1;
        const u32 state = plane * kPlaneStep + kPlaneState;
        store(state, (peek(state) & ~2ull) | (static_cast<u64>(active_bank_[plane]) << 1));
        ++turnovers_[plane];
        pending_[plane] = true;
        update_irq(plane);
    }
}

void IftuController::update_irq(unsigned plane) {
    const bool level = pending_[plane];
    if (level == irq_[plane]) return;
    irq_[plane] = level;
    if (irq_callback_) irq_callback_(204 + plane, level);
}

void IftuController::set_irq_callback(std::function<void(u32, bool)> callback) {
    irq_callback_ = std::move(callback);
    if (irq_callback_) for (unsigned plane = 0; plane < 2; ++plane) irq_callback_(204 + plane, irq_[plane]);
}

void IftuController::reset() {
    RegisterBlock::reset();
    active_bank_.fill(0);
    armed_.fill(false);
    pending_.fill(false);
    turnovers_.fill(0);
    for (unsigned plane = 0; plane < 2; ++plane) update_irq(plane);
}

bool IftuController::selected_scanout(unsigned plane, Scanout& out) const {
    if ((peek(kBusControl) & 1) == 0) return false;
    const u32 local = plane * kPlaneStep;
    // Lowio Enable writes +50=1; Disable writes +50=0 and blanks both banks.
    // shared+4's additional mode bits are retained without inferred semantics.
    if ((peek(local + kPlaneRun) & 1) == 0) return false;
    const u64 bank = active_bank_[plane];
    if (peek(selector_offset(plane)) > 1) return false;
    if (bank > 1) return false;
    const u32 start = local + kBankBase + static_cast<u32>(bank) * kBankStep;
    if ((peek(start + kBankBlank) & 1) != 0 ||
        peek(start + kBankFormat) != kFormatAbgr8888) return false;

    const u64 width = peek(start + kBankWidth);
    const u64 height = peek(start + kBankHeight);
    const u64 stride = width * 4 + peek(start + kBankPadding);
    constexpr u64 max_int = std::numeric_limits<int>::max();
    if (width == 0 || height == 0 || width > max_int || height > max_int || stride > max_int)
        return false;
    const u64 length = stride * height;
    const u32 address = static_cast<u32>(peek(start + kBankAddress));
    // Guard the full padded span and 32-bit PA wrap before obtaining a pointer.
    // MMIO/unmapped inputs never cause device reads or allocation of guest RAM.
    if (length == 0 || length > std::numeric_limits<u32>::max() ||
        static_cast<u64>(address) + length > (1ull << 32)) return false;
    const MemRegion* region = bus_.region_at(address, static_cast<size_t>(length));
    if (region == nullptr) return false;

    out.pixels = region->bytes() + (address - region->base);
    out.address = address;
    out.width = static_cast<int>(width);
    out.height = static_cast<int>(height);
    out.stride = static_cast<int>(stride);
    out.plane = plane;
    out.bank = static_cast<unsigned>(bank);
    return true;
}

bool IftuController::active_scanout(Scanout& out) const {
    // Expose the first supported input plane. Full A/B composition is pending;
    // this does not claim that the pixel alpha byte or merge mode was applied.
    return selected_scanout(0, out) || selected_scanout(1, out);
}

const u8* IftuController::framebuffer(int& width, int& height, int& stride,
                                    int* bytes_per_pixel) const {
    Scanout image;
    if (!active_scanout(image)) {
        width = height = stride = 0;
        if (bytes_per_pixel) *bytes_per_pixel = 0;
        return nullptr;
    }
    width = image.width;
    height = image.height;
    stride = image.stride;
    if (bytes_per_pixel) *bytes_per_pixel = 4;
    return image.pixels;
}

std::string IftuController::summary() const {
    Scanout image;
    if (!active_scanout(image)) return "IFTU0: no supported enabled guest-RAM scanout";
    return format("IFTU0 %c bank%u: PA %08X %dx%d pitch %d RGBA8888",
                  image.plane == 0 ? 'A' : 'B', image.bank, image.address,
                  image.width, image.height, image.stride);
}

void IftuController::describe(std::vector<std::string>& lines) const {
    lines.push_back(summary());
    lines.push_back("  ordinary01: one DSI frame per native +180 rearm; manual00 keeps explicit selection");
    lines.push_back("  raw +40 encoding unrecovered (reads 0); internal IRQ latch, full-word zero ACK; blending unsupported");
    for (unsigned plane = 0; plane < 2; ++plane) {
        lines.push_back(format("  %c: selector=%llu active=%u armed=%u pending=%u IRQ%u=%u turnovers=%llu",
                               plane == 0 ? 'A' : 'B', static_cast<unsigned long long>(peek(selector_offset(plane))),
                               active_bank_[plane], armed_[plane] ? 1u : 0u, pending_[plane] ? 1u : 0u,
                               204 + plane, irq_[plane] ? 1u : 0u,
                               static_cast<unsigned long long>(turnovers_[plane])));
    }
}

}  // namespace zlb::kermit
