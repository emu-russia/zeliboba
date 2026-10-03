// Firmware 1.04 native DSI0 progressive timing.
//
// Lowio maps E5050000/1000 and IRQ213 at linked VA 0x81003EA4.
// StartDisplay 0x8100517C programs VIC0 progressive timing and control=1;
// its handler 0x81003AE0 reads/writes status +50 (W1C), then dispatches
// subinterrupt1 when status and enabled mask +54 both contain bit1.
// Display stores float 0x426FC29E, approximately 60000/1001 Hz.
// See docs/FIRMWARE_DISPLAY_104.md for the exact native register evidence.
//
// Only that timing subset is implemented. Packet/PHY execution and +414
// shutdown completion, +48 idle semantics, +4C scanline semantics, and other
// control modes are unsupported. Ordinary register writes are retained; no
// ready bit or guest callback is synthesized. Physical frame boundaries are
// published to the board's IFTU connection even when the DSI IRQ is masked.
#include "hw/soc/soc_internal.h"

#include "event/providers.h"

namespace zlb::kermit {
namespace {

constexpr u32 kControl = 0x00;
constexpr u32 kProgressive = 0x04;
constexpr u32 kHorizontalTiming = 0x08;
constexpr u32 kVerticalTotal = 0x0C;
constexpr u32 kStatus = 0x50;
constexpr u32 kMask = 0x54;
constexpr u32 kVblank = 2;
constexpr u64 kPhasePerMicrosecond = 60;
constexpr u64 kPhasePerFrame = 1001000;

}  // namespace

DsiController::DsiController() : Device("Kermit.DSI0", kDsi0Base, kDsi0Size) {
    register_name_entry(base_ + kControl, "CONTROL");
    register_name_entry(base_ + kProgressive, "REG_004");
    register_name_entry(base_ + kHorizontalTiming, "H_TIMING");
    register_name_entry(base_ + kVerticalTotal, "V_TOTAL");
    register_name_entry(base_ + kStatus, "STATUS");
    register_name_entry(base_ + kMask, "IRQ_MASK");
    for (u32 offset : {0x10u, 0x14u, 0x1Cu, 0x24u, 0x2Cu, 0x30u, 0x3Cu,
                       0x40u, 0x48u, 0x4Cu, 0x5Cu, 0x60u, 0x6Cu, 0x70u,
                       0x414u, 0x500u, 0x504u, 0x508u, 0x50Cu, 0x514u,
                       0x518u, 0x51Cu, 0x804u, 0x834u, 0x838u, 0x940u,
                       0x944u, 0x948u, 0x94Cu}) {
        register_name_entry(base_ + offset, format("REG_%03X", offset));
    }
    for (u32 offset = 0x810; offset <= 0x830; offset += 4) {
        register_name_entry(base_ + offset, format("REG_%03X", offset));
    }
}

u64 DsiController::read(u32 address, unsigned size) {
    u64 result = 0;
    for (unsigned i = 0; i < size && i < 8; ++i) {
        const u64 current = static_cast<u64>(address) + i;
        if (current < base_ || current >= static_cast<u64>(base_) + size_) continue;
        const u32 offset = static_cast<u32>(current - base_);
        const u32 byte = (registers_[offset / 4] >> ((offset & 3) * 8)) & 0xFFu;
        result |= static_cast<u64>(byte) << (i * 8);
    }
    return result;
}

void DsiController::write(u32 address, unsigned size, u64 value) {
    const bool was_running = supported_running();
    for (unsigned i = 0; i < size && i < 8; ++i) {
        const u64 current = static_cast<u64>(address) + i;
        if (current < base_ || current >= static_cast<u64>(base_) + size_) continue;
        const u32 offset = static_cast<u32>(current - base_);
        const u32 word = offset & ~3u;
        const u32 shift = (offset & 3) * 8;
        const u32 supplied = static_cast<u32>((value >> (i * 8)) & 0xFFu) << shift;
        if (word == kStatus) {
            // Clear only bits supplied in this access's lanes. Other bytes of
            // the pending word must not turn into accidental ACK bits.
            registers_[word / 4] &= ~supplied;
        } else {
            registers_[word / 4] = (registers_[word / 4] & ~(0xFFu << shift)) | supplied;
        }
    }
    // A fresh supported start uses phase zero. The physical first-vblank line
    // is not recovered; this deterministic startup phase is a model choice.
    if (was_running != supported_running()) phase_ = 0;
    if (!was_running && supported_running() &&
        events().should_record(EventProvider::Display, EventLevel::Informational,
                               event_keyword::kDisplay)) {
        events().event(EventProvider::Display, ev::display::kModeSet)
            .field("mode", (u64)1)
            .field("running", (u64)(supported_running() ? 1 : 0))
            .emit();
    }
    update_irq();
}

bool DsiController::supported_running() const {
    // Mode values {1,2,2,3,4} exist in the driver. Only ordinary progressive
    // mode1 with the actual head0 VIC0 timing is understood, not generic bit0.
    return registers_[kControl / 4] == 1 && registers_[kProgressive / 4] == 0 &&
           registers_[kHorizontalTiming / 4] == 0xC4E &&
           registers_[kVerticalTotal / 4] == 0x252;
}

void DsiController::tick(u64 microseconds) {
    if (!supported_running() || microseconds == 0) return;
    // Decompose before multiplying so even UINT64_MAX microseconds is safe.
    // One full 1001000us block is exactly 60 frames. Fractional phase is kept
    // across calls, making sliced and batched time advance identically.
    const u64 blocks = microseconds / kPhasePerFrame;
    const u64 remainder = microseconds % kPhasePerFrame;
    const u64 phase = phase_ + remainder * kPhasePerMicrosecond;
    const u64 frames = blocks * kPhasePerMicrosecond + phase / kPhasePerFrame;
    phase_ = phase % kPhasePerFrame;
    if (frames == 0) return;
    frames_ += frames;
    if (events().should_record(EventProvider::Display, EventLevel::Verbose, event_keyword::kDisplay)) {
        events().event(EventProvider::Display, ev::display::kVblank)
            .field("count", frames_)
            .emit();
    }
    if (frame_callback_) frame_callback_(frames);
    // Pending frames coalesce in the hardware status bit. Retaining raw
    // status while masked is an explicit conventional model choice.
    registers_[kStatus / 4] |= kVblank;
    update_irq();
}

void DsiController::update_irq() {
    const bool asserted = (registers_[kStatus / 4] & registers_[kMask / 4] & kVblank) != 0;
    if (asserted == irq_) return;
    irq_ = asserted;
    if (irq_callback_) irq_callback_(kIrqDsi0, irq_);
    if (asserted && events().should_record(EventProvider::Interrupt, EventLevel::Informational,
                                           event_keyword::kInterrupt)) {
        events().event(EventProvider::Interrupt, ev::interrupt::kRaise)
            .field("line", (u64)213)
            .field("source", (u64)0)
            .emit();
    }
}

void DsiController::set_irq_callback(std::function<void(u32, bool)> callback) {
    irq_callback_ = std::move(callback);
    if (irq_callback_) irq_callback_(kIrqDsi0, irq_);
}

void DsiController::reset() {
    registers_.fill(0);
    phase_ = frames_ = 0;
    update_irq();
}

const char* DsiController::register_name(u32 address) const {
    return handles(address) ? lookup_name(address & ~3u) : nullptr;
}

void DsiController::enumerate_registers(std::vector<RegisterInfo>& out) const {
    for (const auto& entry : names_) out.push_back({entry.first, entry.second, 0, 4});
}

bool DsiController::peek_register(const std::string& name, u64& out) const {
    for (const auto& entry : names_) {
        if (entry.second != name) continue;
        out = registers_[(entry.first - base_) / 4];
        return true;
    }
    return false;
}

bool DsiController::poke_register(const std::string& name, u64 value) {
    for (const auto& entry : names_) {
        if (entry.second != name) continue;
        write(entry.first, 4, value);
        return true;
    }
    return false;
}

std::string DsiController::summary() const {
    return format("DSI0: %s, %llu frames, status=%08X mask=%08X IRQ213=%s",
                  supported_running() ? "progressive VIC0 60000/1001 Hz" : "stopped/unsupported mode",
                  static_cast<unsigned long long>(frames_), registers_[kStatus / 4],
                  registers_[kMask / 4], irq_ ? "high" : "low");
}

void DsiController::describe(std::vector<std::string>& lines) const {
    lines.push_back(summary());
    lines.push_back("  packet/PHY execution, shutdown completion and scanline semantics unsupported");
    lines.push_back("  deterministic first-frame phase; board frame boundaries, no guest callback synthesis");
}

void DsiController::save_state(StateWriter& writer) const {
    // The whole register array (including the timing words and the status/mask
    // pair), the frame phase and the frame counter. The two callbacks are host
    // wiring and are never serialised.
    writer.fixed(registers_, [&](u32 value) { writer.put_u32(value); });
    writer.put_u64(phase_);
    writer.put_u64(frames_);
    writer.put_bool(irq_);
}

void DsiController::load_state(StateReader& reader) {
    reader.fixed(registers_, [&](u32& value) { value = reader.get_u32(); });
    phase_ = reader.get_u64();
    frames_ = reader.get_u64();
    irq_ = reader.get_bool();
}

}  // namespace zlb::kermit
