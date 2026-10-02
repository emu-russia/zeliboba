// zeliboba - Kermit: the ARM Cortex-A9 MPCore side of the board.
//
// This file implements KermitBlock (src/hw/soc.h) and the syscon bridge window
// that hangs off it. The other devices live in gic.cpp, timers.cpp, uart.cpp,
// sdif.cpp, dma.cpp and display.cpp, and their private declarations are in
// soc_internal.h.
//
// Physical map (the memory windows themselves are in hw/soc.h; these are the
// peripheral windows this file installs):
//
//   0x40000000  SCU registers                     TRM 1.5, table 1-3
//   0x40000100  CPU interface (per core)          TRM 3.4.1
//   0x40000200  global timer                      TRM 4.4
//   0x40000600  private timer + watchdog          TRM 4.2
//   0x40001000  interrupt distributor             TRM 3.3
//   0x40002000  CPU interface (generic GIC page)  alias
//   0xE0B00000  SDIF port 0 (eMMC)                sdif.elf table @0x81009FA8
//   0xE0C00000  SDIF port 1 (empty window)        same table
//   0xE0C10000  SDIF port 2 (empty window)        same table
//   0xE2000000  syscon (Ernie) bridge             VitaDevices.cs ARM.SysconBridge
//   0xE2030000  debug UART                        VitaDevices.cs ARM.Uart
//   0xE2040000  SDIF port 0 alias                 ASSUMPTION
//   0xE2060000  DMA controller                    ASSUMPTION
//   0xE2100000  display controller                ASSUMPTION
//
// The syscon bridge window is the one the C# reference model reserves
// (VitaDevices.cs: SpyDevice("ARM.SysconBridge", 0xE2000000, 0x1000)). Its
// internal register offsets are an ASSUMPTION: syscon.elf did not yield a usable
// table, so the shape below is the obvious one - a door bell carrying the
// command id, a parameter block, a status register with the busy/done handshake
// and a response window - and every register has a name so that the debugger can
// be used to correct it against the firmware:
//
//   0x00 SC_COMMAND      write the command id to post it
//   0x04 SC_PARAM0       parameter 0
//   0x08 SC_PARAM1       parameter 1
//   0x0C SC_PARAM2       parameter 2
//   0x10 SC_STATUS       bit0 busy, bit1 response ready, bit2 error
//   0x14 SC_RESPONSE     response word 0
//   0x18 SC_RESPONSE1    response word 1
//   0x1C SC_IRQ_ENABLE   bit0 raise the syscon SPI when a response arrives
//   0x20 SC_DOORBELL     alternative trigger (same as writing SC_COMMAND)
#include "common/log.h"
#include "hw/soc/soc_internal.h"

#include <cstdlib>
#include <functional>
#include "hw/cmep/cmep_internal.h"

namespace {

// The guest's DMA window (0xE0410000) receives a descriptor and then a doorbell
// write at +0x020 that carries the physical address of the descriptor chain.  The
// engine's transfer itself is not implemented, but the doorbell is the point where
// the hardware would complete and interrupt, so this subclass exists to test that:
// with ZLB_DMA_IRQ=<n> it asserts interrupt <n> (a pulse) on the doorbell.  The
// DMA library registered handlers 0x70..0x7F, one per channel, so 0x7D is the
// natural default for the channel-13 operation the display module submits.
/// A window that stores whatever the guest writes, learning the register layout
/// from the accesses themselves. It is used for the windows the kernel's device
/// table declares but whose semantics are not recovered: a plain RegisterBlock
/// drops writes to offsets it was not told about, which is how 89 writes vanished
/// during a single boot before this existed. Save states are keyed by offset, so a
/// register defined at run time round-trips correctly.
class StorageWindow : public zlb::kermit::RegisterBlock {
public:
    StorageWindow(std::string name, zlb::u32 base, zlb::u32 size)
        : RegisterBlock(std::move(name), base, size) {}

    void write(zlb::u32 address, unsigned size, zlb::u64 value) override {
        const zlb::u32 offset = (address - base()) & ~3u;
        if (!is_defined(offset)) define(offset, "W_" + std::to_string(offset));
        RegisterBlock::write(address, size, value);
    }
};

class DmaWindow : public zlb::kermit::RegisterBlock {
public:
    DmaWindow(std::string name, zlb::u32 base, zlb::u32 size, std::function<void(zlb::u32)> raise)
        : RegisterBlock(std::move(name), base, size), raise_(std::move(raise)) {}

    void tick(zlb::u64 ticks) override {
        if (!busy_) return;
        if (ticks >= budget_) {
            budget_ = 0;
        } else {
            budget_ -= static_cast<zlb::u32>(ticks);
        }
        if (budget_ == 0u) {
            busy_ = false;
            RegisterBlock::write(base() + 0x024u, 4u, 0u);
            RegisterBlock::write(base() + 0x028u, 4u, 3u);
            ZLB_LOG_INFO("machine", "module: DMA engine finished after the transfer budget -> pulse irq 0x%X", irq_);
            if (raise_) raise_(irq_);
        }
    }

    void set_budget(zlb::u32 ticks) { budget_ticks_ = ticks ? ticks : 1u; }

    void write(zlb::u32 address, unsigned size, zlb::u64 value) override {
        RegisterBlock::write(address, size, value);
        if (enabled_ && address - base() == 0x020u && value != 0) {
            // +0x24 bit 0 is BUSY, not done: the guest spins at 0x438046
            // ("tst.w r3,#1; bne") until it clears, then checks [+0x28] at 0x438432
            // for the finished channels. The engine completes as soon as it is armed.
            // A poll-driven variant - finish on the first read of +0x24 - was tried
            // and is *worse*: the thread sleeps on the completion event rather than
            // polling (start #22 then hangs again, 22 starts / 21 results), so the
            // engine has to complete unprompted. What is still not modelled is the
            // transfer itself: the bytes never move.
            RegisterBlock::write(base() + 0x024u, 4u, 1u);
            RegisterBlock::write(base() + 0x028u, 4u, 0u);
            busy_ = true;
            budget_ = budget_ticks_;
            ZLB_LOG_INFO("machine", "module: DMA doorbell +0x020 = 0x%08X -> busy for %u ticks",
                         static_cast<unsigned>(value), budget_ticks_);
        }
    }

    void set_irq(zlb::u32 irq) { irq_ = irq; }
    void set_enabled(bool on) { enabled_ = on; }

private:
    std::function<void(zlb::u32)> raise_;
    zlb::u32 irq_ = 0x7Cu;
    bool enabled_ = false;
    bool busy_ = false;
    zlb::u32 budget_ = 0;
    zlb::u32 budget_ticks_ = 1000u;
};

}  // namespace

namespace zlb::kermit {

namespace detail {

// Syscon bridge register offsets (ASSUMPTION, see the file comment).
constexpr u32 kCommand = 0x00;
constexpr u32 kParam0 = 0x04;
constexpr u32 kParam1 = 0x08;
constexpr u32 kParam2 = 0x0C;
constexpr u32 kStatus = 0x10;
constexpr u32 kResponse0 = 0x14;
constexpr u32 kResponse1 = 0x18;
constexpr u32 kIrqEnable = 0x1C;
constexpr u32 kDoorbell = 0x20;

constexpr u32 kStBusy = 1u << 0;
constexpr u32 kStResponseReady = 1u << 1;
constexpr u32 kStError = 1u << 2;

// SCU register offsets (TRM 2.2: SCU Control / Configuration / Power Status).
constexpr u32 kScuControl = 0x00;
constexpr u32 kScuConfig = 0x04;
constexpr u32 kScuPowerStatus = 0x08;

/// The device names KermitBlock::tick advances; must match install().
bool needs_tick(const std::string& name) {
    return name == "Kermit.GIC" || name == "Kermit.GT" || name == "Kermit.PT" || name == "Kermit.DmaWin" ||
           name == "Kermit.Sdif0" ||
           name == "Kermit.Sdif1" || name == "Kermit.Sdif2" || name == "Kermit.DMA" || name == "Kermit.Display" ||
           name == "Kermit.DSI0" || name == "Kermit.EmcTop" || name == "Kermit.LT5" || name == "Kermit.WT7" || name == "Kermit.Spi0";
}

/// The devices KermitBlock::devices() reports; must match install().
bool is_owned(const std::string& name) {
    return name == "Kermit.SCU" || name == "Kermit.GT" || name == "Kermit.PT" || name == "Kermit.GIC" ||
           name == "Kermit.Uart" || name == "Kermit.Sdif0" || name == "Kermit.Sdif1" || name == "Kermit.Sdif2" ||
           name == "Kermit.DMA" || name == "Kermit.Display" || name == "Kermit.IFTU0" || name == "Kermit.DSI0" ||
           name == "Kermit.Syscon" || name == "Kermit.I2c0" || name == "Kermit.I2c1" || name == "Kermit.EmcTop" ||
           name == "Kermit.LT5" || name == "Kermit.WT7" ||
           name.compare(0, 18, "Kermit.PeriphPower") == 0 || name == "Kermit.BootHandshake";
}

const char* const kGicDeviceName = "Kermit.GIC";

/// Find a device by name in a bus (nullptr when there is none).
Device* find_device(Bus& bus, const std::string& name) {
    for (const auto& device : bus.devices()) {
        if (device->name() == name) return device.get();
    }
    return nullptr;
}

}  // namespace detail

// ---------------------------------------------------------------------------
// RegisterBlock: byte-addressable register storage shared by every device here.
//
// Bus turns a sub-word MMIO access into one Device::read/write call per byte, so
// a whole-word register has to be assembled from (or split into) its bytes by
// hand. Every access therefore goes through the byte image first and then
// through read_word()/write_word(), which decode the register as a whole (a two
// byte register that shares its word with a sibling, the 64 bit timer counters,
// a FIFO port, ...). `store_` is the effective value; `values_` is the byte
// image used by read_word()'s default implementation.
// ---------------------------------------------------------------------------

void RegisterBlock::define(u32 offset, const std::string& name, u64 reset_value, unsigned width) {
    values_[offset] = reset_value;
    defaults_[offset] = reset_value;
    widths_[offset] = width;
    names_[offset] = name;
    by_name_[name] = offset;
    store_[offset] = reset_value;
}

void RegisterBlock::define_bytes(u32 offset, const std::string& name, const char* text, unsigned length) {
    u64 value = 0;
    for (unsigned i = 0; i < length && text[i] != '\0'; ++i) {
        value |= static_cast<u64>(static_cast<u8>(text[i])) << (8 * i);
    }
    define(offset, name, value, length);
}

u64 RegisterBlock::peek(u32 offset) const {    // store_ is the single source of truth; poke() and store() both keep it up
    // to date, and the byte image in values_ is only used by the fallback path.
    auto it = store_.find(offset);
    return it == store_.end() ? 0 : it->second;
}

void RegisterBlock::poke(u32 offset, u64 value) {
    auto it = values_.find(offset);
    if (it == values_.end()) return;
    const unsigned width = widths_.count(offset) ? widths_[offset] : 4;
    const u64 mask = width >= 8 ? ~0ull : ((1ull << (width * 8)) - 1ull);
    it->second = value & mask;
    store_[offset] = value;
}

u64 RegisterBlock::store(u32 offset) const {
    auto it = store_.find(offset);
    return it == store_.end() ? 0 : it->second;
}

void RegisterBlock::store(u32 offset, u64 value) {
    store_[offset] = value;
    poke(offset, value);
}

void RegisterBlock::reset() {
    for (const auto& entry : defaults_) {
        values_[entry.first] = entry.second;
        store_[entry.first] = entry.second;
    }
}

namespace {

/// True when an access of `size` bytes at `offset` stays inside one 32 bit
/// register word.
inline bool within_one_word(u32 offset, unsigned size) {
    return size != 0 && size <= 4 && (offset & 3u) <= (4u - size);
}

}  // namespace

const char* RegisterBlock::register_name(u32 address) const {
    if (!handles(address)) return nullptr;
    return lookup_name(address - base_);
}

u64 RegisterBlock::read(u32 address, unsigned size) {
    const u32 offset = address - base_;
    // An access that fits inside one 32 bit register word is decoded once, by
    // read_word(), so that a register with read side effects (the GIC's ICCIAR)
    // is only touched once and a register that shares its word with a sibling
    // (the SDIF transfer mode / command pair) is seen as a whole.
    if (within_one_word(offset, size) && is_defined(offset & ~3u)) {
        const u32 word = offset & ~3u;
        const u64 value = read_word(word, store(word));
        const u32 shift = (offset & 3u) * 8;
        u64 out = 0;
        for (unsigned i = 0; i < size; ++i) {
            out |= ((value >> ((shift + i * 8u) & 63u)) & 0xFFull) << (8 * i);
        }
        return out;
    }

    // Anything else (a FIFO, an unaligned or oversized access) is assembled from
    // the byte image one byte at a time.
    u64 out = 0;
    for (unsigned i = 0; i < size; ++i) {
        const u32 current = offset + i;
        const u32 word_offset = current & ~3u;
        if (!is_defined(word_offset)) continue;
        const u64 word_value = read_word(word_offset, store(word_offset));
        const u8 byte = static_cast<u8>((word_value >> ((current & 3u) * 8)) & 0xFF);
        out |= static_cast<u64>(byte) << (8 * i);
    }
    return out;
}

void RegisterBlock::write(u32 address, unsigned size, u64 value) {
    const u32 offset = address - base_;
    // The byte image is updated first, so that a device's write_word() can read
    // the value the caller just stored with store() even for a register that
    // shares its 32 bit word with another register. write_word() only has to
    // apply the side effects of the write.
    if (within_one_word(offset, size) && is_defined(offset & ~3u)) {
        const u32 word = offset & ~3u;
        // For a 4 byte aligned access the caller's word replaces the register;
        // otherwise the untouched bytes come from the effective value. The
        // device's write_word() is called afterwards and may adjust the stored
        // value (a write-one-to-clear register, for instance).
        u64 merged = store(word);
        const u32 shift = (offset & 3u) * 8;
        for (unsigned i = 0; i < size; ++i) {
            const u32 byte_shift = (shift + i * 8u) & 63u;
            merged = (merged & ~(0xFFull << byte_shift)) |
                     ((value >> (8 * i) & 0xFFull) << byte_shift);
        }
        store(word, merged);
        write_word(word, merged);
        return;
    }

    for (unsigned i = 0; i < size; ++i) {
        const u32 current = offset + i;
        const u8 byte = static_cast<u8>((value >> (8 * i)) & 0xFF);
        if (!is_defined(current & ~3u)) continue;
        const u32 target = current & ~3u;
        const u32 shift = (current & 3u) * 8;
        store(target, (store(target) & ~(0xFFull << shift)) | (static_cast<u64>(byte) << shift));
    }

    if (within_one_word(offset, size) && is_defined(offset & ~3u)) {
        write_word(offset & ~3u, store(offset & ~3u));
    }
}

void RegisterBlock::enumerate_registers(std::vector<RegisterInfo>& out) const {
    for (const auto& entry : names_) {
        RegisterInfo info;
        info.address = base_ + entry.first;
        info.name = entry.second;
        auto def = defaults_.find(entry.first);
        info.reset_value = def == defaults_.end() ? 0 : def->second;
        auto width = widths_.find(entry.first);
        info.width = width == widths_.end() ? 4u : width->second;
        out.push_back(std::move(info));
    }
}

bool RegisterBlock::peek_register(const std::string& name, u64& out) const {
    auto it = by_name_.find(name);
    if (it != by_name_.end()) {
        out = peek(it->second);
        return true;
    }
    u64 address = 0;
    if (parse_u64(name, address) && values_.count(static_cast<u32>(address))) {
        out = peek(static_cast<u32>(address));
        return true;
    }
    return false;
}

bool RegisterBlock::poke_register(const std::string& name, u64 value) {
    auto it = by_name_.find(name);
    if (it != by_name_.end()) {
        write_word(it->second, value);
        return true;
    }
    u64 address = 0;
    if (parse_u64(name, address) && values_.count(static_cast<u32>(address))) {
        write_word(static_cast<u32>(address), value);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// SCU registers
// ---------------------------------------------------------------------------

/// Minimal SCU register block, enough for the kernel's early bring-up (the
/// kernel enables the SCU and reads the core count from it).
class ScuRegisters : public RegisterBlock {
public:
    ScuRegisters(std::string name, u32 base, u32 size) : RegisterBlock(std::move(name), base, size) {
        define(detail::kScuControl, "SCU_CONTROL", 0);
        define(detail::kScuConfig, "SCU_CONFIG", 0x00000101u);
        define(detail::kScuPowerStatus, "SCU_POWER_STATUS", 0x00000001u);
    }

    u64 read_word(u32 offset, u64 stored) override {
        switch (offset) {
            case detail::kScuConfig: return 0x00000101u;  // one core in the cluster, SMP
            case detail::kScuPowerStatus: return 0x00000001u;
            default: return stored;
        }
    }

    std::string summary() const override {
        return format("%s control=0x%X cores=1", name_.c_str(),
                      static_cast<unsigned>(peek(detail::kScuControl) & 0xF));
    }
};

// ---------------------------------------------------------------------------
// SysconBridge
// ---------------------------------------------------------------------------

SysconBridge::SysconBridge(std::string name, u32 base, u32 size) : RegisterBlock(std::move(name), base, size) {
    define(detail::kCommand, "SC_COMMAND", 0);
    define(detail::kParam0, "SC_PARAM0", 0);
    define(detail::kParam1, "SC_PARAM1", 0);
    define(detail::kParam2, "SC_PARAM2", 0);
    define(detail::kStatus, "SC_STATUS", 0);
    define(detail::kResponse0, "SC_RESPONSE", 0);
    define(detail::kResponse1, "SC_RESPONSE1", 0);
    define(detail::kIrqEnable, "SC_IRQ_ENABLE", 0);
    define(detail::kDoorbell, "SC_DOORBELL", 0);
}

void SysconBridge::reset() {
    RegisterBlock::reset();
    last_command_ = 0;
    commands_ = 0;
    if (irq_callback_) irq_callback_(static_cast<u32>(Irq::Syscon), false);
}

u64 SysconBridge::read_word(u32 offset, u64 stored) {
    (void)offset;
    return stored;
}

void SysconBridge::write_word(u32 offset, u64 value) {
    poke(offset, value);
    if (offset == detail::kCommand || offset == detail::kDoorbell) post_command();
}

void SysconBridge::post_command() {
    last_command_ = static_cast<u32>(peek(detail::kCommand));
    ++commands_;
    poke(detail::kStatus, detail::kStBusy);

    std::vector<u8> payload;
    const u32 p0 = static_cast<u32>(peek(detail::kParam0));
    const u32 p1 = static_cast<u32>(peek(detail::kParam1));
    const u32 p2 = static_cast<u32>(peek(detail::kParam2));
    for (u32 word : {p0, p1, p2}) {
        for (unsigned i = 0; i < 4; ++i) payload.push_back(static_cast<u8>((word >> (i * 8)) & 0xFF));
    }

    bool ok = false;
    if (ernie_ != nullptr) {
        const std::vector<u8> response = ernie_->dispatch_command(last_command_, payload);
        ok = ernie_->response_ready();
        u32 words[2] = {0, 0};
        for (size_t i = 0; i < response.size() && i < 8; ++i) {
            words[i / 4] |= static_cast<u32>(response[i]) << ((i % 4) * 8);
        }
        poke(detail::kResponse0, words[0]);
        poke(detail::kResponse1, words[1]);
    }

    // The functional SC model answers immediately, so the handshake completes
    // before the driver's first poll: that is what keeps the kernel's polling
    // loops terminating.
    poke(detail::kStatus, ok ? detail::kStResponseReady : detail::kStError);
    if (irq_callback_) {
        irq_callback_(static_cast<u32>(Irq::Syscon), false);
        if ((peek(detail::kIrqEnable) & 1u) != 0) irq_callback_(static_cast<u32>(Irq::Syscon), true);
    }
}

// ---------------------------------------------------------------------------
// PeripheralPower
// ---------------------------------------------------------------------------

PeripheralPower::PeripheralPower(std::string name, u32 base, u32 size)
    : RegisterBlock(std::move(name), base, size) {
    define(kStatus, "PERIPH_STATUS", kReadyBits);
    define(0x04, "PERIPH_CTRL04", 0);
    define(0x10, "PERIPH_CTRL10", 0);
    define(0x20, "PERIPH_CTRL20", 0);
    define(0x30, "PERIPH_CTRL30", 0);
    define(0x40, "PERIPH_CTRL40", 0);
    define(0x50, "PERIPH_CTRL50", 0);
    define(0x60, "PERIPH_CTRL60", 0);
    define(0x64, "PERIPH_KICK", 0);
    define(0x70, "PERIPH_ENABLE", 0);
}

u64 PeripheralPower::read_word(u32 offset, u64 stored) {
    if (offset == kStatus) return kReadyBits;
    // +0x64 is a command register, not storage: the CMeP second loader starts a
    // peripheral by writing 0x101 to it and then spins until it reads back zero
    // (0x4882A: `sw $0,100($3)` / `erepeat 0x48834` / `lw $0,($9)` /
    // `beqz $0,0x48838`).  The write is latched so the debugger can see it, but
    // the hardware consumes it immediately and the read returns zero.
    if (offset == kKick) return 0;
    return stored;
}

void PeripheralPower::write_word(u32 offset, u64 value) {
    // The kick writes are latched in the byte image by the base class; remember
    // the last one so `devices` can show what was started.
    if (offset == kKick) kicks_ = static_cast<u32>(value);
}

std::string PeripheralPower::summary() const {
    return format("%s ready=0x%X enable=0x%X kick=0x%X", name_.c_str(), kReadyBits,
                  static_cast<unsigned>(peek(0x70)), kicks_);
}

void PeripheralPower::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s: status=0x%X (bit 8 = powered, bit 9 = clocked), enable=0x%X, last kick=0x%X",
                           name_.c_str(), kReadyBits, static_cast<unsigned>(peek(0x70)), kicks_));
}

void PeripheralPower::save_state(StateWriter& writer) const {
    RegisterBlock::save_state(writer);
    writer.put_u32(kicks_);
}

void PeripheralPower::load_state(StateReader& reader) {
    RegisterBlock::load_state(reader);
    kicks_ = reader.get_u32();
}

// ---------------------------------------------------------------------------
// BootHandshake
// ---------------------------------------------------------------------------

BootHandshake::BootHandshake(std::string name, u32 base, u32 size)
    : RegisterBlock(std::move(name), base, size) {
    define(kCommand0, "BOOT_CMD0", 0);
    define(kCommand1, "BOOT_CMD1", 0);
    define(kStatus, "BOOT_STATUS", 0);
    define(0x100, "BOOT_REG100", 0);
    define(0x108, "BOOT_REG108", 0);
    define(0x180, "BOOT_REG180", 0);
}

void BootHandshake::reset() {
    RegisterBlock::reset();
    last_command_ = 0;
    commands_ = 0;
}

void BootHandshake::write_word(u32 offset, u64 value) {
    if (offset == kCommand0 || offset == kCommand1) {
        last_command_ = static_cast<u32>(value);
        ++commands_;
        poke(kCommand0, last_command_);
    }
    poke(offset, value);
}

u64 BootHandshake::read_word(u32 offset, u64 stored) {
    if (offset == kStatus) {
        if (last_command_ == 2) return kAckAfter2;
        if (last_command_ == 1) return kAckAfter1;
        return 0;
    }
    return stored;
}

std::string BootHandshake::summary() const {
    return format("%s commands=%llu last=0x%X status=0x%X", name_.c_str(),
                  static_cast<unsigned long long>(commands_), last_command_,
                  static_cast<unsigned>(peek(kStatus)));
}

void BootHandshake::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s: %llu boot commands, last 0x%X, status 0x%X (0x44 after 2, 0x11 after 1)",
                           name_.c_str(), static_cast<unsigned long long>(commands_), last_command_,
                           static_cast<unsigned>(peek(kStatus))));
}

void BootHandshake::save_state(StateWriter& writer) const {
    RegisterBlock::save_state(writer);
    writer.put_u32(last_command_);
    writer.put_u64(commands_);
}

void BootHandshake::load_state(StateReader& reader) {
    RegisterBlock::load_state(reader);
    last_command_ = reader.get_u32();
    commands_ = reader.get_u64();
}

std::string SysconBridge::summary() const {
    return format("%s commands=%llu last=0x%X status=0x%X", name_.c_str(),
                  static_cast<unsigned long long>(commands_), last_command_,
                  static_cast<unsigned>(peek(detail::kStatus) & 0xFF));
}

void SysconBridge::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s: %llu SC commands, last id 0x%X, response 0x%08X/0x%08X", name_.c_str(),
                           static_cast<unsigned long long>(commands_), last_command_,
                           static_cast<unsigned>(peek(detail::kResponse0)),
                           static_cast<unsigned>(peek(detail::kResponse1))));
    lines.push_back(format("    status=0x%X %s", static_cast<unsigned>(peek(detail::kStatus) & 0xFF),
                           ernie_ ? (ernie_->running_firmware() ? "(Ernie runs its firmware)" : "(functional SC model)")
                                  : "(no syscon fitted)"));
}

void SysconBridge::save_state(StateWriter& writer) const {
    RegisterBlock::save_state(writer);
    // `ernie_` (a pointer to the syscon block) and `irq_callback_` are wiring.
    writer.put_u32(last_command_);
    writer.put_u64(commands_);
}

void SysconBridge::load_state(StateReader& reader) {
    RegisterBlock::load_state(reader);
    last_command_ = reader.get_u32();
    commands_ = reader.get_u64();
}

}  // namespace zlb::kermit

// ---------------------------------------------------------------------------
// KermitBlock: the public block declared in hw/soc.h
// ---------------------------------------------------------------------------

namespace zlb {

/// The private state of KermitBlock. Nested so that it can be named as
/// KermitBlock::Impl (the header forward declares it that way).
struct KermitBlock::Impl {
    Bus& bus;
    EmmcCard* card = nullptr;

    // The devices are owned by the bus once install() has run; these raw
    // pointers are the accessors the public methods use (a unique_ptr would be
    // nulled out by the move into the bus).
    kermit::ScuRegisters* scu = nullptr;
    kermit::Gic* gic = nullptr;
    kermit::GlobalTimer* global_timer = nullptr;
    kermit::PrivateTimer* private_timer = nullptr;
    kermit::Uart* uart = nullptr;
    kermit::Sdif* sdif0 = nullptr;
    kermit::Sdif* sdif1 = nullptr;
    kermit::Sdif* sdif2 = nullptr;
    Device* sdif0_alias = nullptr;
    kermit::Spi* spi0 = nullptr;
    kermit::Spi* spi1 = nullptr;
    kermit::Spi* spi2 = nullptr;
    kermit::DmaController* dma = nullptr;
    kermit::DisplayController* display = nullptr;
    kermit::IftuController* iftu = nullptr;
    kermit::DsiController* dsi = nullptr;
    kermit::SysconBridge* syscon_bridge = nullptr;

    Cpu* cpu = nullptr;
    bool installed = false;

    /// Fractional CPU cycles left over when converting to PERIPHCLK ticks.
    u64 cycle_accumulator = 0;
    u64 total_cycles = 0;

    Impl(Bus& bus_, EmmcCard* card_) : bus(bus_), card(card_) {}

    /// CPU cycles are converted to PERIPHCLK ticks here, once, so that every
    /// device's tick() works in the same unit (microseconds) and no device has
    /// to know the core clock.
    u64 periph_ticks(u64 cycles) const { return kermit::periph_ticks_from_cycles(cycles); }

    void raise(u32 id, bool level) {
        if (!gic) return;
        gic->distributor().set_level(id, level);
        // Round 339: the CPU line is a level derived continuously from the
        // distributor on hardware, but the model only recomputed it when the guest
        // wrote to the distributor/CPU interface or when a pulse expired.  A device
        // that asserts its line (Kermit.Sdif0 does, and the guest had enabled its
        // signal bits - see docs/NSKBL.md 10.42) therefore never reached the CPU:
        // a breakpoint on the non-secure IRQ vector (VA 0x40118) never fired in a
        // whole run, and the storage completion NSKBL waits for never arrived.
        gic->refresh_line();
    }
};

namespace {

/// Find the interrupt controller among the bus devices (the free functions
/// below look it up by name so they do not need access to KermitBlock::impl_).
kermit::Gic* find_gic(Bus& bus) {
    for (const auto& device : bus.devices()) {
        if (device->name() == "Kermit.GIC") return static_cast<kermit::Gic*>(device.get());
    }
    return nullptr;
}

const kermit::Gic* find_gic(const Bus& bus) {
    for (const auto& device : bus.devices()) {
        if (device->name() == "Kermit.GIC") return static_cast<const kermit::Gic*>(device.get());
    }
    return nullptr;
}

}  // namespace

// --- the two extensions hw/soc.h does not declare -------------------------
//
// hw/soc.h belongs to the machine workstream and must not be touched, so these
// are free functions (declared in soc_internal.h) that look the GIC up on the
// bus. KermitBlock::install() must have run first.

void kermit_set_cpu(Bus& bus, Cpu* cpu, unsigned core) {
    if (kermit::Gic* gic = find_gic(bus)) gic->set_cpu(cpu, core);
}

void kermit_set_secure_world_left_enabled(Bus& bus, bool value) {
    if (kermit::Gic* gic = find_gic(bus)) gic->set_secure_world_left_enabled(value);
}

bool kermit_irq_line(const Bus& bus) {
    const kermit::Gic* gic = find_gic(bus);
    if (gic == nullptr) return false;
    // The line is re-evaluated on every read/write of the controller, so asking
    // for it has to do the same or the answer would be one update stale.
    const_cast<kermit::Gic*>(gic)->refresh_line();
    return gic->line();
}

void kermit_attach_syscon_spi(Bus& bus, Bus& other, ErnieBlock* ernie) {
    if (ernie == nullptr) return;
    std::vector<Device*> shared;
    for (const auto& device : bus.devices()) {
        const std::string& name = device->name();
        // The three SPI masters (the CMeP second loader drives the syscon link)
        // and the peripheral power/clock windows: the second loader powers each
        // peripheral up itself (0x48810..0x48836 writes 0x101 to +0x64 of
        // 0xE2030000 + index * 0x10000 and waits for the read to clear), because
        // it is the secure processor and the ARM is still held in reset.
        const bool spi = name.rfind("Kermit.Spi", 0) == 0;
        const bool periph_power = name.rfind("Kermit.PeriphPower", 0) == 0;
        // ... and the eMMC host: the second loader reads secure_kernel and
        // kernel_boot_loader through the same SDHCI block the ARM driver uses.
        // Its device descriptor (0x4752C: `movh $7,0xe0b0`) points straight at
        // 0xE0B00000, and the register offsets it programs are the standard ones
        // (0x24 Present State bit 1, 0x28 Host Control 1 bit 2, 0x2C Clock
        // Control with [15:8] = divider, 0x2F Software Reset, 0x34..0x3A the
        // interrupt enable masks). Bus::find_device prefers the smallest window,
        // so this 64 KiB block wins over the CMeP's much larger ScXfer window
        // for the register part and leaves the descriptor page above it alone.
        const bool emmc = name == "Kermit.Sdif0";
        if (!spi && !periph_power && !emmc) continue;
        shared.push_back(device.get());
    }
    cmep_detail::GpioDevice* gpio = nullptr;
    for (const auto& device : other.devices()) {
        if (device->name() == "CMeP.GPIO") gpio = static_cast<cmep_detail::GpioDevice*>(device.get());
    }
    for (Device* device : shared) {
        if (device->name() == "Kermit.Spi0") {
            static_cast<kermit::Spi*>(device)->set_slave(
                [ernie](const std::vector<u8>& request) { return ernie->spi_transfer(request); });
            if (gpio != nullptr) {
                auto* spi0 = static_cast<kermit::Spi*>(device);
                // One shared GPIO object owns input and latches; endpoint
                // wiring alone delivers physical parent248..252 to this GIC.
                gpio->set_irq_callback([&bus](u32 id, bool level) {
                    if (auto* gic = find_gic(bus)) { gic->distributor().set_level(id, level); gic->refresh_line(); }
                });
                spi0->set_syscon_ready_callback([gpio](bool high) { gpio->set_external_input(0x10, high ? 0x10 : 0); });
                gpio->set_reset_callback([spi0] { spi0->reset_syscon_ready_wire(); });
                gpio->set_output_callback([gpio, spi0](u32 direction, u32 output) {
                    u64 mode = 0, mask = 0;
                    gpio->peek_register("MODE_0_15", mode);
                    gpio->peek_register("MASK0", mask);
                    const bool qualified = (direction & 0x10) == 0 &&
                        ((mode >> 8) & 3) == 3 && (mask & 0x10) == 0;
                    spi0->set_syscon_gpio(qualified, (direction & output & 8) != 0);
                });
            }
        }
        // The SDHCI block moves data by ADMA2 on the CMeP side, so it has to be
        // able to read and write that address space as well as the ARM's.
        if (device->name().rfind("Kermit.Sdif", 0) == 0) {
            static_cast<kermit::Sdif*>(device)->set_dma_buses(&bus, &other);
        }
        other.add_device(std::make_unique<DeviceMirror>(*device, device->base(), device->size()));
    }
}

void KermitBlock::attach_syscon_spi(Bus& other, ErnieBlock* ernie) {
    kermit_attach_syscon_spi(impl_->bus, other, ernie);
}

KermitBlock::KermitBlock(Bus& bus, EmmcCard* card) : impl_(std::make_unique<Impl>(bus, card)) {
    Impl& d = *impl_;
    const u32 periph_base = kermit::kScuBase;

    d.bus.add_device(std::make_unique<kermit::ScuRegisters>("Kermit.SCU", periph_base + kermit::kScuRegsOffset,
                                                            kermit::kScuRegsSize));
    d.bus.add_device(std::make_unique<kermit::GlobalTimer>("Kermit.GT", periph_base + kermit::kGlobalTimerOffset,
                                                           kermit::kGlobalTimerSize));
    d.bus.add_device(std::make_unique<kermit::PrivateTimer>("Kermit.PT", periph_base + kermit::kPrivateTimerOffset,
                                                            kermit::kPrivateTimerSize, 0,
                                                            kermit::kIrqPpiPrivateTimer, kermit::kIrqPpiWatchdog));

    auto cpu_interface =
        std::make_unique<kermit::GicCpuInterface>("Kermit.GIC.cpu", periph_base + kermit::kIccOffset, kermit::kIccSize);
    // The GIC's outer window must cover only the MPCore peripheral block, never
    // the whole SCU/DRAM window: Bus::find_device prefers a device over RAM, so a
    // 128 MiB window here shadows the kernel boot loader image at 0x40020000 and
    // every read of it returns zero.
    d.bus.add_device(std::make_unique<kermit::Gic>(kermit::kScuBase, kermit::kPeripheralWindowSize,
                                                   periph_base + kermit::kGicDistOffset, kermit::kGicDistSize,
                                                   periph_base + kermit::kIccOffset, kermit::kIccSize,
                                                   nullptr, std::move(cpu_interface)));

    d.bus.add_device(std::make_unique<kermit::Uart>("Kermit.Uart", kermit::kUartBase, kermit::kUartSize));
    // 0xE2040000 is the *second console* of the firmware's console table, not an
    // SDIF alias: kernel_boot_loader keeps the table at 0x4005C058 and its
    // putchar (0x4003BC88) polls +0x28 (bit 8 = ready) and stores the character
    // at +0x70 in every entry. A write trap over the Kermit window shows ASCII
    // going to both 0xE2030070 and 0xE2040070. The mirror flag keeps the second
    // copy out of the log.
    {
        auto console1 = std::make_unique<kermit::Uart>("Kermit.Uart1", kermit::kSdif0ArmMirror, kermit::kUartSize);
        console1->set_mirror(true);
        d.bus.add_device(std::move(console1));
    }
    d.bus.add_device(std::make_unique<kermit::Sdif>("Kermit.Sdif0", kermit::kSdif0Base, kermit::kSdifSize, card, 0));
    d.bus.add_device(std::make_unique<kermit::Sdif>("Kermit.Sdif1", kermit::kSdif1Base, kermit::kSdifSize, nullptr, 1));
    d.bus.add_device(std::make_unique<kermit::Sdif>("Kermit.Sdif2", kermit::kSdif2Base, kermit::kSdifSize, nullptr, 2));
    // The ADMA2 descriptor walk reads and writes guest physical memory, and the
    // controller is mirrored into the CMeP's address space too (see
    // kermit_attach_syscon_spi), which is where the boot chain's SD driver
    // lives. Both buses see the same DRAM; the model picks whichever maps the
    // table address it was handed.
    for (const char* name : {"Kermit.Sdif0", "Kermit.Sdif1", "Kermit.Sdif2"}) {
        static_cast<kermit::Sdif*>(kermit::detail::find_device(d.bus, name))->set_dma_buses(&d.bus, nullptr);
    }

    // SPI masters. Only SPI0 (the syscon link) has a device behind it; the other
    // two are wired up to the panel/motion side later, and answer with an empty
    // packet meanwhile.
    d.bus.add_device(std::make_unique<kermit::Spi>("Kermit.Spi0", kermit::kSpi0Base, kermit::kSpiSize, 0));
    d.bus.add_device(std::make_unique<kermit::Spi>("Kermit.Spi1", kermit::kSpi1Base, kermit::kSpiSize, 1));
    d.bus.add_device(std::make_unique<kermit::Spi>("Kermit.Spi2", kermit::kSpi2Base, kermit::kSpiSize, 2));

    // Lowio initializes both native I2C windows, even before any slave is used.
    // These controllers implement only idle/reset; transfers remain visibly
    // unsupported and never assert a guessed IRQ or fabricate a slave ACK.
    d.bus.add_device(std::make_unique<kermit::I2cController>("Kermit.I2c0", kermit::kI2c0Base, 0));
    d.bus.add_device(std::make_unique<kermit::I2cController>("Kermit.I2c1", kermit::kI2c1Base, 1));

    // SceDriverTzs SMC117 programs this native CDRAM controller window.
    // Exact initialization commands and mode requests have bounded timing;
    // unknown commands stay busy and calibration IRQ34 remains unattached.
    d.bus.add_device(std::make_unique<kermit::EmcTopController>());

    // PowerVR SGX register window (task part 2 of the Live Area goal). The window
    // identifies itself and implements the command-queue kick handshake; the
    // command buffers themselves and the OpenGL side come next (docs/GPU.md).
    // Its interrupt line is deliberately left unattached until the GPU driver's
    // own interrupt id is recovered from the firmware.
    d.bus.add_device(std::make_unique<kermit::Sgx>("Kermit.SGX", kermit::kSgxBase, kermit::kSgxSize, d.bus));

    // Resolve the accessors by name, so this constructor stays the only place
    // that knows which device object corresponds to which member.
    d.scu = static_cast<kermit::ScuRegisters*>(kermit::detail::find_device(d.bus, "Kermit.SCU"));
    d.global_timer = static_cast<kermit::GlobalTimer*>(kermit::detail::find_device(d.bus, "Kermit.GT"));
    d.private_timer = static_cast<kermit::PrivateTimer*>(kermit::detail::find_device(d.bus, "Kermit.PT"));
    d.gic = static_cast<kermit::Gic*>(kermit::detail::find_device(d.bus, "Kermit.GIC"));
    d.gic->set_access_context(&d.bus.context);
    d.uart = static_cast<kermit::Uart*>(kermit::detail::find_device(d.bus, "Kermit.Uart"));
    d.sdif0 = static_cast<kermit::Sdif*>(kermit::detail::find_device(d.bus, "Kermit.Sdif0"));
    d.sdif1 = static_cast<kermit::Sdif*>(kermit::detail::find_device(d.bus, "Kermit.Sdif1"));
    d.sdif2 = static_cast<kermit::Sdif*>(kermit::detail::find_device(d.bus, "Kermit.Sdif2"));
    d.spi0 = static_cast<kermit::Spi*>(kermit::detail::find_device(d.bus, "Kermit.Spi0"));
    d.spi1 = static_cast<kermit::Spi*>(kermit::detail::find_device(d.bus, "Kermit.Spi1"));
    d.spi2 = static_cast<kermit::Spi*>(kermit::detail::find_device(d.bus, "Kermit.Spi2"));

    // 0xE2040000 is the second entry of the firmware console table (see the
    // device registration above); sdif.elf itself uses 0xE0B00000/0xE0C00000/
    // 0xE0C10000 (its base table at 0x81009FA8).

    d.bus.add_device(std::make_unique<kermit::DmaController>("Kermit.DMA", kermit::kDmaBase, kermit::kDmaSize, d.bus,
                                                             kermit::kDmaChannels));
    d.bus.add_device(std::make_unique<kermit::DisplayController>("Kermit.Display", kermit::kDisplayBase,
                                                                 kermit::kDisplaySize, kermit::kPanelWidth,
                                                                 kermit::kPanelHeight));
    d.bus.add_device(std::make_unique<kermit::IftuController>("Kermit.IFTU0", kermit::kIftu0Base,
                                                              kermit::kIftu0Size, d.bus));
    d.bus.add_device(std::make_unique<kermit::DsiController>());
    d.bus.add_device(std::make_unique<kermit::SysconBridge>("Kermit.Syscon", kermit::kSysconBridgeBase,
                                                            kermit::kSysconBridgeSize));
    // Peripheral power/clock glue and the boot handshake window: both are windows
    // KBL only polls, see PeripheralPower / BootHandshake.  One instance per 64 KiB
    // block, so the register offsets stay the block-relative ones KBL uses.
    for (u32 i = 0; i < kermit::kPeriphPowerCount; ++i) {
        d.bus.add_device(std::make_unique<kermit::PeripheralPower>(
            format("Kermit.PeriphPower%u", i), kermit::kPeriphPowerBase + i * kermit::kPeriphPowerBlock,
            kermit::kPeriphPowerBlock));
    }
    d.bus.add_device(std::make_unique<kermit::BootHandshake>("Kermit.BootHandshake", kermit::kBootHandshakeBase,
                                                             kermit::kBootHandshakeSize));
    // E8000000 remains opaque peripheral glue. E20B6000 is now established
    // as native ThreadMgr LT5, not an empty/drop-write descriptor window.
    d.bus.add_device(std::make_unique<kermit::RegisterBlock>("Kermit.UnkE8000", 0xE8000000, 0x1000));
    // 0xE3320000: the per-core control window the non-secure kernel (NSKBL at
    // 0x51003328, then the kernel's context code at 0x000EC39C) writes.  The
    // kernel's own TTBR1 maps all of 0xE0000000-0xE7FFFFFF as 1 MiB sections, so
    // VA == PA here.  Measured with ZLB_WTRAP over a cold boot: for every core it
    // programs +0x100 = 1 and then +0x000 = 0x87, i.e. 0xE3320000 + 0x1000 * core
    // with a two-register sequence - 214 writes, and (with the 16-bit read trap
    // fixed) not one read.  The meaning of the value is not recovered, so the block
    // only keeps and names the four observed slots instead of dropping the writes,
    // the way Kermit.UnkE8000 keeps the other opaque window.
    {
        auto per_core = std::make_unique<kermit::RegisterBlock>("Kermit.PerCore", 0xE3320000u, 0x4000u);
        for (u32 core = 0; core < 4u; ++core) {
            per_core->define(core * 0x1000u, format("CORE%u_00", core));
            per_core->define(core * 0x1000u + 0x100u, format("CORE%u_100", core));
        }
        d.bus.add_device(std::move(per_core));
    }
    auto lt5 = std::make_unique<kermit::VitaSystemTimer>("Kermit.LT5", kermit::kLt5Base, true, kermit::kIrqLt5);
    auto wt7 = std::make_unique<kermit::VitaSystemTimer>("Kermit.WT7", kermit::kWt7Base, false, kermit::kIrqWt7);
    lt5->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });
    wt7->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });
    d.bus.add_device(std::move(lt5));
    d.bus.add_device(std::move(wt7));
    // 0xE0410000: the DMA window the guest actually drives.  The kernel's own
    // device table (PA 0x40102E58) maps it to physical 0x2802A000, and the DMA
    // library's kick code (0x4381B6..0x438262) writes its descriptor here:
    // +0x10/+0x14 and +0x30/+0x3C carry callbacks, +0x10C/+0x100 a control word,
    // and +0x200..+0x23C the descriptor proper (op pointer 0xFA01F0, buffer
    // 0x7D00F9, size 0x100, code pointers 0x43C000/0x5C1C54).  The window was
    // unmapped, so every one of those writes vanished and the single read at
    // +0x3C came back as 0xFFFFFFFF.  Modelled as storage with the observed
    // offsets: the engine's semantics - and the interrupt that would set the
    // completion event flag ksceKernelDmaOpSync waits on - are not recovered, so
    // this only stops the guest's descriptor from disappearing.  The separate
    // Kermit.DMA block stays at 0xE2060000, where the KBL's console table and
    // docs/HARDWARE.md place it.
    {
        auto dma_win = std::make_unique<DmaWindow>(
            "Kermit.DmaWin", 0xE0410000u, 0x1000u, [this](u32 id) { pulse_irq(id, 2000000); });
        // The engine is modelled as an instant completion: the doorbell clears the
        // busy bit, reports the finished channels and raises the channel interrupt.
        // That is what takes the guest from "ksceKernelDmaOpSync waits forever" to
        // "the twenty-second module start returns 0 and the loop moves on" - see
        // docs/STATUS.md, round 26. What is *not* modelled is the transfer itself:
        // the bytes never move, so a module that reads the transferred buffer sees
        // whatever was there before. ZLB_DMA_IRQ only overrides the interrupt number.
        dma_win->set_enabled(true);
        if (const char* irq_env = std::getenv("ZLB_DMA_IRQ")) {
            dma_win->set_irq(static_cast<u32>(std::strtoul(irq_env, nullptr, 0)));
        }
        if (const char* budget_env = std::getenv("ZLB_DMA_TICKS")) {
            dma_win->set_budget(static_cast<u32>(std::strtoul(budget_env, nullptr, 0)));
        }
        dma_win->define(0x010, "CTRL_010");
        dma_win->define(0x014, "CALLBACK_014");
        dma_win->define(0x020, "DOORBELL");
        // The completion routine 0x438400 reads these two and leaves at 0x438432
        // when ([+0x28] & 3) == 0: they are the engine's transfer-finished status,
        // which the guest never writes. They must be defined or RegisterBlock::write
        // drops the hardware's update.
        dma_win->define(0x024, "STATUS_024");
        dma_win->define(0x028, "STATUS_028");
        dma_win->define(0x02C, "MASK_02C", 0x07FFFFFF);
        dma_win->define(0x030, "CTRL_030");
        dma_win->define(0x03C, "CALLBACK_03C");
        dma_win->define(0x100, "CTRL_100");
        dma_win->define(0x104, "FIELD_104");
        dma_win->define(0x200, "DESC_200");
        dma_win->define(0x204, "DESC_204");
        dma_win->define(0x208, "DESC_208");
        dma_win->define(0x20C, "DESC_20C");
        dma_win->define(0x210, "DESC_210");
        dma_win->define(0x214, "DESC_214");
        dma_win->define(0x218, "DESC_218");
        dma_win->define(0x21C, "DESC_21C");
        dma_win->define(0x220, "DESC_220");
        dma_win->define(0x224, "DESC_224");
        dma_win->define(0x228, "DESC_228");
        dma_win->define(0x22C, "DESC_22C");
        dma_win->define(0x230, "DESC_230");
        dma_win->define(0x234, "DESC_234");
        dma_win->define(0x238, "DESC_238");
        dma_win->define(0x23C, "DESC_23C");
        d.bus.add_device(std::move(dma_win));
    }
    // 0xE04E0000: the companion window of the DMA/secure engine. The kernel's device
    // table maps it to physical 0x2802B000, and the engine driver (module code at
    // 0x3BE5xx) writes +0x400 = 0x200000FF and +0x404 = 0xFFFFFFFF there. The window
    // was unmapped, so both writes vanished. Storage only, like Kermit.DmaWin: the
    // engine's semantics are not recovered.
    {
        auto eng_win = std::make_unique<kermit::RegisterBlock>("Kermit.EngWin", 0xE04E0000u, 0x1000u);
        eng_win->define(0x400, "CFG_400");
        eng_win->define(0x404, "CFG_404");
        d.bus.add_device(std::move(eng_win));
    }
    // Windows the guest really writes but the model left unmapped. They come from
    // the kernel's own device table (PA 0x40102E48: 0x28024000 -> 0xE20BE000,
    // 0x28025000 -> 0xE3000000, 0x28026000 -> 0xE3010000, 0x28027000 -> 0xE5000000,
    // 0x28028000 -> 0xE5010000, 0x28029000 -> 0xE0400000, 0x2802A000 -> 0xE0410000,
    // 0x2802B000 -> 0xE04E0000, 0x2802C000 -> 0xE50C0000) plus the timer blocks the
    // kernel programs at 0xE20B1000 and 0xE20BC000/0xE20BD000. Before this, ZLB_WTRAP
    // showed them as <UNMAPPED>: 70 writes to E20B, 10 to E040, 5 to E010 and 4 to
    // E300/E301 in a single boot. Storage only - the semantics are not recovered.
    struct ExtraWindow {
        const char* name;
        u32 base;
        u32 size;
    };
    // Every window the kernel's device table (PA 0x40621500: 128 {base,size} pairs)
    // declares that the model did not already cover. Before this, a boot wrote 89
    // times into addresses that belonged to no device at all.
    static const ExtraWindow extra_windows[] = {
        // The kernel's table lists the whole 0xE20B timer bank; Kermit.TimerB covers
        // 0xE20B7000, so these two banks cover the rest.
        // The kernel's table lists the timer bank block by block (0x1000 each). They
        // are added individually rather than as two big banks so the overlap check
        // below can skip just the blocks the model already implements (LT5, WT7, PT,
        // TimerB) instead of dropping the whole bank with them.
        {"Kermit.Tmr0", 0xE20B0000u, 0x1000u}, {"Kermit.Tmr1", 0xE20B1000u, 0x1000u},
        {"Kermit.Tmr2", 0xE20B2000u, 0x1000u}, {"Kermit.Tmr3", 0xE20B3000u, 0x1000u},
        {"Kermit.Tmr4", 0xE20B4000u, 0x1000u}, {"Kermit.Tmr5", 0xE20B5000u, 0x1000u},
        {"Kermit.Tmr6", 0xE20B6000u, 0x1000u}, {"Kermit.Tmr8", 0xE20B8000u, 0x1000u},
        {"Kermit.Tmr9", 0xE20B9000u, 0x1000u}, {"Kermit.TmrA", 0xE20BA000u, 0x1000u},
        {"Kermit.TmrB", 0xE20BB000u, 0x1000u}, {"Kermit.TmrC", 0xE20BC000u, 0x1000u},
        {"Kermit.TmrD", 0xE20BD000u, 0x1000u}, {"Kermit.TmrE", 0xE20BE000u, 0x1000u},
        {"Kermit.TmrF", 0xE20BF000u, 0x1000u},
        {"Kermit.Dev20C", 0xE20C0000u, 0x10000u},
        {"Kermit.Dev300", 0xE3000000u, 0x10000u},
        {"Kermit.Dev301", 0xE3010000u, 0x10000u},
        {"Kermit.Dev302", 0xE3020000u, 0x10000u},
        {"Kermit.Dev303", 0xE3030000u, 0x10000u},
        {"Kermit.Dev305", 0xE3050000u, 0x10000u},
        {"Kermit.Dev306", 0xE3060000u, 0x10000u},
        {"Kermit.Dev3101", 0xE3101000u, 0x1000u},
        {"Kermit.Dev3102", 0xE3102000u, 0x1000u},
        {"Kermit.Dev3103", 0xE3103000u, 0x1000u},
        {"Kermit.Dev3104", 0xE3104000u, 0x1000u},
        {"Kermit.Dev3105", 0xE3105000u, 0x1000u},
        {"Kermit.Dev3106", 0xE3106000u, 0x1000u},
        {"Kermit.Dev3108", 0xE3108000u, 0x1000u},
        {"Kermit.Dev3109", 0xE3109000u, 0x1000u},
        {"Kermit.Dev310A", 0xE310A000u, 0x1000u},
        {"Kermit.Dev310B", 0xE310B000u, 0x1000u},
        {"Kermit.Dev310C", 0xE310C000u, 0x1000u},
        {"Kermit.Dev310D", 0xE310D000u, 0x1000u},
        {"Kermit.Dev310E", 0xE310E000u, 0x1000u},
        {"Kermit.Dev310F", 0xE310F000u, 0x1000u},
        {"Kermit.Dev311", 0xE3110000u, 0x10000u},
        {"Kermit.Dev320", 0xE3200000u, 0x6000u},
        {"Kermit.Dev330", 0xE3300000u, 0x1000u},
        {"Kermit.Dev331", 0xE3310000u, 0x10000u},
        {"Kermit.Dev402", 0xE4020000u, 0x1000u},
        {"Kermit.Dev40B", 0xE40B0000u, 0x1000u},
        {"Kermit.Dev40C", 0xE40C0000u, 0x10000u},
        {"Kermit.Dev40D", 0xE40D0000u, 0x10000u},
        {"Kermit.Dev40E", 0xE40E0000u, 0x1000u},
        {"Kermit.Dev40F", 0xE40F0000u, 0x10000u},
        {"Kermit.Dev500", 0xE5000000u, 0x10000u},
        {"Kermit.Dev501", 0xE5010000u, 0x10000u},
        {"Kermit.Dev5020", 0xE5020000u, 0x1000u},
        {"Kermit.Dev5021", 0xE5021000u, 0x1000u},
        {"Kermit.Dev5022", 0xE5022000u, 0x1000u},
        {"Kermit.Dev5030", 0xE5030000u, 0x1000u},
        {"Kermit.Dev5031", 0xE5031000u, 0x1000u},
        {"Kermit.Dev5032", 0xE5032000u, 0x1000u},
        {"Kermit.Dev504", 0xE5040000u, 0x1000u},
        {"Kermit.Dev505", 0xE5050000u, 0x10000u},
        {"Kermit.Dev506", 0xE5060000u, 0x10000u},
        {"Kermit.Dev5070", 0xE5070000u, 0x1000u},
        {"Kermit.Dev5071", 0xE5071000u, 0x1000u},
        {"Kermit.Dev50C", 0xE50C0000u, 0x10000u},
        {"Kermit.Dev50D", 0xE50D0000u, 0x10000u},
        {"Kermit.Dev580", 0xE5800000u, 0x10000u},
        {"Kermit.Dev581", 0xE5810000u, 0x10000u},
        {"Kermit.Dev8400", 0xE8400000u, 0x4000u},
        {"Kermit.Dev8401", 0xE8404000u, 0x4000u},
        {"Kermit.Dev8402", 0xE8408000u, 0x4000u},
        {"Kermit.Dev8403", 0xE840C000u, 0x4000u},
        {"Kermit.Dev8410", 0xE8410000u, 0x4000u},
        {"Kermit.Dev8411", 0xE8414000u, 0x4000u},
        {"Kermit.Dev040", 0xE0400000u, 0x1000u},
        {"Kermit.Dev010", 0xE0100000u, 0x1000u},
    };
    // Several of the kernel's windows already have a real device in the model
    // (kIftu0Base 0xE5020000 with its three planes, kDsi0Base 0xE5050000, the CMeP
    // blocks, the timers, Kermit.PerCore). Adding a storage window over one of those
    // shadows it - the first attempt did exactly that and broke the IFTU tests - so
    // this runs at the very end of install() and skips anything already claimed.
    for (const ExtraWindow& w : extra_windows) {
        if (d.bus.find_device(w.base) != nullptr) continue;
        if (d.bus.find_device(w.base + w.size - 1u) != nullptr) continue;
        d.bus.add_device(std::make_unique<StorageWindow>(w.name, w.base, w.size));
    }

    // 0xE20B7000: a second long-range timer channel, right after LT5.  The window
    // was missing, so the guest's programming fell into unmapped space - measured
    // with ZLB_WTRAP, the kernel writes +0x00/+0x04/+0x08/+0x0C = 0 and +0x14 = 3
    // from pc 0xEA074..0xEA07C during boot, i.e. exactly LT5's counter/deadline/aux
    // offsets.  The guest then never reads it back and never writes CONFIG, so it is
    // modelled as storage with the timer's register names rather than as a second
    // ticking timer: the interrupt it would raise is not known, and inventing one
    // would be a guess (docs/STATUS.md).
    {
        auto timer_b = std::make_unique<kermit::RegisterBlock>("Kermit.TimerB", 0xE20B7000u, 0x1000u);
        timer_b->define(0x00, "COUNTER_LO");
        timer_b->define(0x04, "COUNTER_HI");
        timer_b->define(0x08, "DEADLINE_LO");
        timer_b->define(0x0C, "DEADLINE_HI");
        timer_b->define(0x10, "AUX_LO");
        timer_b->define(0x14, "AUX_HI");
        timer_b->define(0x18, "CONFIG");
        timer_b->define(0x1C, "STATUS");
        d.bus.add_device(std::move(timer_b));
    }
    // Round 142: 0x1D000000 is the hardware /dev/null window (wiki Physical_Memory):
    // SceMsif drains its data stream here.  An empty RegisterBlock is exactly that -
    // reads return 0 (no defined register) and writes are dropped.
    d.bus.add_device(std::make_unique<kermit::RegisterBlock>("Kermit.Null", kermit::kNullDeviceBase,
                                                             kermit::kNullDeviceSize));
    d.dma = static_cast<kermit::DmaController*>(kermit::detail::find_device(d.bus, "Kermit.DMA"));
    d.display = static_cast<kermit::DisplayController*>(kermit::detail::find_device(d.bus, "Kermit.Display"));
    d.iftu = static_cast<kermit::IftuController*>(kermit::detail::find_device(d.bus, "Kermit.IFTU0"));
    d.dsi = static_cast<kermit::DsiController*>(kermit::detail::find_device(d.bus, "Kermit.DSI0"));
    // The display scans its framebuffer out of guest memory (round 191), so it
    // needs the bus the buffers live on.
    if (d.display != nullptr) d.display->set_bus(&d.bus);
    d.syscon_bridge = static_cast<kermit::SysconBridge*>(kermit::detail::find_device(d.bus, "Kermit.Syscon"));

    d.global_timer->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });
    d.private_timer->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });
    d.uart->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });
    d.sdif0->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });
    d.sdif1->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });
    d.sdif2->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });
    d.dma->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });
    d.iftu->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });
    d.dsi->set_frame_callback([&d](u64 frames) { d.iftu->frame_boundary(frames); });
    d.dsi->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });
    d.syscon_bridge->set_irq_callback([&d](u32 id, bool level) { d.raise(id, level); });

    // The DMA engine may address the memory mapped peripherals that have a data
    // port, so it is told about their windows and goes through the devices
    // rather than around them.
    d.dma->add_target(kermit::kSdif0Base, kermit::kSdifSize, d.sdif0);
    d.dma->add_target(kermit::kUartBase, kermit::kUartSize, d.uart);

    d.gic->set_line_callback([&d](bool asserted) {
        if (d.cpu != nullptr) d.cpu->set_irq(static_cast<int>(IrqLine::Irq), asserted);
    });

    ZLB_LOG_INFO("kermit", "installed GIC@0x%08X timer@0x%08X uart@0x%08X sdif0@0x%08X dma@0x%08X display@0x%08X",
                 kermit::kScuBase + kermit::kGicDistOffset, kermit::kScuBase + kermit::kGlobalTimerOffset,
                 kermit::kUartBase, kermit::kSdif0Base, kermit::kDmaBase, kermit::kDisplayBase);
}

KermitBlock::~KermitBlock() = default;

void KermitBlock::install() { impl_->installed = true; }

u64 KermitBlock::total_cycles() const { return impl_->total_cycles; }

void KermitBlock::reset() {
    Impl& d = *impl_;
    d.cycle_accumulator = 0;
    d.total_cycles = 0;
    // Every Kermit device is on the bus, so power-on reset is a bus walk.
    d.bus.reset_devices();
}

void KermitBlock::tick(u64 cycles) {
    Impl& d = *impl_;
    if (cycles == 0) return;
    d.total_cycles += cycles;
    // The CPU runs at 333 MHz and the peripherals at 1 MHz, so a slice of 256
    // CPU cycles is only 0.77 PERIPHCLK ticks. Converting slice by slice
    // truncated that to zero and *no* Kermit device was ever ticked: the timers,
    // the DMA engine and the eMMC host's command-inhibit countdown all stood
    // still. Accumulate the leftover cycles instead.
    d.cycle_accumulator += cycles;
    const u64 ticks = d.periph_ticks(d.cycle_accumulator);
    if (ticks == 0) return;
    d.cycle_accumulator -= kermit::cycles_from_periph_ticks(ticks);
    if (d.cycle_accumulator > cycles * 8) d.cycle_accumulator = 0;  // guard against drift

    for (const auto& device : d.bus.devices()) {
        if (kermit::detail::needs_tick(device->name())) device->tick(ticks);
    }
}

void KermitBlock::raise_irq(u32 id, bool level) {
    impl_->raise(id, level);
    impl_->gic->refresh_line();
}

void KermitBlock::pulse_irq(u32 id, u64 cycles) {
    impl_->gic->distributor().pulse(id, cycles);
    impl_->gic->refresh_line();
}

u32 KermitBlock::pending_irq_count() const { return impl_->gic->distributor().pending_count(); }

std::string KermitBlock::gic_summary() const { return impl_->gic->summary(); }

std::string KermitBlock::take_uart_output() { return impl_->uart->take_output(); }

bool KermitBlock::uart_has_output() const { return impl_->uart->has_output(); }

void KermitBlock::uart_write(const std::string& text) { impl_->uart->push_rx(text); }

u64 KermitBlock::emmc_transfers() const { return impl_->sdif0 ? impl_->sdif0->transfers() : 0; }

u64 KermitBlock::last_emmc_lba() const { return impl_->sdif0 ? impl_->sdif0->last_lba() : 0; }

u32 KermitBlock::last_emmc_count() const { return impl_->sdif0 ? impl_->sdif0->last_count() : 0; }

const u8* KermitBlock::framebuffer(int& width, int& height, int& stride,
                                 int* bytes_per_pixel) const {
    if (impl_->iftu) {
        const u8* pixels = impl_->iftu->framebuffer(width, height, stride, bytes_per_pixel);
        if (pixels) return pixels;
    }
    return impl_->display->framebuffer(width, height, stride, bytes_per_pixel);
}

u64 KermitBlock::frame_counter() const {
    int width = 0, height = 0, stride = 0;
    // Native scanout has no fabricated frame-completion timer or counter.
    if (impl_->iftu && impl_->iftu->framebuffer(width, height, stride)) return 0;
    return impl_->display->frame_counter();
}

void KermitBlock::describe(std::vector<std::string>& lines) const {
    const Impl& d = *impl_;
    lines.push_back("Kermit: ARM Cortex-A9 MPCore (1 core @ 333 MHz, PERIPHCLK 1 MHz)");
    lines.push_back(format("  memory: boot ROM 0x%08X/%u KiB, SRAM 0x%08X/%u KiB, DRAM 0x%08X/%u MiB, private "
                           "0x%08X/%u MiB",
                           kermit::kBootRomBase, kermit::kBootRomSize / KB, kermit::kSramBase,
                           kermit::kSramSize / KB, kermit::kDramBase, kermit::kDramSize / MB, kermit::kScuBase,
                           kermit::kScuSize / MB));
    lines.push_back(format("  private region: SCU +0x%03X, CPU interface +0x%03X, global timer +0x%03X, private "
                           "timer +0x%03X, GIC +0x%03X",
                           kermit::kScuRegsOffset, kermit::kIccOffset, kermit::kGlobalTimerOffset,
                           kermit::kPrivateTimerOffset, kermit::kGicDistOffset));
    if (d.gic) d.gic->describe(lines);
    if (d.global_timer) d.global_timer->describe(lines);
    if (d.private_timer) d.private_timer->describe(lines);
    if (d.uart) d.uart->describe(lines);
    if (d.sdif0) d.sdif0->describe(lines);
    if (d.dma) d.dma->describe(lines);
    if (d.display) d.display->describe(lines);
    if (d.iftu) d.iftu->describe(lines);
    if (d.dsi) d.dsi->describe(lines);
    if (d.syscon_bridge) d.syscon_bridge->describe(lines);
    lines.push_back(format("  time: %llu CPU cycles = %.6f s", static_cast<unsigned long long>(d.total_cycles),
                           static_cast<double>(d.total_cycles) / kermit::kCpuClockHz));
}

std::string KermitBlock::summary() const {
    const Impl& d = *impl_;
    int width = 0;
    int height = 0;
    int stride = 0;
    const u8* pixels = framebuffer(width, height, stride);
    (void)stride;
    return format("Kermit ARM SoC: GIC %u pending, %s, %llu eMMC transfers, display %s", pending_irq_count(),
                  d.uart && d.uart->has_output() ? "UART has output" : "UART idle",
                  static_cast<unsigned long long>(emmc_transfers()),
                  pixels ? format("%dx%d", width, height).c_str() : "off");
}

std::vector<Device*> KermitBlock::devices() const {
    const Impl& d = *impl_;
    std::vector<Device*> out;
    for (const auto& device : d.bus.devices()) {
        if (kermit::detail::is_owned(device->name())) out.push_back(device.get());
    }
    return out;
}

void KermitBlock::save_state(StateWriter& writer) const {
    const Impl& d = *impl_;
    // The devices Impl owns are registered on the bus, which serialises each
    // one where it lives; writing them again here would apply them twice on
    // load. Only the block's own scheduler state travels in this section:
    // `bus`, `card`, the device/core pointers and `ernie_` are host wiring.
    writer.put_bool(d.installed);
    writer.put_u64(d.cycle_accumulator);
    writer.put_u64(d.total_cycles);
}

void KermitBlock::load_state(StateReader& reader) {
    Impl& d = *impl_;
    d.installed = reader.get_bool();
    d.cycle_accumulator = reader.get_u64();
    d.total_cycles = reader.get_u64();
}

}  // namespace zlb
