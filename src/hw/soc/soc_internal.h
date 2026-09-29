// zeliboba - Kermit peripheral block internals.
//
// The private class declarations for the devices that KermitBlock owns. They
// are deliberately kept next to the implementation (this header is private to
// src/hw/soc/*.cpp): the public surface is src/hw/soc.h.
//
// Register addresses come from three sources:
//
//  1. the ARM Cortex-A9 MPCore TRM (datasheets/cortex_a9_mpcore_trm_100486_0401_10_en.pdf),
//     which fixes the whole private memory region: the base is the PERIPHBASE
//     pin and chapter 1.5 "Private Memory Region" (page 1-17) lists the
//     offsets, chapter 3.3/3.4 the GIC registers and chapter 4.2/4.4 the
//     private timer / watchdog / global timer registers;
//  2. the frozen C# device models that already exist in this workspace
//     (VitaTestSuite/Core/Devices/VitaDevices.cs), which place the ARM
//     peripheral windows at 0xE2000000 (syscon bridge), 0xE2030000 (UART)
//     and 0xE20A0000 (GPIO) - see DeviceSetup.Install;
//  3. the decrypted 1.04 kernel modules under
//     Vita_104_Firmware/Out/fs_dec/os0/kd/.
//
// Every address that is not backed by (1) or (2) is marked ASSUMPTION below and
// has its own named constant so it can be corrected in one place.
#pragma once

#include <array>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "bus/device.h"
#include "common/log.h"
#include "common/types.h"
#include "common/util.h"
#include "cpu/cpu.h"
#include "hw/emmc.h"
#include "hw/soc.h"
#include "hw/syscon.h"
namespace zlb::kermit {

// ---------------------------------------------------------------------------
// Clocking
// ---------------------------------------------------------------------------

/// Kermit (Cortex-A9 MPCore) core clock. The Vita SoC runs the A9 cluster at
/// 333 MHz; the value only has to be self consistent, it sets the conversion
/// between Cpu::cycles and wall clock time. (ASSUMPTION: 333 MHz is the
/// commonly quoted figure; the TRM only says the timers are clocked from
/// PERIPHCLK.)
constexpr u32 kCpuClockHz = 333000000;

/// PERIPHCLK, the clock that feeds the global timer, the private timers and
/// the watchdog. ZLB_CLOCK: set to 1 MHz so the global timer's 64 bit counter
/// is a microsecond counter, which is what the debugger and the tests want.
constexpr u32 kPeriphClockHz = 1000000;

// ---------------------------------------------------------------------------
// MPCore private memory region (TRM 1.5, page 1-17, base = PERIPHBASE)
// ---------------------------------------------------------------------------

constexpr u32 kScuRegsOffset = 0x0000;      // SCU registers
constexpr u32 kScuRegsSize = 0x0100;
constexpr u32 kIccOffset = 0x0100;          // per-core interrupt interface
constexpr u32 kIccSize = 0x0100;
constexpr u32 kGlobalTimerOffset = 0x0200;  // global timer
constexpr u32 kGlobalTimerSize = 0x0100;
constexpr u32 kPrivateTimerOffset = 0x0600; // private timer + watchdog
constexpr u32 kPrivateTimerSize = 0x0100;
constexpr u32 kGicDistOffset = 0x1000;      // interrupt distributor
constexpr u32 kGicDistSize = 0x1000;
// The TRM also documents a 4 KiB "CPU interface" page at 0x2000 (the generic
// GIC architecture's own offset) in addition to the 0x0100 page. The firmware
// uses one of the two; both are mapped, one as an alias of the other.
constexpr u32 kIccAliasOffset = 0x2000;

/// Addresses used by the C# reference models (VitaDevices.cs DeviceSetup) and
/// therefore also by the ARM sandbox in this workspace.
constexpr u32 kSysconBridgeBase = 0xE2000000;  // VitaDevices: ARM.SysconBridge
constexpr u32 kSysconBridgeSize = 0x1000;
constexpr u32 kUartBase = 0xE2030000;          // VitaDevices: ARM.Uart
constexpr u32 kUartSize = 0x100;

/// SD/eMMC host controllers (SDIF). sdif.elf contains a table at
/// 0x81009FA8 holding 0xE0B00000 / 0xE0C00000 / 0xE0C10000 together with the
/// strings "SceSdif0/1/2" and "%d:0x%08x:0x%08x:%d:Skip sdif port[%d] reset";
/// the first entry is therefore SDIF port 0 (the eMMC port the kernel uses).
/// The same three values appear in sysmem.elf at 0x810314E0. (FACT: the values
/// and the strings. INFERENCE: they are the three ports; only port 0 is
/// modelled, the other two are modelled as empty windows.)
constexpr u32 kSdif0Base = 0xE0B00000;
constexpr u32 kSdif1Base = 0xE0C00000;
constexpr u32 kSdif2Base = 0xE0C10000;
constexpr u32 kSdif3Base = 0xE0C20000;
/// sysmem.elf records each SDIF port as a 64 KiB resource (table at
/// 0x810314E0: {0xE0B00000, 0x10000} {0xE0C00000, 0x10000} {0xE0C10000, 0x10000}
/// {0xE0C20000, 0x10000}), so the window is 0x10000 and not 0x1000.
constexpr u32 kSdifSize = 0x10000;
/// Mirror of SDIF0 inside the ARM peripheral window. ASSUMPTION: the exact ARM
/// alias is unknown, this one only exists so that a driver using the
/// "0xE2xxxxxx" convention still reaches the controller.
constexpr u32 kSdif0ArmMirror = 0xE2040000;
/// The seven 64 KiB windows KBL configures before it touches the eMMC
/// (0x4005C058 = {0xE2030000, 0xE2040000, ... 0xE2090000}).
constexpr u32 kPeriphPowerBase = 0xE2030000;
constexpr u32 kPeriphPowerBlock = 0x00010000;   ///< one 64 KiB window per device
constexpr u32 kPeriphPowerCount = 7;            ///< 0xE2030000 .. 0xE2090000
/// KBL's boot handshake window (0x400217FA..0x40021872).
constexpr u32 kBootHandshakeBase = 0xE5888000;
constexpr u32 kBootHandshakeSize = 0x1000;

/// SPI masters. The SoC has three identical SPI blocks (VitaDevWiki
/// "SPI Registers", which is also where the register map below comes from):
///
///   0xE0A00000  SceSpi0Reg  - the syscon (Ernie) link, the only one the boot
///                             chain uses; the CMeP second loader drives it
///                             (SPI transfer at 0x436E4) and so does the ARM
///                             syscon.elf driver (0x8100003C)
///   0xE0A10000  SceSpi1Reg  - motion sensor
///   0xE0A20000  SceSpi2Reg  - OLED/LCD
///
/// The CMeP's own view of the block is the same address, which is why the
/// device lives in the shared bus rather than in the ARM-only part.
constexpr u32 kSpi0Base = 0xE0A00000;
constexpr u32 kSpi1Base = 0xE0A10000;
constexpr u32 kSpi2Base = 0xE0A20000;
constexpr u32 kSpiSize = 0x10000;

/// DMA controller. ASSUMPTION: no register-level evidence was recovered from
/// dmacmgr.elf, so this is a fresh window in the free part of the ARM
/// peripheral space (0xE2000000-0xE20A0000 in the C# map).
constexpr u32 kDmaBase = 0xE2060000;
constexpr u32 kDmaSize = 0x1000;
constexpr u32 kDmaChannels = 4;

/// Display controller. ASSUMPTION: same situation as the DMA controller. The
/// panel itself is fixed by the hardware documentation (CXD5315GG, 960x544).
constexpr u32 kDisplayBase = 0xE2100000;
constexpr u32 kDisplaySize = 0x1000;
constexpr int kPanelWidth = 960;
constexpr int kPanelHeight = 544;

/// IRQ ids of the private peripheral interrupts (PPI). TRM 4.2.3: the private
/// timer raises ID 29, and 4.3: the global timer comparator raises ID 27.
constexpr u32 kIrqPpiGlobalTimer = 27;
constexpr u32 kIrqPpiPrivateTimer = 29;
constexpr u32 kIrqPpiWatchdog = 30;
constexpr u32 kGicMaxIrq = 128;

}  // namespace zlb::kermit

// Extensions to the public KermitBlock surface that hw/soc.h does not declare
// but that the machine layer and the tests need (the task explicitly asks for a
// way to read the current IRQ line state and to connect the CPU). hw/soc.h
// belongs to another workstream and must not be touched, so these are free
// functions declared here and defined in kermit.cpp instead of new members.
namespace zlb {

class KermitBlock;
class Cpu;

/// Connect the Cortex-A9 core so that GIC interrupts reach Cpu::set_irq(line 0).
/// The block must have been installed on the bus first (KermitBlock::install);
/// this looks the interrupt controller up by name, so it works without touching
/// a private member of KermitBlock (hw/soc.h cannot be modified).
void kermit_set_cpu(Bus& bus, Cpu* cpu);
/// Current state of the CPU's IRQ input line (IrqLine::Irq, i.e. line 0), or
/// false when the block is not installed.
bool kermit_irq_line(const Bus& bus);

/// Attach the syscon to SPI0 (0xE0A00000) and mirror the SPI masters into
/// `other`, the second processor's bus. Every SPI transfer the guest starts is
/// handed to ErnieBlock::spi_transfer, which is how the CMeP second loader
/// (0x436E4) and the ARM syscon module (0x8100003C) reach the syscon. One Spi
/// object serves both buses, exactly like the single hardware block does.
void kermit_attach_syscon_spi(Bus& bus, Bus& other, ErnieBlock* ernie);

}  // namespace zlb

namespace zlb::kermit {

// ---------------------------------------------------------------------------
// Register plumbing
// ---------------------------------------------------------------------------

/// Convert CPU cycles into PERIPHCLK ticks (the unit every device's `tick()`
/// works in). The conversion is the only place the two constants above meet:
/// one PERIPHCLK tick is kCpuClockHz / kPeriphClockHz core cycles.
inline u64 periph_ticks_from_cycles(u64 cycles) {
    return cycles * kPeriphClockHz / kCpuClockHz;
}

/// The reciprocal, used by the timer models for their prescaler arithmetic.
inline u64 cycles_from_periph_ticks(u64 ticks) {
    return (ticks * kCpuClockHz + kPeriphClockHz - 1) / kPeriphClockHz;
}

/// A byte-addressable register block.
///
/// `Bus` implements sub-word MMIO by splitting the access into single byte
/// accesses to `Device::read(address + i, 1)`, so devices must decode any
/// address, not just the aligned word ones. This base class keeps the little
/// endian word values in a map and implements the byte plumbing once; the
/// concrete devices override `read_word`/`write_word` for the registers that
/// have side effects.
class RegisterBlock : public Device {
public:
    RegisterBlock(std::string name, u32 base, u32 size) : Device(std::move(name), base, size) {}

    /// Declare a register. `width` is in bytes (4 unless told otherwise).
    void define(u32 offset, const std::string& name, u64 reset_value = 0, unsigned width = 4);
    /// Declare a byte-mapped register whose value is a packed vector of bytes.
    void define_bytes(u32 offset, const std::string& name, const char* text, unsigned length);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;

    /// The debugger's MMIO view asks the device for the name of the register it
    /// just accessed; every RegisterBlock defines its registers by offset, so
    /// the lookup is defined here once.
    const char* register_name(u32 address) const override;

    void reset() override;

    u64 peek(u32 offset) const;
    void poke(u32 offset, u64 value);
    /// The effective value of a register: what `peek` would return if the map
    /// were wide enough. Registers wider than 32 bits (the global timer, the
    /// DMA addresses) live here, and `poke` keeps its low 32 bits in the byte
    /// image so the byte-level fallback paths keep working.
    u64 store(u32 offset) const;
    void store(u32 offset, u64 value);

    void enumerate_registers(std::vector<RegisterInfo>& out) const override;
    bool peek_register(const std::string& name, u64& out) const override;
    bool poke_register(const std::string& name, u64 value) override;

protected:
    /// Called for a whole-word access to a defined register. This is where a
    /// device decodes a register and applies the side effects of a write; the
    /// base class has already updated the byte image with the value the caller
    /// stored, so `stored` (and store()) is always current for a 4 byte access.
    virtual u64 read_word(u32 offset, u64 stored) { (void)offset; return stored; }
    virtual void write_word(u32 offset, u64 value) { (void)offset; (void)value; }

    bool is_fifo(u32 offset) const { return fifos_.count(offset) != 0; }
    bool is_defined(u32 offset) const { return values_.count(offset) != 0; }

private:
    std::map<u32, u64> values_;
    std::map<u32, u64> defaults_;
    std::map<u32, unsigned> widths_;
    std::map<std::string, u32> by_name_;
    std::map<u32, u8> fifos_;
    std::map<u32, u64> store_;
};

// ---------------------------------------------------------------------------
// GIC (Cortex-A9 interrupt controller)
// ---------------------------------------------------------------------------

/// One Cortex-A9 processor interrupt interface (TRM 3.4.1, page 3-60).
class GicCpuInterface : public RegisterBlock {
public:
    GicCpuInterface(std::string name, u32 base, u32 size);

    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;

    // State the distributor/CPU side of the model needs.
    bool enabled() const { return (peek(0x000) & 1) != 0; }
    u8 priority_mask() const { return static_cast<u8>(peek(0x004) & 0xFF); }
    /// Take the highest priority pending interrupt, or 1023 when there is none.
    u32 acknowledge();
    void end_of_interrupt(u32 id);
    u32 running_priority() const;
    u32 highest_pending() const;

    void set_sources(std::function<u32()> highest_pending_source);
    void set_ack_handler(std::function<u32()> handler);
    void set_eoi_handler(std::function<void(u32)> handler);

    std::string summary() const override;

private:
    std::function<u32()> highest_pending_;
    std::function<u32()> acknowledge_;
    std::function<void(u32)> eoi_;
};

/// The Cortex-A9 interrupt distributor (TRM 3.3, page 3-51).
class GicDistributor : public RegisterBlock {
public:
    GicDistributor(std::string name, u32 base, u32 size, u32 max_irq);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;
    void reset() override;
    /// Rebuild the enable/pending/active status registers from the model state.
    void sync_status_registers();

    void set_level(u32 id, bool level);
    void pulse(u32 id, u64 ticks);
    void clear_pending(u32 id);

    bool enabled(u32 id) const;
    bool pending(u32 id) const;
    bool active(u32 id) const;
    u16 priority(u32 id) const;
    u8 targets(u32 id) const;
    bool edge_triggered(u32 id) const;
    /// True when the distributor would signal an interrupt for this CPU.
    bool line_asserted(u32 core, u8 priority_mask) const;
    /// Highest priority pending interrupt id for `core`, or 1023.
    u32 highest_pending(u32 core, u8 priority_mask) const;
    u32 acknowledge(u32 core);
    void end_of_interrupt(u32 core, u32 id);
    u32 pending_count() const;
    void set_default_priority(u32 id, u8 value);
    /// Consume `ticks` from every pending pulse counter. Returns true when any
    /// line was deasserted as a result.
    bool tick_pulses(u64 ticks);

    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

private:
    u32 max_irq_;
    std::vector<bool> enabled_;
    std::vector<bool> pending_;
    std::vector<bool> active_;
    std::vector<bool> level_;      ///< level triggered when true
    std::vector<bool> sampled_;    ///< last sampled level for edge detection
    std::vector<bool> pending_after_eoi_;  ///< an edge arrived while active
    std::vector<u8> priority_;
    std::vector<u8> targets_;
    std::vector<u32> pulse_left_;
};

/// The GIC as one debugger-visible device. It covers the whole MPCore private
/// region so that the per-core interface pages are routed to it (the SCU, the
/// timers and the distributor are separate, smaller devices in the same
/// region, and the bus prefers the smallest match). Only CPU0's interface is
/// modelled.
class Gic : public Device {
public:
    Gic(u32 region_base, u32 region_size, u32 distrib_base, u32 distrib_size, u32 cpuif_base, u32 cpuif_size,
        std::unique_ptr<Device> cpuif_alias, std::unique_ptr<GicCpuInterface> cpuif);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;
    void tick(u64 cycles) override;

    const char* register_name(u32 address) const override;
    void enumerate_registers(std::vector<RegisterInfo>& out) const override;
    bool peek_register(const std::string& name, u64& out) const override;
    bool poke_register(const std::string& name, u64 value) override;
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    GicDistributor& distributor() { return *distributor_; }
    GicCpuInterface& cpu_interface() { return *cpu_interface_; }

    /// Set when the CPU's IRQ input should change.
    void set_line_callback(std::function<void(bool)> callback) { line_callback_ = std::move(callback); }
    /// Connect a core: the IRQ line is asserted/deasserted through
    /// Cpu::set_irq(IrqLine::Irq) and the current state is applied immediately.
    void set_cpu(Cpu* cpu);
    bool line() const { return line_; }
    /// Re-evaluate the IRQ line and notify the CPU when it changed.
    void refresh_line();

private:
    std::unique_ptr<GicDistributor> distributor_;
    std::unique_ptr<GicCpuInterface> cpu_interface_;
    std::unique_ptr<Device> cpu_interface_alias_;
    std::vector<u32> active_;  ///< stack of acknowledged-but-not-EOI'd ids
    std::function<void(bool)> line_callback_;
    Cpu* cpu_ = nullptr;
    bool line_ = false;
};

// ---------------------------------------------------------------------------
// Timers
// ---------------------------------------------------------------------------

/// The Cortex-A9 global timer (TRM 4.4, page 4-71). 64 bit up counter clocked
/// by PERIPHCLK with a banked comparator.
class GlobalTimer : public RegisterBlock {
public:
    GlobalTimer(std::string name, u32 base, u32 size);

    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;
    void tick(u64 cycles) override;
    void reset() override;

    /// Set when the comparator fires (the model raises PPI 27).
    void set_irq_callback(std::function<void(u32, bool)> callback) { irq_ = std::move(callback); }

    u64 counter() const { return counter_; }
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

private:
    void refresh_irq();

    u64 counter_ = 0;
    u64 comparator_ = 0;
    u64 accumulator_ = 0;  ///< fractional PERIPHCLK cycles
    bool enabled_ = false;
    bool irq_state_ = false;
    std::function<void(u32, bool)> irq_;
};

/// One Cortex-A9 private timer plus its watchdog (TRM 4.2, page 4-64).
class PrivateTimer : public RegisterBlock {
public:
    PrivateTimer(std::string name, u32 base, u32 size, u32 core, u32 timer_irq, u32 watchdog_irq);

    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;
    void tick(u64 cycles) override;
    void reset() override;

    void set_irq_callback(std::function<void(u32, bool)> callback) { irq_ = std::move(callback); }
    u32 core() const { return core_; }
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

private:
    struct Counter {
        u32 load = 0;
        u32 value = 0;
        bool running = false;
        bool auto_reload = false;
        bool irq_enable = false;
        u8 prescaler = 0;
        u32 prescale_left = 0;
        bool event = false;  ///< interrupt status flag
    };

    void step(Counter& counter, u32 id, u64 periph_ticks);
    void refresh(Counter& counter, u32 id);

    u32 core_;
    u32 timer_irq_;
    u32 watchdog_irq_;
    u64 accumulator_ = 0;
    u64 expiries_ = 0;
    Counter timer_;
    Counter watchdog_;
    bool watchdog_reset_status_ = false;
    bool watchdog_disabled_ = false;
    std::function<void(u32, bool)> irq_;
};

// ---------------------------------------------------------------------------
// UART
// ---------------------------------------------------------------------------

/// 16550-ish debug UART. The Vita uses a small serial block for the kernel
/// printf console; the register set below follows the 16550 layout because the
/// C# reference model reserves a 0x100 byte window at 0xE2030000 and the
/// driver in lowio.elf does not contradict it. (ASSUMPTION: exact register
/// semantics are unverified against the firmware; the offsets are the classic
/// 16550 ones with a 4 byte register stride.)
class Uart : public RegisterBlock {
public:
    Uart(std::string name, u32 base, u32 size);

    void reset() override;

    /// Mirror console (0xE2040000): capture the bytes but do not log them, the
    /// KBL writes every character to both consoles and the log would double.
    void set_mirror(bool mirror) { mirror_ = mirror; }

    void push_rx(u8 byte);
    void push_rx(const std::string& text);
    /// Take everything transmitted since the last call.
    std::string take_output();
    bool has_output() const { return !output_.empty(); }
    void set_irq_callback(std::function<void(u32, bool)> callback) { irq_callback_ = std::move(callback); }
    bool irq_line() const { return irq_line_; }

    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

protected:
    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;

private:
    static constexpr u32 kRegCount = 8;
    struct Regs {
        u8 ier = 0;
        u8 fcr = 0;
        u8 lcr = 0;
        u8 mcr = 0;
        u8 scr = 0;
        u8 msr = 0xB0;
        u16 divisor = 0;
    };

    void update_irq();
    /// Refresh the line status register from the FIFO/overrun state.
    void update_status();
    /// The transmit/receive register (0x00) is a FIFO port, not a register.
    u8 read_data_register();
    void write_data_register(u8 value);
    u32 reg_offset(u32 offset) const { return (offset >> 2) & (kRegCount - 1); }

    Regs r_;
    std::deque<u8> rx_;
    std::string output_;
    std::string log_line_;
    bool irq_line_ = false;
    bool overrun_ = false;
    bool mirror_ = false;
    u64 bytes_tx_ = 0;
    u64 bytes_rx_ = 0;
    std::function<void(u32, bool)> irq_callback_;
};

// ---------------------------------------------------------------------------
// SD/eMMC host controller (SDIF)
// ---------------------------------------------------------------------------

/// SDHCI-compatible SD/eMMC host controller. See soc_internal.h header comment
/// for the evidence for the base address; the register map is the SD Host
/// Controller Simplified Specification one, which is what the 0x00-0x3F
/// register block of every controller of this family implements.
class Sdif : public RegisterBlock {
public:
    Sdif(std::string name, u32 base, u32 size, EmmcCard* card, u32 port);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;
    void tick(u64 cycles) override;

    void set_irq_callback(std::function<void(u32, bool)> callback) { irq_callback_ = std::move(callback); }
    bool irq_line() const { return irq_line_; }

    u64 transfers() const { return transfers_; }
    u64 last_lba() const { return last_lba_; }
    u32 last_count() const { return last_count_; }
    /// Offset of the command register (used by the build-soc self test).
    static constexpr u32 command_offset() { return 0x0E; }
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    /// Physical memory used by the ADMA2 descriptor walk. The controller is
    /// mirrored into two address spaces (the ARM's and the CMeP's), so the
    /// machine hands over both buses and the model picks whichever one maps the
    /// descriptor table. Both see the same DRAM, so either moves the same bytes.
    void set_dma_buses(Bus* primary, Bus* secondary) {
        dma_primary_ = primary;
        dma_secondary_ = secondary;
    }

protected:
    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;

private:
    void execute_command();
    void finish_transfer(bool ok);
    /// The buffer data port (0x20) is a byte FIFO rather than a register.
    u8 read_data_port();
    void write_data_port(u8 value);
    u64 current_lba() const;
    bool use_adma() const;
    /// ADMA2 descriptor walk (SD Host Controller Simplified Specification 2.0,
    /// section 2.2.13): the table is a chain of 8 byte records
    /// `{u16 attribute, u16 length, u32 address}` in guest physical memory.
    /// `read` moves `payload` from the card into memory, otherwise the bytes are
    /// read out of memory into `sink` and pushed to the card by the caller.
    bool adma_transfer(bool read, const std::vector<u8>& payload, std::vector<u8>* sink);
    /// Whichever bus maps `address`, or nullptr when neither does.
    Bus* dma_bus_for(u32 address, u32 length) const;
    void update_irq();

    EmmcCard* card_ = nullptr;
    u32 port_ = 0;
    bool irq_line_ = false;
    std::function<void(u32, bool)> irq_callback_;

    u64 transfers_ = 0;
    u64 last_lba_ = 0;
    u32 last_count_ = 0;
    std::string last_command_;

    // Response of the last command, little endian, 4 words.
    std::array<u32, 4> response_{};
    std::vector<u8> data_;
    u32 data_index_ = 0;
    bool data_pending_ = false;
    bool read_direction_ = false;
    bool command_ok_ = true;
    u32 rca_ = 0;
    /// SEND_OP_COND calls seen since the last GO_IDLE_STATE: the first one
    /// reports "power up in progress" (OCR bit 31) like a real card.
    u32 op_cond_polls_ = 0;
    /// PERIPHCLK ticks left before the data phase becomes visible (Buffer
    /// Read/Write Ready is raised a few ticks after Command Complete, the way a
    /// real controller does; see Sdif::execute_command()).
    u32 data_ready_in_ = 0;
    /// PERIPHCLK ticks left before Transfer Complete is reported after the last
    /// data byte was consumed.
    u32 transfer_complete_in_ = 0;
    /// Set while the pending deferred Transfer Complete belongs to an ADMA2
    /// transfer, so the interrupt is reported together with the DMA bit.
    bool dma_complete_ = false;
    Bus* dma_primary_ = nullptr;
    Bus* dma_secondary_ = nullptr;
    /// Diagnostics for the last descriptor walk (see describe()).
    u32 adma_address_ = 0;
    u32 adma_bytes_ = 0;
    bool adma_ok_ = false;
    bool app_cmd_ = false;
    bool high_capacity_ = true;
    u32 busy_left_ = 0;
    u32 last_command_register_ = 0;
};

// ---------------------------------------------------------------------------
// SPI master (syscon / motion / OLED links)
// ---------------------------------------------------------------------------

/// SPI master block. Register map (VitaDevWiki "SPI Registers", cross checked
/// against the two drivers in the boot chain: CMeP second loader 0x436E4 and
/// ARM syscon.elf 0x8100003C - both use the same offsets):
///
///   0x00  SPI_RXFIFO          read: pop a 16 bit word from the receive FIFO
///   0x04  SPI_TXFIFO          write: push a 16 bit word into the transmit FIFO
///   0x08  SPI_CTL             configuration (the drivers write 0 before starting)
///   0x0C  SPI_INTCTL          interrupt control
///   0x10  SPI_STATUS          write bit 0 = 1 starts a transfer; read bit 0 = busy
///   0x14  SPI_DMACTL          DMA control
///   0x18                      start/direction, not used by the boot chain
///   0x20                      cleared by the second loader before a transfer
///   0x24  SPI_INT_STATUS      bit 9 = "RX FIFO not empty", write 1 to clear (the
///                             drivers write 0x600, i.e. bits 9 and 10)
///   0x28  SPI_RXFIFO_STATUS   number of bytes available to read
///   0x2C  SPI_TXFIFO_STATUS   number of bytes still pending in the TX FIFO
///
/// The transfer itself is delegated to an attached slave: when software sets
/// the start bit the accumulated transmit bytes are clocked out and the slave's
/// answer is queued into the receive FIFO. The block is byte oriented on the
/// outside (0x28 counts bytes, FIFO accesses are 16 bit words, low byte first)
/// because that is how both drivers count the reply.
class Spi : public RegisterBlock {
public:
    /// The slave side: consume the transmitted bytes, produce the answer.
    using Transfer = std::function<std::vector<u8>(const std::vector<u8>&)>;

    Spi(std::string name, u32 base, u32 size, u32 port);

    void set_slave(Transfer transfer) { slave_ = std::move(transfer); }
    u32 port() const { return port_; }

    void reset() override;
    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;

    void set_irq_callback(std::function<void(u32, bool)> callback) { irq_callback_ = std::move(callback); }
    bool irq_line() const { return irq_line_; }

    u64 transfers() const { return transfers_; }
    u64 bytes_tx() const { return bytes_tx_; }
    u64 bytes_rx() const { return bytes_rx_; }
    const std::vector<u8>& last_request() const { return last_request_; }
    const std::vector<u8>& last_response() const { return last_response_; }
    size_t rx_pending() const { return rx_.size(); }

    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    static constexpr u32 kRxFifo = 0x00;
    static constexpr u32 kTxFifo = 0x04;
    static constexpr u32 kCtl = 0x08;
    static constexpr u32 kIntCtl = 0x0C;
    static constexpr u32 kStatus = 0x10;
    static constexpr u32 kDmaCtl = 0x14;
    static constexpr u32 kReg18 = 0x18;
    static constexpr u32 kReg20 = 0x20;
    static constexpr u32 kIntStatus = 0x24;
    static constexpr u32 kRxFifoStatus = 0x28;
    static constexpr u32 kTxFifoStatus = 0x2C;
    static constexpr u32 kIntStatusRxNotEmpty = 1u << 9;

private:
    void start_transfer();
    void update_irq();

    u32 port_;
    std::vector<u8> tx_;      ///< bytes handed to the slave so far
    std::deque<u8> rx_;       ///< bytes the slave returned
    u32 ctl_ = 0;
    u32 int_ctl_ = 0;
    u32 dma_ctl_ = 0;
    u32 reg18_ = 0;
    u32 reg20_ = 0;
    u32 int_status_ = 0;
    bool busy_ = false;
    bool irq_line_ = false;
    Transfer slave_;
    std::function<void(u32, bool)> irq_callback_;
    u64 transfers_ = 0;
    u64 bytes_tx_ = 0;
    u64 bytes_rx_ = 0;
    std::vector<u8> last_request_;
    std::vector<u8> last_response_;
};

// ---------------------------------------------------------------------------
// DMA controller
// ---------------------------------------------------------------------------

class DmaController : public Device {
public:
    DmaController(std::string name, u32 base, u32 size, Bus& bus, u32 channels);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;
    void tick(u64 cycles) override;

    const char* register_name(u32 address) const override;
    void enumerate_registers(std::vector<RegisterInfo>& out) const override;
    bool peek_register(const std::string& name, u64& out) const override;
    bool poke_register(const std::string& name, u64 value) override;
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    void set_irq_callback(std::function<void(u32, bool)> callback) { irq_callback_ = std::move(callback); }
    /// Registers a device that can be the target of a channel transfer.
    void add_target(u32 base, u32 size, Device* device);
    u64 transfers() const { return transfers_; }
    u64 bytes_copied() const { return bytes_copied_; }

    static constexpr u32 kChannelStride = 0x20;
    static constexpr u32 kGlobalOffset = 0x00;
    static constexpr u32 kChannelOffset = 0x100;

private:
    struct Channel {
        u32 source = 0;
        u32 dest = 0;
        u32 count = 0;      ///< in bytes
        u32 control = 0;
        u32 status = 0;
        bool active = false;
    };
    struct Target {
        u32 base = 0;
        u32 size = 0;
        Device* device = nullptr;
    };

    void start(u32 index);
    void step(u32 index);
    void complete(u32 index, bool ok);
    bool read_source(std::vector<u8>& out, u32 address, u32 length);
    bool write_dest(const std::vector<u8>& data, u32 address);
    u32 irq_status() const;
    void update_irq();

    Bus& bus_;
    std::vector<Channel> channels_;
    std::vector<Target> targets_;
    std::map<u32, std::string> names_;
    bool irq_ = false;
    u32 irq_mask_ = 0;
    u32 global_control_ = 0;
    u64 transfers_ = 0;
    u64 bytes_copied_ = 0;
    std::function<void(u32, bool)> irq_callback_;
};

// ---------------------------------------------------------------------------
// Display controller
// ---------------------------------------------------------------------------

class DisplayController : public Device {
public:
    DisplayController(std::string name, u32 base, u32 size, int width, int height);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;
    void tick(u64 cycles) override;

    const char* register_name(u32 address) const override;
    void enumerate_registers(std::vector<RegisterInfo>& out) const override;
    bool peek_register(const std::string& name, u64& out) const override;
    bool poke_register(const std::string& name, u64 value) override;
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    /// Currently selected buffer, or nullptr when the display is powered down.
    const u8* framebuffer(int& width, int& height, int& stride) const;
    u64 frame_counter() const { return frame_counter_; }
    /// Called after every completed frame (used by tests and the debugger).
    void set_frame_callback(std::function<void()> callback) { frame_callback_ = std::move(callback); }

    enum class Format : u32 { Rgb565 = 0, Rgba8888 = 1 };
    static size_t bytes_per_pixel(Format format) { return format == Format::Rgb565 ? 2u : 4u; }

private:
    struct Buffer {
        u32 address = 0;
        Format format = Format::Rgb565;
        int stride = 0;
        int width = 0;
        int height = 0;
        std::vector<u8> pixels;
    };

    void update_layout();
    Buffer* buffer_for(u32 address);
    const Buffer* buffer_for(u32 address) const;

    int panel_width_;
    int panel_height_;
    std::array<Buffer, 2> buffers_;
    u32 active_ = 0;
    u32 control_ = 0;
    u32 status_ = 0;
    u32 vsync_period_ = 16667;  ///< PERIPHCLK ticks for 60 Hz
    u32 vsync_left_ = 0;
    u64 frame_counter_ = 0;
    u64 frames_ = 0;
    u64 writes_ = 0;
    std::map<u32, std::string> names_;
    std::function<void()> frame_callback_;
};

// ---------------------------------------------------------------------------
// Syscon bridge
// ---------------------------------------------------------------------------

/// The ARM to Ernie (syscon) "SC" bridge. The register layout is an
/// ASSUMPTION: the C# reference model only reserves the 0xE2000000 window
/// ("ARM.SysconBridge"), and the real protocol register offsets were not
/// recovered from syscon.elf. This model implements the obvious shape: a door
/// bell register carrying the command id plus a small parameter block, a
/// status register with a busy/done handshake, and a response window.
class SysconBridge : public RegisterBlock {
public:
    SysconBridge(std::string name, u32 base, u32 size);

    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;
    void reset() override;

    void set_syscon(ErnieBlock* ernie) { ernie_ = ernie; }
    void set_irq_callback(std::function<void(u32, bool)> callback) { irq_callback_ = std::move(callback); }

    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

private:
    void post_command();

    ErnieBlock* ernie_ = nullptr;
    u32 last_command_ = 0;
    u64 commands_ = 0;
    std::function<void(u32, bool)> irq_callback_;
};

// ---------------------------------------------------------------------------
// Peripheral power / clock glue (0xE2030000..0xE209FFFF)
// ---------------------------------------------------------------------------

/// KBL brings each of the seven 64 KiB windows in its table at 0x4005C058 up with
/// one helper (0x4003BBCC): it writes +0x04, +0x10, +0x20, +0x30, +0x40, +0x50,
/// +0x60 and +0x64, then waits for a ready bit at +0x28 (0x4003BC98 tests bit 8,
/// 0x4003BCCC tests bit 9) before writing +0x70.  Nothing in the workspace
/// materials documents what +0x28 reports, so the model answers "powered and
/// clocked" and stores the configuration - the loader's own sequence (configure,
/// poll, enable) still runs unchanged.
class PeripheralPower : public RegisterBlock {
public:
    PeripheralPower(std::string name, u32 base, u32 size);

    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;

    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    static constexpr u32 kStatus = 0x28;
    static constexpr u32 kReadyBits = 0x0300;  ///< bit 8 ("powered") + bit 9 ("clocked")
    /// Command register: a write starts the peripheral, the read returns zero.
    static constexpr u32 kKick = 0x64;

private:
    u32 kicks_ = 0;
};

// ---------------------------------------------------------------------------
// Boot handshake window (0xE5888000)
// ---------------------------------------------------------------------------

/// KBL (0x400217FA..0x40021872) writes a command to +0x00 and +0x08 and then polls
/// +0x20 until it reads a per-command acknowledgement: 0x44 after a command of 2
/// and 0x11 after a command of 1, after which it programs +0x100/+0x108 with 0x302
/// and +0x180 with 1 and drops a 0xDEADCAFE marker at 0x80000000 (DRAM).
/// No module in the workspace references the window, so the model has to supply
/// the acknowledgements itself: it tracks the last command and answers with the
/// value that command's poll expects, which is what lets the loader reach DRAM.
class BootHandshake : public RegisterBlock {
public:
    BootHandshake(std::string name, u32 base, u32 size);

    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;
    void reset() override;

    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    static constexpr u32 kCommand0 = 0x00;
    static constexpr u32 kCommand1 = 0x08;
    static constexpr u32 kStatus = 0x20;
    static constexpr u32 kAckAfter2 = 0x44;
    static constexpr u32 kAckAfter1 = 0x11;

private:
    u32 last_command_ = 0;
    u64 commands_ = 0;
};

// ---------------------------------------------------------------------------
// Small helper used by several devices
// ---------------------------------------------------------------------------

/// Little endian byte extraction/insertion for sub-word register access.
inline u64 extract_register_bytes(u64 value, u32 offset, u32 base, unsigned size) {
    const u32 shift = ((offset - base) & 3u) * 8u;
    u64 out = 0;
    for (unsigned i = 0; i < size; ++i) {
        out |= ((value >> ((shift + i * 8u) & 63u)) & 0xFFull) << (8u * i);
    }
    return out;
}

inline u64 merge_register_bytes(u64 stored, u64 value, u32 offset, u32 base, unsigned size) {
    const u32 shift = ((offset - base) & 3u) * 8u;
    u64 mask = 0;
    for (unsigned i = 0; i < size; ++i) {
        mask |= 0xFFull << ((shift + i * 8u) & 63u);
    }
    return (stored & ~mask) | ((value << shift) & mask);
}

}  // namespace zlb::kermit

namespace zlb::kermit {

// ---------------------------------------------------------------------------
// PowerVR SGX543MP4+ register window (round 187, task part 2)
// ---------------------------------------------------------------------------

/// Base of the SGX register window. ASSUMPTION: no firmware module yields this
/// address directly - `libgpu_es4.elf` is a user library that reaches the GPU
/// through kernel services, and the kernel side was not recovered - so the
/// model puts it in the free part of the ARM peripheral space next to the other
/// model-only windows (DMA 0xE2060000, display 0xE2100000). The offsets below
/// are the model's own contract until a firmware-derived map replaces them;
/// they are named so that the whole map can be corrected in one place.
constexpr u32 kSgxBase = 0xE2400000;
constexpr u32 kSgxSize = 0x1000;

/// Register offsets of the model's SGX window.
constexpr u32 kSgxCoreId = 0x0000;        ///< read: identification (0x54354305)
constexpr u32 kSgxCoreRevision = 0x0004;  ///< read: revision
constexpr u32 kSgxCoreStatus = 0x0008;    ///< read: busy flags (model: 0)
constexpr u32 kSgxEventStatus = 0x0010;   ///< read: completed/queued events
constexpr u32 kSgxEventClear = 0x0014;    ///< write 1 to a bit to clear it
constexpr u32 kSgxEventEnable = 0x0018;   ///< bits that raise the IRQ line
constexpr u32 kSgxQueueBase = 0x0020;     ///< physical base of the command queue
constexpr u32 kSgxQueueSize = 0x0024;     ///< queue size in bytes
constexpr u32 kSgxQueueControl = 0x0028;  ///< bit 0 enable; write 1 = kick
constexpr u32 kSgxQueueWrite = 0x002C;    ///< producer offset (written by the driver)
constexpr u32 kSgxQueueRead = 0x0030;     ///< consumer offset (model)
constexpr u32 kSgxIrqStatus = 0x0040;     ///< read: pending interrupt bits
constexpr u32 kSgxIrqClear = 0x0044;      ///< write 1 to a bit to clear it

/// GPU MMU registers of the model. The SGX has its own page table walker; the
/// firmware side was not recovered, so the layout below is the model's contract
/// and is documented here in one place:
///   * MMU_DIR_BASE points at a table of 8 byte entries {u32 page, u32 flags},
///     indexed by a >> 12 (bit 0 of flags = valid),
///   * MMU_CONTROL bit 0 enables translation,
///   * writing 1 into MMU_INVALIDATE drops the model's cached translations,
///   * MMU_STATUS reports {faults, translations} as two halfwords.
constexpr u32 kSgxMmuDirBase = 0x0100;
constexpr u32 kSgxMmuControl = 0x0104;
constexpr u32 kSgxMmuInvalidate = 0x0108;
constexpr u32 kSgxMmuStatus = 0x010C;
constexpr u32 kSgxMmuEntryValid = 1u << 0;
constexpr u32 kSgxMmuFault = 0xFFFFFFFFu;

/// Event bits of the model (the SGX reports TA/3D completion separately).
constexpr u32 kSgxEventTa = 1u << 0;      ///< a kick was accepted
constexpr u32 kSgxEvent3d = 1u << 1;      ///< the queue was drained
constexpr u32 kSgxEventErr = 1u << 2;     ///< a malformed command was seen

/// Identity values the window reports. The SGX543MP4+ identification register is
/// a documented PowerVR constant in the 0x543543xx family; the exact low byte is
/// an ASSUMPTION of this model.
constexpr u32 kSgxCoreIdValue = 0x54354305;
constexpr u32 kSgxCoreRevisionValue = 0x00000100;

/// The modelled SGX block: a register window plus the command-queue handshake.
/// A kick (write 1 into QUEUE_CONTROL) is completed synchronously - the model
/// has no shader pipeline yet - and sets EVENT_TA | EVENT_3D, which the enable
/// mask turns into an interrupt request. Command buffers themselves are not
/// parsed here yet (that is the GXM step of docs/GPU.md).
class Sgx : public RegisterBlock {
public:
    Sgx(std::string name, u32 base, u32 size, Bus& bus);

    void reset() override;
    void tick(u64 cycles) override;

    void set_irq_callback(std::function<void(u32, bool)> callback) { irq_ = std::move(callback); }
    bool irq_line() const { return irq_line_; }

    u64 kicks() const { return kicks_; }
    /// Bytes the model consumed from the command queue (published by the driver).
    u64 queue_bytes() const { return queue_bytes_; }
    /// First command words of the last kick, as read from guest memory.
    const std::vector<u32>& last_words() const { return last_words_; }

    /// GPU MMU: translate a device virtual address through the modelled table.
    /// Returns kSgxMmuFault when translation is disabled, the entry is invalid
    /// or the walk fails; every attempt is counted.
    u32 translate(u32 va);
    u64 translations() const { return translations_; }
    u64 faults() const { return faults_; }
    u64 invalidations() const { return invalidations_; }
    u64 commands() const { return commands_; }
    u32 queue_base() const { return queue_base_; }
    u32 queue_size() const { return queue_size_; }
    u32 queue_write_offset() const { return queue_write_; }
    u32 queue_read_offset() const { return queue_read_; }

    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

protected:
    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;

private:
    void refresh_irq();
    /// Complete one kick: raise the completion events and advance the consumer.
    void complete_kick();

    Bus& bus_;
    std::function<void(u32, bool)> irq_;
    std::vector<u32> last_words_;   ///< first words of the last consumed kick
    u64 queue_bytes_ = 0;
    u32 mmu_dir_base_ = 0;
    u32 mmu_control_ = 0;
    u64 translations_ = 0;
    u64 faults_ = 0;
    u64 invalidations_ = 0;
    u32 events_ = 0;
    u32 event_enable_ = 0;
    u32 irq_status_ = 0;
    u32 queue_base_ = 0;
    u32 queue_size_ = 0;
    u32 queue_control_ = 0;
    u32 queue_write_ = 0;
    u32 queue_read_ = 0;
    u64 kicks_ = 0;
    u64 commands_ = 0;
    bool irq_line_ = false;
};

}  // namespace zlb::kermit
