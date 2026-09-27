// zeliboba - Kermit: the ARM Cortex-A9 MPCore side of the SoC.
//
// This block models the peripherals the kernel boot loader and the kernel need
// before any higher level device is interesting: the interrupt controller, the
// timers, the debug UART, the SD/eMMC interface, DMA and the syscon bridge.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/types.h"

namespace zlb {

class EmmcCard;
class ErnieBlock;

namespace kermit {
/// Physical map used by the boot loaders (see docs/HARDWARE.md).
constexpr u32 kBootRomBase = 0x00000000;
constexpr u32 kBootRomSize = 0x10000;
constexpr u32 kSramBase = 0x1F000000;
constexpr u32 kSramSize = 0x40000;
constexpr u32 kDramBase = 0x80000000;
constexpr u32 kDramSize = 0x04000000;  // 64 MiB window used by the boot chain
constexpr u32 kScuBase = 0x40000000;   ///< private/secure window; the kernel boot loader
                                        ///< ELF is linked at 0x40020000 and runs with the MMU off
constexpr u32 kPeripheralWindowSize = 0x00010000;  ///< MPCore peripheral block (SCU/GIC/timers)
constexpr u32 kScuSize = 0x08000000;    ///< 128 MiB physical DRAM window (the KBL
                                        ///< touches 0x44000000, past the first 64 MiB)

/// Shared peripheral interrupt numbers (GIC SPI ids).
enum class Irq : u32 {
    Uart0 = 32 + 12,
    Timer0 = 32 + 20,
    Timer1 = 32 + 21,
    Emmc = 32 + 26,
    Dma0 = 32 + 30,
    Syscon = 32 + 40,
};
}  // namespace kermit

/// The ARM side of the board.
class KermitBlock {
public:
    KermitBlock(Bus& bus, EmmcCard* card);
    ~KermitBlock();

    KermitBlock(const KermitBlock&) = delete;
    KermitBlock& operator=(const KermitBlock&) = delete;

    void install();
    void reset();

    /// Advance time based devices (timers, DMA) by `cycles` CPU cycles.
    void tick(u64 cycles);

    /// Connect the syscon, so the ARM can post SC commands to Ernie.
    void set_syscon(ErnieBlock* ernie) { ernie_ = ernie; }

    /// Attach the syscon to the SPI0 master and mirror the three SPI blocks into
    /// `other` (the CMeP's bus - its second loader is what drives the link).
    /// Defined in hw/soc/kermit.cpp; see hw/soc/soc_internal.h for the details.
    void attach_syscon_spi(Bus& other, ErnieBlock* ernie);

    // ------------------------------------------------------------------
    // Interrupts
    // ------------------------------------------------------------------

    /// Raise/deassert a shared peripheral interrupt (see kermit::Irq).
    void raise_irq(u32 id, bool level);
    /// Pulse an interrupt (raise then lower after `cycles`).
    void pulse_irq(u32 id, u64 cycles = 64);
    u32 pending_irq_count() const;
    std::string gic_summary() const;

    // ------------------------------------------------------------------
    // Debug console
    // ------------------------------------------------------------------

    /// Output captured from the emulated UART (the kernel prints here).
    std::string take_uart_output();
    bool uart_has_output() const;
    void uart_write(const std::string& text);

    // ------------------------------------------------------------------
    // Storage
    // ------------------------------------------------------------------

    /// True when the SD/eMMC interface has completed a transfer.
    u64 emmc_transfers() const;
    /// Last block address / count the SD interface worked on (for the debugger).
    u64 last_emmc_lba() const;
    u32 last_emmc_count() const;

    // ------------------------------------------------------------------
    // Display (for the SDL3 frontend)
    // ------------------------------------------------------------------

    /// Currently selected framebuffer, or nullptr when the display is off.
    const u8* framebuffer(int& width, int& height, int& stride) const;
    /// Block until the display has been written, used for the UI refresh timer.
    u64 frame_counter() const;

    void describe(std::vector<std::string>& lines) const;
    std::string summary() const;
    std::vector<Device*> devices() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    ErnieBlock* ernie_ = nullptr;
};

}  // namespace zlb
