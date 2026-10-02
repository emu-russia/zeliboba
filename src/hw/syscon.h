// zeliboba - Ernie (Renesas RL78 syscon) hardware block.
//
// Ernie is a separate microcontroller on the board. It owns the power/reset
// sequencing, the RTC, the fuel gauge, the SD/eMMC host and the "SC" command
// channel that the CMeP and the SoC use to talk to it.
//
// The block can run in two modes:
//
//   * firmware mode  - the real RL78 firmware (ernie-master/USS-1001.bin) executes
//                      and drives the peripherals through SFR accesses;
//   * functional mode- a C++ model of the SC command dispatcher answers the
//                      commands directly (used when no dump is fitted, and as a
//                      reference oracle while bringing the firmware up).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/state.h"
#include "common/types.h"

namespace zlb {

class EmmcCard;
class Cpu;

namespace ernie {
/// RL78 SFR window (the last 256 bytes of the 1 MiB space, also mirrored at the
/// top of the 32 bit space because of the way the core forms addresses).
constexpr u32 kSfrBase = 0x000FFF00;
constexpr u32 kSfrSize = 0x100;
constexpr u32 kSfrMirrorBase = 0x0FFFFF00;
constexpr u32 kFlashBase = 0x00000000;
constexpr u32 kFlashSize = 0x100000;  // 1 MiB code/data flash
constexpr u32 kResetVector1001 = 0xE000;
constexpr u32 kResetVector1002 = 0x0DC00;

/// SC register windows used by the CMeP first loader and the secure kernel.
constexpr u32 kScCmdWindow = 0xE0B00000;
constexpr u32 kScRespWindow = 0xE0BF0000;

/// 0xE3101000 - SC interface status. The CMeP second loader reads it while it
/// validates the chip configuration (0x40DB6: `lw $10,(0xE3101000)` and then
/// requires exactly 0x0001000F, otherwise it aborts with 0x800F0010 and reports
/// the 0x500 failure code to the ARM mailbox). The low nibble is the FIFO state,
/// bit 16 the "interface present" strap.
constexpr u32 kScIfStatus = 0xE3101000;
constexpr u32 kScIfStatusValue = 0x0001000F;

/// 0xE3100000 - SC interface type. The second loader caches it and treats it as a
/// signed interface generation: a negative value means "no SC interface" and the
/// eMMC open path (0x47EBA -> 0x48014/0x4804A) only accepts 48 ('0') or 50 ('2').
/// It is read in five places (0x404A0, 0x437BC, 0x47EC2, 0x483EE, 0x4D9EC) and
/// every one of them only compares it against zero or those two codes, so 48 is
/// the generation the 1.04 boot chain expects.
constexpr u32 kScIfType = 0xE3100000;
constexpr u32 kScIfTypeValue = 48;  // '0'

/// The SPI packet layer (hw/syscon/ernie_spi.cpp). Packets are
/// `[cmd_lo][cmd_hi][len][payload...][checksum]` for a request and
/// `[cmd_lo][cmd_hi][len][flags][payload...][checksum]` for a response, with
/// `checksum` the negation of the sum of the bytes before it.
std::vector<u8> make_spi_request(u16 command, const std::vector<u8>& payload);
std::vector<u8> make_spi_response(u16 command, u8 flags, const std::vector<u8>& payload);
/// Decode a request. Returns false (and fills `error`) when the packet is short
/// or the checksum does not match.
bool parse_spi_request(const std::vector<u8>& packet, u16& command, std::vector<u8>& payload,
                       std::string& error);
/// The response code the 1.04 boot chain accepts for a data reply.
constexpr u16 kSpiResponseOk = 0x0004;

/// Name of an SC command from the USS-1001 command table ("get_status",
/// "nvs_read", ...), or an empty string when the number is not in it.  The table
/// lives inside the syscon model; this is the public way for a tool (the
/// debugger's `sc` command) to name what the guest asked for without including
/// the model's private header.
const char* sc_command_name(u32 number);
}  // namespace ernie

/// The syscon side of the board.
class ErnieBlock {
public:
    ErnieBlock(Bus& bus, EmmcCard* card);
    ~ErnieBlock();

    ErnieBlock(const ErnieBlock&) = delete;
    ErnieBlock& operator=(const ErnieBlock&) = delete;

    /// Map flash, install SFRs, host interface and the eMMC host.
    void install();

    void reset();

    /// Fit the RL78 firmware dump. When `run_firmware` is false the functional
    /// SC model answers commands instead of executing the RL78 core.
    bool load_firmware(const std::vector<u8>& dump, bool run_firmware = true);
    bool firmware_loaded() const;
    bool running_firmware() const { return run_firmware_; }
    void set_running_firmware(bool value) { run_firmware_ = value; }
    u32 reset_vector() const { return reset_vector_; }

    Cpu* cpu() { return cpu_.get(); }

    // ------------------------------------------------------------------
    // SC command channel (used by the CMeP first loader and the SoC)
    // ------------------------------------------------------------------

    /// A command descriptor has been written to the SC window. In firmware mode
    /// this just wakes the RL78; in functional mode it is decoded here.
    void command_posted(const u8* descriptor, size_t length);

    /// True when a reply is waiting to be read back by the caller.
    bool response_ready() const { return response_ready_; }
    const std::vector<u8>& response() const { return response_; }
    void clear_response();

    bool busy() const { return busy_; }
    u64 commands_served() const { return commands_served_; }

    /// The last few SC commands that were dispatched, oldest first, as
    /// {command number, reply byte count}.  The model has always collected this
    /// ring (and serialises it) but nothing could read it back, so "what is the
    /// guest actually asking the syscon while the boot sits idle" was
    /// unanswerable; the debugger's `sc` command now prints it with the
    /// USS-1001 command names.  Defined in hw/syscon/ernie.cpp (the ring lives in
    /// the private implementation).
    const std::vector<std::pair<u32, u32>>& recent_commands() const;

    // ------------------------------------------------------------------
    // Peripherals
    // ------------------------------------------------------------------

    /// Milliseconds since power-on, as the RTC/fuel gauge would see it.
    u64 milliseconds() const { return milliseconds_; }
    void advance_milliseconds(u64 delta);
    /// Advance the core-cycle driven devices (the SFR clock, RTC and interval
    /// timer).  The RL78 firmware's start-up waits for the X1 oscillator to
    /// stabilise (`cmp !0xFFFA2, #0xC0` at 0x3005A), which only ever completes if
    /// somebody hands the SFR the core's cycle count - before round 93 nothing
    /// did, so the firmware spun there forever and never left its clock init.
    void tick(u64 cycles);
    void set_rtc_time(u64 unix_seconds) { rtc_seconds_ = unix_seconds; }
    u64 rtc_time() const { return rtc_seconds_; }

    /// Fuel gauge (Abby / bq27520) state used by the syscon firmware.
    int battery_percent() const { return battery_percent_; }
    void set_battery_percent(int value) { battery_percent_ = value; }
    /// 0 = on battery, 1 = charging, 2 = charged.
    int charger_state() const { return charger_state_; }
    void set_charger_state(int value) { charger_state_ = value; }

    /// Power button / home button lines read by the syscon.
    void set_power_button(bool pressed);
    void set_ps_button(bool pressed);
    void set_volume_up(bool pressed);
    void set_volume_down(bool pressed);

    /// Wake the SoC (the "ARM can start" handshake the first loader performs).
    bool soc_released() const { return soc_released_; }
    void release_soc();

    std::string summary() const;
    void describe(std::vector<std::string>& lines) const;
    std::vector<Device*> devices() const;

    /// Functional SC command dispatch (also used by tests).
    std::vector<u8> dispatch_command(u32 command, const std::vector<u8>& payload);

    /// Read the modeled NVS for board handoff inputs without issuing a guest
    /// SC command or changing mailbox/protocol state.
    bool read_nvs(u16 offset, size_t length, std::vector<u8>& out) const;

    // ------------------------------------------------------------------
    // Save states
    // ------------------------------------------------------------------

    /// ErnieBlock is not a Device: the bus serialises the devices it registered
    /// (flash, SFR, SC window, message windows, gate and strap), so this method
    /// adds the parts none of them own: the block's own latches, the SC channel
    /// (the register file is not bus registered), the board/power model, the NVS
    /// and scratch pad stores, the eMMC host statistics and the RL78 core.
    void save_state(StateWriter& writer) const;
    void load_state(StateReader& reader);

    // ------------------------------------------------------------------
    // SPI0 link (0xE0A00000) - the boot chain's own path to the syscon
    // ------------------------------------------------------------------

    /// Clock one SPI packet through the link and return the answer. The CMeP
    /// second loader (transfer at 0x436E4) and the ARM syscon.elf driver both
    /// use this path; see hw/syscon/ernie_spi.cpp for the packet format.
    std::vector<u8> spi_transfer(const std::vector<u8>& request);

    u64 spi_transfers() const { return spi_transfers_; }
    u64 spi_bad_packets() const { return spi_bad_packets_; }
    u32 last_spi_command() const { return last_spi_command_; }
    const std::vector<u8>& last_spi_request() const { return last_spi_request_; }
    const std::vector<u8>& last_spi_response() const { return last_spi_response_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    Cpu* cpu_raw_ = nullptr;
    bool run_firmware_ = true;
    bool firmware_loaded_ = false;
    u32 reset_vector_ = ernie::kResetVector1001;

    bool response_ready_ = false;
    bool busy_ = false;
    std::vector<u8> response_;
    u64 commands_served_ = 0;

    // SPI0 link statistics (see spi_transfer).
    u64 spi_transfers_ = 0;
    u64 spi_bad_packets_ = 0;
    u32 last_spi_command_ = 0;
    std::vector<u8> last_spi_request_;
    std::vector<u8> last_spi_response_;

    u64 milliseconds_ = 0;
    u64 rtc_seconds_ = 0x4E24E1C0;  // 2011-07-29, a plausible manufacturing date
    int battery_percent_ = 87;
    int charger_state_ = 0;
    bool soc_released_ = false;

    // std::unique_ptr<Cpu> cpu_ lives inside Impl; mirrored here for accessors.
    std::unique_ptr<Cpu> cpu_;
};

}  // namespace zlb
