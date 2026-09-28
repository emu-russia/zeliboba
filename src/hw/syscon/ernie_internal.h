// zeliboba - Ernie internal interfaces.
//
// This header is private to src/hw/syscon/: it carries the SC register map, the
// SC bridge device and the eMMC host so that ernie.cpp, ernie_sc.cpp,
// ernie_emmc.cpp and ernie_power.cpp can share them without widening the public
// surface (src/hw/syscon.h).
//
// ---------------------------------------------------------------------------
// The SC protocol, reconstructed
// ---------------------------------------------------------------------------
//
// The CMeP first loader talks to Ernie through a fixed register file
// (all citations are offsets into dumps/bootrom_analysis/
// pch-5c-cold_first_loader.annotated.asm):
//
//   sc_xfer (0x5CF88) builds a 28-byte descriptor at 0x5EE20 and stores the
//   command word at 0x5EDA0:
//       +0x00 u8  mode        0x30 / 0x80 / 0x101      (loc_5CFB0..loc_5CFE4)
//       +0x04 u32 timeout     0x0B71B000 << 2          (0x5CFC2)
//       +0x08 u32 timeout2    0x0B71B000 << 1          (0x5CFD4)
//       +0x0E u8  channel     = command & 0xFF         (0x5CFBE: `sb $tp,14($7)`)
//       +0x18 u32 window      0xE0B00000 | (channel<<16) (0x5CF9C..0x5CFAC)
//   sc_read (0x5D02E) then drives the transfer:
//       - the command word goes to the window selected by the channel
//         (`sw $12,($7)` at loc_5CFF4)
//       - when mode bit 1 is set it programs the window's control block and
//         spins on `lw $12,36($3)` waiting for bit 1 (0xE3100024 bit 1)
//       - the 8 reply bytes come from offsets 48..55 of the SC block
//         (0x5C302..0x5C338), i.e. 0xE3100030..0xE3100037
//       - it then waits for control+0x24 bit 1 again and for the two 16-bit
//         words at +0x30/+0x32 to become zero (loc_5D06C..loc_5D08E)
//
//   boot_path_bit5 (0x5C20A) and boot_path_bit6 (0x5C388) show the SoC-side
//   handshake around it:
//       0xE3100124 |= 1                       (0x5C238) "syscon may service"
//       0xE3101190  = 1                       (0x5C246) request a reply
//       0xE31020A0  = 1                       (0x5C248) post the command
//       wait 0xE31020A0 != 0 and 0xE31010A0 != 0, then clear both and poll
//       0xE3101190 back to 0                  (0x5C24A..0x5C290)
//   The SoC side of the same block is 0x30000118 (bit 1 gate) and 0x30000208
//   (0x5C294..0x5C2B4), and the JIG/state strap is 0xE0064060 (0x5C2C0).
//
//   On the Ernie side the command table lives at 0x26BE (USS-1001, 70 entries,
//   {u16 number, u16 flags, u32 handler}, flags bit 0 = "requires payload"), and
//   every handler answers into a 4+32 byte response record at 0x0DD98:
//       +0  u8   result            (0 = ok, 0xFE = busy, else error)
//       +1  u8   byte 1 of the 6-byte panel/status block
//       +2  u8   length of the payload that follows at 0x0DD9E
//   which is exactly the 8 bytes sc_read reads back out of 0xE3100030.
//   Handler 0x0100 (0x36132) fills +0/+1 and length 0x06; handler 0x0103
//   (0x361C5) additionally calls 0x4C60 (a "result == 0 || result == 0xFE"
//   predicate) and reports 0x80 when it fails.  sc_xfer is entered with
//   $1 = 0x101 from the main path, which is command 0x0101 -> handler 0x36118.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "bus/device.h"
#include "hw/emmc.h"
#include "hw/syscon.h"
#include "hw/syscon/ernie_sfr.h"

namespace zlb {

class ErnieBlock;
class Cpu;

namespace ernie {

// ---------------------------------------------------------------------------
// SC register map
// ---------------------------------------------------------------------------

/// SoC side (CMeP + Kermit), window 0xE3100000.  The window is 64 KiB so that
/// every register the first loader touches (0x0194, 0x0124, 0x10A0, 0x1190,
/// 0x20A0, 0x3040 ...) is decoded by one device.
constexpr u32 kScWindowBase = 0xE3100000;
/// 128 KiB: KBL (running on the ARM) reaches beyond the first 64 KiB - it writes
/// 0xE3110FC0 = 0x0007FFFF at 0x4002158A while it brings its storage path up.
constexpr u32 kScWindowSize = 0x20000;
constexpr u32 kScStat24 = 0xE3100024;   ///< bit1 = response ready, bit2 = error
/// 0xE3101000 (see hw/syscon.h): the interface status the CMeP validates.
constexpr u32 kScStat1000 = kScIfStatus;
constexpr u32 kScStat1000Value = kScIfStatusValue;
constexpr u32 kScStat36 = 0xE3100036;
constexpr u32 kScReq122 = 0xE3100122;
constexpr u32 kScIrq124 = 0xE3100124;   ///< bit0 = bridge enabled (0x5C238)
constexpr u32 kScReply1190 = 0xE3101190;
constexpr u32 kScCmd10A0 = 0xE31010A0;
constexpr u32 kScAck10A4 = 0xE31010A4;
constexpr u32 kScResp20A0 = 0xE31020A0;
constexpr u32 kScResp20A4 = 0xE31020A4;
constexpr u32 kScReg1100 = 0xE3101100;
constexpr u32 kScReg2100 = 0xE3102100;
constexpr u32 kScReg3040 = 0xE3103040;
constexpr u32 kScReg3050 = 0xE3103050;
constexpr u32 kScReg0194 = 0xE3100194;
constexpr u32 kScEventsC0 = 0xE31000C0;  ///< SC event register (write-1-to-clear)
/// KBL writes 1 here and polls until it reads back 0 (0x4003C014..0x4003C026:
/// `movs/movt r3,#0xE310005C`, `str r1=1`, `ldr`/`bne` loop): a service request
/// latch the syscon clears once the request has been taken.
constexpr u32 kScReq5C = 0xE310005C;
/// The same latch one word further on: KBL's next bring-up step writes 1 to
/// 0xE3100060 and polls it the same way (0x4003C04C..0x4003C05E).
constexpr u32 kScReq60 = 0xE3100060;

/// Ernie side: the byte lanes through which the SC engine passes a command.
constexpr u32 kScEngineCmd = 0xE3100030;  ///< sc_read's "control+0x30" data port
constexpr u32 kScEngineAux = 0xE3100032;  ///< sc_read's "control+0x32" data port

/// The two command windows.  `kScRespWindow + (channel << 16)` is the reply.
constexpr u32 kScDescriptorMax = 0x40;

/// Command numbers taken verbatim from the Ernie command table (USS-1001,
/// 0x26BE .. 0x26BE + 70*8).  Only the ones the machine layer actually needs to
/// name are listed; every number in the table is accepted by the dispatcher.
enum class ScCommand : u32 {
    GetStatus = 0x0000,
    GetPanel = 0x0100,
    GetModel = 0x0103,
    GetFirmwareVersion = 0x0111,
    GetFirmwareVersionAlt = 0x0012,
    PowerOff = 0x0013,
    GetRtc = 0x0101,
    SetRtc = 0x0102,
    GetBattery = 0x0800,
    GetCharger = 0x0820,
    SetChargeCurrent = 0x0806,
    ResetDevice = 0x0900,
    Suspend = 0x0901,
    Resume = 0x0902,
    StorageRead = 0x1100,
    StorageWrite = 0x1101,
    StorageInfo = 0x1000,
    StorageErase = 0x1180,
    BootPartitionSelect = 0x1184,
    BootPartitionRead = 0x1185,
};

/// Per-command metadata: number, name and whether the handler needs a payload
/// (flags bit 0 of the command table entry).
struct ScCommandInfo {
    u16 number = 0;
    u16 flags = 0;
    u32 handler = 0;
    const char* name = "";
};

/// The USS-1001 command table (0x26BE, 70 entries).  Provided so the debugger
/// and the functional dispatcher can both name commands.
const std::vector<ScCommandInfo>& sc_command_table();
const ScCommandInfo* sc_command_info(u32 number);

/// Flags the response header can carry (the byte at 0xE3100032 / 0x0DD9A).
constexpr u8 kScResultOk = 0x00;
constexpr u8 kScResultBusy = 0xFE;

// ---------------------------------------------------------------------------
// Ernie NVS and scratch pad
// ---------------------------------------------------------------------------
//
// Two small stores the console reads over the SC channel.  Both are addressed
// as `u16 offset` + `u8 length`; the request and the reply share the 4 + 32 byte
// response record, so a transfer is capped at 32 bytes.
//
// Evidence (all from the wiki unless marked):
//   * command 0x1082 is the NVS read -- the 1.04 second loader sends
//     `80 04 08` (we see the same bytes in the SPI trace) and the decompiled
//     3.60 loader calls `syscon_read_cmd_0x1082_ptr_0x480_into_gbuf`;
//   * command 0x1083 carries the same header followed by data (NVS write; the
//     wiki does not name it, the payload shape and the `flags` bit in the
//     USS-1001 command table are the evidence);
//   * command 0x0090/0x0091 are the scratch pad read/write.  The loader sends
//     `E0 00 20`, i.e. offset 0xE0 length 0x20, which is exactly the
//     `SceDIPSW dipsw` field of the documented `SceSysconScratchPad`.
//
// NVS layout the boot chain consumes (wiki "Ernie"):
//   0x400..0x47F  Qaf token
//   0x480         Qaf token flag        1 = token not set (area is 0xFF)
//   0x481         extra UART flag       0x01 = extra UART only with a JIG dongle
//   0x483         safe mode flags       0xFF = not safe mode
//   0x486         MCEmu flag
//   0x4A0         update mode           0xFF = not update mode
//   0x4E0..0x4FF  KibanID (ASCII serial)
// An unprovisioned console reads as 0xFF everywhere, which is what `reset()`
// fills.  No dump in the workspace carries a real NVS, so a console built by
// the model is "not provisioned" rather than impersonating one.

/// The Ernie non-volatile store (an area of the MCU data flash).
class NvsStore {
public:
    /// Visible from Kermit on 3.60 and later (the chip has more; the rest is
    /// internal).  Old revisions have 0xC20, new ones 0xBA0.
    static constexpr size_t kSize = 0xB60;

    void reset() { bytes_.assign(kSize, 0xFF); }

    size_t size() const { return bytes_.size(); }
    const std::vector<u8>& bytes() const { return bytes_; }

    /// Read `length` bytes at `offset`.  A request past the end of the store is
    /// answered with the bytes that exist (the firmware never sees one); an
    /// offset outside the store is an error.
    bool read(u16 offset, size_t length, std::vector<u8>& out) const {
        if (offset >= bytes_.size()) return false;
        const size_t available = std::min<size_t>(length, bytes_.size() - offset);
        out.assign(bytes_.begin() + offset, bytes_.begin() + offset + available);
        return true;
    }

    bool write(u16 offset, const u8* data, size_t length) {
        if (offset >= bytes_.size()) return false;
        const size_t count = std::min<size_t>(length, bytes_.size() - offset);
        std::copy(data, data + count, bytes_.begin() + offset);
        return true;
    }

private:
    std::vector<u8> bytes_ = std::vector<u8>(kSize, 0xFF);
};

/// The 0x100 byte scratch pad Ernie keeps across resets.
class ScratchPad {
public:
    static constexpr size_t kSize = 0x100;
    static constexpr u16 kResumeContextOffset = 0x000C;  ///< PA of the resume buffer
    static constexpr u16 kDipSwitchOffset = 0x00E0;      ///< SceDIPSW, 0x20 bytes
    static constexpr size_t kDipSwitchSize = 0x20;

    /// The DIP switch block a retail console reports.  The wiki says the CP part
    /// ("Set on DevKit having a CP.  Hence not set on Retail nor TestKit") is
    /// unset, and lists the release mode values for the rest; the model writes
    /// the same 0x20 bytes into `SceKblParam +0x40`, which is where the second
    /// loader puts what it read here [model assumption].
    static const std::array<u8, kDipSwitchSize>& retail_dip_switches();

    ScratchPad() { reset(); }

    void reset();

    size_t size() const { return bytes_.size(); }
    const std::array<u8, kSize>& bytes() const { return bytes_; }

    bool read(u16 offset, size_t length, std::vector<u8>& out) const {
        if (offset >= kSize) return false;
        const size_t available = std::min<size_t>(length, kSize - offset);
        out.assign(bytes_.begin() + offset, bytes_.begin() + offset + available);
        return true;
    }

    bool write(u16 offset, const u8* data, size_t length) {
        if (offset >= kSize) return false;
        const size_t count = std::min<size_t>(length, kSize - offset);
        std::copy(data, data + count, bytes_.begin() + offset);
        return true;
    }

private:
    std::array<u8, kSize> bytes_{};
};

// ---------------------------------------------------------------------------
// SC registers
// ---------------------------------------------------------------------------

/// The shared SC register file.  Both the SoC side (0xE3100xxx / 0xE3101xxx /
/// 0xE3102xxx) and the CMeP's program-and-poll pattern live here.
struct ScRegs {
    u32 cmd10A0 = 0;
    u32 ack10A4 = 0;
    u32 resp20A0 = 0;
    u32 resp20A4 = 0;
    u32 reg1190 = 0;
    u32 stat24 = 0;
    u32 stat36 = 0;
    u32 req122 = 0;
    u32 irq124 = 0;
    u32 reg1100 = 0;
    u32 reg2100 = 0;
    u32 reg3040 = 0;
    u32 reg3050 = 0;
    u32 reg0194 = 0;
    u32 engine30 = 0;
    u32 engine32 = 0;

    u32 command = 0;             ///< last command word accepted
    u32 error = 0;               ///< last error code (0 when the reply is good)
    u8 status_byte = 0;          ///< byte 1 of the response record
    u8 payload_length = 0;       ///< byte 2 of the response record

    /// The reply the SC engine hands back through 0xE3100030/0xE3100032.
    std::vector<u8> response;
    size_t response_byte = 0;

    u64 commands = 0;
    u64 request_polls = 0;
    u64 ack_polls = 0;

    void reset();
    std::string summary() const;
};

// ---------------------------------------------------------------------------
// SC channel - state machine
// ---------------------------------------------------------------------------

/// The SC command channel: turns register writes from the SoC side and window
/// writes from the CMeP side into `ErnieBlock` commands, and publishes the
/// reply back into both.
class ScChannel {
public:
    explicit ScChannel(ErnieBlock& owner);

    void reset();

    ScRegs& regs() { return regs_; }
    const ScRegs& regs() const { return regs_; }

    /// Decode a descriptor the CMeP left in a command window and run it.
    void post_descriptor(const u8* descriptor, size_t length, u32 window_base);

    /// Publish a reply into the response window and the status registers.
    void publish_reply(u32 command, const std::vector<u8>& reply);

    /// The SoC side wrote a command word to a window (register-pair path).
    void on_command_word_written(u32 value);

    /// The SoC cleared the reply request; drop the response.
    void on_reply_cleared();

    /// Called by ScWindowDevice when the reply data port is read.
    u64 read_data_port();
    void set_reply_request(u32 value);

    bool response_ready() const { return (regs_.stat24 & 0x02u) != 0; }
    bool busy() const { return (regs_.stat24 & 0x01u) != 0; }
    u64 commands() const { return regs_.commands; }

private:
    ErnieBlock& owner_;
    ScRegs regs_;
};

/// The SC window as a Device, so the CMeP boot ROM's polling loops terminate
/// and the debugger can see every SC register.
///
/// The device does not own the register file: `ScChannel` does, and `install()`
/// binds the device to it with `set_channel()`.  Keeping a single `ScRegs` is
/// deliberate - two copies would let the register view and the protocol state
/// machine drift apart.
class ScWindowDevice : public Device {
public:
    ScWindowDevice();

    /// Bind the channel that owns the register file and the protocol state
    /// machine.  Called by ErnieBlock::install().
    void set_channel(ScChannel* channel) { channel_ = channel; }

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;
    const char* register_name(u32 address) const override;
    void enumerate_registers(std::vector<RegisterInfo>& out) const override;
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

private:
    ScRegs& regs();
    const ScRegs& regs() const;

    /// Fallback state used only if the device is never bound to a channel (the
    /// debugger can construct one standalone).
    ScRegs fallback_;
    ScChannel* channel_ = nullptr;

    /// Storage for SC registers that have no modelled behaviour. The CMeP uses
    /// several of them as "write 1 and poll it back" strobes - 0xE3102120 in the
    /// second loader (0x487E8: `sw $9,($3)` / `erepeat` / `lw $9,($3)` /
    /// `bnez $9,0x487F6`), and 0xE3101120 just after - so an unknown register
    /// reads back what was written instead of zero.
    std::map<u32, u32> scratch_;

    /// 0xE31000C0: the SC event register.  The kernel boot loader waits for bit 3
    /// (0x8) and bit 4 (0x10) and acknowledges them write-1-to-clear:
    ///   4003C02E  ldr r2,[0xE31000C0] / 4003C036 tst r2,#8 / 4003C03A beq  (wait set)
    ///   4003C040  str #8,[0xE31000C0]                                  (ack)
    ///   4003C042  ldr r3,[r2] / 4003C044 ands r0,r3,#8 / 4003C048 bne   (wait clear)
    ///   4003C066  ldr r2,[0xE31000C0] / 4003C06E tst r2,#0x10 / beq    (wait bit 4)
    /// The syscon raises the next event once the guest has taken the previous one,
    /// so a read re-arms the bits after handing them out.
    u32 events_c0_ = 0x18u;
};

/// 0x30000118 / 0x30000208: the SoC boot gate the CMeP programs at 0x5C294.
class SocGateDevice : public Device {
public:
    SocGateDevice();

    /// True once the CMeP has written 0x8001 to 0x30000208 while bit 1 of
    /// 0x30000118 was set (0x5C298..0x5C2B4).
    bool released_soc() const { return released_; }
    void set_released(bool value) { released_ = value; }

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;
    const char* register_name(u32 address) const override;
    std::string summary() const override;

    static constexpr u32 kBase = 0x30000100;
    static constexpr u32 kSize = 0x200;
    static constexpr u32 kBootState = 0x30000118;
    static constexpr u32 kRelease = 0x30000208;

private:
    u32 boot_state_ = 0x00000002;  ///< bit 1 set: "CMeP is up, SoC may be released"
    u32 release_ = 0;
    bool released_ = false;
};

/// 0xE0064060: the JIG/boot state strap the SC path reads at 0x5C2C0.
class ScStrapDevice : public Device {
public:
    ScStrapDevice();

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;
    const char* register_name(u32 address) const override;
    std::string summary() const override;

    static constexpr u32 kBase = 0xE0064060;
    static constexpr u32 kSize = 4;

    void set_value(u32 value) { value_ = value; }
    u32 value() const { return value_; }

private:
    u32 value_ = 0x00010000;  ///< bit16 clear: normal boot, not JIG
};

// ---------------------------------------------------------------------------
// 0xE0B00000 / 0xE0BF0000 - command and reply windows
// ---------------------------------------------------------------------------

/// Holds the 28-byte descriptor the CMeP sc_xfer builds and the reply that the
/// syscon writes back.  `ErnieBlock::command_posted` is called with the bytes
/// written here.
class ScMessageWindow : public Device {
public:
    ScMessageWindow(u32 base, u32 size, bool response);

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;
    void reset() override;
    const char* register_name(u32 address) const override;
    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    const std::vector<u8>& bytes() const { return bytes_; }
    void load(const std::vector<u8>& data);
    void store(u32 offset, const u8* data, size_t length);
    bool dirty() const { return dirty_; }
    void clear_dirty() { dirty_ = false; }

private:
    std::vector<u8> bytes_;
    bool response_ = false;
    bool dirty_ = false;
};

// ---------------------------------------------------------------------------
// eMMC host
// ---------------------------------------------------------------------------

/// The syscon's SD/eMMC host.  The RL78 firmware owns a real controller that is
/// not memory mapped into the SoC address space at all - the SoC only ever sees
/// the block transfers it asks for over SC.  This class is that host: it drives
/// the attached `EmmcCard` and keeps the transfer statistics the debugger and
/// the SC replies need.
class EmmcHost {
public:
    EmmcHost(EmmcCard* card);

    void reset();

    bool attached() const { return card_ != nullptr && card_->attached(); }

    /// 4-bit / 1-bit bus width, as reported to the SC "card info" command.
    void set_bus_width(int bits) { bus_width_ = bits; }
    int bus_width() const { return bus_width_; }
    void set_clock_hz(u32 hz) { clock_hz_ = hz; }
    u32 clock_hz() const { return clock_hz_; }

    /// Card identification, cached at init.
    bool init_card();
    u32 relative_address() const { return rca_; }
    EmmcPartition partition() const { return partition_; }
    bool select_partition(EmmcPartition partition);
    bool select_partition(u32 index);

    bool read_blocks(u64 lba, u32 count, std::vector<u8>& out);
    bool write_blocks(u64 lba, u32 count, const std::vector<u8>& data);
    bool erase_blocks(u64 lba, u32 count);

    u64 block_reads() const { return block_reads_; }
    u64 block_writes() const { return block_writes_; }
    u64 blocks_read() const { return blocks_read_; }
    u64 blocks_written() const { return blocks_written_; }
    u64 errors() const { return errors_; }
    u64 last_lba() const { return last_lba_; }
    u32 last_count() const { return last_count_; }
    u32 card_status() const { return card_status_; }

    std::string summary() const;
    void describe(std::vector<std::string>& lines) const;

private:
    EmmcCard* card_ = nullptr;
    u32 rca_ = 1;
    EmmcPartition partition_ = EmmcPartition::User;
    int bus_width_ = 4;
    u32 clock_hz_ = 40000000;
    u32 card_status_ = 0x00000100;  ///< R1: ready for data, not busy
    u64 block_reads_ = 0;
    u64 block_writes_ = 0;
    u64 blocks_read_ = 0;
    u64 blocks_written_ = 0;
    u64 errors_ = 0;
    u64 last_lba_ = 0;
    u32 last_count_ = 0;
};

// ---------------------------------------------------------------------------
// helpers shared by the SC and eMMC translation units
// ---------------------------------------------------------------------------

/// Little endian helpers for the SC descriptor/response marshalling.
inline u32 load_le32(const u8* p) {
    return static_cast<u32>(p[0]) | (static_cast<u32>(p[1]) << 8) | (static_cast<u32>(p[2]) << 16) |
           (static_cast<u32>(p[3]) << 24);
}
inline void store_le32(u8* p, u32 value) {
    p[0] = static_cast<u8>(value);
    p[1] = static_cast<u8>(value >> 8);
    p[2] = static_cast<u8>(value >> 16);
    p[3] = static_cast<u8>(value >> 24);
}
inline u32 load_le32(const std::vector<u8>& v, size_t offset) {
    if (offset + 4 > v.size()) return 0;
    return load_le32(v.data() + offset);
}
inline void store_le32(std::vector<u8>& v, size_t offset, u32 value) {
    if (offset + 4 > v.size()) return;
    store_le32(v.data() + offset, value);
}

}  // namespace ernie
}  // namespace zlb

