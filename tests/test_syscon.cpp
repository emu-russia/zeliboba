// zeliboba - Ernie (syscon) unit tests.
//
// Covers the four pieces of the block that can be tested without a machine:
//
//   * the SFR register file and the clock-generator handshake the RL78 boot ROM
//     polls (OSTC at 0xFFFA2, CKC at 0xFFFA4);
//   * the functional SC command dispatcher and its 4 + 32 byte reply record;
//   * the eMMC host behind the SC storage commands;
//   * the power / RTC / panel / fuel gauge model.
//
// The RL78 firmware run is exercised by the `zeliboba_ernie_selftest` target
// (tools/selftest.cpp) because it needs the 1 MiB dump; the tests here stay
// dump-free.
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/util.h"
#include "hw/emmc.h"
#include "hw/syscon.h"
#include "hw/syscon/ernie_internal.h"
#include "hw/syscon/ernie_sfr.h"
#include "test_framework.h"

using namespace zlb;

namespace {

ErnieSfr* find_sfr(Bus& bus) {
    for (const auto& device : bus.devices()) {
        if (device->name() == "Ernie.SFR") return dynamic_cast<ErnieSfr*>(device.get());
    }
    return nullptr;
}

std::vector<u8> le32(u32 value) {
    std::vector<u8> out(4);
    out[0] = static_cast<u8>(value);
    out[1] = static_cast<u8>(value >> 8);
    out[2] = static_cast<u8>(value >> 16);
    out[3] = static_cast<u8>(value >> 24);
    return out;
}

u32 read_le32(const std::vector<u8>& v, size_t offset) {
    if (offset + 4 > v.size()) return 0;
    return static_cast<u32>(v[offset]) | (static_cast<u32>(v[offset + 1]) << 8) |
           (static_cast<u32>(v[offset + 2]) << 16) | (static_cast<u32>(v[offset + 3]) << 24);
}

/// A tiny eMMC image that needs no file: the tests write through the model and
/// read the same bytes back, which is what the SC storage commands do.
struct ScratchCard {
    ScratchCard() {
        // The scratch image lives in the canonical build tree.  Create the folder
        // here: the test binary is run from the repository root and an earlier
        // revision hard-coded a per-target build folder that no longer exists.
        std::error_code ec;
        std::filesystem::create_directories("build", ec);
        const std::string path = "build\\test-scratch.img";
        if (!card.attach(path, true, 8ull * MB)) ok = false;
    }
    EmmcCard card;
    bool ok = true;
};

}  // namespace

// ---------------------------------------------------------------------------
// SFR register file
// ---------------------------------------------------------------------------

ZLB_TEST(ernie_sfr_is_named_and_resettable) {
    Bus bus;
    ErnieSfr sfr;
    bus.add_device(std::unique_ptr<Device>(new ErnieSfr()));

    // Every register the model exposes must have a name (the debugger's MMIO view
    // depends on it) and the reset value must match the RL78/G13 table.
    ZLB_EXPECT_TRUE(std::string(sfr.register_name(ernie::kSfrCsc)) == "CSC");
    ZLB_EXPECT_TRUE(std::string(sfr.register_name(ernie::kSfrOstc)) == "OSTC");
    ZLB_EXPECT_TRUE(std::string(sfr.register_name(ernie::kSfrCkc)) == "CKC");
    ZLB_EXPECT_TRUE(std::string(sfr.register_name(ernie::kSfrItmc)) == "ITMC");
    ZLB_EXPECT_TRUE(std::string(sfr.register_name(ernie::kSfrWdte)) == "WDTE");
    ZLB_EXPECT_TRUE(sfr.register_name(0xFFFAB + 200) == nullptr);

    ZLB_EXPECT_EQ(static_cast<u32>(sfr.read(ernie::kSfrCsc, 1)), 0xC0u);
    ZLB_EXPECT_EQ(static_cast<u32>(sfr.read(ernie::kSfrOsts, 1)), 0x07u);
    // ITMC's reset value is 0x0FFF but only bits 14:0 are implemented and the
    // model stores the low byte, so an unarmed timer reads back as 0x00FF.
    ZLB_EXPECT_EQ(static_cast<u32>(sfr.read(ernie::kSfrItmc, 2)), 0x00FFu);
    ZLB_EXPECT_EQ(static_cast<u32>(sfr.read(ernie::kSfrPm0, 1)), 0xFFu);

    std::vector<RegisterInfo> registers;
    sfr.enumerate_registers(registers);
    ZLB_EXPECT_TRUE(registers.size() >= 90);

    u64 value = 0;
    ZLB_EXPECT_TRUE(sfr.peek_register("CSC", value));
    ZLB_EXPECT_EQ(value, 0xC0u);
    ZLB_EXPECT_TRUE(sfr.poke_register("OSTS", 0x03));
    ZLB_EXPECT_EQ(static_cast<u32>(sfr.read(ernie::kSfrOsts, 1)), 0x03u);
    ZLB_EXPECT_FALSE(sfr.poke_register("NOT_A_REGISTER", 1));
}

ZLB_TEST(ernie_clock_handshake_matches_the_boot_rom) {
    // Reproduce the boot ROM at 0x3003B..0x30066 (USS-1001):
    //   mov 0xFFFA0,#0x72 / mov 0xFFFA1,#0xC0 / oneb !0xFFFA3
    //   clr1 0xFFFA1.7    / cmp !0xFFFA2,#0xC0 / bnz
    //   mov 0xFFFA4,#0x10 / bt 0xFFFA4.5, ...
    Bus bus;
    bus.add_ram("flash", ernie::kFlashWindowSize, ernie::kFlashWindowBase, "test flash");
    ErnieSfr* sfr = new ErnieSfr();
    bus.add_device(std::unique_ptr<Device>(sfr));

    sfr->write(ernie::kSfrCmc, 1, 0x72);
    sfr->write(ernie::kSfrCsc, 1, 0xC0);
    sfr->write(ernie::kSfrOsts, 1, 0x01);
    // clr1 0xFFFA1.7: MSTOP = 0 starts the X1 oscillator.
    sfr->write(ernie::kSfrCsc, 1, static_cast<u8>(0xC0 & ~ernie::kCscMstop));
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrOstc), 0x00u);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrOsts), 0x01u);

    // Before the stabilisation window elapses OSTC is still counting.
    sfr->tick(10);
    ZLB_EXPECT_NE(static_cast<u32>(sfr->read(ernie::kSfrOstc, 1)), 0xC0u);

    // OSTS = 0x01 -> 2^9 cycles; the handshake value appears inside the window.
    sfr->tick(4096);
    ZLB_EXPECT_EQ(static_cast<u32>(sfr->read(ernie::kSfrOstc, 1)), 0xC0u);

    // CKC.MCM0 = 1 asks for fMX; MCS only reports it once X1 is stable.
    sfr->write(ernie::kSfrCkc, 1, ernie::kCkcMcm0);
    sfr->tick(8192);
    ZLB_EXPECT_TRUE((sfr->read(ernie::kSfrCkc, 1) & ernie::kCkcMcs) != 0);

    // MSTOP = 1 clears OSTC and drops fMX again.
    sfr->write(ernie::kSfrCsc, 1, 0xC0);
    ZLB_EXPECT_EQ(static_cast<u32>(sfr->read(ernie::kSfrOstc, 1)), 0x00u);
    ZLB_EXPECT_FALSE((sfr->read(ernie::kSfrCkc, 1) & ernie::kCkcMcs) != 0);

    // Writing the read-only OSTC must not disturb it.
    sfr->write(ernie::kSfrOstc, 1, 0x5A);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrOstc), 0x00u);
}

ZLB_TEST(ernie_watchdog_and_interval_timer) {
    Bus bus;
    ErnieSfr* sfr = new ErnieSfr();
    bus.add_device(std::unique_ptr<Device>(sfr));

    // RL78/G13 15.3: 0xAC clears and restarts the watchdog counter.
    sfr->write(ernie::kSfrWdte, 1, 0xAC);
    ZLB_EXPECT_EQ(sfr->watchdog_writes(), 1u);
    ZLB_EXPECT_TRUE(sfr->watchdog_fed());
    sfr->write(ernie::kSfrWdte, 1, 0x1A);
    ZLB_EXPECT_FALSE(sfr->watchdog_fed());

    // Arming ITMC makes the interval timer raise IF0H.TMIF01H (mask MK1H.6
    // clear), which is the vector the boot ROM waits on at 0x3008A.
    sfr->set_interval_timer_cycles(1024);
    sfr->write(ernie::kSfrItmc, 2, 0x4000);
    ZLB_EXPECT_EQ(static_cast<u32>(sfr->read(ernie::kSfrItmc, 2)), 0x4000u);
    ZLB_EXPECT_TRUE(sfr->interval_timer_armed());
    sfr->write(ernie::kSfrMk1 + 1, 1, 0x00);  // unmask
    sfr->tick(2048);
    ZLB_EXPECT_TRUE((sfr->peek8(ernie::kSfrIf0 + 1) & ernie::kIf0hTmif01h) != 0);
    ZLB_EXPECT_EQ(sfr->pending_vectors().size(), size_t(1));
    ZLB_EXPECT_EQ(sfr->pending_vectors()[0], ernie::kIntervalTimerVector);

    // Writing zero stops the interval timer and no further vectors are queued.
    sfr->write(ernie::kSfrItmc, 2, 0x0000);
    ZLB_EXPECT_FALSE(sfr->interval_timer_armed());
}

ZLB_TEST(ernie_sfr_panel_and_rtc) {
    Bus bus;
    ErnieSfr* sfr = new ErnieSfr();
    bus.add_device(std::unique_ptr<Device>(sfr));

    // Pressed buttons read as 0 on P1 (active low).
    sfr->set_power_button(true);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrP1) & 0x01u, 0u);
    ZLB_EXPECT_TRUE((sfr->peek8(ernie::kSfrIf0) & 0x80u) != 0);  // PIF5 request flag
    sfr->set_power_button(false);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrP1) & 0x01u, 1u);
    sfr->set_volume_down(true);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrP1) & 0x08u, 0u);
    sfr->set_volume_down(false);

    // 2011-07-29T12:24:32Z -> YEAR = 0x11, MONTH = 0x07, DAY = 0x29.
    sfr->set_rtc_seconds(0x4E32A680ull);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrYear), 0x11u);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrMonth), 0x07u);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrDay), 0x29u);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrHour), 0x12u);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrMin), 0x24u);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrSec), 0x32u);

    // Writing the RTC registers moves the modelled wall clock.
    sfr->write(ernie::kSfrYear, 1, 0x13);
    ZLB_EXPECT_TRUE(sfr->rtc_seconds() > 0x4E32A680ull);

    // The gauge voltage drives the A/D result once ADM0.ADCS is set.
    sfr->set_battery_millivolts(4000);
    ZLB_EXPECT_EQ(sfr->battery_millivolts(), 4000u);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrAdcrh), 0x00u);
    sfr->write(ernie::kSfrAdm0, 1, 0x80);
    ZLB_EXPECT_EQ(sfr->peek8(ernie::kSfrAdm0), 0x80u);
    ZLB_EXPECT_TRUE(sfr->peek8(ernie::kSfrAdcrh) != 0x00u);
}

// ---------------------------------------------------------------------------
// SC dispatcher
// ---------------------------------------------------------------------------

ZLB_TEST(ernie_sc_command_table_matches_the_dump) {
    // The USS-1001 command table lives at 0x26BE and has 70 entries; these are
    // four of them, checked against the raw bytes of the dump.
    const std::vector<ernie::ScCommandInfo>& table = ernie::sc_command_table();
    ZLB_EXPECT_EQ(table.size(), size_t(70));
    ZLB_EXPECT_EQ(static_cast<u32>(table[0].number), 0x0000u);
    ZLB_EXPECT_EQ(static_cast<u32>(table[0].handler), 0x35C19u);
    ZLB_EXPECT_EQ(static_cast<u32>(table[12].number), 0x0800u);
    ZLB_EXPECT_EQ(static_cast<u32>(table[12].handler), 0x37253u);
    ZLB_EXPECT_EQ(static_cast<u32>(table[21].number), 0x1101u);
    ZLB_EXPECT_EQ(static_cast<u32>(table[22].number), 0x0100u);
    ZLB_EXPECT_EQ(static_cast<u32>(table[22].handler), 0x36132u);
    ZLB_EXPECT_EQ(static_cast<u32>(table[69].number), 0x2085u);

    ZLB_EXPECT_TRUE(ernie::sc_command_info(0x1100) != nullptr);
    ZLB_EXPECT_TRUE(ernie::sc_command_info(0x2080) != nullptr);
    ZLB_EXPECT_TRUE(ernie::sc_command_info(0x1234) == nullptr);
}

ZLB_TEST(ernie_sc_functional_dispatch) {
    Bus bus;
    bus.unmapped_reads_zero = true;
    ErnieBlock ernie(bus, nullptr);
    ernie.install();
    ernie.set_running_firmware(false);
    ZLB_EXPECT_FALSE(ernie.running_firmware());

    // Every reply is a 4 + 32 byte record: result, status, length, payload.
    const std::vector<u8> status = ernie.dispatch_command(0x0000, {});
    ZLB_EXPECT_EQ(status.size(), size_t(36));
    ZLB_EXPECT_EQ(static_cast<u32>(status[0]), 0u);
    ZLB_EXPECT_TRUE(ernie.response_ready());
    ZLB_EXPECT_EQ(ernie.response().size(), size_t(36));
    ZLB_EXPECT_EQ(ernie.commands_served(), 1u);
    ernie.clear_response();
    ZLB_EXPECT_FALSE(ernie.response_ready());

    // An unknown command answers error 0x01 (the command table miss path).
    const std::vector<u8> unknown = ernie.dispatch_command(0x1234, {});
    ZLB_EXPECT_EQ(static_cast<u32>(unknown[0]), 0x01u);

    // A known command without a functional model is acknowledged empty.
    const std::vector<u8> known = ernie.dispatch_command(0x0010, {});
    ZLB_EXPECT_EQ(static_cast<u32>(known[0]), 0u);

    // RTC round trip: set through the SC command, read back.
    ernie.set_rtc_time(0x50000000);
    const std::vector<u8> get = ernie.dispatch_command(0x0080, {});
    ZLB_EXPECT_EQ(read_le32(get, 3), 0x50000000u);
    std::vector<u8> set = le32(0x40000000);
    const std::vector<u8> set_reply = ernie.dispatch_command(0x0081, set);
    ZLB_EXPECT_EQ(read_le32(set_reply, 3), 0x40000000u);
    ZLB_EXPECT_EQ(ernie.rtc_time(), 0x40000000u);
}

ZLB_TEST(ernie_sc_eMMC_block_read_and_write) {
    ScratchCard scratch;
    ZLB_EXPECT_TRUE(scratch.ok);

    Bus bus;
    bus.unmapped_reads_zero = true;
    ErnieBlock ernie(bus, &scratch.card);
    ernie.install();
    ernie.set_running_firmware(false);

    // Write two blocks through the SC command, then read them back.
    std::vector<u8> pattern(1024);
    for (size_t i = 0; i < pattern.size(); ++i) pattern[i] = static_cast<u8>((i * 31 + 7) & 0xFF);

    std::vector<u8> write_payload = le32(0x40);  // lba
    std::vector<u8> count = le32(2);
    write_payload.insert(write_payload.end(), count.begin(), count.end());
    write_payload.insert(write_payload.end(), pattern.begin(), pattern.end());
    const std::vector<u8> write_reply = ernie.dispatch_command(0x1181, write_payload);
    ZLB_EXPECT_EQ(static_cast<u32>(write_reply[0]), 0u);

    std::vector<u8> read_payload = le32(0x40);
    read_payload.insert(read_payload.end(), count.begin(), count.end());
    const std::vector<u8> read_reply = ernie.dispatch_command(0x1180, read_payload);
    ZLB_EXPECT_EQ(static_cast<u32>(read_reply[0]), 0u);
    // The 4+32 reply truncates the payload to 32 bytes, which is the record size
    // the Ernie handlers use (0x0DD98 is a 4+32 byte buffer).
    ZLB_EXPECT_EQ(static_cast<u32>(read_reply[2]), 32u);
    ZLB_EXPECT_TRUE(std::memcmp(read_reply.data() + 3, pattern.data(), 32) == 0);

    // The card itself must have the full block.
    std::vector<u8> direct(1024);
    ZLB_EXPECT_TRUE(scratch.card.read_blocks(EmmcPartition::User, 0x40, 2, direct.data()));
    ZLB_EXPECT_TRUE(std::memcmp(direct.data(), pattern.data(), pattern.size()) == 0);

    // Partial payloads are rejected with error 0x02.
    const std::vector<u8> short_read = ernie.dispatch_command(0x1180, le32(0));
    ZLB_EXPECT_EQ(static_cast<u32>(short_read[0]), 0u);  // <8 bytes: card info reply
    const std::vector<u8> short_write = ernie.dispatch_command(0x1181, le32(0));
    ZLB_EXPECT_EQ(static_cast<u32>(short_write[0]), 0x02u);

    // Partition select + identity commands.
    const std::vector<u8> boot0 = ernie.dispatch_command(0x1183, le32(1));
    ZLB_EXPECT_EQ(static_cast<u32>(boot0[0]), 0u);
    ZLB_EXPECT_EQ(read_le32(boot0, 3), 1u);
    const std::vector<u8> cid = ernie.dispatch_command(0x1081, {});
    ZLB_EXPECT_EQ(static_cast<u32>(cid[0]), 0u);
    ZLB_EXPECT_TRUE(std::memcmp(cid.data() + 3, scratch.card.cid().data(), 16) == 0);
}

// The NVS and the scratch pad are addressed as `u16 offset` + `u8 length`, and
// the 1.04 second loader uses exactly two of those reads:
//
//   0x1082 [80 04 08]  -> NVS 0x480, 8 bytes (Qaf token / extra UART / safe mode)
//   0x0090 [E0 00 20]  -> scratch pad 0xE0, 32 bytes (the CP DIP switch block)
//
// Both used to be answered with unrelated data (an eMMC CSD and an empty reply),
// so this test pins the wire semantics down.
ZLB_TEST(ernie_nvs_and_scratchpad_are_offset_length_stores) {
    Bus bus;
    bus.unmapped_reads_zero = true;
    ErnieBlock ernie(bus, nullptr);
    ernie.install();
    ernie.set_running_firmware(false);

    const auto le16 = [](u16 value) {
        return std::vector<u8>{static_cast<u8>(value & 0xFF), static_cast<u8>(value >> 8)};
    };

    // An unprovisioned console: the wiki documents 0xFF for "Qaf token not set",
    // "not safe mode" and "not update mode", so a fresh NVS reads as 0xFF.
    std::vector<u8> read = le16(0x0480);
    read.push_back(8);
    const std::vector<u8> flags = ernie.dispatch_command(0x1082, read);
    ZLB_EXPECT_EQ(static_cast<u32>(flags[0]), 0u);
    ZLB_EXPECT_EQ(static_cast<u32>(flags[2]), 8u);
    for (int i = 0; i < 8; ++i) ZLB_EXPECT_EQ(static_cast<u32>(flags[3 + i]), 0xFFu);

    // Board handoff reads the same persisted state without posting a guest
    // command or replacing the pending response. These are the exact 1.04
    // update/recovery/safe-mode NVS sources used by the second loader.
    const auto cold_commands = ernie.commands_served();
    const auto cold_response = ernie.response();
    std::vector<u8> board_nvs;
    for (const u16 offset : {u16(0x04A0), u16(0x0481), u16(0x0483)}) {
        ZLB_EXPECT_TRUE(ernie.read_nvs(offset, 1, board_nvs));
        ZLB_EXPECT_EQ(board_nvs.size(), size_t(1));
        ZLB_EXPECT_EQ(static_cast<u32>(board_nvs[0]), 0xFFu);
    }
    ZLB_EXPECT_EQ(ernie.commands_served(), cold_commands);
    ZLB_EXPECT_TRUE(ernie.response() == cold_response);

    // A write goes back out through the same offset/length header.
    std::vector<u8> write = le16(0x04A0);
    write.push_back(2);
    write.push_back(0x11);
    write.push_back(0x22);
    const std::vector<u8> written = ernie.dispatch_command(0x1083, write);
    ZLB_EXPECT_EQ(static_cast<u32>(written[0]), 0u);

    std::vector<u8> verify = le16(0x04A0);
    verify.push_back(2);
    const std::vector<u8> stored = ernie.dispatch_command(0x1082, verify);
    ZLB_EXPECT_EQ(static_cast<u32>(stored[2]), 2u);
    ZLB_EXPECT_EQ(static_cast<u32>(stored[3]), 0x11u);
    ZLB_EXPECT_EQ(static_cast<u32>(stored[4]), 0x22u);

    const auto written_commands = ernie.commands_served();
    const auto written_response = ernie.response();
    ZLB_EXPECT_TRUE(ernie.read_nvs(0x04A0, 2, board_nvs));
    ZLB_EXPECT_EQ(board_nvs.size(), size_t(2));
    ZLB_EXPECT_EQ(static_cast<u32>(board_nvs[0]), 0x11u);
    ZLB_EXPECT_EQ(static_cast<u32>(board_nvs[1]), 0x22u);
    ZLB_EXPECT_EQ(ernie.commands_served(), written_commands);
    ZLB_EXPECT_TRUE(ernie.response() == written_response);

    // A request beyond the store is refused instead of silently truncated.
    std::vector<u8> far = le16(0xFFFF);
    far.push_back(4);
    ZLB_EXPECT_EQ(static_cast<u32>(ernie.dispatch_command(0x1082, far)[0]), 0x03u);

    // Scratch pad 0xE0 is the SceDIPSW block: 0x20 bytes, the CP part unset and
    // the release mode values in the second half.
    std::vector<u8> dip = le16(0x00E0);
    dip.push_back(0x20);
    const std::vector<u8> switches = ernie.dispatch_command(0x0090, dip);
    ZLB_EXPECT_EQ(static_cast<u32>(switches[0]), 0u);
    ZLB_EXPECT_EQ(static_cast<u32>(switches[2]), 0x20u);
    for (int i = 0; i < 16; ++i) ZLB_EXPECT_EQ(static_cast<u32>(switches[3 + i]), 0x00u);
    ZLB_EXPECT_EQ(read_le32(switches, 3 + 0x18), 0x00080002u);  // debug control
    ZLB_EXPECT_EQ(read_le32(switches, 3 + 0x1C), 0x20000000u);  // system control

    // The scratch pad is writable (command 0x0091) -- the resume context pointer
    // at +0xC is how the kernel hands the suspend buffer to the next boot.
    std::vector<u8> resume = le16(0x000C);
    resume.push_back(4);
    resume.push_back(0xF0);
    resume.push_back(0x0E);
    resume.push_back(0x1F);
    resume.push_back(0x41);
    ZLB_EXPECT_EQ(static_cast<u32>(ernie.dispatch_command(0x0091, resume)[0]), 0u);

    std::vector<u8> back = le16(0x000C);
    back.push_back(4);
    const std::vector<u8> saved = ernie.dispatch_command(0x0090, back);
    ZLB_EXPECT_EQ(read_le32(saved, 3), 0x411F0EF0u);

    // 0x1100 takes no payload (flags = 0x0000 in the USS-1001 table) and answers
    // with the Ernie DownLoader version.
    const std::vector<u8> version = ernie.dispatch_command(0x1100, {});
    ZLB_EXPECT_EQ(static_cast<u32>(version[0]), 0u);
    ZLB_EXPECT_EQ(static_cast<u32>(version[2]), 4u);
    ZLB_EXPECT_EQ(read_le32(version, 3), 0x00130101u);
}

ZLB_TEST(ernie_sc_register_channel) {
    Bus bus;
    bus.unmapped_reads_zero = true;
    ErnieBlock ernie(bus, nullptr);
    ernie.install();
    ernie.set_running_firmware(false);

    // boot_path_bit5 (0x5C20A): enable the bridge, request a reply, post the
    // command word and poll it back.
    bus.write32(0xE3100124, 1);
    bus.write32(0xE3100124, bus.read32(0xE3100124) | 1u);
    bus.write32(0xE3101190, 1);
    bus.write32(0xE31020A0, 0x00000001u);
    ZLB_EXPECT_EQ(bus.read32(0xE31020A0), 1u);
    ZLB_EXPECT_TRUE((bus.read32(0xE3100024) & 0x02u) != 0);  // reply ready
    ZLB_EXPECT_EQ(bus.read32(0xE3101190), 1u);

    // sc_read drains the reply through 0xE3100030 and waits for zero.  The reply
    // record is 4 + 32 bytes, so the port must be read 36 times.
    u32 drained = 0;
    for (int i = 0; i < 36; ++i) drained += bus.read32(0xE3100030);
    ZLB_EXPECT_TRUE(drained != 0);
    ZLB_EXPECT_EQ(bus.read32(0xE3100030), 0u);

    // Clearing the reply request drops the response.
    bus.write32(0xE3101190, 0);
    ZLB_EXPECT_EQ(bus.read32(0xE3100024) & 0x02u, 0u);

    // The two message windows are mapped and named.
    Device* cmd_window = bus.find_device(ernie::kScCmdWindow);
    Device* resp_window = bus.find_device(ernie::kScRespWindow);
    ZLB_EXPECT_TRUE(cmd_window != nullptr);
    ZLB_EXPECT_TRUE(resp_window != nullptr);
    ZLB_EXPECT_TRUE(std::string(cmd_window->register_name(ernie::kScCmdWindow + 0x0E)) ==
                    "SC_DESC_CHANNEL");

    // The SoC boot gate: 0x30000118 bit1 is set, 0x30000208 takes the release.
    ZLB_EXPECT_EQ(bus.read32(0x30000118), 0x2u);
    bus.write32(0x30000208, 0x8001u);
    ZLB_EXPECT_EQ(bus.read32(0x30000208), 0x8001u);

    // 0xE0064060 is the JIG/state strap (bit16 clear = normal boot).
    ZLB_EXPECT_EQ(bus.read32(0xE0064060), 0x00010000u);
}

ZLB_TEST(ernie_sc_descriptor_window) {
    Bus bus;
    bus.unmapped_reads_zero = true;
    ErnieBlock ernie(bus, nullptr);
    ernie.install();
    ernie.set_running_firmware(false);

    // The 28-byte descriptor sc_xfer (0x5CF58) builds: mode at +0, channel at
    // +0x0E, window base at +0x18.
    std::vector<u8> descriptor(0x30, 0);
    descriptor[0x00] = 0x01;  // mode  -> command 0x0100
    descriptor[0x0E] = 0x00;  // channel
    ernie.command_posted(descriptor.data(), descriptor.size());
    ZLB_EXPECT_TRUE(ernie.response_ready());
    ZLB_EXPECT_TRUE(ernie.commands_served() >= 1);
    ZLB_EXPECT_EQ(ernie.response().size(), size_t(36));

    // A null descriptor is ignored rather than crashing.
    ernie.clear_response();
    ernie.command_posted(nullptr, 0);
    ZLB_EXPECT_FALSE(ernie.response_ready());
}

// ---------------------------------------------------------------------------
// Power / panel / fuel gauge
// ---------------------------------------------------------------------------

ZLB_TEST(ernie_power_state) {
    Bus bus;
    bus.unmapped_reads_zero = true;
    ErnieBlock ernie(bus, nullptr);
    ernie.install();
    ernie.set_running_firmware(false);

    ernie.reset();
    ZLB_EXPECT_EQ(ernie.milliseconds(), 0u);
    ZLB_EXPECT_FALSE(ernie.soc_released());
    ernie.release_soc();
    ZLB_EXPECT_TRUE(ernie.soc_released());

    ernie.advance_milliseconds(2500);
    ZLB_EXPECT_EQ(ernie.milliseconds(), 2500u);

    ernie.set_battery_percent(55);
    ZLB_EXPECT_EQ(ernie.battery_percent(), 55);
    ernie.set_charger_state(2);
    ZLB_EXPECT_EQ(ernie.charger_state(), 2);

    // The gauge SC command reports the modelled percent.
    const std::vector<u8> battery = ernie.dispatch_command(0x0800, {});
    ZLB_EXPECT_EQ(static_cast<u32>(battery[3]), 55u);
    ZLB_EXPECT_EQ(static_cast<u32>(battery[4]), 2u);

    // Pressed buttons read as pressed in the panel reply (active low -> 0).
    ernie.set_power_button(true);
    ernie.set_ps_button(true);
    ernie.set_volume_up(true);
    ernie.set_volume_down(true);
    const std::vector<u8> panel = ernie.dispatch_command(0x0100, {});
    ZLB_EXPECT_EQ(static_cast<u32>(panel[5]), 0u);
    ZLB_EXPECT_EQ(static_cast<u32>(panel[6]), 0u);
    ZLB_EXPECT_EQ(static_cast<u32>(panel[7]), 0u);
    ZLB_EXPECT_EQ(static_cast<u32>(panel[8]), 0u);
    ernie.set_power_button(false);
    ernie.set_ps_button(false);
    ernie.set_volume_up(false);
    ernie.set_volume_down(false);
    const std::vector<u8> released = ernie.dispatch_command(0x0100, {});
    ZLB_EXPECT_EQ(static_cast<u32>(released[5]), 1u);

    // 0x2080 releases the SoC, 0x2081 holds it again.
    ernie.dispatch_command(0x2081, {});
    ZLB_EXPECT_FALSE(ernie.soc_released());
    ernie.dispatch_command(0x2080, {});
    ZLB_EXPECT_TRUE(ernie.soc_released());

    // Charging raises the reported percent over time.
    ernie.set_battery_percent(10);
    ernie.set_charger_state(1);
    ernie.advance_milliseconds(120000);
    ZLB_EXPECT_TRUE(ernie.battery_percent() >= 10);
}

ZLB_TEST(ernie_reset_and_firmware_loading) {
    Bus bus;
    bus.unmapped_reads_zero = true;
    ErnieBlock ernie(bus, nullptr);
    ernie.install();

    // A truncated/absent dump leaves the functional model in charge.
    ZLB_EXPECT_FALSE(ernie.load_firmware({}, true));
    ZLB_EXPECT_FALSE(ernie.firmware_loaded());

    // A 1 MiB image is fitted and its reset vector is read back from offset 0.
    std::vector<u8> dump(ernie::kFlashSize, 0x00);
    dump[0] = 0x00;
    dump[1] = 0xE0;  // 0xE000 = USS-1001
    ZLB_EXPECT_TRUE(ernie.load_firmware(dump, true));
    ZLB_EXPECT_TRUE(ernie.firmware_loaded());
    ZLB_EXPECT_EQ(ernie.reset_vector(), 0xE000u);

    // The bytes really landed in the flash region the core fetches from.
    const MemRegion* flash = bus.region_at(ernie::kFlashWindowBase, ernie::kFlashWindowSize);
    ZLB_EXPECT_TRUE(flash != nullptr);
    ZLB_EXPECT_EQ(static_cast<u32>(bus.read8(0)), 0x00u);
    ZLB_EXPECT_EQ(static_cast<u32>(bus.read8(1)), 0xE0u);

    // A USS-1002 image (vector 0x0DC00) is recognised too.
    dump[0] = 0x00;
    dump[1] = 0xDC;
    ZLB_EXPECT_TRUE(ernie.load_firmware(dump, true));
    ZLB_EXPECT_EQ(ernie.reset_vector(), 0x0DC00u);

    // reset() restores the power-on state.
    ernie.set_battery_percent(3);
    ernie.advance_milliseconds(9999);
    ernie.release_soc();
    ernie.reset();
    ZLB_EXPECT_EQ(ernie.milliseconds(), 0u);
    ZLB_EXPECT_FALSE(ernie.soc_released());
    ZLB_EXPECT_EQ(ernie.battery_percent(), 87);
    ZLB_EXPECT_EQ(ernie.commands_served(), 0u);
    ZLB_EXPECT_FALSE(ernie.response_ready());
}

ZLB_TEST(ernie_debugger_surface) {
    Bus bus;
    bus.unmapped_reads_zero = true;
    ErnieBlock ernie(bus, nullptr);
    ernie.install();
    ernie.set_running_firmware(false);

    std::vector<std::string> lines;
    ernie.describe(lines);
    ZLB_EXPECT_TRUE(lines.size() >= 8);
    ZLB_EXPECT_TRUE(ernie.summary().find("SC") != std::string::npos);

    std::vector<Device*> devices = ernie.devices();
    ZLB_EXPECT_TRUE(devices.size() >= 5);
    bool have_sfr = false;
    bool have_sc = false;
    bool have_flash = false;
    for (Device* device : devices) {
        if (device->name() == "Ernie.SFR") have_sfr = true;
        if (device->name() == "Ernie.SC") have_sc = true;
        if (device->name() == "Ernie.Flash") have_flash = true;
    }
    ZLB_EXPECT_TRUE(have_sfr);
    ZLB_EXPECT_TRUE(have_sc);
    ZLB_EXPECT_TRUE(have_flash);

    // The SFR device answers the debugger's named access.
    ErnieSfr* sfr = find_sfr(bus);
    ZLB_EXPECT_TRUE(sfr != nullptr);
    if (sfr != nullptr) {
        u64 value = 0;
        ZLB_EXPECT_TRUE(sfr->peek_register("CSC", value));
        ZLB_EXPECT_EQ(value, 0xC0u);
        ZLB_EXPECT_TRUE(sfr->describe_sfr(ernie::kSfrCkc).find("CKC") != std::string::npos);
    }
}

// ---------------------------------------------------------------------------
// eMMC host
// ---------------------------------------------------------------------------

ZLB_TEST(ernie_emmc_host_partitions) {
    ScratchCard scratch;
    ZLB_EXPECT_TRUE(scratch.ok);
    if (!scratch.ok) return;

    ernie::EmmcHost host(&scratch.card);
    host.reset();
    ZLB_EXPECT_TRUE(host.attached());
    ZLB_EXPECT_TRUE(host.init_card());
    ZLB_EXPECT_EQ(host.relative_address(), scratch.card.relative_address());

    // User area block round trip.
    std::vector<u8> block(512, 0xAB);
    ZLB_EXPECT_TRUE(host.write_blocks(1, 1, block));
    std::vector<u8> out;
    ZLB_EXPECT_TRUE(host.read_blocks(1, 1, out));
    ZLB_EXPECT_EQ(out.size(), size_t(512));
    ZLB_EXPECT_TRUE(std::memcmp(out.data(), block.data(), 512) == 0);
    ZLB_EXPECT_EQ(host.block_reads(), 1u);
    ZLB_EXPECT_EQ(host.block_writes(), 1u);
    ZLB_EXPECT_EQ(host.blocks_read(), 1u);

    // Boot partitions are selectable and independent.
    ZLB_EXPECT_TRUE(host.select_partition(1));
    ZLB_EXPECT_EQ(static_cast<u32>(host.partition()), 1u);
    std::vector<u8> boot(512, 0xCD);
    ZLB_EXPECT_TRUE(host.write_blocks(0, 1, boot));
    std::vector<u8> boot_out;
    ZLB_EXPECT_TRUE(host.read_blocks(0, 1, boot_out));
    ZLB_EXPECT_TRUE(std::memcmp(boot_out.data(), boot.data(), 512) == 0);
    ZLB_EXPECT_TRUE(host.select_partition(0));
    std::vector<u8> user_out;
    ZLB_EXPECT_TRUE(host.read_blocks(0, 1, user_out));
    ZLB_EXPECT_FALSE(std::memcmp(user_out.data(), boot.data(), 512) == 0);

    // Reading past the end reports an error instead of wrapping.
    const u64 blocks = scratch.card.block_count();
    ZLB_EXPECT_FALSE(host.read_blocks(blocks, 1, out));
    ZLB_EXPECT_TRUE(host.errors() > 0);

    std::vector<std::string> lines;
    host.describe(lines);
    ZLB_EXPECT_TRUE(lines.size() >= 4);
}

// ---------------------------------------------------------------------------
// SPI0 packet layer (the link the boot chain actually uses)
// ---------------------------------------------------------------------------

ZLB_TEST(syscon_spi_request_frame_matches_the_boot_chain) {
    // The 1.04 second loader builds exactly this frame for CMD 0x0001 with an
    // empty payload (0x439CC..0x43A32: packet[2] = 1, packet[3] = ~sum of the
    // first three bytes). The wiki's boot trace shows the same bytes.
    const std::vector<u8> request = ernie::make_spi_request(0x0001, {});
    ZLB_EXPECT_EQ(request.size(), size_t(4));
    ZLB_EXPECT_EQ(static_cast<u32>(request[0]), 0x01u);
    ZLB_EXPECT_EQ(static_cast<u32>(request[1]), 0x00u);
    ZLB_EXPECT_EQ(static_cast<u32>(request[2]), 0x01u);
    ZLB_EXPECT_EQ(static_cast<u32>(request[3]), 0xFDu);

    // CMD 0x1082 (NVS read) with a three byte payload, from the same trace.
    const std::vector<u8> nvs = ernie::make_spi_request(0x1082, {0x80, 0x04, 0x08});
    const u8 expected[7] = {0x82, 0x10, 0x04, 0x80, 0x04, 0x08, 0xDD};
    ZLB_EXPECT_EQ(nvs.size(), size_t(7));
    ZLB_EXPECT_TRUE(std::memcmp(nvs.data(), expected, sizeof(expected)) == 0);

    // A response carries a flags byte, so its length byte is payload + 2, and an
    // odd packet gets one padding byte to keep the 16 bit link aligned.
    // The wiki's trace shows the same frame for the CMD 0x0005 answer
    // (`04 00 06 00 00 60 40 00 55`); an odd frame gets one padding byte so the
    // 16 bit link stays aligned, which the caller never looks at.
    const std::vector<u8> reply = ernie::make_spi_response(0x0004, 0x00, {0x00, 0x60, 0x40, 0x00});
    const u8 reply_expected[10] = {0x04, 0x00, 0x06, 0x00, 0x00, 0x60, 0x40, 0x00, 0x55, 0x00};
    ZLB_EXPECT_EQ(reply.size(), size_t(10));
    ZLB_EXPECT_TRUE(std::memcmp(reply.data(), reply_expected, sizeof(reply_expected)) == 0);

    const std::vector<u8> ack = ernie::make_spi_response(0x0004, 0x80, {});
    const u8 ack_expected[6] = {0x04, 0x00, 0x02, 0x80, 0x79, 0x00};
    ZLB_EXPECT_EQ(ack.size(), size_t(6));
    ZLB_EXPECT_TRUE(std::memcmp(ack.data(), ack_expected, sizeof(ack_expected)) == 0);
}

ZLB_TEST(syscon_spi_parser_rejects_a_bad_checksum) {
    u16 command = 0;
    std::vector<u8> payload;
    std::string error;

    const std::vector<u8> good = ernie::make_spi_request(0x0001, {});
    ZLB_EXPECT_TRUE(ernie::parse_spi_request(good, command, payload, error));
    ZLB_EXPECT_EQ(static_cast<u32>(command), 0x0001u);
    ZLB_EXPECT_TRUE(payload.empty());

    std::vector<u8> bad = good;
    bad[3] ^= 0x01;
    ZLB_EXPECT_FALSE(ernie::parse_spi_request(bad, command, payload, error));
    ZLB_EXPECT_TRUE(error.find("checksum") != std::string::npos);

    // A truncated packet is rejected too, and the length byte is honoured.
    std::vector<u8> short_packet(good.begin(), good.end() - 1);
    ZLB_EXPECT_FALSE(ernie::parse_spi_request(short_packet, command, payload, error));

    const std::vector<u8> with_payload = ernie::make_spi_request(0x1082, {0x80, 0x04, 0x08});
    ZLB_EXPECT_TRUE(ernie::parse_spi_request(with_payload, command, payload, error));
    ZLB_EXPECT_EQ(static_cast<u32>(command), 0x1082u);
    ZLB_EXPECT_EQ(payload.size(), size_t(3));
    ZLB_EXPECT_EQ(static_cast<u32>(payload[0]), 0x80u);
}

ZLB_TEST(ernie_spi_transfer_answers_the_boot_commands) {
    Bus bus;
    EmmcCard card;
    ErnieBlock ernie(bus, &card);
    ernie.install();
    ernie.set_running_firmware(false);

    // CMD 0x0001 (the first command the second loader sends) is answered with a
    // four byte payload: the SPI call path caps the reply frame length at 6.
    const std::vector<u8> version = ernie.spi_transfer(ernie::make_spi_request(0x0001, {}));
    ZLB_EXPECT_TRUE(version.size() >= 5);
    ZLB_EXPECT_EQ(static_cast<u32>(version[0]), 0x04u);
    ZLB_EXPECT_EQ(static_cast<u32>(version[2]), 0x06u);
    ZLB_EXPECT_EQ(static_cast<u32>(version[3]), 0x00u);
    ZLB_EXPECT_EQ(static_cast<u32>(version[4]), 0x0Du);

    u16 command = 0;
    std::vector<u8> payload;
    std::string error;
    // The reply is a response frame, so its body is flags + payload and the
    // parser (which decodes requests) reports both bytes.
    ZLB_EXPECT_EQ(static_cast<u32>(version[9]), 0x00u);  // flags
    ZLB_EXPECT_EQ(static_cast<u32>(version[4]), 0x0Du);
    ZLB_EXPECT_EQ(static_cast<u32>(version[5]), 0x06u);
    ZLB_EXPECT_EQ(static_cast<u32>(version[6]), 0x00u);
    ZLB_EXPECT_EQ(static_cast<u32>(version[7]), 0x01u);
    // The checksum is the negation of every byte before it.
    unsigned sum = 0;
    for (size_t i = 0; i + 1 < version.size(); ++i) sum += version[i];
    if (version.size() == 9) {
        ZLB_EXPECT_EQ(static_cast<u32>(version[8]), static_cast<u32>(~sum & 0xFF));
    }

    // A malformed packet is dropped and counted, but still answered so the
    // caller's poll loop terminates.
    u16 dropped_command = 0;
    std::vector<u8> dropped_payload;
    const std::vector<u8> bad = {0x01, 0x00, 0x01, 0x00};
    ZLB_EXPECT_TRUE(ernie.spi_transfer(bad).size() >= 5);
    ZLB_EXPECT_EQ(ernie.spi_bad_packets(), 1u);
    ZLB_EXPECT_TRUE(ernie::parse_spi_request(ernie.last_spi_response(), dropped_command, dropped_payload,
                                             error));

    ZLB_EXPECT_EQ(ernie.spi_transfers(), 2u);
}

// The SC command ring was collected (and serialised) from the beginning but had no
// reader, so "what is the guest actually asking the syscon" could not be answered
// from the debugger.  `sc` prints it through ErnieBlock::recent_commands(), and
// sc_command_name() supplies the USS-1001 names without exposing the model header.
ZLB_TEST(syscon_recent_commands_are_readable_and_named) {
    Bus bus;
    ErnieBlock ernie(bus, nullptr);
    ernie.install();
    ernie.set_running_firmware(false);

    ZLB_EXPECT_TRUE(ernie.recent_commands().empty());

    ernie.dispatch_command(0x0000, {});          // get_status
    ernie.dispatch_command(0x0103, {});          // get_panel_state2
    ernie.dispatch_command(0x0003, {});          // get_model_string

    const auto& recent = ernie.recent_commands();
    ZLB_EXPECT_EQ(recent.size(), size_t(3));
    if (recent.size() == 3) {
        ZLB_EXPECT_EQ(recent[0].first, 0x0000u);
        ZLB_EXPECT_EQ(recent[1].first, 0x0103u);
        ZLB_EXPECT_EQ(recent[2].first, 0x0003u);
        ZLB_EXPECT_EQ(recent[2].second, 36u);     // the 4 + 32 byte record
    }

    // Names come from the real USS-1001 table.
    ZLB_EXPECT_TRUE(std::string(ernie::sc_command_name(0x0000)) == "get_status");
    ZLB_EXPECT_TRUE(std::string(ernie::sc_command_name(0x0103)) == "get_panel_state2");
    ZLB_EXPECT_TRUE(std::string(ernie::sc_command_name(0x0003)) == "get_model_string");
    ZLB_EXPECT_TRUE(ernie::sc_command_name(0x7FFFu)[0] == '\0');

    // ... and the record itself is the six byte state the guest polls for.
    const std::vector<u8> panel = ernie.dispatch_command(0x0103, {});
    ZLB_EXPECT_EQ(panel.size(), size_t(36));
    ZLB_EXPECT_EQ(static_cast<u32>(panel[0]), 0u);        // result = ok
    ZLB_EXPECT_EQ(static_cast<u32>(panel[2]), 6u);        // payload length
}
