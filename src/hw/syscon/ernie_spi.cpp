// zeliboba - Ernie (syscon) SPI link.
//
// The SoC talks to the syscon over SPI0 (0xE0A00000, see hw/soc/spi.cpp). This
// file implements the packet layer of that link. The format is the one the
// VitaDevWiki "Ernie" page documents (and the one the boot chain's own drivers
// enforce - both the CMeP second loader at 0x43C92 and the ARM syscon.elf at
// 0x8100003C recompute the checksum the same way):
//
//   request   [cmd_lo][cmd_hi][len][payload (len-1 bytes)][checksum]
//   response  [cmd_lo][cmd_hi][len][flags][payload (len-2 bytes)][checksum]
//
//   * `len` counts the bytes after the length byte plus one, so a response
//     always carries at least the flags byte (the loader rejects len < 2);
//   * the checksum is the binary negation of the sum of every byte that
//     precedes it, i.e. the last byte of the packet;
//   * the total packet length is len + 3 bytes.
//
// Verified against the wiki's 3.60 boot log: CMD 0x0001 is the four byte packet
// `01 00 01 FD`, which is exactly what the 1.04 second loader pushes out of the
// SPI FIFO at 0x4376C, and the response to 0x0005 (`04 00 06 00 00 60 40 00 55`)
// recomputes to the logged 0x55.
//
// The responder below is the functional model: it decodes the request, hands the
// command to ErnieBlock::dispatch_command (the same dispatcher the CMeP's SC
// mailbox uses) and frames the answer. Only the commands the boot chain asks for
// are answered with real data; everything else is answered with an empty, valid
// packet so the caller's state machine advances instead of spinning.
#include <algorithm>
#include <cstring>

#include "common/log.h"
#include "common/util.h"
#include "hw/syscon.h"
#include "hw/syscon/ernie_internal.h"

namespace zlb {
namespace ernie {

/// Response codes. The 1.04 second loader accepts exactly this set (0x43D1C..
/// 0x43D40): {3, 4} go to the "extract boot info" path, {0x100, 0x101, 0x104}
/// to the second table and 0x103 to the scratch pad path.
constexpr u16 kSpiResponseNoData = 0x0004;   ///< acknowledgement, no payload
constexpr u16 kSpiResponseScratch = 0x0103;  ///< scratch pad read/write answer
constexpr u16 kSpiResponseAck = 0x0024;      ///< "no data pending" ack class (bit 5)
constexpr u8 kSpiFlagAck = 0x80;             ///< "acknowledged, data follows later"

namespace {

/// Negation of the sum of the first `count` bytes. The packet checksum is the
/// negation of every byte that precedes it, and the position of the checksum is
/// the last byte of the packet (request and response alike).
u8 checksum_over(const std::vector<u8>& packet, size_t count) {
    unsigned sum = 0;
    for (size_t i = 0; i < count && i < packet.size(); ++i) sum += packet[i];
    return static_cast<u8>(~sum);
}

u16 load_u16_le(const std::vector<u8>& bytes, size_t offset) {
    return static_cast<u16>(bytes[offset] | (bytes[offset + 1] << 8));
}

}  // namespace

std::vector<u8> make_spi_request(u16 command, const std::vector<u8>& payload) {
    std::vector<u8> packet;
    packet.push_back(static_cast<u8>(command & 0xFF));
    packet.push_back(static_cast<u8>(command >> 8));
    packet.push_back(static_cast<u8>(payload.size() + 1));
    packet.insert(packet.end(), payload.begin(), payload.end());
    packet.push_back(checksum_over(packet, packet.size()));
    return packet;
}

std::vector<u8> make_spi_response(u16 command, u8 flags, const std::vector<u8>& payload) {
    std::vector<u8> packet;
    packet.push_back(static_cast<u8>(command & 0xFF));
    packet.push_back(static_cast<u8>(command >> 8));
    packet.push_back(static_cast<u8>(payload.size() + 2));
    packet.push_back(flags);
    packet.insert(packet.end(), payload.begin(), payload.end());
    packet.push_back(checksum_over(packet, packet.size()));
    // The link is 16 bit wide, so an odd packet gets one padding byte after the
    // checksum. That byte is the "unknown" byte the wiki's response dumps show.
    if ((packet.size() & 1u) != 0) packet.push_back(0);
    return packet;
}

bool parse_spi_request(const std::vector<u8>& packet, u16& command, std::vector<u8>& payload,
                       std::string& error) {
    command = 0;
    payload.clear();
    if (packet.size() < 4) {
        error = format("packet is %zu bytes, the header alone needs 4", packet.size());
        return false;
    }
    const u8 length = packet[2];
    if (length < 1) {
        error = format("length byte is %u, needs at least 1", length);
        return false;
    }
    if (packet.size() < static_cast<size_t>(length) + 3) {
        error = format("length byte %u needs %u bytes but the packet has %zu", length, length + 3,
                       packet.size());
        return false;
    }
    const u8 expect = checksum_over(packet, static_cast<size_t>(length) + 2);
    const u8 actual = packet[length + 2];
    if (expect != actual) {
        error = format("checksum 0x%02X, packet says 0x%02X", expect, actual);
        return false;
    }
    command = load_u16_le(packet, 0);
    payload.assign(packet.begin() + 3, packet.begin() + 3 + (length - 1));
    return true;
}

/// Response code for a command.
///
/// The boot chain's poll helper (second loader 0x449F0) sends CMD 0x0000 and
/// then waits for a response whose *code* satisfies a mask its caller picked:
///
///   0x44986 / 0x44ACE   mask 0x20   - wait for an acknowledgement class reply
///   0x44A80             mask 0x04   - wait for a reply that carries data
///
/// so the syscon has to answer the same command with 0x0024 (bit 5, "nothing
/// pending, this is the ack") or 0x0004 (bit 2, "here is the data"), which is
/// exactly the pair the VitaDevWiki boot trace shows. The model has no pending
/// asynchronous data yet, so 0x0000 always answers with the ack class; every
/// command that returns a payload answers with 0x0004.
u16 response_code_for(u16 command) {
    switch (command) {
        case 0x0000:  // get_status, the "is there anything for me?" poll
        case 0x0080:
            return kSpiResponseAck;
        default:
            return kSpiResponseOk;
    }
}

}  // namespace ernie

std::vector<u8> ErnieBlock::spi_transfer(const std::vector<u8>& request) {
    using ernie::make_spi_response;
    using ernie::parse_spi_request;
    using ernie::kSpiResponseOk;

    ++spi_transfers_;
    last_spi_request_ = request;

    u16 command = 0;
    std::vector<u8> payload;
    std::string error;
    if (!parse_spi_request(request, command, payload, error)) {
        ++spi_bad_packets_;
        ZLB_LOG_WARN("ernie", "SPI: dropping malformed request (%s)", error.c_str());
        last_spi_response_ = make_spi_response(ernie::kSpiResponseNoData, ernie::kSpiFlagAck, {});
        return last_spi_response_;
    }

    // The dispatcher returns the internal 36 byte "result record"
    // (result, status, payload length, payload...). Its payload is what the
    // packet layer transports.
    const std::vector<u8> record = dispatch_command(command, payload);
    std::vector<u8> answer;
    u8 flags = 0;
    u16 code = ernie::response_code_for(command);
    if (record.size() >= 3) {
        const u8 result = record[0];
        // The second loader's record command moves 40 bytes, so the SPI layer
        // transports up to 64 bytes of the dispatcher's payload.
        const u8 length = std::min<u8>(record[2], 64);
        if (result != ernie::kScResultOk) {
            // Errors travel as a one byte status in the same envelope.
            answer.assign(1, result);
        } else if (length > 0) {
            answer.assign(record.begin() + 3, record.begin() + 3 + length);
        } else {
            flags = ernie::kSpiFlagAck;
        }
        if (command == 0x0090 || command == 0x0091) code = ernie::kSpiResponseScratch;
    }

    last_spi_command_ = command;
    last_spi_response_ = make_spi_response(code, flags, answer);
    std::string payload_hex;
    for (size_t i = 0; i < payload.size() && i < 16; ++i) {
        payload_hex += format("%02X ", payload[i]);
    }
    ZLB_LOG_DBG("ernie", "SPI cmd 0x%04X (%s) payload=%zu [%s...] -> code 0x%04X payload=%zu",
                command, ernie::sc_command_info(command) ? ernie::sc_command_info(command)->name : "unknown",
                payload.size(), payload_hex.c_str(), code, answer.size());
    return last_spi_response_;
}

}  // namespace zlb
