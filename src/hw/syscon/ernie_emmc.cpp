// zeliboba - Ernie eMMC host.
//
// The syscon is the storage host for the early boot path: its RL78 firmware
// brings the card up and serves block transfers to the CMeP first loader (and to
// the secure kernel once it is running).  Only the transactor below is modelled
// here; the SC commands that drive it live in ernie_sc.cpp / ernie.cpp.
//
// Evidence for the command layout used by the SC handlers:
//
//   * the Ernie command table (USS-1001.bin 0x26BE) has three consecutive
//     storage groups: 0x1100/0x1101 (21 entries 20/21: the "data" path through
//     0x35B57/0x35B8E), 0x1080..0x1083 (card identification) and
//     0x1180..0x1185 (0x35749..0x35B36: partition / boot area handling).
//   * handler 0x1180 (0x35749) reads a 16-bit little endian field at
//     `es:[de]` (the SC payload), passes the high byte to 0x37C9B and stores the
//     result into the 4+32 byte response record at 0x0DD98.  Handler 0x1181
//     (0x3578B) compares a global at `es:!0xD67C` (the card init flag) and takes
//     the "not ready" path, which is the pattern this model reproduces.
//   * handler 0x1184 (0x35961) walks a table at `es:0xDA84[bc]` for 0x180
//     entries counting non-zero bytes -- the boot-partition presence scan.
#include <algorithm>
#include <cstring>

#include "common/log.h"
#include "common/util.h"
#include "hw/syscon.h"
#include "hw/syscon/ernie_internal.h"

namespace zlb {
namespace ernie {

EmmcHost::EmmcHost(EmmcCard* card) : card_(card) {}

void EmmcHost::reset() {
    rca_ = 1;
    partition_ = EmmcPartition::User;
    bus_width_ = 4;
    clock_hz_ = 40000000;
    card_status_ = 0x00000100;
    block_reads_ = 0;
    block_writes_ = 0;
    blocks_read_ = 0;
    blocks_written_ = 0;
    errors_ = 0;
    last_lba_ = 0;
    last_count_ = 0;
}

bool EmmcHost::init_card() {
    if (!attached()) {
        ++errors_;
        return false;
    }
    rca_ = card_->relative_address() != 0 ? card_->relative_address() : 1;
    partition_ = card_->current_partition();
    // The card answers its identification in the response record: the SC
    // "card info" command reports CID/CSD, which the card model builds from the
    // Toshiba geometry.
    card_status_ = 0x00000100;  // R1: READY_FOR_DATA, no error bits
    return true;
}

bool EmmcHost::select_partition(EmmcPartition partition) {
    if (!attached()) {
        ++errors_;
        return false;
    }
    card_->select_partition(partition);
    partition_ = card_->current_partition();
    return true;
}

bool EmmcHost::select_partition(u32 index) {
    switch (index & 0x07u) {
        case 0: return select_partition(EmmcPartition::User);
        case 1: return select_partition(EmmcPartition::Boot0);
        case 2: return select_partition(EmmcPartition::Boot1);
        case 3: return select_partition(EmmcPartition::Rpmb);
        default:
            ++errors_;
            return false;
    }
}

bool EmmcHost::read_blocks(u64 lba, u32 count, std::vector<u8>& out) {
    last_lba_ = lba;
    last_count_ = count;
    ++block_reads_;
    if (!attached()) {
        ++errors_;
        return false;
    }
    if (count == 0) return true;
    const u64 block_size = card_->block_size();
    if (lba + count > card_->block_count()) {
        ++errors_;
        ZLB_LOG_WARN("ernie.emmc", "read past the end: lba=%llu count=%u (card has %llu blocks)",
                     static_cast<unsigned long long>(lba), count,
                     static_cast<unsigned long long>(card_->block_count()));
        return false;
    }
    out.assign(static_cast<size_t>(block_size) * count, 0);
    if (!card_->read_blocks(partition_, lba, count, out.data())) {
        ++errors_;
        out.clear();
        return false;
    }
    blocks_read_ += count;
    return true;
}

bool EmmcHost::write_blocks(u64 lba, u32 count, const std::vector<u8>& data) {
    last_lba_ = lba;
    last_count_ = count;
    ++block_writes_;
    if (!attached()) {
        ++errors_;
        return false;
    }
    if (count == 0) return true;
    const u64 block_size = card_->block_size();
    if (lba + count > card_->block_count()) {
        ++errors_;
        return false;
    }
    if (data.size() < static_cast<size_t>(block_size) * count) {
        ++errors_;
        return false;
    }
    if (!card_->write_blocks(partition_, lba, count, data.data())) {
        ++errors_;
        return false;
    }
    blocks_written_ += count;
    return true;
}

bool EmmcHost::erase_blocks(u64 lba, u32 count) {
    last_lba_ = lba;
    last_count_ = count;
    if (!attached()) {
        ++errors_;
        return false;
    }
    if (!card_->erase(lba, count)) {
        ++errors_;
        return false;
    }
    return true;
}

std::string EmmcHost::summary() const {
    if (!attached()) return "no card attached";
    return format("%s, %s, part=%u, %llu blocks read / %llu written",
                  card_->path().empty() ? "(card model)" : path_filename(card_->path()).c_str(),
                  human_size(card_->capacity_bytes()).c_str(), static_cast<unsigned>(partition_),
                  static_cast<unsigned long long>(blocks_read_),
                  static_cast<unsigned long long>(blocks_written_));
}

void EmmcHost::describe(std::vector<std::string>& lines) const {
    lines.push_back("Ernie eMMC host");
    if (card_ == nullptr) {
        lines.push_back("  card      : (not constructed)");
        return;
    }
    lines.push_back(format("  card      : %s", card_->attached() ? card_->path().c_str() : "(detached)"));
    lines.push_back(format("  capacity  : %s (%llu blocks of %u)",
                           human_size(card_->capacity_bytes()).c_str(),
                           static_cast<unsigned long long>(card_->block_count()), card_->block_size()));
    lines.push_back(format("  partition : %u (0 = user, 1 = boot0, 2 = boot1, 3 = rpmb)",
                           static_cast<unsigned>(partition_)));
    lines.push_back(format("  bus       : %d bit @ %u Hz, RCA %u", bus_width_, clock_hz_, rca_));
    lines.push_back(format("  transfers : %llu reads (%llu blocks) / %llu writes (%llu blocks)",
                           static_cast<unsigned long long>(block_reads_),
                           static_cast<unsigned long long>(blocks_read_),
                           static_cast<unsigned long long>(block_writes_),
                           static_cast<unsigned long long>(blocks_written_)));
    lines.push_back(format("  errors    : %llu, last lba %llu count %u, status 0x%08X",
                           static_cast<unsigned long long>(errors_),
                           static_cast<unsigned long long>(last_lba_), last_count_, card_status_));
    if (card_->attached()) card_->describe(lines);
}

}  // namespace ernie
}  // namespace zlb
