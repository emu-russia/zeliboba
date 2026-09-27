// zeliboba - keyring controller (0xE0030000).
//
// ---------------------------------------------------------------------------
// Protocol - evidence from the annotated first loader
// ---------------------------------------------------------------------------
// init_emmc_keyrings (0x5C0B8) and check_boot_mode (0x5C0EE) are the two
// routines that touch the block at boot:
//
//   5C0DC  KeyringClearFlags (0xE0030024) <- 0x1C0F020E
//   5C0E6  KeyringClearFlags (0xE0030024) <- 0x1C0F020F
//   5C102  KeyringQueryFlagsRequest (0xE0030028) <- 0x501
//   5C104  $12 = KeyringQueryFlagsResponse (0xE003002C)
//   5C106  ($12 & 3) != 3 -> return -1 (keyring 0x501 not usable)
//
// keyring_set_value (0x5C12A) is the write path:
//
//   5C1A4..5C1FC  KeyringNewValue[0..7] <- eight 256-bit words
//   5C1FE         KeyringSetValueTrigger (0xE0030020) <- 0x501
//   5C200         KeyringClearFlags (0xE0030024) <- 0x08000501
//
// The ClearFlags word is "index in the low 13 bits, flag mask in the rest":
// 0x1C0F020E clears mask 0x1C0F on keyring 0x20E, 0x08000501 clears mask 0x0800
// (WriteByCmepAllowed) on keyring 0x501.
//
// KeyringNewValue[0] doubles as the flag word of the record:
//   5C188  `or3 $10,$4,0x4`   - $4 is the first payload word read from the
//                               strap block (0xE0062020), and bit 2 is forced
//   5C18E  `or $0,$12`        - the low 12 bits of the last word are replaced
//                               with `($2 & 0xF) << 28` (0x5C17C/0x5C18C)
// so a captured slot is {index, flags = word0 & 0xFFFF, value[32]}.
#include <cstring>

#include "common/log.h"
#include "hw/cmep/cmep_internal.h"

namespace zlb {
namespace cmep_detail {

namespace {

/// Name of KeyringNewValue[n].
const char* new_value_name(u32 word) {
    static const char* names[8] = {"KeyringNewValue[0] (flags)", "KeyringNewValue[1]",
                                   "KeyringNewValue[2]", "KeyringNewValue[3]",
                                   "KeyringNewValue[4]", "KeyringNewValue[5]",
                                   "KeyringNewValue[6]", "KeyringNewValue[7] (index/mode)"};
    return word < 8 ? names[word] : "KeyringNewValue";
}

}  // namespace

KeyringDevice::KeyringDevice(Bus& bus, SceKeys& keys, CmepBlock& owner)
    : RegisterFile("CMeP.Keyring", 0xE0030000, 0x1000), bus_(bus), keys_(keys), owner_(owner) {
    for (u32 i = 0; i < 8; ++i) define(kNewValue + 4 * i, new_value_name(i), 0);
    define(kSetValueTrigger, "KeyringSetValueTrigger", 0);
    define(kClearFlags, "KeyringClearFlags (index | mask<<13)", 0);
    define(kQueryRequest, "KeyringQueryFlagsRequest", 0);
    define(kQueryResponse, "KeyringQueryFlagsResponse", 3);
    // The remaining registers of the block are documented as scratch by the
    // wiki (keyring[0x100..] addresses); name them so the MMIO view is useful.
    define(0xE0030030, "KeyringStatus", 0);
    define(0xE0030034, "KeyringCommand", 0);
}

namespace {

// The "keyring valid" bit lives in the class (KeyringDevice::kValidBit) because
// the inline setter in the header needs it too.

}  // namespace

void KeyringDevice::reset() {
    RegisterFile::reset();
    slots_.clear();
    clear_history_.clear();
    writes_.clear();
    last_query_ = 0;
    // The answer always carries the "keyring valid" bit; see kValidBit.
    flags_response_ = kValidBit | owner_.keyring_flags();
    poke(kQueryResponse, flags_response_);
    set_value_triggers_ = 0;
    clear_flags_writes_ = 0;
    // Slot 10 is the fused boot key; mirror it into the table so the debugger can
    // show it, but read_keyring() answers from fused_value_ regardless.
    fused_value_ = fused_boot_key(keys_);
    install_slot(cmep::kBootKeyring, 0x0200, fused_value_);
}

void KeyringDevice::install_slot(u32 index, u32 flags, const std::array<u8, 32>& value) {
    KeyringSlot& slot = slots_[index];
    slot.index = index;
    slot.flags = flags;
    slot.value = value;
    slot.present = true;
    // The first loader clears ReadByCmepAllowed (0x0800) on 0x501 after the
    // write (0x5C200 writes 0x08000501) which is what locks the slot.
    slot.locked = (flags & 0x0800u) == 0;
    keys_.keyring()[index] = slot;
}

void KeyringDevice::stage_keyring(u32 index, u32 flags, const std::vector<u8>& payload) {
    std::array<u8, 32> value{};
    if (!payload.empty()) {
        const size_t n = payload.size() < 32 ? payload.size() : 32;
        std::memcpy(value.data(), payload.data(), n);
    }
    KeyringWriteRecord record;
    record.raw_trigger = index;
    record.index = index;
    record.flags = flags;
    record.value = value;
    record.had_payload = !payload.empty();
    writes_.push_back(record);
    install_slot(index, flags, value);
}

bool KeyringDevice::read_keyring(u32 index, std::vector<u8>& out) const {
    auto it = slots_.find(index);
    if (it == slots_.end() || !it->second.present) {
        // The fused boot slot is part of the chip, not of the RAM-backed slot
        // table, so it answers even before (or after) the first loader "loads" it
        // from image+0xE0 and whatever a bus reset does to the table.
        if (index == cmep::kBootKeyring) {
            out.assign(fused_value_.begin(), fused_value_.end());
            return true;
        }
        return false;
    }
    out.assign(it->second.value.begin(), it->second.value.end());
    return true;
}

std::array<u8, 32> KeyringDevice::fused_boot_key(const SceKeys& keys) {
    std::array<u8, 32> material{};
    const std::vector<u8>& key = keys.get("ENC_KEY");
    const std::vector<u8>& iv = keys.get("ENC_IV");
    const size_t key_bytes = key.size() < 16 ? key.size() : 16;
    const size_t iv_bytes = iv.size() < 16 ? iv.size() : 16;
    std::copy(key.begin(), key.begin() + static_cast<std::ptrdiff_t>(key_bytes), material.begin());
    std::copy(iv.begin(), iv.begin() + static_cast<std::ptrdiff_t>(iv_bytes), material.begin() + 16);
    return material;
}

void KeyringDevice::copy_to_bus(u32 address, const std::vector<u8>& bytes) {
    for (size_t i = 0; i < bytes.size(); ++i) {
        const u32 a = address + static_cast<u32>(i);
        if (!bus_.is_mapped(a)) return;
        bus_.write8(a, bytes[i]);
    }
}

u64 KeyringDevice::read(u32 address, unsigned size) {
    if (address == kQueryResponse) {
        // 0xE003002C answers the state of the keyring selected at 0xE0030028;
        // 3 means "no flags set", which is what check_boot_mode (0x5C106)
        // requires.
        const u32 value = flags_response_;
        return value & (size >= 8 ? ~0ull : ((1ull << (8 * size)) - 1));
    }
    return RegisterFile::read(address, size);
}

void KeyringDevice::write(u32 address, unsigned size, u64 value) {
    const u32 v = static_cast<u32>(value);
    if (address == kSetValueTrigger) {
        // keyring_set_value writes the trigger after the eight value words
        // (0x5C1A4..0x5C1FC) and before clearing the flags (0x5C200).
        const u32 index = v & 0x1FFFu;
        std::array<u8, 32> material{};
        for (u32 word = 0; word < 8; ++word) {
            const u64 staged = peek(kNewValue + 4 * word);
            // The engine consumes the 32 bytes as eight little-endian words
            // (the loop at 0x5CD64 builds a big-endian word out of four
            // consecutive bytes, i.e. the memory order is little-endian).
            material[4 * word + 0] = static_cast<u8>(staged & 0xFF);
            material[4 * word + 1] = static_cast<u8>((staged >> 8) & 0xFF);
            material[4 * word + 2] = static_cast<u8>((staged >> 16) & 0xFF);
            material[4 * word + 3] = static_cast<u8>((staged >> 24) & 0xFF);
        }
        KeyringWriteRecord record;
        record.raw_trigger = v;
        record.index = index;
        record.flags = static_cast<u32>(peek(kNewValue) & 0xFFFFu);
        record.value = material;
        record.had_payload = true;
        writes_.push_back(record);
        set_value_triggers_++;
        install_slot(index, record.flags, material);
        poke(kSetValueTrigger, v);
        return;
    }
    if (address == kClearFlags) {
        const u32 index = v & 0x1FFFu;
        const u32 mask = v & ~0x1FFFu;
        clear_history_.emplace_back(index, v);
        clear_flags_writes_++;
        auto it = slots_.find(index);
        if (it != slots_.end()) {
            it->second.flags &= ~mask;
            // 0x0800 = WriteByCmepAllowed, 0x1000 = ReadByCmepAllowed: once
            // cleared the slot is locked against CMeP itself (0x5C200).
            if (mask & 0x0800u) it->second.locked = true;
            keys_.keyring()[index] = it->second;
        }
        poke(kClearFlags, peek(kClearFlags) | v);
        return;
    }
    if (address == kQueryRequest) {
        last_query_ = v & 0xFFFFu;
        // Flag bits of the answer: 3 for a keyring that is usable, 0 for one the
        // CMeP has written but that is not present.  Keyring 0x501 additionally
        // reports the boot record flags CmepBlock::set_keyring_flags() installs,
        // which is what check_boot_mode (0x5C106) looks at.
        u32 flags = 3u;
        if (last_query_ == 0x501) {
            // 0x501 answers the boot record flags CmepBlock::set_keyring_flags()
            // installs, whatever the CMeP itself may have written into the slot:
            // check_boot_mode (0x5C106) reads them before the write.
            flags = owner_.keyring_flags();
        } else if (auto it = slots_.find(last_query_); it != slots_.end()) {
            flags = it->second.present ? 3u : 0u;
        }
        flags_response_ = kValidBit | flags;
        poke(kQueryRequest, v);
        poke(kQueryResponse, flags_response_);
        return;
    }
    RegisterFile::write(address, size, value);
}

std::string KeyringDevice::summary() const {
    return format("slots=%zu writes=%llu clears=%llu query=0x%X flags=0x%X",
                  slots_.size(), static_cast<unsigned long long>(set_value_triggers_),
                  static_cast<unsigned long long>(clear_flags_writes_), last_query_,
                  flags_response_);
}

void KeyringDevice::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("Keyring query        = 0x%X -> 0x%X", last_query_, flags_response_));
    lines.push_back(format("Keyring set triggers = %llu, clear-flag writes = %llu",
                           static_cast<unsigned long long>(set_value_triggers_),
                           static_cast<unsigned long long>(clear_flags_writes_)));
    for (const auto& entry : slots_) {
        const KeyringSlot& slot = entry.second;
        std::string hexvalue;
        for (unsigned i = 0; i < 32; ++i) hexvalue += format("%02X", slot.value[i]);
        lines.push_back(format("Keyring[0x%03X] flags=0x%04X locked=%d present=%d %s", slot.index,
                               slot.flags, slot.locked ? 1 : 0, slot.present ? 1 : 0,
                               hexvalue.c_str()));
    }
    for (const auto& entry : clear_history_) {
        lines.push_back(format("ClearFlags index=0x%03X mask=0x%05X raw=0x%08X", entry.first,
                               entry.second & ~0x1FFFu, entry.second));
    }
}

}  // namespace cmep_detail
}  // namespace zlb
