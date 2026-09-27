#include "bus/device.h"

#include "common/util.h"

namespace zlb {

std::string Device::summary() const {
    return format("%s @ 0x%08X size=0x%X", name_.c_str(), base_, size_);
}

const char* Device::lookup_name(u32 address) const {
    auto it = names_.find(address);
    return it == names_.end() ? nullptr : it->second.c_str();
}

bool Device::peek_register(const std::string& name, u64& out) const {
    for (const auto& entry : names_) {
        if (entry.second == name) {
            out = 0;
            return false;  // unknown value: subclasses with storage override this
        }
    }
    return false;
}

bool Device::poke_register(const std::string& name, u64 value) {
    (void)name;
    (void)value;
    return false;
}

// ---------------------------------------------------------------------------
// DeviceMirror
// ---------------------------------------------------------------------------

void DeviceMirror::enumerate_registers(std::vector<RegisterInfo>& out) const {
    std::vector<RegisterInfo> inner;
    target_.enumerate_registers(inner);
    for (auto& info : inner) {
        RegisterInfo mirrored = info;
        mirrored.address = info.address - target_.base() + window_base_;
        out.push_back(std::move(mirrored));
    }
}

// ---------------------------------------------------------------------------
// RegisterFile
// ---------------------------------------------------------------------------

void RegisterFile::define(u32 address, const std::string& name, u64 reset_value, unsigned width) {
    values_[address] = reset_value;
    defaults_[address] = reset_value;
    widths_[address] = width;
    names_[address] = name;
    by_name_[name] = address;
}

u64 RegisterFile::peek(u32 address) const {
    auto it = values_.find(address);
    return it == values_.end() ? 0 : it->second;
}

void RegisterFile::poke(u32 address, u64 value) { values_[address] = value; }

void RegisterFile::set_bits(u32 address, u64 mask, bool set) {
    u64 old = peek(address);
    poke(address, set ? (old | mask) : (old & ~mask));
}

void RegisterFile::reset() {
    for (const auto& entry : defaults_) values_[entry.first] = entry.second;
}

u64 RegisterFile::read_register(u32 address, unsigned size) {
    u64 value = peek(address);
    switch (size) {
        case 1: return value & 0xFFull;
        case 2: return value & 0xFFFFull;
        case 4: return value & 0xFFFFFFFFull;
        default: return value;
    }
}

void RegisterFile::write_register(u32 address, unsigned size, u64 value) {
    (void)size;
    poke(address, value);
}

u64 RegisterFile::read(u32 address, unsigned size) { return read_register(address, size); }

void RegisterFile::write(u32 address, unsigned size, u64 value) { write_register(address, size, value); }

void RegisterFile::enumerate_registers(std::vector<RegisterInfo>& out) const {
    out.reserve(out.size() + names_.size());
    for (const auto& entry : names_) {
        RegisterInfo info;
        info.address = entry.first;
        info.name = entry.second;
        auto def = defaults_.find(entry.first);
        info.reset_value = def == defaults_.end() ? 0 : def->second;
        auto width = widths_.find(entry.first);
        info.width = width == widths_.end() ? 4u : width->second;
        out.push_back(std::move(info));
    }
}

bool RegisterFile::peek_register(const std::string& name, u64& out) const {
    auto it = by_name_.find(name);
    if (it == by_name_.end()) {
        // accept "0xADDR" as well
        u64 address = 0;
        if (parse_u64(name, address) && values_.count(static_cast<u32>(address))) {
            out = peek(static_cast<u32>(address));
            return true;
        }
        return false;
    }
    out = peek(it->second);
    return true;
}

bool RegisterFile::poke_register(const std::string& name, u64 value) {
    auto it = by_name_.find(name);
    if (it == by_name_.end()) {
        u64 address = 0;
        if (parse_u64(name, address) && values_.count(static_cast<u32>(address))) {
            poke(static_cast<u32>(address), value);
            return true;
        }
        return false;
    }
    poke(it->second, value);
    return true;
}

}  // namespace zlb
