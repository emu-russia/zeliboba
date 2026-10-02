// zeliboba - memory mapped device interface.
//
// A device is a window of the physical address space that decodes register
// accesses. Everything the debugger needs (names, reset values, poking) is part
// of the interface so that new hardware blocks are automatically visible in the
// UI and the command line.
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "common/state.h"
#include "common/types.h"
#include "common/util.h"

namespace zlb {

struct RegisterInfo {
    u32 address = 0;
    std::string name;
    u64 reset_value = 0;
    unsigned width = 4;  // in bytes
};

class Device {
public:
    Device(std::string name, u32 base, u32 size)
        : name_(std::move(name)), base_(base), size_(size) {}
    virtual ~Device() = default;

    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    const std::string& name() const { return name_; }
    u32 base() const { return base_; }
    u32 size() const { return size_; }
    u32 end() const { return base_ + size_; }

    bool handles(u32 address) const { return address >= base_ && (address - base_) < size_; }

    /// Read `size` (1, 2, 4 or 8) bytes at `address`, zero-extended.
    virtual u64 read(u32 address, unsigned size) = 0;
    /// Write the low `size` bytes of `value` at `address`.
    virtual void write(u32 address, unsigned size, u64 value) = 0;

    /// Power-on reset. Called for every device when the machine resets.
    virtual void reset() {}

    /// Name of the register at `address`, or nullptr.
    virtual const char* register_name(u32 address) const { return lookup_name(address); }

    /// One line state summary for the debugger's `devices` command.
    virtual std::string summary() const;

    /// All named registers, for the hex/register view.
    virtual void enumerate_registers(std::vector<RegisterInfo>& out) const { (void)out; }

    /// Named register access, used by the debugger (`devset`/`devget`) and tests.
    virtual bool peek_register(const std::string& name, u64& out) const;
    virtual bool poke_register(const std::string& name, u64 value);

    /// Called once per emulated frame so devices with time can advance.
    virtual void tick(u64 cycles) { (void)cycles; }

    /// Extra state lines for the UI.
    virtual void describe(std::vector<std::string>& lines) const { (void)lines; }

    // ---- save states -----------------------------------------------------

    /// Persist the device's mutable state. A device that owns no state can keep
    /// the default; a device with state writes one or more values here and reads
    /// them back in the same order in `load_state`. Cross-device pointers and
    /// `std::function` hooks are wiring, not state: they are rebuilt by the
    /// machine and must not be serialised.
    virtual void save_state(StateWriter& writer) const { (void)writer; }
    virtual void load_state(StateReader& reader) { (void)reader; }

    /// Base class helper used by register-file devices.
    void register_name_entry(u32 address, const std::string& name) { names_[address] = name; }
    const char* lookup_name(u32 address) const;

protected:
    std::string name_;
    u32 base_;
    u32 size_;
    std::map<u32, std::string> names_;
};

/// Forwards accesses to another device, so one hardware block can be visible in
/// more than one address space (the CMeP/ARM mailbox, the SC bridge, ...).
class DeviceMirror : public Device {
public:
    DeviceMirror(Device& target, u32 base, u32 size)
        : Device(target.name() + "@mirror", base, size), target_(target), window_base_(base) {}

    Device& target() { return target_; }
    const Device& target() const { return target_; }

    u64 read(u32 address, unsigned size) override {
        return target_.read(address - window_base_ + target_.base(), size);
    }
    void write(u32 address, unsigned size, u64 value) override {
        target_.write(address - window_base_ + target_.base(), size, value);
    }
    void reset() override {}
    const char* register_name(u32 address) const override {
        return target_.register_name(address - window_base_ + target_.base());
    }
    void enumerate_registers(std::vector<RegisterInfo>& out) const override;
    bool peek_register(const std::string& name, u64& out) const override {
        return target_.peek_register(name, out);
    }
    bool poke_register(const std::string& name, u64 value) override {
        return target_.poke_register(name, value);
    }
    std::string summary() const override {
        return format("%s (mirror of %s)", name_.c_str(), target_.name().c_str());
    }
    void describe(std::vector<std::string>& lines) const override { target_.describe(lines); }

    /// A mirror holds no state of its own: the target is serialised where it is
    /// registered. Writing nothing here keeps a state file from carrying the
    /// same device twice (and from applying it out of order on load).
    void save_state(StateWriter& writer) const override { (void)writer; }
    void load_state(StateReader& reader) override { (void)reader; }

private:
    Device& target_;
    u32 window_base_;
};

/// A device backed by a plain register file. `define()` records a name and the
/// power-on value, which `reset()` restores.
class RegisterFile : public Device {
public:
    RegisterFile(std::string name, u32 base, u32 size) : Device(std::move(name), base, size) {}

    void define(u32 address, const std::string& name, u64 reset_value = 0, unsigned width = 4);

    u64 peek(u32 address) const;
    void poke(u32 address, u64 value);
    void set_bits(u32 address, u64 mask, bool set);

    void reset() override;

    u64 read(u32 address, unsigned size) override;
    void write(u32 address, unsigned size, u64 value) override;

    void enumerate_registers(std::vector<RegisterInfo>& out) const override;
    bool peek_register(const std::string& name, u64& out) const override;
    bool poke_register(const std::string& name, u64 value) override;

    /// The stored register values (the defaults/widths/names are construction
    /// time configuration and stay in the build).
    void save_state(StateWriter& writer) const override;
    void load_state(StateReader& reader) override;

protected:
    /// Overridable decode hooks; default implementation is plain storage.
    virtual u64 read_register(u32 address, unsigned size);
    virtual void write_register(u32 address, unsigned size, u64 value);

    std::map<u32, u64> values_;
    std::map<u32, u64> defaults_;
    std::map<u32, unsigned> widths_;
    std::map<std::string, u32> by_name_;
};

}  // namespace zlb
