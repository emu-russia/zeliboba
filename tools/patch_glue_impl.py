p = 'src/hw/soc/kermit.cpp'
s = open(p, encoding='utf-8').read()

anchor = """std::string SysconBridge::summary() const {"""
assert anchor in s

impl = '''// ---------------------------------------------------------------------------
// PeripheralPower
// ---------------------------------------------------------------------------

PeripheralPower::PeripheralPower(std::string name, u32 base, u32 size)
    : RegisterBlock(std::move(name), base, size) {
    define(kStatus, "PERIPH_STATUS", kReadyBits);
    define(0x04, "PERIPH_CTRL04", 0);
    define(0x10, "PERIPH_CTRL10", 0);
    define(0x20, "PERIPH_CTRL20", 0);
    define(0x30, "PERIPH_CTRL30", 0);
    define(0x40, "PERIPH_CTRL40", 0);
    define(0x50, "PERIPH_CTRL50", 0);
    define(0x60, "PERIPH_CTRL60", 0);
    define(0x64, "PERIPH_CTRL64", 0);
    define(0x70, "PERIPH_ENABLE", 0);
}

u64 PeripheralPower::read_word(u32 offset, u64 stored) {
    if (offset == kStatus) return kReadyBits;
    return stored;
}

std::string PeripheralPower::summary() const {
    return format("%s ready=0x%X enable=0x%X", name_.c_str(), kReadyBits,
                  static_cast<unsigned>(peek(0x70)));
}

void PeripheralPower::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s: status=0x%X (bit 8 = powered, bit 9 = clocked), enable=0x%X", name_.c_str(),
                           kReadyBits, static_cast<unsigned>(peek(0x70))));
}

// ---------------------------------------------------------------------------
// BootHandshake
// ---------------------------------------------------------------------------

BootHandshake::BootHandshake(std::string name, u32 base, u32 size)
    : RegisterBlock(std::move(name), base, size) {
    define(kCommand0, "BOOT_CMD0", 0);
    define(kCommand1, "BOOT_CMD1", 0);
    define(kStatus, "BOOT_STATUS", 0);
    define(0x100, "BOOT_REG100", 0);
    define(0x108, "BOOT_REG108", 0);
    define(0x180, "BOOT_REG180", 0);
}

void BootHandshake::reset() {
    RegisterBlock::reset();
    last_command_ = 0;
    commands_ = 0;
}

u64 BootHandshake::write_word(u32 offset, u64 value) {
    if (offset == kCommand0 || offset == kCommand1) {
        last_command_ = static_cast<u32>(value);
        ++commands_;
        poke(kCommand0, last_command_);
    }
    poke(offset, value);
    return value;
}

u64 BootHandshake::read_word(u32 offset, u64 stored) {
    if (offset == kStatus) {
        if (last_command_ == 2) return kAckAfter2;
        if (last_command_ == 1) return kAckAfter1;
        return 0;
    }
    return stored;
}

std::string BootHandshake::summary() const {
    return format("%s commands=%llu last=0x%X status=0x%X", name_.c_str(),
                  static_cast<unsigned long long>(commands_), last_command_,
                  static_cast<unsigned>(peek(kStatus)));
}

void BootHandshake::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("  %s: %llu boot commands, last 0x%X, status 0x%X (0x44 after 2, 0x11 after 1)",
                           name_.c_str(), static_cast<unsigned long long>(commands_), last_command_,
                           static_cast<unsigned>(peek(kStatus))));
}

''' + anchor

s = s.replace(anchor, impl, 1)

# register the devices in install()
old_add = '''    d.bus.add_device(std::make_unique<kermit::SysconBridge>("Kermit.Syscon", kermit::kSysconBridgeBase,
                                                            kermit::kSysconBridgeSize));'''
new_add = '''    d.bus.add_device(std::make_unique<kermit::SysconBridge>("Kermit.Syscon", kermit::kSysconBridgeBase,
                                                            kermit::kSysconBridgeSize));
    // Peripheral power/clock glue and the boot handshake window: both are windows
    // KBL only polls, see PeripheralPower / BootHandshake.
    d.bus.add_device(std::make_unique<kermit::PeripheralPower>("Kermit.PeriphPower", kermit::kPeriphPowerBase,
                                                              kermit::kPeriphPowerSize));
    d.bus.add_device(std::make_unique<kermit::BootHandshake>("Kermit.BootHandshake", kermit::kBootHandshakeBase,
                                                             kermit::kBootHandshakeSize));'''
assert old_add in s
s = s.replace(old_add, new_add, 1)

# report them through devices()
old_owned = '''           name == "Kermit.DMA" || name == "Kermit.Display" || name == "Kermit.Syscon";'''
new_owned = '''           name == "Kermit.DMA" || name == "Kermit.Display" || name == "Kermit.Syscon" ||
           name == "Kermit.PeriphPower" || name == "Kermit.BootHandshake";'''
assert old_owned in s
s = s.replace(old_owned, new_owned, 1)

open(p, 'w', encoding='utf-8').write(s)
print('implemented and installed')
