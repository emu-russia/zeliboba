p = 'src/hw/soc/soc_internal.h'
s = open(p, encoding='utf-8').read()

anchor = """// ---------------------------------------------------------------------------
// Small helper used by several devices
// ---------------------------------------------------------------------------"""
assert anchor in s

new_classes = '''// ---------------------------------------------------------------------------
// Peripheral power / clock glue (0xE2030000..0xE209FFFF)
// ---------------------------------------------------------------------------

/// KBL brings each of the seven 64 KiB windows in its table at 0x4005C058 up with
/// one helper (0x4003BBCC): it writes +0x04, +0x10, +0x20, +0x30, +0x40, +0x50,
/// +0x60 and +0x64, then waits for a ready bit at +0x28 (0x4003BC98 tests bit 8,
/// 0x4003BCCC tests bit 9) before writing +0x70.  Nothing in the workspace
/// materials documents what +0x28 reports, so the model answers "powered and
/// clocked" and stores the configuration - the loader's own sequence (configure,
/// poll, enable) still runs unchanged.
class PeripheralPower : public RegisterBlock {
public:
    PeripheralPower(std::string name, u32 base, u32 size);

    u64 read_word(u32 offset, u64 stored) override;

    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    static constexpr u32 kStatus = 0x28;
    static constexpr u32 kReadyBits = 0x0300;  ///< bit 8 ("powered") + bit 9 ("clocked")
};

// ---------------------------------------------------------------------------
// Boot handshake window (0xE5888000)
// ---------------------------------------------------------------------------

/// KBL (0x400217FA..0x40021872) writes a command to +0x00 and +0x08 and then polls
/// +0x20 until it reads a per-command acknowledgement: 0x44 after a command of 2
/// and 0x11 after a command of 1, after which it programs +0x100/+0x108 with 0x302
/// and +0x180 with 1 and drops a 0xDEADCAFE marker at 0x80000000 (DRAM).
/// No module in the workspace references the window, so the model has to supply
/// the acknowledgements itself: it tracks the last command and answers with the
/// value that command's poll expects, which is what lets the loader reach DRAM.
class BootHandshake : public RegisterBlock {
public:
    BootHandshake(std::string name, u32 base, u32 size);

    u64 read_word(u32 offset, u64 stored) override;
    void write_word(u32 offset, u64 value) override;
    void reset() override;

    std::string summary() const override;
    void describe(std::vector<std::string>& lines) const override;

    static constexpr u32 kCommand0 = 0x00;
    static constexpr u32 kCommand1 = 0x08;
    static constexpr u32 kStatus = 0x20;
    static constexpr u32 kAckAfter2 = 0x44;
    static constexpr u32 kAckAfter1 = 0x11;

private:
    u32 last_command_ = 0;
    u64 commands_ = 0;
};

''' + anchor

s = s.replace(anchor, new_classes, 1)
open(p, 'w', encoding='utf-8').write(s)
print('classes declared')
