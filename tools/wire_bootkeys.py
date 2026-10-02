import io

# 1) VitaConfig gains the flag.
h = r'.\src\machine\vita.h'
s = open(h, encoding='utf-8').read()
if 'provision_keys' not in s:
    s = s.replace('''    bool rebuild_emmc = true;        ///< build the image when it is missing''',
                  '''    bool rebuild_emmc = true;        ///< build the image when it is missing
    /// Seed the emulated CMeP key table (0xE0066000) with a development key and
    /// re-sign the staged second loader, so the first loader's RSA verification can
    /// succeed. The console's real key table is not in any dump we have; see
    /// docs/KBL.md and src/machine/bootkeys.h. The eMMC image is never modified.
    bool provision_keys = true;''', 1)
    open(h, 'w', encoding='utf-8').write(s)
    print('patched vita.h')
else:
    print('vita.h already has the flag')

# 2) bootchain.cpp provisions before staging.
c = r'.\src\machine\bootchain.cpp'
t = open(c, encoding='utf-8').read()
if 'provision_boot_keys' in t:
    print('bootchain.cpp already patched')
else:
    if '#include "machine/bootkeys.h"' not in t:
        t = t.replace('#include "machine/bootchain.h"', '#include "machine/bootchain.h"\n#include "machine/bootkeys.h"', 1)

    old = '''    if (second->data.size() > shared_sram_.size()) {
        ZLB_LOG_ERROR("machine", "second loader (%zu B) does not fit the %zu B boot SRAM", second->data.size(),
                      shared_sram_.size());
        return false;
    }

    std::copy(second->data.begin(), second->data.end(), shared_sram_.begin());'''
    new = '''    if (second->data.size() > shared_sram_.size()) {
        ZLB_LOG_ERROR("machine", "second loader (%zu B) does not fit the %zu B boot SRAM", second->data.size(),
                      shared_sram_.size());
        return false;
    }

    // Provision the emulated CMeP key table and re-sign the payload so the first
    // loader's own RSA verification can succeed (see machine/bootkeys.h).
    std::vector<u8> staged = second->data;
    if (config_.provision_keys) {
        std::string why;
        if (provision_boot_keys(*cmep_bus_, staged, why)) {
            add_milestone("CMeP boot keys provisioned (development key table at 0xE0066000)");
        } else {
            ZLB_LOG_WARN("boot", "boot key provisioning skipped: %s", why.c_str());
        }
    }

    std::copy(staged.begin(), staged.end(), shared_sram_.begin());'''
    if old not in t:
        raise SystemExit('staging block not found')
    t = t.replace(old, new, 1)

    # The bare-file fallback path should provision too.
    old2 = '''                std::copy(raw->begin(), raw->end(), shared_sram_.begin());
                second_loader_pa_ = board::kSharedSramBase;'''
    new2 = '''                std::vector<u8> staged = *raw;
                if (config_.provision_keys) {
                    std::string why;
                    provision_boot_keys(*cmep_bus_, staged, why);
                }
                std::copy(staged.begin(), staged.end(), shared_sram_.begin());
                second_loader_pa_ = board::kSharedSramBase;'''
    if old2 in t:
        t = t.replace(old2, new2, 1)
    open(c, 'w', encoding='utf-8').write(t)
    print('patched bootchain.cpp')

# 3) CLI flag --no-provision.
cl = r'.\src\debug\cli.cpp'
u = open(cl, encoding='utf-8').read()
if '--no-provision' not in u:
    u = u.replace('''        else if (arg == "--no-rebuild") config.rebuild_emmc = false;''',
                  '''        else if (arg == "--no-rebuild") config.rebuild_emmc = false;
        else if (arg == "--no-provision") config.provision_keys = false;''', 1)
    u = u.replace('''        "  --no-rebuild           never build the eMMC image\\n"''',
                  '''        "  --no-rebuild           never build the eMMC image\\n"
        "  --no-provision         leave the emulated CMeP key table empty (the first\\n"
        "                         loader's RSA check then fails, as on an unprovisioned\\n"
        "                         console - see docs/KBL.md)\\n"''', 1)
    open(cl, 'w', encoding='utf-8').write(u)
    print('patched cli.cpp')
else:
    print('cli.cpp already patched')
