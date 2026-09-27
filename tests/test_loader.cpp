// zeliboba - loader unit tests (PUP / SLB2 / SELF / ELF / keys / NID / crypto).
//
// The parser tests use synthetic buffers so they always run; the tests that need
// real firmware are guarded by file_exists() and simply return when the data is
// not next to the build (the workspace path comes from the ZLB_WORKSPACE_DIR
// define of the CMake build, falling back to "..").
#include "test_framework.h"

#include <algorithm>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "bus/bus.h"
#include "common/util.h"
#include "loader/keys.h"
#include "loader/loader.h"
#include "loader/loader_extra.h"
#include "loader/nid.h"

using namespace zlb;

namespace {

#ifndef ZLB_WORKSPACE_DIR
#define ZLB_WORKSPACE_DIR ".."
#endif

std::string workspace(const std::string& relative) {
    return std::string(ZLB_WORKSPACE_DIR) + "/" + relative;
}

std::vector<u8> hex_to_bytes(const std::string& text) {
    auto digit = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    std::string clean;
    for (char c : text) {
        if (digit(c) >= 0) clean.push_back(c);
    }
    if ((clean.size() & 1u) != 0) clean.insert(clean.begin(), '0');
    std::vector<u8> out;
    for (size_t i = 0; i + 1 < clean.size(); i += 2)
        out.push_back(static_cast<u8>((digit(clean[i]) << 4) | digit(clean[i + 1])));
    return out;
}

std::string hex_string(const std::vector<u8>& data) { return ld::hex_bytes(data.data(), data.size()); }

void expect_text(const std::string& got, const std::string& want) {
    if (got != want) {
        ::zlb::test::report_failure(__FILE__, __LINE__,
                                    "expected '" + got + "' == '" + want + "'");
    }
}

void put_u16(std::vector<u8>& data, size_t offset, u16 value) {
    data[offset + 0] = static_cast<u8>(value);
    data[offset + 1] = static_cast<u8>(value >> 8);
}

void put_u32(std::vector<u8>& data, size_t offset, u32 value) {
    data[offset + 0] = static_cast<u8>(value);
    data[offset + 1] = static_cast<u8>(value >> 8);
    data[offset + 2] = static_cast<u8>(value >> 16);
    data[offset + 3] = static_cast<u8>(value >> 24);
}

void put_u64(std::vector<u8>& data, size_t offset, u64 value) {
    put_u32(data, offset, static_cast<u32>(value));
    put_u32(data, offset + 4, static_cast<u32>(value >> 32));
}

/// A minimal but valid ELF32 ARM module image.
///
///   e_entry 0x80 is the SceModuleInfo offset from segment 0, so the structure
///   lives at file offset 0x100 + 0x80 = 0x180 (segment 0 is at 0x100 with
///   vaddr 0x81000000 and covers [0x100, 0x200)).
///   The export table window [0x04, 0x24) is one library entry at 0x104; its
///   name, NID table and entry table point into segment 0 as well.
std::vector<u8> make_module_elf() {
    std::vector<u8> elf(0x200, 0);
    elf[0] = 0x7F;
    elf[1] = 'E';
    elf[2] = 'L';
    elf[3] = 'F';
    elf[4] = 1;                  // ELF32
    elf[5] = 1;                  // little endian
    elf[6] = 1;                  // EV_CURRENT
    put_u16(elf, 0x10, 0xFE04);  // e_type: PS Vita module
    put_u16(elf, 0x12, 0x28);    // e_machine: ARM
    put_u32(elf, 0x14, 1);       // e_version
    put_u32(elf, 0x18, 0x80);    // e_entry: SceModuleInfo offset from segment 0
    put_u32(elf, 0x1C, 0x34);    // e_phoff
    put_u16(elf, 0x28, 0x34);    // e_ehsize
    put_u16(elf, 0x2A, 32);      // e_phentsize
    put_u16(elf, 0x2C, 1);       // e_phnum
    put_u32(elf, 0x34, 1);       // phdr[0].p_type = PT_LOAD
    put_u32(elf, 0x38, 0x100);   // p_offset
    put_u32(elf, 0x3C, 0x81000000);   // p_vaddr
    put_u32(elf, 0x44, 0x100);   // p_filesz
    put_u32(elf, 0x48, 0x100);   // p_memsz
    put_u32(elf, 0x4C, 5);       // p_flags = RX

    const size_t module = 0x180;
    put_u16(elf, module + 0x00, 0x0007);   // modattribute
    const char* name = "SceTestModule";
    std::memcpy(elf.data() + module + 4, name, std::strlen(name));
    elf[module + 0x1F] = 0x06;             // module type
    put_u32(elf, module + 0x24, 0x04);     // export_top (segment relative)
    put_u32(elf, module + 0x28, 0x24);     // export_end
    put_u32(elf, module + 0x2C, 0x24);     // import_top
    put_u32(elf, module + 0x30, 0x24);     // import_end
    put_u32(elf, module + 0x34, 0x11223344);   // module_nid

    // One export library at segment relative 0x04 -> file offset 0x104.
    const size_t library = 0x104;
    elf[library + 0x00] = 0x20;            // size
    put_u16(elf, library + 0x02, 0x0101);  // version
    put_u16(elf, library + 0x06, 2);       // num_functions
    put_u32(elf, library + 0x10, 0xDEADBEEFu);          // lib_nid
    put_u32(elf, library + 0x14, 0x81000030);           // lib_name (0x130)
    put_u32(elf, library + 0x18, 0x81000040);           // nid_table (0x140)
    put_u32(elf, library + 0x1C, 0x81000048);           // entry_table (0x148)

    const char* library_name = "SceTestLib";
    std::memcpy(elf.data() + 0x130, library_name, std::strlen(library_name));
    put_u32(elf, 0x140, 0xD0595CE4);   // sceAVConfigChangeReg
    put_u32(elf, 0x144, 0x11111111);
    put_u32(elf, 0x148, 0x81000010);
    put_u32(elf, 0x14C, 0x81000020);
    return elf;
}

std::vector<u8> make_mep_header(u32 size, u32 offset, u32 length, u32 field_0x10, u16 field_0x16) {
    std::vector<u8> data(0x400, 0);
    put_u32(data, 0x00, 0x64B2C8E5);
    put_u32(data, 0x04, size);
    put_u32(data, 0x08, offset);
    put_u32(data, 0x0C, length);
    put_u32(data, 0x10, field_0x10);
    put_u16(data, 0x16, field_0x16);
    return data;
}

bool firmware_present() {
    return file_exists(workspace("Vita_104_Firmware/PSP2UPDAT104.PUP"));
}

}  // namespace

// ---------------------------------------------------------------------------
// Crypto
// ---------------------------------------------------------------------------

ZLB_TEST(loader_aes128_ecb_fips197) {
    const std::vector<u8> key = hex_to_bytes("000102030405060708090a0b0c0d0e0f");
    const std::vector<u8> plain = hex_to_bytes("00112233445566778899aabbccddeeff");
    u8 cipher[16];
    aes128_ecb_encrypt_block(key.data(), plain.data(), cipher);
    expect_text(hex_string(std::vector<u8>(cipher, cipher + 16)),
                  "69c4e0d86a7b0430d8cdb78070b4c55a");
    u8 back[16];
    aes128_ecb_decrypt_block(key.data(), cipher, back);
    ZLB_EXPECT_TRUE(std::memcmp(back, plain.data(), 16) == 0);
}

ZLB_TEST(loader_aes256_ecb_fips197) {
    const std::vector<u8> key =
        hex_to_bytes("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f");
    const std::vector<u8> cipher = hex_to_bytes("8ea2b7ca516745bfeafc49904b496089");
    u8 plain[16];
    aes256_ecb_decrypt_block(key.data(), cipher.data(), plain);
    expect_text(hex_string(std::vector<u8>(plain, plain + 16)),
                  "00112233445566778899aabbccddeeff");
}

ZLB_TEST(loader_aes_cbc_nist) {
    const std::vector<u8> iv = hex_to_bytes("000102030405060708090a0b0c0d0e0f");
    const std::string plain_hex =
        "6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51"
        "30c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710";

    // SP 800-38A F.2.1/F.2.2 (CBC-AES128)
    const std::string cipher128_hex =
        "7649abac8119b246cee98e9b12e9197d5086cb9b507219ee95db113a917678b2"
        "73bed6b8e3c1743b7116e69e222295163ff1caa1681fac09120eca307586e1a7";
    const std::vector<u8> cipher128 = hex_to_bytes(cipher128_hex);
    const std::vector<u8> key128 = hex_to_bytes("2b7e151628aed2a6abf7158809cf4f3c");
    std::vector<u8> out128(cipher128.size());
    aes128_cbc_decrypt(key128.data(), iv.data(), cipher128.data(), cipher128.size(), out128.data());
    expect_text(hex_string(out128), plain_hex);

    // SP 800-38A F.2.5/F.2.6 (CBC-AES256)
    const std::string cipher256_hex =
        "f58c4c04d6e5f1ba779eabfb5f7bfbd69cfc4e967edb808d679f777bc6702c7d"
        "39f23369a9d9bacfa530e26304231461b2eb05e2c39be9fcda6c19078c6a9d1b";
    const std::vector<u8> cipher256 = hex_to_bytes(cipher256_hex);
    const std::vector<u8> key256 =
        hex_to_bytes("603deb1015ca71be2b73aef0857d77811f352c073b6108d72d9810a30914dff4");
    std::vector<u8> out256(cipher256.size());
    aes256_cbc_decrypt(key256.data(), iv.data(), cipher256.data(), cipher256.size(), out256.data());
    expect_text(hex_string(out256), plain_hex);

    // The generic entry point picks AES-256 for a 32 byte key (the SCE metadata
    // keys are 32 bytes).
    std::vector<u8> out_any(cipher256.size());
    aes_cbc_decrypt_any(key256, iv, cipher256.data(), cipher256.size(), out_any.data());
    expect_text(hex_string(out_any), plain_hex);
}

ZLB_TEST(loader_sha1_vectors) {
    u8 digest[20];
    const std::string abc = "abc";
    sha1(reinterpret_cast<const u8*>(abc.data()), abc.size(), digest);
    expect_text(hex_string(std::vector<u8>(digest, digest + 20)),
                  "a9993e364706816aba3e25717850c26c9cd0d89d");
    sha1(nullptr, 0, digest);
    expect_text(hex_string(std::vector<u8>(digest, digest + 20)),
                  "da39a3ee5e6b4b0d3255bfef95601890afd80709");
}

ZLB_TEST(loader_sha256_vectors) {
    u8 digest[32];
    const std::string abc = "abc";
    sha256(reinterpret_cast<const u8*>(abc.data()), abc.size(), digest);
    expect_text(hex_string(std::vector<u8>(digest, digest + 32)),
                  "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    sha256(nullptr, 0, digest);
    expect_text(hex_string(std::vector<u8>(digest, digest + 32)),
                  "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

ZLB_TEST(loader_hmac_sha256_rfc4231) {
    u8 mac[32];
    std::vector<u8> key(20, 0x0B);
    const std::string data = "Hi There";
    hmac_sha256(key.data(), key.size(), reinterpret_cast<const u8*>(data.data()), data.size(), mac);
    expect_text(hex_string(std::vector<u8>(mac, mac + 32)),
                  "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7");
    const std::string jefe = "Jefe";
    const std::string message = "what do ya want for nothing?";
    hmac_sha256(reinterpret_cast<const u8*>(jefe.data()), jefe.size(),
                reinterpret_cast<const u8*>(message.data()), message.size(), mac);
    expect_text(hex_string(std::vector<u8>(mac, mac + 32)),
                  "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843");
}

ZLB_TEST(loader_rsa_modexp) {
    // 512 bit key pair generated with Python (two random 256 bit primes,
    // Miller-Rabin checked, e = 65537); the expected values are pow(m, e, n) and
    // pow(c, d, n).
    const std::vector<u8> n = hex_to_bytes(
        "961eb898ae980ebdd9c3f2c9048f293c82f32cc23edca11f93ee30d131f97dbd"
        "ecbe29c5a4a46675f32eedf25d73725b1919e2a95ab4ed1056be79e7fa02e489");
    const std::vector<u8> d = hex_to_bytes(
        "8ba3f40e828a0b5c6a4ead6e73813fd87beefbf98c43ead4d6e44020b81ddde9"
        "fa135b44b4540bf3b183fabf94d73f916917904a97e4093c61f5633b5f6b8c01");
    const std::vector<u8> m = hex_to_bytes(
        "0102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20"
        "0102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20");
    const std::vector<u8> c = hex_to_bytes(
        "94e7e65063887d07f7ce51e9e4efa4c046e195cb629ce711df0c93a2fd40a9b4"
        "e75333a4aa568ffb1b557119109fccd6f55d89a2bebec6e1467812ba3d99212e");
    expect_text(hex_string(rsa_public(n, hex_to_bytes("10001"), m)), hex_string(c));
    expect_text(hex_string(rsa_public(n, d, c)), hex_string(m));

    // The exponent 1 case also proves that input >= modulus is reduced first:
    // (n + 1)^1 mod n == 1.
    std::vector<u8> n_plus_one = n;
    for (size_t i = n_plus_one.size(); i-- > 0;) {
        if (++n_plus_one[i] != 0) break;
    }
    const std::vector<u8> one = rsa_public(n, std::vector<u8>{1}, n_plus_one);
    ZLB_EXPECT_EQ(one.size(), n.size());
    ZLB_EXPECT_EQ(one.back(), 1);
}

ZLB_TEST(loader_inflate) {
    // zlib.compress(b"zeliboba loader deflate test: " + bytes(range(256)) * 8, 9)
    const std::vector<u8> compressed = hex_to_bytes(
        "78daab4acdc94cca4f4a54c8c94f4c492d5248494dcb492c495528492d2eb1526060646266616563e7e0e4"
        "e2e6e1e5e3171014121611151397909492969195935750545256515553d7d0d4d2d6d1d5d33730343236"
        "313533b7b0b4b2b6b1b5b37770747276717573f7f0f4f2f6f1f5f30f080c0a0e090d0b8f888c8a8e898d"
        "8b4f484c4a06da989e9199959d939b975f5058545c525a565e5159555d535b57dfd0d8d4dcd2dad6ded1"
        "d9d5ddd3dbd73f61e2a4c953a64e9b3e63e6acd973e6ce9bbf60e1a2c54b962e5bbe62e5aad56bd6ae5b"
        "bf61e3a6cd5bb66edbbe63e7aedd7bf6eedb7fe0e0a1c3478e1e3b7ee2e4a9d367ce9e3b7fe1e2a5cb57"
        "ae5ebb7ee3e6addb77eedebbffe0e1a3c74f9e3e7bfee2e5abd76fdebe7bffe1e3a7cf5fbe7efbfee3e7"
        "afdf7ffefefb3feaff51ff8ffa7fd4ffa3fe1ff5ff48f43f00686f074b");
    std::vector<u8> expected;
    const std::string prefix = "zeliboba loader deflate test: ";
    expected.insert(expected.end(), prefix.begin(), prefix.end());
    for (int repeat = 0; repeat < 8; ++repeat) {
        for (int i = 0; i < 256; ++i) expected.push_back(static_cast<u8>(i));
    }
    std::string mode;
    auto out = inflate_zlib_or_raw(compressed.data(), compressed.size(), &mode);
    ZLB_EXPECT_TRUE(out.has_value());
    if (out) {
        ZLB_EXPECT_EQ(out->size(), expected.size());
        ZLB_EXPECT_TRUE(*out == expected);
        ZLB_EXPECT_TRUE(mode.find("zlib") != std::string::npos);
    }

    // 78 01 | BFINAL=1, BTYPE=00 | LEN=5, NLEN=0xFFFA | "hello"
    const std::vector<u8> stored = hex_to_bytes("7801010500faff68656c6c6f");
    auto out2 = inflate_zlib_or_raw(stored.data(), stored.size());
    ZLB_EXPECT_TRUE(out2.has_value());
    if (out2) expect_text(hex_string(*out2), "68656c6c6f");

    const std::vector<u8> garbage(32, 0x5A);
    ZLB_EXPECT_FALSE(inflate_zlib_or_raw(garbage.data(), garbage.size()).has_value());
}

// ---------------------------------------------------------------------------
// Image sniffing / parsers
// ---------------------------------------------------------------------------

ZLB_TEST(loader_is_sniffers) {
    const std::vector<u8> elf = make_module_elf();
    ZLB_EXPECT_TRUE(is_elf(elf));
    ZLB_EXPECT_FALSE(is_self(elf));
    ZLB_EXPECT_FALSE(is_slb2(elf));
    ZLB_EXPECT_FALSE(is_pup(elf));
    ZLB_EXPECT_FALSE(is_mep_image(elf));

    std::vector<u8> self(32, 0);
    put_u32(self, 0, 0x00454353);
    ZLB_EXPECT_TRUE(is_self(self));
    ZLB_EXPECT_FALSE(is_elf(self));

    std::vector<u8> slb2(0x200, 0xFF);
    put_u32(slb2, 0, 0x32424C53);
    ZLB_EXPECT_TRUE(is_slb2(slb2));

    std::vector<u8> pup(0x100, 0);
    std::memcpy(pup.data(), "SCEUF", 5);
    ZLB_EXPECT_TRUE(is_pup(pup));
    std::memcpy(pup.data(), "UPUP", 4);
    ZLB_EXPECT_TRUE(is_pup(pup));

    const std::vector<u8> enp = make_mep_header(0x2C0, 0x10, 0, 0x1000, 0);
    ZLB_EXPECT_TRUE(is_mep_image(enp));
}

ZLB_TEST(loader_identify) {
    const std::vector<u8> elf = make_module_elf();
    const ImageInfo info = identify(elf, "test.elf");
    ZLB_EXPECT_TRUE(info.kind == ImageKind::Elf);
    ZLB_EXPECT_TRUE(info.arch == Arch::Arm);
    ZLB_EXPECT_EQ(info.load_address, 0x81000000u);
    ZLB_EXPECT_EQ(info.entry_point, 0x81000080u);   // 0x81000000 + module offset 0x80
    expect_text(std::string(to_string(info.kind)), std::string("elf"));

    const ImageInfo raw =
        identify(std::vector<u8>{0x18, 0xF0, 0x9F, 0xE5, 0x00, 0x00, 0x00, 0x00}, "blob.bin");
    ZLB_EXPECT_TRUE(raw.kind == ImageKind::RawBinary);
    ZLB_EXPECT_TRUE(raw.arch == Arch::Arm);

    const ImageInfo mep = identify(std::vector<u8>(64, 0), "second_loader.bin");
    ZLB_EXPECT_TRUE(mep.kind == ImageKind::BootImage);
    ZLB_EXPECT_TRUE(mep.arch == Arch::MeP);
    ZLB_EXPECT_EQ(mep.load_address, kMepBootWindow);

    ZLB_EXPECT_TRUE(identify({}, "empty").kind == ImageKind::Unknown);
}

ZLB_TEST(loader_parse_elf) {
    const std::vector<u8> bytes = make_module_elf();
    auto elf = parse_elf(bytes);
    ZLB_EXPECT_TRUE(elf.has_value());
    if (!elf) return;
    ZLB_EXPECT_TRUE(elf->valid);
    ZLB_EXPECT_FALSE(elf->is64);
    ZLB_EXPECT_EQ(elf->machine, 0x28);
    ZLB_EXPECT_EQ(elf->type, 0xFE04);
    ZLB_EXPECT_EQ(elf->entry, 0x80u);
    ZLB_EXPECT_EQ(elf->phnum, 1);
    ZLB_EXPECT_EQ(elf->segments.size(), 1u);
    if (!elf->segments.empty()) {
        expect_text(elf->segments.front().name, std::string("load"));
        ZLB_EXPECT_EQ(elf->segments.front().vaddr, 0x81000000u);
        ZLB_EXPECT_EQ(elf->segments.front().filesz, 0x100u);
    }
    ZLB_EXPECT_EQ(elf_resolved_entry(*elf), 0x81000080u);
    ZLB_EXPECT_EQ(elf->module_info_offset(), 0x180);   // segment 0 offset 0x100 + 0x80

    std::vector<u8> truncated = bytes;
    truncated.resize(16);
    ZLB_EXPECT_FALSE(parse_elf(truncated).has_value());
    std::vector<u8> bad = bytes;
    bad[0] = 0;
    ZLB_EXPECT_FALSE(parse_elf(bad).has_value());
    ZLB_EXPECT_FALSE(parse_elf({}).has_value());
}

ZLB_TEST(loader_parse_module_info) {
    const std::vector<u8> bytes = make_module_elf();
    auto module = parse_module_info(bytes);
    ZLB_EXPECT_TRUE(module.has_value());
    if (!module) return;
    expect_text(module->name, std::string("SceTestModule"));
    ZLB_EXPECT_EQ(module->type, 0x06u);
    ZLB_EXPECT_EQ(module->module_nid, 0x11223344u);
    ZLB_EXPECT_EQ(module->exports_start, 0x04u);
    ZLB_EXPECT_EQ(module->exports_end, 0x24u);
    ZLB_EXPECT_EQ(module->imports.size(), 0u);
    ZLB_EXPECT_EQ(module->exports.size(), 2u);
    if (module->exports.size() == 2) {
        ZLB_EXPECT_EQ(module->exports[0].nid, 0xD0595CE4u);
        ZLB_EXPECT_EQ(module->exports[0].address, 0x81000010u);
        ZLB_EXPECT_EQ(module->exports[0].library, 0);
        ZLB_EXPECT_EQ(module->exports[1].nid, 0x11111111u);
        ZLB_EXPECT_EQ(module->exports[1].address, 0x81000020u);
    }
    ZLB_EXPECT_EQ(module_info_file_offset(bytes), 0x180u);

    const auto libraries = parse_module_libraries(bytes, nullptr);
    ZLB_EXPECT_EQ(libraries.size(), 1u);
    if (!libraries.empty()) {
        expect_text(libraries[0].name, std::string("SceTestLib"));
        ZLB_EXPECT_EQ(libraries[0].nid, 0xDEADBEEFu);
        ZLB_EXPECT_TRUE(libraries[0].is_export);
        ZLB_EXPECT_FALSE(libraries[0].is_export ? false : true);
        ZLB_EXPECT_EQ(libraries[0].functions.size(), 2u);
    }
}

ZLB_TEST(loader_module_info_section_path) {
    // An ELF whose e_entry is a real virtual address (so the "module info
    // offset" rule does not apply) but which has a section header table naming
    // .sceModuleInfo.rodata: parse_module_info must use the section.
    std::vector<u8> elf(0x300, 0);
    elf[0] = 0x7F;
    elf[1] = 'E';
    elf[2] = 'L';
    elf[3] = 'F';
    elf[4] = 1;
    elf[5] = 1;
    elf[6] = 1;
    put_u16(elf, 0x10, 0xFE00);        // e_type: module (no e_entry convention here)
    put_u16(elf, 0x12, 0x28);          // ARM
    put_u32(elf, 0x14, 1);
    put_u32(elf, 0x18, 0x81000040);    // e_entry: a virtual address inside PT_LOAD
    put_u32(elf, 0x1C, 0x34);          // e_phoff
    put_u32(elf, 0x20, 0x200);         // e_shoff
    put_u16(elf, 0x2A, 32);            // e_phentsize
    put_u16(elf, 0x2C, 1);             // e_phnum
    put_u16(elf, 0x2E, 40);            // e_shentsize
    put_u16(elf, 0x30, 3);             // e_shnum
    put_u16(elf, 0x32, 2);             // e_shstrndx
    put_u32(elf, 0x34, 1);             // PT_LOAD
    put_u32(elf, 0x38, 0x100);         // p_offset
    put_u32(elf, 0x3C, 0x81000000);    // p_vaddr
    put_u32(elf, 0x44, 0x100);         // p_filesz
    put_u32(elf, 0x48, 0x100);         // p_memsz

    // SceModuleInfo at 0x180 (inside the PT_LOAD), no export/import tables.
    const size_t module = 0x180;
    put_u16(elf, module + 0x00, 0x0007);
    const char* name = "SceSectionModule";
    std::memcpy(elf.data() + module + 4, name, std::strlen(name));
    elf[module + 0x1F] = 0x07;
    put_u32(elf, module + 0x34, 0xAABBCCDD);

    // Section header table: [0] NULL, [1] .sceModuleInfo.rodata, [2] .shstrtab.
    const char* names = "\0.sceModuleInfo.rodata\0.shstrtab\0";
    const size_t names_size = 29;
    std::memcpy(elf.data() + 0x280, names, names_size);
    put_u32(elf, 0x200 + 1 * 40 + 0, 1);            // sh_name -> ".sceModuleInfo.rodata"
    put_u32(elf, 0x200 + 1 * 40 + 4, 1);            // sh_type = PROGBITS
    put_u32(elf, 0x200 + 1 * 40 + 8, 2);            // sh_flags = ALLOC
    put_u32(elf, 0x200 + 1 * 40 + 12, 0x81000080);  // sh_addr
    put_u32(elf, 0x200 + 1 * 40 + 16, 0x180);       // sh_offset
    put_u32(elf, 0x200 + 1 * 40 + 20, 0x34);        // sh_size
    put_u32(elf, 0x200 + 2 * 40 + 0, 23);           // sh_name -> ".shstrtab"
    put_u32(elf, 0x200 + 2 * 40 + 4, 3);            // sh_type = STRTAB
    put_u32(elf, 0x200 + 2 * 40 + 16, 0x280);       // sh_offset
    put_u32(elf, 0x200 + 2 * 40 + 20, names_size);  // sh_size

    auto image = parse_elf(elf);
    ZLB_EXPECT_TRUE(image.has_value());
    if (!image) return;
    ZLB_EXPECT_EQ(elf_resolved_entry(*image), 0x81000040u);   // a real VA
    ZLB_EXPECT_EQ(image->module_info_offset(), -1);            // so no module offset rule
    ZLB_EXPECT_TRUE(elf_section_headers_sane(elf));
    auto section = elf_section_offset(elf, ".sceModuleInfo.rodata");
    ZLB_EXPECT_TRUE(section.has_value());
    if (section) ZLB_EXPECT_EQ(*section, 0x180u);

    auto module_info = parse_module_info(elf);
    ZLB_EXPECT_TRUE(module_info.has_value());
    if (module_info) {
        expect_text(module_info->name, std::string("SceSectionModule"));
        ZLB_EXPECT_EQ(module_info->type, 0x07u);
        ZLB_EXPECT_EQ(module_info->module_nid, 0xAABBCCDDu);
        ZLB_EXPECT_EQ(module_info->exports.size(), 0u);
        ZLB_EXPECT_EQ(module_info->imports.size(), 0u);
    }
    ZLB_EXPECT_EQ(module_info_file_offset(elf), 0x180u);

    // A garbage section header table is rejected by the sanity check (it is what
    // makes self_to_elf clear e_shoff for a retail SELF).
    std::vector<u8> garbage = elf;
    put_u32(garbage, 0x200 + 1 * 40 + 4, 0x12345678);   // an impossible sh_type
    ZLB_EXPECT_FALSE(elf_section_headers_sane(garbage));
    ZLB_EXPECT_FALSE(elf_section_offset(garbage, ".sceModuleInfo.rodata").has_value());
}

ZLB_TEST(loader_mep_header_checks) {
    MepImageHeader header;
    ZLB_EXPECT_TRUE(parse_mep_header(make_mep_header(0x2C0, 0x10, 0, 0x1000, 0), header));
    ZLB_EXPECT_TRUE(header.valid);
    ZLB_EXPECT_EQ(header.magic, 0x64B2C8E5u);
    ZLB_EXPECT_EQ(header.size, 0x2C0u);
    ZLB_EXPECT_EQ(header.offset, 0x10u);
    ZLB_EXPECT_EQ(header.length, 0u);
    ZLB_EXPECT_EQ(header.field_0x10, 0x1000u);
    ZLB_EXPECT_TRUE(std::string(describe_mep_header(header)).find("0x64B2C8E5") != std::string::npos);

    // The five validation rules of dumps/bootrom_analysis/ANALYSIS.md 4.3.
    ZLB_EXPECT_FALSE(parse_mep_header(make_mep_header(0x2AF, 0, 0, 0x1000, 0), header));
    ZLB_EXPECT_FALSE(parse_mep_header(make_mep_header(0x10000, 0, 0, 0x1000, 0), header));
    ZLB_EXPECT_FALSE(parse_mep_header(make_mep_header(0x2C0, 0x20, 0, 0x1000, 0), header));
    ZLB_EXPECT_FALSE(parse_mep_header(make_mep_header(0x2C0, 0x10, 0, 0x1C000, 0), header));
    ZLB_EXPECT_FALSE(parse_mep_header(make_mep_header(0x2C0, 0x10, 0, 0x1000, 0x10), header));
    std::vector<u8> bad_magic = make_mep_header(0x2C0, 0x10, 0, 0x1000, 0);
    put_u32(bad_magic, 0, 0x12345678);
    ZLB_EXPECT_FALSE(parse_mep_header(bad_magic, header));
    // offset + length + 0x2B0 == size is fine for a non-zero length too.
    ZLB_EXPECT_TRUE(parse_mep_header(make_mep_header(0x300, 0x10, 0x40, 0x1000, 0), header));
    ZLB_EXPECT_EQ(header.length, 0x40u);

    // The payload starts at `size` and is field_0x10 bytes long.
    std::vector<u8> image = make_mep_header(0x2C0, 0x10, 0, 0x10, 0);
    for (size_t i = 0; i < 0x10; ++i) image[0x2C0 + i] = static_cast<u8>(i + 1);
    MepImageHeader parsed;
    ZLB_EXPECT_TRUE(parse_mep_header(image, parsed));
    auto payload = mep_payload(image, parsed);
    ZLB_EXPECT_TRUE(payload.has_value());
    if (payload) {
        ZLB_EXPECT_EQ(payload->size(), 0x10u);
        ZLB_EXPECT_EQ((*payload)[0], 1);
        ZLB_EXPECT_EQ((*payload)[15], 16);
    }
}

ZLB_TEST(loader_slb2_round_trip) {
    std::vector<Slb2Entry> entries;
    for (int i = 0; i < 3; ++i) {
        Slb2Entry entry;
        entry.name = format("entry%d.bin", i);
        entry.size = static_cast<u32>(0x200 + i * 0x180);
        entry.data.resize(entry.size);
        for (size_t b = 0; b < entry.data.size(); ++b)
            entry.data[b] = static_cast<u8>(b + i);
        entries.push_back(std::move(entry));
    }
    const std::vector<u8> container = build_slb2(entries);
    ZLB_EXPECT_TRUE(is_slb2(container));
    ZLB_EXPECT_EQ(ld::read_u32(container, 4), 1u);       // version
    ZLB_EXPECT_EQ(ld::read_u32(container, 8), 0x200u);   // header size
    ZLB_EXPECT_EQ(ld::read_u32(container, 12), 3u);      // entry count
    ZLB_EXPECT_EQ(container[0x1F], 0u);                  // header padding is zero
    ZLB_EXPECT_EQ(container[0x1FF], 0xFFu);              // unused header tail is 0xFF

    auto parsed = parse_slb2(container);
    ZLB_EXPECT_TRUE(parsed.has_value());
    if (!parsed) return;
    ZLB_EXPECT_EQ(parsed->version, 1u);
    ZLB_EXPECT_EQ(parsed->entries.size(), 3u);
    u32 expected_block = 1;
    for (size_t i = 0; i < parsed->entries.size() && i < entries.size(); ++i) {
        expect_text(parsed->entries[i].name, entries[i].name);
        ZLB_EXPECT_EQ(parsed->entries[i].offset, expected_block * 512u);
        ZLB_EXPECT_EQ(parsed->entries[i].size, entries[i].size);
        ZLB_EXPECT_TRUE(parsed->entries[i].data == entries[i].data);
        expected_block += static_cast<u32>((entries[i].size + 511) / 512);
    }

    // A bogus entry count is clamped to what fits instead of faulting.
    std::vector<u8> bogus(0x40, 0);
    put_u32(bogus, 0, 0x32424C53);
    put_u32(bogus, 0x0C, 1000);
    auto clamped = parse_slb2(bogus);
    ZLB_EXPECT_TRUE(clamped.has_value());
    if (clamped) ZLB_EXPECT_EQ(clamped->entries.size(), 0u);
}

ZLB_TEST(loader_parse_pup) {
    // A synthetic PUP with the 1.04 header layout and one version.txt record.
    std::vector<u8> pup(0x100, 0);
    std::memcpy(pup.data(), "SCEUF", 5);
    pup[7] = 1;
    put_u32(pup, 0x08, 2);           // version
    put_u32(pup, 0x10, 0x01040000);  // firmware version
    put_u32(pup, 0x14, 230019);      // build number
    put_u32(pup, 0x18, 1);           // entry count
    put_u64(pup, 0x80, 0x100);       // file type: version.txt
    put_u64(pup, 0x88, 0xC0);        // offset
    put_u64(pup, 0x90, 7);           // size
    put_u64(pup, 0x98, 2);           // flags
    const char* text = "1.040.0";
    std::memcpy(pup.data() + 0xC0, text, 7);
    ZLB_EXPECT_TRUE(is_pup(pup));

    auto image = parse_pup(pup);
    ZLB_EXPECT_TRUE(image.has_value());
    if (!image) return;
    ZLB_EXPECT_EQ(image->magic, 0x55454353u);
    ZLB_EXPECT_EQ(image->version, 2u);
    ZLB_EXPECT_EQ(image->firmware_version, 0x01040000ull);
    ZLB_EXPECT_EQ(image->entries.size(), 1u);
    if (image->entries.size() == 1) {
        expect_text(image->entries[0].name, std::string("version.txt"));
        ZLB_EXPECT_EQ(image->entries[0].offset, 0xC0u);
        ZLB_EXPECT_EQ(image->entries[0].size, 7u);
        ZLB_EXPECT_EQ(image->entries[0].flags, 2u);
        expect_text(hex_string(image->entries[0].data), "312e3034302e30");
    }
    ZLB_EXPECT_FALSE(parse_pup(std::vector<u8>(64, 0)).has_value());
}

// ---------------------------------------------------------------------------
// SELF
// ---------------------------------------------------------------------------

ZLB_TEST(loader_self_header_and_failure_report) {
    // A synthetic SCE header with nothing behind it: the parser must fail
    // gracefully and name the stage instead of faulting.
    std::vector<u8> data(32, 0);
    put_u32(data, 0, 0x00454353);
    put_u32(data, 4, 3);
    data[8] = 0x40;                  // VITA platform
    data[9] = 1;                     // key revision
    put_u16(data, 10, 1);            // SELF
    put_u32(data, 12, 0x600);        // metadata offset
    put_u64(data, 16, 0x1000);       // header length
    ZLB_EXPECT_TRUE(is_self(data));
    ZLB_EXPECT_FALSE(is_elf(data));

    SelfHeader header;
    ZLB_EXPECT_FALSE(parse_self_header(data, header));

    SelfDecryptReport report;
    ZLB_EXPECT_FALSE(self_to_elf_report(data, SceKeys::default_keys(), {}, report).has_value());
    expect_text(report.stage, std::string("self header"));
    ZLB_EXPECT_TRUE(report.message.find("88 byte") != std::string::npos);

    std::vector<u8> not_sce(64, 0);
    ZLB_EXPECT_FALSE(parse_self_header(not_sce, header));
    ZLB_EXPECT_FALSE(self_to_elf(not_sce, SceKeys::default_keys()).has_value());

    // An SRVK/SPKG container is not a SELF: the pipeline must say so.
    std::vector<u8> srvk = data;
    put_u16(srvk, 10, 2);
    ZLB_EXPECT_FALSE(self_to_elf_report(srvk, SceKeys::default_keys(), {}, report).has_value());
    ZLB_EXPECT_TRUE(report.message.find("not SELF") != std::string::npos);
}

// ---------------------------------------------------------------------------
// Key store / NID database
// ---------------------------------------------------------------------------

ZLB_TEST(loader_key_store_candidates) {
    const SceKeys& keys = SceKeys::default_keys();
    // kernel_boot_loader.self: SELF/BOOT keyrev 1, sys_version 0x10400000000.
    auto boot = sce_key_candidates(keys, static_cast<int>(SceKeyKind::Metadata),
                                   static_cast<int>(SceContainerKind::Self), false, 0x10400000000ll,
                                   1, static_cast<int>(SceSelfKind::Boot));
    ZLB_EXPECT_TRUE(!boot.empty());
    if (!boot.empty()) {
        expect_text(hex_string(boot.front().key),
                      "7a7fb1560dcd121cea5e11b90124b13282752f2d5b95d75036ab3a29bb3bd2ab");
        expect_text(hex_string(boot.front().iv), "6c71642a042a041f1ee3094070b009be");
        expect_text(boot.front().source, std::string("pup_fiction/keys.py:73"));
        ZLB_EXPECT_EQ(boot.front().key.size(), 32u);   // the SCE metadata keys are AES-256
    }

    // The system version window filters entries out (0x100000000000 is above
    // every registered max_version, 0xFFF00000000); ignore_sys_version does not.
    ZLB_EXPECT_TRUE(sce_key_candidates(keys, static_cast<int>(SceKeyKind::Metadata),
                                       static_cast<int>(SceContainerKind::Self), false,
                                       0x100000000000ll, 1, static_cast<int>(SceSelfKind::Boot))
                        .empty());
    ZLB_EXPECT_TRUE(!sce_key_candidates(keys, static_cast<int>(SceKeyKind::Metadata),
                                        static_cast<int>(SceContainerKind::Self), true,
                                        0x100000000000ll, 1, static_cast<int>(SceSelfKind::Boot))
                         .empty());

    // NPDRM keys are AES-128, and an unknown SELF type has no key at all.
    SceKeyCandidate candidate;
    ZLB_EXPECT_TRUE(sce_keys_lookup(keys, static_cast<int>(SceKeyKind::Npdrm),
                                    static_cast<int>(SceContainerKind::Self), -1, 0,
                                    static_cast<int>(SceSelfKind::App), candidate));
    ZLB_EXPECT_EQ(candidate.key.size(), 16u);
    ZLB_EXPECT_FALSE(sce_keys_lookup(keys, static_cast<int>(SceKeyKind::Metadata),
                                     static_cast<int>(SceContainerKind::Self), -1, 9,
                                     static_cast<int>(SceSelfKind::User), candidate));

    ZLB_EXPECT_EQ(keys.select_self_key(0x40, 1, static_cast<u32>(SceContainerKind::Self)).size(), 32u);
    ZLB_EXPECT_EQ(keys.select_self_key(0x40, 77, static_cast<u32>(SceContainerKind::Self)).size(), 0u);
    ZLB_EXPECT_TRUE(describe_key_table(keys).size() >= 49);
    ZLB_EXPECT_TRUE(keys.has("ENC_KEY"));
    expect_text(hex_string(keys.get("ENC_IV")), "af5f2cb04ac1751abf51cef1c8096210");
    ZLB_EXPECT_EQ(keys.get("no such key").size(), 0u);
}

ZLB_TEST(loader_nid_database) {
    if (find_nid_databases().empty()) return;
    auto database = load_nid_database(find_nid_databases().front());
    ZLB_EXPECT_TRUE(database.has_value());
    if (!database) return;
    ZLB_EXPECT_TRUE(database->size() > 8000);
    expect_text(database->lookup(0xD0595CE4), std::string("sceAVConfigChangeReg"));
    expect_text(database->lookup(0x00000000), std::string());
    ZLB_EXPECT_TRUE(database->has(0xD0595CE4));
    ZLB_EXPECT_FALSE(load_nid_database("no/such/db.yml").has_value());
}

// ---------------------------------------------------------------------------
// Real firmware (skipped when the workspace data is not next to the build)
// ---------------------------------------------------------------------------

ZLB_TEST(loader_real_pup) {
    if (!firmware_present()) return;
    auto data = read_file(workspace("Vita_104_Firmware/PSP2UPDAT104.PUP"));
    ZLB_EXPECT_TRUE(data.has_value());
    if (!data) return;
    auto pup = parse_pup(*data);
    ZLB_EXPECT_TRUE(pup.has_value());
    if (!pup) return;
    ZLB_EXPECT_EQ(pup->entries.size(), 19u);
    ZLB_EXPECT_EQ(pup->version, 2u);
    ZLB_EXPECT_EQ(pup->firmware_version, 0x01040000ull);
    if (pup->entries.size() == 19) {
        expect_text(pup->entries[0].name, std::string("version.txt"));
        ZLB_EXPECT_EQ(pup->entries[0].size, 7u);
        expect_text(pup->entries[4].name, std::string("boot_slb2-00.pkg"));
        ZLB_EXPECT_EQ(pup->entries[4].offset, 0x483E00u);
        ZLB_EXPECT_EQ(pup->entries[4].size, 648320u);
        expect_text(pup->entries[16].name, std::string("vs0-10.pkg"));
        ZLB_EXPECT_EQ(pup->entries[16].size, 3560576u);
        expect_text(pup->entries[18].name, std::string("package_sceas.as"));
        ZLB_EXPECT_EQ(pup->entries[18].size, 1024u);
    }
}

ZLB_TEST(loader_real_slb2) {
    const std::string path = workspace("Vita_104_Firmware/Out/PUP_dec/boot_slb2-00.pkg.seg02");
    if (!file_exists(path)) return;
    auto data = read_file(path);
    ZLB_EXPECT_TRUE(data.has_value());
    if (!data) return;
    auto slb2 = parse_slb2(*data);
    ZLB_EXPECT_TRUE(slb2.has_value());
    if (!slb2) return;
    ZLB_EXPECT_EQ(slb2->entries.size(), 7u);
    if (slb2->entries.size() == 7) {
        expect_text(slb2->entries[0].name, std::string("second_loader.enp"));
        ZLB_EXPECT_EQ(slb2->entries[0].offset, 0x200u);
        ZLB_EXPECT_EQ(slb2->entries[0].size, 93184u);
        expect_text(slb2->entries[4].name, std::string("kernel_boot_loader.self"));
        ZLB_EXPECT_EQ(slb2->entries[4].size, 355220u);
        expect_text(slb2->entries[6].name, std::string("prog_rvk.srvk"));
        ZLB_EXPECT_EQ(slb2->entries[6].size, 1728u);
        // The payloads start with the container magics, which is what fixes the
        // entry table layout.
        ZLB_EXPECT_EQ(ld::read_u32(slb2->entries[0].data, 0), 0x64B2C8E5u);
        ZLB_EXPECT_EQ(ld::read_u32(slb2->entries[4].data, 0), 0x00454353u);
    }

    // build_slb2() with the retail parameters reproduces the container exactly.
    const std::vector<u8> rebuilt = build_slb2(slb2->entries, 1, 0x2000, 0xFF);
    ZLB_EXPECT_EQ(rebuilt.size(), 0x9DC00u);
    ZLB_EXPECT_TRUE(std::memcmp(rebuilt.data(), data->data(), rebuilt.size()) == 0);
}

ZLB_TEST(loader_real_self_to_elf) {
    const std::string path = workspace("Vita_104_Firmware/Out/SLB2/kernel_boot_loader.self");
    if (!file_exists(path)) return;
    auto data = read_file(path);
    ZLB_EXPECT_TRUE(data.has_value());
    if (!data) return;

    const ImageInfo info = identify(*data, path);
    ZLB_EXPECT_TRUE(info.kind == ImageKind::SceSelf);
    ZLB_EXPECT_TRUE(info.arch == Arch::Arm);

    SelfHeader header;
    ZLB_EXPECT_TRUE(parse_self_header(*data, header));
    ZLB_EXPECT_EQ(header.key_revision, 1);
    ZLB_EXPECT_EQ(header.sce_type, 1);
    ZLB_EXPECT_EQ(header.self_type, 0x09);
    ZLB_EXPECT_EQ(header.sys_version, 0x10400000000ull);
    ZLB_EXPECT_EQ(header.header_len, 0x1000u);
    ZLB_EXPECT_EQ(header.elf_offset, 0xA0u);

    SelfDecryptReport report;
    auto elf = self_to_elf_report(*data, SceKeys::default_keys(), {}, report);
    ZLB_EXPECT_TRUE(elf.has_value());
    if (!elf) return;
    ZLB_EXPECT_TRUE(report.ok);
    expect_text(report.metadata_key, std::string("pup_fiction/keys.py:73"));
    ZLB_EXPECT_EQ(report.metadata_sections, 4);
    ZLB_EXPECT_EQ(report.metadata_keys, 24);
    ZLB_EXPECT_TRUE(report.section_headers_dropped);
    ZLB_EXPECT_EQ(elf->size(), 0x55B94u);
    ZLB_EXPECT_EQ((*elf)[0], 0x7F);
    ZLB_EXPECT_EQ((*elf)[3], 'F');
    auto parsed = parse_elf(*elf);
    ZLB_EXPECT_TRUE(parsed.has_value());
    if (parsed) {
        ZLB_EXPECT_EQ(parsed->machine, 0x28);
        ZLB_EXPECT_EQ(parsed->phnum, 5);
        ZLB_EXPECT_EQ(elf_resolved_entry(*parsed), 0x40020000u);
    }

    // Every decrypted segment matches the pup_fiction reference output.
    const auto segments = sce_decrypt_segments(*data, SceKeys::default_keys(), {}, nullptr);
    ZLB_EXPECT_EQ(segments.size(), 4u);
    const char* references[] = {
        "Vita_104_Firmware/Out/SLB2_dec/kernel_boot_loader.self.seg00",
        "Vita_104_Firmware/Out/SLB2_dec/kernel_boot_loader.self.seg01",
        "Vita_104_Firmware/Out/SLB2_dec/kernel_boot_loader.self.seg02",
        "Vita_104_Firmware/Out/SLB2_dec/kernel_boot_loader.self.seg03",
    };
    for (size_t i = 0; i < 4 && i < segments.size(); ++i) {
        auto expected = read_file(workspace(references[i]));
        ZLB_EXPECT_TRUE(expected.has_value());
        if (expected) ZLB_EXPECT_TRUE(segments[i].data == *expected);
    }

    // ... and the SELF -> ELF -> bus chain loads the image.
    Bus bus;
    const LoadResult result = load_image(bus, *data, path, SceKeys::default_keys());
    ZLB_EXPECT_TRUE(result.ok);
    ZLB_EXPECT_EQ(result.entry, 0x40020000u);
    ZLB_EXPECT_TRUE(!result.info.plain.empty());
    ZLB_EXPECT_TRUE(bus.is_ram(result.entry));
}

ZLB_TEST(loader_real_module_elf) {
    const std::string path = workspace("Vita_104_Firmware/Out/fs_dec/os0/kd/threadmgr.elf");
    if (!file_exists(path)) return;
    auto data = read_file(path);
    ZLB_EXPECT_TRUE(data.has_value());
    if (!data) return;
    auto module = parse_module_info(*data);
    ZLB_EXPECT_TRUE(module.has_value());
    if (!module) return;
    expect_text(module->name, std::string("SceKernelThreadMgr"));
    ZLB_EXPECT_EQ(module->module_nid, 0x4AC29B7Du);
    ZLB_EXPECT_EQ(module->exports.size(), 760u);
    ZLB_EXPECT_EQ(module->imports.size(), 137u);
    ZLB_EXPECT_EQ(module_info_file_offset(*data), 0x27B40u);
    if (!module->imports.empty()) {
        ZLB_EXPECT_EQ(module->imports.front().nid, 0x03499636u);
        expect_text(module->imports.front().name, std::string("ksceExcpmgrRegisterHandler"));
    }

    Bus bus;
    const LoadResult result = load_file(bus, path, SceKeys::default_keys());
    ZLB_EXPECT_TRUE(result.ok);
    ZLB_EXPECT_EQ(result.entry, 0x81027AA0u);
    ZLB_EXPECT_TRUE(result.module.has_value());
    ZLB_EXPECT_TRUE(bus.is_ram(result.entry));
}

ZLB_TEST(loader_real_cui_setupper_self) {
    // A second retail SELF (a user/app type container) through the same chain.
    const std::string path = workspace("Vita_104_Firmware/Out/PUP/cui_setupper.self");
    if (!file_exists(path)) return;
    auto data = read_file(path);
    if (!data) return;
    const ImageInfo info = identify(*data, path);
    ZLB_EXPECT_TRUE(info.kind == ImageKind::SceSelf);
    SelfDecryptReport report;
    auto elf = self_to_elf_report(*data, SceKeys::default_keys(), {}, report);
    ZLB_EXPECT_TRUE(elf.has_value());
    if (!elf) return;
    ZLB_EXPECT_TRUE((*elf)[0] == 0x7F && (*elf)[3] == 'F');
    auto parsed = parse_elf(*elf);
    ZLB_EXPECT_TRUE(parsed.has_value());
    if (parsed) ZLB_EXPECT_EQ(parsed->machine, 0x28);
    auto reference = read_file(workspace("Vita_104_Firmware/Out/PUP_dec/cui_setupper.elf"));
    if (reference) ZLB_EXPECT_TRUE(parsed.has_value());
}
