// zeliboba - Bigmac crypto engine (0xE0050000).
//
// ---------------------------------------------------------------------------
// What the first loader does with it
// ---------------------------------------------------------------------------
// bigmac_cmd (0x5CD20) is the generic issue routine:
//
//   5CD32  $1 -> +0x04        (cmd3)
//   5CD34  $2 -> +0x00        (the command word)
//   5CD36  $3 -> +0x08        (cmd4)
//   5CD42  $? -> +0x14        (key pointer; only written when non-zero, 0x5CD3A)
//   5CD5C  $4 -> +0x10        (flags; only when $4 < 0x1000)
//   5CD64  $4 bytes -> +0x200 (8 little-endian words when $4 >= 0x1000)
//   5CDAA  $5 -> +0x0C        (function)
//   5CDAC  1  -> +0x1C        (start)
//   5CDB6  poll +0x24 until bit 0 clears
//   5CDC8  return -1 when (+0x24 & 0x78000)
//
// The wrappers and their constants are the ground truth for the field layout:
//
//   keyring_write1 0x5CCA8  cmd=0x00000008, function=0x0301, cmd3=len,
//                           cmd4=dst, key=src, flags=$4   (0x5C69E: len=32,
//                           src=0x5E744, dst=32, flags=0x344)
//   keyring_write2 0x5CCC0  cmd=$1 (index), function=0x010A (0x5CCC6
//                           `mov $11,266`), cmd3=$2 (src), cmd4=$3 (dst),
//                           key=$4 (len), flags=stack[0]
//   keyring_read1  0x5CCD8  cmd=$1, function=0x030A (0x5CCDE `mov $11,778`)
//   keyring_read2  0x5CCF0  cmd=0, function=0x0033 (0x5CCF6 `mov $11,51`)
//   keyring_op     0x5CD06  cmd=$1, function=0x000C (0x5CD10 `mov $11,12`),
//                           cmd4=0, flags=0
//   bignum / rng are not Bigmac commands (see bignum.cpp).
//
// So the command word is `(flags << 16) | keyring_index` for transfers, and the
// function selects the engine.  bigmac_cmd's return value distinguishes success
// (0) from a signalled error (-1).
//
// ---------------------------------------------------------------------------
// RNG
// ---------------------------------------------------------------------------
// The hardware RNG is genuinely non-deterministic.  The emulator must be
// reproducible, so the model drives a 64-bit LCG/xorshift seeded from the fixed
// constant kRngSeed below.  Documented here and in the README-worthy comments of
// cmep_block.cpp: two runs with the same boot image produce the same values.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "common/log.h"
#include "hw/cmep/cmep_internal.h"
#include "loader/loader_extra.h"

namespace zlb {

namespace {

/// Temporary bring-up trace (`ZLB_BIGMAC_TRACE=1`): every command the CMeP
/// issues to the engine and every failure code, so a stage that silently bails
/// out of a keyed transform can be pinpointed.
bool bigmac_trace() {
    static const bool on = [] {
        const char* value = std::getenv("ZLB_BIGMAC_TRACE");
        return value != nullptr && value[0] != '0';
    }();
    return on;
}

/// The staged second loader is decrypted to 0x40000 (cmep::kRamBase).
constexpr u32 kSecondLoaderBase = 0x00040000;

/// Entries are `{offset, length, word, forced, what}` (see cmep_internal.h).
/// Every "clear" entry is applied before any "forced" one: 0x64A6 appears in
/// both, and the replacement instruction has to win over the cleared branch.
using cmep_detail::SecondLoaderPatch;
const SecondLoaderPatch kSecondLoaderPatches[] = {
    {0x63A0, 4, 0, false, "SMI outer signature check"},
        {0x63D4, 4, 0, false, "SMI signature post-check"},
        {0x6488, 2, 0, false, "SMI integrity check 1"},
        {0x64A6, 2, 0, false, "SMI integrity check 2"},
        // The per-console configuration record carries a MAC over its payload
        // (bytes 8..39) that the boot chain verifies with the same keyed Bigmac
        // family the SMI payload uses - keyring slot 0x213, which no dump in the
        // workspace has (see docs/KBL.md round 26).  The three branches after
        // 0x49272/0x48CDE turn a failed verification into the 0x800F0027 exit.
        {0x9122, 2, 0, false, "configuration record MAC check 1"},
        {0x912C, 2, 0, false, "configuration record MAC check 2"},
        {0x9136, 2, 0, false, "configuration record MAC check 3"},
        // The index-15 record goes through its own validator (0x429xx): it
        // compares the record's keyed digest (0x42E42) against the stored field
        // with 0x4C3AA and exits with 0x800F0027 on a mismatch.
        {0x292E, 2, 0, false, "configuration record 0x0F digest check"},
        // There used to be a ninth entry here, "ARM boot-context gate
        // (beqz $8,0x40858)", written with the *absolute* address 0x40850 while
        // every other entry is an offset from 0x40000.  The loop therefore
        // patched 0x80850 - unmapped memory (reads return 0xFFFFFFFF) - so the
        // substitution never had any effect, although it logged that it did
        // (docs/KBL.md round 91).  It also modelled the wrong thing: the branch
        // at 0x40850 is the *cold boot / resume* selector.  $8 is bit 7 of the
        // syscon wakeup factor (`lw $8,0x3c($sp)` at 0x40596, `srl $8,7`,
        // `and3 $8,$8,1`; command 0x0010 answers 0xFF14 on a cold boot and
        // 0xFF80 on a resume), the *taken* branch is the cold path (0x40858 ->
        // 0x4112E, which builds the context), and only the not-taken, resume-only
        // path (0x40852 -> 0x40A86) copies a saved context from DRAM to the
        // scratchpad.  Forcing it on a cold boot would have invented a resume.
        //
        // The entries below replace an instruction instead of clearing one.
        //
        // The SMI signature itself cannot be forged: the validator exponentiates
        // the leaf's signature with the SMI public key at 0x4D024 (e = 0x00010001)
        // and bignum_run (0x4BBCE) then insists on a valid PKCS#1 v1.5 block -
        // dest[0] = 0, dest[1] <= 1, ... (0x4BC1E..0x4BD22) - which our
        // zero-filled signature leaf can never produce, so the engine reports
        // 0x800F0024 and 0x4643C takes the error exit.  Replacing that one
        // conditional branch at 0x4643C (`beqz $0,0x4648C`, word 0xA050) with
        // `bra 0x4648C` (word 0xB050) makes the validation continue on the
        // "signature verified" path.
        //
        // The SMI validator's two keyed checks (0x4646E and 0x4648C) then open
        // keyring slot 0x213 - the per-console key that encrypts the SMI payload -
        // and run the second loader's keyed Bigmac family (function
        // 0x2309/0x238A/0x2093, bit 13 = "the flags field names the keyring
        // slot").  Without that key the operations fail with 0x800F0005 at
        // 0x45042 and the validator hands that status back to its caller even
        // though every branch that reads it is already substituted.  Installing
        // `mov $0,0; bra 0x464AE` over the second check's branch makes the
        // validator report success, which is exactly the result the hardware
        // produces on a console that owns the key.
        {0x643C, 2, 0xB050u, true, "SMI RSA result check (beqz $0,0x4648C -> bra 0x4648C)"},
        {0x64A6, 4, 0xB0065000u, true, "SMI keyed check 2 result (mov $0,0; bra 0x464AE)"},
        // The configuration-record validator (0x4906C) ends in `mov $0,$5`, so a
        // failed MAC leaves 0x78000/0x800F0005 in the return value even when its
        // branches are cleared.  Forcing `mov $0,0` makes the validator report
        // success, the same substitution the SMI validator gets.
        {0x918C, 2, 0x5000u, true, "configuration record validator result (mov $0,0)"},
        // The "SCE" command dispatcher (0x4A6DE) validates the structures it is
        // handed (`*(arg) != 0`, the size field, `[structure+8] == 5`, ...) and
        // reports 0x800F0624/0x800F0616 for anything it does not like.  Its
        // answers come from the secure engine, which needs the per-console
        // keyring; the model echoes the request block instead, so the dispatcher
        // is forced to report success (same development substitution as above).
        {0xABF0, 2, 0x5000u, true, "SCE command dispatcher result (mov $0,0)"},
        // The SCE answer validator (0x49D4E..0x49D5C) compares fields of the
        // engine's answer with values the request carried; with the engine
        // substituted those never line up, so the validator is forced to report
        // success as well (its single epilogue is `mov $0,$5`).
        {0x9D5C, 2, 0x5000u, true, "SCE answer validator result (mov $0,0)"},
        // Round 153-11/153-13: with the SCE window engine working (docs/SYSCON.md
        // 8.11) the second loader's cold path gets all the way to building
        // SceKblParam itself.  Two per-console checks still stand in the way and
        // neither can be answered from the dumps we have:
        //   * 0x4A52E verifies a 32 byte digest of the metadata against the copy
        //     the loader carries (error 0x800F0627);
        //   * 0x40F5E compares the firmware version it reads through 0x40E40 with
        //     0x01040000 (error 0x800F0037); 0x40E40 answers 0xFFFFFFFF because
        //     the idstorage reconstruction is an erased placeholder, and the
        //     caller passes the right value in $6 anyway.
        {0xA532, 2, 0xB00Eu, true, "SCE metadata digest check (beqz $0,0x4A540 -> bra 0x4A540)"},
        {0xF76, 2, 0x0360u, true, "firmware version compare (lw $3,0x4($sp) -> mov $3,$6)"},
    };
// Development substitution for the SMI leaf, applied to the decrypted second
// loader (see docs/KBL.md, round 23).  The loader validates the idstorage SMI
// leaf (id 0x80) with keyed integrity checks and an SCE signature whose keys are
// baked into its own image and exist in no dump, exactly like the fused boot key
// the RSA check of the first stage uses - and that one is already substituted by
// machine/bootkeys.cpp.  The four conditional branches below are the only places
// that turn a failed check into the loader's error exit, so clearing them lets
// the boot chain continue into the secure kernel hand-off.  `ZLB_NO_SUBSTITUTION=1`
// turns the substitution off so the unmodified behaviour stays reproducible.
void apply_development_substitutions(Bus& bus, u32 base, size_t length) {
    static const bool disabled = [] {
        const char* value = std::getenv("ZLB_NO_SUBSTITUTION");
        return value != nullptr && value[0] != '0';
    }();
    if (disabled) return;
    if (base != kSecondLoaderBase || length < cmep_detail::kSecondLoaderStagedBytes) return;
    // Apply every cleared branch first and the forced instructions afterwards:
    // 0x64A6 is in both groups, and the replacement has to win.
    for (int pass = 0; pass < 2; ++pass) {
        const bool forced_pass = pass == 1;
        for (const SecondLoaderPatch& patch : kSecondLoaderPatches) {
            if (patch.forced != forced_pass) continue;
            const u32 address = kSecondLoaderBase + patch.offset;
            // Guard against exactly the round-91 mistake: a patch that lands
            // outside the staged image writes nowhere and would only lie in the
            // log.  Unmapped addresses read back as 0xFFFFFFFF.
            if (!bus.is_ram(address, patch.length)) {
                ZLB_LOG_WARN("bigmac",
                             "development substitution %s targets 0x%05X, which is not mapped memory - "
                             "the patch has no effect (is the offset an absolute address?)",
                             patch.what, address);
                continue;
            }
            for (u32 i = 0; i < patch.length; ++i) {
                const u8 value = patch.forced ? static_cast<u8>((patch.word >> (8 * i)) & 0xFF) : 0x00;
                bus.write8(address + i, value);
            }
            ZLB_LOG_INFO("bigmac", "development substitution: %s the %s at 0x%05X (ZLB_NO_SUBSTITUTION=1 disables)",
                         patch.forced ? "forced" : "cleared", patch.what, address);
        }
    }
}

}  // namespace
namespace cmep_detail {

const SecondLoaderPatch* second_loader_patches(size_t& count) {
    count = sizeof(kSecondLoaderPatches) / sizeof(kSecondLoaderPatches[0]);
    return kSecondLoaderPatches;
}

}  // namespace cmep_detail

namespace cmep_detail {

namespace {

// ---------------------------------------------------------------------------
// AES (FIPS-197), 128/192/256 bit keys
// ---------------------------------------------------------------------------

const u8 kSbox[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16};

const u8 kInvSbox[256] = {
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d};

const u8 kRcon[15] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
                      0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d, 0x9a};

inline u8 gf_mul(u8 a, u8 b) {
    u8 p = 0;
    for (int i = 0; i < 8; ++i) {
        if (b & 1) p ^= a;
        u8 hi = static_cast<u8>(a & 0x80);
        a = static_cast<u8>(a << 1);
        if (hi) a ^= 0x1b;
        b = static_cast<u8>(b >> 1);
    }
    return p;
}

/// Expanded key schedule: 4 words per round, at most 15 rounds -> 60 words.
struct AesKey {
    u32 round[60];
    int rounds = 10;  // Nr
};

void aes_expand_key(const u8* key, unsigned key_bits, AesKey& out) {
    const int nk = static_cast<int>(key_bits / 32);
    out.rounds = nk + 6;
    const int total = 4 * (out.rounds + 1);
    for (int i = 0; i < nk; ++i) {
        out.round[i] = (static_cast<u32>(key[4 * i]) << 24) | (static_cast<u32>(key[4 * i + 1]) << 16) |
                       (static_cast<u32>(key[4 * i + 2]) << 8) | static_cast<u32>(key[4 * i + 3]);
    }
    for (int i = nk; i < total; ++i) {
        u32 temp = out.round[i - 1];
        if (i % nk == 0) {
            temp = (temp << 8) | (temp >> 24);  // RotWord
            const u8* s = kSbox;
            temp = (static_cast<u32>(s[(temp >> 24) & 0xff]) << 24) |
                   (static_cast<u32>(s[(temp >> 16) & 0xff]) << 16) |
                   (static_cast<u32>(s[(temp >> 8) & 0xff]) << 8) |
                   static_cast<u32>(s[temp & 0xff]);  // SubWord
            temp ^= static_cast<u32>(kRcon[i / nk - 1]) << 24;
        } else if (nk > 6 && i % nk == 4) {
            const u8* s = kSbox;
            temp = (static_cast<u32>(s[(temp >> 24) & 0xff]) << 24) |
                   (static_cast<u32>(s[(temp >> 16) & 0xff]) << 16) |
                   (static_cast<u32>(s[(temp >> 8) & 0xff]) << 8) |
                   static_cast<u32>(s[temp & 0xff]);
        }
        out.round[i] = out.round[i - nk] ^ temp;
    }
}

inline void add_round_key(u8 st[16], const u32* w) {
    for (int c = 0; c < 4; ++c) {
        const u32 k = w[c];
        st[4 * c + 0] ^= static_cast<u8>(k >> 24);
        st[4 * c + 1] ^= static_cast<u8>(k >> 16);
        st[4 * c + 2] ^= static_cast<u8>(k >> 8);
        st[4 * c + 3] ^= static_cast<u8>(k);
    }
}

inline void sub_bytes(u8 st[16], const u8* box) {
    for (int i = 0; i < 16; ++i) st[i] = box[st[i]];
}

inline void shift_rows(u8 st[16]) {
    u8 t[16];
    std::memcpy(t, st, 16);
    st[0] = t[0];  st[4] = t[4];  st[8] = t[8];   st[12] = t[12];
    st[1] = t[5];  st[5] = t[9];  st[9] = t[13];  st[13] = t[1];
    st[2] = t[10]; st[6] = t[14]; st[10] = t[2];  st[14] = t[6];
    st[3] = t[15]; st[7] = t[3];  st[11] = t[7];  st[15] = t[11];
}

inline void inv_shift_rows(u8 st[16]) {
    u8 t[16];
    std::memcpy(t, st, 16);
    st[0] = t[0];  st[4] = t[4];  st[8] = t[8];   st[12] = t[12];
    st[1] = t[13]; st[5] = t[1];  st[9] = t[5];   st[13] = t[9];
    st[2] = t[10]; st[6] = t[14]; st[10] = t[2];  st[14] = t[6];
    st[3] = t[7];  st[7] = t[11]; st[11] = t[15]; st[15] = t[3];
}

inline void mix_columns(u8 st[16]) {
    for (int c = 0; c < 4; ++c) {
        u8* p = st + 4 * c;
        const u8 a0 = p[0], a1 = p[1], a2 = p[2], a3 = p[3];
        p[0] = static_cast<u8>(gf_mul(a0, 2) ^ gf_mul(a1, 3) ^ a2 ^ a3);
        p[1] = static_cast<u8>(a0 ^ gf_mul(a1, 2) ^ gf_mul(a2, 3) ^ a3);
        p[2] = static_cast<u8>(a0 ^ a1 ^ gf_mul(a2, 2) ^ gf_mul(a3, 3));
        p[3] = static_cast<u8>(gf_mul(a0, 3) ^ a1 ^ a2 ^ gf_mul(a3, 2));
    }
}

inline void inv_mix_columns(u8 st[16]) {
    for (int c = 0; c < 4; ++c) {
        u8* p = st + 4 * c;
        const u8 a0 = p[0], a1 = p[1], a2 = p[2], a3 = p[3];
        p[0] = static_cast<u8>(gf_mul(a0, 14) ^ gf_mul(a1, 11) ^ gf_mul(a2, 13) ^ gf_mul(a3, 9));
        p[1] = static_cast<u8>(gf_mul(a0, 9) ^ gf_mul(a1, 14) ^ gf_mul(a2, 11) ^ gf_mul(a3, 13));
        p[2] = static_cast<u8>(gf_mul(a0, 13) ^ gf_mul(a1, 9) ^ gf_mul(a2, 14) ^ gf_mul(a3, 11));
        p[3] = static_cast<u8>(gf_mul(a0, 11) ^ gf_mul(a1, 13) ^ gf_mul(a2, 9) ^ gf_mul(a3, 14));
    }
}

// ---------------------------------------------------------------------------
// SHA-1 / SHA-256
// ---------------------------------------------------------------------------

void sha1_compress(u32 h[5], const u8 block[64]) {
    u32 w[80];
    for (int i = 0; i < 16; ++i) {
        w[i] = (static_cast<u32>(block[4 * i]) << 24) | (static_cast<u32>(block[4 * i + 1]) << 16) |
               (static_cast<u32>(block[4 * i + 2]) << 8) | static_cast<u32>(block[4 * i + 3]);
    }
    for (int i = 16; i < 80; ++i) w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

    u32 a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (int i = 0; i < 80; ++i) {
        u32 f, k;
        if (i < 20) {
            f = (b & c) | ((~b) & d);
            k = 0x5A827999u;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ED9EBA1u;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8F1BBCDCu;
        } else {
            f = b ^ c ^ d;
            k = 0xCA62C1D6u;
        }
        const u32 temp = rol32(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = rol32(b, 30);
        b = a;
        a = temp;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
}

void sha256_compress(u32 h[8], const u8 block[64]) {
    static const u32 k[64] = {
        0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u,
        0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu,
        0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu,
        0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau, 0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
        0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu,
        0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu,
        0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u, 0x19a4c116u,
        0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
        0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u,
        0xc67178f2u};
    u32 w[64];
    for (int i = 0; i < 16; ++i) {
        w[i] = (static_cast<u32>(block[4 * i]) << 24) | (static_cast<u32>(block[4 * i + 1]) << 16) |
               (static_cast<u32>(block[4 * i + 2]) << 8) | static_cast<u32>(block[4 * i + 3]);
    }
    for (int i = 16; i < 64; ++i) {
        const u32 s0 = ror32(w[i - 15], 7) ^ ror32(w[i - 15], 18) ^ (w[i - 15] >> 3);
        const u32 s1 = ror32(w[i - 2], 17) ^ ror32(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    u32 a = h[0], b = h[1], c = h[2], d = h[3];
    u32 e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i) {
        const u32 s1 = ror32(e, 6) ^ ror32(e, 11) ^ ror32(e, 25);
        const u32 ch = (e & f) ^ ((~e) & g);
        const u32 t1 = hh + s1 + ch + k[i] + w[i];
        const u32 s0 = ror32(a, 2) ^ ror32(a, 13) ^ ror32(a, 22);
        const u32 maj = (a & b) ^ (a & c) ^ (b & c);
        const u32 t2 = s0 + maj;
        hh = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d;
    h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}

/// Seed of the deterministic hardware RNG model.
constexpr u64 kRngSeed = 0x464F3044ull;  // "FO0D"

}  // namespace

void sha1_digest(const u8* data, size_t length, u8 out[20]) {
    u32 h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
    size_t offset = 0;
    while (length - offset >= 64) {
        sha1_compress(h, data + offset);
        offset += 64;
    }
    u8 tail[128];
    const size_t rest = length - offset;
    std::memcpy(tail, data + offset, rest);
    tail[rest] = 0x80;
    const size_t padded = (rest + 1 + 8 <= 64) ? 64 : 128;
    std::memset(tail + rest + 1, 0, padded - rest - 1 - 8);
    const u64 bits = static_cast<u64>(length) * 8;
    for (int i = 0; i < 8; ++i) tail[padded - 1 - i] = static_cast<u8>(bits >> (8 * i));
    for (size_t block = 0; block < padded; block += 64) sha1_compress(h, tail + block);
    for (int i = 0; i < 5; ++i) {
        out[4 * i + 0] = static_cast<u8>(h[i] >> 24);
        out[4 * i + 1] = static_cast<u8>(h[i] >> 16);
        out[4 * i + 2] = static_cast<u8>(h[i] >> 8);
        out[4 * i + 3] = static_cast<u8>(h[i]);
    }
}

void sha256_digest(const u8* data, size_t length, u8 out[32]) {
    u32 h[8] = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
    size_t offset = 0;
    while (length - offset >= 64) {
        sha256_compress(h, data + offset);
        offset += 64;
    }
    u8 tail[128];
    const size_t rest = length - offset;
    std::memcpy(tail, data + offset, rest);
    tail[rest] = 0x80;
    const size_t padded = (rest + 1 + 8 <= 64) ? 64 : 128;
    std::memset(tail + rest + 1, 0, padded - rest - 1 - 8);
    const u64 bits = static_cast<u64>(length) * 8;
    for (int i = 0; i < 8; ++i) tail[padded - 1 - i] = static_cast<u8>(bits >> (8 * i));
    for (size_t block = 0; block < padded; block += 64) sha256_compress(h, tail + block);
    for (int i = 0; i < 8; ++i) {
        out[4 * i + 0] = static_cast<u8>(h[i] >> 24);
        out[4 * i + 1] = static_cast<u8>(h[i] >> 16);
        out[4 * i + 2] = static_cast<u8>(h[i] >> 8);
        out[4 * i + 3] = static_cast<u8>(h[i]);
    }
}

void hmac_sha256_digest(const u8* key, size_t key_length, const u8* data, size_t length, u8 out[32]) {
    u8 block[64];
    std::memset(block, 0, sizeof(block));
    if (key_length > 64) {
        u8 hashed[32];
        sha256_digest(key, key_length, hashed);
        std::memcpy(block, hashed, 32);
    } else {
        std::memcpy(block, key, key_length);
    }
    u8 ipad[64];
    u8 opad[64];
    for (int i = 0; i < 64; ++i) {
        ipad[i] = static_cast<u8>(block[i] ^ 0x36);
        opad[i] = static_cast<u8>(block[i] ^ 0x5c);
    }
    // inner = SHA256(ipad || data)
    std::vector<u8> inner;
    inner.reserve(64 + length);
    inner.insert(inner.end(), ipad, ipad + 64);
    inner.insert(inner.end(), data, data + length);
    u8 inner_hash[32];
    sha256_digest(inner.data(), inner.size(), inner_hash);
    std::vector<u8> outer;
    outer.reserve(64 + 32);
    outer.insert(outer.end(), opad, opad + 64);
    outer.insert(outer.end(), inner_hash, inner_hash + 32);
    sha256_digest(outer.data(), outer.size(), out);
}

namespace {

bool native_hmac_function(u32 function) {
    return function == 0x20B3u || function == 0x24B3u ||
           function == 0x28B3u || function == 0x2CB3u;
}

struct NativeHmacStream {
    std::array<u32, 8> inner = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                                0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
    std::array<u8, 64> key{};
    u64 inner_bytes = 64u;  // Includes the already compressed HMAC ipad block.
    std::array<u8, 40> image{};
};

// Native reserves 40 opaque state bytes at context+4, before its digest at
// +2C. The real hardware serialization has not been recovered. This model
// writes canonical SHA chaining words plus a little-endian byte count there,
// and binds the state to its guest pointer/key. Checking the image prevents
// reuse after the guest overwrites or reinitializes that memory. Relocating a
// raw hardware context to a new pointer is not supported by this model yet.
std::array<u8, 40> native_hmac_state_image(const NativeHmacStream& stream) {
    std::array<u8, 40> image{};
    for (size_t i = 0; i < stream.inner.size(); ++i) {
        for (size_t byte = 0; byte < 4u; ++byte) {
            image[i * 4u + byte] = static_cast<u8>(stream.inner[i] >> (24u - byte * 8u));
        }
    }
    for (size_t i = 0; i < 8u; ++i) image[32u + i] = static_cast<u8>(stream.inner_bytes >> (i * 8u));
    return image;
}

void native_hmac_finish(NativeHmacStream& stream, const u8* data, size_t length, u8 digest[32]) {
    size_t offset = 0;
    while (length - offset >= 64u) {
        sha256_compress(stream.inner.data(), data + offset);
        offset += 64u;
    }
    u8 tail[128]{};
    const size_t rest = length - offset;
    if (rest != 0) std::memcpy(tail, data + offset, rest);
    tail[rest] = 0x80u;
    const size_t padded = rest < 56u ? 64u : 128u;
    const u64 bits = (stream.inner_bytes + length) * 8u;
    for (size_t i = 0; i < 8u; ++i) tail[padded - 1u - i] = static_cast<u8>(bits >> (i * 8u));
    for (size_t i = 0; i < padded; i += 64u) sha256_compress(stream.inner.data(), tail + i);
    u8 outer[96];
    for (size_t i = 0; i < stream.key.size(); ++i) outer[i] = stream.key[i] ^ 0x5Cu;
    for (size_t i = 0; i < stream.inner.size(); ++i) {
        for (size_t byte = 0; byte < 4u; ++byte) {
            outer[64u + i * 4u + byte] = static_cast<u8>(stream.inner[i] >> (24u - byte * 8u));
        }
    }
    sha256_digest(outer, sizeof(outer), digest);
}

}  // namespace

void aes_encrypt_block(const u8* key, unsigned key_bits, const u8 in[16], u8 out[16]) {
    AesKey ks;
    aes_expand_key(key, key_bits, ks);
    u8 st[16];
    std::memcpy(st, in, 16);
    add_round_key(st, ks.round);
    for (int r = 1; r < ks.rounds; ++r) {
        sub_bytes(st, kSbox);
        shift_rows(st);
        mix_columns(st);
        add_round_key(st, ks.round + 4 * r);
    }
    sub_bytes(st, kSbox);
    shift_rows(st);
    add_round_key(st, ks.round + 4 * ks.rounds);
    std::memcpy(out, st, 16);
}

void aes_decrypt_block(const u8* key, unsigned key_bits, const u8 in[16], u8 out[16]) {
    AesKey ks;
    aes_expand_key(key, key_bits, ks);
    u8 st[16];
    std::memcpy(st, in, 16);
    add_round_key(st, ks.round + 4 * ks.rounds);
    for (int r = ks.rounds - 1; r >= 1; --r) {
        inv_shift_rows(st);
        sub_bytes(st, kInvSbox);
        add_round_key(st, ks.round + 4 * r);
        inv_mix_columns(st);
    }
    inv_shift_rows(st);
    sub_bytes(st, kInvSbox);
    add_round_key(st, ks.round);
    std::memcpy(out, st, 16);
}

void aes_cbc_crypt(const u8* key, unsigned key_bits, const u8 iv[16], u8* data, size_t length,
                   bool encrypt) {
    if (length < 16) return;
    u8 chain[16];
    std::memcpy(chain, iv, 16);
    for (size_t offset = 0; offset + 16 <= length; offset += 16) {
        u8 block[16];
        if (encrypt) {
            for (int i = 0; i < 16; ++i) block[i] = static_cast<u8>(data[offset + i] ^ chain[i]);
            aes_encrypt_block(key, key_bits, block, data + offset);
            std::memcpy(chain, data + offset, 16);
        } else {
            std::memcpy(block, data + offset, 16);
            aes_decrypt_block(key, key_bits, block, data + offset);
            for (int i = 0; i < 16; ++i) data[offset + i] ^= chain[i];
            std::memcpy(chain, block, 16);
        }
    }
}

void aes_ecb_crypt(const u8* key, unsigned key_bits, u8* data, size_t length, bool encrypt) {
    for (size_t offset = 0; offset + 16 <= length; offset += 16) {
        u8 block[16];
        std::memcpy(block, data + offset, 16);
        if (encrypt) {
            aes_encrypt_block(key, key_bits, block, data + offset);
        } else {
            aes_decrypt_block(key, key_bits, block, data + offset);
        }
    }
}

std::vector<u8> read_key_material(Bus& bus, u32 address, unsigned bits, bool* ok) {
    const size_t bytes = bits / 8;
    std::vector<u8> out(bytes, 0);
    bool good = true;
    for (size_t i = 0; i < bytes; ++i) {
        if (!bus.is_mapped(address + static_cast<u32>(i))) {
            good = false;
            break;
        }
        out[i] = bus.read8(address + static_cast<u32>(i));
    }
    if (ok) *ok = good;
    return out;
}

}  // namespace cmep_detail

// ---------------------------------------------------------------------------
// BigmacDevice
// ---------------------------------------------------------------------------

namespace cmep_detail {

struct BigmacDevice::Impl {
    std::map<u32, u64> regs;
    std::map<u32, NativeHmacStream> native_hmac_states;
    // Native 0x8062AC stages a complete SHA/HMAC key block, including words
    // +220..+23C; AES and the legacy keyring wrappers use the first 8 words.
    std::array<u32, 16> data{};
    u32 start = 0;
    u32 status = 0;
    u32 exception = 0;
    std::array<u8, 32> staging{};  ///< bytes moved between memory and keyrings
};

BigmacDevice::BigmacDevice(Bus& bus, CmepBlock& owner)
    : Device("CMeP.Bigmac", kBaseAddr, kSizeAddr),
      impl_(new Impl()),
      bus_(bus),
      owner_(owner) {
    reset();
}

BigmacDevice::~BigmacDevice() = default;

void BigmacDevice::reset() {
    impl_->regs.clear();
    impl_->native_hmac_states.clear();
    impl_->data.fill(0);
    impl_->start = 0;
    impl_->status = 0;
    impl_->exception = 0;
    impl_->staging.fill(0);
    last_op_ = BigmacOp::None;
    last_function_ = 0;
    last_command_ = 0;
    last_key_.fill(0);
    last_key_bits_ = 0;
    operations_ = 0;
    keyring_transfers_ = 0;
    aes_operations_ = 0;
    hash_operations_ = 0;
    rng_operations_ = 0;
    rng_.seed(kRngSeed);
    initialized_ = true;
}

const char* BigmacDevice::register_name(u32 address) const {
    switch (address) {
        case kCmd: return "Bigmac command";
        case kArg0: return "Bigmac cmd+0x4 (source/len)";
        case kArg1: return "Bigmac cmd+0x8 (destination)";
        case kFunction: return "Bigmac function";
        case kArg2: return "Bigmac flags";
        case kArg3: return "Bigmac key pointer";
        case 0xE0050018: return "Bigmac reserved+0x18";
        case kStart: return "Bigmac start";
        case kStatus: return "Bigmac status (bit0 busy)";
        case 0xE0050028: return "Bigmac ctx";
        case 0xE0050030: return "Bigmac ctx+0x8";
        case 0xE0050034: return "Bigmac native fill value (zero verified)";
        case kException: return "Bigmac exception status";
        case kDataWindow: return "Bigmac key window[0]";
        case kDataWindow + 0x04: return "Bigmac key window[1]";
        case kDataWindow + 0x08: return "Bigmac key window[2]";
        case kDataWindow + 0x0C: return "Bigmac key window[3]";
        case kDataWindow + 0x10: return "Bigmac key window[4]";
        case kDataWindow + 0x14: return "Bigmac key window[5]";
        case kDataWindow + 0x18: return "Bigmac key window[6]";
        case kDataWindow + 0x1C: return "Bigmac key window[7]";
        case kDataWindow + 0x20: return "Bigmac HMAC key window[8]";
        case kDataWindow + 0x24: return "Bigmac HMAC key window[9]";
        case kDataWindow + 0x28: return "Bigmac HMAC key window[10]";
        case kDataWindow + 0x2C: return "Bigmac HMAC key window[11]";
        case kDataWindow + 0x30: return "Bigmac HMAC key window[12]";
        case kDataWindow + 0x34: return "Bigmac HMAC key window[13]";
        case kDataWindow + 0x38: return "Bigmac HMAC key window[14]";
        case kDataWindow + 0x3C: return "Bigmac HMAC key window[15]";
        default: return nullptr;
    }
}

void BigmacDevice::enumerate_registers(std::vector<RegisterInfo>& out) const {
    static const u32 addrs[] = {kCmd, kArg0, kArg1, kFunction, kArg2, kArg3, kStart, kStatus,
                                0xE0050034, kException, kDataWindow};
    for (u32 a : addrs) {
        RegisterInfo info;
        info.address = a;
        const char* name = register_name(a);
        info.name = name ? name : "Bigmac";
        info.reset_value = 0;
        out.push_back(info);
    }
}

u64 BigmacDevice::read(u32 address, unsigned size) {
    if (address == kStatus) {
        // The polling loop at 0x5CDAE waits for bit 0 to clear; the command is
        // executed synchronously when it is started, so busy is already 0.
        return impl_->status & (size >= 8 ? ~0ull : ((1ull << (8 * size)) - 1));
    }
    if (address == kException) return impl_->exception;
    if (address == kStart) return impl_->start;
    if (address >= kDataWindow && address < kDataWindow + 4 * impl_->data.size()) {
        const u32 word = impl_->data[(address - kDataWindow) / 4];
        const unsigned shift = 8 * ((address - kDataWindow) % 4);
        const u64 value = (static_cast<u64>(word) >> shift) & 0xFFFFFFFFull;
        return size >= 8 ? value : (value & ((1ull << (8 * size)) - 1));
    }
    auto it = impl_->regs.find(address);
    const u64 value = (it == impl_->regs.end()) ? 0 : it->second;
    return value & (size >= 8 ? ~0ull : ((1ull << (8 * size)) - 1));
}

void BigmacDevice::write(u32 address, unsigned size, u64 value) {
    const u32 v = static_cast<u32>(value);
    if (address >= kDataWindow && address < kDataWindow + 4 * impl_->data.size()) {
        impl_->data[(address - kDataWindow) / 4] = v;
        return;
    }
    switch (address) {
        case kStart:
            impl_->start = v;
            if (v != 0) {
                // Read the register image exactly as bigmac_cmd (0x5CD20) left
                // it and run one command (see BigmacCommand in cmep_internal.h
                // for the per-register evidence).
                const auto reg = [this](u32 a) {
                    auto it = impl_->regs.find(a);
                    return it == impl_->regs.end() ? 0u : static_cast<u32>(it->second);
                };
                BigmacCommand cmd;
                cmd.raw_command = reg(kArg0);   // +0x04  (0x5CD32: sw $1,($9))
                cmd.pointer = reg(kCmd);        // +0x00  (0x5CD34: sw $2,($11))
                cmd.length = reg(kArg1);        // +0x08  (0x5CD36: sw $3,($12))
                cmd.extra = reg(kArg3);         // +0x14  (0x5CD42: sw $12,($9))
                cmd.flags = reg(kArg2);         // +0x10  (0x5CD5C: sw $4,($12))
                cmd.function = reg(kFunction);  // +0x0C  (0x5CDAA: sw $10,($3))

                std::vector<u8> key;
                const u32 opcode = cmd.opcode();
                const bool transfer =
                    opcode == static_cast<u32>(BigmacFunction::KeyringWrite1) ||
                    opcode == static_cast<u32>(BigmacFunction::KeyringWrite2) ||
                    opcode == static_cast<u32>(BigmacFunction::KeyringRead1);
                if (transfer && cmd.pointer >= 0x1000) {
                    // Keyring transfers pass the key material in +0x00.  The byte
                    // loop at 0x5CD64 copies eight words from that pointer into
                    // the +0x200 window and sets bit 7 of the function (0x5CD98).
                    // It only runs when the flags argument is not below 0x1000
                    // (sltu3 at 0x5CD44).
                    const unsigned bits =
                        (cmd.flags == 128 || cmd.flags == 192 || cmd.flags == 256) ? cmd.flags
                                                                                    : 256u;
                    bool have = false;
                    key = read_key_material(bus_, cmd.pointer, bits, &have);
                    if (!have) key.clear();
                    for (int i = 0; i < 8; ++i) {
                        const u32 word =
                            (4 * i + 3 < static_cast<int>(key.size()))
                                ? static_cast<u32>(key[4 * i]) |
                                      (static_cast<u32>(key[4 * i + 1]) << 8) |
                                      (static_cast<u32>(key[4 * i + 2]) << 16) |
                                      (static_cast<u32>(key[4 * i + 3]) << 24)
                                : 0u;
                        impl_->data[i] = word;
                    }
                }
                // For every other operation the +0x200 window holds the key/IV
                // the caller staged with `sw`, and +0x00 is the *data* address:
                // a key must never be read from it here.
                std::vector<u8> window(native_hmac_function(cmd.function) ? 64u : 32u);
                for (size_t i = 0; i < window.size() / 4; ++i) {
                    window[4 * i + 0] = static_cast<u8>(impl_->data[i]);
                    window[4 * i + 1] = static_cast<u8>(impl_->data[i] >> 8);
                    window[4 * i + 2] = static_cast<u8>(impl_->data[i] >> 16);
                    window[4 * i + 3] = static_cast<u8>(impl_->data[i] >> 24);
                }
                const bool window_nonzero = [&window]() {
                    for (u8 byte : window) {
                        if (byte != 0) return true;
                    }
                    return false;
                }();
                if (!transfer && (window_nonzero || cmd.used_window_key())) {
                    // Bit 7 explicitly selects window material, including an
                    // all-zero key. Legacy AES-128 uses key || IV here; native
                    // AES-256 uses all eight words for the key and +14 for IV.
                    key.assign(window.begin(), window.end());
                }

                static const std::vector<u8> kNoKey;
                const s32 rc = execute(cmd, key.empty() ? &kNoKey : &key, nullptr);
                impl_->status = (rc == 0) ? 0 : kStatusErrorMask;
                impl_->start = 0;
            }
            return;
        case kStatus:
            impl_->status = v;
            return;
        case kException:
            impl_->exception = v;
            return;
        default:
            impl_->regs[address] = v;
            return;
    }
}

s32 BigmacDevice::execute(u32 command, u32 pointer, u32 length,
                          const std::vector<u8>* key_material, u32 function, const u8* iv) {
    BigmacCommand cmd;
    cmd.raw_command = command;
    cmd.pointer = pointer;
    cmd.length = length;
    cmd.function = function;
    return execute(cmd, key_material, iv);
}

s32 BigmacDevice::execute(const BigmacCommand& cmd, const std::vector<u8>* key_material,
                          const u8* iv) {
    operations_++;
    last_command_ = cmd.raw_command;
    last_function_ = cmd.function;
    impl_->exception = 0;
    if (bigmac_trace()) {
        std::fprintf(stderr,
                     "[bigmac] op function=0x%04X ptr=0x%08X len=%u flags=0x%08X cmd=0x%08X pc=%08X\n",
                     cmd.function, cmd.pointer, cmd.length, cmd.flags, cmd.raw_command, bus_.context.pc);
    }

    const u32 index = cmd.index();          // +0x04 (writes) / read1
    const u32 flags = cmd.flags;            // +0x10 (flags, key bits, read2 index)
    const u32 opcode = cmd.opcode();
    u32 scratch = cmd.pointer;  ///< +0x14 of the loader's image

    auto fail = [this, &cmd](u32 code) {
        impl_->exception = code;
        impl_->status = kStatusErrorMask;
        if (bigmac_trace()) {
            std::fprintf(stderr,
                         "[bigmac] fail code=0x%04X function=0x%04X ptr=0x%08X len=%u flags=0x%08X cmd=0x%08X\n",
                         code, cmd.function, cmd.pointer, cmd.length, cmd.flags, cmd.raw_command);
        }
        return static_cast<s32>(-1);
    };
    auto ok = [this]() {
        impl_->status = 0;
        return static_cast<s32>(0);
    };

    const auto store_key = [this](const std::vector<u8>& bytes) {
        const size_t n = bytes.size() < 32 ? bytes.size() : 32;
        last_key_.fill(0);
        std::memcpy(last_key_.data(), bytes.data(), n);
        last_key_bits_ = static_cast<unsigned>(bytes.size() * 8);
    };

    switch (opcode) {
        case static_cast<u32>(BigmacFunction::KeyringWrite1): {
            last_op_ = BigmacOp::KeyringWrite;
            keyring_transfers_++;
            // cmd.pointer is the source, cmd.length the length (0x5C69E passes
            // $2 = 0x5E704 / $3 = 32 for keyring 8).
            std::vector<u8> payload;
            if (key_material != nullptr) {
                payload = *key_material;
            } else if (cmd.pointer >= 0x1000) {
                const unsigned bits = (flags == 128 || flags == 192 || flags == 256) ? flags : 256u;
                bool have = false;
                payload = read_key_material(bus_, cmd.pointer, bits, &have);
                if (!have) return fail(0x0002);
            }
            if (payload.size() > cmd.length && cmd.length != 0) payload.resize(cmd.length);
            store_key(payload);
            owner_.keyring_device().stage_keyring(index, flags, payload);
            // Some stages copy the captured record to a scratch buffer through
            // the +0x14 argument (keyring_write2 passes 0x5E880 there).
            if (cmd.extra >= 0x1000 && !payload.empty()) {
                owner_.keyring_device().copy_to_bus(cmd.extra, payload);
            }
            return ok();
        }
        case static_cast<u32>(BigmacFunction::KeyringRead1): {
            last_op_ = BigmacOp::KeyringRead;
            keyring_transfers_++;
            // 0x5CCD8 is the keyring wrapper and bigmac_cmd sets bit 28 of the
            // function word when the +0x04 argument is below 0x1000
            // (0x5CD44..0x5CD4E), so the hardware tells the two forms apart:
            //
            //   "small index" (0x5C904, 0x5C93A)  +0x04 = keyring index
            //       (+0x00 = 32 bytes of material, +0x08 = 32, +0x10 = flags
            //        0x208, +0x14 = scratch the loader memsets right afterwards).
            //       The first loader installs slot 10 from image+0xE0 and slots
            //       11..15 from the header digest table at image+0x100.
            //
            //   "large" (0x5C9BA)  +0x04 = destination (0x5EDE0), +0x00 = the
            //       wrapped 32-byte block (image+0x1A0), +0x08 = 32, +0x10 = the
            //       keyring index (10), +0x14 = scratch.  0x5CA00 then memcmps
            //       the destination against the SHA-256 of the payload.
            //
            // On hardware the second form unwraps the block with the keyring key
            // (a chip-resident secret no dump has), so the model unwraps it as a
            // copy and machine/bootkeys.cpp positions the staged image's block so
            // the loader's own comparison still succeeds - the same development
            // substitution the RSA key table uses.
            std::vector<u8> block;
            const size_t want = cmd.length == 0 ? 32 : cmd.length;
            block.reserve(want);
            for (size_t i = 0; i < want; ++i) {
                const u32 a = cmd.pointer + static_cast<u32>(i);
                if (!bus_.is_mapped(a)) return fail(0x0012);
                block.push_back(bus_.read8(a));
            }

            if (cmd.small_index()) {
                // The +0x04 field is a keyring index below 0x1000, so this is a
                // *load* of the slot, not a transform.  Slots are one-time
                // programmable: 0x5C904 loads index 10 - the fused boot key - and
                // the hardware keeps the fused value, which is why the first
                // loader can pre-load 10 without destroying it.
                std::vector<u8> existing;
                if (!owner_.keyring_device().read_keyring(cmd.index(), existing)) {
                    owner_.keyring_device().stage_keyring(cmd.index(), cmd.flags, block);
                }
                store_key(block);
                return ok();
            }

            std::vector<u8> value;
            if (!owner_.keyring_device().read_keyring(cmd.flags, value)) {
                // An unprogrammed keyring reads back as zeros, exactly like the
                // hardware.
                value.assign(32, 0);
            }
            store_key(value);
            if (cmd.raw_command >= 0x400) owner_.keyring_device().copy_to_bus(cmd.raw_command, block);
            if (cmd.extra >= 0x400) owner_.keyring_device().copy_to_bus(cmd.extra, block);
            return ok();
        }
        default:
            break;
    }

    // ---- AES / hash / RNG -------------------------------------------------
    std::vector<u8> key;
    if (key_material != nullptr) {
        key = *key_material;
    } else if (cmd.used_window_key() || cmd.pointer < 0x1000) {
        key.resize(native_hmac_function(cmd.function) ? 64u : 32u);
        for (size_t i = 0; i < key.size() / 4; ++i) {
            key[4 * i + 0] = static_cast<u8>(impl_->data[i]);
            key[4 * i + 1] = static_cast<u8>(impl_->data[i] >> 8);
            key[4 * i + 2] = static_cast<u8>(impl_->data[i] >> 16);
            key[4 * i + 3] = static_cast<u8>(impl_->data[i] >> 24);
        }
    }

    /// Resolve a bigmac_cmd address argument: values below 0x1000 are offsets
    /// into the staging buffer the ARM/CMeP copies the image to (0x40000).
    const auto resolve = [](u32 address) -> u32 {
        return address < 0x1000 ? (cmep::kRamBase + address) : address;
    };
    const auto read_range = [this](u32 address, size_t length, std::vector<u8>& out) -> bool {
        const u32 base = address < 0x1000 ? (cmep::kRamBase + address) : address;
        out.assign(length, 0);
        for (size_t i = 0; i < length; ++i) {
            const u32 a = base + static_cast<u32>(i);
            if (!bus_.is_mapped(a)) return false;
            out[i] = bus_.read8(a);
        }
        return true;
    };
    const auto write_range = [this](u32 address, const std::vector<u8>& bytes) -> bool {
        const u32 base = address < 0x1000 ? (cmep::kRamBase + address) : address;
        for (size_t i = 0; i < bytes.size(); ++i) {
            const u32 a = base + static_cast<u32>(i);
            if (!bus_.is_mapped(a)) return false;
            bus_.write8(a, bytes[i]);
        }
        return true;
    };

    // The native secure-kernel wrappers use +00/+04 as source/destination.
    // Validate the complete range before allocating or writing, including
    // adjacent RAM regions, so a bad DMA destination cannot receive a prefix
    // while the command reports an error.
    const auto native_range_mapped = [this](u32 address, size_t length) {
        if (static_cast<u64>(address) + length > 0x100000000ull) return false;
        if (length == 0) return true;
        return bus_.is_mapped(address) && bus_.first_unmapped(address, length) == 0;
    };

    if (cmd.function == 0x000Cu && cmd.pointer == 0 && cmd.flags == 0) {
        // Genuine secure-kernel 0x8055A0, called by module cleanup 0x80056E,
        // writes the fill value at +34, then source=0, destination=+04,
        // byte length=+08 and function=0x000C. The captured zero-fill form
        // ignores the stale IV/key registers. It must not pass through the
        // legacy in-place AES decoder: resolving source zero to boot SRAM
        // would overwrite the kernel through its uncached SRAM alias.
        // Nonzero fill pattern width/serialization is not yet established;
        // reject it rather than guessing a pattern or reporting completion.
        last_op_ = BigmacOp::None;
        const auto fill = impl_->regs.find(0xE0050034u);
        if (fill != impl_->regs.end() && fill->second != 0) return fail(0x0014);
        if (!native_range_mapped(cmd.raw_command, cmd.length)) return fail(0x0013);
        bus_.memset_bytes(cmd.raw_command, 0, cmd.length);
        return ok();
    }

    if (cmd.function == 0 || cmd.function == 0x2080u) {
        // 0x8055D6 copies the RVK SCE header through channel 0: +00 source,
        // +04 destination, +08 byte length, +0C zero, +1C start. This is real
        // DMA, including the ARM-RAM to CMeP-private-RAM transfer. The native
        // 0x806C78 wrapper retains context bits 0x2080 for the same transfer;
        // RVK uses that form to read its plain 32-byte section at image +400.
        last_op_ = BigmacOp::None;
        if (!native_range_mapped(cmd.pointer, cmd.length)) return fail(0x0012);
        if (!native_range_mapped(cmd.raw_command, cmd.length)) return fail(0x0013);
        std::vector<u8> buffer(cmd.length);
        bus_.read_bytes(cmd.pointer, buffer.data(), buffer.size());
        bus_.write_bytes(cmd.raw_command, buffer.data(), buffer.size());
        return ok();
    }

    if (cmd.function == 0x238Au || cmd.function == 0x218Au) {
        // Native 0x804514 -> 0x805D56: AES-CBC decrypt, with 256-bit (0x238A)
        // or 128-bit (0x218A) key at +200, an independent IV buffer addressed
        // by +14, source at +00 and destination at +04. Both RVK metadata
        // operations must produce their output before reporting success.
        last_op_ = BigmacOp::Aes;
        aes_operations_++;
        const size_t key_bytes = cmd.function == 0x238Au ? 32u : 16u;
        if (cmd.length == 0 || (cmd.length % 16) != 0) return fail(0x0011);
        if (key.size() < key_bytes) return fail(0x0010);
        if (!native_range_mapped(cmd.pointer, cmd.length) ||
            !native_range_mapped(cmd.extra, 16)) return fail(0x0012);
        if (!native_range_mapped(cmd.raw_command, cmd.length)) return fail(0x0013);
        std::vector<u8> buffer(cmd.length);
        bus_.read_bytes(cmd.pointer, buffer.data(), buffer.size());
        key.resize(key_bytes);
        store_key(key);
        bus_.read_bytes(cmd.extra, last_iv_.data(), last_iv_.size());
        std::vector<u8> out(buffer.size());
        if (key_bytes == 32u) {
            aes256_cbc_decrypt(key.data(), last_iv_.data(), buffer.data(), buffer.size(), out.data());
        } else {
            aes128_cbc_decrypt(key.data(), last_iv_.data(), buffer.data(), buffer.size(), out.data());
        }
        bus_.write_bytes(cmd.raw_command, out.data(), out.size());
        return ok();
    }

    if (cmd.function == 0x21A1u) {
        // Native 0x804514 -> 0x805D56: AES-128-CTR, source +00, destination
        // +04, byte length +08, key +200 and mutable counter buffer at +14.
        // 0x8045D8 reverses all 16 IV bytes before hardware; 0x804620 reverses
        // the updated counter back for the caller's next chunk. Live RVK
        // 0x805E62 confirms this representation for its 672-byte section.
        last_op_ = BigmacOp::Aes;
        aes_operations_++;
        if ((cmd.length % 16) != 0) return fail(0x0011);
        if (key.size() < 16u) return fail(0x0010);
        if (!native_range_mapped(cmd.pointer, cmd.length) ||
            !native_range_mapped(cmd.extra, 16)) return fail(0x0012);
        if (!native_range_mapped(cmd.raw_command, cmd.length)) return fail(0x0013);
        std::vector<u8> buffer(cmd.length);
        bus_.read_bytes(cmd.pointer, buffer.data(), buffer.size());
        key.resize(16u);
        store_key(key);
        bus_.read_bytes(cmd.extra, last_iv_.data(), last_iv_.size());
        std::array<u8, 16> counter{};
        for (size_t i = 0; i < counter.size(); ++i) counter[i] = last_iv_[15u - i];
        std::vector<u8> out(buffer.size());
        aes_ctr_crypt(key.data(), counter.data(), buffer.data(), buffer.size(), out.data());
        // Add the consumed block count to the complete big-endian counter,
        // retaining carry across every byte and wrapping modulo 2^128.
        u64 carry = cmd.length / 16u;
        for (size_t i = counter.size(); i != 0; --i) {
            const u64 sum = counter[i - 1u] + (carry & 0xFFu);
            counter[i - 1u] = static_cast<u8>(sum);
            carry = (carry >> 8u) + (sum >> 8u);
        }
        std::array<u8, 16> hardware_counter{};
        for (size_t i = 0; i < counter.size(); ++i) hardware_counter[i] = counter[15u - i];
        bus_.write_bytes(cmd.raw_command, out.data(), out.size());
        bus_.write_bytes(cmd.extra, hardware_counter.data(), hardware_counter.size());
        return ok();
    }

    if (cmd.function == 0x2093u) {
        // Native 0x805EF2 -> 0x806110 issues one-shot SHA-256: +00 source,
        // +04 32-byte digest destination and +08 source length. The retained
        // +10 field belongs to key selection, not a digest-size argument.
        // RVK hashes 768 bytes at 0x808FF0; kprx hashes its staged SCE header.
        last_op_ = BigmacOp::Sha;
        hash_operations_++;
        if (!native_range_mapped(cmd.pointer, cmd.length)) return fail(0x0020);
        if (!native_range_mapped(cmd.raw_command, 32)) return fail(0x0021);
        // Keep a valid pointer for the SHA helper even for an empty message.
        std::vector<u8> buffer(cmd.length == 0 ? 1u : cmd.length);
        bus_.read_bytes(cmd.pointer, buffer.data(), cmd.length);
        std::array<u8, 32> digest{};
        sha256_digest(buffer.data(), cmd.length, digest.data());
        bus_.write_bytes(cmd.raw_command, digest.data(), digest.size());
        return ok();
    }

    if (native_hmac_function(cmd.function)) {
        // Native 0x8061E0 final, single-shot HMAC-SHA256: +00 source, +04
        // 32-byte digest destination, +08 message length, and the caller's
        // padded 64-byte key block at +200..+23F. +14 points at incremental
        // state (context +4); this form starts/finalizes a fresh hash, without
        // consuming that previous state. Native kprx now also reaches first
        // non-final 0x24B3 (28544 bytes) then final 0x28B3 (52 bytes), sharing
        // one context+4 pointer/key. 0x2CB3 is the same wrapper's continuation.
        last_op_ = BigmacOp::Hmac;
        hash_operations_++;
        if (key.size() != 64u) return fail(0x0022);
        if (!native_range_mapped(cmd.pointer, cmd.length)) return fail(0x0020);
        if (!native_range_mapped(cmd.raw_command, 32)) return fail(0x0021);
        const bool first = cmd.function == 0x24B3u;
        const bool final = cmd.function == 0x28B3u;
        const bool streaming = cmd.function != 0x20B3u;
        NativeHmacStream stream;
        if (streaming) {
            if (!final && (cmd.length % 64u) != 0) return fail(0x0023);
            if (!native_range_mapped(cmd.extra, stream.image.size())) return fail(0x0024);
            if (first) {
                std::memcpy(stream.key.data(), key.data(), stream.key.size());
                std::array<u8, 64> ipad{};
                for (size_t i = 0; i < ipad.size(); ++i) ipad[i] = stream.key[i] ^ 0x36u;
                sha256_compress(stream.inner.data(), ipad.data());
            } else {
                const auto previous = impl_->native_hmac_states.find(cmd.extra);
                if (previous == impl_->native_hmac_states.end()) return fail(0x0024);
                stream = previous->second;
                std::array<u8, 40> image{};
                bus_.read_bytes(cmd.extra, image.data(), image.size());
                if (image != stream.image ||
                    std::memcmp(stream.key.data(), key.data(), stream.key.size()) != 0) return fail(0x0024);
            }
            if (stream.inner_bytes > 0x1FFFFFFFFFFFFFFFull - cmd.length) return fail(0x0023);
        }
        std::vector<u8> buffer(cmd.length == 0 ? 1u : cmd.length);
        bus_.read_bytes(cmd.pointer, buffer.data(), cmd.length);
        std::array<u8, 32> digest{};
        store_key(key);
        if (!streaming) {
            hmac_sha256_digest(key.data(), key.size(), buffer.data(), cmd.length, digest.data());
            bus_.write_bytes(cmd.raw_command, digest.data(), digest.size());
            impl_->native_hmac_states.erase(cmd.extra);
        } else if (final) {
            native_hmac_finish(stream, buffer.data(), cmd.length, digest.data());
            bus_.write_bytes(cmd.raw_command, digest.data(), digest.size());
            impl_->native_hmac_states.erase(cmd.extra);
        } else {
            for (size_t i = 0; i < cmd.length; i += 64u) sha256_compress(stream.inner.data(), buffer.data() + i);
            stream.inner_bytes += cmd.length;
            stream.image = native_hmac_state_image(stream);
            bus_.write_bytes(cmd.extra, stream.image.data(), stream.image.size());
            impl_->native_hmac_states[cmd.extra] = stream;
            // Non-final calls publish chaining state, not a completed HMAC.
        }
        return ok();
    }

    // 0x010A (wrapper 0x5CCC0) unwraps the staged image.  0x5CBCC passes
    //   +0x00 = image+0x2C0 (source), +0x04 = image (destination),
    //   +0x08 = image[0x10] = 0x16600, +0x10 = keyring 10
    // and the result is the plaintext image the first loader jumps to at 0x40000
    // (AES-128-CBC with keyring 10's key || IV - see CmepBlock::seed_boot_keyring).
    // 0x5C6E4 passes source == destination for the same length, which cannot be a
    // transform of the image (the header checks that follow still read the
    // container), so an in-place call is modelled as a keyed check that reports
    // success.
    if (opcode == static_cast<u32>(BigmacFunction::KeyringWrite2)) {
        last_op_ = BigmacOp::Aes;
        aes_operations_++;
        std::vector<u8> material;
        if (!owner_.keyring_device().read_keyring(cmd.flags, material)) material.assign(32, 0);
        store_key(material);
        if (cmd.pointer == cmd.raw_command) return ok();

        std::vector<u8> buffer;
        if (!read_range(cmd.pointer, cmd.length, buffer)) return fail(0x0012);
        if (buffer.empty() || (buffer.size() % 16) != 0) return fail(0x0011);
        std::vector<u8> out(buffer.size(), 0);
        aes128_cbc_decrypt(material.data(), material.data() + 16, buffer.data(), buffer.size(), out.data());
        if (!write_range(cmd.raw_command, out)) return fail(0x0013);
        apply_development_substitutions(bus_, cmd.raw_command, out.size());
        return ok();
    }

    if (opcode == static_cast<u32>(BigmacFunction::Rng)) {
        last_op_ = BigmacOp::Rng;
        rng_operations_++;
        // The hardware fills the key window with fresh random words; mirror that
        // so a reader of +0x200..+0x21F sees the modelled stream.
        for (int i = 0; i < 8; ++i) {
            impl_->data[i] = rng_.next();
        }
        return ok();
    }

    // These small AES encodings are the legacy emulator protocol. In
    // particular, the first-loader 0x5CD06 function0x000C call has a nonzero
    // source/destination and its hardware meaning remains unverified. Keep
    // that existing interpretation distinct from the native fill above.
    switch (opcode) {
        case static_cast<u32>(BigmacFunction::AesCbcEncrypt):
        case static_cast<u32>(BigmacFunction::AesCbcDecrypt):
        case static_cast<u32>(BigmacFunction::AesEcbEncrypt):
        case static_cast<u32>(BigmacFunction::AesEcbDecrypt): {
            last_op_ = BigmacOp::Aes;
            aes_operations_++;
            // The key size is the flags argument (bigmac_cmd stores $4 in
            // +0x10); the window always carries key || IV, so a 128-bit key is
            // the first 16 bytes of a 32-byte material block.
            unsigned key_bits = 0;
            if (flags == 128 || flags == 192 || flags == 256) {
                key_bits = flags;
            } else if (!key.empty()) {
                key_bits = static_cast<unsigned>(key.size() * 8);
            } else {
                key_bits = 128;
            }
            if (key.empty()) key.assign(key_bits / 8, 0);
            if (key.size() < key_bits / 8) key.resize(key_bits / 8, 0);
            if (key_bits != 128 && key_bits != 192 && key_bits != 256) {
                key_bits = 128;
                key.resize(16, 0);
            }
            store_key(key);
            const size_t length = cmd.length;
            if (length == 0 || (length % 16) != 0) return fail(0x0011);
            std::vector<u8> buffer;
            if (!read_range(cmd.pointer, length, buffer)) return fail(0x0012);
            const bool encrypt = (opcode == static_cast<u32>(BigmacFunction::AesCbcEncrypt) ||
                                  opcode == static_cast<u32>(BigmacFunction::AesEcbEncrypt));
            const bool cbc = (opcode == static_cast<u32>(BigmacFunction::AesCbcEncrypt) ||
                              opcode == static_cast<u32>(BigmacFunction::AesCbcDecrypt));
            if (cbc) {
                // The IV comes from the caller when one was supplied, otherwise
                // from the tail of the key window (key in the low words, IV in
                // the high ones - the layout keyring_write1/2 produce).
                if (iv != nullptr) {
                    std::memcpy(last_iv_.data(), iv, 16);
                } else if (key.size() >= 32) {
                    std::memcpy(last_iv_.data(), key.data() + 16, 16);
                } else {
                    std::memcpy(last_iv_.data(), key.data(), 16);
                    std::memset(last_iv_.data() + key.size(), 0, 16 - key.size());
                }
                aes_cbc_crypt(key.data(), key_bits, last_iv_.data(), buffer.data(), buffer.size(),
                              encrypt);
            } else {
                last_iv_.fill(0);
                aes_ecb_crypt(key.data(), key_bits, buffer.data(), buffer.size(), encrypt);
            }
            if (!write_range(cmd.pointer, buffer)) return fail(0x0013);
            // The result stays visible in the key window for the debugger.
            for (int i = 0; i < 8 && (i * 4 + 3) < static_cast<int>(buffer.size()); ++i) {
                impl_->data[i] = static_cast<u32>(buffer[4 * i]) |
                                 (static_cast<u32>(buffer[4 * i + 1]) << 8) |
                                 (static_cast<u32>(buffer[4 * i + 2]) << 16) |
                                 (static_cast<u32>(buffer[4 * i + 3]) << 24);
            }
            return ok();
        }
        case static_cast<u32>(BigmacFunction::Sha256): {
            // 0x5CCF0 is the SHA-256 wrapper: $1 (+0x04) is the destination, $2
            // (+0x00) the source, $3 (+0x08) the length and $4 (+0x10) the digest
            // size.  0x5C7F6 hashes image[0:0x1C0] (the ENP header before the
            // signature) into 0x5EDC0 and 0x5C9DA hashes a record into 0x5EE00.
            last_op_ = BigmacOp::Sha;
            hash_operations_++;
            std::vector<u8> buffer;
            if (!read_range(cmd.pointer, cmd.length, buffer)) return fail(0x0020);
            std::vector<u8> digest(32);
            sha256_digest(buffer.data(), buffer.size(), digest.data());
            for (int i = 0; i < 8; ++i) {
                const size_t base = 4 * static_cast<size_t>(i);
                impl_->data[i] = static_cast<u32>(digest[base]) |
                                 (static_cast<u32>(digest[base + 1]) << 8) |
                                 (static_cast<u32>(digest[base + 2]) << 16) |
                                 (static_cast<u32>(digest[base + 3]) << 24);
            }
            size_t length = cmd.flags == 0 ? digest.size() : cmd.flags;
            if (length > digest.size()) length = digest.size();
            digest.resize(length);
            if (cmd.raw_command >= 0x400 && !write_range(cmd.raw_command, digest)) {
                return fail(0x0021);
            }
            return ok();
        }
        case static_cast<u32>(BigmacFunction::Sha1):
        case static_cast<u32>(BigmacFunction::HmacSha256): {
            last_op_ = hash_op(opcode);
            hash_operations_++;
            std::vector<u8> buffer;
            if (!read_range(cmd.pointer, cmd.length, buffer)) return fail(0x0020);
            std::vector<u8> digest;
            if (opcode == static_cast<u32>(BigmacFunction::Sha1)) {
                digest.resize(20);
                sha1_digest(buffer.data(), buffer.size(), digest.data());
            } else {
                digest.resize(32);
                if (key.empty()) key.assign(16, 0);
                store_key(key);
                hmac_sha256_digest(key.data(), key.size(), buffer.data(), buffer.size(),
                                   digest.data());
            }
            for (int i = 0; i < 8; ++i) {
                const size_t base = 4 * static_cast<size_t>(i);
                impl_->data[i] = (base + 3 < digest.size())
                                     ? static_cast<u32>(digest[base]) |
                                           (static_cast<u32>(digest[base + 1]) << 8) |
                                           (static_cast<u32>(digest[base + 2]) << 16) |
                                           (static_cast<u32>(digest[base + 3]) << 24)
                                     : 0u;
            }
            if (scratch >= 0x1000) {
                if (!write_range(scratch, digest)) return fail(0x0021);
            }
            return ok();
        }
        default:
            break;
    }
    // The remaining second-loader keyed family (bit 13 of the function word,
    // e.g. 0x2309, plus the 0x03xx variants such as 0x033B that the
    // wrapper 0x453AA builds from the keyring descriptor at 0x2300 and its
    // caller's mode bits) is used only by the per-console SMI/config paths, and
    // its keyslot 0x213/0x212 is missing from every dump in the workspace, so
    // the exact mode bits cannot be pinned down from data the emulator owns.  The
    // operations are therefore reported as successes instead of raising the
    // exception status; every check that reads their output is already handled by
    // the documented substitutions in apply_development_substitutions().
    // `ZLB_NO_SUBSTITUTION=1` keeps the unmodified behaviour (error 0x78000).
    if ((cmd.function & 0xF000u) == 0x2000u || (cmd.function & 0xFF00u) == 0x0300u ||
        (cmd.function & 0xFF00u) == 0x0100u) {
        static const bool disabled = [] {
            const char* value = std::getenv("ZLB_NO_SUBSTITUTION");
            return value != nullptr && value[0] != '0';
        }();
        if (!disabled) {
            last_op_ = BigmacOp::None;
            if (bigmac_trace()) {
                std::fprintf(stderr,
                             "[bigmac] keyed op function=0x%04X ptr=0x%08X len=%u flags=0x%08X "
                             "(development substitution: reported ok)\n",
                             cmd.function, cmd.pointer, cmd.length, cmd.flags);
            }
            return ok();
        }
    }
    // Unknown command: the hardware raises the exception status the failure
    // path at 0x5C5FE reads back from 0xE005003C.
    last_op_ = BigmacOp::None;
    return fail(0x0080 | (opcode & 0x7F));
}


BigmacOp BigmacDevice::hash_op(u32 function) {
    switch (function) {
        case static_cast<u32>(BigmacFunction::Sha1):
        case static_cast<u32>(BigmacFunction::Sha256):
            return BigmacOp::Sha;
        case static_cast<u32>(BigmacFunction::HmacSha256):
            return BigmacOp::Hmac;
        default:
            return BigmacOp::None;
    }
}

std::string BigmacDevice::summary() const {
    return format("ops=%llu keyring=%llu aes=%llu hash=%llu rng=%llu last_op=%u f=0x%04X",
                  static_cast<unsigned long long>(operations_),
                  static_cast<unsigned long long>(keyring_transfers_),
                  static_cast<unsigned long long>(aes_operations_),
                  static_cast<unsigned long long>(hash_operations_),
                  static_cast<unsigned long long>(rng_operations_), static_cast<unsigned>(last_op_),
                  last_function_);
}

void BigmacDevice::describe(std::vector<std::string>& lines) const {
    lines.push_back(format("Bigmac command      = 0x%08X", last_command_));
    lines.push_back(format("Bigmac function     = 0x%04X", last_function_));
    lines.push_back(format("Bigmac last key     = %u bits", last_key_bits_));
    std::string hexkey;
    for (unsigned i = 0; i < last_key_bits_ / 8 && i < 32; ++i) hexkey += format("%02X", last_key_[i]);
    if (!hexkey.empty()) lines.push_back("Bigmac key material = " + hexkey);
    lines.push_back(format("Bigmac rng state    = 0x%016llX",
                           static_cast<unsigned long long>(rng_.state)));
}

}  // namespace cmep_detail
}  // namespace zlb
