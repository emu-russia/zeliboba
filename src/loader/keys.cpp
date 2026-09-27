// zeliboba - SCE key store and the crypto primitives the boot chain needs.
//
// The key tables are a faithful transcription of pup_fiction's three key sets
// (keys.py == keys_external.py, keys_internal.py, keys_proto.py) in the same
// insertion order, because KeyStore.get() is order dependent: it walks the
// entries of (key type, SCE type, SELF type) in registration order and returns
// the first one whose system version window and key revision match
// (scetypes.py KeyStore.get).
//
// keys.h only exposes a *named* byte store (get/has/set + select_self_key), so
// the table is kept inside that store with a documented internal naming scheme
// and the KeyStore logic is exported as free functions from loader_extra.h:
//
//   @key/<keytype>/<scetype>/<selftype>/<seq:04>
//        value = the key bytes exactly as registered (AES-128 or AES-256)
//   @meta/<keytype>/<scetype>/<selftype>/<seq:04>/<keyrev:02>/<min:016X>/<max:016X>/<source>:<line>
//        value = the 16 byte IV
//
// `<seq>` is the registration index, so the std::map order inside a triple is
// exactly the python registration order. This keeps SceKeys copyable, keeps the
// built-in tables available in every instance (a file loaded with
// load_from_file() *adds* candidates) and needs no extra member in keys.h.
//
// Everything is dependency free: AES-128/192/256 (block encrypt/decrypt + CBC),
// SHA-1, SHA-256, HMAC-SHA256 and a big-endian bignum modular exponentiation
// for RSA. AES-CTR (needed by the SELF segments) is built on the AES-128 ECB
// block primitive elsewhere; the declaration lives in loader/loader_extra.h.
#include "loader/keys.h"

#include <array>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "common/util.h"
#include "loader/loader_extra.h"

namespace zlb {

// ---------------------------------------------------------------------------
// Key table metadata (not keys.h ABI)
// ---------------------------------------------------------------------------

namespace {

constexpr int kKeyTypeMetadata = 0;
constexpr int kKeyTypeNpdrm = 1;

// SCE container types (scetypes.py SceType).
constexpr int kSceTypeSelf = 1;
constexpr int kSceTypeSrvk = 2;
constexpr int kSceTypeSpkg = 3;
constexpr int kSceTypeDev = 0xC0;

// SELF types (scetypes.py SelfType).
constexpr int kSelfTypeNone = 0x00;
constexpr int kSelfTypeKernel = 0x07;
constexpr int kSelfTypeApp = 0x08;
constexpr int kSelfTypeBoot = 0x09;
constexpr int kSelfTypeSecure = 0x0B;
constexpr int kSelfTypeUser = 0x0D;

constexpr u64 kMaxVerAll = 0xFFFFFFFFFFFFFFFFull;

struct KeyRow {
    const char* source;
    int line;
    int key_type;
    int sce_type;
    int self_type;
    int key_rev;
    u64 min_ver;
    u64 max_ver;
    const char* key_hex;
    const char* iv_hex;
};

const KeyRow kKeyRows[] = {
    // ---- pup_fiction/keys.py (byte identical to keys_external.py) ----------
    {"pup_fiction/keys.py", 7, kKeyTypeMetadata, kSceTypeSpkg, kSelfTypeNone, 0, 0x00000000000ull,
     0xFFF00000000ull,
     "2E6F4751D15B06C51F572A9306E52DD7007EA56A31D459EC6D3681AB08625501",
     "B3D541A568751DF8F4833BAB4EFE0537"},
    {"pup_fiction/keys.py", 18, kKeyTypeMetadata, kSceTypeSrvk, kSelfTypeNone, 0, 0x10300000000ull,
     0x16920000000ull,
     "4648164DB9E67009456C7CA6F2378835FD678539B36B3DE6F1C604B7D4258141",
     "6EC8AD67993DAE75675F0AFFDE5C41F3"},
    {"pup_fiction/keys.py", 29, kKeyTypeMetadata, kSceTypeSrvk, kSelfTypeNone, 0, 0x18000000000ull,
     0xFFF00000000ull,
     "DAE4B0F901E338DEFF3CCDBDEA1E2FDEA9926BB98CB182443CC0C0F7FAE428EE",
     "18D925FA885C7E28A9CFF458C24D8BED"},
    {"pup_fiction/keys.py", 40, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeSecure, 1, 0x10300000000ull,
     0x16920000000ull,
     "9D4E4CE92EA1C4576EB9601EC43EC03AAE8EC324ECF6DE01E918E61D2223EE55",
     "CFEA3CCBA454D3279AD7CB0510431434"},
    {"pup_fiction/keys.py", 51, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeSecure, 1, 0x18000000000ull,
     0x36100000000ull,
     "B1B6FEB39A8BD7A2AC584D435E150C624F560D3EFB03E745C575E0844569E2D0",
     "89B4E6BAB03B03D49BF0FC927FEA8659"},
    {"pup_fiction/keys.py", 62, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeSecure, 1, 0x36300000000ull,
     0xFFF00000000ull,
     "59AC7F05E115D758201A3F3461BCA0D42BD186F00CFC24263973F622AD9ED30C",
     "A053B00BA4BF880799B4265C6BC064B5"},
    {"pup_fiction/keys.py", 73, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeBoot, 1, 0x10300000000ull,
     0x16920000000ull,
     "7A7FB1560DCD121CEA5E11B90124B13282752F2D5B95D75036AB3A29BB3BD2AB",
     "6C71642A042A041F1EE3094070B009BE"},
    {"pup_fiction/keys.py", 84, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeBoot, 1, 0x18000000000ull,
     0x36100000000ull,
     "B1B936B512F9A16E51B948622B26F15C53680C77AC332EC25846B839520393EC",
     "90D527BAF7296B5B6A576CFA6B54D266"},
    {"pup_fiction/keys.py", 95, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeBoot, 1, 0x36300000000ull,
     0xFFF00000000ull,
     "426FD1D33FEBBFAC560B7957B94F445AE5F1DED2AA70F74DB944645DC439122F",
     "995F1364BB9735FA448B18D886150C85"},
    {"pup_fiction/keys.py", 106, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeKernel, 1, 0x10300000000ull,
     0x16920000000ull,
     "B4AAF62D48FBD898C240308A9773AFE57B8A18D783F0B37932BB21B51386A9A0",
     "8CD162C5C613376F3E4BEA0B8FD5A3D0"},
    {"pup_fiction/keys.py", 117, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeKernel, 1, 0x18000000000ull,
     0x36100000000ull,
     "849AF7E8DE5B9C28C38CA74963FCF155E0F200FB08185E46CDA87790AAA10D72",
     "88710E219454A3CBF6D382D4BBD22BFC"},
    {"pup_fiction/keys.py", 128, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeKernel, 1, 0x36300000000ull,
     0xFFF00000000ull,
     "18E26DF712C362769D4F5E70460D28D88B7B991733DE692C2B9463B41FF4B925",
     "5B13077EEA801FC77D492050801FA507"},
    {"pup_fiction/keys.py", 139, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeUser, 0, 0x10300000000ull,
     0x16920000000ull,
     "4769935C3B1CB248C3A88A406B1535D5DC2C0279D5901DE534DC4A11B8F60804",
     "0CE906F746D40105660456D827CEBD25"},
    {"pup_fiction/keys.py", 150, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeUser, 0, 0x18000000000ull,
     0xFFF00000000ull,
     "4769935C3B1CB248C3A88A406B1535D5DC2C0279D5901DE534DC4A11B8F60804",
     "0CE906F746D40105660456D827CEBD25"},
    {"pup_fiction/keys.py", 161, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeUser, 1, 0x10300000000ull,
     0xFFF00000000ull,
     "613AD6EAC63D4E14F51A8C6AF18C66621968323B6F205B5E515C16D77BB06671",
     "ADBDAA5041B2094CF2B359301DE64171"},
    {"pup_fiction/keys.py", 172, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeUser, 2, 0x18000000000ull,
     0xFFF00000000ull,
     "0F2041269B26D6B7EF143E35E83E914629A92F50F3A4CEE14CDFF63AEC641117",
     "07EF64437F0CB6995E6D785E42796C83"},
    {"pup_fiction/keys.py", 183, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeUser, 3, 0x18000000000ull,
     0xFFF00000000ull,
     "3AFADA34660C6515B539EBBBC79C9C0ADA4337C32652CA03C6DD21A1D612D8F4",
     "7F98A137869B91B1EB9604F81FD74C50"},
    {"pup_fiction/keys.py", 194, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeUser, 4, 0x36300000000ull,
     0xFFF00000000ull,
     "8FF491B36713E8AA38DE30B303689657F07AE70A8A8B1D7867441C52DB39C806",
     "D9CC7E26CE99053E48F9BEF1CB93C184"},
    {"pup_fiction/keys.py", 205, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeUser, 5, 0x36300000000ull,
     0xFFF00000000ull,
     "4D71B2FB3D4359FB34445305C88A5E82FA12D34A8308F312AA34B58F6112253A",
     "04A27133FF0205C96B7F45A60D7D417B"},
    {"pup_fiction/keys.py", 216, kKeyTypeNpdrm, kSceTypeSelf, kSelfTypeApp, 0, 0x00000000000ull,
     kMaxVerAll, "C10368BF3D2943BC6E5BD05E46A9A7B6", "00000000000000000000000000000000"},
    {"pup_fiction/keys.py", 227, kKeyTypeNpdrm, kSceTypeSelf, kSelfTypeApp, 1, 0x00000000000ull,
     kMaxVerAll, "16419DD3BFBE8BDC596929B72CE237CD", "00000000000000000000000000000000"},
    {"pup_fiction/keys.py", 238, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeApp, 0, 0x00000000000ull,
     0x16920000000ull,
     "AAA508FA5E85EAEE597ED2B27804D22287CFADF1DF32EDC7A7C58E8C9AA8BB36",
     "CD1BD3A59200CC67A3B804808DC2AE73"},
    {"pup_fiction/keys.py", 249, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeApp, 0, 0x18000000000ull,
     kMaxVerAll,
     "5661E5FB20CFD1D1DFF50C1E59A6EA977D0AA5C5770F53B9CDD4E9451FFF55CB",
     "23D02FF79BF430E2D123869BF0CACAA0"},
    {"pup_fiction/keys.py", 260, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeApp, 1, 0x00000000000ull,
     kMaxVerAll,
     "4181B2DF5F5D94D3C80B7D86EACF1928533A49BA58EDE2B43CDEE7E572568BD4",
     "B1678C0543B6C1997B63A6F4F3C8FD33"},
    {"pup_fiction/keys.py", 271, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeApp, 2, 0x18000000000ull,
     kMaxVerAll,
     "5282582F17F068F89A260AAFB71C58928F45A8D08C681376B07FF9EAB1114226",
     "29672DF43E426F41AF46D42E8437D449"},
    {"pup_fiction/keys.py", 282, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeApp, 3, 0x18000000000ull,
     kMaxVerAll,
     "270CBA370061B87077672ADB5142D18844AAED352A9CCEE63602B0D740594334",
     "1CF2454FBF47D76221B91AFC3B608C28"},
    {"pup_fiction/keys.py", 293, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeApp, 4, 0x35700000000ull,
     kMaxVerAll,
     "A782BC5A9EDDFC49A513FF3E592C4677A8C8920F23C9F11F2558FB9D99A43868",
     "559B5E658559EB65EBF892C274E098A9"},
    {"pup_fiction/keys.py", 304, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeApp, 5, 0x35700000000ull,
     kMaxVerAll,
     "12D64D0172495226010A687DE245A73DE028B3561E25E69BABC325636F3CAE0A",
     "F149EED1757E5A915B24309795BFC380"},

    // ---- pup_fiction/keys_internal.py (devkit / prototype set) -------------
    {"pup_fiction/keys_internal.py", 7, kKeyTypeMetadata, kSceTypeSpkg, kSelfTypeNone, 0,
     0x00000000000ull, 0xFFF00000000ull,
     "23F1D525244266E6DA7A52DA9446318301EE8CC58D54901AE94D93010F7DEE6B",
     "3721F7C05DE5F55ECC39BDDB4A6C585D"},
    {"pup_fiction/keys_internal.py", 18, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeSecure, 0,
     0x00000000000ull, 0xFFF00000000ull,
     "AED9D76EE1E29290002BFF32D4B0656EEE40FBDA4F8B55BE5BE0ED83530F27D2",
     "DB50912F2416B54F7F36227169ECE500"},
    {"pup_fiction/keys_internal.py", 29, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeBoot, 0,
     0x00000000000ull, 0xFFF00000000ull,
     "9D3F28DE30DED1D503DB6FA762A571C422A88D0F361899EF36D357059C72EC43",
     "30E43CFB57D418A5A0D32A9939D23501"},
    {"pup_fiction/keys_internal.py", 40, kKeyTypeMetadata, kSceTypeSrvk, kSelfTypeNone, 0,
     0x00000000000ull, 0xFFF00000000ull,
     "EAB14F9BE15EAEC1603BE63C9FCDE4099D601FB0E9FC4DF250B8DEC635987A1C",
     "30B9E61707993B635D0E182446DB0B8D"},
    {"pup_fiction/keys_internal.py", 51, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeKernel, 0,
     0x00000000000ull, 0xFFF00000000ull,
     "74F6D2A1D2A093AE32B83337E0AE4AD2E6D93B034F5BF3B68DB77131883310D4",
     "926AB55BDADC45DBB610E90E56A0368C"},
    {"pup_fiction/keys_internal.py", 62, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeUser, 0,
     0x00000000000ull, 0xFFF00000000ull,
     "322D706CB6EBEA14DEF7BFE45F812971347DC95CD7697C16A71EA4B2A1E12C0D",
     "31FA2E606031EDF39665B5616E9F937D"},
    {"pup_fiction/keys_internal.py", 73, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeUser, 1,
     0x00000000000ull, 0xFFF00000000ull,
     "DA3BE69B77B3A857EA4F6CDC73C0AB0590C0A95E145B8D55D2D3A6447C247F46",
     "A0385383AB31497E3AFB7CCDDB30CA5A"},
    {"pup_fiction/keys_internal.py", 84, kKeyTypeNpdrm, kSceTypeSelf, kSelfTypeApp, 0,
     0x00000000000ull, kMaxVerAll, "C10368BF3D2943BC6E5BD05E46A9A7B6",
     "00000000000000000000000000000000"},
    {"pup_fiction/keys_internal.py", 95, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeApp, 0,
     0x00000000000ull, 0x16920000000ull,
     "AAA508FA5E85EAEE597ED2B27804D22287CFADF1DF32EDC7A7C58E8C9AA8BB36",
     "CD1BD3A59200CC67A3B804808DC2AE73"},
    {"pup_fiction/keys_internal.py", 106, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeApp, 1,
     0x00000000000ull, kMaxVerAll,
     "4181B2DF5F5D94D3C80B7D86EACF1928533A49BA58EDE2B43CDEE7E572568BD4",
     "B1678C0543B6C1997B63A6F4F3C8FD33"},

    // ---- pup_fiction/keys_proto.py (prototype set) ------------------------
    {"pup_fiction/keys_proto.py", 9, kKeyTypeMetadata, kSceTypeSpkg, kSelfTypeNone, 0,
     0x00000000000ull, 0xFFF00000000ull,
     "FA88E5B5CBB49603DF689F139045E7C3C9C7E33B5923DF54E4C5FE5298B4FD32",
     "5EAA69AB35E737EC22C721A916E00263"},
    {"pup_fiction/keys_proto.py", 20, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeSecure, 1,
     0x00000000000ull, 0xFFF00000000ull,
     "B982589B568CDD4055433747DF19644A8D1B479B17CA44ECE5E82694550FEC74",
     "BECEDF96543939032CC4DD7D95E47720"},
    {"pup_fiction/keys_proto.py", 31, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeBoot, 1,
     0x00000000000ull, 0xFFF00000000ull,
     "9EE16CA4AADD77F53BEE0F4AE3D45326D009806D2DE9942CE0836E43DC5DD1CE",
     "CFBA84A87EE29C9A521CA20691485E45"},
    {"pup_fiction/keys_proto.py", 42, kKeyTypeMetadata, kSceTypeSrvk, kSelfTypeNone, 0,
     0x00000000000ull, 0xFFF00000000ull,
     "A603AA68753CEE3E186C81900A862DCDB13505D39FC59C62BBFAD94C526B8A06",
     "352F596CFB513A148B95F9D78E57E755"},
    {"pup_fiction/keys_proto.py", 53, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeKernel, 1,
     0x00000000000ull, 0xFFF00000000ull,
     "61E7E786BB6F67570A71FC92E73885439CD16B96BC7C37C200EF11D3446FCF69",
     "99E8B68EE784FDAFC3294B8E55F0C529"},
    {"pup_fiction/keys_proto.py", 64, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeUser, 0,
     0x00000000000ull, 0xFFF00000000ull,
     "DA3BE69B77B3A857EA4F6CDC73C0AB0590C0A95E145B8D55D2D3A6447C247F46",
     "A0385383AB31497E3AFB7CCDDB30CA5A"},
    {"pup_fiction/keys_proto.py", 75, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeUser, 1,
     0x00000000000ull, 0xFFF00000000ull,
     "8D355E70736EF7AA508D640D8D382B19D9C8747C4A8273A6D5707F227F49592E",
     "BEB4819878915F3025978538693B3EBB"},
    {"pup_fiction/keys_proto.py", 86, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeApp, 0,
     0x00000000000ull, 0x16920000000ull,
     "AAA508FA5E85EAEE597ED2B27804D22287CFADF1DF32EDC7A7C58E8C9AA8BB36",
     "CD1BD3A59200CC67A3B804808DC2AE73"},
    {"pup_fiction/keys_proto.py", 97, kKeyTypeMetadata, kSceTypeSelf, kSelfTypeApp, 1,
     0x00000000000ull, kMaxVerAll,
     "4181B2DF5F5D94D3C80B7D86EACF1928533A49BA58EDE2B43CDEE7E572568BD4",
     "B1678C0543B6C1997B63A6F4F3C8FD33"},
    {"pup_fiction/keys_proto.py", 108, kKeyTypeNpdrm, kSceTypeSelf, kSelfTypeApp, 0,
     0x00000000000ull, kMaxVerAll, "C10368BF3D2943BC6E5BD05E46A9A7B6",
     "00000000000000000000000000000000"},
    // keys_proto.py registers an *all zero* keyrev 1 NPDRM key; kept as is.
    {"pup_fiction/keys_proto.py", 119, kKeyTypeNpdrm, kSceTypeSelf, kSelfTypeApp, 1,
     0x00000000000ull, kMaxVerAll, "00000000000000000000000000000000",
     "00000000000000000000000000000000"},
};

constexpr size_t kKeyRowCount = sizeof(kKeyRows) / sizeof(kKeyRows[0]);

std::vector<u8> hex_to_bytes(const std::string& text) {
    auto is_hex = [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    };
    std::string filtered;
    filtered.reserve(text.size());
    for (char c : text) {
        if (is_hex(c)) filtered.push_back(c);
    }
    if ((filtered.size() & 1u) != 0) return {};
    auto digit = [](char c) -> unsigned {
        if (c >= '0' && c <= '9') return static_cast<unsigned>(c - '0');
        if (c >= 'a' && c <= 'f') return static_cast<unsigned>(c - 'a' + 10);
        return static_cast<unsigned>(c - 'A' + 10);
    };
    std::vector<u8> out;
    out.reserve(filtered.size() / 2);
    for (size_t i = 0; i < filtered.size(); i += 2)
        out.push_back(static_cast<u8>((digit(filtered[i]) << 4) | digit(filtered[i + 1])));
    return out;
}

std::string entry_key_name(int key_type, int sce_type, int self_type, unsigned sequence) {
    return format("@key/%d/%02X/%02X/%04u", key_type, sce_type, self_type, sequence);
}

std::string entry_meta_name(const KeyRow& row, unsigned sequence) {
    // The version window carries an explicit 0x prefix: parse_u64() would read a
    // plain "0000010300000000" as a *decimal* number and the window comparison
    // would then filter out every key.
    return format("@meta/%d/%02X/%02X/%04u/%02d/0x%016llX/0x%016llX/%s:%d", row.key_type,
                  row.sce_type, row.self_type, sequence, row.key_rev,
                  static_cast<unsigned long long>(row.min_ver),
                  static_cast<unsigned long long>(row.max_ver), row.source, row.line);
}

const char* key_type_name(int key_type) { return key_type == kKeyTypeNpdrm ? "npdrm" : "metadata"; }

const char* sce_type_name(int sce_type) {
    switch (sce_type) {
        case kSceTypeSelf: return "self";
        case kSceTypeSrvk: return "srvk";
        case kSceTypeSpkg: return "spkg";
        case kSceTypeDev: return "dev";
        default: return "sce";
    }
}

const char* self_type_name(int self_type) {
    switch (self_type) {
        case kSelfTypeNone: return "NONE";
        case kSelfTypeKernel: return "KERNEL";
        case kSelfTypeApp: return "APP";
        case kSelfTypeBoot: return "BOOT";
        case kSelfTypeSecure: return "SECURE";
        case kSelfTypeUser: return "USER";
        default: return "SELF_TYPE";
    }
}

/// Registered entries of one (key type, SCE type, SELF type) triple, in
/// registration order. Parsed back out of the SceKeys name/value store.
std::vector<SceKeyCandidate> entries_of(const SceKeys& keys, int key_type, int sce_type,
                                        int self_type) {
    std::vector<SceKeyCandidate> out;
    const std::string prefix = format("@meta/%d/%02X/%02X/", key_type, sce_type, self_type);
    for (const std::string& name : keys.names()) {
        if (name.compare(0, prefix.size(), prefix) != 0) continue;

        // @meta/<kt>/<st>/<selft>/<seq>/<keyrev>/<min>/<max>/<source>:<line>
        std::vector<std::string> parts = split(name, '/');
        if (parts.size() < 10) continue;

        SceKeyCandidate candidate;
        candidate.key_revision = static_cast<int>(std::strtol(parts[5].c_str(), nullptr, 10));
        u64 min_ver = 0;
        u64 max_ver = 0;
        parse_u64(parts[6], min_ver);
        parse_u64(parts[7], max_ver);
        candidate.min_version = min_ver;
        candidate.max_version = max_ver;
        candidate.source = parts[8];
        for (size_t i = 9; i < parts.size(); ++i) candidate.source += "/" + parts[i];
        candidate.iv = keys.get(name);
        candidate.key = keys.get(format("@key/%d/%02X/%02X/%s", key_type, sce_type, self_type,
                                        parts[4].c_str()));
        if (candidate.key.empty()) continue;
        out.push_back(std::move(candidate));
    }
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// KeyStore semantics (declared in loader_extra.h)
// ---------------------------------------------------------------------------

std::vector<SceKeyCandidate> sce_key_candidates(const SceKeys& keys, int key_type, int sce_type,
                                                bool ignore_sys_version, s64 sys_version,
                                                int key_revision, int self_type) {
    std::vector<SceKeyCandidate> out;
    for (SceKeyCandidate candidate : entries_of(keys, key_type, sce_type, self_type)) {
        if (!ignore_sys_version && sys_version >= 0) {
            if (static_cast<u64>(sys_version) < candidate.min_version ||
                static_cast<u64>(sys_version) > candidate.max_version)
                continue;
        }
        if (key_revision >= 0 && key_revision != candidate.key_revision) continue;
        out.push_back(std::move(candidate));
    }
    return out;
}

bool sce_keys_lookup(const SceKeys& keys, int key_type, int sce_type, s64 sys_version,
                     int key_revision, int self_type, SceKeyCandidate& out) {
    const std::vector<SceKeyCandidate> candidates =
        sce_key_candidates(keys, key_type, sce_type, false, sys_version, key_revision, self_type);
    if (candidates.empty()) return false;
    out = candidates.front();
    return true;
}

// ===========================================================================
//  AES
// ===========================================================================
//
// FIPS-197 byte oriented implementation; RoundKey[16*round + 4*col + row].

namespace {

constexpr u8 kSbox[256] = {
    0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5, 0x30, 0x01, 0x67, 0x2B, 0xFE, 0xD7, 0xAB, 0x76,
    0xCA, 0x82, 0xC9, 0x7D, 0xFA, 0x59, 0x47, 0xF0, 0xAD, 0xD4, 0xA2, 0xAF, 0x9C, 0xA4, 0x72, 0xC0,
    0xB7, 0xFD, 0x93, 0x26, 0x36, 0x3F, 0xF7, 0xCC, 0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
    0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A, 0x07, 0x12, 0x80, 0xE2, 0xEB, 0x27, 0xB2, 0x75,
    0x09, 0x83, 0x2C, 0x1A, 0x1B, 0x6E, 0x5A, 0xA0, 0x52, 0x3B, 0xD6, 0xB3, 0x29, 0xE3, 0x2F, 0x84,
    0x53, 0xD1, 0x00, 0xED, 0x20, 0xFC, 0xB1, 0x5B, 0x6A, 0xCB, 0xBE, 0x39, 0x4A, 0x4C, 0x58, 0xCF,
    0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85, 0x45, 0xF9, 0x02, 0x7F, 0x50, 0x3C, 0x9F, 0xA8,
    0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5, 0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2,
    0xCD, 0x0C, 0x13, 0xEC, 0x5F, 0x97, 0x44, 0x17, 0xC4, 0xA7, 0x7E, 0x3D, 0x64, 0x5D, 0x19, 0x73,
    0x60, 0x81, 0x4F, 0xDC, 0x22, 0x2A, 0x90, 0x88, 0x46, 0xEE, 0xB8, 0x14, 0xDE, 0x5E, 0x0B, 0xDB,
    0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C, 0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79,
    0xE7, 0xC8, 0x37, 0x6D, 0x8D, 0xD5, 0x4E, 0xA9, 0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
    0xBA, 0x78, 0x25, 0x2E, 0x1C, 0xA6, 0xB4, 0xC6, 0xE8, 0xDD, 0x74, 0x1F, 0x4B, 0xBD, 0x8B, 0x8A,
    0x70, 0x3E, 0xB5, 0x66, 0x48, 0x03, 0xF6, 0x0E, 0x61, 0x35, 0x57, 0xB9, 0x86, 0xC1, 0x1D, 0x9E,
    0xE1, 0xF8, 0x98, 0x11, 0x69, 0xD9, 0x8E, 0x94, 0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
    0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68, 0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16,
};

constexpr u8 kInvSbox[256] = {
    0x52, 0x09, 0x6A, 0xD5, 0x30, 0x36, 0xA5, 0x38, 0xBF, 0x40, 0xA3, 0x9E, 0x81, 0xF3, 0xD7, 0xFB,
    0x7C, 0xE3, 0x39, 0x82, 0x9B, 0x2F, 0xFF, 0x87, 0x34, 0x8E, 0x43, 0x44, 0xC4, 0xDE, 0xE9, 0xCB,
    0x54, 0x7B, 0x94, 0x32, 0xA6, 0xC2, 0x23, 0x3D, 0xEE, 0x4C, 0x95, 0x0B, 0x42, 0xFA, 0xC3, 0x4E,
    0x08, 0x2E, 0xA1, 0x66, 0x28, 0xD9, 0x24, 0xB2, 0x76, 0x5B, 0xA2, 0x49, 0x6D, 0x8B, 0xD1, 0x25,
    0x72, 0xF8, 0xF6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xD4, 0xA4, 0x5C, 0xCC, 0x5D, 0x65, 0xB6, 0x92,
    0x6C, 0x70, 0x48, 0x50, 0xFD, 0xED, 0xB9, 0xDA, 0x5E, 0x15, 0x46, 0x57, 0xA7, 0x8D, 0x9D, 0x84,
    0x90, 0xD8, 0xAB, 0x00, 0x8C, 0xBC, 0xD3, 0x0A, 0xF7, 0xE4, 0x58, 0x05, 0xB8, 0xB3, 0x45, 0x06,
    0xD0, 0x2C, 0x1E, 0x8F, 0xCA, 0x3F, 0x0F, 0x02, 0xC1, 0xAF, 0xBD, 0x03, 0x01, 0x13, 0x8A, 0x6B,
    0x3A, 0x91, 0x11, 0x41, 0x4F, 0x67, 0xDC, 0xEA, 0x97, 0xF2, 0xCF, 0xCE, 0xF0, 0xB4, 0xE6, 0x73,
    0x96, 0xAC, 0x74, 0x22, 0xE7, 0xAD, 0x35, 0x85, 0xE2, 0xF9, 0x37, 0xE8, 0x1C, 0x75, 0xDF, 0x6E,
    0x47, 0xF1, 0x1A, 0x71, 0x1D, 0x29, 0xC5, 0x89, 0x6F, 0xB7, 0x62, 0x0E, 0xAA, 0x18, 0xBE, 0x1B,
    0xFC, 0x56, 0x3E, 0x4B, 0xC6, 0xD2, 0x79, 0x20, 0x9A, 0xDB, 0xC0, 0xFE, 0x78, 0xCD, 0x5A, 0xF4,
    0x1F, 0xDD, 0xA8, 0x33, 0x88, 0x07, 0xC7, 0x31, 0xB1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xEC, 0x5F,
    0x60, 0x51, 0x7F, 0xA9, 0x19, 0xB5, 0x4A, 0x0D, 0x2D, 0xE5, 0x7A, 0x9F, 0x93, 0xC9, 0x9C, 0xEF,
    0xA0, 0xE0, 0x3B, 0x4D, 0xAE, 0x2A, 0xF5, 0xB0, 0xC8, 0xEB, 0xBB, 0x3C, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2B, 0x04, 0x7E, 0xBA, 0x77, 0xD6, 0x26, 0xE1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0C, 0x7D,
};

constexpr u8 kRcon[15] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
                          0x1B, 0x36, 0x6C, 0xD8, 0xAB, 0x4D, 0x9A};

struct AesSchedule {
    int rounds = 10;
    std::array<u8, 240> rk{};   // up to 15 round keys of 16 bytes
};

void aes_expand_key(const u8* key, size_t key_length, AesSchedule& out) {
    const size_t nk = key_length / 4;         // 4, 6 or 8 words
    out.rounds = static_cast<int>(nk) + 6;    // 10, 12 or 14
    const size_t words = 4 * (static_cast<size_t>(out.rounds) + 1);

    for (size_t i = 0; i < nk; ++i) {
        out.rk[4 * i + 0] = key[4 * i + 0];
        out.rk[4 * i + 1] = key[4 * i + 1];
        out.rk[4 * i + 2] = key[4 * i + 2];
        out.rk[4 * i + 3] = key[4 * i + 3];
    }

    for (size_t i = nk; i < words; ++i) {
        u8 t[4] = {out.rk[4 * (i - 1) + 0], out.rk[4 * (i - 1) + 1], out.rk[4 * (i - 1) + 2],
                   out.rk[4 * (i - 1) + 3]};
        if (i % nk == 0) {
            const u8 first = t[0];
            t[0] = static_cast<u8>(kSbox[t[1]] ^ kRcon[i / nk - 1]);
            t[1] = kSbox[t[2]];
            t[2] = kSbox[t[3]];
            t[3] = kSbox[first];
        } else if (nk > 6 && i % nk == 4) {
            t[0] = kSbox[t[0]];
            t[1] = kSbox[t[1]];
            t[2] = kSbox[t[2]];
            t[3] = kSbox[t[3]];
        }
        for (size_t j = 0; j < 4; ++j)
            out.rk[4 * i + j] = static_cast<u8>(out.rk[4 * (i - nk) + j] ^ t[j]);
    }
}

inline u8 gf_mul(u8 a, u8 b) {
    u8 result = 0;
    while (b != 0) {
        if (b & 1) result ^= a;
        const u8 high = static_cast<u8>(a & 0x80);
        a = static_cast<u8>(a << 1);
        if (high) a ^= 0x1B;
        b = static_cast<u8>(b >> 1);
    }
    return result;
}

void aes_encrypt_block_schedule(const AesSchedule& s, const u8 in[16], u8 out[16]) {
    u8 st[16];
    std::memcpy(st, in, 16);
    for (int i = 0; i < 16; ++i) st[i] ^= s.rk[i];

    for (int round = 1; round <= s.rounds; ++round) {
        for (int i = 0; i < 16; ++i) st[i] = kSbox[st[i]];   // SubBytes
        u8 tmp[16];                                          // ShiftRows
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r) tmp[4 * c + r] = st[4 * ((c + r) & 3) + r];
        std::memcpy(st, tmp, 16);
        if (round != s.rounds) {                             // MixColumns
            for (int c = 0; c < 4; ++c) {
                u8* col = st + 4 * c;
                const u8 a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
                col[0] = static_cast<u8>(gf_mul(a0, 2) ^ gf_mul(a1, 3) ^ a2 ^ a3);
                col[1] = static_cast<u8>(a0 ^ gf_mul(a1, 2) ^ gf_mul(a2, 3) ^ a3);
                col[2] = static_cast<u8>(a0 ^ a1 ^ gf_mul(a2, 2) ^ gf_mul(a3, 3));
                col[3] = static_cast<u8>(gf_mul(a0, 3) ^ a1 ^ a2 ^ gf_mul(a3, 2));
            }
        }
        const u8* rk = s.rk.data() + 16 * round;             // AddRoundKey
        for (int i = 0; i < 16; ++i) st[i] ^= rk[i];
    }
    std::memcpy(out, st, 16);
}

void aes_decrypt_block_schedule(const AesSchedule& s, const u8 in[16], u8 out[16]) {
    u8 st[16];
    std::memcpy(st, in, 16);
    const u8* last = s.rk.data() + 16 * s.rounds;
    for (int i = 0; i < 16; ++i) st[i] ^= last[i];

    for (int round = s.rounds - 1; round >= 0; --round) {
        u8 tmp[16];                                          // InvShiftRows
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r) tmp[4 * ((c + r) & 3) + r] = st[4 * c + r];
        std::memcpy(st, tmp, 16);
        for (int i = 0; i < 16; ++i) st[i] = kInvSbox[st[i]];   // InvSubBytes
        const u8* rk = s.rk.data() + 16 * round;                // AddRoundKey
        for (int i = 0; i < 16; ++i) st[i] ^= rk[i];
        if (round != 0) {                                       // InvMixColumns
            for (int c = 0; c < 4; ++c) {
                u8* col = st + 4 * c;
                const u8 a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
                col[0] = static_cast<u8>(gf_mul(a0, 14) ^ gf_mul(a1, 11) ^ gf_mul(a2, 13) ^
                                         gf_mul(a3, 9));
                col[1] = static_cast<u8>(gf_mul(a0, 9) ^ gf_mul(a1, 14) ^ gf_mul(a2, 11) ^
                                         gf_mul(a3, 13));
                col[2] = static_cast<u8>(gf_mul(a0, 13) ^ gf_mul(a1, 9) ^ gf_mul(a2, 14) ^
                                         gf_mul(a3, 11));
                col[3] = static_cast<u8>(gf_mul(a0, 11) ^ gf_mul(a1, 13) ^ gf_mul(a2, 9) ^
                                         gf_mul(a3, 14));
            }
        }
    }
    std::memcpy(out, st, 16);
}

/// Build a schedule from a 16/24/32 byte key; other lengths are zero
/// padded/truncated to 16 bytes so a malformed table entry cannot read out of
/// bounds (mirrors CoreAes.NormalizeKey).
void aes_schedule_any(const u8* key, size_t key_length, AesSchedule& schedule) {
    if (key_length == 16 || key_length == 24 || key_length == 32) {
        aes_expand_key(key, key_length, schedule);
        return;
    }
    u8 padded[16] = {};
    const size_t n = key_length < 16 ? key_length : 16;
    if (n > 0) std::memcpy(padded, key, n);
    aes_expand_key(padded, 16, schedule);
}

/// CBC decryption with the key size taken into account (16/24/32 bytes); a
/// trailing partial block is zero padded like the C# reference.
void cbc_decrypt_with(const u8* key, size_t key_length, const u8* iv, const u8* input, size_t length,
                      u8* output) {
    const u8 empty_iv[16] = {};
    AesSchedule schedule;
    aes_schedule_any(key, key_length, schedule);

    u8 chain[16];
    std::memcpy(chain, iv != nullptr ? iv : empty_iv, 16);

    size_t done = 0;
    while (done < length) {
        const size_t count = (length - done) < 16 ? (length - done) : 16;
        u8 block[16] = {};
        std::memcpy(block, input + done, count);
        u8 plain[16];
        aes_decrypt_block_schedule(schedule, block, plain);
        for (size_t i = 0; i < 16; ++i) output[done + i] = static_cast<u8>(plain[i] ^ chain[i]);
        std::memcpy(chain, block, 16);
        done += 16;
    }
}

// ---------------------------------------------------------------------------
// SHA-1 / SHA-256 (FIPS 180-4)
// ---------------------------------------------------------------------------

inline u32 rotl32(u32 v, unsigned n) { return static_cast<u32>((v << n) | (v >> (32 - n))); }
inline u32 rotr32(u32 v, unsigned n) { return static_cast<u32>((v >> n) | (v << (32 - n))); }

inline u32 load_be32(const u8* p) {
    return (static_cast<u32>(p[0]) << 24) | (static_cast<u32>(p[1]) << 16) |
           (static_cast<u32>(p[2]) << 8) | static_cast<u32>(p[3]);
}

inline void store_be32(u8* p, u32 v) {
    p[0] = static_cast<u8>(v >> 24);
    p[1] = static_cast<u8>(v >> 16);
    p[2] = static_cast<u8>(v >> 8);
    p[3] = static_cast<u8>(v);
}

inline void store_be64(u8* p, u64 v) {
    store_be32(p, static_cast<u32>(v >> 32));
    store_be32(p + 4, static_cast<u32>(v));
}

struct Sha1State {
    u32 h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
    void block(const u8* p) {
        u32 w[80];
        for (int i = 0; i < 16; ++i) w[i] = load_be32(p + 4 * i);
        for (int i = 16; i < 80; ++i) w[i] = rotl32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

        u32 a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for (int i = 0; i < 80; ++i) {
            u32 f = 0;
            u32 k = 0;
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
            const u32 temp = rotl32(a, 5) + f + e + k + w[i];
            e = d;
            d = c;
            c = rotl32(b, 30);
            b = a;
            a = temp;
        }
        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
    }
};

constexpr u32 kSha256K[64] = {
    0x428A2F98u, 0x71374491u, 0xB5C0FBCFu, 0xE9B5DBA5u, 0x3956C25Bu, 0x59F111F1u, 0x923F82A4u,
    0xAB1C5ED5u, 0xD807AA98u, 0x12835B01u, 0x243185BEu, 0x550C7DC3u, 0x72BE5D74u, 0x80DEB1FEu,
    0x9BDC06A7u, 0xC19BF174u, 0xE49B69C1u, 0xEFBE4786u, 0x0FC19DC6u, 0x240CA1CCu, 0x2DE92C6Fu,
    0x4A7484AAu, 0x5CB0A9DCu, 0x76F988DAu, 0x983E5152u, 0xA831C66Du, 0xB00327C8u, 0xBF597FC7u,
    0xC6E00BF3u, 0xD5A79147u, 0x06CA6351u, 0x14292967u, 0x27B70A85u, 0x2E1B2138u, 0x4D2C6DFCu,
    0x53380D13u, 0x650A7354u, 0x766A0ABBu, 0x81C2C92Eu, 0x92722C85u, 0xA2BFE8A1u, 0xA81A664Bu,
    0xC24B8B70u, 0xC76C51A3u, 0xD192E819u, 0xD6990624u, 0xF40E3585u, 0x106AA070u, 0x19A4C116u,
    0x1E376C08u, 0x2748774Cu, 0x34B0BCB5u, 0x391C0CB3u, 0x4ED8AA4Au, 0x5B9CCA4Fu, 0x682E6FF3u,
    0x748F82EEu, 0x78A5636Fu, 0x84C87814u, 0x8CC70208u, 0x90BEFFFAu, 0xA4506CEBu, 0xBEF9A3F7u,
    0xC67178F2u};

struct Sha256State {
    u32 h[8] = {0x6A09E667u, 0xBB67AE85u, 0x3C6EF372u, 0xA54FF53Au,
                0x510E527Fu, 0x9B05688Cu, 0x1F83D9ABu, 0x5BE0CD19u};
    void block(const u8* p) {
        u32 w[64];
        for (int i = 0; i < 16; ++i) w[i] = load_be32(p + 4 * i);
        for (int i = 16; i < 64; ++i) {
            const u32 s0 = rotr32(w[i - 15], 7) ^ rotr32(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const u32 s1 = rotr32(w[i - 2], 17) ^ rotr32(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }

        u32 a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
        for (int i = 0; i < 64; ++i) {
            const u32 s1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
            const u32 ch = (e & f) ^ ((~e) & g);
            const u32 t1 = hh + s1 + ch + kSha256K[i] + w[i];
            const u32 s0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
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
        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
        h[5] += f;
        h[6] += g;
        h[7] += hh;
    }
};

}  // namespace

void sha1(const u8* data, size_t length, u8 out[20]) {
    Sha1State state;
    const u8* p = data;
    size_t remaining = length;
    while (remaining >= 64) {
        state.block(p);
        p += 64;
        remaining -= 64;
    }

    u8 tail[128] = {};
    if (remaining > 0) std::memcpy(tail, p, remaining);
    tail[remaining] = 0x80;
    const size_t total = (remaining + 1 <= 56) ? 64 : 128;
    store_be64(tail + total - 8, static_cast<u64>(length) * 8);
    for (size_t i = 0; i < total; i += 64) state.block(tail + i);

    for (int i = 0; i < 5; ++i) store_be32(out + 4 * i, state.h[i]);
}

void sha256(const u8* data, size_t length, u8 out[32]) {
    Sha256State state;
    const u8* p = data;
    size_t remaining = length;
    while (remaining >= 64) {
        state.block(p);
        p += 64;
        remaining -= 64;
    }

    u8 tail[128] = {};
    if (remaining > 0) std::memcpy(tail, p, remaining);
    tail[remaining] = 0x80;
    const size_t total = (remaining + 1 <= 56) ? 64 : 128;
    store_be64(tail + total - 8, static_cast<u64>(length) * 8);
    for (size_t i = 0; i < total; i += 64) state.block(tail + i);

    for (int i = 0; i < 8; ++i) store_be32(out + 4 * i, state.h[i]);
}

void hmac_sha256(const u8* key, size_t key_length, const u8* data, size_t length, u8 out[32]) {
    u8 block_key[64] = {};
    if (key_length > 64) {
        sha256(key, key_length, block_key);   // long keys are hashed first (RFC 2104)
    } else if (key_length > 0) {
        std::memcpy(block_key, key, key_length);
    }

    u8 inner_pad[64];
    u8 outer_pad[64];
    for (int i = 0; i < 64; ++i) {
        inner_pad[i] = static_cast<u8>(block_key[i] ^ 0x36);
        outer_pad[i] = static_cast<u8>(block_key[i] ^ 0x5C);
    }

    // SHA256(inner_pad || data): the concatenation is materialised because the
    // hashes we compute this way (SELF metadata digests) are small.
    std::vector<u8> inner;
    inner.reserve(64 + length);
    inner.insert(inner.end(), inner_pad, inner_pad + 64);
    if (length > 0) inner.insert(inner.end(), data, data + length);

    u8 inner_hash[32];
    sha256(inner.data(), inner.size(), inner_hash);

    u8 outer[96];
    std::memcpy(outer, outer_pad, 64);
    std::memcpy(outer + 64, inner_hash, 32);
    sha256(outer, sizeof(outer), out);
}

// ===========================================================================
//  Bignum modular exponentiation (RSA)
// ===========================================================================
//
// Big-endian limbs (limbs[0] is the most significant 32 bit word), leading
// zeros always trimmed. Multiplication is schoolbook, the reduction is a
// bit-by-bit shift-and-subtract long division. This is deliberately simple:
// the loader only performs one RSA-2048 public operation per image.

namespace {

using Limbs = std::vector<u32>;

Limbs limbs_from_bytes(const std::vector<u8>& bytes) {
    const size_t pad = (4 - (bytes.size() % 4)) % 4;
    const size_t total = bytes.size() + pad;
    Limbs out;
    out.reserve(total / 4);
    for (size_t i = 0; i < total; i += 4) {
        u32 value = 0;
        for (size_t j = 0; j < 4; ++j) {
            const size_t index = i + j;
            const u8 byte = index < pad ? 0 : bytes[index - pad];
            value = (value << 8) | byte;
        }
        out.push_back(value);
    }
    while (!out.empty() && out.front() == 0) out.erase(out.begin());
    return out;
}

std::vector<u8> limbs_to_bytes(const Limbs& limbs, size_t width) {
    std::vector<u8> out;
    out.reserve(limbs.size() * 4);
    for (u32 limb : limbs) {
        out.push_back(static_cast<u8>(limb >> 24));
        out.push_back(static_cast<u8>(limb >> 16));
        out.push_back(static_cast<u8>(limb >> 8));
        out.push_back(static_cast<u8>(limb));
    }
    while (!out.empty() && out.front() == 0) out.erase(out.begin());
    if (out.size() < width) out.insert(out.begin(), width - out.size(), 0);
    return out;
}

int compare(const Limbs& a, const Limbs& b) {
    if (a.size() != b.size()) return a.size() < b.size() ? -1 : 1;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) return a[i] < b[i] ? -1 : 1;
    }
    return 0;
}

/// a - b for a >= b.
Limbs subtract(const Limbs& a, const Limbs& b) {
    Limbs r = a;
    std::int64_t borrow = 0;
    for (size_t i = 0; i < r.size(); ++i) {
        const std::int64_t bv = i < b.size() ? static_cast<std::int64_t>(b[b.size() - 1 - i]) : 0;
        std::int64_t diff = static_cast<std::int64_t>(r[r.size() - 1 - i]) - bv - borrow;
        if (diff < 0) {
            diff += (static_cast<std::int64_t>(1) << 32);
            borrow = 1;
        } else {
            borrow = 0;
        }
        r[r.size() - 1 - i] = static_cast<u32>(diff);
    }
    while (!r.empty() && r.front() == 0) r.erase(r.begin());
    return r;
}

Limbs multiply(const Limbs& a, const Limbs& b) {
    if (a.empty() || b.empty()) return {};
    Limbs r(a.size() + b.size(), 0);
    for (size_t i = a.size(); i-- > 0;) {
        u64 carry = 0;
        const u64 av = a[i];
        for (size_t j = b.size(); j-- > 0;) {
            const size_t pos = i + j + 1;
            const u64 cur = static_cast<u64>(r[pos]) + av * b[j] + carry;
            r[pos] = static_cast<u32>(cur);
            carry = cur >> 32;
        }
        size_t pos = i;
        while (carry != 0) {
            const u64 cur = static_cast<u64>(r[pos]) + carry;
            r[pos] = static_cast<u32>(cur);
            carry = cur >> 32;
            if (pos == 0) break;
            --pos;
        }
    }
    while (!r.empty() && r.front() == 0) r.erase(r.begin());
    return r;
}

int bit_length(const Limbs& a) {
    if (a.empty()) return 0;
    u32 top = a[0];
    if (top == 0) return 0;
    int lead = 0;
    while ((top & 0x80000000u) == 0) {
        top <<= 1;
        ++lead;
    }
    return static_cast<int>(a.size()) * 32 - lead;
}

/// Bit `index` of `a` counted from the least significant bit (0 = LSB).
bool bit_from_lsb(const Limbs& a, int index) {
    if (index < 0) return false;
    const size_t limb_index = static_cast<size_t>(index / 32);
    if (limb_index >= a.size()) return false;
    const size_t limb = a.size() - 1 - limb_index;
    const unsigned bit = static_cast<unsigned>(index % 32);
    return ((a[limb] >> bit) & 1u) != 0;
}

/// Bit `index` of `a` counted from the most significant bit *of the value*
/// (0 = MSB of bit_length(a), not of the limb array).
bool bit_from_msb(const Limbs& a, int index) { return bit_from_lsb(a, bit_length(a) - 1 - index); }

Limbs modulo(const Limbs& a, const Limbs& m) {
    if (m.empty()) return {};
    Limbs r;
    const int bits = bit_length(a);
    for (int i = 0; i < bits; ++i) {
        u32 carry = bit_from_msb(a, i) ? 1u : 0u;    // r = r * 2 + bit
        for (size_t k = r.size(); k-- > 0;) {
            const u32 next = r[k] >> 31;
            r[k] = static_cast<u32>((r[k] << 1) | carry);
            carry = next;
        }
        if (carry) r.insert(r.begin(), carry);
        while (!r.empty() && r.front() == 0) r.erase(r.begin());
        if (compare(r, m) >= 0) r = subtract(r, m);
    }
    return r;
}

}  // namespace

std::vector<u8> rsa_public(const std::vector<u8>& modulus, const std::vector<u8>& exponent,
                           const std::vector<u8>& input) {
    // The result keeps the modulus width: the SCE code feeds a full modulus
    // sized block to the Bigmac and callers compare fixed size blobs.
    const size_t width = modulus.size();
    const Limbs n = limbs_from_bytes(modulus);
    const Limbs e = limbs_from_bytes(exponent);
    if (n.empty() || e.empty()) return std::vector<u8>(width, 0);

    Limbs base = modulo(limbs_from_bytes(input), n);
    Limbs result = modulo(Limbs{1}, n);

    // Right-to-left binary exponentiation: at step i, base == input^(2^i), so
    // the LSB-first bit i of the exponent selects it.
    const int bits = bit_length(e);
    for (int i = 0; i < bits; ++i) {
        if (bit_from_lsb(e, i)) result = modulo(multiply(result, base), n);
        if (i + 1 < bits) base = modulo(multiply(base, base), n);
    }
    return limbs_to_bytes(result, width);
}

// ===========================================================================
//  AES public API (keys.h)
// ===========================================================================

void aes128_ecb_encrypt_block(const u8* key, const u8* input, u8* output) {
    AesSchedule schedule;
    aes_expand_key(key, 16, schedule);
    aes_encrypt_block_schedule(schedule, input, output);
}

void aes128_ecb_decrypt_block(const u8* key, const u8* input, u8* output) {
    AesSchedule schedule;
    aes_expand_key(key, 16, schedule);
    aes_decrypt_block_schedule(schedule, input, output);
}

void aes256_ecb_decrypt_block(const u8* key, const u8* input, u8* output) {
    AesSchedule schedule;
    aes_expand_key(key, 32, schedule);
    aes_decrypt_block_schedule(schedule, input, output);
}

void aes128_cbc_decrypt(const u8* key, const u8* iv, const u8* input, size_t length, u8* output) {
    cbc_decrypt_with(key, 16, iv, input, length, output);
}

void aes256_cbc_decrypt(const u8* key, const u8* iv, const u8* input, size_t length, u8* output) {
    cbc_decrypt_with(key, 32, iv, input, length, output);
}

// ===========================================================================
//  Crypto helpers used by the loaders (loader_extra.h)
// ===========================================================================

void aes_ctr_crypt(const u8* key128, const u8* iv, const u8* input, size_t length, u8* output) {
    u8 counter[16] = {};
    if (iv != nullptr) std::memcpy(counter, iv, 16);

    size_t done = 0;
    while (done < length) {
        u8 keystream[16];
        aes128_ecb_encrypt_block(key128, counter, keystream);
        const size_t count = (length - done) < 16 ? (length - done) : 16;
        for (size_t i = 0; i < count; ++i)
            output[done + i] = static_cast<u8>(input[done + i] ^ keystream[i]);
        // 128 bit big-endian increment.
        for (int i = 15; i >= 0; --i) {
            if (++counter[i] != 0) break;
        }
        done += count;
    }
}

void aes_cbc_decrypt_any(const std::vector<u8>& key, const std::vector<u8>& iv, const u8* input,
                         size_t length, u8* output) {
    u8 iv16[16] = {};
    const size_t iv_len = iv.size() < 16 ? iv.size() : 16;
    if (iv_len > 0) std::memcpy(iv16, iv.data(), iv_len);
    cbc_decrypt_with(key.data(), key.size(), iv16, input, length, output);
}

// ===========================================================================
//  SceKeys (keys.h)
// ===========================================================================

SceKeys::SceKeys() {
    // The built-in tables are registered here, so a default constructed SceKeys
    // already carries them (SceKeys.Default() in the C# reference).
    for (size_t i = 0; i < kKeyRowCount; ++i) {
        const KeyRow& row = kKeyRows[i];
        const std::vector<u8> key = hex_to_bytes(row.key_hex);
        const std::vector<u8> iv = hex_to_bytes(row.iv_hex);
        if (key.empty() || iv.empty()) continue;
        keys_[entry_key_name(row.key_type, row.sce_type, row.self_type, static_cast<unsigned>(i))] =
            key;
        keys_[entry_meta_name(row, static_cast<unsigned>(i))] = iv;
    }

    // Named raw keys from the python modules: the eMMC ".enc" blobs use
    // ENC_KEY/ENC_IV (pup_fiction.enc_decrypt) and the prototype key set adds
    // XXX_KEY/XXX_IV.
    set("ENC_KEY", hex_to_bytes("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"));
    set("ENC_IV", hex_to_bytes("AF5F2CB04AC1751ABF51CEF1C8096210"));
    set("XXX_KEY",
        hex_to_bytes("992EF70868DE1B219EC3618FA79DAEC39067FE5638116C29FC0FF7E2A58FBD9E"));
    set("XXX_IV", hex_to_bytes("00000000000000000000000000000000"));
}

const SceKeys& SceKeys::default_keys() {
    static const SceKeys keys;
    return keys;
}

SceKeys SceKeys::load_from_file(const std::string& path) {
    SceKeys keys;   // built-in tables included; a file only adds candidates
    auto data = read_file(path);
    if (!data) return keys;

    const std::string text(reinterpret_cast<const char*>(data->data()), data->size());
    unsigned sequence = 1000;   // after the built-in rows
    int line_number = 0;
    for (const std::string& raw_line : split(text, '\n')) {
        ++line_number;
        std::string line = raw_line;
        const size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = trim(line);
        if (line.empty()) continue;

        // "name = hexbytes" form.
        const size_t equals = line.find('=');
        if (equals != std::string::npos && line.find(':') == std::string::npos) {
            const std::string name = trim(line.substr(0, equals));
            const std::string value = trim(line.substr(equals + 1));
            std::vector<u8> bytes = hex_to_bytes(value);
            if (!name.empty() && !bytes.empty()) keys.set(name, std::move(bytes));
            continue;
        }

        // The 8 field form of the C# reference:
        //   KEYTYPE SCETYPE KEYREV SELFTYPE MINVER MAXVER KEYHEX IVHEX
        std::vector<std::string> fields;
        std::string current;
        for (char c : line) {
            if (c == ' ' || c == '\t' || c == ',' || c == ';') {
                if (!current.empty()) fields.push_back(current);
                current.clear();
            } else {
                current.push_back(c);
            }
        }
        if (!current.empty()) fields.push_back(current);
        if (fields.size() < 8) continue;

        auto as_int = [](const std::string& text_value, int fallback) -> int {
            u64 value = 0;
            if (!parse_u64(text_value, value)) return fallback;
            return static_cast<int>(value);
        };
        auto parse_key_type = [&](const std::string& text_value) -> int {
            const std::string lowered = to_lower(text_value);
            if (lowered == "metadata" || lowered == "meta") return kKeyTypeMetadata;
            if (lowered == "npdrm") return kKeyTypeNpdrm;
            return as_int(text_value, kKeyTypeMetadata);
        };
        auto parse_sce_type = [&](const std::string& text_value) -> int {
            const std::string lowered = to_lower(text_value);
            if (lowered == "self") return kSceTypeSelf;
            if (lowered == "srvk") return kSceTypeSrvk;
            if (lowered == "spkg") return kSceTypeSpkg;
            if (lowered == "dev") return kSceTypeDev;
            return as_int(text_value, kSceTypeSelf);
        };
        auto parse_self_type = [&](const std::string& text_value) -> int {
            const std::string lowered = to_lower(text_value);
            if (lowered == "none") return kSelfTypeNone;
            if (lowered == "kernel") return kSelfTypeKernel;
            if (lowered == "app") return kSelfTypeApp;
            if (lowered == "boot") return kSelfTypeBoot;
            if (lowered == "secure") return kSelfTypeSecure;
            if (lowered == "user") return kSelfTypeUser;
            return as_int(text_value, kSelfTypeNone);
        };
        auto as_version = [](const std::string& text_value, u64 fallback) -> u64 {
            if (text_value == "-") return fallback;
            u64 value = 0;
            if (!parse_u64(text_value, value)) return fallback;
            return value;
        };

        KeyRow row{};
        const std::string source = path_filename(path);
        row.source = source.c_str();
        row.line = line_number;
        row.key_type = parse_key_type(fields[0]);
        row.sce_type = parse_sce_type(fields[1]);
        row.key_rev = as_int(fields[2], 0);
        row.self_type = parse_self_type(fields[3]);
        row.min_ver = as_version(fields[4], 0);
        row.max_ver = as_version(fields[5], kMaxVerAll);
        row.key_hex = fields[6].c_str();
        row.iv_hex = fields[7].c_str();

        const std::vector<u8> key = hex_to_bytes(row.key_hex);
        const std::vector<u8> iv = hex_to_bytes(row.iv_hex);
        if (key.empty() || iv.empty()) continue;
        keys.keys_[entry_key_name(row.key_type, row.sce_type, row.self_type, sequence)] = key;
        keys.keys_[entry_meta_name(row, sequence)] = iv;
        ++sequence;
    }
    return keys;
}

const std::vector<u8>& SceKeys::get(const std::string& name) const {
    static const std::vector<u8> empty;
    const auto it = keys_.find(name);
    return it == keys_.end() ? empty : it->second;
}

bool SceKeys::has(const std::string& name) const { return keys_.find(name) != keys_.end(); }

void SceKeys::set(const std::string& name, std::vector<u8> bytes) { keys_[name] = std::move(bytes); }

const std::vector<u8>& SceKeys::select_self_key(u16 platform, u16 key_revision, u32 sce_type) const {
    static const std::vector<u8> empty;
    // The hardware keyring picks the metadata key for (SCE type, key revision).
    // The emulated keyring has no sys_version/SELF type to filter on, so the
    // first registered entry with the matching key revision wins; a named key
    // ("SELF_KEY_<platform>_<keyrev>_<scetype>" set through set() or a key file)
    // overrides the table.
    const std::string named = format("SELF_KEY_%04X_%u_%u", platform, key_revision, sce_type);
    const auto named_it = keys_.find(named);
    if (named_it != keys_.end() && !named_it->second.empty()) return named_it->second;

    const std::string prefix =
        format("@meta/%d/%02X/", kKeyTypeMetadata, static_cast<int>(sce_type));
    for (const auto& entry : keys_) {
        if (entry.first.compare(0, prefix.size(), prefix) != 0) continue;
        const std::vector<std::string> parts = split(entry.first, '/');
        if (parts.size() < 6) continue;
        if (static_cast<u16>(std::strtol(parts[5].c_str(), nullptr, 10)) != key_revision) continue;
        const std::vector<u8>& key =
            get(format("@key/%d/%02X/%s/%s", kKeyTypeMetadata, static_cast<int>(sce_type),
                       parts[3].c_str(), parts[4].c_str()));
        if (!key.empty()) return key;
    }
    return empty;
}

std::vector<std::string> SceKeys::names() const {
    std::vector<std::string> out;
    out.reserve(keys_.size());
    for (const auto& entry : keys_) out.push_back(entry.first);
    for (const auto& slot : keyring_) out.push_back(format("keyring_%X", slot.first));
    return out;
}

std::string describe_key_entry(const std::string& name) {
    // "@meta/<kt>/<st>/<selft>/<seq>/<keyrev>/<min>/<max>/<source>:<line>"
    const std::vector<std::string> parts = split(name, '/');
    if (parts.size() < 10) return name;
    u64 min_ver = 0;
    u64 max_ver = 0;
    parse_u64(parts[6], min_ver);
    parse_u64(parts[7], max_ver);
    std::string source = parts[8];
    for (size_t i = 9; i < parts.size(); ++i) source += "/" + parts[i];
    return format("%s %s %s keyrev=%s min=0x%llX max=0x%llX (%s)",
                  key_type_name(static_cast<int>(std::strtol(parts[1].c_str(), nullptr, 10))),
                  sce_type_name(static_cast<int>(std::strtol(parts[2].c_str(), nullptr, 16))),
                  self_type_name(static_cast<int>(std::strtol(parts[3].c_str(), nullptr, 16))),
                  parts[5].c_str(), static_cast<unsigned long long>(min_ver),
                  static_cast<unsigned long long>(max_ver), source.c_str());
}

std::vector<std::string> describe_key_table(const SceKeys& keys) {
    std::vector<std::string> out;
    for (const std::string& name : keys.names()) {
        if (name.compare(0, 6, "@meta/") == 0) out.push_back(describe_key_entry(name));
    }
    return out;
}

}  // namespace zlb
