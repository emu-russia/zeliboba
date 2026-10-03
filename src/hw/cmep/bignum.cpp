// zeliboba - Bignum / RSA engine (0xE0040000, mirrored every 0x1000).
//
// ---------------------------------------------------------------------------
// Protocol - the first loader's bignum_op (0x5CE04) and the second loader's
// wrapper (0x4BA20), which agree on every register
// ---------------------------------------------------------------------------
//   64 words from the caller's base blob  -> E0040108..E0040207
//   64 words from the caller's modulus    -> E0040400..E00404FF
//   exponent words (n2 of them)           -> E0040808, one at a time
//   control 0x91000000 | n1 << 18 | n2 << 9 -> E0040800   (0x5CE5C, 0x4BAA0)
//   status                                <- E0040804
//       bit 31 busy, bit 25 error, bit 26 ready, bit 27 wants a word,
//       bits 16..23 result length in words (0x5CE6A/0x5CE7A/0x5CE8E,
//       0x4BAC2/0x4BAA6/0x4BADE/0x4BB32)
//   64 result words                       <- E0040508..E0040607
//   completion semaphore                  -> E0020020 (0x5CF12, 0x4BBB0)
//
// bignum_op loads the base from its `$3` argument (0x5CE06) and the modulus
// from `$4` (0x5CE22) - 0x5E970/0x5E764 in the prototype boot ROM - writes the
// first exponent word (0x5CE50) and then the control word, whose exponent field
// is `stack[0] << 9`, i.e. the number of exponent words (1 for e = 0x00010001).
// The second loader's wrapper is called with a 4-word descriptor
// {modulus pointer, modulus words, exponent pointer, exponent words}: the SMI
// key at 0x4D024 is 64 words of modulus and 0x4D124 holds e = 0x00010001.
//
// The engine itself is implemented with a correct big-endian limb modular
// exponentiation: the windows hold the little-endian word encoding of the big
// integers - the loader's copy helpers reverse the word order and byte-swap
// every word (0x4B5B2 for the inputs, 0x4BB60/0x5CECA for the result), which is
// exactly what read_window()/set_result() invert.
#include <cstring>

#include "common/log.h"
#include "event/providers.h"
#include "hw/cmep/cmep_internal.h"

namespace zlb {
namespace cmep_detail {

namespace {

constexpr size_t kWindowWords = 64;  ///< 512 bits per operand

std::string hex_bytes(const u8* data, size_t length) {
    std::string out;
    out.reserve(length * 2);
    for (size_t i = 0; i < length; ++i) out += format("%02X", data[i]);
    return out;
}

/// Big-endian byte string -> limbs (little-endian 32-bit limbs).
std::vector<u32> to_limbs(const std::vector<u8>& big_endian) {
    std::vector<u32> limbs((big_endian.size() + 3) / 4, 0);
    const size_t n = big_endian.size();
    for (size_t i = 0; i < n; ++i) {
        const u8 byte = big_endian[n - 1 - i];
        limbs[i / 4] |= static_cast<u32>(byte) << (8 * (i % 4));
    }
    return limbs;
}

std::vector<u8> from_limbs(const std::vector<u32>& limbs, size_t bytes) {
    std::vector<u8> out(bytes, 0);
    for (size_t i = 0; i < bytes; ++i) {
        const u32 limb = (i / 4 < limbs.size()) ? limbs[i / 4] : 0;
        out[bytes - 1 - i] = static_cast<u8>((limb >> (8 * (i % 4))) & 0xFF);
    }
    return out;
}

/// Drop non-significant zero limbs.  Without this the schoolbook multiply grows
/// its operands (128, 256, 512 limbs ...) even though the value stays small,
/// which turns the exponentiation into a memory explosion.
std::vector<u32> trim(std::vector<u32> value) {
    while (!value.empty() && value.back() == 0) value.pop_back();
    return value;
}

int compare(const std::vector<u32>& a, const std::vector<u32>& b) {
    const size_t n = a.size() > b.size() ? a.size() : b.size();
    for (size_t i = n; i-- > 0;) {
        const u32 av = i < a.size() ? a[i] : 0;
        const u32 bv = i < b.size() ? b[i] : 0;
        if (av != bv) return av < bv ? -1 : 1;
    }
    return 0;
}

void sub_in_place(std::vector<u32>& a, const std::vector<u32>& b) {
    u64 borrow = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        const u64 bv = (i < b.size() ? b[i] : 0) + borrow;
        const u64 av = a[i];
        a[i] = static_cast<u32>(av - bv);
        borrow = (av < bv) ? 1 : 0;
    }
}

/// out = a * b (schoolbook).
std::vector<u32> mul(const std::vector<u32>& a, const std::vector<u32>& b) {
    std::vector<u32> out(a.size() + b.size(), 0);
    for (size_t i = 0; i < a.size(); ++i) {
        u64 carry = 0;
        for (size_t j = 0; j < b.size(); ++j) {
            const u64 cur = static_cast<u64>(a[i]) * b[j] + out[i + j] + carry;
            out[i + j] = static_cast<u32>(cur);
            carry = cur >> 32;
        }
        size_t k = i + b.size();
        while (carry != 0 && k < out.size()) {
            const u64 cur = static_cast<u64>(out[k]) + carry;
            out[k] = static_cast<u32>(cur);
            carry = cur >> 32;
            ++k;
        }
    }
    return trim(std::move(out));
}

/// Reduces `value` modulo `modulus` with a bit-by-bit division.  Kept here so
/// the block is self-contained: docs/DEVELOPMENT.md has every workstream build
/// in its own directory, so the loader's rsa_public() may not be linked.
///
/// The running remainder is one limb wider than the modulus so that `<<= 1`
/// cannot drop its top bit.  A 2048-bit modulus fills its 64 limbs exactly, so
/// without the extra limb the reduction silently lost the carry on every
/// doubling that overflowed and returned a wrong result.
std::vector<u32> mod_reduce(const std::vector<u32>& raw_value, const std::vector<u32>& modulus) {
    if (modulus.empty()) return {};
    const std::vector<u32> value = trim(raw_value);
    if (compare(value, modulus) < 0) return value;
    std::vector<u32> remainder(modulus.size() + 1, 0);
    // Walk the bits of `value` from the most significant down.
    size_t top = value.size();
    while (top > 0 && value[top - 1] == 0) --top;
    if (top == 0) return remainder;
    const size_t bits = top * 32;
    for (size_t i = bits; i-- > 0;) {
        // remainder <<= 1
        u32 carry = 0;
        for (size_t j = 0; j < remainder.size(); ++j) {
            const u32 next = remainder[j] >> 31;
            remainder[j] = (remainder[j] << 1) | carry;
            carry = next;
        }
        const u32 bit = (value[i / 32] >> (i % 32)) & 1u;
        remainder[0] |= bit;
        if (compare(remainder, modulus) >= 0) sub_in_place(remainder, modulus);
    }
    return remainder;
}

/// Modular exponentiation (square and multiply) over big-endian byte strings.
/// Same semantics as rsa_public() in loader/keys.h; kept local so this
/// translation unit links on its own (docs/DEVELOPMENT.md: each workstream
/// builds in its own directory).
std::vector<u8> local_rsa(const std::vector<u8>& base_be, const std::vector<u8>& exp_be,
                          const std::vector<u8>& mod_be, size_t out_bytes) {
    std::vector<u32> modulus = to_limbs(mod_be);
    while (!modulus.empty() && modulus.back() == 0) modulus.pop_back();
    if (modulus.empty()) return std::vector<u8>(out_bytes, 0);
    std::vector<u32> base = mod_reduce(to_limbs(base_be), modulus);
    std::vector<u32> result(modulus.size(), 0);
    result[0] = 1;

    // Square-and-multiply, big-endian (most significant byte first).  Leading
    // zero bytes are skipped; the first byte is processed bit by bit from its
    // most significant set bit, so an all-zero exponent returns 1 immediately.
    size_t first = exp_be.size();
    for (size_t i = 0; i < exp_be.size(); ++i) {
        if (exp_be[i] != 0) {
            first = i;
            break;
        }
    }
    for (size_t i = first; i < exp_be.size(); ++i) {
        const u8 byte = exp_be[i];
        for (int bit = 7; bit >= 0; --bit) {
            result = mod_reduce(mul(result, result), modulus);
            if ((byte >> bit) & 1u) result = mod_reduce(mul(result, base), modulus);
        }
    }
    return from_limbs(result, out_bytes);
}

}  // namespace

/// The engine used unless a host substitutes one.  Identical to rsa_public()
/// from loader/keys.h; the local copy keeps this block linkable by itself.
std::vector<u8> bignum_default_engine(const std::vector<u8>& modulus,
                                      const std::vector<u8>& exponent,
                                      const std::vector<u8>& input) {
    // 512-bit operand window, matching the 64-word windows the loader programs.
    const size_t out_bytes = modulus.size() > 64 ? modulus.size() : 64;
    return local_rsa(input, exponent, modulus, out_bytes);
}

BignumDevice::BignumDevice(Bus& bus, CmepBlock& owner)
    : Device("CMeP.Bignum", kBaseAddr, kSizeAddr), bus_(bus), owner_(owner) {
    reset();
}

void BignumDevice::reset() {
    words_.clear();
    result_.assign(kWindowWords * 4, 0);
    port_words_.clear();
    control_ = 0;
    mod_words_ = 0;
    exp_words_ = 0;
    result_words_ = 0;
    operations_ = 0;
    error_ = false;
    busy_ = false;
    want_word_ = false;
}

const char* BignumDevice::register_name(u32 address) const {
    const u32 window = address & ~(kMirrorStride - 1);
    const u32 offset = address - window;
    if (offset == 0x800) return "Bignum control (0x91000000|n1<<18|n2<<9)";
    if (offset == 0x804) return "Bignum status (bit31 busy, 25 err, 26 ready, 27 want, 16..23 len)";
    if (offset == 0x808) return "Bignum exponent word";
    if (offset >= 0x508 && offset < 0x608) return "Bignum output word";
    if (offset >= 0x400 && offset < 0x500) return "Bignum modulus word";
    if (offset >= 0x108 && offset < 0x208) return "Bignum base/input word";
    if (offset < 0x100) return "Bignum window word";
    return nullptr;
}

void BignumDevice::enumerate_registers(std::vector<RegisterInfo>& out) const {
    static const u32 offsets[] = {0x800, 0x804, 0x808, 0x508, 0x100, 0x000, 0x400};
    for (u32 offset : offsets) {
        RegisterInfo info;
        info.address = kBaseAddr + offset;
        const char* name = register_name(info.address);
        info.name = name ? name : "Bignum";
        info.reset_value = 0;
        out.push_back(info);
    }
}

/// Reads a 64-word window as a little-endian byte string and returns it in
/// big-endian order, which is what the modular exponentiation consumes.  The
/// engine's own byte-swap loop (0x5CECA..0x5CF04) shows the register words are
/// little-endian with word 0 the least significant one, and the loader copies
/// the operands in with `lw` (0x5CE16/0x5CE32), so the memory blob is the
/// little-endian encoding of the big integer.
std::vector<u8> BignumDevice::read_window(u32 base) const {
    std::vector<u8> little(kWindowWords * 4, 0);
    for (u32 word = 0; word < kWindowWords; ++word) {
        auto it = words_.find(base + 4 * word);
        const u64 value = (it == words_.end()) ? 0 : it->second;
        little[4 * word + 0] = static_cast<u8>(value & 0xFF);
        little[4 * word + 1] = static_cast<u8>((value >> 8) & 0xFF);
        little[4 * word + 2] = static_cast<u8>((value >> 16) & 0xFF);
        little[4 * word + 3] = static_cast<u8>((value >> 24) & 0xFF);
    }
    return std::vector<u8>(little.rbegin(), little.rend());
}

std::vector<u8> BignumDevice::modulus() const { return read_window(kModulus); }

std::vector<u8> BignumDevice::exponent() const {
    // The loaders stream the exponent one word at a time through 0xE0040808,
    // byte-swapping each word on the way in (0x4BAE2..0x4BB1E), so the engine
    // sees a sequence of big-endian words, most significant first: the exponent
    // blob is a big-endian byte string (0x4D124 holds `00 01 00 01` = 65537) and
    // the stream walks it forwards.  The value is right-aligned in the 64-word
    // operand buffer, like every other big integer the engine handles.  The first
    // loader writes word 0 before the control store (0x5CE50), which is why the
    // stream lives outside the operation.
    const size_t window_bytes = kWindowWords * 4;
    std::vector<u8> stream;
    stream.reserve(port_words_.size() * 4);
    for (u32 word : port_words_) {
        stream.push_back(static_cast<u8>((word >> 24) & 0xFF));
        stream.push_back(static_cast<u8>((word >> 16) & 0xFF));
        stream.push_back(static_cast<u8>((word >> 8) & 0xFF));
        stream.push_back(static_cast<u8>(word & 0xFF));
    }
    if (stream.size() > window_bytes) stream.erase(stream.begin(), stream.end() - window_bytes);
    std::vector<u8> big(window_bytes - stream.size(), 0);
    big.insert(big.end(), stream.begin(), stream.end());
    return big;
}

void BignumDevice::set_result(const std::vector<u8>& big_endian) {
    result_ = big_endian;
    while (result_.size() < kWindowWords * 4) result_.push_back(0);
    // The window holds the little-endian encoding of the result - exactly the
    // inverse of read_window(), which reverse()s the window to get the
    // big-endian operand.  The loader then reverses it once more while copying it
    // out (0x5CECA..0x5CF04 byte-swaps every word and stores it at
    // destination + 252 - 4*i), so the buffer at 0x5C83A's $1 ends up holding the
    // big-endian block the memcmps at 0x5C862..0x5C898 expect.
    std::vector<u8> little(result_.rbegin(), result_.rend());
    for (u32 word = 0; word < kWindowWords; ++word) {
        const size_t index = 4 * static_cast<size_t>(word);
        words_[kOutput + 4 * word] = static_cast<u32>(little[index]) |
                                     (static_cast<u32>(little[index + 1]) << 8) |
                                     (static_cast<u32>(little[index + 2]) << 16) |
                                     (static_cast<u32>(little[index + 3]) << 24);
    }
}

/// Run the engine and publish its output.  Called as soon as the exponent
/// stream is complete - the hardware takes a few hundred cycles, but the model
/// is synchronous and only has to keep the register handshake truthful.
void BignumDevice::finish_operation() {
    const std::vector<u8> mod = modulus();
    const std::vector<u8> exp = exponent();
    const std::vector<u8> base = read_window(kInput);
    const std::vector<u8> output =
        engine_ ? engine_(mod, exp, base) : bignum_default_engine(mod, exp, base);
    ZLB_LOG_DBG("bignum", "n1=%u n2=%u mod=..%s exp=%s base=..%s -> ..%s", mod_words_, exp_words_,
                hex_bytes(mod.data() + mod.size() - 8, 8).c_str(), hex_bytes(exp.data(), 8).c_str(),
                hex_bytes(base.data() + base.size() - 8, 8).c_str(),
                hex_bytes(output.data() + output.size() - 8, 8).c_str());
    // Only a genuine engine failure raises bit 25 (0x02000000): the loaders turn
    // it into an error return (0x5CEBA..0x5CEC6, 0x4BB8E).  A zero modulus (the
    // state before the loader has programmed anything) is not an error - the
    // engine simply has nothing to compute and reports an all-zero result.
    error_ = output.empty() && !mod.empty();
    set_result(output);
    // The result is a full 64-word window; the loaders check that the length the
    // engine reports (status bits 16..23) equals the modulus word count
    // (0x4BB3E) and reject anything longer than 66 words.
    result_words_ = mod_words_ != 0 ? mod_words_ : static_cast<u32>(kWindowWords);
    busy_ = false;
    want_word_ = false;
    // E0020020 is the completion semaphore: engine_wait (0x4BBB0) spins until it
    // is non-zero and clears it by writing the value back, and the next
    // operation reports "busy" (0x800F0010) until that has happened.
    owner_.flags_device().set_work_state(1);
    port_words_.clear();
    if (events().should_record(EventProvider::Cmep, EventLevel::Verbose, event_keyword::kSecurity)) {
        events().event(EventProvider::Cmep, ev::cmep::kBignum)
            .field("op", (u64)1)
            .field("bits", (u64)mod_words_ * 32u)
            .emit();
    }
}

u64 BignumDevice::read(u32 address, unsigned size) {
    const u32 window = address & ~(kMirrorStride - 1);
    const u32 offset = address - window;
    u64 value = 0;
    if (offset == 0x804) {
        // bignum_op's loop (0x5CE60..0x5CEAE) and the second loader's wrapper
        // (0x4BAA6/0x4BAC2) read the same bits: see the class comment.
        u32 status = (result_words_ & 0xFFu) << 16;
        if (busy_) status |= kStatusBusy;
        if (error_) status |= kStatusError;
        if (!busy_ || want_word_) status |= kStatusReady;
        if (want_word_) status |= kStatusFetch;
        value = status;
    } else if (offset == 0x800 || offset == 0x808) {
        auto it = words_.find(window + offset);
        value = (it == words_.end()) ? 0 : it->second;
    } else {
        auto it = words_.find(window + offset);
        value = (it == words_.end()) ? 0 : it->second;
    }
    return value & (size >= 8 ? ~0ull : ((1ull << (8 * size)) - 1));
}

void BignumDevice::write(u32 address, unsigned size, u64 value) {
    const u32 window = address & ~(kMirrorStride - 1);
    const u32 offset = address - window;
    const u32 v = static_cast<u32>(value);
    if (offset == 0x808) {
        // Every word handed over here is an exponent word: the first loader
        // writes word 0 *before* the control store (0x5CE50), the second loader
        // feeds them from the descriptor (0x4BAF6) as the engine asks for them.
        words_[kData] = v;
        port_words_.push_back(v);
        if (busy_ && port_words_.size() >= exp_words_) finish_operation();
        return;
    }
    if (offset == 0x800) {
        // 0x5CE5C / 0x4BAA0: control = 0x91000000 | (modulus words << 18) |
        // (exponent words << 9).  This is the single "start" store; from here on
        // the loaders only poll the status word.
        words_[kControl] = v;
        control_ = v;
        mod_words_ = (v >> 18) & 0x3FFu;
        exp_words_ = (v >> 9) & 0x1FFu;
        ++operations_;
        error_ = false;
        result_words_ = 0;
        busy_ = true;
        want_word_ = false;
        if (port_words_.size() >= exp_words_) {
            // A one-word exponent can already be in the port (first loader).
            finish_operation();
        } else {
            want_word_ = true;  // bits 26/27: "ready, hand me the next word"
        }
        return;
    }
    if (offset == 0x804) {
        words_[window + offset] = v;
        return;
    }
    // The result window is 64 words: 0xE0040508..0xE0040607.
    if (offset >= 0x508 && offset < 0x608) {
        words_[kOutput + offset - 0x508] = v;
        return;
    }
    words_[window + offset] = v;
}

std::string BignumDevice::summary() const {
    return format("ops=%llu busy=%d error=%d n=%u/%u words=%u", static_cast<unsigned long long>(operations_),
                  busy_ ? 1 : 0, error_ ? 1 : 0, mod_words_, exp_words_, result_words_);
}

void BignumDevice::describe(std::vector<std::string>& lines) const {
    const std::vector<u8> mod = modulus();
    lines.push_back(format("Bignum modulus      = %s", hex_bytes(mod.data(), 8).c_str()));
    lines.push_back(format("Bignum operations   = %llu (error=%d)",
                           static_cast<unsigned long long>(operations_), error_ ? 1 : 0));
    lines.push_back(format("Bignum result[0..8] = %s", hex_bytes(result_.data(), 8).c_str()));
}

// ---------------------------------------------------------------------------
// Save state
// ---------------------------------------------------------------------------
// The word map holds every register window (base, modulus, control, status and
// result words) and the port stream/result copy hold the operation in flight,
// so all of it travels. `engine_`, `bus_` and `owner_` are wiring.

void BignumDevice::save_state(StateWriter& writer) const {
    writer.map(words_, [&](u32 address, u64 value) {
        writer.put_u32(address);
        writer.put_u64(value);
    });
    writer.list(result_, [&](u8 byte) { writer.put_u8(byte); });
    writer.list(port_words_, [&](u32 word) { writer.put_u32(word); });
    writer.put_u32(control_);
    writer.put_u32(mod_words_);
    writer.put_u32(exp_words_);
    writer.put_u32(result_words_);
    writer.put_u64(operations_);
    writer.put_bool(error_);
    writer.put_bool(busy_);
    writer.put_bool(want_word_);
}

void BignumDevice::load_state(StateReader& reader) {
    reader.map(words_, [&](u32& address, u64& value) {
        address = reader.get_u32();
        value = reader.get_u64();
    });
    reader.list(result_, [&](u8& byte) { byte = reader.get_u8(); });
    reader.list(port_words_, [&](u32& word) { word = reader.get_u32(); });
    control_ = reader.get_u32();
    mod_words_ = reader.get_u32();
    exp_words_ = reader.get_u32();
    result_words_ = reader.get_u32();
    operations_ = reader.get_u64();
    error_ = reader.get_bool();
    busy_ = reader.get_bool();
    want_word_ = reader.get_bool();
}

}  // namespace cmep_detail
}  // namespace zlb
