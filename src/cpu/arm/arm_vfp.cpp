#include "cpu/arm/arm_vfp.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace zlb {

namespace {

bool is_negative_zero(f64 d) { return d == 0.0 && std::signbit(d); }
bool is_negative_zero_single(f32 f) { return f == 0.0f && std::signbit(f); }

}  // namespace

void ArmVfp::reset() {
    for (u32& slot : regs) slot = 0;
    fpscr = 0;
    fpexc = 0;
    fpsid = 0x41033090;
    mvfr0 = 0x10110221;
    mvfr1 = 0x12000011;
}

// ---------------------------------------------------------------------------
// Bit reinterpretation
// ---------------------------------------------------------------------------

u32 ArmVfp::float_to_bits(f32 f) {
    u32 bits = 0;
    std::memcpy(&bits, &f, sizeof(bits));
    return bits;
}

f32 ArmVfp::bits_to_float(u32 bits) {
    f32 f = 0.0f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

f32 ArmVfp::read_f32(int n) const { return bits_to_float(regs[n & 31]); }
void ArmVfp::write_f32(int n, f32 v) { regs[n & 31] = float_to_bits(v); }

f64 ArmVfp::read_f64(int n) const {
    const u64 bits = read_d(n);
    f64 d = 0.0;
    std::memcpy(&d, &bits, sizeof(d));
    return d;
}

void ArmVfp::write_f64(int n, f64 v) {
    u64 bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    write_d(n, bits);
}

// ---------------------------------------------------------------------------
// FPSCR flag handling
// ---------------------------------------------------------------------------

void ArmVfp::set_flags(bool n, bool z, bool c, bool v) {
    fpscr &= 0x0FFFFFFFu;
    if (n) fpscr |= kNs;
    if (z) fpscr |= kZs;
    if (c) fpscr |= kCs;
    if (v) fpscr |= kVs;
}

void ArmVfp::set_flags_double(f64 d) {
    if (std::isnan(d)) {
        set_flags(false, false, true, true);
        return;
    }
    set_flags(d < 0.0, d == 0.0, false, false);
}

void ArmVfp::set_flags_single(f32 f) {
    if (std::isnan(f)) {
        set_flags(false, false, true, true);
        return;
    }
    set_flags(f < 0.0f, f == 0.0f, false, false);
}

void ArmVfp::set_ioc() {
    fpscr |= 1u;       // IOC
    fpscr |= 1u << 7;  // IDC (input denormal) - deliberately conservative
}

void ArmVfp::set_ixc() { fpscr |= 1u << 4; }
void ArmVfp::set_ofc() {
    fpscr |= 1u << 2;  // OFC
    fpscr |= 1u << 3;  // UFC
}
void ArmVfp::set_dzc() { fpscr |= 1u << 1; }

// ---------------------------------------------------------------------------
// Conversions
// ---------------------------------------------------------------------------

f32 ArmVfp::int_to_float(u32 value, bool is_unsigned) {
    const f64 d =
        is_unsigned ? static_cast<f64>(value) : static_cast<f64>(static_cast<s32>(value));
    switch (fpscr & (3u << 22)) {
        case 1u << 22: return static_cast<f32>(std::ceil(d));     // RP
        case 2u << 22: return static_cast<f32>(std::floor(d));    // RM
        case 3u << 22: return static_cast<f32>(std::trunc(d));    // RZ
        default: return static_cast<f32>(d);                      // RN
    }
}

u32 ArmVfp::float_to_int(f64 value, bool is_unsigned) {
    f64 r;
    switch (fpscr & (3u << 22)) {
        case 1u << 22: r = std::floor(value + 0.5); break;   // RP
        case 2u << 22: r = std::ceil(value - 0.5); break;    // RM
        case 3u << 22: r = std::trunc(value); break;         // RZ
        default: r = std::nearbyint(value); break;           // RN (ties to even)
    }

    if (std::isnan(value)) {
        set_ioc();
        set_flags(false, false, true, true);
        return 0;
    }

    if (is_unsigned) {
        if (r < 0.0) {
            set_ioc();
            set_flags(false, false, true, true);
            return 0;
        }
        if (r > 4294967295.0) {
            set_ioc();
            set_flags(false, false, true, true);
            return 0xFFFFFFFFu;
        }
        return static_cast<u32>(r);
    }

    if (r < -2147483648.0) {
        set_ioc();
        set_flags(false, false, true, true);
        return 0x80000000u;
    }
    if (r > 2147483647.0) {
        set_ioc();
        set_flags(false, false, true, true);
        return 0x7FFFFFFFu;
    }
    return static_cast<u32>(static_cast<s32>(r));
}

// ---------------------------------------------------------------------------
// Double precision arithmetic
// ---------------------------------------------------------------------------

f64 ArmVfp::add_double(f64 a, f64 b) {
    const f64 r = a + b;
    set_flags_double(r);
    return r;
}

f64 ArmVfp::sub_double(f64 a, f64 b) {
    const f64 r = a - b;
    set_flags_double(r);
    return r;
}

f64 ArmVfp::mul_double(f64 a, f64 b) {
    const f64 r = a * b;
    set_flags_double(r);
    return r;
}

f64 ArmVfp::div_double(f64 a, f64 b) {
    if (b == 0.0) {
        if (a == 0.0) {
            set_ioc();
            set_flags(false, false, true, true);
            return std::numeric_limits<f64>::quiet_NaN();
        }
        set_dzc();
        const bool neg = (a < 0.0) != is_negative_zero(b);
        set_flags(neg, false, false, false);
        return neg ? -std::numeric_limits<f64>::infinity() : std::numeric_limits<f64>::infinity();
    }
    const f64 r = a / b;
    set_flags_double(r);
    return r;
}

f64 ArmVfp::sqrt_double(f64 a) {
    if (a < 0.0) {
        set_ioc();
        set_flags(false, false, true, true);
        return std::numeric_limits<f64>::quiet_NaN();
    }
    const f64 r = std::sqrt(a);
    set_flags_double(r);
    return r;
}

// ---------------------------------------------------------------------------
// Single precision arithmetic
// ---------------------------------------------------------------------------

f32 ArmVfp::add_single(f32 a, f32 b) {
    const f32 r = a + b;
    set_flags_single(r);
    return r;
}

f32 ArmVfp::sub_single(f32 a, f32 b) {
    const f32 r = a - b;
    set_flags_single(r);
    return r;
}

f32 ArmVfp::mul_single(f32 a, f32 b) {
    const f32 r = a * b;
    set_flags_single(r);
    return r;
}

f32 ArmVfp::div_single(f32 a, f32 b) {
    if (b == 0.0f) {
        if (a == 0.0f) {
            set_ioc();
            set_flags(false, false, true, true);
            return std::numeric_limits<f32>::quiet_NaN();
        }
        set_dzc();
        const bool neg = (a < 0.0f) != is_negative_zero_single(b);
        set_flags(neg, false, false, false);
        return neg ? -std::numeric_limits<f32>::infinity() : std::numeric_limits<f32>::infinity();
    }
    const f32 r = a / b;
    set_flags_single(r);
    return r;
}

f32 ArmVfp::sqrt_single(f32 a) {
    if (a < 0.0f) {
        set_ioc();
        set_flags(false, false, true, true);
        return std::numeric_limits<f32>::quiet_NaN();
    }
    const f32 r = std::sqrt(a);
    set_flags_single(r);
    return r;
}

// ---------------------------------------------------------------------------
// Comparison
// ---------------------------------------------------------------------------

int ArmVfp::compare(f64 a, f64 b, bool quiet) {
    if (std::isnan(a) || std::isnan(b)) {
        if (!quiet) set_ioc();
        set_flags(false, false, true, true);
        return 3;
    }
    if (a < b) {
        set_flags(true, false, false, false);
        return 0;
    }
    if (a == b) {
        set_flags(false, true, true, false);
        return 1;
    }
    set_flags(false, false, true, false);
    return 2;
}

// ---------------------------------------------------------------------------
// Immediate expansion (ARM ARM A2.7.8 VFPExpandImm)
// ---------------------------------------------------------------------------

u32 ArmVfp::expand_immediate(u32 imm8) {
    const u32 b = (imm8 >> 6) & 1u;
    const u32 sign = (imm8 >> 7) & 1u;
    const u32 exp = (b == 0u) ? (0x80u | ((imm8 >> 4) & 3u)) : (0x7Cu | ((imm8 >> 4) & 3u));
    return (sign << 31) | (exp << 23) | ((imm8 & 0xFu) << 19);
}

u64 ArmVfp::expand_immediate64(u32 imm8) {
    const u32 b = (imm8 >> 6) & 1u;
    const u64 sign = (imm8 >> 7) & 1u;
    const u64 exp = (b == 0u) ? (0x400u | ((imm8 >> 4) & 3u)) : (0x3FCu | ((imm8 >> 4) & 3u));
    return (sign << 63) | (exp << 52) | (static_cast<u64>(imm8 & 0xFu) << 48);
}

std::string ArmVfp::describe() const {
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), "VFP %s FPSCR=0x%08X FPEXC=0x%08X",
                  enabled() ? "on" : "off", fpscr, fpexc);
    return std::string(buffer);
}

}  // namespace zlb
