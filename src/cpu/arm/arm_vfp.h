// zeliboba - VFPv3-D16 floating point unit for the ARM Cortex-A9 (ARMv7-A).
//
// Implements:
//   * the 32 x 64-bit register file (16 double / 32 single registers, d0..d15)
//   * FPSCR with the NZCV flags and the cumulative exception bits, plus FPEXC
//   * VADD/VMUL/VNMUL/VSUB/VDIV/VABS/VNEG/VSQRT/VMLA/VMLS/VNMLA/VNMLS
//   * VCMP/VCMPE including the "#0.0" forms
//   * VCVT between single/double and int/uint
//   * VMOV (immediate, core register <-> single, two singles <-> double)
//   * VMRS/VMSR
//
// The register file is kept as a u32[32] of single-precision slots so both views
// (sN and dN) are addressable without allocation: dN occupies slots 2N / 2N+1.
#pragma once

#include <string>

#include "common/types.h"

namespace zlb {

class ArmVfp {
public:
    // ---- register file ----------------------------------------------------

    /// 32 double registers as 64 single-precision slots. The VFPv3-D16 *view*
    /// only names s0-s31 / d0-d15 (those are the encodings VFP can produce), but
    /// the Cortex-A9 in the PS Vita also has Advanced SIMD, and NEON shares this
    /// register file and can name d0-d31, so the storage covers all of it.
    u32 regs[64] = {};

    /// FPSCR - floating point status and control register.
    u32 fpscr = 0;
    /// FPEXC - floating point exception register (bit 30 = EN).
    u32 fpexc = 0;
    /// FPSID (Cortex-A9 VFPv3-D16).
    u32 fpsid = 0x41033090;
    /// MVFR0 - media and VFP feature register 0.
    u32 mvfr0 = 0x10110221;
    /// MVFR1 - media and VFP feature register 1.
    u32 mvfr1 = 0x12000011;

    static constexpr u32 kFpexcEn = 1u << 30;

    // FPSCR flag bits.
    static constexpr u32 kNs = 1u << 31;
    static constexpr u32 kZs = 1u << 30;
    static constexpr u32 kCs = 1u << 29;
    static constexpr u32 kVs = 1u << 28;

    void reset();

    bool enabled() const { return (fpexc & kFpexcEn) != 0; }

    // ---- register access --------------------------------------------------

    u32 read_s(int n) const { return regs[n & 31]; }
    void write_s(int n, u32 v) { regs[n & 31] = v; }

    /// VFP view (VFPv3-D16: d0-d15).
    u64 read_d(int n) const { return read_d32(n & 15); }
    void write_d(int n, u64 v) { write_d32(n & 15, v); }

    /// Full 32-register view used by Advanced SIMD (NEON addresses d0-d31).
    u64 read_d32(int n) const {
        const int i = (n & 31) * 2;
        return static_cast<u64>(regs[i]) | (static_cast<u64>(regs[i + 1]) << 32);
    }
    void write_d32(int n, u64 v) {
        const int i = (n & 31) * 2;
        regs[i] = static_cast<u32>(v);
        regs[i + 1] = static_cast<u32>(v >> 32);
    }

    f32 read_f32(int n) const;
    void write_f32(int n, f32 v);
    f64 read_f64(int n) const;
    void write_f64(int n, f64 v);

    static u32 float_to_bits(f32 f);
    static f32 bits_to_float(u32 bits);

    // ---- FPSCR flags ------------------------------------------------------

    /// Copy the FPSCR NZCV bits into the CPSR NZCV bits (ARM ARM A2.5).
    u32 flags_to_cpsr(u32 cpsr) const {
        return (cpsr & ~0xF0000000u) | ((fpscr >> 28) << 28);
    }
    /// Copy the CPSR NZCV bits into the FPSCR.
    void cpsr_to_flags(u32 cpsr) { fpscr = (fpscr & 0x0FFFFFFFu) | (cpsr & 0xF0000000u); }

    void set_flags(bool n, bool z, bool c, bool v);
    void set_flags_double(f64 d);
    void set_flags_single(f32 f);

    // ---- conversions ------------------------------------------------------

    /// VCVT integer -> floating point honouring FPSCR.RMode.
    f32 int_to_float(u32 value, bool is_unsigned);
    /// VCVT floating point -> integer honouring FPSCR.RMode.
    u32 float_to_int(f64 value, bool is_unsigned);

    // ---- flag-updating arithmetic -----------------------------------------

    f64 add_double(f64 a, f64 b);
    f64 sub_double(f64 a, f64 b);
    f64 mul_double(f64 a, f64 b);
    f64 div_double(f64 a, f64 b);
    f64 sqrt_double(f64 a);

    f32 add_single(f32 a, f32 b);
    f32 sub_single(f32 a, f32 b);
    f32 mul_single(f32 a, f32 b);
    f32 div_single(f32 a, f32 b);
    f32 sqrt_single(f32 a);

    /// VCMP/VCMPE: sets FPSCR NZCV and returns 0 = lt, 1 = eq, 2 = gt, 3 = unordered.
    int compare(f64 a, f64 b, bool quiet);

    // ---- immediate expansion ----------------------------------------------

    /// VFPExpandImm for single precision (ARM ARM A2.7.8).
    static u32 expand_immediate(u32 imm8);
    /// VFPExpandImm for double precision.
    static u64 expand_immediate64(u32 imm8);

    std::string describe() const;

private:
    void set_ioc();
    void set_ixc();
    void set_ofc();
    void set_dzc();
};

}  // namespace zlb
