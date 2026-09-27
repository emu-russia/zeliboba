// zeliboba - RL78 disassembler (see rl78_disasm.h).
#include "cpu/rl78/rl78_disasm.h"

#include <string>

#include "common/util.h"

namespace zlb {
namespace {

const char* const kConditionNames[8] = {"t", "f", "c", "nc", "h", "nh", "z", "nz"};

/// CLR1 PSW.7 / SET1 PSW.7 are the DI / EI aliases; the reference printer does
/// the same substitution through rl78->syntax.
bool is_psw_bit7(const Rl78Operand& op) {
    return op.type == Rl78OpType::BitInd && op.reg == Rl78Reg::None &&
           op.address == kRl78SfrPsw && (op.bit & 7) == 7;
}

void append_immediate(std::string& out, const Rl78Operand& op, bool address_modifier,
                      bool sfr_modifier) {
    (void)sfr_modifier;
    switch (op.add_kind) {
        case kRl78AkField:
        case kRl78AkLit:
            // Shift counts ("sar a, 4") and SEL bank numbers ("rb0").
            out += format("%d", op.addend);
            return;
        default:
            if (address_modifier || op.add_kind == kRl78AkImmu3) {
                // Branch / call target (20-bit code space).
                out += format("0x%05X", static_cast<u32>(op.addend) & 0xFFFFFu);
                return;
            }
            break;
    }
    const int digits = op.size == 1 ? 2 : op.size == 3 ? 6 : 4;
    out += format("0x%0*X", digits, static_cast<u32>(op.addend) & 0xFFFFu);
}

void append_address(std::string& out, const Rl78Operand& op, bool bang, bool es_modifier,
                    bool sfr_modifier, bool word) {
    if (es_modifier && op.es) out += "es:";

    if (op.reg != Rl78Reg::None) {
        const Rl78Reg reg = op.reg;
        if (reg == Rl78Reg::B || reg == Rl78Reg::C || reg == Rl78Reg::BC) {
            // Renesas syntax: <base>[<reg>].
            out += format("0x%04X", static_cast<u32>(op.addend) & 0xFFFFu);
            out += "[";
            out += rl78_reg_name(reg);
            out += "]";
            return;
        }
        out += "[";
        out += rl78_reg_name(reg);
        if (op.reg2 != Rl78Reg::None) {
            out += "+";
            out += rl78_reg_name(op.reg2);
        }
        if (op.addend != 0) out += format("+0x%02X", static_cast<u32>(op.addend) & 0xFFu);
        out += "]";
        return;
    }

    // Direct / short-direct / SFR.
    const u32 address = op.address & 0xFFFFFu;
    switch (op.add_kind) {
        case kRl78AkCallt:
            out += format("0x%04X", address);
            return;
        case kRl78AkSfr:
        case kRl78AkSaddr:
            break;
        default:
            // !addr16: the data page lives in ES, so show the raw address.
            if (bang) out += "!";
            out += format("0x%04X", address & 0xFFFFu);
            return;
    }

    if (address >= 0xFFE20u) {
        if (sfr_modifier && !bang) {
            if (const char* name = rl78_sfr_name(address, word)) {
                out += name;
                return;
            }
        }
        out += format("0x%05X", address);
        return;
    }
    if (bang) out += "!";
    out += format("0x%04X", address & 0xFFFFu);
}

void append_operand(std::string& out, const Rl78Operand& op, bool bang, bool es_modifier,
                    bool address_modifier, bool sfr_modifier, bool word) {
    switch (op.type) {
        case Rl78OpType::None:
            return;
        case Rl78OpType::PreDec:
            out += "[--";
            out += rl78_reg_name(op.reg);
            out += "]";
            return;
        case Rl78OpType::PostInc:
            out += "[";
            out += rl78_reg_name(op.reg);
            out += "++]";
            return;
        case Rl78OpType::Imm:
            append_immediate(out, op, address_modifier, sfr_modifier);
            return;
        case Rl78OpType::Reg:
            if (op.reg == Rl78Reg::None) return;
            out += rl78_reg_name(op.reg);
            return;
        case Rl78OpType::Bit:
            if (op.reg == Rl78Reg::None) return;
            out += rl78_reg_name(op.reg);
            out += ".";
            out += format("%d", op.bit & 7);
            return;
        case Rl78OpType::Ind:
        case Rl78OpType::BitInd:
            append_address(out, op, bang, es_modifier, sfr_modifier, word);
            if (op.type == Rl78OpType::BitInd) {
                out += ".";
                out += format("%d", op.bit & 7);
            }
            return;
        default:
            return;
    }
}

}  // namespace

const char* rl78_sfr_name(u32 address, bool word) {
    if (address == kRl78SfrPsw) return word ? nullptr : "psw";
    if (address == kRl78SfrSp) return word ? "sp" : "spl";
    if (address == kRl78SfrSp + 1) return word ? nullptr : "sph";
    if (address == kRl78SfrCs) return word ? nullptr : "cs";
    if (address == kRl78SfrEs) return word ? nullptr : "es";
    if (address == kRl78SfrPmc) return word ? nullptr : "pmc";
    if (address == kRl78SfrMem) return word ? nullptr : "mem";
    return nullptr;
}

std::string rl78_format_instruction(const Rl78Decoded& insn) {
    if (insn.row < 0) return "??";
    const char* syntax = insn.syntax;
    if (syntax == nullptr || *syntax == '\0') return "??";

    if ((std::string(insn.mnemonic) == "clr1" || std::string(insn.mnemonic) == "set1") &&
        is_psw_bit7(insn.ops[0])) {
        return std::string(insn.mnemonic) == "clr1" ? "di" : "ei";
    }

    std::string out;
    out.reserve(40);
    const std::string template_text(syntax);
    for (size_t i = 0; i < template_text.size(); ++i) {
        const char character = template_text[i];
        if (character == '\t') {
            out += ' ';
            continue;
        }
        if (character != '%') {
            out += character;
            continue;
        }

        bool bang = false;
        bool es_modifier = false;
        bool address_modifier = false;
        bool sfr_modifier = false;
        bool condition = false;
        size_t j = i + 1;
        while (j < template_text.size()) {
            const char modifier = template_text[j];
            // '%x' is accepted but has no effect: the only template that uses it
            // is "callt [%x0]" and a CALLT operand is an address, not an
            // immediate, so the reference printer ignores it too.
            if (modifier == 'x') {
                // consumed
            } else if (modifier == '!') {
                bang = true;
            } else if (modifier == 'e') {
                es_modifier = true;
            } else if (modifier == 'a') {
                address_modifier = true;
            } else if (modifier == 's') {
                sfr_modifier = true;
            } else if (modifier == 'c') {
                condition = true;
            } else {
                break;
            }
            ++j;
        }
        if (j >= template_text.size()) break;
        const char operand_char = template_text[j];
        i = j;

        if (operand_char == '%') {
            out += '%';
            continue;
        }
        if (operand_char != '0' && operand_char != '1') {
            out += operand_char;
            continue;
        }

        const Rl78Operand& op = insn.operand(operand_char - '0');
        if (condition) {
            out += kConditionNames[static_cast<int>(op.condition) & 7];
            continue;
        }
        // "!%!" would print two bangs: the literal one in the template wins.
        if (bang && !out.empty() && out.back() == '!') bang = false;
        append_operand(out, op, bang, es_modifier, address_modifier, sfr_modifier, insn.is_word);
    }
    return out;
}

std::string rl78_disassemble(Bus& bus, u32 address, unsigned& length) {
    bus.context.pc = address;
    bus.context.core = "RL78";

    Rl78Decoded insn;
    if (!rl78_decode(bus, address, insn)) {
        length = 1;
        return "??";
    }
    length = insn.length;
    return rl78_format_instruction(insn);
}

}  // namespace zlb
