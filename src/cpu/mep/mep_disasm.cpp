// zeliboba - standalone MeP-c5 disassembler.
#include "cpu/mep/mep_disasm.h"

#include "cpu/mep/mep_isa.h"

namespace zlb {

std::string mep_disassemble(Bus& bus, u32 address, unsigned& length) {
    bus.context.core = "CMeP";
    bus.context.pc = address;

    const u32 word = static_cast<u32>(bus.fetch16(address)) |
                     (static_cast<u32>(bus.fetch16(address + 2)) << 16);

    const mep::Insn* insn = mep::decode(word);
    if (insn->op == mep::Op::None) {
        length = 4;
        return "*unknown*";
    }
    length = insn->len;
    return mep::format(*insn, word, address);
}

}  // namespace zlb
