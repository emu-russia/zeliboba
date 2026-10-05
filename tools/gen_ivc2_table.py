#!/usr/bin/env python3
"""Generate the zeliboba IVC2 slot decode table from a CGEN description.

    python tools/gen_ivc2_table.py [--cpu <mep-ivc2.cpu>] [--core <mep-core.cpu>]

Writes `src/cpu/mep/mep_ivc2_table.inc`.  The inputs are the binutils/GDB CGEN
architecture descriptions for Toshiba's MeP: `mep-ivc2.cpu` (the Venezia VLIW
coprocessor) and `mep-core.cpu` (the core fields and operands that the IVC2 file
references but does not declare).  Neither ships with this repository; fetch
them from binutils (`cpu/mep-ivc2.cpu`, `cpu/mep-core.cpu`) or point --cpu/--core
at local copies.

How the encodings are read
--------------------------

An instruction is declared as

    (dni NAME "doc" (OPTIONAL_CP_INSN <isa> (SLOTS <slot>) ...) "syntax"
         (+ TOKEN TOKEN ...) <semantics> ())

and the `(+ ...)` list is the machine encoding.  CGEN does not lay the tokens
out left to right: every token is a field (or an operand wrapping one) whose
*bits come from the field declaration* (`(dnf f-ivc2-5u21 ... 21 5)` is five
bits ending at bit 21, in the little endian instruction word).  Tokens that are
not operands pin their bits to the value given (or zero for a bare atom), and
`MAJ_n` is the major opcode macro from the core file.  Operand bits are variable
and stay out of the match mask - the earlier version of this script applied the
operand fields as if they were fixed and made encodings that differ only inside
an operand collide.

Known limitation
----------------

This script only sees the two files given to it.  `mep-core.cpu` does not
declare every field the IVC2 file uses (`f-ivc2-4u20`, `f-21`, `f-29`, ... are
referenced by instructions but not defined anywhere), so those operand bits end
up unconstrained and a few dozen encodings collide (527 distinct mask/value
pairs for 688 forms).  Pinning them down needs the full MeP-Integrator field
set.  `tools/verify_ivc2_table.py` in the analysis workspace cross-checks the
result against the encodings written in the source comments.
"""
import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT = os.path.join(ROOT, "src", "cpu", "mep", "mep_ivc2_table.inc")


def num(tok):
    """Scheme style integer literal: #x1f, #b101, 42, -7, #f/#t."""
    t = tok.strip()
    if t in ("#f", "#F"):
        return 0
    if t in ("#t", "#T"):
        return 1
    for prefix, base in (("#x", 16), ("#X", 16), ("#b", 2), ("#B", 2),
                         ("#o", 8), ("#O", 8), ("#d", 10), ("#D", 10)):
        if t.startswith(prefix):
            return int(t[2:], base)
    return int(t, 0)


def tokenize(s):
    i = 0
    n = len(s)
    while i < n:
        c = s[i]
        if c == ";":
            while i < n and s[i] != "\n":
                i += 1
            continue
        if c in "()":
            yield c, i
            i += 1
            continue
        if c.isspace():
            i += 1
            continue
        if c == '"':
            j = i + 1
            while j < n and s[j] != '"':
                if s[j] == "\\":
                    j += 1
                j += 1
            yield s[i:j + 1], i
            i = j + 1
            continue
        j = i
        while j < n and not s[j].isspace() and s[j] not in "()":
            j += 1
        yield s[i:j], i
        i = j


def parse_all(s, primary=True):
    """Top level forms as nested lists; core forms are tagged with False."""
    stack = []
    forms = []
    for tok, _ in tokenize(s):
        if tok == "(":
            stack.append([])
        elif tok == ")":
            if not stack:
                continue
            done = stack.pop()
            if stack:
                stack[-1].append(done)
            else:
                forms.append(done if primary else done + [False])
        elif stack:
            stack[-1].append(tok)
    return forms


def split_format(fmt):
    """Split a CGEN syntax string into ('txt', s) / ('op', name) tokens."""
    out = []
    i = 0
    while i < len(fmt):
        if fmt[i] == "$":
            j = i + 1
            while j < len(fmt) and (fmt[j].isalnum() or fmt[j] == "_"):
                j += 1
            out.append(("op", fmt[i + 1:j]))
            i = j
        else:
            j = i
            while j < len(fmt) and fmt[j] != "$":
                j += 1
            out.append(("txt", fmt[i:j]))
            i = j
    return out


REG_PRINT = {
    "h-gpr": "Reg",
    "h-cr64": "Cp64",
    "h-cr": "Cp32",
    "h-ccr-ivc2": "Ivc2Ccr",
    "h-csr": "Csr",
}


def flatten_hw(item):
    if isinstance(item, str):
        return item if item.startswith("h-") else ""
    if isinstance(item, list):
        for sub in item:
            r = flatten_hw(sub)
            if r:
                return r
    return ""


def attr_value(attrs, key):
    for item in attrs:
        if isinstance(item, list) and item and item[0] == key:
            return item[1:]
    return None


def find_encoding(form):
    for item in form:
        if isinstance(item, list) and item and item[0] == "+":
            return item[1:]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cpu", help="path to mep-ivc2.cpu")
    ap.add_argument("--core", help="path to mep-core.cpu")
    ap.add_argument("--out", default=DEFAULT_OUT)
    args = ap.parse_args()

    if not args.cpu:
        ap.error("--cpu is required (the CGEN description is not part of this repository)")
    text = open(args.cpu, encoding="latin1").read()
    core_text = open(args.core, encoding="latin1").read() if args.core and \
        os.path.exists(args.core) else ""

    forms = parse_all(text)
    for form in parse_all(core_text, primary=False):
        forms.append(form)

    # ---- fields -----------------------------------------------------------
    fields = {}
    for form in forms:
        if not form or form[0] not in ("dnf", "df"):
            continue
        try:
            start = num(form[4])
            length = num(form[5])
        except (IndexError, ValueError):
            continue
        mode = form[6] if len(form) > 6 else "UINT"
        fields[form[1]] = dict(start=start, length=length,
                               signed=mode in ("INT", "SI", "DI"))

    # multi-ifields: subfields, most significant part first, with the shift the
    # extract expression applies
    multis = {}
    for form in forms:
        if not form or form[0] != "define-multi-ifield":
            continue
        name = None
        subs = []
        for item in form[1:]:
            if isinstance(item, list) and item:
                if item[0] == "name":
                    name = item[1]
                elif item[0] == "subfields":
                    subs = item[1:]
        if not name:
            continue
        parts = [[sub, 0] for sub in subs]
        for item in form[1:]:
            if isinstance(item, list) and item and item[0] == "extract":
                stack = [item]
                while stack:
                    node = stack.pop()
                    if isinstance(node, list):
                        if node and node[0] == "sll" and len(node) > 2:
                            fld = node[1]
                            if isinstance(fld, list) and fld and fld[0] == "ifield":
                                for part in parts:
                                    if part[0] == fld[1]:
                                        part[1] = num(node[2])
                        stack.extend(node)
        multis[name] = parts

    # The IVC2 field names encode their own geometry: f-ivc2-<bits><u|s><start>.
    for name in re.findall(r"f-ivc2-\d+[us]\d+", text):
        m = re.match(r"f-ivc2-(\d+)([us])(\d+)$", name)
        if m:
            fields.setdefault(name, dict(start=int(m.group(3)), length=int(m.group(1)),
                                         signed=m.group(2) == "s"))
    fields.setdefault("f-ivc2-imm16p0", dict(start=20, length=16, signed=False))
    for bitn in range(32):
        fields.setdefault("f-%d" % bitn, dict(start=bitn, length=1, signed=False))

    # MAJ_n opcode macros (declared in the core file as (define-pmacro (MAJ_n) ...)).
    # The value comes from the core file; the fallback names use the decimal digit.
    macros = {"MAJ_%d" % v: (31, 4, v) for v in range(16)}
    for form in forms:
        if not form or form[0] != "define-pmacro" or form[-1] is False:
            continue
        head = form[1]
        if not (isinstance(head, list) and len(head) == 1):
            continue
        for item in form[2:]:
            if isinstance(item, list) and item and item[0] == "f-major" and len(item) > 1:
                macros[head[0]] = (31, 4, num(item[1]))

    # ---- operands ---------------------------------------------------------
    operands = {}
    for form in forms:
        if not form or form[0] not in ("dnop", "dpop"):
            continue
        from_core = form[-1] is False
        body = [x for x in form if x is not False]
        if len(body) < 6:
            continue
        name = body[1]
        if from_core and name in operands:
            continue    # the IVC2 file's own operand definitions win
        hw = body[4] if isinstance(body[4], str) else flatten_hw(body[4])
        fld = body[5] if isinstance(body[5], str) else None
        if hw in REG_PRINT and fld:
            operands[name] = dict(print=REG_PRINT[hw], field=[fld])
        else:
            operands[name] = dict(print="SImm" if hw == "h-sint" else "Imm",
                                  field=[fld] if fld else [])

    # ---- instructions -----------------------------------------------------
    warnings = []
    insns = []
    for form in forms:
        if not form or form[0] != "dni" or form[-1] is False:
            continue
        name = form[1]
        attrs = form[3]
        fmt = form[4]
        if len(fmt) >= 2 and fmt[0] == '"' and fmt[-1] == '"':
            fmt = fmt[1:-1]
        enc = find_encoding(form)
        if enc is None:
            continue
        slots = attr_value(attrs, "SLOTS")
        if not slots:
            warnings.append("%s: no SLOTS" % name)
            continue
        slot_list = [s.strip() for s in slots[0].split(",")]

        def slot_fits(candidate):
            if candidate in ("C3", "P0"):
                return True
            for item in enc:
                if isinstance(item, list):
                    fi = fields.get(item[0])
                    if fi and fi["start"] - fi["length"] + 1 < 0:
                        return False
            return True

        slot = next((s for s in slot_list if slot_fits(s)), slot_list[0])

        def operand_parts(op_name):
            parts = []
            for fname in operands[op_name]["field"]:
                if fname in multis:
                    for sub, shift in sorted(multis[fname], key=lambda p: -p[1]):
                        if sub in fields:
                            fi = fields[sub]
                            parts.append((fi["start"], fi["length"], shift))
                elif fname in fields:
                    fi = fields[fname]
                    parts.append((fi["start"], fi["length"], 0))
            return parts

        mask = 0
        value = 0
        ops = []
        ok = True
        for item in enc:
            if isinstance(item, list):
                fname = item[0]
                val = num(item[1]) if len(item) > 1 else 0
            elif item in operands:
                ops.append((item, operands[item]))
                continue
            else:
                fname = item
                val = macros.get(fname, (0, 0, 0))[2]
            fi = fields.get(fname)
            if fi is None and fname in macros:
                st, ln, mval = macros[fname]
                fi = dict(start=st, length=ln, signed=False)
                val = mval
            if fi is None:
                warnings.append("%s: unknown field %s" % (name, fname))
                ok = False
                break
            lo = fi["start"] - fi["length"] + 1
            if lo < 0:
                inside = fi["length"] + lo
                if inside > 0:
                    m = (1 << inside) - 1
                    mask |= m
                    value |= (val & m)
                continue
            m = ((1 << fi["length"]) - 1) << lo
            mask |= m
            value |= (val & ((1 << fi["length"]) - 1)) << lo
        if not ok:
            continue

        # only the operands the syntax string names get printed, in that order
        syntax_order = [t[1] for t in split_format(fmt) if t[0] == "op"]
        by_name = {oname: oinfo for oname, oinfo in ops}
        ordered = []
        for oname in syntax_order:
            if oname in by_name:
                ordered.append((oname, by_name.pop(oname)))
        for oname, oinfo in by_name.items():
            ordered.append((oname, oinfo))
        insns.append(dict(name=name, slot=slot, mask=mask, value=value, fmt=fmt,
                          ops=ordered))

    # ---- emit -------------------------------------------------------------
    lines = []
    lines.append("// zeliboba - MeP IVC2 coprocessor slot decoding table (generated).")
    lines.append("//")
    lines.append("// Generated by tools/gen_ivc2_table.py from the CGEN description")
    lines.append("// cpu/mep-ivc2.cpu (binutils/GDB) plus the core fields from")
    lines.append("// cpu/mep-core.cpu.  Do not edit by hand.  One entry per `(dni ...)` form:")
    lines.append("// `mask`/`value` are the bits the encoding pins down (operand bits are")
    lines.append("// variable and deliberately not masked), `ops` the operands in syntax order.")
    lines.append("//")
    lines.append("// Field extraction goes through mep::raw(), which numbers the bits the CGEN")
    lines.append("// way (MSB first inside each 16 bit half); the slot word itself is built in")
    lines.append("// mep_ivc2.cpp, which also implements binutils' V1/V2/V3 packet split.")
    lines.append("//")
    lines.append("// Known limitation: mep-core.cpu does not declare every field the IVC2 file")
    lines.append("// uses (f-ivc2-4u20, f-21, f-29, ...), so some operand bits stay")
    lines.append("// unconstrained and a few dozen encodings collide.  The full MeP-Integrator")
    lines.append("// field set is needed to pin those down.")
    lines.append("")

    order = ["C3", "P0S", "P0", "P1"]
    for slot in order:
        group = [i for i in insns if i["slot"] == slot]
        group.sort(key=lambda i: -bin(i["mask"]).count("1"))
        lines.append("// ---------------------------------------------------------------------------")
        lines.append("// %s slot (%d encodings)" % (slot, len(group)))
        lines.append("// ---------------------------------------------------------------------------")
        for ins in group:
            ops = []
            for oname, oinfo in ins["ops"]:
                pr = oinfo["print"]
                if pr == "Name":
                    nm = oname[5:] if oname.startswith("ivc2_") else oname
                    ops.append('{"%s", Ivc2Print::Name}' % nm)
                elif not oinfo["field"]:
                    ops.append('{"%s", Ivc2Print::%s}' % (oname, pr))
                else:
                    parts = ", ".join("{%d, %d, %d, false}" % p
                                      for p in operand_parts(oname))
                    ops.append('{"%s", Ivc2Print::%s, {%s}}' % (oname, pr, parts))
            escaped = ins["fmt"].replace("\\", "\\\\").replace('"', '\\"')
            lines.append('    IVC2_SLOT("%s", "%s", 0x%08Xu, 0x%08Xu, %s, %d,' % (
                ins["name"], escaped, ins["mask"], ins["value"], slot, len(ops)))
            for op in ops:
                lines.append("        %s," % op)
            lines.append("    )")
        lines.append("")

    open(args.out, "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")
    by_slot = {}
    for ins in insns:
        by_slot[ins["slot"]] = by_slot.get(ins["slot"], 0) + 1
    print("wrote %s: %d encodings %s, %d warnings"
          % (args.out, len(insns), by_slot, len(warnings)))
    for w in warnings[:20]:
        print("  WARN", w)
    return 0


if __name__ == "__main__":
    sys.exit(main())
