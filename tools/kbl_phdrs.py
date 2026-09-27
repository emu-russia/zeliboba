"""Print the program headers of the ELF embedded in kernel_boot_loader.self.

The SELF keeps the ELF header and its program header table in the clear, so the
segment list (including p_paddr, which nothing else in the toolchain shows) can
be read straight out of the container.  p_paddr is the load address a boot ROM
uses while the MMU is off; p_vaddr is where the segment expects to be *seen*
once the tables are up.  For the kernel boot loader the two differ, which is
exactly the kind of thing a loader bug hides.
"""
import struct
import sys

path = sys.argv[1] if len(sys.argv) > 1 else (
    r"C:\Work\PSVita\Vita_104_Firmware\Out\SLB2\kernel_boot_loader.self")
data = open(path, "rb").read()
print(f"{path}: {len(data)} bytes, magic {data[:4]!r}")

# SELF header: magic, version, sdk_type, header_len, elf_offset(?), ...
# The Vita SELF puts the ELF at a fixed offset recorded in the header; scan for
# \x7fELF to be independent of the exact field layout.
elf_off = data.find(b"\x7fELF")
if elf_off < 0:
    raise SystemExit("no ELF magic in the container")
print(f"ELF at 0x{elf_off:X}")

e_ident = data[elf_off:elf_off + 16]
is64 = e_ident[4] == 2
end = "<" if e_ident[5] == 1 else ">"
print(f"class={'ELF64' if is64 else 'ELF32'}")

if is64:
    (e_type, e_machine, e_version, e_entry, e_phoff, e_shoff, e_flags, e_ehsize,
     e_phentsize, e_phnum) = struct.unpack_from(end + "HHIQQQIHHH", data, elf_off + 16)
else:
    (e_type, e_machine, e_version, e_entry, e_phoff, e_shoff, e_flags, e_ehsize,
     e_phentsize, e_phnum) = struct.unpack_from(end + "HHIIIIIHHH", data, elf_off + 16)

print(f"type={e_type} machine={e_machine} entry=0x{e_entry:08X} phoff=0x{e_phoff:X} "
      f"phnum={e_phnum} phentsize={e_phentsize}")

names = {0: "NULL", 1: "LOAD", 2: "DYNAMIC", 3: "INTERP", 4: "NOTE", 6: "PHDR",
         7: "TLS", 0x60000000: "SCE_LOAD", 0x61000000: "SCE_RELA", 0x61000001: "SCE_RELA?"}
for i in range(e_phnum):
    off = elf_off + e_phoff + i * e_phentsize
    if is64:
        p_type, p_flags, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_align = \
            struct.unpack_from(end + "IIQQQQQQ", data, off)
    else:
        p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_flags, p_align = \
            struct.unpack_from(end + "IIIIIIII", data, off)
    print(f"  [{i}] {names.get(p_type, hex(p_type)):<10} off=0x{p_offset:06X} "
          f"vaddr=0x{p_vaddr:08X} paddr=0x{p_paddr:08X} filesz=0x{p_filesz:06X} "
          f"memsz=0x{p_memsz:06X} flags={p_flags}")
