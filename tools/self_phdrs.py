"""Print the program headers of a Vita SELF container (segments + paddr).

The SELF keeps its own program header table (separate from the inner ELF header)
in the clear, and each entry carries both p_vaddr and p_paddr.  p_paddr is the
load address a boot ROM uses while the MMU is off, so for kernel_boot_loader.self
the two differ and the difference is what a loader must honour.
"""
import struct
import sys

path = sys.argv[1] if len(sys.argv) > 1 else (
    r"..\Vita_104_Firmware\Out\SLB2\kernel_boot_loader.self")
data = open(path, "rb").read()
u32 = lambda o: struct.unpack_from("<I", data, o)[0]
u64 = lambda o: struct.unpack_from("<Q", data, o)[0]

print(f"{path}: {len(data)} bytes, magic {data[:4]!r}")
sdk = data[8:0x0C]
header_len = u32(0x0C)
elf_offset = u64(0x40)
phdr_offset = u64(0x48)
shdr_offset = u64(0x50)
seg_info_offset = u64(0x38)
print(f"SELF: sdk={sdk.hex()} header_len=0x{header_len:X} elf_offset=0x{elf_offset:X} "
      f"phdr_offset=0x{phdr_offset:X} shdr_offset=0x{shdr_offset:X} "
      f"segment_info=0x{seg_info_offset:X}")

if data[elf_offset:elf_offset + 4] != b"\x7fELF":
    raise SystemExit("no inner ELF")
e_phnum = struct.unpack_from("<H", data, elf_offset + 0x2C)[0]
e_phentsize = struct.unpack_from("<H", data, elf_offset + 0x2A)[0]
e_entry = u32(elf_offset + 0x18)
print(f"inner ELF: entry=0x{e_entry:08X} phnum={e_phnum} phentsize={e_phentsize}")

types = {0: "NULL", 1: "LOAD", 2: "DYNAMIC"}
for i in range(e_phnum):
    po = phdr_offset + i * e_phentsize
    p_type = u32(po + 0x00)
    p_offset = u32(po + 0x04)
    p_vaddr = u32(po + 0x08)
    p_paddr = u32(po + 0x0C)
    p_filesz = u32(po + 0x10)
    p_memsz = u32(po + 0x14)
    p_flags = u32(po + 0x18)
    p_align = u32(po + 0x1C)
    print(f"  [{i}] {types.get(p_type, hex(p_type)):<8} off=0x{p_offset:06X} "
          f"vaddr=0x{p_vaddr:08X} paddr=0x{p_paddr:08X} filesz=0x{p_filesz:06X} "
          f"memsz=0x{p_memsz:06X} flags={p_flags} align=0x{p_align:X}")
