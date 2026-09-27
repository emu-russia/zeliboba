import struct
import sys

path = sys.argv[1]
d = open(path, 'rb').read()
print('file', path, len(d), 'bytes')
if d[:4] != b'\x7fELF':
    print('not an ELF:', d[:16].hex(' '))
    sys.exit(0)
e_type, e_machine, e_version, e_entry, e_phoff, e_shoff, e_flags = struct.unpack_from('<HHIIIII', d, 16)
e_ehsize, e_phentsize, e_phnum = struct.unpack_from('<HHH', d, 40)
print('type=0x%04X machine=0x%04X entry=0x%08X phoff=0x%X phnum=%d flags=0x%X' %
      (e_type, e_machine, e_entry, e_phoff, e_phnum, e_flags))
for i in range(e_phnum):
    off = e_phoff + i * e_phentsize
    p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_flags, p_align = struct.unpack_from('<IIIIIIII', d, off)
    print('  ph[%d] type=0x%08X off=0x%06X vaddr=0x%08X filesz=0x%06X memsz=0x%06X flags=%d' %
          (i, p_type, p_offset, p_vaddr, p_filesz, p_memsz, p_flags))

# Print the SceModuleInfo name if we can find it (search for the module-info magic layout).
for off in range(0, min(len(d), 0x40000) - 64, 4):
    if d[off:off + 4] == b'\x04\xfe\x00\x01' or d[off:off + 2] == b'\x04\xfe':
        pass

# crude string extraction around the module info area
import re
strings = re.findall(rb'[ -~]{6,}', d[:0x8000])
print('early strings:')
for s in strings[:40]:
    print('   ', s.decode('latin1'))
