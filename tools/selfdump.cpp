// zeliboba - dump the decrypted image and every segment of a SELF container.
//
// The boot chain needs the pieces of kernel_boot_loader.self separately: the
// secure kernel bootloader (segment at PA 0x40020000), its data (0x4005C000) and
// the ARZL-compressed non-secure kernel bootloader the loader stages at PA
// 0x50000000 before decompressing it to 0x51000000 (wiki NSKBL).  self_to_elf()
// already does the whole SCE key / metadata / AES-CTR pipeline, so this tool only
// has to cut the result along the program headers.
//
// Usage: selfdump <file.self> [output-dir]
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "common/util.h"
#include "loader/keys.h"
#include "loader/loader.h"
#include "loader/loader_extra.h"

using namespace zlb;

namespace {

u32 rd32(const std::vector<u8>& data, size_t offset) {
    if (offset + 4 > data.size()) return 0;
    return static_cast<u32>(data[offset]) | (static_cast<u32>(data[offset + 1]) << 8) |
           (static_cast<u32>(data[offset + 2]) << 16) | (static_cast<u32>(data[offset + 3]) << 24);
}

u16 rd16(const std::vector<u8>& data, size_t offset) {
    if (offset + 2 > data.size()) return 0;
    return static_cast<u16>(data[offset] | (data[offset + 1] << 8));
}

std::string printable(const u8* bytes, size_t size) {
    std::string text;
    for (size_t i = 0; i < size; ++i)
        text.push_back(bytes[i] >= 32 && bytes[i] < 127 ? static_cast<char>(bytes[i]) : '.');
    return text;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: selfdump <file.self> [output-dir]\n");
        return 2;
    }
    const std::string path = argv[1];
    const std::string outdir = argc > 2 ? argv[2] : ".";

    auto data = read_file(path);
    if (!data) {
        std::fprintf(stderr, "selfdump: cannot read %s\n", path.c_str());
        return 1;
    }

    SelfDecryptReport report;
    auto elf = self_to_elf_report(*data, SceKeys::default_keys(), {}, report);
    if (!elf) {
        std::fprintf(stderr, "selfdump: SELF -> ELF failed at '%s': %s\n", report.stage.c_str(),
                     report.message.c_str());
        return 1;
    }
    std::printf("%s: %zu bytes -> decrypted image 0x%zX bytes\n", path.c_str(), data->size(),
                elf->size());
    std::printf("  metadata key=%s sections=%d keys=%d inflate=%s\n", report.metadata_key.c_str(),
                report.metadata_sections, report.metadata_keys, report.inflate_mode.c_str());
    for (const std::string& note : report.segment_notes) std::printf("  %s\n", note.c_str());

    make_directories(outdir);
    const std::string image = path_join(outdir, path_stem(path_filename(path)) + ".elf");
    if (!write_file(image, *elf)) {
        std::fprintf(stderr, "selfdump: cannot write %s\n", image.c_str());
        return 1;
    }
    std::printf("  wrote %s\n", image.c_str());

    // The program header table of the reconstructed image describes the segments.
    const size_t phoff = rd32(*elf, 0x1C);
    const size_t phentsize = rd16(*elf, 0x2A);
    const size_t phnum = rd16(*elf, 0x2C);
    std::printf("  phoff=0x%zX phnum=%zu phentsize=0x%zX\n", phoff, phnum, phentsize);
    for (size_t i = 0; i < phnum; ++i) {
        const size_t po = phoff + i * phentsize;
        const u32 type = rd32(*elf, po + 0x00);
        const u32 offset = rd32(*elf, po + 0x04);
        const u32 vaddr = rd32(*elf, po + 0x08);
        const u32 paddr = rd32(*elf, po + 0x0C);
        const u32 filesz = rd32(*elf, po + 0x10);
        const u32 memsz = rd32(*elf, po + 0x14);
        std::printf("  [%zu] type=%u off=0x%06X vaddr=0x%08X paddr=0x%08X filesz=0x%06X memsz=0x%06X\n",
                    i, type, offset, vaddr, paddr, filesz, memsz);
        if (type != 1 || filesz == 0 || offset + filesz > elf->size()) continue;
        const std::string name =
            path_join(outdir, format("seg%zu_%08X.bin", i, static_cast<unsigned>(paddr)));
        write_file(name, elf->data() + offset, filesz);
        std::string hexdump;
        for (int b = 0; b < 16; ++b) hexdump += format("%02X ", (*elf)[offset + static_cast<size_t>(b)]);
        std::printf("      wrote %s (0x%X bytes), first16: %s | %s\n", name.c_str(), filesz,
                    hexdump.c_str(), printable(elf->data() + offset, 16).c_str());
    }
    return 0;
}
