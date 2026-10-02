// zeliboba - emmc_rebuild: build and inspect the reconstructed eMMC image.
//
//   emmc_rebuild --firmware <Out dir> --out <image> [--size <n>] [--verbose]
//   emmc_rebuild --inspect <image>
//   emmc_rebuild --verify  <image> [--firmware <Out dir>]
//
// Build mode writes the console's user area (master block + partition table,
// idstorage, the two SLB2 copies, os0 x2, vs0) and both eMMC boot partitions
// with the SLB2 stage. --size accepts plain numbers of bytes or the suffixes
// K/M/G (1024 based). --verify mounts the FAT16 volumes back out of the image,
// checks every file against the tree it was built from and re-reads the SLB2
// entry table.
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "common/types.h"
#include "common/util.h"
#include "hw/emmc/emmc.h"

using namespace zlb;

namespace {

const char* partition_label(EmmcPartition partition) {
    switch (partition) {
        case EmmcPartition::User: return "user";
        case EmmcPartition::Boot0: return "boot0";
        case EmmcPartition::Boot1: return "boot1";
        case EmmcPartition::Rpmb: return "rpmb";
    }
    return "?";
}

/// Parse "512M", "4G", "0x20000000" or "1073741824".
bool parse_size(const std::string& text, u64& out) {
    std::string value = trim(text);
    if (value.empty()) return false;
    u64 multiplier = 1;
    const char suffix = value.back();
    if (suffix == 'K' || suffix == 'k') {
        multiplier = KB;
        value.pop_back();
    } else if (suffix == 'M' || suffix == 'm') {
        multiplier = MB;
        value.pop_back();
    } else if (suffix == 'G' || suffix == 'g') {
        multiplier = GB;
        value.pop_back();
    }
    u64 base = 0;
    if (!parse_u64(trim(value), base)) return false;
    out = base * multiplier;
    return true;
}

void print_layout(const EmmcImagePlan& plan, bool show_notes) {
    std::printf("\n%-14s %-10s %-12s %-12s %s\n", "region", "area", "offset", "size", "source");
    std::printf("%s\n", std::string(104, '-').c_str());
    for (const auto& entry : plan.entries) {
        std::printf("%-14s %-10s 0x%010llX 0x%010llX %s\n", entry.name.c_str(),
                    partition_label(entry.partition), static_cast<unsigned long long>(entry.offset),
                    static_cast<unsigned long long>(entry.size), entry.source.c_str());
    }
    std::printf("%s\n", std::string(104, '-').c_str());
    std::printf("%-14s %-10s 0x%010llX 0x%010llX %s\n", "TOTAL", "-",
                static_cast<unsigned long long>(0),
                static_cast<unsigned long long>(plan.total_size), plan.message.c_str());
    if (show_notes) {
        std::printf("\nnotes:\n");
        for (const auto& note : plan.notes) std::printf("  - %s\n", note.c_str());
    }
}

void print_verify(const EmmcVerifyReport& report) {
    std::printf("\nverify %s\n", report.ok ? "OK" : "FAILED");
    std::printf("  master block : %s\n", report.master_block_message.c_str());
    std::printf("  SLB2         : %s (%u copies checked)\n", report.slb2_message.c_str(),
                report.slb2_entries);
    for (const auto& partition : report.partitions) {
        std::printf("  %s\n", partition.c_str());
    }
    std::printf("  files checked: %llu, mismatches: %llu\n",
                static_cast<unsigned long long>(report.files_checked),
                static_cast<unsigned long long>(report.file_mismatches));
    if (!report.problems.empty()) {
        std::printf("  problems (%u):\n", static_cast<unsigned>(report.problems.size()));
        for (size_t i = 0; i < report.problems.size() && i < 60; ++i) {
            std::printf("    ! %s\n", report.problems[i].c_str());
        }
        if (report.problems.size() > 60) {
            std::printf("    ... %u more\n", static_cast<unsigned>(report.problems.size() - 60));
        }
    }
}

void usage() {
    std::printf(
        "emmc_rebuild - build and inspect the reconstructed Vita eMMC image\n"
        "\n"
        "usage:\n"
        "  emmc_rebuild --firmware <Out dir> --out <image> [--size <n>] [--verbose]\n"
        "  emmc_rebuild --inspect <image> [--verbose]\n"
        "  emmc_rebuild --verify <image> [--firmware <Out dir>]\n"
        "\n"
        "  --firmware <dir>  Vita_104_Firmware/Out (SLB2/, PUP_dec/, fs/)\n"
        "  --out <image>     image file to create or overwrite\n"
        "  --size <n>        user area size (512M, 1G, 0x20000000 ...); default is the\n"
        "                    console's real 3.55 GiB. Minimum 432 MiB + boot/RPMB.\n"
        "  --from-tree       synthesise the FAT16 volumes from fs/os0 and fs/vs0\n"
        "                    instead of laying down the real PUP_dec/os0.bin and\n"
        "                    PUP_dec/vs0.bin verbatim (which is the default)\n"
        "  --inspect <image> print the partition table and mount the FAT volumes\n"
        "  --verify <image>  mount everything back and compare against fs/\n"
        "  --verbose         per-file / per-entry progress\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string firmware;
    std::string out_path;
    std::string inspect_path;
    std::string verify_path;
    bool verify_requested = false;
    u64 size_bytes = 0;
    bool size_given = false;
    bool verbose = false;
    bool from_tree = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::printf("error: %s needs a value\n", what);
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--firmware") {
            firmware = next("--firmware");
        } else if (arg == "--out") {
            out_path = next("--out");
        } else if (arg == "--inspect") {
            inspect_path = next("--inspect");
        } else if (arg == "--verify") {
            // `--verify <image>` checks an image explicitly; a bare `--verify`
            // checks the image `--out` just wrote, which is what verify.ps1 and
            // the build recipes want.
            verify_requested = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') verify_path = argv[++i];
        } else if (arg == "--size") {
            const std::string value = next("--size");
            if (!parse_size(value, size_bytes)) {
                std::printf("error: cannot parse --size %s\n", value.c_str());
                return 2;
            }
            size_given = true;
        } else if (arg == "--from-tree") {
            from_tree = true;
        } else if (arg == "--verbose" || arg == "-v") {
            verbose = true;
        } else if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else {
            std::printf("error: unknown argument '%s'\n", arg.c_str());
            usage();
            return 2;
        }
    }

    if (inspect_path.empty() && verify_path.empty() && out_path.empty()) {
        usage();
        return 2;
    }

    int result = 0;

    if (!out_path.empty()) {
        if (firmware.empty()) {
            std::printf("error: --firmware is required to build an image\n");
            return 2;
        }
        std::printf("building %s from %s\n", out_path.c_str(), firmware.c_str());
        const EmmcImagePlan plan =
            build_emmc_image(firmware, out_path, verbose, size_given ? size_bytes : 0, from_tree);
        print_layout(plan, true);
        if (!plan.ok) {
            std::printf("\nBUILD FAILED: %s\n", plan.message.c_str());
            return 1;
        }
        std::printf("\nBUILD OK: %s\n", plan.message.c_str());
        if (file_exists(out_path)) {
            std::printf("image file: %s on disk (sparse where possible)\n",
                        human_size(file_size(out_path)).c_str());
        }
    }

    if (!inspect_path.empty()) {
        const EmmcImagePlan plan = inspect_emmc_image(inspect_path);
        print_layout(plan, true);
        if (!plan.ok) {
            std::printf("\nINSPECT FAILED: %s\n", plan.message.c_str());
            result = 1;
        }
    }

    if (verify_path.empty() && verify_requested) verify_path = out_path;

    if (!verify_path.empty()) {
        std::string root = firmware;
        if (root.empty()) {
            // Default to the workspace layout next to the emulator checkout.
            for (const char* candidate : {"../Vita_104_Firmware/Out", "../../Vita_104_Firmware/Out",
                                          "../Vita_104_Firmware/Out"}) {
                if (std::filesystem::is_directory(candidate)) {
                    root = candidate;
                    break;
                }
            }
        }
        if (root.empty()) {
            std::printf("error: --verify needs --firmware <Out dir> to compare against the tree\n");
            return 2;
        }
        std::printf("verifying %s against %s\n", verify_path.c_str(), root.c_str());
        const EmmcVerifyReport report = verify_emmc_image(verify_path, root);
        print_verify(report);
        if (!report.ok) result = 1;
    }

    return result;
}
