# Clean zeliboba source delivery — native macOS logo

The top-level folder remains `zeliboba-main/`. This delivery contains the current
implementation, Mac launch/build scripts, Windows project files, required icon
assets, vendored SDL libraries/licenses, documentation and the session debrief.

Standalone PNG/BMP and other raster captures, the screenshots tree, machine
build/runtime logs, capture metadata and fixed-host probe harnesses are omitted.
Text included from the session is sanitized to project-relative paths. Required
`artwork/zeliboba.ico` and `src/ui/zeliboba_icon_pixels.h` remain; these are normal
build/runtime assets. No original working source or capture was deleted.

The original source ZIP contained 69 PNG files and 27 BMP files. This cleaned
ZIP contains none. It also omits firmware dumps, firmware archives, external
Graphics PDFs, eMMC images, CMake caches and compiled host executables/objects.

## Build and run

Place the supplied firmware and boot/Syscon dumps beside the source folder as
shown in README.md. Install Apple command line tools, CMake and SDL3, then:

```bash
cd zeliboba-main
./build-macos.sh
./run-macos.sh --run 0 -ex "runm 1000000"
```

Press F7 for Display. `./run-macos.sh --cli` starts the CLI. The validated source
reaches genuine os0 execution and the white PlayStation logo. The measured Mac
capture took about 109 seconds. Full kernel startup, LiveArea, guest SGX rendering
and complete physical display behavior remain unfinished.

## Review and verification

Read `docs/STATUS.md` and `docs/BOOT_LOGO_DEBRIEF_2026-10-01.md`.
`VERIFICATION.json` summarizes the verified implementation result without local
capture paths. The path-free full test output is included at
`build/goal-native-iftu-arm-tests.log` (617 passed, zero failures).

`SOURCE_CHANGES.json` distinguishes implementation changes from delivery-only
sanitization and image/capture omissions. `SOURCE_CHANGES.patch` compares a
sanitized baseline text view with sanitized delivered text; it is a review aid,
not a byte-exact patch against the unsanitized incoming ZIP. Authoritative
baseline/delivery hashes are in JSON. `PACKAGE_MANIFEST.json` hashes all included
files except itself. Windows configurations are preserved; the fresh runtime and
build validation reported here was performed on macOS.
