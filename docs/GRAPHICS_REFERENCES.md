# Local graphics references

Reviewed 2026-10-01. The workspace's `../../Graphics` folder contains **15 PDFs,
800 physical pages**. All were inventoried and made searchable; the architecture,
rendering, display, texture and compiler sections were reviewed against the
current code. This is a focused review, not an exhaustive audit of every API.
Representative diagrams and format tables were also rendered and checked.

These are preliminary SDK documents from 2010–2011. The GPU guide's cover says
Release **0.945**; the other fourteen covers say **0.940**, although their footers
say 0.945. Check version-specific behavior against the supplied firmware 1.04.
Page references below use physical PDF pages, which match printed page numbers.

## Inventory

| Reference | Pages | Use in this project |
|---|---:|---|
| [GPU User Guide](../../Graphics/GPU-Users_Guide_e.pdf) | 151 | SGX543MP4+ architecture, pipeline, texture/surface memory layouts |
| [GXM Overview](../../Graphics/libgxm-Overview_e.pdf) | 43 | Rendering workflow, rings, jobs, synchronization and shader patching |
| [GXM Reference](../../Graphics/libgxm-Reference_e.pdf) | 379 | API contracts, structures, states, mapping and completion semantics |
| [Display Overview](../../Graphics/Display-Overview_e.pdf) | 5 | Framebuffer presentation, alignment, refresh and swap timing |
| [Display Reference](../../Graphics/Display-Reference_e.pdf) | 20 | Formats, pixel pitch, pending buffers, waits and callbacks |
| [Shader Compiler Guide](../../Graphics/Shader_Compiler-Users_Guide_e.pdf) | 36 | Cg → GXP, profiles, output conversion, program introspection |
| [Texture Pipeline Guide](../../Graphics/Texture_Pipeline-Users_Guide_e.pdf) | 12 | DDS/PVR/TGA conversion, GXT, compression, swizzling and mipmaps |
| [GXT Overview](../../Graphics/libgxt-Overview_e.pdf) | 7 | GPU-ready texture container, alignment and palettes |
| [GXT Reference](../../Graphics/libgxt-Reference_e.pdf) | 12 | Container structures and texture initialization APIs |
| [Debug Font Overview](../../Graphics/libdbgfont-Overview_e.pdf) | 5 | Guest debug-text rendering workflow |
| [Debug Font Reference](../../Graphics/libdbgfont-Reference_e.pdf) | 16 | Initialization, printing and flush APIs |
| [PGF Overview](../../Graphics/libpgf-Overview_e.pdf) | 11 | Grayscale glyphs, metrics and font selection |
| [PGF Reference](../../Graphics/libpgf-Reference_e.pdf) | 81 | Font API contracts; useful later for shell text |
| [ColladaRenderUtil Overview](../../Graphics/ColladaRenderUtil-Overview_e.pdf) | 6 | Sample scene/rendering workflow using GXM and its patcher |
| [ColladaRenderUtil Reference](../../Graphics/ColladaRenderUtil-Reference_e.pdf) | 16 | Helper-library API; lower priority than the driver/display path |

## Useful implementation anchors

### GPU architecture and texture layouts

[GPU guide, pp.12–25](../../Graphics/GPU-Users_Guide_e.pdf#page=12) describes four
SGX543+ cores with sixteen USSE pipes, distinct vertex and fragment processing,
and tile-based deferred rendering. Figure 1 on p.13 shows the shared master blocks.
The VDM front end has four conceptual command types on p.25: vertex-processing
state, indexed draw, stream link and termination. Their binary encodings are absent.

For data decoding, use **pp.37–42**: linear row pitch normally rounds to eight
texels; swizzled layouts interleave coordinates in Morton order; tiled layouts
use 32×32 texel tiles. Surface and texture dimensions are bounded at 4096×4096
(pp.37, 101, 128). Raster tiles and their sample quadrants are described separately
on pp.140–141. Texture layout tiles should not be confused with scheduling behavior.

### GXM jobs, resource ownership and shaders

[GXM Overview](../../Graphics/libgxm-Overview_e.pdf#page=18) **pp.18–24** explains
asynchronous scenes and four context rings: VDM commands, vertex data/PDS,
fragment data/PDS, and fragment USSE. **pp.28–29** describes separate vertex and
fragment notifications and scene dependencies. **pp.32–34** covers the display
queue and synchronization objects associated with shared buffers.

[GXM Reference](../../Graphics/libgxm-Reference_e.pdf#page=169) **pp.169–170**
specifies copied display-callback data and display-queue completion; **pp.177–178**
distinguishes scene submission from waiting for GPU completion. A display callback
must retain the old buffer's ownership until presentation has released it.
Overview **pp.35–36** and Reference **pp.186–188** describe mapped data using CPU
virtual addresses, 4 KiB mapping alignment, and distinct vertex/fragment USSE offsets.
Render targets and their backing surfaces are separate objects (Overview pp.24–25).

[Shader Compiler Guide](../../Graphics/Shader_Compiler-Users_Guide_e.pdf#page=5)
**pp.5, 7** describes Cg 2.2 input, `psp2cgc`, `.gxp` output and PSP2 profiles.
**p.23** describes `psp2cgnm` parameter/property inspection, not USSE disassembly.
**pp.14–16**, together with GXM Overview **pp.39–40**, explains patching for
vertex fetch, fragment blending and output conversion: a compiler-produced GXP
is not necessarily the final executable shader.

### Display and texture containers

[Display Reference](../../Graphics/Display-Reference_e.pdf#page=4) **p.4** defines
RGBA8888 as API value 3, with little-endian byte order R, G, B, A. **p.6** requires
256-byte framebuffer-base alignment and pitch measured in pixels, in multiples
of 64. Listed dimensions are 480×272, 640×368, 720×408 and 960×544.
**pp.9–11** distinguishes HSYNC/VBLANK update timing and active/pending buffers;
**p.13** describes the free-running 16-bit VBLANK counter. Display Overview **p.5**
gives a refresh rate of 59.94005995 Hz for the documented outputs.
These API values do not establish hardware register encodings.

[GXT Overview, pp.5–7](../../Graphics/libgxt-Overview_e.pdf#page=5) and
[GXT Reference, pp.4–5](../../Graphics/libgxt-Reference_e.pdf#page=4) describe
32-byte file/texture records, file-relative offsets and four encoded texture control
words. Texture data has 16-byte alignment; palettes have 64-byte alignment.
Whole-container alignment is 64 bytes when palettes exist, otherwise 16 bytes.
[Texture Pipeline, pp.4–9](../../Graphics/Texture_Pipeline-Users_Guide_e.pdf#page=4)
connects the container to GPU-ready formats, mipmaps and compression.

For later fixtures, Debug Font Overview pp.4–5 gives an initialization/print/flush
sequence; PGF Overview p.3 covers glyph images and metrics. ColladaRenderUtil
Overview pp.4–6 demonstrates the renderer/patcher workflow and supported asset subset.
These are guest libraries, separate from the debugger's host bitmap font.

## Comparison with the emulator

| Current code | What the references establish | Remaining work |
|---|---|---|
| [SGX register/MMU declarations](../src/hw/soc/soc_internal.h) | Architecture and mapping semantics | Base `0xE2400000`, offsets and flat eight-byte page entries remain assumed; recover hardware/driver protocol |
| [SGX command consumer](../src/hw/soc/sgx.cpp) | Four conceptual VDM command types, four context rings | Current 16-byte NOP/STATE/DRAW/SHADER/TEXTURE/SYNC units are synthetic; fixed framing also exists outside `decode_unit()` |
| SGX synchronous `complete_kick()` | Separate vertex/fragment completion, dependencies, display ownership | Implement actual job/resource descriptors and deterministic completion; simultaneous TA/3D acknowledgement is scaffolding |
| [Display controller](../src/hw/soc/display.cpp) | Active/pending buffers, update timing and VBLANK counter | Current flip is immediate, counter behavior differs, scanout requires synthetic DMA_CONTROL; MMIO map remains assumed |
| [Native IFTU0](../src/hw/soc/iftu.cpp) | PDFs establish API pitch/presentation; actual 1.04 Lowio establishes register bytes | Guest-RAM scanout and bounded DSI-frame turnover/IRQ204/205 implemented; physical status encoding, exact rearm timing and blending remain unresolved |
| [Native DSI0](../src/hw/soc/dsi.cpp) | API refresh rate; actual 1.04 driver establishes IRQ213/sub1 and W1C status | Progressive VIC0 60000/1001 timing and GIC delivery implemented; PHY/packets, shutdown completion, other modes and scanline semantics remain unsupported |
| [Panel blitter](../src/ui/ui_main.cpp) | Explicit format and padded pixel pitch | Fixed: scanout now passes bytes per pixel from the buffer format separately from row stride; regressions cover width480/pitch640/RGBA8888 and width480/pitch1024/RGB565 |
| Display color-bar self-test | Guest-memory scanout can be checked independently | Confirms the presentation path, not real GXM command execution, shaders or LiveArea rendering |

The subsequent boot work fixed the padded-pitch upload defect identified above.
The native SDL color-bar scanout was captured and checked visually; it remains a
synthetic display fixture. No guest SGX command or shader behavior was added.
Native SDL's Metal
renderer presents the debugger; it does not supply a guest SGX shader backend.
The host renderer choice remains separate from the documented guest semantics.

Following the PDF review, actual firmware 1.04 `display.elf` and `lowio.elf`
provided the [native display contract](FIRMWARE_DISPLAY_104.md): IFTU0 at
`E5020000`, DSI0 at `E5050000`, IRQ213/sub1, and the real CPU gzip logo producer.
The embedded white PlayStation bitmap has now also been decoded by the ordinary
guest producer; its RAM bytes match the offline reference. This logo uses the CPU/display path and does
not require a complete SGX shader backend. Minimal native IFTU scanout is now
implemented and tested. Progressive DSI0 vblank timing now raises IRQ213 through
the ordinary GIC; bounded deferred-bank turnover also dispatches IRQ204/205.
Blending and complete hardware timing remain unsupported.
The integrated 617-test binary executes all native Lowio initializers, all six
EMC commands and the genuine GPIO248/sub4 Syscon receive/ACK/stop path. The ARM
RFE return defect is fixed: the unchanged OLED A1 helper retains its stack and
clocks exactly18 TX/RX bytes. The chosen SPI2 undriven-high input is explicitly
an assumption; the guest rejects the supplier ID and stores ready2. Its API
error remains `803F0A03`. Native Display WaitReady, OLED script and head setup
return zero; the guest enables DSI0 and modeled frames advance. The checksum
VLD1/VMOVL/VADD/VPADD/high-D VMOV sequence is fixed. Correct early NVS boot flags
and preservation of native per-core stacks let all four boot predicates return0.
Native producer/gzip/SetFrameBuf execute:1FE000 decoded bytes match the exact
embedded logo. The first deferred submission fills bank1; the next DSI frame
publishes it through physical IRQ204. Native zero ACK, +180 rearm, software
pending clear and replay fill old bank0. One frame per full-word arm and an
internal IRQ latch with raw status0 are declared model choices.
[Native evidence](../build/goal-native-iftu-arm-graphics-evidence.md) and
[integrated evidence](../build/goal-native-iftu-arm-integrated-evidence.md)
record this route. The actual
SDL screenshot (image omitted from source delivery) shows the guest's
white PlayStation logo on black. Offline pixels have never been injected into
guest RAM. This CPU/display milestone does not establish SGX shader execution.

## Missing evidence and next work

The PDFs provide **no recovered MMIO register map, binary VDM/PDS commands,
GPU MMU page-table encoding, or USSE instruction encoding**. The GPU guide
explicitly defers shader instruction details on **p.133**. Its p.141 label `USSE2`
is pipe number 2 in a diagram, not a specification of a second ISA version.

SDK headers (`gxm.h`, `gxm/shader_patcher.h`, `display.h`, `gxt/gxt_conversion.h`)
and tools (`psp2cgc`, `psp2cgnm`, `psp2gxt`) were not found in the workspace.
The supplied firmware does contain `os0/us/libgxm_es4.elf`, `libgpu_es4.elf` and
`os0/kd/display.elf`, `gpucoredump_es4.elf`, `vipimg.elf`, which remain useful
reverse-engineering inputs. See [GPU.md](GPU.md) for the plan and
[STATUS.md](STATUS.md) for the boot milestone; kernel/module bring-up still
precedes a real shell frame.

Prioritize driver/job/mapping traces and protocol recovery, then separate
completion and buffer ownership, texture layouts,
and the real shader/patcher path. Verify recovered behavior with small rendering
fixtures instead of extending synthetic opcodes as if they were hardware.

Two early-document inconsistencies need care: GPU guide p.28 versus p.139 gives
different valid-bit counts for 32-bit vertex indices (22 versus 24); its 2bpp PVRT
tables on pp.76/78 conflict with the 8×4 block description on p.147. Use binary
behavior or later documentation to resolve those details before implementing them.

## Searchable local cache

`build/graphics-reference/` contains one layout-preserving text extraction per
PDF, `inventory.json`, and representative page renders. This generated cache is
not required to build or run the emulator. The original PDFs were not modified.
To refresh an extraction from the repository root:

```sh
mkdir -p build/graphics-reference
pdftotext -layout ../Graphics/GPU-Users_Guide_e.pdf \
  build/graphics-reference/GPU-Users_Guide_e.txt
rg -n 'VDM|USSE|swizzl|tile' build/graphics-reference/GPU-Users_Guide_e.txt
```
