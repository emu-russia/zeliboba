# Native IFTU publication and guest logo — 617-test integrated build

Fresh `iftu-arm` read-only probe on the frozen all-target macOS build; full suite **617 passed, 0 failed**. All inherited `ZLB_*` variables were removed. The image was a fresh APFS clone of `build/goal-emmc.img`; no watchpoints, guest RAM/register writes, forced return values, or logo injection were used. The probe exited normally after 106 checkpoints and one fixed 300,000-slice no-hit window.

## Guest pixels

Genuine Display producer `005B7048` allocates guest VA `10000000` → PA `1C000000`. Its unchanged gzip call returns `001FE000` at `005B70B0`. The debugger then exports that guest RAM only. The exported 2,088,960 bytes are 960×544 RGBA and exactly equal the independent offline firmware reference:

`SHA256 80c43bdc43d6fcc7f1419960726e6dc3e6be906ae0faac2cdf060204717a6979`

Raw export: `goal-arm-native-syscon-oled-iftu-arm-guest-framebuffer.rgba`. PNG rendered from those exported bytes: `goal-arm-native-syscon-oled-iftu-arm-guest-framebuffer.png`. The reference was used only for comparison; it was never supplied to the guest. The image alpha bytes are zero; existing SDL presentation converts these four-byte inputs to opaque RGB.

## Independent macOS SDL presentation

The separate fresh SDL run visibly presents the **white PlayStation logo on black** in the actual F7 guest display panel: SDL screenshot (image omitted from source delivery). Root, the hardware agent and this graphics reviewer inspected that screenshot. Its header reports a 960×544 guest framebuffer, stride3840, four-byte RGBA8888; the SDL startup log confirms the Metal renderer. This is an application screenshot of the guest scanout, separate from the raw RAM export and the instrumented native-handler trace below.

The SDL process executes ordinary `runm 1000000` on its own fresh clone, then captures the panel. It exits0 in **108.528 seconds** with **10,753 reads / 0 writes**. Source image size and modification time remain unchanged. [SDL run metadata](goal-native-iftu-arm-macos-ui.json) records the command, timing, framebuffer format, image checks and visual assessment; [startup log](goal-native-iftu-arm-macos-ui.log) records Metal initialization. No offline logo was injected into either run.

## Actual native submission, interrupt and replay

Native cold parameter VA `000047C0` preserves bytes `FF FF 00 FF` at +30 and word `00000004` at +6C. All four native Display logo predicates return zero. `SetFrameBufInternal` returns zero at `005B70E2`; it programs the opposite A bank1 with PA `1C000000`, format `10`, 960×544, padding0 and blank0 while currentbank remains0. Lowio A record VA `004F2298` (PA `4073C298`) acquires its genuine pending value `80000000`, recording oldbank0.

| Point | Native PC/core | Hardware/software observation |
| --- | --- | --- |
| SFB return | `005B70E2`, ARM1 | A bank1 valid; current0; software pending `80000000`; no scanout yet |
| Actual IRQ204 entry | `005AD99C`, ARM0, R0=`CC` | DSI frame8→9; A +4=`2`/current1; scanout A bank1; selector stays0 |
| Zero ACK before | `005AD9BA`, ARM0 | GIC pending word6 at `1A001218`=`00001000` (IRQ204 bit12) |
| Zero ACK after | `005AD9BC`, ARM0 | GIC pending word6=0; opaque raw +40 remains0 |
| Cached rearm | `005AD9BE`→`005AD9C2`, ARM0 | Unchanged native write of cached optional flag1 to plane +180 |
| Old-bank replay call | `005AD9D8`, ARM0 | Native code has cleared SWpending to0 and calls bank helper for oldbank0 |
| Old-bank replay return | `005AD9DC`, ARM0 | A bank0 now also contains PA `1C000000`, blank0; active/current remains1 |
| First submission vblank return | `005B70F6`, ARM1, R0=0 | Both bank descriptors match; scanout A bank1/current1 |

Final one-window state: DSI0 has22 frames; A current0 and active scanout bank0 agree, both banks hold the logo descriptor, manual selector remains0, pitch3840. No undefined CPU messages occur in the final window. The cloned image stays clean with **10,753 reads / 0 writes**. Final fade/producer return is outside this bounded capture; it was not extended to seek them.

## Evidence and limits

Full observational log and checkpoint registers: `goal-arm-native-syscon-oled-iftu-arm.log` / `.json`. Extracted register, native software, GIC and scanout checkpoints plus final ARM PCs: `goal-native-iftu-arm-graphics-evidence.json`. Early blank/B-plane interrupts were preserved as separate observations; logo-specific checkpoints were reinstalled only when the actual guest SetInput descriptor PA was `1C000000` and retained through native pending/replay. Guest pending was inspected by the debugger and is not an input to the hardware model.

The production IFTU implementation uses the explicit bounded model reviewed by all source owners: one ordinary01-mode publication per actual full-word +180=1 write on a DSI0 frame boundary, coherent +4/current before normal physical IRQ204/205, native full-word zero ACK, and native rearm. It does not recover the exact physical silicon status encoding or settle continuous-versus-rearmed hardware semantics. Raw +40 therefore reads0 with an internal completion latch. Full A/B blending, fade composition, interlaced modes and general deferred timing remain unsupported. This proves the reached native guest path and model publication, not an exhaustive hardware validation.
