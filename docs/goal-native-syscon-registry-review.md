# Native Syscon startup / ARM0 registry review (560-test binary)

Read-only capture using the current integrated `build/bin/zeliboba`, an environment with all `ZLB_*` variables removed, and a fresh APFS clone `build/goal-arm-syscon-registry-clone.img` of `build/goal-emmc.img`. No source or firmware edits, register injection, builds, or guest service substitutions were made for this investigation.

## Result

The 300,000-slice ARM0 snapshot was ordinary forward progress in native name-tree insertion/reuse. It did not establish a stalled byte compare or class-registry lock. After the captured registry operations and 50,000 additional native slices, UART reports genuine `SceSyscon`, `SceOled`, `SceDisplay`, and `SceSblSsSmComm` `Starting... OK`. ARM0 subsequently reaches the native idle/WFE routine at `0x0047969C` (LR `0x004A1253`). There is no demonstrated ARM instruction defect here.

## Exact code and data

- `syscon.elf` PT_LOAD0: linked `0x81000000`, file offset `0xA0`, runtime `0x004F8000`, size `0x7480`. Module start is Thumb `0x004F9DF4` (module-info start pointer `0x1DF5`). The fresh capture reaches this code with LR `0x51018887`.
- `sysmem.elf` PT_LOAD0: linked `0x81000000`, file offset `0x100`, runtime `0x00460000`.
- `0x0046586C` is a native name-tree insertion/reuse routine. It calls bounded A32 `strncmp` at `0x00462630` with limit 31, tests its signed result at `0x00465882`, and follows the left/right child at node offsets `0x18`/`0x1C`. Equal-name code at `0x00465904` increments the reference count at node `+0x14` and returns the existing node. Captured searches reach NULL and take the native new-node path.
- The compared strings and nodes are genuine RAM: input `0x004FE714` is `SceSpi0Reg`, `0x004FE720` is `SceSysconCmdclr`, `0x004FE730` is `SceSyscon`; node `0x002015C0` contains `SceSyscon`.
- Exact byte loop: `0x00462648 E4D02001` = `LDRB r2,[r0],#1`; `0x0046264C E4D13001` = `LDRB r3,[r1],#1`; bound compare at `0x00462650`, conditional NUL and byte compares at `0x00462654/58`, conditional loop at `0x0046265C`, subtraction and `BX LR` at `0x00462660/64`.

The old snapshot had R0 `0x004FE739`, R1 `0x002015C9`, R2/R3 `0x6E` at the first LDRB. These are the addresses of the NUL bytes after the nine-character equal string `SceSyscon`, immediately after comparing its final `n`. That state is expected before the next NUL read; it is not an invalid-address or compare-loop signature.

## Captured execution checks

All 15 captured native `strncmp` entry/return pairs agree exactly with an independent 31-byte unsigned-byte comparison of the captured input RAM. Both positive and negative comparisons traverse the expected tree direction. In particular, `SceSysconCmdclr` versus node `SceSyscon` returns `0x43` (the input `C` minus the node NUL) after ten byte reads, proving the relevant NUL termination / condition sequence works. The tree then continues and reaches NULL/new-node allocation.

The limited detailed breakpoint series ends at the first `SceSyscon` name-tree entry; its exact zero return was not separately recorded. Subsequent native Syscon/OLED/Display successful startup and ARM0 idle are directly recorded. No claim is made that all pending secure-worker hardware waits or display presentation have completed.

Artifacts: `goal-arm-native-syscon-registry.py`, `.log`, `.json`; fresh private clone as named above. The clone remains clean: 10,753 eMMC reads, zero writes.
