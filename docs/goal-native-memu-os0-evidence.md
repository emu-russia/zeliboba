# Genuine native os0 module milestones — integrated 545

These are read-only captures with every inherited `ZLB_*` variable removed, the ordinary CLI, `--no-rebuild`, and a separate absolute-path clone (`goal-arm-rendezvous-clone.img`). No module/file/path injection, guest register modification, image write, or shared build was performed. Capture scripts and raw logs preserve the inputs.

## Native load lists and relocation

The corrected CPU MemU path returns the actual `/kd/` and `sysmem.skprx` components at `5102398E`. All fourteen core-module load returns and fourteen driver-module load returns are positive native UIDs. The first list returns `R0=0` at `000CC018`; the second returns `R0=0` at `000CC02E`. The earlier `803FF007` results and zero UID arrays are historical and no longer the current frontier.

The independent placement capture stops at the genuine loader's `5101A3E2`: `R0=0` for ScePsp2BootConfig and each of the 28 modules. It saves every native module object and compares its retained PT_LOAD headers with the corresponding genuine decrypted ELF. Each retained header matches exactly. The guest removes zero-memsize PT_LOAD records; its relocation records may normalize memsize to filesz, so only actual retained load segments are compared. The allocation address is the native per-segment record at `module+0xA8+0x74*i+0x60`; original linked VA is record+8. Text and writable data receive independent allocations.

ScePsp2BootConfig itself remains text `000CC000`, BSS `000CD000`. The following table gives actual native VAs, not physical addresses and not a guessed common relocation delta. The second segment's two sizes are ELF file bytes / total memory bytes; a zero file size represents genuine BSS.

| Module | Native UID | Text VA | Text memory size | Writable VA | Writable file / memory size |
| --- | --- | --- | --- | --- | --- |
| sysmem | `0002007B` | `00460000` | `0x393A0` | `000CE000` | `0x6C / 0x12F8` |
| excpmgr | `00020081` | `000E6000` | `0x2390` | `00430000` | `0x0 / 0x41C0` |
| intrmgr | `00020087` | `000EC000` | `0x34B0` | `00500000` | `0x0 / 0x8471C` |
| buserror | `0002008B` | `000E5000` | `0x4E0` | — | — |
| systimer | `00020091` | `000EA000` | `0x1290` | `000E9000` | `0x268 / 0x270` |
| acmgr | `00020095` | `00436000` | `0x1F00` | — | — |
| threadmgr | `0002009B` | `004A0000` | `0x2DEB0` | `00435000` | `0x0 / 0xE8` |
| dmacmgr | `000200A1` | `00438000` | `0x3930` | `0043C000` | `0x0 / 0x29C` |
| smsc_proxy | `000200A7` | `0043E000` | `0x16F0` | `0043D000` | `0x4 / 0x64` |
| authmgr | `000200AD` | `0049A000` | `0x1DD0` | `0049C000` | `0x0 / 0x40C` |
| iofilemgr | `000200B3` | `004D0000` | `0x1D090` | `004EE000` | `0x18 / 0x21D4` |
| modulemgr | `000200B9` | `00588000` | `0xD900` | `0049D000` | `0x10 / 0x4D0` |
| processmgr | `000200BF` | `00598000` | `0xA310` | `0049E000` | `0x120 / 0x368` |
| backtrace | `000200C3` | `004CE000` | `0x1D8C` | — | — |
| stdio | `000200C9` | `0049F000` | `0x8E0` | `004F1000` | `0xD4 / 0xF0` |
| lowio | `000200CF` | `005A8000` | `0xA6D0` | `004F2000` | `0x0 / 0xDBC` |
| syscon | `000200D5` | `004F8000` | `0x7480` | `004F3000` | `0x0 / 0x3D8` |
| oled | `000200DB` | `004F4000` | `0x23C0` | `004F7000` | `0x0 / 0x13C` |
| display | `000200E1` | `005B4000` | `0x6418` | `00585000` | `0x0 / 0x200` |
| sm_comm | `000200E7` | `00586000` | `0x1040` | `00596000` | `0x0 / 0x94` |
| ss_mgr | `000200ED` | `005BC000` | `0x6810` | `00597000` | `0x8 / 0x818` |
| sdif | `000200F3` | `005C8000` | `0xAA20` | `005D8000` | `0x0 / 0xA030` |
| msif | `000200F9` | `005E8000` | `0x9770` | `005A4000` | `0x0 / 0x19B0` |
| gcauthmgr | `000200FF` | `005F8000` | `0xAF40` | `00604000` | `0x0 / 0x4334` |
| sdstor | `00020105` | `0060C000` | `0x5534` | `005A6000` | `0x28 / 0x14FC` |
| rtc | `0002010B` | `00614000` | `0x4AE0` | `005A3000` | `0x0 / 0x140` |
| exfatfs | `00020111` | `00620000` | `0x1D88C` | `00800000` | `0x50 / 0x299610` |
| sysstatemgr | `00020117` | `005C4000` | `0x3758` | `005B3000` | `0xC / 0x10` |

## Actual ThreadMgr callbacks and current stop

The ordinary noname export NID `935CD196` enters `004A1560` with `R0=4`, executes `MOVS R0,#0`, and reaches `004A1562` with `R0=0`. The full ordinary log prints `SceKernelThreadMgr: Starting... OK`. This proves the initial start callback, not completion of its later per-core extension.

The extension export NID `5C424D40` enters `004A19E4` on ARM3, ARM0, ARM1, ARM2. Its actual relocated code calls the native CPU-index provider at `004A4334`, then `BLX 004A0000` at `004A19F2`. Each core reaches `004A0008` with `LR=004A19F7`, `R0=004A0048`, and `D0=7FF8DEAD7F80DEAD`. The initializer's actual bytes at +8 are `10 11 20 F2`, raw word `F2201110`: `VORR D1,D0,D0`. D-register index 1 is legal for the D form; the CPU owner is correcting that decoder. No further module or display start is claimed by this capture.

The actual relocated extension also constructs `R7=00435000` using MOVW/MOVT at `004A1A0E/004A1A16`, independently confirming its separate writable allocation. `004CE000` belongs to Backtrace text. An exploratory contiguous-BSS probe of `004CE000` in the raw log must not be interpreted as ThreadMgr data.

The ordinary full 545 run shows seven additional core-module starts through ThreadMgr after ScePsp2BootConfig, then all four cores halt at this same instruction. It records 10,753 eMMC reads, zero writes/dirty no, 1,081 Bigmac operations, 235 native IRQ wakeups, and zero forced cluster wakeups or counter patches. CMeP remains genuinely asleep at `0080048A`. The image need not be repaired to explain this stop.

## Frozen artifacts

- `goal-arm-native-memu-list-results.py/.log`: original paths, all 28 native UID returns, both list results.
- `goal-arm-native-memu-placement.py/.log/.json`: all 29 relocation returns, native objects, initial ThreadMgr callback, all four extension/initializer paths.
- `goal-native-memu-os0-placement.json`: parsed UID, allocation, linked-VA and ELF size data. Each module's full object is `goal-arm-native-module-NN-object.bin`.
- `goal-arm-native-threadmgr-initializer.bin`: actual guest initializer bytes (0xB0 bytes).
- `goal-arm-native-threadmgr-extension.bin`: actual relocated extension bytes (0x60 bytes).
- `goal-arm-native-threadmgr-moduleinfo.bin`: actual guest module-info/export window (0x120 bytes).
- `goal-native-memu-full.log`: parent ordinary uninterrupted full boot with the same stop and storage/wake counters.

These artifacts establish successful native load/relocation and early starts. They establish neither full kernel initialization nor a displayed guest logo. Driver modules Lowio/OLED/Display are loaded, and their later starts remain the next capture targets.
