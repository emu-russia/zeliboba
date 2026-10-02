# Native graphics and thread context: integrated 560

Frozen read-only evidence from the integrated 560-test binary. Both captures remove all `ZLB_*` environment variables and use separate fresh APFS clones of `build/goal-emmc.img`. No source/firmware edits, register injection, framebuffer extraction injection, forced wake, or host service return was introduced by these probes. The display clone finishes with 10,753 card reads, zero writes, and dirty=no.

## What now runs

The native Lowio module reaches every initializer return with R0=0, including I2C, DSI, IFTU and its final CSI/CIF call. The actual queued CDRAM callback executes on ARM2, distinct from its queue wrapper's forced-zero result; callback completion is not established here. Lowio/Syscon/OLED/Display/SblSsSmComm UART `Starting... OK` messages are recorded independently from asynchronous worker completion.

I2C creation now returns genuine positive UIDs: mutex0 `10A5B`, mutex1 `10A5D`, event0 `10A5F`, event1 `10A61`. The native I2C0 state at VA `4F21C4` contains mapped register VA `280CF000`, memory-block UID `10A57` and IRQ142; I2C1 state at `4F2204` contains VA `280D0000`, UID `10A59`, IRQ143. Hardware summaries record BUSY=0/reset-command7, one reset per controller, no transfer writes and IRQs low. No I2C slave exchange is claimed.

Independent post-fix TPIDR capture proves the genuine writer at `4A1DE8` persists each core's R9 into TPIDRPRW: ARM3=`6B828`, ARM0=`6BA28`, ARM1=`6BC28`, ARM2=`6BE28`. Both creation providers subsequently read ARM0's `6BA28` and return the positive UIDs above. Labels in `goal-arm-native-tpidr-postfix.log` calling the shared epilogue PCs `4AC3C0`/`4ADB34` an 'illegal-context return' were inherited from the prior failing capture: those PCs also execute after successful creation. The actual provider values and caller R0 establish success.

## Current display gate

Native Display enqueues `SceDisplayInit` with R0=`10023`, R1=`5B9920`, R2=`5B4F15`, R3=0. Before its module wrapper overwrites R0 with zero, the enqueue call returns positive UID `10AA7`. The genuine Display work callback executes at `5B4F14` on ARM1, then calls OLED readiness and reaches `4F48D2`; the OLED global at VA `4F7004` remains zero.

The same capture records ARM3 halting on `FFC70E1F` at genuine Syscon VA `4F9BEA`. Offline native code identifies this instruction as `VMOV.I8 D16,#0xFF` during packet construction. This is the demonstrated CPU stop preceding the forthcoming Syscon/GPIO/SPI investigation; no ready interrupt or successful packet result is asserted.

OLED work also executes on ARM1 at `4F4780`, but has TPIDRPRW=`6BC28`, whereas Display work uses `44B228`. Their sharing a physical core does not establish that OLED work returned: these are different scheduled native contexts. Display is directly observed waiting for OLED readiness; a direct Display wait on the secure CDRAM callback is not established.

After the 20 milestones below, three additional 300,000-slice windows reach no Display OLED-ready return, setup return, native logo producer, gzip, SetFrameBuf, fade or vblank-handler breakpoint. Final native DSI0 control/mask/status and IFTU0 bus/mode/selected banks remain zero, including all bank addresses/dimensions. No guest framebuffer or logo exists in this capture. CPU instruction progress continues in worker polls while emulated time remains approximately 0.276s; added slice count alone does not prove a native timeout or scheduling delay elapsed.

## Native placements and milestones

The probe validates genuine module headers before using recovered addresses. Lowio text/BSS are `5A8000`/`4F2000`, OLED `4F4000`/`4F7000`, Display `5B4000`/`585000`. Text VA is separate from BSS VA.

| Native milestone | Core | VA PC | R0 |
|---|---|---|---|
| Lowio module_start | arm0 | `005A80B0` | `00000004` |
| Lowio first init returned | arm0 | `005A80BA` | `00000000` |
| Lowio second init returned | arm0 | `005A80C2` | `00000000` |
| Lowio PWM init returned | arm0 | `005A80CA` | `00000000` |
| Lowio I2C init returned | arm0 | `005A80D2` | `00000000` |
| Lowio immediate init returned | arm0 | `005A80DA` | `00000000` |
| Lowio CDRAM work enqueue returned | arm0 | `005A80E2` | `00000000` |
| Lowio DSI init | arm0 | `005AC06C` | `00000004` |
| Native CDRAM work callback | arm2 | `005AB98C` | `00000000` |
| Lowio DSI init returned | arm0 | `005A80EA` | `00000000` |
| Lowio IFTU init | arm0 | `005AE2C8` | `00000004` |
| Lowio IFTU init returned | arm0 | `005A80F2` | `00000000` |
| Lowio all init calls returned | arm0 | `005A80FA` | `00000000` |
| OLED module_start | arm0 | `004F4834` | `00000004` |
| OLED work callback | arm1 | `004F4780` | `00000000` |
| Display module_start | arm0 | `005B4CF8` | `00000004` |
| Display enqueue-work call | arm0 | `005B4D1A` | `00010023` |
| Display enqueue-work result before overwrite | arm0 | `005B4D1E` | `00010AA7` |
| Display init work callback | arm1 | `005B4F14` | `00000000` |
| OLED readiness poll | arm1 | `004F48D2` | `00000000` |

## Reproducible artifacts

- `goal-arm-native-display-probe.py`, `.log`, `.json`; private image `goal-arm-display-560-clone.img`.
- `goal-arm-native-tpidr-postfix.py`, `.log`, `.json`; private image `goal-arm-tpidr-560-clone.img`.
- `goal-native-spi-gpio-evidence.md` and genuine Syscon/OLED byte listings identify the next native packet/ready-gate observations.

The earlier 548 TPIDR/I2C error reports remain historical evidence. This report supersedes their current frontier, without rewriting their captured failing values.
