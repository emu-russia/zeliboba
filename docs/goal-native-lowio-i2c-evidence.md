# Native Lowio/I2C frontier — integrated 548

Fresh CLI captures use the ordinary integrated 548 binary, remove all inherited `ZLB_*` variables, and boot an absolute-path clone `goal-arm-lowio-clone.img` with `--no-rebuild`. No guest register change, MMIO injection, image write, source edit, or shared build was made.

## Actual Lowio start sequence and blocker

Lowio's actual native UID is `000200CF`, text VA `005A8000` (first PT_LOAD size `A6D0`), writable/BSS VA `004F2000` (size `DBC`). The live MMU maps those bases to PA `40740000` and `4073C000`. Its header at `005AF6D0` identifies `SceLowio`.

Native module_start enters `005A80B0` on ARM0 with `R0=4`, `R1=51184C00`, `LR=51018887`. The first two initializers `005A867C` and `005AA770` run; the third `005AAC48` also returns zero, observed at `005A80CA`. This third initializer maps **ScePwmReg** at `E20C0000`. It is not the I2C routine.

The fourth module_start call is `005A80CE -> 005AB24C`, which calls `005AB024`. This is the native two-controller SceI2c0/1 initializer. It maps both real physical windows and issues native event/object creation and interrupt-registration calls before entering its first hardware idle check:

| Native state | Runtime MMIO VA | Actual translated PA | Window size | Registered native IRQ |
| --- | --- | --- | --- | --- |
| I2c0 `004F21C4` | `280BF000` | `E0500000` | `1000` | 142 (`8E`) |
| I2c1 `004F2204` | `280C0000` | `E0510000` | `1000` | 143 (`8F`) |

`goal-arm-native-lowio-map.log` independently records both `vpa` translations, live state records, genuine `SceI2c0Reg`, `SceI2c1Reg`, `SceI2c0`, `SceI2c1` strings, and the still-preserved second map options on the guest stack with PA `E0510000`.

The runtime code at `005AB1C4..005AB1DC` loads controller0's mapped base, writes its initialization control, and polls idle:

```text
005AB1C4 LDR R3,[R8,#-18]      R8=004F21DC -> R3=280BF000
005AB1CA STR R7,[R3,#2C]      R7=0100F70F
005AB1CC STR.W R9,[R3,#8]     R9=1
005AB1D0 STR.W R9,[R3,#C]     R9=1
005AB1D4 STR R2,[R3,#14]      R2=7
005AB1D6 DSB SY
005AB1DA LDR R4,[R3,#1C]
005AB1DC CMP R4,#0
005AB1DE BNE 005AB1DA
```

Actual before/after captures prove that load returns `FFFFFFFF`, because both physical I2C windows are absent from the bus. `R6=0` is the first iteration; the later increment and `CMP R6,#2` at `005AB208` prove the intended two-controller loop. It has not completed initialization of controller0, so no second-loop hardware execution is claimed. Another 30,000 ordinary slices remain at the same wait. The CPU decodes the load/compare/branch correctly; this wait does not require replacing a function result or interrupt status.

The native controller records have `80027101` at both +18 and +1C for each instance after the object-creation calls. Those values are recorded as observed; successful object creation or a usable later transaction-event path is not claimed. This is separate from the current unambiguous hardware-idle poll.

The IRQ handler supplied by the firmware is Lowio linked `81002F18`, runtime `005AAF18`; the static hardware-contract owner recovered its read/echo-W1C of controller+28 and forwarding into the native event state. No peripheral transaction, successful slave ACK, or I2C completion IRQ is observed by this capture. Any initial model should preserve that boundary.

## Later Lowio initialization — static actual code, not executed yet

After the fourth call returns, module_start proceeds in this exact order, all offsets relative to observed Lowio text `005A8000`:

| Caller PC | Actual target | Recovered behavior |
| --- | --- | --- |
| `005A80D6` | `005AB948` (+3948) | Immediately returns zero (`MOVS R0,#0; BX LR`). |
| `005A80DE` | `005AB9D4` (+39D4) | Queues native `SceCdramInit` callback +398C (`005AB98C`) through a ThreadMgr import, then returns zero. Passing the queue call is not proof the callback ran. |
| `005A80E6` | `005AC06C` (+406C) | Calls DSI mapping/init +3EA4 (`005ABEA4`), then translates negative return to2 and nonnegative to0. Maps DSI0 `E5050000/1000` and DSI1 `E5060000/1000`; registers native IRQ213 and210 with top handler +3AE0 (`005ABAE0`). |
| `005A80EE` | `005AE2C8` (+62C8) | Calls IFTU mapping/init +5D3C (`005ADD3C`), then returns zero. Table +9498 describes IFTU0 A/B (`E5020000/1000`, `E5021000/1000`, shared `E5022000/1000`, IRQ204/205), IFTU1 A/B (`E5030000/1000`, `E5031000/1000`, shared `E5032000/1000`, IRQ206/207), and IFTU2 (`E5040000/1000`, IRQ255). |
| `005A80F6` | `005AF498` (+7498) | Calls CSI/CIF mapping/init +7228 (`005AF228`), then translates its return as DSI does. Table +9624 maps SceCsi0 `E3050000/1000` IRQ160, SceCif0 `E3020000/1000` IRQ166, SceCsi1 `E3060000/1000` IRQ162, SceCif1 `E3030000/1000` IRQ168. It registers native callbacks and performs later Pervasive helper calls; no unseen peripheral readiness value should be invented. |
| `005A80FA..FC` | Return to native loader | Forces module_start R0=0 and returns. Therefore Lowio Start OK alone will not prove every initializer succeeded. Capture the individual returns when reached. |

The DSI/IFTU register map and unsupported timing/blending limits are already recovered in the separate display reports. No speculative model extension is suggested for an unreached routine.

## Display state is still untouched

Neither breakpoint `005AC06C` (DSI init) nor `005AE2C8` (IFTU init) is reached in the fresh run. Read-only named diagnostics show DSI control0/mask0 and IFTU bus control0/mode0. There is no Lowio Start OK, no OLED start callback, no Display start/work callback, no guest logo allocation or pixel decode, and no native framebuffer presentation yet.

The future graphics probe uses actual independently allocated module segments:

| Module | UID | Text VA | Writable/BSS VA |
| --- | --- | --- | --- |
| Lowio | `000200CF` | `005A8000` | `004F2000` |
| OLED | `000200DB` | `004F4000` | `004F7000` |
| Display | `000200E1` | `005B4000` | `00585000` |

`goal-arm-native-display-probe.py` is prepared, syntax-checked and not run. It boots a separate absolute clone `goal-arm-display-clone.img`, first stops at the genuine driver-list success and validates all three guest module-info names at the captured text bases. It then records native init/workqueue/OLED-ready, guest allocation/gzip/SetFrameBuf, DSI vblank and fade milestones. It reads named IFTU/DSI diagnostics, can save only actual guest-generated logo RAM, and never supplies pixels, enables a framebuffer, creates a status, or switches an IFTU bank.

## Frozen artifacts

- `goal-arm-native-lowio-start.py/.log/.json`: actual module_start/call sequence, before/after polling, untouched display state and prolonged wait.
- `goal-arm-native-lowio-map.py/.log`: both native mappings, live two-controller records and raw hardware registers.
- `goal-native-memu-os0-evidence.md` and `goal-native-memu-os0-placement.json`: all28nativeUIDs/29relocationreturns and exact segment allocation records from545.
- `goal-arm-native-display-probe.py`: ready future ordinary graphics capture; separate clone already exists.
- Parent `goal-native-neon-full.log`: uninterrupted ordinary548 run, all14 core starts and Stdio start, matching I2C frontier.
