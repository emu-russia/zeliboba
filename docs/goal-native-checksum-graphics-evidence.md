# Native 1.04 graphics frontier after checksum CPU support (611 tests)

Fresh private APFS clone `goal-arm-native-syscon-oled-checksum-clone.img`; every inherited `ZLB_*` variable removed; unchanged guest code and API results. The bounded capture recorded 70 native checkpoints, followed by one 300,000-slice no-hit window, then exited cleanly. No guest memory/register writes or extracted-image injection were performed. Existing model stand-ins and the explicitly chosen undriven-high SPI2 wire input remain active and are not hardware proofs.

## Reached native behavior

- Native secondloader service/handoff reaches secure-kernel and genuine NSKBL; driver-list return at `CC02E` is zero.
- GPIO248 parent/sub4 invokes the genuine Syscon response callback and ACK/stop path. With the checksum load/store family supported, Syscon result/transport checkpoints `4FA728` and `4FA73A` both retain R0=0.
- OLED A1 returns zero at `4F47AA` with SP intact; SPI2 records exactly one transfer, 18 bytes TX / 18 bytes sampled RX, no unread bytes. The explicit high input yields the firmware's rejected-panel `ready=2`. GetDDB still returns `803F0A03`; native OLED initialization script and Display OLED setup nevertheless return zero.
- Native IFTU SetInput `5AE5E0`, both plane enables, DSI start and head-enabled store execute. DSI parent IRQ `5ABAE0` invokes Display vblank callback `5B6940`; native counter increment `5B6956` executes.

## Concrete logo gate

Display `5B4FDA` calls its UpdateMode predicate (NID `B0E1FC67`). At return `5B4FDE`, ARM1 R0=1, R1=00004000 (sysroot), R2=000CE328 (Sysmem global), and R3=000047C0 (actual preserved boot parameter). Inspection proves:

| Observed input | Value |
| --- | --- |
| parameter VA / PA | `000047C0` / `403047C0` |
| parameter bytes +30..33 | `00 00 00 FF` |
| parameter word +6C | `00000005` |
| parameter word +C0 | `00000060` |
| parameter word +C4 | `0000FF14` |
| sysroot+3C | `000047C0` |

Genuine sysmem.elf provider linked `8101CA7C` loads byte +30, subtracts FF, and sets R0=1 when it differs. The real Display branch therefore goes directly to `5B4FE0` and returns from initialization. Later ExternalBootMode, unnamed bit16, and UsbEnumWakeup predicates are not executed; neither are logo producer `5B7048`, gzip, or SetFrameBuf. No framebuffer is produced/exported. This is an observed UpdateMode gate; the +6C bit0 may independently imply external mode, but it is not the executed gate in this run. Boot-parameter provenance and authoring belong to the separate read-only audit; this report proposes no forced flag/predicate changes.

## Final bounded state

ARM0/1/2 are ThreadMgr idle at `47969C`; ARM3 is the genuine Syscon timed worker loop at `4F9DC4`, with SP=A6D20 and no undefined halt. CMeP sleeps at `80048A`. No new CPU fault is logged. DSI0 has 21 modeled progressive frames, status0/mask2/IRQ213low. IFTU banks all have PA0 and blank=1, format10/960x544/padding0; bus1/modeA/selectors0, plane50=1/58=108/180=1. There is no supported enabled guest-RAM scanout. No deferred IFTU turnover or plane IRQ is inferred from this run.

The clone is clean: 10,753 card reads, zero writes. SPI0 has 275 valid framed transfers and ready edges; EMC has six completed commands, no rejected commands; I2C reset/idle paths remain ready. Artifact sources: `goal-arm-native-syscon-oled-checksum.log` / `.json`, `goal-native-display-predicate-static.txt`. Three predicate names are corroborated by the primary [VitaSDK NID database](https://github.com/vitasdk/vita-headers/blob/master/db/360/SceSysmem.yml); unknown `7918D44E` remains named by its exact bit16 operation.
