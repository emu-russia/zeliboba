# Firmware 1.04 bootconfig continuation and next kernel path

Read-only static recovery, correlated with integrated native captures. No firmware modification, source change, host module injection, or new boot run was performed for this report.

## Current live boundary

`goal-arm-native-bootconfig-list-results.log` records R0=`0x803FF007` at CC0018 and CC002E: both core and driver batch loads failed. Bootconfig ignores these returns and its first init callback returns zero, producing the misleading `Starting... OK`. The later correctly resolved import at CC00FC reaches NSKBL 51010710 and calls a zero sysroot+0x40 pointer. This proves the CPU/import branch path, but does not demonstrate kernel modules loaded. Graphics owns the native open-path failure diagnosis.

## Bootconfig load/start order

First PT_LOAD is ELF file offset A0, linked VA 81000000, length 658. Runtime capture maps it to CC000. The tables hold fourteen 12-byte records each: a relocated name pointer plus two u32 fields. Names are relative module basenames used by the native os0 loader. UID arrays occupy linked VA 81001038 (core) and 81001000 (driver).

| Table | Names, in actual binary order |
| --- | --- |
| Core +394 | sysmem.skprx, excpmgr.skprx, intrmgr.skprx, buserror.skprx, systimer.skprx, acmgr.skprx, threadmgr.skprx, dmacmgr.skprx, smsc_proxy.skprx, authmgr.skprx, iofilemgr.skprx, modulemgr.skprx, processmgr.skprx, backtrace.skprx |
| Driver +520 | stdio.skprx, lowio.skprx, syscon.skprx, oled.skprx, display.skprx, sm_comm.skprx, ss_mgr.skprx, sdif.skprx, msif.skprx, gcauthmgr.skprx, sdstor.skprx, rtc.skprx, exfatfs.skprx, sysstatemgr.skprx |

All 28 original SELF files and corresponding decoded ELF files exist in the supplied os0 tree. A missing input filename does not explain the two current batch errors.

| Runtime breakpoint | Native operation / useful evidence |
| --- | --- |
| CC0014 / CC0018 | First batch load import CC011C → Thumb 510012D8; return currently 803FF007. |
| CC002A / CC002E | Second batch load, same import; return currently 803FF007. |
| CC0080 / CC0084 | Start core batch through CC012C → Thumb 510012F4, then call CC00FC. |
| CC0084 / 5101071E | CC00FC, SceKblForKernel NID 1DB28F02, invokes sysroot+40; current target zero. |
| CC0088 | CC014C → 510012E4 clears native batch-error word 5102B024. |
| CC008C / 510107D4 | CC00EC, NID 161D6FCC, unconditionally invokes sysroot+C4. |
| CC0090 / 51010F94 | CC016C, NID C7B77991, unconditionally invokes sysroot+264. |
| CC00AA / CC00AE | Starts driver batch through CC012C → 510012F4; SysStateMgr is last. |
| 51001498, 510013EC, 5100140C | ARM0 calls native module-start helper 510194C0; inspect module and start target. |
| 510014CA, 51001472, 51001442 | Secondary cores call each module extension-start callback. |

Fixing or substituting sysroot+40 alone would still leave the other two required callbacks and all failed modules absent. None of these calls should be replaced by a host success return.

## Native SysStateMgr continuation

All addresses below are *linked* addresses from sysstatemgr.elf; its future live relocation base is not yet known. Convert offsets using the actual loaded text base. PT_LOAD0 begins at ELF offset A0. Module-info offset 25D0 has module-start offset 1, so its Thumb start is linked 81000000.

| Linked address / offset | Native behavior |
| --- | --- |
| 81000012 / +12 | Calls import 8100235C, Modulemgr NID FDD7F646 (`ksceKernelFinalizeKbl` in the supplied later-version NID database). |
| 81000082 / +82 | Calls import 8100231C, Modulemgr NID 01360661; 1.04 provider is linked Modulemgr 81005B10. This mounts its in-memory bootimage filesystem. |
| 81000094..9E / +94..9E | Selects string `os0:psp2config.skprx` at 81002D20 and calls internal config loader 81000FD8. Development/external fallback paths are host0, sd0, ux0. |
| 81000FD8 / +FD8 | Initializes parsing context, calls 81000D2C to obtain configuration data, then 81000504, 81000F08, 810009CC, and cleanup 81000B60. |
| 81000D3A / +D3A | Opens configuration path through import 8100238C, Iofilemgr NID 75192972. Repeats only for native retry error 8009000A. |
| 81000EC8..EF4 / +EC8..EF4 | Checks read header against 00454353 (`SCE`); genuine SELF config takes 81000414. |
| 810000C0 / +C0 | Calls Modulemgr NID 9C838A6B via 8100233C: unmounts bootfs after config execution. |
| 81000106 / +106 | Calls 81002254; this reports state through Sysroot/DebugLed imports then exits/deletes the current thread via Threadmgr NID 1D17DECF at 81002264. The fallback loop at 81002268 is terminal. |

The database names above are corroborating labels from supplied db.yml (later firmware); the actual 1.04 provider disassembly and exact NIDs establish the relevant behavior. There is no evidence of an architectural CPU fault in this static continuation.

## SceKernelBootimage is an embedded-module container

bootimage.elf e_entry=0 identifies the module-info at the beginning of PT_LOAD0, not executable code. Module start/stop are FFFFFFFF. Export library SceKernelBootimage (NID 17E65BD7) has zero functions and two data markers: NID C08FC9B5 at linked end 812A542C, NID DF5E79B8 at linked start 810000C8. It is not a flat kernel entry to execute.

Native Modulemgr export NID 01360661 at linked 81005B10 loads `os0:kd/bootimage.skprx` (path at 8100B988) through its loader 810040F0 at 81005B2A. At 81005B62 it queries the loaded module object; text pointer +108 and text size +114 determine the span. Its parser at 81005BA2..BC6 reads each u32 payload length, uses path=record+4 and ELF payload=record+104, then advances by 104+length. This exactly matches all 42 recovered records and terminates at PT_LOAD end 2A542C. The guest memory-backed reader later uses the intentional descriptor token 7F7F7F7F (81005AD4), not a real SDIF descriptor.

| Container segment offset | Original path | ELF payload offset | Payload bytes |
| --- | --- | --- | --- |
| C8 | os0:kd/clockgen.skprx | 1CC | DB0 |
| F7C | os0:kd/idstorage.skprx | 1080 | 37CC |
| 484C | os0:kd/ctrl.skprx | 4950 | 48A8 |
| 91F8 | os0:kd/touch.skprx | 92FC | 80EC |
| 113E8 | os0:kd/motion.skprx | 114EC | D2CC |
| 1E7B8 | os0:kd/codec.skprx | 1E8BC | 29D0 |
| 2128C | os0:kd/audio.skprx | 21390 | D808 |
| 2EB98 | os0:kd/hpremote.skprx | 2EC9C | 1850 |
| 304EC | os0:kd/power.skprx | 305F0 | DDCC |
| 3E3BC | os0:kd/usbd.skprx | 3E4C0 | E698 |
| 4CB58 | os0:kd/udcd.skprx | 4CC5C | BBA0 |
| 587FC | os0:kd/usbserv.skprx | 58900 | 3260 |
| 5BB60 | os0:kd/usbmtp.skprx | 5BC64 | 4920 |
| 60584 | os0:kd/mtpif.skprx | 60688 | 2520 |
| 62BA8 | os0:kd/post_ss_mgr.skprx | 62CAC | 66F4 |
| 693A0 | os0:kd/update_mgr.skprx | 694A4 | 115DC |
| 7AA80 | os0:kd/regmgr.skprx | 7AB84 | 1B21C |
| 95DA0 | os0:kd/fwloader.skprx | 95EA4 | EAC |
| 96D50 | os0:kd/vnz_wrapper.skprx | 96E54 | 11400 |
| A8254 | os0:kd/error.skprx | A8358 | 2638 |
| AA990 | os0:kd/npdrm.skprx | AAA94 | A654 |
| B50E8 | os0:kd/ulobjmgr.skprx | B51EC | 1DD4 |
| B6FC0 | os0:kd/net_ps.skprx | B70C4 | 3AE9C |
| F1F60 | os0:kd/gps.skprx | F2064 | 62B0 |
| F8314 | os0:kd/bbmc.skprx | F8418 | 587F8 |
| 150C10 | os0:kd/wlanbt.skprx | 150D14 | C8DC |
| 15D5F0 | os0:kd/bt.skprx | 15D6F4 | 25648 |
| 182D3C | os0:kd/avcodec.skprx | 182E40 | 1AEF4 |
| 19DD34 | os0:kd/audioin.skprx | 19DE38 | 61BC |
| 1A3FF4 | os0:kd/ngs.skprx | 1A40F8 | 1E854 |
| 1C294C | os0:kd/gpu_es4.skprx | 1C2A50 | F124 |
| 1D1B74 | os0:kd/gpuinit_es4.skprx | 1D1C78 | 139E0 |
| 1E5658 | os0:kd/compat.skprx | 1E575C | 16DF0 |
| 1FC54C | os0:kd/camera.skprx | 1FC650 | 179AC |
| 213FFC | os0:kd/coredump.skprx | 214100 | D3F0 |
| 2214F0 | os0:kd/av_config.skprx | 2215F4 | 5A2C |
| 227020 | os0:kd/fios2.skprx | 227124 | 5DB8 |
| 22CEDC | os0:kd/pfsmgr.skprx | 22CFE0 | 235AC |
| 25058C | os0:kd/appmgr.skprx | 250690 | 4AF48 |
| 29B5D8 | os0:kd/sysmodule.skprx | 29B6DC | 2E90 |
| 29E56C | os0:kd/vshbridge.skprx | 29E670 | 59A4 |
| 2A4014 | os0:kd/tty2uart.skprx | 2A4118 | 1314 |

The missing individual files named by psp2config are therefore usually expected: they are present under their exact paths inside bootimage. GPU ES4 and gpuinit ES4 are embedded; GPU ES3 variants are absent and occur only inside the `KERMIT_REV_ES3_X` branch. No claim is made here that a particular revision predicate has executed.

## System configuration and final process

The provided `fs_dec/os0/psp2config.skprx.seg00` is the decoded SceKernelPsp2Config data segment (8A8 bytes). Its ASCII script starts at offset CA. The original SELF `fs/os0/psp2config.skprx` exists, although there is no psp2config.elf. The script selects board/boot-mode branches and loads the embedded modules. Normal final process is `vs0:vsh/shell/shell.self`, which exists in the supplied tree. `os0:ue/safemode.self` also exists. Manufacturing sd0/ux0 diagnostics and ud0/ur0 update payloads do not exist in this static extraction; they are conditional external-media/update paths.

```text
# PSP2 System Configuration for Release
#
# [NOTICE]
# 
# This configuration is only for kernel_boot_loader_release.self.
#

load    os0:kd/clockgen.skprx
#load	os0:kd/syscon.skprx
#load	os0:kd/rtc.skprx

#load	os0:kd/sm_comm.skprx
#load	os0:kd/ss_mgr.skprx
load	os0:kd/idstorage.skprx

load	os0:kd/ctrl.skprx
load	os0:kd/touch.skprx
load	os0:kd/motion.skprx
load	os0:kd/codec.skprx
load	os0:kd/audio.skprx
load	os0:kd/hpremote.skprx
load	os0:kd/power.skprx

load	os0:kd/usbd.skprx
load	os0:kd/udcd.skprx
load	os0:kd/usbserv.skprx
load	os0:kd/usbmtp.skprx
load	os0:kd/mtpif.skprx

load	os0:kd/post_ss_mgr.skprx
load	os0:kd/update_mgr.skprx
load	os0:kd/regmgr.skprx

if USB_ENUM_WAKEUP
load	os0:kd/enum_wakeup.skprx
else

load	os0:kd/fwloader.skprx
load	os0:kd/vnz_wrapper.skprx
#load	os0:kd/applier.skprx
- load	os0:kd/mgkeymgr.skprx

load	os0:kd/error.skprx

#load	os0:kd/gcauthmgr.skprx
load	os0:kd/npdrm.skprx

load	os0:kd/ulobjmgr.skprx

load	os0:kd/net_ps.skprx
load	os0:kd/gps.skprx
load	os0:kd/bbmc.skprx
load	os0:kd/wlanbt.skprx
load	os0:kd/bt.skprx

load	os0:kd/avcodec.skprx
load	os0:kd/audioin.skprx
load	os0:kd/ngs.skprx

if KERMIT_REV_ES3_X
load   os0:kd/gpu_es3.skprx
load   os0:kd/gpuinit_es3.skprx
endif

if KERMIT_REV_ES4_X
load   os0:kd/gpu_es4.skprx
load   os0:kd/gpuinit_es4.skprx
endif

- load	os0:kd/compat.skprx
load	os0:kd/camera.skprx
load	os0:kd/coredump.skprx
load	os0:kd/av_config.skprx
load	os0:kd/fios2.skprx
load	os0:kd/pfsmgr.skprx
load	os0:kd/appmgr.skprx

load	os0:kd/sysmodule.skprx
load	os0:kd/vshbridge.skprx
load	os0:kd/marlin_hci.skprx
- load	os0:kd/tty2uart.skprx

if MANUFACTURING_MODE
- spawnwait	sd0:psp2diag.self
- spawnwait	ux0:psp2diag.self
endif

if SAFE_MODE
spawnwait	os0:ue/safemode.self
endif

if UPDATE_MODE
if UD0_EXIST
spawn ud0:PSP2UPDATE/psp2swu.self
else
spawn ur0:PSP2UPDATE/psp2swu.self
endif
end
endif

- appspawn vs0:vsh/shell/shell.self SHELL_BUDGET_ID

endif
end

# Local variables:
# mode: c
# tab-width: 4
# c-basic-offset: 4
# coding: utf-8
# End:
```

## Artifacts and confidence

Primary evidence is the genuine 1.04 decoded binaries, exact native live captures, and successful record-boundary parsing. Static evidence does not claim those future modules started. Source files were left unchanged.

- goal-integrated-bootconfig-imports-capture.log / goal-integrated-bootconfig-linked-imports.txt: actual import binding.
- goal-arm-native-bootconfig-list-results.log: actual failed loads.
- goal-nskbl-bootconfig-continuation-helpers.txt: native callback helpers and multicore start dispatcher.
- goal-sysstatemgr-native-start-static.txt / goal-sysstatemgr-config-parser-static.txt: linked start/config path.
- goal-modulemgr-bootimage-static.txt / goal-modulemgr-bootimage-init-static.txt: native container mount/reader and Modulemgr start.
- Binary SHA256 bootimage.elf: 4a5bb59a4ac346c65320b843f3b538a7222a059b128436cc816cda4e69819ba9.
- Binary SHA256 psp2bootconfig.elf: 9f1218a6869e52dc69282acd00929c6dfc0ea756fc859db8944a3a81ae9d9841.
