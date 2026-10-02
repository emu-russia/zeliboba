# Bounded retail first-loader smoke — IFTU arm 617

2026-10-01. Fresh unique APFS clone `goal-native-iftu-arm-retail.img` of the unchanged `goal-emmc.img`; raw log `goal-native-iftu-arm-retail.log`, commands `.cmd`, argv/exit/timing `.run.json`. All inherited ZLB variables were removed. Only the explicitly requested first-loader file was selected; no guest flag/MMIO writes, watchpoints, stage changes or service overrides were issued. Existing default board substitutions remain active and are visible in the log. The run ends after exactly 300000 machine slices; it was not extended.

Input: `../dumps/pch-5c-cold_first_loader.bin`, 16384 bytes, SHA256 3ac05d08eacd8398ea56f19c20391b14781ff3a2ea4d6052fd653b4e917caba6. This equals the user-provided workspace-root copy: True. Current integrated all-target suite is 617/0 (root validation).

## Positive first-loader and kernel progress

Startup detects `retail PCH (RAM snapshot)`, constant shift -128, signed-block blob 0x5E6E4 and 16/16 marker bytes verified (log19). Native first loader enters 0x5C01E and reports SUCCESS (31/47); secondloader handoff 0x402FA follows. The existing ROM stand-in then stages the secure kernel, and ordinary native ARM scheduler/auth/load paths run. A late `bootkeys` says unknown because the resident first-loader RAM has already been replaced at handoff; the startup detection is the comparable layout evidence.

At exactly 300000slices:155213084 total instructions, root diagnostic emulated time 0.274s, checkpoint A9. Native module execution is present: ARM1 sleeps in kernel WFE helper0x0047969C with a valid native thread context; ARM3 runs supplied Syscon import/time path0x004FD9B0 (LR0x004F9DBD); ARM2 waits in genuine Secure delay helper0x000CD024 (LR0x003BF1D3). ARM0 is still in NSKBL extension/start helper0x5101CA50 with R0=0x005A9275 (Lowio runtime export). TPIDRPRW/core state and all six processor registers were recorded. These are genuine relocated os0 kernel/driver execution milestones, not a host `stage kernel` flag. No textual all 28 module-list result count or logo milestone is claimed by this bounded smoke: UART is idle, and no native loader breakpoint was installed.

Cold physical KBL record at 0x4001FD00 retains bytes30..33 `FF FF 00 FF` and word +0x6C=4. No unconditional allocated stage-stack bias is logged. eMMC 10753 reads, 0 writes,dirty=no; no undefined instruction or Kernel Panic appears. Existing 8 early MMU repairs on ARM0 are retained; no new CPU frontier is demonstrated.

Exit 0, wall 23.266 seconds. This smoke validates the supplied retail dump under the current cold board profile and reaches genuine kernel/driver startup. It does not establish completed system boot or SDL presentation. Those are separate fresh prototype/default captures owned by the parent/graphics/UI agents.
