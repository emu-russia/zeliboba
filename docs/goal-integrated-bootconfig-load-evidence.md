# Integrated native bootconfig load evidence (539-test build)

Read-only captures used the ordinary environment-clean CLI and the absolute `build/goal-emmc.img` clone. Sources stayed frozen; no shared build was run by this agent.

`goal-integrated-bootconfig-inflate-capture.log` shows:

* Native inflater return at 510172E0: R0=658, output VA CC000. Saved 1,624 bytes exactly match the genuine firmware 1.04 ELF PT_LOAD0 at file A0, SHA256 `7d92a892f9c72a9e15fff70b4ffcfc174984f03f802e710e87d861b8468a93c2`.
* PT_LOAD1 contains no file bytes and has zero-fill memsz70; its native segment-load return is zero.
* A second native inflater call returns318 into VA CE000. Saved 792 bytes exactly match the ELF relocation segment at file700, SHA256 `298c8315063c67793f9f82782dc27d7500eee98e7a88ee2debb340efe7edafde`.
* All three native segment-load returns at51019E70 are zero. Native relocation dispatcher5101B18C receives the genuine module object VA300930. MeP is already at800002 at that point; this separate fault has not prevented the ARM milestones.

`goal-integrated-bootconfig-relocation-capture.log` shows native relocation return R0=0 at5101A3E2. Bootconfig first export (Thumb callbackCC0B5) starts atCC0B4 onARM0, its body returns0 atCC0BE, and UART prints `ScePsp2BootConfig Starting... OK`. The second export (CC0C1) enters onARM3, ARM0, ARM1 andARM2 with R0=4, R1=51184C00, LR510014CD. No core reaches its post-bodyCC0C8 before the later ARM0 restarted-startup/barrier stop.

The second export's body atCC0038 first callsCC017C, thenCC013C, CC015C, CC012C, CC00FC, CC014C, CC00EC, CC016C andCC012C. Its CC0084 BLX calls the CC00FC import stub, with continuationCC0088.

`goal-integrated-bootconfig-imports-capture.log`/`linked-code.bin` prove all twelve SceKblForKernel stubs are genuinely bound. SpecificallyCC00FC (function NID1DB28F02, library NIDD0FC2991) contains A32 MOVW ip,#0711; MOVT ip,#5101; BX ip, correctly transferring to Thumb51010710. That helper loads global[5113B61C], then reads the sysroot callback at+40 and BLXes it at5101071E. The graphics agent independently captured that callback as zero, producing the first VA0 fetch. Import binding and the A32-to-Thumb branch are valid; the dependency that should populate sysroot+40 remains that agent's investigation.

The ELF e_entry190 names the SceModuleInfo data structure; it is not an executable entry. Genuine start/export callbacks are segment0+B5 and+C1. Other next-frontier CPU paths include relocation jump-table LDR.W PC at5101B302/5101B5EE and Thumb halfword MOVW/MOVT relocation helpers5101B000/5101B084; no new decoder defect is demonstrated by these successful captures.

Binary/disassembly artifacts: `goal-integrated-bootconfig-inflate-{0,1}.bin`, `goal-integrated-bootconfig-relocated-code.bin`, `goal-integrated-bootconfig-linked-code.bin`, `goal-integrated-bootconfig-linked-imports.txt`, `goal-nskbl-bootconfig-after-inflate.txt`, `goal-nskbl-bootconfig-relocation-dispatch.txt`. Native capture success does not establish os0 kernel handoff or a displayed logo.
