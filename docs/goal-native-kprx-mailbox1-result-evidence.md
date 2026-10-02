# Native 1.04 kprx request and cleanup evidence

Read-only investigation, 2026-10-01. Sources are the genuine unpacked NSKBL
(`goal-nskbl.bin`, base `51000000`), runtime-decrypted kprx payload
(`goal-native-kprx-stream-hmac-result-payload.bin`, base `80B000`), original
SLB2 `secure_kernel.bin`, and the current integrated CLI. No source changes,
forced returns, or mailbox injections were used for these captures.

## Reached behavior

`goal-arm-mailbox1-results-capture.py` runs the current CLI with inherited
`ZLB_*` variables removed, diagnostic GIC/fault logging enabled, and the
absolute cloned image `goal-startup-wait-clone.img`. Its log is
`goal-arm-mailbox1-results-capture.log`. The clone ends clean, with 8345
reads and zero writes.

The first request is `40350201` on ARM-to-CMeP mailbox1, representing command
PA `40350200`: size `190`, function `10001`, initial result `FFFFFFFF`.
The returned mailbox1 value is **5**. At actual ARM3 callback PC `51016B7A`,
R2 is 5 and R1 is 2; the instruction stores 2 at `5113BAC0`. ARM0 then
reaches `51016AD0` with R8=`800F052A` and returns that exact value at
`51016ADE` and the header-auth caller PC `51016FE8`.

This is transport rejection before authentication service execution. The
command result at `40350208` remains `FFFFFFFF`; it was never read through
the successful-transport branch `51016B00`. The callback's path is native:
read through SMC137, ACK through SMC136, require bit0, then accept exact
value1 or store transport-failure marker2. The GIC201/FIQ/SGI4 delivery and
callback are working at this point. The independently captured MeP validator
and aperture cause are owned by the separate slot513 investigation.

## The second request is stop, and the final WFE waits for exit

The final command at `40350400` is size `50`, function `FFFFFFFF`, result
`FFFFFFFF`, payload zero. It is the cleanup/stop sentinel, constructed at
`51016DD2..51016DD8`, translated at `51016DEC`, and submitted through SMC133
at `51016E08` as PA-or-one. It does not use the generic `10001` completion
callback. The submit succeeds at the transport doorbell level, after which
`51016E48` calls `51016348` to wait for module termination.

Fresh ARM0 capture at `51015D88` shows event object VA `F01F0` / PA
`403901F0`, first words `{780F9, 2A8109, 2, 1}`. Event field `+8` is 2,
and the wait mask R5 is 1. The native waiter `51015D64` tests
`[event+8] & mask`; because bit0 is absent, it executes WFE through
`51015D8C -> 510147D8` and sleeps at `510147DC`. No `51011030` panic
breakpoint is hit. Root's full `goal-native-swi-dsi-full.log` confirms the
second MeP request also receives mailbox1 value5.

The kprx sentinel semantics are explicit at `80E65C..80E66C`: after command
validation/copy, function `FFFFFFFF` stores zero to the master object's first
word (`[811FAC]`) and returns without the ordinary mailbox1 response. The
worker at `80E844..80E84C` then leaves its state-one work loop, unregisters
IRQ9 using SWI4, and returns to main. Main reaches terminal SWI1 at
`80B04A` with its real result. Secure-kernel SWI1 handler `800C30` sets state8,
stores that result via `800538`, and raises mailbox0 high-half notification
`10000` at `8009CE`. That real termination notification is what cleanup
needs. Rejection before the sentinel dispatch leaves the worker alive and
the exit event absent.

Low-half status `103` is the successful module-entry handoff emitted at
secure-kernel `80094A`, before `RETI` into module `80B000`; it should not be
confused with terminal SWI1's high-half exit notification.

## Genuine first os0 authentication input

Pre-SWI input capture is `goal-os0-auth-input-capture.py/.log`, with command,
range-list, and header binaries named `goal-os0-auth-input-*`. Its command
has one address/length range, list PA `5113B7C0`, pointing to PA `40500EC0`,
length `1000`. The captured 4096 bytes exactly equal the first 4096 bytes of
`Vita_104_Firmware/Out/fs/os0/psp2bootconfig.skprx`, SHA-256
`c78d690687fde6b372587c4278a32cd6d948edb8b084647b45e43ba6d0425670`.

The genuine SCE header has version3, SDK field `0140`, header type1,
metadata offset `600`, header length `1000`, data length `A18`, and file
length `14D2`. The command's context self type at `+44` is `100`, header
range count at `+170` is 1, and range-list pointer at `+174` is `5113B7C0`.
These inputs establish an actual signed os0 module request, without replacing
its SCE bytes by an extracted ELF.

## Checkpoints after the real aperture input is supplied

The command dispatcher is IRQ9 handler `80E55E`, registered by genuine SWI4
as callback `8079E4=80E55E`, with IER source9 enabled. Selected checkpoints:

| PC | What to inspect |
|---|---|
| `80E572` / `80E57E` | Read PA-or-one from mailbox1 and acknowledge it. |
| `80E646` / `80E652` | Copy into private buffer `812FC0`; nonzero copy return selects transport9. |
| `80E65C` / `80E68C` | Function ID and registered dispatch target after validation. |
| `80DF16` | First 10001 service entry; command pointer is R2. |
| `80DF36` | Real header-auth return R0, before it moves into R7. |
| `80DF4A` | Write real service R7 to command+8. |
| `80E6A0` / `80E6B4` | Copy entire command back and publish transport result. |
| `51016B7A` | ARM stores transport marker0 for exact mailbox1, else2. |
| `51016B00` / `51016FE8` | Read actual service result only after successful transport. |

The static command 10001 registration table at `8106F4` names real service
handler `80DF16`. That handler enters `80DD80` between SWI2/SWI3 wrappers;
any subsequent negative result must be traced through its genuine header,
metadata, policy, and crypto operations. A service error can coexist with
transport1. Conversely, transport5/9 or a callback that never executes is
not evidence of an invalid signature. Crypto owners are tracing later
hardware commands; no failure beyond the reached aperture rejection is
assumed here.
