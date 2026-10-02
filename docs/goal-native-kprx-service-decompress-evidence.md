# Native 1.04 authentication succeeds; ARM decompression fails

Read-only evidence from the integrated 533-test revision, 2026-10-01.
Captures use the current CLI, an absolute clone `goal-arm-auth-dram-clone.img`,
and no inherited `ZLB_*` variables or runtime overrides. All captures report
8345 eMMC reads, zero writes, and a clean image. No source changes were made.

## Native service results

`goal-arm-auth-dram-results-capture.py/.log` captures every actual mailbox1
completion and ARM submission return for the first os0 module:

| Function | Mailbox1 completion | Command+8 | ARM return at `51016ADE` | Caller return |
|---|---:|---:|---:|---|
| `10001` header authentication | `1` | `0` | `0` | `51016FE8`: R0=0 |
| `20001` segment setup | `1` | `0` | `0` | `510170B2`: R0=0 |
| `30001` segment decrypt | `1` | `0` | `0` | `5101723C` success path reached |

All three commands use PA `40350200` / VA `30000200`. The actual ARM3
callback at `51016B7A` has R2=1 and R1=0, and stores zero to wait word
`5113BAC0`. The segment setup response at command+44 is compression mode2.
This is successful transport and successful native service execution, not
the earlier transport5 buffer rejection or an authentication error.

## Exact next failure

`goal-arm-native-decompress-capture.py/.log` follows the real ARM consumer:

- `51017250` selects mode2 and calls wrapper `510172CC`.
- At call `510172DC`, arguments to native `51024E18` are output VA R0=`CC000`,
  output capacity R1=`658`, compressed source R2=`5113C000`, R3=0.
- The mode record at `5113B800` is `{2, CC000, 2DD, 658, 0}`: compression
  mode, output VA, compressed byte count, output capacity, current offset.
- At `510172E0`, native decoder return R0 is **`80560100`**.
- The wrapper maps that negative result to **`800F0516`** at `510172F0`.
- Caller `5101ACBE` receives that same error and branches to cleanup.
- After real module stop/exit, ARM0 reaches main's error loop `51000D0C`,
  retaining R0=`800F0516`. The other cores sleep; DSI/IFTU remain inactive.

Thus the first os0 code segment has reached native authentication and
decryption, but its guest decompression fails before loading completes.
No later kernel or display-module progress is established by this run.

## Input and address-view validation

`goal-arm-native-decompress-source-capture.py/.log` saves the actual 733-byte
compressed stream before decoder execution as
`goal-arm-native-decompress-source.bin`. It begins `78 9C CD 93 5F 48` and
has SHA-256
`3f2043fbfda85346d7c9c638dd30cf081e521a3c31742a0eb030c879e95921bd`.

Python's zlib decompresses it successfully to 1624 bytes (`658`), exactly
the guest output capacity. The output is saved as
`goal-arm-native-decompress-offline-validation.bin`, SHA-256
`7d92a892f9c72a9e15fff70b4ffcfc174984f03f802e710e87d861b8468a93c2`.
It exactly matches PT_LOAD0 of genuine
`Vita_104_Firmware/Out/fs_dec/os0/psp2bootconfig.elf`: file offset `A0`,
VA `81000000`, file/memory size `658`, flags5. This independently validates
the native crypto output against the original firmware's decrypted segment.

`goal-arm-native-decompress-alias-capture.py/.log` repeats the pre-decoder
capture and proves source VA `5113C000` translates to PA `5113C000` through
TTBR1. ARM translated, ARM physical, and CMeP physical copies are saved as
`goal-arm-native-decompress-source-{arm-VA,arm-PA,mep-PA}.bin`; all three have
the identical 733-byte hash above. The actual output VA `CC000` translates
to PA `40373000`. A shared-address mismatch does not explain this failure.

An earlier mistaken physical save of untranslated output VA `CC000` is now
named `goal-arm-native-decompress-raw-physical-cc000.bin`; its all-FF bytes
are not the compressed input and are excluded from the input evidence.

## Next scope

The ARM/Thumb owner is auditing the native dynamic Huffman decoder beginning
at `51024E8C`, with the exact valid input stream and decoder return above.
The narrow next fix should follow a demonstrated CPU/decode/memory defect
there. No decoder bypass, extracted-ELF substitution, or fabricated service
success is justified by these observations.
