# zeliboba - partition block cache and free lists after the KBL built them.
#
#   zeliboba.exe --script tools/dbg_kbl_pool.cmd --log off
#
# The KBL builds the partition itself (0x40032108): pool VA 0x51C0, cache table at
# [pool+0x9C] (four slots per core: size class * 0x20, {u16 target, u16 count,
# u32 head}), size-class free lists at partition/region +0x38 + class*8.
core arm0
runm 20000
core arm0
vpa 0x4900
vmem 0x4900 1
vpa 0x51C0
vmem 0x51C0 14
vpa 0x52C0
vmem 0x52C0 8
vpa 0x5180
vmem 0x5180 14
quit
