# zeliboba - kernel_boot_loader checkpoint snapshot.
#
#   zeliboba.exe --script tools/dbg_kbl_status.cmd --log off
#
# Dumps the KBL's own objects *by virtual address* (vpa/vmem translate through the
# active core's tables), which is the only safe way to look at them: the loader
# rebuilds its page tables as it boots, so one and the same object lives at a
# different physical address at different times (see docs/KBL.md round 57).
#
# Read the output like this:
#   0x4900  boot context    -> +0x04 = entry count, +0x08/+0x0C = DRAM base/size,
#                              +0x8C = the object manager (0 = it was never created)
#   0x5180  region          -> 0x40000000 0x00300000 0x00040000 ... (base, size, VA base)
#   0x51C0  partition       -> magic 0x4080502B, 0x00010002 markers = free chunks
#   0x5400  class registry  -> created by 0x40031410 ("SceKernelFixedHeapClass")
#   0x5A40  UID entry heap  -> chained on top of 0x5400
#
# A healthy cold boot (before the KBL's own memory manager exists) shows the magic
# and the region; the interesting question each round is whether +0x8C of the boot
# context has become non-zero and whether the free-chunk markers move.
core arm0
runm 20000
core arm0
vpa 0x4900
vmem 0x4900 2
vpa 0x5180
vmem 0x5180 2
vpa 0x51C0
vmem 0x51C0 3
vpa 0x5400
vmem 0x5400 1
vpa 0x5A40
vmem 0x5A40 1
regs
quit
