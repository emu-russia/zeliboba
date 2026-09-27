# zeliboba - who still holds the KBL class object after it is created.
#
#   zeliboba.exe --script tools/dbg_kbl_who.cmd --log off
#
# Reads the KBL's own globals and the two method slots the class created at
# 0x400313C8 carries (+0x2C/+0x30).  Everything is virtual: the loader rebuilds
# its page tables while it boots.
core arm0
runm 20000
core arm0
vmem 0x4900 2
vmem 0x5400 4
vmem 0x5A40 4
mem 0x400B3040 4
mem 0x400B3048 1
mem 0x400B2910 2
regs
quit
