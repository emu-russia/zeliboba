# zeliboba debugger script: walk the CMeP first loader.
#
#   build\bin\zeliboba.exe --script scripts\first-loader.cmd
#
# It stops on the mailbox poll, shows what the loader sees, and then follows the
# hand-off into the staged second loader.

echo ==== machine layout ====
info

echo ==== boot plan ====
boot

echo ==== run the first loader up to the mailbox poll ====
core mep
bp 0x5C578
run 400000
regs
trace 16

echo ==== what did it write to the ARM mailbox? ====
devget CMeP.Mailbox.MailboxCmepToArm.Status
devget CMeP.Mailbox.MailboxArmToCmep.Command

echo ==== keyring activity ====
keyring

echo ==== continue and watch the hand-off ====
bpcall
run 2000000
core mep
dis
boot
quit
