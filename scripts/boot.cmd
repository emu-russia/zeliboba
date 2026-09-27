# zeliboba debugger script: full boot chain walkthrough.
#
#   build\bin\zeliboba.exe --script scripts\boot.cmd
#
# Each stage is entered explicitly, so the output tells you exactly how far the
# emulator gets today and what the first unexpected thing was.

echo ==== stage 1: CMeP first loader ====
core mep
run 2000000
boot

echo ==== stage 2: CMeP second loader (loaded directly from the decrypted image) ====
stage second
run 2000000
boot

echo ==== stage 3: ARM kernel boot loader from SLB2 ====
stage kbl
core arm
run 2000000
dis
boot

echo ==== stage 4: kernel modules ====
stage kernel
run 2000000
boot

echo ==== eMMC ====
emmc info

echo ==== MMIO that was touched ====
devices

quit
