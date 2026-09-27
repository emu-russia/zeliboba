core mep
bp 0x4A99A
run 8000000
mem 0x5EE60 3
watch 0x5EE78 w
bpc 0x4A99A
run 8000000
trace find 0x5EE78
quit
