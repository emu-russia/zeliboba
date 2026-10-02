lines = ['core arm', 'bp 0x4003A9D4', 'run 3000000']
for i in range(6):
    lines += ['regs', 'bpc 0x4003A9D4', 'step 1', 'bp 0x4003A9D4', 'run 3000000']
lines.append('quit')
open(r'.\build\sec_calls.txt', 'w', encoding='ascii').write('\n'.join(lines) + '\n')
print('written')
