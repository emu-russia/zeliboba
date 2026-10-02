lines = ['core arm', 'bp 0x4003AA24', 'run 9000000']
for i in range(24):
    lines.append('regs')
    lines.append('run 3000000')
lines.append('quit')
open(r'.\build\loop_hits.txt', 'w', encoding='ascii').write('\n'.join(lines) + '\n')
print('written')
