lines = ['core arm', 'bp 0x40020E48', 'run 12000000']
for i in range(6):
    lines.append('regs')
    lines.append('run 4000000')
lines.append('quit')
open(r'C:\Work\PSVita\zeliboba\build\map_hits.txt', 'w', encoding='ascii').write('\n'.join(lines) + '\n')
print('written')
