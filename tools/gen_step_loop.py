lines = ['core arm', 'bp 0x4003AA44', 'run 8000000']
for i in range(10):
    lines.append('regs')
    lines.append('step 1')
lines.append('quit')
open(r'.\build\step_loop.txt', 'w', encoding='ascii').write('\n'.join(lines) + '\n')
print('script written')
