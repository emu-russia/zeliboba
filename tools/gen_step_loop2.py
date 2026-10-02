lines = ['core arm', 'bp 0x4003AA44', 'run 8000000', 'bpc 0x4003AA44', 'regs', 'step 14', 'regs', 'step 14', 'regs', 'quit']
open(r'.\build\step_loop2.txt', 'w', encoding='ascii').write('\n'.join(lines) + '\n')
print('script written')
