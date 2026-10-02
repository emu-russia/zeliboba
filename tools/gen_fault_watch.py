lines = ['core arm', 'run 313200', 'info']
for extra in (500, 2000, 20000, 100000):
    lines.append('run %d' % extra)
    lines.append('info')
lines.append('quit')
open(r'.\build\fault_watch.txt', 'w', encoding='ascii').write('\n'.join(lines) + '\n')
print('written')
