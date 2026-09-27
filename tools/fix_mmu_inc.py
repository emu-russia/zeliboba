p = 'src/cpu/arm/arm_mmu.cpp'
s = open(p, encoding='utf-8').read()
inc = '#include "common/log.h"'
if inc not in s:
    s = s.replace('#include "common/util.h"', inc + '\n#include "common/util.h"', 1)
    open(p, 'w', encoding='utf-8').write(s)
    print('log.h included')
else:
    print('already included')
