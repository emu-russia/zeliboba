p = 'src/cpu/arm/arm_mmu.cpp'
s = open(p, encoding='utf-8').read()
old = """        if (domain_fault(dacr, domain)) {
            result.fault = arm::MmFaultKind::Domain;"""
new = """        if (domain_fault(dacr, domain)) {
            ZLB_LOG_DEBUG("mmu", "domain fault: VA=0x%08X dacr=0x%08X domain=%u l1=0x%08X", va, dacr, domain,
                          l1);
            result.fault = arm::MmFaultKind::Domain;"""
assert old in s
s = s.replace(old, new, 1)
if '#include "common/log.h"' not in s:
    s = s.replace('#include "bus/bus.h"', '#include "bus/bus.h"\n#include "common/log.h"', 1)
open(p, 'w', encoding='utf-8').write(s)
print('debug log added')
