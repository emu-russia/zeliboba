p = 'src/machine/bootchain.cpp'
s = open(p, encoding='utf-8').read()
s = s.replace('ZLB_LOG_ERROR("machine", "SLB2 does not contain a usable second_loader.enp");',
              'ZLB_LOG_ERROR("machine", "SLB2 does not contain a usable second_loader container");')
s = s.replace('add_milestone("ARM boot ROM staged second_loader.enp at 0x" + hex(second_loader_pa_, 8) + " (" +',
              'add_milestone("ARM boot ROM staged " + second->name + " at 0x" + hex(second_loader_pa_, 8) + " (" +')
open(p, 'w', encoding='utf-8').write(s)
print('bootchain messages updated')
