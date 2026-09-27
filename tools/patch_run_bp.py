p = r'C:\Work\PSVita\zeliboba\src\debug\debugger.cpp'
s = open(p, encoding='utf-8').read()
old = '''void Debugger::run(int64_t count) {
    stop_requested_ = false;
    int64_t done = 0;
    while (done < count && !stop_requested_) {
        step(1);
        last_stop_.steps = static_cast<u64>(done);
        ++done;
        if (last_stop_.stopped && stop_requested_) break;
        if (vita_.stage() == BootStage::Failed) break;
    }
}'''
new = '''void Debugger::run(int64_t count) {
    stop_requested_ = false;
    last_stop_.stopped = false;
    int64_t done = 0;
    while (done < count && !stop_requested_) {
        step(1);
        last_stop_.steps = static_cast<u64>(done);
        ++done;
        // step() reports a breakpoint or a watchpoint here; without this the loop
        // kept re-hitting the same breakpoint until the count ran out.
        if (last_stop_.stopped) break;
        if (vita_.stage() == BootStage::Failed) break;
    }
}'''
if 'last_stop_.stopped = false;\n    int64_t done = 0;' in s:
    print('already patched')
elif old not in s:
    raise SystemExit('run() body not found')
else:
    open(p, 'w', encoding='utf-8').write(s.replace(old, new, 1))
    print('patched')
