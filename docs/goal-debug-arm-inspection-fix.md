# ARM virtual instruction inspection

The native 548-test boot ends at Lowio VA `0x005AB1DC`. The macOS screenshot
`goal-native-neon-macos-ui.png` showed FF bytes because the debugger read that
virtual address directly from the physical bus. It also selected ARM0 for its
listing even when a different ARM core was selected.

`Debugger::disassemble` now uses the selected ARM core, that core's execution
security bank, and a copied MMU state. The shared descriptor walk has a
RAM-only inspection specialization; it never invokes devices, fault hooks,
or bus reads. Instruction bytes likewise come directly from host RAM and are
decoded through the supplied-byte disassembler. Branch targets retain their
virtual addresses. Monitor uses the Secure execution bank even with SCR.NS=1.

Missing translations, non-RAM code, and inaccessible second Thumb halfwords
produce explicit unavailable text rather than fabricated FF instructions.
The listing stores the decoded/known instruction width separately from the
available bytes. UI step-over takes its width from this same debugger view.
Physical memory/save commands retain their existing semantics.

Three regressions prove:

- The actual Lowio bytes `DC69`, `2C00`, `D1FC` decode as LDR R4,[R3,#28],
  CMP R4,#0, and BNE to VA `0x005AB1DA`, through the selected core's mapping.
  Different ARM0/ARM3 mappings, Monitor Secure-vs-NS selection, and A32 virtual
  branch targets are checked.
- A Thumb32 instruction spans noncontiguous physical pages and advances by
  four bytes. If its second page is absent, inspection returns unavailable
  and preserves the live fault ring and counters.
- MMIO instruction bytes and MMIO descriptor tables are refused even with
  RAM underneath; no device reads or boot fault repairs occur. MMU-off RAM
  inspection also creates no bus traffic.

Tests compare CPU registers/state, MMU statistics/fault records/PAR, bus
statistics/context/trace and fault-hook calls before and after inspection.
All **168 isolated ARM+debugger tests** passed after this patch, and **171**
pass combined with the subsequently authorized TPIDR patch. UI main and
panels compile independently. No shared build or new UI boot capture was run.

Files: `debug/debugger.cpp/.h`, `cpu/arm/arm_disasm.cpp/.h`, inspection-only
`cpu/arm/arm_mmu.cpp/.h` and `arm_core.h`, `ui/ui_main.cpp`,
`tests/test_debugger.cpp`. Log: `goal-debug-arm-inspection-tests.log`.
