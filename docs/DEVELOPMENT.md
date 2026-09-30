# zeliboba - internal contracts

This file is the interface contract between the parallel workstreams. Read it
before touching anything under `src/`.

## Layout

```
src/common/     types, logging, file/string helpers          (done)
src/bus/        Device, RegisterFile, DeviceMirror, Bus, TraceLog   (done)
src/cpu/        Cpu interface + factory.h                    (done)
src/cpu/arm/    Cortex-A9 core + disassembler                (workstream ARM)
src/cpu/mep/    MeP-c5 core + disassembler                   (workstream MEP)
src/cpu/rl78/   RL78 core + disassembler + ISA table         (workstream RL78)
src/hw/         hardware blocks (cmep, syscon, soc, emmc)     (mixed)
src/loader/     PUP/SLB2/SELF/ELF + keys + NID               (workstream LOADER)
src/machine/    Vita: buses + cores + boot chain             (main)
src/debug/      headless debugger (breakpoints, CLI)         (main)
src/ui/         SDL3 frontend                                (workstream UI)
tools/          emmc_rebuild, zdis, ...                      (workstream EMMC)
tests/          zlb_tests self test binary                   (everyone)
```

## Ground rules

* C++20, no exceptions in hot paths, no RTTI requirement, no global state other
  than `Log::instance()`.
* Everything is in `namespace zlb`.
* Never `#include` a `.cpp`. Public API lives in headers.
* All CPU cores derive from `zlb::Cpu` (`src/cpu/cpu.h`) and implement the three
  pure virtuals `reset()`, `step()`, `disassemble()`, plus `registers()`.
* All hardware derives from `zlb::Device` (`src/bus/device.h`); use
  `RegisterFile` unless you need custom decode.
* `Bus::readN`/`writeN` are the only way to touch memory. `bus.context.pc` and
  `bus.context.core` must be updated by the core before each access so the trace
  is meaningful.
* Every core must be deterministic: no wall clock, no random, no thread locals.
  The hardware RNG is a device (`BigmacDevice`), not a core feature.
* Do not add dependencies. SDL3 is the only allowed third party library.
* Use the C# `VitaTestSuite` reference in the workspace, the binutils-generated
  ISA tables (`rl78ref`, `_rl78docs`), the datasheets in `datasheets/` and the
  boot ROM analysis in `dumps/bootrom_analysis/`.
* Do not copy code from another emulator. External *references* are allowed and
  are used where the workspace has no ground truth: `docs/CPU_ARM_AUDIT.md`
  compares the ARM core against capstone/GNU as (decoding) and Unicorn 2.1.4 /
  QEMU (execution semantics). Those tools are not part of the repository, so any
  measurement that depends on them has to name the version and the artifact, and
  cannot be reproduced from the workspace alone.

## Cpu interface cheat sheet

```cpp
class Cpu {
  Bus* bus; std::string name; u64 instructions, cycles;
  bool halted; std::string halt_reason; u32 pc; bool undefined_instruction;
  std::set<u32> breakpoints;

  virtual Arch arch() const;                 // MeP / Arm / Rl78
  virtual const char* core_name() const;
  virtual void reset();
  virtual void reset(u32 entry);             // has a default body; a core overrides it
                                             // when the entry needs arch specific semantics
  virtual void prepare_reset_context(u64 a0, u64 a1, u64 a2, u64 a3);
  virtual StepResult step();
  virtual std::string disassemble(u32 address, unsigned& length);
  virtual void registers(std::vector<RegValue>& out) const;
  virtual bool set_register(const std::string&, u64);
  virtual bool get_register(const std::string&, u64&) const;
  virtual std::string status_line() const;
  virtual void describe_state(std::vector<std::string>&) const;
  virtual void set_irq(int line, bool asserted);
  virtual bool interrupt_pending() const;
  virtual void tick(u64 cycles);
  u32 get_pc() const; void set_pc(u32);
};
```

`step()` must:
1. set `bus->context.pc` and `bus->context.core` to the current PC / core name,
2. execute exactly one instruction, updating `pc`, `instructions` and `cycles`,
3. return a `StepResult` with the address, byte length and the disassembly text,
4. on an unimplemented encoding set `undefined_instruction = true` and fill
   `StepResult::faulted/fault` instead of throwing,
5. never advance `pc` past an instruction it did not execute.

`reset(u32 entry)` is how the boot chain starts a core at an explicit address.
For ARM the low bit of `entry` selects Thumb state and must be honoured.

## Factory contract

Each core subdirectory supplies one function:

```cpp
std::unique_ptr<Cpu> create_arm_core(Bus& bus);
std::unique_ptr<Cpu> create_mep_core(Bus& bus);
std::unique_ptr<Cpu> create_rl78_core(Bus& bus);
```

Declared in `src/cpu/factory.h`.

## Device contract

```cpp
class Device {
  Device(std::string name, u32 base, u32 size);
  virtual u64  read(u32 address, unsigned size);   // size in {1,2,4,8}
  virtual void write(u32 address, unsigned size, u64 value);
  virtual void reset();
  virtual const char* register_name(u32 address) const;
  virtual std::string summary() const;
  virtual void enumerate_registers(std::vector<RegisterInfo>&) const;
  virtual bool peek_register(const std::string& name, u64& out) const;
  virtual bool poke_register(const std::string& name, u64 value);
  virtual void tick(u64 cycles);
  virtual void describe(std::vector<std::string>& lines) const;
};
```

Register names matter: the debugger's MMIO view and `devices` command print them.

## Paths at runtime

`resolve_workspace_path()` (in `src/machine/vita.h`) turns a relative path into an
absolute one below the folder that contains `zeliboba` (e.g. `dumps/...`,
`Vita_104_Firmware/Out/...`).
