# Save states

A save state is a deterministic snapshot of the whole machine: every CPU core,
every RAM region, every MMIO device and the machine's own boot/scheduler flags.
Loading one and continuing the run must produce exactly the same execution as
never having stopped. The debugger exposes it as `savestate <file>` /
`loadstate <file>`.

## Stream API (`src/common/state.h`)

`StateWriter`:

| helper | meaning |
|---|---|
| `put_u8/put_u16/put_u32/put_u64` | little-endian scalars |
| `put_i32/put_bool/put_f32/put_f64` | the obvious encodings |
| `put_pod<T>(v)` | raw bytes of a trivially copyable value |
| `bytes(ptr, len)` | raw buffer |
| `str(s)` | length-prefixed string |
| `list(c, fn)` | length + every element of a sized range-for container |
| `fixed(c, fn)` | every element of a `std::array`/C array (no length) |
| `map(m, fn)` | length + `fn(key, value)` for every entry |
| `begin(name)` / `end()` | length-prefixed named section |

`StateReader` mirrors them with `get_*`, and the container helpers take a
mutable reference to fill:

```cpp
writer.map(values_, [&](u32 key, u64 value) { writer.put_u32(key); writer.put_u64(value); });
reader.map(values_, [&](u32& key, u64& value) { key = reader.get_u32(); value = reader.get_u64(); });
```

`begin()`/`end()` nest. A missing/renamed section is a load error (the stream is
length-prefixed, so the reader skips it and reports the mismatch); this is how an
old state file is rejected instead of misread.

Raw byte buffers (RAM, display buffers, FIFOs) use `state_write_pages` /
`state_read_pages`, which write a zero-page bitmap and only the non-zero pages.
A freshly reset machine therefore costs a few kilobytes, not hundreds of
megabytes.

## Contract for a stateful class

1. Declare in the class body:

   ```cpp
   void save_state(StateWriter& writer) const override;
   void load_state(StateReader& reader) override;
   ```

   `override` only when the base already declares the virtuals (`Device`,
   `RegisterBlock`, `DeviceMirror`, `Cpu`, `RegisterFile`). A plain helper class
   declares them without `override`.

2. Read and write **in exactly the same order**.

3. Serialise every mutable member that can change after construction:
   registers, counters, timers, latches, pending/armed flags, FIFOs, deques,
   vectors, maps, runtime strings, statistics counters. When unsure whether a
   field is state or configuration, serialise it — extra bytes are harmless,
   missing state silently breaks determinism.

4. Do **not** serialise: raw pointers/references to other objects (`Bus*`,
   `Device*`, `EmmcCard*`, `CmepBlock*`, `FILE*`), `std::function` hooks and
   callbacks, and construction-time configuration that is byte-identical in
   every build (names, base addresses, register layout tables, capacities,
   hardware ID constants).

5. A `RegisterBlock` subclass calls `RegisterBlock::save_state(writer)` first
   (the base serialises the register image and the wide `store()` values). A
   `Cpu` subclass calls `Cpu::save_state(writer)` first. `DeviceMirror`
   subclasses inherit the no-op: a mirror's state lives in its target, which the
   bus serialises where it is registered.

6. Wrap nested sub-objects in their own section (`writer.begin("Foo.bar")`).
   Do **not** wrap a device's own fields — `Bus::save_state` already puts each
   device in a `device` section.

7. `load_state` restores fields from the reader; it never resizes or rebuilds
   construction-time structures and never trusts the file.

## What the machine adds on top

* `Bus::save_state` walks RAM regions and every registered non-mirror device,
  plus the SCU exclusive monitor. A region with its own storage is written
  through `bytes()`, so a region the boot chain re-pointed at a shared buffer
  (the CMeP mirrors its 0x40000 window onto the private SRAM while it runs) is
  captured self-contained; a pure alias created by `add_ram_alias` records only
  its geometry and relies on the machine section for the shared buffer.
* `Vita::save_state` writes the shared host buffers, the boot chain and the
  scheduler flags, the eMMC card identity and the three CPU cores. The RL78 is
  owned by `ErnieBlock` and serialised there.
* The eMMC image file itself is external input: a state records the card's mode
  and the image path/size, not the 3.8 GiB image. A run that wrote to the card
  is reported before saving, because the state then depends on that file.
* Each bus checks that the state's device count matches the build's
  (`state file: bus has N devices, this build has M`). Adding a device in
  `KermitBlock::install()` therefore rejects every older snapshot - deliberately,
  because a state whose sections no longer line up would otherwise restore a
  half-built machine. Regenerate snapshots after such a change.

## Testing

* `tests/test_state.cpp` round-trips a device fixture through a writer/reader and
  checks that a saved-and-loaded machine continues bit-identically.
* End-to-end: save in the middle of a cold boot, run on, then load and run the
  same distance again — the two resulting save files must be byte-identical.
  See `verify.ps1` / `docs/DEBUGGER.md`.
