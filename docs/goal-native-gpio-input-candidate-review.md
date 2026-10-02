# Native GPIO0 input qualification review

2026-10-01. Read-only review of the new GPIO0/SPI0 endpoint work. No production source edits or shared build were made by this reviewer. The CPU RFE/User-System fix remains frozen, isolated ARM174/0.

## Correction to the earlier SPI-ready proposal

The earlier `goal-native-gpio-spi0-ready-proposal.md` endpoint step4 proposed holding physical input4 low through native GPIO parent dispatch. Exact supplied1.04 execution excludes that choice under the documented +04 physical-input interpretation.

The unchanged Lowio mode setter at linked810029BC..81002A00, after its mutex acquisition, receives pin4 and mode3. It writes GPIO+14=0x300 and clears bit4 in the driver record+28. Starting that shadow atFFFFFFFF demonstrates the resultFFFFFFEF. The native parent handler candidate block, after parent mutex acquisition, at810022A4..810022CA computes:

```
candidate = ((GPIO_INPUT_04 ^ record_28) & ~record_24) & gate_enable_shadow
```

For the actually reached gate0/IRQ248, fixture record+10=0x10 and record+24=0, a low input4 gives candidate0 and falls through810022CC. A high input4 gives candidate0x10 and branches to810022D6, the normal pending/status/subinterrupt path.

`goal-lowio-gpio-candidate-probe.cpp` loads the unchanged supplied Lowio PT_LOAD (fileA0, linked81000000), executes the actual mode setter and candidate block, and logs:

```
MODE pc=81002A00 hardware=00000300 shadow28=FFFFFFEF
SAMPLE=00 pc=810022CA candidateR3=00000000 Z=1 afterBNE=810022CC
SAMPLE=10 pc=810022CA candidateR3=00000010 Z=0 afterBNE=810022D6
```

This probe establishes candidate eligibility only. It does not execute a complete native parent/sub4 callback or prove a successful Syscon request. The pending status still needs a real edge and ordinary W1C handling.

The resulting compatible board-model proposal is a real falling edge whose input sample has returned idle-high before native handler execution. A sub-tick pulse is an explicit emulator timing choice; the physical pulse duration, pull-up/reset level and five-gate event fanout are not measured. Do not invert mode3, override the handler candidate or call sub4 directly.

## Primary source cross-check

[Hardware-tested gpio.c](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/gpio.c) treats +04 as port input, separates output-latch readback+34, uses five active-high masks and pending status words, and acknowledges pending bits explicitly. The [gpio.h encoding](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/include/gpio.h) assigns input4, output3 and falling-edge mode3. These sources do not document electrical pulse duration.

[Hardware-tested syscon.c](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/syscon.c) raises output3 after SPI write completion, then queries and acknowledges GPIO interrupt pending before draining RX and lowering output3. It polls interrupt pending rather than the physical pin in the receive path, so it provides no evidence that input4 must remain low through native dispatch.

## Implementation review boundaries

The new GPIO object coherently separates direction, output latch and sampled input; set/clear and W1C use only supplied byte lanes; reads/peek do not consume state; parent248..252 are mask-gated levels; reset restores masks and deasserts callbacks. Only input4/falling/mode3 generates the bounded native event. Unsupported pin/mode handling remains inert. Five-gate retention/fanout and all-masked reset are labeled model choices.

The SPI0 wire now visible in source associates only a fresh framed request/reply generation in CTL0 with a driven output3 rise after native phase qualification. It waits for a real PERIPHCLK tick, then applies a physical falling/rising pulse through the GPIO input API. RX drain, stop, output-low and reset cancel scheduling. No service result, callback or thread wake is synthesized. Existing polling transport remains independent.

Required integration evidence remains the actual native parent/sub4 callback and request result after the pulse, plus ordinary first-loader milestones. The electrical pulse choice is compatible with the native candidate, not established hardware timing. No blocker was found in the standalone GPIO register/edge mechanics. The owner's forthcoming unchanged-candidate wire regression should retain the low-sample negative case and high-after-pulse positive case.
