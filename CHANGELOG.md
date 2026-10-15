# Changelog

## Unreleased

Added:

- Side-set.  Instructions drive `SIDESET_COUNT` side-set bits onto pin levels,
  or onto pin directions with `SIDE_PINDIR`, mapped from `SIDESET_BASE` and
  wrapping at 32.  With `SIDE_EN`, the top bit is an enable.  Side-set applies on
  every cycle an instruction executes, including when it stalls, and wins over
  an `OUT` or `SET` from the same SM to the same pin.  Instructions run via
  `OUT EXEC`, `MOV EXEC` and apio pre-instructions apply their side-set too.
- SM clock dividers, opt-in with `epio_set_clkdiv_enabled()` so existing
  users see no change.  When enabled, `epio_step_cycles()` steps system clock
  cycles and each SM executes only on the cycles its `CLKDIV` divider (16 bit
  integer, 8 bit fraction, first-order delta-sigma) asserts its clock enable.
  Dividers are free-running, as on hardware, and `epio_restart_clkdiv()`
  restarts them from phase 0 like `CTRL` `CLKDIV_RESTART`.
- `epio_set_sysclk_hz()` and `epio_get_sysclk_hz()`, a nominal system clock
  frequency (default 150 MHz) for hosts converting cycle counts to time.

## v0.2.2 - 2026-10-13

Fixed:

- `JMP X--` and `JMP Y--` test all 32 bits of the register.  They used to test
  the low 8 bits so a loop count of 256 or more ended early.
- `JMP !OSRE` jumps when the OSR is not empty.  It used to jump when the OSR was
  empty.
- `MOV x, STATUS` with `STATUS_SEL` IRQ reads the next PIO block's flag for
  `STATUS_N` 0x10-0x17.  It used to read a flag in this block instead.  epio now
  asserts on 0x18-0x1F, which the datasheet doesn't define.
- `OUT PC` jumps to the low 5 bits of the value it shifts out.  A value above 31
  used to jump outside instruction memory.
- The PC steps from address 31 to 0 when 31 isn't the wrap top.  It used to step
  outside instruction memory.
- `WAIT JMPPIN` adds its index to `JMP_PIN`, wrapping at 32.  It used to ignore
  the index.  epio now asserts on an index of 4 or more, which the datasheet
  reserves.
- `WAIT PIN` adds its index to `IN_BASE`, wrapping at 32.  It used to read past
  pin 31 instead.
- `OUT PINDIRS` and `MOV PINDIRS` change only the directions of pins the block
  controls, as `SET PINDIRS` does.  A 0 bit used to make any mapped pin an
  input.
- `SET PINS` and `SET PINDIRS` write `SET_COUNT` pins.  `SIDESET_COUNT` used to
  leak into the count.
- `MOV PINS` and `MOV PINDIRS` write 32 pins when `OUT_COUNT` is 32.  They used
  to write none.
- A delay excludes the bits `SIDESET_COUNT` reserves for side-set.  A program
  using side-set used to wait extra cycles.  Side-set itself isn't emulated.
- `MOV x, PINS` reads all 32 pins when `IN_COUNT` is 0.  It used to read none.
- `IN PINS` and `WAIT PIN` read pins at and above `IN_COUNT` as 0, as
  `MOV x, PINS` does.  They used to ignore `IN_COUNT`.
- `OUT PINS` and `OUT PINDIRS` write `OUT_COUNT` pins, with 0 beyond the bits
  shifted out.  They used to write one pin per bit shifted out.
- Autopull refills an empty OSR on any cycle, not only at the next `OUT`, and an
  `OUT` that finds the OSR empty stalls while it refills.  An `OUT` waiting for
  data used to run a cycle early.
- epio asserts on an `OUT_COUNT` above 32, or a `SET_COUNT` or `SIDESET_COUNT`
  above 5, which the datasheet doesn't allow.
- `epio_from_apio()` returns NULL when `APIO_ENABLE_PIOS()` hasn't been called.
  It used to assert first.

## v0.2.1 - 2026-08-20

Requires `apio` **v0.3.0** or later.  `apio` v0.3.0 makes `APIO_RXF` a read on
both a device and a host, where the emulated form used to append a word instead,
and epio's own tests were the only thing in either repository writing through
it.  A word is staged in an emulated RX FIFO with `epio_push_rx_fifo()`, which
is what the rest of the tests already did.

No change to epio's API or behaviour - the library never used those macros, only
the tests did.

## v0.2.0 - 2026-07-27

First tagged release.  Covers everything in the dated entries below.

Requires `apio` **v0.2.0** or later.  `epio_from_apio()` reads the `pull_up`,
`pull_down`, `input_only`, `drive_strength` and `slew_fast` fields of
`_apio_emulated_gpios`, which earlier versions of `apio` do not provide.

**Breaking** (relative to untagged `main` before this release):

- `epio_init_gpios()` now defaults all pins to pull-down, 4 mA drive and slow
  slew, matching the RP2350 hardware reset state.  Previously all pins defaulted
  to pull-up.  This is a silent behavioural change - it produces no compiler
  diagnostic.
- `APIO_ENABLE_PIOS()` must now be called before `APIO_ASM_INIT()`.  Code that
  omitted it was already failing to enable the PIO blocks on real hardware.
- `epio_drive_gpios_ext()` now restores undriven pins to their configured pull
  state, rather than unconditionally to 1.

Added:

- GPIO pull, drive strength and slew rate emulation, with the associated
  `epio_set_*` / `epio_get_*` accessors.
- Float mode control for undriven `PULL_NONE` pins: `epio_set_float_mode()` and
  `epio_set_float_seed()`.
- `epio_read_pull_up_pins()` and `epio_read_pull_down_pins()`.
- `epio_get_sram_ptr()` and `epio_update_from_apio()`, giving emulators more
  direct control.
- `epio_dma_setup_capture_pio_ring()`: a capture-mode DMA channel draining a
  source SM's RX FIFO into a circular SRAM ring, modelling the RP2350 address
  monitor path.  `epio_dma_capture_write_slot()` exposes the live host write
  position, so a consumer can see how far the ring has filled without polling
  every cycle.
- The emulated SRAM buffer is now aligned to the maximum DMA ring size (2^15),
  so a ring placed at a ring-size-aligned SRAM offset is also aligned in host
  address space, rather than depending on where `calloc` happened to place it.

## 2026-07-27

Align the emulated SRAM buffer to the maximum DMA ring size (2^15) instead of relying on the allocator's default alignment. A ring buffer placed at a ring-size-aligned SRAM offset is now also aligned in host address space, so a consumer that validates a host ring pointer's alignment (as the One ROM firmware does for the address monitor) behaves as it would against the fixed, aligned SRAM base on real hardware, rather than passing or failing depending on where `calloc` happened to place the buffer.

## 2026-07-26

Added `epio_dma_setup_capture_pio_ring()`: a capture-mode DMA channel that drains a source state machine's RX FIFO into a circular ring buffer in SRAM, advancing an exposed write position (wrapping within the ring), one entry per configurable transfer latency. This models the RP2350 address-monitor DMA (PIO RX FIFO -> circular SRAM buffer), complementing the existing serving-path `epio_dma_setup_read_pio_chain()`. Added `epio_dma_capture_write_slot()` to expose the channel's live host write pointer, so a consumer can observe how far the ring has filled without polling every cycle.

## 2026-06-28

Fixed `apply_apio_state()` clobbering a running SM's in-flight delay when applying injected pre-instructions (e.g. the PULL/MOV X,OSR pair from `pio_switch_rom_region`). The pending delay is now preserved across pre-instruction execution, so switching the served region on a live SM no longer phase-shifts it against the bus.

## 2026-06-28

Added `epio_get_sram_ptr()` and `epio_update_from_apio()` methods to give emulators more control.

## 2026-06-22

Police APIO_ENABLE_PIOS() being called.

This may break existing code that does not call APIO_ENABLE_PIOS() before calling APIO_ASM_INIT().  However, this is considered a bug fix - if existing code doesn't call APIO_ENABLE_PIOS(), it will not correctly enable the PIO blocks on real hardware. 

## 2026-06-18

Add `epio_read_pull_up_pins` and `epio_read_pull_down_pins` APIs to read the current state of pull-up and pull-down configurations.

## 2026-06-18

Fix bug introduced in 2026-06-16 that caused `epio_drive_gpios_ext` to
incorrectly restore undriven pins to their configured pull state, even when
the pin was marked as input-only.

## 2026-06-16

Added support for emulating internal GPIO pulls.

- Added `epio_set_gpio_pull_up`, `epio_get_gpio_pull_up`,
  `epio_set_gpio_pull_down`, `epio_get_gpio_pull_down`,
  `epio_set_gpio_pull_none`: per-pin pull resistor configuration.  Pull-up and
  pull-down are mutually exclusive; setting one clears the other.
- Added `epio_set_gpio_input_only`, `epio_get_gpio_input_only`: mark a pin as
  output-disabled.  `epio_set_gpio_output` now asserts if called on a pin
  marked input-only.
- Added `epio_set_gpio_drive`, `epio_get_gpio_drive`: per-pin drive strength
  (stored; no current behavioural effect).
- Added `epio_set_gpio_slew_fast`, `epio_get_gpio_slew_fast`: per-pin slew rate
  (stored; no current behavioural effect).
- Added `epio_set_float_mode(epio, mode)` and `epio_set_float_seed(epio, seed)`:
  control the value returned when reading an undriven pin configured with
  `PULL_NONE`.  `epio_float_mode_t` values: `EPIO_FLOAT_LOW` (0),
  `EPIO_FLOAT_HIGH` (1), `EPIO_FLOAT_RANDOM` (2).  Default: `EPIO_FLOAT_HIGH`.
  `epio_set_float_seed` seeds the internal PRNG, enabling deterministic replay
  of failing `EPIO_FLOAT_RANDOM` runs.
- **Breaking change**: `epio_init_gpios` now defaults all pins to pull-down,
  4 mA drive, slow slew, matching RP2350 hardware reset state.  Previously all
  pins defaulted to pull-up (high).  The following tests in `test/gpio.c` and
  `test/apio.c` require updating: `gpios_default_input_high`,
  `pin_states_all_high_on_init`, `drive_gpios_ext_undriven_pulled_up`,
  `output_to_input_pulls_up`, `init_gpios_clears_force_low`,
  `init_gpios_clears_force_high`, `invert_transfers_via_apio`.
- `epio_drive_gpios_ext` now restores undriven pins to their configured pull
  state (pull-up → 1, pull-down → 0, pull-none → float mode value) rather than
  unconditionally to 1.
- Added new fields to `epio_gpio_state_t`: `pull_up`, `pull_down`,
  `input_only`, `drive_strength[NUM_GPIOS]`, `slew_fast`.
- `epio_from_apio` now propagates `pull_up`, `pull_down`, `input_only`,
  `drive_strength`, and `slew_fast` state from `_apio_emulated_gpios`.
- Updated docstrings in `epio.h` to reflect pull-down as the hardware default.

### Examples

- Updated `apio/example/main.c` and `epio/example/firmware_main.c` to use
  `APIO_GPIO_INPUT_OUTPUT` in place of the removed `APIO_GPIO_OUTPUT`.

## 2026-05-09

- Fixed invalid assertion in `epio_apio.c` related to the maximum number of
  PIO instructions.  The assertion now correctly checks against
  `APIO_MAX_PIO_INSTRS - 1` instead of the previously incorrect
  `MAX_PRE_INSTRS`.

## 2026-02-24

- Changed `epio_set_gpio_inverted` to `epio_set_gpio_input_inverted` and
  `epio_get_gpio_inverted` to `epio_get_gpio_input_inverted` for clarity.
- Added `epio_set_gpio_force_input_low` and `epio_set_gpio_force_input_high`
  APIs to allow forcing GPIO input levels, and updated the internal logic to
  apply these forced levels immediately when set.
- Added `epio_get_gpio_force_input_low` and `epio_get_gpio_force_input_high`
  APIs to retrieve the current forced input level settings for GPIO pins.
- Removed `epio_read_gpios_ext` as it was not a useful API and confusing.
