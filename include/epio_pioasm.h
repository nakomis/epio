// Copyright (C) 2026 Martin Harris <epio@nakomis.com>
//
// MIT License

// epio - A PIO emulator
//
// Loading programs assembled by pioasm ("pioasm -o c-sdk"), with a
// pico-sdk compatible subset of the hardware_pio program and state machine
// configuration API.
//
// pioasm's c-sdk output includes "hardware/pio.h" and calls sm_config_*()
// functions to build each program's default configuration.  Put epio's
// shim/ directory on the include path and that header compiles unmodified
// on the host, using the types and functions here.  Then load the program
// with epio_add_program() and configure an SM with epio_sm_init():
//
//     #include "my_program.pio.h"
//
//     uint offset = epio_add_program(epio, 0, &my_program_program, 0);
//     pio_sm_config c = my_program_program_get_default_config(offset);
//     sm_config_set_out_pins(&c, 18, 23);
//     epio_sm_init(epio, 0, 0, offset, &c);
//     epio_enable_sm(epio, 0, 0);
//
// The sm_config_*() functions match the pico-sdk's register encodings.  As
// in the pico-sdk, pin numbers are taken modulo 32 - for GPIOs 32 to 47, set
// the block's GPIOBASE to 16 with epio_set_gpiobase().

#ifndef EPIO_PIOASM_H
#define EPIO_PIOASM_H

// epio's tests redirect assert() to cmocka before including this
#ifndef assert
#include <assert.h>
#endif
#include <stdbool.h>
#include <stdint.h>
#include <epio.h>

#ifdef __cplusplus
extern "C" {
#endif

// The pico-sdk's uint.  C11 allows this typedef to repeat one in a system
// header.
typedef unsigned int uint;

// pioasm's output includes used_gpio_ranges for PIO version 1 (RP2350)
#ifndef PICO_PIO_VERSION
#define PICO_PIO_VERSION 1
#endif

/**
 * @brief A PIO program, as pioasm's c-sdk output defines it.
 *
 * Matches the pico-sdk's pio_program_t.
 */
typedef struct pio_program {
    /** The program's instructions, assembled as for offset 0 */
    const uint16_t *instructions;
    /** Number of instructions */
    uint8_t length;
    /** Required load offset (from .origin), or -1 for any */
    int8_t origin;
    /** PIO version the program needs (0 RP2040, 1 RP2350) */
    uint8_t pio_version;
    /** Bitmap of the 16 GPIO ranges the program uses */
    uint8_t used_gpio_ranges;
} pio_program_t;

/**
 * @brief An SM configuration, as built by the sm_config_*() functions.
 *
 * Matches the pico-sdk's pio_sm_config, without its GPIOBASE tracking.
 */
typedef struct {
    uint32_t clkdiv;
    uint32_t execctrl;
    uint32_t shiftctrl;
    uint32_t pinctrl;
} pio_sm_config;

// SMx register fields (RP2350 datasheet 11.7)
#define EPIO_CLKDIV_FRAC_LSB            8
#define EPIO_CLKDIV_INT_LSB             16
#define EPIO_EXECCTRL_SIDE_EN_LSB       30
#define EPIO_EXECCTRL_SIDE_PINDIR_LSB   29
#define EPIO_EXECCTRL_JMP_PIN_LSB       24
#define EPIO_EXECCTRL_OUT_EN_SEL_LSB    19
#define EPIO_EXECCTRL_INLINE_OUT_EN_LSB 18
#define EPIO_EXECCTRL_OUT_STICKY_LSB    17
#define EPIO_EXECCTRL_WRAP_TOP_LSB      12
#define EPIO_EXECCTRL_WRAP_BOTTOM_LSB   7
#define EPIO_EXECCTRL_STATUS_SEL_LSB    5
#define EPIO_SHIFTCTRL_FJOIN_RX_LSB     31
#define EPIO_SHIFTCTRL_FJOIN_TX_LSB     30
#define EPIO_SHIFTCTRL_PULL_THRESH_LSB  25
#define EPIO_SHIFTCTRL_PUSH_THRESH_LSB  20
#define EPIO_SHIFTCTRL_OUT_SHIFTDIR_LSB 19
#define EPIO_SHIFTCTRL_IN_SHIFTDIR_LSB  18
#define EPIO_SHIFTCTRL_AUTOPULL_LSB     17
#define EPIO_SHIFTCTRL_AUTOPUSH_LSB     16
#define EPIO_SHIFTCTRL_IN_COUNT_LSB     0
#define EPIO_PINCTRL_SIDESET_COUNT_LSB  29
#define EPIO_PINCTRL_SET_COUNT_LSB      26
#define EPIO_PINCTRL_OUT_COUNT_LSB      20
#define EPIO_PINCTRL_IN_BASE_LSB        15
#define EPIO_PINCTRL_SIDESET_BASE_LSB   10
#define EPIO_PINCTRL_SET_BASE_LSB       5
#define EPIO_PINCTRL_OUT_BASE_LSB       0

// Replaces a register field
#define EPIO_FIELD(REG, LSB, WIDTH, VALUE) \
    (((REG) & ~(((1u << (WIDTH)) - 1) << (LSB))) | \
     (((uint32_t)(VALUE) & ((1u << (WIDTH)) - 1)) << (LSB)))

static inline void sm_config_set_out_pin_base(pio_sm_config *c, uint out_base) {
    c->pinctrl = EPIO_FIELD(c->pinctrl, EPIO_PINCTRL_OUT_BASE_LSB, 5, out_base & 31);
}

static inline void sm_config_set_out_pin_count(pio_sm_config *c, uint out_count) {
    assert(out_count <= 32 && "OUT count must be 0 to 32");
    c->pinctrl = EPIO_FIELD(c->pinctrl, EPIO_PINCTRL_OUT_COUNT_LSB, 6, out_count);
}

static inline void sm_config_set_out_pins(pio_sm_config *c, uint out_base, uint out_count) {
    sm_config_set_out_pin_base(c, out_base);
    sm_config_set_out_pin_count(c, out_count);
}

static inline void sm_config_set_set_pin_base(pio_sm_config *c, uint set_base) {
    c->pinctrl = EPIO_FIELD(c->pinctrl, EPIO_PINCTRL_SET_BASE_LSB, 5, set_base & 31);
}

static inline void sm_config_set_set_pin_count(pio_sm_config *c, uint set_count) {
    assert(set_count <= 5 && "SET count must be 0 to 5");
    c->pinctrl = EPIO_FIELD(c->pinctrl, EPIO_PINCTRL_SET_COUNT_LSB, 3, set_count);
}

static inline void sm_config_set_set_pins(pio_sm_config *c, uint set_base, uint set_count) {
    sm_config_set_set_pin_base(c, set_base);
    sm_config_set_set_pin_count(c, set_count);
}

static inline void sm_config_set_in_pin_base(pio_sm_config *c, uint in_base) {
    c->pinctrl = EPIO_FIELD(c->pinctrl, EPIO_PINCTRL_IN_BASE_LSB, 5, in_base & 31);
}

static inline void sm_config_set_in_pins(pio_sm_config *c, uint in_base) {
    sm_config_set_in_pin_base(c, in_base);
}

static inline void sm_config_set_in_pin_count(pio_sm_config *c, uint in_count) {
    assert(in_count >= 1 && in_count <= 32 && "IN count must be 1 to 32");
    c->shiftctrl = EPIO_FIELD(c->shiftctrl, EPIO_SHIFTCTRL_IN_COUNT_LSB, 5, in_count);
}

static inline void sm_config_set_sideset_pin_base(pio_sm_config *c, uint sideset_base) {
    c->pinctrl = EPIO_FIELD(c->pinctrl, EPIO_PINCTRL_SIDESET_BASE_LSB, 5, sideset_base & 31);
}

static inline void sm_config_set_sideset_pins(pio_sm_config *c, uint sideset_base) {
    sm_config_set_sideset_pin_base(c, sideset_base);
}

static inline void sm_config_set_sideset(pio_sm_config *c, uint bit_count, bool optional, bool pindirs) {
    assert(bit_count <= 5 && "Side-set bit count must be 0 to 5");
    assert((!optional || bit_count >= 1) && "Optional side-set needs at least 1 bit");
    c->pinctrl = EPIO_FIELD(c->pinctrl, EPIO_PINCTRL_SIDESET_COUNT_LSB, 3, bit_count);
    c->execctrl = EPIO_FIELD(c->execctrl, EPIO_EXECCTRL_SIDE_EN_LSB, 1, optional);
    c->execctrl = EPIO_FIELD(c->execctrl, EPIO_EXECCTRL_SIDE_PINDIR_LSB, 1, pindirs);
}

static inline void sm_config_set_clkdiv_int_frac8(pio_sm_config *c, uint32_t div_int, uint8_t div_frac8) {
    assert((div_int >> 16) == 0 && "Clock divider integer part must fit in 16 bits");
    assert((div_int != 0 || div_frac8 == 0) && "A divider of 0 (65536) cannot have a fraction");
    c->clkdiv = ((uint32_t)div_frac8 << EPIO_CLKDIV_FRAC_LSB) |
                (div_int << EPIO_CLKDIV_INT_LSB);
}

static inline void sm_config_set_clkdiv_int_frac(pio_sm_config *c, uint16_t div_int, uint8_t div_frac8) {
    sm_config_set_clkdiv_int_frac8(c, div_int, div_frac8);
}

static inline void sm_config_set_wrap(pio_sm_config *c, uint wrap_target, uint wrap) {
    assert(wrap < NUM_INSTRS_PER_BLOCK && wrap_target < NUM_INSTRS_PER_BLOCK && "Invalid wrap");
    c->execctrl = EPIO_FIELD(c->execctrl, EPIO_EXECCTRL_WRAP_BOTTOM_LSB, 5, wrap_target);
    c->execctrl = EPIO_FIELD(c->execctrl, EPIO_EXECCTRL_WRAP_TOP_LSB, 5, wrap);
}

static inline void sm_config_set_jmp_pin(pio_sm_config *c, uint pin) {
    c->execctrl = EPIO_FIELD(c->execctrl, EPIO_EXECCTRL_JMP_PIN_LSB, 5, pin & 31);
}

static inline void sm_config_set_in_shift(pio_sm_config *c, bool shift_right, bool autopush, uint push_threshold) {
    assert(push_threshold <= 32 && "Push threshold must be 0 to 32");
    c->shiftctrl = EPIO_FIELD(c->shiftctrl, EPIO_SHIFTCTRL_IN_SHIFTDIR_LSB, 1, shift_right);
    c->shiftctrl = EPIO_FIELD(c->shiftctrl, EPIO_SHIFTCTRL_AUTOPUSH_LSB, 1, autopush);
    c->shiftctrl = EPIO_FIELD(c->shiftctrl, EPIO_SHIFTCTRL_PUSH_THRESH_LSB, 5, push_threshold & 0x1f);
}

static inline void sm_config_set_out_shift(pio_sm_config *c, bool shift_right, bool autopull, uint pull_threshold) {
    assert(pull_threshold <= 32 && "Pull threshold must be 0 to 32");
    c->shiftctrl = EPIO_FIELD(c->shiftctrl, EPIO_SHIFTCTRL_OUT_SHIFTDIR_LSB, 1, shift_right);
    c->shiftctrl = EPIO_FIELD(c->shiftctrl, EPIO_SHIFTCTRL_AUTOPULL_LSB, 1, autopull);
    c->shiftctrl = EPIO_FIELD(c->shiftctrl, EPIO_SHIFTCTRL_PULL_THRESH_LSB, 5, pull_threshold & 0x1f);
}

static inline void sm_config_set_out_special(pio_sm_config *c, bool sticky, bool has_enable_pin, uint enable_bit_index) {
    c->execctrl = EPIO_FIELD(c->execctrl, EPIO_EXECCTRL_OUT_STICKY_LSB, 1, sticky);
    c->execctrl = EPIO_FIELD(c->execctrl, EPIO_EXECCTRL_INLINE_OUT_EN_LSB, 1, has_enable_pin);
    c->execctrl = EPIO_FIELD(c->execctrl, EPIO_EXECCTRL_OUT_EN_SEL_LSB, 5, enable_bit_index);
}

/**
 * @brief The pico-sdk's default SM configuration.
 *
 * Clock divider 1, wrap over the whole instruction memory, IN and OUT
 * shifting right with no autopush or autopull and thresholds of 32.
 */
static inline pio_sm_config pio_get_default_sm_config(void) {
    pio_sm_config c = {0};
    sm_config_set_clkdiv_int_frac8(&c, 1, 0);
    sm_config_set_wrap(&c, 0, 31);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_out_shift(&c, true, false, 32);
    return c;
}

/**
 * @brief Load a program into a block's instruction memory.
 *
 * As the pico-sdk's pio_add_program_at_offset(): JMP targets are relocated
 * by @p offset, and with GPIOBASE 16, WAIT GPIO instructions are adjusted to
 * address the block's GPIO window, so set the block's GPIOBASE first.
 *
 * Asserts if the program does not fit, or has an .origin other than
 * @p offset.
 *
 * @param epio     The epio instance.
 * @param block    PIO block index (0 to NUM_PIO_BLOCKS-1).
 * @param program  The program, e.g. pioasm's <name>_program.
 * @param offset   Instruction memory offset to load at.
 * @return         @p offset, for passing to <name>_program_get_default_config().
 */
EPIO_EXPORT uint epio_add_program(epio_t *epio, uint8_t block, const pio_program_t *program, uint8_t offset);

/**
 * @brief Initialise an SM with a configuration, ready to be enabled.
 *
 * As the pico-sdk's pio_sm_init(): disables the SM, applies @p config (or
 * the default configuration if NULL), clears its FIFOs, resets its shift
 * registers, delay and stall state, and sets its PC to @p initial_pc.  Does
 * not enable the SM.
 *
 * @param epio        The epio instance.
 * @param block       PIO block index (0 to NUM_PIO_BLOCKS-1).
 * @param sm          State machine index (0 to NUM_SMS_PER_BLOCK-1).
 * @param initial_pc  Instruction to start from.
 * @param config      Configuration, or NULL for pio_get_default_sm_config().
 */
EPIO_EXPORT void epio_sm_init(epio_t *epio, uint8_t block, uint8_t sm, uint8_t initial_pc, const pio_sm_config *config);

#ifdef __cplusplus
}
#endif

#endif // EPIO_PIOASM_H
