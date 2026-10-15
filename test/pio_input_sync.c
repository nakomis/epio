// Copyright (C) 2026 Martin Harris <epio@nakomis.com>
//
// MIT License

// epio - A PIO emulator
//
// Unit tests for hardware input timing: pad loopback, input synchronisers
// and INPUT_SYNC_BYPASS (RP2350 datasheet 11.5.6.3)

#define APIO_LOG_IMPL
#include "test.h"

#define WRAP(TOP, BOTTOM)   (((TOP) << 12) | ((BOTTOM) << 7))
#define GPIO(N)             (1ULL << (N))

// Loads a program into block 0 SM 0, wrapping over the whole program.
static epio_t *input_setup(
    const uint16_t *prog,
    uint8_t len,
    uint32_t execctrl,
    uint32_t pinctrl,
    uint8_t input_sync
) {
    epio_t *epio = epio_init();
    assert_non_null(epio);
    for (int ii = 0; ii < len; ii++) {
        epio_set_instr(epio, 0, ii, prog[ii]);
    }
    epio_sm_reg_t reg = {
        .execctrl = execctrl | WRAP(len - 1, 0),
        .pinctrl = pinctrl,
    };
    epio_set_sm_reg(epio, 0, 0, &reg);
    if (input_sync) {
        epio_set_input_sync_enabled(epio, 1);
    }
    epio_enable_sm(epio, 0, 0);
    return epio;
}

// MOV X, PINS every cycle, with IN_BASE at the given pin, so bit 0 of X is
// the level PIO saw on that pin in the latest cycle.
static epio_t *sampler_setup(uint8_t in_base, uint8_t input_sync) {
    const uint16_t prog[] = { APIO_MOV_X_PINS };
    return input_setup(prog, 1, 0, APIO_IN_BASE(in_base), input_sync);
}

#define SAMPLED()   (SM(0, 0).x & 0x1)

// By default (input timing disabled) PIO reads the input level directly: a
// level set by the host is seen on the next cycle, and an output pin reads
// its external level, not its driven level.  This is epio's existing
// behaviour.
static void input_direct_by_default(void **state) {
    (void)state;
    epio_t *epio = sampler_setup(5, 0);
    assert_int_equal(epio_get_input_sync_enabled(epio), 0);

    epio_set_gpio_input_level(epio, 5, 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(SAMPLED(), 1);

    epio_set_gpio_input_level(epio, 5, 0);
    epio_set_gpio_output(epio, 5);
    epio_set_gpio_output_level(epio, 5, 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(SAMPLED(), 0);

    epio_free(epio);
}

static void input_sync_enable_flag(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);

    epio_set_input_sync_enabled(epio, 3);
    assert_int_equal(epio_get_input_sync_enabled(epio), 1);
    epio_set_input_sync_enabled(epio, 0);
    assert_int_equal(epio_get_input_sync_enabled(epio), 0);

    epio_free(epio);
}

// 11.5.6.3: the 2-flipflop synchroniser adds two cycles of latency.
static void input_sync_two_cycle_latency(void **state) {
    (void)state;
    epio_t *epio = sampler_setup(5, 1);

    epio_set_gpio_input_level(epio, 5, 1);
    const uint8_t expected[] = {0, 0, 1, 1};
    for (size_t ii = 0; ii < sizeof(expected); ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(SAMPLED(), expected[ii]);
    }

    // And going low again
    epio_set_gpio_input_level(epio, 5, 0);
    const uint8_t expected_low[] = {1, 1, 0, 0};
    for (size_t ii = 0; ii < sizeof(expected_low); ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(SAMPLED(), expected_low[ii]);
    }

    epio_free(epio);
}

// 11.5.6.3, INPUT_SYNC_BYPASS: a bypassed input has no synchroniser latency.
static void input_sync_bypass(void **state) {
    (void)state;
    epio_t *epio = sampler_setup(5, 1);
    epio_set_input_sync_bypass(epio, 0, 1u << 5);
    assert_int_equal(epio_get_input_sync_bypass(epio, 0), 1u << 5);

    epio_set_gpio_input_level(epio, 5, 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(SAMPLED(), 1);

    // Let the synchroniser settle at 1
    epio_step_cycles(epio, 2);

    // Bypassing a different GPIO leaves this one synchronised
    epio_set_input_sync_bypass(epio, 0, 1u << 6);
    epio_set_gpio_input_level(epio, 5, 0);
    epio_step_cycles(epio, 1);
    assert_int_equal(SAMPLED(), 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(SAMPLED(), 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(SAMPLED(), 0);

    epio_free(epio);
}

// INPUT_SYNC_BYPASS bits index the block's 32 GPIO window, so with GPIOBASE
// 16, bit 5 is GPIO 21.
//
// AMBIGUOUS: the register description says only "each bit corresponds to
// one GPIO"; this is the reading consistent with the rest of PIO's
// GPIOBASE-relative mapping.
static void input_sync_bypass_gpiobase16(void **state) {
    (void)state;
    epio_t *epio = sampler_setup(5, 1);
    epio_set_gpiobase(epio, 0, 16);

    epio_set_input_sync_bypass(epio, 0, 1u << 5);
    epio_set_gpio_input_level(epio, 21, 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(SAMPLED(), 1);

    epio_free(epio);
}

// Each block has its own synchronisers and bypass register.
static void input_sync_bypass_per_block(void **state) {
    (void)state;
    epio_t *epio = sampler_setup(5, 1);
    epio_set_instr(epio, 1, 0, APIO_MOV_X_PINS);
    epio_sm_reg_t reg = { .pinctrl = APIO_IN_BASE(5) };
    epio_set_sm_reg(epio, 1, 0, &reg);
    epio_enable_sm(epio, 1, 0);
    epio_set_input_sync_bypass(epio, 1, 1u << 5);

    epio_set_gpio_input_level(epio, 5, 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(SM(0, 0).x & 1, 0);
    assert_int_equal(SM(1, 0).x & 1, 1);

    epio_free(epio);
}

// With hardware input timing, PIO reads the pad, so an output GPIO reads
// back its driven level - here against an external level of 0.  An SM's
// write in cycle t is on the pad from cycle t + 1, and through the
// synchroniser is seen from cycle t + 3.
//
// AMBIGUOUS: the exact loopback latency (output register to pad to
// synchroniser) - see HARDWARE-CHECKS in the PR.
static void input_reads_back_driven_level(void **state) {
    (void)state;
    for (int bypass = 0; bypass < 2; bypass++) {
        const uint16_t prog[] = {
            APIO_SET_PINS(1),
            APIO_MOV_X_PINS,
        };
        epio_t *epio = input_setup(prog, 2, 0,
            APIO_SET_BASE(7) | APIO_SET_COUNT(1) | APIO_IN_BASE(7), 1);
        epio_set_gpio_output_control(epio, 7, 0);
        epio_set_gpio_output(epio, 7);
        epio_set_input_sync_bypass(epio, 0, bypass ? (1u << 7) : 0);
        // Run the MOV from cycle 1 onwards
        epio_sm_reg_t reg = {
            .execctrl = WRAP(1, 1),
            .pinctrl = APIO_SET_BASE(7) | APIO_SET_COUNT(1) | APIO_IN_BASE(7),
        };
        epio_set_sm_reg(epio, 0, 0, &reg);

        // Cycle 0: SET PINS, 1
        epio_step_cycles(epio, 1);
        assert_int_equal(epio_get_gpio_input(epio, 7), 0);

        // Cycles 1-3: MOV X, PINS
        const uint8_t expected[2][3] = {{0, 0, 1}, {1, 1, 1}};
        for (int ii = 0; ii < 3; ii++) {
            epio_step_cycles(epio, 1);
            assert_int_equal(SAMPLED(), expected[bypass][ii]);
        }

        epio_free(epio);
    }
}

// Input inversion applies to the pad level PIO sees.
static void input_sync_inverted(void **state) {
    (void)state;
    epio_t *epio = sampler_setup(5, 0);
    epio_set_gpio_input_inverted(epio, 5, 1);
    epio_set_input_sync_enabled(epio, 1);

    epio_step_cycles(epio, 1);
    assert_int_equal(SAMPLED(), 1);

    epio_free(epio);
}

// Enabling starts the synchronisers settled at the current pad levels.
static void input_sync_starts_settled(void **state) {
    (void)state;
    epio_t *epio = sampler_setup(5, 0);
    epio_set_gpio_input_level(epio, 5, 1);
    epio_set_input_sync_enabled(epio, 1);

    epio_step_cycles(epio, 1);
    assert_int_equal(SAMPLED(), 1);

    epio_free(epio);
}

// WAIT GPIO reads through the synchroniser.
static void input_sync_wait_gpio(void **state) {
    (void)state;
    const uint16_t prog[] = {
        APIO_WAIT_GPIO_HIGH(5),
        APIO_SET_X(1),
    };
    epio_t *epio = input_setup(prog, 2, 0, 0, 1);

    epio_set_gpio_input_level(epio, 5, 1);
    epio_step_cycles(epio, 2);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 0);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 1);

    epio_free(epio);
}

// WAIT PIN reads through the synchroniser.
static void input_sync_wait_pin(void **state) {
    (void)state;
    const uint16_t prog[] = {
        APIO_WAIT_PIN_HIGH(0),
        APIO_SET_X(1),
    };
    epio_t *epio = input_setup(prog, 2, 0, APIO_IN_BASE(5), 1);

    epio_set_gpio_input_level(epio, 5, 1);
    epio_step_cycles(epio, 2);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 0);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 1);

    epio_free(epio);
}

// JMP PIN reads through the synchroniser.
static void input_sync_jmp_pin(void **state) {
    (void)state;
    const uint16_t prog[] = {
        APIO_JMP_PIN(0),
    };
    epio_t *epio = input_setup(prog, 1, (5u << 24), 0, 1);
    epio_set_instr(epio, 0, 1, APIO_NOP);
    epio_sm_reg_t reg = { .execctrl = (5u << 24) | WRAP(1, 0) };
    epio_set_sm_reg(epio, 0, 0, &reg);

    epio_set_gpio_input_level(epio, 5, 1);
    // Not taken on cycles 0 and 1, so PC goes 0 -> 1 -> 0
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 0);
    // Taken on cycle 2
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 0);

    epio_free(epio);
}

// IN PINS reads through the synchroniser.
static void input_sync_in_pins(void **state) {
    (void)state;
    const uint16_t prog[] = {
        APIO_IN_PINS(1),
    };
    epio_t *epio = input_setup(prog, 1, 0, APIO_IN_BASE(5), 1);
    epio_set_gpio_input_level(epio, 5, 1);

    // ISR shifts left by default, so bit 0 is the latest sample
    epio_step_cycles(epio, 3);
    assert_int_equal(epio_peek_sm_isr(epio, 0, 0) & 0x7, 0b001);

    epio_free(epio);
}

static void input_sync_bypass_invalid_block(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);

    expect_assert_failure(epio_set_input_sync_bypass(epio, NUM_PIO_BLOCKS, 0));
    expect_assert_failure(epio_get_input_sync_bypass(epio, NUM_PIO_BLOCKS));

    epio_free(epio);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(input_direct_by_default),
        cmocka_unit_test(input_sync_enable_flag),
        cmocka_unit_test(input_sync_two_cycle_latency),
        cmocka_unit_test(input_sync_bypass),
        cmocka_unit_test(input_sync_bypass_gpiobase16),
        cmocka_unit_test(input_sync_bypass_per_block),
        cmocka_unit_test(input_reads_back_driven_level),
        cmocka_unit_test(input_sync_inverted),
        cmocka_unit_test(input_sync_starts_settled),
        cmocka_unit_test(input_sync_wait_gpio),
        cmocka_unit_test(input_sync_wait_pin),
        cmocka_unit_test(input_sync_jmp_pin),
        cmocka_unit_test(input_sync_in_pins),
        cmocka_unit_test(input_sync_bypass_invalid_block),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
