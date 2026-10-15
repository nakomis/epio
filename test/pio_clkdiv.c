// Copyright (C) 2026 Martin Harris <epio@nakomis.com>
//
// MIT License

// epio - A PIO emulator
//
// Unit tests for SM clock dividers (RP2350 datasheet 11.5.5)

#define APIO_LOG_IMPL
#include "test.h"

#define WRAP(TOP, BOTTOM)       (((TOP) << 12) | ((BOTTOM) << 7))
#define CLKDIV(INT, FRAC)       (((uint32_t)(INT) << 16) | ((uint32_t)(FRAC) << 8))

// Each execution of JMP X-- 0 decrements X, so X counts SM cycles
#define START_X                 1000000
#define EXECUTED(BLOCK, _SM)    (START_X - SM(BLOCK, _SM).x)

// Loads a single JMP X-- loop into block 0 for the SMs in sm_mask, with the
// given CLKDIV, and enables them.  Clock dividers are left at their default
// (disabled) unless clkdiv_enabled is set.
static epio_t *counter_setup(uint32_t clkdiv, uint8_t sm_mask, uint8_t clkdiv_enabled) {
    epio_t *epio = epio_init();
    assert_non_null(epio);
    if (clkdiv_enabled) {
        epio_set_clkdiv_enabled(epio, 1);
    }
    epio_set_instr(epio, 0, 0, APIO_JMP_X_DEC(0));
    for (int sm = 0; sm < NUM_SMS_PER_BLOCK; sm++) {
        if (sm_mask & (1 << sm)) {
            epio_sm_reg_t reg = { .clkdiv = clkdiv, .execctrl = WRAP(0, 0) };
            epio_set_sm_reg(epio, 0, sm, &reg);
            SM(0, sm).x = START_X;
            epio_enable_sm(epio, 0, sm);
        }
    }
    return epio;
}

// Clock dividers are off by default, so CLKDIV is ignored and SMs run every
// cycle - preserving epio's behaviour for existing users.
static void clkdiv_ignored_by_default(void **state) {
    (void)state;
    epio_t *epio = counter_setup(CLKDIV(4, 0), 0x1, 0);
    assert_int_equal(epio_get_clkdiv_enabled(epio), 0);

    epio_step_cycles(epio, 10);
    assert_int_equal(EXECUTED(0, 0), 10);

    epio_free(epio);
}

static void clkdiv_enable_flag(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);

    epio_set_clkdiv_enabled(epio, 7);
    assert_int_equal(epio_get_clkdiv_enabled(epio), 1);
    epio_set_clkdiv_enabled(epio, 0);
    assert_int_equal(epio_get_clkdiv_enabled(epio), 0);

    epio_free(epio);
}

// 11.5.5: a divisor of 1 runs the SM on every cycle.
static void clkdiv_one_runs_every_cycle(void **state) {
    (void)state;
    epio_t *epio = counter_setup(CLKDIV(1, 0), 0x1, 1);

    for (int ii = 1; ii <= 5; ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(EXECUTED(0, 0), ii);
    }

    epio_free(epio);
}

// 11.5.5: an integer divisor of n runs the SM 1 cycle in every n.  From an
// initial phase of 0 the first cycle is an enable pulse.
static void clkdiv_integer(void **state) {
    (void)state;
    epio_t *epio = counter_setup(CLKDIV(3, 0), 0x1, 1);

    const uint8_t expected[] = {1, 1, 1, 2, 2, 2, 3, 3, 3, 4};
    for (size_t ii = 0; ii < sizeof(expected); ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(EXECUTED(0, 0), expected[ii]);
    }
    assert_int_equal(epio_get_cycle_count(epio), 10);

    epio_free(epio);
}

// SMx_CLKDIV: an integer part of 0 is a divisor of 65536.
static void clkdiv_int_zero_is_65536(void **state) {
    (void)state;
    epio_t *epio = counter_setup(CLKDIV(0, 0), 0x1, 1);

    epio_step_cycles(epio, 1);
    assert_int_equal(EXECUTED(0, 0), 1);
    epio_step_cycles(epio, 65535);
    assert_int_equal(EXECUTED(0, 0), 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(EXECUTED(0, 0), 2);

    epio_free(epio);
}

// 11.5.5 (Figure 56): a fractional divisor keeps a running total of the
// fraction, and each time it wraps through 1 the next period is n + 1.  For
// 2.5 the periods alternate 2, 3, 2, 3...
//
// AMBIGUOUS: which period is extended first (the phase of the pattern) is
// this reading of Figure 56 - see HARDWARE-CHECKS in the PR.
static void clkdiv_fractional(void **state) {
    (void)state;
    epio_t *epio = counter_setup(CLKDIV(2, 128), 0x1, 1);

    // Enable pulses on cycles 0, 2, 5, 7, 10, 12, 15, 17
    const uint8_t pulses[] = {0, 2, 5, 7, 10, 12, 15, 17};
    size_t next = 0;
    for (int cycle = 0; cycle < 20; cycle++) {
        epio_step_cycles(epio, 1);
        if (next < sizeof(pulses) && pulses[next] == cycle) {
            next++;
        }
        assert_int_equal(EXECUTED(0, 0), next);
    }
    // Steady state rate of n + f/256: 20 / 2.5 = 8
    assert_int_equal(EXECUTED(0, 0), 8);

    epio_free(epio);
}

// 11.5.5: over a long run the average divisor is n + f/256.
static void clkdiv_fractional_average(void **state) {
    (void)state;
    epio_t *epio = counter_setup(CLKDIV(3, 64), 0x1, 1);

    // 3.25 * 4000 = 13000 cycles
    epio_step_cycles(epio, 13000);
    assert_int_equal(EXECUTED(0, 0), 4000);

    epio_free(epio);
}

// 11.5.5: delay cycles are SM cycles, so with a divisor they last n system
// clock cycles each.
static void clkdiv_delay_counts_sm_cycles(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    epio_set_clkdiv_enabled(epio, 1);
    epio_set_gpio_output_control(epio, 0, 0);
    epio_set_gpio_output(epio, 0);
    epio_set_instr(epio, 0, 0, APIO_ADD_DELAY(APIO_SET_PINS(1), 2));
    epio_set_instr(epio, 0, 1, APIO_SET_PINS(0));
    epio_sm_reg_t reg = {
        .clkdiv = CLKDIV(3, 0),
        .execctrl = WRAP(1, 0),
        .pinctrl = APIO_SET_COUNT(1),
    };
    epio_set_sm_reg(epio, 0, 0, &reg);
    epio_enable_sm(epio, 0, 0);

    // SET on cycle 0, delays on cycles 3 and 6, next SET on cycle 9
    for (int cycle = 0; cycle < 9; cycle++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(epio_read_pin_states(epio) & 1, 1);
    }
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & 1, 0);

    epio_free(epio);
}

// CTRL CLKDIV_RESTART (11.7): setting/clearing SM_ENABLE does not stop the
// clock divider, so enabling an SM part way through a period waits for the
// divider's next pulse.
static void clkdiv_free_running_while_disabled(void **state) {
    (void)state;
    epio_t *epio = counter_setup(CLKDIV(4, 0), 0x1, 1);
    epio_disable_sm(epio, 0, 0);

    // Pulse on cycle 0 while disabled
    epio_step_cycles(epio, 3);
    epio_enable_sm(epio, 0, 0);
    epio_step_cycles(epio, 1);
    assert_int_equal(EXECUTED(0, 0), 0);
    // Next pulse is cycle 4
    epio_step_cycles(epio, 1);
    assert_int_equal(EXECUTED(0, 0), 1);

    epio_free(epio);
}

// CTRL CLKDIV_RESTART (11.7): restarting dividers with the same divisor
// together makes them run in lockstep.
static void clkdiv_restart_lockstep(void **state) {
    (void)state;
    epio_t *epio = counter_setup(CLKDIV(4, 0), 0x3, 1);

    // Both pulse on cycle 0.  Restarting SM 1 after cycle 1 knocks it out of
    // phase: SM 0 pulses on 4, 8...; SM 1 on 2, 6, 10...
    epio_step_cycles(epio, 2);
    epio_restart_clkdiv(epio, 0, 0x2);
    epio_step_cycles(epio, 1);
    assert_int_equal(EXECUTED(0, 0), 1);
    assert_int_equal(EXECUTED(0, 1), 2);
    epio_step_cycles(epio, 4);
    assert_int_equal(EXECUTED(0, 0), 2);
    assert_int_equal(EXECUTED(0, 1), 3);

    // Restart both - they now pulse on the same cycles, so the difference
    // between their counts never changes
    epio_restart_clkdiv(epio, 0, 0x3);
    for (int ii = 0; ii < 12; ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(EXECUTED(0, 1) - EXECUTED(0, 0), 1);
    }
    assert_int_equal(EXECUTED(0, 0), 5);

    epio_free(epio);
}

// CTRL CLKDIV_RESTART (11.7): restart also clears the fractional
// accumulator, so the fractional pattern starts again from its beginning.
static void clkdiv_restart_clears_fraction(void **state) {
    (void)state;
    epio_t *epio = counter_setup(CLKDIV(2, 128), 0x1, 1);

    // Pulses on 0 and 2 - accumulator has wrapped once and is 0 again, then
    // pulse 5 leaves it at 128
    epio_step_cycles(epio, 6);
    assert_int_equal(EXECUTED(0, 0), 3);
    epio_restart_clkdiv(epio, 0, 0x1);

    // From restart: pulses on 0, 2, 5 again
    const uint8_t expected[] = {4, 4, 5, 5, 5, 6};
    for (size_t ii = 0; ii < sizeof(expected); ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(EXECUTED(0, 0), expected[ii]);
    }

    epio_free(epio);
}

// SMx_CLKDIV can be changed while running; the new divisor applies from the
// next division period.
static void clkdiv_change_while_running(void **state) {
    (void)state;
    epio_t *epio = counter_setup(CLKDIV(2, 0), 0x1, 1);

    // Pulses on 0, 2
    epio_step_cycles(epio, 3);
    assert_int_equal(EXECUTED(0, 0), 2);
    epio_sm_reg_t reg = { .clkdiv = CLKDIV(5, 0), .execctrl = WRAP(0, 0) };
    epio_set_sm_reg(epio, 0, 0, &reg);

    // The period started on cycle 2 is still 2 long, so pulse on 4, then 9
    epio_step_cycles(epio, 1);
    assert_int_equal(EXECUTED(0, 0), 2);
    epio_step_cycles(epio, 1);
    assert_int_equal(EXECUTED(0, 0), 3);
    epio_step_cycles(epio, 4);
    assert_int_equal(EXECUTED(0, 0), 3);
    epio_step_cycles(epio, 1);
    assert_int_equal(EXECUTED(0, 0), 4);

    epio_free(epio);
}

static void clkdiv_restart_invalid(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);

    expect_assert_failure(epio_restart_clkdiv(epio, NUM_PIO_BLOCKS, 0x1));
    expect_assert_failure(epio_restart_clkdiv(epio, 0, 0x10));

    epio_free(epio);
}

static void sysclk_hz(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);

    assert_int_equal(epio_get_sysclk_hz(epio), EPIO_DEFAULT_SYSCLK_HZ);
    assert_int_equal(EPIO_DEFAULT_SYSCLK_HZ, 150000000);
    epio_set_sysclk_hz(epio, 200000000);
    assert_int_equal(epio_get_sysclk_hz(epio), 200000000);
    expect_assert_failure(epio_set_sysclk_hz(epio, 0));

    epio_free(epio);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(clkdiv_ignored_by_default),
        cmocka_unit_test(clkdiv_enable_flag),
        cmocka_unit_test(clkdiv_one_runs_every_cycle),
        cmocka_unit_test(clkdiv_integer),
        cmocka_unit_test(clkdiv_int_zero_is_65536),
        cmocka_unit_test(clkdiv_fractional),
        cmocka_unit_test(clkdiv_fractional_average),
        cmocka_unit_test(clkdiv_delay_counts_sm_cycles),
        cmocka_unit_test(clkdiv_free_running_while_disabled),
        cmocka_unit_test(clkdiv_restart_lockstep),
        cmocka_unit_test(clkdiv_restart_clears_fraction),
        cmocka_unit_test(clkdiv_change_while_running),
        cmocka_unit_test(clkdiv_restart_invalid),
        cmocka_unit_test(sysclk_hz),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
