// Copyright (C) 2026 Martin Harris <epio@nakomis.com>
//
// MIT License

// epio - A PIO emulator
//
// Unit tests for the CTRL register, including RP2350 cross-block
// NEXTPREV operations (RP2350 datasheet 11.1.1, 11.7)

#define APIO_LOG_IMPL
#include "test.h"

#define WRAP(TOP, BOTTOM)       (((TOP) << 12) | ((BOTTOM) << 7))
#define CLKDIV(INT, FRAC)       (((uint32_t)(INT) << 16) | ((uint32_t)(FRAC) << 8))

#define SM_ENABLE(MASK)         ((uint32_t)(MASK))
#define SM_RESTART(MASK)        ((uint32_t)(MASK) << 4)
#define CLKDIV_RESTART(MASK)    ((uint32_t)(MASK) << 8)
#define PREV_PIO_MASK(MASK)     ((uint32_t)(MASK) << 16)
#define NEXT_PIO_MASK(MASK)     ((uint32_t)(MASK) << 20)
#define NEXTPREV_SM_ENABLE      (1u << 24)
#define NEXTPREV_SM_DISABLE     (1u << 25)
#define NEXTPREV_CLKDIV_RESTART (1u << 26)

// Each execution of JMP X-- 0 decrements X, so X counts SM cycles
#define START_X                 1000000
#define EXECUTED(BLOCK, _SM)    (START_X - SM(BLOCK, _SM).x)

// Loads a JMP X-- loop into every SM of every block, all disabled
static epio_t *ctrl_setup(uint32_t clkdiv) {
    epio_t *epio = epio_init();
    assert_non_null(epio);
    for (int block = 0; block < NUM_PIO_BLOCKS; block++) {
        epio_set_instr(epio, block, 0, APIO_JMP_X_DEC(0));
        for (int sm = 0; sm < NUM_SMS_PER_BLOCK; sm++) {
            epio_sm_reg_t reg = { .clkdiv = clkdiv, .execctrl = WRAP(0, 0) };
            epio_set_sm_reg(epio, block, sm, &reg);
            SM(block, sm).x = START_X;
        }
    }
    return epio;
}

// CTRL SM_ENABLE: writing 1/0 to each bit enables/disables each SM.
static void ctrl_sm_enable(void **state) {
    (void)state;
    epio_t *epio = ctrl_setup(0);

    epio_write_ctrl(epio, 0, SM_ENABLE(0b0101));
    assert_int_equal(epio_read_ctrl(epio, 0), 0b0101);
    assert_int_equal(epio_is_sm_enabled(epio, 0, 0), 1);
    assert_int_equal(epio_is_sm_enabled(epio, 0, 1), 0);
    assert_int_equal(epio_is_sm_enabled(epio, 0, 2), 1);
    assert_int_equal(epio_is_sm_enabled(epio, 0, 3), 0);

    // A write sets every bit, so also disables
    epio_write_ctrl(epio, 0, SM_ENABLE(0b0010));
    assert_int_equal(epio_read_ctrl(epio, 0), 0b0010);

    // Other blocks are unaffected
    assert_int_equal(epio_read_ctrl(epio, 1), 0);
    assert_int_equal(epio_read_ctrl(epio, 2), 0);

    epio_free(epio);
}

// CTRL NEXT_PIO_MASK + NEXTPREV_SM_ENABLE: SMs in this block and the next
// start on exactly the same cycle.
static void ctrl_nextprev_enable_same_cycle(void **state) {
    (void)state;
    epio_t *epio = ctrl_setup(0);

    epio_write_ctrl(epio, 0, SM_ENABLE(0b0001) | NEXT_PIO_MASK(0b0010) | NEXTPREV_SM_ENABLE);
    assert_int_equal(epio_read_ctrl(epio, 0), 0b0001);
    assert_int_equal(epio_read_ctrl(epio, 1), 0b0010);
    assert_int_equal(epio_read_ctrl(epio, 2), 0);

    epio_step_cycles(epio, 7);
    assert_int_equal(EXECUTED(0, 0), 7);
    assert_int_equal(EXECUTED(1, 1), 7);

    epio_free(epio);
}

// CTRL PREV_PIO_MASK: the block before block 0 is the highest-numbered
// block; NEXT_PIO_MASK from the highest-numbered block is block 0.
static void ctrl_nextprev_wraps(void **state) {
    (void)state;
    epio_t *epio = ctrl_setup(0);

    epio_write_ctrl(epio, 0, PREV_PIO_MASK(0b1000) | NEXTPREV_SM_ENABLE);
    assert_int_equal(epio_read_ctrl(epio, NUM_PIO_BLOCKS - 1), 0b1000);
    assert_int_equal(epio_read_ctrl(epio, 1), 0);

    epio_write_ctrl(epio, NUM_PIO_BLOCKS - 1, NEXT_PIO_MASK(0b0100) | NEXTPREV_SM_ENABLE);
    assert_int_equal(epio_read_ctrl(epio, 0), 0b0100);

    epio_free(epio);
}

// CTRL NEXTPREV_SM_DISABLE stops SMs in neighbouring blocks, and takes
// precedence when NEXTPREV_SM_ENABLE is also set.
static void ctrl_nextprev_disable(void **state) {
    (void)state;
    epio_t *epio = ctrl_setup(0);
    for (int block = 0; block < NUM_PIO_BLOCKS; block++) {
        epio_write_ctrl(epio, block, SM_ENABLE(0xF));
    }

    epio_write_ctrl(epio, 1,
        SM_ENABLE(0xF) | PREV_PIO_MASK(0b0011) | NEXT_PIO_MASK(0b1100) |
        NEXTPREV_SM_ENABLE | NEXTPREV_SM_DISABLE);
    assert_int_equal(epio_read_ctrl(epio, 0), 0b1100);
    assert_int_equal(epio_read_ctrl(epio, 1), 0b1111);
    assert_int_equal(epio_read_ctrl(epio, 2), 0b0011);

    epio_free(epio);
}

// Without NEXTPREV_SM_ENABLE/DISABLE, the masks alone change nothing.
static void ctrl_masks_alone_do_nothing(void **state) {
    (void)state;
    epio_t *epio = ctrl_setup(0);

    epio_write_ctrl(epio, 1, PREV_PIO_MASK(0xF) | NEXT_PIO_MASK(0xF));
    assert_int_equal(epio_read_ctrl(epio, 0), 0);
    assert_int_equal(epio_read_ctrl(epio, 2), 0);

    epio_free(epio);
}

// CTRL CLKDIV_RESTART restarts this block's dividers.
static void ctrl_clkdiv_restart(void **state) {
    (void)state;
    epio_t *epio = ctrl_setup(CLKDIV(4, 0));
    epio_set_clkdiv_enabled(epio, 1);
    epio_write_ctrl(epio, 0, SM_ENABLE(0b0001));

    // Pulse on cycle 0, next due on cycle 4
    epio_step_cycles(epio, 2);
    assert_int_equal(EXECUTED(0, 0), 1);
    epio_write_ctrl(epio, 0, SM_ENABLE(0b0001) | CLKDIV_RESTART(0b0001));
    epio_step_cycles(epio, 1);
    assert_int_equal(EXECUTED(0, 0), 2);

    epio_free(epio);
}

// The sinter use case (datasheet 11.1.1): one CTRL write enables SMs in two
// blocks and restarts their dividers, so with the same divisor they run in
// exact lockstep - even though both dividers were running out of phase.
static void ctrl_enable_in_sync_across_blocks(void **state) {
    (void)state;
    epio_t *epio = ctrl_setup(CLKDIV(4, 0));
    epio_set_clkdiv_enabled(epio, 1);

    // Knock block 1's dividers out of phase with block 0's
    epio_step_cycles(epio, 1);
    epio_restart_clkdiv(epio, 1, 0xF);
    epio_step_cycles(epio, 2);

    // Equivalent of pio_enable_sm_multi_mask_in_sync(pio0, 0, 1, 1)
    epio_write_ctrl(epio, 0,
        SM_ENABLE(0b0001) | CLKDIV_RESTART(0b0001) |
        NEXT_PIO_MASK(0b0001) | NEXTPREV_SM_ENABLE | NEXTPREV_CLKDIV_RESTART);

    for (int ii = 0; ii < 20; ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(EXECUTED(0, 0), EXECUTED(1, 0));
    }
    assert_int_equal(EXECUTED(0, 0), 5);

    epio_free(epio);
}

// NEXTPREV_CLKDIV_RESTART only restarts the selected SMs' dividers.
static void ctrl_nextprev_clkdiv_restart_masked(void **state) {
    (void)state;
    epio_t *epio = ctrl_setup(CLKDIV(4, 0));
    epio_set_clkdiv_enabled(epio, 1);
    epio_write_ctrl(epio, 2, SM_ENABLE(0b0011));

    // Pulse on cycle 0; restart SM 1 only, from block 1 (NEXT of 1 is 2)
    epio_step_cycles(epio, 2);
    epio_write_ctrl(epio, 1, NEXT_PIO_MASK(0b0010) | NEXTPREV_CLKDIV_RESTART);
    epio_step_cycles(epio, 1);
    assert_int_equal(EXECUTED(2, 0), 1);
    assert_int_equal(EXECUTED(2, 1), 2);

    // Without NEXTPREV_CLKDIV_RESTART the mask restarts nothing: SM 0's next
    // pulse is still cycle 4, where a restart would make it cycle 3
    epio_write_ctrl(epio, 1, NEXT_PIO_MASK(0b0001));
    epio_step_cycles(epio, 1);
    assert_int_equal(EXECUTED(2, 0), 1);

    epio_free(epio);
}

static void ctrl_invalid(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);

    expect_assert_failure(epio_write_ctrl(epio, NUM_PIO_BLOCKS, 0));
    expect_assert_failure(epio_read_ctrl(epio, NUM_PIO_BLOCKS));
    expect_assert_failure(epio_write_ctrl(epio, 0, SM_RESTART(1)));
    expect_assert_failure(epio_write_ctrl(epio, 0, 1u << 12));
    expect_assert_failure(epio_write_ctrl(epio, 0, 1u << 27));

    epio_free(epio);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(ctrl_sm_enable),
        cmocka_unit_test(ctrl_nextprev_enable_same_cycle),
        cmocka_unit_test(ctrl_nextprev_wraps),
        cmocka_unit_test(ctrl_nextprev_disable),
        cmocka_unit_test(ctrl_masks_alone_do_nothing),
        cmocka_unit_test(ctrl_clkdiv_restart),
        cmocka_unit_test(ctrl_enable_in_sync_across_blocks),
        cmocka_unit_test(ctrl_nextprev_clkdiv_restart_masked),
        cmocka_unit_test(ctrl_invalid),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
