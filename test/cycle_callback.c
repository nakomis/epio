// Copyright (C) 2026 Martin Harris <epio@nakomis.com>
//
// MIT License

// epio - A PIO emulator
//
// Unit tests for the per-cycle host callback and host-owned GPIOs

#define APIO_LOG_IMPL
#include "test.h"

#define WRAP(TOP, BOTTOM)   (((TOP) << 12) | ((BOTTOM) << 7))
#define GPIO(N)             (1ULL << (N))
#define MAX_CALLS           16

typedef struct {
    int calls;
    epio_t *epio[MAX_CALLS];
    uint64_t cycle[MAX_CALLS];
    uint64_t out[MAX_CALLS];
    uint64_t oe[MAX_CALLS];
    uint64_t changed[MAX_CALLS];
    // If >= 0, raise GPIO 5 at the end of this cycle
    int64_t raise_gpio5_at;
} record_t;

static void record_cycle(
    epio_t *epio,
    uint64_t cycle,
    uint64_t out,
    uint64_t oe,
    uint64_t changed,
    void *ctx
) {
    record_t *rec = (record_t *)ctx;
    assert_true(rec->calls < MAX_CALLS);
    rec->epio[rec->calls] = epio;
    rec->cycle[rec->calls] = cycle;
    rec->out[rec->calls] = out;
    rec->oe[rec->calls] = oe;
    rec->changed[rec->calls] = changed;
    rec->calls++;
    if ((int64_t)cycle == rec->raise_gpio5_at) {
        epio_set_gpio_input_level(epio, 5, 1);
    }
}

// Block 0 SM 0: SET PINDIRS 1, then alternately SET PINS 1 and SET PINS 0,
// on GPIO 3.
static epio_t *toggle_setup(void) {
    epio_t *epio = epio_init();
    assert_non_null(epio);
    epio_set_gpio_output_control(epio, 3, 0);
    epio_set_instr(epio, 0, 0, APIO_SET_PIN_DIRS(1));
    epio_set_instr(epio, 0, 1, APIO_SET_PINS(1));
    epio_set_instr(epio, 0, 2, APIO_SET_PINS(0));
    epio_sm_reg_t reg = {
        .execctrl = WRAP(2, 1),
        .pinctrl = APIO_SET_BASE(3) | APIO_SET_COUNT(1),
    };
    epio_set_sm_reg(epio, 0, 0, &reg);
    epio_enable_sm(epio, 0, 0);
    return epio;
}

// The callback is made once per cycle, with the cycle just executed and the
// context pointer.
static void callback_every_cycle(void **state) {
    (void)state;
    epio_t *epio = toggle_setup();
    record_t rec = { .raise_gpio5_at = -1 };
    epio_set_cycle_callback(epio, record_cycle, &rec);

    epio_step_cycles(epio, 3);
    epio_step_cycles(epio, 2);
    assert_int_equal(rec.calls, 5);
    for (int ii = 0; ii < 5; ii++) {
        assert_ptr_equal(rec.epio[ii], epio);
        assert_int_equal(rec.cycle[ii], ii);
    }
    assert_int_equal(epio_get_cycle_count(epio), 5);

    epio_free(epio);
}

// The callback sees output levels, output enables, and what changed.
static void callback_reports_outputs(void **state) {
    (void)state;
    epio_t *epio = toggle_setup();
    record_t rec = { .raise_gpio5_at = -1 };
    epio_set_cycle_callback(epio, record_cycle, &rec);

    epio_step_cycles(epio, 4);

    // SET PINDIRS 1: enable changes
    assert_int_equal(rec.oe[0] & GPIO(3), GPIO(3));
    assert_int_equal(rec.out[0] & GPIO(3), 0);
    assert_int_equal(rec.changed[0], GPIO(3));
    // SET PINS 1: level changes
    assert_int_equal(rec.out[1] & GPIO(3), GPIO(3));
    assert_int_equal(rec.changed[1], GPIO(3));
    // SET PINS 0
    assert_int_equal(rec.out[2] & GPIO(3), 0);
    assert_int_equal(rec.changed[2], GPIO(3));
    // SET PINS 1
    assert_int_equal(rec.changed[3], GPIO(3));

    epio_free(epio);
}

// A cycle that changes nothing reports no changes.
static void callback_no_change(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    epio_set_instr(epio, 0, 0, APIO_NOP);
    epio_enable_sm(epio, 0, 0);
    record_t rec = { .raise_gpio5_at = -1 };
    epio_set_cycle_callback(epio, record_cycle, &rec);

    epio_step_cycles(epio, 2);
    assert_int_equal(rec.changed[0], 0);
    assert_int_equal(rec.changed[1], 0);

    epio_free(epio);
}

// Changes are relative to the outputs when the callback was set, so earlier
// changes are not reported.
static void callback_changes_relative_to_set(void **state) {
    (void)state;
    epio_t *epio = toggle_setup();
    epio_step_cycles(epio, 2);

    record_t rec = { .raise_gpio5_at = -1 };
    epio_set_cycle_callback(epio, record_cycle, &rec);
    epio_set_instr(epio, 0, 2, APIO_SET_PINS(1));
    epio_step_cycles(epio, 1);
    assert_int_equal(rec.calls, 1);
    assert_int_equal(rec.changed[0], 0);

    epio_free(epio);
}

// GPIOs no PIO block controls are host-owned: PIO doesn't write them, and the
// host's changes between steps are reported by the next callback.
static void callback_host_owned_gpio(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    // SET PINDIRS / SET PINS on GPIO 12, which block 0 doesn't control
    epio_set_instr(epio, 0, 0, APIO_SET_PIN_DIRS(1));
    epio_set_instr(epio, 0, 1, APIO_SET_PINS(1));
    epio_sm_reg_t reg = {
        .execctrl = WRAP(1, 0),
        .pinctrl = APIO_SET_BASE(12) | APIO_SET_COUNT(1),
    };
    epio_set_sm_reg(epio, 0, 0, &reg);
    epio_enable_sm(epio, 0, 0);
    record_t rec = { .raise_gpio5_at = -1 };
    epio_set_cycle_callback(epio, record_cycle, &rec);

    epio_step_cycles(epio, 2);
    assert_int_equal(rec.oe[1] & GPIO(12), 0);
    assert_int_equal(rec.out[1] & GPIO(12), 0);
    assert_int_equal(rec.changed[1], 0);

    // The host drives it
    epio_set_gpio_output(epio, 12);
    epio_set_gpio_output_level(epio, 12, 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(rec.oe[2] & GPIO(12), GPIO(12));
    assert_int_equal(rec.out[2] & GPIO(12), GPIO(12));
    assert_int_equal(rec.changed[2], GPIO(12));

    // Nothing changes on the next cycle
    epio_step_cycles(epio, 1);
    assert_int_equal(rec.changed[3], 0);

    epio_free(epio);
}

// Inputs the callback sets take effect from the next cycle - so a host
// model of a device can respond to PIO.
static void callback_sets_inputs(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    epio_set_instr(epio, 0, 0, APIO_WAIT_GPIO_HIGH(5));
    epio_set_instr(epio, 0, 1, APIO_NOP);
    epio_sm_reg_t reg = { .execctrl = WRAP(1, 1) };
    epio_set_sm_reg(epio, 0, 0, &reg);
    epio_enable_sm(epio, 0, 0);
    record_t rec = { .raise_gpio5_at = 2 };
    epio_set_cycle_callback(epio, record_cycle, &rec);

    epio_step_cycles(epio, 3);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 0);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 1);

    epio_free(epio);
}

// Setting a NULL callback removes it.
static void callback_removed(void **state) {
    (void)state;
    epio_t *epio = toggle_setup();
    record_t rec = { .raise_gpio5_at = -1 };
    epio_set_cycle_callback(epio, record_cycle, &rec);
    epio_step_cycles(epio, 1);
    epio_set_cycle_callback(epio, NULL, NULL);
    epio_step_cycles(epio, 2);
    assert_int_equal(rec.calls, 1);

    epio_free(epio);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(callback_every_cycle),
        cmocka_unit_test(callback_reports_outputs),
        cmocka_unit_test(callback_no_change),
        cmocka_unit_test(callback_changes_relative_to_set),
        cmocka_unit_test(callback_host_owned_gpio),
        cmocka_unit_test(callback_sets_inputs),
        cmocka_unit_test(callback_removed),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
