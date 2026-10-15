// Copyright (C) 2026 Martin Harris <epio@nakomis.com>
//
// MIT License

// epio - A PIO emulator
//
// Unit tests for loading pioasm c-sdk output, and the pico-sdk compatible
// SM configuration functions.
//
// The headers in test/pioasm/ were generated from the .pio files beside
// them by pioasm 2.2.0:
//
//     pioasm -o c-sdk <name>.pio <name>.pio.h

#define APIO_LOG_IMPL
#include "test.h"
#include "epio_pioasm.h"
#include "pulse_train.pio.h"
#include "clocked_out.pio.h"

#define GPIO(N)     (1ULL << (N))

// The pulse_train program, loaded at a non-zero offset, runs end to end:
// X + 1 pulses on its SET pin, then it idles.  Its JMPs only work if they
// were relocated.
static void pioasm_pulse_train_runs(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    epio_set_gpio_output_control(epio, 4, 0);
    epio_set_gpio_output(epio, 4);

    uint offset = epio_add_program(epio, 0, &pulse_train_program, 7);
    assert_int_equal(offset, 7);
    pio_sm_config c = pulse_train_config(offset, 4);
    epio_sm_init(epio, 0, 0, offset, &c);
    epio_enable_sm(epio, 0, 0);
    epio_push_tx_fifo(epio, 0, 0, 2);

    int rising = 0;
    uint8_t last = 0;
    for (int ii = 0; ii < 30; ii++) {
        epio_step_cycles(epio, 1);
        uint8_t level = (epio_read_pin_states(epio) & GPIO(4)) != 0;
        if (level && !last) {
            rising++;
        }
        last = level;
    }
    assert_int_equal(rising, 3);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), offset + pulse_train_wrap);

    epio_free(epio);
}

// pio_add_program_at_offset(): JMP targets are relocated by the offset;
// other instructions are copied unchanged.
static void pioasm_add_program_relocates_jmps(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);

    epio_add_program(epio, 1, &pulse_train_program, 20);
    for (int ii = 0; ii < pulse_train_program.length; ii++) {
        uint16_t instr = pulse_train_program_instructions[ii];
        uint16_t expected = ((instr >> 13) == 0) ? instr + 20 : instr;
        assert_int_equal(epio_get_instr(epio, 1, 20 + ii), expected);
    }
    // jmp x--, 2 -> jmp x--, 22
    assert_int_equal(epio_get_instr(epio, 1, 24), 0x0042 + 20);

    epio_free(epio);
}

// pio_add_program_at_offset(): with GPIOBASE 16, WAIT GPIO's 5 bit GPIO
// number has bit 4 flipped, so GPIO 20 is index 4 in the block's window.
static void pioasm_add_program_wait_gpio_gpiobase(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    epio_set_gpiobase(epio, 1, 16);

    epio_add_program(epio, 0, &clocked_out_program, 0);
    epio_add_program(epio, 1, &clocked_out_program, 0);
    assert_int_equal(epio_get_instr(epio, 0, 0) & 0x1F, 20);
    assert_int_equal(epio_get_instr(epio, 1, 0) & 0x1F, 4);

    // And block 1's SM waits on GPIO 20
    pio_sm_config c = clocked_out_program_get_default_config(0);
    epio_sm_init(epio, 1, 0, 0, &c);
    epio_enable_sm(epio, 1, 0);
    epio_step_cycles(epio, 2);
    assert_int_equal(epio_peek_sm_pc(epio, 1, 0), 0);
    epio_set_gpio_input_level(epio, 20, 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_pc(epio, 1, 0), 1);

    epio_free(epio);
}

static void pioasm_add_program_invalid(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);

    expect_assert_failure(epio_add_program(epio, NUM_PIO_BLOCKS, &pulse_train_program, 0));
    expect_assert_failure(epio_add_program(epio, 0, NULL, 0));
    // 6 instructions don't fit at 27
    expect_assert_failure(epio_add_program(epio, 0, &pulse_train_program, 27));
    // .origin must match
    pio_program_t origin = pulse_train_program;
    origin.origin = 3;
    expect_assert_failure(epio_add_program(epio, 0, &origin, 4));
    assert_int_equal(epio_add_program(epio, 0, &origin, 3), 3);

    epio_free(epio);
}

// pio_get_default_sm_config(): divider 1, wrap 0-31, shift right both ways,
// thresholds 32 (encoded 0), everything else 0.
static void pioasm_default_config(void **state) {
    (void)state;
    pio_sm_config c = pio_get_default_sm_config();
    assert_int_equal(c.clkdiv, 1u << 16);
    assert_int_equal(c.execctrl, 31u << 12);
    assert_int_equal(c.shiftctrl, APIO_IN_SHIFTDIR_R | APIO_OUT_SHIFTDIR_R);
    assert_int_equal(c.pinctrl, 0);
}

// pioasm's generated config for .side_set 2 opt pindirs: 3 bits including
// the enable, SIDE_EN and SIDE_PINDIR set, and the program's wrap.
static void pioasm_generated_sideset_config(void **state) {
    (void)state;
    pio_sm_config c = clocked_out_program_get_default_config(10);
    assert_int_equal((c.pinctrl >> 29) & 0x7, 3);
    assert_int_equal((c.execctrl >> 30) & 0x1, 1);
    assert_int_equal((c.execctrl >> 29) & 0x1, 1);
    assert_int_equal((c.execctrl >> 7) & 0x1F, 11);
    assert_int_equal((c.execctrl >> 12) & 0x1F, 12);
}

// The pin setters use the same PINCTRL fields as apio's encoders, modulo 32.
static void pioasm_pin_setters(void **state) {
    (void)state;
    pio_sm_config c = {0};
    sm_config_set_out_pins(&c, 18, 23);
    sm_config_set_set_pins(&c, 33, 5);
    sm_config_set_in_pins(&c, 7);
    sm_config_set_sideset_pins(&c, 16);
    assert_int_equal(c.pinctrl,
        APIO_OUT_BASE(18) | APIO_OUT_COUNT(23) |
        APIO_SET_BASE(1) | APIO_SET_COUNT(5) |
        APIO_IN_BASE(7) | APIO_SIDE_SET_BASE(16));

    // Setters replace their field
    sm_config_set_out_pins(&c, 0, 32);
    assert_int_equal(c.pinctrl & 0x1F, 0);
    assert_int_equal((c.pinctrl >> 20) & 0x3F, 32);
    assert_int_equal((c.pinctrl >> 5) & 0x1F, 1);

    sm_config_set_in_pin_count(&c, 10);
    assert_int_equal(c.shiftctrl & 0x1F, 10);

    sm_config_set_jmp_pin(&c, 41);
    assert_int_equal((c.execctrl >> 24) & 0x1F, 9);

    // ... and record each pin's 16-pin range, as the pico-sdk's pinhi does
    assert_int_equal((c.pinhi >> 0) & 0x1F, 0);    // OUT 0
    assert_int_equal((c.pinhi >> 5) & 0x1F, 2);    // SET 33
    assert_int_equal((c.pinhi >> 10) & 0x1F, 1);   // SIDESET 16
    assert_int_equal((c.pinhi >> 15) & 0x1F, 0);   // IN 7
    assert_int_equal((c.pinhi >> 20) & 0x1F, 2);   // JMP_PIN 41
    expect_assert_failure(sm_config_set_out_pin_base(&c, 48));
    expect_assert_failure(sm_config_set_jmp_pin(&c, 48));
}

// pio_sm_set_config() with GPIOBASE 16: absolute pins are moved into the
// block's window (GPIO 16-47 -> 0-31), as on hardware via the pico-sdk.
static void pioasm_sm_init_gpiobase_16(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    epio_set_gpiobase(epio, 1, 16);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_out_pins(&c, 18, 23);
    sm_config_set_sideset_pins(&c, 16);
    sm_config_set_set_pins(&c, 40, 2);
    sm_config_set_in_pins(&c, 32);
    sm_config_set_jmp_pin(&c, 47);
    epio_sm_init(epio, 1, 2, 0, &c);

    epio_sm_reg_t reg;
    epio_get_sm_reg(epio, 1, 2, &reg);
    assert_int_equal(reg.pinctrl & 0x1F, 2);            // OUT_BASE: GPIO 18
    assert_int_equal((reg.pinctrl >> 10) & 0x1F, 0);    // SIDESET_BASE: GPIO 16
    assert_int_equal((reg.pinctrl >> 5) & 0x1F, 24);    // SET_BASE: GPIO 40
    assert_int_equal((reg.pinctrl >> 15) & 0x1F, 16);   // IN_BASE: GPIO 32
    assert_int_equal((reg.execctrl >> 24) & 0x1F, 31);  // JMP_PIN: GPIO 47
    assert_int_equal((reg.pinctrl >> 20) & 0x3F, 23);   // counts untouched
    assert_int_equal((reg.pinctrl >> 26) & 0x07, 2);

    // A pin set by hand into the window's lower half stays where it is if
    // its field was never set through a setter (pinhi 0b11111)
    pio_sm_config d = pio_get_default_sm_config();
    d.pinctrl = 5;   // OUT_BASE 5, relative
    epio_sm_init(epio, 1, 3, 0, &d);
    epio_get_sm_reg(epio, 1, 3, &reg);
    assert_int_equal(reg.pinctrl & 0x1F, 5);

    epio_free(epio);
}

// With GPIOBASE 0, pins 0-31 are applied as given
static void pioasm_sm_init_gpiobase_0(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_out_pins(&c, 3, 8);
    sm_config_set_sideset_pins(&c, 20);
    sm_config_set_jmp_pin(&c, 31);
    epio_sm_init(epio, 0, 0, 0, &c);
    epio_sm_reg_t reg;
    epio_get_sm_reg(epio, 0, 0, &reg);
    assert_int_equal(reg.pinctrl, c.pinctrl);
    assert_int_equal(reg.execctrl, c.execctrl);

    epio_free(epio);
}

// A config whose pins can't all be reached from the block's window, as
// pio_sm_set_config()'s PICO_ERROR_BAD_ALIGNMENT
static void pioasm_sm_init_pins_outside_window(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    epio_set_gpiobase(epio, 1, 16);

    pio_sm_config low = pio_get_default_sm_config();
    sm_config_set_out_pins(&low, 3, 4);
    expect_assert_failure(epio_sm_init(epio, 1, 0, 0, &low));

    pio_sm_config high = pio_get_default_sm_config();
    sm_config_set_jmp_pin(&high, 40);
    expect_assert_failure(epio_sm_init(epio, 0, 0, 0, &high));

    epio_free(epio);
}

// End to end: pulse_train given absolute SET pin 16 in a GPIOBASE 16 block
// pulses GPIO 16 (before the fix it pulsed GPIO 32)
static int edges_16, edges_32;
static void count_edges(epio_t *e, uint64_t cycle, uint64_t out, uint64_t oe, uint64_t changed, void *ctx) {
    (void)e; (void)cycle; (void)oe; (void)ctx;
    if ((changed >> 16) & (out >> 16) & 1) edges_16++;
    if ((changed >> 32) & (out >> 32) & 1) edges_32++;
}

static void pioasm_gpiobase_16_drives_absolute_pin(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    epio_set_gpiobase(epio, 0, 16);
    for (uint8_t pin = 16; pin < 48; pin++) {
        epio_set_gpio_output_control(epio, pin, 0);
        epio_set_gpio_output(epio, pin);
    }
    uint off = epio_add_program(epio, 0, &pulse_train_program, 0);
    pio_sm_config c = pulse_train_config(off, 16);
    epio_sm_init(epio, 0, 0, off, &c);
    edges_16 = edges_32 = 0;
    epio_set_cycle_callback(epio, count_edges, NULL);
    epio_push_tx_fifo(epio, 0, 0, 2);   // three pulses
    epio_enable_sm(epio, 0, 0);
    epio_step_cycles(epio, 40);
    assert_int_equal(edges_16, 3);
    assert_int_equal(edges_32, 0);

    epio_free(epio);
}

static void pioasm_shift_and_clkdiv_setters(void **state) {
    (void)state;
    pio_sm_config c = {0};
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_in_shift(&c, true, true, 32);
    assert_int_equal(c.shiftctrl,
        APIO_OUT_SHIFTDIR_L | APIO_AUTOPULL | APIO_PULL_THRESH(8) |
        APIO_IN_SHIFTDIR_R | APIO_AUTOPUSH | APIO_PUSH_THRESH(0));

    sm_config_set_clkdiv_int_frac(&c, 600, 128);
    assert_int_equal(c.clkdiv, (600u << 16) | (128u << 8));
    sm_config_set_clkdiv_int_frac8(&c, 0, 0);
    assert_int_equal(c.clkdiv, 0);

    sm_config_set_out_special(&c, true, true, 7);
    assert_int_equal(c.execctrl, (1u << 17) | (1u << 18) | (7u << 19));
}

static void pioasm_setters_invalid(void **state) {
    (void)state;
    pio_sm_config c = {0};
    expect_assert_failure(sm_config_set_out_pin_count(&c, 33));
    expect_assert_failure(sm_config_set_set_pin_count(&c, 6));
    expect_assert_failure(sm_config_set_in_pin_count(&c, 0));
    expect_assert_failure(sm_config_set_in_pin_count(&c, 33));
    expect_assert_failure(sm_config_set_sideset(&c, 6, false, false));
    expect_assert_failure(sm_config_set_sideset(&c, 0, true, false));
    expect_assert_failure(sm_config_set_clkdiv_int_frac8(&c, 65536, 0));
    expect_assert_failure(sm_config_set_clkdiv_int_frac8(&c, 0, 1));
    expect_assert_failure(sm_config_set_wrap(&c, 0, 32));
    expect_assert_failure(sm_config_set_wrap(&c, 32, 0));
    expect_assert_failure(sm_config_set_in_shift(&c, true, false, 33));
    expect_assert_failure(sm_config_set_out_shift(&c, true, false, 33));
}

// pio_sm_init(): disables the SM, applies the config, clears the FIFOs and
// the shift, delay and stall state, and sets the PC.
static void pioasm_sm_init(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    epio_enable_sm(epio, 2, 3);
    epio_push_tx_fifo(epio, 2, 3, 1);
    epio_push_rx_fifo(epio, 2, 3, 2);
    SM(2, 3).isr = 0x1234;
    SM(2, 3).isr_count = 9;
    SM(2, 3).osr_count = 4;
    SM(2, 3).delay = 5;
    SM(2, 3).stalled = 1;
    SM(2, 3).exec_pending = 1;

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_out_pins(&c, 3, 4);
    epio_sm_init(epio, 2, 3, 17, &c);

    assert_int_equal(epio_is_sm_enabled(epio, 2, 3), 0);
    epio_sm_reg_t reg;
    epio_get_sm_reg(epio, 2, 3, &reg);
    assert_int_equal(reg.clkdiv, c.clkdiv);
    assert_int_equal(reg.execctrl, c.execctrl);
    assert_int_equal(reg.shiftctrl, c.shiftctrl);
    assert_int_equal(reg.pinctrl, c.pinctrl);
    assert_int_equal(epio_tx_fifo_depth(epio, 2, 3), 0);
    assert_int_equal(epio_rx_fifo_depth(epio, 2, 3), 0);
    assert_int_equal(SM(2, 3).isr, 0);
    assert_int_equal(SM(2, 3).isr_count, 0);
    assert_int_equal(SM(2, 3).osr_count, 32);
    assert_int_equal(SM(2, 3).delay, 0);
    assert_int_equal(SM(2, 3).stalled, 0);
    assert_int_equal(SM(2, 3).exec_pending, 0);
    assert_int_equal(epio_peek_sm_pc(epio, 2, 3), 17);

    // NULL config is the default config
    epio_sm_init(epio, 2, 3, 0, NULL);
    epio_get_sm_reg(epio, 2, 3, &reg);
    pio_sm_config d = pio_get_default_sm_config();
    assert_int_equal(reg.execctrl, d.execctrl);
    assert_int_equal(reg.pinctrl, d.pinctrl);

    expect_assert_failure(epio_sm_init(epio, 2, 3, 32, NULL));
    expect_assert_failure(epio_sm_init(epio, 3, 0, 0, NULL));

    epio_free(epio);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(pioasm_pulse_train_runs),
        cmocka_unit_test(pioasm_add_program_relocates_jmps),
        cmocka_unit_test(pioasm_add_program_wait_gpio_gpiobase),
        cmocka_unit_test(pioasm_add_program_invalid),
        cmocka_unit_test(pioasm_default_config),
        cmocka_unit_test(pioasm_generated_sideset_config),
        cmocka_unit_test(pioasm_pin_setters),
        cmocka_unit_test(pioasm_shift_and_clkdiv_setters),
        cmocka_unit_test(pioasm_setters_invalid),
        cmocka_unit_test(pioasm_sm_init),
        cmocka_unit_test(pioasm_sm_init_gpiobase_16),
        cmocka_unit_test(pioasm_sm_init_gpiobase_0),
        cmocka_unit_test(pioasm_sm_init_pins_outside_window),
        cmocka_unit_test(pioasm_gpiobase_16_drives_absolute_pin),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
