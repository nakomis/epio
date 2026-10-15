// Copyright (C) 2026 Martin Harris <epio@nakomis.com>
//
// MIT License

// epio - A PIO emulator
//
// Unit tests for side-set (RP2350 datasheet 11.5.1)

#define APIO_LOG_IMPL
#include "test.h"

#define GPIO(N)         (1ULL << (N))

// EXECCTRL fields not provided by apio
#define WRAP(TOP, BOTTOM)   (((TOP) << 12) | ((BOTTOM) << 7))
#define SIDE_EN             (1u << 30)
#define SIDE_PINDIR         (1u << 29)

// Encode a side-set value into the top COUNT bits of the 5 bit Delay/side-set
// field.  With SIDE_EN, VAL includes the enable bit as its MSB.
#define SIDE(INSTR, COUNT, VAL) \
    ((uint16_t)((INSTR) | ((VAL) << (13 - (COUNT)))))

// Creates an epio instance with a program loaded at instruction 0 for block 0
// SM 0, wrapping over the whole program, with block 0 controlling the GPIOs
// in pin_mask.
static epio_t *sideset_setup(
    uint32_t execctrl,
    uint32_t shiftctrl,
    uint32_t pinctrl,
    const uint16_t *prog,
    uint8_t len,
    uint64_t pin_mask
) {
    epio_t *epio = epio_init();
    assert_non_null(epio);

    for (int pin = 0; pin < NUM_GPIOS; pin++) {
        if (pin_mask & GPIO(pin)) {
            epio_set_gpio_output_control(epio, pin, 0);
        }
    }
    for (int ii = 0; ii < len; ii++) {
        epio_set_instr(epio, 0, ii, prog[ii]);
    }
    epio_sm_reg_t reg = {
        .execctrl = execctrl | WRAP(len - 1, 0),
        .shiftctrl = shiftctrl,
        .pinctrl = pinctrl,
    };
    epio_set_sm_reg(epio, 0, 0, &reg);
    epio_enable_sm(epio, 0, 0);

    return epio;
}

// 11.5.1: every instruction performs a side-set when SIDESET_COUNT is
// nonzero and SIDE_EN is clear (the spi_tx_fast example).
static void sideset_drives_pin_level(void **state) {
    (void)state;
    const uint16_t prog[] = {
        SIDE(APIO_NOP, 1, 1),
        SIDE(APIO_NOP, 1, 0),
    };
    epio_t *epio = sideset_setup(
        0, 0, APIO_SIDE_SET_COUNT(1) | APIO_SIDE_SET_BASE(3),
        prog, 2, GPIO(3));
    epio_set_gpio_output(epio, 3);

    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & GPIO(3), GPIO(3));
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & GPIO(3), 0);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & GPIO(3), GPIO(3));

    epio_free(epio);
}

// 11.5.1: side-set takes the MSBs of the field, the remaining LSBs are delay,
// and the LSB of the side-set data maps to SIDESET_BASE.
static void sideset_multiple_bits_with_delay(void **state) {
    (void)state;
    const uint16_t prog[] = {
        APIO_ADD_DELAY(SIDE(APIO_NOP, 2, 0b10), 2),
        SIDE(APIO_NOP, 2, 0b01),
    };
    epio_t *epio = sideset_setup(
        0, 0, APIO_SIDE_SET_COUNT(2) | APIO_SIDE_SET_BASE(4),
        prog, 2, GPIO(4) | GPIO(5));
    epio_set_gpio_output(epio, 4);
    epio_set_gpio_output(epio, 5);

    // Instruction cycle plus 2 delay cycles
    for (int ii = 0; ii < 3; ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal((epio_read_pin_states(epio) >> 4) & 0b11, 0b10);
    }
    epio_step_cycles(epio, 1);
    assert_int_equal((epio_read_pin_states(epio) >> 4) & 0b11, 0b01);

    epio_free(epio);
}

// 11.5.1 / EXECCTRL_SIDE_EN: the MSB of the side-set bits is an enable, and
// SIDESET_COUNT includes it.  Instructions with the enable clear leave the
// side-set pins alone.
static void sideset_opt_enable_bit(void **state) {
    (void)state;
    const uint16_t prog[] = {
        SIDE(APIO_NOP, 2, 0b11),    // enabled, data 1
        APIO_NOP,                   // not enabled
        SIDE(APIO_NOP, 2, 0b10),    // enabled, data 0
    };
    epio_t *epio = sideset_setup(
        SIDE_EN, 0, APIO_SIDE_SET_COUNT(2) | APIO_SIDE_SET_BASE(2),
        prog, 3, GPIO(2) | GPIO(3));
    epio_set_gpio_output(epio, 2);
    epio_set_gpio_output(epio, 3);

    epio_step_cycles(epio, 1);
    assert_int_equal((epio_read_pin_states(epio) >> 2) & 0b11, 0b01);
    epio_step_cycles(epio, 1);
    assert_int_equal((epio_read_pin_states(epio) >> 2) & 0b11, 0b01);
    epio_step_cycles(epio, 1);
    // Only one data bit, so the enable bit's position (GPIO 3) is never
    // written
    assert_int_equal((epio_read_pin_states(epio) >> 2) & 0b11, 0b00);

    epio_free(epio);
}

// EXECCTRL_SIDE_PINDIR: side-set data goes to pin directions, not levels.
static void sideset_pindirs(void **state) {
    (void)state;
    const uint16_t prog[] = {
        SIDE(APIO_NOP, 1, 1),
        SIDE(APIO_NOP, 1, 0),
    };
    epio_t *epio = sideset_setup(
        SIDE_PINDIR, 0, APIO_SIDE_SET_COUNT(1) | APIO_SIDE_SET_BASE(6),
        prog, 2, GPIO(6));
    epio_set_gpio_output_level(epio, 6, 1);

    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_driven_pins(epio) & GPIO(6), GPIO(6));
    assert_int_equal(epio_read_pin_states(epio) & GPIO(6), GPIO(6));
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_driven_pins(epio) & GPIO(6), 0);

    epio_free(epio);
}

// 11.5.1, 11.5.6: if side-set and a SET from the same SM write the same pin on
// the same cycle, the side-set data is used.
static void sideset_beats_set_on_same_pin(void **state) {
    (void)state;
    const uint16_t prog[] = {
        SIDE(APIO_SET_PINS(0), 1, 1),
        SIDE(APIO_SET_PINS(1), 1, 0),
    };
    epio_t *epio = sideset_setup(
        0, 0,
        APIO_SIDE_SET_COUNT(1) | APIO_SIDE_SET_BASE(7) |
        APIO_SET_COUNT(1) | APIO_SET_BASE(7),
        prog, 2, GPIO(7));
    epio_set_gpio_output(epio, 7);

    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & GPIO(7), GPIO(7));
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & GPIO(7), 0);

    epio_free(epio);
}

// 11.5.1, 11.5.6: OUT and side-set on different pins both take effect on the
// same cycle.
static void sideset_and_out_same_cycle(void **state) {
    (void)state;
    const uint16_t prog[] = {
        SIDE(APIO_PULL_BLOCK, 1, 0),
        SIDE(APIO_OUT_PINS(2), 1, 1),
    };
    epio_t *epio = sideset_setup(
        0, APIO_OUT_SHIFTDIR_R,
        APIO_SIDE_SET_COUNT(1) | APIO_SIDE_SET_BASE(8) |
        APIO_OUT_COUNT(2) | APIO_OUT_BASE(0),
        prog, 2, GPIO(0) | GPIO(1) | GPIO(8));
    epio_set_gpio_output(epio, 0);
    epio_set_gpio_output(epio, 1);
    epio_set_gpio_output(epio, 8);
    epio_push_tx_fifo(epio, 0, 0, 0b11);

    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & 0x103, 0);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & 0x103, 0x103);

    epio_free(epio);
}

// 11.2.5, 11.5.1: side-set takes effect immediately on a stalled instruction.
// 11.5.4: "side-set still takes place on cycles where the instruction
// stalls", so it is re-asserted on every stalled cycle - shown here by
// another writer changing the pin mid-stall.
static void sideset_applies_while_stalled(void **state) {
    (void)state;
    const uint16_t prog[] = {
        SIDE(APIO_PULL_BLOCK, 1, 1),
        SIDE(APIO_NOP, 1, 0),
    };
    epio_t *epio = sideset_setup(
        0, 0, APIO_SIDE_SET_COUNT(1) | APIO_SIDE_SET_BASE(9),
        prog, 2, GPIO(9));
    epio_set_gpio_output(epio, 9);

    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_stalled(epio, 0, 0), 1);
    assert_int_equal(epio_read_pin_states(epio) & GPIO(9), GPIO(9));

    epio_set_gpio_output_level(epio, 9, 0);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_stalled(epio, 0, 0), 1);
    assert_int_equal(epio_read_pin_states(epio) & GPIO(9), GPIO(9));

    // Unstall - the PULL completes and the NOP's side-set follows
    epio_push_tx_fifo(epio, 0, 0, 0);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & GPIO(9), 0);

    epio_free(epio);
}

// 11.5.4: an OUT stalled by autopull on an empty TX FIFO still performs its
// side-set, so a clock driven by side-set freezes in the OUT's phase while
// the FIFO is starved.
static void sideset_holds_during_autopull_underrun(void **state) {
    (void)state;
    const uint16_t prog[] = {
        SIDE(APIO_OUT_PINS(1), 1, 1),
        SIDE(APIO_NOP, 1, 0),
    };
    epio_t *epio = sideset_setup(
        0,
        APIO_OUT_SHIFTDIR_R | APIO_AUTOPULL | APIO_PULL_THRESH(1),
        APIO_SIDE_SET_COUNT(1) | APIO_SIDE_SET_BASE(10) |
        APIO_OUT_COUNT(1) | APIO_OUT_BASE(0),
        prog, 2, GPIO(0) | GPIO(10));
    epio_set_gpio_output(epio, 0);
    epio_set_gpio_output(epio, 10);
    epio_push_tx_fifo(epio, 0, 0, 1);

    // 11.5.4.2: an OUT cannot fill an empty OSR and shift it out on the same
    // cycle, so the first OUT stalls for a cycle while autopull fills the
    // OSR - with its side-set applied.
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 0);
    assert_int_equal(epio_read_pin_states(epio) & (GPIO(0) | GPIO(10)), GPIO(10));

    // OUT completes, then NOP
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & (GPIO(0) | GPIO(10)), GPIO(0) | GPIO(10));
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & GPIO(10), 0);

    // OSR at threshold and FIFO empty - the OUT stalls with side-set high
    for (int ii = 0; ii < 5; ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 0);
        assert_int_equal(epio_read_pin_states(epio) & GPIO(10), GPIO(10));
    }

    epio_free(epio);
}

// 11.5.6: side-set pins wrap after GPIO 31 of the block's 32 pin window, which
// starts at GPIOBASE.
static void sideset_wraps_with_gpiobase16(void **state) {
    (void)state;
    const uint16_t prog[] = {
        SIDE(APIO_NOP, 3, 0b101),
    };
    epio_t *epio = sideset_setup(
        0, 0, APIO_SIDE_SET_COUNT(3) | APIO_SIDE_SET_BASE(31),
        prog, 1, GPIO(47) | GPIO(16) | GPIO(17));
    epio_set_gpiobase(epio, 0, 16);
    epio_set_gpio_output(epio, 47);
    epio_set_gpio_output(epio, 16);
    epio_set_gpio_output(epio, 17);

    epio_step_cycles(epio, 1);
    uint64_t pins = epio_read_pin_states(epio);
    assert_int_equal(pins & GPIO(47), GPIO(47));
    assert_int_equal(pins & GPIO(16), 0);
    assert_int_equal(pins & GPIO(17), GPIO(17));

    epio_free(epio);
}

// Side-set only writes GPIOs the block controls.
static void sideset_ignores_uncontrolled_pins(void **state) {
    (void)state;
    const uint16_t prog[] = {
        SIDE(APIO_NOP, 2, 0b11),
    };
    epio_t *epio = sideset_setup(
        0, 0, APIO_SIDE_SET_COUNT(2) | APIO_SIDE_SET_BASE(11),
        prog, 1, GPIO(11));
    epio_set_gpio_output(epio, 11);
    epio_set_gpio_output(epio, 12);

    epio_step_cycles(epio, 1);
    uint64_t pins = epio_read_pin_states(epio);
    assert_int_equal(pins & GPIO(11), GPIO(11));
    assert_int_equal(pins & GPIO(12), 0);

    epio_free(epio);
}

// 11.5.6.1: when two SMs side-set the same GPIO on the same cycle, the
// highest-numbered SM wins.
static void sideset_highest_sm_wins(void **state) {
    (void)state;
    for (int sm0_val = 0; sm0_val < 2; sm0_val++) {
        const uint16_t prog[] = {
            SIDE(APIO_NOP, 1, sm0_val),
            SIDE(APIO_NOP, 1, !sm0_val),
        };
        uint32_t pinctrl = APIO_SIDE_SET_COUNT(1) | APIO_SIDE_SET_BASE(13);
        epio_t *epio = sideset_setup(0, 0, pinctrl, prog, 1, GPIO(13));
        epio_set_instr(epio, 0, 1, prog[1]);
        epio_sm_reg_t reg = { .execctrl = WRAP(1, 1), .pinctrl = pinctrl };
        epio_set_sm_reg(epio, 0, 1, &reg);
        PC(0, 1) = 1;
        epio_enable_sm(epio, 0, 1);
        epio_set_gpio_output(epio, 13);

        epio_step_cycles(epio, 1);
        assert_int_equal(!!(epio_read_pin_states(epio) & GPIO(13)), !sm0_val);

        epio_free(epio);
    }
}

// 11.5.7: an instruction run via OUT EXEC performs its own side-set.
static void sideset_on_out_exec(void **state) {
    (void)state;
    const uint16_t prog[] = {
        SIDE(APIO_OUT_EXEC(16), 1, 0),
    };
    epio_t *epio = sideset_setup(
        0,
        APIO_OUT_SHIFTDIR_R | APIO_AUTOPULL | APIO_PULL_THRESH(16),
        APIO_SIDE_SET_COUNT(1) | APIO_SIDE_SET_BASE(14),
        prog, 1, GPIO(14));
    epio_set_gpio_output(epio, 14);
    epio_push_tx_fifo(epio, 0, 0, SIDE(APIO_NOP, 1, 1));

    // Autopull fill stall (11.5.4.2), then the OUT EXEC itself
    for (int ii = 0; ii < 2; ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(epio_read_pin_states(epio) & GPIO(14), 0);
    }
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_read_pin_states(epio) & GPIO(14), GPIO(14));

    epio_free(epio);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(sideset_drives_pin_level),
        cmocka_unit_test(sideset_multiple_bits_with_delay),
        cmocka_unit_test(sideset_opt_enable_bit),
        cmocka_unit_test(sideset_pindirs),
        cmocka_unit_test(sideset_beats_set_on_same_pin),
        cmocka_unit_test(sideset_and_out_same_cycle),
        cmocka_unit_test(sideset_applies_while_stalled),
        cmocka_unit_test(sideset_holds_during_autopull_underrun),
        cmocka_unit_test(sideset_wraps_with_gpiobase16),
        cmocka_unit_test(sideset_ignores_uncontrolled_pins),
        cmocka_unit_test(sideset_highest_sm_wins),
        cmocka_unit_test(sideset_on_out_exec),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
