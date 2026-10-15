// Copyright (C) 2026 Martin Harris <epio@nakomis.com>
//
// MIT License

// epio - A PIO emulator
//
// Loading programs assembled by pioasm, and pico-sdk style SM initialisation

#include <epio_priv.h>
#include <epio_pioasm.h>

uint epio_add_program(epio_t *epio, uint8_t block, const pio_program_t *program, uint8_t offset) {
    CHECK_BLOCK();
    assert(program != NULL && "Program cannot be NULL");
    assert(offset + program->length <= NUM_INSTRS_PER_BLOCK && "Program does not fit at this offset");
    assert((program->origin < 0 || program->origin == offset) && "Program must be loaded at its .origin");

    for (int ii = 0; ii < program->length; ii++) {
        uint16_t instr = program->instructions[ii];
        uint16_t opcode = (instr >> 13) & 0x7;
        if (opcode == OC_WAIT && ((instr >> 5) & 0b11) == WAIT_SRC_GPIO) {
            // WAIT GPIO encodes the low 5 bits of the GPIO number.  With a
            // GPIOBASE of 16, GPIOs 16-47 are 0-31 in the block's window,
            // which flipping bit 4 achieves for both halves (as the
            // pico-sdk's add_program_at_offset() does).
            instr ^= (uint16_t)GPIOBASE(block);
        } else if (opcode == OC_JMP) {
            // pioasm assembles JMP targets for offset 0
            instr += offset;
        }
        INSTR(block, offset + ii) = instr;
    }
    return offset;
}

void epio_sm_init(epio_t *epio, uint8_t block, uint8_t sm, uint8_t initial_pc, const pio_sm_config *config) {
    CHECK_BLOCK_SM();
    assert(initial_pc < NUM_INSTRS_PER_BLOCK && "Invalid initial PC");

    pio_sm_config c = config ? *config : pio_get_default_sm_config();

    // pio_sm_set_config() with PICO_PIO_USE_GPIO_BASE: the config holds
    // absolute pins (pin & 31, plus its 16-pin range in pinhi). Each pin
    // field in use must fall in the block's 32-pin window, and with GPIOBASE
    // 16 its bit 4 flips (16-31 -> 0-15, 32-47 -> 16-31).
    const uint32_t lsbs = (1u << EPIO_PINHI_EXECCTRL_LSB) |
                          (1u << EPIO_PINCTRL_IN_BASE_LSB) | (1u << EPIO_PINCTRL_OUT_BASE_LSB) |
                          (1u << EPIO_PINCTRL_SET_BASE_LSB) | (1u << EPIO_PINCTRL_SIDESET_BASE_LSB);
    uint32_t used = (~c.pinhi >> 4) & lsbs;
    uint32_t under_16 = (~c.pinhi) & (~c.pinhi >> 1) & used;
    uint32_t over_32 = (c.pinhi >> 1) & used;
    uint32_t gpio_base = GPIOBASE(block);
    assert(!(under_16 && gpio_base) && "Config uses GPIO 0-15, outside a GPIOBASE 16 block's window");
    assert(!(over_32 && !gpio_base) && "Config uses GPIO 32-47, outside a GPIOBASE 0 block's window");
    if (gpio_base) {
        c.execctrl ^= (used >> EPIO_PINHI_EXECCTRL_LSB) << (EPIO_EXECCTRL_JMP_PIN_LSB + 4);
        c.pinctrl ^= (used & ~(1u << EPIO_PINHI_EXECCTRL_LSB)) << 4;
    }

    epio_disable_sm(epio, block, sm);
    epio_sm_reg_t reg = {
        .clkdiv = c.clkdiv,
        .execctrl = c.execctrl,
        .shiftctrl = c.shiftctrl,
        .pinctrl = c.pinctrl,
    };
    epio_set_sm_reg(epio, block, sm, &reg);

    // pio_sm_clear_fifos()
    FIFO(block, sm).tx_fifo_count = 0;
    FIFO(block, sm).rx_fifo_count = 0;

    // pio_sm_restart(): clear the shift registers' state, delay and any
    // stalled or pending EXEC instruction.  The OSR is left empty, as after
    // epio_init().
    SM(block, sm).isr = 0;
    SM(block, sm).isr_count = 0;
    SM(block, sm).osr_count = 32;
    SM(block, sm).delay = 0;
    SM(block, sm).stalled = 0;
    SM(block, sm).exec_pending = 0;

    // pio_sm_exec(pio, sm, pio_encode_jmp(initial_pc))
    PC(block, sm) = initial_pc;
}
