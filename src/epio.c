// Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
//
// MIT License

// epio - A PIO emulator
//
// Limitations - See also /README.md.
// - No side set pins support (delays _are_ supported)
//   - Note side setting pins takes precedence over OUT/SET for same pins from
//     same SM
//   - For clashes between SMs, highest numbered is preferred (easy to deal
//     with just using incrementing scheduling order).  This is handled
//     separately for direction and levels.
// - MOV instructions for RX FIFO access aren't supported
// - Does not include/support 2 cycle GPIO input delay via flip-flops
// - Only supports 4 word FIFOs
// - Ignores clock dividers

#include <stdlib.h>
#include <string.h>
#include <epio_priv.h>

void epio_set_sm_debug(epio_t *epio, uint8_t block, uint8_t sm, epio_sm_debug_t *debug) {
    CHECK_BLOCK_SM();
    assert(debug != NULL && "Debug information cannot be NULL");
    assert(debug->first_instr <= debug->start_instr && "first_instr must be <= start_instr");
    assert(debug->start_instr <= debug->end_instr && "start_instr must be <= end_instr");
    memcpy(&SM(block, sm).debug, debug, sizeof(epio_sm_debug_t));
}

void epio_get_sm_debug(epio_t *epio, uint8_t block, uint8_t sm, epio_sm_debug_t *debug) {
    CHECK_BLOCK_SM();
    memcpy(debug, &SM(block, sm).debug, sizeof(epio_sm_debug_t));
}

static void epio_init_sm(epio_t *epio, uint8_t block, uint8_t sm) {
    CHECK_BLOCK_SM();

    SM(block, sm).debug.first_instr = 0xFF;
    SM(block, sm).debug.start_instr = 0xFF;
    SM(block, sm).debug.end_instr   = 0xFF;

    SM(block, sm).x         = 0;
    SM(block, sm).y         = 0;
    SM(block, sm).isr       = 0;
    SM(block, sm).osr       = 0;
    SM(block, sm).isr_count = 0;
    SM(block, sm).osr_count = 32;   // Empty OSR indicated by count of 32
    SM(block, sm).pc        = 0;
    SM(block, sm).delay     = 0;
    SM(block, sm).stalled   = 0;
    SM(block, sm).enabled   = 0;
    SM(block, sm).exec_pending = 0;
    SM(block, sm).exec_instr   = 0;
    SM(block, sm).clkdiv_count    = 1;
    SM(block, sm).clkdiv_frac_acc = 0;

    FIFO(block, sm).tx_fifo_count = 0;
    FIFO(block, sm).rx_fifo_count = 0;
}

void epio_set_gpiobase(epio_t *epio, uint8_t block, uint32_t gpio_base) {
    assert(block < NUM_PIO_BLOCKS && "Invalid PIO block");
    assert((gpio_base == 0 || gpio_base == 16) && "GPIO base must be 0 or 16");
    GPIOBASE(block) = gpio_base;
}

uint32_t epio_get_gpiobase(epio_t *epio, uint8_t block) {
    assert(block < NUM_PIO_BLOCKS && "Invalid PIO block");
    uint32_t gpio_base = GPIOBASE(block);
    assert((gpio_base == 0 || gpio_base == 16) && "GPIO base must be 0 or 16");
    return gpio_base;
}

static void epio_init_block(epio_t *epio, uint8_t block) {
    GPIOBASE(block) = 0;

    IRQ(block).irq          = 0;
    IRQ(block).irq_to_clear = 0;
    IRQ(block).irq_to_set   = 0;

    for (int sm = 0; sm < NUM_SMS_PER_BLOCK; sm++) {
        epio_init_sm(epio, block, sm);
    }
}

#ifdef TEST_EPIO
void *(*epio_calloc)(size_t count, size_t size) = calloc;
#endif

epio_t *epio_init(void) {
    epio_t *epio = (epio_t *)epio_calloc(1, sizeof(epio_t));
    if (epio == NULL) {
        return NULL;
    }

    // calloc zeroes everything; EPIO_FLOAT_HIGH = 1 so must be set explicitly
    epio->float_mode = EPIO_FLOAT_HIGH;

    epio_sram_init(epio);
    if (epio->sram == NULL) {
        free(epio);
        return NULL;
    }

    epio_init_gpios(epio);
    epio_init_dma(epio);

    for (int ii = 0; ii < NUM_PIO_BLOCKS; ii++) {
        epio_init_block(epio, ii);
    }

    epio->cycle_count = 0;
    epio->clkdiv_enabled = 0;
    epio->sysclk_hz = EPIO_DEFAULT_SYSCLK_HZ;

    return epio;
}

void epio_free(epio_t *epio) {
    assert(epio != NULL && "Cannot free a NULL epio instance");
    epio_sram_free(epio);
    free(epio);
}

void epio_set_sm_reg(epio_t *epio, uint8_t block, uint8_t sm, epio_sm_reg_t *reg) {
    CHECK_BLOCK_SM();
    assert(reg != NULL && "Register configuration cannot be NULL");
    memcpy(&REG(block, sm), reg, sizeof(epio_sm_reg_t));
}

void epio_get_sm_reg(epio_t *epio, uint8_t block, uint8_t sm, epio_sm_reg_t *reg) {
    CHECK_BLOCK_SM();
    assert(reg != NULL && "Output register pointer cannot be NULL");
    memcpy(reg, &REG(block, sm), sizeof(epio_sm_reg_t));
}

void epio_enable_sm(epio_t *epio, uint8_t block, uint8_t sm) {
    CHECK_BLOCK_SM();
    SM(block, sm).enabled = 1;
}

uint8_t epio_is_sm_enabled(epio_t *epio, uint8_t block, uint8_t sm) {
    CHECK_BLOCK_SM();
    return SM(block, sm).enabled;
}

// CTRL register fields (11.7)
#define CTRL_SM_ENABLE(V)               ((V) & 0xF)
#define CTRL_SM_RESTART(V)              (((V) >> 4) & 0xF)
#define CTRL_CLKDIV_RESTART(V)          (((V) >> 8) & 0xF)
#define CTRL_PREV_PIO_MASK(V)           (((V) >> 16) & 0xF)
#define CTRL_NEXT_PIO_MASK(V)           (((V) >> 20) & 0xF)
#define CTRL_NEXTPREV_SM_ENABLE         (1u << 24)
#define CTRL_NEXTPREV_SM_DISABLE        (1u << 25)
#define CTRL_NEXTPREV_CLKDIV_RESTART    (1u << 26)
#define CTRL_RESERVED                   (0xF8000000u | 0x0000F000u)

// Applies NEXTPREV_SM_ENABLE/DISABLE to a set of SMs in another block
static void epio_ctrl_nextprev_enable(epio_t *epio, uint8_t block, uint8_t sm_mask, uint32_t value) {
    for (int sm = 0; sm < NUM_SMS_PER_BLOCK; sm++) {
        if (sm_mask & (1 << sm)) {
            // If both are set, the disable takes precedence
            if (value & CTRL_NEXTPREV_SM_DISABLE) {
                SM(block, sm).enabled = 0;
            } else if (value & CTRL_NEXTPREV_SM_ENABLE) {
                SM(block, sm).enabled = 1;
            }
        }
    }
}

void epio_write_ctrl(epio_t *epio, uint8_t block, uint32_t value) {
    CHECK_BLOCK();
    assert((value & CTRL_RESERVED) == 0 && "CTRL reserved bits must be 0");
    assert(CTRL_SM_RESTART(value) == 0 && "CTRL SM_RESTART is not supported");

    // Every effect of a single write lands between two cycles, so all the
    // SMs and dividers it touches - in this and neighbouring blocks - start
    // on the same cycle.
    for (int sm = 0; sm < NUM_SMS_PER_BLOCK; sm++) {
        SM(block, sm).enabled = (CTRL_SM_ENABLE(value) >> sm) & 0x1;
    }
    epio_restart_clkdiv(epio, block, CTRL_CLKDIV_RESTART(value));

    // NEXT_PIO_MASK and PREV_PIO_MASK select SMs in the neighbouring blocks,
    // wrapping around from the highest-numbered block to block 0
    uint8_t prev_block = (block + NUM_PIO_BLOCKS - 1) % NUM_PIO_BLOCKS;
    uint8_t next_block = (block + 1) % NUM_PIO_BLOCKS;
    uint8_t prev_mask = CTRL_PREV_PIO_MASK(value);
    uint8_t next_mask = CTRL_NEXT_PIO_MASK(value);

    if (value & CTRL_NEXTPREV_CLKDIV_RESTART) {
        epio_restart_clkdiv(epio, prev_block, prev_mask);
        epio_restart_clkdiv(epio, next_block, next_mask);
    }
    epio_ctrl_nextprev_enable(epio, prev_block, prev_mask, value);
    epio_ctrl_nextprev_enable(epio, next_block, next_mask, value);
}

uint32_t epio_read_ctrl(epio_t *epio, uint8_t block) {
    CHECK_BLOCK();
    // All other fields are self-clearing or reserved, so read as 0
    uint32_t value = 0;
    for (int sm = 0; sm < NUM_SMS_PER_BLOCK; sm++) {
        if (SM(block, sm).enabled) {
            value |= (1u << sm);
        }
    }
    return value;
}

void epio_disable_sm(epio_t *epio, uint8_t block, uint8_t sm) {
    CHECK_BLOCK_SM();
    SM(block, sm).enabled = 0;
}