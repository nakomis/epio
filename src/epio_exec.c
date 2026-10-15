// Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
//
// MIT License

// epio - A PIO emulator
//
// Routines to execute PIO instructions, step SMs, and manage cycle counts

#include <epio.h>
#include <epio_priv.h>
#include <apio_dis.h>

// Forward declare private helper functions
static void epio_sm_step(epio_t *epio, uint8_t block, uint8_t sm);
static void epio_finish_step(epio_t *epio);
static void epio_after_step(epio_t *epio);
static void epio_cycle_callback(epio_t *epio);

// Refills an empty OSR from the TX FIFO, if autopull is enabled
static void epio_autopull(epio_t *epio, uint8_t block, uint8_t sm) {
    if (AUTOPULL_GET(block, sm) &&
        SM(block, sm).osr_count >= PULL_THRESH_GET(block, sm) &&
        epio_tx_fifo_depth(epio, block, sm) > 0) {
        SM(block, sm).osr = epio_pop_tx_fifo(epio, block, sm);
        SM(block, sm).osr_count = 0;
    }
}

// Applies the side-set encoded in an instruction's Delay/side-set field
// (11.5.1).
//
// Called after the instruction's main effect, so that where side-set and an
// OUT/SET (or MOV PINS) from the same SM write the same pin on the same
// cycle, the side-set data is used (11.5.6).
//
// Called every cycle the instruction executes, including when it stalls -
// side-set "is not affected by stalls, and always takes place on the first
// cycle of the attached instruction" (11.2.5).  Re-applying the same value
// on later stalled cycles is indistinguishable from applying it once, unless
// another SM writes the pin during the stall.  Delay cycles do not execute
// the instruction, so do not re-apply side-set.
static void epio_apply_sideset(epio_t *epio, uint8_t block, uint8_t sm, uint16_t instr) {
    uint8_t sideset_count = SIDESET_COUNT_GET(block, sm);
    if (sideset_count == 0) {
        return;
    }

    // Side-set takes the top SIDESET_COUNT bits of the 5 bit field, and
    // SIDESET_COUNT includes the enable bit, if SIDE_EN is set.
    uint8_t sideset_data = ((instr >> 8) & 0x1F) >> (5 - sideset_count);
    uint8_t sideset_pins = sideset_count;
    if (SIDE_EN_GET(block, sm)) {
        sideset_pins--;
        if (!((sideset_data >> sideset_pins) & 0x1)) {
            // Enable bit clear - no side-set on this instruction
            return;
        }
    }

    uint8_t sideset_base = SIDESET_BASE_GET(block, sm);
    uint8_t pindir = SIDE_PINDIR_GET(block, sm);
    for (int ii = 0; ii < sideset_pins; ii++) {
        uint8_t pin = ((sideset_base + ii) % 32) + GPIOBASE(block);
        if (!epio_block_can_control_gpio_output(epio, block, pin)) {
            continue;
        }
        uint8_t bit = (sideset_data >> ii) & 0x1;
        if (pindir) {
            if (bit) {
                epio_set_gpio_output(epio, pin);
            } else {
                epio_set_gpio_input(epio, pin);
            }
        } else {
            epio_set_gpio_output_level(epio, pin, bit);
        }
    }
}

// PINCTRL OUT_COUNT, which the datasheet limits to 32
static uint8_t epio_out_count(epio_t *epio, uint8_t block, uint8_t sm) {
    uint8_t out_count = OUT_COUNT_GET(block, sm);
    assert(out_count <= 32 && "Invalid OUT_COUNT");
    return out_count;
}

// PINCTRL SET_COUNT, which the datasheet limits to 5
static uint8_t epio_set_count(epio_t *epio, uint8_t block, uint8_t sm) {
    uint8_t set_count = SET_COUNT_GET(block, sm);
    assert(set_count <= 5 && "Invalid SET_COUNT");
    return set_count;
}

void epio_set_instr(epio_t *epio, uint8_t block, uint8_t instr_num, uint16_t instr) {
    assert(block < NUM_PIO_BLOCKS && "Invalid PIO block");
    assert(instr_num < NUM_INSTRS_PER_BLOCK && "Instruction number exceeds block capacity");
    INSTR(block, instr_num) = instr;
}

uint16_t epio_get_instr(epio_t *epio, uint8_t block, uint8_t instr_num) {
    assert(block < NUM_PIO_BLOCKS && "Invalid PIO block");
    assert(instr_num < NUM_INSTRS_PER_BLOCK && "Instruction number exceeds block capacity");
    return INSTR(block, instr_num);
}

// Advances an SM's clock divider by one system clock cycle, returning 1 if
// the SM's clock enable is asserted this cycle (11.5.5).
//
// The divider counts down from the divisor n and emits an enable pulse when
// it reaches 1.  For a fractional divisor it keeps a running total of the
// fractional part, and each time that wraps through 1 the next division
// period is n + 1 rather than n.  An integer part of 0 means 65536.
static uint8_t epio_clkdiv_tick(epio_t *epio, uint8_t block, uint8_t sm) {
    if (!epio->clkdiv_enabled) {
        return 1;
    }

    if (SM(block, sm).clkdiv_count > 1) {
        SM(block, sm).clkdiv_count--;
        return 0;
    }

    uint32_t period = CLKDIV_INT_GET(block, sm);
    if (period == 0) {
        period = 65536;
    }
    SM(block, sm).clkdiv_frac_acc += CLKDIV_FRAC_GET(block, sm);
    if (SM(block, sm).clkdiv_frac_acc >= 256) {
        SM(block, sm).clkdiv_frac_acc -= 256;
        period++;
    }
    SM(block, sm).clkdiv_count = period;
    return 1;
}

void epio_set_clkdiv_enabled(epio_t *epio, uint8_t enabled) {
    epio->clkdiv_enabled = enabled ? 1 : 0;
}

uint8_t epio_get_clkdiv_enabled(epio_t *epio) {
    return epio->clkdiv_enabled;
}

void epio_restart_clkdiv(epio_t *epio, uint8_t block, uint8_t sm_mask) {
    assert(block < NUM_PIO_BLOCKS && "Invalid PIO block");
    assert(sm_mask <= 0xF && "Invalid SM mask");
    for (int sm = 0; sm < NUM_SMS_PER_BLOCK; sm++) {
        if (sm_mask & (1 << sm)) {
            // Initial phase of 0 (CTRL CLKDIV_RESTART, 11.7): the next
            // cycle is an enable pulse, as from reset.
            SM(block, sm).clkdiv_count = 1;
            SM(block, sm).clkdiv_frac_acc = 0;
        }
    }
}

void epio_set_sysclk_hz(epio_t *epio, uint32_t hz) {
    assert(hz > 0 && "System clock frequency must be non-zero");
    epio->sysclk_hz = hz;
}

uint32_t epio_get_sysclk_hz(epio_t *epio) {
    return epio->sysclk_hz;
}

// Step all enabled SMs once.
void epio_step_cycles(epio_t *epio, uint32_t cycles) {
    assert(cycles > 0 && "Must step at least one cycle");
    for (uint32_t ii = 0; ii < cycles; ii++) {
        EPIO_DBG("Step...");

        epio_input_sync_sample(epio);

        // !!!
        //
        // It is important that SMs be stepped in ascending order, as in this
        // way any GPIO output setting clashes between SMs will be resolved in
        // the correct way - that is highest numbered SM takes precedence, as
        // per the datasheet.
        //
        // !!!
        for (int block = 0; block < NUM_PIO_BLOCKS; block++) {
            for (int sm = 0; sm < NUM_SMS_PER_BLOCK; sm++) {
                // The divider runs whether or not the SM is enabled
                uint8_t clock_enable = epio_clkdiv_tick(epio, block, sm);
                if (SM(block, sm).enabled && clock_enable) {
                    epio_sm_step(epio, block, sm);
                }
            }
        }
        epio_finish_step(epio);
        epio_input_sync_clock(epio);
        epio_after_step(epio);
        epio_cycle_callback(epio);
        epio->cycle_count++;
    }
}

void epio_set_cycle_callback(epio_t *epio, epio_cycle_callback_t callback, void *ctx) {
    epio->cycle_callback = callback;
    epio->cycle_callback_ctx = ctx;
    epio->cycle_callback_out = epio->gpio.gpio_output_state;
    epio->cycle_callback_oe = epio->gpio.gpio_direction;
}

// Reports the end of a cycle to the host's callback, if any
static void epio_cycle_callback(epio_t *epio) {
    if (epio->cycle_callback == NULL) {
        return;
    }

    uint64_t out = epio->gpio.gpio_output_state;
    uint64_t oe = epio->gpio.gpio_direction;
    uint64_t changed = (out ^ epio->cycle_callback_out) | (oe ^ epio->cycle_callback_oe);
    epio->cycle_callback_out = out;
    epio->cycle_callback_oe = oe;
    epio->cycle_callback(epio, epio->cycle_count, out, oe, changed, epio->cycle_callback_ctx);
}

uint64_t epio_get_cycle_count(epio_t *epio) {
    return epio->cycle_count;
}

void epio_reset_cycle_count(epio_t *epio) {
    epio->cycle_count = 0;
}

// Does any final work after all SMs have executed, like combining GPIO output
// from multiple SMs, deactivating IRQs if they were waited on, etc.
static void epio_finish_step(epio_t *epio) {
    for (int block = 0; block < NUM_PIO_BLOCKS; block++) {
        // https://github.com/raspberrypi/pico-feedback/issues/490 indicates
        // that when a set and clear are applied on the same cycle, the set
        // takes priority, so apply clears first, then sets.
        IRQ(block).irq &= ~IRQ(block).irq_to_clear;
        IRQ(block).irq_to_clear = 0;
        IRQ(block).irq |= IRQ(block).irq_to_set;
        IRQ(block).irq_to_set = 0;
    }
}

// Handles any non-PIO work that needs to be done after each step, like
// DMA chains
static void epio_after_step(epio_t *epio) {
    epio_dma_step(epio);
    epio_dreq_step(epio);
}

static void epio_sm_step(epio_t *epio, uint8_t block, uint8_t sm) {
    assert(SM(block, sm).enabled && "Attempting to step an SM that isn't enabled");

    uint16_t instr;

    // Check whether we have a pending EXEC instruction from previous OUT EXEC
    if (SM(block, sm).exec_pending) {
        // Do not mark EXEC as NOT pending, unless the instruction fully
        // executes with no delay
        instr = SM(block, sm).exec_instr;
    } else {
        instr = CUR_INSTR(block, sm);
    }

    // Execute the instruction
    uint8_t dont_update_pc;
    uint8_t was_exec_pending = SM(block, sm).exec_pending;
    uint8_t out_cycle = 0;
    if (SM(block, sm).delay > 0) {
        SM(block, sm).delay--;
#if defined(EPIO_DEBUG)
        char instr_str[64];
        apio_instruction_decoder(instr, instr_str, 0);
        EPIO_DBG("  PIO%d SM%d PC=%d 0x%04X %-20s Delayed: %d cycles remaining", block, sm, PC(block, sm), instr, instr_str, SM(block, sm).delay);
#endif // EPIO_DEBUG
        dont_update_pc = 1; // PC already points to the next instruction
    } else {
        out_cycle = ((instr >> 13) & 0x7) == OC_OUT;
        dont_update_pc = epio_exec_instr_sm(epio, block, sm, instr);
    }

    // Autopull also refills on cycles without an OUT (11.5.4.2)
    if (!out_cycle) {
        epio_autopull(epio, block, sm);
    }

    // Handle wrap
    if (dont_update_pc) {
        // JMP
    } else {
        if (was_exec_pending) {
            // Now, we can mark the EXEC as no longer pending.  But not if
            // it's just been set
            SM(block, sm).exec_pending = 0;
        }

        uint8_t wrap_top = WRAP_TOP(block, sm);
        uint8_t wrap_bottom = WRAP_BOTTOM(block, sm);

        // Defensive coding would suggest >=, but the datasheet implies it
        // will only wrap after executing the instruction a wrap_top.
        if (PC(block, sm) == wrap_top) {
            PC(block, sm) = wrap_bottom;
        } else {
            PC(block, sm) = (PC(block, sm) + 1) % NUM_INSTRS_PER_BLOCK;
        }
    }
}

// Execute a single instruction for the specified SM, handling any side
// effects and returning whether the PC should be updated or not (e.g. due to 
// a JMP or WAIT).
uint8_t epio_exec_instr_sm(epio_t *epio, uint8_t block, uint8_t sm, uint16_t instr) {
#if defined(EPIO_DEBUG)
    char instr_str[64];
    // Decode using absolute (not offset) addressing
    apio_instruction_decoder(instr, instr_str, 0);
    EPIO_DBG("  PIO%d SM%d PC=%d 0x%04X %-20s X=0x%08X Y=0x%08X ISR=0x%08X OSR=0x%08X RX_FIFO=%d TX_FIFO=%d",
        block, sm, PC(block, sm), instr, instr_str,
        SM(block, sm).x, SM(block, sm).y,
        SM(block, sm).isr, SM(block, sm).osr,
        epio_rx_fifo_depth(epio, block, sm), epio_tx_fifo_depth(epio, block, sm));
#endif // EPIO_DEBUG

    uint8_t dont_update_pc = 0;
    uint8_t process_new_delay = 1;

    // Calculate any delay from this instruction now, so it can be overriden on
    // a per-instruction basis (OUT EXEC, delay is ignored).  Side-set takes the
    // top SIDESET_COUNT bits of the field.
    uint8_t sideset_count = SIDESET_COUNT_GET(block, sm);
    assert(sideset_count <= 5 && "Invalid SIDESET_COUNT");
    uint8_t new_delay = (instr >> 8) & (0x1F >> sideset_count);

    uint16_t opcode = (instr >> 13) & 0x7;
    switch (opcode) {
        case OC_JMP:
            ;
            uint8_t cond = (instr >> 5) & 0x7;
            uint8_t new = instr & 0x1F;
            switch (cond) {
                case JMP_COND_ALWAYS:
                    NEW_INSTR(new);
                    break;

                case JMP_COND_NOT_X:
                    if (SM(block, sm).x == 0) {
                        NEW_INSTR(new);
                    }
                    break;

                case JMP_COND_X_DEC:
                    ;
                    uint32_t x = SM(block, sm).x;
                    SM(block, sm).x--;
                    if (x != 0) {
                        NEW_INSTR(new);
                    }
                    break;

                case JMP_COND_NOT_Y:
                    if (SM(block, sm).y == 0) {
                        NEW_INSTR(new);
                    }
                    break;

                case JMP_COND_Y_DEC:
                    ;
                    uint32_t y = SM(block, sm).y;
                    SM(block, sm).y--;
                    if (y != 0) {
                        NEW_INSTR(new);
                    }
                    break;

                case JMP_COND_X_NOT_Y:
                    if (SM(block, sm).x != SM(block, sm).y) {
                        NEW_INSTR(new);
                    }
                    break;

                case JMP_COND_PIN:
                    ;
                    if (epio_get_jmp_pin_state(epio, block, sm, 0)) {
                        NEW_INSTR(new);
                    }
                    break;

                case JMP_NOT_OSRE:
                    ;
                    uint8_t pull_threshold = PULL_THRESH_GET(block, sm);
                    if (SM(block, sm).osr_count < pull_threshold) {
                        NEW_INSTR(new);
                    }
                    break;
            }
            break;

        case OC_WAIT:
            ;
            uint8_t polarity = (instr >> 7) & 0x1;
            uint8_t source = (instr >> 5) & 0x3;
            uint8_t wait_index = instr & 0x1F;
            uint8_t condition_met = 0;
            
            switch (source) {
                case WAIT_SRC_GPIO:
                    ;
                    uint8_t gpio_state = epio_get_pio_input(epio, block, wait_index + GPIOBASE(block));
                    condition_met = (gpio_state == polarity);
                    break;
                    
                case WAIT_SRC_PIN:
                    ;
                    uint8_t pin_base = IN_BASE_GET(block, sm);
                    uint8_t pin = ((pin_base + wait_index) % 32) + GPIOBASE(block);
                    uint8_t pin_state = (wait_index < IN_COUNT(block, sm)) ? epio_get_pio_input(epio, block, pin) : 0;
                    condition_met = (pin_state == polarity);
                    break;
                    
                case WAIT_SRC_IRQ:
                    ;
                    uint8_t irq_block_bits = (wait_index >> 3) & 0b11;
                    uint8_t irq_block = block;
                    uint8_t irq_index = wait_index & 0b111;
                    switch (irq_block_bits) {
                        case IRQ_BLOCK_THIS:
                            break;

                        case IRQ_BLOCK_PREV:
                            irq_block = (block == 0) ? (NUM_PIO_BLOCKS - 1) : (block - 1);
                            break;

                        case IRQ_BLOCK_REL:
                            irq_index = (irq_index & 0b100) | ((irq_index + sm) & 0b11);
                            break;

                        case IRQ_BLOCK_NEXT:
                            irq_block = (block + 1) % NUM_PIO_BLOCKS;
                            break;
                    }
                    uint8_t irq_state = epio_peek_block_irq_num(epio, irq_block, irq_index);
                    condition_met = (irq_state == polarity);
                    if (condition_met && polarity) {
                        // If we were waiting for an IRQ to be set, we will clear it
                        IRQ(irq_block).irq_to_clear |= (1 << irq_index);
                    }
                    break;

                case WAIT_SRC_JMP_PIN:
                    ;
                    assert(wait_index < 4 && "Reserved WAIT JMPPIN index");
                    uint8_t jmp_pin_state = epio_get_jmp_pin_state(epio, block, sm, wait_index);
                    condition_met = (jmp_pin_state == polarity);
                    break;
            }
            
            if (!condition_met) {
                SM(block, sm).stalled = 1;
                dont_update_pc = 1;
                process_new_delay = 0;
            } else {
                SM(block, sm).stalled = 0;
            }
            break;

        case OC_IN:
            // If we're NOT retrying a stalled autopush, execute the IN
            if (!SM(block, sm).stalled) {
                uint8_t in_source = (instr >> 5) & 0x7;
                uint8_t in_count = instr & 0x1F;
                if (in_count == 0) in_count = 32;
                
                // Get source data
                uint32_t in_data = 0;
                switch (in_source) {
                    case IN_SRC_PINS:
                        ;
                        uint8_t in_base = IN_BASE_GET(block, sm);
                        uint8_t in_pins = IN_COUNT(block, sm);
                        for (int ii = 0; ii < in_count && ii < in_pins; ii++) {
                            uint8_t pin = ((in_base + ii) % 32) + GPIOBASE(block);
                            if (epio_get_pio_input(epio, block, pin)) {
                                in_data |= (1u << ii);
                            }
                        }
                        break;
                        
                    case IN_SRC_X:
                        in_data = SM(block, sm).x;
                        break;
                        
                    case IN_SRC_Y:
                        in_data = SM(block, sm).y;
                        break;
                        
                    case IN_SRC_NULL:
                        in_data = 0;
                        break;
                        
                    case IN_SRC_ISR:
                        in_data = SM(block, sm).isr;
                        break;
                        
                    case IN_SRC_OSR:
                        in_data = SM(block, sm).osr;
                        break;
                        
                    default:
                        assert(0 && "Invalid IN source");
                }
                
                // Shift into ISR
                uint8_t shift_right = IN_SHIFTDIR_R(block, sm);
                uint32_t mask = (in_count == 32) ? 0xFFFFFFFF : ((1U << in_count) - 1);
                if (shift_right) {
                    uint32_t shifted = (in_count == 32) ? 0 : (SM(block, sm).isr >> in_count);
                    SM(block, sm).isr = shifted | (in_data << (32 - in_count));
                } else {
                    uint32_t shifted = (in_count == 32) ? 0 : (SM(block, sm).isr << in_count);
                    SM(block, sm).isr = shifted | (in_data & mask);
                }
                SM(block, sm).isr_count += in_count;
                if (SM(block, sm).isr_count > 32) SM(block, sm).isr_count = 32;
            }

            // Autopush check
            uint8_t autopush = AUTOPUSH_GET(block, sm);
            uint8_t push_threshold = PUSH_THRESH_GET(block, sm);
            if (autopush && SM(block, sm).isr_count >= push_threshold) {
                if (epio_rx_fifo_depth(epio, block, sm) < MAX_FIFO_DEPTH) {
                    epio_push_rx_fifo(epio, block, sm, SM(block, sm).isr);
                    SM(block, sm).isr = 0;
                    SM(block, sm).isr_count = 0;
                    SM(block, sm).stalled = 0;
                } else {
                    // FIFO full - stall
                    SM(block, sm).stalled = 1;
                    dont_update_pc = 1;
                    process_new_delay = 0;
                }
            }
            break;

        case OC_OUT:
            ;
            // An OUT can't refill an empty OSR and shift it out on the same
            // cycle so it stalls whether or not the refill happens (11.5.4.2)
            uint8_t autopull = AUTOPULL_GET(block, sm);
            uint8_t pull_threshold = PULL_THRESH_GET(block, sm);
            if (autopull && SM(block, sm).osr_count >= pull_threshold) {
                epio_autopull(epio, block, sm);
                SM(block, sm).stalled = 1;
                dont_update_pc = 1;
                process_new_delay = 0;
                break;
            }
            SM(block, sm).stalled = 0;

            uint8_t out_dest = (instr >> 5) & 0x7;
            uint8_t out_count = instr & 0x1F;
            if (out_count == 0) out_count = 32;

            // Extract data from OSR
            uint32_t out_data;
            uint8_t out_shift_right = OUT_SHIFTDIR_R(block, sm);
            if (out_shift_right) {
                uint32_t mask = (out_count == 32) ? 0xFFFFFFFF : ((1U << out_count) - 1);
                out_data = SM(block, sm).osr & mask;
                SM(block, sm).osr = (out_count == 32) ? 0 : (SM(block, sm).osr >> out_count);
            } else {
                out_data = SM(block, sm).osr >> (32 - out_count);
                SM(block, sm).osr = (out_count == 32) ? 0 : (SM(block, sm).osr << out_count);
            }
            SM(block, sm).osr_count += out_count;
            if (SM(block, sm).osr_count > 32) {
                SM(block, sm).osr_count = 32;
            }
            
            // Write to destination
            switch (out_dest) {
                case OUT_DEST_PINS:
                    ;
                    uint8_t out_base = OUT_BASE_GET(block, sm);
                    uint8_t out_pins = epio_out_count(epio, block, sm);
                    for (int ii = 0; ii < out_pins; ii++) {
                        uint8_t pin = ((out_base + ii) % 32) + GPIOBASE(block);
                        if (epio_block_can_control_gpio_output(epio, block, pin)) {
                            epio_set_gpio_output_level(epio, pin, (out_data >> ii) & 0x1);
                        }
                    }
                    break;
                    
                case OUT_DEST_X:
                    SM(block, sm).x = out_data;
                    break;
                    
                case OUT_DEST_Y:
                    SM(block, sm).y = out_data;
                    break;
                    
                case OUT_DEST_NULL:
                    break;
                    
                case OUT_DEST_PINDIRS:
                    ;
                    uint8_t pindirs_base = OUT_BASE_GET(block, sm);
                    uint8_t pindirs_pins = epio_out_count(epio, block, sm);
                    for (int ii = 0; ii < pindirs_pins; ii++) {
                        uint8_t pin = ((pindirs_base + ii) % 32) + GPIOBASE(block);
                        if ((out_data >> ii) & 0x1) {
                            if (epio_block_can_control_gpio_output(epio, block, pin)) {
                                epio_set_gpio_output(epio, pin);
                            }
                        } else {
                            if (epio_block_can_control_gpio_output(epio, block, pin)) {
                                epio_set_gpio_input(epio, pin);
                            }
                        }
                    }
                    break;
                    
                case OUT_DEST_PC:
                    PC(block, sm) = out_data & 0x1F; // PC is 5 bits
                    dont_update_pc = 1;
                    break;
                    
                case OUT_DEST_ISR:
                    SM(block, sm).isr = out_data;
                    SM(block, sm).isr_count = out_count;  // Sets ISR shift counter
                    break;
                    
                case OUT_DEST_EXEC:
                    SM(block, sm).exec_instr = out_data & 0xFFFF;
                    SM(block, sm).exec_pending = 1;
                    new_delay = 0; // OUT EXEC ignores delay field in instruction
                    break;
            }

            // The refill can happen alongside the last of the data shifting out
            epio_autopull(epio, block, sm);
            break;

        case OC_PUSH_PULL_MOV:
            ;
            uint8_t bit_3 = (instr >> 3) & 0b1;
            assert(bit_3 == 0 && "MOV to/from RX FIFO not yet implemented");

            uint8_t is_pull = (instr >> 7) & 0b1;
            
            if (is_pull) {
                // PULL
                uint8_t if_empty = (instr >> 6) & 0b1;
                uint8_t block_bit = (instr >> 5) & 0b1;
                uint8_t pull_threshold = PULL_THRESH_GET(block, sm);

                // Check if_empty condition
                uint8_t should_pull = 1;
                if (if_empty) {
                    if (SM(block, sm).osr_count < pull_threshold) {
                        should_pull = 0;
                    }
                }
                
                // If autopull enabled and OSR is full, PULL is a no-op (barrier)
                uint8_t autopull = AUTOPULL_GET(block, sm);
                if (autopull && SM(block, sm).osr_count < pull_threshold) {
                    should_pull = 0;
                }
                
                if (should_pull) {
                    if (epio_tx_fifo_depth(epio, block, sm) > 0) {
                        SM(block, sm).osr = epio_pop_tx_fifo(epio, block, sm);
                        SM(block, sm).osr_count = 0;
                        SM(block, sm).stalled = 0;
                    } else {
                        // TX FIFO empty
                        if (block_bit) {
                            // Stall
                            SM(block, sm).stalled = 1;
                            dont_update_pc = 1;
                            process_new_delay = 0;
                        } else {
                            // Non-blocking: copy X to OSR
                            SM(block, sm).osr = SM(block, sm).x;
                            SM(block, sm).osr_count = 0;
                            SM(block, sm).stalled = 0;
                        }
                    }
                }
            } else {
                // PUSH
                uint8_t if_full = (instr >> 6) & 0b1;
                uint8_t block_bit = (instr >> 5) & 0b1;
                
                // Check if_full condition
                uint8_t should_push = 1;
                if (if_full) {
                    uint8_t push_threshold = PUSH_THRESH_GET(block, sm);
                    if (SM(block, sm).isr_count < push_threshold) {
                        should_push = 0;
                    }
                }
                
                if (should_push) {
                    if (epio_rx_fifo_depth(epio, block, sm) < MAX_FIFO_DEPTH) {
                        epio_push_rx_fifo(epio, block, sm, SM(block, sm).isr);
                        SM(block, sm).isr = 0;
                        SM(block, sm).isr_count = 0;
                        SM(block, sm).stalled = 0;
                    } else {
                        // RX FIFO full
                        if (block_bit) {
                            // Stall
                            SM(block, sm).stalled = 1;
                            dont_update_pc = 1;
                            process_new_delay = 0;
                        } else {
                            // Non-blocking: clear ISR, lose data, set error flag (not implemented)
                            SM(block, sm).isr = 0;
                            SM(block, sm).isr_count = 0;
                        }
                    }
                }
            }
            break;

        case OC_MOV:
            ;
            uint8_t mov_dest = (instr >> 5) & 0b111;
            uint8_t mov_op = (instr >> 3) & 0b11;
            uint8_t mov_src = instr & 0b111;
            
            assert(mov_src != 0b100 && "Reserved MOV source");
            assert(mov_op != 0b11 && "Reserved MOV operation");
            
            // Get source value
            uint32_t mov_value = 0;
            switch (mov_src) {
                case MOV_SRC_PINS:
                    ;
                    uint8_t in_base = IN_BASE_GET(block, sm);
                    uint8_t in_count = IN_COUNT(block, sm);
                    for (int ii = 0; ii < in_count; ii++) {
                        uint8_t pin = ((in_base + ii) % 32) + GPIOBASE(block);
                        if (epio_get_pio_input(epio, block, pin)) {
                            mov_value |= (1u << ii);
                        }
                    }
                    break;
                    
                case MOV_SRC_X:
                    mov_value = SM(block, sm).x;
                    break;
                    
                case MOV_SRC_Y:
                    mov_value = SM(block, sm).y;
                    break;
                    
                case MOV_SRC_NULL:
                    mov_value = 0;
                    break;
                    
                case MOV_SRC_STATUS:
                    ;
                    uint8_t status_sel = STATUS_SEL_GET(block, sm);
                    uint8_t status_n = STATUS_N_GET(block, sm);
                    
                    switch (status_sel) {
                        case 0b00: // TXLEVEL
                            mov_value = (epio_tx_fifo_depth(epio, block, sm) < status_n) ? 0xFFFFFFFF : 0;
                            break;
                            
                        case 0b01: // RXLEVEL
                            mov_value = (epio_rx_fifo_depth(epio, block, sm) < status_n) ? 0xFFFFFFFF : 0;
                            break;
                            
                        case 0b10: // IRQ
                            ;
                            uint8_t irq_block = block;
                            switch ((status_n >> 3) & 0b11) {
                                case STATUS_N_IRQ_THIS:
                                    break;

                                case STATUS_N_IRQ_PREV:
                                    irq_block = (block == 0) ? (NUM_PIO_BLOCKS - 1) : (block - 1);
                                    break;

                                case STATUS_N_IRQ_NEXT:
                                    irq_block = (block + 1) % NUM_PIO_BLOCKS;
                                    break;

                                default:
                                    assert(0 && "Invalid STATUS_N");
                            }
                            uint8_t irq_state = epio_peek_block_irq_num(epio, irq_block, status_n & 0b111);
                            mov_value = irq_state ? 0xFFFFFFFF : 0;
                            break;
                            
                        default:
                            assert(0 && "Invalid STATUS_SEL");
                    }
                    break;                    
                case MOV_SRC_ISR:
                    mov_value = SM(block, sm).isr;
                    break;
                    
                case MOV_SRC_OSR:
                    mov_value = SM(block, sm).osr;
                    break;
            }
            
            // Apply operation
            switch (mov_op) {
                case MOV_OP_NONE:
                    break;
                    
                case MOV_OP_INVERT:
                    mov_value = ~mov_value;
                    break;
                    
                case MOV_OP_BITREV:
                    ;
                    uint32_t reversed = 0;
                    for (int ii = 0; ii < 32; ii++) {
                        if (mov_value & (1 << ii)) {
                            reversed |= (1 << (31 - ii));
                        }
                    }
                    mov_value = reversed;
                    break;
            }
            
            // Write to destination
            switch (mov_dest) {
                case MOV_DEST_PINS:
                    ;
                    uint8_t out_base = OUT_BASE_GET(block, sm);
                    uint8_t out_count = epio_out_count(epio, block, sm);
                    for (int ii = 0; ii < out_count; ii++) {
                        uint8_t pin = ((out_base + ii) % 32) + GPIOBASE(block);
                        if (epio_block_can_control_gpio_output(epio, block, pin)) {
                            epio_set_gpio_output_level(epio, pin, (mov_value >> ii) & 0b1);
                        }
                    }
                    break;
                    
                case MOV_DEST_X:
                    SM(block, sm).x = mov_value;
                    break;
                    
                case MOV_DEST_Y:
                    SM(block, sm).y = mov_value;
                    break;
                    
                case MOV_DEST_PINDIRS:
                    ;
                    uint8_t pindirs_base = OUT_BASE_GET(block, sm);
                    uint8_t pindirs_count = epio_out_count(epio, block, sm);
                    for (int ii = 0; ii < pindirs_count; ii++) {
                        uint8_t pin = ((pindirs_base + ii) % 32) + GPIOBASE(block);
                        if ((mov_value >> ii) & 0b1) {
                            if (epio_block_can_control_gpio_output(epio, block, pin)) {
                                epio_set_gpio_output(epio, pin);
                            }
                        } else {
                            if (epio_block_can_control_gpio_output(epio, block, pin)) {
                                epio_set_gpio_input(epio, pin);
                            }
                        }
                    }
                    break;
                    
                case MOV_DEST_EXEC:
                    SM(block, sm).exec_instr = mov_value & 0xFFFF;
                    SM(block, sm).exec_pending = 1;
                    new_delay = 0; // MOV EXEC ignores delay field in instruction
                    break;
                    
                case MOV_DEST_PC:
                    PC(block, sm) = mov_value & 0x1F; // PC is 5 bits
                    dont_update_pc = 1;
                    break;
                    
                case MOV_DEST_ISR:
                    SM(block, sm).isr = mov_value;
                    SM(block, sm).isr_count = 0;
                    break;
                    
                case MOV_DEST_OSR:
                    SM(block, sm).osr = mov_value;
                    SM(block, sm).osr_count = 0;
                    break;
            }
            break;

        case OC_IRQ:
            ;
            uint8_t clr = (instr >> 6) & 0b1;
            uint8_t wait = (instr >> 5) & 0b1;
            uint8_t idx_mode = (instr >> 3) & 0b11;
            uint8_t index = instr & 0b111;

            // Determine which block and index
            uint8_t irq_block, irq_index;
            HANDLE_IRQ_MODE(block, sm, idx_mode, index, irq_block, irq_index);
            
            if (clr) {
                IRQ(irq_block).irq_to_clear |= (1 << irq_index);
            } else {
                if (wait) {
                    if (SM(block, sm).stalled) {
                        // Re-execution: check if cleared
                        uint8_t irq_state = epio_peek_block_irq_num(epio, irq_block, irq_index);
                        if (!irq_state) {
                            SM(block, sm).stalled = 0;
                        } else {
                            dont_update_pc = 1;
                            process_new_delay = 0;
                        }
                    } else {
                        // First execution: set and stall - only set the first
                        // time through this instruction.
                        IRQ(irq_block).irq_to_set |= (1 << irq_index);
                        SM(block, sm).stalled = 1;
                        dont_update_pc = 1;
                        process_new_delay = 0;
                    }
                } else {
                    // Not stalling, so set the IRQ
                    IRQ(irq_block).irq_to_set |= (1 << irq_index);
                }
            }
            break;

        case OC_SET:
            ;
            uint8_t set_dest = (instr >> 5) & 0b111;
            uint8_t set_data = instr & 0x1F;
            
            switch (set_dest) {
                case SET_DEST_PINS:
                    ;
                    uint8_t set_base = SET_BASE_GET(block, sm);
                    uint8_t set_count = epio_set_count(epio, block, sm);
                    for (int ii = 0; ii < set_count; ii++) {
                        uint8_t pin = ((set_base + ii) % 32) + GPIOBASE(block);
                        if (epio_block_can_control_gpio_output(epio, block, pin)) {
                            epio_set_gpio_output_level(epio, pin, (set_data >> ii) & 0b1);
                        }
                    }
                    break;
                    
                case SET_DEST_X:
                    SM(block, sm).x = set_data;
                    break;
                    
                case SET_DEST_Y:
                    SM(block, sm).y = set_data;
                    break;
                    
                case SET_DEST_PIN_DIRS:
                    ;
                    uint8_t pindirs_base = SET_BASE_GET(block, sm);
                    uint8_t pindirs_count = epio_set_count(epio, block, sm);
                    for (int ii = 0; ii < pindirs_count; ii++) {
                        uint8_t pin = ((pindirs_base + ii) % 32) + GPIOBASE(block);
                        if ((set_data >> ii) & 0b1) {
                            if (epio_block_can_control_gpio_output(epio, block, pin)) {
                                epio_set_gpio_output(epio, pin);
                            }
                        } else {
                            if (epio_block_can_control_gpio_output(epio, block, pin)) {
                                epio_set_gpio_input(epio, pin);
                            }
                        }
                    }
                    break;
                    
                default:
                    assert(0 && "Invalid SET destination");
            }
            break;
    }

    epio_apply_sideset(epio, block, sm, instr);

    if (process_new_delay) {
        SM(block, sm).delay = new_delay;
    }

    EPIO_DBG("                                            X=0x%08X Y=0x%08X ISR=0x%08X OSR=0x%08X RX_FIFO=%d TX_FIFO=%d",
        SM(block, sm).x, SM(block, sm).y,
        SM(block, sm).isr, SM(block, sm).osr,
        epio_rx_fifo_depth(epio, block, sm), epio_tx_fifo_depth(epio, block, sm));

    return dont_update_pc;
}
