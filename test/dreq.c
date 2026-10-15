// Copyright (C) 2026 Martin Harris <epio@nakomis.com>
//
// MIT License

// epio - A PIO emulator
//
// Unit tests for the DREQ callbacks, which model DMA feeding TX FIFOs and
// draining RX FIFOs

#define APIO_LOG_IMPL
#include "test.h"

#define WRAP(TOP, BOTTOM)   (((TOP) << 12) | ((BOTTOM) << 7))
#define MAX_WORDS           16

typedef struct {
    // TX: words to supply, in order
    uint32_t tx_words[MAX_WORDS];
    int tx_len;
    int tx_next;
    int tx_calls;
    // RX: words received, in order
    uint32_t rx_words[MAX_WORDS];
    int rx_len;
    // Block and SM of the last call
    uint8_t block;
    uint8_t sm;
    epio_t *epio;
} dma_t;

static uint8_t tx_feed(epio_t *epio, uint8_t block, uint8_t sm, uint32_t *word, void *ctx) {
    dma_t *dma = (dma_t *)ctx;
    dma->tx_calls++;
    dma->epio = epio;
    dma->block = block;
    dma->sm = sm;
    if (dma->tx_next >= dma->tx_len) {
        return 0;
    }
    *word = dma->tx_words[dma->tx_next++];
    return 1;
}

static void rx_drain(epio_t *epio, uint8_t block, uint8_t sm, uint32_t word, void *ctx) {
    dma_t *dma = (dma_t *)ctx;
    assert_true(dma->rx_len < MAX_WORDS);
    dma->epio = epio;
    dma->block = block;
    dma->sm = sm;
    dma->rx_words[dma->rx_len++] = word;
}

// TX DREQ is asserted while the TX FIFO is not full, so the feeder tops it
// up one word per cycle and is not called once it is full.  DREQ does not
// depend on the SM being enabled.
static void dreq_tx_fills_fifo(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    dma_t dma = { .tx_words = {1, 2, 3, 4, 5, 6}, .tx_len = 6 };
    epio_set_dreq_callbacks(epio, 2, 3, tx_feed, NULL, &dma);

    for (int ii = 1; ii <= MAX_FIFO_DEPTH; ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(epio_tx_fifo_depth(epio, 2, 3), ii);
        assert_int_equal(dma.tx_calls, ii);
    }
    epio_step_cycles(epio, 3);
    assert_int_equal(dma.tx_calls, MAX_FIFO_DEPTH);
    assert_ptr_equal(dma.epio, epio);
    assert_int_equal(dma.block, 2);
    assert_int_equal(dma.sm, 3);

    for (uint32_t ii = 1; ii <= MAX_FIFO_DEPTH; ii++) {
        assert_int_equal(epio_pop_tx_fifo(epio, 2, 3), ii);
    }

    epio_free(epio);
}

// A feeder with no data (a DMA underrun) pushes nothing.
static void dreq_tx_underrun(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    dma_t dma = { .tx_len = 0 };
    epio_set_dreq_callbacks(epio, 0, 0, tx_feed, NULL, &dma);

    epio_step_cycles(epio, 3);
    assert_int_equal(dma.tx_calls, 3);
    assert_int_equal(epio_tx_fifo_depth(epio, 0, 0), 0);

    epio_free(epio);
}

// A word supplied at the end of cycle t is available to the SM in cycle
// t + 1: a blocking PULL stalls on cycle 0 and completes on cycle 1.
static void dreq_tx_word_next_cycle(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    epio_set_instr(epio, 0, 0, APIO_PULL_BLOCK);
    epio_set_instr(epio, 0, 1, APIO_MOV_X_OSR);
    epio_sm_reg_t reg = { .execctrl = WRAP(1, 1) };
    epio_set_sm_reg(epio, 0, 0, &reg);
    epio_enable_sm(epio, 0, 0);
    dma_t dma = { .tx_words = {0xCAFEF00D}, .tx_len = 1 };
    epio_set_dreq_callbacks(epio, 0, 0, tx_feed, NULL, &dma);

    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 0);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_pc(epio, 0, 0), 1);
    epio_step_cycles(epio, 1);
    assert_int_equal(epio_peek_sm_x(epio, 0, 0), 0xCAFEF00D);

    epio_free(epio);
}

// RX DREQ is asserted while the RX FIFO is not empty, so the drain receives
// one word per cycle, in order.
static void dreq_rx_drains_fifo(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    for (uint32_t ii = 10; ii < 13; ii++) {
        epio_push_rx_fifo(epio, 1, 2, ii);
    }
    dma_t dma = { .tx_len = 0 };
    epio_set_dreq_callbacks(epio, 1, 2, NULL, rx_drain, &dma);

    for (int ii = 1; ii <= 3; ii++) {
        epio_step_cycles(epio, 1);
        assert_int_equal(dma.rx_len, ii);
        assert_int_equal(epio_rx_fifo_depth(epio, 1, 2), 3 - ii);
    }
    epio_step_cycles(epio, 2);
    assert_int_equal(dma.rx_len, 3);
    assert_int_equal(dma.rx_words[0], 10);
    assert_int_equal(dma.rx_words[1], 11);
    assert_int_equal(dma.rx_words[2], 12);
    assert_ptr_equal(dma.epio, epio);
    assert_int_equal(dma.block, 1);
    assert_int_equal(dma.sm, 2);

    epio_free(epio);
}

// An SM fed and drained by DMA: each word through TX comes back out of RX.
static void dreq_tx_and_rx_together(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    // PULL BLOCK; MOV ISR, OSR; PUSH BLOCK
    epio_set_instr(epio, 0, 0, APIO_PULL_BLOCK);
    epio_set_instr(epio, 0, 1, APIO_MOV_ISR_OSR);
    epio_set_instr(epio, 0, 2, APIO_PUSH_BLOCK);
    epio_sm_reg_t reg = { .execctrl = WRAP(2, 0) };
    epio_set_sm_reg(epio, 0, 0, &reg);
    epio_enable_sm(epio, 0, 0);
    dma_t dma = { .tx_words = {7, 8, 9}, .tx_len = 3 };
    epio_set_dreq_callbacks(epio, 0, 0, tx_feed, rx_drain, &dma);

    epio_step_cycles(epio, 12);
    assert_int_equal(dma.rx_len, 3);
    assert_int_equal(dma.rx_words[0], 7);
    assert_int_equal(dma.rx_words[1], 8);
    assert_int_equal(dma.rx_words[2], 9);

    epio_free(epio);
}

// NULL callbacks remove them.
static void dreq_removed(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);
    dma_t dma = { .tx_words = {1, 2}, .tx_len = 2 };
    epio_set_dreq_callbacks(epio, 0, 0, tx_feed, rx_drain, &dma);
    epio_step_cycles(epio, 1);
    epio_set_dreq_callbacks(epio, 0, 0, NULL, NULL, NULL);
    epio_push_rx_fifo(epio, 0, 0, 3);
    epio_step_cycles(epio, 2);
    assert_int_equal(dma.tx_calls, 1);
    assert_int_equal(dma.rx_len, 0);
    assert_int_equal(epio_rx_fifo_depth(epio, 0, 0), 1);

    epio_free(epio);
}

static void dreq_invalid(void **state) {
    (void)state;
    epio_t *epio = epio_init();
    assert_non_null(epio);

    expect_assert_failure(epio_set_dreq_callbacks(epio, NUM_PIO_BLOCKS, 0, NULL, NULL, NULL));
    expect_assert_failure(epio_set_dreq_callbacks(epio, 0, NUM_SMS_PER_BLOCK, NULL, NULL, NULL));

    epio_free(epio);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(dreq_tx_fills_fifo),
        cmocka_unit_test(dreq_tx_underrun),
        cmocka_unit_test(dreq_tx_word_next_cycle),
        cmocka_unit_test(dreq_rx_drains_fifo),
        cmocka_unit_test(dreq_tx_and_rx_together),
        cmocka_unit_test(dreq_removed),
        cmocka_unit_test(dreq_invalid),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
