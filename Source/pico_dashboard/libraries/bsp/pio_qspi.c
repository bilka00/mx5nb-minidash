/**
 * pio_qspi.c — PIO-based QSPI driver for CO5300 OLED display
 *
 * The CO5300 accepts commands in 1-bit SPI mode (prefix 0x02) and bulk
 * pixel data in 4-bit QSPI mode (prefix 0x32).  This driver implements
 * both paths using a single PIO2 state machine:
 *
 *   1-bit path: each source byte is expanded to 4 FIFO bytes (one
 *               data bit on D0, other lanes idle).  Used for register
 *               writes — always blocking.
 *
 *   4-bit path: each source byte maps 1:1 to one FIFO byte (4 bits
 *               per clock on D0-D3).  Pixel flushes use DMA with an
 *               ISR callback to signal completion.
 *
 * The PIO program shifts out 8 bits per FIFO entry with auto-pull,
 * TX FIFO joined (8-deep) for maximum throughput.
 */

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/platform.h"
#include "pico/stdlib.h"

#include "pio_qspi.h"
#include "pio_qspi.pio.h"

/* ---- State ---- */

static uint pio_qspi_sm;
static int  pio_qspi_dma_chan;

/* ---- Internal helpers ---- */

/**
 * Configure and start the PIO state machine for QSPI output.
 *
 * SCLK is driven via side-set; D0-D3 are mapped as OUT pins.
 * TX FIFO is joined (8-entry depth) with 8-bit auto-pull, MSB first.
 */
static inline void qspi_program_init(PIO pio, uint sm, uint offset,
                                      uint sclk_pin, uint d0_pin, float div)
{
    pio_sm_config c = qspi_program_get_default_config(offset);

    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_sideset_pins(&c, sclk_pin);
    sm_config_set_clkdiv(&c, div);

    /* Connect GPIO pads to PIO */
    pio_gpio_init(pio, sclk_pin);
    for (uint i = 0; i < 4; i++)
        pio_gpio_init(pio, d0_pin + i);

    /* Internal pull-ups keep lines defined when idle */
    gpio_pull_up(sclk_pin);
    for (uint i = 0; i < 4; i++)
        gpio_pull_up(d0_pin + i);

    /* Pin directions: SCLK + D0-D3 all outputs */
    pio_sm_set_consecutive_pindirs(pio, sm, sclk_pin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, d0_pin,   4, true);

    sm_config_set_out_pins(&c, d0_pin, 4);
    sm_config_set_out_shift(&c, false, true, 8);  /* MSB first, auto-pull, 8-bit */

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

/** Claim a DMA channel and configure it for PIO TX FIFO writes (8-bit). */
static void pio_qspi_dma_init(void)
{
    pio_qspi_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config cfg = dma_channel_get_default_config(pio_qspi_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(QSPI_PIO, pio_qspi_sm, true));

    dma_channel_configure(
        pio_qspi_dma_chan, &cfg,
        &QSPI_PIO->txf[pio_qspi_sm],   /* write address: PIO TX FIFO */
        NULL,                            /* read address: set per transfer */
        0,                               /* count: set per transfer */
        false);                          /* don't start yet */
}

/* ---- 1-bit SPI expansion ----
 *
 * The PIO program always outputs 4 data bits per clock cycle (QSPI mode).
 * To send 1-bit SPI, each source byte is exploded into 4 FIFO bytes:
 * bits [1:0] → byte 0, bits [3:2] → byte 1, etc., with each source bit
 * placed on D0 and the other three lanes held low.
 */

/** Expand a single source byte into 4 FIFO-ready bytes in `out[0..3]`. */
static inline void expand_byte_1bit(uint8_t src, uint8_t out[4])
{
    for (int i = 0; i < 4; i++) {
        uint8_t bit_lo = (src >> (2 * i))     & 1;
        uint8_t bit_hi = (src >> (2 * i + 1)) & 1;
        out[3 - i] = bit_lo | (bit_hi << 4);
    }
}

/* ---- Public API ---- */

void pio_qspi_init(uint sclk_pin, uint d0_pin, uint32_t baudrate,
                    channel_irq_callback_t irq_cb)
{
    float div = (float)clock_get_hz(clk_sys) / (float)baudrate / 2.0f;
    if (div < 1.0f)
        div = 1.0f;

    pio_qspi_sm = pio_claim_unused_sm(QSPI_PIO, true);
    uint offset = pio_add_program(QSPI_PIO, &qspi_program);
    qspi_program_init(QSPI_PIO, pio_qspi_sm, offset, sclk_pin, d0_pin, div);

    pio_qspi_dma_init();

    if (irq_cb != NULL)
        bsp_dma_channel_irq_add(1, pio_qspi_dma_chan, irq_cb);
}

void pio_qspi_wait_idle(void)
{
    while (!pio_sm_is_tx_fifo_empty(QSPI_PIO, pio_qspi_sm))
        tight_loop_contents();
    /* After FIFO empties the shift register may still hold the last byte.
     * At div ~1.6 that's ~4 PIO cycles ≈ 27 ns.  8 NOPs at 240 MHz ≈ 33 ns. */
    __asm__ volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
}

void pio_qspi_1bit_write_blocking(uint8_t byte)
{
    uint8_t expanded[4];
    expand_byte_1bit(byte, expanded);

    for (int i = 0; i < 4; i++) {
        while (pio_sm_is_tx_fifo_full(QSPI_PIO, pio_qspi_sm))
            tight_loop_contents();
        *(volatile uint8_t *)&QSPI_PIO->txf[pio_qspi_sm] = expanded[i];
    }
}

void pio_qspi_1bit_write_data_blocking(uint8_t *buf, size_t len)
{
    uint8_t expanded[4 * len];   /* VLA — OK for short command sequences */

    for (size_t j = 0; j < len; j++)
        expand_byte_1bit(buf[j], &expanded[j * 4]);

    for (size_t i = 0; i < 4 * len; i++) {
        while (pio_sm_is_tx_fifo_full(QSPI_PIO, pio_qspi_sm))
            tight_loop_contents();
        *(volatile uint8_t *)&QSPI_PIO->txf[pio_qspi_sm] = expanded[i];
    }

    pio_qspi_wait_idle();
}

void pio_qspi_4bit_write_data_blocking(uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        while (pio_sm_is_tx_fifo_full(QSPI_PIO, pio_qspi_sm))
            tight_loop_contents();
        *(volatile uint8_t *)&QSPI_PIO->txf[pio_qspi_sm] = buf[i];
    }
    pio_qspi_wait_idle();
}

void pio_qspi_1bit_write_data(uint8_t *buf, size_t len)
{
    uint8_t expanded[4 * len];   /* VLA on stack — keep len small */

    for (size_t j = 0; j < len; j++)
        expand_byte_1bit(buf[j], &expanded[j * 4]);

    dma_channel_set_read_addr(pio_qspi_dma_chan, expanded, false);
    dma_channel_set_trans_count(pio_qspi_dma_chan, 4 * len, true);
}

void pio_qspi_4bit_write_data(uint8_t *buf, size_t len)
{
    /* Drain the M33 store buffer so DMA sees the latest SRAM contents */
    __asm volatile("dsb sy" ::: "memory");

    dma_channel_set_read_addr(pio_qspi_dma_chan, buf, false);
    dma_channel_set_trans_count(pio_qspi_dma_chan, len, true);
}
