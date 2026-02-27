/**
 * pio_qspi.h — PIO-based QSPI driver for CO5300 display
 *
 * Uses PIO2 to bit-bang SPI/QSPI at up to 75 MHz.
 * Two transfer modes:
 *   1-bit (SPI):  commands and register writes — blocking.
 *   4-bit (QSPI): pixel data — DMA with ISR completion callback.
 */

#ifndef PIO_QSPI_H
#define PIO_QSPI_H

#include "pico/stdlib.h"
#include "bsp_dma_channel_irq.h"

#define QSPI_PIO    pio2

/**
 * Initialise PIO state machine, DMA channel, and optional completion IRQ.
 *
 * @param sclk_pin  GPIO number for the SCLK output.
 * @param d0_pin    GPIO number for data line D0 (D1-D3 follow consecutively).
 * @param baudrate  Target bit clock in Hz (PIO divider is derived from it).
 * @param irq_cb    DMA-complete callback (may be NULL if not needed).
 */
void pio_qspi_init(uint sclk_pin, uint d0_pin, uint32_t baudrate,
                    channel_irq_callback_t irq_cb);

/** Expand a single byte to 1-bit-over-4-lanes format and send (blocking). */
void pio_qspi_1bit_write_blocking(uint8_t byte);

/** Send an array of bytes in 1-bit SPI mode (blocking, waits for idle). */
void pio_qspi_1bit_write_data_blocking(uint8_t *buf, size_t len);

/** Send an array of bytes in native 4-bit QSPI mode (blocking). */
void pio_qspi_4bit_write_data_blocking(uint8_t *buf, size_t len);

/** Send bytes in 1-bit mode via DMA (non-blocking — uses VLA on stack). */
void pio_qspi_1bit_write_data(uint8_t *buf, size_t len);

/** Send bytes in 4-bit QSPI mode via DMA (non-blocking). */
void pio_qspi_4bit_write_data(uint8_t *buf, size_t len);

/** Busy-wait until the PIO TX FIFO is fully drained and the last byte
 *  has been shifted out on the bus. */
void pio_qspi_wait_idle(void);

#endif /* PIO_QSPI_H */
