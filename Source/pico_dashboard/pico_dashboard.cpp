/**
 * pico_dashboard — main entry point
 *
 * Core 0: LVGL rendering, display flush (PIO2 QSPI DMA), touch input.
 * Core 1: CAN bus reception + protocol parsing (PIO0, ME442 mode only).
 *
 * ECU data flows:  core 1 → invent_ems_data_t → core 0 LVGL timer → UI.
 * All LVGL widget updates happen inside lv_timer_handler() to respect
 * LVGL's single-threaded dirty-area tracking.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/structs/sio.h"

#include "config.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "bsp_i2c.h"
#include "ui/ui_dashboard.h"
#include "ui/ui_debug_console.h"

extern "C" {
#include "bsp_serial.h"
#include "bsp_can.h"
#include "protocol/invent_ems.h"
}

#if ECU_PROTOCOL == ECU_ME442
#include "pico/multicore.h"
#endif

/* ---- Clock configuration ---- */

static void set_cpu_clock(uint32_t freq_mhz)
{
    set_sys_clock_hz(freq_mhz * MHZ, true);
    /* Re-derive peripheral clock from the new PLL frequency so that
     * UART / SPI baud rates remain correct after the speed change. */
    clock_configure(clk_peri, 0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    freq_mhz * MHZ, freq_mhz * MHZ);
}

/* ---- LVGL tick (1 ms hardware timer ISR) ---- */

static bool repeating_lvgl_timer_cb(struct repeating_timer *t)
{
    (void)t;
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
    return true;
}

/* ======================================================================
 * ECU_INVENT_EMS — UART path (core 0 only)
 * ====================================================================== */
#if ECU_PROTOCOL == ECU_INVENT_EMS

/*
 * UART0 RX interrupt ring buffer.
 * At 19200 baud packets arrive every ~22 ms, but lv_timer_handler() can
 * block for 20-50 ms during rendering.  The hardware FIFO is only 32 B
 * (~16.7 ms at 19200), so without an ISR we lose bytes and get CRC errors.
 */
#define UART_RX_BUF_SIZE 256
static volatile uint8_t  uart_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t uart_rx_head = 0;
static uint16_t          uart_rx_tail = 0;

extern "C" void uart0_irq_handler(void)
{
    while (uart_is_readable(uart0)) {
        uint8_t ch = (uint8_t)uart_getc(uart0);
        uint16_t next = (uart_rx_head + 1) % UART_RX_BUF_SIZE;
        if (next != uart_rx_tail) {           /* drop on overflow */
            uart_rx_buf[uart_rx_head] = ch;
            uart_rx_head = next;
        }
    }
}

#endif /* ECU_INVENT_EMS */

/* ======================================================================
 * ECU_ME442 — CAN path (core 1 drains CAN + parses, core 0 just reads)
 * ====================================================================== */
#if ECU_PROTOCOL == ECU_ME442

static void core1_entry(void)
{
    bsp_can_init();     /* PIO0 IRQ is registered on core 1 NVIC */

    bsp_can_frame_t frame;
    while (true) {
        while (bsp_can_recv(&frame))
            invent_ems_feed_can_frame(frame.id, frame.data, frame.dlc);
        tight_loop_contents();
    }
}

#endif /* ECU_ME442 */

/* ======================================================================
 * LVGL timer callbacks (run inside lv_timer_handler on core 0)
 * ====================================================================== */

/* Flag set by main loop when the protocol parser signals new data.
 * The LVGL timer reads it so that widget updates stay inside the
 * lv_timer_handler() context (required for correct dirty-area tracking). */
static volatile bool ecu_data_ready = false;

static void dashboard_update_cb(lv_timer_t *timer)
{
    (void)timer;
    if (!ecu_data_ready) return;
    ecu_data_ready = false;

    const invent_ems_data_t *ecu = invent_ems_get_data();
    ui_dashboard_set_oil_pressure(ecu->oil_pressure);
    ui_dashboard_set_coolant_temp(ecu->clt);
    ui_dashboard_set_oil_temp(ecu->oil_temp);
}

#if ENABLE_DEBUG_CONSOLE
static void debug_stats_cb(lv_timer_t *timer)
{
    (void)timer;
    const invent_ems_data_t *ecu = invent_ems_get_data();

    bsp_can_stats_t can = {0};
#if ECU_PROTOCOL == ECU_ME442
    can = bsp_can_get_stats();
    can.rx_pin_raw = (sio_hw->gpio_in & (1u << BSP_CAN_GPIO_RX)) ? 1 : 0;
#endif

    ui_debug_console_update_stats(
        ecu->packet_count, ecu->error_count, ecu->connected, &can);
}
#endif /* ENABLE_DEBUG_CONSOLE */

/* ======================================================================
 * main
 * ====================================================================== */

int main()
{
    stdio_init_all();
    set_cpu_clock(CPU_CLOCK_MHZ);
    bsp_i2c_init();

    /* ---- LVGL init ---- */
    lv_init();
    lv_port_disp_init(DISP_HOR_RES, DISP_VER_RES, 0, false);
    lv_port_indev_init(DISP_HOR_RES, DISP_VER_RES, 0);

    static struct repeating_timer lvgl_timer;
    add_repeating_timer_ms(LVGL_TICK_PERIOD_MS,
                           repeating_lvgl_timer_cb, NULL, &lvgl_timer);

    /* ---- Protocol init ---- */
    invent_ems_init();

#if ECU_PROTOCOL == ECU_INVENT_EMS
    bsp_serial_init();
    uart_set_baudrate(uart0, INVENT_EMS_BAUD_RATE);
    irq_set_exclusive_handler(UART0_IRQ, uart0_irq_handler);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irqs_enabled(uart0, true, false);
#elif ECU_PROTOCOL == ECU_ME442
    multicore_launch_core1(core1_entry);
#endif

    /* ---- UI init ---- */
    ui_dashboard_init();
    ui_debug_console_init();

    lv_timer_create(dashboard_update_cb, DASHBOARD_UPDATE_MS, NULL);
#if ENABLE_DEBUG_CONSOLE
    lv_timer_create(debug_stats_cb, DEBUG_STATS_UPDATE_MS, NULL);
#endif

    /* ---- Super-loop ---- */
    uint16_t sleep_ms_val = LVGL_TICK_PERIOD_MS;

    while (true) {
#if ECU_PROTOCOL == ECU_INVENT_EMS
        /* Drain UART ring buffer → byte-level protocol parser */
        uint16_t head = uart_rx_head;
        while (uart_rx_tail != head) {
            uint8_t byte = uart_rx_buf[uart_rx_tail];
            uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUF_SIZE;
            invent_ems_feed_byte(byte);
        }
#endif
        /* Propagate "new ECU data" flag for the next LVGL timer tick */
        if (invent_ems_has_new_data())
            ecu_data_ready = true;

        sleep_ms_val = lv_timer_handler();
        if (sleep_ms_val > 500)             sleep_ms_val = 500;
        if (sleep_ms_val < LVGL_TICK_PERIOD_MS) sleep_ms_val = LVGL_TICK_PERIOD_MS;
        sleep_ms(sleep_ms_val);
    }
}
