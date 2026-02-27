/**
 * ui_debug_console.c — tap-to-show bus statistics overlay
 *
 * A full-screen semi-transparent panel (80 % opacity) sits on top of
 * the dashboard.  Tap the dashboard to open, tap the panel to close.
 * When hidden, update_stats() returns immediately — no rendering cost.
 *
 * Compiled only when ENABLE_DEBUG_CONSOLE=1 (see ui_debug_console.h).
 */

#include "ui_debug_console.h"

#if ENABLE_DEBUG_CONSOLE

#include "lvgl.h"
#include <stdio.h>

#define PANEL_SIZE      466
#define PANEL_RADIUS    (PANEL_SIZE / 2)    /* circular to match display */

/* ---- State ---- */
static lv_obj_t *console_panel;
static lv_obj_t *console_label;
static bool      console_visible;

/* Previous counters for rate calculation (delta × 5 = per-second) */
static uint32_t prev_uart_pkts;
static uint32_t prev_can_rx;

/* ---- Event handlers ---- */

static void dashboard_click_cb(lv_event_t *e)
{
    (void)e;
    if (!console_visible) {
        lv_obj_clear_flag(console_panel, LV_OBJ_FLAG_HIDDEN);
        console_visible = true;
    }
}

static void panel_click_cb(lv_event_t *e)
{
    (void)e;
    if (console_visible) {
        lv_obj_add_flag(console_panel, LV_OBJ_FLAG_HIDDEN);
        console_visible = false;
    }
}

/* ---- Public API ---- */

void ui_debug_console_init(void)
{
    lv_obj_t *screen = lv_scr_act();

    /* Make the dashboard itself tappable so we can open the console */
    lv_obj_add_flag(screen, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(screen, dashboard_click_cb, LV_EVENT_CLICKED, NULL);

    /* Semi-transparent overlay */
    console_panel = lv_obj_create(screen);
    lv_obj_set_size(console_panel, PANEL_SIZE, PANEL_SIZE);
    lv_obj_center(console_panel);
    lv_obj_set_style_bg_color(console_panel, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(console_panel, LV_OPA_80, 0);
    lv_obj_set_style_border_width(console_panel, 0, 0);
    lv_obj_set_style_radius(console_panel, PANEL_RADIUS, 0);
    lv_obj_set_style_pad_top(console_panel, 90, 0);
    lv_obj_set_style_pad_bottom(console_panel, 90, 0);
    lv_obj_set_style_pad_left(console_panel, 70, 0);
    lv_obj_set_style_pad_right(console_panel, 70, 0);
    lv_obj_clear_flag(console_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(console_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(console_panel, panel_click_cb, LV_EVENT_CLICKED, NULL);

    /* Monospaced-looking stats label */
    console_label = lv_label_create(console_panel);
    lv_label_set_text(console_label, "Bus Statistics\nWaiting for data...");
    lv_obj_set_style_text_color(console_label, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(console_label, &lv_font_montserrat_12, 0);
    lv_obj_set_width(console_label, 260);
    lv_label_set_long_mode(console_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(console_label, LV_ALIGN_TOP_MID, 0, 0);

    /* Start hidden — no rendering cost until the user taps */
    lv_obj_add_flag(console_panel, LV_OBJ_FLAG_HIDDEN);
    console_visible = false;
}

void ui_debug_console_update_stats(
    uint32_t uart_pkts, uint32_t uart_errs, bool uart_connected,
    const bsp_can_stats_t *can)
{
    if (!console_visible) return;

    /* Rate: this callback fires every 200 ms → delta × 5 = per second */
    uint32_t uart_rate = (uart_pkts - prev_uart_pkts) * 5;
    uint32_t can_rate  = (can->rx_total - prev_can_rx) * 5;
    prev_uart_pkts = uart_pkts;
    prev_can_rx    = can->rx_total;

    static char buf[384];
    snprintf(buf, sizeof(buf),
        "UART  %s\n"
        "  pkts:%lu rate:%lu err:%lu\n"
        "\n"
        "CAN   %s\n"
        "  rx:%lu tx:%lu att:%lu\n"
        "  rate:%lu err:%lu\n"
        "  irq:%lu clk:%luMHz\n"
        "  RXpin:%u errSt:%lu",
        uart_connected ? "OK" : "--",
        (unsigned long)uart_pkts,
        (unsigned long)uart_rate,
        (unsigned long)uart_errs,
        can->connected ? "OK" : "--",
        (unsigned long)can->rx_total,
        (unsigned long)can->tx_total,
        (unsigned long)can->tx_attempt,
        (unsigned long)can_rate,
        (unsigned long)can->parse_error,
        (unsigned long)can->irq_count,
        (unsigned long)(can->sys_clk_hz / 1000000),
        (unsigned)can->rx_pin_raw,
        (unsigned long)can->err_state);

    lv_label_set_text_static(console_label, buf);
}

#endif /* ENABLE_DEBUG_CONSOLE */
