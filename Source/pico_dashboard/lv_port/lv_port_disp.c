/**
 * lv_port_disp.c — LVGL display driver for CO5300 OLED
 *
 * Sets up double-buffered rendering (2x 1/8-screen buffers) with DMA
 * flushing.  Software rotation (270 degrees) is used because the
 * CO5300 MADCTL register does not support hardware rotation.
 *
 * A rounder callback aligns dirty areas to even pixel boundaries —
 * required by the CO5300 column/row addressing.
 */

#include "lv_port_disp.h"
#include <stdbool.h>
#include "config.h"
#include "bsp_co5300.h"

/* ---- State ---- */

static lv_disp_drv_t         disp_drv;
static bsp_display_interface_t *display_if;

/* ---- Callbacks ---- */

/** DMA-complete callback — invoked from ISR context by bsp_cd5300. */
static void disp_flush_done(void)
{
    lv_disp_flush_ready(&disp_drv);
}

/**
 * Rounder: align dirty-area edges to even pixel boundaries.
 * The CO5300 column/row commands require even-aligned start addresses
 * and odd-aligned end addresses (pairs of pixels per transfer unit).
 */
static void rounder_cb(lv_disp_drv_t *drv, lv_area_t *area)
{
    (void)drv;
    area->x1 = (area->x1 >> 1) << 1;           /* round down to even */
    area->y1 = (area->y1 >> 1) << 1;
    area->x2 = ((area->x2 >> 1) << 1) + 1;     /* round up to odd   */
    area->y2 = ((area->y2 >> 1) << 1) + 1;
}

/**
 * Flush callback — hands the pixel buffer to the display driver for
 * DMA transfer.  lv_disp_flush_ready() is called asynchronously from
 * the DMA-complete ISR via disp_flush_done().
 */
static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area,
                        lv_color_t *color_p)
{
    (void)drv;
    bsp_display_area_t da = {
        .x1 = area->x1, .y1 = area->y1,
        .x2 = area->x2, .y2 = area->y2,
    };
    display_if->flush_dma(&da, (uint16_t *)color_p);
}

/* ---- Public API ---- */

void lv_port_disp_init(uint16_t width, uint16_t height,
                        uint16_t rotation, bool enabled_direct_mode)
{
    /* Initialise the CO5300 hardware driver */
    bsp_display_info_t info = {
        .width     = height,        /* swapped: logical W/H vs panel H/W */
        .height    = width,
        .x_offset  = 6,
        .y_offset  = 0,
        .rotation  = rotation,
        .brightness = 80,
        .dma_flush_done_cb = disp_flush_done,
    };
    bsp_display_new_co5300(&display_if, &info);
    display_if->init();

    /* Allocate draw buffers */
    static lv_disp_draw_buf_t draw_buf;

    if (enabled_direct_mode) {
        lv_color_t *buf = (lv_color_t *)malloc(
            width * height * sizeof(lv_color_t));
        lv_disp_draw_buf_init(&draw_buf, buf, NULL, width * height);
    } else {
        /* Double buffer, each 1/8 of the screen — allows DMA to flush
         * one buffer while LVGL renders into the other. */
        size_t buf_px = (size_t)width * height / 8;
        lv_color_t *buf1 = (lv_color_t *)malloc(buf_px * sizeof(lv_color_t));
        lv_color_t *buf2 = (lv_color_t *)malloc(buf_px * sizeof(lv_color_t));
        lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buf_px);
    }

    /* Register the LVGL display driver */
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res      = width;
    disp_drv.ver_res      = height;
    disp_drv.flush_cb     = disp_flush;
    disp_drv.direct_mode  = enabled_direct_mode;
    disp_drv.rounder_cb   = rounder_cb;

    /* Software rotation (MADCTL does not work on CO5300) */
    disp_drv.sw_rotate = (DISP_ROTATION != LV_DISP_ROT_NONE);
    disp_drv.rotated   = DISP_ROTATION;

    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
}
