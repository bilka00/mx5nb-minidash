/**
 * bsp_cd5300.c — CO5300 OLED display controller driver
 *
 * Drives a 466x466 circular AMOLED panel via PIO2 QSPI.
 *
 * Command path (register writes, window setup):
 *   CS LOW → 1-bit SPI blocking write → CS HIGH
 *
 * Pixel flush path (DMA, non-blocking):
 *   set_window(CASET + RASET) → CS LOW → RAMWR cmd (1-bit) →
 *   pixel data (4-bit QSPI DMA) → DMA ISR → wait_idle → CS HIGH →
 *   lv_disp_flush_ready()
 *
 * Brightness is deferred: if a brightness change is requested while a
 * DMA flush is in progress, it is applied in the DMA-done callback
 * after CS is released.
 */

#include "bsp_co5300.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pio_qspi.h"

/* ---- State ---- */

static bsp_display_interface_t *g_display_if;
static bsp_display_info_t      *g_display_info;
static bool                     g_set_brightness_flag = false;

/* ---- Command transmission ---- */

typedef struct {
    uint8_t       reg;          /* CO5300 register / command byte */
    uint8_t      *data;         /* Parameter bytes (may be NULL) */
    size_t        data_bytes;   /* Length of data[] */
    unsigned int  delay_ms;     /* Post-command delay (0 = none) */
} oled_cmd_t;

/**
 * Send one or more register commands over 1-bit SPI.
 * Each command is framed with: [0x02, 0x00, reg, 0x00, data...]
 * (0x02 = CO5300 SPI-mode write prefix).
 */
static void tx_param(oled_cmd_t *cmds, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        uint8_t pkt[4 + cmds[i].data_bytes];

        pkt[0] = 0x02;                 /* SPI write prefix */
        pkt[1] = 0x00;
        pkt[2] = cmds[i].reg;
        pkt[3] = 0x00;
        for (size_t j = 0; j < cmds[i].data_bytes; j++)
            pkt[4 + j] = cmds[i].data[j];

        gpio_put(BSP_OLED_CS_PIN, 0);
        pio_qspi_1bit_write_data_blocking(pkt, 4 + cmds[i].data_bytes);
        gpio_put(BSP_OLED_CS_PIN, 1);

        if (cmds[i].delay_ms > 0)
            sleep_ms(cmds[i].delay_ms);
    }
}

/* ---- DMA completion ISR callback ---- */

/**
 * Called from DMA IRQ when a pixel flush transfer completes.
 * Must run in ISR context (registered on DMA_IRQ_1 via bsp_dma_channel_irq).
 *
 * Ordering is critical: wait for PIO shift register to drain, then
 * release CS *before* signalling LVGL that the buffer is free.
 */
static void __no_inline_not_in_flash_func(flush_dma_done_cb)(void)
{
    pio_qspi_wait_idle();
    gpio_put(BSP_OLED_CS_PIN, 1);

    if (g_display_info->dma_flush_done_cb)
        g_display_info->dma_flush_done_cb();

    /* Apply deferred brightness change now that CS is free */
    if (g_set_brightness_flag) {
        g_set_brightness_flag = false;
        uint8_t level = 0x25 + g_display_info->brightness * (0xFF - 0x25) / 100;
        oled_cmd_t cmd = {
            .reg = 0x51, .data = &level, .data_bytes = 1, .delay_ms = 0,
        };
        tx_param(&cmd, 1);
    }
}

/* ---- Interface implementation ---- */

static void set_rotation(uint16_t rotation)
{
    (void)rotation;
    g_display_info->rotation = 0;
    /* Hardware rotation is not supported on CO5300 — use sw_rotate */
}

static void set_brightness(uint8_t brightness)
{
    if (brightness > 100)
        brightness = 100;
    g_display_info->brightness = brightness;
    g_set_brightness_flag = true;
}

static void init(void)
{
    /* GPIO setup for CS, RST, and panel power */
    gpio_init(BSP_OLED_CS_PIN);
    gpio_init(BSP_OLED_RST_PIN);
    gpio_init(BSP_OLED_PWR_PIN);

    gpio_set_dir(BSP_OLED_CS_PIN,  GPIO_OUT);
    gpio_set_dir(BSP_OLED_RST_PIN, GPIO_OUT);
    gpio_set_dir(BSP_OLED_PWR_PIN, GPIO_OUT);
    gpio_put(BSP_OLED_PWR_PIN, 1);

    pio_qspi_init(BSP_OLED_SCLK_PIN, BSP_OLED_D0_PIN,
                   75 * 1000 * 1000, flush_dma_done_cb);

    /* Hardware reset */
    gpio_put(BSP_OLED_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(BSP_OLED_RST_PIN, 1);
    sleep_ms(200);

    /* CO5300 initialisation sequence */
    oled_cmd_t init_cmds[] = {
        {0x11, (uint8_t[]){0x00}, 0, 120},  /* Sleep out */
        {0xC4, (uint8_t[]){0x80}, 1, 0},    /* Column inversion */
        {0x44, (uint8_t[]){0x01, 0xD7}, 2, 0}, /* TE scanline */
        {0x35, (uint8_t[]){0x00}, 1, 0},    /* Tearing effect on */
        {0x53, (uint8_t[]){0x20}, 1, 10},   /* Brightness ctrl on */
        {0x29, (uint8_t[]){0x00}, 0, 10},   /* Display on */
        {0x51, (uint8_t[]){0xA0}, 1, 0},    /* Initial brightness */
        {0x20, (uint8_t[]){0x00}, 0, 0},    /* Inversion off */
        {0x36, (uint8_t[]){0x00}, 1, 0},    /* MADCTL = 0 (no hw rotation) */
        {0x3A, (uint8_t[]){0x05}, 1, 0},    /* Pixel format: RGB565 */
    };
    tx_param(init_cmds, sizeof(init_cmds) / sizeof(init_cmds[0]));

    set_brightness(g_display_info->brightness);
}

/**
 * Program the CO5300 column and row address window (CASET + RASET).
 * Called before every pixel data transfer.
 */
static void set_window(bsp_display_area_t *area)
{
    uint16_t x1 = area->x1 + g_display_info->x_offset;
    uint16_t x2 = area->x2 + g_display_info->x_offset;
    uint16_t y1 = area->y1 + g_display_info->y_offset;
    uint16_t y2 = area->y2 + g_display_info->y_offset;

    uint8_t col_data[] = { x1 >> 8, x1 & 0xFF, x2 >> 8, x2 & 0xFF };
    uint8_t row_data[] = { y1 >> 8, y1 & 0xFF, y2 >> 8, y2 & 0xFF };

    oled_cmd_t cmds[] = {
        { .reg = 0x2A, .data = col_data, .data_bytes = 4, .delay_ms = 0 },
        { .reg = 0x2B, .data = row_data, .data_bytes = 4, .delay_ms = 0 },
    };
    tx_param(cmds, 2);
}

static void get_rotation(uint16_t *rotation)
{
    *rotation = 0;
}

static void get_brightness(uint8_t *brightness)
{
    *brightness = g_display_info->brightness;
}

/**
 * Start a non-blocking DMA pixel flush.
 *
 * Sends set_window, then the RAMWR command (0x2C) in 1-bit SPI mode,
 * followed by the pixel buffer in 4-bit QSPI mode via DMA.  The DMA
 * completion ISR (flush_dma_done_cb) handles CS release and signals
 * LVGL.
 */
static void flush_dma(bsp_display_area_t *area, uint16_t *color_p)
{
    size_t pixel_count = (area->x2 - area->x1 + 1) *
                         (area->y2 - area->y1 + 1);
    set_window(area);

    /* RAMWR command header (0x32 = QSPI write prefix, 0x2C = RAMWR) */
    uint8_t ramwr_cmd[] = { 0x32, 0x00, 0x2C, 0x00 };
    gpio_put(BSP_OLED_CS_PIN, 0);
    pio_qspi_1bit_write_data_blocking(ramwr_cmd, 4);

    /* Pixel data via 4-bit QSPI DMA (non-blocking) */
    pio_qspi_4bit_write_data((uint8_t *)color_p, pixel_count * 2);
}

/* ---- Constructor ---- */

bool bsp_display_new_co5300(bsp_display_interface_t **interface,
                            bsp_display_info_t *info)
{
    if (info == NULL)
        return false;

    static bsp_display_interface_t display_if;
    static bsp_display_info_t      display_info;

    memcpy(&display_info, info, sizeof(bsp_display_info_t));

    display_if.init           = init;
    display_if.reset          = NULL;       /* reset is part of init */
    display_if.set_rotation   = set_rotation;
    display_if.set_brightness = set_brightness;
    display_if.set_window     = set_window;
    display_if.get_brightness = get_brightness;
    display_if.get_rotation   = get_rotation;
    display_if.flush          = NULL;       /* blocking flush not used */
    display_if.flush_dma      = flush_dma;

    *interface     = &display_if;
    g_display_if   = &display_if;
    g_display_info = &display_info;
    return true;
}
