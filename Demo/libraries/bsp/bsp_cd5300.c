

#include "bsp_co5300.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pio_qspi.h"

static bsp_display_interface_t *g_display_if;
static bsp_display_info_t *g_display_info;
static bool g_set_brightness_flag = false;


static void init(void);
static void reset(void);
static void set_rotation(uint16_t rotation);
static void set_brightness(uint8_t brightness);
static void set_window(bsp_display_area_t *area);
static void get_rotation(uint16_t *rotation);
static void get_brightness(uint8_t *brightness);
static void flush(bsp_display_area_t *area, uint16_t *color_p);

typedef struct
{
    uint8_t reg;           /*<! The specific OLED command */
    uint8_t *data;         /*<! Buffer that holds the command specific data */
    size_t data_bytes;     /*<! Size of `data` in memory, in bytes */
    unsigned int delay_ms; /*<! Delay in milliseconds after this command */
} oled_cmd_t;

static void tx_param(oled_cmd_t *cmds, size_t cmd_len)
{
    for (int i = 0; i < cmd_len; i++)
    {
        uint8_t write_buffer[4 + cmds[i].data_bytes];
        gpio_put(BSP_OLED_CS_PIN, 0);

        write_buffer[0] = 0x02;
        write_buffer[1] = 0x00;
        write_buffer[2] = cmds[i].reg;
        write_buffer[3] = 0x00;

        for (int j = 0; j < cmds[i].data_bytes; j++)
        {
            write_buffer[4 + j] = cmds[i].data[j];
        }
        pio_qspi_1bit_write_data_blocking(write_buffer, 4 + cmds[i].data_bytes);
        // spi_write_blocking(BSP_OLED_SPI_NUM, write_buffer, 4 + cmds[i].data_bytes);
        gpio_put(BSP_OLED_CS_PIN, 1);
        if (cmds[i].delay_ms > 0)
        {
            sleep_ms(cmds[i].delay_ms);
        }
    }
}

static void __no_inline_not_in_flash_func(flush_dma_done_cb)(void)
{
    __asm__ volatile ("nop");
    __asm__ volatile ("nop");
    if (g_display_info->dma_flush_done_cb)
    {
        g_display_info->dma_flush_done_cb();
    }
    gpio_put(BSP_OLED_CS_PIN, 1);

    if (g_set_brightness_flag)
    {
        g_set_brightness_flag = false;
        oled_cmd_t cmd;
        uint8_t cmd_data = 0x25 + g_display_info->brightness * (0xFF - 0x25) / 100;
        cmd.reg = 0x51;
        cmd.data = &cmd_data;
        cmd.data_bytes = 1;
        cmd.delay_ms = 0;
        tx_param(&cmd, 1);
    }
    

}


static void init(void)
{
    gpio_init(BSP_OLED_CS_PIN);
    gpio_init(BSP_OLED_RST_PIN);
    gpio_init(BSP_OLED_PWR_PIN);

    gpio_set_dir(BSP_OLED_CS_PIN, GPIO_OUT);
    gpio_set_dir(BSP_OLED_RST_PIN, GPIO_OUT);
    gpio_set_dir(BSP_OLED_PWR_PIN, GPIO_OUT);
    gpio_put(BSP_OLED_PWR_PIN, 1);
    pio_qspi_init(BSP_OLED_SCLK_PIN, BSP_OLED_D0_PIN, 75 * 1000 * 1000, flush_dma_done_cb);

    reset();
    oled_cmd_t co5300_init_cmds[] = {
        //  {cmd, { data }, data_size, delay_ms}
        {.reg = 0x11, .data = (uint8_t[]){0x00}, .data_bytes = 0, .delay_ms = 120},
        // {.reg = 0xff, .data = (uint8_t[]){0x00}, .data_bytes = 0, .delay_ms = 120},
        // {.reg = 0x3b, .data = (uint8_t[]){0x00}, .data_bytes = 0, .delay_ms = 10},
        {.reg = 0xc4, .data = (uint8_t[]){0x80}, .data_bytes = 1, .delay_ms = 0},
        {.reg = 0x44, .data = (uint8_t[]){0x01, 0xD7}, .data_bytes = 2, .delay_ms = 0},
        {.reg = 0x35, .data = (uint8_t[]){0x00}, .data_bytes = 1, .delay_ms = 0},
        {.reg = 0x53, .data = (uint8_t[]){0x20}, .data_bytes = 1, .delay_ms = 10},
        {.reg = 0x29, .data = (uint8_t[]){0x00}, .data_bytes = 0, .delay_ms = 10},
        {.reg = 0x51, .data = (uint8_t[]){0xA0}, .data_bytes = 1, .delay_ms = 0},
        {.reg = 0x20, .data = (uint8_t[]){0x00}, .data_bytes = 0, .delay_ms = 0},
        {.reg = 0x36, .data = (uint8_t[]){0x00}, .data_bytes = 1, 0},
        {.reg = 0x3A, .data = (uint8_t[]){0x05}, .data_bytes = 1, .delay_ms = 0},
    };
    tx_param(co5300_init_cmds, sizeof(co5300_init_cmds) / sizeof(oled_cmd_t));
    set_brightness(g_display_info->brightness);
}

static void reset(void)
{
    gpio_put(BSP_OLED_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(BSP_OLED_RST_PIN, 1);
    sleep_ms(200);
}

static void set_rotation(uint16_t rotation)
{
    g_display_info->rotation = 0;
    printf("Hardware rotation is not supported!!\r\n");
}
static void set_brightness(uint8_t brightness)
{

    if (brightness > 100)
        brightness = 100;
    
    g_display_info->brightness = brightness; 
    g_set_brightness_flag = true;
}
static void set_window(bsp_display_area_t *area)
{
    oled_cmd_t cmds[2];

    uint16_t x_start = area->x1 + g_display_info->x_offset;
    uint16_t x_end = area->x2 + g_display_info->x_offset;

    uint16_t y_start = area->y1 + g_display_info->y_offset;
    uint16_t y_end = area->y2 + g_display_info->y_offset;

    uint8_t x_data[4];
    uint8_t y_data[4];
    x_data[0] = (x_start >> 8) & 0xFF;
    x_data[1] = x_start & 0xFF;
    x_data[2] = (x_end >> 8) & 0xFF;
    x_data[3] = x_end & 0xFF;

    y_data[0] = (y_start >> 8) & 0xFF;
    y_data[1] = y_start & 0xFF;
    y_data[2] = (y_end >> 8) & 0xFF;
    y_data[3] = y_end & 0xFF;

    cmds[0].reg = 0x2a;
    cmds[0].data = x_data;
    cmds[0].data_bytes = 4;
    cmds[0].delay_ms = 0;

    cmds[1].reg = 0x2b;
    cmds[1].data = y_data;
    cmds[1].data_bytes = 4;
    cmds[1].delay_ms = 0;

    tx_param(cmds, 2);
}

static void get_rotation(uint16_t *rotation)
{
    *rotation = 0;
    printf("Hardware rotation is not supported!!\r\n");
}

static void get_brightness(uint8_t *brightness)
{
    *brightness = g_display_info->brightness;
}
static void flush(bsp_display_area_t *area, uint16_t *color_p)
{

}

static void flush_dma(bsp_display_area_t *area, uint16_t *color_p)
{
    uint8_t cmd_arr[4];
    cmd_arr[0] = 0x32;
    cmd_arr[1] = 0x00;
    cmd_arr[2] = 0x2c;
    cmd_arr[3] = 0x00;
    size_t color_len = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    set_window(area);
    gpio_put(BSP_OLED_CS_PIN, 0);
    pio_qspi_1bit_write_data_blocking(cmd_arr, 4);
    pio_qspi_4bit_write_data((uint8_t *)color_p, color_len * 2);
}

bool bsp_display_new_co5300(bsp_display_interface_t **interface, bsp_display_info_t *info)
{
    if (info == NULL)
        return false;
    static bsp_display_interface_t display_if;
    static bsp_display_info_t display_info;

    memcpy(&display_info, info, sizeof(bsp_display_info_t));


    display_if.init = init;
    display_if.reset = reset;

    display_if.set_rotation = set_rotation;
    display_if.set_brightness = set_brightness;
    display_if.set_window = set_window;

    display_if.get_brightness = get_brightness;
    display_if.get_rotation = get_rotation;

    display_if.flush = flush;
    display_if.flush_dma = flush_dma;

    *interface = &display_if;
    g_display_if = &display_if;
    g_display_info = &display_info;
    return true;
}