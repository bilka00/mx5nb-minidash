#include "bsp_ft6146.h"
#include "bsp_i2c.h"

static bsp_touch_interface_t *g_touch_if;
static bsp_touch_info_t *g_touch_info;

static bool g_ft6146_irq_flag = false;

static void bsp_ft6146_reg_read_byte(uint8_t reg_addr, uint8_t *data, size_t len)
{
    bsp_i2c_read_reg8(FT6146_DEVICE_ADDR, reg_addr, data, len);
}

static void bsp_ft6146_reg_write_byte(uint8_t reg_addr, uint8_t *data, size_t len)
{
    bsp_i2c_write_reg8(FT6146_DEVICE_ADDR, reg_addr, data, len);
}

static void bsp_ft6146_reset(void)
{

    gpio_put(BSP_FT6146_RST_PIN, 0);
    sleep_ms(10);
    gpio_put(BSP_FT6146_RST_PIN, 1);
    sleep_ms(100);
}

static void bsp_ft6146_read(void)
{

#if defined(BSP_FT6146_INT_PIN) && (BSP_FT6146_INT_PIN != -1)
    if (g_ft6146_irq_flag == false)
    {
        g_touch_info->data.points = 0;
        return;
    }
    g_ft6146_irq_flag = false;
#endif
    uint8_t buffer[11];
    bsp_ft6146_reg_read_byte(FT6146_REG_DATA_START, buffer, 11);
    if (buffer[0] > 0 && buffer[0] <= FT6146_LCD_TOUCH_MAX_POINTS)
    {
        for (int i = 0; i < buffer[0]; i++)
        {
            g_touch_info->data.coords[i].x = (uint16_t)((buffer[1 + 6 * i] & 0x0f) << 8);
            g_touch_info->data.coords[i].x |= buffer[2 + 6 * i];

            g_touch_info->data.coords[i].y = (uint16_t)((buffer[3 + 6 * i] & 0x0f) << 8);
            g_touch_info->data.coords[i].y |= buffer[4 + 6 * i];
        }
        g_touch_info->data.points = buffer[0];
    }
    else
        g_touch_info->data.points = 0;
}

static bool bsp_ft6146_get_touch_data(bsp_touch_data_t *data)
{
    memcpy(data, &g_touch_info->data, sizeof(bsp_touch_data_t));
    g_touch_info->data.points = 0;

    if (data->points == 0)
        return false;

    switch (g_touch_info->rotation)
    {
    case 1:
        for (int i = 0; i < data->points; i++)
        {
            data->coords[i].x = g_touch_info->data.coords[i].y;
            data->coords[i].y = g_touch_info->height - 1 - g_touch_info->data.coords[i].x;
        }
        break;
    case 2:
        for (int i = 0; i < data->points; i++)
        {
            data->coords[i].x = g_touch_info->width - 1 - g_touch_info->data.coords[i].x;
            data->coords[i].y = g_touch_info->height - 1 - g_touch_info->data.coords[i].y;
        }
        break;

    case 3:
        for (int i = 0; i < data->points; i++)
        {
            data->coords[i].x = g_touch_info->width - g_touch_info->data.coords[i].y;
            data->coords[i].y = g_touch_info->data.coords[i].x;
        }
        break;
    default:
        break;
    }
    return true;
}

static void bsp_ft6146_set_rotation(uint16_t rotation)
{
    uint16_t swap;

    if (rotation == 1 || rotation == 3)
    {
        if (g_touch_info->width < g_touch_info->height)
        {
            swap = g_touch_info->width;
            g_touch_info->width = g_touch_info->height;
            g_touch_info->height = swap;
        }
    }
    else
    {
        if (g_touch_info->width > g_touch_info->height)
        {
            swap = g_touch_info->width;
            g_touch_info->width = g_touch_info->height;
            g_touch_info->height = swap;
        }
    }
    g_touch_info->rotation = rotation;
}

static void gpio_irq_callbac(uint gpio, uint32_t event_mask)
{
    if (event_mask == GPIO_IRQ_EDGE_FALL)
    {
        g_ft6146_irq_flag = true;
    }
}

static void bsp_ft6146_init(void)
{
    uint8_t id = 0;
#if defined(BSP_FT6146_RST_PIN) && (BSP_FT6146_RST_PIN != -1)
    gpio_init(BSP_FT6146_RST_PIN);
    gpio_set_dir(BSP_FT6146_RST_PIN, GPIO_OUT);
    bsp_ft6146_reset();
#endif

#if defined(BSP_FT6146_INT_PIN) && (BSP_FT6146_INT_PIN != -1)
    gpio_init(BSP_FT6146_INT_PIN);
    gpio_set_dir(BSP_FT6146_INT_PIN, GPIO_IN);
    gpio_pull_up(BSP_FT6146_INT_PIN);
    gpio_set_irq_enabled_with_callback(BSP_FT6146_INT_PIN, GPIO_IRQ_EDGE_FALL, true, gpio_irq_callbac);
#endif

    bsp_ft6146_reg_read_byte(FT6146_REG_CHIP_ID, &id, 1);
    printf("id: 0x%02x\r\n", id);
}

static void bsp_ft6146_get_rotation(uint16_t *rotation)
{
    *rotation = g_touch_info->rotation;
}

bool bsp_touch_new_ft6146(bsp_touch_interface_t **interface, bsp_touch_info_t *info)
{
    if (info == NULL)
        return false;

    static bsp_touch_interface_t touch_if;
    static bsp_touch_info_t touch_info;

    memcpy(&touch_info, info, sizeof(bsp_touch_info_t));

    touch_if.init = bsp_ft6146_init;
    touch_if.reset = bsp_ft6146_reset;
    touch_if.read = bsp_ft6146_read;
    touch_if.get_data = bsp_ft6146_get_touch_data;
    touch_if.get_rotation = bsp_ft6146_get_rotation;
    touch_if.set_rotation = bsp_ft6146_set_rotation;

    g_touch_if = &touch_if;
    *interface = &touch_if;
    g_touch_info = &touch_info;
    return true;
}