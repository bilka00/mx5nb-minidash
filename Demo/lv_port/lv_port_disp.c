/**
 * @file lv_port_disp_templ.c
 *
 */

/*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include <stdbool.h>

#include "bsp_co5300.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

static lv_disp_drv_t disp_drv; /*Descriptor of a display driver*/
bsp_display_interface_t *display_if;
/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_flush_done(void);

static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);
// static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//         const lv_area_t * fill_area, lv_color_t color);

/**********************
 *  STATIC VARIABLES
 **********************/

void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{

    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;

    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    // round the start of coordinate down to the nearest 2M number
    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;
    // round the end of coordinate up to the nearest 2N+1 number
    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}

void lv_port_disp_init(uint16_t width, uint16_t height, uint16_t rotation, bool enabled_direct_mode)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/


    bsp_display_info_t display_info;
    display_info.height = width;
    display_info.width = height;
    display_info.x_offset = 6;
    display_info.y_offset = 0;
    display_info.rotation = rotation;
    display_info.brightness = 80;
    display_info.dma_flush_done_cb = disp_flush_done;
    bsp_display_new_co5300(&display_if, &display_info);
    display_if->init();

    /*-----------------------------
     * Create a buffer for drawing
     *----------------------------*/

    /**
     * LVGL requires a buffer where it internally draws the widgets.
     * Later this buffer will passed to your display driver's `flush_cb` to copy its content to your display.
     * The buffer has to be greater than 1 display row
     *
     * There are 3 buffering configurations:
     * 1. Create ONE buffer:
     *      LVGL will draw the display's content here and writes it to your display
     *
     * 2. Create TWO buffer:
     *      LVGL will draw the display's content to a buffer and writes it your display.
     *      You should use DMA to write the buffer's content to the display.
     *      It will enable LVGL to draw the next part of the screen to the other buffer while
     *      the data is being sent form the first buffer. It makes rendering and flushing parallel.
     *
     * 3. Double buffering
     *      Set 2 screens sized buffers and set disp_drv.full_refresh = 1.
     *      This way LVGL will always provide the whole rendered screen in `flush_cb`
     *      and you only need to change the frame buffer's address.
     */

    /* Example for 1) */
    static lv_disp_draw_buf_t draw_buf_dsc;

    if (enabled_direct_mode)
    {
        // static lv_color_t buf[MY_DISP_HOR_RES * MY_DISP_VER_RES]; /*A screen sized buffer*/
        lv_color_t *buf = (lv_color_t *)malloc(width * height * sizeof(uint16_t));
        lv_disp_draw_buf_init(&draw_buf_dsc, buf, NULL,
                              width * height); /*Initialize the display buffer*/
    }
    else
    {
        lv_color_t *buf_1 = (lv_color_t *)malloc(width * height * sizeof(uint16_t) / 8);
        lv_color_t *buf_2 = (lv_color_t *)malloc(width * height * sizeof(uint16_t) / 8);
        lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, buf_2, width * height / 8); /*Initialize the display buffer*/
    }
    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    lv_disp_drv_init(&disp_drv); /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = width;
    disp_drv.ver_res = height;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;
    disp_drv.direct_mode = enabled_direct_mode;
    disp_drv.rounder_cb = example_lvgl_rounder_cb;

    /*Software rotation - 90 degrees counter-clockwise (= 270 degrees clockwise)*/
    disp_drv.sw_rotate = 1;
    disp_drv.rotated = LV_DISP_ROT_270;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc;

    /*Required for Example 3)*/
    // disp_drv.full_refresh = 1;

    /* Fill a memory array with a color if you have GPU.
     * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
     * But if you have a different GPU you can use with this callback.*/
    // disp_drv.gpu_fill_cb = gpu_fill;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    size_t color_len = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    bsp_display_area_t display_area = {
        .x1 = area->x1,
        .x2 = area->x2,
        .y1 = area->y1,
        .y2 = area->y2,
    };
    display_if->flush_dma(&display_area, (uint16_t *)color_p);
    // printf(" x1: %d, y1:%d \n x2:%d, y2:%d \n len:%d \n", area->x1, area->y1, area->x2, area->y2, color_len, color_p);
}

static void disp_flush_done(void)
{
    lv_disp_flush_ready(&disp_drv);
}

/*OPTIONAL: GPU INTERFACE*/

/*If your MCU has hardware accelerator (GPU) then you can use it to fill a memory with a color*/
// static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//                     const lv_area_t * fill_area, lv_color_t color)
//{
//     /*It's an example code which should be done by your GPU*/
//     int32_t x, y;
//     dest_buf += dest_width * fill_area->y1; /*Go to the first line*/
//
//     for(y = fill_area->y1; y <= fill_area->y2; y++) {
//         for(x = fill_area->x1; x <= fill_area->x2; x++) {
//             dest_buf[x] = color;
//         }
//         dest_buf+=dest_width;    /*Go to the next line*/
//     }
// }

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
