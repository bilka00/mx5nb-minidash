#ifndef __BSP_FT6146_H__
#define __BSP_FT6146_H__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "bsp_touch.h"


#define BSP_FT6146_RST_PIN 17
#define BSP_FT6146_INT_PIN 20

#define FT6146_LCD_TOUCH_MAX_POINTS (1)

#define FT6146_DEVICE_ADDR 0x38
#define FT6146_ID 0x02

typedef enum
{
    FT6146_REG_DATA_START = 0x02,
    FT6146_REG_CHIP_ID = 0xA0, 
} ft6146_reg_t;



bool bsp_touch_new_ft6146(bsp_touch_interface_t **interface, bsp_touch_info_t *info);

#endif