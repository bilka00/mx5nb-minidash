
#ifndef __BSP_CO5300_H__
#define __BSP_CO5300_H__

#include "bsp_display.h"


#define BSP_OLED_SCLK_PIN       10
#define BSP_OLED_D0_PIN         11
#define BSP_OLED_D1_PIN         12
#define BSP_OLED_D2_PIN         13
#define BSP_OLED_D3_PIN         14


#define BSP_OLED_CS_PIN         15
#define BSP_OLED_RST_PIN        16

#define BSP_OLED_PWR_PIN        19


bool bsp_display_new_co5300(bsp_display_interface_t **interface, bsp_display_info_t *info);

#endif // __BSP_CO5300_H__