/**
 * lv_port_disp.h — LVGL display-port interface
 */

#ifndef LV_PORT_DISP_H
#define LV_PORT_DISP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

/**
 * Initialise the display driver and register it with LVGL.
 *
 * @param width              Horizontal resolution (pixels).
 * @param height             Vertical resolution (pixels).
 * @param rotation           Hardware rotation (unused — sw_rotate is forced).
 * @param enabled_direct_mode  If true, allocate a full-screen buffer and
 *                             enable LVGL direct mode; otherwise use double
 *                             1/8-screen buffers with DMA pipelining.
 */
void lv_port_disp_init(uint16_t width, uint16_t height,
                        uint16_t rotation, bool enabled_direct_mode);

#ifdef __cplusplus
}
#endif

#endif /* LV_PORT_DISP_H */
