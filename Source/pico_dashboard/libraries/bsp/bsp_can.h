#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include <stdint.h>
#include <stdbool.h>

#define BSP_CAN_PIO_NUM   0
#define BSP_CAN_BITRATE   500000
#define BSP_CAN_GPIO_TX   21
#define BSP_CAN_GPIO_RX   22
#define BSP_CAN_GPIO_SLP  23

typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
} bsp_can_frame_t;

typedef struct {
    uint32_t rx_total;
    uint32_t tx_total;
    uint32_t tx_attempt;
    uint32_t parse_error;
    uint32_t sys_clk_hz;
    uint32_t irq_count;
    bool     connected;
    uint8_t  rx_pin_raw;   /* live GPIO22 state: 1=recessive, 0=dominant */
    uint32_t err_state;    /* last parse_state that caused parse_error */
} bsp_can_stats_t;

void bsp_can_init(void);
bsp_can_stats_t bsp_can_get_stats(void);

/* Returns true and fills *frame if a frame is available */
bool bsp_can_recv(bsp_can_frame_t *frame);

#endif /* __BSP_CAN_H__ */
