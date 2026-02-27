#include "bsp_can.h"
#include "can2040.h"
#include "pico.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include <string.h>

static struct can2040 cbus;
static volatile uint32_t irq_cnt = 0;

/* ---- RX ring buffer (IRQ → main loop) ---- */
#define CAN_RX_BUF_SIZE 32
static bsp_can_frame_t rx_buf[CAN_RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;
static uint8_t rx_tail = 0;

/* ---- can2040 RX callback (IRQ context) ---- */
static void can_rx_cb(struct can2040 *cd, uint32_t notify,
                       struct can2040_msg *msg)
{
    if (notify == CAN2040_NOTIFY_RX) {
        uint8_t next = (rx_head + 1) % CAN_RX_BUF_SIZE;
        if (next != rx_tail) {
            rx_buf[rx_head].id  = msg->id;
            rx_buf[rx_head].dlc = (uint8_t)msg->dlc;
            memcpy(rx_buf[rx_head].data, msg->data, 8);
            rx_head = next;
        }
    }
}

/* ---- PIO IRQ trampoline — kept out of flash for XIP safety ---- */
static void __not_in_flash_func(can_pio_irq_handler)(void)
{
    irq_cnt++;
    can2040_pio_irq_handler(&cbus);
}

/* ---- Public API ---- */

void bsp_can_init(void)
{
    /* SN65HVD230 Rs pin: 10k pull-down keeps ~1.5V (slope control, active).
       Leave it alone — same as InventEmu. */

    can2040_setup(&cbus, BSP_CAN_PIO_NUM);
    can2040_callback_config(&cbus, can_rx_cb);

    uint pio_irq = PIO0_IRQ_0;
    irq_set_exclusive_handler(pio_irq, can_pio_irq_handler);
    irq_set_priority(pio_irq, 0);
    irq_set_enabled(pio_irq, true);

    can2040_start(&cbus, clock_get_hz(clk_sys), BSP_CAN_BITRATE,
                  BSP_CAN_GPIO_RX, BSP_CAN_GPIO_TX);
}

bsp_can_stats_t bsp_can_get_stats(void)
{
    struct can2040_stats raw;
    can2040_get_statistics(&cbus, &raw);

    bsp_can_stats_t st;
    st.rx_total    = raw.rx_total;
    st.tx_total    = raw.tx_total;
    st.tx_attempt  = raw.tx_attempt;
    st.parse_error = raw.parse_error;
    st.sys_clk_hz  = clock_get_hz(clk_sys);
    st.irq_count   = irq_cnt;
    st.connected   = (raw.rx_total > 0);
    st.err_state   = raw.parse_error_state;
    return st;
}

bool bsp_can_recv(bsp_can_frame_t *frame)
{
    if (rx_tail == rx_head) return false;
    *frame = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % CAN_RX_BUF_SIZE;
    return true;
}
