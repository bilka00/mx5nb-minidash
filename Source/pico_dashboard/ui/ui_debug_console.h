#ifndef UI_DEBUG_CONSOLE_H
#define UI_DEBUG_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "bsp_can.h"

#if ENABLE_DEBUG_CONSOLE

void ui_debug_console_init(void);

void ui_debug_console_update_stats(
    uint32_t uart_pkts, uint32_t uart_errs, bool uart_connected,
    const bsp_can_stats_t *can);

#else /* stubs — optimised away completely */

static inline void ui_debug_console_init(void) {}
static inline void ui_debug_console_update_stats(
    uint32_t uart_pkts, uint32_t uart_errs, bool uart_connected,
    const bsp_can_stats_t *can)
{
    (void)uart_pkts; (void)uart_errs; (void)uart_connected; (void)can;
}

#endif /* ENABLE_DEBUG_CONSOLE */

#ifdef __cplusplus
}
#endif

#endif /* UI_DEBUG_CONSOLE_H */
