#ifndef UI_DASHBOARD_H
#define UI_DASHBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

void ui_dashboard_init(void);
void ui_dashboard_set_oil_pressure(float bar);
void ui_dashboard_set_coolant_temp(float celsius);
void ui_dashboard_set_oil_temp(float celsius);
void ui_dashboard_demo_mode(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* UI_DASHBOARD_H */
