/**
 * config.h — compile-time project configuration
 *
 * All tuneable parameters in one place.  Override any value via
 * CMakeLists.txt  target_compile_definitions()  or  -D  on the
 * command line — the #ifndef guards will respect the override.
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ---- ECU protocol -------------------------------------------------- */

#define ECU_INVENT_EMS      1       /* Invent Labs EMS, RS232 19200 bps */
#define ECU_ME442           2       /* ME442, CAN bus 500 kbps          */

#ifndef ECU_PROTOCOL
#define ECU_PROTOCOL        ECU_ME442
#endif

/* ---- Display ------------------------------------------------------- */

#ifndef DISP_HOR_RES
#define DISP_HOR_RES        466
#endif

#ifndef DISP_VER_RES
#define DISP_VER_RES        466
#endif

/* Software rotation (MADCTL does not work on CO5300).
 * Possible values: LV_DISP_ROT_NONE, LV_DISP_ROT_90,
 *                  LV_DISP_ROT_180, LV_DISP_ROT_270              */
#ifndef DISP_ROTATION
#define DISP_ROTATION       LV_DISP_ROT_270
#endif

/* ---- CPU clock (MHz) ----------------------------------------------- */

#ifndef CPU_CLOCK_MHZ
#define CPU_CLOCK_MHZ       240
#endif

/* ---- LVGL timing --------------------------------------------------- */

#ifndef LVGL_TICK_PERIOD_MS
#define LVGL_TICK_PERIOD_MS 1
#endif

/* ---- Dashboard refresh rate ---------------------------------------- */

#ifndef DASHBOARD_UPDATE_MS
#define DASHBOARD_UPDATE_MS 50      /* arc gauge refresh interval */
#endif

/* ---- Debug console ------------------------------------------------- */

#ifndef ENABLE_DEBUG_CONSOLE
#define ENABLE_DEBUG_CONSOLE 1
#endif

#ifndef DEBUG_STATS_UPDATE_MS
#define DEBUG_STATS_UPDATE_MS 200
#endif

#endif /* CONFIG_H */
