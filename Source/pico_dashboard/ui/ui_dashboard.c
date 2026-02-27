/**
 * ui_dashboard.c — concentric arc gauge display
 *
 * Three gauges rendered as LVGL arc pairs (background + foreground).
 * Each gauge has a separate background arc (static dark track) and a
 * foreground arc (colored indicator only, MAIN part transparent).
 * Splitting into two objects prevents LVGL dirty-area glitches that
 * occur when a single arc changes both MAIN and INDICATOR regions.
 *
 * All setter functions must be called from LVGL timer context only.
 * They use lv_label_set_text_static() to avoid repeated lv_mem
 * allocations — each static buffer stays valid until the next call.
 */

#include "ui_dashboard.h"
#include <stdio.h>
#include <math.h>

/* ---- Layout ---- */
#define SCREEN_SIZE             466

#define ARC_WIDTH               30
#define ARC_GAP                 8
#define ARC_OIL_PRESS_RADIUS    (SCREEN_SIZE / 2 - 20)
#define ARC_COOLANT_RADIUS      (ARC_OIL_PRESS_RADIUS - ARC_WIDTH - ARC_GAP)
#define ARC_OIL_TEMP_RADIUS     (ARC_COOLANT_RADIUS   - ARC_WIDTH - ARC_GAP)

/* ---- Gauge ranges ---- */
#define OIL_PRESSURE_MIN    0.0f
#define OIL_PRESSURE_MAX    10.0f       /* bar */
#define COOLANT_TEMP_MIN    40.0f
#define COOLANT_TEMP_MAX    120.0f      /* celsius */
#define OIL_TEMP_MIN        40.0f
#define OIL_TEMP_MAX        150.0f      /* celsius */

/* ---- Palette ---- */
#define COLOR_OIL_PRESSURE  lv_color_hex(0x00BFFF)     /* Deep Sky Blue */
#define COLOR_COOLANT       lv_color_hex(0xFF6B6B)     /* Red / Coral */
#define COLOR_OIL_TEMP      lv_color_hex(0xFFD93D)     /* Yellow / Gold */
#define COLOR_BG_ARC        lv_color_hex(0x2D2D2D)     /* Dark gray track */
#define COLOR_TEXT          lv_color_hex(0xFFFFFF)
#define COLOR_TEXT_DIM      lv_color_hex(0x888888)

/* ---- Foreground arc handles (set_value targets) ---- */
static lv_obj_t *arc_oil_pressure;
static lv_obj_t *arc_coolant_temp;
static lv_obj_t *arc_oil_temp;

/* ---- Value label handles ---- */
static lv_obj_t *label_oil_pressure_value;
static lv_obj_t *label_coolant_temp_value;
static lv_obj_t *label_oil_temp_value;

/* ---- Helpers ---- */

/** Map a float value within [min, max] to an arc angle in [0, 270]. */
static int32_t value_to_arc_angle(float value, float min, float max)
{
    float normalized = (value - min) / (max - min);
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    return (int32_t)(normalized * 270);
}

/**
 * Create a background + foreground arc pair.
 *
 * Background: dark track, 270-degree sweep, never changes.
 * Foreground: colored INDICATOR only (MAIN part fully transparent),
 *             value driven by lv_arc_set_value().
 */
static void create_arc_gauge(lv_obj_t *parent, lv_obj_t **arc_fg,
                              int32_t radius, lv_color_t color)
{
    /* --- Background arc (static dark track) --- */
    lv_obj_t *arc_bg = lv_arc_create(parent);
    lv_obj_set_size(arc_bg, radius * 2, radius * 2);
    lv_obj_center(arc_bg);
    lv_arc_set_rotation(arc_bg, 135);
    lv_arc_set_bg_angles(arc_bg, 0, 270);
    lv_arc_set_value(arc_bg, 0);
    lv_obj_remove_style(arc_bg, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc_bg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(arc_bg, COLOR_BG_ARC, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc_bg, ARC_WIDTH, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc_bg, false, LV_PART_MAIN);

    /* --- Foreground arc (colored indicator) --- */
    *arc_fg = lv_arc_create(parent);
    lv_obj_set_size(*arc_fg, radius * 2, radius * 2);
    lv_obj_center(*arc_fg);
    lv_arc_set_rotation(*arc_fg, 135);
    lv_arc_set_bg_angles(*arc_fg, 0, 270);
    lv_arc_set_range(*arc_fg, 0, 270);
    lv_arc_set_value(*arc_fg, 0);
    lv_obj_remove_style(*arc_fg, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(*arc_fg, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(*arc_fg, color, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(*arc_fg, ARC_WIDTH, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(*arc_fg, false, LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(*arc_fg, LV_OPA_TRANSP, LV_PART_MAIN);
}

/** Create a title + value label pair stacked vertically at y_offset. */
static void create_value_label(lv_obj_t *parent, lv_obj_t **label,
                                const char *title, lv_color_t color,
                                int32_t y_offset)
{
    lv_obj_t *lbl_title = lv_label_create(parent);
    lv_label_set_text(lbl_title, title);
    lv_obj_set_style_text_color(lbl_title, COLOR_TEXT_DIM, 0);
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_title, LV_ALIGN_CENTER, 0, y_offset - 12);

    *label = lv_label_create(parent);
    lv_label_set_text(*label, "--");
    lv_obj_set_style_text_color(*label, color, 0);
    lv_obj_set_style_text_font(*label, &lv_font_montserrat_20, 0);
    lv_obj_align(*label, LV_ALIGN_CENTER, 0, y_offset + 8);
}

/* ---- Public API ---- */

void ui_dashboard_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    /* Arcs: outer → inner */
    create_arc_gauge(scr, &arc_oil_pressure, ARC_OIL_PRESS_RADIUS, COLOR_OIL_PRESSURE);
    create_arc_gauge(scr, &arc_coolant_temp, ARC_COOLANT_RADIUS,   COLOR_COOLANT);
    create_arc_gauge(scr, &arc_oil_temp,     ARC_OIL_TEMP_RADIUS,  COLOR_OIL_TEMP);

    /* Center title */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "ENGINE");
    lv_obj_set_style_text_color(title, COLOR_TEXT, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -80);

    /* Value labels in center area */
    create_value_label(scr, &label_oil_pressure_value, "OIL PRESS", COLOR_OIL_PRESSURE, -30);
    create_value_label(scr, &label_coolant_temp_value, "COOLANT",   COLOR_COOLANT,        30);
    create_value_label(scr, &label_oil_temp_value,     "OIL TEMP",  COLOR_OIL_TEMP,        90);

    /* NaN → arcs at 0, labels show "--" until real data arrives */
    ui_dashboard_set_oil_pressure(NAN);
    ui_dashboard_set_coolant_temp(NAN);
    ui_dashboard_set_oil_temp(NAN);
}

void ui_dashboard_set_oil_pressure(float bar)
{
    if (isnan(bar)) {
        lv_arc_set_value(arc_oil_pressure, 0);
        lv_label_set_text_static(label_oil_pressure_value, "--");
        return;
    }
    lv_arc_set_value(arc_oil_pressure,
                     value_to_arc_angle(bar, OIL_PRESSURE_MIN, OIL_PRESSURE_MAX));

    static char buf[16];
    snprintf(buf, sizeof(buf), "%.1f bar", (double)bar);
    lv_label_set_text_static(label_oil_pressure_value, buf);
}

void ui_dashboard_set_coolant_temp(float celsius)
{
    if (isnan(celsius)) {
        lv_arc_set_value(arc_coolant_temp, 0);
        lv_label_set_text_static(label_coolant_temp_value, "--");
        return;
    }
    lv_arc_set_value(arc_coolant_temp,
                     value_to_arc_angle(celsius, COOLANT_TEMP_MIN, COOLANT_TEMP_MAX));

    static char buf[16];
    snprintf(buf, sizeof(buf), "%.0f C", (double)celsius);
    lv_label_set_text_static(label_coolant_temp_value, buf);
}

void ui_dashboard_set_oil_temp(float celsius)
{
    if (isnan(celsius)) {
        lv_arc_set_value(arc_oil_temp, 0);
        lv_label_set_text_static(label_oil_temp_value, "--");
        return;
    }
    lv_arc_set_value(arc_oil_temp,
                     value_to_arc_angle(celsius, OIL_TEMP_MIN, OIL_TEMP_MAX));

    static char buf[16];
    snprintf(buf, sizeof(buf), "%.0f C", (double)celsius);
    lv_label_set_text_static(label_oil_temp_value, buf);
}
