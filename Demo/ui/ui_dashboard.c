#include "ui_dashboard.h"
#include <stdio.h>
#include <math.h>

#define SCREEN_SIZE     466
#define CENTER_X        (SCREEN_SIZE / 2)
#define CENTER_Y        (SCREEN_SIZE / 2)

/* Arc dimensions (from outer to inner) */
#define ARC_WIDTH           30
#define ARC_GAP             8
#define ARC_OIL_PRESS_RADIUS    (SCREEN_SIZE / 2 - 20)
#define ARC_COOLANT_RADIUS      (ARC_OIL_PRESS_RADIUS - ARC_WIDTH - ARC_GAP)
#define ARC_OIL_TEMP_RADIUS     (ARC_COOLANT_RADIUS - ARC_WIDTH - ARC_GAP)

/* Value ranges */
#define OIL_PRESSURE_MIN    0.0f
#define OIL_PRESSURE_MAX    10.0f   /* bar */
#define COOLANT_TEMP_MIN    40.0f
#define COOLANT_TEMP_MAX    120.0f  /* celsius */
#define OIL_TEMP_MIN        40.0f
#define OIL_TEMP_MAX        150.0f  /* celsius */

/* Colors */
#define COLOR_OIL_PRESSURE  lv_color_hex(0x00BFFF)  /* Deep Sky Blue */
#define COLOR_COOLANT       lv_color_hex(0xFF6B6B)  /* Red/Coral */
#define COLOR_OIL_TEMP      lv_color_hex(0xFFD93D)  /* Yellow/Gold */
#define COLOR_BG_ARC        lv_color_hex(0x2D2D2D)  /* Dark gray */
#define COLOR_TEXT          lv_color_hex(0xFFFFFF)  /* White */
#define COLOR_TEXT_DIM      lv_color_hex(0x888888)  /* Gray */

/* UI elements */
static lv_obj_t *arc_oil_pressure;
static lv_obj_t *arc_coolant_temp;
static lv_obj_t *arc_oil_temp;

static lv_obj_t *label_oil_pressure_value;
static lv_obj_t *label_coolant_temp_value;
static lv_obj_t *label_oil_temp_value;

static lv_obj_t *label_center_title;

/* Current values */
static float current_oil_pressure = 0.0f;
static float current_coolant_temp = 40.0f;
static float current_oil_temp = 40.0f;

/* Demo mode */
static bool demo_mode_enabled = false;
static lv_timer_t *demo_timer = NULL;

static int32_t value_to_arc_angle(float value, float min, float max)
{
    float normalized = (value - min) / (max - min);
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    /* Arc goes from 135 to 45 degrees (270 degree sweep) */
    return (int32_t)(normalized * 270);
}

static void create_arc_gauge(lv_obj_t *parent, lv_obj_t **arc, int32_t radius, lv_color_t color)
{
    /* Background arc */
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

    /* Foreground arc (indicator) */
    *arc = lv_arc_create(parent);
    lv_obj_set_size(*arc, radius * 2, radius * 2);
    lv_obj_center(*arc);
    lv_arc_set_rotation(*arc, 135);
    lv_arc_set_bg_angles(*arc, 0, 270);
    lv_arc_set_range(*arc, 0, 270);
    lv_arc_set_value(*arc, 0);
    lv_obj_remove_style(*arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(*arc, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_set_style_arc_color(*arc, color, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(*arc, ARC_WIDTH, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(*arc, false, LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(*arc, LV_OPA_TRANSP, LV_PART_MAIN);
}

static void create_value_label(lv_obj_t *parent, lv_obj_t **label,
                                const char *title, lv_color_t color, int32_t y_offset)
{
    /* Title label */
    lv_obj_t *lbl_title = lv_label_create(parent);
    lv_label_set_text(lbl_title, title);
    lv_obj_set_style_text_color(lbl_title, COLOR_TEXT_DIM, 0);
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_title, LV_ALIGN_CENTER, 0, y_offset - 12);

    /* Value label */
    *label = lv_label_create(parent);
    lv_label_set_text(*label, "--");
    lv_obj_set_style_text_color(*label, color, 0);
    lv_obj_set_style_text_font(*label, &lv_font_montserrat_20, 0);
    lv_obj_align(*label, LV_ALIGN_CENTER, 0, y_offset + 8);
}

static void demo_timer_cb(lv_timer_t *timer)
{
    static float phase = 0.0f;
    phase += 0.05f;
    if (phase > 6.28318f) phase = 0.0f;

    /* Simulate varying values using sine waves with different phases */
    float oil_press = 3.0f + 2.5f * sinf(phase);
    float coolant = 85.0f + 15.0f * sinf(phase * 0.7f + 1.0f);
    float oil_temp = 95.0f + 25.0f * sinf(phase * 0.5f + 2.0f);

    ui_dashboard_set_oil_pressure(oil_press);
    ui_dashboard_set_coolant_temp(coolant);
    ui_dashboard_set_oil_temp(oil_temp);
}

void ui_dashboard_init(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);

    /* Create arc gauges (outer to inner) */
    create_arc_gauge(screen, &arc_oil_pressure, ARC_OIL_PRESS_RADIUS, COLOR_OIL_PRESSURE);
    create_arc_gauge(screen, &arc_coolant_temp, ARC_COOLANT_RADIUS, COLOR_COOLANT);
    create_arc_gauge(screen, &arc_oil_temp, ARC_OIL_TEMP_RADIUS, COLOR_OIL_TEMP);

    /* Center title */
    label_center_title = lv_label_create(screen);
    lv_label_set_text(label_center_title, "ENGINE");
    lv_obj_set_style_text_color(label_center_title, COLOR_TEXT, 0);
    lv_obj_set_style_text_font(label_center_title, &lv_font_montserrat_20, 0);
    lv_obj_align(label_center_title, LV_ALIGN_CENTER, 0, -80);

    /* Value labels in center area */
    create_value_label(screen, &label_oil_pressure_value, "OIL PRESS", COLOR_OIL_PRESSURE, -30);
    create_value_label(screen, &label_coolant_temp_value, "COOLANT", COLOR_COOLANT, 30);
    create_value_label(screen, &label_oil_temp_value, "OIL TEMP", COLOR_OIL_TEMP, 90);

    /* Set initial values */
    ui_dashboard_set_oil_pressure(0.0f);
    ui_dashboard_set_coolant_temp(40.0f);
    ui_dashboard_set_oil_temp(40.0f);
}

void ui_dashboard_set_oil_pressure(float bar)
{
    current_oil_pressure = bar;
    int32_t angle = value_to_arc_angle(bar, OIL_PRESSURE_MIN, OIL_PRESSURE_MAX);
    lv_arc_set_value(arc_oil_pressure, angle);

    static char buf[16];
    snprintf(buf, sizeof(buf), "%.1f bar", bar);
    lv_label_set_text(label_oil_pressure_value, buf);
}

void ui_dashboard_set_coolant_temp(float celsius)
{
    current_coolant_temp = celsius;
    int32_t angle = value_to_arc_angle(celsius, COOLANT_TEMP_MIN, COOLANT_TEMP_MAX);
    lv_arc_set_value(arc_coolant_temp, angle);

    static char buf[16];
    snprintf(buf, sizeof(buf), "%.0f C", celsius);
    lv_label_set_text(label_coolant_temp_value, buf);
}

void ui_dashboard_set_oil_temp(float celsius)
{
    current_oil_temp = celsius;
    int32_t angle = value_to_arc_angle(celsius, OIL_TEMP_MIN, OIL_TEMP_MAX);
    lv_arc_set_value(arc_oil_temp, angle);

    static char buf[16];
    snprintf(buf, sizeof(buf), "%.0f C", celsius);
    lv_label_set_text(label_oil_temp_value, buf);
}

void ui_dashboard_demo_mode(bool enable)
{
    demo_mode_enabled = enable;

    if (enable && demo_timer == NULL) {
        demo_timer = lv_timer_create(demo_timer_cb, 50, NULL);
    } else if (!enable && demo_timer != NULL) {
        lv_timer_del(demo_timer);
        demo_timer = NULL;
    }
}
