#ifndef INVENT_EMS_H
#define INVENT_EMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * Invent Labs EMS Dashboard Protocol parser
 *
 * UART: 19200 bps, 8N1
 * Packet: [0x55 0x00 0xAA 0x00] [version] [length] [payload...] [CRC16-LE]
 * Protocol version: 0x54 (v5.4)
 *
 * Payload = TInfoPacket: fast data + rotating slow packet (1 of 10 types)
 */

#define INVENT_EMS_BAUD_RATE      19200
#define INVENT_EMS_PROTOCOL_VER   0x54

/* Accumulated ECU data with engineering-unit conversions */
typedef struct {
    /* Connection status */
    bool connected;
    uint32_t packet_count;
    uint32_t error_count;

    /* ---- Fast data (updated every packet ~50 Hz) ---- */
    float rpm;
    float ign_angle;        /* degrees, 0.25 resolution */
    float inj_time_ms;      /* injector pulse width */
    float tps;              /* throttle position 0-100 % */
    float dbw_pos;          /* electronic throttle actual 0-100 % */
    float map_kpa;          /* manifold pressure */
    float lambda;           /* wideband lambda */
    float speed;            /* vehicle speed km/h */
    float fuel_flow;        /* instantaneous fuel flow (1/16 units) */
    float knock_v;          /* knock sensor voltage */
    int8_t transient_corr;  /* transient fuel correction */
    uint8_t runlevel;
    uint8_t cyl_no;

    /* ---- Slow0: corrections & electrical ---- */
    int8_t corr_angle;
    float lambda_target;
    int8_t lambda_corr_fast;
    int8_t lambda_corr_slow;
    float fuel_pressure_kpa;
    float dwell_ms;
    float voltage;          /* battery voltage V */
    int8_t gear;
    float dbw_cmd;          /* DBW commanded position */
    float lambda2;

    /* ---- Slow1: flags & boost ---- */
    uint8_t flag_major;
    uint8_t flag_minor;
    uint8_t flag_notify;
    uint8_t flag_notify2;
    uint8_t flag_protection;
    float idle_pos;         /* idle valve 0-100 % */
    uint16_t airflow;
    uint8_t boost_duty;
    uint8_t boost_target;

    /* ---- Slow2: injection details ---- */
    uint8_t egr_pos;
    uint8_t egr_target;
    uint8_t inj_duty;       /* injection duty cycle % */
    int16_t inj_lag_time;
    int8_t inj_end_angle;
    uint8_t fuel_press_coef;
    int8_t air_charge_t;
    int8_t inj_air_charge_corr;
    uint8_t speed2;
    float back_pressure_kpa;

    /* ---- Slow3: VVT & traction ---- */
    int16_t ign_accel_corr;
    int8_t vvt1_curr;
    int8_t vvt1_target;
    int8_t vvt2_curr;
    int8_t vvt2_target;
    int8_t vvt1b_curr;
    int8_t vvt2b_curr;
    uint8_t tcs_corr;
    float pwm3d_target;
    float pwm3d_curr;

    /* ---- Slow4: trip computer ---- */
    float trip_fuel_l;       /* 0.01 L resolution */
    float trip_path_km;      /* 0.1 km resolution */
    float curr_fuel_cons;    /* L/100km */
    float trip_fuel_cons;    /* L/100km */
    float fuel_composition;  /* ethanol % */

    /* ---- Slow5: raw ADC ---- */
    uint8_t adc_tps;
    uint8_t adc_ct;
    uint8_t adc_iat;
    uint8_t adc_dbw1;
    uint8_t adc_dbw2;
    uint8_t adc_map;
    uint8_t adc_lambda;

    /* ---- Slow6: analog inputs ADC ---- */
    uint8_t adc_an[10];

    /* ---- Slow7: I/O state ---- */
    uint8_t input_state;
    uint16_t output_state;
    uint8_t dbw_driver_status;
    uint8_t dbw_system_status;
    uint8_t gas_state;
    int8_t at_temp;
    uint8_t at_state;
    uint8_t fuel_level;

    /* ---- Slow8: temperatures & pressures (KEY for dashboard) ---- */
    float clt;              /* coolant temp C */
    float iat;              /* intake air temp C */
    float oil_temp;         /* oil temp C */
    float fuel_temp;        /* fuel temp C */
    float egt1;             /* exhaust gas temp 1 */
    float egt2;             /* exhaust gas temp 2 */
    float oil_pressure;     /* bar (0.1 resolution) */

    /* ---- Slow9: PWM outputs ---- */
    float pwm_duty[6];     /* PWM channels 1-6, 0-100 % */

} invent_ems_data_t;

/* Initialize the parser (call once at startup) */
void invent_ems_init(void);

/* Feed one byte from UART into the parser state machine */
void invent_ems_feed_byte(uint8_t byte);

/* Get pointer to the latest accumulated ECU data (always valid) */
const invent_ems_data_t *invent_ems_get_data(void);

/* Feed one CAN frame (DBC decode into ECU data). Returns true if ID was recognized. */
bool invent_ems_feed_can_frame(uint32_t id, const uint8_t *data, uint8_t dlc);

/* Returns true once after each successfully parsed packet (auto-clears) */
bool invent_ems_has_new_data(void);

#ifdef __cplusplus
}
#endif

#endif /* INVENT_EMS_H */
