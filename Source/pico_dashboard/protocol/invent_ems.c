#include "invent_ems.h"
#include <string.h>
#include <math.h>

/*
 * Protocol wire format (after UART byte stream):
 *
 *   Header:  0x55 0x00 0xAA 0x00
 *   Version: 0x54
 *   Payload: rxbuf[0]=Length, rxbuf[1..Length-2]=data, rxbuf[Length-1..Length]=CRC16-LE
 *
 * CRC16 (CCITT variant, init 0xFFFF) covers rxbuf[0] through rxbuf[Length-2].
 *
 * TInfoPacket layout in rxbuf (packed, little-endian):
 *   [0]  Length          uint8   (= sizeof(TInfoPacket) + 1, typically 37)
 *   [1]  Type            uint8
 *   [2]  Runlevel        uint8
 *   [3]  Uoz             int16   (ign angle, *0.25 deg)
 *   [5]  Rashod          uint8   (fuel flow, *1/16)
 *   [6]  Period          uint16  (RPM = 10000000/Period)
 *   [8]  InjTime         uint16  (*0.004 ms)
 *   [10] KnockVoltage    uint8   (*5/256 V)
 *   [11] Tps             uint8   (*100/255 %)
 *   [12] DbwCurrPos      uint8   (*100/255 %)
 *   [13] MapKpa          uint8   (*2 kPa)
 *   [14] Lambda          uint8   (*1/128)
 *   [15] CylNo           uint8
 *   [16] TransientCorr   int8
 *   [17] Speed           uint8   (km/h)
 *   [18] KnockVolPerCyl  uint8
 *   [19] KnockRetPerCyl  uint8
 *   [20] TmrDifPerCyl    int8
 *   [21] Debug1          uint8
 *   [22] Debug2          int16
 *   [24] SlowPacketId    uint8   (0-9)
 *   [25] SlowPacket[11]          (11 bytes, type determined by SlowPacketId)
 *   [36] CRC16 low
 *   [37] CRC16 high
 */

/* ---- Protocol constants ---- */
#define HEADER_0     0x55
#define HEADER_1     0x00
#define HEADER_2     0xAA
#define HEADER_3     0x00

#define MAX_RX_BUF   64
#define MAX_PACKET_LEN  48   /* max valid Length field */
#define MIN_PACKET_LEN   4   /* minimum: type + at least 1 data + CRC16 */

#define SLOW_PACKET_OFFSET  25
#define SLOW_PACKET_SIZE    11
#define SLOW_PACKET_COUNT   10

/* ---- Parser state ---- */
static uint8_t rxstate;
static uint8_t rxbuf[MAX_RX_BUF];
static uint8_t rxptr;

/* ---- Data ---- */
static invent_ems_data_t ecu_data;
static volatile bool new_data_flag;

/* ---- Helpers ---- */
static inline int16_t read_i16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint16_t read_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* ---- CRC16-CCITT (matches Invent EMS firmware) ---- */
static uint16_t checksum(const uint8_t *buf)
{
    uint8_t len = buf[0] - 2;  /* CRC covers buf[0] through buf[len] */
    if (len >= MAX_RX_BUF - 2) return 0;

    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i <= len; i++) {
        uint8_t d = buf[i];
        d ^= (uint8_t)(crc & 0xFF);
        d ^= (uint8_t)(d << 4);
        uint16_t t = ((uint16_t)d << 8) | ((crc >> 8) & 0xFF);
        t ^= (uint8_t)(d >> 4);
        t ^= (uint16_t)d << 3;
        crc = t;
    }
    return crc;
}

/* ---- Fast data parsing ---- */
static void parse_fast(const uint8_t *buf)
{
    ecu_data.runlevel = buf[2];

    ecu_data.ign_angle    = read_i16(&buf[3]) * 0.25f;
    ecu_data.fuel_flow    = buf[5] * (1.0f / 16.0f);

    uint16_t period = read_u16(&buf[6]);
    ecu_data.rpm = (period > 0) ? (10000000.0f / period) : 0.0f;

    ecu_data.inj_time_ms  = read_u16(&buf[8]) * 0.004f;
    ecu_data.knock_v      = buf[10] * (5.0f / 256.0f);
    ecu_data.tps          = buf[11] * (100.0f / 255.0f);
    ecu_data.dbw_pos      = buf[12] * (100.0f / 255.0f);
    ecu_data.map_kpa      = buf[13] * 2.0f;
    ecu_data.lambda       = buf[14] * (1.0f / 128.0f);
    ecu_data.cyl_no       = buf[15];
    ecu_data.transient_corr = (int8_t)buf[16];
    ecu_data.speed        = buf[17];
}

/* ---- Slow packet parsing ---- */
static void parse_slow(uint8_t id, const uint8_t *s)
{
    switch (id) {
    case 0: /* corrections & electrical */
        ecu_data.corr_angle      = (int8_t)s[0];
        ecu_data.lambda_target   = s[1] * (1.0f / 128.0f);
        ecu_data.lambda_corr_fast = (int8_t)s[2];
        ecu_data.lambda_corr_slow = (int8_t)s[3];
        ecu_data.fuel_pressure_kpa = (float)read_u16(&s[4]);
        ecu_data.dwell_ms        = s[6];
        ecu_data.voltage         = s[7] * 0.1f;
        ecu_data.gear            = (int8_t)s[8];
        ecu_data.dbw_cmd         = s[9];
        ecu_data.lambda2         = s[10] * (1.0f / 128.0f);
        break;

    case 1: /* flags & boost */
        ecu_data.flag_major      = s[0];
        ecu_data.flag_minor      = s[1];
        ecu_data.flag_notify     = s[2];
        ecu_data.flag_notify2    = s[3];
        ecu_data.flag_protection = s[4];
        ecu_data.idle_pos        = s[5] * (100.0f / 256.0f);
        ecu_data.airflow         = read_u16(&s[6]);
        ecu_data.boost_duty      = s[8];
        ecu_data.boost_target    = s[9];
        break;

    case 2: /* injection details */
        ecu_data.egr_pos         = s[0];
        ecu_data.egr_target      = s[1];
        ecu_data.inj_duty        = s[2];
        ecu_data.inj_lag_time    = read_i16(&s[3]);
        ecu_data.inj_end_angle   = (int8_t)s[5];
        ecu_data.fuel_press_coef = s[6];
        ecu_data.air_charge_t    = (int8_t)s[7];
        ecu_data.inj_air_charge_corr = (int8_t)s[8];
        ecu_data.speed2          = s[9];
        ecu_data.back_pressure_kpa = s[10] * 2.0f;
        break;

    case 3: /* VVT & traction */
        ecu_data.ign_accel_corr  = read_i16(&s[0]);
        ecu_data.vvt1_curr       = (int8_t)s[2];
        ecu_data.vvt1_target     = (int8_t)s[3];
        ecu_data.vvt2_curr       = (int8_t)s[4];
        ecu_data.vvt2_target     = (int8_t)s[5];
        ecu_data.vvt1b_curr      = (int8_t)s[6];
        ecu_data.vvt2b_curr      = (int8_t)s[7];
        ecu_data.tcs_corr        = s[8];
        ecu_data.pwm3d_target    = s[9] * (100.0f / 256.0f);
        ecu_data.pwm3d_curr      = s[10] * (100.0f / 256.0f);
        break;

    case 4: /* trip computer */
        ecu_data.trip_fuel_l     = read_u16(&s[0]) * 0.01f;
        ecu_data.trip_path_km    = read_u16(&s[2]) * 0.1f;
        ecu_data.curr_fuel_cons  = read_u16(&s[4]) * 0.1f;
        ecu_data.trip_fuel_cons  = read_u16(&s[6]) * 0.1f;
        ecu_data.fuel_composition = s[8] * (100.0f / 256.0f);
        break;

    case 5: /* raw ADC */
        ecu_data.adc_tps    = s[0];
        ecu_data.adc_ct     = s[1];
        ecu_data.adc_iat    = s[2];
        ecu_data.adc_dbw1   = s[3];
        ecu_data.adc_dbw2   = s[4];
        ecu_data.adc_map    = s[5];
        ecu_data.adc_lambda = s[6];
        break;

    case 6: /* analog inputs ADC */
        for (int i = 0; i < 10 && i < SLOW_PACKET_SIZE; i++)
            ecu_data.adc_an[i] = s[i];
        break;

    case 7: /* I/O state */
        ecu_data.input_state       = s[0];
        ecu_data.output_state      = read_u16(&s[1]);
        ecu_data.dbw_driver_status = s[3];
        ecu_data.dbw_system_status = s[4];
        ecu_data.gas_state         = s[5];
        ecu_data.at_temp           = (int8_t)s[6];
        ecu_data.at_state          = s[7];
        ecu_data.fuel_level        = s[8];
        break;

    case 8: /* temperatures & pressures */
        ecu_data.clt          = (int8_t)s[0];
        ecu_data.iat          = (int8_t)s[1];
        ecu_data.oil_temp     = s[2];
        ecu_data.fuel_temp    = (int8_t)s[3];
        /* s[4] = _free */
        ecu_data.egt1         = (float)read_u16(&s[5]);
        ecu_data.egt2         = (float)read_u16(&s[7]);
        ecu_data.oil_pressure = s[9] * 0.1f;
        break;

    case 9: /* PWM duties */
        for (int i = 0; i < 6; i++)
            ecu_data.pwm_duty[i] = s[i] * (100.0f / 256.0f);
        break;
    }
}

/* ---- Full packet parsing ---- */
static void parse_packet(void)
{
    parse_fast(rxbuf);

    uint8_t slow_id = rxbuf[24];
    if (slow_id < SLOW_PACKET_COUNT) {
        parse_slow(slow_id, &rxbuf[SLOW_PACKET_OFFSET]);
    }

    ecu_data.connected = true;
    ecu_data.packet_count++;
    new_data_flag = true;
}

/* ---- Public API ---- */

void invent_ems_init(void)
{
    memset(&ecu_data, 0, sizeof(ecu_data));

    /* All floats start as NaN — "no data yet" */
    ecu_data.rpm = NAN;
    ecu_data.ign_angle = NAN;
    ecu_data.inj_time_ms = NAN;
    ecu_data.tps = NAN;
    ecu_data.dbw_pos = NAN;
    ecu_data.map_kpa = NAN;
    ecu_data.lambda = NAN;
    ecu_data.speed = NAN;
    ecu_data.fuel_flow = NAN;
    ecu_data.knock_v = NAN;

    ecu_data.lambda_target = NAN;
    ecu_data.fuel_pressure_kpa = NAN;
    ecu_data.dwell_ms = NAN;
    ecu_data.voltage = NAN;
    ecu_data.dbw_cmd = NAN;
    ecu_data.lambda2 = NAN;

    ecu_data.idle_pos = NAN;
    ecu_data.back_pressure_kpa = NAN;

    ecu_data.pwm3d_target = NAN;
    ecu_data.pwm3d_curr = NAN;

    ecu_data.trip_fuel_l = NAN;
    ecu_data.trip_path_km = NAN;
    ecu_data.curr_fuel_cons = NAN;
    ecu_data.trip_fuel_cons = NAN;
    ecu_data.fuel_composition = NAN;

    ecu_data.clt = NAN;
    ecu_data.iat = NAN;
    ecu_data.oil_temp = NAN;
    ecu_data.fuel_temp = NAN;
    ecu_data.egt1 = NAN;
    ecu_data.egt2 = NAN;
    ecu_data.oil_pressure = NAN;

    for (int i = 0; i < 6; i++)
        ecu_data.pwm_duty[i] = NAN;

    rxstate = 0;
    rxptr = 0;
    new_data_flag = false;
}

void invent_ems_feed_byte(uint8_t byte)
{
    bool retry;
    do {
        retry = false;
        switch (rxstate) {
        case 0:
            if (byte == HEADER_0) rxstate++;
            break;
        case 1:
            if (byte == HEADER_1) rxstate++;
            else { rxstate = 0; retry = true; }
            break;
        case 2:
            if (byte == HEADER_2) rxstate++;
            else { rxstate = 0; retry = true; }
            break;
        case 3:
            if (byte == HEADER_3) rxstate++;
            else { rxstate = 0; retry = true; }
            break;
        case 4: /* protocol version */
            if (byte == INVENT_EMS_PROTOCOL_VER) rxstate++;
            else { rxstate = 0; retry = true; }
            break;
        case 5: /* length byte */
            if (byte >= MIN_PACKET_LEN && byte <= MAX_PACKET_LEN) {
                rxbuf[0] = byte;
                rxptr = 1;
                rxstate++;
            } else {
                rxstate = 0;
            }
            break;
        case 6: /* payload + CRC */
            if (rxptr < MAX_RX_BUF) {
                rxbuf[rxptr] = byte;
            }
            rxptr++;
            if (rxptr > rxbuf[0]) {
                /* Full packet received — verify CRC */
                uint16_t crc_rx   = (uint16_t)rxbuf[rxptr - 2]
                                  | ((uint16_t)rxbuf[rxptr - 1] << 8);
                uint16_t crc_calc = checksum(rxbuf);
                if (crc_calc == crc_rx) {
                    parse_packet();
                } else {
                    ecu_data.error_count++;
                }
                rxstate = 0;
            }
            break;
        default:
            rxstate = 0;
        }
    } while (retry);
}

const invent_ems_data_t *invent_ems_get_data(void)
{
    return &ecu_data;
}

/* ---- CAN DBC decode (matches InventEmu TX encoding) ---- */
bool invent_ems_feed_can_frame(uint32_t id, const uint8_t *d, uint8_t dlc)
{
    (void)dlc;
    switch (id) {
    case 0x300: /* RPM, TPS, MAP, IAT */
        ecu_data.rpm     = (float)read_u16(&d[0]);
        ecu_data.tps     = read_i16(&d[2]) * 0.1f;
        ecu_data.map_kpa = read_u16(&d[4]) * 0.01f;
        ecu_data.iat     = read_i16(&d[6]) * 0.1f;
        break;
    case 0x302: /* IgnAngle, Dwell, InjAngle, InjPW */
        ecu_data.ign_angle   = read_i16(&d[0]) * 0.1f;
        ecu_data.dwell_ms    = read_u16(&d[2]) * 0.1f;
        ecu_data.inj_time_ms = read_u16(&d[6]) * 0.001f;
        break;
    case 0x304: /* OilT, OilP, CLT, VBAT */
        ecu_data.oil_temp     = read_i16(&d[0]) * 0.1f;
        ecu_data.oil_pressure = read_i16(&d[2]) * 0.1f / 100.0f;
        ecu_data.clt          = read_i16(&d[4]) * 0.1f;
        ecu_data.voltage      = read_i16(&d[6]) * 0.1f;
        break;
    case 0x305: /* Gear, MapTarget, Speed, EvtMask */
        ecu_data.gear  = (int8_t)read_i16(&d[0]);
        ecu_data.speed = read_u16(&d[4]) * 0.1f;
        break;
    case 0x306: /* Knock1, Knock2, FuelP, FuelT */
        ecu_data.knock_v           = read_i16(&d[0]) * 0.1f;
        ecu_data.fuel_pressure_kpa = read_u16(&d[4]) * 0.1f;
        ecu_data.fuel_temp         = read_i16(&d[6]) * 0.1f;
        break;
    case 0x307: /* EGT1, EGT2 */
        ecu_data.egt1 = read_i16(&d[0]) * 0.1f;
        ecu_data.egt2 = read_i16(&d[2]) * 0.1f;
        break;
    case 0x340: /* Vehicle speed */
        ecu_data.speed = read_u16(&d[0]) * 0.1f;
        break;
    default:
        return false;
    }

    ecu_data.connected = true;
    new_data_flag = true;
    return true;
}

bool invent_ems_has_new_data(void)
{
    if (new_data_flag) {
        new_data_flag = false;
        return true;
    }
    return false;
}
