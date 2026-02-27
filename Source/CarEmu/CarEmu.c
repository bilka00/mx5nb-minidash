/*
 * Invent EMS Protocol TX Emulator
 * RP2040 (Pico SDK) — UART1 (GP4=TX, GP5=RX)
 *
 * Generates TInfoPacket with rotating slow packets
 * for testing the parser/receiver side.
 *
 * USB CDC serial used for debug/commands (printf).
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "can2040.h"

// ============================================================
// Type aliases
// ============================================================
typedef uint8_t  uint8;
typedef int8_t   int8;
typedef uint16_t uint16;
typedef int16_t  int16;

// ============================================================
// Protocol constants
// ============================================================
#define PROTOCOL_VERSION    0x54
#define SLOW_PACKET_SIZE    11
#define SLOW_PACKETS_COUNT  4
#define VERY_SLOW_PACKETS_COUNT 6
#define TOTAL_SLOW_PACKETS  (SLOW_PACKETS_COUNT + VERY_SLOW_PACKETS_COUNT) // 10

// Header: 0x55 0x00 0xAA 0x00 <version>
static const uint8_t HEADER[] = { 0x55, 0x00, 0xAA, 0x00, PROTOCOL_VERSION };
#define HEADER_SIZE 5

// UART1 pins & baud
#define UART_ID      uart1
#define UART_TX_PIN  4
#define UART_RX_PIN  5
#define UART_BAUD    19200

// CAN bus (PIO0, SN65HVD230 transceiver)
#define CAN_PIO_NUM    0
#define CAN_BITRATE    500000
#define CAN_GPIO_TX    21
#define CAN_GPIO_RX    22

// Packet send interval (ms)
#define TX_INTERVAL_MS 20  // ~50 Hz main packet rate

// ============================================================
// Packed structs (mirroring packet_info.h)
// ============================================================
#pragma pack(push, 1)

// --- Flag unions ---

typedef union {
    uint8 All;
    struct {
        uint8 DPKV     : 1,
              DF       : 1,
              DBW      : 1,
              DAD      : 1,
              DNO      : 1,
              CLT      : 1,
              OILP     : 1,
              FUELP    : 1;
    };
} TFlagMajor;

typedef union {
    uint8 All;
    struct {
        uint8 Lambda : 1,
              IAT    : 1,
              FuelT  : 1,
              OilT   : 1,
              VVT    : 1,
              EGR    : 1;
    };
} TFlagMinor;

typedef union {
    uint8 All;
    struct {
        uint8 TableSwitch      : 1,
              InjTimeLimit     : 1,
              Marker           : 1,
              AfterStartEnrich : 1,
              PhasedMode       : 1,
              WritePending     : 1,
              Methanol         : 1,
              IdleCutoff       : 1;
    };
} TFlagNotify;

typedef union {
    uint8 All;
    struct {
        uint8 Launch    : 1,
              FlatShift : 1,
              AntiLag   : 1;
    };
} TFlagNotify2;

typedef union {
    uint8 All;
    struct {
        uint8 Rpm       : 1,
              Overboost : 1,
              Lambda    : 1,
              InjDuty   : 1,
              Clt       : 1,
              Egt       : 1,
              OilT      : 1;
    };
} TFlagProtection;

typedef union {
    uint8 All;
    struct {
        uint8 PartLock  : 1,
              FullLock  : 1,
              Shifting  : 1,
              Retard    : 1,
              __free    : 1,
              Selector  : 3;
    };
} TATState;

// --- Slow packet structs (all padded to SLOW_PACKET_SIZE = 11 bytes) ---

typedef struct {
    int8   CorrAngle;
    uint8  LambdaTarget;
    int8   LambdaCorrFast;
    int8   LambdaCorrSlow;
    uint16 FuelPKpa;
    uint8  DwellTime;
    uint8  Voltage;
    int8   GearNo;
    uint8  DbwCommandedPos;
    uint8  Lambda2;
} TInfoPacketSlow0;

typedef struct {
    TFlagMajor      FlagMajor;
    TFlagMinor      FlagMinor;
    TFlagNotify     FlagNotify;
    TFlagNotify2    FlagNotify2;
    TFlagProtection FlagProtection;
    uint8  IdlePos;
    uint16 Airflow;
    uint8  BoostDuty;
    uint8  BoostTarget;
    uint8  _free;
} TInfoPacketSlow1;

typedef struct {
    uint8  EgrCurrentPos;
    uint8  EgrTargetPos;
    uint8  InjDutyCycle;
    int16  InjLagTime;
    int8   InjEndAngle4;
    uint8  FuelPressureCoef;
    int8   AirChargeT;
    int8   InjAirChargeCorr;
    uint8  Speed2;
    uint8  BackPKpa;
} TInfoPacketSlow2;

typedef struct {
    int16  IgnAccelCorrDelta;
    int8   Vvt1CurrAngle;
    int8   Vvt1TargetAngle;
    int8   Vvt2CurrAngle;
    int8   Vvt2TargetAngle;
    int8   Vvt1bCurrAngle;
    int8   Vvt2bCurrAngle;
    uint8  TcsCorr;
    uint8  Pwm3DTarget;
    uint8  Pwm3DCurr;
} TInfoPacketSlow3;

typedef struct {
    uint16 TripFuel;
    uint16 TripPath;
    uint16 CurrFuelCons;
    uint16 TripFuelCons;
    uint8  FuelComposition;
    uint8  free2;
    uint8  _free;
} TInfoPacketSlow4;

typedef struct {
    uint8  AdcTps;
    uint8  AdcCt;
    uint8  AdcIat;
    uint8  DbwAdc1;
    uint8  DbwAdc2;
    uint8  AdcMap;
    uint8  AdcLambda;
    uint8  SlotNo;
    uint8  SlotLatency;
    uint8  SlotTime;
    uint8  _free;
} TInfoPacketSlow5;

typedef struct {
    uint8  AdcAn1;
    uint8  AdcAn2;
    uint8  AdcAn3;
    uint8  AdcAn4;
    uint8  AdcAn5;
    uint8  AdcAn6;
    uint8  AdcAn7;
    uint8  AdcAn8;
    uint8  AdcAn9;
    uint8  AdcAn10;
    uint8  _free;
} TInfoPacketSlow6;

typedef struct {
    uint8  InputState;
    uint16 OutputState;
    uint8  DbwDriverStatus;
    uint8  DbwSystemStatus;
    uint8  GasState;
    int8   ATTemp;
    uint8  ATState;
    uint8  FuelLevel;
    uint8  padding4;
    uint8  _free;
} TInfoPacketSlow7;

typedef struct {
    int8   Clt;
    int8   Iat;
    uint8  OilT;
    int8   FuelT;
    uint8  _free;
    uint16 Egt1;
    uint16 Egt2;
    uint8  OilP;
    uint8  _free2;
} TInfoPacketSlow8;

typedef struct {
    uint8  Pwm1Duty;
    uint8  Pwm2Duty;
    uint8  Pwm3Duty;
    uint8  Pwm4Duty;
    uint8  Pwm5Duty;
    uint8  Pwm6Duty;
    uint8  _free;
    uint8  _free2;
    uint8  _free3;
    uint8  _free4;
    uint8  _free5;
} TInfoPacketSlow9;

// Slow packet union
typedef union {
    TInfoPacketSlow0 Pkt0;
    TInfoPacketSlow1 Pkt1;
    TInfoPacketSlow2 Pkt2;
    TInfoPacketSlow3 Pkt3;
    TInfoPacketSlow4 Pkt4;
    TInfoPacketSlow5 Pkt5;
    TInfoPacketSlow6 Pkt6;
    TInfoPacketSlow7 Pkt7;
    TInfoPacketSlow8 Pkt8;
    TInfoPacketSlow9 Pkt9;
    uint8 raw[SLOW_PACKET_SIZE];
} TSlowPacket;

// Main info packet
typedef struct {
    uint8       Length;
    uint8       Type;
    uint8       Runlevel;
    int16       Uoz;
    uint8       Rashod;
    uint16      Period;
    uint16      InjTime;
    uint8       KnockVoltage;
    uint8       Tps;
    uint8       DbwCurrPos;
    uint8       MapKpa;
    uint8       Lambda;
    uint8       CylNo;
    int8        TransientCorr;
    uint8       Speed;
    uint8       KnockVoltagePerCyl;
    uint8       KnockRetardPerCyl;
    int8        TmrDifPerCyl;
    uint8       Debug1;
    int16       Debug2;
    uint8       SlowPacketId;
    TSlowPacket SlowPacket;
} TInfoPacket;

#pragma pack(pop)

// Compile-time struct size checks
_Static_assert(sizeof(TInfoPacketSlow0) == SLOW_PACKET_SIZE, "Slow0 size mismatch");
_Static_assert(sizeof(TInfoPacketSlow1) == SLOW_PACKET_SIZE, "Slow1 size mismatch");
_Static_assert(sizeof(TInfoPacketSlow2) == SLOW_PACKET_SIZE, "Slow2 size mismatch");
_Static_assert(sizeof(TInfoPacketSlow3) == SLOW_PACKET_SIZE, "Slow3 size mismatch");
_Static_assert(sizeof(TInfoPacketSlow4) == SLOW_PACKET_SIZE, "Slow4 size mismatch");
_Static_assert(sizeof(TInfoPacketSlow5) == SLOW_PACKET_SIZE, "Slow5 size mismatch");
_Static_assert(sizeof(TInfoPacketSlow6) == SLOW_PACKET_SIZE, "Slow6 size mismatch");
_Static_assert(sizeof(TInfoPacketSlow7) == SLOW_PACKET_SIZE, "Slow7 size mismatch");
_Static_assert(sizeof(TInfoPacketSlow8) == SLOW_PACKET_SIZE, "Slow8 size mismatch");
_Static_assert(sizeof(TInfoPacketSlow9) == SLOW_PACKET_SIZE, "Slow9 size mismatch");
_Static_assert(sizeof(TSlowPacket)      == SLOW_PACKET_SIZE, "SlowPacket union size mismatch");

// ============================================================
// CRC-16 (matches the receiver's Checksum function exactly)
// ============================================================
static uint16_t calcChecksum(const uint8_t* buffer)
{
    uint8_t len = buffer[0] - 2;
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i <= len; i++)
    {
        uint8_t data = buffer[i];
        data ^= crc & 0xFF;
        data ^= data << 4;
        uint16_t t = ((uint16_t)data << 8) | ((crc >> 8) & 0xFF);
        t ^= (uint8_t)(data >> 4);
        t ^= (uint16_t)data << 3;
        crc = t;
    }
    return crc;
}

// ============================================================
// Simulated engine state
// ============================================================
typedef struct {
    // Fast channels
    float   rpm;
    float   angleDeg;
    float   tpsPercent;
    float   dbwPercent;
    float   mapKpa;
    float   lambdaVal;
    float   injTimeMs;
    float   rashodLH;
    uint8_t knockV;
    uint8_t cylNo;
    int8_t  transCorr;
    uint8_t speed;
    uint8_t runlevel;

    // Slow0
    int8_t   corrAngle;
    float    lambdaTarget;
    int8_t   lambdaCorrFast;
    int8_t   lambdaCorrSlow;
    uint16_t fuelPKpa;
    uint8_t  dwellTime;
    float    voltageV;
    int8_t   gearNo;
    uint8_t  dbwCmdPos;
    float    lambda2Val;

    // Slow1
    uint8_t  flagMajor;
    uint8_t  flagMinor;
    uint8_t  flagNotify;
    uint8_t  flagNotify2;
    uint8_t  flagProtection;
    float    idlePosPercent;
    uint16_t airflow;
    uint8_t  boostDuty;
    uint8_t  boostTarget;

    // Slow2
    uint8_t  egrCurrPos;
    uint8_t  egrTargetPos;
    uint8_t  injDutyCycle;
    int16_t  injLagTime;
    int8_t   injEndAngle4;
    uint8_t  fuelPressCoef;
    int8_t   airChargeT;
    int8_t   injAirChargeCorr;
    uint8_t  speed2;
    float    backPKpa;

    // Slow3
    int16_t  ignAccelCorr;
    int8_t   vvt1Curr;
    int8_t   vvt1Target;
    int8_t   vvt2Curr;
    int8_t   vvt2Target;
    int8_t   vvt1bCurr;
    int8_t   vvt2bCurr;
    uint8_t  tcsCorr;
    float    pwm3dTarget;
    float    pwm3dCurr;

    // Slow4
    float    tripFuelL;
    float    tripPathKm;
    float    currFuelCons;
    float    tripFuelCons;
    float    fuelCompPct;

    // Slow5
    float    adcTpsV;
    float    adcCtV;
    float    adcIatV;
    float    dbwAdc1V;
    float    dbwAdc2V;
    float    adcMapV;
    float    adcLambdaV;

    // Slow6
    float    adcAn[10];

    // Slow7
    uint8_t  inputState;
    uint16_t outputState;
    uint8_t  dbwDriverStatus;
    uint8_t  dbwSystemStatus;
    uint8_t  gasState;
    int8_t   atTemp;
    uint8_t  atState;
    uint8_t  fuelLevel;

    // Slow8
    int8_t   clt;
    int8_t   iat;
    uint8_t  oilT;
    int8_t   fuelT;
    uint16_t egt1;
    uint16_t egt2;
    float    oilPBar;

    // Manual override flags (skip simulation when set via serial)
    bool     cltOverride;
    bool     oilTOverride;
    bool     oilPOverride;

    // Slow9
    float    pwmDuty[6];

    // CAN DBC extra fields
    float    afrTarget;
    uint16_t rpmHardLimit;
    uint16_t knockEvsCnt;
    float    mapTargetKpa;
} EngineState;

static EngineState eng;

static void initEngineState(void)
{
    memset(&eng, 0, sizeof(eng));

    // Fast channels
    eng.rpm        = 850.0f;
    eng.angleDeg   = 10.0f;
    eng.tpsPercent = 5.0f;
    eng.dbwPercent = 5.0f;
    eng.mapKpa     = 35.0f;
    eng.lambdaVal  = 1.0f;
    eng.injTimeMs  = 2.5f;
    eng.rashodLH   = 1.5f;
    eng.knockV     = 10;
    eng.runlevel   = 2;

    // Slow0
    eng.corrAngle      = -2;
    eng.lambdaTarget   = 1.0f;
    eng.lambdaCorrFast = 3;
    eng.lambdaCorrSlow = -1;
    eng.fuelPKpa       = 300;
    eng.dwellTime      = 35;
    eng.voltageV       = 14.1f;
    eng.dbwCmdPos      = 12;
    eng.lambda2Val     = 1.02f;

    // Slow1
    eng.flagNotify     = 0x10; // PhasedMode
    eng.idlePosPercent = 30.0f;
    eng.airflow        = 120;
    eng.boostTarget    = 50;

    // Slow2
    eng.injDutyCycle   = 15;
    eng.injLagTime     = 120;
    eng.injEndAngle4   = -20;
    eng.fuelPressCoef  = 100;
    eng.airChargeT     = 25;
    eng.backPKpa       = 101.0f;

    // Slow3
    eng.vvt1Curr       = 5;
    eng.vvt1Target     = 10;

    // Slow4
    eng.tripFuelL      = 3.25f;
    eng.tripPathKm     = 42.5f;
    eng.currFuelCons   = 7.6f;
    eng.tripFuelCons   = 7.6f;

    // Slow5
    eng.adcTpsV    = 0.8f;
    eng.adcCtV     = 2.1f;
    eng.adcIatV    = 2.5f;
    eng.dbwAdc1V   = 0.8f;
    eng.dbwAdc2V   = 3.2f;
    eng.adcMapV    = 1.0f;
    eng.adcLambdaV = 0.45f;

    // Slow6
    for (int i = 0; i < 10; i++)
        eng.adcAn[i] = 1.0f;

    // Slow7
    eng.atTemp    = 60;
    eng.fuelLevel = 128;

    // Slow8
    eng.clt    = 85;
    eng.iat    = 30;
    eng.oilT   = 95;
    eng.fuelT  = 35;
    eng.egt1   = 650;
    eng.egt2   = 640;
    eng.oilPBar = 3.5f;

    // Slow9
    eng.pwmDuty[0] = 50.0f;

    // CAN DBC extra
    eng.afrTarget    = 14.7f;
    eng.rpmHardLimit = 8000;
    eng.knockEvsCnt  = 0;
    eng.mapTargetKpa = 100.0f;
}

// ============================================================
// Helpers
// ============================================================

static uint16_t rpmToPeriod(float rpm)
{
    if (rpm <= 0) return 0;
    uint32_t p = (uint32_t)(10000000.0f / rpm);
    return (p > 65535) ? 65535 : (uint16_t)p;
}

static uint8_t clampU8(float v)
{
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)(v + 0.5f);
}

// ============================================================
// Fill slow packets
// ============================================================

static void fillSlow0(TInfoPacketSlow0* p)
{
    p->CorrAngle      = eng.corrAngle;
    p->LambdaTarget   = clampU8(eng.lambdaTarget * 128.0f);
    p->LambdaCorrFast = eng.lambdaCorrFast;
    p->LambdaCorrSlow = eng.lambdaCorrSlow;
    p->FuelPKpa       = eng.fuelPKpa;
    p->DwellTime      = eng.dwellTime;
    p->Voltage        = clampU8(eng.voltageV / 0.1f);
    p->GearNo         = eng.gearNo;
    p->DbwCommandedPos = eng.dbwCmdPos;
    p->Lambda2        = clampU8(eng.lambda2Val * 128.0f);
}

static void fillSlow1(TInfoPacketSlow1* p)
{
    p->FlagMajor.All      = eng.flagMajor;
    p->FlagMinor.All      = eng.flagMinor;
    p->FlagNotify.All     = eng.flagNotify;
    p->FlagNotify2.All    = eng.flagNotify2;
    p->FlagProtection.All = eng.flagProtection;
    p->IdlePos            = clampU8(eng.idlePosPercent * 256.0f / 100.0f);
    p->Airflow            = eng.airflow;
    p->BoostDuty          = eng.boostDuty;
    p->BoostTarget        = eng.boostTarget;
    p->_free              = 0;
}

static void fillSlow2(TInfoPacketSlow2* p)
{
    p->EgrCurrentPos    = eng.egrCurrPos;
    p->EgrTargetPos     = eng.egrTargetPos;
    p->InjDutyCycle     = eng.injDutyCycle;
    p->InjLagTime       = eng.injLagTime;
    p->InjEndAngle4     = eng.injEndAngle4;
    p->FuelPressureCoef = eng.fuelPressCoef;
    p->AirChargeT       = eng.airChargeT;
    p->InjAirChargeCorr = eng.injAirChargeCorr;
    p->Speed2           = eng.speed2;
    p->BackPKpa         = clampU8(eng.backPKpa / 2.0f);
}

static void fillSlow3(TInfoPacketSlow3* p)
{
    p->IgnAccelCorrDelta = eng.ignAccelCorr;
    p->Vvt1CurrAngle    = eng.vvt1Curr;
    p->Vvt1TargetAngle  = eng.vvt1Target;
    p->Vvt2CurrAngle    = eng.vvt2Curr;
    p->Vvt2TargetAngle  = eng.vvt2Target;
    p->Vvt1bCurrAngle   = eng.vvt1bCurr;
    p->Vvt2bCurrAngle   = eng.vvt2bCurr;
    p->TcsCorr          = eng.tcsCorr;
    p->Pwm3DTarget      = clampU8(eng.pwm3dTarget * 256.0f / 100.0f);
    p->Pwm3DCurr        = clampU8(eng.pwm3dCurr * 256.0f / 100.0f);
}

static void fillSlow4(TInfoPacketSlow4* p)
{
    p->TripFuel        = (uint16_t)(eng.tripFuelL / 0.01f);
    p->TripPath        = (uint16_t)(eng.tripPathKm / 0.1f);
    p->CurrFuelCons    = (uint16_t)(eng.currFuelCons / 0.1f);
    p->TripFuelCons    = (uint16_t)(eng.tripFuelCons / 0.1f);
    p->FuelComposition = clampU8(eng.fuelCompPct * 256.0f / 100.0f);
    p->free2           = 0;
    p->_free           = 0;
}

static void fillSlow5(TInfoPacketSlow5* p)
{
    p->AdcTps    = clampU8(eng.adcTpsV * 256.0f / 5.0f);
    p->AdcCt     = clampU8(eng.adcCtV * 256.0f / 5.0f);
    p->AdcIat    = clampU8(eng.adcIatV * 256.0f / 5.0f);
    p->DbwAdc1   = clampU8(eng.dbwAdc1V * 256.0f / 5.0f);
    p->DbwAdc2   = clampU8(eng.dbwAdc2V * 256.0f / 5.0f);
    p->AdcMap    = clampU8(eng.adcMapV * 256.0f / 5.0f);
    p->AdcLambda = clampU8(eng.adcLambdaV * 256.0f / 5.0f);
    p->SlotNo      = 0;
    p->SlotLatency = 0;
    p->SlotTime    = 0;
    p->_free       = 0;
}

static void fillSlow6(TInfoPacketSlow6* p)
{
    uint8_t* an = &p->AdcAn1;
    for (int i = 0; i < 10; i++)
        an[i] = clampU8(eng.adcAn[i] * 256.0f / 5.0f);
    p->_free = 0;
}

static void fillSlow7(TInfoPacketSlow7* p)
{
    p->InputState       = eng.inputState;
    p->OutputState      = eng.outputState;
    p->DbwDriverStatus  = eng.dbwDriverStatus;
    p->DbwSystemStatus  = eng.dbwSystemStatus;
    p->GasState         = eng.gasState;
    p->ATTemp           = eng.atTemp;
    p->ATState          = eng.atState;
    p->FuelLevel        = eng.fuelLevel;
    p->padding4         = 0;
    p->_free            = 0;
}

static void fillSlow8(TInfoPacketSlow8* p)
{
    p->Clt    = eng.clt;
    p->Iat    = eng.iat;
    p->OilT   = eng.oilT;
    p->FuelT  = eng.fuelT;
    p->_free  = 0;
    p->Egt1   = eng.egt1;
    p->Egt2   = eng.egt2;
    p->OilP   = clampU8(eng.oilPBar / 0.1f);
    p->_free2 = 0;
}

static void fillSlow9(TInfoPacketSlow9* p)
{
    p->Pwm1Duty = clampU8(eng.pwmDuty[0] * 256.0f / 100.0f);
    p->Pwm2Duty = clampU8(eng.pwmDuty[1] * 256.0f / 100.0f);
    p->Pwm3Duty = clampU8(eng.pwmDuty[2] * 256.0f / 100.0f);
    p->Pwm4Duty = clampU8(eng.pwmDuty[3] * 256.0f / 100.0f);
    p->Pwm5Duty = clampU8(eng.pwmDuty[4] * 256.0f / 100.0f);
    p->Pwm6Duty = clampU8(eng.pwmDuty[5] * 256.0f / 100.0f);
    p->_free  = 0;
    p->_free2 = 0;
    p->_free3 = 0;
    p->_free4 = 0;
    p->_free5 = 0;
}

// ============================================================
// Build and send one complete packet
// ============================================================

static uint8_t txbuf[sizeof(TInfoPacket) + 2];
static uint8_t slowPacketIndex = 0;

static void buildAndSend(void)
{
    TInfoPacket* pkt = (TInfoPacket*)txbuf;
    memset(txbuf, 0, sizeof(txbuf));

    // Length field: sizeof(TInfoPacket) + 1
    pkt->Length = sizeof(TInfoPacket) + 1;

    pkt->Type     = 0x01;
    pkt->Runlevel = eng.runlevel;

    // Fast channels
    pkt->Uoz       = (int16_t)(eng.angleDeg / 0.25f);
    pkt->Rashod     = clampU8(eng.rashodLH * 16.0f);
    pkt->Period     = rpmToPeriod(eng.rpm);
    pkt->InjTime    = (uint16_t)(eng.injTimeMs / 0.004f);
    pkt->KnockVoltage = eng.knockV;
    pkt->Tps        = clampU8(eng.tpsPercent * 255.0f / 100.0f);
    pkt->DbwCurrPos = clampU8(eng.dbwPercent * 255.0f / 100.0f);
    pkt->MapKpa     = clampU8(eng.mapKpa / 2.0f);
    pkt->Lambda     = clampU8(eng.lambdaVal * 128.0f);
    pkt->CylNo      = eng.cylNo;
    pkt->TransientCorr = eng.transCorr;
    pkt->Speed      = eng.speed;
    pkt->KnockVoltagePerCyl = 0;
    pkt->KnockRetardPerCyl  = 0;
    pkt->TmrDifPerCyl       = 0;
    pkt->Debug1     = 0;
    pkt->Debug2     = 0;

    // Slow packet
    pkt->SlowPacketId = slowPacketIndex;
    memset(&pkt->SlowPacket, 0, SLOW_PACKET_SIZE);

    switch (slowPacketIndex) {
        case 0: fillSlow0(&pkt->SlowPacket.Pkt0); break;
        case 1: fillSlow1(&pkt->SlowPacket.Pkt1); break;
        case 2: fillSlow2(&pkt->SlowPacket.Pkt2); break;
        case 3: fillSlow3(&pkt->SlowPacket.Pkt3); break;
        case 4: fillSlow4(&pkt->SlowPacket.Pkt4); break;
        case 5: fillSlow5(&pkt->SlowPacket.Pkt5); break;
        case 6: fillSlow6(&pkt->SlowPacket.Pkt6); break;
        case 7: fillSlow7(&pkt->SlowPacket.Pkt7); break;
        case 8: fillSlow8(&pkt->SlowPacket.Pkt8); break;
        case 9: fillSlow9(&pkt->SlowPacket.Pkt9); break;
    }

    // CRC
    uint16_t crc = calcChecksum(txbuf);
    uint8_t crcOffset = sizeof(TInfoPacket);
    txbuf[crcOffset]     = crc & 0xFF;
    txbuf[crcOffset + 1] = (crc >> 8) & 0xFF;

    // Transmit: header + payload + CRC
    uart_write_blocking(UART_ID, HEADER, HEADER_SIZE);
    uart_write_blocking(UART_ID, txbuf, sizeof(TInfoPacket) + 2);

    // Advance slow packet index
    slowPacketIndex++;
    if (slowPacketIndex >= TOTAL_SLOW_PACKETS)
        slowPacketIndex = 0;
}

// ============================================================
// Simple engine simulation
// ============================================================
static float simTime = 0.0f;

static void simulateEngine(float dt)
{
    simTime += dt;

    eng.rpm = 850.0f + 30.0f * sinf(simTime * 0.5f);
    eng.tpsPercent = 5.0f + 1.0f * sinf(simTime * 0.3f);
    eng.dbwPercent = eng.tpsPercent;
    eng.mapKpa = 35.0f + 3.0f * sinf(simTime * 0.4f);
    eng.lambdaVal = 1.0f + 0.02f * sinf(simTime * 2.0f);
    eng.lambda2Val = eng.lambdaVal;
    eng.injTimeMs = 2.5f + 0.3f * sinf(simTime * 0.5f);
    eng.angleDeg = 12.0f + 2.0f * sinf(simTime * 0.6f);
    eng.voltageV = 14.0f + 0.2f * sinf(simTime * 1.5f);
    eng.lambdaCorrFast = (int8_t)(5.0f * sinf(simTime * 2.0f));
    eng.lambdaCorrSlow = (int8_t)(2.0f * sinf(simTime * 0.2f));

    // CLT: warm-up from initial value towards 90 C, then oscillate around 88
    if (!eng.cltOverride) {
        if (simTime < 180.0f) {
            float target = 90.0f;
            float alpha = 1.0f - expf(-simTime / 60.0f);
            eng.clt = (int8_t)(eng.clt * (1.0f - alpha * dt * 0.05f) + target * (alpha * dt * 0.05f));
            if (eng.clt > 90) eng.clt = 90;
        } else {
            eng.clt = (int8_t)(88.0f + 2.0f * sinf(simTime * 0.1f));
        }
    }

    // Oil temp: follows CLT but ~10 C higher, lags behind
    if (!eng.oilTOverride) {
        float oilTarget = (float)eng.clt + 10.0f + 3.0f * sinf(simTime * 0.08f);
        if (oilTarget < 0) oilTarget = 0;
        if (oilTarget > 150) oilTarget = 150;
        eng.oilT = (uint8_t)(eng.oilT + (oilTarget - (float)eng.oilT) * dt * 0.02f);
    }

    // Oil pressure: depends on RPM, drops slightly when hot
    if (!eng.oilPOverride) {
        eng.oilPBar = 3.0f + 0.5f * (eng.rpm / 1000.0f) - 0.1f * sinf(simTime * 0.15f);
        if (eng.oilPBar < 0.5f) eng.oilPBar = 0.5f;
    }

    eng.cylNo = (uint8_t)((uint32_t)(simTime * eng.rpm / 60.0f * 2.0f) % 4);
    eng.tripFuelL += eng.rashodLH * dt / 3600.0f;
}

// ============================================================
// CAN bus (can2040 on PIO0)
// ============================================================

static struct can2040 cbus;
static volatile bool canRunning = false;
static volatile uint32_t canTxOk = 0;
static volatile uint32_t canErrors = 0;

static void can2040_cb(struct can2040 *cd, uint32_t notify,
                       struct can2040_msg *msg)
{
    (void)msg;
    if (notify & CAN2040_NOTIFY_TX) {
        canTxOk++;
        canErrors = 0;  // reset error streak on success
    }
    if (notify & CAN2040_NOTIFY_ERROR) {
        canErrors++;
        if (canErrors > 50) {
            // Bus is dead (no transceiver / no ACK) — stop PIO to free CPU
            can2040_stop(cd);
            canRunning = false;
        }
    }
}

static void PIO0_IRQ0_Handler(void)
{
    can2040_pio_irq_handler(&cbus);
}

static void canbus_setup(void)
{
    can2040_setup(&cbus, CAN_PIO_NUM);
    can2040_callback_config(&cbus, can2040_cb);

    irq_set_exclusive_handler(PIO0_IRQ_0, PIO0_IRQ0_Handler);
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, true);

    can2040_start(&cbus, clock_get_hz(clk_sys), CAN_BITRATE,
                  CAN_GPIO_RX, CAN_GPIO_TX);
    canRunning = true;
    canErrors = 0;
    canTxOk = 0;
}

// --- CAN frame builders (DBC: ME1_4.dbc, all 8-byte LE) ---

static void buildCAN_ME1_1(struct can2040_msg *msg)
{
    msg->id  = 0x300;
    msg->dlc = 8;
    memset(msg->data, 0, 8);
    uint16_t rpm = (uint16_t)eng.rpm;
    int16_t  tps = (int16_t)(eng.tpsPercent / 0.1f);
    uint16_t map = (uint16_t)(eng.mapKpa / 0.01f);
    int16_t  iat = (int16_t)((float)eng.iat / 0.1f);
    memcpy(&msg->data[0], &rpm, 2);
    memcpy(&msg->data[2], &tps, 2);
    memcpy(&msg->data[4], &map, 2);
    memcpy(&msg->data[6], &iat, 2);
}

static void buildCAN_ME1_2(struct can2040_msg *msg)
{
    msg->id  = 0x301;
    msg->dlc = 8;
    memset(msg->data, 0, 8);
    uint16_t rpmLimit = eng.rpmHardLimit;
    int16_t  afr1 = (int16_t)(eng.lambdaVal * 14.7f / 0.01f);
    int16_t  afr2 = (int16_t)(eng.lambda2Val * 14.7f / 0.01f);
    int16_t  lambdaTrim = (int16_t)(eng.lambdaCorrFast / 0.1f);
    memcpy(&msg->data[0], &rpmLimit, 2);
    memcpy(&msg->data[2], &afr1, 2);
    memcpy(&msg->data[4], &afr2, 2);
    memcpy(&msg->data[6], &lambdaTrim, 2);
}

static void buildCAN_ME1_3(struct can2040_msg *msg)
{
    msg->id  = 0x302;
    msg->dlc = 8;
    memset(msg->data, 0, 8);
    int16_t  ignAdv  = (int16_t)(eng.angleDeg / 0.1f);
    uint16_t dwell   = (uint16_t)(eng.dwellTime * 10);  // 0.1 ms units
    int16_t  injAng  = (int16_t)(eng.injEndAngle4 / 0.1f);
    uint16_t injPw   = (uint16_t)(eng.injTimeMs / 0.001f);
    memcpy(&msg->data[0], &ignAdv, 2);
    memcpy(&msg->data[2], &dwell, 2);
    memcpy(&msg->data[4], &injAng, 2);
    memcpy(&msg->data[6], &injPw, 2);
}

static void buildCAN_ME1_4(struct can2040_msg *msg)
{
    msg->id  = 0x303;
    msg->dlc = 8;
    memset(msg->data, 0, 8);
    uint16_t priDuty = (uint16_t)(eng.injDutyCycle / 0.1f);
    uint16_t secDuty = 0;
    int16_t  secAng  = 0;
    uint16_t boostD  = (uint16_t)(eng.boostDuty / 0.1f);
    memcpy(&msg->data[0], &priDuty, 2);
    memcpy(&msg->data[2], &secDuty, 2);
    memcpy(&msg->data[4], &secAng, 2);
    memcpy(&msg->data[6], &boostD, 2);
}

static void buildCAN_ME1_5(struct can2040_msg *msg)
{
    msg->id  = 0x304;
    msg->dlc = 8;
    memset(msg->data, 0, 8);
    int16_t  oilT = (int16_t)((float)eng.oilT / 0.1f);
    int16_t  oilP = (int16_t)(eng.oilPBar * 100.0f / 0.1f);  // bar→kPa→raw
    int16_t  clt  = (int16_t)((float)eng.clt / 0.1f);
    int16_t  vbat = (int16_t)(eng.voltageV / 0.1f);
    memcpy(&msg->data[0], &oilT, 2);
    memcpy(&msg->data[2], &oilP, 2);
    memcpy(&msg->data[4], &clt, 2);
    memcpy(&msg->data[6], &vbat, 2);
}

static void buildCAN_ME1_6(struct can2040_msg *msg)
{
    msg->id  = 0x305;
    msg->dlc = 8;
    memset(msg->data, 0, 8);
    int16_t  gear   = (int16_t)eng.gearNo;
    uint16_t mapTgt = (uint16_t)(eng.mapTargetKpa / 0.01f);
    uint16_t speed  = (uint16_t)(eng.speed / 0.1f);
    uint16_t evsMsk = 0;
    memcpy(&msg->data[0], &gear, 2);
    memcpy(&msg->data[2], &mapTgt, 2);
    memcpy(&msg->data[4], &speed, 2);
    memcpy(&msg->data[6], &evsMsk, 2);
}

static void buildCAN_ME1_7(struct can2040_msg *msg)
{
    msg->id  = 0x306;
    msg->dlc = 8;
    memset(msg->data, 0, 8);
    int16_t  knock1 = (int16_t)(eng.knockV / 0.1f);
    int16_t  knock2 = knock1;
    uint16_t fuelP  = (uint16_t)(eng.fuelPKpa / 0.1f);
    int16_t  fuelT  = (int16_t)((float)eng.fuelT / 0.1f);
    memcpy(&msg->data[0], &knock1, 2);
    memcpy(&msg->data[2], &knock2, 2);
    memcpy(&msg->data[4], &fuelP, 2);
    memcpy(&msg->data[6], &fuelT, 2);
}

static void buildCAN_ME1_8(struct can2040_msg *msg)
{
    msg->id  = 0x307;
    msg->dlc = 8;
    memset(msg->data, 0, 8);
    int16_t egt1 = (int16_t)(eng.egt1 / 0.1f);
    int16_t egt2 = (int16_t)(eng.egt2 / 0.1f);
    int16_t gpt1 = 0;
    int16_t gpt2 = 0;
    memcpy(&msg->data[0], &egt1, 2);
    memcpy(&msg->data[2], &egt2, 2);
    memcpy(&msg->data[4], &gpt1, 2);
    memcpy(&msg->data[6], &gpt2, 2);
}

static void buildCAN_ME1_In1(struct can2040_msg *msg)
{
    msg->id  = 0x340;
    msg->dlc = 8;
    memset(msg->data, 0, 8);
    uint16_t vspeed = (uint16_t)(eng.speed / 0.1f);
    memcpy(&msg->data[0], &vspeed, 2);
}

typedef void (*can_builder_t)(struct can2040_msg *);

static const can_builder_t canBuilders[] = {
    buildCAN_ME1_1, buildCAN_ME1_2, buildCAN_ME1_3,
    buildCAN_ME1_4, buildCAN_ME1_5, buildCAN_ME1_6,
    buildCAN_ME1_7, buildCAN_ME1_8, buildCAN_ME1_In1,
};
#define CAN_MSG_COUNT (sizeof(canBuilders) / sizeof(canBuilders[0]))

static void sendAllCAN(void)
{
    if (!canRunning)
        return;
    struct can2040_msg msg;
    for (unsigned i = 0; i < CAN_MSG_COUNT; i++) {
        canBuilders[i](&msg);
        /* Wait for TX queue space — at 500 kbps each frame ≈ 260 µs,
           worst case 4 frames draining ≈ 1 ms, well within 20 ms cycle */
        while (can2040_transmit(&cbus, &msg) < 0)
            tight_loop_contents();
    }
}

// ============================================================
// Serial command interface (USB CDC via stdio)
// ============================================================
#define CMD_BUF_SIZE 64
static char cmdBuf[CMD_BUF_SIZE];
static int  cmdPos = 0;

static void parseCommand(const char* cmd)
{
    if (strncmp(cmd, "rpm=", 4) == 0) {
        eng.rpm = (float)atof(cmd + 4);
        printf("RPM set to %.0f\n", eng.rpm);
    }
    else if (strncmp(cmd, "tps=", 4) == 0) {
        eng.tpsPercent = (float)atof(cmd + 4);
        eng.dbwPercent = eng.tpsPercent;
        printf("TPS set to %.1f%%\n", eng.tpsPercent);
    }
    else if (strncmp(cmd, "map=", 4) == 0) {
        eng.mapKpa = (float)atof(cmd + 4);
        printf("MAP set to %.1f kPa\n", eng.mapKpa);
    }
    else if (strncmp(cmd, "clt=", 4) == 0) {
        eng.clt = (int8_t)atoi(cmd + 4);
        eng.cltOverride = true;
        printf("CLT set to %d C (override)\n", eng.clt);
    }
    else if (strncmp(cmd, "iat=", 4) == 0) {
        eng.iat = (int8_t)atoi(cmd + 4);
        printf("IAT set to %d C\n", eng.iat);
    }
    else if (strncmp(cmd, "fuelt=", 6) == 0) {
        eng.fuelT = (int8_t)atoi(cmd + 6);
        printf("Fuel temp set to %d C\n", eng.fuelT);
    }
    else if (strncmp(cmd, "speed=", 6) == 0) {
        eng.speed = (uint8_t)atoi(cmd + 6);
        eng.speed2 = eng.speed;
        printf("Speed set to %d km/h\n", eng.speed);
    }
    else if (strncmp(cmd, "lambda=", 7) == 0) {
        eng.lambdaVal = (float)atof(cmd + 7);
        printf("Lambda set to %.3f\n", eng.lambdaVal);
    }
    else if (strncmp(cmd, "angle=", 6) == 0) {
        eng.angleDeg = (float)atof(cmd + 6);
        printf("Angle set to %.1f deg\n", eng.angleDeg);
    }
    else if (strncmp(cmd, "runlevel=", 9) == 0) {
        eng.runlevel = (uint8_t)atoi(cmd + 9);
        printf("Runlevel set to %d\n", eng.runlevel);
    }
    else if (strncmp(cmd, "gear=", 5) == 0) {
        eng.gearNo = (int8_t)atoi(cmd + 5);
        printf("Gear set to %d\n", eng.gearNo);
    }
    else if (strncmp(cmd, "egt1=", 5) == 0) {
        eng.egt1 = (uint16_t)atoi(cmd + 5);
        printf("EGT1 set to %d C\n", eng.egt1);
    }
    else if (strncmp(cmd, "egt2=", 5) == 0) {
        eng.egt2 = (uint16_t)atoi(cmd + 5);
        printf("EGT2 set to %d C\n", eng.egt2);
    }
    else if (strncmp(cmd, "fault=", 6) == 0) {
        eng.flagMajor = (uint8_t)strtol(cmd + 6, NULL, 16);
        printf("FlagMajor set to 0x%02X\n", eng.flagMajor);
    }
    else if (strncmp(cmd, "oilp=", 5) == 0) {
        eng.oilPBar = (float)atof(cmd + 5);
        eng.oilPOverride = true;
        printf("Oil pressure set to %.1f bar (override)\n", eng.oilPBar);
    }
    else if (strncmp(cmd, "oilt=", 5) == 0) {
        eng.oilT = (uint8_t)atoi(cmd + 5);
        eng.oilTOverride = true;
        printf("Oil temp set to %d C (override)\n", eng.oilT);
    }
    else if (strncmp(cmd, "vvt1=", 5) == 0) {
        eng.vvt1Target = (int8_t)atoi(cmd + 5);
        printf("VVT1 target set to %d deg\n", eng.vvt1Target);
    }
    else if (strcmp(cmd, "sim") == 0) {
        printf("Simulation mode: engine values change automatically\n");
    }
    else if (strcmp(cmd, "status") == 0) {
        printf("RPM=%.0f TPS=%.1f%% MAP=%.1f CLT=%d LAMBDA=%.3f ANGLE=%.1f\n",
               eng.rpm, eng.tpsPercent, eng.mapKpa, eng.clt, eng.lambdaVal, eng.angleDeg);
        printf("OilP=%.1fbar OilT=%dC IAT=%dC FuelT=%dC EGT1=%d EGT2=%d\n",
               eng.oilPBar, eng.oilT, eng.iat, eng.fuelT, eng.egt1, eng.egt2);
        printf("SlowPktIdx=%d  sizeof(TInfoPacket)=%d\n",
               slowPacketIndex, (int)sizeof(TInfoPacket));
        printf("CAN: %s  tx_ok=%lu  errors=%lu\n",
               canRunning ? "RUNNING" : "STOPPED",
               (unsigned long)canTxOk, (unsigned long)canErrors);
    }
    else if (strcmp(cmd, "canstat") == 0) {
        struct can2040_stats stats;
        can2040_get_statistics(&cbus, &stats);
        printf("CAN %s  tx_ok=%lu  errors=%lu\n",
               canRunning ? "RUNNING" : "STOPPED",
               (unsigned long)canTxOk, (unsigned long)canErrors);
        printf("  lib: tx=%lu rx=%lu attempt=%lu parse_err=%lu\n",
               (unsigned long)stats.tx_total, (unsigned long)stats.rx_total,
               (unsigned long)stats.tx_attempt, (unsigned long)stats.parse_error);
    }
    else if (strcmp(cmd, "canstart") == 0) {
        if (canRunning) {
            printf("CAN already running\n");
        } else {
            canbus_setup();
            printf("CAN restarted\n");
        }
    }
    else if (strcmp(cmd, "canstop") == 0) {
        if (canRunning) {
            can2040_stop(&cbus);
            canRunning = false;
            printf("CAN stopped\n");
        } else {
            printf("CAN already stopped\n");
        }
    }
    else if (strcmp(cmd, "help") == 0) {
        printf("Commands: rpm=N tps=N map=N clt=N speed=N lambda=N angle=N\n");
        printf("          runlevel=N gear=N egt1=N egt2=N fault=HH vvt1=N\n");
        printf("          oilp=N oilt=N iat=N fuelt=N\n");
        printf("          canstat  canstart  canstop\n");
        printf("          status  help\n");
    }
    else {
        printf("Unknown command. Type 'help'.\n");
    }
}

static void processSerialCommands(void)
{
    int ch = getchar_timeout_us(0);
    while (ch != PICO_ERROR_TIMEOUT) {
        if (ch == '\n' || ch == '\r') {
            if (cmdPos > 0) {
                cmdBuf[cmdPos] = '\0';
                parseCommand(cmdBuf);
                cmdPos = 0;
            }
        } else if (cmdPos < CMD_BUF_SIZE - 1) {
            cmdBuf[cmdPos++] = (char)ch;
        }
        ch = getchar_timeout_us(0);
    }
}

// ============================================================
// Main
// ============================================================
int main(void)
{
    // Init USB stdio
    stdio_init_all();

    // Init UART1
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Init engine state
    initEngineState();

    // CAN bus starts disabled — type 'canstart' when bus is ready.
    // Keep TX pin HIGH (recessive) so the SN65HVD230 doesn't drive
    // the bus dominant while CAN is not yet initialised.
    gpio_init(CAN_GPIO_TX);
    gpio_set_dir(CAN_GPIO_TX, GPIO_OUT);
    gpio_put(CAN_GPIO_TX, 1);

    // Wait for USB CDC connection before printing
    while (!stdio_usb_connected())
        sleep_ms(100);

    printf("=== Invent EMS Protocol TX Emulator ===\n");
    printf("sizeof(TInfoPacket) = %d\n", (int)sizeof(TInfoPacket));
    printf("Packet on wire: %d bytes (header %d + payload %d + CRC 2)\n",
           (int)(HEADER_SIZE + sizeof(TInfoPacket) + 2),
           HEADER_SIZE, (int)sizeof(TInfoPacket));
    printf("UART1: TX=GP%d  RX=GP%d  %d baud\n", UART_TX_PIN, UART_RX_PIN, UART_BAUD);
    printf("CAN:   TX=GP%d  RX=GP%d  %d kbps (PIO%d) — type 'canstart' to enable\n",
           CAN_GPIO_TX, CAN_GPIO_RX, CAN_BITRATE / 1000, CAN_PIO_NUM);
    printf("Type 'help' for commands.\n");

    uint32_t lastTx = to_ms_since_boot(get_absolute_time());

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (now - lastTx >= TX_INTERVAL_MS) {
            float dt = (now - lastTx) / 1000.0f;
            lastTx = now;

            simulateEngine(dt);
            buildAndSend();
            sendAllCAN();
        }

        processSerialCommands();
    }

    return 0;
}
