#include "lim.h"

#define  CCS_STATUS_NOT_READY 0X0
#define  CCS_STATUS_INIT   0X1
#define  CCS_STATUS_READY 0x2



typedef struct
{
    uint8_t endCharge;
    uint8_t charge;
    /* data */
} ccsRequest_t;
ccsRequest_t ccsRequest;

typedef struct
{
    uint8_t notReady;
    uint8_t ready;
} ccsReady_t;
ccsReady_t ccsReady;

typedef struct
{
    uint8_t standby;
    uint8_t initialization;
    uint8_t subpoena;
    uint8_t energyTransfer;
    uint8_t shutDown;
    uint8_t cableTest;
    uint8_t reserved;
    uint8_t invalidSignal;
} ccsPhase_t;
ccsPhase_t ccsPhase;


static uint8_t CP_Mode = 0;
static uint8_t phaseState = 0;
static uint8_t lim_state = 0;
static uint8_t lim_stateCnt = 0;
static uint8_t ctr_1second = 0;
static uint8_t ctr_5second = 0;
static uint8_t ctr_20ms = 0;
static uint8_t vin_ctr = 0;
static uint8_t Timer_1Sec = 0;
static uint8_t Timer_60Sec = 0;