#if !defined(__LIM__H_)
#define __LIM__H_

#include "main.h"
#include "stdbool.h"
#include "bms.h"
#include "can_setup.h"
#include "settings.h"

/*  TO DO LIST

IMPORT FP LIBRARY
DEAL WITH SFP32
VERIFY IMPORTED VALUES OF 10MS MSG.
User values Votspnt
int32_t I_Batt needs to be properly assigned to current, volt place holder to allow compiling

*/




enum i3LIMChargingState
{
    No_Chg,
    AC_Chg,
    DC_Chg
};


enum modes
{
    MOD_OFF = 0,
    MOD_RUN,
    MOD_PRECHARGE,
    MOD_PCHFAIL,
    MOD_CHARGE,
    MOD_LAST
};


typedef struct 
{
    uint8_t PilotLim; //amps
    uint8_t CableLim; //amps
    bool PlugDet; //EVSE connection status
    uint8_t PilotType;
    uint16_t CCS_V_Con;
    uint16_t CCS_V_Avail;
    uint16_t CCS_I_Avail;
    uint8_t CCS_COND;
    uint16_t CCS_V;
    uint16_t CCS_I;
    uint16_t CCS_V_Min;
    uint8_t CCS_Contactor;
    uint8_t CP_DOOR;
    uint8_t BattCap;
    uint8_t opmode;
    uint8_t CCS_State;
    uint16_t CCS_Ireq;
    uint16_t CCS_ILim;

}parameters_t;
parameters_t PARAM;


typedef uint16_t i3LIMChargingState;

static void handle3B4(uint32_t data[2]);
static void handle29E(uint32_t data[2]);
static void handle2B2(uint32_t data[2]);
static void handle2EF(uint32_t data[2]);
static void handle272(uint32_t data[2]);
static void Send200msMessages();
static void Send100msMessages();
static void Send10msMessages();
static i3LIMChargingState Control_Charge(bool RunCh);

static void CCS_Pwr_Con();
static void Chg_Timers();
void initLim(void);

#endif // __LIM__H_
