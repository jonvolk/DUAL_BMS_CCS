#if !defined(__LIM__H_)
#define __LIM__H_

#include "main.h"
#include "stdbool.h"

enum 
{
    No_Chg,
    AC_Chg,
    DC_Chg
};

uint16_t i3LIMChargingState;

static void handle3B4(uint32_t data[2]);
static void handle29E(uint32_t data[2]);
static void handle2B2(uint32_t data[2]);
static void handle2EF(uint32_t data[2]);
static void handle272(uint32_t data[2]);
static void Send200msMessages();
static void Send100msMessages();
static void Send10msMessages();
static uint16_t Control_Charge(bool RunCh);

static void CCS_Pwr_Con();
static void Chg_Timers();
void initLim(void);

#endif // __LIM__H_
