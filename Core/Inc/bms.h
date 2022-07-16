#if !defined(__BMS_H_)
#define __BMS_H_

#include "main.h"
#include "can_setup.h"
#include "stdbool.h"


typedef struct bms
{
    volatile uint16_t cellVolt[96]; //all cell calcs in millivolts
    volatile uint16_t tempSensor[16];
    volatile uint16_t packVolt; // pack volt x 10
    volatile uint16_t lowCellVolt;
    volatile uint16_t highCellVolt;
    volatile uint16_t avgCellVolt;
    volatile uint16_t cellDelta;
    volatile uint16_t highCellTemp;
    volatile uint16_t lowCellTemp;
    volatile uint16_t avgCellTemp;
    volatile uint16_t tempDelta;
    volatile uint8_t state;
    volatile uint8_t SOC;
    volatile uint8_t chargeRequest; // conditions met for pack to request charging
    volatile bool balancecells; //conditions met for pack to request balancing
} bms_t;
bms_t BMS[2];

//uint8_t vehicleState; //CAN signal from VCU to get key/run state
bool chargerOn; // flag to synch both packs
uint8_t watchdogBits;

void sendCommand(int pack);
void requestBICMdata(bms_t *bms, int pack);
void refreshData(void);
void acChargeCommand(void);
void balanceCommand(bms_t *bms, int pack);
void vehicleComms(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);
void decodeVolt(bms_t *bms, CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);
void decodeTemp(bms_t *bms, CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);
void bmsStateHandler(bms_t *bms);
void tx500kData(void);
void initBMS(void);
void synchChargers(void);

#endif // __BMS_H_
