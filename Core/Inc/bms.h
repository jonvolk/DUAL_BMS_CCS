#if !defined(__BMS_H_)
#define __BMS_H_

#include "main.h"
#include "can_setup.h"
#include "stdbool.h"


typedef struct bms
{
    uint16_t cellVolt[96]; //all cell calcs in millivolts
    uint16_t tempSensor[16];
    uint16_t packVolt; // pack volt x 10
    uint16_t lowCellVolt;
    uint16_t highCellVolt;
    uint16_t avgCellVolt;
    uint16_t cellDelta;
    uint16_t highCellTemp;
    uint16_t lowCellTemp;
    uint16_t avgCellTemp;
    uint16_t tempDelta;
    uint8_t state;
    uint8_t SOC;
    uint8_t chargeRequest; // conditions met for pack to request charging
    bool balancecells; //conditions met for pack to request balancing
} bms_t;
bms_t BMS[2];

uint8_t vechicleState; //CAN signal from VCU to get key/run state
bool charged; // flag to synch both packs

void sendCommand(void);
void requestBICMdata(bms_t *bms);
void refreshData(void);
void acChargeCommand(void);
void balanceCommand(bms_t *bms, int pack);
void vehicleComms(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);
void decodeVolt(bms_t *bms, CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);
void decodeTemp(bms_t *bms, CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);
void bmsStateHandler(bms_t *bms);
void tx500kData(void);
void initBMS(void);

#endif // __BMS_H_
