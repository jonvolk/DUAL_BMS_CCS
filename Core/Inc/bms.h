#if !defined(__BMS_H_)
#define __BMS_H_

#include "main.h"
#include "can_setup.h"

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
} bms_t;
bms_t BMS[2];

void sendCommand(void);
void refreshData(void);
void balanceCommand(bms_t *bms);
void decodeVolt(bms_t *bms, CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);
void decodeTemp(bms_t *bms, CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);
void bmsStateHandler(void);
void tx500kData(void);

#endif // __BMS_H_
