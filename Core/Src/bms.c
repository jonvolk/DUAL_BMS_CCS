#include "bms.h"

static uint16_t getPackVolt(bms_t *bms);
static uint16_t getAvgCellVolt(bms_t *bms);
static uint16_t getLowCellVolt(bms_t *bms);
static uint16_t getHighCellVolt(bms_t *bms);
static uint16_t getCellDelta(bms_t *bms);
static uint16_t getLowCellTemp(bms_t *bms);
static uint16_t getHighCellTemp(bms_t *bms);
static uint16_t getAvgCellTemp(bms_t *bms);
static uint16_t getTempDelta(bms_t *bms);

static const uint8_t balanceByte[96] =
    {0, 0, 0, 0, 0, 0,
     1, 1, 1, 1, 1, 1, 1, 1,
     2, 2, 2, 2, 2, 2, 2, 2,
     3, 3, 3, 3, 3, 3, 3, 3,
     4, 4, 4, 4, 4, 4, 4, 4,
     5, 5, 5, 5, 5, 5, 5, 5,
     6, 6, 6, 6, 6, 6, 6, 6,
     7, 7, 7, 7, 7, 7, 7, 7,
     0, 0, 0, 0, 0, 0, 0, 0,
     1, 1, 1, 1, 1, 1, 1, 1,
     2, 2, 2, 2, 2, 2,
     3, 3, 3, 3, 3, 3,
     4, 4, 4, 4, 4, 4};

static const uint8_t balanceShift[96] =
    {1, 2, 4, 8, 10, 20,
     1, 2, 4, 8, 10, 20, 40, 80,
     1, 2, 4, 8, 10, 20, 40, 80,
     1, 2, 4, 8, 10, 20, 40, 80,
     1, 2, 4, 8, 10, 20, 40, 80,
     1, 2, 4, 8, 10, 20, 40, 80,
     1, 2, 4, 8, 10, 20, 40, 80,
     1, 2, 4, 8, 10, 20, 40, 80,
     1, 2, 4, 8, 10, 20, 40, 80,
     1, 2, 4, 8, 10, 20, 40, 80,
     1, 2, 4, 8, 10, 20,
     1, 2, 4, 8, 10, 20,
     1, 2, 4, 8, 10, 20};

// Primary State Machine ///////////////////////////////////////////////////////////////
void bmsStateHandler(void)
{
}

// Send CAN Data /////////////////////////////////////////////////////////////////////
void tx500kData(void)
{

    txMsg2.StdId = 0x138; //BMS1
    txMsg2.DLC = 8;
    canTx2[0] = BMS[0].packVolt & 0xFF;
    canTx2[1] = (BMS[0].packVolt >> 8) & 0xFF;
    canTx2[2] = BMS[0].avgCellTemp & 0XFF;
    canTx2[3] = (BMS[0].avgCellTemp >> 8) & 0XFF;
    canTx2[4] = BMS[0].cellDelta & 0XFF;
    canTx2[5] = (BMS[0].cellDelta >> 8) & 0XFF;
    canTx2[6] = 0;
    canTx2[7] = 0;
    c2tx(&txMsg2, canTx2);

    txMsg2.StdId = 0x139; //BMS2
    txMsg2.DLC = 8;
    canTx2[0] = BMS[1].packVolt & 0xFF;
    canTx2[1] = (BMS[1].packVolt >> 8) & 0xFF;
    canTx2[2] = BMS[1].avgCellTemp & 0XFF;
    canTx2[3] = (BMS[1].avgCellTemp >> 8) & 0XFF;
    canTx2[4] = BMS[1].cellDelta & 0XFF;
    canTx2[5] = (BMS[1].cellDelta >> 8) & 0XFF;
    canTx2[6] = 0;
    canTx2[7] = 0;
    c2tx(&txMsg2, canTx2);
}

void refreshData(void)
{
    for (size_t i = 0; i < 2; i++)
    {
        getPackVolt(&BMS[i]);
        getAvgCellVolt(&BMS[i]);
        getLowCellVolt(&BMS[i]);
        getHighCellVolt(&BMS[i]);
        getCellDelta(&BMS[i]);
        getLowCellTemp(&BMS[i]);
        getHighCellTemp(&BMS[i]);
        getAvgCellTemp(&BMS[i]);
        getTempDelta(&BMS[i]);
    }
}

// send every 200ms //////////////////////////////////////////////////////////////////
void sendCommand(void)
{
    txMsg.StdId = 0x200;
    txMsg.DLC = 3;
    canTx[0] = 0x02;
    canTx[1] = 0x00;
    canTx[2] = 0x00;
    c1tx(&txMsg, canTx); //pack 1
    txMsg3.StdId = 0x200;
    txMsg3.DLC = 3;
    canTx3[0] = 0x02;
    canTx3[1] = 0x00;
    canTx3[2] = 0x00;
    c3tx(&txMsg3, canTx3); // pack 2
}

// send every 200ms //////////////////////////////////////////////////////////////////
void balanceCommand(bms_t *bms)
{
    txMsg.StdId = 0x300;
    txMsg.DLC = 8;
    for (size_t i = 0; i < 62; i++)
    {
        if (bms->avgCellVolt < bms->cellVolt[i])
        {
            canTx[balanceByte[i]] |= balanceShift[i];
        }
    }
    c1tx(&txMsg, canTx);

    txMsg.StdId = 0x310;
    txMsg.DLC = 5;
    for (size_t i = 62; i < 96; i++)
    {
        if (bms->avgCellVolt < bms->cellVolt[i])
        {
            canTx[balanceByte[i]] |= balanceShift[i];
        }
    }
    c1tx(&txMsg, canTx);
}

///////////////////////////////////////////////////////////////////////////////////////
uint16_t getHighCellVolt(bms_t *bms)
{
    bms->highCellVolt = 0;
    for (size_t i = 0; i < 96; i++)
    {
        if (bms->cellVolt[i] > bms->highCellVolt)
        {
            bms->highCellVolt = bms->cellVolt[i];
        }
    }
    return bms->highCellVolt;
}

///////////////////////////////////////////////////////////////////////////////////////
uint16_t getHighCellTemp(bms_t *bms)
{
    bms->highCellTemp = 0;
    for (size_t i = 0; i < 16; i++)
    {
        if ((bms->tempSensor[i] > bms->highCellTemp) && (bms->tempSensor[i] > 0))
        {
            bms->highCellTemp = bms->tempSensor[i];
        }
    }
    return bms->highCellTemp;
}

///////////////////////////////////////////////////////////////////////////////////////
uint16_t getLowCellTemp(bms_t *bms)
{
    bms->lowCellTemp = 0;
    for (size_t i = 0; i < 16; i++)
    {
        if (bms->tempSensor[i] > bms->lowCellTemp)
        {
            bms->lowCellTemp = bms->tempSensor[i];
        }
    }
    return bms->lowCellVolt;
}

///////////////////////////////////////////////////////////////////////////////////////
uint16_t getAvgCellTemp(bms_t *bms)
{
    int zeroCounter = 0;
    int avgTemp = 0;
    for (size_t i = 0; i < 16; i++)
    {
        if (bms->tempSensor[i] == 0)
        {
            zeroCounter++;
        }

        avgTemp += bms->tempSensor[i];
    }
    bms->avgCellTemp = avgTemp / (16 - zeroCounter);
    return bms->avgCellTemp;
}

///////////////////////////////////////////////////////////////////////////////////////
uint16_t getTempDelta(bms_t *bms)
{
    bms->tempDelta = bms->highCellTemp - bms->lowCellTemp;
    return bms->tempDelta;
}

///////////////////////////////////////////////////////////////////////////////////////
uint16_t getCellDelta(bms_t *bms)
{
    bms->cellDelta = bms->highCellVolt - bms->lowCellVolt;

    return bms->cellDelta;
}

///////////////////////////////////////////////////////////////////////////////////////
uint16_t getLowCellVolt(bms_t *bms)
{
    bms->lowCellVolt = 5000;
    for (size_t i = 0; i < 96; i++)
    {
        if (bms->cellVolt[i] < bms->lowCellVolt)
        {
            bms->lowCellVolt = bms->cellVolt[i];
        }
    }
    return bms->lowCellVolt;
}

///////////////////////////////////////////////////////////////////////////////////////
uint16_t getAvgCellVolt(bms_t *bms)
{

    int cellSum = 0;
    for (size_t i = 0; i < 96; i++)
    {

        cellSum += bms->cellVolt[i];
    }
    bms->avgCellTemp = cellSum / 96;
    return bms->avgCellVolt;
}

///////////////////////////////////////////////////////////////////////////////////////
uint16_t getPackVolt(bms_t *bms)
{
    uint32_t packSum = 0;
    for (size_t i = 0; i < 96; i++)
    {
        packSum += bms->cellVolt[i];
    }
    bms->packVolt = packSum / 10;
    return bms->packVolt;
}

///////////////////////////////////////////////////////////////////////////////////////
void decodeVolt(bms_t *bms, CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx) // cell voltage in millivolts
{
    switch (rxMsg->StdId)
    {
    case 0x460: // Begin module 1
        bms->cellVolt[0] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[1] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[2] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        break;

    case 0x470:
        bms->cellVolt[3] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[4] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[5] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        break;

    case 0x461:
        bms->cellVolt[6] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[7] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[8] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[9] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x471:
        bms->cellVolt[10] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[11] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[12] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[13] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x462:
        bms->cellVolt[14] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[15] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[16] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[17] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x472:
        bms->cellVolt[18] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[19] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[20] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[21] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;
    case 0x463:
        bms->cellVolt[22] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[23] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[24] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[25] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x473:
        bms->cellVolt[26] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[27] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[28] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[29] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x464:
        bms->cellVolt[30] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25); // Begin module 2
        bms->cellVolt[31] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[32] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[33] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x474:
        bms->cellVolt[34] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[35] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[36] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[37] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x465:
        bms->cellVolt[38] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[39] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[40] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[41] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x475:
        bms->cellVolt[42] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[43] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[44] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[45] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x466:
        bms->cellVolt[46] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[47] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[48] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[49] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x476:
        bms->cellVolt[50] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[51] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[52] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[53] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x468:
        bms->cellVolt[54] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25); //Begin module 3
        bms->cellVolt[55] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[56] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[57] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x478:
        bms->cellVolt[58] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[59] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[60] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[61] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x469:
        bms->cellVolt[62] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[63] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[64] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[65] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x479:
        bms->cellVolt[66] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[67] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[68] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[69] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x46A:
        bms->cellVolt[70] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[71] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[72] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[73] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x47A:
        bms->cellVolt[74] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[75] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[76] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        bms->cellVolt[77] = ((((canRx[6] & 0x0F) << 8) + canRx[7]) * 1.25);
        break;

    case 0x46C:
        bms->cellVolt[78] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25); //Begin module 4
        bms->cellVolt[79] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[80] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        break;

    case 0x47C:
        bms->cellVolt[81] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[82] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[83] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        break;

    case 0x46D:
        bms->cellVolt[84] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[85] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[86] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        break;

    case 0x47D:
        bms->cellVolt[87] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[88] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[89] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        break;

    case 0x46E:
        bms->cellVolt[90] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[91] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[92] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        break;

    case 0x47E:
        bms->cellVolt[93] = ((((canRx[0] & 0x0F) << 8) + canRx[1]) * 1.25);
        bms->cellVolt[94] = ((((canRx[2] & 0x0F) << 8) + canRx[3]) * 1.25);
        bms->cellVolt[95] = ((((canRx[4] & 0x0F) << 8) + canRx[5]) * 1.25);
        break;

    default:
        break;
    }
}

void decodeTemp(bms_t *bms, CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx)
{
    switch (rxMsg->StdId)
    {
    case 0x7E0: //Begin module 1 temp sensors
        bms->tempSensor[0] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7E1:
        bms->tempSensor[1] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7E2:
        bms->tempSensor[2] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        bms->tempSensor[3] = -((((canRx[2] << 8) + canRx[3]) * 7) - 28700);
        break;

    case 0x7E3:
        bms->tempSensor[4] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7E4: //Begin module 2 temp sensors
        bms->tempSensor[5] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7E5:
        bms->tempSensor[6] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        bms->tempSensor[7] = -((((canRx[2] << 8) + canRx[3]) * 7) - 28700);
        break;

    case 0x7E6:
        bms->tempSensor[8] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7E8: //Begin module 3 temp sensors
        bms->tempSensor[9] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7E9:
        bms->tempSensor[10] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        bms->tempSensor[11] = -((((canRx[2] << 8) + canRx[3]) * 7) - 28700);
        break;

    case 0x7EA:
        bms->tempSensor[12] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7EC: //Begin module 4 temp sensors
        bms->tempSensor[13] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7ED:
        bms->tempSensor[14] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7EE:
        bms->tempSensor[15] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    default:
        break;
    }
}