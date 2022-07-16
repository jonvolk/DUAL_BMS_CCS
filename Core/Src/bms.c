#include "bms.h"
#include "settings.h"
#include "my_math.h"

static void getPackVolt(bms_t *bms);
static void getAvgCellVolt(bms_t *bms);
static void getLowCellVolt(bms_t *bms);
static void getHighCellVolt(bms_t *bms);
static void getCellDelta(bms_t *bms);
static void getLowCellTemp(bms_t *bms);
static void getHighCellTemp(bms_t *bms);
static void getAvgCellTemp(bms_t *bms);
static void getTempDelta(bms_t *bms);
static void getCellCount(bms_t *bms, int pack);
static void getSOC(bms_t *bms);

enum
{
    Boot,
    Ready,
    Drive,
    Charge,
    Error
};

enum
{
    off,
    on, //key on, inverter off
    charge_keyOff,
    charge_keyOn, //who even does that
    idle,         //key on, inverter on
    run,          //key on direction selected
    launchMode,   //break shit
    burnout,      //destroy tires
};

/*/SOC filtering
static const int numReadings = 10;
static int readings[10];  // the readings from the analog input
static int readIndex = 0; // the index of the current reading
static int total = 0;     // the running total
static int average = 0;   // the average
*/

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
    {0x01, 0x02, 0x04, 0x08, 0x10, 0x20,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20,
     0x01, 0x02, 0x04, 0x08, 0x10, 0x20};

static int errorCount;
// Initialize BMS //////////////////////////////////////////////////////////////////
void initBMS(void)
{
    for (size_t i = 0; i < 2; i++)
    {
        BMS[i].state = Boot;
        BMS[i].cellDelta = 0;
        BMS[i].highCellTemp = 0;
        BMS[i].chargeRequest = 0;
        BMS[i].highCellVolt = 0;
        BMS[i].lowCellVolt = 0;
        BMS[i].lowCellTemp = 0;
        BMS[i].packVolt = 0;
        BMS[i].SOC = 0;
        BMS[i].tempDelta = 0;

        for (size_t j = 0; j < 96; j++)
        {
            BMS[i].cellVolt[j] = 0;
        }

        for (size_t j = 0; j < 16; j++)
        {
            BMS[i].tempSensor[j] = 0;
        }
    }
    //vehicleState = off;
    watchdogBits = 0b0000;
    chargerOn = false;
    errorCount = 0;

}

// Primary State Machine ///////////////////////////////////////////////////////////////
void bmsStateHandler(bms_t *bms)
{
    switch (bms->state)
    {
    case Boot:
        bms->chargeRequest = 0;
        bms->state = Ready;

        break;

    case Ready:
        bms->chargeRequest = 0;
        if (bms->avgCellVolt > BALANCE_VOLTAGE)
        {
            if ((bms->highCellVolt - bms->lowCellVolt) > (BALANCE_HYS << 2))
            {
                bms->balancecells = true;
            }

            else if ((bms->highCellVolt - bms->lowCellVolt) <= BALANCE_HYS )
            {
                bms->balancecells = false;
            }
        }
        else
        {
            bms->balancecells = false;
        }

        if ((bms->highCellVolt < (CHARGE_V_SETPOINT - CHARGE_HYS)) && (bms->lowCellVolt > UNDER_V_SETPOINT))
        {
            bms->state = Charge;
        }
        break;

    case Charge:
        bms->balancecells = false;
        bms->chargeRequest = 1;        

        if (bms->highCellVolt > CHARGE_V_SETPOINT)// || bms->highCellTemp > OVER_T_SETPOINT)
        {
            bms->state = Ready;
        }
        break;

    case Error:
        HAL_NVIC_SystemReset();
        break;

    default:
        break;
    }
}

// 10kw Tesla Charger /////////////////////////////////////////////////////////////////////
void acChargeCommand(void)
{
    uint8_t canTx2[8];
    if (chargerOn)
    {
        int val = 32;
        canTx2[0] = 0x40;
        canTx2[1] = 0x00;
        canTx2[2] = 0x20;
        canTx2[3] = 9;
        canTx2[4] = val & 0xFF;
        canTx2[5] = (val >> 8) & 0xFF; 
        canTx2[6] = (val >> 16) & 0xFF;
        canTx2[7] = (val >> 24) & 0xFF;
        can2tx(0x605, 8, canTx2); 
    }
    else
    {
        int val = 0;
        canTx2[0] = 0x40;
        canTx2[1] = 0x00;
        canTx2[2] = 0x20;
        canTx2[3] = 9;
        canTx2[4] = val & 0xFF;
        canTx2[5] = (val >> 8) & 0xFF;
        canTx2[6] = (val >> 16) & 0xFF;
        canTx2[7] = (val >> 24) & 0xFF;
        can2tx(0x605, 8, canTx2);

    }
    HAL_Delay(1);
}
// Send CAN Data /////////////////////////////////////////////////////////////////////
void tx500kData(void)
{

    uint8_t bms0[8];
    bms0[0] = BMS[0].packVolt & 0xFF;
    bms0[1] = (BMS[0].packVolt >> 8) & 0xFF;
    bms0[2] = BMS[0].avgCellTemp & 0XFF;
    bms0[3] = (BMS[0].avgCellTemp >> 8) & 0XFF;
    bms0[4] = BMS[0].cellDelta & 0XFF;
    bms0[5] = (BMS[0].cellDelta >> 8) & 0XFF;
    bms0[6] = (BMS[0].SOC);
    bms0[7] = (BMS[0].balancecells);//0;
    can2tx(0x138, 8, bms0);


    uint8_t bms1[8];
    bms1[0] = BMS[1].packVolt & 0xFF;
    bms1[1] = (BMS[1].packVolt >> 8) & 0xFF;
    bms1[2] = BMS[1].avgCellTemp & 0XFF;
    bms1[3] = (BMS[1].avgCellTemp >> 8) & 0XFF;
    bms1[4] = BMS[1].cellDelta & 0XFF;
    bms1[5] = (BMS[1].cellDelta >> 8) & 0XFF;
    bms1[6] = (BMS[1].SOC);
    bms1[7] = (BMS[1].balancecells);//0;
    can2tx(0x139, 8, bms1);

}

void refreshData(void)
{

    for (size_t i = 0; i < 2; i++)
    {
        //requestBICMdata(&BMS[i], i);
        getPackVolt(&BMS[i]);
        getAvgCellVolt(&BMS[i]);
        getLowCellVolt(&BMS[i]);
        getHighCellVolt(&BMS[i]);
        getCellDelta(&BMS[i]);
        getLowCellTemp(&BMS[i]);
        getHighCellTemp(&BMS[i]);
        getAvgCellTemp(&BMS[i]);
        getTempDelta(&BMS[i]);
        getCellCount(&BMS[i], i);
        getSOC(&BMS[i]);
    }
}

// send every 200ms //////////////////////////////////////////////////////////////////
void sendCommand(int pack)
{
    uint8_t canTx[3];
    canTx[0] = 0x02;
    canTx[1] = 0x00;
    canTx[2] = 0x00;

    switch (pack)
    {
    case 0:
        can1tx(0x200, 3, canTx);
        break;

    case 1:
        can3tx(0x200, 3, canTx);
        break;

    default:
        break;
    }
}

void requestBICMdata(bms_t *bms, int pack)
{

    uint8_t canTx[8];
    if (!bms->balancecells)
    {
        sendCommand(pack);
        for (size_t i = 0; i < 8; i++)
        {
            canTx[i] = 0x00;
        }
        switch (pack)
        {
        case 0:
            can1tx(0x300, 8, canTx);
            break;

        case 1:
            can3tx(0x300, 8, canTx);
            break;

        default:
            break;
        }

        for (size_t i = 0; i < 5; i++)
        {
            canTx[i] = 0x00;
        }

        switch (pack)
        {
        case 0:
            can1tx(0x310, 5, canTx);
            break;

        case 1:
            can3tx(0x310, 5, canTx);
            break;

        default:
            break;
        }
    }
}

// send every 200ms //////////////////////////////////////////////////////////////////
void balanceCommand(bms_t *bms, int pack)
{
    sendCommand(pack);
    uint8_t balance[8];

    switch (pack)
    {
    case 0:
        for (size_t i = 0; i < 8; i++)
        {
            balance[i] = 0;
        }

        for (size_t i = 0; i < 62; i++)
        {
            if (bms->avgCellVolt < bms->cellVolt[i])
            {
                balance[balanceByte[i]] |= balanceShift[i];
            }
        }
        can1tx(0x300, 8, balance);

        for (size_t i = 0; i < 8; i++)
        {
            balance[i] = 0;
        }

        for (size_t i = 62; i < 96; i++)
        {
            if (bms->avgCellVolt < bms->cellVolt[i])
            {
                balance[balanceByte[i]] |= balanceShift[i];
            }
        }
        can1tx(0x310, 5, balance);
        break;

    case 1:

        for (size_t i = 0; i < 8; i++)
        {
            balance[i] = 0;
        }
        for (size_t i = 0; i < 62; i++)
        {
            if (bms->avgCellVolt < bms->cellVolt[i])
            {
                balance[balanceByte[i]] |= balanceShift[i];
            }
        }
        can3tx(0x300, 8, balance);

        for (size_t i = 0; i < 8; i++)
        {
            balance[i] = 0;
        }
        for (size_t i = 62; i < 96; i++)
        {
            if (bms->avgCellVolt < bms->cellVolt[i])
            {
                balance[balanceByte[i]] |= balanceShift[i];
            }
        }
        can3tx(0x310, 5, balance);
        break;

    default:
        break;
    }
}

///////////////////////////////////////////////////////////////////////////////////////
void getSOC(bms_t *bms)
{
    bms->SOC = MAP(bms->avgCellVolt, SOC_VOLT_10, SOC_VOLT_90, 10, 90);
    /*
    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = bms->SOC;
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;
    // if we're at the end of the array...
    if (readIndex >= numReadings)
    {
        // ...wrap around to the beginning:
        readIndex = 0;
    }
    // calculate the average:
    average = total / numReadings;
    */
}

///////////////////////////////////////////////////////////////////////////////////////
void getHighCellVolt(bms_t *bms)
{
    bms->highCellVolt = 0;
    for (size_t i = 0; i < 96; i++)
    {
        if (bms->cellVolt[i] > bms->highCellVolt)
        {
            bms->highCellVolt = bms->cellVolt[i];
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////
void getHighCellTemp(bms_t *bms)
{
    bms->highCellTemp = 0;
    for (size_t i = 0; i < 16; i++)
    {
        if (bms->tempSensor[i] > bms->highCellTemp)
        {
            bms->highCellTemp = bms->tempSensor[i];
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////
void getLowCellTemp(bms_t *bms)
{
    bms->lowCellTemp = 20000;
    for (size_t i = 0; i < 16; i++)
    {
        if (bms->tempSensor[i] > 0)
        {
            if (bms->tempSensor[i] < bms->lowCellTemp)
            {
                bms->lowCellTemp = bms->tempSensor[i];
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////
void getAvgCellTemp(bms_t *bms)
{
    int zeroCounter = 0;
    int sumTemp = 0;
    for (size_t i = 0; i < 16; i++)
    {
        if (bms->tempSensor[i] == 0) //IGNORE_TEMP)
        {
            zeroCounter++;
        }

        sumTemp += bms->tempSensor[i];
    }
    bms->avgCellTemp = (sumTemp / (16 - zeroCounter));
}

///////////////////////////////////////////////////////////////////////////////////////
void getTempDelta(bms_t *bms)
{
    bms->tempDelta = bms->highCellTemp - bms->lowCellTemp;
}

///////////////////////////////////////////////////////////////////////////////////////
void getCellDelta(bms_t *bms)
{
    bms->cellDelta = bms->highCellVolt - bms->lowCellVolt;
}

///////////////////////////////////////////////////////////////////////////////////////
void getLowCellVolt(bms_t *bms)
{
    bms->lowCellVolt = 5000;
    for (size_t i = 0; i < 96; i++)
    {
        if (bms->cellVolt[i] < bms->lowCellVolt)
        {
            bms->lowCellVolt = bms->cellVolt[i];
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////
void getAvgCellVolt(bms_t *bms)
{

    int cellSum = 0;
    for (size_t i = 0; i < 96; i++)
    {

        cellSum += bms->cellVolt[i];
    }
    bms->avgCellVolt = (cellSum / 96);
}

///////////////////////////////////////////////////////////////////////////////////////
void getPackVolt(bms_t *bms)
{
    uint32_t packSum = 0;
    for (size_t i = 0; i < 96; i++)
    {
        packSum += bms->cellVolt[i];
    }
    bms->packVolt = packSum / 10;
}
///////////////////////////////////////////////////////////////////////////////////////
void getCellCount(bms_t *bms, int pack)
{
    uint16_t cellCount = 0;
    for (size_t i = 0; i < 96; i++)
    {
        if (bms->cellVolt[i] > IGNORE_VOLT)
        {
            cellCount++;
        }
    }
    if (cellCount != 96)
    {
        //BMS[pack].state = Error;
        errorCount++;
        if (errorCount > 5)
        {
            BMS[pack].state = Error;
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////
void vehicleComms(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx)
{
    switch (rxMsg->StdId)
    {
    case 0x313:
        //vehicleState = canRx[0];
        break;

    default:
        break;
    }
}

///////////////////////////////////////////////////////////////////////////////////////
void synchChargers(void)
{

    if (BMS[0].chargeRequest && BMS[1].chargeRequest)
    {
        chargerOn = true;
    }

    if (chargerOn)
    {
        if ((!BMS[0].chargeRequest) || (!BMS[1].chargeRequest))
        {
            chargerOn = false;
            BMS[0].state = Ready;
            BMS[1].state = Ready;
        }
    }
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

void decodeTemp(bms_t *bms, CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx) // in degrees F x 100
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
        bms->tempSensor[2] = 0; //-((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        bms->tempSensor[3] = -((((canRx[2] << 8) + canRx[3]) * 7) - 28700);
        break;

    case 0x7E3:
        bms->tempSensor[4] = 0; //-((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7E4: //Begin module 2 temp sensors
        bms->tempSensor[5] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7E5:
        bms->tempSensor[6] = 0; //-((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        bms->tempSensor[7] = -((((canRx[2] << 8) + canRx[3]) * 7) - 28700);
        break;

    case 0x7E6:
        bms->tempSensor[8] = 0; //-((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7E8: //Begin module 3 temp sensors
        bms->tempSensor[9] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7E9:
        bms->tempSensor[10] = 0; //-((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        bms->tempSensor[11] = -((((canRx[2] << 8) + canRx[3]) * 7) - 28700);
        break;

    case 0x7EA:
        bms->tempSensor[12] = 0; //-((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7EC: //Begin module 4 temp sensors
        bms->tempSensor[13] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7ED:
        bms->tempSensor[14] = -((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    case 0x7EE:
        bms->tempSensor[15] = 0; //-((((canRx[0] << 8) + canRx[1]) * 7) - 28700);
        break;

    default:
        break;
    }
}