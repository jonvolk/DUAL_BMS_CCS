#include "lim.h"

typedef int fixedPointToFixLater;
typedef uint8_t ChargeStatus;
typedef uint8_t ChargeRequest;
typedef uint8_t ChargeReady;

#define CCS_STATUS_NOT_READY 0X0
#define CCS_STATUS_INIT 0X1
#define CCS_STATUS_READY 0x2

#define CCS_REQUEST_END_CHARGE 0X0
#define CCS_REQUEST_CHARGE 0X1

#define CCS_READY_NOT_RDY 0X0
#define CCS_READY_RDY 0X1

#define CCS_PHASE_STANDY 0X0
#define CCS_PHASE_INITIALIZATION 0X1
#define CCS_PHASE_SUBPOENA 0X2
#define CCS_PHASE_ENGERGY_TRANSFER 0X3
#define CCS_PHASE_SHUTDOWN 0X4
#define CCS_PHASE_CABLE_TEST 0X9
#define CCS_PHASE_RESERVED 0XE
#define CCS_PHASE_INVALID_SIGNAL 0XF

static uint8_t CP_Mode = 0;
static uint8_t Chg_Phase = CCS_PHASE_STANDY;
static uint8_t lim_state = 0;
static uint8_t lim_stateCnt = 0;
static uint8_t ctr_1second = 0;
static uint8_t ctr_5second = 0;
static uint8_t ctr_20ms = 0;
static uint8_t vin_ctr = 0;
static uint8_t Timer_1Sec = 0;
static uint8_t Timer_60Sec = 0;
uint8_t ChargeType = 0;
uint8_t CCS_Plim = 0;    //ccs power limit flag. 0=no,1=yes,3=invalid.
uint8_t CCS_Ilim = 0;    //ccs current limit flag. 0=no,1=yes,3=invalid.
uint8_t CCS_Vlim = 0;    //ccs voltage limit flag. 0=no,1=yes,3=invalid.
uint8_t CCS_Stat = 0;    //ccs charging status. 0=standby,1=charging,3=invalid.
uint8_t CCS_Malf = 0;    //ccs malfunction status. 0=normal,1=fail,3=invalid.
uint8_t CCS_Bmalf = 0;   //ccs battery malfunction status. 0=no,1=yes,3=invalid.
uint8_t CCS_Stop = 0;    //ccs chargeing stop status. 0=tracking,1=supression,3=invalid.
uint8_t CCS_Iso = 0;     //ccs isolation status. 0=invalid,1=valid,2=error,3=invalid signal.
uint8_t CCS_IntStat = 0; //ccs charger internal status. 0=not ready,1=ready,2=switch off charger,3=interruption,4=pre charge,5=insulation monitor,6=estop,7=malfunction,0x13=reserved,0x14=reserved,0x15=invlaid signal.
static uint32_t sec_328 = 0;
static uint16_t Cont_Volts = 0;
static uint16_t Bulk_SOCt = 0;                         //Time to bulk soc target.
static uint16_t Full_SOCt = 0;                         //Time to full SOC target.
static fixedPointToFixLater CHG_Pwr = 0;               //calculated charge power. 12 bit value scale x25. Values based on 50kw DC fc and 1kw and 3kw ac logs. From bms???
static int16_t FC_Cur = 0;                             //10 bit signed int with the ccs dc current command.scale of 1.
static uint8_t EOC_Time = 0x00;                        //end of charge time in minutes.
static ChargeStatus CHG_Status = CCS_STATUS_NOT_READY; //observed values 0 when not charging , 1 and transition to 2 when commanded to charge. only 4 bits used.
                                                       //seems to control led colour.
static ChargeRequest CHG_Req = CCS_REQUEST_END_CHARGE; //observed values 0 when not charging , 1 when requested to charge. only 1 bit used in logs so far.
static ChargeReady CHG_Ready = CCS_READY_NOT_RDY;      //indicator to the LIM that we are ready to charge. observed values 0 when not charging , 1 when commanded to charge. only 2 bits used.
static uint8_t CONT_Ctrl = 0;                          //4 bits with DC ccs contactor command.
static uint8_t CCSI_Spnt = 0;

void handle3B4(uint32_t data[2]) //Lim data
{
    /*
    0x3B4 D4 low nible: status pilot
0=no pilot
1=10-96%PWM not charge ready
2=10-96%PWM charge ready
3=error
4=5% not charge ready
5=5% charge ready
6=pilot static
    */
    uint8_t *bytes = (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    uint8_t CP_Amps = bytes[0];
    PARAM.PilotLim = CP_Amps; //Param::SetInt(Param::PilotLim, CP_Amps);
    uint8_t PP_Amps = bytes[1];
    PARAM.CableLim = PP_Amps; //Param::SetInt(Param::CableLim, PP_Amps);
    bool PP = (bytes[2] & 0x1);
    PARAM.PlugDet = PP; //Param::SetInt(Param::PlugDet, PP);
    CP_Mode = (bytes[4] & 0x7);
    PARAM.PilotType = CP_Mode; //Param::SetInt(Param::PilotTyp, CP_Mode);
    Cont_Volts = bytes[7] * 2;
    // Cont_Volts=FP_MUL(Cont_Volts,2);
    PARAM.CCS_V_Con = Cont_Volts; //Param::SetInt(Param::CCS_V_Con, Cont_Volts); //voltage measured on the charger side of the hv ccs contactors in the car
    ChargeType = bytes[6];
}

void handle29E(uint32_t data[2]) //Lim data. Available current and voltage from the ccs charger
{
    uint8_t *bytes = (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    uint16_t V_Avail = ((bytes[2] << 8) | (bytes[1]));
    V_Avail = FP_TOINT(FP_DIV(V_Avail, 10));
    PARAM.CCS_V_Avail = V_Avail; //Param::SetInt(Param::CCS_V_Avail,V_Avail);//available voltage from ccs charger

    uint16_t I_Avail = ((bytes[4] << 8) | (bytes[3]));
    I_Avail = FP_TOINT(FP_DIV(I_Avail, 10));
    PARAM.CCS_I_Avail = I_Avail; //Param::SetInt(Param::CCS_I_Avail,I_Avail);//available current from ccs charger

    CCS_Iso = (bytes[0] >> 6) & 0x03;
    CCS_IntStat = (bytes[0] >> 2) & 0x0f;
    PARAM.CCS_COND = CCS_IntStat; //Param::SetInt(Param::CCS_COND,CCS_IntStat);//update evse condition on webui
}

void handle2B2(uint32_t data[2]) //Lim data. Current and Votage as measured by the ccs charger
{
    uint8_t *bytes = (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    uint16_t CCS_Vmeas = ((bytes[1] << 8) | (bytes[0]));
    CCS_Vmeas = FP_TOINT(FP_DIV(CCS_Vmeas, 10));
    PARAM.CCS_V = CCS_Vmeas; //Param::SetInt(Param::CCS_V, CCS_Vmeas); //Voltage measurement from ccs charger

    uint16_t CCS_Imeas = ((bytes[3] << 8) | (bytes[2]));
    CCS_Imeas = FP_TOINT(FP_DIV(CCS_Imeas, 10));
    PARAM.CCS_I = CCS_Imeas;            //Param::SetInt(Param::CCS_I, CCS_Imeas);              //Current measurement from ccs charger
                                        //[[maybe_unused]]
    uint8_t Batt_Cmp = bytes[4] & 0xc0; //battrery compatability flag from charger? upper two bits of byte 4.

    CCS_Ilim = (bytes[5] >> 4) & 0x03;
    CCS_Vlim = (bytes[5] >> 6) & 0x03;
    CCS_Stat = bytes[4] & 0x03;
    CCS_Malf = (bytes[4] >> 2) & 0x03;
    CCS_Bmalf = bytes[5] & 0x03;
    CCS_Stop = (bytes[5] >> 2) & 0x03;
}

void handle2EF(uint32_t data[2]) //Lim data. Min available voltage from the ccs charger.
{
    uint8_t *bytes = (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    uint16_t minV_Avail = ((bytes[1] << 8) | (bytes[0]));
    minV_Avail = FP_TOINT(FP_DIV(minV_Avail, 10));
    PARAM.CCS_V_Min = minV_Avail; //Param::SetInt(Param::CCS_V_Min,minV_Avail);//minimum available voltage from ccs charger

    CCS_Plim = (bytes[6] >> 4) & 0x03;
}

void handle272(uint32_t data[2]) //Lim data. CCS contactor state and charge flap open/close status.
{
    uint8_t *bytes = (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    // Only the top 6-bits indicate the contactor state
    uint8_t Cont_stat = bytes[2] >> 2;
    PARAM.CCS_Contactor = Cont_stat; //Param::SetInt(Param::CCS_Contactor, Cont_stat);

    uint8_t drmodes = bytes[2] & 0x03;
    PARAM.CP_DOOR = drmodes; //Param::SetInt(Param::CP_DOOR, drmodes);
}

void Send10msMessages()
{
    uint16_t V_Batt = BMS[0].packVolt * 10; //Param::GetInt(Param::udc) * 10;
    uint8_t V_Batt2 = BMS[0].packVolt / 4;  //(Param::GetInt(Param::udc)) / 4;
    int32_t I_Batt = BMS[0].packVolt;       //(Param::GetInt(Param::idc) + 819) * 10; //(Param::GetInt(Param::idc);FP_FROMINT
    //I_Batt=0xa0a0;
    //uint16_t SOC_Local=25*10;//(Param::GetInt(Param::SOC))*10;
    uint16_t SOC_Local = BMS[0].SOC * 10; //(Param::GetInt(Param::SOC)) * 10;
    uint8_t bytes[8];                     //seems to be from i3 BMS.
    txMsg2.StdId = 0x112;
    txMsg2.DLC = 8;
    bytes[0] = I_Batt & 0xFF;    //Battery current LSB. Scale 0.1 offset 819.2. 16 bit unsigned int
    bytes[1] = I_Batt >> 8;      //Battery current MSB. Scale 0.1 offset 819.2.  16 bit unsigned int
    bytes[2] = V_Batt & 0xFF;    //Battery voltage LSB. Scale 0.1. 16 bit unsigned int.
    bytes[3] = V_Batt >> 8;      //Battery voltage MSB. Scale 0.1. 16 bit unsigned int.
    bytes[4] = SOC_Local & 0xFF; //Battery SOC LSB. 12 bit unsigned int. Scale 0.1. 0-100%
    bytes[5] = SOC_Local >> 8;   //Battery SOC MSB. 12 bit unsigned int. Scale 0.1. 0-100%
    bytes[6] = 0x65;             //Low nibble battery status. Seem to need to be 0x5.
    bytes[7] = V_Batt2;          //zwischenkreis. Battery voltage. Scale 4. 8 bit unsigned int.
    c2tx(&txMsg2, bytes);
    //Can::GetInterface(0)->Send(0x112, (uint32_t *)bytes, 8); //Send on CAN1

    ctr_20ms++;
    if (ctr_20ms == 2)
    {
        ctr_20ms = 0;

        //Vehicle speed msg should be 20ms. Lets try 10...
        txMsg2.StdId = 0x1A1;
        txMsg2.DLC = 5;
        bytes[0] = 0x7C;
        bytes[1] = 0xCB;
        bytes[2] = 0x00;
        bytes[3] = 0x00;
        bytes[4] = 0x8A;
        c2tx(&txMsg2, bytes);
        //Can::GetInterface(0)->Send(0x1a1, (uint32_t *)bytes, 5); //Send on CAN1. average 20ms
    }
}

void Send200msMessages()
{

    txMsg2.StdId = 0x2FA;
    txMsg2.DLC = 8;
    uint8_t bytes[8];
    //Lim command 3. Used in DC mode.
    if (CP_Mode == 0x4 || CP_Mode == 0x5)
        bytes[0] = 0xFC;
    else
        bytes[0] = 0xFD;
    //bytes[0] = 0xFD;// FD at standby, change to FC on 5% pilot. Change back to FD during energy transfer
    bytes[1] = 0xFF;                     //these bytes are used as a timer during energy transfer but not at setup
    bytes[2] = (uint8_t)Chg_Phase << 4; //upper nibble seems to be a mode command to the ccs station. 0 when off, 9 when in constant current phase of cycle.
                                         //more investigation needed here...
                                         //Lower nibble seems to be intended for two end charge commands each of 2 bits.

    bytes[4] = 0xff;
    bytes[5] = 0xff;
    bytes[6] = 0xff;
    bytes[7] = 0xff;
    c2tx(&txMsg2, bytes);
    //Can::GetInterface(0)->Send(0x2fa, (uint32_t*)bytes,8); //Send on CAN1. this msg varies from 82ms to 1s intervals.

    //////////////////////////////////////////////////////////////////////////////
    //Possibly needed for dc ccs.
    ////////////////////////////////////

    txMsg2.StdId = 0x432;
    txMsg2.DLC = 8;
    uint16_t SOC_Local = BMS[0].SOC * 2; //(Param::GetInt(Param::SOC))*2;
    bytes[0] = 0x2c;                     //BMS soc msg. May need to be dynamic
    bytes[1] = 0xe2;
    bytes[2] = 0x10;
    bytes[3] = 0xa3;
    //bytes[4] = 0x30;    //display soc. scale 0.5.
    bytes[4] = SOC_Local; //display soc. scale 0.5.
    bytes[5] = 0xff;
    bytes[6] = 0x02;
    bytes[7] = 0xff;
    c2tx(&txMsg2, bytes);
    //Can::GetInterface(0)->Send(0x432, (uint32_t*)bytes,8); //Send on CAN1. average 190ms

    txMsg2.StdId = 0x51A;
    txMsg2.DLC = 8;
    bytes[0] = 0x00; //network management
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x50;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x1a;
    c2tx(&txMsg2, bytes);
    //Can::GetInterface(0)->Send(0x51a, (uint32_t*)bytes,8); //Send on CAN1. average 640ms

    txMsg2.StdId = 0x540;
    txMsg2.DLC = 8;
    bytes[0] = 0x00; //network management.May need to be dynamic
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0xfd;
    bytes[5] = 0x3c;
    bytes[6] = 0xff;
    bytes[7] = 0x40;
    c2tx(&txMsg2, bytes);
    //Can::GetInterface(0)->Send(0x540, (uint32_t*)bytes,8); //Send on CAN1. average 640ms

    txMsg2.StdId = 0x510;
    txMsg2.DLC = 8;
    bytes[0] = 0x40; //network management zgw
    bytes[1] = 0x10;
    bytes[2] = 0x20;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    c2tx(&txMsg2, bytes);
    //Can::GetInterface(0)->Send(0x510, (uint32_t*)bytes,8); //Send on CAN1. average 640ms

    ctr_1second++;
    if (ctr_1second == 5) //only send every 1 second.
    {
        ctr_1second = 0;
        sec_328++; //increment seconds counter.
        txMsg2.StdId = 0x328;
        txMsg2.DLC = 6;
        bytes[0] = sec_328; //rtc msg. needs to be every 1 sec. first 32 bits are 1 second wrap counter
        bytes[1] = sec_328 << 8;
        bytes[2] = sec_328 << 16;
        bytes[3] = sec_328 << 24;
        bytes[4] = 0x87; //day counter 16 bit.
        bytes[5] = 0x1e;
        c2tx(&txMsg2, bytes);
        //Can::GetInterface(0)->Send(0x328, (uint32_t*)bytes,6); //Send on CAN1

        //if(Param::GetInt(Param::opmode)==MOD_RUN) bytes[0] = 0xfb;//f1=no obd reset. fb=obd reset.
        //if(Param::GetInt(Param::opmode)!=MOD_RUN) bytes[0] = 0xf1;//f1=no obd reset. fb=obd reset.
        txMsg2.StdId = 0xE8;
        txMsg2.DLC = 2;
        bytes[0] = 0xf1;
        bytes[1] = 0xff;
        c2tx(&txMsg2, bytes);
        //Can::GetInterface(0)->Send(0x3e8, (uint32_t*)bytes,2); //Send on CAN1

        txMsg2.StdId = 0x3F9;
        bytes[0] = 0xc0; //engine info? rex?
        bytes[1] = 0xf9;
        bytes[2] = 0x80;
        bytes[3] = 0xe0;
        bytes[4] = 0x43;
        bytes[5] = 0x3c;
        bytes[6] = 0xc3; //0x3=park
        bytes[7] = 0xff;
        c2tx(&txMsg2, bytes);
        //Can::GetInterface(0)->Send(0x3f9, (uint32_t*)bytes,8); //Send on CAN1.average 1s

        ctr_5second++;
        if (ctr_5second == 4) //only send every 4 second.
        {
            ctr_5second = 0;

            //central locking status message.
            txMsg2.StdId = 0x2FC;
            txMsg2.DLC = 8;
            bytes[0] = 0x81; //81=flap unlock, 80=flap lock.
            bytes[1] = 0x00;
            bytes[2] = 0x04;
            bytes[3] = 0xff;
            bytes[4] = 0xff;
            bytes[5] = 0xff;
            bytes[6] = 0xff;
            bytes[7] = 0xff;
            c2tx(&txMsg2, bytes);
            //Can::GetInterface(0)->Send(0x2fc, (uint32_t*)bytes,8); //Send on CAN1. average 5s.

            txMsg2.StdId = 0x2A0;
            txMsg2.DLC = 8;
            bytes[0] = 0x88; //central locking
            bytes[1] = 0x88;
            bytes[2] = 0xf8;
            bytes[3] = 0x0f;
            bytes[4] = 0xff;
            bytes[5] = 0xff;
            bytes[6] = 0xff;
            bytes[7] = 0xff;
            c2tx(&txMsg2, bytes);
            //Can::GetInterface(0)->Send(0x2a0, (uint32_t*)bytes,8); //Send on CAN1. average 5s.

            txMsg2.StdId = 0x3A0;
            txMsg2.DLC = 8;
            bytes[0] = 0xff; //vehicle condition
            bytes[1] = 0xff;
            bytes[2] = 0xc0;
            bytes[3] = 0xff;
            bytes[4] = 0xff;
            bytes[5] = 0xff;
            bytes[6] = 0xff;
            bytes[7] = 0xfc;
            c2tx(&txMsg2, bytes);
            //Can::GetInterface(0)->Send(0x3a0, (uint32_t*)bytes,8); //Send on CAN1. average 4s.
        }
    }

    /*not needed msgs at least on efacec
bytes[0] = 0x00;//network management edme
bytes[1] = 0x00;
bytes[2] = 0x00;
bytes[3] = 0x00;
bytes[4] = 0x00;
bytes[5] = 0x00;
bytes[6] = 0x00;
bytes[7] = 0x12;
Can::GetInterface(0)->Send(0x512, (uint32_t*)bytes,8); //Send on CAN1. only sent once on 19 log.

bytes[0] = 0x00;//network management kombi
bytes[1] = 0x00;
bytes[2] = 0x00;
bytes[3] = 0x00;
bytes[4] = 0xfe;
bytes[5] = 0x00;
bytes[6] = 0x00;
bytes[7] = 0x60;
Can::GetInterface(0)->Send(0x560, (uint32_t*)bytes,8); //Send on CAN1. not on is 2019 log

bytes[0] = 0xa8;//range info, milage display
bytes[1] = 0x86;
bytes[2] = 0x01;
bytes[3] = 0x02;
bytes[4] = 0x00;
bytes[5] = 0x05;
bytes[6] = 0xac;
bytes[7] = 0x03;
Can::GetInterface(0)->Send(0x330, (uint32_t*)bytes,8); //Send on CAN1. not on is 2019 log

bytes[0] = 0x00;//obd msg
bytes[1] = 0x2a;
bytes[2] = 0x00;
bytes[3] = 0x6c;
bytes[4] = 0x0f;
bytes[5] = 0x55;
bytes[6] = 0x00;
Can::GetInterface(0)->Send(0x397, (uint32_t*)bytes,7); //Send on CAN1. not on 19 log

*/
}
////////////////////////////////////////////////////////////////////////////////

void Send100msMessages()
{
    uint8_t bytes[8];
    bytes[0] = 0xff; //vehicle status msg
    bytes[1] = 0x5f;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0xff;
    bytes[7] = 0xff;
    can2tx(0x03C, 8, bytes);
    //Can::GetInterface(0)->Send(0x03c, (uint32_t *)bytes, 8); //Send on CAN1. average 100ms

    uint16_t Wh_Local = PARAM.BattCap; //Param::GetInt(Param::BattCap);
    CHG_Pwr = (CHG_Pwr & 0xFFF);
    bytes[0] = Wh_Local & 0xFF;                                   //Battery Wh lowbyte
    bytes[1] = Wh_Local >> 8;                                     //BAttery Wh high byte
    bytes[2] = (((uint8_t)CHG_Status << 4) | ((uint8_t)CHG_Req)); //charge status in bits 4-7.goes to 1 then 2.8 secs later to 2. Plug locking???. Charge request in lower nibble. 1 when charging. 0 when not charging.
    bytes[3] = (((CHG_Pwr) << 4) | (uint8_t)CHG_Ready);           //charge readiness in bits 0 and 1. 1 = ready to charge.upper nibble is LSB of charge power.Charge power forecast not actual power!
    bytes[4] = CHG_Pwr >> 4;                                      //MSB of charge power.in this case 0x28 = 40x25 = 1000W. Probably net DC power into the Batt.
    bytes[5] = FC_Cur & 0xff;                                     //LSB of the DC ccs current command
    bytes[6] = ((CONT_Ctrl << 4) | (FC_Cur >> 12));               //bits 0 and 1 MSB of the DC ccs current command.Upper nibble is DC ccs contactor control. Observed in DC fc logs only.
                                                                  //transitions from 0 to 2 and start of charge but 2 to 1 to 0 at end. Status and Ready operate the same as in AC logs.
    bytes[7] = EOC_Time;                                          // end of charge timer.
    can2tx(0x3E9, 8, bytes);
    //Can::GetInterface(0)->Send(0x3E9, (uint32_t *)bytes, 8); //Send on CAN1. average 128ms

    //LIM needs to see this but doesnt control anything...
    bytes[0] = 0xca;
    bytes[1] = 0xff;
    bytes[2] = 0x0b;
    bytes[3] = 0x02;
    bytes[4] = 0x69;
    bytes[5] = 0x26;
    bytes[6] = 0xf3;
    bytes[7] = 0x4b;
    can2tx(0x431, 8, bytes);
    //Can::GetInterface(0)->Send(0x431, (uint32_t *)bytes, 8); //Send on CAN1.average 197ms but as low as 49ms.

    bytes[0] = 0xf5; //Wake up message.
    bytes[1] = 0x28;
    if (PARAM.opmode == MOD_RUN) //Param::GetInt(Param::opmode) == MOD_RUN)
        bytes[2] = 0x8a;         //ignition on
    if (PARAM.opmode != MOD_RUN) //Param::GetInt(Param::opmode) != MOD_RUN)
        bytes[2] = 0x86;         //ignition off 86
    bytes[3] = 0x1d;
    bytes[4] = 0xf1;
    bytes[5] = 0x35;
    bytes[6] = 0x30;
    bytes[7] = 0x80;
    can2tx(0x12F, 8, bytes);
    //Can::GetInterface(0)->Send(0x12f, (uint32_t *)bytes, 8); //Send on CAN1. average 100ms

    //Lim command 2. Used in DC mode
    uint16_t V_limit = 0;
    //if(lim_state==6) V_limit=401*10;//set to 400v in energy transfer state
    //if(lim_state!=6) V_limit=Param::GetInt(Param::udc)*10;
    if (lim_state == 4)
        V_limit = BMS->packVolt * 10; //Param::GetInt(Param::udc) * 10; // drop vlim only during precharge
    else
        V_limit = 415 * 10;      //set to 415v in all other states
    uint8_t I_limit = 125;       //125A limit. may not work
    bytes[0] = V_limit & 0xFF;   //Charge voltage limit LSB. 14 bit signed int.scale 0.1 0xfa2=4002*.1=400.2Volts
    bytes[1] = V_limit >> 8;     //Charge voltage limit MSB. 14 bit signed int.scale 0.1
    bytes[2] = I_limit;          //Fast charge current limit. Not used in logs from 2014-15 vehicle so far. 8 bit unsigned int. scale 1.so max 254amps in theory...
    bytes[3] = Full_SOCt & 0xFF; //time remaining in seconds to hit soc target from byte 7 in AC mode. LSB. 16 bit unsigned int. scale 10.Full SOC.
    bytes[4] = Full_SOCt >> 8;   //time remaining in seconds to hit soc target from byte 7 in AC mode. MSB. 16 bit unsigned int. scale 10.Full SOC.
    bytes[5] = Bulk_SOCt & 0xFF; //time remaining in seconds to hit soc target from byte 7 in ccs mode. LSB. 16 bit unsigned int. scale 10.Bulk SOC.
    bytes[6] = Bulk_SOCt >> 8;   //time remaining in seconds to hit soc target from byte 7 in ccs mode. MSB. 16 bit unsigned int. scale 10.Bulk SOC.
    bytes[7] = 0xA0;             //Fast charge SOC target. 8 bit unsigned int. scale 0.5. 0xA0=160*0.5=80%
    can2tx(0x2F1, 8, bytes);
    //Can::GetInterface(0)->Send(0x2f1, (uint32_t *)bytes, 8); //Send on CAN1. average 100ms

    if (PARAM.opmode != MOD_RUN) //Param::GetInt(Param::opmode) != MOD_RUN)
        vin_ctr = 0;
    if ((PARAM.opmode == MOD_RUN) && vin_ctr < 5)
    {
        /*
bytes[0] = 0x56;                //vin in ascii from 2017 i3 : VB87926
bytes[1] = 0x42;
bytes[2] = 0x38;
bytes[3] = 0x37;
bytes[4] = 0x39;
bytes[5] = 0x32;
bytes[6] = 0x36;
Can::GetInterface(0)->Send(0x380, (uint32_t*)bytes,7); //Send on CAN1
vin_ctr++;
*/
    }
}

i3LIMChargingState Control_Charge(bool RunCh)
{
    int opmode = PARAM.opmode;//Param::GetInt(Param::opmode);
    if (opmode != MOD_RUN) //only do this if we are not in run mode
    {
        //if (Param::GetBool(Param::PlugDet) && (CP_Mode == 0x1 || CP_Mode == 0x2)) //if we have an enable and a plug in and a std ac pilot lets go AC charge mode.
        if (PARAM.PlugDet && (CP_Mode == 0x1 || CP_Mode == 0x2)) //if we have an enable and a plug in and a std ac pilot lets go AC charge mode.
        {
            lim_state = 0; //return to state 0
            PARAM.CCS_State = lim_state;//Param::SetInt(Param::CCS_State, lim_state);
            Chg_Phase = CCS_PHASE_STANDY;//ChargePhase::Standby;
            CONT_Ctrl = 0x0; //dc contactor mode 0 in AC
            FC_Cur = 0;      //ccs current request zero
            EOC_Time = 0xFE;
            CHG_Status = CCS_STATUS_READY;//ChargeStatus::Rdy;
            CHG_Req = CCS_REQUEST_CHARGE;//ChargeRequest::Charge;
            CHG_Ready = CCS_READY_RDY;//ChargeReady::Rdy;
            CHG_Pwr = 6500 / 25; //approx 6.5kw ac

            if (RunCh)
                return AC_Chg; //set ac charge mode if we are enabled on webui

            if (!RunCh)
            {
                lim_state = 0; //return to state 0
                PARAM.CCS_State = lim_state;//Param::SetInt(Param::CCS_State, lim_state);
                Chg_Phase = CCS_PHASE_STANDY;//ChargePhase::Standby;
                CONT_Ctrl = 0x0; //dc contactor mode 0 in off
                FC_Cur = 0;      //ccs current request zero
                EOC_Time = 0x00;
                CHG_Status = CCS_STATUS_NOT_READY;//ChargeStatus::NotRdy;
                CHG_Req = CCS_REQUEST_END_CHARGE;//ChargeRequest::EndCharge;
                CHG_Ready = CCS_READY_NOT_RDY;//ChargeReady::NotRdy;
                CHG_Pwr = 0;
                return No_Chg; //set no charge mode if we are disabled on webui and in state 9 of dc machine
            }
        }

        //if (Param::GetBool(Param::PlugDet) && (CP_Mode == 0x4 || CP_Mode == 0x5 || CP_Mode == 0x6))
        if (PARAM.PlugDet && (CP_Mode == 0x4 || CP_Mode == 0x5 || CP_Mode == 0x6)) //if we have an enable and a plug in and a 5% pilot or a static pilot lets go DC charge mode.
        {
            /*

0=no pilot
1=10-96%PWM not charge ready
2=10-96%PWM charge ready
3=error
4=5% not charge ready
5=5% charge ready
6=pilot static

*/

            PARAM.CCS_State = lim_state;//Param::SetInt(Param::CCS_State, lim_state); //update state machine level on webui
            switch (lim_state)
            {

            case 0:
            {
                Chg_Phase = CCS_PHASE_STANDY;//ChargePhase::Standby;
                CONT_Ctrl = 0x0; //dc contactor mode control required in DC
                FC_Cur = 0;      //ccs current request from web ui for now.
                EOC_Time = 0x00; //end of charge timer
                CHG_Status = CCS_STATUS_INIT;//ChargeStatus::Init;
                CHG_Req = CCS_REQUEST_CHARGE;//ChargeRequest::Charge;
                CHG_Ready = CCS_READY_NOT_RDY;//ChargeReady::NotRdy;
                CHG_Pwr = 0;           //0 power
                CCSI_Spnt = 0;         //No current
                                       //if(CP_Mode==0x4 && opmode==MOD_CHARGE) lim_state++;
                lim_stateCnt++;        //increment state timer counter
                if (lim_stateCnt > 20) //2 second delay
                {
                    lim_state++; //next state after 2 secs
                    lim_stateCnt = 0;
                }
            }
            break;

            case 1:
            {
                //uint16_t I_avail_tmp=Param::GetInt(Param::CCS_I_Avail);
                Chg_Phase = CCS_PHASE_INITIALIZATION;//ChargePhase::Initialisation;
                CONT_Ctrl = 0x0; //dc contactor mode control required in DC
                FC_Cur = 0;      //ccs current request from web ui for now.
                EOC_Time = 0x00; //end of charge timer
                CHG_Status = CCS_STATUS_INIT;//ChargeStatus::Init;
                CHG_Req = CCS_REQUEST_CHARGE;//ChargeRequest::Charge;
                CHG_Ready = CCS_READY_NOT_RDY;//ChargeReady::NotRdy;
                CHG_Pwr = 0;   //0 power
                CCSI_Spnt = 0; //No current
                if (CP_Mode == 0x6)
                    lim_state = 0; //Reset to state 0 if we get a static pilot
                                   //if(I_avail_tmp>10 && I_avail_tmp<500) lim_stateCnt++;

                if (ChargeType == 0x09)
                    lim_stateCnt++;
                if (lim_stateCnt > 25) //2 secs efacec critical! 20 works. 50 does not.
                {
                    lim_state++; //next state after 4 secs
                    lim_stateCnt = 0;
                }
            }
            break;

            case 2:
            { //
                Chg_Phase = CCS_PHASE_CABLE_TEST;//ChargePhase::CableTest;
                CONT_Ctrl = 0x0;  //dc contactor mode control required in DC
                FC_Cur = 0;       //ccs current request from web ui for now.
                EOC_Time = 0x1E;  //end of charge timer 30 mins
                Bulk_SOCt = 1800; //Set bulk SOC timer to 30 minutes.
                Full_SOCt = 2400; //Set full SOC timer to 40 minutes.
                Timer_1Sec = 5;   //Load the 1 second loop counter. 5 loops=1sec.
                Timer_60Sec = 60; //Load the 60 second loop counter. 5 loops=1sec.
                CHG_Status = CCS_STATUS_INIT;//ChargeStatus::Init;
                CHG_Req = CCS_REQUEST_CHARGE;//ChargeRequest::Charge;
                CHG_Ready = CCS_READY_RDY;//ChargeReady::Rdy;
                CHG_Pwr = 44000 / 25; //44kw approx power
                CCSI_Spnt = 0;        //No current
                if (Cont_Volts > 0)
                    lim_state++; //we wait for the contactor voltage to rise before hitting next state.
            }
            break;

            case 3:
            { //I don't like this state CableTest here. Should it remain in Initialisation ....
                Chg_Phase = CCS_PHASE_CABLE_TEST;//ChargePhase::CableTest;
                CONT_Ctrl = 0x0; //dc contactor mode control required in DC
                FC_Cur = 0;      //ccs current request from web ui for now.
                                 // EOC_Time=0x1E;//end of charge timer
                CHG_Status = CCS_STATUS_INIT;//ChargeStatus::Init;
                CHG_Req = CCS_REQUEST_CHARGE;//ChargeRequest::Charge;
                CHG_Ready = CCS_READY_RDY;//ChargeReady::Rdy;
                CHG_Pwr = 44000 / 25; //39kw approx power
                CCSI_Spnt = 0;        //No current

                if (Cont_Volts == 0)
                    lim_stateCnt++; //we wait for the contactor voltage to return to 0 to indicate end of cable test
                if (lim_stateCnt > 20)
                {
                    if (CCS_Iso == 0x1)
                        lim_state++; //next state after 2 secs if we have valid iso test
                    lim_stateCnt = 0;
                }
            }
            break;

            case 4:
            {
                Chg_Phase = CCS_PHASE_SUBPOENA;//ChargePhase::Subpoena; //precharge phase in this state
                CONT_Ctrl = 0x0;                   //dc contactor mode control required in DC
                FC_Cur = 0;                        //ccs current request from web ui for now.
                                                   // EOC_Time=0x1E;//end of charge timer
                CHG_Status = CCS_STATUS_INIT;//ChargeStatus::Init;
                CHG_Req = CCS_REQUEST_CHARGE;//ChargeRequest::Charge;
                CHG_Ready = CCS_READY_RDY;//ChargeReady::Rdy;
                CHG_Pwr = 44000 / 25; //49kw approx power
                CCSI_Spnt = 0;        //No current

                //if ((Param::GetInt(Param::udc) - Cont_Volts) < 20)
                 if ((BMS[0].packVolt - Cont_Volts) < 20)
                {
                    lim_stateCnt++; //we wait for the contactor voltage to be 20v or less diff to main batt v
                }
                else
                {
                    // If the contactor voltage wanders out of range start again
                    lim_stateCnt = 0;
                }

                // Wait for contactor voltage to be stable for 2 seconds
                if (lim_stateCnt > 20)
                {
                    lim_state++; //next state after 2 secs
                    lim_stateCnt = 0;
                }
            }
            break;
            case 5:
            {
                //precharge phase in this state but voltage close enough to close contactors
                Chg_Phase = CCS_PHASE_SUBPOENA;//ChargePhase::Subpoena;
                CONT_Ctrl = 0x2; //dc contactor closed
                FC_Cur = 0;      //ccs current request from web ui for now.
                                 // EOC_Time=0x1E;//end of charge timer
                CHG_Status = CCS_STATUS_INIT;//ChargeStatus::Init;
                CHG_Req = CCS_REQUEST_CHARGE;//ChargeRequest::Charge;
                CHG_Ready = CCS_READY_RDY;//ChargeReady::Rdy;
                CHG_Pwr = 44000 / 25; //49kw approx power
                CCSI_Spnt = 0;        //No current

                // Once the contactors report as closed we're OK to proceed to energy transfer
                if (PARAM.CCS_Contactor)//(Param::GetBool(Param::CCS_Contactor))
                {
                    lim_state++;
                }
            }
            break;

            case 6:
            {
                Chg_Phase = CCS_PHASE_ENGERGY_TRANSFER;//ChargePhase::EnergyTransfer;
                CONT_Ctrl = 0x2; //dc contactor to close mode
                //FC_Cur=Param::GetInt(Param::CCS_ICmd);//ccs manual control
                FC_Cur = CCSI_Spnt; //Param::GetInt(Param::CCS_ICmd);//ccs auto ramp
                CCS_Pwr_Con();      //ccs power control subroutine
                Chg_Timers();       //Handle remaining time timers.
                                    //  EOC_Time=0x1E;//end of charge timer
                CHG_Status = CCS_STATUS_READY;//ChargeStatus::Rdy;
                CHG_Req = CCS_REQUEST_CHARGE;//ChargeRequest::Charge;
                CHG_Ready = CCS_READY_RDY;//ChargeReady::Rdy;
                CHG_Pwr = 44000 / 25; //49kw approx power
                //we chill out here charging.

                if ((!RunCh) || CCS_IntStat == 0x02) //if we have a request to terminate from the web ui or the evse then move to next state.
                {
                    FC_Cur = 0;  //set current to 0
                    lim_state++; //move to state 7 (shutdown)
                }
            }
            break;

            case 7: //shutdown state
            {
                Chg_Phase = CCS_PHASE_SHUTDOWN;//ChargePhase::Shutdown;
                CONT_Ctrl = 0x2; //dc contactor to close mode
                FC_Cur = 0;      //current command to 0
                EOC_Time = 0x1E; //end of charge timer
                CHG_Status = CCS_STATUS_INIT;
                CHG_Req = CCS_REQUEST_CHARGE;//ChargeRequest::Charge;
                CHG_Ready = CCS_READY_RDY;//ChargeReady::Rdy;
                CHG_Pwr = 44000 / 25; //49kw approx power
                lim_stateCnt++;
                if (lim_stateCnt > 10) //wait 2 seconds
                {
                    lim_state++; //next state after 2 secs
                    lim_stateCnt = 0;
                }
            }
            break;

            case 8: //shutdown state
            {
                Chg_Phase = CCS_PHASE_SHUTDOWN;//ChargePhase::Shutdown;
                CONT_Ctrl = 0x1; //dc contactor to open with diag mode
                FC_Cur = 0;      //current command to 0
                EOC_Time = 0x1E; //end of charge timer
                CHG_Status = CCS_STATUS_INIT;
                CHG_Req = CCS_REQUEST_CHARGE;//ChargeRequest::Charge;
                CHG_Ready = CCS_READY_NOT_RDY;//ChargeReady::NotRdy;
                CHG_Pwr = 44000 / 25; //49kw approx power
                lim_stateCnt++;
                if (Cont_Volts == 0)
                    lim_stateCnt++; //we wait for the contactor voltage to return to 0 to indicate contactors open
                if (lim_stateCnt > 10)
                {
                    lim_state++; //next state after 2 secs
                    lim_stateCnt = 0;
                }
            }
            break;

            case 9: //shutdown state
            {
                Chg_Phase = CCS_PHASE_STANDY;//ChargePhase::Standby;
                CONT_Ctrl = 0x0; //dc contactor to open mode
                FC_Cur = 0;      //current command to 0
                EOC_Time = 0x1E; //end of charge timer
                CHG_Status = CCS_STATUS_INIT;
                CHG_Req = CCS_REQUEST_END_CHARGE;//ChargeRequest::EndCharge;
                CHG_Ready = CCS_READY_NOT_RDY;//ChargeReady::NotRdy;
                CHG_Pwr = 0; //0 power
                return No_Chg;
            }
            break;
            }

            if (RunCh)
                return DC_Chg; //set dc charge mode if we are enabled on webui
            if ((!RunCh) && lim_state == 9)
                return No_Chg; //set no charge mode if we are disabled on webui and in state 9 of dc machine
        }

        if (!PARAM.PlugDet)//(!Param::GetBool(Param::PlugDet)) //if we  plug remove shut down
        {
            lim_state = 0; //return to state 0
            PARAM.CCS_State =  lim_state;   //Param::SetInt(Param::CCS_State, lim_state);
            Chg_Phase = CCS_PHASE_STANDY;//ChargePhase::Standby;
            CONT_Ctrl = 0x0; //dc contactor mode 0 in off
            FC_Cur = 0;      //ccs current request zero
            EOC_Time = 0x00;
            CHG_Status = CCS_STATUS_NOT_READY;//ChargeStatus::NotRdy;
            CHG_Req = CCS_REQUEST_END_CHARGE;//ChargeRequest::EndCharge;
            CHG_Ready = CCS_READY_RDY;//ChargeReady::NotRdy;
            CHG_Pwr = 0;
            return No_Chg;
        }
    }
    // If nothing matches then we aren't charging
    return No_Chg;
}

void CCS_Pwr_Con() //here we control ccs charging during state 6.
{
    uint16_t Tmp_Vbatt = BMS[0].packVolt;//Param::GetInt(Param::udc); //Actual measured battery voltage by isa shunt
    uint16_t Tmp_Vbatt_Spnt = Voltspnt;//Param::GetInt(Param::Voltspnt);
    uint16_t Tmp_ICCS_Lim = PARAM.CCS_ILim;//Param::GetInt(Param::CCS_ILim);
    uint16_t Tmp_ICCS_Avail = PARAM.CCS_I_Avail;//Param::GetInt(Param::CCS_I_Avail);
    //int16_t Tmp_Ibatt=Param::GetInt(Param::idc);

    if (CCSI_Spnt > Tmp_ICCS_Lim)
        CCSI_Spnt = Tmp_ICCS_Lim; //clamp setpoint to current lim paramater.
    if (CCSI_Spnt > 150)
        CCSI_Spnt = 150; //never exceed 150amps for now.
    if (CCSI_Spnt >= Tmp_ICCS_Avail)
        CCSI_Spnt = Tmp_ICCS_Avail; //never exceed available current
    if (CCSI_Spnt > 250)
        CCSI_Spnt = 0; //crude way to prevent rollover
    if ((Tmp_Vbatt < Tmp_Vbatt_Spnt) && (CCS_Ilim == 0x0) && (CCS_Plim == 0x0))
        CCSI_Spnt++; //increment if voltage lower than setpoint and power and current limts not set from charger.
    if (Tmp_Vbatt > Tmp_Vbatt_Spnt)
        CCSI_Spnt--; //decrement if voltage equal to or greater than setpoint.
    if (CCS_Ilim == 0x1)
        CCSI_Spnt--; //decrement if current limit flag is set
    if (CCS_Plim == 0x1)
        CCSI_Spnt--; //decrement if Power limit flag is set
    PARAM.CCS_Ireq =  CCSI_Spnt;//Param::SetInt(Param::CCS_Ireq, CCSI_Spnt);
}

void Chg_Timers()
{
    Timer_1Sec--; //decrement the loop counter

    if (Timer_1Sec == 0) //1 second has elapsed
    {
        Timer_1Sec = 5;
        Bulk_SOCt--; //Decrement timers. Just on time for now will be current based in final version
        Full_SOCt--;
        Timer_60Sec--; //decrement the 1 minute counter
        if (Timer_60Sec == 0)
        {
            Timer_60Sec = 60;
            EOC_Time--; //decrement end of charge minutes timer
        }
    }
}
