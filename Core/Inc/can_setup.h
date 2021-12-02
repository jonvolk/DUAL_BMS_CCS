
#ifndef __CAN_SETUP_H
#define __CAN_SETUP_H


#include "main.h"
#include "stdio.h"
#include "bms.h"

 
//CAN_HandleTypeDef hcan;
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_HandleTypeDef hcan3;
CAN_FilterTypeDef sf;  //= {0};  //CAN Bus Filter
CAN_FilterTypeDef sf2; // = {0}; //CAN2 Bus Filter
CAN_FilterTypeDef sf3; // CAN3 Bus Filter
CAN_FilterTypeDef sf4; // not used
CAN_FilterTypeDef sf5; // not used

CAN_RxHeaderTypeDef rxMsg; //CAN Bus Receive Header
CAN_TxHeaderTypeDef txMsg; //CAN Bus Transmit Header
CAN_TxHeaderTypeDef txMsgExt;

CAN_RxHeaderTypeDef rxMsg2; //CAN2 Bus Receive Header
CAN_TxHeaderTypeDef txMsg2; //CAN2 Bus Transmit Header
CAN_TxHeaderTypeDef txMsgExt2;

CAN_RxHeaderTypeDef rxMsg3; //CAN3 Bus Receive Header
CAN_TxHeaderTypeDef txMsg3; //CAN3 Bus Transmit Header
CAN_TxHeaderTypeDef txMsgExt3;

uint8_t canRx[8]; //= {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t canRx2[8];
uint8_t canRx3[8];

uint32_t canMailbox; //CAN Bus Mail box variable
uint32_t canMailbox2;
uint32_t canMailbox3;

//uint8_t canTx[8];
//uint8_t canTx2[8];
//uint8_t canTx3[8];


int txCycle; //CAN TX fill level counter
int txCycle2;
int txCycle3;


void canSettings(void);
//void c1tx(CAN_TxHeaderTypeDef *txMsg, uint8_t *canTx);
void can1tx(uint16_t msgId, uint8_t DLC, uint8_t *canTx);
//void c2tx(CAN_TxHeaderTypeDef *txMsg2, uint8_t *canTx2);
void can2tx(uint16_t msgId, uint8_t DLC, uint8_t *canTx2);
//void c3tx(CAN_TxHeaderTypeDef *txMsg3, uint8_t *canTx3);
void can3tx(uint16_t msgId, uint8_t DLC, uint8_t *canTx3);
void c1txExt(CAN_TxHeaderTypeDef *txMsgExt, uint8_t *canTx);
void c2txExt(CAN_TxHeaderTypeDef *txMsg2Ext, uint8_t *canTx2);
void c3txExt(CAN_TxHeaderTypeDef *txMsg3Ext, uint8_t *canTx3);

#endif
