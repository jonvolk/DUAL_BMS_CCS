#include <can_setup.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxMsg, canRx) != HAL_OK)
        {
            Error_Handler();
        }
        //do stuff
        decodeVolt(&BMS[0], &rxMsg, canRx);
        decodeTemp(&BMS[0], &rxMsg, canRx);
    }

    if (hcan->Instance == CAN3)
    {
        if (HAL_CAN_GetRxMessage(&hcan3, CAN_RX_FIFO0, &rxMsg3, canRx3) != HAL_OK)
        {
            Error_Handler();
        }
        //do stuff
        decodeVolt(&BMS[1], &rxMsg3, canRx3);
        decodeTemp(&BMS[1], &rxMsg3, canRx3);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    if (hcan->Instance == CAN2)
    {
        if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &rxMsg2, canRx2) != HAL_OK)
        {
            Error_Handler();
        }
    }

    if (hcan->Instance == CAN3)
    {
        if (HAL_CAN_GetRxMessage(&hcan3, CAN_RX_FIFO1, &rxMsg3, canRx3) != HAL_OK)
        {
            Error_Handler();
        }
        //do stuff
        decodeVolt(&BMS[1], &rxMsg3, canRx3);
        decodeTemp(&BMS[1], &rxMsg3, canRx3);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void canSettings(void)
{

    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.TransmitGlobalTime = DISABLE;

    txMsgExt.IDE = CAN_ID_EXT;
    txMsgExt.RTR = CAN_RTR_DATA;
    txMsgExt.TransmitGlobalTime = DISABLE;

    sf.FilterBank = 0; // CAN1 Filter bank starts at 0
    sf.FilterMode = CAN_FILTERMODE_IDMASK;
    sf.FilterScale = CAN_FILTERSCALE_16BIT;
    sf.FilterIdLow = 0xffff;
    sf.FilterIdHigh = 0x1fff;
    sf.FilterMaskIdLow = 0x0000;
    sf.FilterMaskIdHigh = 0x0000;
    sf.FilterFIFOAssignment = CAN_RX_FIFO0;
    sf.SlaveStartFilterBank = 14;
    sf.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan1, &sf) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);

    //hcan2
    txMsg2.IDE = CAN_ID_STD;
    txMsg2.RTR = CAN_RTR_DATA;
    txMsg2.TransmitGlobalTime = DISABLE;

    txMsgExt2.IDE = CAN_ID_EXT;
    txMsgExt2.RTR = CAN_RTR_DATA;
    txMsgExt2.TransmitGlobalTime = DISABLE;

    sf2.FilterBank = 14; // CAN2 Filter bank starts at 14
    sf2.FilterMode = CAN_FILTERMODE_IDMASK;
    sf2.FilterScale = CAN_FILTERSCALE_16BIT;
    sf2.FilterIdLow = 0xffff;
    sf2.FilterIdHigh = 0x1fff;
    sf2.FilterMaskIdLow = 0x0000;
    sf2.FilterMaskIdHigh = 0x0000;
    sf2.FilterFIFOAssignment = CAN_RX_FIFO1;
    sf2.SlaveStartFilterBank = 14;
    sf2.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan2, &sf2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);

    //hcan3
    txMsg3.IDE = CAN_ID_STD;
    txMsg3.RTR = CAN_RTR_DATA;
    txMsg3.TransmitGlobalTime = DISABLE;

    txMsgExt3.IDE = CAN_ID_EXT;
    txMsgExt3.RTR = CAN_RTR_DATA;
    txMsgExt3.TransmitGlobalTime = DISABLE;

    sf3.FilterBank = 1; // CAN1 Filter bank starts at 0
    sf3.FilterMode = CAN_FILTERMODE_IDMASK;
    sf3.FilterScale = CAN_FILTERSCALE_32BIT;
    sf3.FilterIdLow = 0x0000;
    sf3.FilterIdHigh = 0x0000;
    sf3.FilterMaskIdLow = 0x0000;
    sf3.FilterMaskIdHigh = 0x0000;
    sf3.FilterFIFOAssignment = CAN_RX_FIFO0;
    sf3.SlaveStartFilterBank = 14;
    sf3.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan3, &sf3) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan3) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan3, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(CAN3_TX_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void c1tx(CAN_TxHeaderTypeDef *txMsg, uint8_t *canTx)
{

    HAL_CAN_AddTxMessage(&hcan1, txMsg, canTx, &canMailbox);
    txCycle++;
    if (txCycle >= 3)
    {
        HAL_Delay(1);
        txCycle = 0;
    }
}

void c2tx(CAN_TxHeaderTypeDef *txMsg2, uint8_t *canTx2)
{

    HAL_CAN_AddTxMessage(&hcan2, txMsg2, canTx2, &canMailbox2);
    txCycle2++;
    if (txCycle2 >= 3)
    {
        HAL_Delay(1);
        txCycle2 = 0;
    }
}

void c3tx(CAN_TxHeaderTypeDef *txMsg3, uint8_t *canTx3)
{

    HAL_CAN_AddTxMessage(&hcan3, txMsg3, canTx3, &canMailbox2);
    txCycle2++;
    if (txCycle3 >= 3)
    {
        HAL_Delay(1);
        txCycle3 = 0;
    }
}

void c1txExt(CAN_TxHeaderTypeDef *txMsgExt, uint8_t *canTx)
{

    HAL_CAN_AddTxMessage(&hcan1, txMsgExt, canTx, &canMailbox);
    txCycle++;
    if (txCycle >= 3)
    {
        HAL_Delay(1);
        txCycle = 0;
    }
}

void c2txExt(CAN_TxHeaderTypeDef *txMsg2Ext, uint8_t *canTx2)
{

    HAL_CAN_AddTxMessage(&hcan2, txMsg2Ext, canTx2, &canMailbox2);
    txCycle2++;
    if (txCycle2 >= 3)
    {
        HAL_Delay(1);
        txCycle2 = 0;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/***************** Filter Config ID mask: Allow All *******************
    sf.FilterBank = 0; // CAN1 Filter bank starts at 0 
    sf.FilterMode = CAN_FILTERMODE_IDMASK;
    sf.FilterScale = CAN_FILTERSCALE_32BIT;
    sf.FilterIdLow = 0xffff;  
    sf.FilterIdHigh = 0x1fff;
    sf.FilterMaskIdLow = 0x0000;
    sf.FilterMaskIdHigh = 0x0000;
    sf.FilterFIFOAssignment = CAN_RX_FIFO0;
    sf.SlaveStartFilterBank = 14;
    sf.FilterActivation = ENABLE;
*/

/***************** Filter Config ID list: Allow 4 discreet ID *******************
    sf1.FilterBank = 1;
    sf1.FilterMode = CAN_FILTERMODE_IDLIST; 
    sf1.FilterScale = CAN_FILTERSCALE_16BIT;
    sf1.FilterIdLow = 0xFD3<<5; 
    sf1.FilterIdHigh = 0X120<<5;
    sf1.FilterMaskIdLow = 0x184<<5;//X120<<5;
    sf1.FilterMaskIdHigh = 0x084<<5;//X120<<5;
    sf1.FilterFIFOAssignment = CAN_RX_FIFO0;
    sf1.SlaveStartFilterBank = 14;
    sf1.FilterActivation = ENABLE;
*/

/***************** Filter Config EXTID list: Allow 2 discreet ID *******************
    sf.FilterBank = 0;
    sf.FilterMode = CAN_FILTERMODE_IDLIST;
    sf.FilterScale = CAN_FILTERSCALE_32BIT;
    sf.FilterIdLow = ((0x18FF11F2 << 3) & 0xFFF8) | 4;   
    sf.FilterIdHigh = (0x18FF11F2 >> 13) & 0xFFFF;         
    sf.FilterMaskIdLow = ((0x18FF0FF2 << 3) & 0xFFF8) | 4; 
    sf.FilterMaskIdHigh = (0x18FF0FF2 >> 13) & 0xFFFF;     
    sf.FilterFIFOAssignment = CAN_RX_FIFO0;
    sf.SlaveStartFilterBank = 14;
    sf.FilterActivation = ENABLE;
*/