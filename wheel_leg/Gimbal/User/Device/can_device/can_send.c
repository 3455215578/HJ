//
// Created by Gularx on 2024/12/10.
//
#include "can_send.h"

/******************include******************/
static CAN_TxHeaderTypeDef  tx_message;
/* 0x110 0x111 */
void Send_Chassis_Speed(int16_t ch1, int16_t ch2, int16_t ch4, char sl, char sr){
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    union I16 ch;
    tx_message.StdId = 0x110;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    ch.value = ch1; /* 前后 */
    Send_data[0] = ch.data[0];
    Send_data[1] = ch.data[1];
    ch.value = ch2; /* 转向 */
    Send_data[2] = ch.data[0];
    Send_data[3] = ch.data[1];
    ch.value = ch4; /* 小陀螺 */
    Send_data[4] = ch.data[0];
    Send_data[5] = ch.data[1];
    Send_data[6] = sl;
    Send_data[7] = sr;
    // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
    HAL_CAN_AddTxMessage(&hcan2, &tx_message, Send_data, &send_mail_box);
}

void Send_Control(int32_t W, int32_t A, int32_t S, int32_t D) {
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    tx_message.StdId = 0x111;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    Send_data[0] = (uint8_t)(W & 0xFF);
    Send_data[1] = (uint8_t)(A & 0xFF);
    Send_data[2] = (uint8_t)(S & 0xFF);
    Send_data[3] = (uint8_t)(D & 0xFF);
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
    HAL_CAN_AddTxMessage(&hcan2, &tx_message, Send_data, &send_mail_box);
}