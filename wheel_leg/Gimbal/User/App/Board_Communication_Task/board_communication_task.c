#include "board_communication_task.h"

/* 0x110 0x111 */
void Gimbal_Send_Chassis(int16_t vx_channel, int16_t yaw_channel, char sl, char sr)
{
    /** ���巢�ͽṹ�� **/
    CAN_TxHeaderTypeDef  tx_message;

    tx_message.StdId = 0x110; //CAN ID
    tx_message.IDE = CAN_ID_STD; // ��׼֡
    tx_message.RTR = CAN_RTR_DATA; // ����֡
    tx_message.DLC = 0x06; // ���ݳ����ݶ�Ϊ�����ֽ�

    /** ���巢������ **/
    uint32_t send_mail_box;

    /** ���巢�͵������ֽ� **/
    uint8_t Send_data[8];

    union I16 ch;

    /** ǰ�� **/
    ch.value = vx_channel;
    Send_data[0] = ch.data[0];
    Send_data[1] = ch.data[1];

    /** ת�� **/
    ch.value = yaw_channel;
    Send_data[2] = ch.data[0];
    Send_data[3] = ch.data[1];

    /** ���Ҳ�ť **/
    Send_data[4] = sl;
    Send_data[5] = sr;

    /** ��ȡ���� **/
    uint32_t can_send_mail = get_can_free_mail(&hcan2);

    /** �������� **/
    if (can_send_mail != 0) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, Send_data, &send_mail_box);
    }
}

void Send_Control(int32_t W, int32_t A, int32_t S, int32_t D) {
    CAN_TxHeaderTypeDef  tx_message;
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
    /** ��ȡ���� **/
    uint32_t can_send_mail = get_can_free_mail(&hcan2);

    /** �������� **/
    if (can_send_mail != 0) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, Send_data, &send_mail_box);
    }
}

void Chassis_to_Gimbal_Can(uint32_t can_id, const uint8_t *rx_data) {
    switch (can_id) {
        case 0x115:
        {
            break;
        }
        default:
        {
            break;
        }
    }
}
