//
// Created by xhuanc on 2021/9/27.
//

#include "can_receive.h"
#include "Gimbal_Task.h"

void CAN1_RX1_IRQHandler() {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &rx_header, rx_data);

// ��̨�������
#ifdef GIMBAL
    Gimbal_Can_Msg(rx_header.StdId,rx_data);
#endif //!GIMBAL

// ���̵������
#ifdef CHASSIS
    Chassis_Can_Msg(rx_header.StdId,rx_data);
#endif //!CHASSIS
}


void CAN2_RX0_IRQHandler() {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);

// �ϰ����
#ifdef GIMBAL
    // ���°�ͨ������
    // Chassis_to_Gimbal_Can(rx_header.StdId, rx_data);
#endif //!GIMBAL

// �°����
#ifdef CHASSIS
    Gimbal_to_Chassis_Can(rx_header.StdId, rx_data);
    Cap_Data_Get(rx_header.StdId, rx_data);
#endif //!CHASSIS
}