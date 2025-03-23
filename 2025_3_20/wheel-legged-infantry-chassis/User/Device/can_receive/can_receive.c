#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>

#include "joint.h"
#include "wheel.h"
#include "communication.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8]={0};

    if (hcan == &hcan1) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            // 接收轮毂电机反馈
            lk9025_can_msg_unpack(rx_header.StdId, rx_data);
            // 接收云台C板数据
            gimbal_msg_unpack(rx_header.StdId, rx_data);
        }
    } else if (hcan == &hcan2) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            // 接收关节电机反馈
            dm8009_can_msg_unpack(rx_header.StdId, rx_data);

        }
    }
}