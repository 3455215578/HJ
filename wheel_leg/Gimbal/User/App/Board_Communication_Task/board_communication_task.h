#ifndef _BOARD_COMMUNICATION_TASK_H
#define _BOARD_COMMUNICATION_TASK_H

#include <stdint-gcc.h>
#include "bsp_can.h"

union I16 {
    uint8_t data[2];
    int16_t value;
};

/** ��䷢�ͺ��� **/
void Send_Chassis_Speed(int16_t vx_channel, int16_t yaw_channel, char sl, char sr);
void Send_Control(int32_t W, int32_t A, int32_t S, int32_t D);

/** �����պ��� **/
void Chassis_to_Gimbal_Can(uint32_t can_id, const uint8_t *rx_data);

#endif
