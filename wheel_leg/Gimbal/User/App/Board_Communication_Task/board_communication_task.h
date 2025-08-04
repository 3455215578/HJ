#ifndef _BOARD_COMMUNICATION_TASK_H
#define _BOARD_COMMUNICATION_TASK_H

#include <stdint-gcc.h>
#include <stdbool.h>
#include "bsp_can.h"

union I16 {
    uint8_t data[2];
    int16_t value;
};

/** 板间发送函数 **/
void Send_Chassis_Data(int16_t vx_channel, int16_t leg_channel, char sr, bool gimbal_init_flag, float yaw_relative_angle);
void Send_Control(int32_t W, int32_t A, int32_t S, int32_t D);

/** 板间接收函数 **/
void Chassis_to_Gimbal_Can(uint32_t can_id, const uint8_t *rx_data);

#endif
