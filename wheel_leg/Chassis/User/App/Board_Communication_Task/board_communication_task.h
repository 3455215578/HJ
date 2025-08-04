#ifndef _BOARD_COMMUNICATION_TASK_H
#define _BOARD_COMMUNICATION_TASK_H

#include <stdint-gcc.h>
#include <stdbool.h>

union I16{
    uint8_t data[2];
    int16_t value;
};

typedef struct
{
    union I16 vx_channel; // ǰ��
    union I16 leg_channel; // �ȳ�

    char sr;
    bool gimbal_init_flag;

    float yaw_relative_angle; // yaw��������������ԽǶ�



}Gimbal_Unpack_Data;

void Gimbal_Data_Unpack(const uint8_t *rx_data);

#endif
