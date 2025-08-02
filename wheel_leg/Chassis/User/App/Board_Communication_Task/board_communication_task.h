#ifndef _BOARD_COMMUNICATION_TASK_H
#define _BOARD_COMMUNICATION_TASK_H

#include <stdint-gcc.h>

union I16{
    uint8_t data[2];
    int16_t value;
};

void Gimbal_to_Chassis_Can(uint32_t can_id, const uint8_t *rx_data);

#endif
