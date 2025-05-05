#ifndef _BOARD_COMMUNICATION_TASK_H
#define _BOARD_COMMUNICATION_TASK_H

#include <stdint.h>

void Gimbal_to_Chassis_Can(uint32_t can_id, const uint8_t *rx_data);

#endif
