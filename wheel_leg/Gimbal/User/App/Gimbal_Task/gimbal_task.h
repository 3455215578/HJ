#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "PID.h"
#include "filter.h"
#include "DJI_Motor.h"
#include "key_board.h"
#include "robot_def.h"

extern void Gimbal_Can_Msg(uint32_t can_id, uint8_t *can_msg);
extern void Chassis_to_Gimbal_Can(uint32_t can_id, const uint8_t *rx_data);

void Gimbal_task(void const*pvParameters);

#endif
