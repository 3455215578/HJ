#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint-gcc.h>
#include "chassis.h"



void gimbal_msg_unpack(uint32_t id, uint8_t data[]);

GimbalMsg *get_gimbal_msg();

extern void gimbal_task(void const *pvParameters);

#endif
