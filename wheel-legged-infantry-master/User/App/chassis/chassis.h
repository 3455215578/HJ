#ifndef CHASSIS_H
#define CHASSIS_H

#include "vmc.h"
#include "robot_def.h"

void chassis_init();

Chassis *get_chassis();

void is_chassis_off_ground();

extern void chassis_task(void const *pvParameters);

#endif //CHASSIS_H
