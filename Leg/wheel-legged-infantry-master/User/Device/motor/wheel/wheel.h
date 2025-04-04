#ifndef WHEEL_H
#define WHEEL_H

#include "../../can_device/can_device.h"
#include "lk_9025.h"

void left_wheel_init();
void right_wheel_init();

void left_wheel_enable();
void right_wheel_enable();

void wheel_stop();

void set_left_wheel_torque(float torque_nm_L);
void set_right_wheel_torque(float torque_nm_R);

Lk9025 *get_wheel_motors();

#endif //WHEEL_H
