#ifndef WHEEL_H
#define WHEEL_H

#include "../../can_device/can_device.h"
#include "lk9025.h"

enum WheelMotorIndex{
    L = 0,
    R = 1,
};

/** ��챵����ʼ�� **/
void wheel_init(void);

/** ������챵��ָ�� **/
Lk9025 *get_wheel_motors();

extern Lk9025 wheel[2];

#endif
