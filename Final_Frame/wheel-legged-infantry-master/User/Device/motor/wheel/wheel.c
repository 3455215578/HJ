#include "wheel.h"
#include <cmsis_os.h>

static Lk9025 wheel[2];

void wheel_init()
{
    lk9025_init(&wheel[0], WHEEL_L_RECEIVE);
    lk9025_init(&wheel[1], WHEEL_R_RECEIVE);
}

void wheel_enable()
{
    lk9025_set_enable(CAN_1, WHEEL_L_SEND);
    lk9025_set_enable(CAN_1, WHEEL_R_SEND);
}


void wheel_stop()
{
    lk9025_stop(CAN_1,WHEEL_L_SEND);
    lk9025_stop(CAN_1,WHEEL_R_SEND);
}

// 发驱动轮力矩不要加osDelay
void set_wheel_torque(float torque_nm_L, float torque_nm_R) {
    lk9025_torque_set(CAN_1,WHEEL_L_SEND,torque_nm_L);
    lk9025_torque_set(CAN_1,WHEEL_R_SEND,torque_nm_R);
}

Lk9025 *get_wheel_motors(){
    return wheel;
}