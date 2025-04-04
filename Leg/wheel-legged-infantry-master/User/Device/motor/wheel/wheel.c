#include "wheel.h"
#include <cmsis_os.h>

static Lk9025 wheel[2];

void left_wheel_init()
{
    lk9025_init(&wheel[0], WHEEL_L_RECEIVE);
}

void right_wheel_init()
{
    lk9025_init(&wheel[1], WHEEL_R_RECEIVE);
}

void left_wheel_enable()
{
    lk9025_set_enable(CAN_1, WHEEL_L_SEND);
}

void right_wheel_enable()
{
    lk9025_set_enable(CAN_1, WHEEL_R_SEND);
}

void wheel_stop()
{
    lk9025_stop(CAN_1,WHEEL_L_SEND);
    lk9025_stop(CAN_1,WHEEL_R_SEND);
}

void set_left_wheel_torque(float torque_nm_L) {
    lk9025_torque_set(CAN_1,WHEEL_L_SEND,torque_nm_L);
    osDelay(1);
}

void set_right_wheel_torque(float torque_nm_R) {
    lk9025_torque_set(CAN_1,WHEEL_R_SEND,torque_nm_R);
    osDelay(1);
}


Lk9025 *get_wheel_motors(){
    return wheel;
}