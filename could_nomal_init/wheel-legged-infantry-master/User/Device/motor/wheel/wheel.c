#include <cmsis_os.h>
#include "wheel.h"

static Lk9025 wheel[2];

void wheel_disable(){
    lk9025_init(&wheel[0], WHEEL_L_RECEIVE);
    lk9025_init(&wheel[1], WHEEL_R_RECEIVE);

    lk9025_disable(CAN_1, WHEEL_L_SEND);
    osDelay(10);
    lk9025_disable(CAN_1, WHEEL_R_SEND);
}

void wheel_enable() {
    lk9025_init(&wheel[0], WHEEL_L_RECEIVE);
    lk9025_init(&wheel[1], WHEEL_R_RECEIVE);

    lk9025_set_enable(CAN_1,WHEEL_L_SEND);
    lk9025_set_enable(CAN_1,WHEEL_R_SEND);
  }

void set_wheel_torque(float torque_nm_L, float torque_nm_R) {
  lk9025_torque_set(CAN_1,WHEEL_L_SEND,torque_nm_L);
  lk9025_torque_set(CAN_1,WHEEL_R_SEND,torque_nm_R);
}

Lk9025 *get_wheel_motors(){
  return wheel;
}