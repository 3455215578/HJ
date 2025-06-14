#ifndef LK_9025_H
#define LK_9025_H

#include <stdint-gcc.h>
#include "can_device.h"
/********************  һ������ز���  *****************************/
/** ���س���(1A��Ӧ������ ��λ Nm) **/
#define LK_TORQUE_CONSTANT 0.32f

/** 1A������Ӧ����ֵ value / A **/
#define LK_CURRENT_2_DATA 62.5f

typedef struct{

    uint32_t id;

    /** ���ٶ� **/
    float angular_vel;

    /** ���� **/
    float torque;
} Lk9025;

void lk9025_init(Lk9025 *motor, uint32_t device_id);

void lk9025_torque_set(Lk9025 *motor, float motor_torque);

/** ����ת�رջ� **/
void lk9025_multi_torque_set(float motor1_torque, float motor2_torque);

/** ��챵���������� **/
void lk9025_info_update(Lk9025 *motor, uint8_t data[]);

#endif
