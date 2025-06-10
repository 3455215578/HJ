#ifndef DM_8009_H
#define DM_8009_H

#include "stdint-gcc.h"
#include "can_device.h"

/********************  һ������ز���  *****************************/
/** ���س���(1A��Ӧ������ ��λ Nm) **/
#define DM8009_TORQUE_CONSTANT 1.261575f

/** ������ ��λ��A **/
#define DM8009_IMAX 41.044777f

/** 1A��Ӧ�Ŀ�������ֵ **/
#define DM8009_CURRENT_2_DATA 16384.0f / DM8009_IMAX

/** תһȦ�ı�����ֵ **/
#define ENCODER_PER_ROUND 8192.0f

/** rpm(rad per min) to (rad per second) **/
#define RPM_TO_RAD_PER_S (2 * PI) / 60.0f


typedef struct{
    uint32_t id;
    /** ����λ��(0 ~ pi/2)  (-pi/2 ~ 0) **/
    float pos_r;

    /** �ؽڵ�����ٶ� rad/s ��? **/
    float angular_vel;

    /** �ؽڵ���������� **/
    float torque;
} Dm8009;

/** ��ʼ�����ID **/
void dm8009_init(Dm8009 *motor, uint32_t device_id);

/** ʹ�ܵ�� **/
void set_dm8009_enable(Dm8009* motor);

/** ʧ�ܵ�� **/
void set_dm8009_disable(Dm8009* motor);

/** λ���ٶ�ģʽģʽ **/
void set_dm8009_pos_speed(Dm8009* motor,
                          float pos_rad,
                          float speed_rps);

/** �����MITģʽ **/
void set_dm8009_MIT(Dm8009* motor,
                    float pos,
                    float speed,
                    float kp,
                    float kd,
                    float torque);

/** �ؽڵ���������� **/
void dm8009_info_update(Dm8009* motor, uint8_t data[]);


#endif
