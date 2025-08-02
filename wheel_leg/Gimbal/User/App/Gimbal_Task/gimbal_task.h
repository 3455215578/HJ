//
// Created by xhuanc on 2021/10/13.
//

#ifndef HERO_GIMBAL_H
#define HERO_GIMBAL_H

/*********************************************************************************************************
*                                              ����ͷ�ļ�
*********************************************************************************************************/
#include "Balance.h"
#include "PID.h"
#include "filter.h"
#include "DJI_Motor.h"
#include "key_board.h"

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/
/* ��̨�����ʼ��ʱ�� */
#define GIMBAL_TASK_INIT_TIME 800

/* ��̨������������ */
#define GIMBAL_PERIOD 1

/* pitch�� PID */
#define GIMBAL_PITCH_ANGLE_PID_KP           50.0f
#define GIMBAL_PITCH_ANGLE_PID_KI           0.0f
#define GIMBAL_PITCH_ANGLE_PID_KD           0.0f
#define GIMBAL_PITCH_ANGLE_MAX_IOUT         0.0f
#define GIMBAL_PITCH_ANGLE_MAX_OUT          9000.f

#define GIMBAL_PITCH_SPEED_PID_KP           30.0f
#define GIMBAL_PITCH_SPEED_PID_KI           0.0f
#define GIMBAL_PITCH_SPEED_PID_KD           0.0f
#define GIMBAL_PITCH_SPEED_MAX_IOUT         0.0f
#define GIMBAL_PITCH_SPEED_MAX_OUT          10000.f

/* yaw��PID */
#define GIMBAL_YAW_ANGLE_PID_KP             20.0f // 30 35 35 35 35
#define GIMBAL_YAW_ANGLE_PID_KI             0.0f
#define GIMBAL_YAW_ANGLE_PID_KD             20.0f // 0 100 120 150 170
#define GIMBAL_YAW_ANGLE_MAX_IOUT           0.0f
#define GIMBAL_YAW_ANGLE_MAX_OUT            10000.f

#define GIMBAL_YAW_SPEED_PID_KP             300.0f // 500
#define GIMBAL_YAW_SPEED_PID_KI             0.0f
#define GIMBAL_YAW_SPEED_PID_KD             0.0f
#define GIMBAL_YAW_SPEED_MAX_IOUT           0.0f
#define GIMBAL_YAW_SPEED_MAX_OUT            15000.0f

/* �� pitch ���ж�̬��λ */
#define MAX_ABS_ANGLE 30
#define MIN_ABS_ANGLE (-23)

/* ECD ����ֵ */
#define PITCH_OFFSET_ECD 0
#define YAW_OFFSET_ECD 7496

/*********************************************************************************************************
*                                              ö�ٽṹ�嶨��
*********************************************************************************************************/
/* ����Ļ�ȡֵ�ͼ���ֵ */
typedef struct {
    DJI_Motor_t motor_measure;    // �������ʵ��Ϣ

    pid_type_def speed_p;   //�ٶȻ� PID ���Ʋ���
    pid_type_def angle_p;   //�ǶȻ� PID ���Ʋ���
    fp32 gyro_set;          //ת������
    int16_t target_current;   //���õ���ֵ

    fp32 relative_angle_set; //��    �趨
    fp32 relative_angle_get; //��    ��ȡ
    fp32 absolute_angle_set; //      rad
    fp32 absolute_angle_get; //      ��̨��
}Motor_Gimbal_t;

typedef struct {
    /* �����Ϣ */
    Motor_Gimbal_t yaw;
    Motor_Gimbal_t pitch;

    /* ��̨״̬��Ϣ */
    Gimbal_Mode_e mode;
    Gimbal_Mode_e last_mode;

    /* ��̬�� */
    fp32 absolute_gyro_yaw;
    fp32 absolute_gyro_pitch;

    first_order_filter_type_t mouse_in_y;
    first_order_filter_type_t mouse_in_x;

    first_order_filter_type_t auto_pitch;
    first_order_filter_type_t auto_yaw[2];
    first_kalman_filter_t filter_autoYaw;

    first_order_filter_type_t filter_pitch_gyro_in;
    first_order_filter_type_t filter_yaw_gyro_in;

    first_order_filter_type_t pitch_first_order_set;
    first_order_filter_type_t pitch_current_first_order_set;

}gimbal_t;


/*********************************************************************************************************
*                                              ������������ļ�
*********************************************************************************************************/
extern gimbal_t gimbal;
extern void Gimbal_Can_Msg(uint32_t can_id, uint8_t *can_msg);
extern void Chassis_to_Gimbal_Can(uint32_t can_id, const uint8_t *rx_data);

void Gimbal_task(void const*pvParameters);

#endif //HERO_GIMBAL_H
