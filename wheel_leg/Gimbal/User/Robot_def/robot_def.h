#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "DJI_Motor.h"
#include "pid.h"
#include "user_lib.h"
#include "filter.h"
#include "protocol_Balance.h"

/* ��̨�����ʼ��ʱ�� */
#define GIMBAL_TASK_INIT_TIME 800

/* ��̨������������ */
#define GIMBAL_PERIOD 1

/**********************************************************************************
 *                                      ��̨                                       *
 **********************************************************************************/

/** �궨�� **/

/* Pitch��̬��λ */
#define MAX_ABS_ANGLE 30
#define MIN_ABS_ANGLE (-23)

/* ��̨���б���ֵ */
#define PITCH_OFFSET_ECD 0
#define YAW_OFFSET_ECD 7460


/** PID���� **/

// Pitch
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

// Yaw
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


/** ��̨����ģʽ **/
typedef enum {
    GIMBAL_DISABLE,   // ʧ��
    GIMBAL_INIT,      // ��ʼ��
    GIMBAL_ENABLE,    // ʹ��
    GIMBAL_AUTO,    //��̨����ģʽ
    GIMBAL_FIRE,    //��̨��ģʽ
}Gimbal_Mode_e;

/** ��̨����ṹ�� **/
typedef struct {
    DJI_Motor_t motor_measure;    // �������ʵ��Ϣ

    float gyro; // ��/s

    Pid speed_pid;   //�ٶȻ� PID
    Pid angle_pid;   //�ǶȻ� PID

    int16_t target_current; // ��������ֵ

    fp32 absolute_angle_get; // �� IMU�Ƕ�ֵ
    fp32 relative_angle_get; // �� �����ϵ縴λλ�õĽǶȲ�ֵ

    fp32 absolute_angle_set; // ��

    // �ϵ縴λ��־λ
    bool reset_finished;

}Motor_Gimbal_t;


/** ��̨�ṹ�� **/
typedef struct {

    /** ��̨��� **/
    Motor_Gimbal_t yaw;
    Motor_Gimbal_t pitch;

    /** ��̨����ģʽ **/
    Gimbal_Mode_e gimbal_ctrl_mode;
    Gimbal_Mode_e gimbal_last_ctrl_mode;

    /** �˲��� **/

    // ��������˲�
    first_order_filter_type_t mouse_in_y;
    first_order_filter_type_t mouse_in_x;

    // ���Ӿ����ص�yaw��pitch�˲�
    first_order_filter_type_t auto_pitch;
    first_order_filter_type_t auto_yaw[2];

    // ���ٶ��˲�
    first_order_filter_type_t pitch_gyro_filter;
    first_order_filter_type_t yaw_gyro_filter;

    bool init_flag;

}gimbal_t;

extern gimbal_t gimbal;

/**********************************************************************************
 *                                    �������                                      *
 **********************************************************************************/

/** �궨�� **/

// Ħ����ת��
#define FIRE_SPEED_L  4700
#define FIRE_SPEED_R  FIRE_SPEED_L

// ����ת��
#define TRIGGER_SPEED -1000

// 3508������תһȦ����ֵ��8192  ���ٱ�1:19  ������ת19Ȧ������תһȦ  19��8192
// 2006������תһȦ����ֵ��8192  ���ٱ�1:36  ������ת36Ȧ������תһȦ  36��8192/8
#define DEGREE_45_TO_ENCODER  36864.f
#define DEGREE_90_TO_ENCODER 73728.f

/** PID���� **/
#define TRIGGER_ANGLE_PID_KP        1.7f
#define TRIGGER_ANGLE_PID_KI        0.f
#define TRIGGER_ANGLE_PID_KD        0.5f
#define TRIGGER_ANGLE_PID_MAX_IOUT  0
#define TRIGGER_ANGLE_PID_MAX_OUT   5000

#define TRIGGER_SPEED_PID_KP        10.f
#define TRIGGER_SPEED_PID_KI        0.f
#define TRIGGER_SPEED_PID_KD        18.f
#define TRIGGER_SPEED_PID_MAX_IOUT  0
#define TRIGGER_SPEED_PID_MAX_OUT   10000

/** Ħ����״̬ **/
// ��Ϊ���ж�Ħ����ģʽ��ʱ���˳���жϲ���ģʽ�����԰�Ħ���ֿ���������������������ر�(�ݶ����������������ܻ�ֿ�����������)
typedef enum{
    Fire_OFF=0,             //��������ر�
    Fire_ON=1,              //�����������
}Fir_Wheel_Mode_e;

/** ����״̬ **/
typedef enum{
    SHOOT_CLOSE=0,          //����ر�
    SHOOT_CONTINUE,         //����ָ��

    SHOOT_READY_TO_SINGLE,  //����ָ��
    SHOOT_BLOCK,            //������ת

    SHOOT_SINGLE,           //������
    SHOOT_INVERSING,        //��ת��

    SHOOT_OVER,             //�������
    SHOOT_FAIL              //���̻���
}Shoot_Cmd_e;


/** �����������ṹ�� **/
typedef struct {
    DJI_Motor_t motor_measure;    // �������ʵ��Ϣ

    fp32 target_speed;            // Ħ����ת���趨ֵ
    Pid speed_pid;                // �����ٶȻ�pid
    Pid angle_pid;                // ���̽ǶȻ�pid
    int16_t target_current;       // ��������ֵ

}Motor_Launcher_t;

/** ��������ṹ�� **/
typedef struct {

    Motor_Launcher_t fire_l;
    Motor_Launcher_t fire_r;
    Motor_Launcher_t trigger;

    Fir_Wheel_Mode_e fir_wheel_last_mode;
    Fir_Wheel_Mode_e fir_wheel_mode;

    Shoot_Cmd_e trigger_last_mode;
    Shoot_Cmd_e trigger_mode;


}launcher_t;

extern launcher_t launcher;


/**********************************************************************************
 *                                    Vision                                      *
 **********************************************************************************/

extern robot_ctrl_info_t robot_ctrl;

#endif
