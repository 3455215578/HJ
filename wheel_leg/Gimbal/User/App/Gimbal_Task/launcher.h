#ifndef LAUNCHER_H
#define LAUNCHER_H

/*********************************************************************************************************
*                                              ����ͷ�ļ�
*********************************************************************************************************/
#include "Balance.h"
#include "DJI_Motor.h"
#include "user_lib.h"
#include "PID.h"
#include "SMC.h"

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/
/** Ħ����ת�� **/
#define FIRE_SPEED_L  4700
#define FIRE_SPEED_R  FIRE_SPEED_L
/** ����ת�� **/
#define TRIGGER_SPEED -1000

//// 3508������תһȦ����ֵ��8192  ���ٱ�1:19  ������ת19Ȧ������תһȦ  19��8192
//// 2006������תһȦ����ֵ��8192  ���ٱ�1:36  ������ת36Ȧ������תһȦ  36��8192/8
#define DEGREE_45_TO_ENCODER  36864.f
#define DEGREE_90_TO_ENCODER 73728.f

/** ����PID **/
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

/*********************************************************************************************************
*                                              ö�ٽṹ�嶨��
*********************************************************************************************************///
/* ����Ļ�ȡֵ�ͼ���ֵ */
typedef struct {
    DJI_Motor_t motor_measure;    // �������ʵ��Ϣ

    fp32 target_speed;            // Ħ����ת���趨ֵ
    pid_type_def speed_p;         // �����ٶȻ�pid
    pid_type_def angle_p;         // ���̽ǶȻ�pid
    int16_t target_current;         // ��������ֵ
}Motor_Launcher_t;

/* �����������Ϣ���� */
typedef struct {
    /* �����Ϣ */
    Motor_Launcher_t fire_l;
    Motor_Launcher_t fire_r;
    Motor_Launcher_t trigger;

    /* Ħ����(Friction Wheel)״̬ */
    Fir_Wheel_Mode_e fir_wheel_last_mode;
    Fir_Wheel_Mode_e fir_wheel_mode;

    /* ����(Trigger)״̬ */
    Shoot_Cmd_e trigger_last_mode;
    Shoot_Cmd_e trigger_mode;

    /** �˲���������ʱû���õ� **/
    first_order_filter_type_t filter_fire;
    first_order_filter_type_t filter_trigger;
}launcher_t;

/*********************************************************************************************************
*                                              ������������ļ�
*********************************************************************************************************/
extern launcher_t launcher;
extern void Launcher_Init(void);
extern void Launcher_Mode_Set(void);
extern void Launcher_Control(void);
extern void Launcher_Disable(void);

#endif //LAUNCHER_H