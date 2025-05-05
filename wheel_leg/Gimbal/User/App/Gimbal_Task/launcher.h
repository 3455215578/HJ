#ifndef LAUNCHER_H
#define LAUNCHER_H

/*********************************************************************************************************
*                                              ����ͷ�ļ�
*********************************************************************************************************/
#include "Balance.h"
#include "Feedforward_PID.h"
#include "PID.h"
#include "DJI_Motor.h"
#include "SMC.h"


/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/
//// Ħ����ת��
#define FIRE_SPEED_L  4700//5000//4600//5000
#define FIRE_SPEED_R  FIRE_SPEED_L
#define FIRE_SPEED_ON 4750//4650//5050

//// 3508������תһȦ����ֵ��8192  ���ٱ�1:19  ������ת19Ȧ������תһȦ  19��8192
//// 2006������תһȦ����ֵ��8192  ���ٱ�1:36  ������ת36Ȧ������תһȦ  36��8192/8
#define DEGREE_45_TO_ENCODER  36864.f
#define DEGREE_90_TO_ENCODER 73728.f

/*********************************************************************************************************
*                                              ö�ٽṹ�嶨��
*********************************************************************************************************///
/* ����Ļ�ȡֵ�ͼ���ֵ */
typedef struct {
    DJI_Motor_t motor_measure;    // �������ʵ��Ϣ

    fp32 speed;                   // Ħ����ת���趨ֵ
    VSP_PID_t feedforward_speed_p;// Ħ���ּ�ǰ���ٶȻ�pid
    pid_type_def speed_p;         // �����ٶȻ�pid
    pid_type_def angle_p;         // ���̽ǶȻ�pid
    int16_t give_current;         // �����ĵ���ֵ
}Motor_Launcher_t;

/* �����������Ϣ���� */
typedef struct {
    /* �����Ϣ */
    Motor_Launcher_t fire_l;
    Motor_Launcher_t fire_r;
    Motor_Launcher_t trigger;

    /* Ħ����״̬ */
    Fire_Mode_e fire_mode;
    Fire_Mode_e fire_last_mode;

    /* ����״̬ */
    Shoot_Cmd_e shoot_last_cmd;
    Shoot_Cmd_e shoot_cmd;

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
extern void Launcher_Relax_Handle(void);

#endif //LAUNCHER_H