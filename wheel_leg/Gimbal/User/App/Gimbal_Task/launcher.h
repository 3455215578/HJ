#ifndef LAUNCHER_H
#define LAUNCHER_H

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Balance.h"
#include "DJI_Motor.h"
#include "user_lib.h"
#include "PID.h"
#include "SMC.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
/** 摩擦轮转速 **/
#define FIRE_SPEED_L  4700
#define FIRE_SPEED_R  FIRE_SPEED_L
/** 拨盘转速 **/
#define TRIGGER_SPEED -1000

//// 3508编码器转一圈编码值加8192  减速比1:19  编码器转19圈输出轴才转一圈  19×8192
//// 2006编码器转一圈编码值加8192  减速比1:36  编码器转36圈输出轴才转一圈  36×8192/8
#define DEGREE_45_TO_ENCODER  36864.f
#define DEGREE_90_TO_ENCODER 73728.f

/** 拨盘PID **/
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
*                                              枚举结构体定义
*********************************************************************************************************///
/* 电机的获取值和计算值 */
typedef struct {
    DJI_Motor_t motor_measure;    // 电机的真实信息

    fp32 target_speed;            // 摩擦轮转速设定值
    pid_type_def speed_p;         // 拨盘速度环pid
    pid_type_def angle_p;         // 拨盘角度环pid
    int16_t target_current;         // 期望电流值
}Motor_Launcher_t;

/* 发射机构的信息集合 */
typedef struct {
    /* 电机信息 */
    Motor_Launcher_t fire_l;
    Motor_Launcher_t fire_r;
    Motor_Launcher_t trigger;

    /* 摩擦轮(Friction Wheel)状态 */
    Fir_Wheel_Mode_e fir_wheel_last_mode;
    Fir_Wheel_Mode_e fir_wheel_mode;

    /* 拨盘(Trigger)状态 */
    Shoot_Cmd_e trigger_last_mode;
    Shoot_Cmd_e trigger_mode;

    /** 滤波器但是暂时没有用到 **/
    first_order_filter_type_t filter_fire;
    first_order_filter_type_t filter_trigger;
}launcher_t;

/*********************************************************************************************************
*                                              对外允许调用文件
*********************************************************************************************************/
extern launcher_t launcher;
extern void Launcher_Init(void);
extern void Launcher_Mode_Set(void);
extern void Launcher_Control(void);
extern void Launcher_Disable(void);

#endif //LAUNCHER_H