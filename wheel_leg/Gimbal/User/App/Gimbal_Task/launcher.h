#ifndef LAUNCHER_H
#define LAUNCHER_H

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Balance.h"
#include "Feedforward_PID.h"
#include "PID.h"
#include "DJI_Motor.h"
#include "SMC.h"


/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
//// 摩擦轮转速
#define FIRE_SPEED_L  4700//5000//4600//5000
#define FIRE_SPEED_R  FIRE_SPEED_L
#define FIRE_SPEED_ON 4750//4650//5050

//// 3508编码器转一圈编码值加8192  减速比1:19  编码器转19圈输出轴才转一圈  19×8192
//// 2006编码器转一圈编码值加8192  减速比1:36  编码器转36圈输出轴才转一圈  36×8192/8
#define DEGREE_45_TO_ENCODER  36864.f
#define DEGREE_90_TO_ENCODER 73728.f

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************///
/* 电机的获取值和计算值 */
typedef struct {
    DJI_Motor_t motor_measure;    // 电机的真实信息

    fp32 speed;                   // 摩擦轮转速设定值
    VSP_PID_t feedforward_speed_p;// 摩擦轮加前馈速度环pid
    pid_type_def speed_p;         // 拨盘速度环pid
    pid_type_def angle_p;         // 拨盘角度环pid
    int16_t give_current;         // 给定的电流值
}Motor_Launcher_t;

/* 发射机构的信息集合 */
typedef struct {
    /* 电机信息 */
    Motor_Launcher_t fire_l;
    Motor_Launcher_t fire_r;
    Motor_Launcher_t trigger;

    /* 摩擦轮状态 */
    Fire_Mode_e fire_mode;
    Fire_Mode_e fire_last_mode;

    /* 拨盘状态 */
    Shoot_Cmd_e shoot_last_cmd;
    Shoot_Cmd_e shoot_cmd;

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
extern void Launcher_Relax_Handle(void);

#endif //LAUNCHER_H