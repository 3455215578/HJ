#ifndef _CHASSIS_H_
#define _CHASSIS_H_

/*include*/
#include "struct_typedef.h"
#include "FreeRTOS.h"
#include "can_receive.h"

#include "PID.h"
#include "user_lib.h"
#include "ramp.h"

#include "key_board.h"
#include "remote.h"

#include "queue.h"
#include "cmsis_os.h"

#include "bsp_buzzer.h"
#include "math.h"

#include "gimbal_task.h"
#include "referee_task.h"
#include "detect_task.h"
#include "cap_task.h"

#include "protocol_shaob.h"
#include "packet.h"


/** 底盘电机枚举 **/
typedef enum {
    RF = 0,
    LF,
    LB,
    RB
} chassis_motor_index_e;

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 157

/** 遥控器路径映射 **/
#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Y_CHANNEL 0
#define CHASSIS_Z_CHANNEL 4

/** 遥控器数值映射 **/
#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_VY  (MAX_CHASSIS_VY_SPEED/660)
#define RC_TO_VW  (MAX_CHASSIS_VW_SPEED/660)    //MAX_CHASSIS_VR_SPEED / RC_MAX_VALUE

/** 底盘3508速度环 **/
#define CHASSIS_3508_SPEED_PID_KP     10.0f
#define CHASSIS_3508_SPEED_PID_KI     1.0f // 1.0f
#define CHASSIS_3508_SPEED_PID_KD     0.0f
#define CHASSIS_3508_SPEED_PID_MAX_OUT 16384.0f
#define CHASSIS_3508_SPEED_PID_MAX_IOUT 1000.0f

/** 底盘跟随云台PID **/
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 0.30f //0.1    0.28
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f //        0.1
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 8.5f //        7.5 8.25
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 4.3f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

/** 限幅 **/
#define MAX_CHASSIS_VX_SPEED 4.4f //根据3508最高转速计算底盘最快移动速度应为3.3m/s左右  减小电机减速比为1：14后，能达到4.47m/s
#define MAX_CHASSIS_VY_SPEED 4.4f
#define MAX_CHASSIS_VW_SPEED 4.4f

#define MAX_CHASSIS_AUTO_VX_SPEED 8.0f //根据3508最高转速计算底盘最快移动速度应为3300左右
#define MAX_CHASSIS_AUTO_VY_SPEED 3.0f
#define MAX_CHASSIS_AUTO_VW_SPEED 1.5f//150

/** 小陀螺转速 **/
#define CHASSIS_SWING_SPEED -10.5f //-10.5f

#define CHASSIS_ARMOR_NOT_FACING_ENEMY_SPIN_SPEED 0.5 //装甲板没有面向敌人的速度
#define CHASSIS_ARMOR_FACING_ENEMY_SPIN_SPEED  3.0 //装甲板面向敌人的速度

/** 功率预测模型参数，详情见华中科技大学 狼牙战队 功率控制 **/
#define CHASSIS_POWER_K0  0.0020f   //转矩系数    0.0021
#define CHASSIS_POWER_R0  0.0001f  //电机电阻      0.0820
#define CHASSIS_POWER_P0  11.3380f   //底盘静息功率    8.4443加滤波后的拟合数据
#define CHASSIS_CURRENT_CONVERT 20/16384.0f  //电机反馈电流毫安转安

//底盘机械信息 /m
#define Wheel_axlespacing 0.448f //H
#define Wheel_spacing 0.391f //W
#define GIMBAL_OFFSET 0
#define PERIMETER 0.47414f //轮子周长 /m
#define M3508_DECELE_RATIO (1.0f/14.0f)  //1：19 3508减速比
#define M3508_MAX_RPM 8000              //3508最大转速
#define TREAD 480 //lun ju
#define WHEEL_MOTO_RATE 0.00041591f

/** 底盘模式结构体 **/
typedef enum {
    CHASSIS_RELAX,
    CHASSIS_ONLY,
    CHASSIS_SPIN,
    CHASSIS_FOLLOW_GIMBAL,
    CHASSIS_BLOCK
} chassis_mode_e;

/** 底盘旋转模式结构体 **/
typedef enum {
    NORMAL_SPIN,
    HIDDEN_ARMOR_SPEED_CHANGE
} chassis_spin_mode_e;

/** 功率控制结构体 **/
typedef struct {
    fp32 power_buff;
    fp32 limit_k;
    fp32 total_current;
    fp32 total_current_limit;

    fp32 K[4];
    fp32 M[4];
    fp32 k_c;    //rpm_set缩减系数
    fp32 predict_send_power; //预测的当前周期的功率
    fp32 power_set;//最大功率限制

} chassis_power_limit_t;

/** 底盘结构体 **/
typedef struct {

    /** 底盘模式 **/
    chassis_mode_e mode;
    chassis_mode_e last_mode;

    /** 旋转模式 **/
    chassis_spin_mode_e spin_mode;

    /** 底盘电机结构体 **/
    motor_3508_t motor_chassis[4];

    QueueHandle_t motor_data_queue;

    /** 底盘跟随云台pid **/
    pid_t chassis_follow_gimbal_pid;


    fp32 vx;
    fp32 vy;
    fp32 vw;

    fp32 vx_pc;
    fp32 vy_pc;
    fp32 vw_pc;

    chassis_power_limit_t chassis_power_limit;
} chassis_t;

//函数声明
_Noreturn extern void chassis_task(void const *pvParameters);

//
#endif

