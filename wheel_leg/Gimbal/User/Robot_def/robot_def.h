#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "DJI_Motor.h"
#include "pid.h"
#include "user_lib.h"
#include "filter.h"
#include "protocol_Balance.h"

/* 云台任务初始化时间 */
#define GIMBAL_TASK_INIT_TIME 800

/* 云台任务运行周期 */
#define GIMBAL_PERIOD 1

/**********************************************************************************
 *                                      云台                                       *
 **********************************************************************************/

/** 宏定义 **/

/* Pitch动态限位 */
#define MAX_ABS_ANGLE 30
#define MIN_ABS_ANGLE (-23)

/* 云台回中编码值 */
#define PITCH_OFFSET_ECD 0
#define YAW_OFFSET_ECD 7460


/** PID参数 **/

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


/** 云台控制模式 **/
typedef enum {
    GIMBAL_DISABLE,   // 失能
    GIMBAL_INIT,      // 初始化
    GIMBAL_ENABLE,    // 使能
    GIMBAL_AUTO,    //云台自瞄模式
    GIMBAL_FIRE,    //云台打弹模式
}Gimbal_Mode_e;

/** 云台电机结构体 **/
typedef struct {
    DJI_Motor_t motor_measure;    // 电机的真实信息

    float gyro; // °/s

    Pid speed_pid;   //速度环 PID
    Pid angle_pid;   //角度环 PID

    int16_t target_current; // 期望电流值

    fp32 absolute_angle_get; // ° IMU角度值
    fp32 relative_angle_get; // ° 与电机上电复位位置的角度差值

    fp32 absolute_angle_set; // °

    // 上电复位标志位
    bool reset_finished;

}Motor_Gimbal_t;


/** 云台结构体 **/
typedef struct {

    /** 云台电机 **/
    Motor_Gimbal_t yaw;
    Motor_Gimbal_t pitch;

    /** 云台控制模式 **/
    Gimbal_Mode_e gimbal_ctrl_mode;
    Gimbal_Mode_e gimbal_last_ctrl_mode;

    /** 滤波器 **/

    // 鼠标输入滤波
    first_order_filter_type_t mouse_in_y;
    first_order_filter_type_t mouse_in_x;

    // 对视觉传回的yaw和pitch滤波
    first_order_filter_type_t auto_pitch;
    first_order_filter_type_t auto_yaw[2];

    // 角速度滤波
    first_order_filter_type_t pitch_gyro_filter;
    first_order_filter_type_t yaw_gyro_filter;

    bool init_flag;

}gimbal_t;

extern gimbal_t gimbal;

/**********************************************************************************
 *                                    发射机构                                      *
 **********************************************************************************/

/** 宏定义 **/

// 摩擦轮转速
#define FIRE_SPEED_L  4700
#define FIRE_SPEED_R  FIRE_SPEED_L

// 拨盘转速
#define TRIGGER_SPEED -1000

// 3508编码器转一圈编码值加8192  减速比1:19  编码器转19圈输出轴才转一圈  19×8192
// 2006编码器转一圈编码值加8192  减速比1:36  编码器转36圈输出轴才转一圈  36×8192/8
#define DEGREE_45_TO_ENCODER  36864.f
#define DEGREE_90_TO_ENCODER 73728.f

/** PID参数 **/
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

/** 摩擦轮状态 **/
// 因为在判断摩擦轮模式的时候会顺带判断拨盘模式，所以把摩擦轮开启与否看作发射机构开启或关闭(暂定是这样，后续可能会分开，便于区别)
typedef enum{
    Fire_OFF=0,             //发射机构关闭
    Fire_ON=1,              //发射机构开启
}Fir_Wheel_Mode_e;

/** 拨盘状态 **/
typedef enum{
    SHOOT_CLOSE=0,          //发射关闭
    SHOOT_CONTINUE,         //连发指令

    SHOOT_READY_TO_SINGLE,  //单发指令
    SHOOT_BLOCK,            //单发堵转

    SHOOT_SINGLE,           //单发中
    SHOOT_INVERSING,        //反转中

    SHOOT_OVER,             //发射完成
    SHOOT_FAIL              //拨盘坏了
}Shoot_Cmd_e;


/** 发射机构电机结构体 **/
typedef struct {
    DJI_Motor_t motor_measure;    // 电机的真实信息

    fp32 target_speed;            // 摩擦轮转速设定值
    Pid speed_pid;                // 拨盘速度环pid
    Pid angle_pid;                // 拨盘角度环pid
    int16_t target_current;       // 期望电流值

}Motor_Launcher_t;

/** 发射机构结构体 **/
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
