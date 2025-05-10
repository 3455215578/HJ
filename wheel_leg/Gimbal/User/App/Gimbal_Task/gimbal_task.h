//
// Created by xhuanc on 2021/10/13.
//

#ifndef HERO_GIMBAL_H
#define HERO_GIMBAL_H

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Balance.h"
#include "PID.h"
#include "filter.h"
#include "DJI_Motor.h"
#include "key_board.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
/* 云台任务初始化时间 */
#define GIMBAL_TASK_INIT_TIME 800

/* 云台任务运行周期 */
#define GIMBAL_PERIOD 1

/* pitch轴 PID */
#define GIMBAL_PITCH_ANGLE_PID_KP           90.f
#define GIMBAL_PITCH_ANGLE_PID_KI           0.f
#define GIMBAL_PITCH_ANGLE_PID_KD           100.f
#define GIMBAL_PITCH_ANGLE_MAX_IOUT         0.f
#define GIMBAL_PITCH_ANGLE_MAX_OUT          9000.f

#define GIMBAL_PITCH_SPEED_PID_KP           40.f
#define GIMBAL_PITCH_SPEED_PID_KI           0.1f
#define GIMBAL_PITCH_SPEED_PID_KD           80.f
#define GIMBAL_PITCH_SPEED_MAX_IOUT         5000.f
#define GIMBAL_PITCH_SPEED_MAX_OUT          10000.f

/* yaw轴PID */
#define GIMBAL_YAW_ANGLE_PID_KP             60.f
#define GIMBAL_YAW_ANGLE_PID_KI             0.f
#define GIMBAL_YAW_ANGLE_PID_KD             250.0f
#define GIMBAL_YAW_ANGLE_MAX_IOUT           80.f
#define GIMBAL_YAW_ANGLE_MAX_OUT            10000.f

#define GIMBAL_YAW_SPEED_PID_KP             30.f//140.f//100.0f
#define GIMBAL_YAW_SPEED_PID_KI             0.f
#define GIMBAL_YAW_SPEED_PID_KD             0.f//0.f//100.0f
#define GIMBAL_YAW_SPEED_MAX_IOUT           3000.f
#define GIMBAL_YAW_SPEED_MAX_OUT            15000.f

/* 对 pitch 进行动态限位 */
#define MAX_ABS_ANGLE 14
#define MIN_ABS_ANGLE (-24)

/* ECD 回中值 */
#define PITCH_OFFSET_ECD 0
#define YAW_OFFSET_ECD 7471

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
/* 电机的获取值和计算值 */
typedef struct {
    DJI_Motor_t motor_measure;    // 电机的真实信息

    pid_type_def speed_p;   //速度环 PID 控制参数
    pid_type_def angle_p;   //角度环 PID 控制参数
    fp32 gyro_set;          //转速设置
    int16_t target_current;   //设置电流值

    fp32 relative_angle_set; //°    设定
    fp32 relative_angle_get; //°    获取
    fp32 absolute_angle_set; //      rad
    fp32 absolute_angle_get; //      云台角
}Motor_Gimbal_t;

typedef struct {
    /* 电机信息 */
    Motor_Gimbal_t yaw;
    Motor_Gimbal_t pitch;

    /* 云台状态信息 */
    Gimbal_Mode_e mode;
    Gimbal_Mode_e last_mode;

    /* 姿态角 */
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
*                                              对外允许调用文件
*********************************************************************************************************/
extern gimbal_t gimbal;
extern void Gimbal_Can_Msg(uint32_t can_id, uint8_t *can_msg);
extern void Chassis_to_Gimbal_Can(uint32_t can_id, const uint8_t *rx_data);

void Gimbal_task(void const*pvParameters);

#endif //HERO_GIMBAL_H
