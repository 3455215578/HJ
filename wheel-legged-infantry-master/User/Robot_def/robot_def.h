#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdbool.h>

#include "pid.h"
#include "vmc.h"
#include "vx_kalman_filter.h"

/**  宏定义 宏定义 宏定义 宏定义 宏定义 宏定义 宏定义  **/
//CHASSIS_REMOTE置1：底盘由遥控器控制
#define CHASSIS_REMOTE 1

//DEBUG_MODE置1：进入调试模式，关闭关节和轮毂输出
#define DEBUG_MODE 0

#define CHASSIS_PERIOD 2 //5 // ms 控制频率: 200Hz

/****************** PID参数 *********************/

/** Wheel Wheel Wheel Wheel Wheel **/
// 转向PID
#define CHASSIS_TURN_PID_P 0.0f
#define CHASSIS_TURN_PID_I 0.0f
#define CHASSIS_TURN_PID_D 0.0f
#define CHASSIS_TURN_PID_IOUT_LIMIT 0.0f
#define CHASSIS_TURN_PID_OUT_LIMIT 0.0f
/************************************/


/** Joint Joint Joint Joint Joint **/
// 腿长位置环PID
#define CHASSIS_LEG_L0_POS_PID_P 0.0f
#define CHASSIS_LEG_L0_POS_PID_I 0.0f
#define CHASSIS_LEG_L0_POS_PID_D 0.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT 0.0f

// 防劈叉PID
#define CHASSIS_LEG_COORDINATION_PID_P 0.0f
#define CHASSIS_LEG_COORDINATION_PID_I 0.0f
#define CHASSIS_LEG_COORDINATION_PID_D 0.0f
#define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 0.0f

// Roll PID
#define CHASSIS_ROLL_PID_P 0.0f
#define CHASSIS_ROLL_PID_I 0.0f
#define CHASSIS_ROLL_PID_D 0.0f
#define CHASSIS_ROLL_PID_IOUT_LIMIT 0.0f
#define CHASSIS_ROLL_PID_OUT_LIMIT 0.0f

// 离地后的腿长PID
#define CHASSIS_OFFGROUND_LO_PID_P 0.0f
#define CHASSIS_OFFGROUND_L0_PID_I 0.0f
#define CHASSIS_OFFGROUND_L0_PID_D 0.0f
#define CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT 0.0f
#define CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT 0.0f
/*************************************/


/**********************************************/



/****************** 变量约束 *********************/
#define MAX_CHASSIS_VX_SPEED 1.8f
#define MAX_PITCH 0.174533f
#define MIN_PITCH (-0.174533f)
#define MAX_ROLL 0.12f
#define MIN_ROLL (-0.12f)
#define MAX_WHEEL_TORQUE 10.f
#define MIN_WHEEL_TORQUE (-10.f)
#define MAX_JOINT_TORQUE 40.f
#define MIN_JOINT_TORQUE (-40.f)

#define MIN_L0 0.13f
#define MAX_L0 0.40f
#define MID_L0 0.24f
/**********************************************/



/************** 遥控器路径、映射 *****************/
#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Z_CHANNEL 2
#define CHASSIS_PIT_CHANNEL 3
#define CHASSIS_ROLL_CHANNEL 0
#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define MAX_CHASSIS_YAW_INCREMENT 0.02f
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)
#define RC_TO_PITCH ((MAX_PITCH-MIN_PITCH)/660)
#define RC_TO_ROLL ((MAX_ROLL-MIN_ROLL)/660)
/**********************************************/

typedef struct{
    float wheel_radius;
    float body_weight;
    float wheel_weight;
    float mechanical_leg_limit_angle;

    float l1, l2, l3, l4, l5;
} ChassisPhysicalConfig;

typedef enum{
    CHASSIS_DISABLE = 1,
    CHASSIS_ENABLE,
    CHASSIS_INIT,
    CHASSIS_JUMP,
    CHASSIS_SPIN,
} ChassisCtrlMode;

typedef enum{
    R = 1,
    L = 0,
} LegIndex;

typedef enum{
    NOT_READY,
    READY,
    STRETCHING,
    SHRINKING,
    STRETCHING_AGAIN,
    LANDING,
} JumpState;

typedef struct{
    // 欧拉角
    float pitch_angle;
    float yaw_angle;
    float yaw_last_angle;
    float yaw_total_angle;
    float yaw_round_count;
    float roll_angle;

    //三轴角速度
    float pitch_gyro;
    float yaw_gyro;
    float roll_gyro;

    //三轴加速度
    float ax;
    float ay;
    float az;

    //去除重力加速度影响后的三轴加速度
    float ax_filtered;
    float ay_filtered;
    float az_filtered;

    // 机体竖直向上的加速度
    float robot_az;

} IMUReference;

typedef struct{
    float v_m_per_s;
    float x; // 位移
    float pitch_angle_rad;
    float yaw_angle_rad;
    float roll_angle_rad;
    float height_m;
    float spin_speed;

} ChassisCtrlInfo;

typedef struct{
    float theta;
    float theta_last;
    float theta_dot;
    float theta_dot_last;
    float theta_ddot;

    float x;
    float x_dot;
    float x_dot_last;
    float x_ddot;

    float phi;
    float phi_dot;
} StateVariable;

/** VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC **/

/** 正运动学解算 正运动学解算 正运动学解算 正运动学解算 **/
typedef struct{
    float L0;
    float L0_last;
    float L0_dot;
    float L0_dot_last;
    float L0_ddot;
} FKL0;

typedef struct{// 五连杆中的角度
    float phi1;
    float phi2;
    float phi3;
    float phi4;

    float phi0;
    float last_phi0;
    float d_phi0;// 摆角变化速度
    float last_d_phi0;
    float dd_phi0;
} FKPhi;

typedef struct{// 五连杆中的坐标
    float a_x, a_y;
    float b_x, b_y;
    float c_x, c_y;
    float d_x, d_y;
    float e_x, e_y;
} FKPointCoordinates;

typedef struct{
    FKL0 fk_L0;
    FKPhi fk_phi;
    FKPointCoordinates fk_point_coordinates;
    float d_alpha; //

/*******************************************/

/** 正动力学解算 正动力学解算 正动力学解算 正动力学解算 **/
    union { // 自行学习联合体的特性: union
        float array[2][2];
        struct {
            float Tp_set_point;
            float Fy_set_point;
        } E;
    } Fxy_set_point;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_F_to_T;

    union {
        float array[2][1];
        struct {
            float T1_set_point;
            float T4_set_point;
        } E;
    } T1_T4_set_point;
/************************************************/

} ForwardKinematics;


/** 逆动力学解算 逆动力学解算 逆动力学解算 逆动力学解算 **/
typedef struct {
    union {
        float array[2][1];
        struct {
            float T1_fdb;
            float T4_fdb;
        } E;
    } T1_T4_fdb;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_T_to_F;

    union {
        float array[2][1];
        struct {
            float Tp_fdb;
            float Fy_fdb;
        } E;
    } Fxy_fdb;

    union {
        float array[2][1];
        struct {
            float w1_fdb;// 关节电机反馈回来的角速度
            float w4_fdb;
        } E;
    } W_fdb;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_w_to_v;

    union {
        float array[2][1];
        struct {
            float d_L0_fdb; // 腿长变化速度
            float d_phi0_fdb; // 摆角(phi0)变化速度
        } E;
    } V_fdb;

}InverseKinematics;

/************************************************/

typedef struct{
    ForwardKinematics forward_kinematics;
    InverseKinematics inverse_kinematics;
} VMC;

/** VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC **/



/** Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg **/
typedef struct{
    LegIndex leg_index;
    StateVariable state_variable_feedback;
    StateVariable state_variable_set_point;
    StateVariable state_variable_error;
    StateVariable state_variable_wheel_out;
    StateVariable state_variable_joint_out;

    VMC vmc;

    Pid leg_pos_pid; // 腿长位置环pid
//    Pid leg_speed_pid; // 腿长速度环pid
    Pid offground_leg_pid;

    float* kalman_result; // 轮毂速度与加速度融合后的结果
    float L0_set_point; // 期望腿长

    float wheel_torque;
    float joint_F_torque;
    float joint_B_torque;

    float Fn; // 竖直方向支持力

} Leg;

/** Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg **/

/** Chassis Chassis Chassis Chassis Chassis Chassis Chassis Chassis **/

typedef struct{
    /** Remote Remote Remote Remote **/
    ChassisCtrlMode chassis_ctrl_mode;
    ChassisCtrlMode chassis_ctrl_mode_last;
    ChassisCtrlInfo chassis_ctrl_info;

    JumpState jump_state;

    IMUReference imu_reference;

    Leg leg_L;
    Leg leg_R;

    KalmanFilter vx_kalman;
    float kalman_measure[2];

    /** PID PID PID PID PID PID PID PID PID PID **/
//    Pid chassis_vw_speed_pid;
//    Pid chassis_spin_pid;
    Pid chassis_turn_pid; // 转向pid
    float wheel_turn_torque; // 转向力矩
    float theta_error; // 两条腿之间theta的误差
    Pid chassis_leg_coordination_pid; // 防劈叉pid
    float steer_compensatory_torque; // 防劈叉力矩
    Pid chassis_roll_pid; // roll补偿pid

    /** flag flag flag flag flag flag flag flag **/
    bool is_joint_enable; // 关节电机使能标志位
    bool is_wheel_enable; // 轮毂电机使能标志位
    bool init_flag; // 底盘初始化完成标志位

    bool is_chassis_balance; // 平衡标志位
    bool recover_finish; // 倒地自起完成标志位
    bool is_chassis_offground; // 离地标志位
    bool jump_flag; // 跳跃标志位


} Chassis;

/** Chassis Chassis Chassis Chassis Chassis Chassis Chassis Chassis **/



#endif
