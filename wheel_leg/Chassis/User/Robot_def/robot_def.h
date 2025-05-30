#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdbool.h>

#include "pid.h"
#include "moving_filter.h"
#include "can_device.h"
#include "user_lib.h"

typedef float fp32; // 表明某个float类型变量是32位浮点数
typedef double fp64;

/** 宏定义 **/
//CHASSIS_REMOTE置1：底盘由遥控器控制
#define CHASSIS_REMOTE 1

//DEBUG_MODE置1：进入调试模式，关闭关节和轮毂输出
#define DEBUG_MODE 0

#define CHASSIS_PERIOD 1 // ms 计算频率: 1kHz

/** 遥控器路径 **/
// x : 2-左手 ; 0-右手
// y : 3-左手 ; 1-右手
#define CHASSIS_SPEED_CHANNEL 1
#define CHASSIS_YAW_CHANNEL 2
#define CHASSIS_SPIN_CHANNEL 4

#define TEST_CHASSIS_ROLL_CHANNEL 2
#define TEST_CHASSIS_LEG_CHANNEL 3
#define CHASSIS_PIT_CHANNEL 3
#define CHASSIS_ROLL_CHANNEL 0

/** 变量约束 **/
#define MAX_CHASSIS_VX_SPEED 2.1f
#define MAX_PITCH 0.174533f
#define MIN_PITCH (-0.174533f)
#define MAX_ROLL 0.12f
#define MIN_ROLL (-0.12f)
#define MAX_WHEEL_TORQUE 10.f
#define MIN_WHEEL_TORQUE (-10.f)
#define MAX_JOINT_TORQUE 40.f
#define MIN_JOINT_TORQUE (-40.f)

#define MIN_L0 0.10f
#define MAX_L0 0.40f


/** 遥控器值映射 **/
#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define MAX_CHASSIS_YAW_INCREMENT 0.005f
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)
#define RC_TO_PITCH ((MAX_PITCH-MIN_PITCH)/660)
#define RC_TO_ROLL ((MAX_ROLL-MIN_ROLL)/660)


/** PID参数 **/
/** 转向位置环PID **/
#define CHASSIS_VW_POS_PID_P 0.0f
#define CHASSIS_VW_POS_PID_I 0.0f
#define CHASSIS_VW_POS_PID_D 0.0f
#define CHASSIS_VW_POS_PID_IOUT_LIMIT 0.0f
#define CHASSIS_VW_POS_PID_OUT_LIMIT 0.0f

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> parent of f4dde99 (璋冨弬)
/** 转向速度环PID **/
#define CHASSIS_VW_SPEED_PID_P 0.0f
#define CHASSIS_VW_SPEED_PID_I 0.0f
#define CHASSIS_VW_SPEED_PID_D 0.0f
#define CHASSIS_VW_SPEED_PID_IOUT_LIMIT 0.0f
#define CHASSIS_VW_SPEED_PID_OUT_LIMIT 0.0f

>>>>>>> parent of f4dde99 (璋冨弬)
/** 腿长位置环PID **/
#define CHASSIS_LEG_L0_POS_PID_P 15.0f
#define CHASSIS_LEG_L0_POS_PID_I 0.0f
#define CHASSIS_LEG_L0_POS_PID_D 15.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT 2.0f

/** 腿长速度环PID **/
#define CHASSIS_LEG_L0_SPEED_PID_P 27.0f // 50.0f
#define CHASSIS_LEG_L0_SPEED_PID_I 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_D 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT 60.0f

/** Roll PID **/
#define CHASSIS_ROLL_PID_P 500.0f
#define CHASSIS_ROLL_PID_I 0.0f
#define CHASSIS_ROLL_PID_D 0.0f
#define CHASSIS_ROLL_PID_IOUT_LIMIT 0.0f
#define CHASSIS_ROLL_PID_OUT_LIMIT 50.0f

/** 离地后的腿长PID **/
#define CHASSIS_OFFGROUND_LO_PID_P 0.0f
#define CHASSIS_OFFGROUND_L0_PID_I 0.0f
#define CHASSIS_OFFGROUND_L0_PID_D 0.0f
#define CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT 0.0f
#define CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT 0.0f


#define PHI_BALANCE 0.5f * DEGREE_TO_RAD

/*******************************************************************************
 *                                    底盘                                     *
 *******************************************************************************/

/** 底盘物理参数结构体 **/
typedef struct
{
    float wheel_radius; // 驱动轮半径
    float body_weight; // 机体质量(有云台要算上云台)
    float wheel_weight; // 驱动轮重量(算上电机)

    // 五连杆每根腿的长度(m)
    float L1a, L2a;
    float L1u, L2u;
    float L1d, L2d;

} ChassisPhysicalConfig;

/** 底盘模式结构体 **/
typedef enum{
    CHASSIS_DISABLE = 1, // 失能模式
    CHASSIS_INIT, // 初始化模式
    CHASSIS_ENABLE, // 使能模式
    CHASSIS_SPIN, // 小陀螺
    CHASSIS_JUMP, // 跳跃模式
} ChassisCtrlMode;

typedef struct{
    float s_dot; // 期望速度 m/s
    float yaw_rad;
    float roll_rad;
    float height_m; // 期望腿长
    float spin_speed;

} ChassisCtrlInfo;


/** 跳跃状态结构体 **/
typedef enum{
    NOT_READY,
    READY, // 第一阶段：收腿蓄力
    STRETCHING, // 第二阶段：伸腿蹬地
    SHRINKING, // 第三阶段：空中收腿
    LANDING, // 第四阶段：落地
} JumpState;

/** 传感器结构体 **/
typedef struct{
    // 欧拉角
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float yaw_total_rad;


    //三轴角速度
    float pitch_gyro;
    float yaw_gyro;
    float roll_gyro;

    //三轴加速度
    float ax;
    float ay;
    float az;

    // 机体竖直向上的加速度
    float robot_az;

} IMUReference;


/** 状态变量结构体 **/
typedef struct{

    // 状态变量
    // 运动部分
    float s;
    float s_dot;
    float yaw;
    float yaw_dot;

    // 平衡部分
    float theta_l;
    float theta_l_dot;
    float theta_r;
    float theta_r_dot;
    float theta_b;
    float theta_b_dot;


    // 其他
    float theta_l_last;
    float theta_l_dot_last;
    float theta_l_ddot;

    float theta_r_last;
    float theta_r_dot_last;
    float theta_r_ddot;


    float s_dot_last;
    float s_ddot;


} StateVariable;

/** VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC **/

/** 正运动学解算  FK == Forward Kinematics(正运动学) **/
typedef struct{// 腿长
    float L0;
    float L0_last;
    float L0_dot;
    float L0_dot_last;
    float L0_ddot;
} FKL0;

typedef struct{// 五连杆中的角度
    float phi1;
    float phi2;


    float phi0; // 腿摆角
    float last_phi0;
    float d_phi0;// 摆角变化速度
    float last_d_phi0;
    float dd_phi0;
} FKPhi;

typedef struct{// VMC正运动学解算中的中间变量
    float x_1, y_1;
    float x_2, y_2;
    float sigma1;
    float Xe, Ye;
} FKTemp;

typedef struct{
    FKL0 fk_L0;
    FKPhi fk_phi;
    FKTemp fk_temp;

/** 正动力学解算(Forward Dynamics)：从 末端力(F Tp) 到 末端执行器(T1 T4) **/
    union { // 联合体
        float array[2][2];
        struct {
            float Tb_set_point;
            float F_set_point;
        } E;
    } Tb_F_set_point;

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

} ForwardKinematics;


/** 逆动力学解算(Inverse Dynamics): 从 末端执行器(T1 T4) 到 末端力(T Tp) **/
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

 /** 逆运动学解算(Inverse Dynamics): 从 末端执行器(w1 w4) 到 末端编码(d_L0 d_phi0) **/
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

            float last_d_L0_fdb;
            float dd_L0_fdb;
        } E;
    } V_fdb;

}InverseKinematics;

/** 腿部VMC结构体 **/
typedef struct{
    ForwardKinematics forward_kinematics;
    InverseKinematics inverse_kinematics;
} VMC;
/*****************************************************************************/



/** 腿部结构体 **/
typedef struct{

    ChassisCtrlInfo chassis_ctrl_info;

    /** 状态变量 **/
    StateVariable state_variable_feedback;  // 反馈状态变量

    /** 腿部VMC **/
    VMC vmc;

    /** 腿长串级PID **/
    Pid leg_pos_pid; // 腿长位置环
    Pid leg_speed_pid; // 腿长速度环

    /** 离地后的腿长PID **/
    Pid offground_leg_pid; // 离地后的腿长pid  使腿尽量接近地面，增加缓冲

    float wheel_torque; // 轮毂力矩
    float joint_F_torque; // 关节力矩
    float joint_B_torque;

    /** 竖直方向支持力 **/
    MovingAverageFilter theta_ddot_filter; // dd_theta的移动平均滤波器, 用于计算竖直方向支持力Fn
    MovingAverageFilter Fn_filter; // 竖直方向支持力Fn的移动平均滤波器
    float Fn; // 竖直方向支持力

    bool leg_is_offground;

} Leg;

/** 底盘结构体 **/
typedef struct{

    /** 传感器 **/
    IMUReference imu_reference;

    /** 遥控器信息 **/
    ChassisCtrlMode chassis_ctrl_mode;
    ChassisCtrlMode chassis_last_ctrl_mode;
    ChassisCtrlInfo chassis_ctrl_info;

    /** 腿部 **/
    Leg leg_L;
    Leg leg_R;

    /** 跳跃 **/
    JumpState jump_state;

    /** PID **/
    // Wheel
    float target_spin_speed;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> parent of f4dde99 (璋冨弬)
    Pid chassis_vw_pos_pid;
    Pid chassis_vw_speed_pid;
    float wheel_turn_torque;          // 转向力矩

>>>>>>> parent of f4dde99 (璋冨弬)
    // Joint
    Pid chassis_roll_pid;             // roll补偿pid

    /** flag **/

    bool init_flag;            // 底盘初始化完成标志位

    bool chassis_is_balance;   // 平衡标志位
    bool recover_finish;       // 倒地自起完成标志位

    bool chassis_is_offground; // 离地标志位

    bool jump_flag;            // 跳跃标志位

} Chassis;


extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;

#endif
