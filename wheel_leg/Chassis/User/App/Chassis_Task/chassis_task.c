#include <math.h>
#include <stdio.h>
#include "chassis_task.h"
#include "robot_def.h"
#include "user_lib.h"
#include "joint.h"
#include "wheel.h"
#include "remote.h"
#include "vmc.h"
#include "error.h"
#include "ins_task.h"
#include "vx_kalman_filter.h"
#include "lqr.h"
#include "bsp_delay.h"
#include "bsp_dwt.h"

/** 底盘pid初始化 **/
static void chassis_pid_init() {

    /** Wheel **/

    // 转向PID
    pid_init(&chassis.chassis_turn_pid,
             CHASSIS_TURN_PID_OUT_LIMIT,
             CHASSIS_TURN_PID_IOUT_LIMIT,
             CHASSIS_TURN_PID_P,
             CHASSIS_TURN_PID_I,
             CHASSIS_TURN_PID_D);

    // 小陀螺PID
    pid_init(&chassis.chassis_spin_pid,
             CHASSIS_SPIN_PID_OUT_LIMIT,
             CHASSIS_SPIN_PID_IOUT_LIMIT,
             CHASSIS_SPIN_PID_P,
             CHASSIS_SPIN_PID_I,
             CHASSIS_SPIN_PID_D);

    /** Joint **/

    // 防劈叉PID
    pid_init(&chassis.chassis_leg_coordination_pid,
             CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_P,
             CHASSIS_LEG_COORDINATION_PID_I,
             CHASSIS_LEG_COORDINATION_PID_D);

    // 腿长位置环PID
    pid_init(&chassis.leg_L.leg_pos_pid,
             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_P,
             CHASSIS_LEG_L0_POS_PID_I,
             CHASSIS_LEG_L0_POS_PID_D);

    pid_init(&chassis.leg_R.leg_pos_pid,
             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_P,
             CHASSIS_LEG_L0_POS_PID_I,
             CHASSIS_LEG_L0_POS_PID_D);

    // 腿长速度环PID
    pid_init(&chassis.leg_L.leg_speed_pid,
             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_P,
             CHASSIS_LEG_L0_SPEED_PID_I,
             CHASSIS_LEG_L0_SPEED_PID_D);

    pid_init(&chassis.leg_R.leg_speed_pid,
             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_P,
             CHASSIS_LEG_L0_SPEED_PID_I,
             CHASSIS_LEG_L0_SPEED_PID_D);

    // 离地后的腿长PID 暂时没用到
    pid_init(&chassis.leg_L.offground_leg_pid,
             CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT,
             CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT,
             CHASSIS_OFFGROUND_LO_PID_P,
             CHASSIS_OFFGROUND_L0_PID_I,
             CHASSIS_OFFGROUND_L0_PID_D);

    pid_init(&chassis.leg_R.offground_leg_pid,
             CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT,
             CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT,
             CHASSIS_OFFGROUND_LO_PID_P,
             CHASSIS_OFFGROUND_L0_PID_I,
             CHASSIS_OFFGROUND_L0_PID_D);

    // Roll补偿PID
    pid_init(&chassis.chassis_roll_pid,
             CHASSIS_ROLL_PID_OUT_LIMIT,
             CHASSIS_ROLL_PID_IOUT_LIMIT,
             CHASSIS_ROLL_PID_P,
             CHASSIS_ROLL_PID_I,
             CHASSIS_ROLL_PID_D);
}

/** 底盘初始化 **/
void chassis_init(void)
{
    /** 初始化底盘模式 **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** 关节电机初始化 **/
    joint_init();

    /** 轮毂电机初始化 **/
    wheel_init();

    /** 底盘pid初始化 **/
    chassis_pid_init();

    /** 关节移动平均滤波器初始化 **/
    moving_average_filter_init(&chassis.leg_L.Fn_filter);
    moving_average_filter_init(&chassis.leg_R.Fn_filter);
    moving_average_filter_init(&chassis.leg_L.theta_ddot_filter);
    moving_average_filter_init(&chassis.leg_R.theta_ddot_filter);

    xvEstimateKF_Init(&vaEstimateKF);
}

/** 底盘平衡判断 **/
static void chassis_is_balanced(void) {

    if(!chassis.chassis_is_offground) // 跳跃时不进行平衡判断
    {
        if (ABS(chassis.imu_reference.pitch_rad) <= 0.1744f) // -10° ~ 10°
        {
            chassis.chassis_is_balance = true;

            if((ABS(chassis.imu_reference.pitch_rad) <= 0.05233f)) // -3° ~ 3°
            {
                chassis.recover_finish = true;
            }

        }
        else
        {
            chassis.chassis_is_balance = false;
            chassis.recover_finish = false;
        }
    }
}

/** 底盘倒地自救 **/
static void chassis_selfhelp(void)
{
    chassis_is_balanced();

    if (!chassis.recover_finish)
    {
        chassis.leg_L.joint_F_torque = 0;
        chassis.leg_L.joint_B_torque = 0;
        chassis.leg_R.joint_F_torque = 0;
        chassis.leg_R.joint_B_torque = 0;

        if(!chassis.joint_is_reset)
        {
            chassis.leg_L.wheel_torque = 0;
            chassis.leg_R.wheel_torque = 0;

            joint_reset();
        }
    }
}

/** 获取底盘传感器数据 **/
static void get_IMU_info(void) {

    /** Yaw **/
    chassis.imu_reference.yaw_rad = -INS.Yaw * DEGREE_TO_RAD;
    chassis.imu_reference.yaw_total_rad = -INS.YawTotalAngle * DEGREE_TO_RAD;

    /** Pitch **/
    chassis.imu_reference.pitch_rad = -INS.Roll * DEGREE_TO_RAD;

    /** Roll **/
    chassis.imu_reference.roll_rad = INS.Pitch * DEGREE_TO_RAD;

    /** 更新各轴加速度和角速度 **/
    chassis.imu_reference.pitch_gyro = -INS.Gyro[Y];
    chassis.imu_reference.yaw_gyro = -INS.Gyro[Z];
    chassis.imu_reference.roll_gyro = INS.Gyro[X];

    chassis.imu_reference.ax = INS.Accel[X];
    chassis.imu_reference.ay = INS.Accel[Y];
    chassis.imu_reference.az = INS.Accel[Z];

    /** 机体竖直方向加速度 **/
    chassis.imu_reference.robot_az = INS.MotionAccel_n[Z];
}

/** 更新底盘变量 **/
static void chassis_variable_update(void) {

    get_IMU_info();

    // 5.phi
    chassis.leg_L.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;
    chassis.leg_R.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;

    // 6.phi_dot
    chassis.leg_L.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;
    chassis.leg_R.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;

    //theta_last
    chassis.leg_L.state_variable_feedback.theta_last = chassis.leg_L.state_variable_feedback.theta;
    chassis.leg_R.state_variable_feedback.theta_last = chassis.leg_R.state_variable_feedback.theta;

    //1.theta
    chassis.leg_L.state_variable_feedback.theta = cal_leg_theta(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0, chassis.leg_L.state_variable_feedback.phi);
    chassis.leg_R.state_variable_feedback.theta = cal_leg_theta(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0, chassis.leg_R.state_variable_feedback.phi);

    //2.theta_dot
    chassis.leg_L.state_variable_feedback.theta_dot_last = chassis.leg_L.state_variable_feedback.theta_dot;
    chassis.leg_L.state_variable_feedback.theta_dot = (chassis.leg_L.state_variable_feedback.theta - chassis.leg_L.state_variable_feedback.theta_last) / (CHASSIS_PERIOD * 0.001f);
    float theta_ddot_raw_L = (chassis.leg_L.state_variable_feedback.theta_dot - chassis.leg_L.state_variable_feedback.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);
    chassis.leg_L.state_variable_feedback.theta_ddot = update_moving_average_filter(&chassis.leg_L.theta_ddot_filter, theta_ddot_raw_L);

    chassis.leg_R.state_variable_feedback.theta_dot_last = chassis.leg_R.state_variable_feedback.theta_dot;
    chassis.leg_R.state_variable_feedback.theta_dot = (chassis.leg_R.state_variable_feedback.theta - chassis.leg_R.state_variable_feedback.theta_last) / (CHASSIS_PERIOD * 0.001f);
    float theta_ddot_raw_R = (chassis.leg_R.state_variable_feedback.theta_dot - chassis.leg_R.state_variable_feedback.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);
    chassis.leg_R.state_variable_feedback.theta_ddot = update_moving_average_filter(&chassis.leg_R.theta_ddot_filter, theta_ddot_raw_R);

    //4.x_dot
    chassis.leg_L.state_variable_feedback.x_dot = vel_acc[0];
    chassis.leg_R.state_variable_feedback.x_dot = vel_acc[0];

    //3.x

    if(chassis.chassis_ctrl_info.v_m_per_s != 0.0f)
    {
        chassis.leg_L.state_variable_feedback.x = 0.0f;
        chassis.leg_R.state_variable_feedback.x = 0.0f;
    }
    else
    {
        chassis.leg_L.state_variable_feedback.x = chassis.leg_L.state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * chassis.leg_L.state_variable_feedback.x_dot;
        chassis.leg_R.state_variable_feedback.x = chassis.leg_R.state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * chassis.leg_R.state_variable_feedback.x_dot;

    }

    // x_ddot
    chassis.leg_L.state_variable_feedback.x_dot_last = chassis.leg_L.state_variable_feedback.x_dot;
    chassis.leg_L.state_variable_feedback.x_ddot = (chassis.leg_L.state_variable_feedback.x_dot - chassis.leg_L.state_variable_feedback.x_dot_last) / (CHASSIS_PERIOD * 0.001f);
    chassis.leg_R.state_variable_feedback.x_dot_last = chassis.leg_R.state_variable_feedback.x_dot;
    chassis.leg_R.state_variable_feedback.x_ddot = (chassis.leg_R.state_variable_feedback.x_dot - chassis.leg_R.state_variable_feedback.x_dot_last) / (CHASSIS_PERIOD * 0.001f);
}

/** 计算驱动轮力矩 **/
static void wheel_calc(void)
{
    /******************************* Wheel *************************************/

    /** 根据腿长和三次拟合系数拟合出反馈增益K **/
    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);

    if (chassis.chassis_ctrl_mode != CHASSIS_SPIN)
    {
        // 计算转向力矩
        chassis.wheel_turn_torque =  CHASSIS_TURN_PID_P * (chassis.imu_reference.yaw_total_rad - chassis.chassis_ctrl_info.yaw_rad)
                                   + CHASSIS_TURN_PID_D * chassis.imu_reference.yaw_gyro;

    }
    else
    {

    }



    chassis.leg_L.wheel_torque =  wheel_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
                                  + wheel_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
                                  + wheel_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
                                  + wheel_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                  + wheel_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - PHI_BALANCE)
                                  + wheel_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_R.wheel_torque =  wheel_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
                                  + wheel_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
                                  + wheel_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
                                  + wheel_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                  + wheel_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - PHI_BALANCE)
                                  + wheel_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_L.wheel_torque += chassis.wheel_turn_torque;
    chassis.leg_R.wheel_torque -= chassis.wheel_turn_torque;
    chassis.leg_R.wheel_torque *= -1;

    VAL_LIMIT(chassis.leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    VAL_LIMIT(chassis.leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);

}

/** 计算关节力矩 **/
static void joint_calc(void)
{
/******************************* Joint *************************************/

    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

    /** Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp **/

    /****** 防劈叉pid ******/
    chassis.steer_compensatory_torque =  CHASSIS_LEG_COORDINATION_PID_P * (0.0f - chassis.phi0_error)
                                         + CHASSIS_LEG_COORDINATION_PID_D * (0.0f - chassis.d_phi0_error); // 注意微分项正负

    //Left
    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
                                                                         + joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
                                                                         + joint_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
                                                                         + joint_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                                                         + joint_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - PHI_BALANCE)
                                                                         + joint_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);


    //Right
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
                                                                         + joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
                                                                         + joint_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
                                                                         + joint_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                                                         + joint_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - PHI_BALANCE)
                                                                         + joint_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;

    /** End End End End End End End End End End End End End End End End End End End End End End End End **/


    /** F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F **/

    /****** Leg pid ******/

    float L_L0_dot_set = pid_calc(&chassis.leg_L.leg_pos_pid,
                                  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,
                                  chassis.chassis_ctrl_info.height_m);

    float R_L0_dot_set = pid_calc(&chassis.leg_R.leg_pos_pid,
                                  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
                                  chassis.chassis_ctrl_info.height_m);

    pid_calc(&chassis.leg_L.leg_speed_pid,
             chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
             L_L0_dot_set);

    pid_calc(&chassis.leg_R.leg_speed_pid,
             chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot,
             R_L0_dot_set);

    /****** Roll pid ******/
    pid_calc(&chassis.chassis_roll_pid,
             chassis.imu_reference.roll_rad,
             chassis.chassis_ctrl_info.roll_rad);


    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_L.state_variable_feedback.theta)
                                                                         + chassis.leg_L.leg_speed_pid.out;

    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_R.state_variable_feedback.theta)
                                                                         + chassis.leg_R.leg_speed_pid.out;

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += chassis.chassis_roll_pid.out;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point -= chassis.chassis_roll_pid.out;

    /** End End End End End End End End End End End End End End End End End End End End End End End End **/

    // 计算关节电机力矩
    vmc_forward_dynamics(&chassis.leg_L.vmc, &chassis_physical_config);
    vmc_forward_dynamics(&chassis.leg_R.vmc, &chassis_physical_config);

    chassis.leg_L.joint_F_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_L.joint_B_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    chassis.leg_R.joint_F_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_R.joint_B_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    // 输出限幅
    VAL_LIMIT(chassis.leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);

}

/** 设置腿长 **/
static void leg_length_set(void)
{
    if(switch_is_down(get_rc_ctrl()->rc.s[RC_s_L]))
    {
        chassis.chassis_ctrl_info.height_m = 0.12f;
    }
    else if(switch_is_mid(get_rc_ctrl()->rc.s[RC_s_L]))
    {
        chassis.chassis_ctrl_info.height_m = 0.22f;
    }
    else if(switch_is_up(get_rc_ctrl()->rc.s[RC_s_L]))
    {
        chassis.chassis_ctrl_info.height_m = 0.35f;
    }
}

/** 控制器计算 **/
static void controller_calc(void)
{
    /** 更新五连杆参数 **/
    vmc_calc();
    /** 速度融合 **/
    speed_calc();
    /** 更新底盘变量 **/
    chassis_variable_update();
    /** 计算驱动轮力矩 **/
    wheel_calc();
    /** 计算关节力矩 **/
    joint_calc();
}


/*******************************************************************************
 *                                  Task                                       *
 *******************************************************************************/

/** 底盘失能任务 **/
static void chassis_disable_task() {

    pos = 0.0f;
    speed = 0.0f;
    Kp = 0.0f;
    Kd = 0.0f;

    chassis.leg_L.wheel_torque = 0;
    chassis.leg_R.wheel_torque = 0;

    chassis.leg_L.joint_F_torque = 0;
    chassis.leg_L.joint_B_torque = 0;
    chassis.leg_R.joint_F_torque = 0;
    chassis.leg_R.joint_B_torque = 0;

    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    chassis.leg_L.state_variable_feedback.x = 0.0f;
    chassis.leg_R.state_variable_feedback.x = 0.0f;

    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

    chassis.chassis_ctrl_info.height_m = MIN_L0;

    /** 初始化标志位 **/

    // 底盘初始化标志位
    chassis.init_flag = false;

    // 关节复位标志位
    chassis.joint_is_reset = false;

    // 平衡标志位
    chassis.chassis_is_balance = false;

    // 倒地自救成功标志位
    chassis.recover_finish = false;

    // 离地标志位
    chassis.leg_L.leg_is_offground = false;
    chassis.leg_R.leg_is_offground = false;
    chassis.chassis_is_offground   = false;

    // 跳跃标志位
    chassis.jump_state = NOT_READY;
    chassis.jump_flag = false;

}

/** 底盘初始化任务 **/
static void chassis_init_task()
{
    joint_enable();

    chassis.init_flag = true;
}

/** 底盘使能任务 **/
static void chassis_enable_task(void)
{
    /** 设置期望腿长 **/
    leg_length_set();

    /** 控制器计算 **/
    controller_calc();

    /** 倒地自救(简陋) **/
    chassis_selfhelp();
}

/** 发送力矩任务 **/
static void send_torque_task(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque,
                             float wheel_L_torque, float wheel_R_torque,
                             float pos, float speed, float Kp, float Kd)
{
    set_dm8009_MIT(&joint[LF],pos, speed, Kp, Kd,joint_LF_torque);
    set_dm8009_MIT(&joint[LB],pos, speed, Kp, Kd,joint_LB_torque);
    DWT_Delay(0.0002);
    set_dm8009_MIT(&joint[RF],pos, speed, Kp, Kd,joint_RF_torque);
    set_dm8009_MIT(&joint[RB],pos, speed, Kp, Kd,joint_RB_torque);

    lk9025_torque_set(&wheel[L], wheel_L_torque);
    lk9025_torque_set(&wheel[R], wheel_R_torque);

//    set_dm8009_MIT(&joint[LF],0.0f,0.0f, 0.0f, 0.0f,0.0f);
//    set_dm8009_MIT(&joint[LB],0.0f,0.0f, 0.0f, 0.0f,0.0f);
//    DWT_Delay(0.0002);
//    set_dm8009_MIT(&joint[RF],0.0f,0.0f, 0.0f, 0.0f,0.0f);
//    set_dm8009_MIT(&joint[RB],0.0f,0.0f, 0.0f, 0.0f,0.0f);
//
//    lk9025_torque_set(&wheel[L], 0.0f);
//    lk9025_torque_set(&wheel[R], 0.0f);

}

void chassis_task(void)
{
    /** 获取遥控器信息(模式 + 数据) **/
    remote_cmd();

    switch (chassis.chassis_ctrl_mode)
    {
        case CHASSIS_DISABLE:
        {
            chassis_disable_task();
            break;
        }


        case CHASSIS_INIT:
        {
            chassis_init_task();
            break;
        }


        case CHASSIS_ENABLE:
        case CHASSIS_SPIN:
        {
            chassis_enable_task();
            break;
        }


        default:
        {
            break;
        }
    }

    send_torque_task(-chassis.leg_L.joint_F_torque,
                     -chassis.leg_L.joint_B_torque,
                     chassis.leg_R.joint_F_torque,
                     chassis.leg_R.joint_B_torque,
                     -chassis.leg_L.wheel_torque,
                     -chassis.leg_R.wheel_torque,
                     pos,
                     speed,
                     Kp,
                     Kd);

}