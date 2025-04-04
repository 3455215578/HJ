#include "right_task.h"
#include "robot_def.h"
#include "wheel.h"
#include "joint.h"
#include "user_lib.h"
#include "ins_task.h"
#include "vmc.h"
#include "lqr.h"

static void right_pid_init(void)
{
    // 右腿PID

    /** 腿长PID **/
    pid_init(&chassis.leg_R.leg_pos_pid,
             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_P,
             CHASSIS_LEG_L0_POS_PID_I,
             CHASSIS_LEG_L0_POS_PID_D);

    pid_init(&chassis.leg_R.leg_speed_pid,
             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_P,
             CHASSIS_LEG_L0_SPEED_PID_I,
             CHASSIS_LEG_L0_SPEED_PID_D);

    // 转向、防劈叉、Roll已在左腿任务中初始化，此处不再重复
}

/** 左腿初始化 **/
static void right_init(void)
{
    /** 初始化底盘模式 **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** 右轮毂电机初始化 **/
    right_wheel_init();

    /** 右关节电机初始化 **/
    right_joint_init();

    /** pid初始化(右腿pid) **/
    right_pid_init();
}

static void right_feedback_update(void)
{
    /** vmc正解算角度更新 **/
    float RF_joint_pos = (get_joint_motors() + 3)->pos_r;
    float RB_joint_pos = (get_joint_motors() + 2)->pos_r;

    chassis.leg_R.vmc.forward_kinematics.fk_phi.phi4 = PI/2 - RF_joint_pos;
    chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1 = PI/2 - RB_joint_pos;

    /** 状态变量更新 **/
    // phi phi_dot
    chassis.leg_R.state_variable_feedback.phi = -INS.Roll * DEGREE_TO_RAD;
    chassis.leg_R.state_variable_feedback.phi_dot = -INS.Gyro[Y];

    // yaw roll phi0_error已在左腿任务中更新，此处不再重复
}

/** 底盘平衡判断 **/
static void chassis_is_balanced() {

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
    if (!chassis.recover_finish)
    {
        chassis.leg_R.joint_F_torque = 0;
        chassis.leg_R.joint_B_torque = 0;
    }
}

/** 向CAN总线发送右电机力矩 **/
static void right_motor_cmd_send() {

/** DEBUG_MODE: 置1时进入调试模式，关闭关节和轮毂输出 **/
#if DEBUG_MODE
    set_right_joint_torque(0.0f,
                           0.0f);

        set_right_wheel_torque(0.0f);
#else
    if(chassis.chassis_ctrl_mode != CHASSIS_DISABLE)
    {
        set_right_joint_torque(chassis.leg_R.joint_F_torque,
                               chassis.leg_R.joint_B_torque);

        set_right_wheel_torque(-chassis.leg_R.wheel_torque);
    }
    else
    {
        set_right_joint_torque(0.0f,
                               0.0f);

        set_right_wheel_torque(0.0f);
    }
#endif

}

/** 右轮毂力矩计算 **/
static void right_wheel_torque_calc(void)
{
    // 转向力矩已在左腿任务计算，此处不再重复

    /** LQR **/
    chassis.leg_R.wheel_torque =   wheel_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
                                 + wheel_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
                                 + wheel_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
                                 + wheel_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                 + wheel_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
                                 + wheel_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_R.wheel_torque -= chassis.wheel_turn_torque;

    chassis.leg_R.wheel_torque *= -1;

    VAL_LIMIT(chassis.leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
}

/** 右关节力矩计算 **/
static void right_joint_torque_calc(void)
{
    // Tp
    // 防劈叉pid已在左腿任务计算，此处不再重复

    chassis.leg_R.vmc.forward_kinematics.F_Tp_set_point.E.Tp_set_point =   joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
                                                                        + joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
                                                                        + joint_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
                                                                        + joint_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                                                        + joint_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
                                                                        + joint_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_R.vmc.forward_kinematics.F_Tp_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;

    // F
    /****** 腿长pid ******/
    float R_L0_dot_set = pid_calc(&chassis.leg_R.leg_pos_pid,
                                  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
                                  chassis.chassis_ctrl_info.height_m);

    pid_calc(&chassis.leg_R.leg_speed_pid,
             chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot,
             R_L0_dot_set);

    // Roll pid已在左腿任务计算，此处不再重复

    chassis.leg_R.vmc.forward_kinematics.F_Tp_set_point.E.F_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_R.state_variable_feedback.theta)
                                                                         + chassis.leg_R.leg_speed_pid.out
                                                                         - chassis.chassis_roll_pid.out;

    /** 映射到真实关节力矩 **/
    vmc_forward_dynamics(&chassis.leg_R.vmc, &chassis_physical_config);

    chassis.leg_R.joint_F_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;
    chassis.leg_R.joint_B_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;

    VAL_LIMIT(chassis.leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
}

/*******************************************************************************
 *                                  Task                                       *
 *******************************************************************************/

/*********************** 初始化任务 ***************************/
static void right_init_task()
{
    right_joint_enable();
    right_wheel_enable();

    chassis.left_init_flag = true;

    if(chassis.left_init_flag && chassis.right_init_flag)
    {
        chassis.init_flag = true;
    }
}

/*********************** 失能任务 ***************************/
static void right_disable_task()
{
    chassis.leg_R.joint_F_torque = 0.0f;
    chassis.leg_R.joint_B_torque = 0.0f;
    chassis.leg_R.wheel_torque = 0.0f;

    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    chassis.jump_state = NOT_READY;

    chassis.leg_R.state_variable_feedback.x = 0.0f;

    /** 初始化标志位 **/

    // 初始化标志位
    chassis.right_init_flag = false;

    // 平衡标志位
    chassis.chassis_is_balance = false;

    // 倒地自救成功标志位
    chassis.recover_finish = false;

    // 离地标志位
    chassis.leg_R.leg_is_offground = false;

    // 跳跃标志位
    chassis.jump_flag = false;

}

/*********************** 关节失能任务 ***************************/
static void right_enable_task()
{
    chassis_is_balanced();

    vmc_forward_kinematics(&chassis.leg_R, &chassis_physical_config);

    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

    right_wheel_torque_calc();
    right_joint_torque_calc();

    Inverse_Dynamics(&chassis.leg_R.vmc,
                     -(get_joint_motors() + 2)->torque,
                     -(get_joint_motors() + 3)->torque,
                     &chassis_physical_config);

    Inverse_Kinematics(&chassis.leg_R.vmc,
                       -(get_joint_motors() + 2)->angular_vel,
                       -(get_joint_motors() + 3)->angular_vel,
                       &chassis_physical_config);

    fn_cal(&chassis.leg_R, chassis.imu_reference.robot_az, &chassis_physical_config);

    chassis_selfhelp();
}

void right_task(void const *pvParameters)
{
    right_init();

    TickType_t last_wake_time = xTaskGetTickCount();

    while(1)
    {
        right_feedback_update();

        switch (chassis.chassis_ctrl_mode) {

            case CHASSIS_DISABLE:
                right_disable_task();
                break;

            case CHASSIS_INIT:
                right_init_task();
                break;

            case CHASSIS_ENABLE:
                right_enable_task();
                break;

            default:break;
        }

        right_motor_cmd_send();

        vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
    }
}