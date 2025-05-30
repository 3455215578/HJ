#include <math.h>
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
#include "delay.h"
#include "bsp_dwt.h"


/** ����pid��ʼ�� **/
static void chassis_pid_init() {
<<<<<<< HEAD
=======

    /** ת��λ�û�PID **/
    pid_init(&chassis.chassis_vw_pos_pid,
             CHASSIS_VW_POS_PID_OUT_LIMIT,
             CHASSIS_VW_POS_PID_IOUT_LIMIT,
             CHASSIS_VW_POS_PID_P,
             CHASSIS_VW_POS_PID_I,
             CHASSIS_VW_POS_PID_D);
<<<<<<< HEAD
=======

    /** ת���ٶȻ�PID **/
    pid_init(&chassis.chassis_vw_speed_pid,
             CHASSIS_VW_SPEED_PID_OUT_LIMIT,
             CHASSIS_VW_SPEED_PID_IOUT_LIMIT,
             CHASSIS_VW_SPEED_PID_P,
             CHASSIS_VW_SPEED_PID_I,
             CHASSIS_VW_SPEED_PID_D);
>>>>>>> parent of f4dde99 (调参)

    /** ת���ٶȻ�PID **/
    pid_init(&chassis.chassis_vw_speed_pid,
             CHASSIS_VW_SPEED_PID_OUT_LIMIT,
             CHASSIS_VW_SPEED_PID_IOUT_LIMIT,
             CHASSIS_VW_SPEED_PID_P,
             CHASSIS_VW_SPEED_PID_I,
             CHASSIS_VW_SPEED_PID_D);

>>>>>>> parent of f4dde99 (调参)
    /** �ȳ�PID **/
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

    // ��غ���ȳ�PID
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

    /** Roll PID **/
    pid_init(&chassis.chassis_roll_pid,
             CHASSIS_ROLL_PID_OUT_LIMIT,
             CHASSIS_ROLL_PID_IOUT_LIMIT,
             CHASSIS_ROLL_PID_P,
             CHASSIS_ROLL_PID_I,
             CHASSIS_ROLL_PID_D);
}

/** ���̳�ʼ�� **/
void chassis_init(void)
{
    /** ��ʼ������ģʽ **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** �ؽڵ����ʼ�� **/
    joint_init();

    /** ��챵����ʼ�� **/
    wheel_init();

    /** ����pid��ʼ�� **/
    chassis_pid_init();

    /** �ؽ��ƶ�ƽ���˲�����ʼ�� **/
    moving_average_filter_init(&chassis.leg_L.Fn_filter);
    moving_average_filter_init(&chassis.leg_R.Fn_filter);
    moving_average_filter_init(&chassis.leg_L.theta_ddot_filter);
    moving_average_filter_init(&chassis.leg_R.theta_ddot_filter);

    xvEstimateKF_Init(&vaEstimateKF);
}

/** ����ƽ���ж� **/
static void chassis_is_balanced(void) {

    if(!chassis.chassis_is_offground) // ��Ծʱ������ƽ���ж�
    {
        if (ABS(chassis.imu_reference.pitch_rad) <= 0.1744f) // -10�� ~ 10��
        {
            chassis.chassis_is_balance = true;

            if((ABS(chassis.imu_reference.pitch_rad) <= 0.05233f)) // -3�� ~ 3��
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

/** ���̵����Ծ� **/
static void chassis_selfhelp(void)
{
    chassis_is_balanced();

    if (!chassis.recover_finish)
    {
        chassis.leg_L.joint_F_torque = 0;
        chassis.leg_L.joint_B_torque = 0;
        chassis.leg_R.joint_F_torque = 0;
        chassis.leg_R.joint_B_torque = 0;

    }


}

/** ��ȡ���̴��������� **/
static void get_IMU_info(void) {

    /** Yaw **/
    chassis.imu_reference.yaw_rad = -INS.Yaw * DEGREE_TO_RAD;
    chassis.imu_reference.yaw_total_rad = -INS.YawTotalAngle * DEGREE_TO_RAD;

    /** Pitch **/
    chassis.imu_reference.pitch_rad = -INS.Roll * DEGREE_TO_RAD;

    /** Roll **/
    chassis.imu_reference.roll_rad = INS.Pitch * DEGREE_TO_RAD;

    /** ���¸�����ٶȺͽ��ٶ� **/
    chassis.imu_reference.pitch_gyro = -INS.Gyro[Y];
    chassis.imu_reference.yaw_gyro = -INS.Gyro[Z];
    chassis.imu_reference.roll_gyro = INS.Gyro[X];

    chassis.imu_reference.ax = INS.Accel[X];
    chassis.imu_reference.ay = INS.Accel[Y];
    chassis.imu_reference.az = INS.Accel[Z];

    /** ������ֱ������ٶ� **/
    chassis.imu_reference.robot_az = INS.MotionAccel_n[Z];
}

/** ���µ��̱��� **/
static void chassis_variable_update(void) {

    get_IMU_info();

    /** s s_dot **/

    // s_dot
    chassis.leg_L.state_variable_feedback.s_dot = vel_acc[0];
    chassis.leg_R.state_variable_feedback.s_dot = vel_acc[0];

    // s
    if(chassis.chassis_ctrl_info.s_dot != 0.0f)
    {
        chassis.leg_L.state_variable_feedback.s = 0.0f;
        chassis.leg_R.state_variable_feedback.s = 0.0f;
    }
    else
    {
        chassis.leg_L.state_variable_feedback.s = chassis.leg_L.state_variable_feedback.s + CHASSIS_PERIOD * 0.001f * chassis.leg_L.state_variable_feedback.s_dot;
        chassis.leg_R.state_variable_feedback.s = chassis.leg_R.state_variable_feedback.s + CHASSIS_PERIOD * 0.001f * chassis.leg_R.state_variable_feedback.s_dot;

    }

    // s_ddot
    chassis.leg_L.state_variable_feedback.s_dot_last = chassis.leg_L.state_variable_feedback.s_dot;
    chassis.leg_R.state_variable_feedback.s_dot_last = chassis.leg_R.state_variable_feedback.s_dot;

    chassis.leg_L.state_variable_feedback.s_ddot = (chassis.leg_L.state_variable_feedback.s_dot - chassis.leg_L.state_variable_feedback.s_dot_last) / (CHASSIS_PERIOD * 0.001f);
    chassis.leg_R.state_variable_feedback.s_ddot = (chassis.leg_R.state_variable_feedback.s_dot - chassis.leg_R.state_variable_feedback.s_dot_last) / (CHASSIS_PERIOD * 0.001f);

    /** yaw yaw_dot **/

    // yaw
    chassis.leg_L.state_variable_feedback.yaw = chassis.imu_reference.yaw_total_rad;
    chassis.leg_R.state_variable_feedback.yaw = chassis.imu_reference.yaw_total_rad;

    // yaw_dot
    chassis.leg_L.state_variable_feedback.yaw_dot = chassis.imu_reference.yaw_gyro;
    chassis.leg_R.state_variable_feedback.yaw_dot = chassis.imu_reference.yaw_gyro;

    /** phi phi_dot **/

    // phi
    chassis.leg_L.state_variable_feedback.theta_b = chassis.imu_reference.pitch_rad;
    chassis.leg_R.state_variable_feedback.theta_b = chassis.imu_reference.pitch_rad;

    // phi_dot
    chassis.leg_L.state_variable_feedback.theta_b_dot = chassis.imu_reference.pitch_gyro;
    chassis.leg_R.state_variable_feedback.theta_b_dot = chassis.imu_reference.pitch_gyro;


    /** theta theta_dot **/
    chassis.leg_L.state_variable_feedback.theta_l_last = chassis.leg_L.state_variable_feedback.theta_l;
    chassis.leg_R.state_variable_feedback.theta_r_last = chassis.leg_R.state_variable_feedback.theta_r;

    // theta
    chassis.leg_L.state_variable_feedback.theta_l = cal_leg_theta(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0, chassis.leg_L.state_variable_feedback.theta_b);
    chassis.leg_R.state_variable_feedback.theta_r = cal_leg_theta(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0, chassis.leg_R.state_variable_feedback.theta_b);

    // theta_dot
    chassis.leg_L.state_variable_feedback.theta_l_dot_last = chassis.leg_L.state_variable_feedback.theta_l_dot;
    chassis.leg_L.state_variable_feedback.theta_l_dot = (chassis.leg_L.state_variable_feedback.theta_l - chassis.leg_L.state_variable_feedback.theta_l_last) / (CHASSIS_PERIOD * 0.001f);

    chassis.leg_R.state_variable_feedback.theta_r_dot_last = chassis.leg_R.state_variable_feedback.theta_r_dot;
    chassis.leg_R.state_variable_feedback.theta_r_dot = (chassis.leg_R.state_variable_feedback.theta_r - chassis.leg_R.state_variable_feedback.theta_r_last) / (CHASSIS_PERIOD * 0.001f);

    // theta_ddot
    float theta_l_ddot_raw = (chassis.leg_L.state_variable_feedback.theta_l_dot - chassis.leg_L.state_variable_feedback.theta_l_dot_last) / (CHASSIS_PERIOD * 0.001f);
    chassis.leg_L.state_variable_feedback.theta_l_ddot = update_moving_average_filter(&chassis.leg_L.theta_ddot_filter, theta_l_ddot_raw);

    float theta_r_ddot_raw = (chassis.leg_R.state_variable_feedback.theta_r_dot - chassis.leg_R.state_variable_feedback.theta_r_dot_last) / (CHASSIS_PERIOD * 0.001f);
    chassis.leg_R.state_variable_feedback.theta_r_ddot = update_moving_average_filter(&chassis.leg_R.theta_ddot_filter, theta_r_ddot_raw);


}

/** �������������� **/
static void wheel_calc(void)
{
    /******************************* Wheel *************************************/
    chassis.leg_L.wheel_torque =  wheel_K_L[0] * (chassis.leg_L.state_variable_feedback.s - 0.0f)
                                + wheel_K_L[1] * (chassis.leg_L.state_variable_feedback.s_dot - chassis.chassis_ctrl_info.s_dot)
                                + wheel_K_L[2] * (chassis.leg_L.state_variable_feedback.yaw - chassis.chassis_ctrl_info.yaw_rad)
                                + wheel_K_L[3] * (chassis.leg_L.state_variable_feedback.yaw_dot - 0.0f)
                                + wheel_K_L[4] * (chassis.leg_L.state_variable_feedback.theta_l - 0.0f)
                                + wheel_K_L[5] * (chassis.leg_L.state_variable_feedback.theta_l_dot - 0.0f)
                                + wheel_K_L[6] * (chassis.leg_R.state_variable_feedback.theta_r - 0.0f)
                                + wheel_K_L[7] * (chassis.leg_R.state_variable_feedback.theta_r_dot - 0.0f)
                                + wheel_K_L[8] * (chassis.leg_L.state_variable_feedback.theta_b - PHI_BALANCE)
                                + wheel_K_L[9] * (chassis.leg_L.state_variable_feedback.theta_b_dot - 0.0f);

    chassis.leg_R.wheel_torque =    wheel_K_R[0] * (chassis.leg_R.state_variable_feedback.s - 0.0f)
                                  + wheel_K_R[1] * (chassis.leg_R.state_variable_feedback.s_dot - chassis.chassis_ctrl_info.s_dot)
                                  + wheel_K_R[2] * (chassis.leg_R.state_variable_feedback.yaw - chassis.chassis_ctrl_info.yaw_rad)
                                  + wheel_K_R[3] * (chassis.leg_R.state_variable_feedback.yaw_dot - 0.0f)
                                  + wheel_K_R[4] * (chassis.leg_L.state_variable_feedback.theta_l - 0.0f)
                                  + wheel_K_R[5] * (chassis.leg_L.state_variable_feedback.theta_l_dot - 0.0f)
                                  + wheel_K_R[6] * (chassis.leg_R.state_variable_feedback.theta_r - 0.0f)
                                  + wheel_K_R[7] * (chassis.leg_R.state_variable_feedback.theta_r_dot - 0.0f)
                                  + wheel_K_R[8] * (chassis.leg_R.state_variable_feedback.theta_b - PHI_BALANCE)
                                  + wheel_K_R[9] * (chassis.leg_R.state_variable_feedback.theta_b_dot - 0.0f);

<<<<<<< HEAD
=======
    if (chassis.chassis_ctrl_mode != CHASSIS_SPIN)
    {
        // ����ת������
        chassis.target_spin_speed = pid_calc(&chassis.chassis_vw_pos_pid,
                                           chassis.imu_reference.yaw_total_rad,
                                           chassis.chassis_ctrl_info.yaw_rad); // rad/s

        chassis.wheel_turn_torque = pid_calc(&chassis.chassis_vw_speed_pid,
                                             chassis.imu_reference.yaw_gyro, // ��/s
                                             chassis.target_spin_speed);

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
>>>>>>> parent of f4dde99 (调参)
    chassis.leg_R.wheel_torque *= -1;

    VAL_LIMIT(chassis.leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    VAL_LIMIT(chassis.leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);

}

/** ����ؽ����� **/
static void joint_calc(void)
{
/******************************* Joint *************************************/

    /** Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp **/

<<<<<<< HEAD
    //Left
    chassis.leg_R.vmc.forward_kinematics.Tb_F_set_point.E.Tb_set_point =  joint_K_L[0] * (chassis.leg_L.state_variable_feedback.s - 0.0f)
                                                                        + joint_K_L[1] * (chassis.leg_L.state_variable_feedback.s_dot - chassis.chassis_ctrl_info.s_dot)
                                                                        + joint_K_L[2] * (chassis.leg_L.state_variable_feedback.yaw - chassis.chassis_ctrl_info.yaw_rad)
                                                                        + joint_K_L[3] * (chassis.leg_L.state_variable_feedback.yaw_dot - 0.0f)
                                                                        + joint_K_L[4] * (chassis.leg_L.state_variable_feedback.theta_l - 0.0f)
                                                                        + joint_K_L[5] * (chassis.leg_L.state_variable_feedback.theta_l_dot - 0.0f)
                                                                        + joint_K_L[6] * (chassis.leg_R.state_variable_feedback.theta_r - 0.0f)
                                                                        + joint_K_L[7] * (chassis.leg_R.state_variable_feedback.theta_r_dot - 0.0f)
                                                                        + joint_K_L[8] * (chassis.leg_L.state_variable_feedback.theta_b - PHI_BALANCE)
                                                                        + joint_K_L[9] * (chassis.leg_L.state_variable_feedback.theta_b_dot - 0.0f);


    //Right
    chassis.leg_R.vmc.forward_kinematics.Tb_F_set_point.E.Tb_set_point =  joint_K_R[0] * (chassis.leg_R.state_variable_feedback.s - 0.0f)
                                                                        + joint_K_R[1] * (chassis.leg_R.state_variable_feedback.s_dot - chassis.chassis_ctrl_info.s_dot)
                                                                        + joint_K_R[2] * (chassis.leg_R.state_variable_feedback.yaw - chassis.chassis_ctrl_info.yaw_rad)
                                                                        + joint_K_R[3] * (chassis.leg_R.state_variable_feedback.yaw_dot - 0.0f)
                                                                        + joint_K_R[4] * (chassis.leg_L.state_variable_feedback.theta_l - 0.0f)
                                                                        + joint_K_R[5] * (chassis.leg_L.state_variable_feedback.theta_l_dot - 0.0f)
                                                                        + joint_K_R[6] * (chassis.leg_R.state_variable_feedback.theta_r - 0.0f)
                                                                        + joint_K_R[7] * (chassis.leg_R.state_variable_feedback.theta_r_dot - 0.0f)
                                                                        + joint_K_R[8] * (chassis.leg_R.state_variable_feedback.theta_b - PHI_BALANCE)
                                                                        + joint_K_R[9] * (chassis.leg_R.state_variable_feedback.theta_b_dot - 0.0f);
=======
    /****** ������pid ******/
    chassis.steer_compensatory_torque = pid_calc(&chassis.chassis_leg_coordination_pid,
                                                 chassis.phi0_error,
                                                 0);

//    //Left
//    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
//                                                                       + joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
//                                                                       + joint_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
//                                                                       + joint_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
//                                                                       + joint_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - PHI_BALANCE)
//                                                                       + joint_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);
//
//
//    //Right
//    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
//                                                                       + joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
//                                                                       + joint_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
//                                                                       + joint_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
//                                                                       + joint_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - PHI_BALANCE)
//                                                                       + joint_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);
//
//    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;
//    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;

    //Left
    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
                                                                         + joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f);


    //Right
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
                                                                         + joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f);

//    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;
//    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;
>>>>>>> parent of f4dde99 (调参)

    /** F F F F F F F F F F F F F F F **/

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


    chassis.leg_L.vmc.forward_kinematics.Tb_F_set_point.E.F_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_L.state_variable_feedback.theta_l)
                                                                         + chassis.leg_L.leg_speed_pid.out;

    chassis.leg_R.vmc.forward_kinematics.Tb_F_set_point.E.F_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_R.state_variable_feedback.theta_l)
                                                                        + chassis.leg_R.leg_speed_pid.out;

    chassis.leg_L.vmc.forward_kinematics.Tb_F_set_point.E.F_set_point += chassis.chassis_roll_pid.out;
    chassis.leg_R.vmc.forward_kinematics.Tb_F_set_point.E.F_set_point -= chassis.chassis_roll_pid.out;

    // ����ؽڵ������
    vmc_forward_dynamics(&chassis.leg_L.vmc, &chassis_physical_config);
    vmc_forward_dynamics(&chassis.leg_R.vmc, &chassis_physical_config);

    chassis.leg_L.joint_F_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_L.joint_B_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    chassis.leg_R.joint_F_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_R.joint_B_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    // ����޷�
    VAL_LIMIT(chassis.leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);

}

/** �����ȳ� **/
static void leg_length_set(void)
{
    if(switch_is_down(get_rc_ctrl()->rc.s[RC_s_L])){
        chassis.chassis_ctrl_info.height_m = 0.12f;
    }else if(switch_is_mid(get_rc_ctrl()->rc.s[RC_s_L])){
        chassis.chassis_ctrl_info.height_m = 0.22f;
    }else if(switch_is_up(get_rc_ctrl()->rc.s[RC_s_L])){
        chassis.chassis_ctrl_info.height_m = 0.35f;
    }
}

/** ���������� **/
static void controller_calc(void)
{
    /** ���������˲��� **/
    vmc_calc();
    /** �ٶ��ں� **/
    speed_calc();
    /** ���µ��̱��� **/
    chassis_variable_update();
    /** �������������� **/
    wheel_calc();
    /** ����ؽ����� **/
    joint_calc();
}


/*******************************************************************************
 *                                  Task                                       *
 *******************************************************************************/

/** ����ʧ������ **/
static void chassis_disable_task() {

    chassis.leg_L.wheel_torque = 0;
    chassis.leg_R.wheel_torque = 0;

    chassis.leg_L.joint_F_torque = 0;
    chassis.leg_L.joint_B_torque = 0;
    chassis.leg_R.joint_F_torque = 0;
    chassis.leg_R.joint_B_torque = 0;

    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    chassis.leg_L.state_variable_feedback.s = 0.0f;
    chassis.leg_R.state_variable_feedback.s = 0.0f;

    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

    chassis.chassis_ctrl_info.height_m = MIN_L0;

    /** ��ʼ����־λ **/
    chassis.init_flag = false;

    // ƽ���־λ
    chassis.chassis_is_balance = false;
    // �����Ծȳɹ���־λ
    chassis.recover_finish = false;

    // ��ر�־λ
    chassis.leg_L.leg_is_offground = false;
    chassis.leg_R.leg_is_offground = false;
    chassis.chassis_is_offground   = false;

    // ��Ծ��־λ
    chassis.jump_state = NOT_READY;
    chassis.jump_flag = false;

}

/** ���̳�ʼ������ **/
static void chassis_init_task()
{
    joint_enable();

    chassis.init_flag = true;
}

/** ����ʹ������ **/
static void chassis_enable_task(void)
{
    // �����ȳ����K
    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,
                             chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
                             chassis_fitting_factor);

    /** ���������ȳ� **/
    leg_length_set();

    controller_calc();

    chassis_selfhelp();
}

/** ������������ **/
// 500Hz
static void send_torque_task(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque,
                             float wheel_L_torque, float wheel_R_torque)
{
    /** DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM DM **/
    static int time = 0;

    if(time % 2 == 0)
    {
//        set_dm8009_MIT(&joint[LF],LF_pos,0.0f, Kp, Kd,joint_LF_torque);
//        set_dm8009_MIT(&joint[LB],LB_pos,0.0f, Kp, Kd,joint_LB_torque);
//        DWT_Delay(0.0002);
//        set_dm8009_MIT(&joint[RF],RF_pos,0.0f, Kp, Kd,joint_RF_torque);
//        set_dm8009_MIT(&joint[RB],RB_pos,0.0f, Kp, Kd,joint_RB_torque);
//
//        lk9025_multi_torque_set(wheel_L_torque, wheel_R_torque);

        set_dm8009_MIT(&joint[LF],0.0f,0.0f, 0.0f, 0.0f,0.0f);
        set_dm8009_MIT(&joint[LB],0.0f,0.0f, 0.0f, 0.0f,0.0f);
        DWT_Delay(0.0002);
        set_dm8009_MIT(&joint[RF],0.0f,0.0f, 0.0f, 0.0f,0.0f);
        set_dm8009_MIT(&joint[RB],0.0f,0.0f, 0.0f, 0.0f,0.0f);

        lk9025_multi_torque_set(0.0f, 0.0f);
    }

    time ++;

    if(time >= 100000000)
    {
        time = 0;
    }

}

// 1kHz
void chassis_task(void)
{
    /** ��ȡң������Ϣ(ģʽ + ����) **/
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
                     -chassis.leg_R.wheel_torque);

}

