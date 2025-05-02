#include "left_task.h"
#include "robot_def.h"
#include "wheel.h"
#include "joint.h"
#include "user_lib.h"
#include "ins_task.h"
#include "vmc.h"
#include "lqr.h"
#include "remote.h"
#include "error.h"

/*******************************************************************************
 *                                    Remote                                   *
 *******************************************************************************/

/** ģ�����ߴ��� **/
static void chassis_device_offline_handle() {
    check_is_rc_online(get_rc_ctrl());
    if (get_errors() != 0) {
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    }
}

/** ���̽���ң������Ϣ **/
static void set_chassis_ctrl_info() {
    chassis.chassis_ctrl_info.v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_SPEED_CHANNEL]) * RC_TO_VX;

    chassis.chassis_ctrl_info.yaw_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_YAW_CHANNEL]) * (-RC_TO_YAW_INCREMENT);

}

/** ���̸���ң��������ģʽ **/
static void set_chassis_mode() {
    if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) { // ʧ��
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    }
    else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == false) { // ��ʼ��ģʽ
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_INIT;
    }
    else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == true) { // ʹ��
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_ENABLE;

        if(switch_is_down(get_rc_ctrl()->rc.s[RC_s_L])){
            chassis.chassis_ctrl_info.height_m = 0.10f;
        }else if(switch_is_mid(get_rc_ctrl()->rc.s[RC_s_L])){
            chassis.chassis_ctrl_info.height_m = 0.22f;
        }else if(switch_is_up(get_rc_ctrl()->rc.s[RC_s_L])){
            chassis.chassis_ctrl_info.height_m = 0.35f;
        }

    }

}

/** ����ͨ�����ͨ�Ž�����̨����Ϣ **/
static void set_chassis_ctrl_info_from_gimbal_msg()
{

}

/** ���̸�����̨��Ϣ����ģʽ **/
static void set_chassis_mode_from_gimbal_msg()
{

}


static void left_pid_init(void)
{
    // ����PID

    /** �ȳ�PID **/
    pid_init(&chassis.leg_L.leg_pos_pid,
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

    // �����ȹ���PID

    /** ת��PID **/
    pid_init(&chassis.chassis_turn_pid,
             CHASSIS_TURN_PID_OUT_LIMIT,
             CHASSIS_TURN_PID_IOUT_LIMIT,
             CHASSIS_TURN_PID_P,
             CHASSIS_TURN_PID_I,
             CHASSIS_TURN_PID_D);

    /** ������PID **/
    pid_init(&chassis.chassis_leg_coordination_pid,
             CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_P,
             CHASSIS_LEG_COORDINATION_PID_I,
             CHASSIS_LEG_COORDINATION_PID_D);

    /** Roll PID **/
    pid_init(&chassis.chassis_roll_pid,
             CHASSIS_ROLL_PID_OUT_LIMIT,
             CHASSIS_ROLL_PID_IOUT_LIMIT,
             CHASSIS_ROLL_PID_P,
             CHASSIS_ROLL_PID_I,
             CHASSIS_ROLL_PID_D);
}

/** ���ȳ�ʼ�� **/
static void left_init(void)
{
    /** ��ʼ������ģʽ **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** ����챵����ʼ�� **/
    left_wheel_init();

    /** ��ؽڵ����ʼ�� **/
    left_joint_init();

    /** pid��ʼ��(��������pid�������ȹ��е�) **/
    left_pid_init();
}

static void left_feedback_update(void)
{
    /** vmc������Ƕȸ��� **/
    float LF_joint_pos = (get_joint_motors() + 0)->pos_r;
    float LB_joint_pos = get_joint_motors()[1].pos_r;

    chassis.leg_L.vmc.forward_kinematics.fk_phi.phi4 = PI/2 + LF_joint_pos;
    chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1 = PI/2 + LB_joint_pos;

    /** ״̬�������� **/
    // phi phi_dot
    chassis.leg_L.state_variable_feedback.phi = -INS.Roll * DEGREE_TO_RAD;
    chassis.leg_L.state_variable_feedback.phi_dot = -INS.Gyro[Y];

    /** ��̬�� **/
    // yaw
    chassis.imu_reference.yaw_rad = -INS.Yaw * DEGREE_TO_RAD;
    chassis.imu_reference.yaw_total_rad = -INS.YawTotalAngle * DEGREE_TO_RAD;
    chassis.imu_reference.yaw_gyro = -INS.Gyro[Z];

    // roll
    chassis.imu_reference.roll_rad = INS.Pitch * DEGREE_TO_RAD;

    /** ������ **/
    chassis.phi0_error = chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0;

}

/** ����ƽ���ж� **/
static void chassis_is_balanced() {

    if(!chassis.chassis_is_offground) // ��Ծʱ������ƽ���ж�
    {
        if (ABS(chassis.leg_L.state_variable_feedback.phi) <= 0.1744f) // -10�� ~ 10��
        {
            chassis.chassis_is_balance = true;

            if((ABS(chassis.leg_L.state_variable_feedback.phi) <= 0.05233f)) // -3�� ~ 3��
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
    if (!chassis.recover_finish)
    {
        chassis.leg_L.joint_F_torque = 0;
        chassis.leg_L.joint_B_torque = 0;
    }
}

/** ��CAN���߷����������� **/
static void left_motor_cmd_send() {

/** DEBUG_MODE: ��1ʱ�������ģʽ���رչؽں������� **/
#if DEBUG_MODE
    set_left_joint_torque(0.0f,
                          0.0f);

    set_left_wheel_torque(0.0f);
#else
    if(chassis.chassis_ctrl_mode != CHASSIS_DISABLE)
    {
        set_left_joint_torque(-chassis.leg_L.joint_F_torque,
                              -chassis.leg_L.joint_B_torque);

        set_left_wheel_torque(-chassis.leg_L.wheel_torque);
    }
    else
    {
        set_left_joint_torque(0.0f,
                              0.0f);

        set_left_wheel_torque(0.0f);
    }
#endif

}

/** ��������ؼ��� **/
static void left_wheel_torque_calc(void)
{
    /** ����ת������ **/
    if (chassis.chassis_ctrl_mode != CHASSIS_SPIN)
    {
        chassis.wheel_turn_torque =  CHASSIS_TURN_PID_P * (chassis.imu_reference.yaw_total_rad - chassis.chassis_ctrl_info.yaw_rad)
                                     + CHASSIS_TURN_PID_D * chassis.imu_reference.yaw_gyro;

    }else
    {
//      chassis->wheel_turn_torque = pid_calc(&chassis->chassis_spin_pid,
//                                              chassis->imu_reference.yaw_gyro,
//                                              chassis->chassis_ctrl_info.spin_speed);
    }

    /** LQR **/
    chassis.leg_L.wheel_torque =   wheel_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
                                 + wheel_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
                                 + wheel_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
                                 + wheel_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                 + wheel_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
                                 + wheel_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_L.wheel_torque += chassis.wheel_turn_torque;

    VAL_LIMIT(chassis.leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
}

/** ��ؽ����ؼ��� **/
static void left_joint_torque_calc(void)
{
    // Tp
    /** ���������pid **/
    chassis.steer_compensatory_torque = pid_calc(&chassis.chassis_leg_coordination_pid,
                                                  chassis.phi0_error,
                                                  0.0f);

    chassis.leg_L.vmc.forward_kinematics.F_Tp_set_point.E.Tp_set_point =   joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
                                                                        + joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
                                                                        + joint_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
                                                                        + joint_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                                                        + joint_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
                                                                        + joint_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_L.vmc.forward_kinematics.F_Tp_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;

    // F
    /****** �ȳ�pid ******/
    float L_L0_dot_set = pid_calc(&chassis.leg_L.leg_pos_pid,
                                  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,
                                  chassis.chassis_ctrl_info.height_m);

    pid_calc(&chassis.leg_L.leg_speed_pid,
             chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
             L_L0_dot_set);

    /****** Roll pid ******/
    pid_calc(&chassis.chassis_roll_pid,
             chassis.imu_reference.roll_rad,
             chassis.chassis_ctrl_info.roll_rad);

    chassis.leg_L.vmc.forward_kinematics.F_Tp_set_point.E.F_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_L.state_variable_feedback.theta)
                                                                       + chassis.leg_L.leg_speed_pid.out
                                                                       + chassis.chassis_roll_pid.out;

    /** ӳ�䵽��ʵ�ؽ����� **/
    vmc_forward_dynamics(&chassis.leg_L.vmc, &chassis_physical_config);

    chassis.leg_L.joint_F_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;
    chassis.leg_L.joint_B_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;

    VAL_LIMIT(chassis.leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
}

/*******************************************************************************
 *                                  Task                                       *
 *******************************************************************************/

/*********************** ��ʼ������ ***************************/
static void left_init_task()
{
    left_joint_enable();
    left_wheel_enable();

    chassis.left_init_flag = true;

    if(chassis.left_init_flag && chassis.right_init_flag)
    {
        chassis.init_flag = true;
    }
}

/*********************** ʧ������ ***************************/
static void left_disable_task()
{
    chassis.leg_L.joint_F_torque = 0.0f;
    chassis.leg_L.joint_B_torque = 0.0f;
    chassis.leg_L.wheel_torque = 0.0f;

    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    chassis.jump_state = NOT_READY;

    chassis.leg_L.state_variable_feedback.x = 0.0f;
    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

    /** ��ʼ����־λ **/

    // ��ʼ����־λ
    chassis.left_init_flag = false;

    // ƽ���־λ
    chassis.chassis_is_balance = false;

    // �����Ծȳɹ���־λ
    chassis.recover_finish = false;

    // ��ر�־λ
    chassis.leg_L.leg_is_offground = false;
    chassis.chassis_is_offground = false;

    // ��Ծ��־λ
    chassis.jump_flag = false;

}

/*********************** �ؽ�ʧ������ ***************************/
static void left_enable_task()
{
    chassis_is_balanced();

    vmc_forward_kinematics(&chassis.leg_L, &chassis_physical_config);

    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);

    left_wheel_torque_calc();
    left_joint_torque_calc();

    Inverse_Dynamics(&chassis.leg_L.vmc,
                     (get_joint_motors() + 1)->torque,
                     get_joint_motors()->torque,
                     &chassis_physical_config);

    Inverse_Kinematics(&chassis.leg_L.vmc,
                       (get_joint_motors() + 1)->angular_vel,
                       get_joint_motors()->angular_vel,
                       &chassis_physical_config);

    fn_cal(&chassis.leg_L, chassis.imu_reference.robot_az, &chassis_physical_config);

    chassis_selfhelp();
}

void left_task(void const *pvParameters)
{
    left_init();

    TickType_t last_wake_time = xTaskGetTickCount();

    while(1)
    {
        left_feedback_update();

        /** ����ң����Ϣ **/
#if CHASSIS_REMOTE
        set_chassis_mode();

        set_chassis_ctrl_info();

        chassis_device_offline_handle();
#else
        set_chassis_mode_from_gimbal_msg();
        set_chassis_ctrl_info_from_gimbal_msg();
#endif

        switch (chassis.chassis_ctrl_mode) {

            case CHASSIS_DISABLE:
                left_disable_task();
                break;

            case CHASSIS_INIT:
                left_init_task();
                break;

            case CHASSIS_ENABLE:
                left_enable_task();
                break;

            default:break;
        }

        left_motor_cmd_send();

        vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
    }
}