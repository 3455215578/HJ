#include <stdbool.h>
#include <math.h>
#include <cmsis_os.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>

#include "chassis.h"
#include "gimbal.h"

#include "lqr.h"
#include "Atti.h"
#include "user_lib.h"
#include "moving_filter.h"

#include "joint.h"
#include "wheel.h"
#include "remote.h"
#include "error.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//int jump_finish = 0;

extern Chassis chassis;
const ChassisPhysicalConfig chassis_physical_config = {0.075f,
                                                       5.486f,
                                                       1.18f,
                                                       0.2618f,
                                                       0.15f,
                                                       0.27f,
                                                       0.27f,
                                                       0.15f,
                                                       0.15f};


MovingAverageFilter robot_az_filter;

extern MovingAverageFilter theta_ddot_filter_L, theta_ddot_filter_R;

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
    chassis.chassis_ctrl_info.v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX;

    chassis.chassis_ctrl_info.x = (float) chassis.chassis_ctrl_info.x + chassis.chassis_ctrl_info.v_m_per_s * (CHASSIS_PERIOD * 0.001f);

    chassis.chassis_ctrl_info.yaw_angle_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_Z_CHANNEL]) * (-RC_TO_YAW_INCREMENT);

}

/** ���̸���ң��������ģʽ **/
static void set_chassis_mode() {
    if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) { // ʧ��
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == false) { // ��ʼ��ģʽ
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_INIT;
    } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == true) { // ʹ��
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
    }
}

/** ����ͨ�����ͨ�Ž�����̨����Ϣ **/
static void set_chassis_ctrl_info_from_gimbal_msg() {
    chassis.chassis_ctrl_info.v_m_per_s = get_gimbal_msg()->chassis_ctrl_info.v_m_per_s;
    chassis.chassis_ctrl_info.yaw_angle_rad = get_gimbal_msg()->chassis_ctrl_info.yaw_angle_rad;
    chassis.chassis_ctrl_info.roll_angle_rad = get_gimbal_msg()->chassis_ctrl_info.roll_angle_rad;
    chassis.chassis_ctrl_info.height_m = get_gimbal_msg()->chassis_ctrl_info.height_m;
}

/** ���̸�����̨��Ϣ����ģʽ **/
static void set_chassis_mode_from_gimbal_msg() {
    if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_DISABLE) {
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_ENABLE && chassis.init_flag == false) {
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_INIT;
    } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_ENABLE && chassis.init_flag == true) {
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
    }
}

/*******************************************************************************
 *                                 Function                                    *
 *******************************************************************************/

/*********************** ��ȡ���̴�������Ϣ *************************/
static void get_IMU_info() {

    /** Yaw **/
    chassis.imu_reference.yaw_angle = -*(get_ins_angle() + 0);

    // Ȧ�����
    if (chassis.imu_reference.yaw_angle - chassis.imu_reference.yaw_last_angle > 3.1415926f)
    {
        chassis.imu_reference.yaw_round_count--;
    }
    else if (chassis.imu_reference.yaw_angle - chassis.imu_reference.yaw_last_angle < -3.1415926f)
    {
        chassis.imu_reference.yaw_round_count++;
    }
    chassis.imu_reference.yaw_total_angle = 6.283f * chassis.imu_reference.yaw_round_count + chassis.imu_reference.yaw_angle;
    chassis.imu_reference.yaw_last_angle = chassis.imu_reference.yaw_angle;

    /** Pitch **/
    chassis.imu_reference.pitch_angle = -*(get_ins_angle() + 2);

    /** Roll **/
    chassis.imu_reference.roll_angle = -*(get_ins_angle() + 1);

    /** ���¸�����ٶȺͽ��ٶ� **/
    chassis.imu_reference.pitch_gyro = -*(get_ins_gyro() + 0);
    chassis.imu_reference.yaw_gyro = -*(get_ins_gyro() + 2);
    chassis.imu_reference.roll_gyro = -*(get_ins_gyro() + 1);

    chassis.imu_reference.ax = -*(get_ins_accel() + 1);
    chassis.imu_reference.ay = *(get_ins_accel() + 0);
    chassis.imu_reference.az = *(get_ins_accel() + 2);

    /** ȥ������Ӱ��ĸ�����ٶ�  ��ת���� **/
    chassis.imu_reference.ax_filtered = chassis.imu_reference.ax - GRAVITY * sinf(chassis.imu_reference.pitch_angle);
    chassis.imu_reference.ay_filtered =  chassis.imu_reference.ay
                                         - GRAVITY * cosf(chassis.imu_reference.pitch_angle) * sinf(chassis.imu_reference.roll_angle);
    chassis.imu_reference.az_filtered =  chassis.imu_reference.az
                                         - GRAVITY * cosf(chassis.imu_reference.pitch_angle) * cosf(chassis.imu_reference.roll_angle);

    /** ������ֱ������ٶ� **/
    float robot_az_raw =  chassis.imu_reference.ax_filtered * sinf(chassis.imu_reference.pitch_angle)
                          + chassis.imu_reference.ay_filtered * sinf(-chassis.imu_reference.roll_angle) * cosf(chassis.imu_reference.pitch_angle)
                          + chassis.imu_reference.az_filtered * cosf(chassis.imu_reference.pitch_angle) * cosf(chassis.imu_reference.roll_angle);

    update_moving_average_filter(&robot_az_filter, robot_az_raw);
    chassis.imu_reference.robot_az = get_moving_average_filtered_value(&robot_az_filter);

}

/************************ ����pid��ʼ�� **********************/
static void chassis_pid_init() {

    // ת��PID
    pid_init(&chassis.chassis_turn_pid,
             CHASSIS_TURN_PID_OUT_LIMIT,
             CHASSIS_TURN_PID_IOUT_LIMIT,
             CHASSIS_TURN_PID_P,
             CHASSIS_TURN_PID_I,
             CHASSIS_TURN_PID_D);

    // �ȳ�PID
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

    // ������PID
    pid_init(&chassis.chassis_leg_coordination_pid,
             CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_P,
             CHASSIS_LEG_COORDINATION_PID_I,
             CHASSIS_LEG_COORDINATION_PID_D);

    // Roll PID
    pid_init(&chassis.chassis_roll_pid,
             CHASSIS_ROLL_PID_OUT_LIMIT,
             CHASSIS_ROLL_PID_IOUT_LIMIT,
             CHASSIS_ROLL_PID_P,
             CHASSIS_ROLL_PID_I,
             CHASSIS_ROLL_PID_D);
}

/************************ ������ز�����ʼ�� **********************/
void chassis_init() {

    chassis.leg_L.leg_index = L;
    chassis.leg_R.leg_index = R;

    /** ����pid��ʼ�� **/
    chassis_pid_init();

    /** �ƶ�ƽ���˲�����ʼ�� **/
    moving_average_filter_init(&robot_az_filter);
    moving_average_filter_init(&theta_ddot_filter_L);
    moving_average_filter_init(&theta_ddot_filter_R);

//  chassis.chassis_ctrl_info.spin_speed = 5.0f;

    vTaskSuspendAll();

    /** ���-�ٶ��ںϼ��ٶ� �������˲��� ��ʼ�� **/
    chassis_kalman_init(&chassis.vx_kalman);

    xTaskResumeAll();
}

/************************ ����̵���������� **********************/
static void chassis_motor_cmd_send() {

/** DEBUG_MODE: ��1ʱ�������ģʽ���رչؽں������� **/
#if DEBUG_MODE
    set_joint_torque(0, 0, 0, 0);
    osDelay(2);
    set_wheel_torque(0, 0);

#else

//    set_joint_torque(-chassis.leg_L.joint_F_torque,
//                   -chassis.leg_L.joint_B_torque,
//                   chassis.leg_R.joint_F_torque,
//                   chassis.leg_R.joint_B_torque);

  set_joint_torque(0, 0, 0, 0);

//  set_wheel_torque(-chassis.leg_L.wheel_torque, -chassis.leg_R.wheel_torque);

  set_wheel_torque(0, 0);

#endif
}

/********************** ���ؽں�����Ƿ�ʹ�ܳɹ� **************************/
static void check_enable(void)
{
    if (!chassis.is_joint_enable) {
        set_dm8009_enable(CAN_2, JOINT_LF_SEND);
        set_dm8009_enable(CAN_2, JOINT_LB_SEND);
        HAL_Delay(2);
        set_dm8009_enable(CAN_2, JOINT_RF_SEND);
        set_dm8009_enable(CAN_2, JOINT_RB_SEND);
        HAL_Delay(2);

        chassis.is_joint_enable = true;
        return;
    }

    if(!chassis.is_wheel_enable)
    {
        lk9025_set_enable(CAN_1,WHEEL_L_SEND);
        HAL_Delay(2);
        lk9025_set_enable(CAN_1,WHEEL_R_SEND);
        HAL_Delay(2);

        chassis.is_wheel_enable = true;
        return;
    }

}

/*************************** ���ص��̽ṹ��ָ�� *****************************/
Chassis *get_chassis() {
  return &chassis;
}

/*********************************************************************************/




/*******************************************************************************
 *                                    Task                                     *
 *******************************************************************************/


/************************* ʧ������ ***************************/
static void chassis_disable_task() {

    chassis.leg_L.wheel_torque = 0;
    chassis.leg_R.wheel_torque = 0;
    chassis.leg_L.joint_F_torque = 0;
    chassis.leg_L.joint_B_torque = 0;
    chassis.leg_R.joint_F_torque = 0;
    chassis.leg_R.joint_B_torque = 0;

    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    chassis.leg_L.state_variable_feedback.x = 0;
    chassis.leg_R.state_variable_feedback.x = 0;

    chassis.chassis_ctrl_info.height_m = MIN_L0;

    chassis.chassis_ctrl_info.yaw_angle_rad = chassis.imu_reference.yaw_total_angle;

    /** ��ʼ����־λ **/
    chassis.is_joint_enable = false; // �ؽڵ��ʹ�ܱ�־λ
    chassis.is_wheel_enable = false; // ��챵��ʹ�ܱ�־λ
    chassis.init_flag = false; // ��ʼ���ɹ���־λ

    chassis.is_chassis_balance = false; // ƽ���־λ
    chassis.recover_finish = false; // �����Ծȳɹ���־λ
    chassis.is_chassis_offground = false; // ��ر�־λ
    chassis.jump_flag = false; // ��Ծ��־λ ��1ʱ������Ծģʽ

    /**
     * ��ΪLK����ϵ�Ĭ��ʹ�ܣ�֮ǰ�й�ң����ʧ�ܺ���챵����Ȼ��ת������
     * ���ѡ����INITģʽ��ʹ�ܣ�����ʧ�ܺ�ʧ����챵��
     * **/
    wheel_disable();
}

/*********************** ��ʼ������ ***************************/
static void chassis_init_task() {

    HAL_Delay(1500); // �ϵ���ʱ1500ms�ٷ��ؽڵ��ʹ�ܱ��ģ�����ʹ�ܲ���

    /** �ؽڵ��ʹ�� **/
    joint_enable();
    /** ��챵��ʹ�� **/
    wheel_enable();

    check_enable();

    if(chassis.is_joint_enable && chassis.is_wheel_enable)
    {
        chassis.init_flag = true;
    }

}

/*********************** ʹ������ ****************************/
static void chassis_enable_task() {

}

/*********************** ������ ******************************/
extern void chassis_task(void const *pvParameters) {

    chassis_init();

    // ???
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {

        // ???
        vTaskSuspendAll();

        get_IMU_info();

#if CHASSIS_REMOTE
        set_chassis_mode();

        set_chassis_ctrl_info();
#else
        set_chassis_mode_from_gimbal_msg();
        set_chassis_ctrl_info_from_gimbal_msg();
#endif
        chassis_device_offline_handle();

        switch (chassis.chassis_ctrl_mode) {
            case CHASSIS_INIT:
                chassis_init_task();
                break;

            case CHASSIS_ENABLE:
                chassis_enable_task();
                break;

            case CHASSIS_DISABLE:
                chassis_disable_task();
                break;

            default:break;
        }

        // ???
        xTaskResumeAll();

        chassis_motor_cmd_send();

        // ???
        vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
    }
}
/*********************************************************************************************/
