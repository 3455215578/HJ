#include <stdbool.h>
#include <math.h>
#include <cmsis_os.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>

#include "chassis.h"
#include "gimbal.h"
#include "ins.h"

#include "lqr.h"
#include "user_lib.h"
#include "moving_filter.h"
#include "vx_kalman_filter.h"

#include "Device_joint.h"
#include "Device_wheel.h"
#include "remote.h"
#include "error.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern Chassis chassis;

extern KalmanFilter_t vaEstimateKF;


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

//    float target_speed = (float) (get_rc_ctrl()->rc.ch[CHASSIS_SPEED_CHANNEL]) * RC_TO_VX;
//
//    // ����
//    if(target_speed != 0.0f)
//    {
//        slope_following(&target_speed, &chassis.chassis_ctrl_info.v_m_per_s, 0.02f);
//    }
//    else // ����
//    {
//        slope_following(&target_speed, &chassis.chassis_ctrl_info.v_m_per_s, 0.05f);
//
//    }
    chassis.chassis_ctrl_info.x = chassis.chassis_ctrl_info.x + CHASSIS_PERIOD * 0.001f * chassis.chassis_ctrl_info.v_m_per_s;

    chassis.chassis_ctrl_info.yaw_angle_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_YAW_CHANNEL]) * (-RC_TO_YAW_INCREMENT);

//    chassis.chassis_ctrl_info.height_m = chassis.chassis_ctrl_info.height_m + (float) (get_rc_ctrl()->rc.ch[TEST_CHASSIS_LEG_CHANNEL]) * 0.00001f;
//    VAL_LIMIT(chassis.chassis_ctrl_info.height_m, MIN_L0, MAX_L0);

//    chassis.chassis_ctrl_info.roll_angle_rad = (float) (get_rc_ctrl()->rc.ch[TEST_CHASSIS_ROLL_CHANNEL]) * 0.001f;

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

    }

}

/** ����ͨ�����ͨ�Ž�����̨����Ϣ **/
static void set_chassis_ctrl_info_from_gimbal_msg() {
    chassis.chassis_ctrl_info.v_m_per_s = get_gimbal_msg()->chassis_ctrl_info.v_m_per_s;
    chassis.chassis_ctrl_info.x = chassis.chassis_ctrl_info.x + CHASSIS_PERIOD * 0.001f * chassis.chassis_ctrl_info.v_m_per_s;
//    chassis.chassis_ctrl_info.yaw_angle_rad = get_gimbal_msg()->chassis_ctrl_info.yaw_angle_rad;
//    chassis.chassis_ctrl_info.roll_angle_rad = get_gimbal_msg()->chassis_ctrl_info.roll_angle_rad;
//    chassis.chassis_ctrl_info.height_m = get_gimbal_msg()->chassis_ctrl_info.height_m;
//    VAL_LIMIT(chassis.chassis_ctrl_info.height_m, MIN_L0, MAX_L0);
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
    chassis.imu_reference.yaw_angle = -INS.Yaw * DEGREE_TO_RAD;
    chassis.imu_reference.yaw_total_angle = -INS.YawTotalAngle * DEGREE_TO_RAD;

    /** Pitch **/
    chassis.imu_reference.pitch_angle = -INS.Roll * DEGREE_TO_RAD;

    /** Roll **/
    chassis.imu_reference.roll_angle = INS.Pitch * DEGREE_TO_RAD;

    /** ���¸�����ٶȺͽ��ٶ� **/
    chassis.imu_reference.pitch_gyro = -INS.Gyro[Y];
    chassis.imu_reference.yaw_gyro = -INS.Gyro[Z];

    /** ������ֱ������ٶ� **/
    chassis.imu_reference.robot_az = INS.MotionAccel_n[Z];

}

/************************ ����̵���������� **********************/

float LF_pos = 0.0f;
float LB_pos = 0.0f;
float RF_pos = 0.0f;
float RB_pos = 0.0f;
float p = 0.0f;
float d = 0.0f;

static void chassis_motor_cmd_send() {

/** DEBUG_MODE: ��1ʱ�������ģʽ���رչؽں������� **/
#if DEBUG_MODE
    set_joint_torque(0, 0, 0, 0);
    osDelay(2);
    set_wheel_torque(0, 0);

#else

    if(chassis.chassis_ctrl_mode != CHASSIS_DISABLE)
    {

        set_wheel_torque(-chassis.leg_L.wheel_torque, -chassis.leg_R.wheel_torque);

        set_joint_torque(-chassis.leg_L.joint_F_torque,
                         -chassis.leg_L.joint_B_torque,
                         chassis.leg_R.joint_F_torque,
                         chassis.leg_R.joint_B_torque);

//        osDelay(1);
//        set_wheel_torque(-chassis.leg_L.wheel_torque, -chassis.leg_R.wheel_torque);

    }
    else
    {
        for(int i = 0; i < 10; i++)
        {
            set_joint_torque(0.0f,
                             0.0f,
                             0.0f,
                             0.0f);
            osDelay(1);
        }
        for(int i = 0; i < 10; i++)
        {
            set_wheel_torque(0, 0);
            osDelay(1);
        }


    }




#endif
}

/************************ ����pid��ʼ�� **********************/
static void chassis_pid_init() {

    /** ת��PID **/
    pid_init(&chassis.chassis_turn_pid,
             CHASSIS_TURN_PID_OUT_LIMIT,
             CHASSIS_TURN_PID_IOUT_LIMIT,
             CHASSIS_TURN_PID_P,
             CHASSIS_TURN_PID_I,
             CHASSIS_TURN_PID_D);

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

/************************ ������ز�����ʼ�� **********************/
static void chassis_init() {

    /** ��ʼ������ģʽ **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** ��챵����ʼ�� **/
    wheel_init();

    /** �ؽڵ����ʼ�� **/
    joint_init();

    /** ����pid��ʼ�� **/
    chassis_pid_init();

    /** �ƶ�ƽ���˲�����ʼ�� **/
    moving_average_filter_init(&chassis.leg_L.Fn_filter);
    moving_average_filter_init(&chassis.leg_R.Fn_filter);
    moving_average_filter_init(&chassis.leg_L.theta_ddot_filter);
    moving_average_filter_init(&chassis.leg_R.theta_ddot_filter);

//  chassis.chassis_ctrl_info.spin_speed = 5.0f;

    vTaskSuspendAll();

    /** ���-�ٶ��ںϼ��ٶ� �������˲��� ��ʼ�� **/
    xvEstimateKF_Init(&vaEstimateKF);

    xTaskResumeAll();
}

static void chassis_is_balanced() {

    if(!chassis.chassis_is_offground) // ��Ծʱ������ƽ���ж�
    {
        if (ABS(chassis.imu_reference.pitch_angle) <= 0.1744f) // -10�� ~ 10��
        {
            chassis.chassis_is_balance = true;

            if((ABS(chassis.imu_reference.pitch_angle) <= 0.05233f)) // -3�� ~ 3��
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




// �����Ծ�
static void chassis_selfhelp(void)
{
    if (!chassis.recover_finish)
    {
//        joint_reset();

        chassis.leg_L.joint_F_torque = 0;
        chassis.leg_L.joint_B_torque = 0;
        chassis.leg_R.joint_F_torque = 0;
        chassis.leg_R.joint_B_torque = 0;

//        if(!chassis.joint_is_reset)
//        {
//            chassis.leg_L.wheel_torque = 0.0f;
//            chassis.leg_R.wheel_torque = 0.0f;
//        }
    }
}



// ������ؼ��
static void chassis_is_offground(void)
{
    if(!chassis.recover_finish)
    {
        chassis.chassis_is_offground = false;
    }
    else
    {
        if(chassis.leg_L.leg_is_offground || chassis.leg_R.leg_is_offground)
        {
            chassis.chassis_is_offground = true;
        }else{
            chassis.chassis_is_offground = false;
        }
    }

}


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
    chassis.jump_state = NOT_READY;

    chassis.leg_L.state_variable_feedback.x = 0.0f;
    chassis.leg_R.state_variable_feedback.x = 0.0f;

    chassis.chassis_ctrl_info.yaw_angle_rad = chassis.imu_reference.yaw_total_angle;

    chassis.chassis_ctrl_info.height_m = MIN_L0;


    /** ��ʼ����־λ **/
    // �ؽ�Ĭ�ϸ�λ
//    chassis.joint_is_reset = true;

    // ���̳�ʼ����־λ
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
    chassis.jump_flag = false;


    /**
     * ��ΪLK����ϵ�Ĭ��ʹ�ܣ�֮ǰ�й�ң����ʧ�ܺ���챵����Ȼ��ת������
     * ���ѡ����INITģʽ��ʹ�ܣ�����ʧ�ܺ�ֹͣ��챵��������ʱ����Կ��Խ�����Ϣ����������
     * **/
    wheel_stop();
}

/*********************** ��ʼ������ ***************************/
static void chassis_init_task() {

    joint_enable();
    wheel_enable();

    chassis.init_flag = true;
}

/*********************** ʹ������ ****************************/
static void chassis_enable_task() {

    chassis.jump_state = NOT_READY;

    if(switch_is_down(get_rc_ctrl()->rc.s[RC_s_L])){
        chassis.chassis_ctrl_info.height_m = 0.10f;
    }else if(switch_is_mid(get_rc_ctrl()->rc.s[RC_s_L])){
        chassis.chassis_ctrl_info.height_m = 0.22f;
    }else if(switch_is_up(get_rc_ctrl()->rc.s[RC_s_L])){
        chassis.chassis_ctrl_info.height_m = 0.35f;
    }

    chassis_is_balanced();

    lqr_ctrl();

    vmc_ctrl();

    chassis_selfhelp();

    chassis_vx_kalman_run();

//    chassis_is_offground();

}



/*********************** ��Ծ���� ****************************/


/*********************** ������ ******************************/
extern void chassis_task(void const *pvParameters) {

    chassis_init();

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {

        get_IMU_info();

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
                chassis_disable_task();
                break;

            case CHASSIS_INIT:
                chassis_init_task();
                break;

            case CHASSIS_ENABLE:
                chassis_enable_task();
                break;

            default:break;
        }

        chassis_motor_cmd_send();

        vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
    }
}
/*********************************************************************************************/
