#include "App_wheel.h"
#include "robot_def.h"
#include "Device_wheel.h"

#include "cmsis_os.h"

#include "remote.h"
#include "error.h"
#include "ins.h"
#include "user_lib.h"
#include "vx_kalman_filter.h"

Wheel App_wheel;

extern KalmanFilter_t vaEstimateKF;

/**********************  Function  ******************************/

/** ��챽���ң������Ϣ **/
static void set_wheel_ctrl_info() {

    App_wheel.wheel_ctrl_info.v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_SPEED_CHANNEL]) * RC_TO_VX;
    App_wheel.wheel_ctrl_info.yaw_angle_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_YAW_CHANNEL]) * (-RC_TO_YAW_INCREMENT);
}

/** ��챸���ң��������ģʽ **/
static void set_joint_mode() {
    if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) { // ʧ��
        App_wheel.wheel_ctrl_mode_last = App_wheel.wheel_ctrl_mode;
        App_wheel.wheel_ctrl_mode = WHEEL_DISABLE;
    }
    else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && App_wheel.init_flag == false) { // ��ʼ��ģʽ
        App_wheel.wheel_ctrl_mode_last = App_wheel.wheel_ctrl_mode;
        App_wheel.wheel_ctrl_mode = WHEEL_INIT;
    }
    else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && App_wheel.init_flag == true) { // ʹ��
        App_wheel.wheel_ctrl_mode_last = App_wheel.wheel_ctrl_mode;
        App_wheel.wheel_ctrl_mode = WHEEL_ENABLE;

    }
}

/** ��챹���ģ�����ߵĴ��� **/
static void joint_device_offline_handle() {
    check_is_rc_online(get_rc_ctrl());
    if (get_errors() != 0) {
        App_wheel.wheel_ctrl_mode = WHEEL_DISABLE;
    }
}

static void wheel_get_IMU_info() {

    /** Yaw **/
    App_wheel.imu_reference.yaw_angle = -INS.Yaw * DEGREE_TO_RAD;
    App_wheel.imu_reference.yaw_total_angle = -INS.YawTotalAngle * DEGREE_TO_RAD;

    /** Pitch **/
    App_wheel.imu_reference.pitch_angle = -INS.Roll * DEGREE_TO_RAD;

    /** Roll **/
    App_wheel.imu_reference.roll_angle = INS.Pitch * DEGREE_TO_RAD;

    /** ���¸�����ٶȺͽ��ٶ� **/
    App_wheel.imu_reference.pitch_gyro = -INS.Gyro[Y];

    /** ������ֱ������ٶ� **/
    App_wheel.imu_reference.robot_az = INS.MotionAccel_n[Z];

}

static void wheel_motor_cmd_send() {

/** DEBUG_MODE: ��1ʱ�������ģʽ���رչؽں������� **/
#if DEBUG_MODE
    set_wheel_torque(0, 0);
#else

    if(App_wheel.wheel_ctrl_mode != WHEEL_DISABLE)
    {
        set_wheel_torque(-leg_L.wheel_torque, -leg_R.wheel_torque);}
    else
    {
        for(int i = 0; i < 10; i++)
        {
            set_wheel_torque(0, 0);
            osDelay(1);
        }

    }
#endif
}

static void Wheel_Pid_Init(void)
{
    /** ת��PID **/
    pid_init(&App_wheel.chassis_turn_pid,
             CHASSIS_TURN_PID_OUT_LIMIT,
             CHASSIS_TURN_PID_IOUT_LIMIT,
             CHASSIS_TURN_PID_P,
             CHASSIS_TURN_PID_I,
             CHASSIS_TURN_PID_D);
}

static void Wheel_Init() {

    /** ��ʼ�����ģʽ **/
    App_wheel.wheel_ctrl_mode = WHEEL_DISABLE;

    /** ��챵����ʼ�� **/
    wheel_init();

    /** ����pid��ʼ�� **/
    Wheel_Pid_Init();

    vTaskSuspendAll();

    /** ���-�ٶ��ںϼ��ٶ� �������˲��� ��ʼ�� **/
    xvEstimateKF_Init(&vaEstimateKF);

    xTaskResumeAll();
}

static void Wheel_Torque_Calc(void)
{

}


/*********************** ���� ***************************/
static void wheel_disable_task()
{
    leg_L.wheel_torque = 0;
    leg_R.wheel_torque = 0;

    App_wheel.wheel_ctrl_mode = WHEEL_DISABLE;

    leg_L.state_variable_feedback.x = 0.0f;
    leg_R.state_variable_feedback.x = 0.0f;

    App_wheel.wheel_ctrl_info.yaw_angle_rad = App_wheel.imu_reference.yaw_total_angle;

}

static void joint_init_task() {

    /** ��챻�ʹ�� **/
    wheel_enable();

    /** ��챵����ʼ����� **/
    App_wheel.init_flag = true;
}

/*********************** ʹ������ ****************************/
static void chassis_enable_task() {

    lqr_ctrl();

    vmc_ctrl();

    chassis_selfhelp();

    chassis_vx_kalman_run();

//    chassis_is_offground();

}