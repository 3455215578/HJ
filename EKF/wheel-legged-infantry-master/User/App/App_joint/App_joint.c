#include "App_joint.h"
#include "ins.h"

#include "robot_def.h"

#include "device_joint.h"
#include "remote.h"
#include "user_lib.h"
#include "error.h"
#include "vmc.h"

Joint App_joint;
extern ChassisPhysicalConfig chassis_physical_config;

/**********************  Function  ******************************/
static void joint_state_variable_update(void)
{

}

/** 关节接收遥控器信息 **/
static void set_joint_ctrl_info() {

    App_joint.joint_ctrl_info.v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_SPEED_CHANNEL]) * RC_TO_VX;
}

/** 关节根据遥控器设置模式 **/
static void set_joint_mode() {
    if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) { // 失能
        App_joint.joint_ctrl_mode_last = App_joint.joint_ctrl_mode;
        App_joint.joint_ctrl_mode = JOINT_DISABLE;
    }
    else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && App_joint.init_flag == false) { // 初始化模式
        App_joint.joint_ctrl_mode_last = App_joint.joint_ctrl_mode;
        App_joint.joint_ctrl_mode = JOINT_INIT;
    }
    else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && App_joint.init_flag == true) { // 使能
        App_joint.joint_ctrl_mode_last = App_joint.joint_ctrl_mode;
        App_joint.joint_ctrl_mode = JOINT_ENABLE;

    }
}

/** 关节关于模块离线的处理 **/
static void joint_device_offline_handle() {
    check_is_rc_online(get_rc_ctrl());
    if (get_errors() != 0) {
        App_joint.joint_ctrl_mode = JOINT_DISABLE;
    }
}

static void joint_get_IMU_info() {

    /** Yaw **/
    App_joint.imu_reference.yaw_angle = -INS.Yaw * DEGREE_TO_RAD;
    App_joint.imu_reference.yaw_total_angle = -INS.YawTotalAngle * DEGREE_TO_RAD;

    /** Pitch **/
    App_joint.imu_reference.pitch_angle = -INS.Roll * DEGREE_TO_RAD;

    /** Roll **/
    App_joint.imu_reference.roll_angle = INS.Pitch * DEGREE_TO_RAD;

    /** 更新各轴加速度和角速度 **/
    App_joint.imu_reference.pitch_gyro = -INS.Gyro[Y];

    /** 机体竖直方向加速度 **/
    App_joint.imu_reference.robot_az = INS.MotionAccel_n[Z];

}


static void joint_motor_cmd_send() {

/** DEBUG_MODE: 置1时进入调试模式，关闭关节和轮毂输出 **/
#if DEBUG_MODE
    set_joint_torque(0, 0, 0, 0);
#else

    if(App_joint.joint_ctrl_mode != JOINT_DISABLE)
    {
        set_joint_torque(-leg_L.joint_F_torque,
                         -leg_L.joint_B_torque,
                         leg_R.joint_F_torque,
                         leg_R.joint_B_torque);
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

    }
#endif
}

static void Joint_Pid_Init(void)
{
    /** 腿长PID **/
    pid_init(&leg_L.leg_pos_pid,
             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_P,
             CHASSIS_LEG_L0_POS_PID_I,
             CHASSIS_LEG_L0_POS_PID_D);

    pid_init(&leg_R.leg_pos_pid,
             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_POS_PID_P,
             CHASSIS_LEG_L0_POS_PID_I,
             CHASSIS_LEG_L0_POS_PID_D);

    pid_init(&leg_L.leg_speed_pid,
             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_P,
             CHASSIS_LEG_L0_SPEED_PID_I,
             CHASSIS_LEG_L0_SPEED_PID_D);

    pid_init(&leg_R.leg_speed_pid,
             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT,
             CHASSIS_LEG_L0_SPEED_PID_P,
             CHASSIS_LEG_L0_SPEED_PID_I,
             CHASSIS_LEG_L0_SPEED_PID_D);

    // 离地后的腿长PID
    pid_init(&leg_L.offground_leg_pid,
             CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT,
             CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT,
             CHASSIS_OFFGROUND_LO_PID_P,
             CHASSIS_OFFGROUND_L0_PID_I,
             CHASSIS_OFFGROUND_L0_PID_D);

    pid_init(&leg_R.offground_leg_pid,
             CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT,
             CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT,
             CHASSIS_OFFGROUND_LO_PID_P,
             CHASSIS_OFFGROUND_L0_PID_I,
             CHASSIS_OFFGROUND_L0_PID_D);

    /** 防劈叉PID **/
    pid_init(&App_joint.chassis_leg_coordination_pid,
             CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_P,
             CHASSIS_LEG_COORDINATION_PID_I,
             CHASSIS_LEG_COORDINATION_PID_D);

    /** Roll PID **/
    pid_init(&App_joint.chassis_roll_pid,
             CHASSIS_ROLL_PID_OUT_LIMIT,
             CHASSIS_ROLL_PID_IOUT_LIMIT,
             CHASSIS_ROLL_PID_P,
             CHASSIS_ROLL_PID_I,
             CHASSIS_ROLL_PID_D);
}

static void Joint_Init(void)
{
    /** 初始化关节模式 **/
    App_joint.joint_ctrl_mode = JOINT_DISABLE;

    /** 关节电机初始化 **/
    joint_init();

    /** 关节Pid初始化 **/
    Joint_Pid_Init();

    /** 移动平均滤波器初始化 **/
    moving_average_filter_init(&leg_L.Fn_filter);
    moving_average_filter_init(&leg_R.Fn_filter);
    moving_average_filter_init(&leg_L.theta_ddot_filter);
    moving_average_filter_init(&leg_R.theta_ddot_filter);
}

static void joint_chassis_is_balanced() {

    if(!App_joint.chassis_is_offground) // 跳跃时不进行平衡判断
    {
        if (ABS(App_joint.imu_reference.pitch_angle) <= 0.1744f) // -10° ~ 10°
        {
            App_joint.chassis_is_balance = true;

            if((ABS(App_joint.imu_reference.pitch_angle) <= 0.05233f)) // -3° ~ 3°
            {
                App_joint.recover_finish = true;
            }

        }
        else
        {
            App_joint.chassis_is_balance = false;
            App_joint.recover_finish = false;
        }
    }
}

// 倒地自救
static void joint_chassis_selfhelp(void)
{
    if (!App_joint.recover_finish)
    {
        leg_L.joint_F_torque = 0;
        leg_L.joint_B_torque = 0;
        leg_R.joint_F_torque = 0;
        leg_R.joint_B_torque = 0;
    }
}

/*********************** 任务 ***************************/
static void joint_disable_task() {

    leg_L.joint_F_torque = 0;
    leg_L.joint_B_torque = 0;
    leg_R.joint_F_torque = 0;
    leg_R.joint_B_torque = 0;

    App_joint.joint_ctrl_mode = JOINT_DISABLE;
    App_joint.jump_state = NOT_READY;

    leg_L.state_variable_feedback.x = 0.0f;
    leg_R.state_variable_feedback.x = 0.0f;

    App_joint.joint_ctrl_info.height_m = MIN_L0;


    // 关节初始化标志位
    App_joint.init_flag = false;

    // 平衡标志位
    App_joint.chassis_is_balance = false;
    // 倒地自救成功标志位
    App_joint.recover_finish = false;

    // 离地标志位
    leg_L.leg_is_offground = false;
    leg_R.leg_is_offground = false;
    App_joint.chassis_is_offground = false;

}

static void joint_init_task() {

    /** 关节电机使能 **/
    joint_enable();

    /** 关节电机初始化完毕 **/
    App_joint.init_flag = true;
}


static void joint_enable_task() {

    App_joint.jump_state = NOT_READY;

    if(switch_is_down(get_rc_ctrl()->rc.s[RC_s_L])){
        App_joint.joint_ctrl_info.height_m = 0.10f;
    }else if(switch_is_mid(get_rc_ctrl()->rc.s[RC_s_L])){
        App_joint.joint_ctrl_info.height_m = 0.22f;
    }else if(switch_is_up(get_rc_ctrl()->rc.s[RC_s_L])){
        App_joint.joint_ctrl_info.height_m = 0.35f;
    }

    joint_chassis_is_balanced();

    lqr_ctrl();

    joint_vmc_ctrl();

    joint_chassis_selfhelp();
//
//    chassis_vx_kalman_run();

//    chassis_is_offground();

}

void Joint_Task(void)
{
    Joint_Init();

    TickType_t last_wake_time = xTaskGetTickCount();

    while(1)
    {
        joint_get_IMU_info();

#if CHASSIS_REMOTE
        set_joint_mode();

        set_joint_ctrl_info();

        joint_device_offline_handle();
#else

#endif

        switch (App_joint.joint_ctrl_mode) {

            case JOINT_DISABLE:
                joint_disable_task();
                break;

            case JOINT_INIT:
                joint_init_task();
                break;

            case JOINT_ENABLE:
                joint_enable_task();
                break;

            default:break;
        }

        joint_motor_cmd_send();

        vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
    }
}
