#include "wheel_task.h"
#include "robot_def.h"
#include "wheel.h"
#include "cmsis_os.h"
#include "user_lib.h"

extern Chassis chassis;

/** 驱动轮pid初始化 **/
static void wheel_pid_init()
{
    /** 转向PID **/
    pid_init(&chassis.chassis_turn_pid,
             CHASSIS_TURN_PID_OUT_LIMIT,
             CHASSIS_TURN_PID_IOUT_LIMIT,
             CHASSIS_TURN_PID_P,
             CHASSIS_TURN_PID_I,
             CHASSIS_TURN_PID_D);
}

static void App_Wheel_Init(void)
{
    /** 初始化底盘模式 **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** 轮毂电机初始化 **/
    wheel_init();

    /** 驱动轮pid初始化 **/
    wheel_pid_init();
}

/** 向CAN1总线发送轮毂电机力矩 **/
static void wheel_motor_cmd_send() {

/** DEBUG_MODE: 置1时进入调试模式，关闭关节和轮毂输出 **/
#if DEBUG_MODE
    set_joint_torque(0, 0, 0, 0);
#else
    if(chassis.chassis_ctrl_mode != CHASSIS_DISABLE)
    {
        set_wheel_torque(-chassis.leg_L.wheel_torque, -chassis.leg_R.wheel_torque);
    }
    else
    {
        set_wheel_torque(0.0f, 0.0f);
    }
#endif

}



/*******************************************************************************
 *                                  Task                                       *
 *******************************************************************************/

/*********************** 轮毂初始化任务 ***************************/
static void wheel_init_task()
{
    wheel_enable();

    chassis.wheel_init_flag = true;

    if(chassis.joint_init_flag &&chassis.wheel_init_flag)
    {
        chassis.init_flag = true;
    }
}

/************************* 轮毂失能任务 ***************************/
static void wheel_disable_task() {

    chassis.leg_L.wheel_torque = 0;
    chassis.leg_R.wheel_torque = 0;

    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    chassis.leg_L.state_variable_feedback.x = 0.0f;
    chassis.leg_R.state_variable_feedback.x = 0.0f;

    chassis.chassis_ctrl_info.yaw_angle_rad = chassis.imu_reference.yaw_total_angle;

    /** 初始化标志位 **/

    // 底盘初始化标志位
    chassis.wheel_init_flag = false;

    // 平衡标志位
    chassis.chassis_is_balance = false;
    // 倒地自救成功标志位
    chassis.recover_finish = false;

}

static void wheel_enable_task(void)
{
    if (chassis.chassis_ctrl_mode != CHASSIS_SPIN)
    {
        // 计算转向力矩
        chassis.wheel_turn_torque =  CHASSIS_TURN_PID_P * (chassis.imu_reference.yaw_total_angle - chassis.chassis_ctrl_info.yaw_angle_rad)
                                     + CHASSIS_TURN_PID_D * chassis.imu_reference.yaw_gyro;

    }else
    {
//        chassis->wheel_turn_torque = pid_calc(&chassis->chassis_spin_pid,
//                                              chassis->imu_reference.yaw_gyro,
//                                              chassis->chassis_ctrl_info.spin_speed);
    }

    chassis.leg_L.wheel_torque = 0;
    chassis.leg_R.wheel_torque = 0;

    chassis.leg_L.wheel_torque += chassis.leg_L.state_variable_wheel_out.theta;
    chassis.leg_L.wheel_torque += chassis.leg_L.state_variable_wheel_out.theta_dot;
    chassis.leg_L.wheel_torque += chassis.leg_L.state_variable_wheel_out.x;
    chassis.leg_L.wheel_torque += chassis.leg_L.state_variable_wheel_out.x_dot;
    chassis.leg_L.wheel_torque += chassis.leg_L.state_variable_wheel_out.phi;//
    chassis.leg_L.wheel_torque += chassis.leg_L.state_variable_wheel_out.phi_dot;
    chassis.leg_L.wheel_torque += chassis.wheel_turn_torque;

    chassis.leg_R.wheel_torque += chassis.leg_R.state_variable_wheel_out.theta;
    chassis.leg_R.wheel_torque += chassis.leg_R.state_variable_wheel_out.theta_dot;
    chassis.leg_R.wheel_torque += chassis.leg_R.state_variable_wheel_out.x;
    chassis.leg_R.wheel_torque += chassis.leg_R.state_variable_wheel_out.x_dot;
    chassis.leg_R.wheel_torque += chassis.leg_R.state_variable_wheel_out.phi;
    chassis.leg_R.wheel_torque += chassis.leg_R.state_variable_wheel_out.phi_dot;
    chassis.leg_R.wheel_torque -= chassis.wheel_turn_torque;
    chassis.leg_R.wheel_torque *= -1;


    VAL_LIMIT(chassis.leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    VAL_LIMIT(chassis.leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
}

void wheel_task(void const *pvParameters)
{
    App_Wheel_Init();

//    TickType_t last_wake_time = xTaskGetTickCount();

    while(1)
    {
        switch (chassis.chassis_ctrl_mode) {

            case CHASSIS_DISABLE:
                wheel_disable_task();
                break;

            case CHASSIS_INIT:
                wheel_init_task();
                break;

            case CHASSIS_ENABLE:
                wheel_enable_task();
                break;

            default:break;
        }

        wheel_motor_cmd_send();

        osDelay(CHASSIS_PERIOD);

//        vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
    }

}

