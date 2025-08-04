#include "gimbal_task.h"
#include "launcher.h"
#include "protocol_balance.h"
#include "ins_task.h"
#include "packet.h"
#include "cmsis_os.h"
#include "board_communication_task.h"


vision_t vision_data;   // 给视觉传信息
extern robot_ctrl_info_t robot_ctrl;    // 获取视觉信息
//todo: 图传的
extern uint8_t control_flag;        // 通过状态判断是什么链路

/** 云台PID初始化 **/
static void Gimbal_Pid_Init(void)
{
    // Pitch
    pid_init(&gimbal.pitch.speed_pid,
             GIMBAL_PITCH_SPEED_MAX_OUT,
             GIMBAL_PITCH_SPEED_MAX_IOUT,
             GIMBAL_PITCH_SPEED_PID_KP,
             GIMBAL_PITCH_SPEED_PID_KI,
             GIMBAL_PITCH_SPEED_PID_KD);

    pid_init(&gimbal.pitch.angle_pid,
             GIMBAL_PITCH_ANGLE_MAX_OUT,
             GIMBAL_PITCH_ANGLE_MAX_IOUT,
             GIMBAL_PITCH_ANGLE_PID_KP,
             GIMBAL_PITCH_ANGLE_PID_KI,
             GIMBAL_PITCH_ANGLE_PID_KD);

    // Yaw
    pid_init(&gimbal.yaw.speed_pid,
             GIMBAL_YAW_SPEED_MAX_OUT,
             GIMBAL_YAW_SPEED_MAX_IOUT,
             GIMBAL_YAW_SPEED_PID_KP,
             GIMBAL_YAW_SPEED_PID_KI,
             GIMBAL_YAW_SPEED_PID_KD);

    pid_init(&gimbal.yaw.angle_pid,
             GIMBAL_YAW_ANGLE_MAX_OUT,
             GIMBAL_YAW_ANGLE_MAX_IOUT,
             GIMBAL_YAW_ANGLE_PID_KP,
             GIMBAL_YAW_ANGLE_PID_KI,
             GIMBAL_YAW_ANGLE_PID_KD);

}

/** 云台初始化 **/
static void Gimbal_Init(void) {

    gimbal.pitch.target_current = 0;
    gimbal.yaw.target_current = 0;

    /** 初始化云台模式 **/
    gimbal.gimbal_ctrl_mode = gimbal.gimbal_last_ctrl_mode = GIMBAL_DISABLE;

    /** 云台PID初始化 **/
    Gimbal_Pid_Init();

    /** 确定Yaw、Pitch上电复位的电机编码值 **/
    gimbal.yaw.motor_measure.offset_ecd = YAW_OFFSET_ECD;
    gimbal.pitch.motor_measure.offset_ecd = PITCH_OFFSET_ECD;

    /** 将PID状态变量误差置0（期望 = 误差），防止电机疯转 **/
    gimbal.yaw.absolute_angle_set = gimbal.pitch.absolute_angle_get;
    gimbal.pitch.absolute_angle_set = gimbal.pitch.absolute_angle_get;

    /** 低通滤波初始化 **/
    // 鼠标输入滤波
    first_order_filter_init(&gimbal.mouse_in_x, 1, 40);
    first_order_filter_init(&gimbal.mouse_in_y, 1, 10);

    // 角速度滤波
    first_order_filter_init(&gimbal.pitch_gyro_filter, 1, 20);
    first_order_filter_init(&gimbal.yaw_gyro_filter, 5, 30);

    // 对视觉传回的yaw和pitch滤波
    first_order_filter_init(&gimbal.auto_pitch, 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[0], 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[1], 1, 15);

}


/** 更新云台角度 **/
static void Gimbal_Angle_Update(void)
{
    /** Pitch **/
    // IMU角度
    gimbal.pitch.absolute_angle_get = INS_angle[2] * MOTOR_RAD_TO_ANGLE;
    // 与上电复位位置的角度差
    gimbal.pitch.relative_angle_get = Motor_Ecd_To_Angle_Change(gimbal.pitch.motor_measure.ecd,
                                                                gimbal.pitch.motor_measure.offset_ecd);
    // 角速度
    gimbal.pitch.gyro = -INS_gyro[0] * MOTOR_RAD_TO_ANGLE;

    /** Yaw **/
    // IMU角度
    gimbal.yaw.absolute_angle_get = INS_angle[0] * MOTOR_RAD_TO_ANGLE;
    // 与上电复位位置的角度差
    gimbal.yaw.relative_angle_get = Motor_Ecd_To_Angle_Change(gimbal.yaw.motor_measure.ecd,
                                                             gimbal.yaw.motor_measure.offset_ecd);
    // 角速度
    gimbal.yaw.gyro = INS_gyro[2] * MOTOR_RAD_TO_ANGLE;

}

// 因为这个任务是在1kHz的云台当中运行的，我希望它以500Hz运行
static void Gimbal_Send_Chassis_Data(void) {

    static int count = 1;

    if(count % 2 == 1) // 奇数发，偶数不发，实现500Hz
    {
        Send_Chassis_Data(rc_ctrl.rc.ch[CHASSIS_VX_CHANNEL],
                          rc_ctrl.rc.ch[CHASSIS_LEG_CHANNEL],
                          rc_ctrl.rc.s[RC_s_R],
                          gimbal.init_flag,
                          gimbal.yaw.relative_angle_get);
    }

    count ++;

}

/** 云台控制器计算 **/
static void Gimbal_Controller_Calc(void)
{
    float yaw_gyro_set;
    float pitch_gyro_set;

    /** Yaw **/
    yaw_gyro_set = pid_loop_calc(&gimbal.yaw.angle_pid,
                            gimbal.yaw.absolute_angle_get,
                            gimbal.yaw.absolute_angle_set,
                            180,
                            -180);

    // 期望角速度滤波
    first_order_filter_cali(&gimbal.yaw_gyro_filter, gimbal.yaw.gyro);

    gimbal.yaw.target_current = (int16_t)pid_calc(&gimbal.yaw.speed_pid,
                                                  gimbal.yaw_gyro_filter.out,//gimbal.yaw.motor_measure->speed_rpm,
                                                  yaw_gyro_set);

    /** Pitch **/
    pitch_gyro_set = pid_calc(&gimbal.pitch.angle_pid,
                        gimbal.pitch.absolute_angle_get,
                        gimbal.pitch.absolute_angle_set);

    // 期望角速度滤波
    first_order_filter_cali(&gimbal.pitch_gyro_filter,gimbal.pitch.gyro);

    gimbal.pitch.target_current = (int16_t)pid_calc(&gimbal.pitch.speed_pid,
                                                    gimbal.pitch_gyro_filter.out,
                                                    pitch_gyro_set);
}

/** Yaw电机上电复位 **/
static void Gimbal_Yaw_Reset(void)
{
    if(ABS(gimbal.yaw.relative_angle_get) > 1.0f)
    {
        float yaw_gyro_set;

        /** Yaw **/
        yaw_gyro_set = pid_calc(&gimbal.yaw.angle_pid,
                                gimbal.yaw.relative_angle_get,
                                0.0f);

        // 期望角速度滤波
        first_order_filter_cali(&gimbal.yaw_gyro_filter, gimbal.yaw.gyro);

        gimbal.yaw.target_current = (int16_t)pid_calc(&gimbal.yaw.speed_pid,
                                                      gimbal.yaw_gyro_filter.out,
                                                      yaw_gyro_set);
    }
    else
    {
        gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_get;

        gimbal.yaw.reset_finished = true;
    }
}

static void Gimbal_Pitch_Reset(void)
{

}

/*************************************************************************************************
 *                                           Task                                                *
 *************************************************************************************************/

static void Gimbal_Disable_Task(void)
{
    gimbal.pitch.target_current = 0;
    gimbal.yaw.target_current = 0;

    /** 避免PID误差过大，电机疯转 **/
    gimbal.pitch.absolute_angle_set = gimbal.pitch.absolute_angle_get;
    gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_get;

    // 云台电机上电复位标志位
    gimbal.yaw.reset_finished = false;
    gimbal.pitch.reset_finished = false;

    // 云台初始化标志位
    gimbal.init_flag = false;

}

/** 上电云台复位 **/
static void Gimbal_Init_Task(void)
{
    Gimbal_Yaw_Reset();

    if(gimbal.yaw.reset_finished)
    {
        gimbal.init_flag = true;
    }

//    if(gimbal.yaw.reset_finished && gimbal.pitch.reset_finished)
//    {
//        gimbal.init_flag = true;
//    }

}

static void Gimbal_Enable_Task(void)
{
    Gimbal_Controller_Calc();
}

void Gimbal_task(void const*pvParameters) {
    /* 任务初始化时间 */
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    /** 云台初始化 **/
    Gimbal_Init();

    while(1) {

        // 根据遥控器设置模式、控制信息
        Gimbal_Remote_Cmd();

        // 更新云台角度
        Gimbal_Angle_Update();

        // 板间通信
        Gimbal_Send_Chassis_Data();

        switch(gimbal.gimbal_ctrl_mode)
        {
            case GIMBAL_DISABLE:
            {
                Gimbal_Disable_Task();
                break;
            }

            case GIMBAL_INIT:
            {
                Gimbal_Init_Task();
                break;
            }

            case GIMBAL_ENABLE:
            {
                Gimbal_Enable_Task();
                break;
            }
        }

        DJI_Send_Motor_Mapping(CAN_1,
                               CAN_DJI_MOTOR_0x200_ID,
                               0,    //201 左摩擦轮
                               0,    //202 右摩擦轮
                               0,    //203 拨盘
                               0     // 204 无
        );

        DJI_Send_Motor_Mapping(CAN_1,
                               CAN_DJI_MOTOR_0x1FF_ID,
                               gimbal.yaw.target_current,     //205 yaw
                               0,     //206 pitch
                               0,     //207 无
                               0      //208 无
        );

        vTaskDelay(GIMBAL_PERIOD);
    }
}
