#include "rc_task.h"
#include "robot_def.h"
#include "remote.h"
#include "error.h"
#include "cmsis_os.h"

extern Chassis chassis;

#define RC_PERIOD 10 // 遥控器任务周期为10ms

/*******************************************************************************
 *                                    Remote                                   *
 *******************************************************************************/

/** 模块离线处理 **/
static void chassis_device_offline_handle() {
    check_is_rc_online(get_rc_ctrl());
    if (get_errors() != 0) {
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    }
}

/** 底盘接收遥控器信息 **/
static void set_chassis_ctrl_info() {
    chassis.chassis_ctrl_info.v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_SPEED_CHANNEL]) * RC_TO_VX;

    chassis.chassis_ctrl_info.yaw_angle_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_YAW_CHANNEL]) * (-RC_TO_YAW_INCREMENT);

}

/** 底盘根据遥控器设置模式 **/
static void set_chassis_mode() {
    if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) { // 失能
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    }
    else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == false) { // 初始化模式
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_INIT;
    }
    else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == true) { // 使能
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_ENABLE;

    }

}

/** 底盘通过板间通信接收云台的信息 **/
static void set_chassis_ctrl_info_from_gimbal_msg()
{

}

/** 底盘根据云台信息设置模式 **/
static void set_chassis_mode_from_gimbal_msg()
{

}

// 128
void RC_task(void const *pvParameters)
{
    while(1)
    {
        /** 设置遥控信息 **/
#if CHASSIS_REMOTE
        set_chassis_mode();

        set_chassis_ctrl_info();

        chassis_device_offline_handle();
#else
        set_chassis_mode_from_gimbal_msg();
        set_chassis_ctrl_info_from_gimbal_msg();
#endif

        osDelay(RC_PERIOD);
    }

}