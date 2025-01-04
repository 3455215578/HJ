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


/** 模块离线处理 **/
static void chassis_device_offline_handle() {
    check_is_rc_online(get_rc_ctrl());
    if (get_errors() != 0) {
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    }
}

/** 底盘接收遥控器信息 **/
static void set_chassis_ctrl_info() {
    chassis.chassis_ctrl_info.v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX;

    chassis.chassis_ctrl_info.x = (float) chassis.chassis_ctrl_info.x + chassis.chassis_ctrl_info.v_m_per_s * (CHASSIS_PERIOD * 0.001f);

    chassis.chassis_ctrl_info.yaw_angle_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_Z_CHANNEL]) * (-RC_TO_YAW_INCREMENT);

//  chassis.chassis_ctrl_info.pitch_angle_rad = (float) (get_rc_ctrl()->rc.ch[CHASSIS_PIT_CHANNEL]) * RC_TO_PITCH;

//  chassis.chassis_ctrl_info.height_m = chassis.chassis_ctrl_info.height_m + (float) (get_rc_ctrl()->rc.ch[3]) * RC_TO_L0;
//  VAL_LIMIT(chassis.chassis_ctrl_info.height_m, MIN_L0, MAX_L0)
}

/** 底盘根据遥控器设置模式 **/
static void set_chassis_mode() {
    if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) { // 失能
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == false) { // 初始化模式
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_INIT;
    } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == true) { // 使能
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_ENABLE;

        if(switch_is_down(get_rc_ctrl()->rc.s[RC_s_L]))
        {
            chassis.chassis_ctrl_info.height_m = 0.15f;
        }
        else if(switch_is_mid(get_rc_ctrl()->rc.s[RC_s_L]))
        {
            chassis.chassis_ctrl_info.height_m = 0.245f;
        }
        else if(switch_is_up(get_rc_ctrl()->rc.s[RC_s_L]))
        {
            chassis.chassis_ctrl_info.height_m = 0.35f;
        }
    }
    //else if ((chassis.chassis_ctrl_mode_last == CHASSIS_ENABLE) && (switch_is_down(get_rc_ctrl()->rc.s[RC_s_L]) && (switch_is_up(get_rc_ctrl()->rc.s[RC_s_R])))) { // 跳跃
//    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//    chassis.chassis_ctrl_mode = CHASSIS_JUMP;
//  }
}

/** 底盘通过板间通信接收云台的信息 **/
static void set_chassis_ctrl_info_from_gimbal_msg() {
    chassis.chassis_ctrl_info.v_m_per_s = get_gimbal_msg()->chassis_ctrl_info.v_m_per_s;
    chassis.chassis_ctrl_info.yaw_angle_rad = get_gimbal_msg()->chassis_ctrl_info.yaw_angle_rad;
    chassis.chassis_ctrl_info.roll_angle_rad = get_gimbal_msg()->chassis_ctrl_info.roll_angle_rad;
    chassis.chassis_ctrl_info.height_m = get_gimbal_msg()->chassis_ctrl_info.height_m;
}

/** 底盘根据云台信息设置模式 **/
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
 *                                    Init                                     *
 *******************************************************************************/

static void chassis_pid_init() {

  // 转向PID
  pid_init(&chassis.chassis_turn_pid,
             CHASSIS_TURN_PID_OUT_LIMIT,
             CHASSIS_TURN_PID_IOUT_LIMIT,
             CHASSIS_TURN_PID_P,
             CHASSIS_TURN_PID_I,
             CHASSIS_TURN_PID_D);

    // 腿长PID
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

    // 离地后的腿长PID
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

  // 防劈叉PID
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

void chassis_init() {
  osDelay(2000);

  chassis.leg_L.leg_index = L;
  chassis.leg_R.leg_index = R;

  /** 关节电机使能 **/
  joint_enable();

  osDelay(2);

  /** 轮毂电机使能 **/
  wheel_enable();

  /** 底盘PID初始化 **/
  chassis_pid_init();

  /********* 移动平均滤波器初始化 *************/

  /** 机体竖直方向加速度 **/
  moving_average_filter_init(&robot_az_filter);
  /** theta_ddot **/
  moving_average_filter_init(&theta_ddot_filter_L);
  moving_average_filter_init(&theta_ddot_filter_R);


//  chassis.chassis_ctrl_info.spin_speed = 5.0f;

//  vTaskSuspendAll();

  /** 轮毂-速度融合加速度 卡尔曼滤波器 初始化 **/
  chassis_kalman_init(&chassis.vx_kalman);
//  xTaskResumeAll();
}



/*******************************************************************************
 *                                 Function                                    *
 *******************************************************************************/

/************************ 向电机发送力矩 **********************/
static void chassis_motor_cmd_send() {

#if DEBUG_MODE // 通过宏定义预编译决定运行模式(DEBUG模式和正常模式)
  set_joint_torque(0, 0, 0, 0);
  osDelay(2);
  set_wheel_torque(0, 0);

#else

//  set_joint_torque(-chassis.leg_L.joint_F_torque,
//                   -chassis.leg_L.joint_B_torque,
//                   chassis.leg_R.joint_F_torque,
//                   chassis.leg_R.joint_B_torque);
//
//  osDelay(2);
//
//  set_wheel_torque(-chassis.leg_L.wheel_torque, -chassis.leg_R.wheel_torque);


 set_joint_torque(0, 0, 0, 0);
 osDelay(2);
 set_wheel_torque(0, 0);

#endif
}

/*********************** 获取传感器信息 *************************/
static void get_IMU_info() {

  /** Yaw **/
  chassis.imu_reference.yaw_angle = -*(get_ins_angle() + 0);

  // 圈数检测
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

  /** 更新各轴加速度和角速度 **/
  chassis.imu_reference.pitch_gyro = -*(get_ins_gyro() + 0);
  chassis.imu_reference.yaw_gyro = -*(get_ins_gyro() + 2);
  chassis.imu_reference.roll_gyro = -*(get_ins_gyro() + 1);

  chassis.imu_reference.ax = -*(get_ins_accel() + 1);
  chassis.imu_reference.ay = *(get_ins_accel() + 0);
  chassis.imu_reference.az = *(get_ins_accel() + 2);

  /** 去除重力影响的各轴加速度  旋转矩阵法 **/
  chassis.imu_reference.ax_filtered = chassis.imu_reference.ax - GRAVITY * sinf(chassis.imu_reference.pitch_angle);
  chassis.imu_reference.ay_filtered =  chassis.imu_reference.ay
                                     - GRAVITY * cosf(chassis.imu_reference.pitch_angle) * sinf(chassis.imu_reference.roll_angle);
  chassis.imu_reference.az_filtered =  chassis.imu_reference.az
                                     - GRAVITY * cosf(chassis.imu_reference.pitch_angle) * cosf(chassis.imu_reference.roll_angle);

  /** 机体竖直方向加速度 **/
  float robot_az_raw =  chassis.imu_reference.ax_filtered * sinf(chassis.imu_reference.pitch_angle)
                      + chassis.imu_reference.ay_filtered * sinf(-chassis.imu_reference.roll_angle) * cosf(chassis.imu_reference.pitch_angle)
                      + chassis.imu_reference.az_filtered * cosf(chassis.imu_reference.pitch_angle) * cosf(chassis.imu_reference.roll_angle);

  update_moving_average_filter(&robot_az_filter, robot_az_raw);
  chassis.imu_reference.robot_az = get_moving_average_filtered_value(&robot_az_filter);

}


/************************* 判断机体是否平衡 ****************************/
//static void is_chassis_ballanced() {
//
//    if((!chassis.is_chassis_offground) && (!chassis.jump_flag)) // 跳跃时不进行平衡判断
//    {
//        if (ABS(chassis.imu_reference.pitch_angle) <= 0.17444f) // -10° ~ 10°
//        {
//            chassis.is_chassis_balance = true;
//
//            if((ABS(chassis.imu_reference.pitch_angle) <= 0.05233f)) // -3° ~ 3°
//            {
//                chassis.recover_finish = true;
//            }
//
//        }
//        else
//        {
//            chassis.is_chassis_balance = false;
//            chassis.recover_finish = false;
//        }
//    }
//}



/*************************** 判断机体是否离地 *****************************/
//void is_chassis_off_ground()
//{
//
//    if((!chassis.recover_finish) || (chassis.jump_flag)) // 倒地自起未完成或跳跃时默认未离地
//    {
//        chassis.is_chassis_offground = false;
//    }
//    else
//    {
//        if ((chassis.leg_L.Fn < 12.5f) && (chassis.leg_R.Fn < 12.5f)) {
//            chassis.is_chassis_offground = true;
//        } else{
//            chassis.is_chassis_offground = false;
//        }
//    }
//
//}

/*************************** 返回底盘结构体指针 *****************************/
Chassis *get_chassis() {
  return &chassis;
}


/*************************************************************************/




/*******************************************************************************
 *                                    Task                                     *
 *******************************************************************************/


/** 失能 **/
static void chassis_disable_task() {
    wheel_disable();

    chassis.leg_L.wheel_torque = 0;
    chassis.leg_R.wheel_torque = 0;
    chassis.leg_L.joint_F_torque = 0;
    chassis.leg_L.joint_B_torque = 0;
    chassis.leg_R.joint_F_torque = 0;
    chassis.leg_R.joint_B_torque = 0;

    // 后面再抄达妙的
    chassis.leg_L.state_variable_feedback.x = 0;
    chassis.leg_R.state_variable_feedback.x = 0;

    chassis.chassis_ctrl_info.height_m = 0.14f;
    chassis.chassis_ctrl_info.yaw_angle_rad = chassis.imu_reference.yaw_total_angle;

    chassis.init_flag = false;
    chassis.is_joint_enable = false;
    chassis.is_wheel_enable = false;

    chassis.is_chassis_balance = false;
    chassis.recover_finish = false;

    chassis.is_chassis_offground = false;

    chassis.jump_state = NOT_READY;
    chassis.jump_flag = false;

}

/** 初始化 **/
static void chassis_init_task() {

  chassis_init();

  if(!chassis.is_joint_enable)
  {
        set_dm8009_enable(CAN_2, JOINT_LF_SEND);
        set_dm8009_enable(CAN_2, JOINT_LB_SEND);
        osDelay(1);
        set_dm8009_enable(CAN_2, JOINT_RF_SEND);
        set_dm8009_enable(CAN_2, JOINT_RB_SEND);

        chassis.is_joint_enable = true;
  }
  if(!chassis.is_wheel_enable)
  {
      lk9025_set_enable(CAN_1,WHEEL_L_SEND);
      osDelay(1);
      lk9025_set_enable(CAN_1,WHEEL_R_SEND);

      chassis.is_wheel_enable = true;
  }

  if(chassis.is_joint_enable && chassis.is_wheel_enable)
  {
      chassis.init_flag = true;
  }

}

//static void chassis_init_task() {
//
//    if (!chassis.is_joint_enable) {
//        set_dm8009_enable(CAN_2, JOINT_LF_SEND);
//        set_dm8009_enable(CAN_2, JOINT_LB_SEND);
//        HAL_Delay(2);
//        set_dm8009_enable(CAN_2, JOINT_RF_SEND);
//        set_dm8009_enable(CAN_2, JOINT_RB_SEND);
//        HAL_Delay(2);
//
//        chassis.is_joint_enable = true;
//    }
//
//    chassis.init_flag = true;
//
//}

/** 使能 **/
static void chassis_enable_task() {

//  chassis.jump_state = NOT_READY;

//  // 判断车体是否平衡
//  is_chassis_ballanced();
//
//  // 离地检测
//  is_chassis_off_ground();

//  if(chassis.recover_finish == false) // 倒地自起没完成时用最低腿长
//  {
//      chassis.chassis_ctrl_info.height_m = 0.14f;
//  }
//  else{//自起完之后再根据腿长档变化
//            if(switch_is_down(get_rc_ctrl()->rc.s[RC_s_L]))
//            {
//                chassis.chassis_ctrl_info.height_m = 0.14f;
//             }
//            else if(switch_is_mid(get_rc_ctrl()->rc.s[RC_s_L]))
//            {
//                chassis.chassis_ctrl_info.height_m = 0.245f;
//            }
//            else if(switch_is_up(get_rc_ctrl()->rc.s[RC_s_L]))
//            {
//                 chassis.chassis_ctrl_info.height_m = 0.35f;
//            }
//  }

//  lqr_ctrl();
//  vmc_ctrl();
//  chassis_vx_kalman_run();
//
//  fall_selfhelp();
}

/** 跳跃 **/
//static void chassis_jump_task() {
//
//    // 后面改成伸腿之后之后收腿  保证跳跃高度
//    if((chassis.jump_state == NOT_READY) && (jump_finish == 0) && (chassis.chassis_ctrl_info.height_m == 0.14f)) // 最低腿长时视为准备就绪
//    {
//        chassis.jump_state = READY;
//        chassis.jump_flag = true; // 进入跳跃状态
//
//    }
//    else if(chassis.jump_state == READY) // 伸腿蹬地
//    {
//        chassis.jump_state = STRETCHING;
//        chassis.chassis_ctrl_info.height_m = 0.40f;
//
//    }
//    else if((chassis.jump_state == STRETCHING) && (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 > 0.35f) && (chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 > 0.35f)) // 收腿腾空，保持姿态
//    {
//        chassis.jump_state = SHRINKING;
//        chassis.chassis_ctrl_info.height_m = 0.14f;
//
//    }
//    else if(chassis.jump_state == SHRINKING) // 伸腿缓冲
//    {
//        chassis.jump_state = STRETCHING_AGAIN;
//        chassis.chassis_ctrl_info.height_m = 0.20f;
//    }
//    else if((chassis.jump_state == STRETCHING_AGAIN) && (chassis.leg_L.Fn > 12.5f) && (chassis.leg_R.Fn > 12.5f)) // 落地
//    {
//        chassis.jump_state = LANDING;
//        chassis.chassis_ctrl_info.height_m = 0.14f;
//        chassis.jump_flag = false;
//
//    }
//
//    is_chassis_ballanced();
//    is_chassis_off_ground();
//
//    lqr_ctrl();
//    vmc_ctrl();
//    chassis_vx_kalman_run();
//
//    fall_selfhelp();
//}

/******************************** 总任务 *********************************/
extern void chassis_task(void const *pvParameters) {

  TickType_t last_wake_time = xTaskGetTickCount();

  while (1) {

    vTaskSuspendAll();

    /** 解算姿态信息 **/
    get_IMU_info();

    /** 遥控器控制 **/
#if CHASSIS_REMOTE
    set_chassis_mode();

    set_chassis_ctrl_info();

    /** 云台控制 **/
#else
    set_chassis_mode_from_gimbal_msg();
    set_chassis_ctrl_info_from_gimbal_msg();
#endif

    /** 模块离线处理 **/
    chassis_device_offline_handle();

    switch (chassis.chassis_ctrl_mode) {

      case CHASSIS_DISABLE: // 失能模式
          chassis_disable_task();
        break;

      case CHASSIS_INIT: // 初始化模式
          chassis_init_task();
        break;

      case CHASSIS_ENABLE:// 使能模式
          chassis_enable_task();
        break;



//      case CHASSIS_JUMP: // 跳跃模式
//            chassis_jump_task();
//        break;

//      case CHASSIS_SPIN:chassis.chassis_ctrl_info.v_m_per_s = 0;
//        chassis.chassis_ctrl_info.yaw_angle_rad = 0;
//        chassis_enable_task();
//        break;

      default:break;
    }

    xTaskResumeAll();

    chassis_motor_cmd_send();

    vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
  }
}

/*********************************************************************************************/
