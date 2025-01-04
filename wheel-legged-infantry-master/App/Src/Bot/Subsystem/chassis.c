#include <stdbool.h>
#include <math.h>
#include "chassis.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "Atti.h"
#include "remote.h"
#include "user_lib.h"
#include "dm_8009.h"
#include "error.h"
#include "filter.h"
#include "joint.h"
#include "gimbal_communication.h"

#define DEBUG 0

const struct ChassisPhysicalConfig chassis_physical_config = {0.075f,
                                                              5.486f,
                                                              1.18f,
                                                              0.2618f,
                                                              0.15f,
                                                              0.27f,
                                                              0.27f,
                                                              0.15f,
                                                              0.15f};


/*******************************************************************************
 *                                Remote control                               *
 *******************************************************************************/
#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Z_CHANNEL 2
#define CHASSIS_PIT_CHANNEL 3
#define CHASSIS_ROLL_CHANNEL 0

#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)
#define RC_TO_PITCH ((MAX_PITCH-MIN_PITCH)/660)
#define RC_TO_ROLL ((MAX_ROLL-MIN_ROLL)/660)

/*******************************************************************************
 *                                PID parameters                               *
 *******************************************************************************/

#define CHASSIS_VW_SPEED_PID_P (-4.5)
#define CHASSIS_VW_SPEED_PID_I (-0.1)
#define CHASSIS_VW_SPEED_PID_D (20)
#define CHASSIS_VW_SPEED_PID_IOUT_LIMIT 0.1
#define CHASSIS_VW_SPEED_PID_OUT_LIMIT 3

#define CHASSIS_SPIN_PID_P (-5.5)
#define CHASSIS_SPIN_PID_I (-0.0)
#define CHASSIS_SPIN_PID_D (-5)
#define CHASSIS_SPIN_PID_IOUT_LIMIT 0.1
#define CHASSIS_SPIN_PID_OUT_LIMIT 10

//#define CHASSIS_OFFGROUND_LEG_LO_PID_P 250
//#define CHASSIS_OFFGROUND_LEG_L0_PID_I 0
//#define CHASSIS_OFFGROUND_LEG_L0_PID_D 00000
//#define CHASSIS_OFFGROUND_LEG_L0_PID_IOUT_LIMIT 3
//#define CHASSIS_OFFGROUND_LEG_L0_PID_OUT_LIMIT 10000


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static bool init_flag = false;
static bool is_joint_enable = false;
struct Chassis chassis;
struct MovingAverageFilter robot_az_filter;


/*******************************************************************************
 *                                    Init                                     *
 *******************************************************************************/
static void chassis_pid_init() {

  // 转向 PID
  pid_init(&chassis.chassis_turn_pid,
             CHASSIS_TURN_PID_OUT_LIMIT,
             CHASSIS_TURN_PID_IOUT_LIMIT,
             CHASSIS_TURN_PID_P,
             CHASSIS_TURN_PID_I,
             CHASSIS_TURN_PID_D);

  // 腿长 PID
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


  // Roll PID
  pid_init(&chassis.chassis_roll_pid,
           CHASSIS_ROLL_PID_OUT_LIMIT,
           CHASSIS_ROLL_PID_IOUT_LIMIT,
           CHASSIS_ROLL_PID_P,
           CHASSIS_ROLL_PID_I,
           CHASSIS_ROLL_PID_D);

// 防劈叉PID
  pid_init(&chassis.chassis_leg_coordination_pid,
           CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
           CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
           CHASSIS_LEG_COORDINATION_PID_P,
           CHASSIS_LEG_COORDINATION_PID_I,
           CHASSIS_LEG_COORDINATION_PID_D);
}

static void chassis_kalman_init(struct KalmanFilter *kalman_filter) {
  float dt = 0.005f;//单位s，后续传入的初始值都是ms为单位

  float R_init[4] = {0.0025f, 0,
                       0, 1.0f};//观测误差
  float Q_init[4] = {dt * dt, dt,
                       dt, 1.0f};//process noise

//    float R_init[4] = {0.01f, 0, // 速度的测量噪声大，测量的置信度就小了，测量结果就失真了
//                       0, 2.5f};//观测误差
//    float R_init[4] = {0.01f, 0, // 速度的测量噪声大，测量的置信度就小了，测量结果就失真了
//                       0, 0.001f};//观测误差
//    float Q_init[4] = {0.00002f, 0,
//                       0, 0.8f};//process noise

  // 观测矩阵
  float H_init[4] = {1, 0,
                     0, 1};
  float Xk_init[2] = {0, 0};

  // 状态转移矩阵
  float F_init[4] = {1, dt,
                     0, 1};

  kalman_filter->H_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->H_data, 0, sizeof(float));
  memcpy(kalman_filter->H_data, H_init, sizeof(H_init));

  kalman_filter->F_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->F_data, 0, sizeof(float));
  memcpy(kalman_filter->F_data, F_init, sizeof(F_init));

  kalman_filter->R_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->R_data, 0, sizeof(float));
  memcpy(kalman_filter->R_data, R_init, sizeof(R_init));

  kalman_filter->Q_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->Q_data, 0, sizeof(float));
  memcpy(kalman_filter->Q_data, Q_init, sizeof(Q_init));

  kalman_filter->Xk_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->Xk_data, 0, sizeof(float));
  memcpy(kalman_filter->Xk_data, Xk_init, sizeof(Xk_init));

  kalman_filter->sigma_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->sigma_data, 0, sizeof(float));
  memcpy(kalman_filter->sigma_data, Q_init, sizeof(Q_init));

  kalman_filter->sigma_minus_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->sigma_minus_data, 0, sizeof(float));
  memcpy(kalman_filter->sigma_minus_data, Q_init, sizeof(Q_init));

  kalman_Filter_Init(kalman_filter, 2, 0, 1, 2);
}

void chassis_init() {
  osDelay(2000);

  chassis.leg_L.leg_index = L;
  chassis.leg_R.leg_index = R;

  // 轮毂电机使能
  wheel_init();

  osDelay(2);

  // 关节电机使能
  joint_init();

  chassis_pid_init();

  // 底盘模式初始化为失能
  chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

  // 底盘状态位
  chassis.is_chassis_offground = false;

  //
  init_filter(&robot_az_filter);
  init_filter(&theta_ddot_filter_L);
  init_filter(&theta_ddot_filter_R);

//  chassis.chassis_ctrl_info.spin_speed = 5.0f;

  vTaskSuspendAll();

  // 轮毂-速度融合加速度 卡尔曼滤波器 初始化
  chassis_kalman_init(&chassis.leg_L.vx_kalman);
  chassis_kalman_init(&chassis.leg_R.vx_kalman);
  xTaskResumeAll();
}
/*******************************************************************************
 *                                Getter&Setter                                *
 *******************************************************************************/
static void get_IMU_info() {

    // 更新底盘姿态角
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


  chassis.imu_reference.pitch_angle = -*(get_ins_angle() + 2);
  chassis.imu_reference.roll_angle = -*(get_ins_angle() + 1);

  // 更新各轴加速度和角速度
  chassis.imu_reference.pitch_gyro = -*(get_ins_gyro() + 0);
  chassis.imu_reference.yaw_gyro = -*(get_ins_gyro() + 2);
  chassis.imu_reference.roll_gyro = -*(get_ins_gyro() + 1);

  chassis.imu_reference.ax = -*(get_ins_accel() + 1);
  chassis.imu_reference.ay = *(get_ins_accel() + 0);
  chassis.imu_reference.az = *(get_ins_accel() + 2);

  // 去除重力影响的各轴加速度  旋转矩阵
  chassis.imu_reference.ax_filtered = chassis.imu_reference.ax - GRAVITY_A * sinf(chassis.imu_reference.pitch_angle);
  chassis.imu_reference.ay_filtered =  chassis.imu_reference.ay
                                     - GRAVITY_A * cosf(chassis.imu_reference.pitch_angle) * sinf(chassis.imu_reference.roll_angle);
  chassis.imu_reference.az_filtered =  chassis.imu_reference.az
                                     - GRAVITY_A * cosf(chassis.imu_reference.pitch_angle) * cosf(chassis.imu_reference.roll_angle);

  // 机体竖直方向加速度
  float robot_az_raw =  chassis.imu_reference.ax_filtered * sinf(chassis.imu_reference.pitch_angle)
                      + chassis.imu_reference.ay_filtered * sinf(-chassis.imu_reference.roll_angle) * cosf(chassis.imu_reference.pitch_angle)
                      + chassis.imu_reference.az_filtered * cosf(chassis.imu_reference.pitch_angle) * cosf(chassis.imu_reference.roll_angle);

  updateFilter(&robot_az_filter, robot_az_raw);
  chassis.imu_reference.robot_az = getFilteredValue(&robot_az_filter);
}


static void is_chassis_ballanced() {
    if(!chassis.is_chassis_offground)
    {
        if (ABS(chassis.imu_reference.pitch_angle) <= 0.17444f) { // 20°
            chassis.is_chassis_balance = true;
            if((ABS(chassis.imu_reference.pitch_angle) <= 0.05233f)) // 3°
            {
                chassis.recover_finish = true;
            }
            else{

            }
        } else {
            chassis.is_chassis_balance = false;
            chassis.recover_finish = false;
        }
    }
//    if ((ABS(chassis.imu_reference.pitch_angle) <= 0.17444f)&&(!chassis.is_chassis_offground)) { // 20°
//        chassis.is_chassis_balance = true;
//        if((ABS(chassis.imu_reference.pitch_angle) <= 0.05233f)) // 3°
//        {
//            chassis.recover_finish = true;
//        }
//        else{
//
//        }
//    } else {
//        chassis.is_chassis_balance = false;
//        chassis.recover_finish = false;
//    }
}



static bool is_joints_reduced() {
  if (ABS(get_joint_motors()[0].pos_r) <= 0.174533 &&
      ABS(get_joint_motors()[1].pos_r) <= 0.174533 &&
      ABS(get_joint_motors()[2].pos_r) <= 0.174533 &&
      ABS(get_joint_motors()[3].pos_r) <= 0.174533) {
    return true;
  } else {
    return false;
  }
}

// 倒地自救
static void fall_selfhelp() {
    // 当车的俯仰角不在正常范围内才起作用
    // 运行逻辑：当车没离地且不平衡时，关闭关节输出，然后检查各个关节位置是否异常，若异常，则先执行关节复位，再进行轮毂输出
    if (!chassis.is_chassis_balance) {

        if ((!is_joints_reduced())){ // 如果关节电机位置异常并且没离地，则先进行关节复位，使关节回到正常位置

        chassis.leg_L.wheel_torque = 0;
        chassis.leg_R.wheel_torque = 0;
        chassis.leg_L.joint_F_torque = 0;
        chassis.leg_L.joint_B_torque = 0;
        chassis.leg_R.joint_F_torque = 0;
        chassis.leg_R.joint_B_torque = 0;

        set_dm8009_MIT(CAN_2, JOINT_LF_SEND, 0, 0, 20, 5, 0);
        set_dm8009_MIT(CAN_2, JOINT_LB_SEND, 0, 0, 20, 5, 0);
        HAL_Delay(2);
        set_dm8009_MIT(CAN_2, JOINT_RF_SEND, 0, 0, 20, 5, 0);
        set_dm8009_MIT(CAN_2, JOINT_RB_SEND, 0, 0, 20, 5, 0);

        }
        else if (!chassis.recover_finish) { // 先用轮毂使机体平衡，再开启关节
            chassis.leg_L.joint_F_torque = 0;
            chassis.leg_L.joint_B_torque = 0;
            chassis.leg_R.joint_F_torque = 0;
            chassis.leg_R.joint_B_torque = 0;
        }

    }

}

bool is_chassis_off_ground() {

    if(!chassis.recover_finish)
    {
        chassis.is_chassis_offground = false;
    }
    else{
        if ((chassis.leg_L.Fn < 12.5f) && (chassis.leg_R.Fn < 12.5f)) {
            chassis.is_chassis_offground = true;
            return true;
        } else{
            chassis.is_chassis_offground = false;
            return false;
        }
    }

}

//static void set_chassis_mode() {
//  if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) {
//    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
//    chassis.jump_state == NOT_READY;
//  } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && init_flag == false) {
//    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//    chassis.chassis_ctrl_mode = CHASSIS_INIT;
//  } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && init_flag == true) {
//    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
//    chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
//    chassis.jump_state = NOT_READY;
//  }
//}

static void set_chassis_mode() {
  if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) { // 失能
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    chassis.jump_state = NOT_READY;
  } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && init_flag == false) { // 初始化模式
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_INIT;
  } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && init_flag == true) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
    chassis.jump_state = NOT_READY;

//    if(switch_is_down(get_rc_ctrl()->rc.s[RC_s_L]))
//    {
//        chassis.chassis_ctrl_info.height_m = 0.14f;
//    }
//    else if(switch_is_mid(get_rc_ctrl()->rc.s[RC_s_L]))
//    {
//          chassis.chassis_ctrl_info.height_m = 0.245f;
//    }
//    else if(switch_is_up(get_rc_ctrl()->rc.s[RC_s_L]))
//      {
//          chassis.chassis_ctrl_info.height_m = 0.35f;
//      }
  }
}

static void set_chassis_mode_from_gimbal_msg() {
  if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_DISABLE) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
  } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_ENABLE && init_flag == false) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_INIT;
  } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_ENABLE && init_flag == true) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
  }
}

static void chassis_device_offline_handle() {
  check_is_rc_online(get_rc_ctrl());
  if (get_errors() != 0) {
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
  }
}

static void set_chassis_ctrl_info() {
  chassis.chassis_ctrl_info.v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX;

  chassis.chassis_ctrl_info.yaw_angle_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_Z_CHANNEL]) * (-RC_TO_YAW_INCREMENT);

//  chassis.chassis_ctrl_info.pitch_angle_rad = (float) (get_rc_ctrl()->rc.ch[CHASSIS_PIT_CHANNEL]) * RC_TO_PITCH;

//  chassis.chassis_ctrl_info.height_m = chassis.chassis_ctrl_info.height_m + (float) (get_rc_ctrl()->rc.ch[3]) * RC_TO_L0;
//  VAL_LIMIT(chassis.chassis_ctrl_info.height_m, MIN_L0, MAX_L0)
}

//static void set_chassis_ctrl_info_from_gimbal_msg() {
//  chassis.chassis_ctrl_info.v_m_per_s = get_gimbal_msg()->chassis_ctrl_info.v_m_per_s;
//  chassis.chassis_ctrl_info.yaw_angle_rad = get_gimbal_msg()->chassis_ctrl_info.yaw_angle_rad;
//  chassis.chassis_ctrl_info.roll_angle_rad = get_gimbal_msg()->chassis_ctrl_info.roll_angle_rad;
//  chassis.chassis_ctrl_info.height_m = get_gimbal_msg()->chassis_ctrl_info.height_m;
//}

static void chassis_motor_cmd_send() {
#if DEBUG
  set_joint_torque(0, 0, 0, 0);
  osDelay(2);
  set_wheel_torque(0, 0);

#else

//  set_joint_torque(-chassis.leg_L.joint_F_torque,
//                   -chassis.leg_L.joint_B_torque,
//                   chassis.leg_R.joint_F_torque,
//                   chassis.leg_R.joint_B_torque);

  set_joint_torque(0, 0, 0, 0);

//  set_wheel_torque(-chassis.leg_L.wheel_torque, -chassis.leg_R.wheel_torque);

  set_wheel_torque(0, 0);

#endif
}

static void chassis_vx_kalman_run() {
  kalman_Filter_Predict(&chassis.leg_L.vx_kalman);
  kalman_Filter_Predict(&chassis.leg_R.vx_kalman);

  // d_alpha项正负待判断
  float w_left = -chassis.leg_L.state_variable_feedback.phi_dot + chassis.leg_L.vmc.forward_kinematics.d_alpha + get_wheel_motors()[0].angular_vel;
  float w_right = -chassis.leg_R.state_variable_feedback.phi_dot + chassis.leg_R.vmc.forward_kinematics.d_alpha - get_wheel_motors()[1].angular_vel;

  float v_left = w_left * chassis_physical_config.wheel_radius + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_L.state_variable_feedback.theta_dot * cosf(chassis.leg_L.state_variable_feedback.theta) + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot * sinf(chassis.leg_L.state_variable_feedback.theta);
  float v_right = w_right * chassis_physical_config.wheel_radius + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_R.state_variable_feedback.theta_dot * cosf(chassis.leg_R.state_variable_feedback.theta) + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot * sinf(chassis.leg_R.state_variable_feedback.theta);

    chassis.leg_L.kalman_measure[0] = v_left;
    chassis.leg_L.kalman_measure[1] = chassis.imu_reference.ax_filtered;

    chassis.leg_R.kalman_measure[0] = v_right;
    chassis.leg_R.kalman_measure[1] = chassis.imu_reference.ax_filtered;

  chassis.leg_L.kalman_result = kalman_Filter_Update(&chassis.leg_L.vx_kalman, chassis.leg_L.kalman_measure);
  chassis.leg_R.kalman_result = kalman_Filter_Update(&chassis.leg_R.vx_kalman, chassis.leg_R.kalman_measure);
}

struct Chassis *get_chassis() {
  return &chassis;
}
/*******************************************************************************
 *                                    Task                                     *
 *******************************************************************************/
static void chassis_init_task() {

  if (!is_joint_enable) {
    set_dm8009_enable(CAN_2, JOINT_LF_SEND);
    set_dm8009_enable(CAN_2, JOINT_LB_SEND);
    HAL_Delay(2);
    set_dm8009_enable(CAN_2, JOINT_RF_SEND);
    set_dm8009_enable(CAN_2, JOINT_RB_SEND);
    HAL_Delay(2);
    is_joint_enable = true;
    return;
  }

  init_flag = true;

}

static void chassis_enable_task() {

  is_chassis_ballanced();

  // 离地检测
  is_chassis_off_ground();

  if(chassis.recover_finish == false) // 倒地自起没完成时用最低腿长
  {
      chassis.chassis_ctrl_info.height_m = 0.14f;
  }
  else{//自起完之后再根据腿长档变化
            if(switch_is_down(get_rc_ctrl()->rc.s[RC_s_L]))
            {
                chassis.chassis_ctrl_info.height_m = 0.14f;
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

  lqr_ctrl(&chassis);
  vmc_ctrl(&chassis, &chassis_physical_config);
  chassis_vx_kalman_run();

  fall_selfhelp();
}

static void chassis_disable_task() {
  chassis.leg_L.wheel_torque = 0;
  chassis.leg_R.wheel_torque = 0;
  chassis.leg_L.joint_F_torque = 0;
  chassis.leg_L.joint_B_torque = 0;
  chassis.leg_R.joint_F_torque = 0;
  chassis.leg_R.joint_B_torque = 0;

  chassis.leg_L.state_variable_feedback.x = 0;
  chassis.leg_R.state_variable_feedback.x = 0;

  chassis.chassis_ctrl_info.height_m = MIN_L0;

  chassis.chassis_ctrl_info.yaw_angle_rad = chassis.imu_reference.yaw_total_angle;

  init_flag = false;
  is_joint_enable = false;
  chassis.is_chassis_balance = false;
//  chassis.could_use_joint = false;
  chassis.recover_finish = false;
  chassis.is_chassis_offground = false;
  chassis.jump_flag = false;

  lk9025_set_enable(CAN_2, WHEEL_L_SEND);
  lk9025_set_enable(CAN_2, WHEEL_R_SEND);
}


extern void chassis_task(void const *pvParameters) {

  chassis_init();

  TickType_t last_wake_time = xTaskGetTickCount();
  while (1) {

    vTaskSuspendAll();

    get_IMU_info();

#if REMOTE
    set_chassis_mode();

    set_chassis_ctrl_info();
#else
    set_chassis_mode_from_gimbal_msg();
    set_chassis_ctrl_info_from_gimbal_msg();
#endif
    chassis_device_offline_handle();

    switch (chassis.chassis_ctrl_mode) {
      case CHASSIS_INIT:
//        xTaskResumeAll();
        chassis_init_task();
//        chassis_enable_task();
//        is_chassis_off_ground();
        break;

      case CHASSIS_ENABLE:chassis_enable_task();
        break;

      case CHASSIS_DISABLE:chassis_disable_task();
        break;

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
