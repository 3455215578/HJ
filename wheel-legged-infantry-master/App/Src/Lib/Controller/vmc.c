#include "vmc.h"
#include "user_lib.h"
#include "math.h"
#include "chassis.h"
#include "joint.h"
#include "filter.h"

static float phi0_error;
extern float theta_error;


static float steer_compensatory_torque;
struct MovingAverageFilter Fn_filter_L, Fn_filter_R;

float LF_joint_pos, LB_joint_pos, RF_joint_pos, RB_joint_pos;


// 矩阵乘法
static void Matrix_multiply(int rows1, int cols1, float matrix1[rows1][cols1],
                            int rows2, int cols2, float matrix2[rows2][cols2],
                            float result[rows1][cols2]) {
  if (cols1 != rows2)
    return;

  // Perform matrix multiplication
  for (int i = 0; i < rows1; ++i) {
    for (int j = 0; j < cols2; ++j) {
      result[i][j] = 0;
      for (int k = 0; k < cols1; ++k) {
        result[i][j] += matrix1[i][k] * matrix2[k][j];
      }
    }
  }
}

/*
 *              生而无畏
 *    phi4                      phi4
 *
 *    phi1                      phi1
 *
 *
 * 将某一边的电机面朝自己，逆时针转动编码器值增大
 *
 *
 *
 *
 */


static void vmc_phi_update(struct Leg *leg_L, struct Leg *leg_R, const struct ChassisPhysicalConfig *physical_config) {


    RF_joint_pos = (get_joint_motors() + 3)->pos_r;
    RB_joint_pos = (get_joint_motors() + 2)->pos_r;
    LF_joint_pos = (get_joint_motors() + 0)->pos_r;
    LB_joint_pos = get_joint_motors()[1].pos_r;

    leg_L->vmc.forward_kinematics.fk_phi.phi1 = // LB
            PI - (-get_joint_motors()[1].pos_r - physical_config->mechanical_leg_limit_angle);

    leg_L->vmc.forward_kinematics.fk_phi.phi4 = // LF
            (get_joint_motors() + 0)->pos_r - physical_config->mechanical_leg_limit_angle;

    leg_R->vmc.forward_kinematics.fk_phi.phi1 = // RB
            PI - ((get_joint_motors() + 2)->pos_r - physical_config->mechanical_leg_limit_angle);

    leg_R->vmc.forward_kinematics.fk_phi.phi4 = // RF
            -(get_joint_motors() + 3)->pos_r - physical_config->mechanical_leg_limit_angle;

}

// vmc正运动学解算
static void forward_kinematics(struct Leg *leg_L,
                               struct Leg *leg_R,
                               const struct ChassisPhysicalConfig *physical_config) {
  /***LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L***/

  leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x = physical_config->l1 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi1);
  leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y = physical_config->l1 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi1);
  leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x = physical_config->l5 + physical_config->l4 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi4);
  leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y = physical_config->l4 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi4); // l5 改成了 l4

  float L_A0 = 2.0f * physical_config->l2 * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x);
  float L_B0 = 2.0f * physical_config->l2 * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y);
  float L_BD_sq =  (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x) * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x)
                 + (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y) * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y);
  float L_C0 = physical_config->l2 * physical_config->l2 + L_BD_sq - physical_config->l3 * physical_config->l3;

  float temp = L_A0 * L_A0 + L_B0 * L_B0 - L_C0 * L_C0;
  float y = L_B0 + sqrtf(ABS(temp));
  float x = L_A0 + L_C0;
  leg_L->vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

  leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x = physical_config->l1 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi2);
  leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y = physical_config->l1 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi2);
  y = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y;
  x = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x;
  leg_L->vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

  temp =  (leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f) * (leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
        + leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y * leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y;
  leg_L->vmc.forward_kinematics.fk_L0.L0_last = leg_L->vmc.forward_kinematics.fk_L0.L0;
  leg_L->vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));
  leg_L->vmc.forward_kinematics.fk_L0.L0_dot_last = leg_L->vmc.forward_kinematics.fk_L0.L0_dot;
  leg_L->vmc.forward_kinematics.fk_L0.L0_dot =
      (leg_L->vmc.forward_kinematics.fk_L0.L0 - leg_L->vmc.forward_kinematics.fk_L0.L0_last)
          / (CHASSIS_PERIOD * 0.001f);
  leg_L->vmc.forward_kinematics.fk_L0.L0_ddot =
      (leg_L->vmc.forward_kinematics.fk_L0.L0_dot - leg_L->vmc.forward_kinematics.fk_L0.L0_dot_last)
          / (CHASSIS_PERIOD * 0.001f);

  leg_L->vmc.forward_kinematics.fk_phi.last_phi0 = leg_L->vmc.forward_kinematics.fk_phi.phi0;
  y = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y;
  x = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f;
  leg_L->vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);
  leg_L->vmc.forward_kinematics.fk_phi.last_d_phi0 = leg_L->vmc.forward_kinematics.fk_phi.d_phi0;
  leg_L->vmc.forward_kinematics.fk_phi.d_phi0 =  (leg_L->vmc.forward_kinematics.fk_phi.phi0 - leg_L->vmc.forward_kinematics.fk_phi.last_phi0)
                                               / (CHASSIS_PERIOD * 0.001f);
  leg_L->vmc.forward_kinematics.fk_phi.dd_phi0 =  (leg_L->vmc.forward_kinematics.fk_phi.d_phi0 - leg_L->vmc.forward_kinematics.fk_phi.last_d_phi0)
                                                   / (CHASSIS_PERIOD * 0.001f);

  leg_L->vmc.forward_kinematics.d_alpha = 0.0f - leg_L->vmc.forward_kinematics.fk_phi.d_phi0;

  /***LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R***/

  leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x = physical_config->l1 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi1);
  leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y = physical_config->l1 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi1);
  leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x = physical_config->l5 + physical_config->l4 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi4);
  leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y = physical_config->l4 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi4);

  float R_A0 = 2.0f * physical_config->l2 * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x);
  float R_B0 = 2.0f * physical_config->l2 * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y);
  float R_BD_sq =  (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x) * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x)
                 + (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y) * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y);
  float R_C0 = physical_config->l2 * physical_config->l2 + R_BD_sq - physical_config->l3 * physical_config->l3;

  temp = R_A0 * R_A0 + R_B0 * R_B0 - R_C0 * R_C0;
  y = R_B0 + sqrtf(ABS(temp));
  x = R_A0 + R_C0;
  leg_R->vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

  leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x = physical_config->l1 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi2);
  leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y = physical_config->l1 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi2);
  y = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y;
  x = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x;
  leg_R->vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

  temp =  (leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f) * (leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
         + leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y * leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y;
  leg_R->vmc.forward_kinematics.fk_L0.L0_last = leg_R->vmc.forward_kinematics.fk_L0.L0;
  leg_R->vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));
  leg_R->vmc.forward_kinematics.fk_L0.L0_dot_last = leg_R->vmc.forward_kinematics.fk_L0.L0_dot;
  leg_R->vmc.forward_kinematics.fk_L0.L0_dot = (leg_R->vmc.forward_kinematics.fk_L0.L0 - leg_R->vmc.forward_kinematics.fk_L0.L0_last) / (CHASSIS_PERIOD * 0.001f);
  leg_R->vmc.forward_kinematics.fk_L0.L0_ddot = (leg_R->vmc.forward_kinematics.fk_L0.L0_dot - leg_R->vmc.forward_kinematics.fk_L0.L0_dot_last) / (CHASSIS_PERIOD * 0.001f);

  leg_R->vmc.forward_kinematics.fk_phi.last_phi0 = leg_R->vmc.forward_kinematics.fk_phi.phi0;
  y = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y;
  x = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f;


  leg_R->vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);
  leg_R->vmc.forward_kinematics.fk_phi.last_d_phi0 = leg_R->vmc.forward_kinematics.fk_phi.d_phi0;
  leg_R->vmc.forward_kinematics.fk_phi.d_phi0 =  (leg_R->vmc.forward_kinematics.fk_phi.phi0 - leg_R->vmc.forward_kinematics.fk_phi.last_phi0) / (CHASSIS_PERIOD * 0.001f);
  leg_R->vmc.forward_kinematics.fk_phi.dd_phi0 =  (leg_R->vmc.forward_kinematics.fk_phi.d_phi0 - leg_R->vmc.forward_kinematics.fk_phi.last_d_phi0) / (CHASSIS_PERIOD * 0.001f);

  leg_R->vmc.forward_kinematics.d_alpha = 0.0f - leg_R->vmc.forward_kinematics.fk_phi.d_phi0;

  phi0_error = leg_L->vmc.forward_kinematics.fk_phi.phi0 - leg_R->vmc.forward_kinematics.fk_phi.phi0;


}

// vmc正动力学解算
static void forward_dynamics(struct VMC *vmc, const struct ChassisPhysicalConfig *physical_config) {
  if (vmc == NULL)
  {
      return;
  }

  vmc->J_F_to_T.E.x1_1 =
      physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
      / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

  vmc->J_F_to_T.E.x1_2 =
      physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
      / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

  vmc->J_F_to_T.E.x2_1 =
      physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
      / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

  vmc->J_F_to_T.E.x2_2 = physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
          / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

  Matrix_multiply(2, 2, vmc->J_F_to_T.array, 2, 1, vmc->Fxy_set_point.array, vmc->T1_T4_set_point.array);
}

static void wheel_motors_torque_set(struct Chassis *chassis) {

  if (chassis == NULL) {
    return;
  }
  if (chassis->chassis_ctrl_mode != CHASSIS_SPIN) {
#if REMOTE

// 计算转向力矩
chassis->wheel_turn_torque =  CHASSIS_TURN_PID_P * (chassis->imu_reference.yaw_total_angle - chassis->chassis_ctrl_info.yaw_angle_rad)
                            + CHASSIS_TURN_PID_D * chassis->imu_reference.yaw_gyro;
#else
    float turn_speed = pid_calc(&chassis->chassis_vw_speed_pid, chassis->chassis_ctrl_info.yaw_angle_rad, 0);

    chassis->wheel_turn_torque = pid_calc(&chassis->chassis_spin_pid,
                                           chassis->imu_reference.yaw_gyro,
                                           turn_speed);

//    chassis->wheel_turn_torque = -pid_calc(&chassis->chassis_vw_current_pid,
//                                               chassis->chassis_ctrl_info.yaw_angle_rad,
//                                               0);

//    chassis->wheel_turn_torque = pid_loop_calc(&chassis->chassis_vw_current_pid,
//                                               chassis->imu_reference.yaw_angle,
//                                               chassis->chassis_ctrl_info.yaw_angle_rad,
//                                               PI,
//                                               -PI);
#endif
  } else {
//    chassis->wheel_turn_torque = pid_calc(&chassis->chassis_spin_pid,
//                                          chassis->imu_reference.yaw_gyro,
//                                          chassis->chassis_ctrl_info.spin_speed);
  }

  chassis->leg_L.wheel_torque = 0;
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.theta;//
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.theta_dot;// √
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.x;
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.x_dot;
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.phi;//
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.phi_dot;
  chassis->leg_L.wheel_torque += chassis->wheel_turn_torque;


  chassis->leg_R.wheel_torque = 0;
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta;
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta_dot; // √
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x;
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x_dot;
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi;
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi_dot;
  chassis->leg_R.wheel_torque -= chassis->wheel_turn_torque;
  chassis->leg_R.wheel_torque *= -1;

  if (chassis->is_chassis_offground == true) {
      chassis->leg_L.wheel_torque = 0;
      chassis->leg_R.wheel_torque = 0;
    }

  VAL_LIMIT(chassis->leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
  VAL_LIMIT(chassis->leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
}

static void joint_motors_torque_set(struct Chassis *chassis,
                                    const struct ChassisPhysicalConfig *chassis_physical_config) {

  //// Tp
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point = 0;
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point = 0;

  // 防劈叉力矩 无所谓theta或者phi0 调得够硬就行

//  steer_compensatory_torque = pid_calc(&chassis->chassis_leg_coordination_pid,
//                                       phi0_error,
//                                       0);

    steer_compensatory_torque = pid_calc(&chassis->chassis_leg_coordination_pid,
                                       theta_error,
                                       0);


    //Left
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta; // √
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta_dot; // √
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.x; // √
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.x_dot; // √
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.phi; // ! 应该没啥问题吧 它的输出加了负号更爆
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.phi_dot;
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point -= steer_compensatory_torque;

    //Right
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta; // √
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta_dot; // √
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.x; // √
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.x_dot; // √
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.phi; // ! 应该没啥问题吧 它的输出加了负号更爆
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.phi_dot;
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += steer_compensatory_torque;

    if (chassis->is_chassis_offground == true) {
        chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point = 0;
        chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point = 0;

        chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta; // √
        chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta_dot;
        chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point -= steer_compensatory_torque;

        chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta; // √
        chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta_dot;
        chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += steer_compensatory_torque;
    }


    // Fy

    chassis->leg_L.L0_set_point = chassis->chassis_ctrl_info.height_m;
    chassis->leg_R.L0_set_point = chassis->chassis_ctrl_info.height_m;

  VAL_LIMIT(chassis->leg_L.L0_set_point, MIN_L0, MAX_L0);
  VAL_LIMIT(chassis->leg_R.L0_set_point, MIN_L0, MAX_L0);

// 计算腿长pid输出
    pid_calc(&chassis->leg_L.leg_pos_pid,
             chassis->leg_L.vmc.forward_kinematics.fk_L0.L0,
             chassis->leg_L.L0_set_point);

    pid_calc(&chassis->leg_R.leg_pos_pid,
             chassis->leg_R.vmc.forward_kinematics.fk_L0.L0,
             chassis->leg_R.L0_set_point);


//  Roll pid
//pid_calc(&get_chassis()->chassis_roll_pid,
//         get_chassis()->imu_reference.roll_angle,
//         get_chassis()->chassis_ctrl_info.roll_angle_rad);

    pid_calc(&get_chassis()->chassis_roll_pid,
             get_chassis()->imu_reference.roll_angle,
             0.0f);


    if (chassis->is_chassis_offground == true) {
        chassis->leg_L.vmc.Fxy_set_point.E.Fy_set_point = chassis->leg_L.leg_pos_pid.out;
        chassis->leg_R.vmc.Fxy_set_point.E.Fy_set_point = chassis->leg_R.leg_pos_pid.out;
    }
    else{
//        chassis->leg_L.vmc.Fxy_set_point.E.Fy_set_point = chassis->leg_L.leg_pos_pid.out + chassis_physical_config->body_weight * GRAVITY_A * cosf(chassis->leg_L.state_variable_feedback.theta) + chassis->chassis_roll_pid.out;
//        chassis->leg_R.vmc.Fxy_set_point.E.Fy_set_point = chassis->leg_R.leg_pos_pid.out + chassis_physical_config->body_weight * GRAVITY_A * cosf(chassis->leg_R.state_variable_feedback.theta) - chassis->chassis_roll_pid.out;

        chassis->leg_L.vmc.Fxy_set_point.E.Fy_set_point = chassis->leg_L.leg_pos_pid.out + chassis_physical_config->body_weight * GRAVITY_A * cosf(chassis->leg_L.state_variable_feedback.theta) + chassis->chassis_roll_pid.out;
        chassis->leg_R.vmc.Fxy_set_point.E.Fy_set_point = chassis->leg_R.leg_pos_pid.out + chassis_physical_config->body_weight * GRAVITY_A * cosf(chassis->leg_R.state_variable_feedback.theta) - chassis->chassis_roll_pid.out;

    }

// 计算关节电机力矩
  forward_dynamics(&chassis->leg_L.vmc, chassis_physical_config);
  forward_dynamics(&chassis->leg_R.vmc, chassis_physical_config);

  chassis->leg_L.joint_F_torque = chassis->leg_L.vmc.T1_T4_set_point.E.T1_set_point;//F
  chassis->leg_L.joint_B_torque = chassis->leg_L.vmc.T1_T4_set_point.E.T4_set_point;//B

  chassis->leg_R.joint_F_torque = chassis->leg_R.vmc.T1_T4_set_point.E.T1_set_point;//F
  chassis->leg_R.joint_B_torque = chassis->leg_R.vmc.T1_T4_set_point.E.T4_set_point;//B

  // 输出限幅
  VAL_LIMIT(chassis->leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
  VAL_LIMIT(chassis->leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
  VAL_LIMIT(chassis->leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
  VAL_LIMIT(chassis->leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
}

// 逆解算腿长变化速度、摆角变化速度
static void Inverse_Kinematics(struct VMC *vmc,
                               float w1,
                               float w4,
                               const struct ChassisPhysicalConfig *chassis_physical_config) {
  if (vmc == NULL) {
    return;
  }
  vmc->W_fdb.E.w1_fdb = w1;
  vmc->W_fdb.E.w4_fdb = w4;

  vmc->J_w_to_v.E.x1_1 =  -chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                         / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);


  vmc->J_w_to_v.E.x1_2 = -chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                         / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);


  vmc->J_w_to_v.E.x2_1 = -chassis_physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                         / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));


  vmc->J_w_to_v.E.x2_2 = -chassis_physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                        / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));

  Matrix_multiply(2, 2, vmc->J_w_to_v.array, 2, 1, vmc->W_fdb.array, vmc->V_fdb.array);
}

// 逆解算出虚拟力矩和沿腿方向支持力
static void Inverse_Dynamics(struct VMC *vmc,
                             float T1, // phi1
                             float T4, // phi4
                             const struct ChassisPhysicalConfig *chassis_physical_config) {
  if (vmc == NULL) {
    return;
  }
  vmc->T1_T4_fdb.E.T1_fdb = T1;
  vmc->T1_T4_fdb.E.T4_fdb = T4;

    vmc->J_T_to_F.E.x1_1 =
            vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
            / (chassis_physical_config->l1
               * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->J_T_to_F.E.x1_2 =
            vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
            / (chassis_physical_config->l4
               * sinf(vmc->forward_kinematics.fk_phi.phi4 - vmc->forward_kinematics.fk_phi.phi3));

    vmc->J_T_to_F.E.x2_1 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
                           / (chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi1));

    vmc->J_T_to_F.E.x2_2 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
                           / (chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4));


  Matrix_multiply(2, 2, vmc->J_T_to_F.array, 2, 1, vmc->T1_T4_fdb.array, vmc->Fxy_fdb.array);
}

// 计算竖直方向支持力
static void fn_cal(struct Leg *leg, float az, const struct ChassisPhysicalConfig *chassis_physical_config) {
  if (leg == NULL) {
    return;
  }

  // 用逆解算的数据计算
  float P = leg->vmc.Fxy_fdb.E.Fy_fdb * cosf(leg->state_variable_feedback.theta) + leg->vmc.Fxy_fdb.E.Tp_fdb * sinf(leg->state_variable_feedback.theta) / leg->vmc.forward_kinematics.fk_L0.L0;

  // 用正解算的数据计算
//  float P = leg->vmc.Fxy_set_point.E.Fy_set_point * cosf(leg->state_variable_feedback.theta) + leg->vmc.Fxy_set_point.E.Tp_set_point * sinf(leg->state_variable_feedback.theta) / leg->vmc.forward_kinematics.fk_L0.L0;

  float leg_az;
  leg_az = az - leg->vmc.forward_kinematics.fk_L0.L0_ddot * cosf(leg->state_variable_feedback.theta)
      + 2.0f * leg->vmc.forward_kinematics.fk_L0.L0_dot * leg->state_variable_feedback.theta_dot
          * sinf(leg->state_variable_feedback.theta)
      + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_ddot
          * sinf(leg->state_variable_feedback.theta)
      + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_dot
          * leg->state_variable_feedback.theta_dot * cosf(leg->state_variable_feedback.theta);

  float Fn_raw = P + chassis_physical_config->wheel_weight * 9.8f + chassis_physical_config->wheel_weight * leg_az;

  if (leg->leg_index == L) {
    updateFilter(&Fn_filter_L, Fn_raw);
    leg->Fn = getFilteredValue(&Fn_filter_L);
  } else if (leg->leg_index == R) {
    updateFilter(&Fn_filter_R, Fn_raw);
    leg->Fn = getFilteredValue(&Fn_filter_R);
  }
}
/*******************************************************************************
 *                                     VMC                                     *
 *******************************************************************************/
void vmc_ctrl(struct Chassis *chassis, const struct ChassisPhysicalConfig *chassis_physical_config) {

  // 更新phi1 phi4
  vmc_phi_update(&chassis->leg_L, &chassis->leg_R, chassis_physical_config);

  //*VMC 正解算 *//
  forward_kinematics(&chassis->leg_L, &chassis->leg_R, chassis_physical_config);

  // 配置输出力矩
  wheel_motors_torque_set(chassis);
  joint_motors_torque_set(chassis, chassis_physical_config);

  //*VMC 逆解算 *//
  // 根据关节反馈力矩逆解算出虚拟力(F Tp)
  Inverse_Dynamics(&chassis->leg_L.vmc,
                   (get_joint_motors() + 1)->torque,
                   get_joint_motors()->torque,
                   chassis_physical_config);
  Inverse_Dynamics(&chassis->leg_R.vmc,
                   -(get_joint_motors() + 2)->torque,
                   -(get_joint_motors() + 3)->torque,
                   chassis_physical_config);

  fn_cal(&chassis->leg_L, chassis->imu_reference.robot_az, chassis_physical_config);
  fn_cal(&chassis->leg_R, chassis->imu_reference.robot_az, chassis_physical_config);
}