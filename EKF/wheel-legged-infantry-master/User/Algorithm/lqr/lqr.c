#include <stddef.h>
#include <math.h>

#include "lqr.h"
#include "user_lib.h"
#include "moving_filter.h"

#include "robot_def.h"

extern Chassis chassis;
extern Joint App_joint;

extern float vel_acc[2]; // 轮毂速度与加速度融合后的结果

// 初始化K矩阵
float wheel_K_L[6] = {0, 0, 0, 0, 0, 0};
float joint_K_L[6] = {0, 0, 0, 0, 0, 0};

float wheel_K_R[6] = {0, 0, 0, 0, 0, 0};
float joint_K_R[6] = {0, 0, 0, 0, 0, 0};

// K拟合系数矩阵
float wheel_fitting_factor[6][4] = {
        {-168.903086f,213.041046f,-121.637415f,0.154455f},
        {-7.745521f,8.419162f,-9.308221f,-0.157014f},

        {-14.883342f,13.567137f,-3.908516f,-1.640752f},
        {-24.260909f,21.932164f,-6.444196f,-3.069910f},

        {-2633.317694f,2974.220590f,-1263.285121f,185.536117f},
        {-52.331141f,62.341544f,-29.463826f,6.959294f}
};float joint_fitting_factor[6][4] = {
        {282.972516f,-275.699159f,97.637613f,8.281031f},
        {11.122920f,-6.983759f,-4.105174f,1.172934f},

        {-52.874047f,58.770756f,-24.214638f,3.108526f},
        {-99.904442f,110.169355f,-45.111496f,5.756459f},

        {3821.070263f,-3612.908812f,1137.623899f,308.164187f},
        {102.357453f,-103.854191f,39.464075f,7.800856f}
};



void chassis_K_matrix_fitting(float L0, float K[6], const float KL[6][4]) {
  for (int i = 0; i < 6; i++) {
    K[i] = KL[i][0] * powf(L0, 3) + KL[i][1] * powf(L0, 2) + KL[i][2] * powf(L0, 1) + KL[i][3] * powf(L0, 0);
  }
}

// 计算状态变量theta
static float cal_leg_theta(float phi0, float phi) {
  float theta = 0, alpha = 0;//alpha is the Angle at which the virtual joint motor is turned
  alpha = PI / 2 - phi0;

  if (alpha * phi < 0) {
    theta = ABS(alpha) - ABS(phi);
    if ((alpha > 0) && (phi < 0)) {
      theta *= -1;
    } else {

    }
  } else {
    theta = ABS(alpha) + ABS(phi);
    if ((alpha < 0) && (phi < 0)) {
    } else {
      theta *= -1;
    }
  }
  return theta;
}

static void state_variable_update(Leg* leg_L, Leg* leg_R, float phi, float phi_dot) {
  if ((leg_L == NULL) || (leg_R == NULL)) {
    return;
  }
  //theta_last
  leg_L->state_variable_feedback.theta_last = leg_L->state_variable_feedback.theta;
  leg_R->state_variable_feedback.theta_last = leg_R->state_variable_feedback.theta;

  //1.theta
  leg_L->state_variable_feedback.theta = cal_leg_theta(leg_L->vmc.forward_kinematics.fk_phi.phi0, phi);
  leg_R->state_variable_feedback.theta = cal_leg_theta(leg_R->vmc.forward_kinematics.fk_phi.phi0, phi);

  //2.theta_dot、theta_ddot
  leg_L->state_variable_feedback.theta_dot_last = leg_L->state_variable_feedback.theta_dot;
  leg_L->state_variable_feedback.theta_dot = (leg_L->state_variable_feedback.theta - leg_L->state_variable_feedback.theta_last) / (CHASSIS_PERIOD * 0.001f);
  float theta_ddot_raw_L = (leg_L->state_variable_feedback.theta_dot - leg_L->state_variable_feedback.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);
  update_moving_average_filter(&leg_L->theta_ddot_filter, theta_ddot_raw_L);
  leg_L->state_variable_feedback.theta_ddot = get_moving_average_filtered_value(&leg_L->theta_ddot_filter);

  leg_R->state_variable_feedback.theta_dot_last = leg_R->state_variable_feedback.theta_dot;
  leg_R->state_variable_feedback.theta_dot = (leg_R->state_variable_feedback.theta - leg_R->state_variable_feedback.theta_last) / (CHASSIS_PERIOD * 0.001f);
  float theta_ddot_raw_R = (leg_R->state_variable_feedback.theta_dot - leg_R->state_variable_feedback.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);
  update_moving_average_filter(&leg_R->theta_ddot_filter, theta_ddot_raw_R);
  leg_R->state_variable_feedback.theta_ddot = get_moving_average_filtered_value(&leg_R->theta_ddot_filter);

  //4.x_dot
  leg_L->state_variable_feedback.x_dot = vel_acc[0];
  leg_R->state_variable_feedback.x_dot = vel_acc[0];

  //3.x

  if(chassis.chassis_ctrl_info.v_m_per_s != 0.0f)
  {
      leg_L->state_variable_feedback.x = 0.0f;
      leg_R->state_variable_feedback.x = 0.0f;
  }
  else
  {
      leg_L->state_variable_feedback.x = leg_L->state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * leg_L->state_variable_feedback.x_dot;
      leg_R->state_variable_feedback.x = leg_R->state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * leg_R->state_variable_feedback.x_dot;

  }

  // x_ddot
  leg_L->state_variable_feedback.x_dot_last = leg_L->state_variable_feedback.x_dot;
  leg_L->state_variable_feedback.x_ddot = (leg_L->state_variable_feedback.x_dot - leg_L->state_variable_feedback.x_dot_last) / (CHASSIS_PERIOD * 0.001f);
  leg_R->state_variable_feedback.x_dot_last = leg_R->state_variable_feedback.x_dot;
  leg_R->state_variable_feedback.x_ddot = (leg_R->state_variable_feedback.x_dot - leg_R->state_variable_feedback.x_dot_last) / (CHASSIS_PERIOD * 0.001f);

  //5.phi
  leg_L->state_variable_feedback.phi = phi;
  leg_R->state_variable_feedback.phi = phi;

  // 6.phi_dot
  leg_L->state_variable_feedback.phi_dot = phi_dot;
  leg_R->state_variable_feedback.phi_dot = phi_dot;
}

static void state_variable_set(Leg* leg_L, Leg* leg_R) {

  leg_L->state_variable_set_point.x = 0.0f;
  leg_L->state_variable_set_point.x_dot = chassis.chassis_ctrl_info.v_m_per_s;
  leg_L->state_variable_set_point.theta = 0.0f;
  leg_L->state_variable_set_point.theta_dot = 0.0f;
  leg_L->state_variable_set_point.phi = 0.0f;
  leg_L->state_variable_set_point.phi_dot = 0.0f;

  leg_R->state_variable_set_point.x = 0.0f;
  leg_R->state_variable_set_point.x_dot = chassis.chassis_ctrl_info.v_m_per_s;
  leg_R->state_variable_set_point.theta = 0.0f;
  leg_R->state_variable_set_point.theta_dot = 0.0f;
  leg_R->state_variable_set_point.phi = 0.0f;
  leg_R->state_variable_set_point.phi_dot = 0.0f;

}

static void state_variable_error(Leg *leg_L, Leg *leg_R) {
  if (leg_L == NULL || leg_R == NULL) {
    return;
  }

  leg_L->state_variable_error.x = leg_L->state_variable_feedback.x - leg_L->state_variable_set_point.x;
  leg_L->state_variable_error.x_dot = leg_L->state_variable_feedback.x_dot - leg_L->state_variable_set_point.x_dot;
  leg_L->state_variable_error.theta = leg_L->state_variable_feedback.theta - leg_L->state_variable_set_point.theta;
  leg_L->state_variable_error.theta_dot = leg_L->state_variable_feedback.theta_dot - leg_L->state_variable_set_point.theta_dot;
  leg_L->state_variable_error.phi = leg_L->state_variable_feedback.phi - leg_L->state_variable_set_point.phi;
  leg_L->state_variable_error.phi_dot = leg_L->state_variable_feedback.phi_dot - leg_L->state_variable_set_point.phi_dot;


  leg_R->state_variable_error.x = leg_R->state_variable_feedback.x - leg_R->state_variable_set_point.x;
  leg_R->state_variable_error.x_dot = leg_R->state_variable_feedback.x_dot - leg_R->state_variable_set_point.x_dot;
  leg_R->state_variable_error.theta = leg_R->state_variable_feedback.theta - leg_R->state_variable_set_point.theta;
  leg_R->state_variable_error.theta_dot = leg_R->state_variable_feedback.theta_dot - leg_R->state_variable_set_point.theta_dot;
  leg_R->state_variable_error.phi = leg_R->state_variable_feedback.phi - leg_R->state_variable_set_point.phi;
  leg_R->state_variable_error.phi_dot = leg_R->state_variable_feedback.phi_dot - leg_R->state_variable_set_point.phi_dot;

}

static void state_variable_out(Leg* leg_L, Leg* leg_R) {

  leg_L->state_variable_wheel_out.theta = leg_L->state_variable_error.theta * wheel_K_L[0]; // 极性正确
  leg_L->state_variable_wheel_out.theta_dot = leg_L->state_variable_error.theta_dot * wheel_K_L[1]; // 极性正确
  leg_L->state_variable_wheel_out.x = leg_L->state_variable_error.x * wheel_K_L[2];
  leg_L->state_variable_wheel_out.x_dot = leg_L->state_variable_error.x_dot * wheel_K_L[3];
  leg_L->state_variable_wheel_out.phi = leg_L->state_variable_error.phi * wheel_K_L[4]; // 极性正确
  leg_L->state_variable_wheel_out.phi_dot = leg_L->state_variable_error.phi_dot * wheel_K_L[5]; // 极性正确

  leg_R->state_variable_wheel_out.theta = leg_R->state_variable_error.theta * wheel_K_R[0]; // 极性正确
  leg_R->state_variable_wheel_out.theta_dot = leg_R->state_variable_error.theta_dot * wheel_K_R[1]; // 极性正确
  leg_R->state_variable_wheel_out.x = leg_R->state_variable_error.x * wheel_K_R[2];
  leg_R->state_variable_wheel_out.x_dot = leg_R->state_variable_error.x_dot * wheel_K_R[3];
  leg_R->state_variable_wheel_out.phi = leg_R->state_variable_error.phi * wheel_K_R[4]; // 极性正确
  leg_R->state_variable_wheel_out.phi_dot = leg_R->state_variable_error.phi_dot * wheel_K_R[5]; // 极性正确

  leg_L->state_variable_joint_out.theta = leg_L->state_variable_error.theta * joint_K_L[0];
  leg_L->state_variable_joint_out.theta_dot = leg_L->state_variable_error.theta_dot * joint_K_L[1];
  leg_L->state_variable_joint_out.x = leg_L->state_variable_error.x * joint_K_L[2];
  leg_L->state_variable_joint_out.x_dot = leg_L->state_variable_error.x_dot * joint_K_L[3];
  leg_L->state_variable_joint_out.phi = leg_L->state_variable_error.phi * joint_K_L[4];
  leg_L->state_variable_joint_out.phi_dot = leg_L->state_variable_error.phi_dot * joint_K_L[5];

  leg_R->state_variable_joint_out.theta = leg_R->state_variable_error.theta * joint_K_R[0];
  leg_R->state_variable_joint_out.theta_dot = leg_R->state_variable_error.theta_dot * joint_K_R[1];
  leg_R->state_variable_joint_out.x = leg_R->state_variable_error.x * joint_K_R[2];
  leg_R->state_variable_joint_out.x_dot = leg_R->state_variable_error.x_dot * joint_K_R[3];
  leg_R->state_variable_joint_out.phi = leg_R->state_variable_error.phi * joint_K_R[4];
  leg_R->state_variable_joint_out.phi_dot = leg_R->state_variable_error.phi_dot * joint_K_R[5];

}

/*******************************************************************************
 *                                     LQR                                     *
 *******************************************************************************/
void lqr_ctrl(void) {

  // K矩阵拟合
  chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

  // 更新反馈状态变量
  state_variable_update(&chassis.leg_L,
                        &chassis.leg_R,
                        chassis.imu_reference.pitch_angle,
                        chassis.imu_reference.pitch_gyro);

  // 设置期望状态变量
  state_variable_set(&chassis.leg_L, &chassis.leg_R);

  // 计算误差(反馈 - 期望)
  state_variable_error(&chassis.leg_L, &chassis.leg_R);

  // 计算每个状态变量项的输出(注意极性)
  state_variable_out(&chassis.leg_L, &chassis.leg_R);
}