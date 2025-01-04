#include <stddef.h>
#include "lqr.h"
#include "chassis.h"
#include "user_lib.h"
#include "math.h"
#include "remote.h"
#include "buzzer.h"
#include "filter.h"

float theta_error;

struct MovingAverageFilter theta_ddot_filter_L, theta_ddot_filter_R;

// 初始化K矩阵
float wheel_K_L[6] = {0, 0, 0, 0, 0, 0};
float joint_K_L[6] = {0, 0, 0, 0, 0, 0};

float wheel_K_R[6] = {0, 0, 0, 0, 0, 0};
float joint_K_R[6] = {0, 0, 0, 0, 0, 0};

// K拟合系数矩阵
float wheel_fitting_factor[6][4] = {
        {-169.170549,181.637228,-91.948396,-2.498374},
        {-7.557340,5.260738,-9.380363,-0.558334},

        {-18.355650,16.790592,-5.070429,-1.069684},
        {-42.985695,39.104083,-12.281315,-3.021568},

        {-1403.429691,1442.595783,-520.748777,71.339311},
        {-30.798225,33.957859,-13.768371,2.656491}
};float joint_fitting_factor[6][4] = {
        {-127.117385,156.908432,-69.802780,13.324063},
        {-36.697649,40.576464,-16.416814,2.664070},

        {-64.889867,66.531312,-23.874580,3.180600},
        {-181.758676,185.222796,-65.980631,8.717958},

        {1710.944945,-1570.814551,477.446559,86.946618},
        {42.493041,-40.524228,13.250061,2.107847}
};


/////////////////////////////////////////////////////////////////////////////////////////////////>>

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

void Matrix_multiply(int rows1, int cols1, float matrix1[rows1][cols1],
                     int rows2, int cols2, float matrix2[rows2][cols2],
                     float result[rows1][cols2]) {
  if (cols1 != rows2){
      return;
  }

  for (int i = 0; i < rows1; ++i) {
    for (int j = 0; j < cols2; ++j) {
      result[i][j] = 0;
      for (int k = 0; k < cols1; ++k) {
        result[i][j] += matrix1[i][k] * matrix2[k][j];
      }
    }
  }
}

static void state_variable_update(struct Leg *leg_L, struct Leg *leg_R, float phi, float phi_dot) {
  if (leg_L == NULL || leg_R == NULL) {
    return;
  }
  //theta_last
  leg_L->state_variable_feedback.theta_last = leg_L->state_variable_feedback.theta;
  leg_R->state_variable_feedback.theta_last = leg_R->state_variable_feedback.theta;

  //1.theta
  leg_L->state_variable_feedback.theta = cal_leg_theta(leg_L->vmc.forward_kinematics.fk_phi.phi0, phi);
  leg_R->state_variable_feedback.theta = cal_leg_theta(leg_R->vmc.forward_kinematics.fk_phi.phi0, phi);

  theta_error = leg_L->state_variable_feedback.theta - leg_R->state_variable_feedback.theta;

  //theta_ddot
  leg_L->state_variable_feedback.theta_dot_last = leg_L->state_variable_feedback.theta_dot;
  leg_L->state_variable_feedback.theta_dot = (leg_L->state_variable_feedback.theta - leg_L->state_variable_feedback.theta_last) / (CHASSIS_PERIOD * 0.001f);
  float theta_ddot_raw_L = (leg_L->state_variable_feedback.theta_dot - leg_L->state_variable_feedback.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);
  updateFilter(&theta_ddot_filter_L, theta_ddot_raw_L);
  leg_L->state_variable_feedback.theta_ddot = getFilteredValue(&theta_ddot_filter_L);

  leg_R->state_variable_feedback.theta_dot_last = leg_R->state_variable_feedback.theta_dot;
  leg_R->state_variable_feedback.theta_dot = (leg_R->state_variable_feedback.theta - leg_R->state_variable_feedback.theta_last) / (CHASSIS_PERIOD * 0.001f);
  float theta_ddot_raw_R = (leg_R->state_variable_feedback.theta_dot - leg_R->state_variable_feedback.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);
  updateFilter(&theta_ddot_filter_R, theta_ddot_raw_R);
  leg_R->state_variable_feedback.theta_ddot = getFilteredValue(&theta_ddot_filter_R);

  //4.x_dot
  leg_L->state_variable_feedback.x_dot = leg_L->kalman_result[0];
  leg_R->state_variable_feedback.x_dot = leg_R->kalman_result[0];

  //3.x
  if (get_chassis()->chassis_ctrl_info.v_m_per_s != 0 || get_chassis()->is_chassis_offground == true) { // 运动或离地时位移不介入
      leg_L->state_variable_feedback.x = 0;
      leg_R->state_variable_feedback.x = 0;
  } else {
      leg_L->state_variable_feedback.x = leg_L->state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * leg_L->state_variable_feedback.x_dot;
      leg_R->state_variable_feedback.x = leg_R->state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * leg_R->state_variable_feedback.x_dot;

      // 待修改 限幅小了会溜车 大了会冲一段距离
      VAL_LIMIT(leg_L->state_variable_feedback.x, -5, 5)
      VAL_LIMIT(leg_R->state_variable_feedback.x, -5, 5)
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

static void state_variable_set(struct Chassis *chassis) {
  if (chassis == NULL) {
    return;
  }

  chassis->leg_L.state_variable_set_point.x = 0.0f;
  chassis->leg_L.state_variable_set_point.x_dot = chassis->chassis_ctrl_info.v_m_per_s;
  chassis->leg_L.state_variable_set_point.theta = 0.0f;
  chassis->leg_L.state_variable_set_point.theta_dot = 0.0f;
  chassis->leg_L.state_variable_set_point.phi = 0.0f;
  chassis->leg_L.state_variable_set_point.phi_dot = 0.0f;

  chassis->leg_R.state_variable_set_point.x = 0.0f;
  chassis->leg_R.state_variable_set_point.x_dot = chassis->chassis_ctrl_info.v_m_per_s;
  chassis->leg_R.state_variable_set_point.theta = 0.0f;
  chassis->leg_R.state_variable_set_point.theta_dot = 0.0f;
  chassis->leg_R.state_variable_set_point.phi = 0.0f;
  chassis->leg_R.state_variable_set_point.phi_dot = 0.0f;
}

static void state_variable_error(struct Leg *leg_L, struct Leg *leg_R) {
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

static void state_variable_out(struct Chassis *chassis) {
  if (chassis == NULL) {
    return;
  }

  chassis->leg_L.state_variable_wheel_out.theta = chassis->leg_L.state_variable_error.theta * wheel_K_L[0]; // 极性正确
  chassis->leg_L.state_variable_wheel_out.theta_dot = chassis->leg_L.state_variable_error.theta_dot * wheel_K_L[1]; // 极性正确
  chassis->leg_L.state_variable_wheel_out.x = chassis->leg_L.state_variable_error.x * wheel_K_L[2];
  chassis->leg_L.state_variable_wheel_out.x_dot = chassis->leg_L.state_variable_error.x_dot * wheel_K_L[3];
  chassis->leg_L.state_variable_wheel_out.phi = chassis->leg_L.state_variable_error.phi * wheel_K_L[4]; // 极性正确
  chassis->leg_L.state_variable_wheel_out.phi_dot = chassis->leg_L.state_variable_error.phi_dot * wheel_K_L[5]; // 极性正确

  chassis->leg_R.state_variable_wheel_out.theta = chassis->leg_R.state_variable_error.theta * wheel_K_R[0]; // 极性正确
  chassis->leg_R.state_variable_wheel_out.theta_dot = chassis->leg_R.state_variable_error.theta_dot * wheel_K_R[1]; // 极性正确
  chassis->leg_R.state_variable_wheel_out.x = chassis->leg_R.state_variable_error.x * wheel_K_R[2];
  chassis->leg_R.state_variable_wheel_out.x_dot = chassis->leg_R.state_variable_error.x_dot * wheel_K_R[3];
  chassis->leg_R.state_variable_wheel_out.phi = chassis->leg_R.state_variable_error.phi * wheel_K_R[4]; // 极性正确
  chassis->leg_R.state_variable_wheel_out.phi_dot = chassis->leg_R.state_variable_error.phi_dot * wheel_K_R[5]; // 极性正确

  // -?  待测
  chassis->leg_L.state_variable_joint_out.theta = chassis->leg_L.state_variable_error.theta * joint_K_L[0]; // 极性正确
  chassis->leg_L.state_variable_joint_out.theta_dot = chassis->leg_L.state_variable_error.theta_dot * joint_K_L[1];
  chassis->leg_L.state_variable_joint_out.x = chassis->leg_L.state_variable_error.x * joint_K_L[2];
  chassis->leg_L.state_variable_joint_out.x_dot = chassis->leg_L.state_variable_error.x_dot * joint_K_L[3];
  chassis->leg_L.state_variable_joint_out.phi = chassis->leg_L.state_variable_error.phi * joint_K_L[4];
  chassis->leg_L.state_variable_joint_out.phi_dot = chassis->leg_L.state_variable_error.phi_dot * joint_K_L[5];

  chassis->leg_R.state_variable_joint_out.theta = chassis->leg_R.state_variable_error.theta * joint_K_R[0];
  chassis->leg_R.state_variable_joint_out.theta_dot = chassis->leg_R.state_variable_error.theta_dot * joint_K_R[1];
  chassis->leg_R.state_variable_joint_out.x = chassis->leg_R.state_variable_error.x * joint_K_R[2];
  chassis->leg_R.state_variable_joint_out.x_dot = chassis->leg_R.state_variable_error.x_dot * joint_K_R[3];
  chassis->leg_R.state_variable_joint_out.phi = chassis->leg_R.state_variable_error.phi * joint_K_R[4];
  chassis->leg_R.state_variable_joint_out.phi_dot = chassis->leg_R.state_variable_error.phi_dot * joint_K_R[5];

}

/*******************************************************************************
 *                                     LQR                                     *
 *******************************************************************************/

void lqr_ctrl(struct Chassis *chassis) {

  // K矩阵拟合
  chassis_K_matrix_fitting(chassis->leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis->leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
  chassis_K_matrix_fitting(chassis->leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis->leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

  // 更新反馈状态变量
  state_variable_update(&chassis->leg_L,
                        &chassis->leg_R,
                        chassis->imu_reference.pitch_angle,
                        chassis->imu_reference.pitch_gyro);

  // 设置期望状态变量
  state_variable_set(chassis);

  // 计算误差(反馈 - 期望)
  state_variable_error(&chassis->leg_L, &chassis->leg_R);

  // 计算每个状态变量项的输出(注意极性)
  state_variable_out(chassis);
}