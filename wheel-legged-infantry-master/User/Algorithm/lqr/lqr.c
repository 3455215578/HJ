#include <stddef.h>
#include <stdbool.h>
#include <math.h>

#include "lqr.h"
#include "user_lib.h"
#include "moving_filter.h"
#include "vx_kalman_filter.h"

#include "robot_def.h"

extern Chassis chassis;

extern float vel_acc[2]; // ����ٶ�����ٶ��ںϺ�Ľ��

// ��ʼ��K����
float wheel_K_L[6] = {0, 0, 0, 0, 0, 0};
float joint_K_L[6] = {0, 0, 0, 0, 0, 0};

float wheel_K_R[6] = {0, 0, 0, 0, 0, 0};
float joint_K_R[6] = {0, 0, 0, 0, 0, 0};

// K���ϵ������
float wheel_fitting_factor[6][4] = {
        {-249.086510,270.177299,-116.702579,-2.411292},
        {-7.691924,8.340473,-8.012965,-0.346925},

        {-11.258136,10.600664,-3.342542,-0.533792},
        {-21.085062,19.840095,-6.455910,-1.188925},

        {-667.632339,726.822616,-287.136864,45.839083},
        {-29.067231,33.867452,-14.895650,3.028022}
};float joint_fitting_factor[6][4] = {
        {15.825437,27.691505,-33.484621,12.916646},
        {-23.859491,27.671226,-12.109134,2.251502},

        {-32.041601,34.654385,-13.503680,2.057876},
        {-71.981809,76.862853,-29.496898,4.419616},

        {1278.343430,-1209.662796,384.994573,50.701281},
        {59.417907,-58.523980,19.965917,2.359251}
};



void chassis_K_matrix_fitting(float L0, float K[6], const float KL[6][4]) {
  for (int i = 0; i < 6; i++) {
    K[i] = KL[i][0] * powf(L0, 3) + KL[i][1] * powf(L0, 2) + KL[i][2] * powf(L0, 1) + KL[i][3] * powf(L0, 0);
  }
}

// ����״̬����theta
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

  chassis.theta_error = leg_L->state_variable_feedback.theta - leg_R->state_variable_feedback.theta;

  //theta_ddot
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
  leg_L->state_variable_feedback.x = leg_L->state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * leg_L->state_variable_feedback.x_dot;
  leg_R->state_variable_feedback.x = leg_R->state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * leg_R->state_variable_feedback.x_dot;


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

  leg_L->state_variable_set_point.x = chassis.chassis_ctrl_info.x;
  leg_L->state_variable_set_point.x_dot = chassis.chassis_ctrl_info.v_m_per_s;
  leg_L->state_variable_set_point.theta = 0.0f;
  leg_L->state_variable_set_point.theta_dot = 0.0f;
  leg_L->state_variable_set_point.phi = 0.0f;
  leg_L->state_variable_set_point.phi_dot = 0.0f;

  leg_R->state_variable_set_point.x = chassis.chassis_ctrl_info.x;
  leg_R->state_variable_set_point.x_dot = chassis.chassis_ctrl_info.v_m_per_s;
  leg_R->state_variable_set_point.theta = 0.0f;
  leg_R->state_variable_set_point.theta_dot = 0.0f;
  leg_R->state_variable_set_point.phi = 0.0f;
  leg_R->state_variable_set_point.phi_dot = 0.0f;

//  if(chassis->jump_flag)
//  {
//      leg_L->state_variable_set_point.x_dot = 1.0f;
//      leg_R->state_variable_set_point.x_dot = 1.0f;
//  }

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

//  leg_L->state_variable_wheel_out.theta = leg_L->state_variable_error.theta * wheel_K_L[0]; // ������ȷ
//  leg_L->state_variable_wheel_out.theta_dot = leg_L->state_variable_error.theta_dot * wheel_K_L[1]; // ������ȷ
  leg_L->state_variable_wheel_out.x = leg_L->state_variable_error.x * wheel_K_L[2];
  leg_L->state_variable_wheel_out.x_dot = leg_L->state_variable_error.x_dot * wheel_K_L[3];
  leg_L->state_variable_wheel_out.phi = leg_L->state_variable_error.phi * wheel_K_L[4]; // ������ȷ
  leg_L->state_variable_wheel_out.phi_dot = leg_L->state_variable_error.phi_dot * wheel_K_L[5]; // ������ȷ

//  leg_R->state_variable_wheel_out.theta = leg_R->state_variable_error.theta * wheel_K_R[0]; // ������ȷ
//  leg_R->state_variable_wheel_out.theta_dot = leg_R->state_variable_error.theta_dot * wheel_K_R[1]; // ������ȷ
  leg_R->state_variable_wheel_out.x = leg_R->state_variable_error.x * wheel_K_R[2];
  leg_R->state_variable_wheel_out.x_dot = leg_R->state_variable_error.x_dot * wheel_K_R[3];
  leg_R->state_variable_wheel_out.phi = leg_R->state_variable_error.phi * wheel_K_R[4]; // ������ȷ
  leg_R->state_variable_wheel_out.phi_dot = leg_R->state_variable_error.phi_dot * wheel_K_R[5]; // ������ȷ

  // -?  ����
//  leg_L->state_variable_joint_out.theta = leg_L->state_variable_error.theta * joint_K_L[0];
//  leg_L->state_variable_joint_out.theta_dot = leg_L->state_variable_error.theta_dot * joint_K_L[1];
  leg_L->state_variable_joint_out.x = leg_L->state_variable_error.x * joint_K_L[2];
  leg_L->state_variable_joint_out.x_dot = leg_L->state_variable_error.x_dot * joint_K_L[3];
  leg_L->state_variable_joint_out.phi = leg_L->state_variable_error.phi * joint_K_L[4];
  leg_L->state_variable_joint_out.phi_dot = leg_L->state_variable_error.phi_dot * joint_K_L[5];

//  leg_R->state_variable_joint_out.theta = leg_R->state_variable_error.theta * joint_K_R[0];
//  leg_R->state_variable_joint_out.theta_dot = leg_R->state_variable_error.theta_dot * joint_K_R[1];
  leg_R->state_variable_joint_out.x = leg_R->state_variable_error.x * joint_K_R[2];
  leg_R->state_variable_joint_out.x_dot = leg_R->state_variable_error.x_dot * joint_K_R[3];
  leg_R->state_variable_joint_out.phi = leg_R->state_variable_error.phi * joint_K_R[4];
  leg_R->state_variable_joint_out.phi_dot = leg_R->state_variable_error.phi_dot * joint_K_R[5];

}

/*******************************************************************************
 *                                     LQR                                     *
 *******************************************************************************/

void lqr_ctrl(void) {

  // K�������
  chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

  // ���·���״̬����
  state_variable_update(&chassis.leg_L,
                        &chassis.leg_R,
                        chassis.imu_reference.pitch_angle,
                        chassis.imu_reference.pitch_gyro);

  // ��������״̬����
  state_variable_set(&chassis.leg_L, &chassis.leg_R);

  // �������(���� - ����)
  state_variable_error(&chassis.leg_L, &chassis.leg_R);

  // ����ÿ��״̬����������(ע�⼫��)
  state_variable_out(&chassis.leg_L, &chassis.leg_R);
}