#include <stddef.h>
#include <math.h>

#include "lqr.h"
#include "user_lib.h"
#include "moving_filter.h"

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
        {-245.693828f,256.855837f,-121.836308f,-8.726481f},
        {-6.106865f,4.348125f,-11.449125f,-0.532048f},

        {-47.135334f,42.005241f,-12.225610f,-4.488930f},
        {-11.342346f,7.266851f,-2.207412f,-5.875007f},

        {-3991.522014f,3879.698024f,-1298.078318f,157.961034f},
        {-93.662983f,96.393369f,-35.681087f,5.769284f}
};float joint_fitting_factor[6][4] = {
        {-527.269511f,523.672746f,-180.870982f,25.660620f},
        {-50.181267f,50.933633f,-18.694410f,2.582528f},

        {-289.909871f,281.206496f,-93.630122f,11.146175f},
        {-353.300372f,337.833766f,-110.255664f,12.752557f},

        {3474.542352f,-3105.884065f,908.952092f,295.241051f},
        {88.957720f,-83.440892f,26.826007f,6.030945f}
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

  leg_L->state_variable_wheel_out.theta = leg_L->state_variable_error.theta * wheel_K_L[0]; // ������ȷ
  leg_L->state_variable_wheel_out.theta_dot = leg_L->state_variable_error.theta_dot * wheel_K_L[1]; // ������ȷ
  leg_L->state_variable_wheel_out.x = leg_L->state_variable_error.x * wheel_K_L[2];
  leg_L->state_variable_wheel_out.x_dot = leg_L->state_variable_error.x_dot * wheel_K_L[3];
  leg_L->state_variable_wheel_out.phi = leg_L->state_variable_error.phi * wheel_K_L[4]; // ������ȷ
  leg_L->state_variable_wheel_out.phi_dot = leg_L->state_variable_error.phi_dot * wheel_K_L[5]; // ������ȷ

  leg_R->state_variable_wheel_out.theta = leg_R->state_variable_error.theta * wheel_K_R[0]; // ������ȷ
  leg_R->state_variable_wheel_out.theta_dot = leg_R->state_variable_error.theta_dot * wheel_K_R[1]; // ������ȷ
  leg_R->state_variable_wheel_out.x = leg_R->state_variable_error.x * wheel_K_R[2];
  leg_R->state_variable_wheel_out.x_dot = leg_R->state_variable_error.x_dot * wheel_K_R[3];
  leg_R->state_variable_wheel_out.phi = leg_R->state_variable_error.phi * wheel_K_R[4]; // ������ȷ
  leg_R->state_variable_wheel_out.phi_dot = leg_R->state_variable_error.phi_dot * wheel_K_R[5]; // ������ȷ

  // -?  ����
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