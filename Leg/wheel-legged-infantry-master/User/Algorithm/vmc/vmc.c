#include <stdbool.h>
#include <math.h>

#include "robot_def.h"
#include "vmc.h"
#include "user_lib.h"
#include "joint.h"
#include "moving_filter.h"
#include "ins_task.h"

extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;

/*               正方向
 *    phi4                      phi4
 *
 *    phi1                      phi1
 */

/** 计算状态变量theta **/
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

// vmc正运动学解算
void vmc_forward_kinematics(Leg* leg, ChassisPhysicalConfig *physical_config) {
    leg->vmc.forward_kinematics.fk_point_coordinates.b_x = physical_config->l1 * cosf(leg->vmc.forward_kinematics.fk_phi.phi1);
    leg->vmc.forward_kinematics.fk_point_coordinates.b_y = physical_config->l1 * sinf(leg->vmc.forward_kinematics.fk_phi.phi1);
    leg->vmc.forward_kinematics.fk_point_coordinates.d_x = physical_config->l5 + physical_config->l4 * cosf(leg->vmc.forward_kinematics.fk_phi.phi4);
    leg->vmc.forward_kinematics.fk_point_coordinates.d_y = physical_config->l4 * sinf(leg->vmc.forward_kinematics.fk_phi.phi4); // l5 改成了 l4

    float A0 = 2.0f * physical_config->l2 * (leg->vmc.forward_kinematics.fk_point_coordinates.d_x - leg->vmc.forward_kinematics.fk_point_coordinates.b_x);
    float B0 = 2.0f * physical_config->l2 * (leg->vmc.forward_kinematics.fk_point_coordinates.d_y - leg->vmc.forward_kinematics.fk_point_coordinates.b_y);
    float BD_sq =  (leg->vmc.forward_kinematics.fk_point_coordinates.d_x - leg->vmc.forward_kinematics.fk_point_coordinates.b_x) * (leg->vmc.forward_kinematics.fk_point_coordinates.d_x - leg->vmc.forward_kinematics.fk_point_coordinates.b_x)
                     + (leg->vmc.forward_kinematics.fk_point_coordinates.d_y - leg->vmc.forward_kinematics.fk_point_coordinates.b_y) * (leg->vmc.forward_kinematics.fk_point_coordinates.d_y - leg->vmc.forward_kinematics.fk_point_coordinates.b_y);
    float C0 = physical_config->l2 * physical_config->l2 + BD_sq - physical_config->l3 * physical_config->l3;

    float temp = A0 * A0 + B0 * B0 - C0 * C0;
    float y = B0 + sqrtf(ABS(temp));
    float x = A0 + C0;
    leg->vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

    leg->vmc.forward_kinematics.fk_point_coordinates.c_x = physical_config->l1 * cosf(leg->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * cosf(leg->vmc.forward_kinematics.fk_phi.phi2);
    leg->vmc.forward_kinematics.fk_point_coordinates.c_y = physical_config->l1 * sinf(leg->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * sinf(leg->vmc.forward_kinematics.fk_phi.phi2);
    y = leg->vmc.forward_kinematics.fk_point_coordinates.c_y - leg->vmc.forward_kinematics.fk_point_coordinates.d_y;
    x = leg->vmc.forward_kinematics.fk_point_coordinates.c_x - leg->vmc.forward_kinematics.fk_point_coordinates.d_x;
    leg->vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

    temp =  (leg->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f) * (leg->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
            + leg->vmc.forward_kinematics.fk_point_coordinates.c_y * leg->vmc.forward_kinematics.fk_point_coordinates.c_y;
    leg->vmc.forward_kinematics.fk_L0.L0_last = leg->vmc.forward_kinematics.fk_L0.L0;
    leg->vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));
    leg->vmc.forward_kinematics.fk_L0.L0_dot_last = leg->vmc.forward_kinematics.fk_L0.L0_dot;
    leg->vmc.forward_kinematics.fk_L0.L0_dot =
            (leg->vmc.forward_kinematics.fk_L0.L0 - leg->vmc.forward_kinematics.fk_L0.L0_last)
            / (CHASSIS_PERIOD * 0.001f);
    leg->vmc.forward_kinematics.fk_L0.L0_ddot =
            (leg->vmc.forward_kinematics.fk_L0.L0_dot - leg->vmc.forward_kinematics.fk_L0.L0_dot_last)
            / (CHASSIS_PERIOD * 0.001f);

    leg->vmc.forward_kinematics.fk_phi.last_phi0 = leg->vmc.forward_kinematics.fk_phi.phi0;
    y = leg->vmc.forward_kinematics.fk_point_coordinates.c_y;
    x = leg->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f;
    leg->vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);
    leg->vmc.forward_kinematics.fk_phi.last_d_phi0 = leg->vmc.forward_kinematics.fk_phi.d_phi0;
    leg->vmc.forward_kinematics.fk_phi.d_phi0 =  (leg->vmc.forward_kinematics.fk_phi.phi0 - leg->vmc.forward_kinematics.fk_phi.last_phi0)
                                                 / (CHASSIS_PERIOD * 0.001f);
    leg->vmc.forward_kinematics.fk_phi.dd_phi0 =  (leg->vmc.forward_kinematics.fk_phi.d_phi0 - leg->vmc.forward_kinematics.fk_phi.last_d_phi0)
                                                  / (CHASSIS_PERIOD * 0.001f);

    leg->vmc.forward_kinematics.d_alpha = 0.0f - leg->vmc.forward_kinematics.fk_phi.d_phi0;

    // theta theta_dot
    leg->state_variable_feedback.theta_last = leg->state_variable_feedback.theta;
    leg->state_variable_feedback.theta = cal_leg_theta(leg->vmc.forward_kinematics.fk_phi.phi0, leg->state_variable_feedback.phi);
    leg->state_variable_feedback.theta_dot = (leg->state_variable_feedback.theta - leg->state_variable_feedback.theta_last) / (CHASSIS_PERIOD * 0.001f);

    // theta_ddot
    leg->state_variable_feedback.theta_dot_last = leg->state_variable_feedback.theta_dot;
    leg->state_variable_feedback.theta_ddot = (leg->state_variable_feedback.theta_dot - leg->state_variable_feedback.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);

}

// vmc正动力学解算
void vmc_forward_dynamics(VMC *vmc, const ChassisPhysicalConfig *physical_config) {
    if (vmc == NULL)
    {
        return;
    }

    vmc->forward_kinematics.J_F_to_T.E.x1_1 =
            physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
            / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->forward_kinematics.J_F_to_T.E.x1_2 =
            physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
            / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

    vmc->forward_kinematics.J_F_to_T.E.x2_1 =
            physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
            / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->forward_kinematics.J_F_to_T.E.x2_2 = physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                                              / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

    Matrix_multiply(2, 2, vmc->forward_kinematics.J_F_to_T.array,
                    2, 1, vmc->forward_kinematics.Fxy_set_point.array,
                    vmc->forward_kinematics.T1_T4_set_point.array);
}

// 逆解算腿长变化速度、摆角变化速度
static void Inverse_Kinematics(VMC *vmc,
                               float w1,
                               float w4,
                               ChassisPhysicalConfig *chassis_physical_config) {
    if (vmc == NULL) {
        return;
    }
    vmc->inverse_kinematics.W_fdb.E.w1_fdb = w1;
    vmc->inverse_kinematics.W_fdb.E.w4_fdb = w4;

    vmc->inverse_kinematics.J_w_to_v.E.x1_1 =  -chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                                               / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);


    vmc->inverse_kinematics.J_w_to_v.E.x1_2 = -chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                                              / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);


    vmc->inverse_kinematics.J_w_to_v.E.x2_1 = -chassis_physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                                              / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));


    vmc->inverse_kinematics.J_w_to_v.E.x2_2 = -chassis_physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                                              / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));

    vmc->inverse_kinematics.V_fdb.E.last_d_L0_fdb = vmc->inverse_kinematics.V_fdb.E.d_L0_fdb;

    Matrix_multiply(2, 2, vmc->inverse_kinematics.J_w_to_v.array,
                    2, 1, vmc->inverse_kinematics.W_fdb.array,
                    vmc->inverse_kinematics.V_fdb.array);


    vmc->inverse_kinematics.V_fdb.E.dd_L0_fdb = (vmc->inverse_kinematics.V_fdb.E.d_L0_fdb - vmc->inverse_kinematics.V_fdb.E.last_d_L0_fdb) / (CHASSIS_PERIOD * 0.001f);
}

// 逆解算出虚拟力矩和沿腿方向支持力
static void Inverse_Dynamics(VMC *vmc,
                             float T1, // phi1
                             float T4, // phi4
                             ChassisPhysicalConfig *chassis_physical_config) {
    if (vmc == NULL) {
        return;
    }
    vmc->inverse_kinematics.T1_T4_fdb.E.T1_fdb = T1;
    vmc->inverse_kinematics.T1_T4_fdb.E.T4_fdb = T4;

    vmc->inverse_kinematics.J_T_to_F.E.x1_1 =
            vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
            / (chassis_physical_config->l1
               * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->inverse_kinematics.J_T_to_F.E.x1_2 =
            vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
            / (chassis_physical_config->l4
               * sinf(vmc->forward_kinematics.fk_phi.phi4 - vmc->forward_kinematics.fk_phi.phi3));

    vmc->inverse_kinematics.J_T_to_F.E.x2_1 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
                                              / (chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi1));

    vmc->inverse_kinematics.J_T_to_F.E.x2_2 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
                                              / (chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4));


    Matrix_multiply(2, 2, vmc->inverse_kinematics.J_T_to_F.array,
                    2, 1, vmc->inverse_kinematics.T1_T4_fdb.array,
                    vmc->inverse_kinematics.Fxy_fdb.array);
}

// 计算竖直方向支持力
static void fn_cal(Leg *leg, float body_az, ChassisPhysicalConfig *chassis_physical_config) {

    if (leg == NULL) {
        return;
    }

    // 用逆解算的数据计算
    float P = leg->vmc.inverse_kinematics.Fxy_fdb.E.Fy_fdb * cosf(leg->state_variable_feedback.theta)
              + leg->vmc.inverse_kinematics.Fxy_fdb.E.Tp_fdb * sinf(leg->state_variable_feedback.theta) / leg->vmc.forward_kinematics.fk_L0.L0;

    float wheel_az = body_az - leg->vmc.inverse_kinematics.V_fdb.E.dd_L0_fdb * cosf(leg->state_variable_feedback.theta)
                   + 2.0f * leg->vmc.inverse_kinematics.V_fdb.E.d_L0_fdb * leg->state_variable_feedback.theta_dot
                     * sinf(leg->state_variable_feedback.theta)
                   + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_ddot
                     * sinf(leg->state_variable_feedback.theta)
                   + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_dot
                     * leg->state_variable_feedback.theta_dot * cosf(leg->state_variable_feedback.theta);


        leg->Fn = P + chassis_physical_config->wheel_weight * (GRAVITY + wheel_az);

}

/*******************************************************************************
 *                                     VMC                                     *
 *******************************************************************************/
void left_vmc_ctrl(void) {

    // 更新phi1 phi4
    vmc_left_phi_update(&chassis.leg_L);

    // VMC 正运动学解算
    forward_kinematics(&chassis.leg_L, &chassis_physical_config);

    // 配置输出力矩
    joint_motors_torque_set(&chassis, &chassis_physical_config);

    // VMC 逆动力学解算
    // 根据关节反馈力矩逆解算出虚拟力(F Tp)
    Inverse_Dynamics(&chassis.leg_L.vmc,
                     (get_joint_motors() + 1)->torque,
                     get_joint_motors()->torque,
                     &chassis_physical_config);
    Inverse_Dynamics(&chassis.leg_R.vmc,
                     -(get_joint_motors() + 2)->torque,
                     -(get_joint_motors() + 3)->torque,
                     &chassis_physical_config);


    Inverse_Kinematics(&chassis.leg_L.vmc,
                       (get_joint_motors() + 1)->angular_vel,
                       get_joint_motors()->angular_vel,
                       &chassis_physical_config);

    Inverse_Kinematics(&chassis.leg_R.vmc,
                       -(get_joint_motors() + 2)->angular_vel,
                       -(get_joint_motors() + 3)->angular_vel,
                       &chassis_physical_config);

    // 竖直方向支持力解算
    fn_cal(&chassis.leg_L, chassis.imu_reference.robot_az, &chassis_physical_config);
    fn_cal(&chassis.leg_R, chassis.imu_reference.robot_az, &chassis_physical_config);

//    // 腿部离地检测
//    leg_is_offground(&chassis);

}