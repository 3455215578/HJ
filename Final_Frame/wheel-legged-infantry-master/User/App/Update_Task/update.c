#include "update.h"
#include "robot_def.h"

#include "remote.h"
#include "error.h"
#include "ins.h"
#include "user_lib.h"

#define UPDATE_PERIOD 3 // 数据更新任务周期为3ms

extern Chassis chassis;
extern float vel_acc[2];

// 初始化K矩阵
float wheel_K_L[6] = {0, 0, 0, 0, 0, 0};
float joint_K_L[6] = {0, 0, 0, 0, 0, 0};

float wheel_K_R[6] = {0, 0, 0, 0, 0, 0};
float joint_K_R[6] = {0, 0, 0, 0, 0, 0};

// K拟合系数矩阵
float wheel_fitting_factor[6][4] = {
        {-115.015570f,150.811623f,-90.385399f,-0.683845f},
        {-2.961175f,3.934224f,-6.074763f,-0.252076f},

        {-3.197913f,2.994897f,-0.912153f,-0.770101f},
        {-4.585014f,4.245380f,-1.343306f,-1.832575f},

        {-1560.279813f,1753.178328f,-751.049746f,122.776898f},
        {-28.079548f,33.933430f,-17.010394f,5.064552f}
};float joint_fitting_factor[6][4] = {
        {258.977305f,-263.622791f,101.677280f,7.728395f},
        {4.739531f,-1.272110f,-4.793368f,1.251058f},

        {-27.388500f,30.412116f,-12.660277f,1.778826f},
        {-66.806639f,73.128794f,-29.955460f,4.144668f},

        {3129.744241f,-3030.664276f,1001.402362f,547.097076f},
        {25.711759f,-32.151577f,18.367651f,12.656690f}
};



/*******************************************************************************
 *                            获取底盘传感器信息                                   *
 *******************************************************************************/
static void get_IMU_info() {

    /** Yaw **/
    chassis.imu_reference.yaw_angle = -INS.Yaw * DEGREE_TO_RAD;
    chassis.imu_reference.yaw_total_angle = -INS.YawTotalAngle * DEGREE_TO_RAD;

    /** Pitch **/
    chassis.imu_reference.pitch_angle = -INS.Roll * DEGREE_TO_RAD;

    /** Roll **/
    chassis.imu_reference.roll_angle = INS.Pitch * DEGREE_TO_RAD;

    /** 更新各轴加速度和角速度 **/
    chassis.imu_reference.pitch_gyro = -INS.Gyro[Y];
    chassis.imu_reference.yaw_gyro = -INS.Gyro[Z];
    chassis.imu_reference.roll_gyro = INS.Gyro[X];

    chassis.imu_reference.ax = INS.Accel[X];
    chassis.imu_reference.ay = INS.Accel[Y];
    chassis.imu_reference.az = INS.Accel[Z];

    /** 机体竖直方向加速度 **/
    chassis.imu_reference.robot_az = INS.MotionAccel_n[Z];

}

/*******************************************************************************
 *                                  LQR                                        *
 *******************************************************************************/

/** 拟合K **/
void chassis_K_matrix_fitting(float L0, float K[6], const float KL[6][4]) {
    for (int i = 0; i < 6; i++) {
        K[i] = KL[i][0] * powf(L0, 3) + KL[i][1] * powf(L0, 2) + KL[i][2] * powf(L0, 1) + KL[i][3] * powf(L0, 0);
    }
}

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

/** 更新状态变量 **/
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

/** 设置期望状态变量 **/
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

/** 计算误差 **/
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

/** 计算各项输出 **/
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

/*******************************************************************************
 *                                  Task                                       *
 *******************************************************************************/
 // 512
void update_task(void const *pvParameters)
{
    while(1)
    {
        /** 更新底盘结构体中IMU数据 **/
        get_IMU_info();

        /** 更新lqr **/
        lqr_ctrl();

        osDelay(UPDATE_PERIOD);
    }
}
