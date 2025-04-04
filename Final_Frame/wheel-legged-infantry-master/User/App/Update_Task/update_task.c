#include "update_task.h"
#include "robot_def.h"

#include "remote.h"
#include "error.h"
#include "ins_task.h"
#include "user_lib.h"
#include "wheel.h"

#define UPDATE_PERIOD 3 // ���ݸ�����������Ϊ3ms

extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;

/** �ٶ��ں� **/
KalmanFilter_t vaEstimateKF;	   // �������˲����ṹ��

float vel_acc[2]; // ����ٶ�����ٶ��ںϺ�Ľ��

float vaEstimateKF_F[4] = {1.0f, 0.005f,
                           0.0f, 1.0f};	   // ״̬ת�ƾ��󣬿�������Ϊ0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // �������Э�����ʼֵ

float vaEstimateKF_Q[4] = {0.000025f, 0.005f,
                           0.005f, 1.0f};    // Q�����ʼֵ���������ֵ��������

float vaEstimateKF_R[4] = {0.0025f, 0.0f,
                           0.0f,  1.0f}; 	//200��200Ϊ������������

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// ���þ���HΪ����

/** LQR **/
// ��ʼ��K����
float wheel_K_L[6] = {0, 0, 0, 0, 0, 0};
float joint_K_L[6] = {0, 0, 0, 0, 0, 0};

float wheel_K_R[6] = {0, 0, 0, 0, 0, 0};
float joint_K_R[6] = {0, 0, 0, 0, 0, 0};

// K���ϵ������
float wheel_fitting_factor[6][4] = {
        {-180.128775f,224.401180f,-139.003572f,-0.093513f},
        {-9.718106f,8.949069f,-14.856590f,-0.143007f},

        {-73.127694f,66.874031f,-19.362594f,-8.209220f},
        {-49.589597f,44.528163f,-14.217075f,-7.527445f},

        {-2568.054756f,2918.121091f,-1249.704742f,189.518497f},
        {-44.300525f,55.407893f,-27.380095f,8.304001f}
};float joint_fitting_factor[6][4] = {
        {314.132566f,-300.671491f,93.604539f,10.136978f},
        {25.116916f,-20.663557f,-3.491195f,1.588599f},

        {-255.367230f,285.537094f,-118.590711f,15.388693f},
        {-237.748535f,261.755785f,-107.900778f,13.868445f},

        {3958.282632f,-3767.171439f,1200.715310f,300.167031f},
        {145.325564f,-150.778294f,58.573076f,5.365314f}
};


/*******************************************************************************
 *                                    Remote                                   *
 *******************************************************************************/

/** ģ�����ߴ��� **/
static void chassis_device_offline_handle() {
    check_is_rc_online(get_rc_ctrl());
    if (get_errors() != 0) {
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    }
}

/** ���̽���ң������Ϣ **/
static void set_chassis_ctrl_info() {
    chassis.chassis_ctrl_info.v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_SPEED_CHANNEL]) * RC_TO_VX;

    chassis.chassis_ctrl_info.yaw_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_YAW_CHANNEL]) * (-RC_TO_YAW_INCREMENT);

}

/** ���̸���ң��������ģʽ **/
static void set_chassis_mode() {
    if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) { // ʧ��
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    }
    else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == false) { // ��ʼ��ģʽ
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_INIT;
    }
    else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && chassis.init_flag == true) { // ʹ��
        chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_ENABLE;

    }

}

/** ����ͨ�����ͨ�Ž�����̨����Ϣ **/
static void set_chassis_ctrl_info_from_gimbal_msg()
{

}

/** ���̸�����̨��Ϣ����ģʽ **/
static void set_chassis_mode_from_gimbal_msg()
{

}


/*******************************************************************************
 *                            ��ȡ���̴�������Ϣ                                   *
 *******************************************************************************/
static void get_IMU_info() {

    /** Yaw **/
    chassis.imu_reference.yaw_rad = -INS.Yaw * DEGREE_TO_RAD;
    chassis.imu_reference.yaw_total_rad = -INS.YawTotalAngle * DEGREE_TO_RAD;

    /** Pitch **/
    chassis.imu_reference.pitch_rad = -INS.Roll * DEGREE_TO_RAD;

    /** Roll **/
    chassis.imu_reference.roll_rad = INS.Pitch * DEGREE_TO_RAD;

    /** ���¸�����ٶȺͽ��ٶ� **/
    chassis.imu_reference.pitch_gyro = -INS.Gyro[Y];
    chassis.imu_reference.yaw_gyro = -INS.Gyro[Z];
    chassis.imu_reference.roll_gyro = INS.Gyro[X];

    chassis.imu_reference.ax = INS.Accel[X];
    chassis.imu_reference.ay = INS.Accel[Y];
    chassis.imu_reference.az = INS.Accel[Z];

    /** ������ֱ������ٶ� **/
    chassis.imu_reference.robot_az = INS.MotionAccel_n[Z];

}

/*******************************************************************************
 *                                  �ٶ��ں�                                    *
 *******************************************************************************/
void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)//��ʼ���������ṹ�壬���Ѹÿ�ͷ����ľ����Ƶ��ṹ���еľ���
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// ״̬����2ά û�п����� ��������2ά

    memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

static void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{
    //�������˲�������ֵ����
    EstimateKF->MeasuredVector[0] =	vel;//�����ٶ�
    EstimateKF->MeasuredVector[1] = acc;//�������ٶ�

    //�������˲������º���
    Kalman_Filter_Update(EstimateKF);

    // ��ȡ����ֵ
    for (uint8_t i = 0; i < 2; i++)
    {
        vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

void speed_calc(void)
{
    static float w_l,w_r=0.0f;//���������ֵĽ��ٶ�
    static float v_lb,v_rb=0.0f;//ͨ����������������Ļ����ٶ�
    static float aver_v=0.0f;//ͨ��ȡƽ������������ٶ�

    // ���������ת����Դ�ؽ��ٶȣ����ﶨ�����˳ʱ��Ϊ��
    w_l = -get_wheel_motors()->angular_vel - INS.Gyro[Y] + chassis.leg_L.vmc.forward_kinematics.d_alpha;
    // �������ڻ���(bϵ)���ٶ�
    v_lb = w_l * chassis_physical_config.wheel_radius + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_L.state_variable_feedback.theta_dot * arm_cos_f32(chassis.leg_L.state_variable_feedback.theta) + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot * arm_sin_f32(chassis.leg_L.state_variable_feedback.theta);

    // �ұ�������ת����Դ�ؽ��ٶȣ����ﶨ�����˳ʱ��Ϊ��
    w_r = -(get_wheel_motors() + 1)->angular_vel - INS.Gyro[Y] + chassis.leg_R.vmc.forward_kinematics.d_alpha;
    // �������ڻ���(bϵ)���ٶ�
    v_rb = w_r * chassis_physical_config.wheel_radius + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_R.state_variable_feedback.theta_dot * arm_cos_f32(chassis.leg_R.state_variable_feedback.theta) + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot * arm_sin_f32(chassis.leg_R.state_variable_feedback.theta);

    aver_v = (v_rb - v_lb) / 2.0f;//ȡƽ��

    xvEstimateKF_Update(&vaEstimateKF,INS.MotionAccel_b[X],aver_v);//���ϸ��¿������˲��еĸ������
}


/*******************************************************************************
 *                                  LQR                                        *
 *******************************************************************************/

/** ���K **/
void chassis_K_matrix_fitting(float L0, float K[6], const float KL[6][4]) {
    for (int i = 0; i < 6; i++) {
        K[i] = KL[i][0] * powf(L0, 3) + KL[i][1] * powf(L0, 2) + KL[i][2] * powf(L0, 1) + KL[i][3] * powf(L0, 0);
    }
}

/** ����״̬����theta **/
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

/** ����״̬���� **/
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

    //2.theta_dot��theta_ddot
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

        if(ABS(vel_acc[0]) < 0.1f)
        {
            leg_L->state_variable_feedback.x = 0.0f;
            leg_R->state_variable_feedback.x = 0.0f;
        }

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

/** ��������״̬���� **/
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

/** ������� **/
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

/** ���������� **/
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

    // K�������
    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

    // ���·���״̬����
    state_variable_update(&chassis.leg_L,
                          &chassis.leg_R,
                          chassis.imu_reference.pitch_rad,
                          chassis.imu_reference.pitch_gyro);

    // ��������״̬����
    state_variable_set(&chassis.leg_L, &chassis.leg_R);

    // �������(���� - ����)
    state_variable_error(&chassis.leg_L, &chassis.leg_R);

    // ����ÿ��״̬����������(ע�⼫��)
    state_variable_out(&chassis.leg_L, &chassis.leg_R);
}

/*******************************************************************************
 *                                  Task                                       *
 *******************************************************************************/
void update_task(void const *pvParameters)
{
    /** �ں��ٶȿ������˲�����ʼ�� **/
    xvEstimateKF_Init(&vaEstimateKF);

    TickType_t last_wake_time = xTaskGetTickCount();

    while(1)
    {
        /** ����ң����Ϣ **/
#if CHASSIS_REMOTE
        set_chassis_mode();

        set_chassis_ctrl_info();

        chassis_device_offline_handle();
#else
        set_chassis_mode_from_gimbal_msg();
        set_chassis_ctrl_info_from_gimbal_msg();
#endif

        /** ���µ��̽ṹ����IMU���� **/
        get_IMU_info();

        /** �����ٶ� **/
        speed_calc();

        /** ����lqr **/
        lqr_ctrl();

        vTaskDelayUntil(&last_wake_time, UPDATE_PERIOD);
    }
}
