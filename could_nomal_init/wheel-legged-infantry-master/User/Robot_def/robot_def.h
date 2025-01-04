#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdbool.h>

#include "pid.h"
#include "vmc.h"
#include "vx_kalman_filter.h"

/**  �궨�� �궨�� �궨�� �궨�� �궨�� �궨�� �궨��  **/
//CHASSIS_REMOTE��1��������ң��������
#define CHASSIS_REMOTE 1

//DEBUG_MODE��1���������ģʽ���رչؽں�������
#define DEBUG_MODE 0

#define CHASSIS_PERIOD 2 //5 // ms ����Ƶ��: 200Hz

/****************** PID���� *********************/

/** Wheel Wheel Wheel Wheel Wheel **/
// ת��PID
#define CHASSIS_TURN_PID_P 0.0f
#define CHASSIS_TURN_PID_I 0.0f
#define CHASSIS_TURN_PID_D 0.0f
#define CHASSIS_TURN_PID_IOUT_LIMIT 0.0f
#define CHASSIS_TURN_PID_OUT_LIMIT 0.0f
/************************************/


/** Joint Joint Joint Joint Joint **/
// �ȳ�λ�û�PID
#define CHASSIS_LEG_L0_POS_PID_P 0.0f
#define CHASSIS_LEG_L0_POS_PID_I 0.0f
#define CHASSIS_LEG_L0_POS_PID_D 0.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT 0.0f

// ������PID
#define CHASSIS_LEG_COORDINATION_PID_P 0.0f
#define CHASSIS_LEG_COORDINATION_PID_I 0.0f
#define CHASSIS_LEG_COORDINATION_PID_D 0.0f
#define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 0.0f

// Roll PID
#define CHASSIS_ROLL_PID_P 0.0f
#define CHASSIS_ROLL_PID_I 0.0f
#define CHASSIS_ROLL_PID_D 0.0f
#define CHASSIS_ROLL_PID_IOUT_LIMIT 0.0f
#define CHASSIS_ROLL_PID_OUT_LIMIT 0.0f

// ��غ���ȳ�PID
#define CHASSIS_OFFGROUND_LO_PID_P 0.0f
#define CHASSIS_OFFGROUND_L0_PID_I 0.0f
#define CHASSIS_OFFGROUND_L0_PID_D 0.0f
#define CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT 0.0f
#define CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT 0.0f
/*************************************/


/**********************************************/



/****************** ����Լ�� *********************/
#define MAX_CHASSIS_VX_SPEED 1.8f
#define MAX_PITCH 0.174533f
#define MIN_PITCH (-0.174533f)
#define MAX_ROLL 0.12f
#define MIN_ROLL (-0.12f)
#define MAX_WHEEL_TORQUE 10.f
#define MIN_WHEEL_TORQUE (-10.f)
#define MAX_JOINT_TORQUE 40.f
#define MIN_JOINT_TORQUE (-40.f)

#define MIN_L0 0.13f
#define MAX_L0 0.40f
#define MID_L0 0.24f
/**********************************************/



/************** ң����·����ӳ�� *****************/
#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Z_CHANNEL 2
#define CHASSIS_PIT_CHANNEL 3
#define CHASSIS_ROLL_CHANNEL 0
#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define MAX_CHASSIS_YAW_INCREMENT 0.02f
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)
#define RC_TO_PITCH ((MAX_PITCH-MIN_PITCH)/660)
#define RC_TO_ROLL ((MAX_ROLL-MIN_ROLL)/660)
/**********************************************/

typedef struct{
    float wheel_radius;
    float body_weight;
    float wheel_weight;
    float mechanical_leg_limit_angle;

    float l1, l2, l3, l4, l5;
} ChassisPhysicalConfig;

typedef enum{
    CHASSIS_DISABLE = 1,
    CHASSIS_ENABLE,
    CHASSIS_INIT,
    CHASSIS_JUMP,
    CHASSIS_SPIN,
} ChassisCtrlMode;

typedef enum{
    R = 1,
    L = 0,
} LegIndex;

typedef enum{
    NOT_READY,
    READY,
    STRETCHING,
    SHRINKING,
    STRETCHING_AGAIN,
    LANDING,
} JumpState;

typedef struct{
    // ŷ����
    float pitch_angle;
    float yaw_angle;
    float yaw_last_angle;
    float yaw_total_angle;
    float yaw_round_count;
    float roll_angle;

    //������ٶ�
    float pitch_gyro;
    float yaw_gyro;
    float roll_gyro;

    //������ٶ�
    float ax;
    float ay;
    float az;

    //ȥ���������ٶ�Ӱ����������ٶ�
    float ax_filtered;
    float ay_filtered;
    float az_filtered;

    // ������ֱ���ϵļ��ٶ�
    float robot_az;

} IMUReference;

typedef struct{
    float v_m_per_s;
    float x; // λ��
    float pitch_angle_rad;
    float yaw_angle_rad;
    float roll_angle_rad;
    float height_m;
    float spin_speed;

} ChassisCtrlInfo;

typedef struct{
    float theta;
    float theta_last;
    float theta_dot;
    float theta_dot_last;
    float theta_ddot;

    float x;
    float x_dot;
    float x_dot_last;
    float x_ddot;

    float phi;
    float phi_dot;
} StateVariable;

/** VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC **/

/** ���˶�ѧ���� ���˶�ѧ���� ���˶�ѧ���� ���˶�ѧ���� **/
typedef struct{
    float L0;
    float L0_last;
    float L0_dot;
    float L0_dot_last;
    float L0_ddot;
} FKL0;

typedef struct{// �������еĽǶ�
    float phi1;
    float phi2;
    float phi3;
    float phi4;

    float phi0;
    float last_phi0;
    float d_phi0;// �ڽǱ仯�ٶ�
    float last_d_phi0;
    float dd_phi0;
} FKPhi;

typedef struct{// �������е�����
    float a_x, a_y;
    float b_x, b_y;
    float c_x, c_y;
    float d_x, d_y;
    float e_x, e_y;
} FKPointCoordinates;

typedef struct{
    FKL0 fk_L0;
    FKPhi fk_phi;
    FKPointCoordinates fk_point_coordinates;
    float d_alpha; //

/*******************************************/

/** ������ѧ���� ������ѧ���� ������ѧ���� ������ѧ���� **/
    union { // ����ѧϰ�����������: union
        float array[2][2];
        struct {
            float Tp_set_point;
            float Fy_set_point;
        } E;
    } Fxy_set_point;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_F_to_T;

    union {
        float array[2][1];
        struct {
            float T1_set_point;
            float T4_set_point;
        } E;
    } T1_T4_set_point;
/************************************************/

} ForwardKinematics;


/** �涯��ѧ���� �涯��ѧ���� �涯��ѧ���� �涯��ѧ���� **/
typedef struct {
    union {
        float array[2][1];
        struct {
            float T1_fdb;
            float T4_fdb;
        } E;
    } T1_T4_fdb;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_T_to_F;

    union {
        float array[2][1];
        struct {
            float Tp_fdb;
            float Fy_fdb;
        } E;
    } Fxy_fdb;

    union {
        float array[2][1];
        struct {
            float w1_fdb;// �ؽڵ�����������Ľ��ٶ�
            float w4_fdb;
        } E;
    } W_fdb;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_w_to_v;

    union {
        float array[2][1];
        struct {
            float d_L0_fdb; // �ȳ��仯�ٶ�
            float d_phi0_fdb; // �ڽ�(phi0)�仯�ٶ�
        } E;
    } V_fdb;

}InverseKinematics;

/************************************************/

typedef struct{
    ForwardKinematics forward_kinematics;
    InverseKinematics inverse_kinematics;
} VMC;

/** VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC **/



/** Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg **/
typedef struct{
    LegIndex leg_index;
    StateVariable state_variable_feedback;
    StateVariable state_variable_set_point;
    StateVariable state_variable_error;
    StateVariable state_variable_wheel_out;
    StateVariable state_variable_joint_out;

    VMC vmc;

    Pid leg_pos_pid; // �ȳ�λ�û�pid
//    Pid leg_speed_pid; // �ȳ��ٶȻ�pid
    Pid offground_leg_pid;

    float* kalman_result; // ����ٶ�����ٶ��ںϺ�Ľ��
    float L0_set_point; // �����ȳ�

    float wheel_torque;
    float joint_F_torque;
    float joint_B_torque;

    float Fn; // ��ֱ����֧����

} Leg;

/** Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg Leg **/

/** Chassis Chassis Chassis Chassis Chassis Chassis Chassis Chassis **/

typedef struct{
    /** Remote Remote Remote Remote **/
    ChassisCtrlMode chassis_ctrl_mode;
    ChassisCtrlMode chassis_ctrl_mode_last;
    ChassisCtrlInfo chassis_ctrl_info;

    JumpState jump_state;

    IMUReference imu_reference;

    Leg leg_L;
    Leg leg_R;

    KalmanFilter vx_kalman;
    float kalman_measure[2];

    /** PID PID PID PID PID PID PID PID PID PID **/
//    Pid chassis_vw_speed_pid;
//    Pid chassis_spin_pid;
    Pid chassis_turn_pid; // ת��pid
    float wheel_turn_torque; // ת������
    float theta_error; // ������֮��theta�����
    Pid chassis_leg_coordination_pid; // ������pid
    float steer_compensatory_torque; // ����������
    Pid chassis_roll_pid; // roll����pid

    /** flag flag flag flag flag flag flag flag **/
    bool is_joint_enable; // �ؽڵ��ʹ�ܱ�־λ
    bool is_wheel_enable; // ��챵��ʹ�ܱ�־λ
    bool init_flag; // ���̳�ʼ����ɱ�־λ

    bool is_chassis_balance; // ƽ���־λ
    bool recover_finish; // ����������ɱ�־λ
    bool is_chassis_offground; // ��ر�־λ
    bool jump_flag; // ��Ծ��־λ


} Chassis;

/** Chassis Chassis Chassis Chassis Chassis Chassis Chassis Chassis **/



#endif
