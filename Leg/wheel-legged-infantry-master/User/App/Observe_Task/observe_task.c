#include "observe_task.h"
#include "robot_def.h"
#include "wheel.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "user_lib.h"

#define OBSERVE_PERIOD 3 // 观测任务周期为3ms

KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vel_acc[2]; // 轮毂速度与加速度融合后的结果

float vaEstimateKF_F[4] = {1.0f, 0.005f,
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.000025f, 0.005f,
                           0.005f, 1.0f};    // Q矩阵初始值、先验估计值方差噪声

float vaEstimateKF_R[4] = {0.0025f, 0.0f,
                           0.0f,  1.0f}; 	//200、200为测量噪声方差

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量


void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)//初始化卡尔曼结构体，并把该开头定义的矩阵复制到结构体中的矩阵
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维

    memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

static void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度

    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
        vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

void observe_task(void const *pvParameters)
{
    static float w_l,w_r=0.0f;//左右驱动轮的角速度
    static float v_lb,v_rb=0.0f;//通过左右驱动轮算出的机体速度
    static float aver_v=0.0f;//通过取平均计算出机体速度

    xvEstimateKF_Init(&vaEstimateKF);

//    TickType_t last_wake_time = xTaskGetTickCount();

    while(1)
    {
        // 左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
        w_l = -get_wheel_motors()->angular_vel - INS.Gyro[Y] + chassis.leg_L.vmc.forward_kinematics.d_alpha;
        // 轮毂相对于机体(b系)的速度
        v_lb = w_l * chassis_physical_config.wheel_radius + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_L.state_variable_feedback.theta_dot * arm_cos_f32(chassis.leg_L.state_variable_feedback.theta) + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot * arm_sin_f32(chassis.leg_L.state_variable_feedback.theta);

        // 右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
        w_r = -(get_wheel_motors() + 1)->angular_vel - INS.Gyro[Y] + chassis.leg_R.vmc.forward_kinematics.d_alpha;
        // 轮毂相对于机体(b系)的速度
        v_rb = w_r * chassis_physical_config.wheel_radius + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_R.state_variable_feedback.theta_dot * arm_cos_f32(chassis.leg_R.state_variable_feedback.theta) + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot * arm_sin_f32(chassis.leg_R.state_variable_feedback.theta);

        aver_v = (v_rb - v_lb) / 2.0f;//取平均

        xvEstimateKF_Update(&vaEstimateKF,INS.MotionAccel_b[X],aver_v);//不断更新卡尔曼滤波中的各项参数

        /** 更新状态变量 **/
        chassis.leg_L.state_variable_feedback.x_dot = vel_acc[0];
        chassis.leg_R.state_variable_feedback.x_dot = vel_acc[0];

        if(chassis.chassis_ctrl_info.v_m_per_s != 0.0f)
        {
            chassis.leg_L.state_variable_feedback.x = 0.0f;
            chassis.leg_R.state_variable_feedback.x = 0.0f;
        }
        else
        {
            chassis.leg_L.state_variable_feedback.x = chassis.leg_L.state_variable_feedback.x + OBSERVE_PERIOD * 0.001f * chassis.leg_L.state_variable_feedback.x_dot;
            chassis.leg_R.state_variable_feedback.x = chassis.leg_R.state_variable_feedback.x + OBSERVE_PERIOD * 0.001f * chassis.leg_R.state_variable_feedback.x_dot;

            if(ABS(vel_acc[0]) < 0.1f)
            {
                chassis.leg_L.state_variable_feedback.x = 0.0f;
                chassis.leg_R.state_variable_feedback.x = 0.0f;
            }

        }

        osDelay(OBSERVE_PERIOD);
//        vTaskDelayUntil(&last_wake_time, OBSERVE_PERIOD);
    }
}


