#include "gimbal_task.h"
#include "launcher.h"
#include "protocol_balance.h"
#include "ins_task.h"
#include "packet.h"
#include "cmsis_os.h"
#include "board_communication_task.h"


vision_t vision_data;   // ���Ӿ�����Ϣ
extern robot_ctrl_info_t robot_ctrl;    // ��ȡ�Ӿ���Ϣ
//todo: ͼ����
extern uint8_t control_flag;        // ͨ��״̬�ж���ʲô��·

/** ��̨PID��ʼ�� **/
static void Gimbal_Pid_Init(void)
{
    // Pitch
    pid_init(&gimbal.pitch.speed_pid,
             GIMBAL_PITCH_SPEED_MAX_OUT,
             GIMBAL_PITCH_SPEED_MAX_IOUT,
             GIMBAL_PITCH_SPEED_PID_KP,
             GIMBAL_PITCH_SPEED_PID_KI,
             GIMBAL_PITCH_SPEED_PID_KD);

    pid_init(&gimbal.pitch.angle_pid,
             GIMBAL_PITCH_ANGLE_MAX_OUT,
             GIMBAL_PITCH_ANGLE_MAX_IOUT,
             GIMBAL_PITCH_ANGLE_PID_KP,
             GIMBAL_PITCH_ANGLE_PID_KI,
             GIMBAL_PITCH_ANGLE_PID_KD);

    // Yaw
    pid_init(&gimbal.yaw.speed_pid,
             GIMBAL_YAW_SPEED_MAX_OUT,
             GIMBAL_YAW_SPEED_MAX_IOUT,
             GIMBAL_YAW_SPEED_PID_KP,
             GIMBAL_YAW_SPEED_PID_KI,
             GIMBAL_YAW_SPEED_PID_KD);

    pid_init(&gimbal.yaw.angle_pid,
             GIMBAL_YAW_ANGLE_MAX_OUT,
             GIMBAL_YAW_ANGLE_MAX_IOUT,
             GIMBAL_YAW_ANGLE_PID_KP,
             GIMBAL_YAW_ANGLE_PID_KI,
             GIMBAL_YAW_ANGLE_PID_KD);

}

/** ��̨��ʼ�� **/
static void Gimbal_Init(void) {

    gimbal.pitch.target_current = 0;
    gimbal.yaw.target_current = 0;

    /** ��ʼ����̨ģʽ **/
    gimbal.gimbal_ctrl_mode = gimbal.gimbal_last_ctrl_mode = GIMBAL_DISABLE;

    /** ��̨PID��ʼ�� **/
    Gimbal_Pid_Init();

    /** ȷ��Yaw��Pitch�ϵ縴λ�ĵ������ֵ **/
    gimbal.yaw.motor_measure.offset_ecd = YAW_OFFSET_ECD;
    gimbal.pitch.motor_measure.offset_ecd = PITCH_OFFSET_ECD;

    /** ��PID״̬���������0������ = ������ֹ�����ת **/
    gimbal.yaw.absolute_angle_set = gimbal.pitch.absolute_angle_get;
    gimbal.pitch.absolute_angle_set = gimbal.pitch.absolute_angle_get;

    /** ��ͨ�˲���ʼ�� **/
    // ��������˲�
    first_order_filter_init(&gimbal.mouse_in_x, 1, 40);
    first_order_filter_init(&gimbal.mouse_in_y, 1, 10);

    // ���ٶ��˲�
    first_order_filter_init(&gimbal.pitch_gyro_filter, 1, 20);
    first_order_filter_init(&gimbal.yaw_gyro_filter, 5, 30);

    // ���Ӿ����ص�yaw��pitch�˲�
    first_order_filter_init(&gimbal.auto_pitch, 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[0], 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[1], 1, 15);

}


/** ������̨�Ƕ� **/
static void Gimbal_Angle_Update(void)
{
    /** Pitch **/
    // IMU�Ƕ�
    gimbal.pitch.absolute_angle_get = INS_angle[2] * MOTOR_RAD_TO_ANGLE;
    // ���ϵ縴λλ�õĽǶȲ�
    gimbal.pitch.relative_angle_get = Motor_Ecd_To_Angle_Change(gimbal.pitch.motor_measure.ecd,
                                                                gimbal.pitch.motor_measure.offset_ecd);
    // ���ٶ�
    gimbal.pitch.gyro = -INS_gyro[0] * MOTOR_RAD_TO_ANGLE;

    /** Yaw **/
    // IMU�Ƕ�
    gimbal.yaw.absolute_angle_get = INS_angle[0] * MOTOR_RAD_TO_ANGLE;
    // ���ϵ縴λλ�õĽǶȲ�
    gimbal.yaw.relative_angle_get = Motor_Ecd_To_Angle_Change(gimbal.yaw.motor_measure.ecd,
                                                             gimbal.yaw.motor_measure.offset_ecd);
    // ���ٶ�
    gimbal.yaw.gyro = INS_gyro[2] * MOTOR_RAD_TO_ANGLE;

}

// ��Ϊ�����������1kHz����̨�������еģ���ϣ������500Hz����
static void Gimbal_Send_Chassis_Data(void) {

    static int count = 1;

    if(count % 2 == 1) // ��������ż��������ʵ��500Hz
    {
        Send_Chassis_Data(rc_ctrl.rc.ch[CHASSIS_VX_CHANNEL],
                          rc_ctrl.rc.ch[CHASSIS_LEG_CHANNEL],
                          rc_ctrl.rc.s[RC_s_R],
                          gimbal.init_flag,
                          gimbal.yaw.relative_angle_get);
    }

    count ++;

}

/** ��̨���������� **/
static void Gimbal_Controller_Calc(void)
{
    float yaw_gyro_set;
    float pitch_gyro_set;

    /** Yaw **/
    yaw_gyro_set = pid_loop_calc(&gimbal.yaw.angle_pid,
                            gimbal.yaw.absolute_angle_get,
                            gimbal.yaw.absolute_angle_set,
                            180,
                            -180);

    // �������ٶ��˲�
    first_order_filter_cali(&gimbal.yaw_gyro_filter, gimbal.yaw.gyro);

    gimbal.yaw.target_current = (int16_t)pid_calc(&gimbal.yaw.speed_pid,
                                                  gimbal.yaw_gyro_filter.out,//gimbal.yaw.motor_measure->speed_rpm,
                                                  yaw_gyro_set);

    /** Pitch **/
    pitch_gyro_set = pid_calc(&gimbal.pitch.angle_pid,
                        gimbal.pitch.absolute_angle_get,
                        gimbal.pitch.absolute_angle_set);

    // �������ٶ��˲�
    first_order_filter_cali(&gimbal.pitch_gyro_filter,gimbal.pitch.gyro);

    gimbal.pitch.target_current = (int16_t)pid_calc(&gimbal.pitch.speed_pid,
                                                    gimbal.pitch_gyro_filter.out,
                                                    pitch_gyro_set);
}

/** Yaw����ϵ縴λ **/
static void Gimbal_Yaw_Reset(void)
{
    if(ABS(gimbal.yaw.relative_angle_get) > 1.0f)
    {
        float yaw_gyro_set;

        /** Yaw **/
        yaw_gyro_set = pid_calc(&gimbal.yaw.angle_pid,
                                gimbal.yaw.relative_angle_get,
                                0.0f);

        // �������ٶ��˲�
        first_order_filter_cali(&gimbal.yaw_gyro_filter, gimbal.yaw.gyro);

        gimbal.yaw.target_current = (int16_t)pid_calc(&gimbal.yaw.speed_pid,
                                                      gimbal.yaw_gyro_filter.out,
                                                      yaw_gyro_set);
    }
    else
    {
        gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_get;

        gimbal.yaw.reset_finished = true;
    }
}

static void Gimbal_Pitch_Reset(void)
{

}

/*************************************************************************************************
 *                                           Task                                                *
 *************************************************************************************************/

static void Gimbal_Disable_Task(void)
{
    gimbal.pitch.target_current = 0;
    gimbal.yaw.target_current = 0;

    /** ����PID�����󣬵����ת **/
    gimbal.pitch.absolute_angle_set = gimbal.pitch.absolute_angle_get;
    gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_get;

    // ��̨����ϵ縴λ��־λ
    gimbal.yaw.reset_finished = false;
    gimbal.pitch.reset_finished = false;

    // ��̨��ʼ����־λ
    gimbal.init_flag = false;

}

/** �ϵ���̨��λ **/
static void Gimbal_Init_Task(void)
{
    Gimbal_Yaw_Reset();

    if(gimbal.yaw.reset_finished)
    {
        gimbal.init_flag = true;
    }

//    if(gimbal.yaw.reset_finished && gimbal.pitch.reset_finished)
//    {
//        gimbal.init_flag = true;
//    }

}

static void Gimbal_Enable_Task(void)
{
    Gimbal_Controller_Calc();
}

void Gimbal_task(void const*pvParameters) {
    /* �����ʼ��ʱ�� */
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    /** ��̨��ʼ�� **/
    Gimbal_Init();

    while(1) {

        // ����ң��������ģʽ��������Ϣ
        Gimbal_Remote_Cmd();

        // ������̨�Ƕ�
        Gimbal_Angle_Update();

        // ���ͨ��
        Gimbal_Send_Chassis_Data();

        switch(gimbal.gimbal_ctrl_mode)
        {
            case GIMBAL_DISABLE:
            {
                Gimbal_Disable_Task();
                break;
            }

            case GIMBAL_INIT:
            {
                Gimbal_Init_Task();
                break;
            }

            case GIMBAL_ENABLE:
            {
                Gimbal_Enable_Task();
                break;
            }
        }

        DJI_Send_Motor_Mapping(CAN_1,
                               CAN_DJI_MOTOR_0x200_ID,
                               0,    //201 ��Ħ����
                               0,    //202 ��Ħ����
                               0,    //203 ����
                               0     // 204 ��
        );

        DJI_Send_Motor_Mapping(CAN_1,
                               CAN_DJI_MOTOR_0x1FF_ID,
                               gimbal.yaw.target_current,     //205 yaw
                               0,     //206 pitch
                               0,     //207 ��
                               0      //208 ��
        );

        vTaskDelay(GIMBAL_PERIOD);
    }
}
