
/*ң������λ��

        S1                                                                         S0
                    |                                                   |
                    |                                                   |
                    |                                                   |
                    |                                                   |
           ------------------ 2                                ------------------ 0
                    |                                                   |
                    |                                                   |
                    |                                                   |
                    3                                                   1

*/

/*********************************************************************************************************
*                                              ����ͷ�ļ�
*********************************************************************************************************/
#include "gimbal_task.h"
#include "launcher.h"
#include "protocol_balance.h"
#include "ins_task.h"
#include "packet.h"
#include "cmsis_os.h"
#include "board_communication_task.h"

/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/

vision_t vision_data;   // ���Ӿ�����Ϣ
extern robot_ctrl_info_t robot_ctrl;    // ��ȡ�Ӿ���Ϣ
//todo: ͼ����
extern uint8_t control_flag;        // ͨ��״̬�ж���ʲô��·

float gyro_pitch = 0.0f;
float gyro_yaw = 0.0f;

/*********************************************************************************************************
*                                              �ڲ���������
*********************************************************************************************************/
_Noreturn void Gimbal_task(void const*pvParameters);
static void Gimbal_Init(void);

static void Gimbal_Angle_Update(void);
static void Send_Vision_Data(void);
static void Send_Gimbal_Data(void);
static void Gimbal_Device_Offline_Handle(void);

static void Gimbal_Mode_Set(void);
static void Gimbal_Control(void);

static void Gimbal_Disable(void);
static void Gimbal_Active_Handle(void);
static void Gimbal_Auto_Handle(void);
static void Gimbal_Current_Calc(void);

/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
/**
  * @brief          ��̨����
  * @param[in]      pvParameters
  * ��ʼ��������ģʽ��ʵ�ֹ��ܣ����� CAN �źţ���̨�ľ���ʵ���߼�
  * @retval         ����ָ��
  */
void Gimbal_task(void const*pvParameters) {
    /* �����ʼ��ʱ�� */
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    /* ��̨��ʼ�� */
    Gimbal_Init();
    /* ���������ʼ�� */
    Launcher_Init();

    while(1) {
        /* ����ң�����ͼ�����Ϣ */
        update_pc_info();
        /* ���ո��¾�����ԽǶ�ֵ */
        Gimbal_Angle_Update();
        /* ���Ӿ�����Ϣ */
        Send_Vision_Data();

        /* ģʽ���� */
        Gimbal_Mode_Set();
        Launcher_Mode_Set();

        /* ���ң�������������Դ�Ƿ���� */
        Gimbal_Device_Offline_Handle();

        /* ���°�䴫����Ϣ */
        Send_Gimbal_Data();

        /* ����ģʽ����ִ�п��� */
        Gimbal_Control();
        Launcher_Control();

        /* ���ң�������������Դ�Ƿ���� */
        Gimbal_Device_Offline_Handle();

        /* ���Ƶ�� */
//        DJI_Send_Motor_Mapping(CAN_1,
//                               CAN_DJI_MOTOR_0x200_ID,
//                               launcher.fire_l.target_current,    //201 ��Ħ����
//                               launcher.fire_r.target_current,    //202 ��Ħ����
//                               launcher.trigger.target_current,   //203 ����
//                               0    // 204 ��
//        );
//
//        DJI_Send_Motor_Mapping(CAN_1,
//                               CAN_DJI_MOTOR_0x1FF_ID,
//                               gimbal.yaw.target_current,       //205 yaw
//                               gimbal.pitch.target_current,     //206 pitch
//                               0,                               //207 ��
//                               0                                //208 ��
//        );

        DJI_Send_Motor_Mapping(CAN_1,
                               CAN_DJI_MOTOR_0x200_ID,
                               0,    //201 ��Ħ����
                               0,    //202 ��Ħ����
                               0,   //203 ����
                               0    // 204 ��
        );

        DJI_Send_Motor_Mapping(CAN_1,
                               CAN_DJI_MOTOR_0x1FF_ID,
                               gimbal.yaw.target_current,      //205 yaw
                               0,     //206 pitch
                               0,                               //207 ��
                               0                                //208 ��
        );


        vTaskDelay(GIMBAL_PERIOD);
    }
}


/******************/
/**
  * @brief          ��̨��ʼ��
  * @param[in]      none
  * ��ʼ��Ӣ�� mode �� last_mode ��ģʽΪʧ�ܣ���ȡ pitch �������
  * ��ʼ�� pitch ��� yaw �����ĽǶȻ����ٶȻ� PID (��������)
  * �� pitch ��� yaw �����ĵ�������Ϊ 0����ʧ��
  * ��ʼ��ʱ����̨��Ϊδ����״̬��У׼ pitch ��� yaw �����ı���ֵ
  * @retval         ���ؿ�
  */
static void Gimbal_Init(void) {

    gimbal.gimbal_ctrl_mode = gimbal.gimbal_last_ctrl_mode = GIMBAL_DISABLE;

    /* pit �����ǶȻ����ٶȻ�PID��ʼ�� */
    pid_init(&gimbal.pitch.speed_p,
             GIMBAL_PITCH_SPEED_MAX_OUT,
             GIMBAL_PITCH_SPEED_MAX_IOUT,
             GIMBAL_PITCH_SPEED_PID_KP,
             GIMBAL_PITCH_SPEED_PID_KI,
             GIMBAL_PITCH_SPEED_PID_KD);

    pid_init(&gimbal.pitch.angle_p,
             GIMBAL_PITCH_ANGLE_MAX_OUT,
             GIMBAL_PITCH_ANGLE_MAX_IOUT,
             GIMBAL_PITCH_ANGLE_PID_KP,
             GIMBAL_PITCH_ANGLE_PID_KI,
             GIMBAL_PITCH_ANGLE_PID_KD);

    /* yaw �����ǶȻ����ٶȻ�PID��ʼ�� */
    pid_init(&gimbal.yaw.speed_p,
             GIMBAL_YAW_SPEED_MAX_OUT, GIMBAL_YAW_SPEED_MAX_IOUT,
             GIMBAL_YAW_SPEED_PID_KP, GIMBAL_YAW_SPEED_PID_KI, GIMBAL_YAW_SPEED_PID_KD);

    pid_init(&gimbal.yaw.angle_p,
             GIMBAL_YAW_ANGLE_MAX_OUT, GIMBAL_YAW_ANGLE_MAX_IOUT,
             GIMBAL_YAW_ANGLE_PID_KP, GIMBAL_YAW_ANGLE_PID_KI, GIMBAL_YAW_ANGLE_PID_KD);

    /* �ҵ�yaw�������� */
    gimbal.yaw.motor_measure.offset_ecd = YAW_OFFSET_ECD;
    /* ʹpitchʧ�ܱ��ֵ�ǰ״̬ */
    gimbal.pitch.absolute_angle_set = gimbal.pitch.absolute_angle_get;

    gimbal.pitch.target_current = 0;
    gimbal.yaw.target_current = 0;

    //��ͨ�˲���ʼ��
    first_order_filter_init(&gimbal.mouse_in_x, 1, 40);
    first_order_filter_init(&gimbal.mouse_in_y, 1, 10);

    first_order_filter_init(&gimbal.auto_pitch, 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[0], 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[1], 1, 15);
    first_Kalman_Create(&gimbal.filter_autoYaw,1,20);  //�������˲�
    first_order_filter_init(&gimbal.filter_pitch_gyro_in, 1, 20);
    first_order_filter_init(&gimbal.filter_yaw_gyro_in, 5, 30);

    first_order_filter_init(&gimbal.pitch_first_order_set, 0.f, 500);
    first_order_filter_init(&gimbal.pitch_current_first_order_set, 5, 30);
}


/**
  * @brief          ��̨�Ƕȸ��£����Ӿ�ͨѶ
  * @param[in]      none
  * ���ݹ��Ե���ϵͳ�����ݸ�����̨�ĽǶ���Ϣ��������Ӧ����Ϣ���͵��Ӿ�ϵͳ��
  * ���У��Ƕ���Ϣͨ�������ǲ�������������������Լ����Ե���ϵͳ�ĽǶ���Ϣ�Ȼ�á�
  * @retval         ���ؿ�
  */
static void Gimbal_Angle_Update(void)
{
    /** Pitch **/
    gimbal.pitch.absolute_angle_get = INS_angle[2]*MOTOR_RAD_TO_ANGLE;
    gimbal.pitch.relative_angle_get = Motor_Ecd_To_Angle_Change(gimbal.pitch.motor_measure.ecd,
                                                                gimbal.pitch.motor_measure.offset_ecd);
    gyro_pitch = -INS_gyro[0]*MOTOR_RAD_TO_ANGLE;
    gimbal.absolute_gyro_pitch = (float) INS_gyro[0];

    /** Yaw **/
    gimbal.yaw.absolute_angle_get=INS_angle[0]*MOTOR_RAD_TO_ANGLE;
    gimbal.yaw.relative_angle_get= Motor_Ecd_To_Angle_Change(gimbal.yaw.motor_measure.ecd,
                                                             gimbal.yaw.motor_measure.offset_ecd);

    gyro_yaw = INS_gyro[2]*MOTOR_RAD_TO_ANGLE;
    gimbal.absolute_gyro_yaw = (float) INS_gyro[2];

}


static void Send_Vision_Data(void)
{
    // 107:�� 7:��
    if (Referee.GameRobotStat.robot_id < 10)
    {
        vision_data.id = 107;
    }
    else
    {
        vision_data.id = 7;
    }


    /* ���Ӿ��������� */
    if (gimbal.gimbal_ctrl_mode == GIMBAL_AUTO)
    {
        vision_data.mode = 0x21;
    }
    else
    {
        vision_data.mode = 0;
    }

    vision_data.pitch = gimbal.pitch.absolute_angle_get;
    vision_data.yaw   = gimbal.yaw.absolute_angle_get;
    vision_data.roll = (float) INS_angle[1] * MOTOR_RAD_TO_ANGLE;

    for (int i = 0; i < 4; ++i)
    {
        vision_data.quaternion[i] = INS_quat[i];
    }

    vision_data.shoot_speed = Referee.ShootData.bullet_speed;
    rm_queue_data(VISION_ID, &vision_data, sizeof(vision_t));
}


static void Gimbal_Device_Offline_Handle(void) {
    if(detect_list[DETECT_REMOTE].status == OFFLINE &&
       detect_list[DETECT_VIDEO_TRANSIMITTER].status == OFFLINE)
    {
        Gimbal_Disable();
        rc_ctrl.rc.ch[1] = rc_ctrl.rc.ch[2] = 0;
        KeyBoard.W.status = KeyBoard.A.status = KeyBoard.S.status = KeyBoard.D.status = 0;

    }
    if (detect_list[DETECT_GIMBAL_6020_PITCH].status == OFFLINE)
    {
        gimbal.pitch.absolute_angle_set = gimbal.pitch.absolute_angle_get;
        gimbal.pitch.target_current = 0;
    }
    if (detect_list[DETECT_GIMBAL_6020_YAW].status == OFFLINE)
    {
        gimbal.yaw.target_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_FIRE_L].status == OFFLINE)
    {
        launcher.fire_l.target_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_FIRE_R].status == OFFLINE)
    {
        launcher.fire_r.target_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_TRIGGER].status == OFFLINE)
    {
        launcher.trigger.target_current = 0;
    }
}

// �ú�����1kHz����̨�������ˣ���ϣ����500Hz����
static void Send_Gimbal_Data(void) {

    static int count = 1;

    if(count % 2 == 1) // ��������ż��������ʵ��500Hz
    {
        /* ң�������� */
        Gimbal_Send_Chassis(rc_ctrl.rc.ch[CHASSIS_VX_CHANNEL],
                            rc_ctrl.rc.ch[CHASSIS_YAW_CHANNEL],
                            rc_ctrl.rc.s[RC_s_L],
                            rc_ctrl.rc.s[RC_s_R]);
    }

    count ++;

}


/**
  * @brief          ��̨ģʽ���ã���ȡң������Ϣ���ж�ģʽ��
  * @param[in]      none
  * �ұ߲���������̨ʧ��
  * �ұ߲������к����ϵõ�ң��������̨����Ŀ��� ���� ��̨���У������ж�
  * @retval         ���ؿ�
  */
static void Gimbal_Mode_Set(void)
{
    //����ң����������̨ģʽ��ֻʹ�õ��ұ߲��ˣ�
    switch (rc_ctrl.rc.s[RC_s_R])
    {
        /** �������棬��̨ʧ�� **/
        case RC_SW_DOWN:
        {
            gimbal.gimbal_last_ctrl_mode = gimbal.gimbal_ctrl_mode;
            gimbal.gimbal_ctrl_mode = GIMBAL_DISABLE;
        } break;

        /** �����м�����棬��̨ʹ�� **/
        case RC_SW_MID:
        case RC_SW_UP:
        {
            /** ����ģʽ�ж� **/
            if (((KeyBoard.Mouse_r.status == KEY_PRESS) && (robot_ctrl.target_lock == 0x31) && (detect_list[DETECT_AUTO_AIM].status == ONLINE))
                ||((rc_ctrl.rc.ch[GIMBAL_AUTO_CHANNEL]> 50) && (robot_ctrl.target_lock == 0x31) && (detect_list[DETECT_AUTO_AIM].status == ONLINE)))
            {
                gimbal.gimbal_last_ctrl_mode = GIMBAL_ENABLE;
                gimbal.gimbal_ctrl_mode = GIMBAL_AUTO;
            }
            else
            {
                gimbal.gimbal_last_ctrl_mode = gimbal.gimbal_ctrl_mode;
                gimbal.gimbal_ctrl_mode = GIMBAL_ENABLE;
            }
        } break;

        default:
            break;
    }

    /** �˳�����ģʽ **/
    if (gimbal.gimbal_ctrl_mode == GIMBAL_AUTO)
    {
        if ((detect_list[DETECT_AUTO_AIM].status == OFFLINE) || robot_ctrl.target_lock == 0x32) //0x32��ʾ����������Ч
        {
            gimbal.gimbal_last_ctrl_mode = GIMBAL_AUTO;
            gimbal.gimbal_ctrl_mode = GIMBAL_ENABLE;
        }
    }
}


/**
  * @brief          ��̨ģʽ����ʵ��(����ģʽ)
  * @param[in]      none
  * ��̨ʧ�ܣ���̨���У���̨���ƣ���̨����
  * @retval         ���ؿ�
  */
static void Gimbal_Control(void) {
    switch (gimbal.gimbal_ctrl_mode)
    {
        case GIMBAL_DISABLE://��̨ʧ�ܣ�fire, pitch, single_shoot��
            Gimbal_Disable();
            break;

        case GIMBAL_ENABLE: {//��̨����
            Gimbal_Active_Handle();  //�õ�ң��������̨����Ŀ���
            Gimbal_Current_Calc();  //��̨����ջ����ƺ���
        } break;

        case GIMBAL_AUTO: { //��̨����ģʽ
            Gimbal_Auto_Handle();
            Gimbal_Current_Calc();  //��̨����ջ����ƺ���
        } break;
        default:
            break;
    }
}


/**
  * @brief          ��̨ʧ��ģʽ(�ĸ����)
  * @param[in]      none
  * pitch �ᣬ����Ħ���֣����������ĸ�����������Ϊ 0
  * @retval         ���ؿ�
  */
void Gimbal_Disable(void) {

    gimbal.pitch.absolute_angle_set = gimbal.pitch.absolute_angle_get;
    gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_get;

    gimbal.pitch.target_current = 0;
    gimbal.yaw.target_current = 0;

    Launcher_Disable();
}


//һ����ͷ
static void gimbal_turn_back_judge(void)
{
    if(KeyBoard.R.click_flag == 1)
    {
        KeyBoard.R.click_flag = 0;
        gimbal.yaw.absolute_angle_set += 180;
    }
}


/**
  * @brief          ʹ��ģʽ, �õ�ң��������̨����Ŀ���
  * @param[in]      none
  * ͨ��ң������ҡ�˻������� pitch ��� yaw �ᣬ�ԽǶ�ֵ�����޷�
  * @retval         ���ؿ�
  */
void Gimbal_Active_Handle(void) {
    //��������˲�
    //�������˲�������У׼��������x������Ϊ����
    if(control_flag == ALL_ONLINE) {
        /* ͼ������ */
        first_order_filter_cali(&gimbal.mouse_in_x,Referee.keyboard.mouse_x);
        first_order_filter_cali(&gimbal.mouse_in_y,Referee.keyboard.mouse_y);
        /* ң�������� */
        // first_order_filter_cali(&gimbal.mouse_in_x,rc_ctrl.mouse.x);
        // first_order_filter_cali(&gimbal.mouse_in_y,rc_ctrl.mouse.y);
    }
    if(control_flag == RC_ONLINE) {
        first_order_filter_cali(&gimbal.mouse_in_x,rc_ctrl.mouse.x);
        first_order_filter_cali(&gimbal.mouse_in_y,rc_ctrl.mouse.y);
    }
    if(control_flag == VT_ONLINE) {
        first_order_filter_cali(&gimbal.mouse_in_x,Referee.keyboard.mouse_x);
        first_order_filter_cali(&gimbal.mouse_in_y,Referee.keyboard.mouse_y);
    }

    /** pitch **/
    //��pit����ֵ��,��ң������������������
    gimbal.pitch.absolute_angle_set += (float)rc_ctrl.rc.ch[GIMBAL_PITCH_CHANNEL] * RC_TO_PITCH + (float)gimbal.mouse_in_y.out * MOUSE_Y_RADIO;  // rc_ctrl.mouse.y

    //��pit����ֵ���ж�̬�޷���ͨ�������Ǻͱ������õ���̬����λ��
    gimbal.pitch.absolute_angle_set = fp32_constrain(gimbal.pitch.absolute_angle_set,
                                                     MIN_ABS_ANGLE,
                                                     MAX_ABS_ANGLE);

    /** yaw **/
    //��yaw����ֵ��,��ң������������������
    gimbal.yaw.absolute_angle_set -= (float)rc_ctrl.rc.ch[GIMBAL_YAW_CHANNEL] * RC_TO_YAW + (float)gimbal.mouse_in_x.out * MOUSE_X_RADIO;    // rc_ctrl.mouse.x

    //��̨��Ȧʱ���о��Խ�ѭ������, ��yaw����ֵ����180��ʱ, ���������[-180,180]�ķ�Χ
    if (gimbal.yaw.absolute_angle_set >= 180) {
        gimbal.yaw.absolute_angle_set -= 360;
    }
    else if (gimbal.yaw.absolute_angle_set <= -180) {
        gimbal.yaw.absolute_angle_set += 360;
    }
}



/**
  * @brief          ��̨ pitch ����ջ����ƺ���
  * @param[in]      none
  * ʵ�� Pitch ��Ŀ��Ƽ��㣬��������������ֵ�ļ���͵�������ļ��㡣
  * �������Ƕ���ʵ�ʽǶ�֮��Ĳ�ֵ���д���ȷ���ò�ֵ�ں���Χ�ڡ�
  * �������ڽǶȻ������ʵ���PID���ƹ����еĲ��ȶ��ԡ�
  * @retval         ���ؿ�
  */
void Gimbal_Current_Calc(void){
    //����yaw��Ŀ������
    gimbal.yaw.gyro_set= pid_loop_calc(&gimbal.yaw.angle_p,
                                       gimbal.yaw.absolute_angle_get,
                                       gimbal.yaw.absolute_angle_set,
                                       180,
                                       -180);//gimbal.yaw.absolute_angle_set

    first_order_filter_cali(&gimbal.filter_yaw_gyro_in, gyro_yaw);

    gimbal.yaw.target_current = (int16_t)pid_calc(&gimbal.yaw.speed_p,
                                                gimbal.filter_yaw_gyro_in.out,//gimbal.yaw.motor_measure->speed_rpm,
                                                gimbal.yaw.gyro_set);

    //����pitch��Ŀ������
    gimbal.pitch.gyro_set = pid_calc(&gimbal.pitch.angle_p,
                                    gimbal.pitch.absolute_angle_get,
                                    gimbal.pitch.absolute_angle_set);//Vision_info.pitch.value

    first_order_filter_cali(&gimbal.filter_pitch_gyro_in,gyro_pitch);
    ///// ��ȡ�����ǵĽ��ٶȼ����ڻ�����������
    gimbal.pitch.target_current = (int16_t)pid_calc(&gimbal.pitch.speed_p,
                                                 gimbal.filter_pitch_gyro_in.out,
                                                 gimbal.pitch.gyro_set);


}


/**
  * @brief          ���鴦��õ�����̨����Ŀ���
  * @param[in]      none
  * ��ȡ�Ӿ����͵ĽǶ����ת�� pitch ��� yaw ��
  * @retval         ���ؿ�
  */


// �ƺ�ֻ�Ǵ�����yaw pitch����ֵ
float a_pitch = 0.0f;
float a_yaw = 0.0f;
void Gimbal_Auto_Handle(void) {

    //��ȡ�Ӿ����͵ĽǶ����
    first_order_filter_cali(&gimbal.auto_pitch, robot_ctrl.pitch);
    first_order_filter_cali(&gimbal.auto_yaw[0], sinf(robot_ctrl.yaw / 180.0f * PI));//yaw���ݷֽ��x
    first_order_filter_cali(&gimbal.auto_yaw[1], cosf(robot_ctrl.yaw / 180.0f * PI));//yaw���ݷֽ��y

    /** pitch **/
    gimbal.pitch.absolute_angle_set = gimbal.auto_pitch.out + a_pitch;

    //��pit����ֵ���ж�̬�޷���ͨ�������Ǻͱ������õ���̬����λ��
    gimbal.pitch.absolute_angle_set = fp32_constrain(gimbal.pitch.absolute_angle_set,
                                                     MIN_ABS_ANGLE,
                                                     MAX_ABS_ANGLE);

    /** yaw **/
    gimbal.yaw.absolute_angle_set = (atan2f(gimbal.auto_yaw[0].out, gimbal.auto_yaw[1].out) * 180.0f / PI) + a_yaw;//�ڴ˴����ϳ�

    //��̨��Ȧʱ���о��Խ�ѭ������
    if(gimbal.yaw.absolute_angle_set >= 180){
        gimbal.yaw.absolute_angle_set -= 360;
    }
    else if(gimbal.yaw.absolute_angle_set <= -180){
        gimbal.yaw.absolute_angle_set += 360;
    }

}