
/*遥控器键位：

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
*                                              包含头文件
*********************************************************************************************************/
#include "gimbal_task.h"
#include "launcher.h"
#include "protocol_balance.h"
#include "ins_task.h"
#include "packet.h"
#include "cmsis_os.h"
#include "board_communication_task.h"

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/

vision_t vision_data;   // 给视觉传信息
extern robot_ctrl_info_t robot_ctrl;    // 获取视觉信息
//todo: 图传的
extern uint8_t control_flag;        // 通过状态判断是什么链路

float gyro_pitch = 0.0f;
float gyro_yaw = 0.0f;

/*********************************************************************************************************
*                                              内部函数声明
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
*                                              内部函数实现
*********************************************************************************************************/
/**
  * @brief          云台任务
  * @param[in]      pvParameters
  * 初始化，设置模式，实现功能，接受 CAN 信号，云台的具体实现逻辑
  * @retval         返回指针
  */
void Gimbal_task(void const*pvParameters) {
    /* 任务初始化时间 */
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    /* 云台初始化 */
    Gimbal_Init();
    /* 发射机构初始化 */
    Launcher_Init();

    while(1) {
        /* 更新遥控器和键鼠信息 */
        update_pc_info();
        /* 接收更新绝对相对角度值 */
        Gimbal_Angle_Update();
        /* 给视觉发信息 */
        Send_Vision_Data();

        /* 模式设置 */
        Gimbal_Mode_Set();
        Launcher_Mode_Set();

        /* 检测遥控器、电机、电源是否断线 */
        Gimbal_Device_Offline_Handle();

        /* 更新板间传输信息 */
        Send_Gimbal_Data();

        /* 根据模式设置执行控制 */
        Gimbal_Control();
        Launcher_Control();

        /* 检测遥控器、电机、电源是否断线 */
        Gimbal_Device_Offline_Handle();

        /* 控制电机 */
//        DJI_Send_Motor_Mapping(CAN_1,
//                               CAN_DJI_MOTOR_0x200_ID,
//                               launcher.fire_l.target_current,    //201 左摩擦轮
//                               launcher.fire_r.target_current,    //202 右摩擦轮
//                               launcher.trigger.target_current,   //203 拨盘
//                               0    // 204 无
//        );
//
//        DJI_Send_Motor_Mapping(CAN_1,
//                               CAN_DJI_MOTOR_0x1FF_ID,
//                               gimbal.yaw.target_current,       //205 yaw
//                               gimbal.pitch.target_current,     //206 pitch
//                               0,                               //207 无
//                               0                                //208 无
//        );

        DJI_Send_Motor_Mapping(CAN_1,
                               CAN_DJI_MOTOR_0x200_ID,
                               0,    //201 左摩擦轮
                               0,    //202 右摩擦轮
                               0,   //203 拨盘
                               0    // 204 无
        );

        DJI_Send_Motor_Mapping(CAN_1,
                               CAN_DJI_MOTOR_0x1FF_ID,
                               gimbal.yaw.target_current,      //205 yaw
                               0,     //206 pitch
                               0,                               //207 无
                               0                                //208 无
        );


        vTaskDelay(GIMBAL_PERIOD);
    }
}


/******************/
/**
  * @brief          云台初始化
  * @param[in]      none
  * 初始化英雄 mode 和 last_mode 的模式为失能，获取 pitch 电机数据
  * 初始化 pitch 轴和 yaw 轴电机的角度环和速度环 PID (包括自瞄)
  * 将 pitch 轴和 yaw 轴电机的电流给定为 0，即失能
  * 初始化时，云台设为未回中状态，校准 pitch 轴和 yaw 轴电机的编码值
  * @retval         返回空
  */
static void Gimbal_Init(void) {

    gimbal.gimbal_ctrl_mode = gimbal.gimbal_last_ctrl_mode = GIMBAL_DISABLE;

    /* pit 轴电机角度环和速度环PID初始化 */
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

    /* yaw 轴电机角度环和速度环PID初始化 */
    pid_init(&gimbal.yaw.speed_p,
             GIMBAL_YAW_SPEED_MAX_OUT, GIMBAL_YAW_SPEED_MAX_IOUT,
             GIMBAL_YAW_SPEED_PID_KP, GIMBAL_YAW_SPEED_PID_KI, GIMBAL_YAW_SPEED_PID_KD);

    pid_init(&gimbal.yaw.angle_p,
             GIMBAL_YAW_ANGLE_MAX_OUT, GIMBAL_YAW_ANGLE_MAX_IOUT,
             GIMBAL_YAW_ANGLE_PID_KP, GIMBAL_YAW_ANGLE_PID_KI, GIMBAL_YAW_ANGLE_PID_KD);

    /* 找到yaw轴正方向 */
    gimbal.yaw.motor_measure.offset_ecd = YAW_OFFSET_ECD;
    /* 使pitch失能保持当前状态 */
    gimbal.pitch.absolute_angle_set = gimbal.pitch.absolute_angle_get;

    gimbal.pitch.target_current = 0;
    gimbal.yaw.target_current = 0;

    //低通滤波初始化
    first_order_filter_init(&gimbal.mouse_in_x, 1, 40);
    first_order_filter_init(&gimbal.mouse_in_y, 1, 10);

    first_order_filter_init(&gimbal.auto_pitch, 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[0], 1, 15);
    first_order_filter_init(&gimbal.auto_yaw[1], 1, 15);
    first_Kalman_Create(&gimbal.filter_autoYaw,1,20);  //卡尔曼滤波
    first_order_filter_init(&gimbal.filter_pitch_gyro_in, 1, 20);
    first_order_filter_init(&gimbal.filter_yaw_gyro_in, 5, 30);

    first_order_filter_init(&gimbal.pitch_first_order_set, 0.f, 500);
    first_order_filter_init(&gimbal.pitch_current_first_order_set, 5, 30);
}


/**
  * @brief          云台角度更新，与视觉通讯
  * @param[in]      none
  * 根据惯性导航系统的数据更新云台的角度信息，并将相应的信息发送到视觉系统。
  * 其中，角度信息通过陀螺仪测量、电机编码器数据以及惯性导航系统的角度信息等获得。
  * @retval         返回空
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
    // 107:蓝 7:红
    if (Referee.GameRobotStat.robot_id < 10)
    {
        vision_data.id = 107;
    }
    else
    {
        vision_data.id = 7;
    }


    /* 给视觉发开自瞄 */
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

// 该函数在1kHz的云台任务中运，我希望以500Hz运行
static void Send_Gimbal_Data(void) {

    static int count = 1;

    if(count % 2 == 1) // 奇数发，偶数不发，实现500Hz
    {
        /* 遥控器数据 */
        Gimbal_Send_Chassis(rc_ctrl.rc.ch[CHASSIS_VX_CHANNEL],
                            rc_ctrl.rc.ch[CHASSIS_YAW_CHANNEL],
                            rc_ctrl.rc.s[RC_s_L],
                            rc_ctrl.rc.s[RC_s_R]);
    }

    count ++;

}


/**
  * @brief          云台模式设置（获取遥控器信息，判断模式）
  * @param[in]      none
  * 右边拨杆向下云台失能
  * 右边拨杆向中和向上得到遥控器对云台电机的控制 —— 云台回中，自瞄判定
  * @retval         返回空
  */
static void Gimbal_Mode_Set(void)
{
    //根据遥控器设置云台模式（只使用到右边拨杆）
    switch (rc_ctrl.rc.s[RC_s_R])
    {
        /** 拨到下面，云台失能 **/
        case RC_SW_DOWN:
        {
            gimbal.gimbal_last_ctrl_mode = gimbal.gimbal_ctrl_mode;
            gimbal.gimbal_ctrl_mode = GIMBAL_DISABLE;
        } break;

        /** 拨到中间或上面，云台使能 **/
        case RC_SW_MID:
        case RC_SW_UP:
        {
            /** 自瞄模式判定 **/
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

    /** 退出自瞄模式 **/
    if (gimbal.gimbal_ctrl_mode == GIMBAL_AUTO)
    {
        if ((detect_list[DETECT_AUTO_AIM].status == OFFLINE) || robot_ctrl.target_lock == 0x32) //0x32表示自瞄数据无效
        {
            gimbal.gimbal_last_ctrl_mode = GIMBAL_AUTO;
            gimbal.gimbal_ctrl_mode = GIMBAL_ENABLE;
        }
    }
}


/**
  * @brief          云台模式设置实现(四种模式)
  * @param[in]      none
  * 云台失能，云台回中，云台控制，云台自瞄
  * @retval         返回空
  */
static void Gimbal_Control(void) {
    switch (gimbal.gimbal_ctrl_mode)
    {
        case GIMBAL_DISABLE://云台失能（fire, pitch, single_shoot）
            Gimbal_Disable();
            break;

        case GIMBAL_ENABLE: {//云台控制
            Gimbal_Active_Handle();  //得到遥控器对云台电机的控制
            Gimbal_Current_Calc();  //云台电机闭环控制函数
        } break;

        case GIMBAL_AUTO: { //云台自瞄模式
            Gimbal_Auto_Handle();
            Gimbal_Current_Calc();  //云台电机闭环控制函数
        } break;
        default:
            break;
    }
}


/**
  * @brief          云台失能模式(四个电机)
  * @param[in]      none
  * pitch 轴，左右摩擦轮，主动单发的给定电流设置为 0
  * @retval         返回空
  */
void Gimbal_Disable(void) {

    gimbal.pitch.absolute_angle_set = gimbal.pitch.absolute_angle_get;
    gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_get;

    gimbal.pitch.target_current = 0;
    gimbal.yaw.target_current = 0;

    Launcher_Disable();
}


//一键掉头
static void gimbal_turn_back_judge(void)
{
    if(KeyBoard.R.click_flag == 1)
    {
        KeyBoard.R.click_flag = 0;
        gimbal.yaw.absolute_angle_set += 180;
    }
}


/**
  * @brief          使能模式, 得到遥控器对云台电机的控制
  * @param[in]      none
  * 通过遥控器左摇杆或鼠标控制 pitch 轴和 yaw 轴，对角度值进行限幅
  * @retval         返回空
  */
void Gimbal_Active_Handle(void) {
    //鼠标输入滤波
    //对输入滤波器进行校准，将鼠标的x坐标作为输入
    if(control_flag == ALL_ONLINE) {
        /* 图传优先 */
        first_order_filter_cali(&gimbal.mouse_in_x,Referee.keyboard.mouse_x);
        first_order_filter_cali(&gimbal.mouse_in_y,Referee.keyboard.mouse_y);
        /* 遥控器优先 */
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
    //在pit期望值上,按遥控器或者鼠标进行增减
    gimbal.pitch.absolute_angle_set += (float)rc_ctrl.rc.ch[GIMBAL_PITCH_CHANNEL] * RC_TO_PITCH + (float)gimbal.mouse_in_y.out * MOUSE_Y_RADIO;  // rc_ctrl.mouse.y

    //对pit期望值进行动态限幅（通过陀螺仪和编码器得到动态的限位）
    gimbal.pitch.absolute_angle_set = fp32_constrain(gimbal.pitch.absolute_angle_set,
                                                     MIN_ABS_ANGLE,
                                                     MAX_ABS_ANGLE);

    /** yaw **/
    //在yaw期望值上,按遥控器或者鼠标进行增减
    gimbal.yaw.absolute_angle_set -= (float)rc_ctrl.rc.ch[GIMBAL_YAW_CHANNEL] * RC_TO_YAW + (float)gimbal.mouse_in_x.out * MOUSE_X_RADIO;    // rc_ctrl.mouse.x

    //云台绕圈时进行绝对角循环设置, 当yaw期望值超过180度时, 将其调整到[-180,180]的范围
    if (gimbal.yaw.absolute_angle_set >= 180) {
        gimbal.yaw.absolute_angle_set -= 360;
    }
    else if (gimbal.yaw.absolute_angle_set <= -180) {
        gimbal.yaw.absolute_angle_set += 360;
    }
}



/**
  * @brief          云台 pitch 电机闭环控制函数
  * @param[in]      none
  * 实现 Pitch 轴的控制计算，包括陀螺仪期望值的计算和电流输出的计算。
  * 对期望角度与实际角度之间的差值进行处理，确保该差值在合理范围内。
  * 避免由于角度环形性质导致PID控制过程中的不稳定性。
  * @retval         返回空
  */
void Gimbal_Current_Calc(void){
    //计算yaw轴的控制输出
    gimbal.yaw.gyro_set= pid_loop_calc(&gimbal.yaw.angle_p,
                                       gimbal.yaw.absolute_angle_get,
                                       gimbal.yaw.absolute_angle_set,
                                       180,
                                       -180);//gimbal.yaw.absolute_angle_set

    first_order_filter_cali(&gimbal.filter_yaw_gyro_in, gyro_yaw);

    gimbal.yaw.target_current = (int16_t)pid_calc(&gimbal.yaw.speed_p,
                                                gimbal.filter_yaw_gyro_in.out,//gimbal.yaw.motor_measure->speed_rpm,
                                                gimbal.yaw.gyro_set);

    //计算pitch轴的控制输出
    gimbal.pitch.gyro_set = pid_calc(&gimbal.pitch.angle_p,
                                    gimbal.pitch.absolute_angle_get,
                                    gimbal.pitch.absolute_angle_set);//Vision_info.pitch.value

    first_order_filter_cali(&gimbal.filter_pitch_gyro_in,gyro_pitch);
    ///// 读取陀螺仪的角速度加在内环的期望上面
    gimbal.pitch.target_current = (int16_t)pid_calc(&gimbal.pitch.speed_p,
                                                 gimbal.filter_pitch_gyro_in.out,
                                                 gimbal.pitch.gyro_set);


}


/**
  * @brief          自瞄处理得到对云台电机的控制
  * @param[in]      none
  * 获取视觉发送的角度误差转动 pitch 轴和 yaw 轴
  * @retval         返回空
  */


// 似乎只是处理了yaw pitch期望值
float a_pitch = 0.0f;
float a_yaw = 0.0f;
void Gimbal_Auto_Handle(void) {

    //获取视觉发送的角度误差
    first_order_filter_cali(&gimbal.auto_pitch, robot_ctrl.pitch);
    first_order_filter_cali(&gimbal.auto_yaw[0], sinf(robot_ctrl.yaw / 180.0f * PI));//yaw数据分解成x
    first_order_filter_cali(&gimbal.auto_yaw[1], cosf(robot_ctrl.yaw / 180.0f * PI));//yaw数据分解成y

    /** pitch **/
    gimbal.pitch.absolute_angle_set = gimbal.auto_pitch.out + a_pitch;

    //对pit期望值进行动态限幅（通过陀螺仪和编码器得到动态的限位）
    gimbal.pitch.absolute_angle_set = fp32_constrain(gimbal.pitch.absolute_angle_set,
                                                     MIN_ABS_ANGLE,
                                                     MAX_ABS_ANGLE);

    /** yaw **/
    gimbal.yaw.absolute_angle_set = (atan2f(gimbal.auto_yaw[0].out, gimbal.auto_yaw[1].out) * 180.0f / PI) + a_yaw;//在此处做合成

    //云台绕圈时进行绝对角循环设置
    if(gimbal.yaw.absolute_angle_set >= 180){
        gimbal.yaw.absolute_angle_set -= 360;
    }
    else if(gimbal.yaw.absolute_angle_set <= -180){
        gimbal.yaw.absolute_angle_set += 360;
    }

}