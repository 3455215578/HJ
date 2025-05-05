/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "launcher.h"
#include "gimbal_task.h"
#include "key_board.h"
#include "protocol_balance.h"
#include "Balance.h"
#include "gimbal_task.h"
#include "../../Algorithm/SMC/SMC.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
/* 左摩擦轮转速 PID */
#define SHOOT_FIRE_L_PID_KP         8//15//10//30 //80
#define SHOOT_FIRE_L_PID_KI         0.f
#define SHOOT_FIRE_L_PID_KD         1.f//50.f
#define SHOOT_FIRE_L_PID_MAX_IOUT   100
#define SHOOT_FIRE_L_PID_MAX_OUT    20000

/* 右摩擦轮转速 PID */
#define SHOOT_FIRE_R_PID_KP         8//15//10//30//80
#define SHOOT_FIRE_R_PID_KI         0.f
#define SHOOT_FIRE_R_PID_KD         1.f//50.f
#define SHOOT_FIRE_R_PID_MAX_IOUT   100
#define SHOOT_FIRE_R_PID_MAX_OUT    20000

/* 拨盘 PID */
#define TRIGGER_ANGLE_PID_KP        1.7f//0.2f//0.7f
#define TRIGGER_ANGLE_PID_KI        0.f
#define TRIGGER_ANGLE_PID_KD        0.5f//0.f//1.5f
#define TRIGGER_ANGLE_PID_MAX_IOUT  0
#define TRIGGER_ANGLE_PID_MAX_OUT   5000//3000

#define TRIGGER_SPEED_PID_KP        10.f//18.f//4.f//8.f
#define TRIGGER_SPEED_PID_KI        0.f
#define TRIGGER_SPEED_PID_KD        18.f
#define TRIGGER_SPEED_PID_MAX_IOUT  0
#define TRIGGER_SPEED_PID_MAX_OUT   10000//25000

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
launcher_t launcher;
extern robot_ctrl_info_t robot_ctrl;
static fp32 total_ecd_ref_tri = 0;
static uint32_t trigger_time = 0;

//滑膜控制系数
double dt = 0.001;
SMC_Params params = {
        .c = 5,             //滑模面系数（决定收敛速度）
        .rho = 2.0,         //切换增益（需大于扰动幅度）
        .epsilon = 1,       //边界层厚度（抑制抖振）
        .max_i = 10000      //控制输入限幅
};

//为每个电机定义状态变量
SystemState state_l = {0};
SystemState state_r = {0};

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static void Trigger_Finish_Judge();

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/**
 * 识别到在 SHOOT_READLY 时进入 SHOOT_SINGLE
 * 在规定时间内(1s)转动 ecd 差值小于 5000 则为完成单发任务
 * 在规定时间内(0.5s)转动 ecd 差值未达到规定则为堵转，触发堵转任务
 * 反转未成功会再反转一次(此时可能会导致电流过大损坏电机)
 */
#define TRI_MINECD 5000
#define TRI_MAXTIME 1000
static fp32 total_time = 0;
static fp32 total_ecd = 0;
static void Trigger_Finish_Judge() {
    if(launcher.shoot_cmd != SHOOT_CLOSE) {
        /* 单发中时规定时间内差值过大, 将其保持在单发状态 */
        total_time = HAL_GetTick() - trigger_time;
        total_ecd = ABS(total_ecd_ref_tri - launcher.trigger.motor_measure.total_ecd);
        if(launcher.shoot_cmd == SHOOT_SINGLE &&
           total_ecd > TRI_MINECD && total_time < TRI_MAXTIME) {
            launcher.shoot_cmd = SHOOT_SINGLE;
        }
            /* 在单发状态里面待的时间太长,判断其为堵转状态 */
        else if(launcher.shoot_cmd == SHOOT_SINGLE &&
                total_ecd > TRI_MINECD && total_time > TRI_MAXTIME) {
            launcher.shoot_cmd = SHOOT_BLOCK;
        }
        /* 反转时规定时间内差值过大, 将其保持在反转状态 */
        if(launcher.shoot_cmd == SHOOT_BLOCK_BACK && total_time < TRI_MAXTIME) {
            launcher.shoot_cmd = SHOOT_BLOCK_BACK;
        }
            /* 在封装状态里面待的时间太长,判断其为失败状态 */
        else if(launcher.shoot_cmd == SHOOT_BLOCK_BACK && total_time > TRI_MAXTIME) {
            launcher.shoot_cmd = SHOOT_FAIL;
        }
            /* 在规定时间内完成则将状态改为发射完成 */
        else if(total_ecd < TRI_MINECD) {
            launcher.shoot_cmd = SHOOT_OVER;
        }
    }
}


/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/**
 * 发射机构初始化
  */
void Launcher_Init(void) {
    /* 摩擦轮状态默认关闭 */
    launcher.fire_mode = Fire_OFF;
    launcher.fire_last_mode=Fire_OFF;
    /* 拨盘状态默认关闭 */
    launcher.shoot_last_cmd = SHOOT_CLOSE;
    launcher.shoot_cmd = SHOOT_CLOSE;

    /* 默认此时拨盘的初始ECD等于当前ECD, 提高发射准度 */
    launcher.trigger.motor_measure.total_ecd=launcher.trigger.motor_measure.offset_ecd=launcher.trigger.motor_measure.ecd;

    /* 拨盘PID */
    pid_init(&launcher.trigger.angle_p,
             TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT, TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD);
    pid_init(&launcher.trigger.speed_p,
             TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD);

    /* 电流初始化为0 */
    launcher.fire_l.give_current = 0;
    launcher.fire_r.give_current = 0;
    launcher.trigger.give_current = 0;

    /* 最开始的编码值作为拨轮电机的校准值 */
    first_order_filter_init(&launcher.filter_fire,0.05f, 0.5f);
    first_order_filter_init(&launcher.filter_trigger,1,1);
}

/**
 * 发射模式逻辑
 */
static uint8_t rc_last_sw_L;
static uint32_t time = 0;
static uint32_t last_time = 0;
void Launcher_Mode_Set() {
    if((!switch_is_up(rc_last_sw_L)) && switch_is_up(rc_ctrl.rc.s[RC_s_L])) {
        if(KeyBoard.Q.click_flag==1 && gimbal.mode!= GIMBAL_RELAX) KeyBoard.Q.click_flag=0;
        else if(KeyBoard.Q.click_flag==0 && gimbal.mode!= GIMBAL_RELAX) KeyBoard.Q.click_flag=1;
    }

    if(KeyBoard.Q.click_flag==1 && gimbal.mode!= GIMBAL_RELAX) launcher.fire_mode=Fire_ON;
    else if (KeyBoard.Q.click_flag==0 && gimbal.mode!= GIMBAL_RELAX) launcher.fire_mode=Fire_OFF;

    if(launcher.fire_mode == Fire_ON) {
        time = HAL_GetTick();
        /* 遥控器左键往下拨或长按鼠标左键，每100ms发出一颗弹丸 */
        if((switch_is_down(rc_last_sw_L) || KeyBoard.Mouse_l.status == KEY_PRESS)
           && time - last_time > 100 && launcher.shoot_cmd == SHOOT_OVER) {
            last_time = HAL_GetTick();
            launcher.shoot_cmd = SHOOT_READLY;
        }
        else if(robot_ctrl.fire_command == 1 && gimbal.mode == GIMBAL_AUTO) {
            launcher.shoot_cmd = SHOOT_READLY;
        }
        else if(launcher.shoot_cmd == SHOOT_BLOCK || launcher.shoot_cmd == SHOOT_BLOCK_BACK ||
                launcher.shoot_cmd == SHOOT_FAIL || launcher.shoot_cmd == SHOOT_SINGLE) {}
        else launcher.shoot_cmd = SHOOT_OVER;
    }
    else if(launcher.fire_mode == Fire_OFF) {
        launcher.shoot_cmd=SHOOT_CLOSE;
    }
    rc_last_sw_L=rc_ctrl.rc.s[RC_s_L];
}


/**
 * 发射机构的逻辑实现
  */
void Launcher_Control(void) {
    if(gimbal.mode == GIMBAL_RELAX) Launcher_Relax_Handle();
    else {
        if (launcher.fire_mode == Fire_ON) {
            launcher.fire_r.speed = FIRE_SPEED_R;
            launcher.fire_l.speed = -FIRE_SPEED_L;

            if (launcher.shoot_cmd == SHOOT_READLY) {
                trigger_time=HAL_GetTick(); //这时候开始计时，开始转的时候计时
                total_ecd_ref_tri = launcher.trigger.motor_measure.total_ecd - DEGREE_45_TO_ENCODER;
                launcher.shoot_cmd = SHOOT_SINGLE;   // 单发中
            }
            else if(launcher.shoot_cmd == SHOOT_BLOCK) {
                trigger_time=HAL_GetTick(); //这时候开始计时，开始转的时候计时(反转也需要重新计时)
                total_ecd_ref_tri = total_ecd_ref_tri + DEGREE_90_TO_ENCODER;
                launcher.shoot_cmd = SHOOT_BLOCK_BACK;
            }
            launcher.trigger.speed = pid_calc(&launcher.trigger.angle_p,
                                              launcher.trigger.motor_measure.total_ecd,
                                              total_ecd_ref_tri);
            launcher.trigger.give_current =
                    (int16_t)pid_calc(&launcher.trigger.speed_p,
                                      launcher.trigger.motor_measure.speed_rpm,
                                      launcher.trigger.speed);
        }
        else {
            launcher.fire_l.speed = 0;
            launcher.fire_r.speed = 0;
            launcher.trigger.give_current = 0;
            total_ecd_ref_tri=launcher.trigger.motor_measure.total_ecd;
        }

        state_l = update_system(launcher.fire_l.speed, launcher.fire_l.motor_measure.speed_rpm, dt, state_l);
        launcher.fire_l.give_current = -smc_controller(state_l, params);

        state_r = update_system(launcher.fire_r.speed, launcher.fire_r.motor_measure.speed_rpm, dt, state_r);
        launcher.fire_r.give_current = -smc_controller(state_r, params);

        Trigger_Finish_Judge(); // 堵转检测
    }
}


void Launcher_Relax_Handle(void) {
    launcher.fire_mode = Fire_OFF;
    launcher.shoot_cmd = SHOOT_CLOSE;
    launcher.fire_r.give_current = 0;
    launcher.fire_l.give_current = 0;
    launcher.trigger.give_current = 0;
    total_ecd_ref_tri = (int32_t)launcher.trigger.motor_measure.total_ecd;
}