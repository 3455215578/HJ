#include "cmsis_os.h"
#include "robot_def.h"
#include "joint.h"
#include "bsp_dwt.h"

Dm8009 joint[4];

float LF_pos = 0.0f;
float LB_pos = 0.0f;
float RF_pos = 0.0f;
float RB_pos = 0.0f;
float Kp = 0.0f;
float Kd = 0.0f;

/** 初始化关节电机ID **/
void joint_init(void) {
    dm8009_init(&joint[LF], JOINT_LF_SEND);
    dm8009_init(&joint[LB], JOINT_LB_SEND);
    dm8009_init(&joint[RF], JOINT_RF_SEND);
    dm8009_init(&joint[RB], JOINT_RB_SEND);
}

/** 使能关节电机 **/
void joint_enable(void) {
    for(int i = 0; i < 10; i ++)
    {
        set_dm8009_enable(&joint[LF]);
        osDelay(1);
    }
    for(int i = 0; i < 10; i ++)
    {
        set_dm8009_enable(&joint[LB]);
        osDelay(1);
    }
    for(int i = 0; i < 10; i ++)
    {
        set_dm8009_enable(&joint[RF]);
        osDelay(1);
    }
    for(int i = 0; i < 10; i ++)
    {
        set_dm8009_enable(&joint[RB]);
        osDelay(1);
    }
}

/** 返回关节电机指针 **/
Dm8009* get_joint_motors(void){
    return joint;
}

/** 倒地时关节位置异常  -> 复位 **/
//static void joint_is_reset(void)
//{
//    if(!chassis.recover_finish)
//    {
//        if((joint[LF].pos_r > LF_RESET) || ((joint[LB].pos_r < LB_RESET)
//        || (joint[RF].pos_r < RF_RESET) || (joint[RB].pos_r > RB_RESET)))
//        {
//            chassis.joint_is_reset = false;
//        }
//        else
//        {
//            chassis.joint_is_reset = true;
//        }
//    }
//
//
//}
//
//void joint_reset(void)
//{
//    joint_is_reset();
//
//    if(!chassis.joint_is_reset)
//    {
//        Kp = 20.0f;
//        Kd = 5.0f;
//
//        LF_pos = LF_RESET;
//        LB_pos = LB_RESET;
//        RF_pos = RF_RESET;
//        RB_pos = RB_RESET;
//
//    }
//    else
//    {
//        Kp = 0.0f;
//        Kd = 0.0f;
//
//        LF_pos = 0.0f;
//        LB_pos = 0.0f;
//        RF_pos = 0.0f;
//        RB_pos = 0.0f;
//    }
//}
