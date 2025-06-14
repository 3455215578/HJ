#include "cmsis_os.h"
#include "robot_def.h"
#include "joint.h"
#include "bsp_dwt.h"

Dm8009 joint[4];

float Kp, Kd;
float pos, speed;

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

/** 判断关节是否复位 **/
static void joint_is_reset(void)
{

    if((joint[LF].pos_r > RESET_THRESHOLD) || ((joint[LB].pos_r < -RESET_THRESHOLD)
    || (joint[RF].pos_r < -RESET_THRESHOLD) || (joint[RB].pos_r > RESET_THRESHOLD)))
    {
        chassis.joint_is_reset = false;
    }
    else
    {
        chassis.joint_is_reset = true;
    }
}


void joint_reset(void)
{
    joint_is_reset();

    if(!chassis.joint_is_reset)
    {
        Kp = 3.0f;
        Kd = 0.0f;
        pos = 0.0f;
        speed = 0.0f;
    }
    else
    {
        Kp = 0.0f;
        Kd = 0.0f;
        pos = 0.0f;
        speed = 0.0f;
    }
}

/** 返回关节电机指针 **/
Dm8009* get_joint_motors(void)
{
    return joint;
}

