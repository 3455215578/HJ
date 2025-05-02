#include "cmsis_os.h"
#include "robot_def.h"
#include "joint.h"

Dm8009 joint[4];

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

/** 关节异常时复位 待修改 **/
void joint_reset(void)
{
    if((joint[LF].pos_r > LF_RESET) || ((joint[LB].pos_r < LB_RESET)
     || (joint[RF].pos_r < RF_RESET) || (joint[RB].pos_r > RB_RESET)))
    {
        chassis.joint_is_reset = false;
    }
    else
    {
        chassis.joint_is_reset = true;
    }

}
