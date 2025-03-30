#include "device_joint.h"
#include "cmsis_os.h"
#include "chassis.h"

Dm8009 joint[4];
extern Chassis chassis;

void joint_init() {
    dm8009_init(&joint[LF], JOINT_LF_SEND);
    dm8009_init(&joint[LB], JOINT_LB_SEND);
    dm8009_init(&joint[RF], JOINT_RF_SEND);
    dm8009_init(&joint[RB], JOINT_RB_SEND);
}

void joint_enable() {
    for(int i = 0; i < 10; i ++)
    {
        set_dm8009_enable(CAN_2, JOINT_LF_SEND);
        osDelay(1);
        set_dm8009_enable(CAN_2, JOINT_LB_SEND);
        osDelay(1);
        set_dm8009_enable(CAN_2, JOINT_RF_SEND);
        osDelay(1);
        set_dm8009_enable(CAN_2, JOINT_RB_SEND);
        osDelay(1);
    }
}

void set_joint_torque(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque) {
    set_dm8009_torque(CAN_2, JOINT_LF_SEND, joint_LF_torque);
    osDelay(1);
    set_dm8009_torque(CAN_2, JOINT_LB_SEND, joint_LB_torque);
    osDelay(1);
    set_dm8009_torque(CAN_2, JOINT_RF_SEND, joint_RF_torque);
    osDelay(1);
    set_dm8009_torque(CAN_2, JOINT_RB_SEND, joint_RB_torque);
}

//void set_joint_torque(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque,
//                      float LF_pos, float LB_pos, float RF_pos, float RB_pos,
//                      float p, float d) {
//    set_dm8009_MIT(CAN_2,
//                   JOINT_LF_SEND,
//                   LF_pos,
//                   0,
//                   p,
//                   d,
//                   joint_LF_torque);
//
//    osDelay(1);
//
//    set_dm8009_MIT(CAN_2,
//                   JOINT_LB_SEND,
//                   LB_pos,
//                   0,
//                   p,
//                   d,
//                   joint_LB_torque);
//
//    osDelay(1);
//
//    set_dm8009_MIT(CAN_2,
//                   JOINT_RF_SEND,
//                   RF_pos,
//                   0,
//                   p,
//                   d,
//                   joint_RF_torque);
//
//    osDelay(1);
//
//    set_dm8009_MIT(CAN_2,
//                   JOINT_RB_SEND,
//                   RB_pos,
//                   0,
//                   p,
//                   d,
//                   joint_RB_torque);
//
//    osDelay(1);
//
//}

Dm8009* get_joint_motors(){
    return joint;
}

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
