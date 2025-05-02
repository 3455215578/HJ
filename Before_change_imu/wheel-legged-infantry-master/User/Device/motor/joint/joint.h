#ifndef JOINT_H
#define JOINT_H

#include "can_device.h"
#include "dm_8009.h"
#include "cybergear.h"

#define LF_RESET -1.576
#define LB_RESET 1.487
#define RF_RESET 1.533
#define RB_RESET -1.521

enum JointMotorIndex{
    LF=0,
    LB=1,
    RB=2,
    RF=3,
};

void joint_reset(void);

void joint_init();

void joint_enable();

void set_joint_torque(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque);

//void set_joint_torque(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque,
//                      float LF_pos, float LB_pos, float RF_pos, float RB_pos,
//                      float p, float d);
Dm8009* get_joint_motors();

#endif //JOINT_H
