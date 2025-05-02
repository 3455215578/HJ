#ifndef JOINT_H
#define JOINT_H

#include "../../can_device/can_device.h"
#include "dm_8009.h"

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

void left_joint_init();
void right_joint_init();

void left_joint_enable();
void right_joint_enable();

void set_left_joint_torque(float joint_LF_torque, float joint_LB_torque);
void set_right_joint_torque(float joint_RF_torque, float joint_RB_torque);

Dm8009* get_joint_motors();

#endif //JOINT_H
