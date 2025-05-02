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
    RF=2,
    RB=3,
};

/** 初始化关节电机ID **/
void joint_init(void);

/** 使能关节电机 **/
void joint_enable(void);

/** 返回关节电机指针 **/
Dm8009* get_joint_motors(void);

/** 关节异常时复位 待修改 **/
void joint_reset(void);

extern Dm8009 joint[4];

#endif
