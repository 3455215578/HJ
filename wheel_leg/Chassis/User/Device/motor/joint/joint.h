#ifndef JOINT_H
#define JOINT_H

#include "can_device.h"
#include "dm_8009.h"

// 10���Ӧ0.174rad
#define RESET_THRESHOLD 0.522f

enum JointMotorIndex{
    LF=0,
    LB=1,
    RF=2,
    RB=3,
};

/** ��ʼ���ؽڵ��ID **/
void joint_init(void);

/** ʹ�ܹؽڵ�� **/
void joint_enable(void);

/** ���عؽڵ��ָ�� **/
Dm8009* get_joint_motors(void);

extern Dm8009 joint[4];

#endif
