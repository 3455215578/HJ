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

/** ��ʼ���ؽڵ��ID **/
void joint_init(void);

/** ʹ�ܹؽڵ�� **/
void joint_enable(void);

/** ���عؽڵ��ָ�� **/
Dm8009* get_joint_motors(void);

/** �ؽ��쳣ʱ��λ ���޸� **/
void joint_reset(void);

extern Dm8009 joint[4];

#endif
