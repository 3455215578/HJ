#ifndef JOINT_H
#define JOINT_H

#include "can_device.h"
#include "dm_8009.h"

// 10���Ӧ0.174rad

//#define LF_RESET -1.576f // -90��
//#define LB_RESET 1.576f // 90��
//#define RF_RESET 1.576f // 90��
//#define RB_RESET -1.576f // -90��

#define LF_RESET (-1.744f + 0.174f) // -80��
#define LB_RESET (1.576f - 0.174f) // 80��
#define RF_RESET (1.576f - 0.174f) // 80��
#define RB_RESET (-1.576f + 0.174f) // -80��

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

///** �ؽ�λ���쳣ʱ��λ **/
//void joint_reset(void);

extern Dm8009 joint[4];

extern float LF_pos;
extern float LB_pos;
extern float RF_pos;
extern float RB_pos;
extern float Kp;
extern float Kd;

#endif
