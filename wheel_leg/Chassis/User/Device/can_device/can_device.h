#ifndef CAN_DEVICE_H
#define CAN_DEVICE_H

#include <stdint-gcc.h>
#include <stdbool.h>

/** �ؽں���챵��һ����ģʽ��CAN��ʶ�� **/
#define LK_FDB_Identifier 0x140
#define DM_FDB_Identifier 0x300

/** ���ͱ���ID **/
typedef enum{

    // �ؽڷ��ͱ���ID
    JOINT_LF_SEND = 0x01,
    JOINT_LB_SEND = 0x02,
    JOINT_RF_SEND = 0x03,
    JOINT_RB_SEND = 0x04,

    // ��챷��ͱ���ID
    WHEEL_L_SEND = 0x01,
    WHEEL_R_SEND = 0x02,

} CanSendDeviceId;

/** ��������ID **/
typedef enum{

    // �ؽڷ�������ID
    JOINT_LF_RECEIVE = 0x11,
    JOINT_LB_RECEIVE = 0x12,
    JOINT_RF_RECEIVE = 0x13,
    JOINT_RB_RECEIVE = 0x14,

} CanReceiveDeviceId;

#endif
