#ifndef CAN_DEVICE_H
#define CAN_DEVICE_H

#include <stdbool.h>

/** �ؽں���챵��һ����ģʽ��CAN��ʶ�� **/
#define LK_FDB_Identifier 0x140
#define DM_FDB_Identifier 0x300 // �ؽ�û��һ����ģʽ

/** �ؽڵ������ID **/
typedef enum{
  JOINT_LF_SEND = 0x01,
  JOINT_LB_SEND = 0x02,
  JOINT_RF_SEND = 0x03,
  JOINT_RB_SEND = 0x04,
} Dm8009SendID;

/** ��챵������ID **/
typedef enum{
  WHEEL_L_SEND = 0x01,
  WHEEL_R_SEND = 0x02,
} Lk9025SendID;

typedef enum{
  CHASSIS_ANGLE_VEL_INFO = 0x111,
} GimbalSendID;

/** ��������ID **/
typedef enum{

  JOINT_LF_RECEIVE = 0x11,
  JOINT_LB_RECEIVE = 0x12,
  JOINT_RF_RECEIVE = 0x13,
  JOINT_RB_RECEIVE = 0x14,

  WHEEL_L_RECEIVE = 0x141,
  WHEEL_R_RECEIVE = 0x142,

  CHASSIS_MODE_HEIGHT_INFO = 0x101,
  CHASSIS_SPEED_INFO = 0x102,
  CHASSIS_ANGLE_INFO = 0x103,

} CanReceiveDeviceId;


#endif
