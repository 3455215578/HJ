//
// Created by Pazfic on 2024/10/26.
//

#ifndef DHSENTRY_CHASSIS_DJI_MOTOR_H
#define DHSENTRY_CHASSIS_DJI_MOTOR_H

#include "struct_typedef.h"
#include "bsp_can.h"

typedef enum {
    CAN_1,
    CAN_2,
}CAN_TYPE;

// DJI�����CAN_ID
typedef enum {
    CAN_DJI_MOTOR_0x200_ID = 0x200, // C620/C610 id=1~4 (0x201~0x204)
    CAN_DJI_MOTOR_0x1FF_ID = 0x1FF, // C620/C610 id=5~8 (0x205~0x208);GM6020 id=1~4 (0x205~0x208)
    CAN_DJI_MOTOR_0x2FF_ID = 0x2FF, // GM6020 id=5~7 (0x209~0x20B)
}DJI_MOTOR_ID_e;

// DJI�����CAN_ID
typedef enum {
    /* ���̵������ģ�� */
    CAN_CHASSIS_MOTOR_RF=0x201,     //2     ǰ��
    CAN_CHASSIS_MOTOR_LF=0x202,     //2     ǰ��
    CAN_CHASSIS_MOTOR_LB=0x203,     //2     ����
    CAN_CHASSIS_MOTOR_RB=0x204,     //2     ����

    /* ��̨�ͷ�������������ģ�� */
    CAN_LAUNCHER_FIRE_L=0X201,     //2     ��Ħ����
    CAN_LAUNCHER_FIRE_R=0X202,     //2     ��Ħ����
    CAN_LAUNCHER_TRIGGER=0X203,    //2     ����
    CAN_GIMBAL_YAW=0x205,          //2     yaw��
    CAN_GIMBAL_PITCH=0x206,        //2     pitch��
}DJI_CAN_ID_e;

//���������
typedef struct {
    /* ʵ�ʵ���������� */
    int16_t last_ecd;       //��һ�εĵ������������ֵ
    uint16_t ecd;           //ת�ӻ�е�Ƕ�, �������������ֵ
    int16_t speed_rpm;      //ת��ת��, ���ת�٣�ÿ����ת����RPM��
    int16_t given_current;  //ʵ��Ť�ص���
    uint8_t temperate;      //����¶�

    /* �Զ������� */
    int32_t total_ecd;      //�����ת���ܱ�������ֵ
    uint16_t offset_ecd;    //�����У׼����ֵ
    int32_t round_cnt;      //�����ת����Ȧ��
} DJI_Motor_t;

void DJI_Motor_Decode(DJI_Motor_t *motor, uint8_t *data);
void DJI_Round_Count(DJI_Motor_t *motor);
fp32 DJI_Encoder_Limit(int16_t ecd);
void DJI_Send_Motor_Mapping(CAN_TYPE hcan, uint32_t can_id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
fp32 Motor_Ecd_To_Angle_Change(uint16_t ecd, uint16_t offset_ecd);


#endif //DHSENTRY_CHASSIS_DJI_MOTOR_H

