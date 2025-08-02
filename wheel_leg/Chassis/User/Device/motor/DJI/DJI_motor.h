#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

#include "bsp_can.h"

// C620/C610 id=1~4 (0x201~0x204)
#define CAN_DJI_MOTOR_0x200_ID 0x200

// C620/C610 id=5~8 (0x205~0x208)
// GM6020 id=1~4 (0x205~0x208)
#define CAN_DJI_MOTOR_0x1FF_ID 0x1FF

// GM6020 id=5~7 (0x209~0x20B)
#define CAN_DJI_MOTOR_0x2FF_ID 0x2FF

#define ECD360 8192
#define ECD180 4096
#define ECD90 2048
#define ECD45 1024


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

void DJI_info_update(DJI_Motor_t *motor, uint8_t *data);
void DJI_Round_Count(DJI_Motor_t *motor);
float DJI_Encoder_Limit(int16_t ecd);
float Motor_Ecd_To_Angle_Change(uint16_t ecd, uint16_t offset_ecd);


#endif

