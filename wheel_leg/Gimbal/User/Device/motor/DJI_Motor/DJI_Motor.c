
//
// Created by Pazfic on 2024/10/26.
//

#include "DJI_Motor.h"

// �������ֵ�����Ӧ�Ƕ�
#define ECD360 8192
#define ECD180 4096
#define ECD90 2048
#define ECD45 1024



/**
 * @brief DJI�������
 * @param[in] motor  ����ṹ��ָ��
 * @param[in] data   ���յ������ݵ�ָ��
 */
void DJI_Motor_Decode(DJI_Motor_t *motor, uint8_t *data) {
    motor->last_ecd = motor->ecd;
    /* ת�ӻ�е�Ƕ� */
    motor->ecd = (uint16_t)(data[0]<<8 | data[1]);
    /* ת��ת�� */
    motor->speed_rpm = (int16_t)(data[2]<<8 | data[3]);
    /* ʵ��Ť�ص��� */
    motor->given_current = (int16_t)(data[4]<<8 | data[5]);
    /* ����¶� */
    motor->temperate = data[6];
}

/**
 * @brief ����DJI���ת��Ȧ��, ����ܱ���ֵ�ļ���, ������������
 * @param[in] motor  ����ṹ��ָ��
 */
void DJI_Round_Count(DJI_Motor_t *motor) {
    if(motor->ecd - motor->last_ecd > ECD180){
        motor->round_cnt--;
    }
    else if(motor->ecd - motor->last_ecd < -ECD180)
    {
        motor->round_cnt++;
    }
    motor->total_ecd = motor->round_cnt*ECD360 + (motor->ecd - motor->offset_ecd);
}

/**
 * @brief DJI����������޷�
 * @param[in] ecd  ����ֵ
 * @return  ����ֵ�޷����ֵ
 */
fp32 DJI_Encoder_Limit(int16_t ecd) {
    while(ecd < 0 || ecd > ECD360) {
        if(ecd < 0) {
            ecd += ECD360;
        } else if(ecd > ECD360) {
            ecd -= ECD360;
        }
    }
    return (fp32)ecd;
}


/**
 * ����������Ķ���  -180-180
 * ���ݵ�����������ݺ�ƫ����������ԽǶȱ仯�ĺ���
 */
fp32 Motor_Ecd_To_Angle_Change(uint16_t ecd, uint16_t offset_ecd) {
    int16_t tmp = 0;
    if(offset_ecd >= ECD180) {
        if(ecd > offset_ecd - ECD180) {
            tmp = ecd - offset_ecd;
        }
        else {
            tmp = ecd + ECD360 - offset_ecd;
        }
    }
    else {
        if(ecd > offset_ecd + ECD180) {
            tmp = ecd - ECD360 - offset_ecd;
        }
        else {
            tmp = ecd - offset_ecd;
        }
    }
    return (fp32)(tmp / 8192.f * 360);//TODO:����Ҫ������
}

/**
 * @brief ͨ��CAN����DJI����ĵ���
 * @param[in] can      CANͨ��
 * @param[in] can_id   ����֡ID
 * @param[in] data     ���͵�����
 */
void DJI_Send_Motor_Mapping(CAN_TYPE can, uint32_t can_id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t can_send_data[8];
    tx_message.StdId = can_id;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = motor1 >> 8;
    can_send_data[1] = motor1;
    can_send_data[2] = motor2 >> 8;
    can_send_data[3] = motor2;
    can_send_data[4] = motor3 >> 8;
    can_send_data[5] = motor3;
    can_send_data[6] = motor4 >> 8;
    can_send_data[7] = motor4;

    if(can == CAN_1) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
    } else if(can == CAN_2) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
    }
}