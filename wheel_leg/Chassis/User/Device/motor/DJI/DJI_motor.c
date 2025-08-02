#include "DJI_motor.h"


/**
 * @brief DJI�������
 * @param[in] motor  ����ṹ��ָ��
 * @param[in] data   ���յ������ݵ�ָ��
 */
void DJI_info_update(DJI_Motor_t *motor, uint8_t *data) {
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
float DJI_Encoder_Limit(int16_t ecd) {
    while(ecd < 0 || ecd > ECD360) {
        if(ecd < 0) {
            ecd += ECD360;
        } else if(ecd > ECD360) {
            ecd -= ECD360;
        }
    }
    return (float)ecd;
}


/**
 * ����������Ķ���  -180-180
 * ���ݵ�����������ݺ�ƫ����������ԽǶȱ仯�ĺ���
 */
float Motor_Ecd_To_Angle_Change(uint16_t ecd, uint16_t offset_ecd) {
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
    return (float)(tmp / 8192.f * 360);//TODO:����Ҫ������
}