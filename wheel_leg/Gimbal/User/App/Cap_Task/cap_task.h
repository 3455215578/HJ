//
// Created by Gularx on 2025/3/8.
//

#ifndef INC_2025_HERO_THREE_CAP_H
#define INC_2025_HERO_THREE_CAP_H

/*********************************************************************************************************
*                                              ����ͷ�ļ�
*********************************************************************************************************/
#include "cmsis_os.h"
#include "Referee.h"
#include "bsp_can.h"

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/
/* ����ʼ����һ��ʱ�� */
#define CAP_TASK_INIT_TIME 357



/*********************************************************************************************************
*                                              ö�ٽṹ�嶨��
*********************************************************************************************************/
/** ������ָ���յ��ݵ�����, ������ָ�����ݷ������� */

/** ���� ��ȫ��ʾ֡ 0x001 */
typedef enum {
    /* ��ȫ��ʾ֡ */
    SAFETY_FRAME=0X001,
    SAFE=0X00,              // ��ȫ
    WARN=0X01,              // ����
    RISK=0X02,              // ����
    DANGEROUS=0X03,         // Σ��
    IRREVERSIBLE=0X04,      // ����������
}CAP_SAFETY_ID_e;
typedef struct{
    uint8_t Firmware_err;       // �̼���������İ�ȫ�ȼ�
    uint8_t Can_err;            // CAN��������İ�ȫ�ȼ�
    uint8_t Overheat_err;       // ���ȱ��������İ�ȫ�ȼ�
    uint8_t Calibration_err;    // У׼��������İ�ȫ�ȼ�
    uint8_t Voltage_err;        // ��ѹ��������İ�ȫ�ȼ�
    uint8_t Current_err;        // ������������İ�ȫ�ȼ�
    uint8_t Power_err;          // ���ʴ�������İ�ȫ�ȼ�
    uint8_t Sampling_err;       // ������������İ�ȫ�ȼ�
} Cap_Safety_Data;


/** ���� ����֡֡ 0X003 ���ݵķ������� */
typedef enum {
    /* ����֡ */
    FEEDBACK_FRAME=0X003,
}CAP_FEEDBACK_ID_e;
typedef struct{
    uint16_t esr_v;         // �������ѹ
    uint8_t work_s1;        // ����ǿ��1
    uint8_t work_s2;        // ����ǿ��2
    uint16_t input_power;   // ��Դ���빦��
} Cap_Receive_Data;


/** ���� ����֡ 0X005 ��֡���ڿ������ϵ����һ��,��������Ϊ��ȫ�ȼ��ı仯�����ٴν��з��� */
typedef enum {
    /* ����֡ */
    READY_FRAME=0X005,
    CAP_INIT_RECEIVE_ENABLE=0xFF,   // �������Ѿ���, ���Խ���can�����շ�
    CAP_INIT_RECEIVE_DISABLE=0x00,  // ������������, ���ܵ�����can֡��������
}CAP_READY_ID_e;
typedef struct{
    uint8_t controller_state;
} Cap_Ready_Data;


/** ���� ��ʼ��֡ 0X002 ��֡ÿ���ϵ�������һ�� */
typedef enum {
    /* ��ʼ��֡ */
    INIT_FRAME=0X002,
    CAP_INIT_MODE_24V=0x00,      // ��������������1,��������߳���ѹΪ24V
    CAP_INIT_MODE_28V=0x01,      // ��������������2,��������߳���ѹΪ28V
    CAP_INIT_MODE_30V=0x02,      // ��������������3,��������߳���ѹΪ30V
}CAP_INIT_ID_e;

/** ���� ����֡ 0X004 */
typedef enum {
    /* ����֡ */
    CONTRAL_FRAME=0X004,
    SILENT=0X00,            // Silent ����ģʽ
    WORK=0X01,              // Work ����ģʽ
    CHARGE=0X02,            // Charge ����ģʽ
}CAP_CONTRAL_ID_e;
typedef enum {
    Exceed_DISABLE=0X00,    // Exceed ʧ��
    Exceed_ENABLE=0X01,     // Exceed ʹ��
}CAP_Exceed_ID_e;

/* ���н������� */
typedef struct {
    Cap_Receive_Data capReceiveData;    // ����֡������
    Cap_Safety_Data err;                // ��ȫ��ʾ����
    Cap_Ready_Data state;               // ����֡��״̬(can)
    uint8_t cap_mode;                   // ����״̬(ʧ��ʹ��)
    CAP_CONTRAL_ID_e mode;
} Bear_Cap;

extern Bear_Cap bearCap;

extern void Cap_Data_Get(uint32_t can_id, const uint8_t *rx_data);


#endif //INC_2025_HERO_THREE_CAP_H
