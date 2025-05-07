//
// Created by Gularx on 2024/2/26.
//
#include "ramp.h"

#ifndef INC_BALANCE_H
#define INC_BALANCE_H

#define GIMBAL
// #define CHASSIS

/**** ģʽѡ�� ****/
typedef enum {
    GIMBAL_RELAX,   //��̨ʧ��
    GIMBAL_BACK,    //��ʧ�ܵ�ʹ��
    GIMBAL_ACTIVE,  //�õ�ң��������̨����Ŀ���,�����ж�
    GIMBAL_AUTO,    //��̨����ģʽ
    GIMBAL_FIRE,    //��̨��ģʽ
}Gimbal_Mode_e;

/* Ħ����״ֵ̬ */
/** ��Ϊ���ж�Ħ����ģʽ��ʱ���˳���жϲ���ģʽ�����԰�Ħ���ֿ���������������������ر�(�ݶ����������������ܻ�ֿ�����������) **/
typedef enum{
    Fire_OFF=0,             //��������ر�
    Fire_ON=1,              //�����������
}Fir_Wheel_Mode_e;

/* ����״ֵ̬ */
typedef enum{
    SHOOT_CLOSE=0,          //����ر�
    SHOOT_CONTINUE,           //����ָ��

    SHOOT_READY_TO_SINGLE,  //����ָ��
    SHOOT_BLOCK,            //������ת

    SHOOT_SINGLE,           //����ִ��
    SHOOT_INVERSING,        //��ת��

    SHOOT_OVER,             //�������
    SHOOT_FAIL              //���̻���
}Shoot_Cmd_e;


#endif //INC_BALANCE_H
