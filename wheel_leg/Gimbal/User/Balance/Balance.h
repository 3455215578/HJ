//
// Created by Gularx on 2024/2/26.
//
#include "ramp.h"

#ifndef INC_2024_HERO_HERO_H
#define INC_2024_HERO_HERO_H

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
typedef enum{
    Fire_OFF=0,             //��������ر�
    Fire_ON=1,              //�����������
}Fire_Mode_e;

/* ����״ֵ̬ */
typedef enum{
    SHOOT_CLOSE=0,          //����ر�
    SHOOT_READLY,           //����ָ��
    SHOOT_SINGLE,           //����ִ��
    SHOOT_OVER,             //�������
    SHOOT_BLOCK,            //������ת
    SHOOT_BLOCK_BACK,       //��ת��
    SHOOT_FAIL              //���̻���
}Shoot_Cmd_e;

/* ����״ֵ̬ */
typedef enum {
    CHASSIS_RELAX,          //����ʧ�� ��������
    CHASSIS_ONLY,           //���̶��� ���� E.click_flag==1
    CHASSIS_FIRE,           //���̸�����̨ ����
    CHASSIS_FOLLOW_GIMBAL,  //���̸�����̨ ����
    CHASSIS_SPIN_R,           //С����
    CHASSIS_SPIN_L,           //С����
} Chassis_Mode_e;

/**** �������� ****/


#endif //INC_2024_HERO_HERO_H
