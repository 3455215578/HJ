//
// Created by laiyo on 24-12-17.
//

#ifndef FEEDFORWARD_PID_H
#define FEEDFORWARD_PID_H

#include "struct_typedef.h"
#include "math.h"
#include "user_lib.h"

// ��Ϊ�Ǹ���ʱ��������������ΪTIME
enum{
    PREV = 0,
    CURR = 1,
    TIME
};

enum {
    KP = 0,
    KI = 1,
    KD = 2,
    MAX_INTEGRAL = 3,
    MAX_OUTPUT = 4,
    PARAMS
};

typedef struct PID_Controller {

    fp32 parameters[PARAMS];        // ����������
    fp32 error[TIME];               // ���
    fp32 reference;                 // Ŀ��ֵ
    fp32 particle_p;                // �������
    fp32 particle_i;                // �������
    fp32 particle_d;                // ΢�����
    fp32 output;                    // ���ֵ

}PID_t;

typedef struct Variable_Speed_Proportion_PID_Controller {

    PID_t controller;
    // ���ٱ����������棬���㹫ʽΪ: kp = k0*lg(k1*abs(error))+k2
    fp32 k[3];
    // ��С�����������棬��������С���µı�����������Ϊ0��ɵ���
    fp32 minimum_kp;

}VSP_PID_t;

/**
 * @brief PID��ʼ��
 * @param[in] PID         PID�ṹ��ָ��
 * @param[in] parameters  PID��������ͷָ��
 */
void PID_Init(PID_t *PID, const fp32 parameters[PARAMS]);

/**
     * @brief ���ٱ�������PID��ʼ��
     * @param[in] VSP         ���ٱ�������PID�ṹ��ָ��
     * @param[in] parameters  PID��������
     * @param[in] k0          ������������1
     * @param[in] k1          ������������2
     * @param[in] k2          ������������3
     * @param[in] min_kp      ��С������������
     */
void VSP_PID_Init(VSP_PID_t *VSP, const fp32 parameters[PARAMS], fp32 k0, fp32 k1, fp32 k2, fp32 min_kp);

/**
 * @brief ���ٱ�������PID����
 * @param[in] VSP         ���ٱ�������PID�ṹ��ָ��
 * @param[in] feedback    ����ֵ
 * @param[in] reference   Ŀ��ֵ
 * @retval                ���ֵ
 */
fp32 VSP_PID_Calc(VSP_PID_t *VSP, fp32 feedback, fp32 reference);

#endif //FEEDFORWARD_PID_H