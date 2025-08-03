//
// Created by laiyo on 25-3-31.
//
#ifndef _SMC_H_
#define _SMC_H_

#include <stdint-gcc.h>
#include "math.h"

// ����ϵͳ״̬��ת������������
typedef struct {
    double error;
    double error_deriv;
    double previous_error;  // ÿ��������������ϴ����
} SystemState;

// ���廬ģ����������
typedef struct {
    double c;          // ��ģ��ϵ��
    double rho;        // �л�����
    double epsilon;    // �߽���ȣ����ڱ��ͺ�����
    double max_i;      // ���������޷�ֵ
} SMC_Params;

// // ������������
// typedef struct {
//     double u;          // �������루����PWMռ�ձȣ�
//     double sigma;      // ��ģ��ֵ
// } SMC_Output;


// ��ģ���ƺ���
double smc_controller(SystemState state, SMC_Params params);

// Ħ����ϵͳ״̬����
SystemState update_system(float speed, int16_t speed_rpm, double dt, SystemState prev_state);

#endif