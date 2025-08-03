//
// Created by laiyo on 25-3-31.
//
#ifndef _SMC_H_
#define _SMC_H_

#include <stdint-gcc.h>
#include "math.h"

// 定义系统状态（转速误差和误差导数）
typedef struct {
    double error;
    double error_deriv;
    double previous_error;  // 每个电机独立保存上次误差
} SystemState;

// 定义滑模控制器参数
typedef struct {
    double c;          // 滑模面系数
    double rho;        // 切换增益
    double epsilon;    // 边界层厚度（用于饱和函数）
    double max_i;      // 控制输入限幅值
} SMC_Params;

// // 定义控制器输出
// typedef struct {
//     double u;          // 控制输入（如电机PWM占空比）
//     double sigma;      // 滑模面值
// } SMC_Output;


// 滑模控制函数
double smc_controller(SystemState state, SMC_Params params);

// 摩擦轮系统状态更新
SystemState update_system(float speed, int16_t speed_rpm, double dt, SystemState prev_state);

#endif