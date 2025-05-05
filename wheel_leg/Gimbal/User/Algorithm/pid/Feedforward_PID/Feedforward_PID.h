//
// Created by laiyo on 24-12-17.
//

#ifndef FEEDFORWARD_PID_H
#define FEEDFORWARD_PID_H

#include "struct_typedef.h"
#include "math.h"
#include "user_lib.h"

// 因为是跟随时域分离的所以命名为TIME
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

    fp32 parameters[PARAMS];        // 控制器参数
    fp32 error[TIME];               // 误差
    fp32 reference;                 // 目标值
    fp32 particle_p;                // 比例输出
    fp32 particle_i;                // 积分输出
    fp32 particle_d;                // 微分输出
    fp32 output;                    // 输出值

}PID_t;

typedef struct Variable_Speed_Proportion_PID_Controller {

    PID_t controller;
    // 变速比例因子增益，计算公式为: kp = k0*lg(k1*abs(error))+k2
    fp32 k[3];
    // 最小比例因子增益，避免误差过小导致的比例因子增益为0造成的震荡
    fp32 minimum_kp;

}VSP_PID_t;

/**
 * @brief PID初始化
 * @param[in] PID         PID结构体指针
 * @param[in] parameters  PID参数数组头指针
 */
void PID_Init(PID_t *PID, const fp32 parameters[PARAMS]);

/**
     * @brief 变速比例因子PID初始化
     * @param[in] VSP         变速比例因子PID结构体指针
     * @param[in] parameters  PID参数数组
     * @param[in] k0          比例因子增益1
     * @param[in] k1          比例因子增益2
     * @param[in] k2          比例因子增益3
     * @param[in] min_kp      最小比例因子增益
     */
void VSP_PID_Init(VSP_PID_t *VSP, const fp32 parameters[PARAMS], fp32 k0, fp32 k1, fp32 k2, fp32 min_kp);

/**
 * @brief 变速比例因子PID计算
 * @param[in] VSP         变速比例因子PID结构体指针
 * @param[in] feedback    反馈值
 * @param[in] reference   目标值
 * @retval                输出值
 */
fp32 VSP_PID_Calc(VSP_PID_t *VSP, fp32 feedback, fp32 reference);

#endif //FEEDFORWARD_PID_H