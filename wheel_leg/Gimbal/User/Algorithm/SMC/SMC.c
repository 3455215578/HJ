#include "SMC.h"


// 滑模控制函数
double smc_controller(SystemState state, SMC_Params params) {
    // 1. 计算滑模面
    double sigma = state.error_deriv + params.c * state.error;

    // 2. 使用饱和函数替代符号函数以抑制抖振
    double sat_value;
    if (fabs(sigma) <= params.epsilon) {
        sat_value = sigma / params.epsilon;
    } else {
        sat_value = (sigma > 0) ? 1.0 : -1.0;
    }

    // 3. 计算控制输入
    double output = -params.c * state.error - params.rho * sat_value;

    // 4. 控制输入限幅
    if (output > params.max_i) {
        output= params.max_i;
    } else if (output < -params.max_i) {
        output = -params.max_i;
    }

    return output;
}

// 摩擦轮系统状态更新
SystemState update_system(float speed, int16_t speed_rpm, double dt, SystemState prev_state) {
    SystemState new_state;
    new_state.error = speed - speed_rpm;
    new_state.error_deriv = (new_state.error - prev_state.previous_error) / dt;
    new_state.previous_error = new_state.error;  // 更新为当前误差
    return new_state;
}