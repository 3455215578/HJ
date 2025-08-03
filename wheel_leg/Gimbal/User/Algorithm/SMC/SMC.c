#include "SMC.h"


// ��ģ���ƺ���
double smc_controller(SystemState state, SMC_Params params) {
    // 1. ���㻬ģ��
    double sigma = state.error_deriv + params.c * state.error;

    // 2. ʹ�ñ��ͺ���������ź��������ƶ���
    double sat_value;
    if (fabs(sigma) <= params.epsilon) {
        sat_value = sigma / params.epsilon;
    } else {
        sat_value = (sigma > 0) ? 1.0 : -1.0;
    }

    // 3. �����������
    double output = -params.c * state.error - params.rho * sat_value;

    // 4. ���������޷�
    if (output > params.max_i) {
        output= params.max_i;
    } else if (output < -params.max_i) {
        output = -params.max_i;
    }

    return output;
}

// Ħ����ϵͳ״̬����
SystemState update_system(float speed, int16_t speed_rpm, double dt, SystemState prev_state) {
    SystemState new_state;
    new_state.error = speed - speed_rpm;
    new_state.error_deriv = (new_state.error - prev_state.previous_error) / dt;
    new_state.previous_error = new_state.error;  // ����Ϊ��ǰ���
    return new_state;
}