//
// Created by laiyo on 24-12-17.
//

#include "Feedforward_PID.h"

void PID_Init(PID_t *PID, const fp32 parameters[PARAMS]) {
    PID->parameters[KP] = parameters[KP];
    PID->parameters[KI] = parameters[KI];
    PID->parameters[KD] = parameters[KD];
    PID->parameters[MAX_INTEGRAL] = parameters[MAX_INTEGRAL];
    PID->parameters[MAX_OUTPUT] = parameters[MAX_OUTPUT];

    PID->error[CURR] = 0.0f;
    PID->error[PREV] = 0.0f;

    PID->reference = 0.0f;

    PID->particle_p = 0.0f;
    PID->particle_i = 0.0f;
    PID->particle_d = 0.0f;

    PID->output = 0.0f;
}

void VSP_PID_Init(VSP_PID_t *VSP, const fp32 parameters[PARAMS], fp32 k0, fp32 k1, fp32 k2, fp32 min_kp) {
    PID_Init(&VSP->controller, parameters);
    VSP->k[0] = k0;
    VSP->k[1] = k1;
    VSP->k[2] = k2;
    VSP->minimum_kp = min_kp;
}

fp32 VSP_PID_Calc(VSP_PID_t *VSP, fp32 feedback, fp32 reference) {
    VSP->controller.reference = reference;
    VSP->controller.error[PREV] = VSP->controller.error[CURR];
    VSP->controller.error[CURR] = reference - feedback;

    // 计算当前的比例因子
    VSP->controller.parameters[KP] = VSP->k[0] * log10f(VSP->k[1] * ABS(VSP->controller.error[CURR])) + VSP->k[2];
    // 对比例因子限幅
    if(VSP->controller.parameters[KP] < VSP->minimum_kp) {
        VSP->controller.parameters[KP] = VSP->minimum_kp;
    }

    VSP->controller.particle_p = VSP->controller.parameters[KP] * VSP->controller.error[CURR];
    VSP->controller.particle_i += VSP->controller.parameters[KI] * (VSP->controller.error[PREV] + VSP->controller.error[CURR]);
    VSP->controller.particle_d = VSP->controller.parameters[KD] * (VSP->controller.error[CURR] - VSP->controller.error[PREV]);

    fabs_limit(&VSP->controller.particle_i, VSP->controller.parameters[MAX_INTEGRAL]);
    VSP->controller.output = VSP->controller.particle_p + VSP->controller.particle_i + VSP->controller.particle_d;
    fabs_limit(&VSP->controller.output, VSP->controller.parameters[MAX_OUTPUT]);

    return VSP->controller.output;
}
