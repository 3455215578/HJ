#ifndef VMC_H
#define VMC_H

#include "robot_def.h"

void vmc_forward_kinematics(Leg* leg, ChassisPhysicalConfig *physical_config);

void vmc_forward_dynamics(VMC *vmc, const ChassisPhysicalConfig *physical_config);

void Inverse_Kinematics(VMC *vmc,
                        float w1,
                        float w4,
                        ChassisPhysicalConfig *chassis_physical_config);

void Inverse_Dynamics(VMC *vmc,
                      float T1,
                      float T4,
                      ChassisPhysicalConfig *chassis_physical_config);

void fn_cal(Leg *leg, float body_az, ChassisPhysicalConfig *chassis_physical_config);

#endif
