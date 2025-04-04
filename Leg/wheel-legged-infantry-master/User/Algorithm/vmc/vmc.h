#ifndef VMC_H
#define VMC_H

#include "robot_def.h"

void vmc_forward_kinematics(Leg* leg, ChassisPhysicalConfig *physical_config);
void vmc_forward_dynamics(VMC *vmc, const ChassisPhysicalConfig *physical_config);
void vmc_ctrl(void);

#endif //VMC_H
