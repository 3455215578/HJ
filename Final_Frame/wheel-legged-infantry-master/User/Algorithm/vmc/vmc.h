#ifndef VMC_H
#define VMC_H

#include "robot_def.h"

void forward_dynamics(VMC *vmc, const ChassisPhysicalConfig *physical_config);
void joint_vmc_ctrl(void);
void vmc_ctrl(void);

#endif //VMC_H
