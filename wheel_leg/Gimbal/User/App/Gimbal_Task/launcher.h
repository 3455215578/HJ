#ifndef LAUNCHER_H
#define LAUNCHER_H

#include "DJI_Motor.h"
#include "user_lib.h"
#include "PID.h"
#include "SMC.h"

extern void Launcher_Init(void);
extern void Launcher_Mode_Set(void);
extern void Launcher_Control(void);
extern void Launcher_Disable(void);

#endif