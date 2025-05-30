 #ifndef _LQR_H
#define _LQR_H

extern float wheel_K_L[10];
extern float joint_K_L[10];
extern float wheel_K_R[10];
extern float joint_K_R[10];

// KÄâºÏÏµÊı¾ØÕó
extern float chassis_fitting_factor[40][6];


 void chassis_K_matrix_fitting(float L0_l, float L0_r, const float K_fitting_factor[40][6]);

#endif
