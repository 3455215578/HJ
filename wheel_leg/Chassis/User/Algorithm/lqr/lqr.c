#include "lqr.h"
#include "math.h"

/** LQR **/
// K拟合系数矩阵
float chassis_fitting_factor[40][6] = {};

// 初始化K矩阵
float wheel_K_L[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float wheel_K_R[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float joint_K_L[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float joint_K_R[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


/** 拟合K **/
void chassis_K_matrix_fitting(float L0_l, float L0_r, const float K_fitting_factor[40][6])
{
    /** Wheel **/
    for (int i = 0; i < 20; i++)
    {
        // Left
        if(i < 10)
        {
            wheel_K_L[i] =  K_fitting_factor[i][0] * powf(L0_l, 0)
                          + K_fitting_factor[i][1] * powf(L0_l, 1)
                          + K_fitting_factor[i][2] * powf(L0_l, 2)
                          + K_fitting_factor[i][3] * powf(L0_r, 1)
                          + K_fitting_factor[i][4] * powf(L0_r, 2)
                          + K_fitting_factor[i][5] * L0_l * L0_r;
        }
        // Right
        else
        {
            wheel_K_R[i - 10] =  K_fitting_factor[i][0] * powf(L0_l, 0)
                               + K_fitting_factor[i][1] * powf(L0_l, 1)
                               + K_fitting_factor[i][2] * powf(L0_l, 2)
                               + K_fitting_factor[i][3] * powf(L0_r, 1)
                               + K_fitting_factor[i][4] * powf(L0_r, 2)
                               + K_fitting_factor[i][5] * L0_l * L0_r;
        }

    }


    /** Joint**/
    for (int i = 20; i < 40; i++)
    {
        // Left
        if(i < 30)
        {
            joint_K_L[i - 20] =  K_fitting_factor[i][0] * powf(L0_l, 0)
                               + K_fitting_factor[i][1] * powf(L0_l, 1)
                               + K_fitting_factor[i][2] * powf(L0_l, 2)
                               + K_fitting_factor[i][3] * powf(L0_r, 1)
                               + K_fitting_factor[i][4] * powf(L0_r, 2)
                               + K_fitting_factor[i][5] * L0_l * L0_r;
        }
        // Right
        else
        {
            joint_K_R[i - 30] =  K_fitting_factor[i][0] * powf(L0_l, 0)
                               + K_fitting_factor[i][1] * powf(L0_l, 1)
                               + K_fitting_factor[i][2] * powf(L0_l, 2)
                               + K_fitting_factor[i][3] * powf(L0_r, 1)
                               + K_fitting_factor[i][4] * powf(L0_r, 2)
                               + K_fitting_factor[i][5] * L0_l * L0_r;
        }

    }

}
