#include "vx_kalman_filter.h"

#include "robot_def.h"
#include "wheel.h"

uint16_t sizeof_float;

extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;

void kalman_Filter_Init(KalmanFilter* kf, uint8_t xSize, uint8_t uSize, uint8_t zSize, uint8_t ySize){
  sizeof_float = sizeof(float);

//    kf->dt = delta_t;
  kf->xSize = xSize;
  kf->uSize = uSize;
  kf->zSize = zSize;//这个一般都是1
  kf->ySize = ySize;

  Matrix_Init(&kf->F, kf->xSize, kf->xSize, (float *)kf->F_data);

  Matrix_Init(&kf->H, kf->ySize, kf->xSize, (float *)kf->H_data);

  Matrix_Init(&kf->R, kf->ySize, kf->ySize, (float *)kf->R_data);

  Matrix_Init(&kf->Q, kf->xSize, kf->xSize, (float *)kf->Q_data);

  Matrix_Init(&kf->Xk, kf->xSize, kf->zSize, (float *)kf->Xk_data);

  kf->Xk_1_data=(float*)user_malloc(sizeof_float*xSize*zSize);
  memset(kf->Xk_1_data,0,sizeof_float*xSize*zSize);
  Matrix_Init(&kf->Xk_1, kf->xSize, kf->zSize, (float *)kf->Xk_1_data);

  kf->k_data=(float*)user_malloc(sizeof_float*xSize*ySize);
  memset(kf->k_data,0,sizeof_float*xSize*ySize);
  Matrix_Init(&kf->K, kf->xSize, kf->ySize, (float *)kf->k_data);

  Matrix_Init(&kf->sigma, kf->xSize, kf->xSize, (float *)kf->sigma_data);

  Matrix_Init(&kf->sigma_minus, kf->xSize, kf->xSize, (float *)kf->sigma_minus_data);

  kf->FT_data=(float*)user_malloc(sizeof_float*xSize*xSize);
  memset(kf->FT_data,0,sizeof_float*xSize*xSize);
  Matrix_Init(&kf->FT, kf->xSize, kf->xSize, (float *)kf->FT_data);

  kf->HT_data=(float*)user_malloc(sizeof_float*xSize*ySize);
  memset(kf->HT_data,0,sizeof_float*xSize*ySize);
  Matrix_Init(&kf->HT, kf->xSize, kf->ySize, (float *)kf->HT_data);

  kf->tempmat_data=(float*)user_malloc(sizeof_float*ySize*xSize);
  memset(kf->tempmat_data,0,sizeof_float*ySize*xSize);
  Matrix_Init(&kf->tempmat, kf->ySize, kf->xSize, (float *)kf->tempmat_data);

  kf->tempmat1_data=(float*)user_malloc(sizeof_float*ySize*ySize);
  memset(kf->tempmat1_data,0,sizeof_float*ySize*ySize);
  Matrix_Init(&kf->tempmat1, kf->ySize, kf->ySize, (float *)kf->tempmat1_data);

  kf->tempmat6_data=(float*)user_malloc(sizeof_float*ySize*ySize);
  memset(kf->tempmat6_data,0,sizeof_float*ySize*ySize);
  Matrix_Init(&kf->tempmat6, kf->ySize, kf->ySize, (float *)kf->tempmat6_data);

  kf->tempmat2_data=(float*)user_malloc(sizeof_float*xSize*ySize);
  memset(kf->tempmat2_data,0,sizeof_float*xSize*ySize);
  Matrix_Init(&kf->tempmat2, kf->xSize, kf->ySize, (float *)kf->tempmat2_data);

  kf->tempmat3_data=(float*)user_malloc(sizeof_float*ySize*zSize);
  memset(kf->tempmat3_data,0,sizeof_float*ySize*zSize);
  Matrix_Init(&kf->tempmat3, kf->ySize, kf->zSize, (float *)kf->tempmat3_data);

  kf->tempmat7_data=(float*)user_malloc(sizeof_float*ySize*zSize);
  memset(kf->tempmat7_data,0,sizeof_float*ySize*zSize);
  Matrix_Init(&kf->tempmat7, kf->ySize, kf->zSize, (float *)kf->tempmat7_data);

  kf->tempmat4_data=(float*)user_malloc(sizeof_float*xSize*xSize);
  memset(kf->tempmat4_data,0,sizeof_float*xSize*xSize);
  Matrix_Init(&kf->tempmat4, kf->xSize, kf->xSize, (float *)kf->tempmat4_data);

  kf->tempmat5_data=(float*)user_malloc(sizeof_float*xSize*xSize);
  memset(kf->tempmat5_data,0,sizeof_float*xSize*xSize);
  Matrix_Init(&kf->tempmat5, kf->xSize, kf->xSize, (float *)kf->tempmat5_data);

  kf->tempmat8_data=(float*)user_malloc(sizeof_float*xSize*zSize);
  memset(kf->tempmat8_data,0,sizeof_float*xSize*zSize);
  Matrix_Init(&kf->tempmat8, kf->xSize, kf->zSize, (float *)kf->tempmat8_data);

//    kf->measure_data=(float*)user_malloc(sizeof_float*ySize*zSize);
//    memset(kf->measure_data,0,sizeof_float*ySize*zSize);
//    Matrix_Init(&kf->measure, kf->ySize, kf->zSize, (float *)kf->measure_data);

  kf->E_data=(float*)user_malloc(sizeof_float*xSize*xSize);
  memset(kf->E_data,0,sizeof_float*xSize*xSize);
  uint8_t i=0;
  while(i<xSize*xSize){
    kf->E_data[i]=1;
    i=i+xSize+1;
  }
  Matrix_Init(&kf->E, kf->xSize, kf->xSize, (float *)kf->E_data);
//    measureSize=sizeof(kf->measure_data);
}

float* kalman_Filter_Predict(KalmanFilter* kf){
  Matrix_Multiply(&kf->F, &kf->Xk, &kf->Xk_1);
  return kf->Xk_1.pData;
}

float* kalman_Filter_Update(KalmanFilter* kf, float* measure_temp){
  //sigma_minus = F * sigma * F.transpose() + Q;
  Matrix_Multiply(&kf->F, &kf->sigma, &kf->tempmat4);
  Matrix_Transpose(&kf->F, &kf->FT);
  Matrix_Multiply(&kf->tempmat4, &kf->FT, &kf->tempmat5);
  Matrix_Add(&kf->tempmat5, &kf->Q, &kf->sigma);

  //K = sigma_minus * H.transpose() * (H * sigma * H.transpose() + R).inverse();
  Matrix_Transpose(&kf->H, &kf->HT);
  Matrix_Multiply(&kf->H, &kf->sigma, &kf->tempmat);
  Matrix_Multiply(&kf->tempmat, &kf->HT, &kf->tempmat1);
  Matrix_Add(&kf->tempmat1, &kf->R, &kf->tempmat6);
  Matrix_Inverse(&kf->tempmat6, &kf->tempmat1);
  Matrix_Multiply(&kf->sigma, &kf->HT, &kf->tempmat2);
  Matrix_Multiply(&kf->tempmat2, &kf->tempmat1, &kf->K);

  //Xk = Xk_1 + K * (measured - H * Xk_1);
  mat measure;
  Matrix_Init(&measure, kf->ySize, kf->zSize, (float *)measure_temp);
  Matrix_Multiply(&kf->H, &kf->Xk_1, &kf->tempmat3);
  Matrix_Subtract(&measure, &kf->tempmat3, &kf->tempmat7);
  Matrix_Multiply(&kf->K, &kf->tempmat7, &kf->tempmat8);
  Matrix_Add(&kf->Xk_1, &kf->tempmat8, &kf->Xk);

  //sigma = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * sigma_minus;
  Matrix_Multiply(&kf->K, &kf->H, &kf->tempmat4);
  Matrix_Subtract(&kf->E, &kf->tempmat4, &kf->tempmat5);
  Matrix_Multiply(&kf->tempmat5, &kf->sigma_minus, &kf->sigma);

//  for (uint8_t i = 0; i < kf->xSize; i++)
//  {
//    if (ABS(kf->Xk.pData[i] - kf->Xk_1.pData[i])>kf->restrict_Variance[i])
//      kf->Xk.pData[i] = kf->Xk.pData[i] - kf->Xk_1.pData[i] > 0 ?
//          kf->restrict_Variance[i] + kf->Xk_1.pData[i] : kf->restrict_Variance[i] - kf->Xk_1.pData[i] ;
//  }

  return kf->Xk.pData;
}

void chassis_kalman_init(KalmanFilter *kalman_filter) {
    float dt = 0.005f;//单位s，后续传入的初始值都是ms为单位

    float R_init[4] = {0.0025f, 0,
                       0, 1.0f};//观测误差
    float Q_init[4] = {dt * dt, dt,
                       dt, 1.0f};//process noise

//    float R_init[4] = {0.01f, 0, // 速度的测量噪声大，测量的置信度就小了，测量结果就失真了
//                       0, 2.5f};//观测误差
//    float R_init[4] = {0.01f, 0, // 速度的测量噪声大，测量的置信度就小了，测量结果就失真了
//                       0, 0.001f};//观测误差
//    float Q_init[4] = {0.00002f, 0,
//                       0, 0.8f};//process noise

    // 观测矩阵
    float H_init[4] = {1, 0,
                       0, 1};
    float Xk_init[2] = {0, 0};

    // 状态转移矩阵
    float F_init[4] = {1, dt,
                       0, 1};

    kalman_filter->H_data = (float *) malloc(sizeof(float) * 9);
    memset(kalman_filter->H_data, 0, sizeof(float));
    memcpy(kalman_filter->H_data, H_init, sizeof(H_init));

    kalman_filter->F_data = (float *) malloc(sizeof(float) * 9);
    memset(kalman_filter->F_data, 0, sizeof(float));
    memcpy(kalman_filter->F_data, F_init, sizeof(F_init));

    kalman_filter->R_data = (float *) malloc(sizeof(float) * 9);
    memset(kalman_filter->R_data, 0, sizeof(float));
    memcpy(kalman_filter->R_data, R_init, sizeof(R_init));

    kalman_filter->Q_data = (float *) malloc(sizeof(float) * 9);
    memset(kalman_filter->Q_data, 0, sizeof(float));
    memcpy(kalman_filter->Q_data, Q_init, sizeof(Q_init));

    kalman_filter->Xk_data = (float *) malloc(sizeof(float) * 9);
    memset(kalman_filter->Xk_data, 0, sizeof(float));
    memcpy(kalman_filter->Xk_data, Xk_init, sizeof(Xk_init));

    kalman_filter->sigma_data = (float *) malloc(sizeof(float) * 9);
    memset(kalman_filter->sigma_data, 0, sizeof(float));
    memcpy(kalman_filter->sigma_data, Q_init, sizeof(Q_init));

    kalman_filter->sigma_minus_data = (float *) malloc(sizeof(float) * 9);
    memset(kalman_filter->sigma_minus_data, 0, sizeof(float));
    memcpy(kalman_filter->sigma_minus_data, Q_init, sizeof(Q_init));

    kalman_Filter_Init(kalman_filter, 2, 0, 1, 2);
}

void chassis_vx_kalman_run(void) {
    kalman_Filter_Predict(&chassis.vx_kalman);

    float w_left = -chassis.leg_L.state_variable_feedback.phi_dot + chassis.leg_L.vmc.forward_kinematics.d_alpha + get_wheel_motors()[0].angular_vel;
    float w_right = -chassis.leg_R.state_variable_feedback.phi_dot + chassis.leg_R.vmc.forward_kinematics.d_alpha - get_wheel_motors()[1].angular_vel;

    float v_left = w_left * chassis_physical_config.wheel_radius + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_L.state_variable_feedback.theta_dot * cosf(chassis.leg_L.state_variable_feedback.theta) + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot * sinf(chassis.leg_L.state_variable_feedback.theta);
    float v_right = w_right * chassis_physical_config.wheel_radius + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_R.state_variable_feedback.theta_dot * cosf(chassis.leg_R.state_variable_feedback.theta) + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot * sinf(chassis.leg_R.state_variable_feedback.theta);

    float aver_v = (v_left - v_right) / 2.0f; // 取平均值(避免多次转圈之后动一下遥控器底盘会甩一下)

    chassis.kalman_measure[0] = aver_v;
    chassis.kalman_measure[1] = chassis.imu_reference.ax_filtered;

    float* vel_acc = kalman_Filter_Update(&chassis.vx_kalman, chassis.kalman_measure);

    chassis.leg_L.kalman_result = vel_acc;
    chassis.leg_R.kalman_result = vel_acc;
}

