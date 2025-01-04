#ifndef VMC_H
#define VMC_H

/*******************************************************************************
 *                            FK (Forward Kinematics)                                *
 *******************************************************************************/
struct FKL0 {
  float L0;
  float L0_last;
  float L0_dot;
  float L0_dot_last;
  float L0_ddot;
};

struct FKPhi {//The phi Angle in the five-link
  float phi1;
  float phi2;
  float phi3;
  float phi4;


  float phi0;
  float last_phi0;
  float d_phi0;// �ڽǱ仯�ٶ�
  float last_d_phi0;
  float dd_phi0;
};

struct FKPointCoordinates {
  float a_x, a_y;
  float b_x, b_y;
  float c_x, c_y;
  float d_x, d_y;
  float e_x, e_y;
};

struct ForwardKinematics {
  struct FKL0 fk_L0;
  struct FKPhi fk_phi;
  struct FKPointCoordinates fk_point_coordinates;

  float d_alpha;
};

struct VMC {
  struct ForwardKinematics forward_kinematics;
  union {
    float array[2][1];
    struct {
      float w1_fdb;// �ؽڵ�����ٶ�
      float w4_fdb;
    } E;
  } W_fdb;

  union {
    float array[2][1];
    struct {
      float d_L0_fdb; // �ȳ��仯�ٶ�
      float d_phi0_fdb; // �ڽ�(phi0)�仯�ٶ�

    } E;
  } V_fdb;

  union {
    float array[2][1];
    struct {
      float T1_fdb;
      float T4_fdb;
    } E;
  } T1_T4_fdb;

  union {
    float array[2][1];
    struct {
      float T1_set_point;
      float T4_set_point;
    } E;
  } T1_T4_set_point;

  union {
    float array[2][1];
    struct {
      float Tp_fdb;
      float Fy_fdb;
    } E;
  } Fxy_fdb;

  union {
    float array[2][2];
    struct {
      float Tp_set_point;
      float Fy_set_point;
    } E;
  } Fxy_set_point;

  union {
    float array[2][2];
    struct {
      float x1_1;
      float x1_2;
      float x2_1;
      float x2_2;
    } E;
  } J_w_to_v;

  union {
    float array[2][2];
    struct {
      float x1_1;
      float x1_2;
      float x2_1;
      float x2_2;
    } E;
  } J_F_to_T;

  union {
    float array[2][2];
    struct {
      float x1_1;
      float x1_2;
      float x2_1;
      float x2_2;
    } E;
  } J_T_to_F;
};

/*******************************************************************************
 *                           Inverse Kinematics                                *
 *******************************************************************************/
struct Chassis;
struct ChassisPhysicalConfig;
void vmc_ctrl(struct Chassis *chassis,const struct ChassisPhysicalConfig *chassis_physical_config);

#endif //VMC_H
