//
// Created by xhuanc on 2021/11/14.
//

#ifndef DEMO1_AHRS_MIDDLEWARE_H
#define DEMO1_AHRS_MIDDLEWARE_H

#ifndef NULL
#define NULL 0
#endif

//����PI ֵ
#ifndef PI
#define PI 3.14159265358979f
#endif

//���� �Ƕ�(��)ת���� ���ȵı���
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

//���� ���� ת���� �Ƕȵı���
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

extern void AHRS_get_height(float *high);
extern void AHRS_get_latitude(float *latitude);
extern float AHRS_invSqrt(float num);
extern float AHRS_sinf(float angle);
extern float AHRS_cosf(float angle);
extern float AHRS_tanf(float angle);
extern float AHRS_asinf(float sin);
extern float AHRS_acosf(float cos);
extern float AHRS_atan2f(float y, float x);
#endif //DEMO1_AHRS_MIDDLEWARE_H
