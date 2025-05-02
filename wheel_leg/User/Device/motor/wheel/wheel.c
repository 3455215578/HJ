#include "wheel.h"


Lk9025 wheel[2];

/** ��챵����ʼ�� **/
void wheel_init(void)
{
    lk9025_init(&wheel[L], WHEEL_L_RECEIVE);
    lk9025_init(&wheel[R], WHEEL_R_RECEIVE);
}

/** ������챵��ָ�� **/
Lk9025* get_wheel_motors(void){
    return wheel;
}