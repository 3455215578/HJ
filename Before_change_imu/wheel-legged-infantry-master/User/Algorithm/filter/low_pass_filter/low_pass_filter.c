#include "low_pass_filter.h"

void low_pass_filter_init(LowPassFilter *filter, float alpha) {
    // ��ʼ���˲�����ϵ��
    filter->alpha = alpha;

    // ��ʼ����һʱ�̵�ֵΪ0
    filter->prev_value = 0;
}

float update_low_pass_filter(LowPassFilter *filter, float value) {
    // �����˲����ֵ
    float filted_value = filter->alpha * value + (1 - filter->alpha) * filter->prev_value;

    // ������һʱ�̵�ֵ
    filter->prev_value = filted_value;

    return filted_value;
}
