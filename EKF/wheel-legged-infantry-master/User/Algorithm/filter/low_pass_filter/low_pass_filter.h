#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

// �����ͨ�˲����ṹ��
typedef struct {
    float alpha;         // �˲�ϵ��
    float prev_value;   // ��һʱ�̵�ֵ
} LowPassFilter;

// ��ʼ����ͨ�˲���
void low_pass_filter_init(LowPassFilter *filter, float alpha);

// ���µ�ͨ�˲���״̬�������˲����ֵ
float update_low_pass_filter(LowPassFilter *filter, float value);

#endif

