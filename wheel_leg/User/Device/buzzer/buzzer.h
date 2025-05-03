#ifndef _Buzzer_H
#define _Buzzer_H

#include "main.h"

//��������Ч��������10ms�����鲻Ҫ����30��
#define BUZZER_TASK_CONTROL_TIME 10

/*===| Ԥ����Чö�ٶ��� |===*/
typedef enum
{
    Buzzer_SoundEffect_OFF = 0,
    Buzzer_SoundEffect_SystemStart,
    Buzzer_SoundEffect_SuperCap_ON,
    Buzzer_SoundEffect_SuperCap_OFF,
    Buzzer_SoundEffect_Change,
} Buzzer_SoundEffect_EnumTypedef;

/*===| ����������ö�ٶ��� |===*/
typedef enum
{
    P = 0,
    L1, L2, L3, L4, L5, L6, L7,
    M1, M2, M3, M4, M5, M6, M7,
    H1, H2, H3, H4, H5, H6, H7,
} Buzzer_Tone_EnumTypedef;


void Buzzer_Init(void);

void Buzzer_Set_SoundEffect(Buzzer_SoundEffect_EnumTypedef SoundEffect);

void Buzzer_Set_Tone(Buzzer_Tone_EnumTypedef Tone);

void Gala_You(void);

#endif


