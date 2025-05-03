#include "buzzer.h"
#include "tim.h"
#include "cmsis_os.h"


#define System_Fequency 170000000

Buzzer_SoundEffect_EnumTypedef Buzzer_SoundEffect;

void Buzzer_Init(void)
{
    /*===| ����������PWM |===*/
    HAL_TIM_Base_Start(&htim4);
}

void Buzzer_Start(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void Buzzer_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}

void Gala_You(void)
{

    /** ��һֱ׷Ѱ���� **/
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // һ
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ֱ
    Buzzer_Set_Tone(M2); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ׷~
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // Ѱ
    Buzzer_Set_Tone(M2); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M5); osDelay(1000); Buzzer_Set_Tone(P); osDelay(200);

    /** �����ԶҲ���� **/
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M2); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // Զ
    Buzzer_Set_Tone(M2); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // Ҳ
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��~
    Buzzer_Set_Tone(M5); osDelay(1000); Buzzer_Set_Tone(P); osDelay(100);
    // ��
    Buzzer_Set_Tone(M1); osDelay(1000); Buzzer_Set_Tone(P); osDelay(100);

    /** ȴ�ܱ����ž��� **/
    // ȴ
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(100);
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��~
    Buzzer_Set_Tone(M2); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��~
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M2); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M1); osDelay(500); Buzzer_Set_Tone(P); osDelay(100);
    // ��
    Buzzer_Set_Tone(M2); osDelay(1000); Buzzer_Set_Tone(P); osDelay(100);

    /** ��һֱ�������� **/
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // һ
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ֱ
    Buzzer_Set_Tone(M2); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��~
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M2); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M5); osDelay(1000); Buzzer_Set_Tone(P); osDelay(200);

    /** ������� ���һ��� **/
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M2); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M2); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��
    Buzzer_Set_Tone(M3); osDelay(500); Buzzer_Set_Tone(P); osDelay(50);
    // ��~
    Buzzer_Set_Tone(H1); osDelay(1000); Buzzer_Set_Tone(P); osDelay(100);
    // ��
    Buzzer_Set_Tone(M1); osDelay(1000); Buzzer_Set_Tone(P); osDelay(100);

}

/**
 * @brief ���÷�������Ч
 */
void Buzzer_Set_SoundEffect(Buzzer_SoundEffect_EnumTypedef SoundEffect)
{
    Buzzer_SoundEffect = SoundEffect;
}

/**
 * @brief ���÷���������
 */
void Buzzer_Set_Tone(Buzzer_Tone_EnumTypedef Tone)
{
    uint16_t PSC;
    uint32_t ARL;

    switch((uint8_t)Tone)
    {
        case P:  PSC = 50;    ARL = 100;  break;
        case L1: PSC = 50;    ARL = System_Fequency / PSC / 262;  break;
        case L2: PSC = 50;    ARL = System_Fequency / PSC / 294;  break;
        case L3: PSC = 50;    ARL = System_Fequency / PSC / 330;  break;
        case L4: PSC = 50;    ARL = System_Fequency / PSC / 349;  break;
        case L5: PSC = 50;    ARL = System_Fequency / PSC / 392;  break;
        case L6: PSC = 50;    ARL = System_Fequency / PSC / 440;  break;
        case L7: PSC = 50;    ARL = System_Fequency / PSC / 494;  break;
        case M1: PSC = 50;    ARL = System_Fequency / PSC / 523;  break;
        case M2: PSC = 50;    ARL = System_Fequency / PSC / 587;  break;
        case M3: PSC = 50;    ARL = System_Fequency / PSC / 659;  break;
        case M4: PSC = 50;    ARL = System_Fequency / PSC / 698;  break;
        case M5: PSC = 50;    ARL = System_Fequency / PSC / 784;  break;
        case M6: PSC = 50;    ARL = System_Fequency / PSC / 880;  break;
        case M7: PSC = 50;    ARL = System_Fequency / PSC / 988;  break;
        case H1: PSC = 50;    ARL = System_Fequency / PSC / 1046; break;
        case H2: PSC = 50;    ARL = System_Fequency / PSC / 1175; break;
        case H3: PSC = 50;    ARL = System_Fequency / PSC / 1318; break;
        case H4: PSC = 50;    ARL = System_Fequency / PSC / 1397; break;
        case H5: PSC = 50;    ARL = System_Fequency / PSC / 1568; break;
        case H6: PSC = 50;    ARL = System_Fequency / PSC / 1760; break;
        case H7: PSC = 50;    ARL = System_Fequency / PSC / 1976; break;
    }
    __HAL_TIM_SET_PRESCALER(&htim4, PSC - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim4, ARL - 1);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, ARL / 2);
}

