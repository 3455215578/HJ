#include "bsp_can.h"
#include "main.h"
#include "detect_task.h"
#include "DJI_Motor.h"
#include "gimbal_task.h"
#include "launcher.h"

/* ���սṹ�� */
CAN_RxFrame_TypeDef CAN1_RxFrame;
CAN_RxFrame_TypeDef CAN2_RxFrame;

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


//**����STM32��HAL���CAN���裬���䲻Ϊ�յ�����ͨ����ͨ����鴫��״̬�Ĵ�����TSR���е��ض�λ��ȷ���ġ�
// ������˵��ÿ�����䶼��һ����Ӧ�ġ���������ա���Transmit Mailbox Empty��TME��λ������λΪ0ʱ����ʾ���䲻Ϊ�գ�����λΪ1ʱ����ʾ����Ϊ���ҿ��á�

/** Ѱ�Ҳ�Ϊ�յ����� **/
uint32_t get_can_free_mail(CAN_HandleTypeDef* hcan)
{
    if ((hcan->Instance->TSR & CAN_TSR_TME0) != RESET){
        return CAN_TX_MAILBOX0;
    }
    else if ((hcan->Instance->TSR & CAN_TSR_TME1) != RESET){
        return CAN_TX_MAILBOX1;
    }
    else if ((hcan->Instance->TSR & CAN_TSR_TME2) != RESET){
        return CAN_TX_MAILBOX2;
    }
    else{
        return 0;
    }
}

/** CAN1�����жϴ��� **/
static void CAN1_RxFifo0RxHandler(uint32_t *StdId, uint8_t Data[])
{
    switch (CAN1_RxFrame.Header.StdId) {

        case CAN_LAUNCHER_FIRE_L: //201
        {
            DJI_Motor_Decode(&launcher.fire_l.motor_measure, Data);
            detect_handle(DETECT_LAUNCHER_3508_FIRE_L);
            break;
        }

        case CAN_LAUNCHER_FIRE_R: //202
        {
            DJI_Motor_Decode(&launcher.fire_r.motor_measure, Data);
            detect_handle(DETECT_LAUNCHER_3508_FIRE_R);
            break;
        }

        case CAN_LAUNCHER_TRIGGER: //203
        {
            DJI_Motor_Decode(&launcher.trigger.motor_measure, Data);
            DJI_Round_Count(&launcher.trigger.motor_measure);//��ȡת�����ֵ��ת��Ȧ�����ܱ���ֵ
            detect_handle(DETECT_LAUNCHER_3508_TRIGGER);
            break;
        }

        case CAN_GIMBAL_YAW: //205
        {
            DJI_Motor_Decode(&gimbal.yaw.motor_measure, Data);
            detect_handle(DETECT_GIMBAL_6020_YAW);
            break;
        }
        case CAN_GIMBAL_PITCH: //206
        {
            DJI_Motor_Decode(&gimbal.pitch.motor_measure, Data);
            detect_handle(DETECT_GIMBAL_6020_PITCH);
            break;
        }
        default:
        {
            break;
        }
    }
}

/** CAN2�����жϴ��� **/
static void CAN2_RxFifo0RxHandler(uint32_t *StdId, uint8_t Data[])
{
    switch(CAN2_RxFrame.Header.StdId) {
        default:
        {
            break;
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

    if (hcan == &hcan1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_RxFrame.Header, CAN1_RxFrame.Data) == HAL_OK)
        {
            CAN1_RxFifo0RxHandler(&CAN1_RxFrame.Header.StdId, CAN1_RxFrame.Data);
        }
    }
    else if (hcan == &hcan2)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN2_RxFrame.Header, CAN2_RxFrame.Data) == HAL_OK)
        {
            CAN2_RxFifo0RxHandler(&CAN2_RxFrame.Header.StdId, CAN2_RxFrame.Data);
        }
    }
}

