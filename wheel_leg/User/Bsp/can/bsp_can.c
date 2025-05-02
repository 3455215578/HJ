#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>
#include "bsp_can.h"

#include "wheel.h"
#include "joint.h"

/* �ؽڷ��ͽṹ�� */
CAN_TxFrame_TypeDef JointTxFrame = {

        .hcan = &hcan1,

        .Header.StdId = 0x00,
        .Header.ExtId = 0,
        .Header.IDE = CAN_ID_STD,
        .Header.RTR = CAN_RTR_DATA,
        .Header.DLC = 0x08,
        .Header.TransmitGlobalTime = DISABLE,
};

/* ��챷��ͽṹ�� */
CAN_TxFrame_TypeDef WheelTxFrame = {

        .hcan = &hcan2,

        .Header.StdId = 0x280,
        .Header.ExtId = 0,
        .Header.IDE = CAN_ID_STD,
        .Header.RTR = CAN_RTR_DATA,
        .Header.DLC = 0x08,
        .Header.TransmitGlobalTime = DISABLE,
};

/* ���սṹ�� */
CAN_RxFrame_TypeDef CAN1_RxFrame;
CAN_RxFrame_TypeDef CAN2_RxFrame;

void can_filter_init(void) {
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
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//**����STM32��HAL���CAN���裬���䲻Ϊ�յ�����ͨ����ͨ����鴫��״̬�Ĵ�����TSR���е��ض�λ��ȷ���ġ�
// ������˵��ÿ�����䶼��һ����Ӧ�ġ���������ա���Transmit Mailbox Empty��TME��λ������λΪ0ʱ����ʾ���䲻Ϊ�գ�����λΪ1ʱ����ʾ����Ϊ���ҿ��á�

/** Ѱ�Ҳ�Ϊ�յ����� **/
static uint32_t get_can_free_mail(CAN_HandleTypeDef* hcan)
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

void CAN_SendJointData(CAN_TxFrame_TypeDef* JointTxFrame)
{
    /** ��ȡ���� **/
    uint32_t can_send_mail = get_can_free_mail(JointTxFrame->hcan);

    /** �������� **/
    if (can_send_mail != 0) {
        HAL_CAN_AddTxMessage(JointTxFrame->hcan, &JointTxFrame->Header, JointTxFrame->Data, &can_send_mail);
    }
}

void CAN_SendWheelData(CAN_TxFrame_TypeDef* WheelTxFrame)
{
    /** ��ȡ���� **/
    uint32_t can_send_mail = get_can_free_mail(WheelTxFrame->hcan);

    /** �������� **/
    if (can_send_mail != 0) {
        HAL_CAN_AddTxMessage(WheelTxFrame->hcan, &WheelTxFrame->Header, WheelTxFrame->Data, &can_send_mail);
    }
}

/** CAN1�����жϴ��� **/
static void CAN1_RxFifo0RxHandler(uint32_t *StdId, uint8_t Data[])
{
    switch (CAN1_RxFrame.Header.StdId) {
        case JOINT_LF_RECEIVE:
        {
            dm8009_info_update(&joint[LF], Data);
            break;
        }

        case JOINT_LB_RECEIVE:
        {
            dm8009_info_update(&joint[LB], Data);
            break;
        }

        case JOINT_RF_RECEIVE:
        {
            dm8009_info_update(&joint[RF], Data);
            break;
        }

        case JOINT_RB_RECEIVE:
        {
            dm8009_info_update(&joint[RB], Data);
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
    switch(CAN2_RxFrame.Header.StdId){

        case (LK_FDB_Identifier + WHEEL_L_SEND) :
        {
            lk9025_info_update(&wheel[L], Data);
            break;
        }

        case (LK_FDB_Identifier + WHEEL_R_SEND) :
        {
            lk9025_info_update(&wheel[R], Data);
            break;
        }

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

