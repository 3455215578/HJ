#include "bsp_can.h"
#include "main.h"
#include "detect_task.h"
#include "DJI_Motor.h"
#include "gimbal_task.h"
#include "launcher.h"

/* 接收结构体 */
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


//**对于STM32的HAL库和CAN外设，邮箱不为空的条件通常是通过检查传输状态寄存器（TSR）中的特定位来确定的。
// 具体来说，每个邮箱都有一个对应的“传输邮箱空”（Transmit Mailbox Empty，TME）位，当该位为0时，表示邮箱不为空；当该位为1时，表示邮箱为空且可用。

/** 寻找不为空的邮箱 **/
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

/** CAN1接收中断处理 **/
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
            DJI_Round_Count(&launcher.trigger.motor_measure);//获取转动拨轮电机转动圈数和总编码值
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

/** CAN2接收中断处理 **/
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

