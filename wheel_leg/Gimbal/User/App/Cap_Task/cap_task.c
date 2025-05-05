//
// Created by Gularx on 2025/3/8.
//

/*********************************************************************************************************
*                                              ����ͷ�ļ�
*********************************************************************************************************/
#include "cap_task.h"
#include "can.h"
#include "task.h"
#include "cmsis_os.h"
#include "Detection.h"
#include "key_board.h"
#include "Referee.h"

/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/
Bear_Cap bearCap;
uint8_t isInited = 0;


/*********************************************************************************************************
*                                              �ڲ���������
*********************************************************************************************************/
_Noreturn void Cap_task(void const *pvParameters);
static void Cap_Can_Init_Mode(CAP_INIT_ID_e mode);
static void Cap_Can_Control(uint8_t pb_set, CAP_CONTRAL_ID_e mode);


/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
void Cap_task(void const *pvParameters) {
    vTaskDelay(CAP_TASK_INIT_TIME);

    while (1) {
        /* ������������can�ź� */
        if (bearCap.state.controller_state == CAP_INIT_RECEIVE_ENABLE) {
            if(isInited == 0) {
                Cap_Can_Init_Mode(CAP_INIT_MODE_28V);
                isInited = 1;
            }
            //����С��50%�ŷ���������50%������0011 0010��С��15%ʱ��bit2�������������λ��Ϊ0
            if((!(Referee.Buff.remaining_energy >> 2 & 1)) && Referee.Buff.remaining_energy != 50) {
                bearCap.mode = SILENT;
                Cap_Can_Control(Referee.GameRobotStat.chassis_power_limit-8, SILENT);
            }else {
                bearCap.mode = WORK;
                Cap_Can_Control(Referee.GameRobotStat.chassis_power_limit-8, WORK);
            }
            // Cap_Can_Control(Referee.GameRobotStat.chassis_power_limit-8, WORK);
        }
        if(detect_list[DETECT_CAP].status == OFFLINE) {
            Cap_Can_Init_Mode(CAP_INIT_MODE_28V);
        }
        /* ���̶ϵ�,Ҫ�����ݷ�ʧ�ܵ��ź� */
        // if (Referee.GameRobotStat.power_management_chassis_output == 0) {
        //     bearCap.cap_mode = DISABLE;
        // }

        /* ���������˶�ʱ,�����Ƿ񷢵��ܵ��ݱ���ȫ״̬�Ŀ��� */
        // if (bearCap.cap_mode == ENABLE) {
        //     if(bearCap.err.Calibration_err < RISK) {
        //         Cap_Can_Control(Referee.GameRobotStat.chassis_power_limit-8, WORK);
        // last_chassis_power_limit = Referee.GameRobotStat.chassis_power_limit;
        // }
        // if (last_chassis_power_limit != Referee.GameRobotStat.chassis_power_limit) {
        //     Cap_Can_Control(Referee.GameRobotStat.chassis_power_limit-8, WORK);
        //     last_chassis_power_limit = Referee.GameRobotStat.chassis_power_limit;
        // }
        // }
        osDelay(100);
    }
}

static void Cap_Can_Init_Mode(CAP_INIT_ID_e mode) {
    CAN_TxHeaderTypeDef tx_message;
    uint8_t cap_can_send_data[8];
    uint32_t send_mail_box;
    tx_message.StdId = INIT_FRAME;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x01;
    cap_can_send_data[0] = mode;
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, cap_can_send_data, &send_mail_box);
}

static void Cap_Can_Control(uint8_t pb_set, CAP_CONTRAL_ID_e mode) {
    CAN_TxHeaderTypeDef tx_message;
    uint8_t cap_can_send_data[8];
    uint32_t send_mail_box;
    tx_message.StdId = CONTRAL_FRAME;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x03;
    cap_can_send_data[0] = pb_set;
    cap_can_send_data[1] = Exceed_DISABLE;
    cap_can_send_data[2] = mode;
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, cap_can_send_data, &send_mail_box);
}

/*********************************************************************************************************
*                                              API����ʵ��
*********************************************************************************************************/
void Cap_Data_Get(uint32_t can_id, const uint8_t *rx_data) {
    /* ��ȫ��ʾ֡ */
    if (can_id == SAFETY_FRAME) {
        bearCap.err.Firmware_err = rx_data[0];
    }
        /* �յ���can��Ϣ */
    else if (can_id == FEEDBACK_FRAME) {
        bearCap.capReceiveData.esr_v = (uint16_t)((rx_data[0]<<8)|rx_data[1]);
        bearCap.capReceiveData.work_s1 = rx_data[2];
        bearCap.capReceiveData.work_s2 = rx_data[3];
        bearCap.capReceiveData.input_power = (uint16_t)((rx_data[4]<<8)|rx_data[5]);
        /* ���޷����ܵ�can�źŸ�Ϊ������������can�ź� */
        if (bearCap.state.controller_state == CAP_INIT_RECEIVE_DISABLE){
            bearCap.state.controller_state = CAP_INIT_RECEIVE_ENABLE;
        }
        detect_handle(DETECT_CAP);
    }
        /* �Ƿ��շ�can�ź� */
    else if (can_id == READY_FRAME) {
        bearCap.state.controller_state = rx_data[0];
    }
}

