//
// Created by xhuanc on 2021/9/27.
//

#include "can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "cap_task.h"
#include "bsp_cap.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/******************** define *******************/

//电子数据解算,do while作为保护性代码，防止在展开时被错误编译
#define get_motor_measure(ptr, data)                                    \
    do{                                                                 \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    } while(0)

//电机总编码值的计算,do while作为保护性代码，防止在展开时被错误编译
#define get_motor_round_cnt(ptr)  \
    do{                            \
             if(ptr.ecd-ptr.last_ecd> 4192){ \
                ptr.round_cnt--;                    \
             }                   \
             else if(ptr.ecd-ptr.last_ecd< -4192)    \
             {                   \
                ptr.round_cnt++;            \
             }                   \
             ptr.total_ecd= ptr.round_cnt*8192+ptr.ecd-ptr.offset_ecd;\
                                 \
    }while(0)
//电机真实距离计算
#define get_motor_real_distance(ptr)\
{\
    get_motor_round_cnt(ptr);\
    ptr.torque_round_cnt=ptr.total_ecd/8192.f; \
    ptr.real_round_cnt=ptr.torque_round_cnt/19.f; \
    ptr.real_angle_deg=fmodf(ptr.real_round_cnt,1.0f);             \
    if(ptr.real_angle_deg<0.0f)ptr.real_angle_deg+=1.0f;                \
    ptr.real_angle_deg*=360.0f;     \
    ptr.total_dis=ptr.real_round_cnt*wheel_circumference;      \
}\

//3508减速比
#define motor_3508_reduction_ratio (3591.0f/187.0f)
//轮子周长
#define wheel_circumference (70*2*3.14f)
/******************** variable *******************/


motor_measure_t motor_3508_measure[6];
motor_measure_t motor_yaw_measure;
motor_measure_t motor_pitch_measure;
motor_measure_t motor_shoot_measure[4];//0:TRIGGER,建为数组方便以后添加
motor_measure_t motor_2006_measure[1];//TRIGGER
extern cap2_info_t cap2;

static CAN_TxHeaderTypeDef tx_message;
static uint8_t can_send_data[8];
int32_t cap_percentage;

int cap_can_cnt = 0;

void cap2_info_decode(cap2_info_t *cap, uint8_t *rx_data) {
    cap->mode = rx_data[0];
//    cap->rec_cap_cmd=rx_data[1];
//    cap->cap_voltage=rx_data[2];
    cap->cap_voltage = (uint16_t) (rx_data[2] << 8 | rx_data[3]);
//    cap->chassis_current=(uint16_t)(rx_data[4]<<8|rx_data[5])/1000;
//    cap_percentage=cap->cap_voltage/22.f;
//
//    cap_can_cnt++;
}

//车轮电机的发送函数
void CAN_cmd_motor(CAN_TYPE can_type, can_msg_id_e CMD_ID, int16_t RF_motor, int16_t LF_motor, int16_t LB_motor,
                   int16_t RB_motor) {
    uint32_t send_mail_box;
    tx_message.StdId = CMD_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = RF_motor >> 8;
    can_send_data[1] = RF_motor;
    can_send_data[2] = LF_motor >> 8;
    can_send_data[3] = LF_motor;
    can_send_data[4] = LB_motor >> 8;
    can_send_data[5] = LB_motor;
    can_send_data[6] = RB_motor >> 8;
    can_send_data[7] = RB_motor;

    if (can_type == CAN_1) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
    } else if (can_type == CAN_2) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
    }

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;

    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1) {
        switch (rx_header.StdId) {
            case CAN_CHASSIS_3508_MOTOR_RF: {
                get_motor_measure(&motor_3508_measure[RF], rx_data);
                detect_handle(DETECT_CHASSIS_3508_RF);
            }
                break;

            case CAN_CHASSIS_3508_MOTOR_LF: {
                get_motor_measure(&motor_3508_measure[LF], rx_data);
                detect_handle(DETECT_CHASSIS_3508_LF);
            }
                break;

            case CAN_CHASSIS_3508_MOTOR_LB: {
                get_motor_measure(&motor_3508_measure[LB], rx_data);
                detect_handle(DETECT_CHASSIS_3508_LB);
            }
                break;

            case CAN_CHASSIS_3508_MOTOR_RB: {
                get_motor_measure(&motor_3508_measure[RB], rx_data);
                detect_handle(DETECT_CHASSIS_3508_RB);
            }
                break;

            case CAN_GIMBAL_6020_YAW: {
                get_motor_measure(&motor_yaw_measure, rx_data);
                detect_handle(DETECT_GIMBAL_6020_YAW);
            }
                break;

            case CAP_ERR_FEEDBACK_ID: {
                Cap.err_state[CAP_FIRMWARE_ERR] = rx_data[CAP_FIRMWARE_ERR];
                Cap.err_state[CAP_CAN_ERR] = rx_data[CAP_CAN_ERR];
                Cap.err_state[CAP_TEMP_ERR] = rx_data[CAP_TEMP_ERR];
                Cap.err_state[CAP_CALI_ERR] = rx_data[CAP_CALI_ERR];
                Cap.err_state[CAP_VOLT_ERR] = rx_data[CAP_VOLT_ERR];
                Cap.err_state[CAP_CURR_ERR] = rx_data[CAP_CURR_ERR];
                Cap.err_state[CAP_POWER_ERR] = rx_data[CAP_POWER_ERR];
                Cap.err_state[CAP_SAMP_ERR] = rx_data[CAP_SAMP_ERR];
                if (Cap.can_init_state != CAP_INIT_FINISHED) {
                    Cap.can_init_state = CAP_INIT_FINISHED;
                }
                break;
            }
            case CAP_INFO_FEEDBACK_ID: {
                Cap.capFeedback.esr_v = (uint16_t) ((rx_data[0] << 8) | rx_data[1]);
                Cap.capFeedback.work_s1 = rx_data[2];
                Cap.capFeedback.work_s2 = rx_data[3];
                Cap.capFeedback.input_power = (uint16_t) ((rx_data[4] << 8) | rx_data[5]);
                cap_percentage = Cap.capFeedback.esr_v / 2800;
                if (Cap.can_init_state != CAP_INIT_FINISHED) {
                    Cap.can_init_state = CAP_INIT_FINISHED;
                }
                detect_handle(DETECT_CAP);
                break;
            }
            case CAP_INIT_FEEDBACK_ID: {
                Cap.can_init_state = rx_data[0];
                hcan->ErrorCode = HAL_CAN_ERROR_NONE;
                break;
            }

            default:
                break;
        }
    } else if (hcan == &hcan2) {
        switch (rx_header.StdId) {

            case CAN_LAUNCHER_3508_FIRE_L: {
                get_motor_measure(&motor_3508_measure[4], rx_data);
                detect_handle(DETECT_LAUNCHER_3508_FIRE_L);
            }
                break;

            case CAN_LAUNCHER_3508_FIRE_R: {
                get_motor_measure(&motor_3508_measure[5], rx_data);
                detect_handle(DETECT_LAUNCHER_3508_FIRE_R);
            }
                break;

            case CAN_GIMBAL_6020_PITCH: {
                get_motor_measure(&motor_pitch_measure, rx_data);
                detect_handle(DETECT_GIMBAL_6020_PITCH);
            }
                break;

            case CAN_LAUNCHER_2006_TRIGGER: {
                get_motor_measure(&motor_2006_measure[0], rx_data);
                get_motor_round_cnt(motor_2006_measure[0]);//获取转动拨轮电机转动圈数和总编码值
                detect_handle(DETECT_LAUNCHER_2006_TRIGGER);
            }
                break;
            default:
                break;
        }
    }
}


fp32 motor_ecd_to_rad_change(uint16_t ecd, uint16_t offset_ecd) {
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE) {
        relative_ecd -= ECD_RANGE;
    } else if (relative_ecd < -HALF_ECD_RANGE) {
        relative_ecd += ECD_RANGE;
    }

    return ((fp32) relative_ecd * MOTOR_ECD_TO_RAD);
}


//计算距离零点的度数  -180-180
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd) {
    int32_t tmp = 0;
    if (offset_ecd >= 4096) {
        if (ecd > offset_ecd - 4096) {
            tmp = ecd - offset_ecd;
        } else {
            tmp = ecd + 8192 - offset_ecd;
        }
    } else {
        if (ecd > offset_ecd + 4096) {
            tmp = ecd - 8192 - offset_ecd;
        } else {
            tmp = ecd - offset_ecd;
        }
    }
    return (fp32) tmp / 8192.f * 360;
}

void CAN_cmd_cap2(cap2_info_t *cap) {
    uint32_t send_mail_box;
    tx_message.StdId = 0x002;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x06;
    for (uint8_t i = 0; i <= 4; ++i) {
        can_send_data[i] = cap->send_data[i];
    }
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, cap->send_data, &send_mail_box);
}
