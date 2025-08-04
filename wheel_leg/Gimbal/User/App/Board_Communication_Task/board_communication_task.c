#include "board_communication_task.h"

static int float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}

// 前后（2）、腿长（2）、右侧拨杆（1）、gimbal.init_flag（1）、一个相对角度（2）

void Send_Chassis_Data(int16_t vx_channel, int16_t leg_channel, char sr, bool gimbal_init_flag, float yaw_relative_angle)
{
    /** 定义发送结构体 **/
    CAN_TxHeaderTypeDef  tx_message;

    tx_message.StdId = 0x110;      //CAN ID
    tx_message.IDE = CAN_ID_STD;   // 标准帧
    tx_message.RTR = CAN_RTR_DATA; // 数据帧
    tx_message.DLC = 0x08;

    /** 定义发送邮箱 **/
    uint32_t send_mail_box;

    /** 定义发送的数据字节 **/
    uint8_t Send_data[8];

    union I16 ch;

    /** 1.前后 **/
    ch.value = vx_channel;
    Send_data[0] = ch.data[0];
    Send_data[1] = ch.data[1];

    /** 2.腿长 **/
    ch.value = leg_channel;
    Send_data[2] = ch.data[0];
    Send_data[3] = ch.data[1];

    /** 3.右拨钮 **/
    Send_data[4] = sr;

    /** 4.云台初始化标志位 **/
    Send_data[5] = gimbal_init_flag;

    /** 5.云台与底盘正方向相对角度 **/
    uint16_t yaw_relative_angle_temp = (uint16_t)float_to_uint(yaw_relative_angle, -180.0f, 180.0f, 16);

    ch.value = (int16_t)yaw_relative_angle_temp;
    Send_data[6] = ch.data[0];
    Send_data[7] = ch.data[1];

    /** 获取邮箱 **/
    uint32_t can_send_mail = get_can_free_mail(&hcan2);

    /** 发送数据 **/
    if (can_send_mail != 0) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, Send_data, &send_mail_box);
    }
}

void Send_Control(int32_t W, int32_t A, int32_t S, int32_t D) {
    CAN_TxHeaderTypeDef  tx_message;
    uint32_t send_mail_box;
    uint8_t Send_data[8];
    tx_message.StdId = 0x111;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    Send_data[0] = (uint8_t)(W & 0xFF);
    Send_data[1] = (uint8_t)(A & 0xFF);
    Send_data[2] = (uint8_t)(S & 0xFF);
    Send_data[3] = (uint8_t)(D & 0xFF);
    /** 获取邮箱 **/
    uint32_t can_send_mail = get_can_free_mail(&hcan2);

    /** 发送数据 **/
    if (can_send_mail != 0) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, Send_data, &send_mail_box);
    }
}

void Chassis_to_Gimbal_Can(uint32_t can_id, const uint8_t *rx_data) {
    switch (can_id) {
        case 0x115:
        {
            break;
        }
        default:
        {
            break;
        }
    }
}
