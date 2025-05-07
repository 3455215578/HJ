#ifndef _BOARD_COMMUNICATION_TASK_H
#define _BOARD_COMMUNICATION_TASK_H

#include <stdint-gcc.h>

union I16 {
    uint8_t data[2];
    int16_t value;
};

/** 板间发送函数 **/
extern void Send_Chassis_Rc(int16_t ch1, int16_t ch2, int16_t ch4, char sl, char sr);
extern void Send_Control(int32_t W, int32_t A, int32_t S, int32_t D);

/** 板间接收函数 **/
void Chassis_to_Gimbal_Can(uint32_t can_id, const uint8_t *rx_data);

#endif
