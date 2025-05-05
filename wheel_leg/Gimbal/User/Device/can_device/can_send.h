//
// Created by Gularx on 2024/12/10.
//

#ifndef INC_2024_HERO_THREE_CAN_SEND_H
#define INC_2024_HERO_THREE_CAN_SEND_H
#include "struct_typedef.h"
#include "bsp_can.h"

union I16 {
    uint8_t data[2];
    int16_t value;
};

union UI16 {
    uint8_t data[2];
    uint16_t value;
};

union FP32{
    uint8_t data[4];
    fp32 value;
};

extern void Send_Chassis_Speed(int16_t ch1, int16_t ch2, int16_t ch4, char sl, char sr);
extern void Send_Control(int32_t W, int32_t A, int32_t S, int32_t D);

#endif //INC_2024_HERO_THREE_CAN_SEND_H
