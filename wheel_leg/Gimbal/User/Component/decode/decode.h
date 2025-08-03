//
// Created by Shockley on 2022/12/5.
//

#ifndef OSX_PROJECT_DECODE_H
#define OSX_PROJECT_DECODE_H

#include <stdint-gcc.h>

//反序列化函数
void decode_unpack_fifo_data(void);
uint16_t decode_data_solve(uint8_t *frame);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

//协议序列化函数
void encode_send_data(uint16_t cmd_id, void* buf, uint16_t len);

uint16_t referee_data_solve(uint8_t *frame);
void usb_fifo_init();
void Decode_task(void const *arg);

#endif //OSX_PROJECT_DECODE_H
