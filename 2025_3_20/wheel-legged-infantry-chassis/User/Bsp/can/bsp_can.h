#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint-gcc.h>

void can_filter_init(void);

uint32_t get_can1_free_mailbox();
uint32_t get_can2_free_mailbox();

#endif
