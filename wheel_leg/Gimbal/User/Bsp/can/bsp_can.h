#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

typedef enum {
    CAN_1,
    CAN_2,
}CAN_TYPE;

extern void can_filter_init(void);
extern void CAN_init();

#endif
