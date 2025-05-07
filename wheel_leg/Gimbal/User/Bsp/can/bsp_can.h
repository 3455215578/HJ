#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint-gcc.h>
#include "DJI_Motor.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

typedef struct {

    CAN_HandleTypeDef 	*hcan;
    CAN_TxHeaderTypeDef  Header;
    uint8_t				 Data[8];

}CAN_TxFrame_TypeDef;

typedef struct {

    CAN_HandleTypeDef   *hcan;
    CAN_RxHeaderTypeDef  Header;
    uint8_t 			 Data[8];

} CAN_RxFrame_TypeDef;

extern CAN_TxFrame_TypeDef Gimbal_Send_Chassis_TxFrame;

extern CAN_RxFrame_TypeDef CAN1_RxFrame;
extern CAN_RxFrame_TypeDef CAN2_RxFrame;

void can_filter_init(void);
uint32_t get_can_free_mail(CAN_HandleTypeDef* hcan);
void CAN_Gimbal_Send_Chassis_Data(CAN_TxFrame_TypeDef* Gimbal_Send_Chassis_TxFrame);

#endif
