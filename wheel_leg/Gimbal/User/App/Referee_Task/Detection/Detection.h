//
// Created by xhuanc on 2022/3/10.
//

#ifndef DEMO1_DETECTION_H
#define DEMO1_DETECTION_H
#include "struct_typedef.h"
#include <stdbool.h>
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"

#define ONLINE 1
#define OFFLINE 0

typedef struct {
//    uint8_t enable;
    uint32_t last_online_time;
//    uint32_t current_time;
    uint8_t status;
    uint32_t offline_threshold;
    uint8_t warning_level;
//    void (*offline_handle)(void*);
}__packed detect_device_t;

typedef enum {
    DETECT_CHASSIS_3508_RF=0,
    DETECT_CHASSIS_3508_LF,
    DETECT_CHASSIS_3508_LB,
    DETECT_CHASSIS_3508_RB,
    DETECT_GIMBAL_6020_PITCH,
    DETECT_GIMBAL_6020_YAW,
    DETECT_LAUNCHER_3508_FIRE_L,
    DETECT_LAUNCHER_3508_FIRE_R,
    DETECT_LAUNCHER_3508_TRIGGER,     //拨盘电机
    DETECT_REMOTE,      //遥控器
    DETECT_AUTO_AIM,    //自动瞄准
    DETECT_REFEREE,     //裁判
    DETECT_CAP,
    DETECT_VIDEO_TRANSIMITTER,  //图传
    DETECT_DEVICE_LIST_LEN, //设备列表LEN
}detect_device_index;

typedef enum
{
    ALL_ONLINE = 1,
    RC_ONLINE,
    VT_ONLINE,
    ALL_OFFLINE,
}control_status;

extern detect_device_t detect_list[DETECT_DEVICE_LIST_LEN];
extern void detect_handle(uint8_t index);

void Detect_task(void const*pvParameters);

#endif //DEMO1_DETECTION_H
