//
// Created by Gularx on 2025/3/8.
//

#ifndef INC_2025_HERO_THREE_CAP_H
#define INC_2025_HERO_THREE_CAP_H

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "cmsis_os.h"
#include "Referee.h"
#include "bsp_can.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
/* 任务开始空闲一段时间 */
#define CAP_TASK_INIT_TIME 357



/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
/** 接收是指接收电容的数据, 发送是指给电容发送数据 */

/** 接收 安全提示帧 0x001 */
typedef enum {
    /* 安全提示帧 */
    SAFETY_FRAME=0X001,
    SAFE=0X00,              // 安全
    WARN=0X01,              // 警告
    RISK=0X02,              // 风险
    DANGEROUS=0X03,         // 危险
    IRREVERSIBLE=0X04,      // 不可逆损伤
}CAP_SAFETY_ID_e;
typedef struct{
    uint8_t Firmware_err;       // 固件错误检测项的安全等级
    uint8_t Can_err;            // CAN错误检测项的安全等级
    uint8_t Overheat_err;       // 过热保护检测项的安全等级
    uint8_t Calibration_err;    // 校准错误检测项的安全等级
    uint8_t Voltage_err;        // 电压错误检测项的安全等级
    uint8_t Current_err;        // 电流错误检测项的安全等级
    uint8_t Power_err;          // 功率错误检测项的安全等级
    uint8_t Sampling_err;       // 采样错误检测项的安全等级
} Cap_Safety_Data;


/** 接收 反馈帧帧 0X003 电容的反馈数据 */
typedef enum {
    /* 反馈帧 */
    FEEDBACK_FRAME=0X003,
}CAP_FEEDBACK_ID_e;
typedef struct{
    uint16_t esr_v;         // 电容组电压
    uint8_t work_s1;        // 工作强度1
    uint8_t work_s2;        // 工作强度2
    uint16_t input_power;   // 电源输入功率
} Cap_Receive_Data;


/** 接收 就绪帧 0X005 该帧会在控制器上电后发送一次,工作中因为安全等级的变化可能再次进行发送 */
typedef enum {
    /* 就绪帧 */
    READY_FRAME=0X005,
    CAP_INIT_RECEIVE_ENABLE=0xFF,   // 控制器已就绪, 可以进行can数据收发
    CAP_INIT_RECEIVE_DISABLE=0x00,  // 控制器不可用, 接受的所有can帧将被忽略
}CAP_READY_ID_e;
typedef struct{
    uint8_t controller_state;
} Cap_Ready_Data;


/** 发送 初始化帧 0X002 该帧每次上电后仅接受一次 */
typedef enum {
    /* 初始化帧 */
    INIT_FRAME=0X002,
    CAP_INIT_MODE_24V=0x00,      // 超级电容组类型1,电容组最高充电电压为24V
    CAP_INIT_MODE_28V=0x01,      // 超级电容组类型2,电容组最高充电电压为28V
    CAP_INIT_MODE_30V=0x02,      // 超级电容组类型3,电容组最高充电电压为30V
}CAP_INIT_ID_e;

/** 发送 控制帧 0X004 */
typedef enum {
    /* 控制帧 */
    CONTRAL_FRAME=0X004,
    SILENT=0X00,            // Silent 工作模式
    WORK=0X01,              // Work 工作模式
    CHARGE=0X02,            // Charge 工作模式
}CAP_CONTRAL_ID_e;
typedef enum {
    Exceed_DISABLE=0X00,    // Exceed 失能
    Exceed_ENABLE=0X01,     // Exceed 使能
}CAP_Exceed_ID_e;

/* 所有接收数据 */
typedef struct {
    Cap_Receive_Data capReceiveData;    // 反馈帧的数据
    Cap_Safety_Data err;                // 安全提示错误
    Cap_Ready_Data state;               // 就绪帧的状态(can)
    uint8_t cap_mode;                   // 工作状态(失能使能)
    CAP_CONTRAL_ID_e mode;
} Bear_Cap;

extern Bear_Cap bearCap;

extern void Cap_Data_Get(uint32_t can_id, const uint8_t *rx_data);


#endif //INC_2025_HERO_THREE_CAP_H
