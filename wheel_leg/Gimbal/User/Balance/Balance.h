//
// Created by Gularx on 2024/2/26.
//
#include "ramp.h"

#ifndef INC_BALANCE_H
#define INC_BALANCE_H

#define GIMBAL
// #define CHASSIS

/**** 模式选择 ****/
typedef enum {
    GIMBAL_RELAX,   //云台失能
    GIMBAL_BACK,    //从失能到使能
    GIMBAL_ACTIVE,  //得到遥控器对云台电机的控制,自瞄判定
    GIMBAL_AUTO,    //云台自瞄模式
    GIMBAL_FIRE,    //云台打弹模式
}Gimbal_Mode_e;

/* 摩擦轮状态值 */
/** 因为在判断摩擦轮模式的时候会顺带判断拨盘模式，所以把摩擦轮开启与否看作发射机构开启或关闭(暂定是这样，后续可能会分开，便于区别) **/
typedef enum{
    Fire_OFF=0,             //发射机构关闭
    Fire_ON=1,              //发射机构开启
}Fir_Wheel_Mode_e;

/* 拨盘状态值 */
typedef enum{
    SHOOT_CLOSE=0,          //发射关闭
    SHOOT_CONTINUE,           //连发指令

    SHOOT_READY_TO_SINGLE,  //单发指令
    SHOOT_BLOCK,            //单发堵转

    SHOOT_SINGLE,           //单发执行
    SHOOT_INVERSING,        //反转中

    SHOOT_OVER,             //发射完成
    SHOOT_FAIL              //拨盘坏了
}Shoot_Cmd_e;


#endif //INC_BALANCE_H
