//
// Created by Gularx on 2024/2/26.
//
#include "ramp.h"

#ifndef INC_2024_HERO_HERO_H
#define INC_2024_HERO_HERO_H

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
typedef enum{
    Fire_OFF=0,             //发射机构关闭
    Fire_ON=1,              //发射机构开启
}Fire_Mode_e;

/* 拨盘状态值 */
typedef enum{
    SHOOT_CLOSE=0,          //发射关闭
    SHOOT_READLY,           //单发指令
    SHOOT_SINGLE,           //单发执行
    SHOOT_OVER,             //发射完成
    SHOOT_BLOCK,            //单发堵转
    SHOOT_BLOCK_BACK,       //反转中
    SHOOT_FAIL              //拨盘坏了
}Shoot_Cmd_e;

/* 底盘状态值 */
typedef enum {
    CHASSIS_RELAX,          //底盘失能 左下右下
    CHASSIS_ONLY,           //底盘独立 左不下 E.click_flag==1
    CHASSIS_FIRE,           //底盘跟随云台 右中
    CHASSIS_FOLLOW_GIMBAL,  //底盘跟随云台 右中
    CHASSIS_SPIN_R,           //小陀螺
    CHASSIS_SPIN_L,           //小陀螺
} Chassis_Mode_e;

/**** 整车数据 ****/


#endif //INC_2024_HERO_HERO_H
