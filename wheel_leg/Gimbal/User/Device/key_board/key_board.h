//
// Created by xhuanc on 2021/12/3.
//

#ifndef DEMO1_KEY_BOARD_H
#define DEMO1_KEY_BOARD_H

#include "remote.h"
#include "Referee.h"

#define DEFAULT_CNT 100/24 // 100/任务周期=实际经过时间
#define MOUSE_CLICK_R_CNT 5
#define MOUSE_CLICK_L_CNT 5




/** 遥控器摇杆通道标号 **/

// 用于板间通信
#define CHASSIS_VX_CHANNEL 1
#define CHASSIS_YAW_CHANNEL 0

// 云台控制
#define GIMBAL_YAW_CHANNEL     2
#define GIMBAL_PITCH_CHANNEL   3
#define GIMBAL_AUTO_CHANNEL    4

/* 修改摇杆的参数值 */
#define RC_TO_YAW       0.0008f//0.0003f//0.00025f
#define RC_TO_PITCH     0.0002f

/* 修改鼠标的参数值 */
#define MOUSE_X_RADIO 0.004f//0.01f//0.01f
#define MOUSE_Y_RADIO 0.001f//0.005f//0.01f

/* 遥控器控制底盘最大速度增量 */
#define MAX_CHASSIS_VX_SPEED 3.f
#define MAX_CHASSIS_VY_SPEED 3.f
#define MAX_CHASSIS_VW_SPEED 3.f

// 冯楚乔注：我认为这个状态的逻辑在使用上有问题，同时在状态切换的逻辑也有问题，
// 建议不要使用这个状态用于判断类似发射单发、一键掉头这样的操作。
// 使用 `click_flag == 1` 来判断是否是单击，并将其改回0。
// 如果要修改，需要重写按键使用的逻辑，改成事件驱动型。
// 例如，按下按键时，触发一个事件，松开按键时，触发一个事件。
// 事件的类型可以是：按下、松开、单击、双击、长按、长按松开等等。
// 事件的处理函数可以是：发射单发、发射连发、掉头、切换摩擦轮速度等等。
// 这样的逻辑更加清晰，也更加灵活。
// 但是，这样的逻辑需要重写按键的使用逻辑，所以暂时不改。
// 以上建议有部分是 GitHub Copilot 的建议。
typedef enum {
    KEY_RELAX,//按键没被按下
    KEY_DOWN,
    KEY_CLICK,//PRESS 一下之后就RELAX 就是CLICK
    KEY_PRESS,//PRESS 超过一段时间 DOWN
} Key_Status;

typedef struct {
    Key_Status status;
    Key_Status last_status;
    uint32_t press_cnt;
    uint8_t click_flag;
    uint32_t click_cnt;
} Key;

//键盘结构体
typedef struct {
    Key Mouse_l;
    Key Mouse_r;
    Key W;
    Key S;
    Key A;
    Key D;
    Key Q;
    Key E;
    Key R;
    Key F;
    Key G;
    Key Z;
    Key X;
    Key C;
    Key B;
    Key V;
    Key SHIFT;
    Key CTRL;
} key_board_t;

extern void update_pc_info();
extern key_board_t KeyBoard;

#endif //DEMO1_KEY_BOARD_H
