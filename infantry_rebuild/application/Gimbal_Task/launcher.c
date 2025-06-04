//%%88888888888888888888888888888888888888888888888888888888888888888888888888888%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8%&%BB%B%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%i. .  `iJ8B8B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B8%%Bw''.  '.''`q%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB].. .......'.w$%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BBm^'.........`+B8B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%88' .........."B%BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8Bf .........'.0&88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%B&@8B%8%%%%%%%%%%%%BB8%%w  ........`]CB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%BW@%B%8%%%%%%%%%%%%%%%@~  . .......^88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%BB%8%%8#>d8%%$%8%%%%%%%B8%%b. ........  #8%%%%%8%%}}C,UWW%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%W&%pl` ''''`[LB%%BZq-x{Z@BJ   ..... ..lJ%%B}C}W%W*`...... ]8$%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%O`'`'.    . '  '.'^ ?%%B%b:. ...... ..&@%%B~.... .. .....^` `w#&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%W`.'..'    ... .'.X{BB%L..   ...  ..O%8BBz..            ..:C%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%BB_'......... ' `w@%B%B`  `......`Ib@%%&%JI .           ..QBB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%J' ..... '. Ih@#BB%%{a'  .......'"&}BBBBBBB%B%%hu '.    . ''Q$&%B%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%B%%%%%%%8%8%b^''..' . "+{WW8%%%%&%J       .....0@BBBBBBB%%%%BB%B$w .'.     'h8%%%%B%%%B%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%@^ .'''"..''.''.'.''M%#8B%%%%%%8M`'..     ...h%BBBBB%%%%%%%BBB%BB%C[`   .. '','''.'` B%%%%%%%%%%%%%%%%%%%
//%%%%%%%%Bj`    .'. '......^B%B%%%%%%%%%%k"'.'       ,M@BBB&%%BBBB8%&ZUYhaw#%&W'`  ...........' 8%%%%%%%%%%%%%%%%%%
//%%%%%%%8L``'.   '    . ^`?@%%8%%%%%%B%8L` '        ''.'...'...........'..z@BB@BL.'...        .`_Z%%%%%%%%%%%%%%%%%
//%%%%%%%m;`''''''......."&8%%8%%%%%%%%B%` .       . . .'.  .......    '.'C%%BB%B&J'         ..'':_%@%%%%%%%%%%%%%%%
//%%%%%%%%B%%%%%o ..... .v%B%%%%%%%%%BB%'^..                          .''J&88%%%B%8t.....   0&@88%8%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%f      .'$B%%%%%%%%%%8Q+'. . ... '''..    '`^.`.        a8&8%8%%%%B$`.......kB%%888%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%+'      ^B%%%%%%%%8%%&.'  . '';Y%%%8%BB%BBBBb  . . .   08BB88%%%%%%}'.     .n%88888%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%B8h^      'MB8%%%%%%%8%:.   ......^u&%%B%%BB&h       '.'q%8%%%%%%%%%BZ'.   . .bB88888%%%%%%%%%%%%%%%%%
//%%%%%%%W%$MQh:..     .._B%%%%%%%B#j       .  .U%%88888BB!..    ... v%8%%%%%%%%%%Bl     .'''nw&W@$%%%%%%%%%%%%%%%%%
//%%%%%%%#+''.''`'''... . *$%%%%%%W''        . zB%%%8888B<'`    ....J&8%%%%%%%%%BBw.      ...'.',.b%%%%%%%%%%%%%%%%%
//%%%%%%%B8l.............'."O@%%8W^.  ..     :u&%&%B%%%p' '''..   .bBW%%%%%%%%%%C+.'     ....... %W%%%%%%%%%%%%%%%%%
//%%%%%%%%%&..'.''. `  ..''' n&%QnkhJM&8B@BBB%%88%8888q...        o%88%%%%%%%&%]`.. ...  .  .  'M&%%%%%%%%%%%%%%%%%%
//%%%%%%%B%@ZadJLO888h`'''..   ;%@8%BB%%8&8%%888888%%Q'..       '+&8%%%%%%%W@l. .. .'''C%BW0Xkkq%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%B%%B%8%BWQ '.''''' `>{BW%%B8%%%%%8888BZ`'.     . 'q88%BB%B8B!^. ..'... [&%%BBB%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%B%8%%8%%Bk .'''''`.^`'xY8%%%%%%%%%8f ^       . r$8%%&Bt`^   .. ... Y@&88%%%%8%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%B%%%%B%8B%8%Wl^'.. ......  ''',[J&%8Bv` .       .,%8%_.`' .....   . 'Z&88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%B%%%%%%%%%%%%%%%%%%@k'`''........ .   . +B%%X!.        ..b}%al    '... .    '.`tBB8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%8B{~''`'........:j,'.']BBBJ. .     ..  v%@q ^  '``... .     '.'L8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%@%&BQ` ''  ,#%B%8BB&8B&0' .      ..'<&8&888%88&%%Bu     'w#$%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%B%%%%%%%%%%%%%%%%%%%88WkMB8%%%%%%%%%%f'          .b%8%%%%%%%%%%%%8%@mW@B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%8B%%%B%%8%%%%%B%x`.        . -B8%%%%%%%%%%%B&8%%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#z'`.      .' .B&B8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8B@M` ..      . ^nOB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%C'.. .     ..']%B%%%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB#....        ' BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Bq .   .    ' Q%BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%88888] .^^.   "p@B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B%%B%$d'"  ^.&BB8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB%8M '^hB%8%88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B88%%%%W&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#include <stdbool.h>
#include "launcher.h"
#include "gimbal_task.h"
#include "can_receive.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "remote.h"
#include "key_board.h"
#include "bsp_laser.h"
#include "filter.h"
#include "referee_task.h"
#include "detect_task.h"
#include "STL.h"
#include "Feedforward_PID.h"

uint8_t rc_last_sw_L;
uint8_t rc_last_sw_R;
uint8_t blocked_flag;
uint8_t reverse_flag;
uint32_t continue_shoot_time;//遥控器左边拨杆down的持续时间 或者 鼠标左键按下的持续时间
uint32_t blocked_start_time;
uint32_t reverse_start_time;
uint32_t Cooling_time = 0;
first_order_filter_type_t filter_trigger_rpm_in;
extern key_board_t KeyBoard;
extern RC_ctrl_t rc_ctrl;
extern gimbal_t gimbal;
extern motor_measure_t motor_2006_measure[1];
extern chassis_t chassis;
extern robot_ctrl_info_t robot_ctrl;

bool is_blocked(float blockSpeed);

void launcher_relax_handle();

void trigger_block_handle(fp32 blockSpeed, fp32 reverse_time, fp32 reverseSpeed);

int thisRc = 0, lastRc = 0;
//通过赋值进行发射机构的初始化
launcher_t launcher;
Queue launchQueue;   //发射队列，储存弹丸发射速度，进行动态射速控制
int32_t total_ecd_ref = 0;

void launcher_mode_set() {

    //摩擦轮关闭时,做拨杆向上拨一下开启摩擦轮
    //遥控器和键盘可以同步修改摩擦轮状态
    // 键盘直接修改相关按键click_flag的状态 通过click状态直接判断摩擦轮模式 避免遥控打开的摩擦轮被键盘关闭
    if ((!switch_is_up(rc_last_sw_L) && switch_is_up(rc_ctrl.rc.s[RC_s_L]))) {
        if (KeyBoard.Q.click_flag == 1 && (chassis.mode != CHASSIS_RELAX && chassis.mode != CHASSIS_ONLY)) {
            KeyBoard.Q.click_flag = 0;
        } else if (KeyBoard.Q.click_flag == 0 && (chassis.mode != CHASSIS_RELAX && chassis.mode != CHASSIS_ONLY)) {
            KeyBoard.Q.click_flag = 1;
        }
    }

    if (KeyBoard.Q.click_flag == 1 && (chassis.mode != CHASSIS_RELAX && chassis.mode != CHASSIS_ONLY)) {
        launcher.fire_mode = Fire_ON;
        laser_on();
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
    } else if (KeyBoard.Q.click_flag == 0 && (chassis.mode != CHASSIS_RELAX && chassis.mode != CHASSIS_ONLY)) {
        launcher.fire_mode = Fire_OFF;
        laser_off();
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    }

    //摩擦轮开启时,做拨杆向上拨一下关闭摩擦轮

    launcher.trigger_last_cmd = launcher.trigger_cmd;
    //拨轮控制 拨轮转动的条件是 当前摩擦轮是开的
    //要么遥控器拨杆向下播一下
    //要么鼠标左键点一下

    // 2023-04-22 临时修改单发逻辑，将鼠标按下视为拨杆按下，因为实测拨杆的单发逻辑是正常的。
    // 使用 shoot_enable 变量来替代下面原来的几个条件中拨杆下拨
    int shoot_enable = (switch_is_down(rc_ctrl.rc.s[RC_s_L]) ||
                        KeyBoard.Mouse_l.status == KEY_CLICK) || robot_ctrl.fire_command != 0;
    if (gimbal.mode != GIMBAL_AUTO && robot_ctrl.fire_command != 0 ||
        gimbal.mode != GIMBAL_BUFF && robot_ctrl.fire_command != 0 ||
        gimbal.mode != GIMBAL_SBUFF && robot_ctrl.fire_command != 0) {
        launcher.trigger_cmd = SHOOT_CLOSE;
    }
    if (launcher.fire_mode == Fire_ON
        && ((shoot_enable)
            || KeyBoard.Mouse_l.status == KEY_CLICK)
        || KeyBoard.Mouse_l.status == KEY_PRESS
        || launcher.trigger_cmd == SHOOT_ING ||
        robot_ctrl.fire_command != 0) //摩擦轮没开默认关闭gimbal.mode!=GIMBAL_AUTO&&robot_ctrl.fire_command!=0
    {
        if ((!switch_is_down(rc_last_sw_L) && (shoot_enable /* switch_is_down(rc_ctrl.rc.s[RC_s_L] */))
            || KeyBoard.Mouse_l.status == KEY_CLICK &&
               launcher.trigger_cmd != SHOOT_ING)//launcher.trigger_cmd!=SHOOT_ING 表示当前不处于单发状态中
        {
            launcher.trigger_cmd = SHOOT_SINGLE;
            continue_shoot_time = HAL_GetTick();//计时
        }
        if (robot_ctrl.fire_command == 1) {
            launcher.trigger_cmd = SHOOT_SINGLE;
        }

        //计时达到触发连发的时间
        if (((shoot_enable) && CONTINUES_SHOOT_TIMING_COMPLETE())
            || (KeyBoard.Mouse_l.status == KEY_PRESS)) {
            launcher.trigger_cmd = SHOOT_CONTINUES;
        }
        if (robot_ctrl.fire_command == 2) {
            launcher.trigger_cmd = SHOOT_DOUBLE;
        }
    } else {
        launcher.trigger_cmd = SHOOT_CLOSE;
    }
    thisRc = rc_ctrl.rc.ch[4];
    if (thisRc != 0 && lastRc == 0) {
        //   launcher.trigger_cmd=SHOOT_SINGLE;  //调试用单发
    }
    lastRc = thisRc;
    rc_last_sw_L = rc_ctrl.rc.s[RC_s_L];
    rc_last_sw_R = rc_ctrl.rc.s[RC_s_R];
}

//发射机构控制
moving_Average_Filter speed_avg_18 = {
        .length=10,
};
uint8_t trigger_flag = 0;
moving_Average_Filter bullet_speed_avg = {
        .length=30,
};
bool thisReverseFlag = false, lastReverseFlag = false;
fp32 Fire_Speed = 6400;
int next_goal = 0, block_flag = false;   //记录单发堵转的目标ecd，以及是否进行单发堵转处理
float current_speed, last_speed, avg_speed;//记录当前子弹射速和上一发子弹射速，平均射速
float launch_add = 0;//进行动态射速调整的发射pid增加值
void launcher_control() {

    if (gimbal.mode == GIMBAL_RELAX ||
        (detect_list[DETECT_REMOTE].status == OFFLINE && detect_list[DETECT_VIDEO_TRANSIMITTER].status == OFFLINE)) {
        launcher_relax_handle();
    } else {
        if (launcher.fire_mode == Fire_ON) {
            launcher.fire_l.speed = -Fire_Speed;
            launcher.fire_r.speed = Fire_Speed;
            if (launcher.trigger_cmd == SHOOT_CLOSE) {
                launcher.trigger.speed = 0;
            } else if (launcher.trigger_cmd == SHOOT_SINGLE) //收到单发命令
            {
                launcher.trigger_cmd = SHOOT_ING;//进入正在单发状态
                if (block_flag) {
                    total_ecd_ref = next_goal;
                    block_flag = false;
                } else {
                    total_ecd_ref = launcher.trigger.motor_measure->total_ecd + DEGREE_45_TO_ENCODER;//单发拨动45
                }
                average_add(&bullet_speed_avg, Referee.ShootData.bullet_speed);
            } else if (launcher.trigger_cmd == SHOOT_DOUBLE) { //收到双发命令
                launcher.trigger_cmd = SHOOT_ING;//进入正在双发状态
                if (block_flag) {
                    total_ecd_ref = next_goal;
                    block_flag = false;
                } else {
                    total_ecd_ref = launcher.trigger.motor_measure->total_ecd + 2 * DEGREE_45_TO_ENCODER;//双发拨动90
                }
                average_add(&bullet_speed_avg, Referee.ShootData.bullet_speed);
            } else if (launcher.trigger_cmd == SHOOT_ING) //单发状态
            {
                //判断是否到达目标位置
                if (KeyBoard.Mouse_l.status != KEY_PRESS) //鼠标左键单击松手
                {
                    launcher.trigger_cmd = SHOOT_CLOSE;
                    launcher.trigger.speed = 0;
                }
            }
            launcher.trigger.speed = pid_calc(&launcher.trigger.angle_p, launcher.trigger.motor_measure->total_ecd,
                                              total_ecd_ref);
//注：单发堵转应对逻辑
//单发堵转后，电机反转一段时间后停止转动，记录当前一发的目标ecd值，
//在下次进入单发状态时，将目标ecd值设置为上一发记录的目标ecd值，
// 即完成视觉的要求，单发堵转时，堵的那一发不射出来，后续正常单发
            if (launcher.trigger_cmd != SHOOT_CONTINUES) {
                trigger_block_handle(1000, 80, 4000);
                laucher_heat_control();
                thisReverseFlag = reverse_flag;
                if (lastReverseFlag && !thisReverseFlag) {
                    next_goal = total_ecd_ref;
                    block_flag = true;
                    total_ecd_ref = launcher.trigger.motor_measure->total_ecd;
                }
                lastReverseFlag = thisReverseFlag;

            }
            if (launcher.trigger_cmd == SHOOT_CONTINUES) {
                block_flag = false;
                total_ecd_ref = launcher.trigger.motor_measure->total_ecd;
                launcher.trigger.speed = TRIGGER_CONTINUES_SPEED;
                trigger_block_handle(1000, 100, 4000);
                laucher_heat_control();
            }
        } else {
            launcher.fire_l.speed = 0;
            launcher.fire_r.speed = 0;
            launcher.trigger.speed = 0;
            total_ecd_ref = launcher.trigger.motor_measure->total_ecd;
            launcher.trigger_cmd = SHOOT_CLOSE;
        }

        if (ABS(total_ecd_ref - launcher.trigger.motor_measure->total_ecd) > 2000) {
            trigger_flag = 1;
        } else {
            trigger_flag = 0;
        }
    }


    //动态射速调整
    current_speed = Referee.ShootData.bullet_speed;
    if (current_speed != last_speed) {                //调试发现每发射速不一致，这里基本能保证每射出一发子弹，就进一次队列
        if (QueueSize(&launchQueue, &avg_speed) < 10) {//保持队列长度最大为10
            QueuePush(&launchQueue, current_speed);
        } else {
            QueuePush(&launchQueue, current_speed);
            QueuePop(&launchQueue);
        }
        if (QueueSize(&launchQueue, &avg_speed) > 7) {
//            launch_add+=(TARGET_SHOOT_SPEED-avg_speed)*DYNAMIC_SPEED_RATIO;
        }
    }
    //对动态射速调整输出限幅
    VAL_LIMIT(launch_add, -7000, 7000);
    last_speed = current_speed;


    launcher.trigger.give_current = pid_calc(&launcher.trigger.speed_p, launcher.trigger.motor_measure->speed_rpm,
                                             launcher.trigger.speed);
//    launcher.fire_l.give_current =  pid_calc(&launcher.fire_l.speed_p, launcher.fire_l.motor_measure->speed_rpm,
//                                             launcher.fire_l.speed);
//    launcher.fire_r.give_current =  pid_calc(&launcher.fire_r.speed_p, launcher.fire_r.motor_measure->speed_rpm,
//                                             launcher.fire_r.speed);
    launcher.fire_l.give_current = VSP_PID_Calc(&launcher.fire_l.feedforward, launcher.fire_l.motor_measure->speed_rpm,
                                                launcher.fire_l.speed);
    launcher.fire_r.give_current = VSP_PID_Calc(&launcher.fire_r.feedforward, launcher.fire_r.motor_measure->speed_rpm,
                                                launcher.fire_r.speed);
}

//发射机构失能
void launcher_relax_handle() {
    launcher.trigger.give_current = 0;
    launcher.fire_r.give_current = 0;
    launcher.fire_l.give_current = 0;
    launcher.trigger.speed = 0;
    launcher.fire_r.speed = 0;
    launcher.fire_l.speed = 0;
}

fp32 left_vsp_pid[PARAMS];
fp32 right_vsp_pid[PARAMS];
int hot = 0;

void laucher_heat_control() {
    if ((Referee.GameRobotStat.shooter_barrel_heat_limit - Referee.PowerHeatData.shooter_heat0) <= 30) {
        Cooling_time = HAL_GetTick();
        hot = 1;
    }
    if (HAL_GetTick() - Cooling_time <= 3000) {
        launcher.trigger.give_current = 0;
        launcher.trigger.speed = 0;
    } else {
        hot = 0;
    }

    if (KeyBoard.X.status == KEY_CLICK) {
        Fire_Speed -= 100;
    }
    if (KeyBoard.C.status == KEY_CLICK) {
        Fire_Speed += 100;
    }
}

void launcher_init() {
    blocked_flag = false;//堵转标志位置0

    reverse_flag = false;//反转标志位置0

    launcher.trigger_cmd = SHOOT_CLOSE;//初始时发射机构默认关闭

    launcher.fire_last_mode = Fire_OFF;//初始时摩擦轮默认关闭

    launcher.fire_mode = Fire_OFF;//初始时摩擦轮默认关闭

    //获取发射机构电机数据结构体
    launcher.fire_l.motor_measure = &motor_3508_measure[FIRE_L];
    launcher.fire_r.motor_measure = &motor_3508_measure[FIRE_R];
    launcher.trigger.motor_measure = &motor_2006_measure[TRIGGER];

    //发射机构电机PID初始化
    launcher.fire_r.speed_p.p = SHOOT_FIRE_R_PID_KP;
    launcher.fire_r.speed_p.i = SHOOT_FIRE_R_PID_KI;
    launcher.fire_r.speed_p.d = SHOOT_FIRE_R_PID_KD;
    launcher.fire_r.speed_p.max_output = SHOOT_FIRE_R_PID_MAX_OUT;
    launcher.fire_r.speed_p.integral_limit = SHOOT_FIRE_R_PID_MAX_IOUT;

    launcher.fire_l.speed_p.p = SHOOT_FIRE_L_PID_KP;
    launcher.fire_l.speed_p.i = SHOOT_FIRE_L_PID_KI;
    launcher.fire_l.speed_p.d = SHOOT_FIRE_L_PID_KD;
    launcher.fire_l.speed_p.max_output = SHOOT_FIRE_L_PID_MAX_OUT;
    launcher.fire_l.speed_p.integral_limit = SHOOT_FIRE_L_PID_MAX_IOUT;

    launcher.trigger.angle_p.p = SHOOT_TRI_ANGLE_PID_KP;
    launcher.trigger.angle_p.i = SHOOT_TRI_ANGLE_PID_KI;
    launcher.trigger.angle_p.d = SHOOT_TRI_ANGLE_PID_KD;
    launcher.trigger.angle_p.max_output = SHOOT_TRI_ANGLE_PID_MAX_OUT;
    launcher.trigger.angle_p.integral_limit = SHOOT_TRI_ANGLE_PID_MAX_IOUT;

    launcher.trigger.speed_p.p = SHOOT_TRI_SPEED_PID_KP;
    launcher.trigger.speed_p.i = SHOOT_TRI_SPEED_PID_KI;
    launcher.trigger.speed_p.d = SHOOT_TRI_SPEED_PID_KD;
    launcher.trigger.speed_p.max_output = SHOOT_TRI_SPEED_PID_MAX_OUT;
    launcher.trigger.speed_p.integral_limit = SHOOT_TRI_SPEED_PID_MAX_IOUT;

    launcher.trigger.motor_measure->total_ecd = launcher.trigger.motor_measure->offset_ecd = launcher.trigger.motor_measure->ecd;
    //最开始的编码值作为拨轮电机的校准值
    first_order_filter_init(&filter_trigger_rpm_in, 1, 1);

    left_vsp_pid[KP] = 30, left_vsp_pid[KI] = 0, left_vsp_pid[KD] = 0, left_vsp_pid[MAX_INTEGRAL] = 0, left_vsp_pid[MAX_OUTPUT] = 16000;
    right_vsp_pid[KP] = 30, right_vsp_pid[KI] = 0, right_vsp_pid[KD] = 0, right_vsp_pid[MAX_INTEGRAL] = 0, right_vsp_pid[MAX_OUTPUT] = 16000;

    VSP_PID_Init(&launcher.fire_l.feedforward,
                 left_vsp_pid,
                 SHOOT_FIRE_L_K0,
                 SHOOT_FIRE_L_K1,
                 SHOOT_FIRE_L_K2,
                 SHOOT_FIRE_L_MINKP);

    VSP_PID_Init(&launcher.fire_r.feedforward,
                 right_vsp_pid,
                 SHOOT_FIRE_R_K0,
                 SHOOT_FIRE_R_K1,
                 SHOOT_FIRE_R_K2,
                 SHOOT_FIRE_R_MINKP);
}

//判断是否堵转
/**
 *
 * @param blockSpeed 当转速差距大于blockSpeed时，判定堵转
 * @return 是否堵转
 */
bool is_blocked(float blockSpeed) {
    //在标识位为0时，电机转速低于阈值时，判定堵转开始
    if (blocked_flag == false &&
        abs(launcher.trigger.motor_measure->speed_rpm - launcher.trigger.speed) > blockSpeed) {
        blocked_start_time = HAL_GetTick();//获取堵转开始时间
        blocked_flag = true;
    }

    //标识位为1时，已经开始堵转，判断是否堵转达到一定时间，若达到，则判定堵转
    if (blocked_flag == true &&
        abs(launcher.trigger.motor_measure->speed_rpm - launcher.trigger.speed) > blockSpeed) {
        if (CONTINUES_BLOCKED_JUDGE()) {
            blocked_flag = false;
            return true;
        }
    } else {
        blocked_flag = false;
    }
    return false;
}

/**
 * 都转处理函数
 * @param blockSpeed 期望速度和实际速度相差超过blockSpeed时判定堵转
 * @param reverse_time 反转持续时间
 * @param reverseSpeed 反转速度
 */
void trigger_block_handle(fp32 blockSpeed, fp32 reverse_time, fp32 reverseSpeed) {
    //判断堵转并且反转标识为0时
    if ((is_blocked(blockSpeed) && reverse_flag == false)) {
        reverse_flag = true;//判定开始反转
        reverse_start_time = HAL_GetTick();//获取开始反转时间
    }

    //判定反转开始并且时间没有达到反转结束时间
    if (reverse_flag == true && (HAL_GetTick() - reverse_start_time < reverse_time)) {
        launcher.trigger.speed = reverseSpeed;//拨单电机设置为反转速度
    } else {
        reverse_flag = false;
    }
}