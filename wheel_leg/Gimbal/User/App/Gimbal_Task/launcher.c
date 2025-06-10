#include "launcher.h"
#include "gimbal_task.h"
#include "key_board.h"
#include "protocol_balance.h"

/**
 *
 * 发射机构：摩擦轮只需要负责高速旋转，发射模式(单发、连发、堵转检测)主要由拨盘判断
 *
**/

/*********************************************************************************************************
*                                              内部变量                                                   *
*********************************************************************************************************/
launcher_t launcher;                    // 发射机构数据
extern robot_ctrl_info_t robot_ctrl;    // 上位机数据
static fp32 trigger_target_total_ecd = 0;      // 总拨盘电机转动ECD
static uint32_t trigger_time = 0;       // 拨盘电机每次转动时间
static uint8_t rc_last_sw_L;            // 拨杆上一时刻的状态值记录

/** 为每个电机定义状态变量 **/
SystemState state_l = {0};
SystemState state_r = {0};

/** 滑膜控制系数 **/
double dt = 0.001;
SMC_Params params =
        {
                .c = 5,             // 滑模面系数（决定收敛速度）
                .rho = 2.0,         // 切换增益（需大于扰动幅度）
                .epsilon = 1,       // 边界层厚度（抑制抖振）
                .max_i = 10000      // 控制输入限幅
        };

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static void Trigger_Mode_Set(void);
static void Trigger_Control(void);
static void Trigger_Finish_Judge(void);
static void Launcher_Current_Calc(void);

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
//TODO: 非也非也，还得测试角度发和速度发哪个准度更高，以及要不要做一定频率的双连发
// #define ANGLE // 代码基于Hero修改，因为它的发弹是角度环，所以会有个ANGLE宏定义
#define BRUSTS // 步兵的发弹只要一直转就行

// #define BLOCK_SPEED   // 反转的时候固定时间转动固定速度，不会转动失败
#define BLOCK_ANGLE   // 反转的时候固定角度转动固定时间，会转动失败


/**
 * 拨盘在发射机构开启时实现的模式，主要为SHOOT_READLY(单发)、SHOOT_BURSTS(连发)两种发射模式
 * 以及拨盘的模式判断，主要判断在SHOOT_OVER和SHOOT_BURSTS，SHOOT_FAIL两种模式下会做出什么决策
 */
static uint32_t time = 0;
static uint32_t last_time = 0;


static void Trigger_Mode_Set() {
    if (switch_is_down(rc_last_sw_L) || (KeyBoard.Mouse_l.status == KEY_PRESS))
    {
        launcher.trigger_mode = SHOOT_CONTINUE;
    }

    /* 拨盘处于正常模式才执行下一次的指令 */
    if((launcher.trigger_mode == SHOOT_CONTINUE) || (launcher.trigger_mode == SHOOT_OVER))
    {
#ifdef ANGLE
        time = HAL_GetTick();
        /* 遥控器左键往下拨或长按鼠标左键，每1s转45度，用于调试 */
        else if ((switch_is_down(rc_last_sw_L) || (KeyBoard.Mouse_l.status == KEY_PRESS)) && ((time - last_time) > 1000))
        {
            last_time = HAL_GetTick();
            launcher.trigger_state = SHOOT_READLY;
        }
#endif //!ANGLE
#ifdef BRUSTS
        /* 遥控器左键在下面或长按鼠标左键，以200的转速连续发射弹丸 */
        /** Q: 检测上一次的左拨杆在下面能证明左键是向下拨吗 **/
        /** A: 这里修正一下，是判断遥控器左键在不在最下面，在最下面就是连发 **/

#endif //!BRUSTS
        if ((gimbal.mode == GIMBAL_AUTO) && (robot_ctrl.fire_command == 1))
        {/** 当云台为自瞄模式且接收到视觉火控开启标志位为1时，准备进入单发模式，即视觉发一帧火控数据拨盘就转动一个弹丸角度 **/
            launcher.trigger_mode = SHOOT_READY_TO_SINGLE;
        }
    }
        /* 当拨盘正反转都失败后，将拨盘失能 */
    else if(launcher.trigger_mode == SHOOT_FAIL)
    {
        launcher.trigger.target_speed = 0;
        launcher.trigger.target_current = 0;
        trigger_target_total_ecd = launcher.trigger.motor_measure.total_ecd;
    }
}


/**
 * 拨盘的模式控制逻辑，主要为SHOOT_BURSTS，SHOOT_READLY，SHOOT_BLOCK模式的实现
 */
static void Trigger_Control(void) {

    /** 连发模式 **/
    if(launcher.trigger_mode == SHOOT_CONTINUE)
    {
        // 设置拨盘期望转速
        launcher.trigger.target_speed = TRIGGER_SPEED;

        // 连发时让期望总编码器值总与实际反馈总编码器值相等，便于后续切换单发模式
        trigger_target_total_ecd = launcher.trigger.motor_measure.total_ecd;
    }
    /** 读取单发指令，预备进入单发模式 **/
    else if (launcher.trigger_mode == SHOOT_READY_TO_SINGLE)
    {
        trigger_time = HAL_GetTick(); // 这时候开始计时，开始转的时候计时

        // 开始单发处理
        trigger_target_total_ecd = launcher.trigger.motor_measure.total_ecd - DEGREE_45_TO_ENCODER;

        // 切换为单发模式
        launcher.trigger_mode = SHOOT_SINGLE; // 处于单发中还未执行完成
    }
        /** 如果拨盘堵转，则切换为反转模式 **/
    else if (launcher.trigger_mode == SHOOT_BLOCK)
    {
#ifdef BLOCK_ANGLE
        trigger_time = HAL_GetTick(); // 这时候开始计时，开始转的时候计时(反转也需要重新计时)

        // Q: 我以为是以固定速度反转，原来是每隔一段时间变化一定角度吗?
        // A: 这里也可以改成固定速度反转固定时间，不过我写的是固定时间反转固定角度，可以写一个第一种方案测试哪个效果好
        trigger_target_total_ecd += DEGREE_90_TO_ENCODER;

        // 切换为反转模式
        launcher.trigger_mode = SHOOT_INVERSING;
#endif //!BLOCK_ANGLE

#ifdef BLOCK_SPEED
        // 设置拨盘期望转速
        launcher.trigger.target_speed = -TRIGGER_SPEED;
#endif //!BLOCK_SPEED
    }

    /* 只有在不处于连续发射的状态下才会进入角度环模式，连续发射会直接定义拨盘转速 */
    if(launcher.trigger_mode != SHOOT_CONTINUE && launcher.trigger_mode != SHOOT_BLOCK) {
        /** 拨盘的位置环pid **/
        launcher.trigger.target_speed = pid_calc(&launcher.trigger.angle_p,
                                                 launcher.trigger.motor_measure.total_ecd,
                                                 trigger_target_total_ecd);
    }
}

/**
 * 识别到在 SHOOT_READLY 时进入 SHOOT_SINGLE
 * 在规定时间内(1s)转动 ecd 差值小于 5000 则为完成单发任务 ????????????????????????????????
 * 在规定时间内(0.5s)转动 ecd 差值未达到规定则为堵转，触发堵转任务 ??????????????????????????
 * 反转未成功会再反转一次(此时可能会导致电流过大损坏电机)
 */
#define TRI_MINECD 5000     // 5000ecd
#define TRI_MAXTIME 1000    // 1s
#define TRI_MAXSPEED 100    // rpm
static fp32 total_time = 0;
static fp32 total_ecd_error = 0;
static uint32_t continue_time = 0;

static void Trigger_Finish_Judge() {
    if(launcher.trigger_mode != SHOOT_CLOSE) {
        /* 这里记录的是拨盘完成一次单发使用的时间 */
        total_time = HAL_GetTick() - trigger_time;
        /* 这里记录的是拨盘当前总编码器值与我们期望它转动到的总编码器值的差值 */
        total_ecd_error = ABS(trigger_target_total_ecd - launcher.trigger.motor_measure.total_ecd);

        /***********************        判断单发模式和连发模式时拨盘是否卡弹       *************************************/

        // Q：对(total_ecd_error > TRI_MINECD)的判断是为了什么， 大于时说明什么?
        // A：total_ecd_error的物理意义是当前电机的实时总ECD与目标总ECD的差值，如果差值还大于TRI_MINECD就说明还没有完成转动任务，需要继续停留在单发执行任务里面
        // A: 当然不是差值一直大于TRI_MINECD都判断为单发未完成，如果在TRI_MAXTIME时间内都没有完成就会认为他堵转了，将状态改为单发堵转
        // A: 所以这里判断是SHOOT_SINGLE还是SHOOT_BLOCK的关键在于total_time的判断

        /** 单发 **/
        if(total_ecd_error > TRI_MINECD) // 说明还未完成转动任务，需要继续停留在单发执行任务
        {
            if((launcher.trigger_mode == SHOOT_SINGLE) && (total_time < TRI_MAXTIME))
            {/* 未超过规定最大时间，则维持单发模式不变 */
                launcher.trigger_mode = SHOOT_SINGLE;
            }
            else if((launcher.trigger_mode == SHOOT_SINGLE) && (total_time > TRI_MAXTIME))
            {/* 如果在单发状态里面待的时间太长,则判断其为堵转状态 */
                launcher.trigger_mode = SHOOT_BLOCK;
            }
        }
         // 单发顺利
        else if((total_ecd_error < TRI_MINECD) && (launcher.trigger_mode != SHOOT_CONTINUE))
        {
            launcher.trigger_mode = SHOOT_OVER;
        }

        /** 连发 **/
        else if(launcher.trigger_mode == SHOOT_CONTINUE)
        {
            /* 如果在连发状态下的反馈转速小于TRI_MAXSPEED持续一段时间，则判断其为堵转状态 */
            if(ABS(launcher.trigger.motor_measure.speed_rpm) < TRI_MAXSPEED) {
                continue_time++;
            }
            else{
                continue_time = 0;
            }

            if(continue_time > 200)
            {
                launcher.trigger_mode = SHOOT_BLOCK;
            }
        }

        /************************************        End       **************************************************/


        /***********************        判断反转状态时是否解决卡弹       *************************************/

#ifdef BLOCK_ANGLE
        //* 反转逻辑需要测试平步的拨盘是否能实现反转这个功能，才会考虑添加，需要测试，如果不行的话要考虑第二个方案反方向转动固定时间 */
        if(total_ecd_error > TRI_MINECD) {
            if((launcher.trigger_mode == SHOOT_INVERSING) && (total_time < TRI_MAXTIME))
            {// 实际反转时间未超过最大反转时间，则维持反转模式不变
                launcher.trigger_mode = SHOOT_INVERSING;
            }
            else if((launcher.trigger_mode == SHOOT_INVERSING) && (total_time > TRI_MAXTIME))
            {// 如果反转时间超过了规定的最大反转时间，则视为解决失败
                launcher.trigger_mode = SHOOT_FAIL;
            }
        }
            /* 在规定时间内完成则将状态改为发射完成 */
            // total_ecd_error刚刚说过了是当前电机的实时总ECD与目标总ECD的差值，那么总差值小于TRI_MINECD
            // 就说明已经转动到我们想要他达到的转动角度，所以判断为完成一次单发
        else if((total_ecd_error < TRI_MINECD) && (launcher.trigger_mode != SHOOT_CONTINUE))
        {
            launcher.trigger_mode = SHOOT_OVER;
        }
#endif //!BLOCK_ANGLE

#ifdef BLOCK_SPEED
        /* TRI_MAXTIME时间内都保持在反转的状态，TRI_MAXTIME后将模式改为完成模式再进行下一次程序判断 */
        if((launcher.trigger_mode == SHOOT_BLOCK) && (total_time < TRI_MAXTIME))
        {// 实际反转时间未超过最大反转时间，则维持反转模式不变
            launcher.trigger_mode = SHOOT_BLOCK;
        }
        else if((launcher.trigger_mode == SHOOT_BLOCK) && (total_time > TRI_MAXTIME))
        {// 如果反转时间超过了规定的最大反转时间，则视为解决失败
            launcher.trigger_mode = SHOOT_OVER;
        }
#endif //!BLOCK_SPEED
    }
}


/** 计算发射机构电流 **/
static void Launcher_Current_Calc(void) {

    /** 拨盘电流计算(串级pid 位置环在别的地方) **/
    launcher.trigger.target_current = (int16_t)pid_calc(&launcher.trigger.speed_p,
                                                        launcher.trigger.motor_measure.speed_rpm,
                                                        launcher.trigger.target_speed);

    /** 摩擦轮电流计算 **/
    state_l = update_system(launcher.fire_l.target_speed, launcher.fire_l.motor_measure.speed_rpm, dt, state_l);
    launcher.fire_l.target_current = -smc_controller(state_l, params);

    state_r = update_system(launcher.fire_r.target_speed, launcher.fire_r.motor_measure.speed_rpm, dt, state_r);
    launcher.fire_r.target_current = -smc_controller(state_r, params);
}


/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/

/** 发射机构初始化 **/
void Launcher_Init(void) {

    /** 摩擦轮默认为失能模式 **/
    launcher.fir_wheel_mode = launcher.fir_wheel_last_mode = Fire_OFF;

    /** 拨盘默认为失能模式 **/
    launcher.trigger_mode = launcher.trigger_last_mode = SHOOT_CLOSE;

    /* 默认此时拨盘的初始ECD等于当前ECD, 提高发射准度 */
    /* 以后再看 似乎与上电回中有关 */
    launcher.trigger.motor_measure.total_ecd = launcher.trigger.motor_measure.offset_ecd = launcher.trigger.motor_measure.ecd;

    /** 拨盘电机pid **/
    // 位置环pid
    pid_init(&launcher.trigger.angle_p,
             TRIGGER_ANGLE_PID_MAX_OUT,
             TRIGGER_ANGLE_PID_MAX_IOUT,
             TRIGGER_ANGLE_PID_KP,
             TRIGGER_ANGLE_PID_KI,
             TRIGGER_ANGLE_PID_KD);

    // 速度环pid
    pid_init(&launcher.trigger.speed_p,
             TRIGGER_SPEED_PID_MAX_OUT,
             TRIGGER_SPEED_PID_MAX_IOUT,
             TRIGGER_SPEED_PID_KP,
             TRIGGER_SPEED_PID_KI,
             TRIGGER_SPEED_PID_KD);

    /* 发射机构期望电流置零 */
    launcher.fire_l.target_current = 0;
    launcher.fire_r.target_current = 0;
    launcher.trigger.target_current = 0;

    /* 最开始的编码值作为拨轮电机的校准值 */
    /* 以后再看 */
    first_order_filter_init(&launcher.filter_fire,0.05f, 0.5f);
    first_order_filter_init(&launcher.filter_trigger,1,1);
}

/** 我觉得这个函数本质上是对摩擦轮模式进行判断，是因为顺带在摩擦轮模式为开启时进行了拨盘的模式判断，引起了发弹，所以被定义为发射模式设置吗? **/
/** 看不懂下面有讲解 看不懂下面有讲解 看不懂下面有讲解 看不懂下面有讲解 看不懂下面有讲解 看不懂下面有讲解 看不懂下面有讲解 看不懂下面有讲解 **/
void Launcher_Mode_Set() {
    /**
     * 当左边拨盘上一个模式不在上面且此刻模式在上面时进入发射机构模式切换（即开变为关，关变为开）
     * 即将当Q为0时将其变为1，Q为1时将其变为0
     * Q=1代表摩擦轮开启（Fire_ON），Q=0代表摩擦轮关闭（Fire_OFF）
     * 只有在摩擦轮开启的时候才可以进入拨盘模式判断
     **/

    /**********************************    摩擦轮模式判断    ************************************/

    /** 决定 Q键 的值 **/
    if ((!switch_is_up(rc_last_sw_L)) && switch_is_up(rc_ctrl.rc.s[RC_s_L]))
    {
        // 不懂键盘
        if ((KeyBoard.Q.click_flag == 1) && (gimbal.mode != GIMBAL_DISABLE))
        {
            KeyBoard.Q.click_flag = 0;
        }
        else if ((KeyBoard.Q.click_flag == 0) && (gimbal.mode != GIMBAL_DISABLE))
        {
            KeyBoard.Q.click_flag = 1;
        }
    }

    /** 根据上面对 Q键 的值进行判断，决定摩擦轮是否开启 **/
    if ((KeyBoard.Q.click_flag == 1) && (gimbal.mode != GIMBAL_DISABLE))
    {
        launcher.fir_wheel_mode = Fire_ON;
    }
    else if ((KeyBoard.Q.click_flag == 0) && (gimbal.mode != GIMBAL_DISABLE))
    {
        launcher.fir_wheel_mode = Fire_OFF;
    }

    /**********************************      End      **************************************/



    /**********************************    拨盘模式判断    ************************************/

    // 如果只有摩擦轮转动是无法发弹的，同理，如果摩擦轮不转动，也无法发弹。
    // 所以只在摩擦轮开启时才进行拨盘模式判断, 发射模式(单发、连发)是由拨盘决定的 **/
    if (launcher.fir_wheel_mode == Fire_ON)
    {
        Trigger_Mode_Set();
    }
    else if (launcher.fir_wheel_mode == Fire_OFF)
    {// 如果摩擦轮关闭，则将拨盘也关闭
        launcher.trigger_mode = SHOOT_CLOSE;
    }

    /** 更新上一次的左拨杆值 **/
    rc_last_sw_L = rc_ctrl.rc.s[RC_s_L];

    /**********************************      End      **************************************/



    /** Important Important Important Important Important Important Important Important Important
 *
 * Tips:    ① 判断逻辑只与左边拨杆有关
 *          ② KeyBoard.Q.click_flag可由遥控器或图传控制，默认是0, 即《未按下Q键》
 *          ③ 只有当《上一时刻左边拨杆不在最上面，而下一时刻在最上面》时，才会进行发射机构模式判断
 *
 * Q1: 摩擦轮是如何被开启的?
 * A1: 当你将左拨杆从任意位置(特指中间或下面)拨到最上面时，成功进入判断，并进入 else if 分支，KeyBoard.Q.click_flag 被置1， 摩擦轮开启
 *
 *
 * Q2: 摩擦轮在开启后如何被关闭？
 * A2；需要关闭摩擦轮的情况这里分两种讨论：
 *
 *    ① 只开了摩擦轮，没开拨盘：   当你刚打开摩擦轮的时候，你的左拨杆肯定是在最上面的，这个时候 rc_last_sw_L = rc_ctrl.rc.s[RC_s_L] = up，
 *                          就不会再进入Q键的判断，因此摩擦轮则会保持开启模式，这时你将左拨杆拨到中间，rc_last_sw_L = rc_ctrl.rc.s[RC_s_L] = mid,
 *                          再将左拨杆拨到上面，必定存在一个时刻为《rc_last_sw_L = mid 而 rc_ctrl.rc.s[RC_s_L] = up》，
 *                          此时会再度进入Q键的判断，因为上次打开摩擦轮时 KeyBoard.Q.click_flag 被置1了，所以这次的判断会被置0，摩擦轮就被关闭了
 *
 *    ② 摩擦轮和拨盘都开了，打完弹想关闭摩擦轮：      打弹的时候，你的左拨杆应该是在最下面的，这时，你将左拨杆打到最上面，
 *                                          必定存在一个时刻为《rc_last_sw_L = mid 而 rc_ctrl.rc.s[RC_s_L] = up》，
 *                                          此时会再度进入Q键的判断，因为上次打开摩擦轮时 KeyBoard.Q.click_flag 被置1了，所以这次的判断会被置0，摩擦轮就被关闭了
 *
 *
 * **/



}


/**
 * 发射机构的逻辑实现
 */
void Launcher_Control(void) {

    /** 云台失能时 **/
    if (gimbal.mode == GIMBAL_DISABLE)
    {
        Launcher_Disable();
    }
    else {
        if (launcher.fir_wheel_mode == Fire_ON) {
            launcher.fire_r.target_speed = FIRE_SPEED_R;
            launcher.fire_l.target_speed = -FIRE_SPEED_L;
            Trigger_Control();
        }
        else if(launcher.fir_wheel_mode == Fire_OFF)
        {/** 发射机构失能时 关闭摩擦轮和拨盘 **/
            // Q: 为什么不在计算完后再进行发射机构失能判断？ 直接把计算后的结果置0然后发出去
            launcher.fire_l.target_speed = 0;
            launcher.fire_r.target_speed = 0;
            launcher.trigger.target_speed = 0;

            launcher.trigger.target_current = 0;

            trigger_target_total_ecd = launcher.trigger.motor_measure.total_ecd;
        }

        /** 发射机构电流控制计算 **/
        Launcher_Current_Calc();

        /* 堵转检测 */
        Trigger_Finish_Judge();
    }
}

/** 发射机构失能 **/
void Launcher_Disable(void) {

    /** 关闭摩擦轮 **/
    launcher.fir_wheel_mode = Fire_OFF;

    /** 其实这句可以省略，因为把摩擦轮关闭后，在Launcher_Mode_Set函数的判断里也会关闭拨盘，但是为了完整性还是保留吧 **/
    launcher.trigger_mode = SHOOT_CLOSE;

    /** 左右摩擦轮和拨盘电流置零 **/
    launcher.fire_r.target_current = 0;
    launcher.fire_l.target_current = 0;
    launcher.trigger.target_current = 0;

    /** 失能时令期望总编码器值总等于反馈的总编码器值，便于下次发射 **/
    trigger_target_total_ecd = (int32_t)launcher.trigger.motor_measure.total_ecd;
}