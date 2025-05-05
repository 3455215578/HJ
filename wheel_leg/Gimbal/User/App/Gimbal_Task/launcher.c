/*********************************************************************************************************
*                                              ����ͷ�ļ�
*********************************************************************************************************/
#include "launcher.h"
#include "gimbal_task.h"
#include "key_board.h"
#include "protocol_balance.h"
#include "Balance.h"
#include "gimbal_task.h"
#include "../../Algorithm/SMC/SMC.h"

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/
/* ��Ħ����ת�� PID */
#define SHOOT_FIRE_L_PID_KP         8//15//10//30 //80
#define SHOOT_FIRE_L_PID_KI         0.f
#define SHOOT_FIRE_L_PID_KD         1.f//50.f
#define SHOOT_FIRE_L_PID_MAX_IOUT   100
#define SHOOT_FIRE_L_PID_MAX_OUT    20000

/* ��Ħ����ת�� PID */
#define SHOOT_FIRE_R_PID_KP         8//15//10//30//80
#define SHOOT_FIRE_R_PID_KI         0.f
#define SHOOT_FIRE_R_PID_KD         1.f//50.f
#define SHOOT_FIRE_R_PID_MAX_IOUT   100
#define SHOOT_FIRE_R_PID_MAX_OUT    20000

/* ���� PID */
#define TRIGGER_ANGLE_PID_KP        1.7f//0.2f//0.7f
#define TRIGGER_ANGLE_PID_KI        0.f
#define TRIGGER_ANGLE_PID_KD        0.5f//0.f//1.5f
#define TRIGGER_ANGLE_PID_MAX_IOUT  0
#define TRIGGER_ANGLE_PID_MAX_OUT   5000//3000

#define TRIGGER_SPEED_PID_KP        10.f//18.f//4.f//8.f
#define TRIGGER_SPEED_PID_KI        0.f
#define TRIGGER_SPEED_PID_KD        18.f
#define TRIGGER_SPEED_PID_MAX_IOUT  0
#define TRIGGER_SPEED_PID_MAX_OUT   10000//25000

/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/
launcher_t launcher;
extern robot_ctrl_info_t robot_ctrl;
static fp32 total_ecd_ref_tri = 0;
static uint32_t trigger_time = 0;

//��Ĥ����ϵ��
double dt = 0.001;
SMC_Params params = {
        .c = 5,             //��ģ��ϵ�������������ٶȣ�
        .rho = 2.0,         //�л����棨������Ŷ����ȣ�
        .epsilon = 1,       //�߽���ȣ����ƶ���
        .max_i = 10000      //���������޷�
};

//Ϊÿ���������״̬����
SystemState state_l = {0};
SystemState state_r = {0};

/*********************************************************************************************************
*                                              �ڲ���������
*********************************************************************************************************/
static void Trigger_Finish_Judge();

/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
/**
 * ʶ���� SHOOT_READLY ʱ���� SHOOT_SINGLE
 * �ڹ涨ʱ����(1s)ת�� ecd ��ֵС�� 5000 ��Ϊ��ɵ�������
 * �ڹ涨ʱ����(0.5s)ת�� ecd ��ֵδ�ﵽ�涨��Ϊ��ת��������ת����
 * ��תδ�ɹ����ٷ�תһ��(��ʱ���ܻᵼ�µ��������𻵵��)
 */
#define TRI_MINECD 5000
#define TRI_MAXTIME 1000
static fp32 total_time = 0;
static fp32 total_ecd = 0;
static void Trigger_Finish_Judge() {
    if(launcher.shoot_cmd != SHOOT_CLOSE) {
        /* ������ʱ�涨ʱ���ڲ�ֵ����, ���䱣���ڵ���״̬ */
        total_time = HAL_GetTick() - trigger_time;
        total_ecd = ABS(total_ecd_ref_tri - launcher.trigger.motor_measure.total_ecd);
        if(launcher.shoot_cmd == SHOOT_SINGLE &&
           total_ecd > TRI_MINECD && total_time < TRI_MAXTIME) {
            launcher.shoot_cmd = SHOOT_SINGLE;
        }
            /* �ڵ���״̬�������ʱ��̫��,�ж���Ϊ��ת״̬ */
        else if(launcher.shoot_cmd == SHOOT_SINGLE &&
                total_ecd > TRI_MINECD && total_time > TRI_MAXTIME) {
            launcher.shoot_cmd = SHOOT_BLOCK;
        }
        /* ��תʱ�涨ʱ���ڲ�ֵ����, ���䱣���ڷ�ת״̬ */
        if(launcher.shoot_cmd == SHOOT_BLOCK_BACK && total_time < TRI_MAXTIME) {
            launcher.shoot_cmd = SHOOT_BLOCK_BACK;
        }
            /* �ڷ�װ״̬�������ʱ��̫��,�ж���Ϊʧ��״̬ */
        else if(launcher.shoot_cmd == SHOOT_BLOCK_BACK && total_time > TRI_MAXTIME) {
            launcher.shoot_cmd = SHOOT_FAIL;
        }
            /* �ڹ涨ʱ���������״̬��Ϊ������� */
        else if(total_ecd < TRI_MINECD) {
            launcher.shoot_cmd = SHOOT_OVER;
        }
    }
}


/*********************************************************************************************************
*                                              API����ʵ��
*********************************************************************************************************/
/**
 * ���������ʼ��
  */
void Launcher_Init(void) {
    /* Ħ����״̬Ĭ�Ϲر� */
    launcher.fire_mode = Fire_OFF;
    launcher.fire_last_mode=Fire_OFF;
    /* ����״̬Ĭ�Ϲر� */
    launcher.shoot_last_cmd = SHOOT_CLOSE;
    launcher.shoot_cmd = SHOOT_CLOSE;

    /* Ĭ�ϴ�ʱ���̵ĳ�ʼECD���ڵ�ǰECD, ��߷���׼�� */
    launcher.trigger.motor_measure.total_ecd=launcher.trigger.motor_measure.offset_ecd=launcher.trigger.motor_measure.ecd;

    /* ����PID */
    pid_init(&launcher.trigger.angle_p,
             TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT, TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD);
    pid_init(&launcher.trigger.speed_p,
             TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD);

    /* ������ʼ��Ϊ0 */
    launcher.fire_l.give_current = 0;
    launcher.fire_r.give_current = 0;
    launcher.trigger.give_current = 0;

    /* �ʼ�ı���ֵ��Ϊ���ֵ����У׼ֵ */
    first_order_filter_init(&launcher.filter_fire,0.05f, 0.5f);
    first_order_filter_init(&launcher.filter_trigger,1,1);
}

/**
 * ����ģʽ�߼�
 */
static uint8_t rc_last_sw_L;
static uint32_t time = 0;
static uint32_t last_time = 0;
void Launcher_Mode_Set() {
    if((!switch_is_up(rc_last_sw_L)) && switch_is_up(rc_ctrl.rc.s[RC_s_L])) {
        if(KeyBoard.Q.click_flag==1 && gimbal.mode!= GIMBAL_RELAX) KeyBoard.Q.click_flag=0;
        else if(KeyBoard.Q.click_flag==0 && gimbal.mode!= GIMBAL_RELAX) KeyBoard.Q.click_flag=1;
    }

    if(KeyBoard.Q.click_flag==1 && gimbal.mode!= GIMBAL_RELAX) launcher.fire_mode=Fire_ON;
    else if (KeyBoard.Q.click_flag==0 && gimbal.mode!= GIMBAL_RELAX) launcher.fire_mode=Fire_OFF;

    if(launcher.fire_mode == Fire_ON) {
        time = HAL_GetTick();
        /* ң����������²��򳤰���������ÿ100ms����һ�ŵ��� */
        if((switch_is_down(rc_last_sw_L) || KeyBoard.Mouse_l.status == KEY_PRESS)
           && time - last_time > 100 && launcher.shoot_cmd == SHOOT_OVER) {
            last_time = HAL_GetTick();
            launcher.shoot_cmd = SHOOT_READLY;
        }
        else if(robot_ctrl.fire_command == 1 && gimbal.mode == GIMBAL_AUTO) {
            launcher.shoot_cmd = SHOOT_READLY;
        }
        else if(launcher.shoot_cmd == SHOOT_BLOCK || launcher.shoot_cmd == SHOOT_BLOCK_BACK ||
                launcher.shoot_cmd == SHOOT_FAIL || launcher.shoot_cmd == SHOOT_SINGLE) {}
        else launcher.shoot_cmd = SHOOT_OVER;
    }
    else if(launcher.fire_mode == Fire_OFF) {
        launcher.shoot_cmd=SHOOT_CLOSE;
    }
    rc_last_sw_L=rc_ctrl.rc.s[RC_s_L];
}


/**
 * ����������߼�ʵ��
  */
void Launcher_Control(void) {
    if(gimbal.mode == GIMBAL_RELAX) Launcher_Relax_Handle();
    else {
        if (launcher.fire_mode == Fire_ON) {
            launcher.fire_r.speed = FIRE_SPEED_R;
            launcher.fire_l.speed = -FIRE_SPEED_L;

            if (launcher.shoot_cmd == SHOOT_READLY) {
                trigger_time=HAL_GetTick(); //��ʱ��ʼ��ʱ����ʼת��ʱ���ʱ
                total_ecd_ref_tri = launcher.trigger.motor_measure.total_ecd - DEGREE_45_TO_ENCODER;
                launcher.shoot_cmd = SHOOT_SINGLE;   // ������
            }
            else if(launcher.shoot_cmd == SHOOT_BLOCK) {
                trigger_time=HAL_GetTick(); //��ʱ��ʼ��ʱ����ʼת��ʱ���ʱ(��תҲ��Ҫ���¼�ʱ)
                total_ecd_ref_tri = total_ecd_ref_tri + DEGREE_90_TO_ENCODER;
                launcher.shoot_cmd = SHOOT_BLOCK_BACK;
            }
            launcher.trigger.speed = pid_calc(&launcher.trigger.angle_p,
                                              launcher.trigger.motor_measure.total_ecd,
                                              total_ecd_ref_tri);
            launcher.trigger.give_current =
                    (int16_t)pid_calc(&launcher.trigger.speed_p,
                                      launcher.trigger.motor_measure.speed_rpm,
                                      launcher.trigger.speed);
        }
        else {
            launcher.fire_l.speed = 0;
            launcher.fire_r.speed = 0;
            launcher.trigger.give_current = 0;
            total_ecd_ref_tri=launcher.trigger.motor_measure.total_ecd;
        }

        state_l = update_system(launcher.fire_l.speed, launcher.fire_l.motor_measure.speed_rpm, dt, state_l);
        launcher.fire_l.give_current = -smc_controller(state_l, params);

        state_r = update_system(launcher.fire_r.speed, launcher.fire_r.motor_measure.speed_rpm, dt, state_r);
        launcher.fire_r.give_current = -smc_controller(state_r, params);

        Trigger_Finish_Judge(); // ��ת���
    }
}


void Launcher_Relax_Handle(void) {
    launcher.fire_mode = Fire_OFF;
    launcher.shoot_cmd = SHOOT_CLOSE;
    launcher.fire_r.give_current = 0;
    launcher.fire_l.give_current = 0;
    launcher.trigger.give_current = 0;
    total_ecd_ref_tri = (int32_t)launcher.trigger.motor_measure.total_ecd;
}