#include "launcher.h"
#include "gimbal_task.h"
#include "key_board.h"
#include "protocol_balance.h"

/**
 *
 * ���������Ħ����ֻ��Ҫ���������ת������ģʽ(��������������ת���)��Ҫ�ɲ����ж�
 *
**/

/*********************************************************************************************************
*                                              �ڲ�����                                                   *
*********************************************************************************************************/
launcher_t launcher;                    // �����������
extern robot_ctrl_info_t robot_ctrl;    // ��λ������
static fp32 trigger_target_total_ecd = 0;      // �ܲ��̵��ת��ECD
static uint32_t trigger_time = 0;       // ���̵��ÿ��ת��ʱ��
static uint8_t rc_last_sw_L;            // ������һʱ�̵�״ֵ̬��¼

/** Ϊÿ���������״̬���� **/
SystemState state_l = {0};
SystemState state_r = {0};

/** ��Ĥ����ϵ�� **/
double dt = 0.001;
SMC_Params params =
        {
                .c = 5,             // ��ģ��ϵ�������������ٶȣ�
                .rho = 2.0,         // �л����棨������Ŷ����ȣ�
                .epsilon = 1,       // �߽���ȣ����ƶ���
                .max_i = 10000      // ���������޷�
        };

/*********************************************************************************************************
*                                              �ڲ���������
*********************************************************************************************************/
static void Trigger_Mode_Set(void);
static void Trigger_Control(void);
static void Trigger_Finish_Judge(void);
static void Launcher_Current_Calc(void);

/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
//TODO: ��Ҳ��Ҳ�����ò��ԽǶȷ����ٶȷ��ĸ�׼�ȸ��ߣ��Լ�Ҫ��Ҫ��һ��Ƶ�ʵ�˫����
// #define ANGLE // �������Hero�޸ģ���Ϊ���ķ����ǽǶȻ������Ի��и�ANGLE�궨��
#define BRUSTS // �����ķ���ֻҪһֱת����

// #define BLOCK_SPEED   // ��ת��ʱ��̶�ʱ��ת���̶��ٶȣ�����ת��ʧ��
#define BLOCK_ANGLE   // ��ת��ʱ��̶��Ƕ�ת���̶�ʱ�䣬��ת��ʧ��


/**
 * �����ڷ����������ʱʵ�ֵ�ģʽ����ҪΪSHOOT_READLY(����)��SHOOT_BURSTS(����)���ַ���ģʽ
 * �Լ����̵�ģʽ�жϣ���Ҫ�ж���SHOOT_OVER��SHOOT_BURSTS��SHOOT_FAIL����ģʽ�»�����ʲô����
 */
static uint32_t time = 0;
static uint32_t last_time = 0;


static void Trigger_Mode_Set() {
    if (switch_is_down(rc_last_sw_L) || (KeyBoard.Mouse_l.status == KEY_PRESS))
    {
        launcher.trigger_mode = SHOOT_CONTINUE;
    }

    /* ���̴�������ģʽ��ִ����һ�ε�ָ�� */
    if((launcher.trigger_mode == SHOOT_CONTINUE) || (launcher.trigger_mode == SHOOT_OVER))
    {
#ifdef ANGLE
        time = HAL_GetTick();
        /* ң����������²��򳤰���������ÿ1sת45�ȣ����ڵ��� */
        else if ((switch_is_down(rc_last_sw_L) || (KeyBoard.Mouse_l.status == KEY_PRESS)) && ((time - last_time) > 1000))
        {
            last_time = HAL_GetTick();
            launcher.trigger_state = SHOOT_READLY;
        }
#endif //!ANGLE
#ifdef BRUSTS
        /* ң�������������򳤰�����������200��ת���������䵯�� */
        /** Q: �����һ�ε��󲦸���������֤����������²��� **/
        /** A: ��������һ�£����ж�ң��������ڲ��������棬��������������� **/

#endif //!BRUSTS
        if ((gimbal.mode == GIMBAL_AUTO) && (robot_ctrl.fire_command == 1))
        {/** ����̨Ϊ����ģʽ�ҽ��յ��Ӿ���ؿ�����־λΪ1ʱ��׼�����뵥��ģʽ�����Ӿ���һ֡������ݲ��̾�ת��һ������Ƕ� **/
            launcher.trigger_mode = SHOOT_READY_TO_SINGLE;
        }
    }
        /* ����������ת��ʧ�ܺ󣬽�����ʧ�� */
    else if(launcher.trigger_mode == SHOOT_FAIL)
    {
        launcher.trigger.target_speed = 0;
        launcher.trigger.target_current = 0;
        trigger_target_total_ecd = launcher.trigger.motor_measure.total_ecd;
    }
}


/**
 * ���̵�ģʽ�����߼�����ҪΪSHOOT_BURSTS��SHOOT_READLY��SHOOT_BLOCKģʽ��ʵ��
 */
static void Trigger_Control(void) {

    /** ����ģʽ **/
    if(launcher.trigger_mode == SHOOT_CONTINUE)
    {
        // ���ò�������ת��
        launcher.trigger.target_speed = TRIGGER_SPEED;

        // ����ʱ�������ܱ�����ֵ����ʵ�ʷ����ܱ�����ֵ��ȣ����ں����л�����ģʽ
        trigger_target_total_ecd = launcher.trigger.motor_measure.total_ecd;
    }
    /** ��ȡ����ָ�Ԥ�����뵥��ģʽ **/
    else if (launcher.trigger_mode == SHOOT_READY_TO_SINGLE)
    {
        trigger_time = HAL_GetTick(); // ��ʱ��ʼ��ʱ����ʼת��ʱ���ʱ

        // ��ʼ��������
        trigger_target_total_ecd = launcher.trigger.motor_measure.total_ecd - DEGREE_45_TO_ENCODER;

        // �л�Ϊ����ģʽ
        launcher.trigger_mode = SHOOT_SINGLE; // ���ڵ����л�δִ�����
    }
        /** ������̶�ת�����л�Ϊ��תģʽ **/
    else if (launcher.trigger_mode == SHOOT_BLOCK)
    {
#ifdef BLOCK_ANGLE
        trigger_time = HAL_GetTick(); // ��ʱ��ʼ��ʱ����ʼת��ʱ���ʱ(��תҲ��Ҫ���¼�ʱ)

        // Q: ����Ϊ���Թ̶��ٶȷ�ת��ԭ����ÿ��һ��ʱ��仯һ���Ƕ���?
        // A: ����Ҳ���Ըĳɹ̶��ٶȷ�ת�̶�ʱ�䣬������д���ǹ̶�ʱ�䷴ת�̶��Ƕȣ�����дһ����һ�ַ��������ĸ�Ч����
        trigger_target_total_ecd += DEGREE_90_TO_ENCODER;

        // �л�Ϊ��תģʽ
        launcher.trigger_mode = SHOOT_INVERSING;
#endif //!BLOCK_ANGLE

#ifdef BLOCK_SPEED
        // ���ò�������ת��
        launcher.trigger.target_speed = -TRIGGER_SPEED;
#endif //!BLOCK_SPEED
    }

    /* ֻ���ڲ��������������״̬�²Ż����ǶȻ�ģʽ�����������ֱ�Ӷ��岦��ת�� */
    if(launcher.trigger_mode != SHOOT_CONTINUE && launcher.trigger_mode != SHOOT_BLOCK) {
        /** ���̵�λ�û�pid **/
        launcher.trigger.target_speed = pid_calc(&launcher.trigger.angle_p,
                                                 launcher.trigger.motor_measure.total_ecd,
                                                 trigger_target_total_ecd);
    }
}

/**
 * ʶ���� SHOOT_READLY ʱ���� SHOOT_SINGLE
 * �ڹ涨ʱ����(1s)ת�� ecd ��ֵС�� 5000 ��Ϊ��ɵ������� ????????????????????????????????
 * �ڹ涨ʱ����(0.5s)ת�� ecd ��ֵδ�ﵽ�涨��Ϊ��ת��������ת���� ??????????????????????????
 * ��תδ�ɹ����ٷ�תһ��(��ʱ���ܻᵼ�µ��������𻵵��)
 */
#define TRI_MINECD 5000     // 5000ecd
#define TRI_MAXTIME 1000    // 1s
#define TRI_MAXSPEED 100    // rpm
static fp32 total_time = 0;
static fp32 total_ecd_error = 0;
static uint32_t continue_time = 0;

static void Trigger_Finish_Judge() {
    if(launcher.trigger_mode != SHOOT_CLOSE) {
        /* �����¼���ǲ������һ�ε���ʹ�õ�ʱ�� */
        total_time = HAL_GetTick() - trigger_time;
        /* �����¼���ǲ��̵�ǰ�ܱ�����ֵ������������ת�������ܱ�����ֵ�Ĳ�ֵ */
        total_ecd_error = ABS(trigger_target_total_ecd - launcher.trigger.motor_measure.total_ecd);

        /***********************        �жϵ���ģʽ������ģʽʱ�����Ƿ񿨵�       *************************************/

        // Q����(total_ecd_error > TRI_MINECD)���ж���Ϊ��ʲô�� ����ʱ˵��ʲô?
        // A��total_ecd_error�����������ǵ�ǰ�����ʵʱ��ECD��Ŀ����ECD�Ĳ�ֵ�������ֵ������TRI_MINECD��˵����û�����ת��������Ҫ����ͣ���ڵ���ִ����������
        // A: ��Ȼ���ǲ�ֵһֱ����TRI_MINECD���ж�Ϊ����δ��ɣ������TRI_MAXTIMEʱ���ڶ�û����ɾͻ���Ϊ����ת�ˣ���״̬��Ϊ������ת
        // A: ���������ж���SHOOT_SINGLE����SHOOT_BLOCK�Ĺؼ�����total_time���ж�

        /** ���� **/
        if(total_ecd_error > TRI_MINECD) // ˵����δ���ת��������Ҫ����ͣ���ڵ���ִ������
        {
            if((launcher.trigger_mode == SHOOT_SINGLE) && (total_time < TRI_MAXTIME))
            {/* δ�����涨���ʱ�䣬��ά�ֵ���ģʽ���� */
                launcher.trigger_mode = SHOOT_SINGLE;
            }
            else if((launcher.trigger_mode == SHOOT_SINGLE) && (total_time > TRI_MAXTIME))
            {/* ����ڵ���״̬�������ʱ��̫��,���ж���Ϊ��ת״̬ */
                launcher.trigger_mode = SHOOT_BLOCK;
            }
        }
         // ����˳��
        else if((total_ecd_error < TRI_MINECD) && (launcher.trigger_mode != SHOOT_CONTINUE))
        {
            launcher.trigger_mode = SHOOT_OVER;
        }

        /** ���� **/
        else if(launcher.trigger_mode == SHOOT_CONTINUE)
        {
            /* ���������״̬�µķ���ת��С��TRI_MAXSPEED����һ��ʱ�䣬���ж���Ϊ��ת״̬ */
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


        /***********************        �жϷ�ת״̬ʱ�Ƿ�������       *************************************/

#ifdef BLOCK_ANGLE
        //* ��ת�߼���Ҫ����ƽ���Ĳ����Ƿ���ʵ�ַ�ת������ܣ��Żῼ����ӣ���Ҫ���ԣ�������еĻ�Ҫ���ǵڶ�������������ת���̶�ʱ�� */
        if(total_ecd_error > TRI_MINECD) {
            if((launcher.trigger_mode == SHOOT_INVERSING) && (total_time < TRI_MAXTIME))
            {// ʵ�ʷ�תʱ��δ�������תʱ�䣬��ά�ַ�תģʽ����
                launcher.trigger_mode = SHOOT_INVERSING;
            }
            else if((launcher.trigger_mode == SHOOT_INVERSING) && (total_time > TRI_MAXTIME))
            {// �����תʱ�䳬���˹涨�����תʱ�䣬����Ϊ���ʧ��
                launcher.trigger_mode = SHOOT_FAIL;
            }
        }
            /* �ڹ涨ʱ���������״̬��Ϊ������� */
            // total_ecd_error�ո�˵�����ǵ�ǰ�����ʵʱ��ECD��Ŀ����ECD�Ĳ�ֵ����ô�ܲ�ֵС��TRI_MINECD
            // ��˵���Ѿ�ת����������Ҫ���ﵽ��ת���Ƕȣ������ж�Ϊ���һ�ε���
        else if((total_ecd_error < TRI_MINECD) && (launcher.trigger_mode != SHOOT_CONTINUE))
        {
            launcher.trigger_mode = SHOOT_OVER;
        }
#endif //!BLOCK_ANGLE

#ifdef BLOCK_SPEED
        /* TRI_MAXTIMEʱ���ڶ������ڷ�ת��״̬��TRI_MAXTIME��ģʽ��Ϊ���ģʽ�ٽ�����һ�γ����ж� */
        if((launcher.trigger_mode == SHOOT_BLOCK) && (total_time < TRI_MAXTIME))
        {// ʵ�ʷ�תʱ��δ�������תʱ�䣬��ά�ַ�תģʽ����
            launcher.trigger_mode = SHOOT_BLOCK;
        }
        else if((launcher.trigger_mode == SHOOT_BLOCK) && (total_time > TRI_MAXTIME))
        {// �����תʱ�䳬���˹涨�����תʱ�䣬����Ϊ���ʧ��
            launcher.trigger_mode = SHOOT_OVER;
        }
#endif //!BLOCK_SPEED
    }
}


/** ���㷢��������� **/
static void Launcher_Current_Calc(void) {

    /** ���̵�������(����pid λ�û��ڱ�ĵط�) **/
    launcher.trigger.target_current = (int16_t)pid_calc(&launcher.trigger.speed_p,
                                                        launcher.trigger.motor_measure.speed_rpm,
                                                        launcher.trigger.target_speed);

    /** Ħ���ֵ������� **/
    state_l = update_system(launcher.fire_l.target_speed, launcher.fire_l.motor_measure.speed_rpm, dt, state_l);
    launcher.fire_l.target_current = -smc_controller(state_l, params);

    state_r = update_system(launcher.fire_r.target_speed, launcher.fire_r.motor_measure.speed_rpm, dt, state_r);
    launcher.fire_r.target_current = -smc_controller(state_r, params);
}


/*********************************************************************************************************
*                                              API����ʵ��
*********************************************************************************************************/

/** ���������ʼ�� **/
void Launcher_Init(void) {

    /** Ħ����Ĭ��Ϊʧ��ģʽ **/
    launcher.fir_wheel_mode = launcher.fir_wheel_last_mode = Fire_OFF;

    /** ����Ĭ��Ϊʧ��ģʽ **/
    launcher.trigger_mode = launcher.trigger_last_mode = SHOOT_CLOSE;

    /* Ĭ�ϴ�ʱ���̵ĳ�ʼECD���ڵ�ǰECD, ��߷���׼�� */
    /* �Ժ��ٿ� �ƺ����ϵ�����й� */
    launcher.trigger.motor_measure.total_ecd = launcher.trigger.motor_measure.offset_ecd = launcher.trigger.motor_measure.ecd;

    /** ���̵��pid **/
    // λ�û�pid
    pid_init(&launcher.trigger.angle_p,
             TRIGGER_ANGLE_PID_MAX_OUT,
             TRIGGER_ANGLE_PID_MAX_IOUT,
             TRIGGER_ANGLE_PID_KP,
             TRIGGER_ANGLE_PID_KI,
             TRIGGER_ANGLE_PID_KD);

    // �ٶȻ�pid
    pid_init(&launcher.trigger.speed_p,
             TRIGGER_SPEED_PID_MAX_OUT,
             TRIGGER_SPEED_PID_MAX_IOUT,
             TRIGGER_SPEED_PID_KP,
             TRIGGER_SPEED_PID_KI,
             TRIGGER_SPEED_PID_KD);

    /* ������������������� */
    launcher.fire_l.target_current = 0;
    launcher.fire_r.target_current = 0;
    launcher.trigger.target_current = 0;

    /* �ʼ�ı���ֵ��Ϊ���ֵ����У׼ֵ */
    /* �Ժ��ٿ� */
    first_order_filter_init(&launcher.filter_fire,0.05f, 0.5f);
    first_order_filter_init(&launcher.filter_trigger,1,1);
}

/** �Ҿ�����������������Ƕ�Ħ����ģʽ�����жϣ�����Ϊ˳����Ħ����ģʽΪ����ʱ�����˲��̵�ģʽ�жϣ������˷��������Ա�����Ϊ����ģʽ������? **/
/** �����������н��� �����������н��� �����������н��� �����������н��� �����������н��� �����������н��� �����������н��� �����������н��� **/
void Launcher_Mode_Set() {
    /**
     * ����߲�����һ��ģʽ���������Ҵ˿�ģʽ������ʱ���뷢�����ģʽ�л���������Ϊ�أ��ر�Ϊ����
     * ������QΪ0ʱ�����Ϊ1��QΪ1ʱ�����Ϊ0
     * Q=1����Ħ���ֿ�����Fire_ON����Q=0����Ħ���ֹرգ�Fire_OFF��
     * ֻ����Ħ���ֿ�����ʱ��ſ��Խ��벦��ģʽ�ж�
     **/

    /**********************************    Ħ����ģʽ�ж�    ************************************/

    /** ���� Q�� ��ֵ **/
    if ((!switch_is_up(rc_last_sw_L)) && switch_is_up(rc_ctrl.rc.s[RC_s_L]))
    {
        // ��������
        if ((KeyBoard.Q.click_flag == 1) && (gimbal.mode != GIMBAL_DISABLE))
        {
            KeyBoard.Q.click_flag = 0;
        }
        else if ((KeyBoard.Q.click_flag == 0) && (gimbal.mode != GIMBAL_DISABLE))
        {
            KeyBoard.Q.click_flag = 1;
        }
    }

    /** ��������� Q�� ��ֵ�����жϣ�����Ħ�����Ƿ��� **/
    if ((KeyBoard.Q.click_flag == 1) && (gimbal.mode != GIMBAL_DISABLE))
    {
        launcher.fir_wheel_mode = Fire_ON;
    }
    else if ((KeyBoard.Q.click_flag == 0) && (gimbal.mode != GIMBAL_DISABLE))
    {
        launcher.fir_wheel_mode = Fire_OFF;
    }

    /**********************************      End      **************************************/



    /**********************************    ����ģʽ�ж�    ************************************/

    // ���ֻ��Ħ����ת�����޷������ģ�ͬ�����Ħ���ֲ�ת����Ҳ�޷�������
    // ����ֻ��Ħ���ֿ���ʱ�Ž��в���ģʽ�ж�, ����ģʽ(����������)���ɲ��̾����� **/
    if (launcher.fir_wheel_mode == Fire_ON)
    {
        Trigger_Mode_Set();
    }
    else if (launcher.fir_wheel_mode == Fire_OFF)
    {// ���Ħ���ֹرգ��򽫲���Ҳ�ر�
        launcher.trigger_mode = SHOOT_CLOSE;
    }

    /** ������һ�ε��󲦸�ֵ **/
    rc_last_sw_L = rc_ctrl.rc.s[RC_s_L];

    /**********************************      End      **************************************/



    /** Important Important Important Important Important Important Important Important Important
 *
 * Tips:    �� �ж��߼�ֻ����߲����й�
 *          �� KeyBoard.Q.click_flag����ң������ͼ�����ƣ�Ĭ����0, ����δ����Q����
 *          �� ֻ�е�����һʱ����߲��˲��������棬����һʱ���������桷ʱ���Ż���з������ģʽ�ж�
 *
 * Q1: Ħ��������α�������?
 * A1: ���㽫�󲦸˴�����λ��(��ָ�м������)����������ʱ���ɹ������жϣ������� else if ��֧��KeyBoard.Q.click_flag ����1�� Ħ���ֿ���
 *
 *
 * Q2: Ħ�����ڿ�������α��رգ�
 * A2����Ҫ�ر�Ħ���ֵ����������������ۣ�
 *
 *    �� ֻ����Ħ���֣�û�����̣�   ����մ�Ħ���ֵ�ʱ������󲦸˿϶�����������ģ����ʱ�� rc_last_sw_L = rc_ctrl.rc.s[RC_s_L] = up��
 *                          �Ͳ����ٽ���Q�����жϣ����Ħ������ᱣ�ֿ���ģʽ����ʱ�㽫�󲦸˲����м䣬rc_last_sw_L = rc_ctrl.rc.s[RC_s_L] = mid,
 *                          �ٽ��󲦸˲������棬�ض�����һ��ʱ��Ϊ��rc_last_sw_L = mid �� rc_ctrl.rc.s[RC_s_L] = up����
 *                          ��ʱ���ٶȽ���Q�����жϣ���Ϊ�ϴδ�Ħ����ʱ KeyBoard.Q.click_flag ����1�ˣ�������ε��жϻᱻ��0��Ħ���־ͱ��ر���
 *
 *    �� Ħ���ֺͲ��̶����ˣ����굯��ر�Ħ���֣�      �򵯵�ʱ������󲦸�Ӧ������������ģ���ʱ���㽫�󲦸˴������棬
 *                                          �ض�����һ��ʱ��Ϊ��rc_last_sw_L = mid �� rc_ctrl.rc.s[RC_s_L] = up����
 *                                          ��ʱ���ٶȽ���Q�����жϣ���Ϊ�ϴδ�Ħ����ʱ KeyBoard.Q.click_flag ����1�ˣ�������ε��жϻᱻ��0��Ħ���־ͱ��ر���
 *
 *
 * **/



}


/**
 * ����������߼�ʵ��
 */
void Launcher_Control(void) {

    /** ��̨ʧ��ʱ **/
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
        {/** �������ʧ��ʱ �ر�Ħ���ֺͲ��� **/
            // Q: Ϊʲô���ڼ�������ٽ��з������ʧ���жϣ� ֱ�ӰѼ����Ľ����0Ȼ�󷢳�ȥ
            launcher.fire_l.target_speed = 0;
            launcher.fire_r.target_speed = 0;
            launcher.trigger.target_speed = 0;

            launcher.trigger.target_current = 0;

            trigger_target_total_ecd = launcher.trigger.motor_measure.total_ecd;
        }

        /** ��������������Ƽ��� **/
        Launcher_Current_Calc();

        /* ��ת��� */
        Trigger_Finish_Judge();
    }
}

/** �������ʧ�� **/
void Launcher_Disable(void) {

    /** �ر�Ħ���� **/
    launcher.fir_wheel_mode = Fire_OFF;

    /** ��ʵ������ʡ�ԣ���Ϊ��Ħ���ֹرպ���Launcher_Mode_Set�������ж���Ҳ��رղ��̣�����Ϊ�������Ի��Ǳ����� **/
    launcher.trigger_mode = SHOOT_CLOSE;

    /** ����Ħ���ֺͲ��̵������� **/
    launcher.fire_r.target_current = 0;
    launcher.fire_l.target_current = 0;
    launcher.trigger.target_current = 0;

    /** ʧ��ʱ�������ܱ�����ֵ�ܵ��ڷ������ܱ�����ֵ�������´η��� **/
    trigger_target_total_ecd = (int32_t)launcher.trigger.motor_measure.total_ecd;
}