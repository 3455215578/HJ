//
// Created by xhuanc on 2021/10/23.
//

#ifndef DEMO1_REFEREE_H
#define DEMO1_REFEREE_H

#include "stdbool.h"
#include "CRC8_CRC16.h"
#include "bsp_usart.h"

#define REFREE_HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))
#define REFEREE_BUFFER_SIZE     200

#define LEN_HEADER 5

//ͨ��Э���ʽ λ����
typedef enum
{
    FRAME_HEADER= 0,
    CMD_ID               = 5,
    DATA                 = 7,
}RefereeFrameOffset;

// frame_header ��ʽ
typedef enum
{
    SOF          = 0,//��ʼλ
    DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
    SEQ          = 3,//�����
    CRC8         = 4 //CRC8
}FrameHeaderOffset;


//����ϵͳ����ID
/***************������ID********************
    / ��������ȫ�������
    ID: 0x0001  Byte:  11   ����״̬����,�̶���1HzƵ�ʷ���
    ID: 0x0002  Byte:   1   �����������,������������
    ID: 0x0003  Byte:  32   ����������Ѫ������,�̶���3HzƵ�ʷ���
    / ������������ȫ�������
    ID: 0x0101  Byte:   4   �����¼�����,�̶���1HzƵ�ʷ���
    / �����������з���ȫ�������
    ID: 0X0104  Byte:   3   ���о�������,�����з�/�и�ʱ��������,����ʱ����1HzƵ�ʷ���
    / ������������ȫ�������
    ID: 0x0105  Byte:   3   ���ڷ����������,�̶���1HzƵ�ʷ���
    / ����ģ�����Ӧ������
    ID: 0X0201  Byte:  13    ������������ϵ����,�̶���10HzƵ�ʷ���
    ID: 0X0202  Byte:  16    ʵʱ���̻��������������������,�̶���10HzƵ�ʷ���
    ID: 0x0203  Byte:  16    ������λ������,�̶���1HzƵ�ʷ���
    / ����������Ӧ������
    ID: 0x0204  Byte:   7    ����������͵�����������,�̶���3HzƵ�ʷ���
    / ����ģ�����Ӧ������
    ID: 0x0206  Byte:   1    �˺�״̬����,�˺���������
    ID: 0x0207  Byte:   7    ʵʱ�������,���跢�����
    / ������������Ӣ�ۡ��������ڱ������л�����
    ID: 0x0208  Byte:   6    ��������,�̶���10HzƵ�ʷ���
    / ������������װ��RFIDģ��Ļ�����
    ID: 0x0209  Byte:   4    ������RFIDģ��״̬���̶���3HzƵ�ʷ���
    / ������������������
    ID: 0x020A  Byte:   6    ����ѡ�ֶ�ָ�����ݣ��̶���3HzƵ�ʷ���
    ID: 0x020B  Byte:  40    ���������λ�����ݣ��̶���1HzƵ�ʷ���
    ID: 0x020C  Byte:   1    �״��ǽ������ݣ��̶���1HzƵ�ʷ���
    ID: 0x020D  Byte:   6    �ڱ�����������Ϣͬ�����̶���1HzƵ�ʷ���
    ID: 0x020E  Byte:   1    �״�����������Ϣͬ�����̶���1HzƵ�ʷ���

    ID: 0x0301  Byte: 127    �����˽������ݣ����ͷ��������ͣ�Ƶ������Ϊ30Hz
    / �Զ����������ѡ�ֶ�ͼ�����ӵĻ�����
    ID: 0x0302  Byte:  30    �Զ��������������˽������ݣ����ͷ��������ͣ�Ƶ������Ϊ30Hz
    / ѡ�ֶ˵���������������ͷ�ѡ��ļ���������
    ID: 0x0303  Byte:  15    ѡ�ֶ�С��ͼ�������ݣ�ѡ�ֶ˴�������
    / ѡ�ֶˡ�ѡ�ֶ�ͼ�����ӵĻ�����
    ID: 0x0304  Byte:  12    ����ң�����ݣ��̶�30HzƵ�ʷ���
    / �״������������������ѡ�ֶ�
    ID: 0x0305  Byte:  24    ѡ�ֶ�С��ͼ�����״����ݣ�Ƶ������Ϊ5Hz
    / �Զ����������ѡ�ֶ�
    ID: 0x0306  Byte:   8    �Զ����������ѡ�ֶ˽������ݣ����ͷ��������ͣ�Ƶ������Ϊ30Hz
    / �ڱ�/���Զ����ƻ����ˡ���Ӧ������ѡ�ֶ�
    ID: 0x0307  Byte: 103    ѡ�ֶ�С��ͼ�����ڱ����ݣ�Ƶ������Ϊ1Hz
    / ���������ˡ�����ѡ�ֶ�
    ID: 0x0308  Byte:  34    ѡ�ֶ�С��ͼ���ջ��������ݣ�Ƶ������Ϊ3Hz
    / ���������ˡ���Ӧ������ѡ�ֶ����ӵ��Զ��������
    ID: 0x0309  Byte:  30    �Զ�����������ջ��������ݣ�Ƶ������Ϊ10Hz
*/


typedef enum
{
    Referee_ID_game_state                   = 0x0001,//����״̬����
    Referee_ID_game_result                  = 0x0002,//�����������
    Referee_ID_game_robot_HP                = 0x0003,//������Ѫ������

    Referee_ID_event_data                   = 0x0101,//�����¼�����
    Referee_ID_supply_warm                  = 0x0104,//����ϵͳ��������
    Referee_ID_dart_info                    = 0x0105,//���ڷ����������

    Referee_ID_game_robot_state             = 0x0201,//������������ϵ����
    Referee_ID_power_heat_data              = 0x0202,//ʵʱ���̻��������������������
    Referee_ID_game_robot_pos               = 0x0203,//������λ������
    Referee_ID_buff_musk                    = 0x0204,//����������͵�����������
    Referee_ID_robot_hurt                   = 0x0206,//�˺�״̬���ݣ��˺���������
    Referee_ID_shoot_data                   = 0x0207,//ʵʱ������ݣ����跢�����
    Referee_ID_bullet_remaining             = 0x0208,//��������
    Referee_ID_rfid_status                  = 0x0209,//������RFID״̬��3Hz
    Referee_ID_dart_client_directive        = 0x020A,//����ѡ�ֶ�ָ������, 3Hz
    Referee_ID_dart_all_robot_position      = 0x020B,//���������λ������
    Referee_ID_radar_mark                   = 0x020C,//�״��ǽ�������
    Referee_ID_entry_info                   = 0x020D,//�ڱ�����������Ϣͬ�� 1Hz
    Referee_ID_radar_info                   = 0x020E,//�״�����������Ϣͬ�� 1Hz

    Referee_ID_robot_interactive_header_data        = 0x0301,//�����˽������ݣ��������ͷ������������� 30Hz
    Referee_ID_controller_interactive_header_data   = 0x0302,//�Զ��������������˽������ݣ����ͷ��������ͣ�Ƶ������30Hz
    Referee_ID_map_command                          = 0x0303,//ѡ�ֶ�С��ͼ�������ݣ�ѡ�ֶ˴�������
    Referee_ID_keyboard_information                 = 0x0304,//����ң�����ݣ��̶�30Hz,ͼ����·
    Referee_ID_robot_map_robot_data                 = 0x0305,//ѡ�ֶ�С��ͼ�����״�����,����5Hz
    Referee_ID_robot_custom_client                  = 0x0306,//�Զ����������ѡ�ֶ˽������ݣ����ͷ��������ͣ�Ƶ������30Hz
    Referee_ID_robot_entry_info_receive             = 0x0307,//ѡ�ֶ�С��ͼ�����ڱ�����,����1Hz
    Referee_ID_robot_custom_info_receive            = 0x0308,//ͨ��������·���ջ����˵�����,���ض�λ����ʾ������3Hz
    Referee_ID_robot_custom                         = 0x0309,//�Զ�����������ջ��������ݣ�Ƶ������Ϊ10Hz
}referee_cmd_id_t;

//����ϵͳ����������ݳ���
typedef enum
{
    /* Stdͨ��Э���ʽ */
    Referee_LEN_FRAME_HEAD                      = 5,   // ֡ͷ���� frame_header
    Referee_LEN_CMD_ID                          = 2,   // �����볤�� cmd_id
    Referee_LEN_FRAME_TAIL                      = 2,   // ֡βCRC16 frame_tail
    /* Ext */

    Referee_LEN_game_state                      =  11, //0x0001
    Referee_LEN_game_result                     =  1,  //0x0002
    Referee_LEN_game_robot_HP                   =  32, //0x0003  ����������Ѫ������

    Referee_LEN_event_data                      =  4,  //0x0101  �����¼�����
    Referee_LEN_supply_warm                     =  3,   //0x0104 ����ϵͳ����
    Referee_LEN_dart_info                       =  3,   //0x0105 ���ڷ���ڵ���ʱ

    Referee_LEN_game_robot_state                = 13,  //0x0201 ������״̬����
    Referee_LEN_power_heat_data                 = 16,  //0x0202 ʵʱ������������
    Referee_LEN_game_robot_pos                  = 16,  //0x0203 ������λ������
    Referee_LEN_buff_musk                       =  7,  //0x0204 ��������������
    Referee_LEN_robot_hurt                      =  1,  //0x0206 �˺�״̬����
    Referee_LEN_shoot_data                      =  7,  //0x0207 ʵʱ�������
    Referee_LEN_bullet_remaining                =  6,    //0x0208ʣ�෢����
    Referee_LEN_rfid_status                     =  4,    //0x0209
    Referee_LEN_dart_client_directive           =  6,    //0x020A
    Referee_LEN_dart_all_robot_position         = 40,    //0x020B
    Referee_LEN_radar_mark                      =  1,    //0x020C
    Referee_LEN_entry_info                      =  6,    //0x020D
    Referee_LEN_radar_info                      =  1,    //0x020E

    Referee_LEN_robot_interactive_header_data   =127,    //0x0301
    Referee_LEN_controller_interactive_header_data=30,   //0x0302
    Referee_LEN_map_command                     = 15,    //0x0303
    Referee_LEN_keyboard_information            = 12,    //0x0304
    Referee_LEN_robot_map_robot_data            = 24,    //0x0305
    Referee_LEN_robot_custom_client             =  8,    //0x0306
    Referee_LEN_robot_entry_info_receive        =103,    //0x0307
    Referee_LEN_robot_custom_info_receive       = 34,    //0x0308
    Referee_LEN_robot_custom                    = 30,    //0x0309
}RefereeDataLength;


typedef enum{
    Referee_hero_red       = 1,
    Referee_engineer_red   = 2,
    Referee_infantry3_red  = 3,
    Referee_infantry4_red  = 4,
    Referee_infantry5_red  = 5,
    Referee_plane_red      = 6,

    Referee_hero_blue      = 101,
    Referee_engineer_blue  = 102,
    Referee_infantry3_blue = 103,
    Referee_infantry4_blue = 104,
    Referee_infantry5_blue = 105,
    Referee_plane_blue     = 106,
}Referee_robot_ID;


typedef struct {

    bool static_update;//��̬Ԫ���Ƿ�Ҫˢ��
    uint8_t gimbal_mode;//��̨ģʽ ���˿� �����Կأ��Կ��Ǵ����������)
    uint8_t chassis_mode;//
    uint8_t block_warning;//�µ�����
    uint8_t shoot_heat_limit;//��ǰ��������
    fp32 super_cap_value;//��������ֵ
    uint8_t fire_mode;
    float pitch_value;
    float relative_yaw_value;
}ui_robot_status_t;//������״̬�ṹ�� ��������Ƿ��,�����Ƿ�򿪣������Ƿ�򿪵���Ϣ

//����ϵͳ֡ͷ�ṹ��
typedef  struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
}__packed frame_header_struct_t;


/* ID: 0x0001  Byte:  11    ����״̬���� */
typedef  struct
{
    uint8_t game_type : 4;      // ��������
    uint8_t game_progress : 4;  // ��ǰ�����׶�
    uint16_t stage_remain_time; // ��ǰ�׶�ʣ��ʱ�䣬��λ����
    uint64_t SyncTimeStamp;     // UNIXʱ�䣬����������ȷ���ӵ�����ϵͳ��NTP����������Ч
} __packed ext_game_state_t;


/* ID: 0x0002  Byte:  1    ����������� */
typedef  struct
{
    uint8_t winner;         // 0��ƽ�� 1���췽ʤ�� 2������ʤ��
}__packed  ext_game_result_t;


/* ID: 0x0003  Byte:  32    ����������Ѫ������ */
typedef  struct
{
    uint16_t red_1_robot_HP;    // ��1Ӣ�ۻ�����Ѫ�������û�����δ�ϳ����߱����£���Ѫ��Ϊ0
    uint16_t red_2_robot_HP;    // ��2���̻�����Ѫ��
    uint16_t red_3_robot_HP;    // ��3����������Ѫ��
    uint16_t red_4_robot_HP;    // ��4����������Ѫ��
    uint16_t reserved1;         // ����λ
    uint16_t red_7_robot_HP;    // ��7�ڱ�������Ѫ��
    uint16_t red_outpost_HP;    // �췽ǰ��վѪ��
    uint16_t red_base_HP;       // �췽����Ѫ��

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t reserved2;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} __packed ext_game_robot_HP_t;


/** ID: 0x0101  Byte:  4    �����¼�����
 * 0��δռ��/δ����
 * 1����ռ��/�Ѽ���
 * bit 0-2��
 * bit 0��������һ������ص��Ĳ�����ռ��״̬��1Ϊ��ռ��
 * bit 1��������һ����ص��Ĳ�����ռ��״̬��1Ϊ��ռ��
 * bit 2��������������ռ��״̬��1Ϊ��ռ�죨�� RMUL ���ã�
 * bit 3-5��������������״̬
 * bit 3������С�������صļ���״̬��1Ϊ�Ѽ���
 * bit 4���������������صļ���״̬��1Ϊ�Ѽ���
 * bit 5-6����������ߵص�ռ��״̬��1Ϊ������ռ�죬2Ϊ���Է�ռ��
 * bit 7-8���������θߵص�ռ��״̬��1Ϊ��ռ��
 * bit 9-17���Է��������һ�λ��м���ǰ��վ����ص�ʱ�䣨0-420������Ĭ��Ϊ0��
 * bit 18-20���Է��������һ�λ��м���ǰ��վ����صľ���Ŀ�꣬����Ĭ��Ϊ0��1Ϊ����ǰ��վ��2Ϊ���л��ع̶�Ŀ�꣬3Ϊ���л�������̶�Ŀ�꣬4Ϊ���л�������ƶ�Ŀ��
 * bit 21-22������������ռ��״̬��0Ϊδ��ռ�죬1Ϊ������ռ�죬2Ϊ���Է�ռ�죬3Ϊ��˫��ռ�졣����RMUL���ã�
 * bit 23-31������λ
 */
typedef  struct
{
    uint32_t event_type;
} __packed ext_event_data_t;


/* ID: 0x0104  Byte: 2->3   ����ϵͳ������Ϣ */
typedef  struct
{
    uint8_t level;              // �������һ���ܵ��з��ĵȼ�
    uint8_t offending_robot_id; // �������һ���ܵ��з���Υ�������ID�������1������IDΪ1����1������IDΪ101��
    uint8_t count;              // �������һ���ܵ��з���Υ������˶�Ӧ�з��ȼ���Υ�������������Ĭ��Ϊ0����
} __packed  ext_referee_warning_t;

/** ID: 0x0105  Byte:1->3  ���ڷ���ڵ���ʱ
 * bit 0-2��
 * ���һ�μ������ڻ��е�Ŀ�꣬����Ĭ��Ϊ0��1Ϊ����ǰ��վ��2Ϊ���л��ع̶�Ŀ�꣬3Ϊ���л�������̶�Ŀ�꣬4Ϊ���л�������ƶ�Ŀ��
 * bit 3-5��
 * �Է���������е�Ŀ���ۼƱ����мƴ���������Ĭ��Ϊ0������Ϊ4
 * bit 6-7��
 * ���ڴ�ʱѡ���Ļ���Ŀ�꣬����Ĭ�ϻ�δѡ��/ѡ��ǰ��վʱΪ0��ѡ�л��ع̶�Ŀ��Ϊ1��ѡ�л�������̶�Ŀ��Ϊ2��ѡ�л�������ƶ�Ŀ��Ϊ3
 * bit 8-15������
 */
typedef  struct
{
    uint8_t dart_remaining_time;    // �������ڷ���ʣ��ʱ�䣬��λ����
    uint16_t dart_info;             //����
} __packed ext_dart_remaining_time_t;

/* ID: 0X0201  Byte:     ������״̬���� */
typedef  struct
{
    uint8_t robot_id;       // ��������ID
    uint8_t robot_level;    // �����˵ȼ�
    uint16_t current_HP;    // �����˵�ǰѪ��
    uint16_t maximum_HP;    // ������Ѫ������

    uint16_t shooter_barrel_cooling_value;  // �������������ÿ����ȴֵ
    uint16_t shooter_barrel_heat_limit;     // �����������������
    uint16_t chassis_power_limit;           // �����˵��̹�������
    /* ��Դ����ģ��������� */
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} __packed ext_robot_status_t;

/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef  struct
{
    uint16_t reserved1;
    uint16_t reserved2;
    float    reserved3;
    uint16_t buffer_energy;                 // ������������λ��J��
    uint16_t shooter_17mm_1_barrel_heat;    // ��1�� 17mm ���������ǹ������
    uint16_t shooter_17mm_2_barrel_heat;    // ��2�� 17mm ���������ǹ������
    uint16_t shooter_42mm_barrel_heat;      // 42mm ���������ǹ������
} __packed ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    ������λ������ */
typedef  struct
{
    float x;    // ��������λ��x���꣬��λ��m
    float y;    // ��������λ��y���꣬��λ��m
    float angle;// �������˲���ģ��ĳ��򣬵�λ���ȡ�����Ϊ0��
}__packed ext_robot_pos_t;


/* ID: 0x0204  Byte:  1    �������������� */
typedef struct
{
    uint8_t recovery_buff;          // �����˻�Ѫ���棨�ٷֱȣ�ֵΪ10��ʾÿ��ָ�Ѫ�����޵�10%��
    uint8_t cooling_buff;           // ���������������ȴ���ʣ�ֱ��ֵ��ֵΪ5��ʾ5����ȴ��
    uint8_t defence_buff;           // �����˷������棨�ٷֱȣ�ֵΪ50��ʾ50%�������棩
    uint8_t vulnerability_buff;     // �����˸��������棨�ٷֱȣ�ֵΪ30��ʾ-30%�������棩
    uint16_t attack_buff;           // �����˹������棨�ٷֱȣ�ֵΪ50��ʾ50%�������棩
    /**bit 0-4��������ʣ������ֵ��������16���Ʊ�ʶ������ʣ������ֵ������
     * ���ڻ�����ʣ������С��50%ʱ����������Ĭ�Ϸ���0x32�� 00110 0 10       000111111
     * bit 0����ʣ��������50%ʱΪ1���������Ϊ0
     * bit 1����ʣ��������30%ʱΪ1���������Ϊ0
     * bit 2����ʣ��������15%ʱΪ1���������Ϊ0
     * bit 3����ʣ��������5%ʱΪ1���������Ϊ0Bit4����ʣ��������1%ʱΪ1���������Ϊ0g
     */
    uint8_t remaining_energy;       // ������ʣ����������ֵ
}__packed  ext_buff_t;


/** ID: 0x0206  Byte:  1    �˺�״̬����
 * bit 0-3������Ѫԭ��Ϊװ��ģ�鱻���蹥������ײ�������߻����ģ������ʱ����4 bit��ɵ���ֵΪװ��ģ������ģ���ID��ţ�������ԭ���¿�Ѫʱ������ֵΪ0
 * bit 4-7��Ѫ���仯����
 * 0��װ��ģ�鱻���蹥�����¿�Ѫ
 * 1������ϵͳ��Ҫģ�����ߵ��¿�Ѫ
 * 2��������ٶȳ��޵��¿�Ѫ
 * 3������������޵��¿�Ѫ
 * 4�����̹��ʳ��޵��¿�Ѫ
 * 5��װ��ģ���ܵ�ײ�����¿�Ѫ
 */
typedef  struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} __packed ext_robot_hurt_t;


/* ID: 0x0207  Byte:  7    ʵʱ������� */
typedef  struct
{
    uint8_t bullet_type;    // 1��17mm���� 2��42mm����
    uint8_t shooter_id;     // 1����1��17mm�������  2����2��17mm������� 3��42mm�������
    uint8_t bullet_freq;    // �������٣���λ��Hz��
    float bullet_speed;     // ������ٶȣ���λ��m/s��
}__packed  ext_shoot_data_t;


/* ID: 0x0208  Byte:  6    �ӵ�ʣ������ */
typedef  struct
{
    uint16_t projectile_allowance_17mm; // 17mm������������
    uint16_t projectile_allowance_42mm; // 42mm������������
    uint16_t remaining_gold_coin;       // ʣ��������
} __packed ext_bullet_remaining_t;

/** ID: 0x0209  Byte:  4 	������RFID״̬
 * bitλֵΪ1/0�ĺ��壺�Ƿ��Ѽ�⵽�������RFID��
 * bit 0���������������
 * bit 1����������ߵ������
 * bit 2���Է�����ߵ������
 * bit 3���������θߵ������
 * bit 4���Է����θߵ������
 * bit 5���������ο�Խ����㣨���£�����������һ�����ǰ��
 * bit 6���������ο�Խ����㣨���£�����������һ����º�
 * bit 7���Է����ο�Խ����㣨���£��������Է�һ�����ǰ��
 * bit 8���Է����ο�Խ����㣨���£��������Է�һ����º�
 * bit 9���������ο�Խ����㣨����ߵ��·���
 * bit 10���������ο�Խ����㣨����ߵ��Ϸ���
 * bit 11���Է����ο�Խ����㣨����ߵ��·���
 * bit 12���Է����ο�Խ����㣨����ߵ��Ϸ���
 * bit 13���������ο�Խ����㣨��·�·���
 */
typedef  struct
{
    uint32_t rfid_status;
} __packed ext_rfid_status_t;


/* ID:  0x020A  Byte:6   	 */
typedef  struct
{
    uint8_t dart_launch_opening_status;     // ���ڷ���״̬ 0 �Ѿ�����, 1 �ر� 2 ���ڿ������߹ر���,
    uint8_t reserved;                       // ����
    uint16_t target_change_time;            // �л�����Ŀ��ʱ�ı���ʣ��ʱ�䣬��λs,Ĭ��Ϊ0
    uint16_t latest_launch_cmd_time;        // ���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ��,��λs,��ʼֵΪ0
}__packed ext_dart_client_cmd_t;

//V1.6.1���� 24�����Զ�����
/* ID:   0x020B  Byte:40   	 */
typedef  struct
{
    float hero_x;
    float hero_y;

    float engineer_x;
    float engineer_y;

    float standard_3_x;
    float standard_3_y;

    float standard_4_x;
    float standard_4_y;

    float standard_5_x;
    float standard_5_y;
}__packed ext_ground_robot_position_t;

//V1.6.1���� 24�����״��޸�
/* ID:   0x020C  Byte:6  �����˱��״��ǽ��� 0-120	 */
typedef  struct
{
    uint8_t mark_progress;   //bit0-4 �ֱ��Ӧ�Է�Ӣ�ۡ����̡�����3 4�š��ڱ����������
}__packed ext_radar_mark_data_t;

//V1.6.1���� 24�����ڱ��޸�
/* ID:   0x020D  Byte:4  �ڱ��һ���������Ѫ���� */
typedef  struct
{
    uint32_t sentry_info;
    uint32_t sentry_info_2;
} __packed ext_sentry_info_t;

//V1.6.1���� 24�����״��޸�
/* ID:   0x020E  Byte:1  �״ﴥ��˫����Ϣ	 */
typedef struct
{
    uint8_t radar_info;
} __packed  ext_radar_info_t;

/*
	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��

	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	11��Ӣ��(��)��
	12������(��)��
	13/14/15������(��)��
	16������(��)��
	17���ڱ�(��)��
	�ͻ��� ID��
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)��
	0x0111��Ӣ�۲����ֿͻ���(��)��
	0x0112�����̲����ֿͻ���(��)��
	0x0113/0x0114/0x0115�������ֿͻ��˲���(��)��
	0x0116�����в����ֿͻ���(��)��
*/


/* �������ݽ�����Ϣ��0x0301  */
typedef  struct
{
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
}__packed ext_student_interactive_header_data_t;

typedef struct{
    uint16_t teammate_hero;
    uint16_t teammate_engineer;
    uint16_t teammate_infantry3;
    uint16_t teammate_infantry4;
    uint16_t teammate_infantry5;
    uint16_t teammate_plane;
    uint16_t teammate_sentry;

    uint16_t client_hero;
    uint16_t client_engineer;
    uint16_t client_infantry3;
    uint16_t client_infantry4;
    uint16_t client_infantry5;
    uint16_t client_plane;
}ext_interact_id_t;

/* ѡ�ֶ�С��ͼ�������ݣ�0x0303  */
typedef  struct
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
}__packed ext_map_command_t;

/* ����ң�����ݣ�0x0304  */
typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
}__packed ext_remote_control_t;

/* ѡ�ֶ�С��ͼ�����״����ݣ�0x0305  */
typedef struct
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} ext_map_robot_data_t;

/* �Զ����������ѡ�ֶ˽������ݣ�0x0306  */
typedef struct
{
    uint16_t key_value;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
}__packed ext_custom_client_data_t;

/* ѡ�ֶ�С��ͼ�����ڱ����ݣ�0x0307  */
typedef  struct
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
}__packed ext_map_data_t;

/* ѡ�ֶ�С��ͼ���ջ��������ݣ�0x0308  */
typedef  struct
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} __packed ext_custom_info_t;

/*
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz

	�ֽ�ƫ���� 	��С 	˵�� 			��ע
	0 			2 		���ݵ����� ID 	0x0200~0x02FF
										���������� ID ��ѡȡ������ ID �����ɲ������Զ���

	2 			2 		�����ߵ� ID 	��ҪУ�鷢���ߵ� ID ��ȷ�ԣ�

	4 			2 		�����ߵ� ID 	��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID

	6 			n 		���ݶ� 			n ��ҪС�� 113

*/
typedef  struct
{
    uint8_t data[113]; //���ݶ�,n��ҪС��113
} __packed robot_interactive_data_t;

typedef struct judge_info_struct {
    frame_header_struct_t 							FrameHeader;				// ֡ͷ��Ϣ

    ext_game_state_t 							    GameState;				    // 0x0001           ����״̬����
    ext_game_result_t 							    GameResult;				    // 0x0002         �����������
    ext_game_robot_HP_t 						    GameRobotHP;			    // 0x0003         ������Ѫ������

    ext_event_data_t								EventData;					// 0x0101         �����¼�����
    ext_referee_warning_t						    RefereeWarning;		        // 0x0104         ���о�����Ϣ
    ext_dart_remaining_time_t				        DartRemainingTime;          // 0x0105         ���ڷ���ڵ���ʱ


    ext_robot_status_t					            GameRobotStat;	            // 0x0201         ����������״̬
    ext_power_heat_data_t						    PowerHeatData;		        // 0x0202         ʵʱ������������
    ext_robot_pos_t						            GameRobotPos;			    // 0x0203         ������λ��
    ext_buff_t									    Buff;						// 0x0204     ����������
    ext_robot_hurt_t								RobotHurt;					//0x0206         �˺�״̬
    ext_shoot_data_t								ShootData;					//0x0207         ʵʱ�����Ϣ(��Ƶ  ����  �ӵ���Ϣ)
    ext_bullet_remaining_t					        BulletRemaining;		    //0x0208	        �ӵ�ʣ�෢����
    ext_rfid_status_t								RfidStatus;				    //0x0209	        RFID��Ϣ
    ext_dart_client_cmd_t                           DartClient;                 //0x020A         ���ڿͻ���
    ext_ground_robot_position_t                     RobotPosition;              //0x020B
    ext_radar_mark_data_t                           RadarMark;                  //0x020C
    ext_sentry_info_t                               SentryInfo;                 //0x020D
    ext_radar_info_t                                RadarInfo;                  //0x020E

    ext_student_interactive_header_data_t           StudentInteractive;         //0x0301
    ext_map_command_t                               MapCommand;                 //0x0303
    ext_remote_control_t                            keyboard;                   //0x0304 ����
    ext_map_robot_data_t                            EnemyPosition;              //0x0305 �з�������λ��
    ext_custom_client_data_t                        Custom;                     //0x0306 �Զ��������
    ext_map_data_t                                  SentryMapData;              //0x0307 �ڱ���������
    ext_custom_info_t                               SendData;                   //0x0308 �������Զ��巢����Ϣ

    ext_interact_id_t								ids;			            //�뱾�������Ļ�����id
    uint16_t                                        SelfClient;                 //�����ͻ���

} Referee_info_t;


/*
	ѧ�������˼�ͨ�� cmd_id 0x0301������ data_ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����������кϼƴ������� 5000 Byte�� �����з���Ƶ�ʷֱ𲻳���30Hz��
 * +------+------+-------------+------------------------------------+
 * | byte | size |    breif    |            note                    |
 * |offset|      |             |                                    |
 * +------+------+-------------+------------------------------------+
 * |  0   |  2   | 	 data_ID   | 0x0200~0x02FF,��������Щ ID ��ѡȡ    |
 * |      |      |             | ����ID�����ɲ������Զ���               |
 * +------|------|-------------|------------------------------------|
 * |  2   |  2   | 	sender_ID  | ��ҪУ�鷢���ߵ� ID ��ȷ��				|
 * +------|------|-------------|------------------------------------|
 * |  4   |  2   | receiver_ID | ��ҪУ������ߵ� ID ��ȷ��				|
 * |      |      |             | ���粻�ܷ��͵��жԻ����˵�ID			|
 * +------|------|-------------|------------------------------------|
 * |  6   |  n   |    Data     | n ��ҪС�� 113 										|
 * +------+------+-------------+------------------------------------+
*/


//����ͼ��ID
typedef enum
{
    //0x200-0x02ff 	�����Զ������� ��ʽ  INTERACT_ID_XXXX
    UI_INTERACT_ID_delete_graphic 			= 0x0100,	/*�ͻ���ɾ��ͼ��*/
    UI_INTERACT_ID_draw_one_graphic 		= 0x0101,	/*�ͻ��˻���һ��ͼ��*/
    UI_INTERACT_ID_draw_two_graphic 		= 0x0102,	/*�ͻ��˻���2��ͼ��*/
    UI_INTERACT_ID_draw_five_graphic 	    = 0x0103,	/*�ͻ��˻���5��ͼ��*/
    UI_INTERACT_ID_draw_seven_graphic 	    = 0x0104,	/*�ͻ��˻���7��ͼ��*/
    UI_INTERACT_ID_draw_char_graphic 	    = 0x0110,	/*�ͻ��˻����ַ�ͼ��*/
    UI_INTERACT_ID_bigbome_num				= 0x02ff
}Interact_ID;

typedef enum
{
    UI_LEN_INTERACT_delete_graphic     = 8,  //ɾ��ͼ�� 2(��������ID)+2(������ID)+2��������ID��+2���������ݣ�
    UI_LEN_INTERACT_draw_one_graphic   = 21, // ����2+2+2+15
    UI_LEN_INTERACT_draw_two_graphic   = 36, //6+15*2
    UI_LEN_INTERACT_draw_five_graphic  = 81, //6+15*5
    UI_LEN_INTERACT_draw_seven_graphic = 111,//6+15*7
    UI_LEN_INTERACT_draw_char_graphic  = 51, //6+15+30���ַ������ݣ�
}Interact_ID_len;

//ͼ������
typedef  struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;          //ֱ��  ����  ��Բ  ��Բ  Բ��  ����  ����  �ַ�
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;           //��    ��    ��    ��    �Ƕ�  ��С  ��С  ��С
    uint32_t end_angle:9;             //��    ��    ��    ��          λ��  ��    ����
    uint32_t width:10;
    uint32_t start_x:11;              //���  ���  Բ��  Բ��  Բ��  ���  ���  ���
    uint32_t start_y:11;              //
    union {
        struct {
            uint32_t radius:10;      //��    ��    �뾶  ��    ��    ��    ��    ��
            uint32_t end_x:11;       //�յ�  �Զ�  ��    ����  ����  ��    ��    ��
            uint32_t end_y:11;       //                              ��    ��    ��                  ��    ��    ��
        };
        int32_t number;
    };
} __packed ui_graphic_data_struct_t;//ui��ͷ �����κ͸�����Ҳ��Ϊͼ�� ������graphic����

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;                                //������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData;            //���ݶ�
    uint16_t	FrameTail;                          //֡β
}__packed ext_graphic_one_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;                                //������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData[2];		    //����0x0201��
    uint16_t	FrameTail;							//֡β
}__packed ext_graphic_two_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;                                //������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData[5];		    //���ݶ�
    uint16_t	FrameTail;							//֡β
}__packed ext_graphic_five_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;								//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData[7];		    //���ݶ�
    uint16_t	FrameTail;							//֡β
}__packed ext_graphic_seven_data_t;

//���ַ���
//�ַ�������ui_graphic_data_struct_t����ṹ�������
// ����30���ֽڵ��ַ����ݴ洢�ַ���
typedef  struct
{
    ui_graphic_data_struct_t graphic_data_struct;
    uint8_t data[30];
}__packed ui_string_t;

//�̶����ݶγ������ݰ�
typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;								//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_string_t clientData;                         //���ݶ�
    uint16_t	FrameTail;							//֡β
}__packed ext_string_data_t;

//****************************��ͼ�����ݶ�����****************************/
/* data_ID: 0X0100  Byte:  2	    �ͻ���ɾ��ͼ��*/
typedef  struct
{
    uint8_t operate_type;
    uint8_t layer;//ͼ������0~9
}__packed ext_client_custom_graphic_delete_t;

/* ͼ��ɾ��������ö�� */
typedef enum
{
    UI_NONE_delete    = 0,
    UI_GRAPHIC_delete = 1,
    UI_ALL_delete     = 2
}Delete_Graphic_Operate;//ext_client_custom_graphic_delete_t��uint8_t operate_type

//bit 0-2
typedef enum
{
    UI_NONE   = 0,  /*�ղ���*/
    UI_ADD    = 1,  /*����ͼ��*/
    UI_MODIFY = 2,  /*�޸�ͼ��*/
    UI_DELETE = 3,  /*ɾ��ͼ��*/
}Graphic_Operate;//graphic_data_struct_t��uint32_t operate_tpye
/*ͼ�����*/

//bit3-5
/*ͼ������*/
typedef enum
{
    UI_LINE      = 0,//ֱ��
    UI_RECTANGLE = 1,//����
    UI_CIRCLE    = 2,//��Բ
    UI_OVAL      = 3,//��Բ
    UI_ARC       = 4,//Բ��
    UI_FLOAT     = 5,//������
    UI_INT       = 6,//������
    UI_CHAR      = 7 //�ַ�
}Graphic_Type;

//bit 6-9ͼ���� ���Ϊ9����С0

//bit 10-13��ɫ
typedef enum
{
    UI_RED_BLUE  = 0,   /* ������ɫ */
    UI_YELLOW    = 1,
    UI_GREEN     = 2,
    UI_ORANGE    = 3,
    UI_FUCHSIA   = 4,	/* �Ϻ�ɫ */
    UI_PINK      = 5,
    UI_CYAN_BLUE = 6,	/* ��ɫ */
    UI_BLACK     = 7,
    UI_WHITE     = 8
}Graphic_Color;

typedef enum {

    UI_ZERO_LAYER=0,
    UI_ONE_LAYER,
    UI_TWO_LAYER,
    UI_THREE_LAYER,
    UI_FOUR_LAYER,
    UI_FIVE_LAYER,
    UI_SIX_LAYER,
    UI_SEVEN_LAYER,
    UI_EIGHT_LAYER,

}Graphic_layer;
/*ͼ����ɫ����*/
//bit 14-31 �Ƕ� [0,360]

/*
 * ���ݽṹ��
 */

//ɾ��ͼ��
typedef  struct
{
    frame_header_struct_t txFrameHeader;
    uint16_t  CmdID;
    ext_student_interactive_header_data_t   dataFrameHeader;
    ext_client_custom_graphic_delete_t clientData;
    uint16_t	FrameTail;
} __packed deleteLayer_data_t;

/* ID:   0x0120  Byte:4  �ڱ���������ָ�� */
typedef  struct
{
    uint32_t sentry_cmd;
} __packed ext_sentry_cmd_t;

/* ID:   0x0121  Byte:1  �״���������ָ�� */
typedef  struct
{
    uint8_t radar_cmd;
} __packed ext_radar_cmd_t;


//��������ui����ɫ
typedef struct
{
    uint32_t cover_color;       // ���տ���
    uint32_t auto_aim_color;    // ���鿪��
    uint32_t spin_color;        // ���ݿ�����
    uint32_t change_color;      // ����������
    uint32_t fire_color;        // Ħ���ֿ�����
    uint32_t gimbal_color;      // ��̨�Ƿ���
    uint32_t shoot_color;       // �����Ƿ�ת��
    uint32_t cap_color;         // �����Ƿ���
} __packed ext_ui_color;

typedef struct
{
    int16_t launcher_speed_rpm; // Ħ����ת��
    uint32_t spin_startangle;   // UI spin��ȱ����ʼ�Ƕ�
    uint32_t spin_endangle;     // UI spin��ȱ����ֹ�Ƕ�
    uint32_t cap_endangle;    // UI ����Բ����ʼ�Ƕ�
} __packed ext_ui_change;

extern ui_robot_status_t ui_robot_status;
extern Referee_info_t Referee;
extern void referee_task(void const*argument);
extern void UI_paint_task(void const*argument);
extern uint8_t usart6_buf[REFEREE_BUFFER_SIZE];
extern uint8_t usart1_buf[REFEREE_BUFFER_SIZE];

extern float all_rpm_mul_current;
extern float all_current_pingfang;

#endif //DEMO1_REFEREE_H
