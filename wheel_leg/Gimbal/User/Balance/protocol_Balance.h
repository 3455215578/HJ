#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"
#include "../App/Referee_Task/referee_task.h"
#define HEADER_SOF 0xA5
#define END1_SOF   0x0D
#define END2_SOF   0x0A
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    CHASSIS_ODOM_CMD_ID = 0x0101,
    CHASSIS_CTRL_CMD_ID = 0x0102,
    RGB_ID              = 0x0103,
    RC_ID               = 0x0104,
    VISION_ID           = 0x0105
} data_cmd_id;

typedef enum
{
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;

//����ṹ��
typedef struct
{
    frame_header_struct_t *p_header;
    uint16_t       data_len;    // ���ݳ���
    uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e  unpack_step;
    uint16_t       index;
} unpack_data_t;

typedef struct
{
    uint8_t end1;
    uint8_t end2;
} msg_end_info ;

typedef struct
{
    float vx;
    float vy;
    float vw;
}  chassis_odom_info_t;

//���Ӿ��Ϸ��͵�����
typedef struct
{
    uint16_t id;        //
    uint16_t mode;      // auto mode 0x21
    fp32 pitch;         // �����ǵ�pitch
    fp32 yaw;           // �����ǵ�yaw
    fp32 roll;          // �����ǵ�roll��Ӣ��û���õ�
    fp32 quaternion[4]; // ��Ԫ��
    fp32 shoot_speed;   // �����ٶ�
} vision_t;

//�����˿�������
typedef struct
{
    fp32 vx;
    fp32 vy;
    fp32 vw;
    fp32 yaw;
    fp32 pitch;
    int8_t target_lock;
    int8_t fire_command;
//    int8_t aim_id;
}  robot_ctrl_info_t;

// ң����
typedef struct
{
    int16_t ch[5];
    char s[2];
} rc_info_t;

// ����c��LED��
typedef struct
{
    uint16_t R;
    uint16_t G;
    uint16_t B;
} RBG_info_t;

#pragma pack(pop)

#endif
