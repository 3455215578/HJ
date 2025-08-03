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

//通信协议格式 位移量
typedef enum
{
    FRAME_HEADER= 0,
    CMD_ID               = 5,
    DATA                 = 7,
}RefereeFrameOffset;

// frame_header 格式
typedef enum
{
    SOF          = 0,//起始位
    DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
    SEQ          = 3,//包序号
    CRC8         = 4 //CRC8
}FrameHeaderOffset;


//裁判系统命令ID
/***************命令码ID********************
    / 服务器→全体机器人
    ID: 0x0001  Byte:  11   比赛状态数据,固定以1Hz频率发送
    ID: 0x0002  Byte:   1   比赛结果数据,比赛结束后发送
    ID: 0x0003  Byte:  32   比赛机器人血量数据,固定以3Hz频率发送
    / 服务器→己方全体机器人
    ID: 0x0101  Byte:   4   场地事件数据,固定以1Hz频率发送
    / 服务器→被判罚方全体机器人
    ID: 0X0104  Byte:   3   裁判警告数据,己方判罚/判负时触发发送,其余时间以1Hz频率发送
    / 服务器→己方全体机器人
    ID: 0x0105  Byte:   3   飞镖发射相关数据,固定以1Hz频率发送
    / 主控模块→对应机器人
    ID: 0X0201  Byte:  13    机器人性能体系数据,固定以10Hz频率发送
    ID: 0X0202  Byte:  16    实时底盘缓冲能量和射击热量数据,固定以10Hz频率发送
    ID: 0x0203  Byte:  16    机器人位置数据,固定以1Hz频率发送
    / 服务器→对应机器人
    ID: 0x0204  Byte:   7    机器人增益和底盘能量数据,固定以3Hz频率发送
    / 主控模块→对应机器人
    ID: 0x0206  Byte:   1    伤害状态数据,伤害发生后发送
    ID: 0x0207  Byte:   7    实时射击数据,弹丸发射后发送
    / 服务器→己方英雄、步兵、哨兵、空中机器人
    ID: 0x0208  Byte:   6    允许发弹量,固定以10Hz频率发送
    / 服务器→己方装有RFID模块的机器人
    ID: 0x0209  Byte:   4    机器人RFID模块状态，固定以3Hz频率发送
    / 服务器→己方机器人
    ID: 0x020A  Byte:   6    飞镖选手端指令数据，固定以3Hz频率发送
    ID: 0x020B  Byte:  40    地面机器人位置数据，固定以1Hz频率发送
    ID: 0x020C  Byte:   1    雷达标记进度数据，固定以1Hz频率发送
    ID: 0x020D  Byte:   6    哨兵自主决策信息同步，固定以1Hz频率发送
    ID: 0x020E  Byte:   1    雷达自主决策信息同步，固定以1Hz频率发送

    ID: 0x0301  Byte: 127    机器人交互数据，发送方触发发送，频率上限为30Hz
    / 自定义控制器→选手端图传连接的机器人
    ID: 0x0302  Byte:  30    自定义控制器与机器人交互数据，发送方触发发送，频率上限为30Hz
    / 选手端点击→服务器→发送方选择的己方机器人
    ID: 0x0303  Byte:  15    选手端小地图交互数据，选手端触发发送
    / 选手端→选手端图传连接的机器人
    ID: 0x0304  Byte:  12    键鼠遥控数据，固定30Hz频率发送
    / 雷达→服务器→己方所有选手端
    ID: 0x0305  Byte:  24    选手端小地图接收雷达数据，频率上限为5Hz
    / 自定义控制器→选手端
    ID: 0x0306  Byte:   8    自定义控制器与选手端交互数据，发送方触发发送，频率上限为30Hz
    / 哨兵/半自动控制机器人→对应操作手选手端
    ID: 0x0307  Byte: 103    选手端小地图接收哨兵数据，频率上限为1Hz
    / 己方机器人→己方选手端
    ID: 0x0308  Byte:  34    选手端小地图接收机器人数据，频率上限为3Hz
    / 己方机器人→对应操作手选手端连接的自定义控制器
    ID: 0x0309  Byte:  30    自定义控制器接收机器人数据，频率上限为10Hz
*/


typedef enum
{
    Referee_ID_game_state                   = 0x0001,//比赛状态数据
    Referee_ID_game_result                  = 0x0002,//比赛结果数据
    Referee_ID_game_robot_HP                = 0x0003,//机器人血量数据

    Referee_ID_event_data                   = 0x0101,//场地事件数据
    Referee_ID_supply_warm                  = 0x0104,//裁判系统警告数据
    Referee_ID_dart_info                    = 0x0105,//飞镖发射相关数据

    Referee_ID_game_robot_state             = 0x0201,//机器人性能体系数据
    Referee_ID_power_heat_data              = 0x0202,//实时底盘缓冲能量和射击热量数据
    Referee_ID_game_robot_pos               = 0x0203,//机器人位置数据
    Referee_ID_buff_musk                    = 0x0204,//机器人增益和底盘能量数据
    Referee_ID_robot_hurt                   = 0x0206,//伤害状态数据，伤害发生后发送
    Referee_ID_shoot_data                   = 0x0207,//实时射击数据，弹丸发射后发送
    Referee_ID_bullet_remaining             = 0x0208,//允许发弹量
    Referee_ID_rfid_status                  = 0x0209,//机器人RFID状态，3Hz
    Referee_ID_dart_client_directive        = 0x020A,//飞镖选手端指令数据, 3Hz
    Referee_ID_dart_all_robot_position      = 0x020B,//地面机器人位置数据
    Referee_ID_radar_mark                   = 0x020C,//雷达标记进度数据
    Referee_ID_entry_info                   = 0x020D,//哨兵自主决策信息同步 1Hz
    Referee_ID_radar_info                   = 0x020E,//雷达自主决策信息同步 1Hz

    Referee_ID_robot_interactive_header_data        = 0x0301,//机器人交互数据，——发送方触发——发送 30Hz
    Referee_ID_controller_interactive_header_data   = 0x0302,//自定义控制器与机器人交互数据，发送方触发发送，频率上限30Hz
    Referee_ID_map_command                          = 0x0303,//选手端小地图交互数据，选手端触发发送
    Referee_ID_keyboard_information                 = 0x0304,//键鼠遥控数据，固定30Hz,图传链路
    Referee_ID_robot_map_robot_data                 = 0x0305,//选手端小地图接收雷达数据,上限5Hz
    Referee_ID_robot_custom_client                  = 0x0306,//自定义控制器与选手端交互数据，发送方触发发送，频率上限30Hz
    Referee_ID_robot_entry_info_receive             = 0x0307,//选手端小地图接收哨兵数据,上限1Hz
    Referee_ID_robot_custom_info_receive            = 0x0308,//通过常规链路接收机器人的数据,在特定位置显示，上限3Hz
    Referee_ID_robot_custom                         = 0x0309,//自定义控制器接收机器人数据，频率上限为10Hz
}referee_cmd_id_t;

//裁判系统各命令的数据长度
typedef enum
{
    /* Std通信协议格式 */
    Referee_LEN_FRAME_HEAD                      = 5,   // 帧头长度 frame_header
    Referee_LEN_CMD_ID                          = 2,   // 命令码长度 cmd_id
    Referee_LEN_FRAME_TAIL                      = 2,   // 帧尾CRC16 frame_tail
    /* Ext */

    Referee_LEN_game_state                      =  11, //0x0001
    Referee_LEN_game_result                     =  1,  //0x0002
    Referee_LEN_game_robot_HP                   =  32, //0x0003  比赛机器人血量数据

    Referee_LEN_event_data                      =  4,  //0x0101  场地事件数据
    Referee_LEN_supply_warm                     =  3,   //0x0104 裁判系统警告
    Referee_LEN_dart_info                       =  3,   //0x0105 飞镖发射口倒计时

    Referee_LEN_game_robot_state                = 13,  //0x0201 机器人状态数据
    Referee_LEN_power_heat_data                 = 16,  //0x0202 实时功率热量数据
    Referee_LEN_game_robot_pos                  = 16,  //0x0203 机器人位置数据
    Referee_LEN_buff_musk                       =  7,  //0x0204 机器人增益数据
    Referee_LEN_robot_hurt                      =  1,  //0x0206 伤害状态数据
    Referee_LEN_shoot_data                      =  7,  //0x0207 实时射击数据
    Referee_LEN_bullet_remaining                =  6,    //0x0208剩余发射数
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

    bool static_update;//静态元素是否要刷新
    uint8_t gimbal_mode;//云台模式 是人控 还是自控（自控是打符还是自瞄)
    uint8_t chassis_mode;//
    uint8_t block_warning;//堵弹警告
    uint8_t shoot_heat_limit;//当前热量限制
    fp32 super_cap_value;//超级电容值
    uint8_t fire_mode;
    float pitch_value;
    float relative_yaw_value;
}ui_robot_status_t;//机器人状态结构体 如果陀螺是否打开,弹舱是否打开，自瞄是否打开等信息

//裁判系统帧头结构体
typedef  struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
}__packed frame_header_struct_t;


/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef  struct
{
    uint8_t game_type : 4;      // 比赛类型
    uint8_t game_progress : 4;  // 当前比赛阶段
    uint16_t stage_remain_time; // 当前阶段剩余时间，单位：秒
    uint64_t SyncTimeStamp;     // UNIX时间，当机器人正确连接到裁判系统的NTP服务器后生效
} __packed ext_game_state_t;


/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef  struct
{
    uint8_t winner;         // 0：平局 1：红方胜利 2：蓝方胜利
}__packed  ext_game_result_t;


/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */
typedef  struct
{
    uint16_t red_1_robot_HP;    // 红1英雄机器人血量。若该机器人未上场或者被罚下，则血量为0
    uint16_t red_2_robot_HP;    // 红2工程机器人血量
    uint16_t red_3_robot_HP;    // 红3步兵机器人血量
    uint16_t red_4_robot_HP;    // 红4步兵机器人血量
    uint16_t reserved1;         // 保留位
    uint16_t red_7_robot_HP;    // 红7哨兵机器人血量
    uint16_t red_outpost_HP;    // 红方前哨站血量
    uint16_t red_base_HP;       // 红方基地血量

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t reserved2;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} __packed ext_game_robot_HP_t;


/** ID: 0x0101  Byte:  4    场地事件数据
 * 0：未占领/未激活
 * 1：已占领/已激活
 * bit 0-2：
 * bit 0：己方与兑换区不重叠的补给区占领状态，1为已占领
 * bit 1：己方与兑换区重叠的补给区占领状态，1为已占领
 * bit 2：己方补给区的占领状态，1为已占领（仅 RMUL 适用）
 * bit 3-5：己方能量机关状态
 * bit 3：己方小能量机关的激活状态，1为已激活
 * bit 4：己方大能量机关的激活状态，1为已激活
 * bit 5-6：己方中央高地的占领状态，1为被己方占领，2为被对方占领
 * bit 7-8：己方梯形高地的占领状态，1为已占领
 * bit 9-17：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
 * bit 18-20：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为0，1为击中前哨站，2为击中基地固定目标，3为击中基地随机固定目标，4为击中基地随机移动目标
 * bit 21-22：中心增益点的占领状态，0为未被占领，1为被己方占领，2为被对方占领，3为被双方占领。（仅RMUL适用）
 * bit 23-31：保留位
 */
typedef  struct
{
    uint32_t event_type;
} __packed ext_event_data_t;


/* ID: 0x0104  Byte: 2->3   裁判系统警告信息 */
typedef  struct
{
    uint8_t level;              // 己方最后一次受到判罚的等级
    uint8_t offending_robot_id; // 己方最后一次受到判罚的违规机器人ID。（如红1机器人ID为1，蓝1机器人ID为101）
    uint8_t count;              // 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为0。）
} __packed  ext_referee_warning_t;

/** ID: 0x0105  Byte:1->3  飞镖发射口倒计时
 * bit 0-2：
 * 最近一次己方飞镖击中的目标，开局默认为0，1为击中前哨站，2为击中基地固定目标，3为击中基地随机固定目标，4为击中基地随机移动目标
 * bit 3-5：
 * 对方最近被击中的目标累计被击中计次数，开局默认为0，至多为4
 * bit 6-7：
 * 飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为0，选中基地固定目标为1，选中基地随机固定目标为2，选中基地随机移动目标为3
 * bit 8-15：保留
 */
typedef  struct
{
    uint8_t dart_remaining_time;    // 己方飞镖发射剩余时间，单位：秒
    uint16_t dart_info;             //新增
} __packed ext_dart_remaining_time_t;

/* ID: 0X0201  Byte:     机器人状态数据 */
typedef  struct
{
    uint8_t robot_id;       // 本机器人ID
    uint8_t robot_level;    // 机器人等级
    uint16_t current_HP;    // 机器人当前血量
    uint16_t maximum_HP;    // 机器人血量上限

    uint16_t shooter_barrel_cooling_value;  // 机器人射击热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;     // 机器人射击热量上限
    uint16_t chassis_power_limit;           // 机器人底盘功率上限
    /* 电源管理模块的输出情况 */
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} __packed ext_robot_status_t;

/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef  struct
{
    uint16_t reserved1;
    uint16_t reserved2;
    float    reserved3;
    uint16_t buffer_energy;                 // 缓冲能量（单位：J）
    uint16_t shooter_17mm_1_barrel_heat;    // 第1个 17mm 发射机构的枪口热量
    uint16_t shooter_17mm_2_barrel_heat;    // 第2个 17mm 发射机构的枪口热量
    uint16_t shooter_42mm_barrel_heat;      // 42mm 发射机构的枪口热量
} __packed ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef  struct
{
    float x;    // 本机器人位置x坐标，单位：m
    float y;    // 本机器人位置y坐标，单位：m
    float angle;// 本机器人测速模块的朝向，单位：度。正北为0度
}__packed ext_robot_pos_t;


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef struct
{
    uint8_t recovery_buff;          // 机器人回血增益（百分比，值为10表示每秒恢复血量上限的10%）
    uint8_t cooling_buff;           // 机器人射击热量冷却倍率（直接值，值为5表示5倍冷却）
    uint8_t defence_buff;           // 机器人防御增益（百分比，值为50表示50%防御增益）
    uint8_t vulnerability_buff;     // 机器人负防御增益（百分比，值为30表示-30%防御增益）
    uint16_t attack_buff;           // 机器人攻击增益（百分比，值为50表示50%攻击增益）
    /**bit 0-4：机器人剩余能量值反馈，以16进制标识机器人剩余能量值比例，
     * 仅在机器人剩余能量小于50%时反馈，其余默认反馈0x32。 00110 0 10       000111111
     * bit 0：在剩余能量≥50%时为1，其余情况为0
     * bit 1：在剩余能量≥30%时为1，其余情况为0
     * bit 2：在剩余能量≥15%时为1，其余情况为0
     * bit 3：在剩余能量≥5%时为1，其余情况为0Bit4：在剩余能量≥1%时为1，其余情况为0g
     */
    uint8_t remaining_energy;       // 机器人剩余能量反馈值
}__packed  ext_buff_t;


/** ID: 0x0206  Byte:  1    伤害状态数据
 * bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，该4 bit组成的数值为装甲模块或测速模块的ID编号；当其他原因导致扣血时，该数值为0
 * bit 4-7：血量变化类型
 * 0：装甲模块被弹丸攻击导致扣血
 * 1：裁判系统重要模块离线导致扣血
 * 2：射击初速度超限导致扣血
 * 3：射击热量超限导致扣血
 * 4：底盘功率超限导致扣血
 * 5：装甲模块受到撞击导致扣血
 */
typedef  struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} __packed ext_robot_hurt_t;


/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef  struct
{
    uint8_t bullet_type;    // 1：17mm弹丸 2：42mm弹丸
    uint8_t shooter_id;     // 1：第1个17mm发射机构  2：第2个17mm发射机构 3：42mm发射机构
    uint8_t bullet_freq;    // 弹丸射速（单位：Hz）
    float bullet_speed;     // 弹丸初速度（单位：m/s）
}__packed  ext_shoot_data_t;


/* ID: 0x0208  Byte:  6    子弹剩余数量 */
typedef  struct
{
    uint16_t projectile_allowance_17mm; // 17mm弹丸允许发弹量
    uint16_t projectile_allowance_42mm; // 42mm弹丸允许发弹量
    uint16_t remaining_gold_coin;       // 剩余金币数量
} __packed ext_bullet_remaining_t;

/** ID: 0x0209  Byte:  4 	机器人RFID状态
 * bit位值为1/0的含义：是否已检测到该增益点RFID卡
 * bit 0：己方基地增益点
 * bit 1：己方中央高地增益点
 * bit 2：对方中央高地增益点
 * bit 3：己方梯形高地增益点
 * bit 4：对方梯形高地增益点
 * bit 5：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
 * bit 6：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
 * bit 7：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
 * bit 8：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
 * bit 9：己方地形跨越增益点（中央高地下方）
 * bit 10：己方地形跨越增益点（中央高地上方）
 * bit 11：对方地形跨越增益点（中央高地下方）
 * bit 12：对方地形跨越增益点（中央高地上方）
 * bit 13：己方地形跨越增益点（公路下方）
 */
typedef  struct
{
    uint32_t rfid_status;
} __packed ext_rfid_status_t;


/* ID:  0x020A  Byte:6   	 */
typedef  struct
{
    uint8_t dart_launch_opening_status;     // 飞镖发射状态 0 已经开启, 1 关闭 2 正在开启或者关闭中,
    uint8_t reserved;                       // 保留
    uint16_t target_change_time;            // 切换击打目标时的比赛剩余时间，单位s,默认为0
    uint16_t latest_launch_cmd_time;        // 最后一次操作手确定发射指令时的比赛剩余时间,单位s,初始值为0
}__packed ext_dart_client_cmd_t;

//V1.6.1新增 24赛季自动控制
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

//V1.6.1新增 24赛季雷达修改
/* ID:   0x020C  Byte:6  机器人被雷达标记进度 0-120	 */
typedef  struct
{
    uint8_t mark_progress;   //bit0-4 分别对应对方英雄、工程、步兵3 4号、哨兵的易伤情况
}__packed ext_radar_mark_data_t;

//V1.6.1新增 24赛季哨兵修改
/* ID:   0x020D  Byte:4  哨兵兑换发单量和血量信 */
typedef  struct
{
    uint32_t sentry_info;
    uint32_t sentry_info_2;
} __packed ext_sentry_info_t;

//V1.6.1新增 24赛季雷达修改
/* ID:   0x020E  Byte:1  雷达触发双伤信息	 */
typedef struct
{
    uint8_t radar_info;
} __packed  ext_radar_info_t;

/*
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。

	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	11，英雄(蓝)；
	12，工程(蓝)；
	13/14/15，步兵(蓝)；
	16，空中(蓝)；
	17，哨兵(蓝)。
	客户端 ID：
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)；
	0x0111，英雄操作手客户端(蓝)；
	0x0112，工程操作手客户端(蓝)；
	0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
	0x0116，空中操作手客户端(蓝)。
*/


/* 交互数据接收信息：0x0301  */
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

/* 选手端小地图交互数据：0x0303  */
typedef  struct
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
}__packed ext_map_command_t;

/* 键鼠遥控数据：0x0304  */
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

/* 选手端小地图接收雷达数据：0x0305  */
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

/* 自定义控制器与选手端交互数据：0x0306  */
typedef struct
{
    uint16_t key_value;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
}__packed ext_custom_client_data_t;

/* 选手端小地图接收哨兵数据：0x0307  */
typedef  struct
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
}__packed ext_map_data_t;

/* 选手端小地图接收机器人数据：0x0308  */
typedef  struct
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} __packed ext_custom_info_t;

/*
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz

	字节偏移量 	大小 	说明 			备注
	0 			2 		数据的内容 ID 	0x0200~0x02FF
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义

	2 			2 		发送者的 ID 	需要校验发送者的 ID 正确性，

	4 			2 		接收者的 ID 	需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID

	6 			n 		数据段 			n 需要小于 113

*/
typedef  struct
{
    uint8_t data[113]; //数据段,n需要小于113
} __packed robot_interactive_data_t;

typedef struct judge_info_struct {
    frame_header_struct_t 							FrameHeader;				// 帧头信息

    ext_game_state_t 							    GameState;				    // 0x0001           比赛状态数据
    ext_game_result_t 							    GameResult;				    // 0x0002         比赛结果数据
    ext_game_robot_HP_t 						    GameRobotHP;			    // 0x0003         机器人血量数据

    ext_event_data_t								EventData;					// 0x0101         场地事件数据
    ext_referee_warning_t						    RefereeWarning;		        // 0x0104         裁判警告信息
    ext_dart_remaining_time_t				        DartRemainingTime;          // 0x0105         飞镖发射口倒计时


    ext_robot_status_t					            GameRobotStat;	            // 0x0201         比赛机器人状态
    ext_power_heat_data_t						    PowerHeatData;		        // 0x0202         实时功率热量数据
    ext_robot_pos_t						            GameRobotPos;			    // 0x0203         机器人位置
    ext_buff_t									    Buff;						// 0x0204     机器人增益
    ext_robot_hurt_t								RobotHurt;					//0x0206         伤害状态
    ext_shoot_data_t								ShootData;					//0x0207         实时射击信息(射频  射速  子弹信息)
    ext_bullet_remaining_t					        BulletRemaining;		    //0x0208	        子弹剩余发射数
    ext_rfid_status_t								RfidStatus;				    //0x0209	        RFID信息
    ext_dart_client_cmd_t                           DartClient;                 //0x020A         飞镖客户端
    ext_ground_robot_position_t                     RobotPosition;              //0x020B
    ext_radar_mark_data_t                           RadarMark;                  //0x020C
    ext_sentry_info_t                               SentryInfo;                 //0x020D
    ext_radar_info_t                                RadarInfo;                  //0x020E

    ext_student_interactive_header_data_t           StudentInteractive;         //0x0301
    ext_map_command_t                               MapCommand;                 //0x0303
    ext_remote_control_t                            keyboard;                   //0x0304 键鼠
    ext_map_robot_data_t                            EnemyPosition;              //0x0305 敌方机器人位置
    ext_custom_client_data_t                        Custom;                     //0x0306 自定义控制器
    ext_map_data_t                                  SentryMapData;              //0x0307 哨兵发送数据
    ext_custom_info_t                               SendData;                   //0x0308 机器人自定义发送消息

    ext_interact_id_t								ids;			            //与本机交互的机器人id
    uint16_t                                        SelfClient;                 //本机客户端

} Referee_info_t;


/*
	学生机器人间通信 cmd_id 0x0301，内容 data_ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：数据上下行合计带宽不超过 5000 Byte。 上下行发送频率分别不超过30Hz。
 * +------+------+-------------+------------------------------------+
 * | byte | size |    breif    |            note                    |
 * |offset|      |             |                                    |
 * +------+------+-------------+------------------------------------+
 * |  0   |  2   | 	 data_ID   | 0x0200~0x02FF,可以在这些 ID 段选取    |
 * |      |      |             | 具体ID含义由参赛队自定义               |
 * +------|------|-------------|------------------------------------|
 * |  2   |  2   | 	sender_ID  | 需要校验发送者的 ID 正确性				|
 * +------|------|-------------|------------------------------------|
 * |  4   |  2   | receiver_ID | 需要校验接收者的 ID 正确性				|
 * |      |      |             | 例如不能发送到敌对机器人的ID			|
 * +------|------|-------------|------------------------------------|
 * |  6   |  n   |    Data     | n 需要小于 113 										|
 * +------+------+-------------+------------------------------------+
*/


//绘制图形ID
typedef enum
{
    //0x200-0x02ff 	队伍自定义命令 格式  INTERACT_ID_XXXX
    UI_INTERACT_ID_delete_graphic 			= 0x0100,	/*客户端删除图形*/
    UI_INTERACT_ID_draw_one_graphic 		= 0x0101,	/*客户端绘制一个图形*/
    UI_INTERACT_ID_draw_two_graphic 		= 0x0102,	/*客户端绘制2个图形*/
    UI_INTERACT_ID_draw_five_graphic 	    = 0x0103,	/*客户端绘制5个图形*/
    UI_INTERACT_ID_draw_seven_graphic 	    = 0x0104,	/*客户端绘制7个图形*/
    UI_INTERACT_ID_draw_char_graphic 	    = 0x0110,	/*客户端绘制字符图形*/
    UI_INTERACT_ID_bigbome_num				= 0x02ff
}Interact_ID;

typedef enum
{
    UI_LEN_INTERACT_delete_graphic     = 8,  //删除图层 2(数据内容ID)+2(发送者ID)+2（接收者ID）+2（数据内容）
    UI_LEN_INTERACT_draw_one_graphic   = 21, // 以上2+2+2+15
    UI_LEN_INTERACT_draw_two_graphic   = 36, //6+15*2
    UI_LEN_INTERACT_draw_five_graphic  = 81, //6+15*5
    UI_LEN_INTERACT_draw_seven_graphic = 111,//6+15*7
    UI_LEN_INTERACT_draw_char_graphic  = 51, //6+15+30（字符串内容）
}Interact_ID_len;

//图形数据
typedef  struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;          //直线  矩形  正圆  椭圆  圆弧  浮点  整型  字符
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;           //空    空    空    空    角度  大小  大小  大小
    uint32_t end_angle:9;             //空    空    空    空          位数  空    长度
    uint32_t width:10;
    uint32_t start_x:11;              //起点  起点  圆心  圆心  圆心  起点  起点  起点
    uint32_t start_y:11;              //
    union {
        struct {
            uint32_t radius:10;      //空    空    半径  空    空    、    、    空
            uint32_t end_x:11;       //终点  对顶  空    半轴  半轴  、    、    空
            uint32_t end_y:11;       //                              数    数    空                  数    数    空
        };
        int32_t number;
    };
} __packed ui_graphic_data_struct_t;//ui开头 将整形和浮点型也视为图形 所以用graphic命名

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//帧头
    uint16_t  CmdID;                                //命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    ui_graphic_data_struct_t clientData;            //数据段
    uint16_t	FrameTail;                          //帧尾
}__packed ext_graphic_one_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//帧头
    uint16_t  CmdID;                                //命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    ui_graphic_data_struct_t clientData[2];		    //数据0x0201段
    uint16_t	FrameTail;							//帧尾
}__packed ext_graphic_two_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//帧头
    uint16_t  CmdID;                                //命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    ui_graphic_data_struct_t clientData[5];		    //数据段
    uint16_t	FrameTail;							//帧尾
}__packed ext_graphic_five_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//帧头
    uint16_t  CmdID;								//命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    ui_graphic_data_struct_t clientData[7];		    //数据段
    uint16_t	FrameTail;							//帧尾
}__packed ext_graphic_seven_data_t;

//绘字符串
//字符串除了ui_graphic_data_struct_t这个结构体变量外
// 还有30个字节的字符数据存储字符串
typedef  struct
{
    ui_graphic_data_struct_t graphic_data_struct;
    uint8_t data[30];
}__packed ui_string_t;

//固定数据段长度数据包
typedef  struct
{
    frame_header_struct_t txFrameHeader;			//帧头
    uint16_t  CmdID;								//命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    ui_string_t clientData;                         //数据段
    uint16_t	FrameTail;							//帧尾
}__packed ext_string_data_t;

//****************************绘图的数据段内容****************************/
/* data_ID: 0X0100  Byte:  2	    客户端删除图形*/
typedef  struct
{
    uint8_t operate_type;
    uint8_t layer;//图层数：0~9
}__packed ext_client_custom_graphic_delete_t;

/* 图层删除操作的枚举 */
typedef enum
{
    UI_NONE_delete    = 0,
    UI_GRAPHIC_delete = 1,
    UI_ALL_delete     = 2
}Delete_Graphic_Operate;//ext_client_custom_graphic_delete_t：uint8_t operate_type

//bit 0-2
typedef enum
{
    UI_NONE   = 0,  /*空操作*/
    UI_ADD    = 1,  /*增加图层*/
    UI_MODIFY = 2,  /*修改图层*/
    UI_DELETE = 3,  /*删除图层*/
}Graphic_Operate;//graphic_data_struct_t：uint32_t operate_tpye
/*图层操作*/

//bit3-5
/*图层类型*/
typedef enum
{
    UI_LINE      = 0,//直线
    UI_RECTANGLE = 1,//矩形
    UI_CIRCLE    = 2,//正圆
    UI_OVAL      = 3,//椭圆
    UI_ARC       = 4,//圆弧
    UI_FLOAT     = 5,//浮点数
    UI_INT       = 6,//整型数
    UI_CHAR      = 7 //字符
}Graphic_Type;

//bit 6-9图层数 最大为9，最小0

//bit 10-13颜色
typedef enum
{
    UI_RED_BLUE  = 0,   /* 红蓝主色 */
    UI_YELLOW    = 1,
    UI_GREEN     = 2,
    UI_ORANGE    = 3,
    UI_FUCHSIA   = 4,	/* 紫红色 */
    UI_PINK      = 5,
    UI_CYAN_BLUE = 6,	/* 青色 */
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
/*图层颜色类型*/
//bit 14-31 角度 [0,360]

/*
 * 数据结构体
 */

//删除图层
typedef  struct
{
    frame_header_struct_t txFrameHeader;
    uint16_t  CmdID;
    ext_student_interactive_header_data_t   dataFrameHeader;
    ext_client_custom_graphic_delete_t clientData;
    uint16_t	FrameTail;
} __packed deleteLayer_data_t;

/* ID:   0x0120  Byte:4  哨兵自主决策指令 */
typedef  struct
{
    uint32_t sentry_cmd;
} __packed ext_sentry_cmd_t;

/* ID:   0x0121  Byte:1  雷达自主决策指令 */
typedef  struct
{
    uint8_t radar_cmd;
} __packed ext_radar_cmd_t;


//用来配置ui的颜色
typedef struct
{
    uint32_t cover_color;       // 弹舱开否
    uint32_t auto_aim_color;    // 自瞄开否
    uint32_t spin_color;        // 陀螺开启否
    uint32_t change_color;      // 反拨开启否
    uint32_t fire_color;        // 摩擦轮开启否
    uint32_t gimbal_color;      // 云台是否开启
    uint32_t shoot_color;       // 拨盘是否转动
    uint32_t cap_color;         // 电容是否开启
} __packed ext_ui_color;

typedef struct
{
    int16_t launcher_speed_rpm; // 摩擦轮转速
    uint32_t spin_startangle;   // UI spin的缺口起始角度
    uint32_t spin_endangle;     // UI spin的缺口终止角度
    uint32_t cap_endangle;    // UI 电容圆弧起始角度
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
