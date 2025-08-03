
#include "referee_task.h"
#include "launcher.h"
#include "gimbal_task.h"
#include "key_board.h"

uint8_t Referee_ID;
uint8_t Referee_Hit_Flag;
//��ģʽ����ʱ�ļ��
#define UI_CHAR_LENGTH 15
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;

extern key_board_t KeyBoard;
//extern chassis_t chassis;//��ȡ����ģʽ
extern gimbal_t gimbal;//��ȡ��̨ģʽ

ext_ui_color uiColor;//�ж�ui��ɫ
ext_ui_change uiChange; //���ƶ�̬UI�Ĳ���

ext_ui_color uiColor;//�ж�ui��ɫ

Graphic_Operate static_update_flag=UI_ADD;
Graphic_Operate one_layer_update_flag=UI_NONE;
Graphic_Operate two_layer_update_flag=UI_NONE;
Graphic_Operate three_layer_update_flag=UI_NONE;
Delete_Graphic_Operate delete_all_layer = UI_ALL_delete;

uint8_t usart1_buf[REFEREE_BUFFER_SIZE]={0};
uint8_t usart6_buf[REFEREE_BUFFER_SIZE]={0};

Referee_info_t Referee;

uint8_t bit_2;

/*����������*/
static void referee_unpack_fifo_data(void);
static bool Referee_read_data(uint8_t *ReadFromUsart);
static void ui_static_draw();
/*����ϵͳ������*/

extern float INS_angle[3];

void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);//��ȡUART1-SR ��UART1-DR; ����жϱ�־λ

        __HAL_DMA_DISABLE(huart1.hdmarx); //ʹ��dma_rx

        Referee_read_data(&usart1_buf[0]);

        memset(&usart1_buf[0],0,REFEREE_BUFFER_SIZE);//��0

        __HAL_DMA_CLEAR_FLAG(huart1.hdmarx,DMA_LISR_TCIF1); //���������ɱ�־λ

        __HAL_DMA_SET_COUNTER(huart1.hdmarx, REFEREE_BUFFER_SIZE);//����DMA �������ݴ�С ��λΪ�ֽ�

        __HAL_DMA_ENABLE(huart1.hdmarx); //ʹ��DMARx

        detect_handle(DETECT_VIDEO_TRANSIMITTER);
    }
}


//���ݲ���ϵͳ��Ϣ�жϻ����˵�ID�Ͷ�Ӧ�ͻ��˵�ID
void judge_team_client(){
    //��������Ϊ�췽
    if(Referee.GameRobotStat.robot_id<10)
    {
        Referee.ids.teammate_hero 	   = 1;
        Referee.ids.teammate_engineer  = 2;
        Referee.ids.teammate_infantry3 = 3;
        Referee.ids.teammate_infantry4 = 4;
        Referee.ids.teammate_infantry5 = 5;
        Referee.ids.teammate_plane	   = 6;
        Referee.ids.teammate_sentry	   = 7;

        Referee.ids.client_hero 	 = 0x0101;
        Referee.ids.client_engineer  = 0x0102;
        Referee.ids.client_infantry3 = 0x0103;
        Referee.ids.client_infantry4 = 0x0104;
        Referee.ids.client_infantry5 = 0x0105;
        Referee.ids.client_plane	 = 0x0106;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_red:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;

            case Referee_engineer_red:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;

            case Referee_infantry3_red:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_red:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_red:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_red:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }//��������Ϊ����
    else{
        Referee.ids.teammate_hero 	   = 101;
        Referee.ids.teammate_engineer  = 102;
        Referee.ids.teammate_infantry3 = 103;
        Referee.ids.teammate_infantry4 = 104;
        Referee.ids.teammate_infantry5 = 105;
        Referee.ids.teammate_plane	   = 106;
        Referee.ids.teammate_sentry    = 107;

        Referee.ids.client_hero 	 = 0x0165;
        Referee.ids.client_engineer  = 0x0166;
        Referee.ids.client_infantry3 = 0x0167;
        Referee.ids.client_infantry4 = 0x0168;
        Referee.ids.client_infantry5 = 0x0169;
        Referee.ids.client_plane	 = 0x016A;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_blue:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;

            case Referee_engineer_blue:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;

            case Referee_infantry3_blue:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_blue:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_blue:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_blue:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }
}

float all_rpm_mul_current = 0;
float all_current_pingfang = 0;
bool Referee_read_data(uint8_t *ReadFromUsart)
{
    int CmdID=0;//�������������

    uint16_t judge_length;
    // Referee.RobotHurt.being_hurt = false;
    if(ReadFromUsart==NULL)
        return 0 ;

    memcpy(&Referee.FrameHeader,ReadFromUsart,Referee_LEN_FRAME_HEAD);

    if(ReadFromUsart[SOF]==REFREE_HEADER_SOF) //�ж�֡ͷ�Ƿ�Ϊ0xA5
    {
        if(verify_CRC8_check_sum(ReadFromUsart,LEN_HEADER)) //CRC ֡ͷУ��
        {
            judge_length=ReadFromUsart[DATA_LENGTH]+LEN_HEADER+Referee_LEN_CMD_ID+Referee_LEN_FRAME_TAIL;
            if(verify_CRC16_check_sum(ReadFromUsart,judge_length))  //֡βУ��
            {
//                retval_tf=1;//��ʾ���ݿ���
                CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)

                switch (CmdID)
                {

                    case Referee_ID_game_state://0x0001 ����״̬ 1HZ
                        memcpy(&Referee.GameState,ReadFromUsart+DATA,Referee_LEN_game_state);
                        break;

                    case Referee_ID_game_result://0x0002 �������   ������������
                        memcpy(&Referee.GameResult,ReadFromUsart+DATA,Referee_LEN_game_result);
                        // Referee.GameResult.game_over = true;
                        break;

                    case Referee_ID_game_robot_HP://0x0003 ������״̬HP   1HZ
                        memcpy(&Referee.GameRobotHP,ReadFromUsart+DATA,Referee_LEN_game_robot_HP);
                        break;

//V1.6.1ɾ��
//                    case Referee_ID_game_dart_state: //0x0004 ���ڷ���״̬
//                        memcpy(&Referee.GameDartStatus,ReadFromUsart+DATA,Referee_LED_game_missile_state);
//                        break;
//
//                    case Referee_ID_game_buff: //0x0005 ICRA_BUFF״̬     1HZ
//                        memcpy(&Referee.GameICRABuff,ReadFromUsart+DATA,Referee_LED_game_buff);
//                        break;

                    case Referee_ID_event_data://0x0101 �����¼�����      1HZ
                        memcpy(&Referee.EventData,ReadFromUsart+DATA,Referee_LEN_event_data);
                        break;

                        // case Referee_ID_supply_projectile_action://0x0102 ���ز���վ������ʶ����   �����ı�֮����
                        //     memcpy(&Referee.SupplyProjectileAction,ReadFromUsart+DATA,Referee_LEN_supply_projectile_action);
                        //     break;

                    case Referee_ID_supply_warm://0x0104    ����ϵͳ��������    ��������֮����
                        memcpy(&Referee.RefereeWarning,ReadFromUsart+DATA,Referee_LEN_supply_warm);
                        break;

                    case Referee_ID_dart_info://0x0105    ���ڷ���ڵ���ʱ    1HZ
                        memcpy(&Referee.DartRemainingTime,ReadFromUsart+DATA,Referee_LEN_dart_info);
                        break;

                    case Referee_ID_game_robot_state://0x0201   ������״̬����     10HZ
                        memcpy(&Referee.GameRobotStat,ReadFromUsart+DATA,Referee_LEN_game_robot_state);
                        judge_team_client();//�ж�һ�»����������Ķ�������� �Լ���Ӧ�Ļ�е��id�Ϳͻ���id
                        break;

                    case Referee_ID_power_heat_data://0x0202    ʵʱ������������    50HZ
                        memcpy(&Referee.PowerHeatData,ReadFromUsart+DATA,Referee_LEN_power_heat_data);
                        //��������ģ���ã�Ϊʹ����ֵ����Ƶ�ʺͲ���ϵͳ�ر��Ĺ���Ƶ��һ��
                        //�ռ�����
                        // float tmp1= 0,tmp2 = 0;
                        // for (int i = 0; i < 4; ++i) {
                        // float filtercurrent= first_Kalman_Filter(&chassis_filter[i],chassis.motor_chassis[i].motor_measure->given_current);
                        // tmp1 += chassis.motor_chassis[i].motor_measure.given_current*chassis.motor_chassis[i].motor_measure.given_current;//r0
                        // tmp2 += chassis.motor_chassis[i].motor_measure.speed_rpm*chassis.motor_chassis[i].motor_measure.given_current;//k0
                        // tmp1+=pow(filtercurrent*20/16384.0,2);
                        // tmp2+=filtercurrent*20/16384.0*chassis.motor_chassis[i].motor_measure->speed_rpm;

                        // }
                        // all_current_pingfang = tmp1*20.0/16384*20/16384;//��������ֵת���ʵ�λ/A
                        // all_rpm_mul_current = tmp2*20.0/16384;
                        // power_nihe = CHASSIS_POWER_R0*tmp1 + CHASSIS_POWER_K0*tmp2 + CHASSIS_POWER_P0;
                        // power_nihe = 0.000002623f*tmp2 + 0.0000001025f*tmp1 + 3.067f;
                        //
                        // if(power_nihe < 0)
                        //     power_nihe = 0;
                        break;

                    case Referee_ID_game_robot_pos://0x0203     ������λ������     10HZ
                        memcpy(&Referee.GameRobotPos,ReadFromUsart+DATA,Referee_LEN_game_robot_pos);
                        break;

                    case Referee_ID_buff_musk://0x0204  ��������������     1HZ
                        memcpy(&Referee.Buff,ReadFromUsart+DATA,Referee_LEN_buff_musk);
                        bit_2 = !(Referee.Buff.remaining_energy >> 2 & 1);
                        break;

                        // case Referee_ID_aerial_robot_energy://0x0205    ���л���������״̬���� 10HZ
                        //     memcpy(&Referee.AerialRobotEnergy,ReadFromUsart+DATA,Referee_LEN_aerial_robot_energy);
                        //     break;

                    case Referee_ID_robot_hurt://0x0206     �˺�״̬����  �˺���������
                        memcpy(&Referee.RobotHurt,ReadFromUsart+DATA,Referee_LEN_robot_hurt);
                        // Referee.RobotHurt.being_hurt = true;//�ܻ��ж�
                        if(Referee.RobotHurt.hurt_type == 0){
                            Referee_ID = Referee.RobotHurt.armor_id;
                            Referee_Hit_Flag = 1;
                        }
                        break;

                    case Referee_ID_shoot_data://0x0207     ʵʱ�������  �������
                        memcpy(&Referee.ShootData,ReadFromUsart+DATA,Referee_LEN_shoot_data);
                        break;

                    case Referee_ID_bullet_remaining://0x0208   ʣ�෢����   10HZ���ڷ���
                        memcpy(&Referee.BulletRemaining,ReadFromUsart+DATA,Referee_LEN_bullet_remaining);
                        break;

                    case Referee_ID_rfid_status://0x0209    ������RFID״̬��1Hz
                        memcpy(&Referee.RfidStatus,ReadFromUsart+DATA,Referee_LEN_rfid_status);
                        break;

                    case Referee_ID_dart_client_directive://0x020A  ���ڻ����˿ͻ���ָ����, 10Hz
                        memcpy(&Referee.DartClient,ReadFromUsart+DATA,Referee_LEN_dart_client_directive);
                        break;

                    case Referee_ID_dart_all_robot_position://0x020B
                        memcpy(&Referee.RobotPosition,ReadFromUsart+DATA,Referee_LEN_dart_all_robot_position);
                        break;

                    case Referee_ID_radar_mark://0x020C
                        memcpy(&Referee.RadarMark,ReadFromUsart+DATA,Referee_LEN_radar_mark);
                        break;

                    case Referee_ID_entry_info://0x020D
                        memcpy(&Referee.SentryInfo,ReadFromUsart+DATA,Referee_LEN_entry_info);
                        break;

                    case Referee_ID_radar_info://0x020E
                        memcpy(&Referee.RadarInfo,ReadFromUsart+DATA,Referee_LEN_radar_info);
                        break;

                    case Referee_ID_robot_interactive_header_data://0x0301
                        memcpy(&Referee.StudentInteractive,ReadFromUsart+DATA,Referee_LEN_robot_interactive_header_data);
                        break;

                    case Referee_ID_map_command://0x0303
                        memcpy(&Referee.MapCommand,ReadFromUsart+DATA,Referee_LEN_map_command);
                        break;

                        /* ͼ�� */
                    case Referee_ID_keyboard_information://0x0304
                        memcpy(&Referee.keyboard,ReadFromUsart+DATA,Referee_LEN_keyboard_information);
                        break;

                    case Referee_ID_robot_map_robot_data://0x0305
                        memcpy(&Referee.EnemyPosition,ReadFromUsart+DATA,Referee_LEN_robot_map_robot_data);
                        break;

                    case Referee_ID_robot_custom_client://0x0306
                        memcpy(&Referee.Custom,ReadFromUsart+DATA,Referee_LEN_robot_custom_client);
                        break;

                    case Referee_ID_robot_entry_info_receive://0x0307
                        memcpy(&Referee.SentryMapData,ReadFromUsart+DATA,Referee_LEN_robot_entry_info_receive);
                        break;

                    case Referee_ID_robot_custom_info_receive://0x0308
                        memcpy(&Referee.SendData,ReadFromUsart+DATA,Referee_LEN_robot_custom_info_receive);
                        break;

                    default:
                        break;
                }
                detect_handle(DETECT_REFEREE);
            }
        }
        if(*(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length +Referee_LEN_FRAME_TAIL) == 0xA5)
        {
            //���һ�����ݰ������˶�֡����,���ٴζ�ȡ
            Referee_read_data(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length+ Referee_LEN_FRAME_TAIL);
        }
    }
}


/**
 * �����ַ���
 * @param graphic
 * @param name
 * @param op_type
 * @param layer
 * @param color
 * @param size
 * @param length
 * @param width
 * @param start_x
 * @param start_y
 * @param character
 */
void String_Graphic(ui_string_t*clientData,
                    const char* name,
                    uint32_t op_type,
                    uint32_t layer,
                    uint32_t color,
                    uint32_t size,
                    uint32_t length,
                    uint32_t width,
                    uint32_t start_x,
                    uint32_t start_y,
                    const char *character)// ����
{
    ui_graphic_data_struct_t*data_struct=&clientData->graphic_data_struct;
    data_struct->graphic_tpye=UI_CHAR;

    for(char i=0;i<3;i++)
        data_struct->graphic_name[i] = name[i];	//�ַ�����
    data_struct->operate_tpye=op_type;// ͼ�����  1Ϊ����
    data_struct->layer=layer;//�ڵڼ�ͼ��
    data_struct->color=color;//��ɫ
    data_struct->start_angle=size;
    data_struct->end_angle=length;
    data_struct->width=width;
    data_struct->start_x=start_x;
    data_struct->start_y=start_y;
    data_struct->radius = 0;
    data_struct->end_x = 0;
    data_struct->end_y = 0;

    memcpy(clientData->data,character,30);
}

/**
 * ���Ƹ�ͼ��
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param start_angle
 * @param end_angle
 * @param width
 * @param start_x
 * @param start_y
 * @param radius
 * @param end_x
 * @param end_y
 */
void Figure_Graphic(ui_graphic_data_struct_t* graphic,//����Ҫ����ȥ����������ݶ�����
                    const char* name,
                    uint32_t operate_tpye,
                    uint32_t graphic_tpye,//����ʲôͼ��
                    uint32_t layer,
                    uint32_t color,
                    uint32_t start_angle,
                    uint32_t end_angle,
                    uint32_t width,
                    uint32_t start_x,
                    uint32_t start_y,
                    uint32_t radius,
                    uint32_t end_x,
                    uint32_t end_y)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//�ַ�����
    graphic->operate_tpye = operate_tpye; //ͼ�����
    graphic->graphic_tpye = graphic_tpye;         //Char��
    graphic->layer        = layer;//���ڵ�һ��
    graphic->color        = color;//��ɫ
    graphic->start_angle  = start_angle;
    graphic->end_angle    = end_angle;
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->radius = radius;
    graphic->end_x  = end_x;
    graphic->end_y  = end_y;
}

/**
 * ���Ƹ�����
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param size
 * @param decimal
 * @param width
 * @param start_x
 * @param start_y
 * @param number
 */
void Float_Graphic(ui_graphic_data_struct_t* graphic,//����Ҫ����ȥ����������ݶ�����
                   const char* name,
                   uint32_t operate_tpye,
                   uint32_t graphic_tpye,//����ʲôͼ��
                   uint32_t layer,
                   uint32_t color,
                   uint32_t size,
                   uint32_t decimal,
                   uint32_t width,
                   uint32_t start_x,
                   uint32_t start_y,
                   float number)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//�ַ�����
    graphic->operate_tpye = operate_tpye; //ͼ�����
    graphic->graphic_tpye = graphic_tpye;
    graphic->layer        = layer;//
    graphic->color        = color;//��ɫ
    graphic->start_angle  = size;
    graphic->end_angle    = decimal;//С����Чλ
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->number       = number*1000;//�������͵�Ҫ��1000��ת��Ϊһ��int32���͵�
}

/**
 * ��������
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param size
 * @param zero
 * @param width
 * @param start_x
 * @param start_y
 * @param number
 */
void Int_Graphic(ui_graphic_data_struct_t* graphic,//����Ҫ����ȥ����������ݶ�����
                 const char* name,
                 uint32_t operate_tpye,
                 uint32_t graphic_tpye,//����ʲôͼ��
                 uint32_t layer,
                 uint32_t color,
                 uint32_t size,
                 uint32_t zero,
                 uint32_t width,
                 uint32_t start_x,
                 uint32_t start_y,
                 int32_t number)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//�ַ�����
    graphic->operate_tpye = operate_tpye; //ͼ�����
    graphic->graphic_tpye = graphic_tpye;
    graphic->layer        = layer;//���ڵ�һ��
    graphic->color        = color;//��ɫ
    graphic->start_angle  = size;
    graphic->end_angle    = zero;
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->number       = number;
}

void Delete_All_Layer(ext_client_custom_graphic_delete_t* graphic,//����Ҫ����ȥ����������ݶ�����
                      uint32_t operate_tpye,
                      uint32_t layer)
{
    graphic->operate_type = operate_tpye;
    graphic->layer = layer;
}


//���Ʊ���
uint8_t state_first_graphic;//0~7ѭ�� ���µ�ͼ����
uint8_t ClientTxBuffer[200];//���͸��ͻ��˵����ݻ�����
uint8_t ClientTxBufferRect[200];//��̬�����ͻ�����
// uint8_t ClientTxBufferChar[200];//�����ַ��Ļ�����
// uint8_t ClientTXBufferCir[200];//��̬ԲȦ��ʾ������
// uint8_t ClientTxCapBuffer[200];//������ӻ�����
// uint8_t ClientTxPass[200];//��ͨ��������ʾ
//��������
//��0�㻭���ַ��� �ַ����ֻ�� 30 Byte
/**************************************/
/* ��̬Ԫ�أ����Ͻǰ�ɫ����ͼ�� */
void ui_fire_spin_init()
{
    ext_graphic_seven_data_t ui_fire_spin;

    //����ϵͳ֡ͷ
    ui_fire_spin.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_fire_spin.txFrameHeader.data_length = sizeof(ext_student_interactive_header_data_t) +
                                             sizeof (ui_graphic_data_struct_t) * 7;
    ui_fire_spin.txFrameHeader.seq = 0;//�����
    memcpy(ClientTxBufferRect, &ui_fire_spin.txFrameHeader, sizeof (frame_header_struct_t));//��֡ͷ����
    //CRC8У��
    append_CRC8_check_sum(ClientTxBufferRect, sizeof (frame_header_struct_t));
    ui_fire_spin.CmdID = Referee_ID_robot_interactive_header_data;
    //����֡ͷ
    ui_fire_spin.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_fire_spin.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_fire_spin.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_seven_graphic;//�����߸�ͼ��
    //�������
    //��һ����Χ��Բ��
    Figure_Graphic(&ui_fire_spin.clientData[0], "CO1", UI_ADD, UI_CIRCLE, UI_ZERO_LAYER, UI_WHITE,
                   0, 0, 1, 1430, 753, 76, 0, 0);
    //�ڶ���Ħ���ַ���
    Figure_Graphic(&ui_fire_spin.clientData[1], "RC1", UI_ADD, UI_RECTANGLE, UI_ZERO_LAYER, UI_WHITE,
                   0, 0, 1, 1386, 699, 0, 1414, 809);
    //�������̷���
    Figure_Graphic(&ui_fire_spin.clientData[2], "RC2", UI_ADD, UI_RECTANGLE, UI_ONE_LAYER, UI_WHITE,
                   0, 0, 1, 1446, 699, 0, 1474, 809);
    //�����������
    Int_Graphic(&ui_fire_spin.clientData[3], "INT1", UI_ADD, UI_INT, UI_ONE_LAYER, UI_WHITE,
                20, 0, 2, 1423, 812,7);
    Int_Graphic(&ui_fire_spin.clientData[4], "INT2", UI_ADD, UI_INT, UI_ZERO_LAYER, UI_WHITE,
                20, 0, 2, 1423, 766, 6);
    Int_Graphic(&ui_fire_spin.clientData[5], "INT3", UI_ADD, UI_INT, UI_TWO_LAYER, UI_WHITE,
                20, 0, 2, 1423, 713, 5);

    //ȥ��֡ͷ���֣��������뻺����
    memcpy(ClientTxBufferRect + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_fire_spin.CmdID, sizeof (ui_fire_spin));
    //CRC16У��
    append_CRC16_check_sum(ClientTxBufferRect, sizeof(ui_fire_spin));
    //���ڷ���
    usart6_tx_dma_enable(ClientTxBufferRect, sizeof(ui_fire_spin));
    osDelay(100);
}

/* ��̬Ԫ�أ��ұ������̶��ߺ�����90 */
void ui_line5_90_draw_init()
{
    ext_graphic_seven_data_t ui_line;
    //����ͨ��֡ͷ
    ui_line.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_line.txFrameHeader.data_length = sizeof (ext_student_interactive_header_data_t) +
                                        sizeof (ui_graphic_data_struct_t) * 7;
    ui_line.txFrameHeader.seq = 0;//���������Ϊ0
    memcpy(ClientTxBuffer, &ui_line.txFrameHeader, sizeof (frame_header_struct_t));//��֡ͷ�Ž�ȥ
    //CRC8У��֡ͷ
    append_CRC8_check_sum(ClientTxBuffer, sizeof (frame_header_struct_t));
    ui_line.CmdID = Referee_ID_robot_interactive_header_data;
    //����֡ͷ
    ui_line.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_line.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_line.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_seven_graphic;
    //�����������
    //�ұ߿̶��ߴ�������
    Figure_Graphic(&ui_line.clientData[0], &"LI1"[Referee_ID], UI_ADD, UI_ARC, UI_FOUR_LAYER, UI_WHITE,
                   49, 51, 30, 960, 539, 0, 360, 360); //��һ

    Figure_Graphic(&ui_line.clientData[1], &"LI2"[Referee_ID], UI_ADD, UI_ARC, UI_THREE_LAYER, UI_WHITE,
                   69, 71, 30, 960, 539, 0, 360, 360); //�Ҷ�

    Figure_Graphic(&ui_line.clientData[2], &"LI3"[Referee_ID], UI_ADD, UI_ARC, UI_THREE_LAYER, UI_WHITE,
                   89, 91, 30, 960, 539, 0, 360, 360); //����

    Figure_Graphic(&ui_line.clientData[3], &"LI4"[Referee_ID], UI_ADD, UI_ARC, UI_THREE_LAYER, UI_WHITE,
                   109, 111, 30, 960, 539, 0, 360, 360); //����

    Figure_Graphic(&ui_line.clientData[4], &"LI5"[Referee_ID], UI_ADD, UI_ARC, UI_THREE_LAYER, UI_WHITE,
                   129, 131, 30, 960, 539, 0, 360, 360); //����

    Int_Graphic(&ui_line.clientData[5], "INT5", UI_ADD, UI_INT, UI_THREE_LAYER, UI_WHITE,
                15, 0, 2, 1252, 549, 90);

    //ȥ��֡ͷ���֣��������뻺����
    memcpy(ClientTxBuffer + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_line.CmdID, sizeof (ui_line));
    //֡βʹ��CRC16����
    append_CRC16_check_sum(ClientTxBuffer, sizeof (ui_line));
    //����6����
    usart6_tx_dma_enable(ClientTxBuffer, sizeof (ui_line));
    osDelay(100);
}

/* ��̬Ԫ�أ���������̶��ߡ�׼�ĺͷ�ײ�� */
void ui_line3_aim_draw_init()
{
    ext_graphic_seven_data_t ui_left_line;
    //����ͨ��֡ͷ
    ui_left_line.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_left_line.txFrameHeader.data_length = sizeof (ext_student_interactive_header_data_t) +
                                             sizeof (ui_graphic_data_struct_t) * 7;
    ui_left_line.txFrameHeader.seq = 0;//���������Ϊ0
    memcpy(ClientTxBuffer, &ui_left_line.txFrameHeader, sizeof (frame_header_struct_t));//��֡ͷ�Ž�ȥ
    //CRC8У��֡ͷ
    append_CRC8_check_sum(ClientTxBuffer, sizeof (frame_header_struct_t));
    ui_left_line.CmdID = Referee_ID_robot_interactive_header_data;
    //����֡ͷ
    ui_left_line.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_left_line.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_left_line.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_seven_graphic;
    //��ߵ�һ���̶���
    Figure_Graphic(&ui_left_line.clientData[0], "LI1", UI_ADD, UI_LINE, UI_ZERO_LAYER, UI_WHITE,
                   0, 0, 10, 666, 782, 0, 691, 763);
    //��ߵڶ����̶���
    Figure_Graphic(&ui_left_line.clientData[1], "LI2", UI_ADD, UI_LINE, UI_ZERO_LAYER, UI_WHITE,
                   0, 0, 35, 598, 537, 0, 598, 546);
    //��ߵ������̶���
    Figure_Graphic(&ui_left_line.clientData[2], "LI3", UI_ADD,  UI_LINE, UI_ZERO_LAYER, UI_WHITE,
                   0, 0, 10, 677, 285, 0, 702, 306);
    //׼��
    // Figure_Graphic(&ui_left_line.clientData[3], "LI4", UI_ADD, UI_CIRCLE, UI_ZERO_LAYER, UI_WHITE,
    //                0, 0, 2, 940, 380, 40, 0, 0); //�м�Բ��
    //��߷�ײ��
    Figure_Graphic(&ui_left_line.clientData[3], "LI4", UI_ADD,  UI_LINE, UI_ZERO_LAYER, UI_ORANGE,
                   0, 0, 2, 693, 0, 0, 837, 313); //-10 -8 //-20
    //�ұ߷�ײ��
    Figure_Graphic(&ui_left_line.clientData[4], "LI5", UI_ADD,  UI_LINE, UI_ZERO_LAYER, UI_ORANGE,
                   0, 0, 2, 1225, 0, 0, 1064, 313);

    Figure_Graphic(&ui_left_line.clientData[5], "LI6", UI_ADD,  UI_LINE, UI_ZERO_LAYER, UI_CYAN_BLUE,
                   0, 0, 2, 920, 300, 0, 960, 300);

    Figure_Graphic(&ui_left_line.clientData[6], "LI7", UI_ADD,  UI_LINE, UI_ZERO_LAYER, UI_CYAN_BLUE,
                   0, 0, 2, 920, 400, 0, 960, 400);


    //�ѳ�ȥ֡ͷ���������ַŽ�������
    memcpy(ClientTxBuffer + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_left_line.CmdID, sizeof (ui_left_line));
    //֡βʹ��CRC16����
    append_CRC16_check_sum(ClientTxBuffer, sizeof (ui_left_line));
    //����6����
    usart6_tx_dma_enable(ClientTxBuffer, sizeof (ui_left_line));
    osDelay(100);
}

/* ��̬Ԫ�أ��м�ʮ��׼�� */
void ui_aim_draw_init ()
{
    ext_graphic_seven_data_t ui_aim;
    //����ͨ��֡ͷ
    ui_aim.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_aim.txFrameHeader.data_length = sizeof (ext_student_interactive_header_data_t) +
                                       sizeof (ui_graphic_data_struct_t) * 7;
    ui_aim.txFrameHeader.seq = 0;//���������Ϊ0
    memcpy(ClientTxBuffer, &ui_aim.txFrameHeader, sizeof (frame_header_struct_t));//��֡ͷ�Ž�ȥ
    //CRC8У��֡ͷ
    append_CRC8_check_sum(ClientTxBuffer, sizeof (frame_header_struct_t));
    ui_aim.CmdID = Referee_ID_robot_interactive_header_data;
    //����֡ͷ
    ui_aim.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_aim.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_aim.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_seven_graphic;
    //�����������
    Figure_Graphic(&ui_aim.clientData[0], "LI1", UI_ADD, UI_CIRCLE, UI_ZERO_LAYER, UI_WHITE,
                   0, 0, 1, 959, 540, 22, 0, 0); //�м�Բ��

    Figure_Graphic(&ui_aim.clientData[1], "LI2", UI_ADD, UI_LINE, UI_ZERO_LAYER, UI_WHITE,
                   0, 0, 1, 696, 539, 0, 1225, 539); //�������µ�һ������

    Figure_Graphic(&ui_aim.clientData[2], "LI3", UI_ADD, UI_LINE, UI_ONE_LAYER, UI_WHITE,
                   0, 0, 1, 960, 320, 0, 960, 640); //����

    Figure_Graphic(&ui_aim.clientData[3], "LI4", UI_ADD, UI_LINE, UI_ONE_LAYER, UI_WHITE,
                   0, 0, 1, 915, 505, 0, 1006, 505); //�������µڶ�������

    Figure_Graphic(&ui_aim.clientData[4], "LI5", UI_ADD,  UI_LINE, UI_ONE_LAYER, UI_WHITE,
                   0, 0, 1, 880, 478, 0, 1043, 478); //�������µ���������

    Figure_Graphic(&ui_aim.clientData[5], "LI5", UI_ADD, UI_LINE, UI_TWO_LAYER, UI_WHITE,
                   0, 0, 1, 845, 451, 0, 1077, 451); //�������µ���������
    Figure_Graphic(&ui_aim.clientData[6], "LI6", UI_ADD, UI_LINE, UI_ONE_LAYER, UI_WHITE,
                   0, 0, 1, 810, 424, 0, 1114, 424); //�������µ���������

    //�ѳ�ȥ֡ͷ���������ַŽ�������
    memcpy(ClientTxBuffer + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_aim.CmdID, sizeof (ui_aim));
    //֡βʹ��CRC16����
    append_CRC16_check_sum(ClientTxBuffer, sizeof (ui_aim));
    //����6����
    usart6_tx_dma_enable(ClientTxBuffer, sizeof (ui_aim));
    osDelay(100);
}

/* 6����̬Ԫ�س�ʼ������ */
void ui_draw_change_init()
{
    ext_graphic_seven_data_t ui_change;
    //����ͨ��֡ͷ
    ui_change.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_change.txFrameHeader.data_length = sizeof (ext_student_interactive_header_data_t) +
                                          sizeof (ui_graphic_data_struct_t) * 7;
    ui_change.txFrameHeader.seq = 0;//���������Ϊ0
    memcpy(ClientTxBuffer, &ui_change.txFrameHeader, sizeof (frame_header_struct_t));//��֡ͷ�Ž�ȥ
    //CRC8У��֡ͷ
    append_CRC8_check_sum(ClientTxBuffer, sizeof (frame_header_struct_t));
    ui_change.CmdID = Referee_ID_robot_interactive_header_data;
    //����֡ͷ
    ui_change.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_change.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_change.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_seven_graphic;
    //�����������
    //���Ͻ�С����Բ��
    Figure_Graphic(&ui_change.clientData[0], &"SPI"[Referee_ID], UI_ADD, UI_ARC, UI_ONE_LAYER, UI_CYAN_BLUE,
                   30, 330, 25, 1430, 753, 0, 87, 87);

    //���Ͻ�Ħ��������ͼ
    Figure_Graphic(&ui_change.clientData[1], "FI1", UI_ADD, UI_LINE, UI_ONE_LAYER, UI_CYAN_BLUE,
                   0, 0, 23, 1400, 699, 0, 1400, 757);

    //�м��ɫ�̶���
    Figure_Graphic(&ui_change.clientData[2], &"LIO"[Referee_ID], UI_ADD, UI_ARC, UI_ZERO_LAYER, UI_CYAN_BLUE,
                   89, 91, 45, 960, 539, 0, 360, 360);

    //���·���ɫ����״̬Բ��
    Figure_Graphic(&ui_change.clientData[3], &"aut"[Referee_ID], UI_ADD, UI_ARC, UI_TWO_LAYER, UI_CYAN_BLUE,
                   280, 300, 15, 960, 539, 0, 363, 363);

    //���½ǵ���Բ��
    Figure_Graphic(&ui_change.clientData[4], &"ca1"[Referee_ID], UI_ADD, UI_ARC, UI_ONE_LAYER, UI_CYAN_BLUE,
                   229, 269, 15, 960, 539, 0, 363, 363); //238 259

    //���Ͻǲ�������ͼ
    Figure_Graphic(&ui_change.clientData[5], "FI2", UI_ADD, UI_LINE, UI_ONE_LAYER, UI_CYAN_BLUE,
                   0, 0, 23, 1460, 699, 0, 1460, 757);

    Int_Graphic(&ui_change.clientData[6], "SHOOT", UI_ADD, UI_INT, UI_THREE_LAYER, UI_PINK,
                20, 0, 2, 1020, 480,0);

    //�ѳ�ȥ֡ͷ���������ַŽ�������
    memcpy(ClientTxBuffer + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_change.CmdID, sizeof (ui_change));
    //֡βʹ��CRC16����
    append_CRC16_check_sum(ClientTxBuffer, sizeof (ui_change));
    //����6����
    usart6_tx_dma_enable(ClientTxBuffer, sizeof (ui_change));
    osDelay(100);
}

void dynamic_change_delete()
{
    deleteLayer_data_t ui_change;
    //����ͨ��֡ͷ
    ui_change.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_change.txFrameHeader.data_length = sizeof (ext_student_interactive_header_data_t) +
                                          sizeof (ext_client_custom_graphic_delete_t) * 1;
    ui_change.txFrameHeader.seq = 0;//���������Ϊ0
    memcpy(ClientTxBuffer, &ui_change.txFrameHeader, sizeof (frame_header_struct_t));//��֡ͷ�Ž�ȥ
    //CRC8У��֡ͷ
    append_CRC8_check_sum(ClientTxBuffer, sizeof (frame_header_struct_t));
    ui_change.CmdID = Referee_ID_robot_interactive_header_data;
    //����֡ͷ
    ui_change.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_change.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_change.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_delete_graphic;

    Delete_All_Layer(&ui_change.clientData, UI_ALL_delete, UI_ONE_LAYER );

    //�ѳ�ȥ֡ͷ���������ַŽ�������
    memcpy(ClientTxBuffer + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_change.CmdID, sizeof (ui_change));
    //֡βʹ��CRC16����
    append_CRC16_check_sum(ClientTxBuffer, sizeof (ui_change));
    //����6����
    usart6_tx_dma_enable(ClientTxBuffer, sizeof (ui_change));
    osDelay(100);
}

/*��̬Ԫ�أ���ɫת��*/
void dynamic_color_draw()
{
    if(gimbal.gimbal_ctrl_mode == GIMBAL_AUTO )//����ģʽ��ui���ɫ
    {
        uiColor.auto_aim_color = UI_YELLOW;
    }
    else
    {
        uiColor.auto_aim_color = UI_CYAN_BLUE;//������ʱ����ɫ
    }

//    if(chassis.mode == CHASSIS_SPIN_R || chassis.mode == CHASSIS_SPIN_L)//С����ģʽ��ui���ɫ
//    {
//        uiColor.spin_color = UI_YELLOW;
//    }
//    else
//    {
//        uiColor.spin_color = UI_CYAN_BLUE;//������ʱ����ɫ
//    }

    if(launcher.fir_wheel_mode == Fire_ON)
    {
        uiColor.fire_color = UI_YELLOW;//Ħ��������,ui���ɫ
    }
    else
    {
        uiColor.fire_color = UI_CYAN_BLUE;//������ʱ����ɫ
    }

    if(launcher.trigger_mode == SHOOT_FAIL){
        uiColor.shoot_color = UI_PINK; //����ת��,ui���ɫ
    }
    else
    {
        uiColor.shoot_color = UI_CYAN_BLUE;//��ת��ʱ����ɫ
    }

}

/* 6����̬Ԫ�ص�UI���� */
void dynamic_change_draw()
{
    ext_graphic_seven_data_t ui_change;
    //����ͨ��֡ͷ
    ui_change.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_change.txFrameHeader.data_length = sizeof (ext_student_interactive_header_data_t) +
                                          sizeof (ui_graphic_data_struct_t) * 7;
    ui_change.txFrameHeader.seq = 0;//���������Ϊ0
    memcpy(ClientTxBuffer, &ui_change.txFrameHeader, sizeof (frame_header_struct_t));//��֡ͷ�Ž�ȥ
    //CRC8У��֡ͷ
    append_CRC8_check_sum(ClientTxBuffer, sizeof (frame_header_struct_t));
    ui_change.CmdID = Referee_ID_robot_interactive_header_data;
    //����֡ͷ
    ui_change.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_change.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_change.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_seven_graphic;
    //�����������
    //С������ɫԲ��
    Figure_Graphic(&ui_change.clientData[0], &"SPI"[Referee_ID], UI_MODIFY, UI_ARC, UI_ONE_LAYER, uiColor.spin_color,
                   uiChange.spin_startangle, uiChange.spin_endangle, 25, 1430, 753, 0, 87, 87);

    //Ħ��������ͼ
    Figure_Graphic(&ui_change.clientData[1], "FI1", UI_MODIFY, UI_LINE, UI_ONE_LAYER, uiColor.fire_color,
                   0, 0, 23, 1400, 699, 0, 1400,757 );

    //��ɫ�̶���
    Figure_Graphic(&ui_change.clientData[2], &"LIO"[Referee_ID], UI_MODIFY, UI_ARC, UI_ZERO_LAYER, UI_ORANGE,
                   -gimbal.pitch.absolute_angle_get+90-1, -gimbal.pitch.absolute_angle_get+90+1, 45, 960, 539, 0, 360, 360);

    //����״̬Բ��
    Figure_Graphic(&ui_change.clientData[3], &"aut"[Referee_ID], UI_MODIFY, UI_ARC, UI_TWO_LAYER, uiColor.auto_aim_color,
                   280, 300, 15, 960, 539, 0, 363, 363);

    //����Բ��
    Figure_Graphic(&ui_change.clientData[4], &"ca1"[Referee_ID], UI_MODIFY, UI_ARC, UI_ONE_LAYER, uiColor.cap_color,
                   229, uiChange.cap_endangle, 15, 960, 539, 0, 363, 363);

    // Figure_Graphic(&ui_change.clientData[4], &"ca1"[Referee_ID], UI_MODIFY, UI_ARC, UI_ONE_LAYER, UI_ORANGE,
    //                229, 269, 15, 960, 539, 0, 363, 363);

    //���Ͻǲ�������ͼ
    Figure_Graphic(&ui_change.clientData[5], "FI2", UI_MODIFY, UI_LINE, UI_ONE_LAYER, uiColor.shoot_color,
                   0, 0, 23, 1460, 699, 0, 1460, 757);

    Int_Graphic(&ui_change.clientData[6], "SHOOT", UI_MODIFY, UI_INT, UI_THREE_LAYER, UI_PINK,
                20, 0, 2, 1020, 480,Referee.PowerHeatData.shooter_42mm_barrel_heat);

    //�ѳ�ȥ֡ͷ���������ַŽ�������
    memcpy(ClientTxBuffer + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_change.CmdID, sizeof (ui_change));
    //֡βʹ��CRC16����
    append_CRC16_check_sum(ClientTxBuffer, sizeof (ui_change));
    //����6����
    usart6_tx_dma_enable(ClientTxBuffer, sizeof (ui_change));
    osDelay(100);
}

void UI_Paint_task(void const*argument){
    vTaskDelay(20);
    int flag = 0;
    while (1)
    {
//        if(chassis.mode == CHASSIS_RELAX)
//        {
//            dynamic_change_delete();
//            flag = 0;
//        }
//        else
//        {
//            if(flag == 0) {
//                ui_fire_spin_init();
//                ui_line5_90_draw_init();
//                ui_line3_aim_draw_init();
//                ui_draw_change_init();
//                flag = 1;
//            }else {
//                dynamic_color_draw();
////                dynamic_chassis_draw();
//                dynamic_cap_draw();
//                dynamic_change_draw();
//            }
//        }
    }
}