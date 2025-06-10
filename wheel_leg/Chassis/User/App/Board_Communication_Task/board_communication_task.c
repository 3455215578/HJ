/*Ò£¿ØÆ÷¼üÎ»£º

        S1                                                                         S0
                    |                                                   |
                    |                                                   |
                    |                                                   |
                    |                                                   |
           ------------------ 2                                ------------------ 0
                    |                                                   |
                    |                                                   |
                    |                                                   |
                    3                                                   1

*/



#include "board_communication_task.h"
#include "remote.h"
#include "robot_def.h"

//void board_info_update(void)
//{
//    case 0x110:{
//        union I16 ch1;
//        union I16 ch2;
//        {
//            ch1.data[0] = rx_data[0];
//            ch1.data[1] = rx_data[1];
//            ch2.data[0] = rx_data[2];
//            ch2.data[1] = rx_data[3];
//            rc_ctrl.rc.s[RC_s_L] = rx_data[4];
//            rc_ctrl.rc.s[RC_s_R] = rx_data[5];
//        }
//        rc_ctrl.rc.ch[1] = ch1.value;
//        rc_ctrl.rc.ch[2] = ch2.value;
//    } break;
//
//}

union I16{
    uint8_t data[2];
    int16_t value;
};

void Gimbal_to_Chassis_Can(uint32_t can_id, const uint8_t *rx_data) {
    switch (can_id) {
        case 0x110:
        {
            union I16 ch0; // ×óÓÒ
            union I16 ch1; // Ç°ºó

            {
                /** ½âÎöChannel 0 **/
                ch0.data[0] = rx_data[0];
                ch0.data[1] = rx_data[1];

                /** ½âÎöChannel 1 **/
                ch1.data[0] = rx_data[2];
                ch1.data[1] = rx_data[3];

                /** ½âÎö×óÓÒ²¦Å¥ **/
                rc_ctrl.rc.s[RC_s_L] = rx_data[4];
                rc_ctrl.rc.s[RC_s_R] = rx_data[5];
            }

            rc_ctrl.rc.ch[CHASSIS_YAW_CHANNEL] = ch0.value;
            rc_ctrl.rc.ch[CHASSIS_SPEED_CHANNEL] = ch1.value;

        } break;


        default:
            break;
    }
}


