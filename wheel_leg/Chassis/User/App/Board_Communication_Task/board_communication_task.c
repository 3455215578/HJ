/*ң������λ��

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

void Gimbal_to_Chassis_Can(uint32_t can_id, const uint8_t *rx_data) {
    switch (can_id) {
        case 0x110:
        {
            union I16 vx_channel; // ǰ��
            union I16 yaw_channel; // ����


            {
                /** ����ǰ��ͨ�� **/
                vx_channel.data[0] = rx_data[0];
                vx_channel.data[1] = rx_data[1];

                /** ����ת��ͨ�� **/
                yaw_channel.data[0] = rx_data[2];
                yaw_channel.data[1] = rx_data[3];

                /** �������Ҳ�ť **/
                rc_ctrl.rc.s[RC_s_L] = rx_data[4];
                rc_ctrl.rc.s[RC_s_R] = rx_data[5];
            }

            rc_ctrl.rc.ch[CHASSIS_VX_CHANNEL] = vx_channel.value;
            rc_ctrl.rc.ch[CHASSIS_YAW_CHANNEL] = yaw_channel.value;


        } break;


        default:
            break;
    }
}


