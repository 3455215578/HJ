//
// Created by xhuanc on 2021/10/11.
//

//#include

#include "remote.h"

#include "main.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "user_lib.h"
//define
#define RC_CHANNAL_ERROR_VALUE 700


//变量
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

RC_ctrl_t rc_ctrl;

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

int16_t last_last_rc_s[2];


//函数定义及声明 声明靠前

/**
 * 取正函数
 */


static int16_t RC_abs(int16_t value);

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//在main函数中调用此函数
void remote_control_init(void) {
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}


uint8_t RC_data_is_error(void) {
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0) {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0) {
        goto error;
    }
    return 0;

    error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void) {
    RC_restart(SBUS_RX_BUF_NUM);
}

void slove_data_error(void) {
    RC_restart(SBUS_RX_BUF_NUM);
}

//串口中断
void USART3_IRQHandler(void) {
    if (huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    } else if (USART3->SR & UART_FLAG_IDLE) {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH) {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                //记录数据接收时间
            }
        } else {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH) {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                //记录数据接收时间
//                detect_hook(DBUS_TOE);
//                sbus_to_usart1(sbus_rx_buf[1]);
            }
        }
    }
}

//取正函数
static int16_t RC_abs(int16_t value) {
    if (value > 0) {
        return value;
    } else {
        return -value;
    }
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl) {
    static char rc_temp_L_s[2] = {'0', '0'};  //遥控器拨杆数值缓冲区，用于防止C板接收发癫,'0'为空
    static char rc_temp_R_s[2] = {'0', '0'};

    if (sbus_buf == NULL || rc_ctrl == NULL) {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3

//    //START
//    //遥控器拨杆错误检测，若缓冲区连续两次的值相同则OK，不一样则清除缓冲区上一次的值
//    rc_temp_L_s[1] = rc_temp_L_s[0];
//    rc_temp_L_s[0] = ((sbus_buf[5] >> 4) & 0x0003);
//    if((rc_temp_L_s[0] == rc_temp_L_s[1]) && rc_temp_L_s[1] != '0') {
//        rc_ctrl->rc.s[0] = rc_temp_L_s[1];
//    }else{
//        rc_temp_L_s[1] = '0';
//    }
//
//    rc_temp_R_s[1] = rc_temp_R_s[0];
//    rc_temp_R_s[0] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
//    if((rc_temp_R_s[0] == rc_temp_R_s[1]) && rc_temp_R_s[1] != '0') {
//        rc_ctrl->rc.s[1] = rc_temp_R_s[1];
//    }else{
//        rc_temp_R_s[1] = '0';
//    }
//    //END

    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right

    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    RC_data_is_error();
    if (rc_ctrl->rc.ch[2] <= 10 && rc_ctrl->rc.ch[2] >= -10) {
        rc_ctrl->rc.ch[2] = 0;
    }

    //摇杆错误检测，若摇杆一次变化量超过500，即产生错误，将本次的值回归上次的值
    for (int i = 0; i < 4; ++i) {
        if (abs((rc_ctrl->rc.last_ch[i] - rc_ctrl->rc.ch[i])) > 500)
            rc_ctrl->rc.ch[i] = rc_ctrl->rc.last_ch[i];
        else
            rc_ctrl->rc.last_ch[i] = rc_ctrl->rc.ch[i];
    }
    detect_handle(DETECT_REMOTE);
//    SEGGER_RTT_printf(0,"%d ,",rc_ctrl->rc.s[0]);
}



