//
// Created by Shockley on 2022/11/9.
//

#ifndef PROJECT_USB_TASK_H
#define PROJECT_USB_TASK_H

#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "CRC8_CRC16.h"
#include "packet.h"
#include "fifo.h"
#include "protocol_balance.h"
#include <stdio.h>
#include <stdarg.h>

_Noreturn void USB_task(void const * argument);

#endif //PROJECT_USB_TASK_H
