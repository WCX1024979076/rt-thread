/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022/12/25     flyingcys    first version
 */

#ifndef __DRV_USART_H__
#define __DRV_USART_H__

#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include "bflb_uart.h"
// #include "bl_uart.h"
// #include "bl808_uart.h"

// #include "uart_config.h"
#include "hardware/uart_reg.h"

int rt_hw_uart_init(void);
typedef void (*cb_uart_notify_t)(void *arg);

typedef struct bl_uart_notify {
    cb_uart_notify_t rx_cb;
    void            *rx_cb_arg;

    cb_uart_notify_t tx_cb;
    void            *tx_cb_arg;
} bl_uart_notify_t;

#define UART_NUMBER_SUPPORTED   4

#endif  /* __DRV_USART_H__ */
