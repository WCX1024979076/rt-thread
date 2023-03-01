/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022/12/25     flyingcys    first version
 * 2023/02/21    wcx1024979076 use bl_mcu_sdk
*/

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "board.h"
#include "drv_uart.h"

struct device_uart
{
    struct rt_serial_device serial;
    uint8_t port;
    uint8_t tx_pin;
    uint8_t rx_pin;
    char* name;
};

static bl_uart_notify_t g_uart_notify_arg[UART_NUMBER_SUPPORTED];

static void _uart_rx_irq(void *param)
{
    struct device_uart *uart = (struct device_uart *)param;;

    struct rt_serial_device *serial = &uart->serial;

    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
}


int _bl_uart_int_rx_notify_register(uint8_t id, cb_uart_notify_t cb, void *arg)
{
    if (!(id < UART_NUMBER_SUPPORTED)) {
        /*UART ID overflow*/
        return -1;
    }
    g_uart_notify_arg[id].rx_cb = cb;
    g_uart_notify_arg[id].rx_cb_arg = arg;
    return 0;
}

int _bl_uart_int_tx_notify_register(uint8_t id, cb_uart_notify_t cb, void *arg)
{
    if (!(id < UART_NUMBER_SUPPORTED)) {
        /*UART ID overflow*/
        return -1;
    }
    g_uart_notify_arg[id].tx_cb = cb;
    g_uart_notify_arg[id].tx_cb_arg = arg;
    return 0;
}

int _bl_uart_int_rx_notify_unregister(uint8_t id, cb_uart_notify_t cb, void *arg)
{
    if (!(id < UART_NUMBER_SUPPORTED)) {
        /*UART ID overflow*/
        return -1;
    }
    g_uart_notify_arg[id].rx_cb = NULL;
    g_uart_notify_arg[id].rx_cb_arg = NULL;
    return 0;
}

int _bl_uart_int_tx_notify_unregister(uint8_t id, cb_uart_notify_t cb, void *arg)
{
    if (!(id < UART_NUMBER_SUPPORTED)) {
        /*UART ID overflow*/
        return -1;
    }
    g_uart_notify_arg[id].tx_cb = NULL;
    g_uart_notify_arg[id].tx_cb_arg = NULL;
    return 0;
}

static inline void _uart_generic_notify_handler(char* name, uint32_t id)
{
    cb_uart_notify_t cb;
    void *arg;
    struct bflb_device_s *uart;
    uart = bflb_device_get_by_name(name);
    uint32_t int_status = bflb_uart_get_intstatus(uart);
    /* Length of uart tx data transfer arrived interrupt */
    if(int_status & UART_INTSTS_TX_END) {
        bflb_uart_int_clear(uart, UART_INTCLR_TX_END);
    }

    /* Length of uart rx data transfer arrived interrupt */
    if(int_status & UART_INTSTS_RX_END) {
        bflb_uart_int_clear(uart, UART_INTCLR_RX_END);

        /*Receive Data ready*/
        cb = g_uart_notify_arg[id].rx_cb;
        arg = g_uart_notify_arg[id].rx_cb_arg;

        if (cb) {
            /*notify up layer*/
            cb(arg);
        }
    }

    /* Tx fifo ready interrupt,auto-cleared when data is pushed */
    if(int_status & UART_INTSTS_TX_FIFO) {
        /* Transmit data request interrupt */
        cb = g_uart_notify_arg[id].tx_cb;
        arg = g_uart_notify_arg[id].tx_cb_arg;

        if (cb) {
            /*notify up layer*/
            cb(arg);
        }
    }

    /* Rx fifo ready interrupt,auto-cleared when data is popped */
    if(int_status & UART_INTSTS_RX_FIFO) {
        /*Receive Data ready*/

        cb = g_uart_notify_arg[id].rx_cb;
        arg = g_uart_notify_arg[id].rx_cb_arg;

        if (cb) {
            /*notify up layer*/
            cb(arg);
        }
    }

    /* Rx time-out interrupt */
    if (int_status & UART_INTSTS_RTO) {
        bflb_uart_int_clear(uart, UART_INTCLR_RTO);

        /*Receive Data ready*/
        cb = g_uart_notify_arg[id].rx_cb;
        arg = g_uart_notify_arg[id].rx_cb_arg;

        if (cb) {
            /*notify up layer*/
            cb(arg);
        }
    }

    /* Rx parity check error interrupt */
    if(int_status & UART_INTSTS_PCE) {
        bflb_uart_int_clear(uart, UART_INTCLR_PCE);
    }

    /* Tx fifo overflow/underflow error interrupt */
    if(int_status & UART_INTSTS_TX_FER) {
    }

    /* Rx fifo overflow/underflow error interrupt */
    if(int_status & UART_INTSTS_RX_FER) {
    }

    return;
}

void UART0_IRQHandler(void)
{
    _uart_generic_notify_handler("uart0", 0);
}

void UART1_IRQHandler(void)
{
    _uart_generic_notify_handler("uart1", 1);
}

void UART2_IRQHandler(void)
{
    _uart_generic_notify_handler("uart2", 2);
}

void UART3_IRQHandler(void)
{
    _uart_generic_notify_handler("uart3", 3);
}

static rt_err_t _uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    struct bflb_device_s *bflb_uart;
    struct bflb_device_s *gpio;

    gpio = bflb_device_get_by_name("gpio");
#if defined(CPU_M0)
    bflb_gpio_uart_init(gpio, BSP_UART0_TXD_PIN, GPIO_UART_FUNC_UART0_TX);
    bflb_gpio_uart_init(gpio, BSP_UART0_RXD_PIN, GPIO_UART_FUNC_UART0_RX);
#elif defined(CPU_D0)
    /* sipeed m1s dock */
    bflb_gpio_init(gpio, GPIO_PIN_16, 21 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_17, 21 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
#endif
    struct bflb_uart_config_s uart_cfg;
    uart_cfg.data_bits = UART_DATA_BITS_8;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.parity = UART_PARITY_NONE;
    uart_cfg.flow_ctrl = 0;
    uart_cfg.tx_fifo_threshold = 16;
    uart_cfg.rx_fifo_threshold = 16;

    uart_cfg.baudrate = cfg->baud_rate;

    switch (cfg->data_bits)
    {
        case DATA_BITS_5:
            uart_cfg.data_bits = UART_DATA_BITS_5;
            break;

        case DATA_BITS_6:
            uart_cfg.data_bits = UART_DATA_BITS_6;
            break;

        case DATA_BITS_7:
            uart_cfg.data_bits = UART_DATA_BITS_7;
            break;

        case DATA_BITS_8:
            uart_cfg.data_bits = UART_DATA_BITS_8;
            break;

        default:
            uart_cfg.data_bits = UART_DATA_BITS_8;
            break;
    }

    switch (cfg->stop_bits)
    {
        case STOP_BITS_1:
            uart_cfg.stop_bits = UART_STOP_BITS_1;
            break;

        case STOP_BITS_2:
            uart_cfg.stop_bits = UART_STOP_BITS_2;
            break;

        default:
            uart_cfg.stop_bits = UART_STOP_BITS_1;
            break;
    }

    switch (cfg->parity)
    {
        case PARITY_NONE:
            uart_cfg.parity = UART_PARITY_NONE;
            break;

        case PARITY_ODD:
            uart_cfg.parity = UART_PARITY_ODD;
            break;

        case PARITY_EVEN:
            uart_cfg.parity = UART_PARITY_EVEN;
            break;

        default:
            uart_cfg.parity = UART_PARITY_NONE;
            break;
    }

    struct device_uart *uart;

    uart = serial->parent.user_data;
    RT_ASSERT(uart != RT_NULL);
    bflb_uart = bflb_device_get_by_name(uart->name);

    bflb_uart_init(bflb_uart, &uart_cfg);

    return RT_EOK;
}

int _bl_uart_int_disable(uint8_t id)
{
    struct bflb_device_s *uart;
    switch (id) {
        case 0:
        {
            uart = bflb_device_get_by_name("uart0");
            bflb_uart_txint_mask(uart, false);
            bflb_uart_rxint_mask(uart, false);
            bflb_uart_feature_control(uart, UART_CMD_SET_RX_END_INTERRUPT, false);
            bflb_irq_detach(UART0_IRQn);
            bflb_irq_disable(UART0_IRQn);
        }
        break;
        case 1:
        {
            uart = bflb_device_get_by_name("uart1");
            bflb_uart_txint_mask(uart, false);
            bflb_uart_rxint_mask(uart, false);
            bflb_uart_feature_control(uart, UART_CMD_SET_RX_END_INTERRUPT, false);
            bflb_irq_detach(UART1_IRQn);
            bflb_irq_disable(UART1_IRQn);
        }
        break;
        case 2:
        {
            uart = bflb_device_get_by_name("uart2");
            bflb_uart_txint_mask(uart, false);
            bflb_uart_rxint_mask(uart, false);
            bflb_uart_feature_control(uart, UART_CMD_SET_RX_END_INTERRUPT, false);
            bflb_irq_detach(UART2_IRQn);
            bflb_irq_disable(UART2_IRQn);
        }
        break;
        case 3:
        {
            uart = bflb_device_get_by_name("uart3");
            bflb_uart_txint_mask(uart, false);
            bflb_uart_rxint_mask(uart, false);
            bflb_uart_feature_control(uart, UART_CMD_SET_RX_END_INTERRUPT, false);
            bflb_irq_detach(UART3_IRQn);
            bflb_irq_disable(UART3_IRQn);
        }
        break;
        default:
        {
            return -1;
        }
    }

    return 0;
}

int _bl_uart_int_enable(uint8_t id)
{
    struct bflb_device_s *uart;

    switch (id) {
        case 0:
        {
            uart = bflb_device_get_by_name("uart0");
            bflb_uart_rxint_mask(uart, true);
            bflb_uart_feature_control(uart, UART_CMD_SET_RX_END_INTERRUPT, true);
            bflb_irq_attach(UART0_IRQn, UART0_IRQHandler, NULL);
            bflb_irq_enable(UART0_IRQn);
        }
        break;
        case 1:
        {
            uart = bflb_device_get_by_name("uart1");
            bflb_uart_rxint_mask(uart, true);
            bflb_uart_feature_control(uart, UART_CMD_SET_RX_END_INTERRUPT, true);
            bflb_irq_attach(UART1_IRQn, UART1_IRQHandler, NULL);
            bflb_irq_enable(UART1_IRQn);
        }
        break;
        case 2:
        {
            uart = bflb_device_get_by_name("uart2");
            bflb_uart_rxint_mask(uart, true);
            bflb_uart_feature_control(uart, UART_CMD_SET_RX_END_INTERRUPT, true);
            bflb_irq_attach(UART2_IRQn, UART2_IRQHandler, NULL);
            bflb_irq_enable(UART2_IRQn);
        }
        break;
        case 3:
        {
            uart = bflb_device_get_by_name("uart3");
            bflb_uart_rxint_mask(uart, true);
            bflb_uart_feature_control(uart, UART_CMD_SET_RX_END_INTERRUPT, true);
            bflb_irq_attach(UART3_IRQn, UART3_IRQHandler, NULL);
            bflb_irq_enable(UART3_IRQn);
        }
        break;
        default:
        {
            return -1;
        }
    }

    return 0;
}

static rt_err_t _uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct device_uart *uart;

    RT_ASSERT(serial != RT_NULL);

    uart = serial->parent.user_data;
    RT_ASSERT(uart != RT_NULL);

    switch (cmd)
    {
    /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        _bl_uart_int_disable(uart->port);
        _bl_uart_int_rx_notify_unregister(uart->port, _uart_rx_irq, uart);
        break;

    /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        _bl_uart_int_rx_notify_register(uart->port, _uart_rx_irq, uart);
        _bl_uart_int_enable(uart->port);
        break;
    }
    return RT_EOK;
}

static int _uart_putc(struct rt_serial_device *serial, char c)
{
    struct device_uart *uart;
    struct bflb_device_s *bflb_uart;

    RT_ASSERT(serial != RT_NULL);

    uart = serial->parent.user_data;
    bflb_uart = bflb_device_get_by_name(uart->name);
    RT_ASSERT(uart != RT_NULL);
    bflb_uart_putchar(bflb_uart, c);

    return 1;
}

static int _uart_getc(struct rt_serial_device *serial)
{
    int ch = -1;
    struct device_uart *uart;
    struct bflb_device_s *bflb_uart;

    RT_ASSERT(serial != RT_NULL);
    uart = serial->parent.user_data;
    RT_ASSERT(uart != RT_NULL);
    bflb_uart = bflb_device_get_by_name(uart->name);
    ch = bflb_uart_getchar(bflb_uart);

    return ch;
}

static const struct rt_uart_ops _uart_ops =
{
    .configure = _uart_configure,
    .control = _uart_control,
    .putc = _uart_putc,
    .getc = _uart_getc,
    .dma_transmit = RT_NULL
};

/*
 * UART Initiation
 */
int rt_hw_uart_init(void)
{
    rt_err_t result = 0;

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    struct rt_serial_device *serial;
    struct device_uart      *uart;

#ifdef BSP_USING_UART0
    static struct device_uart uart0;

    serial  = &uart0.serial;
    uart    = &uart0;

    serial->ops              = &_uart_ops;
    serial->config           = config;
    serial->config.baud_rate = 2000000;

    uart->port = 0;
    uart->tx_pin = BSP_UART0_TXD_PIN;
    uart->rx_pin = BSP_UART0_RXD_PIN;

    #if defined(CPU_M0)
        uart->name = "uart0";
    #elif defined(CPU_D0)
        uart->name = "uart3";
    #endif

    /* register USART device */
    result = rt_hw_serial_register(serial,
                                    uart->name,
                                    RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                                    uart);
    RT_ASSERT(result == RT_EOK);
#endif

    return 0;
}
