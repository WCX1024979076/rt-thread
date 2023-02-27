/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2022/12/25     flyingcys     first version
 * 2023/01/17     chushicheng   add pin and i2c
 * 2023/02/21     wcx1024979076 use bl_mcu_sdk & add spi and lcd
 */
#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "bl808_glb.h"

/* This is the timer interrupt service routine. */
static void mtime_handler(void)
{
    rt_tick_increase();

    csi_coret_config(CPU_Get_MTimer_Clock() / RT_TICK_PER_SECOND, MTIME_IRQn);
}

void rt_hw_board_init(void)
{
    board_init();   // define at /library/bl_mcu_sdk/bsp/board/bl808dk/board.c

    csi_coret_config(CPU_Get_MTimer_Clock() / RT_TICK_PER_SECOND, MTIME_IRQn);
    bflb_irq_attach(MTIME_IRQn, mtime_handler, NULL);
    bflb_irq_enable(MTIME_IRQn);

#ifdef RT_USING_HEAP
    /* initialize memory system */
    rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
#endif

    /* GPIO driver initialization is open by default */
#ifdef RT_USING_PIN
    rt_hw_pin_init();
#endif

    /* I2C driver initialization is open by default */
#ifdef RT_USING_I2C
    rt_hw_i2c_init();
#endif

    /* SPI driver initialization is open by default */ 
#ifdef RT_USING_SPI
    rt_hw_spi_init();
#endif

    /* UART driver initialization is open by default */
#ifdef RT_USING_SERIAL
    rt_hw_uart_init();
#endif

    /* Set the shell console output device */
#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}

void rt_hw_cpu_reset(void)
{
    __disable_irq();
    GLB_SW_POR_Reset();
    while(1);
}

MSH_CMD_EXPORT_ALIAS(rt_hw_cpu_reset, reboot, reset machine);
