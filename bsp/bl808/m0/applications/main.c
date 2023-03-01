/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022/12/25     flyingcys    first version
 */

#include <rtthread.h>
#include <stdio.h>
#include "drv_lcd.h"

int main(void)
{
    rt_kprintf("Hello, world\n");
#ifdef BSP_USING_LCD
    lcd_show_num(0, 0, 1, 1, 32);
    lcd_fill(0, 0, 240, 240, RED);
#endif
    return 0;
}
