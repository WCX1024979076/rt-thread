/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2023-2-24    wcx1024979076      first version
 */

#include <rtdevice.h>
#include <drv_gpio.h>
#include <drv_spi.h>
#include "drv_flash.h"
#include "bflb_clock.h"
#include <rtthread.h>
#include <stdio.h>

int rt_hw_flash_init(void)
{
    rt_hw_spi_device_attach("spi0", "spi00", GPIO_PIN_4); // ST7789V_SPI_CS_PIN
    uint32_t wdata[1] = {0x9f};
    uint32_t rdata[1] = {0x9f};

    struct rt_spi_device *spi_dev_flash = (struct rt_spi_device *)rt_device_find("spi00");
    /* config spi */
    {
        struct rt_spi_configuration cfg = {0};
        cfg.data_width = 32;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
        cfg.max_hz = 1 * 1000 * 1000; /* 40M,SPI max 40MHz,lcd 4-wire spi */
        rt_spi_configure(spi_dev_flash, &cfg);
    }
    rt_kprintf("flash jedec %x %d %x %d\n", rdata[0], rdata[0], wdata[0], wdata[0]);
    rt_spi_send_then_recv(spi_dev_flash, wdata, 1, rdata, 1);
    rt_kprintf("flash jedec %x %d %x %d\n", rdata[0], rdata[0], wdata[0], wdata[0]);
    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_flash_init);