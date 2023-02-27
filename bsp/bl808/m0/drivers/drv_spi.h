/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date             Author          Notes
 * 2023-01-19       wcx1024979076    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include "bflb_spi.h"

#ifndef __DRV_SPI_H_
#define __DRV_SPI_H_

rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name, rt_uint32_t ss_pin);

//SPI bus config
#ifdef BSP_USING_SPI0
#define MCU_SPI0_CONFIG          \
{                                \
    .bus_name = "spi0",          \
}
#endif
#ifdef BSP_USING_SPI1
#define MCU_SPI1_CONFIG          \
{                                \
    .bus_name = "spi1",          \
}
#endif

struct mcu_drv_spi_config
{
    char *bus_name;
};

struct mcu_drv_spi
{
    struct rt_spi_configuration *cfg;
    struct rt_spi_bus spi_bus;
};

int rt_hw_spi_init(void);

#endif  /*__DRV_SPI_H_*/
