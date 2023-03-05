/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date             Author          Notes
 * 2023-01-19       wcx1024979076    first version
 */

#include <stdint.h>
#include <string.h>
#include "board.h"
#include "drv_spi.h"

#define DBG_LEVEL   DBG_LOG
#include <rtdbg.h>
#define LOG_TAG "drv.spi"

#ifdef BSP_USING_SPI

#if defined(BSP_USING_SPI0) || defined(BSP_USING_SPI1) || defined(BSP_USING_SPI2)
static struct mcu_drv_spi_config spi_config[] =
{
#ifdef BSP_USING_SPI0
    MCU_SPI0_CONFIG,
#endif
#ifdef BSP_USING_SPI1
    MCU_SPI1_CONFIG,
#endif
};

static struct mcu_drv_spi spi_bus_obj[sizeof(spi_config) / sizeof(spi_config[0])];

static rt_err_t spi_configure(struct rt_spi_device *device,
                              struct rt_spi_configuration *configuration)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);
    RT_ASSERT(device->bus->parent.user_data != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);
    struct mcu_drv_spi *tmp_spi;
    tmp_spi = rt_container_of(device->bus, struct mcu_drv_spi, spi_bus);

    struct bflb_spi_config_s spi_cfg = {
        .freq = 1 * 1000 * 1000,
        .role = SPI_ROLE_MASTER,
        .mode = SPI_MODE3,
        .data_width = SPI_DATA_WIDTH_8BIT,
        .bit_order = SPI_BIT_MSB,
        .byte_order = SPI_BYTE_LSB,
        .tx_fifo_threshold = 0,
        .rx_fifo_threshold = 0,
    };

    switch (configuration->mode & RT_SPI_MODE_3)
    {
    case RT_SPI_MODE_0/* RT_SPI_CPOL:0 , RT_SPI_CPHA:0 */:
        spi_cfg.mode = SPI_MODE0;
        break;
    case RT_SPI_MODE_1/* RT_SPI_CPOL:0 , RT_SPI_CPHA:1 */:
        spi_cfg.mode = SPI_MODE1;
        break;
    case RT_SPI_MODE_2/* RT_SPI_CPOL:1 , RT_SPI_CPHA:0 */:
        spi_cfg.mode = SPI_MODE2;
        break;
    case RT_SPI_MODE_3/* RT_SPI_CPOL:1 , RT_SPI_CPHA:1 */:
        spi_cfg.mode = SPI_MODE3;
        break;
    default:
        LOG_E("spi_configure mode error %x\n", configuration->mode);
        return RT_ERROR;
    }

    switch (configuration->data_width)
    {
    case 8:
        spi_cfg.data_width = SPI_DATA_WIDTH_8BIT;
        break;
    case 16:
        spi_cfg.data_width = SPI_DATA_WIDTH_16BIT;
        break;
    case 24:
        spi_cfg.data_width = SPI_DATA_WIDTH_24BIT;
        break;
    case 32:
        spi_cfg.data_width = SPI_DATA_WIDTH_32BIT;
        break;
    default:
        LOG_E("spi_configure data_width error %x\n", configuration->data_width);
        return RT_ERROR;
    }

    spi_cfg.freq = configuration->max_hz;

    if(configuration->mode & RT_SPI_MSB)
        spi_cfg.bit_order = SPI_BIT_MSB;
    else
        spi_cfg.bit_order = SPI_BIT_LSB;

    if(configuration->mode & RT_SPI_MASTER)
        spi_cfg.role = SPI_ROLE_MASTER;
    else
        spi_cfg.role = SPI_ROLE_SLAVE;

    struct bflb_device_s *spi;
    struct mcu_drv_spi_config *mcu_spi_config;

    mcu_spi_config = tmp_spi->spi_bus.parent.user_data;
    spi = bflb_device_get_by_name(mcu_spi_config->bus_name);
    bflb_spi_init(spi, &spi_cfg);
    LOG_D("%s config end", mcu_spi_config->bus_name);

    return RT_EOK;
}

static rt_uint32_t spixfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);
    RT_ASSERT(device->bus->parent.user_data != RT_NULL);
    rt_uint32_t cs_pin = (rt_uint32_t)device->parent.user_data;
    struct bflb_device_s* gpio = bflb_device_get_by_name("gpio");
    struct mcu_drv_spi *tmp_spi;
    struct bflb_device_s *spi;
    struct mcu_drv_spi_config *mcu_spi_config;

    tmp_spi = rt_container_of(device->bus, struct mcu_drv_spi, spi_bus);
    mcu_spi_config = tmp_spi->spi_bus.parent.user_data;
    spi = bflb_device_get_by_name(mcu_spi_config->bus_name);
    if(message->cs_take)
        bflb_gpio_reset(gpio, cs_pin);
    bflb_spi_poll_exchange(spi, (void *)message->send_buf, (void *)message->recv_buf,  message->length);
    if(message->cs_release)
        bflb_gpio_set(gpio, cs_pin);
    return message->length;
}

/* spi bus callback function  */
static const struct rt_spi_ops nrfx_spi_ops =
{
    .configure = spi_configure,
    .xfer = spixfer,
};

/*spi bus init*/
static int rt_hw_spi_bus_init(void)
{
    rt_err_t result = RT_ERROR;
    for (int i = 0; i < sizeof(spi_config) / sizeof(spi_config[0]); i++)
    {
        spi_bus_obj[i].spi_bus.parent.user_data = &spi_config[i];   //SPI INSTANCE
        result = rt_spi_bus_register(&spi_bus_obj[i].spi_bus, spi_config[i].bus_name, &nrfx_spi_ops);
        RT_ASSERT(result == RT_EOK);
    }
    return result;
}

int rt_hw_spi_init(void)
{
    return rt_hw_spi_bus_init();
}

/**
  * Attach the spi device to SPI bus, this function must be used after initialization.
  */
rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name, rt_uint32_t cs_pin)
{
    RT_ASSERT(bus_name != RT_NULL);
    RT_ASSERT(device_name != RT_NULL);
    RT_ASSERT(cs_pin != RT_NULL);

    rt_err_t result;
    struct rt_spi_device *spi_device;
    /* attach the device to spi bus*/
    spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));

    RT_ASSERT(spi_device != RT_NULL);
    /* initialize the cs pin */
    result = rt_spi_bus_attach_device(spi_device, device_name, bus_name, (void *)cs_pin);
    if (result != RT_EOK)
    {
        LOG_E("%s attach to %s faild, %d", device_name, bus_name, result);
        result = RT_ERROR;
    }

    /* SET THE GPIO */
    struct bflb_device_s *gpio;
    gpio = bflb_device_get_by_name("gpio");
    if(strcmp(bus_name, "spi1") == 0)
    {
        /* spi cs */
        bflb_gpio_init(gpio, cs_pin,      GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
        /* spi miso */
        bflb_gpio_init(gpio, GPIO_PIN_26, GPIO_FUNC_SPI1 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
        /* spi mosi */
        bflb_gpio_init(gpio, GPIO_PIN_25, GPIO_FUNC_SPI1 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
        /* spi clk */
        bflb_gpio_init(gpio, GPIO_PIN_19, GPIO_FUNC_SPI1 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    }
    else
    {
        /* spi cs */
        bflb_gpio_init(gpio, cs_pin,     GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
        /* spi miso */
        bflb_gpio_init(gpio, GPIO_PIN_1, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
        /* spi mosi */
        bflb_gpio_init(gpio, GPIO_PIN_2, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
        /* spi clk */
        bflb_gpio_init(gpio, GPIO_PIN_3, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    }
    RT_ASSERT(result == RT_EOK);
    return result;
}

#endif /* BSP_USING_SPI0 || BSP_USING_SPI1 || BSP_USING_SPI2 */
#endif /*BSP_USING_SPI*/
