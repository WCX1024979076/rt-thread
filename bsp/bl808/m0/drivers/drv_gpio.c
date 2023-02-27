/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023/01/5      chushicheng  first version
 * 2023/02/27    wcx1024979076 use bl_mcu_sdk
 *
 */

#include "drv_gpio.h"
#include <stdbool.h>

#ifdef RT_USING_PIN

#define DBG_TAG              "drv.gpio"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>

static void GPIO0_IRQHandler(void);

struct gpio_int_cfg_private
{
    bflb_dlist_t list;
    uint32_t pin;
    void (*hdr)(uint32_t pin);
};

static bflb_dlist_t gpio_int_head = DLIST_OBJECT_INIT(gpio_int_head);


static void bl808_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    struct bflb_device_s* gpio;
    gpio = bflb_device_get_by_name("gpio");
    if(value != 0)
        bflb_gpio_set(gpio, pin);
    else
        bflb_gpio_reset(gpio, pin);
}

static int bl808_pin_read(rt_device_t dev, rt_base_t pin)
{
    int value;
    struct bflb_device_s* gpio;
    gpio = bflb_device_get_by_name("gpio");
    value = bflb_gpio_read(gpio, pin);
    return value;
}

static void bl808_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    struct bflb_device_s* gpio;
    gpio = bflb_device_get_by_name("gpio");
    uint32_t gpio_cfg;
    
    gpio_cfg |= GPIO_DRV_0;
    gpio_cfg |= GPIO_SMT_EN;

    switch (mode)
    {
        case GPIO_OUTPUT_MODE:
            gpio_cfg |= GPIO_OUTPUT;
            gpio_cfg |=  GPIO_FLOAT;
            break;

        case GPIO_OUTPUT_PP_MODE:
            gpio_cfg |= GPIO_OUTPUT;
            gpio_cfg |= GPIO_PULLUP;
            break;

        case GPIO_OUTPUT_PD_MODE:
            gpio_cfg |= GPIO_OUTPUT;
            gpio_cfg |= GPIO_PULLDOWN;
            break;

        case GPIO_INPUT_MODE:
            gpio_cfg |= GPIO_INPUT;
            gpio_cfg |= GPIO_FLOAT;
            break;

        case GPIO_INPUT_PP_MODE:
            gpio_cfg |= GPIO_INPUT;
            gpio_cfg |= GPIO_PULLUP;
            break;

        case GPIO_INPUT_PD_MODE:
            gpio_cfg |= GPIO_INPUT;
            gpio_cfg |= GPIO_PULLDOWN;
            break;
        case GPIO_HZ_MODE:
            GLB_GPIO_Set_HZ(pin);
        default:
            bflb_irq_disable(GPIO_INT0_IRQn);
            bflb_gpio_int_mask(gpio, pin, true);
            gpio_cfg |= GPIO_INPUT;

            if (mode == GPIO_ASYNC_RISING_TRIGER_INT_MODE)
            {
                gpio_cfg |= GPIO_PULLDOWN;
                gpio_cfg |= GPIO_INT_TRIG_MODE_ASYNC_RISING_EDGE;
            }
            else if (mode == GPIO_ASYNC_FALLING_TRIGER_INT_MODE)
            {
                gpio_cfg |= GPIO_PULLUP;
                gpio_cfg |= GPIO_INT_TRIG_MODE_ASYNC_FALLING_EDGE;
            }
            else if (mode == GPIO_ASYNC_HIGH_LEVEL_INT_MODE)
            {
                gpio_cfg |= GPIO_PULLDOWN;
                gpio_cfg |= GPIO_INT_TRIG_MODE_ASYNC_HIGH_LEVEL;
            }
            else if (mode == GPIO_ASYNC_LOW_LEVEL_INT_MODE)
            {
                gpio_cfg |= GPIO_PULLUP;
                gpio_cfg |= GPIO_INT_TRIG_MODE_ASYNC_LOW_LEVEL;
            }
            else if (mode == GPIO_SYNC_RISING_TRIGER_INT_MODE)
            {
                gpio_cfg |= GPIO_PULLDOWN;
                gpio_cfg |= GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE;
            }
            else if (mode == GPIO_SYNC_FALLING_TRIGER_INT_MODE)
            {
                gpio_cfg |= GPIO_PULLUP;
                gpio_cfg |= GPIO_INT_TRIG_MODE_SYNC_FALLING_EDGE;
            }
            else if (mode == GPIO_SYNC_FALLING_TRIGER_INT_MODE)
            {
                gpio_cfg |= GPIO_FLOAT;
                gpio_cfg |= GPIO_INT_TRIG_MODE_SYNC_FALLING_RISING_EDGE;
            }
            else if (mode == GPIO_SYNC_HIGH_LEVEL_INT_MODE)
            {
                gpio_cfg |= GPIO_PULLDOWN;
                gpio_cfg |= GPIO_INT_TRIG_MODE_SYNC_HIGH_LEVEL;
            }
            else if (mode == GPIO_SYNC_LOW_LEVEL_INT_MODE)
            {
                gpio_cfg |= GPIO_PULLUP;
                gpio_cfg |= GPIO_INT_TRIG_MODE_SYNC_LOW_LEVEL;
            }

            break;
    }
    bflb_gpio_init(gpio, pin, gpio_cfg);
}


static rt_err_t bl808_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                                   rt_uint32_t irq_mode, void (*hdr)(void *args), void *args)
{
    struct bflb_device_s* gpio;
    gpio = bflb_device_get_by_name("gpio");
    struct gpio_int_cfg_private *int_cfg = malloc(sizeof(struct gpio_int_cfg_private));
    int_cfg->hdr = hdr;
    int_cfg->pin = pin;
    bflb_dlist_insert_after(&gpio_int_head, &int_cfg->list);
    bflb_irq_disable(GPIO_INT0_IRQn);
    bflb_irq_attach(GPIO_INT0_IRQn, GPIO0_IRQHandler, NULL);
    bflb_irq_enable(GPIO_INT0_IRQn);
    return RT_EOK;
}


static rt_err_t bl808_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                                   rt_uint32_t enabled)
{
    struct bflb_device_s* gpio;
    gpio = bflb_device_get_by_name("gpio");
    if (enabled)
    {
        bflb_gpio_int_mask(gpio, pin, false);
    }
    else
    {
        bflb_gpio_int_mask(gpio, pin, true);
    }
    return RT_EOK;
}

const static struct rt_pin_ops _bl808_pin_ops =
{
    bl808_pin_mode,
    bl808_pin_write,
    bl808_pin_read,
    bl808_pin_attach_irq,
    bl808_pin_irq_enable,
    NULL,
};

int rt_hw_pin_init(void)
{
    bflb_dlist_init(&gpio_int_head);
    return rt_device_pin_register("pin", &_bl808_pin_ops, RT_NULL);
}
INIT_BOARD_EXPORT(rt_hw_pin_init);

/* irq handle */
void GPIO0_IRQHandler(void)
{
    rt_interrupt_enter();
    // GPIO_INT0_IRQHandler();
    rt_interrupt_leave();
}

#endif /* RT_USING_PIN */
