/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT luat_air105_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>

#include <luat/air105_gpio.h>

struct gpio_air105_config {
	struct gpio_driver_config common;
	GPIO_TypeDef *regs;
	void (*irq_config_func)(void);
};

struct gpio_air105_data {
	struct gpio_driver_data common;
	sys_slist_t cb;
};

static int gpio_air105_get_port_idx(const struct device *dev)
{
	const struct gpio_air105_config *config = dev->config;

	if (config->regs == GPIOA) {
		return 0;
	} else if (config->regs == GPIOB) {
		return 1;
	} else if (config->regs == GPIOC) {
		return 2;
	} else if (config->regs == GPIOD) {
		return 3;
	} else if (config->regs == GPIOE) {
		return 4;
	} else if (config->regs == GPIOF) {
		return 5;
	} else {
		/* Never happen */
		return -1;
	}
}

static int gpio_air105_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_air105_config *config = dev->config;

	if (flags & GPIO_INPUT) {
		config->regs->OEN |= BIT(pin);
	} else if (flags & GPIO_OUTPUT) {
		config->regs->OEN &= ~BIT(pin);
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			config->regs->IODR |= BIT(pin);
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			config->regs->IODR &= ~BIT(pin);
		}
	} else {
		return -ENOTSUP;
	}

	if (flags & GPIO_PULL_UP) {
		config->regs->PUE |= BIT(pin);
	} else if (flags & GPIO_PULL_DOWN) {
		return -ENOTSUP;
	} else {
		config->regs->PUE &= ~BIT(pin);
	}

	return 0;
}

static int gpio_air105_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct gpio_air105_config *config = dev->config;

	*value = (config->regs->IODR >> 16) & BIT_MASK(16);

	return 0;
}

static int gpio_air105_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					   gpio_port_value_t value)
{
	const struct gpio_air105_config *config = dev->config;

	config->regs->IODR = (config->regs->IODR & ~mask) | value;

	return 0;
}

static int gpio_air105_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_air105_config *config = dev->config;

	config->regs->BSRR = pins;

	return 0;
}

static int gpio_air105_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_air105_config *config = dev->config;

	config->regs->BSRR = pins << 16;

	return 0;
}

static int gpio_air105_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_air105_config *config = dev->config;

	config->regs->IODR ^= pins;

	return 0;
}

static int gpio_air105_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					       enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	uint32_t regval;
	uint32_t idx;
	uint32_t pos;

	if (mode == GPIO_INT_MODE_LEVEL) {
		return -ENOTSUP;
	}

	idx = gpio_air105_get_port_idx(dev);
	pos = pin * 2;

	regval = GPIO->INTP_TYPE_STA[idx].INTP_TYPE;
	regval &= ~(BIT_MASK(2) << pos);

	switch (trig) {
	case GPIO_INT_TRIG_LOW:
		regval |= (0x2 << pos);
		break;
	case GPIO_INT_TRIG_HIGH:
		regval |= (0x1 << pos);
		break;
	case GPIO_INT_TRIG_BOTH:
		regval |= (0x3 << pos);
		break;
	}

	GPIO->INTP_TYPE_STA[idx].INTP_TYPE = regval;

	return 0;
}

static int gpio_air105_manage_callback(const struct device *dev, struct gpio_callback *cb, bool set)
{
	struct gpio_air105_data *data = dev->data;

	return gpio_manage_callback(&data->cb, cb, set);
}

static void gpio_air105_isr(const struct device *dev)
{
	struct gpio_air105_data *data = dev->data;

	uint32_t regval;
	uint32_t idx;

	idx = gpio_air105_get_port_idx(dev);

	regval = GPIO->INTP_TYPE_STA[idx].INTP_STA;

	gpio_fire_callbacks(&data->cb, dev, regval);

	/* Clear the interrupts */
	GPIO->INTP_TYPE_STA[idx].INTP_STA = regval;
}

int gpio_air105_init(const struct device *dev)
{
	const struct gpio_air105_config *config = dev->config;

	config->irq_config_func();

	return 0;
}

static const struct gpio_driver_api gpio_air105_api = {
	.pin_configure = gpio_air105_pin_configure,
	.port_get_raw = gpio_air105_port_get_raw,
	.port_set_masked_raw = gpio_air105_port_set_masked_raw,
	.port_set_bits_raw = gpio_air105_port_set_bits_raw,
	.port_clear_bits_raw = gpio_air105_port_clear_bits_raw,
	.port_toggle_bits = gpio_air105_port_toggle_bits,
	.pin_interrupt_configure = gpio_air105_pin_interrupt_configure,
	.manage_callback = gpio_air105_manage_callback,
};

#define GPIO_AIR105_INIT(n)                                                                        \
	static void gpio_air105_irq_config_func_##n(void)                                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), gpio_air105_isr,            \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static struct gpio_air105_data gpio_air105_data_##n;                                       \
                                                                                                   \
	static const struct gpio_air105_config gpio_air105_config_##n = {                          \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),               \
			},                                                                         \
		.regs = (GPIO_TypeDef *)DT_INST_REG_ADDR(n),                                       \
		.irq_config_func = gpio_air105_irq_config_func_##n,                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, gpio_air105_init, NULL, &gpio_air105_data_##n,                    \
			      &gpio_air105_config_##n, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,    \
			      &gpio_air105_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_AIR105_INIT)
