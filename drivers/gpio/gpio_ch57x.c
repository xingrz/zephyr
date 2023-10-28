/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch57x_gpio

#include <zephyr/arch/cpu.h>
#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#define GPIO_R32_PAD_DIR(base)    (base + 0x00)
#define GPIO_R32_PAD_PIN(base)    (base + 0x04)
#define GPIO_R32_PAD_OUT(base)    (base + 0x08)
#define GPIO_R32_PAD_CLR(base)    (base + 0x0C)
#define GPIO_R32_PAD_PU(base)     (base + 0x10)
#define GPIO_R32_PAD_PD_DRV(base) (base + 0x14)

struct gpio_ch57x_config {
	struct gpio_driver_config common;
	void (*irq_config_func)(void);
	mem_addr_t base;
};

struct gpio_ch57x_data {
	struct gpio_driver_data common;
	sys_slist_t cb;
};

static int gpio_ch57x_pin_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_ch57x_config *cfg = port->config;
	uint32_t regval;

	if (flags & GPIO_OPEN_DRAIN) {
		/* Open drain not supported */
		return -ENOTSUP;
	}

	/* Reset */
	{
		regval = sys_read32(GPIO_R32_PAD_PU(cfg->base));
		regval &= ~BIT(pin);
		sys_write32(regval, GPIO_R32_PAD_PU(cfg->base));

		regval = sys_read32(GPIO_R32_PAD_PD_DRV(cfg->base));
		regval &= ~BIT(pin);
		sys_write32(regval, GPIO_R32_PAD_PD_DRV(cfg->base));
	}

	if (flags & GPIO_INPUT) {
		/* Set input direction */
		{
			regval = sys_read32(GPIO_R32_PAD_DIR(cfg->base));
			regval &= ~BIT(pin);
			sys_write32(regval, GPIO_R32_PAD_DIR(cfg->base));
		}

		/* Set pulls */
		if (flags & GPIO_PULL_UP) {
			regval = sys_read32(GPIO_R32_PAD_PU(cfg->base));
			regval |= BIT(pin);
			sys_write32(regval, GPIO_R32_PAD_PU(cfg->base));
		} else if (flags & GPIO_PULL_DOWN) {
			regval = sys_read32(GPIO_R32_PAD_PD_DRV(cfg->base));
			regval |= BIT(pin);
			sys_write32(regval, GPIO_R32_PAD_PD_DRV(cfg->base));
		}
	} else if (flags & GPIO_OUTPUT) {
		/* Set output direction */
		{
			regval = sys_read32(GPIO_R32_PAD_DIR(cfg->base));
			regval |= BIT(pin);
			sys_write32(regval, GPIO_R32_PAD_DIR(cfg->base));
		}

		/* Set initial level */
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			regval = sys_read32(GPIO_R32_PAD_OUT(cfg->base));
			regval |= BIT(pin);
			sys_write32(regval, GPIO_R32_PAD_OUT(cfg->base));
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			regval = sys_read32(GPIO_R32_PAD_CLR(cfg->base));
			regval |= BIT(pin);
			sys_write32(regval, GPIO_R32_PAD_CLR(cfg->base));
		}
	}

	return 0;
}

static int gpio_ch57x_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
	const struct gpio_ch57x_config *cfg = port->config;

	*value = sys_read32(GPIO_R32_PAD_PIN(cfg->base));

	return 0;
}

static int gpio_ch57x_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct gpio_ch57x_config *cfg = port->config;
	uint32_t regval;

	regval = sys_read32(GPIO_R32_PAD_OUT(cfg->base));
	regval &= ~mask;
	regval |= value;
	sys_write32(regval, GPIO_R32_PAD_OUT(cfg->base));

	return 0;
}

static int gpio_ch57x_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_ch57x_config *cfg = port->config;
	uint32_t regval;

	regval = sys_read32(GPIO_R32_PAD_OUT(cfg->base));
	regval |= pins;
	sys_write32(regval, GPIO_R32_PAD_OUT(cfg->base));

	return 0;
}

static int gpio_ch57x_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_ch57x_config *cfg = port->config;

	sys_write32(pins, GPIO_R32_PAD_CLR(cfg->base));

	return 0;
}

static int gpio_ch57x_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_ch57x_config *cfg = port->config;
	uint32_t regval;

	regval = sys_read32(GPIO_R32_PAD_OUT(cfg->base));
	regval ^= pins;
	sys_write32(regval, GPIO_R32_PAD_OUT(cfg->base));

	return 0;
}

static int gpio_ch57x_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
					      enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	return 0;
}

static int gpio_ch57x_manage_callback(const struct device *port, struct gpio_callback *cb, bool set)
{
	struct gpio_ch57x_data *data = port->data;

	return gpio_manage_callback(&data->cb, cb, set);
}

static void gpio_ch57x_isr(const struct device *port)
{
}

int gpio_ch57x_init(const struct device *port)
{
	const struct gpio_ch57x_config *cfg = port->config;

	cfg->irq_config_func();

	return 0;
}

static const struct gpio_driver_api gpio_ch57x_api = {
	.pin_configure = gpio_ch57x_pin_configure,
	.port_get_raw = gpio_ch57x_port_get_raw,
	.port_set_masked_raw = gpio_ch57x_port_set_masked_raw,
	.port_set_bits_raw = gpio_ch57x_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ch57x_port_clear_bits_raw,
	.port_toggle_bits = gpio_ch57x_port_toggle_bits,
	.pin_interrupt_configure = gpio_ch57x_pin_interrupt_configure,
	.manage_callback = gpio_ch57x_manage_callback,
};

#define GPIO_CH57X_INST(n)                                                                         \
	static struct gpio_ch57x_data gpio_ch57x_data_##n;                                         \
                                                                                                   \
	static void gpio_ch57x_irq_config_func_##n(void)                                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), gpio_ch57x_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct gpio_ch57x_config gpio_ch57x_cfg_##n = {                               \
		.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n)},                   \
		.irq_config_func = gpio_ch57x_irq_config_func_##n,                                 \
		.base = DT_INST_REG_ADDR(n),                                                       \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, gpio_ch57x_init, NULL, &gpio_ch57x_data_##n, &gpio_ch57x_cfg_##n, \
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &gpio_ch57x_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_CH57X_INST)
