/*
 * Copyright (c) 2025 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_gpio_uart_line_control

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/dt-bindings/gpio/gpio-uart-line-control.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_uart_line, CONFIG_GPIO_LOG_LEVEL);

struct gpio_uart_line_config {
	struct gpio_driver_config common;
	const struct device *uart_dev;
};

struct gpio_uart_line_data {
	struct gpio_driver_data common;
	gpio_port_value_t value;
};

static int gpio_uart_line_pin_configure(const struct device *port, gpio_pin_t pin,
					gpio_flags_t flags)
{
	switch (pin) {
	case UART_LINE_DTR:
	case UART_LINE_RTS:
		if ((flags & GPIO_INPUT) == 0) {
			LOG_ERR("Pin %d must be configured as input", pin);
			return -EINVAL;
		}
		break;
	case UART_LINE_DCD:
	case UART_LINE_DSR:
		if ((flags & GPIO_OUTPUT) == 0) {
			LOG_ERR("Pin %d must be configured as output", pin);
			return -EINVAL;
		}
		break;
	default:
		LOG_ERR("Unsupported UART line control pin: %d", pin);
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_uart_line_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
	const struct gpio_uart_line_config *cfg = port->config;
	struct gpio_uart_line_data *data = port->data;
	uint32_t line_ctrl = 0;
	int ret;

	ret = uart_line_ctrl_get(cfg->uart_dev, UART_LINE_CTRL_DTR, &line_ctrl);
	if (ret < 0) {
		return ret;
	}

	if (line_ctrl) {
		data->value |= BIT(UART_LINE_DTR);
	} else {
		data->value &= ~BIT(UART_LINE_DTR);
	}

	ret = uart_line_ctrl_get(cfg->uart_dev, UART_LINE_CTRL_RTS, &line_ctrl);
	if (ret < 0) {
		return ret;
	}

	if (line_ctrl) {
		data->value |= BIT(UART_LINE_RTS);
	} else {
		data->value &= ~BIT(UART_LINE_RTS);
	}

	*value = data->value;

	return 0;
}

static int gpio_uart_line_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					      gpio_port_value_t value)
{
	const struct gpio_uart_line_config *cfg = port->config;
	struct gpio_uart_line_data *data = port->data;
	int ret;

	if (mask & BIT(UART_LINE_DCD)) {
		ret = uart_line_ctrl_set(cfg->uart_dev, UART_LINE_CTRL_DTR,
					 (value & BIT(UART_LINE_DCD)) ? 1 : 0);
		if (ret < 0) {
			return ret;
		}

		if (value & BIT(UART_LINE_DCD)) {
			data->value |= BIT(UART_LINE_DCD);
		} else {
			data->value &= ~BIT(UART_LINE_DCD);
		}
	}

	if (mask & BIT(UART_LINE_DSR)) {
		ret = uart_line_ctrl_set(cfg->uart_dev, UART_LINE_CTRL_DSR,
					 (value & BIT(UART_LINE_DSR)) ? 1 : 0);
		if (ret < 0) {
			return ret;
		}

		if (value & BIT(UART_LINE_DSR)) {
			data->value |= BIT(UART_LINE_DSR);
		} else {
			data->value &= ~BIT(UART_LINE_DSR);
		}
	}

	return 0;
}

static int gpio_uart_line_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_uart_line_config *cfg = port->config;
	struct gpio_uart_line_data *data = port->data;
	int ret;

	if (pins & BIT(UART_LINE_DCD)) {
		ret = uart_line_ctrl_set(cfg->uart_dev, UART_LINE_CTRL_DTR, 1);
		if (ret < 0) {
			return ret;
		}

		data->value |= BIT(UART_LINE_DCD);
	}

	if (pins & BIT(UART_LINE_DSR)) {
		ret = uart_line_ctrl_set(cfg->uart_dev, UART_LINE_CTRL_DSR, 1);
		if (ret < 0) {
			return ret;
		}

		data->value |= BIT(UART_LINE_DSR);
	}

	return 0;
}

static int gpio_uart_line_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_uart_line_config *cfg = port->config;
	struct gpio_uart_line_data *data = port->data;
	int ret;

	if (pins & BIT(UART_LINE_DCD)) {
		ret = uart_line_ctrl_set(cfg->uart_dev, UART_LINE_CTRL_DTR, 0);
		if (ret < 0) {
			return ret;
		}

		data->value &= ~BIT(UART_LINE_DCD);
	}

	if (pins & BIT(UART_LINE_DSR)) {
		ret = uart_line_ctrl_set(cfg->uart_dev, UART_LINE_CTRL_DSR, 0);
		if (ret < 0) {
			return ret;
		}

		data->value &= ~BIT(UART_LINE_DSR);
	}

	return 0;
}

static int gpio_uart_line_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_uart_line_config *cfg = port->config;
	struct gpio_uart_line_data *data = port->data;
	int ret;

	if (pins & BIT(UART_LINE_DCD)) {
		if (data->value & BIT(UART_LINE_DCD)) {
			ret = uart_line_ctrl_set(cfg->uart_dev, UART_LINE_CTRL_DTR, 0);
			if (ret < 0) {
				return ret;
			}

			data->value &= ~BIT(UART_LINE_DCD);
		} else {
			ret = uart_line_ctrl_set(cfg->uart_dev, UART_LINE_CTRL_DTR, 1);
			if (ret < 0) {
				return ret;
			}

			data->value |= BIT(UART_LINE_DCD);
		}
	}

	if (pins & BIT(UART_LINE_DSR)) {
		if (data->value & BIT(UART_LINE_DSR)) {
			ret = uart_line_ctrl_set(cfg->uart_dev, UART_LINE_CTRL_DSR, 0);
			if (ret < 0) {
				return ret;
			}

			data->value &= ~BIT(UART_LINE_DSR);
		} else {
			ret = uart_line_ctrl_set(cfg->uart_dev, UART_LINE_CTRL_DSR, 1);
			if (ret < 0) {
				return ret;
			}

			data->value |= BIT(UART_LINE_DSR);
		}
	}

	return 0;
}

static int gpio_uart_line_init(const struct device *port)
{
	const struct gpio_uart_line_config *cfg = port->config;

	if (!device_is_ready(cfg->uart_dev)) {
		LOG_ERR("UART device %s is not ready", cfg->uart_dev->name);
		return -ENODEV;
	}

	return 0;
}

static DEVICE_API(gpio, gpio_uart_line_api) = {
	.pin_configure = gpio_uart_line_pin_configure,
	.port_get_raw = gpio_uart_line_port_get_raw,
	.port_set_masked_raw = gpio_uart_line_port_set_masked_raw,
	.port_set_bits_raw = gpio_uart_line_port_set_bits_raw,
	.port_clear_bits_raw = gpio_uart_line_port_clear_bits_raw,
	.port_toggle_bits = gpio_uart_line_port_toggle_bits,
};

#define GPIO_UART_LINE_INIT(n)                                                                     \
	static const struct gpio_uart_line_config gpio_uart_line_cfg_##n = {                       \
		.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0)},                   \
		.uart_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                      \
	};                                                                                         \
                                                                                                   \
	static struct gpio_uart_line_data gpio_uart_line_data_##n;                                 \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, gpio_uart_line_init, NULL, &gpio_uart_line_data_##n,              \
			      &gpio_uart_line_cfg_##n, POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,     \
			      &gpio_uart_line_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_UART_LINE_INIT)
