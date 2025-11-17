/*
 * Copyright (c) 2025 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT m5stack_unit_extio2_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mfd/m5stack_unit_extio2.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(extio2_gpio, CONFIG_GPIO_LOG_LEVEL);

#define EXTIO2_IO_COUNT 8

struct extio2_gpio_config {
	struct gpio_driver_config common;
	struct i2c_dt_spec i2c;
};

struct extio2_gpio_data {
	struct gpio_driver_data common;
	uint8_t mode[EXTIO2_IO_COUNT];
	uint8_t output[EXTIO2_IO_COUNT];
	uint8_t input[EXTIO2_IO_COUNT];
};

static int extio2_gpio_pin_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct extio2_gpio_config *cfg = port->config;
	struct extio2_gpio_data *data = port->data;
	int ret;

	if (flags & (GPIO_OPEN_DRAIN | GPIO_PULL_UP | GPIO_PULL_DOWN)) {
		return -ENOTSUP;
	}

	if (flags & GPIO_INPUT) {
		data->mode[pin] = EXTIO2_MODE_INPUT;
		ret = i2c_reg_write_byte_dt(&cfg->i2c, EXTIO2_REG_MODE_SETTING + pin,
					    data->mode[pin]);
		if (ret < 0) {
			return ret;
		}
	} else if (flags & GPIO_OUTPUT) {
		data->mode[pin] = EXTIO2_MODE_OUTPUT;
		ret = i2c_reg_write_byte_dt(&cfg->i2c, EXTIO2_REG_MODE_SETTING + pin,
					    data->mode[pin]);
		if (ret < 0) {
			return ret;
		}

		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			data->output[pin] = 1;
			ret = i2c_reg_write_byte_dt(&cfg->i2c, EXTIO2_REG_OUTPUT_CTRL + pin,
						    data->output[pin]);
			if (ret < 0) {
				return ret;
			}
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			data->output[pin] = 0;
			ret = i2c_reg_write_byte_dt(&cfg->i2c, EXTIO2_REG_OUTPUT_CTRL + pin,
						    data->output[pin]);
			if (ret < 0) {
				return ret;
			}
		}
	}

	return 0;
}

static int extio2_gpio_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
	const struct extio2_gpio_config *cfg = port->config;
	struct extio2_gpio_data *data = port->data;
	gpio_port_value_t tmp = 0;
	int ret;

	ret = i2c_burst_read_dt(&cfg->i2c, EXTIO2_REG_DIGITAL_INPUT, data->input, EXTIO2_IO_COUNT);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < EXTIO2_IO_COUNT; i++) {
		if (data->mode[i] == EXTIO2_MODE_INPUT) {
			WRITE_BIT(tmp, i, data->input[i]);
		} else if (data->mode[i] == EXTIO2_MODE_OUTPUT) {
			WRITE_BIT(tmp, i, data->output[i]);
		}
	}

	*value = tmp;

	return 0;
}

static int extio2_gpio_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					   gpio_port_value_t value)
{
	const struct extio2_gpio_config *cfg = port->config;
	struct extio2_gpio_data *data = port->data;

	for (int i = 0; i < EXTIO2_IO_COUNT; i++) {
		if (data->mode[i] == EXTIO2_MODE_OUTPUT && IS_BIT_SET(mask, i)) {
			data->output[i] = !!IS_BIT_SET(value, i);
		}
	}

	return i2c_burst_write_dt(&cfg->i2c, EXTIO2_REG_OUTPUT_CTRL, data->output, EXTIO2_IO_COUNT);
}

static int extio2_gpio_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct extio2_gpio_config *cfg = port->config;
	struct extio2_gpio_data *data = port->data;

	for (int i = 0; i < EXTIO2_IO_COUNT; i++) {
		if (data->mode[i] == EXTIO2_MODE_OUTPUT && IS_BIT_SET(pins, i)) {
			data->output[i] = 1;
		}
	}

	return i2c_burst_write_dt(&cfg->i2c, EXTIO2_REG_OUTPUT_CTRL, data->output, EXTIO2_IO_COUNT);
}

static int extio2_gpio_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct extio2_gpio_config *cfg = port->config;
	struct extio2_gpio_data *data = port->data;

	for (int i = 0; i < EXTIO2_IO_COUNT; i++) {
		if (data->mode[i] == EXTIO2_MODE_OUTPUT && IS_BIT_SET(pins, i)) {
			data->output[i] = 0;
		}
	}

	return i2c_burst_write_dt(&cfg->i2c, EXTIO2_REG_OUTPUT_CTRL, data->output, EXTIO2_IO_COUNT);
}

static int extio2_gpio_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct extio2_gpio_config *cfg = port->config;
	struct extio2_gpio_data *data = port->data;

	for (int i = 0; i < EXTIO2_IO_COUNT; i++) {
		if (data->mode[i] == EXTIO2_MODE_OUTPUT && IS_BIT_SET(pins, i)) {
			data->output[i] ^= 1;
		}
	}

	return i2c_burst_write_dt(&cfg->i2c, EXTIO2_REG_OUTPUT_CTRL, data->output, EXTIO2_IO_COUNT);
}

static int extio2_gpio_init(const struct device *port)
{
	const struct extio2_gpio_config *cfg = port->config;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	return 0;
}

static DEVICE_API(gpio, extio2_gpio_api) = {
	.pin_configure = extio2_gpio_pin_configure,
	.port_get_raw = extio2_gpio_port_get_raw,
	.port_set_masked_raw = extio2_gpio_port_set_masked_raw,
	.port_set_bits_raw = extio2_gpio_port_set_bits_raw,
	.port_clear_bits_raw = extio2_gpio_port_clear_bits_raw,
	.port_toggle_bits = extio2_gpio_port_toggle_bits,
};

#define EXTIO2_GPIO_INIT(n)                                                                        \
	static const struct extio2_gpio_config extio2_gpio_cfg_##n = {                             \
		.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n)},                   \
		.i2c = I2C_DT_SPEC_GET(DT_INST_PARENT(n)),                                         \
	};                                                                                         \
                                                                                                   \
	static struct extio2_gpio_data extio2_gpio_data_##n;                                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, extio2_gpio_init, NULL, &extio2_gpio_data_##n,                    \
			      &extio2_gpio_cfg_##n, POST_KERNEL,                                   \
			      CONFIG_GPIO_M5STACK_INIT_PRIORITY, &extio2_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(EXTIO2_GPIO_INIT)
