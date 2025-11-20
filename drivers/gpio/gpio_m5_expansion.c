/*
 * Copyright (c) 2025 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT m5stack_expansion_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(m5exp_gpio, CONFIG_GPIO_LOG_LEVEL);

#define MODE_INPUT  0
#define MODE_OUTPUT 1

struct m5exp_gpio_config {
	struct gpio_driver_config common;
	struct i2c_dt_spec i2c;
	uint8_t reg_mode;
	uint8_t reg_output;
	uint8_t reg_input;
	uint8_t io_count;
	uint8_t *mode;
	uint8_t *output;
	uint8_t *input;
};

struct m5exp_gpio_data {
	struct gpio_driver_data common;
};

static int m5exp_gpio_pin_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct m5exp_gpio_config *cfg = port->config;
	int ret;

	if (flags & (GPIO_OPEN_DRAIN | GPIO_PULL_UP | GPIO_PULL_DOWN)) {
		return -ENOTSUP;
	}

	if (flags & GPIO_INPUT) {
		cfg->mode[pin] = MODE_INPUT;
		ret = i2c_reg_write_byte_dt(&cfg->i2c, cfg->reg_mode + pin, cfg->mode[pin]);
		if (ret < 0) {
			return ret;
		}
	} else if (flags & GPIO_OUTPUT) {
		cfg->mode[pin] = MODE_OUTPUT;
		ret = i2c_reg_write_byte_dt(&cfg->i2c, cfg->reg_mode + pin, cfg->mode[pin]);
		if (ret < 0) {
			return ret;
		}

		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			cfg->output[pin] = 1;
			ret = i2c_reg_write_byte_dt(&cfg->i2c, cfg->reg_output + pin,
						    cfg->output[pin]);
			if (ret < 0) {
				return ret;
			}
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			cfg->output[pin] = 0;
			ret = i2c_reg_write_byte_dt(&cfg->i2c, cfg->reg_output + pin,
						    cfg->output[pin]);
			if (ret < 0) {
				return ret;
			}
		}
	}

	return 0;
}

static int m5exp_gpio_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
	const struct m5exp_gpio_config *cfg = port->config;
	gpio_port_value_t tmp = 0;
	int ret;

	ret = i2c_burst_read_dt(&cfg->i2c, cfg->reg_input, cfg->input, cfg->io_count);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < cfg->io_count; i++) {
		if (cfg->mode[i] == MODE_INPUT) {
			WRITE_BIT(tmp, i, cfg->input[i]);
		} else if (cfg->mode[i] == MODE_OUTPUT) {
			WRITE_BIT(tmp, i, cfg->output[i]);
		}
	}

	*value = tmp;

	return 0;
}

static int m5exp_gpio_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct m5exp_gpio_config *cfg = port->config;

	for (int i = 0; i < cfg->io_count; i++) {
		if (cfg->mode[i] == MODE_OUTPUT && IS_BIT_SET(mask, i)) {
			cfg->output[i] = !!IS_BIT_SET(value, i);
		}
	}

	return i2c_burst_write_dt(&cfg->i2c, cfg->reg_output, cfg->output, cfg->io_count);
}

static int m5exp_gpio_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct m5exp_gpio_config *cfg = port->config;

	for (int i = 0; i < cfg->io_count; i++) {
		if (cfg->mode[i] == MODE_OUTPUT && IS_BIT_SET(pins, i)) {
			cfg->output[i] = 1;
		}
	}

	return i2c_burst_write_dt(&cfg->i2c, cfg->reg_output, cfg->output, cfg->io_count);
}

static int m5exp_gpio_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct m5exp_gpio_config *cfg = port->config;

	for (int i = 0; i < cfg->io_count; i++) {
		if (cfg->mode[i] == MODE_OUTPUT && IS_BIT_SET(pins, i)) {
			cfg->output[i] = 0;
		}
	}

	return i2c_burst_write_dt(&cfg->i2c, cfg->reg_output, cfg->output, cfg->io_count);
}

static int m5exp_gpio_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct m5exp_gpio_config *cfg = port->config;

	for (int i = 0; i < cfg->io_count; i++) {
		if (cfg->mode[i] == MODE_OUTPUT && IS_BIT_SET(pins, i)) {
			cfg->output[i] ^= 1;
		}
	}

	return i2c_burst_write_dt(&cfg->i2c, cfg->reg_output, cfg->output, cfg->io_count);
}

static int m5exp_gpio_init(const struct device *port)
{
	const struct m5exp_gpio_config *cfg = port->config;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	return 0;
}

static DEVICE_API(gpio, m5exp_gpio_api) = {
	.pin_configure = m5exp_gpio_pin_configure,
	.port_get_raw = m5exp_gpio_port_get_raw,
	.port_set_masked_raw = m5exp_gpio_port_set_masked_raw,
	.port_set_bits_raw = m5exp_gpio_port_set_bits_raw,
	.port_clear_bits_raw = m5exp_gpio_port_clear_bits_raw,
	.port_toggle_bits = m5exp_gpio_port_toggle_bits,
};

#define M5EXP_GPIO_INIT(n)                                                                         \
	static uint8_t m5exp_gpio_mode_##n[DT_INST_PROP(n, ngpios)];                               \
	static uint8_t m5exp_gpio_output_##n[DT_INST_PROP(n, ngpios)];                             \
	static uint8_t m5exp_gpio_input_##n[DT_INST_PROP(n, ngpios)];                              \
                                                                                                   \
	static const struct m5exp_gpio_config m5exp_gpio_cfg_##n = {                               \
		.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n)},                   \
		.i2c = I2C_DT_SPEC_GET(DT_INST_PARENT(n)),                                         \
		.reg_mode = DT_INST_REG_ADDR_BY_NAME(n, mode),                                     \
		.reg_output = DT_INST_REG_ADDR_BY_NAME(n, output),                                 \
		.reg_input = DT_INST_REG_ADDR_BY_NAME(n, input),                                   \
		.io_count = DT_INST_PROP(n, ngpios),                                               \
		.mode = m5exp_gpio_mode_##n,                                                       \
		.output = m5exp_gpio_output_##n,                                                   \
		.input = m5exp_gpio_input_##n,                                                     \
	};                                                                                         \
                                                                                                   \
	static struct m5exp_gpio_data m5exp_gpio_data_##n;                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, m5exp_gpio_init, NULL, &m5exp_gpio_data_##n, &m5exp_gpio_cfg_##n, \
			      POST_KERNEL, CONFIG_GPIO_M5_EXPANSION_INIT_PRIORITY,                 \
			      &m5exp_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(M5EXP_GPIO_INIT)
