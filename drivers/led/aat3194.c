/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT skyworks_aat3194

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(aat3194, CONFIG_LED_LOG_LEVEL);

struct aat3194_data {
	uint8_t level;
};

struct aat3194_config {
	struct gpio_dt_spec output_gpio;
	uint8_t steps;
	bool decreasing;
};

static int aat3194_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
{
	struct aat3194_data *data = dev->data;
	const struct aat3194_config *config = dev->config;
	int ret;

	if (value > 100) {
		value = 100;
	}

	int level = value * config->steps / 100;
	if (level == data->level) {
		return 0;
	}

	LOG_DBG("Set brightness to %d%% (%d/%d)", value, level, config->steps);

	if (level == 0) {
		// Disable output
		if ((ret = gpio_pin_set_dt(&config->output_gpio, 0)) != 0) {
			return ret;
		}
		k_msleep(3);
	} else {
		if (data->level == 0) {
			data->level = config->decreasing ? config->steps : 1;
			// Enable output
			if ((ret = gpio_pin_set_dt(&config->output_gpio, 1)) != 0) {
				return ret;
			}
			k_usleep(30);
		}

		int pulses;
		if (config->decreasing) {
			int pulses_start = config->steps - data->level;
			int pulses_end = config->steps - level;
			pulses = (config->steps + pulses_end - pulses_start) % config->steps;
		} else {
			pulses = (config->steps + level - data->level) % config->steps;
		}
		LOG_DBG("Level change: %d -> %d (%d pulses)", data->level, level, pulses);

		for (int i = 0; i < pulses; i++) {
			if ((ret = gpio_pin_set_dt(&config->output_gpio, 0)) != 0) {
				return ret;
			}
			if ((ret = gpio_pin_set_dt(&config->output_gpio, 1)) != 0) {
				return ret;
			}
		}
	}
	data->level = level;

	return 0;
}

static int aat3194_on(const struct device *dev, uint32_t led)
{
	struct aat3194_data *data = dev->data;
	if (data->level == 0) {
		return aat3194_set_brightness(dev, led, 100);
	} else {
		return 0;
	}
}

static int aat3194_off(const struct device *dev, uint32_t led)
{
	struct aat3194_data *data = dev->data;
	if (data->level != 0) {
		return aat3194_set_brightness(dev, led, 0);
	} else {
		return 0;
	}
}

static int aat3194_init(const struct device *dev)
{
	const struct aat3194_config *config = dev->config;
	int ret;

	if (!device_is_ready(config->output_gpio.port)) {
		LOG_ERR("Output GPIO device %s is not ready", config->output_gpio.port->name);
		return -ENODEV;
	}

	if ((ret = gpio_pin_configure_dt(&config->output_gpio, GPIO_OUTPUT_LOW)) != 0) {
		LOG_ERR("Failed to configure output GPIO: %d", ret);
		return ret;
	}

	return 0;
}

static const struct led_driver_api aat3194_led_api = {
	.on = aat3194_on,
	.off = aat3194_off,
	.set_brightness = aat3194_set_brightness,
};

#define AAT3194_DEVICE(id)                                                                         \
	static struct aat3194_data aat3194_data_##id = {                                           \
		.level = 0,                                                                        \
	};                                                                                         \
                                                                                                   \
	static const struct aat3194_config aat3194_config_##id = {                                 \
		.output_gpio = GPIO_DT_SPEC_INST_GET(id, output_gpios),                            \
		.steps = DT_INST_PROP(id, steps),                                                  \
		.decreasing = DT_INST_PROP(id, decreasing),                                        \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(id, &aat3194_init, NULL, &aat3194_data_##id, &aat3194_config_##id,   \
			      POST_KERNEL, CONFIG_LED_INIT_PRIORITY, &aat3194_led_api);

DT_INST_FOREACH_STATUS_OKAY(AAT3194_DEVICE)
