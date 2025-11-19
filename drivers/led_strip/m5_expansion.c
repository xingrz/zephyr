/*
 * Copyright (c) 2025 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT m5stack_expansion_led_strip

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/dt-bindings/led/led.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(m5exp_strip, CONFIG_LED_STRIP_LOG_LEVEL);

struct m5exp_strip_config {
	struct i2c_dt_spec i2c;
	uint8_t reg_brightness;
	uint8_t reg_pixels;
	uint32_t pixel_stride;
	uint32_t pixel_offset;
	uint32_t pixel_count;
	uint8_t *pixel_buf;
	uint8_t color_count;
	const uint8_t *color_mapping;
};

static int m5exp_strip_update_rgb(const struct device *dev, struct led_rgb *pixels,
				  size_t num_pixels)
{
	const struct m5exp_strip_config *cfg = dev->config;
	uint8_t *pixel = cfg->pixel_buf;
	uint8_t reg;
	int ret;

	if (num_pixels > cfg->pixel_count) {
		return -EINVAL;
	}

	for (size_t i = 0; i < num_pixels; i++) {
		reg = cfg->reg_pixels + i * cfg->pixel_stride + cfg->pixel_offset;

		for (uint8_t c = 0; c < cfg->color_count; c++) {
			switch (cfg->color_mapping[c]) {
			case LED_COLOR_ID_RED:
				pixel[c] = pixels[i].r;
				break;
			case LED_COLOR_ID_GREEN:
				pixel[c] = pixels[i].g;
				break;
			case LED_COLOR_ID_BLUE:
				pixel[c] = pixels[i].b;
				break;
			default:
				return -EINVAL;
			}
		}

		ret = i2c_burst_write_dt(&cfg->i2c, reg, pixel, cfg->color_count);
		if (ret < 0) {
			LOG_ERR("%s: Update pixel %d failed: %d", dev->name, i, ret);
			return ret;
		}
	}

	return 0;
}

static size_t m5exp_strip_length(const struct device *dev)
{
	const struct m5exp_strip_config *cfg = dev->config;

	return cfg->pixel_count;
}

static int m5exp_strip_init(const struct device *dev)
{
	const struct m5exp_strip_config *cfg = dev->config;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("%s: I2C bus not ready", dev->name);
		return -ENODEV;
	}

	if (cfg->pixel_stride < cfg->color_count) {
		LOG_ERR("%s: pixel stride (%d) is less than color count (%d)", dev->name,
			cfg->pixel_stride, cfg->color_count);
		return -EINVAL;
	}

	if (cfg->pixel_offset + cfg->color_count > cfg->pixel_stride) {
		LOG_ERR("%s: pixel offset (%d) + color count (%d) exceeds pixel stride (%d)",
			dev->name, cfg->pixel_offset, cfg->color_count, cfg->pixel_stride);
		return -EINVAL;
	}

	for (int i = 0; i < cfg->color_count; i++) {
		switch (cfg->color_mapping[i]) {
		case LED_COLOR_ID_RED:
		case LED_COLOR_ID_GREEN:
		case LED_COLOR_ID_BLUE:
			break;
		default:
			LOG_ERR("%s: Invalid channel %d to color mapping", dev->name,
				cfg->color_mapping[i]);
			return -EINVAL;
		}
	}

	/* Set all LEDs to maximum brightness if needed */
	if (cfg->reg_brightness != UINT32_MAX) {
		for (int i = 0; i < cfg->pixel_count; i++) {
			ret = i2c_reg_write_byte_dt(&cfg->i2c, cfg->reg_brightness + i, 255);
			if (ret < 0) {
				LOG_ERR("%s: Set brightness for pixel %d failed: %d", dev->name, i,
					ret);
				return ret;
			}
		}
	}

	return 0;
}

static DEVICE_API(led_strip, m5exp_strip_api) = {
	.update_rgb = m5exp_strip_update_rgb,
	.length = m5exp_strip_length,
};

#define M5EXP_STRIP_INIT(n)                                                                        \
	static uint8_t m5exp_strip_pixel_buf_##n[DT_INST_PROP_LEN(n, color_mapping)];              \
                                                                                                   \
	static const uint8_t m5exp_strip_rgb_color_mapping_##n[] = DT_INST_PROP(n, color_mapping); \
                                                                                                   \
	static const struct m5exp_strip_config m5exp_strip_cfg_##n = {                             \
		.i2c = I2C_DT_SPEC_GET(DT_INST_PARENT(n)),                                         \
		.reg_brightness = DT_INST_REG_ADDR_BY_NAME_OR(n, brightness, UINT32_MAX),          \
		.reg_pixels = DT_INST_REG_ADDR_BY_NAME_OR(n, pixels, DT_INST_REG_ADDR(n)),         \
		.pixel_stride = DT_INST_PROP_OR(n, pixel_stride, 3),                               \
		.pixel_offset = DT_INST_PROP_OR(n, pixel_offset, 0),                               \
		.pixel_count = DT_INST_PROP(n, chain_length),                                      \
		.pixel_buf = m5exp_strip_pixel_buf_##n,                                            \
		.color_count = DT_INST_PROP_LEN(n, color_mapping),                                 \
		.color_mapping = m5exp_strip_rgb_color_mapping_##n,                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, m5exp_strip_init, NULL, NULL, &m5exp_strip_cfg_##n, POST_KERNEL,  \
			      CONFIG_LED_STRIP_M5_EXPANSION_INIT_PRIORITY, &m5exp_strip_api);

DT_INST_FOREACH_STATUS_OKAY(M5EXP_STRIP_INIT)
