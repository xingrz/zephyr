/*
 * Copyright (c) 2025 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/mfd/m5stack_unit_bytebutton.h>
#include <zephyr/dt-bindings/led/led.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bytebutton_led_strip, CONFIG_LED_STRIP_LOG_LEVEL);

#define BYTEBUTTON_LED_STRIP_COUNT        9
#define BYTEBUTTON_LED_STRIP_PIXEL_COLORS 3
#define BYTEBUTTON_LED_STRIP_PIXEL_ALIGN  4

struct bytebutton_led_strip_config {
	struct i2c_dt_spec i2c;
	uint8_t num_colors;
	const uint8_t *color_mapping;
};

static int bytebutton_led_strip_update_rgb(const struct device *dev, struct led_rgb *pixels,
					   size_t num_pixels)
{
	const struct bytebutton_led_strip_config *cfg = dev->config;
	uint8_t reg;
	static uint8_t buf[BYTEBUTTON_LED_STRIP_PIXEL_COLORS];
	int ret;

	if (num_pixels > BYTEBUTTON_LED_STRIP_COUNT) {
		return -EINVAL;
	}

	for (size_t i = 0; i < num_pixels; i++) {
		reg = BYTEBUTTON_REG_LED_SELF_RGB888 + i * BYTEBUTTON_LED_STRIP_PIXEL_ALIGN;

		for (uint8_t c = 0; c < cfg->num_colors; c++) {
			switch (cfg->color_mapping[c]) {
			case LED_COLOR_ID_RED:
				buf[c] = pixels[i].r;
				break;
			case LED_COLOR_ID_GREEN:
				buf[c] = pixels[i].g;
				break;
			case LED_COLOR_ID_BLUE:
				buf[c] = pixels[i].b;
				break;
			default:
				return -EINVAL;
			}
		}

		ret = i2c_burst_write_dt(&cfg->i2c, reg, buf, sizeof(buf));
		if (ret < 0) {
			LOG_ERR("I2C write to 0x%02x failed: %d", reg, ret);
			return ret;
		}
	}

	return 0;
}

static size_t bytebutton_led_strip_length(const struct device *dev)
{
	ARG_UNUSED(dev);

	return BYTEBUTTON_LED_STRIP_COUNT;
}

static int bytebutton_led_strip_init(const struct device *dev)
{
	const struct bytebutton_led_strip_config *cfg = dev->config;
	static uint8_t bri_buf[BYTEBUTTON_LED_STRIP_COUNT];
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	for (int i = 0; i < cfg->num_colors; i++) {
		switch (cfg->color_mapping[i]) {
		case LED_COLOR_ID_RED:
		case LED_COLOR_ID_GREEN:
		case LED_COLOR_ID_BLUE:
			break;
		default:
			LOG_ERR("%s: invalid channel to color mapping."
				"Check the color-mapping DT property",
				dev->name);
			return -EINVAL;
		}
	}

	/* Set all LEDs to maximum brightness */
	for (int i = 0; i < BYTEBUTTON_LED_STRIP_COUNT; i++) {
		bri_buf[i] = 255;
	}
	ret = i2c_burst_write_dt(&cfg->i2c, BYTEBUTTON_REG_LED_BRIGHTNESS, bri_buf,
				 sizeof(bri_buf));
	if (ret < 0) {
		LOG_ERR("I2C write to set brightness failed: %d", ret);
		return ret;
	}

	/* Set self mode */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, BYTEBUTTON_REG_LED_SHOW_MODE, 0);
	if (ret < 0) {
		LOG_ERR("I2C write to set LED mode failed: %d", ret);
		return ret;
	}

	return 0;
}

static DEVICE_API(led_strip, bytebutton_led_strip_api) = {
	.update_rgb = bytebutton_led_strip_update_rgb,
	.length = bytebutton_led_strip_length,
};

#define BYTEBUTTON_LED_STRIP_INIT(n, part)                                                         \
	static const uint8_t part##_rgb_color_mapping_##n[] = DT_INST_PROP(n, color_mapping);      \
                                                                                                   \
	static const struct bytebutton_led_strip_config part##_led_strip_cfg_##n = {               \
		.i2c = I2C_DT_SPEC_GET(DT_INST_PARENT(n)),                                         \
		.num_colors = DT_INST_PROP_LEN(n, color_mapping),                                  \
		.color_mapping = part##_rgb_color_mapping_##n,                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, bytebutton_led_strip_init, NULL, NULL, &part##_led_strip_cfg_##n, \
			      POST_KERNEL, CONFIG_LED_STRIP_M5STACK_INIT_PRIORITY,                 \
			      &bytebutton_led_strip_api);

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT m5stack_unit_bytebutton_led_strip
DT_INST_FOREACH_STATUS_OKAY_VARGS(BYTEBUTTON_LED_STRIP_INIT, bytebutton)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT m5stack_unit_byteswitch_led_strip
DT_INST_FOREACH_STATUS_OKAY_VARGS(BYTEBUTTON_LED_STRIP_INIT, byteswitch)
