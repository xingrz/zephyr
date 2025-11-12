/*
 * Copyright (c) 2025 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mfd/m5stack_unit_bytebutton.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bytebutton_input, CONFIG_INPUT_LOG_LEVEL);

#define BYTEBUTTON_KEY_COUNT 8

struct bytebutton_input_config {
	struct i2c_dt_spec i2c;
	uint32_t poll_interval_ms;
	uint16_t key_codes[BYTEBUTTON_KEY_COUNT];
};

struct bytebutton_input_data {
	struct k_work_delayable dwork;
	const struct device *self;
	uint8_t button_states;
};

static void bytebutton_input_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct bytebutton_input_data *data =
		CONTAINER_OF(dwork, struct bytebutton_input_data, dwork);
	const struct device *dev = data->self;
	const struct bytebutton_input_config *cfg = dev->config;
	uint8_t states;
	uint8_t change;
	bool pressed;
	int ret;

	ret = i2c_reg_read_byte_dt(&cfg->i2c, BYTEBUTTON_REG_BUTTON_VALUE_BITS, &states);
	if (ret < 0) {
		LOG_ERR("Failed to read button states: %d", ret);
		goto next;
	}

	change = data->button_states ^ states;
	for (int i = 0; change && i < BYTEBUTTON_KEY_COUNT; i++) {
		if ((change & BIT(i)) == 0) {
			continue;
		}

		LOG_DBG("Button %d state changed: %08x", i, (int)(states & BIT(i)));

		if (cfg->key_codes[i] == 0) {
			LOG_DBG("Button %d is not mapped to any key code, ignoring", i);
			continue;
		}

		pressed = !(states & BIT(i));
		input_report_key(dev, cfg->key_codes[i], pressed, true, K_FOREVER);
	}

	data->button_states = states;

next:
	ret = k_work_reschedule(&data->dwork, K_MSEC(cfg->poll_interval_ms));
	if (ret < 0) {
		LOG_ERR("Failed to reschedule work: %d", ret);
	}
}

static int bytebutton_input_init(const struct device *dev)
{
	const struct bytebutton_input_config *cfg = dev->config;
	struct bytebutton_input_data *data = dev->data;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	data->self = dev;
	k_work_init_delayable(&data->dwork, bytebutton_input_work_handler);

	/* Fill button_states with initial values */
	ret = i2c_reg_read_byte_dt(&cfg->i2c, BYTEBUTTON_REG_BUTTON_VALUE_BITS,
				   &data->button_states);
	if (ret < 0) {
		LOG_ERR("Failed to read initial button states: %d", ret);
		return ret;
	}

	ret = k_work_reschedule(&data->dwork, K_MSEC(cfg->poll_interval_ms));
	if (ret < 0) {
		LOG_ERR("Failed to schedule work: %d", ret);
		return ret;
	}

	return 0;
}

#define BYTEBUTTON_KEY_CODE(node_id) [DT_REG_ADDR(node_id)] = DT_PROP(node_id, zephyr_code)

#define BYTEBUTTON_INPUT_INIT(n, part)                                                             \
	static const struct bytebutton_input_config part##_input_cfg_##n = {                       \
		.i2c = I2C_DT_SPEC_GET(DT_INST_PARENT(n)),                                         \
		.poll_interval_ms = DT_INST_PROP(n, poll_interval_ms),                             \
		.key_codes = {DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(n, BYTEBUTTON_KEY_CODE,        \
								    (, ))},                        \
	};                                                                                         \
                                                                                                   \
	static struct bytebutton_input_data part##_input_data_##n;                                 \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, bytebutton_input_init, NULL, &part##_input_data_##n,              \
			      &part##_input_cfg_##n, POST_KERNEL,                                  \
			      CONFIG_INPUT_M5STACK_INIT_PRIORITY, NULL);

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT m5stack_unit_bytebutton_input
DT_INST_FOREACH_STATUS_OKAY_VARGS(BYTEBUTTON_INPUT_INIT, bytebutton)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT m5stack_unit_byteswitch_input
DT_INST_FOREACH_STATUS_OKAY_VARGS(BYTEBUTTON_INPUT_INIT, byteswitch)
