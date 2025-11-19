/*
 * Copyright (c) 2025 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT m5stack_expansion_rotary_input

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(m5exp_rotary_input, CONFIG_INPUT_LOG_LEVEL);

struct m5exp_rotary_input_config {
	struct i2c_dt_spec i2c;
	uint8_t reg;
	uint32_t poll_interval_ms;
	uint16_t axis;
};

struct m5exp_rotary_input_data {
	struct k_work_delayable dwork;
	const struct device *self;
};

static void m5exp_rotary_input_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct m5exp_rotary_input_data *data = CONTAINER_OF(dwork, struct m5exp_rotary_input_data, dwork);
	const struct device *dev = data->self;
	const struct m5exp_rotary_input_config *cfg = dev->config;
	int32_t value = 0;
	int ret;

	ret = i2c_burst_read_dt(&cfg->i2c, cfg->reg, (uint8_t *)&value, 4);
	if (ret < 0) {
		LOG_ERR("%s: Failed to read counter: %d", dev->name, ret);
		goto next;
	}

	if (value != 0) {
		input_report_rel(dev, cfg->axis, value, true, K_FOREVER);
	}

next:
	ret = k_work_reschedule(&data->dwork, K_MSEC(cfg->poll_interval_ms));
	if (ret < 0) {
		LOG_ERR("%s: Failed to reschedule work: %d", dev->name, ret);
	}
}

static int m5exp_rotary_input_init(const struct device *dev)
{
	const struct m5exp_rotary_input_config *cfg = dev->config;
	struct m5exp_rotary_input_data *data = dev->data;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("%s: I2C bus not ready", dev->name);
		return -ENODEV;
	}

	data->self = dev;
	k_work_init_delayable(&data->dwork, m5exp_rotary_input_work_handler);

	ret = k_work_reschedule(&data->dwork, K_MSEC(cfg->poll_interval_ms));
	if (ret < 0) {
		LOG_ERR("%s: Failed to schedule work: %d", dev->name, ret);
		return ret;
	}

	return 0;
}

#define M5EXP_ROTARY_INPUT_INIT(n)                                                                 \
	static const struct m5exp_rotary_input_config m5exp_rotary_input_cfg_##n = {               \
		.i2c = I2C_DT_SPEC_GET(DT_INST_PARENT(n)),                                         \
		.reg = DT_INST_REG_ADDR_BY_NAME(n, counter_incr),                                  \
		.poll_interval_ms = DT_INST_PROP(n, poll_interval_ms),                             \
	};                                                                                         \
                                                                                                   \
	static struct m5exp_rotary_input_data m5exp_rotary_input_data_##n;                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, m5exp_rotary_input_init, NULL, &m5exp_rotary_input_data_##n,      \
			      &m5exp_rotary_input_cfg_##n, POST_KERNEL,                            \
			      CONFIG_INPUT_M5_EXPANSION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(M5EXP_ROTARY_INPUT_INIT)
