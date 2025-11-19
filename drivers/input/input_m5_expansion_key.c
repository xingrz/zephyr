/*
 * Copyright (c) 2025 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT m5stack_expansion_key_input

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(m5exp_key_input, CONFIG_INPUT_LOG_LEVEL);

struct m5exp_key_input_config {
	struct i2c_dt_spec i2c;
	uint8_t reg;
	uint32_t poll_interval_ms;
	uint32_t key_count;
	const uint16_t *key_codes;
	uint8_t *curr_states;
	uint8_t *next_states;
};

struct m5exp_key_input_data {
	struct k_work_delayable dwork;
	const struct device *self;
};

static int m5exp_key_input_read_states(const struct device *dev, uint8_t *states)
{
	const struct m5exp_key_input_config *cfg = dev->config;
	int ret;

	/* Burst read is not supported for some devices (e.g., Module HMI), so
	 * read key states one by one for maximum compatibility. */
	for (int i = 0; i < cfg->key_count; i++) {
		ret = i2c_reg_read_byte_dt(&cfg->i2c, cfg->reg + i, &states[i]);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static void m5exp_key_input_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct m5exp_key_input_data *data = CONTAINER_OF(dwork, struct m5exp_key_input_data, dwork);
	const struct device *dev = data->self;
	const struct m5exp_key_input_config *cfg = dev->config;
	bool pressed;
	int ret;

	ret = m5exp_key_input_read_states(dev, cfg->next_states);
	if (ret < 0) {
		LOG_ERR("%s: Failed to read button states: %d", dev->name, ret);
		goto next;
	}

	for (int i = 0; i < cfg->key_count; i++) {
		if (cfg->key_codes[i] == 0) {
			/* Ignore key without key code mapping */
			continue;
		}

		if (cfg->curr_states[i] == cfg->next_states[i]) {
			continue;
		}

		pressed = !cfg->next_states[i];
		input_report_key(dev, cfg->key_codes[i], pressed, true, K_FOREVER);
	}

	memcpy(cfg->curr_states, cfg->next_states, cfg->key_count);

next:
	ret = k_work_reschedule(&data->dwork, K_MSEC(cfg->poll_interval_ms));
	if (ret < 0) {
		LOG_ERR("%s: Failed to reschedule work: %d", dev->name, ret);
	}
}

static int m5exp_key_input_init(const struct device *dev)
{
	const struct m5exp_key_input_config *cfg = dev->config;
	struct m5exp_key_input_data *data = dev->data;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("%s: I2C bus not ready", dev->name);
		return -ENODEV;
	}

	data->self = dev;
	k_work_init_delayable(&data->dwork, m5exp_key_input_work_handler);

	/* Fill button_states with initial values */
	ret = m5exp_key_input_read_states(dev, cfg->curr_states);
	if (ret < 0) {
		LOG_ERR("%s: Failed to read initial button states: %d", dev->name, ret);
		return ret;
	}

	ret = k_work_reschedule(&data->dwork, K_MSEC(cfg->poll_interval_ms));
	if (ret < 0) {
		LOG_ERR("%s: Failed to schedule work: %d", dev->name, ret);
		return ret;
	}

	return 0;
}

#define M5EXP_KEY_INPUT_KEY_CODE(node_id) [DT_REG_ADDR(node_id)] = DT_PROP(node_id, zephyr_code)

#define M5EXP_KEY_INPUT_INIT(n)                                                                    \
	static const uint16_t m5exp_key_input_key_codes_##n[] = {                                  \
		DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(n, M5EXP_KEY_INPUT_KEY_CODE, (, ))};         \
                                                                                                   \
	static uint8_t m5exp_key_input_curr_states_##n[DT_INST_REG_SIZE(n)];                       \
	static uint8_t m5exp_key_input_next_states_##n[DT_INST_REG_SIZE(n)];                       \
                                                                                                   \
	static const struct m5exp_key_input_config m5exp_key_input_cfg_##n = {                     \
		.i2c = I2C_DT_SPEC_GET(DT_INST_PARENT(n)),                                         \
		.reg = DT_INST_REG_ADDR(n),                                                        \
		.poll_interval_ms = DT_INST_PROP(n, poll_interval_ms),                             \
		.key_count = DT_INST_REG_SIZE(n),                                                  \
		.key_codes = m5exp_key_input_key_codes_##n,                                        \
		.curr_states = m5exp_key_input_curr_states_##n,                                    \
		.next_states = m5exp_key_input_next_states_##n,                                    \
	};                                                                                         \
                                                                                                   \
	static struct m5exp_key_input_data m5exp_key_input_data_##n;                               \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, m5exp_key_input_init, NULL, &m5exp_key_input_data_##n,            \
			      &m5exp_key_input_cfg_##n, POST_KERNEL,                               \
			      CONFIG_INPUT_M5_EXPANSION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(M5EXP_KEY_INPUT_INIT)
