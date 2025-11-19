/*
 * Copyright (c) 2025 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>

#define M5STACK_REG_FIRMWARE_VERSION 0xFE

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(m5exp, CONFIG_MFD_LOG_LEVEL);

struct m5exp_config {
	struct i2c_dt_spec i2c;
	const struct device *vin_supply;
};

static int m5exp_init(const struct device *dev)
{
	const struct m5exp_config *cfg = dev->config;
	uint8_t reg;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("%s: I2C bus not ready", dev->name);
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_REGULATOR) && cfg->vin_supply != NULL) {
		if (!device_is_ready(cfg->vin_supply)) {
			LOG_ERR("%s: VIN supply device not ready", dev->name);
			return -ENODEV;
		}

		ret = regulator_enable(cfg->vin_supply);
		if (ret < 0) {
			LOG_ERR("%s: Failed to enable VIN supply: %d", dev->name, ret);
			return ret;
		}
	}

	ret = i2c_reg_read_byte_dt(&cfg->i2c, M5STACK_REG_FIRMWARE_VERSION, &reg);
	if (ret < 0) {
		LOG_ERR("%s: Failed to read firmware version: %d", dev->name, ret);
		return ret;
	}

	LOG_DBG("%s: Firmware version: %d", dev->name, reg);

	return 0;
}

#define M5EXP_INIT(n, part, bus)                                                                   \
	static const struct m5exp_config m5_##part##_expansion_cfg_##n = {                         \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.vin_supply = DEVICE_DT_GET_OR_NULL(DT_PHANDLE(DT_NODELABEL(bus), vin_supply)),    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, m5exp_init, NULL, NULL, &m5_##part##_expansion_cfg_##n,           \
			      POST_KERNEL, CONFIG_MFD_M5_EXPANSION_INIT_PRIORITY, NULL);

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT m5stack_grove_porta
DT_INST_FOREACH_STATUS_OKAY_VARGS(M5EXP_INIT, grove_porta, grove_header)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT m5stack_mbus
DT_INST_FOREACH_STATUS_OKAY_VARGS(M5EXP_INIT, mbus, m5stack_mbus_header)
