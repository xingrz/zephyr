/*
 * Copyright (c) 2025 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT m5stack_unit_extio2

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/mfd/m5stack_unit_extio2.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(extio2_mfd, CONFIG_MFD_LOG_LEVEL);

struct extio2_mfd_config {
	struct i2c_dt_spec i2c;
	const struct device *vin_supply;
};

static int extio2_mfd_init(const struct device *dev)
{
	const struct extio2_mfd_config *cfg = dev->config;
	uint8_t reg;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_REGULATOR) && cfg->vin_supply != NULL) {
		if (!device_is_ready(cfg->vin_supply)) {
			LOG_ERR("VIN supply device not ready");
			return -ENODEV;
		}

		ret = regulator_enable(cfg->vin_supply);
		if (ret < 0) {
			LOG_ERR("Failed to enable VIN supply: %d", ret);
			return ret;
		}
	}

	ret = i2c_reg_read_byte_dt(&cfg->i2c, EXTIO2_REG_FIRMWARE_VERSION, &reg);
	if (ret < 0) {
		LOG_ERR("Failed to read firmware version: %d", ret);
		return ret;
	}

	LOG_DBG("Firmware version: %d", reg);

	return 0;
}

#define GROVE_HEADER_NODE DT_NODELABEL(grove_header)

#define EXTIO2_MFD_INIT(n)                                                                         \
	static const struct extio2_mfd_config extio2_mfd_cfg_##n = {                               \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.vin_supply = DEVICE_DT_GET_OR_NULL(DT_PHANDLE(GROVE_HEADER_NODE, vin_supply)),    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, extio2_mfd_init, NULL, NULL, &extio2_mfd_cfg_##n, POST_KERNEL,    \
			      CONFIG_MFD_M5STACK_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(EXTIO2_MFD_INIT)
