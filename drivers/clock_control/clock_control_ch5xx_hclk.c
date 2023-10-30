/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_hclk

#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/ch5xx.h>

struct ch5xx_hclk_config {
	const struct device *parent;
};

static inline bool clock_dev_config_valid(const struct ch5xx_hclk_config *cfg,
					  const struct ch5xx_clock_dev_config *ccfg)
{
	return ccfg->offset < CH32V_SYS_R8_SLP_CLK_OFF_CNT && ccfg->bit < 8;
}

static int ch5xx_hclk_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct ch5xx_hclk_config *cfg = dev->config;
	const struct ch5xx_clock_dev_config *ccfg = (const struct ch5xx_clock_dev_config *)sys;
	uint32_t regval;

	if (!clock_dev_config_valid(cfg, ccfg)) {
		return -EINVAL;
	}

	ch32v_sys_unlock();
	regval = sys_read8(CH32V_SYS_R8_SLP_CLK_OFF_REG(ccfg->offset));

	regval &= ~BIT(ccfg->bit);

	ch32v_sys_unlock();
	sys_write8(regval, CH32V_SYS_R8_SLP_CLK_OFF_REG(ccfg->offset));

	ch32v_sys_relock();

	return 0;
}

static int ch5xx_hclk_off(const struct device *dev, clock_control_subsys_t sys)
{
	const struct ch5xx_hclk_config *cfg = dev->config;
	const struct ch5xx_clock_dev_config *ccfg = (const struct ch5xx_clock_dev_config *)sys;
	uint32_t regval;

	if (!clock_dev_config_valid(cfg, ccfg)) {
		return -EINVAL;
	}

	ch32v_sys_unlock();
	regval = sys_read8(CH32V_SYS_R8_SLP_CLK_OFF_REG(ccfg->offset));

	regval |= BIT(ccfg->bit);

	ch32v_sys_unlock();
	sys_write8(regval, CH32V_SYS_R8_SLP_CLK_OFF_REG(ccfg->offset));

	ch32v_sys_relock();

	return 0;
}

static int ch5xx_hclk_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	const struct ch5xx_hclk_config *cfg = dev->config;

	ARG_UNUSED(sys);

	return clock_control_get_rate(cfg->parent, 0, rate);
}

static enum clock_control_status ch5xx_hclk_get_status(const struct device *dev,
						       clock_control_subsys_t sys)
{
	const struct ch5xx_hclk_config *cfg = dev->config;
	const struct ch5xx_clock_dev_config *ccfg = (const struct ch5xx_clock_dev_config *)sys;
	uint32_t regval;

	if (!clock_dev_config_valid(cfg, ccfg)) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	ch32v_sys_unlock();
	regval = sys_read8(CH32V_SYS_R8_SLP_CLK_OFF_REG(ccfg->offset));
	ch32v_sys_relock();

	if (regval & BIT(ccfg->bit)) {
		return CLOCK_CONTROL_STATUS_OFF;
	} else {
		return CLOCK_CONTROL_STATUS_ON;
	}
}

static int ch5xx_hclk_init(const struct device *dev)
{
	/* Disable all pheripheral clocks by default */
	for (uint32_t i = 0; i < CH32V_SYS_R8_SLP_CLK_OFF_CNT; i++) {
		ch32v_sys_unlock();
		sys_write8(UINT8_MAX, CH32V_SYS_R8_SLP_CLK_OFF_REG(i));
	}

	ch32v_sys_relock();

	return 0;
}

static const struct clock_control_driver_api ch5xx_hclk_api = {
	.on = ch5xx_hclk_on,
	.off = ch5xx_hclk_off,
	.get_rate = ch5xx_hclk_get_rate,
	.get_status = ch5xx_hclk_get_status,
};

#define CH5XX_HCLK_INST(n)                                                                         \
	static const struct ch5xx_hclk_config ch5xx_hclk_cfg_##n = {                               \
		.parent = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                               \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ch5xx_hclk_init, NULL, NULL, &ch5xx_hclk_cfg_##n, PRE_KERNEL_1,   \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ch5xx_hclk_api);

DT_INST_FOREACH_STATUS_OKAY(CH5XX_HCLK_INST)
