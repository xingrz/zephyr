/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch59x_clkmux

#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>

/* CH32V_SYS_R32_CLK_SYS_CFG_REG */
#define CLK_PLL_DIV_MASK  (BIT_MASK(5) << 0)
#define CLK_PLL_DIV(div)  (div & CLK_PLL_DIV_MASK)
#define CLK_SYS_MOD_MASK  (BIT_MASK(2) << 6)
#define CLK_SYS_MOD_CK32M (0 << 6)
#define CLK_SYS_MOD_PLL   (1 << 6)
#define XT_32M_PWR_EN     BIT(18)
#define XT_FORCE_EN       BIT(19)
#define PLL_PWR_EN        BIT(20)

#define NOPS(n)                                                                                    \
	for (int i = 0; i < n; i++) {                                                              \
		__asm__ volatile("nop");                                                           \
	}

struct ch59x_clkmux_config {
	uint32_t hclk_freq;
};

static int ch59x_clkmux_on(const struct device *dev, clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);

	return 0;
}

static int ch59x_clkmux_off(const struct device *dev, clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);

	return 0;
}

static int ch59x_clkmux_get_rate(const struct device *dev, clock_control_subsys_t sys,
				 uint32_t *rate)
{
	const struct ch59x_clkmux_config *cfg = dev->config;

	ARG_UNUSED(sys);

	*rate = cfg->hclk_freq;

	return 0;
}

static enum clock_control_status ch59x_clkmux_get_status(const struct device *dev,
							 clock_control_subsys_t sys)
{
	return CLOCK_CONTROL_STATUS_ON;
}

static int ch59x_clkmux_init(const struct device *dev)
{
	const struct ch59x_clkmux_config *cfg = dev->config;
	uint32_t regval;
	uint32_t source, divider;

	switch (cfg->hclk_freq) {
	case MHZ(4):
		source = CLK_SYS_MOD_CK32M;
		divider = 8;
		break;
	case KHZ(6400):
		source = CLK_SYS_MOD_CK32M;
		divider = 5;
		break;
	case MHZ(8):
		source = CLK_SYS_MOD_CK32M;
		divider = 4;
		break;
	case MHZ(16):
		source = CLK_SYS_MOD_CK32M;
		divider = 2;
		break;
	case MHZ(24):
		source = CLK_SYS_MOD_PLL;
		divider = 20;
		break;
	case MHZ(32):
		source = CLK_SYS_MOD_PLL;
		divider = 15;
		break;
	case MHZ(48):
		source = CLK_SYS_MOD_PLL;
		divider = 10;
		break;
	case MHZ(60):
		source = CLK_SYS_MOD_PLL;
		divider = 8;
		break;
	default:
		return -EINVAL;
	}

	/* Configure HCLK source and divider */

	ch32v_sys_unlock();
	regval = sys_read32(CH32V_SYS_R32_CLK_SYS_CFG_REG);

	if (source == CLK_SYS_MOD_CK32M) {
		regval &= ~PLL_PWR_EN;
		regval |= XT_32M_PWR_EN;
	} else if (source == CLK_SYS_MOD_PLL) {
		regval &= ~XT_32M_PWR_EN;
		regval |= PLL_PWR_EN;
	}

	regval &= ~CLK_SYS_MOD_MASK;
	regval |= source;
	regval &= ~CLK_PLL_DIV_MASK;
	regval |= CLK_PLL_DIV(divider);

	ch32v_sys_unlock();
	sys_write32(regval, CH32V_SYS_R32_CLK_SYS_CFG_REG);

	ch32v_sys_relock();

	NOPS(4);

	return 0;
}

static const struct clock_control_driver_api ch59x_clkmux_api = {
	.on = ch59x_clkmux_on,
	.off = ch59x_clkmux_off,
	.get_rate = ch59x_clkmux_get_rate,
	.get_status = ch59x_clkmux_get_status,
};

#define CH59X_CLKMUX_INST(n)                                                                       \
	static const struct ch59x_clkmux_config ch59x_clkmux_cfg_##n = {                           \
		.hclk_freq = DT_INST_PROP(n, clock_frequency),                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ch59x_clkmux_init, NULL, NULL, &ch59x_clkmux_cfg_##n,             \
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                    \
			      &ch59x_clkmux_api);

DT_INST_FOREACH_STATUS_OKAY(CH59X_CLKMUX_INST)
