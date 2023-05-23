/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT luat_air105_sysctrl

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/air105.h>

#include <zephyr/dt-bindings/clock/air105-clocks.h>

#include <luat/air105_sysctrl.h>

#define IS_APB_PERIPH(clock_gate) ((clock_gate >> 5) == AIR105_PERIPH_APB)
#define IS_AHB_PERIPH(clock_gate) ((clock_gate >> 5) == AIR105_PERIPH_AHB)

#define GATE_CTRL_BIT(clock_gate) BIT((clock_gate) & (BIT_MASK(5)))

static SYSCTRL_TypeDef *regs = (SYSCTRL_TypeDef *)DT_INST_REG_ADDR(0);

static int clock_control_air105_on(const struct device *dev, clock_control_subsys_t sys)
{
	struct air105_clock_config *cfg = (struct air105_clock_config *)sys;

	if (IS_APB_PERIPH(cfg->gate)) {
		regs->CG_CTRL1 |= GATE_CTRL_BIT(cfg->gate);
		regs->SOFT_RST1 |= GATE_CTRL_BIT(cfg->gate);
	} else if (IS_AHB_PERIPH(cfg->gate)) {
		regs->CG_CTRL2 |= GATE_CTRL_BIT(cfg->gate);
		regs->SOFT_RST2 |= GATE_CTRL_BIT(cfg->gate);
	} else {
		return -EINVAL;
	}

	return 0;
}

static int clock_control_air105_off(const struct device *dev, clock_control_subsys_t sys)
{
	struct air105_clock_config *cfg = (struct air105_clock_config *)sys;

	if (IS_APB_PERIPH(cfg->gate)) {
		regs->CG_CTRL1 &= ~GATE_CTRL_BIT(cfg->gate);
		regs->SOFT_RST1 |= GATE_CTRL_BIT(cfg->gate);
	} else if (IS_AHB_PERIPH(cfg->gate)) {
		regs->CG_CTRL2 &= ~GATE_CTRL_BIT(cfg->gate);
		regs->SOFT_RST2 |= GATE_CTRL_BIT(cfg->gate);
	} else {
		return -EINVAL;
	}

	return 0;
}

static int clock_control_air105_get_rate(const struct device *dev, clock_control_subsys_t sys,
					 uint32_t *rate)
{
	struct air105_clock_config *cfg = (struct air105_clock_config *)sys;

	if (cfg->type == AIR105_HCLK) {
		*rate = regs->HCLK_1MS_VAL * 1000;
	} else if (cfg->type == AIR105_PCLK) {
		*rate = regs->PCLK_1MS_VAL * 1000;
	} else {
		return -EINVAL;
	}

	return 0;
}

static enum clock_control_status clock_control_air105_get_status(const struct device *dev,
								 clock_control_subsys_t sys)
{
	struct air105_clock_config *cfg = (struct air105_clock_config *)sys;
	uint32_t regval;

	if (IS_APB_PERIPH(cfg->gate)) {
		regval = regs->CG_CTRL1;
	} else if (IS_AHB_PERIPH(cfg->gate)) {
		regval = regs->CG_CTRL2;
	} else {
		return -CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	if (regval & GATE_CTRL_BIT(cfg->gate)) {
		return CLOCK_CONTROL_STATUS_ON;
	} else {
		return CLOCK_CONTROL_STATUS_OFF;
	}
}

static int clock_control_air105_init(const struct device *dev)
{
	uint32_t regval = regs->FREQ_SEL;

	/* Select clock source */
	regval &= ~SYSCTRL_FREQ_SEL_CLOCK_SOURCE_Mask;
	regval |= (DT_INST_ENUM_IDX(0, clock_source)) << SYSCTRL_FREQ_SEL_CLOCK_SOURCE_Pos;

	/* Configure PLL */
	regval &= ~SYSCTRL_FREQ_SEL_XTAL_Mask;
	regval |= (DT_INST_ENUM_IDX(0, clock_frequency) + 0x08) << SYSCTRL_FREQ_SEL_XTAL_Pos;

	/* Configure FCLK */
	regval &= ~SYSCTRL_FREQ_SEL_PLL_DIV_Mask;
	regval |= (DT_INST_ENUM_IDX(0, pll_divider)) << SYSCTRL_FREQ_SEL_PLL_DIV_Pos;

	/* Configure HCLK */
	regval &= ~SYSCTRL_FREQ_SEL_HCLK_DIV_Mask;
	regval |= (DT_INST_ENUM_IDX(0, hclk_divider)) << SYSCTRL_FREQ_SEL_HCLK_DIV_Pos;

	/* Configure PCLK */
	regval &= ~SYSCTRL_FREQ_SEL_PCLK_DIV_Mask;
	regval |= (DT_INST_ENUM_IDX(0, pclk_divider)) << SYSCTRL_FREQ_SEL_PCLK_DIV_Pos;

	regs->FREQ_SEL = regval;

	return 0;
}

static struct clock_control_driver_api clock_control_air105_api = {
	.on = clock_control_air105_on,
	.off = clock_control_air105_off,
	.get_rate = clock_control_air105_get_rate,
	.get_status = clock_control_air105_get_status,
};

DEVICE_DT_INST_DEFINE(0, clock_control_air105_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_air105_api);
