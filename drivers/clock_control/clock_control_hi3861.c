/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hisilicon_hi3861_cldo

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/hi3861.h>
#include <zephyr/arch/common/sys_io.h>

#include <zephyr/dt-bindings/clock/hi3861-clocks.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#define CLDO_CTL_BASE DT_INST_REG_ADDR(0)

#define TCXO_CLK_FREQ DT_INST_PROP_BY_PHANDLE(0, clocks, clock_frequency)

#define HI3861_CLOCK_REG_OFFSET(id) (id >> 4)
#define HI3861_CLOCK_REG_BIT(id)    (id & BIT_MASK(4))

#define HI3861_CLOCK_BIT_DMA_WBUS BIT(1)
#define HI3861_CLOCK_BIT_MONITOR  BIT(6)

#define HI3861_CLOCK_BIT_PWM     BIT(5)
#define HI3861_CLOCK_BIT_PWM_BUS BIT(6)

#define HI3861_CLOCK_BIT_PWM_ALL                                                                   \
	(BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_PWM0)) |                                            \
	 BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_PWM1)) |                                            \
	 BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_PWM2)) |                                            \
	 BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_PWM3)) |                                            \
	 BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_PWM4)) |                                            \
	 BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_PWM5)))

#define HI3861_CLOCK_BIT_I2S_BUS BIT(11)

static int clock_control_hi3861_on(const struct device *dev, clock_control_subsys_t sys)
{
	struct hi3861_clock_config *cfg = (struct hi3861_clock_config *)sys;
	uint32_t regval;

	ARG_UNUSED(dev);

	if (cfg == NULL) {
		LOG_ERR("The clock config can not be NULL.");
		return -ENXIO;
	}

	LOG_DBG("offset: 0x%02x, bit: %ld, freq_sel: %d KHz",
		HI3861_CLOCK_REG_OFFSET(cfg->clock_reg), HI3861_CLOCK_REG_BIT(cfg->clock_reg),
		cfg->freq_sel);

	regval = sys_read32(CLDO_CTL_BASE + HI3861_CLOCK_REG_OFFSET(cfg->clock_reg));
	regval |= BIT(HI3861_CLOCK_REG_BIT(cfg->clock_reg));

	/* For PWM clocks, enable PWM periph clock and bus clock as well */
	if (HI3861_CLOCK_REG_OFFSET(cfg->clock_reg) == CLDO_CTL_CLKEN1_OFFSET) {
		regval |= HI3861_CLOCK_BIT_PWM | HI3861_CLOCK_BIT_PWM_BUS;
	}

	sys_write32(regval, CLDO_CTL_BASE + HI3861_CLOCK_REG_OFFSET(cfg->clock_reg));

	return 0;
}

static int clock_control_hi3861_off(const struct device *dev, clock_control_subsys_t sys)
{
	struct hi3861_clock_config *cfg = (struct hi3861_clock_config *)sys;
	uint32_t regval;

	ARG_UNUSED(dev);

	if (cfg == NULL) {
		LOG_ERR("The clock config can not be NULL.");
		return -ENXIO;
	}

	LOG_DBG("offset: 0x%02x, bit: %ld, freq_sel: %d KHz",
		HI3861_CLOCK_REG_OFFSET(cfg->clock_reg), HI3861_CLOCK_REG_BIT(cfg->clock_reg),
		cfg->freq_sel);

	regval = sys_read32(CLDO_CTL_BASE + HI3861_CLOCK_REG_OFFSET(cfg->clock_reg));
	regval &= ~BIT(HI3861_CLOCK_REG_BIT(cfg->clock_reg));

	/* For PWM clocks, disable PWM periph clock and bus clock if all PWMs are disabled */
	if (HI3861_CLOCK_REG_OFFSET(cfg->clock_reg) == CLDO_CTL_CLKEN1_OFFSET) {
		if ((regval & HI3861_CLOCK_BIT_PWM_ALL) == 0) {
			regval &= ~(HI3861_CLOCK_BIT_PWM | HI3861_CLOCK_BIT_PWM_BUS);
		}
	}

	sys_write32(regval, CLDO_CTL_BASE + HI3861_CLOCK_REG_OFFSET(cfg->clock_reg));

	return 0;
}

static int clock_control_hi3861_get_rate(const struct device *dev, clock_control_subsys_t sys,
					 uint32_t *rate)
{
	struct hi3861_clock_config *cfg = (struct hi3861_clock_config *)sys;

	ARG_UNUSED(dev);

	if (cfg == NULL) {
		LOG_ERR("The clock config can not be NULL.");
		return -ENXIO;
	}

	if (cfg->freq_sel == HI3861_FREQ_TCXO) {
		*rate = TCXO_CLK_FREQ;
	} else {
		*rate = cfg->freq_sel;
	}

	return 0;
}

static enum clock_control_status clock_control_hi3861_get_status(const struct device *dev,
								 clock_control_subsys_t sys)
{
	struct hi3861_clock_config *cfg = (struct hi3861_clock_config *)sys;
	uint32_t regval;
	enum clock_control_status status;

	ARG_UNUSED(dev);

	if (cfg == NULL) {
		LOG_ERR("The clock config can not be NULL.");
		return -ENXIO;
	}

	regval = sys_read32(CLDO_CTL_BASE + HI3861_CLOCK_REG_OFFSET(cfg->clock_reg));
	if (regval & BIT(HI3861_CLOCK_REG_BIT(cfg->clock_reg))) {
		status = CLOCK_CONTROL_STATUS_ON;
	} else {
		status = CLOCK_CONTROL_STATUS_OFF;
	}

	return status;
}

static int clock_control_hi3861_init(const struct device *dev)
{
	uint32_t regval;

	ARG_UNUSED(dev);

	LOG_DBG("Init with TCXO freq: %d Hz", TCXO_CLK_FREQ);

	/* Disable all clocks on init */

	regval = sys_read32(CLDO_CTL_BASE + CLDO_CTL_CLKEN_OFFSET);
	regval &= ~BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_I2C0));
	regval &= ~BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_I2C1));
	regval &= ~BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_SPI0));
	regval &= ~BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_SPI1));
	regval &= ~HI3861_CLOCK_BIT_DMA_WBUS;
	regval &= ~HI3861_CLOCK_BIT_MONITOR;
	regval &= ~BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_TIMER1));
	regval &= ~BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_TIMER2));
	sys_write32(regval, CLDO_CTL_BASE + CLDO_CTL_CLKEN_OFFSET);

	regval = sys_read32(CLDO_CTL_BASE + CLDO_CTL_CLKEN1_OFFSET);
	regval &= ~HI3861_CLOCK_BIT_PWM_ALL;
	regval &= ~HI3861_CLOCK_BIT_PWM;
	regval &= ~HI3861_CLOCK_BIT_PWM_BUS;
	sys_write32(regval, CLDO_CTL_BASE + CLDO_CTL_CLKEN1_OFFSET);

	regval = sys_read32(CLDO_CTL_BASE + CLDO_CTL_CLKEN2_OFFSET);
	regval &= ~BIT(HI3861_CLOCK_REG_BIT(HI3861_CLOCK_I2S));
	regval &= ~HI3861_CLOCK_BIT_I2S_BUS;
	sys_write32(regval, CLDO_CTL_BASE + CLDO_CTL_CLKEN2_OFFSET);

	return 0;
}

static struct clock_control_driver_api clock_control_hi3861_api = {
	.on = clock_control_hi3861_on,
	.off = clock_control_hi3861_off,
	.get_rate = clock_control_hi3861_get_rate,
	.get_status = clock_control_hi3861_get_status,
};

DEVICE_DT_INST_DEFINE(0, clock_control_hi3861_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_hi3861_api);
