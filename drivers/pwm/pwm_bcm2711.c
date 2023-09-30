/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT brcm_bcm2711_pwm

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bcm2711.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm_bcm2711, CONFIG_PWM_LOG_LEVEL);

#define PWM_CTL(base)    (base + 0x00)
#define PWM_STA(base)    (base + 0x04)
#define PWM_DMAC(base)   (base + 0x08)
#define PWM_RNG(base, n) (base + 0x10 + (0x10 * n))
#define PWM_DAT(base, n) (base + 0x14 + (0x10 * n))
#define PWM_FIF1(base)   (base + 0x18)

/* CTL Register */
#define PWM_CTL_PWEN(n) BIT(n * 8 + 0)
#define PWM_CTL_MODE(n) BIT(n * 8 + 1)
#define PWM_CTL_RPTL(n) BIT(n * 8 + 2)
#define PWM_CTL_SBIT(n) BIT(n * 8 + 3)
#define PWM_CTL_POLA(n) BIT(n * 8 + 4)
#define PWM_CTL_USEF(n) BIT(n * 8 + 5)
#define PWM_CTL_CLRF    BIT(6)
#define PWM_CTL_MSEN(n) BIT(n * 8 + 7)

#define PWM_CTL_MASK(n)                                                                            \
	(PWM_CTL_PWEN(n) | PWM_CTL_MODE(n) | PWM_CTL_RPTL(n) | PWM_CTL_SBIT(n) | PWM_CTL_POLA(n) | \
	 PWM_CTL_USEF(n) | PWM_CTL_MSEN(n))

#define DEV_CFG(dev)  ((const struct pwm_bcm2711_config *const)(dev)->config)
#define DEV_DATA(dev) ((struct pwm_bcm2711_data *const)(dev)->data)

struct pwm_bcm2711_config {
	DEVICE_MMIO_NAMED_ROM(reg_base);
	const struct pinctrl_dev_config *pcfg;
	const struct bcm2711_clock_config ccfg;
};

struct pwm_bcm2711_data {
	DEVICE_MMIO_NAMED_RAM(reg_base);
	mem_addr_t base;
};

static int pwm_bcm2711_set_cycles(const struct device *dev, uint32_t channel,
				  uint32_t period_cycles, uint32_t pulse_cycles, pwm_flags_t flags)
{
	struct pwm_bcm2711_data *data = DEV_DATA(dev);
	uint32_t regval;

	if (channel > 1) {
		return -EINVAL;
	}

	/* Reset channel */
	regval = sys_read32(PWM_CTL(data->base));
	regval &= ~PWM_CTL_MASK(channel);
	sys_write32(regval, PWM_CTL(data->base));

	/* Configure period and pulse */
	sys_write32(period_cycles, PWM_RNG(data->base, channel));
	sys_write32(pulse_cycles, PWM_DAT(data->base, channel));

	/* Enable channel */
	regval = sys_read32(PWM_CTL(data->base));
	regval |= PWM_CTL_PWEN(channel);
	sys_write32(regval, PWM_CTL(data->base));

	return 0;
}

static int pwm_bcm2711_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					  uint64_t *cycles)
{
	const struct pwm_bcm2711_config *cfg = DEV_CFG(dev);

	ARG_UNUSED(channel);

	clock_control_get_rate(BCM2711_CLOCK_CONTROLLER, (clock_control_subsys_t *)&cfg->ccfg,
			       (uint32_t *)cycles);

	return 0;
}

static int pwm_bcm2711_init(const struct device *dev)
{
	const struct pwm_bcm2711_config *cfg = DEV_CFG(dev);
	struct pwm_bcm2711_data *data = DEV_DATA(dev);
	int ret;

	DEVICE_MMIO_NAMED_MAP(dev, reg_base, K_MEM_CACHE_NONE);
	data->base = DEVICE_MMIO_NAMED_GET(dev, reg_base);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_on(BCM2711_CLOCK_CONTROLLER, (clock_control_subsys_t *)&cfg->ccfg);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("Inited PWM device at 0x%08lx", data->base);

	return 0;
}

static const struct pwm_driver_api pwm_bcm2711_api = {
	.set_cycles = pwm_bcm2711_set_cycles,
	.get_cycles_per_sec = pwm_bcm2711_get_cycles_per_sec,
};

#define PWM_BCM2711_INST(n)                                                                        \
	static struct pwm_bcm2711_data pwm_bcm2711_data_##n;                                       \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct pwm_bcm2711_config pwm_bcm2711_cfg_##n = {                             \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)),                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.ccfg = BCM2711_DT_INST_CLOCK_CFG(n),                                              \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, pwm_bcm2711_init, NULL, &pwm_bcm2711_data_##n,                    \
			      &pwm_bcm2711_cfg_##n, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,         \
			      &pwm_bcm2711_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_BCM2711_INST)
