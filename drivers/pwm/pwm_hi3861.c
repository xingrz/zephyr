/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hisilicon_hi3861_pwm

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/arch/common/sys_io.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm_sam, CONFIG_PWM_LOG_LEVEL);

#define PWM_EN_REG(BASE)    (BASE + 0x00)
#define PWM_START_REG(BASE) (BASE + 0x04)
#define PWM_FREQ_REG(BASE)  (BASE + 0x08)
#define PWM_DUTY_REG(BASE)  (BASE + 0x0c)

struct pwm_hi3861_config {
	uint32_t base;
	const struct pinctrl_dev_config *pcfg;
	uint64_t clock_freq;
	uint16_t prescaler;
};

static int pwm_hi3861_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
				 uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_hi3861_config *config = dev->config;

	ARG_UNUSED(channel);
	ARG_UNUSED(flags);

	LOG_DBG("Set PWM with period %u and pulse %u", period_cycles, pulse_cycles);

	uint32_t freq = period_cycles / config->clock_freq;
	if (freq >= UINT16_MAX) {
		return -ENOTSUP;
	}

	uint32_t duty = pulse_cycles / config->clock_freq;
	if (duty >= UINT16_MAX) {
		return -ENOTSUP;
	}

	sys_write32(1, PWM_EN_REG(config->base));
	sys_write32(freq, PWM_FREQ_REG(config->base));
	sys_write32(duty, PWM_DUTY_REG(config->base));
	sys_write32(1, PWM_START_REG(config->base));

	return 0;
}

static int pwm_hi3861_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					 uint64_t *cycles)
{
	const struct pwm_hi3861_config *config = dev->config;

	ARG_UNUSED(channel);

	*cycles = config->clock_freq * config->prescaler;

	return 0;
}

static int pwm_hi3861_init(const struct device *dev)
{
	const struct pwm_hi3861_config *config = dev->config;
	int ret;

	LOG_DBG("Init PWM device 0x%08x with clock %lluHz, prescaler: %u", config->base,
		config->clock_freq, config->prescaler);

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static const struct pwm_driver_api pwm_hi3861_api = {
	.set_cycles = pwm_hi3861_set_cycles,
	.get_cycles_per_sec = pwm_hi3861_get_cycles_per_sec,
};

#define PWM_HI3861_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct pwm_hi3861_config pwm_hi3861_config_##n = {                            \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock_freq = DT_INST_PROP_BY_PHANDLE(n, clocks, clock_frequency),                 \
		.prescaler = DT_INST_PROP(n, prescaler) & BIT_MASK(16),                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, pwm_hi3861_init, NULL, NULL, &pwm_hi3861_config_##n, POST_KERNEL, \
			      CONFIG_PWM_INIT_PRIORITY, &pwm_hi3861_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_HI3861_INIT)
