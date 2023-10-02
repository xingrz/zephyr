/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT brcm_bcm2711_cprman

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bcm2711.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_bcm2711, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#define CM_CTL(base, n) (base + (0x8 * n) + 0x0)
#define CM_DIV(base, n) (base + (0x8 * n) + 0x4)

#define CM_PASSWD (0x5a << 24)

/* CTL Register */
#define CM_CTL_SRC(n)  (n << 0)
#define CM_CTL_ENAB    (1 << 4)
#define CM_CTL_KILL    (1 << 5)
#define CM_CTL_GATE    (1 << 6)
#define CM_CTL_BUSY    (1 << 7)
#define CM_CTL_FLIP    (1 << 8)
#define CM_CTL_MASH(n) (n << 9)

/* Only oscillator is supported */
#define CM_CTL_SRC_OSCILLATOR (1)

/* DIV Register */
#define CM_DIV_DIVF_P  (0)
#define CM_DIV_DIVF(n) (n << CM_DIV_DIVF_P)
#define CM_DIV_DIVI_P  (12)
#define CM_DIV_DIVI(n) (n << CM_DIV_DIVI_P)

DEVICE_MMIO_TOPLEVEL_STATIC(clock_control_bcm2711, DT_DRV_INST(0));

#define BCM2711_SOURCE_ID   (CM_CTL_SRC_OSCILLATOR)
#define BCM2711_SOURCE_FREQ (DT_INST_PROP_BY_PHANDLE(0, clocks, clock_frequency))

#define BCM2711_ACTUAL_FREQ(divi, divf)                                                            \
	(uint32_t)((float)(BCM2711_SOURCE_FREQ) / ((float)(divi) + ((float)divf) / 1024))

static inline void clock_control_bcm2711_write(uint32_t regval, uint32_t reg)
{
	sys_write32(regval | CM_PASSWD, reg);
}

static inline void clock_control_bcm2711_wait_busy(uint32_t reg)
{
	while (sys_read32(reg) & CM_CTL_BUSY) {
	}
}

static int clock_control_bcm2711_on(const struct device *dev, clock_control_subsys_t sys)
{
	mem_addr_t base = DEVICE_MMIO_TOPLEVEL_GET(clock_control_bcm2711);
	const struct bcm2711_clock_config *ccfg = (const struct bcm2711_clock_config *)sys;
	uint32_t regval;
	uint32_t divi, divf;
	float div;

	ARG_UNUSED(dev);

	LOG_DBG("Request for clock %u (int_bits: %u, frac_bits: %u) with frequency %u Hz",
		ccfg->index, ccfg->int_bits, ccfg->frac_bits, ccfg->frequency);

	/* VPU clock is always enabled and can't be configured */
	if (ccfg->index == BCM2711_CLOCK_CONFIG_INDEX(BCM2711_CLOCK_VPU)) {
		return 0;
	}

	if (ccfg->frequency > BCM2711_SOURCE_FREQ) {
		LOG_ERR("Requested frequency (%u Hz) is higher than source clock (%u Hz)",
			ccfg->frequency, BCM2711_SOURCE_FREQ);
		return -EINVAL;
	}

	/* Calculate divi and divf */
	div = (float)BCM2711_SOURCE_FREQ / ccfg->frequency;
	divi = (uint32_t)div;
	divf = (uint32_t)((div - divi) * 1024);

	if (divi > BIT_MASK(ccfg->int_bits)) {
		LOG_ERR("Requested frequency (%u Hz) is too low to be divided from source clock "
			"(%u Hz), while the divider is %u exceeding the maximum value %lu",
			ccfg->frequency, BCM2711_SOURCE_FREQ, divi, BIT_MASK(ccfg->int_bits));
		return -EINVAL;
	}

	if (divf > BIT_MASK(ccfg->frac_bits)) {
		divf = BIT_MASK(ccfg->frac_bits);
		LOG_WRN("Requested frequency (%u Hz) can not be divided from source clock (%u Hz) "
			"precisely, while the fraction divider is %u exceeding the maximum value "
			"%lu",
			ccfg->frequency, BCM2711_SOURCE_FREQ, divf, BIT_MASK(ccfg->frac_bits));
	}

	LOG_DBG("Calculated divi: %u, divf: %u, frequency %u Hz / %.4f = %u Hz", divi, divf,
		BCM2711_SOURCE_FREQ, div, BCM2711_ACTUAL_FREQ(divi, divf));

	/* Disable the running clock before doing any changes */
	regval = sys_read32(CM_CTL(base, ccfg->index));
	if (regval & CM_CTL_ENAB) {
		regval &= ~CM_CTL_ENAB;
		clock_control_bcm2711_write(regval, CM_CTL(base, ccfg->index));
		clock_control_bcm2711_wait_busy(CM_CTL(base, ccfg->index));
	}

	/* Set CM_CTL register */
	regval = CM_CTL_SRC(BCM2711_SOURCE_ID) | CM_CTL_MASH(0);
	clock_control_bcm2711_write(regval, CM_CTL(base, ccfg->index));

	/* Set CM_DIV register */
	regval = CM_DIV_DIVF(divf) | CM_DIV_DIVI(divi);
	clock_control_bcm2711_write(regval, CM_DIV(base, ccfg->index));

	/* Finally, enable the clock */
	regval = sys_read32(CM_CTL(base, ccfg->index));
	regval |= CM_CTL_ENAB;
	clock_control_bcm2711_write(regval, CM_CTL(base, ccfg->index));

	LOG_DBG("Enabled clock %u (0x%02x), value: 0x%08x", ccfg->index, CM_CTL(0, ccfg->index),
		regval);

	return 0;
}

static int clock_control_bcm2711_off(const struct device *dev, clock_control_subsys_t sys)
{
	mem_addr_t base = DEVICE_MMIO_TOPLEVEL_GET(clock_control_bcm2711);
	const struct bcm2711_clock_config *ccfg = (const struct bcm2711_clock_config *)sys;
	uint32_t regval;

	ARG_UNUSED(dev);

	/* VPU clock can't be disabled */
	if (ccfg->index == BCM2711_CLOCK_CONFIG_INDEX(BCM2711_CLOCK_VPU)) {
		return -ENOTSUP;
	}

	regval = sys_read32(CM_CTL(base, ccfg->index));
	regval &= ~CM_CTL_ENAB;
	clock_control_bcm2711_write(regval, CM_CTL(base, ccfg->index));
	clock_control_bcm2711_wait_busy(CM_CTL(base, ccfg->index));

	LOG_DBG("Disabled clock %u (0x%02x), value: 0x%08x", ccfg->index, CM_CTL(0, ccfg->index),
		regval);

	return 0;
}

static int clock_control_bcm2711_get_rate(const struct device *dev, clock_control_subsys_t sys,
					  uint32_t *rate)
{
	mem_addr_t base = DEVICE_MMIO_TOPLEVEL_GET(clock_control_bcm2711);
	const struct bcm2711_clock_config *ccfg = (const struct bcm2711_clock_config *)sys;
	uint32_t regval;
	uint32_t divi, divf;

	ARG_UNUSED(dev);

	regval = sys_read32(CM_DIV(base, ccfg->index));

	divi = (regval >> CM_DIV_DIVI_P) & BIT_MASK(ccfg->int_bits);
	divf = (regval >> CM_DIV_DIVF_P) & BIT_MASK(ccfg->frac_bits);

	*rate = BCM2711_ACTUAL_FREQ(divi, divf);

	return 0;
}

static enum clock_control_status clock_control_bcm2711_get_status(const struct device *dev,
								  clock_control_subsys_t sys)
{
	mem_addr_t base = DEVICE_MMIO_TOPLEVEL_GET(clock_control_bcm2711);
	const struct bcm2711_clock_config *ccfg = (const struct bcm2711_clock_config *)sys;
	uint32_t regval;

	ARG_UNUSED(dev);

	/* VPU clock is always enabled */
	if (ccfg->index == BCM2711_CLOCK_CONFIG_INDEX(BCM2711_CLOCK_VPU)) {
		return CLOCK_CONTROL_STATUS_ON;
	}

	regval = sys_read32(CM_CTL(base, ccfg->index));

	if (regval & CM_CTL_ENAB) {
		return CLOCK_CONTROL_STATUS_ON;
	} else {
		return CLOCK_CONTROL_STATUS_OFF;
	}
}

static int clock_control_bcm2711_init(const struct device *dev)
{
	DEVICE_MMIO_TOPLEVEL_MAP(clock_control_bcm2711, K_MEM_CACHE_NONE);

	LOG_DBG("Inited clock control device at 0x%08lx",
		DEVICE_MMIO_TOPLEVEL_GET(clock_control_bcm2711));

	return 0;
}

static struct clock_control_driver_api clock_control_bcm2711_api = {
	.on = clock_control_bcm2711_on,
	.off = clock_control_bcm2711_off,
	.get_rate = clock_control_bcm2711_get_rate,
	.get_status = clock_control_bcm2711_get_status,
};

DEVICE_DT_INST_DEFINE(0, clock_control_bcm2711_init, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_bcm2711_api);
