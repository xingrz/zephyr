/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT brcm_bcm2711_pinctrl

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pinctrl_bcm2711, CONFIG_PINCTRL_LOG_LEVEL);

#define GPIO_REG_GROUP(n, cnt)       (n / cnt)
#define GPIO_REG_SHIFT(n, cnt, bits) ((n % cnt) * bits)

#define GPFSEL(base, n) (base + 0x00 + 0x04 * n)

#define FSEL_GROUPS (10)
#define FSEL_BITS   (3)

DEVICE_MMIO_TOPLEVEL_STATIC(pinctrl_bcm2711, DT_DRV_INST(0));

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	mm_reg_t base = DEVICE_MMIO_TOPLEVEL_GET(pinctrl_bcm2711);
	uint32_t group;
	uint32_t shift;
	uint32_t regval;

	group = GPIO_REG_GROUP(pin->pin, FSEL_GROUPS);
	shift = GPIO_REG_SHIFT(pin->pin, FSEL_GROUPS, FSEL_BITS);

	regval = sys_read32(GPFSEL(base, group));
	regval &= ~(BIT_MASK(FSEL_BITS) << shift);
	regval |= (pin->alt << shift);
	sys_write32(regval, GPFSEL(base, group));

	LOG_DBG("Configured pin %d (reg: 0x%02x, bit: %d) to alt 0x%x", pin->pin, GPFSEL(0, group),
		shift, pin->alt);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		pinctrl_configure_pin(&pins[i]);
	}

	return 0;
}

static int pinctrl_bcm2711_init(void)
{
	DEVICE_MMIO_TOPLEVEL_MAP(pinctrl_bcm2711, K_MEM_CACHE_NONE);

	LOG_DBG("Inited pinctrl device at 0x%08lx", DEVICE_MMIO_TOPLEVEL_GET(pinctrl_bcm2711));

	return 0;
}

SYS_INIT(pinctrl_bcm2711_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
