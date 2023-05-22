/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT luat_air105_pinctrl

#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/air105-pinctrl.h>

#include <luat/air105_gpio.h>
#undef GPIO

#define PINMUX_PAD(pinmux) ((pinmux >> AIR105_PINMUX_PAD_S) & AIR105_PINMUX_PAD_M)
#define PINMUX_PIN(pinmux) ((pinmux >> AIR105_PINMUX_PIN_S) & AIR105_PINMUX_PIN_M)
#define PINMUX_ALT(pinmux) ((pinmux >> AIR105_PINMUX_ALT_S) & AIR105_PINMUX_ALT_M)

static GPIO_MODULE_TypeDef *regs = (GPIO_MODULE_TypeDef *)DT_INST_REG_ADDR(0);

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t regval;
	uint32_t pad_idx = PINMUX_PAD(pin->pinmux);
	uint32_t pin_idx = PINMUX_PIN(pin->pinmux);
	uint32_t alt_idx = PINMUX_ALT(pin->pinmux);

	regval = regs->ALT[pad_idx];
	regval &= ~(BIT_MASK(2) << (pin_idx * 2));
	regval |= (alt_idx << (pin_idx * 2));
	regs->ALT[pad_idx] = regval;

	if (pin->bias_disable) {
		regs->GPIO[pad_idx].PUE &= ~BIT(pin_idx);
	} else if (pin->bias_pull_up) {
		regs->GPIO[pad_idx].PUE |= BIT(pin_idx);
	}
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		pinctrl_configure_pin(&pins[i]);
	}

	return 0;
}
