/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_pinctrl

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>

#define GET_GPIO_CONTROLLER(node_id, prop, idx)                                                    \
	DEVICE_DT_GET_OR_NULL(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

const struct device *gpio[] = {DT_INST_FOREACH_PROP_ELEM(0, gpio_controllers, GET_GPIO_CONTROLLER)};

#if defined(CH32V_SYS_R16_PIN_ALTERNATE_REG)
#define PIN_ALTERNATE_REG  CH32V_SYS_R16_PIN_ALTERNATE_REG
#define PIN_ALTERNATE_BITS 16
#define READ_REG           sys_read16
#define WRITE_REG          sys_write16
#elif defined(CH32V_SYS_R8_PIN_ALTERNATE_REG)
#define PIN_ALTERNATE_REG  CH32V_SYS_R8_PIN_ALTERNATE_REG
#define PIN_ALTERNATE_BITS 8
#define READ_REG           sys_read8
#define WRITE_REG          sys_write8
#endif

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t regval;

	if (pin->port >= ARRAY_SIZE(gpio) || !device_is_ready(gpio[pin->port])) {
		return;
	}

	if (pin->remap_bit >= PIN_ALTERNATE_BITS) {
		return;
	}

	if (pin->remap_bit) {
		regval = READ_REG(PIN_ALTERNATE_REG);
		if (pin->remap_en) {
			regval |= BIT(pin->remap_bit);
		} else {
			regval &= ~BIT(pin->remap_bit);
		}
		WRITE_REG(regval, PIN_ALTERNATE_REG);
	}

	gpio_pin_configure(gpio[pin->port], pin->pin, pin->flags);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		pinctrl_configure_pin(&pins[i]);
	}

	return 0;
}
