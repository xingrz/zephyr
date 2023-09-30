/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ARM64_BCM2711_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM64_BCM2711_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/pinctrl/bcm2711-pinctrl.h>

struct pinctrl_soc_pin {
	uint32_t pin;
	uint32_t alt;
};

typedef struct pinctrl_soc_pin pinctrl_soc_pin_t;

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{                                                                                          \
		.pin = BCM2711_PINMUX_GET_PIN(DT_PROP_BY_IDX(node_id, prop, idx)),                 \
		.alt = BCM2711_PINMUX_GET_ALT(DT_PROP_BY_IDX(node_id, prop, idx)),                 \
	},

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{                                                                                          \
		DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux,    \
				       Z_PINCTRL_STATE_PIN_INIT)                                   \
	}

#define BCM2711_PINMUX_GET_PIN(pinmux) (((pinmux) >> BCM2711_PINMUX_PIN_S) & BCM2711_PINMUX_PIN_M)
#define BCM2711_PINMUX_GET_ALT(pinmux) (((pinmux) >> BCM2711_PINMUX_ALT_S) & BCM2711_PINMUX_ALT_M)

#endif /* ZEPHYR_SOC_ARM64_BCM2711_PINCTRL_SOC_H_ */
