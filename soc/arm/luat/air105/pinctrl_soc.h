/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LUAT_AIR105_PINCTRL_SOC_H_
#define _LUAT_AIR105_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#include <zephyr/dt-bindings/pinctrl/air105-pinctrl.h>

struct pinctrl_soc_pin {
	uint32_t pinmux;
	uint8_t bias_disable;
	uint8_t bias_pull_up;
};

typedef struct pinctrl_soc_pin pinctrl_soc_pin_t;

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{                                                                                          \
		.pinmux = DT_PROP_BY_IDX(node_id, prop, idx),                                      \
		.bias_disable = DT_PROP(node_id, bias_disable),                                    \
		.bias_pull_up = DT_PROP(node_id, bias_pull_up),                                    \
	},

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{                                                                                          \
		DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux,    \
				       Z_PINCTRL_STATE_PIN_INIT)                                   \
	}

#endif /* _LUAT_AIR105_PINCTRL_SOC_H_ */
