/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CLOCK_CONTROL_BCM2711_
#define ZEPHYR_DRIVERS_CLOCK_CONTROL_BCM2711_

#include <zephyr/devicetree/clocks.h>
#include <zephyr/dt-bindings/clock/bcm2711-clocks.h>

#define BCM2711_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(clocks))

struct bcm2711_clock_config {
	uint32_t index;
	uint32_t int_bits;
	uint32_t frac_bits;
	uint32_t frequency;
};

#define Z_BCM2711_CLOCK_CFG_INIT(config)                                                           \
	.index = BCM2711_CLOCK_CONFIG_INDEX(config),                                               \
	.int_bits = BCM2711_CLOCK_CONFIG_INT_BITS(config),                                         \
	.frac_bits = BCM2711_CLOCK_CONFIG_FRAC_BITS(config),

#define BCM2711_DT_CLOCK_CFG(node_id, clock_id)                                                    \
	{                                                                                          \
		Z_BCM2711_CLOCK_CFG_INIT(DT_CLOCKS_CELL_BY_IDX(node_id, clock_id, config))         \
			.frequency = DT_CLOCKS_CELL_BY_IDX(node_id, clock_id, frequency),          \
	}

#define BCM2711_DT_INST_CLOCK_CFG(inst) BCM2711_DT_CLOCK_CFG(DT_DRV_INST(inst), 0)

#endif /* ZEPHYR_DRIVERS_CLOCK_CONTROL_BCM2711_ */
