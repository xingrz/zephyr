/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _INCLUDE_ZEPHYR_CLOCK_CONTROL_AIR105_H_
#define _INCLUDE_ZEPHYR_CLOCK_CONTROL_AIR105_H_

#include <zephyr/device.h>

#define AIR105_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(sysctrl))

struct air105_clock_config {
	uint32_t type;
	uint32_t gate;
};

#define AIR105_DT_CLOCK_CFG(node_id, clock_id)                                                     \
	{                                                                                          \
		.type = DT_CLOCKS_CELL_BY_IDX(node_id, clock_id, clock_type),                      \
		.gate = DT_CLOCKS_CELL_BY_IDX(node_id, clock_id, clock_gate),                      \
	}

#define AIR105_DT_INST_CLOCK_CFG(inst) AIR105_DT_CLOCK_CFG(DT_DRV_INST(inst), 0)

#endif /* _INCLUDE_ZEPHYR_CLOCK_CONTROL_AIR105_H_ */
