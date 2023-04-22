/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HI3861_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HI3861_H_

#define HI3861_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(cldo))

struct hi3861_clock_config {
	uint32_t clock_reg;
	uint32_t freq_sel;
};

#define HI3861_DT_CLOCK_CFG(node_id, clock_id)                                                     \
	{                                                                                          \
		.clock_reg = DT_CLOCKS_CELL_BY_IDX(node_id, clock_id, clock_reg),                  \
		.freq_sel = DT_CLOCKS_CELL_BY_IDX(node_id, clock_id, freq_sel),                    \
	}

#define HI3861_DT_INST_CLOCK_CFG(inst) HI3861_DT_CLOCK_CFG(DT_DRV_INST(inst), 0)

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HI3861_H_ */
