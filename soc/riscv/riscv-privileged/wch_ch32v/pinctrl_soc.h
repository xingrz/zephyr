/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_RISCV_CH32V_PINCTRL_SOC_H_
#define ZEPHYR_SOC_RISCV_CH32V_PINCTRL_SOC_H_

#if defined(CONFIG_SOC_CH56X) || \
    defined(CONFIG_SOC_CH57X) || \
    defined(CONFIG_SOC_CH58X) || \
    defined(CONFIG_SOC_CH59X)
#include <pinctrl_soc_ch5xx.h>
#endif

#endif /* ZEPHYR_SOC_RISCV_CH32V_PINCTRL_SOC_H_ */
