/*
 * Copyright (c) 2023-2024 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOC_H__
#define __SOC_H__

#include <zephyr/device.h>

#define CH32V_SYS_BASE DT_REG_ADDR(DT_NODELABEL(syscon))

void ch32v_sys_unlock(void);
void ch32v_sys_relock(void);

#endif /* __SOC_H__ */
