/*
 * Copyright (c) 2023-2024 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOC_H__
#define __SOC_H__

#include <soc_common.h>

#if defined(CONFIG_SOC_CH56X)
#include "syscon_regs_ch56x.h"
#elif defined(CONFIG_SOC_CH57X)
#include "syscon_regs_ch57x.h"
#endif

#endif /* __SOC_H__ */
