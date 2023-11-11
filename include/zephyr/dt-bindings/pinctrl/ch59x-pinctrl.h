/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DT_CH59X_PINCTRL_H_
#define ZEPHYR_DT_CH59X_PINCTRL_H_

#include <zephyr/dt-bindings/dt-util.h>
#include <zephyr/dt-bindings/pinctrl/ch5xx-pinctrl-common.h>

#define CH59X_PINMUX(port, pin) CH5XX_PINMUX(port, pin)

#define CH59X_PINMUX_REMAP(port, pin, func, remap)                                                 \
	CH5XX_PINMUX_REMAP(port, pin, CH59X_PINMUX_REMAP_##func, remap)

#define CH59X_PINMUX_REMAP_TMR0  0
#define CH59X_PINMUX_REMAP_TMR1  1
#define CH59X_PINMUX_REMAP_TMR2  2
#define CH59X_PINMUX_REMAP_UART0 4
#define CH59X_PINMUX_REMAP_UART1 5
#define CH59X_PINMUX_REMAP_UART2 6
#define CH59X_PINMUX_REMAP_SPI0  8
#define CH59X_PINMUX_REMAP_PWMX  10
#define CH59X_PINMUX_REMAP_MODEM 12

#endif /* ZEPHYR_DT_CH59X_PINCTRL_H_ */
