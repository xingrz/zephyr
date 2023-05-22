/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT luat_air105_uart

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/air105.h>
#include <zephyr/irq.h>

#include <luat/air105_uart.h>

struct uart_air105_config {
	UART_TypeDef *regs;
	const struct pinctrl_dev_config *pincfg;
	const struct air105_clock_config clkcfg;
	uint32_t baud_rate;
};

struct uart_air105_data {
};

static int uart_air105_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_air105_config *config = dev->config;

	if (!(config->regs->LSR & UART_LINE_STATUS_RX_RECVD)) {
		return -1;
	}

	*c = config->regs->OFFSET_0.RBR & BIT_MASK(8);

	return 0;
}

static void uart_air105_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_air105_config *config = dev->config;

	/* Wait for TX buffer to be empty */
	while (!(config->regs->LSR & UART_LINE_STATUS_TX_EMPTY)) {
	}

	config->regs->OFFSET_0.THR = c & BIT_MASK(8);
}

static int uart_air105_init(const struct device *dev)
{
	const struct uart_air105_config *config = dev->config;
	uint32_t pclk_freq;
	uint32_t baud_rate_div;
	int ret = 0;

	ret = clock_control_on(AIR105_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkcfg);
	if (ret < 0) {
		return ret;
	}

#if defined(CONFIG_PINCTRL)
	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}
#endif

	clock_control_get_rate(AIR105_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkcfg,
			       &pclk_freq);

	baud_rate_div = ((pclk_freq / 16) + (config->baud_rate / 2)) / config->baud_rate;

	config->regs->LCR |= UART_LCR_DLAB;
	config->regs->OFFSET_0.DLL = baud_rate_div & BIT_MASK(8);
	config->regs->OFFSET_4.DLH = (baud_rate_div >> 8) & BIT_MASK(8);
	config->regs->LCR &= ~UART_LCR_DLAB;

	config->regs->LCR = UART_WordLength_8b | UART_StopBits_1 | UART_Parity_No;

	config->regs->MCR &= ~UART_MCR_AFCE;

	return ret;
}

static const struct uart_driver_api uart_air105_api = {
	.poll_in = uart_air105_poll_in,
	.poll_out = uart_air105_poll_out,
};

#define UART_AIR105_INIT(n)                                                                        \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static struct uart_air105_data uart_air105_data_##n = {};                                  \
                                                                                                   \
	static const struct uart_air105_config uart_air105_config_##n = {                          \
		.regs = (UART_TypeDef *)DT_INST_REG_ADDR(n),                                       \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.clkcfg = AIR105_DT_INST_CLOCK_CFG(n),                                             \
		.baud_rate = DT_INST_PROP(n, current_speed),                                       \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, uart_air105_init, NULL, &uart_air105_data_##n,                    \
			      &uart_air105_config_##n, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,  \
			      &uart_air105_api);

DT_INST_FOREACH_STATUS_OKAY(UART_AIR105_INIT)
