/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT brcm_bcm2711_spi

#include <zephyr/arch/cpu.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bcm2711.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_bcm2711, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

#define SPI_CS(base)   (base + 0x00)
#define SPI_FIFO(base) (base + 0x04)
#define SPI_CLK(base)  (base + 0x08)
#define SPI_DLEN(base) (base + 0x0C)
#define SPI_LTOH(base) (base + 0x10)
#define SPI_DC(base)   (base + 0x14)

/* CS Register */
#define SPI_CS_CS(n)    ((n) << 0)
#define SPI_CS_CPHA     BIT(2)
#define SPI_CS_CPOL     BIT(3)
#define SPI_CS_CLEAR    BIT(4)
#define SPI_CS_CSPOL    BIT(6)
#define SPI_CS_TA       BIT(7)
#define SPI_CS_DMAEN    BIT(8)
#define SPI_CS_INTD     BIT(9)
#define SPI_CS_INTR     BIT(10)
#define SPI_CS_ADCS     BIT(11)
#define SPI_CS_REN      BIT(12)
#define SPI_CS_LEN      BIT(13)
#define SPI_CS_DONE     BIT(16)
#define SPI_CS_RXD      BIT(17)
#define SPI_CS_TXD      BIT(18)
#define SPI_CS_RXR      BIT(19)
#define SPI_CS_RXF      BIT(20)
#define SPI_CS_CSPOL0   BIT(21)
#define SPI_CS_CSPOL1   BIT(22)
#define SPI_CS_CSPOL2   BIT(23)
#define SPI_CS_DMA_LEN  BIT(24)
#define SPI_CS_LEN_LONG BIT(25)

#define DEV_CFG(dev)  ((const struct spi_bcm2711_config *const)(dev)->config)
#define DEV_DATA(dev) ((struct spi_bcm2711_data *const)(dev)->data)

struct spi_bcm2711_config {
	DEVICE_MMIO_NAMED_ROM(reg_base);

	const struct pinctrl_dev_config *pcfg;
	const struct bcm2711_clock_config ccfg;

	void (*irq_config_func)(void);
};

struct spi_bcm2711_data {
	struct spi_context ctx;

	DEVICE_MMIO_NAMED_RAM(reg_base);
	mem_addr_t base;
};

static int spi_bcm2711_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_bcm2711_config *cfg = DEV_CFG(dev);
	struct spi_bcm2711_data *data = DEV_DATA(dev);
	uint32_t clk_freq;
	uint32_t regval;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode is not supported");
		return -ENOTSUP;
	}

	/* Reset controller */

	regval = 0;
	sys_write32(regval, SPI_CS(data->base));

	/* Set CS bits */

	if (config->operation & SPI_MODE_CPHA) {
		regval |= SPI_CS_CPHA;
	}

	if (config->operation & SPI_MODE_CPOL) {
		regval |= SPI_CS_CPOL;
	}

	if (config->operation & SPI_CS_ACTIVE_HIGH) {
		regval |= SPI_CS_CSPOL;
	}

	sys_write32(regval, SPI_CS(data->base));

	/* Set clock frequency */

	clock_control_get_rate(BCM2711_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->ccfg,
			       &clk_freq);

	regval = clk_freq / config->frequency;
	if (regval > 0xFFFE) {
		LOG_ERR("Clock frequency is too high");
		return -EINVAL;
	}

	sys_write32(regval, SPI_CLK(data->base));

	return 0;
}

static inline void spi_bcm2711_rd_byte(struct spi_bcm2711_data *data)
{
	uint32_t regval;

	regval = sys_read32(SPI_FIFO(data->base));
	if (data->ctx.rx_buf) {
		*data->ctx.rx_buf = (uint8_t)(regval & 0xFF);
		LOG_DBG("Read byte: 0x%02x", regval);
	} else {
		LOG_DBG("Read byte: 0x%02x (SKIP)", regval);
	}

	spi_context_update_rx(&data->ctx, 1, 1);
}

static inline void spi_bcm2711_wr_byte(struct spi_bcm2711_data *data)
{
	uint32_t regval;

	if (data->ctx.tx_buf) {
		regval = (uint32_t)*data->ctx.tx_buf;
		LOG_DBG("Write byte: 0x%02x", regval);
	} else {
		regval = 0x00;
		LOG_DBG("Write byte: 0x%02x (SKIP)", regval);
	}
	sys_write32(regval, SPI_FIFO(data->base));

	spi_context_update_tx(&data->ctx, 1, 1);
}

static int spi_bcm2711_rd_fifo(const struct device *dev)
{
	struct spi_bcm2711_data *data = DEV_DATA(dev);

	uint32_t count = 0;
	LOG_DBG("Request to read all");

	while (spi_context_rx_on(&data->ctx) && (sys_read32(SPI_CS(data->base)) & SPI_CS_RXD)) {
		spi_bcm2711_rd_byte(data);
		count++;
	}

	LOG_DBG("Actually read %d bytes", count);

	return 0;
}

static int spi_bcm2711_wr_fifo(const struct device *dev)
{
	struct spi_bcm2711_data *data = DEV_DATA(dev);

	uint32_t count = 0;
	LOG_DBG("Request to write all");

	while (spi_context_tx_on(&data->ctx) && (sys_read32(SPI_CS(data->base)) & SPI_CS_TXD)) {
		spi_bcm2711_wr_byte(data);
		count++;
	}

	LOG_DBG("Actually wrote %d bytes", count);

	return 0;
}

static int spi_bcm2711_rd_fifo_blind(const struct device *dev, uint32_t limit)
{
	struct spi_bcm2711_data *data = DEV_DATA(dev);

	uint32_t count = 0;
	LOG_DBG("Request to read %d bytes", limit);

	while (spi_context_rx_on(&data->ctx) && limit--) {
		spi_bcm2711_rd_byte(data);
		count++;
	}

	LOG_DBG("Actually read %d bytes", count);

	return 0;
}

static int spi_bcm2711_wr_fifo_blind(const struct device *dev, uint32_t limit)
{
	struct spi_bcm2711_data *data = DEV_DATA(dev);

	uint32_t count = 0;
	LOG_DBG("Request to write %d bytes", limit);

	while (spi_context_tx_on(&data->ctx) && limit--) {
		spi_bcm2711_wr_byte(data);
		count++;
	}

	LOG_DBG("Actually wrote %d bytes", count);

	return 0;
}

static int spi_bcm2711_transceive_impl(const struct device *dev, const struct spi_config *config,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				       void *userdata)
{
	struct spi_bcm2711_data *data = DEV_DATA(dev);
	uint32_t regval;
	int ret;

	spi_context_lock(&data->ctx, false, cb, userdata, config);

	ret = spi_bcm2711_configure(dev, config);
	if (ret < 0) {
		goto exit;
	}

	spi_context_cs_control(&data->ctx, true);

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	/* Start transfer */
	regval = sys_read32(SPI_CS(data->base));
	regval |= (SPI_CS_INTR | SPI_CS_INTD | SPI_CS_TA);
	sys_write32(regval, SPI_CS(data->base));

	// ret = spi_context_wait_for_completion(&data->ctx);
	/* Keep going even if ret < 0 */

	spi_context_cs_control(&data->ctx, false);

exit:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_bcm2711_transceive(const struct device *dev, const struct spi_config *config,
				  const struct spi_buf_set *tx_bufs,
				  const struct spi_buf_set *rx_bufs)
{
	return spi_bcm2711_transceive_impl(dev, config, tx_bufs, rx_bufs, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_bcm2711_transceive_async(const struct device *dev, const struct spi_config *config,
					const struct spi_buf_set *tx_bufs,
					const struct spi_buf_set *rx_bufs, spi_callback_t cb,
					void *userdata)
{
	return spi_bcm2711_transceive_impl(dev, config, tx_bufs, rx_bufs, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_bcm2711_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_bcm2711_data *data = DEV_DATA(dev);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static void spi_bcm2711_isr(const struct device *dev)
{
	struct spi_bcm2711_data *data = DEV_DATA(dev);
	uint32_t regval;

	regval = sys_read32(SPI_CS(data->base));
	LOG_DBG("SPI ISR: 0x%08x (RXF: %d, RXR: %d, DONE: %d)", regval, !!(regval & SPI_CS_RXF),
		!!(regval & SPI_CS_RXR), !!(regval & SPI_CS_DONE));

	/* Read based on status of RX FIFO */
	if (regval & SPI_CS_RXF) {
		spi_bcm2711_rd_fifo_blind(dev, 64);
	} else if (regval & SPI_CS_RXR) {
		spi_bcm2711_rd_fifo_blind(dev, 48);
	}

	/* Fill TX FIFO */
	if (regval & SPI_CS_DONE) {
		spi_bcm2711_wr_fifo_blind(dev, 64);

		/* Clear TX status */
		regval = sys_read32(SPI_CS(data->base));
		regval &= ~(SPI_CS_TA);
		sys_write32(regval, SPI_CS(data->base));
	}

	/* Drain RX FIFO */
	spi_bcm2711_rd_fifo(dev);

	/* Drain TX FIFO */
	spi_bcm2711_wr_fifo(dev);

	LOG_DBG("ISR processed");

	/* Transfer completed */
	if (!spi_context_rx_on(&data->ctx)) {
		regval = sys_read32(SPI_CS(data->base));
		regval &= ~(SPI_CS_INTR | SPI_CS_INTD);
		sys_write32(regval, SPI_CS(data->base));

		spi_context_complete(&data->ctx, dev, 0);

		LOG_DBG("Transfer complete");
	}
}

static int spi_bcm2711_init(const struct device *dev)
{
	const struct spi_bcm2711_config *cfg = DEV_CFG(dev);
	struct spi_bcm2711_data *data = DEV_DATA(dev);
	int ret;

	DEVICE_MMIO_NAMED_MAP(dev, reg_base, K_MEM_CACHE_NONE);
	data->base = DEVICE_MMIO_NAMED_GET(dev, reg_base);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_on(BCM2711_CLOCK_CONTROLLER, (clock_control_subsys_t *)&cfg->ccfg);
	if (ret < 0) {
		return ret;
	}

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

	cfg->irq_config_func();

	spi_context_unlock_unconditionally(&data->ctx);

	LOG_DBG("Inited SPI device at 0x%08lx", data->base);

	return 0;
}

const struct spi_driver_api spi_bcm2711_api = {
	.transceive = spi_bcm2711_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_bcm2711_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
	.release = spi_bcm2711_release,
};

#define SPI_BCM2711_INST(n)                                                                        \
	static struct spi_bcm2711_data spi_bcm2711_data_##n = {                                    \
		SPI_CONTEXT_INIT_LOCK(spi_bcm2711_data_##n, ctx),                                  \
		SPI_CONTEXT_INIT_SYNC(spi_bcm2711_data_##n, ctx),                                  \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static void spi_bcm2711_config_func_##n(void)                                              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), spi_bcm2711_isr,            \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct spi_bcm2711_config spi_bcm2711_config_##n = {                          \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)),                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.ccfg = BCM2711_DT_INST_CLOCK_CFG(n),                                              \
		.irq_config_func = spi_bcm2711_config_func_##n,                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &spi_bcm2711_init, NULL, &spi_bcm2711_data_##n,                   \
			      &spi_bcm2711_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,      \
			      &spi_bcm2711_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_BCM2711_INST)
