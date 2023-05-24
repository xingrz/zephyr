/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT luat_air105_flash_controller

#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <luat/air105_qspi.h>

LOG_MODULE_REGISTER(flash_air105, CONFIG_FLASH_LOG_LEVEL);

#include "spi_nor.h"

static QSPI_TypeDef *regs = (QSPI_TypeDef *)DT_INST_REG_ADDR(0);

static const struct flash_parameters flash_air105_parameters = {
	.write_block_size = FLASH_WRITE_BLK_SZ,
	.erase_value = 0xff,
};

static int flash_air105_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	ARG_UNUSED(dev);

	memcpy(data, (const void *)(DT_REG_ADDR(SOC_NV_FLASH_NODE) + offset), len);

	return 0;
}

static int flash_air105_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	uint32_t addr;
	uint32_t size;

	QSPI_CommandTypeDef cmd;
	uint8_t ret;

	ARG_UNUSED(dev);

	cmd.Instruction = QUAD_INPUT_PAGE_PROG_CMD;
	cmd.BusMode = QSPI_BUSMODE_114;
	cmd.CmdFormat = QSPI_CMDFORMAT_CMD8_ADDR24_PDAT;

	for (addr = 0; addr < len; addr += FLASH_WRITE_BLK_SZ) {
		size = MIN(len - addr, FLASH_WRITE_BLK_SZ);

		ret = ROM_QSPI_ProgramPage(&cmd, DMA_Channel_1, offset + addr, size,
					   (uint8_t *)data + addr);
		if (ret != 0) {
			return -1;
		}
	}

	return 0;
}

static int flash_air105_erase(const struct device *dev, off_t offset, size_t size)
{
	uint32_t addr;

	ARG_UNUSED(dev);

	for (addr = 0; addr < size; addr += FLASH_ERASE_BLK_SZ) {
		ROM_QSPI_EraseSector(NULL, offset + addr);
	}

	return 0;
}

static const struct flash_parameters *flash_air105_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_air105_parameters;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout flash_air105_pages_layout = {
	.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) / FLASH_ERASE_BLK_SZ,
	.pages_size = FLASH_ERASE_BLK_SZ,
};

static void flash_air105_page_layout(const struct device *dev,
				     const struct flash_pages_layout **layout, size_t *layout_size)
{
	ARG_UNUSED(dev);

	*layout = &flash_air105_pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_air105_sfdp_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	ARG_UNUSED(dev);

	return -ENOTSUP;
}

static int flash_air105_read_jedec_id(const struct device *dev, uint8_t *id)
{
	uint32_t regval;

	ARG_UNUSED(dev);

	flash_air105_cmd(SPI_NOR_CMD_RDID, QSPI_BUSMODE_111, QSPI_CMDFORMAT_CMD8_RREG24);

	regval = regs->REG_RDATA;

	*(id + 0) = (regval >> 16) & 0xFF;
	*(id + 1) = (regval >> 8) & 0xFF;
	*(id + 2) = (regval >> 0) & 0xFF;

	return 0;
}
#endif /* CONFIG_FLASH_JESD216_API */

static int flash_air105_init(const struct device *dev)
{
	uint32_t regval;
	uint32_t latency_cycs;

	regval = regs->DEVICE_PARA;

	latency_cycs = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC * 2 / 1000000;
	regval &= ~(BIT_MASK(16) << 16);
	regval |= (latency_cycs << 16);

	regval &= ~(BIT_MASK(8));
	regval |= QUADSPI_DEVICE_PARA_FREQ_SEL;
	regval |= QUADSPI_DEVICE_PARA_FLASH_READY;
	regval |= (0x06 << 4); // dummy_cycles
	regval |= QUADSPI_DEVICE_PARA_SAMPLE_PHA;

	regs->DEVICE_PARA = regval;

	return 0;
}

static const struct flash_driver_api flash_air105_api = {
	.read = flash_air105_read,
	.write = flash_air105_write,
	.erase = flash_air105_erase,
	.get_parameters = flash_air105_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_air105_page_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = flash_air105_sfdp_read,
	.read_jedec_id = flash_air105_read_jedec_id,
#endif /* CONFIG_FLASH_JESD216_API */
};

DEVICE_DT_INST_DEFINE(0, flash_air105_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_FLASH_INIT_PRIORITY, &flash_air105_api);
