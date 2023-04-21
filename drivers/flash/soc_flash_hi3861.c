/*
 * Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hisilicon_hi3861_flash_controller

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <hi3861/pmu_cmu_ctl.h>
#include <hi3861/cldo_ctl.h>

LOG_MODULE_REGISTER(flash_hi3861, CONFIG_FLASH_LOG_LEVEL);

#include "spi_nor.h"

#define SFC_BASE DT_INST_REG_ADDR(0)

#define SFC_GLOBAL_CONFIG_REG       (SFC_BASE + 0x0100)
#define SFC_TIMING_REG              (SFC_BASE + 0x0110)
#define SFC_INT_RAW_STATUS_REG      (SFC_BASE + 0x0120)
#define SFC_INT_STATUS_REG          (SFC_BASE + 0x0124)
#define SFC_INT_MASK_REG            (SFC_BASE + 0x0128)
#define SFC_INT_CLEAR_REG           (SFC_BASE + 0x012C)
#define SFC_NOSCM_ADDR_START_REG    (SFC_BASE + 0x0150)
#define SFC_NOSCM_ADDR_END_REG      (SFC_BASE + 0x0154)
#define SFC_NOSCM_EN_REG            (SFC_BASE + 0x0158)
#define SFC_VERSION_REG             (SFC_BASE + 0x01F8)
#define SFC_VERSION_SEL_REG         (SFC_BASE + 0x01FC)
#define SFC_BUS_CONFIG1_REG         (SFC_BASE + 0x0200)
#define SFC_BUS_CONFIG2_REG         (SFC_BASE + 0x0204)
#define SFC_BUS_FLASH_SIZE_REG      (SFC_BASE + 0x0210)
#define SFC_BUS_BASE_ADDR_CS0_REG   (SFC_BASE + 0x0214)
#define SFC_BUS_BASE_ADDR_CS1_REG   (SFC_BASE + 0x0218)
#define SFC_BUS_ALIAS_ADDR_REG      (SFC_BASE + 0x021C)
#define SFC_BUS_ALIAS_CS_REG        (SFC_BASE + 0x0220)
#define SFC_BUS_DMA_CTRL_REG        (SFC_BASE + 0x0240)
#define SFC_BUS_DMA_MEM_SADDR_REG   (SFC_BASE + 0x0244)
#define SFC_BUS_DMA_FLASH_SADDR_REG (SFC_BASE + 0x0248)
#define SFC_BUS_DMA_LEN_REG         (SFC_BASE + 0x024C)
#define SFC_BUS_DMA_AHB_CTRL_REG    (SFC_BASE + 0x0250)
#define SFC_CMD_CONFIG_REG          (SFC_BASE + 0x0300)
#define SFC_CMD_INS_REG             (SFC_BASE + 0x0308)
#define SFC_CMD_ADDR_REG            (SFC_BASE + 0x030C)
#define SFC_CMD_DATABUF(n)          (SFC_BASE + 0x0400 + n * 4)

enum MEM_IF_TYPE {
	MEM_IF_STANDARD = 0,
	MEM_IF_DUAL_IN_DUAL_OUT = 1,
	MEM_IF_DUAL_IO = 2,
	MEM_IF_QUAD_IN_QUAD_OUT = 5,
	MEM_IF_QUAD_IO = 6,
};

#define SFC_CONFIG_MEM_IF_TYPE_MASK  GENMASK(19, 17)
#define SFC_CONFIG_MEM_IF_TYPE(type) FIELD_PREP(SFC_CONFIG_MEM_IF_TYPE_MASK, type)
#define SFC_CONFIG_DATA_CNT_MASK     GENMASK(14, 9)
#define SFC_CONFIG_DATA_CNT(nbytes)  FIELD_PREP(SFC_CONFIG_DATA_CNT_MASK, (nbytes - 1))
#define SFC_CONFIG_WRITE             (0)
#define SFC_CONFIG_READ              BIT(8)
#define SFC_CONFIG_DATA_EN           BIT(7)
#define SFC_CONFIG_ADDR_EN           BIT(3)
#define SFC_CONFIG_SEL_CS            BIT(1)
#define SFC_CONFIG_START             BIT(0)

#define PLL2DBB_192M_MASK GENMASK(9, 8)

struct flash_clock_voltage_config {
	uint32_t jedec_id;
	uint8_t freq;
	uint8_t voltage;
};

#define FLASH_FREQ_96M 0
#define FLASH_FREQ_80M 1
#define FLASH_FREQ_60M 2
#define FLASH_FREQ_48M 3

#define FLASH_VOL_1V8 0
#define FLASH_VOL_3V3 1

static const struct flash_clock_voltage_config flash_clock_voltage_configs[] = {
	{0x000000, FLASH_FREQ_80M, FLASH_VOL_3V3},
	{0x1560ef, FLASH_FREQ_80M, FLASH_VOL_1V8}, /* W25Q16JW */
	{0x1540ef, FLASH_FREQ_96M, FLASH_VOL_3V3}, /* W25Q16JL */
	{0x1560c8, FLASH_FREQ_80M, FLASH_VOL_1V8}, /* GD25LE16 */
	{0x1565c8, FLASH_FREQ_96M, FLASH_VOL_3V3}, /* GD25WQ16 */
	{0x15381c, FLASH_FREQ_80M, FLASH_VOL_1V8}, /* EN25S16 */
	{0x15701c, FLASH_FREQ_96M, FLASH_VOL_3V3}, /* EN25QH16 */
	{0x156085, FLASH_FREQ_96M, FLASH_VOL_3V3}, /* P25Q16 */
};

static const struct device *pmu_cmu_ctl = DEVICE_DT_GET(DT_NODELABEL(pmu_cmu_ctl));
static const struct device *cldo_ctl = DEVICE_DT_GET(DT_NODELABEL(cldo_ctl));

static int flash_hi3861_access(uint8_t opcode, uint32_t flags, off_t addr, void *buf, size_t len)
{
	uint32_t regval;

	if (len > 64) {
		return -EINVAL;
	}

	sys_write32(opcode, SFC_CMD_INS_REG);

	if (flags & SFC_CONFIG_ADDR_EN) {
		sys_write32(addr, SFC_CMD_ADDR_REG);
	}

	sys_write32(SFC_CONFIG_MEM_IF_TYPE(MEM_IF_STANDARD) | SFC_CONFIG_SEL_CS | flags |
			    SFC_CONFIG_START,
		    SFC_CMD_CONFIG_REG);

	/* Wait for the command to complete */
	while (sys_read32(SFC_CMD_CONFIG_REG) & SFC_CONFIG_START) {
	}

	if (flags & SFC_CONFIG_DATA_EN) {
		for (size_t i = 0; i < DIV_ROUND_UP(len, sizeof(uint32_t)); i++) {
			regval = sys_read32(SFC_CMD_DATABUF(i));
			((uint32_t *)buf)[i] = regval;
		}
	}

	return 0;
}

#define flash_hi3861_cmd_read(opcode, buf, len)                                                    \
	flash_hi3861_access(opcode,                                                                \
			    SFC_CONFIG_READ | SFC_CONFIG_DATA_EN | SFC_CONFIG_DATA_CNT(len), 0,    \
			    buf, len)

#define flash_hi3861_cmd_addr_read(opcode, addr, buf, len)                                         \
	flash_hi3861_access(opcode,                                                                \
			    SFC_CONFIG_READ | SFC_CONFIG_DATA_EN | SFC_CONFIG_DATA_CNT(len) |      \
				    SFC_CONFIG_ADDR_EN,                                            \
			    addr, buf, len)

#define flash_hi3861_cmd_write(opcode, buf, len)                                                   \
	flash_hi3861_access(opcode,                                                                \
			    SFC_CONFIG_WRITE | SFC_CONFIG_DATA_EN | SFC_CONFIG_DATA_CNT(len), 0,   \
			    buf, len)

#define flash_hi3861_cmd_addr_write(opcode, addr, buf, len)                                        \
	flash_hi3861_access(opcode,                                                                \
			    SFC_CONFIG_WRITE | SFC_CONFIG_DATA_EN | SFC_CONFIG_DATA_CNT(len) |     \
				    SFC_CONFIG_ADDR_EN,                                            \
			    addr, buf, len)

static int flash_hi3861_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	ARG_UNUSED(dev);

	for (off_t page = 0; page < len; page += 64) {
		flash_hi3861_cmd_addr_read(SPI_NOR_CMD_READ, offset + page, (uint8_t *)data + page,
					   MIN(64, len - page));
	}

	return 0;
}

static int flash_hi3861_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	ARG_UNUSED(dev);

	uint32_t status[2];

	flash_hi3861_access(SPI_NOR_CMD_WREN, 0, 0, NULL, 0);
	sys_write32(1, SFC_INT_CLEAR_REG);

	for (off_t page = 0; page < len; page += 64) {
		flash_hi3861_cmd_addr_write(SPI_NOR_CMD_PP, offset + page, (uint8_t *)data + page,
					    MIN(64, len - page));
		sys_write32(1, SFC_INT_CLEAR_REG);
	}

	do {
		flash_hi3861_cmd_read(SPI_NOR_CMD_RDSR, status, sizeof(status));
	} while (status[1] & BIT(0));

	return 0;
}

static int flash_hi3861_erase(const struct device *dev, off_t offset, size_t size)
{
	ARG_UNUSED(dev);

	return 0;
}

static const struct flash_parameters *flash_hi3861_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	static const struct flash_parameters flash_hi3861_parameters = {
		.write_block_size = 1,
		.erase_value = 0xff,
	};

	return &flash_hi3861_parameters;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_hi3861_page_layout(const struct device *dev,
				     const struct flash_pages_layout **layout, size_t *layout_size)
{
	ARG_UNUSED(dev);

	static const struct flash_pages_layout flash_hi3861_pages_layout = {
		.pages_count = (2 * 1024 * 1024) / 64,
		.pages_size = 64,
	};

	*layout = &flash_hi3861_pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_hi3861_sfdp_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	ARG_UNUSED(dev);

	return 0;
}
#endif /* CONFIG_FLASH_JESD216_API */

static int flash_hi3861_read_jedec_id(const struct device *dev, uint8_t *id)
{
	ARG_UNUSED(dev);

	return flash_hi3861_cmd_read(SPI_NOR_CMD_RDID, id, SPI_NOR_MAX_ID_LEN);
}

static int flash_hi3861_init(const struct device *dev)
{
	uint32_t regval;
	uint32_t jedec_id;
	const struct flash_clock_voltage_config *cfg = &flash_clock_voltage_configs[0];

	flash_hi3861_read_jedec_id(dev, (uint8_t *)&jedec_id);
	LOG_DBG("JEDEC ID: %06x", jedec_id);

	syscon_read_reg(pmu_cmu_ctl, PMU_CMU_CTL_CLK_192M_GT_OFFSET, &regval);
	regval &= ~BIT(0);
	syscon_write_reg(pmu_cmu_ctl, PMU_CMU_CTL_CLK_192M_GT_OFFSET, regval);

	syscon_read_reg(cldo_ctl, CLDO_CTL_CLK_SEL_OFFSET, &regval);
	regval |= BIT(1);
	syscon_write_reg(cldo_ctl, CLDO_CTL_CLK_SEL_OFFSET, regval);

	syscon_read_reg(pmu_cmu_ctl, PMU_CMU_CTL_CMU_CLK_SEL_OFFSET, &regval);
	regval &= ~GENMASK(6, 4);
	syscon_write_reg(pmu_cmu_ctl, PMU_CMU_CTL_CMU_CLK_SEL_OFFSET, regval);

	syscon_read_reg(cldo_ctl, CLDO_CTL_CLK_DIV1_OFFSET, &regval);
	regval &= ~GENMASK(6, 4);
	syscon_write_reg(cldo_ctl, CLDO_CTL_CLK_DIV1_OFFSET, regval);

	/* Enable flash LDO bypass */
	syscon_read_reg(pmu_cmu_ctl, PMU_CMU_CTL_FLASHLDO_CFG_1_OFFSET, &regval);
	regval |= BIT(6);
	syscon_write_reg(pmu_cmu_ctl, PMU_CMU_CTL_FLASHLDO_CFG_1_OFFSET, regval);

	/* Find best match config */
	for (int i = 0; i < ARRAY_SIZE(flash_clock_voltage_configs); i++) {
		if (flash_clock_voltage_configs[i].jedec_id == jedec_id) {
			cfg = &flash_clock_voltage_configs[i];
			break;
		}
	}

	LOG_DBG("Using config: freq=%d vol=%d", cfg->freq, cfg->voltage);

	/* Update clock config */
	syscon_read_reg(pmu_cmu_ctl, PMU_CMU_CTL_CMU_CLK_SEL_OFFSET, &regval);
	regval &= ~PLL2DBB_192M_MASK;
	regval |= FIELD_PREP(PLL2DBB_192M_MASK, cfg->freq);
	syscon_write_reg(pmu_cmu_ctl, PMU_CMU_CTL_CMU_CLK_SEL_OFFSET, regval);

	sys_write32(0x1, SFC_TIMING_REG);

	return 0;
}

static const struct flash_driver_api flash_hi3861_api = {
	.read = flash_hi3861_read,
	.write = flash_hi3861_write,
	.erase = flash_hi3861_erase,
	.get_parameters = flash_hi3861_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_hi3861_page_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = flash_hi3861_sfdp_read,
	.read_jedec_id = flash_hi3861_read_jedec_id,
#endif /* CONFIG_FLASH_JESD216_API */
};

DEVICE_DT_INST_DEFINE(0, flash_hi3861_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_FLASH_INIT_PRIORITY, &flash_hi3861_api);
